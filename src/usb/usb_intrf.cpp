/**-------------------------------------------------------------------------
@file	usb_intrf.cpp

@brief	Generic USB device data interface implementation.

@author	Hoang Nguyen Hoan
@date	Sep. 1, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <string.h>

#include "coredev/interrupt.h"
#include "usb/usb_intrf.h"


static UsbDevIntrf_t *UsbIntrfData(DevIntrf_t * const pDevIntrf)
{
	if (pDevIntrf == nullptr || pDevIntrf->pDevData == nullptr)
	{
		return nullptr;
	}

	return static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);
}

// Producer path accessor. pDevData is set once in UsbIntrfInit and never
// cleared, so a handler reached through DevIntrf.TxData always has it. The
// checked form above stays for the public entry points; on the byte path it
// would cost a call and two compares per queued byte.
static inline __attribute__((always_inline))
UsbDevIntrf_t *UsbIntrfDataUnchecked(DevIntrf_t * const pDevIntrf)
{
	return static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);
}

static uint8_t *UsbIntrfPktData(UsbPktHdr_t *pPacket)
{
	return reinterpret_cast<uint8_t *>(pPacket + 1);
}

static inline __attribute__((always_inline))
bool UsbIntrfEnabled(const UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr &&
		atomic_load_explicit(&pIntrf->DevIntrf.EnCnt,
							 memory_order_relaxed) > 0;
}

static uint8_t UsbIntrfRxAddr(const UsbDevIntrf_t *pIntrf)
{
	return USB_ENDPADDR_DIROUT(pIntrf->EpNo);
}

static inline __attribute__((always_inline))
uint8_t UsbIntrfTxAddr(const UsbDevIntrf_t *pIntrf)
{
	return USB_ENDPADDR_DIRIN(pIntrf->EpNo);
}

static void UsbIntrfRxSubmit(UsbDevIntrf_t *pIntrf)
{
	(void)UsbCtrlrEpXfer(pIntrf->DevNo, UsbIntrfRxAddr(pIntrf),
						 pIntrf->pRxBuffer, pIntrf->Mps);
}

static void UsbIntrfRxArm(UsbDevIntrf_t *pIntrf)
{
	if (!UsbIntrfEnabled(pIntrf) || pIntrf->Mps == 0U ||
		pIntrf->hRxFifo == nullptr || pIntrf->pRxBuffer == nullptr ||
		CFifoAvail(pIntrf->hRxFifo) <= 0)
	{
		return;
	}

	UsbIntrfRxSubmit(pIntrf);
}

static inline __attribute__((always_inline))
bool UsbIntrfCanTx(UsbDevIntrf_t *pIntrf)
{
	return UsbIntrfEnabled(pIntrf) && pIntrf->Mps > 0U &&
		   pIntrf->pTxBuffer != nullptr;
}

static inline __attribute__((always_inline))
void UsbIntrfSetTxIdle(UsbDevIntrf_t *pIntrf)
{
	atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
						  memory_order_release);
}

static inline __attribute__((always_inline))
bool UsbIntrfTakeTx(UsbDevIntrf_t *pIntrf)
{
	return atomic_exchange_explicit(&pIntrf->DevIntrf.bTxReady, false,
									memory_order_acquire);
}

/**
 * True while a transfer owns the token. The producer runs this per queued
 * byte, so it is a relaxed load: the byte is already in the FIFO and the
 * completion drains whatever is there, so a stale false only costs one
 * rejected claim and a stale true costs nothing.
 *
 * An earlier version read the flag through a bool pointer to skip the out of
 * line template at -O0. Equal size and lock free do not make that alias legal,
 * so it is a normal atomic load.
 */
static inline __attribute__((always_inline))
bool UsbIntrfTxHeld(const UsbDevIntrf_t *pIntrf)
{
	return !atomic_load_explicit(&pIntrf->DevIntrf.bTxReady,
								 memory_order_relaxed);
}

static void UsbIntrfTxFailure(UsbDevIntrf_t *pIntrf, uint16_t Length)
{
	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_TIMEOUT,
							   nullptr, Length);
	}
}

/** Packet mode fill. One CFifo block is one USB packet. */
static int UsbIntrfFillOnePacket(UsbDevIntrf_t *pIntrf)
{
	UsbPktHdr_t *pPacket = reinterpret_cast<UsbPktHdr_t *>(
		CFifoPeek(pIntrf->hTxFifo));
	if (pPacket == nullptr || pPacket->Length > pIntrf->Mps)
	{
		if (pPacket != nullptr)
		{
			(void)CFifoGet(pIntrf->hTxFifo);
		}
		return -1;
	}

	const uint16_t length = pPacket->Length;
	if (length > 0U)
	{
		memcpy(pIntrf->pTxBuffer, UsbIntrfPktData(pPacket), length);
	}
	(void)CFifoGet(pIntrf->hTxFifo);
	return length;
}

/** Byte mode fill. CFifoGetMultiple returns one contiguous run up to MPS. */
static int UsbIntrfFillBytes(UsbDevIntrf_t *pIntrf)
{
	uint8_t *pBuffer = pIntrf->pTxBuffer;
	int remain = (int)pIntrf->Mps;
	int cnt = 0;

	// Gather across the ring wrap. One CFifoGetMultiple returns only the
	// contiguous run, so a wrap would otherwise cut a full packet short.
	while (remain > 0)
	{
		int length = remain;
		uint8_t *p = CFifoGetMultiple(pIntrf->hTxFifo, &length);
		if (p == nullptr || length <= 0)
		{
			break;
		}

		memcpy(pBuffer + cnt, p, (size_t)length);
		cnt += length;
		remain -= length;
	}

	return cnt;
}

static bool UsbIntrfSubmit(UsbDevIntrf_t *pIntrf)
{
	const bool packetMode = CFifoBlockSize(pIntrf->hTxFifo) != 1U;
	const int length = packetMode ? UsbIntrfFillOnePacket(pIntrf) :
								  UsbIntrfFillBytes(pIntrf);

	// Nothing left in the FIFO is the one thing that releases the flag. While
	// it is held the producer only queues, so the FIFO accumulates for the
	// next submission.
	if (length < 0 || (length == 0 && !packetMode))
	{
		UsbIntrfSetTxIdle(pIntrf);
		return false;
	}


	if (UsbCtrlrEpXfer(pIntrf->DevNo, UsbIntrfTxAddr(pIntrf), pIntrf->pTxBuffer,
						(uint16_t)length))
	{
		return true;
	}

	// A refusal by the controller is a fault, not an idle transmit. The packet
	// is still staged in the buffer, so releasing the token here would let the
	// next submission overwrite it. Report and hold until configure or
	// unconfigure puts the endpoint back to a known state.
	if (length > 0)
	{
		UsbIntrfTxFailure(pIntrf, (uint16_t)length);
	}

	return false;
}

static inline __attribute__((always_inline))
bool UsbIntrfStartXfer(UsbDevIntrf_t *pIntrf)
{
	// Busy is the common producer path. Test ownership first so it returns
	// without loading enable/configuration state on every queued write.
	if (!UsbIntrfTakeTx(pIntrf))
	{
		return false;
	}
//	if (!UsbIntrfCanTx(pIntrf))
//	{
//		UsbIntrfSetTxIdle(pIntrf);
//		return false;
//	}

	return UsbIntrfSubmit(pIntrf);
}

static void UsbIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void UsbIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	UsbIntrfRxArm(pIntrf);
	(void)UsbIntrfStartXfer(pIntrf);
}

static uint32_t UsbIntrfGetRate(DevIntrf_t * const)
{
	return 0;
}

static uint32_t UsbIntrfSetRate(DevIntrf_t * const, uint32_t)
{
	return 0;
}

static bool UsbIntrfStartRx(DevIntrf_t * const, uint32_t)
{
	return true;
}

/** Consume only complete stored packets, combining packets when they fit. */
static int UsbIntrfRxData(DevIntrf_t * const pDevIntrf, uint8_t *pBuffer, int BufferLen)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pIntrf->hRxFifo == nullptr ||
		pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	int cnt = 0;
	bool checkedFull = false;
	bool wasFull = false;
	bool released = false;

	while (BufferLen > 0)
	{
		UsbPkt_t *pkt = reinterpret_cast<UsbPkt_t *>(CFifoPeek(pIntrf->hRxFifo));

		if (pkt == nullptr)
		{
			break;
		}
		if (!checkedFull)
		{
			wasFull = CFifoAvail(pIntrf->hRxFifo) <= 0;
			checkedFull = true;
		}

		const uint16_t len = pkt->Hdr.Length;

		if (len > pIntrf->Mps)
		{
			(void)CFifoGet(pIntrf->hRxFifo);
			pIntrf->RxDropCnt++;
			released = true;
			continue;
		}

		if (BufferLen < (int)len)
		{
			break;
		}

		if (len > 0U)
		{
			memcpy(pBuffer, pkt->Data, len);
		}

		(void)CFifoGet(pIntrf->hRxFifo);
		released = true;

		pBuffer += len;
		BufferLen -= len;
		cnt += len;
	}

	if (wasFull && released)
	{
		UsbIntrfRxArm(pIntrf);
	}

	return cnt;
}

static void UsbIntrfStopRx(DevIntrf_t * const)
{
}

static bool UsbIntrfStartTx(DevIntrf_t * const, uint32_t)
{
	return true;
}

/**
 * Queue and activate. The flag is the whole decision: while a transfer holds
 * it the producer only appends, so the FIFO accumulates until the completion
 * comes back for it.
 */
static inline __attribute__((always_inline))
void UsbIntrfTxQueued(UsbDevIntrf_t *pIntrf, int Cnt)
{
	// Cnt zero means the FIFO is full, which can only happen while a transfer
	// is running, so there is nothing to start. A failed transfer does not
	// release the token, so an idle transmit never has a full FIFO.
	if (Cnt <= 0 || pIntrf->Mps == 0U)
	{
		return;
	}

	// A transfer in flight is the common case. Filter on a plain read so the
	// read modify write is only paid when the token looks free.
	if (UsbIntrfTxHeld(pIntrf))
	{
		return;
	}

	(void)UsbIntrfStartXfer(pIntrf);
}

/**
 * Packet mode write. Each CFifo block is one USB packet, so the caller passes
 * whole blocks and UsbIntrf never combines them.
 */
static int UsbIntrfTxPackets(DevIntrf_t * const pDevIntrf,
							 const uint8_t *pData, int DataLen)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	const uint32_t blockSize = CFifoBlockSize(pIntrf->hTxFifo);

	if (blockSize != sizeof(UsbPktHdr_t) + pIntrf->Mps ||
		(DataLen % (int)blockSize) != 0)
	{
		return 0;
	}

	for (int offset = 0; offset < DataLen; offset += (int)blockSize)
	{
		const UsbPktHdr_t *pPacket =
			reinterpret_cast<const UsbPktHdr_t *>(pData + offset);
		if (pPacket->Length > pIntrf->Mps)
		{
			return 0;
		}
	}

	// CFifoPutMultiple publishes its producer index before memcpy fills the
	// returned area. Keep the USB completion consumer out until publication is
	// complete.
	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (DataLen > 0)
	{
		int blocks = DataLen / (int)blockSize;
		uint8_t *p = CFifoPutMultiple(pIntrf->hTxFifo, &blocks);
		if (p == nullptr || blocks <= 0)
		{
			break;
		}

		const int length = blocks * (int)blockSize;
		memcpy(p, pData, (size_t)length);
		pData += length;
		DataLen -= length;
		cnt += length;
	}

	EnableInterrupt(state);

	//UsbIntrfTxQueued(pIntrf, cnt);
	(void)UsbIntrfStartXfer(pIntrf);

	return cnt;
}

/**
 * Byte mode write, the CDC path. UsbIntrf packetizes queued bytes up to MPS.
 *
 * This runs once per DeviceIntrfTx call, so a one byte write walks all of it.
 * Packet mode is a separate DevIntrf handler chosen at init rather than a test
 * here, because that test would be paid on every byte.
 */
static int UsbIntrfTxBytes(DevIntrf_t * const pDevIntrf,
						   const uint8_t *pData, int DataLen)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfDataUnchecked(pDevIntrf);

	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (DataLen > 0)
	{
		int length = DataLen;
		uint8_t *p = CFifoPutMultiple(pIntrf->hTxFifo, &length);
		if (p == nullptr || length <= 0)
		{
			break;
		}

		// One byte per call is the CDC stream case, and memcpy is an out of
		// line call for it. Store it directly instead.
		if (length == 1)
		{
			*p = *pData;
		}
		else
		{
			memcpy(p, pData, (size_t)length);
		}

		pData += length;
		DataLen -= length;
		cnt += length;
	}

	EnableInterrupt(state);

	(void)UsbIntrfStartXfer(pIntrf);

//	UsbIntrfTxQueued(pIntrf, cnt);

	return cnt;
}

static int UsbIntrfTxSrData(DevIntrf_t * const pDevIntrf,
							 const uint8_t *pData, int DataLen)
{
	return pDevIntrf->TxData(pDevIntrf, pData, DataLen);
}

static void UsbIntrfStopTx(DevIntrf_t * const)
{
	// DeviceIntrf StopTx only ends the application-side operation. USB may
	// still be transmitting and continues to drain the CFifo independently.
}

static void UsbIntrfReset(DevIntrf_t * const pDevIntrf)
{
	UsbIntrfUnconfigure(UsbIntrfData(pDevIntrf));
}

static void UsbIntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	UsbIntrfDisable(pDevIntrf);
}

static void *UsbIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return UsbIntrfData(pDevIntrf);
}

bool UsbIntrfInit(UsbDevIntrf_t *pIntrf, const UsbIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		pCfg->EpNo == 0U || pCfg->EpNo > USB_ENDPADDR_NUM_MASK ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0 ||
		pCfg->pRxBuffer == nullptr || pCfg->pTxBuffer == nullptr ||
		pCfg->BufferSize == 0U || pCfg->TxFifoBlkSize == 0U ||
		(((uintptr_t)pCfg->pRxFifoMem & 3U) != 0U) ||
		(((uintptr_t)pCfg->pTxFifoMem & 3U) != 0U) ||
		(((uintptr_t)pCfg->pRxBuffer & 3U) != 0U) ||
		(((uintptr_t)pCfg->pTxBuffer & 3U) != 0U))
	{
		return false;
	}

	pIntrf->hTxFifo = CFifoInit(pCfg->pTxFifoMem,
								 (uint32_t)pCfg->TxFifoMemSize,
								 pCfg->TxFifoBlkSize, pCfg->bBlocking);
	pIntrf->hRxFifo = CFifoInit(pCfg->pRxFifoMem,
								 (uint32_t)pCfg->RxFifoMemSize,
								 USB_INTRF_PKT_BLKSIZE(pCfg->BufferSize), true);
	if (pIntrf->hTxFifo == nullptr || pIntrf->hRxFifo == nullptr)
	{
		return false;
	}

	pIntrf->DevNo = pCfg->DevNo;
	pIntrf->EpNo = pCfg->EpNo;
	pIntrf->pRxBuffer = pCfg->pRxBuffer;
	pIntrf->pTxBuffer = pCfg->pTxBuffer;
	pIntrf->BufferSize = pCfg->BufferSize;
	pIntrf->Mps = 0U;
	pIntrf->RxDropCnt = 0U;

	pIntrf->DevIntrf.pDevData = pIntrf;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = true;
	pIntrf->DevIntrf.bIntEn = true;
	pIntrf->DevIntrf.Disable = UsbIntrfDisable;
	pIntrf->DevIntrf.Enable = UsbIntrfEnable;
	pIntrf->DevIntrf.GetRate = UsbIntrfGetRate;
	pIntrf->DevIntrf.SetRate = UsbIntrfSetRate;
	pIntrf->DevIntrf.StartRx = UsbIntrfStartRx;
	pIntrf->DevIntrf.RxData = UsbIntrfRxData;
	pIntrf->DevIntrf.StopRx = UsbIntrfStopRx;
	pIntrf->DevIntrf.StartTx = UsbIntrfStartTx;
	// Chosen once. Byte mode is the CDC hot path and must not pay a mode test
	// on every call.
	pIntrf->DevIntrf.TxData = pCfg->TxFifoBlkSize == 1U ?
		UsbIntrfTxBytes : UsbIntrfTxPackets;
	pIntrf->DevIntrf.TxSrData = UsbIntrfTxSrData;
	pIntrf->DevIntrf.StopTx = UsbIntrfStopTx;
	pIntrf->DevIntrf.Reset = UsbIntrfReset;
	pIntrf->DevIntrf.PowerOff = UsbIntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbIntrfGetHandle;

	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	DeviceIntrfEnable(&pIntrf->DevIntrf);

	return true;
}

bool UsbIntrfConfigure(UsbDevIntrf_t *pIntrf, uint16_t Mps)
{
	if (pIntrf == nullptr || Mps == 0U || Mps > pIntrf->BufferSize ||
		pIntrf->hRxFifo == nullptr ||
		(CFifoBlockSize(pIntrf->hTxFifo) != 1U &&
		 CFifoBlockSize(pIntrf->hTxFifo) != sizeof(UsbPktHdr_t) + Mps))
	{
		return false;
	}


	pIntrf->Mps = Mps;
	CFifoFlush(pIntrf->hRxFifo);
	CFifoFlush(pIntrf->hTxFifo);
	UsbIntrfSetTxIdle(pIntrf);
	UsbIntrfRxArm(pIntrf);

	return true;
}

void UsbIntrfUnconfigure(UsbDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	// Clear MPS first so a completion caused by closing the endpoint cannot
	// submit another transfer.
	pIntrf->Mps = 0U;

	if (pIntrf->hRxFifo != nullptr)
	{
		CFifoFlush(pIntrf->hRxFifo);
	}
	CFifoFlush(pIntrf->hTxFifo);
	UsbIntrfSetTxIdle(pIntrf);
}

static void UsbIntrfRxXferComplete(UsbDevIntrf_t *pIntrf, uint16_t Length,
								UsbCtrlrXferResult_t Result)
{
	if (Result == USB_CTRLR_XFER_SUCCESS && pIntrf->Mps > 0U &&
		Length <= pIntrf->Mps &&
		UsbIntrfEnabled(pIntrf) && pIntrf->hRxFifo != nullptr)
	{
		UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(
			CFifoPut(pIntrf->hRxFifo));
		if (pPacket != nullptr)
		{
			pPacket->Hdr.Length = Length;
			pPacket->Hdr.Reserved = 0U;
			if (Length > 0U)
			{
				memcpy(pPacket->Data, pIntrf->pRxBuffer, Length);
			}

			const int used = CFifoUsed(pIntrf->hRxFifo);
			if (used < pIntrf->hRxFifo->MaxIdxCnt)
			{
				UsbIntrfRxSubmit(pIntrf);
			}

			if (used >= pIntrf->hRxFifo->MaxIdxCnt &&
				pIntrf->DevIntrf.EvtCB != nullptr)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
									   DEVINTRF_EVT_RX_FIFO_FULL,
									   nullptr, used);
			}

			if (pIntrf->DevIntrf.EvtCB != nullptr)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
									   DEVINTRF_EVT_RX_DATA,
									   nullptr, used);
			}
			return;
		}
		else
		{
			pIntrf->RxDropCnt++;
		}
	}
	else if (Result == USB_CTRLR_XFER_FAILED)
	{
		pIntrf->RxDropCnt++;
		if (pIntrf->DevIntrf.EvtCB != nullptr)
		{
			pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
								   DEVINTRF_EVT_RX_TIMEOUT,
								   nullptr, Length);
		}
	}

	UsbIntrfRxArm(pIntrf);
}

static void UsbIntrfTxXferComplete(UsbDevIntrf_t *pIntrf,
								uint16_t Length, UsbCtrlrXferResult_t Result)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr)
	{
		return;
	}


	if (Result != USB_CTRLR_XFER_SUCCESS)
	{
		// Same as a refused submission: a fault holds the token. Configure and
		// unconfigure are what release it again.
		if (Result == USB_CTRLR_XFER_FAILED)
		{
			UsbIntrfTxFailure(pIntrf, Length);
		}
		return;
	}

	// Refill the DMA buffer and go again without touching the flag. Only an
	// empty FIFO inside UsbIntrfSubmit releases it.
	if (UsbIntrfCanTx(pIntrf))
	{
		if (UsbIntrfSubmit(pIntrf))
		{
			return;
		}
	}
	else
	{
		UsbIntrfSetTxIdle(pIntrf);
	}

	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_FIFO_EMPTY,
							   nullptr, 0);
	}
}

void UsbIntrfXferComplete(UsbDevIntrf_t *pIntrf, uint8_t EpAddr,
						  uint16_t Length, UsbCtrlrXferResult_t Result)
{
	if (pIntrf == nullptr || USB_ENDPADDR_NUM(EpAddr) != pIntrf->EpNo)
	{
		return;
	}

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		UsbIntrfTxXferComplete(pIntrf, Length, Result);
	}
	else
	{
		UsbIntrfRxXferComplete(pIntrf, Length, Result);
	}
}

bool UsbIntrfRequestToSend(UsbDevIntrf_t *pIntrf, int NbBytes)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr || NbBytes <= 0)
	{
		return false;
	}

	if (pIntrf->hTxFifo->bBlocking == false)
	{
		return true;
	}
	const uint32_t blockSize = CFifoBlockSize(pIntrf->hTxFifo);
	int blocks = NbBytes;
	if (blockSize != 1U)
	{
		if ((NbBytes % (int)blockSize) != 0)
		{
			return false;
		}
		blocks = NbBytes / (int)blockSize;
	}

	if (CFifoAvail(pIntrf->hTxFifo) < blocks)
	{
		(void)UsbIntrfStartXfer(pIntrf);
	}

	return CFifoAvail(pIntrf->hTxFifo) >= blocks;
}

int UsbIntrfTxUsed(UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hTxFifo != nullptr ?
		   CFifoUsed(pIntrf->hTxFifo) : 0;
}
