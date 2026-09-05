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

#include "istddef.h"
#include "coredev/interrupt.h"
#include "usb/usb_intrf.h"


static int UsbIntrfEpSendByteMode(UsbDevIntrf_t *pIntrf);
static int UsbIntrfEpSendPktMode(UsbDevIntrf_t *pIntrf);
static void UsbIntrfRxXferComplete(UsbDevIntrf_t *pIntrf, uint16_t Length,
								   UsbCtrlrXferResult_t Result);
static void UsbIntrfTxXferComplete(UsbDevIntrf_t *pIntrf, uint16_t Length,
								   UsbCtrlrXferResult_t Result);

static inline __attribute__((always_inline))
bool UsbIntrfEnabled(const UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr &&
		atomic_load_explicit(&pIntrf->DevIntrf.EnCnt,
							 memory_order_relaxed) > 0;
}

static void UsbIntrfRxArm(UsbDevIntrf_t *pIntrf)
{
	if (!UsbIntrfEnabled(pIntrf) || pIntrf->Mps == 0U ||
		pIntrf->hRxFifo == nullptr || pIntrf->pRxBuffer == nullptr ||
		CFifoAvail(pIntrf->hRxFifo) <= 0)
	{
		return;
	}

	(void)UsbCtrlrEpRxArm(pIntrf->DevNo, pIntrf->EpNo);
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

static void UsbIntrfTxFailure(UsbDevIntrf_t *pIntrf, uint16_t Length)
{
	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_TIMEOUT,
							   nullptr, Length);
	}
}

static int UsbIntrfEpSendByteMode(UsbDevIntrf_t *pIntrf)
{
	int cnt = 0;
	int length = pIntrf->Mps;
	uint8_t *buff = pIntrf->pTxBuffer;
	uint32_t state = DisableInterrupt();

	while (length > 0)
	{
		int l = length;

		uint8_t *p = CFifoGetMultiple(pIntrf->hTxFifo, &l);

		if (p == nullptr)
		{
			break;
		}

		memcpy(buff, p, l);

		length -= l;
		buff += l;
		cnt += l;
	}

	EnableInterrupt(state);

	// Nothing left in the FIFO is the one thing that releases the flag. While
	// it is held the producer only queues, so the FIFO accumulates for the
	// next submission.
	if (cnt <= 0)
	{
		UsbIntrfSetTxIdle(pIntrf);

		return -1;
	}


	if (UsbCtrlrEpSend(pIntrf->DevNo, pIntrf->EpNo, (uint16_t)cnt) == false)
	{
		// A refusal by the controller is a fault, not an idle transmit. The packet
		// is still staged in the buffer, so releasing the token here would let the
		// next submission overwrite it. Report and hold until configure or
		// unconfigure puts the endpoint back to a known state.
		UsbIntrfTxFailure(pIntrf, (uint16_t)cnt);
	}

	return cnt;
}

static int UsbIntrfEpSendPktMode(UsbDevIntrf_t *pIntrf)
{
	UsbPkt_t *pkt = reinterpret_cast<UsbPkt_t *>(CFifoPeek(pIntrf->hTxFifo));

	if (pkt == nullptr)
	{
		UsbIntrfSetTxIdle(pIntrf);
		return -1;
	}

	int cnt = pkt->Hdr.Length;
	if (cnt > 0)
	{
		memcpy(pIntrf->pTxBuffer, pkt->Data, cnt);
	}
	(void)CFifoGet(pIntrf->hTxFifo);

	if (UsbCtrlrEpSend(pIntrf->DevNo, pIntrf->EpNo, (uint16_t)cnt) == false)
	{
		// A refusal by the controller is a fault, not an idle transmit. The packet
		// is still staged in the buffer, so releasing the token here would let the
		// next submission overwrite it. Report and hold until configure or
		// unconfigure puts the endpoint back to a known state.
		UsbIntrfTxFailure(pIntrf, (uint16_t)cnt);
	}

	return cnt;
}


static void UsbIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void UsbIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);

	UsbIntrfRxArm(pIntrf);
//	(void)UsbIntrfStartXfer(pIntrf);
}

/**
 * Link rate in bits per second, zero while the endpoint is not configured.
 * The host owns the bus, so this reports what enumeration settled on rather
 * than anything the device picked.
 */
static uint32_t UsbIntrfGetRate(DevIntrf_t * const pDevIntrf)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);


	if (pIntrf == nullptr || pIntrf->Mps == 0U)
	{
		return 0;
	}

	return UsbCtrlrHighSpeed(pIntrf->DevNo) ?
		   USB_LINK_RATE_HIGH : USB_LINK_RATE_FULL;
}

/**
 * The rate cannot be set. Speed is fixed by the reset handshake, so the
 * closest match to any request is the rate already running.
 */
static uint32_t UsbIntrfSetRate(DevIntrf_t * const pDevIntrf, uint32_t)
{
	return UsbIntrfGetRate(pDevIntrf);
}

static bool UsbIntrfStartRx(DevIntrf_t * const, uint32_t)
{
	return true;
}

/** Consume only complete stored packets, combining packets when they fit. */
static int UsbIntrfRxData(DevIntrf_t * const pDevIntrf, uint8_t *pBuffer, int BufferLen)
{
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);

	if (pIntrf == nullptr || pIntrf->hRxFifo == nullptr ||
		pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	int cnt = 0;
	bool released = false;

	while (BufferLen > 0)
	{
		UsbPkt_t *pkt = reinterpret_cast<UsbPkt_t *>(CFifoPeek(pIntrf->hRxFifo));

		if (pkt == nullptr)
		{
			break;
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

	// OUT may have stopped because the FIFO became full after foreground
	// entered this function. Retry whenever storage was released; the
	// controller owns the active-transfer state and rejects a duplicate arm.
	if (released)
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
 * Packet mode write. Each CFifo block is one USB packet, so the caller passes
 * whole blocks and UsbIntrf never combines them.
 */
static int UsbIntrfTxPackets(DevIntrf_t * const pDevIntrf,
							 const uint8_t *pData, int DataLen)
{
	if (pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);

	const int blockSize = (int)CFifoBlockSize(pIntrf->hTxFifo);

	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (DataLen >= blockSize)
	{
		uint8_t *p = CFifoPut(pIntrf->hTxFifo);

		if (p == nullptr)
		{
			break;
		}

		memcpy(p, pData, (size_t)blockSize);
		pData += blockSize;
		DataLen -= blockSize;
		cnt += blockSize;
	}

	EnableInterrupt(state);

	if (UsbIntrfTakeTx(pIntrf))
	{
		UsbIntrfEpSendPktMode(pIntrf);
	}

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
	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData);

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

	if (UsbIntrfTakeTx(pIntrf))
	{
		(void)UsbIntrfEpSendByteMode(pIntrf);
	}

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
	UsbIntrfUnconfigure(static_cast<UsbDevIntrf_t *>(pDevIntrf->pDevData));
}

static void UsbIntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	UsbIntrfDisable(pDevIntrf);
}

static void *UsbIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return pDevIntrf->pDevData;
}

static void UsbIntrfCtrlrRx(uint8_t, uint16_t Length,
							UsbCtrlrXferResult_t Result, void *pContext)
{
	UsbIntrfRxXferComplete(static_cast<UsbDevIntrf_t *>(pContext), Length,
						  Result);
}

static void UsbIntrfCtrlrTx(uint8_t, uint16_t Length,
							UsbCtrlrXferResult_t Result, void *pContext)
{
	UsbIntrfTxXferComplete(static_cast<UsbDevIntrf_t *>(pContext), Length,
						  Result);
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
	if (pCfg->TxFifoBlkSize == 1)
	{
		pIntrf->DevIntrf.TxData = UsbIntrfTxBytes;
		pIntrf->EpSend = UsbIntrfEpSendByteMode;
	}
	else
	{
		pIntrf->DevIntrf.TxData = UsbIntrfTxPackets;
		pIntrf->EpSend = UsbIntrfEpSendPktMode;
	}
	pIntrf->DevIntrf.TxSrData = UsbIntrfTxSrData;
	pIntrf->DevIntrf.StopTx = UsbIntrfStopTx;
	pIntrf->DevIntrf.Reset = UsbIntrfReset;
	pIntrf->DevIntrf.PowerOff = UsbIntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbIntrfGetHandle;

	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	if (!UsbCtrlrEpRegister(pIntrf->DevNo,
		USB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->pRxBuffer,
		UsbIntrfCtrlrRx, pIntrf) ||
		!UsbCtrlrEpRegister(pIntrf->DevNo,
		USB_ENDPADDR_DIRIN(pIntrf->EpNo), pIntrf->pTxBuffer,
		UsbIntrfCtrlrTx, pIntrf))
	{
		return false;
	}

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
				(void)UsbCtrlrEpRxArm(pIntrf->DevNo, pIntrf->EpNo);
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

	if (pIntrf->EpSend(pIntrf) >= 0)
	{
		return;
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

	return CFifoAvail(pIntrf->hTxFifo) >= blocks;
}

int UsbIntrfTxUsed(UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hTxFifo != nullptr ?
		   CFifoUsed(pIntrf->hTxFifo) : 0;
}
