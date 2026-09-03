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

static uint8_t *UsbIntrfPktData(UsbPktHdr_t *pPacket)
{
	return reinterpret_cast<uint8_t *>(pPacket + 1);
}

static bool UsbIntrfEnabled(const UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr &&
		atomic_load_explicit(&pIntrf->DevIntrf.EnCnt,
							 memory_order_relaxed) > 0;
}

static uint8_t UsbIntrfRxAddr(const UsbDevIntrf_t *pIntrf)
{
	return USB_ENDPADDR_DIROUT(pIntrf->EpNo);
}

static uint8_t UsbIntrfTxAddr(const UsbDevIntrf_t *pIntrf)
{
	return USB_ENDPADDR_DIRIN(pIntrf->EpNo);
}

static void UsbIntrfRxArm(UsbDevIntrf_t *pIntrf)
{
	if (!UsbIntrfEnabled(pIntrf) || pIntrf->Mps == 0U ||
		pIntrf->hRxFifo == nullptr || pIntrf->pRxBuffer == nullptr ||
		CFifoAvail(pIntrf->hRxFifo) <= 0)
	{
		return;
	}

	const uint8_t epAddr = UsbIntrfRxAddr(pIntrf);
	if (!UsbCtrlrEpBusy(pIntrf->DevNo, epAddr))
	{
		(void)UsbCtrlrEpXfer(pIntrf->DevNo, epAddr,
							 pIntrf->pRxBuffer, pIntrf->Mps);
	}
}

static bool UsbIntrfCanTx(UsbDevIntrf_t *pIntrf)
{
	return UsbIntrfEnabled(pIntrf) && pIntrf->Mps > 0U &&
		   pIntrf->pTxBuffer != nullptr;
}

static void UsbIntrfSetTxIdle(UsbDevIntrf_t *pIntrf)
{
	atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
						  memory_order_release);
}

static bool UsbIntrfTakeTx(UsbDevIntrf_t *pIntrf)
{
	return atomic_exchange_explicit(&pIntrf->DevIntrf.bTxReady, false,
									memory_order_acquire);
}

static bool UsbIntrfTxIdle(UsbDevIntrf_t *pIntrf)
{
	return atomic_load_explicit(&pIntrf->DevIntrf.bTxReady,
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

static int UsbIntrfFillPacket(UsbDevIntrf_t *pIntrf)
{
	if (pIntrf->TxBlkSize != 1U)
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

	uint8_t *pBuffer = pIntrf->pTxBuffer;
	int remain = (int)pIntrf->Mps;
	int cnt = 0;

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

static bool UsbIntrfSubmit(UsbDevIntrf_t *pIntrf, bool Tail)
{
	const int used = CFifoUsed(pIntrf->hTxFifo);
	int length = 0;
	const bool packetMode = pIntrf->TxBlkSize != 1U;

	if ((packetMode && used > 0) ||
		(!packetMode && (used >= (int)pIntrf->Mps || (Tail && used > 0))))
	{
		length = UsbIntrfFillPacket(pIntrf);
	}

	if (length < 0)
	{
		UsbIntrfSetTxIdle(pIntrf);
		return false;
	}

	if (length == 0 && !(packetMode && used > 0))
	{
		UsbIntrfSetTxIdle(pIntrf);
		return false;
	}

	pIntrf->TxTailArmed = false;

	// What the producer had queued when this packet was filled. Comparing it
	// at completion says whether the producer kept running during the packet.
	pIntrf->TxUsedMark = CFifoUsed(pIntrf->hTxFifo);

	if (UsbCtrlrEpXfer(pIntrf->DevNo, UsbIntrfTxAddr(pIntrf), pIntrf->pTxBuffer,
						(uint16_t)length))
	{
		return true;
	}

	UsbIntrfSetTxIdle(pIntrf);
	if (length > 0)
	{
		UsbIntrfTxFailure(pIntrf, (uint16_t)length);
	}

	return false;
}

static bool UsbIntrfStartXfer(UsbDevIntrf_t *pIntrf, bool Tail)
{
	// USB transaction state belongs to the consumer. The producer can keep
	// filling the CFifo while this endpoint is busy.
	if (!UsbIntrfTxIdle(pIntrf) || !UsbIntrfCanTx(pIntrf) ||
		!UsbIntrfTakeTx(pIntrf))
	{
		return false;
	}

	return UsbIntrfSubmit(pIntrf, Tail);
}

static void UsbIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void UsbIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	if (pIntrf == nullptr)
	{
		return;
	}

	UsbIntrfRxArm(pIntrf);
	(void)UsbIntrfStartXfer(pIntrf, false);
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

		pBuffer += len;
		BufferLen -= len;
		cnt += len;
	}

	UsbIntrfRxArm(pIntrf);

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
 * whole blocks and every stored length is checked before anything is queued.
 * Kept out of UsbIntrfTxData so byte mode does not pay for any of it.
 */
/**
 * Queue bytes and start the endpoint if this write made it startable. Inlined
 * on purpose: it is the tail of every write, and byte mode runs it once per
 * DeviceIntrfTx call.
 */
static inline __attribute__((always_inline))
void UsbIntrfTxQueued(UsbDevIntrf_t *pIntrf, int Cnt)
{
	if (Cnt <= 0 || pIntrf->Mps == 0U)
	{
		return;
	}

	const int used = CFifoUsed(pIntrf->hTxFifo);

	// An empty FIFO starts immediately. While an earlier USB transaction is
	// active, the producer only queues; completion drains full packets.
	if (used >= (int)pIntrf->Mps || used == Cnt)
	{
		(void)UsbIntrfStartXfer(pIntrf, used == Cnt);
	}
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

	const uint32_t blockSize = pIntrf->TxBlkSize;

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

	UsbIntrfTxQueued(pIntrf, cnt);

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
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

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

		memcpy(p, pData, (size_t)length);
		pData += length;
		DataLen -= length;
		cnt += length;
	}

	EnableInterrupt(state);

	UsbIntrfTxQueued(pIntrf, cnt);

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
	pIntrf->TxBlkSize = pCfg->TxFifoBlkSize;
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
		(pIntrf->TxBlkSize != 1U &&
		 pIntrf->TxBlkSize != sizeof(UsbPktHdr_t) + Mps))
	{
		return false;
	}

	pIntrf->Mps = Mps;
	CFifoFlush(pIntrf->hRxFifo);
	CFifoFlush(pIntrf->hTxFifo);
	UsbIntrfSetTxIdle(pIntrf);
	UsbIntrfRxArm(pIntrf);
	(void)UsbIntrfStartXfer(pIntrf, false);

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
	if (Result == USB_CTRLR_XFER_SUCCESS && Length <= pIntrf->Mps &&
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

			if (CFifoAvail(pIntrf->hRxFifo) <= 0 &&
				pIntrf->DevIntrf.EvtCB != nullptr)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
									   DEVINTRF_EVT_RX_FIFO_FULL,
									   nullptr, CFifoUsed(pIntrf->hRxFifo));
			}

			if (pIntrf->DevIntrf.EvtCB != nullptr)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
									   DEVINTRF_EVT_RX_DATA,
									   nullptr, CFifoUsed(pIntrf->hRxFifo));
			}
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
		if (Result == USB_CTRLR_XFER_FAILED)
		{
			UsbIntrfTxFailure(pIntrf, Length);
		}
		UsbIntrfSetTxIdle(pIntrf);
		return;
	}

	// Chain everything already queued while this consumer owns the endpoint.
	// A producer that runs later observes the idle token and starts itself.
	//
	// A tail shorter than MPS goes out here only when the producer added
	// nothing while this packet was on the bus. A producer that kept running
	// is faster served by waiting for a full packet, since a short packet
	// costs the same bus slot as a full one. A producer that stopped is
	// waiting on this data, so sending it now is what keeps a request and
	// response workload from paying a frame per exchange.
	const int used = CFifoUsed(pIntrf->hTxFifo);

	if (UsbIntrfCanTx(pIntrf) && used > 0 &&
		UsbIntrfSubmit(pIntrf, used <= pIntrf->TxUsedMark))
	{
		return;
	}

	// Terminate a byte-mode transfer ending on an MPS boundary without keeping
	// another software state bit. Completion of this ZLP has Length zero.
	if (UsbIntrfCanTx(pIntrf) && pIntrf->TxBlkSize == 1U &&
		Length == pIntrf->Mps && CFifoUsed(pIntrf->hTxFifo) == 0 &&
		UsbCtrlrEpXfer(pIntrf->DevNo, UsbIntrfTxAddr(pIntrf),
						pIntrf->pTxBuffer, 0U))
	{
		return;
	}

	if (!UsbIntrfTxIdle(pIntrf))
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

/**
 * Start of frame. Byte mode accumulates a full packet before sending, so a
 * tail shorter than MPS would otherwise wait for the application to produce
 * more. Sending it after one idle frame bounds that wait without breaking up
 * back to back full packets, which is what costs throughput.
 */
void UsbIntrfSof(UsbDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->TxBlkSize != 1U ||
		!UsbIntrfCanTx(pIntrf) || !UsbIntrfTxIdle(pIntrf) ||
		CFifoUsed(pIntrf->hTxFifo) <= 0)
	{
		if (pIntrf != nullptr) { pIntrf->TxTailArmed = false; }
		return;
	}

	// One frame of grace. Armed here, sent at the next SOF if still idle.
	if (!pIntrf->TxTailArmed)
	{
		pIntrf->TxTailArmed = true;
		return;
	}

	(void)UsbIntrfStartXfer(pIntrf, true);
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
	const uint32_t blockSize = pIntrf->TxBlkSize;
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
		(void)UsbIntrfStartXfer(pIntrf, false);
	}

	return CFifoAvail(pIntrf->hTxFifo) >= blocks;
}

int UsbIntrfTxUsed(UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hTxFifo != nullptr ?
		   CFifoUsed(pIntrf->hTxFifo) : 0;
}
