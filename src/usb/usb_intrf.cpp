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

/// Packet slot at a monotonic ring index.
static UsbPktHdr_t *UsbIntrfRxSlot(UsbDevIntrf_t *pIntrf, uint32_t Index)
{
	return reinterpret_cast<UsbPktHdr_t *>(
		pIntrf->pRxMem + (Index % pIntrf->RxSlotCnt) * pIntrf->RxSlotSize);
}

static uint32_t UsbIntrfRxPut(const UsbDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->RxPut, __ATOMIC_ACQUIRE);
}

static uint32_t UsbIntrfRxGet(const UsbDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->RxGet, __ATOMIC_ACQUIRE);
}

static bool UsbIntrfRxEnabled(const UsbDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->RxAccepting, __ATOMIC_ACQUIRE);
}

/**
 * Arm the OUT endpoint on the slot the ring will publish next. The controller
 * writes straight into it. The slot is not visible to the reader until the
 * transfer completes, so nothing is copied and there is no critical section.
 */
static void UsbIntrfRxKick(UsbDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || !pIntrf->bEnabled ||
		!UsbIntrfRxEnabled(pIntrf) || pIntrf->RxEpAddr == 0U ||
		pIntrf->RxMps == 0U || pIntrf->RxSlotCnt == 0U)
	{
		return;
	}

	if (__atomic_exchange_n(&pIntrf->RxActive, true, __ATOMIC_ACQ_REL))
	{
		return;
	}

	const uint32_t put = UsbIntrfRxPut(pIntrf);
	const uint32_t get = UsbIntrfRxGet(pIntrf);

	// Full ring leaves the endpoint unarmed. The host is backpressured by USB
	// rather than a packet being dropped.
	if ((put - get) >= pIntrf->RxSlotCnt)
	{
		__atomic_store_n(&pIntrf->RxActive, false, __ATOMIC_RELEASE);
		return;
	}

	UsbPktHdr_t *pSlot = UsbIntrfRxSlot(pIntrf, put);
	pSlot->Length = 0U;
	pSlot->Reserved = 0U;

	if (!UsbCtrlrEpXfer(pIntrf->DevNo, pIntrf->RxEpAddr,
						UsbIntrfPktData(pSlot), pIntrf->RxMps))
	{
		__atomic_store_n(&pIntrf->RxActive, false, __ATOMIC_RELEASE);
	}
}

static bool UsbIntrfCanTx(UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->bEnabled &&
		   pIntrf->TxEpAddr != 0U && pIntrf->pTxBuffer != nullptr;
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
	if (CFifoBlockSize(pIntrf->hTxFifo) != 1U)
	{
		UsbPktHdr_t *pPacket = reinterpret_cast<UsbPktHdr_t *>(
			CFifoPeek(pIntrf->hTxFifo));
		if (pPacket == nullptr || pPacket->Length > pIntrf->TxMps)
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
	int remain = (int)pIntrf->TxMps;
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
	const bool packetMode = CFifoBlockSize(pIntrf->hTxFifo) != 1U;

	if ((packetMode && used > 0) ||
		(!packetMode && (used >= (int)pIntrf->TxMps || (Tail && used > 0))))
	{
		length = UsbIntrfFillPacket(pIntrf);
	}

	if (length < 0)
	{
		UsbIntrfSetTxIdle(pIntrf);
		return false;
	}

	if (length == 0 && !(packetMode && used > 0) &&
		!(Tail && pIntrf->TxZlpRequired))
	{
		UsbIntrfSetTxIdle(pIntrf);
		return false;
	}

	pIntrf->TxZlpRequired = false;
	pIntrf->TxTailArmed = false;

	if (UsbCtrlrEpXfer(pIntrf->DevNo, pIntrf->TxEpAddr, pIntrf->pTxBuffer,
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
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	if (pIntrf != nullptr)
	{
		pIntrf->bEnabled = false;
	}
}

static void UsbIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->bEnabled = true;
	UsbIntrfRxKick(pIntrf);
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

/**
 * Byte stream view of the stored packets. Packet boundaries stay in the ring.
 * The read position within the head packet is RxOffset on this object, so the
 * stored packet header carries packet information only.
 */
static int UsbIntrfRxData(DevIntrf_t * const pDevIntrf,
						  uint8_t *pBuffer, int BufferLen)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pIntrf->pRxMem == nullptr ||
		pIntrf->RxSlotCnt == 0U || pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	int cnt = 0;
	bool released = false;

	while (BufferLen > 0)
	{
		const uint32_t get = UsbIntrfRxGet(pIntrf);

		if (get == UsbIntrfRxPut(pIntrf))
		{
			break;
		}

		UsbPktHdr_t *pSlot = UsbIntrfRxSlot(pIntrf, get);
		const uint16_t length = pSlot->Length;

		// A zero length packet carries no stream data. It is released here
		// without consuming any of the caller's buffer.
		if (pIntrf->RxOffset >= length)
		{
			pIntrf->RxOffset = 0U;
			__atomic_store_n(&pIntrf->RxGet, get + 1U, __ATOMIC_RELEASE);
			released = true;
			continue;
		}

		int copyLen = (int)(length - pIntrf->RxOffset);
		if (copyLen > BufferLen)
		{
			copyLen = BufferLen;
		}

		memcpy(pBuffer, UsbIntrfPktData(pSlot) + pIntrf->RxOffset,
			   (size_t)copyLen);
		pBuffer += copyLen;
		BufferLen -= copyLen;
		cnt += copyLen;
		pIntrf->RxOffset = (uint16_t)(pIntrf->RxOffset + copyLen);
		__atomic_fetch_sub(&pIntrf->RxUsed, (uint32_t)copyLen,
						   __ATOMIC_RELEASE);

		if (pIntrf->RxOffset >= length)
		{
			pIntrf->RxOffset = 0U;
			__atomic_store_n(&pIntrf->RxGet, get + 1U, __ATOMIC_RELEASE);
			released = true;
		}
	}

	if (released)
	{
		UsbIntrfRxKick(pIntrf);
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

static int UsbIntrfTxData(DevIntrf_t * const pDevIntrf,
						   const uint8_t *pData, int DataLen)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	// CFifoPutMultiple publishes its producer index before memcpy fills the
	// returned area. Keep the USB completion consumer out until publication is
	// complete.
	uint32_t state = DisableInterrupt();
	int cnt = 0;

	const uint32_t blockSize = CFifoBlockSize(pIntrf->hTxFifo);
	if (blockSize != 1U &&
		(blockSize != sizeof(UsbPktHdr_t) + pIntrf->TxMps ||
		 (DataLen % (int)blockSize) != 0))
	{
		EnableInterrupt(state);
		return 0;
	}
	if (blockSize != 1U)
	{
		for (int offset = 0; offset < DataLen; offset += (int)blockSize)
		{
			const UsbPktHdr_t *pPacket =
				reinterpret_cast<const UsbPktHdr_t *>(pData + offset);
			if (pPacket->Length > pIntrf->TxMps)
			{
				EnableInterrupt(state);
				return 0;
			}
		}
	}

	while (DataLen > 0)
	{
		int blocks = blockSize == 1U ? DataLen : DataLen / (int)blockSize;
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

	if (cnt > 0 && pIntrf->TxMps > 0U)
	{
		const int used = CFifoUsed(pIntrf->hTxFifo);

		// An empty FIFO starts immediately. While an earlier USB transaction is
		// active, the producer only queues; completion drains full packets.
		if (blockSize != 1U || used >= (int)pIntrf->TxMps || used == cnt)
		{
			(void)UsbIntrfStartXfer(pIntrf, used == cnt);
		}
	}

	return cnt;
}

static int UsbIntrfTxSrData(DevIntrf_t * const pDevIntrf,
							 const uint8_t *pData, int DataLen)
{
	return UsbIntrfTxData(pDevIntrf, pData, DataLen);
}

static void UsbIntrfStopTx(DevIntrf_t * const)
{
	// DeviceIntrf StopTx only ends the application-side operation. USB may
	// still be transmitting and continues to drain the CFifo independently.
}

static void UsbIntrfReset(DevIntrf_t * const pDevIntrf)
{
	UsbDevIntrf_t *pIntrf = UsbIntrfData(pDevIntrf);
	UsbIntrfResetRx(pIntrf);
	UsbIntrfResetTx(pIntrf);
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
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0 ||
		pCfg->TxFifoBlkSize == 0U ||
		(((uintptr_t)pCfg->pRxFifoMem & 3U) != 0U) ||
		(((uintptr_t)pCfg->pTxFifoMem & 3U) != 0U))
	{
		return false;
	}

	hCFifo_t hTxFifo = CFifoInit(pCfg->pTxFifoMem,
								 (uint32_t)pCfg->TxFifoMemSize,
								 pCfg->TxFifoBlkSize, pCfg->bBlocking);

	if (hTxFifo == nullptr)
	{
		return false;
	}

	pIntrf->DevNo = pCfg->DevNo;
	pIntrf->hTxFifo = hTxFifo;
	pIntrf->pRxMem = pCfg->pRxFifoMem;
	pIntrf->RxMemSize = (uint32_t)pCfg->RxFifoMemSize;
	pIntrf->RxPut = 0U;
	pIntrf->RxGet = 0U;
	pIntrf->RxSlotSize = 0U;
	pIntrf->RxSlotCnt = 0U;
	pIntrf->RxOffset = 0U;
	pIntrf->RxMps = 0U;
	pIntrf->RxEpAddr = 0U;
	pIntrf->TxEpAddr = 0U;
	pIntrf->pTxBuffer = nullptr;
	pIntrf->TxMps = 0U;
	pIntrf->bEnabled = false;
	pIntrf->RxActive = false;
	pIntrf->RxAccepting = false;
	pIntrf->RxDropCnt = 0U;
	pIntrf->TxZlpRequired = false;
	pIntrf->TxTailArmed = false;
	__atomic_store_n(&pIntrf->RxUsed, 0U, __ATOMIC_RELEASE);

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
	pIntrf->DevIntrf.TxData = UsbIntrfTxData;
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

bool UsbIntrfConfigRx(UsbDevIntrf_t *pIntrf, uint8_t EpAddr, uint16_t Mps)
{
	if (pIntrf == nullptr || pIntrf->pRxMem == nullptr ||
		EpAddr == 0U || USB_ENDPADDR_IS_IN(EpAddr) || Mps == 0U)
	{
		return false;
	}

	const uint32_t slotSize = USB_INTRF_PKT_BLKSIZE(Mps);
	const uint32_t slotCnt = pIntrf->RxMemSize / slotSize;

	if (slotSize > UINT16_MAX || slotCnt == 0U || slotCnt > UINT16_MAX)
	{
		return false;
	}

	__atomic_store_n(&pIntrf->RxActive, false, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxAccepting, false, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxPut, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxGet, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxUsed, 0U, __ATOMIC_RELEASE);
	pIntrf->RxEpAddr = EpAddr;
	pIntrf->RxMps = Mps;
	pIntrf->RxOffset = 0U;
	pIntrf->RxSlotSize = (uint16_t)slotSize;
	pIntrf->RxSlotCnt = (uint16_t)slotCnt;
	memset(pIntrf->pRxMem, 0, pIntrf->RxMemSize);

	return true;
}

void UsbIntrfResetRx(UsbDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	__atomic_store_n(&pIntrf->RxAccepting, false, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxActive, false, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxPut, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxGet, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxUsed, 0U, __ATOMIC_RELEASE);
	pIntrf->RxEpAddr = 0U;
	pIntrf->RxMps = 0U;
	pIntrf->RxOffset = 0U;
	pIntrf->RxSlotSize = 0U;
	pIntrf->RxSlotCnt = 0U;
}
void UsbIntrfRxEnable(UsbDevIntrf_t *pIntrf, bool Enable)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	__atomic_store_n(&pIntrf->RxAccepting, Enable, __ATOMIC_RELEASE);

	if (Enable)
	{
		UsbIntrfRxKick(pIntrf);
	}
}

void UsbIntrfRxXferComplete(UsbDevIntrf_t *pIntrf, uint16_t Length,
							UsbCtrlrXferResult_t Result)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	__atomic_store_n(&pIntrf->RxActive, false, __ATOMIC_RELEASE);

	// Length zero is a zero length packet, which is a packet. Publishing it
	// keeps the boundary the host sent.
	if (Result == USB_CTRLR_XFER_SUCCESS && Length <= pIntrf->RxMps &&
		pIntrf->bEnabled && UsbIntrfRxEnabled(pIntrf) &&
		pIntrf->RxSlotCnt > 0U)
	{
		const uint32_t put = UsbIntrfRxPut(pIntrf);
		const uint32_t get = UsbIntrfRxGet(pIntrf);

		if ((put - get) < pIntrf->RxSlotCnt)
		{
			// The controller has finished writing this slot. Storing the
			// length and then advancing RxPut publishes a packet that is
			// already complete, so no reader can see a partial one.
			UsbIntrfRxSlot(pIntrf, put)->Length = Length;
			__atomic_fetch_add(&pIntrf->RxUsed, Length, __ATOMIC_RELEASE);
			__atomic_store_n(&pIntrf->RxPut, put + 1U, __ATOMIC_RELEASE);

			if ((put + 1U - get) >= pIntrf->RxSlotCnt &&
				pIntrf->DevIntrf.EvtCB != nullptr)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
									   DEVINTRF_EVT_RX_FIFO_FULL,
									   nullptr, UsbIntrfRxUsed(pIntrf));
			}

			if (pIntrf->DevIntrf.EvtCB != nullptr)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
									   DEVINTRF_EVT_RX_DATA,
									   nullptr, UsbIntrfRxUsed(pIntrf));
			}
		}
		else
		{
			// The endpoint is armed only when a slot is free, so reaching
			// here means the ring moved underneath the transfer. Count it,
			// the packet is gone.
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

	UsbIntrfRxKick(pIntrf);
}

void UsbIntrfConfigTx(UsbDevIntrf_t *pIntrf, uint8_t EpAddr,
					   uint16_t Mps, uint8_t *pBuffer)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	if (EpAddr == 0U || !USB_ENDPADDR_IS_IN(EpAddr) || Mps == 0U ||
		pBuffer == nullptr ||
		(CFifoBlockSize(pIntrf->hTxFifo) != 1U &&
		 CFifoBlockSize(pIntrf->hTxFifo) != sizeof(UsbPktHdr_t) + Mps))
	{
		pIntrf->TxEpAddr = 0U;
		pIntrf->TxMps = 0U;
		pIntrf->pTxBuffer = nullptr;
		UsbIntrfResetTx(pIntrf);
		return;
	}

	pIntrf->TxEpAddr = EpAddr;
	pIntrf->TxMps = Mps;
	pIntrf->pTxBuffer = pBuffer;
	UsbIntrfResetTx(pIntrf);
}

void UsbIntrfResetTx(UsbDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->TxZlpRequired = false;
	pIntrf->TxTailArmed = false;
	CFifoFlush(pIntrf->hTxFifo);
	UsbIntrfSetTxIdle(pIntrf);
}

void UsbIntrfTxXferComplete(UsbDevIntrf_t *pIntrf,
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

	// Chain the next full packet while this consumer still owns the endpoint.
	if (UsbIntrfCanTx(pIntrf) && UsbIntrfSubmit(pIntrf, false))
	{
		return;
	}

	if (!UsbIntrfTxIdle(pIntrf))
	{
		UsbIntrfSetTxIdle(pIntrf);
	}

	if (CFifoUsed(pIntrf->hTxFifo) > 0)
	{
		return;
	}

	if (CFifoBlockSize(pIntrf->hTxFifo) == 1U && Length == pIntrf->TxMps)
	{
		pIntrf->TxZlpRequired = true;
		return;
	}

	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_FIFO_EMPTY,
							   nullptr, 0);
	}
}

void UsbIntrfSof(UsbDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr ||
		!UsbIntrfTxIdle(pIntrf))
	{
		return;
	}

	if (CFifoUsed(pIntrf->hTxFifo) == 0 && !pIntrf->TxZlpRequired)
	{
		pIntrf->TxTailArmed = false;
		return;
	}

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
		(void)UsbIntrfStartXfer(pIntrf, false);
	}

	return CFifoAvail(pIntrf->hTxFifo) >= blocks;
}

int UsbIntrfRxUsed(UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr ?
		   (int)__atomic_load_n(&pIntrf->RxUsed, __ATOMIC_ACQUIRE) : 0;
}

int UsbIntrfTxUsed(UsbDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hTxFifo != nullptr ?
		   CFifoUsed(pIntrf->hTxFifo) : 0;
}
