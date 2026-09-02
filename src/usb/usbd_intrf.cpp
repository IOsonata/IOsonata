/**-------------------------------------------------------------------------
@file	usbd_intrf.cpp

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
#include "usb/usbd_intrf.h"

typedef struct __Usbd_Intrf_Rx_Slot {
	uint16_t Length;
	uint16_t Offset;
} UsbdIntrfRxSlot_t;

static UsbdDevIntrf_t *UsbdIntrfData(DevIntrf_t * const pDevIntrf)
{
	if (pDevIntrf == nullptr || pDevIntrf->pDevData == nullptr)
	{
		return nullptr;
	}

	return static_cast<UsbdDevIntrf_t *>(pDevIntrf->pDevData);
}

static UsbdIntrfRxSlot_t *UsbdIntrfRxSlot(UsbdDevIntrf_t *pIntrf,
									  uint32_t Index)
{
	return reinterpret_cast<UsbdIntrfRxSlot_t *>(
		pIntrf->pRxMem + (Index % pIntrf->RxSlotCnt) * pIntrf->RxSlotSize);
}

static uint8_t *UsbdIntrfRxBuffer(UsbdIntrfRxSlot_t *pSlot)
{
	return reinterpret_cast<uint8_t *>(pSlot + 1);
}

static uint32_t UsbdIntrfRxPut(const UsbdDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->RxPut, __ATOMIC_ACQUIRE);
}

static uint32_t UsbdIntrfRxGet(const UsbdDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->RxGet, __ATOMIC_ACQUIRE);
}

static bool UsbdIntrfRxEnabled(const UsbdDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->RxAccepting, __ATOMIC_ACQUIRE);
}

static void UsbdIntrfRxKick(UsbdDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || !pIntrf->bEnabled ||
		!UsbdIntrfRxEnabled(pIntrf) || pIntrf->RxEpAddr == 0U ||
		pIntrf->RxMps == 0U || pIntrf->RxSlotCnt == 0U)
	{
		return;
	}

	if (__atomic_exchange_n(&pIntrf->RxActive, true, __ATOMIC_ACQ_REL))
	{
		return;
	}

	const uint32_t put = UsbdIntrfRxPut(pIntrf);
	const uint32_t get = UsbdIntrfRxGet(pIntrf);

	if ((put - get) >= pIntrf->RxSlotCnt)
	{
		__atomic_store_n(&pIntrf->RxActive, false, __ATOMIC_RELEASE);
		return;
	}

	UsbdIntrfRxSlot_t *pSlot = UsbdIntrfRxSlot(pIntrf, put);
	pSlot->Length = 0U;
	pSlot->Offset = 0U;

	if (!UsbdCtrlrEpXfer(pIntrf->RxEpAddr,
						 UsbdIntrfRxBuffer(pSlot), pIntrf->RxMps))
	{
		__atomic_store_n(&pIntrf->RxActive, false, __ATOMIC_RELEASE);
	}
}

static bool UsbdIntrfCanTx(UsbdDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->bEnabled &&
		   pIntrf->TxEpAddr != 0U && pIntrf->pTxBuffer != nullptr;
}

static void UsbdIntrfSetTxIdle(UsbdDevIntrf_t *pIntrf)
{
	atomic_store_explicit(&pIntrf->DevIntrf.bTxReady, true,
						  memory_order_release);
}

static bool UsbdIntrfTakeTx(UsbdDevIntrf_t *pIntrf)
{
	return atomic_exchange_explicit(&pIntrf->DevIntrf.bTxReady, false,
									memory_order_acquire);
}

static bool UsbdIntrfTxIdle(UsbdDevIntrf_t *pIntrf)
{
	return atomic_load_explicit(&pIntrf->DevIntrf.bTxReady,
								memory_order_acquire);
}

static void UsbdIntrfTxFailure(UsbdDevIntrf_t *pIntrf, uint16_t Length)
{
	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_TIMEOUT,
							   nullptr, Length);
	}
}

static int UsbdIntrfFillPacket(UsbdDevIntrf_t *pIntrf)
{
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

static bool UsbdIntrfSubmit(UsbdDevIntrf_t *pIntrf, bool Tail)
{
	const int used = CFifoUsed(pIntrf->hTxFifo);
	int length = 0;

	if (used >= (int)pIntrf->TxMps || (Tail && used > 0))
	{
		length = UsbdIntrfFillPacket(pIntrf);
	}

	if (length == 0 && !(Tail && pIntrf->TxZlpRequired))
	{
		UsbdIntrfSetTxIdle(pIntrf);
		return false;
	}

	pIntrf->TxZlpRequired = false;
	pIntrf->TxTailArmed = false;

	if (UsbdCtrlrEpXfer(pIntrf->TxEpAddr, pIntrf->pTxBuffer,
						(uint16_t)length))
	{
		return true;
	}

	UsbdIntrfSetTxIdle(pIntrf);
	if (length > 0)
	{
		UsbdIntrfTxFailure(pIntrf, (uint16_t)length);
	}

	return false;
}

static bool UsbdIntrfStartXfer(UsbdDevIntrf_t *pIntrf, bool Tail)
{
	// USB transaction state belongs to the consumer. The producer can keep
	// filling the CFifo while this endpoint is busy.
	if (!UsbdIntrfTxIdle(pIntrf) || !UsbdIntrfCanTx(pIntrf) ||
		!UsbdIntrfTakeTx(pIntrf))
	{
		return false;
	}

	return UsbdIntrfSubmit(pIntrf, Tail);
}

static void UsbdIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	UsbdDevIntrf_t *pIntrf = UsbdIntrfData(pDevIntrf);

	if (pIntrf != nullptr)
	{
		pIntrf->bEnabled = false;
	}
}

static void UsbdIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	UsbdDevIntrf_t *pIntrf = UsbdIntrfData(pDevIntrf);

	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->bEnabled = true;
	UsbdIntrfRxKick(pIntrf);
	(void)UsbdIntrfStartXfer(pIntrf, false);
}

static uint32_t UsbdIntrfGetRate(DevIntrf_t * const)
{
	return 0;
}

static uint32_t UsbdIntrfSetRate(DevIntrf_t * const, uint32_t)
{
	return 0;
}

static bool UsbdIntrfStartRx(DevIntrf_t * const, uint32_t)
{
	return true;
}

static int UsbdIntrfRxData(DevIntrf_t * const pDevIntrf,
						   uint8_t *pBuffer, int BufferLen)
{
	UsbdDevIntrf_t *pIntrf = UsbdIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pIntrf->pRxMem == nullptr ||
		pIntrf->RxSlotCnt == 0U || pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	int cnt = 0;

	while (BufferLen > 0)
	{
		const uint32_t get = UsbdIntrfRxGet(pIntrf);
		const uint32_t put = UsbdIntrfRxPut(pIntrf);

		if (get == put)
		{
			break;
		}

		UsbdIntrfRxSlot_t *pSlot = UsbdIntrfRxSlot(pIntrf, get);
		const uint16_t length = pSlot->Length;
		uint16_t offset = pSlot->Offset;

		if (length == 0U || offset >= length)
		{
			pSlot->Length = 0U;
			pSlot->Offset = 0U;
			__atomic_store_n(&pIntrf->RxGet, get + 1U, __ATOMIC_RELEASE);
			continue;
		}

		int copyLen = (int)(length - offset);
		if (copyLen > BufferLen)
		{
			copyLen = BufferLen;
		}

		memcpy(pBuffer, UsbdIntrfRxBuffer(pSlot) + offset,
			   (size_t)copyLen);
		pBuffer += copyLen;
		BufferLen -= copyLen;
		cnt += copyLen;
		offset = (uint16_t)(offset + copyLen);
		pSlot->Offset = offset;
		__atomic_fetch_sub(&pIntrf->RxUsed, (uint32_t)copyLen,
						   __ATOMIC_RELEASE);

		if (offset == length)
		{
			pSlot->Length = 0U;
			pSlot->Offset = 0U;
			__atomic_store_n(&pIntrf->RxGet, get + 1U, __ATOMIC_RELEASE);
		}
	}

	if (cnt > 0)
	{
		UsbdIntrfRxKick(pIntrf);
	}

	return cnt;
}

static void UsbdIntrfStopRx(DevIntrf_t * const)
{
}

static bool UsbdIntrfStartTx(DevIntrf_t * const, uint32_t)
{
	return true;
}

static int UsbdIntrfTxData(DevIntrf_t * const pDevIntrf,
						   const uint8_t *pData, int DataLen)
{
	UsbdDevIntrf_t *pIntrf = UsbdIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	// CFifoPutMultiple publishes its producer index before memcpy fills the
	// returned area. Keep the USB completion consumer out until publication is
	// complete.
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

	if (cnt > 0 && pIntrf->TxMps > 0U)
	{
		const int used = CFifoUsed(pIntrf->hTxFifo);

		// An empty FIFO starts immediately. While an earlier USB transaction is
		// active, the producer only queues; completion drains full packets.
		if (used >= (int)pIntrf->TxMps || used == cnt)
		{
			(void)UsbdIntrfStartXfer(pIntrf, used == cnt);
		}
	}

	return cnt;
}

static int UsbdIntrfTxSrData(DevIntrf_t * const pDevIntrf,
							 const uint8_t *pData, int DataLen)
{
	return UsbdIntrfTxData(pDevIntrf, pData, DataLen);
}

static void UsbdIntrfStopTx(DevIntrf_t * const)
{
	// DeviceIntrf StopTx only ends the application-side operation. USB may
	// still be transmitting and continues to drain the CFifo independently.
}

static void UsbdIntrfReset(DevIntrf_t * const pDevIntrf)
{
	UsbdDevIntrf_t *pIntrf = UsbdIntrfData(pDevIntrf);
	UsbdIntrfResetTx(pIntrf);
}

static void UsbdIntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	UsbdIntrfDisable(pDevIntrf);
}

static void *UsbdIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return UsbdIntrfData(pDevIntrf);
}

bool UsbdIntrfInit(UsbdDevIntrf_t *pIntrf, const UsbdIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0 ||
		(((uintptr_t)pCfg->pRxFifoMem & 3U) != 0U))
	{
		return false;
	}

	hCFifo_t hTxFifo = CFifoInit(pCfg->pTxFifoMem,
								 (uint32_t)pCfg->TxFifoMemSize,
								 1, pCfg->bBlocking);

	if (hTxFifo == nullptr)
	{
		return false;
	}

	pIntrf->hTxFifo = hTxFifo;
	pIntrf->pRxMem = pCfg->pRxFifoMem;
	pIntrf->RxMemSize = (uint32_t)pCfg->RxFifoMemSize;
	pIntrf->RxMps = 0U;
	pIntrf->RxSlotSize = 0U;
	pIntrf->RxSlotCnt = 0U;
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
	__atomic_store_n(&pIntrf->RxPut, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxGet, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxUsed, 0U, __ATOMIC_RELEASE);

	pIntrf->DevIntrf.pDevData = pIntrf;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = true;
	pIntrf->DevIntrf.bIntEn = true;
	pIntrf->DevIntrf.Disable = UsbdIntrfDisable;
	pIntrf->DevIntrf.Enable = UsbdIntrfEnable;
	pIntrf->DevIntrf.GetRate = UsbdIntrfGetRate;
	pIntrf->DevIntrf.SetRate = UsbdIntrfSetRate;
	pIntrf->DevIntrf.StartRx = UsbdIntrfStartRx;
	pIntrf->DevIntrf.RxData = UsbdIntrfRxData;
	pIntrf->DevIntrf.StopRx = UsbdIntrfStopRx;
	pIntrf->DevIntrf.StartTx = UsbdIntrfStartTx;
	pIntrf->DevIntrf.TxData = UsbdIntrfTxData;
	pIntrf->DevIntrf.TxSrData = UsbdIntrfTxSrData;
	pIntrf->DevIntrf.StopTx = UsbdIntrfStopTx;
	pIntrf->DevIntrf.Reset = UsbdIntrfReset;
	pIntrf->DevIntrf.PowerOff = UsbdIntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbdIntrfGetHandle;

	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	DeviceIntrfEnable(&pIntrf->DevIntrf);

	return true;
}

bool UsbdIntrfConfigRx(UsbdDevIntrf_t *pIntrf, uint8_t EpAddr,
					   uint16_t Mps)
{
	if (pIntrf == nullptr || pIntrf->pRxMem == nullptr ||
		EpAddr == 0U || USB_ENDPADDR_IS_IN(EpAddr) || Mps == 0U)
	{
		return false;
	}

	const uint32_t slotSize =
		((uint32_t)sizeof(UsbdIntrfRxSlot_t) + Mps + 3U) & ~3U;
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
	pIntrf->RxSlotSize = (uint16_t)slotSize;
	pIntrf->RxSlotCnt = (uint16_t)slotCnt;
	memset(pIntrf->pRxMem, 0, pIntrf->RxMemSize);

	return true;
}

void UsbdIntrfResetRx(UsbdDevIntrf_t *pIntrf)
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
	pIntrf->RxSlotSize = 0U;
	pIntrf->RxSlotCnt = 0U;
}

void UsbdIntrfRxEnable(UsbdDevIntrf_t *pIntrf, bool Enable)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	__atomic_store_n(&pIntrf->RxAccepting, Enable, __ATOMIC_RELEASE);

	if (Enable)
	{
		UsbdIntrfRxKick(pIntrf);
	}
}

void UsbdIntrfRxXferComplete(UsbdDevIntrf_t *pIntrf,
						 uint16_t Length, UsbdCtrlrXferResult_t Result)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	__atomic_store_n(&pIntrf->RxActive, false, __ATOMIC_RELEASE);

	if (Result == USBD_CTRLR_XFER_SUCCESS && Length > 0U &&
		Length <= pIntrf->RxMps && pIntrf->bEnabled &&
		UsbdIntrfRxEnabled(pIntrf) && pIntrf->RxSlotCnt > 0U)
	{
		const uint32_t put = UsbdIntrfRxPut(pIntrf);
		const uint32_t get = UsbdIntrfRxGet(pIntrf);

		if ((put - get) < pIntrf->RxSlotCnt)
		{
			UsbdIntrfRxSlot_t *pSlot = UsbdIntrfRxSlot(pIntrf, put);
			pSlot->Length = Length;
			pSlot->Offset = 0U;
			__atomic_fetch_add(&pIntrf->RxUsed, Length, __ATOMIC_RELEASE);
			__atomic_store_n(&pIntrf->RxPut, put + 1U, __ATOMIC_RELEASE);

			if ((put + 1U - get) >= pIntrf->RxSlotCnt &&
				pIntrf->DevIntrf.EvtCB != nullptr)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
								   DEVINTRF_EVT_RX_FIFO_FULL,
								   nullptr, UsbdIntrfRxUsed(pIntrf));
			}

			if (pIntrf->DevIntrf.EvtCB != nullptr)
			{
				pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
								   DEVINTRF_EVT_RX_DATA,
								   nullptr, UsbdIntrfRxUsed(pIntrf));
			}
		}
		else
		{
			pIntrf->RxDropCnt++;
		}
	}
	else if (Result == USBD_CTRLR_XFER_FAILED)
	{
		pIntrf->RxDropCnt++;
		if (pIntrf->DevIntrf.EvtCB != nullptr)
		{
			pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_RX_TIMEOUT,
							   nullptr, Length);
		}
	}

	UsbdIntrfRxKick(pIntrf);
}

void UsbdIntrfConfigTx(UsbdDevIntrf_t *pIntrf, uint8_t EpAddr,
					   uint16_t Mps, uint8_t *pBuffer)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	if (EpAddr == 0U || !USB_ENDPADDR_IS_IN(EpAddr) || Mps == 0U ||
		pBuffer == nullptr)
	{
		pIntrf->TxEpAddr = 0U;
		pIntrf->TxMps = 0U;
		pIntrf->pTxBuffer = nullptr;
		UsbdIntrfResetTx(pIntrf);
		return;
	}

	pIntrf->TxEpAddr = EpAddr;
	pIntrf->TxMps = Mps;
	pIntrf->pTxBuffer = pBuffer;
	UsbdIntrfResetTx(pIntrf);
}

void UsbdIntrfResetTx(UsbdDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->TxZlpRequired = false;
	pIntrf->TxTailArmed = false;
	UsbdIntrfSetTxIdle(pIntrf);
}

void UsbdIntrfTxXferComplete(UsbdDevIntrf_t *pIntrf,
						 uint16_t Length, UsbdCtrlrXferResult_t Result)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr)
	{
		return;
	}

	if (Result != USBD_CTRLR_XFER_SUCCESS)
	{
		if (Result == USBD_CTRLR_XFER_FAILED)
		{
			UsbdIntrfTxFailure(pIntrf, Length);
		}
		UsbdIntrfSetTxIdle(pIntrf);
		return;
	}

	// Chain the next full packet while this consumer still owns the endpoint.
	if (UsbdIntrfCanTx(pIntrf) && UsbdIntrfSubmit(pIntrf, false))
	{
		return;
	}

	if (!UsbdIntrfTxIdle(pIntrf))
	{
		UsbdIntrfSetTxIdle(pIntrf);
	}

	if (CFifoUsed(pIntrf->hTxFifo) > 0)
	{
		return;
	}

	if (Length == pIntrf->TxMps)
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

void UsbdIntrfSof(UsbdDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr ||
		!UsbdIntrfTxIdle(pIntrf))
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

	(void)UsbdIntrfStartXfer(pIntrf, true);
}

bool UsbdIntrfRequestToSend(UsbdDevIntrf_t *pIntrf, int NbBytes)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr || NbBytes <= 0)
	{
		return false;
	}

	if (pIntrf->hTxFifo->bBlocking == false)
	{
		return true;
	}

	if (CFifoAvail(pIntrf->hTxFifo) < NbBytes)
	{
		(void)UsbdIntrfStartXfer(pIntrf, false);
	}

	return CFifoAvail(pIntrf->hTxFifo) >= NbBytes;
}

int UsbdIntrfRxUsed(UsbdDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr ?
		   (int)__atomic_load_n(&pIntrf->RxUsed, __ATOMIC_ACQUIRE) : 0;
}

int UsbdIntrfTxUsed(UsbdDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hTxFifo != nullptr ?
		   CFifoUsed(pIntrf->hTxFifo) : 0;
}
