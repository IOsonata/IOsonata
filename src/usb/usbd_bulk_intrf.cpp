/**-------------------------------------------------------------------------
@file	usbd_bulk_intrf.cpp

@brief	Generic USB device bulk data interface implementation.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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
#include "usb/usbd_bulk_intrf.h"

static UsbdBulkDevIntrf_t *UsbdBulkIntrfData(DevIntrf_t * const pDevIntrf)
{
	if (pDevIntrf == nullptr || pDevIntrf->pDevData == nullptr)
	{
		return nullptr;
	}

	return static_cast<UsbdBulkDevIntrf_t *>(pDevIntrf->pDevData);
}

static void UsbdBulkIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);

	if (pIntrf != nullptr)
	{
		pIntrf->bEnabled = false;
	}
}

static void UsbdBulkIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);

	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->bEnabled = true;

	if (pIntrf->RxKick != nullptr)
	{
		pIntrf->RxKick(pIntrf, pIntrf->pContext);
	}

	if (pIntrf->TxKick != nullptr && CFifoUsed(pIntrf->hTxFifo) > 0)
	{
		pIntrf->TxKick(pIntrf, pIntrf->pContext);
	}
}

static uint32_t UsbdBulkIntrfGetRate(DevIntrf_t * const)
{
	// USB Bulk has no application-settable transfer rate.
	return 0;
}

static uint32_t UsbdBulkIntrfSetRate(DevIntrf_t * const, uint32_t)
{
	// USB Bulk has no application-settable transfer rate.
	return 0;
}

static bool UsbdBulkIntrfStartRx(DevIntrf_t * const, uint32_t)
{
	return true;
}

static int UsbdBulkIntrfRxData(DevIntrf_t * const pDevIntrf,
							   uint8_t *pBuffer, int BufferLen)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pIntrf->hRxFifo == nullptr ||
		pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (BufferLen > 0)
	{
		int length = BufferLen;
		uint8_t *p = CFifoGetMultiple(pIntrf->hRxFifo, &length);
		if (p == nullptr || length <= 0)
		{
			break;
		}

		memcpy(pBuffer, p, (size_t)length);
		pBuffer += length;
		BufferLen -= length;
		cnt += length;
	}

	EnableInterrupt(state);

	if (cnt > 0 && pIntrf->bEnabled && pIntrf->RxKick != nullptr)
	{
		pIntrf->RxKick(pIntrf, pIntrf->pContext);
	}

	return cnt;
}

static void UsbdBulkIntrfStopRx(DevIntrf_t * const)
{
}

static bool UsbdBulkIntrfStartTx(DevIntrf_t * const, uint32_t)
{
	return true;
}

static int UsbdBulkIntrfTxData(DevIntrf_t * const pDevIntrf,
							   const uint8_t *pData, int DataLen)
{
	UsbdBulkDevIntrf_t *pIntrf = UsbdBulkIntrfData(pDevIntrf);

	if (pIntrf == nullptr || pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	// The USB completion interrupt is the Tx FIFO consumer. CFifoPutMultiple
	// publishes PutIdx before the payload copy, so keep the producer critical
	// section identical to UART and copy directly into the reserved FIFO slots.
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

	// Same Tx ownership model as UART. The application only fills the FIFO.
	// The first writer that observes an idle transmitter claims it and kicks
	// the endpoint with whatever is currently queued. While a transfer is in
	// progress, subsequent writes return immediately after filling the FIFO;
	// completion interrupts keep draining it.
	if (cnt > 0 && pIntrf->bEnabled && pIntrf->TxKick != nullptr &&
		atomic_exchange(&pIntrf->DevIntrf.bTxReady, false))
	{
		pIntrf->TxKick(pIntrf, pIntrf->pContext);
	}

	return cnt;
}

static int UsbdBulkIntrfTxSrData(DevIntrf_t * const pDevIntrf,
								 const uint8_t *pData, int DataLen)
{
	return UsbdBulkIntrfTxData(pDevIntrf, pData, DataLen);
}

static void UsbdBulkIntrfStopTx(DevIntrf_t * const)
{
}

static void UsbdBulkIntrfReset(DevIntrf_t * const)
{
	// Bus reset does not discard application queues. The owning USB function
	// decides what to do with any transfer buffer that was active on the bus.
}

static void UsbdBulkIntrfPowerOff(DevIntrf_t * const pDevIntrf)
{
	UsbdBulkIntrfDisable(pDevIntrf);
}

static void *UsbdBulkIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return UsbdBulkIntrfData(pDevIntrf);
}

bool UsbdBulkIntrfInit(UsbdBulkDevIntrf_t *pIntrf,
					   const UsbdBulkIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0)
	{
		return false;
	}

	hCFifo_t hRxFifo = CFifoInit(pCfg->pRxFifoMem,
								 (uint32_t)pCfg->RxFifoMemSize,
								 1, pCfg->bBlocking);
	hCFifo_t hTxFifo = CFifoInit(pCfg->pTxFifoMem,
								 (uint32_t)pCfg->TxFifoMemSize,
								 1, pCfg->bBlocking);

	if (hRxFifo == nullptr || hTxFifo == nullptr)
	{
		return false;
	}

	pIntrf->hRxFifo = hRxFifo;
	pIntrf->hTxFifo = hTxFifo;
	pIntrf->RxKick = pCfg->RxKick;
	pIntrf->TxKick = pCfg->TxKick;
	pIntrf->pContext = pCfg->pContext;
	pIntrf->bEnabled = false;

	pIntrf->DevIntrf.pDevData = pIntrf;
	pIntrf->DevIntrf.IntPrio = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.bDma = false;
	pIntrf->DevIntrf.bIntEn = false;
	pIntrf->DevIntrf.Disable = UsbdBulkIntrfDisable;
	pIntrf->DevIntrf.Enable = UsbdBulkIntrfEnable;
	pIntrf->DevIntrf.GetRate = UsbdBulkIntrfGetRate;
	pIntrf->DevIntrf.SetRate = UsbdBulkIntrfSetRate;
	pIntrf->DevIntrf.StartRx = UsbdBulkIntrfStartRx;
	pIntrf->DevIntrf.RxData = UsbdBulkIntrfRxData;
	pIntrf->DevIntrf.StopRx = UsbdBulkIntrfStopRx;
	pIntrf->DevIntrf.StartTx = UsbdBulkIntrfStartTx;
	pIntrf->DevIntrf.TxData = UsbdBulkIntrfTxData;
	pIntrf->DevIntrf.TxSrData = UsbdBulkIntrfTxSrData;
	pIntrf->DevIntrf.StopTx = UsbdBulkIntrfStopTx;
	pIntrf->DevIntrf.Reset = UsbdBulkIntrfReset;
	pIntrf->DevIntrf.PowerOff = UsbdBulkIntrfPowerOff;
	pIntrf->DevIntrf.GetHandle = UsbdBulkIntrfGetHandle;

	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	DeviceIntrfEnable(&pIntrf->DevIntrf);

	return true;
}

int UsbdBulkIntrfPutRxData(UsbdBulkDevIntrf_t *pIntrf,
						   const uint8_t *pData, int DataLen)
{
	if (pIntrf == nullptr || pIntrf->hRxFifo == nullptr ||
		pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	const int requested = DataLen;
	uint32_t dropCnt = pIntrf->hRxFifo->DropCnt;
	uint32_t state = DisableInterrupt();
	int cnt = 0;

	while (DataLen > 0)
	{
		int length = DataLen;
		uint8_t *p = CFifoPutMultiple(pIntrf->hRxFifo, &length);
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

	if ((cnt < requested || pIntrf->hRxFifo->DropCnt != dropCnt) &&
		pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_RX_FIFO_FULL,
							   nullptr, CFifoUsed(pIntrf->hRxFifo));
	}

	if (cnt > 0 && pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_RX_DATA,
							   nullptr, CFifoUsed(pIntrf->hRxFifo));
	}

	return cnt;
}

int UsbdBulkIntrfGetTxData(UsbdBulkDevIntrf_t *pIntrf,
						   uint8_t *pBuffer, int BufferLen)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr ||
		pBuffer == nullptr || BufferLen <= 0)
	{
		return 0;
	}

	int cnt = 0;

	while (BufferLen > 0)
	{
		int length = BufferLen;
		uint8_t *p = CFifoGetMultiple(pIntrf->hTxFifo, &length);
		if (p == nullptr || length <= 0)
		{
			break;
		}

		memcpy(pBuffer, p, (size_t)length);
		pBuffer += length;
		BufferLen -= length;
		cnt += length;
	}

	return cnt;
}

void UsbdBulkIntrfTxComplete(UsbdBulkDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr)
	{
		return;
	}

	if (CFifoUsed(pIntrf->hTxFifo) > 0)
	{
		if (pIntrf->bEnabled && pIntrf->TxKick != nullptr)
		{
			pIntrf->TxKick(pIntrf, pIntrf->pContext);
		}
		return;
	}

	// The endpoint has caught the producer. Publish idle only after confirming
	// the FIFO is empty. Recheck after the store so a writer racing this edge
	// cannot leave queued data with no transfer active.
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);

	if (CFifoUsed(pIntrf->hTxFifo) > 0)
	{
		if (pIntrf->bEnabled && pIntrf->TxKick != nullptr &&
			atomic_exchange(&pIntrf->DevIntrf.bTxReady, false))
		{
			pIntrf->TxKick(pIntrf, pIntrf->pContext);
		}
		return;
	}

	if (pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
							   DEVINTRF_EVT_TX_FIFO_EMPTY,
							   nullptr, 0);
	}
}

bool UsbdBulkIntrfRequestToSend(UsbdBulkDevIntrf_t *pIntrf, int NbBytes)
{
	if (pIntrf == nullptr || pIntrf->hTxFifo == nullptr || NbBytes <= 0)
	{
		return false;
	}

	if (pIntrf->hTxFifo->bBlocking == false)
	{
		return true;
	}

	if (CFifoAvail(pIntrf->hTxFifo) < NbBytes &&
		pIntrf->bEnabled && pIntrf->TxKick != nullptr)
	{
		pIntrf->TxKick(pIntrf, pIntrf->pContext);
	}

	return CFifoAvail(pIntrf->hTxFifo) >= NbBytes;
}

int UsbdBulkIntrfRxUsed(UsbdBulkDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hRxFifo != nullptr ?
		   CFifoUsed(pIntrf->hRxFifo) : 0;
}

int UsbdBulkIntrfTxUsed(UsbdBulkDevIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->hTxFifo != nullptr ?
		   CFifoUsed(pIntrf->hTxFifo) : 0;
}
