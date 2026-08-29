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

	if (pIntrf == nullptr)
	{
		return 0;
	}

	return CFifoRead(pIntrf->hRxFifo, pBuffer, BufferLen);
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

	int cnt = CFifoWrite(pIntrf->hTxFifo,
						 const_cast<uint8_t *>(pData), DataLen);

	if (cnt > 0 && pIntrf->bEnabled && pIntrf->TxKick != nullptr)
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

	uint32_t dropCnt = pIntrf->hRxFifo->DropCnt;
	int cnt = CFifoWrite(pIntrf->hRxFifo,
						 const_cast<uint8_t *>(pData), DataLen);

	if ((cnt < DataLen || pIntrf->hRxFifo->DropCnt != dropCnt) &&
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

	return CFifoRead(pIntrf->hTxFifo, pBuffer, BufferLen);
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
