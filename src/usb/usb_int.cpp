/**-------------------------------------------------------------------------
@file	usb_int.cpp

@brief	USB interrupt-transfer specialization of UsbIntrf.

@author	Hoang Nguyen Hoan
@date	Sep. 10, 2026

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

#include "usb/usb_int.h"

static bool UsbIntIntrfEpSupported(int DevNo, uint8_t EpNo)
{
	return DevNo >= 0 && DevNo < USB_CTRLR_CNT && EpNo > 0U &&
		EpNo < USB_EPIN_CNT(DevNo) && EpNo < USB_EPOUT_CNT(DevNo) &&
		USB_INT_INTRF_MAX_MPS > 0U;
}

static bool UsbIntIntrfOpenEndpoint(UsbIntIntrf_t *pIntrf, bool bIn)
{
	return UsbCtrlrEpOpenData(pIntrf->pData->DevNo, pIntrf->pData->EpNo, bIn,
		USB_ENDPATT_TRANS_INT, pIntrf->Mps);
}

static void UsbIntIntrfDeactivate(UsbIntIntrf_t *pIntrf)
{
	if (pIntrf->pData->Mps != 0U)
	{
		UsbCtrlrEpClose(pIntrf->pData->DevNo, pIntrf->pData->EpNo, false);
		UsbCtrlrEpClose(pIntrf->pData->DevNo, pIntrf->pData->EpNo, true);
	}
	UsbIntrfUnconfigure(pIntrf->pData);
}

static bool UsbIntIntrfActivate(UsbIntIntrf_t *pIntrf)
{
	if (pIntrf->pData->Mps != 0U)
	{
		return true;
	}
	if (pIntrf->Mps == 0U || !UsbIntrfConfigure(pIntrf->pData, pIntrf->Mps))
	{
		return false;
	}
	if (!UsbIntIntrfOpenEndpoint(pIntrf, true))
	{
		UsbIntrfUnconfigure(pIntrf->pData);
		return false;
	}
	if (!UsbIntIntrfOpenEndpoint(pIntrf, false))
	{
		UsbCtrlrEpClose(pIntrf->pData->DevNo, pIntrf->pData->EpNo, true);
		UsbIntrfUnconfigure(pIntrf->pData);
		return false;
	}
	return true;
}

static void UsbIntIntrfDisable(DevIntrf_t * const pDev)
{
	UsbDevIntrf_t *pData = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	UsbIntIntrfDeactivate(static_cast<UsbIntIntrf_t *>(pData->pClassContext));
}

static void UsbIntIntrfEnable(DevIntrf_t * const pDev)
{
	UsbDevIntrf_t *pData = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	(void)UsbIntIntrfActivate(
		static_cast<UsbIntIntrf_t *>(pData->pClassContext));
}

static int UsbIntIntrfDataEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
								uint8_t *pBuffer, int Length)
{
	UsbDevIntrf_t *pData = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	UsbIntIntrf_t *pIntrf =
		static_cast<UsbIntIntrf_t *>(pData->pClassContext);

	switch (Event)
	{
		case DEVINTRF_EVT_RX_DATA:
			if (Length > (int)pIntrf->Mps)
			{
				pIntrf->RxErrorCnt++;
				return 0;
			}
			if (Length == 0)
			{
				pIntrf->RxEmptyCnt++;
			}
			if (pIntrf->RxHandler == nullptr)
			{
				return 0;
			}
			pIntrf->RxHandler(pIntrf, pBuffer, (uint16_t)Length,
				USB_CTRLR_XFER_SUCCESS, pIntrf->pContext);
			return Length;

		case DEVINTRF_EVT_RX_TIMEOUT:
			pIntrf->RxErrorCnt++;
			if (pIntrf->RxHandler != nullptr)
			{
				pIntrf->RxHandler(pIntrf, pData->pRxBuffer,
					(uint16_t)Length,
					USB_CTRLR_XFER_FAILED, pIntrf->pContext);
			}
			return 0;

		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			if (Length == 0)
			{
				pIntrf->TxEmptyCnt++;
			}
			if (pIntrf->TxHandler != nullptr)
			{
				pIntrf->TxHandler(pIntrf,
					(uint16_t)Length,
					USB_CTRLR_XFER_SUCCESS, pIntrf->pContext);
			}
			return Length;

		case DEVINTRF_EVT_TX_TIMEOUT:
			pIntrf->TxErrorCnt++;
			if (pIntrf->TxHandler != nullptr)
			{
				pIntrf->TxHandler(pIntrf,
					(uint16_t)Length,
					USB_CTRLR_XFER_FAILED, pIntrf->pContext);
			}
			return 0;

		default:
			return 0;
	}
}

bool UsbIntIntrfInit(UsbIntIntrf_t *pIntrf, UsbDevIntrf_t *pData,
					 const UsbIntIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pData == nullptr || pCfg == nullptr ||
		!UsbIntIntrfEpSupported(pCfg->DevNo, pCfg->EpNo))
	{
		return false;
	}

	memset(pIntrf, 0, sizeof(*pIntrf));
	pIntrf->pData = pData;
	pIntrf->pContext = pCfg->pContext;
	pIntrf->RxHandler = pCfg->RxHandler;
	pIntrf->TxHandler = pCfg->TxHandler;

	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = pCfg->DevNo;
	cfg.EpNo = pCfg->EpNo;
	cfg.bBlocking = true;
	cfg.Mode = USB_INTRF_MODE_DIRECT;
	cfg.BufferSize = USB_INT_INTRF_MAX_MPS;
	cfg.pRxBuffer = reinterpret_cast<uint8_t *>(pIntrf->RxBuffer);
	cfg.pTxBuffer = reinterpret_cast<uint8_t *>(pIntrf->TxBuffer);
	cfg.EvtCB = UsbIntIntrfDataEvent;

	if (!UsbIntrfInit(pIntrf->pData, &cfg))
	{
		return false;
	}

	pIntrf->pData->pClassContext = pIntrf;
	pIntrf->pData->DevIntrf.Disable = UsbIntIntrfDisable;
	pIntrf->pData->DevIntrf.Enable = UsbIntIntrfEnable;
	return true;
}

bool UsbIntIntrfOpen(UsbIntIntrf_t *pIntrf, uint16_t MaxPacketSize,
					  uint8_t Interval)
{
	if (pIntrf == nullptr ||
		MaxPacketSize == 0U || MaxPacketSize > USB_INT_INTRF_MAX_MPS ||
		Interval == 0U ||
		(UsbCtrlrHighSpeed(pIntrf->pData->DevNo) ?
		 Interval > 16U : MaxPacketSize > USB_INT_INTRF_FS_MPS))
	{
		return false;
	}

	UsbIntIntrfClose(pIntrf);
	pIntrf->Mps = MaxPacketSize;
	pIntrf->Interval = Interval;

	if (atomic_load_explicit(&pIntrf->pData->DevIntrf.EnCnt,
			memory_order_acquire) > 0 &&
		!UsbIntIntrfActivate(pIntrf))
	{
		pIntrf->Mps = 0U;
		pIntrf->Interval = 0U;
		return false;
	}
	return true;
}

void UsbIntIntrfClose(UsbIntIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	UsbIntIntrfDeactivate(pIntrf);
	pIntrf->Mps = 0U;
	pIntrf->Interval = 0U;
}

void UsbIntIntrfReset(UsbIntIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	UsbIntIntrfClose(pIntrf);
	pIntrf->RxErrorCnt = 0U;
	pIntrf->TxErrorCnt = 0U;
	pIntrf->RxEmptyCnt = 0U;
	pIntrf->TxEmptyCnt = 0U;
}
