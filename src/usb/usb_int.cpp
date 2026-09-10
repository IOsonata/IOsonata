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

static bool UsbIntIntrfOpenEndpoint(UsbIntIntrf_t *pIntrf, uint8_t EpAddr)
{
	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bEndpointAddress = EpAddr;
	desc.bmAttributes = USB_ENDPATT_TRANS_INT;
	desc.wMaxPacketSize = pIntrf->Mps;
	desc.bInterval = pIntrf->Interval;

	return UsbCtrlrEpOpen(pIntrf->IntrfData.DevNo, &desc);
}

static int UsbIntIntrfDataEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
								uint8_t *pBuffer, int Length)
{
	UsbDevIntrf_t *pData = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	UsbIntIntrf_t *pIntrf = pData != nullptr ?
		static_cast<UsbIntIntrf_t *>(pData->pClassContext) : nullptr;
	if (pIntrf == nullptr)
	{
		return 0;
	}

	switch (Event)
	{
		case DEVINTRF_EVT_RX_DATA:
			if (Length < 0 || Length > (int)pIntrf->Mps)
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
					Length > 0 ? (uint16_t)Length : 0U,
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
					Length > 0 ? (uint16_t)Length : 0U,
					USB_CTRLR_XFER_SUCCESS, pIntrf->pContext);
			}
			return Length;

		case DEVINTRF_EVT_TX_TIMEOUT:
			pIntrf->TxErrorCnt++;
			if (pIntrf->TxHandler != nullptr)
			{
				pIntrf->TxHandler(pIntrf,
					Length > 0 ? (uint16_t)Length : 0U,
					USB_CTRLR_XFER_FAILED, pIntrf->pContext);
			}
			return 0;

		default:
			return 0;
	}
}

bool UsbIntIntrfInit(UsbIntIntrf_t *pIntrf, const UsbIntIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		!UsbIntIntrfEpSupported(pCfg->DevNo, pCfg->EpNo))
	{
		return false;
	}

	memset(pIntrf, 0, sizeof(*pIntrf));
	pIntrf->pContext = pCfg->pContext;
	pIntrf->RxHandler = pCfg->RxHandler;
	pIntrf->TxHandler = pCfg->TxHandler;
	pIntrf->EpNo = pCfg->EpNo;

	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = pCfg->DevNo;
	cfg.EpNo = pCfg->EpNo;
	cfg.bBlocking = !USB_OUT_PREARM(pCfg->DevNo);
	cfg.bRxPrearm = USB_OUT_PREARM(pCfg->DevNo);
	cfg.Mode = USB_INTRF_MODE_DIRECT;
	cfg.BufferSize = USB_INT_INTRF_MAX_MPS;
	cfg.pRxBuffer = reinterpret_cast<uint8_t *>(pIntrf->RxBuffer);
	cfg.pTxBuffer = reinterpret_cast<uint8_t *>(pIntrf->TxBuffer);
	cfg.EvtCB = UsbIntIntrfDataEvent;

	if (!UsbIntrfInit(&pIntrf->IntrfData, &cfg))
	{
		return false;
	}

	pIntrf->IntrfData.pClassContext = pIntrf;
	return true;
}

bool UsbIntIntrfOpen(UsbIntIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval)
{
	if (pIntrf == nullptr ||
		!UsbIntIntrfEpSupported(pIntrf->IntrfData.DevNo, pIntrf->EpNo) ||
		Mps == 0U || Mps > USB_INT_INTRF_MAX_MPS || Interval == 0U ||
		(!UsbCtrlrHighSpeed(pIntrf->IntrfData.DevNo) &&
		 Mps > USB_INT_INTRF_FS_MPS) ||
		(UsbCtrlrHighSpeed(pIntrf->IntrfData.DevNo) && Interval > 16U))
	{
		return false;
	}

	UsbIntIntrfClose(pIntrf);
	if (!UsbIntrfConfigure(&pIntrf->IntrfData, Mps))
	{
		return false;
	}

	pIntrf->Mps = Mps;
	pIntrf->Interval = Interval;
	pIntrf->Suspended = false;

	if (!UsbIntIntrfOpenEndpoint(pIntrf, USB_ENDPADDR_DIRIN(pIntrf->EpNo)) ||
		!UsbIntIntrfOpenEndpoint(pIntrf, USB_ENDPADDR_DIROUT(pIntrf->EpNo)) ||
		(pIntrf->IntrfData.bRxPrearm &&
		 !UsbIntrfArmRx(&pIntrf->IntrfData)))
	{
		UsbCtrlrEpClose(pIntrf->IntrfData.DevNo,
			USB_ENDPADDR_DIROUT(pIntrf->EpNo));
		UsbCtrlrEpClose(pIntrf->IntrfData.DevNo,
			USB_ENDPADDR_DIRIN(pIntrf->EpNo));
		UsbIntrfUnconfigure(&pIntrf->IntrfData);
		pIntrf->Mps = 0U;
		pIntrf->Interval = 0U;
		return false;
	}

	pIntrf->Opened = true;
	return true;
}

void UsbIntIntrfClose(UsbIntIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	if (pIntrf->Opened)
	{
		UsbCtrlrEpClose(pIntrf->IntrfData.DevNo,
			USB_ENDPADDR_DIROUT(pIntrf->EpNo));
		UsbCtrlrEpClose(pIntrf->IntrfData.DevNo,
			USB_ENDPADDR_DIRIN(pIntrf->EpNo));
	}

	UsbIntrfUnconfigure(&pIntrf->IntrfData);
	pIntrf->Opened = false;
	pIntrf->Suspended = false;
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

void UsbIntIntrfSuspend(UsbIntIntrf_t *pIntrf)
{
	if (pIntrf != nullptr && pIntrf->Opened)
	{
		pIntrf->Suspended = true;
	}
}

bool UsbIntIntrfResume(UsbIntIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || !pIntrf->Opened)
	{
		return false;
	}

	pIntrf->Suspended = false;
	return true;
}

bool UsbIntIntrfSendPacket(UsbIntIntrf_t *pIntrf, const uint8_t *pData,
						   uint16_t Length)
{
	if (pIntrf == nullptr || !pIntrf->Opened || pIntrf->Suspended ||
		Length > pIntrf->Mps || (Length != 0U && pData == nullptr) ||
		!UsbIntrfRequestToSend(&pIntrf->IntrfData, Length))
	{
		return false;
	}

	const int sent = DeviceIntrfTxData(&pIntrf->IntrfData.DevIntrf,
		pData, (int)Length);
	if (Length != 0U)
	{
		return sent == (int)Length;
	}

	return !atomic_load_explicit(&pIntrf->IntrfData.DevIntrf.bTxReady,
		memory_order_acquire);
}
