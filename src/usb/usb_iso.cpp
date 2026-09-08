/**-------------------------------------------------------------------------
@file	usb_iso.cpp

@brief	Reusable bidirectional USB isochronous endpoint implementation.

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2026

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

#include "usb/usb_iso.h"

static uint8_t *UsbIsoIntrfRxBuffer(UsbIsoIntrf_t *pIntrf)
{
	return reinterpret_cast<uint8_t *>(pIntrf->RxBuffer);
}

static uint8_t *UsbIsoIntrfTxBuffer(UsbIsoIntrf_t *pIntrf)
{
	return reinterpret_cast<uint8_t *>(pIntrf->TxBuffer);
}

static bool UsbIsoIntrfEpSupported(const UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->DevNo < 0 ||
		pIntrf->DevNo >= USB_CTRLR_CNT || pIntrf->EpNo == 0U ||
		pIntrf->EpNo > 15U || !USB_ISO_SUPPORTED(pIntrf->DevNo))
	{
		return false;
	}

	const uint16_t bit = (uint16_t)(1U << pIntrf->EpNo);
	return (USB_ISO_EPIN_MASK(pIntrf->DevNo) & bit) != 0U &&
		(USB_ISO_EPOUT_MASK(pIntrf->DevNo) & bit) != 0U;
}

static void UsbIsoIntrfComplete(uint8_t EpAddr, UsbCtrlrEvtType_t Event,
								uint16_t Length,
								UsbCtrlrXferResult_t Result, void *pContext)
{
	UsbIsoIntrf_t *pIntrf = static_cast<UsbIsoIntrf_t *>(pContext);
	if (pIntrf == nullptr || USB_ENDPADDR_NUM(EpAddr) != pIntrf->EpNo)
	{
		return;
	}

	if (Event == USB_CTRLR_EVT_DRDY)
	{
		if (USB_ENDPADDR_IS_IN(EpAddr) || !pIntrf->Opened ||
			pIntrf->Suspended || pIntrf->RxArmed)
		{
			return;
		}
		if (UsbCtrlrEpXfer(pIntrf->DevNo, EpAddr, pIntrf->Mps))
		{
			pIntrf->RxArmed = true;
		}
		else
		{
			pIntrf->RxMissCnt++;
		}
		return;
	}

	if (Event != USB_CTRLR_EVT_XFER_CMPL)
	{
		return;
	}

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		if (!pIntrf->TxActive)
		{
			return;
		}

		const uint16_t expected = pIntrf->TxLength;
		pIntrf->TxActive = false;
		pIntrf->TxLength = 0U;

		UsbCtrlrXferResult_t result = Result;
		if (result == USB_CTRLR_XFER_SUCCESS && Length != expected)
		{
			result = USB_CTRLR_XFER_FAILED;
		}
		if (result != USB_CTRLR_XFER_SUCCESS)
		{
			pIntrf->TxMissCnt++;
		}
		else if (Length == 0U)
		{
			pIntrf->TxEmptyCnt++;
		}

		if (pIntrf->TxHandler != nullptr)
		{
			pIntrf->TxHandler(pIntrf, Length, result, pIntrf->pContext);
		}
		return;
	}

	pIntrf->RxArmed = false;
	UsbCtrlrXferResult_t result = Result;
	if (result == USB_CTRLR_XFER_SUCCESS && Length > pIntrf->Mps)
	{
		result = USB_CTRLR_XFER_FAILED;
	}
	if (result != USB_CTRLR_XFER_SUCCESS)
	{
		pIntrf->RxMissCnt++;
	}
	else if (Length == 0U)
	{
		pIntrf->RxEmptyCnt++;
	}

	if (pIntrf->RxHandler != nullptr)
	{
		pIntrf->RxHandler(pIntrf, UsbIsoIntrfRxBuffer(pIntrf), Length,
			result, pIntrf->pContext);
	}
}

bool UsbIsoIntrfInit(UsbIsoIntrf_t *pIntrf, const UsbIsoIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr)
	{
		return false;
	}

	memset(pIntrf, 0, sizeof(*pIntrf));
	pIntrf->DevNo = pCfg->DevNo;
	pIntrf->EpNo = pCfg->EpNo;
	pIntrf->RxHandler = pCfg->RxHandler;
	pIntrf->TxHandler = pCfg->TxHandler;
	pIntrf->pContext = pCfg->pContext;

	if (!UsbIsoIntrfEpSupported(pIntrf) || USB_ISO_INTRF_MAX_MPS == 0U)
	{
		return false;
	}

	if (!UsbCtrlrEpRegister(pIntrf->DevNo,
			USB_ENDPADDR_DIROUT(pIntrf->EpNo), UsbIsoIntrfRxBuffer(pIntrf),
			false, UsbIsoIntrfComplete, pIntrf) ||
		!UsbCtrlrEpRegister(pIntrf->DevNo,
			USB_ENDPADDR_DIRIN(pIntrf->EpNo), UsbIsoIntrfTxBuffer(pIntrf),
			false, UsbIsoIntrfComplete, pIntrf))
	{
		return false;
	}

	return true;
}

bool UsbIsoIntrfOpen(UsbIsoIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval)
{
	if (!UsbIsoIntrfEpSupported(pIntrf) || Mps == 0U ||
		Mps > USB_ISO_INTRF_MAX_MPS || Interval == 0U)
	{
		return false;
	}

	UsbIsoIntrfClose(pIntrf);

	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bmAttributes = USB_ENDPATT_TRANS_ISO;
	desc.wMaxPacketSize = Mps;
	desc.bInterval = Interval;
	desc.bEndpointAddress = USB_ENDPADDR_DIROUT(pIntrf->EpNo);
	if (!UsbCtrlrEpOpen(pIntrf->DevNo, &desc))
	{
		return false;
	}

	desc.bEndpointAddress = USB_ENDPADDR_DIRIN(pIntrf->EpNo);
	if (!UsbCtrlrEpOpen(pIntrf->DevNo, &desc))
	{
		UsbCtrlrEpClose(pIntrf->DevNo, USB_ENDPADDR_DIROUT(pIntrf->EpNo));
		return false;
	}

	pIntrf->Mps = Mps;
	pIntrf->Interval = Interval;
	pIntrf->Opened = true;
	pIntrf->Suspended = false;
	pIntrf->RxArmed = false;
	pIntrf->TxActive = false;
	pIntrf->TxLength = 0U;

	return true;
}

void UsbIsoIntrfClose(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	if (pIntrf->Opened)
	{
		UsbCtrlrEpClose(pIntrf->DevNo, USB_ENDPADDR_DIROUT(pIntrf->EpNo));
		UsbCtrlrEpClose(pIntrf->DevNo, USB_ENDPADDR_DIRIN(pIntrf->EpNo));
	}

	pIntrf->Opened = false;
	pIntrf->Suspended = false;
	pIntrf->RxArmed = false;
	pIntrf->TxActive = false;
	pIntrf->Mps = 0U;
	pIntrf->Interval = 0U;
	pIntrf->TxLength = 0U;
}

void UsbIsoIntrfReset(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	UsbIsoIntrfClose(pIntrf);
	pIntrf->RxMissCnt = 0U;
	pIntrf->TxMissCnt = 0U;
	pIntrf->RxEmptyCnt = 0U;
	pIntrf->TxEmptyCnt = 0U;
}

void UsbIsoIntrfSuspend(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf != nullptr && pIntrf->Opened)
	{
		pIntrf->Suspended = true;
	}
}

bool UsbIsoIntrfResume(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || !pIntrf->Opened)
	{
		return false;
	}

	pIntrf->Suspended = false;
	return true;
}

bool UsbIsoIntrfSendFrame(UsbIsoIntrf_t *pIntrf, const uint8_t *pData,
						  uint16_t Length)
{
	if (pIntrf == nullptr || !pIntrf->Opened || pIntrf->Suspended ||
		pIntrf->TxActive || Length > pIntrf->Mps ||
		(Length != 0U && pData == nullptr))
	{
		return false;
	}

	if (Length != 0U)
	{
		memcpy(UsbIsoIntrfTxBuffer(pIntrf), pData, Length);
	}

	pIntrf->TxLength = Length;
	pIntrf->TxActive = true;
	if (!UsbCtrlrEpXfer(pIntrf->DevNo, USB_ENDPADDR_DIRIN(pIntrf->EpNo), Length))
	{
		pIntrf->TxActive = false;
		pIntrf->TxLength = 0U;
		pIntrf->TxMissCnt++;
		return false;
	}

	return true;
}
