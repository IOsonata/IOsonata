/**-------------------------------------------------------------------------
@file	usb_iso.cpp

@brief	USB isochronous specialization of UsbIntrf.

@author	Hoang Nguyen Hoan
@date	Sep. 8, 2026

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

static UsbPkt_t *UsbIsoIntrfTxPacket(UsbIsoIntrf_t *pIntrf)
{
	return reinterpret_cast<UsbPkt_t *>(pIntrf->TxPacket);
}

static bool UsbIsoIntrfEpSupported(int DevNo, uint8_t EpNo)
{
	if (DevNo < 0 || DevNo >= USB_CTRLR_CNT || EpNo == 0U || EpNo > 15U ||
		!USB_ISO_SUPPORTED(DevNo))
	{
		return false;
	}

	const uint16_t bit = (uint16_t)(1U << EpNo);
	return (USB_ISO_EPIN_MASK(DevNo) & bit) != 0U &&
		(USB_ISO_EPOUT_MASK(DevNo) & bit) != 0U;
}

static bool UsbIsoIntrfOpenEndpoint(UsbIsoIntrf_t *pIntrf, uint8_t EpAddr)
{
	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bEndpointAddress = EpAddr;
	desc.bmAttributes = USB_ENDPATT_TRANS_ISO | pIntrf->Attributes;
	desc.wMaxPacketSize = pIntrf->Mps;
	desc.bInterval = pIntrf->Interval;

	return UsbCtrlrEpOpen(pIntrf->pIntrfData->DevNo, &desc);
}

static void UsbIsoIntrfSetTxIdle(UsbDevIntrf_t *pData)
{
	atomic_store_explicit(&pData->DevIntrf.bTxReady, true,
						  memory_order_release);
}

// UsbIntrf remains the controller endpoint owner. This callback is only the
// DeviceIntrf-facing ISO adapter above UsbIntrf; it never receives controller
// events directly and never submits an endpoint transfer itself.
static int UsbIsoIntrfDataEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
								uint8_t *, int Length)
{
	UsbDevIntrf_t *pData = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	UsbIsoIntrf_t *pIntrf = pData != nullptr ?
		static_cast<UsbIsoIntrf_t *>(pData->pClassContext) : nullptr;
	if (pIntrf == nullptr)
	{
		return 0;
	}

	switch (Event)
	{
		case DEVINTRF_EVT_RX_DATA:
		{
			UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(
				CFifoPeek(pData->hRxFifo));
			if (pPacket == nullptr)
			{
				return 0;
			}

			const uint16_t length = pPacket->Hdr.Length;
			UsbCtrlrXferResult_t result = USB_CTRLR_XFER_SUCCESS;
			if (length > pIntrf->Mps)
			{
				result = USB_CTRLR_XFER_FAILED;
				pIntrf->RxMissCnt++;
			}
			else if (length == 0U)
			{
				pIntrf->RxEmptyCnt++;
			}

			if (pIntrf->RxHandler != nullptr)
			{
				pIntrf->RxHandler(pIntrf, pPacket->Data, length, result,
					pIntrf->pContext);
			}
			(void)CFifoGet(pData->hRxFifo);
			return length;
		}

		case DEVINTRF_EVT_RX_TIMEOUT:
			pIntrf->RxMissCnt++;
			if (pIntrf->RxHandler != nullptr)
			{
				pIntrf->RxHandler(pIntrf, pData->pRxBuffer,
					Length > 0 ? (uint16_t)Length : 0U,
					USB_CTRLR_XFER_FAILED, pIntrf->pContext);
			}
			return 0;

		case DEVINTRF_EVT_TX_FIFO_EMPTY:
		{
			if (!pIntrf->TxActive)
			{
				return 0;
			}

			const uint16_t length = pIntrf->TxLength;
			pIntrf->TxActive = false;
			pIntrf->TxLength = 0U;
			if (length == 0U)
			{
				pIntrf->TxEmptyCnt++;
			}
			if (pIntrf->TxHandler != nullptr)
			{
				pIntrf->TxHandler(pIntrf, length,
					USB_CTRLR_XFER_SUCCESS, pIntrf->pContext);
			}
			return length;
		}

		case DEVINTRF_EVT_TX_TIMEOUT:
		{
			const uint16_t length = pIntrf->TxLength;
			pIntrf->TxActive = false;
			pIntrf->TxLength = 0U;
			pIntrf->TxMissCnt++;
			CFifoFlush(pData->hTxFifo);
			UsbIsoIntrfSetTxIdle(pData);
			if (pIntrf->TxHandler != nullptr)
			{
				pIntrf->TxHandler(pIntrf,
					Length > 0 ? (uint16_t)Length : length,
					USB_CTRLR_XFER_FAILED, pIntrf->pContext);
			}
			return 0;
		}

		default:
			return 0;
	}
}

bool UsbIsoIntrfInitData(UsbIsoIntrf_t *pIntrf, UsbDevIntrf_t *pData,
						 const UsbIsoIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pData == nullptr || pCfg == nullptr ||
		USB_ISO_INTRF_MAX_MPS == 0U ||
		!UsbIsoIntrfEpSupported(pCfg->DevNo, pCfg->EpNo))
	{
		return false;
	}

	memset(pIntrf, 0, sizeof(*pIntrf));
	pIntrf->pIntrfData = pData;
	pIntrf->pContext = pCfg->pContext;
	pIntrf->RxHandler = pCfg->RxHandler;
	pIntrf->TxHandler = pCfg->TxHandler;
	pIntrf->EpNo = pCfg->EpNo;
	pIntrf->Attributes = pCfg->Attributes &
		(USB_ENDPATT_ISO_SYNC_MASK | USB_ENDPATT_ISO_USAGE_MASK);

	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = pCfg->DevNo;
	cfg.EpNo = pCfg->EpNo;
	cfg.bBlocking = false;
	cfg.RxFifoMemSize = (int)sizeof(pIntrf->RxFifoMem);
	cfg.pRxFifoMem = reinterpret_cast<uint8_t *>(pIntrf->RxFifoMem);
	cfg.TxFifoMemSize = (int)sizeof(pIntrf->TxFifoMem);
	cfg.pTxFifoMem = reinterpret_cast<uint8_t *>(pIntrf->TxFifoMem);
	cfg.TxFifoBlkSize = USB_ISO_INTRF_PKT_BLKSIZE;
	cfg.BufferSize = USB_ISO_INTRF_MAX_MPS;
	cfg.pRxBuffer = UsbIsoIntrfRxBuffer(pIntrf);
	cfg.pTxBuffer = UsbIsoIntrfTxBuffer(pIntrf);
	cfg.EvtCB = UsbIsoIntrfDataEvent;

	if (!UsbIntrfInit(pData, &cfg))
	{
		pIntrf->pIntrfData = nullptr;
		return false;
	}

	pData->pClassContext = pIntrf;
	return true;
}

bool UsbIsoIntrfInit(UsbIsoIntrf_t *pIntrf, const UsbIsoIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	return UsbIsoIntrfInitData(pIntrf, &pIntrf->LocalData, pCfg);
}

bool UsbIsoIntrfOpen(UsbIsoIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval)
{
	if (pIntrf == nullptr || pIntrf->pIntrfData == nullptr ||
		!UsbIsoIntrfEpSupported(pIntrf->pIntrfData->DevNo, pIntrf->EpNo) ||
		Mps == 0U || Mps > USB_ISO_INTRF_MAX_MPS || Interval == 0U)
	{
		return false;
	}

	UsbIsoIntrfClose(pIntrf);
	if (!UsbIntrfConfigure(pIntrf->pIntrfData, Mps))
	{
		return false;
	}

	pIntrf->Mps = Mps;
	pIntrf->Interval = Interval;
	pIntrf->Suspended = false;
	pIntrf->TxActive = false;
	pIntrf->TxLength = 0U;

	if (!UsbIsoIntrfOpenEndpoint(pIntrf, USB_ENDPADDR_DIRIN(pIntrf->EpNo)) ||
		!UsbIsoIntrfOpenEndpoint(pIntrf, USB_ENDPADDR_DIROUT(pIntrf->EpNo)))
	{
		UsbCtrlrEpClose(pIntrf->pIntrfData->DevNo,
			USB_ENDPADDR_DIROUT(pIntrf->EpNo));
		UsbCtrlrEpClose(pIntrf->pIntrfData->DevNo,
			USB_ENDPADDR_DIRIN(pIntrf->EpNo));
		UsbIntrfUnconfigure(pIntrf->pIntrfData);
		pIntrf->Mps = 0U;
		pIntrf->Interval = 0U;
		return false;
	}

	pIntrf->Opened = true;
	return true;
}

void UsbIsoIntrfClose(UsbIsoIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->pIntrfData == nullptr)
	{
		return;
	}

	if (pIntrf->Opened)
	{
		UsbCtrlrEpClose(pIntrf->pIntrfData->DevNo,
			USB_ENDPADDR_DIROUT(pIntrf->EpNo));
		UsbCtrlrEpClose(pIntrf->pIntrfData->DevNo,
			USB_ENDPADDR_DIRIN(pIntrf->EpNo));
	}

	UsbIntrfUnconfigure(pIntrf->pIntrfData);
	pIntrf->Opened = false;
	pIntrf->Suspended = false;
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
	if (pIntrf == nullptr || pIntrf->pIntrfData == nullptr ||
		!pIntrf->Opened || pIntrf->Suspended || pIntrf->TxActive ||
		Length > pIntrf->Mps || (Length != 0U && pData == nullptr))
	{
		return false;
	}

	UsbPkt_t *pPacket = UsbIsoIntrfTxPacket(pIntrf);
	pPacket->Hdr.Length = Length;
	pPacket->Hdr.Reserved = 0U;
	if (Length > 0U)
	{
		memcpy(pPacket->Data, pData, Length);
	}

	pIntrf->TxLength = Length;
	pIntrf->TxActive = true;
	const int queued = DeviceIntrfTxData(&pIntrf->pIntrfData->DevIntrf,
		reinterpret_cast<const uint8_t *>(pPacket),
		(int)USB_ISO_INTRF_PKT_BLKSIZE);
	if (queued != (int)USB_ISO_INTRF_PKT_BLKSIZE)
	{
		pIntrf->TxActive = false;
		pIntrf->TxLength = 0U;
		pIntrf->TxMissCnt++;
		return false;
	}

	return true;
}
