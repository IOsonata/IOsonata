/**-------------------------------------------------------------------------
@file	usbd_hid.cpp

@brief	Generic USB HID device class implementation.

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
#include <limits.h>
#include <string.h>

#include "usb/usbd_epalloc.h"
#include "usb/usbd_hid.h"

static uint16_t UsbdHidMps(const UsbdHidDev_t *pHid)
{
	return UsbCtrlrHighSpeed(pHid->DevNo) ? pHid->HsMps : pHid->FsMps;
}

static uint8_t UsbdHidInterval(const UsbdHidDev_t *pHid)
{
	return UsbCtrlrHighSpeed(pHid->DevNo) ?
		pHid->HsInterval : pHid->FsInterval;
}

static void UsbdHidRx(UsbIntIntrf_t *, const uint8_t *pData,
					  uint16_t Length, UsbCtrlrXferResult_t Result,
					  void *pContext)
{
	UsbdHidDev_t *pHid = static_cast<UsbdHidDev_t *>(pContext);
	if (pHid != nullptr && pHid->RxHandler != nullptr)
	{
		pHid->RxHandler(pHid, pData, Length, Result, pHid->pContext);
	}
}

static void UsbdHidTx(UsbIntIntrf_t *, uint16_t Length,
					  UsbCtrlrXferResult_t Result, void *pContext)
{
	UsbdHidDev_t *pHid = static_cast<UsbdHidDev_t *>(pContext);
	if (pHid != nullptr && pHid->TxHandler != nullptr)
	{
		pHid->TxHandler(pHid, Length, Result, pHid->pContext);
	}
}

static void UsbdHidUnconfigure(UsbdHidDev_t *pHid)
{
	UsbIntIntrfClose(&pHid->IntIntrf);
	pHid->Configured = false;
}

static bool UsbdHidConfig(uint8_t Configuration, void *pContext)
{
	UsbdHidDev_t *pHid = static_cast<UsbdHidDev_t *>(pContext);
	if (pHid == nullptr)
	{
		return false;
	}

	UsbdHidUnconfigure(pHid);
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != USBD_HID_CONFIG_VALUE ||
		!UsbIntIntrfOpen(&pHid->IntIntrf, UsbdHidMps(pHid),
			UsbdHidInterval(pHid)))
	{
		return false;
	}

	pHid->Configured = true;
	return true;
}

static void UsbdHidReset(void *pContext)
{
	UsbdHidDev_t *pHid = static_cast<UsbdHidDev_t *>(pContext);
	if (pHid != nullptr)
	{
		UsbIntIntrfReset(&pHid->IntIntrf);
		pHid->Configured = false;
		pHid->Idle = 0U;
		pHid->ActiveProtocol = USBD_HID_PROTOCOL_REPORT;
	}
}

static bool UsbdHidInterfaceRequest(const UsbdHidDev_t *pHid,
									const UsbSetupData_t *pSetup)
{
	return pSetup != nullptr &&
		(pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT) ==
			USB_REQTYPE_INTERFACE &&
		(pSetup->wIndex & 0xFF00U) == 0U &&
		(uint8_t)pSetup->wIndex == (uint8_t)pHid->ItfNo;
}

static bool UsbdHidGetDescriptor(UsbdHidDev_t *pHid,
								 const UsbSetupData_t *pSetup,
								 UsbCtrlStage_t Stage, uint8_t **ppData,
								 uint16_t *pLength)
{
	if ((pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) != USB_REQTYPE_DIRHOST ||
		pSetup->bRequest != USB_REQ_GET_DESCRIPTOR || pSetup->wLength == 0U ||
		(uint8_t)pSetup->wValue != 0U)
	{
		return false;
	}
	if (Stage != USB_CTRL_SETUP)
	{
		return true;
	}
	if (ppData == nullptr)
	{
		return false;
	}

	const uint8_t type = (uint8_t)(pSetup->wValue >> 8);
	if (type == USB_DESCTYPE_HID)
	{
		*ppData = reinterpret_cast<uint8_t *>(&pHid->HidDesc);
		*pLength = sizeof(pHid->HidDesc);
		return true;
	}
	if (type == USB_DESCTYPE_HID_REPORT)
	{
		*ppData = const_cast<uint8_t *>(pHid->pReportDesc);
		*pLength = pHid->ReportDescLength;
		return true;
	}
	return false;
}

static bool UsbdHidClassRequest(UsbdHidDev_t *pHid,
								const UsbSetupData_t *pSetup,
								UsbCtrlStage_t Stage, uint8_t **ppData,
								uint16_t *pLength)
{
	const bool dirIn =
		(pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) == USB_REQTYPE_DIRHOST;

	switch (pSetup->bRequest)
	{
		case USB_HID_REQ_GET_REPORT:
			if (!dirIn || pSetup->wLength == 0U ||
				pHid->ReportHandler == nullptr)
			{
				return false;
			}
			return pHid->ReportHandler(pSetup, Stage, ppData, pLength,
				pHid->pReportContext);

		case USB_HID_REQ_SET_REPORT:
			if (dirIn || pSetup->wLength == 0U ||
				pHid->ReportHandler == nullptr)
			{
				return false;
			}
			return pHid->ReportHandler(pSetup, Stage, ppData, pLength,
				pHid->pReportContext);

		case USB_HID_REQ_GET_IDLE:
			if (!dirIn || pSetup->wLength != 1U ||
				(pSetup->wValue & 0xFF00U) != 0U)
			{
				return false;
			}
			if (Stage == USB_CTRL_SETUP)
			{
				if (ppData == nullptr)
				{
					return false;
				}
				pHid->CtrlReply = pHid->Idle;
				*ppData = &pHid->CtrlReply;
				*pLength = 1U;
			}
			return true;

		case USB_HID_REQ_SET_IDLE:
			if (dirIn || pSetup->wLength != 0U)
			{
				return false;
			}
			if (Stage == USB_CTRL_SETUP)
			{
				pHid->PendingIdle = (uint8_t)(pSetup->wValue >> 8);
				pHid->PendingRequest = USB_HID_REQ_SET_IDLE;
			}
			else if (Stage == USB_CTRL_COMPLETE &&
				pHid->PendingRequest == USB_HID_REQ_SET_IDLE)
			{
				pHid->Idle = pHid->PendingIdle;
				pHid->PendingRequest = 0U;
			}
			else if (Stage == USB_CTRL_ABORT)
			{
				pHid->PendingRequest = 0U;
			}
			return true;

		case USB_HID_REQ_GET_PROTOCOL:
			if (!dirIn || pHid->SubClass != USB_HID_SUBCLASS_BOOT ||
				pSetup->wValue != 0U || pSetup->wLength != 1U)
			{
				return false;
			}
			if (Stage == USB_CTRL_SETUP)
			{
				if (ppData == nullptr)
				{
					return false;
				}
				pHid->CtrlReply = pHid->ActiveProtocol;
				*ppData = &pHid->CtrlReply;
				*pLength = 1U;
			}
			return true;

		case USB_HID_REQ_SET_PROTOCOL:
			if (dirIn || pHid->SubClass != USB_HID_SUBCLASS_BOOT ||
				pSetup->wValue > USBD_HID_PROTOCOL_REPORT ||
				pSetup->wLength != 0U)
			{
				return false;
			}
			if (Stage == USB_CTRL_SETUP)
			{
				pHid->PendingProtocol = (uint8_t)pSetup->wValue;
				pHid->PendingRequest = USB_HID_REQ_SET_PROTOCOL;
			}
			else if (Stage == USB_CTRL_COMPLETE &&
				pHid->PendingRequest == USB_HID_REQ_SET_PROTOCOL)
			{
				pHid->ActiveProtocol = pHid->PendingProtocol;
				pHid->PendingRequest = 0U;
			}
			else if (Stage == USB_CTRL_ABORT)
			{
				pHid->PendingRequest = 0U;
			}
			return true;

		default:
			return false;
	}
}

static bool UsbdHidRequest(const UsbSetupData_t *pSetup,
						   UsbCtrlStage_t Stage, uint8_t **ppData,
						   uint16_t *pLength, void *pContext)
{
	UsbdHidDev_t *pHid = static_cast<UsbdHidDev_t *>(pContext);
	if (pHid == nullptr || pLength == nullptr ||
		!UsbdHidInterfaceRequest(pHid, pSetup))
	{
		return false;
	}

	const uint8_t type = pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE;
	if (type == USB_REQTYPE_STANDARD)
	{
		return UsbdHidGetDescriptor(pHid, pSetup, Stage, ppData, pLength);
	}
	if (type == USB_REQTYPE_CLASS)
	{
		return UsbdHidClassRequest(pHid, pSetup, Stage, ppData, pLength);
	}
	return false;
}

static bool UsbdHidFillDesc(UsbdHidDesc_t *pDesc,
							const UsbdHidDev_t *pHid, UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pHid == nullptr || pHid->ItfNo < 0 ||
		pHid->ItfNo > UINT8_MAX || pHid->EpNo == 0U || pHid->EpNo > 15U)
	{
		return false;
	}

	const uint16_t mps = Speed == USB_SPEED_HIGH ? pHid->HsMps : pHid->FsMps;
	const uint8_t interval = Speed == USB_SPEED_HIGH ?
		pHid->HsInterval : pHid->FsInterval;
	if (mps == 0U || mps > USB_INT_INTRF_MAX_MPS || interval == 0U ||
		(Speed == USB_SPEED_FULL && mps > USB_INT_INTRF_FS_MPS) ||
		(Speed == USB_SPEED_HIGH && interval > 16U))
	{
		return false;
	}

	memset(pDesc, 0, sizeof(*pDesc));
	pDesc->Interface.bLength = sizeof(pDesc->Interface);
	pDesc->Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Interface.bInterfaceNumber = (uint8_t)pHid->ItfNo;
	pDesc->Interface.bAlternateSetting = 0U;
	pDesc->Interface.bNumEndpoints = 2U;
	pDesc->Interface.bInterfaceClass = USB_INTRFCLASS_HID;
	pDesc->Interface.bInterfaceSubClass = pHid->SubClass;
	pDesc->Interface.bInterfaceProtocol = pHid->Protocol;
	pDesc->Interface.iInterface = pHid->InterfaceString;
	pDesc->Hid = pHid->HidDesc;

	pDesc->Out.bLength = sizeof(pDesc->Out);
	pDesc->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pHid->EpNo);
	pDesc->Out.bmAttributes = USB_ENDPATT_TRANS_INT;
	pDesc->Out.wMaxPacketSize = mps;
	pDesc->Out.bInterval = interval;
	pDesc->In = pDesc->Out;
	pDesc->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pHid->EpNo);
	return true;
}

bool UsbdHidInit(UsbdHidDev_t *pHid, const UsbdHidCfg_t *pCfg)
{
	if (pHid == nullptr || pCfg == nullptr ||
		UsbGetCfg(pCfg->DevNo) == nullptr || pCfg->pReportDesc == nullptr ||
		pCfg->ReportDescLength == 0U ||
		pCfg->SubClass > USB_HID_SUBCLASS_BOOT ||
		(pCfg->SubClass == USB_HID_SUBCLASS_NONE &&
		 pCfg->Protocol != USB_HID_PROT_NONE) ||
		(pCfg->SubClass == USB_HID_SUBCLASS_BOOT &&
		 pCfg->Protocol != USB_HID_PROT_KEYBOARD &&
		 pCfg->Protocol != USB_HID_PROT_MOUSE))
	{
		return false;
	}

	memset(pHid, 0, sizeof(*pHid));
	pHid->DevNo = pCfg->DevNo;
	pHid->pReportDesc = pCfg->pReportDesc;
	pHid->ReportDescLength = pCfg->ReportDescLength;
	pHid->BcdHid = pCfg->BcdHid != 0U ? pCfg->BcdHid : USBD_HID_BCD_VERSION;
	pHid->FsMps = pCfg->FsMps != 0U ? pCfg->FsMps : USBD_HID_FS_MPS;
	pHid->HsMps = pCfg->HsMps != 0U ? pCfg->HsMps : USBD_HID_HS_MPS;
	pHid->FsInterval = pCfg->FsInterval != 0U ?
		pCfg->FsInterval : USBD_HID_FS_INTERVAL;
	pHid->HsInterval = pCfg->HsInterval != 0U ?
		pCfg->HsInterval : USBD_HID_HS_INTERVAL;
	pHid->SubClass = pCfg->SubClass;
	pHid->Protocol = pCfg->Protocol;
	pHid->CountryCode = pCfg->CountryCode;
	pHid->InterfaceString = pCfg->InterfaceString;
	pHid->ReportHandler = pCfg->ReportHandler;
	pHid->pReportContext = pCfg->pReportContext;
	pHid->RxHandler = pCfg->RxHandler;
	pHid->TxHandler = pCfg->TxHandler;
	pHid->pContext = pCfg->pContext;
	pHid->ActiveProtocol = USBD_HID_PROTOCOL_REPORT;

	if (pHid->FsMps == 0U || pHid->FsMps > USB_INT_INTRF_FS_MPS ||
		pHid->FsMps > USB_INT_INTRF_MAX_MPS || pHid->FsInterval == 0U ||
		(USB_HIGHSPEED_CAPABLE(pHid->DevNo) &&
		 (pHid->HsMps == 0U || pHid->HsMps > USB_INT_INTRF_MAX_MPS ||
		  pHid->HsInterval == 0U || pHid->HsInterval > 16U)))
	{
		return false;
	}

	pHid->HidDesc.bLength = sizeof(pHid->HidDesc);
	pHid->HidDesc.bDescriptorType = USB_DESCTYPE_HID;
	pHid->HidDesc.bcdHID = pHid->BcdHid;
	pHid->HidDesc.bCountryCode = pHid->CountryCode;
	pHid->HidDesc.bNumDescriptors = 1U;
	pHid->HidDesc.RepDesc[0].bDescriptorType = USB_DESCTYPE_HID_REPORT;
	pHid->HidDesc.RepDesc[0].wDescriptorLength = pHid->ReportDescLength;

	UsbdClassCfg_t coreCfg = {};
	coreCfg.RequestHandler = UsbdHidRequest;
	coreCfg.ConfigHandler = UsbdHidConfig;
	coreCfg.ResetHandler = UsbdHidReset;
	coreCfg.pContext = pHid;

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;

	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(pHid->DevNo, &req, &coreCfg, &alloc))
	{
		return false;
	}
	pHid->ItfNo = alloc.FirstInterface;
	pHid->EpNo = alloc.Bidirectional[0];

	UsbIntIntrfCfg_t intCfg = {};
	intCfg.DevNo = pHid->DevNo;
	intCfg.EpNo = pHid->EpNo;
	intCfg.RxHandler = UsbdHidRx;
	intCfg.TxHandler = UsbdHidTx;
	intCfg.pContext = pHid;
	if (!UsbIntIntrfInit(&pHid->IntIntrf, &intCfg))
	{
		return false;
	}

	if (pCfg->pDesc != nullptr)
	{
		const UsbSpeed_t speed = USB_HIGHSPEED_CAPABLE(pHid->DevNo) ?
			USB_SPEED_HIGH : USB_SPEED_FULL;
		if (!UsbdHidFillDesc(pCfg->pDesc, pHid, speed))
		{
			return false;
		}
	}
	return true;
}

bool UsbdHidSendReport(UsbdHidDev_t *pHid, const uint8_t *pData,
					   uint16_t Length)
{
	return pHid != nullptr && pHid->Configured &&
		UsbIntIntrfSendPacket(&pHid->IntIntrf, pData, Length);
}

void UsbdHidSuspend(UsbdHidDev_t *pHid)
{
	if (pHid != nullptr && pHid->Configured)
	{
		UsbIntIntrfSuspend(&pHid->IntIntrf);
	}
}

bool UsbdHidResume(UsbdHidDev_t *pHid)
{
	return pHid != nullptr && pHid->Configured &&
		UsbIntIntrfResume(&pHid->IntIntrf);
}
