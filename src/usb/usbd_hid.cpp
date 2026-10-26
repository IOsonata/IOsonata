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

// Out of line so static instances use zero-filled storage instead of a
// flash initializer containing the entire object and its vtable pointers.
UsbdHid::UsbdHid() = default;

static void UsbdHidUnconfigure(UsbdHidDev_t *pHid)
{
	UsbIntIntrfClose(pHid->pIntIntrf);
	pHid->Configured = false;
	pHid->PendingRequest = 0U;
}

// Out of line for the same reason as UsbdHidInitInternal.
__attribute__((noinline))
static bool UsbdHidConfig(UsbdHidDev_t *pHid, uint8_t Configuration)
{
	UsbdHidUnconfigure(pHid);
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != USBD_HID_CONFIG_VALUE)
	{
		return false;
	}

	const bool highSpeed = UsbCtrlrHighSpeed(pHid->DevNo);
	pHid->Configured = UsbIntIntrfOpen(pHid->pIntIntrf,
		highSpeed ? pHid->HsMps : pHid->FsMps,
		highSpeed ? pHid->HsInterval : pHid->FsInterval);
	return pHid->Configured;
}

static void UsbdHidReset(UsbdHidDev_t *pHid)
{
	UsbIntIntrfReset(pHid->pIntIntrf);
	pHid->Configured = false;
	pHid->Idle = 0U;
	pHid->ActiveProtocol = USBD_HID_PROTOCOL_REPORT;
	pHid->PendingRequest = 0U;
}

static bool UsbdHidReportRequest(UsbdHidDev_t *pHid,
								const UsbSetupData_t *pSetup,
								UsbCtrlStage_t Stage, uint8_t **ppData,
								uint16_t *pLength)
{
	const bool get = pSetup->bRequest == USB_HID_REQ_GET_REPORT;
	const bool dirIn =
		(pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) == USB_REQTYPE_DIRHOST;
	const uint16_t type = pSetup->wValue & USB_HID_REPTYPE_MASK;
	const uint8_t reportId = pSetup->wValue & USB_HID_REPID_MASK;
	if (get != dirIn || pSetup->wLength == 0U ||
		type < USB_HID_REPTYPE_INPUT || type > USB_HID_REPTYPE_FEATURE ||
		(get ? pHid->GetReportCB == nullptr : pHid->SetReportCB == nullptr))
	{
		return false;
	}
	if (Stage == USB_CTRL_SETUP)
	{
		if (ppData == nullptr ||
			(!get && pSetup->wLength > pHid->ControlBufferSize))
		{
			return false;
		}
		int length = pSetup->wLength;
		if (get)
		{
			length = pHid->GetReportCB((USB_HID_REPTYPE)type, reportId,
				pHid->pControlBuffer, pHid->ControlBufferSize, pHid->pContext);
			if (length < 0 || length > pHid->ControlBufferSize)
			{
				return false;
			}
		}
		*ppData = pHid->pControlBuffer;
		*pLength = (uint16_t)length;
		pHid->PendingRequest = pSetup->bRequest;
		pHid->PendingReportValue = pSetup->wValue;
		pHid->PendingReportLength = pSetup->wLength;
		return true;
	}
	if (Stage == USB_CTRL_ABORT || Stage == USB_CTRL_COMPLETE)
	{
		pHid->PendingRequest = 0U;
		return true;
	}
	if (Stage != USB_CTRL_DATA ||
		pHid->PendingRequest != pSetup->bRequest ||
		pHid->PendingReportValue != pSetup->wValue ||
		pHid->PendingReportLength != pSetup->wLength)
	{
		return false;
	}
	if (get)
	{
		return true;
	}
	// Reject a short OUT transfer without presenting a partial report. Clear
	// first so an abort, duplicate notification or callback cannot replay it.
	pHid->PendingRequest = 0U;
	return *pLength == pSetup->wLength &&
		pHid->SetReportCB((USB_HID_REPTYPE)type, reportId,
			pHid->pControlBuffer, *pLength, pHid->pContext);
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
	uint8_t *data;
	uint16_t length;
	if (type == USB_DESCTYPE_HID)
	{
		data = reinterpret_cast<uint8_t *>(&pHid->HidDesc);
		length = sizeof(pHid->HidDesc);
	}
	else if (type == USB_DESCTYPE_HID_REPORT)
	{
		data = const_cast<uint8_t *>(pHid->pReportDesc);
		length = pHid->ReportDescLength;
	}
	else
		return false;
	*ppData = data;
	*pLength = length;
	return true;
}

static bool UsbdHidClassRequest(UsbdHidDev_t *pHid,
								const UsbSetupData_t *pSetup,
								UsbCtrlStage_t Stage, uint8_t **ppData,
								uint16_t *pLength)
{
	const bool dirIn =
		(pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) == USB_REQTYPE_DIRHOST;
	// Get requests reply with *pActive. Set requests stage the new value in
	// *pPending and commit it to *pActive on completion.
	uint8_t *pActive;
	uint8_t *pPending = nullptr;
	uint8_t value = 0U;

	switch (pSetup->bRequest)
	{
		case USB_HID_REQ_GET_REPORT:
		case USB_HID_REQ_SET_REPORT:
			return UsbdHidReportRequest(pHid, pSetup, Stage, ppData, pLength);

		case USB_HID_REQ_GET_IDLE:
			if ((pSetup->wValue & 0xFF00U) != 0U)
			{
				return false;
			}
			pActive = &pHid->Idle;
			break;

		case USB_HID_REQ_SET_IDLE:
			pActive = &pHid->Idle;
			pPending = &pHid->PendingIdle;
			value = (uint8_t)(pSetup->wValue >> 8);
			break;

		case USB_HID_REQ_GET_PROTOCOL:
		case USB_HID_REQ_SET_PROTOCOL:
			if (pHid->SubClass != USB_HID_SUBCLASS_BOOT ||
				pSetup->wValue > USBD_HID_PROTOCOL_REPORT)
				return false;
			pActive = &pHid->ActiveProtocol;
			if (pSetup->bRequest == USB_HID_REQ_GET_PROTOCOL)
			{
				if (pSetup->wValue != 0U)
					return false;
			}
			else
			{
				pPending = &pHid->PendingProtocol;
				value = (uint8_t)pSetup->wValue;
			}
			break;

		default:
			return false;
	}

	if (pPending == nullptr)
	{
		if (!dirIn || pSetup->wLength != 1U)
			return false;
		if (Stage == USB_CTRL_SETUP)
		{
			if (ppData == nullptr)
			{
				return false;
			}
			pHid->CtrlReply = *pActive;
			*ppData = &pHid->CtrlReply;
			*pLength = 1U;
		}
		return true;
	}

	if (dirIn || pSetup->wLength != 0U)
		return false;
	if (Stage == USB_CTRL_SETUP)
	{
		*pPending = value;
		pHid->PendingRequest = pSetup->bRequest;
	}
	else if (Stage == USB_CTRL_COMPLETE &&
		pHid->PendingRequest == pSetup->bRequest)
	{
		*pActive = *pPending;
		pHid->PendingRequest = 0U;
	}
	else if (Stage == USB_CTRL_ABORT)
	{
		pHid->PendingRequest = 0U;
	}
	return true;
}

// Out of line, with state first, for the same short offsets and tail-call
// wrapper used by UsbdHidConfig.
__attribute__((noinline))
static bool UsbdHidRequest(UsbdHidDev_t *pHid, const UsbSetupData_t *pSetup,
						   UsbCtrlStage_t Stage, uint8_t **ppData,
						   uint16_t *pLength)
{
	if (Stage == USB_CTRL_SETUP)
	{
		pHid->PendingRequest = 0U;
	}
	if (pSetup == nullptr || pLength == nullptr ||
		pSetup->wIndex != (uint8_t)pHid->ItfNo)
	{
		return false;
	}

	const uint8_t type = pSetup->bmRequestType &
		(USB_REQTYPE_MASK_TYPE | USB_REQTYPE_MASK_RECIPIENT);
	if (type == (USB_REQTYPE_STANDARD | USB_REQTYPE_INTERFACE))
	{
		return UsbdHidGetDescriptor(pHid, pSetup, Stage, ppData, pLength);
	}
	if (type == (USB_REQTYPE_CLASS | USB_REQTYPE_INTERFACE))
	{
		return UsbdHidClassRequest(pHid, pSetup, Stage, ppData, pLength);
	}
	return false;
}

static constexpr UsbdHidDesc_t UsbdHidDescTemplate(void)
{
	UsbdHidDesc_t desc = {};
	desc.Interface.bLength = sizeof(desc.Interface);
	desc.Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Interface.bNumEndpoints = 2U;
	desc.Interface.bInterfaceClass = USB_INTRFCLASS_HID;
	desc.Out.bLength = sizeof(desc.Out);
	desc.Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.Out.bmAttributes = USB_ENDPATT_TRANS_INT;
	desc.In = desc.Out;
	return desc;
}

static constexpr UsbdHidDesc_t s_HidDescTemplate = UsbdHidDescTemplate();

// Out of line: shared by the registered build hook and UsbdHidMakeDesc.
__attribute__((noinline))
static void UsbdHidPatchDesc(UsbdHidDesc_t *pDesc,
							 const UsbdHidDev_t *pHid, UsbSpeed_t Speed)
{
	const uint16_t mps = Speed == USB_SPEED_HIGH ? pHid->HsMps : pHid->FsMps;
	const uint8_t interval = Speed == USB_SPEED_HIGH ?
		pHid->HsInterval : pHid->FsInterval;

	pDesc->Interface.bInterfaceNumber = (uint8_t)pHid->ItfNo;
	pDesc->Interface.bInterfaceSubClass = pHid->SubClass;
	pDesc->Interface.bInterfaceProtocol = pHid->Protocol;
	pDesc->Interface.iInterface = pHid->InterfaceString;
	pDesc->Hid = pHid->HidDesc;
	pDesc->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pHid->EpNo);
	pDesc->Out.wMaxPacketSize = mps;
	pDesc->Out.bInterval = interval;
	pDesc->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pHid->EpNo);
	pDesc->In.wMaxPacketSize = mps;
	pDesc->In.bInterval = interval;
}

static void UsbdHidPatchRegistered(const UsbDeviceClass *pClass,
									 uint8_t *pDesc, UsbSpeed_t Speed)
{
	const UsbdHidDev_t *pHid = *static_cast<const UsbdHid *>(pClass);
	UsbdHidPatchDesc(reinterpret_cast<UsbdHidDesc_t *>(pDesc), pHid, Speed);
}

// Weak so an application can replace runtime fragment building with a static
// fragment. When overridden, this default is dropped by unused-section removal.
// Pair a replacement with a strong UsbGetDescriptor for fully static
// descriptors, or the assembled configuration will not match.
__attribute__((weak))
bool UsbdHidMakeDesc(UsbdHidDesc_t *pDesc,
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

	memcpy(pDesc, &s_HidDescTemplate, sizeof(*pDesc));
	UsbdHidPatchDesc(pDesc, pHid, Speed);
	return true;
}

static bool UsbdHidBufferOverlap(const uint8_t *pA, uint16_t SizeA,
	const uint8_t *pB, uint16_t SizeB)
{
	const uintptr_t a = (uintptr_t)pA;
	const uintptr_t b = (uintptr_t)pB;
	return a <= b ? b - a < SizeA : a - b < SizeB;
}

// Out of line: the class wrapper becomes a tail call and the body addresses
// the device state with short offsets.
__attribute__((noinline))
static bool UsbdHidInitInternal(UsbdHidDev_t *pHid,
								UsbIntIntrf_t *pIntrf, UsbDevIntrf_t *pData,
								const UsbdHidCfg_t *pCfg,
								uint8_t *pControlBuffer,
								UsbDeviceClass *pClass)
{
	if (UsbGetCfg(pCfg->DevNo) == nullptr || pCfg->pReportDesc == nullptr ||
		pCfg->ReportDescLength == 0U ||
		((pCfg->pControlBuffer == nullptr) != (pCfg->ControlBufferSize == 0U)) ||
		((uintptr_t)pCfg->pControlBuffer & 3U) != 0U ||
		pCfg->pRxBuffer == nullptr || pCfg->pTxBuffer == nullptr ||
		(((uintptr_t)pCfg->pRxBuffer | (uintptr_t)pCfg->pTxBuffer) & 3U) != 0U ||
		(pCfg->pControlBuffer != nullptr &&
		 (UsbdHidBufferOverlap(pCfg->pControlBuffer, pCfg->ControlBufferSize,
			pCfg->pRxBuffer, USB_INT_INTRF_PKT_BLKSIZE) ||
		  UsbdHidBufferOverlap(pCfg->pControlBuffer, pCfg->ControlBufferSize,
			pCfg->pTxBuffer, USB_INT_INTRF_PKT_BLKSIZE))) ||
		pCfg->SubClass > USB_HID_SUBCLASS_BOOT ||
		(pCfg->SubClass == USB_HID_SUBCLASS_NONE &&
		 pCfg->Protocol != USB_HID_PROT_NONE) ||
		(pCfg->SubClass == USB_HID_SUBCLASS_BOOT &&
		 (uint32_t)pCfg->Protocol - USB_HID_PROT_KEYBOARD >
		 USB_HID_PROT_MOUSE - USB_HID_PROT_KEYBOARD))
	{
		return false;
	}

	memset(pHid, 0, sizeof(*pHid));
	pHid->pIntIntrf = pIntrf;
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
	pHid->pContext = pCfg->pContext;
	pHid->GetReportCB = pCfg->GetReportCB;
	pHid->SetReportCB = pCfg->SetReportCB;
	pHid->pControlBuffer = pCfg->pControlBuffer != nullptr ?
		pCfg->pControlBuffer : pControlBuffer;
	pHid->ControlBufferSize = pCfg->pControlBuffer != nullptr ?
		pCfg->ControlBufferSize : USBD_HID_CTRL_REPORT_SIZE;
	pHid->ActiveProtocol = USBD_HID_PROTOCOL_REPORT;

	// Zero MPS and interval values were replaced by the defaults above.
	if (pHid->FsMps > USB_INT_INTRF_FS_MPS ||
		pHid->FsMps > USB_INT_INTRF_MAX_MPS ||
		(USB_HIGHSPEED_CAPABLE(pHid->DevNo) &&
		 (pHid->HsMps > USB_INT_INTRF_MAX_MPS || pHid->HsInterval > 16U)))
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

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;
	req.BidirectionalType = USB_ENDPATT_TRANS_INT;

	// The allocator fills each requested result before returning success.
	UsbdEpAllocRes_t alloc;
	if (!UsbdEpAlloc(pHid->DevNo, &req, pClass, &alloc))
	{
		return false;
	}
	pHid->ItfNo = alloc.FirstInterface;
	pHid->EpNo = alloc.Bidirectional[0];

	// Every configuration member is assigned below.
	UsbIntIntrfCfg_t intCfg;
	intCfg.DevNo = pHid->DevNo;
	intCfg.EpNo = pHid->EpNo;
	intCfg.EvtCB = pCfg->EvtCB;
	intCfg.pContext = pHid;
	intCfg.pRxBuffer = pCfg->pRxBuffer;
	intCfg.pTxBuffer = pCfg->pTxBuffer;
	if (!UsbIntIntrfInit(pHid->pIntIntrf, pData, &intCfg))
	{
		return false;
	}

	return UsbDescRegister(pHid->DevNo, pClass,
		&s_HidDescTemplate, sizeof(s_HidDescTemplate),
		UsbdHidPatchRegistered);
}

bool UsbdHid::Init(const UsbdHidCfg_t &Cfg)
{
	return UsbdHidInitInternal(&vUsbdHid, &vUsbIntIntrf,
		&vUsbDevIntrf, &Cfg, vControlBuffer, this);
}

bool UsbdHid::Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
					   uint8_t **ppData, uint16_t *pLength)
{
	return UsbdHidRequest(&vUsbdHid, pSetup, Stage, ppData, pLength);
}

bool UsbdHid::SelectConfig(uint8_t ConfigValue)
{
	return UsbdHidConfig(&vUsbdHid, ConfigValue);
}


void UsbdHid::Reset(void)
{
	UsbdHidReset(&vUsbdHid);
}


