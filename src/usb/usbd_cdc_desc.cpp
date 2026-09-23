/**-------------------------------------------------------------------------
@file	usbd_cdc_desc.cpp

@brief	USB CDC ACM configuration descriptor fragment builder.

The generic USB layer owns device, configuration and string descriptors.
This file builds the static per-instance CDC fragment registered by UsbdCdc.

@author	Hoang Nguyen Hoan
@date	Sep. 11, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <string.h>

#include "usb/usbd_cdc.h"

#define USBD_CDC_NOTIF_INTERVAL_FS		16U
#define USBD_CDC_NOTIF_INTERVAL_HS		8U

static_assert(sizeof(UsbInrtfAssDesc_t) == 8U, "USB IAD size");
static_assert(sizeof(UsbIntrfDesc_t) == 9U, "USB interface descriptor size");
static_assert(sizeof(UsbEndPointDesc_t) == 7U, "USB endpoint descriptor size");

static constexpr UsbdCdcDesc_t UsbdCdcDescTemplate(void)
{
	UsbdCdcDesc_t desc = {};

	desc.Association.bLength = sizeof(desc.Association);
	desc.Association.bDescriptorType = USB_DESCTYPE_IA;
	desc.Association.bInterfaceCount = 2U;
	desc.Association.bFunctionClass = USB_INTRFCLASS_CDC;
	desc.Association.bFunctionSubClass = USB_CDC_SUBCLASS_ACM;
	desc.Association.bFunctionProtocol = USB_CDC_PROT_NONE;

	desc.Control.bLength = sizeof(desc.Control);
	desc.Control.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Control.bNumEndpoints = 1U;
	desc.Control.bInterfaceClass = USB_INTRFCLASS_CDC;
	desc.Control.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM;
	desc.Control.bInterfaceProtocol = USB_CDC_PROT_NONE;

	desc.Header.bFunctionLength = sizeof(desc.Header);
	desc.Header.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	desc.Header.bDescriptorSubtype = USB_CDC_FSUBTYPE_HEADER;
	desc.Header.bcdCDC = 0x0120U;

	desc.CallManagement.bFunctionLength = sizeof(desc.CallManagement);
	desc.CallManagement.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	desc.CallManagement.bDescriptorSubtype = USB_CDC_FSUBTYPE_CM;

	desc.Acm.bFunctionLength = sizeof(desc.Acm);
	desc.Acm.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	desc.Acm.bDescriptorSubtype = USB_CDC_FSUBTYPE_ACM;
	desc.Acm.bmCapabilities = USB_CDC_ACM_CAP_LINE_CODING;

	desc.Union.bFunctionLength = sizeof(desc.Union);
	desc.Union.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	desc.Union.bDescriptorSubtype = USB_CDC_FSUBTYPE_UNION;

	desc.Notification.bLength = sizeof(desc.Notification);
	desc.Notification.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.Notification.bmAttributes = USB_ENDPATT_TRANS_INT;
	desc.Notification.wMaxPacketSize = USBD_CDC_NOTIF_MPS;

	desc.Data.bLength = sizeof(desc.Data);
	desc.Data.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Data.bNumEndpoints = 2U;
	desc.Data.bInterfaceClass = USB_INTRFCLASS_CDCDATA;
	desc.Data.bInterfaceProtocol = USB_CDCDATA_PROT_NONE;

	desc.Out.bLength = sizeof(desc.Out);
	desc.Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.Out.bmAttributes = USB_ENDPATT_TRANS_BULK;

	desc.In = desc.Out;
	return desc;
}

extern const UsbdCdcDesc_t g_UsbdCdcDescTemplate = UsbdCdcDescTemplate();

void UsbdCdcPatchDesc(UsbdCdcDesc_t *pDesc, const UsbdCdcDev_t *pCdc,
					 UsbSpeed_t Speed, bool HasFunctionString)
{
	const uint8_t control = pCdc->CtrlIfNo;
	const uint8_t data = (uint8_t)(control + 1U);
	const uint16_t bulkMps = Speed == USB_SPEED_HIGH ?
		USBD_CDC_BULK_HS_MPS : USBD_CDC_BULK_FS_MPS;
	const uint8_t interval = Speed == USB_SPEED_HIGH ?
		USBD_CDC_NOTIF_INTERVAL_HS : USBD_CDC_NOTIF_INTERVAL_FS;
	const uint8_t stringIndex = HasFunctionString ? 4U : 0U;

	pDesc->Association.bFirstInterface = control;
	pDesc->Association.iFunction = stringIndex;
	pDesc->Control.bInterfaceNumber = control;
	pDesc->Control.iInterface = stringIndex;
	pDesc->CallManagement.bDataInterface = data;
	pDesc->Union.bControlInterface = control;
	pDesc->Union.bSubordinateInterf[0] = data;
	pDesc->Notification.bEndpointAddress = USB_ENDPADDR_DIRIN(pCdc->NotifyEpNo);
	pDesc->Notification.bInterval = interval;
	pDesc->Data.bInterfaceNumber = data;
	pDesc->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pCdc->DataEpNo);
	pDesc->Out.wMaxPacketSize = bulkMps;
	pDesc->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pCdc->DataEpNo);
	pDesc->In.wMaxPacketSize = bulkMps;
}

// Weak so an application can replace runtime fragment building with a static
// fragment. When overridden, this default is dropped by unused-section removal.
// Pair a replacement here with a strong UsbGetDescriptor when building fully
// static descriptors, otherwise the assembled configuration will not match.
__attribute__((weak))
bool UsbdCdcMakeDesc(UsbdCdcDesc_t *pDesc, const UsbdCdcDev_t *pCdc,
					 UsbSpeed_t Speed, bool HasFunctionString)
{
	if (pDesc == nullptr || pCdc == nullptr)
	{
		return false;
	}

	memcpy(pDesc, &g_UsbdCdcDescTemplate, sizeof(*pDesc));
	UsbdCdcPatchDesc(pDesc, pCdc, Speed, HasFunctionString);
	return true;
}
