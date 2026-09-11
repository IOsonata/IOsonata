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

bool UsbdCdcMakeDesc(UsbdCdcDesc_t *pDesc, const UsbdCdcDev_t *pCdc,
					 UsbSpeed_t Speed, bool HasFunctionString)
{
	if (pDesc == nullptr || pCdc == nullptr || pCdc->CtrlIfNo > 14U ||
		pCdc->NotifyEpNo == 0U || pCdc->NotifyEpNo > 15U ||
		pCdc->DataEpNo == 0U || pCdc->DataEpNo > 15U)
	{
		return false;
	}

	const uint8_t control = pCdc->CtrlIfNo;
	const uint8_t data = (uint8_t)(control + 1U);
	const uint16_t bulkMps = Speed == USB_SPEED_HIGH ?
		USBD_CDC_BULK_HS_MPS : USBD_CDC_BULK_FS_MPS;
	const uint8_t interval = Speed == USB_SPEED_HIGH ?
		USBD_CDC_NOTIF_INTERVAL_HS : USBD_CDC_NOTIF_INTERVAL_FS;

	memset(pDesc, 0, sizeof(*pDesc));

	pDesc->Association.bLength = sizeof(pDesc->Association);
	pDesc->Association.bDescriptorType = USB_DESCTYPE_IA;
	pDesc->Association.bFirstInterface = control;
	pDesc->Association.bInterfaceCount = 2U;
	pDesc->Association.bFunctionClass = USB_INTRFCLASS_CDC;
	pDesc->Association.bFunctionSubClass = USB_CDC_SUBCLASS_ACM;
	pDesc->Association.bFunctionProtocol = USB_CDC_PROT_NONE;
	pDesc->Association.iFunction = HasFunctionString ? 4U : 0U;

	pDesc->Control.bLength = sizeof(pDesc->Control);
	pDesc->Control.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Control.bInterfaceNumber = control;
	pDesc->Control.bAlternateSetting = 0U;
	pDesc->Control.bNumEndpoints = 1U;
	pDesc->Control.bInterfaceClass = USB_INTRFCLASS_CDC;
	pDesc->Control.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM;
	pDesc->Control.bInterfaceProtocol = USB_CDC_PROT_NONE;
	pDesc->Control.iInterface = HasFunctionString ? 4U : 0U;

	pDesc->Header.bFunctionLength = sizeof(pDesc->Header);
	pDesc->Header.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	pDesc->Header.bDescriptorSubtype = USB_CDC_FSUBTYPE_HEADER;
	pDesc->Header.bcdCDC = 0x0120U;

	pDesc->CallManagement.bFunctionLength = sizeof(pDesc->CallManagement);
	pDesc->CallManagement.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	pDesc->CallManagement.bDescriptorSubtype = USB_CDC_FSUBTYPE_CM;
	pDesc->CallManagement.bDataInterface = data;

	pDesc->Acm.bFunctionLength = sizeof(pDesc->Acm);
	pDesc->Acm.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	pDesc->Acm.bDescriptorSubtype = USB_CDC_FSUBTYPE_ACM;
	pDesc->Acm.bmCapabilities = USB_CDC_ACM_CAP_LINE_CODING;

	pDesc->Union.bFunctionLength = sizeof(pDesc->Union);
	pDesc->Union.bDescriptorType = USB_FUNCTYPE_CS_INTERFACE;
	pDesc->Union.bDescriptorSubtype = USB_CDC_FSUBTYPE_UNION;
	pDesc->Union.bControlInterface = control;
	pDesc->Union.bSubordinateInterf[0] = data;

	pDesc->Notification.bLength = sizeof(pDesc->Notification);
	pDesc->Notification.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->Notification.bEndpointAddress =
		USB_ENDPADDR_DIRIN(pCdc->NotifyEpNo);
	pDesc->Notification.bmAttributes = USB_ENDPATT_TRANS_INT;
	pDesc->Notification.wMaxPacketSize = USBD_CDC_NOTIF_MPS;
	pDesc->Notification.bInterval = interval;

	pDesc->Data.bLength = sizeof(pDesc->Data);
	pDesc->Data.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Data.bInterfaceNumber = data;
	pDesc->Data.bAlternateSetting = 0U;
	pDesc->Data.bNumEndpoints = 2U;
	pDesc->Data.bInterfaceClass = USB_INTRFCLASS_CDCDATA;
	pDesc->Data.bInterfaceProtocol = USB_CDCDATA_PROT_NONE;

	pDesc->Out.bLength = sizeof(pDesc->Out);
	pDesc->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pCdc->DataEpNo);
	pDesc->Out.bmAttributes = USB_ENDPATT_TRANS_BULK;
	pDesc->Out.wMaxPacketSize = bulkMps;

	pDesc->In = pDesc->Out;
	pDesc->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pCdc->DataEpNo);
	return true;
}
