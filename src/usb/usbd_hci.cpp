/**-------------------------------------------------------------------------
@file	usbd_hci.cpp

@brief	USB Bluetooth HCI device function implementation.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

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

#include "usb_func.h"
#include "usb/usbd_hci.h"

static uint16_t UsbdHciEventMps(const UsbdHciDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ? pHci->EventHsMps : pHci->EventFsMps;
}

static uint16_t UsbdHciAclMps(const UsbdHciDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ? pHci->AclHsMps : pHci->AclFsMps;
}

static uint8_t UsbdHciEventInterval(const UsbdHciDev_t *pHci)
{
	return UsbCtrlrHighSpeed(pHci->DevNo) ?
		pHci->EventHsInterval : pHci->EventFsInterval;
}

static bool UsbdHciOpenEndpoint(UsbdHciDev_t *pHci, uint8_t EpAddr,
								 uint8_t TransferType, uint16_t Mps,
								 uint8_t Interval)
{
	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bEndpointAddress = EpAddr;
	desc.bmAttributes = TransferType;
	desc.wMaxPacketSize = Mps;
	desc.bInterval = Interval;

	return UsbCtrlrEpOpen(pHci->DevNo, &desc);
}

static void UsbdHciCloseEndpoints(UsbdHciDev_t *pHci)
{
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->EventEpNo));
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIROUT(pHci->AclEpNo));
	UsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->AclEpNo));
}

static bool UsbdHciConfig(uint8_t Configuration, void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);

	if (pHci == nullptr)
	{
		return false;
	}
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != USBD_HCI_CONFIG_VALUE)
	{
		return false;
	}

	const uint16_t eventMps = UsbdHciEventMps(pHci);
	const uint16_t aclMps = UsbdHciAclMps(pHci);
	const uint8_t eventInterval = UsbdHciEventInterval(pHci);

	if (!UsbdHciOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->EventEpNo),
							 USB_ENDPATT_TRANS_INT, eventMps, eventInterval) ||
		!UsbdHciOpenEndpoint(pHci, USB_ENDPADDR_DIROUT(pHci->AclEpNo),
							 USB_ENDPATT_TRANS_BULK, aclMps, 0U) ||
		!UsbdHciOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->AclEpNo),
							 USB_ENDPATT_TRANS_BULK, aclMps, 0U))
	{
		UsbdHciCloseEndpoints(pHci);
		return false;
	}

	return true;
}

static bool UsbdHciSetInterface(uint8_t InterfaceNo, uint8_t Alt,
								void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);

	return pHci != nullptr && Alt == 0U &&
		(InterfaceNo == (uint8_t)pHci->HciItfNo ||
		 InterfaceNo == (uint8_t)pHci->SyncItfNo);
}

static bool UsbdHciRequest(const UsbSetupData_t *pSetup,
						   UsbCtrlStage_t Stage, uint8_t **ppData,
						   uint16_t *pLength, void *pContext)
{
	UsbdHciDev_t *pHci = static_cast<UsbdHciDev_t *>(pContext);

	if (pHci == nullptr || pHci->RequestHandler == nullptr || pSetup == nullptr)
	{
		return false;
	}

	const uint8_t recipient =
		pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT;
	if (recipient == USB_REQTYPE_INTERFACE)
	{
		if ((pSetup->wIndex & 0xFF00U) != 0U ||
			(uint8_t)pSetup->wIndex != (uint8_t)pHci->HciItfNo)
		{
			return false;
		}
	}
	else if (recipient != USB_REQTYPE_DEVICE)
	{
		return false;
	}

	return pHci->RequestHandler(pSetup, Stage, ppData, pLength,
							   pHci->pRequestContext);
}

static void UsbdHciReset(void *pContext)
{
	(void)pContext;
}

bool UsbdHciMakeDesc(UsbdHciDesc_t *pDesc, const UsbdHciDev_t *pHci,
					 UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pHci == nullptr ||
		pHci->HciItfNo < 0 || pHci->HciItfNo > UINT8_MAX - 1 ||
		pHci->SyncItfNo != pHci->HciItfNo + 1 ||
		pHci->EventEpNo == 0U || pHci->EventEpNo > 15U ||
		pHci->AclEpNo == 0U || pHci->AclEpNo > 15U)
	{
		return false;
	}

	const uint16_t eventMps = Speed == USB_SPEED_HIGH ?
		pHci->EventHsMps : pHci->EventFsMps;
	const uint16_t aclMps = Speed == USB_SPEED_HIGH ?
		pHci->AclHsMps : pHci->AclFsMps;
	const uint8_t eventInterval = Speed == USB_SPEED_HIGH ?
		pHci->EventHsInterval : pHci->EventFsInterval;

	if (eventMps == 0U || eventMps > USB_PKT_MAXLEN(0, INT) ||
		aclMps == 0U || aclMps > USB_PKT_MAXLEN(0, BULK) ||
		eventInterval == 0U)
	{
		return false;
	}

	memset(pDesc, 0, sizeof(*pDesc));

	pDesc->Association.bLength = sizeof(pDesc->Association);
	pDesc->Association.bDescriptorType = USB_DESCTYPE_IA;
	pDesc->Association.bFirstInterface = (uint8_t)pHci->HciItfNo;
	pDesc->Association.bInterfaceCount = 2U;
	pDesc->Association.bFunctionClass = USB_INTRFCLASS_WIRELESS;
	pDesc->Association.bFunctionSubClass = USBD_HCI_SUBCLASS_RF;
	pDesc->Association.bFunctionProtocol = USBD_HCI_PROTOCOL_BT;
	pDesc->Association.iFunction = pHci->InterfaceString;

	pDesc->Hci.bLength = sizeof(pDesc->Hci);
	pDesc->Hci.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Hci.bInterfaceNumber = (uint8_t)pHci->HciItfNo;
	pDesc->Hci.bAlternateSetting = 0U;
	pDesc->Hci.bNumEndpoints = 3U;
	pDesc->Hci.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
	pDesc->Hci.bInterfaceSubClass = USBD_HCI_SUBCLASS_RF;
	pDesc->Hci.bInterfaceProtocol = USBD_HCI_PROTOCOL_BT;
	pDesc->Hci.iInterface = pHci->InterfaceString;

	pDesc->EventIn.bLength = sizeof(pDesc->EventIn);
	pDesc->EventIn.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->EventIn.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->EventEpNo);
	pDesc->EventIn.bmAttributes = USB_ENDPATT_TRANS_INT;
	pDesc->EventIn.wMaxPacketSize = eventMps;
	pDesc->EventIn.bInterval = eventInterval;

	pDesc->AclOut.bLength = sizeof(pDesc->AclOut);
	pDesc->AclOut.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->AclOut.bEndpointAddress = USB_ENDPADDR_DIROUT(pHci->AclEpNo);
	pDesc->AclOut.bmAttributes = USB_ENDPATT_TRANS_BULK;
	pDesc->AclOut.wMaxPacketSize = aclMps;
	pDesc->AclOut.bInterval = 0U;

	pDesc->AclIn = pDesc->AclOut;
	pDesc->AclIn.bEndpointAddress = USB_ENDPADDR_DIRIN(pHci->AclEpNo);

	pDesc->Sync.bLength = sizeof(pDesc->Sync);
	pDesc->Sync.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Sync.bInterfaceNumber = (uint8_t)pHci->SyncItfNo;
	pDesc->Sync.bAlternateSetting = 0U;
	pDesc->Sync.bNumEndpoints = 0U;
	pDesc->Sync.bInterfaceClass = USB_INTRFCLASS_WIRELESS;
	pDesc->Sync.bInterfaceSubClass = USBD_HCI_SUBCLASS_RF;
	pDesc->Sync.bInterfaceProtocol = USBD_HCI_PROTOCOL_BT;
	pDesc->Sync.iInterface = pHci->InterfaceString;

	return true;
}

bool UsbdHciInit(UsbdHciDev_t * const pHci, const UsbdHciCfg_t *pCfg)
{
	if (pHci == nullptr || pCfg == nullptr || UsbGetCfg(pCfg->DevNo) == nullptr)
	{
		return false;
	}

	memset(pHci, 0, sizeof(*pHci));
	pHci->RequestHandler = pCfg->RequestHandler;
	pHci->pRequestContext = pCfg->pRequestContext;
	pHci->DevNo = pCfg->DevNo;
	pHci->InterfaceString = pCfg->InterfaceString;
	pHci->EventFsMps = pCfg->EventFsMps != 0U ?
		pCfg->EventFsMps : USBD_HCI_EVENT_FS_MPS;
	pHci->EventHsMps = pCfg->EventHsMps != 0U ?
		pCfg->EventHsMps : USBD_HCI_EVENT_HS_MPS;
	pHci->AclFsMps = pCfg->AclFsMps != 0U ?
		pCfg->AclFsMps : USBD_HCI_ACL_FS_MPS;
	pHci->AclHsMps = pCfg->AclHsMps != 0U ?
		pCfg->AclHsMps : USBD_HCI_ACL_HS_MPS;
	pHci->EventFsInterval = pCfg->EventFsInterval != 0U ?
		pCfg->EventFsInterval : USBD_HCI_EVENT_FS_INTERVAL;
	pHci->EventHsInterval = pCfg->EventHsInterval != 0U ?
		pCfg->EventHsInterval : USBD_HCI_EVENT_HS_INTERVAL;

	if (pHci->EventFsMps > USB_PKT_MAXLEN(0, INT) ||
		pHci->AclFsMps > USB_PKT_MAXLEN(0, BULK) ||
		(USB_HIGHSPEED_CAPABLE(0) &&
		 (pHci->EventHsMps > USB_PKT_MAXLEN(0, INT) ||
		  pHci->AclHsMps > USB_PKT_MAXLEN(0, BULK))))
	{
		return false;
	}

	UsbFuncCfg_t coreCfg = {};
	coreCfg.RequestHandler = pCfg->RequestHandler != nullptr ?
		UsbdHciRequest : nullptr;
	coreCfg.ConfigHandler = UsbdHciConfig;
	coreCfg.SetInterfaceHandler = UsbdHciSetInterface;
	coreCfg.XferHandler = nullptr;
	coreCfg.ResetHandler = UsbdHciReset;
	coreCfg.SofHandler = nullptr;
	coreCfg.ProcessHandler = nullptr;
	coreCfg.pContext = pHci;

	UsbFuncReq_t req = {};
	req.InterfaceCount = 2U;
	req.BidirectionalCount = 1U;
	req.InCount = 1U;

	UsbFuncAlloc_t alloc = {};
	if (!UsbRegisterFuncAuto(pHci->DevNo, &req, &coreCfg, &alloc))
	{
		return false;
	}

	pHci->HciItfNo = alloc.FirstInterface;
	pHci->SyncItfNo = alloc.FirstInterface + 1U;
	pHci->EventEpNo = alloc.In[0];
	pHci->AclEpNo = alloc.Bidirectional[0];

	return true;
}
