/**-------------------------------------------------------------------------
@file	usbd_bulk.cpp

@brief	USB vendor bulk interface implementation.

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

#include "usb/usbd_epalloc.h"
#include "usb/usbd_bulk.h"

static uint8_t *UsbdBulkRxBuffer(UsbdBulkDev_t *pBulk)
{
	return reinterpret_cast<uint8_t *>(pBulk->RxTransfer);
}

static uint16_t UsbdBulkMps(const UsbdBulkDev_t *pBulk)
{
	return UsbCtrlrHighSpeed(pBulk->DevNo) ? pBulk->HsMps : pBulk->FsMps;
}

static bool UsbdBulkOpenEndpoint(UsbdBulkDev_t *pBulk, bool bIn,
								 uint16_t MaxPacketSize)
{
	return UsbCtrlrEpOpenData(pBulk->DevNo, pBulk->EpNo, bIn,
		USB_ENDPATT_TRANS_BULK, MaxPacketSize);
}

static void UsbdBulkCloseEndpoints(UsbdBulkDev_t *pBulk)
{
	UsbCtrlrEpClose(pBulk->DevNo, pBulk->EpNo, false);
	UsbCtrlrEpClose(pBulk->DevNo, pBulk->EpNo, true);
}

static bool UsbdBulkConfig(UsbdBulkDev_t *pBulk, uint8_t Configuration)
{
	if (pBulk == nullptr)
	{
		return false;
	}

	UsbIntrfUnconfigure(pBulk->pData);
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != USBD_BULK_CONFIG_VALUE)
	{
		return false;
	}

	const uint16_t mps = UsbdBulkMps(pBulk);
	if (!UsbIntrfConfigure(pBulk->pData, mps))
	{
		return false;
	}

	if (!UsbdBulkOpenEndpoint(pBulk, true, mps) ||
		!UsbdBulkOpenEndpoint(pBulk, false, mps))
	{
		UsbdBulkCloseEndpoints(pBulk);
		UsbIntrfUnconfigure(pBulk->pData);
		return false;
	}

	return true;
}

static void UsbdBulkReset(UsbdBulkDev_t *pBulk)
{
	if (pBulk != nullptr)
	{
		UsbIntrfUnconfigure(pBulk->pData);
	}
}

static constexpr UsbdBulkDesc_t UsbdBulkDescTemplate(void)
{
	UsbdBulkDesc_t desc = {};
	desc.Interface.bLength = sizeof(desc.Interface);
	desc.Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Interface.bNumEndpoints = 2U;
	desc.Interface.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	desc.Out.bLength = sizeof(desc.Out);
	desc.Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.Out.bmAttributes = USB_ENDPATT_TRANS_BULK;
	desc.In = desc.Out;
	return desc;
}

static constexpr UsbdBulkDesc_t s_BulkDescTemplate = UsbdBulkDescTemplate();

static void UsbdBulkPatchDesc(UsbdBulkDesc_t *pDesc,
							  const UsbdBulkDev_t *pBulk, UsbSpeed_t Speed)
{
	const uint16_t mps = Speed == USB_SPEED_HIGH ? pBulk->HsMps : pBulk->FsMps;
	pDesc->Interface.bInterfaceNumber = (uint8_t)pBulk->ItfNo;
	pDesc->Interface.bInterfaceSubClass = pBulk->SubClass;
	pDesc->Interface.bInterfaceProtocol = pBulk->Protocol;
	pDesc->Interface.iInterface = pBulk->InterfaceString;
	pDesc->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pBulk->EpNo);
	pDesc->Out.wMaxPacketSize = mps;
	pDesc->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pBulk->EpNo);
	pDesc->In.wMaxPacketSize = mps;
}

// Weak so an application can replace runtime fragment building with a static
// fragment. When overridden, this default is dropped by unused-section removal.
// Pair a replacement with a strong UsbGetDescriptor for fully static
// descriptors, or the assembled configuration will not match.
__attribute__((weak))
bool UsbdBulkMakeDesc(UsbdBulkDesc_t *pDesc, const UsbdBulkDev_t *pBulk,
					  UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pBulk == nullptr || pBulk->ItfNo < 0 ||
		pBulk->ItfNo > UINT8_MAX || pBulk->EpNo == 0U || pBulk->EpNo > 15U)
	{
		return false;
	}

	const uint16_t mps = Speed == USB_SPEED_HIGH ? pBulk->HsMps : pBulk->FsMps;
	if (mps == 0U || mps > USBD_BULK_MAX_MPS)
	{
		return false;
	}

	memcpy(pDesc, &s_BulkDescTemplate, sizeof(*pDesc));
	UsbdBulkPatchDesc(pDesc, pBulk, Speed);
	return true;
}

static bool UsbdBulkInitInternal(UsbdBulkDev_t * const pBulk,
								 UsbDevIntrf_t *pData,
								 const UsbdBulkCfg_t *pCfg,
								 UsbDeviceClass *pClass)
{
	if (pBulk == nullptr || pCfg == nullptr || pClass == nullptr ||
		UsbGetCfg(pCfg->DevNo) == nullptr ||
		pCfg->pRxFifoMem == nullptr || pCfg->RxFifoMemSize <= 0 ||
		pCfg->pTxFifoMem == nullptr || pCfg->TxFifoMemSize <= 0 ||
		pCfg->Mode > USBD_BULK_MODE_PACKET)
	{
		return false;
	}

	pBulk->pData = pData;
	pBulk->DevNo = pCfg->DevNo;
	pBulk->SubClass = pCfg->SubClass;
	pBulk->Protocol = pCfg->Protocol;
	pBulk->InterfaceString = pCfg->InterfaceString;
	pBulk->FsMps = pCfg->FsMps != 0U ? pCfg->FsMps : USBD_BULK_FS_MPS;
	pBulk->HsMps = pCfg->HsMps != 0U ? pCfg->HsMps : USBD_BULK_HS_MPS;

	if (pBulk->FsMps > USBD_BULK_MAX_MPS ||
		(USB_HIGHSPEED_CAPABLE(0) && pBulk->HsMps > USBD_BULK_MAX_MPS))
	{
		return false;
	}

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;

	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(pBulk->DevNo, &req, pClass, &alloc))
	{
		return false;
	}

	pBulk->ItfNo = alloc.FirstInterface;
	pBulk->EpNo = alloc.Bidirectional[0];

	UsbIntrfCfg_t dataCfg = {};
	dataCfg.bBlocking = pCfg->bBlocking;
	dataCfg.RxFifoMemSize = pCfg->RxFifoMemSize;
	dataCfg.pRxFifoMem = pCfg->pRxFifoMem;
	dataCfg.TxFifoMemSize = pCfg->TxFifoMemSize;
	dataCfg.pTxFifoMem = pCfg->pTxFifoMem;
	dataCfg.TxFifoBlkSize = pCfg->Mode == USBD_BULK_MODE_PACKET ?
		USBD_BULK_PKT_BLKSIZE : 1U;
	dataCfg.DevNo = pBulk->DevNo;
	dataCfg.EvtCB = pCfg->EvtCB;
	dataCfg.EpNo = pBulk->EpNo;
	dataCfg.BufferSize = (uint16_t)sizeof(pBulk->RxTransfer);
	dataCfg.pRxBuffer = UsbdBulkRxBuffer(pBulk);

	if (!UsbIntrfInit(pBulk->pData, &dataCfg))
	{
		return false;
	}

	pBulk->pData->pClassContext = pBulk;

	const void *pHsDesc = USB_HIGHSPEED_CAPABLE(pBulk->DevNo) ?
		&s_BulkDescTemplate : nullptr;
	const uint16_t hsDescLength = pHsDesc != nullptr ?
		sizeof(s_BulkDescTemplate) : 0U;

	return UsbDescriptorRegister(pBulk->DevNo, pClass,
		&s_BulkDescTemplate, sizeof(s_BulkDescTemplate),
		pHsDesc, hsDescLength);
}

bool UsbdBulk::Init(const UsbdBulkCfg_t &Cfg)
{
	return UsbdBulkInitInternal(&vUsbdBulk, &vUsbDevIntrf, &Cfg, this);
}

bool UsbdBulk::Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
						uint8_t **ppData, uint16_t *pLength)
{
	(void)pSetup;
	(void)Stage;
	(void)ppData;
	(void)pLength;
	return false;
}

bool UsbdBulk::SelectConfig(uint8_t ConfigValue)
{
	return UsbdBulkConfig(&vUsbdBulk, ConfigValue);
}

void UsbdBulk::PatchDescriptor(uint8_t *pDesc, UsbSpeed_t Speed) const
{
	UsbdBulkPatchDesc(reinterpret_cast<UsbdBulkDesc_t *>(pDesc),
		&vUsbdBulk, Speed);
}

void UsbdBulk::Reset(void)
{
	UsbdBulkReset(&vUsbdBulk);
}
