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

#include "usb/usbd_bulk.h"

static uint8_t *UsbdBulkRxBuffer(UsbdBulkDev_t *pBulk)
{
	return reinterpret_cast<uint8_t *>(pBulk->RxTransfer);
}

static uint8_t *UsbdBulkTxBuffer(UsbdBulkDev_t *pBulk)
{
	return reinterpret_cast<uint8_t *>(pBulk->TxTransfer);
}

static uint16_t UsbdBulkMps(const UsbdBulkDev_t *pBulk)
{
	return UsbCtrlrHighSpeed(pBulk->DevNo) ? pBulk->HsMps : pBulk->FsMps;
}

static bool UsbdBulkOpenEndpoint(UsbdBulkDev_t *pBulk, uint8_t EpAddr,
								 uint16_t Mps)
{
	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bEndpointAddress = EpAddr;
	desc.bmAttributes = USB_ENDPATT_TRANS_BULK;
	desc.wMaxPacketSize = Mps;
	desc.bInterval = 0U;

	return UsbCtrlrEpOpen(pBulk->DevNo, &desc);
}

static void UsbdBulkCloseEndpoints(UsbdBulkDev_t *pBulk)
{
	UsbCtrlrEpClose(pBulk->DevNo, USB_ENDPADDR_DIROUT(pBulk->EpNo));
	UsbCtrlrEpClose(pBulk->DevNo, USB_ENDPADDR_DIRIN(pBulk->EpNo));
}

static bool UsbdBulkConfig(uint8_t Configuration, void *pContext)
{
	UsbdBulkDev_t *pBulk = static_cast<UsbdBulkDev_t *>(pContext);

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
	if (!UsbdBulkOpenEndpoint(pBulk, USB_ENDPADDR_DIROUT(pBulk->EpNo), mps) ||
		!UsbdBulkOpenEndpoint(pBulk, USB_ENDPADDR_DIRIN(pBulk->EpNo), mps))
	{
		UsbdBulkCloseEndpoints(pBulk);
		return false;
	}

	if (!UsbIntrfConfigure(pBulk->pData, mps))
	{
		UsbdBulkCloseEndpoints(pBulk);
		return false;
	}

	return true;
}

static bool UsbdBulkRequest(const UsbSetupData_t *pSetup,
							UsbCtrlStage_t Stage, uint8_t **ppData,
							uint16_t *pLength, void *pContext)
{
	UsbdBulkDev_t *pBulk = static_cast<UsbdBulkDev_t *>(pContext);

	if (pBulk == nullptr || pBulk->RequestHandler == nullptr ||
		pSetup == nullptr)
	{
		return false;
	}

	const uint8_t recipient =
		pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT;
	if (recipient == USB_REQTYPE_INTERFACE &&
		((pSetup->wIndex & 0xFF00U) != 0U ||
		 (uint8_t)pSetup->wIndex != (uint8_t)pBulk->ItfNo))
	{
		return false;
	}
	if (recipient == USB_REQTYPE_ENDPOINT &&
		((pSetup->wIndex & 0xFF00U) != 0U ||
		 USB_ENDPADDR_NUM((uint8_t)pSetup->wIndex) != pBulk->EpNo))
	{
		return false;
	}

	return pBulk->RequestHandler(pSetup, Stage, ppData, pLength,
								 pBulk->pRequestContext);
}

static void UsbdBulkXfer(uint8_t EpAddr, uint16_t Length,
						 UsbCtrlrXferResult_t Result, void *pContext)
{
	UsbdBulkDev_t *pBulk = static_cast<UsbdBulkDev_t *>(pContext);

	if (pBulk != nullptr)
	{
		UsbIntrfXferComplete(pBulk->pData, EpAddr, Length, Result);
	}
}

static void UsbdBulkReset(void *pContext)
{
	UsbdBulkDev_t *pBulk = static_cast<UsbdBulkDev_t *>(pContext);

	if (pBulk != nullptr)
	{
		UsbIntrfUnconfigure(pBulk->pData);
	}
}

bool UsbdBulkMakeDesc(UsbdBulkDesc_t *pDesc, const UsbdBulkCfg_t *pCfg,
					  UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pCfg == nullptr || pCfg->ItfNo < 0 ||
		pCfg->ItfNo > UINT8_MAX || pCfg->EpNo == 0U || pCfg->EpNo > 15U)
	{
		return false;
	}

	uint16_t mps = Speed == USB_SPEED_HIGH ? pCfg->HsMps : pCfg->FsMps;
	if (mps == 0U)
	{
		mps = Speed == USB_SPEED_HIGH ? USBD_BULK_HS_MPS : USBD_BULK_FS_MPS;
	}
	if (mps > USBD_BULK_MAX_MPS)
	{
		return false;
	}

	memset(pDesc, 0, sizeof(*pDesc));
	pDesc->Interface.bLength = sizeof(pDesc->Interface);
	pDesc->Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Interface.bInterfaceNumber = (uint8_t)pCfg->ItfNo;
	pDesc->Interface.bAlternateSetting = 0U;
	pDesc->Interface.bNumEndpoints = 2U;
	pDesc->Interface.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	pDesc->Interface.bInterfaceSubClass = pCfg->SubClass;
	pDesc->Interface.bInterfaceProtocol = pCfg->Protocol;
	pDesc->Interface.iInterface = pCfg->InterfaceString;

	pDesc->Out.bLength = sizeof(pDesc->Out);
	pDesc->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pCfg->EpNo);
	pDesc->Out.bmAttributes = USB_ENDPATT_TRANS_BULK;
	pDesc->Out.wMaxPacketSize = mps;
	pDesc->Out.bInterval = 0U;

	pDesc->In = pDesc->Out;
	pDesc->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pCfg->EpNo);

	return true;
}

bool UsbdBulkInit(UsbdBulkDev_t * const pBulk,
				  UsbDevIntrf_t * const pData,
				  const UsbdBulkCfg_t *pCfg)
{
	if (pBulk == nullptr || pData == nullptr || pCfg == nullptr ||
		UsbGetCfg(pCfg->DevNo) == nullptr ||
		pCfg->ItfNo < 0 || pCfg->ItfNo > UINT8_MAX ||
		pCfg->EpNo == 0U || pCfg->EpNo > 15U ||
		pCfg->Mode > USBD_BULK_MODE_PACKET)
	{
		return false;
	}

	pBulk->pData = pData;
	pBulk->RequestHandler = pCfg->RequestHandler;
	pBulk->pRequestContext = pCfg->pRequestContext;
	pBulk->ItfNo = pCfg->ItfNo;
	pBulk->DevNo = pCfg->DevNo;
	pBulk->EpNo = pCfg->EpNo;
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

	UsbFuncCfg_t coreCfg = {};
	coreCfg.FirstInterface = (uint8_t)pCfg->ItfNo;
	coreCfg.InterfaceCount = 1U;
	coreCfg.EpInMask = (uint16_t)(1U << pCfg->EpNo);
	coreCfg.EpOutMask = (uint16_t)(1U << pCfg->EpNo);
	coreCfg.RequestHandler = pCfg->RequestHandler != nullptr ?
		UsbdBulkRequest : nullptr;
	coreCfg.ConfigHandler = UsbdBulkConfig;
	coreCfg.SetInterfaceHandler = nullptr;
	coreCfg.XferHandler = UsbdBulkXfer;
	coreCfg.ResetHandler = UsbdBulkReset;
	coreCfg.SofHandler = nullptr;
	coreCfg.ProcessHandler = nullptr;
	coreCfg.pContext = pBulk;

	return UsbRegisterFunc(pBulk->DevNo, &coreCfg);
}

bool UsbdBulk::Init(const UsbdBulkCfg_t &Cfg)
{
	UsbIntrfCfg_t dataCfg = {};
	dataCfg.bBlocking = Cfg.bBlocking;
	dataCfg.RxFifoMemSize = Cfg.RxFifoMemSize;
	dataCfg.pRxFifoMem = Cfg.pRxFifoMem;
	dataCfg.TxFifoMemSize = Cfg.TxFifoMemSize;
	dataCfg.pTxFifoMem = Cfg.pTxFifoMem;
	dataCfg.TxFifoBlkSize = Cfg.Mode == USBD_BULK_MODE_PACKET ?
		USBD_BULK_PKT_BLKSIZE : 1U;
	dataCfg.DevNo = Cfg.DevNo;
	dataCfg.EvtCB = Cfg.EvtCB;
	dataCfg.EpNo = Cfg.EpNo;
	dataCfg.BufferSize = (uint16_t)sizeof(vUsbdBulk.RxTransfer);
	dataCfg.pRxBuffer = UsbdBulkRxBuffer(&vUsbdBulk);
	dataCfg.pTxBuffer = UsbdBulkTxBuffer(&vUsbdBulk);

	if (!UsbIntrf::Init(dataCfg))
	{
		return false;
	}

	return UsbdBulkInit(&vUsbdBulk, &vUsbDevIntrf, &Cfg);
}
