/**-------------------------------------------------------------------------
@file	usbd_cdc.cpp

@brief	USB CDC ACM class adapter.

CDC owns ACM control requests and notifications. Generic UsbdIntrf owns both
application FIFOs and the non-control RX/TX data path.

@author	Hoang Nguyen Hoan
@date	May 2, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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

#include "usb/usb.h"
#include "usb/usbd_cdc.h"

static uint8_t *UsbdCdcRxBuffer(UsbdCdcDev_t *pCdc)
{
	return reinterpret_cast<uint8_t *>(pCdc->RxTransfer);
}

static uint8_t *UsbdCdcTxBuffer(UsbdCdcDev_t *pCdc)
{
	return reinterpret_cast<uint8_t *>(pCdc->TxTransfer);
}

static uint8_t *UsbdCdcNotifBuffer(UsbdCdcDev_t *pCdc)
{
	return reinterpret_cast<uint8_t *>(pCdc->NotifTransfer);
}

static uint16_t UsbdCdcSerialStateMask(void)
{
	return USB_CDC_SERIAL_STATE_RX_CARRIER |
		   USB_CDC_SERIAL_STATE_TX_CARRIER |
		   USB_CDC_SERIAL_STATE_BREAK |
		   USB_CDC_SERIAL_STATE_RING |
		   USB_CDC_SERIAL_STATE_FRAMING |
		   USB_CDC_SERIAL_STATE_PARITY |
		   USB_CDC_SERIAL_STATE_OVERRUN;
}

static void UsbdCdcDefaultLineCoding(UsbdCdcDev_t *pCdc)
{
	pCdc->LineCoding.dwDTERate = 115200U;
	pCdc->LineCoding.bCharFormat = USB_CDC_LINE_STOP_1;
	pCdc->LineCoding.bParityType = USB_CDC_LINE_PARITY_NONE;
	pCdc->LineCoding.bDataBits = 8U;
	pCdc->PendingLineCoding = pCdc->LineCoding;
}

static uint16_t UsbdCdcMps(UsbdCdcDev_t *pCdc)
{
	return UsbCtrlrHighSpeed(pCdc->DevNo) ?
		   USBD_CDC_BULK_HS_MPS : USBD_CDC_BULK_FS_MPS;
}

static uint8_t UsbdCdcNotifInterval(UsbdCdcDev_t *pCdc)
{
	return UsbCtrlrHighSpeed(pCdc->DevNo) ? 8U : 16U;
}

bool UsbdCdcPortIsOpen(const UsbdCdcDev_t * const pCdc)
{
	return pCdc != nullptr && pCdc->Configured &&
		   (pCdc->ControlLineState & USB_CDC_CTRL_LINE_STATE_DTR) != 0U;
}

static void UsbdCdcNotifyPortState(UsbdCdcDev_t *pCdc, bool Open)
{
	if (pCdc->Data.DevIntrf.EvtCB != nullptr)
	{
		pCdc->Data.DevIntrf.EvtCB(&pCdc->Data.DevIntrf,
								 DEVINTRF_EVT_STATECHG,
								 nullptr, Open ? 1 : 0);
	}
}

static void UsbdCdcNotifKick(UsbdCdcDev_t *pCdc)
{
	if (pCdc == nullptr || !pCdc->Configured ||
		pCdc->NotifActive || !pCdc->SerialStatePending)
	{
		return;
	}

	UsbCdcNotification_t notification = {};
	notification.bmRequestType =
		USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS | USB_REQTYPE_INTERFACE;
	notification.bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notification.wValue = 0U;
	notification.wIndex = USBD_CDC_CTRL_INTRF(pCdc->ItfNo);
	notification.wLength = 2U;

	uint8_t *pData = UsbdCdcNotifBuffer(pCdc);
	memcpy(pData, &notification, sizeof(notification));
	pData[sizeof(notification)] = (uint8_t)pCdc->SerialState;
	pData[sizeof(notification) + 1U] =
		(uint8_t)(pCdc->SerialState >> 8);

	pCdc->SerialStatePending = false;
	pCdc->NotifActive = true;

	if (!UsbCtrlrEpXfer(pCdc->DevNo, USBD_CDC_NOTIF_EP(pCdc->ItfNo),
						 pData, USBD_CDC_NOTIFY_LEN))
	{
		pCdc->NotifActive = false;
		pCdc->SerialStatePending = true;
	}
}

static bool UsbdCdcOpenEndpoint(UsbdCdcDev_t *pCdc, uint8_t EpAddr, uint8_t Type,
								uint16_t Mps, uint8_t Interval)
{
	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bEndpointAddress = EpAddr;
	desc.bmAttributes = Type;
	desc.wMaxPacketSize = Mps;
	desc.bInterval = Interval;

	return UsbCtrlrEpOpen(pCdc->DevNo, &desc);
}

static void UsbdCdcCloseEndpoints(UsbdCdcDev_t *pCdc)
{
	UsbCtrlrEpClose(pCdc->DevNo, USBD_CDC_NOTIF_EP(pCdc->ItfNo));
	UsbCtrlrEpClose(pCdc->DevNo, USBD_CDC_DATA_OUT_EP(pCdc->ItfNo));
	UsbCtrlrEpClose(pCdc->DevNo, USBD_CDC_DATA_IN_EP(pCdc->ItfNo));
}

static void UsbdCdcCancelBusState(UsbdCdcDev_t *pCdc)
{
	pCdc->NotifActive = false;
	pCdc->SerialStatePending = false;
	UsbIntrfRxEnable(&pCdc->Data, false);
	UsbIntrfResetRx(&pCdc->Data);
	UsbIntrfResetTx(&pCdc->Data);
}

static bool UsbdCdcConfig(uint8_t Configuration, void *pContext)
{
	UsbdCdcDev_t *pCdc = static_cast<UsbdCdcDev_t *>(pContext);

	if (pCdc == nullptr)
	{
		return false;
	}

	pCdc->Configured = false;
	pCdc->ControlLineState = 0U;
	pCdc->PendingControlLineState = 0U;
	UsbdCdcCancelBusState(pCdc);

	if (Configuration == 0U)
	{
		return true;
	}

	if (Configuration != USBD_CDC_CONFIG_VALUE)
	{
		return false;
	}

	pCdc->DataMps = UsbdCdcMps(pCdc);

	if (!UsbdCdcOpenEndpoint(pCdc, USBD_CDC_NOTIF_EP(pCdc->ItfNo),
							 USB_ENDPATT_TRANS_INT,
							 USBD_CDC_NOTIF_MPS,
							 UsbdCdcNotifInterval(pCdc)) ||
		!UsbdCdcOpenEndpoint(pCdc, USBD_CDC_DATA_OUT_EP(pCdc->ItfNo),
							 USB_ENDPATT_TRANS_BULK,
							 pCdc->DataMps, 0U) ||
		!UsbdCdcOpenEndpoint(pCdc, USBD_CDC_DATA_IN_EP(pCdc->ItfNo),
							 USB_ENDPATT_TRANS_BULK,
							 pCdc->DataMps, 0U))
	{
		UsbdCdcCloseEndpoints(pCdc);
		return false;
	}

	if (!UsbIntrfConfigRx(&pCdc->Data,
						USBD_CDC_DATA_OUT_EP(pCdc->ItfNo), pCdc->DataMps,
						UsbdCdcRxBuffer(pCdc)))
	{
		UsbdCdcCloseEndpoints(pCdc);
		return false;
	}

	UsbIntrfConfigTx(&pCdc->Data,
					   USBD_CDC_DATA_IN_EP(pCdc->ItfNo),
					   pCdc->DataMps, UsbdCdcTxBuffer(pCdc));

	pCdc->Configured = true;
	pCdc->SerialStatePending = true;
	UsbdCdcNotifKick(pCdc);

	return true;
}

static bool UsbdCdcRequest(const UsbSetupData_t *pSetup,
						   UsbCtrlStage_t Stage,
						   uint8_t **ppData,
						   uint16_t *pLength,
						   void *pContext)
{
	UsbdCdcDev_t *pCdc = static_cast<UsbdCdcDev_t *>(pContext);

	if (pSetup == nullptr || pCdc == nullptr || pLength == nullptr ||
		!pCdc->Configured ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) != USB_REQTYPE_CLASS ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT) !=
			USB_REQTYPE_INTERFACE ||
		(pSetup->wIndex & 0xFF00U) != 0U ||
		(uint8_t)pSetup->wIndex != USBD_CDC_CTRL_INTRF(pCdc->ItfNo))
	{
		return false;
	}

	switch (pSetup->bRequest)
	{
		case USB_CDC_REQ_SET_LINE_CODING:
			if ((pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) !=
					USB_REQTYPE_DIRDEV ||
				pSetup->wValue != 0U ||
				pSetup->wLength != sizeof(UsbCdcLineCoding_t))
			{
				return false;
			}

			if (Stage == USB_CTRL_SETUP)
			{
				if (ppData == nullptr)
				{
					return false;
				}
				*ppData =
					reinterpret_cast<uint8_t *>(&pCdc->PendingLineCoding);
				*pLength = sizeof(pCdc->PendingLineCoding);
				return true;
			}

			if (Stage == USB_CTRL_DATA)
			{
				return *pLength == sizeof(UsbCdcLineCoding_t);
			}

			if (Stage == USB_CTRL_COMPLETE)
			{
				pCdc->LineCoding = pCdc->PendingLineCoding;
				return true;
			}
			return false;

		case USB_CDC_REQ_GET_LINE_CODING:
			if ((pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) !=
					USB_REQTYPE_DIRHOST ||
				pSetup->wValue != 0U ||
				pSetup->wLength != sizeof(UsbCdcLineCoding_t))
			{
				return false;
			}

			if (Stage == USB_CTRL_SETUP)
			{
				if (ppData == nullptr)
				{
					return false;
				}
				*ppData = reinterpret_cast<uint8_t *>(&pCdc->LineCoding);
				*pLength = sizeof(pCdc->LineCoding);
			}
			return true;

		case USB_CDC_REQ_SET_CTRL_LINE_STATE:
			if ((pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) !=
					USB_REQTYPE_DIRDEV ||
				pSetup->wLength != 0U ||
				(pSetup->wValue &
				 ~(USB_CDC_CTRL_LINE_STATE_DTR |
				   USB_CDC_CTRL_LINE_STATE_RTS)) != 0U)
			{
				return false;
			}

			if (Stage == USB_CTRL_SETUP)
			{
				pCdc->PendingControlLineState = pSetup->wValue;
				*pLength = 0U;
				return true;
			}

			if (Stage == USB_CTRL_COMPLETE)
			{
				pCdc->ControlLineState = pCdc->PendingControlLineState;
				UsbIntrfRxEnable(&pCdc->Data, UsbdCdcPortIsOpen(pCdc));
				return true;
			}
			return Stage == USB_CTRL_DATA;

		default:
			return false;
	}
}

static void UsbdCdcXfer(uint8_t EpAddr, uint16_t Length,
						UsbCtrlrXferResult_t Result, void *pContext)
{
	UsbdCdcDev_t *pCdc = static_cast<UsbdCdcDev_t *>(pContext);

	if (pCdc == nullptr)
	{
		return;
	}

	if (EpAddr == pCdc->Data.RxEpAddr)
	{
		UsbIntrfRxXferComplete(&pCdc->Data, Length, Result);
		return;
	}

	if (EpAddr == pCdc->Data.TxEpAddr)
	{
		UsbIntrfTxXferComplete(&pCdc->Data, Length, Result);
		return;
	}

	if (EpAddr == USBD_CDC_NOTIF_EP(pCdc->ItfNo))
	{
		pCdc->NotifActive = false;
		if (Result == USB_CTRLR_XFER_SUCCESS)
		{
			UsbdCdcNotifKick(pCdc);
		}
	}
}

static void UsbdCdcSof(uint16_t, void *pContext)
{
	UsbdCdcDev_t *pCdc = static_cast<UsbdCdcDev_t *>(pContext);

	if (pCdc != nullptr)
	{
		UsbIntrfSof(&pCdc->Data);
	}
}

static void UsbdCdcReset(void *pContext)
{
	UsbdCdcDev_t *pCdc = static_cast<UsbdCdcDev_t *>(pContext);

	if (pCdc == nullptr)
	{
		return;
	}

	pCdc->Configured = false;
	pCdc->ControlLineState = 0U;
	pCdc->PendingControlLineState = 0U;
	pCdc->SerialState = 0U;
	UsbdCdcCancelBusState(pCdc);
	UsbdCdcDefaultLineCoding(pCdc);
}

static void UsbdCdcPump(void *pContext)
{
	UsbdCdcProcess(static_cast<UsbdCdcDev_t *>(pContext));
}

bool UsbdCdcInit(UsbdCdcDev_t * const pCdc, const UsbdCdcCfg_t *pCfg)
{
	if (pCdc == nullptr || pCfg == nullptr ||
		pCfg->ItfNo < 0 || pCfg->ItfNo >= USBD_CDC_FUNC_MAXCNT ||
		pCfg->pRxFifoMem == nullptr || pCfg->pTxFifoMem == nullptr)
	{
		return false;
	}

	const UsbCfg_t *pDevCfg = UsbGetCfg(pCdc->DevNo);
	if (pDevCfg == nullptr || pCfg->ItfNo >= pDevCfg->NbCdc)
	{
		return false;
	}

	UsbIntrfCfg_t dataCfg = {};
	dataCfg.bBlocking = pCfg->bBlocking;
	dataCfg.RxFifoMemSize = pCfg->RxFifoMemSize;
	dataCfg.pRxFifoMem = pCfg->pRxFifoMem;
	dataCfg.TxFifoMemSize = pCfg->TxFifoMemSize;
	dataCfg.pTxFifoMem = pCfg->pTxFifoMem;
	dataCfg.DevNo = pCfg->DevNo;
	dataCfg.EvtCB = pCfg->EvtCB;
	pCdc->DevNo = pCfg->DevNo;

	if (!UsbIntrfInit(&pCdc->Data, &dataCfg))
	{
		return false;
	}

	pCdc->ItfNo = pCfg->ItfNo;
	pCdc->DataMps = UsbdCdcMps(pCdc);
	pCdc->ControlLineState = 0U;
	pCdc->PendingControlLineState = 0U;
	pCdc->SerialState = 0U;
	pCdc->RxDropCnt = 0U;
	pCdc->TxDropCnt = 0U;
	pCdc->Configured = false;
	pCdc->ReportedOpen = false;
	UsbdCdcCancelBusState(pCdc);
	UsbdCdcDefaultLineCoding(pCdc);

	UsbFuncCfg_t coreCfg = {};
	coreCfg.FirstInterface = USBD_CDC_CTRL_INTRF(pCfg->ItfNo);
	coreCfg.InterfaceCount = 2U;
	coreCfg.EpInMask =
		(uint16_t)((1U << USB_ENDPADDR_NUM(
			USBD_CDC_NOTIF_EP(pCfg->ItfNo))) |
				   (1U << USB_ENDPADDR_NUM(
			USBD_CDC_DATA_IN_EP(pCfg->ItfNo))));
	coreCfg.EpOutMask =
		(uint16_t)(1U << USB_ENDPADDR_NUM(
			USBD_CDC_DATA_OUT_EP(pCfg->ItfNo)));
	coreCfg.RequestHandler = UsbdCdcRequest;
	coreCfg.ConfigHandler = UsbdCdcConfig;
	coreCfg.SetInterfaceHandler = nullptr;
	coreCfg.XferHandler = UsbdCdcXfer;
	coreCfg.ResetHandler = UsbdCdcReset;
	coreCfg.SofHandler = UsbdCdcSof;
	coreCfg.ProcessHandler = UsbdCdcPump;
	coreCfg.pContext = pCdc;

	if (!UsbRegisterFunc(pCdc->DevNo, &coreCfg))
	{
		return false;
	}


	return true;
}

void UsbdCdcProcess(UsbdCdcDev_t * const pCdc)
{
	if (pCdc == nullptr)
	{
		return;
	}

	const bool open = UsbdCdcPortIsOpen(pCdc);

	if (open != pCdc->ReportedOpen)
	{
		pCdc->ReportedOpen = open;
		UsbIntrfRxEnable(&pCdc->Data, open);
		UsbdCdcNotifyPortState(pCdc, open);
	}

	UsbdCdcNotifKick(pCdc);
	pCdc->RxDropCnt = pCdc->Data.RxDropCnt;
	pCdc->TxDropCnt = pCdc->Data.hTxFifo != nullptr ?
		pCdc->Data.hTxFifo->DropCnt : 0U;
}

const UsbCdcLineCoding_t *UsbdCdcLineCoding(const UsbdCdcDev_t * const pCdc)
{
	return pCdc != nullptr ? &pCdc->LineCoding : nullptr;
}

uint16_t UsbdCdcControlLineState(const UsbdCdcDev_t * const pCdc)
{
	return pCdc != nullptr ? pCdc->ControlLineState : 0U;
}

void UsbdCdcSetSerialState(UsbdCdcDev_t * const pCdc, uint16_t SerialState)
{
	if (pCdc == nullptr)
	{
		return;
	}

	pCdc->SerialState = SerialState & UsbdCdcSerialStateMask();
	pCdc->SerialStatePending = true;
	UsbdCdcNotifKick(pCdc);
}

bool UsbdCdc::Init(const UsbdCdcCfg_t &Cfg)
{
	return UsbdCdcInit(&vUsbdCdc, &Cfg);
}

bool UsbdCdc::IsPortOpen(void)
{
	return UsbdCdcPortIsOpen(&vUsbdCdc);
}

const UsbCdcLineCoding_t *UsbdCdc::LineCoding(void)
{
	return UsbdCdcLineCoding(&vUsbdCdc);
}

uint16_t UsbdCdc::ControlLineState(void)
{
	return UsbdCdcControlLineState(&vUsbdCdc);
}

void UsbdCdc::SetSerialState(uint16_t SerialState)
{
	UsbdCdcSetSerialState(&vUsbdCdc, SerialState);
}
