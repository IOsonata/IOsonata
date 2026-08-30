/**-------------------------------------------------------------------------
@file	usbd_cdc_intrf.cpp

@brief	USB CDC ACM device interface

One CDC ACM port : the ACM control protocol, the endpoint staging and the
DeviceIntrf the application talks to, in one instance owned by the caller.

The Bulk interface is the first member of the instance, so the CDC interface
is that DeviceIntrf rather than a second one forwarding to it. RxKick and
TxKick carry the instance itself as their context, and so does the device core
function registration, so nothing here looks a port number up in a table.

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

#include "usb/usb_dev.h"
#include "usb/usbd_cdc.h"
#include "usb/usbd_core.h"
#include "usb/usbd_ctrlr.h"

static uint8_t *UsbdCdcRxBuffer(UsbdCdcRxStage_t *pStage)
{
	return reinterpret_cast<uint8_t *>(pStage->Data);
}

static UsbdCdcRxStage_t *UsbdCdcRxStage(UsbdCdcDevIntrf_t *pIntrf,
										uint32_t Index)
{
	return &pIntrf->RxStage[Index % USBD_CDC_RX_STAGE_COUNT];
}

static uint8_t *UsbdCdcTxBuffer(UsbdCdcDevIntrf_t *pIntrf)
{
	return reinterpret_cast<uint8_t *>(pIntrf->TxTransfer);
}

static uint8_t *UsbdCdcNotifBuffer(UsbdCdcDevIntrf_t *pIntrf)
{
	return reinterpret_cast<uint8_t *>(pIntrf->NotifTransfer);
}

static uint32_t UsbdCdcRxPut(const UsbdCdcDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->RxPut, __ATOMIC_ACQUIRE);
}

static uint32_t UsbdCdcRxGet(const UsbdCdcDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->RxGet, __ATOMIC_ACQUIRE);
}

static bool UsbdCdcTxCompletionPending(const UsbdCdcDevIntrf_t *pIntrf)
{
	return __atomic_load_n(&pIntrf->TxCompletePending, __ATOMIC_ACQUIRE);
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

static void UsbdCdcDefaultLineCoding(UsbdCdcDevIntrf_t *pIntrf)
{
	pIntrf->LineCoding.dwDTERate = 115200U;
	pIntrf->LineCoding.bCharFormat = USB_CDC_LINE_STOP_1;
	pIntrf->LineCoding.bParityType = USB_CDC_LINE_PARITY_NONE;
	pIntrf->LineCoding.bDataBits = 8U;
	pIntrf->PendingLineCoding = pIntrf->LineCoding;
}

static uint16_t UsbdCdcMps(void)
{
	return UsbdCtrlrHighSpeed() ?
		USBD_CDC_BULK_HS_MPS : USBD_CDC_BULK_FS_MPS;
}

static uint8_t UsbdCdcNotifInterval(void)
{
	return UsbdCtrlrHighSpeed() ? 8U : 16U;
}

bool UsbdCdcIntrfPortIsOpen(const UsbdCdcDevIntrf_t * const pIntrf)
{
	return pIntrf != nullptr && pIntrf->Configured &&
		(pIntrf->ControlLineState & USB_CDC_CTRL_LINE_STATE_DTR) != 0U;
}

static void UsbdCdcNotifyPortState(UsbdCdcDevIntrf_t *pIntrf, bool Open)
{
	if (pIntrf->Bulk.DevIntrf.EvtCB != nullptr)
	{
		pIntrf->Bulk.DevIntrf.EvtCB(&pIntrf->Bulk.DevIntrf,
									DEVINTRF_EVT_STATECHG,
									nullptr, Open ? 1 : 0);
	}
}

static void UsbdCdcRxKick(UsbdBulkDevIntrf_t * const pBulk, void *pContext)
{
	UsbdCdcDevIntrf_t *pIntrf = static_cast<UsbdCdcDevIntrf_t *>(pContext);

	if (pIntrf == nullptr || &pIntrf->Bulk != pBulk ||
		!pBulk->bEnabled || !UsbdCdcIntrfPortIsOpen(pIntrf) ||
		pIntrf->RxActive)
	{
		return;
	}

	const uint32_t put = UsbdCdcRxPut(pIntrf);
	const uint32_t get = UsbdCdcRxGet(pIntrf);
	if ((put - get) >= USBD_CDC_RX_STAGE_COUNT)
	{
		return;
	}

	UsbdCdcRxStage_t *pStage = UsbdCdcRxStage(pIntrf, put);
	pIntrf->RxActive = true;
	if (!UsbdCtrlrEpXfer(USBD_CDC_DATA_OUT_EP(pIntrf->ItfNo),
						 UsbdCdcRxBuffer(pStage), pIntrf->BulkMps))
	{
		pIntrf->RxActive = false;
	}
}

static void UsbdCdcTxKick(UsbdBulkDevIntrf_t * const pBulk, void *pContext)
{
	UsbdCdcDevIntrf_t *pIntrf = static_cast<UsbdCdcDevIntrf_t *>(pContext);

	if (pIntrf == nullptr || &pIntrf->Bulk != pBulk ||
		!pBulk->bEnabled || !UsbdCdcIntrfPortIsOpen(pIntrf) ||
		pIntrf->TxActive || pIntrf->TxZlpActive || pIntrf->TxZlpRequired ||
		UsbdCdcTxCompletionPending(pIntrf))
	{
		return;
	}

	if (pIntrf->TxLength == 0U)
	{
		//
		// Stage as much as the transfer buffer holds, not one packet.
		// UsbdCtrlrEpXfer takes a logical length and the controller splits it,
		// so one submission moves several back to back packets. Staging
		// one packet at a time left the endpoint idle for a whole completion
		// round trip between packets and put the ceiling at one packet per
		// pump pass.
		//
		const int length = UsbdBulkIntrfGetTxData(pBulk,
			UsbdCdcTxBuffer(pIntrf), (int)sizeof(pIntrf->TxTransfer));
		if (length <= 0)
		{
			return;
		}

		pIntrf->TxLength = (uint16_t)length;
	}

	pIntrf->TxActive = true;
	if (!UsbdCtrlrEpXfer(USBD_CDC_DATA_IN_EP(pIntrf->ItfNo),
						 UsbdCdcTxBuffer(pIntrf), pIntrf->TxLength))
	{
		pIntrf->TxActive = false;
	}
}

static void UsbdCdcNotifKick(UsbdCdcDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || !pIntrf->Configured ||
		pIntrf->NotifActive || !pIntrf->SerialStatePending)
	{
		return;
	}

	UsbCdcNotification_t notification = {};
	notification.bmRequestType =
		USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS | USB_REQTYPE_INTERFACE;
	notification.bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notification.wValue = 0U;
	notification.wIndex = USBD_CDC_CTRL_INTRF(pIntrf->ItfNo);
	notification.wLength = 2U;

	uint8_t *pData = UsbdCdcNotifBuffer(pIntrf);
	memcpy(pData, &notification, sizeof(notification));
	pData[sizeof(notification)] = (uint8_t)pIntrf->SerialState;
	pData[sizeof(notification) + 1U] = (uint8_t)(pIntrf->SerialState >> 8);

	pIntrf->SerialStatePending = false;
	pIntrf->NotifActive = true;
	if (!UsbdCtrlrEpXfer(USBD_CDC_NOTIF_EP(pIntrf->ItfNo),
						 pData, USBD_CDC_NOTIFY_LEN))
	{
		pIntrf->NotifActive = false;
		pIntrf->SerialStatePending = true;
	}
}

static bool UsbdCdcOpenEndpoint(uint8_t EpAddr, uint8_t Type,
								uint16_t Mps, uint8_t Interval)
{
	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bEndpointAddress = EpAddr;
	desc.bmAttributes = Type;
	desc.wMaxPacketSize = Mps;
	desc.bInterval = Interval;
	return UsbdCtrlrEpOpen(&desc);
}

static void UsbdCdcCloseEndpoints(UsbdCdcDevIntrf_t *pIntrf)
{
	UsbdCtrlrEpClose(USBD_CDC_NOTIF_EP(pIntrf->ItfNo));
	UsbdCtrlrEpClose(USBD_CDC_DATA_OUT_EP(pIntrf->ItfNo));
	UsbdCtrlrEpClose(USBD_CDC_DATA_IN_EP(pIntrf->ItfNo));
}

static void UsbdCdcCancelBusState(UsbdCdcDevIntrf_t *pIntrf)
{
	pIntrf->RxActive = false;
	pIntrf->TxActive = false;
	pIntrf->TxZlpActive = false;
	pIntrf->TxZlpRequired = false;
	pIntrf->NotifActive = false;
	pIntrf->SerialStatePending = false;
	__atomic_store_n(&pIntrf->TxCompletePending, false, __ATOMIC_RELEASE);
}

static bool UsbdCdcConfig(uint8_t Configuration, void *pContext)
{
	UsbdCdcDevIntrf_t *pIntrf = static_cast<UsbdCdcDevIntrf_t *>(pContext);
	if (pIntrf == nullptr)
	{
		return false;
	}

	pIntrf->Configured = false;
	pIntrf->ControlLineState = 0U;
	pIntrf->PendingControlLineState = 0U;
	UsbdCdcCancelBusState(pIntrf);

	if (Configuration == 0U)
	{
		return true;
	}

	if (Configuration != USBD_CDC_CONFIG_VALUE)
	{
		return false;
	}

	pIntrf->BulkMps = UsbdCdcMps();

	if (!UsbdCdcOpenEndpoint(USBD_CDC_NOTIF_EP(pIntrf->ItfNo),
			USB_ENDPATT_TRANS_INT, USBD_CDC_NOTIF_MPS,
			UsbdCdcNotifInterval()) ||
		!UsbdCdcOpenEndpoint(USBD_CDC_DATA_OUT_EP(pIntrf->ItfNo),
			USB_ENDPATT_TRANS_BULK, pIntrf->BulkMps, 0U) ||
		!UsbdCdcOpenEndpoint(USBD_CDC_DATA_IN_EP(pIntrf->ItfNo),
			USB_ENDPATT_TRANS_BULK, pIntrf->BulkMps, 0U))
	{
		UsbdCdcCloseEndpoints(pIntrf);
		return false;
	}

	pIntrf->Configured = true;
	pIntrf->SerialStatePending = true;
	UsbdCdcNotifKick(pIntrf);

	return true;
}

static bool UsbdCdcRequest(const UsbSetupData_t *pSetup,
						   UsbdCoreCtrlStage_t Stage,
						   uint8_t **ppData,
						   uint16_t *pLength,
						   void *pContext)
{
	UsbdCdcDevIntrf_t *pIntrf = static_cast<UsbdCdcDevIntrf_t *>(pContext);

	if (pSetup == nullptr || pIntrf == nullptr || pLength == nullptr ||
		!pIntrf->Configured ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) != USB_REQTYPE_CLASS ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT) != USB_REQTYPE_INTERFACE ||
		(pSetup->wIndex & 0xFF00U) != 0U ||
		(uint8_t)pSetup->wIndex != USBD_CDC_CTRL_INTRF(pIntrf->ItfNo))
	{
		return false;
	}

	switch (pSetup->bRequest)
	{
		case USB_CDC_REQ_SET_LINE_CODING:
			if ((pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) != USB_REQTYPE_DIRDEV ||
				pSetup->wValue != 0U || pSetup->wLength != sizeof(UsbCdcLineCoding_t))
			{
				return false;
			}

			if (Stage == USBD_CORE_CTRL_SETUP)
			{
				if (ppData == nullptr)
				{
					return false;
				}
				*ppData = reinterpret_cast<uint8_t *>(&pIntrf->PendingLineCoding);
				*pLength = sizeof(pIntrf->PendingLineCoding);
				return true;
			}

			if (Stage == USBD_CORE_CTRL_DATA)
			{
				return *pLength == sizeof(UsbCdcLineCoding_t);
			}

			if (Stage == USBD_CORE_CTRL_COMPLETE)
			{
				pIntrf->LineCoding = pIntrf->PendingLineCoding;
				return true;
			}
			return false;

		case USB_CDC_REQ_GET_LINE_CODING:
			if ((pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) != USB_REQTYPE_DIRHOST ||
				pSetup->wValue != 0U || pSetup->wLength != sizeof(UsbCdcLineCoding_t))
			{
				return false;
			}

			if (Stage == USBD_CORE_CTRL_SETUP)
			{
				if (ppData == nullptr)
				{
					return false;
				}
				*ppData = reinterpret_cast<uint8_t *>(&pIntrf->LineCoding);
				*pLength = sizeof(pIntrf->LineCoding);
			}
			return true;

		case USB_CDC_REQ_SET_CTRL_LINE_STATE:
			if ((pSetup->bmRequestType & USB_REQTYPE_MASK_DIR) != USB_REQTYPE_DIRDEV ||
				pSetup->wLength != 0U ||
				(pSetup->wValue &
				 ~(USB_CDC_CTRL_LINE_STATE_DTR | USB_CDC_CTRL_LINE_STATE_RTS)) != 0U)
			{
				return false;
			}

			if (Stage == USBD_CORE_CTRL_SETUP)
			{
				pIntrf->PendingControlLineState = pSetup->wValue;
				*pLength = 0U;
				return true;
			}

			if (Stage == USBD_CORE_CTRL_COMPLETE)
			{
				pIntrf->ControlLineState = pIntrf->PendingControlLineState;
				return true;
			}
			return Stage == USBD_CORE_CTRL_DATA;

		default:
			return false;
	}
}

static void UsbdCdcXfer(uint8_t EpAddr, uint16_t Length,
						UsbdCtrlrXferResult_t Result, void *pContext)
{
	UsbdCdcDevIntrf_t *pIntrf = static_cast<UsbdCdcDevIntrf_t *>(pContext);
	if (pIntrf == nullptr)
	{
		return;
	}

	if (EpAddr == USBD_CDC_DATA_OUT_EP(pIntrf->ItfNo))
	{
		pIntrf->RxActive = false;

		if (Result == USBD_CTRLR_XFER_SUCCESS && Length > 0U &&
			Length <= pIntrf->BulkMps)
		{
			const uint32_t put = UsbdCdcRxPut(pIntrf);
			const uint32_t get = UsbdCdcRxGet(pIntrf);
			if ((put - get) < USBD_CDC_RX_STAGE_COUNT)
			{
				UsbdCdcRxStage_t *pStage = UsbdCdcRxStage(pIntrf, put);
				pStage->Length = Length;
				__atomic_store_n(&pIntrf->RxPut, put + 1U, __ATOMIC_RELEASE);
			}
		}

		// A failed or zero-length OUT consumes no application FIFO space. Rearm
		// immediately when another private staging slot is available.
		UsbdCdcRxKick(&pIntrf->Bulk, pIntrf);
		return;
	}

	if (EpAddr == USBD_CDC_DATA_IN_EP(pIntrf->ItfNo))
	{
		pIntrf->TxCompleteLength = Length;
		pIntrf->TxCompleteExpected = pIntrf->TxZlpActive ? 0U : pIntrf->TxLength;
		pIntrf->TxCompleteResult = Result;

		// A successful payload is already owned by the host. Clear its staged
		// length before publishing completion so a simultaneous USB reset cannot
		// cause the same payload to be retransmitted after re-enumeration.
		if (!pIntrf->TxZlpActive && Result == USBD_CTRLR_XFER_SUCCESS)
		{
			pIntrf->TxLength = 0U;
		}

		__atomic_store_n(&pIntrf->TxCompletePending, true, __ATOMIC_RELEASE);
		return;
	}

	if (EpAddr == USBD_CDC_NOTIF_EP(pIntrf->ItfNo))
	{
		pIntrf->NotifActive = false;
		if (Result == USBD_CTRLR_XFER_SUCCESS)
		{
			UsbdCdcNotifKick(pIntrf);
		}
	}
}

static void UsbdCdcReset(void *pContext)
{
	UsbdCdcDevIntrf_t *pIntrf = static_cast<UsbdCdcDevIntrf_t *>(pContext);
	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->Configured = false;
	pIntrf->ControlLineState = 0U;
	pIntrf->PendingControlLineState = 0U;
	pIntrf->SerialState = 0U;
	UsbdCdcCancelBusState(pIntrf);
	UsbdCdcDefaultLineCoding(pIntrf);
}

static void UsbdCdcProcessRx(UsbdCdcDevIntrf_t *pIntrf)
{
	if (!pIntrf->Bulk.bEnabled)
	{
		return;
	}

	for (;;)
	{
		const uint32_t get = UsbdCdcRxGet(pIntrf);
		const uint32_t put = UsbdCdcRxPut(pIntrf);
		if (get == put)
		{
			break;
		}

		UsbdCdcRxStage_t *pStage = UsbdCdcRxStage(pIntrf, get);
		const uint16_t length = pStage->Length;

		if (pIntrf->Bulk.hRxFifo->bBlocking &&
			CFifoAvail(pIntrf->Bulk.hRxFifo) < length)
		{
			break;
		}

		if (length > 0U)
		{
			const int accepted = UsbdBulkIntrfPutRxData(
				&pIntrf->Bulk, UsbdCdcRxBuffer(pStage), length);
			if (accepted != length)
			{
				// Blocking FIFOs are checked above; a short write can only mean
				// the application violated CFifo's single-context requirement.
				break;
			}
		}

		pStage->Length = 0U;
		__atomic_store_n(&pIntrf->RxGet, get + 1U, __ATOMIC_RELEASE);
	}
}

static void UsbdCdcReportTxFailure(UsbdCdcDevIntrf_t *pIntrf, uint16_t Length)
{
	if (pIntrf->Bulk.DevIntrf.EvtCB != nullptr)
	{
		pIntrf->Bulk.DevIntrf.EvtCB(&pIntrf->Bulk.DevIntrf,
									DEVINTRF_EVT_TX_TIMEOUT,
									nullptr, Length);
	}
}

static void UsbdCdcTryZlp(UsbdCdcDevIntrf_t *pIntrf)
{
	if (!pIntrf->TxZlpRequired || pIntrf->TxZlpActive ||
		!pIntrf->Bulk.bEnabled || !UsbdCdcIntrfPortIsOpen(pIntrf))
	{
		return;
	}

	// If more data arrived before the terminating ZLP was started, it is a
	// continuation of the same byte stream. Suppress the ZLP and send it.
	if (UsbdBulkIntrfTxUsed(&pIntrf->Bulk) > 0)
	{
		pIntrf->TxZlpRequired = false;
		UsbdBulkIntrfTxComplete(&pIntrf->Bulk);
		return;
	}

	// nRF EasyDMA still captures EPIN.PTR for MAXCNT=0. Keep the pointer in
	// aligned RAM instead of passing nullptr.
	pIntrf->TxZlpRequired = false;
	pIntrf->TxZlpActive = true;
	if (!UsbdCtrlrEpXfer(USBD_CDC_DATA_IN_EP(pIntrf->ItfNo),
						 UsbdCdcTxBuffer(pIntrf), 0U))
	{
		pIntrf->TxZlpActive = false;
		pIntrf->TxZlpRequired = true;
	}
}

static void UsbdCdcProcessTx(UsbdCdcDevIntrf_t *pIntrf)
{
	if (__atomic_exchange_n(&pIntrf->TxCompletePending, false,
							__ATOMIC_ACQ_REL))
	{
		const UsbdCtrlrXferResult_t result = pIntrf->TxCompleteResult;
		const uint16_t length = pIntrf->TxCompleteLength;
		const uint16_t expected = pIntrf->TxCompleteExpected;

		if (pIntrf->TxZlpActive)
		{
			pIntrf->TxZlpActive = false;
			if (result == USBD_CTRLR_XFER_SUCCESS && length == 0U)
			{
				UsbdBulkIntrfTxComplete(&pIntrf->Bulk);
			}
			else
			{
				// Retrying a ZLP cannot duplicate payload bytes.
				pIntrf->TxZlpRequired = true;
			}
		}
		else if (pIntrf->TxActive)
		{
			pIntrf->TxActive = false;

			if (result == USBD_CTRLR_XFER_SUCCESS && length == expected)
			{
				pIntrf->TxLength = 0U;
				if (length != 0U && (length % pIntrf->BulkMps) == 0U &&
					UsbdBulkIntrfTxUsed(&pIntrf->Bulk) == 0)
				{
					pIntrf->TxZlpRequired = true;
				}
				else
				{
					UsbdBulkIntrfTxComplete(&pIntrf->Bulk);
				}
			}
			else if (length == 0U)
			{
				// No payload byte reached the host, so retrying the staged
				// request is safe. TxLength intentionally remains unchanged.
				UsbdCdcTxKick(&pIntrf->Bulk, pIntrf);
			}
			else
			{
				// Some bytes may already have reached the host. Never resend the
				// whole staged packet after a short or failed completion.
				pIntrf->TxLength = 0U;
				UsbdCdcReportTxFailure(pIntrf, length);
				UsbdBulkIntrfTxComplete(&pIntrf->Bulk);
			}
		}
	}

	UsbdCdcTryZlp(pIntrf);
}

static void UsbdCdcPump(void *pContext)
{
	UsbdCdcIntrfProcess(static_cast<UsbdCdcDevIntrf_t *>(pContext));
}

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t * const pIntrf,
					  const UsbdCdcIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr ||
		pCfg->ItfNo < 0 || pCfg->ItfNo >= USBD_CDC_FUNC_MAXCNT ||
		pCfg->pRxFifoMem == nullptr || pCfg->pTxFifoMem == nullptr)
	{
		return false;
	}

	const UsbDevCfg_t *pDevCfg = UsbDevGetCfg();
	if (pDevCfg == nullptr || pCfg->ItfNo >= pDevCfg->NbCdc)
	{
		return false;
	}

	UsbdBulkIntrfCfg_t bulkCfg = {};
	bulkCfg.bBlocking = pCfg->bBlocking;
	bulkCfg.RxFifoMemSize = pCfg->RxFifoMemSize;
	bulkCfg.pRxFifoMem = pCfg->pRxFifoMem;
	bulkCfg.TxFifoMemSize = pCfg->TxFifoMemSize;
	bulkCfg.pTxFifoMem = pCfg->pTxFifoMem;
	bulkCfg.RxKick = UsbdCdcRxKick;
	bulkCfg.TxKick = UsbdCdcTxKick;
	bulkCfg.pContext = pIntrf;
	bulkCfg.EvtCB = pCfg->EvtCB;

	if (!UsbdBulkIntrfInit(&pIntrf->Bulk, &bulkCfg))
	{
		return false;
	}

	//
	// The CDC half of the instance is cleared field by field rather than with
	// one memset. The Bulk member holds the DeviceIntrf atomics, which
	// UsbdBulkIntrfInit has just set up, and clearing an object that holds
	// them by writing raw zeroes over it is not defined.
	//
	pIntrf->ItfNo = pCfg->ItfNo;
	pIntrf->BulkMps = UsbdCdcMps();
	pIntrf->ControlLineState = 0U;
	pIntrf->PendingControlLineState = 0U;
	pIntrf->SerialState = 0U;
	pIntrf->TxLength = 0U;
	pIntrf->TxCompleteLength = 0U;
	pIntrf->TxCompleteExpected = 0U;
	pIntrf->TxCompleteResult = USBD_CTRLR_XFER_SUCCESS;
	pIntrf->RxDropCnt = 0;
	pIntrf->TxDropCnt = 0;
	pIntrf->Configured = false;
	pIntrf->ReportedOpen = false;
	UsbdCdcCancelBusState(pIntrf);
	UsbdCdcDefaultLineCoding(pIntrf);

	for (uint32_t i = 0; i < USBD_CDC_RX_STAGE_COUNT; i++)
	{
		pIntrf->RxStage[i].Length = 0U;
	}

	__atomic_store_n(&pIntrf->RxPut, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->RxGet, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pIntrf->TxCompletePending, false, __ATOMIC_RELEASE);

	UsbdCoreFuncCfg_t coreCfg = {};
	coreCfg.FirstInterface = USBD_CDC_CTRL_INTRF(pCfg->ItfNo);
	coreCfg.InterfaceCount = 2U;
	coreCfg.EpInMask =
		(uint16_t)((1U << USB_ENDPADDR_NUM(USBD_CDC_NOTIF_EP(pCfg->ItfNo))) |
				   (1U << USB_ENDPADDR_NUM(USBD_CDC_DATA_IN_EP(pCfg->ItfNo))));
	coreCfg.EpOutMask =
		(uint16_t)(1U << USB_ENDPADDR_NUM(USBD_CDC_DATA_OUT_EP(pCfg->ItfNo)));
	coreCfg.RequestHandler = UsbdCdcRequest;
	coreCfg.ConfigHandler = UsbdCdcConfig;
	coreCfg.SetInterfaceHandler = nullptr;
	coreCfg.XferHandler = UsbdCdcXfer;
	coreCfg.ResetHandler = UsbdCdcReset;
	coreCfg.pContext = pIntrf;

	if (!UsbdCoreRegisterFunction(&coreCfg))
	{
		return false;
	}

	if (!UsbDevRegisterFunc(UsbdCdcPump, pIntrf))
	{
		return false;
	}

	return true;
}

void UsbdCdcIntrfProcess(UsbdCdcDevIntrf_t * const pIntrf)
{
	if (pIntrf == nullptr || pIntrf->Bulk.hRxFifo == nullptr)
	{
		return;
	}

	const bool open = UsbdCdcIntrfPortIsOpen(pIntrf);
	if (open != pIntrf->ReportedOpen)
	{
		pIntrf->ReportedOpen = open;
		UsbdCdcNotifyPortState(pIntrf, open);
	}

	UsbdCdcProcessRx(pIntrf);
	UsbdCdcProcessTx(pIntrf);
	UsbdCdcNotifKick(pIntrf);

	if (pIntrf->Bulk.bEnabled && open)
	{
		UsbdCdcRxKick(&pIntrf->Bulk, pIntrf);
		UsbdCdcTxKick(&pIntrf->Bulk, pIntrf);
	}

	pIntrf->RxDropCnt = pIntrf->Bulk.hRxFifo->DropCnt;
	pIntrf->TxDropCnt = pIntrf->Bulk.hTxFifo->DropCnt;
}

const UsbCdcLineCoding_t *UsbdCdcIntrfLineCoding(
									const UsbdCdcDevIntrf_t * const pIntrf)
{
	return pIntrf != nullptr ? &pIntrf->LineCoding : nullptr;
}

uint16_t UsbdCdcIntrfControlLineState(const UsbdCdcDevIntrf_t * const pIntrf)
{
	return pIntrf != nullptr ? pIntrf->ControlLineState : 0U;
}

void UsbdCdcIntrfSetSerialState(UsbdCdcDevIntrf_t * const pIntrf,
								uint16_t SerialState)
{
	if (pIntrf == nullptr)
	{
		return;
	}

	pIntrf->SerialState = SerialState & UsbdCdcSerialStateMask();
	pIntrf->SerialStatePending = true;
	UsbdCdcNotifKick(pIntrf);
}

bool UsbdCdcIntrf::Init(const UsbdCdcIntrfCfg_t &Cfg)
{
	return UsbdCdcIntrfInit(&vUsbDevIntrf, &Cfg);
}

bool UsbdCdcIntrf::IsPortOpen(void)
{
	return UsbdCdcIntrfPortIsOpen(&vUsbDevIntrf);
}

const UsbCdcLineCoding_t *UsbdCdcIntrf::LineCoding(void)
{
	return UsbdCdcIntrfLineCoding(&vUsbDevIntrf);
}

uint16_t UsbdCdcIntrf::ControlLineState(void)
{
	return UsbdCdcIntrfControlLineState(&vUsbDevIntrf);
}

void UsbdCdcIntrf::SetSerialState(uint16_t SerialState)
{
	UsbdCdcIntrfSetSerialState(&vUsbDevIntrf, SerialState);
}

bool UsbdCdcIntrf::RequestToSend(int NbBytes)
{
	return UsbdBulkIntrfRequestToSend(&vUsbDevIntrf.Bulk, NbBytes);
}
