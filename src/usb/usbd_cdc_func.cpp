/**-------------------------------------------------------------------------
@file	usbd_cdc_func.cpp

@brief	Native USB CDC ACM function implementation.

@author	Hoang Nguyen Hoan
@date	Aug. 30, 2026

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

#include "usbd_cdc_priv.h"

static_assert(sizeof(UsbCdcLineCoding_t) == 7U, "CDC line coding size");
static_assert(sizeof(UsbCdcNotification_t) == 8U, "CDC notification header size");

static UsbdCdcRxStage_t *UsbdCdcFuncRxStage(UsbdCdcFunc_t *pFunc,
										 uint32_t Index)
{
	return &pFunc->RxStage[Index % USBD_CDC_RX_STAGE_COUNT];
}

static uint8_t *UsbdCdcFuncRxBuffer(UsbdCdcRxStage_t *pStage)
{
	return reinterpret_cast<uint8_t *>(pStage->Data);
}

static uint8_t *UsbdCdcFuncTxBuffer(UsbdCdcFunc_t *pFunc)
{
	return reinterpret_cast<uint8_t *>(pFunc->TxTransfer);
}

static uint8_t *UsbdCdcFuncNotifBuffer(UsbdCdcFunc_t *pFunc)
{
	return reinterpret_cast<uint8_t *>(pFunc->NotifTransfer);
}

static uint32_t UsbdCdcFuncRxPut(const UsbdCdcFunc_t *pFunc)
{
	return __atomic_load_n(&pFunc->RxPut, __ATOMIC_ACQUIRE);
}

static uint32_t UsbdCdcFuncRxGet(const UsbdCdcFunc_t *pFunc)
{
	return __atomic_load_n(&pFunc->RxGet, __ATOMIC_ACQUIRE);
}

static bool UsbdCdcFuncTxCompletionPending(const UsbdCdcFunc_t *pFunc)
{
	return __atomic_load_n(&pFunc->TxCompletePending, __ATOMIC_ACQUIRE);
}

static uint16_t UsbdCdcFuncSerialStateMask(void)
{
	return USB_CDC_SERIAL_STATE_RX_CARRIER |
		USB_CDC_SERIAL_STATE_TX_CARRIER |
		USB_CDC_SERIAL_STATE_BREAK |
		USB_CDC_SERIAL_STATE_RING |
		USB_CDC_SERIAL_STATE_FRAMING |
		USB_CDC_SERIAL_STATE_PARITY |
		USB_CDC_SERIAL_STATE_OVERRUN;
}

static void UsbdCdcFuncDefaultLineCoding(UsbdCdcFunc_t *pFunc)
{
	pFunc->LineCoding.dwDTERate = 115200U;
	pFunc->LineCoding.bCharFormat = USB_CDC_LINE_STOP_1;
	pFunc->LineCoding.bParityType = USB_CDC_LINE_PARITY_NONE;
	pFunc->LineCoding.bDataBits = 8U;
	pFunc->PendingLineCoding = pFunc->LineCoding;
}

static uint16_t UsbdCdcFuncMps(void)
{
	return UsbdCtrlrHighSpeed() ?
		USBD_CDC_BULK_HS_MPS : USBD_CDC_BULK_FS_MPS;
}

static uint8_t UsbdCdcFuncNotifInterval(void)
{
	return UsbdCtrlrHighSpeed() ? 8U : 16U;
}

bool UsbdCdcFuncPortIsOpen(const UsbdCdcFunc_t *pFunc)
{
	return pFunc != nullptr && pFunc->Configured &&
		(pFunc->ControlLineState & USB_CDC_CTRL_LINE_STATE_DTR) != 0U;
}

static void UsbdCdcFuncNotifyPortState(UsbdCdcFunc_t *pFunc, bool Open)
{
	if (pFunc->pBulk != nullptr && pFunc->pBulk->DevIntrf.EvtCB != nullptr)
	{
		pFunc->pBulk->DevIntrf.EvtCB(&pFunc->pBulk->DevIntrf,
									  DEVINTRF_EVT_STATECHG,
									  nullptr, Open ? 1 : 0);
	}
}

static void UsbdCdcFuncRxKick(UsbdBulkDevIntrf_t * const pBulk,
							   void *pContext)
{
	UsbdCdcFunc_t *pFunc = static_cast<UsbdCdcFunc_t *>(pContext);

	if (pFunc == nullptr || pFunc->pBulk != pBulk ||
		!pBulk->bEnabled || !UsbdCdcFuncPortIsOpen(pFunc) ||
		pFunc->RxActive)
	{
		return;
	}

	const uint32_t put = UsbdCdcFuncRxPut(pFunc);
	const uint32_t get = UsbdCdcFuncRxGet(pFunc);
	if ((put - get) >= USBD_CDC_RX_STAGE_COUNT)
	{
		return;
	}

	UsbdCdcRxStage_t *pStage = UsbdCdcFuncRxStage(pFunc, put);
	pFunc->RxActive = true;
	if (!UsbdCtrlrEpXfer(USBD_CDC_DATA_OUT_EP(pFunc->FunctionNo),
						 UsbdCdcFuncRxBuffer(pStage), pFunc->BulkMps))
	{
		pFunc->RxActive = false;
	}
}

static void UsbdCdcFuncTxKick(UsbdBulkDevIntrf_t * const pBulk,
							   void *pContext)
{
	UsbdCdcFunc_t *pFunc = static_cast<UsbdCdcFunc_t *>(pContext);

	if (pFunc == nullptr || pFunc->pBulk != pBulk ||
		!pBulk->bEnabled || !UsbdCdcFuncPortIsOpen(pFunc) ||
		pFunc->TxActive || pFunc->TxZlpActive || pFunc->TxZlpRequired ||
		UsbdCdcFuncTxCompletionPending(pFunc))
	{
		return;
	}

	if (pFunc->TxLength == 0U)
	{
		const int length = UsbdBulkIntrfGetTxData(pBulk,
			UsbdCdcFuncTxBuffer(pFunc), pFunc->BulkMps);
		if (length <= 0)
		{
			return;
		}

		pFunc->TxLength = (uint16_t)length;
	}

	pFunc->TxActive = true;
	if (!UsbdCtrlrEpXfer(USBD_CDC_DATA_IN_EP(pFunc->FunctionNo),
						 UsbdCdcFuncTxBuffer(pFunc), pFunc->TxLength))
	{
		pFunc->TxActive = false;
	}
}

static void UsbdCdcFuncNotifKick(UsbdCdcFunc_t *pFunc)
{
	if (pFunc == nullptr || !pFunc->Configured ||
		pFunc->NotifActive || !pFunc->SerialStatePending)
	{
		return;
	}

	UsbCdcNotification_t notification = {};
	notification.bmRequestType =
		USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS | USB_REQTYPE_INTERFACE;
	notification.bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notification.wValue = 0U;
	notification.wIndex = USBD_CDC_CTRL_INTRF(pFunc->FunctionNo);
	notification.wLength = 2U;

	uint8_t *pData = UsbdCdcFuncNotifBuffer(pFunc);
	memcpy(pData, &notification, sizeof(notification));
	pData[sizeof(notification)] = (uint8_t)pFunc->SerialState;
	pData[sizeof(notification) + 1U] = (uint8_t)(pFunc->SerialState >> 8);

	pFunc->SerialStatePending = false;
	pFunc->NotifActive = true;
	if (!UsbdCtrlrEpXfer(USBD_CDC_NOTIF_EP(pFunc->FunctionNo),
						 pData, USBD_CDC_NOTIFY_LEN))
	{
		pFunc->NotifActive = false;
		pFunc->SerialStatePending = true;
	}
}

static bool UsbdCdcFuncOpenEndpoint(uint8_t EpAddr, uint8_t Type,
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

static void UsbdCdcFuncCloseEndpoints(UsbdCdcFunc_t *pFunc)
{
	UsbdCtrlrEpClose(USBD_CDC_NOTIF_EP(pFunc->FunctionNo));
	UsbdCtrlrEpClose(USBD_CDC_DATA_OUT_EP(pFunc->FunctionNo));
	UsbdCtrlrEpClose(USBD_CDC_DATA_IN_EP(pFunc->FunctionNo));
}

static void UsbdCdcFuncCancelBusState(UsbdCdcFunc_t *pFunc)
{
	pFunc->RxActive = false;
	pFunc->TxActive = false;
	pFunc->TxZlpActive = false;
	pFunc->TxZlpRequired = false;
	pFunc->NotifActive = false;
	pFunc->SerialStatePending = false;
	__atomic_store_n(&pFunc->TxCompletePending, false, __ATOMIC_RELEASE);
}

static bool UsbdCdcFuncConfig(uint8_t Configuration, void *pContext)
{
	UsbdCdcFunc_t *pFunc = static_cast<UsbdCdcFunc_t *>(pContext);
	if (pFunc == nullptr || pFunc->pBulk == nullptr)
	{
		return false;
	}

	pFunc->Configured = false;
	pFunc->ControlLineState = 0U;
	pFunc->PendingControlLineState = 0U;
	UsbdCdcFuncCancelBusState(pFunc);

	if (Configuration == 0U)
	{
		return true;
	}

	if (Configuration != 1U)
	{
		return false;
	}

	pFunc->BulkMps = UsbdCdcFuncMps();

	if (!UsbdCdcFuncOpenEndpoint(USBD_CDC_NOTIF_EP(pFunc->FunctionNo),
			USB_ENDPATT_TRANS_INT, USBD_CDC_NOTIF_MPS,
			UsbdCdcFuncNotifInterval()) ||
		!UsbdCdcFuncOpenEndpoint(USBD_CDC_DATA_OUT_EP(pFunc->FunctionNo),
			USB_ENDPATT_TRANS_BULK, pFunc->BulkMps, 0U) ||
		!UsbdCdcFuncOpenEndpoint(USBD_CDC_DATA_IN_EP(pFunc->FunctionNo),
			USB_ENDPATT_TRANS_BULK, pFunc->BulkMps, 0U))
	{
		UsbdCdcFuncCloseEndpoints(pFunc);
		return false;
	}

	pFunc->Configured = true;
	pFunc->SerialStatePending = true;
	UsbdCdcFuncNotifKick(pFunc);

	return true;
}

static void UsbdCdcFuncApplyControlLineState(UsbdCdcFunc_t *pFunc)
{
	pFunc->ControlLineState = pFunc->PendingControlLineState;
}

static bool UsbdCdcFuncRequest(const UsbSetupData_t *pSetup,
							   UsbdCoreCtrlStage_t Stage,
							   uint8_t **ppData,
							   uint16_t *pLength,
							   void *pContext)
{
	UsbdCdcFunc_t *pFunc = static_cast<UsbdCdcFunc_t *>(pContext);

	if (pSetup == nullptr || pFunc == nullptr || pLength == nullptr ||
		!pFunc->Configured ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) != USB_REQTYPE_CLASS ||
		(pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT) != USB_REQTYPE_INTERFACE ||
		(pSetup->wIndex & 0xFF00U) != 0U ||
		(uint8_t)pSetup->wIndex != USBD_CDC_CTRL_INTRF(pFunc->FunctionNo))
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
				*ppData = reinterpret_cast<uint8_t *>(&pFunc->PendingLineCoding);
				*pLength = sizeof(pFunc->PendingLineCoding);
				return true;
			}

			if (Stage == USBD_CORE_CTRL_DATA)
			{
				return *pLength == sizeof(UsbCdcLineCoding_t);
			}

			if (Stage == USBD_CORE_CTRL_COMPLETE)
			{
				pFunc->LineCoding = pFunc->PendingLineCoding;
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
				*ppData = reinterpret_cast<uint8_t *>(&pFunc->LineCoding);
				*pLength = sizeof(pFunc->LineCoding);
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
				pFunc->PendingControlLineState = pSetup->wValue;
				*pLength = 0U;
				return true;
			}

			if (Stage == USBD_CORE_CTRL_COMPLETE)
			{
				UsbdCdcFuncApplyControlLineState(pFunc);
				return true;
			}
			return Stage == USBD_CORE_CTRL_DATA;

		default:
			return false;
	}
}

static void UsbdCdcFuncXfer(uint8_t EpAddr, uint16_t Length,
							UsbdCtrlrXferResult_t Result, void *pContext)
{
	UsbdCdcFunc_t *pFunc = static_cast<UsbdCdcFunc_t *>(pContext);
	if (pFunc == nullptr || pFunc->pBulk == nullptr)
	{
		return;
	}

	if (EpAddr == USBD_CDC_DATA_OUT_EP(pFunc->FunctionNo))
	{
		pFunc->RxActive = false;

		if (Result == USBD_CTRLR_XFER_SUCCESS && Length > 0U &&
			Length <= pFunc->BulkMps)
		{
			const uint32_t put = UsbdCdcFuncRxPut(pFunc);
			const uint32_t get = UsbdCdcFuncRxGet(pFunc);
			if ((put - get) < USBD_CDC_RX_STAGE_COUNT)
			{
				UsbdCdcRxStage_t *pStage = UsbdCdcFuncRxStage(pFunc, put);
				pStage->Length = Length;
				__atomic_store_n(&pFunc->RxPut, put + 1U, __ATOMIC_RELEASE);
			}
		}

		// A failed or zero-length OUT consumes no application FIFO space. Rearm
		// immediately when another private staging slot is available.
		UsbdCdcFuncRxKick(pFunc->pBulk, pFunc);
		return;
	}

	if (EpAddr == USBD_CDC_DATA_IN_EP(pFunc->FunctionNo))
	{
		pFunc->TxCompleteLength = Length;
		pFunc->TxCompleteExpected = pFunc->TxZlpActive ? 0U : pFunc->TxLength;
		pFunc->TxCompleteResult = Result;

		// A successful payload is already owned by the host. Clear its staged
		// length before publishing completion so a simultaneous USB reset cannot
		// cause the same payload to be retransmitted after re-enumeration.
		if (!pFunc->TxZlpActive && Result == USBD_CTRLR_XFER_SUCCESS)
		{
			pFunc->TxLength = 0U;
		}

		__atomic_store_n(&pFunc->TxCompletePending, true, __ATOMIC_RELEASE);
		return;
	}

	if (EpAddr == USBD_CDC_NOTIF_EP(pFunc->FunctionNo))
	{
		pFunc->NotifActive = false;
		if (Result == USBD_CTRLR_XFER_SUCCESS)
		{
			UsbdCdcFuncNotifKick(pFunc);
		}
	}
}

static void UsbdCdcFuncReset(void *pContext)
{
	UsbdCdcFunc_t *pFunc = static_cast<UsbdCdcFunc_t *>(pContext);
	if (pFunc == nullptr)
	{
		return;
	}

	pFunc->Configured = false;
	pFunc->ControlLineState = 0U;
	pFunc->PendingControlLineState = 0U;
	pFunc->SerialState = 0U;
	UsbdCdcFuncCancelBusState(pFunc);
	UsbdCdcFuncDefaultLineCoding(pFunc);
}

static void UsbdCdcFuncProcessRx(UsbdCdcFunc_t *pFunc)
{
	if (!pFunc->pBulk->bEnabled)
	{
		return;
	}

	for (;;)
	{
		const uint32_t get = UsbdCdcFuncRxGet(pFunc);
		const uint32_t put = UsbdCdcFuncRxPut(pFunc);
		if (get == put)
		{
			break;
		}

		UsbdCdcRxStage_t *pStage = UsbdCdcFuncRxStage(pFunc, get);
		const uint16_t length = pStage->Length;

		if (pFunc->pBulk->hRxFifo->bBlocking &&
			CFifoAvail(pFunc->pBulk->hRxFifo) < length)
		{
			break;
		}

		if (length > 0U)
		{
			const int accepted = UsbdBulkIntrfPutRxData(
				pFunc->pBulk, UsbdCdcFuncRxBuffer(pStage), length);
			if (accepted != length)
			{
				// Blocking FIFOs are checked above; a short write can only mean
				// the application violated CFifo's single-context requirement.
				break;
			}
		}

		pStage->Length = 0U;
		__atomic_store_n(&pFunc->RxGet, get + 1U, __ATOMIC_RELEASE);
	}
}

static void UsbdCdcFuncReportTxFailure(UsbdCdcFunc_t *pFunc, uint16_t Length)
{
	if (pFunc->pBulk->DevIntrf.EvtCB != nullptr)
	{
		pFunc->pBulk->DevIntrf.EvtCB(&pFunc->pBulk->DevIntrf,
									  DEVINTRF_EVT_TX_TIMEOUT,
									  nullptr, Length);
	}
}

static void UsbdCdcFuncTryZlp(UsbdCdcFunc_t *pFunc)
{
	if (!pFunc->TxZlpRequired || pFunc->TxZlpActive ||
		!pFunc->pBulk->bEnabled || !UsbdCdcFuncPortIsOpen(pFunc))
	{
		return;
	}

	// If more data arrived before the terminating ZLP was started, it is a
	// continuation of the same byte stream. Suppress the ZLP and send it.
	if (UsbdBulkIntrfTxUsed(pFunc->pBulk) > 0)
	{
		pFunc->TxZlpRequired = false;
		UsbdBulkIntrfTxComplete(pFunc->pBulk);
		return;
	}

	// nRF EasyDMA still captures EPIN.PTR for MAXCNT=0. Keep the pointer in
	// aligned RAM instead of passing nullptr.
	pFunc->TxZlpRequired = false;
	pFunc->TxZlpActive = true;
	if (!UsbdCtrlrEpXfer(USBD_CDC_DATA_IN_EP(pFunc->FunctionNo),
						 UsbdCdcFuncTxBuffer(pFunc), 0U))
	{
		pFunc->TxZlpActive = false;
		pFunc->TxZlpRequired = true;
	}
}

static void UsbdCdcFuncProcessTx(UsbdCdcFunc_t *pFunc)
{
	if (__atomic_exchange_n(&pFunc->TxCompletePending, false,
							__ATOMIC_ACQ_REL))
	{
		const UsbdCtrlrXferResult_t result = pFunc->TxCompleteResult;
		const uint16_t length = pFunc->TxCompleteLength;
		const uint16_t expected = pFunc->TxCompleteExpected;

		if (pFunc->TxZlpActive)
		{
			pFunc->TxZlpActive = false;
			if (result == USBD_CTRLR_XFER_SUCCESS && length == 0U)
			{
				UsbdBulkIntrfTxComplete(pFunc->pBulk);
			}
			else
			{
				// Retrying a ZLP cannot duplicate payload bytes.
				pFunc->TxZlpRequired = true;
			}
		}
		else if (pFunc->TxActive)
		{
			pFunc->TxActive = false;

			if (result == USBD_CTRLR_XFER_SUCCESS && length == expected)
			{
				pFunc->TxLength = 0U;
				if (length != 0U && (length % pFunc->BulkMps) == 0U &&
					UsbdBulkIntrfTxUsed(pFunc->pBulk) == 0)
				{
					pFunc->TxZlpRequired = true;
				}
				else
				{
					UsbdBulkIntrfTxComplete(pFunc->pBulk);
				}
			}
			else if (length == 0U)
			{
				// No payload byte reached the host, so retrying the staged
				// request is safe. TxLength intentionally remains unchanged.
				UsbdCdcFuncTxKick(pFunc->pBulk, pFunc);
			}
			else
			{
				// Some bytes may already have reached the host. Never resend the
				// whole staged packet after a short or failed completion.
				pFunc->TxLength = 0U;
				UsbdCdcFuncReportTxFailure(pFunc, length);
				UsbdBulkIntrfTxComplete(pFunc->pBulk);
			}
		}
	}

	UsbdCdcFuncTryZlp(pFunc);
}

bool UsbdCdcFuncInit(UsbdCdcFunc_t *pFunc, int FunctionNo,
					 UsbdBulkDevIntrf_t *pBulk)
{
	if (pFunc == nullptr || pBulk == nullptr ||
		pBulk->hRxFifo == nullptr || pBulk->hTxFifo == nullptr ||
		FunctionNo < 0 || FunctionNo >= USBD_CDC_FUNC_MAXCNT)
	{
		return false;
	}

	memset(pFunc, 0, sizeof(*pFunc));
	pFunc->pBulk = pBulk;
	pFunc->FunctionNo = FunctionNo;
	pFunc->BulkMps = UsbdCdcFuncMps();
	UsbdCdcFuncDefaultLineCoding(pFunc);
	__atomic_store_n(&pFunc->RxPut, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pFunc->RxGet, 0U, __ATOMIC_RELEASE);
	__atomic_store_n(&pFunc->TxCompletePending, false, __ATOMIC_RELEASE);

	UsbdCoreFuncCfg_t cfg = {};
	cfg.FirstInterface = USBD_CDC_CTRL_INTRF(FunctionNo);
	cfg.InterfaceCount = 2U;
	cfg.EpInMask =
		(uint16_t)((1U << USB_ENDPADDR_NUM(USBD_CDC_NOTIF_EP(FunctionNo))) |
				   (1U << USB_ENDPADDR_NUM(USBD_CDC_DATA_IN_EP(FunctionNo))));
	cfg.EpOutMask =
		(uint16_t)(1U << USB_ENDPADDR_NUM(USBD_CDC_DATA_OUT_EP(FunctionNo)));
	cfg.RequestHandler = UsbdCdcFuncRequest;
	cfg.ConfigHandler = UsbdCdcFuncConfig;
	cfg.SetInterfaceHandler = nullptr;
	cfg.XferHandler = UsbdCdcFuncXfer;
	cfg.ResetHandler = UsbdCdcFuncReset;
	cfg.pContext = pFunc;

	if (!UsbdCoreRegisterFunction(&cfg))
	{
		pFunc->pBulk = nullptr;
		return false;
	}

	pBulk->RxKick = UsbdCdcFuncRxKick;
	pBulk->TxKick = UsbdCdcFuncTxKick;
	pBulk->pContext = pFunc;

	return true;
}

void UsbdCdcFuncProcess(UsbdCdcFunc_t *pFunc)
{
	if (pFunc == nullptr || pFunc->pBulk == nullptr)
	{
		return;
	}

	const bool open = UsbdCdcFuncPortIsOpen(pFunc);
	if (open != pFunc->ReportedOpen)
	{
		pFunc->ReportedOpen = open;
		UsbdCdcFuncNotifyPortState(pFunc, open);
	}

	UsbdCdcFuncProcessRx(pFunc);
	UsbdCdcFuncProcessTx(pFunc);
	UsbdCdcFuncNotifKick(pFunc);

	if (pFunc->pBulk->bEnabled && open)
	{
		UsbdCdcFuncRxKick(pFunc->pBulk, pFunc);
		UsbdCdcFuncTxKick(pFunc->pBulk, pFunc);
	}
}

const UsbCdcLineCoding_t *UsbdCdcFuncLineCoding(const UsbdCdcFunc_t *pFunc)
{
	return pFunc != nullptr ? &pFunc->LineCoding : nullptr;
}

uint16_t UsbdCdcFuncControlLineState(const UsbdCdcFunc_t *pFunc)
{
	return pFunc != nullptr ? pFunc->ControlLineState : 0U;
}

void UsbdCdcFuncSetSerialState(UsbdCdcFunc_t *pFunc, uint16_t SerialState)
{
	if (pFunc == nullptr)
	{
		return;
	}

	pFunc->SerialState = SerialState & UsbdCdcFuncSerialStateMask();
	pFunc->SerialStatePending = true;
	UsbdCdcFuncNotifKick(pFunc);
}
