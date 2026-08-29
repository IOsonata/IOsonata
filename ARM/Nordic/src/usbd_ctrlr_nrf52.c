/**-------------------------------------------------------------------------
@file	usbd_ctrlr_nrf52.c

@brief	Native IOsonata USB device controller for Nordic nRF52 USBD.

Implements usbd_ctrlr.h directly on the nRF52840 USBD peripheral. It owns
endpoint state, EasyDMA arbitration and USBD_IRQHandler. USB power, VBUS and
clock sequencing remain in usbd_nrfx.cpp.

Only control, bulk and interrupt endpoints are implemented here. Isochronous
support is intentionally left out of the first native USB milestone.

The nRF52840 has one USBD EasyDMA engine shared by all endpoints. Pending DMA
work is kept as endpoint bit masks. Control tasks are serviced first, then OUT
endpoints, then IN endpoints so a ready OUT endpoint is rearmed promptly.

Errata 199 is applied around every EasyDMA task.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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
#include <stdatomic.h>
#include <string.h>

#include "nrf.h"
#include "nrf_peripherals.h"
#include "nrf_erratas.h"

#include "usb/usbd_ctrlr.h"

#ifndef USBD_PRESENT
#error "usbd_ctrlr_nrf52: this part has no USBD peripheral"
#endif

enum
{
	NRFX_USBD_EP_COUNT = 8,
	NRFX_USBD_MAX_PACKET_SIZE = 64,
};

#define NRFX_USBD_EDPT_END_ALL_MASK \
	((0xFFUL << USBD_INTEN_ENDEPIN0_Pos) | \
	 (0xFFUL << USBD_INTEN_ENDEPOUT0_Pos))

#define NRFX_USBD_IRQ_EVENT_COUNT	(USBD_INTEN_EPDATA_Pos + 1)

#define NRFX_USBD_ERRATA_199_REG \
	(*((volatile uint32_t *)0x40027C1CUL))

typedef struct __nRF_Usbd_Xfer
{
	uint8_t *pBuffer;
	uint16_t TotalLen;
	volatile uint16_t ActualLen;
	uint16_t Mps;
	volatile bool DataReceived;
	volatile bool Started;
} nRFUsbdXfer_t;

typedef struct __nRF_Usbd_Ctrlr
{
	nRFUsbdXfer_t Xfer[NRFX_USBD_EP_COUNT][2];
	UsbdCtrlrEvtHandler_t EvtHandler;
	void *pContext;
	bool SofEnabled;
	bool SetupDirIn;
} nRFUsbdCtrlr_t;

static nRFUsbdCtrlr_t s_Ctrlr;
static atomic_flag s_DmaRunning = ATOMIC_FLAG_INIT;
static atomic_uint_fast16_t s_PendingOut;
static atomic_uint_fast16_t s_PendingIn;
static atomic_bool s_PendingEp0Status;
static atomic_bool s_PendingEp0RcvOut;

static inline uint8_t nRFUsbdDir(uint8_t EpAddr)
{
	return USBD_EP_IS_IN(EpAddr) ? 1U : 0U;
}

static inline nRFUsbdXfer_t *nRFUsbdGetXfer(uint8_t EpAddr)
{
	return &s_Ctrlr.Xfer[USBD_EP_NUM(EpAddr)][nRFUsbdDir(EpAddr)];
}

static uint8_t nRFUsbdFirstPending(uint16_t Mask)
{
	for (uint8_t EpNum = 0; EpNum < NRFX_USBD_EP_COUNT; EpNum++)
	{
		if ((Mask & (1U << EpNum)) != 0U)
		{
			return EpNum;
		}
	}

	return NRFX_USBD_EP_COUNT;
}

static void nRFUsbdEmit(const UsbdCtrlrEvt_t *pEvt)
{
	if (s_Ctrlr.EvtHandler != NULL)
	{
		s_Ctrlr.EvtHandler(pEvt, s_Ctrlr.pContext);
	}
}

static void nRFUsbdEmitSimple(UsbdCtrlrEvtType_t Type)
{
	UsbdCtrlrEvt_t evt = { .Type = Type };
	nRFUsbdEmit(&evt);
}

static void nRFUsbdEmitXfer(uint8_t EpAddr, uint16_t Length,
						 UsbdCtrlrXferResult_t Result)
{
	UsbdCtrlrEvt_t evt = {
		.Type = USBD_CTRLR_EVT_XFER_CMPL,
		.Xfer = {
			.EpAddr = EpAddr,
			.Length = Length,
			.Result = Result,
		},
	};
	nRFUsbdEmit(&evt);
}

static void nRFUsbdDmaRelease(void)
{
	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000000UL;
	}

	atomic_flag_clear(&s_DmaRunning);
}

static void nRFUsbdDmaStart(volatile uint32_t *pTask)
{
	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000082UL;
	}

	*pTask = 1;
	__ISB();
	__DSB();
}

static void nRFUsbdNoDmaTask(volatile uint32_t *pTask)
{
	*pTask = 1;
	__ISB();
	__DSB();

	atomic_flag_clear(&s_DmaRunning);
}

static bool nRFUsbdStartOutDmaNow(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][0];

	if (!pXfer->Started || pXfer->ActualLen > pXfer->TotalLen)
	{
		return false;
	}

	const uint16_t Remaining =
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);
	const uint16_t Received = (uint16_t)NRF_USBD->SIZE.EPOUT[EpNum];
	const uint16_t TransferLen =
		Received < Remaining ? Received : Remaining;

	NRF_USBD->EPOUT[EpNum].PTR =
		(uint32_t)(uintptr_t)pXfer->pBuffer;
	NRF_USBD->EPOUT[EpNum].MAXCNT = TransferLen;
	nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTEPOUT[EpNum]);

	return true;
}

static bool nRFUsbdStartInDmaNow(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][1];

	if (!pXfer->Started || pXfer->ActualLen > pXfer->TotalLen)
	{
		return false;
	}

	const uint16_t Remaining =
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);
	const uint16_t TransferLen =
		Remaining < pXfer->Mps ? Remaining : pXfer->Mps;

	NRF_USBD->EPIN[EpNum].PTR =
		(uint32_t)(uintptr_t)pXfer->pBuffer;
	NRF_USBD->EPIN[EpNum].MAXCNT = TransferLen;
	nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTEPIN[EpNum]);

	return true;
}

static void nRFUsbdServicePending(void)
{
	for (;;)
	{
		if (atomic_flag_test_and_set(&s_DmaRunning))
		{
			return;
		}

		if (atomic_exchange(&s_PendingEp0Status, false))
		{
			nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0STATUS);
			continue;
		}

		if (atomic_exchange(&s_PendingEp0RcvOut, false))
		{
			nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0RCVOUT);
			continue;
		}

		uint16_t pending = (uint16_t)atomic_load(&s_PendingOut);
		uint8_t epNum = nRFUsbdFirstPending(pending);
		if (epNum < NRFX_USBD_EP_COUNT)
		{
			atomic_fetch_and(&s_PendingOut,
						 (uint_fast16_t)~(1U << epNum));
			if (nRFUsbdStartOutDmaNow(epNum))
			{
				return;
			}

			atomic_flag_clear(&s_DmaRunning);
			continue;
		}

		pending = (uint16_t)atomic_load(&s_PendingIn);
		epNum = nRFUsbdFirstPending(pending);
		if (epNum < NRFX_USBD_EP_COUNT)
		{
			atomic_fetch_and(&s_PendingIn,
						 (uint_fast16_t)~(1U << epNum));
			if (nRFUsbdStartInDmaNow(epNum))
			{
				return;
			}

			atomic_flag_clear(&s_DmaRunning);
			continue;
		}

		atomic_flag_clear(&s_DmaRunning);
		return;
	}
}

static void nRFUsbdQueueOut(uint8_t EpNum)
{
	atomic_fetch_or(&s_PendingOut, (uint_fast16_t)(1U << EpNum));
	nRFUsbdServicePending();
}

static void nRFUsbdQueueIn(uint8_t EpNum)
{
	atomic_fetch_or(&s_PendingIn, (uint_fast16_t)(1U << EpNum));
	nRFUsbdServicePending();
}

static void nRFUsbdQueueEp0Status(void)
{
	atomic_store(&s_PendingEp0Status, true);
	nRFUsbdServicePending();
}

static void nRFUsbdQueueEp0RcvOut(void)
{
	atomic_store(&s_PendingEp0RcvOut, true);
	nRFUsbdServicePending();
}

static void nRFUsbdResetState(void)
{
	memset(s_Ctrlr.Xfer, 0, sizeof(s_Ctrlr.Xfer));
	s_Ctrlr.Xfer[0][0].Mps = NRFX_USBD_MAX_PACKET_SIZE;
	s_Ctrlr.Xfer[0][1].Mps = NRFX_USBD_MAX_PACKET_SIZE;
	s_Ctrlr.SofEnabled = false;
	s_Ctrlr.SetupDirIn = false;

	atomic_store(&s_PendingOut, 0);
	atomic_store(&s_PendingIn, 0);
	atomic_store(&s_PendingEp0Status, false);
	atomic_store(&s_PendingEp0RcvOut, false);
	atomic_flag_clear(&s_DmaRunning);

	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000000UL;
	}
}

bool UsbdCtrlrInit(UsbdCtrlrEvtHandler_t EvtHandler, void *pContext)
{
	s_Ctrlr.EvtHandler = EvtHandler;
	s_Ctrlr.pContext = pContext;
	nRFUsbdResetState();

	return true;
}

void UsbdCtrlrIntEnable(void)
{
	NVIC_EnableIRQ(USBD_IRQn);
}

void UsbdCtrlrIntDisable(void)
{
	NVIC_DisableIRQ(USBD_IRQn);
}

void UsbdCtrlrConnect(void)
{
	NRF_USBD->USBPULLUP = 1;
}

void UsbdCtrlrDisconnect(void)
{
	NRF_USBD->USBPULLUP = 0;
}

void UsbdCtrlrRemoteWakeup(void)
{
	NRF_USBD->LOWPOWER = 0;
}

void UsbdCtrlrSofEnable(bool Enable)
{
	s_Ctrlr.SofEnabled = Enable;

	if (Enable)
	{
		NRF_USBD->EVENTS_SOF = 0;
		NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
	}
	else
	{
		NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
	}
}

void UsbdCtrlrSetAddress(uint8_t Address)
{
	// nRF52840 USBADDR is read-only. SET_ADDRESS is completed by hardware.
	(void)Address;
}

bool UsbdCtrlrEpOpen(const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == NULL)
	{
		return false;
	}

	const uint8_t epAddr = pDesc->bEndpointAddress;
	const uint8_t epNum = USBD_EP_NUM(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;

	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT ||
		(type != USB_ENDPATT_TRANS_BULK &&
		 type != USB_ENDPATT_TRANS_INT) ||
		pDesc->wMaxPacketSize == 0 ||
		pDesc->wMaxPacketSize > NRFX_USBD_MAX_PACKET_SIZE)
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(epAddr);
	pXfer->Mps = pDesc->wMaxPacketSize;
	pXfer->Started = false;
	pXfer->DataReceived = false;

	if (USBD_EP_IS_IN(epAddr))
	{
		NRF_USBD->EVENTS_ENDEPIN[epNum] = 0;
		NRF_USBD->INTENSET =
			(1UL << (USBD_INTEN_ENDEPIN0_Pos + epNum));
		NRF_USBD->EPINEN |= (1UL << epNum);
	}
	else
	{
		NRF_USBD->EVENTS_ENDEPOUT[epNum] = 0;
		NRF_USBD->INTENSET =
			(1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum));
		NRF_USBD->EPOUTEN |= (1UL << epNum);
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}

	NRF_USBD->EPSTALL =
		(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | epAddr;
	NRF_USBD->DTOGGLE =
		(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | epAddr;
	__ISB();
	__DSB();

	return true;
}

void UsbdCtrlrEpClose(uint8_t EpAddr)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);

	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT)
	{
		return;
	}

	const uint_fast16_t bit = (uint_fast16_t)(1U << epNum);
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);

	if (USBD_EP_IS_IN(EpAddr))
	{
		atomic_fetch_and(&s_PendingIn, ~bit);
		NRF_USBD->INTENCLR =
			(1UL << (USBD_INTEN_ENDEPIN0_Pos + epNum));
		NRF_USBD->EPINEN &= ~(1UL << epNum);
		NRF_USBD->EVENTS_ENDEPIN[epNum] = 0;
		NRF_USBD->EPDATASTATUS = (1UL << epNum);
	}
	else
	{
		atomic_fetch_and(&s_PendingOut, ~bit);
		NRF_USBD->INTENCLR =
			(1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum));
		NRF_USBD->EPOUTEN &= ~(1UL << epNum);
		NRF_USBD->EVENTS_ENDEPOUT[epNum] = 0;
		NRF_USBD->EPDATASTATUS = (1UL << (16U + epNum));
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}

	pXfer->Started = false;
	pXfer->DataReceived = false;
	pXfer->ActualLen = 0;
	pXfer->TotalLen = 0;
	pXfer->pBuffer = NULL;
	pXfer->Mps = 0;

	__ISB();
	__DSB();
}

void UsbdCtrlrEpCloseAll(void)
{
	for (uint8_t epNum = 1; epNum < NRFX_USBD_EP_COUNT; epNum++)
	{
		UsbdCtrlrEpClose(epNum);
		UsbdCtrlrEpClose((uint8_t)(epNum | USBD_EP_DIR_IN));
	}

	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;
}

bool UsbdCtrlrEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);

	if (epNum >= NRFX_USBD_EP_COUNT ||
		(TotalBytes > 0 && pBuffer == NULL))
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);
	if (pXfer->Started || pXfer->Mps == 0)
	{
		return false;
	}

	if (USBD_EP_IS_IN(EpAddr) && epNum > 0 &&
		(NRF_USBD->EPDATASTATUS & (1UL << epNum)) != 0)
	{
		NRF_USBD->EPDATASTATUS = (1UL << epNum);
		__ISB();
		__DSB();
	}

	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = TotalBytes;
	pXfer->ActualLen = 0;
	pXfer->Started = true;

	const bool controlStatus =
		epNum == 0 && TotalBytes == 0 &&
		USBD_EP_IS_IN(EpAddr) != s_Ctrlr.SetupDirIn;

	if (controlStatus)
	{
		pXfer->Started = false;
		nRFUsbdEmitXfer(EpAddr, 0, USBD_CTRLR_XFER_SUCCESS);
		nRFUsbdQueueEp0Status();
	}
	else if (USBD_EP_IS_IN(EpAddr))
	{
		nRFUsbdQueueIn(epNum);
	}
	else if (epNum == 0)
	{
		nRFUsbdQueueEp0RcvOut();
	}
	else if (pXfer->DataReceived)
	{
		pXfer->DataReceived = false;
		nRFUsbdQueueOut(epNum);
	}

	return true;
}

bool UsbdCtrlrEpBusy(uint8_t EpAddr)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);
	return epNum < NRFX_USBD_EP_COUNT && nRFUsbdGetXfer(EpAddr)->Started;
}

void UsbdCtrlrEpStall(uint8_t EpAddr)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);

	if (epNum >= NRFX_USBD_EP_COUNT)
	{
		return;
	}

	if (epNum == 0)
	{
		NRF_USBD->TASKS_EP0STALL = 1;
	}
	else
	{
		NRF_USBD->EPSTALL =
			(USBD_EPSTALL_STALL_Stall << USBD_EPSTALL_STALL_Pos) | EpAddr;
	}

	__ISB();
	__DSB();
}

void UsbdCtrlrEpClearStall(uint8_t EpAddr)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);

	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT)
	{
		return;
	}

	NRF_USBD->DTOGGLE = EpAddr;
	NRF_USBD->DTOGGLE =
		(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | EpAddr;
	NRF_USBD->EPSTALL =
		(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | EpAddr;

	if (!USBD_EP_IS_IN(EpAddr))
	{
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}

	__ISB();
	__DSB();
}

static uint32_t nRFUsbdCollectEvents(void)
{
	const uint32_t inten = NRF_USBD->INTEN;
	uint32_t intStatus = 0;
	volatile uint32_t *pEvent = &NRF_USBD->EVENTS_USBRESET;

	for (uint8_t index = 0; index < NRFX_USBD_IRQ_EVENT_COUNT; index++)
	{
		const uint32_t mask = (1UL << index);
		if ((inten & mask) == 0 || pEvent[index] == 0)
		{
			continue;
		}

		intStatus |= mask;
		pEvent[index] = 0;
		__ISB();
		__DSB();
	}

	return intStatus;
}

static void nRFUsbdBusReset(void)
{
	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;

	for (uint8_t epNum = 0; epNum < NRFX_USBD_EP_COUNT; epNum++)
	{
		NRF_USBD->TASKS_STARTEPIN[epNum] = 0;
		NRF_USBD->TASKS_STARTEPOUT[epNum] = 0;
	}

	const uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
	NRF_USBD->EPDATASTATUS = dataStatus;

	NRF_USBD->EVENTS_USBEVENT = 0;
	const uint32_t cause = NRF_USBD->EVENTCAUSE;
	NRF_USBD->EVENTCAUSE = cause;

	NRF_USBD->INTENCLR = NRF_USBD->INTEN;
	NRF_USBD->INTENSET =
		USBD_INTEN_USBRESET_Msk |
		USBD_INTEN_USBEVENT_Msk |
		USBD_INTEN_EPDATA_Msk |
		USBD_INTEN_EP0SETUP_Msk |
		USBD_INTEN_EP0DATADONE_Msk |
		USBD_INTEN_ENDEPIN0_Msk |
		USBD_INTEN_ENDEPOUT0_Msk;

	nRFUsbdResetState();
}

static void nRFUsbdSetupEvent(void)
{
	UsbdCtrlrEvt_t evt = { .Type = USBD_CTRLR_EVT_SETUP };
	evt.Setup.bmRequestType = (uint8_t)NRF_USBD->BMREQUESTTYPE;
	evt.Setup.bRequest = (uint8_t)NRF_USBD->BREQUEST;
	evt.Setup.wValue = (uint16_t)NRF_USBD->WVALUEL |
		((uint16_t)NRF_USBD->WVALUEH << 8);
	evt.Setup.wIndex = (uint16_t)NRF_USBD->WINDEXL |
		((uint16_t)NRF_USBD->WINDEXH << 8);
	evt.Setup.wLength = (uint16_t)NRF_USBD->WLENGTHL |
		((uint16_t)NRF_USBD->WLENGTHH << 8);

	s_Ctrlr.SetupDirIn =
		(evt.Setup.bmRequestType & USB_REQTYPE_MASK_DIR) != 0;

	const bool setAddress =
		(evt.Setup.bmRequestType &
		 (USB_REQTYPE_MASK_RECEIPT | USB_REQTYPE_MASK_TYPE)) == 0 &&
		evt.Setup.bRequest == USB_REQ_SET_ADDRESS;

	if (setAddress)
	{
		UsbdCtrlrEvt_t addrEvt = {
			.Type = USBD_CTRLR_EVT_ADDRESS,
			.Address = (uint8_t)(evt.Setup.wValue & 0x7FU),
		};
		nRFUsbdEmit(&addrEvt);
		return;
	}

	nRFUsbdEmit(&evt);
}

static void nRFUsbdIrqHandler(void)
{
	const uint32_t intStatus = nRFUsbdCollectEvents();
	if (intStatus == 0)
	{
		return;
	}

	uint32_t eventCause = 0;
	if ((intStatus & USBD_INTEN_USBEVENT_Msk) != 0)
	{
		eventCause = NRF_USBD->EVENTCAUSE;
		NRF_USBD->EVENTCAUSE = eventCause;
		__ISB();
		__DSB();
	}

	if ((intStatus & USBD_INTEN_USBRESET_Msk) != 0)
	{
		nRFUsbdBusReset();
		nRFUsbdEmitSimple(USBD_CTRLR_EVT_RESET);
	}

	if ((intStatus & USBD_INTEN_USBEVENT_Msk) != 0)
	{
		if ((eventCause & USBD_EVENTCAUSE_SUSPEND_Msk) != 0)
		{
			NRF_USBD->LOWPOWER = 1;
			nRFUsbdEmitSimple(USBD_CTRLR_EVT_SUSPEND);
		}

		if ((eventCause & USBD_EVENTCAUSE_USBWUALLOWED_Msk) != 0)
		{
			NRF_USBD->DPDMVALUE = USBD_DPDMVALUE_STATE_Resume;
			NRF_USBD->TASKS_DPDMDRIVE = 1;
			if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0)
			{
				NRF_USBD->EVENTS_SOF = 0;
			}
			NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
		}

		if ((eventCause & USBD_EVENTCAUSE_RESUME_Msk) != 0)
		{
			nRFUsbdEmitSimple(USBD_CTRLR_EVT_RESUME);
		}
	}

	if ((intStatus & USBD_INTEN_EP0SETUP_Msk) != 0)
	{
		nRFUsbdSetupEvent();
	}

	if ((intStatus & NRFX_USBD_EDPT_END_ALL_MASK) != 0)
	{
		nRFUsbdDmaRelease();
	}

	for (uint8_t epNum = 0; epNum < NRFX_USBD_EP_COUNT; epNum++)
	{
		if ((intStatus &
			 (1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum))) == 0)
		{
			continue;
		}

		nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum][0];
		if (!pXfer->Started)
		{
			continue;
		}

		const uint16_t transferLen =
			(uint16_t)NRF_USBD->EPOUT[epNum].AMOUNT;

		if (pXfer->pBuffer != NULL)
		{
			pXfer->pBuffer += transferLen;
		}
		pXfer->ActualLen += transferLen;

		if (transferLen == pXfer->Mps &&
			pXfer->ActualLen < pXfer->TotalLen)
		{
			if (epNum == 0)
			{
				nRFUsbdQueueEp0RcvOut();
			}
		}
		else
		{
			pXfer->Started = false;
			nRFUsbdEmitXfer(epNum, pXfer->ActualLen,
							 USBD_CTRLR_XFER_SUCCESS);
		}
	}

	if ((intStatus &
		 (USBD_INTEN_EPDATA_Msk | USBD_INTEN_EP0DATADONE_Msk)) != 0)
	{
		const uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
		NRF_USBD->EPDATASTATUS = dataStatus;
		__ISB();
		__DSB();

		const bool controlIn =
			(intStatus & USBD_INTEN_EP0DATADONE_Msk) != 0 &&
			s_Ctrlr.SetupDirIn;
		const bool controlOut =
			(intStatus & USBD_INTEN_EP0DATADONE_Msk) != 0 &&
			!s_Ctrlr.SetupDirIn;

		// OUT first so host-to-device traffic is rearmed promptly.
		for (uint8_t epNum = 0; epNum < NRFX_USBD_EP_COUNT; epNum++)
		{
			if ((dataStatus & (1UL << (16U + epNum))) == 0 &&
				!(epNum == 0 && controlOut))
			{
				continue;
			}

			nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum][0];
			if (pXfer->Started &&
				(pXfer->ActualLen < pXfer->TotalLen ||
				 pXfer->TotalLen == 0))
			{
				pXfer->DataReceived = false;
				nRFUsbdQueueOut(epNum);
			}
			else
			{
				pXfer->DataReceived = true;
			}
		}

		for (uint8_t epNum = 0; epNum < NRFX_USBD_EP_COUNT; epNum++)
		{
			if ((dataStatus & (1UL << epNum)) == 0 &&
				!(epNum == 0 && controlIn))
			{
				continue;
			}

			nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum][1];
			if (!pXfer->Started)
			{
				continue;
			}

			const uint16_t transferLen =
				(uint16_t)NRF_USBD->EPIN[epNum].AMOUNT;

			if (pXfer->pBuffer != NULL)
			{
				pXfer->pBuffer += transferLen;
			}
			pXfer->ActualLen += transferLen;

			if (pXfer->ActualLen < pXfer->TotalLen)
			{
				nRFUsbdQueueIn(epNum);
			}
			else
			{
				pXfer->Started = false;
				nRFUsbdEmitXfer(
					(uint8_t)(epNum | USBD_EP_DIR_IN),
					pXfer->ActualLen,
					USBD_CTRLR_XFER_SUCCESS);
			}
		}
	}

	if ((intStatus & USBD_INTEN_SOF_Msk) != 0)
	{
		UsbdCtrlrEvt_t evt = {
			.Type = USBD_CTRLR_EVT_SOF,
			.FrameNo = (uint16_t)NRF_USBD->FRAMECNTR,
		};
		nRFUsbdEmit(&evt);

		if (!s_Ctrlr.SofEnabled)
		{
			NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
		}
	}

	nRFUsbdServicePending();
}

void USBD_IRQHandler(void)
{
	nRFUsbdIrqHandler();
}
