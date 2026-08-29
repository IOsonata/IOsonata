/**-------------------------------------------------------------------------
@file	tu_dcd_nrfx.c

@brief	Device stack dcd implementation for the Nordic USBD peripheral.

Implements the device stack's dcd interface for the nRF52840 USBD peripheral,
replacing the one that ships with the device stack. It is compiled in place of
external/tinyusb/src/portable/nordic/nrf5x/dcd_nrf5x.c, which must be excluded
from the project when this file is present.

nRF52 only, and the guard below makes that so. The nRF54LM20 has USBHS, a
DesignWare core, and its dcd is the device stack's own dcd_dwc2.c with
dwc2_common.c, 1550 lines shared with every other vendor that uses the same
core. Nothing in it is broken for us, so there is nothing here to replace.
What is Nordic specific on that part is the wrapper, the PHY and the
regulator, and those are in usbd_nrfx.cpp.

It exists because the stock driver puts the whole start sequence inside
tusb_hal_nrf_power_event, which expects the application to own the POWER
interrupt. On this family that vector is taken by MPSL whenever a radio stack
is in the build. Here the power, clock and cable work sits in usbd_nrfx.cpp
and this file owns only the controller : transfer descriptors, EasyDMA
arbitration, endpoint state and the interrupt.

Errata 199 is handled around every DMA start. The remaining errata belong to
the enable sequence and are in usbd_nrfx.cpp.

@author	Hoang Nguyen Hoan
@date	Aug. 28, 2026

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

#include "tusb_option.h"
#include "device/dcd.h"
#include "device/usbd_pvt.h"

#ifndef USBD_PRESENT
#error "tu_dcd_nrfx: this part has no USBD peripheral"
#endif

enum
{
	NRFX_DCD_MAX_PACKET_SIZE = 64,
	NRFX_DCD_EP_ISO = 8,
	NRFX_DCD_EP_CBI_COUNT = 8
};

#define NRFX_DCD_EDPT_END_ALL_MASK \
	((0xFFUL << USBD_INTEN_ENDEPIN0_Pos) | \
	 (0xFFUL << USBD_INTEN_ENDEPOUT0_Pos) | \
	 USBD_INTEN_ENDISOIN_Msk | USBD_INTEN_ENDISOOUT_Msk)

#define NRFX_DCD_IRQ_EVENT_COUNT (USBD_INTEN_EPDATA_Pos + 1)

#define NRFX_DCD_ERRATA_199_REG \
	(*((volatile uint32_t *)0x40027C1CUL))

typedef struct
{
	uint8_t *pBuffer;
	uint16_t TotalLen;
	volatile uint16_t ActualLen;
	uint16_t Mps;
	volatile bool DataReceived;
	volatile bool Started;
	bool IsoInTransferReady;
} nRFUsbdXfer_t;

typedef struct
{
	nRFUsbdXfer_t Xfer[NRFX_DCD_EP_CBI_COUNT + 1][2];
	bool SofEnabled;
} nRFUsbdDcdState_t;

static nRFUsbdDcdState_t s_Dcd;
static atomic_flag s_DmaRunning = ATOMIC_FLAG_INIT;

static inline nRFUsbdXfer_t *nRFUsbdGetXfer(uint8_t EpNum, uint8_t Dir)
{
	return &s_Dcd.Xfer[EpNum][Dir];
}

static bool nRFUsbdInIsr(void)
{
	return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

static void nRFUsbdDmaStart(volatile uint32_t *pTask)
{
	const bool NoDma =
		pTask == &NRF_USBD->TASKS_EP0STATUS ||
		pTask == &NRF_USBD->TASKS_EP0RCVOUT;

	if (!NoDma && nrf52_errata_199())
	{
		NRFX_DCD_ERRATA_199_REG = 0x00000082UL;
	}

	*pTask = 1;
	__ISB();
	__DSB();

	if (NoDma)
	{
		atomic_flag_clear(&s_DmaRunning);
	}
}

static void nRFUsbdDmaEnd(void)
{
	if (nrf52_errata_199())
	{
		NRFX_DCD_ERRATA_199_REG = 0x00000000UL;
	}
	atomic_flag_clear(&s_DmaRunning);
}

static void nRFUsbdEdptDmaStart(volatile uint32_t *pTask);

static void nRFUsbdEdptDmaStartDeferred(void *pContext)
{
	nRFUsbdEdptDmaStart(
		(volatile uint32_t *)(uintptr_t)pContext);
}

static void nRFUsbdEdptDmaStart(volatile uint32_t *pTask)
{
	if (atomic_flag_test_and_set(&s_DmaRunning))
	{
		usbd_defer_func(nRFUsbdEdptDmaStartDeferred,
						(void *)(uintptr_t)pTask,
						nRFUsbdInIsr());
		return;
	}

	nRFUsbdDmaStart(pTask);
}

static void nRFUsbdStartOutDma(uint8_t EpNum);

static void nRFUsbdStartOutDmaDeferred(void *pContext)
{
	nRFUsbdStartOutDma((uint8_t)(uintptr_t)pContext);
}

static void nRFUsbdStartOutDma(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpNum, TUSB_DIR_OUT);

	if (atomic_flag_test_and_set(&s_DmaRunning))
	{
		usbd_defer_func(nRFUsbdStartOutDmaDeferred,
						(void *)(uintptr_t)EpNum,
						nRFUsbdInIsr());
		return;
	}

	if (EpNum == NRFX_DCD_EP_ISO)
	{
		const uint32_t Length = NRF_USBD->SIZE.ISOOUT;
		if ((Length & USBD_SIZE_ISOOUT_ZERO_Msk) != 0 ||
			!pXfer->Started)
		{
			atomic_flag_clear(&s_DmaRunning);
			return;
		}

		NRF_USBD->ISOOUT.PTR = (uint32_t)(uintptr_t)pXfer->pBuffer;
		NRF_USBD->ISOOUT.MAXCNT = Length;
		nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTISOOUT);
		return;
	}

	if (!pXfer->Started || pXfer->ActualLen >= pXfer->TotalLen)
	{
		pXfer->DataReceived = true;
		atomic_flag_clear(&s_DmaRunning);
		return;
	}

	const uint16_t Remaining = pXfer->TotalLen - pXfer->ActualLen;
	const uint16_t Received = (uint16_t)NRF_USBD->SIZE.EPOUT[EpNum];
	const uint16_t TransferLen =
		Received < Remaining ? Received : Remaining;

	NRF_USBD->EPOUT[EpNum].PTR =
		(uint32_t)(uintptr_t)pXfer->pBuffer;
	NRF_USBD->EPOUT[EpNum].MAXCNT = TransferLen;
	nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTEPOUT[EpNum]);
}

static void nRFUsbdDrainReceivedOut(uint8_t EpNum);

static void nRFUsbdDrainReceivedOutDeferred(void *pContext)
{
	nRFUsbdDrainReceivedOut((uint8_t)(uintptr_t)pContext);
}

static void nRFUsbdDrainReceivedOut(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpNum, TUSB_DIR_OUT);

	if (atomic_flag_test_and_set(&s_DmaRunning))
	{
		usbd_defer_func(nRFUsbdDrainReceivedOutDeferred,
						(void *)(uintptr_t)EpNum,
						nRFUsbdInIsr());
		return;
	}

	const uint16_t Remaining =
		pXfer->TotalLen >= pXfer->ActualLen ?
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen) : 0;
	const uint16_t Received = (uint16_t)NRF_USBD->SIZE.EPOUT[EpNum];
	const uint16_t TransferLen =
		Received < Remaining ? Received : Remaining;

	NRF_USBD->EPOUT[EpNum].PTR =
		(uint32_t)(uintptr_t)pXfer->pBuffer;
	NRF_USBD->EPOUT[EpNum].MAXCNT = TransferLen;
	nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTEPOUT[EpNum]);
}

static void nRFUsbdStartInDma(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpNum, TUSB_DIR_IN);
	const uint16_t Remaining = pXfer->TotalLen - pXfer->ActualLen;
	const uint16_t TransferLen =
		Remaining < pXfer->Mps ? Remaining : pXfer->Mps;

	NRF_USBD->EPIN[EpNum].PTR =
		(uint32_t)(uintptr_t)pXfer->pBuffer;
	NRF_USBD->EPIN[EpNum].MAXCNT = TransferLen;
	nRFUsbdEdptDmaStart(&NRF_USBD->TASKS_STARTEPIN[EpNum]);
}

bool dcd_init(uint8_t RhPort, const tusb_rhport_init_t *pRhInit)
{
	(void)RhPort;
	(void)pRhInit;

	memset(&s_Dcd, 0, sizeof(s_Dcd));
	atomic_flag_clear(&s_DmaRunning);
	s_Dcd.Xfer[0][TUSB_DIR_IN].Mps = NRFX_DCD_MAX_PACKET_SIZE;
	s_Dcd.Xfer[0][TUSB_DIR_OUT].Mps = NRFX_DCD_MAX_PACKET_SIZE;

	return true;
}

void dcd_int_enable(uint8_t RhPort)
{
	(void)RhPort;
	NVIC_EnableIRQ(USBD_IRQn);
}

void dcd_int_disable(uint8_t RhPort)
{
	(void)RhPort;
	NVIC_DisableIRQ(USBD_IRQn);
}

void dcd_set_address(uint8_t RhPort, uint8_t DevAddr)
{
	(void)RhPort;
	(void)DevAddr;

	const uint32_t Cause = NRF_USBD->EVENTCAUSE;
	NRF_USBD->EVENTCAUSE = Cause;
	NRF_USBD->EVENTS_USBEVENT = 0;
	NRF_USBD->INTENSET = USBD_INTEN_USBEVENT_Msk;
}

void dcd_remote_wakeup(uint8_t RhPort)
{
	(void)RhPort;
	NRF_USBD->LOWPOWER = 0;
}

void dcd_disconnect(uint8_t RhPort)
{
	(void)RhPort;
	NRF_USBD->USBPULLUP = 0;
	dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, false);
}

void dcd_connect(uint8_t RhPort)
{
	(void)RhPort;
	NRF_USBD->USBPULLUP = 1;
}

void dcd_sof_enable(uint8_t RhPort, bool Enable)
{
	(void)RhPort;

	s_Dcd.SofEnabled = Enable;
	if (Enable)
	{
		NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
	}
	else
	{
		NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
	}
}

bool dcd_edpt_open(uint8_t RhPort,
				   const tusb_desc_endpoint_t *pDesc)
{
	(void)RhPort;

	const uint8_t EpAddr = pDesc->bEndpointAddress;
	const uint8_t EpNum = tu_edpt_number(EpAddr);
	const uint8_t Dir = tu_edpt_dir(EpAddr);

	if (EpNum > NRFX_DCD_EP_ISO)
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpNum, Dir);
	pXfer->Mps = tu_edpt_packet_size(pDesc);

	if (pDesc->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS)
	{
		if (EpNum >= NRFX_DCD_EP_CBI_COUNT)
		{
			return false;
		}

		if (Dir == TUSB_DIR_OUT)
		{
			NRF_USBD->INTENSET =
				(1UL << (USBD_INTEN_ENDEPOUT0_Pos + EpNum));
			NRF_USBD->EPOUTEN |= (1UL << (EpNum));
			NRF_USBD->SIZE.EPOUT[EpNum] = 0;
		}
		else
		{
			NRF_USBD->INTENSET =
				(1UL << (USBD_INTEN_ENDEPIN0_Pos + EpNum));
			NRF_USBD->EPINEN |= (1UL << (EpNum));
		}

		NRF_USBD->EPSTALL =
			(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) |
			EpAddr;
		NRF_USBD->DTOGGLE =
			(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) |
			EpAddr;
	}
	else
	{
		if (EpNum != NRFX_DCD_EP_ISO)
		{
			return false;
		}

		if (Dir == TUSB_DIR_OUT)
		{
			if (s_Dcd.Xfer[NRFX_DCD_EP_ISO][TUSB_DIR_IN].Mps != 0)
			{
				NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;
			}

			NRF_USBD->EVENTS_ENDISOOUT = 0;
			if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0)
			{
				NRF_USBD->EVENTS_SOF = 0;
			}
			NRF_USBD->INTENSET =
				USBD_INTENSET_ENDISOOUT_Msk | USBD_INTENSET_SOF_Msk;
			NRF_USBD->EPOUTEN |= USBD_EPOUTEN_ISOOUT_Msk;
		}
		else
		{
			NRF_USBD->EVENTS_ENDISOIN = 0;
			if (s_Dcd.Xfer[NRFX_DCD_EP_ISO][TUSB_DIR_OUT].Mps != 0)
			{
				NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;
			}

			if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0)
			{
				NRF_USBD->EVENTS_SOF = 0;
			}
			NRF_USBD->INTENSET =
				USBD_INTENSET_ENDISOIN_Msk | USBD_INTENSET_SOF_Msk;
			NRF_USBD->EPINEN |= USBD_EPINEN_ISOIN_Msk;
		}
	}

	__ISB();
	__DSB();
	return true;
}

void dcd_edpt_close_all(uint8_t RhPort)
{
	dcd_int_disable(RhPort);

	for (uint8_t EpNum = 1;
		 EpNum < NRFX_DCD_EP_CBI_COUNT;
		 EpNum++)
	{
		NRF_USBD->INTENCLR =
			(1UL << (USBD_INTEN_ENDEPOUT0_Pos + EpNum)) |
			(1UL << (USBD_INTEN_ENDEPIN0_Pos + EpNum));
		NRF_USBD->TASKS_STARTEPIN[EpNum] = 0;
		NRF_USBD->TASKS_STARTEPOUT[EpNum] = 0;
		memset(s_Dcd.Xfer[EpNum], 0, 2 * sizeof(nRFUsbdXfer_t));
	}

	NRF_USBD->INTENCLR =
		USBD_INTENCLR_SOF_Msk |
		USBD_INTENCLR_ENDISOOUT_Msk |
		USBD_INTENCLR_ENDISOIN_Msk;
	NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_OneDir;
	NRF_USBD->TASKS_STARTISOIN = 0;
	NRF_USBD->TASKS_STARTISOOUT = 0;
	memset(s_Dcd.Xfer[NRFX_DCD_EP_ISO], 0, 2 * sizeof(nRFUsbdXfer_t));

	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;

	dcd_int_enable(RhPort);
}

bool dcd_edpt_iso_alloc(uint8_t RhPort,
						uint8_t EpAddr,
						uint16_t LargestPacketSize)
{
	(void)RhPort;
	(void)LargestPacketSize;
	if (tu_edpt_number(EpAddr) != NRFX_DCD_EP_ISO)
	{
		return false;
	}
	return true;
}

bool dcd_edpt_iso_activate(uint8_t RhPort,
						   const tusb_desc_endpoint_t *pDesc)
{
	(void)RhPort;

	const uint8_t EpAddr = pDesc->bEndpointAddress;
	const uint8_t EpNum = tu_edpt_number(EpAddr);
	const uint8_t Dir = tu_edpt_dir(EpAddr);

	if (EpNum != NRFX_DCD_EP_ISO)
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpNum, Dir);
	pXfer->Started = false;
	pXfer->DataReceived = false;
	pXfer->IsoInTransferReady = false;
	pXfer->Mps = tu_edpt_packet_size(pDesc);

	if (Dir == TUSB_DIR_OUT)
	{
		if (s_Dcd.Xfer[NRFX_DCD_EP_ISO][TUSB_DIR_IN].Mps != 0)
		{
			NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;
		}
		NRF_USBD->EVENTS_ENDISOOUT = 0;
		if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0)
		{
			NRF_USBD->EVENTS_SOF = 0;
		}
		NRF_USBD->INTENSET =
			USBD_INTENSET_ENDISOOUT_Msk | USBD_INTENSET_SOF_Msk;
		NRF_USBD->EPOUTEN |= USBD_EPOUTEN_ISOOUT_Msk;
	}
	else
	{
		NRF_USBD->EVENTS_ENDISOIN = 0;
		if (s_Dcd.Xfer[NRFX_DCD_EP_ISO][TUSB_DIR_OUT].Mps != 0)
		{
			NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;
		}
		if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0)
		{
			NRF_USBD->EVENTS_SOF = 0;
		}
		NRF_USBD->INTENSET =
			USBD_INTENSET_ENDISOIN_Msk | USBD_INTENSET_SOF_Msk;
		NRF_USBD->EPINEN |= USBD_EPINEN_ISOIN_Msk;
	}

	__ISB();
	__DSB();
	return true;
}

bool dcd_edpt_xfer(uint8_t RhPort,
				   uint8_t EpAddr,
				   uint8_t *pBuffer,
				   uint16_t TotalBytes,
				   bool IsIsr)
{
	(void)RhPort;
	(void)IsIsr;

	const uint8_t EpNum = tu_edpt_number(EpAddr);
	const uint8_t Dir = tu_edpt_dir(EpAddr);

	if (EpNum > NRFX_DCD_EP_ISO)
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpNum, Dir);
	if (pXfer->Started)
	{
		return false;
	}

	/*
	 * A status bit left over from the previous transfer on this endpoint
	 * makes the interrupt handler credit the new transfer with a completion
	 * it never had. Clear it before the transfer starts. Applies to every IN
	 * endpoint, not only to the one this driver was first written for.
	 */
	if (Dir == TUSB_DIR_IN && EpNum > 0 &&
		(NRF_USBD->EPDATASTATUS & (1UL << (EpNum))) != 0)
	{
		NRF_USBD->EPDATASTATUS = (1UL << (EpNum));
		__ISB();
		__DSB();
	}

	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = TotalBytes;
	pXfer->ActualLen = 0;

	const bool ControlStatus =
		EpNum == 0 &&
		TotalBytes == 0 &&
		Dir != tu_edpt_dir((uint8_t)NRF_USBD->BMREQUESTTYPE);

	if (ControlStatus)
	{
		dcd_event_xfer_complete(0,
								EpAddr,
								0,
								XFER_RESULT_SUCCESS,
								nRFUsbdInIsr());
		nRFUsbdEdptDmaStart(&NRF_USBD->TASKS_EP0STATUS);
	}
	else if (Dir == TUSB_DIR_OUT)
	{
		pXfer->Started = true;

		if (EpNum == 0)
		{
			nRFUsbdEdptDmaStart(&NRF_USBD->TASKS_EP0RCVOUT);
		}
		else
		{
			__ISB();
			__DSB();

			if (pXfer->DataReceived && pXfer->Started)
			{
				pXfer->DataReceived = false;
				nRFUsbdStartOutDma(EpNum);
			}
		}
	}
	else
	{
		nRFUsbdStartInDma(EpNum);
	}

	return true;
}

void dcd_edpt_stall(uint8_t RhPort, uint8_t EpAddr)
{
	(void)RhPort;

	const uint8_t EpNum = tu_edpt_number(EpAddr);
	const uint8_t Dir = tu_edpt_dir(EpAddr);
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpNum, Dir);

	if (EpNum == 0)
	{
		NRF_USBD->TASKS_EP0STALL = 1;
	}
	else if (EpNum != NRFX_DCD_EP_ISO)
	{
		NRF_USBD->EPSTALL =
			(USBD_EPSTALL_STALL_Stall << USBD_EPSTALL_STALL_Pos) |
			EpAddr;

		if (Dir == TUSB_DIR_OUT && pXfer->DataReceived)
		{
			pXfer->DataReceived = false;
			nRFUsbdDrainReceivedOut(EpNum);
		}
	}

	__ISB();
	__DSB();
}

void dcd_edpt_clear_stall(uint8_t RhPort, uint8_t EpAddr)
{
	(void)RhPort;

	const uint8_t EpNum = tu_edpt_number(EpAddr);
	const uint8_t Dir = tu_edpt_dir(EpAddr);

	if (EpNum != 0 && EpNum != NRFX_DCD_EP_ISO)
	{
		NRF_USBD->DTOGGLE = EpAddr;
		NRF_USBD->DTOGGLE =
			(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) |
			EpAddr;
		NRF_USBD->EPSTALL =
			(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) |
			EpAddr;

		if (Dir == TUSB_DIR_OUT)
		{
			NRF_USBD->SIZE.EPOUT[EpNum] = 0;
		}

		__ISB();
		__DSB();
	}
}

static uint32_t nRFUsbdCollectEvents(void)
{
	const uint32_t Inten = NRF_USBD->INTEN;
	uint32_t IntStatus = 0;
	volatile uint32_t *pEvent = &NRF_USBD->EVENTS_USBRESET;

	for (uint8_t Index = 0; Index < NRFX_DCD_IRQ_EVENT_COUNT; Index++)
	{
		const uint32_t Mask = (1UL << (Index));
		if ((Inten & Mask) == 0 || pEvent[Index] == 0)
		{
			continue;
		}

		IntStatus |= Mask;
		pEvent[Index] = 0;
		__ISB();
		__DSB();
	}

	return IntStatus;
}

static void nRFUsbdBusReset(void)
{
	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;

	for (uint8_t EpNum = 0;
		 EpNum < NRFX_DCD_EP_CBI_COUNT;
		 EpNum++)
	{
		NRF_USBD->TASKS_STARTEPIN[EpNum] = 0;
		NRF_USBD->TASKS_STARTEPOUT[EpNum] = 0;
	}

	NRF_USBD->TASKS_STARTISOIN = 0;
	NRF_USBD->TASKS_STARTISOOUT = 0;

	NRF_USBD->EVENTS_USBEVENT = 0;
	const uint32_t Cause = NRF_USBD->EVENTCAUSE;
	NRF_USBD->EVENTCAUSE = Cause;

	NRF_USBD->INTENCLR = NRF_USBD->INTEN;
	NRF_USBD->INTENSET =
		USBD_INTEN_USBRESET_Msk |
		USBD_INTEN_USBEVENT_Msk |
		USBD_INTEN_EPDATA_Msk |
		USBD_INTEN_EP0SETUP_Msk |
		USBD_INTEN_EP0DATADONE_Msk |
		USBD_INTEN_ENDEPIN0_Msk |
		USBD_INTEN_ENDEPOUT0_Msk;

	memset(&s_Dcd, 0, sizeof(s_Dcd));
	atomic_flag_clear(&s_DmaRunning);
	s_Dcd.Xfer[0][TUSB_DIR_IN].Mps = NRFX_DCD_MAX_PACKET_SIZE;
	s_Dcd.Xfer[0][TUSB_DIR_OUT].Mps = NRFX_DCD_MAX_PACKET_SIZE;
}

void dcd_int_handler(uint8_t RhPort)
{
	(void)RhPort;

	const uint32_t IntStatus = nRFUsbdCollectEvents();
	if (IntStatus == 0)
	{
		return;
	}

	uint32_t EventCause = 0;
	if ((IntStatus & USBD_INTEN_USBEVENT_Msk) != 0)
	{
		EventCause = NRF_USBD->EVENTCAUSE;
		NRF_USBD->EVENTCAUSE = EventCause;
		__ISB();
		__DSB();

	}

	if ((IntStatus & USBD_INTEN_USBRESET_Msk) != 0)
	{
		nRFUsbdBusReset();
		dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
	}

	if ((IntStatus & USBD_INTEN_ENDISOIN_Msk) != 0)
	{
		nRFUsbdXfer_t *pXfer =
			nRFUsbdGetXfer(NRFX_DCD_EP_ISO, TUSB_DIR_IN);
		pXfer->ActualLen = NRF_USBD->ISOIN.AMOUNT;
		pXfer->IsoInTransferReady = true;
	}

	if ((IntStatus & USBD_INTEN_SOF_Msk) != 0)
	{
		bool IsoEnabled = false;

		if ((NRF_USBD->EPOUTEN & USBD_EPOUTEN_ISOOUT_Msk) != 0)
		{
			IsoEnabled = true;
			if ((IntStatus & USBD_INTEN_USBEVENT_Msk) == 0 ||
				(EventCause & USBD_EVENTCAUSE_ISOOUTCRC_Msk) == 0)
			{
				nRFUsbdStartOutDma(NRFX_DCD_EP_ISO);
			}
		}

		if ((NRF_USBD->EPINEN & USBD_EPINEN_ISOIN_Msk) != 0)
		{
			IsoEnabled = true;
			nRFUsbdXfer_t *pXfer =
				nRFUsbdGetXfer(NRFX_DCD_EP_ISO, TUSB_DIR_IN);
			if (pXfer->IsoInTransferReady)
			{
				pXfer->IsoInTransferReady = false;
				dcd_event_xfer_complete(
					0,
					NRFX_DCD_EP_ISO | TUSB_DIR_IN_MASK,
					pXfer->ActualLen,
					XFER_RESULT_SUCCESS,
					true);
			}
		}

		if (!IsoEnabled && !s_Dcd.SofEnabled)
		{
			NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
		}

		dcd_event_sof(0, NRF_USBD->FRAMECNTR, true);
	}

	if ((IntStatus & USBD_INTEN_USBEVENT_Msk) != 0)
	{

		if ((EventCause & USBD_EVENTCAUSE_SUSPEND_Msk) != 0)
		{
			NRF_USBD->LOWPOWER = 1;
			dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
		}

		if ((EventCause & USBD_EVENTCAUSE_USBWUALLOWED_Msk) != 0)
		{
			NRF_USBD->DPDMVALUE = USBD_DPDMVALUE_STATE_Resume;
			NRF_USBD->TASKS_DPDMDRIVE = 1;
			if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0)
			{
				NRF_USBD->EVENTS_SOF = 0;
			}
			NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
		}

		if ((EventCause & USBD_EVENTCAUSE_RESUME_Msk) != 0)
		{
			dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
		}
	}

	if ((IntStatus & USBD_INTEN_EP0SETUP_Msk) != 0)
	{
		const uint8_t Setup[8] =
		{
			NRF_USBD->BMREQUESTTYPE,
			NRF_USBD->BREQUEST,
			NRF_USBD->WVALUEL,
			NRF_USBD->WVALUEH,
			NRF_USBD->WINDEXL,
			NRF_USBD->WINDEXH,
			NRF_USBD->WLENGTHL,
			NRF_USBD->WLENGTHH
		};

		const tusb_control_request_t *pRequest =
			(const tusb_control_request_t *)Setup;

		if (!(TUSB_REQ_RCPT_DEVICE ==
				  pRequest->bmRequestType_bit.recipient &&
			  TUSB_REQ_TYPE_STANDARD ==
				  pRequest->bmRequestType_bit.type &&
			  TUSB_REQ_SET_ADDRESS == pRequest->bRequest))
		{
			dcd_event_setup_received(0, Setup, true);
		}
	}

	if ((IntStatus & NRFX_DCD_EDPT_END_ALL_MASK) != 0)
	{
		nRFUsbdDmaEnd();
	}

	for (uint8_t EpNum = 0;
		 EpNum < NRFX_DCD_EP_CBI_COUNT + 1;
		 EpNum++)
	{
		if (!tu_bit_test(IntStatus,
						 USBD_INTEN_ENDEPOUT0_Pos + EpNum))
		{
			continue;
		}

		nRFUsbdXfer_t *pXfer =
			nRFUsbdGetXfer(EpNum, TUSB_DIR_OUT);
		if (!pXfer->Started)
		{
			continue;
		}

		const uint16_t TransferLen =
			(uint16_t)NRF_USBD->EPOUT[EpNum].AMOUNT;

		pXfer->pBuffer += TransferLen;
		pXfer->ActualLen += TransferLen;

		if (EpNum != NRFX_DCD_EP_ISO &&
			TransferLen == pXfer->Mps &&
			pXfer->ActualLen < pXfer->TotalLen)
		{
			if (EpNum == 0)
			{
				nRFUsbdEdptDmaStart(&NRF_USBD->TASKS_EP0RCVOUT);
			}
		}
		else
		{
			pXfer->TotalLen = pXfer->ActualLen;
			pXfer->Started = false;
			dcd_event_xfer_complete(0,
									EpNum,
									pXfer->ActualLen,
									XFER_RESULT_SUCCESS,
									true);
		}
	}

	if ((IntStatus &
		 (USBD_INTEN_EPDATA_Msk | USBD_INTEN_EP0DATADONE_Msk)) != 0)
	{
		const uint32_t DataStatus = NRF_USBD->EPDATASTATUS;
		NRF_USBD->EPDATASTATUS = DataStatus;
		__ISB();
		__DSB();

		const bool IsControlIn =
			(IntStatus & USBD_INTEN_EP0DATADONE_Msk) != 0 &&
			(NRF_USBD->BMREQUESTTYPE & TUSB_DIR_IN_MASK) != 0;
		const bool IsControlOut =
			(IntStatus & USBD_INTEN_EP0DATADONE_Msk) != 0 &&
			(NRF_USBD->BMREQUESTTYPE & TUSB_DIR_IN_MASK) == 0;

		/*
		 * OUT before IN. An OUT endpoint whose DMA is not restarted promptly
		 * stops the host sending, and letting IN completions run first delays
		 * that restart by the whole loop. The original of this driver gave
		 * that priority to one endpoint by number; every OUT endpoint has the
		 * same need, so the whole loop moves ahead instead.
		 */
		for (uint8_t EpNum = 0;
			 EpNum < NRFX_DCD_EP_CBI_COUNT;
			 EpNum++)
		{
			if (!tu_bit_test(DataStatus, 16 + EpNum) &&
				!(EpNum == 0 && IsControlOut))
			{
				continue;
			}

			nRFUsbdXfer_t *pXfer =
				nRFUsbdGetXfer(EpNum, TUSB_DIR_OUT);

			if (pXfer->Started &&
				pXfer->ActualLen < pXfer->TotalLen)
			{
				pXfer->DataReceived = false;
				nRFUsbdStartOutDma(EpNum);
			}
			else
			{
				pXfer->DataReceived = true;
			}
		}

		for (uint8_t EpNum = 0;
			 EpNum < NRFX_DCD_EP_CBI_COUNT;
			 EpNum++)
		{
			if (!tu_bit_test(DataStatus, EpNum) &&
				!(EpNum == 0 && IsControlIn))
			{
				continue;
			}

			nRFUsbdXfer_t *pXfer =
				nRFUsbdGetXfer(EpNum, TUSB_DIR_IN);
			const uint8_t TransferLen =
				(uint8_t)NRF_USBD->EPIN[EpNum].AMOUNT;

			pXfer->pBuffer += TransferLen;
			pXfer->ActualLen += TransferLen;

			if (pXfer->ActualLen < pXfer->TotalLen)
			{
				nRFUsbdStartInDma(EpNum);
			}
			else
			{
				dcd_event_xfer_complete(
					0,
					EpNum | TUSB_DIR_IN_MASK,
					pXfer->ActualLen,
					XFER_RESULT_SUCCESS,
					true);
			}
		}

	}
}

void dcd_edpt_close(uint8_t RhPort, uint8_t EpAddr)
{
	const uint8_t EpNum = tu_edpt_number(EpAddr);
	const uint8_t Dir = tu_edpt_dir(EpAddr);

	if (EpNum == 0 || EpNum >= NRFX_DCD_EP_CBI_COUNT)
	{
		return;
	}

	bool DmaEnded;
	uint32_t DataStatusBit;

	if (Dir == TUSB_DIR_OUT)
	{
		NRF_USBD->INTENCLR =
			(1UL << (USBD_INTEN_ENDEPOUT0_Pos + EpNum));
		NRF_USBD->EPOUTEN &= ~(1UL << (EpNum));

		DmaEnded = NRF_USBD->EVENTS_ENDEPOUT[EpNum] != 0;
		NRF_USBD->EVENTS_ENDEPOUT[EpNum] = 0;
		DataStatusBit = (1UL << (16 + EpNum));
	}
	else
	{
		NRF_USBD->INTENCLR =
			(1UL << (USBD_INTEN_ENDEPIN0_Pos + EpNum));
		NRF_USBD->EPINEN &= ~(1UL << (EpNum));

		DmaEnded = NRF_USBD->EVENTS_ENDEPIN[EpNum] != 0;
		NRF_USBD->EVENTS_ENDEPIN[EpNum] = 0;
		DataStatusBit = (1UL << (EpNum));
	}

	__ISB();
	__DSB();

	if (DmaEnded)
	{
		nRFUsbdDmaEnd();
	}

	NRF_USBD->EPDATASTATUS = DataStatusBit;
	__ISB();
	__DSB();

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpNum, Dir);
	pXfer->Started = false;
	pXfer->DataReceived = false;
	pXfer->IsoInTransferReady = false;

	usbd_edpt_clear_stall(RhPort, EpAddr);
}

//
// The USBD interrupt belongs to this file. It is the driver that owns the
// controller, so the vector goes straight to its handler rather than through
// anything that has to be configured first. An application driving this
// driver without IOsonata's UsbDev on top still gets a serviced interrupt.
//
void USBD_IRQHandler(void)
{
	dcd_int_handler(0);
}
