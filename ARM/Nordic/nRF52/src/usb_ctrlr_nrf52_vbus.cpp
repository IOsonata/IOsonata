/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrf52_vbus.cpp

@brief	USB cable attach and removal interrupt of the nRF52

The USB port owns its cable events. Without a SoftDevice they are the
USBDETECTED and USBREMOVED events of the POWER peripheral, on the
POWER_CLOCK vector it shares with the clock (power_clock_irq_nrf52.h). With
a SoftDevice, the SoftDevice owns POWER and reports them as SoC events. Both
queue the USB process event, which reports the edge and connects or
disconnects.

@author	Hoang Nguyen Hoan
@date	Oct. 3, 2026

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
#include <stdint.h>

#include "nrf.h"

#include "usb/usb.h"
#include "power_clock_irq_nrf52.h"

// Same test as usb_ctrlr_nrf52.cpp
#if defined(NRF52_SERIES) && \
	(defined(SOFTDEVICE_PRESENT) || defined(S140))
#define SOFTDEVICE_PRESENT			1
#endif

#ifdef SOFTDEVICE_PRESENT
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_mbr.h"
#include "nrf_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#endif

// Called by UsbCtrlrInit of usb_ctrlr_nrf52.cpp
void nRFUsbdVbusIntInit(uint32_t Prio);

// Shared vector part, without a SoftDevice. Interrupt context.
void nRFUsbPowerIrqHandler(void)
{
	bool edge = false;

	if (NRF_POWER->EVENTS_USBDETECTED != 0U)
	{
		NRF_POWER->EVENTS_USBDETECTED = 0U;
		edge = true;
	}

	if (NRF_POWER->EVENTS_USBREMOVED != 0U)
	{
		NRF_POWER->EVENTS_USBREMOVED = 0U;
		edge = true;
	}

	if (edge)
	{
		UsbProcessQue(0);
	}
}

#ifdef SOFTDEVICE_PRESENT
// SoftDevice SoC events, SD_EVT interrupt context
static void nRFUsbdSocEvt(uint32_t EvtId, void *pCtx)
{
	(void)pCtx;

	if (EvtId == NRF_EVT_POWER_USB_DETECTED || EvtId == NRF_EVT_POWER_USB_REMOVED)
	{
		UsbProcessQue(0);
	}
}

NRF_SDH_SOC_OBSERVER(s_nRFUsbdSocObserver, 0, nRFUsbdSocEvt, nullptr);

// Value at the magic number location of every SoftDevice image
#define NRFUSBD_SD_MAGIC_NUMBER		0x51B1E5DBUL

// Whether a SoftDevice is programmed and enabled. sd_softdevice_is_enabled is
// an SVC that only a programmed SoftDevice implements, so the image is looked
// for first: its magic number follows the MBR.
static bool nRFUsbdVbusSdRunning(void)
{
	const volatile uint32_t *pMagic = (const volatile uint32_t *)(
		MBR_SIZE + SOFTDEVICE_INFO_STRUCT_OFFSET + 4U);

	if (*pMagic != NRFUSBD_SD_MAGIC_NUMBER)
	{
		return false;
	}

	uint8_t en = 0;

	return sd_softdevice_is_enabled(&en) == NRF_SUCCESS && en != 0;
}

static void nRFUsbdSdEnable(void)
{
	(void)sd_power_usbdetected_enable(1);
	(void)sd_power_usbremoved_enable(1);
}

static void nRFUsbdVbusPowerIntEnable(void);

// A SoftDevice enabled after USB init takes POWER over. sd_softdevice_enable
// refuses to start (NRF_ERROR_SDM_INCORRECT_INTERRUPT_CONFIGURATION) while the
// POWER_CLOCK interrupt is enabled, so it is released before, and the cable
// events are asked from the SoftDevice once it runs. Taken back when the
// SoftDevice is disabled.
static void nRFUsbdSdState(nrf_sdh_state_evt_t State, void *pCtx)
{
	(void)pCtx;

	switch (State)
	{
		case NRF_SDH_EVT_STATE_ENABLE_PREPARE:
			NRF_POWER->INTENCLR = POWER_INTENCLR_USBDETECTED_Msk |
								  POWER_INTENCLR_USBREMOVED_Msk;
			NVIC_DisableIRQ(POWER_CLOCK_IRQn);
			NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
			break;

		case NRF_SDH_EVT_STATE_ENABLED:
			nRFUsbdSdEnable();
			break;

		case NRF_SDH_EVT_STATE_DISABLED:
			nRFUsbdVbusPowerIntEnable();
			break;

		default:
			break;
	}
}

NRF_SDH_STATE_OBSERVER(s_nRFUsbdSdStateObserver, 0) = {
	.handler = nRFUsbdSdState,
	.p_context = nullptr
};
#endif

// USB interrupt priority, kept for taking POWER back from a SoftDevice
static uint32_t s_nRFUsbdVbusPrio;

// POWER cable events on the shared POWER_CLOCK vector
static void nRFUsbdVbusPowerIntEnable(void)
{
	NRF_POWER->EVENTS_USBDETECTED = 0U;
	NRF_POWER->EVENTS_USBREMOVED = 0U;
	NRF_POWER->INTENSET = POWER_INTENSET_USBDETECTED_Msk |
						  POWER_INTENSET_USBREMOVED_Msk;
	nRFPowerClockIrqEnable(s_nRFUsbdVbusPrio);
}

// Called by UsbCtrlrInit
void nRFUsbdVbusIntInit(uint32_t Prio)
{
	s_nRFUsbdVbusPrio = Prio;

#ifdef SOFTDEVICE_PRESENT
	if (nRFUsbdVbusSdRunning())
	{
		nRFUsbdSdEnable();
		return;
	}
#endif

	nRFUsbdVbusPowerIntEnable();
}
