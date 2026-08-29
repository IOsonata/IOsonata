/**-------------------------------------------------------------------------
@file	usbd_nrfx.cpp

@brief	USB device port layer for Nordic parts.

One file for the whole family. Two controllers appear in it and the choice is
made on the MDK capability symbol, not on a part name :

	USBD_PRESENT	nRF52840 and relatives. Nordic's own full speed
					controller, with the USB regulator and cable detect in
					POWER and four errata around the enable sequence.

	USBHS_PRESENT	nRF54LM20. A DesignWare core behind a Nordic wrapper,
					running at high speed. The device stack's own dwc2 layer
					starts the regulator and the PHY, so the controller step
					is not repeated here. What it does not do is ask for the
					24 MHz clock the PHY runs from, or read the regulator's
					VBUS state, and both are done below.

What both have in common is a clock the PHY needs, cable detect, the
interrupt and the unique id, and that is what this layer exists for.

The POWER or CLOCK interrupt is deliberately not taken. On this family that
vector is claimed by MPSL whenever a radio stack is in the build, and MPSL
hands the whole vector to its own handler, so a USB driver that wanted cable
events from it would either lose them or fight for the vector. Bus power is
polled from UsbdProcess instead, which reads the same bit, works the same
under a SoftDevice, under MPSL and bare metal, and costs one register read
per pass. Attach is then noticed within one pump interval, which enumeration
does not care about.

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
#include <string.h>

#include "nrf.h"
#include "nrf_peripherals.h"
#include "hal/nrf_ficr.h"

#if defined(USBD_PRESENT)
#define NRFX_USBD_HAS_USBD					1
#include "nrf_erratas.h"
#elif defined(USBHS_PRESENT)
#define NRFX_USBD_HAS_USBHS				1
#include "hal/nrf_clock.h"
#include "hal/nrf_vregusb.h"
#include "soc/nrfx_coredep.h"
#else
#error "usbd_nrfx: this part has no USB device controller"
#endif

#ifdef NRFX_USBD_HAS_USBHS
#include "device/dcd.h"
#endif

#include "usb/usbd.h"

// The same reasoning as nvm_nrfx: a SoftDevice build does not always define
// SOFTDEVICE_PRESENT, and on the nRF54L one archive serves SoftDevice, link
// controller and bare metal alike. The path is compiled in and the choice is
// made at run time, because getting it wrong the other way drives the clock
// while the SoftDevice owns it, and that is not a compile error.
#if defined(SOFTDEVICE_PRESENT) || defined(S112) || defined(S113) || \
	defined(S132) || defined(S140) || defined(S145)
#define NRFX_USBD_SOFTDEVICE				1
#elif defined(NRF54L_SERIES) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#define NRFX_USBD_SOFTDEVICE				1
#endif

#ifdef NRFX_USBD_SOFTDEVICE
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_error.h"
#endif

#ifndef NRFX_USBD_XTAL_WAIT_LOOPS
#define NRFX_USBD_XTAL_WAIT_LOOPS			2000000UL
#endif

#ifndef NRFX_USBD_READY_WAIT_LOOPS
#define NRFX_USBD_READY_WAIT_LOOPS			2000000UL
#endif

#ifdef NRFX_USBD_HAS_USBD

//
// Errata registers. These addresses are not in the MDK because the registers
// they name are not documented; they come from Nordic's own driver and from
// the errata sheet. nrf52_errata_nnn() decides whether the part in hand needs
// each one, so the values below are only ever written where they apply.
//
#define NRFX_USBD_ERRATA_171_REG			0x4006EC14UL
#define NRFX_USBD_ERRATA_187_REG			0x4006ED14UL
#define NRFX_USBD_ERRATA_166_REG_A			(NRF_USBD_BASE + 0x800UL)
#define NRFX_USBD_ERRATA_166_REG_B			(NRF_USBD_BASE + 0x804UL)

#define NRFX_USBD_REG32(a)					(*(volatile uint32_t *)(a))

#endif

//
// One USB controller per part, so this is file scope. Nothing here is written
// from an interrupt : the only interrupt this file owns hands straight to the
// device stack and touches none of it.
//
static UsbdCfg_t s_UsbdCfg;
static bool s_UsbdInitialized = false;
static bool s_UsbdStarted = false;
static bool s_UsbdXtalHeld = false;
static bool s_UsbdVbusLast = false;

#ifdef NRFX_USBD_HAS_USBHS
// The nRF52 reads bus power from a register whenever it is asked. The nRF54
// reports edges only, so the level it leaves behind is kept here.
static bool s_UsbdVbusLevel = false;
#endif

#ifdef NRFX_USBD_SOFTDEVICE
static bool UsbdSdRunning(void)
{
	uint8_t en = 0;

	if (sd_softdevice_is_enabled(&en) != NRF_SUCCESS)
	{
		return false;
	}

	return en != 0;
}
#endif

__attribute__((weak)) bool UsbdXtalRequest(void)
{
#ifdef NRFX_USBD_SOFTDEVICE
	if (UsbdSdRunning())
	{
		uint32_t running = 0;

		if (sd_clock_hfclk_request() != NRF_SUCCESS)
		{
			return false;
		}

		for (uint32_t i = 0; i < NRFX_USBD_XTAL_WAIT_LOOPS; i++)
		{
			if (sd_clock_hfclk_is_running(&running) != NRF_SUCCESS)
			{
				return false;
			}

			if (running != 0)
			{
				return true;
			}
		}

		(void)sd_clock_hfclk_release();

		return false;
	}
#endif

#if defined(NRF52_SERIES)
	if ((NRF_CLOCK->HFCLKSTAT &
		 (CLOCK_HFCLKSTAT_STATE_Msk | CLOCK_HFCLKSTAT_SRC_Msk)) ==
		(CLOCK_HFCLKSTAT_STATE_Msk |
		 (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)))
	{
		return true;
	}

	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	for (uint32_t i = 0; i < NRFX_USBD_XTAL_WAIT_LOOPS; i++)
	{
		if (NRF_CLOCK->EVENTS_HFCLKSTARTED != 0)
		{
			NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
			return true;
		}
	}

	return false;
#elif NRF_CLOCK_HAS_HFCLK24M
	//
	// The USBHS PHY runs from the 24 MHz clock, not from the radio crystal.
	// This is a separate domain with its own task and its own started event,
	// and neither the device stack's dwc2 layer nor its Nordic header asks
	// for it, so it is asked for here.
	//
	if (nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK24M, NULL))
	{
		return true;
	}

	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLK24MSTARTED);
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLK24MSTART);

	for (uint32_t i = 0; i < NRFX_USBD_XTAL_WAIT_LOOPS; i++)
	{
		if (nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLK24MSTARTED))
		{
			nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLK24MSTARTED);
			return true;
		}
	}

	return false;
#else
#error "usbd_nrfx: no clock source known for this USB controller"
#endif
}

__attribute__((weak)) void UsbdXtalRelease(void)
{
#ifdef NRFX_USBD_SOFTDEVICE
	if (UsbdSdRunning())
	{
		(void)sd_clock_hfclk_release();
		return;
	}
#endif

	//
	// Left running on both parts. Other peripherals take these clocks without
	// counting, so stopping one here would stop it under them.
	//
}

#ifdef NRFX_USBD_HAS_USBD

static void UsbdErrataWrite(uint32_t Reg, uint32_t Value)
{
	NRFX_USBD_REG32(Reg) = Value;
}

/**
 * Errata 187 and 171 are applied around the enable and taken away again once
 * the controller reports itself ready. Errata 166 is undone at the same time.
 * Nordic's own driver does exactly this and the order matters : the writes
 * before the enable are what make the enable work.
 */
static void UsbdErrataApply(void)
{
	if (nrf52_errata_187())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_187_REG, 0x00000003UL);
	}

	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x000000C0UL);
	}
}

static void UsbdErrataRevert(void)
{
	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x00000000UL);
	}

	if (nrf52_errata_187())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_187_REG, 0x00000000UL);
	}

	if (nrf52_errata_166())
	{
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_166_REG_A) = 0x7E3UL;
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_166_REG_B) = 0x40UL;
		__ISB();
		__DSB();
	}
}

/**
 * The part does not leave USB low power on its own. The device stack writes
 * LOWPOWER on suspend and signals resume, but no driver anywhere writes
 * ForceNormal except the remote wakeup path, which is the device asking the
 * host to wake, not the other way round. Without this the board is gone after
 * the host sleeps and needs a power cycle.
 */
static void UsbdLowPowerExit(void)
{
	if (NRF_USBD->LOWPOWER == USBD_LOWPOWER_LOWPOWER_ForceNormal)
	{
		return;
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	NRF_USBD->LOWPOWER = USBD_LOWPOWER_LOWPOWER_ForceNormal;

	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x000000C0UL);
	}

	for (uint32_t i = 0; i < NRFX_USBD_READY_WAIT_LOOPS; i++)
	{
		if (NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_READY_Msk)
		{
			break;
		}
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;

	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x00000000UL);
	}
}

static bool UsbdStartCtrlr(void)
{
	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	__ISB();
	__DSB();

	UsbdErrataApply();

	NRF_USBD->ENABLE = 1;
	__ISB();
	__DSB();

	uint32_t i;

	for (i = 0; i < NRFX_USBD_READY_WAIT_LOOPS; i++)
	{
		if (NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_READY_Msk)
		{
			break;
		}
	}

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_READY_Msk) == 0)
	{
		UsbdErrataRevert();
		NRF_USBD->ENABLE = 0;
		return false;
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	__ISB();
	__DSB();

	UsbdErrataRevert();

	//
	// The regulator reports itself usable separately from the controller, and
	// pulling up before it does gives the host a device that cannot answer.
	//
	for (i = 0; i < NRFX_USBD_READY_WAIT_LOOPS; i++)
	{
		if (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_OUTPUTRDY_Msk)
		{
			break;
		}
	}

	if ((NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_OUTPUTRDY_Msk) == 0)
	{
		NRF_USBD->ENABLE = 0;
		return false;
	}

	//
	// The stock Nordic power-ready path arms this interrupt before the pull-up
	// is enabled. This port replaces that power handler, so do the same here.
	// The DCD expands the mask after it receives the first bus reset.
	//
	NRF_USBD->EVENTS_USBRESET = 0;
	NRF_USBD->INTENCLR = NRF_USBD->INTEN;
	NRF_USBD->INTENSET = USBD_INTEN_USBRESET_Msk;
	NVIC_ClearPendingIRQ(USBD_IRQn);

	return true;
}

static void UsbdStopCtrlr(void)
{
	NRF_USBD->INTEN = 0;
	NRF_USBD->USBPULLUP = 0;
	NRF_USBD->ENABLE = 0;
	__ISB();
	__DSB();
}

#endif	// NRFX_USBD_HAS_USBD

#ifdef NRFX_USBD_HAS_USBHS

//
// Undocumented VREGUSB status register. The events below report the edges and
// are in the MDK, but the level is not, and something has to answer what the
// state is at start up before any edge has happened. Offset and bit are as
// used by Zephyr's regulator_nrf_vregusb driver, which says in its own source
// that the register is not part of NRF_VREGUSB_Type.
//
#define NRFX_USBD_VREGUSB_STATUS_OFS		0x400UL
#define NRFX_USBD_VREGUSB_STATUS_VBUSDET	(1UL << 2)

/**
 * Consume the VBUS edges the regulator has recorded and answer the level they
 * leave behind. Polled rather than taken as an interrupt for the same reason
 * as the nRF52 side, and the events hold until they are cleared, so nothing
 * is lost between passes.
 */
static bool UsbdVbusPoll(void)
{
	if (nrf_vregusb_event_check(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_DETECTED))
	{
		nrf_vregusb_event_clear(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_DETECTED);
		s_UsbdVbusLevel = true;
	}

	if (nrf_vregusb_event_check(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_REMOVED))
	{
		nrf_vregusb_event_clear(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_REMOVED);
		s_UsbdVbusLevel = false;
	}

	return s_UsbdVbusLevel;
}

/**
 * Power the USBHS wrapper, its PHY and the core, in the order the part
 * requires. Seven steps, and the two delays between them are not optional :
 * a core register read before the PHY clock is up hangs the bus.
 *
 * The device stack's own Nordic header has this same sequence, but the whole
 * of it sits behind NRF54LM20A_ENGA_XXAA, so on a production part it compiles
 * to an empty function and the core is never powered. Doing it here, before
 * the device stack is initialised, leaves that empty function harmless and
 * keeps us off a patch against an external tree.
 *
 * The sequence itself is not invented here. It follows Zephyr's
 * usbhs_enable_core in drivers/usb/udc/udc_dwc2_vendor_quirks.h.
 */
static bool UsbdStartCtrlr(void)
{
	// Core first, PHY still held in reset
	NRF_USBHS->ENABLE = USBHS_ENABLE_CORE_Msk;

	// Device role, and hold VBUSVALID asserted while the core comes up
	NRF_USBHS->PHY.OVERRIDEVALUES =
		USBHS_PHY_OVERRIDEVALUES_ID_Device << USBHS_PHY_OVERRIDEVALUES_ID_Pos;
	NRF_USBHS->PHY.INPUTOVERRIDE =
		USBHS_PHY_INPUTOVERRIDE_ID_Msk | USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk;

	// Release the PHY power on reset
	NRF_USBHS->ENABLE = USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk;

	// PHY clock start
	nrfx_coredep_delay_us(45);

	NRF_USBHS->TASKS_START = USBHS_TASKS_START_TASKS_START_Trigger;

	// Settle before anything reads a core register
	nrfx_coredep_delay_us(2);

	// Drop the VBUSVALID override and keep the device role. The pull up goes
	// on when the device stack clears SftDiscon, not here.
	NRF_USBHS->PHY.INPUTOVERRIDE = USBHS_PHY_INPUTOVERRIDE_ID_Msk;

	//
	// The wrapper and the core are separate peripheral blocks at separate
	// addresses. Without this the writes above can still be sitting in the
	// write buffer when the core is first read.
	//
	__DSB();

	return true;
}

static void UsbdStopCtrlr(void)
{
	NVIC_DisableIRQ(USBHS_IRQn);

	NRF_USBHS->ENABLE = 0;
	__ISB();
	__DSB();
}

//
// The nRF52 USBD vector belongs to tu_dcd_nrfx.c, the driver that owns that
// controller. The nRF54 has no equivalent : its dwc2 driver is vendor neutral
// and supplies no Nordic vector, so this file provides it. Either way the
// vector reaches dcd_int_handler directly, because nothing above the
// controller should have to be configured first for an interrupt to be
// serviced.
//
extern "C" void USBHS_IRQHandler(void)
{
	dcd_int_handler(0);
}

#endif	// NRFX_USBD_HAS_USBHS

bool UsbdVbusDetected(void)
{
#ifdef NRFX_USBD_HAS_USBD
	return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
#else
	return UsbdVbusPoll();
#endif
}

UsbdSpeed_t UsbdMaxSpeed(void)
{
#ifdef NRFX_USBD_HAS_USBHS
	return USBD_SPEED_HIGH;
#else
	return USBD_SPEED_FULL;
#endif
}

size_t UsbdGetSerial(char *pBuff, size_t BuffLen)
{
	static const char hex[] = "0123456789ABCDEF";
	size_t cnt = 0;

	if (pBuff == nullptr || BuffLen == 0)
	{
		return 0;
	}

	//
	// nrf_ficr_deviceid_get and not FICR->DEVICEID, because the nRF52 keeps
	// the id flat and the nRF54 keeps it under INFO, and the HAL already
	// knows which.
	//
	for (int i = 0; i < 2; i++)
	{
		uint32_t id = nrf_ficr_deviceid_get(NRF_FICR, (uint32_t)i);

		for (int n = 7; n >= 0; n--)
		{
			if (cnt + 1 >= BuffLen)
			{
				pBuff[cnt] = '\0';
				return cnt;
			}

			pBuff[cnt++] = hex[(id >> (n * 4)) & 0x0F];
		}
	}

	pBuff[cnt] = '\0';

	return cnt;
}

bool UsbdInit(const UsbdCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	memcpy(&s_UsbdCfg, pCfg, sizeof(UsbdCfg_t));

#ifdef NRFX_USBD_HAS_USBHS
	//
	// The regulator reports nothing until it is started, so it is started
	// here rather than left to the device stack, which starts it much later
	// and only on the way to enabling the core.
	//
	nrf_vregusb_event_clear(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_DETECTED);
	nrf_vregusb_event_clear(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_REMOVED);
	nrf_vregusb_task_trigger(NRF_VREGUSB, NRF_VREGUSB_TASK_START);

	s_UsbdVbusLevel =
		(*(volatile uint32_t *)((uintptr_t)NRF_VREGUSB +
								NRFX_USBD_VREGUSB_STATUS_OFS) &
		 NRFX_USBD_VREGUSB_STATUS_VBUSDET) != 0;
#endif

	s_UsbdInitialized = true;
	s_UsbdStarted = false;
	s_UsbdVbusLast = UsbdVbusDetected();

	return true;
}

bool UsbdStart(void)
{
	if (s_UsbdInitialized == false)
	{
		return false;
	}

	if (s_UsbdStarted)
	{
		return true;
	}

	if (UsbdVbusDetected() == false)
	{
		//
		// No cable. Not a failure : the poll in UsbdProcess reports the
		// attach and the caller comes back.
		//
		return false;
	}

	if (UsbdXtalRequest() == false)
	{
		return false;
	}

	s_UsbdXtalHeld = true;

#ifdef NRFX_USBD_HAS_USBD
	NVIC_SetPriority(USBD_IRQn, s_UsbdCfg.IntPrio);
#else
	NVIC_SetPriority(USBHS_IRQn, s_UsbdCfg.IntPrio);
#endif

	if (UsbdStartCtrlr() == false)
	{
		UsbdXtalRelease();
		s_UsbdXtalHeld = false;
		return false;
	}

	s_UsbdStarted = true;
	s_UsbdVbusLast = true;

	return true;
}

void UsbdStop(void)
{
	if (s_UsbdStarted == false)
	{
		return;
	}

	//
	// The interrupt goes first. Everything after this runs with no chance of
	// the device stack being entered from the vector while it is being torn
	// down.
	//
#ifdef NRFX_USBD_HAS_USBD
	NVIC_DisableIRQ(USBD_IRQn);
#else
	NVIC_DisableIRQ(USBHS_IRQn);
#endif

	UsbdStopCtrlr();

	if (s_UsbdXtalHeld)
	{
		UsbdXtalRelease();
		s_UsbdXtalHeld = false;
	}

	s_UsbdStarted = false;
}

void UsbdProcess(void)
{
	if (s_UsbdInitialized == false)
	{
		return;
	}

#ifdef NRFX_USBD_HAS_USBD
	if (s_UsbdStarted && s_UsbdCfg.bLowPowerSuspend == false)
	{
		UsbdLowPowerExit();
	}
#endif

	bool vbus = UsbdVbusDetected();

	if (vbus == s_UsbdVbusLast)
	{
		return;
	}

	s_UsbdVbusLast = vbus;

	if (s_UsbdCfg.EvtHandler != nullptr)
	{
		s_UsbdCfg.EvtHandler(vbus ? USBD_EVT_ATTACHED :
									  USBD_EVT_DETACHED);
	}
}
