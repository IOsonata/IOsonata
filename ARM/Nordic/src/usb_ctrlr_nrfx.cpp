/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrfx.cpp

@brief	USB controller for Nordic parts.

One file for the whole nRF family, the same shape as uart_nrfx.cpp. Direct
register control throughout, no nrfx USB driver.

Three things used to live in three files and are joined here because they are
one job: bus power, clock and VBUS detection, the endpoint and DMA registers,
and the capability record. Splitting power from registers forced a caller to
call UsbdStart() and then UsbdCtrlrStart(), and forced the same facts to be
stated in usbd.h and usbd_ctrlr.h. UsbCtrlrStart() is now one call.

Two peripherals are covered, selected by the MDK macros and never both:

  USBD_PRESENT		nRF52840, nRF52833, nRF52820, nRF5340. Full speed.
  USBHS_PRESENT		nRF54H20, nRF54LM20A, nRF54LM20B. High speed, with its
					own USBHSCORE and VREGUSB.

Both are device only. No Nordic part has a USB host controller, so there is no
usbh split here, and the entry points keep the role neutral UsbCtrlr name.

DevNo selects the controller. Every nRF part has exactly one, USB_CTRLR_CNT is
1, so the entry points validate DevNo and the state stays a singleton. Arraying
it is work for the first part that carries two.

@author	Hoang Nguyen Hoan
@date	Sep. 3, 2026

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
#include <string.h>

#include "nrf.h"
#include "nrf_peripherals.h"
#include "nrf_erratas.h"
#include "hal/nrf_ficr.h"

#include "usb/usb.h"

#if !defined(USBD_PRESENT) && !defined(USBHS_PRESENT)
#error "usb_ctrlr_nrfx: this part has no USB controller"
#endif

/// Only DevNo 0 exists on every nRF part shipped so far.
static inline __attribute__((always_inline))
bool nRFUsbValidDevNo(int DevNo)
{
	return DevNo >= 0 && DevNo < USB_CTRLR_CNT;
}

//
// Bus power, clock and VBUS. Common to both peripherals, with the part
// differences handled inline the way uart_nrfx.cpp does.
//

#if defined(USBD_PRESENT)
#define NRFX_USBD_HAS_USBD					1
#elif defined(USBHS_PRESENT)
#define NRFX_USBD_HAS_USBHS				1
// The USBHS clock and VBUS regulator are separate peripherals on this part.
#include "hal/nrf_clock.h"
#include "hal/nrf_vregusb.h"
#include "soc/nrfx_coredep.h"
#else
#error "usb_ctrlr_nrfx: this part has no USB device controller"
#endif


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
#define NRFX_USBD_ERRATA_UNLOCK_REG		0x4006EC00UL
#define NRFX_USBD_ERRATA_UNLOCK_KEY		0x00009375UL
#define NRFX_USBD_ERRATA_171_REG			0x4006EC14UL
#define NRFX_USBD_ERRATA_187_REG			0x4006ED14UL
#define NRFX_USBD_ERRATA_166_REG_A			(NRF_USBD_BASE + 0x800UL)
#define NRFX_USBD_ERRATA_166_REG_B			(NRF_USBD_BASE + 0x804UL)

#define NRFX_USBD_REG32(a)					(*(volatile uint32_t *)(a))

#endif

// One USB controller per part, so the common power/clock state is file scope.
// Controller interrupts are owned by the corresponding UsbdCtrlr backend.
static UsbCtrlrCfg_t s_UsbdCfg;
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
/**
 * Whether a SoftDevice is programmed at all.
 *
 * sd_softdevice_is_enabled is an SVC. On a part with no SoftDevice in flash
 * nothing implements that vector, so the call lands in the default handler
 * and stops there. The image has to be found before it may be asked
 * anything. Same test as SdPresent in nvm_nrfx.cpp.
 */
static bool UsbdSdPresent(void)
{
#if defined(SD_MAGIC_NUMBER) && defined(MBR_SIZE)
	return SD_MAGIC_NUMBER_GET(MBR_SIZE) == SD_MAGIC_NUMBER;
#else
	return false;
#endif
}

static bool UsbdSdRunning(void)
{
	uint8_t en = 0;

	if (UsbdSdPresent() == false)
	{
		return false;
	}

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
	// The USBHS PHY runs from the 24 MHz clock, not from the radio crystal.
	// This is a separate domain with its own task and its own started event.
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

	// Left running on both parts. Other peripherals take these clocks without
	// counting, so stopping one here would stop it under them.
}

#ifdef NRFX_USBD_HAS_USBD

/**
 * Write one errata register.
 *
 * The registers these workarounds use sit behind an unlock word. When that
 * word reads zero the block is locked and a write to the errata register is
 * discarded, so the key has to be presented around it. Errata 187 is titled
 * "USBD cannot be enabled", and losing its write is not visible until ENABLE
 * is set and EVENTCAUSE never reports READY.
 *
 * Same sequence as Nordic's own driver.
 */
static void UsbdErrataWrite(uint32_t Reg, uint32_t Value)
{
	if (NRFX_USBD_REG32(NRFX_USBD_ERRATA_UNLOCK_REG) == 0)
	{
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_UNLOCK_REG) =
			NRFX_USBD_ERRATA_UNLOCK_KEY;
		NRFX_USBD_REG32(Reg) = Value;
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_UNLOCK_REG) =
			NRFX_USBD_ERRATA_UNLOCK_KEY;
	}
	else
	{
		NRFX_USBD_REG32(Reg) = Value;
	}
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
 * The part does not leave USB low power on its own. The native controller
 * writes LOWPOWER on suspend and signals resume, but no driver anywhere
 * writes ForceNormal except the remote wakeup path, which is the device
 * asking the host to wake, not the other way round. Without this the board is
 * gone after the host sleeps and needs a power cycle.
 */
/**
 * Leave USBD low power without waiting.
 *
 * Requesting the exit and observing READY are separate steps here. Spinning
 * for READY put a bound of NRFX_USBD_READY_WAIT_LOOPS iterations directly
 * under the application main loop, which is around a hundred milliseconds at
 * 64 MHz if the bit is slow to arrive. Nothing needs the exit to have finished
 * by the time this returns, so the request is raised and the next call
 * finishes it.
 */
static bool s_LowPowerExitPending = false;

static void UsbdLowPowerExitFinish(void)
{
	if (!s_LowPowerExitPending ||
		(NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_READY_Msk) == 0U)
	{
		return;
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;

	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x00000000UL);
	}

	s_LowPowerExitPending = false;
}

static void UsbdLowPowerExit(void)
{
	// Retire a request raised by an earlier call before looking at anything
	// else. Only this clears the errata register the request set.
	UsbdLowPowerExitFinish();

	if (s_LowPowerExitPending ||
		NRF_USBD->LOWPOWER == USBD_LOWPOWER_LOWPOWER_ForceNormal)
	{
		return;
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	NRF_USBD->LOWPOWER = USBD_LOWPOWER_LOWPOWER_ForceNormal;

	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x000000C0UL);
	}

	s_LowPowerExitPending = true;
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

	// The regulator reports itself usable separately from the controller, and
	// pulling up before it does gives the host a device that cannot answer.
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

	// The stock Nordic power-ready path arms USBRESET before the pull-up is
	// enabled. This port replaces that power handler, so do the same here. The
	// native nRF52 controller expands the mask after the first bus reset.
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

// Undocumented VREGUSB status register. The events below report the edges and
// are in the MDK, but the level is not, and something has to answer what the
// state is at start up before any edge has happened. Offset and bit are as
// used by Zephyr's regulator_nrf_vregusb driver, which says in its own source
// that the register is not part of NRF_VREGUSB_Type.
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
 * requires. The two delays are not optional: a core register read before the
 * PHY clock is up can hang the bus. This sequence completes before
 * nRFUsbRegStart() touches NRF_USBHSCORE.
 *
 * The sequence follows Nordic's USBHS integration requirements, matching the
 * ordering used by Zephyr's Nordic USBHS vendor quirk implementation.
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

	// Drop the VBUSVALID override and keep the device role. The native
	// controller connects when it clears SftDiscon.
	NRF_USBHS->PHY.INPUTOVERRIDE = USBHS_PHY_INPUTOVERRIDE_ID_Msk;

	// The wrapper and the core are separate peripheral blocks at separate
	// addresses. Without this the writes above can still be sitting in the
	// write buffer when the core is first read.
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

#endif	// NRFX_USBD_HAS_USBHS

static bool nRFUsbVbusDetected(void)
{
#ifdef NRFX_USBD_HAS_USBD
	return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
#else
	return UsbdVbusPoll();
#endif
}

static UsbSpeed_t nRFUsbMaxSpeed(void)
{
#ifdef NRFX_USBD_HAS_USBHS
	return USB_SPEED_HIGH;
#else
	return USB_SPEED_FULL;
#endif
}

static size_t nRFUsbSerial(char *pBuff, size_t BuffLen)
{
	static const char hex[] = "0123456789ABCDEF";
	size_t cnt = 0;

	if (pBuff == nullptr || BuffLen == 0)
	{
		return 0;
	}

	// nrf_ficr_deviceid_get and not FICR->DEVICEID, because the nRF52 keeps
	// the id flat and the nRF54 keeps it under INFO, and the HAL already knows
	// which.
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

static bool nRFUsbPowerInit(const UsbCtrlrCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	memcpy(&s_UsbdCfg, pCfg, sizeof(UsbCtrlrCfg_t));

#ifdef NRFX_USBD_HAS_USBHS
	// Start the regulator here so VBUS state is available before the native
	// controller is enabled. The wrapper/core power sequence happens later in
	// UsbdStart(), before nRFUsbRegStart() reads the DesignWare registers.
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
	s_UsbdVbusLast = nRFUsbVbusDetected();

	return true;
}

static bool nRFUsbPowerStart(void)
{
	if (s_UsbdInitialized == false)
	{
		return false;
	}

	if (s_UsbdStarted)
	{
		return true;
	}

	if (nRFUsbVbusDetected() == false)
	{
		// No cable. Not a failure: the poll in UsbdProcess reports the attach
		// and the caller comes back.
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

static void nRFUsbPowerStop(void)
{
	if (s_UsbdStarted == false)
	{
		return;
	}

	// Stop the controller interrupt before powering down the wrapper.
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

/**
 * Called from the application main loop through UsbCtrlrProcess. It must cost
 * nothing when there is nothing to do: a pending low power exit to retire, or
 * a VBUS edge. Neither waits.
 */
static void nRFUsbPowerProcess(void)
{
	if (s_UsbdInitialized == false)
	{
		return;
	}

#ifdef NRFX_USBD_HAS_USBD
	if (s_LowPowerExitPending)
	{
		UsbdLowPowerExitFinish();
	}
	else if (s_UsbdStarted && s_UsbdCfg.bLowPowerSuspend == false &&
			 NRF_USBD->LOWPOWER != USBD_LOWPOWER_LOWPOWER_ForceNormal)
	{
		UsbdLowPowerExit();
	}
#endif

	const bool vbus = nRFUsbVbusDetected();

	if (vbus == s_UsbdVbusLast)
	{
		return;
	}

	// The port reports bus power state through UsbCtrlrVbusDetected(). Turning
	// an edge into USB_EVT_ATTACHED or USB_EVT_DETACHED for the application is
	// the generic layer's job, so there is no cable callback here. The edge is
	// still tracked because the low power path above needs it.
	s_UsbdVbusLast = vbus;
}

//
// Endpoint and DMA registers. Exactly one of these compiles.
//

#if defined(USBD_PRESENT)

#ifndef USBD_PRESENT
#error "usbd_ctrlr_nrf52: this part has no USBD peripheral"
#endif

/* Keep the ISR/application synchronization independent of the C++ standard
 * library. These GCC builtins generate the same lock-free operations used by
 * the previous C11 atomic implementation on Cortex-M4. */
typedef bool atomic_flag;
typedef bool atomic_bool;
typedef uint_fast8_t atomic_uint_fast8_t;
typedef uint_fast16_t atomic_uint_fast16_t;

#define ATOMIC_FLAG_INIT false
#define atomic_load(p) __atomic_load_n((p), __ATOMIC_SEQ_CST)
#define atomic_store(p, v) __atomic_store_n((p), (v), __ATOMIC_SEQ_CST)
#define atomic_exchange(p, v) __atomic_exchange_n((p), (v), __ATOMIC_SEQ_CST)
#define atomic_compare_exchange_strong(p, expected, desired) \
	__atomic_compare_exchange_n((p), (expected), (desired), false, \
		__ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)
#define atomic_fetch_or(p, v) __atomic_fetch_or((p), (v), __ATOMIC_SEQ_CST)
#define atomic_fetch_and(p, v) __atomic_fetch_and((p), (v), __ATOMIC_SEQ_CST)
#define atomic_flag_test_and_set(p) __atomic_exchange_n((p), true, __ATOMIC_SEQ_CST)
#define atomic_flag_clear(p) __atomic_store_n((p), false, __ATOMIC_SEQ_CST)

enum
{
	NRFX_USBD_EP_COUNT = 8,
	NRFX_USBD_MAX_PACKET_SIZE = 64,
	NRFX_USBD_DMA_EP_NONE = 0xFFU,
};

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
	UsbCtrlrEvtHandler_t EvtHandler;
	void *pContext;
	bool SofEnabled;
	bool SetupDirIn;
} nRFUsbdCtrlr_t;

static nRFUsbdCtrlr_t s_Ctrlr;
static atomic_flag s_DmaRunning = ATOMIC_FLAG_INIT;
static atomic_uint_fast8_t s_DmaEpAddr;
// Bit N identifies the deferred endpoint. Direction is kept by the separate
// masks and the complete request is in Xfer[N][direction].
static atomic_uint_fast16_t s_PendingOut;
static atomic_uint_fast16_t s_PendingIn;
static atomic_bool s_PendingEp0Status;
static atomic_bool s_PendingEp0RcvOut;
static atomic_bool s_BusSuspended;
static atomic_bool s_SuspendPending;
static atomic_bool s_RemoteWakePending;
static atomic_bool s_HostResumePending;
static atomic_bool s_MacAwake;

#ifdef USB_CTRLR_TX_TIMING
// Controller timing. Off by default; define USB_CTRLR_TX_TIMING to build it
// in. Needs the DWT cycle counter already running. Read with a debugger; at
// 64 MHz one cycle is 15.6 ns.
//
// DmaStart  the errata 199 register write, the task write and the barriers
// Svc       nRFUsbdServicePending, the arbitration around it
// Isr       USBD_IRQHandler, since an interrupt landing inside a producer
//           call is charged to that call
uint32_t g_DmaStartMin = 0xFFFFFFFFU;
uint32_t g_DmaStartMax;
uint32_t g_DmaStartSum;
uint32_t g_DmaStartCnt;
uint32_t g_SvcMin = 0xFFFFFFFFU;
uint32_t g_SvcMax;
uint32_t g_SvcSum;
uint32_t g_SvcCnt;
uint32_t g_IsrMin = 0xFFFFFFFFU;
uint32_t g_IsrMax;
uint32_t g_IsrSum;
uint32_t g_IsrCnt;
// Collect   the entry prologue: reading and clearing event registers, with a
//           barrier pair per event cleared.
// Refill    nRFUsbdEmitXfer and everything it reaches, which is the whole
//           completion chain out to UsbIntrfSubmit and back into the port.
// Isr minus these two is the remainder: sweeps, EPDATASTATUS, DMA start.
uint32_t g_CollectMin = 0xFFFFFFFFU;
uint32_t g_CollectMax;
uint32_t g_CollectSum;
uint32_t g_CollectCnt;
uint32_t g_RefillMin = 0xFFFFFFFFU;
uint32_t g_RefillMax;
uint32_t g_RefillSum;
uint32_t g_RefillCnt;

// Records elapsed cycles into one of the sets above when it leaves scope, so
// a function with several exits needs no bookkeeping at each one.
struct nRFUsbdCycleSpan {
	uint32_t At;
	uint32_t *pMin;
	uint32_t *pMax;
	uint32_t *pSum;
	uint32_t *pCnt;

	~nRFUsbdCycleSpan()
	{
		const uint32_t d = DWT->CYCCNT - At;
		if (d < *pMin) { *pMin = d; }
		if (d > *pMax) { *pMax = d; }
		*pSum += d;
		(*pCnt)++;
	}
};

#define NRFUSBD_CYCLE_SPAN(name) \
	nRFUsbdCycleSpan span = { DWT->CYCCNT, &g_##name##Min, &g_##name##Max, \
							  &g_##name##Sum, &g_##name##Cnt }; \
	(void)span

// Counting starts at reset, so the maxima include enumeration: SETUP handling,
// descriptor transfers and bus reset are long and happen once. Reset when a
// data endpoint opens, which is when the class configures, so the values
// describe the steady state only. Local to this file: the example must not
// have to link a symbol that exists only under this guard.
static void nRFUsbdTimingReset(void)
{
	// The cycle counter has to be running for any of this to mean anything.
	// Enabling it here keeps this guard self sufficient, so the library does
	// not depend on the application having started DWT. Idempotent.
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	g_DmaStartMin = 0xFFFFFFFFU;
	g_DmaStartMax = 0;
	g_DmaStartSum = 0;
	g_DmaStartCnt = 0;
	g_SvcMin = 0xFFFFFFFFU;
	g_SvcMax = 0;
	g_SvcSum = 0;
	g_SvcCnt = 0;
	g_IsrMin = 0xFFFFFFFFU;
	g_IsrMax = 0;
	g_IsrSum = 0;
	g_IsrCnt = 0;
	g_CollectMin = 0xFFFFFFFFU;
	g_CollectMax = 0;
	g_CollectSum = 0;
	g_CollectCnt = 0;
	g_RefillMin = 0xFFFFFFFFU;
	g_RefillMax = 0;
	g_RefillSum = 0;
	g_RefillCnt = 0;
}
#endif

static inline __attribute__((always_inline))
uint8_t nRFUsbdDir(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? 1U : 0U;
}

static inline __attribute__((always_inline))
nRFUsbdXfer_t *nRFUsbdGetXfer(uint8_t EpAddr)
{
	return &s_Ctrlr.Xfer[USB_ENDPADDR_NUM(EpAddr)][nRFUsbdDir(EpAddr)];
}

/**
 * Lowest set bit position. Walking a mask this way visits only the bits that
 * are set, in ascending order, instead of testing every position. CLZ is one
 * cycle on Cortex-M, and this is on the interrupt entry path.
 */
static inline __attribute__((always_inline))
uint32_t nRFUsbdLowestBit(uint32_t Mask)
{
	return 31U - (uint32_t)__CLZ(Mask & (uint32_t)(0U - Mask));
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

static uint32_t nRFUsbdDmaEndMask(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);

	return USB_ENDPADDR_IS_IN(EpAddr) ?
		(1UL << (USBD_INTEN_ENDEPIN0_Pos + epNum)) :
		(1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum));
}

static volatile uint32_t *nRFUsbdDmaEndEvent(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);

	return USB_ENDPADDR_IS_IN(EpAddr) ?
		&NRF_USBD->EVENTS_ENDEPIN[epNum] :
		&NRF_USBD->EVENTS_ENDEPOUT[epNum];
}

static void nRFUsbdEmit(const UsbCtrlrEvt_t *pEvt)
{
	if (s_Ctrlr.EvtHandler != NULL)
	{
		s_Ctrlr.EvtHandler(0, pEvt, s_Ctrlr.pContext);
	}
}

static void nRFUsbdEmitSimple(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t evt = {};
	evt.Type = Type;
	nRFUsbdEmit(&evt);
}

static void nRFUsbdEmitXfer(uint8_t EpAddr, uint16_t Length,
						 UsbCtrlrXferResult_t Result)
{
	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_XFER_CMPL;
	evt.Xfer.EpAddr = EpAddr;
	evt.Xfer.Length = Length;
	evt.Xfer.Result = Result;
	nRFUsbdEmit(&evt);
}

static void nRFUsbdDmaEndIntEnable(uint8_t EpAddr)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();

	if ((uint8_t)atomic_load(&s_DmaEpAddr) == EpAddr)
	{
		NRF_USBD->INTENSET = nRFUsbdDmaEndMask(EpAddr);
	}

	__set_PRIMASK(primask);
}

/**
 * Retire a completed data IN DMA when its completion interrupt was not needed.
 * EPDATA proves that EasyDMA finished before the host took the packet.
 */
static void nRFUsbdDmaReclaim(void)
{
	uint_fast8_t epAddr = atomic_load(&s_DmaEpAddr);

	if ((uint8_t)epAddr == NRFX_USBD_DMA_EP_NONE ||
		!USB_ENDPADDR_IS_IN((uint8_t)epAddr) ||
		USB_ENDPADDR_NUM((uint8_t)epAddr) == 0U)
	{
		return;
	}

	volatile uint32_t *pEvent = nRFUsbdDmaEndEvent((uint8_t)epAddr);
	if (*pEvent == 0U)
	{
		return;
	}

	if (!atomic_compare_exchange_strong(&s_DmaEpAddr, &epAddr,
		(uint_fast8_t)NRFX_USBD_DMA_EP_NONE))
	{
		return;
	}

	*pEvent = 0;
	__ISB();
	__DSB();

	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000000UL;
	}

	atomic_flag_clear(&s_DmaRunning);
}

static void nRFUsbdDmaRelease(void)
{
	const uint8_t epAddr = (uint8_t)atomic_load(&s_DmaEpAddr);
	if (epAddr != NRFX_USBD_DMA_EP_NONE &&
		USB_ENDPADDR_IS_IN(epAddr) && USB_ENDPADDR_NUM(epAddr) != 0U)
	{
		// Data IN enables ENDEPIN only when another endpoint is waiting.
		NRF_USBD->INTENCLR = nRFUsbdDmaEndMask(epAddr);
	}

	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000000UL;
	}

	atomic_store(&s_DmaEpAddr, NRFX_USBD_DMA_EP_NONE);
	atomic_flag_clear(&s_DmaRunning);
}


static void nRFUsbdDmaStart(volatile uint32_t *pTask, uint8_t EpAddr)
{
#ifdef USB_CTRLR_TX_TIMING
	NRFUSBD_CYCLE_SPAN(DmaStart);
#endif

	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000082UL;
	}

	// Data IN normally completes without an ENDEPIN interrupt. A competing
	// request enables it while this DMA is active so it gets an immediate
	// wakeup; the event must therefore always start clear.
	*nRFUsbdDmaEndEvent(EpAddr) = 0;
	__ISB();
	__DSB();

	atomic_store(&s_DmaEpAddr, EpAddr);
	*pTask = 1;
	__ISB();
	__DSB();
}

static void nRFUsbdDmaWait(void)
{
	for (;;)
	{
		const uint8_t epAddr = (uint8_t)atomic_load(&s_DmaEpAddr);
		if (epAddr == NRFX_USBD_DMA_EP_NONE)
		{
			return;
		}

		if (NRF_USBD->EVENTS_USBRESET != 0U)
		{
			nRFUsbdDmaRelease();
			return;
		}

		volatile uint32_t *pEvent = nRFUsbdDmaEndEvent(epAddr);
		if (*pEvent == 0U)
		{
			continue;
		}

		*pEvent = 0;
		__ISB();
		__DSB();
		nRFUsbdDmaRelease();
	}
}

static void nRFUsbdNoDmaTask(volatile uint32_t *pTask)
{
	*pTask = 1;
	__ISB();
	__DSB();
	atomic_flag_clear(&s_DmaRunning);
}

static void nRFUsbdEp0StatusNow(void)
{
	const uint8_t epAddr = s_Ctrlr.SetupDirIn ?
		USB_ENDPADDR_DIR_OUT : USB_ENDPADDR_DIR_IN;
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(epAddr);

	NRF_USBD->TASKS_EP0STATUS = 1;
	__ISB();
	__DSB();

	if (pXfer->Started && pXfer->TotalLen == 0U)
	{
		pXfer->Started = false;
		nRFUsbdEmitXfer(epAddr, 0, USB_CTRLR_XFER_SUCCESS);
	}

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
	const uint16_t TransferLen = Received < Remaining ? Received : Remaining;

	NRF_USBD->EPOUT[EpNum].PTR = (uint32_t)(uintptr_t)pXfer->pBuffer;
	NRF_USBD->EPOUT[EpNum].MAXCNT = TransferLen;
	nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTEPOUT[EpNum], EpNum);

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
	const uint16_t TransferLen = Remaining < pXfer->Mps ? Remaining : pXfer->Mps;

	NRF_USBD->EPIN[EpNum].PTR = (uint32_t)(uintptr_t)pXfer->pBuffer;
	NRF_USBD->EPIN[EpNum].MAXCNT = TransferLen;
	nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTEPIN[EpNum],
					 (uint8_t)(EpNum | USB_ENDPADDR_DIR_IN));

	return true;
}

static void nRFUsbdServicePending(void)
{
#ifdef USB_CTRLR_TX_TIMING
	NRFUSBD_CYCLE_SPAN(Svc);
#endif

	if (atomic_load(&s_HostResumePending) ||
		(atomic_load(&s_BusSuspended) && !atomic_load(&s_SuspendPending)))
	{
		return;
	}

	for (;;)
	{
		if (atomic_flag_test_and_set(&s_DmaRunning))
		{
			// If a data IN DMA has already ended, retire it immediately. If it
			// is still running, enable its completion interrupt so this deferred
			// endpoint request is processed as soon as EasyDMA becomes free.
			nRFUsbdDmaReclaim();

			if (atomic_flag_test_and_set(&s_DmaRunning))
			{
				const uint8_t epAddr =
					(uint8_t)atomic_load(&s_DmaEpAddr);
				if (epAddr != NRFX_USBD_DMA_EP_NONE &&
					USB_ENDPADDR_IS_IN(epAddr) &&
					USB_ENDPADDR_NUM(epAddr) != 0U)
				{
					nRFUsbdDmaEndIntEnable(epAddr);
				}
				return;
			}
		}

		if (atomic_exchange(&s_PendingEp0Status, false))
		{
			nRFUsbdEp0StatusNow();
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
			atomic_fetch_and(&s_PendingOut, (uint_fast16_t)~(1U << epNum));
			if (nRFUsbdStartOutDmaNow(epNum))
			{
				return;
			}

			// The request is consumed before the attempt, so a refusal here
			// would discard it. The endpoint still holds the packet and its
			// EPDATASTATUS was acknowledged when it arrived, so nothing else
			// would ever fetch it. Hand it back to the arrival flag that
			// nRFUsbdEpXfer checks, which is how an unarmed endpoint already
			// carries a waiting packet to the next submit.
			s_Ctrlr.Xfer[epNum][0].DataReceived = true;

			atomic_flag_clear(&s_DmaRunning);
			continue;
		}

		pending = (uint16_t)atomic_load(&s_PendingIn);
		epNum = nRFUsbdFirstPending(pending);
		if (epNum < NRFX_USBD_EP_COUNT)
		{
			atomic_fetch_and(&s_PendingIn, (uint_fast16_t)~(1U << epNum));
			if (nRFUsbdStartInDmaNow(epNum))
			{
				return;
			}

			// Same shape as the OUT branch above: the request is consumed
			// before the attempt. A refusal while the transfer is still
			// started leaves it with no DMA and no completion, so the sender
			// waits on a packet that will never finish. Retire it instead, so
			// the layer above gets its endpoint back.
			{
				nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum][1];
				if (pXfer->Started)
				{
					pXfer->Started = false;
					atomic_flag_clear(&s_DmaRunning);
					nRFUsbdEmitXfer((uint8_t)(epNum | USB_ENDPADDR_DIR_IN),
									 pXfer->ActualLen,
									 USB_CTRLR_XFER_FAILED);
					continue;
				}
			}

			atomic_flag_clear(&s_DmaRunning);
			continue;
		}

		atomic_flag_clear(&s_DmaRunning);

		// A request can arrive after the empty-mask reads above but before
		// s_DmaRunning is cleared. Its nested service call sees the flag set
		// and returns, so recheck after releasing it to avoid losing the wakeup.
		if (atomic_load(&s_PendingOut) != 0U ||
			atomic_load(&s_PendingIn) != 0U ||
			atomic_load(&s_PendingEp0Status) ||
			atomic_load(&s_PendingEp0RcvOut))
		{
			continue;
		}

		return;
	}
}

static inline __attribute__((always_inline))
bool nRFUsbdDeferFromInterrupt(void)
{
	const uint32_t exception = __get_IPSR();
	if (exception == 0U)
	{
		return false;
	}

	if (exception != (uint32_t)USBD_IRQn + 16U)
	{
		NVIC_SetPendingIRQ(USBD_IRQn);
	}

	return true;
}

static void nRFUsbdQueueOut(uint8_t EpNum)
{
	atomic_fetch_or(&s_PendingOut, (uint_fast16_t)(1U << EpNum));
	if (nRFUsbdDeferFromInterrupt())
	{
		return;
	}
	nRFUsbdServicePending();
}

static void nRFUsbdQueueIn(uint8_t EpNum)
{
	atomic_fetch_or(&s_PendingIn, (uint_fast16_t)(1U << EpNum));
	if (nRFUsbdDeferFromInterrupt())
	{
		return;
	}
	nRFUsbdServicePending();
}

static void nRFUsbdQueueEp0Status(void)
{
	atomic_store(&s_PendingEp0Status, true);
	if (nRFUsbdDeferFromInterrupt())
	{
		return;
	}
	nRFUsbdServicePending();
}

static void nRFUsbdQueueEp0RcvOut(void)
{
	atomic_store(&s_PendingEp0RcvOut, true);
	if (nRFUsbdDeferFromInterrupt())
	{
		return;
	}
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
	atomic_store(&s_BusSuspended, false);
	atomic_store(&s_SuspendPending, false);
	atomic_store(&s_RemoteWakePending, false);
	atomic_store(&s_HostResumePending, false);
	atomic_store(&s_MacAwake, true);
	atomic_store(&s_DmaEpAddr, NRFX_USBD_DMA_EP_NONE);
	atomic_flag_clear(&s_DmaRunning);

	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000000UL;
	}
}

static void nRFUsbdAbortEp0(void)
{
	const uint8_t dmaEpAddr = (uint8_t)atomic_load(&s_DmaEpAddr);
	if (dmaEpAddr != NRFX_USBD_DMA_EP_NONE && USB_ENDPADDR_NUM(dmaEpAddr) == 0U)
	{
		nRFUsbdDmaWait();
	}

	atomic_fetch_and(&s_PendingOut, (uint_fast16_t)~1U);
	atomic_fetch_and(&s_PendingIn, (uint_fast16_t)~1U);
	atomic_store(&s_PendingEp0Status, false);
	atomic_store(&s_PendingEp0RcvOut, false);

	for (uint8_t dir = 0; dir < 2U; dir++)
	{
		nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[0][dir];
		pXfer->pBuffer = NULL;
		pXfer->TotalLen = 0;
		pXfer->ActualLen = 0;
		pXfer->Mps = NRFX_USBD_MAX_PACKET_SIZE;
		pXfer->DataReceived = false;
		pXfer->Started = false;
	}

	NRF_USBD->EVENTS_ENDEPIN[0] = 0;
	NRF_USBD->EVENTS_ENDEPOUT[0] = 0;
	NRF_USBD->EPDATASTATUS = (1UL << 0) | (1UL << 16);
	__ISB();
	__DSB();
}

static void nRFUsbdHostResumeDetected(void);

static void nRFUsbdTryEnterLowPower(void)
{
	if (!atomic_load(&s_BusSuspended) ||
		!atomic_load(&s_SuspendPending) ||
		atomic_load(&s_RemoteWakePending) ||
		atomic_load(&s_HostResumePending) ||
		(uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE ||
		atomic_load(&s_PendingOut) != 0U ||
		atomic_load(&s_PendingIn) != 0U ||
		atomic_load(&s_PendingEp0Status) ||
		atomic_load(&s_PendingEp0RcvOut))
	{
		return;
	}

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		nRFUsbdHostResumeDetected();
		return;
	}

	if (atomic_flag_test_and_set(&s_DmaRunning))
	{
		return;
	}

	if (!atomic_load(&s_BusSuspended) ||
		!atomic_load(&s_SuspendPending) ||
		atomic_load(&s_RemoteWakePending) ||
		atomic_load(&s_HostResumePending) ||
		(uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE ||
		atomic_load(&s_PendingOut) != 0U ||
		atomic_load(&s_PendingIn) != 0U ||
		atomic_load(&s_PendingEp0Status) ||
		atomic_load(&s_PendingEp0RcvOut))
	{
		atomic_flag_clear(&s_DmaRunning);
		return;
	}

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		atomic_flag_clear(&s_DmaRunning);
		nRFUsbdHostResumeDetected();
		return;
	}

	atomic_store(&s_MacAwake, false);
	NRF_USBD->LOWPOWER =
		USBD_LOWPOWER_LOWPOWER_LowPower << USBD_LOWPOWER_LOWPOWER_Pos;
	(void)NRF_USBD->LOWPOWER;
	__ISB();
	__DSB();

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		atomic_flag_clear(&s_DmaRunning);
		nRFUsbdHostResumeDetected();
		return;
	}

	if (atomic_load(&s_RemoteWakePending) || !atomic_load(&s_SuspendPending))
	{
		atomic_store(&s_SuspendPending, false);
		NRF_USBD->LOWPOWER =
			USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos;
		__ISB();
		__DSB();
		atomic_flag_clear(&s_DmaRunning);
		return;
	}

	atomic_store(&s_SuspendPending, false);
	atomic_flag_clear(&s_DmaRunning);
}

static void nRFUsbdTryRemoteWake(void)
{
	if (!atomic_load(&s_RemoteWakePending) ||
		!atomic_load(&s_BusSuspended) ||
		atomic_load(&s_HostResumePending) ||
		!atomic_load(&s_MacAwake) ||
		NRF_USBD->LOWPOWER !=
			(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
	{
		return;
	}

	if (atomic_flag_test_and_set(&s_DmaRunning))
	{
		return;
	}

	if (!atomic_load(&s_RemoteWakePending) ||
		!atomic_load(&s_BusSuspended) ||
		atomic_load(&s_HostResumePending) ||
		!atomic_load(&s_MacAwake) ||
		(uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE)
	{
		atomic_flag_clear(&s_DmaRunning);
		return;
	}

	atomic_store(&s_RemoteWakePending, false);
	NRF_USBD->DPDMVALUE = USBD_DPDMVALUE_STATE_Resume;
	NRF_USBD->TASKS_DPDMDRIVE = 1;
	__ISB();
	__DSB();
	atomic_flag_clear(&s_DmaRunning);

	if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0U)
	{
		NRF_USBD->EVENTS_SOF = 0;
	}
	NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
}

static void nRFUsbdHostResumeDetected(void)
{
	if (!atomic_load(&s_BusSuspended))
	{
		return;
	}

	atomic_store(&s_BusSuspended, false);
	atomic_store(&s_SuspendPending, false);
	atomic_store(&s_RemoteWakePending, false);

	if (!atomic_load(&s_MacAwake) ||
		NRF_USBD->LOWPOWER !=
			(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
	{
		atomic_store(&s_HostResumePending, true);
		if (NRF_USBD->LOWPOWER !=
			(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
		{
			NRF_USBD->LOWPOWER =
				USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos;
			__ISB();
			__DSB();
		}
		return;
	}

	atomic_store(&s_HostResumePending, false);
	nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
}

static void nRFUsbdWakeAllowed(void)
{
	atomic_store(&s_MacAwake, true);

	if (atomic_exchange(&s_HostResumePending, false))
	{
		nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
		return;
	}

	nRFUsbdTryRemoteWake();
}

static bool nRFUsbRegInit(UsbCtrlrEvtHandler_t EvtHandler, void *pContext)
{
	s_Ctrlr.EvtHandler = EvtHandler;
	s_Ctrlr.pContext = pContext;
	nRFUsbdResetState();
	return true;
}

static void nRFUsbRegStop(void)
{
	nRFUsbdDmaWait();
	nRFUsbdResetState();
}

static void nRFUsbRegIntEnable(void)
{
	NVIC_EnableIRQ(USBD_IRQn);
}

static void nRFUsbRegIntDisable(void)
{
	NVIC_DisableIRQ(USBD_IRQn);
}

static void nRFUsbRegConnect(void)
{
	NRF_USBD->USBPULLUP = 1;
}

static void nRFUsbRegDisconnect(void)
{
	NRF_USBD->USBPULLUP = 0;
}

static void nRFUsbRegRemoteWakeup(void)
{
	if (!atomic_load(&s_BusSuspended) || atomic_load(&s_HostResumePending))
	{
		return;
	}

	atomic_store(&s_SuspendPending, false);
	atomic_store(&s_RemoteWakePending, true);

	if (NRF_USBD->LOWPOWER !=
		(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
	{
		NRF_USBD->LOWPOWER =
			USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos;
		__ISB();
		__DSB();
	}

	nRFUsbdTryRemoteWake();
}

static void nRFUsbRegSofEnable(bool Enable)
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

static void nRFUsbRegSetAddress(uint8_t Address)
{
	(void)Address;
}

static bool nRFUsbRegEpOpen(const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == NULL)
	{
		return false;
	}

	const uint8_t epAddr = pDesc->bEndpointAddress;
	const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;

	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT ||
		(type != USB_ENDPATT_TRANS_BULK && type != USB_ENDPATT_TRANS_INT) ||
		pDesc->wMaxPacketSize == 0 ||
		pDesc->wMaxPacketSize > NRFX_USBD_MAX_PACKET_SIZE)
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(epAddr);
	pXfer->Mps = pDesc->wMaxPacketSize;
	pXfer->Started = false;
	pXfer->DataReceived = false;

#ifdef USB_CTRLR_TX_TIMING
	if (epNum > 0U)
	{
		nRFUsbdTimingReset();
	}
#endif

	if (USB_ENDPADDR_IS_IN(epAddr))
	{
		NRF_USBD->EVENTS_ENDEPIN[epNum] = 0;
		// Avoid a second interrupt for every IN packet. If another endpoint
		// requests the shared EasyDMA while this one owns it, the scheduler
		// enables ENDEPIN until that DMA completes.
		NRF_USBD->EPINEN |= (1UL << epNum);
	}
	else
	{
		NRF_USBD->EVENTS_ENDEPOUT[epNum] = 0;
		NRF_USBD->INTENSET = (1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum));
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

static void nRFUsbRegEpClose(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT)
	{
		return;
	}

	if ((uint8_t)atomic_load(&s_DmaEpAddr) == EpAddr)
	{
		nRFUsbdDmaWait();
	}

	const uint_fast16_t bit = (uint_fast16_t)(1U << epNum);
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		atomic_fetch_and(&s_PendingIn, ~bit);
		NRF_USBD->INTENCLR = (1UL << (USBD_INTEN_ENDEPIN0_Pos + epNum));
		NRF_USBD->EPINEN &= ~(1UL << epNum);
		NRF_USBD->EVENTS_ENDEPIN[epNum] = 0;
		NRF_USBD->EPDATASTATUS = (1UL << epNum);
	}
	else
	{
		atomic_fetch_and(&s_PendingOut, ~bit);
		NRF_USBD->INTENCLR = (1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum));
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

static void nRFUsbRegEpCloseAll(void)
{
	nRFUsbdDmaWait();

	for (uint8_t epNum = 1; epNum < NRFX_USBD_EP_COUNT; epNum++)
	{
		nRFUsbRegEpClose(epNum);
		nRFUsbRegEpClose((uint8_t)(epNum | USB_ENDPADDR_DIR_IN));
	}

	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;
}

static bool nRFUsbRegEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);

	if (epNum >= NRFX_USBD_EP_COUNT ||
		(TotalBytes > 0 && pBuffer == NULL) ||
		atomic_load(&s_BusSuspended))
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);
	if (pXfer->Started || pXfer->Mps == 0)
	{
		return false;
	}

	if (USB_ENDPADDR_IS_IN(EpAddr) && epNum > 0 &&
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
		USB_ENDPADDR_IS_IN(EpAddr) != s_Ctrlr.SetupDirIn;

	if (controlStatus)
	{
		nRFUsbdQueueEp0Status();
	}
	else if (USB_ENDPADDR_IS_IN(EpAddr))
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

static bool nRFUsbRegEpBusy(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	return epNum < NRFX_USBD_EP_COUNT && nRFUsbdGetXfer(EpAddr)->Started;
}

static void nRFUsbRegEpStall(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
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

static void nRFUsbRegEpClearStall(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT)
	{
		return;
	}

	NRF_USBD->DTOGGLE = EpAddr;
	NRF_USBD->DTOGGLE =
		(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | EpAddr;
	NRF_USBD->EPSTALL =
		(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | EpAddr;

	if (!USB_ENDPADDR_IS_IN(EpAddr))
	{
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}
	__ISB();
	__DSB();
}

static uint32_t nRFUsbdCollectEvents(void)
{
#ifdef USB_CTRLR_TX_TIMING
	NRFUSBD_CYCLE_SPAN(Collect);
#endif

	// One IRQ line and no aggregate pending register, so the enabled events
	// have to be read to find which fired. Only the bits set in INTEN can be
	// pending, so walk those; the rest are provably zero and testing them
	// costs on every entry.
	uint32_t enabled = NRF_USBD->INTEN &
		(uint32_t)((1ULL << NRFX_USBD_IRQ_EVENT_COUNT) - 1ULL);
	uint32_t intStatus = 0;
	volatile uint32_t *pEvent = &NRF_USBD->EVENTS_USBRESET;

	while (enabled != 0U)
	{
		const uint32_t index = nRFUsbdLowestBit(enabled);
		enabled &= enabled - 1U;

		if (pEvent[index] == 0U)
		{
			continue;
		}

		intStatus |= (1UL << index);
		pEvent[index] = 0;
		__ISB();
		__DSB();
	}

	return intStatus;
}

static void nRFUsbdBusReset(void)
{
	if ((uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE)
	{
		nRFUsbdDmaRelease();
	}

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
	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_SETUP;
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
		UsbCtrlrEvt_t addrEvt = {};
		addrEvt.Type = USB_CTRLR_EVT_ADDRESS;
		addrEvt.Address = (uint8_t)(evt.Setup.wValue & 0x7FU);
		nRFUsbdEmit(&addrEvt);
		return;
	}

	nRFUsbdEmit(&evt);
}

static void nRFUsbdHandleOutEnd(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][0];
	if (!pXfer->Started)
	{
		return;
	}

	const uint16_t transferLen = (uint16_t)NRF_USBD->EPOUT[EpNum].AMOUNT;
	if (pXfer->pBuffer != NULL)
	{
		pXfer->pBuffer += transferLen;
	}
	pXfer->ActualLen += transferLen;

	if (transferLen == pXfer->Mps && pXfer->ActualLen < pXfer->TotalLen)
	{
		if (EpNum == 0)
		{
			nRFUsbdQueueEp0RcvOut();
		}
	}
	else
	{
		pXfer->Started = false;
		nRFUsbdEmitXfer(EpNum, pXfer->ActualLen, USB_CTRLR_XFER_SUCCESS);
	}
}

static void nRFUsbdHandleOutData(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][0];
	if (pXfer->Started &&
		(pXfer->ActualLen < pXfer->TotalLen || pXfer->TotalLen == 0))
	{
		pXfer->DataReceived = false;
		nRFUsbdQueueOut(EpNum);
	}
	else
	{
		pXfer->DataReceived = true;
	}
}

static void nRFUsbdHandleInData(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][1];
	if (!pXfer->Started)
	{
		return;
	}

	const uint16_t transferLen = (uint16_t)NRF_USBD->EPIN[EpNum].AMOUNT;
	if (pXfer->pBuffer != NULL)
	{
		pXfer->pBuffer += transferLen;
	}
	pXfer->ActualLen += transferLen;

	if (pXfer->ActualLen < pXfer->TotalLen)
	{
		if (EpNum > 0U)
		{
			nRFUsbdDmaReclaim();
		}

		nRFUsbdQueueIn(EpNum);
	}
	else
	{
		pXfer->Started = false;

		if (EpNum > 0U)
		{
			nRFUsbdDmaReclaim();
		}

		{
#ifdef USB_CTRLR_TX_TIMING
			NRFUSBD_CYCLE_SPAN(Refill);
#endif
			nRFUsbdEmitXfer((uint8_t)(EpNum | USB_ENDPADDR_DIR_IN),
							 pXfer->ActualLen, USB_CTRLR_XFER_SUCCESS);
		}
	}
}

extern "C" void USBD_IRQHandler(void)
{
#ifdef USB_CTRLR_TX_TIMING
	NRFUSBD_CYCLE_SPAN(Isr);
#endif

	const uint32_t intStatus = nRFUsbdCollectEvents();
	if (intStatus == 0)
	{
		// A transfer requested from another interrupt raises a software USBD
		// interrupt so EasyDMA still starts from the controller context.
		nRFUsbdServicePending();
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

	const uint8_t dmaEpAddr = (uint8_t)atomic_load(&s_DmaEpAddr);
	if (dmaEpAddr != NRFX_USBD_DMA_EP_NONE &&
		(intStatus & nRFUsbdDmaEndMask(dmaEpAddr)) != 0U)
	{
		nRFUsbdDmaRelease();
	}

	if ((intStatus & USBD_INTEN_USBRESET_Msk) != 0)
	{
		nRFUsbdBusReset();
		nRFUsbdEmitSimple(USB_CTRLR_EVT_RESET);
		return;
	}

	if ((intStatus & USBD_INTEN_USBEVENT_Msk) != 0)
	{
		if ((eventCause & USBD_EVENTCAUSE_SUSPEND_Msk) != 0 &&
			!atomic_exchange(&s_BusSuspended, true))
		{
			atomic_store(&s_SuspendPending, true);
			atomic_store(&s_RemoteWakePending, false);
			atomic_store(&s_HostResumePending, false);
			if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0U)
			{
				NRF_USBD->EVENTS_SOF = 0;
			}
			NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
			nRFUsbdEmitSimple(USB_CTRLR_EVT_SUSPEND);
		}

		if ((eventCause & USBD_EVENTCAUSE_RESUME_Msk) != 0)
		{
			nRFUsbdHostResumeDetected();
		}

		if ((eventCause & USBD_EVENTCAUSE_USBWUALLOWED_Msk) != 0)
		{
			nRFUsbdWakeAllowed();
		}
	}

	// Endpoint zero is handled further down with the setup sequence.
	uint32_t outEnd = (intStatus >> USBD_INTEN_ENDEPOUT0_Pos) &
					  (uint32_t)(((1UL << NRFX_USBD_EP_COUNT) - 1UL) & ~1UL);

	while (outEnd != 0U)
	{
		const uint32_t epNum = nRFUsbdLowestBit(outEnd);
		outEnd &= outEnd - 1U;
		nRFUsbdHandleOutEnd((uint8_t)epNum);
	}

	uint32_t dataStatus = 0;
	if ((intStatus & (USBD_INTEN_EPDATA_Msk | USBD_INTEN_EP0DATADONE_Msk)) != 0)
	{
		dataStatus = NRF_USBD->EPDATASTATUS;
		NRF_USBD->EPDATASTATUS = dataStatus;
		__ISB();
		__DSB();

		const uint32_t epMask =
			(uint32_t)(((1UL << NRFX_USBD_EP_COUNT) - 1UL) & ~1UL);
		uint32_t outData = (dataStatus >> 16U) & epMask;
		uint32_t inData = dataStatus & epMask;

		while (outData != 0U)
		{
			const uint32_t epNum = nRFUsbdLowestBit(outData);
			outData &= outData - 1U;
			nRFUsbdHandleOutData((uint8_t)epNum);
		}

		while (inData != 0U)
		{
			const uint32_t epNum = nRFUsbdLowestBit(inData);
			inData &= inData - 1U;
			nRFUsbdHandleInData((uint8_t)epNum);
		}
	}

	const bool setupPending = (intStatus & USBD_INTEN_EP0SETUP_Msk) != 0;
	if (setupPending)
	{
		nRFUsbdHostResumeDetected();
		nRFUsbdAbortEp0();
		nRFUsbdSetupEvent();
	}
	else
	{
		if ((intStatus & USBD_INTEN_ENDEPOUT0_Msk) != 0)
		{
			nRFUsbdHandleOutEnd(0);
		}

		if ((intStatus & USBD_INTEN_EP0DATADONE_Msk) != 0)
		{
			if (s_Ctrlr.SetupDirIn)
			{
				nRFUsbdHandleInData(0);
			}
			else
			{
				nRFUsbdHandleOutData(0);
			}
		}
	}

	if ((intStatus & USBD_INTEN_SOF_Msk) != 0)
	{
		nRFUsbdHostResumeDetected();

		UsbCtrlrEvt_t evt = {};
		evt.Type = USB_CTRLR_EVT_SOF;
		evt.FrameNo = (uint16_t)NRF_USBD->FRAMECNTR;
		nRFUsbdEmit(&evt);

		if (!s_Ctrlr.SofEnabled)
		{
			NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
		}
	}

	nRFUsbdTryRemoteWake();
	nRFUsbdTryEnterLowPower();

	// Starting EasyDMA is the last USBD operation in this interrupt. Endpoint
	// callbacks only queued requests, so no handler below the start can touch
	// controller registers while the shared DMA engine owns them.
	nRFUsbdServicePending();
}

/**
 * USBD has no second register stage. The power stage leaves the peripheral
 * enabled and endpoint zero is prepared when the bus reset arrives, so there
 * is nothing to do between the two. It was a weak default in usbd_ctrlr.cpp
 * before, which hid the fact that this backend never implemented it.
 */
static bool nRFUsbRegStart(void)
{
	return true;
}

/** USBD is full speed only, USB_HIGHSPEED_CAPABLE(0) is 0 for these parts. */
static bool nRFUsbRegHighSpeed(void)
{
	return false;
}


#elif defined(USBHS_PRESENT)

#ifndef USBHS_PRESENT
#error "usbd_ctrlr_nrf54: this part has no USBHS peripheral"
#endif

#ifndef USBHSCORE_PRESENT
#error "usbd_ctrlr_nrf54: this part has no USBHS core"
#endif

#define NRF54_USBD_EP_COUNT				16U
#define NRF54_USBD_EP0_MPS				64U
#define NRF54_USBD_SETUP_COUNT			3U
#define NRF54_USBD_SETUP_SIZE			8U
#define NRF54_USBD_WAIT_LOOPS			1000000UL

#if defined(NRF_TRUSTZONE_NONSECURE)
#define NRF54_USBD_CORE					NRF_USBHSCORE_NS
#else
#define NRF54_USBD_CORE					NRF_USBHSCORE_S
#endif

#define NRF54_USBD_REG(ofs) \
	(*(volatile uint32_t *)((uintptr_t)NRF54_USBD_CORE + (ofs)))

#define NRF54_USBD_GAHBCFG				NRF54_USBD_REG(0x008U)
#define NRF54_USBD_GUSBCFG				NRF54_USBD_REG(0x00CU)
#define NRF54_USBD_GRSTCTL				NRF54_USBD_REG(0x010U)
#define NRF54_USBD_GINTSTS				NRF54_USBD_REG(0x014U)
#define NRF54_USBD_GINTMSK				NRF54_USBD_REG(0x018U)
#define NRF54_USBD_GRXFSIZ				NRF54_USBD_REG(0x024U)
#define NRF54_USBD_GNPTXFSIZ			NRF54_USBD_REG(0x028U)
#define NRF54_USBD_GHWCFG2				NRF54_USBD_REG(0x048U)
#define NRF54_USBD_GHWCFG3				NRF54_USBD_REG(0x04CU)
#define NRF54_USBD_GDFIFOCFG			NRF54_USBD_REG(0x05CU)

#define NRF54_USBD_DCFG					NRF54_USBD_REG(0x800U)
#define NRF54_USBD_DCTL					NRF54_USBD_REG(0x804U)
#define NRF54_USBD_DSTS					NRF54_USBD_REG(0x808U)
#define NRF54_USBD_DIEPMSK				NRF54_USBD_REG(0x810U)
#define NRF54_USBD_DOEPMSK				NRF54_USBD_REG(0x814U)
#define NRF54_USBD_DAINT				NRF54_USBD_REG(0x818U)
#define NRF54_USBD_DAINTMSK				NRF54_USBD_REG(0x81CU)

#define NRF54_USBD_DIEPCTL(ep)			NRF54_USBD_REG(0x900U + ((ep) * 0x20U))
#define NRF54_USBD_DIEPINT(ep)			NRF54_USBD_REG(0x908U + ((ep) * 0x20U))
#define NRF54_USBD_DIEPTSIZ(ep)			NRF54_USBD_REG(0x910U + ((ep) * 0x20U))
#define NRF54_USBD_DIEPDMA(ep)			NRF54_USBD_REG(0x914U + ((ep) * 0x20U))

#define NRF54_USBD_DOEPCTL(ep)			NRF54_USBD_REG(0xB00U + ((ep) * 0x20U))
#define NRF54_USBD_DOEPINT(ep)			NRF54_USBD_REG(0xB08U + ((ep) * 0x20U))
#define NRF54_USBD_DOEPTSIZ(ep)			NRF54_USBD_REG(0xB10U + ((ep) * 0x20U))
#define NRF54_USBD_DOEPDMA(ep)			NRF54_USBD_REG(0xB14U + ((ep) * 0x20U))

#define NRF54_USBD_DIEPTXF(ep)			NRF54_USBD_REG(0x104U + (((ep) - 1U) * 4U))

#define NRF54_USBD_GAHBCFG_GINT			(1UL << 0)
#define NRF54_USBD_GAHBCFG_HBSTLEN_INCR4	(1UL << 1)
#define NRF54_USBD_GAHBCFG_DMAEN			(1UL << 5)

#define NRF54_USBD_GUSBCFG_FORCEDEVMODE	(1UL << 30)
#define NRF54_USBD_GUSBCFG_FORCEHSTMODE	(1UL << 29)

#define NRF54_USBD_GRSTCTL_CSFTRST		(1UL << 0)
#define NRF54_USBD_GRSTCTL_RXFFLSH		(1UL << 4)
#define NRF54_USBD_GRSTCTL_TXFFLSH		(1UL << 5)
#define NRF54_USBD_GRSTCTL_TXFNUM_Pos		6U
#define NRF54_USBD_GRSTCTL_AHBIDLE		(1UL << 31)
#define NRF54_USBD_GRSTCTL_TXFIFO_ALL		0x10UL

#define NRF54_USBD_GINTSTS_SOF			(1UL << 3)
#define NRF54_USBD_GINTSTS_GOUTNAKEFF		(1UL << 7)
#define NRF54_USBD_GINTSTS_USBSUSP		(1UL << 11)
#define NRF54_USBD_GINTSTS_USBRST		(1UL << 12)
#define NRF54_USBD_GINTSTS_ENUMDONE		(1UL << 13)
#define NRF54_USBD_GINTSTS_IEPINT		(1UL << 18)
#define NRF54_USBD_GINTSTS_OEPINT		(1UL << 19)
#define NRF54_USBD_GINTSTS_WKUPINT		(1UL << 31)

#define NRF54_USBD_GHWCFG2_ARCH_Pos		3U
#define NRF54_USBD_GHWCFG2_ARCH_Msk		(3UL << NRF54_USBD_GHWCFG2_ARCH_Pos)
#define NRF54_USBD_GHWCFG2_ARCH_INTDMA	(2UL << NRF54_USBD_GHWCFG2_ARCH_Pos)
#define NRF54_USBD_GHWCFG3_DFIFO_Pos		16U

#define NRF54_USBD_DCFG_DEVSPD_Msk		3UL
#define NRF54_USBD_DCFG_DEVSPD_HS		0UL
#define NRF54_USBD_DCFG_DEVADDR_Pos		4U
#define NRF54_USBD_DCFG_DEVADDR_Msk		(0x7FUL << NRF54_USBD_DCFG_DEVADDR_Pos)

#define NRF54_USBD_DCTL_RMTWKUPSIG		(1UL << 0)
#define NRF54_USBD_DCTL_SFTDISCON		(1UL << 1)
#define NRF54_USBD_DCTL_SGOUTNAK			(1UL << 9)
#define NRF54_USBD_DCTL_CGOUTNAK			(1UL << 10)

#define NRF54_USBD_DSTS_ENUMSPD_Pos		1U
#define NRF54_USBD_DSTS_ENUMSPD_Msk		(3UL << NRF54_USBD_DSTS_ENUMSPD_Pos)
#define NRF54_USBD_DSTS_ENUMSPD_HS		(0UL << NRF54_USBD_DSTS_ENUMSPD_Pos)
#define NRF54_USBD_DSTS_FNSOF_Pos		8U
#define NRF54_USBD_DSTS_FNSOF_Msk		(0x3FFFUL << NRF54_USBD_DSTS_FNSOF_Pos)

#define NRF54_USBD_DAINT_IN(ep)			(1UL << (ep))
#define NRF54_USBD_DAINT_OUT(ep)			(1UL << (16U + (ep)))

#define NRF54_USBD_DEPCTL_MPS_Msk		0x7FFUL
#define NRF54_USBD_DEPCTL_USBACTEP		(1UL << 15)
#define NRF54_USBD_DEPCTL_EPTYPE_Pos		18U
#define NRF54_USBD_DEPCTL_EPTYPE_Msk		(3UL << NRF54_USBD_DEPCTL_EPTYPE_Pos)
#define NRF54_USBD_DEPCTL_STALL			(1UL << 21)
#define NRF54_USBD_DIEPCTL_TXFNUM_Pos		22U
#define NRF54_USBD_DIEPCTL_TXFNUM_Msk		(0xFUL << NRF54_USBD_DIEPCTL_TXFNUM_Pos)
#define NRF54_USBD_DEPCTL_CNAK			(1UL << 26)
#define NRF54_USBD_DEPCTL_SNAK			(1UL << 27)
#define NRF54_USBD_DEPCTL_SETD0PID		(1UL << 28)
#define NRF54_USBD_DEPCTL_EPDIS			(1UL << 30)
#define NRF54_USBD_DEPCTL_EPENA			(1UL << 31)

#define NRF54_USBD_EP0_MPS_Msk			3UL
#define NRF54_USBD_EP0_MPS_64			0UL

#define NRF54_USBD_DIEPINT_XFRC			(1UL << 0)
#define NRF54_USBD_DIEPINT_EPDISBLD		(1UL << 1)
#define NRF54_USBD_DIEPINT_INEPNAKEFF		(1UL << 6)
#define NRF54_USBD_DOEPINT_XFRC			(1UL << 0)
#define NRF54_USBD_DOEPINT_EPDISBLD		(1UL << 1)
#define NRF54_USBD_DOEPINT_SETUP			(1UL << 3)

#define NRF54_USBD_DIEPTSIZ0_XFERSIZE_Msk	0x7FUL
#define NRF54_USBD_DIEPTSIZ0_PKTCNT_Pos	19U
#define NRF54_USBD_DOEPTSIZ0_XFERSIZE_Msk	0x7FUL
#define NRF54_USBD_DOEPTSIZ0_PKTCNT_Pos	19U
#define NRF54_USBD_DOEPTSIZ0_SUPCNT_Pos	29U

#define NRF54_USBD_DEPTSIZ_XFERSIZE_Msk	0x7FFFFUL
#define NRF54_USBD_DEPTSIZ_PKTCNT_Pos		19U

#define NRF54_USBD_DOEPMSK_XFRC			(1UL << 0)
#define NRF54_USBD_DOEPMSK_SETUP			(1UL << 3)
#define NRF54_USBD_DIEPMSK_XFRC			(1UL << 0)

typedef struct __nRF54_Usbd_Xfer
{
	uint8_t *pBuffer;
	uint16_t TotalLen;
	uint16_t ActualLen;
	uint16_t Mps;
	uint16_t ChunkLen;
	bool Started;
} nRF54UsbdXfer_t;

typedef struct __nRF54_Usbd_Ctrlr
{
	nRF54UsbdXfer_t Xfer[NRF54_USBD_EP_COUNT][2];
	UsbCtrlrEvtHandler_t EvtHandler;
	void *pContext;
	uint16_t TxFifoWords[NRF54_USBD_EP_COUNT];
	uint16_t FifoTop;
	uint16_t RxWords;
	bool Started;
	bool HighSpeed;
	bool SofEnabled;
	bool AddressPending;
	uint8_t PendingAddress;
} nRF54UsbdCtrlr_t;

static nRF54UsbdCtrlr_t s_Ctrlr;

static uint32_t s_SetupBuffer[(NRF54_USBD_SETUP_COUNT * NRF54_USBD_SETUP_SIZE) /
								 sizeof(uint32_t)] __attribute__((aligned(4)));
static uint32_t s_Ep0Bounce[NRF54_USBD_EP0_MPS / sizeof(uint32_t)]
	__attribute__((aligned(4)));

static inline __attribute__((always_inline))
uint8_t nRF54UsbdDir(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? 1U : 0U;
}

static inline __attribute__((always_inline))
nRF54UsbdXfer_t *nRF54UsbdGetXfer(uint8_t EpAddr)
{
	return &s_Ctrlr.Xfer[USB_ENDPADDR_NUM(EpAddr)][nRF54UsbdDir(EpAddr)];
}

static bool nRF54UsbdWaitSet(volatile uint32_t *pReg, uint32_t Mask)
{
	uint32_t count = NRF54_USBD_WAIT_LOOPS;

	while ((*pReg & Mask) == 0U)
	{
		if (--count == 0U)
		{
			return false;
		}
	}

	return true;
}

static bool nRF54UsbdWaitClear(volatile uint32_t *pReg, uint32_t Mask)
{
	uint32_t count = NRF54_USBD_WAIT_LOOPS;

	while ((*pReg & Mask) != 0U)
	{
		if (--count == 0U)
		{
			return false;
		}
	}

	return true;
}

static bool nRF54UsbdFlushTx(uint8_t FifoNo)
{
	NRF54_USBD_GRSTCTL = NRF54_USBD_GRSTCTL_TXFFLSH |
		((uint32_t)FifoNo << NRF54_USBD_GRSTCTL_TXFNUM_Pos);
	return nRF54UsbdWaitClear(&NRF54_USBD_GRSTCTL,
						  NRF54_USBD_GRSTCTL_TXFFLSH);
}

static bool nRF54UsbdFlushRx(void)
{
	NRF54_USBD_GRSTCTL = NRF54_USBD_GRSTCTL_RXFFLSH;
	return nRF54UsbdWaitClear(&NRF54_USBD_GRSTCTL,
						  NRF54_USBD_GRSTCTL_RXFFLSH);
}

static void nRF54UsbdEmit(const UsbCtrlrEvt_t *pEvt)
{
	if (s_Ctrlr.EvtHandler != NULL)
	{
		s_Ctrlr.EvtHandler(0, pEvt, s_Ctrlr.pContext);
	}
}

static void nRF54UsbdEmitSimple(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t evt = { .Type = Type };
	nRF54UsbdEmit(&evt);
}

static void nRF54UsbdEmitXfer(uint8_t EpAddr, uint16_t Length,
							  UsbCtrlrXferResult_t Result)
{
	UsbCtrlrEvt_t evt = {
		.Type = USB_CTRLR_EVT_XFER_CMPL,
		.Xfer = {
			.EpAddr = EpAddr,
			.Length = Length,
			.Result = Result,
		},
	};

	nRF54UsbdEmit(&evt);
}

static uint16_t nRF54UsbdRxWords(uint16_t Mps)
{
	uint16_t packetWords = (uint16_t)((Mps + 3U) / 4U);

	// DWC2 device FIFO recommendation for buffer DMA:
	// 13 setup/control words + global NAK + two largest OUT packets +
	// two status words for each OUT endpoint.
	return (uint16_t)(14U + (2U * (packetWords + 1U)) +
					  (2U * NRF54_USBD_EP_COUNT));
}

static bool nRF54UsbdCoreReset(void)
{
	if (!nRF54UsbdWaitSet(&NRF54_USBD_GRSTCTL,
						 NRF54_USBD_GRSTCTL_AHBIDLE))
	{
		return false;
	}

	NRF54_USBD_GRSTCTL |= NRF54_USBD_GRSTCTL_CSFTRST;

	return nRF54UsbdWaitClear(&NRF54_USBD_GRSTCTL,
						   NRF54_USBD_GRSTCTL_CSFTRST);
}

static void nRF54UsbdResetSoftware(void)
{
	UsbCtrlrEvtHandler_t handler = s_Ctrlr.EvtHandler;
	void *context = s_Ctrlr.pContext;

	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));
	s_Ctrlr.EvtHandler = handler;
	s_Ctrlr.pContext = context;
	s_Ctrlr.Xfer[0][0].Mps = NRF54_USBD_EP0_MPS;
	s_Ctrlr.Xfer[0][1].Mps = NRF54_USBD_EP0_MPS;
}

static void nRF54UsbdPrimeSetup(void)
{
	// A DMA setup queue that is still enabled can already receive the next
	// SETUP packet. Do not rewrite DOEPTSIZ/DOEPDMA under an active endpoint.
	if ((NRF54_USBD_DOEPCTL(0) & NRF54_USBD_DEPCTL_EPENA) != 0U)
	{
		return;
	}

	memset(s_SetupBuffer, 0, sizeof(s_SetupBuffer));

	NRF54_USBD_DOEPTSIZ(0) =
		(NRF54_USBD_SETUP_COUNT << NRF54_USBD_DOEPTSIZ0_SUPCNT_Pos);
	NRF54_USBD_DOEPDMA(0) = (uint32_t)(uintptr_t)s_SetupBuffer;

	// Do not clear STALL here. A new SETUP automatically recovers EP0 from a
	// stalled control request, and the stall must remain asserted long enough
	// for the host to observe it.
	uint32_t ctl = NRF54_USBD_DOEPCTL(0);
	ctl &= ~NRF54_USBD_EP0_MPS_Msk;
	ctl |= NRF54_USBD_EP0_MPS_64 |
		   NRF54_USBD_DEPCTL_USBACTEP |
		   NRF54_USBD_DEPCTL_EPENA;
	NRF54_USBD_DOEPCTL(0) = ctl;
}

static void nRF54UsbdPrepareEp0(void)
{
	uint32_t inCtl = NRF54_USBD_DIEPCTL(0);
	inCtl &= ~(NRF54_USBD_EP0_MPS_Msk | NRF54_USBD_DEPCTL_STALL);
	inCtl |= NRF54_USBD_EP0_MPS_64 | NRF54_USBD_DEPCTL_USBACTEP;
	NRF54_USBD_DIEPCTL(0) = inCtl;

	uint32_t outCtl = NRF54_USBD_DOEPCTL(0);
	outCtl &= ~(NRF54_USBD_EP0_MPS_Msk | NRF54_USBD_DEPCTL_STALL);
	outCtl |= NRF54_USBD_EP0_MPS_64 | NRF54_USBD_DEPCTL_USBACTEP;
	NRF54_USBD_DOEPCTL(0) = outCtl;

	NRF54_USBD_DAINTMSK =
		NRF54_USBD_DAINT_IN(0) | NRF54_USBD_DAINT_OUT(0);
	NRF54_USBD_DIEPMSK = NRF54_USBD_DIEPMSK_XFRC;
	NRF54_USBD_DOEPMSK =
		NRF54_USBD_DOEPMSK_XFRC | NRF54_USBD_DOEPMSK_SETUP;

	nRF54UsbdPrimeSetup();
}

static bool nRF54UsbdAllocateTxFifo(uint8_t EpNum, uint16_t Mps)
{
	if (EpNum >= NRF54_USBD_EP_COUNT)
	{
		return false;
	}

	uint16_t words = (uint16_t)((Mps + 3U) / 4U);
	if (words == 0U)
	{
		words = 1U;
	}

	if (s_Ctrlr.TxFifoWords[EpNum] != 0U)
	{
		return s_Ctrlr.TxFifoWords[EpNum] >= words;
	}

	if (s_Ctrlr.FifoTop < (uint16_t)(s_Ctrlr.RxWords + words))
	{
		return false;
	}

	s_Ctrlr.FifoTop = (uint16_t)(s_Ctrlr.FifoTop - words);
	s_Ctrlr.TxFifoWords[EpNum] = words;

	uint32_t value = ((uint32_t)words << 16) | s_Ctrlr.FifoTop;
	if (EpNum == 0U)
	{
		NRF54_USBD_GNPTXFSIZ = value;
	}
	else
	{
		NRF54_USBD_DIEPTXF(EpNum) = value;
	}

	return true;
}

static bool nRF54UsbdGrowRxFifo(uint16_t Mps)
{
	uint16_t words = nRF54UsbdRxWords(Mps);

	if (words <= s_Ctrlr.RxWords)
	{
		return true;
	}

	if (words > s_Ctrlr.FifoTop)
	{
		return false;
	}

	s_Ctrlr.RxWords = words;
	NRF54_USBD_GRXFSIZ = words;
	return true;
}

static uint32_t nRF54UsbdEpType(uint8_t Type)
{
	switch (Type)
	{
		case USB_ENDPATT_TRANS_BULK:
			return 2UL;

		case USB_ENDPATT_TRANS_INT:
			return 3UL;

		default:
			return 0UL;
	}
}

static bool nRF54UsbdDisableEndpoint(uint8_t EpAddr, bool Stall)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum >= NRF54_USBD_EP_COUNT)
	{
		return false;
	}

	const uint32_t stall = Stall ? NRF54_USBD_DEPCTL_STALL : 0U;

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		uint32_t ctl = NRF54_USBD_DIEPCTL(epNum);

		if ((ctl & NRF54_USBD_DEPCTL_EPENA) != 0U)
		{
			NRF54_USBD_DIEPCTL(epNum) = ctl | NRF54_USBD_DEPCTL_SNAK;
			if (!nRF54UsbdWaitSet(&NRF54_USBD_DIEPINT(epNum),
								 NRF54_USBD_DIEPINT_INEPNAKEFF))
			{
				return false;
			}

			NRF54_USBD_DIEPCTL(epNum) |=
				NRF54_USBD_DEPCTL_EPDIS | stall;
			if (!nRF54UsbdWaitSet(&NRF54_USBD_DIEPINT(epNum),
								 NRF54_USBD_DIEPINT_EPDISBLD))
			{
				return false;
			}

			NRF54_USBD_DIEPINT(epNum) =
				NRF54_USBD_DIEPINT_EPDISBLD |
				NRF54_USBD_DIEPINT_INEPNAKEFF;
		}
		else
		{
			NRF54_USBD_DIEPCTL(epNum) =
				ctl | NRF54_USBD_DEPCTL_SNAK | stall;
		}

		if (epNum != 0U && !nRF54UsbdFlushTx(epNum))
		{
			return false;
		}

		if (!Stall && epNum != 0U)
		{
			NRF54_USBD_DIEPCTL(epNum) &= ~NRF54_USBD_DEPCTL_USBACTEP;
		}

		return true;
	}

	uint32_t ctl = NRF54_USBD_DOEPCTL(epNum);
	if (epNum == 0U)
	{
		if (Stall)
		{
			NRF54_USBD_DOEPCTL(0) = ctl | NRF54_USBD_DEPCTL_STALL;
		}
		return true;
	}

	if ((ctl & NRF54_USBD_DEPCTL_EPENA) != 0U)
	{
		NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SGOUTNAK;
		if (!nRF54UsbdWaitSet(&NRF54_USBD_GINTSTS,
							 NRF54_USBD_GINTSTS_GOUTNAKEFF))
		{
			NRF54_USBD_DCTL |= NRF54_USBD_DCTL_CGOUTNAK;
			return false;
		}

		NRF54_USBD_DOEPCTL(epNum) |= NRF54_USBD_DEPCTL_EPDIS | stall;
		if (!nRF54UsbdWaitSet(&NRF54_USBD_DOEPINT(epNum),
							 NRF54_USBD_DOEPINT_EPDISBLD))
		{
			NRF54_USBD_DCTL |= NRF54_USBD_DCTL_CGOUTNAK;
			return false;
		}

		NRF54_USBD_DOEPINT(epNum) = NRF54_USBD_DOEPINT_EPDISBLD;
		NRF54_USBD_DCTL |= NRF54_USBD_DCTL_CGOUTNAK;
	}
	else
	{
		NRF54_USBD_DOEPCTL(epNum) =
			ctl | NRF54_USBD_DEPCTL_SNAK | stall;
	}

	if (!Stall)
	{
		NRF54_USBD_DOEPCTL(epNum) &= ~NRF54_USBD_DEPCTL_USBACTEP;
	}

	return true;
}

static bool nRF54UsbdStartEp0Chunk(uint8_t EpAddr)
{
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	const uint16_t remaining =
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);
	const uint16_t chunk = remaining < NRF54_USBD_EP0_MPS ?
		remaining : NRF54_USBD_EP0_MPS;

	pXfer->ChunkLen = chunk;

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		if (chunk > 0U)
		{
			memcpy(s_Ep0Bounce, pXfer->pBuffer + pXfer->ActualLen, chunk);
		}

		NRF54_USBD_DIEPDMA(0) = (uint32_t)(uintptr_t)s_Ep0Bounce;
		NRF54_USBD_DIEPTSIZ(0) =
			chunk | (1UL << NRF54_USBD_DIEPTSIZ0_PKTCNT_Pos);
		NRF54_USBD_DIEPCTL(0) |=
			NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}
	else
	{
		NRF54_USBD_DOEPDMA(0) = (uint32_t)(uintptr_t)s_Ep0Bounce;
		NRF54_USBD_DOEPTSIZ(0) =
			chunk | (1UL << NRF54_USBD_DOEPTSIZ0_PKTCNT_Pos);
		NRF54_USBD_DOEPCTL(0) |=
			NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}

	return true;
}

static void nRF54UsbdCompleteEp0(uint8_t EpAddr)
{
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	if (!pXfer->Started)
	{
		return;
	}

	const uint16_t chunk = pXfer->ChunkLen;

	if (!USB_ENDPADDR_IS_IN(EpAddr) && chunk > 0U)
	{
		const uint16_t left =
			(uint16_t)(NRF54_USBD_DOEPTSIZ(0) &
						   NRF54_USBD_DOEPTSIZ0_XFERSIZE_Msk);
		const uint16_t received = left < chunk ?
			(uint16_t)(chunk - left) : 0U;

		if (received > 0U)
		{
			memcpy(pXfer->pBuffer + pXfer->ActualLen,
				   s_Ep0Bounce, received);
		}
		pXfer->ActualLen = (uint16_t)(pXfer->ActualLen + received);

		if (received < chunk)
		{
			pXfer->Started = false;
			nRF54UsbdEmitXfer(EpAddr, pXfer->ActualLen,
							 USB_CTRLR_XFER_SUCCESS);
			return;
		}
	}
	else
	{
		pXfer->ActualLen = (uint16_t)(pXfer->ActualLen + chunk);
	}

	if (pXfer->ActualLen < pXfer->TotalLen)
	{
		(void)nRF54UsbdStartEp0Chunk(EpAddr);
		return;
	}

	const uint16_t total = pXfer->ActualLen;
	const bool zeroLength = pXfer->TotalLen == 0U;
	pXfer->Started = false;

	if (USB_ENDPADDR_IS_IN(EpAddr) && zeroLength && s_Ctrlr.AddressPending)
	{
		uint32_t dcfg = NRF54_USBD_DCFG;
		dcfg &= ~NRF54_USBD_DCFG_DEVADDR_Msk;
		dcfg |= (uint32_t)s_Ctrlr.PendingAddress <<
				NRF54_USBD_DCFG_DEVADDR_Pos;
		NRF54_USBD_DCFG = dcfg;
		s_Ctrlr.AddressPending = false;
	}

	if (zeroLength)
	{
		nRF54UsbdPrimeSetup();
	}

	nRF54UsbdEmitXfer(EpAddr, total, USB_CTRLR_XFER_SUCCESS);
}

static void nRF54UsbdCompleteData(uint8_t EpAddr)
{
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);

	if (!pXfer->Started)
	{
		return;
	}

	uint16_t remaining;
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		remaining = (uint16_t)(NRF54_USBD_DIEPTSIZ(epNum) &
							  NRF54_USBD_DEPTSIZ_XFERSIZE_Msk);
	}
	else
	{
		remaining = (uint16_t)(NRF54_USBD_DOEPTSIZ(epNum) &
							  NRF54_USBD_DEPTSIZ_XFERSIZE_Msk);
	}

	const uint16_t actual = remaining <= pXfer->TotalLen ?
		(uint16_t)(pXfer->TotalLen - remaining) : 0U;

	pXfer->ActualLen = actual;
	pXfer->Started = false;
	nRF54UsbdEmitXfer(EpAddr, actual, USB_CTRLR_XFER_SUCCESS);
}

static void nRF54UsbdBusReset(void)
{
	s_Ctrlr.HighSpeed = false;
	s_Ctrlr.AddressPending = false;
	s_Ctrlr.PendingAddress = 0U;

	uint32_t dcfg = NRF54_USBD_DCFG;
	dcfg &= ~NRF54_USBD_DCFG_DEVADDR_Msk;
	NRF54_USBD_DCFG = dcfg;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		s_Ctrlr.Xfer[epNum][0].Started = false;
		s_Ctrlr.Xfer[epNum][1].Started = false;
		s_Ctrlr.Xfer[epNum][0].ActualLen = 0U;
		s_Ctrlr.Xfer[epNum][1].ActualLen = 0U;

		NRF54_USBD_DIEPINT(epNum) = 0xFFFFFFFFUL;
		NRF54_USBD_DOEPINT(epNum) = 0xFFFFFFFFUL;

		NRF54_USBD_DOEPCTL(epNum) |= NRF54_USBD_DEPCTL_SNAK;

		if ((NRF54_USBD_DIEPCTL(epNum) & NRF54_USBD_DEPCTL_EPENA) != 0U)
		{
			NRF54_USBD_DIEPCTL(epNum) |=
				NRF54_USBD_DEPCTL_SNAK | NRF54_USBD_DEPCTL_EPDIS;
		}
	}

	(void)nRF54UsbdFlushTx((uint8_t)NRF54_USBD_GRSTCTL_TXFIFO_ALL);
	(void)nRF54UsbdFlushRx();

	nRF54UsbdPrepareEp0();
	nRF54UsbdEmitSimple(USB_CTRLR_EVT_RESET);
}

static void nRF54UsbdSetupEvent(void)
{
	// A new SETUP cancels the previous EP0 transaction.
	s_Ctrlr.Xfer[0][0].Started = false;
	s_Ctrlr.Xfer[0][1].Started = false;
	s_Ctrlr.AddressPending = false;

	uintptr_t setupAddr =
		(uintptr_t)NRF54_USBD_DOEPDMA(0) - NRF54_USBD_SETUP_SIZE;
	const uintptr_t first = (uintptr_t)s_SetupBuffer;
	const uintptr_t last = first + sizeof(s_SetupBuffer) -
						   NRF54_USBD_SETUP_SIZE;

	if (setupAddr < first || setupAddr > last)
	{
		setupAddr = first;
	}

	UsbCtrlrEvt_t evt = { .Type = USB_CTRLR_EVT_SETUP };
	memcpy(&evt.Setup, (const void *)setupAddr, sizeof(evt.Setup));
	nRF54UsbdEmit(&evt);
}

static void nRF54UsbdInInterrupt(void)
{
	const uint32_t pending = NRF54_USBD_DAINT &
							 NRF54_USBD_DAINTMSK & 0xFFFFUL;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		if ((pending & NRF54_USBD_DAINT_IN(epNum)) == 0U)
		{
			continue;
		}

		const uint32_t epInt = NRF54_USBD_DIEPINT(epNum) &
							   NRF54_USBD_DIEPMSK;
		NRF54_USBD_DIEPINT(epNum) = epInt;

		if ((epInt & NRF54_USBD_DIEPINT_XFRC) != 0U)
		{
			if (epNum == 0U)
			{
				nRF54UsbdCompleteEp0(USB_ENDPADDR_DIR_IN);
			}
			else
			{
				nRF54UsbdCompleteData((uint8_t)(epNum | USB_ENDPADDR_DIR_IN));
			}
		}
	}
}

static void nRF54UsbdOutInterrupt(void)
{
	const uint32_t pending = (NRF54_USBD_DAINT &
							  NRF54_USBD_DAINTMSK) >> 16;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		if ((pending & (1UL << epNum)) == 0U)
		{
			continue;
		}

		const uint32_t epInt = NRF54_USBD_DOEPINT(epNum) &
							   NRF54_USBD_DOEPMSK;
		NRF54_USBD_DOEPINT(epNum) = epInt;

		if (epNum == 0U && (epInt & NRF54_USBD_DOEPINT_SETUP) != 0U)
		{
			nRF54UsbdSetupEvent();
			continue;
		}

		if ((epInt & NRF54_USBD_DOEPINT_XFRC) != 0U)
		{
			if (epNum == 0U)
			{
				if (s_Ctrlr.Xfer[0][0].Started)
				{
					nRF54UsbdCompleteEp0(USB_ENDPADDR_DIR_OUT);
				}
			}
			else
			{
				nRF54UsbdCompleteData(epNum);
			}
		}
	}
}

static bool nRFUsbRegInit(UsbCtrlrEvtHandler_t EvtHandler, void *pContext)
{
	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));
	s_Ctrlr.EvtHandler = EvtHandler;
	s_Ctrlr.pContext = pContext;
	s_Ctrlr.Xfer[0][0].Mps = NRF54_USBD_EP0_MPS;
	s_Ctrlr.Xfer[0][1].Mps = NRF54_USBD_EP0_MPS;
	return true;
}

static bool nRFUsbRegStart(void)
{
	if ((NRF54_USBD_GHWCFG2 & NRF54_USBD_GHWCFG2_ARCH_Msk) !=
		NRF54_USBD_GHWCFG2_ARCH_INTDMA)
	{
		return false;
	}

	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SFTDISCON;

	if (!nRF54UsbdCoreReset())
	{
		return false;
	}

	nRF54UsbdResetSoftware();

	uint32_t gusbcfg = NRF54_USBD_GUSBCFG;
	gusbcfg &= ~NRF54_USBD_GUSBCFG_FORCEHSTMODE;
	gusbcfg |= NRF54_USBD_GUSBCFG_FORCEDEVMODE;
	NRF54_USBD_GUSBCFG = gusbcfg;

	const uint16_t dfifoDepth =
		(uint16_t)(NRF54_USBD_GHWCFG3 >> NRF54_USBD_GHWCFG3_DFIFO_Pos);
	const uint16_t epInfoWords = 2U * NRF54_USBD_EP_COUNT;
	const uint32_t totalFifo = (uint32_t)dfifoDepth + epInfoWords;

	if (totalFifo > 0xFFFFUL || dfifoDepth <= epInfoWords)
	{
		return false;
	}

	// Buffer DMA reserves one endpoint-info word per endpoint direction. The
	// nRF54 GHWCFG3 depth is the 3040-word data FIFO; GDFIFOCFG then places the
	// endpoint-info controller immediately above it in the 3072-word RAM.
	s_Ctrlr.FifoTop = dfifoDepth;
	NRF54_USBD_GDFIFOCFG =
		((uint32_t)dfifoDepth << 16) | dfifoDepth;

	s_Ctrlr.RxWords = nRF54UsbdRxWords(NRF54_USBD_EP0_MPS);
	NRF54_USBD_GRXFSIZ = s_Ctrlr.RxWords;

	if (!nRF54UsbdAllocateTxFifo(0U, NRF54_USBD_EP0_MPS))
	{
		return false;
	}

	uint32_t gahbcfg = NRF54_USBD_GAHBCFG;
	gahbcfg &= ~(0xFUL << 1);
	gahbcfg |= NRF54_USBD_GAHBCFG_HBSTLEN_INCR4 |
			   NRF54_USBD_GAHBCFG_DMAEN;
	gahbcfg &= ~NRF54_USBD_GAHBCFG_GINT;
	NRF54_USBD_GAHBCFG = gahbcfg;

	uint32_t dcfg = NRF54_USBD_DCFG;
	dcfg &= ~(NRF54_USBD_DCFG_DEVSPD_Msk |
			  NRF54_USBD_DCFG_DEVADDR_Msk);
	dcfg |= NRF54_USBD_DCFG_DEVSPD_HS;
	NRF54_USBD_DCFG = dcfg;

	NRF54_USBD_GINTMSK = 0U;
	NRF54_USBD_GINTSTS = 0xFFFFFFFFUL;
	NRF54_USBD_DAINTMSK = 0U;
	NRF54_USBD_DIEPMSK = 0U;
	NRF54_USBD_DOEPMSK = 0U;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		NRF54_USBD_DIEPINT(epNum) = 0xFFFFFFFFUL;
		NRF54_USBD_DOEPINT(epNum) = 0xFFFFFFFFUL;
	}

	nRF54UsbdPrepareEp0();

	NRF54_USBD_GINTMSK =
		NRF54_USBD_GINTSTS_USBRST |
		NRF54_USBD_GINTSTS_ENUMDONE |
		NRF54_USBD_GINTSTS_IEPINT |
		NRF54_USBD_GINTSTS_OEPINT |
		NRF54_USBD_GINTSTS_USBSUSP |
		NRF54_USBD_GINTSTS_WKUPINT;

	s_Ctrlr.Started = true;
	return true;
}

static void nRFUsbRegStop(void)
{
	if (!s_Ctrlr.Started)
	{
		return;
	}

	NRF54_USBD_GAHBCFG &= ~NRF54_USBD_GAHBCFG_GINT;
	NRF54_USBD_GINTMSK = 0U;
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SFTDISCON;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		s_Ctrlr.Xfer[epNum][0].Started = false;
		s_Ctrlr.Xfer[epNum][1].Started = false;
	}

	s_Ctrlr.Started = false;
	s_Ctrlr.HighSpeed = false;
	s_Ctrlr.AddressPending = false;
}

static bool nRFUsbRegHighSpeed(void)
{
	return s_Ctrlr.Started && s_Ctrlr.HighSpeed;
}

static void nRFUsbRegIntEnable(void)
{
	NRF54_USBD_GAHBCFG |= NRF54_USBD_GAHBCFG_GINT;
	NVIC_ClearPendingIRQ(USBHS_IRQn);
	NVIC_EnableIRQ(USBHS_IRQn);
}

static void nRFUsbRegIntDisable(void)
{
	NVIC_DisableIRQ(USBHS_IRQn);
	NRF54_USBD_GAHBCFG &= ~NRF54_USBD_GAHBCFG_GINT;
}

static void nRFUsbRegConnect(void)
{
	NRF54_USBD_DCTL &= ~NRF54_USBD_DCTL_SFTDISCON;
}

static void nRFUsbRegDisconnect(void)
{
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SFTDISCON;
}

static void nRFUsbRegRemoteWakeup(void)
{
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_RMTWKUPSIG;
	nrfx_coredep_delay_us(2000U);
	NRF54_USBD_DCTL &= ~NRF54_USBD_DCTL_RMTWKUPSIG;
}

static void nRFUsbRegSofEnable(bool Enable)
{
	s_Ctrlr.SofEnabled = Enable;

	if (Enable)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_SOF;
		NRF54_USBD_GINTMSK |= NRF54_USBD_GINTSTS_SOF;
	}
	else
	{
		NRF54_USBD_GINTMSK &= ~NRF54_USBD_GINTSTS_SOF;
	}
}

static void nRFUsbRegSetAddress(uint8_t Address)
{
	s_Ctrlr.PendingAddress = (uint8_t)(Address & 0x7FU);
	s_Ctrlr.AddressPending = true;
}

static bool nRFUsbRegEpOpen(const UsbEndPointDesc_t *pDesc)
{
	if (!s_Ctrlr.Started || pDesc == NULL)
	{
		return false;
	}

	const uint8_t epAddr = pDesc->bEndpointAddress;
	const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;
	const uint16_t mps = pDesc->wMaxPacketSize;

	if (epNum == 0U || epNum >= NRF54_USBD_EP_COUNT ||
		(type != USB_ENDPATT_TRANS_BULK &&
		 type != USB_ENDPATT_TRANS_INT) ||
		mps == 0U)
	{
		return false;
	}

	const uint16_t maxMps = type == USB_ENDPATT_TRANS_BULK ?
		(s_Ctrlr.HighSpeed ? 512U : 64U) :
		(s_Ctrlr.HighSpeed ? 1024U : 64U);
	if (mps > maxMps)
	{
		return false;
	}

	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(epAddr);
	pXfer->Mps = mps;
	pXfer->Started = false;
	pXfer->ActualLen = 0U;

	const uint32_t epType =
		nRF54UsbdEpType(type) << NRF54_USBD_DEPCTL_EPTYPE_Pos;

	if (USB_ENDPADDR_IS_IN(epAddr))
	{
		if (!nRF54UsbdAllocateTxFifo(epNum, mps))
		{
			return false;
		}

		uint32_t ctl = NRF54_USBD_DIEPCTL(epNum);
		ctl &= ~(NRF54_USBD_DEPCTL_MPS_Msk |
				 NRF54_USBD_DEPCTL_EPTYPE_Msk |
				 NRF54_USBD_DIEPCTL_TXFNUM_Msk |
				 NRF54_USBD_DEPCTL_STALL);
		ctl |= mps |
			   epType |
			   NRF54_USBD_DEPCTL_USBACTEP |
			   NRF54_USBD_DEPCTL_SETD0PID |
			   ((uint32_t)epNum << NRF54_USBD_DIEPCTL_TXFNUM_Pos);
		NRF54_USBD_DIEPCTL(epNum) = ctl;
		NRF54_USBD_DAINTMSK |= NRF54_USBD_DAINT_IN(epNum);
	}
	else
	{
		if (!nRF54UsbdGrowRxFifo(mps))
		{
			return false;
		}

		uint32_t ctl = NRF54_USBD_DOEPCTL(epNum);
		ctl &= ~(NRF54_USBD_DEPCTL_MPS_Msk |
				 NRF54_USBD_DEPCTL_EPTYPE_Msk |
				 NRF54_USBD_DEPCTL_STALL);
		ctl |= mps |
			   epType |
			   NRF54_USBD_DEPCTL_USBACTEP |
			   NRF54_USBD_DEPCTL_SETD0PID;
		NRF54_USBD_DOEPCTL(epNum) = ctl;
		NRF54_USBD_DAINTMSK |= NRF54_USBD_DAINT_OUT(epNum);
	}

	return true;
}

static void nRFUsbRegEpClose(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0U || epNum >= NRF54_USBD_EP_COUNT)
	{
		return;
	}

	(void)nRF54UsbdDisableEndpoint(EpAddr, false);

	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	pXfer->Started = false;
	pXfer->ActualLen = 0U;
	pXfer->TotalLen = 0U;
	pXfer->ChunkLen = 0U;
	pXfer->pBuffer = NULL;

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		NRF54_USBD_DAINTMSK &= ~NRF54_USBD_DAINT_IN(epNum);
	}
	else
	{
		NRF54_USBD_DAINTMSK &= ~NRF54_USBD_DAINT_OUT(epNum);
	}
}

static void nRFUsbRegEpCloseAll(void)
{
	for (uint8_t epNum = 1U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		nRFUsbRegEpClose(epNum);
		nRFUsbRegEpClose((uint8_t)(epNum | USB_ENDPADDR_DIR_IN));
	}

	(void)nRF54UsbdFlushRx();
}

static bool nRFUsbRegEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (!s_Ctrlr.Started || epNum >= NRF54_USBD_EP_COUNT ||
		(TotalBytes > 0U && pBuffer == NULL))
	{
		return false;
	}

	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	if (pXfer->Started || pXfer->Mps == 0U)
	{
		return false;
	}

	if (epNum != 0U && TotalBytes > 0U &&
		(((uintptr_t)pBuffer & 0x3U) != 0U))
	{
		// nRF54LM20 has instruction cache only. Buffer DMA therefore needs no
		// data-cache maintenance, but the DMA address must remain word aligned.
		return false;
	}

	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = TotalBytes;
	pXfer->ActualLen = 0U;
	pXfer->ChunkLen = 0U;
	pXfer->Started = true;

	if (epNum == 0U)
	{
		return nRF54UsbdStartEp0Chunk(EpAddr);
	}

	const uint32_t packetCnt = TotalBytes == 0U ? 1U :
		((uint32_t)TotalBytes + pXfer->Mps - 1U) / pXfer->Mps;

	if (packetCnt > 0x3FFUL)
	{
		pXfer->Started = false;
		return false;
	}

	const uint32_t size =
		((uint32_t)TotalBytes & NRF54_USBD_DEPTSIZ_XFERSIZE_Msk) |
		(packetCnt << NRF54_USBD_DEPTSIZ_PKTCNT_Pos);

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		NRF54_USBD_DIEPDMA(epNum) = (uint32_t)(uintptr_t)pBuffer;
		NRF54_USBD_DIEPTSIZ(epNum) = size;
		NRF54_USBD_DIEPCTL(epNum) |=
			NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}
	else
	{
		NRF54_USBD_DOEPDMA(epNum) = (uint32_t)(uintptr_t)pBuffer;
		NRF54_USBD_DOEPTSIZ(epNum) = size;
		NRF54_USBD_DOEPCTL(epNum) |=
			NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}

	return true;
}

static bool nRFUsbRegEpBusy(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	return epNum < NRF54_USBD_EP_COUNT &&
		   nRF54UsbdGetXfer(EpAddr)->Started;
}

static void nRFUsbRegEpStall(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum >= NRF54_USBD_EP_COUNT)
	{
		return;
	}

	(void)nRF54UsbdDisableEndpoint(EpAddr, true);

	// EP0 OUT remains active for SETUP reception. If the setup queue was
	// exhausted by the failed request, re-arm it without clearing STALL.
	if (epNum == 0U && !USB_ENDPADDR_IS_IN(EpAddr))
	{
		nRF54UsbdPrimeSetup();
	}
}

static void nRFUsbRegEpClearStall(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum >= NRF54_USBD_EP_COUNT)
	{
		return;
	}

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		uint32_t ctl = NRF54_USBD_DIEPCTL(epNum);
		ctl &= ~NRF54_USBD_DEPCTL_STALL;
		if (epNum != 0U)
		{
			ctl |= NRF54_USBD_DEPCTL_SETD0PID;
		}
		NRF54_USBD_DIEPCTL(epNum) = ctl;
	}
	else
	{
		uint32_t ctl = NRF54_USBD_DOEPCTL(epNum);
		ctl &= ~NRF54_USBD_DEPCTL_STALL;
		if (epNum != 0U)
		{
			ctl |= NRF54_USBD_DEPCTL_SETD0PID;
		}
		NRF54_USBD_DOEPCTL(epNum) = ctl;
	}
}

void USBHS_IRQHandler(void)
{
	const uint32_t status = NRF54_USBD_GINTSTS & NRF54_USBD_GINTMSK;
	if (status == 0U)
	{
		return;
	}

	if ((status & NRF54_USBD_GINTSTS_USBRST) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_USBRST;
		nRF54UsbdBusReset();
	}

	if ((status & NRF54_USBD_GINTSTS_ENUMDONE) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_ENUMDONE;
		s_Ctrlr.HighSpeed =
			(NRF54_USBD_DSTS & NRF54_USBD_DSTS_ENUMSPD_Msk) ==
			NRF54_USBD_DSTS_ENUMSPD_HS;
	}

	if ((status & NRF54_USBD_GINTSTS_OEPINT) != 0U)
	{
		nRF54UsbdOutInterrupt();
	}

	if ((status & NRF54_USBD_GINTSTS_IEPINT) != 0U)
	{
		nRF54UsbdInInterrupt();
	}

	if ((status & NRF54_USBD_GINTSTS_USBSUSP) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_USBSUSP;
		nRF54UsbdEmitSimple(USB_CTRLR_EVT_SUSPEND);
	}

	if ((status & NRF54_USBD_GINTSTS_WKUPINT) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_WKUPINT;
		nRF54UsbdEmitSimple(USB_CTRLR_EVT_RESUME);
	}

	if ((status & NRF54_USBD_GINTSTS_SOF) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_SOF;

		if (s_Ctrlr.SofEnabled)
		{
			UsbCtrlrEvt_t evt = {
				.Type = USB_CTRLR_EVT_SOF,
				.FrameNo = (uint16_t)((NRF54_USBD_DSTS &
									  NRF54_USBD_DSTS_FNSOF_Msk) >>
									  NRF54_USBD_DSTS_FNSOF_Pos),
			};
			nRF54UsbdEmit(&evt);
		}
	}
}

#endif

//
// Entry points declared in usb.h. Each validates DevNo and then runs the
// power stage and the register stage in the order the hardware needs.
//

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (!nRFUsbValidDevNo(DevNo) || pCfg == NULL)
	{
		return false;
	}

	// Power first. The register stage must not touch a peripheral that has no
	// clock, which is why nRFUsbRegInit only sets software state.
	if (!nRFUsbPowerInit(pCfg))
	{
		return false;
	}

	return nRFUsbRegInit(pCfg->EvtHandler, pCfg->pContext);
}

bool UsbCtrlrStart(int DevNo)
{
	if (!nRFUsbValidDevNo(DevNo))
	{
		return false;
	}

	// One call where usbd.h and usbd_ctrlr.h used to need two. Power, clock
	// and PHY come up, then endpoint zero is prepared.
	if (!nRFUsbPowerStart())
	{
		return false;
	}

	if (!nRFUsbRegStart())
	{
		nRFUsbPowerStop();
		return false;
	}

	return true;
}

void UsbCtrlrStop(int DevNo)
{
	if (!nRFUsbValidDevNo(DevNo))
	{
		return;
	}

	nRFUsbRegStop();
	nRFUsbPowerStop();
}

void UsbCtrlrProcess(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbPowerProcess();
	}
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	return nRFUsbValidDevNo(DevNo) && nRFUsbVbusDetected();
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	return nRFUsbValidDevNo(DevNo) && nRFUsbRegHighSpeed();
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	return nRFUsbValidDevNo(DevNo) ? nRFUsbSerial(pBuff, BuffLen) : 0;
}

void UsbCtrlrIntEnable(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegIntEnable(); }
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegIntDisable(); }
}

void UsbCtrlrConnect(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegConnect(); }
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegDisconnect(); }
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegRemoteWakeup(); }
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegSofEnable(Enable); }
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegSetAddress(Address); }
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	return nRFUsbValidDevNo(DevNo) && nRFUsbRegEpOpen(pDesc);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegEpClose(EpAddr); }
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegEpCloseAll(); }
}

bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
					uint16_t TotalBytes)
{
	return nRFUsbValidDevNo(DevNo) &&
		   nRFUsbRegEpXfer(EpAddr, pBuffer, TotalBytes);
}

bool UsbCtrlrEpBusy(int DevNo, uint8_t EpAddr)
{
	return nRFUsbValidDevNo(DevNo) && nRFUsbRegEpBusy(EpAddr);
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegEpStall(EpAddr); }
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo)) { nRFUsbRegEpClearStall(EpAddr); }
}
