/**-------------------------------------------------------------------------
@file	system_clock_nrfx.cpp

@brief	Clock functions of system_core_clock.h on the nRF52, nRF53, nRF54L and
		nRF91

The CPU always runs from HFCLK, sourced by the internal RC oscillator (HFINT)
or by the 32 MHz crystal (HFXO). The crystal only improves accuracy. The
radio and USB need it and it draws much more current, so it is started on
request, not at startup.

SystemCoreClockSelect configures the core oscillator descriptor. Shared
high-frequency oscillator use is controlled by SystemHighFreqClockEnable and
SystemHighFreqClockDisable. The first enable starts HFXO when configured,
and the final disable stops it without disabling the CPU clock.
SoftDevice and MPSL ownership is handled by the MCU implementation.

On the nRF53, HFCLKSRC selects the crystal before HFCLKSTART and selects the
RC again after HFCLKSTOP. On the nRF54L the crystal is started and stopped
with XOSTART and XOSTOP, its state is in XO.STAT. The PLL is not touched, the
CPU clock started by SystemInit keeps running from it.

SystemCoreClockSelect does not change g_McuOsc, it stays the description of
the board.

SystemLowFreqClockSelect validates the board-declared source and configures
LFCLK without starting it. Enable and Disable own the oscillator lifecycle.
The board's g_McuOsc declaration is never rewritten. The RC, the crystal, and on the
nRF52 a rail to rail external clock on XL1 (OSC_TYPE_TCXO) are supported.
On the nRF54L the LFCLK registers are LFCLK.SRC, LFCLK.STAT and
LFCLK.SRCCOPY.
When a BLE stack is enabled it owns LFCLK, whose source it set when it was
enabled, and the function returns false. If the crystal does not start,
LFCLK runs again from its previous source.

The peripheral clock is the fixed 16 MHz PCLK16M, peripheral clock 0.
SystemPeriphClockSet returns it, the only frequency there is.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_soc.h"
#endif

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "nrf_mpsl.h"
#include "idelay.h"

#if defined(NRF52_SERIES) && defined(NRFX_LFRC_CAL_LOG)
#include "app_evt_handler.h"
#include "syslog.h"
#endif

// CLOCK instance, as in timer_hf_nrfx.cpp
#if defined(NRF91_SERIES)
// Non secure on nRF91, as POWER, which the Modem library uses there
// (nrfx_config_nrf91.h)
#define NRF_CLOCK					NRF_CLOCK_NS
#elif defined(NRF5340_XXAA_APPLICATION)
#define NRF_CLOCK					NRF_CLOCK_S
#elif defined(NRF5340_XXAA_NETWORK)
#define NRF_CLOCK					NRF_CLOCK_NS
#endif

// About 50 ms at 64 MHz, the crystal starts in a few ms
#define NRFX_HFXO_WAIT_LOOPS		1000000

// Peripherals run from the fixed 16 MHz PCLK16M
#define NRFX_PCLK_FREQ				16000000UL

// Poll budget for MPSL's LFCLK wait; normal shared-clock start/stop uses
// the microsecond timeout below.
#define NRFX_LFCLK_WAIT_LOOPS		10000000

// LFXO on nRF5340 may need several hundred milliseconds to start.
// Bound the wait in time rather than compiler-dependent loop iterations.
#define NRFX_LFCLK_START_TIMEOUT_US	1500000U
#define NRFX_LFCLK_START_POLL_US	100U

// LFCLK registers and source values
#if defined(CLOCK_LFCLK_SRC_SRC_Msk)
#define NRFX_LFCLKSRC				LFCLK.SRC
#define NRFX_LFCLKSTAT				LFCLK.STAT
#define NRFX_LFCLKSRCCOPY			LFCLK.SRCCOPY
#define NRFX_LFCLK_SRC_Pos			CLOCK_LFCLK_SRC_SRC_Pos
#define NRFX_LFCLK_SRC_Msk			CLOCK_LFCLK_SRC_SRC_Msk
#define NRFX_LFCLK_STATE_Msk		CLOCK_LFCLK_STAT_STATE_Msk
#define NRFX_LFCLK_SRC_RC			CLOCK_LFCLK_SRC_SRC_LFRC
#define NRFX_LFCLK_SRC_XTAL			CLOCK_LFCLK_SRC_SRC_LFXO
#define NRFX_LFCLK_STAT_SRC_Msk		CLOCK_LFCLK_STAT_SRC_Msk
#else
#define NRFX_LFCLKSRC				LFCLKSRC
#define NRFX_LFCLKSTAT				LFCLKSTAT
#define NRFX_LFCLKSRCCOPY			LFCLKSRCCOPY
#define NRFX_LFCLK_SRC_Pos			CLOCK_LFCLKSRC_SRC_Pos
#define NRFX_LFCLK_SRC_Msk			CLOCK_LFCLKSRC_SRC_Msk
#define NRFX_LFCLK_STATE_Msk		CLOCK_LFCLKSTAT_STATE_Msk
#ifdef CLOCK_LFCLKSRC_SRC_Xtal
#define NRFX_LFCLK_SRC_RC			CLOCK_LFCLKSRC_SRC_RC
#define NRFX_LFCLK_SRC_XTAL			CLOCK_LFCLKSRC_SRC_Xtal
#else
#define NRFX_LFCLK_SRC_RC			CLOCK_LFCLKSRC_SRC_LFRC
#define NRFX_LFCLK_SRC_XTAL			CLOCK_LFCLKSRC_SRC_LFXO
#endif
#define NRFX_LFCLK_STAT_SRC_Msk		CLOCK_LFCLKSTAT_SRC_Msk
#endif

// Crystal start and stop tasks and started event
#if defined(CLOCK_XO_STAT_STATE_Msk)
#define NRFX_HFXO_START				TASKS_XOSTART
#define NRFX_HFXO_STOP				TASKS_XOSTOP
#define NRFX_HFXO_STARTED			EVENTS_XOSTARTED
#else
#define NRFX_HFXO_START				TASKS_HFCLKSTART
#define NRFX_HFXO_STOP				TASKS_HFCLKSTOP
#define NRFX_HFXO_STARTED			EVENTS_HFCLKSTARTED
#ifdef CLOCK_HFCLKSTAT_SRC_Xtal
#define NRFX_HFXO_STAT_SRC			(CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)
#else
#define NRFX_HFXO_STAT_SRC			(CLOCK_HFCLKSTAT_SRC_HFXO << CLOCK_HFCLKSTAT_SRC_Pos)
#endif
#endif

// Only the first user starts HFXO. A short interrupt-safe transition flag
// prevents concurrent start/stop operations without waiting in an ISR.
typedef enum {
	NRFX_HFXO_OWNER_NONE,
	NRFX_HFXO_OWNER_DIRECT,
	NRFX_HFXO_OWNER_SOFTDEVICE,
	NRFX_HFXO_OWNER_MPSL
} nRFxHfxoOwner_t;

static uint32_t s_nRFxHfxoReqCnt = 0U;
static bool s_bNRFxHfxoTransition = false;
static nRFxHfxoOwner_t s_nRFxHfxoOwner = NRFX_HFXO_OWNER_NONE;
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
// A direct request may outlive SoftDevice initialization. The last
// application user releases its logical reference immediately, but the
// physical direct STOP is deferred until SoftDevice has been disabled.
static bool s_bNRFxSdDirectStopPending;
#endif
#if defined(NRF52_SERIES)
// MPSL external clock-control calls can run at high priority. Its requests
// are counted separately from IOsonata users but share the same oscillator.
static bool s_bNRFxMpslExternalClock;
static uint32_t s_nRFxMpslHfxoReqCnt;
static uint32_t s_nRFxMpslLfclkReqCnt;
static bool s_bNRFxMpslHfxoOwned;
static bool s_bNRFxMpslLfclkOwned;
static bool s_bNRFxMpslCalibrationPending;
static bool s_bNRFxMpslCalibrationRunning;
static volatile uint32_t s_nRFxMpslCalibrationCount;
#endif

// Shared LFCLK enable count belongs here, not in the RTC driver.
static uint32_t s_nRFxLfclkReqCnt = 0U;
static bool s_bNRFxLfclkTransition = false;
static bool s_bNRFxLfclkOwned = false;
static bool s_bNRFxLfclkStopPending = false;
#if defined(NRF52_SERIES) && defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
// A direct LFCLK owner may release while the SoftDevice controls CLOCK.
// Its physical STOP is deferred until the SoftDevice has shut down.
static bool s_bNRFxSdLfclkStopPending;
#endif
#if defined(NRF5340_XXAA_NETWORK)
// XTAL remains the board-selected source. RC is an effective-clock fallback.
static bool s_bNRFxNetLfclkRcFallback;
#endif

// Temporarily selected sources, independent of the board oscillator hardware.
static bool s_bNRFxCoreOscSelected;
static bool s_bNRFxLowOscSelected;
static OSC_TYPE s_nRFxCoreOscSelected;
static OSC_TYPE s_nRFxLowOscSelected;

static inline OSC_TYPE nRFxCoreOscSelected(void)
{
	return s_bNRFxCoreOscSelected ? s_nRFxCoreOscSelected : g_McuOsc.CoreOsc.Type;
}

static inline OSC_TYPE nRFxLowOscSelected(void)
{
	return s_bNRFxLowOscSelected ? s_nRFxLowOscSelected : g_McuOsc.LowPwrOsc.Type;
}
#if defined(NRF53_SERIES) || defined(NRF52_SERIES)
static bool s_bNRFxLfclkCalibrating;
#endif

// Weak: the SoftDevice Controller is not in the build. nrf_mpsl.cpp has the
// version that goes through MPSL once it is initialized.
__WEAK int nRFxMpslHfxo(bool bOn)
{
	(void)bOn;

	return -1;
}

// Weak: the SoftDevice Controller is not in the build, MPSL never owns CLOCK
__WEAK bool nRFxMpslActive(void)
{
	return false;
}

// A compiled SoftDevice header or a signature in flash does not mean the
// SoftDevice is running. The SDK maintains this state in RAM. The weak
// reference allows bare-metal applications to link the same HAL archive
// without pulling in the SoftDevice handler or executing an SVC.
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#pragma weak nrf_sdh_is_enabled
#endif
static bool nRFxClockSdEnabled(void)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	return nrf_sdh_is_enabled != nullptr && nrf_sdh_is_enabled();
#else
	return false;
#endif
}

static bool nRFxHfxoRunning(void)
{
#if defined(CLOCK_XO_STAT_STATE_Msk)
	return (NRF_CLOCK->XO.STAT & CLOCK_XO_STAT_STATE_Msk) != 0;
#else
	return (NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_STATE_Msk | CLOCK_HFCLKSTAT_SRC_Msk)) ==
		   (CLOCK_HFCLKSTAT_STATE_Msk | NRFX_HFXO_STAT_SRC);
#endif
}

// Start the oscillator through its current owner. The caller holds the
// transition flag, but interrupts remain enabled during the start wait.
static bool nRFxHfxoStart(nRFxHfxoOwner_t *pOwner)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	uint32_t running;
#endif
	int res;
	int i;

	res = nRFxMpslHfxo(true);
	if (res >= 0)
	{
		if (res == 0)
		{
			*pOwner = NRFX_HFXO_OWNER_MPSL;
			return true;
		}
		return false;
	}

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	if (nRFxClockSdEnabled())
	{
		running = 0U;
		if (sd_clock_hfclk_request() != NRF_SUCCESS)
		{
			return false;
		}
		for (i = 0; i < NRFX_HFXO_WAIT_LOOPS; i++)
		{
			if (sd_clock_hfclk_is_running(&running) != NRF_SUCCESS)
			{
				(void)sd_clock_hfclk_release();
				return false;
			}
			if (running != 0U)
			{
				*pOwner = NRFX_HFXO_OWNER_SOFTDEVICE;
				return true;
			}
		}
		(void)sd_clock_hfclk_release();
		return false;
	}
#endif

	if (nRFxHfxoRunning())
	{
		// Another component started HFXO. Do not stop its oscillator.
		*pOwner = NRFX_HFXO_OWNER_NONE;
		return true;
	}

#ifdef CLOCK_HFCLKSRC_SRC_Msk
	NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFXO << CLOCK_HFCLKSRC_SRC_Pos;
#endif
	NRF_CLOCK->NRFX_HFXO_STARTED = 0U;
	NRF_CLOCK->NRFX_HFXO_START = 1U;

	for (i = 0; i < NRFX_HFXO_WAIT_LOOPS; i++)
	{
		if (nRFxHfxoRunning())
		{
			NRF_CLOCK->NRFX_HFXO_STARTED = 0U;
			*pOwner = NRFX_HFXO_OWNER_DIRECT;
			return true;
		}
	}

	NRF_CLOCK->NRFX_HFXO_STOP = 1U;
#ifdef CLOCK_HFCLKSRC_SRC_Msk
	NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos;
#endif
	return false;
}

// Release HFXO through the owner that accepted its first request.
static bool nRFxHfxoStop(nRFxHfxoOwner_t Owner)
{
	switch (Owner)
	{
		case NRFX_HFXO_OWNER_MPSL:
			return nRFxMpslHfxo(false) == 0;
		case NRFX_HFXO_OWNER_SOFTDEVICE:
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
			if (nRFxClockSdEnabled())
			{
				return sd_clock_hfclk_release() == NRF_SUCCESS;
			}
#endif
			return false;
		case NRFX_HFXO_OWNER_DIRECT:
			// A BLE stack may take CLOCK ownership after the acquisition.
			// Retain the request rather than report a release that did not occur.
			if (nRFxClockSdEnabled())
			{
				return false;
			}
#if defined(NRF52_SERIES)
			if (nRFxMpslActive())
			{
				if (s_bNRFxMpslExternalClock)
				{
					return false;
				}
				// LFRC uses MPSL's internal calibration and CLOCK control.
				// An MPSL request/release cannot release the direct HFXO
				// start made before MPSL initialized. Keep that request
				// recorded until a safe ownership transfer is available.
				return false;
			}
#else
			if (nRFxMpslActive())
			{
				return false;
			}
#endif
			NRF_CLOCK->NRFX_HFXO_STOP = 1U;
#ifdef CLOCK_HFCLKSRC_SRC_Msk
			NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos;
#endif
			return true;
		default:
			return true;
	}
}

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
static void nRFxClockSdhStateHandler(nrf_sdh_state_evt_t Event, void *pContext)
{
	uint32_t state;

	(void)pContext;
	if (Event != NRF_SDH_EVT_STATE_DISABLED)
	{
		return;
	}

	state = DisableInterrupt();
	if (s_bNRFxSdDirectStopPending && s_nRFxHfxoReqCnt == 0U &&
		!s_bNRFxHfxoTransition)
	{
		NRF_CLOCK->NRFX_HFXO_STOP = 1U;
#ifdef CLOCK_HFCLKSRC_SRC_Msk
		NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos;
#endif
		s_bNRFxSdDirectStopPending = false;
	}
	else if (s_bNRFxSdDirectStopPending && s_nRFxHfxoReqCnt != 0U)
	{
		// Outstanding users keep the oscillator. The released
		// SoftDevice owner is no longer the owner after shutdown.
		s_nRFxHfxoOwner = NRFX_HFXO_OWNER_DIRECT;
		s_bNRFxSdDirectStopPending = false;
	}
#if defined(NRF52_SERIES)
	if (s_bNRFxSdLfclkStopPending && !s_bNRFxLfclkTransition)
	{
		if (s_nRFxLfclkReqCnt != 0U)
		{
			// A new application user inherited the original direct clock.
			s_bNRFxLfclkOwned = true;
			s_bNRFxSdLfclkStopPending = false;
		}
		else if (!nRFxMpslActive() && s_nRFxMpslLfclkReqCnt == 0U)
		{
			NRF_CLOCK->TASKS_LFCLKSTOP = 1U;
			s_bNRFxLfclkStopPending = true;
			s_bNRFxSdLfclkStopPending = false;
		}
	}
#endif
	EnableInterrupt(state);
}

NRF_SDH_STATE_OBSERVER(s_NRfxClockSdhObserver, 0) = {
	.handler = nRFxClockSdhStateHandler,
	.p_context = nullptr,
};
#endif

// nRF5340 application core implements real running-frequency scaling in
// system_nrf5340_application.c. Other Nordic cores currently have no
// validated runtime CPU scaling: retain their configured performance clock.
#if !defined(NRF5340_XXAA_APPLICATION)
bool SystemPowerModeSet(SYSTEM_POWER_MODE Mode)
{
	return Mode == SYSTEM_POWER_MODE_PERFORMANCE;
}

SYSTEM_POWER_MODE SystemPowerModeGet(void)
{
	return SYSTEM_POWER_MODE_PERFORMANCE;
}
#endif

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t state;

	if (ClkSrc != OSC_TYPE_RC &&
		(ClkSrc != g_McuOsc.CoreOsc.Type || g_McuOsc.CoreOsc.Freq == 0U))
	{
		return false;
	}
	if (OscFreq != 0U && OscFreq != g_McuOsc.CoreOsc.Freq)
	{
		return false;
	}

	state = DisableInterrupt();
	if (s_bNRFxHfxoTransition || s_nRFxHfxoReqCnt != 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	s_nRFxCoreOscSelected = ClkSrc;
	s_bNRFxCoreOscSelected = true;
	EnableInterrupt(state);
	return true;
}

bool SystemHighFreqClockEnable(void)
{
	nRFxHfxoOwner_t owner;
	uint32_t state;
	bool success;

	state = DisableInterrupt();
	if (s_bNRFxHfxoTransition || s_nRFxHfxoReqCnt == UINT32_MAX)
	{
		EnableInterrupt(state);
		return false;
	}
	if (s_nRFxHfxoReqCnt != 0U)
	{
		s_nRFxHfxoReqCnt++;
		EnableInterrupt(state);
		return true;
	}
	if (nRFxCoreOscSelected() == OSC_TYPE_RC)
	{
		s_nRFxHfxoReqCnt = 1U;
		EnableInterrupt(state);
		return true;
	}
	if (nRFxCoreOscSelected() != OSC_TYPE_XTAL &&
		nRFxCoreOscSelected() != OSC_TYPE_TCXO)
	{
		EnableInterrupt(state);
		return false;
	}
	s_bNRFxHfxoTransition = true;
	EnableInterrupt(state);

	owner = NRFX_HFXO_OWNER_NONE;
	success = nRFxHfxoStart(&owner);
	state = DisableInterrupt();
	if (success)
	{
		s_nRFxHfxoReqCnt = 1U;
		s_nRFxHfxoOwner = owner;
	}
	s_bNRFxHfxoTransition = false;
	EnableInterrupt(state);
	return success;
}

void SystemHighFreqClockDisable(void)
{
	uint32_t state;
	nRFxHfxoOwner_t owner;
	bool success;

	state = DisableInterrupt();
	if (s_bNRFxHfxoTransition || s_nRFxHfxoReqCnt == 0U)
	{
		EnableInterrupt(state);
		return;
	}
	if (s_nRFxHfxoReqCnt > 1U)
	{
		s_nRFxHfxoReqCnt--;
		EnableInterrupt(state);
		return;
	}
	if (nRFxCoreOscSelected() == OSC_TYPE_RC)
	{
		s_nRFxHfxoReqCnt = 0U;
		EnableInterrupt(state);
		return;
	}
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	if (s_nRFxHfxoOwner == NRFX_HFXO_OWNER_DIRECT &&
		nRFxClockSdEnabled())
	{
		// BLE retains physical CLOCK ownership until its shutdown.
		// A direct STOP here would violate SoftDevice ownership.
		s_bNRFxSdDirectStopPending = true;
		s_nRFxHfxoReqCnt = 0U;
		s_nRFxHfxoOwner = NRFX_HFXO_OWNER_NONE;
		EnableInterrupt(state);
		return;
	}
#endif
#if defined(NRF52_SERIES)
	if (s_bNRFxMpslExternalClock)
	{
		// The registered MPSL driver uses our physical CLOCK controller.
		// Decide the last STOP atomically with both reference counters so
		// a high-priority MPSL request cannot race with the STOP task.
		if (s_nRFxMpslHfxoReqCnt != 0U)
		{
			if (s_nRFxHfxoOwner == NRFX_HFXO_OWNER_DIRECT)
			{
				s_bNRFxMpslHfxoOwned = true;
			}
		}
		else if (s_nRFxHfxoOwner == NRFX_HFXO_OWNER_DIRECT ||
			s_bNRFxMpslHfxoOwned)
		{
			NRF_CLOCK->NRFX_HFXO_STOP = 1U;
#ifdef CLOCK_HFCLKSRC_SRC_Msk
			NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos;
#endif
			s_bNRFxMpslHfxoOwned = false;
		}
		s_nRFxHfxoReqCnt = 0U;
		s_nRFxHfxoOwner = NRFX_HFXO_OWNER_NONE;
		EnableInterrupt(state);
		return;
	}
#endif
	s_bNRFxHfxoTransition = true;
	owner = s_nRFxHfxoOwner;
	EnableInterrupt(state);

	success = nRFxHfxoStop(owner);
	state = DisableInterrupt();
	if (success)
	{
		s_nRFxHfxoReqCnt = 0U;
		s_nRFxHfxoOwner = NRFX_HFXO_OWNER_NONE;
#if defined(NRF52_SERIES)
		if (s_bNRFxMpslExternalClock && s_nRFxMpslHfxoReqCnt == 0U)
		{
			s_bNRFxMpslHfxoOwned = false;
		}
#endif
	}
	s_bNRFxHfxoTransition = false;
	EnableInterrupt(state);
}

// System OFF requires HFXO stopped on nRF54L. Disable all active
// clock users before entering System OFF.
#if defined(NRF54L_SERIES)
bool nRFxClockPrepareSystemOff(void)
{
	uint32_t state;
	int i;

	state = DisableInterrupt();
	if (s_bNRFxHfxoTransition || s_nRFxHfxoReqCnt != 0U ||
		nRFxMpslActive() || nRFxClockSdEnabled())
	{
		EnableInterrupt(state);
		return false;
	}

	EnableInterrupt(state);

	// A running oscillator with no IOsonata request belongs to another
	// component. Never stop it here. Wait only for a pending stop to settle.
	for (i = 0; i < NRFX_HFXO_WAIT_LOOPS; i++)
	{
		if (nRFxHfxoRunning() == false)
		{
			return true;
		}
	}

	return false;
}
#endif

// Convert a supported generic oscillator choice to the Nordic LFCLK source.
static bool nRFxLfclkOscSource(OSC_TYPE Type, uint32_t *pSrc)
{
	switch (Type)
	{
		case OSC_TYPE_RC:
			*pSrc = NRFX_LFCLK_SRC_RC << NRFX_LFCLK_SRC_Pos;
			return true;
		case OSC_TYPE_XTAL:
			*pSrc = NRFX_LFCLK_SRC_XTAL << NRFX_LFCLK_SRC_Pos;
			return true;
#if defined(CLOCK_LFCLKSRC_BYPASS_Msk) && defined(CLOCK_LFCLKSRC_EXTERNAL_Msk)
		case OSC_TYPE_TCXO:
			*pSrc = (NRFX_LFCLK_SRC_XTAL << NRFX_LFCLK_SRC_Pos) |
				CLOCK_LFCLKSRC_BYPASS_Msk | CLOCK_LFCLKSRC_EXTERNAL_Msk;
			return true;
#endif
		default:
			return false;
	}
}

#if defined(NRF53_SERIES) || defined(NRF52_SERIES)
// The LFRC CAL task uses HFXO as a reference. nRF52 callers must
// request HFXO explicitly; nRF53 CLOCK owns that calibration request.
// Wait for DONE outside the interrupt mask.
#define NRFX_LFRC_CAL_TIMEOUT_US	250000U
#define NRFX_LFRC_CAL_RETRY_US		1000U

static bool nRFxLfclkCalibrate(void)
{
	uint32_t elapsed;
	uint32_t nextRetry;

	NRF_CLOCK->EVENTS_DONE = 0U;
	NRF_CLOCK->TASKS_CAL = 1U;
	elapsed = 0U;
	nextRetry = NRFX_LFRC_CAL_RETRY_US;
	while (elapsed < NRFX_LFRC_CAL_TIMEOUT_US)
	{
		if (NRF_CLOCK->EVENTS_DONE != 0U)
		{
			NRF_CLOCK->EVENTS_DONE = 0U;
			return true;
		}
		// nRF5340 revision 1 anomaly 137: a higher priority LFCLK
		// source requested by the other core can prevent DONE.
		// Re-trigger every 1 ms; CAL tasks during an active calibration
		// are ignored by hardware.
		if (elapsed >= nextRetry)
		{
			NRF_CLOCK->TASKS_CAL = 1U;
			nextRetry += NRFX_LFRC_CAL_RETRY_US;
		}
		usDelay(NRFX_LFCLK_START_POLL_US);
		elapsed += NRFX_LFCLK_START_POLL_US;
	}
	if (NRF_CLOCK->EVENTS_DONE != 0U)
	{
		NRF_CLOCK->EVENTS_DONE = 0U;
		return true;
	}
	return false;
}
#endif

bool SystemHighFreqClockCalibrate(void)
{
	// No generic hardware calibration operation for HFRC on these targets.
	// High-frequency crystal request/ownership is not RC calibration.
	return false;
}

bool SystemLowFreqClockCalibrate(void)
{
#if defined(NRF53_SERIES) || defined(NRF52_SERIES)
	uint32_t state;
	bool active;
	bool success;
#if defined(NRF52_SERIES)
	bool hfxo;
#endif

	// Calibration waits for hardware DONE. Timer ISR callbacks should
	// dispatch foreground work rather than wait here.
	if (__get_IPSR() != 0U)
	{
		return false;
	}
	state = DisableInterrupt();
	active = nRFxLowOscSelected() == OSC_TYPE_RC &&
		(NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) != 0U &&
		(NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STAT_SRC_Msk) ==
			(NRFX_LFCLK_SRC_RC << NRFX_LFCLK_SRC_Pos) &&
		s_nRFxLfclkReqCnt != 0U && !s_bNRFxLfclkTransition &&
		!s_bNRFxLfclkCalibrating &&
		!nRFxMpslActive() && !nRFxClockSdEnabled();

	if (active)
	{
		s_bNRFxLfclkCalibrating = true;
	}
	EnableInterrupt(state);

	if (!active)
	{
		return false;
	}

	if (!SystemLowFreqClockEnable())
	{
		state = DisableInterrupt();
		s_bNRFxLfclkCalibrating = false;
		EnableInterrupt(state);
		return false;
	}

	// Hold an additional shared LFCLK reference during calibration.
	// A separate user's Disable cannot stop LFCLK before DONE.
#if defined(NRF52_SERIES)
	// nRF52 CAL requires crystal-sourced HFCLK. Use the shared owner,
	// not direct CLOCK tasks, so existing radio requests remain intact.
	hfxo = SystemHighFreqClockEnable();
	if (hfxo)
	{
		success = nRFxLfclkCalibrate();
		SystemHighFreqClockDisable();
	}
	else
	{
		success = false;
	}
#else
	success = nRFxLfclkCalibrate();
#endif
	SystemLowFreqClockDisable();

	state = DisableInterrupt();
	s_bNRFxLfclkCalibrating = false;
	EnableInterrupt(state);

	return success;
#else
	// Other targets retain their existing vendor/stack calibration owner.
	return false;
#endif
}

// The clock manager alone starts and stops the shared oscillator.
// LFXO may take hundreds of milliseconds to take over from LFRC.
// Read the active source, not the selected-source register copy.
static bool nRFxLfclkFinishStart(uint32_t Src)
{
#if defined(NRF53_SERIES)
	uint32_t elapsed;

	if ((Src & NRFX_LFCLK_SRC_Msk) ==
		(NRFX_LFCLK_SRC_XTAL << NRFX_LFCLK_SRC_Pos))
	{
		elapsed = 0U;
		while (elapsed < NRFX_LFCLK_START_TIMEOUT_US)
		{
			if ((NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) != 0U &&
				(NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STAT_SRC_Msk) ==
				(NRFX_LFCLK_SRC_XTAL << NRFX_LFCLK_SRC_Pos))
			{
				return true;
			}
			usDelay(NRFX_LFCLK_START_POLL_US);
			elapsed += NRFX_LFCLK_START_POLL_US;
		}
		return false;
	}
	if ((Src & NRFX_LFCLK_SRC_Msk) ==
		(NRFX_LFCLK_SRC_RC << NRFX_LFCLK_SRC_Pos))
	{
		// A higher-priority LFXO request from the other core can supply
		// this LFCLK even though this controller requested LFRC.
		// On nRF5340, such an override prevents LFRC calibration DONE
		// (anomaly 137); calibration must not gate LFCLK availability.
		if ((NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) == 0U)
		{
			return false;
		}
		if ((NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STAT_SRC_Msk) ==
			(NRFX_LFCLK_SRC_RC << NRFX_LFCLK_SRC_Pos))
		{
			(void)nRFxLfclkCalibrate();
		}
		return true;
	}
#else
	(void)Src;
#endif
	return true;
}

static bool nRFxLfclkStart(uint32_t Src)
{
	uint32_t elapsed;

	NRF_CLOCK->NRFX_LFCLKSRC = Src;
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0U;
	NRF_CLOCK->TASKS_LFCLKSTART = 1U;
	elapsed = 0U;
	while (elapsed < NRFX_LFCLK_START_TIMEOUT_US)
	{
		if (NRF_CLOCK->EVENTS_LFCLKSTARTED != 0U)
		{
			NRF_CLOCK->EVENTS_LFCLKSTARTED = 0U;
			if (nRFxLfclkFinishStart(Src))
			{
				return true;
			}
			NRF_CLOCK->TASKS_LFCLKSTOP = 1U;
			return false;
		}
		usDelay(NRFX_LFCLK_START_POLL_US);
		elapsed += NRFX_LFCLK_START_POLL_US;
	}
	if (NRF_CLOCK->EVENTS_LFCLKSTARTED != 0U)
	{
		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0U;
		if (nRFxLfclkFinishStart(Src))
		{
			return true;
		}
	}
	NRF_CLOCK->TASKS_LFCLKSTOP = 1U;
	return false;
}

// LFCLK STOP is nonblocking. Wait before a later clock enable, not in an ISR.
static bool nRFxLfclkWaitStopped(void)
{
	uint32_t elapsed;

	elapsed = 0U;
	while (elapsed < NRFX_LFCLK_START_TIMEOUT_US)
	{
		if ((NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) == 0U)
		{
			return true;
		}
		usDelay(NRFX_LFCLK_START_POLL_US);
		elapsed += NRFX_LFCLK_START_POLL_US;
	}
	return (NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) == 0U;
}

// Snapshot actual LFCLK hardware state before sDelay requests an RTC.
// A running oscillator avoids the cold LFXO startup penalty. No ownership
// change or oscillator start takes place in this query.
extern "C" bool nRFxSleepLfClockRunning(void)
{
	return (NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) != 0U;
}

bool SystemLowFreqClockEnable(void)
{
	uint32_t src;
	uint32_t effectiveSrc;
	uint32_t state;
	bool running;
	bool stopped;
	bool started;

	if (!nRFxLfclkOscSource(nRFxLowOscSelected(), &src))
	{
		return false;
	}
	effectiveSrc = src;
#if defined(NRF5340_XXAA_NETWORK)
	if (s_bNRFxNetLfclkRcFallback)
	{
		effectiveSrc = NRFX_LFCLK_SRC_RC << NRFX_LFCLK_SRC_Pos;
	}
#endif
	state = DisableInterrupt();
	if (s_bNRFxLfclkTransition || s_nRFxLfclkReqCnt == UINT32_MAX)
	{
		EnableInterrupt(state);
		return false;
	}
	if (s_nRFxLfclkReqCnt != 0U)
	{
		if ((NRF_CLOCK->NRFX_LFCLKSRCCOPY & NRFX_LFCLK_SRC_Msk) !=
			(effectiveSrc & NRFX_LFCLK_SRC_Msk))
		{
			EnableInterrupt(state);
			return false;
		}
		s_nRFxLfclkReqCnt++;
		EnableInterrupt(state);
		return true;
	}
	running = (NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) != 0U;
	if (running && !s_bNRFxLfclkStopPending)
	{
		if ((NRF_CLOCK->NRFX_LFCLKSRCCOPY & NRFX_LFCLK_SRC_Msk) !=
			(effectiveSrc & NRFX_LFCLK_SRC_Msk))
		{
			EnableInterrupt(state);
			return false;
		}
		s_nRFxLfclkReqCnt = 1U;
		EnableInterrupt(state);
		return true;
	}
	if (__get_IPSR() != 0U || nRFxMpslActive() || nRFxClockSdEnabled())
	{
		EnableInterrupt(state);
		return false;
	}
	s_bNRFxLfclkTransition = true;
	EnableInterrupt(state);

	stopped = !s_bNRFxLfclkStopPending || nRFxLfclkWaitStopped();
	started = stopped && nRFxLfclkStart(src);
#if defined(NRF5340_XXAA_NETWORK)
	if (!started && stopped &&
		(src & NRFX_LFCLK_SRC_Msk) ==
		(NRFX_LFCLK_SRC_XTAL << NRFX_LFCLK_SRC_Pos))
	{
		// The shared crystal is not available to this network core.
		// Failed START issued STOP. Wait for completion before using LFRC.
		if (nRFxLfclkWaitStopped())
		{
			started = nRFxLfclkStart(NRFX_LFCLK_SRC_RC << NRFX_LFCLK_SRC_Pos);
			if (started)
			{
				effectiveSrc = NRFX_LFCLK_SRC_RC << NRFX_LFCLK_SRC_Pos;
			}
		}
	}
#endif
	state = DisableInterrupt();
	if (stopped)
	{
		s_bNRFxLfclkStopPending = false;
	}
	if (started)
	{
		s_nRFxLfclkReqCnt = 1U;
		s_bNRFxLfclkOwned = true;
#if defined(NRF5340_XXAA_NETWORK)
		s_bNRFxNetLfclkRcFallback =
		(effectiveSrc & NRFX_LFCLK_SRC_Msk) !=
		(src & NRFX_LFCLK_SRC_Msk);
#endif
	}
	else if (stopped)
	{
		// nRFxLfclkStart issued a STOP after a start timeout.
		// The next enable must wait for that STOP to settle.
		s_bNRFxLfclkStopPending = true;
		s_bNRFxLfclkOwned = false;
#if defined(NRF5340_XXAA_NETWORK)
		s_bNRFxNetLfclkRcFallback = false;
#endif
	}
	s_bNRFxLfclkTransition = false;
	EnableInterrupt(state);
	return started;
}

void SystemLowFreqClockDisable(void)
{
	uint32_t state;

	state = DisableInterrupt();
	if (s_nRFxLfclkReqCnt != 0U)
	{
		s_nRFxLfclkReqCnt--;
		if (s_nRFxLfclkReqCnt == 0U)
		{
#if defined(NRF52_SERIES)
			if (s_bNRFxMpslExternalClock &&
				s_nRFxMpslLfclkReqCnt != 0U && s_bNRFxLfclkOwned)
			{
				s_bNRFxMpslLfclkOwned = true;
			}
			if ((s_bNRFxLfclkOwned || s_bNRFxMpslLfclkOwned) &&
				s_nRFxMpslLfclkReqCnt == 0U &&
				(!nRFxMpslActive() || s_bNRFxMpslExternalClock) &&
#else
			if (s_bNRFxLfclkOwned && !nRFxMpslActive() &&
#endif
				!nRFxClockSdEnabled())
			{
				NRF_CLOCK->TASKS_LFCLKSTOP = 1U;
				s_bNRFxLfclkStopPending = true;
#if defined(NRF5340_XXAA_NETWORK)
				s_bNRFxNetLfclkRcFallback = false;
#endif
#if defined(NRF52_SERIES)
				s_bNRFxMpslLfclkOwned = false;
#endif
			}
			#if defined(NRF52_SERIES) && defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
			// Preserve the direct oscillator's last physical STOP until
			// SoftDevice shutdown; it must not be issued under stack ownership.
			if (s_bNRFxLfclkOwned && nRFxClockSdEnabled())
			{
				s_bNRFxSdLfclkStopPending = true;
			}
			#endif
			s_bNRFxLfclkOwned = false;
		}
	}
	EnableInterrupt(state);
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t src;
	uint32_t state;
	bool running;
	bool stopped;

	if ((ClkSrc != OSC_TYPE_RC &&
		(ClkSrc != g_McuOsc.LowPwrOsc.Type ||
		g_McuOsc.LowPwrOsc.Freq != 32768U)) ||
		(OscFreq != 0U && OscFreq != 32768U) ||
		!nRFxLfclkOscSource(ClkSrc, &src) || __get_IPSR() != 0U)
	{
		return false;
	}
	state = DisableInterrupt();
	if (s_bNRFxLfclkTransition || s_nRFxLfclkReqCnt != 0U ||
		nRFxMpslActive() || nRFxClockSdEnabled())
	{
		EnableInterrupt(state);
		return false;
	}
	running = (NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) != 0U;
	if (running && !s_bNRFxLfclkStopPending &&
		(NRF_CLOCK->NRFX_LFCLKSRCCOPY & NRFX_LFCLK_SRC_Msk) !=
		(src & NRFX_LFCLK_SRC_Msk))
	{
		EnableInterrupt(state);
		return false;
	}
	s_bNRFxLfclkTransition = true;
	EnableInterrupt(state);

	stopped = !s_bNRFxLfclkStopPending || nRFxLfclkWaitStopped();
	state = DisableInterrupt();
	if (stopped)
	{
		NRF_CLOCK->NRFX_LFCLKSRC = src;
		s_bNRFxLfclkStopPending = false;
		s_nRFxLowOscSelected = ClkSrc;
		s_bNRFxLowOscSelected = true;
	}
	s_bNRFxLfclkTransition = false;
	EnableInterrupt(state);
	return stopped;
}

uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx == 0 ? NRFX_PCLK_FREQ : 0;
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	// Fixed, the closest frequency is the one there is
	(void)Freq;

	return SystemPeriphClockGet(Idx);
}
#if defined(NRF52_SERIES)
#if defined(NRFX_LFRC_CAL_LOG)
// Report calibration in the foreground. Never format or transmit UART data
// from the POWER_CLOCK interrupt handler.
static void nRFxMpslCalibrationLog(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	SysLogPrintf(SysLogGet(), "LFRC calibration: completed=%lu\r\n",
		(unsigned long)s_nRFxMpslCalibrationCount);
}
#endif
// Start LFRC calibration once HFXO is ready. The callback and CLOCK
// interrupt both use this helper so startup never waits in an interrupt.
static void nRFxMpslCalibrationBegin(void)
{
	if (s_bNRFxMpslCalibrationPending &&
		!s_bNRFxMpslCalibrationRunning && nRFxHfxoRunning())
	{
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0U;
		NRF_CLOCK->INTENCLR = CLOCK_INTENCLR_HFCLKSTARTED_Msk;
		NRF_CLOCK->EVENTS_DONE = 0U;
		NRF_CLOCK->TASKS_CAL = 1U;
		s_bNRFxMpslCalibrationRunning = true;
	}
}

void nRFxMpslLfclkCalibrationStart(void)
{
	uint32_t state;

	state = DisableInterrupt();
	if (!s_bNRFxMpslExternalClock ||
		nRFxLowOscSelected() != OSC_TYPE_RC ||
		s_bNRFxMpslCalibrationPending ||
		(NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) == 0U)
	{
		EnableInterrupt(state);
		return;
	}
	s_bNRFxMpslCalibrationPending = true;
	// Acquire the clock before interrupts are restored. The HFCLK
	// callback only writes registers and updates the reference count.
	nRFxMpslHfclkRequest();
	NRF_CLOCK->INTENSET = CLOCK_INTENSET_DONE_Msk |
		CLOCK_INTENSET_HFCLKSTARTED_Msk;
	nRFxMpslCalibrationBegin();
	EnableInterrupt(state);
}

bool nRFxMpslLfclkCalibrationIsEnabled(void)
{
	return nRFxLowOscSelected() == OSC_TYPE_RC;
}

uint32_t nRFxMpslLfclkCalibrationCount(void)
{
	return s_nRFxMpslCalibrationCount;
}

void nRFxMpslClockIrq(void)
{
	if (!s_bNRFxMpslExternalClock)
	{
		return;
	}
	if (NRF_CLOCK->EVENTS_CTTO != 0U)
	{
		NRF_CLOCK->EVENTS_CTTO = 0U;
		nRFxMpslLfclkCalibrationStart();
	}
	if (s_bNRFxMpslCalibrationPending)
	{
		nRFxMpslCalibrationBegin();
		if (s_bNRFxMpslCalibrationRunning &&
			NRF_CLOCK->EVENTS_DONE != 0U)
		{
			NRF_CLOCK->EVENTS_DONE = 0U;
			NRF_CLOCK->INTENCLR = CLOCK_INTENCLR_DONE_Msk;
			s_bNRFxMpslCalibrationRunning = false;
			s_bNRFxMpslCalibrationPending = false;
			s_nRFxMpslCalibrationCount++;
			nRFxMpslHfclkRelease();
#if defined(NRFX_LFRC_CAL_LOG)
			(void)AppEvtHandlerQue(0U, nullptr, nRFxMpslCalibrationLog);
#endif
			// CTIV is specified in 0.25-second units on nRF52.
			// Recalibrate every four seconds; MPSL can request
			// additional calibrations after temperature changes.
			NRF_CLOCK->CTIV = 16U;
			NRF_CLOCK->EVENTS_CTTO = 0U;
			NRF_CLOCK->INTENSET = CLOCK_INTENSET_CTTO_Msk;
			NRF_CLOCK->TASKS_CTSTART = 1U;
		}
	}
}

// The external MPSL clock driver uses these short interrupt-safe callbacks.
// Do not call MPSL's internal HF source request API in this configuration.
void nRFxMpslExternalClockSet(bool Enabled)
{
	uint32_t state;

	state = DisableInterrupt();
	s_bNRFxMpslExternalClock = Enabled;
	EnableInterrupt(state);
}

bool nRFxMpslExternalClockGet(void)
{
	return s_bNRFxMpslExternalClock;
}

void nRFxMpslHfclkRequest(void)
{
	uint32_t state;

	state = DisableInterrupt();
	if (s_nRFxMpslHfxoReqCnt != UINT32_MAX)
	{
		if (s_nRFxMpslHfxoReqCnt == 0U && s_nRFxHfxoReqCnt == 0U &&
			!nRFxHfxoRunning())
		{
#ifdef CLOCK_HFCLKSRC_SRC_Msk
			NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFXO << CLOCK_HFCLKSRC_SRC_Pos;
#endif
			NRF_CLOCK->NRFX_HFXO_STARTED = 0U;
			NRF_CLOCK->NRFX_HFXO_START = 1U;
			s_bNRFxMpslHfxoOwned = true;
		}
		s_nRFxMpslHfxoReqCnt++;
	}
	EnableInterrupt(state);
}

void nRFxMpslHfclkRelease(void)
{
	uint32_t state;

	state = DisableInterrupt();
	if (s_nRFxMpslHfxoReqCnt != 0U)
	{
		s_nRFxMpslHfxoReqCnt--;
		if (s_nRFxMpslHfxoReqCnt == 0U)
		{
			if (s_bNRFxMpslHfxoOwned && s_nRFxHfxoReqCnt == 0U)
			{
				NRF_CLOCK->NRFX_HFXO_STOP = 1U;
#ifdef CLOCK_HFCLKSRC_SRC_Msk
				NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos;
#endif
				s_bNRFxMpslHfxoOwned = false;
			}
		}
	}
	EnableInterrupt(state);
}

bool nRFxMpslHfclkIsRunning(void)
{
	return nRFxHfxoRunning();
}

int32_t nRFxMpslLfclkRequest(void)
{
	uint32_t state;
	uint32_t src;

	if (!nRFxLfclkOscSource(nRFxLowOscSelected(), &src))
	{
		return -1;
	}
	state = DisableInterrupt();
	if (s_nRFxMpslLfclkReqCnt == UINT32_MAX ||
		s_bNRFxLfclkTransition || s_bNRFxLfclkStopPending)
	{
		EnableInterrupt(state);
		return -1;
	}
	if ((NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) != 0U &&
		(NRF_CLOCK->NRFX_LFCLKSRCCOPY & NRFX_LFCLK_SRC_Msk) !=
		(src & NRFX_LFCLK_SRC_Msk))
	{
		EnableInterrupt(state);
		return -1;
	}
	if (s_nRFxMpslLfclkReqCnt == 0U && s_nRFxLfclkReqCnt == 0U &&
		(NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) == 0U)
	{
		NRF_CLOCK->NRFX_LFCLKSRC = src;
		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0U;
		NRF_CLOCK->TASKS_LFCLKSTART = 1U;
		s_bNRFxMpslLfclkOwned = true;
	}
	s_nRFxMpslLfclkReqCnt++;
	EnableInterrupt(state);
	return 0;
}

int32_t nRFxMpslLfclkWait(void)
{
	int i;

	for (i = 0; i < NRFX_LFCLK_WAIT_LOOPS; i++)
	{
		if ((NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) != 0U)
		{
			return 0;
		}
	}
	return -1;
}

int32_t nRFxMpslLfclkRelease(void)
{
	uint32_t state;

	state = DisableInterrupt();
	if (s_nRFxMpslLfclkReqCnt == 0U)
	{
		EnableInterrupt(state);
		return -1;
	}
	s_nRFxMpslLfclkReqCnt--;
	if (s_nRFxMpslLfclkReqCnt == 0U && s_nRFxLfclkReqCnt == 0U &&
		s_bNRFxMpslLfclkOwned)
	{
		NRF_CLOCK->TASKS_LFCLKSTOP = 1U;
		s_bNRFxLfclkStopPending = true;
		s_bNRFxMpslLfclkOwned = false;
	}
	EnableInterrupt(state);
	return 0;
}
#endif
