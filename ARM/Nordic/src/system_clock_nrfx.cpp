/**-------------------------------------------------------------------------
@file	system_clock_nrfx.cpp

@brief	Clock functions of system_core_clock.h on the nRF52, nRF53, nRF54L and
		nRF91

The CPU always runs from HFCLK, sourced by the internal RC oscillator (HFINT)
or by the 32 MHz crystal (HFXO). The crystal only improves accuracy. The
radio and USB need it and it draws much more current, so it is started on
request, not at startup.

SystemCoreClockSelect with OSC_TYPE_XTAL or OSC_TYPE_TCXO is one request for
the crystal, with OSC_TYPE_RC it releases one. The requests of the drivers
and of the application are counted. The crystal starts on the first request
and stops when the last one is released. A crystal already running before
the first request, started by other code, is left running.

When a BLE stack is enabled it owns CLOCK. The requests then go to the
SoftDevice (sd_clock_hfclk_request and sd_clock_hfclk_release) or to MPSL for
the SoftDevice Controller (nRFxMpslHfxo in nrf_mpsl.cpp). Neither counts the
callers of the application, the count stays here. Every request is passed to
the stack, so a request made before the stack was enabled is passed again on
the next one.

On the nRF53, HFCLKSRC selects the crystal before HFCLKSTART and selects the
RC again after HFCLKSTOP. On the nRF54L the crystal is started and stopped
with XOSTART and XOSTOP, its state is in XO.STAT. The PLL is not touched, the
CPU clock started by SystemInit keeps running from it.

SystemCoreClockSelect does not change g_McuOsc, it stays the description of
the board.

SystemLowFreqClockSelect selects the source of LFCLK, the 32 kHz clock of
the RTCs, and starts it, restarting it when it ran from another source. It
records the source in g_McuOsc.LowPwrOsc. The RC, the crystal, and on the
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
#include "nrf_mbr.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "nrf_mpsl.h"

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

// Well over the start time of the 32 kHz crystal, about 0.25 s
#define NRFX_LFCLK_WAIT_LOOPS		10000000

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

// sd_softdevice_is_enabled is an SVC that only a programmed SoftDevice
// implements, so the image is looked for first: its magic number follows the
// MBR.
static bool nRFxClockSdEnabled(void)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	const volatile uint32_t *pMagic = (const volatile uint32_t *)(
		MBR_SIZE + SOFTDEVICE_INFO_STRUCT_OFFSET + 4U);

	if (*pMagic != 0x51B1E5DBUL)
	{
		return false;
	}

	uint8_t enabled = 0U;

	(void)sd_softdevice_is_enabled(&enabled);

	return enabled != 0U;
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

// Acquire the oscillator through its current owner. The caller holds the
// transition flag, but interrupts remain enabled during the start wait.
static bool nRFxHfxoStart(nRFxHfxoOwner_t *pOwner)
{
	int res;

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
		uint32_t running;
		int i;

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

	for (int i = 0; i < NRFX_HFXO_WAIT_LOOPS; i++)
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

// Release only through the backend that accepted the first request.
static void nRFxHfxoStop(nRFxHfxoOwner_t Owner)
{
	switch (Owner)
	{
		case NRFX_HFXO_OWNER_MPSL:
			(void)nRFxMpslHfxo(false);
			break;
		case NRFX_HFXO_OWNER_SOFTDEVICE:
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
			if (nRFxClockSdEnabled())
			{
				(void)sd_clock_hfclk_release();
			}
#endif
			break;
		case NRFX_HFXO_OWNER_DIRECT:
			// Once a BLE stack owns CLOCK, direct register writes are unsafe.
			if (!nRFxMpslActive() && !nRFxClockSdEnabled())
			{
				NRF_CLOCK->NRFX_HFXO_STOP = 1U;
#ifdef CLOCK_HFCLKSRC_SRC_Msk
				NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos;
#endif
			}
			break;
		default:
			break;
	}
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	nRFxHfxoOwner_t owner;
	uint32_t state;
	bool success;

	(void)OscFreq;
	if (ClkSrc != OSC_TYPE_RC && ClkSrc != OSC_TYPE_XTAL &&
		ClkSrc != OSC_TYPE_TCXO)
	{
		return false;
	}

	state = DisableInterrupt();
	if (s_bNRFxHfxoTransition)
	{
		EnableInterrupt(state);
		// Do not wait for a preempted clock transition in an ISR.
		return false;
	}
	if (ClkSrc != OSC_TYPE_RC && s_nRFxHfxoReqCnt != 0U)
	{
		s_nRFxHfxoReqCnt++;
		EnableInterrupt(state);
		return true;
	}
	if (ClkSrc == OSC_TYPE_RC && s_nRFxHfxoReqCnt == 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	if (ClkSrc == OSC_TYPE_RC && s_nRFxHfxoReqCnt > 1U)
	{
		s_nRFxHfxoReqCnt--;
		EnableInterrupt(state);
		return true;
	}

	s_bNRFxHfxoTransition = true;
	owner = s_nRFxHfxoOwner;
	EnableInterrupt(state);

	if (ClkSrc == OSC_TYPE_RC)
	{
		nRFxHfxoStop(owner);
		success = true;
	}
	else
	{
		owner = NRFX_HFXO_OWNER_NONE;
		success = nRFxHfxoStart(&owner);
	}

	state = DisableInterrupt();
	if (ClkSrc == OSC_TYPE_RC)
	{
		s_nRFxHfxoReqCnt = 0U;
		s_nRFxHfxoOwner = NRFX_HFXO_OWNER_NONE;
	}
	else if (success)
	{
		s_nRFxHfxoReqCnt = 1U;
		s_nRFxHfxoOwner = owner;
	}
	s_bNRFxHfxoTransition = false;
	EnableInterrupt(state);

	return success;
}

// Starts LFCLK from Src and waits until it runs. Stopped again if it does
// not start.
static bool nRFxLfclkStart(uint32_t Src)
{
	NRF_CLOCK->NRFX_LFCLKSRC = Src;
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;

	for (int i = 0; i < NRFX_LFCLK_WAIT_LOOPS; i++)
	{
		if (NRF_CLOCK->EVENTS_LFCLKSTARTED)
		{
			NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

			return true;
		}
	}

	NRF_CLOCK->TASKS_LFCLKSTOP = 1;

	return false;
}

static void nRFxLfclkStop(void)
{
	NRF_CLOCK->TASKS_LFCLKSTOP = 1;

	for (int i = 0; i < NRFX_LFCLK_WAIT_LOOPS; i++)
	{
		if ((NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) == 0)
		{
			break;
		}
	}
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	// The 32 kHz oscillators of these parts are 32768 Hz
	(void)OscFreq;

	uint32_t src;

	switch (ClkSrc)
	{
		case OSC_TYPE_RC:
			src = NRFX_LFCLK_SRC_RC << NRFX_LFCLK_SRC_Pos;
			break;
		case OSC_TYPE_XTAL:
			src = NRFX_LFCLK_SRC_XTAL << NRFX_LFCLK_SRC_Pos;
			break;
#if defined(CLOCK_LFCLKSRC_BYPASS_Msk) && defined(CLOCK_LFCLKSRC_EXTERNAL_Msk)
		case OSC_TYPE_TCXO:
			// Rail to rail clock on XL1
			src = (NRFX_LFCLK_SRC_XTAL << NRFX_LFCLK_SRC_Pos) |
				  CLOCK_LFCLKSRC_BYPASS_Msk | CLOCK_LFCLKSRC_EXTERNAL_Msk;
			break;
#endif
		default:
			return false;
	}

	// A BLE stack owns LFCLK, it set the source when it was enabled
	if (nRFxClockSdEnabled() || nRFxMpslActive())
	{
		return false;
	}

	bool running = (NRF_CLOCK->NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk) != 0;
	uint32_t prev = NRF_CLOCK->NRFX_LFCLKSRC;

	if (running == false || prev != src ||
		(NRF_CLOCK->NRFX_LFCLKSRCCOPY & NRFX_LFCLK_SRC_Msk) != (src & NRFX_LFCLK_SRC_Msk))
	{
		if (running)
		{
			nRFxLfclkStop();
		}

		if (nRFxLfclkStart(src) == false)
		{
			// Back to the source that ran before
			if (running)
			{
				(void)nRFxLfclkStart(prev);
			}

			return false;
		}
	}

	g_McuOsc.LowPwrOsc.Type = ClkSrc;
	g_McuOsc.LowPwrOsc.Freq = 32768;

	return true;
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
