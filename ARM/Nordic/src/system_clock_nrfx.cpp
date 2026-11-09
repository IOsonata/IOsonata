/**-------------------------------------------------------------------------
@file	system_clock_nrfx.cpp

@brief	Core clock selection of system_core_clock.h on the nRF52, nRF53 and nRF91

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
RC again after HFCLKSTOP.

g_McuOsc stays the description of the board, it is not changed.

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
#include <atomic>

#include "nrf.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_mbr.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#include "coredev/system_core_clock.h"
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

#ifdef CLOCK_HFCLKSTAT_SRC_Xtal
#define NRFX_HFXO_STAT_SRC			(CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)
#else
#define NRFX_HFXO_STAT_SRC			(CLOCK_HFCLKSTAT_SRC_HFXO << CLOCK_HFCLKSTAT_SRC_Pos)
#endif

static std::atomic<int> s_nRFxHfxoReqCnt(0);
static volatile bool s_bnRFxHfxoOwned = false;

// Weak: the SoftDevice Controller is not in the build. nrf_mpsl.cpp has the
// version that goes through MPSL once it is initialized.
__WEAK int nRFxMpslHfxo(bool bOn)
{
	(void)bOn;

	return -1;
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
	return (NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_STATE_Msk | CLOCK_HFCLKSTAT_SRC_Msk)) ==
		   (CLOCK_HFCLKSTAT_STATE_Msk | NRFX_HFXO_STAT_SRC);
}

// Starts the crystal if needed and waits until it runs
static bool nRFxHfxoStart(void)
{
	int res = nRFxMpslHfxo(true);

	if (res >= 0)
	{
		return res == 0;
	}

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	if (nRFxClockSdEnabled())
	{
		uint32_t running = 0;

		if (sd_clock_hfclk_request() != NRF_SUCCESS)
		{
			return false;
		}

		for (int i = 0; i < NRFX_HFXO_WAIT_LOOPS; i++)
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

		return false;
	}
#endif

	if (nRFxHfxoRunning())
	{
		return true;
	}

#ifdef CLOCK_HFCLKSRC_SRC_Msk
	NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFXO << CLOCK_HFCLKSRC_SRC_Pos;
#endif
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	s_bnRFxHfxoOwned = true;

	for (int i = 0; i < NRFX_HFXO_WAIT_LOOPS; i++)
	{
		if (nRFxHfxoRunning())
		{
			NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

			return true;
		}
	}

	return false;
}

// Stops the crystal, only if it was started here
static void nRFxHfxoStop(void)
{
	if (nRFxMpslHfxo(false) >= 0)
	{
		return;
	}

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	if (nRFxClockSdEnabled())
	{
		(void)sd_clock_hfclk_release();

		return;
	}
#endif

	if (s_bnRFxHfxoOwned)
	{
		s_bnRFxHfxoOwned = false;
		NRF_CLOCK->TASKS_HFCLKSTOP = 1;
#ifdef CLOCK_HFCLKSRC_SRC_Msk
		NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos;
#endif
	}
}

// Releases one request, stops the crystal on the last one
static void nRFxHfxoRelease(void)
{
	int cnt = s_nRFxHfxoReqCnt.load();

	while (cnt > 0 && !s_nRFxHfxoReqCnt.compare_exchange_weak(cnt, cnt - 1));

	if (cnt == 1)
	{
		nRFxHfxoStop();

		// A request made from an interrupt while it was stopping
		if (s_nRFxHfxoReqCnt.load() > 0)
		{
			(void)nRFxHfxoStart();
		}
	}
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	// The crystal is 32 MHz on these parts, the frequency changes nothing
	(void)OscFreq;

	if (ClkSrc == OSC_TYPE_RC)
	{
		nRFxHfxoRelease();

		return true;
	}

	s_nRFxHfxoReqCnt++;

	if (nRFxHfxoStart())
	{
		return true;
	}

	// Not started, the request is dropped
	nRFxHfxoRelease();

	return false;
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
