/**-------------------------------------------------------------------------
@file	wdt_lpc546xx.cpp

@brief	LPC546xx windowed watchdog (WWDT)

Generic watchdog API (coredev/wdt.h) on the WWDT of the LPC546xx. DevNo 0
only.

The WWDT counts down a 24 bit counter at the watchdog oscillator divided by
4. The oscillator frequency is set by SYSCON WDTOSCCTRL, its reset value
gives 500 kHz (FREQSEL 5, 1 MHz, DIVSEL 0, divided by 2), a 125 kHz count.
The driver uses the frequency of the WDTOSCCTRL value it finds, with the
FREQSEL table of the NXP SDK clock driver, and sets the reset value when
FREQSEL is 0. The watchdog oscillator is accurate to +-40 percent, the
timeout of a part follows its oscillator. The timeout is msTimeout rounded
up to the next count, at least 255 counts (the smallest TC), at most
0xFFFFFF counts (134 s at 125 kHz); a longer timeout fails WdtInit.
WdtDev_t::msTimeout is the timeout in effect, rounded up.

msWindow: a feed sooner than msWindow, rounded down to a count, after the
previous one resets the MCU (WINDOW register). Not above msTimeout.

NbChan 1-32 in software: the WWDT is fed when every channel has been
reloaded since the last feed.

WdtStart sets WDEN, WDRESET (a timeout resets the MCU) and LOCK (the
watchdog oscillator cannot be turned off, also in deep sleep), feeds, then
waits for TV to leave its reset value, the sign that the counter runs.
These bits are cleared only by a reset: the watchdog runs until the next
reset. bRunSleep false fails WdtInit. The WWDT has no setting for a
debugger halt, bRunHalt is accepted and has no effect.

EvtHandler: the warning interrupt (WARNINT) comes 1023 counts before the
timeout (8 ms at 125 kHz), or at half the timeout for a timeout of less than
2046 counts, on WDT_BOD_IRQHandler (shared with the brown-out detector).

A WWDT running at WdtInit (WDEN set) keeps its configuration, reported in
WdtDev_t::msTimeout; its warning interrupt is set from this Init.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#include <stddef.h>

#include "LPC546xx.h"

#include "coredev/interrupt.h"
#include "coredev/wdt.h"

#define LPC546XX_WDT_PRESCALE		4U			// The counter runs at the oscillator / 4
#define LPC546XX_WDT_TC_MIN			0xFFU
#define LPC546XX_WDT_TC_MAX			0xFFFFFFU
#define LPC546XX_WDT_WINDOW_NONE	0xFFFFFFU
#define LPC546XX_WDT_WARN_MAX		0x3FFU
#define LPC546XX_WDT_FEED1			0xAAU
#define LPC546XX_WDT_FEED2			0x55U
#define LPC546XX_WDT_CHAN_MAX		32
#define LPC546XX_WDT_TV_RESET		0xFFU		// TV until the counter has taken TC
#define LPC546XX_WDT_START_READS	100000		// TV reads while the counter starts

// WDTOSCCTRL reset value, FREQSEL 5 (1 MHz) and DIVSEL 0 (divided by 2)
#define LPC546XX_WDT_OSCCTRL_RESET	(SYSCON_WDTOSCCTRL_FREQSEL(5) | SYSCON_WDTOSCCTRL_DIVSEL(0))

// MOD bits written back by a read modify write. WDINT is cleared by writing
// 1, it is left out so that a write does not clear it.
#define LPC546XX_WDT_MOD_KEEP		(WWDT_MOD_WDEN_MASK | WWDT_MOD_WDRESET_MASK | WWDT_MOD_WDTOF_MASK | \
									 WWDT_MOD_WDPROTECT_MASK | WWDT_MOD_LOCK_MASK)

#pragma pack(push, 4)

typedef struct {
	WdtDev_t *pDev;					//!< Device of the last Init
	uint32_t Tc;					//!< Timeout counts for WdtStart
	uint32_t Window;				//!< Window register value for WdtStart
	uint32_t WarnInt;				//!< Warning interrupt counts, 0 none
	uint32_t ChanMask;				//!< Reload channels
	uint32_t ReloadMask;			//!< Channels reloaded since the last feed
} Lpc546xxWdt_t;

#pragma pack(pop)

static Lpc546xxWdt_t s_Lpc546xxWdt;

// Watchdog oscillator frequency of each FREQSEL value, in 50 kHz steps, as
// the NXP SDK clock driver
static const uint8_t s_Lpc546xxWdtOscSel[32] = {
	0, 8, 12, 15, 18, 20, 24, 26, 28, 30, 32, 34, 36, 38, 40, 41,
	42, 44, 45, 46, 48, 49, 50, 52, 53, 54, 56, 57, 58, 59, 60, 61,
};

extern "C" void WDT_BOD_IRQHandler(void);

// Counter clock in Hz, 0 when WDTOSCCTRL selects no frequency
static uint32_t Lpc546xxWdtClock(void)
{
	uint32_t r = SYSCON->WDTOSCCTRL;
	uint32_t f = s_Lpc546xxWdtOscSel[(r & SYSCON_WDTOSCCTRL_FREQSEL_MASK) >> SYSCON_WDTOSCCTRL_FREQSEL_SHIFT] * 50000U;
	uint32_t div = (((r & SYSCON_WDTOSCCTRL_DIVSEL_MASK) >> SYSCON_WDTOSCCTRL_DIVSEL_SHIFT) + 1U) * 2U;

	return f / (div * LPC546XX_WDT_PRESCALE);
}

// Timeout of a TC value, msec rounded up
static uint32_t Lpc546xxWdtMs(uint32_t Tc, uint32_t Clk)
{
	return (uint32_t)(((uint64_t)Tc * 1000U + Clk - 1U) / Clk);
}

// Warning interrupt counts before the timeout
static uint32_t Lpc546xxWdtWarn(uint32_t Tc)
{
	uint32_t w = Tc / 2U;

	return w > LPC546XX_WDT_WARN_MAX ? LPC546XX_WDT_WARN_MAX : w;
}

static bool Lpc546xxWdtRunning(void)
{
	return (SYSCON->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_WWDT_MASK) && (WWDT->MOD & WWDT_MOD_WDEN_MASK);
}

bool WdtInit(WdtDev_t * const pDev, const WdtCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->DevNo != 0 || pCfg->msTimeout == 0 ||
		pCfg->bRunSleep == false)
	{
		return false;
	}

	int nbchan = pCfg->NbChan > 0 ? pCfg->NbChan : 1;

	if (nbchan > LPC546XX_WDT_CHAN_MAX)
	{
		return false;
	}

	bool running = Lpc546xxWdtRunning();

	// The oscillator frequency in use. FREQSEL 0 selects none, the reset value
	// is set then, only while the WWDT is stopped.
	if (running == false && (SYSCON->WDTOSCCTRL & SYSCON_WDTOSCCTRL_FREQSEL_MASK) == 0)
	{
		SYSCON->WDTOSCCTRL = LPC546XX_WDT_OSCCTRL_RESET;
	}

	uint32_t clk = Lpc546xxWdtClock();

	if (clk == 0)
	{
		return false;
	}

	// Counts rounded up, at least the smallest TC
	uint64_t tc = ((uint64_t)pCfg->msTimeout * clk + 999U) / 1000U;

	if (tc > LPC546XX_WDT_TC_MAX)
	{
		return false;
	}
	if (tc < LPC546XX_WDT_TC_MIN)
	{
		tc = LPC546XX_WDT_TC_MIN;
	}

	uint32_t window = LPC546XX_WDT_WINDOW_NONE;

	if (pCfg->msWindow != 0)
	{
		if (pCfg->msWindow >= pCfg->msTimeout)
		{
			return false;
		}

		// A feed is allowed when the counter is at the window value or lower:
		// window counts after the feed, rounded down
		uint64_t win = (uint64_t)pCfg->msWindow * clk / 1000U;

		window = (uint32_t)tc - (uint32_t)win;
	}

	NVIC_DisableIRQ(WDT_BOD_IRQn);

	uint32_t state = DisableInterrupt();

	s_Lpc546xxWdt.pDev = pDev;
	s_Lpc546xxWdt.ChanMask = nbchan == LPC546XX_WDT_CHAN_MAX ? 0xFFFFFFFFU : (1UL << nbchan) - 1U;
	s_Lpc546xxWdt.ReloadMask = 0;
	EnableInterrupt(state);

	pDev->DevNo = 0;
	pDev->NbChan = nbchan;
	pDev->EvtHandler = pCfg->EvtHandler;
	pDev->pDevData = &s_Lpc546xxWdt;

	if (running)
	{
		// Started before: its configuration is kept, report it
		uint32_t tcnow = WWDT->TC & WWDT_TC_COUNT_MASK;

		pDev->msTimeout = Lpc546xxWdtMs(tcnow, clk);
		s_Lpc546xxWdt.WarnInt = pCfg->EvtHandler != nullptr ? Lpc546xxWdtWarn(tcnow) : 0;

		// The warning interrupt of this Init, a pending one cleared
		WWDT->WARNINT = s_Lpc546xxWdt.WarnInt;
		WWDT->MOD = (WWDT->MOD & LPC546XX_WDT_MOD_KEEP) | WWDT_MOD_WDINT_MASK;
	}
	else
	{
		s_Lpc546xxWdt.Tc = (uint32_t)tc;
		s_Lpc546xxWdt.Window = window;
		s_Lpc546xxWdt.WarnInt = pCfg->EvtHandler != nullptr ? Lpc546xxWdtWarn((uint32_t)tc) : 0;
		pDev->msTimeout = Lpc546xxWdtMs((uint32_t)tc, clk);
	}

	NVIC_ClearPendingIRQ(WDT_BOD_IRQn);
	if (pCfg->EvtHandler != nullptr)
	{
		NVIC_SetPriority(WDT_BOD_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(WDT_BOD_IRQn);
	}

	return true;
}

bool WdtStart(WdtDev_t * const pDev)
{
	if (pDev == nullptr || pDev->pDevData != &s_Lpc546xxWdt)
	{
		return false;
	}

	// Started before: its configuration is kept
	if (Lpc546xxWdtRunning())
	{
		return true;
	}

	// Watchdog oscillator powered, bus clock on
	SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_WDT_OSC_MASK;
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_WWDT_MASK;

	uint32_t state = DisableInterrupt();

	s_Lpc546xxWdt.ReloadMask = 0;

	WWDT->TC = s_Lpc546xxWdt.Tc;
	WWDT->MOD = (WWDT->MOD & LPC546XX_WDT_MOD_KEEP) | WWDT_MOD_WDEN_MASK | WWDT_MOD_WDRESET_MASK |
				WWDT_MOD_LOCK_MASK | WWDT_MOD_WDINT_MASK;
	WWDT->WINDOW = s_Lpc546xxWdt.Window;
	WWDT->WARNINT = s_Lpc546xxWdt.WarnInt;

	// The first feed starts the counter
	WWDT->FEED = LPC546XX_WDT_FEED1;
	WWDT->FEED = LPC546XX_WDT_FEED2;
	EnableInterrupt(state);

	// TV reads its reset value until the counter has taken TC, a few
	// watchdog clocks, as the NXP SDK waits for it. TC of 0xFF is seen once
	// it counts down.
	for (int i = 0; i < LPC546XX_WDT_START_READS && WWDT->TV == LPC546XX_WDT_TV_RESET; i++)
	{
	}

	return true;
}

void WdtReload(WdtDev_t * const pDev, int Chan)
{
	if (pDev == nullptr || pDev->pDevData != &s_Lpc546xxWdt || Chan < 0 || Chan >= pDev->NbChan)
	{
		return;
	}

	// Interrupts masked: nothing else may access the WWDT between the two
	// writes of the feed sequence
	uint32_t state = DisableInterrupt();
	uint32_t mask = s_Lpc546xxWdt.ReloadMask | (1UL << Chan);

	if ((mask & s_Lpc546xxWdt.ChanMask) == s_Lpc546xxWdt.ChanMask)
	{
		WWDT->FEED = LPC546XX_WDT_FEED1;
		WWDT->FEED = LPC546XX_WDT_FEED2;
		mask = 0;
	}
	s_Lpc546xxWdt.ReloadMask = mask;
	EnableInterrupt(state);
}

bool WdtRunning(WdtDev_t * const pDev)
{
	(void)pDev;

	return Lpc546xxWdtRunning();
}

// Shared with the brown-out detector: only the watchdog warning is handled
void WDT_BOD_IRQHandler(void)
{
	if ((SYSCON->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_WWDT_MASK) && (WWDT->MOD & WWDT_MOD_WDINT_MASK))
	{
		WWDT->MOD = (WWDT->MOD & LPC546XX_WDT_MOD_KEEP) | WWDT_MOD_WDINT_MASK;

		WdtDev_t *dev = s_Lpc546xxWdt.pDev;

		if (dev != nullptr && dev->EvtHandler != nullptr)
		{
			dev->EvtHandler(dev);
		}
	}
}
