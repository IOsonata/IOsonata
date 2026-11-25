/**-------------------------------------------------------------------------
@file	timer_nrfx.cpp

@brief	timer implementation on Nordic nRFx series

Low freq timers, RTC, are assigned to device number from 0 to TIMER_NRFX_RTC_MAX - 1
High freq timers, TIMER, are assigned to device number from TIMER_NRFX_RTC_MAX to
TIMER_NRFX_RTC_MAX + TIMER_NRFX_HF_MAX -1

Device number mapping as follow:

DevNo		nRF52832 hardware		Clock type		Max Freq (Hz)	Used by SoftDevice
  0			RTC0					LFCLK			32k				Y
  1			RTC1					LFCLK			32k				Y
  2			RTC2					LFCLK			32k
  3			TIMER0					HFCLK			16M				Y
  4			TIMER1					HFCLK			16M
  5			TIMER2					HFCLK			16M
  6			TIMER3					HFCLK			16M
  7			TIMER4					HFCLK			16M

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2017

@license

MIT License

Copyright (c) 2017 I-SYST inc. All rights reserved.

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
#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "nrf.h"

#include "timer_nrfx.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"

#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
bool nRFxTimerEnablePeiTrigger(TimerDev_t * const pTimer, int TrigNo,
	uint64_t nsPeriod, TIMER_TRIG_TYPE Type, PeiDev_t *pPei,
	PeiEvent_t *pEvent);
bool nRFxRtcEnablePeiTrigger(TimerDev_t *pTimer, int TrigNo,
	uint64_t nsPeriod, TIMER_TRIG_TYPE Type, PeiDev_t *pPei,
	PeiEvent_t *pEvent);
bool nRFxRtcEnablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvent, uint32_t Event);
bool nRFxRtcDisablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t Event);
#endif

#if defined(NRF54L_SERIES)
// A newly started direct LFXO GRTC may pay the crystal's 430-ms startup
// cost. When GRTC is already running, or system LFCLK is already running
// from LFXO, its source is warm. An RC-configured board does not have
// this long LFXO startup penalty. Do not start/stop any clock here.
static bool nRFxGrtcSleepClockWarm(void)
{
	uint32_t lfStat;

	if ((NRF_GRTC->MODE & GRTC_MODE_SYSCOUNTEREN_Msk) != 0U)
	{
		return true;
	}
	if (GetLowFreqOscType() != OSC_TYPE_XTAL)
	{
		return true;
	}
	lfStat = NRF_CLOCK_S->LFCLK.STAT;
	return (lfStat & CLOCK_LFCLK_STAT_STATE_Msk) != 0U &&
		(lfStat & CLOCK_LFCLK_STAT_SRC_Msk) ==
		(CLOCK_LFCLK_SRC_SRC_LFXO << CLOCK_LFCLK_STAT_SRC_Pos);
}
#endif


int TimerEnablePeiTrigger(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvt, uint32_t msPeriod, TIMER_TRIG_TYPE Type)
{
#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	int trig;
	bool success;

	if (pTimer == NULL || pPei == NULL || pEvt == NULL || msPeriod == 0U ||
		pTimer->FindAvailTrigger == NULL)
	{
		return -1;
	}
	trig = pTimer->FindAvailTrigger(pTimer);
	if (trig < 0)
	{
		return -1;
	}
	if (pTimer->DevNo < TIMER_NRFX_RTC_MAX)
	{
		success = nRFxRtcEnablePeiTrigger(pTimer, trig,
			(uint64_t)msPeriod * 1000000ULL, Type, pPei, pEvt);
	}
	else
	{
		success = nRFxTimerEnablePeiTrigger(pTimer, trig,
			(uint64_t)msPeriod * 1000000ULL, Type, pPei, pEvt);
	}
	return success ? trig : -1;
#else
	(void)pTimer;
	(void)pPei;
	(void)pEvt;
	(void)msPeriod;
	(void)Type;
	return -1;
#endif
}

bool TimerEnablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvt, uint32_t Event)
{
#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	if (pTimer == NULL || pPei == NULL || pEvt == NULL ||
		pTimer->DevNo < 0 || pTimer->DevNo >= TIMER_NRFX_RTC_MAX)
	{
		return false;
	}
	return nRFxRtcEnablePeiEvent(pTimer, pPei, pEvt, Event);
#else
	(void)pTimer;
	(void)pPei;
	(void)pEvt;
	(void)Event;
	return false;
#endif
}

bool TimerDisablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t Evt)
{
#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	if (pTimer == NULL || pPei == NULL ||
		pTimer->DevNo < 0 || pTimer->DevNo >= TIMER_NRFX_RTC_MAX)
	{
		return false;
	}
	return nRFxRtcDisablePeiEvent(pTimer, pPei, Evt);
#else
	(void)pTimer;
	(void)pPei;
	(void)Evt;
	return false;
#endif
}

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < TIMER_NRFX_RTC_MAX)
	{
#if defined(NRF54L_SERIES) || defined(NRF71_SERIES)
		return nRFxGrtcInit(pTimer, pCfg);
#else
		return nRFxRtcInit(pTimer, pCfg);
#endif
	}

	return nRFxTimerInit(pTimer, pCfg);
}

int TimerGetLowFreqDevCount()
{
	return TIMER_NRFX_RTC_MAX;
}

int TimerGetHighFreqDevCount()
{
	return TIMER_NRFX_HF_MAX;
}

int TimerGetHighFreqDevNo()
{
	return TIMER_NRFX_RTC_MAX;
}



#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA) || \
	defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || \
	defined(NRF54L_SERIES) || defined(NRF91_SERIES)

// TimerInit selects an unused LF timer for DevNo=-1.
#if !defined(NRF54L_SERIES)
extern "C" bool nRFxSleepLfClockRunning(void);
#endif
#if defined(NRF91_SERIES)
extern "C" bool nRFxSleepLfClockAccurate(void);
#endif

#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)
extern "C" uint32_t nRFxSleepLfClockStartupWaitUs(void);
#endif

// Signed end-to-end compensation in LF ticks. Positive values shorten
// the programmed timer; negative values lengthen it. These are production
// defaults from two independent CoreClockTest runs per nRF5340 core.
// Application cold: 8,539 ticks from the uncorrected 1,260,591 us result;
// corrected cold: 1,000,268 / 999,881 us, warm: 999,653 / 999,679 us.
// Network cold: 1,000,237 / 1,000,245 us, warm: 999,690 / 999,702 us.
// The cold startup value can vary with oscillator conditions.
#if defined(NRF5340_XXAA_APPLICATION)
#define NRF_SDELAY_COLD_DEFAULT_TICKS 8539
#define NRF_SDELAY_WARM_DEFAULT_TICKS (-11)
#elif defined(NRF5340_XXAA_NETWORK)
#define NRF_SDELAY_COLD_DEFAULT_TICKS 8
#define NRF_SDELAY_WARM_DEFAULT_TICKS (-10)
#elif defined(NRF54L15_XXAA)
// The nRF54L15's direct GRTC LFXO takes about 0.43 s to start.
// CoreClockTest measured 1,434,775 us for a cold sDelay(1) with
// zero compensation and 1,000,047 us on the following warm call.
// GRTC uses 1-MHz ticks, not 32,768-Hz RTC ticks. Provisional:
// the crystal startup may change with temperature and board load.
#define NRF_SDELAY_COLD_DEFAULT_TICKS 434775
#define NRF_SDELAY_WARM_DEFAULT_TICKS 0
#elif defined(NRF54LM20A_XXAA)
// Two LM20A CoreClockTest runs at 128 MHz measured cold sDelay(1)
// as 1,433,700 and 1,433,521 us (zero compensation); warm/reuse
// measured 1,000,000/999,998 and 1,000,000/999,999 us.
// GRTC runs at 1 MHz, so compensate approximately 433,611 ticks
// only on cold startup. This value is provisional until retested.
#define NRF_SDELAY_COLD_DEFAULT_TICKS 433611
#define NRF_SDELAY_WARM_DEFAULT_TICKS 0
#elif defined(NRF52832_XXAA) || defined(NRF52832_XXAB)
// nRF52832 cold LFCLK: initial 1,262,331 us at zero compensation.
// With 8,596 ticks: 1,000,118 and 1,000,346 us on two cold runs.
// Their average +232 us excess is approximately 8 LF ticks.
// Use 8,604 ticks provisionally until the next hardware retest.
#define NRF_SDELAY_COLD_DEFAULT_TICKS 8604
#define NRF_SDELAY_WARM_DEFAULT_TICKS 0
#elif defined(NRF52840_XXAA)
// nRF52840 DK: cold/reuse sDelay(1) measured 1,250,500/1,250,506 us
// with zero compensation; warm measured 1,000,125 us.
// Subtract the ~250.5 ms LFCLK cold-start overhead (8,208 LF ticks).
// This default is provisional until hardware is rerun.
#define NRF_SDELAY_COLD_DEFAULT_TICKS 8208
#define NRF_SDELAY_WARM_DEFAULT_TICKS 0
#else
#define NRF_SDELAY_COLD_DEFAULT_TICKS 0
#define NRF_SDELAY_WARM_DEFAULT_TICKS 0
#endif
#ifndef NRF_SDELAY_COLD_OVERHEAD_TICKS
#define NRF_SDELAY_COLD_OVERHEAD_TICKS NRF_SDELAY_COLD_DEFAULT_TICKS
#endif
#ifndef NRF_SDELAY_WARM_OVERHEAD_TICKS
#define NRF_SDELAY_WARM_OVERHEAD_TICKS NRF_SDELAY_WARM_DEFAULT_TICKS
#endif

#define NRF_SDELAY_MAX_CHUNK_SECONDS 60U

static TimerDev_t s_SleepTimer;
static volatile bool s_bSleepTimerBusy;
static volatile bool s_bSleepTimerDone;

static void nRFxSleepTimerWake(TimerDev_t *pTimer, int Trigger,
	void *pContext)
{
	(void)pTimer;
	(void)Trigger;
	(void)pContext;
	s_bSleepTimerDone = true;
	// WFE must still return if the compare IRQ ran immediately before
	// the foreground entered sleep. This event remains latched.
	__SEV();
}

// A low-power seconds delay. The complete function call, including timer
// initialization and disable, is the delay. Busy loops and high-frequency timers are not
// used. WFE uses the same System ON sleep depth as SystemSleep, but the
// latched event prevents a lost-wakeup race between checking the deadline
// and entering sleep.
extern "C" bool sDelay(uint32_t Seconds)
{
	TimerCfg_t cfg;
	uint32_t irqState;
	uint32_t oldScr;
	uint32_t chunk;
	int32_t compensation;
	uint64_t ticks;
	uint64_t nanoseconds;
	uint32_t frequency;
	bool warmClock;
#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)
	uint32_t startupWaitUs;
#endif
	bool initialized;
	bool success;

	if (Seconds == 0U)
	{
		return true;
	}
	if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
	{
		return false;
	}
#if defined(NRF91_SERIES)
	// The nRF91 RC clock has no hardware calibration and may run far
	// from 32768 Hz. Never report a seconds delay from that source.
	if (!nRFxSleepLfClockAccurate())
	{
		return false;
	}
#endif
	irqState = __get_PRIMASK();
	__disable_irq();
	if (s_bSleepTimerBusy)
	{
		__set_PRIMASK(irqState);
		return false;
	}
	s_bSleepTimerBusy = true;
	__set_PRIMASK(irqState);

#if defined(NRF54L_SERIES)
	// The direct LFXO needs a cold-start correction if the GRTC has
	// not started and a ready system LFXO is not already available.
	warmClock = nRFxGrtcSleepClockWarm();
#else
	warmClock = nRFxSleepLfClockRunning();
#endif
	cfg = {};
	cfg.DevNo = -1;
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.Freq = 0U;
	cfg.IntPrio = 6;
	cfg.EvtHandler = nullptr;
	cfg.bTickInt = false;
	initialized = TimerInit(&s_SleepTimer, &cfg);
#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)
	// LFCLK may start immediately after a STOP, or require hundreds of
	// milliseconds for LFXO. Its pre-init running bit cannot distinguish
	// these cases. Use the clock owner's actual startup wait this time.
	startupWaitUs = initialized && !warmClock ?
		nRFxSleepLfClockStartupWaitUs() : 0U;
#endif
	success = initialized;
	if (initialized)
	{
		frequency = TimerGetFrequency(&s_SleepTimer);
		success = frequency != 0U;
	}
	else
	{
		frequency = 0U;
	}

	oldScr = SCB->SCR;
	if (success)
	{
		SCB->SCR = (oldScr & ~(SCB_SCR_SLEEPDEEP_Msk |
			SCB_SCR_SLEEPONEXIT_Msk)) | SCB_SCR_SEVONPEND_Msk;

		while (Seconds != 0U && success)
		{
			chunk = Seconds > NRF_SDELAY_MAX_CHUNK_SECONDS ?
				NRF_SDELAY_MAX_CHUNK_SECONDS : Seconds;
			ticks = (uint64_t)chunk * frequency;
			// The overhead factor applies once to the full call, not
			// once per chunk. RTC compare has a minimum 2-tick lead.
			compensation = Seconds == chunk ?
				(warmClock ? NRF_SDELAY_WARM_OVERHEAD_TICKS :
					NRF_SDELAY_COLD_OVERHEAD_TICKS) : 0U;
#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)
			// Startup wait is counted in calibrated 100-us polls by the
			// Nordic clock owner. Compensate what actually happened, rather
			// than assuming every stopped-LFCLK request takes 260 ms.
			if (!warmClock && Seconds == chunk)
			{
				compensation = (int32_t)(((uint64_t)startupWaitUs *
					frequency + 500000ULL) / 1000000ULL) +
					NRF_SDELAY_WARM_OVERHEAD_TICKS;
			}
#endif
			if (compensation > 0)
			{
				// Never program less than the RTC's two-tick compare lead.
				if (ticks <= (uint32_t)compensation + 2U)
				{
					ticks = 2U;
				}
				else
				{
					ticks -= (uint32_t)compensation;
				}
			}
			else if (compensation < 0)
			{
				// A negative correction adds ticks without unsigned wrap.
				ticks += (uint32_t)(-(int64_t)compensation);
			}
			nanoseconds = (ticks * 1000000000ULL + frequency / 2U) /
				frequency;
			s_bSleepTimerDone = false;
			success = nsTimerEnableTrigger(&s_SleepTimer, 0,
				nanoseconds, TIMER_TRIG_TYPE_SINGLE, nRFxSleepTimerWake,
				nullptr) != 0U;
			if (success)
			{
				// Interrupt-driven System ON sleep, with a latched event
				// in the handler to avoid missing an already-serviced IRQ.
				while (!s_bSleepTimerDone)
				{
					__DSB();
					__WFE();
				}
				Seconds -= chunk;
			}
		}
		SCB->SCR = oldScr;
		__ISB();
	}
	// Always return a successfully initialized automatic timer to the
	// driver, including a zero-frequency or trigger-setup failure.
	if (initialized)
	{
		TimerDisableTrigger(&s_SleepTimer, 0);
		if (!TimerPowerOff(&s_SleepTimer))
		{
			success = false;
		}
	}
	irqState = __get_PRIMASK();
	__disable_irq();
	s_bSleepTimerBusy = false;
	__set_PRIMASK(irqState);
	return success && Seconds == 0U;
}
#endif
