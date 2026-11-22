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
#if defined(NRF54L_SERIES)
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
	defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK)

// The RTC backend manages timer ownership and LFCLK reference counting.
// Claiming skips initialized timers rather than replacing a user handle.
extern "C" int nRFxRtcSleepClaim(TimerDev_t *pTimer, TimerCfg_t *pCfg);
extern "C" void nRFxRtcSleepRelease(TimerDev_t *pTimer);
extern "C" bool nRFxSleepLfClockRunning(void);

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
#elif defined(NRF52832_XXAA) || defined(NRF52832_XXAB)
// First nRF52832 cold LFCLK measurement: 1,262,331 us without correction.
// This 8,596-tick candidate must be verified with repeated cold startups.
#define NRF_SDELAY_COLD_DEFAULT_TICKS 8596
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
// claim/release, is the delay. Busy loops and high-frequency timers are not
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
	bool success;

	if (Seconds == 0U)
	{
		return true;
	}
	if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
	{
		return false;
	}
	irqState = __get_PRIMASK();
	__disable_irq();
	if (s_bSleepTimerBusy)
	{
		__set_PRIMASK(irqState);
		return false;
	}
	s_bSleepTimerBusy = true;
	__set_PRIMASK(irqState);

	warmClock = nRFxSleepLfClockRunning();
	cfg = {};
	cfg.DevNo = -1;
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.Freq = 0U;
	cfg.IntPrio = 6;
	cfg.EvtHandler = nullptr;
	cfg.bTickInt = false;
	success = nRFxRtcSleepClaim(&s_SleepTimer, &cfg) >= 0;
	if (success)
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
		nRFxRtcSleepRelease(&s_SleepTimer);
	}
	else if (s_SleepTimer.Enable != nullptr &&
		s_SleepTimer.DevNo >= 1 &&
		s_SleepTimer.DevNo < TimerGetLowFreqDevCount())
	{
		// A claim can fail after a timer was partially configured;
		// the RTC backend only releases handles it still owns.
		nRFxRtcSleepRelease(&s_SleepTimer);
	}
	irqState = __get_PRIMASK();
	__disable_irq();
	s_bSleepTimerBusy = false;
	__set_PRIMASK(irqState);
	return success && Seconds == 0U;
}
#endif
