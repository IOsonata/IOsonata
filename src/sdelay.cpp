/**-------------------------------------------------------------------------
@file	sdelay.cpp

@brief	Seconds delay in light sleep on a timer

sDelay takes a free timer for the time of the call, a low power one when there
is one, a high frequency one otherwise. sDelayTimer takes the timer given.
TimerInit refuses a timer held by another Timer object, so the application
timers are never taken. The time the clock code waited to start the timer clock
and the setup time of the timer, nsSetup, are part of the delay: the call lasts
the requested time from the call to the return.

@author	Hoang Nguyen Hoan
@date	Nov. 27, 2026

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
#include <stdbool.h>

#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "idelay.h"

// Longest wait of one trigger, a longer delay is done in parts
#define SDELAY_PART_SEC			60U

// Timer rate asked, the timer takes the closest one it has
#define SDELAY_TIMER_FREQ		32768U

#if defined(__arm__) || defined(__ICCARM__)
// System Control Register, at the same address on every Cortex-M
#define SDELAY_SCR				(*(volatile uint32_t *)0xE000ED10UL)
#define SDELAY_SCR_SLEEPONEXIT	(1UL << 1)
#define SDELAY_SCR_SLEEPDEEP	(1UL << 2)
#endif

static TimerDev_t s_SDelayTimer;
static volatile bool s_bSDelayBusy = false;
static volatile bool s_bSDelayDone = false;

static void SDelayTimerHandler(TimerDev_t *pTimer, int TrigNo, void *pContext)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pContext;

	s_bSDelayDone = true;
}

// Called from an interrupt or with the interrupts masked, the timer interrupt
// could not end the wait
static bool SDelayIntContext(void)
{
#if defined(__arm__) || defined(__ICCARM__)
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || \
	defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8_1M_MAIN__)
	if (__get_BASEPRI() != 0U || __get_FAULTMASK() != 0U)
	{
		return true;
	}
#endif
	return __get_IPSR() != 0U || __get_PRIMASK() != 0U;
#elif defined(__riscv)
	uintptr_t status;

	// MIE, bit 3 of mstatus, is off in a trap handler and when masked
	status = DisableInterrupt();
	EnableInterrupt(status);

	return (status & 8U) == 0U;
#else
	return false;
#endif
}

// Takes the delay for this call. False from an interrupt, with the interrupts
// masked or during another delay: the timer interrupt could not end the wait,
// or the timer is in use.
static bool SDelayAcquire(void)
{
	uint32_t state;
	bool success;

	if (SDelayIntContext())
	{
		return false;
	}
	state = DisableInterrupt();
	success = !s_bSDelayBusy;
	if (success)
	{
		s_bSDelayBusy = true;
	}
	EnableInterrupt(state);

	return success;
}

static void SDelayRelease(void)
{
	uint32_t state;

	state = DisableInterrupt();
	s_bSDelayBusy = false;
	EnableInterrupt(state);
}

// Init the first free timer from DevNo First, Count timers, DevNo moving by
// Step. The clock of the timers is requested before, so the time its start
// took is known. The timer keeps its own request from its Init.
static bool SDelayTimerInit(int First, int Count, int Step, bool bLowFreq, uint32_t *pStartUs)
{
	TimerCfg_t cfg;
	bool clock;
	bool success;
	int i;

	clock = bLowFreq ? SystemLowFreqClockEnable() : SystemHighFreqClockEnable();
	*pStartUs = 0U;
	if (clock)
	{
		*pStartUs = bLowFreq ? SystemLowFreqClockStartTime() : SystemHighFreqClockStartTime();
	}

	cfg = {};
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.Freq = SDELAY_TIMER_FREQ;
	cfg.IntPrio = IRQ_PRIO_LOW;
	cfg.EvtHandler = nullptr;
	cfg.bTickInt = false;
	success = false;
	for (i = 0; i < Count && !success; i++)
	{
		cfg.DevNo = First + i * Step;
		// 0 for a timer driver that does not set it
		s_SDelayTimer.nsSetup = 0;
		success = TimerInit(&s_SDelayTimer, &cfg);
	}

	if (clock)
	{
		if (bLowFreq)
		{
			SystemLowFreqClockDisable();
		}
		else
		{
			SystemHighFreqClockDisable();
		}
	}

	return success;
}

// The delay on the timer that SDelayTimerInit started, StartUs the time its
// clock took to start. The timer is powered off at the end.
static bool SDelayRun(uint32_t Seconds, uint32_t StartUs)
{
	uint64_t remaining;
	uint64_t actual;
	uint64_t ns;
	int64_t setup;
	uint32_t state;
	bool success;
#if defined(__arm__) || defined(__ICCARM__)
	uint32_t scr;
#endif

	// The clock start and the setup of the timer are part of the call
	remaining = (uint64_t)Seconds * 1000000000ULL;
	setup = (int64_t)StartUs * 1000LL + s_SDelayTimer.nsSetup;
	if (setup >= 0)
	{
		remaining -= (uint64_t)setup < remaining ? (uint64_t)setup : remaining;
	}
	else
	{
		remaining += (uint64_t)(-setup);
	}
	success = true;
#if defined(__arm__) || defined(__ICCARM__)
	// Light sleep, the timer keeps running
	scr = SDELAY_SCR;
	SDELAY_SCR = scr & ~(SDELAY_SCR_SLEEPDEEP | SDELAY_SCR_SLEEPONEXIT);
#endif
	while (success && remaining != 0U)
	{
		ns = SDELAY_PART_SEC * 1000000000ULL;
		if (remaining < ns)
		{
			ns = remaining;
		}
		s_bSDelayDone = false;
		actual = nsTimerEnableTrigger(&s_SDelayTimer, 0, ns, TIMER_TRIG_TYPE_SINGLE,
			SDelayTimerHandler, nullptr);
		success = actual != 0U;
		// A narrow counter can fit only part of the requested interval.
		// Treat rounding within one timer tick as the full interval.
		remaining -= actual < ns && ns - actual > s_SDelayTimer.nsPeriod ? actual : ns;

		// With the interrupts masked a pending timer interrupt still ends
		// the wait, its handler runs once they are on again. No interrupt
		// is lost between the check and the wait.
		state = DisableInterrupt();
		while (success && !s_bSDelayDone)
		{
#if defined(__arm__) || defined(__ICCARM__)
			__DSB();
#endif
			__WFI();
			EnableInterrupt(state);
			state = DisableInterrupt();
		}
		EnableInterrupt(state);
	}
#if defined(__arm__) || defined(__ICCARM__)
	SDELAY_SCR = scr;
#endif
	TimerDisableTrigger(&s_SDelayTimer, 0);
	if (!TimerPowerOff(&s_SDelayTimer))
	{
		success = false;
	}

	return success;
}

bool sDelay(uint32_t Seconds)
{
	uint32_t startUs;
	int count;
	bool success;

	if (Seconds == 0U)
	{
		return true;
	}
	if (!SDelayAcquire())
	{
		return false;
	}

	// A low power timer first, from the last one as the application timers
	// usually start at DevNo 0. A high frequency timer otherwise, from the
	// lowest power one.
	count = TimerGetLowFreqDevCount();
	success = count > 0 && SDelayTimerInit(count - 1, count, -1, true, &startUs);
	if (!success)
	{
		success = SDelayTimerInit(TimerGetHighFreqDevNo(), TimerGetHighFreqDevCount(), 1, false, &startUs);
	}
	if (success)
	{
		success = SDelayRun(Seconds, startUs);
	}
	SDelayRelease();

	return success;
}

bool sDelayTimer(int DevNo, uint32_t Seconds)
{
	uint32_t startUs;
	bool success;

	if (Seconds == 0U)
	{
		return true;
	}
	if (DevNo < 0 || DevNo >= TimerGetHighFreqDevNo() + TimerGetHighFreqDevCount() ||
		!SDelayAcquire())
	{
		return false;
	}

	success = SDelayTimerInit(DevNo, 1, 1, DevNo < TimerGetHighFreqDevNo(), &startUs) &&
		SDelayRun(Seconds, startUs);
	SDelayRelease();

	return success;
}
