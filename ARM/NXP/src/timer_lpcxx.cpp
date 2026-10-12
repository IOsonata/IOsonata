/**-------------------------------------------------------------------------
@file	timer_lpcxx.cpp

@brief	Shared LPC CTIMER count, frequency and trigger implementation.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include <stddef.h>
#include <stdint.h>
#include "timer_lpcxx.h"
#include "coredev/interrupt.h"

static LpcTimer_t *LpcTimerData(TimerDev_t * const pTimer);
static uint64_t LpcTimerCount(LpcTimer_t * const pDev);
static uint64_t LpcTimerTicks(uint32_t Freq, uint64_t nsPeriod);
static uint64_t LpcTimerTickNs(uint32_t Freq, uint64_t Ticks);
static void LpcTimerArm(LpcTimer_t * const pDev, int TrigNo);
static void LpcTimerResetCounter(LpcTimer_t * const pDev);

static void LpcTimerDisable(TimerDev_t * const pTimer);
static bool LpcTimerEnable(TimerDev_t * const pTimer);
static void LpcTimerReset(TimerDev_t * const pTimer);
static uint64_t LpcTimerGetTickCount(TimerDev_t * const pTimer);
static uint32_t LpcTimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq);
static int LpcTimerGetMaxTrigger(TimerDev_t * const pTimer);
static int LpcTimerFindAvailTrigger(TimerDev_t * const pTimer);
static void LpcTimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo);
static uint64_t LpcTimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
										   TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
										   void * const pContext);
static void LpcTimerDisableExtTrigger(TimerDev_t * const pTimer);
static bool LpcTimerEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense);

static LpcTimer_t *LpcTimerData(TimerDev_t * const pTimer)
{
	if (pTimer == NULL || pTimer->DevNo < 0 || pTimer->DevNo >= g_LpcTimerCount ||
		g_LpcTimer[pTimer->DevNo].pTimer != pTimer)
	{
		return NULL;
	}

	return &g_LpcTimer[pTimer->DevNo];
}

// 64 bit count. Called with interrupts disabled. A counter value lower than
// the last one read is a wrap, the match 3 interrupt reads it at least twice
// per counter cycle.
static uint64_t LpcTimerCount(LpcTimer_t * const pDev)
{
	TimerDev_t *t = pDev->pTimer;
	uint32_t tc = pDev->pReg->TC & pDev->CounterMask;

	if (tc < t->LastCount)
	{
		t->Rollover += ((uint64_t)pDev->CounterMask + 1U);
	}
	t->LastCount = tc;

	return t->Rollover + tc;
}

// Ticks of a period, rounded to the nearest tick and at least
// LPC_TIMER_TICKS_MIN
static uint64_t LpcTimerTicks(uint32_t Freq, uint64_t nsPeriod)
{
	uint64_t seconds = nsPeriod / 1000000000ULL;
	uint64_t fraction = ((nsPeriod % 1000000000ULL) * Freq + 500000000ULL) / 1000000000ULL;
	uint64_t ticks = INT64_MAX;

	if (seconds <= ((uint64_t)INT64_MAX - fraction) / Freq)
	{
		ticks = seconds * Freq + fraction;
	}

	return ticks < LPC_TIMER_TICKS_MIN ? LPC_TIMER_TICKS_MIN : ticks;
}

static uint64_t LpcTimerTickNs(uint32_t Freq, uint64_t Ticks)
{
	uint64_t seconds = Ticks / Freq;
	uint64_t fraction = ((Ticks % Freq) * 1000000000ULL + Freq / 2U) / Freq;

	if (seconds > (UINT64_MAX - fraction) / 1000000000ULL)
	{
		return UINT64_MAX;
	}
	return seconds * 1000000000ULL + fraction;
}

// Program the match register of a trigger. Called with interrupts disabled.
static void LpcTimerArm(LpcTimer_t * const pDev, int TrigNo)
{
	LpcTimerReg_t *reg = pDev->pReg;
	uint32_t mcr = LPC_TIMER_MATCH_INT << (TrigNo * LPC_TIMER_MCR_BITS);

	reg->MCR &= ~mcr;
	reg->IR = LPC_TIMER_MATCH_FLAG << TrigNo;
	*LpcTimerMatchReg(reg, (unsigned)TrigNo) = (uint32_t)pDev->Trig[TrigNo].Deadline & pDev->CounterMask;
	reg->MCR |= mcr;

	// A deadline that passed while the match register was written would only
	// match after a full counter cycle. It goes through the interrupt now.
	if ((reg->TCR & LPC_TIMER_ENABLE) && LpcTimerCount(pDev) >= pDev->Trig[TrigNo].Deadline)
	{
		NVIC_SetPendingIRQ(pDev->IrqNo);
	}
}

// Counter to 0 and active triggers restarted for a full period. Called with
// interrupts disabled.
static void LpcTimerResetCounter(LpcTimer_t * const pDev)
{
	LpcTimerReg_t *reg = pDev->pReg;
	uint32_t tcr = reg->TCR & LPC_TIMER_ENABLE;

	// The read back makes sure the reset is taken before it is released,
	// also through the async APB bridge.
	reg->TCR = LPC_TIMER_RESET;
	(void)reg->TCR;
	*LpcTimerMatchReg(reg, LPC_TIMER_WRAP_MR) = ((pDev->CounterMask >> 1) + 1U);
	reg->IR = LPC_TIMER_MATCH_FLAGS;
	NVIC_ClearPendingIRQ(pDev->IrqNo);

	pDev->pTimer->Rollover = 0;
	pDev->pTimer->LastCount = 0;
	pDev->Epoch++;

	for (int i = 0; i < LPC_TIMER_TRIG_MAX; i++)
	{
		if (pDev->Trig[i].bActive)
		{
			pDev->Trig[i].Deadline = pDev->Trig[i].Ticks;
			LpcTimerArm(pDev, i);
		}
	}
	reg->TCR = tcr;
}

void LpcTimerIrqHandler(LpcTimer_t * const pDev)
{
	TimerDev_t *t = pDev->pTimer;
	LpcTimerReg_t *reg = pDev->pReg;

	if (t == NULL)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	uint32_t ir = reg->IR & LPC_TIMER_MATCH_FLAGS;
	bool wrap = false;

	reg->IR = ir;

	if (ir & (LPC_TIMER_MATCH_FLAG << LPC_TIMER_WRAP_MR))
	{
		// Match 3 alternates between the counter middle and 0, the match at 0
		// is the wrap.
		wrap = *LpcTimerMatchReg(reg, LPC_TIMER_WRAP_MR) == 0U;
		*LpcTimerMatchReg(reg, LPC_TIMER_WRAP_MR) = (*LpcTimerMatchReg(reg, LPC_TIMER_WRAP_MR) +
										 ((pDev->CounterMask >> 1) + 1U)) & pDev->CounterMask;
		(void)LpcTimerCount(pDev);
	}

	uint32_t epoch = pDev->Epoch;

	EnableInterrupt(state);

	if (wrap && t->EvtHandler)
	{
		t->EvtHandler(t, TIMER_EVT_COUNTER_OVR);
	}

	for (int i = 0; i < LPC_TIMER_TRIG_MAX; i++)
	{
		state = DisableInterrupt();

		// A handler may have reset the counter or stopped the timer
		if (epoch != pDev->Epoch || (reg->TCR & LPC_TIMER_ENABLE) == 0)
		{
			EnableInterrupt(state);

			return;
		}

		LpcTimerTrig_t *tr = &pDev->Trig[i];
		uint64_t now = LpcTimerCount(pDev);

		// A match on the low counter bits of a deadline more than one counter
		// cycle away is not the trigger yet.
		if (tr->bActive == false || now < tr->Deadline)
		{
			EnableInterrupt(state);
			continue;
		}

		TimerTrig_t info = tr->Info;

		if (info.Type == TIMER_TRIG_TYPE_SINGLE)
		{
			LpcTimerDisableTrigger(t, i);
		}
		else
		{
			// Same phase, periods missed are reported once.
			tr->Deadline += ((now - tr->Deadline) / tr->Ticks + 1U) * tr->Ticks;
			LpcTimerArm(pDev, i);
		}

		EnableInterrupt(state);

		if (info.Handler)
		{
			info.Handler(t, i, info.pContext);
		}
		else if (t->EvtHandler)
		{
			t->EvtHandler(t, TIMER_EVT_TRIGGER(i));
		}
	}
}

// Counter stopped, count and trigger deadlines kept
static void LpcTimerDisable(TimerDev_t * const pTimer)
{
	LpcTimer_t *dev = LpcTimerData(pTimer);

	if (dev == NULL)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	dev->pReg->TCR &= ~LPC_TIMER_ENABLE;
	NVIC_DisableIRQ(dev->IrqNo);

	EnableInterrupt(state);
}

static bool LpcTimerEnable(TimerDev_t * const pTimer)
{
	LpcTimer_t *dev = LpcTimerData(pTimer);

	if (dev == NULL)
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	dev->pReg->TCR |= LPC_TIMER_ENABLE;
	NVIC_EnableIRQ(dev->IrqNo);

	EnableInterrupt(state);

	return true;
}

static void LpcTimerReset(TimerDev_t * const pTimer)
{
	LpcTimer_t *dev = LpcTimerData(pTimer);

	if (dev == NULL)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	LpcTimerResetCounter(dev);

	EnableInterrupt(state);
}

static uint64_t LpcTimerGetTickCount(TimerDev_t * const pTimer)
{
	LpcTimer_t *dev = LpcTimerData(pTimer);

	if (dev == NULL)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	uint64_t count = LpcTimerCount(dev);

	EnableInterrupt(state);

	return count;
}

// Closest frequency from the prescaler, 0 selects the base clock. The
// counter is reset, active triggers keep their period in time.
static uint32_t LpcTimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	LpcTimer_t *dev = LpcTimerData(pTimer);

	if (dev == NULL)
	{
		return 0;
	}

	uint32_t base = LpcTimerBaseFreq(dev);

	if (base == 0)
	{
		return 0;
	}

	uint64_t maxdiv = (uint64_t)dev->PrescalerMax + 1U;
	uint64_t div = Freq == 0 ? 1U : (uint64_t)base / Freq;

	// Public frequencies are integer Hz. Keep at least 1 Hz representable.
	if (maxdiv > base)
	{
		maxdiv = base;
	}
	if (div < 1U)
	{
		div = 1U;
	}
	if (div > maxdiv)
	{
		div = maxdiv;
	}
	if (Freq != 0 && div < maxdiv)
	{
		uint64_t product = (uint64_t)Freq * div;
		uint64_t error = product > base ? product - base : base - product;
		uint64_t nextproduct = (uint64_t)Freq * (div + 1U);
		uint64_t nexterror = nextproduct > base ? nextproduct - base : base - nextproduct;

		if (nexterror * div < error * (div + 1U))
		{
			div++;
		}
	}

	uint32_t freq = (uint32_t)(base / div);
	uint32_t state = DisableInterrupt();

	uint32_t running = dev->pReg->TCR & LPC_TIMER_ENABLE;

	dev->pReg->TCR = 0;
	dev->pReg->PR = (uint32_t)(div - 1U);
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + freq / 2U) / freq;

	for (int i = 0; i < LPC_TIMER_TRIG_MAX; i++)
	{
		if (dev->Trig[i].bActive)
		{
			dev->Trig[i].Ticks = LpcTimerTicks(freq, dev->Trig[i].Info.nsPeriod);
			dev->Trig[i].Info.nsPeriod = LpcTimerTickNs(freq, dev->Trig[i].Ticks);
		}
	}

	LpcTimerResetCounter(dev);
	dev->pReg->TCR = running;

	EnableInterrupt(state);

	return freq;
}

static int LpcTimerGetMaxTrigger(TimerDev_t * const pTimer)
{
	return LpcTimerData(pTimer) != NULL ? LPC_TIMER_TRIG_MAX : 0;
}

static int LpcTimerFindAvailTrigger(TimerDev_t * const pTimer)
{
	LpcTimer_t *dev = LpcTimerData(pTimer);

	if (dev == NULL)
	{
		return -1;
	}

	uint32_t state = DisableInterrupt();
	int idx = -1;

	for (int i = 0; i < LPC_TIMER_TRIG_MAX; i++)
	{
		if (dev->Trig[i].bActive == false)
		{
			idx = i;
			break;
		}
	}

	EnableInterrupt(state);

	return idx;
}

static void LpcTimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	LpcTimer_t *dev = LpcTimerData(pTimer);

	if (dev == NULL || TrigNo < 0 || TrigNo >= LPC_TIMER_TRIG_MAX)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	dev->Trig[TrigNo].bActive = false;
	dev->pReg->MCR &= ~(LPC_TIMER_MATCH_INT << (TrigNo * LPC_TIMER_MCR_BITS));
	dev->pReg->IR = LPC_TIMER_MATCH_FLAG << TrigNo;

	EnableInterrupt(state);
}

// The period is the closest number of ticks, at least
// LPC_TIMER_TICKS_MIN. Returns the actual period in nsec.
static uint64_t LpcTimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
										   TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
										   void * const pContext)
{
	LpcTimer_t *dev = LpcTimerData(pTimer);

	if (dev == NULL || TrigNo < 0 || TrigNo >= LPC_TIMER_TRIG_MAX || pTimer->Freq == 0 ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}

	uint64_t ticks = LpcTimerTicks(pTimer->Freq, nsPeriod);
	uint64_t actual = LpcTimerTickNs(pTimer->Freq, ticks);
	uint32_t state = DisableInterrupt();
	LpcTimerTrig_t *tr = &dev->Trig[TrigNo];

	tr->Info.Type = Type;
	tr->Info.nsPeriod = actual;
	tr->Info.Handler = Handler;
	tr->Info.pContext = pContext;
	tr->Ticks = ticks;
	tr->Deadline = LpcTimerCount(dev) + ticks;
	tr->bActive = true;
	LpcTimerArm(dev, TrigNo);

	EnableInterrupt(state);

	return actual;
}

// Capture inputs are not supported as external trigger
static void LpcTimerDisableExtTrigger(TimerDev_t * const pTimer)
{
	(void)pTimer;
}

static bool LpcTimerEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	(void)pTimer;
	(void)TrigDevNo;
	(void)Sense;

	return false;
}

/**
 * @brief	Timer initialization.
 *
 * Clock source TIMER_CLKSRC_DEFAULT only, no tick interrupt.
 */
bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == NULL || pCfg == NULL || pCfg->DevNo < 0 || pCfg->DevNo >= g_LpcTimerCount ||
		pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT || pCfg->bTickInt)
	{
		return false;
	}

	LpcTimer_t *dev = &g_LpcTimer[pCfg->DevNo];
	if (LpcTimerBaseFreq(dev) == 0)
	{
		return false;
	}
	uint32_t state = DisableInterrupt();

	// One CTIMER per timer object
	for (int i = 0; i < g_LpcTimerCount; i++)
	{
		if ((i != pCfg->DevNo && g_LpcTimer[i].pTimer == pTimer) ||
			(i == pCfg->DevNo && g_LpcTimer[i].pTimer != NULL && g_LpcTimer[i].pTimer != pTimer))
		{
			EnableInterrupt(state);

			return false;
		}
	}

	NVIC_DisableIRQ(dev->IrqNo);

	LpcTimerClockEnable(dev);

	for (int i = 0; i < LPC_TIMER_TRIG_MAX; i++)
	{
		dev->Trig[i].bActive = false;
	}

	dev->pTimer = pTimer;

	pTimer->DevNo = pCfg->DevNo;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->Disable = LpcTimerDisable;
	pTimer->Enable = LpcTimerEnable;
	pTimer->Reset = LpcTimerReset;
	pTimer->GetTickCount = LpcTimerGetTickCount;
	pTimer->SetFrequency = LpcTimerSetFrequency;
	pTimer->GetMaxTrigger = LpcTimerGetMaxTrigger;
	pTimer->FindAvailTrigger = LpcTimerFindAvailTrigger;
	pTimer->DisableTrigger = LpcTimerDisableTrigger;
	pTimer->EnableTrigger = LpcTimerEnableTrigger;
	pTimer->DisableExtTrigger = LpcTimerDisableExtTrigger;
	pTimer->EnableExtTrigger = LpcTimerEnableExtTrigger;

	// Timer mode, no match action other than the match 3 interrupt
	dev->pReg->TCR = 0;
	dev->pReg->CTCR = 0;
	dev->pReg->MCR = LPC_TIMER_MATCH_INT << (LPC_TIMER_WRAP_MR * LPC_TIMER_MCR_BITS);
	dev->pReg->CCR = 0;
	dev->pReg->EMR = 0;
	LpcTimerDisablePwm(dev->pReg);

	if (LpcTimerSetFrequency(pTimer, pCfg->Freq) == 0)
	{
		dev->pTimer = NULL;
		EnableInterrupt(state);

		return false;
	}

	int priority = pCfg->IntPrio;
	int maxprio = (1U << __NVIC_PRIO_BITS) - 1U;

	if (priority < 0)
	{
		priority = 0;
	}
	else if (priority > maxprio)
	{
		priority = maxprio;
	}
	NVIC_ClearPendingIRQ(dev->IrqNo);
	NVIC_SetPriority(dev->IrqNo, (uint32_t)priority);
	NVIC_EnableIRQ(dev->IrqNo);
	dev->pReg->TCR = LPC_TIMER_ENABLE;

	EnableInterrupt(state);

	return true;
}

int TimerGetLowFreqDevCount(void)
{
	return 0;
}

int TimerGetHighFreqDevCount(void)
{
	return g_LpcTimerCount;
}

int TimerGetHighFreqDevNo(void)
{
	return 0;
}

