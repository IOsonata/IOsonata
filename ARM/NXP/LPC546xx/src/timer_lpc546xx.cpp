/**-------------------------------------------------------------------------
@file	timer_lpc546xx.cpp

@brief	LPC546xx CTIMER implementation

		Five 32 bit CTIMERs. DevNo is the virtual timer number, lower power
		first:
			0 - CTIMER3, 1 - CTIMER4 : async APB bridge on the 12 MHz FRO
			2 - CTIMER0, 3 - CTIMER1, 4 - CTIMER2 : system clock

		Match registers 0 to 2 are the 3 triggers. Match register 3 interrupts
		at every half counter cycle so that a counter wrap is never missed,
		the 64 bit count is the wrap count plus the counter. Trigger deadlines
		are 64 bit counts, a period may be longer than a counter cycle. SysTick
		is not used here, it stays with the application or RTOS.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
#include <stdbool.h>

#include "LPC546xx.h"

#include "coredev/timer.h"
#include "coredev/interrupt.h"

#define LPC546XX_TIMER_TRIG_MAX		3			// Match 0 to 2
#define LPC546XX_TIMER_WRAP_MR		3			// Match 3, half cycle wrap check
#define LPC546XX_TIMER_HALF_CYCLE	0x80000000UL
#define LPC546XX_TIMER_CYCLE		0x100000000ULL
#define LPC546XX_TIMER_TICKS_MIN	4ULL		// Shortest trigger period in ticks
#define LPC546XX_TIMER_PR_MAX		0xFFFFFFFFULL
#define LPC546XX_TIMER_IR_ALL		0xFFU		// MR0INT to MR3INT, CR0INT to CR3INT
#define LPC546XX_TIMER_MCR_BITS		3			// MRnI, MRnR, MRnS per match register
#define LPC546XX_ASYNCAPB_FRO12M	1U			// ASYNCAPBCLKSELA fro_12m
#define LPC546XX_TIMER_AHB_IDX		1			// AHBCLKCTRL and PRESETCTRL of CTIMER0 to 2

typedef struct {
	TimerTrig_t Info;				//!< Type, actual period, handler and context
	uint64_t Deadline;				//!< Next trigger count
	uint64_t Ticks;					//!< Period in ticks
	bool bActive;					//!< Trigger enabled
} Lpc546xxTimerTrig_t;

typedef struct {
	CTIMER_Type *pReg;				//!< CTIMER registers
	IRQn_Type IrqNo;				//!< Interrupt number
	bool bAsync;					//!< On the async APB bridge, else on the AHB clock
	uint32_t ClkMask;				//!< Clock enable bit
	uint32_t RstMask;				//!< Reset bit
	TimerDev_t *pTimer;				//!< Timer using this CTIMER, NULL when free
	uint32_t Epoch;					//!< Incremented on counter reset
	Lpc546xxTimerTrig_t Trig[LPC546XX_TIMER_TRIG_MAX];
} Lpc546xxTimer_t;

static Lpc546xxTimer_t s_Lpc546xxTimer[] = {
	{ .pReg = CTIMER3, .IrqNo = CTIMER3_IRQn, .bAsync = true,
	  .ClkMask = ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER3_MASK, .RstMask = ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER3_MASK, },
	{ .pReg = CTIMER4, .IrqNo = CTIMER4_IRQn, .bAsync = true,
	  .ClkMask = ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER4_MASK, .RstMask = ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER4_MASK, },
	{ .pReg = CTIMER0, .IrqNo = CTIMER0_IRQn, .bAsync = false,
	  .ClkMask = SYSCON_AHBCLKCTRL_CTIMER0_MASK, .RstMask = SYSCON_PRESETCTRL_CTIMER0_RST_MASK, },
	{ .pReg = CTIMER1, .IrqNo = CTIMER1_IRQn, .bAsync = false,
	  .ClkMask = SYSCON_AHBCLKCTRL_CTIMER1_MASK, .RstMask = SYSCON_PRESETCTRL_CTIMER1_RST_MASK, },
	{ .pReg = CTIMER2, .IrqNo = CTIMER2_IRQn, .bAsync = false,
	  .ClkMask = SYSCON_AHBCLKCTRL_CTIMER2_MASK, .RstMask = SYSCON_PRESETCTRL_CTIMER2_RST_MASK, },
};

#define LPC546XX_TIMER_CNT			((int)(sizeof(s_Lpc546xxTimer) / sizeof(Lpc546xxTimer_t)))

static Lpc546xxTimer_t *Lpc546xxTimerData(TimerDev_t * const pTimer);
static uint32_t Lpc546xxTimerBaseFreq(const Lpc546xxTimer_t *pDev);
static uint64_t Lpc546xxTimerCount(Lpc546xxTimer_t * const pDev);
static uint64_t Lpc546xxTimerTicks(uint32_t Freq, uint64_t nsPeriod);
static uint64_t Lpc546xxTimerTickNs(uint32_t Freq, uint64_t Ticks);
static void Lpc546xxTimerArm(Lpc546xxTimer_t * const pDev, int TrigNo);
static void Lpc546xxTimerResetCounter(Lpc546xxTimer_t * const pDev);
static void Lpc546xxTimerIrqHandler(Lpc546xxTimer_t * const pDev);

static void Lpc546xxTimerDisable(TimerDev_t * const pTimer);
static bool Lpc546xxTimerEnable(TimerDev_t * const pTimer);
static void Lpc546xxTimerReset(TimerDev_t * const pTimer);
static uint64_t Lpc546xxTimerGetTickCount(TimerDev_t * const pTimer);
static uint32_t Lpc546xxTimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq);
static int Lpc546xxTimerGetMaxTrigger(TimerDev_t * const pTimer);
static int Lpc546xxTimerFindAvailTrigger(TimerDev_t * const pTimer);
static void Lpc546xxTimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo);
static uint64_t Lpc546xxTimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
										   TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
										   void * const pContext);
static void Lpc546xxTimerDisableExtTrigger(TimerDev_t * const pTimer);
static bool Lpc546xxTimerEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense);

static Lpc546xxTimer_t *Lpc546xxTimerData(TimerDev_t * const pTimer)
{
	if (pTimer == NULL || pTimer->DevNo < 0 || pTimer->DevNo >= LPC546XX_TIMER_CNT ||
		s_Lpc546xxTimer[pTimer->DevNo].pTimer != pTimer)
	{
		return NULL;
	}

	return &s_Lpc546xxTimer[pTimer->DevNo];
}

static uint32_t Lpc546xxTimerBaseFreq(const Lpc546xxTimer_t *pDev)
{
	return pDev->bAsync ? CLK_FRO_12MHZ : SystemCoreClock;
}

// 64 bit count. Called with interrupts disabled. A counter value lower than
// the last one read is a wrap, the match 3 interrupt reads it at least twice
// per counter cycle.
static uint64_t Lpc546xxTimerCount(Lpc546xxTimer_t * const pDev)
{
	TimerDev_t *t = pDev->pTimer;
	uint32_t tc = pDev->pReg->TC;

	if (tc < t->LastCount)
	{
		t->Rollover += LPC546XX_TIMER_CYCLE;
	}
	t->LastCount = tc;

	return t->Rollover + tc;
}

// Ticks of a period, rounded to the nearest tick and at least
// LPC546XX_TIMER_TICKS_MIN
static uint64_t Lpc546xxTimerTicks(uint32_t Freq, uint64_t nsPeriod)
{
	uint64_t ticks = (nsPeriod / 1000000000ULL) * Freq +
					 ((nsPeriod % 1000000000ULL) * Freq + 500000000ULL) / 1000000000ULL;

	return ticks < LPC546XX_TIMER_TICKS_MIN ? LPC546XX_TIMER_TICKS_MIN : ticks;
}

static uint64_t Lpc546xxTimerTickNs(uint32_t Freq, uint64_t Ticks)
{
	return (Ticks / Freq) * 1000000000ULL + ((Ticks % Freq) * 1000000000ULL + Freq / 2U) / Freq;
}

// Program the match register of a trigger. Called with interrupts disabled.
static void Lpc546xxTimerArm(Lpc546xxTimer_t * const pDev, int TrigNo)
{
	CTIMER_Type *reg = pDev->pReg;
	uint32_t mcr = CTIMER_MCR_MR0I_MASK << (TrigNo * LPC546XX_TIMER_MCR_BITS);

	reg->MCR &= ~mcr;
	reg->IR = CTIMER_IR_MR0INT_MASK << TrigNo;
	reg->MR[TrigNo] = (uint32_t)pDev->Trig[TrigNo].Deadline;
	reg->MCR |= mcr;

	// A deadline that passed while the match register was written would only
	// match after a full counter cycle. It goes through the interrupt now.
	if ((reg->TCR & CTIMER_TCR_CEN_MASK) && Lpc546xxTimerCount(pDev) >= pDev->Trig[TrigNo].Deadline)
	{
		NVIC_SetPendingIRQ(pDev->IrqNo);
	}
}

// Counter to 0 and active triggers restarted for a full period. Called with
// interrupts disabled.
static void Lpc546xxTimerResetCounter(Lpc546xxTimer_t * const pDev)
{
	CTIMER_Type *reg = pDev->pReg;
	uint32_t tcr = reg->TCR & CTIMER_TCR_CEN_MASK;

	// The read back makes sure the reset is taken before it is released,
	// also through the async APB bridge.
	reg->TCR = CTIMER_TCR_CRST_MASK;
	(void)reg->TCR;
	reg->MR[LPC546XX_TIMER_WRAP_MR] = LPC546XX_TIMER_HALF_CYCLE;
	reg->IR = LPC546XX_TIMER_IR_ALL;
	reg->TCR = tcr;
	NVIC_ClearPendingIRQ(pDev->IrqNo);

	pDev->pTimer->Rollover = 0;
	pDev->pTimer->LastCount = 0;
	pDev->Epoch++;

	for (int i = 0; i < LPC546XX_TIMER_TRIG_MAX; i++)
	{
		if (pDev->Trig[i].bActive)
		{
			pDev->Trig[i].Deadline = pDev->Trig[i].Ticks;
			Lpc546xxTimerArm(pDev, i);
		}
	}
}

static void Lpc546xxTimerIrqHandler(Lpc546xxTimer_t * const pDev)
{
	TimerDev_t *t = pDev->pTimer;
	CTIMER_Type *reg = pDev->pReg;

	if (t == NULL)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	uint32_t ir = reg->IR;
	bool wrap = false;

	reg->IR = ir;

	if (ir & (CTIMER_IR_MR0INT_MASK << LPC546XX_TIMER_WRAP_MR))
	{
		// Match 3 alternates between the counter middle and 0, the match at 0
		// is the wrap.
		wrap = reg->MR[LPC546XX_TIMER_WRAP_MR] == 0U;
		reg->MR[LPC546XX_TIMER_WRAP_MR] += LPC546XX_TIMER_HALF_CYCLE;
		(void)Lpc546xxTimerCount(pDev);
	}

	uint32_t epoch = pDev->Epoch;

	EnableInterrupt(state);

	if (wrap && t->EvtHandler)
	{
		t->EvtHandler(t, TIMER_EVT_COUNTER_OVR);
	}

	for (int i = 0; i < LPC546XX_TIMER_TRIG_MAX; i++)
	{
		state = DisableInterrupt();

		// A handler may have reset the counter or stopped the timer
		if (epoch != pDev->Epoch || (reg->TCR & CTIMER_TCR_CEN_MASK) == 0)
		{
			EnableInterrupt(state);

			return;
		}

		Lpc546xxTimerTrig_t *tr = &pDev->Trig[i];
		uint64_t now = Lpc546xxTimerCount(pDev);

		// A match on the low 32 bits of a deadline more than one counter
		// cycle away is not the trigger yet.
		if (tr->bActive == false || now < tr->Deadline)
		{
			EnableInterrupt(state);
			continue;
		}

		TimerTrig_t info = tr->Info;

		if (info.Type == TIMER_TRIG_TYPE_SINGLE)
		{
			Lpc546xxTimerDisableTrigger(t, i);
		}
		else
		{
			// Same phase, periods missed are reported once.
			tr->Deadline += ((now - tr->Deadline) / tr->Ticks + 1U) * tr->Ticks;
			Lpc546xxTimerArm(pDev, i);
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
static void Lpc546xxTimerDisable(TimerDev_t * const pTimer)
{
	Lpc546xxTimer_t *dev = Lpc546xxTimerData(pTimer);

	if (dev == NULL)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	dev->pReg->TCR &= ~CTIMER_TCR_CEN_MASK;
	NVIC_DisableIRQ(dev->IrqNo);

	EnableInterrupt(state);
}

static bool Lpc546xxTimerEnable(TimerDev_t * const pTimer)
{
	Lpc546xxTimer_t *dev = Lpc546xxTimerData(pTimer);

	if (dev == NULL)
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	dev->pReg->TCR |= CTIMER_TCR_CEN_MASK;
	NVIC_EnableIRQ(dev->IrqNo);

	EnableInterrupt(state);

	return true;
}

static void Lpc546xxTimerReset(TimerDev_t * const pTimer)
{
	Lpc546xxTimer_t *dev = Lpc546xxTimerData(pTimer);

	if (dev == NULL)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	Lpc546xxTimerResetCounter(dev);

	EnableInterrupt(state);
}

static uint64_t Lpc546xxTimerGetTickCount(TimerDev_t * const pTimer)
{
	Lpc546xxTimer_t *dev = Lpc546xxTimerData(pTimer);

	if (dev == NULL)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	uint64_t count = Lpc546xxTimerCount(dev);

	EnableInterrupt(state);

	return count;
}

// Closest frequency from the prescaler, 0 selects the base clock. The
// counter is reset, active triggers keep their period in time.
static uint32_t Lpc546xxTimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	Lpc546xxTimer_t *dev = Lpc546xxTimerData(pTimer);

	if (dev == NULL)
	{
		return 0;
	}

	uint32_t base = Lpc546xxTimerBaseFreq(dev);

	if (base == 0)
	{
		return 0;
	}

	uint64_t div = Freq == 0 ? 1U : ((uint64_t)base + Freq / 2U) / Freq;

	if (div < 1U)
	{
		div = 1U;
	}
	else if (div > LPC546XX_TIMER_PR_MAX + 1U)
	{
		div = LPC546XX_TIMER_PR_MAX + 1U;
	}

	uint32_t freq = (uint32_t)(base / div);
	uint32_t state = DisableInterrupt();

	dev->pReg->PR = (uint32_t)(div - 1U);
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + freq / 2U) / freq;

	for (int i = 0; i < LPC546XX_TIMER_TRIG_MAX; i++)
	{
		if (dev->Trig[i].bActive)
		{
			dev->Trig[i].Ticks = Lpc546xxTimerTicks(freq, dev->Trig[i].Info.nsPeriod);
			dev->Trig[i].Info.nsPeriod = Lpc546xxTimerTickNs(freq, dev->Trig[i].Ticks);
		}
	}

	Lpc546xxTimerResetCounter(dev);

	EnableInterrupt(state);

	return freq;
}

static int Lpc546xxTimerGetMaxTrigger(TimerDev_t * const pTimer)
{
	return Lpc546xxTimerData(pTimer) != NULL ? LPC546XX_TIMER_TRIG_MAX : 0;
}

static int Lpc546xxTimerFindAvailTrigger(TimerDev_t * const pTimer)
{
	Lpc546xxTimer_t *dev = Lpc546xxTimerData(pTimer);

	if (dev == NULL)
	{
		return -1;
	}

	uint32_t state = DisableInterrupt();
	int idx = -1;

	for (int i = 0; i < LPC546XX_TIMER_TRIG_MAX; i++)
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

static void Lpc546xxTimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	Lpc546xxTimer_t *dev = Lpc546xxTimerData(pTimer);

	if (dev == NULL || TrigNo < 0 || TrigNo >= LPC546XX_TIMER_TRIG_MAX)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	dev->Trig[TrigNo].bActive = false;
	dev->pReg->MCR &= ~(CTIMER_MCR_MR0I_MASK << (TrigNo * LPC546XX_TIMER_MCR_BITS));
	dev->pReg->IR = CTIMER_IR_MR0INT_MASK << TrigNo;

	EnableInterrupt(state);
}

// The period is the closest number of ticks, at least
// LPC546XX_TIMER_TICKS_MIN. Returns the actual period in nsec.
static uint64_t Lpc546xxTimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
										   TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
										   void * const pContext)
{
	Lpc546xxTimer_t *dev = Lpc546xxTimerData(pTimer);

	if (dev == NULL || TrigNo < 0 || TrigNo >= LPC546XX_TIMER_TRIG_MAX || pTimer->Freq == 0 ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}

	uint64_t ticks = Lpc546xxTimerTicks(pTimer->Freq, nsPeriod);
	uint64_t actual = Lpc546xxTimerTickNs(pTimer->Freq, ticks);
	uint32_t state = DisableInterrupt();
	Lpc546xxTimerTrig_t *tr = &dev->Trig[TrigNo];

	tr->Info.Type = Type;
	tr->Info.nsPeriod = actual;
	tr->Info.Handler = Handler;
	tr->Info.pContext = pContext;
	tr->Ticks = ticks;
	tr->Deadline = Lpc546xxTimerCount(dev) + ticks;
	tr->bActive = true;
	Lpc546xxTimerArm(dev, TrigNo);

	EnableInterrupt(state);

	return actual;
}

// Capture inputs are not supported as external trigger
static void Lpc546xxTimerDisableExtTrigger(TimerDev_t * const pTimer)
{
	(void)pTimer;
}

static bool Lpc546xxTimerEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
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
	if (pTimer == NULL || pCfg == NULL || pCfg->DevNo < 0 || pCfg->DevNo >= LPC546XX_TIMER_CNT ||
		pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT || pCfg->bTickInt ||
		pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS))
	{
		return false;
	}

	Lpc546xxTimer_t *dev = &s_Lpc546xxTimer[pCfg->DevNo];
	uint32_t state = DisableInterrupt();

	// One CTIMER per timer object
	for (int i = 0; i < LPC546XX_TIMER_CNT; i++)
	{
		if ((i != pCfg->DevNo && s_Lpc546xxTimer[i].pTimer == pTimer) ||
			(i == pCfg->DevNo && s_Lpc546xxTimer[i].pTimer != NULL && s_Lpc546xxTimer[i].pTimer != pTimer))
		{
			EnableInterrupt(state);

			return false;
		}
	}

	NVIC_DisableIRQ(dev->IrqNo);

	// Clock on and peripheral reset. CTIMER3 and 4 are on the async APB
	// bridge, which runs from the 12 MHz FRO.
	if (dev->bAsync)
	{
		SYSCON->ASYNCAPBCTRL = SYSCON_ASYNCAPBCTRL_ENABLE_MASK;
		ASYNC_SYSCON->ASYNCAPBCLKSELA = ASYNC_SYSCON_ASYNCAPBCLKSELA_SEL(LPC546XX_ASYNCAPB_FRO12M);
		ASYNC_SYSCON->ASYNCAPBCLKCTRLSET = dev->ClkMask;
		ASYNC_SYSCON->ASYNCPRESETCTRLSET = dev->RstMask;
		ASYNC_SYSCON->ASYNCPRESETCTRLCLR = dev->RstMask;
	}
	else
	{
		SYSCON->AHBCLKCTRLSET[LPC546XX_TIMER_AHB_IDX] = dev->ClkMask;
		SYSCON->PRESETCTRLSET[LPC546XX_TIMER_AHB_IDX] = dev->RstMask;
		SYSCON->PRESETCTRLCLR[LPC546XX_TIMER_AHB_IDX] = dev->RstMask;
	}

	for (int i = 0; i < LPC546XX_TIMER_TRIG_MAX; i++)
	{
		dev->Trig[i].bActive = false;
	}

	dev->pTimer = pTimer;

	pTimer->DevNo = pCfg->DevNo;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->Disable = Lpc546xxTimerDisable;
	pTimer->Enable = Lpc546xxTimerEnable;
	pTimer->Reset = Lpc546xxTimerReset;
	pTimer->GetTickCount = Lpc546xxTimerGetTickCount;
	pTimer->SetFrequency = Lpc546xxTimerSetFrequency;
	pTimer->GetMaxTrigger = Lpc546xxTimerGetMaxTrigger;
	pTimer->FindAvailTrigger = Lpc546xxTimerFindAvailTrigger;
	pTimer->DisableTrigger = Lpc546xxTimerDisableTrigger;
	pTimer->EnableTrigger = Lpc546xxTimerEnableTrigger;
	pTimer->DisableExtTrigger = Lpc546xxTimerDisableExtTrigger;
	pTimer->EnableExtTrigger = Lpc546xxTimerEnableExtTrigger;

	// Timer mode, no match action other than the match 3 interrupt
	dev->pReg->TCR = 0;
	dev->pReg->CTCR = 0;
	dev->pReg->MCR = CTIMER_MCR_MR0I_MASK << (LPC546XX_TIMER_WRAP_MR * LPC546XX_TIMER_MCR_BITS);
	dev->pReg->CCR = 0;
	dev->pReg->EMR = 0;
	dev->pReg->PWMC = 0;

	if (Lpc546xxTimerSetFrequency(pTimer, pCfg->Freq) == 0)
	{
		dev->pTimer = NULL;
		EnableInterrupt(state);

		return false;
	}

	NVIC_ClearPendingIRQ(dev->IrqNo);
	NVIC_SetPriority(dev->IrqNo, pCfg->IntPrio);
	NVIC_EnableIRQ(dev->IrqNo);
	dev->pReg->TCR = CTIMER_TCR_CEN_MASK;

	EnableInterrupt(state);

	return true;
}

int TimerGetLowFreqDevCount(void)
{
	return 0;
}

int TimerGetHighFreqDevCount(void)
{
	return LPC546XX_TIMER_CNT;
}

int TimerGetHighFreqDevNo(void)
{
	return 0;
}

extern "C" void CTIMER0_IRQHandler(void)
{
	Lpc546xxTimerIrqHandler(&s_Lpc546xxTimer[2]);
}

extern "C" void CTIMER1_IRQHandler(void)
{
	Lpc546xxTimerIrqHandler(&s_Lpc546xxTimer[3]);
}

extern "C" void CTIMER2_IRQHandler(void)
{
	Lpc546xxTimerIrqHandler(&s_Lpc546xxTimer[4]);
}

extern "C" void CTIMER3_IRQHandler(void)
{
	Lpc546xxTimerIrqHandler(&s_Lpc546xxTimer[0]);
}

extern "C" void CTIMER4_IRQHandler(void)
{
	Lpc546xxTimerIrqHandler(&s_Lpc546xxTimer[1]);
}
