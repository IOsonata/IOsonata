/**-------------------------------------------------------------------------
@file	timer_systick.cpp

@brief	ARM Cortex-M SysTick timer implementation

SysTick runs on the processor clock. The timer frequency is the core clock
given by SystemCoreClockGet() and the tick count is in core clock cycles.

The counter always runs with its interrupt on. Without a trigger the reload
is the longest one, 2^24 cycles, and the interrupt only extends the tick
count. The trigger sets the reload to its period, the closest number of
cycles from 2 to 2^24, so a continuous trigger keeps its period from the
hardware reload. A single shot trigger fires once, the reload goes back to
2^24 cycles at the next wrap and the tick count goes on.

Enabling a trigger and Reset restart the counter. The few cycles between the
count read and the restart are not in the tick count.

COUNTFLAG records a wrap. A CTRL read clears it, so CTRL is read only with
interrupts masked, by GetTickCount and the interrupt handler, each adding the
wrap to the tick count. The other CTRL accesses are writes, which keep it.

@author	Hoang Nguyen Hoan
@date	Mar. 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <string.h>

#include "systick.h"
#include "coredev/timer.h"
#include "coredev/system_core_clock.h"
#include "timer_systick.h"

// Longest period, LOAD 0xFFFFFF
#define SYSTICK_PERIOD_MAX		0x1000000UL
// Shortest period, LOAD 1. LOAD 0 stops the counter
#define SYSTICK_PERIOD_MIN		2UL

typedef struct {
	volatile uint32_t CTRL;
	volatile uint32_t LOAD;
	volatile uint32_t VAL;
	volatile uint32_t CALIB;
} SysTickReg_t;

#define SYSTICK_REG				((SysTickReg_t *)SYSTICK_BASE)
#define SYSTICK_ICSR			(*(volatile uint32_t *)SCB_ICSR_ADDR)

// CTRL values, processor clock, interrupt on, stopped or running
#define SYSTICK_CTRL_STOPPED	(SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT)
#define SYSTICK_CTRL_RUNNING	(SYSTICK_CTRL_STOPPED | SYSTICK_CTRL_ENABLE)

// One SysTick per Cortex-M core
static SysTick_TimerData_t s_SysTickData;

// Tick count, interrupts masked. A wrap found in COUNTFLAG, which the read
// clears, is added to Rollover here.
static inline __attribute__((always_inline)) uint64_t SystickCount(TimerDev_t * const pTimer)
{
	SysTick_TimerData_t &d = s_SysTickData;
	uint32_t val = SYSTICK_REG->VAL;

	if (SYSTICK_REG->CTRL & SYSTICK_CTRL_COUNTFLAG)
	{
		// Wrapped before or after the VAL read, VAL is read again
		pTimer->Rollover += d.CurPeriod;
		d.CurPeriod = d.NextPeriod;
		d.Wraps++;
		val = SYSTICK_REG->VAL;
		if (val == 0)
		{
			// Still the last cycle of the period that ended, the reload is
			// on the next one
			return pTimer->Rollover - 1;
		}
	}

	return pTimer->Rollover + (d.CurPeriod - val - 1);
}

// Counter restarted with a new period, from tick count Count, interrupts
// masked. A wrap and its interrupt still pending are dropped.
static inline __attribute__((always_inline)) void SystickRestart(TimerDev_t * const pTimer, uint64_t Count, uint32_t Period)
{
	SysTick_TimerData_t &d = s_SysTickData;

	SYSTICK_REG->LOAD = Period - 1;
	// Clears COUNTFLAG, LOAD is reloaded on the next cycle
	SYSTICK_REG->VAL = 0;
	SYSTICK_ICSR = SCB_ICSR_PENDSTCLR;
	pTimer->Rollover = Count;
	d.CurPeriod = Period;
	d.NextPeriod = Period;
	d.Wraps = 0;
}

// Timer frequency and conversion values from the core clock
static void SystickClock(TimerDev_t * const pTimer, uint32_t ClkHz)
{
	SysTick_TimerData_t &d = s_SysTickData;

	pTimer->Freq = ClkHz;
	pTimer->nsPeriod = (1000000000ULL + (ClkHz >> 1)) / ClkHz;

	// Below nsMax, nsPeriod * ClkHz and nsPeriod * CycQ32 fit in 64 bits
	d.nsMax = (SYSTICK_PERIOD_MAX * 1000000000ULL + ClkHz - 1) / ClkHz;
	d.CycQ32 = ((uint64_t)ClkHz << 32) / 1000000000ULL;
	d.NsHi = 1000000000UL / ClkHz;
	d.NsLo = (uint32_t)(((uint64_t)(1000000000UL % ClkHz) << 32) / ClkHz);
}

// Priority level as for the NVIC interrupts, 0 the highest, past the lowest
// one gives the lowest one
static void SystickSetPrio(int IntPrio)
{
	volatile uint32_t *shpr3 = (volatile uint32_t *)SCB_SHPR3_ADDR;

	// The bits not implemented in the priority field read 0
	*shpr3 |= 0xFF000000UL;

	uint32_t impl = *shpr3 >> 24;
	uint32_t shift = __builtin_ctz(impl);
	uint32_t lowest = impl >> shift;
	uint32_t prio = IntPrio < 0 ? 0 : (uint32_t)IntPrio;

	if (prio > lowest)
	{
		prio = lowest;
	}

	SysTickSetPriority((uint8_t)(prio << shift));
}

static bool SystickEnable(TimerDev_t * const pTimer)
{
	(void)pTimer;

	SYSTICK_REG->CTRL = SYSTICK_CTRL_RUNNING;

	return true;
}

static void SystickDisable(TimerDev_t * const pTimer)
{
	(void)pTimer;

	SYSTICK_REG->CTRL = SYSTICK_CTRL_STOPPED;
}

// The tick count restarts from 0 and the trigger restarts its period
static void SystickReset(TimerDev_t * const pTimer)
{
	SysTick_TimerData_t &d = s_SysTickData;
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	SystickRestart(pTimer, 0, d.NextPeriod);
	__set_PRIMASK(primask);

	pTimer->LastCount = 0;
}

// The frequency is always the core clock, read again here. An enabled
// trigger keeps its number of cycles.
static uint32_t SystickSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	(void)Freq;

	uint32_t clk = SystemCoreClockGet();

	if (clk != 0)
	{
		SystickClock(pTimer, clk);
	}

	return pTimer->Freq;
}

static uint64_t SystickGetTickCount(TimerDev_t * const pTimer)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	uint64_t count = SystickCount(pTimer);
	__set_PRIMASK(primask);

	return count;
}

static int SystickGetMaxTrigger(TimerDev_t * const pTimer)
{
	(void)pTimer;

	return SYSTICK_TRIG_MAXCNT;
}

static int SystickFindAvailTrig(TimerDev_t * const pTimer)
{
	(void)pTimer;

	return s_SysTickData.Trigger[0].nsPeriod == 0 ? 0 : -1;
}

// TrigNo 0 only. The period starts when the counter restarts with it.
static uint64_t SystickEnableTrigger(TimerDev_t * const pTimer, int TrigNo,
									 uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
									 TimerTrigEvtHandler_t const Handler,
									 void * const pContext)
{
	SysTick_TimerData_t &d = s_SysTickData;
	TimerTrig_t &trig = d.Trigger[0];

	if (TrigNo != 0 || nsPeriod == 0 ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}

	// Never replace an active trigger
	if (trig.nsPeriod != 0)
	{
		return 0;
	}

	// Closest number of cycles
	uint32_t period = SYSTICK_PERIOD_MAX;

	if (nsPeriod < d.nsMax)
	{
		// Estimate from the cycles per nsec, not above the quotient, then the
		// quotient itself, without a 64 bit division
		uint64_t num = nsPeriod * pTimer->Freq + 500000000ULL;
		uint32_t cycles = (uint32_t)((nsPeriod * d.CycQ32) >> 32);
		uint64_t rem = num - (uint64_t)cycles * 1000000000ULL;

		while (rem >= 1000000000ULL)
		{
			cycles++;
			rem -= 1000000000ULL;
		}

		if (cycles < SYSTICK_PERIOD_MIN)
		{
			period = SYSTICK_PERIOD_MIN;
		}
		else if (cycles < SYSTICK_PERIOD_MAX)
		{
			period = cycles;
		}
	}

	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	SystickRestart(pTimer, SystickCount(pTimer), period);

	// Real period, nearest nsec
	uint64_t ns = (uint64_t)period * d.NsHi + (((uint64_t)period * d.NsLo + 0x80000000UL) >> 32);

	trig.Type = Type;
	trig.Handler = Handler;
	trig.pContext = pContext;
	trig.nsPeriod = ns;
	__set_PRIMASK(primask);

	return ns;
}

// The period in progress ends, the next ones are 2^24 cycles
static void SystickDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	(void)pTimer;

	SysTick_TimerData_t &d = s_SysTickData;
	TimerTrig_t &trig = d.Trigger[0];

	if (TrigNo != 0)
	{
		return;
	}

	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	SYSTICK_REG->LOAD = SYSTICK_PERIOD_MAX - 1;
	d.NextPeriod = SYSTICK_PERIOD_MAX;
	trig.Type = TIMER_TRIG_TYPE_SINGLE;
	trig.Handler = nullptr;
	trig.pContext = nullptr;
	trig.nsPeriod = 0;
	__set_PRIMASK(primask);
}

// SysTick has no external trigger input
static void SystickDisableExtTrig(TimerDev_t * const pTimer)
{
	(void)pTimer;
}

static bool SystickEnableExtTrig(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	(void)pTimer;
	(void)TrigDevNo;
	(void)Sense;

	return false;
}

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == nullptr || pCfg == nullptr || pCfg->bTickInt)
	{
		return false;
	}

	uint32_t clk = SystemCoreClockGet();

	if (clk == 0)
	{
		return false;
	}

	SysTick_TimerData_t &d = s_SysTickData;

	// Stopped, interrupt off
	SYSTICK_REG->CTRL = 0;
	SYSTICK_ICSR = SCB_ICSR_PENDSTCLR;

	memset(&d, 0, sizeof(d));
	d.pTimer = pTimer;
	d.Trigger[0].Type = TIMER_TRIG_TYPE_SINGLE;

	pTimer->DevNo = 0;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->pObj = &d;

	pTimer->Disable = SystickDisable;
	pTimer->Enable = SystickEnable;
	pTimer->Reset = SystickReset;
	pTimer->GetTickCount = SystickGetTickCount;
	pTimer->SetFrequency = SystickSetFrequency;
	pTimer->GetMaxTrigger = SystickGetMaxTrigger;
	pTimer->FindAvailTrigger = SystickFindAvailTrig;
	pTimer->DisableTrigger = SystickDisableTrigger;
	pTimer->EnableTrigger = SystickEnableTrigger;
	pTimer->GetTaskEndpoint = nullptr;
	pTimer->ExecuteTask = nullptr;
	pTimer->DisableExtTrigger = SystickDisableExtTrig;
	pTimer->EnableExtTrigger = SystickEnableExtTrig;

	SystickClock(pTimer, clk);
	SystickSetPrio(pCfg->IntPrio);

	// Longest period, reloaded on the next cycle
	SYSTICK_REG->LOAD = SYSTICK_PERIOD_MAX - 1;
	SYSTICK_REG->VAL = 0;
	d.CurPeriod = SYSTICK_PERIOD_MAX;
	d.NextPeriod = SYSTICK_PERIOD_MAX;

	return SystickEnable(pTimer);
}

// Each wrap extends the tick count, unless GetTickCount did it already. The
// trigger is called once per interrupt with a wrap counted since the last one:
// a restart in between, from an interrupt of higher priority, drops it.
extern "C" void SysTick_Handler(void)
{
	SysTick_TimerData_t &d = s_SysTickData;
	TimerDev_t * const pTimer = d.pTimer;
	TimerTrig_t &trig = d.Trigger[0];

	__disable_irq();

	uint32_t wraps = d.Wraps;

	if (SYSTICK_REG->CTRL & SYSTICK_CTRL_COUNTFLAG)
	{
		pTimer->Rollover += d.CurPeriod;
		d.CurPeriod = d.NextPeriod;
		wraps++;
	}
	d.Wraps = 0;

	if (wraps == 0 || trig.nsPeriod == 0)
	{
		__enable_irq();

		return;
	}

	TimerTrigEvtHandler_t handler = trig.Handler;
	void *ctx = trig.pContext;

	if (trig.Type == TIMER_TRIG_TYPE_SINGLE)
	{
		// Free again. The period in progress ends, the next ones are 2^24
		// cycles
		SYSTICK_REG->LOAD = SYSTICK_PERIOD_MAX - 1;
		d.NextPeriod = SYSTICK_PERIOD_MAX;
		trig.Handler = nullptr;
		trig.pContext = nullptr;
		trig.nsPeriod = 0;
	}

	__enable_irq();

	if (handler != nullptr)
	{
		handler(pTimer, 0, ctx);
	}

	if (pTimer->EvtHandler != nullptr)
	{
		pTimer->EvtHandler(pTimer, TIMER_EVT_TRIGGER(0));
	}
}
