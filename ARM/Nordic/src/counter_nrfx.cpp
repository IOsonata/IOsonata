/**-------------------------------------------------------------------------
@file	counter_nrfx.cpp

@brief	Counter implementation on the Nordic nRF TIMER

The Counter of counter.h on a TIMER of the high frequency timer driver,
timer_hf_nrfx.cpp. The Timer and the Counter are two objects on the same
TIMER instances: this file uses the TIMER table of the timer driver, and its
interrupt handler calls nRFxCounterIrq for a TIMER in counter mode. The timer
driver has an empty weak nRFxCounterIrq, this file replaces it: it is linked
only by the applications calling CounterInit.

The TIMER in low power counter mode, 32 bit, counts its COUNT task. The
events counted are connected to COUNT with PEI, COUNTER_SRC_EVENT only. An
input pin is one of these events, routed by the application:

	IOPinEnableEvent(&Pei, &Evt, PortNo, PinNo, IOPINSENSE_HIGH_TRANSITION);
	CounterConnectTask(&Counter, &Pei, Evt, COUNTER_TASK_COUNT);
	PeiEnableEvent(&Pei, Evt);

The limit is CC[0] with the COMPARE0_CLEAR short: COMPARE0 is the limit
event and the TIMER clears itself in hardware. Without limit CC[0] is 0 and
COMPARE0 is the wrap of the counter. The interrupt adds the period to the
count at each COMPARE0. CountCC captures the counter to read it.

CounterInit and TimerInit set the mode of the TIMER they are given, the last
one called has the TIMER. The calls of a counter whose TIMER went back to
timer mode fail. DevNo -1 takes the TIMER with the highest number that is
neither bound to a timer nor in counter mode, never TIMER0, the timer of the
SoftDevice.

Limit 1 to 0xFFFFFFFF. CounterSetLimit stops the TIMER while it changes the
limit, events arriving then are not counted. It starts the TIMER again when
CounterStart started it, a stop by PEI is not known to it.

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
#include <stdbool.h>

#include "nrf.h"

#include "coredev/counter.h"
#include "coredev/interrupt.h"
#include "timer_nrfx.h"

// Mode of a TIMER used by the counter
#ifdef TIMER_MODE_MODE_LowPowerCounter
#define TIMER_NRFX_COUNTER_MODE		TIMER_MODE_MODE_LowPowerCounter
#else
#define TIMER_NRFX_COUNTER_MODE		TIMER_MODE_MODE_Counter
#endif

#pragma pack(push, 4)

// Counter using a TIMER in counter mode
typedef struct {
	CounterDev_t *pDev;
	bool bRunning;			//!< Started by CounterStart
	bool bLimitDue;			//!< Limit event folded by CounterSetLimit, reported by the interrupt
} nRFCounterBind_t;

#pragma pack(pop)

// Counter of each TIMER in counter mode, set by CounterInit
static nRFCounterBind_t s_nRFxCounter[TIMER_NRFX_HF_MAX];

// Events of one period of the hardware counter
static inline uint64_t nRFxCounterPeriod(CounterDev_t * const pDev)
{
	return pDev->Limit != 0 ? (uint64_t)pDev->Limit : 0x100000000ULL;
}

// Index of the TIMER of an initialized counter, -1 if none
static int nRFxCounterFind(CounterDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return -1;
	}

	for (int i = 0; i < TIMER_NRFX_HF_MAX; i++)
	{
		if (pDev->pDevData == &s_nRFxCounter[i] && s_nRFxCounter[i].pDev == pDev)
		{
			return i;
		}
	}

	return -1;
}

// Same, -1 also when the TIMER went back to timer mode
static int nRFxCounterIdx(CounterDev_t * const pDev)
{
	int idx = nRFxCounterFind(pDev);

	if (idx < 0 || s_nRFxTimerData[idx].pReg->MODE == TIMER_MODE_MODE_Timer)
	{
		return -1;
	}

	return idx;
}

// Count with the hardware counter. Interrupts disabled.
static uint64_t nRFxCounterRead(int Idx, CounterDev_t * const pDev)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[Idx].pReg;
	int cc = s_nRFxTimerData[Idx].CountCC;
	uint32_t before = reg->EVENTS_COMPARE[0];

	reg->TASKS_CAPTURE[cc] = 1;

	uint32_t count = reg->CC[cc];
	uint32_t after = reg->EVENTS_COMPARE[0];

	// Limit or wrap between the reads, the counter is read again after it
	if (after && !before)
	{
		reg->TASKS_CAPTURE[cc] = 1;
		count = reg->CC[cc];
	}

	return pDev->Count + count + (after ? nRFxCounterPeriod(pDev) : 0);
}

// Limit in CC[0], 0 for none. Interrupts disabled, TIMER stopped.
static void nRFxCounterSetLimit(int Idx, CounterDev_t * const pDev, uint32_t Limit)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[Idx].pReg;
	int cc = s_nRFxTimerData[Idx].CountCC;

	// A limit or a wrap not serviced yet is folded with the period it ended,
	// the interrupt reports the limit
	if (reg->EVENTS_COMPARE[0])
	{
		reg->EVENTS_COMPARE[0] = 0;
		pDev->Count += nRFxCounterPeriod(pDev);
		if (pDev->Limit != 0)
		{
			s_nRFxCounter[Idx].bLimitDue = true;
			NVIC_SetPendingIRQ(s_nRFxTimerData[Idx].IrqNo);
		}
	}

	reg->TASKS_CAPTURE[cc] = 1;

	uint32_t count = reg->CC[cc];

	// Period already past the new limit, it restarts now
	if (Limit != 0 && count >= Limit)
	{
		pDev->Count += count;
		reg->TASKS_CLEAR = 1;
	}

	reg->CC[0] = Limit;
	reg->SHORTS = Limit != 0 ? TIMER_SHORTS_COMPARE0_CLEAR_Msk : 0;
	pDev->Limit = Limit;
}

// TIMER no longer used by its counter: stopped and back in timer mode
static void nRFxCounterRelease(int Idx)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[Idx].pReg;
	IRQn_Type irq = s_nRFxTimerData[Idx].IrqNo;
	nRFCounterBind_t *bind = &s_nRFxCounter[Idx];

	NVIC_DisableIRQ(irq);
	reg->TASKS_STOP = 1;
	reg->INTENCLR = 0xFFFFFFFFUL;
	reg->SHORTS = 0;
	reg->TASKS_CLEAR = 1;
	reg->EVENTS_COMPARE[0] = 0;
	reg->MODE = TIMER_MODE_MODE_Timer;
	NVIC_ClearPendingIRQ(irq);

	if (bind->pDev != nullptr)
	{
		bind->pDev->pDevData = nullptr;
	}
	bind->pDev = nullptr;
	bind->bRunning = false;
	bind->bLimitDue = false;
}

// Replaces the empty default of timer_hf_nrfx.cpp
void nRFxCounterIrq(int Idx)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[Idx].pReg;
	nRFCounterBind_t *bind = &s_nRFxCounter[Idx];
	CounterDev_t *dev = bind->pDev;

	if (dev == nullptr)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	bool report = bind->bLimitDue;

	bind->bLimitDue = false;
	if (reg->EVENTS_COMPARE[0])
	{
		reg->EVENTS_COMPARE[0] = 0;
		(void)reg->EVENTS_COMPARE[0];
		dev->Count += nRFxCounterPeriod(dev);
		report = report || dev->Limit != 0;
	}

	uint64_t count = dev->Count;

	EnableInterrupt(state);

	if (report && dev->EvtHandler != nullptr)
	{
		dev->EvtHandler(dev, count);
	}
}

bool CounterInit(CounterDev_t * const pDev, const CounterCfg_t * const pCfg)
{
	// The events counted are routed with PEI, see above
	if (pDev == nullptr || pCfg == nullptr || pCfg->Src != COUNTER_SRC_EVENT ||
		pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS))
	{
		return false;
	}

	int prev = nRFxCounterFind(pDev);
	int idx = pCfg->DevNo - TIMER_NRFX_RTC_MAX;

	if (pCfg->DevNo < 0)
	{
		// Highest TIMER not bound to a timer and not counting, the one of
		// this counter if it still has it, never TIMER0
		int cur = nRFxCounterIdx(pDev);

		idx = -1;
		for (int i = TIMER_NRFX_HF_MAX - 1; i > 0; i--)
		{
			if (i == cur ||
				(s_pnRFxTimerDev[i] == nullptr && s_nRFxTimerData[i].pReg->MODE == TIMER_MODE_MODE_Timer))
			{
				idx = i;
				break;
			}
		}
	}

	if (idx < 0 || idx >= TIMER_NRFX_HF_MAX)
	{
		return false;
	}

	// Initialized before on another TIMER, that TIMER is released if it is
	// still the counter's
	if (prev >= 0 && prev != idx)
	{
		if (s_nRFxTimerData[prev].pReg->MODE != TIMER_MODE_MODE_Timer)
		{
			nRFxCounterRelease(prev);
		}
		else
		{
			s_nRFxCounter[prev].pDev = nullptr;
		}
	}

	nRFCounterBind_t *bind = &s_nRFxCounter[idx];
	NRF_TIMER_Type *reg = s_nRFxTimerData[idx].pReg;
	IRQn_Type irq = s_nRFxTimerData[idx].IrqNo;

	// Taken from another counter
	if (bind->pDev != nullptr && bind->pDev != pDev)
	{
		bind->pDev->pDevData = nullptr;
	}

	NVIC_DisableIRQ(irq);
	reg->TASKS_STOP = 1;
	reg->INTENCLR = 0xFFFFFFFFUL;
	reg->SHORTS = 0;
	reg->MODE = TIMER_NRFX_COUNTER_MODE;
	reg->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	reg->TASKS_CLEAR = 1;
	for (int i = 0; i <= s_nRFxTimerData[idx].CountCC; i++)
	{
		reg->EVENTS_COMPARE[i] = 0;
	}

	// The interrupt reaches the TIMER through it, as for a timer. A timer
	// bound to the TIMER no longer has it.
	s_pnRFxTimerData = s_nRFxTimerData;
	s_pnRFxTimerDev[idx] = nullptr;

	bind->pDev = pDev;
	bind->bRunning = false;
	bind->bLimitDue = false;

	pDev->DevNo = idx + TIMER_NRFX_RTC_MAX;
	pDev->EvtHandler = pCfg->EvtHandler;
	pDev->Count = 0;
	pDev->Limit = 0;
	pDev->pDevData = bind;

	nRFxCounterSetLimit(idx, pDev, pCfg->Limit);

	reg->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
	NVIC_ClearPendingIRQ(irq);
	NVIC_SetPriority(irq, pCfg->IntPrio);
	NVIC_EnableIRQ(irq);

	return true;
}

void CounterDeinit(CounterDev_t * const pDev)
{
	int idx = nRFxCounterFind(pDev);

	if (idx < 0)
	{
		return;
	}

	if (s_nRFxTimerData[idx].pReg->MODE != TIMER_MODE_MODE_Timer)
	{
		nRFxCounterRelease(idx);
	}
	else
	{
		// Taken by a timer since
		s_nRFxCounter[idx].pDev = nullptr;
		pDev->pDevData = nullptr;
	}
}

bool CounterStart(CounterDev_t * const pDev)
{
	int idx = nRFxCounterIdx(pDev);

	if (idx < 0)
	{
		return false;
	}

	s_nRFxCounter[idx].bRunning = true;
	s_nRFxTimerData[idx].pReg->TASKS_START = 1;

	return true;
}

void CounterStop(CounterDev_t * const pDev)
{
	int idx = nRFxCounterIdx(pDev);

	if (idx < 0)
	{
		return;
	}

	s_nRFxTimerData[idx].pReg->TASKS_STOP = 1;
	s_nRFxCounter[idx].bRunning = false;
}

uint64_t CounterGetCount(CounterDev_t * const pDev)
{
	int idx = nRFxCounterIdx(pDev);

	if (idx < 0)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	uint64_t count = nRFxCounterRead(idx, pDev);

	EnableInterrupt(state);

	return count;
}

void CounterClear(CounterDev_t * const pDev)
{
	int idx = nRFxCounterIdx(pDev);

	if (idx < 0)
	{
		return;
	}

	NRF_TIMER_Type *reg = s_nRFxTimerData[idx].pReg;
	uint32_t state = DisableInterrupt();

	reg->TASKS_CLEAR = 1;
	reg->EVENTS_COMPARE[0] = 0;
	pDev->Count = 0;
	s_nRFxCounter[idx].bLimitDue = false;
	NVIC_ClearPendingIRQ(s_nRFxTimerData[idx].IrqNo);
	EnableInterrupt(state);
}

uint32_t CounterSetLimit(CounterDev_t * const pDev, uint32_t Limit)
{
	int idx = nRFxCounterIdx(pDev);

	if (idx < 0)
	{
		return 0;
	}

	NRF_TIMER_Type *reg = s_nRFxTimerData[idx].pReg;
	uint32_t state = DisableInterrupt();

	reg->TASKS_STOP = 1;
	nRFxCounterSetLimit(idx, pDev, Limit);
	if (s_nRFxCounter[idx].bRunning)
	{
		reg->TASKS_START = 1;
	}
	EnableInterrupt(state);

	return pDev->Limit;
}

PeiEndpoint_t CounterGetTaskEndpoint(CounterDev_t * const pDev, COUNTER_TASK Task)
{
	int idx = nRFxCounterIdx(pDev);

	if (idx < 0)
	{
		return 0;
	}

	NRF_TIMER_Type *reg = s_nRFxTimerData[idx].pReg;

	switch (Task)
	{
		case COUNTER_TASK_COUNT:
			return (PeiEndpoint_t)&reg->TASKS_COUNT;
		case COUNTER_TASK_START:
			return (PeiEndpoint_t)&reg->TASKS_START;
		case COUNTER_TASK_STOP:
			return (PeiEndpoint_t)&reg->TASKS_STOP;
	}

	return 0;
}

PeiEndpoint_t CounterGetEventEndpoint(CounterDev_t * const pDev)
{
	int idx = nRFxCounterIdx(pDev);

	if (idx < 0)
	{
		return 0;
	}

	return (PeiEndpoint_t)&s_nRFxTimerData[idx].pReg->EVENTS_COMPARE[0];
}
