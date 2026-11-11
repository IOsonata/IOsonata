/**-------------------------------------------------------------------------
@file	timer_re01.h

@brief	timer implementation on Renesas RE01 series

@author	Hoang Nguyen Hoan
@date	Feb. 3, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#ifndef __TIMER_RE01_H__
#define __TIMER_RE01_H__

#include <stdint.h>

#include "coredev/timer.h"

#include "re01xxx.h"

#define RE01_TIMER_AGT_CNT				2	// AGT0, AGT1
#define RE01_TIMER_TMR_CNT				1	// TMR0 + TMR1 linked as one 16bits high freq timer
#define RE01_TIMER_GPT_CNT				6	// GPT0/1 (32 bits), GPT2..5 (16 bits)
#define RE01_TIMER_GPT_DEVNO				(RE01_TIMER_AGT_CNT + RE01_TIMER_TMR_CNT)
#define RE01_TIMER_MAXCNT				(RE01_TIMER_GPT_DEVNO + RE01_TIMER_GPT_CNT)

#define RE01_TIMER_AGT_TRIG_MAXCNT		2
#define RE01_TIMER_TMR_TRIG_MAXCNT		2
#define RE01_TIMER_CC_MAXCNT				4
#define RE01_TIMER_TRIG_MAXCNT			4

// CPU cycles from the last counter read of a compare interrupt handler to the
// clear of its interrupt flag, after the handler returns
#define RE01_TIMER_ISR_TAIL				256

// Trigger calls in one compare interrupt at most
#define RE01_TIMER_ISR_CALLS				4

// AGT compare values are written with the counter stopped. The counts the
// counter misses are added back: RE01_TIMER_AGT_RESTART count source cycles for
// a start to take effect (3 AGTCLK in the Renesas FSP AGT notes), and the CPU
// cycles from the stop to the start, RE01_TIMER_AGT_STOP_CYCLES (175 to 272 on
// a Cortex-M0+ model of a Release build). To be checked on hardware.
#define RE01_TIMER_AGT_RESTART			3
#define RE01_TIMER_AGT_STOP_CYCLES		224

#pragma pack(push, 4)

typedef struct {
	int DevNo;		//!< Device number (index)
	union {
		AGT0_Type *pAgtReg;
		AGT1_Type *pAgtReg1;
		TMR01_Type *pTmrReg;
		GPT320_Type *pGptReg;
	};
	uint32_t BaseFreq;
	uint8_t TmrClock;		//!< TMR1 source/divider retained across Disable/Enable
	uint32_t CC[RE01_TIMER_CC_MAXCNT];
	uint32_t Deadline[RE01_TIMER_TRIG_MAXCNT];	//!< Count of the next trigger call (TMR, GPT), tick count (AGT, low 16 bits)
	uint32_t Margin;		//!< Counts a compare must be ahead when a compare interrupt returns
	uint32_t Lost;			//!< AGT: counts the counter misses in a stop and a start
	TimerTrig_t Trigger[RE01_TIMER_TRIG_MAXCNT];
	TimerDev_t *pTimer;
	int IntPrio;
	IRQn_Type IrqOvr;
	IRQn_Type IrqMatch[RE01_TIMER_CC_MAXCNT];
} RE01_TimerData_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool Re01AgtInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg);
bool Re01TmrInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg);
bool Re01GptInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg);
extern RE01_TimerData_t g_Re01TimerData[RE01_TIMER_MAXCNT];

// A failed initialization or a handle moved to another device must not control
// a timer now owned by a different handle.
static inline RE01_TimerData_t *Re01TimerData(TimerDev_t *pTimer)
{
	if (pTimer == NULL || pTimer->DevNo < 0 || pTimer->DevNo >= RE01_TIMER_MAXCNT)
	{
		return NULL;
	}
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];
	return dev->pTimer == pTimer ? dev : NULL;
}

static inline void Re01TimerClearPending(IRQn_Type Irq)
{
	if (Irq != (IRQn_Type)-1)
	{
		(&ICU->IELSR0)[Irq - IEL0_IRQn] &= ~ICU_IELSR0_IR_Msk;
		NVIC_ClearPendingIRQ(Irq);
	}
}

static inline uint64_t Re01TimerPeriodTicks(uint32_t Freq, uint64_t nsPeriod)
{
	return (nsPeriod / 1000000000ULL) * Freq +
		((nsPeriod % 1000000000ULL) * Freq + 500000000ULL) / 1000000000ULL;
}

// Closest period the compare takes, from 3 ticks to Mask
static inline uint32_t Re01TimerClosestTicks(uint64_t Ticks, uint32_t Mask)
{
	return Ticks < 3 ? 3 : (Ticks > Mask ? Mask : (uint32_t)Ticks);
}

// Closest period of every active trigger at the new frequency
static inline bool Re01TimerNewPeriods(RE01_TimerData_t *dev, uint32_t Freq,
		uint32_t Mask, uint32_t *CC)
{
	for (int i = 0; i < RE01_TIMER_TRIG_MAXCNT; i++)
	{
		CC[i] = 0;
		if (dev->IrqMatch[i] != (IRQn_Type)-1)
		{
			CC[i] = Re01TimerClosestTicks(Re01TimerPeriodTicks(Freq, dev->Trigger[i].nsPeriod), Mask);
		}
	}
	return true;
}

static inline void Re01TimerSetPeriods(RE01_TimerData_t *dev, const uint32_t *CC)
{
	memcpy(dev->CC, CC, sizeof(dev->CC));
	for (int i = 0; i < RE01_TIMER_TRIG_MAXCNT; i++)
	{
		if (CC[i])
		{
			dev->Trigger[i].nsPeriod = TimerTickToTime(dev->pTimer, CC[i], 1000000000UL);
		}
	}
}

// True if Count has reached Deadline, which is Period after a previous
// deadline or after the count it was set from
static inline bool Re01TimerReached(uint32_t Count, uint32_t Deadline,
		uint32_t Period, uint32_t Mask)
{
	return ((Count - (Deadline - Period)) & Mask) >= Period;
}

// The interrupt dispatcher clears the interrupt flag after the handler
// returns: a compare match before that is lost and waits a counter wrap. A
// compare interrupt handler returns with its compare value this many counts
// ahead of the counter at least.
static inline uint32_t Re01TimerMargin(uint32_t Freq)
{
	uint32_t core = SystemCoreClock;

	if (core == 0)
	{
		return Freq;
	}

	return (uint32_t)(((uint64_t)RE01_TIMER_ISR_TAIL * Freq + core - 1) / core) + 1;
}

// Preserve the previous compare phase and skip elapsed periods in bounded work.
// At most one counter wrap may elapse before the interrupt is serviced.
static inline uint32_t Re01TimerNextCompare(uint32_t Deadline, uint32_t Count,
		uint32_t Period, uint32_t Mask)
{
	uint32_t elapsed = (Count - Deadline) & Mask;

	// The division only when a period or more was missed
	if (elapsed < Period)
	{
		return (Deadline + Period) & Mask;
	}

	uint64_t steps = (uint64_t)elapsed / Period + 1;
	return (uint32_t)(Deadline + steps * Period) & Mask;
}

#ifdef __cplusplus
}
#endif

#endif // __TIMER_RE01_H__
