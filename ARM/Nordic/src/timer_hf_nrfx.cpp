/**-------------------------------------------------------------------------
@file	timer_hf_nrfx.cpp

@brief	Timer class implementation on Nordic nRFx series using high frequency timer (TIMERx)

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2017

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#include <atomic>

#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "nrf.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

#include "timer_nrfx.h"
#include "coredev/pei.h"

// Optional PEI references must not load the PEI matrix from a Timer-only
// firmware. The PEI API installs compare callbacks only when requested.
#if defined(__GNUC__)
#pragma weak PeiBindEvent
#pragma weak PeiUnbindEvent
#pragma weak PeiDisableEvent
#endif

#if defined(NRF91_SERIES) || defined(NRF5340_XXAA_APPLICATION)
#define NRF_TIMER0			NRF_TIMER0_S
#define NRF_TIMER1			NRF_TIMER1_S
#define NRF_TIMER2			NRF_TIMER2_S
#elif defined(NRF5340_XXAA_NETWORK)
#define NRF_TIMER0			NRF_TIMER0_NS
#define NRF_TIMER1			NRF_TIMER1_NS
#define NRF_TIMER2			NRF_TIMER2_NS
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#define NRF_TIMER0			NRF_TIMER00_S
#define NRF_TIMER1			NRF_TIMER10_S
#define NRF_TIMER2			NRF_TIMER20_S
#define NRF_TIMER3			NRF_TIMER21_S
#define NRF_TIMER4			NRF_TIMER22_S
#define NRF_TIMER5			NRF_TIMER23_S
#define NRF_TIMER6			NRF_TIMER24_S
#define TIMER0_CC_NUM		TIMER00_CC_NUM_SIZE
#define TIMER1_CC_NUM		TIMER10_CC_NUM_SIZE
#define TIMER2_CC_NUM		TIMER20_CC_NUM_SIZE
#define TIMER3_CC_NUM		TIMER21_CC_NUM_SIZE
#define TIMER4_CC_NUM		TIMER22_CC_NUM_SIZE
#define TIMER5_CC_NUM		TIMER23_CC_NUM_SIZE
#define TIMER6_CC_NUM		TIMER24_CC_NUM_SIZE
#define TIMER0_IRQn			TIMER00_IRQn
#define TIMER1_IRQn			TIMER10_IRQn
#define TIMER2_IRQn			TIMER20_IRQn
#define TIMER3_IRQn			TIMER21_IRQn
#define TIMER4_IRQn			TIMER22_IRQn
#define TIMER5_IRQn			TIMER23_IRQn
#define TIMER6_IRQn			TIMER24_IRQn
// Clock before the prescaler: TIMER00 the CPU clock (64 or 128 MHz), TIMER10
// 32 MHz, TIMER20 to TIMER24 16 MHz
#define TIMER0_BASE_FREQ	(TIMER00_PCLK_MHZ * 1000000UL)
#define TIMER1_BASE_FREQ	(TIMER10_PCLK_MHZ * 1000000UL)
#endif

#ifndef TIMER0_BASE_FREQ
#define TIMER0_BASE_FREQ	TIMER_NRFX_HF_BASE_FREQ
#define TIMER1_BASE_FREQ	TIMER_NRFX_HF_BASE_FREQ
#endif

// Largest PRESCALER value, the clock divided by 2^PRESCALER
#ifdef TIMER_PRESCALER_PRESCALER_Max
#define TIMER_NRFX_PRESCALER_MAX	TIMER_PRESCALER_PRESCALER_Max
#else
#define TIMER_NRFX_PRESCALER_MAX	9
#endif

// PEI compare sources reuse the ordinary trigger state without extra RAM.
// Separate markers retain the requested single/continuous CC behavior.
static constexpr TIMER_TRIG_TYPE TIMER_TRIG_PEI_SINGLE =
	static_cast<TIMER_TRIG_TYPE>(2);
static constexpr TIMER_TRIG_TYPE TIMER_TRIG_PEI_CONTINUOUS =
	static_cast<TIMER_TRIG_TYPE>(3);

// Trigger state of each timer, sized by the number of compare registers of
// that timer less the one reserved for reading the counter.
static uint32_t s_nRFxTimer0CC[TIMER0_CC_NUM - 1];
static TimerTrig_t s_nRFxTimer0Trig[TIMER0_CC_NUM - 1];
static uint32_t s_nRFxTimer1CC[TIMER1_CC_NUM - 1];
static TimerTrig_t s_nRFxTimer1Trig[TIMER1_CC_NUM - 1];
static uint32_t s_nRFxTimer2CC[TIMER2_CC_NUM - 1];
static TimerTrig_t s_nRFxTimer2Trig[TIMER2_CC_NUM - 1];
#if TIMER_NRFX_HF_MAX > 3
static uint32_t s_nRFxTimer3CC[TIMER3_CC_NUM - 1];
static TimerTrig_t s_nRFxTimer3Trig[TIMER3_CC_NUM - 1];
static uint32_t s_nRFxTimer4CC[TIMER4_CC_NUM - 1];
static TimerTrig_t s_nRFxTimer4Trig[TIMER4_CC_NUM - 1];
#endif
#if TIMER_NRFX_HF_MAX > 5
static uint32_t s_nRFxTimer5CC[TIMER5_CC_NUM - 1];
static TimerTrig_t s_nRFxTimer5Trig[TIMER5_CC_NUM - 1];
static uint32_t s_nRFxTimer6CC[TIMER6_CC_NUM - 1];
static TimerTrig_t s_nRFxTimer6Trig[TIMER6_CC_NUM - 1];
#endif

// Timer device of each timer, set by nRFxTimerInit
TimerDev_t *s_pnRFxTimerDev[TIMER_NRFX_HF_MAX];

// What does not change is constant, so it stays in flash. Only the trigger
// state above and the device pointers are in RAM.
alignas(4) const nRFTimerData_t s_nRFxTimerData[TIMER_NRFX_HF_MAX] = {
	{
		.DevNo = TIMER_NRFX_RTC_MAX,
		.MaxFreq = TIMER0_BASE_FREQ,
		.pReg = NRF_TIMER0,
		.IrqNo = TIMER0_IRQn,
		.MaxNbTrigEvt = TIMER0_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER0_CC_NUM - 1,
		.pCC = s_nRFxTimer0CC,
		.pTrigger = s_nRFxTimer0Trig,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 1,
		.MaxFreq = TIMER1_BASE_FREQ,
		.pReg = NRF_TIMER1,
		.IrqNo = TIMER1_IRQn,
		.MaxNbTrigEvt = TIMER1_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER1_CC_NUM - 1,
		.pCC = s_nRFxTimer1CC,
		.pTrigger = s_nRFxTimer1Trig,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 2,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER2,
		.IrqNo = TIMER2_IRQn,
		.MaxNbTrigEvt = TIMER2_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER2_CC_NUM - 1,
		.pCC = s_nRFxTimer2CC,
		.pTrigger = s_nRFxTimer2Trig,
	},
#if TIMER_NRFX_HF_MAX > 3
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 3,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER3,
		.IrqNo = TIMER3_IRQn,
		.MaxNbTrigEvt = TIMER3_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER3_CC_NUM - 1,
		.pCC = s_nRFxTimer3CC,
		.pTrigger = s_nRFxTimer3Trig,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 4,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER4,
		.IrqNo = TIMER4_IRQn,
		.MaxNbTrigEvt = TIMER4_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER4_CC_NUM - 1,
		.pCC = s_nRFxTimer4CC,
		.pTrigger = s_nRFxTimer4Trig,
	},
#endif
#if TIMER_NRFX_HF_MAX > 5
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 5,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER5,
		.IrqNo = TIMER5_IRQn,
		.MaxNbTrigEvt = TIMER5_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER5_CC_NUM - 1,
		.pCC = s_nRFxTimer5CC,
		.pTrigger = s_nRFxTimer5Trig,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 6,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER6,
		.IrqNo = TIMER6_IRQn,
		.MaxNbTrigEvt = TIMER6_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER6_CC_NUM - 1,
		.pCC = s_nRFxTimer6CC,
		.pTrigger = s_nRFxTimer6Trig,
	},
#endif
};

static std::atomic<int> s_nRfxHFClockSem(0);
static bool s_bTimerEnabled[TIMER_NRFX_HF_MAX];

// CLEAR task endpoint given out: a hardware event can clear the counter
static bool s_bnRFxTimerHwClear[TIMER_NRFX_HF_MAX];

// Set by nRFxTimerInit. The interrupt handlers below are kept by the linker as soon as
// this file is part of a build, used or not. They reach the timer data
// through this pointer, so that the data is linked only when a timer is
// initialized.
const nRFTimerData_t *s_pnRFxTimerData = nullptr;

// Interrupt of a TIMER in counter mode. counter_nrfx.cpp, linked by
// CounterInit, replaces this empty default.
__WEAK void nRFxCounterIrq(int Idx)
{
	(void)Idx;
}

// Tick count of a timer from its counter, read through the last CC. A counter
// lower than the last one read is a wrap, or a clear once the CLEAR task can
// be run by a hardware event: the tick count then restarts from the clear.
// Interrupts are masked, the interrupt handler reads the counter too. The
// handler passes its own table reference, see s_pnRFxTimerData.
static uint64_t nRFxTimerReadCount(const nRFTimerData_t &TData, int Idx, TimerDev_t * const pTimer,
								   bool *pbWrap)
{
	NRF_TIMER_Type *reg = TData.pReg;
	uint32_t state = DisableInterrupt();
	uint32_t t = reg->CC[TData.CountCC];	// Preserve comparator
	uint32_t count;
	bool wrap = false;

	reg->TASKS_CAPTURE[TData.CountCC] = 1;
	while ((count = reg->CC[TData.CountCC]) != reg->CC[TData.CountCC]);

	reg->CC[TData.CountCC] = t;	// Restore comparator

	if (count < pTimer->LastCount)
	{
		if (s_bnRFxTimerHwClear[Idx])
		{
			pTimer->Rollover = 0;
		}
		else
		{
			pTimer->Rollover += 0x100000000ULL;
			wrap = true;
		}
	}
	pTimer->LastCount = count;

	uint64_t ticks = pTimer->Rollover + count;

	EnableInterrupt(state);

	if (pbWrap != nullptr)
	{
		*pbWrap = wrap;
	}

	return ticks;
}

static void TimerIRQHandler(int DevNo)
{
	if (s_pnRFxTimerData == nullptr)
	{
		// No timer of this kind was initialized
		return;
	}

	NRF_TIMER_Type *reg = s_pnRFxTimerData[DevNo].pReg;

	// A TIMER in counter mode is the counter's
	if (reg->MODE != TIMER_MODE_MODE_Timer)
	{
		nRFxCounterIrq(DevNo);
		return;
	}

	const nRFTimerData_t &tdata = s_pnRFxTimerData[DevNo];
	TimerDev_t *timer = s_pnRFxTimerDev[DevNo];
	if (timer == nullptr)
	{
		return;
	}

	uint32_t evt = 0;
	uint32_t ticks;
	bool wrap;

	(void)nRFxTimerReadCount(tdata, DevNo, timer, &wrap);

	uint32_t count = timer->LastCount;

	if (wrap)
	{
		evt |= TIMER_EVT_COUNTER_OVR;
	}

    for (int i = 0; i < tdata.MaxNbTrigEvt; i++)
    {
		if (reg->EVENTS_COMPARE[i])
		{
			reg->EVENTS_COMPARE[i] = 0U;
			if (tdata.pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
			{
				// PEI handles event-to-task routing in hardware. The
				// ISR only advances this CC, preserving elapsed time.
				ticks = (uint32_t)TimerNanosecondToTick(
					timer, tdata.pTrigger[i].nsPeriod);
				reg->CC[i] = count + ticks;
			}
			else if (tdata.pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE)
			{
				// HF TIMER has no EVTEN gate; disable the logical PPI
				// event after its first compare, without stopping TIMER.
				reg->INTENCLR = TIMER_INTENSET_COMPARE0_Msk << i;
				if (PeiDisableEvent != nullptr)
				{
					PeiDisableEvent(
						static_cast<PeiDev_t *>(tdata.pTrigger[i].pContext),
						tdata.pCC[i]);
				}
			}
			else
			{
				TimerTrig_t trig = tdata.pTrigger[i];

				evt |= TIMER_EVT_TRIGGER(i);
				if (trig.Type == TIMER_TRIG_TYPE_CONTINUOUS)
				{
					reg->CC[i] = count + tdata.pCC[i];
				}
				else
				{
					// Single shot done, the trigger is free. Its compare would
					// match again after a wrap of the counter.
					reg->INTENCLR = TIMER_INTENSET_COMPARE0_Msk << i;
					tdata.pCC[i] = 0;
					tdata.pTrigger[i] = {};
				}
				if (trig.Handler != nullptr)
				{
					trig.Handler(timer, i, trig.pContext);
				}
			}
		}

    }

    if (timer->EvtHandler)
    {
    	timer->EvtHandler(timer, evt);
    }
}

extern "C" {

#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
__WEAK void TIMER00_IRQHandler()
{
	TimerIRQHandler(0);
}

__WEAK void TIMER10_IRQHandler()
{
	TimerIRQHandler(1);
}

__WEAK void TIMER20_IRQHandler()
{
	TimerIRQHandler(2);
}

__WEAK void TIMER21_IRQHandler()
{
	TimerIRQHandler(3);
}

__WEAK void TIMER22_IRQHandler()
{
	TimerIRQHandler(4);
}

__WEAK void TIMER23_IRQHandler()
{
	TimerIRQHandler(5);
}

__WEAK void TIMER24_IRQHandler()
{
	TimerIRQHandler(6);
}

#else
__WEAK void TIMER0_IRQHandler()
{
	TimerIRQHandler(0);
}

__WEAK void TIMER1_IRQHandler()
{
	TimerIRQHandler(1);
}

__WEAK void TIMER2_IRQHandler()
{
	TimerIRQHandler(2);
}

#if TIMER_NRFX_HF_MAX > 3
__WEAK void TIMER3_IRQHandler()
{
	TimerIRQHandler(3);
}

__WEAK void TIMER4_IRQHandler()
{
	TimerIRQHandler(4);
}
#endif
#endif

}   // extern "C"

static bool nRFxTimerEnable(TimerDev_t * const pTimer)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	NRF_TIMER_Type *reg = s_nRFxTimerData[devno].pReg;

	if (s_bTimerEnabled[devno])
	{
		return true;
	}

	if (s_nRfxHFClockSem == 0)
	{
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		if (!(NRF_CLOCK->PLL.STAT & CLOCK_PLL_STAT_STATE_Msk))
		{
			NRF_CLOCK->TASKS_PLLSTART = 1;
		}
#endif
		// One request for the 32 MHz crystal while timers run, released by
		// the last Disable
		if (SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000) == false)
		{
			return false;
		}
	}

	s_bTimerEnabled[devno] = true;
	s_nRfxHFClockSem++;

	switch (devno)
	{
		case 0:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
			NVIC_ClearPendingIRQ(TIMER00_IRQn);
			NVIC_EnableIRQ(TIMER00_IRQn);
#else
			NVIC_ClearPendingIRQ(TIMER0_IRQn);
			NVIC_EnableIRQ(TIMER0_IRQn);
#endif
			break;
		case 1:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
			NVIC_ClearPendingIRQ(TIMER10_IRQn);
			NVIC_EnableIRQ(TIMER10_IRQn);
#else
			NVIC_ClearPendingIRQ(TIMER1_IRQn);
			NVIC_EnableIRQ(TIMER1_IRQn);
#endif
			break;
		case 2:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
			NVIC_ClearPendingIRQ(TIMER20_IRQn);
			NVIC_EnableIRQ(TIMER20_IRQn);
#else
			NVIC_ClearPendingIRQ(TIMER2_IRQn);
			NVIC_EnableIRQ(TIMER2_IRQn);
#endif
			break;
#if TIMER_NRFX_HF_MAX > 3
		case 3:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
			NVIC_ClearPendingIRQ(TIMER21_IRQn);
			NVIC_EnableIRQ(TIMER21_IRQn);
#else
			NVIC_ClearPendingIRQ(TIMER3_IRQn);
			NVIC_EnableIRQ(TIMER3_IRQn);
#endif
			break;
		case 4:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
			NVIC_ClearPendingIRQ(TIMER22_IRQn);
			NVIC_EnableIRQ(TIMER22_IRQn);
#else
			NVIC_ClearPendingIRQ(TIMER4_IRQn);
			NVIC_EnableIRQ(TIMER4_IRQn);
#endif
			break;
#endif
#if TIMER_NRFX_HF_MAX > 5
		case 5:
			NVIC_ClearPendingIRQ(TIMER23_IRQn);
			NVIC_EnableIRQ(TIMER23_IRQn);
			break;
		case 6:
			NVIC_ClearPendingIRQ(TIMER24_IRQn);
			NVIC_EnableIRQ(TIMER24_IRQn);
			break;
#endif
    }

    reg->TASKS_START = 1;

    return true;
}

static void nRFxTimerDisable(TimerDev_t * const pTimer)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	NRF_TIMER_Type *reg = s_nRFxTimerData[devno].pReg;

	if (!s_bTimerEnabled[devno])
	{
		return;
	}

	s_bTimerEnabled[devno] = false;
	s_nRfxHFClockSem--;

	reg->TASKS_STOP = 1;

	if (s_nRfxHFClockSem == 0)
	{
		(void)SystemCoreClockSelect(OSC_TYPE_RC, 0);
	}

    switch (devno)
    {
        case 0:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
        	NVIC_ClearPendingIRQ(TIMER00_IRQn);
            NVIC_DisableIRQ(TIMER00_IRQn);
#else
        	NVIC_ClearPendingIRQ(TIMER0_IRQn);
            NVIC_DisableIRQ(TIMER0_IRQn);
#endif
            break;
        case 1:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
        	NVIC_ClearPendingIRQ(TIMER10_IRQn);
            NVIC_DisableIRQ(TIMER10_IRQn);
#else
            NVIC_ClearPendingIRQ(TIMER1_IRQn);
            NVIC_DisableIRQ(TIMER1_IRQn);
#endif
            break;
        case 2:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
        	NVIC_ClearPendingIRQ(TIMER20_IRQn);
            NVIC_DisableIRQ(TIMER20_IRQn);
#else
            NVIC_ClearPendingIRQ(TIMER2_IRQn);
            NVIC_DisableIRQ(TIMER2_IRQn);
#endif
            break;
#if TIMER_NRFX_HF_MAX > 3
        case 3:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
        	NVIC_ClearPendingIRQ(TIMER21_IRQn);
            NVIC_DisableIRQ(TIMER21_IRQn);
#else
            NVIC_ClearPendingIRQ(TIMER3_IRQn);
            NVIC_DisableIRQ(TIMER3_IRQn);
#endif
            break;
        case 4:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
        	NVIC_ClearPendingIRQ(TIMER22_IRQn);
            NVIC_DisableIRQ(TIMER22_IRQn);
#else
            NVIC_ClearPendingIRQ(TIMER4_IRQn);
            NVIC_DisableIRQ(TIMER4_IRQn);
#endif
            break;
#endif
#if TIMER_NRFX_HF_MAX > 5
        case 5:
        	NVIC_ClearPendingIRQ(TIMER23_IRQn);
            NVIC_DisableIRQ(TIMER23_IRQn);
            break;
        case 6:
        	NVIC_ClearPendingIRQ(TIMER24_IRQn);
            NVIC_DisableIRQ(TIMER24_IRQn);
            break;
#endif
    }

	// The count and the triggers are kept, Enable resumes them
}

// Clears the counter and the tick count. The armed triggers restart their
// period from the clear.
static void nRFxTimerReset(TimerDev_t * const pTimer)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];
	NRF_TIMER_Type *reg = tdata.pReg;
	uint32_t state = DisableInterrupt();

	reg->TASKS_CLEAR = 1;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;

	for (int i = 0; i < tdata.MaxNbTrigEvt; i++)
	{
		uint32_t bit = TIMER_INTENSET_COMPARE0_Msk << i;

		if (tdata.pTrigger[i].nsPeriod == 0U || (reg->INTENSET & bit) == 0)
		{
			continue;
		}
		if (tdata.pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE ||
			tdata.pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
		{
			reg->CC[i] = (uint32_t)TimerNanosecondToTick(pTimer, tdata.pTrigger[i].nsPeriod);
		}
		else
		{
			reg->CC[i] = tdata.pCC[i];
		}
		reg->EVENTS_COMPARE[i] = 0;
	}

	EnableInterrupt(state);
}

static uint32_t nRFxTimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	NRF_TIMER_Type *reg = s_nRFxTimerData[devno].pReg;
	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];
	int i;

	// Frequency changes invalidate scheduled compare periods.
	for (i = 0; i < tdata.MaxNbTrigEvt; i++)
	{
		if (tdata.pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE ||
			tdata.pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
		{
			return pTimer->Freq;
		}
	}

	// Clock before the prescaler. TIMER00 of the nRF54L runs from the CPU
	// clock.
	uint32_t base = tdata.MaxFreq;
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	if (devno == 0)
	{
		base = SystemCoreClock;
	}
#endif

	// Closest frequency base / 2^PRESCALER, the higher one of two as close,
	// the highest for 0
	uint32_t prescaler = 0;

	if (Freq > 0)
	{
		uint32_t diff = UINT32_MAX;

		for (uint32_t p = 0; p <= TIMER_NRFX_PRESCALER_MAX; p++)
		{
			uint32_t f = base >> p;
			uint32_t d = f > Freq ? f - Freq : Freq - f;

			if (d < diff)
			{
				diff = d;
				prescaler = p;
			}
		}
	}

	// PRESCALER is written with the timer stopped
	reg->TASKS_STOP = 1;
	reg->PRESCALER = prescaler;

	pTimer->Freq = base >> prescaler;

	// Tick period rounded down to a nanosecond, 62 for the 62.5 ns of 16 MHz.
	// Not precise enough to convert a count or a trigger period, those are
	// computed from Freq, see TimerTickToTime.
    pTimer->nsPeriod = 1000000000ULL / pTimer->Freq;

	// A disabled timer stays stopped
	if (s_bTimerEnabled[devno])
	{
		reg->TASKS_START = 1;
	}

    return pTimer->Freq;
}

static uint64_t nRFxTimerGetTickCount(TimerDev_t * const pTimer)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;

	return nRFxTimerReadCount(s_nRFxTimerData[devno], devno, pTimer, nullptr);
}

// Peripheral hardware task endpoints; PEI owns only the routing.
static uintptr_t nRFxTimerTaskEndpoint(TimerDev_t * const pTimer, TIMER_TASK Task)
{
	int devno;
	NRF_TIMER_Type *reg;

	if (pTimer == nullptr)
	{
		return 0U;
	}
	devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	if (devno < 0 || devno >= TIMER_NRFX_HF_MAX ||
		s_pnRFxTimerDev[devno] != pTimer)
	{
		return 0U;
	}
	reg = s_nRFxTimerData[devno].pReg;
	switch (Task)
	{
		case TIMER_TASK_START:
			return (uintptr_t)&reg->TASKS_START;
		case TIMER_TASK_STOP:
			return (uintptr_t)&reg->TASKS_STOP;
		case TIMER_TASK_CLEAR:
			// From now on a lower counter is a clear, not a wrap
			s_bnRFxTimerHwClear[devno] = true;
			return (uintptr_t)&reg->TASKS_CLEAR;
		default:
			return 0U;
	}
}

// Run a timer task from the CPU without releasing the timer clock.
// This only prepares the initial state in PEI Blinky; buttons subsequently
// activate the same tasks directly in hardware.
static bool nRFxTimerExecuteTask(TimerDev_t * const pTimer, TIMER_TASK Task)
{
	uintptr_t endpoint;

	if (Task == TIMER_TASK_CLEAR)
	{
		if (nRFxTimerTaskEndpoint(pTimer, TIMER_TASK_START) == 0U)
		{
			return false;
		}
		// From the CPU, the tick count and the triggers follow the clear
		nRFxTimerReset(pTimer);
		return true;
	}

	endpoint = nRFxTimerTaskEndpoint(pTimer, Task);
	if (endpoint == 0U)
	{
		return false;
	}
	*(volatile uint32_t *)endpoint = 1U;
	return true;
}

// Bind one free CC to PEI. The free-running counter is never cleared,
// stopped or routed to counter-control tasks by a compare trigger.
// Continuous CC rescheduling follows the original Timer ISR model.
bool nRFxTimerEnablePeiTrigger(TimerDev_t * const pTimer, int TrigNo,
	uint64_t nsPeriod, TIMER_TRIG_TYPE Type, PeiDev_t *pPei,
	PeiEvent_t *pEvent)
{
	const nRFTimerData_t *pData;
	NRF_TIMER_Type *reg;
	PeiEndpoint_t endpoint;
	PeiEvent_t evt;
	uint64_t ticks;
	uint32_t count;
	int devno;

	if (pTimer == nullptr || pPei == nullptr || pEvent == nullptr ||
		PeiBindEvent == nullptr || PeiUnbindEvent == nullptr ||
		PeiDisableEvent == nullptr || nsPeriod == 0U ||
		(Type != TIMER_TRIG_TYPE_SINGLE &&
			Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return false;
	}
	devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	if (devno < 0 || devno >= TIMER_NRFX_HF_MAX || pTimer->Freq == 0U)
	{
		return false;
	}
	pData = &s_nRFxTimerData[devno];
	reg = pData->pReg;
	if (TrigNo < 0 || TrigNo >= pData->MaxNbTrigEvt ||
		pData->pTrigger[TrigNo].nsPeriod != 0U ||
		pData->pTrigger[TrigNo].Type == TIMER_TRIG_PEI_SINGLE ||
		pData->pTrigger[TrigNo].Type == TIMER_TRIG_PEI_CONTINUOUS ||
		reg->SHORTS != 0U)
	{
		return false;
	}
	ticks = TimerNanosecondToTick(pTimer, nsPeriod);
	if (ticks < 2U || ticks > UINT32_MAX)
	{
		return false;
	}
	endpoint = (PeiEndpoint_t)&reg->EVENTS_COMPARE[TrigNo];
	evt = *pEvent;
	if (!PeiBindEvent(pPei, &evt, endpoint))
	{
		return false;
	}

	// Capture the running counter in this newly owned CC slot.
	reg->TASKS_CAPTURE[TrigNo] = 1U;
	count = reg->CC[TrigNo];
	reg->EVENTS_COMPARE[TrigNo] = 0U;
	reg->CC[TrigNo] = count + (uint32_t)ticks;
	pData->pTrigger[TrigNo].Type = Type == TIMER_TRIG_TYPE_CONTINUOUS ?
		TIMER_TRIG_PEI_CONTINUOUS : TIMER_TRIG_PEI_SINGLE;
	pData->pTrigger[TrigNo].nsPeriod = TimerTickToNanosecond(pTimer, ticks);
	pData->pTrigger[TrigNo].Handler = nullptr;
	pData->pTrigger[TrigNo].pContext = pPei;
	pData->pCC[TrigNo] = evt;
	*pEvent = evt;
	reg->INTENSET = TIMER_INTENSET_COMPARE0_Msk << TrigNo;
	return true;
}

static uint64_t nRFxTimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                                       TimerTrigEvtHandler_t const Handler, void * const pContext)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];
	NRF_TIMER_Type *reg = tdata.pReg;

	if (TrigNo < 0 || TrigNo >= tdata.MaxNbTrigEvt)
	{
		return 0;
	}

	// Each CC owns its compare independently; never replace an active one.
	if (tdata.pTrigger[TrigNo].nsPeriod != 0U ||
		tdata.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_SINGLE ||
		tdata.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_CONTINUOUS)
	{
		return 0;
	}

	uint32_t cc = TimerNanosecondToTick(pTimer, nsPeriod);

    if (cc <= 0)
    {
        return 0;
    }

	tdata.pTrigger[TrigNo].Type = Type;
	tdata.pTrigger[TrigNo].nsPeriod = TimerTickToNanosecond(pTimer, cc);
	tdata.pTrigger[TrigNo].Handler = Handler;
	tdata.pTrigger[TrigNo].pContext = pContext;
	tdata.pCC[TrigNo] = cc;

	(void)nRFxTimerReadCount(tdata, devno, pTimer, nullptr);

	// A compare left from before is cleared, the interrupt set last
	reg->CC[TrigNo] = pTimer->LastCount + cc;
	reg->EVENTS_COMPARE[TrigNo] = 0;
	reg->INTENSET = TIMER_INTENSET_COMPARE0_Msk << TrigNo;

	return tdata.pTrigger[TrigNo].nsPeriod; // Return real period in nsec
}

static void nRFxTimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];
	NRF_TIMER_Type *reg = tdata.pReg;
	PeiDev_t *pei;
	PeiEvent_t evt;
	PeiEndpoint_t endpoint;

	if (TrigNo < 0 || TrigNo >= tdata.MaxNbTrigEvt)
	{
		return;
	}

	if (tdata.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_SINGLE ||
		tdata.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_CONTINUOUS)
	{
		pei = static_cast<PeiDev_t *>(tdata.pTrigger[TrigNo].pContext);
		evt = tdata.pCC[TrigNo];
		endpoint = (PeiEndpoint_t)&reg->EVENTS_COMPARE[TrigNo];

		// Keep COMPARE enabled if PEI unbinding fails.
		if (PeiUnbindEvent == nullptr ||
			!PeiUnbindEvent(pei, evt, endpoint))
		{
			return;
		}
	}
	tdata.pCC[TrigNo] = 0;
    reg->CC[TrigNo] = 0;
    reg->INTENCLR = TIMER_INTENSET_COMPARE0_Msk << TrigNo;

	tdata.pTrigger[TrigNo].Type = TIMER_TRIG_TYPE_SINGLE;
	tdata.pTrigger[TrigNo].Handler = NULL;
	tdata.pTrigger[TrigNo].pContext = NULL;
	tdata.pTrigger[TrigNo].nsPeriod = 0;
}

static int nRFxTimerFindAvailTrigger(TimerDev_t * const pTimer)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];

	for (int i = 0; i < tdata.MaxNbTrigEvt; i++)
	{
		if (tdata.pTrigger[i].nsPeriod == 0)
			return i;
	}

	return -1;
}

static int nRFxTimerGetMaxTrigger(TimerDev_t * const pTimer)
{
	return s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX].MaxNbTrigEvt;
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static inline void nRFxTimerDisableExtTrigger(TimerDev_t * const pTimer) {
}

/**
 * @brief	Enable external timer trigger event.
 *
 * @param   TrigDevNo : External trigger device number to enable. Index value starting at 0
 * @param   pContext : Optional pointer to user private data to be passed
 *                     to the callback. This could be a class or structure pointer.
 *
 * @return  true - Success
 */
bool nRFxTimerEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	// No support for external trigger
	return false;
}

bool nRFxTimerInit(TimerDev_t *const pTimer, const TimerCfg_t *const pCfg)
{
	// Any frequency, SetFrequency takes the closest one
	if (pCfg->DevNo < TIMER_NRFX_RTC_MAX
			|| pCfg->DevNo >= (TIMER_NRFX_HF_MAX + TIMER_NRFX_RTC_MAX))
	{
		return false;
	}

	int devno = pCfg->DevNo - TIMER_NRFX_RTC_MAX;
	pTimer->DevNo = pCfg->DevNo;
	pTimer->EvtHandler = pCfg->EvtHandler;
	s_pnRFxTimerData = s_nRFxTimerData;

	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];
	NRF_TIMER_Type *reg = s_nRFxTimerData[devno].pReg;

	// A live PEI endpoint must be unbound explicitly before reinitializing
	// the underlying timer, even if a different Timer object is supplied.
	for (int i = 0; i < tdata.MaxNbTrigEvt; i++)
	{
		if (tdata.pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE ||
			tdata.pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
		{
			return false;
		}
	}
	s_pnRFxTimerDev[devno] = pTimer;
	s_bnRFxTimerHwClear[devno] = false;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;

	memset(tdata.pTrigger, 0, tdata.MaxNbTrigEvt * sizeof(TimerTrig_t));

	pTimer->Disable = nRFxTimerDisable;
	pTimer->Enable = nRFxTimerEnable;
	pTimer->Reset = nRFxTimerReset;
	pTimer->GetTickCount = nRFxTimerGetTickCount;
	pTimer->SetFrequency = nRFxTimerSetFrequency;
	pTimer->GetMaxTrigger = nRFxTimerGetMaxTrigger;
	pTimer->FindAvailTrigger = nRFxTimerFindAvailTrigger;
	pTimer->DisableTrigger = nRFxTimerDisableTrigger;
	pTimer->EnableTrigger = nRFxTimerEnableTrigger;
	pTimer->GetTaskEndpoint = nRFxTimerTaskEndpoint;
	pTimer->ExecuteTask = nRFxTimerExecuteTask;
	pTimer->DisableExtTrigger = nRFxTimerDisableExtTrigger;
	pTimer->EnableExtTrigger = nRFxTimerEnableExtTrigger;

	reg->TASKS_STOP = 1;
	reg->TASKS_CLEAR = 1;

	// No short or interrupt left by a counter that had the TIMER
	reg->SHORTS = 0;
	reg->INTENCLR = 0xFFFFFFFFUL;

	// Only support timer mode, 32-bit counter
	reg->MODE = TIMER_MODE_MODE_Timer;
	reg->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

	// Init leaves the timer running, so it goes through Enable. Enable
	// requests the crystal for the first timer and counts the timer once, so
	// that Disable after Init stops the timer and releases the crystal, and a
	// later Enable does not count it again.
	if (nRFxTimerEnable(pTimer) == false)
	{
		return false;
	}

	// Init interrupt for TIMERs
	switch (devno)
	{
	case 0:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		NVIC_ClearPendingIRQ(TIMER00_IRQn);
		NVIC_SetPriority(TIMER00_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER00_IRQn);
#else
		NVIC_ClearPendingIRQ(TIMER0_IRQn);
		NVIC_SetPriority(TIMER0_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER0_IRQn);
#endif
		break;
	case 1:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		NVIC_ClearPendingIRQ(TIMER10_IRQn);
		NVIC_SetPriority(TIMER10_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER10_IRQn);
#else
		NVIC_ClearPendingIRQ(TIMER1_IRQn);
		NVIC_SetPriority(TIMER1_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER1_IRQn);
#endif
		break;
	case 2:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		NVIC_ClearPendingIRQ(TIMER20_IRQn);
		NVIC_SetPriority(TIMER20_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER20_IRQn);
#else
		NVIC_ClearPendingIRQ(TIMER2_IRQn);
		NVIC_SetPriority(TIMER2_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER2_IRQn);
#endif
		break;
#if TIMER_NRFX_HF_MAX > 3
	case 3:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		NVIC_ClearPendingIRQ(TIMER21_IRQn);
		NVIC_SetPriority(TIMER21_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER21_IRQn);
#else
		NVIC_ClearPendingIRQ(TIMER3_IRQn);
		NVIC_SetPriority(TIMER3_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER3_IRQn);
#endif
		break;
	case 4:
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		NVIC_ClearPendingIRQ(TIMER22_IRQn);
		NVIC_SetPriority(TIMER22_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER22_IRQn);
#else
		NVIC_ClearPendingIRQ(TIMER4_IRQn);
		NVIC_SetPriority(TIMER4_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER4_IRQn);
#endif
		break;
#endif
#if TIMER_NRFX_HF_MAX > 5
	case 5:
		NVIC_ClearPendingIRQ(TIMER23_IRQn);
		NVIC_SetPriority(TIMER23_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER23_IRQn);
		break;
	case 6:
		NVIC_ClearPendingIRQ(TIMER24_IRQn);
		NVIC_SetPriority(TIMER24_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(TIMER24_IRQn);
		break;
#endif
	}


	nRFxTimerSetFrequency(pTimer, pCfg->Freq);

	return true;
}

