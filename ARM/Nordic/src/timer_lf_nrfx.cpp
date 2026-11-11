/**-------------------------------------------------------------------------
@file	timer_lf_nrfx.cpp

@brief	timer class implementation on Nordic nRFx series using the RTC (real time counter)

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

#include "timer_nrfx.h"
#include "coredev/pei.h"
#include "coredev/interrupt.h"

// PEI is an optional Timer feature. Weak references let ordinary RTC
// applications link without the PEI matrix or nRF52 PPI backend.
#if defined(__GNUC__)
#pragma weak PeiBindEvent
#pragma weak PeiUnbindEvent
#pragma weak PeiReleaseEvent
#pragma weak PeiGetEventEndpoint
#pragma weak PeiConnectTask
#pragma weak PeiDisconnectTask
#endif

#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
#define NRF_CLOCK		NRF_CLOCK_NS
#define NRF_RTC0		NRF_RTC0_NS
#define NRF_RTC1		NRF_RTC1_NS
#else
#if defined(NRF91_SERIES)
// Non secure on nRF91, as POWER, which the Modem library uses there
// (nrfx_config_nrf91.h)
#define NRF_CLOCK		NRF_CLOCK_NS
#else
#define NRF_CLOCK		NRF_CLOCK_S
#endif
#define NRF_RTC0		NRF_RTC0_S
#define NRF_RTC1		NRF_RTC1_S
#define NRF_RTC2		NRF_RTC1_S
#endif
#define CLOCK_LFCLKSRC_SRC_RC		CLOCK_LFCLKSRC_SRC_LFRC
#define CLOCK_LFCLKSRC_SRC_Xtal		CLOCK_LFCLKSRC_SRC_LFXO
#endif

// PEI compare channels share ordinary TimerTrig_t state, without
// reserving other CC channels or stopping/clearing the elapsed counter.
static constexpr TIMER_TRIG_TYPE TIMER_TRIG_PEI_SINGLE =
	static_cast<TIMER_TRIG_TYPE>(2);
static constexpr TIMER_TRIG_TYPE TIMER_TRIG_PEI_CONTINUOUS =
	static_cast<TIMER_TRIG_TYPE>(3);

#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
#include "../nRF52/include/pei_rtc_state.h"
#if defined(__GNUC__)
#pragma weak g_PeiNrf52RtcRaw
#endif
typedef PeiNrf52RtcRawState_t nRFRtcRawPei_t;
#define s_nRfxRtcRaw g_PeiNrf52RtcRaw
#define RTC_OPTIONAL_PEI_STATE 1
#else
typedef struct {
	PeiDev_t *pPei;
	PeiEvent_t Event;
	bool EventWasEnabled;
} nRFRtcRawPei_t;
static nRFRtcRawPei_t s_nRfxRtcRaw[TIMER_NRFX_RTC_MAX][2];
#endif

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	uint32_t MaxFreq;
    NRF_RTC_Type *pReg;
    int MaxNbTrigEvt;		//!< Number of trigger is not the same for all timers.
	uint32_t *pCC;			//!< Compare value of each trigger, MaxNbTrigEvt entries
	TimerTrig_t *pTrigger;	//!< State of each trigger, MaxNbTrigEvt entries
} nRFRtcData_t;
#pragma pack(pop)

// Trigger state of each timer, sized by the number of compare registers of
// that timer.
static uint32_t s_nRfxRtc0CC[RTC0_CC_NUM];
static TimerTrig_t s_nRfxRtc0Trig[RTC0_CC_NUM];
static uint32_t s_nRfxRtc1CC[RTC1_CC_NUM];
static TimerTrig_t s_nRfxRtc1Trig[RTC1_CC_NUM];
#if TIMER_NRFX_RTC_MAX > 2
static uint32_t s_nRfxRtc2CC[RTC2_CC_NUM];
static TimerTrig_t s_nRfxRtc2Trig[RTC2_CC_NUM];
#endif

// Timer device of each timer, set by nRFxRtcInit
static TimerDev_t *s_pnRfxRtcDev[TIMER_NRFX_RTC_MAX];

// What does not change is constant, so it stays in flash. Only the trigger
// state above and the device pointers are in RAM.
alignas(4) static const nRFRtcData_t s_nRfxRtcData[TIMER_NRFX_RTC_MAX] = {
	// RTC LF timer first
	{
		.DevNo = 0,
		.MaxFreq = TIMER_NRFX_RTC_BASE_FREQ,
		.pReg = NRF_RTC0,
		.MaxNbTrigEvt = RTC0_CC_NUM,
		.pCC = s_nRfxRtc0CC,
		.pTrigger = s_nRfxRtc0Trig,
	},
	{
		.DevNo = 1,
		.MaxFreq = TIMER_NRFX_RTC_BASE_FREQ,
		.pReg = NRF_RTC1,
		.MaxNbTrigEvt = RTC1_CC_NUM,
		.pCC = s_nRfxRtc1CC,
		.pTrigger = s_nRfxRtc1Trig,
	},
#if TIMER_NRFX_RTC_MAX > 2
	{
		.DevNo = 2,
		.MaxFreq = TIMER_NRFX_RTC_BASE_FREQ,
		.pReg = NRF_RTC2,
		.MaxNbTrigEvt = RTC2_CC_NUM,
		.pCC = s_nRfxRtc2CC,
		.pTrigger = s_nRfxRtc2Trig,
	},
#endif
};

static std::atomic<int> s_nRfxLFClockSem(0);
static bool s_bRtcEnabled[TIMER_NRFX_RTC_MAX];
static bool s_bLFClockOwned = false;
// CLEAR task endpoint given out: a hardware event can clear the counter
static bool s_bRtcHwClear[TIMER_NRFX_RTC_MAX];

// RTC section of the product specifications:
// - STOP and CLEAR take effect 15 to 46 us after the task, up to about 250 us
//   when the LFCLK is not running. Bound of the waits for them, in register
//   reads.
// - A compare written with N + 2 while COUNTER is N matches; with N or N + 1
//   it may not.
#define RTC_TASK_WAIT_MAX		10000
#define RTC_CC_AHEAD_MIN		2U
#define RTC_COUNTER_MASK		0xFFFFFFU

// Ticks around a wrap, 2 s at 32768 Hz: the interrupt handler counts a wrap on
// OVRFLW well within this time.
#define RTC_NEAR_WRAP			0x10000U

// Set by nRFxRtcInit. The interrupt handlers below are kept by the linker as soon as
// this file is part of a build, used or not. They reach the timer data
// through this pointer, so that the data is linked only when a timer is
// initialized.
static const nRFRtcData_t *s_pnRfxRtcData = nullptr;

// Tick count: COUNTER plus the wraps counted by the interrupt handler on
// OVRFLW. Here for a COUNTER value read near a wrap, see nRFxRtcGetTickCount,
// or once the CLEAR endpoint is given out, interrupts masked:
// - before a wrap, COUNTER is read again: a wrap since the first read is taken
//   as after a wrap;
// - after a wrap the handler did not count yet, OVRFLW set, the wrap is added;
// - once the CLEAR endpoint is given out, a COUNTER lower than the last one
//   read is read again: lower again, it is a hardware clear, the tick count
//   restarts from it; not lower, there was a read from a higher priority in
//   between.
static __attribute__((noinline)) uint64_t nRFxRtcReadCountMasked(TimerDev_t * const pTimer,
																 NRF_RTC_Type *pReg, uint32_t Count)
{
	uint32_t state = DisableInterrupt();
	uint64_t rollover = pTimer->Rollover;

	if (Count > RTC_COUNTER_MASK - RTC_NEAR_WRAP)
	{
		uint32_t count = pReg->COUNTER;

		if (count < Count)
		{
			Count = count;
		}
	}

	if (Count < RTC_NEAR_WRAP && pReg->EVENTS_OVRFLW)
	{
		rollover += 0x1000000ULL;
	}
	else if (Count < pTimer->LastCount && s_bRtcHwClear[pTimer->DevNo])
	{
		Count = pReg->COUNTER;
		if (Count < pTimer->LastCount)
		{
			pTimer->Rollover = 0;
			rollover = 0;
		}
	}
	pTimer->LastCount = Count;

	EnableInterrupt(state);

	return rollover + Count;
}

// Compare CC[Idx] = COUNTER + Ticks, Ticks 2 or more, written right after the
// COUNTER read, interrupts masked by the caller. COUNTER can increment once in
// between: with 3 ticks or more the compare is still 2 ticks ahead of it.
// With 2 ticks COUNTER is read again: if it moved, the compare is moved one
// tick later. The first value can still match. CC takes the 24 low bits.
static inline __attribute__((always_inline)) void nRFxRtcSetCompare(NRF_RTC_Type *pReg, int Idx,
								uint32_t Ticks)
{
	volatile uint32_t *cc = &pReg->CC[Idx];

	if (Ticks > RTC_CC_AHEAD_MIN)
	{
		*cc = pReg->COUNTER + Ticks;
	}
	else
	{
		uint32_t count = pReg->COUNTER;

		*cc = count + Ticks;
		if (pReg->COUNTER != count)
		{
			*cc = count + Ticks + 1U;
		}
	}
}

// Period in ticks of a continuous trigger
static inline uint32_t nRFxRtcPeriod(TimerDev_t * const pTimer, const nRFRtcData_t &Rtc, int TrigNo)
{
	if (Rtc.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_CONTINUOUS)
	{
		return (uint32_t)TimerNanosecondToTick(pTimer, Rtc.pTrigger[TrigNo].nsPeriod);
	}

	return Rtc.pCC[TrigNo];
}

// Next compares of the continuous triggers in Mask, already set one period
// after the compare that matched by the interrupt handler, before COUNTER is
// read. A compare at least 2 ticks ahead of COUNTER matches. Closer, it may
// match or not: decided once COUNTER passed it, within 2 ticks, the wait
// bounded. A compare behind COUNTER that did not match, the handler being
// late, is set to the next period at least 3 ticks ahead, the periods passed
// skipped. Once a hardware event can clear the counter, the next compare is
// one period from COUNTER.
static void nRFxRtcCheckCompares(TimerDev_t * const pTimer, const nRFRtcData_t &Rtc, uint32_t Mask)
{
	NRF_RTC_Type *reg = Rtc.pReg;
	bool hwclear = s_bRtcHwClear[pTimer->DevNo];
	uint32_t now = reg->COUNTER;

	while (Mask != 0U)
	{
		int i = __builtin_ctz(Mask);

		Mask &= Mask - 1U;

		uint32_t cc = reg->CC[i];
		uint32_t ahead = (cc - now) & RTC_COUNTER_MASK;

		if (hwclear == false)
		{
			if (ahead >= RTC_CC_AHEAD_MIN && ahead < (RTC_COUNTER_MASK >> 1))
			{
				continue;
			}
			for (int timeout = RTC_TASK_WAIT_MAX; ahead < RTC_CC_AHEAD_MIN && timeout > 0; timeout--)
			{
				now = reg->COUNTER;
				ahead = (cc - now) & RTC_COUNTER_MASK;
			}
			if (ahead < RTC_CC_AHEAD_MIN || reg->EVENTS_COMPARE[i])
			{
				continue;
			}
		}

		uint32_t period = nRFxRtcPeriod(pTimer, Rtc, i);
		uint32_t state = DisableInterrupt();

		now = reg->COUNTER;
		ahead = hwclear ? period : period - ((now - cc) & RTC_COUNTER_MASK) % period;
		if (ahead <= RTC_CC_AHEAD_MIN)
		{
			ahead += period;
		}
		reg->CC[i] = now + ahead;

		EnableInterrupt(state);
	}
}

static void RtcIRQHandler(int DevNo)
{
	if (s_pnRfxRtcData == nullptr)
	{
		// No timer of this kind was initialized
		return;
	}

	TimerDev_t *timer = s_pnRfxRtcDev[DevNo];
	if (timer == nullptr)
	{
		return;
	}

	const nRFRtcData_t &rtc = s_pnRfxRtcData[DevNo];
	NRF_RTC_Type *reg = rtc.pReg;
    uint32_t evt = 0;
	uint32_t ticks;
	uint32_t inten = reg->INTENSET;

    if ((inten & RTC_INTENSET_TICK_Msk) && reg->EVENTS_TICK)
	{
        evt |= TIMER_EVT_TICK;
        reg->EVENTS_TICK = 0;
	}

    if (reg->EVENTS_OVRFLW)
	{
		uint32_t state = DisableInterrupt();

        reg->EVENTS_OVRFLW = 0;
		timer->Rollover += 0x1000000ULL;
		timer->LastCount = 0;

		EnableInterrupt(state);

        evt |= TIMER_EVT_COUNTER_OVR;
	}

	// Only the compares of the armed triggers, their interrupt set
	uint32_t armed = (inten >> RTC_INTENSET_COMPARE0_Pos) & ((1U << rtc.MaxNbTrigEvt) - 1U);

	// Continuous triggers whose next compare is set, checked against COUNTER
	// after the handlers
	uint32_t next = 0;

	while (armed != 0U)
	{
		int i = __builtin_ctz(armed);

		armed &= armed - 1U;

		if (reg->EVENTS_COMPARE[i] == 0U)
		{
			continue;
		}

		reg->EVENTS_COMPARE[i] = 0U;

		TIMER_TRIG_TYPE type = rtc.pTrigger[i].Type;

		if (type == TIMER_TRIG_TYPE_CONTINUOUS || type == TIMER_TRIG_TYPE_SINGLE)
		{
			TimerTrigEvtHandler_t handler = rtc.pTrigger[i].Handler;
			void *ctx = rtc.pTrigger[i].pContext;

			if (type == TIMER_TRIG_TYPE_CONTINUOUS)
			{
				// Next compare one period after the one that matched, first.
				// CC takes the 24 low bits.
				reg->CC[i] = reg->CC[i] + rtc.pCC[i];
				next |= 1U << i;
			}
			else
			{
				// Single shot done, the trigger is free before its handler,
				// which can set it again: interrupt and event off, its
				// compare would match again after a wrap of the counter, and
				// no period, see nRFxRtcFindAvailTrigger.
				reg->INTENCLR = RTC_INTENCLR_COMPARE0_Msk << i;
				reg->EVTENCLR = RTC_EVTEN_COMPARE0_Msk << i;
				rtc.pTrigger[i].nsPeriod = 0;
			}

			evt |= TIMER_EVT_TRIGGER(i);
			if (handler != nullptr)
			{
				handler(timer, i, ctx);
			}
		}
		else if (type == TIMER_TRIG_PEI_CONTINUOUS)
		{
			// Only this CC is rescheduled; PEI routes the event in hardware.
			ticks = (uint32_t)TimerNanosecondToTick(timer, rtc.pTrigger[i].nsPeriod);
			reg->CC[i] = reg->CC[i] + ticks;
			next |= 1U << i;
		}
		else if (type == TIMER_TRIG_PEI_SINGLE)
		{
			reg->INTENCLR = RTC_INTENCLR_COMPARE0_Msk << i;
			reg->EVTENCLR = RTC_EVTEN_COMPARE0_Msk << i;
		}
	}

	if (next != 0U)
	{
		nRFxRtcCheckCompares(timer, rtc, next);
	}

    if (timer->EvtHandler)
	{
        timer->EvtHandler(timer, evt);
	}
}

extern "C" {

__WEAK void RTC0_IRQHandler()
{
	RtcIRQHandler(0);
}

__WEAK void RTC1_IRQHandler()
{
	RtcIRQHandler(1);
}
#if TIMER_NRFX_RTC_MAX > 2
__WEAK void RTC2_IRQHandler()
{
	RtcIRQHandler(2);
}
#endif

}   // extern "C"


static bool nRFxRtcEnable(TimerDev_t * const pTimer)
{
	if (s_bRtcEnabled[pTimer->DevNo])
	{
		return true;
	}

	if (!(NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk))
	{
		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
		NRF_CLOCK->TASKS_LFCLKSTART = 1;

		bool started = false;
		for (int timeout = 1000000; timeout > 0; timeout--)
		{
			if ((NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) &&
				NRF_CLOCK->EVENTS_LFCLKSTARTED)
			{
				started = true;
				break;
			}
		}

		if (!started)
		{
			NRF_CLOCK->TASKS_LFCLKSTOP = 1;
			return false;
		}

		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
		s_bLFClockOwned = true;
	}

	s_bRtcEnabled[pTimer->DevNo] = true;
    s_nRfxLFClockSem++;

    switch (pTimer->DevNo)
	{
        case 0:
            NVIC_ClearPendingIRQ(RTC0_IRQn);
            NVIC_EnableIRQ(RTC0_IRQn);
            break;
        case 1:
			NVIC_ClearPendingIRQ(RTC1_IRQn);
            NVIC_EnableIRQ(RTC1_IRQn);
            break;
#if TIMER_NRFX_RTC_MAX > 2
        case 2:
            NVIC_ClearPendingIRQ(RTC2_IRQn);
            NVIC_EnableIRQ(RTC2_IRQn);
            break;
#endif
	}

    s_nRfxRtcData[pTimer->DevNo].pReg->TASKS_START = 1;

    return true;
}


static void nRFxRtcDisable(TimerDev_t * const pTimer)
{
	if (!s_bRtcEnabled[pTimer->DevNo])
	{
		return;
	}

	s_bRtcEnabled[pTimer->DevNo] = false;
	s_nRfxRtcData[pTimer->DevNo].pReg->TASKS_STOP = 1;

    switch (pTimer->DevNo)
	{
        case 0:
        	NVIC_ClearPendingIRQ(RTC0_IRQn);
            NVIC_DisableIRQ(RTC0_IRQn);
            break;
        case 1:
        	NVIC_ClearPendingIRQ(RTC1_IRQn);
            NVIC_DisableIRQ(RTC1_IRQn);
            break;
#if TIMER_NRFX_RTC_MAX > 2
        case 2:
            NVIC_DisableIRQ(RTC2_IRQn);
            NVIC_ClearPendingIRQ(RTC2_IRQn);
            break;
#endif
	}

    s_nRfxLFClockSem--;

	if (s_nRfxLFClockSem == 0 && s_bLFClockOwned)
	{
		NRF_CLOCK->TASKS_LFCLKSTOP = 1;
		s_bLFClockOwned = false;
	}
}

// CLEAR task, returns once COUNTER reads lower than before it, bounded:
// the task takes effect some LFCLK edges later.
static void nRFxRtcClear(NRF_RTC_Type *pReg)
{
	uint32_t count = pReg->COUNTER;

	pReg->TASKS_CLEAR = 1;
	for (int timeout = RTC_TASK_WAIT_MAX; count > 0U && timeout > 0; timeout--)
	{
		if (pReg->COUNTER < count)
		{
			break;
		}
	}
}

// The counter is cleared, the tick count restarts and the armed triggers
// restart their period from the clear.
static void nRFxRtcReset(TimerDev_t * const pTimer)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];
	NRF_RTC_Type *reg = rtc.pReg;

	nRFxRtcClear(reg);

	uint32_t state = DisableInterrupt();
	uint32_t count = reg->COUNTER;

	// An overflow from before the clear is not counted
	reg->EVENTS_OVRFLW = 0;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;

	for (int i = 0; i < rtc.MaxNbTrigEvt; i++)
	{
		uint32_t cc;

		if (rtc.pTrigger[i].nsPeriod == 0U ||
			(reg->INTENSET & (RTC_INTENSET_COMPARE0_Msk << i)) == 0U)
		{
			continue;
		}
		if (rtc.pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE ||
			rtc.pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
		{
			cc = (uint32_t)TimerNanosecondToTick(pTimer, rtc.pTrigger[i].nsPeriod);
		}
		else
		{
			cc = rtc.pCC[i];
		}
		reg->EVENTS_COMPARE[i] = 0;

		// At least 3 ticks ahead of COUNTER, see nRFxRtcSetCompare
		reg->CC[i] = cc < count + RTC_CC_AHEAD_MIN + 1U ? count + RTC_CC_AHEAD_MIN + 1U : cc;
	}

	EnableInterrupt(state);
}

// Closest of 32768 / (PRESCALER + 1), PRESCALER 0 to 4095, the higher of two
// as close, 32768 Hz for 0. PRESCALER is written only while the RTC is
// stopped, which is 15 to 46 us after the STOP task: written until it reads
// back, bounded. A disabled RTC stays stopped.
static uint32_t nRFxRtcSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];
	NRF_RTC_Type *reg = rtc.pReg;

	for (int i = 0; i < rtc.MaxNbTrigEvt; i++)
	{
		if (rtc.pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE ||
			rtc.pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
		{
			return pTimer->Freq;
		}
	}

	uint32_t div = 1;

	if (Freq > 0U && Freq < TIMER_NRFX_RTC_BASE_FREQ)
	{
		div = TIMER_NRFX_RTC_BASE_FREQ / Freq;
		if (div > RTC_PRESCALER_PRESCALER_Msk)
		{
			div = RTC_PRESCALER_PRESCALER_Msk + 1U;
		}
		else if ((uint64_t)TIMER_NRFX_RTC_BASE_FREQ * (2U * div + 1U) >
				 2ULL * Freq * div * (div + 1U))
		{
			// base / div - Freq > Freq - base / (div + 1)
			div++;
		}
	}

	reg->TASKS_STOP = 1;
	for (int timeout = RTC_TASK_WAIT_MAX; timeout > 0; timeout--)
	{
		reg->PRESCALER = div - 1U;
		if (reg->PRESCALER == div - 1U)
		{
			break;
		}
	}

	// From what PRESCALER holds
	div = (reg->PRESCALER & RTC_PRESCALER_PRESCALER_Msk) + 1U;
	pTimer->Freq = (TIMER_NRFX_RTC_BASE_FREQ + div / 2U) / div;

	// Tick period rounded down to a nanosecond, 30517 for the 30517.58 ns of
	// 32768 Hz. Not precise enough to convert a count or a trigger period,
	// those are computed from Freq, see TimerTickToTime.
    pTimer->nsPeriod = 1000000000ULL / (uint64_t)pTimer->Freq;     // Period in nsec

	if (s_bRtcEnabled[pTimer->DevNo])
	{
		reg->TASKS_START = 1;
	}

    return pTimer->Freq;
}

// Tick count, see nRFxRtcReadCountMasked. A COUNTER away from a wrap is used
// as read: the interrupt handler does not change the wraps counted until the
// next wrap.
static uint64_t nRFxRtcGetTickCount(TimerDev_t * const pTimer)
{
	NRF_RTC_Type *reg = s_nRfxRtcData[pTimer->DevNo].pReg;
	uint32_t count = reg->COUNTER;

	if (count - RTC_NEAR_WRAP < RTC_COUNTER_MASK + 1U - 2U * RTC_NEAR_WRAP)
	{
		return pTimer->Rollover + count;
	}

	return nRFxRtcReadCountMasked(pTimer, reg, count);
}

// Tick count once the CLEAR endpoint is given out: each read is kept to find
// a hardware clear, see nRFxRtcReadCountMasked
static uint64_t nRFxRtcGetTickCountHwClear(TimerDev_t * const pTimer)
{
	NRF_RTC_Type *reg = s_nRfxRtcData[pTimer->DevNo].pReg;

	return nRFxRtcReadCountMasked(pTimer, reg, reg->COUNTER);
}

static int nRFxRtcGetMaxTrigger(TimerDev_t * const pTimer)
{
	return s_nRfxRtcData[pTimer->DevNo].MaxNbTrigEvt;
}

// RTC TASKS_START and TASKS_STOP are genuine hardware task endpoints.
// They can be connected by PEI without introducing a GPIO or RTC ISR.
// The Timer driver retains ownership of clock and RTC configuration.
static uintptr_t nRFxRtcTaskEndpoint(TimerDev_t * const pTimer, TIMER_TASK Task)
{
	int devno;
	NRF_RTC_Type *reg;

	if (pTimer == nullptr)
	{
		return 0U;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX ||
		s_pnRfxRtcDev[devno] != pTimer)
	{
		return 0U;
	}
	reg = s_nRfxRtcData[devno].pReg;
	switch (Task)
	{
		case TIMER_TASK_START:
			return (uintptr_t)&reg->TASKS_START;
		case TIMER_TASK_STOP:
			return (uintptr_t)&reg->TASKS_STOP;
		case TIMER_TASK_CLEAR:
			// From now on a lower COUNTER without wrap is a clear
			if (s_bRtcHwClear[devno] == false)
			{
				uint32_t state = DisableInterrupt();

				pTimer->LastCount = reg->COUNTER;
				s_bRtcHwClear[devno] = true;
				pTimer->GetTickCount = nRFxRtcGetTickCountHwClear;

				EnableInterrupt(state);
			}
			return (uintptr_t)&reg->TASKS_CLEAR;
		case TIMER_TASK_TRIGOVRFLW:
			return (uintptr_t)&reg->TASKS_TRIGOVRFLW;
		default:
			return 0U;
	}
}

static bool nRFxRtcExecuteTask(TimerDev_t * const pTimer, TIMER_TASK Task)
{
	uintptr_t endpoint;

	if (Task == TIMER_TASK_CLEAR)
	{
		if (nRFxRtcTaskEndpoint(pTimer, TIMER_TASK_START) == 0U)
		{
			return false;
		}
		// From the CPU, the tick count and the triggers follow the clear
		nRFxRtcReset(pTimer);
		return true;
	}

	endpoint = nRFxRtcTaskEndpoint(pTimer, Task);
	if (endpoint == 0U)
	{
		return false;
	}
	*(volatile uint32_t *)endpoint = 1U;
	return true;
}

// Bind an RTC TICK or OVRFLW producer without changing its interrupt
// configuration. EVTEN is independent from INTEN on nRF52.
bool nRFxRtcEnablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvent, uint32_t Event)
{
	nRFRtcRawPei_t *pRaw;
	NRF_RTC_Type *reg;
	PeiEndpoint_t endpoint;
	uint32_t eventBit;
	int devno;
	int idx;

	if (pTimer == nullptr || pPei == nullptr || pEvent == nullptr ||
		PeiBindEvent == nullptr || PeiUnbindEvent == nullptr ||
		pPei->DevNo != 0)
	{
		return false;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX ||
		s_pnRfxRtcDev[devno] != pTimer)
	{
		return false;
	}
	idx = -1;
	if (Event == TIMER_EVT_TICK)
	{
		idx = 0;
	}
	else if (Event == TIMER_EVT_COUNTER_OVR)
	{
		idx = 1;
	}
	if (idx < 0)
	{
		return false;
	}
	pRaw = &s_nRfxRtcRaw[devno][idx];
	if (pRaw->pPei != nullptr)
	{
		return false;
	}
	reg = s_nRfxRtcData[devno].pReg;
	if (idx == 0)
	{
		eventBit = RTC_EVTENSET_TICK_Msk;
		endpoint = (PeiEndpoint_t)&reg->EVENTS_TICK;
	}
	else
	{
		eventBit = RTC_EVTENSET_OVRFLW_Msk;
		endpoint = (PeiEndpoint_t)&reg->EVENTS_OVRFLW;
	}
	if (!PeiBindEvent(pPei, pEvent, endpoint))
	{
		return false;
	}
	pRaw->EventWasEnabled = (reg->EVTEN & eventBit) != 0U;
	pRaw->pPei = pPei;
	pRaw->Event = *pEvent;
	reg->EVTENSET = eventBit;
	return true;
}

// Unbind only this RTC event source and leave the shared counter running.
// The PEI matrix owns its event-to-task routes.
bool nRFxRtcDisablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t Event)
{
	nRFRtcRawPei_t *pRaw;
	const nRFRtcData_t *pData;
	NRF_RTC_Type *reg;
	PeiEndpoint_t endpoint;
	uint32_t bit;
	int devno;
	int i;

	if (pTimer == nullptr)
	{
		return false;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX ||
		s_pnRfxRtcDev[devno] != pTimer)
	{
		return false;
	}
	pData = &s_nRfxRtcData[devno];
	reg = pData->pReg;
	if (pPei == nullptr || Event == 0U || PeiUnbindEvent == nullptr)
	{
		return false;
	}
	for (i = 0; i < 2; i++)
	{
		pRaw = &s_nRfxRtcRaw[devno][i];
		if (pRaw->pPei != pPei || pRaw->Event != Event)
		{
			continue;
		}
		if (i == 0)
		{
			bit = RTC_EVTENSET_TICK_Msk;
			endpoint = (PeiEndpoint_t)&reg->EVENTS_TICK;
		}
		else
		{
			bit = RTC_EVTENSET_OVRFLW_Msk;
			endpoint = (PeiEndpoint_t)&reg->EVENTS_OVRFLW;
		}
		if (!PeiUnbindEvent(pPei, Event, endpoint))
		{
			return false;
		}
		if (!pRaw->EventWasEnabled)
		{
			reg->EVTENCLR = bit;
		}
		pRaw->pPei = nullptr;
		pRaw->Event = 0U;
		pRaw->EventWasEnabled = false;
		return true;
	}
	for (i = 0; i < pData->MaxNbTrigEvt; i++)
	{
		if ((pData->pTrigger[i].Type != TIMER_TRIG_PEI_SINGLE &&
			pData->pTrigger[i].Type != TIMER_TRIG_PEI_CONTINUOUS) ||
			pData->pTrigger[i].pContext != pPei ||
			pData->pCC[i] != Event)
		{
			continue;
		}
		endpoint = (PeiEndpoint_t)&reg->EVENTS_COMPARE[i];
		if (!PeiUnbindEvent(pPei, Event, endpoint))
		{
			return false;
		}
		reg->INTENCLR = RTC_INTENCLR_COMPARE0_Msk << i;
		reg->EVTENCLR = RTC_EVTEN_COMPARE0_Msk << i;
		reg->CC[i] = 0U;
		reg->EVENTS_COMPARE[i] = 0U;
		pData->pCC[i] = 0U;
		pData->pTrigger[i].Type = TIMER_TRIG_TYPE_SINGLE;
		pData->pTrigger[i].nsPeriod = 0U;
		pData->pTrigger[i].Handler = nullptr;
		pData->pTrigger[i].pContext = nullptr;
		return true;
	}
	return false;
}

// Bind a free CC to PEI; the RTC counter and all other CCs keep running.
// Continuous compare uses the same ISR reschedule rule as ordinary RTC.
bool nRFxRtcEnablePeiTrigger(TimerDev_t *pTimer, int TrigNo,
	uint64_t nsPeriod, TIMER_TRIG_TYPE Type, PeiDev_t *pPei,
	PeiEvent_t *pEvent)
{
	const nRFRtcData_t *pData;
	NRF_RTC_Type *reg;
	PeiEvent_t evt;
	PeiEndpoint_t endpoint;
	uint64_t ticks;
	uint32_t bit;
	uint32_t count;
	int devno;

	if (pTimer == nullptr || pPei == nullptr || pEvent == nullptr ||
		PeiBindEvent == nullptr || PeiUnbindEvent == nullptr ||
		pPei->DevNo != 0 || nsPeriod == 0U ||
		(Type != TIMER_TRIG_TYPE_SINGLE &&
			Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return false;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX ||
		s_pnRfxRtcDev[devno] != pTimer || !s_bRtcEnabled[devno] ||
		pTimer->Freq == 0U)
	{
		return false;
	}
	pData = &s_nRfxRtcData[devno];
	reg = pData->pReg;
	if (TrigNo < 0 || TrigNo >= pData->MaxNbTrigEvt)
	{
		return false;
	}
	bit = RTC_EVTEN_COMPARE0_Msk << TrigNo;
	if (pData->pTrigger[TrigNo].nsPeriod != 0U ||
		pData->pTrigger[TrigNo].Type == TIMER_TRIG_PEI_SINGLE ||
		pData->pTrigger[TrigNo].Type == TIMER_TRIG_PEI_CONTINUOUS ||
		(reg->EVTEN & bit) != 0U ||
		(reg->INTENSET & (RTC_INTENSET_COMPARE0_Msk << TrigNo)) != 0U)
	{
		return false;
	}
	ticks = TimerNanosecondToTick(pTimer, nsPeriod);
	if (ticks < 3U || ticks > 0xFFFFFEULL)
	{
		return false;
	}
	endpoint = (PeiEndpoint_t)&reg->EVENTS_COMPARE[TrigNo];
	evt = *pEvent;
	if (!PeiBindEvent(pPei, &evt, endpoint))
	{
		return false;
	}

	reg->EVENTS_COMPARE[TrigNo] = 0U;

	// COUNTER read and compare written with interrupts masked
	uint32_t state = DisableInterrupt();

	count = reg->COUNTER;
	reg->CC[TrigNo] = (count + (uint32_t)ticks) & RTC_COUNTER_MASK;

	EnableInterrupt(state);

	pData->pCC[TrigNo] = evt;
	pData->pTrigger[TrigNo].Type = Type == TIMER_TRIG_TYPE_CONTINUOUS ?
		TIMER_TRIG_PEI_CONTINUOUS : TIMER_TRIG_PEI_SINGLE;
	pData->pTrigger[TrigNo].nsPeriod = TimerTickToNanosecond(pTimer, ticks);
	pData->pTrigger[TrigNo].Handler = nullptr;
	pData->pTrigger[TrigNo].pContext = pPei;
	*pEvent = evt;
	reg->EVTENSET = bit;
	reg->INTENSET = RTC_INTENSET_COMPARE0_Msk << TrigNo;
	return true;
}

static uint64_t nRFxRtcEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                              	  	 TimerTrigEvtHandler_t Handler, void *pContext)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];

	if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt || nsPeriod == 0U)
	{
		return 0;
	}
	// CC channels are independent; do not overwrite an active one.
	if (rtc.pTrigger[TrigNo].nsPeriod != 0U ||
		rtc.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_SINGLE ||
		rtc.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_CONTINUOUS)
	{
		return 0;
	}

	// Ticks as TimerNanosecondToTick, one 64 bit division up to 10^14 ns
	// (nsPeriod * Freq fits 64 bits up to 32768 Hz). Closest supported: 2 ticks
	// at least, 0xFFFFFF at most (24 bit compare).
	uint64_t ticks;

	if (nsPeriod <= 100000000000000ULL)
	{
		ticks = (nsPeriod * pTimer->Freq + 500000000ULL) / 1000000000ULL;
	}
	else
	{
		ticks = TimerNanosecondToTick(pTimer, nsPeriod);
	}

	uint32_t cc = ticks < RTC_CC_AHEAD_MIN ? RTC_CC_AHEAD_MIN :
				  ticks > RTC_COUNTER_MASK ? RTC_COUNTER_MASK : (uint32_t)ticks;

	// The period starts at the COUNTER read. A compare left from before is
	// cleared first. COUNTER read and compare written with interrupts masked.
	rtc.pReg->EVENTS_COMPARE[TrigNo] = 0;

	uint32_t state = DisableInterrupt();

	nRFxRtcSetCompare(rtc.pReg, TrigNo, cc);

	EnableInterrupt(state);

	// The trigger is set before its interrupt. Its period in nanoseconds,
	// rounded to the ticks, is set after: until then nsPeriod holds the
	// requested one.
	rtc.pTrigger[TrigNo].Type = Type;
	rtc.pTrigger[TrigNo].nsPeriod = nsPeriod;
	rtc.pTrigger[TrigNo].Handler = Handler;
	rtc.pTrigger[TrigNo].pContext = pContext;
	rtc.pCC[TrigNo] = cc;

    rtc.pReg->EVTENSET = RTC_EVTEN_COMPARE0_Msk << TrigNo;
    rtc.pReg->INTENSET = RTC_INTENSET_COMPARE0_Msk << TrigNo;

	uint64_t ns = TimerTickToNanosecond(pTimer, cc);

	// Not when a single shot already fired: freed, or set again by its handler
	state = DisableInterrupt();
	if (rtc.pTrigger[TrigNo].nsPeriod == nsPeriod)
	{
		rtc.pTrigger[TrigNo].nsPeriod = ns;
	}
	EnableInterrupt(state);

	return ns; // Return real period in nsec
}

static void nRFxRtcDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];
	PeiDev_t *pPei;
	PeiEndpoint_t endpoint;

	if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt)
	{
		return;
	}
	if (rtc.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_SINGLE ||
		rtc.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_CONTINUOUS)
	{
		pPei = static_cast<PeiDev_t *>(rtc.pTrigger[TrigNo].pContext);
		endpoint = (PeiEndpoint_t)&rtc.pReg->EVENTS_COMPARE[TrigNo];

		if (PeiUnbindEvent == nullptr ||
			!PeiUnbindEvent(pPei, rtc.pCC[TrigNo], endpoint))
		{
			return;
		}
	}

	rtc.pCC[TrigNo] = 0U;
	rtc.pReg->CC[TrigNo] = 0U;
	rtc.pReg->EVENTS_COMPARE[TrigNo] = 0U;
	rtc.pReg->EVTENCLR = RTC_EVTEN_COMPARE0_Msk << TrigNo;
	rtc.pReg->INTENCLR = RTC_INTENCLR_COMPARE0_Msk << TrigNo;
	rtc.pTrigger[TrigNo].Type = TIMER_TRIG_TYPE_SINGLE;
	rtc.pTrigger[TrigNo].Handler = nullptr;
	rtc.pTrigger[TrigNo].pContext = nullptr;
	rtc.pTrigger[TrigNo].nsPeriod = 0U;
}

static int nRFxRtcFindAvailTrigger(TimerDev_t * const pTimer)
{
	for (int i = 0; i < s_nRfxRtcData[pTimer->DevNo].MaxNbTrigEvt; i++)
	{
		if (s_nRfxRtcData[pTimer->DevNo].pTrigger[i].nsPeriod == 0U)
		{
			return i;
		}
	}

	return -1;
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static inline void nRFxRtcDisableExtTrigger(TimerDev_t * const) {
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
bool nRFxRtcEnableExtTrigger(TimerDev_t * const, int, TIMER_EXTTRIG_SENSE)
{
	// No support for external trigger
	return false;
}

bool nRFxRtcInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	const nRFRtcData_t *pRtc;
	NRF_RTC_Type *reg;
	TimerDev_t *pOldTimer;
	int i;

	// Any frequency, SetFrequency takes the closest one
	if (pTimer == nullptr || pCfg == nullptr || pCfg->DevNo < 0 ||
		pCfg->DevNo >= TIMER_NRFX_RTC_MAX)
	{
		return false;
	}

	pRtc = &s_nRfxRtcData[pCfg->DevNo];
	reg = pRtc->pReg;
	pOldTimer = s_pnRfxRtcDev[pCfg->DevNo];

	// Do not replace a Timer object with live PEI bindings.
	if (pOldTimer != nullptr)
	{
#if defined(RTC_OPTIONAL_PEI_STATE)
		if (s_nRfxRtcRaw != nullptr)
#endif
		{
			for (i = 0; i < 2; i++)
			{
				if (s_nRfxRtcRaw[pCfg->DevNo][i].pPei != nullptr)
				{
					return false;
				}
			}
		}
	}
	for (i = 0; i < pRtc->MaxNbTrigEvt; i++)
	{
		if (pRtc->pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE ||
			pRtc->pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
		{
			return false;
		}
	}

	pTimer->DevNo = pCfg->DevNo;
	pTimer->EvtHandler = pCfg->EvtHandler;
	s_pnRfxRtcData = s_nRfxRtcData;
	s_pnRfxRtcDev[pCfg->DevNo] = pTimer;
	s_bRtcHwClear[pCfg->DevNo] = false;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	memset(pRtc->pTrigger, 0, pRtc->MaxNbTrigEvt * sizeof(TimerTrig_t));


    pTimer->Disable = nRFxRtcDisable;
    pTimer->Enable = nRFxRtcEnable;
    pTimer->Reset = nRFxRtcReset;
    pTimer->GetTickCount = nRFxRtcGetTickCount;
    pTimer->SetFrequency = nRFxRtcSetFrequency;
    pTimer->GetMaxTrigger = nRFxRtcGetMaxTrigger;
    pTimer->FindAvailTrigger = nRFxRtcFindAvailTrigger;
    pTimer->DisableTrigger = nRFxRtcDisableTrigger;
    pTimer->EnableTrigger = nRFxRtcEnableTrigger;
	pTimer->GetTaskEndpoint = nRFxRtcTaskEndpoint;
	pTimer->ExecuteTask = nRFxRtcExecuteTask;
    pTimer->DisableExtTrigger = nRFxRtcDisableExtTrigger;
    pTimer->EnableExtTrigger = nRFxRtcEnableExtTrigger;

    reg->TASKS_STOP = 1;
    reg->EVENTS_OVRFLW = 0;

	// No compare of an earlier use, the triggers are cleared above
	uint32_t ccmask = ((1U << pRtc->MaxNbTrigEvt) - 1U);

	reg->INTENCLR = ccmask << RTC_INTENSET_COMPARE0_Pos;
	reg->EVTENCLR = ccmask << RTC_EVTEN_COMPARE0_Pos;

	if (!(NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk))
	{
    	NRF_CLOCK->TASKS_LFCLKSTOP = 1;

    	// Init oscillator
		switch (pCfg->ClkSrc)
		{
			case TIMER_CLKSRC_DEFAULT:
			case TIMER_CLKSRC_LFRC:
				NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos;
				break;
			case TIMER_CLKSRC_LFXTAL:
				NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
				break;
			case TIMER_CLKSRC_EXT:
				return false;
	#if defined(NRF52_SERIES) || (NRF51)
			case TIMER_CLKSRC_HFRC:
			case TIMER_CLKSRC_HFXTAL:
				NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Synth << CLOCK_LFCLKSRC_SRC_Pos;
				break;
	#endif
		}

		NRF_CLOCK->TASKS_LFCLKSTART = 1;

		int timeout = 1000000;
		bool started = false;

		do
		{
			if ((NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) && NRF_CLOCK->EVENTS_LFCLKSTARTED)
			{
				started = true;
				break;
			}
		} while (timeout-- > 0);

		if (!started)
		{
			NRF_CLOCK->TASKS_LFCLKSTOP = 1;
			return false;
		}

		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
		s_bLFClockOwned = true;
	}

	// Init interrupt for RTC
	switch (pCfg->DevNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(RTC0_IRQn);
			NVIC_SetPriority(RTC0_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(RTC0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(RTC1_IRQn);
			NVIC_SetPriority(RTC1_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(RTC1_IRQn);
			break;
#if TIMER_NRFX_RTC_MAX > 2
		case 2:
			NVIC_ClearPendingIRQ(RTC2_IRQn);
			NVIC_SetPriority(RTC2_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(RTC2_IRQn);
			break;
#endif
	}

	// Enable tick & overflow interrupt
	reg->INTENSET = RTC_INTENSET_OVRFLW_Msk;
    reg->EVTENSET = RTC_EVTEN_OVRFLW_Msk;

    if (pCfg->EvtHandler && pCfg->bTickInt)
	{
    	reg->INTENSET = RTC_INTENSET_TICK_Msk;
        reg->EVTENSET = RTC_EVTEN_TICK_Msk;
	}

    nRFxRtcSetFrequency(pTimer, pCfg->Freq);

	// Tick count from 0, then started with the LFCLK request of Enable
	nRFxRtcClear(reg);

    return nRFxRtcEnable(pTimer);
}

