/**-------------------------------------------------------------------------
@file	timer_lf_nrf54.cpp

@brief	timer class implementation on Nordic nRF54 series using the GRTC (real time counter)

@author	Hoang Nguyen Hoan
@date	Jan. 25, 2025

@license

MIT License

Copyright (c) 2025, I-SYST inc., all rights reserved

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

#include "nrf.h"

#include "timer_nrfx.h"
#include "coredev/interrupt.h"

// GRTC IRQ_3 and CC 7-11 are reserved by MPSL
#define MPSL_RESERVED_NB_CC		5	// 7-11 are reserved for MPSL

#define NRF_CLOCK			NRF_CLOCK_S
#define NRF_RTC0			NRF_RTC10_S
#define NRF_RTC1			NRF_RTC30_S
#define GRTC0_CC_NUM		GRTC_CC_NUM_SIZE
#define GRTC1_CC_NUM		RTC30_CC_NUM_SIZE

#pragma pack(push, 4)
typedef struct {
	int DevNo;
    int MaxNbTrigEvt;		//!< Number of trigger is not the same for all timers.
    int StartCCIdx;			//!< CC channel start
    uint64_t CC[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];	//!< Trigger period in ticks
    uint64_t Compare[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];	//!< Compare value written to the CC, GRTC reads are slow
    TimerTrig_t Trigger[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];
    TimerDev_t *pTimer;
    volatile uint32_t *pInt;	//!< INTEN, INTENSET, INTENCLR, INTPEND of interrupt group DevNo
    volatile NRF_GRTC_SYSCOUNTER_Type *pSysCnt;	//!< SYSCOUNTER[DevNo]
} nRFGrtcData_t;
#pragma pack(pop)

//static uint32_t s_GrtcMaxFreq = TIMER_NRFX_RTC_BASE_FREQ;

// There are a total of GRTC_NCC_SIZE (12) CC sharing 4 IRQ.  We are splitting into 4 devices
alignas(4) static nRFGrtcData_t s_nRfxGrtcData[TIMER_NRFX_RTC_MAX] = {
	{
		.DevNo = 0,
		.MaxNbTrigEvt = 2,	// CC 0..1
		.StartCCIdx = 0,
	},
	{
		.DevNo = 1,
		.MaxNbTrigEvt = 2,	// CC 2..3
		.StartCCIdx = 2,
	},
	{
		.DevNo = 2,
		.MaxNbTrigEvt = 3,	// CC 4..6
		.StartCCIdx = 4,
	},
	{
		// This one is reserved by MPSL
		.DevNo = 3,
		.MaxNbTrigEvt = 5,	// CC 7..11
		.StartCCIdx = 7,
	},
};

// Timers enabled. The first Enable starts the SYSCOUNTER when it is not
// running, Disable never stops it: the radio stack (MPSL, SoftDevice) uses it
// too and can start it after this driver did.
static bool s_bGrtcEnabled[TIMER_NRFX_RTC_MAX];

// Interrupt of a timer: GRTC_0_IRQn to GRTC_3_IRQn are consecutive, one per
// INTEN register group, the DevNo
static inline IRQn_Type nRFxGrtcIrqNo(int DevNo)
{
	return (IRQn_Type)(GRTC_0_IRQn + DevNo);
}

// Compare channels of a timer, as INTEN bits
static inline uint32_t nRFxGrtcCCMask(const nRFGrtcData_t &Rtc)
{
	return ((1U << Rtc.MaxNbTrigEvt) - 1U) << Rtc.StartCCIdx;
}

static bool nRFxGrtcEnable(TimerDev_t * const pTimer)
{
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];

	if ((NRF_GRTC->MODE & GRTC_MODE_SYSCOUNTEREN_Msk) == 0)
	{
		NRF_GRTC->MODE |= GRTC_MODE_SYSCOUNTEREN_Msk | GRTC_MODE_AUTOEN_CpuActive;
		NRF_GRTC->TASKS_START = 1;
	}

	s_bGrtcEnabled[pTimer->DevNo] = true;

	// Interrupt on while a trigger is armed, see nRFxGrtcEnableTrigger. Off
	// otherwise: the SoftDevice owns GRTC_3_IRQn and wants it off when it
	// starts, after the Init of timer 3 (bt_app_bm.cpp).
	if (rtc.pInt[0] & nRFxGrtcCCMask(rtc))
	{
		NVIC_EnableIRQ(nRFxGrtcIrqNo(pTimer->DevNo));
	}

    return true;
}

// The triggers are kept, their interrupt off until Enable. The SYSCOUNTER
// keeps running, the tick count too.
static void nRFxGrtcDisable(TimerDev_t * const pTimer)
{
	s_bGrtcEnabled[pTimer->DevNo] = false;

	NVIC_DisableIRQ(nRFxGrtcIrqNo(pTimer->DevNo));
	NVIC_ClearPendingIRQ(nRFxGrtcIrqNo(pTimer->DevNo));
}

// SYSCOUNTER, SYSCOUNTERL read before SYSCOUNTERH. Read again while BUSY
// (value not valid) or OVERFLOW (SYSCOUNTERL overflowed after it was read, the
// two halves are not from the same count), as nrfy_grtc_sys_counter_get does.
// OVERFLOW is set once in 71 minutes, the product specification takes
// SYSCOUNTERH - 1 for it.
static inline __attribute__((always_inline)) uint64_t nRFxGrtcSysCounter(volatile NRF_GRTC_SYSCOUNTER_Type *pSysCnt)
{
	uint32_t cl;
	uint32_t ch;

	do {
		cl = pSysCnt->SYSCOUNTERL;
		ch = pSysCnt->SYSCOUNTERH;
	} while (ch & (GRTC_SYSCOUNTER_SYSCOUNTERH_BUSY_Msk | GRTC_SYSCOUNTER_SYSCOUNTERH_OVERFLOW_Msk));

	return ((uint64_t)(ch & GRTC_SYSCOUNTER_SYSCOUNTERH_VALUE_Msk) << 32) | cl;
}

// The SYSCOUNTER is shared by the GRTC timers and the radio stack: Reset does
// not clear it. The tick count restarts from Reset, kept as an offset in
// Rollover, and the armed triggers restart their period from it.
static void nRFxGrtcReset(TimerDev_t * const pTimer)
{
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];
	uint32_t state = DisableInterrupt();
	uint32_t inten = rtc.pInt[0];
	uint64_t now = nRFxGrtcSysCounter(rtc.pSysCnt);

	pTimer->Rollover = 0ULL - now;

	for (int i = 0; i < rtc.MaxNbTrigEvt; i++)
	{
		int idx = rtc.StartCCIdx + i;

		if (rtc.Trigger[i].nsPeriod == 0 || (inten & (1U << idx)) == 0U)
		{
			continue;
		}
		rtc.Compare[i] = now + rtc.CC[i];
		NRF_GRTC->CC[idx].CCL = (uint32_t)rtc.Compare[i];
		NRF_GRTC->EVENTS_COMPARE[idx] = 0;
		NRF_GRTC->CC[idx].CCH = (uint32_t)(rtc.Compare[i] >> 32);
	}

	EnableInterrupt(state);
}

// GRTC is fixed at 1MHz.
static uint32_t nRFxGrtcSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	pTimer->Freq = TIMER_NRFX_RTC_BASE_FREQ;

    // Pre-calculate periods for faster timer counter to time conversion use later
    pTimer->nsPeriod = (1000000000ULL + (pTimer->Freq >> 1))/ (uint64_t)pTimer->Freq;     // Period in nsec

    return pTimer->Freq;
}

// SYSCOUNTER (52 bits at 1 MHz, 142 years), counted from Reset once called
static uint64_t nRFxGrtcGetTickCount(TimerDev_t * const pTimer)
{
	return nRFxGrtcSysCounter(&NRF_GRTC->SYSCOUNTER[pTimer->DevNo]) + pTimer->Rollover;
}

static int nRFxGrtcGetMaxTrigger(TimerDev_t * const pTimer)
{
	return s_nRfxGrtcData[pTimer->DevNo].MaxNbTrigEvt;
}

static uint64_t nRFxGrtcEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                              	  	 TimerTrigEvtHandler_t Handler, void *pContext)
{
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];

    if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt || nsPeriod == 0)
    {
        return 0;
    }

	TimerTrig_t &trig = rtc.Trigger[TrigNo];

	// Each CC owns its compare independently; never replace an active one.
	if (trig.nsPeriod != 0)
	{
		return 0;
	}

	// The period starts at this SYSCOUNTER read, the compare is set from it:
	// a compare value already passed when written matches at once, none is
	// lost, the interrupts can stay on.
	uint64_t start = nRFxGrtcSysCounter(rtc.pSysCnt);

	// Ticks of 1 us, closest, 1 at least. A 32 bit division up to 4.29 s.
	uint64_t cc;

	if (nsPeriod <= 0xFFFFFFFFULL - 500U)
	{
		cc = ((uint32_t)nsPeriod + 500U) / 1000U;
	}
	else
	{
		cc = (nsPeriod + 500U) / 1000U;
	}

    if (cc == 0)
    {
    	cc = 1;
    }

    int idx = rtc.StartCCIdx + TrigNo;

	// CCL disables the channel: an event left from before is cleared before
	// CCH enables it again
	start += cc;
	rtc.Compare[TrigNo] = start;
	NRF_GRTC->CC[idx].CCL = (uint32_t)start;
	NRF_GRTC->EVENTS_COMPARE[idx] = 0;
	NRF_GRTC->CC[idx].CCH = (uint32_t)(start >> 32);

	// Period in nanoseconds, ticks of 1 us
    uint64_t period = pTimer->nsPeriod * cc;

	// The trigger is set before its interrupt
    trig.Type = Type;
    trig.nsPeriod = period;
    trig.Handler = Handler;
    trig.pContext = pContext;
    rtc.CC[TrigNo] = cc;

	// Its interrupt, on in the NVIC only while the timer is enabled
	rtc.pInt[1] = 1U << idx;
	if (s_bGrtcEnabled[rtc.DevNo])
	{
		NVIC_EnableIRQ(nRFxGrtcIrqNo(rtc.DevNo));
	}

    return period; // Return real period in nsec
}

static void nRFxGrtcDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];

	if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt)
        return;

    int idx = rtc.StartCCIdx + TrigNo;

	// Interrupt off first: the interrupt handler only looks at the compares
	// with their interrupt set
	rtc.pInt[2] = 1U << idx;
	if ((rtc.pInt[0] & nRFxGrtcCCMask(rtc)) == 0U)
	{
		NVIC_DisableIRQ(nRFxGrtcIrqNo(rtc.DevNo));
		NVIC_ClearPendingIRQ(nRFxGrtcIrqNo(rtc.DevNo));
	}

    NRF_GRTC->CC[idx].CCEN = 0;
    NRF_GRTC->EVENTS_COMPARE[idx] = 0;

    rtc.CC[TrigNo] = 0;
    rtc.Trigger[TrigNo].Type = TIMER_TRIG_TYPE_SINGLE;
    rtc.Trigger[TrigNo].Handler = NULL;
    rtc.Trigger[TrigNo].pContext = NULL;
    rtc.Trigger[TrigNo].nsPeriod = 0;
}

static int nRFxGrtcFindAvailTrigger(TimerDev_t * const pTimer)
{
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];
	uint32_t mask = 1;
	int32_t i = rtc.MaxNbTrigEvt;

	for (int i = 0; i < rtc.MaxNbTrigEvt; i++)
	{
		if (rtc.Trigger[i].nsPeriod == 0)
			return i;
	}

	return -1;
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static inline void nRFxGrtcDisableExtTrigger(TimerDev_t * const pTimer) {
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
bool nRFxGrtcEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	// No support for external trigger
	return false;
}

bool nRFxGrtcInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	// Any frequency, the GRTC runs at 1 MHz
    if (pCfg->DevNo < 0 || pCfg->DevNo >= TIMER_NRFX_RTC_MAX)
    {
    	return false;
    }

    pTimer->DevNo = pCfg->DevNo;
    pTimer->EvtHandler = pCfg->EvtHandler;
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];
//	NRF_GRTC_Type *reg = s_nRfxGrtcData[pTimer->DevNo].pReg;

	rtc.pInt = &NRF_GRTC->INTEN0 + pCfg->DevNo * 4;
	rtc.pSysCnt = &NRF_GRTC->SYSCOUNTER[pCfg->DevNo];
    memset(rtc.Trigger, 0, sizeof(rtc.Trigger));

    rtc.pTimer = pTimer;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;


    pTimer->Disable = nRFxGrtcDisable;
    pTimer->Enable = nRFxGrtcEnable;
    pTimer->Reset = nRFxGrtcReset;
    pTimer->GetTickCount = nRFxGrtcGetTickCount;
    pTimer->SetFrequency = nRFxGrtcSetFrequency;
    pTimer->GetMaxTrigger = nRFxGrtcGetMaxTrigger;
    pTimer->FindAvailTrigger = nRFxGrtcFindAvailTrigger;
    pTimer->DisableTrigger = nRFxGrtcDisableTrigger;
    pTimer->EnableTrigger = nRFxGrtcEnableTrigger;
    pTimer->DisableExtTrigger = nRFxGrtcDisableExtTrigger;
    pTimer->EnableExtTrigger = nRFxGrtcEnableExtTrigger;
	// No task endpoint on the GRTC timer
	pTimer->GetTaskEndpoint = nullptr;
	pTimer->ExecuteTask = nullptr;

	// Interrupt priority, the interrupt enabled with a trigger
	NVIC_SetPriority(nRFxGrtcIrqNo(pCfg->DevNo), pCfg->IntPrio);

    nRFxGrtcSetFrequency(pTimer, pCfg->Freq);

    return nRFxGrtcEnable(pTimer);
}

// Next compare value of a continuous trigger when Next is not after Now: the
// periods already passed are skipped, the handler is called once for them
static __attribute__((noinline)) uint64_t nRFxGrtcSkip(uint64_t Next, uint64_t Now, uint64_t Period)
{
	return Next + ((Now - Next) / Period + 1U) * Period;
}

// Next compare of continuous trigger TrigNo on CC[Idx], one period after the
// compare that matched: the period does not depend on the interrupt latency.
// The match disabled the channel and its event is cleared, CCL then CCH
// enable it again. A value passed by the time it is written matches at once.
static inline __attribute__((always_inline)) void nRFxGrtcNextCompare(nRFGrtcData_t &Rtc, int TrigNo, int Idx, uint64_t Period)
{
	uint64_t next = Rtc.Compare[TrigNo] + Period;
	uint64_t now = nRFxGrtcSysCounter(Rtc.pSysCnt);

	if (next <= now)
	{
		next = nRFxGrtcSkip(next, now, Period);
	}

	Rtc.Compare[TrigNo] = next;
	NRF_GRTC->CC[Idx].CCL = (uint32_t)next;
	NRF_GRTC->CC[Idx].CCH = (uint32_t)(next >> 32);
}

static void GrtcIRQHandler(nRFGrtcData_t &rtc)
{
	TimerDev_t *timer = rtc.pTimer;

	if (timer == nullptr)
	{
		return;
	}

    uint32_t evt = 0;

	// Compares of the armed triggers with their event set, INTPEND is the
	// events with their interrupt set
	uint32_t pend = (rtc.pInt[3] >> rtc.StartCCIdx) & ((1U << rtc.MaxNbTrigEvt) - 1U);

	while (pend != 0U)
	{
		int i = __builtin_ctz(pend);
		int idx = rtc.StartCCIdx + i;
		TimerTrig_t &trig = rtc.Trigger[i];
		TimerTrigEvtHandler_t handler;
		void *ctx;

		pend &= pend - 1U;

		NRF_GRTC->EVENTS_COMPARE[idx] = 0;

		if (trig.Type == TIMER_TRIG_TYPE_CONTINUOUS)
		{
			// Next compare first, then the handler. No period: disabled by an
			// interrupt of higher priority.
			uint64_t period = rtc.CC[i];

			if (period != 0U)
			{
				nRFxGrtcNextCompare(rtc, i, idx, period);
			}
			handler = trig.Handler;
			ctx = trig.pContext;
		}
		else
		{
			// Single shot done, the match disabled the channel. The trigger is
			// free before its handler, which can set it again: interrupt off
			// and no period, see nRFxGrtcFindAvailTrigger.
			handler = trig.Handler;
			ctx = trig.pContext;
			rtc.pInt[2] = 1U << idx;
			trig.nsPeriod = 0;
		}

		evt |= TIMER_EVT_TRIGGER(i);
		if (handler)
		{
			handler(timer, i, ctx);
		}
	}

    if (timer->EvtHandler)
    {
        timer->EvtHandler(timer, evt);
    }
}

extern "C" {

#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
__WEAK void GRTC_0_IRQHandler(void)
{
	GrtcIRQHandler(s_nRfxGrtcData[0]);
}

__WEAK void GRTC_1_IRQHandler(void)
{
	GrtcIRQHandler(s_nRfxGrtcData[1]);
}

__WEAK void GRTC_2_IRQHandler(void)
{
	GrtcIRQHandler(s_nRfxGrtcData[2]);
}

__WEAK void GRTC_3_IRQHandler(void)
{
	GrtcIRQHandler(s_nRfxGrtcData[3]);
}

#endif

}

