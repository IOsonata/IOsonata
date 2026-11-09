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

typedef struct {
	PeiDev_t *pPei;
	PeiEvent_t Event;
	bool EventWasEnabled;
} nRFRtcRawPei_t;

static nRFRtcRawPei_t s_nRfxRtcRaw[TIMER_NRFX_RTC_MAX][2];

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

// Set by nRFxRtcInit. The interrupt handlers below are kept by the linker as soon as
// this file is part of a build, used or not. They reach the timer data
// through this pointer, so that the data is linked only when a timer is
// initialized.
static const nRFRtcData_t *s_pnRfxRtcData = nullptr;

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

	NRF_RTC_Type *reg = s_pnRfxRtcData[DevNo].pReg;
    uint32_t evt = 0;
	uint32_t ticks;
    uint32_t count = reg->COUNTER;

    if (reg->EVENTS_TICK)
	{
        evt |= TIMER_EVT_TICK;
        reg->EVENTS_TICK = 0;
	}

    if (reg->EVENTS_OVRFLW)
	{
    	timer->Rollover += 0x1000000ULL;
        evt |= TIMER_EVT_COUNTER_OVR;
        reg->EVENTS_OVRFLW = 0;
	}

    timer->LastCount = count;

	for (int i = 0; i < s_pnRfxRtcData[DevNo].MaxNbTrigEvt; i++)
	{
		if (reg->EVENTS_COMPARE[i])
		{
			reg->EVENTS_COMPARE[i] = 0U;
			if (s_pnRfxRtcData[DevNo].pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
			{
				// Only this CC is rescheduled; PEI routes the event in hardware.
				ticks = (uint32_t)TimerNanosecondToTick(timer,
					s_pnRfxRtcData[DevNo].pTrigger[i].nsPeriod);
				reg->CC[i] = (count + ticks) & 0xFFFFFFU;
			}
			else if (s_pnRfxRtcData[DevNo].pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE)
			{
				reg->INTENCLR = RTC_INTENCLR_COMPARE0_Msk << i;
				reg->EVTENCLR = RTC_EVTEN_COMPARE0_Msk << i;
			}
			else
			{
				evt |= TIMER_EVT_TRIGGER(i);
				if (s_pnRfxRtcData[DevNo].pTrigger[i].Type == TIMER_TRIG_TYPE_CONTINUOUS)
				{
					reg->CC[i] = (count + s_pnRfxRtcData[DevNo].pCC[i]) & 0xFFFFFFU;
				}
				if (s_pnRfxRtcData[DevNo].pTrigger[i].Handler != nullptr)
				{
					s_pnRfxRtcData[DevNo].pTrigger[i].Handler(timer, i,
						s_pnRfxRtcData[DevNo].pTrigger[i].pContext);
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

static void nRFxRtcReset(TimerDev_t * const pTimer)
{
	s_nRfxRtcData[pTimer->DevNo].pReg->TASKS_CLEAR = 1;
}

static uint32_t nRFxRtcSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];
	for (int i = 0; i < rtc.MaxNbTrigEvt; i++)
	{
		if (rtc.pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE ||
			rtc.pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
		{
			return pTimer->Freq;
		}
	}
	s_nRfxRtcData[pTimer->DevNo].pReg->TASKS_STOP = 1;

    uint32_t prescaler = 1;

    if (Freq > 0)
	{
    	prescaler = TIMER_NRFX_RTC_BASE_FREQ / Freq;
    	if (prescaler > 0x1000)
    	{
    		// Cap at 12 bits
    		prescaler = 0x1000;
    	}
	}

    s_nRfxRtcData[pTimer->DevNo].pReg->PRESCALER = prescaler - 1;

    pTimer->Freq = TIMER_NRFX_RTC_BASE_FREQ / prescaler;

	// Tick period rounded down to a nanosecond, 30517 for the 30517.58 ns of
	// 32768 Hz. Not precise enough to convert a count or a trigger period,
	// those are computed from Freq, see TimerTickToTime.
    pTimer->nsPeriod = 1000000000ULL / (uint64_t)pTimer->Freq;     // Period in nsec

    s_nRfxRtcData[pTimer->DevNo].pReg->TASKS_START = 1;

    return pTimer->Freq;
}

static uint64_t nRFxRtcGetTickCount(TimerDev_t * const pTimer)
{
	return (uint64_t)s_nRfxRtcData[pTimer->DevNo].pReg->COUNTER + pTimer->Rollover;
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
static bool nRFxRtcEnablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
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
static bool nRFxRtcDisablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
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
	// Internal read-only check: reinitialization must not abandon live
	// TICK/OVRFLW routes or PEI-owned compare slots.
	if (pPei == nullptr && Event == 0U)
	{
		for (i = 0; i < 2; i++)
		{
			if (s_nRfxRtcRaw[devno][i].pPei != nullptr)
			{
				return true;
			}
		}
		for (i = 0; i < pData->MaxNbTrigEvt; i++)
		{
			if (pData->pTrigger[i].Type == TIMER_TRIG_PEI_SINGLE ||
				pData->pTrigger[i].Type == TIMER_TRIG_PEI_CONTINUOUS)
			{
				return true;
			}
		}
		return false;
	}
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
static bool nRFxRtcEnablePeiTrigger(TimerDev_t *pTimer, int TrigNo,
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

	count = reg->COUNTER;
	reg->EVENTS_COMPARE[TrigNo] = 0U;
	reg->CC[TrigNo] = (count + (uint32_t)ticks) & 0xFFFFFFU;
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

// Called only when an application explicitly uses PEI with this RTC.
// Ordinary RTC initialization has no dependency on the PEI matrix.
bool nRFxRtcInstallPei(TimerDev_t *pTimer)
{
	int devno;

	if (pTimer == nullptr || PeiBindEvent == nullptr ||
		PeiUnbindEvent == nullptr)
	{
		return false;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX ||
		s_pnRfxRtcDev[devno] != pTimer)
	{
		return false;
	}
	pTimer->EnablePeiTrigger = nRFxRtcEnablePeiTrigger;
	pTimer->EnablePeiEvent = nRFxRtcEnablePeiEvent;
	pTimer->DisablePeiEvent = nRFxRtcDisablePeiEvent;
	return true;
}

static uint64_t nRFxRtcEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                              	  	 TimerTrigEvtHandler_t Handler, void *pContext)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];

	if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt)
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

	uint32_t cc = TimerNanosecondToTick(pTimer, nsPeriod);

    if (cc <= 0)
	{
        return 0;
	}

	rtc.pTrigger[TrigNo].Type = Type;
	rtc.pCC[TrigNo] = cc;
    rtc.pReg->EVTENSET = RTC_EVTEN_COMPARE0_Msk << TrigNo;

    rtc.pReg->INTENSET = RTC_INTENSET_COMPARE0_Msk << TrigNo;

	rtc.pReg->CC[TrigNo] =rtc.pCC[TrigNo] + rtc.pReg->COUNTER;

	rtc.pTrigger[TrigNo].nsPeriod = TimerTickToNanosecond(pTimer, cc);
	rtc.pTrigger[TrigNo].Handler = Handler;
	rtc.pTrigger[TrigNo].pContext = pContext;

	return rtc.pTrigger[TrigNo].nsPeriod; // Return real period in nsec
}

static void nRFxRtcDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];

	if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt)
	{
		return;
	}
	if (rtc.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_SINGLE ||
		rtc.pTrigger[TrigNo].Type == TIMER_TRIG_PEI_CONTINUOUS)
	{
		// Release only this CC's PEI event source. Keep ownership
		// until the matrix successfully unbinds it.
		if (pTimer->DisablePeiEvent == nullptr)
		{
			return;
		}
		pTimer->DisablePeiEvent(pTimer,
			static_cast<PeiDev_t *>(rtc.pTrigger[TrigNo].pContext),
			rtc.pCC[TrigNo]);
		return;
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
static inline void nRFxRtcDisableExtTrigger(TimerDev_t * const pTimer) {
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
bool nRFxRtcEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
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

	if (pTimer == nullptr || pCfg == nullptr || pCfg->DevNo < 0 ||
		pCfg->DevNo >= TIMER_NRFX_RTC_MAX ||
		pCfg->Freq > TIMER_NRFX_RTC_BASE_FREQ)
	{
		return false;
	}

	pRtc = &s_nRfxRtcData[pCfg->DevNo];
	reg = pRtc->pReg;
	pOldTimer = s_pnRfxRtcDev[pCfg->DevNo];

	// Do not replace a Timer object with live PEI bindings.
	// The optional RTC event callback tracks TICK/OVRFLW owners.
	if (pOldTimer != nullptr &&
		pOldTimer->DisablePeiEvent != nullptr &&
		pOldTimer->DisablePeiEvent(pOldTimer, nullptr, 0U))
	{
		return false;
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
	pTimer->EnablePeiTrigger = nullptr;
	pTimer->EnablePeiEvent = nullptr;
	pTimer->DisablePeiEvent = nullptr;
	pTimer->GetTaskEndpoint = nRFxRtcTaskEndpoint;
	pTimer->ExecuteTask = nRFxRtcExecuteTask;
    pTimer->DisableExtTrigger = nRFxRtcDisableExtTrigger;
    pTimer->EnableExtTrigger = nRFxRtcEnableExtTrigger;

    reg->TASKS_STOP = 1;
    reg->TASKS_CLEAR = 1;

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

    return true;
}

