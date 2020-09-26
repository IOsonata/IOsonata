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

#include "timer_nrfx.h"

#if defined(NRF91_SERIES) || defined(NRF5340_XXAA_APPLICATION)
#define NRF_TIMER0			NRF_TIMER0_S
#define NRF_TIMER1			NRF_TIMER1_S
#define NRF_TIMER2			NRF_TIMER2_S
#define NRF_CLOCK			NRF_CLOCK_S
#elif defined(NRF5340_XXAA_NETWORK)
#define NRF_TIMER0			NRF_TIMER0_NS
#define NRF_TIMER1			NRF_TIMER1_NS
#define NRF_TIMER2			NRF_TIMER2_NS
#define NRF_CLOCK			NRF_CLOCK_NS
#endif

#define	INTERRUPT_LATENCY		11

typedef struct {
	int DevNo;
	uint32_t MaxFreq;
    NRF_TIMER_Type *pReg;
    int MaxNbTrigEvt;		//!< Number of trigger is not the same for all timers.
    uint32_t CC[TIMER_NRFX_HF_MAX_TRIGGER_EVT];
    TIMER_TRIGGER Trigger[TIMER_NRFX_HF_MAX_TRIGGER_EVT];
    TIMER *pTimer;
} NRFX_TIMER_DATA;

static NRFX_TIMER_DATA s_nRFxTimerData[TIMER_NRFX_HF_MAX] = {
	{
		.DevNo = TIMER_NRFX_RTC_MAX,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER0,
		.MaxNbTrigEvt = TIMER0_CC_NUM,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 1,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER1,
		.MaxNbTrigEvt = TIMER1_CC_NUM,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 2,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER2,
		.MaxNbTrigEvt = TIMER2_CC_NUM,
	},
#if TIMER_NRFX_HF_MAX > 3
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 3,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER3,
		.MaxNbTrigEvt = TIMER3_CC_NUM,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 4,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER4,
		.MaxNbTrigEvt = TIMER4_CC_NUM,
	},
#endif
};

static std::atomic<int> s_nRfxHFClockSem(0);

static void TimerIRQHandler(int DevNo)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[DevNo].pReg;
	NRFX_TIMER_DATA &tdata = s_nRFxTimerData[DevNo];
	TIMER *timer = s_nRFxTimerData[DevNo].pTimer;
    uint32_t evt = 0;
    uint32_t count;

    reg->TASKS_CAPTURE[0] = 1;
    count = reg->CC[0];

    for (int i = 0; i < tdata.MaxNbTrigEvt; i++)
    {
        if (reg->EVENTS_COMPARE[i])
        {
            evt |= 1 << (i + 2);
            reg->EVENTS_COMPARE[i] = 0;
            if (tdata.Trigger[i].Type == TIMER_TRIG_TYPE_CONTINUOUS)
            {
            	reg->CC[i] = count + tdata.CC[i] - INTERRUPT_LATENCY;
            }
            if (tdata.Trigger[i].Handler)
            {
            	tdata.Trigger[i].Handler(timer, i, tdata.Trigger[i].pContext);
            }
        }

    }

    if (count < timer->LastCount)
    {
        // Counter wrap arround
        evt |= TIMER_EVT_COUNTER_OVR;
        timer->Rollover += timer->Freq;
    }

    timer->LastCount = count;

    if (timer->EvtHandler)
    {
    	timer->EvtHandler(timer, evt);
    }
}

extern "C" {

void TIMER0_IRQHandler()
{
	TimerIRQHandler(0);
}

void TIMER1_IRQHandler()
{
	TimerIRQHandler(1);
}

void TIMER2_IRQHandler()
{
	TimerIRQHandler(2);
}

#if TIMER_NRFX_HF_MAX > 3
void TIMER3_IRQHandler()
{
	TimerIRQHandler(3);
}

void TIMER4_IRQHandler()
{
	TimerIRQHandler(4);
}
#endif

}   // extern "C"

static bool nRFxTimerEnable(TIMER * const pTimer)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX].pReg;

	// Clock source not available.  Only 64MHz XTAL
	if (s_nRfxHFClockSem <= 0)
	{
		NRF_CLOCK->TASKS_HFCLKSTART = 1;

		int timout = 1000000;

		do
		{
			if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) || NRF_CLOCK->EVENTS_HFCLKSTARTED)
				break;

		} while (timout-- > 0);

		if (timout <= 0)
			return false;

		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

		s_nRfxHFClockSem++;
	}

    reg->TASKS_START = 1;

	switch (pTimer->DevNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(TIMER0_IRQn);
			NVIC_EnableIRQ(TIMER0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(TIMER1_IRQn);
			NVIC_EnableIRQ(TIMER1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(TIMER2_IRQn);
			NVIC_EnableIRQ(TIMER2_IRQn);
			break;
#if TIMER_NRFX_HF_MAX > 3
		case 3:
			NVIC_ClearPendingIRQ(TIMER3_IRQn);
			NVIC_EnableIRQ(TIMER3_IRQn);
			break;
		case 4:
			NVIC_ClearPendingIRQ(TIMER4_IRQn);
			NVIC_EnableIRQ(TIMER4_IRQn);
			break;
#endif
    }

    reg->TASKS_START = 1;

    return true;
}

static void nRFxTimerDisable(TIMER * const pTimer)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX].pReg;

	s_nRfxHFClockSem--;

	reg->TASKS_STOP = 1;

	if (s_nRfxHFClockSem <= 0)
	{
		NRF_CLOCK->TASKS_HFCLKSTOP = 1;
		s_nRfxHFClockSem = 0;
	}

    switch (pTimer->DevNo)
    {
        case 0:
            NVIC_ClearPendingIRQ(TIMER0_IRQn);
            NVIC_DisableIRQ(TIMER0_IRQn);
            break;
        case 1:
            NVIC_ClearPendingIRQ(TIMER1_IRQn);
            NVIC_DisableIRQ(TIMER1_IRQn);
            break;
        case 2:
            NVIC_ClearPendingIRQ(TIMER2_IRQn);
            NVIC_DisableIRQ(TIMER2_IRQn);
            break;
#if TIMER_NRFX_HF_MAX > 3
        case 3:
            NVIC_ClearPendingIRQ(TIMER3_IRQn);
            NVIC_DisableIRQ(TIMER3_IRQn);
            break;
        case 4:
            NVIC_ClearPendingIRQ(TIMER4_IRQn);
            NVIC_DisableIRQ(TIMER4_IRQn);
            break;
#endif
    }

    reg->INTENSET = 0;
}

static void nRFxTimerReset(TIMER * const pTimer)
{
	s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX].pReg->TASKS_CLEAR = 1;
}

static uint32_t nRFxTimerSetFrequency(TIMER * const pTimer, uint32_t Freq)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX].pReg;

	reg->TASKS_STOP = 1;

    uint32_t prescaler = 0;

    if (Freq > 0)
    {
        uint32_t divisor = TIMER_NRFX_HF_BASE_FREQ / Freq;
#ifdef __ICCARM__
        prescaler = 31 - __CLZ(divisor);
#else
        prescaler = 31 - __builtin_clzl(divisor);
#endif
        if (prescaler > 9)
        {
            prescaler = 9;
        }
    }

    reg->PRESCALER = prescaler;

    pTimer->Freq = TIMER_NRFX_HF_BASE_FREQ / (1 << prescaler);

    // Pre-calculate periods for faster timer counter to time conversion use later
    // for precision this value is x10 (in 100 psec)
    pTimer->nsPeriod = 10000000000ULL / pTimer->Freq;     // Period in x10 nsec

    reg->TASKS_START = 1;

    return pTimer->Freq;
}

static uint64_t nRFxTimerGetTickCount(TIMER * const pTimer)
{
	NRF_TIMER_Type *reg = s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX].pReg;

	if (reg->INTENSET == 0)
    {
		reg->TASKS_CAPTURE[pTimer->DevNo] = 1;

		uint32_t count = reg->CC[pTimer->DevNo];

		if (count < pTimer->LastCount)
	    {
	        // Counter wrap arround
	        pTimer->Rollover += 0x100000000ULL;//vFreq;
	    }

		pTimer->LastCount = count;
    }

	return (uint64_t)pTimer->LastCount + pTimer->Rollover;
}

static uint64_t nRFxTimerEnableTrigger(TIMER * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                                       TIMER_TRIGCB const Handler, void * const pContext)
{
	NRFX_TIMER_DATA &tdata = s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX];
	NRF_TIMER_Type *reg = tdata.pReg;

	if (TrigNo < 0 || TrigNo >= tdata.MaxNbTrigEvt)
	{
		return 0;
	}

    // vnsPerios is x10 nsec (100 psec) => nsPeriod * 10ULL
    uint32_t cc = (nsPeriod * 10ULL + (pTimer->nsPeriod >> 1)) / pTimer->nsPeriod;

    if (cc <= 0)
    {
        return 0;
    }

    tdata.Trigger[TrigNo].Type = Type;
    tdata.CC[TrigNo] = cc;
    reg->TASKS_CAPTURE[TrigNo] = 1;

    uint32_t count = reg->CC[TrigNo];

	reg->INTENSET = TIMER_INTENSET_COMPARE0_Msk << TrigNo;

    reg->CC[TrigNo] = count + cc - INTERRUPT_LATENCY;

    if (count < pTimer->LastCount)
    {
        // Counter wrap around
        pTimer->Rollover += 0x100000000ULL;//vFreq;
    }

    pTimer->LastCount = count;

    tdata.Trigger[TrigNo].nsPeriod = pTimer->nsPeriod * (uint64_t)cc / 10ULL;
    tdata.Trigger[TrigNo].Handler = Handler;
    tdata.Trigger[TrigNo].pContext = pContext;

    return pTimer->nsPeriod * (uint64_t)cc / 10ULL; // Return real period in nsec
}

static void nRFxTimerDisableTrigger(TIMER * const pTimer, int TrigNo)
{
	NRFX_TIMER_DATA &tdata = s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX];
	NRF_TIMER_Type *reg = tdata.pReg;

	if (TrigNo < 0 || TrigNo >= tdata.MaxNbTrigEvt)
        return;

    tdata.CC[TrigNo] = 0;
    reg->CC[TrigNo] = 0;
    reg->INTENCLR = TIMER_INTENSET_COMPARE0_Msk << TrigNo;

    tdata.Trigger[TrigNo].Type = TIMER_TRIG_TYPE_SINGLE;
    tdata.Trigger[TrigNo].Handler = NULL;
    tdata.Trigger[TrigNo].pContext = NULL;
    tdata.Trigger[TrigNo].nsPeriod = 0;
}

static int nRFxTimerFindAvailTrigger(TIMER * const pTimer)
{
	NRFX_TIMER_DATA &tdata = s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX];

	for (int i = 0; i < tdata.MaxNbTrigEvt; i++)
	{
		if (tdata.Trigger[i].nsPeriod == 0)
			return i;
	}

	return -1;
}

static int nRFxTimerGetMaxTrigger(TIMER * const pTimer)
{
	return s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX].MaxNbTrigEvt;
}

bool nRFxTimerInit(TIMER * const pTimer, const TIMER_CFG * const pCfg)
{
    if (pCfg->DevNo < 0 || pCfg->DevNo >= TIMER_NRFX_HF_MAX || pCfg->Freq > TIMER_NRFX_HF_BASE_FREQ)
    {
        return false;
    }

    pTimer->DevNo = pCfg->DevNo;
    pTimer->EvtHandler = pCfg->EvtHandler;
	NRFX_TIMER_DATA &tdata = s_nRFxTimerData[pTimer->DevNo];
	NRF_TIMER_Type *reg = s_nRFxTimerData[pTimer->DevNo].pReg;

	tdata.pTimer = pTimer;

    memset(tdata.Trigger, 0, sizeof(tdata.Trigger));

    pTimer->Disable = nRFxTimerDisable;
    pTimer->Enable = nRFxTimerEnable;
    pTimer->Reset = nRFxTimerReset;
    pTimer->GetTickCount = nRFxTimerGetTickCount;
    pTimer->SetFrequency = nRFxTimerSetFrequency;
    pTimer->GetMaxTrigger = nRFxTimerGetMaxTrigger;
    pTimer->FindAvailTrigger = nRFxTimerFindAvailTrigger;
    pTimer->DisableTrigger = nRFxTimerDisableTrigger;
    pTimer->EnableTrigger = nRFxTimerEnableTrigger;

    reg->TASKS_STOP = 1;
    reg->TASKS_CLEAR = 1;

    NRF_CLOCK->TASKS_HFCLKSTOP = 1;

    // Only support timer mode, 32bits counter
    reg->MODE = TIMER_MODE_MODE_Timer;
    reg->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

	switch (pCfg->DevNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(TIMER0_IRQn);
			NVIC_SetPriority(TIMER0_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(TIMER0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(TIMER1_IRQn);
			NVIC_SetPriority(TIMER1_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(TIMER1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(TIMER2_IRQn);
			NVIC_SetPriority(TIMER2_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(TIMER2_IRQn);
			break;
#if TIMER_NRFX_HF_MAX > 3
		case 3:
			NVIC_ClearPendingIRQ(TIMER3_IRQn);
			NVIC_SetPriority(TIMER3_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(TIMER3_IRQn);
			break;
		case 4:
			NVIC_ClearPendingIRQ(TIMER4_IRQn);
			NVIC_SetPriority(TIMER4_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(TIMER4_IRQn);
			break;
#endif
	}

    // Clock source not available.  Only 64MHz XTAL

    s_nRfxHFClockSem++;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    int timout = 1000000;

    do
    {
        if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) || NRF_CLOCK->EVENTS_HFCLKSTARTED)
            break;

    } while (timout-- > 0);

    if (timout <= 0)
        return false;

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    nRFxTimerSetFrequency(pTimer, pCfg->Freq);

    return true;
}

