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

#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
#define NRF_CLOCK		NRF_CLOCK_NS
#define NRF_RTC0		NRF_RTC0_NS
#define NRF_RTC1		NRF_RTC1_NS
#else
#define NRF_CLOCK		NRF_CLOCK_S
#define NRF_RTC0		NRF_RTC0_S
#define NRF_RTC1		NRF_RTC1_S
#define NRF_RTC2		NRF_RTC1_S
#endif
#define CLOCK_LFCLKSRC_SRC_RC		CLOCK_LFCLKSRC_SRC_LFRC
#define CLOCK_LFCLKSRC_SRC_Xtal		CLOCK_LFCLKSRC_SRC_LFXO
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

	NRF_RTC_Type *reg = s_pnRfxRtcData[DevNo].pReg;
	TimerDev_t *timer = s_pnRfxRtcDev[DevNo];
    uint32_t evt = 0;
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
            evt |= TIMER_EVT_TRIGGER(i);
            reg->EVENTS_COMPARE[i] = 0;
            if (s_pnRfxRtcData[DevNo].pTrigger[i].Type == TIMER_TRIG_TYPE_CONTINUOUS)
            {
            	reg->CC[i] = (count + s_pnRfxRtcData[DevNo].pCC[i]) & 0xffffff;
            }
            if (s_pnRfxRtcData[DevNo].pTrigger[i].Handler)
            {
            	s_pnRfxRtcData[DevNo].pTrigger[i].Handler(timer, i, s_pnRfxRtcData[DevNo].pTrigger[i].pContext);
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
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

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

    if (s_nRfxLFClockSem <= 0)
    {
    	s_nRfxLFClockSem = 0;
    	NRF_CLOCK->TASKS_LFCLKSTOP = 1;
    }
}

static void nRFxRtcReset(TimerDev_t * const pTimer)
{
	s_nRfxRtcData[pTimer->DevNo].pReg->TASKS_CLEAR = 1;
}

static uint32_t nRFxRtcSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
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

    // Pre-calculate periods for faster timer counter to time conversion use later
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

static uint64_t nRFxRtcEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                              	  	 TimerTrigEvtHandler_t Handler, void *pContext)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];

    if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt)
        return 0;

    uint32_t cc = (nsPeriod + (pTimer->nsPeriod >> 1)) / pTimer->nsPeriod;

    if (cc <= 0)
    {
        return 0;
    }

    rtc.pTrigger[TrigNo].Type = Type;
    rtc.pCC[TrigNo] = cc;
    rtc.pReg->EVTENSET = RTC_EVTEN_COMPARE0_Msk << TrigNo;

    rtc.pReg->INTENSET = RTC_INTENSET_COMPARE0_Msk << TrigNo;

    rtc.pReg->CC[TrigNo] =rtc.pCC[TrigNo] + rtc.pReg->COUNTER;

    rtc.pTrigger[TrigNo].nsPeriod = pTimer->nsPeriod * (uint64_t)cc;
    rtc.pTrigger[TrigNo].Handler = Handler;
    rtc.pTrigger[TrigNo].pContext = pContext;

    return pTimer->nsPeriod * (uint64_t)cc; // Return real period in nsec
}

static void nRFxRtcDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];

	if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt)
        return;

    rtc.pCC[TrigNo] = 0;
    rtc.pReg->CC[TrigNo] = 0;
    rtc.pReg->EVTENCLR = RTC_EVTEN_COMPARE0_Msk << TrigNo;
    rtc.pReg->INTENCLR = RTC_INTENCLR_COMPARE0_Msk << TrigNo;

    rtc.pTrigger[TrigNo].Type = TIMER_TRIG_TYPE_SINGLE;
    rtc.pTrigger[TrigNo].Handler = NULL;
    rtc.pTrigger[TrigNo].pContext = NULL;
    rtc.pTrigger[TrigNo].nsPeriod = 0;
}

static int nRFxRtcFindAvailTrigger(TimerDev_t * const pTimer)
{
	for (int i = 0; i < s_nRfxRtcData[pTimer->DevNo].MaxNbTrigEvt; i++)
	{
		if (s_nRfxRtcData[pTimer->DevNo].pTrigger[i].nsPeriod == 0)
			return i;
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
    if (pCfg->DevNo < 0 || pCfg->DevNo >= TIMER_NRFX_RTC_MAX || pCfg->Freq > TIMER_NRFX_RTC_BASE_FREQ)
    {
    	return false;
    }

    pTimer->DevNo = pCfg->DevNo;
    pTimer->EvtHandler = pCfg->EvtHandler;
	s_pnRfxRtcData = s_nRfxRtcData;

	const nRFRtcData_t &rtc = s_nRfxRtcData[pTimer->DevNo];
	NRF_RTC_Type *reg = s_nRfxRtcData[pTimer->DevNo].pReg;

    s_pnRfxRtcDev[pTimer->DevNo] = pTimer;

    memset(rtc.pTrigger, 0, rtc.MaxNbTrigEvt * sizeof(TimerTrig_t));


    pTimer->Disable = nRFxRtcDisable;
    pTimer->Enable = nRFxRtcEnable;
    pTimer->Reset = nRFxRtcReset;
    pTimer->GetTickCount = nRFxRtcGetTickCount;
    pTimer->SetFrequency = nRFxRtcSetFrequency;
    pTimer->GetMaxTrigger = nRFxRtcGetMaxTrigger;
    pTimer->FindAvailTrigger = nRFxRtcFindAvailTrigger;
    pTimer->DisableTrigger = nRFxRtcDisableTrigger;
    pTimer->EnableTrigger = nRFxRtcEnableTrigger;
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

		int timout = 1000000;

		do
		{
			if ((NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) && NRF_CLOCK->EVENTS_LFCLKSTARTED)
				break;

		} while (timout-- > 0);

		if (timout <= 0)
			return false;

		NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    }
    else
    {
		// Clock source already enabled externally.
		// This is to ensure we don't disable it when all timers are disabled
    	s_nRfxLFClockSem++;
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

