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
#include <atomic>
#include <stdint.h>

#include "nrf.h"

#include "timer_nrfx.h"

// GRTC IRQ_3 and CC 7-11 are reserved by MPSL
#define MPSL_RESERVED_NB_CC		5	// 7-11 are reserved for MPSL

#define NRF_CLOCK		NRF_CLOCK_S
#define NRF_RTC0		NRF_RTC10_S
#define NRF_RTC1		NRF_RTC30_S
#define GRTC0_CC_NUM		GRTC_CC_NUM_SIZE
#define GRTC1_CC_NUM		RTC30_CC_NUM_SIZE

#pragma pack(push, 4)
typedef struct {
	int DevNo;
    int MaxNbTrigEvt;		//!< Number of trigger is not the same for all timers.
    int StartCCIdx;			//!< CC channel start
    uint64_t CC[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];
    TimerTrig_t Trigger[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];
    TimerDev_t *pTimer;
} nRFGrtcData_t;
#pragma pack(pop)

static uint32_t s_GrtcMaxFreq = 0;

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

static std::atomic<int> s_nRfxGrtcSem(0);

static bool nRFxGrtcEnable(TimerDev_t * const pTimer)
{
    switch (pTimer->DevNo)
    {
        case 0:
            NVIC_ClearPendingIRQ(GRTC_0_IRQn);
            NVIC_EnableIRQ(GRTC_0_IRQn);
            break;
        case 1:
            NVIC_ClearPendingIRQ(GRTC_1_IRQn);
            NVIC_EnableIRQ(GRTC_1_IRQn);
            break;
        case 2:
            NVIC_ClearPendingIRQ(GRTC_2_IRQn);
            NVIC_EnableIRQ(GRTC_2_IRQn);
            break;
        case 3:
            NVIC_ClearPendingIRQ(GRTC_3_IRQn);
            NVIC_EnableIRQ(GRTC_3_IRQn);
            break;
    }

    if (s_nRfxGrtcSem == 0)
    {
		NRF_GRTC->MODE |= GRTC_MODE_SYSCOUNTEREN_Enabled;
		NRF_GRTC->TASKS_START = 1;
    }

    s_nRfxGrtcSem++;

    return true;
}

static void nRFxGrtcDisable(TimerDev_t * const pTimer)
{
	s_nRfxGrtcSem--;

	if (s_nRfxGrtcSem > 0)
	{
		return;
	}
	NRF_GRTC->TASKS_STOP = 1;

    switch (pTimer->DevNo)
    {
        case 0:
        	NVIC_ClearPendingIRQ(GRTC_0_IRQn);
            NVIC_DisableIRQ(GRTC_0_IRQn);
            break;
        case 1:
        	NVIC_ClearPendingIRQ(GRTC_1_IRQn);
            NVIC_DisableIRQ(GRTC_1_IRQn);
            break;
        case 2:
            NVIC_DisableIRQ(GRTC_2_IRQn);
            NVIC_ClearPendingIRQ(GRTC_2_IRQn);
            break;
        case 3:
            NVIC_DisableIRQ(GRTC_3_IRQn);
            NVIC_ClearPendingIRQ(GRTC_3_IRQn);
            break;
    }


    NRF_GRTC->TASKS_STOP = 1;
}

static void nRFxGrtcReset(TimerDev_t * const pTimer)
{
	NRF_GRTC->TASKS_CLEAR = 1;
}

// GRTC is fixed at 16MHz.
static uint32_t nRFxGrtcSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	pTimer->Freq = 16000000;

    // Pre-calculate periods for faster timer counter to time conversion use later
    pTimer->nsPeriod = 1000000000ULL / (uint64_t)pTimer->Freq;     // Period in nsec

    return pTimer->Freq;
}

static uint64_t nRFxGrtcGetTickCount(TimerDev_t * const pTimer)
{
	uint32_t cl;
	uint32_t ch;

	do {
		cl = NRF_GRTC->SYSCOUNTER[pTimer->DevNo].SYSCOUNTERL;
		ch = NRF_GRTC->SYSCOUNTER[pTimer->DevNo].SYSCOUNTERH;
	} while (ch & GRTC_SYSCOUNTER_SYSCOUNTERH_BUSY_Msk);

	if (ch & GRTC_SYSCOUNTER_SYSCOUNTERH_OVERFLOW_Msk)
	{
		// counter overflow
		pTimer->Rollover += 0x10000000000000ULL;
	}
	ch &= GRTC_SYSCOUNTER_SYSCOUNTERH_VALUE_Msk;

	return (((uint64_t)ch << 32ULL) | (cl & GRTC_SYSCOUNTER_SYSCOUNTERL_VALUE_Msk)) + pTimer->Rollover;
}

static int nRFxGrtcGetMaxTrigger(TimerDev_t * const pTimer)
{
	return s_nRfxGrtcData[pTimer->DevNo].MaxNbTrigEvt;
}

static uint64_t nRFxGrtcEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                              	  	 TimerTrigEvtHandler_t Handler, void *pContext)
{
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];

    if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt)
        return 0;

    uint32_t cc = (nsPeriod + (pTimer->nsPeriod >> 1)) / pTimer->nsPeriod;

    if (cc <= 0)
    {
        return 0;
    }

    rtc.Trigger[TrigNo].Type = Type;
    rtc.CC[TrigNo] = cc;

    int idx = rtc.StartCCIdx + TrigNo;
    NRF_GRTC->EVENTS_COMPARE[idx] = 0;

    switch (rtc.DevNo)
    {
		case 0:
			NRF_GRTC->INTENSET0 = (1<<idx);
			NVIC_ClearPendingIRQ(GRTC_0_IRQn);
			NVIC_EnableIRQ(GRTC_0_IRQn);
			break;
		case 1:
			NRF_GRTC->INTENSET1 = (1<<idx);
			NVIC_ClearPendingIRQ(GRTC_1_IRQn);
			NVIC_EnableIRQ(GRTC_1_IRQn);
			break;
		case 2:
			NRF_GRTC->INTENSET2 = (1<<idx);
			NVIC_ClearPendingIRQ(GRTC_2_IRQn);
			NVIC_EnableIRQ(GRTC_2_IRQn);
			break;
		case 3:
			NRF_GRTC->INTENSET3 = (1<<idx);
			NVIC_ClearPendingIRQ(GRTC_3_IRQn);
			NVIC_EnableIRQ(GRTC_3_IRQn);
			break;
    }

    rtc.Trigger[TrigNo].nsPeriod = pTimer->nsPeriod * (uint64_t)cc;
    rtc.Trigger[TrigNo].Handler = Handler;
    rtc.Trigger[TrigNo].pContext = pContext;

    uint64_t period = pTimer->nsPeriod * (uint64_t)cc;

    if (Type == TIMER_TRIG_TYPE_CONTINUOUS)
    {
    	NRF_GRTC->CC[idx].CCADD = rtc.CC[TrigNo];
    }
    else
    {
    	NRF_GRTC->CC[idx].CCADD = 0;
    }
    NRF_GRTC->TASKS_CAPTURE[idx] = 1;

    uint64_t v = *(uint64_t*)&NRF_GRTC->CC[idx] + cc;

    *(uint64_t*)&NRF_GRTC->CC[idx] = v;

    NRF_GRTC->CC[idx].CCEN = 1;

    return period; // Return real period in nsec
}

static void nRFxGrtcDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];

	if (TrigNo < 0 || TrigNo >= rtc.MaxNbTrigEvt)
        return;

    int idx = rtc.StartCCIdx + TrigNo;

    rtc.CC[TrigNo] = 0;

    NRF_GRTC->CC[idx].CCEN = 0;

    switch (rtc.DevNo)
    {
		case 0:
			NRF_GRTC->INTENCLR0 = (1<<idx);
			NVIC_ClearPendingIRQ(GRTC_0_IRQn);
			NVIC_DisableIRQ(GRTC_0_IRQn);
			break;
		case 1:
			NRF_GRTC->INTENCLR1 = (1<<idx);
			NVIC_ClearPendingIRQ(GRTC_1_IRQn);
			NVIC_DisableIRQ(GRTC_1_IRQn);
			break;
		case 2:
			NRF_GRTC->INTENCLR2 = (1<<idx);
			NVIC_ClearPendingIRQ(GRTC_2_IRQn);
			NVIC_DisableIRQ(GRTC_2_IRQn);
			break;
		case 3:
			NRF_GRTC->INTENCLR3 = (1<<idx);
			NVIC_ClearPendingIRQ(GRTC_3_IRQn);
			NVIC_DisableIRQ(GRTC_3_IRQn);
			break;
    }

    NRF_GRTC->EVENTS_COMPARE[idx] = 0;

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
    if (pCfg->DevNo < 0 || pCfg->DevNo >= TIMER_NRFX_RTC_MAX || pCfg->Freq > TIMER_NRFX_RTC_BASE_FREQ)
    {
    	return false;
    }

    pTimer->DevNo = pCfg->DevNo;
    pTimer->EvtHandler = pCfg->EvtHandler;
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];
//	NRF_GRTC_Type *reg = s_nRfxGrtcData[pTimer->DevNo].pReg;

    rtc.pTimer = pTimer;

    memset(rtc.Trigger, 0, sizeof(rtc.Trigger));


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

    NRF_GRTC->TASKS_STOP = 1;
    NRF_GRTC->TASKS_CLEAR = 1;


    // Init interrupt for RTC
	switch (pCfg->DevNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(GRTC_0_IRQn);
			NVIC_SetPriority(GRTC_0_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(GRTC_0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(GRTC_1_IRQn);
			NVIC_SetPriority(GRTC_1_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(GRTC_1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(GRTC_2_IRQn);
			NVIC_SetPriority(GRTC_2_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(GRTC_2_IRQn);
			break;
		case 3:
			NVIC_ClearPendingIRQ(GRTC_3_IRQn);
			NVIC_SetPriority(GRTC_3_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(GRTC_3_IRQn);
			break;
	}

    nRFxGrtcSetFrequency(pTimer, pCfg->Freq);

    if (s_nRfxGrtcSem == 0)
    {
		NRF_GRTC->MODE |= GRTC_MODE_SYSCOUNTEREN_Enabled;
		NRF_GRTC->TASKS_START = 1;
    }

    s_nRfxGrtcSem++;

    return true;
}

static void GrtcIRQHandler(int DevNo)
{
	TimerDev_t *timer = s_nRfxGrtcData[DevNo].pTimer;
	nRFGrtcData_t &rtc = s_nRfxGrtcData[DevNo];
    uint32_t evt = 0;
    uint64_t count = nRFxGrtcGetTickCount(timer);
    int idx = rtc.StartCCIdx;

    timer->LastCount = count;

    for (int i = 0; i < rtc.MaxNbTrigEvt; i++, idx++)
    {
        if (NRF_GRTC->EVENTS_COMPARE[idx])
        {
            evt |= TIMER_EVT_TRIGGER(i);
            NRF_GRTC->EVENTS_COMPARE[idx] = 0;

            if (rtc.Trigger[i].Handler)
            {
            	rtc.Trigger[i].Handler(timer, i, rtc.Trigger[i].pContext);
            }
        }
    }

    if (timer->EvtHandler)
    {
        timer->EvtHandler(timer, evt);
    }
}

extern "C" {

#if defined(NRF54L15_XXAA)
__WEAK void GRTC_0_IRQHandler(void)
{
	GrtcIRQHandler(0);
}

__WEAK void GRTC_1_IRQHandler(void)
{
	GrtcIRQHandler(1);
}

__WEAK void GRTC_2_IRQHandler(void)
{
	GrtcIRQHandler(2);
}

__WEAK void GRTC_3_IRQHandler(void)
{
	GrtcIRQHandler(3);
}

#endif

}

