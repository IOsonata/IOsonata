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

// nRF91 and the nRF5340 application core name their peripherals by security
// state: the secure alias in a secure build, the non secure one in a non
// secure build. The nRF5340 network core has the non secure ones only.
#if defined(NRF5340_XXAA_NETWORK) || \
	((defined(NRF91_SERIES) || defined(NRF5340_XXAA_APPLICATION)) && defined(NRF_TRUSTZONE_NONSECURE))
#define NRF_TIMER0			NRF_TIMER0_NS
#define NRF_TIMER1			NRF_TIMER1_NS
#define NRF_TIMER2			NRF_TIMER2_NS
#define NRF_CLOCK			NRF_CLOCK_NS
#elif defined(NRF91_SERIES) || defined(NRF5340_XXAA_APPLICATION)
#define NRF_TIMER0			NRF_TIMER0_S
#define NRF_TIMER1			NRF_TIMER1_S
#define NRF_TIMER2			NRF_TIMER2_S
#define NRF_CLOCK			NRF_CLOCK_S
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
#endif

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	uint32_t MaxFreq;
	NRF_TIMER_Type *pReg;
	int MaxNbTrigEvt;		//!< Number of trigger is not the same for all timers.
	int CountCC;			//!< Index of last CC register to use for counter reading
	uint32_t *pCC;			//!< Compare value of each trigger, MaxNbTrigEvt entries
	TimerTrig_t *pTrigger;	//!< State of each trigger, MaxNbTrigEvt entries
} nRFTimerData_t;
#pragma pack(pop)

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
static TimerDev_t *s_pnRFxTimerDev[TIMER_NRFX_HF_MAX];

// What does not change is constant, so it stays in flash. Only the trigger
// state above and the device pointers are in RAM.
alignas(4) static const nRFTimerData_t s_nRFxTimerData[TIMER_NRFX_HF_MAX] = {
	{
		.DevNo = TIMER_NRFX_RTC_MAX,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER0,
		.MaxNbTrigEvt = TIMER0_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER0_CC_NUM - 1,
		.pCC = s_nRFxTimer0CC,
		.pTrigger = s_nRFxTimer0Trig,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 1,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER1,
		.MaxNbTrigEvt = TIMER1_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER1_CC_NUM - 1,
		.pCC = s_nRFxTimer1CC,
		.pTrigger = s_nRFxTimer1Trig,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 2,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER2,
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
		.MaxNbTrigEvt = TIMER3_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER3_CC_NUM - 1,
		.pCC = s_nRFxTimer3CC,
		.pTrigger = s_nRFxTimer3Trig,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 4,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER4,
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
		.MaxNbTrigEvt = TIMER5_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER5_CC_NUM - 1,
		.pCC = s_nRFxTimer5CC,
		.pTrigger = s_nRFxTimer5Trig,
	},
	{
		.DevNo = TIMER_NRFX_RTC_MAX + 6,
		.MaxFreq = TIMER_NRFX_HF_BASE_FREQ,
		.pReg = NRF_TIMER6,
		.MaxNbTrigEvt = TIMER6_CC_NUM - 1, // Reserve last CC for reading counter
		.CountCC = TIMER6_CC_NUM - 1,
		.pCC = s_nRFxTimer6CC,
		.pTrigger = s_nRFxTimer6Trig,
	},
#endif
};

static std::atomic<int> s_nRfxHFClockSem(0);

// Set by nRFxTimerInit. The interrupt handlers below are kept by the linker as soon as
// this file is part of a build, used or not. They reach the timer data
// through this pointer, so that the data is linked only when a timer is
// initialized.
static const nRFTimerData_t *s_pnRFxTimerData = nullptr;

static void TimerIRQHandler(int DevNo)
{
	if (s_pnRFxTimerData == nullptr)
	{
		// No timer of this kind was initialized
		return;
	}

	NRF_TIMER_Type *reg = s_pnRFxTimerData[DevNo].pReg;
	const nRFTimerData_t &tdata = s_pnRFxTimerData[DevNo];
	TimerDev_t *timer = s_pnRFxTimerDev[DevNo];
    uint32_t evt = 0;
	uint32_t t = reg->CC[tdata.CountCC];	// Preserve comparator

	// Read counter value
    uint32_t count;
	reg->TASKS_CAPTURE[tdata.CountCC] = 1;
	while ((count = reg->CC[tdata.CountCC]) != reg->CC[tdata.CountCC]);

	reg->CC[tdata.CountCC] = t;	// Restore comparator

    if (count < timer->LastCount)
    {
        // Counter wrap arround
        evt |= TIMER_EVT_COUNTER_OVR;
        timer->Rollover += 0x100000000ULL;
    }

    timer->LastCount = count;

    for (int i = 0; i < tdata.MaxNbTrigEvt; i++)
    {
        if (reg->EVENTS_COMPARE[i])
        {
            evt |= TIMER_EVT_TRIGGER(i);
            reg->EVENTS_COMPARE[i] = 0;
            if (tdata.pTrigger[i].Type == TIMER_TRIG_TYPE_CONTINUOUS)
            {
            	reg->CC[i] = count + tdata.pCC[i];;
            }
            if (tdata.pTrigger[i].Handler)
            {
            	tdata.pTrigger[i].Handler(timer, i, tdata.pTrigger[i].pContext);
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

	// Clock source not available.  Only 64MHz XTAL
	if (s_nRfxHFClockSem <= 0)
	{
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		if (!(NRF_CLOCK->PLL.STAT & CLOCK_PLL_STAT_STATE_Msk))
		{
			NRF_CLOCK->TASKS_PLLSTART = 1;
		}

		if (!(NRF_CLOCK->XO.STAT & CLOCK_XO_STAT_STATE_Msk))
		{
			NRF_CLOCK->TASKS_XOSTART = 1;
		}

#else
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
#endif
	}

	s_nRfxHFClockSem++;

    reg->TASKS_START = 1;

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

	s_nRfxHFClockSem--;

	reg->TASKS_STOP = 1;

	if (s_nRfxHFClockSem <= 0)
	{
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		NRF_CLOCK->TASKS_XOSTOP = 1;
#else
		NRF_CLOCK->TASKS_HFCLKSTOP = 1;
#endif
		s_nRfxHFClockSem = 0;
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

    reg->INTENSET = 0;
}

static void nRFxTimerReset(TimerDev_t * const pTimer)
{
	s_nRFxTimerData[pTimer->DevNo - TIMER_NRFX_RTC_MAX].pReg->TASKS_CLEAR = 1;
}

static uint32_t nRFxTimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	NRF_TIMER_Type *reg = s_nRFxTimerData[devno].pReg;

	reg->TASKS_STOP = 1;

    uint32_t prescaler = 0;

    if (Freq > 0)
    {
        uint32_t divisor = TIMER_NRFX_HF_BASE_FREQ / Freq;
        prescaler = 31 - __CLZ(divisor);

        if (prescaler > 9)
        {
            prescaler = 9;
        }
    }

    reg->PRESCALER = prescaler;

    pTimer->Freq = TIMER_NRFX_HF_BASE_FREQ / (1 << prescaler);

    // Tick period rounded down to a nanosecond, 62 for the 62.5 ns of 16 MHz.
    // Not precise enough to convert a count or a trigger period, those are
    // computed from Freq, see TimerTickToTime.
    pTimer->nsPeriod = 1000000000ULL / pTimer->Freq;

    reg->TASKS_START = 1;

    return pTimer->Freq;
}

static uint64_t nRFxTimerGetTickCount(TimerDev_t * const pTimer)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	NRF_TIMER_Type *reg = s_nRFxTimerData[devno].pReg;
	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];

	uint32_t t = reg->CC[tdata.CountCC];	// Preserve comparator

	// Read counter value
	uint32_t count;
	reg->TASKS_CAPTURE[tdata.CountCC] = 1;
	while ((count = reg->CC[tdata.CountCC]) != reg->CC[tdata.CountCC]);

	reg->CC[tdata.CountCC] = t;	// Restore comparator

	if (count < pTimer->LastCount)
	{
		// Counter wrap arround
		pTimer->Rollover += 0x100000000ULL;
	}

	pTimer->LastCount = count;

	return (uint64_t)pTimer->LastCount + pTimer->Rollover;
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

    uint32_t cc = TimerNanosecondToTick(pTimer, nsPeriod);

    if (cc <= 0)
    {
        return 0;
    }

    tdata.pTrigger[TrigNo].Type = Type;
    tdata.pCC[TrigNo] = cc;
    reg->TASKS_CAPTURE[TrigNo] = 1;

    uint32_t count = reg->CC[TrigNo];

	reg->INTENSET = TIMER_INTENSET_COMPARE0_Msk << TrigNo;

    reg->CC[TrigNo] = count + cc;;

    if (count < pTimer->LastCount)
    {
        // Counter wrap around
        pTimer->Rollover += 0x100000000ULL;
    }

    pTimer->LastCount = count;

    tdata.pTrigger[TrigNo].nsPeriod = TimerTickToNanosecond(pTimer, cc);
    tdata.pTrigger[TrigNo].Handler = Handler;
    tdata.pTrigger[TrigNo].pContext = pContext;

    return tdata.pTrigger[TrigNo].nsPeriod; // Return real period in nsec
}

static void nRFxTimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	int devno = pTimer->DevNo - TIMER_NRFX_RTC_MAX;
	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];
	NRF_TIMER_Type *reg = tdata.pReg;

	if (TrigNo < 0 || TrigNo >= tdata.MaxNbTrigEvt)
        return;

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
	if (pCfg->DevNo < TIMER_NRFX_RTC_MAX
			|| pCfg->DevNo
					>= (TIMER_NRFX_HF_MAX + TIMER_NRFX_RTC_MAX)|| pCfg->Freq > TIMER_NRFX_HF_BASE_FREQ)
	{
		return false;
	}

	int devno = pCfg->DevNo - TIMER_NRFX_RTC_MAX;
	pTimer->DevNo = pCfg->DevNo;
	pTimer->EvtHandler = pCfg->EvtHandler;
	s_pnRFxTimerData = s_nRFxTimerData;

	const nRFTimerData_t &tdata = s_nRFxTimerData[devno];
	NRF_TIMER_Type *reg = s_nRFxTimerData[devno].pReg;

	s_pnRFxTimerDev[devno] = pTimer;

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
	pTimer->DisableExtTrigger = nRFxTimerDisableExtTrigger;
	pTimer->EnableExtTrigger = nRFxTimerEnableExtTrigger;

	reg->TASKS_STOP = 1;
	reg->TASKS_CLEAR = 1;

	// Only support timer mode, 32-bit counter
	reg->MODE = TIMER_MODE_MODE_Timer;
	reg->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

	// Init clock source for TIMERs
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	if (!(NRF_CLOCK->XO.STAT & CLOCK_XO_STAT_STATE_Msk))
	{
		// Clock source not available.  Only 32MHz XTAL
//		s_nRfxHFClockSem++;
		NRF_CLOCK->TASKS_XOSTART = 1;

		// Check if HFCLK is running or not
		int timout = 1000000;
		while (timout-- > 0 && NRF_CLOCK->EVENTS_XOSTARTED == 0);

		if (timout <= 0)
			return false;

		NRF_CLOCK->EVENTS_XOSTARTED = 0;
	}
#else
	if (!(NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk))
	{
		NRF_CLOCK->TASKS_HFCLKSTOP = 1;

		// Clock source not available.  Only 64MHz XTAL
//		s_nRfxHFClockSem++;
		NRF_CLOCK->TASKS_HFCLKSTART = 1;

		// Check if HFCLK is running or not
		int timout = 1000000;
		do
		{
			if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk)
					&& NRF_CLOCK->EVENTS_HFCLKSTARTED)
				break;

		} while (timout-- > 0);

		if (timout <= 0)
			return false;

		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	}
#endif
	else
	{
		// Clock source already enabled externally.
		// This is to ensure we don't disable it when all timers are disabled
		s_nRfxHFClockSem++;
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

