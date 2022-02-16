/**-------------------------------------------------------------------------
@file	timer_re01_tmr.cpp

@brief	Renesas RE01 TMR timer class implementation

This timer is configured as 16bits counter using TMR0 & TMR1 in cascade

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
#include "re01xxx.h"

#include "timer_re01.h"
#include "interrupt_re01.h"

extern RE01_TimerData_t g_Re01TimerData[RE01_TIMER_MAXCNT];

// TMR0 OVR Interrupt handler
// Possible IELS : 4, 12, 20, 28
// 20 is chosen
static void Re01TmrOvrIRQHandler(int IntNo, void *pCtx)
{
	g_Re01TimerData[2].pTimer->Rollover += 0x10000ULL;
	if (g_Re01TimerData[2].pTimer->EvtHandler)
	{
		g_Re01TimerData[2].pTimer->EvtHandler(g_Re01TimerData[2].pTimer, TIMER_EVT_COUNTER_OVR);
	}
}

// TMR0 Compare A interrupt
// Possible IELS : 2, 10, 18, 26
// 18 is chosen
static void Re01TmrTcmAIRQHandler(int IntNo, void *pCtx)
{
	// TMR does not support periodic trigger.
	// Do it manually
	if (g_Re01TimerData[2].Trigger[0].Type == TIMER_TRIG_TYPE_CONTINUOUS)
	{
		TMR01->TCORA = (g_Re01TimerData[2].CC[0] + TMR01->TCNT);
	}
	else
	{
		TMR0->TCR_b.CMIEA = 0;
		TMR01->TCORA = 0;
	}

	if (g_Re01TimerData[2].Trigger[0].Handler)
	{
		g_Re01TimerData[2].Trigger[0].Handler(g_Re01TimerData[2].pTimer, 0, g_Re01TimerData[2].Trigger[0].pContext);
	}
	else if (g_Re01TimerData[2].pTimer->EvtHandler)
	{
		g_Re01TimerData[2].pTimer->EvtHandler(g_Re01TimerData[2].pTimer, TIMER_EVT_TRIGGER(0));
	}
}

// TMR0 Compare B interrupt
// Possible IELS : 3, 11, 19, 27
// 18 is chosen
static void Re01TmrTcmBIRQHandler(int IntNo, void *pCtx)
{
	// TMR does not support periodic trigger.
	// Do it manually
	if (g_Re01TimerData[2].Trigger[1].Type == TIMER_TRIG_TYPE_CONTINUOUS)
	{
		TMR01->TCORB = (g_Re01TimerData[2].CC[1] + TMR01->TCNT);
	}
	else
	{
		TMR0->TCR_b.CMIEB = 0;
		TMR01->TCORB = 0;
	}

	if (g_Re01TimerData[2].Trigger[1].Handler)
	{
		g_Re01TimerData[2].Trigger[1].Handler(g_Re01TimerData[2].pTimer, 1, g_Re01TimerData[2].Trigger[1].pContext);
	}
	else if (g_Re01TimerData[2].pTimer->EvtHandler)
	{
		g_Re01TimerData[2].pTimer->EvtHandler(g_Re01TimerData[2].pTimer, TIMER_EVT_TRIGGER(1));
	}
}

/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
static bool Re01TmrEnable(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

	// 16bits mode
	TMR0->TCCR = TMR0_TCCR_CSS_Msk;

	return true;
}

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 */
static void Re01TmrDisable(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

	TMR0->TCCR = 0;
	TMR1->TCCR = 0;
}

/**
 * @brief   Reset timer.
 */
static void Re01TmrReset(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

	TMR01->TCNT = 0;
	pTimer->LastCount = 0;
	pTimer->Rollover = 0;
}

/**
 * @brief	Set timer main frequency.
 *
 * This function allows dynamically changing the timer frequency.  Timer
 * will be reset and restarted with new frequency
 *
 * @param 	Freq : Frequency in Hz
 *
 * @return  Real frequency
 */
static uint32_t Re01TmrSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];
	uint32_t div = Freq > 0 ? (dev->BaseFreq + (Freq >> 1)) / Freq : 1;
	uint32_t tmp = 0;

	TMR0->TCCR = TMR0_TCCR_CSS_Msk;

	if (div < 2)
	{
		// PCLK
		TMR1->TCCR = 1 << TMR1_TCCR_CSS_Pos;
	}
	else if (div < 8)
	{
		// PCLK/2
		TMR1->TCCR = (1 << TMR1_TCCR_CSS_Pos) | (1 << TMR1_TCCR_CKS_Pos);
		tmp = 1;
	}
	else if (div < 32)
	{
		// PCLK/8
		TMR1->TCCR = (1 << TMR1_TCCR_CSS_Pos) | (2 << TMR1_TCCR_CKS_Pos);
		tmp = 3;
	}
	else if (div < 64)
	{
		// PCLK/32
		TMR1->TCCR = (1 << TMR1_TCCR_CSS_Pos) | (3 << TMR1_TCCR_CKS_Pos);
		tmp = 5;
	}
	else if (div < 1024)
	{
		// PCLK/64
		TMR1->TCCR = (1 << TMR1_TCCR_CSS_Pos) | (4 << TMR1_TCCR_CKS_Pos);
		tmp = 6;
	}
	else if (div < 8192)
	{
		// PCLK/1024
		TMR1->TCCR = (1 << TMR1_TCCR_CSS_Pos) | (5 << TMR1_TCCR_CKS_Pos);
		tmp = 10;
	}
	else
	{
		// PCLK/8192
		TMR1->TCCR = (1 << TMR1_TCCR_CSS_Pos) | (6 << TMR1_TCCR_CKS_Pos);
		tmp = 13;
	}

	pTimer->Freq = dev->BaseFreq >> tmp;

	// Pre-calculate periods for faster timer counter to time conversion use later
    pTimer->nsPeriod = (1000000000ULL + ((uint64_t)pTimer->Freq >> 1))/ (uint64_t)pTimer->Freq;     // Period in nsec

    return pTimer->Freq;
}

static uint64_t Re01TmrGetTickCount(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

    uint16_t cnt = TMR01->TCNT;

    pTimer->LastCount = cnt;

    return (uint64_t)cnt + pTimer->Rollover;
}

/**
 * @brief	Get maximum available timer trigger event for the timer.
 *
 * @return	count
 */
static int Re01TmrGetMaxTrigger(TimerDev_t * const pTimer)
{
	return RE01_TIMER_TRIG_MAXCNT;
}

/**
 * @brief	Enable a specific nanosecond timer trigger event.
 *
 * @param   TrigNo : Trigger number to enable. Index value starting at 0
 * @param   nsPeriod : Trigger period in nsec.
 * @param   Type     : Trigger type single shot or continuous
 * @param	Handler	 : Optional Timer trigger user callback
 * @param   pContext : Optional pointer to user private data to be passed
 *                     to the callback. This could be a class or structure pointer.
 *
 * @return  real period in nsec based on clock calculation
 */
static uint64_t Re01TmrEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
							  TimerTrigEvtHandler_t const Handler, void * const pContext)
{
    if (pTimer == NULL || TrigNo < 0 || TrigNo >= RE01_TIMER_TRIG_MAXCNT)
        return 0;

    RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];
    uint64_t cc = (nsPeriod + (pTimer->nsPeriod >> 1ULL)) / pTimer->nsPeriod;

    if (cc <= 2ULL || cc >= 0x10000ULL)
    {
        return 0;
    }

    uint64_t retval = 0;

    dev->Trigger[TrigNo].Type = Type;
    dev->CC[TrigNo] = cc & 0xFFFF;

    dev->Trigger[TrigNo].nsPeriod = pTimer->nsPeriod * (uint64_t)cc;
    dev->Trigger[TrigNo].Handler = Handler;
    dev->Trigger[TrigNo].pContext = pContext;

	// TMR does not support periodic trigger.
	// Do it manually within interrupt handler
    uint8_t evtid = 0;
   	if (TrigNo > 0)
   	{
   		evtid = RE01_EVTID_TMR_CMIB0;
   		dev->IrqMatch[TrigNo] = Re01RegisterIntHandler(evtid, dev->IntPrio, Re01TmrTcmBIRQHandler, dev);

   		TMR01->TCORB = (cc + TMR01->TCNT);
   	}
    else
    {
   		evtid = RE01_EVTID_TMR_CMIA0;
   		dev->IrqMatch[TrigNo] = Re01RegisterIntHandler(evtid, dev->IntPrio, Re01TmrTcmAIRQHandler, dev);

   		TMR01->TCORA = (cc + TMR01->TCNT);
    }

	if (dev->IrqMatch[TrigNo] == -1)
	{
		return false;
	}

	TMR0->TCR |= (TrigNo + 1) << TMR0_TCR_CMIEA_Pos;

    return pTimer->nsPeriod * cc; // Return real period in nsec
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static void Re01TmrDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
    if (pTimer == NULL || TrigNo < 0 || TrigNo >= RE01_TIMER_TRIG_MAXCNT)
        return;

    RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

    TMR0->TCR &= ~((TrigNo + 1) << TMR0_TCR_CMIEA_Pos);

    if (dev->IrqMatch[TrigNo] != -1)
    {
    	Re01UnregisterIntHandler(dev->IrqMatch[TrigNo]);
    	dev->IrqMatch[TrigNo] = (IRQn_Type)-1;
    }
    if (TrigNo > 0)
    {
		TMR01->TCORB = 0;
    }
    else
    {
		TMR01->TCORA = 0;
    }
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static void Re01TmrDisableExtTrigger(TimerDev_t * const pTimer)
{
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
static bool Re01TmrEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	switch (Sense)
	{
		case TIMER_EXTTRIG_SENSE_LOW_TRANSITION:
			break;
		case TIMER_EXTTRIG_SENSE_HIGH_TRANSITION:
			break;
		case TIMER_EXTTRIG_SENSE_TOGGLE:
			break;
		case TIMER_EXTTRIG_SENSE_DISABLE:
		default:
			break;
	}

	return true;
}

/**
 * @brief	Get first available timer trigger index.
 *
 * This function returns the first available timer trigger to be used to with
 * EnableTimerTrigger
 *
 * @return	success : Timer trigger index
 * 			fail : -1
 */
static int Re01TmrFindAvailTrigger(TimerDev_t * const pTimer)
{
	return 0;
}

/**
 * @brief   Timer initialization.
 *
 * This is specific to each architecture.
 *
 * @param	Cfg	: Timer configuration data.
 *
 * @return
 * 			- true 	: Success
 * 			- false : Otherwise
 */
bool Re01TmrInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg)
{
	if (pCfg->Freq > 64000000)
	{
		return false;
	}

	pTimerData->IntPrio = pCfg->IntPrio;
	pTimerData->pTimer->DevNo = pCfg->DevNo;
	pTimerData->pTimer->EvtHandler = pCfg->EvtHandler;
    pTimerData->pTimer->Disable = Re01TmrDisable;
    pTimerData->pTimer->Enable = Re01TmrEnable;
    pTimerData->pTimer->Reset = Re01TmrReset;
    pTimerData->pTimer->GetTickCount = Re01TmrGetTickCount;
    pTimerData->pTimer->SetFrequency = Re01TmrSetFrequency;
    pTimerData->pTimer->GetMaxTrigger = Re01TmrGetMaxTrigger;
    pTimerData->pTimer->FindAvailTrigger = Re01TmrFindAvailTrigger;
    pTimerData->pTimer->DisableTrigger = Re01TmrDisableTrigger;
    pTimerData->pTimer->EnableTrigger = Re01TmrEnableTrigger;
    pTimerData->pTimer->DisableExtTrigger = Re01TmrDisableExtTrigger;
    pTimerData->pTimer->EnableExtTrigger = Re01TmrEnableExtTrigger;

	MSTP->MSTPCRD &= ~MSTP_MSTPCRD_MSTPD1_Msk;

	TIMER_CLKSRC clksrc = pCfg->ClkSrc;

	if (pCfg->ClkSrc == TIMER_CLKSRC_EXT)
	{
		// External clock source
		// TODO:
	}
	else
	{
		// Only the PCLK is the TMR internal clock source
		pTimerData->BaseFreq = SystemPeriphClockGet(0);
	}

	uint32_t f = Re01TmrSetFrequency(pTimerData->pTimer, pCfg->Freq);
	if (f <= 0)
	{
		return false;
	}

	TMR0->TCR |= TMR0_TCR_OVIE_Msk;

	pTimerData->IrqOvr = Re01RegisterIntHandler(RE01_EVTID_TMR_OVF0, pCfg->IntPrio, Re01TmrOvrIRQHandler, pTimerData);
	if (pTimerData->IrqOvr == -1)
	{
		return false;
	}

    return true;
}


