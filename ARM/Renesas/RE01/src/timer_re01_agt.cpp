/**-------------------------------------------------------------------------
@file	timer_re01_agt.cpp

@brief	Renesas RE01 AGT timer class implementation

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

extern RE01_TimerData_t g_Re01TimerData[RE01_TIMER_MAXCNT];

/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
static bool Re01AgtEnable(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

	dev->pAgtReg->AGTCR |= AGT0_AGTCR_TSTART_Msk;

	return true;
}

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 */
static void Re01AgtDisable(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

	dev->pAgtReg->AGTCR &= ~AGT0_AGTCR_TSTART_Msk;
}

/**
 * @brief   Reset timer.
 */
static void Re01AgtReset(TimerDev_t * const pTimer)
{
//	g_Re01TimerData[pTimer->DevNo].pLPTimReg->ICR = g_Re01TimerData[pTimer->DevNo].pLPTimReg->ISR;
//	g_Re01TimerData[pTimer->DevNo].pLPTimReg->CNT = 0;
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
static uint32_t Re01AgtSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];
	uint32_t div = Freq > 0 ? (dev->BaseFreq + (Freq >> 1)) / Freq : 1;
	uint32_t agtmr2 = dev->pAgtReg->AGTMR2 & ~AGT0_AGTMR2_CKS_Msk;

	agtmr2 |= 31 - __CLZ(div);

	pTimer->Freq = dev->BaseFreq >> (agtmr2 & 0x7f);

	// Pre-calculate periods for faster timer counter to time conversion use later
    pTimer->nsPeriod = (1000000000ULL + ((uint64_t)pTimer->Freq >> 1))/ (uint64_t)pTimer->Freq;     // Period in nsec

	dev->pAgtReg->AGTMR2 = agtmr2;

	return pTimer->Freq;
}

static uint64_t Re01AgtGetTickCount(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

    uint32_t cnt = 0;

    cnt = dev->pAgtReg->AGT;

    return 0xFFFFULL - (uint64_t)cnt + pTimer->Rollover;
}

/**
 * @brief	Get maximum available timer trigger event for the timer.
 *
 * @return	count
 */
int Re01AgtGetMaxTrigger(TimerDev_t * const pTimer)
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
uint64_t Re01AgtEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
								 TimerTrigEvtHandler_t const Handler, void * const pContext)
{
    if (TrigNo < 0 || TrigNo >= RE01_TIMER_TRIG_MAXCNT)
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

//    uint32_t count = dev->DevNo > 0 ? dev->pAgtReg0->AGT : dev->pAgtReg0->AGT;
//    while (count != dev->DevNo > 0 ? dev->pAgtReg0->AGT : dev->pAgtReg0->AGT) {
//    	count = dev->DevNo > 0 ? dev->pAgtReg0->AGT : dev->pAgtReg0->AGT;
//    }

    dev->pAgtReg->AGTCR &= ~(AGT0_AGTCR_TSTART_Msk);
    while (dev->pAgtReg->AGTCR & AGT0_AGTCR_TCSTF_Msk);

    dev->pAgtReg->AGT = cc;
    //dev->pAgtReg->AGTCMA = cc;
    //dev->pAgtReg->AGTCMSR_b.TCMEA = 1;

    dev->pAgtReg->AGTCR |= AGT0_AGTCR_TSTART_Msk;
    while ((dev->pAgtReg->AGTCR & AGT0_AGTCR_TCSTF_Msk) == 0);

    return pTimer->nsPeriod * cc; // Return real period in nsec
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Re01AgtDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	//g_Re01TimerData[pTimer->DevNo].pLPTimReg->IER &= ~LPTIM_IER_CMPMIE;
	//g_Re01TimerData[pTimer->DevNo].pLPTimReg->CMP = 0;
	//while ((g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->ISR & LPTIM_ISR_CMPOK) == 0);
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Re01AgtDisableExtTrigger(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];

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
bool Re01AgtEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];
/*
	LPTIM_TypeDef *reg = g_Re01TimerData[pTimer->DevNo].pLPTimReg;
	uint32_t tmp = reg->CFGR & ~(LPTIM_CFGR_TRIGEN_Msk | LPTIM_CFGR_TRIGSEL_Msk);

	switch (Sense)
	{
		case TIMER_EXTTRIG_SENSE_LOW_TRANSITION:
			tmp |= LPTIM_CFGR_TRIGEN_1;
			break;
		case TIMER_EXTTRIG_SENSE_HIGH_TRANSITION:
			tmp |= LPTIM_CFGR_TRIGEN_0;
			break;
		case TIMER_EXTTRIG_SENSE_TOGGLE:
			tmp |= LPTIM_CFGR_TRIGEN_Msk;
			break;
		case TIMER_EXTTRIG_SENSE_DISABLE:
		default:
			break;
	}
	reg->CFGR = tmp | LPTIM_CFGR_TIMOUT | ((TrigDevNo & 7) << LPTIM_CFGR_TRIGSEL_Pos);// | (LPTIM_CFGR_TRIGEN_Msk);
	reg->OR = 0;
	reg->IER |= LPTIM_IER_EXTTRIGIE;
*/
	return true;
}

void Re01AgtIRQHandler(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = &g_Re01TimerData[pTimer->DevNo];
	/*
	uint32_t flag = dev->pLPTimReg->ISR;
	uint32_t evt = 0;
    uint32_t count = dev->pLPTimReg->CNT;
    while (count != dev->pLPTimReg->CNT) {
    	count = dev->pLPTimReg->CNT;
    }

	if (flag & LPTIM_ISR_ARRM)
	{
        pTimer->Rollover += 0x10000ULL;
        evt |= TIMER_EVT_COUNTER_OVR;
	}

	if ((flag & LPTIM_ISR_EXTTRIG) && (dev->pLPTimReg->IER & LPTIM_IER_EXTTRIGIE))
	{
		evt |= TIMER_EVT_EXTTRIG;
	}

	if ((flag & LPTIM_ISR_CMPM) && (dev->pLPTimReg->IER & LPTIM_IER_CMPMIE))
	{
		// NOTE : Single mode cause the counter to stop
		// Reset compare and disable interrupt for single mode
		if (dev->Trigger[0].Type == TIMER_TRIG_TYPE_CONTINUOUS)
		{
			dev->pLPTimReg->CMP = (count + dev->CC[0]) & 0xFFFF;
			while ((dev->pLPTimReg->ISR & LPTIM_ISR_CMPOK) == 0);
		}
		else
		{
			dev->pLPTimReg->IER &= ~LPTIM_IER_CMPMIE;
	    	dev->pLPTimReg->CMP = 0;
		    while ((dev->pLPTimReg->ISR & LPTIM_ISR_CMPOK) == 0);
		}
		evt |= TIMER_EVT_TRIGGER(0);
        if (dev->Trigger[0].Handler)
        {
        	dev->Trigger[0].Handler(pTimer, 0, dev->Trigger[0].pContext);
        }
	}

	pTimer->LastCount = count;

    if (pTimer->EvtHandler)
	{
    	pTimer->EvtHandler(pTimer, evt);
	}

	dev->pLPTimReg->ICR = flag;*/
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
int Re01AgtFindAvailTrigger(TimerDev_t * const pTimer)
{
	return 0;
}

extern "C" void IEL16_IRQHandler(void)
{
	if (ICU->IELSR16 & ICU_IELSR16_IR_Msk)
	{
		ICU->IELSR16 &= ~ICU_IELSR16_IR_Msk;
		Re01AgtIRQHandler(g_Re01TimerData[1].pTimer);

		if (g_Re01TimerData[1].pAgtReg1->AGTCR_b.TUNDF)
		{
			g_Re01TimerData[1].pAgtReg1->AGTCR_b.TUNDF = 0;
			g_Re01TimerData[1].pTimer->Rollover++;
			if (g_Re01TimerData[1].pTimer->EvtHandler)
			{
				g_Re01TimerData[1].pTimer->EvtHandler(g_Re01TimerData[1].pTimer, TIMER_EVT_COUNTER_OVR);
			}
		}

		if (g_Re01TimerData[1].pAgtReg1->AGTCR_b.TCMAF)
		{
			g_Re01TimerData[1].pAgtReg1->AGTCR_b.TCMAF = 0;
			if (g_Re01TimerData[1].Trigger->Handler)
			{
				g_Re01TimerData[1].Trigger->Handler(g_Re01TimerData[1].pTimer, TIMER_EVT_TRIGGER(0), g_Re01TimerData[1].Trigger->pContext);
			}
			else if (g_Re01TimerData[1].pTimer->EvtHandler)
			{
					g_Re01TimerData[1].pTimer->EvtHandler(g_Re01TimerData[1].pTimer, TIMER_EVT_TRIGGER(0));
			}
		}

		if (g_Re01TimerData[1].pAgtReg1->AGTCR_b.TCMBF)
		{
			g_Re01TimerData[1].pAgtReg1->AGTCR_b.TCMBF = 0;
			if (g_Re01TimerData[1].Trigger->Handler)
			{
				g_Re01TimerData[1].Trigger->Handler(g_Re01TimerData[1].pTimer, TIMER_EVT_TRIGGER(1), g_Re01TimerData[1].Trigger->pContext);
			}
			else if (g_Re01TimerData[1].pTimer->EvtHandler)
			{
					g_Re01TimerData[1].pTimer->EvtHandler(g_Re01TimerData[1].pTimer, TIMER_EVT_TRIGGER(1));
			}
		}
	}
	NVIC_ClearPendingIRQ(IEL16_IRQn);
}

extern "C" void IEL26_IRQHandler(void)
{
	if (ICU->IELSR26 & ICU_IELSR26_IR_Msk)
	{
		ICU->IELSR26 &= ~ICU_IELSR26_IR_Msk;
		Re01AgtIRQHandler(g_Re01TimerData[0].pTimer);

		if (g_Re01TimerData[0].pAgtReg->AGTCR_b.TCMAF)
		{
			g_Re01TimerData[0].pAgtReg->AGTCR_b.TCMAF = 0;
			g_Re01TimerData[0].pAgtReg->AGTCMA += g_Re01TimerData[0].CC[0];
			if (g_Re01TimerData[0].Trigger->Handler)
			{
				g_Re01TimerData[0].Trigger->Handler(g_Re01TimerData[0].pTimer, TIMER_EVT_TRIGGER(0), g_Re01TimerData[0].Trigger->pContext);
			}
			else if (g_Re01TimerData[0].pTimer->EvtHandler)
			{
				g_Re01TimerData[0].pTimer->EvtHandler(g_Re01TimerData[0].pTimer, TIMER_EVT_TRIGGER(0));
			}
		}
	}
	NVIC_ClearPendingIRQ(IEL26_IRQn);
}

extern "C" void IEL27_IRQHandler(void)
{
	if (ICU->IELSR27 & ICU_IELSR27_IR_Msk)
	{
		ICU->IELSR27 &= ~ICU_IELSR27_IR_Msk;
		Re01AgtIRQHandler(g_Re01TimerData[0].pTimer);
		if (g_Re01TimerData[0].pAgtReg->AGTCR_b.TUNDF)
		{
			g_Re01TimerData[0].pAgtReg->AGTCR_b.TUNDF = 0;
			g_Re01TimerData[0].pTimer->Rollover++;
			if (g_Re01TimerData[0].pTimer->EvtHandler)
			{
				g_Re01TimerData[0].pTimer->EvtHandler(g_Re01TimerData[0].pTimer, TIMER_EVT_COUNTER_OVR);
			}
		}
/*		if (g_Re01TimerData[0].pAgtReg0->AGTCR_b.TCMAF)
		{
			g_Re01TimerData[0].pAgtReg0->AGTCR_b.TCMAF = 0;
			if (g_Re01TimerData[0].Trigger->Handler)
			{
				g_Re01TimerData[0].Trigger->Handler(g_Re01TimerData[0].pTimer, TIMER_EVT_TRIGGER(0), g_Re01TimerData[0].Trigger->pContext);
			}
			else if (g_Re01TimerData[0].pTimer->EvtHandler)
			{
					g_Re01TimerData[0].pTimer->EvtHandler(g_Re01TimerData[0].pTimer, TIMER_EVT_TRIGGER(0));
			}

		}
		if (g_Re01TimerData[0].pAgtReg0->AGTCR_b.TCMBF)
		{
			g_Re01TimerData[0].pAgtReg0->AGTCR_b.TCMBF = 0;
			if (g_Re01TimerData[0].Trigger->Handler)
			{
				g_Re01TimerData[0].Trigger->Handler(g_Re01TimerData[0].pTimer, TIMER_EVT_TRIGGER(1), g_Re01TimerData[0].Trigger->pContext);
			}
			else if (g_Re01TimerData[0].pTimer->EvtHandler)
			{
					g_Re01TimerData[0].pTimer->EvtHandler(g_Re01TimerData[0].pTimer, TIMER_EVT_TRIGGER(1));
			}
		}*/
	}
	NVIC_ClearPendingIRQ(IEL27_IRQn);
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
bool Re01AgtInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg)
{
	if (pCfg->Freq > 64000000)
	{
		return false;
	}

	//AGT0_Type *reg = pTimerData->pAgtReg0;

	pTimerData->pTimer->DevNo = pCfg->DevNo;
	pTimerData->pTimer->EvtHandler = pCfg->EvtHandler;
    pTimerData->pTimer->Disable = Re01AgtDisable;
    pTimerData->pTimer->Enable = Re01AgtEnable;
    pTimerData->pTimer->Reset = Re01AgtReset;
    pTimerData->pTimer->GetTickCount = Re01AgtGetTickCount;
    pTimerData->pTimer->SetFrequency = Re01AgtSetFrequency;
    pTimerData->pTimer->GetMaxTrigger = Re01AgtGetMaxTrigger;
    pTimerData->pTimer->FindAvailTrigger = Re01AgtFindAvailTrigger;
    pTimerData->pTimer->DisableTrigger = Re01AgtDisableTrigger;
    pTimerData->pTimer->EnableTrigger = Re01AgtEnableTrigger;
    pTimerData->pTimer->DisableExtTrigger = Re01AgtDisableExtTrigger;
    pTimerData->pTimer->EnableExtTrigger = Re01AgtEnableExtTrigger;


	TIMER_CLKSRC clksrc = pCfg->ClkSrc;

	if (pCfg->ClkSrc == TIMER_CLKSRC_DEFAULT)
	{
		if (GetLowFreqOscType() == OSC_TYPE_RC)
		{
			clksrc = TIMER_CLKSRC_LFRC;
		}
		else
		{
			clksrc = TIMER_CLKSRC_LFXTAL;
		}
	}

	MSTP->MSTPCRD &= ~(0x8 >> pCfg->DevNo) ;

	pTimerData->pAgtReg->AGTMR2 = 0;

	if (pCfg->Freq > 0)
	{
		// Prescale
	}
	switch (clksrc)
	{
		case TIMER_CLKSRC_LFRC:
			pTimerData->BaseFreq = 32768;
			pTimerData->pAgtReg->AGTMR1_b.TCK = 4;
			break;

		case TIMER_CLKSRC_LFXTAL:
			pTimerData->BaseFreq = 32768;
			pTimerData->pAgtReg->AGTMR1_b.TCK = 6;
			break;
		case TIMER_CLKSRC_HFRC:
		case TIMER_CLKSRC_HFXTAL:
		default:
			pTimerData->BaseFreq = SystemPeriphClockGet(1);
	}

	Re01AgtSetFrequency(pTimerData->pTimer, pCfg->Freq);

	__IOM uint32_t *ielsr = (__IOM uint32_t*)&ICU->IELSR0;

	if (pTimerData->pTimer->DevNo == 0)
	{
		NVIC_ClearPendingIRQ(IEL26_IRQn);
		NVIC_SetPriority(IEL26_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(IEL26_IRQn);
		ielsr[26] = 0x4;

		NVIC_ClearPendingIRQ(IEL27_IRQn);
		NVIC_SetPriority(IEL27_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(IEL27_IRQn);
		ielsr[27] = 0x13;
		ICU->WUPEN_b.AGT0CAWUPEN = 1;
	}
	else
	{
		NVIC_ClearPendingIRQ(IEL16_IRQn);
		NVIC_SetPriority(IEL16_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(IEL16_IRQn);
		ielsr[16] = 0x6;
	}

//	pTimerData->pAgtReg->AGTIOC_b.TOE = 1;
	pTimerData->pAgtReg->AGTCMA = 0xFFFF;
	pTimerData->pAgtReg->AGTCMB = 0xFFFF;
	pTimerData->pAgtReg->AGT = 0xFFFF;
	pTimerData->pAgtReg->AGTCR &= ~(AGT0_AGTCR_TUNDF_Msk | AGT0_AGTCR_TCMAF_Msk | AGT0_AGTCR_TCMBF_Msk);
	pTimerData->pAgtReg->AGTCR_b.TSTART = 1;
	while (0 == pTimerData->pAgtReg->AGTCR_b.TCSTF);
	return true;
}


