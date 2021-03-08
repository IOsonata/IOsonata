/**-------------------------------------------------------------------------
@file	timer_lptim_stm32l4xx.cpp

@brief	Timer implementation of STM32L4xx LPTIM

This timer has 16bits counter.  Supported clock sources :
- 32KHz LSI (TIMER_CLKSRC_LFRC)
- 32768Hz LSE (TIMER_CLKSRC_LFXTAL(
- 16MHz HSI (TIMER_CLKSRC_HFRC)

@author	Hoang Nguyen Hoan
@date	July 21, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include <stdio.h>
#include <stdbool.h>

#include "stm32l4xx.h"

#include "idelay.h"
#include "timer_stm32l4x.h"
#include "interrupt.h"

/// Low Power Timer
#define STM32L4XX_LPTIMER_MAXFREQ			16000000

#define STM32L4XX_LPTIMER_TRIG_MAXCNT		1

#define STM32L4XX_LPTIMER_CLKSRC_PCLK		0
#define STM32L4XX_LPTIMER_CLKSRC_LSI		1
#define STM32L4XX_LPTIMER_CLKSRC_HSI		2
#define STM32L4XX_LPTIMER_CLKSRC_LSE		3

extern STM32L4XX_TimerData_t g_Stm32l4TimerData[STM32L4XX_TIMER_MAXCNT];

/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
static bool Stm32l4LptEnable(TimerDev_t * const pTimer)
{
	g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->CR = LPTIM_CR_ENABLE;
	//vpLPTimReg->CR |= LPTIM_CR_CNTSTRT;
	return true;
}

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 */
static void Stm32l4LptDisable(TimerDev_t * const pTimer)
{
	g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->CR = 0;
}

/**
 * @brief   Reset timer.
 */
static void Stm32l4LptReset(TimerDev_t * const pTimer)
{
	g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->ICR = g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->ISR;
	g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->CNT = 0;
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
static uint32_t Stm32l4LptSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	STM32L4XX_TimerData_t *dev = &g_Stm32l4TimerData[pTimer->DevNo];
	uint32_t div = Freq > 0 ? (dev->BaseFreq + (Freq >> 1)) / Freq : 1;
	uint32_t tmp = 0;

	if (div < 2)
	{
		tmp = 0;
	}
	else if (div < 4)
	{
		tmp = 1;
	}
	else if (div < 8)
	{
		tmp = 2;
	}
	else if (div < 16)
	{
		tmp = 3;
	}
	else if (div < 32)
	{
		tmp = 4;
	}
	else if (div < 64)
	{
		tmp = 5;
	}
	else if (div < 128)
	{
		tmp = 6;
	}
	else
	{
		tmp = 7;
	}

	pTimer->Freq = dev->BaseFreq >> tmp;

	// Pre-calculate periods for faster timer counter to time conversion use later
    pTimer->nsPeriod = (1000000000ULL + ((uint64_t)pTimer->Freq >> 1))/ (uint64_t)pTimer->Freq;     // Period in nsec

	dev->pLPTimReg->CFGR = (dev->pLPTimReg->CFGR & ~LPTIM_CFGR_PRESC_Msk) | (tmp << LPTIM_CFGR_PRESC_Pos);

	return pTimer->Freq;
}

static uint64_t Stm32l4LptGetTickCount(TimerDev_t * const pTimer)
{
	STM32L4XX_TimerData_t *dev = &g_Stm32l4TimerData[pTimer->DevNo];
    uint32_t cnt = dev->pLPTimReg->CNT;
    while (cnt != dev->pLPTimReg->CNT) {
    	cnt = dev->pLPTimReg->CNT;
    }
	return (uint64_t)cnt + pTimer->Rollover;
}

/**
 * @brief	Get maximum available timer trigger event for the timer.
 *
 * @return	count
 */
int Stm32l4LptGetMaxTrigger(TimerDev_t * const pTimer)
{
	return STM32L4XX_LPTIMER_TRIG_MAXCNT;
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
uint64_t Stm32l4LptEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
								 TimerTrigEvtHandler_t const Handler, void * const pContext)
{
    if (TrigNo < 0 || TrigNo >= STM32L4XX_LPTIMER_TRIG_MAXCNT)
        return 0;

    STM32L4XX_TimerData_t *dev = &g_Stm32l4TimerData[pTimer->DevNo];
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

    uint32_t count = dev->pLPTimReg->CNT;
    while (count != dev->pLPTimReg->CNT) {
    	count = dev->pLPTimReg->CNT;
    }

    dev->pLPTimReg->IER &= ~LPTIM_IER_CMPMIE;
   	dev->pLPTimReg->CMP = (cc + count) & 0xFFFF;
    while ((dev->pLPTimReg->ISR & LPTIM_ISR_CMPOK) == 0);

    dev->pLPTimReg->IER |= LPTIM_IER_CMPMIE;

	// NOTE : Single mode causes the counter to stop.
	// continuous mode is always on in order for the counter to keep counting
    // single mode is simulated by enabling or disabling interrupt.
/*
    if (Type == TIMER_TRIG_TYPE_CONTINUOUS)
    {
    	dev->pLPTimReg->CR &= ~LPTIM_CR_SNGSTRT;
    	dev->pLPTimReg->CR |= LPTIM_CR_CNTSTRT;
    }
    else
    {
    	dev->pLPTimReg->CR &= ~LPTIM_CR_CNTSTRT;
    	dev->pLPTimReg->CR |= LPTIM_CR_SNGSTRT;
    }
*/
    return pTimer->nsPeriod * cc; // Return real period in nsec
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Stm32l4LptDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->IER &= ~LPTIM_IER_CMPMIE;
	g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->CMP = 0;
	//while ((g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg->ISR & LPTIM_ISR_CMPOK) == 0);
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Stm32l4LptDisableExtTrigger(TimerDev_t * const pTimer)
{
	LPTIM_TypeDef *reg = g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg;

	reg->IER &= ~LPTIM_IER_EXTTRIGIE;
	reg->CFGR &= ~(LPTIM_CFGR_TIMOUT | LPTIM_CFGR_TRIGEN_Msk | LPTIM_CFGR_TRIGSEL_Msk);
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
bool Stm32l4LptEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	LPTIM_TypeDef *reg = g_Stm32l4TimerData[pTimer->DevNo].pLPTimReg;
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

	return true;
}

void Stm32l4LptIRQHandler(TimerDev_t * const pTimer)
{
	STM32L4XX_TimerData_t *dev = &g_Stm32l4TimerData[pTimer->DevNo];
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

	dev->pLPTimReg->ICR = flag;
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
int Stm32l4LptFindAvailTrigger(TimerDev_t * const pTimer)
{
	return 0;
}

extern "C" void LPTIM1_IRQHandler()
{
	Stm32l4LptIRQHandler(g_Stm32l4TimerData[0].pTimer);

	NVIC_ClearPendingIRQ(LPTIM1_IRQn);
}

extern "C" void LPTIM2_IRQHandler()
{
	Stm32l4LptIRQHandler(g_Stm32l4TimerData[1].pTimer);

	NVIC_ClearPendingIRQ(LPTIM2_IRQn);
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
bool Stm32l4LPTimInit(STM32L4XX_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg)
{
	if (pCfg->Freq > 16000000)
	{
		return false;
	}

	LPTIM_TypeDef *reg = pTimerData->pLPTimReg;

	pTimerData->pTimer->DevNo = pCfg->DevNo;
	pTimerData->pTimer->EvtHandler = pCfg->EvtHandler;
    pTimerData->pTimer->Disable = Stm32l4LptDisable;
    pTimerData->pTimer->Enable = Stm32l4LptEnable;
    pTimerData->pTimer->Reset = Stm32l4LptReset;
    pTimerData->pTimer->GetTickCount = Stm32l4LptGetTickCount;
    pTimerData->pTimer->SetFrequency = Stm32l4LptSetFrequency;
    pTimerData->pTimer->GetMaxTrigger = Stm32l4LptGetMaxTrigger;
    pTimerData->pTimer->FindAvailTrigger = Stm32l4LptFindAvailTrigger;
    pTimerData->pTimer->DisableTrigger = Stm32l4LptDisableTrigger;
    pTimerData->pTimer->EnableTrigger = Stm32l4LptEnableTrigger;
    pTimerData->pTimer->DisableExtTrigger = Stm32l4LptDisableExtTrigger;
    pTimerData->pTimer->EnableExtTrigger = Stm32l4LptEnableExtTrigger;

	reg->CR &= ~LPTIM_CR_ENABLE;

	uint32_t tmp = RCC_CCIPR_LPTIM1SEL_Msk << (pTimerData->pTimer->DevNo << 1);

	RCC->CCIPR &= ~tmp;

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

	switch (clksrc)
	{
		case TIMER_CLKSRC_LFRC:
			pTimerData->BaseFreq = 32000;
			RCC->BDCR &= ~RCC_BDCR_LSEON;
			RCC->BDCR |= RCC_BDCR_LSEBYP;

			RCC->CSR |= RCC_CSR_LSION;

			while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);

			tmp = STM32L4XX_LPTIMER_CLKSRC_LSI << (RCC_CCIPR_LPTIM1SEL_Pos + (pCfg->DevNo << 1));
			break;
		case TIMER_CLKSRC_LFXTAL:
			pTimerData->BaseFreq = 32768;

			RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
			PWR->CR1 |= PWR_CR1_DBP;
			RCC->BDCR |= RCC_BDCR_BDRST;
			RCC->BDCR &= ~RCC_BDCR_BDRST;

			RCC->BDCR &= ~RCC_BDCR_LSEBYP;

			RCC->BDCR |= RCC_BDCR_LSEON;

			while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0);

			PWR->CR1 &= ~PWR_CR1_DBP;
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;

			tmp = STM32L4XX_LPTIMER_CLKSRC_LSE << (RCC_CCIPR_LPTIM1SEL_Pos + (pCfg->DevNo << 1));
			break;
		case TIMER_CLKSRC_HFRC:
		case TIMER_CLKSRC_HFXTAL:
		default:
			pTimerData->BaseFreq = 16000000;
			RCC->CR |= RCC_CR_HSION;

			while ((RCC->CR & RCC_CR_HSIRDY) == 0);

			tmp = STM32L4XX_LPTIMER_CLKSRC_HSI << (RCC_CCIPR_LPTIM1SEL_Pos + (pCfg->DevNo << 1));
	}

	RCC->CCIPR |= tmp;

	if (pTimerData->pTimer->DevNo == 0)
	{
		RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

		NVIC_ClearPendingIRQ(LPTIM1_IRQn);
		NVIC_SetPriority(LPTIM1_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(LPTIM1_IRQn);
	}
	else
	{
		RCC->APB1ENR2 |= RCC_APB1ENR2_LPTIM2EN;

		NVIC_ClearPendingIRQ(LPTIM2_IRQn);
		NVIC_SetPriority(LPTIM2_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(LPTIM2_IRQn);
	}

	Stm32l4LptSetFrequency(pTimerData->pTimer, pCfg->Freq);

	reg->OR = 0;
	reg->CR = LPTIM_CR_ENABLE;

	reg->ARR = 0xFFFF;
	reg->CMP = 0;

	// Wait for the value to set, otherwise it will cause wrong frequency to set in.
	while ((reg->ISR & LPTIM_ISR_ARROK) == 0);

	// NOTE : Single mode causes the counter to stop.
	// continuous mode is always on in order for the counter to keep counting

	reg->CR |= LPTIM_CR_CNTSTRT;
	reg->ICR = reg->ISR;

	// Enable overflow to adjust timer counter properly.
    reg->IER |= LPTIM_IER_ARRMIE;

	return true;
}

