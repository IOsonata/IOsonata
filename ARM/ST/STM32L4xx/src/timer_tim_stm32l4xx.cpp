/**-------------------------------------------------------------------------
@file	timer_tim_stm32l4xx.cpp

@brief	Timer implementation of STM32L4xx TIM

These timer only have one clock source which is either PCLK1 for TIM2-7 and
PCLK2 for TIM1,8,15-17

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

///
#define STM32L4XX_TIMER_MAXFREQ				48000000

extern STM32L4XX_TimerData_t g_Stm32l4TimerData[STM32L4XX_TIMER_MAXCNT];

/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
static bool Stm32l4TimerEnable(TimerDev_t * const pTimer)
{
	//s_Stm32l4TimData[pTimer->DevNo - STM32L4XX_LPTIMER_MAXCNT].pReg->CR1 = TIM_CR1_ENABLE;

	return false;
}

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 */
static void Stm32l4TimerDisable(TimerDev_t * const pTimer)
{
//	s_Stm32l4TimData[pTimer->DevNo - STM32L4XX_LPTIMER_MAXCNT].pReg->CR = 0;
}

/**
 * @brief   Reset timer.
 */
static void Stm32l4TimerReset(TimerDev_t * const pTimer)
{
	int idx = pTimer->DevNo - STM32L4XX_LPTIM_CNT;
	//s_Stm32l4TimData[idx].pReg->ICR = s_Stm32l4LptimData[idx].pReg->ISR;
	//s_Stm32l4TimData[idx].pReg->CNT = 0;
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
static uint32_t Stm32l4TimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	STM32L4XX_TimerData_t *dev = &g_Stm32l4TimerData[pTimer->DevNo - STM32L4XX_LPTIM_CNT];
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

	//dev->pReg->CFGR = (dev->pReg->CFGR & ~LPTIM_CFGR_PRESC_Msk) | (tmp << LPTIM_CFGR_PRESC_Pos);

	return pTimer->Freq;
}

static uint64_t Stm32l4TimerGetTickCount(TimerDev_t * const pTimer)
{
	STM32L4XX_TimerData_t *dev = &g_Stm32l4TimerData[pTimer->DevNo - STM32L4XX_LPTIM_CNT];

	uint64_t cnt = 0;

	return (uint64_t)cnt + pTimer->Rollover;
}

/**
 * @brief	Get maximum available timer trigger event for the timer.
 *
 * @return	count
 */
int Stm32l4TimerGetMaxTrigger(TimerDev_t * const pTimer)
{
	return STM32L4XX_TIMER_TRIG_MAXCNT;
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
uint64_t Stm32l4TimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
								 TimerTrigEvtHandler_t const Handler, void * const pContext)
{
    if (TrigNo < 0 || TrigNo >= STM32L4XX_TIMER_TRIG_MAXCNT)
        return 0;

    STM32L4XX_TimerData_t *dev = &g_Stm32l4TimerData[pTimer->DevNo - STM32L4XX_LPTIM_CNT];
    uint64_t cc = (nsPeriod + (pTimer->nsPeriod >> 1ULL)) / pTimer->nsPeriod;

    if (cc <= 0ULL || cc >= 0x10000ULL)
    {
        return 0;
    }

    uint64_t retval = 0;

    dev->Trigger[TrigNo].Type = Type;
    dev->CC[TrigNo] = cc & 0xFFFF;

    dev->Trigger[TrigNo].nsPeriod = pTimer->nsPeriod * (uint64_t)cc;
    dev->Trigger[TrigNo].Handler = Handler;
    dev->Trigger[TrigNo].pContext = pContext;

    uint32_t count = 0;

   // dev->pReg->CMP = (cc + count) & 0xFFFF;

	//while ((dev->pReg->ISR & LPTIM_ISR_CMPOK) == 0);

	//dev->pReg->IER |= LPTIM_IER_CMPMIE;

	// NOTE : Single mode causes the counter to stop.
	// continuous mode is always on in order for the counter to keep counting
    // single mode is simulated by enabling or disabling interrupt.
/*
    if (Type == TIMER_TRIG_TYPE_CONTINUOUS)
    {
    	vpReg->CR &= ~LPTIM_CR_SNGSTRT;
    	vpReg->CR |= LPTIM_CR_CNTSTRT;
    }
    else
    {
    	vpReg->CR &= ~LPTIM_CR_CNTSTRT;
    	vpReg->CR |= LPTIM_CR_SNGSTRT;
    }
*/
    return pTimer->nsPeriod * cc; // Return real period in nsec
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Stm32l4TimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	int idx = pTimer->DevNo - STM32L4XX_LPTIM_CNT;
	//s_Stm32l4TimData[idx].pReg->IER &= ~LPTIM_IER_CMPMIE;
	//s_Stm32l4TimData[idx].pReg->CMP = 0;
}

void Stm32l4TimerIRQHandler(TimerDev_t * const pTimer)
{
	/*
	TIM_DATA *dev = &s_Stm32l4TimData[pTimer->DevNo - STM32L4XX_LPTIMER_MAXCNT];
	uint32_t flag = dev->pReg->ISR;
	uint32_t evt = 0;
    uint32_t count = dev->pReg->CNT;
    while (count != dev->pReg->CNT) {
    	count = dev->pReg->CNT;
    }

	if (flag & LPTIM_ISR_ARRM)
	{
        pTimer->Rollover += 0x10000ULL;
        evt |= TIMER_EVT_COUNTER_OVR;
	}

	if ((flag & LPTIM_ISR_CMPM) && (dev->pReg->IER & LPTIM_IER_CMPMIE))
	{
		// NOTE : Single mode cause the counter to stop
		// Reset compare and disable interrupt for single mode
		if (dev->Trigger[0].Type == TIMER_TRIG_TYPE_CONTINUOUS)
		{
			dev->pReg->CMP = (count + dev->CC[0]) & 0xFFFF;
		}
		else
		{
			dev->pReg->CMP = 0;
			dev->pReg->IER &= ~LPTIM_IER_CMPMIE;
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

	dev->pReg->ICR = flag;*/
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
int Stm32l4TimerFindAvailTrigger(TimerDev_t * const pTimer)
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
bool Stm32l4TimInit(STM32L4XX_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg)
{
	TIM_TypeDef *reg = pTimerData->pTimReg;

	pTimerData->pTimer->EvtHandler = pCfg->EvtHandler;
    pTimerData->pTimer->Disable = Stm32l4TimerDisable;
    pTimerData->pTimer->Enable = Stm32l4TimerEnable;
    pTimerData->pTimer->Reset = Stm32l4TimerReset;
    pTimerData->pTimer->GetTickCount = Stm32l4TimerGetTickCount;
    pTimerData->pTimer->SetFrequency = Stm32l4TimerSetFrequency;
    pTimerData->pTimer->GetMaxTrigger = Stm32l4TimerGetMaxTrigger;
    pTimerData->pTimer->FindAvailTrigger = Stm32l4TimerFindAvailTrigger;
    pTimerData->pTimer->DisableTrigger = Stm32l4TimerDisableTrigger;
    pTimerData->pTimer->EnableTrigger = Stm32l4TimerEnableTrigger;

	//reg->CR &= ~LPTIM_CR_ENABLE;

	uint32_t tmp = RCC_CCIPR_LPTIM1SEL_Msk << (pTimerData->pTimer->DevNo << 1);

	RCC->CCIPR &= ~tmp;
/*
	switch (pCfg->ClkSrc)
	{
		case TIMER_CLKSRC_LFRC:
			dev->BaseFreq = 32000;
			RCC->BDCR &= ~RCC_BDCR_LSEON;
			RCC->BDCR |= RCC_BDCR_LSEBYP;

			RCC->CSR |= RCC_CSR_LSION;

			while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);

			tmp = STM32L4XX_LPTIMER_CLKSRC_LSI << (RCC_CCIPR_LPTIM1SEL_Pos + (pCfg->DevNo << 1));
			break;
		case TIMER_CLKSRC_LFXTAL:
			dev->BaseFreq = 32768;

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
			dev->BaseFreq = 16000000;
			RCC->CR |= RCC_CR_HSION;

			while ((RCC->CR & RCC_CR_HSIRDY) == 0);

			tmp = STM32L4XX_LPTIMER_CLKSRC_HSI << (RCC_CCIPR_LPTIM1SEL_Pos + (pCfg->DevNo << 1));
	}

	RCC->CCIPR |= tmp;

	if (pTimer->DevNo == 0)
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

	Stm32l4LptSetFrequency(pTimer, pCfg->Freq);

	dev->pReg->OR = 0;
	dev->pReg->CR = LPTIM_CR_ENABLE;

	dev->pReg->ARR = 0xFFFF;
	dev->pReg->CMP = 0;

	// Wait for the value to set, otherwise it will cause wrong frequency to set in.
	while ((dev->pReg->ISR & LPTIM_ISR_ARROK) == 0);

	// NOTE : Single mode causes the counter to stop.
	// continuous mode is always on in order for the counter to keep counting

	dev->pReg->CR |= LPTIM_CR_CNTSTRT;
	dev->pReg->ICR = dev->pReg->ISR;

	// Enable overflow to adjust timer counter properly.
    dev->pReg->IER |= LPTIM_IER_ARRMIE;
*/
	return true;
}

extern "C" {

void TIM1_BRK_TIM15_IRQHandler(void)
{

}

void TIM1_UP_TIM16_IRQHandler(void)
{

}

void TIM1_TRG_COM_TIM17_IRQHandler(void)
{

}

void TIM1_CC_IRQHandler(void)
{

}

void TIM2_IRQHandler(void)
{

}

void TIM3_IRQHandler(void)
{

}
void TIM4_IRQHandler(void)
{

}
void TIM5_IRQHandler(void)
{

}
void TIM6_IRQHandler(void)
{

}
void TIM7_IRQHandler(void)
{

}
void TIM8_BRK_IRQHandler(void)
{

}

void TIM8_UP_IRQHandler(void)
{

}

void TIM8_TRG_COM_IRQHandler(void)
{

}

void TIM8_CC_IRQHandler(void)
{

}

}

