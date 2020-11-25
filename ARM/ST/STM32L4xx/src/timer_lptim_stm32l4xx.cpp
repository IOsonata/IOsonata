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

Copyright (c) 2019, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdbool.h>

#include "stm32l4xx.h"

#include "idelay.h"
#include "timer_stm32l4x.h"

#define STM32L4XX_LPTIMER_CLKSRC_PCLK		0
#define STM32L4XX_LPTIMER_CLKSRC_LSI		1
#define STM32L4XX_LPTIMER_CLKSRC_HSI		2
#define STM32L4XX_LPTIMER_CLKSRC_LSE		3

typedef struct {
	LPTIM_TypeDef *pReg;
	uint32_t BaseFreq;
    uint32_t CC[STM32L4XX_LPTIMER_TRIG_MAXCNT];
    TimerTrig_t Trigger[STM32L4XX_LPTIMER_TRIG_MAXCNT];
    TimerDev_t *pTimer;
} LPTIM_DATA;

static LPTIM_DATA s_Stm32l4LptimData[STM32L4XX_LPTIMER_MAXCNT] = {
	{ LPTIM1, 0 },
	{ LPTIM2, 0 }
};


/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
static bool Stm32l4LptEnable(TimerDev_t * const pTimer)
{
	s_Stm32l4LptimData[pTimer->DevNo].pReg->CR = LPTIM_CR_ENABLE;
	//vpReg->CR |= LPTIM_CR_CNTSTRT;
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
	s_Stm32l4LptimData[pTimer->DevNo].pReg->CR = 0;
}

/**
 * @brief   Reset timer.
 */
static void Stm32l4LptReset(TimerDev_t * const pTimer)
{
	s_Stm32l4LptimData[pTimer->DevNo].pReg->ICR = s_Stm32l4LptimData[pTimer->DevNo].pReg->ISR;
	s_Stm32l4LptimData[pTimer->DevNo].pReg->CNT = 0;
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
	LPTIM_DATA *dev = &s_Stm32l4LptimData[pTimer->DevNo];
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

	dev->pReg->CFGR = (dev->pReg->CFGR & ~LPTIM_CFGR_PRESC_Msk) | (tmp << LPTIM_CFGR_PRESC_Pos);

	return pTimer->Freq;
}

static uint64_t Stm32l4LptGetTickCount(TimerDev_t * const pTimer)
{
	LPTIM_DATA *dev = &s_Stm32l4LptimData[pTimer->DevNo];
    uint32_t cnt = dev->pReg->CNT;
    while (cnt != dev->pReg->CNT) {
    	cnt = dev->pReg->CNT;
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

	LPTIM_DATA *dev = &s_Stm32l4LptimData[pTimer->DevNo];
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

    uint32_t count = dev->pReg->CNT;
    while (count != dev->pReg->CNT) {
    	count = dev->pReg->CNT;
    }

    dev->pReg->CMP = (cc + count) & 0xFFFF;

	while ((dev->pReg->ISR & LPTIM_ISR_CMPOK) == 0);

	dev->pReg->IER |= LPTIM_IER_CMPMIE;

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
void Stm32l4LptDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	s_Stm32l4LptimData[pTimer->DevNo].pReg->IER &= ~LPTIM_IER_CMPMIE;
	s_Stm32l4LptimData[pTimer->DevNo].pReg->CMP = 0;
}

void Stm32l4LptIRQHandler(TimerDev_t * const pTimer)
{
	LPTIM_DATA *dev = &s_Stm32l4LptimData[pTimer->DevNo];
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

	dev->pReg->ICR = flag;
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
//	s_Stm32l4LptimData[0].pInstance->IRQHandler();
	Stm32l4LptIRQHandler(s_Stm32l4LptimData[0].pTimer);

	NVIC_ClearPendingIRQ(LPTIM1_IRQn);
}

extern "C" void LPTIM2_IRQHandler()
{
//	s_Stm32l4LptimData[1].pInstance->IRQHandler();
	Stm32l4LptIRQHandler(s_Stm32l4LptimData[1].pTimer);

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
bool Stm32l4LPTimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pCfg->DevNo < 0 || pCfg->DevNo >= STM32L4XX_LPTIMER_MAXCNT || pCfg->Freq > 16000000)
	{
		return false;
	}

	LPTIM_DATA *dev = &s_Stm32l4LptimData[pCfg->DevNo];

	pTimer->DevNo = pCfg->DevNo;
	LPTIM_TypeDef *reg = s_Stm32l4LptimData[pTimer->DevNo].pReg;
//	s_Stm32l4LptimData[pTimer->DevNo].pInstance = this;
	pTimer->EvtHandler = pCfg->EvtHandler;
    pTimer->Disable = Stm32l4LptDisable;
    pTimer->Enable = Stm32l4LptEnable;
    pTimer->Reset = Stm32l4LptReset;
    pTimer->GetTickCount = Stm32l4LptGetTickCount;
    pTimer->SetFrequency = Stm32l4LptSetFrequency;
    pTimer->GetMaxTrigger = Stm32l4LptGetMaxTrigger;
    pTimer->FindAvailTrigger = Stm32l4LptFindAvailTrigger;
    pTimer->DisableTrigger = Stm32l4LptDisableTrigger;
    pTimer->EnableTrigger = Stm32l4LptEnableTrigger;

	reg->CR &= ~LPTIM_CR_ENABLE;

	uint32_t tmp = RCC_CCIPR_LPTIM1SEL_Msk << (pTimer->DevNo << 1);

	RCC->CCIPR &= ~tmp;

	switch (pCfg->ClkSrc)
	{
		case TIMER_CLKSRC_LFRC:
			s_Stm32l4LptimData[pTimer->DevNo].BaseFreq = 32000;
			RCC->BDCR &= ~RCC_BDCR_LSEON;
			RCC->BDCR |= RCC_BDCR_LSEBYP;

			RCC->CSR |= RCC_CSR_LSION;

			while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);

			tmp = STM32L4XX_LPTIMER_CLKSRC_LSI << (RCC_CCIPR_LPTIM1SEL_Pos + (pCfg->DevNo << 1));
			break;
		case TIMER_CLKSRC_LFXTAL:
			s_Stm32l4LptimData[pTimer->DevNo].BaseFreq = 32768;

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
			s_Stm32l4LptimData[pTimer->DevNo].BaseFreq = 16000000;
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

	return true;
}

