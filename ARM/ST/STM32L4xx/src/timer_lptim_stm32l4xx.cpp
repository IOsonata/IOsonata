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
#include "timer_lptim_stm32l4xx.h"

#define STM32L4XX_LPTIMER_CLKSRC_PCLK		0
#define STM32L4XX_LPTIMER_CLKSRC_LSI		1
#define STM32L4XX_LPTIMER_CLKSRC_HSI		2
#define STM32L4XX_LPTIMER_CLKSRC_LSE		3

typedef struct {
	LPTIM_TypeDef *pReg;
	LPTimerSTM32L4xx *pInstance;
} LPTIM_DATA;

static LPTIM_DATA s_Stm32l4LptimData[STM32L4XX_LPTIMER_MAXCNT] = {
	{ LPTIM1, 0 },
	{ LPTIM2, 0 }
};

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
bool LPTimerSTM32L4xx::Init(const TIMER_CFG &Cfg)
{
	if (Cfg.DevNo < 0 || Cfg.DevNo >= STM32L4XX_LPTIMER_MAXCNT || Cfg.Freq > 16000000)
	{
		return false;
	}

	vDevNo = Cfg.DevNo;
	s_Stm32l4LptimData[vDevNo].pInstance = this;
	vEvtHandler = Cfg.EvtHandler;

	vpReg = LPTIM1;

	if (vDevNo > 0)
	{
		vpReg = LPTIM2;
	}
	vpReg->CR &= ~LPTIM_CR_ENABLE;

	uint32_t tmp = RCC_CCIPR_LPTIM1SEL_Msk << (Cfg.DevNo << 1);

	RCC->CCIPR &= ~tmp;

	switch (Cfg.ClkSrc)
	{
		case TIMER_CLKSRC_LFRC:
			vBaseFreq = 32000;
			RCC->BDCR &= ~RCC_BDCR_LSEON;
			RCC->BDCR |= RCC_BDCR_LSEBYP;

			RCC->CSR |= RCC_CSR_LSION;

			while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);

			tmp = STM32L4XX_LPTIMER_CLKSRC_LSI << (RCC_CCIPR_LPTIM1SEL_Pos + (Cfg.DevNo << 1));
			break;
		case TIMER_CLKSRC_LFXTAL:
			vBaseFreq = 32768;

			RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
			PWR->CR1 |= PWR_CR1_DBP;
			RCC->BDCR |= RCC_BDCR_BDRST;
			RCC->BDCR &= ~RCC_BDCR_BDRST;

			RCC->BDCR &= ~RCC_BDCR_LSEBYP;

			RCC->BDCR |= RCC_BDCR_LSEON;

			while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0);

			PWR->CR1 &= ~PWR_CR1_DBP;
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;

			tmp = STM32L4XX_LPTIMER_CLKSRC_LSE << (RCC_CCIPR_LPTIM1SEL_Pos + (Cfg.DevNo << 1));
			break;
		case TIMER_CLKSRC_HFRC:
		case TIMER_CLKSRC_HFXTAL:
		default:
			vBaseFreq = 16000000;
			RCC->CR |= RCC_CR_HSION;

			while ((RCC->CR & RCC_CR_HSIRDY) == 0);

			tmp = STM32L4XX_LPTIMER_CLKSRC_HSI << (RCC_CCIPR_LPTIM1SEL_Pos + (Cfg.DevNo << 1));
	}

	RCC->CCIPR |= tmp;

	if (vDevNo == 0)
	{
		RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

		NVIC_ClearPendingIRQ(LPTIM1_IRQn);
		NVIC_SetPriority(LPTIM1_IRQn, Cfg.IntPrio);
		NVIC_EnableIRQ(LPTIM1_IRQn);
	}
	else
	{
		RCC->APB1ENR2 |= RCC_APB1ENR2_LPTIM2EN;

		NVIC_ClearPendingIRQ(LPTIM2_IRQn);
		NVIC_SetPriority(LPTIM2_IRQn, Cfg.IntPrio);
		NVIC_EnableIRQ(LPTIM2_IRQn);
	}

	Frequency(Cfg.Freq);

	vpReg->OR = 0;
	vpReg->CR = LPTIM_CR_ENABLE;

	vpReg->ARR = 0xFFFF;
	vpReg->CMP = 0;

	// Wait for the value to set, otherwise it will cause wrong frequency to set in.
	while ((vpReg->ISR & LPTIM_ISR_ARROK) == 0);

	// NOTE : Single mode causes the counter to stop.
	// continuous mode is always on in order for the counter to keep counting

	vpReg->CR |= LPTIM_CR_CNTSTRT;
	vpReg->ICR = vpReg->ISR;

	// Enable overflow to adjust timer counter properly.
    vpReg->IER |= LPTIM_IER_ARRMIE;

	return true;
}

/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
bool LPTimerSTM32L4xx::Enable()
{
	vpReg->CR = LPTIM_CR_ENABLE;
	//vpReg->CR |= LPTIM_CR_CNTSTRT;
	return true;
}

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 */
void LPTimerSTM32L4xx::Disable()
{
	vpReg->CR = 0;
}

/**
 * @brief   Reset timer.
 */
void LPTimerSTM32L4xx::Reset()
{
	vpReg->ICR = vpReg->ISR;
	vpReg->CNT = 0;
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
uint32_t LPTimerSTM32L4xx::Frequency(uint32_t Freq)
{
	uint32_t div = Freq > 0 ? (vBaseFreq + (Freq >> 1)) / Freq : 1;
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

	vFreq = vBaseFreq >> tmp;

	// Pre-calculate periods for faster timer counter to time conversion use later
    vnsPeriod = (1000000000ULL + ((uint64_t)vFreq >> 1))/ (uint64_t)vFreq;     // Period in nsec

	vpReg->CFGR = (vpReg->CFGR & ~LPTIM_CFGR_PRESC_Msk) | (tmp << LPTIM_CFGR_PRESC_Pos);

	return vFreq;
}

/**
 * @brief	Get maximum available timer trigger event for the timer.
 *
 * @return	count
 */
int LPTimerSTM32L4xx::MaxTimerTrigger()
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
uint64_t LPTimerSTM32L4xx::EnableTimerTrigger(int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
											  TIMER_TRIGCB const Handler, void * const pContext)
{
    if (TrigNo < 0 || TrigNo >= STM32L4XX_LPTIMER_TRIG_MAXCNT)
        return 0;

    uint64_t cc = (nsPeriod + (vnsPeriod >> 1ULL)) / vnsPeriod;

    if (cc <= 0ULL || cc >= 0x10000ULL)
    {
        return 0;
    }

    uint64_t retval = 0;

    vTrigger[TrigNo].Type = Type;
    vCC[TrigNo] = cc & 0xFFFF;

    vTrigger[TrigNo].nsPeriod = vnsPeriod * (uint64_t)cc;
    vTrigger[TrigNo].Handler = Handler;
    vTrigger[TrigNo].pContext = pContext;

    uint32_t count = vpReg->CNT;
    while (count != vpReg->CNT) {
    	count = vpReg->CNT;
    }

    vpReg->CMP = (cc + count) & 0xFFFF;

	while ((vpReg->ISR & LPTIM_ISR_CMPOK) == 0);

    vpReg->IER |= LPTIM_IER_CMPMIE;

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
    return vnsPeriod * cc; // Return real period in nsec
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void LPTimerSTM32L4xx::DisableTimerTrigger(int TrigNo)
{
	vpReg->IER &= ~LPTIM_IER_CMPMIE;
	vpReg->CMP = 0;
}

void LPTimerSTM32L4xx::IRQHandler()
{
	uint32_t flag = vpReg->ISR;
	uint32_t evt = 0;
    uint32_t count = vpReg->CNT;
    while (count != vpReg->CNT) {
    	count = vpReg->CNT;
    }

	if (flag & LPTIM_ISR_ARRM)
	{
        vRollover += 0x10000ULL;
        evt |= TIMER_EVT_COUNTER_OVR;
	}

	if ((flag & LPTIM_ISR_CMPM) && (vpReg->IER & LPTIM_IER_CMPMIE))
	{
		// NOTE : Single mode cause the counter to stop
		// Reset compare and disable interrupt for single mode
		if (vTrigger[0].Type == TIMER_TRIG_TYPE_CONTINUOUS)
		{
			vpReg->CMP = (count + vCC[0]) & 0xFFFF;
		}
		else
		{
			vpReg->CMP = 0;
			vpReg->IER &= ~LPTIM_IER_CMPMIE;
		}
		evt |= TIMER_EVT_TRIGGER(0);
        if (vTrigger[0].Handler)
        {
        	vTrigger[0].Handler(this, 0, vTrigger[0].pContext);
        }
	}

    vLastCount = count;

    if (vEvtHandler)
	{
		vEvtHandler(this, evt);
	}

	vpReg->ICR = flag;
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
int LPTimerSTM32L4xx::FindAvailTimerTrigger(void)
{
	return 0;
}

extern "C" void LPTIM1_IRQHandler()
{
	s_Stm32l4LptimData[0].pInstance->IRQHandler();

	NVIC_ClearPendingIRQ(LPTIM1_IRQn);
}

extern "C" void LPTIM2_IRQHandler()
{
	s_Stm32l4LptimData[1].pInstance->IRQHandler();

	NVIC_ClearPendingIRQ(LPTIM2_IRQn);
}
