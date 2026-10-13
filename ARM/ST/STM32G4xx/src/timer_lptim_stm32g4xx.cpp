/**-------------------------------------------------------------------------
@file	timer_lptim_stm32g4xx.cpp

@brief	Timer implementation of STM32G4xx LPTIM1

Timer DevNo 0 is LPTIM1, the only low power timer of the G4. The general
purpose TIM are not implemented, TimerInit refuses any other DevNo.

LPTIM1 has a 16 bits counter, counted up to 0xFFFF and extended in software
with the ARR match interrupt. One compare trigger. Clock sources:
- LSI 32 kHz (TIMER_CLKSRC_LFRC)
- LSE 32768 Hz crystal (TIMER_CLKSRC_LFXTAL)
- HSI16 16 MHz (TIMER_CLKSRC_HFRC, TIMER_CLKSRC_HFXTAL)
TIMER_CLKSRC_DEFAULT selects LSI or LSE from g_McuOsc.LowPwrOsc.

RM0440 allows writes to LPTIM_IER and LPTIM_CFGR only with the timer
disabled. The ARR and compare match interrupts are enabled once at
initialization, a disabled trigger only stops its callback. Changing the
frequency restarts the counter. External trigger is not supported.

@author	Hoang Nguyen Hoan
@date	October 13, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#include <stdint.h>
#include <stdbool.h>

#include "stm32g4xx.h"

#include "coredev/timer.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"

#define STM32G4XX_LPTIM_CNT			1		// LPTIM1

#define STM32G4XX_LPTIM_TRIG_MAXCNT	1

// LPTIM1SEL in RCC_CCIPR
#define STM32G4XX_LPTIM_CLKSEL_PCLK	0U
#define STM32G4XX_LPTIM_CLKSEL_LSI	1U
#define STM32G4XX_LPTIM_CLKSEL_HSI	2U
#define STM32G4XX_LPTIM_CLKSEL_LSE	3U

#define STM32G4XX_LSI_FREQ			32000U
#define STM32G4XX_LSE_FREQ			32768U
#define STM32G4XX_HSI_FREQ			16000000U

// Polls of LSIRDY, HSIRDY and of the LPTIM ARROK and CMPOK flags. ARROK and
// CMPOK take a few LPTIM clock cycles, about 100 us at 32 kHz.
#define STM32G4XX_LPTIM_READY_POLLS	200000
// Polls of LSERDY: SystemCoreClock / 2, at least 2 s with 4 or more cycles a
// poll. The LSE crystal can take up to 2 s to start.
#define STM32G4XX_LSE_READY_POLLS	((int)(SystemCoreClock >> 1))

#pragma pack(push, 4)

typedef struct {
	LPTIM_TypeDef *pReg;
	TimerDev_t *pTimer;
	uint32_t BaseFreq;			//!< LPTIM kernel clock in Hz
	uint32_t Cfgr;				//!< CFGR value, prescaler
	uint32_t CC;				//!< Compare period in counts
	bool bTrigEn;				//!< Trigger callback enabled
	TimerTrig_t Trigger;
} Stm32g4LpTim_t;

#pragma pack(pop)

static Stm32g4LpTim_t s_Stm32g4LpTim[STM32G4XX_LPTIM_CNT] = {
	{ .pReg = LPTIM1, },
};

static bool Stm32g4WaitFlag(volatile uint32_t *pReg, uint32_t Mask, int Polls)
{
	while (Polls-- > 0)
	{
		if (*pReg & Mask)
		{
			return true;
		}
	}

	return false;
}

static uint32_t Stm32g4LptCount(LPTIM_TypeDef *pReg)
{
	// The counter runs from another clock, read until two reads agree
	uint32_t cnt = pReg->CNT;

	while (cnt != pReg->CNT)
	{
		cnt = pReg->CNT;
	}

	return cnt;
}

// Enable with the counter at 0, ARR 0xFFFF and the compare value, then count
// continuously. CFGR and IER must have been written with the timer disabled.
static void Stm32g4LptStart(Stm32g4LpTim_t *pDev, uint32_t Cmp)
{
	LPTIM_TypeDef *reg = pDev->pReg;

	reg->CR = LPTIM_CR_ENABLE;
	reg->ARR = 0xFFFF;
	Stm32g4WaitFlag(&reg->ISR, LPTIM_ISR_ARROK, STM32G4XX_LPTIM_READY_POLLS);
	reg->CMP = Cmp & 0xFFFF;
	Stm32g4WaitFlag(&reg->ISR, LPTIM_ISR_CMPOK, STM32G4XX_LPTIM_READY_POLLS);
	reg->ICR = reg->ISR;
	reg->CR = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;
}

// Disabled, prescaler and interrupt enables written, started again
static void Stm32g4LptRestart(Stm32g4LpTim_t *pDev)
{
	LPTIM_TypeDef *reg = pDev->pReg;

	reg->CR = 0;
	reg->CFGR = pDev->Cfgr;
	reg->IER = LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE;
	pDev->pTimer->Rollover = 0;
	pDev->pTimer->LastCount = 0;
	Stm32g4LptStart(pDev, pDev->bTrigEn ? pDev->CC : 0);
}

static bool Stm32g4LptEnable(TimerDev_t * const pTimer)
{
	Stm32g4LpTim_t *dev = &s_Stm32g4LpTim[pTimer->DevNo];

	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
	(void)RCC->APB1ENR1;

	if ((dev->pReg->CR & LPTIM_CR_ENABLE) == 0)
	{
		Stm32g4LptRestart(dev);
	}

	NVIC_EnableIRQ(LPTIM1_IRQn);

	return true;
}

static void Stm32g4LptDisable(TimerDev_t * const pTimer)
{
	Stm32g4LpTim_t *dev = &s_Stm32g4LpTim[pTimer->DevNo];

	NVIC_DisableIRQ(LPTIM1_IRQn);
	dev->pReg->CR = 0;
	RCC->APB1ENR1 &= ~RCC_APB1ENR1_LPTIM1EN;
	NVIC_ClearPendingIRQ(LPTIM1_IRQn);
}

static void Stm32g4LptReset(TimerDev_t * const pTimer)
{
	Stm32g4LpTim_t *dev = &s_Stm32g4LpTim[pTimer->DevNo];

	NVIC_DisableIRQ(LPTIM1_IRQn);
	Stm32g4LptRestart(dev);
	NVIC_ClearPendingIRQ(LPTIM1_IRQn);
	NVIC_EnableIRQ(LPTIM1_IRQn);
}

// Prescaler of the closest frequency, BaseFreq divided by 1 to 128. Returns
// the PRESC field.
static uint32_t Stm32g4LptPresc(uint32_t BaseFreq, uint32_t Freq)
{
	uint32_t div = Freq > 0 ? (BaseFreq + (Freq >> 1)) / Freq : 1;
	uint32_t presc = 0;

	while (presc < 7 && div >= (2UL << presc))
	{
		presc++;
	}

	return presc;
}

/**
 * @brief	Set timer main frequency.
 *
 * Timer is reset and restarted with the new frequency.
 *
 * @param 	Freq : Frequency in Hz, 0 for the highest
 *
 * @return  Real frequency
 */
static uint32_t Stm32g4LptSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	Stm32g4LpTim_t *dev = &s_Stm32g4LpTim[pTimer->DevNo];
	uint32_t presc = Stm32g4LptPresc(dev->BaseFreq, Freq);

	pTimer->Freq = dev->BaseFreq >> presc;
	pTimer->nsPeriod = (1000000000ULL + ((uint64_t)pTimer->Freq >> 1)) / (uint64_t)pTimer->Freq;

	dev->Cfgr = presc << LPTIM_CFGR_PRESC_Pos;
	dev->bTrigEn = false;

	if (dev->pReg->CR & LPTIM_CR_ENABLE)
	{
		uint32_t state = DisableInterrupt();

		Stm32g4LptRestart(dev);
		EnableInterrupt(state);
	}

	return pTimer->Freq;
}

static uint64_t Stm32g4LptGetTickCount(TimerDev_t * const pTimer)
{
	Stm32g4LpTim_t *dev = &s_Stm32g4LpTim[pTimer->DevNo];
	uint32_t state = DisableInterrupt();
	uint32_t cnt = Stm32g4LptCount(dev->pReg);
	uint64_t roll = pTimer->Rollover;

	// Wrapped, the ARR match interrupt not handled yet
	if ((dev->pReg->ISR & LPTIM_ISR_ARRM) && cnt < 0x8000U)
	{
		roll += 0x10000ULL;
	}
	EnableInterrupt(state);

	return roll + cnt;
}

static int Stm32g4LptGetMaxTrigger(TimerDev_t * const pTimer)
{
	return STM32G4XX_LPTIM_TRIG_MAXCNT;
}

static int Stm32g4LptFindAvailTrigger(TimerDev_t * const pTimer)
{
	return s_Stm32g4LpTim[pTimer->DevNo].bTrigEn ? -1 : 0;
}

/**
 * @brief	Enable the compare trigger.
 *
 * @param   TrigNo : 0
 * @param   nsPeriod : Trigger period in nsec, 3 to 65535 counts
 * @param   Type     : Trigger type single shot or continuous
 * @param	Handler	 : Optional trigger callback
 * @param   pContext : Optional pointer passed to the callback
 *
 * @return  real period in nsec, 0 - failed
 */
static uint64_t Stm32g4LptEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
										TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
										void * const pContext)
{
	if (TrigNo < 0 || TrigNo >= STM32G4XX_LPTIM_TRIG_MAXCNT)
	{
		return 0;
	}

	Stm32g4LpTim_t *dev = &s_Stm32g4LpTim[pTimer->DevNo];
	uint64_t cc = (nsPeriod + (pTimer->nsPeriod >> 1ULL)) / pTimer->nsPeriod;

	if (cc <= 2ULL || cc >= 0x10000ULL)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();

	dev->bTrigEn = false;
	dev->CC = (uint32_t)cc;
	dev->Trigger.Type = Type;
	dev->Trigger.nsPeriod = pTimer->nsPeriod * cc;
	dev->Trigger.Handler = Handler;
	dev->Trigger.pContext = pContext;

	// CMP takes the value a few counts later, CMPOK
	dev->pReg->CMP = (Stm32g4LptCount(dev->pReg) + dev->CC) & 0xFFFF;
	Stm32g4WaitFlag(&dev->pReg->ISR, LPTIM_ISR_CMPOK, STM32G4XX_LPTIM_READY_POLLS);
	dev->pReg->ICR = LPTIM_ICR_CMPOKCF | LPTIM_ICR_CMPMCF;
	dev->bTrigEn = true;

	EnableInterrupt(state);

	return dev->Trigger.nsPeriod;
}

static void Stm32g4LptDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	if (TrigNo >= 0 && TrigNo < STM32G4XX_LPTIM_TRIG_MAXCNT)
	{
		s_Stm32g4LpTim[pTimer->DevNo].bTrigEn = false;
	}
}

static bool Stm32g4LptEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	return false;
}

static void Stm32g4LptDisableExtTrigger(TimerDev_t * const pTimer)
{
}

static void Stm32g4LptIRQHandler(Stm32g4LpTim_t *pDev)
{
	TimerDev_t *timer = pDev->pTimer;
	LPTIM_TypeDef *reg = pDev->pReg;
	uint32_t flag = reg->ISR;
	uint32_t evt = 0;
	uint32_t count = Stm32g4LptCount(reg);

	reg->ICR = flag & (LPTIM_ISR_ARRM | LPTIM_ISR_CMPM);

	if (timer == nullptr)
	{
		return;
	}

	if (flag & LPTIM_ISR_ARRM)
	{
		timer->Rollover += 0x10000ULL;
		evt |= TIMER_EVT_COUNTER_OVR;
	}

	if ((flag & LPTIM_ISR_CMPM) && pDev->bTrigEn)
	{
		if (pDev->Trigger.Type == TIMER_TRIG_TYPE_CONTINUOUS)
		{
			reg->CMP = (count + pDev->CC) & 0xFFFF;
			Stm32g4WaitFlag(&reg->ISR, LPTIM_ISR_CMPOK, STM32G4XX_LPTIM_READY_POLLS);
			reg->ICR = LPTIM_ICR_CMPOKCF;
		}
		else
		{
			pDev->bTrigEn = false;
		}

		evt |= TIMER_EVT_TRIGGER(0);
		if (pDev->Trigger.Handler)
		{
			pDev->Trigger.Handler(timer, 0, pDev->Trigger.pContext);
		}
	}

	timer->LastCount = count;

	if (timer->EvtHandler && evt)
	{
		timer->EvtHandler(timer, evt);
	}
}

extern "C" void LPTIM1_IRQHandler(void)
{
	Stm32g4LptIRQHandler(&s_Stm32g4LpTim[0]);
}

// LPTIM1 kernel clock, LPTIM1SEL value in *pSel. Returns 0 if the oscillator
// does not start.
static uint32_t Stm32g4LptClock(TIMER_CLKSRC ClkSrc, uint32_t *pSel)
{
	if (ClkSrc == TIMER_CLKSRC_DEFAULT)
	{
		ClkSrc = GetLowFreqOscType() == OSC_TYPE_RC ? TIMER_CLKSRC_LFRC : TIMER_CLKSRC_LFXTAL;
	}

	switch (ClkSrc)
	{
		case TIMER_CLKSRC_LFRC:
			RCC->CSR |= RCC_CSR_LSION;
			if (Stm32g4WaitFlag(&RCC->CSR, RCC_CSR_LSIRDY, STM32G4XX_LPTIM_READY_POLLS) == false)
			{
				return 0;
			}
			*pSel = STM32G4XX_LPTIM_CLKSEL_LSI;
			return STM32G4XX_LSI_FREQ;

		case TIMER_CLKSRC_LFXTAL:
			if ((RCC->BDCR & RCC_BDCR_LSERDY) == 0)
			{
				// Backup domain write access for LSEON
				RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
				(void)RCC->APB1ENR1;
				PWR->CR1 |= PWR_CR1_DBP;
				RCC->BDCR &= ~RCC_BDCR_LSEBYP;
				RCC->BDCR |= RCC_BDCR_LSEON;
				if (Stm32g4WaitFlag(&RCC->BDCR, RCC_BDCR_LSERDY, STM32G4XX_LSE_READY_POLLS) == false)
				{
					RCC->BDCR &= ~RCC_BDCR_LSEON;
					return 0;
				}
			}
			*pSel = STM32G4XX_LPTIM_CLKSEL_LSE;
			return STM32G4XX_LSE_FREQ;

		default:
			RCC->CR |= RCC_CR_HSION;
			if (Stm32g4WaitFlag(&RCC->CR, RCC_CR_HSIRDY, STM32G4XX_LPTIM_READY_POLLS) == false)
			{
				return 0;
			}
			*pSel = STM32G4XX_LPTIM_CLKSEL_HSI;
			return STM32G4XX_HSI_FREQ;
	}
}

/**
 * @brief   Timer initialization.
 *
 * @param	pTimer : Timer device data to initialize
 * @param	pCfg   : Timer configuration data, DevNo 0 LPTIM1
 *
 * @return	true - Success
 */
bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= STM32G4XX_LPTIM_CNT)
	{
		return false;
	}

	uint32_t sel = 0;
	uint32_t base = Stm32g4LptClock(pCfg->ClkSrc, &sel);

	if (base == 0)
	{
		return false;
	}

	Stm32g4LpTim_t *dev = &s_Stm32g4LpTim[pCfg->DevNo];

	NVIC_DisableIRQ(LPTIM1_IRQn);

	RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_LPTIM1SEL) | (sel << RCC_CCIPR_LPTIM1SEL_Pos);
	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
	(void)RCC->APB1ENR1;
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST;
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;

	pTimer->DevNo = pCfg->DevNo;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->Disable = Stm32g4LptDisable;
	pTimer->Enable = Stm32g4LptEnable;
	pTimer->Reset = Stm32g4LptReset;
	pTimer->GetTickCount = Stm32g4LptGetTickCount;
	pTimer->SetFrequency = Stm32g4LptSetFrequency;
	pTimer->GetMaxTrigger = Stm32g4LptGetMaxTrigger;
	pTimer->FindAvailTrigger = Stm32g4LptFindAvailTrigger;
	pTimer->DisableTrigger = Stm32g4LptDisableTrigger;
	pTimer->EnableTrigger = Stm32g4LptEnableTrigger;
	pTimer->DisableExtTrigger = Stm32g4LptDisableExtTrigger;
	pTimer->EnableExtTrigger = Stm32g4LptEnableExtTrigger;

	dev->pTimer = pTimer;
	dev->BaseFreq = base;
	dev->bTrigEn = false;

	// Prescaler and interrupt enables with the timer disabled
	Stm32g4LptSetFrequency(pTimer, pCfg->Freq);
	Stm32g4LptRestart(dev);

	NVIC_ClearPendingIRQ(LPTIM1_IRQn);
	NVIC_SetPriority(LPTIM1_IRQn, pCfg->IntPrio);
	NVIC_EnableIRQ(LPTIM1_IRQn);

	return true;
}

int TimerGetLowFreqDevCount(void)
{
	return STM32G4XX_LPTIM_CNT;
}

int TimerGetHighFreqDevCount(void)
{
	return 0;
}

int TimerGetHighFreqDevNo(void)
{
	return STM32G4XX_LPTIM_CNT;
}
