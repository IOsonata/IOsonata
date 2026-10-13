/**-------------------------------------------------------------------------
@file	timer_lptim_stm32wbaxx.cpp

@brief	Timer implementation of STM32WBA LPTIM1 and LPTIM2

Timer DevNo 0 is LPTIM1 (APB7, SRD domain), DevNo 1 is LPTIM2 (APB1). The
general purpose TIM are not implemented, TimerInit refuses any other DevNo.

Each LPTIM has a 16 bits counter, counted up to 0xFFFF, and two compare
channels used as triggers 0 and 1 (CCR1, CCR2). The count is extended in
software: a read lower than the previous one means the counter wrapped.
ARRM is set when the counter reaches 0xFFFF, a tick before it wraps, its
interrupt makes sure the counter is read once in each period. A continuous
trigger is scheduled from its previous compare value, it does not drift
with the interrupt latency. Clock sources:
- LSI 32 kHz (TIMER_CLKSRC_LFRC), 250 Hz when the LSI1 prescaler is set
- LSE 32768 Hz (TIMER_CLKSRC_LFXTAL), crystal or, with g_McuOsc.LowPwrOsc
  OSC_TYPE_TCXO, external clock (bypass)
- HSI16 16 MHz (TIMER_CLKSRC_HFRC, TIMER_CLKSRC_HFXTAL)
TIMER_CLKSRC_DEFAULT selects LSI or LSE from g_McuOsc.LowPwrOsc.

This LPTIM takes CFGR writes with the timer disabled and DIER, ARR, CCR1 and
CCR2 writes with the timer enabled, each confirmed by its DIEROK, ARROK,
CMP1OK or CMP2OK flag, as the ST HAL does. The ARR match and both compare
interrupts are enabled once when the timer starts, a disabled trigger only
stops its callback. Changing the frequency restarts the counter. External
trigger is not supported.

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

#include "stm32wbaxx.h"

#include "coredev/timer.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"

#define STM32WBA_LPTIM_CNT			2		// LPTIM1, LPTIM2

#define STM32WBA_LPTIM_TRIG_MAXCNT	2		// CCR1, CCR2

// LPTIMxSEL in RCC_CCIPR3 (LPTIM1) and RCC_CCIPR1 (LPTIM2)
#define STM32WBA_LPTIM_CLKSEL_PCLK	0U
#define STM32WBA_LPTIM_CLKSEL_LSI	1U
#define STM32WBA_LPTIM_CLKSEL_HSI	2U
#define STM32WBA_LPTIM_CLKSEL_LSE	3U
#define STM32WBA_LPTIM_CLKSEL_MSK	3U

#define STM32WBA_LSI_FREQ			32000U
#define STM32WBA_LSE_FREQ			32768U
#define STM32WBA_HSI_FREQ			16000000U

// Polls of LSI1RDY, HSIRDY and of the LPTIM write done flags. A write is
// done after a few kernel clock cycles, 12 ms at the slowest, 250 Hz LSI.
// SystemCoreClock / 64: at least 60 ms with 4 or more cycles a poll.
#define STM32WBA_LPTIM_READY_POLLS	((int)(SystemCoreClock >> 6))
// Polls of LSERDY: SystemCoreClock / 2, at least 2 s with 4 or more cycles a
// poll. The LSE crystal can take up to 2 s to start.
#define STM32WBA_LSE_READY_POLLS	((int)(SystemCoreClock >> 1))

// Interrupt enables, written once when the timer starts
#define STM32WBA_LPTIM_DIER			(LPTIM_DIER_ARRMIE | LPTIM_DIER_CC1IE | LPTIM_DIER_CC2IE)

#pragma pack(push, 4)

typedef struct {
	LPTIM_TypeDef *pReg;
	IRQn_Type IrqNo;
	volatile uint32_t *pEnReg;		//!< RCC clock enable register
	volatile uint32_t *pRstReg;		//!< RCC reset register
	uint32_t Mask;					//!< Enable bit, also the reset bit
	volatile uint32_t *pSelReg;		//!< RCC kernel clock selection register
	uint32_t SelPos;				//!< LPTIMxSEL position
	TimerDev_t *pTimer;
	uint32_t BaseFreq;				//!< LPTIM kernel clock in Hz
	uint32_t Cfgr;					//!< CFGR value, prescaler
	uint32_t CC[STM32WBA_LPTIM_TRIG_MAXCNT];		//!< Compare periods in counts
	uint32_t Next[STM32WBA_LPTIM_TRIG_MAXCNT];		//!< Compare values written
	bool bTrigEn[STM32WBA_LPTIM_TRIG_MAXCNT];		//!< Trigger callback enabled
	TimerTrig_t Trigger[STM32WBA_LPTIM_TRIG_MAXCNT];
} Stm32wbaLpTim_t;

#pragma pack(pop)

static Stm32wbaLpTim_t s_Stm32wbaLpTim[STM32WBA_LPTIM_CNT] = {
	{
		.pReg = LPTIM1,
		.IrqNo = LPTIM1_IRQn,
		.pEnReg = &RCC->APB7ENR,
		.pRstReg = &RCC->APB7RSTR,
		.Mask = RCC_APB7ENR_LPTIM1EN,
		.pSelReg = &RCC->CCIPR3,
		.SelPos = RCC_CCIPR3_LPTIM1SEL_Pos,
	},
	{
		.pReg = LPTIM2,
		.IrqNo = LPTIM2_IRQn,
		.pEnReg = &RCC->APB1ENR2,
		.pRstReg = &RCC->APB1RSTR2,
		.Mask = RCC_APB1ENR2_LPTIM2EN,
		.pSelReg = &RCC->CCIPR1,
		.SelPos = RCC_CCIPR1_LPTIM2SEL_Pos,
	},
};

// Compare register, its write done flag and its match flag, by trigger
static const uint32_t s_Stm32wbaCmpOk[STM32WBA_LPTIM_TRIG_MAXCNT] = { LPTIM_ISR_CMP1OK, LPTIM_ISR_CMP2OK };
static const uint32_t s_Stm32wbaCcIf[STM32WBA_LPTIM_TRIG_MAXCNT] = { LPTIM_ISR_CC1IF, LPTIM_ISR_CC2IF };

static inline volatile uint32_t *Stm32wbaLptCcr(LPTIM_TypeDef *pReg, int TrigNo)
{
	return TrigNo == 0 ? &pReg->CCR1 : &pReg->CCR2;
}

static bool Stm32wbaWaitFlag(volatile uint32_t *pReg, uint32_t Mask, int Polls)
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

// DIER, ARR, CCR1 or CCR2 write with the timer enabled. The flag is cleared
// first, set again once the value is taken. A second write before that gives
// unpredictable results.
static bool Stm32wbaLptWrite(LPTIM_TypeDef *pReg, volatile uint32_t *pDst, uint32_t Val, uint32_t OkFlag)
{
	pReg->ICR = OkFlag;
	*pDst = Val;

	return Stm32wbaWaitFlag(&pReg->ISR, OkFlag, STM32WBA_LPTIM_READY_POLLS);
}

static uint32_t Stm32wbaLptCount(LPTIM_TypeDef *pReg)
{
	// The counter runs from another clock, read until two reads agree
	uint32_t cnt = pReg->CNT;

	while (cnt != pReg->CNT)
	{
		cnt = pReg->CNT;
	}

	return cnt;
}

// Extended count of a counter read. Interrupts disabled.
static uint64_t Stm32wbaLptTick(TimerDev_t * const pTimer, uint32_t Cnt)
{
	if (Cnt < pTimer->LastCount)
	{
		pTimer->Rollover += 0x10000ULL;
	}
	pTimer->LastCount = Cnt;

	return pTimer->Rollover + Cnt;
}

// Disabled, prescaler written, enabled with the interrupt enables, ARR
// 0xFFFF and the compare values, then counting continuously from 0. Returns
// false if a write is not taken.
static bool Stm32wbaLptRestart(Stm32wbaLpTim_t *pDev)
{
	LPTIM_TypeDef *reg = pDev->pReg;
	bool res;

	reg->CR = 0;
	reg->CFGR = pDev->Cfgr;
	pDev->pTimer->Rollover = 0;
	pDev->pTimer->LastCount = 0;

	// DIER is written right after ENABLE, as the ST HAL does. The LPTIM is
	// enabled two counter clock cycles after ENABLE is set: a write not taken
	// in that time is made again once DIEROK does not come.
	reg->CR = LPTIM_CR_ENABLE;
	res = Stm32wbaLptWrite(reg, &reg->DIER, STM32WBA_LPTIM_DIER, LPTIM_ISR_DIEROK) ||
		  Stm32wbaLptWrite(reg, &reg->DIER, STM32WBA_LPTIM_DIER, LPTIM_ISR_DIEROK);
	res = res && Stm32wbaLptWrite(reg, &reg->ARR, 0xFFFF, LPTIM_ISR_ARROK);
	for (int i = 0; res && i < STM32WBA_LPTIM_TRIG_MAXCNT; i++)
	{
		// The counter starts from 0
		pDev->Next[i] = pDev->bTrigEn[i] ? pDev->CC[i] : 0;
		res = Stm32wbaLptWrite(reg, Stm32wbaLptCcr(reg, i), pDev->Next[i], s_Stm32wbaCmpOk[i]);
	}
	if (res == false)
	{
		reg->CR = 0;

		return false;
	}
	reg->ICR = reg->ISR;
	reg->CR = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;

	return true;
}

static bool Stm32wbaLptEnable(TimerDev_t * const pTimer)
{
	Stm32wbaLpTim_t *dev = &s_Stm32wbaLpTim[pTimer->DevNo];

	*dev->pEnReg |= dev->Mask;
	(void)*dev->pEnReg;

	if ((dev->pReg->CR & LPTIM_CR_ENABLE) == 0)
	{
		if (Stm32wbaLptRestart(dev) == false)
		{
			return false;
		}
	}

	NVIC_EnableIRQ(dev->IrqNo);

	return true;
}

static void Stm32wbaLptDisable(TimerDev_t * const pTimer)
{
	Stm32wbaLpTim_t *dev = &s_Stm32wbaLpTim[pTimer->DevNo];

	NVIC_DisableIRQ(dev->IrqNo);
	dev->pReg->CR = 0;
	*dev->pEnReg &= ~dev->Mask;
	NVIC_ClearPendingIRQ(dev->IrqNo);
}

static void Stm32wbaLptReset(TimerDev_t * const pTimer)
{
	Stm32wbaLpTim_t *dev = &s_Stm32wbaLpTim[pTimer->DevNo];

	NVIC_DisableIRQ(dev->IrqNo);
	Stm32wbaLptRestart(dev);
	NVIC_ClearPendingIRQ(dev->IrqNo);
	NVIC_EnableIRQ(dev->IrqNo);
}

// Prescaler of the closest frequency, BaseFreq divided by 1 to 128. Returns
// the PRESC field.
static uint32_t Stm32wbaLptPresc(uint32_t BaseFreq, uint32_t Freq)
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
 * @return  Real frequency, 0 if the running timer did not restart
 */
static uint32_t Stm32wbaLptSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	Stm32wbaLpTim_t *dev = &s_Stm32wbaLpTim[pTimer->DevNo];
	uint32_t presc = Stm32wbaLptPresc(dev->BaseFreq, Freq);

	pTimer->Freq = dev->BaseFreq >> presc;
	pTimer->nsPeriod = (1000000000ULL + ((uint64_t)pTimer->Freq >> 1)) / (uint64_t)pTimer->Freq;

	dev->Cfgr = presc << LPTIM_CFGR_PRESC_Pos;
	for (int i = 0; i < STM32WBA_LPTIM_TRIG_MAXCNT; i++)
	{
		dev->bTrigEn[i] = false;
	}

	if (dev->pReg->CR & LPTIM_CR_ENABLE)
	{
		uint32_t state = DisableInterrupt();
		bool res = Stm32wbaLptRestart(dev);

		EnableInterrupt(state);

		if (res == false)
		{
			return 0;
		}
	}

	return pTimer->Freq;
}

static uint64_t Stm32wbaLptGetTickCount(TimerDev_t * const pTimer)
{
	Stm32wbaLpTim_t *dev = &s_Stm32wbaLpTim[pTimer->DevNo];
	uint32_t state = DisableInterrupt();
	uint64_t tick = Stm32wbaLptTick(pTimer, Stm32wbaLptCount(dev->pReg));

	EnableInterrupt(state);

	return tick;
}

static int Stm32wbaLptGetMaxTrigger(TimerDev_t * const pTimer)
{
	return STM32WBA_LPTIM_TRIG_MAXCNT;
}

static int Stm32wbaLptFindAvailTrigger(TimerDev_t * const pTimer)
{
	Stm32wbaLpTim_t *dev = &s_Stm32wbaLpTim[pTimer->DevNo];

	for (int i = 0; i < STM32WBA_LPTIM_TRIG_MAXCNT; i++)
	{
		if (dev->bTrigEn[i] == false)
		{
			return i;
		}
	}

	return -1;
}

/**
 * @brief	Enable a compare trigger.
 *
 * @param   TrigNo : 0 (CCR1) or 1 (CCR2)
 * @param   nsPeriod : Trigger period in nsec, 3 to 65535 counts
 * @param   Type     : Trigger type single shot or continuous
 * @param	Handler	 : Optional trigger callback
 * @param   pContext : Optional pointer passed to the callback
 *
 * @return  real period in nsec, 0 - failed
 */
static uint64_t Stm32wbaLptEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
										 TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
										 void * const pContext)
{
	if (TrigNo < 0 || TrigNo >= STM32WBA_LPTIM_TRIG_MAXCNT)
	{
		return 0;
	}

	Stm32wbaLpTim_t *dev = &s_Stm32wbaLpTim[pTimer->DevNo];
	LPTIM_TypeDef *reg = dev->pReg;
	uint64_t cc = (nsPeriod + (pTimer->nsPeriod >> 1ULL)) / pTimer->nsPeriod;

	if (cc <= 2ULL || cc >= 0x10000ULL || (reg->CR & LPTIM_CR_ENABLE) == 0)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();

	dev->bTrigEn[TrigNo] = false;
	dev->CC[TrigNo] = (uint32_t)cc;
	dev->Trigger[TrigNo].Type = Type;
	dev->Trigger[TrigNo].nsPeriod = pTimer->nsPeriod * cc;
	dev->Trigger[TrigNo].Handler = Handler;
	dev->Trigger[TrigNo].pContext = pContext;

	dev->Next[TrigNo] = (Stm32wbaLptCount(reg) + dev->CC[TrigNo]) & 0xFFFF;

	bool res = Stm32wbaLptWrite(reg, Stm32wbaLptCcr(reg, TrigNo), dev->Next[TrigNo], s_Stm32wbaCmpOk[TrigNo]);

	// A match of the old compare value is not this trigger
	reg->ICR = s_Stm32wbaCcIf[TrigNo];
	dev->bTrigEn[TrigNo] = res;

	EnableInterrupt(state);

	return res ? dev->Trigger[TrigNo].nsPeriod : 0;
}

static void Stm32wbaLptDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	if (TrigNo >= 0 && TrigNo < STM32WBA_LPTIM_TRIG_MAXCNT)
	{
		s_Stm32wbaLpTim[pTimer->DevNo].bTrigEn[TrigNo] = false;
	}
}

static bool Stm32wbaLptEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	return false;
}

static void Stm32wbaLptDisableExtTrigger(TimerDev_t * const pTimer)
{
}

static void Stm32wbaLptIRQHandler(Stm32wbaLpTim_t *pDev)
{
	TimerDev_t *timer = pDev->pTimer;
	LPTIM_TypeDef *reg = pDev->pReg;
	uint32_t flag = reg->ISR;
	uint32_t evt = 0;

	reg->ICR = flag & (LPTIM_ISR_ARRM | LPTIM_ISR_CC1IF | LPTIM_ISR_CC2IF);

	if (timer == nullptr)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	uint32_t count = Stm32wbaLptCount(reg);

	Stm32wbaLptTick(timer, count);
	EnableInterrupt(state);

	if (flag & LPTIM_ISR_ARRM)
	{
		evt |= TIMER_EVT_COUNTER_OVR;
	}

	for (int i = 0; i < STM32WBA_LPTIM_TRIG_MAXCNT; i++)
	{
		if ((flag & s_Stm32wbaCcIf[i]) && pDev->bTrigEn[i])
		{
			if (pDev->Trigger[i].Type == TIMER_TRIG_TYPE_CONTINUOUS)
			{
				// One period after the last compare value. If the interrupt
				// came too late for it, one period from now.
				uint32_t next = (pDev->Next[i] + pDev->CC[i]) & 0xFFFF;
				uint32_t ahead = (next - count) & 0xFFFF;

				if (ahead <= 2 || ahead > pDev->CC[i])
				{
					next = (count + pDev->CC[i]) & 0xFFFF;
				}
				pDev->Next[i] = next;
				Stm32wbaLptWrite(reg, Stm32wbaLptCcr(reg, i), next, s_Stm32wbaCmpOk[i]);
			}
			else
			{
				pDev->bTrigEn[i] = false;
			}

			evt |= TIMER_EVT_TRIGGER(i);
			if (pDev->Trigger[i].Handler)
			{
				pDev->Trigger[i].Handler(timer, i, pDev->Trigger[i].pContext);
			}
		}
	}

	if (timer->EvtHandler && evt)
	{
		timer->EvtHandler(timer, evt);
	}
}

extern "C" void LPTIM1_IRQHandler(void)
{
	Stm32wbaLptIRQHandler(&s_Stm32wbaLpTim[0]);
}

extern "C" void LPTIM2_IRQHandler(void)
{
	Stm32wbaLptIRQHandler(&s_Stm32wbaLpTim[1]);
}

// Backup domain write access, for the LSE and LSI1 bits of RCC_BDCR1
static void Stm32wbaBackupAccess(void)
{
	RCC->AHB4ENR |= RCC_AHB4ENR_PWREN;
	(void)RCC->AHB4ENR;
	PWR->DBPR |= PWR_DBPR_DBP;
}

// LPTIM kernel clock, LPTIMxSEL value in *pSel. Returns 0 if the oscillator
// does not start.
static uint32_t Stm32wbaLptClock(TIMER_CLKSRC ClkSrc, uint32_t *pSel)
{
	if (ClkSrc == TIMER_CLKSRC_DEFAULT)
	{
		ClkSrc = GetLowFreqOscType() == OSC_TYPE_RC ? TIMER_CLKSRC_LFRC : TIMER_CLKSRC_LFXTAL;
	}

	switch (ClkSrc)
	{
		case TIMER_CLKSRC_LFRC:
			if ((RCC->BDCR1 & RCC_BDCR1_LSI1RDY) == 0)
			{
				Stm32wbaBackupAccess();
				RCC->BDCR1 |= RCC_BDCR1_LSI1ON;
				if (Stm32wbaWaitFlag(&RCC->BDCR1, RCC_BDCR1_LSI1RDY, STM32WBA_LPTIM_READY_POLLS) == false)
				{
					return 0;
				}
			}
			*pSel = STM32WBA_LPTIM_CLKSEL_LSI;
			return RCC->BDCR1 & RCC_BDCR1_LSI1PREDIV ? STM32WBA_LSI_FREQ / 128U : STM32WBA_LSI_FREQ;

		case TIMER_CLKSRC_LFXTAL:
			// LSESYS is the LSE clock of the peripherals other than the RTC
			if ((RCC->BDCR1 & (RCC_BDCR1_LSERDY | RCC_BDCR1_LSESYSRDY)) != (RCC_BDCR1_LSERDY | RCC_BDCR1_LSESYSRDY))
			{
				uint32_t on = RCC->BDCR1 & RCC_BDCR1_LSEON;

				Stm32wbaBackupAccess();
				if (on == 0)
				{
					// LSEBYP only changes with the LSE off
					if (GetLowFreqOscType() == OSC_TYPE_TCXO)
					{
						RCC->BDCR1 |= RCC_BDCR1_LSEBYP;
					}
					else
					{
						RCC->BDCR1 &= ~RCC_BDCR1_LSEBYP;
					}
				}
				RCC->BDCR1 |= RCC_BDCR1_LSEON | RCC_BDCR1_LSESYSEN;
				if (Stm32wbaWaitFlag(&RCC->BDCR1, RCC_BDCR1_LSERDY, STM32WBA_LSE_READY_POLLS) == false ||
					Stm32wbaWaitFlag(&RCC->BDCR1, RCC_BDCR1_LSESYSRDY, STM32WBA_LSE_READY_POLLS) == false)
				{
					if (on == 0)
					{
						RCC->BDCR1 &= ~(RCC_BDCR1_LSEON | RCC_BDCR1_LSESYSEN);
					}
					return 0;
				}
			}
			*pSel = STM32WBA_LPTIM_CLKSEL_LSE;
			return STM32WBA_LSE_FREQ;

		default:
			RCC->CR |= RCC_CR_HSION;
			if (Stm32wbaWaitFlag(&RCC->CR, RCC_CR_HSIRDY, STM32WBA_LPTIM_READY_POLLS) == false)
			{
				return 0;
			}
			*pSel = STM32WBA_LPTIM_CLKSEL_HSI;
			return STM32WBA_HSI_FREQ;
	}
}

/**
 * @brief   Timer initialization.
 *
 * @param	pTimer : Timer device data to initialize
 * @param	pCfg   : Timer configuration data, DevNo 0 LPTIM1, 1 LPTIM2
 *
 * @return	true - Success
 */
bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= STM32WBA_LPTIM_CNT)
	{
		return false;
	}

	uint32_t sel = 0;
	uint32_t base = Stm32wbaLptClock(pCfg->ClkSrc, &sel);

	if (base == 0)
	{
		return false;
	}

	Stm32wbaLpTim_t *dev = &s_Stm32wbaLpTim[pCfg->DevNo];

	NVIC_DisableIRQ(dev->IrqNo);

	*dev->pSelReg = (*dev->pSelReg & ~(STM32WBA_LPTIM_CLKSEL_MSK << dev->SelPos)) | (sel << dev->SelPos);
	*dev->pEnReg |= dev->Mask;
	(void)*dev->pEnReg;
	*dev->pRstReg |= dev->Mask;
	*dev->pRstReg &= ~dev->Mask;

	pTimer->DevNo = pCfg->DevNo;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->Disable = Stm32wbaLptDisable;
	pTimer->Enable = Stm32wbaLptEnable;
	pTimer->Reset = Stm32wbaLptReset;
	pTimer->GetTickCount = Stm32wbaLptGetTickCount;
	pTimer->SetFrequency = Stm32wbaLptSetFrequency;
	pTimer->GetMaxTrigger = Stm32wbaLptGetMaxTrigger;
	pTimer->FindAvailTrigger = Stm32wbaLptFindAvailTrigger;
	pTimer->DisableTrigger = Stm32wbaLptDisableTrigger;
	pTimer->EnableTrigger = Stm32wbaLptEnableTrigger;
	pTimer->DisableExtTrigger = Stm32wbaLptDisableExtTrigger;
	pTimer->EnableExtTrigger = Stm32wbaLptEnableExtTrigger;

	dev->pTimer = pTimer;
	dev->BaseFreq = base;

	// Prescaler, then the timer started. The reset left it disabled.
	Stm32wbaLptSetFrequency(pTimer, pCfg->Freq);
	if (Stm32wbaLptRestart(dev) == false)
	{
		*dev->pEnReg &= ~dev->Mask;

		return false;
	}

	NVIC_ClearPendingIRQ(dev->IrqNo);
	NVIC_SetPriority(dev->IrqNo, pCfg->IntPrio);
	NVIC_EnableIRQ(dev->IrqNo);

	return true;
}

int TimerGetLowFreqDevCount(void)
{
	return STM32WBA_LPTIM_CNT;
}

int TimerGetHighFreqDevCount(void)
{
	return 0;
}

int TimerGetHighFreqDevNo(void)
{
	return STM32WBA_LPTIM_CNT;
}
