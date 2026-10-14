/**-------------------------------------------------------------------------
@file	timer_lptim_stm32.cpp

@brief	STM32 LPTIM timer implementation

		Timer DevNo 0 is LPTIM1, DevNo 1 LPTIM2, on the parts that have them:
		L4 (LPTIM1 and LPTIM2), G4 (LPTIM1), WBA (LPTIM1 and LPTIM2). See
		timer_stm32.h.

		Each LPTIM has a 16 bits counter, counted up to 0xFFFF and extended
		to 64 bits in software: a read lower than the previous one means the
		counter wrapped. ARRM is set when the counter reaches 0xFFFF, a tick
		before it wraps, its interrupt makes sure the counter is read once in
		each period. Compare triggers: one on L4 and G4 (CMP), two on WBA
		(CCR1, CCR2). A continuous trigger is scheduled one period after its
		previous compare value, it does not drift with the interrupt
		latency.

		Two kinds of LPTIM, told apart by the device header:
		- L4 and G4, LPTIM_IER: CFGR and IER written with the timer
		  disabled, ARR and CMP written with the timer enabled, each
		  confirmed by ARROK or CMPOK.
		- WBA, LPTIM_DIER: CFGR written with the timer disabled, DIER, ARR,
		  CCR1 and CCR2 written with the timer enabled, each confirmed by
		  DIEROK, ARROK, CMP1OK or CMP2OK, as the ST HAL does.
		The waits are bounded, a write not confirmed makes TimerInit, Enable
		or EnableTrigger fail. The interrupts are enabled once when the timer
		starts, a disabled trigger only stops its callback. Changing the
		frequency restarts the counter. External trigger is not supported.
		TimerInit refuses an LPTIM whose vector an application handler
		replaced, or that counts without this driver.

		Clock sources:
		- LSI 32 kHz (TIMER_CLKSRC_LFRC), 250 Hz when the LSI prescaler is
		  set (WBA LSI1PREDIV, L4P5 and L4Q5 LSIPREDIV)
		- LSE 32768 Hz (TIMER_CLKSRC_LFXTAL), crystal or, with
		  g_McuOsc.LowPwrOsc OSC_TYPE_TCXO, external clock (bypass)
		- HSI16 16 MHz (TIMER_CLKSRC_HFRC, TIMER_CLKSRC_HFXTAL)
		TIMER_CLKSRC_DEFAULT selects LSI or LSE from g_McuOsc.LowPwrOsc.

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
#include <stdint.h>
#include <stdbool.h>

#include "stm32.h"

#include "coredev/timer.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "timer_stm32.h"

#if ST_LPTIM_CNT > 0

// LPTIMxSEL values
#define ST_LPTIM_CLKSEL_LSI			1U
#define ST_LPTIM_CLKSEL_HSI			2U
#define ST_LPTIM_CLKSEL_LSE			3U
#define ST_LPTIM_CLKSEL_MSK			3U

#define ST_LSI_FREQ					32000U
#define ST_LSE_FREQ					32768U
#define ST_HSI_FREQ					16000000U

// Polls of LSIRDY, HSIRDY and of the LPTIM write done flags. A write is
// done after a few kernel clock cycles, 12 ms at the slowest, 250 Hz LSI.
// SystemCoreClock / 64: at least 60 ms with 4 or more cycles a poll.
#define ST_LPTIM_READY_POLLS		((int)(SystemCoreClock >> 6))
// Polls of LSERDY: SystemCoreClock / 2, at least 2 s with 4 or more cycles a
// poll. The LSE crystal can take up to 2 s to start.
#define ST_LSE_READY_POLLS			((int)(SystemCoreClock >> 1))

#ifdef LPTIM_DIER_ARRMIE
// WBA: interrupt enables in DIER, two compare channels
#define ST_LPTIM_TRIG_MAXCNT		2
#define ST_LPTIM_IE					(LPTIM_DIER_ARRMIE | LPTIM_DIER_CC1IE | LPTIM_DIER_CC2IE)
#define ST_LPTIM_CMPIF				(LPTIM_ISR_CC1IF | LPTIM_ISR_CC2IF)

// Compare write done flag and compare match flag of each trigger
static const uint32_t s_StLptCmpOk[ST_LPTIM_TRIG_MAXCNT] = { LPTIM_ISR_CMP1OK, LPTIM_ISR_CMP2OK };
static const uint32_t s_StLptCmpIf[ST_LPTIM_TRIG_MAXCNT] = { LPTIM_ISR_CC1IF, LPTIM_ISR_CC2IF };

static inline volatile uint32_t *StLptCmpReg(LPTIM_TypeDef *pReg, int TrigNo)
{
	return TrigNo == 0 ? &pReg->CCR1 : &pReg->CCR2;
}
#else
// L4, G4: interrupt enables in IER, one compare
#define ST_LPTIM_TRIG_MAXCNT		1
#define ST_LPTIM_IE					(LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE)
#define ST_LPTIM_CMPIF				LPTIM_ISR_CMPM

static const uint32_t s_StLptCmpOk[ST_LPTIM_TRIG_MAXCNT] = { LPTIM_ISR_CMPOK };
static const uint32_t s_StLptCmpIf[ST_LPTIM_TRIG_MAXCNT] = { LPTIM_ISR_CMPM };

static inline volatile uint32_t *StLptCmpReg(LPTIM_TypeDef *pReg, int TrigNo)
{
	(void)TrigNo;

	return &pReg->CMP;
}
#endif

// Backup domain control register holding the LSE bits, BDCR1 on WBA
#ifdef RCC_BDCR1_LSEON
#define ST_RCC_BDCR					BDCR1
#define ST_RCC_BDCR_LSEON			RCC_BDCR1_LSEON
#define ST_RCC_BDCR_LSERDY			RCC_BDCR1_LSERDY
#define ST_RCC_BDCR_LSEBYP			RCC_BDCR1_LSEBYP
#else
#define ST_RCC_BDCR					BDCR
#define ST_RCC_BDCR_LSEON			RCC_BDCR_LSEON
#define ST_RCC_BDCR_LSERDY			RCC_BDCR_LSERDY
#define ST_RCC_BDCR_LSEBYP			RCC_BDCR_LSEBYP
#endif

#pragma pack(push, 4)

typedef struct {
	LPTIM_TypeDef *pReg;
	IRQn_Type IrqNo;
	volatile uint32_t *pEnReg;		//!< RCC clock enable register
	volatile uint32_t *pRstReg;		//!< RCC reset register
	uint32_t Mask;					//!< Enable bit, also the reset bit
	volatile uint32_t *pSelReg;		//!< RCC kernel clock selection register
	uint32_t SelPos;				//!< LPTIMxSEL position
} StLptInfo_t;

typedef struct {
	TimerDev_t *pTimer;
	uint32_t BaseFreq;				//!< LPTIM kernel clock in Hz
	uint32_t Cfgr;					//!< CFGR value, prescaler
	uint32_t CC[ST_LPTIM_TRIG_MAXCNT];		//!< Compare periods in counts
	uint32_t Next[ST_LPTIM_TRIG_MAXCNT];	//!< Compare values written
	bool bTrigEn[ST_LPTIM_TRIG_MAXCNT];		//!< Trigger callback enabled
	TimerTrig_t Trigger[ST_LPTIM_TRIG_MAXCNT];
} StLptDev_t;

#pragma pack(pop)

static const StLptInfo_t s_StLptInfo[ST_LPTIM_CNT] = {
#ifdef ST_LPTIM1
	{
		.pReg = LPTIM1,
		.IrqNo = LPTIM1_IRQn,
#ifdef RCC_APB7ENR_LPTIM1EN
		// WBA, on APB7
		.pEnReg = &RCC->APB7ENR,
		.pRstReg = &RCC->APB7RSTR,
		.Mask = RCC_APB7ENR_LPTIM1EN,
		.pSelReg = &RCC->CCIPR3,
		.SelPos = RCC_CCIPR3_LPTIM1SEL_Pos,
#else
		.pEnReg = &RCC->APB1ENR1,
		.pRstReg = &RCC->APB1RSTR1,
		.Mask = RCC_APB1ENR1_LPTIM1EN,
		.pSelReg = &RCC->CCIPR,
		.SelPos = RCC_CCIPR_LPTIM1SEL_Pos,
#endif
	},
#endif
#ifdef ST_LPTIM2
	{
		.pReg = LPTIM2,
		.IrqNo = LPTIM2_IRQn,
		.pEnReg = &RCC->APB1ENR2,
		.pRstReg = &RCC->APB1RSTR2,
		.Mask = RCC_APB1ENR2_LPTIM2EN,
#ifdef RCC_CCIPR1_LPTIM2SEL_Pos
		.pSelReg = &RCC->CCIPR1,
		.SelPos = RCC_CCIPR1_LPTIM2SEL_Pos,
#else
		.pSelReg = &RCC->CCIPR,
		.SelPos = RCC_CCIPR_LPTIM2SEL_Pos,
#endif
	},
#endif
};

static StLptDev_t s_StLptDev[ST_LPTIM_CNT];

// DevNo of a TimerDev_t bound to this driver, -1 if it is not
static int StLptDevNo(TimerDev_t * const pTimer)
{
	if (pTimer == nullptr || pTimer->DevNo < 0 || pTimer->DevNo >= ST_LPTIM_CNT ||
		s_StLptDev[pTimer->DevNo].pTimer != pTimer)
	{
		return -1;
	}

	return pTimer->DevNo;
}

static bool StLptWaitFlag(volatile uint32_t *pReg, uint32_t Mask, int Polls)
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

// Register write confirmed by OkFlag: IER/DIER, ARR, CMP or CCRx. The flag
// is cleared first, set again once the value is taken. A second write before
// that gives unpredictable results.
static bool StLptWrite(LPTIM_TypeDef *pReg, volatile uint32_t *pDst, uint32_t Val, uint32_t OkFlag)
{
	pReg->ICR = OkFlag;
	*pDst = Val;

	return StLptWaitFlag(&pReg->ISR, OkFlag, ST_LPTIM_READY_POLLS);
}

static uint32_t StLptCount(LPTIM_TypeDef *pReg)
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
static uint64_t StLptTick(TimerDev_t * const pTimer, uint32_t Cnt)
{
	if (Cnt < pTimer->LastCount)
	{
		pTimer->Rollover += 0x10000ULL;
	}
	pTimer->LastCount = Cnt;

	return pTimer->Rollover + Cnt;
}

// Disabled, prescaler and interrupt enables written, enabled with ARR 0xFFFF
// and the compare values, then counting continuously from 0. Returns false
// if a write is not taken.
static bool StLptRestart(int DevNo)
{
	const StLptInfo_t *info = &s_StLptInfo[DevNo];
	StLptDev_t *dev = &s_StLptDev[DevNo];
	LPTIM_TypeDef *reg = info->pReg;
	bool res;

	reg->CR = 0;
	reg->CFGR = dev->Cfgr;
	dev->pTimer->Rollover = 0;
	dev->pTimer->LastCount = 0;

#ifdef LPTIM_DIER_ARRMIE
	// DIER is written right after ENABLE, as the ST HAL does. The LPTIM is
	// enabled two counter clock cycles after ENABLE is set: a write not taken
	// in that time is made again once DIEROK does not come.
	reg->CR = LPTIM_CR_ENABLE;
	res = StLptWrite(reg, &reg->DIER, ST_LPTIM_IE, LPTIM_ISR_DIEROK) ||
		  StLptWrite(reg, &reg->DIER, ST_LPTIM_IE, LPTIM_ISR_DIEROK);
	res = res && StLptWrite(reg, &reg->ARR, 0xFFFF, LPTIM_ISR_ARROK);
#else
	// IER with the timer disabled. ARR is the first write after ENABLE, made
	// again if ARROK does not come.
	reg->IER = ST_LPTIM_IE;
	reg->CR = LPTIM_CR_ENABLE;
	res = StLptWrite(reg, &reg->ARR, 0xFFFF, LPTIM_ISR_ARROK) ||
		  StLptWrite(reg, &reg->ARR, 0xFFFF, LPTIM_ISR_ARROK);
#endif
	for (int i = 0; res && i < ST_LPTIM_TRIG_MAXCNT; i++)
	{
		// The counter starts from 0
		dev->Next[i] = dev->bTrigEn[i] ? dev->CC[i] : 0;
		res = StLptWrite(reg, StLptCmpReg(reg, i), dev->Next[i], s_StLptCmpOk[i]);
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

static bool StLptEnable(TimerDev_t * const pTimer)
{
	int devno = StLptDevNo(pTimer);

	if (devno < 0)
	{
		return false;
	}

	const StLptInfo_t *info = &s_StLptInfo[devno];

	*info->pEnReg |= info->Mask;
	(void)*info->pEnReg;

	if ((info->pReg->CR & LPTIM_CR_ENABLE) == 0)
	{
		if (StLptRestart(devno) == false)
		{
			return false;
		}
	}

	NVIC_EnableIRQ(info->IrqNo);

	return true;
}

static void StLptStop(int DevNo)
{
	const StLptInfo_t *info = &s_StLptInfo[DevNo];

	NVIC_DisableIRQ(info->IrqNo);
	info->pReg->CR = 0;
	*info->pEnReg &= ~info->Mask;
	NVIC_ClearPendingIRQ(info->IrqNo);
}

static void StLptDisable(TimerDev_t * const pTimer)
{
	int devno = StLptDevNo(pTimer);

	if (devno >= 0)
	{
		StLptStop(devno);
	}
}

static void StLptReset(TimerDev_t * const pTimer)
{
	int devno = StLptDevNo(pTimer);

	if (devno < 0)
	{
		return;
	}

	const StLptInfo_t *info = &s_StLptInfo[devno];

	NVIC_DisableIRQ(info->IrqNo);
	StLptRestart(devno);
	NVIC_ClearPendingIRQ(info->IrqNo);
	NVIC_EnableIRQ(info->IrqNo);
}

// Prescaler of the closest frequency, BaseFreq divided by 1 to 128. Returns
// the PRESC field.
static uint32_t StLptPresc(uint32_t BaseFreq, uint32_t Freq)
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
 * Timer is reset and restarted with the new frequency. Triggers are disabled.
 *
 * @param 	Freq : Frequency in Hz, 0 for the highest
 *
 * @return  Real frequency, 0 if the running timer did not restart
 */
static uint32_t StLptSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	int devno = StLptDevNo(pTimer);

	if (devno < 0)
	{
		return 0;
	}

	StLptDev_t *dev = &s_StLptDev[devno];
	uint32_t presc = StLptPresc(dev->BaseFreq, Freq);

	pTimer->Freq = dev->BaseFreq >> presc;
	pTimer->nsPeriod = (1000000000ULL + ((uint64_t)pTimer->Freq >> 1)) / (uint64_t)pTimer->Freq;

	dev->Cfgr = presc << LPTIM_CFGR_PRESC_Pos;
	for (int i = 0; i < ST_LPTIM_TRIG_MAXCNT; i++)
	{
		dev->bTrigEn[i] = false;
	}

	if (s_StLptInfo[devno].pReg->CR & LPTIM_CR_ENABLE)
	{
		uint32_t state = DisableInterrupt();
		bool res = StLptRestart(devno);

		EnableInterrupt(state);

		if (res == false)
		{
			return 0;
		}
	}

	return pTimer->Freq;
}

static uint64_t StLptGetTickCount(TimerDev_t * const pTimer)
{
	int devno = StLptDevNo(pTimer);

	if (devno < 0)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	uint64_t tick = StLptTick(pTimer, StLptCount(s_StLptInfo[devno].pReg));

	EnableInterrupt(state);

	return tick;
}

static int StLptGetMaxTrigger(TimerDev_t * const pTimer)
{
	return StLptDevNo(pTimer) < 0 ? 0 : ST_LPTIM_TRIG_MAXCNT;
}

static int StLptFindAvailTrigger(TimerDev_t * const pTimer)
{
	int devno = StLptDevNo(pTimer);

	if (devno < 0)
	{
		return -1;
	}

	for (int i = 0; i < ST_LPTIM_TRIG_MAXCNT; i++)
	{
		if (s_StLptDev[devno].bTrigEn[i] == false)
		{
			return i;
		}
	}

	return -1;
}

/**
 * @brief	Enable a compare trigger.
 *
 * @param   TrigNo : 0 (CMP or CCR1) or, on WBA, 1 (CCR2)
 * @param   nsPeriod : Trigger period in nsec, 3 to 65535 counts
 * @param   Type     : Trigger type single shot or continuous
 * @param	Handler	 : Optional trigger callback
 * @param   pContext : Optional pointer passed to the callback
 *
 * @return  real period in nsec, 0 - failed
 */
static uint64_t StLptEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
								   TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
								   void * const pContext)
{
	int devno = StLptDevNo(pTimer);

	if (devno < 0 || TrigNo < 0 || TrigNo >= ST_LPTIM_TRIG_MAXCNT)
	{
		return 0;
	}

	StLptDev_t *dev = &s_StLptDev[devno];
	LPTIM_TypeDef *reg = s_StLptInfo[devno].pReg;
	uint64_t cc = (nsPeriod + (pTimer->nsPeriod >> 1ULL)) / pTimer->nsPeriod;

	// The compare register is written with the timer enabled
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

	dev->Next[TrigNo] = (StLptCount(reg) + dev->CC[TrigNo]) & 0xFFFF;

	bool res = StLptWrite(reg, StLptCmpReg(reg, TrigNo), dev->Next[TrigNo], s_StLptCmpOk[TrigNo]);

	// A match of the old compare value is not this trigger
	reg->ICR = s_StLptCmpIf[TrigNo];
	dev->bTrigEn[TrigNo] = res;

	EnableInterrupt(state);

	return res ? dev->Trigger[TrigNo].nsPeriod : 0;
}

static void StLptDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	int devno = StLptDevNo(pTimer);

	if (devno >= 0 && TrigNo >= 0 && TrigNo < ST_LPTIM_TRIG_MAXCNT)
	{
		s_StLptDev[devno].bTrigEn[TrigNo] = false;
	}
}

static bool StLptEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	return false;
}

static void StLptDisableExtTrigger(TimerDev_t * const pTimer)
{
}

static void StLptIRQHandler(int DevNo)
{
	StLptDev_t *dev = &s_StLptDev[DevNo];
	TimerDev_t *timer = dev->pTimer;
	LPTIM_TypeDef *reg = s_StLptInfo[DevNo].pReg;
	uint32_t flag = reg->ISR;
	uint32_t evt = 0;

	reg->ICR = flag & (LPTIM_ISR_ARRM | ST_LPTIM_CMPIF);

	if (timer == nullptr)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	uint32_t count = StLptCount(reg);

	StLptTick(timer, count);
	EnableInterrupt(state);

	if (flag & LPTIM_ISR_ARRM)
	{
		evt |= TIMER_EVT_COUNTER_OVR;
	}

	for (int i = 0; i < ST_LPTIM_TRIG_MAXCNT; i++)
	{
		if ((flag & s_StLptCmpIf[i]) && dev->bTrigEn[i])
		{
			if (dev->Trigger[i].Type == TIMER_TRIG_TYPE_CONTINUOUS)
			{
				// One period after the last compare value. If the interrupt
				// came too late for it, one period from now.
				uint32_t next = (dev->Next[i] + dev->CC[i]) & 0xFFFF;
				uint32_t ahead = (next - count) & 0xFFFF;

				if (ahead <= 2 || ahead > dev->CC[i])
				{
					next = (count + dev->CC[i]) & 0xFFFF;
				}
				dev->Next[i] = next;
				StLptWrite(reg, StLptCmpReg(reg, i), next, s_StLptCmpOk[i]);
			}
			else
			{
				dev->bTrigEn[i] = false;
			}

			evt |= TIMER_EVT_TRIGGER(i);
			if (dev->Trigger[i].Handler)
			{
				dev->Trigger[i].Handler(timer, i, dev->Trigger[i].pContext);
			}
		}
	}

	if (timer->EvtHandler && evt)
	{
		timer->EvtHandler(timer, evt);
	}
}

// LPTIM vector, from Stm32TimerIRQHandler
void Stm32LptimIRQHandler(int IrqNo)
{
	for (int i = 0; i < ST_LPTIM_CNT; i++)
	{
		if (s_StLptInfo[i].IrqNo == IrqNo)
		{
			StLptIRQHandler(i);
		}
	}
}

// Backup domain write access, for the LSE bits (and LSI1 on WBA)
static void StLptBackupAccess(void)
{
#ifdef PWR_DBPR_DBP
	RCC->AHB4ENR |= RCC_AHB4ENR_PWREN;
	(void)RCC->AHB4ENR;
	PWR->DBPR |= PWR_DBPR_DBP;
#else
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	(void)RCC->APB1ENR1;
	PWR->CR1 |= PWR_CR1_DBP;
#endif
}

// LPTIM kernel clock, LPTIMxSEL value in *pSel. Returns 0 if the oscillator
// does not start.
static uint32_t StLptClock(TIMER_CLKSRC ClkSrc, uint32_t *pSel)
{
	if (ClkSrc == TIMER_CLKSRC_DEFAULT)
	{
		ClkSrc = GetLowFreqOscType() == OSC_TYPE_RC ? TIMER_CLKSRC_LFRC : TIMER_CLKSRC_LFXTAL;
	}

	switch (ClkSrc)
	{
		case TIMER_CLKSRC_LFRC:
#ifdef RCC_BDCR1_LSI1ON
			// WBA: LSI1 is in the backup domain
			if ((RCC->BDCR1 & RCC_BDCR1_LSI1RDY) == 0)
			{
				StLptBackupAccess();
				RCC->BDCR1 |= RCC_BDCR1_LSI1ON;
				if (StLptWaitFlag(&RCC->BDCR1, RCC_BDCR1_LSI1RDY, ST_LPTIM_READY_POLLS) == false)
				{
					return 0;
				}
			}
			*pSel = ST_LPTIM_CLKSEL_LSI;
			return RCC->BDCR1 & RCC_BDCR1_LSI1PREDIV ? ST_LSI_FREQ / 128U : ST_LSI_FREQ;
#else
			if ((RCC->CSR & RCC_CSR_LSIRDY) == 0)
			{
				RCC->CSR |= RCC_CSR_LSION;
				if (StLptWaitFlag(&RCC->CSR, RCC_CSR_LSIRDY, ST_LPTIM_READY_POLLS) == false)
				{
					return 0;
				}
			}
			*pSel = ST_LPTIM_CLKSEL_LSI;
#ifdef RCC_CSR_LSIPREDIV
			return RCC->CSR & RCC_CSR_LSIPREDIV ? ST_LSI_FREQ / 128U : ST_LSI_FREQ;
#else
			return ST_LSI_FREQ;
#endif
#endif

		case TIMER_CLKSRC_LFXTAL:
		{
#ifdef RCC_BDCR1_LSESYSEN
			// WBA: LSESYS is the LSE clock of the peripherals other than the
			// RTC
			uint32_t rdy = RCC_BDCR1_LSERDY | RCC_BDCR1_LSESYSRDY;
			uint32_t on = RCC_BDCR1_LSEON | RCC_BDCR1_LSESYSEN;
#else
			uint32_t rdy = RCC_BDCR_LSERDY;
			uint32_t on = RCC_BDCR_LSEON;
#endif
			if ((RCC->ST_RCC_BDCR & rdy) != rdy)
			{
				uint32_t was = RCC->ST_RCC_BDCR & ST_RCC_BDCR_LSEON;

				StLptBackupAccess();
				if (was == 0)
				{
					// LSEBYP only changes with the LSE off
					if (GetLowFreqOscType() == OSC_TYPE_TCXO)
					{
						RCC->ST_RCC_BDCR |= ST_RCC_BDCR_LSEBYP;
					}
					else
					{
						RCC->ST_RCC_BDCR &= ~ST_RCC_BDCR_LSEBYP;
					}
				}
#ifdef RCC_BDCR_LSESYSDIS
				// L4P5, L4Q5: LSE propagated to the peripherals
				RCC->BDCR &= ~RCC_BDCR_LSESYSDIS;
#endif
				RCC->ST_RCC_BDCR |= on;

				bool ok = StLptWaitFlag(&RCC->ST_RCC_BDCR, ST_RCC_BDCR_LSERDY, ST_LSE_READY_POLLS);
#ifdef RCC_BDCR1_LSESYSEN
				ok = ok && StLptWaitFlag(&RCC->BDCR1, RCC_BDCR1_LSESYSRDY, ST_LSE_READY_POLLS);
#endif
				if (ok == false)
				{
					if (was == 0)
					{
						RCC->ST_RCC_BDCR &= ~on;
					}
					return 0;
				}
			}
			*pSel = ST_LPTIM_CLKSEL_LSE;
			return ST_LSE_FREQ;
		}

		default:
			RCC->CR |= RCC_CR_HSION;
			if (StLptWaitFlag(&RCC->CR, RCC_CR_HSIRDY, ST_LPTIM_READY_POLLS) == false)
			{
				return 0;
			}
			*pSel = ST_LPTIM_CLKSEL_HSI;
			return ST_HSI_FREQ;
	}
}

/**
 * @brief   LPTIM initialization, called by TimerInit.
 *
 * @param	pTimer : Timer device data to initialize
 * @param	pCfg   : Timer configuration data, DevNo 0 to ST_LPTIM_CNT - 1
 *
 * @return	true - Success
 */
bool Stm32LptimInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == nullptr || pCfg == nullptr || pCfg->DevNo < 0 || pCfg->DevNo >= ST_LPTIM_CNT)
	{
		return false;
	}

	int devno = pCfg->DevNo;
	const StLptInfo_t *info = &s_StLptInfo[devno];
	StLptDev_t *dev = &s_StLptDev[devno];

	// Vector replaced by an application handler, or running without this
	// driver: used by other code
	if (Stm32TimerVectorOwned(info->IrqNo) == false ||
		(dev->pTimer == nullptr && (*info->pEnReg & info->Mask) && (info->pReg->CR & LPTIM_CR_ENABLE)))
	{
		return false;
	}

	uint32_t sel = 0;
	uint32_t base = StLptClock(pCfg->ClkSrc, &sel);

	if (base == 0)
	{
		return false;
	}

	NVIC_DisableIRQ(info->IrqNo);

	*info->pSelReg = (*info->pSelReg & ~(ST_LPTIM_CLKSEL_MSK << info->SelPos)) | (sel << info->SelPos);
	*info->pEnReg |= info->Mask;
	(void)*info->pEnReg;
	*info->pRstReg |= info->Mask;
	*info->pRstReg &= ~info->Mask;

	pTimer->DevNo = devno;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->Disable = StLptDisable;
	pTimer->Enable = StLptEnable;
	pTimer->Reset = StLptReset;
	pTimer->GetTickCount = StLptGetTickCount;
	pTimer->SetFrequency = StLptSetFrequency;
	pTimer->GetMaxTrigger = StLptGetMaxTrigger;
	pTimer->FindAvailTrigger = StLptFindAvailTrigger;
	pTimer->DisableTrigger = StLptDisableTrigger;
	pTimer->EnableTrigger = StLptEnableTrigger;
	pTimer->DisableExtTrigger = StLptDisableExtTrigger;
	pTimer->EnableExtTrigger = StLptEnableExtTrigger;

	dev->pTimer = pTimer;
	dev->BaseFreq = base;

	// Prescaler, then the timer started. The reset left it disabled.
	StLptSetFrequency(pTimer, pCfg->Freq);
	if (StLptRestart(devno) == false)
	{
		*info->pEnReg &= ~info->Mask;

		return false;
	}

	NVIC_ClearPendingIRQ(info->IrqNo);
	NVIC_SetPriority(info->IrqNo, pCfg->IntPrio);
	NVIC_EnableIRQ(info->IrqNo);

	return true;
}

int Stm32LptimFind(TimerDev_t * const pTimer)
{
	for (int i = 0; i < ST_LPTIM_CNT; i++)
	{
		if (pTimer != nullptr && s_StLptDev[i].pTimer == pTimer)
		{
			return i;
		}
	}

	return -1;
}

void Stm32LptimRelease(int DevNo)
{
	if (DevNo < 0 || DevNo >= ST_LPTIM_CNT)
	{
		return;
	}

	StLptStop(DevNo);
	for (int i = 0; i < ST_LPTIM_TRIG_MAXCNT; i++)
	{
		s_StLptDev[DevNo].bTrigEn[i] = false;
	}
	s_StLptDev[DevNo].pTimer = nullptr;
}

#endif // ST_LPTIM_CNT > 0
