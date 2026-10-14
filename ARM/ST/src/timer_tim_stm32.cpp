/**-------------------------------------------------------------------------
@file	timer_tim_stm32.cpp

@brief	STM32 general purpose timer (TIM) implementation

		Timer DevNo from TimerGetHighFreqDevNo, see timer_stm32.h. One
		driver for the basic, general purpose and advanced TIM of F0, F4,
		L4, G4 and WBA. The registers used are the same on every TIM; what
		differs is the vector of each timer and the RCC register holding its
		clock enable, mapped below from the CMSIS device header.

		Each TIM counts 0 to 0xFFFF (TIM2 and TIM5 too), extended to 64 bits
		in software with the update interrupt. Triggers are the compare
		channels: 1 on TIM10, TIM11, TIM13, TIM14, TIM16 and TIM17, 2 on
		TIM9, TIM12 and TIM15, 4 on TIM1 to TIM5, TIM8 and TIM20. A trigger
		deadline is a 64 bits count, a compare match before it is ignored,
		so a period can be longer than the counter. A continuous trigger
		keeps its phase, periods missed by a late interrupt are counted as
		one. The basic timers TIM6 and TIM7 have no compare channel: their
		one trigger sets the counter period.

		The timer clock is the APB timer clock: PCLK, twice PCLK when the
		APB prescaler is not 1. The frequency is the closest of that clock
		divided by 1 to 65536. TIMER_CLKSRC_DEFAULT only.

		TimerInit refuses a timer whose vector an application handler
		replaced, or that counts without this driver. Disable stops the
		counter and its interrupts, Enable starts them again with the count
		and the triggers kept. A vector shared by two timers, TIM1 update
		and TIM16 on L4 and G4 for example, services both. External trigger
		is not supported.

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

#define ST_TIM_TRIG_MAXCNT			4

// RCC register holding the APB1 clock enables, split in two on L4, G4 and
// WBA. The reset bit of a peripheral has the same position as its enable
// bit.
#if defined(RCC_APB1ENR1_PWREN) || defined(RCC_APB1ENR1_TIM2EN)
#define ST_RCC_APB1ENR			APB1ENR1
#define ST_RCC_APB1RSTR			APB1RSTR1
#define ST_RCC_APB1EN(Inst)		RCC_APB1ENR1_##Inst##EN
#elif defined(RCC_APB1ENR_PWREN)
#define ST_RCC_APB1ENR			APB1ENR
#define ST_RCC_APB1RSTR			APB1RSTR
#define ST_RCC_APB1EN(Inst)		RCC_APB1ENR_##Inst##EN
#else
#error "timer_tim_stm32.cpp: RCC APB1 register names not mapped"
#endif

// SystemPeriphClockGet index of the APB2 timers. Parts with a single APB
// prescaler have one PCLK.
#if defined(RCC_CFGR_PPRE2) || defined(RCC_CFGR2_PPRE2)
#define ST_PCLK_APB2			1
#else
#define ST_PCLK_APB2			0
#endif

// Vector of each timer, ST_TIMx_IRQn. The advanced timers TIM1, TIM8 and
// TIM20 have their compare interrupt on another vector, TIMx_CC_IRQn. TIM2
// to TIM5 have their own vector on every part. The vector file points these
// vectors to Stm32TimerVector.
#if defined(STM32F0)
// TIM6 is TIM6_DAC on the parts with a DAC, the device header maps the name
#define ST_TIM1_IRQn			TIM1_BRK_UP_TRG_COM_IRQn
#define ST_TIM6_IRQn			TIM6_IRQn
#define ST_TIM7_IRQn			TIM7_IRQn
#define ST_TIM14_IRQn			TIM14_IRQn
#define ST_TIM15_IRQn			TIM15_IRQn
#define ST_TIM16_IRQn			TIM16_IRQn
#define ST_TIM17_IRQn			TIM17_IRQn
#elif defined(STM32F4)
#ifdef TIM10
#define ST_TIM1_IRQn			TIM1_UP_TIM10_IRQn
#else
#define ST_TIM1_IRQn			TIM1_UP_IRQn
#endif
#ifdef DAC
#define ST_TIM6_IRQn			TIM6_DAC_IRQn
#else
#define ST_TIM6_IRQn			TIM6_IRQn
#endif
#define ST_TIM7_IRQn			TIM7_IRQn
#define ST_TIM8_IRQn			TIM8_UP_TIM13_IRQn
#define ST_TIM9_IRQn			TIM1_BRK_TIM9_IRQn
#define ST_TIM10_IRQn			TIM1_UP_TIM10_IRQn
#define ST_TIM11_IRQn			TIM1_TRG_COM_TIM11_IRQn
#define ST_TIM12_IRQn			TIM8_BRK_TIM12_IRQn
#define ST_TIM13_IRQn			TIM8_UP_TIM13_IRQn
#define ST_TIM14_IRQn			TIM8_TRG_COM_TIM14_IRQn
#elif defined(STM32L4) || defined(STM32G4)
#define ST_TIM1_IRQn			TIM1_UP_TIM16_IRQn
#ifdef STM32L4
// TIM6 is TIM6_DAC on the parts with a DAC, the device header maps the name
#define ST_TIM6_IRQn			TIM6_IRQn
#define ST_TIM7_IRQn			TIM7_IRQn
#else
// TIM7 is TIM7 on the parts without DAC3, the device header maps the name
#define ST_TIM6_IRQn			TIM6_DAC_IRQn
#define ST_TIM7_IRQn			TIM7_DAC_IRQn
#endif
#define ST_TIM8_IRQn			TIM8_UP_IRQn
#define ST_TIM15_IRQn			TIM1_BRK_TIM15_IRQn
#define ST_TIM16_IRQn			TIM1_UP_TIM16_IRQn
#define ST_TIM17_IRQn			TIM1_TRG_COM_TIM17_IRQn
#define ST_TIM20_IRQn			TIM20_UP_IRQn
#elif defined(STM32WBA)
#define ST_TIM1_IRQn			TIM1_UP_IRQn
#define ST_TIM16_IRQn			TIM16_IRQn
#define ST_TIM17_IRQn			TIM17_IRQn
#else
#error "timer_tim_stm32.cpp: TIM vectors not mapped for this family"
#endif

#pragma pack(push, 4)

typedef struct {
	TimerTrig_t Info;
	uint64_t Deadline;				//!< Count of the next event
	uint32_t Ticks;					//!< Period in counts
	bool bActive;
} StTimTrig_t;

typedef struct {
	TIM_TypeDef *pReg;
	IRQn_Type IrqNo;				//!< Update interrupt, also compare except
									//!< on the advanced timers
	IRQn_Type CcIrqNo;				//!< Compare interrupt
	volatile uint32_t *pEnReg;		//!< RCC clock enable register
	volatile uint32_t *pRstReg;		//!< RCC reset register
	uint32_t Mask;					//!< Enable bit, also the reset bit
	int PclkIdx;					//!< SystemPeriphClockGet index
	int NbChan;						//!< Compare channels, 0 on basic timers
	StTimTrig_t *pTrig;				//!< NbChan triggers, 1 on basic timers
} StTimInfo_t;

typedef struct {
	TimerDev_t *pTimer;
	uint32_t Epoch;					//!< Changed by each counter reset
	uint32_t Cycle;					//!< Counter period, ARR + 1
	uint32_t Dier;					//!< Interrupt enables while counting
} StTimDev_t;

#pragma pack(pop)

// Trigger storage of each timer
static struct {
#ifdef TIM6
	StTimTrig_t Tim6[1];
#endif
#ifdef TIM7
	StTimTrig_t Tim7[1];
#endif
#ifdef TIM10
	StTimTrig_t Tim10[1];
#endif
#ifdef TIM11
	StTimTrig_t Tim11[1];
#endif
#ifdef TIM13
	StTimTrig_t Tim13[1];
#endif
#ifdef TIM14
	StTimTrig_t Tim14[1];
#endif
#ifdef TIM16
	StTimTrig_t Tim16[1];
#endif
#ifdef TIM17
	StTimTrig_t Tim17[1];
#endif
#ifdef TIM9
	StTimTrig_t Tim9[2];
#endif
#ifdef TIM12
	StTimTrig_t Tim12[2];
#endif
#ifdef TIM15
	StTimTrig_t Tim15[2];
#endif
#ifdef TIM2
	StTimTrig_t Tim2[4];
#endif
#ifdef TIM3
	StTimTrig_t Tim3[4];
#endif
#ifdef TIM4
	StTimTrig_t Tim4[4];
#endif
#ifdef TIM5
	StTimTrig_t Tim5[4];
#endif
#ifdef TIM1
	StTimTrig_t Tim1[4];
#endif
#ifdef TIM8
	StTimTrig_t Tim8[4];
#endif
#ifdef TIM20
	StTimTrig_t Tim20[4];
#endif
} s_StTimTrig;

// Virtual order: basic, one channel, two channels, four channels, advanced
static const StTimInfo_t s_StTimInfo[] = {
#ifdef TIM6
	{
		.pReg = TIM6,
		.IrqNo = ST_TIM6_IRQn,
		.CcIrqNo = ST_TIM6_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM6),
		.PclkIdx = 0,
		.NbChan = 0,
		.pTrig = s_StTimTrig.Tim6,
	},
#endif
#ifdef TIM7
	{
		.pReg = TIM7,
		.IrqNo = ST_TIM7_IRQn,
		.CcIrqNo = ST_TIM7_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM7),
		.PclkIdx = 0,
		.NbChan = 0,
		.pTrig = s_StTimTrig.Tim7,
	},
#endif
#ifdef TIM10
	{
		.pReg = TIM10,
		.IrqNo = ST_TIM10_IRQn,
		.CcIrqNo = ST_TIM10_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM10EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 1,
		.pTrig = s_StTimTrig.Tim10,
	},
#endif
#ifdef TIM11
	{
		.pReg = TIM11,
		.IrqNo = ST_TIM11_IRQn,
		.CcIrqNo = ST_TIM11_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM11EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 1,
		.pTrig = s_StTimTrig.Tim11,
	},
#endif
#ifdef TIM13
	{
		.pReg = TIM13,
		.IrqNo = ST_TIM13_IRQn,
		.CcIrqNo = ST_TIM13_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM13),
		.PclkIdx = 0,
		.NbChan = 1,
		.pTrig = s_StTimTrig.Tim13,
	},
#endif
#ifdef TIM14
	{
		.pReg = TIM14,
		.IrqNo = ST_TIM14_IRQn,
		.CcIrqNo = ST_TIM14_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM14),
		.PclkIdx = 0,
		.NbChan = 1,
		.pTrig = s_StTimTrig.Tim14,
	},
#endif
#ifdef TIM16
	{
		.pReg = TIM16,
		.IrqNo = ST_TIM16_IRQn,
		.CcIrqNo = ST_TIM16_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM16EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 1,
		.pTrig = s_StTimTrig.Tim16,
	},
#endif
#ifdef TIM17
	{
		.pReg = TIM17,
		.IrqNo = ST_TIM17_IRQn,
		.CcIrqNo = ST_TIM17_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM17EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 1,
		.pTrig = s_StTimTrig.Tim17,
	},
#endif
#ifdef TIM9
	{
		.pReg = TIM9,
		.IrqNo = ST_TIM9_IRQn,
		.CcIrqNo = ST_TIM9_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM9EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 2,
		.pTrig = s_StTimTrig.Tim9,
	},
#endif
#ifdef TIM12
	{
		.pReg = TIM12,
		.IrqNo = ST_TIM12_IRQn,
		.CcIrqNo = ST_TIM12_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM12),
		.PclkIdx = 0,
		.NbChan = 2,
		.pTrig = s_StTimTrig.Tim12,
	},
#endif
#ifdef TIM15
	{
		.pReg = TIM15,
		.IrqNo = ST_TIM15_IRQn,
		.CcIrqNo = ST_TIM15_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM15EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 2,
		.pTrig = s_StTimTrig.Tim15,
	},
#endif
#ifdef TIM2
	{
		.pReg = TIM2,
		.IrqNo = TIM2_IRQn,
		.CcIrqNo = TIM2_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM2),
		.PclkIdx = 0,
		.NbChan = 4,
		.pTrig = s_StTimTrig.Tim2,
	},
#endif
#ifdef TIM3
	{
		.pReg = TIM3,
		.IrqNo = TIM3_IRQn,
		.CcIrqNo = TIM3_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM3),
		.PclkIdx = 0,
		.NbChan = 4,
		.pTrig = s_StTimTrig.Tim3,
	},
#endif
#ifdef TIM4
	{
		.pReg = TIM4,
		.IrqNo = TIM4_IRQn,
		.CcIrqNo = TIM4_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM4),
		.PclkIdx = 0,
		.NbChan = 4,
		.pTrig = s_StTimTrig.Tim4,
	},
#endif
#ifdef TIM5
	{
		.pReg = TIM5,
		.IrqNo = TIM5_IRQn,
		.CcIrqNo = TIM5_IRQn,
		.pEnReg = &RCC->ST_RCC_APB1ENR,
		.pRstReg = &RCC->ST_RCC_APB1RSTR,
		.Mask = ST_RCC_APB1EN(TIM5),
		.PclkIdx = 0,
		.NbChan = 4,
		.pTrig = s_StTimTrig.Tim5,
	},
#endif
#ifdef TIM1
	{
		.pReg = TIM1,
		.IrqNo = ST_TIM1_IRQn,
		.CcIrqNo = TIM1_CC_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM1EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 4,
		.pTrig = s_StTimTrig.Tim1,
	},
#endif
#ifdef TIM8
	{
		.pReg = TIM8,
		.IrqNo = ST_TIM8_IRQn,
		.CcIrqNo = TIM8_CC_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM8EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 4,
		.pTrig = s_StTimTrig.Tim8,
	},
#endif
#ifdef TIM20
	{
		.pReg = TIM20,
		.IrqNo = ST_TIM20_IRQn,
		.CcIrqNo = TIM20_CC_IRQn,
		.pEnReg = &RCC->APB2ENR,
		.pRstReg = &RCC->APB2RSTR,
		.Mask = RCC_APB2ENR_TIM20EN,
		.PclkIdx = ST_PCLK_APB2,
		.NbChan = 4,
		.pTrig = s_StTimTrig.Tim20,
	},
#endif
};

#define ST_TIM_CNT		((int)(sizeof(s_StTimInfo) / sizeof(StTimInfo_t)))

static StTimDev_t s_StTimDev[ST_TIM_CNT];

static void StTimTriggerDisable(TimerDev_t * const pTimer, int TrigNo);

// Table index of a TimerDev_t bound to this driver, -1 if it is not
static int StTimIdx(TimerDev_t * const pTimer)
{
	if (pTimer == nullptr)
	{
		return -1;
	}

	int idx = pTimer->DevNo - ST_LPTIM_CNT;

	if (idx < 0 || idx >= ST_TIM_CNT || s_StTimDev[idx].pTimer != pTimer)
	{
		return -1;
	}

	return idx;
}

static inline int StTimTrigCnt(const StTimInfo_t *pInfo)
{
	return pInfo->NbChan > 0 ? pInfo->NbChan : 1;
}

// Clears status flags. SR bits are rc_w0: flags set after the read are kept.
static void StTimClearFlags(const StTimInfo_t *pInfo, uint32_t Flags)
{
	pInfo->pReg->SR = ~Flags & ((1UL << (pInfo->NbChan + 1)) - 1);
}

static void StTimCompare(TIM_TypeDef *pReg, int TrigNo, uint32_t Val)
{
	switch (TrigNo)
	{
		case 0:
			pReg->CCR1 = Val;
			break;
		case 1:
			pReg->CCR2 = Val;
			break;
		case 2:
			pReg->CCR3 = Val;
			break;
		case 3:
			pReg->CCR4 = Val;
			break;
	}
}

// Interrupt enables follow the counter: none while it is stopped, the
// pending flags are kept for Enable
static void StTimRun(int Idx, bool bRun)
{
	TIM_TypeDef *reg = s_StTimInfo[Idx].pReg;

	if (bRun)
	{
		reg->DIER = s_StTimDev[Idx].Dier;
		reg->CR1 |= TIM_CR1_CEN;
	}
	else
	{
		reg->CR1 &= ~TIM_CR1_CEN;
		reg->DIER = 0;
	}
}

// Extended count. Interrupts disabled. A wrap between the reads is seen in
// the update flag, the interrupt must be serviced once in each counter
// period.
static uint64_t StTimCount(int Idx)
{
	TIM_TypeDef *reg = s_StTimInfo[Idx].pReg;
	StTimDev_t *dev = &s_StTimDev[Idx];
	uint32_t before = reg->SR & TIM_SR_UIF;
	uint32_t low = reg->CNT & 0xFFFFU;
	uint32_t after = reg->SR & TIM_SR_UIF;

	if (after && !before)
	{
		low = reg->CNT & 0xFFFFU;
	}

	return dev->pTimer->Rollover + low + (after ? (uint64_t)dev->Cycle : 0);
}

// Prescaler of the closest frequency. Timer clock is twice PCLK when the APB
// prescaler is not 1.
static bool StTimPlan(int PclkIdx, uint32_t Freq, uint32_t *pDiv, uint32_t *pFreq)
{
	uint32_t base = SystemPeriphClockGet(PclkIdx);

	if (base < SystemCoreClock)
	{
		base *= 2;
	}
	if (base == 0)
	{
		return false;
	}

	uint32_t div = Freq > 0 ? (uint32_t)(((uint64_t)base + (Freq >> 1)) / Freq) : 1;

	if (div == 0)
	{
		div = 1;
	}
	if (div > 65536)
	{
		div = 65536;
	}
	*pDiv = div;
	*pFreq = base / div;

	return *pFreq != 0;
}

// Period in counts, 4 to UINT32_MAX, and the period it makes in nsec
static bool StTimPeriod(uint32_t Freq, uint64_t nsPeriod, uint32_t *pTicks, uint64_t *pActual)
{
	if (Freq == 0 || nsPeriod == 0 || nsPeriod / 1000000000ULL > UINT32_MAX / Freq)
	{
		return false;
	}

	uint64_t val = (nsPeriod / 1000000000ULL) * Freq +
				   ((nsPeriod % 1000000000ULL) * Freq + 500000000ULL) / 1000000000ULL;

	if (val < 4 || val > UINT32_MAX)
	{
		return false;
	}
	*pTicks = (uint32_t)val;
	*pActual = (val * 1000000000ULL + (Freq >> 1)) / Freq;

	return true;
}

// Compare of a trigger written from the low 16 bits of its deadline.
// Interrupts disabled.
static void StTimArm(int Idx, int TrigNo)
{
	const StTimInfo_t *info = &s_StTimInfo[Idx];
	StTimDev_t *dev = &s_StTimDev[Idx];
	StTimTrig_t *trig = &info->pTrig[TrigNo];
	TIM_TypeDef *reg = info->pReg;
	bool run = reg->CR1 & TIM_CR1_CEN;

	if (info->NbChan == 0)
	{
		return;
	}

	uint32_t bit = TIM_DIER_CC1IE << TrigNo;

	dev->Dier &= ~bit;
	if (run)
	{
		reg->DIER = dev->Dier;
	}
	StTimClearFlags(info, bit);
	StTimCompare(reg, TrigNo, (uint32_t)trig->Deadline & 0xFFFFU);
	dev->Dier |= bit;
	if (run)
	{
		reg->DIER = dev->Dier;

		// A short deadline may pass while CCR is being written. It is
		// serviced by the interrupt now, not a counter period later.
		if (StTimCount(Idx) >= trig->Deadline)
		{
			NVIC_SetPendingIRQ(info->IrqNo);
		}
	}
}

// Basic timer counter period, the trigger period or 65536. Interrupts
// disabled.
static void StTimBasicCycle(int Idx, uint32_t Cycle)
{
	TIM_TypeDef *reg = s_StTimInfo[Idx].pReg;
	StTimDev_t *dev = &s_StTimDev[Idx];
	bool run = reg->CR1 & TIM_CR1_CEN;

	reg->CR1 = TIM_CR1_URS;
	(void)reg->CR1;

	uint64_t count = StTimCount(Idx);

	dev->Cycle = Cycle;
	reg->ARR = Cycle - 1;
	reg->EGR = TIM_EGR_UG;
	reg->CNT = 0;
	reg->SR = 0;
	dev->pTimer->Rollover = count;
	NVIC_ClearPendingIRQ(s_StTimInfo[Idx].IrqNo);
	if (run)
	{
		reg->CR1 |= TIM_CR1_CEN;
	}
}

// Counter from 0, prescaler loaded, triggers one period from now. Leaves the
// counter stopped. Interrupts disabled.
static void StTimResetCounter(int Idx)
{
	const StTimInfo_t *info = &s_StTimInfo[Idx];
	StTimDev_t *dev = &s_StTimDev[Idx];
	TIM_TypeDef *reg = info->pReg;

	reg->CR1 = TIM_CR1_URS;
	reg->DIER = 0;
	if (info->NbChan == 0)
	{
		dev->Cycle = info->pTrig[0].bActive ? info->pTrig[0].Ticks : 65536;
		reg->ARR = dev->Cycle - 1;
	}
	// Loads PSC and clears the prescaler counter, URS: no update flag
	reg->EGR = TIM_EGR_UG;
	reg->CNT = 0;
	reg->SR = 0;
	NVIC_ClearPendingIRQ(info->IrqNo);
	NVIC_ClearPendingIRQ(info->CcIrqNo);
	dev->pTimer->Rollover = 0;
	dev->pTimer->LastCount = 0;
	dev->Epoch++;
	for (int i = 0; i < StTimTrigCnt(info); i++)
	{
		if (info->pTrig[i].bActive)
		{
			info->pTrig[i].Deadline = info->pTrig[i].Ticks;
			StTimArm(Idx, i);
		}
	}
}

static void StTimDisable(TimerDev_t * const pTimer)
{
	int idx = StTimIdx(pTimer);

	if (idx < 0)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	// Count, triggers and a pending update are kept
	StTimRun(idx, false);
	EnableInterrupt(state);
}

static bool StTimEnable(TimerDev_t * const pTimer)
{
	int idx = StTimIdx(pTimer);

	if (idx < 0)
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	StTimRun(idx, true);
	NVIC_EnableIRQ(s_StTimInfo[idx].IrqNo);
	NVIC_EnableIRQ(s_StTimInfo[idx].CcIrqNo);
	EnableInterrupt(state);

	return true;
}

static void StTimReset(TimerDev_t * const pTimer)
{
	int idx = StTimIdx(pTimer);

	if (idx < 0)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	bool run = s_StTimInfo[idx].pReg->CR1 & TIM_CR1_CEN;

	StTimResetCounter(idx);
	if (run)
	{
		StTimRun(idx, true);
	}
	EnableInterrupt(state);
}

/**
 * @brief	Set timer main frequency.
 *
 * The counter restarts from 0 and runs. Enabled triggers keep their period,
 * the frequency is refused if one can not.
 *
 * @param 	Freq : Frequency in Hz, 0 for the highest
 *
 * @return  Real frequency, 0 - failed
 */
static uint32_t StTimSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	int idx = StTimIdx(pTimer);

	if (idx < 0)
	{
		return 0;
	}

	const StTimInfo_t *info = &s_StTimInfo[idx];
	uint32_t div, freq;

	if (StTimPlan(info->PclkIdx, Freq, &div, &freq) == false)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	uint32_t ticks[ST_TIM_TRIG_MAXCNT] = { 0, };
	uint64_t actual[ST_TIM_TRIG_MAXCNT] = { 0, };

	for (int i = 0; i < StTimTrigCnt(info); i++)
	{
		if (info->pTrig[i].bActive &&
			(StTimPeriod(freq, info->pTrig[i].Info.nsPeriod, &ticks[i], &actual[i]) == false ||
			 (info->NbChan == 0 && ticks[i] > 65536)))
		{
			EnableInterrupt(state);

			return 0;
		}
	}

	info->pReg->CR1 = TIM_CR1_URS;
	info->pReg->PSC = div - 1;
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + (freq >> 1)) / freq;
	for (int i = 0; i < StTimTrigCnt(info); i++)
	{
		if (info->pTrig[i].bActive)
		{
			info->pTrig[i].Ticks = ticks[i];
			info->pTrig[i].Info.nsPeriod = actual[i];
		}
	}
	StTimResetCounter(idx);
	StTimRun(idx, true);
	NVIC_EnableIRQ(info->IrqNo);
	NVIC_EnableIRQ(info->CcIrqNo);
	EnableInterrupt(state);

	return freq;
}

static uint64_t StTimGetTickCount(TimerDev_t * const pTimer)
{
	int idx = StTimIdx(pTimer);

	if (idx < 0)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	uint64_t count = StTimCount(idx);

	pTimer->LastCount = (uint32_t)count;
	EnableInterrupt(state);

	return count;
}

static int StTimGetMaxTrigger(TimerDev_t * const pTimer)
{
	int idx = StTimIdx(pTimer);

	return idx < 0 ? 0 : StTimTrigCnt(&s_StTimInfo[idx]);
}

static int StTimFindAvailTrigger(TimerDev_t * const pTimer)
{
	int idx = StTimIdx(pTimer);

	if (idx < 0)
	{
		return -1;
	}

	const StTimInfo_t *info = &s_StTimInfo[idx];
	uint32_t state = DisableInterrupt();
	int res = -1;

	for (int i = 0; i < StTimTrigCnt(info); i++)
	{
		if (info->pTrig[i].bActive == false)
		{
			res = i;
			break;
		}
	}
	EnableInterrupt(state);

	return res;
}

static void StTimTriggerDisable(TimerDev_t * const pTimer, int TrigNo)
{
	int idx = StTimIdx(pTimer);

	if (idx < 0 || TrigNo < 0 || TrigNo >= StTimTrigCnt(&s_StTimInfo[idx]))
	{
		return;
	}

	const StTimInfo_t *info = &s_StTimInfo[idx];
	StTimDev_t *dev = &s_StTimDev[idx];
	uint32_t state = DisableInterrupt();

	info->pTrig[TrigNo].bActive = false;
	if (info->NbChan == 0)
	{
		StTimBasicCycle(idx, 65536);
	}
	else
	{
		uint32_t bit = TIM_DIER_CC1IE << TrigNo;

		dev->Dier &= ~bit;
		if (info->pReg->CR1 & TIM_CR1_CEN)
		{
			info->pReg->DIER = dev->Dier;
		}
		StTimClearFlags(info, TIM_SR_CC1IF << TrigNo);
	}
	EnableInterrupt(state);
}

/**
 * @brief	Enable a compare trigger.
 *
 * @param   TrigNo : 0 to GetMaxTrigger - 1
 * @param   nsPeriod : Trigger period in nsec, 4 counts or more, 65536 at
 * 					   most on the basic timers
 * @param   Type     : Trigger type single shot or continuous
 * @param	Handler	 : Optional trigger callback
 * @param   pContext : Optional pointer passed to the callback
 *
 * @return  real period in nsec, 0 - failed
 */
static uint64_t StTimTriggerEnable(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
								   TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
								   void * const pContext)
{
	int idx = StTimIdx(pTimer);

	if (idx < 0 || TrigNo < 0 || TrigNo >= StTimTrigCnt(&s_StTimInfo[idx]) ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}

	const StTimInfo_t *info = &s_StTimInfo[idx];
	uint32_t ticks;
	uint64_t actual;

	if (StTimPeriod(pTimer->Freq, nsPeriod, &ticks, &actual) == false ||
		(info->NbChan == 0 && ticks > 65536))
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	StTimTrig_t *trig = &info->pTrig[TrigNo];

	trig->Info.Type = Type;
	trig->Info.nsPeriod = actual;
	trig->Info.Handler = Handler;
	trig->Info.pContext = pContext;
	trig->Ticks = ticks;
	if (info->NbChan == 0)
	{
		StTimBasicCycle(idx, ticks);
		trig->Deadline = pTimer->Rollover + ticks;
	}
	else
	{
		trig->Deadline = StTimCount(idx) + ticks;
	}
	trig->bActive = true;
	StTimArm(idx, TrigNo);
	EnableInterrupt(state);

	return actual;
}

static void StTimDisableExtTrigger(TimerDev_t * const pTimer)
{
}

static bool StTimEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	return false;
}

static void StTimService(int Idx)
{
	const StTimInfo_t *info = &s_StTimInfo[Idx];
	StTimDev_t *dev = &s_StTimDev[Idx];
	TimerDev_t *timer = dev->pTimer;
	TIM_TypeDef *reg = info->pReg;

	if (timer == nullptr || (reg->CR1 & TIM_CR1_CEN) == 0)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	uint32_t flags = reg->SR & reg->DIER;

	StTimClearFlags(info, flags);
	if (flags & TIM_SR_UIF)
	{
		timer->Rollover += dev->Cycle;
	}

	uint32_t epoch = dev->Epoch;

	EnableInterrupt(state);

	if ((flags & TIM_SR_UIF) && timer->EvtHandler)
	{
		timer->EvtHandler(timer, TIMER_EVT_COUNTER_OVR);
	}

	for (int i = 0; i < StTimTrigCnt(info); i++)
	{
		state = DisableInterrupt();

		// A callback reset, reconfigured or stopped the timer
		if (epoch != dev->Epoch || (reg->CR1 & TIM_CR1_CEN) == 0)
		{
			EnableInterrupt(state);

			return;
		}

		StTimTrig_t *trig = &info->pTrig[i];
		uint64_t now = StTimCount(Idx);

		if (trig->bActive == false || now < trig->Deadline)
		{
			EnableInterrupt(state);
			continue;
		}

		TimerTrig_t trinfo = trig->Info;

		if (trinfo.Type == TIMER_TRIG_TYPE_SINGLE)
		{
			StTimTriggerDisable(timer, i);
		}
		else
		{
			// Keeps the phase, missed periods are one event
			trig->Deadline += ((now - trig->Deadline) / trig->Ticks + 1) * trig->Ticks;
			StTimArm(Idx, i);
		}
		EnableInterrupt(state);

		if (trinfo.Handler)
		{
			trinfo.Handler(timer, i, trinfo.pContext);
		}
		if (timer->EvtHandler)
		{
			timer->EvtHandler(timer, TIMER_EVT_TRIGGER(i));
		}
	}
}

// TIM vector, from Stm32TimerIRQHandler. A vector shared by two timers
// services both.
void Stm32TimIRQHandler(int IrqNo)
{
	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		if (s_StTimInfo[i].IrqNo == IrqNo || s_StTimInfo[i].CcIrqNo == IrqNo)
		{
			StTimService(i);
		}
	}
}

// True if a bound timer other than Idx uses the vector
static bool StTimIrqUsed(IRQn_Type IrqNo, int Idx)
{
	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		if (i != Idx && s_StTimDev[i].pTimer != nullptr &&
			(s_StTimInfo[i].IrqNo == IrqNo || s_StTimInfo[i].CcIrqNo == IrqNo))
		{
			return true;
		}
	}

	return false;
}

/**
 * @brief   TIM initialization, called by TimerInit.
 *
 * @param	pTimer : Timer device data to initialize
 * @param	pCfg   : Timer configuration data, DevNo ST_LPTIM_CNT and up
 *
 * @return	true - Success
 */
bool Stm32TimInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == nullptr || pCfg == nullptr || pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT)
	{
		return false;
	}

	int idx = pCfg->DevNo - ST_LPTIM_CNT;

	if (idx < 0 || idx >= ST_TIM_CNT)
	{
		return false;
	}

	const StTimInfo_t *info = &s_StTimInfo[idx];
	StTimDev_t *dev = &s_StTimDev[idx];
	TIM_TypeDef *reg = info->pReg;
	uint32_t div, freq;

	if (StTimPlan(info->PclkIdx, pCfg->Freq, &div, &freq) == false)
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	// Vector replaced by an application handler, or counting without this
	// driver: used by other code
	if (Stm32TimerVectorOwned(info->IrqNo) == false || Stm32TimerVectorOwned(info->CcIrqNo) == false ||
		(dev->pTimer == nullptr && (*info->pEnReg & info->Mask) && (reg->CR1 & TIM_CR1_CEN)))
	{
		EnableInterrupt(state);

		return false;
	}

	*info->pEnReg |= info->Mask;
	(void)*info->pEnReg;
	*info->pRstReg |= info->Mask;
	*info->pRstReg &= ~info->Mask;

	reg->CR1 = TIM_CR1_URS;
	reg->PSC = div - 1;
	dev->Cycle = 65536;
	reg->ARR = dev->Cycle - 1;
	dev->Dier = TIM_DIER_UIE;
	for (int i = 0; i < StTimTrigCnt(info); i++)
	{
		info->pTrig[i].bActive = false;
	}

	dev->pTimer = pTimer;
	pTimer->DevNo = pCfg->DevNo;
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + (freq >> 1)) / freq;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Disable = StTimDisable;
	pTimer->Enable = StTimEnable;
	pTimer->Reset = StTimReset;
	pTimer->GetTickCount = StTimGetTickCount;
	pTimer->SetFrequency = StTimSetFrequency;
	pTimer->GetMaxTrigger = StTimGetMaxTrigger;
	pTimer->FindAvailTrigger = StTimFindAvailTrigger;
	pTimer->DisableTrigger = StTimTriggerDisable;
	pTimer->EnableTrigger = StTimTriggerEnable;
	pTimer->DisableExtTrigger = StTimDisableExtTrigger;
	pTimer->EnableExtTrigger = StTimEnableExtTrigger;

	StTimResetCounter(idx);
	NVIC_SetPriority(info->IrqNo, pCfg->IntPrio);
	NVIC_SetPriority(info->CcIrqNo, pCfg->IntPrio);
	StTimRun(idx, true);
	NVIC_EnableIRQ(info->IrqNo);
	NVIC_EnableIRQ(info->CcIrqNo);
	EnableInterrupt(state);

	return true;
}

int Stm32TimFind(TimerDev_t * const pTimer)
{
	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		if (pTimer != nullptr && s_StTimDev[i].pTimer == pTimer)
		{
			return i + ST_LPTIM_CNT;
		}
	}

	return -1;
}

void Stm32TimRelease(int DevNo)
{
	int idx = DevNo - ST_LPTIM_CNT;

	if (idx < 0 || idx >= ST_TIM_CNT)
	{
		return;
	}

	const StTimInfo_t *info = &s_StTimInfo[idx];
	uint32_t state = DisableInterrupt();

	StTimRun(idx, false);
	for (int i = 0; i < StTimTrigCnt(info); i++)
	{
		info->pTrig[i].bActive = false;
	}
	s_StTimDev[idx].pTimer = nullptr;
	*info->pEnReg &= ~info->Mask;

	// A vector shared with another timer stays enabled
	if (StTimIrqUsed(info->IrqNo, idx) == false)
	{
		NVIC_DisableIRQ(info->IrqNo);
		NVIC_ClearPendingIRQ(info->IrqNo);
	}
	if (StTimIrqUsed(info->CcIrqNo, idx) == false)
	{
		NVIC_DisableIRQ(info->CcIrqNo);
		NVIC_ClearPendingIRQ(info->CcIrqNo);
	}
	EnableInterrupt(state);
}

int Stm32TimCount(void)
{
	return ST_TIM_CNT;
}
