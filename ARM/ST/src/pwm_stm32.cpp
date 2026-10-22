/**-------------------------------------------------------------------------
@file	pwm_stm32.cpp

@brief	STM32 PWM on the TIM output channels

Generic PWM API (coredev/pwm.h) on the TIM of the STM32 families of the
timer driver: F0, F4, L1, L4, L4+, G4, H7, WBA. DevNo is the timer DevNo of
timer_stm32.h, the timer driver gives the TIM of a DevNo (Stm32TimHwGet):
for example 7 is TIM2 and 6 TIM15 on the L476, 6 is TIM2 on the G474. The
LPTIM and the basic timers (TIM6, TIM7) have no output and are refused, as
is a TIM bound by TimerInit or counting for other code.

Channel Chan 0 to 3 is the output CH1 to CH4 of the TIM, as many as the TIM
has (1, 2 or 4). PinOp of the channel is the alternate function of its pin
for that TIM output (IOPINOP_FUNC0 to IOPINOP_FUNC15 for AF0 to AF15), from
the datasheet of the part. Complementary outputs (CHxN) are not supported.

The counter runs from Init or Enable to Disable. An open channel drives its
pin at its inactive level, PWM mode 1 while started: active while the
counter is below the compare value, polarity from CCxP. Edge mode counts up
to ARR, center mode up and down (center aligned mode 1, TIM1 to TIM5, TIM8,
TIM20). The prescaler of the counter clock is the smallest that fits the
period in the counter, 16 bits or 32 bits (TIM2, TIM5 where the part has a 32
bit counter), for the best duty resolution, and the period count the closest
to the frequency asked: PwmDev_t::Freq is the frequency obtained. The duty
cycle is a percentage of the period, 0 keeps the output inactive, 100 active.
New compare values take effect at the next period (preload), a new frequency
at once.

PWMInit of another PwmDev_t on the DevNo takes the TIM over: the open
channels of the first one are closed and its calls fail from then on. PWMInit
of a PwmDev_t on another DevNo releases its previous TIM: channels closed,
counter stopped, clock off.

Not supported, PWMInit or PWMStart fail: interrupt (bIntEn) and events,
Start with a duration (msDur not 0).

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include "coredev/pwm.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "timer_stm32.h"

#define ST_PWM_MAX_CHAN				4

// Output compare modes, OCxM
#define ST_PWM_OCM_INACTIVE			4		// Force inactive level
#define ST_PWM_OCM_PWM1				6		// PWM mode 1

// Largest compare value of the 16 and 32 bit TIM. The compare value of a 100
// percent duty cycle is above ARR and must fit.
#define ST_PWM_CCR16_MAX			0xFFFFULL
#define ST_PWM_CCR32_MAX			0xFFFFFFFFULL

#pragma pack(push, 4)

typedef struct {
	PwmDev_t *pDev;					//!< Owner, null when the TIM is free
	Stm32TimHw_t Hw;				//!< TIM of the DevNo
	uint32_t Top;					//!< Counts of a period, ARR + 1 in edge mode, ARR in center mode
	bool bCenter;					//!< Center aligned
	bool bStarted;					//!< Outputs in PWM mode
	uint8_t OpenMask;				//!< Open channels
	uint8_t Duty[ST_PWM_MAX_CHAN];	//!< Duty cycle of each channel, percent
	PwmChanCfg_t Chan[ST_PWM_MAX_CHAN];
} StPwmDev_t;

#pragma pack(pop)

// One device data for each timer DevNo after the LPTIM: one for each TIM the
// device header defines
static constexpr int s_StPwmTimCnt = 0
#ifdef TIM1
	+ 1
#endif
#ifdef TIM2
	+ 1
#endif
#ifdef TIM3
	+ 1
#endif
#ifdef TIM4
	+ 1
#endif
#ifdef TIM5
	+ 1
#endif
#ifdef TIM6
	+ 1
#endif
#ifdef TIM7
	+ 1
#endif
#ifdef TIM8
	+ 1
#endif
#ifdef TIM9
	+ 1
#endif
#ifdef TIM10
	+ 1
#endif
#ifdef TIM11
	+ 1
#endif
#ifdef TIM12
	+ 1
#endif
#ifdef TIM13
	+ 1
#endif
#ifdef TIM14
	+ 1
#endif
#ifdef TIM15
	+ 1
#endif
#ifdef TIM16
	+ 1
#endif
#ifdef TIM17
	+ 1
#endif
#ifdef TIM20
	+ 1
#endif
	;

static StPwmDev_t s_StPwmDev[s_StPwmTimCnt];

// Device data of an initialized PwmDev_t, null otherwise
static StPwmDev_t *StPwmDev(PwmDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return nullptr;
	}

	StPwmDev_t *dev = (StPwmDev_t *)pDev->pDevData;

	if (dev < s_StPwmDev || dev >= s_StPwmDev + s_StPwmTimCnt || dev->pDev != pDev)
	{
		return nullptr;
	}

	return dev;
}

static bool StPwm32Bit(TIM_TypeDef *pReg)
{
	// The F030 defines the macro as 0, without the instance
	(void)pReg;

	return IS_TIM_32B_COUNTER_INSTANCE(pReg);
}

// Prescaler and period counts of the closest frequency, the smallest
// prescaler for the best duty resolution
static bool StPwmPlan(StPwmDev_t *pDev, uint32_t Freq, uint32_t *pPsc, uint32_t *pTop, uint32_t *pFreq)
{
	uint64_t clk = pDev->Hw.ClkFreq;

	if (Freq == 0 || clk == 0)
	{
		return false;
	}

	// Largest counts of a period: the largest compare value in edge mode
	// (ARR one below), one less in center mode (ARR equal)
	uint64_t max = (StPwm32Bit(pDev->Hw.pReg) ? ST_PWM_CCR32_MAX : ST_PWM_CCR16_MAX) - (pDev->bCenter ? 1 : 0);
	// Counts of a period before the prescaler, half the clocks in center mode
	// where the counter counts up then down
	uint64_t div = pDev->bCenter ? 2ULL * Freq : (uint64_t)Freq;
	uint64_t n = (clk + div / 2) / div;

	if (n < 2)
	{
		return false;
	}

	uint64_t psc = (n + max - 1) / max;

	if (psc > 65536)
	{
		return false;
	}

	// Closest period count of the prescaler, rounded as n: at most max
	uint64_t top = (clk + div * psc / 2) / (div * psc);

	*pPsc = (uint32_t)psc;
	*pTop = (uint32_t)top;
	*pFreq = (uint32_t)(clk / (psc * top * (pDev->bCenter ? 2 : 1)));

	return true;
}

// Compare value of a duty cycle, above ARR for 100 percent
static uint32_t StPwmCompare(StPwmDev_t *pDev, int Chan)
{
	uint32_t pct = pDev->Duty[Chan];

	if (pct >= 100)
	{
		return pDev->bCenter ? pDev->Top + 1 : pDev->Top;
	}

	return (uint32_t)((uint64_t)pDev->Top * pct / 100U);
}

static void StPwmSetCompare(TIM_TypeDef *pReg, int Chan, uint32_t Val)
{
	switch (Chan)
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

// Output compare mode of a channel, preload on, channel as output (CCxS 0)
static void StPwmOcMode(TIM_TypeDef *pReg, int Chan, uint32_t Mode)
{
	volatile uint32_t *ccmr = Chan < 2 ? &pReg->CCMR1 : &pReg->CCMR2;
	uint32_t shift = (Chan & 1) * 8U;
	uint32_t mask = 0xFFUL << shift;

#ifdef TIM_CCMR1_OC1M_3
	// Bit 3 of OCxM, above the channel bytes
	mask |= TIM_CCMR1_OC1M_3 << shift;
#endif

	*ccmr = (*ccmr & ~mask) | (((Mode << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE) << shift);
}

// Pin of a closed channel: GPIO output at the inactive level
static void StPwmPinIdle(const PwmChanCfg_t &Chan)
{
	if (Chan.Pol == PWM_POL_HIGH)
	{
		IOPinClear(Chan.Port, Chan.Pin);
	}
	else
	{
		IOPinSet(Chan.Port, Chan.Pin);
	}
	IOPinConfig(Chan.Port, Chan.Pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

// The pin goes to GPIO at the inactive level before the output is turned off
static void StPwmClose(StPwmDev_t *pDev, int Chan)
{
	if (!(pDev->OpenMask & (1U << Chan)))
	{
		return;
	}

	TIM_TypeDef *reg = pDev->Hw.pReg;

	StPwmPinIdle(pDev->Chan[Chan]);
	reg->CCER &= ~((TIM_CCER_CC1E | TIM_CCER_CC1P) << (Chan * 4));
	pDev->OpenMask &= ~(1U << Chan);
}

// TIM no longer used by its PwmDev_t: channels closed, counter and clock off
static void StPwmRelease(StPwmDev_t *pDev)
{
	for (int i = 0; i < ST_PWM_MAX_CHAN; i++)
	{
		StPwmClose(pDev, i);
	}
	pDev->Hw.pReg->CR1 &= ~TIM_CR1_CEN;
	*pDev->Hw.pEnReg &= ~pDev->Hw.Mask;
	pDev->pDev = nullptr;
}

bool PWMInit(PwmDev_t *pDev, const PwmCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->bIntEn ||
		(pCfg->Mode != PWM_MODE_EDGE && pCfg->Mode != PWM_MODE_CENTER))
	{
		return false;
	}

	Stm32TimHw_t hw;

	if (Stm32TimHwGet(pCfg->DevNo, &hw) == false || hw.NbChan <= 0 || hw.bTimer)
	{
		return false;
	}

	TIM_TypeDef *reg = hw.pReg;
	StPwmDev_t *dev = &s_StPwmDev[pCfg->DevNo - ST_LPTIM_CNT];
	bool center = pCfg->Mode == PWM_MODE_CENTER;

	if (center && !IS_TIM_COUNTER_MODE_SELECT_INSTANCE(reg))
	{
		return false;
	}

	// Counting without this driver: used by other code
	if (dev->pDev == nullptr && (*hw.pEnReg & hw.Mask) && (reg->CR1 & TIM_CR1_CEN))
	{
		return false;
	}

	StPwmDev_t plan = {};
	uint32_t psc, top, freq;

	plan.Hw = hw;
	plan.bCenter = center;
	if (StPwmPlan(&plan, pCfg->Freq, &psc, &top, &freq) == false)
	{
		return false;
	}

	// Initialized before on another TIM: that TIM is released
	StPwmDev_t *prev = StPwmDev(pDev);

	if (prev != nullptr && prev != dev)
	{
		StPwmRelease(prev);
	}

	// Initialized again, or taken from another PwmDev_t: the pins of the open
	// channels go back to idle
	for (int i = 0; dev->pDev != nullptr && i < ST_PWM_MAX_CHAN; i++)
	{
		StPwmClose(dev, i);
	}

	*hw.pEnReg |= hw.Mask;
	(void)*hw.pEnReg;
	*hw.pRstReg |= hw.Mask;
	*hw.pRstReg &= ~hw.Mask;

	reg->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | (center ? TIM_CR1_CMS_0 : 0);
	reg->PSC = psc - 1;
	reg->ARR = center ? top : top - 1;
#ifdef IS_TIM_BREAK_INSTANCE
	// Main output enable of the TIM with a break function, CCxE then
	// enables each output
	if (IS_TIM_BREAK_INSTANCE(reg))
	{
		reg->BDTR = TIM_BDTR_MOE;
	}
#endif
	reg->EGR = TIM_EGR_UG;
	reg->CR1 |= TIM_CR1_CEN;

	*dev = plan;
	dev->Top = top;
	dev->pDev = pDev;

	pDev->DevNo = pCfg->DevNo;
	pDev->Freq = freq;
	pDev->Mode = pCfg->Mode;
	pDev->pEvtHandler = pCfg->pEvtHandler;
	pDev->pDevData = dev;

	return true;
}

bool PWMEnable(PwmDev_t *pDev)
{
	StPwmDev_t *dev = StPwmDev(pDev);

	if (dev == nullptr)
	{
		return false;
	}

	*dev->Hw.pEnReg |= dev->Hw.Mask;
	(void)*dev->Hw.pEnReg;
	dev->Hw.pReg->CR1 |= TIM_CR1_CEN;

	return true;
}

void PWMDisable(PwmDev_t *pDev)
{
	StPwmDev_t *dev = StPwmDev(pDev);

	if (dev == nullptr)
	{
		return;
	}

	PWMStop(pDev);
	dev->Hw.pReg->CR1 &= ~TIM_CR1_CEN;
}

bool PWMOpenChannel(PwmDev_t *pDev, const PwmChanCfg_t *pChanCfg, int NbChan)
{
	StPwmDev_t *dev = StPwmDev(pDev);

	if (dev == nullptr || pChanCfg == nullptr || NbChan <= 0)
	{
		return false;
	}

	for (int i = 0; i < NbChan; i++)
	{
		const PwmChanCfg_t &c = pChanCfg[i];

		if (c.Chan < 0 || c.Chan >= dev->Hw.NbChan || (c.Pol != PWM_POL_LOW && c.Pol != PWM_POL_HIGH) ||
			c.PinOp < IOPINOP_FUNC0 || c.PinOp > IOPINOP_FUNC15)
		{
			return false;
		}
		for (int j = 0; j < i; j++)
		{
			if (pChanCfg[j].Chan == c.Chan)
			{
				return false;
			}
		}
	}

	TIM_TypeDef *reg = dev->Hw.pReg;

	for (int i = 0; i < NbChan; i++)
	{
		const PwmChanCfg_t &c = pChanCfg[i];
		uint32_t shift = c.Chan * 4;

		StPwmClose(dev, c.Chan);
		dev->Chan[c.Chan] = c;

		// Channel driving its inactive level before the pin goes to the TIM
		StPwmSetCompare(reg, c.Chan, StPwmCompare(dev, c.Chan));
		StPwmOcMode(reg, c.Chan, dev->bStarted ? ST_PWM_OCM_PWM1 : ST_PWM_OCM_INACTIVE);
		reg->CCER = (reg->CCER & ~(TIM_CCER_CC1P << shift)) |
					((c.Pol == PWM_POL_LOW ? TIM_CCER_CC1P : 0) | TIM_CCER_CC1E) << shift;
		IOPinConfig(c.Port, c.Pin, c.PinOp, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
		dev->OpenMask |= 1U << c.Chan;
	}

	return true;
}

void PWMCloseChannel(PwmDev_t *pDev, int Chan)
{
	StPwmDev_t *dev = StPwmDev(pDev);

	if (dev == nullptr || Chan < 0 || Chan >= ST_PWM_MAX_CHAN)
	{
		return;
	}

	StPwmClose(dev, Chan);
}

bool PWMStart(PwmDev_t *pDev, uint32_t msDur)
{
	StPwmDev_t *dev = StPwmDev(pDev);

	if (dev == nullptr || msDur != 0 || PWMEnable(pDev) == false)
	{
		return false;
	}

	for (int i = 0; i < ST_PWM_MAX_CHAN; i++)
	{
		if (dev->OpenMask & (1U << i))
		{
			StPwmOcMode(dev->Hw.pReg, i, ST_PWM_OCM_PWM1);
		}
	}
	dev->bStarted = true;

	return true;
}

void PWMStop(PwmDev_t *pDev)
{
	StPwmDev_t *dev = StPwmDev(pDev);

	if (dev == nullptr)
	{
		return;
	}

	for (int i = 0; i < ST_PWM_MAX_CHAN; i++)
	{
		if (dev->OpenMask & (1U << i))
		{
			StPwmOcMode(dev->Hw.pReg, i, ST_PWM_OCM_INACTIVE);
		}
	}
	dev->bStarted = false;
}

bool PWMSetFrequency(PwmDev_t *pDev, uint32_t Freq)
{
	StPwmDev_t *dev = StPwmDev(pDev);
	uint32_t psc, top, freq;

	if (dev == nullptr || StPwmPlan(dev, Freq, &psc, &top, &freq) == false)
	{
		return false;
	}

	TIM_TypeDef *reg = dev->Hw.pReg;

	dev->Top = top;
	reg->PSC = psc - 1;
	reg->ARR = dev->bCenter ? top : top - 1;
	for (int i = 0; i < ST_PWM_MAX_CHAN; i++)
	{
		if (dev->OpenMask & (1U << i))
		{
			StPwmSetCompare(reg, i, StPwmCompare(dev, i));
		}
	}

	// Loads the prescaler, the period and the compare values now
	reg->EGR = TIM_EGR_UG;
	pDev->Freq = freq;

	return true;
}

bool PWMSetDutyCycle(PwmDev_t *pDev, int Chan, int Percent)
{
	StPwmDev_t *dev = StPwmDev(pDev);

	if (dev == nullptr || Chan < 0 || Chan >= dev->Hw.NbChan)
	{
		return false;
	}

	dev->Duty[Chan] = Percent < 0 ? 0 : Percent > 100 ? 100 : Percent;
	if (dev->OpenMask & (1U << Chan))
	{
		StPwmSetCompare(dev->Hw.pReg, Chan, StPwmCompare(dev, Chan));
	}

	return true;
}
