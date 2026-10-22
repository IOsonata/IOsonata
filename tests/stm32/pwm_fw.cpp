/**-------------------------------------------------------------------------
@file	pwm_fw.cpp

@brief	STM32 PWM driver test firmware

Runs the production PWM driver, ARM/ST/src/pwm_stm32.cpp, with the TIM
table of the timer driver (timer_tim_stm32.cpp) in the emulator of
run_pwm.py. Test registers at 0x5FFF0000, word index:

	1	check result, bit 31 set when it failed
	2	end, number of checks
	3	TIM base, records the TIM given by 4, 5, 6 and 9
	4	RCC reset register of the TIM
	5	RCC enable and reset bit of the TIM
	6	TIM features: bit 0 32 bit counter, bit 1 break function (MOE),
		bit 2 center aligned mode, bit 3 OCxM bit 3 in CCMR bits 16 and 24,
		bits 8 - 15 number of channels
	7	GPIO port base
	8	read: number of update events (UG) written
	9	RCC enable register of the TIM
	10	timer clock, Hz
	11	TIM base of the output to measure
	12	channel of the output to measure, 0 - 3
	13	read: period of the output in timer clocks
	14	read: timer clocks at high level in a period, 0xFFFFFFFF when the
		channel does not drive its pin, 0xFFFFFFFE when the counter is stopped
		in PWM mode

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
#include <stddef.h>

#include "stm32.h"
#include "coredev/pwm.h"
#include "coredev/iopincfg.h"
#include "coredev/timer.h"
#include "timer_stm32.h"

// TIM of each kind the test uses
#ifdef TIM2
#define TEST_TIM4CH		TIM2		// Four channels
#else
#define TEST_TIM4CH		TIM3
#endif
#if defined(TIM16)
#define TEST_TIM1CH		TIM16		// One channel, no center mode
#elif defined(TIM11)
#define TEST_TIM1CH		TIM11
#else
#define TEST_TIM1CH		TIM14
#endif
#ifdef TIM3
#define TEST_TIM16C		TIM3		// 16 bit counter, center mode
#else
#define TEST_TIM16C		TIM1
#endif

#define TEST_CLK		80000000U

static volatile uint32_t *const s_Io = (volatile uint32_t *)0x5FFF0000;
static unsigned s_Checks = 0;
static uint32_t s_Pclk = TEST_CLK;
static uint32_t s_Period = 0;		// Period of the last output measured
static uint32_t s_High = 0;			// Its clocks at high level

uint32_t SystemCoreClock = TEST_CLK;
#ifdef IOSONATA_STM32_H7
uint32_t SystemD2Clock = TEST_CLK;
#endif

// Last IOPinConfig and the output of the channel at that time
static int s_CfgCnt = 0;
static int s_CfgPort = -1;
static int s_CfgPin = -1;
static int s_CfgOp = -1;
static IOPINDIR s_CfgDir = IOPINDIR_INPUT;
static uint32_t s_CfgOdr = 0;
static int s_CfgWave[4] = { -9, -9, -9, -9 };
static TIM_TypeDef *s_Tim = nullptr;

extern "C" uint32_t SystemPeriphClockGet(int Idx)
{
	(void)Idx;

	return s_Pclk;
}

// The timer vector function on every vector: the timer driver owns them all
extern "C" void Stm32TimerVector(void)
{
}

#define TEST_V4		Stm32TimerVector, Stm32TimerVector, Stm32TimerVector, Stm32TimerVector
#define TEST_V16	TEST_V4, TEST_V4, TEST_V4, TEST_V4
#define TEST_V64	TEST_V16, TEST_V16, TEST_V16, TEST_V16

extern "C" void (* const __Vectors[256])(void) = { TEST_V64, TEST_V64, TEST_V64, TEST_V64 };

extern "C" void *memset(void *p, int c, size_t n)
{
	volatile unsigned char *d = (volatile unsigned char *)p;

	while (n--)
	{
		*d++ = c;
	}

	return p;
}

extern "C" void *memcpy(void *p, const void *q, size_t n)
{
	volatile unsigned char *d = (volatile unsigned char *)p;
	const volatile unsigned char *s = (const volatile unsigned char *)q;

	while (n--)
	{
		*d++ = *s++;
	}

	return p;
}

static void Check(bool Ok)
{
	s_Io[1] = ++s_Checks | (Ok ? 0 : 0x80000000U);
}

static void TimClock(uint32_t Pclk, uint32_t Core, uint32_t TimClk)
{
	s_Pclk = Pclk;
	SystemCoreClock = Core;
	s_Io[10] = TimClk;
}

// Output of a channel from the register model: per mille of the period at
// high level, -1 not driven, -2 counter stopped in PWM mode. s_Period gets
// the period in timer clocks.
static int Wave(TIM_TypeDef *pReg, int Chan)
{
	s_Io[11] = (uint32_t)pReg;
	s_Io[12] = Chan;
	s_Period = s_Io[13];
	s_High = s_Io[14];

	if (s_High == 0xFFFFFFFFU)
	{
		return -1;
	}
	if (s_High == 0xFFFFFFFEU || s_Period == 0)
	{
		return -2;
	}

	return (int)(((uint64_t)s_High * 1000U + s_Period / 2) / s_Period);
}

// Frequency of the last output measured
static uint32_t WaveFreq(void)
{
	return s_Period ? s_Io[10] / s_Period : 0;
}

static GPIO_TypeDef *Gpio(int Port)
{
	return Port == IOPORTA ? GPIOA : GPIOB;
}

// Channel of the test pins: PA5 CH1, PB1 and PA6 CH2, PA7 CH4
static int PinChan(int Port, int Pin)
{
	if (Port == IOPORTB || Pin == 6)
	{
		return 1;
	}

	return Pin == 5 ? 0 : 3;
}

extern "C" void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	if (Resistor != IOPINRES_NONE || Type != IOPINTYPE_NORMAL)
	{
		return;
	}
	s_CfgCnt++;
	s_CfgPort = PortNo;
	s_CfgPin = PinNo;
	s_CfgOp = PinOp;
	s_CfgDir = Dir;
	s_CfgOdr = Gpio(PortNo)->ODR;
	if (PinOp != IOPINOP_GPIO && s_Tim != nullptr)
	{
		s_CfgWave[PinChan(PortNo, PinNo)] = Wave(s_Tim, PinChan(PortNo, PinNo));
	}
}

// DevNo of a TIM, -1 if the part has no such TIM
static int DevNo(TIM_TypeDef *pReg)
{
	Stm32TimHw_t hw;

	for (int d = 0; Stm32TimHwGet(d, &hw) || d < ST_LPTIM_CNT; d++)
	{
		if (d >= ST_LPTIM_CNT && hw.pReg == pReg)
		{
			return d;
		}
	}

	return -1;
}

static PwmCfg_t Cfg(int Dev, uint32_t Freq, PWM_MODE Mode)
{
	PwmCfg_t cfg = { Dev, Freq, Mode, false, 6, nullptr };

	return cfg;
}

static PwmChanCfg_t Chan(int Ch, PWM_POL Pol, int Port, int Pin)
{
	PwmChanCfg_t c = { Ch, Pol, Port, Pin, IOPINOP_FUNC1 };

	return c;
}

static bool Started(TIM_TypeDef *pReg)
{
	return (pReg->CR1 & TIM_CR1_CEN) != 0;
}

extern "C" void ResetHandler()
{
	extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
	uint32_t *s = &_sidata;

	for (uint32_t *d = &_sdata; d < &_edata;)
	{
		*d++ = *s++;
	}
	for (uint32_t *d = &_sbss; d < &_ebss;)
	{
		*d++ = 0;
	}

	// The TIM and GPIO ports for the model
	Stm32TimHw_t hw;

	for (int d = ST_LPTIM_CNT; Stm32TimHwGet(d, &hw); d++)
	{
		uint32_t flags = (uint32_t)hw.NbChan << 8;

		if (IS_TIM_32B_COUNTER_INSTANCE(hw.pReg))
		{
			flags |= 1;
		}
#ifdef IS_TIM_BREAK_INSTANCE
		if (IS_TIM_BREAK_INSTANCE(hw.pReg))
		{
			flags |= 2;
		}
#endif
		if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE(hw.pReg))
		{
			flags |= 4;
		}
#ifdef TIM_CCMR1_OC1M_3
		flags |= 8;
#endif
		s_Io[4] = (uint32_t)hw.pRstReg;
		s_Io[9] = (uint32_t)hw.pEnReg;
		s_Io[5] = hw.Mask;
		s_Io[6] = flags;
		s_Io[3] = (uint32_t)hw.pReg;
	}
	s_Io[7] = (uint32_t)GPIOA;
	s_Io[7] = (uint32_t)GPIOB;
	TimClock(TEST_CLK, TEST_CLK, TEST_CLK);

	PwmDev_t pwm, other, third;
	TIM_TypeDef *t4 = TEST_TIM4CH;
	TIM_TypeDef *t1 = TEST_TIM1CH;
	int dev4 = DevNo(t4);
	int dev1 = DevNo(t1);
	bool b32 = IS_TIM_32B_COUNTER_INSTANCE(t4);
	PwmCfg_t cfg = Cfg(dev4, 1000, PWM_MODE_EDGE);
	PwmChanCfg_t ch[2] = { Chan(0, PWM_POL_HIGH, IOPORTA, 5), Chan(1, PWM_POL_LOW, IOPORTB, 1) };

	s_Tim = t4;
	Check(dev4 >= ST_LPTIM_CNT && dev1 >= ST_LPTIM_CNT);

	// Nothing works before Init
	pwm.pDevData = nullptr;
	other.pDevData = nullptr;
	third.pDevData = nullptr;
	Check(!PWMOpenChannel(&pwm, ch, 2) && !PWMStart(&pwm, 0) && !PWMSetFrequency(&pwm, 1000) &&
		  !PWMSetDutyCycle(&pwm, 0, 50) && !PWMEnable(&pwm));
	PWMStop(&pwm);
	PWMDisable(&pwm);
	PWMCloseChannel(&pwm, 0);

	// Init refusals
	cfg.bIntEn = true;
	Check(!PWMInit(&pwm, &cfg));
	cfg.bIntEn = false;
	cfg.Mode = (PWM_MODE)5;
	Check(!PWMInit(&pwm, &cfg));
	cfg.Mode = PWM_MODE_EDGE;
	cfg.Freq = 0;
	Check(!PWMInit(&pwm, &cfg));
	cfg.Freq = TEST_CLK;		// One count per period
	Check(!PWMInit(&pwm, &cfg));
	cfg.Freq = 1000;
	cfg.DevNo = 1000;
	Check(!PWMInit(&pwm, &cfg));
	cfg.DevNo = -1;
	Check(!PWMInit(&pwm, &cfg) && !PWMInit(&pwm, nullptr) && !PWMInit(nullptr, &cfg));
#if ST_LPTIM_CNT > 0
	cfg.DevNo = 0;				// LPTIM
	Check(!PWMInit(&pwm, &cfg));
#endif
#ifdef TIM6
	cfg.DevNo = DevNo(TIM6);	// Basic timer
	Check(!PWMInit(&pwm, &cfg));
#endif
	cfg = Cfg(dev1, 1000, PWM_MODE_CENTER);
	Check(!PWMInit(&pwm, &cfg));	// No center mode on this TIM

	// A TIM counting for other code
	Stm32TimHwGet(dev4, &hw);
	*hw.pEnReg |= hw.Mask;
	t4->CR1 = TIM_CR1_CEN;
	cfg = Cfg(dev4, 1000, PWM_MODE_EDGE);
	Check(!PWMInit(&pwm, &cfg));
	t4->CR1 = 0;

	// Edge, 1 kHz from 80 MHz: 80000 counts, 40000 with prescaler 2 on 16 bits
	t4->CCMR2 = 0x5555;			// The RCC reset clears it
	uint32_t ug = s_Io[8];
	Check(PWMInit(&pwm, &cfg) && pwm.DevNo == dev4 && pwm.Freq == 1000 && pwm.Mode == PWM_MODE_EDGE);
	Check(t4->PSC == (b32 ? 0U : 1U) && t4->ARR == (b32 ? 79999U : 39999U) && t4->CCMR2 == 0 && t4->CCER == 0);
	Check(t4->CR1 == (TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN) && s_Io[8] == ug + 1);

	// Another PwmDev_t on the TIM takes it over, the first one fails from then on
	Check(PWMInit(&other, &cfg) && PWMSetDutyCycle(&other, 0, 0) && !PWMSetDutyCycle(&pwm, 0, 0) &&
		  !PWMEnable(&pwm) && !PWMOpenChannel(&pwm, ch, 1));
	Check(PWMInit(&pwm, &cfg) && !PWMEnable(&other) && PWMEnable(&pwm));

	// Channel refusals
	PwmChanCfg_t bad = ch[0];
	bad.Chan = 4;
	Check(!PWMOpenChannel(&pwm, &bad, 1));
	bad.Chan = -1;
	Check(!PWMOpenChannel(&pwm, &bad, 1));
	bad.Chan = 0;
	bad.PinOp = IOPINOP_GPIO;
	Check(!PWMOpenChannel(&pwm, &bad, 1));
	bad.PinOp = IOPINOP_FUNC15 + 1;
	Check(!PWMOpenChannel(&pwm, &bad, 1));
	bad.PinOp = IOPINOP_FUNC1;
	bad.Pol = (PWM_POL)3;
	Check(!PWMOpenChannel(&pwm, &bad, 1));
	PwmChanCfg_t dup[2] = { ch[0], ch[0] };
	PwmChanCfg_t five[5] = { ch[0], ch[1], ch[0], ch[0], ch[0] };
	Check(!PWMOpenChannel(&pwm, dup, 2) && !PWMOpenChannel(&pwm, ch, 0) && !PWMOpenChannel(&pwm, nullptr, 1) &&
		  !PWMOpenChannel(&pwm, five, 5));
	Check(s_CfgCnt == 0 && t4->CCER == 0);

	// Duty set before the channel opens
	Check(PWMSetDutyCycle(&pwm, 0, 25) && !PWMSetDutyCycle(&pwm, 4, 25) && !PWMSetDutyCycle(&pwm, -1, 25));
	t4->CCMR1 = 0xFFFFFFFF;		// Left by other code
	Check(PWMOpenChannel(&pwm, ch, 2) && s_CfgCnt == 2);
	// Pin given to the TIM after the channel drives its inactive level
	Check(s_CfgPort == IOPORTB && s_CfgPin == 1 && s_CfgOp == IOPINOP_FUNC1 && s_CfgDir == IOPINDIR_OUTPUT &&
		  s_CfgWave[0] == 0 && s_CfgWave[1] == 1000);
	Check((t4->CCMR1 & (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE)) == (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE) &&
		  (t4->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S)) == 0);
	Check(Wave(t4, 0) == 0 && s_Period == 80000 && Wave(t4, 1) == 1000 && Wave(t4, 2) == -1);

	// Start: duty cycle on the outputs, active low inverted
	Check(!PWMStart(&pwm, 100) && Wave(t4, 0) == 0);
	Check(PWMStart(&pwm, 0) && Wave(t4, 0) == 250 && Wave(t4, 1) == 1000 && PWMStart(&pwm, 0));
	Check(PWMSetDutyCycle(&pwm, 1, 100) && Wave(t4, 1) == 0 && PWMSetDutyCycle(&pwm, 1, 150) && Wave(t4, 1) == 0);
	Check(PWMSetDutyCycle(&pwm, 1, -5) && Wave(t4, 1) == 1000 && PWMSetDutyCycle(&pwm, 1, 50) && Wave(t4, 1) == 500);
	Check(PWMSetDutyCycle(&pwm, 0, 1) && Wave(t4, 0) == 10 && PWMSetDutyCycle(&pwm, 0, 99) && Wave(t4, 0) == 990);
	Check(PWMSetDutyCycle(&pwm, 1, 300) && Wave(t4, 1) == 0 && PWMSetDutyCycle(&pwm, 1, 50));
	Check(PWMSetDutyCycle(&pwm, 0, 25) && Wave(t4, 0) == 250);
	// A closed channel is not touched
	Check(PWMSetDutyCycle(&pwm, 2, 50) && t4->CCR3 == 0 && Wave(t4, 2) == -1);

	// Frequency: 20 kHz is 4000 counts, the duty cycles stay
	ug = s_Io[8];
	Check(PWMSetFrequency(&pwm, 20000) && pwm.Freq == 20000 && Wave(t4, 0) == 250 && s_Period == 4000 &&
		  Wave(t4, 1) == 500 && t4->PSC == 0 && s_Io[8] == ug + 1);
	Check(!PWMSetFrequency(&pwm, 0) && Wave(t4, 0) == 250 && s_Period == 4000);
	// 1007 Hz: 79443.9 counts, 39721.95 with prescaler 2 on 16 bits
	Check(PWMSetFrequency(&pwm, 1007) && Wave(t4, 0) == 250 && s_Period == 79444 && pwm.Freq == WaveFreq());
	// 7 kHz: 11428.6 counts, 11429
	Check(PWMSetFrequency(&pwm, 7000) && Wave(t4, 0) == 250 && s_Period == 11429 && pwm.Freq == WaveFreq());

	// A channel opened while started runs at once
	PwmChanCfg_t c3 = Chan(3, PWM_POL_HIGH, IOPORTA, 7);
	Check(PWMSetDutyCycle(&pwm, 3, 75) && PWMOpenChannel(&pwm, &c3, 1) && Wave(t4, 3) == 750);

	// An open channel opened again on another pin: the first pin goes back to
	// GPIO at its inactive level
	PwmChanCfg_t two = Chan(1, PWM_POL_HIGH, IOPORTA, 6);
	int cnt = s_CfgCnt;
	GPIOB->ODR &= ~(1U << 1);
	Check(PWMSetDutyCycle(&pwm, 1, 30) && Wave(t4, 1) == 700 && PWMOpenChannel(&pwm, &two, 1) &&
		  s_CfgCnt == cnt + 2 && s_CfgPin == 6 && (GPIOB->ODR & (1U << 1)) && Wave(t4, 1) == 300);
	Check(PWMOpenChannel(&pwm, &ch[1], 1) && s_CfgCnt == cnt + 4 && s_CfgPin == 1 && Wave(t4, 1) == 700 &&
		  !(GPIOA->ODR & (1U << 6)) && PWMSetDutyCycle(&pwm, 1, 50));

	// Stop: outputs at their inactive level, counter running
	PWMStop(&pwm);
	Check(Wave(t4, 0) == 0 && Wave(t4, 1) == 1000 && Wave(t4, 3) == 0 && Started(t4));
	// A channel opened while stopped stays inactive
	Check(PWMOpenChannel(&pwm, &c3, 1) && Wave(t4, 3) == 0);

	// Close: pin back to GPIO at the inactive level, high for active low
	cnt = s_CfgCnt;
	PWMCloseChannel(&pwm, 1);
	Check(s_CfgCnt == cnt + 1 && s_CfgPort == IOPORTB && s_CfgPin == 1 && s_CfgOp == IOPINOP_GPIO &&
		  s_CfgDir == IOPINDIR_OUTPUT && (s_CfgOdr & (1U << 1)) && Wave(t4, 1) == -1);
	GPIOA->ODR |= 1U << 5;
	PWMCloseChannel(&pwm, 0);
	Check(s_CfgPort == IOPORTA && s_CfgPin == 5 && !(s_CfgOdr & (1U << 5)) && Wave(t4, 0) == -1);
	cnt = s_CfgCnt;
	PWMCloseChannel(&pwm, 0);
	PWMCloseChannel(&pwm, 9);
	PWMCloseChannel(&pwm, -1);
	Check(s_CfgCnt == cnt && Wave(t4, 3) == 0 && (t4->CCER & 0xFF) == 0);

	// Disable stops the counter and the outputs, Enable runs the counter again
	Check(PWMStart(&pwm, 0) && Wave(t4, 3) == 750);
	PWMDisable(&pwm);
	Check(!Started(t4) && Wave(t4, 3) == 0 && PWMEnable(&pwm) && Started(t4) && Wave(t4, 3) == 0);
	Check(PWMStart(&pwm, 0) && Wave(t4, 3) == 750);

	// Initialized again: the open channel pin goes back to idle
	cnt = s_CfgCnt;
	GPIOA->ODR |= 1U << 7;
	Check(PWMInit(&pwm, &cfg) && s_CfgCnt == cnt + 1 && s_CfgPin == 7 && s_CfgOp == IOPINOP_GPIO &&
		  !(s_CfgOdr & (1U << 7)) && t4->CCER == 0 && Wave(t4, 3) == -1);

	// Center aligned, 1 kHz: 40000 counts up and 40000 down
	cfg = Cfg(dev4, 1000, PWM_MODE_CENTER);
	Check(PWMInit(&pwm, &cfg) && pwm.Freq == 1000 && pwm.Mode == PWM_MODE_CENTER &&
		  (t4->CR1 & TIM_CR1_CMS) == TIM_CR1_CMS_0);
	Check(PWMOpenChannel(&pwm, ch, 1) && PWMStart(&pwm, 0) && Wave(t4, 0) == 0 && s_Period == 80000 &&
		  PWMSetDutyCycle(&pwm, 0, 100) && Wave(t4, 0) == 1000 && s_High == s_Period &&
		  PWMSetDutyCycle(&pwm, 0, 10) && Wave(t4, 0) == 100);
	Check(PWMSetFrequency(&pwm, 20000) && Wave(t4, 0) == 100 && s_Period == 4000 && pwm.Freq == 20000);

	// 2 Hz: 40000000 counts, 32 bit counter or prescaler 611 on 16 bits
	cfg = Cfg(dev4, 2, PWM_MODE_EDGE);
	Check(PWMInit(&pwm, &cfg) && PWMOpenChannel(&pwm, ch, 1) && PWMSetDutyCycle(&pwm, 0, 100) &&
		  PWMStart(&pwm, 0) && Wave(t4, 0) == 1000 && s_High == s_Period &&
		  s_Period == (b32 ? 40000000U : 611U * 65466U) &&
		  pwm.Freq == 2 && WaveFreq() == 2);
	Check(PWMSetDutyCycle(&pwm, 0, 33) && Wave(t4, 0) == 330);

	// Timer clock twice PCLK when the APB prescaler divides
#ifdef IOSONATA_STM32_H7
	RCC->D2CFGR = (4U << RCC_D2CFGR_D2PPRE1_Pos) | (4U << RCC_D2CFGR_D2PPRE2_Pos);
#endif
	TimClock(TEST_CLK / 2, TEST_CLK, TEST_CLK);
	cfg = Cfg(dev4, 1000, PWM_MODE_EDGE);
	Check(PWMInit(&pwm, &cfg) && PWMOpenChannel(&pwm, ch, 1) && PWMSetDutyCycle(&pwm, 0, 40) &&
		  PWMStart(&pwm, 0) && Wave(t4, 0) == 400 && s_Period == 80000 && pwm.Freq == 1000);
#ifdef IOSONATA_STM32_H7
	RCC->D2CFGR = 0;
#endif
	TimClock(TEST_CLK, TEST_CLK, TEST_CLK);

	// A TIM bound by the timer driver is refused, until released
	TimerDev_t timer;
	TimerCfg_t tcfg = { dev1, TIMER_CLKSRC_DEFAULT, 1000000, 1, nullptr, false };
	cfg = Cfg(dev1, 50000, PWM_MODE_EDGE);
	Check(Stm32TimInit(&timer, &tcfg) && !PWMInit(&other, &cfg));
	timer.Disable(&timer);
	Check(!Started(t1) && !PWMInit(&other, &cfg));
	Stm32TimRelease(dev1);

	// One channel TIM, 50 kHz: 1600 counts
	s_Tim = t1;
	Check(PWMInit(&other, &cfg) && !PWMOpenChannel(&other, &two, 1) && !PWMOpenChannel(&other, ch, 2) &&
		  !PWMSetDutyCycle(&other, 1, 50));
	Check(PWMOpenChannel(&other, ch, 1) && PWMStart(&other, 0) && Wave(t1, 0) == 0 && s_Period == 1600 &&
		  PWMSetDutyCycle(&other, 0, 20) && Wave(t1, 0) == 200);
#ifdef IS_TIM_BREAK_INSTANCE
	Check(!IS_TIM_BREAK_INSTANCE(t1) || (t1->BDTR & TIM_BDTR_MOE));
#endif
	// The two devices are separate
	Check(PWMSetDutyCycle(&pwm, 0, 90) && Wave(t4, 0) == 900 && Wave(t1, 0) == 200 && Started(t4));

	// Init on another TIM releases the previous one
#ifdef TIM1
	TIM_TypeDef *next = TIM1;
#else
	TIM_TypeDef *next = TIM3;
#endif
	cnt = s_CfgCnt;
	cfg = Cfg(DevNo(next), 1000, PWM_MODE_EDGE);
	Stm32TimHwGet(dev1, &hw);
	Check(PWMInit(&other, &cfg) && s_CfgCnt == cnt + 1 && s_CfgOp == IOPINOP_GPIO && !(*hw.pEnReg & hw.Mask) &&
		  Started(t4) && Wave(t4, 0) == 900);
	// Other code turning the clock on finds the counter stopped, and the TIM
	// is no longer seen as used by this driver
	*hw.pEnReg |= hw.Mask;
	Check(!Started(t1));
	t1->CR1 = TIM_CR1_CEN;
	PwmCfg_t cfg1 = Cfg(dev1, 50000, PWM_MODE_EDGE);
	Check(!PWMInit(&third, &cfg1));
	t1->CR1 = 0;
	*hw.pEnReg &= ~hw.Mask;
	s_Tim = next;
	Check(PWMOpenChannel(&other, ch, 1) && PWMSetDutyCycle(&other, 0, 40) && PWMStart(&other, 0) &&
		  Wave(next, 0) == 400 && s_Period == 80000);
#ifdef TIM1
	// Advanced TIM: main output enable
	Check((TIM1->BDTR & TIM_BDTR_MOE) && TIM1->ARR == 39999);
#endif
#ifdef TIM15
	// TIM15: break function, no center mode; a refused Init changes nothing
	cfg = Cfg(DevNo(TIM15), 1000, PWM_MODE_CENTER);
	Check(!PWMInit(&other, &cfg) && Wave(next, 0) == 400 && PWMEnable(&other));
	cfg.Mode = PWM_MODE_EDGE;
	Check(PWMInit(&other, &cfg) && (TIM15->BDTR & TIM_BDTR_MOE));
#endif

	// 16 bit counter at its limit, timer clock 131.07 MHz. Center 1 kHz is
	// 65535 counts up: the compare value of 100 percent would be 65536.
	TimClock(131070000, 131070000, 131070000);
	s_Tim = TEST_TIM16C;
	cfg = Cfg(DevNo(TEST_TIM16C), 1000, PWM_MODE_CENTER);
	Check(PWMInit(&third, &cfg) && PWMOpenChannel(&third, ch, 1) && PWMSetDutyCycle(&third, 0, 100) &&
		  PWMStart(&third, 0) && Wave(TEST_TIM16C, 0) == 1000 && s_High == s_Period && s_Period == 131072 &&
		  third.Freq == WaveFreq());
	Check(PWMSetDutyCycle(&third, 0, 50) && Wave(TEST_TIM16C, 0) == 500);
	// Edge 2 kHz is 65535 counts: ARR 65534, 100 percent 65535
	cfg = Cfg(DevNo(TEST_TIM16C), 2000, PWM_MODE_EDGE);
	Check(PWMInit(&third, &cfg) && PWMOpenChannel(&third, ch, 1) && PWMSetDutyCycle(&third, 0, 100) &&
		  PWMStart(&third, 0) && Wave(TEST_TIM16C, 0) == 1000 && s_High == s_Period && s_Period == 65535 &&
		  third.Freq == 2000);

	// Closest period of the prescaler: 1003 Hz from 80 MHz is 79760.7 counts,
	// 39880.4 with prescaler 2
	TimClock(TEST_CLK, TEST_CLK, TEST_CLK);
	cfg = Cfg(DevNo(TEST_TIM16C), 1003, PWM_MODE_EDGE);
	Check(PWMInit(&third, &cfg) && TEST_TIM16C->PSC == 1 && TEST_TIM16C->ARR == 39879 && third.Freq == 1003);
	// 65535.6 counts: prescaler 2 and 65536 counts, closer than 65535
	TimClock(65535600, 65535600, 65535600);
	cfg.Freq = 1000;
	Check(PWMInit(&third, &cfg) && TEST_TIM16C->PSC == 1 && TEST_TIM16C->ARR == 32767);

	// Largest prescaler, 65536: 1 Hz from 65535 x 65536 Hz, and one Hz more
	// does not fit
	TimClock(4294901760U, 4294901760U, 4294901760U);
	cfg = Cfg(DevNo(TEST_TIM16C), 1, PWM_MODE_EDGE);
	Check(PWMInit(&third, &cfg) && TEST_TIM16C->PSC == 65535 && TEST_TIM16C->ARR == 65534 && third.Freq == 1);
	TimClock(4294967295U, 4294967295U, 4294967295U);
	Check(!PWMInit(&third, &cfg) && TEST_TIM16C->PSC == 65535);

	s_Io[2] = s_Checks;

	while (1)
	{
	}
}
