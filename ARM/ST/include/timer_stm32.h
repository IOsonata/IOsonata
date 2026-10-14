/**-------------------------------------------------------------------------
@file	timer_stm32.h

@brief	STM32 timer implementation, definitions shared by its source files

		TimerInit (timer_stm32.cpp) passes the timer DevNo to the low power
		timer driver, timer_lptim_stm32.cpp, or to the general purpose timer
		driver, timer_tim_stm32.cpp. DevNo is a virtual index, the low power
		timers first, then the TIM from the basic timers to the advanced
		timers:

		LPTIM1, LPTIM2, TIM6, TIM7, TIM10, TIM11, TIM13, TIM14, TIM16, TIM17,
		TIM9, TIM12, TIM15, TIM2, TIM3, TIM4, TIM5, TIM1, TIM8, TIM20

		A timer the device header does not define takes no index:

		F030x8 : 0 TIM6, 1 TIM14, 2 TIM16, 3 TIM17, 4 TIM15, 5 TIM3, 6 TIM1
		F401   : 0 TIM10, 1 TIM11, 2 TIM9, 3 TIM2, 4 TIM3, 5 TIM4, 6 TIM5,
		         7 TIM1
		L432   : 0 LPTIM1, 1 LPTIM2, 2 TIM6, 3 TIM7, 4 TIM16, 5 TIM15,
		         6 TIM2, 7 TIM1
		L476, L496, L4S9 : 0 LPTIM1, 1 LPTIM2, 2 TIM6, 3 TIM7, 4 TIM16,
		         5 TIM17, 6 TIM15, 7 TIM2, 8 TIM3, 9 TIM4, 10 TIM5, 11 TIM1,
		         12 TIM8
		G474   : 0 LPTIM1, 1 TIM6, 2 TIM7, 3 TIM16, 4 TIM17, 5 TIM15, 6 TIM2,
		         7 TIM3, 8 TIM4, 9 TIM5, 10 TIM1, 11 TIM8, 12 TIM20
		WBA65  : 0 LPTIM1, 1 LPTIM2, 2 TIM16, 3 TIM17, 4 TIM2, 5 TIM3,
		         6 TIM4, 7 TIM1

		TimerGetLowFreqDevCount returns the number of LPTIM,
		TimerGetHighFreqDevNo the DevNo of the first TIM.

		Vectors: the vector file of the part points the vectors of these
		timers to Stm32TimerVector, a weak alias each. It calls the weak
		Stm32TimerIRQHandler with the number of the interrupt being
		serviced, defined by timer_stm32.cpp: an application not using
		TimerInit does not link the timer driver. An application handler
		replaces the alias of its vector, TimerInit then refuses the timers
		on that vector.

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
#ifndef __TIMER_STM32_H__
#define __TIMER_STM32_H__

#include <stdint.h>
#include <stdbool.h>

#include "stm32.h"
#include "coredev/timer.h"

// Low power timers handled by timer_lptim_stm32.cpp: LPTIM1 and LPTIM2 of
// the parts selecting their clock in RCC_CCIPR (L4, G4) or in RCC_CCIPR1
// and RCC_CCIPR3 (WBA).
#if defined(LPTIM1) && (defined(RCC_CCIPR_LPTIM1SEL_Pos) || defined(RCC_CCIPR3_LPTIM1SEL_Pos))
#define ST_LPTIM1
#endif
#if defined(LPTIM2) && (defined(RCC_CCIPR_LPTIM2SEL_Pos) || defined(RCC_CCIPR1_LPTIM2SEL_Pos))
#define ST_LPTIM2
#endif

#if defined(ST_LPTIM1) && defined(ST_LPTIM2)
#define ST_LPTIM_CNT		2
#elif defined(ST_LPTIM1) || defined(ST_LPTIM2)
#define ST_LPTIM_CNT		1
#else
#define ST_LPTIM_CNT		0
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Vector table of the part and the timer vector function, both defined by
// its vector file
extern void (* const __Vectors[])(void);
void Stm32TimerVector(void);

// Timer interrupt, IrqNo the interrupt number. Runs the timers on that
// vector.
void Stm32TimerIRQHandler(int IrqNo);

// True if vector IrqNo runs Stm32TimerVector, false if an application
// handler replaced it
static inline bool Stm32TimerVectorOwned(int IrqNo)
{
	return __Vectors[16 + IrqNo] == Stm32TimerVector;
}

// Init binds timer pCfg->DevNo to pTimer, taking it from the TimerDev_t
// it was bound to. Find returns the DevNo bound to pTimer, -1 if none.
// Release stops a timer and unbinds it. IRQHandler services the timers on
// vector IrqNo.

// Low power timers, DevNo 0 to ST_LPTIM_CNT - 1. Not linked when the part
// has none.
bool Stm32LptimInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);
int Stm32LptimFind(TimerDev_t * const pTimer);
void Stm32LptimRelease(int DevNo);
void Stm32LptimIRQHandler(int IrqNo);

// General purpose timers, DevNo ST_LPTIM_CNT and up
bool Stm32TimInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);
int Stm32TimFind(TimerDev_t * const pTimer);
void Stm32TimRelease(int DevNo);
void Stm32TimIRQHandler(int IrqNo);
int Stm32TimCount(void);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_STM32_H__
