/**-------------------------------------------------------------------------
@file	timer_stm32l4x.cpp

@brief	timer implementation on STM32L4xx series

@author	Hoang Nguyen Hoan
@date	Nov. 24, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "stm32l4xx.h"

#include "timer_stm32l4x.h"

STM32L4XX_TimerData_t g_Stm32l4TimerData[STM32L4XX_TIMER_MAXCNT] = {
	{ 0, {.pLPTimReg = LPTIM1} , 0 },
	{ 1, {.pLPTimReg = LPTIM2}, 0 },
	{ 2, {.pTimReg = TIM1}, 0 },
	{ 3, {.pTimReg = TIM2}, 0 },
	{ 4, {.pTimReg = TIM3}, 0 },
	{ 5, {.pTimReg = TIM4}, 0 },
	{ 6, {.pTimReg = TIM5}, 0 },
	{ 7, {.pTimReg = TIM6}, 0 },
	{ 8, {.pTimReg = TIM7}, 0 },
	{ 9, {.pTimReg = TIM8}, 0 },
	{ 10, {.pTimReg = TIM15}, 0 },
	{ 11, {.pTimReg = TIM16}, 0 },
	{ 12, {.pTimReg = TIM17}, 0 },
};

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= STM32L4XX_TIMER_MAXCNT)
	{
		return false;
	}

	g_Stm32l4TimerData[pCfg->DevNo].pTimer = pTimer;

	if (pCfg->DevNo < STM32L4XX_LPTIM_CNT)
	{
		return Stm32l4LPTimInit(&g_Stm32l4TimerData[pCfg->DevNo], pCfg);
	}

	return Stm32l4TimInit(&g_Stm32l4TimerData[pCfg->DevNo], pCfg);
}

int TimerGetLowFreqDevCount()
{
	return STM32L4XX_LPTIM_CNT;
}

int TimerGetHighFreqDevCount()
{
	return STM32L4XX_TIM_CNT;
}

int TimerGetHighFreqDevNo()
{
	return STM32L4XX_LPTIM_CNT;
}

