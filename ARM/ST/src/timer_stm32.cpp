/**-------------------------------------------------------------------------
@file	timer_stm32.cpp

@brief	STM32 timer initialization

		TimerInit passes the low power timers to timer_lptim_stm32.cpp and
		the general purpose timers to timer_tim_stm32.cpp. DevNo order and
		the timers of each part are listed in timer_stm32.h.

		Stm32TimerIRQHandler, called by Stm32TimerVector of the vector file,
		passes the timer interrupts to the two drivers.

		The tick interrupt (bTickInt) is not supported. IntPrio goes from 0
		to the lowest priority the NVIC implements, 3 on F0, 15 on L4, G4,
		F4 and WBA. A TimerDev_t moved to another timer releases the timer it
		was bound to: that timer is stopped, also when the new one fails to
		start.

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
#include "timer_stm32.h"

// DevNo bound to pTimer, -1 if none
static int Stm32TimerFind(TimerDev_t * const pTimer)
{
#if ST_LPTIM_CNT > 0
	int devno = Stm32LptimFind(pTimer);

	if (devno >= 0)
	{
		return devno;
	}
#endif

	return Stm32TimFind(pTimer);
}

/**
 * @brief   Timer initialization.
 *
 * @param	pTimer : Timer device data to initialize
 * @param	pCfg   : Timer configuration data, DevNo as listed in timer_stm32.h
 *
 * @return	true - Success
 */
bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= ST_LPTIM_CNT + Stm32TimCount())
	{
		return false;
	}

	if (pCfg->bTickInt || pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS))
	{
		return false;
	}

	int devno = Stm32TimerFind(pTimer);

	if (devno >= 0 && devno != pCfg->DevNo)
	{
		// Interrupts of the old timer would update pTimer
#if ST_LPTIM_CNT > 0
		if (devno < ST_LPTIM_CNT)
		{
			Stm32LptimRelease(devno);
		}
		else
#endif
		{
			Stm32TimRelease(devno);
		}
	}

#if ST_LPTIM_CNT > 0
	if (pCfg->DevNo < ST_LPTIM_CNT)
	{
		return Stm32LptimInit(pTimer, pCfg);
	}
#endif

	return Stm32TimInit(pTimer, pCfg);
}

// Timer vectors, from Stm32TimerVector of the vector file
void Stm32TimerIRQHandler(int IrqNo)
{
#if ST_LPTIM_CNT > 0
	Stm32LptimIRQHandler(IrqNo);
#endif
	Stm32TimIRQHandler(IrqNo);
}

int TimerGetLowFreqDevCount(void)
{
	return ST_LPTIM_CNT;
}

int TimerGetHighFreqDevCount(void)
{
	return Stm32TimCount();
}

int TimerGetHighFreqDevNo(void)
{
	return ST_LPTIM_CNT;
}
