/**-------------------------------------------------------------------------
@file	timer_stm32l4x.h

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
#ifndef __TIMER_STM32L4X_H__
#define __TIMER_STM32L4X_H__

#include <stdint.h>

#include "coredev/timer.h"

/// Low Power Timer
#define STM32L4XX_LPTIMER_MAXFREQ			16000000

#define STM32L4XX_LPTIMER_MAXCNT			2
#define STM32L4XX_LPTIMER_TRIG_MAXCNT		1

///
#define STM32L4XX_TIMER_MAXFREQ				16000000

#define STM32L4XX_TIMER_MAXCNT				11
#define STM32L4XX_TIMER_TRIG_MAXCNT			1


#ifdef __cplusplus
extern "C" {
#endif

bool Stm32l4LPTimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);
bool Stm32l4TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_STM32L4X_H__
