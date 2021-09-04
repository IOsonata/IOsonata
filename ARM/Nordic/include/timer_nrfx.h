/**-------------------------------------------------------------------------
@file	timer_nrfx.h

@brief	Timer class implementation on Nordic nRF5x & nRF91 series

Low freq timers, RTC, are assigned to device number from 0 to TIMER_NRFX_RTC_MAX - 1
High freq timers, TIMER, are assigned to device number from TIMER_NRFX_RTC_MAX to
TIMER_NRFX_RTC_MAX + TIMER_NRFX_HF_MAX -1

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2017

@license

MIT License

Copyright (c) 2017 I-SYST inc. All rights reserved.

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

#ifndef __TIMER_NRFX_H__
#define __TIMER_NRFX_H__

#include <stdint.h>

#include "nrf.h"
#include "nrf_peripherals.h"

#include "coredev/timer.h"

/// Low frequency timer using Real Time Counter (RTC) 32768 Hz clock source.
///
#define TIMER_NRFX_RTC_BASE_FREQ   			32768
#define TIMER_NRFX_RTC_MAX                 	RTC_COUNT	//!< Number RTC available
#define TIMER_NRFX_RTC_MAX_TRIGGER_EVT     	RTC1_CC_NUM	//!< Max number of supported counter trigger event

/// High frequency timer using Timer 16MHz clock source.
///
#define TIMER_NRFX_HF_BASE_FREQ   			16000000
#define TIMER_NRFX_HF_MAX              		TIMER_COUNT		//!< Number high frequency timer available
#if TIMER_NRFX_HF_MAX < 4
#define TIMER_NRFX_HF_MAX_TRIGGER_EVT  		TIMER2_CC_NUM	//!< Max number of supported counter trigger event
#else
#define TIMER_NRFX_HF_MAX_TRIGGER_EVT  		TIMER3_CC_NUM	//!< Max number of supported counter trigger event
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool nRFxRtcInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);
bool nRFxTimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_NRFX_H__
