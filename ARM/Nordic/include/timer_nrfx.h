/**-------------------------------------------------------------------------
@file	timer_nrfx.h

@brief	Timer class implementation on Nordic nRF5x & nRF91 series

Low freq timers, RTC, are assigned to device number from 0 to TIMER_NRFX_RTC_MAX - 1
High freq timers, TIMER, are assigned to device number from TIMER_NRFX_RTC_MAX to
TIMER_NRFX_RTC_MAX + TIMER_NRFX_HF_MAX -1

Device number mapping as follow:

DevNo		nRF52832 hardware		Clock type		Max Freq (Hz)	Used by SoftDevice
  0			RTC0					LFCLK			32k				Y
  1			RTC1					LFCLK			32k				Y
  2			RTC2					LFCLK			32k
  3			TIMER0					HFCLK			16M				Y
  4			TIMER1					HFCLK			16M
  5			TIMER2					HFCLK			16M
  6			TIMER3					HFCLK			16M
  7			TIMER4					HFCLK			16M

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
#if defined(NRF54L_SERIES)
#define TIMER_NRFX_RTC_BASE_FREQ   			1000000	// GRTC is fixed at 1MHz
#define TIMER_NRFX_RTC_MAX                 	GRTC_GRTC_NINTERRUPTS_SIZE	//!< Number RTC available
#define TIMER_NRFX_RTC_MAX_TRIGGER_EVT     	5	//!< Max number of supported counter trigger event
#else
#define TIMER_NRFX_RTC_BASE_FREQ   			32768
#define TIMER_NRFX_RTC_MAX                 	RTC_COUNT	//!< Number RTC available
#define TIMER_NRFX_RTC_MAX_TRIGGER_EVT     	RTC1_CC_NUM	//!< Max number of supported counter trigger event
#endif

/// High frequency timer using Timer 16MHz clock source.
///
#define TIMER_NRFX_HF_BASE_FREQ   			16000000
#define TIMER_NRFX_HF_MAX              		TIMER_COUNT		//!< Number high frequency timer available
#if TIMER_NRFX_HF_MAX < 4
#define TIMER_NRFX_HF_MAX_TRIGGER_EVT  		TIMER2_CC_NUM	//!< Max number of supported counter trigger event
#elif TIMER_NRFX_HF_MAX < 6
#define TIMER_NRFX_HF_MAX_TRIGGER_EVT  		TIMER3_CC_NUM	//!< Max number of supported counter trigger event
#elif TIMER_NRFX_HF_MAX < 8
#define TIMER_NRFX_HF_MAX_TRIGGER_EVT  		TIMER10_CC_NUM_SIZE	//!< Max number of supported counter trigger event
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(NRF54L_SERIES)
bool nRFxGrtcInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);
#else
bool nRFxRtcInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);
#endif
bool nRFxTimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);

#if defined(NRF54L_SERIES)
#define NRFX_TIMER_LF_INIT		nRFxGrtcInit	//!< Low frequency timer driver of the target
#else
#define NRFX_TIMER_LF_INIT		nRFxRtcInit		//!< Low frequency timer driver of the target
#endif
#define NRFX_TIMER_HF_INIT		nRFxTimerInit	//!< High frequency timer driver of the target

/// Timer drivers TimerInit selects from. A member left NULL is a driver the
/// application does not use: its code and data are not linked, apart from
/// its interrupt handlers, and TimerInit fails for a device number of its
/// range.
typedef struct __nRFx_Timer_Drv {
	bool (*LFInit)(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);	//!< Low frequency timers, device 0 to TIMER_NRFX_RTC_MAX - 1
	bool (*HFInit)(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);	//!< High frequency timers, device TIMER_NRFX_RTC_MAX and up
} nRFxTimerDrv_t;

#define NRFX_TIMER_DRV_LF		{ NRFX_TIMER_LF_INIT, NULL }				//!< Low frequency timers only
#define NRFX_TIMER_DRV_HF		{ NULL, NRFX_TIMER_HF_INIT }				//!< High frequency timers only
#define NRFX_TIMER_DRV_ALL		{ NRFX_TIMER_LF_INIT, NRFX_TIMER_HF_INIT }	//!< Both

/// The drivers TimerInit selects from. The library defines a weak default
/// with both, so any device number works and both drivers are linked. An
/// application that uses only one kind of timer defines it to link only
/// that driver, for instance
///
///		const nRFxTimerDrv_t g_nRFxTimerDrv = NRFX_TIMER_DRV_LF;
extern const nRFxTimerDrv_t g_nRFxTimerDrv;

/**
 * @brief	Initialize a low frequency timer.
 *
 * For code that knows its timer is a low frequency one. TimerInit selects
 * the driver from the device number at run time, which links both the low
 * and the high frequency timer drivers. This call links only the low
 * frequency one.
 *
 * @param	pTimer	: Pointer to the timer device
 * @param	pCfg	: Pointer to the timer configuration, DevNo in the low
 * 					  frequency range (0 to TIMER_NRFX_RTC_MAX - 1)
 *
 * @return	true - success
 */
static inline bool nRFxLFTimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	return NRFX_TIMER_LF_INIT(pTimer, pCfg);
}

#ifdef __cplusplus
}
#endif

#endif // __TIMER_NRFX_H__
