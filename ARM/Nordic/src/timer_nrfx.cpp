/**-------------------------------------------------------------------------
@file	timer_nrfx.cpp

@brief	timer implementation on Nordic nRFx series

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
#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "nrf.h"

#include "timer_nrfx.h"

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < TIMER_NRFX_RTC_MAX)
	{
#if defined(NRF54L_SERIES)
		return nRFxGrtcInit(pTimer, pCfg);
#else
		return nRFxRtcInit(pTimer, pCfg);
#endif
	}

	return nRFxTimerInit(pTimer, pCfg);
}

int TimerGetLowFreqDevCount()
{
	return TIMER_NRFX_RTC_MAX;
}

int TimerGetHighFreqDevCount()
{
	return TIMER_NRFX_HF_MAX;
}

int TimerGetHighFreqDevNo()
{
	return TIMER_NRFX_RTC_MAX;
}

