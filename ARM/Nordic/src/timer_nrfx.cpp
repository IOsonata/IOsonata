/**-------------------------------------------------------------------------
@file	timer_nrfx.cpp

@brief	timer implementation on Nordic nRFx series

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

bool TimerInit(TIMER * const pTimer, const TIMER_CFG * const pCfg)
{
	if (pTimer == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < TIMER_NRFX_RTC_MAX)
	{
		return nRFxRtcInit(pTimer, pCfg);
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

