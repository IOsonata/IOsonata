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

#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
bool nRFxTimerEnablePeiTrigger(TimerDev_t * const pTimer, int TrigNo,
	uint64_t nsPeriod, TIMER_TRIG_TYPE Type, PeiDev_t *pPei,
	PeiEvent_t *pEvent);
bool nRFxRtcEnablePeiTrigger(TimerDev_t *pTimer, int TrigNo,
	uint64_t nsPeriod, TIMER_TRIG_TYPE Type, PeiDev_t *pPei,
	PeiEvent_t *pEvent);
bool nRFxRtcEnablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvent, uint32_t Event);
bool nRFxRtcDisablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t Event);
#endif

int TimerEnablePeiTrigger(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvt, uint32_t msPeriod, TIMER_TRIG_TYPE Type)
{
#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	int trig;
	bool success;

	if (pTimer == NULL || pPei == NULL || pEvt == NULL || msPeriod == 0U ||
		pTimer->FindAvailTrigger == NULL)
	{
		return -1;
	}
	trig = pTimer->FindAvailTrigger(pTimer);
	if (trig < 0)
	{
		return -1;
	}
	if (pTimer->DevNo < TIMER_NRFX_RTC_MAX)
	{
		success = nRFxRtcEnablePeiTrigger(pTimer, trig,
			(uint64_t)msPeriod * 1000000ULL, Type, pPei, pEvt);
	}
	else
	{
		success = nRFxTimerEnablePeiTrigger(pTimer, trig,
			(uint64_t)msPeriod * 1000000ULL, Type, pPei, pEvt);
	}
	return success ? trig : -1;
#else
	(void)pTimer;
	(void)pPei;
	(void)pEvt;
	(void)msPeriod;
	(void)Type;
	return -1;
#endif
}

bool TimerEnablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvt, uint32_t Event)
{
#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	if (pTimer == NULL || pPei == NULL || pEvt == NULL ||
		pTimer->DevNo < 0 || pTimer->DevNo >= TIMER_NRFX_RTC_MAX)
	{
		return false;
	}
	return nRFxRtcEnablePeiEvent(pTimer, pPei, pEvt, Event);
#else
	(void)pTimer;
	(void)pPei;
	(void)pEvt;
	(void)Event;
	return false;
#endif
}

bool TimerDisablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t Evt)
{
#if defined(NRF52_SERIES) || defined(NRF52805_XXAA) || \
	defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	if (pTimer == NULL || pPei == NULL ||
		pTimer->DevNo < 0 || pTimer->DevNo >= TIMER_NRFX_RTC_MAX)
	{
		return false;
	}
	return nRFxRtcDisablePeiEvent(pTimer, pPei, Evt);
#else
	(void)pTimer;
	(void)pPei;
	(void)Evt;
	return false;
#endif
}

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

