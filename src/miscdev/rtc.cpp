/**-------------------------------------------------------------------------
@file	rtc.cpp

@brief	Generic realtime clock, seconds since the epoch

The time_t functions of miscdev/rtc.h, on RtcSetTime and RtcGetTime of the
MCU implementation. The calendar is the local time of the C runtime:
localtime_r and mktime, UTC unless the application sets a time zone.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#include <time.h>

#include "miscdev/rtc.h"

bool RtcSetTimeEpoch(RtcDev_t * const pDev, time_t Time)
{
	struct tm tm;

	if (localtime_r(&Time, &tm) == nullptr)
	{
		return false;
	}

	return RtcSetTime(pDev, &tm);
}

bool RtcGetTimeEpoch(RtcDev_t * const pDev, struct timespec * const pTime)
{
	struct tm tm;
	uint32_t usec = 0;

	if (pTime == nullptr || RtcGetTime(pDev, &tm, &usec) == false)
	{
		return false;
	}

	// Daylight saving time from the time zone of the C runtime
	tm.tm_isdst = -1;

	time_t t = mktime(&tm);

	if (t == (time_t)-1)
	{
		return false;
	}

	pTime->tv_sec = t;
	pTime->tv_nsec = (long)usec * 1000L;

	return true;
}
