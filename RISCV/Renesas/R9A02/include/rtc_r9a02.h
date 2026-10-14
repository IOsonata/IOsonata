/**-------------------------------------------------------------------------
@file	rtc_r9a02.h

@brief	R9A02G021 realtime clock (RTC)

Calendar from 2000 to 2099 in 24-hour mode, fixed-cycle interrupt and
weekday, hour and minute alarm (R9A02G021 User's Manual R01UH1036EJ0110,
chapter 20). The RTC keeps running in Software Standby and its interrupt can
end it.

The count clock is the low frequency clock of g_McuOsc (SystemLowFreqClockSelect,
OSMCR.WUTMMCK0): the calendar and the alarm count only on the 32768 Hz
crystal (SOSC). On the LOCO only the fixed-cycle interrupt works (20.1).

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef __RTC_R9A02_H__
#define __RTC_R9A02_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/// Fixed-cycle interrupt period, RTCC0.CT (20.2.1)
typedef enum __R9a02_Rtc_Period {
	R9A02_RTC_PERIOD_NONE,
	R9A02_RTC_PERIOD_HALF_SEC,		//!< Every 0.5 s
	R9A02_RTC_PERIOD_SEC,			//!< Every second
	R9A02_RTC_PERIOD_MIN,			//!< Every minute, second 00
	R9A02_RTC_PERIOD_HOUR,			//!< Every hour, minute 00
	R9A02_RTC_PERIOD_DAY,			//!< Every day, 00:00:00
	R9A02_RTC_PERIOD_MONTH			//!< Every month, day 1 00:00:00
} R9A02_RTC_PERIOD;

#define R9A02_RTC_EVT_PERIOD		(1U << 0)	//!< Fixed-cycle interrupt
#define R9A02_RTC_EVT_ALARM			(1U << 1)	//!< Alarm match

/// RTC event handler, called in interrupt context
///
/// @param	Evt		: R9A02_RTC_EVT_xxx bits
/// @param	pCtx	: Context given to R9a02RtcInit
typedef void (*R9a02RtcEvtHandler_t)(uint32_t Evt, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

/// @brief	Start the RTC.
///
/// An RTC already counting on the same clock (after a reset other than the
/// power-on reset) keeps its time. Otherwise the time starts at
/// 2000-01-01 00:00:00.
///
/// @param	pHandler	: Event handler, NULL for none
/// @param	pCtx		: Handler context
/// @param	IntPrio		: Interrupt level 0 to 15
/// @param	bWakeup		: true for the RTC interrupt to end Software Standby
///
/// @return	false when the count clock is not running or the interrupt cannot
/// 			be routed
bool R9a02RtcInit(R9a02RtcEvtHandler_t pHandler, void *pCtx, int IntPrio, bool bWakeup);

/// @return	true when the calendar counts (SOSC count clock)
bool R9a02RtcCalendar(void);

/// @brief	Set the date and time. The weekday is computed from the date.
///
/// With the alarm enabled the fixed-cycle interrupt must be every second and
/// the call made from the period event, the counter access must end before
/// the next one (20.2.2).
///
/// @param	pTime	: Year 2000 to 2099 (tm_year 100 to 199)
///
/// @return	false on the LOCO, with an invalid time, or with the alarm
/// 			enabled and a period other than R9A02_RTC_PERIOD_SEC
bool R9a02RtcSetTime(const struct tm *pTime);

/// @brief	Read the date and time, the counters stopped during the read.
///
/// Same rule as R9a02RtcSetTime with the alarm enabled.
///
/// @return	false on the LOCO, or with the alarm enabled and a period other
/// 			than R9A02_RTC_PERIOD_SEC
bool R9a02RtcGetTime(struct tm *pTime);

/// @brief	Set the fixed-cycle interrupt.
bool R9a02RtcSetPeriod(R9A02_RTC_PERIOD Period);

/// @brief	Set the alarm, weekday mask, hour and minute (no seconds).
///
/// @param	WeekDays	: Bit n for weekday n, 0 Sunday to 6 Saturday
/// @param	Hour		: 0 to 23
/// @param	Min			: 0 to 59
///
/// @return	false on the LOCO or with an invalid setting
bool R9a02RtcSetAlarm(uint8_t WeekDays, int Hour, int Min);

/// @brief	Disable the alarm.
void R9a02RtcDisableAlarm(void);

/// @brief	Stop the counters and release the interrupt.
void R9a02RtcStop(void);

#ifdef __cplusplus
}
#endif

#endif
