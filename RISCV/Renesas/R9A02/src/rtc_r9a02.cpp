/**-------------------------------------------------------------------------
@file	rtc_r9a02.cpp

@brief	R9A02G021 realtime clock (RTC)

R9A02G021 User's Manual R01UH1036EJ0110, chapter 20. See rtc_r9a02.h.

The counters are read and written with RWAIT = 1 and RWST = 1 (20.3.3), the
counters stopped for at most a few count clock cycles, then RWAIT = 0 and
RWST = 0 before Software Standby can be entered. RTCC1 is written with 1 in
the flags that must stay set, a write of 1 does not set them (20.2.2).

With the alarm enabled the counters are read and written only with the 1 s
fixed-cycle interrupt, the access done before the next one (20.2.2): call
R9a02RtcGetTime and R9a02RtcSetTime from the period event.

The RTC has no module stop bit. It is not reset by a reset other than the
power-on reset (table 20.2), a counting RTC is kept by R9a02RtcInit. The
manual also lists RTCC0 as reset by every reset (table 5.3), RTCC0.RTCE is
read to know.

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
#include <stdint.h>
#include <stddef.h>

#include "r9a02g021.h"
#include "coredev/interrupt.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "rtc_r9a02.h"

#define R9A02_RTC_WAIT_US			200U		// RWST change, 2 count clock cycles (20.2.2)
#define R9A02_RTC_START_US			100U		// 2 cycles of the slowest LOCO after RTCE = 1 (20.3.2)
#define R9A02_RTC_YEAR_BASE			100			// tm_year of 2000
#define R9A02_RTC_FLAGS				(R9A02_RTCC1_RIFG | R9A02_RTCC1_WAFG)
#define R9A02_RTC_KEEP				(R9A02_RTCC1_WALE | R9A02_RTCC1_WALIE | R9A02_RTCC1_RWAIT)

typedef struct __R9a02_Rtc_Dev {
	R9a02RtcEvtHandler_t Handler;
	void *pCtx;
	int IntNo;						// IELSR, -1 when not routed
	bool bCalendar;					// SOSC count clock
} R9a02RtcDev_t;

static R9a02RtcDev_t s_R9a02RtcDev = { NULL, NULL, -1, false };

static inline uint8_t R9a02ToBcd(int Val)
{
	return (uint8_t)(((Val / 10) << 4) | (Val % 10));
}

static inline int R9a02FromBcd(uint8_t Val)
{
	return (Val >> 4) * 10 + (Val & 0xFU);
}

// Weekday, 0 Sunday, of a Gregorian date
static int R9a02WeekDay(int Year, int Month, int Day)
{
	static const uint8_t t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

	if (Month < 3)
	{
		Year--;
	}

	return (Year + Year / 4 - Year / 100 + Year / 400 + t[Month - 1] + Day) % 7;
}

static int R9a02MonthDays(int Year, int Month)
{
	static const uint8_t d[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

	// 2000 to 2099: every fourth year is a leap year (20.2.9)
	return Month == 2 && (Year % 4) == 0 ? 29 : d[Month - 1];
}

// RTCC1 write: WALE, WALIE and RWAIT kept unless changed, flags in ClrFlags
// cleared, the others written 1 so that they stay
static void R9a02RtcC1Write(uint8_t Set, uint8_t Clr, uint8_t ClrFlags)
{
	uint8_t v = (uint8_t)((R9A02_RTC->RTCC1 & R9A02_RTC_KEEP & ~Clr) | Set);

	R9A02_RTC->RTCC1 = (uint8_t)(v | (R9A02_RTC_FLAGS & ~ClrFlags));
}

static bool R9a02RtcWaitRwst(bool bSet)
{
	uint32_t n = R9A02_RTC_WAIT_US;

	while (((R9A02_RTC->RTCC1 & R9A02_RTCC1_RWST) != 0U) != bSet)
	{
		if (n-- == 0U)
		{
			return false;
		}
		R9a02DelayUs(1);
	}

	return true;
}

// Counters stopped for access (20.3.3)
static bool R9a02RtcHold(void)
{
	R9a02RtcC1Write(R9A02_RTCC1_RWAIT, 0U, 0U);

	return R9a02RtcWaitRwst(true);
}

static bool R9a02RtcRelease(void)
{
	R9a02RtcC1Write(0U, R9A02_RTCC1_RWAIT, 0U);

	return R9a02RtcWaitRwst(false);
}

// With the alarm enabled the counters are accessed only with the 1 s
// fixed-cycle interrupt (20.2.2)
static bool R9a02RtcAccessAllowed(void)
{
	return (R9A02_RTC->RTCC1 & R9A02_RTCC1_WALE) == 0U ||
		   (R9A02_RTC->RTCC0 & R9A02_RTCC0_CT_Msk) == R9A02_RTCC0_CT_SEC;
}

static void R9a02RtcIrqHandler(int IntNo, void *pCtx)
{
	uint8_t c1 = R9A02_RTC->RTCC1;
	uint8_t flags = c1 & R9A02_RTC_FLAGS;
	uint32_t evt = 0;

	if (flags == 0U)
	{
		return;
	}

	R9a02RtcC1Write(0U, 0U, flags);

	if (flags & R9A02_RTCC1_RIFG)
	{
		evt |= R9A02_RTC_EVT_PERIOD;
	}
	if (flags & R9A02_RTCC1_WAFG)
	{
		evt |= R9A02_RTC_EVT_ALARM;
	}

	if (s_R9a02RtcDev.Handler)
	{
		s_R9a02RtcDev.Handler(evt, s_R9a02RtcDev.pCtx);
	}
}

bool R9a02RtcInit(R9a02RtcEvtHandler_t pHandler, void *pCtx, int IntPrio, bool bWakeup)
{
	R9A02_RTC_Type *reg = R9A02_RTC;

	if (IntPrio < 0 || IntPrio > (int)R9A02_CLIC_PRIO_MAX)
	{
		return false;
	}

	// RTCCLK: LOCO with OSMCR.WUTMMCK0 = 1, else SOSC (20.1), which must run
	bool loco = (R9A02_SYSC_OSMCR & R9A02_OSMCR_WUTMMCK0) != 0U;

	if (loco ? (R9A02_SYSC_LOCOCR & R9A02_LOCOCR_LCSTP) : (R9A02_SYSC_SOSCCR & R9A02_SOSCCR_SOSTP))
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();

	// Wake-up source off before the interrupt is released (12.5.2)
	R9a02WakeupSet(R9A02_EVT_RTC_ALM_OR_PRD, false);
	R9a02UnregisterIntHandler(s_R9a02RtcDev.IntNo);
	s_R9a02RtcDev.IntNo = -1;
	s_R9a02RtcDev.bCalendar = !loco;

	bool ok = true;

	if ((reg->RTCC0 & R9A02_RTCC0_RTCE) == 0U || (reg->RTCC0 & R9A02_RTCC0_RTC128EN))
	{
		// Start-up (figure 20.2): stopped first, RCLOE1 and RTC128EN change
		// only with RTCE = 0. 24-hour mode, no fixed-cycle interrupt, the
		// counters set to 2000-01-01 00:00:00, no correction, no alarm.
		reg->RTCC0 = (uint8_t)(reg->RTCC0 & ~R9A02_RTCC0_RTCE);
		reg->RTCC0 = 0U;
		reg->RTCC0 = R9A02_RTCC0_AMPM;
		reg->SEC = 0U;
		reg->MIN = 0U;
		reg->HOUR = 0U;
		reg->WEEK = (uint8_t)R9a02WeekDay(2000, 1, 1);
		reg->DAY = 0x01U;
		reg->MONTH = 0x01U;
		reg->YEAR = 0x00U;
		reg->SUBCUD = 0U;
		reg->RTCC1 = 0U;
		reg->ALARMWM = 0U;
		reg->ALARMWH = 0U;
		reg->ALARMWW = 0U;
		reg->RTCC0 = R9A02_RTCC0_AMPM | R9A02_RTCC0_RTCE;

		// Software Standby can be entered right away after the RWAIT
		// handshake, on the LOCO after 2 count clock cycles (20.3.2). A
		// handshake that does not end means the crystal does not oscillate.
		if (s_R9a02RtcDev.bCalendar)
		{
			ok = R9a02RtcHold();
			ok = R9a02RtcRelease() && ok;
			if (!ok)
			{
				reg->RTCC0 = R9A02_RTCC0_AMPM;
			}
		}
		else
		{
			R9a02DelayUs(R9A02_RTC_START_US);
		}
	}
	else
	{
		// Counting: the time is kept, the stale flags are cleared. A 12-hour
		// mode left by other firmware is changed with RWAIT = 1, HOUR follows
		// (20.2.1).
		if (s_R9a02RtcDev.bCalendar)
		{
			if ((reg->RTCC0 & R9A02_RTCC0_AMPM) == 0U && R9a02RtcHold())
			{
				reg->RTCC0 = (uint8_t)(reg->RTCC0 | R9A02_RTCC0_AMPM);
			}
			ok = R9a02RtcRelease();
		}
		R9a02RtcC1Write(0U, R9A02_RTCC1_RWAIT, R9A02_RTC_FLAGS);
	}

	if (ok)
	{
		s_R9a02RtcDev.Handler = pHandler;
		s_R9a02RtcDev.pCtx = pCtx;
		s_R9a02RtcDev.IntNo = R9a02RegisterIntHandler(R9A02_EVT_RTC_ALM_OR_PRD, IntPrio, R9a02RtcIrqHandler, NULL);
		ok = s_R9a02RtcDev.IntNo >= 0;
	}

	if (ok)
	{
		R9a02WakeupSet(R9A02_EVT_RTC_ALM_OR_PRD, bWakeup);
	}

	EnableInterrupt(state);

	return ok;
}

bool R9a02RtcCalendar(void)
{
	return s_R9a02RtcDev.bCalendar && (R9A02_RTC->RTCC0 & R9A02_RTCC0_RTCE);
}

// The RTC interrupt is held off while the counters are changed (20.3.3)
bool R9a02RtcSetTime(const struct tm *pTime)
{
	if (pTime == NULL || !R9a02RtcCalendar() || !R9a02RtcAccessAllowed())
	{
		return false;
	}

	int year = pTime->tm_year + 1900;
	int month = pTime->tm_mon + 1;

	if (pTime->tm_year < R9A02_RTC_YEAR_BASE || pTime->tm_year > R9A02_RTC_YEAR_BASE + 99 ||
		month < 1 || month > 12 || pTime->tm_mday < 1 || pTime->tm_mday > R9a02MonthDays(year, month) ||
		pTime->tm_hour < 0 || pTime->tm_hour > 23 || pTime->tm_min < 0 || pTime->tm_min > 59 ||
		pTime->tm_sec < 0 || pTime->tm_sec > 59)
	{
		return false;
	}

	R9A02_RTC_Type *reg = R9A02_RTC;
	uintptr_t state = DisableInterrupt();
	bool ok = R9a02RtcHold();

	if (ok)
	{
		reg->SEC = R9a02ToBcd(pTime->tm_sec);
		reg->MIN = R9a02ToBcd(pTime->tm_min);
		reg->HOUR = R9a02ToBcd(pTime->tm_hour);
		reg->DAY = R9a02ToBcd(pTime->tm_mday);
		reg->MONTH = R9a02ToBcd(month);
		reg->YEAR = R9a02ToBcd(pTime->tm_year - R9A02_RTC_YEAR_BASE);
		reg->WEEK = (uint8_t)R9a02WeekDay(year, month, pTime->tm_mday);
	}

	ok = R9a02RtcRelease() && ok;

	// Flags raised by the change are not events (figure 20.5)
	R9a02RtcC1Write(0U, 0U, R9A02_RTC_FLAGS);
	R9a02ClearPendingInt(s_R9a02RtcDev.IntNo);

	EnableInterrupt(state);

	return ok;
}

bool R9a02RtcGetTime(struct tm *pTime)
{
	if (pTime == NULL || !R9a02RtcCalendar() || !R9a02RtcAccessAllowed())
	{
		return false;
	}

	R9A02_RTC_Type *reg = R9A02_RTC;
	uint8_t sec = 0;
	uint8_t min = 0;
	uint8_t hour = 0;
	uint8_t day = 0x01;
	uint8_t month = 0x01;
	uint8_t year = 0;
	uint8_t week = 0;
	uintptr_t state = DisableInterrupt();
	bool ok = R9a02RtcHold();

	if (ok)
	{
		sec = reg->SEC;
		min = reg->MIN;
		hour = reg->HOUR;
		day = reg->DAY;
		month = reg->MONTH;
		year = reg->YEAR;
		week = reg->WEEK;
	}

	ok = R9a02RtcRelease() && ok;

	EnableInterrupt(state);

	if (!ok)
	{
		return false;
	}

	int y = 2000 + R9a02FromBcd(year);
	int m = R9a02FromBcd(month & 0x1FU);
	int yday = 0;

	for (int i = 1; i < m && i <= 12; i++)
	{
		yday += R9a02MonthDays(y, i);
	}

	pTime->tm_sec = R9a02FromBcd(sec & 0x7FU);
	pTime->tm_min = R9a02FromBcd(min & 0x7FU);
	pTime->tm_hour = R9a02FromBcd(hour & 0x3FU);
	pTime->tm_mday = R9a02FromBcd(day & 0x3FU);
	pTime->tm_mon = m - 1;
	pTime->tm_year = y - 1900;
	pTime->tm_wday = week & 7U;
	pTime->tm_yday = yday + pTime->tm_mday - 1;
	pTime->tm_isdst = 0;

	return true;
}

// CT rewritten with the RTC interrupt held off, then RIFG cleared (20.2.1).
// Bit 6 of RTCC0 is written 0.
bool R9a02RtcSetPeriod(R9A02_RTC_PERIOD Period)
{
	if ((uint32_t)Period > R9A02_RTCC0_CT_MONTH || s_R9a02RtcDev.IntNo < 0)
	{
		return false;
	}

	uint32_t keep = R9A02_RTCC0_AMPM | R9A02_RTCC0_RTC128EN | R9A02_RTCC0_RCLOE1 | R9A02_RTCC0_RTCE;
	uintptr_t state = DisableInterrupt();

	R9A02_RTC->RTCC0 = (uint8_t)((R9A02_RTC->RTCC0 & keep) | (uint32_t)Period);
	R9a02RtcC1Write(0U, 0U, R9A02_RTCC1_RIFG);
	R9a02ClearPendingInt(s_R9a02RtcDev.IntNo);

	EnableInterrupt(state);

	return true;
}

// WALE off before the alarm registers and WALIE are written, WAFG cleared
// after WALE is set (20.2.2, figure 20.6)
bool R9a02RtcSetAlarm(uint8_t WeekDays, int Hour, int Min)
{
	if (!R9a02RtcCalendar() || (WeekDays & 0x80U) || WeekDays == 0U || Hour < 0 || Hour > 23 ||
		Min < 0 || Min > 59)
	{
		return false;
	}

	R9A02_RTC_Type *reg = R9A02_RTC;
	uintptr_t state = DisableInterrupt();

	R9a02RtcC1Write(0U, R9A02_RTCC1_WALE, 0U);
	R9a02RtcC1Write(R9A02_RTCC1_WALIE, 0U, 0U);
	reg->ALARMWM = R9a02ToBcd(Min);
	reg->ALARMWH = R9a02ToBcd(Hour);
	reg->ALARMWW = WeekDays;
	R9a02RtcC1Write(R9A02_RTCC1_WALE, 0U, 0U);
	R9a02RtcC1Write(0U, 0U, R9A02_RTCC1_WAFG);
	R9a02ClearPendingInt(s_R9a02RtcDev.IntNo);

	EnableInterrupt(state);

	return true;
}

// WALE off before WALIE, WAFG cleared after the WALE write (20.2.2)
void R9a02RtcDisableAlarm(void)
{
	uintptr_t state = DisableInterrupt();

	R9a02RtcC1Write(0U, R9A02_RTCC1_WALE, 0U);
	R9a02RtcC1Write(0U, R9A02_RTCC1_WALIE, 0U);
	R9a02RtcC1Write(0U, 0U, R9A02_RTCC1_WAFG);
	R9a02ClearPendingInt(s_R9a02RtcDev.IntNo);

	EnableInterrupt(state);
}

void R9a02RtcStop(void)
{
	uintptr_t state = DisableInterrupt();

	R9a02WakeupSet(R9A02_EVT_RTC_ALM_OR_PRD, false);
	R9a02UnregisterIntHandler(s_R9a02RtcDev.IntNo);
	s_R9a02RtcDev.IntNo = -1;
	s_R9a02RtcDev.Handler = NULL;
	R9A02_RTC->RTCC0 = (uint8_t)(R9A02_RTC->RTCC0 & ~R9A02_RTCC0_RTCE);
	R9a02RtcC1Write(0U, R9A02_RTCC1_WALE | R9A02_RTCC1_WALIE | R9A02_RTCC1_RWAIT, R9A02_RTC_FLAGS);

	EnableInterrupt(state);
}
