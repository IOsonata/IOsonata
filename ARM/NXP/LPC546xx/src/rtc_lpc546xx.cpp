/**-------------------------------------------------------------------------
@file	rtc_lpc546xx.cpp

@brief	LPC546xx realtime clock (RTC)

The RTC has a 32 bit counter of seconds (COUNT) with a match register
(MATCH, the 1 Hz alarm) and a 16 bit countdown (WAKE) at 1.024 kHz, both on
the 32.768 kHz crystal, and 8 general purpose registers kept with the RTC.
It is reset only by power on and SWRESET, it keeps counting over a reset of
the MCU.

Init. A running RTC (RTC_EN set, out of reset, crystal powered) keeps its
time. Otherwise SWRESET is cleared alone, as the user manual asks, the
counter loaded with 2000-01-01 00:00:00 while RTC_EN is 0, the crystal
powered and the RTC enabled; Init then waits up to 2 s for the wakeup
countdown to move, the sign that the crystal oscillates, and fails without
it. GPREG7 holds a mark written by SetTime: TimeValid is true when it is
there. GPREG0 to 6 are the backup registers.

Calendar: COUNT is the seconds since 1970-01-01 00:00:00, converted with the
day count algorithms of the proleptic Gregorian calendar, 1970 to 2105.
SetTime writes COUNT with RTC_EN cleared, the next second comes 1 s after
RTC_EN is set again. There is no sub-second counter, GetTime gives pUsec 0.

Alarm 0: the fields are matched in software. MATCH takes the next time that
matches them, after the counter; the interrupt sets it again for the next
one. A day of the month that does not come within 400 days, or a time past
the counter range, leaves the alarm off.

Wakeup: the countdown takes the period rounded to a 1.024 kHz count, 1 to
65535 counts (63999 ms). It stops at 0, the interrupt starts it again: each
period is counted from the interrupt that ends the previous one.

The ALARM1HZ and WAKE1KHZ flags of CTRL are cleared by writing 1: every
read modify write of CTRL leaves them out unless it clears them. Their
interrupt is RTC_IRQn. ALARMDPD_EN and WAKEDPD_EN are set with the alarm and
the wakeup, so that they also wake the part from deep power-down.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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

#include "LPC546xx.h"

#include "coredev/interrupt.h"
#include "idelay.h"
#include "rtc_lpc546xx.h"

#define LPC546XX_RTC_FREQ			32768U		// Crystal
#define LPC546XX_RTC_WAKE_FREQ		1024U		// Wakeup countdown, crystal / 32
#define LPC546XX_RTC_WAKE_MAX		0xFFFFU
#define LPC546XX_RTC_MATCH_OFF		0xFFFFFFFFU	// Match value of no alarm, year 2106
#define LPC546XX_RTC_NBBACKUP		7			// GPREG0 to 6
#define LPC546XX_RTC_VALID_REG		7			// GPREG7, calendar set
#define LPC546XX_RTC_VALID			0x52544356U	// 'RTCV'
#define LPC546XX_RTC_FRESH			946684800U	// 2000-01-01 00:00:00
#define LPC546XX_RTC_YEAR_MIN		1970
#define LPC546XX_RTC_YEAR_MAX		2105
#define LPC546XX_RTC_OSC_US			2000000U	// Crystal start, longest wait
#define LPC546XX_RTC_POLL_US		1000U
#define LPC546XX_RTC_SEC_DAY		86400U

// CTRL flags cleared by writing 1, left out of a read modify write
#define LPC546XX_RTC_FLAGS			(RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK)

// Object of the last Init, which owns the RTC
static RtcLpc546xx *s_pLpc546xxRtc = nullptr;

extern "C" void RTC_IRQHandler(void);

bool RtcLpc546xx::Owned() const
{
	return s_pLpc546xxRtc == this;
}

// CTRL write: Set and Clr bits, the flags of W1c cleared, the other flags
// left
static void Lpc546xxRtcCtrl(uint32_t Set, uint32_t Clr, uint32_t W1c)
{
	RTC->CTRL = ((RTC->CTRL & ~(LPC546XX_RTC_FLAGS | Clr)) | Set | W1c);
}

// Seconds counter
static uint32_t Lpc546xxRtcCount(void)
{
	return RTC->COUNT;
}

// Days since 1970-01-01 of a date, Mon 1 to 12
static int32_t Lpc546xxRtcDays(int Year, int Mon, int MDay)
{
	int32_t y = Year - (Mon <= 2 ? 1 : 0);
	int32_t era = (y >= 0 ? y : y - 399) / 400;
	int32_t yoe = y - era * 400;
	int32_t doy = (153 * (Mon + (Mon > 2 ? -3 : 9)) + 2) / 5 + MDay - 1;
	int32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;

	return era * 146097 + doe - 719468;
}

// Date of a day count since 1970-01-01, Mon 1 to 12
static void Lpc546xxRtcDate(int32_t Days, int &Year, int &Mon, int &MDay)
{
	int32_t z = Days + 719468;
	int32_t era = (z >= 0 ? z : z - 146096) / 146097;
	int32_t doe = z - era * 146097;
	int32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	int32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	int32_t mp = (5 * doy + 2) / 153;

	MDay = doy - (153 * mp + 2) / 5 + 1;
	Mon = mp < 10 ? mp + 3 : mp - 9;
	Year = yoe + era * 400 + (Mon <= 2 ? 1 : 0);
}

// Day of the week of a day count, 0 Sunday, 1970-01-01 was a Thursday
static int Lpc546xxRtcWDay(int32_t Days)
{
	return (int)((Days + 4) % 7);
}

// Next time after Now that matches the alarm fields, in seconds. Days are
// searched for 400 days: a day of the month or of the week comes back in
// less.
static bool Lpc546xxRtcAlarmNext(const RtcAlarm_t &A, uint32_t Now, uint32_t &Next)
{
	uint64_t t = (uint64_t)Now + 1U;

	for (int d = 0; d < 400 && t <= LPC546XX_RTC_MATCH_OFF - 1U; d++)
	{
		int32_t days = (int32_t)(t / LPC546XX_RTC_SEC_DAY);
		uint32_t sod = (uint32_t)(t % LPC546XX_RTC_SEC_DAY);
		int year, mon, mday;

		Lpc546xxRtcDate(days, year, mon, mday);
		if ((A.MDay < 0 || mday == A.MDay) && (A.WDay < 0 || Lpc546xxRtcWDay(days) == A.WDay))
		{
			int h0 = (int)(sod / 3600U);
			int m0 = (int)(sod % 3600U / 60U);
			int s0 = (int)(sod % 60U);

			for (int h = h0; h < 24; h++)
			{
				if (A.Hour >= 0 && h != A.Hour)
				{
					continue;
				}
				for (int m = h == h0 ? m0 : 0; m < 60; m++)
				{
					if (A.Min >= 0 && m != A.Min)
					{
						continue;
					}

					int s = h == h0 && m == m0 ? s0 : 0;

					if (A.Sec >= 0)
					{
						if (A.Sec < s)
						{
							continue;
						}
						s = A.Sec;
					}

					uint64_t n = (uint64_t)days * LPC546XX_RTC_SEC_DAY + (uint32_t)(h * 3600 + m * 60 + s);

					if (n >= LPC546XX_RTC_MATCH_OFF)
					{
						return false;
					}
					Next = (uint32_t)n;

					return true;
				}
			}
		}
		t = ((uint64_t)days + 1U) * LPC546XX_RTC_SEC_DAY;
	}

	return false;
}

// Match register for the next alarm time. Done again when the counter has
// reached the match value while it was written.
void RtcLpc546xx::ArmAlarm()
{
	for (int i = 0; i < 3; i++)
	{
		uint32_t next;

		if (Lpc546xxRtcAlarmNext(vAlarm, Lpc546xxRtcCount(), next) == false)
		{
			RTC->MATCH = LPC546XX_RTC_MATCH_OFF;

			return;
		}
		RTC->MATCH = next;
		if (Lpc546xxRtcCount() < next)
		{
			return;
		}
	}
}

// Alarm and wakeup stopped, their flags cleared
static void Lpc546xxRtcEventsOff(void)
{
	RTC->MATCH = LPC546XX_RTC_MATCH_OFF;
	Lpc546xxRtcCtrl(0, RTC_CTRL_RTC1KHZ_EN_MASK | RTC_CTRL_ALARMDPD_EN_MASK | RTC_CTRL_WAKEDPD_EN_MASK,
					LPC546XX_RTC_FLAGS);
}

bool RtcLpc546xx::Init(const RtcCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	// The crystal is the only clock of this RTC
	if (pIntrf != nullptr || Cfg.DevNo != 0 ||
		(Cfg.ClkSrc != RTC_CLKSRC_DEFAULT && Cfg.ClkSrc != RTC_CLKSRC_LFXTAL))
	{
		return false;
	}

	NVIC_DisableIRQ(RTC_IRQn);

	// Bus clock on, the RTC keeps running over a reset of the MCU
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_RTC_MASK;

	uint32_t ctrl = RTC->CTRL;
	bool running = (ctrl & (RTC_CTRL_SWRESET_MASK | RTC_CTRL_RTC_OSC_PD_MASK | RTC_CTRL_RTC_EN_MASK)) ==
				   RTC_CTRL_RTC_EN_MASK;

	if (running == false)
	{
		// Out of reset first, alone in its write
		if (ctrl & RTC_CTRL_SWRESET_MASK)
		{
			RTC->CTRL = ctrl & ~(RTC_CTRL_SWRESET_MASK | LPC546XX_RTC_FLAGS);
		}

		// Stopped while the counter is loaded, the crystal started
		Lpc546xxRtcCtrl(0, RTC_CTRL_RTC_EN_MASK | RTC_CTRL_RTC_OSC_PD_MASK, LPC546XX_RTC_FLAGS);
		RTC->COUNT = LPC546XX_RTC_FRESH;
		RTC->MATCH = LPC546XX_RTC_MATCH_OFF;
		RTC->GPREG[LPC546XX_RTC_VALID_REG] = 0;
		Lpc546xxRtcCtrl(RTC_CTRL_RTC_EN_MASK | RTC_CTRL_RTC1KHZ_EN_MASK, 0, 0);

		// The wakeup countdown moves once the crystal oscillates
		RTC->WAKE = LPC546XX_RTC_WAKE_MAX;

		uint32_t t = 0;

		for (; RTC->WAKE == LPC546XX_RTC_WAKE_MAX; t += LPC546XX_RTC_POLL_US)
		{
			if (t >= LPC546XX_RTC_OSC_US)
			{
				// No crystal: powered down and stopped again
				Lpc546xxRtcCtrl(RTC_CTRL_RTC_OSC_PD_MASK, RTC_CTRL_RTC_EN_MASK | RTC_CTRL_RTC1KHZ_EN_MASK,
								LPC546XX_RTC_FLAGS);
				SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_RTC_MASK;

				return false;
			}
			usDelay(LPC546XX_RTC_POLL_US);
		}
	}

	if (s_pLpc546xxRtc != nullptr && s_pLpc546xxRtc != this)
	{
		// The previous owner keeps nothing
		s_pLpc546xxRtc->vmsWakeup = 0;
	}
	s_pLpc546xxRtc = this;

	vRtcEvtHandler = Cfg.EvtHandler;
	vClkSrc = RTC_CLKSRC_LFXTAL;
	vClkFreq = LPC546XX_RTC_FREQ;
	vNbAlarm = 1;
	vNbBackup = LPC546XX_RTC_NBBACKUP;
	vbTimeValid = RTC->GPREG[LPC546XX_RTC_VALID_REG] == LPC546XX_RTC_VALID;
	vbAlarm = false;
	vWakeTicks = 0;
	vmsWakeup = 0;

	Lpc546xxRtcEventsOff();

	NVIC_ClearPendingIRQ(RTC_IRQn);
	if (Cfg.EvtHandler != nullptr)
	{
		NVIC_SetPriority(RTC_IRQn, Cfg.IntPrio);
		NVIC_EnableIRQ(RTC_IRQn);
	}

	return true;
}

bool RtcLpc546xx::SetTime(const struct tm &Time)
{
	if (Owned() == false)
	{
		return false;
	}

	int year = Time.tm_year + 1900;
	int mon = Time.tm_mon + 1;

	if (year < LPC546XX_RTC_YEAR_MIN || year > LPC546XX_RTC_YEAR_MAX || mon < 1 || mon > 12 ||
		Time.tm_mday < 1 || Time.tm_hour < 0 || Time.tm_hour > 23 || Time.tm_min < 0 || Time.tm_min > 59 ||
		Time.tm_sec < 0 || Time.tm_sec > 59)
	{
		return false;
	}

	int32_t days = Lpc546xxRtcDays(year, mon, Time.tm_mday);
	int32_t next = mon == 12 ? Lpc546xxRtcDays(year + 1, 1, 1) : Lpc546xxRtcDays(year, mon + 1, 1);

	if (days >= next)
	{
		// Past the last day of the month
		return false;
	}

	uint32_t sec = (uint32_t)days * LPC546XX_RTC_SEC_DAY + (uint32_t)(Time.tm_hour * 3600 + Time.tm_min * 60 + Time.tm_sec);
	uint32_t state = DisableInterrupt();

	// The counter is written with the RTC stopped
	Lpc546xxRtcCtrl(0, RTC_CTRL_RTC_EN_MASK, 0);
	RTC->COUNT = sec;
	RTC->GPREG[LPC546XX_RTC_VALID_REG] = LPC546XX_RTC_VALID;
	Lpc546xxRtcCtrl(RTC_CTRL_RTC_EN_MASK, 0, 0);
	vbTimeValid = true;

	if (vbAlarm)
	{
		ArmAlarm();
	}
	EnableInterrupt(state);

	return true;
}

bool RtcLpc546xx::GetTime(struct tm &Time, uint32_t * const pUsec)
{
	if (Owned() == false)
	{
		return false;
	}

	uint32_t sec = Lpc546xxRtcCount();
	int32_t days = (int32_t)(sec / LPC546XX_RTC_SEC_DAY);
	uint32_t sod = sec % LPC546XX_RTC_SEC_DAY;
	int year, mon, mday;

	Lpc546xxRtcDate(days, year, mon, mday);

	Time.tm_year = year - 1900;
	Time.tm_mon = mon - 1;
	Time.tm_mday = mday;
	Time.tm_hour = (int)(sod / 3600U);
	Time.tm_min = (int)(sod % 3600U / 60U);
	Time.tm_sec = (int)(sod % 60U);
	Time.tm_wday = Lpc546xxRtcWDay(days);
	Time.tm_yday = (int)(days - Lpc546xxRtcDays(year, 1, 1));
	Time.tm_isdst = -1;

	if (pUsec != nullptr)
	{
		// No sub-second counter
		*pUsec = 0;
	}

	return true;
}

bool RtcLpc546xx::SetAlarm(int Idx, const RtcAlarm_t &Alarm)
{
	if (Owned() == false || Idx != 0 || Alarm.Sec < -1 || Alarm.Sec > 59 || Alarm.Min < -1 || Alarm.Min > 59 ||
		Alarm.Hour < -1 || Alarm.Hour > 23 || Alarm.MDay < -1 || Alarm.MDay == 0 || Alarm.MDay > 31 ||
		Alarm.WDay < -1 || Alarm.WDay > 6 || (Alarm.MDay >= 0 && Alarm.WDay >= 0))
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	vAlarm = Alarm;
	vbAlarm = true;
	Lpc546xxRtcCtrl(RTC_CTRL_ALARMDPD_EN_MASK, 0, RTC_CTRL_ALARM1HZ_MASK);
	ArmAlarm();
	EnableInterrupt(state);

	return true;
}

void RtcLpc546xx::DisableAlarm(int Idx)
{
	if (Owned() == false || Idx != 0)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	vbAlarm = false;
	RTC->MATCH = LPC546XX_RTC_MATCH_OFF;
	Lpc546xxRtcCtrl(0, RTC_CTRL_ALARMDPD_EN_MASK, RTC_CTRL_ALARM1HZ_MASK);
	EnableInterrupt(state);
}

// The countdown counts at 1.024 kHz, the period is rounded to a count
bool RtcLpc546xx::SetWakeup(uint32_t msPeriod)
{
	if (Owned() == false)
	{
		return false;
	}

	// A count is shorter than 1 ms: 1 ms and up give 1 count or more, and
	// the counts give back msPeriod when rounded to the msec
	uint32_t ticks = (uint32_t)(((uint64_t)msPeriod * LPC546XX_RTC_WAKE_FREQ + 500U) / 1000U);

	if (ticks > LPC546XX_RTC_WAKE_MAX)
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	vWakeTicks = ticks;
	if (ticks == 0)
	{
		vmsWakeup = 0;
		Lpc546xxRtcCtrl(0, RTC_CTRL_RTC1KHZ_EN_MASK | RTC_CTRL_WAKEDPD_EN_MASK, RTC_CTRL_WAKE1KHZ_MASK);
	}
	else
	{
		vmsWakeup = msPeriod;
		Lpc546xxRtcCtrl(RTC_CTRL_RTC1KHZ_EN_MASK | RTC_CTRL_WAKEDPD_EN_MASK, 0, RTC_CTRL_WAKE1KHZ_MASK);
		RTC->WAKE = ticks;
	}
	EnableInterrupt(state);

	return true;
}

bool RtcLpc546xx::WriteBackup(int Idx, uint32_t Data)
{
	if (Owned() == false || Idx < 0 || Idx >= LPC546XX_RTC_NBBACKUP)
	{
		return false;
	}

	RTC->GPREG[Idx] = Data;

	return true;
}

bool RtcLpc546xx::ReadBackup(int Idx, uint32_t &Data)
{
	if (Owned() == false || Idx < 0 || Idx >= LPC546XX_RTC_NBBACKUP)
	{
		return false;
	}

	Data = RTC->GPREG[Idx];

	return true;
}

// The alarm is set for its next time, the wakeup countdown started again
void RtcLpc546xx::IntHandler()
{
	if (Owned() == false)
	{
		return;
	}

	uint32_t ctrl = RTC->CTRL;
	uint32_t evt = 0;

	if (ctrl & RTC_CTRL_ALARM1HZ_MASK)
	{
		Lpc546xxRtcCtrl(0, 0, RTC_CTRL_ALARM1HZ_MASK);
		if (vbAlarm)
		{
			ArmAlarm();
			evt |= RTC_EVT_ALARM0;
		}
	}
	if (ctrl & RTC_CTRL_WAKE1KHZ_MASK)
	{
		Lpc546xxRtcCtrl(0, 0, RTC_CTRL_WAKE1KHZ_MASK);
		if (vWakeTicks != 0)
		{
			RTC->WAKE = vWakeTicks;
			evt |= RTC_EVT_WAKEUP;
		}
	}

	if (evt != 0)
	{
		RtcEvt(evt);
	}
}

bool RtcLpc546xx::Enable()
{
	return Owned();
}

// Alarm and wakeup timer stopped, the calendar counts
void RtcLpc546xx::Disable()
{
	if (Owned() == false)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	vbAlarm = false;
	vWakeTicks = 0;
	vmsWakeup = 0;
	Lpc546xxRtcEventsOff();
	EnableInterrupt(state);
}

void RtcLpc546xx::Reset()
{
	Disable();
}

RtcLpc546xx::~RtcLpc546xx()
{
	if (Owned())
	{
		Disable();
		NVIC_DisableIRQ(RTC_IRQn);
		s_pLpc546xxRtc = nullptr;
	}
}

// Interrupt of the owner, the flags cleared without one
void RTC_IRQHandler(void)
{
	if (s_pLpc546xxRtc != nullptr)
	{
		s_pLpc546xxRtc->IntHandler();
	}
	else
	{
		Lpc546xxRtcCtrl(0, 0, LPC546XX_RTC_FLAGS);
	}
}
