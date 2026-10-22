/**-------------------------------------------------------------------------
@file	rtc_fw.cpp

@brief	STM32 RTC driver test firmware

Runs the production RTC driver, ARM/ST/src/rtc_stm32.cpp, with the time_t
functions of src/miscdev/rtc.cpp, in the emulator of run_rtc.py, one
scenario per run. usDelay (delay_shim/idelay.h) moves the time of the
model. Test registers at 0x5FFF0000, word index:

	0	read: scenario
	1	check result, bit 31 set when it failed
	2	end, number of checks
	3	time passes, usec
	4	read: boot number, 0 then 1 after a system reset
	5	system reset
	6	read: time, usec
	7	read: calendar of the model, seconds since 2000-01-01 00:00:00
	8	read: calendar of the model, usec in the second of the last read of 7
	9	read: RTC clock frequency, Hz, 0 when the calendar does not count
	10	read: number of backup domain resets
	11	read: number of initialization mode entries since the boot
	12	read: 1 RTC write protected
	13	read: RTC clock selection, 0 none, 1 LSE, 2 LSI, 3 HSE
	14	read: bit 0 LSE on, bit 1 LSE ready, bit 2 LSE bypass, bit 3 LSI on,
		bit 4 settings of the LSE register kept, bit 5 RTC clock enabled
	16	read: number of alarms
	17	read: number of backup registers
	18	read: 1 wakeup timer
	19	read: 1 one interrupt for the alarms and the wakeup timer, 2 one each
	20	read: alarm interrupt number
	21	read: wakeup interrupt number
	22	read: LSI frequency after the predivider, Hz
	23	read: board low frequency oscillator, 0 RC, 1 crystal, 2 external
		oscillator
	24	read: 1 RTC clock enable bit in RCC
	25	read: 1 LSI kept over a system reset
	26	RTC base address
	27	RCC base address
	28	PWR base address
	29	EXTI base address
	30	TAMP base address, 0 none
	31	read: LSE start time, usec
	32	read: LSE frequency, Hz

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

#include "stm32.h"
#include "coredev/system_core_clock.h"
#include "miscdev/rtc.h"
#include "idelay.h"

extern "C" void RTC_IRQHandler(void) __attribute__((weak));
extern "C" void RTC_Alarm_IRQHandler(void) __attribute__((weak));
extern "C" void RTC_WKUP_IRQHandler(void) __attribute__((weak));

McuOsc_t g_McuOsc = { { OSC_TYPE_RC, 16000000, 100, 0 }, { OSC_TYPE_XTAL, 32768, 20, 0 }, false };

#pragma pack(push, 4)

typedef struct {
	uint32_t Bits;
	uint32_t Time;
} Evt_t;

#pragma pack(pop)

static volatile uint32_t *const s_Io = (volatile uint32_t *)0x5FFF0000;
static unsigned s_Checks = 0;
static Evt_t s_Evt[32];
static int s_NbEvt = 0;
static RtcDev_t *s_EvtDev = nullptr;

extern "C" void usDelay(uint32_t cnt)
{
	s_Io[3] = cnt;
}

static void Check(bool Ok)
{
	s_Io[1] = ++s_Checks | (Ok ? 0 : 0x80000000U);
}

static void Fail(void)
{
	Check(false);
	while (1)
	{
	}
}

static uint32_t Now(void)
{
	return s_Io[6];
}

static void Event(RtcDev_t * const pDev, uint32_t Evt)
{
	if (s_NbEvt < 32)
	{
		s_Evt[s_NbEvt].Bits = Evt;
		s_Evt[s_NbEvt].Time = Now();
		s_NbEvt++;
	}
	s_EvtDev = pDev;
}

// Pending RTC interrupts serviced, as the core would
static void Service(void)
{
	int n = s_Io[19] == 1 ? 1 : 2;

	for (int i = 0; i < n; i++)
	{
		IRQn_Type irq = (IRQn_Type)s_Io[20 + i];
		void (*handler)(void) = n == 1 ? RTC_IRQHandler : (i == 0 ? RTC_Alarm_IRQHandler : RTC_WKUP_IRQHandler);
		int calls = 0;

		while (NVIC_GetEnableIRQ(irq) && NVIC_GetPendingIRQ(irq))
		{
			NVIC_ClearPendingIRQ(irq);
			if (handler == nullptr || ++calls > 4)
			{
				Fail();
			}
			handler();
		}
	}
}

// Time passes in steps, the interrupts serviced after each
static void Run(uint32_t Us, uint32_t Step = 10000)
{
	while (Us > 0)
	{
		uint32_t d = Us < Step ? Us : Step;

		usDelay(d);
		Us -= d;
		Service();
	}
}

// Event time in [Expect, Expect + Tol]
static bool Near(uint32_t T, uint32_t Expect, uint32_t Tol)
{
	return T >= Expect && T <= Expect + Tol;
}

// Days since 1970-01-01
static int32_t Days(int Y, int M, int D)
{
	Y -= M <= 2 ? 1 : 0;

	int era = Y / 400;
	int yoe = Y - era * 400;
	int doy = (153 * (M + (M > 2 ? -3 : 9)) + 2) / 5 + D - 1;
	int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;

	return era * 146097 + doe - 719468;
}

static int WeekDay(int Y, int M, int D)
{
	return (Days(Y, M, D) + 4) % 7;
}

// Seconds since 2000-01-01
static uint32_t TmSec(const struct tm &T)
{
	return (uint32_t)(Days(T.tm_year + 1900, T.tm_mon + 1, T.tm_mday) - Days(2000, 1, 1)) * 86400U +
		   T.tm_hour * 3600U + T.tm_min * 60U + T.tm_sec;
}

static struct tm Tm(int Y, int M, int D, int h, int m, int s)
{
	struct tm t = {};

	t.tm_year = Y - 1900;
	t.tm_mon = M - 1;
	t.tm_mday = D;
	t.tm_hour = h;
	t.tm_min = m;
	t.tm_sec = s;
	t.tm_isdst = -1;

	return t;
}

static bool Same(const struct tm &T, int Y, int M, int D, int h, int m, int s)
{
	return T.tm_year == Y - 1900 && T.tm_mon == M - 1 && T.tm_mday == D && T.tm_hour == h && T.tm_min == m &&
		   T.tm_sec == s;
}

// Calendar of the model in usec since 2000
static uint64_t ModelTime(void)
{
	uint64_t s = s_Io[7];

	return s * 1000000ULL + s_Io[8];
}

// Expected prescalers: asynchronous division the largest of 1 to 128 that
// divides the clock and leaves at least 1 kHz
static uint32_t ExpectPrer(uint32_t F)
{
	uint32_t a = 1;

	for (uint32_t d = 128; d > 1; d--)
	{
		if (F % d == 0 && F / d >= 1000U)
		{
			a = d;
			break;
		}
	}

	return ((a - 1U) << 16) | (F / a - 1U);
}

// RtcGetTime against the calendar of the model before and after: within one
// sub-second step, day of the week and of the year from the date
static bool ReadCheck(RtcDev_t *pDev, struct tm *pT, uint32_t *pUs = nullptr)
{
	uint32_t res = 1000000U / ((RTC->PRER & RTC_PRER_PREDIV_S) + 1U) + 1U;
	uint64_t before = ModelTime();
	struct tm t = {};
	uint32_t us = 0xFFFFFFFFU;
	bool ok = RtcGetTime(pDev, &t, &us);
	uint64_t after = ModelTime();
	uint64_t r = (uint64_t)TmSec(t) * 1000000ULL + us;
	int y = t.tm_year + 1900;

	ok = ok && us < 1000000U && r + res >= before && r <= after;
	ok = ok && t.tm_wday == WeekDay(y, t.tm_mon + 1, t.tm_mday) && t.tm_isdst == -1 &&
		 t.tm_yday == Days(y, t.tm_mon + 1, t.tm_mday) - Days(y, 1, 1);
	*pT = t;
	if (pUs != nullptr)
	{
		*pUs = us;
	}

	return ok;
}

static RtcCfg_t Cfg(RTC_CLKSRC ClkSrc, RtcEvtHandler_t Handler)
{
	RtcCfg_t cfg = { 0, ClkSrc, 3, Handler };

	return cfg;
}

// RTC interrupts enabled with their priority, or disabled
static bool Irq(bool bEn, uint32_t Prio)
{
	int n = s_Io[19] == 1 ? 1 : 2;

	for (int i = 0; i < n; i++)
	{
		IRQn_Type irq = (IRQn_Type)s_Io[20 + i];

		if ((NVIC_GetEnableIRQ(irq) != 0) != bEn || (bEn && NVIC_GetPriority(irq) != Prio))
		{
			return false;
		}
	}

	return true;
}

static bool FlagsClear(void)
{
#ifdef RTC_SCR_CALRAF
	return (RTC->SR & (RTC_SR_ALRAF | RTC_SR_ALRBF | RTC_SR_WUTF)) == 0;
#else
	uint32_t f = RTC_ISR_ALRAF;
#ifdef RTC_ISR_ALRBF
	f |= RTC_ISR_ALRBF;
#endif
#ifdef RTC_ISR_WUTF
	f |= RTC_ISR_WUTF;
#endif
	return (RTC->ISR & f) == 0;
#endif
}

static uint32_t EventsOn(void)
{
	uint32_t en = RTC_CR_ALRAE | RTC_CR_ALRAIE;

#ifdef RTC_CR_ALRBE
	en |= RTC_CR_ALRBE | RTC_CR_ALRBIE;
#endif
#ifdef RTC_CR_WUTE
	en |= RTC_CR_WUTE | RTC_CR_WUTIE;
#endif
	return RTC->CR & en;
}

static void Alarms(RtcDev_t &Dev)
{
	struct tm t;
	RtcAlarm_t al = { 30, -1, -1, -1, -1 };

	// Alarm 0 each minute at 30 s
	t = Tm(2024, 3, 1, 10, 0, 20);
	Check(RtcSetTime(&Dev, &t));
	uint32_t tset = Now();
	Check(RtcSetAlarm(&Dev, 0, &al) && s_Io[12] == 1);
	Check(RTC->ALRMAR == (RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | 0x30U) &&
		  (RTC->ALRMASSR & RTC_ALRMASSR_MASKSS) == 0 &&
		  (RTC->CR & (RTC_CR_ALRAE | RTC_CR_ALRAIE)) == (RTC_CR_ALRAE | RTC_CR_ALRAIE));
	s_NbEvt = 0;
	s_EvtDev = nullptr;
	Run(15000000);
	Check(s_NbEvt == 1 && s_Evt[0].Bits == RTC_EVT_ALARM0 && Near(s_Evt[0].Time, tset + 10000000, 10200) &&
		  s_EvtDev == &Dev);
	Run(60000000, 100000);
	Check(s_NbEvt == 2 && s_Evt[1].Bits == RTC_EVT_ALARM0 && Near(s_Evt[1].Time, tset + 70000000, 100200));

#ifdef RTC_CR_ALRBE
	// Alarm 1 daily at 10:02:05, alarm 0 still each minute
	t = Tm(2024, 3, 1, 10, 2, 0);
	Check(RtcSetTime(&Dev, &t));
	tset = Now();
	al = { 5, 2, 10, -1, -1 };
	Check(RtcSetAlarm(&Dev, 1, &al) && RTC->ALRMBR == (RTC_ALRMBR_MSK4 | 0x00100205U) && s_Io[12] == 1);
	s_NbEvt = 0;
	Run(10000000);
	Check(s_NbEvt == 1 && s_Evt[0].Bits == RTC_EVT_ALARM1 && Near(s_Evt[0].Time, tset + 5000000, 10200));

	// Both in the same second
	t = Tm(2024, 3, 1, 10, 2, 50);
	Check(RtcSetTime(&Dev, &t));
	al = { 0, -1, -1, -1, -1 };
	Check(RtcSetAlarm(&Dev, 0, &al));
	al = { 0, 3, 10, -1, -1 };
	Check(RtcSetAlarm(&Dev, 1, &al));
	s_NbEvt = 0;
	Run(15000000);
	uint32_t bits = 0;
	for (int i = 0; i < s_NbEvt; i++)
	{
		bits |= s_Evt[i].Bits;
	}
	Check(s_NbEvt >= 1 && s_NbEvt <= 2 && bits == (RTC_EVT_ALARM0 | RTC_EVT_ALARM1) &&
		  s_Evt[s_NbEvt - 1].Time - s_Evt[0].Time < 10000);
	// Out of range: alarm B kept
	RtcDisableAlarm(&Dev, 2);
	RtcDisableAlarm(&Dev, -1);
	Check((RTC->CR & (RTC_CR_ALRBE | RTC_CR_ALRBIE)) == (RTC_CR_ALRBE | RTC_CR_ALRBIE));
	RtcDisableAlarm(&Dev, 1);
	Check((RTC->CR & (RTC_CR_ALRBE | RTC_CR_ALRBIE)) == 0 && (RTC->CR & RTC_CR_ALRAE) && s_Io[12] == 1);

	// A flag left by the previous alarm B: cleared by the new one
	al = { -1, -1, -1, -1, -1 };
	Check(RtcSetAlarm(&Dev, 1, &al));
	usDelay(1100000);
	t = Tm(2024, 3, 1, 10, 4, 0);
	Check(RtcSetTime(&Dev, &t));
	al = { 30, 4, 10, -1, -1 };
	Check(RtcSetAlarm(&Dev, 1, &al));
	s_NbEvt = 0;
	Run(2000000);
	Check(s_NbEvt == 0);
	RtcDisableAlarm(&Dev, 1);
#endif

	// Day of the month: the 1st at midnight
	t = Tm(2024, 3, 31, 23, 59, 58);
	Check(RtcSetTime(&Dev, &t));
	tset = Now();
	al = { 0, 0, 0, 1, -1 };
	Check(RtcSetAlarm(&Dev, 0, &al) && RTC->ALRMAR == 0x01000000U);
	s_NbEvt = 0;
	Run(5000000);
	Check(s_NbEvt == 1 && s_Evt[0].Bits == RTC_EVT_ALARM0 && Near(s_Evt[0].Time, tset + 2000000, 10200));

	// Day of the week: Tuesday at midnight, from Monday 2024-04-01
	t = Tm(2024, 4, 1, 23, 59, 58);
	Check(RtcSetTime(&Dev, &t));
	tset = Now();
	al = { 0, 0, 0, -1, 2 };
	Check(RtcSetAlarm(&Dev, 0, &al) && RTC->ALRMAR == (RTC_ALRMAR_WDSEL | (2U << 24)));
	s_NbEvt = 0;
	Run(5000000);
	Check(s_NbEvt == 1 && s_Evt[0].Bits == RTC_EVT_ALARM0 && Near(s_Evt[0].Time, tset + 2000000, 10200));

	// Sunday is 7, Wednesday does not match on Tuesday
	al = { 0, 0, 0, -1, 0 };
	Check(RtcSetAlarm(&Dev, 0, &al) && RTC->ALRMAR == (RTC_ALRMAR_WDSEL | (7U << 24)));
	al = { 0, 0, 0, -1, 3 };
	Check(RtcSetAlarm(&Dev, 0, &al));
	t = Tm(2024, 4, 1, 23, 59, 58);
	Check(RtcSetTime(&Dev, &t));
	s_NbEvt = 0;
	Run(5000000, 100000);
	Check(s_NbEvt == 0);

	// Refusals: registers unchanged
	uint32_t ar = RTC->ALRMAR;
	uint32_t cr = RTC->CR;
	static const RtcAlarm_t s_Bad[] = {
		{ 60, -1, -1, -1, -1 }, { -2, -1, -1, -1, -1 }, { 0, 60, -1, -1, -1 }, { 0, -2, -1, -1, -1 },
		{ 0, 0, 24, -1, -1 }, { 0, 0, -2, -1, -1 }, { 0, 0, 0, 0, -1 }, { 0, 0, 0, 32, -1 },
		{ 0, 0, 0, -2, -1 }, { 0, 0, 0, -1, 7 }, { 0, 0, 0, -1, -2 }, { 0, 0, 0, 5, 1 },
	};
	for (const RtcAlarm_t &b : s_Bad)
	{
		Check(!RtcSetAlarm(&Dev, 0, &b));
	}
	al = { 0, -1, -1, -1, -1 };
	Check(!RtcSetAlarm(&Dev, -1, &al) && !RtcSetAlarm(&Dev, Dev.NbAlarm, &al) && !RtcSetAlarm(&Dev, 0, nullptr) &&
		  RTC->ALRMAR == ar && RTC->CR == cr);

	// A flag left by the previous alarm A: cleared by the new one
	al = { -1, -1, -1, -1, -1 };
	Check(RtcSetAlarm(&Dev, 0, &al));
	usDelay(1100000);
	t = Tm(2024, 3, 1, 10, 5, 0);
	Check(RtcSetTime(&Dev, &t));
	al = { 30, -1, -1, -1, -1 };
	Check(RtcSetAlarm(&Dev, 0, &al));
	s_NbEvt = 0;
	Run(2000000);
	Check(s_NbEvt == 0);
	cr = RTC->CR;

	// Disabled: no more events
	RtcDisableAlarm(&Dev, -1);
	RtcDisableAlarm(&Dev, Dev.NbAlarm);
	Check(RTC->CR == cr);
	al = { -1, -1, -1, -1, -1 };
	Check(RtcSetAlarm(&Dev, 0, &al) && (RTC->ALRMAR & 0x80808080U) == 0x80808080U);
	s_NbEvt = 0;
	Run(2000000);
	Check(s_NbEvt == 2);
	RtcDisableAlarm(&Dev, 0);
	Check((RTC->CR & (RTC_CR_ALRAE | RTC_CR_ALRAIE)) == 0 && s_Io[12] == 1);
	s_NbEvt = 0;
	Run(3000000, 100000);
	Check(s_NbEvt == 0 && FlagsClear());
}

#ifdef RTC_CR_WUTE
// Wakeup period: msec asked, msec in effect, WUCKSEL, WUTR
typedef struct {
	uint32_t Ms;
	uint32_t Eff;
	uint32_t Sel;
	uint32_t Wut;
} Wk_t;

static bool Wakeup(RtcDev_t &Dev, const Wk_t &W)
{
	return RtcSetWakeup(&Dev, W.Ms) && Dev.msWakeup == W.Eff && (RTC->CR & RTC_CR_WUCKSEL) == W.Sel &&
		   RTC->WUTR == W.Wut && (RTC->CR & (RTC_CR_WUTE | RTC_CR_WUTIE)) == (RTC_CR_WUTE | RTC_CR_WUTIE) &&
		   s_Io[12] == 1;
}

// Periodic events: N in Us, every Period usec from T
static bool Periodic(uint32_t T, uint32_t Period, int N, uint32_t Tol)
{
	if (s_NbEvt != N)
	{
		return false;
	}
	for (int i = 0; i < N; i++)
	{
		if (s_Evt[i].Bits != RTC_EVT_WAKEUP || !Near(s_Evt[i].Time, T + (i + 1) * Period, Tol))
		{
			return false;
		}
	}

	return true;
}
#endif

static void Wakeups(RtcDev_t &Dev)
{
#ifdef RTC_CR_WUTE
	// 32768 Hz: RTCCLK / 2 to 16, then the 1 Hz clock
	static const Wk_t s_Wk[] = {
		{ 1000, 1000, 3, 16383 }, { 250, 250, 3, 4095 }, { 1, 1, 3, 15 }, { 5000, 5000, 2, 40959 },
		{ 32000, 32000, 0, 65535 }, { 32001, 32000, 4, 31 }, { 60000, 60000, 4, 59 },
		{ 65536000, 65536000, 4, 65535 }, { 65537000, 65537000, 6, 0 }, { 131072000, 131072000, 6, 65535 },
		{ 7, 7, 3, 114 },
	};

	for (const Wk_t &w : s_Wk)
	{
		Check(Wakeup(Dev, w));
	}
	Check(RtcSetWakeup(&Dev, 131072000) && !RtcSetWakeup(&Dev, 131072500) && Dev.msWakeup == 131072000 &&
		  (RTC->CR & RTC_CR_WUCKSEL) == 6);

	s_NbEvt = 0;
	Check(Wakeup(Dev, s_Wk[0]));
	uint32_t t = Now();
	Run(3500000);
	Check(Periodic(t, 1000000, 3, 10200));

	s_NbEvt = 0;
	Check(Wakeup(Dev, s_Wk[1]));
	t = Now();
	Run(1100000, 5000);
	Check(Periodic(t, 250000, 4, 5200));

	s_NbEvt = 0;
	Check(Wakeup(Dev, s_Wk[3]));
	t = Now();
	Run(10500000, 50000);
	Check(Periodic(t, 5000000, 2, 50200));

	// A flag left by the previous period: cleared by the new one
	Check(Wakeup(Dev, s_Wk[1]));
	usDelay(300000);
	s_NbEvt = 0;
	Check(Wakeup(Dev, s_Wk[0]));
	t = Now();
	Run(1500000);
	Check(Periodic(t, 1000000, 1, 10200));

	// 1 Hz clock: the first period starts at the next second
	s_NbEvt = 0;
	Check(Wakeup(Dev, s_Wk[6]));
	t = Now();
	Run(125000000, 100000);
	Check(s_NbEvt == 2 && s_Evt[0].Bits == RTC_EVT_WAKEUP && Near(s_Evt[0].Time, t + 59000000, 1100200) &&
		  s_Evt[1].Time - s_Evt[0].Time >= 59900000 && s_Evt[1].Time - s_Evt[0].Time <= 60100000);

	Check(RtcSetWakeup(&Dev, 0) && Dev.msWakeup == 0 && (RTC->CR & (RTC_CR_WUTE | RTC_CR_WUTIE)) == 0 &&
		  s_Io[12] == 1);
	s_NbEvt = 0;
	Run(3000000, 100000);
	Check(s_NbEvt == 0 && FlagsClear());
#else
	Check(!RtcSetWakeup(&Dev, 1000) && RtcSetWakeup(&Dev, 0) && Dev.msWakeup == 0);
#endif
}

// Refusals, new RTC on the LSE, calendar, backup registers, alarms, wakeup
static void Basic(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, Event);
	struct tm t;
	uint32_t v, us;

	Check(!RtcInit(nullptr, &cfg) && !RtcInit(&dev, nullptr));
	cfg.DevNo = 1;
	Check(!RtcInit(&dev, &cfg));
	cfg.DevNo = 0;
	cfg.ClkSrc = (RTC_CLKSRC)3;
	Check(!RtcInit(&dev, &cfg));
	cfg.ClkSrc = RTC_CLKSRC_DEFAULT;

	// Nothing before Init
	RtcDev_t none = {};
	RtcAlarm_t al = { 0, -1, -1, -1, -1 };
	none.NbAlarm = 1;
	none.NbBackup = 1;
	t = Tm(2024, 1, 1, 0, 0, 0);
	Check(!RtcSetTime(&none, &t) && !RtcGetTime(&none, &t, &us) && !RtcSetAlarm(&none, 0, &al) &&
		  !RtcSetWakeup(&none, 0) && !RtcWriteBackup(&none, 0, 1) && !RtcReadBackup(&none, 0, &v));
	RtcDisableAlarm(&none, 0);
	Check(!RtcSetTime(nullptr, &t) && !RtcGetTime(nullptr, &t, &us) && !RtcSetWakeup(nullptr, 0));
	Check(s_Io[13] == 0 && s_Io[11] == 0 && s_Io[12] == 1 && (s_Io[14] & 1) == 0);

	// New RTC: LSE started, 2000-01-01 00:00:00, a Saturday
	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg));
	uint32_t dt = Now() - t0;
	Check(dt >= s_Io[31] && dt < s_Io[31] + 2000);
	Check(dev.DevNo == 0 && dev.ClkSrc == RTC_CLKSRC_LFXTAL && dev.ClkFreq == 32768 && s_Io[9] == 32768 &&
		  s_Io[13] == 1 && (s_Io[14] & 7) == 3 && dev.NbAlarm == (int)s_Io[16] && dev.NbBackup == (int)s_Io[17] &&
		  dev.msWakeup == 0 && !dev.bTimeValid && dev.EvtHandler == Event);
	Check(RTC->PRER == ExpectPrer(32768) && (RTC->CR & (RTC_CR_BYPSHAD | RTC_CR_FMT)) == RTC_CR_BYPSHAD &&
		  EventsOn() == 0 && s_Io[12] == 1 && s_Io[10] == 0 && s_Io[11] == 1 && Irq(true, 3));
	Check(ReadCheck(&dev, &t, &us) && Same(t, 2000, 1, 1, 0, 0, 0) && t.tm_wday == 6 && t.tm_yday == 0);
	Run(2500000, 100000);
	Check(ReadCheck(&dev, &t, &us) && Same(t, 2000, 1, 1, 0, 0, 2) && us >= 400000 && us < 600000);

	// Leap day, month, year and day of the week rollovers
	t = Tm(2024, 2, 29, 23, 59, 58);
	Check(RtcSetTime(&dev, &t) && dev.bTimeValid && s_Io[12] == 1 && s_Io[11] == 2);
	Check(ReadCheck(&dev, &t) && Same(t, 2024, 2, 29, 23, 59, 58) && t.tm_wday == 4 && t.tm_yday == 59);
	Run(3100000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2024, 3, 1, 0, 0, 1) && t.tm_wday == 5 && t.tm_yday == 60);
	Check(((RTC->DR & RTC_DR_WDU) >> RTC_DR_WDU_Pos) == 5);
	t = Tm(2025, 12, 31, 23, 59, 59);
	Check(RtcSetTime(&dev, &t));
	Run(1100000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2026, 1, 1, 0, 0, 0) && t.tm_wday == 4);
	t = Tm(2099, 12, 31, 23, 59, 59);
	Check(RtcSetTime(&dev, &t) && ReadCheck(&dev, &t) && Same(t, 2099, 12, 31, 23, 59, 59) && t.tm_yday == 364);
	t = Tm(2028, 2, 29, 12, 0, 0);
	Check(RtcSetTime(&dev, &t) && ReadCheck(&dev, &t) && Same(t, 2028, 2, 29, 12, 0, 0) && t.tm_wday == 2);
	t = Tm(2028, 12, 31, 0, 0, 0);
	Check(RtcSetTime(&dev, &t) && ReadCheck(&dev, &t) && t.tm_yday == 365);
	t = Tm(2000, 2, 29, 12, 0, 0);
	Check(RtcSetTime(&dev, &t) && ReadCheck(&dev, &t) && Same(t, 2000, 2, 29, 12, 0, 0) && t.tm_wday == 2);

	// Out of range: refused, calendar unchanged
	static const int16_t s_Bad[][6] = {
		{ 1999, 12, 31, 23, 59, 59 }, { 2100, 1, 1, 0, 0, 0 }, { 2023, 2, 29, 0, 0, 0 }, { 2024, 4, 31, 0, 0, 0 },
		{ 2024, 13, 1, 0, 0, 0 }, { 2024, 0, 1, 0, 0, 0 }, { 2024, 1, 0, 0, 0, 0 }, { 2024, 1, 32, 0, 0, 0 },
		{ 2024, 1, 1, 24, 0, 0 }, { 2024, 1, 1, -1, 0, 0 }, { 2024, 1, 1, 0, 60, 0 }, { 2024, 1, 1, 0, -1, 0 },
		{ 2024, 1, 1, 0, 0, 60 }, { 2024, 1, 1, 0, 0, -1 },
	};
	uint32_t n = s_Io[11];
	for (const int16_t *b : s_Bad)
	{
		t = Tm(b[0], b[1], b[2], b[3], b[4], b[5]);
		Check(!RtcSetTime(&dev, &t));
	}
	Check(!RtcSetTime(&dev, nullptr) && !RtcGetTime(&dev, nullptr, &us) && s_Io[11] == n);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 2, 29, 12, 0, 0) && RtcGetTime(&dev, &t, nullptr));

	// Backup registers
	int nb = dev.NbBackup;
	for (int i = 0; i < nb; i++)
	{
		Check(RtcWriteBackup(&dev, i, 0xA5A50000U ^ (i * 0x01010101U)));
	}
	for (int i = 0; i < nb; i++)
	{
		Check(RtcReadBackup(&dev, i, &v) && v == (0xA5A50000U ^ (i * 0x01010101U)));
	}
	Check(!RtcWriteBackup(&dev, nb, 1) && !RtcWriteBackup(&dev, -1, 1) && !RtcReadBackup(&dev, nb, &v) &&
		  !RtcReadBackup(&dev, -1, &v) && !RtcReadBackup(&dev, 0, nullptr));

	Alarms(dev);
	Wakeups(dev);

	// Init again: time kept, events off, flags cleared
	al = { -1, -1, -1, -1, -1 };
	Check(RtcSetAlarm(&dev, 0, &al));
#ifdef RTC_CR_ALRBE
	Check(RtcSetAlarm(&dev, 1, &al));
#endif
#ifdef RTC_CR_WUTE
	Check(RtcSetWakeup(&dev, 250));
#endif
	usDelay(1100000);
	Check(!FlagsClear());
	uint64_t before = ModelTime();
	n = s_Io[11];
	t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 < 1000 && dev.bTimeValid && dev.msWakeup == 0 && EventsOn() == 0 &&
		  FlagsClear() && s_Io[11] == n && s_Io[10] == 0 && Irq(true, 3));
	Check(ReadCheck(&dev, &t) && TmSec(t) >= before / 1000000U);

	// No handler: interrupts off
	cfg.EvtHandler = nullptr;
	cfg.IntPrio = 2;
	Check(RtcInit(&dev, &cfg) && Irq(false, 0) && dev.EvtHandler == nullptr);
	al = { -1, -1, -1, -1, -1 };
	Check(RtcSetAlarm(&dev, 0, &al));
	s_NbEvt = 0;
	Run(3000000, 100000);
	Check(s_NbEvt == 0);
	cfg.EvtHandler = Event;
	Check(RtcInit(&dev, &cfg) && Irq(true, 2));
}

// No crystal: LSE only refused, the default falls back to the LSI
static void NoCrystal(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_LFXTAL, Event);
	struct tm t;
	uint32_t lsi = s_Io[22];

	uint32_t t0 = Now();
	Check(!RtcInit(&dev, &cfg));
	Check(Now() - t0 >= 4000000 && Now() - t0 < 4100000 && s_Io[13] == 0 && (s_Io[14] & 1) == 0);
	cfg.ClkSrc = RTC_CLKSRC_DEFAULT;
	t0 = Now();
	Check(RtcInit(&dev, &cfg));
	Check(Now() - t0 >= 4000000 && Now() - t0 < 4100000 && dev.ClkSrc == RTC_CLKSRC_LFRC && dev.ClkFreq == lsi &&
		  s_Io[13] == 2 && s_Io[9] == lsi && (s_Io[14] & 9) == 8 && !dev.bTimeValid && s_Io[10] == 0);
	Check(RTC->PRER == ExpectPrer(lsi) && ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 0));
	Run(3100000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 3));

	// LSI kept, no LSE start
	cfg.ClkSrc = RTC_CLKSRC_LFRC;
	t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 < 1000 && dev.ClkSrc == RTC_CLKSRC_LFRC && s_Io[10] == 0);
	cfg.ClkSrc = RTC_CLKSRC_DEFAULT;
	t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 < 1000 && dev.ClkSrc == RTC_CLKSRC_LFRC);

#ifdef RTC_CR_WUTE
	// 1000 msec in RTCCLK / 2 counts
	s_NbEvt = 0;
	Check(RtcSetWakeup(&dev, 1000) && dev.msWakeup == 1000 && RTC->WUTR == lsi / 2 - 1 &&
		  (RTC->CR & RTC_CR_WUCKSEL) == 3);
	t0 = Now();
	Run(2500000);
	Check(Periodic(t0, 1000000, 2, 10200));
#endif
}

// System reset: calendar, clock and backup registers kept, events off
static void SystemReset(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, Event);
	struct tm t;
	uint32_t v;

	if (s_Io[4] == 0)
	{
		Check(RtcInit(&dev, &cfg));
		t = Tm(2025, 6, 15, 12, 0, 0);
		Check(RtcSetTime(&dev, &t));
		for (int i = 0; i < dev.NbBackup; i++)
		{
			Check(RtcWriteBackup(&dev, i, 0x5A000000U | i));
		}
		RtcAlarm_t al = { 5, -1, -1, -1, -1 };
		Check(RtcSetAlarm(&dev, 0, &al));
#ifdef RTC_CR_WUTE
		Check(RtcSetWakeup(&dev, 1000));
#endif
		Run(1500000);
		s_Io[5] = 1;
		while (1)
		{
		}
	}

	usDelay(200000);
	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 < 1000);
	Check(dev.bTimeValid && dev.ClkSrc == RTC_CLKSRC_LFXTAL && s_Io[10] == 0 && s_Io[11] == 0 && dev.msWakeup == 0 &&
		  EventsOn() == 0 && FlagsClear() && RTC->PRER == ExpectPrer(32768) && s_Io[12] == 1);
	Check(ReadCheck(&dev, &t) && Same(t, 2025, 6, 15, 12, 0, 1));
	for (int i = 0; i < dev.NbBackup; i++)
	{
		Check(RtcReadBackup(&dev, i, &v) && v == (0x5A000000U | i));
	}
	s_NbEvt = 0;
	Run(10000000, 100000);
	Check(s_NbEvt == 0);
}

// LSI: stopped by the system reset until Init, except on the WBA
static void LsiReset(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_LFRC, nullptr);
	struct tm t;

	if (s_Io[4] == 0)
	{
		uint32_t t0 = Now();
		Check(RtcInit(&dev, &cfg) && Now() - t0 < 1000 && dev.ClkSrc == RTC_CLKSRC_LFRC && s_Io[13] == 2 &&
			  (s_Io[14] & 1) == 0);
		t = Tm(2025, 1, 1, 0, 0, 0);
		Check(RtcSetTime(&dev, &t));
		Run(2100000, 100000);
		s_Io[5] = 1;
		while (1)
		{
		}
	}

	usDelay(1000000);
	cfg.ClkSrc = RTC_CLKSRC_DEFAULT;
	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 < 1000 && dev.ClkSrc == RTC_CLKSRC_LFRC && dev.bTimeValid &&
		  s_Io[10] == 0 && (s_Io[14] & 1) == 0);
	Check(ReadCheck(&dev, &t) && Same(t, 2025, 1, 1, 0, 0, s_Io[25] ? 3 : 2));
	Run(1000000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2025, 1, 1, 0, 0, s_Io[25] ? 4 : 3));
}

// Crystal lost after a system reset: LSE only refused, the default resets
// the backup domain for the LSI, other settings kept
static void CrystalLost(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;
	uint32_t v;

	if (s_Io[4] == 0)
	{
		Check(RtcInit(&dev, &cfg) && dev.ClkSrc == RTC_CLKSRC_LFXTAL && (s_Io[14] & 16));
		t = Tm(2025, 3, 3, 3, 3, 3);
		Check(RtcSetTime(&dev, &t));
		if (dev.NbBackup > 0)
		{
			Check(RtcWriteBackup(&dev, 0, 0x12345678));
		}
		s_Io[5] = 1;
		while (1)
		{
		}
	}

	cfg.ClkSrc = RTC_CLKSRC_LFXTAL;
	uint32_t t0 = Now();
	Check(!RtcInit(&dev, &cfg) && Now() - t0 >= 4000000 && Now() - t0 < 4100000 && s_Io[10] == 0 && s_Io[13] == 1);
	cfg.ClkSrc = RTC_CLKSRC_DEFAULT;
	t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 >= 4000000 && Now() - t0 < 4100000);
	Check(s_Io[10] == 1 && s_Io[13] == 2 && dev.ClkSrc == RTC_CLKSRC_LFRC && dev.ClkFreq == s_Io[22] &&
		  (s_Io[14] & 0x11) == 0x10 && !dev.bTimeValid && RTC->PRER == ExpectPrer(s_Io[22]));
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 0));
	if (dev.NbBackup > 0)
	{
		Check(RtcReadBackup(&dev, 0, &v) && v == 0);
	}
}

// RTC on the HSE, LSE running for other peripherals: the LSI takes a backup
// domain reset, the LSE and the other settings come back
static void ClockChange(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_LFRC, nullptr);
	struct tm t;

	// LSE restart, then the initialization mode: 2 counts of the LSI, 8 msec at
	// 250 Hz on the WBA with LSI1PREDIV
	Check(s_Io[13] == 3 && (s_Io[14] & 0x13) == 0x13);
	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 >= s_Io[31] && Now() - t0 < s_Io[31] + 10000);
	Check(s_Io[10] == 1 && s_Io[13] == 2 && dev.ClkSrc == RTC_CLKSRC_LFRC && (s_Io[14] & 0x33) == 0x33 &&
		  !dev.bTimeValid && RTC->PRER == ExpectPrer(s_Io[22]));
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 0));

	// LSE asked, LSI running: kept
	cfg.ClkSrc = RTC_CLKSRC_LFXTAL;
	Check(RtcInit(&dev, &cfg) && dev.ClkSrc == RTC_CLKSRC_LFRC && s_Io[10] == 1 && s_Io[13] == 2);
}

// RTC running on the LSE from its reset values: time kept, day of the week
// set from the date
static void RunningResetValues(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;

	usDelay(5000000);
	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 < 1000 && s_Io[10] == 0 && s_Io[11] == 1 &&
		  dev.ClkSrc == RTC_CLKSRC_LFXTAL && !dev.bTimeValid && RTC->PRER == 0x007F00FFU);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 5) && t.tm_wday == 6 &&
		  ((RTC->DR & RTC_DR_WDU) >> RTC_DR_WDU_Pos) == 6);
	Run(1100000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 6));
}

// LSE selected with the RTC clock disabled: new calendar, no backup domain
// reset
static void SelectedNotEnabled(void)
{
	if (s_Io[24] == 0)
	{
		return;
	}

	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;

	usDelay(5000000);
	Check((s_Io[14] & 32) == 0);
	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 < 1000 && s_Io[10] == 0 && (s_Io[14] & 32) &&
		  dev.ClkSrc == RTC_CLKSRC_LFXTAL && RTC->PRER == ExpectPrer(32768) && !dev.bTimeValid);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 0) && t.tm_wday == 6);
	Run(2100000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 2));
}

// LSE selected with the RTC clock disabled, LSI asked: backup domain reset,
// the LSE keeps running
static void SelectedNotEnabledLsi(void)
{
	if (s_Io[24] == 0)
	{
		return;
	}

	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_LFRC, nullptr);
	struct tm t;

	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 >= s_Io[31] && Now() - t0 < s_Io[31] + 2000 && s_Io[10] == 1 &&
		  s_Io[13] == 2 && dev.ClkSrc == RTC_CLKSRC_LFRC && (s_Io[14] & 0x23) == 0x23);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 0));
}

// External oscillator on the LSE pin: bypass, frequency of the board
static void Tcxo(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;
	uint32_t f = s_Io[32];

	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 >= s_Io[31] && Now() - t0 < s_Io[31] + 2000);
	Check(dev.ClkSrc == RTC_CLKSRC_LFXTAL && dev.ClkFreq == f && s_Io[9] == f && (s_Io[14] & 7) == 7 &&
		  s_Io[13] == 1 && RTC->PRER == ExpectPrer(f));
	Run(1100000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 1));
}

// RTC on the HSE, LSE running: the LSE takes a backup domain reset and comes
// back
static void HseToLse(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;

	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 >= s_Io[31] && Now() - t0 < s_Io[31] + 2000);
	Check(s_Io[10] == 1 && s_Io[13] == 1 && dev.ClkSrc == RTC_CLKSRC_LFXTAL && (s_Io[14] & 0x33) == 0x33 &&
		  RTC->PRER == ExpectPrer(32768) && !dev.bTimeValid);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 0));
}

// RTC in the 12 hour format: converted to 24 hours
static void Format12(int Hour, int Wday)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;

	Check(RtcInit(&dev, &cfg) && s_Io[11] == 1 && (RTC->CR & RTC_CR_FMT) == 0 && dev.bTimeValid &&
		  RTC->PRER == 0x007F00FFU && s_Io[10] == 0);
	Check(ReadCheck(&dev, &t) && t.tm_hour == Hour && t.tm_wday == Wday && (RTC->TR & RTC_TR_PM) == 0);
	if (Hour == 23)
	{
		Check(Same(t, 2025, 12, 31, 23, 59, 58));
		Run(3100000, 100000);
		Check(ReadCheck(&dev, &t) && Same(t, 2026, 1, 1, 0, 0, 1) && t.tm_wday == 4);
	}
	else
	{
		Check(Same(t, 2025, 6, 1, 0, 30, 0));
		Run(1100000, 100000);
		Check(ReadCheck(&dev, &t) && Same(t, 2025, 6, 1, 0, 30, 1));
	}
}

// Prescalers not giving 1 Hz: rewritten, time kept
static void Prescaler(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;

	Check(RtcInit(&dev, &cfg) && s_Io[11] == 1 && RTC->PRER == ExpectPrer(32768) && dev.bTimeValid && s_Io[10] == 0);
	Check(ReadCheck(&dev, &t) && Same(t, 2025, 5, 5, 5, 5, 5));
	Run(2100000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2025, 5, 5, 5, 5, 7));
}

// WBA binary mode: the calendar restarts in BCD
static void Binary(void)
{
#ifdef RTC_ICSR_BIN
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;

	Check(RtcInit(&dev, &cfg) && (RTC->ICSR & RTC_ICSR_BIN) == 0 && !dev.bTimeValid && RTC->PRER == ExpectPrer(32768));
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 0));
	Run(1100000, 100000);
	Check(ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 1));
#endif
}

// LSI divided by 128, 250 Hz
static void Predivider(void)
{
#if defined(RCC_CSR_LSIPREDIV) || defined(RCC_BDCR1_LSI1PREDIV)
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_LFRC, Event);
	struct tm t;
	uint32_t us;

	Check(s_Io[22] == 250 && RtcInit(&dev, &cfg) && dev.ClkFreq == 250 && RTC->PRER == 249);
	Check(ReadCheck(&dev, &t, &us) && Same(t, 2000, 1, 1, 0, 0, 0));
	// The calendar counts 4 counts (16 msec) after the initialization
	Run(1502000, 100000);
	Check(ReadCheck(&dev, &t, &us) && Same(t, 2000, 1, 1, 0, 0, 1) && us % 4000 == 0 && us >= 480000 && us <= 488000);

#ifdef RTC_CR_WUTE
	// 250 Hz: the shortest period is 2 counts, 8 msec
	static const Wk_t s_Wk[] = {
		{ 1, 8, 3, 0 }, { 1000, 1000, 3, 124 }, { 4194000, 4193984, 0, 65530 }, { 4195000, 4195000, 4, 4194 },
	};
	for (const Wk_t &w : s_Wk)
	{
		Check(Wakeup(dev, w));
	}
	s_NbEvt = 0;
	Check(Wakeup(dev, s_Wk[1]));
	uint32_t t0 = Now();
	Run(3100000);
	Check(Periodic(t0, 1000000, 3, 10200));
#endif
#endif
}

// Reads across the second, time_t functions, C++ class. Board without the
// low frequency crystal (RC): the LSE is tried, 32768 Hz.
static void CalendarRead(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm t;
	uint32_t us, v;

	Check(RtcInit(&dev, &cfg) && dev.ClkSrc == RTC_CLKSRC_LFXTAL && dev.ClkFreq == 32768 &&
		  RTC->PRER == ExpectPrer(32768));
	t = Tm(2030, 7, 4, 8, 0, 0);
	Check(RtcSetTime(&dev, &t));

	// The next second k usec after the start of the read
	for (uint32_t k = 0; k < 12; k++)
	{
		uint64_t now = ModelTime();
		uint32_t left = 1000000U - (uint32_t)(now % 1000000U);

		usDelay(left > k ? left - k : left + 1000000U - k);
		Check(ReadCheck(&dev, &t, &us));
	}

	// Seconds since the epoch, UTC
	struct timespec ts;
	Check(RtcSetTimeEpoch(&dev, 1700000000) && ReadCheck(&dev, &t) && Same(t, 2023, 11, 14, 22, 13, 20) &&
		  t.tm_wday == 2);
	Run(1500000, 100000);
	Check(RtcGetTimeEpoch(&dev, &ts) && ts.tv_sec == 1700000001 && ts.tv_nsec >= 400000000 &&
		  ts.tv_nsec < 600000000 && ts.tv_nsec % 1000 == 0);
	Check(!RtcSetTimeEpoch(&dev, 946684799) && !RtcSetTimeEpoch(&dev, (time_t)4102444800LL));
	Check(RtcSetTimeEpoch(&dev, 946684800) && ReadCheck(&dev, &t) && Same(t, 2000, 1, 1, 0, 0, 0));
	Check(RtcSetTimeEpoch(&dev, (time_t)4102444799LL) && ReadCheck(&dev, &t) && Same(t, 2099, 12, 31, 23, 59, 59));
	Check(!RtcGetTimeEpoch(&dev, nullptr) && !RtcGetTimeEpoch(nullptr, &ts));

	// C++ class, the device of the last Init
	Rtc rtc;
	RtcCfg_t c = Cfg(RTC_CLKSRC_DEFAULT, nullptr);
	struct tm x = Tm(2031, 1, 2, 3, 4, 5);
	RtcAlarm_t al = { 0, -1, -1, -1, -1 };

	Check(rtc.Init(c) && rtc.ClockSource() == RTC_CLKSRC_LFXTAL && rtc.ClockFrequency() == 32768 &&
		  rtc.NbAlarm() == (int)s_Io[16] && rtc.NbBackup() == (int)s_Io[17] && rtc.TimeValid());
	Check(!RtcGetTime(&dev, &t, &us) && (RtcDev_t *)rtc != nullptr);
	Check(rtc.SetTime(x) && rtc.GetTime(x) && Same(x, 2031, 1, 2, 3, 4, 5));
	Check(rtc.SetTime((time_t)1900000000) && rtc.GetTime(ts) && ts.tv_sec == 1900000000);
	Check(rtc.SetAlarm(0, al) && (RTC->CR & RTC_CR_ALRAE));
	rtc.DisableAlarm(0);
	Check((RTC->CR & RTC_CR_ALRAE) == 0 && rtc.SetWakeup(0) && rtc.Wakeup() == 0);
	if (rtc.NbBackup() > 0)
	{
		Check(rtc.WriteBackup(0, 7) && rtc.ReadBackup(0, v) && v == 7);
	}
}

// RTC on the HSE, the LSE stops at the backup domain reset: Init fails, the
// next one takes the LSI
static void LseLost(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_DEFAULT, nullptr);

	uint32_t t0 = Now();
	Check(!RtcInit(&dev, &cfg) && Now() - t0 >= 4000000 && Now() - t0 < 4100000 && s_Io[10] == 1 &&
		  s_Io[13] == 0 && (s_Io[14] & 1) == 0);
	t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 >= 4000000 && Now() - t0 < 4100000 && s_Io[10] == 1 &&
		  s_Io[13] == 2 && dev.ClkSrc == RTC_CLKSRC_LFRC);
}

// RTC on the HSE, the LSE stops at the backup domain reset, LSI asked: the
// LSI runs, the LSE is off
static void LseLostLsi(void)
{
	RtcDev_t dev;
	RtcCfg_t cfg = Cfg(RTC_CLKSRC_LFRC, nullptr);

	uint32_t t0 = Now();
	Check(RtcInit(&dev, &cfg) && Now() - t0 >= 4000000 && Now() - t0 < 4100000 && s_Io[10] == 1 &&
		  s_Io[13] == 2 && dev.ClkSrc == RTC_CLKSRC_LFRC && (s_Io[14] & 1) == 0);
}

extern "C" void ResetHandler()
{
	extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
	uint32_t *s = &_sidata;

	for (uint32_t *d = &_sdata; d < &_edata;)
	{
		*d++ = *s++;
	}
	for (uint32_t *d = &_sbss; d < &_ebss;)
	{
		*d++ = 0;
	}

	s_Io[26] = (uint32_t)RTC;
	s_Io[27] = (uint32_t)RCC;
	s_Io[28] = (uint32_t)PWR;
	s_Io[29] = (uint32_t)EXTI;
#ifdef TAMP
	s_Io[30] = (uint32_t)TAMP;
#endif
	g_McuOsc.LowPwrOsc.Type = s_Io[23] == 2 ? OSC_TYPE_TCXO : (s_Io[23] == 1 ? OSC_TYPE_XTAL : OSC_TYPE_RC);
	g_McuOsc.LowPwrOsc.Freq = s_Io[32];

	switch (s_Io[0])
	{
		case 0:
			Basic();
			break;
		case 1:
			NoCrystal();
			break;
		case 2:
			SystemReset();
			break;
		case 3:
			LsiReset();
			break;
		case 4:
			CrystalLost();
			break;
		case 5:
			ClockChange();
			break;
		case 6:
			RunningResetValues();
			break;
		case 7:
			SelectedNotEnabled();
			break;
		case 8:
			Tcxo();
			break;
		case 9:
			Format12(23, 3);
			break;
		case 10:
			Prescaler();
			break;
		case 11:
			Binary();
			break;
		case 12:
			Predivider();
			break;
		case 13:
			CalendarRead();
			break;
		case 14:
			Format12(0, 0);
			break;
		case 15:
		case 16:
			Tcxo();
			break;
		case 17:
			HseToLse();
			break;
		case 18:
			LseLost();
			break;
		case 19:
			LseLostLsi();
			break;
		case 20:
			SelectedNotEnabledLsi();
			break;
	}

	s_Io[2] = s_Checks;

	while (1)
	{
	}
}
