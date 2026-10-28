// Host regression of the LPC546xx RTC driver with a register model.
//
// Model: the RTC registers need the RTC bus clock. While SWRESET is set, any
// register write other than the CTRL write that clears SWRESET fails the
// test, and that write must change no other CTRL bit. COUNT is written with
// RTC_EN 0. The crystal (when the board has one, and RTC_OSC_PD is 0) gives
// 32768 cycles a second: COUNT counts every 32768 cycles from RTC_EN set
// (the first second 1 s after it), sets ALARM1HZ when it reaches MATCH; the
// WAKE countdown counts every 32 cycles with RTC1KHZ_EN and sets WAKE1KHZ at
// 0. Flags are cleared by writing 1. The interrupt runs when RTC_IRQn is
// enabled and a flag is set. usDelay advances the time.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include "LPC546xx.h"

struct RtcM {
	uint32_t Ctrl = RTC_CTRL_SWRESET_MASK | RTC_CTRL_RTC_OSC_PD_MASK;
	uint32_t Match = 0xFFFFFFFF, Count = 0, Wake = 0, Gpreg[8] = {};
	bool Crystal = true;
	uint64_t Cycles = 0;		// Crystal cycles since RTC_EN set
	int CtrlWrites = 0;
};

static RtcM s_R;
alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
static SYSCON_Type * const s_Sys = (SYSCON_Type *)s_SysconMem;
uint32_t g_IrqMasked = 0;
static uint32_t s_NvicEn = 0, s_NvicPrio[64];
static uint64_t s_Us = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { s_NvicPrio[n] = p; }
void NVIC_EnableIRQ(IRQn_Type n) { s_NvicEn |= 1UL << (n & 31); }
void NVIC_DisableIRQ(IRQn_Type n) { s_NvicEn &= ~(1UL << (n & 31)); }

static void SysconSync(void)
{
	for (int i = 0; i < 3; i++)
	{
		s_Sys->AHBCLKCTRL[i] |= s_Sys->AHBCLKCTRLSET[i];
		s_Sys->AHBCLKCTRL[i] &= ~s_Sys->AHBCLKCTRLCLR[i];
		s_Sys->AHBCLKCTRLSET[i] = 0;
		s_Sys->AHBCLKCTRLCLR[i] = 0;
	}
}

static void Access(bool bWrite)
{
	SysconSync();
	assert(s_Sys->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_RTC_MASK);
	(void)bWrite;
}

static void WriteCheck(void)
{
	Access(true);
	assert((s_R.Ctrl & RTC_CTRL_SWRESET_MASK) == 0);
}

static bool Clocked(void)
{
	return s_R.Crystal && (s_R.Ctrl & (RTC_CTRL_RTC_EN_MASK | RTC_CTRL_RTC_OSC_PD_MASK | RTC_CTRL_SWRESET_MASK)) ==
		   RTC_CTRL_RTC_EN_MASK;
}

struct CtrlR {
	operator uint32_t() const { Access(false); return s_R.Ctrl; }
	CtrlR &operator=(uint32_t v)
	{
		Access(true);
		s_R.CtrlWrites++;
		uint32_t keep = RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK;
		if (s_R.Ctrl & RTC_CTRL_SWRESET_MASK)
		{
			// Only SWRESET cleared, the other bits as they are
			assert((v & ~keep & ~RTC_CTRL_SWRESET_MASK) == (s_R.Ctrl & ~keep & ~RTC_CTRL_SWRESET_MASK));
		}
		bool wasen = (s_R.Ctrl & RTC_CTRL_RTC_EN_MASK) != 0;
		uint32_t flags = s_R.Ctrl & keep & ~v;
		s_R.Ctrl = (v & ~keep) | flags;
		if (!wasen && (s_R.Ctrl & RTC_CTRL_RTC_EN_MASK))
		{
			s_R.Cycles = 0;
		}
		return *this;
	}
};
// Set by a test: the second ends right after the next COUNT read
static bool s_TickOnRead = false;

struct CountR {
	operator uint32_t() const
	{
		Access(false);
		uint32_t c = s_R.Count;
		if (s_TickOnRead)
		{
			s_TickOnRead = false;
			s_R.Count++;
			if (s_R.Count == s_R.Match)
			{
				s_R.Ctrl |= RTC_CTRL_ALARM1HZ_MASK;
			}
		}
		return c;
	}
	CountR &operator=(uint32_t v)
	{
		WriteCheck();
		assert((s_R.Ctrl & RTC_CTRL_RTC_EN_MASK) == 0);
		s_R.Count = v;
		return *this;
	}
};
struct MatchR {
	operator uint32_t() const { Access(false); return s_R.Match; }
	MatchR &operator=(uint32_t v) { WriteCheck(); s_R.Match = v; return *this; }
};
struct WakeR {
	operator uint32_t() const { Access(false); return s_R.Wake; }
	WakeR &operator=(uint32_t v) { WriteCheck(); s_R.Wake = v & 0xFFFFU; return *this; }
};
struct GpregR {
	int Idx = 0;
	operator uint32_t() const { Access(false); return s_R.Gpreg[Idx]; }
	GpregR &operator=(uint32_t v) { WriteCheck(); s_R.Gpreg[Idx] = v; return *this; }
};
struct RtcRegs {
	CtrlR CTRL;
	MatchR MATCH;
	CountR COUNT;
	WakeR WAKE;
	GpregR GPREG[8];
	RtcRegs() { for (int i = 0; i < 8; i++) GPREG[i].Idx = i; }
};
static RtcRegs s_Regs;

#undef SYSCON
#undef RTC
#define SYSCON s_Sys
#define RTC (&s_Regs)

static void Advance(uint64_t Us);
void ModelDelayUs(uint32_t Us) { Advance(Us); }

#include "../../../src/device.cpp"
#include "../../../src/device_intrf.cpp"
#include "../../../src/miscdev/rtc.cpp"
#include "../../../ARM/NXP/LPC546xx/src/rtc_lpc546xx.cpp"

// Crystal cycles of the elapsed time, the counters and the interrupt
static void Advance(uint64_t Us)
{
	uint64_t end = s_Us + Us;
	static uint64_t s_Frac = 0;

	while (s_Us < end)
	{
		s_Us++;
		s_Frac += 32768;
		while (s_Frac >= 1000000)
		{
			s_Frac -= 1000000;
			if (!Clocked())
			{
				continue;
			}
			s_R.Cycles++;
			if (s_R.Cycles % 32768 == 0)
			{
				s_R.Count++;
				if (s_R.Count == s_R.Match)
				{
					s_R.Ctrl |= RTC_CTRL_ALARM1HZ_MASK;
				}
			}
			if ((s_R.Ctrl & RTC_CTRL_RTC1KHZ_EN_MASK) && s_R.Cycles % 32 == 0 && s_R.Wake > 0)
			{
				if (--s_R.Wake == 0)
				{
					s_R.Ctrl |= RTC_CTRL_WAKE1KHZ_MASK;
				}
			}
		}
		if ((s_R.Ctrl & (RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK)) && (s_NvicEn & (1UL << RTC_IRQn)) &&
			g_IrqMasked == 0)
		{
			RTC_IRQHandler();
		}
	}
}

// Power on: RTC in reset, oscillator off, bus clock off
static void PowerOn(bool bCrystal = true)
{
	memset(s_SysconMem, 0, sizeof(s_SysconMem));
	s_R = RtcM();
	s_R.Crystal = bCrystal;
	s_NvicEn = 0;
}

// Reset of the MCU: the RTC keeps running, the bus clock is off
static void SystemReset(void)
{
	memset(s_SysconMem, 0, sizeof(s_SysconMem));
	s_NvicEn = 0;
}

static int s_Evt = 0;
static int s_AlarmCnt = 0;
static int s_WakeCnt = 0;
static uint64_t s_LastWakeUs = 0;
static uint32_t s_AlarmAt[16];

static void Evt(RtcDevice * const pDev, uint32_t E)
{
	(void)pDev;
	s_Evt |= E;
	if (E & RTC_EVT_ALARM0)
	{
		s_AlarmAt[s_AlarmCnt++ & 15] = s_R.Count;
	}
	if (E & RTC_EVT_WAKEUP)
	{
		s_WakeCnt++;
		s_LastWakeUs = s_Us;
	}
}

static RtcCfg_t Cfg(RtcEvtHandler_t H)
{
	RtcCfg_t c = {};
	c.DevNo = 0;
	c.ClkSrc = RTC_CLKSRC_DEFAULT;
	c.IntPrio = 3;
	c.EvtHandler = H;
	return c;
}

static struct tm Tm(int Y, int Mo, int D, int H, int Mi, int S)
{
	struct tm t = {};
	t.tm_year = Y - 1900;
	t.tm_mon = Mo - 1;
	t.tm_mday = D;
	t.tm_hour = H;
	t.tm_min = Mi;
	t.tm_sec = S;
	t.tm_isdst = -1;
	return t;
}

static void SetGet(RtcLpc546xx &R, int Y, int Mo, int D, int H, int Mi, int S, int WDay, int YDay)
{
	struct tm t = Tm(Y, Mo, D, H, Mi, S);
	struct tm g;
	uint32_t us = 123;

	assert(R.SetTime(t));
	assert(R.GetTime(g, &us) && us == 0);
	assert(g.tm_year == Y - 1900 && g.tm_mon == Mo - 1 && g.tm_mday == D && g.tm_hour == H && g.tm_min == Mi &&
		   g.tm_sec == S && g.tm_wday == WDay && g.tm_yday == YDay && g.tm_isdst == -1);
	// The counter is the seconds since the epoch
	assert((time_t)s_R.Count == timegm(&t));
}

int main()
{
	setenv("TZ", "UTC0", 1);
	tzset();

	// Configurations refused
	{
		PowerOn();
		RtcLpc546xx r;
		RtcCfg_t c = Cfg(nullptr);
		c.ClkSrc = RTC_CLKSRC_LFRC;
		assert(!r.Init(c));
		c = Cfg(nullptr);
		c.DevNo = 1;
		assert(!r.Init(c));
		c = Cfg(nullptr);
		assert(!r.Init(c, (DeviceIntrf *)&r));
		struct tm t;
		assert(!r.GetTime(t) && !r.Enable());
	}

	// No crystal: Init fails after 2 s, the oscillator powered down again
	{
		PowerOn(false);
		RtcLpc546xx r;
		RtcCfg_t c = Cfg(nullptr);
		uint64_t t0 = s_Us;
		assert(!r.Init(c));
		assert(s_Us - t0 >= 2000000 && s_Us - t0 < 2100000);
		assert(s_R.Ctrl & RTC_CTRL_RTC_OSC_PD_MASK);
		assert((s_R.Ctrl & RTC_CTRL_RTC_EN_MASK) == 0);
	}

	// Fresh RTC: out of reset, 2000-01-01, calendar not set, events off
	PowerOn();
	{
		RtcLpc546xx r;
		RtcCfg_t c = Cfg(Evt);
		c.ClkSrc = RTC_CLKSRC_LFXTAL;
		assert(r.Init(c));
		assert(r.Enable() && !r.TimeValid() && r.ClockSource() == RTC_CLKSRC_LFXTAL);
		assert(r.ClockFrequency() == 32768 && r.NbAlarm() == 1 && r.NbBackup() == 7 && r.Wakeup() == 0);
		assert(s_R.Count == 946684800U && s_R.Match == 0xFFFFFFFFU && s_R.Gpreg[7] == 0);
		assert((s_R.Ctrl & (RTC_CTRL_SWRESET_MASK | RTC_CTRL_RTC_OSC_PD_MASK | RTC_CTRL_RTC1KHZ_EN_MASK)) == 0);
		assert(s_R.Ctrl & RTC_CTRL_RTC_EN_MASK);
		assert((s_NvicEn & (1UL << RTC_IRQn)) && s_NvicPrio[RTC_IRQn] == 3);
		struct tm g;
		assert(r.GetTime(g) && g.tm_year == 100 && g.tm_mon == 0 && g.tm_mday == 1 && g.tm_wday == 6);

		// Dates of the range, leap days, day of the week and of the year
		SetGet(r, 1970, 1, 1, 0, 0, 0, 4, 0);
		SetGet(r, 2000, 2, 29, 12, 0, 0, 2, 59);
		SetGet(r, 2026, 10, 27, 13, 45, 59, 2, 299);
		SetGet(r, 2024, 12, 31, 23, 59, 59, 2, 365);
		SetGet(r, 2100, 3, 1, 1, 2, 3, 1, 59);
		SetGet(r, 2105, 12, 31, 23, 59, 59, 4, 364);
		assert(r.TimeValid() && s_R.Gpreg[7] == 0x52544356U);

		// Out of range or not a date
		uint32_t cnt = s_R.Count;
		struct tm bad[] = {
			Tm(1969, 12, 31, 23, 59, 59), Tm(2106, 1, 1, 0, 0, 0), Tm(2026, 2, 29, 0, 0, 0),
			Tm(2024, 2, 30, 0, 0, 0), Tm(2026, 4, 31, 0, 0, 0), Tm(2026, 13, 1, 0, 0, 0),
			Tm(2026, 0, 1, 0, 0, 0), Tm(2026, 1, 0, 0, 0, 0), Tm(2026, 1, 1, 24, 0, 0),
			Tm(2026, 1, 1, 0, 60, 0), Tm(2026, 1, 1, 0, 0, 60), Tm(2026, 1, 1, -1, 0, 0),
		};
		for (auto &b : bad)
		{
			assert(!r.SetTime(b));
		}
		assert(s_R.Count == cnt);
		assert(r.SetTime(Tm(2028, 2, 29, 0, 0, 0)));

		// The counter counts from 1 s after RTC_EN
		assert(r.SetTime(Tm(2026, 10, 27, 8, 0, 0)));
		Advance(999000);
		assert(r.GetTime(g) && g.tm_sec == 0);
		Advance(2000);
		assert(r.GetTime(g) && g.tm_sec == 1);
		Advance(2500000);
		assert(r.GetTime(g) && g.tm_sec == 3);

		// Epoch functions of the generic layer
		struct timespec ts;
		assert(r.GetTimeEpoch(ts) && ts.tv_sec == (time_t)s_R.Count && ts.tv_nsec == 0);
		assert(r.SetTimeEpoch(1700000000) && s_R.Count == 1700000000U);

		// Backup registers 0 to 6
		uint32_t v = 0;
		for (int i = 0; i < 7; i++)
		{
			assert(r.WriteBackup(i, 0x1000U + i) && r.ReadBackup(i, v) && v == 0x1000U + (uint32_t)i);
		}
		assert(!r.WriteBackup(7, 1) && !r.ReadBackup(7, v) && !r.WriteBackup(-1, 1));
		assert(s_R.Gpreg[7] == 0x52544356U);

		// Alarm at second 0 of every minute
		RtcAlarm_t a = { 0, -1, -1, -1, -1 };
		assert(r.SetTime(Tm(2026, 10, 27, 12, 0, 30)));
		s_AlarmCnt = 0;
		s_Evt = 0;
		assert(r.SetAlarm(0, a));
		struct tm want = Tm(2026, 10, 27, 12, 1, 0);
		assert(s_R.Match == (uint32_t)timegm(&want));
		assert(s_R.Ctrl & RTC_CTRL_ALARMDPD_EN_MASK);
		Advance(29500000);
		assert(s_AlarmCnt == 0);
		Advance(1000000);
		assert(s_AlarmCnt == 1 && (s_Evt & RTC_EVT_ALARM0));
		assert(s_R.Match == s_AlarmAt[0] + 60 && (s_R.Ctrl & RTC_CTRL_ALARM1HZ_MASK) == 0);
		Advance(120000000);
		assert(s_AlarmCnt == 3 && s_AlarmAt[2] == s_AlarmAt[0] + 120);
		r.DisableAlarm(0);
		assert(s_R.Match == 0xFFFFFFFFU && (s_R.Ctrl & RTC_CTRL_ALARMDPD_EN_MASK) == 0);
		Advance(61000000);
		assert(s_AlarmCnt == 3);

		// Monday 08:00:00 from a Tuesday: 6 days later
		RtcAlarm_t mon = { 0, 0, 8, -1, 1 };
		assert(r.SetTime(Tm(2026, 10, 27, 9, 0, 0)));
		assert(r.SetAlarm(0, mon));
		want = Tm(2026, 11, 2, 8, 0, 0);
		assert(s_R.Match == (uint32_t)timegm(&want));
		// Day 31 from February 1: March 31
		RtcAlarm_t d31 = { 15, 30, 6, 31, -1 };
		assert(r.SetTime(Tm(2027, 2, 1, 0, 0, 0)));
		assert(r.SetAlarm(0, d31));
		want = Tm(2027, 3, 31, 6, 30, 15);
		assert(s_R.Match == (uint32_t)timegm(&want));
		// Same day, later: hour fixed, minute and second free
		RtcAlarm_t h7 = { -1, -1, 7, -1, -1 };
		assert(r.SetTime(Tm(2027, 5, 5, 6, 59, 58)));
		assert(r.SetAlarm(0, h7));
		want = Tm(2027, 5, 5, 7, 0, 0);
		assert(s_R.Match == (uint32_t)timegm(&want));
		// Every second
		RtcAlarm_t all = { -1, -1, -1, -1, -1 };
		assert(r.SetAlarm(0, all) && s_R.Match == s_R.Count + 1);
		// The second ends between the read of the counter and the write of
		// the match value: armed again for the next second, the alarm comes
		s_TickOnRead = true;
		assert(r.SetAlarm(0, all) && s_R.Match == s_R.Count + 1);
		s_AlarmCnt = 0;
		Advance(1100000);
		assert(s_AlarmCnt >= 1);
		// Hour passed today: tomorrow
		RtcAlarm_t h5 = { 0, 0, 5, -1, -1 };
		assert(r.SetAlarm(0, h5));
		want = Tm(2027, 5, 6, 5, 0, 0);
		assert(s_R.Match == (uint32_t)timegm(&want));
		// SetTime moves the alarm to the next time of the new calendar
		assert(r.SetTime(Tm(2027, 5, 8, 4, 0, 0)));
		want = Tm(2027, 5, 8, 5, 0, 0);
		assert(s_R.Match == (uint32_t)timegm(&want));
		// Past the last day that SetTime takes the counter goes on: 2106-01-01
		assert(r.SetTime(Tm(2105, 12, 31, 23, 59, 59)));
		RtcAlarm_t mid = { 0, 0, 0, -1, -1 };
		assert(r.SetAlarm(0, mid) && s_R.Match == 4291747200U);
		// No such time before the end of the counter range, 2106-02-07 06:28:15
		s_R.Count = 0xFFFFFFF0U;
		RtcAlarm_t late = { 0, 0, 7, -1, -1 };
		assert(r.SetAlarm(0, late) && s_R.Match == 0xFFFFFFFFU);
		assert(r.SetAlarm(0, all) && s_R.Match == 0xFFFFFFF1U);
		// Fields refused
		RtcAlarm_t badal[] = {
			{ 60, -1, -1, -1, -1 }, { -2, -1, -1, -1, -1 }, { 0, 60, -1, -1, -1 }, { 0, 0, 24, -1, -1 },
			{ 0, 0, 0, 0, -1 }, { 0, 0, 0, 32, -1 }, { 0, 0, 0, -1, 7 }, { 0, 0, 0, 1, 1 },
		};
		for (auto &b : badal)
		{
			assert(!r.SetAlarm(0, b));
		}
		assert(!r.SetAlarm(1, a));
		r.DisableAlarm(0);

		// Wakeup: rounded to 1.024 kHz counts, started again by the interrupt
		assert(r.SetTime(Tm(2026, 1, 1, 0, 0, 0)));
		assert(r.SetWakeup(1000) && r.Wakeup() == 1000 && s_R.Wake == 1024);
		assert(s_R.Ctrl & (RTC_CTRL_RTC1KHZ_EN_MASK | RTC_CTRL_WAKEDPD_EN_MASK));
		s_WakeCnt = 0;
		uint64_t t0 = s_Us;
		Advance(1000500);
		assert(s_WakeCnt == 1 && s_LastWakeUs - t0 >= 999000 && s_LastWakeUs - t0 <= 1000100);
		Advance(4000000);
		assert(s_WakeCnt == 5);
		assert(r.SetWakeup(250) && r.Wakeup() == 250 && s_R.Wake == 256);
		assert(r.SetWakeup(1) && r.Wakeup() == 1 && s_R.Wake == 1);
		assert(r.SetWakeup(63999) && s_R.Wake == 65535 && r.Wakeup() == 63999);
		assert(!r.SetWakeup(64000) && r.Wakeup() == 63999);
		assert(r.SetWakeup(0) && r.Wakeup() == 0);
		assert((s_R.Ctrl & (RTC_CTRL_RTC1KHZ_EN_MASK | RTC_CTRL_WAKEDPD_EN_MASK)) == 0);
		s_WakeCnt = 0;
		Advance(2000000);
		assert(s_WakeCnt == 0);

		// A flag is not cleared by a write of CTRL for something else
		{
			RtcLpc546xx q;
			assert(q.Init(Cfg(nullptr)) && q.SetWakeup(1));
			Advance(2000);
			assert(s_R.Ctrl & RTC_CTRL_WAKE1KHZ_MASK);
			assert(q.SetAlarm(0, a));
			assert(s_R.Ctrl & RTC_CTRL_WAKE1KHZ_MASK);
			q.DisableAlarm(0);
			assert(s_R.Ctrl & RTC_CTRL_WAKE1KHZ_MASK);
			assert(r.Init(Cfg(Evt)));
		}

		// Disable stops alarm and wakeup, the calendar counts
		assert(r.SetAlarm(0, a) && r.SetWakeup(100));
		uint32_t c0 = s_R.Count;
		r.Disable();
		assert(s_R.Match == 0xFFFFFFFFU && r.Wakeup() == 0 && (s_R.Ctrl & RTC_CTRL_RTC1KHZ_EN_MASK) == 0);
		s_AlarmCnt = 0;
		s_WakeCnt = 0;
		Advance(70000000);
		assert(s_AlarmCnt == 0 && s_WakeCnt == 0 && s_R.Count >= c0 + 69);
		r.Reset();

		// Leave a running RTC with the calendar set for the next case
		assert(r.SetTime(Tm(2026, 10, 27, 10, 0, 0)));
		assert(r.WriteBackup(0, 41));
	}
	// The destructor of the owner turned the interrupt off
	assert((s_NvicEn & (1UL << RTC_IRQn)) == 0);

	// Reset of the MCU: the time and the backup registers are kept
	SystemReset();
	Advance(5000000);
	{
		RtcLpc546xx r;
		RtcCfg_t c = Cfg(nullptr);
		int writes = s_R.CtrlWrites;
		assert(r.Init(c) && r.TimeValid());
		assert(s_R.CtrlWrites - writes <= 1);
		struct tm g;
		assert(r.GetTime(g) && g.tm_hour == 10 && g.tm_min == 0 && g.tm_sec >= 4 && g.tm_sec <= 6);
		uint32_t v;
		assert(r.ReadBackup(0, v) && v == 41);
		assert((s_NvicEn & (1UL << RTC_IRQn)) == 0);

		// The object of the last Init owns the RTC
		RtcLpc546xx r2;
		assert(r2.Init(c));
		assert(!r.SetTime(g) && !r.GetTime(g) && !r.Enable() && !r.WriteBackup(0, 1) && !r.ReadBackup(0, v));
		assert(!r.SetAlarm(0, RtcAlarm_t{ 0, -1, -1, -1, -1 }) && !r.SetWakeup(1000));
		assert(r2.GetTime(g) && r2.Enable());
	}

	// Interrupt without owner: the flags are cleared
	s_R.Ctrl |= RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK;
	SysconSync();
	s_Sys->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_RTC_MASK;
	RTC_IRQHandler();
	assert((s_R.Ctrl & (RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK)) == 0);

	// Enabled but its crystal powered down: started fresh
	PowerOn();
	s_R.Ctrl = RTC_CTRL_RTC_EN_MASK | RTC_CTRL_RTC_OSC_PD_MASK;
	s_R.Count = 5;
	s_Sys->AHBCLKCTRL[0] = SYSCON_AHBCLKCTRL_RTC_MASK;
	{
		RtcLpc546xx r;
		RtcCfg_t c = Cfg(nullptr);
		assert(r.Init(c) && s_R.Count == 946684800U && (s_R.Ctrl & RTC_CTRL_RTC_OSC_PD_MASK) == 0);
	}

	// A stopped RTC that is out of reset starts fresh
	PowerOn();
	s_R.Ctrl = RTC_CTRL_RTC_OSC_PD_MASK;
	s_Sys->AHBCLKCTRL[0] = SYSCON_AHBCLKCTRL_RTC_MASK;
	s_R.Gpreg[7] = 0x52544356U;
	{
		RtcLpc546xx r;
		RtcCfg_t c = Cfg(nullptr);
		assert(r.Init(c) && !r.TimeValid() && s_R.Count == 946684800U);
	}

	printf("lpc546xx rtc: start, calendar, alarm, wakeup, backup, reset, ownership: PASS\n");

	return 0;
}
