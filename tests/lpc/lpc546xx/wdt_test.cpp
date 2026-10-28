// Host regression of the LPC546xx WWDT driver with a register model.
//
// Model: the WWDT registers need the WWDT bus clock (AHBCLKCTRL0), the
// counter needs the watchdog oscillator powered (PDRUNCFG0). MOD: WDEN,
// WDRESET, WDPROTECT and LOCK stay set once set, WDTOF is cleared by writing
// 0, WDINT by writing 1. TC below 0xFF loads 0xFF. FEED: 0xAA then 0x55 with
// no other WWDT access between them, the first feed after WDEN starts the
// counter, a feed while the counter is above WINDOW resets. Each count the
// counter goes down, WDINT is set when it reaches WARNINT, the timeout at 0
// sets WDTOF and resets with WDRESET. SET and CLR registers of SYSCON are
// folded into their registers at each WWDT access.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "LPC546xx.h"

struct WdtM {
	uint32_t Mod = 0, Tc = 0xFF, Tv = 0xFF, Warn = 0, Window = 0xFFFFFF;
	int FeedState = 0;
	bool Running = false;
	bool Reset = false;
	bool WinReset = false;
	int Feeds = 0;
	int Writes = 0;
	int Sync = 0;			// TV reads at 0xFF before the counter shows TC
	int TvReads = 0;
};

static WdtM s_W;
alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
static SYSCON_Type * const s_Sys = (SYSCON_Type *)s_SysconMem;
uint32_t g_IrqMasked = 0;
static uint32_t s_NvicEn = 0, s_NvicPrio[64];

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
	for (int i = 0; i < 2; i++)
	{
		s_Sys->PDRUNCFG[i] |= s_Sys->PDRUNCFGSET[i];
		s_Sys->PDRUNCFG[i] &= ~s_Sys->PDRUNCFGCLR[i];
		s_Sys->PDRUNCFGSET[i] = 0;
		s_Sys->PDRUNCFGCLR[i] = 0;
	}
}

// Any access other than the second feed write between the two feed writes
static void Access(bool bFeed = false)
{
	SysconSync();
	assert(s_Sys->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_WWDT_MASK);
	assert(bFeed || s_W.FeedState == 0);
}

struct ModR {
	operator uint32_t() const { Access(); return s_W.Mod; }
	ModR &operator=(uint32_t v)
	{
		Access();
		s_W.Writes++;
		uint32_t sticky = WWDT_MOD_WDEN_MASK | WWDT_MOD_WDRESET_MASK | WWDT_MOD_WDPROTECT_MASK | WWDT_MOD_LOCK_MASK;
		uint32_t m = (s_W.Mod & sticky) | (v & sticky);
		if (v & WWDT_MOD_WDTOF_MASK)
		{
			m |= s_W.Mod & WWDT_MOD_WDTOF_MASK;
		}
		if ((v & WWDT_MOD_WDINT_MASK) == 0)
		{
			m |= s_W.Mod & WWDT_MOD_WDINT_MASK;
		}
		s_W.Mod = m;
		return *this;
	}
};
struct TcR {
	operator uint32_t() const { Access(); return s_W.Tc; }
	TcR &operator=(uint32_t v)
	{
		Access();
		s_W.Writes++;
		v &= 0xFFFFFFU;
		s_W.Tc = v < 0xFFU ? 0xFFU : v;
		return *this;
	}
};
struct FeedR {
	FeedR &operator=(uint32_t v)
	{
		Access(true);
		// The two writes with interrupts masked
		assert(g_IrqMasked);
		if (v == 0xAA && s_W.FeedState == 0)
		{
			s_W.FeedState = 1;
		}
		else if (v == 0x55 && s_W.FeedState == 1)
		{
			s_W.FeedState = 0;
			if (s_W.Mod & WWDT_MOD_WDEN_MASK)
			{
				if (s_W.Running && s_W.Tv > s_W.Window)
				{
					s_W.Reset = true;
					s_W.WinReset = true;
				}
				if (!s_W.Running)
				{
					// The counter takes TC a few watchdog clocks later
					s_W.Sync = 3;
				}
				s_W.Running = true;
				s_W.Tv = s_W.Tc;
				s_W.Feeds++;
			}
		}
		else
		{
			assert(false);
		}
		return *this;
	}
};
struct TvR {
	operator uint32_t() const
	{
		Access();
		s_W.TvReads++;
		if (s_W.Sync > 0)
		{
			s_W.Sync--;
			return 0xFF;
		}
		return s_W.Tv;
	}
};
struct WarnR {
	operator uint32_t() const { Access(); return s_W.Warn; }
	WarnR &operator=(uint32_t v) { Access(); s_W.Writes++; s_W.Warn = v & 0x3FFU; return *this; }
};
struct WinR {
	operator uint32_t() const { Access(); return s_W.Window; }
	WinR &operator=(uint32_t v) { Access(); s_W.Writes++; s_W.Window = v & 0xFFFFFFU; return *this; }
};
struct WdtRegs {
	ModR MOD;
	TcR TC;
	FeedR FEED;
	TvR TV;
	uint32_t RESERVED;
	WarnR WARNINT;
	WinR WINDOW;
};
static WdtRegs s_Regs;

#undef SYSCON
#undef WWDT
#define SYSCON s_Sys
#define WWDT (&s_Regs)

#include "../../../ARM/NXP/LPC546xx/src/wdt_lpc546xx.cpp"

static int s_Evt = 0;
static WdtDev_t *s_EvtDev = nullptr;

static void Evt(WdtDev_t * const pDev)
{
	s_Evt++;
	s_EvtDev = pDev;
	// The warning comes before the timeout
	assert(s_W.Running && !s_W.Reset);
}

// Count n periods of the counter, the interrupt runs when enabled
static void Count(uint32_t n)
{
	SysconSync();
	for (uint32_t i = 0; i < n && s_W.Running; i++)
	{
		assert((s_Sys->PDRUNCFG[0] & SYSCON_PDRUNCFG_PDEN_WDT_OSC_MASK) == 0);
		s_W.Tv--;
		if (s_W.Tv == s_W.Warn)
		{
			s_W.Mod |= WWDT_MOD_WDINT_MASK;
		}
		if ((s_W.Mod & WWDT_MOD_WDINT_MASK) && (s_NvicEn & (1UL << WDT_BOD_IRQn)) && g_IrqMasked == 0)
		{
			WDT_BOD_IRQHandler();
		}
		if (s_W.Tv == 0)
		{
			s_W.Mod |= WWDT_MOD_WDTOF_MASK;
			if (s_W.Mod & WWDT_MOD_WDRESET_MASK)
			{
				s_W.Reset = true;
				s_W.Running = false;
			}
		}
	}
}

// Power on reset of the model, WDTOSCCTRL at its reset value
static void PowerOn(void)
{
	memset(s_SysconMem, 0, sizeof(s_SysconMem));
	s_Sys->WDTOSCCTRL = 0xA0U;
	s_Sys->PDRUNCFG[0] = SYSCON_PDRUNCFG_PDEN_WDT_OSC_MASK;
	s_W = WdtM();
	s_NvicEn = 0;
	memset(&s_Lpc546xxWdt, 0, sizeof(s_Lpc546xxWdt));
}

static WdtCfg_t Cfg(uint32_t Ms, uint32_t Win, int NbChan, WdtEvtHandler_t Handler)
{
	WdtCfg_t c = {};
	c.DevNo = 0;
	c.msTimeout = Ms;
	c.msWindow = Win;
	c.NbChan = NbChan;
	c.bRunSleep = true;
	c.bRunHalt = false;
	c.IntPrio = 5;
	c.EvtHandler = Handler;
	return c;
}

int main()
{
	WdtDev_t dev;
	WdtCfg_t c;

	// Configurations refused
	PowerOn();
	c = Cfg(2000, 0, 1, nullptr);
	assert(WdtInit(nullptr, &c) == false && WdtInit(&dev, nullptr) == false);
	c.DevNo = 1;
	assert(!WdtInit(&dev, &c));
	c = Cfg(0, 0, 1, nullptr);
	assert(!WdtInit(&dev, &c));
	c = Cfg(2000, 0, 1, nullptr);
	c.bRunSleep = false;
	assert(!WdtInit(&dev, &c));
	c = Cfg(2000, 0, 33, nullptr);
	assert(!WdtInit(&dev, &c));
	c = Cfg(2000, 2000, 1, nullptr);
	assert(!WdtInit(&dev, &c));
	// 0xFFFFFF counts at 125 kHz: 134217.7 ms
	c = Cfg(134218, 0, 1, nullptr);
	assert(!WdtInit(&dev, &c));
	c = Cfg(134217, 0, 1, nullptr);
	assert(WdtInit(&dev, &c) && dev.msTimeout == 134217);
	assert(!WdtStart(nullptr));

	// Init writes no WWDT register, the clock stays off
	PowerOn();
	c = Cfg(2000, 0, 1, nullptr);
	assert(WdtInit(&dev, &c));
	SysconSync();
	assert((s_Sys->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_WWDT_MASK) == 0 && s_W.Writes == 0);
	assert(dev.msTimeout == 2000 && dev.NbChan == 1 && dev.DevNo == 0 && !WdtRunning(&dev));

	// Start: oscillator on, TC 250000 counts, no window, no warning, reset
	// and lock set, fed once
	assert(WdtStart(&dev) && WdtRunning(&dev));
	assert(s_W.Running && s_W.Feeds == 1 && s_W.Tc == 250000 && s_W.Tv == 250000);
	// Start waited for the counter
	assert(s_W.Sync == 0 && s_W.TvReads == 4);
	assert(s_W.Window == 0xFFFFFF && s_W.Warn == 0);
	assert((s_W.Mod & (WWDT_MOD_WDEN_MASK | WWDT_MOD_WDRESET_MASK | WWDT_MOD_LOCK_MASK)) ==
		   (WWDT_MOD_WDEN_MASK | WWDT_MOD_WDRESET_MASK | WWDT_MOD_LOCK_MASK));
	// Started again: nothing changes
	assert(WdtStart(&dev) && s_W.Feeds == 1);

	// Reloads keep it from the reset, then the timeout resets
	for (int i = 0; i < 10; i++)
	{
		Count(62500);
		WdtReload(&dev, 0);
	}
	assert(!s_W.Reset && s_W.Feeds == 11);
	WdtReload(&dev, 1);
	WdtReload(&dev, -1);
	assert(s_W.Feeds == 11);
	Count(249999);
	assert(!s_W.Reset);
	Count(1);
	assert(s_W.Reset && (s_W.Mod & WWDT_MOD_WDTOF_MASK));

	// The smallest timeout is 255 counts, reported rounded up: 2.04 ms
	PowerOn();
	c = Cfg(1, 0, 1, nullptr);
	assert(WdtInit(&dev, &c) && dev.msTimeout == 3);
	assert(WdtStart(&dev) && s_W.Tc == 0xFF);

	// Timeout rounded up to a count: 1001 ms at 125 kHz is 125125 counts
	PowerOn();
	c = Cfg(1001, 0, 1, nullptr);
	assert(WdtInit(&dev, &c) && dev.msTimeout == 1001);
	assert(WdtStart(&dev) && s_W.Tc == 125125);

	// Another oscillator setting: FREQSEL 1 (400 kHz), DIVSEL 31 (/64),
	// counts at 1562 Hz, 2000 ms is 3124 counts, reported 2000 ms
	PowerOn();
	s_Sys->WDTOSCCTRL = SYSCON_WDTOSCCTRL_FREQSEL(1) | SYSCON_WDTOSCCTRL_DIVSEL(31);
	c = Cfg(2000, 0, 1, nullptr);
	assert(WdtInit(&dev, &c) && dev.msTimeout == 2000);
	assert(WdtStart(&dev) && s_W.Tc == 3124);
	assert(s_Sys->WDTOSCCTRL == (SYSCON_WDTOSCCTRL_FREQSEL(1) | SYSCON_WDTOSCCTRL_DIVSEL(31)));

	// Rounding at 1562 Hz: 1001 ms is 1563.562 counts, 1564; a window of
	// 1 ms is 1.562 counts, 1
	PowerOn();
	s_Sys->WDTOSCCTRL = SYSCON_WDTOSCCTRL_FREQSEL(1) | SYSCON_WDTOSCCTRL_DIVSEL(31);
	c = Cfg(1001, 1, 1, nullptr);
	assert(WdtInit(&dev, &c) && dev.msTimeout == 1002);
	assert(WdtStart(&dev) && s_W.Tc == 1564 && s_W.Window == 1563);

	// Longest timeout at 3125 Hz (FREQSEL 1, DIVSEL 15): 5368709 ms is
	// exactly 0x1000000 counts, one too many
	PowerOn();
	s_Sys->WDTOSCCTRL = SYSCON_WDTOSCCTRL_FREQSEL(1) | SYSCON_WDTOSCCTRL_DIVSEL(15);
	c = Cfg(5368709, 0, 1, nullptr);
	assert(!WdtInit(&dev, &c));
	c = Cfg(5368708, 0, 1, nullptr);
	assert(WdtInit(&dev, &c) && WdtStart(&dev) && s_W.Tc == 16777213);

	// FREQSEL 0 selects no frequency, the reset value is set
	PowerOn();
	s_Sys->WDTOSCCTRL = 0;
	c = Cfg(2000, 0, 1, nullptr);
	assert(WdtInit(&dev, &c) && dev.msTimeout == 2000 && s_Sys->WDTOSCCTRL == 0xA0U);

	// Reload channels: fed once every channel is reloaded
	PowerOn();
	c = Cfg(2000, 0, 3, nullptr);
	assert(WdtInit(&dev, &c) && dev.NbChan == 3);
	assert(WdtStart(&dev) && s_W.Feeds == 1);
	WdtReload(&dev, 0);
	WdtReload(&dev, 2);
	WdtReload(&dev, 0);
	assert(s_W.Feeds == 1);
	WdtReload(&dev, 1);
	assert(s_W.Feeds == 2);
	WdtReload(&dev, 1);
	assert(s_W.Feeds == 2);
	WdtReload(&dev, 3);
	assert(s_W.Feeds == 2);

	// 32 channels
	PowerOn();
	c = Cfg(2000, 0, 32, nullptr);
	assert(WdtInit(&dev, &c) && WdtStart(&dev));
	for (int i = 0; i < 31; i++)
	{
		WdtReload(&dev, i);
	}
	assert(s_W.Feeds == 1);
	WdtReload(&dev, 31);
	assert(s_W.Feeds == 2);

	// Window: 500 ms of 2000 ms, WINDOW = 250000 - 62500. A feed sooner
	// resets, a feed after it does not.
	PowerOn();
	c = Cfg(2000, 500, 1, nullptr);
	assert(WdtInit(&dev, &c) && WdtStart(&dev));
	assert(s_W.Window == 187500 && !s_W.Reset);
	Count(62500);
	WdtReload(&dev, 0);
	assert(!s_W.Reset);
	Count(62499);
	WdtReload(&dev, 0);
	assert(s_W.Reset && s_W.WinReset);
	// Window rounded down: 1 ms below a count
	PowerOn();
	c = Cfg(2000, 1, 1, nullptr);
	assert(WdtInit(&dev, &c) && WdtStart(&dev) && s_W.Window == 250000 - 125);

	// Timeout event: the warning 1023 counts before the timeout, the
	// interrupt priority given, the flag cleared, then the reset
	PowerOn();
	s_Evt = 0;
	c = Cfg(2000, 0, 1, Evt);
	assert(WdtInit(&dev, &c));
	assert((s_NvicEn & (1UL << WDT_BOD_IRQn)) && s_NvicPrio[WDT_BOD_IRQn] == 5);
	assert(WdtStart(&dev) && s_W.Warn == 1023);
	Count(250000 - 1024);
	assert(s_Evt == 0);
	Count(1);
	assert(s_Evt == 1 && s_EvtDev == &dev && (s_W.Mod & WWDT_MOD_WDINT_MASK) == 0);
	Count(1022);
	assert(s_Evt == 1 && !s_W.Reset);
	Count(1);
	assert(s_W.Reset);

	// Short timeout: the warning at half of it
	PowerOn();
	c = Cfg(10, 0, 1, Evt);
	assert(WdtInit(&dev, &c) && WdtStart(&dev) && s_W.Tc == 1250 && s_W.Warn == 625);

	// No handler: the interrupt stays off, the vector without warning does
	// nothing
	PowerOn();
	s_Evt = 0;
	c = Cfg(2000, 0, 1, nullptr);
	assert(WdtInit(&dev, &c) && (s_NvicEn & (1UL << WDT_BOD_IRQn)) == 0);
	WDT_BOD_IRQHandler();
	assert(WdtStart(&dev));
	WDT_BOD_IRQHandler();
	assert(s_Evt == 0);

	// Running at Init (a boot loader started it): TC kept and reported, the
	// warning of this Init set, a pending warning cleared, Start keeps it
	PowerOn();
	s_Sys->AHBCLKCTRL[0] = SYSCON_AHBCLKCTRL_WWDT_MASK;
	s_Sys->PDRUNCFG[0] = 0;
	s_W.Tc = 62500;
	s_W.Mod = WWDT_MOD_WDEN_MASK | WWDT_MOD_WDRESET_MASK | WWDT_MOD_WDINT_MASK;
	s_W.Tv = 62500;
	s_W.Running = true;
	s_Evt = 0;
	c = Cfg(5000, 1000, 1, Evt);
	assert(WdtInit(&dev, &c) && WdtRunning(&dev) && dev.msTimeout == 500);
	assert(s_W.Tc == 62500 && s_W.Warn == 1023 && (s_W.Mod & WWDT_MOD_WDINT_MASK) == 0);
	assert(s_W.Window == 0xFFFFFF);
	int feeds = s_W.Feeds;
	assert(WdtStart(&dev) && s_W.Feeds == feeds && s_W.Tc == 62500);
	WdtReload(&dev, 0);
	assert(s_W.Feeds == feeds + 1);
	// Running: FREQSEL 0 is not changed
	PowerOn();
	s_Sys->AHBCLKCTRL[0] = SYSCON_AHBCLKCTRL_WWDT_MASK;
	s_Sys->WDTOSCCTRL = 0;
	s_W.Mod = WWDT_MOD_WDEN_MASK;
	c = Cfg(2000, 0, 1, nullptr);
	assert(!WdtInit(&dev, &c) && s_Sys->WDTOSCCTRL == 0);

	// Reload of a device of an earlier Init is ignored
	PowerOn();
	WdtDev_t dev2;
	c = Cfg(2000, 0, 1, nullptr);
	assert(WdtInit(&dev, &c) && WdtStart(&dev));
	dev2 = dev;
	dev2.pDevData = nullptr;
	WdtReload(&dev2, 0);
	assert(s_W.Feeds == 1 && !WdtStart(&dev2));

	// The C++ class
	PowerOn();
	Wdt w;
	c = Cfg(3000, 0, 1, nullptr);
	assert(w.Init(c) && w.Timeout() == 3000 && !w.Running() && w.Start() && w.Running());
	w.Reload();
	assert(s_W.Feeds == 2);

	printf("lpc546xx wdt: timeout, window, channels, warning event, running at init: PASS\n");

	return 0;
}
