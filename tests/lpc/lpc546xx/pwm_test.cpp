// Host regression of the LPC546xx PWM driver on SCT0 with a register model.
//
// Model: SCT0 registers need the SCT0 bus clock (AHBCLKCTRL1) and SCT0 out of
// reset (PRESETCTRL1). MATCH, OUTPUT, COUNT and STATE are written only while
// the counter is halted (the SCT bus error of these writes), CONFIG too.
// CLRCTR clears the counter. Each clock of a running counter: the events of
// state 0 whose match equals the count occur, an output with a set and a
// clear event at once follows RES, the output level holds for that count;
// then a LIMIT event clears the counter and loads MATCHREL into MATCH,
// otherwise the counter counts up. The IOCON and GPIO registers are plain
// memory under the pin driver of the port.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "LPC546xx.h"

uint32_t SystemCoreClock = 1000000;
uint32_t g_IrqMasked = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { (void)n; (void)p; }
void NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
void NVIC_DisableIRQ(IRQn_Type n) { (void)n; }

alignas(8) static uint8_t s_IoconMem[sizeof(IOCON_Type)];
alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];
alignas(8) static uint8_t s_PintMem[sizeof(PINT_Type)];
alignas(8) static uint8_t s_InputMuxMem[sizeof(INPUTMUX_Type)];

static uint32_t s_Ahb[3], s_Rst[3];
static int s_Resets = 0;

struct SctM {
	uint32_t Config = 0, Ctrl = SCT_CTRL_HALT_L_MASK, Limit = 0, Output = 0, Res = 0, Count = 0;
	uint32_t Match[10] = {}, MatchRel[10] = {}, EvState[10] = {}, EvCtrl[10] = {}, OutSet[10] = {}, OutClr[10] = {};
};
static SctM s_S;

static void SctReset()
{
	s_S = SctM();
	s_Resets++;
}

enum { AHB_SET, AHB_CLR, RST_SET, RST_CLR };

static void SysWrite(int Kind, int Idx, uint32_t v)
{
	switch (Kind)
	{
		case AHB_SET: s_Ahb[Idx] |= v; break;
		case AHB_CLR: s_Ahb[Idx] &= ~v; break;
		case RST_SET:
			if (Idx == 1 && (v & SYSCON_PRESETCTRL_SCT0_RST_MASK))
			{
				SctReset();
			}
			s_Rst[Idx] |= v;
			break;
		case RST_CLR: s_Rst[Idx] &= ~v; break;
	}
}

struct WReg {
	int Kind, Idx;
	WReg &operator=(uint32_t v) { SysWrite(Kind, Idx, v); return *this; }
};
struct AhbR {
	int Idx;
	operator uint32_t() const { return s_Ahb[Idx]; }
};
struct SysRegs {
	AhbR AHBCLKCTRL[3] = { { 0 }, { 1 }, { 2 } };
	WReg AHBCLKCTRLSET[3] = { { AHB_SET, 0 }, { AHB_SET, 1 }, { AHB_SET, 2 } };
	WReg AHBCLKCTRLCLR[3] = { { AHB_CLR, 0 }, { AHB_CLR, 1 }, { AHB_CLR, 2 } };
	WReg PRESETCTRLSET[3] = { { RST_SET, 0 }, { RST_SET, 1 }, { RST_SET, 2 } };
	WReg PRESETCTRLCLR[3] = { { RST_CLR, 0 }, { RST_CLR, 1 }, { RST_CLR, 2 } };
};
static SysRegs s_SysRegs;

static bool Halted() { return s_S.Ctrl & SCT_CTRL_HALT_L_MASK; }

static void Access()
{
	assert(s_Ahb[1] & SYSCON_AHBCLKCTRL_SCT0_MASK);
	assert((s_Rst[1] & SYSCON_PRESETCTRL_SCT0_RST_MASK) == 0);
}

// Register with a write check
typedef void (*WriteFn)(uint32_t *pReg, uint32_t v);

static void WrAny(uint32_t *pReg, uint32_t v) { *pReg = v; }
static void WrHalted(uint32_t *pReg, uint32_t v) { assert(Halted()); *pReg = v; }

struct Reg {
	uint32_t *p;
	WriteFn Fn;
	operator uint32_t() const { Access(); return *p; }
	Reg &operator=(uint32_t v) { Access(); Fn(p, v); return *this; }
	Reg &operator|=(uint32_t v) { return *this = (uint32_t)*this | v; }
	Reg &operator&=(uint32_t v) { return *this = (uint32_t)*this & v; }
};
struct CtrlR {
	operator uint32_t() const { Access(); return s_S.Ctrl; }
	CtrlR &operator=(uint32_t v)
	{
		Access();
		// PRE and BIDIR unused, CLRCTR reads 0
		assert((v & (SCT_CTRL_PRE_L_MASK | SCT_CTRL_BIDIR_L_MASK)) == 0);
		if (v & SCT_CTRL_CLRCTR_L_MASK)
		{
			s_S.Count = 0;
		}
		s_S.Ctrl = v & ~SCT_CTRL_CLRCTR_L_MASK;
		return *this;
	}
	CtrlR &operator|=(uint32_t v) { return *this = (uint32_t)*this | v; }
	CtrlR &operator&=(uint32_t v) { return *this = (uint32_t)*this & v; }
};
struct EvR {
	Reg STATE, CTRL;
};
struct OutR {
	Reg SET, CLR;
};
struct SctRegs {
	Reg CONFIG = { &s_S.Config, WrHalted };
	CtrlR CTRL;
	Reg LIMIT = { &s_S.Limit, WrAny };
	Reg OUTPUT = { &s_S.Output, WrHalted };
	Reg RES = { &s_S.Res, WrAny };
	Reg MATCH[10] = {
		{ &s_S.Match[0], WrHalted }, { &s_S.Match[1], WrHalted }, { &s_S.Match[2], WrHalted },
		{ &s_S.Match[3], WrHalted }, { &s_S.Match[4], WrHalted }, { &s_S.Match[5], WrHalted },
		{ &s_S.Match[6], WrHalted }, { &s_S.Match[7], WrHalted }, { &s_S.Match[8], WrHalted },
		{ &s_S.Match[9], WrHalted },
	};
	Reg MATCHREL[10] = {
		{ &s_S.MatchRel[0], WrAny }, { &s_S.MatchRel[1], WrAny }, { &s_S.MatchRel[2], WrAny },
		{ &s_S.MatchRel[3], WrAny }, { &s_S.MatchRel[4], WrAny }, { &s_S.MatchRel[5], WrAny },
		{ &s_S.MatchRel[6], WrAny }, { &s_S.MatchRel[7], WrAny }, { &s_S.MatchRel[8], WrAny },
		{ &s_S.MatchRel[9], WrAny },
	};
	EvR EV[10] = {
		{ { &s_S.EvState[0], WrAny }, { &s_S.EvCtrl[0], WrAny } }, { { &s_S.EvState[1], WrAny }, { &s_S.EvCtrl[1], WrAny } },
		{ { &s_S.EvState[2], WrAny }, { &s_S.EvCtrl[2], WrAny } }, { { &s_S.EvState[3], WrAny }, { &s_S.EvCtrl[3], WrAny } },
		{ { &s_S.EvState[4], WrAny }, { &s_S.EvCtrl[4], WrAny } }, { { &s_S.EvState[5], WrAny }, { &s_S.EvCtrl[5], WrAny } },
		{ { &s_S.EvState[6], WrAny }, { &s_S.EvCtrl[6], WrAny } }, { { &s_S.EvState[7], WrAny }, { &s_S.EvCtrl[7], WrAny } },
		{ { &s_S.EvState[8], WrAny }, { &s_S.EvCtrl[8], WrAny } }, { { &s_S.EvState[9], WrAny }, { &s_S.EvCtrl[9], WrAny } },
	};
	OutR OUT[10] = {
		{ { &s_S.OutSet[0], WrAny }, { &s_S.OutClr[0], WrAny } }, { { &s_S.OutSet[1], WrAny }, { &s_S.OutClr[1], WrAny } },
		{ { &s_S.OutSet[2], WrAny }, { &s_S.OutClr[2], WrAny } }, { { &s_S.OutSet[3], WrAny }, { &s_S.OutClr[3], WrAny } },
		{ { &s_S.OutSet[4], WrAny }, { &s_S.OutClr[4], WrAny } }, { { &s_S.OutSet[5], WrAny }, { &s_S.OutClr[5], WrAny } },
		{ { &s_S.OutSet[6], WrAny }, { &s_S.OutClr[6], WrAny } }, { { &s_S.OutSet[7], WrAny }, { &s_S.OutClr[7], WrAny } },
		{ { &s_S.OutSet[8], WrAny }, { &s_S.OutClr[8], WrAny } }, { { &s_S.OutSet[9], WrAny }, { &s_S.OutClr[9], WrAny } },
	};
};
static SctRegs s_SctRegs;

// Clock of the running counter, high counts of each output added to pHigh
static void Clock(uint32_t *pHigh = nullptr)
{
	if (Halted())
	{
		for (int o = 0; pHigh && o < 10; o++)
		{
			pHigh[o] += (s_S.Output >> o) & 1U;
		}
		return;
	}

	assert(s_S.Config == SCT_CONFIG_UNIFY_MASK);

	uint32_t c = s_S.Count;
	uint32_t ev = 0;

	for (int e = 0; e < 10; e++)
	{
		uint32_t ctl = s_S.EvCtrl[e];

		if ((s_S.EvState[e] & 1U) == 0)
		{
			continue;
		}
		// Match only events, no other field
		assert((ctl & ~(SCT_EV_CTRL_MATCHSEL_MASK | SCT_EV_CTRL_COMBMODE_MASK)) == 0);
		assert(((ctl & SCT_EV_CTRL_COMBMODE_MASK) >> SCT_EV_CTRL_COMBMODE_SHIFT) == 1U);
		if (s_S.Match[ctl & SCT_EV_CTRL_MATCHSEL_MASK] == c)
		{
			ev |= 1UL << e;
		}
	}
	for (int o = 0; o < 10; o++)
	{
		bool set = s_S.OutSet[o] & ev;
		bool clr = s_S.OutClr[o] & ev;
		uint32_t bit = 1UL << o;

		if (set && clr)
		{
			switch ((s_S.Res >> (o * 2)) & 3U)
			{
				case 1: s_S.Output |= bit; break;
				case 2: s_S.Output &= ~bit; break;
				case 3: s_S.Output ^= bit; break;
				default: break;
			}
		}
		else if (set)
		{
			s_S.Output |= bit;
		}
		else if (clr)
		{
			s_S.Output &= ~bit;
		}
		if (pHigh)
		{
			pHigh[o] += (s_S.Output >> o) & 1U;
		}
	}
	if (ev & s_S.Limit)
	{
		s_S.Count = 0;
		memcpy(s_S.Match, s_S.MatchRel, sizeof(s_S.Match));
	}
	else
	{
		s_S.Count++;
	}
}

// Clocks until the counter is at 0, the start of a period
static void Align()
{
	assert(!Halted());
	for (int n = 0; s_S.Count != 0; n++)
	{
		assert(n < 10000000);
		Clock();
	}
}

// High counts of each output over one period from count 0
static void Period(uint32_t Top, uint32_t *pHigh)
{
	memset(pHigh, 0, 10 * sizeof(uint32_t));
	Align();
	for (uint32_t i = 0; i < Top; i++)
	{
		Clock(pHigh);
	}
	// The period ends where it began
	assert(s_S.Count == 0);
}

#undef IOCON
#undef GPIO
#undef SYSCON
#undef PINT
#undef INPUTMUX
#undef SCT0
#define IOCON ((IOCON_Type *)s_IoconMem)
#define GPIO ((GPIO_Type *)s_GpioMem)
#define SYSCON (&s_SysRegs)
#define PINT ((PINT_Type *)s_PintMem)
#define INPUTMUX ((INPUTMUX_Type *)s_InputMuxMem)
#define SCT0 (&s_SctRegs)

#include "../../../ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/pwm_lpc546xx.cpp"

static PwmCfg_t Cfg(uint32_t Freq = 1000)
{
	PwmCfg_t c = {};

	c.DevNo = 0;
	c.Freq = Freq;
	c.Mode = PWM_MODE_EDGE;
	return c;
}

static PwmChanCfg_t Chan(int Chan, int Port, int Pin, int Func, PWM_POL Pol)
{
	PwmChanCfg_t c = {};

	c.Chan = Chan;
	c.Pol = Pol;
	c.Port = Port;
	c.Pin = Pin;
	c.PinOp = IOPINOP_FUNC0 + Func;
	return c;
}

static uint32_t Func(int Port, int Pin)
{
	return (IOCON->PIO[Port][Pin] & IOCON_PIO_FUNC_MASK) >> IOCON_PIO_FUNC_SHIFT;
}

// GPIO output, the last write to SET (level 1) or CLR (level 0) has the pin
static bool GpioOut(int Port, int Pin, int Level)
{
	uint32_t bit = 1UL << Pin;

	return Func(Port, Pin) == 0 && (GPIO->DIR[Port] & bit) &&
		   (Level ? (GPIO->SET[Port] & bit) : (GPIO->CLR[Port] & bit));
}

static void GpioReset()
{
	memset(s_GpioMem, 0, sizeof(s_GpioMem));
}

static void ChipReset()
{
	memset(s_IoconMem, 0, sizeof(s_IoconMem));
	GpioReset();
	memset(s_Ahb, 0, sizeof(s_Ahb));
	memset(s_Rst, 0, sizeof(s_Rst));
	s_S = SctM();
	s_Lpc546xxPwm = {};
	SystemCoreClock = 1000000;
}

// LED1 P3_14 SCT0_OUT4 function 1, P3_28 SCT0_OUT2 function 2
#define OUT_LED1	4
#define OUT_J13		2

static void TestInit()
{
	ChipReset();

	PwmDev_t dev = {}, other = {};
	PwmCfg_t c = Cfg();

	assert(!PWMInit(nullptr, &c) && !PWMInit(&dev, nullptr));
	c.DevNo = 1;
	assert(!PWMInit(&dev, &c));
	c = Cfg();
	c.bIntEn = true;
	assert(!PWMInit(&dev, &c));
	c = Cfg();
	c.Mode = PWM_MODE_CENTER;
	assert(!PWMInit(&dev, &c));
	c = Cfg(0);
	assert(!PWMInit(&dev, &c));
	SystemCoreClock = 0;
	c = Cfg();
	assert(!PWMInit(&dev, &c));
	SystemCoreClock = 1000000;
	assert(s_Ahb[1] == 0);

	// SCT0 counting for other code
	s_Ahb[1] = SYSCON_AHBCLKCTRL_SCT0_MASK;
	s_S.Ctrl = 0;
	assert(!PWMInit(&dev, &c));
	s_S.Ctrl = SCT_CTRL_HALT_L_MASK;

	int resets = s_Resets;

	assert(PWMInit(&dev, &c));
	assert(s_Resets == resets + 1);
	assert(dev.Freq == 1000 && dev.DevNo == 0 && dev.Mode == PWM_MODE_EDGE);
	assert(Halted() && s_S.Config == SCT_CONFIG_UNIFY_MASK);
	assert(s_S.Match[0] == 999 && s_S.MatchRel[0] == 999 && s_S.Limit == 1U);
	assert(s_S.EvState[0] == 1U && s_S.EvCtrl[0] == SCT_EV_CTRL_COMBMODE(1));
	assert(s_S.EvState[9] == 1U && s_S.EvCtrl[9] == (SCT_EV_CTRL_MATCHSEL(9) | SCT_EV_CTRL_COMBMODE(1)));
	assert(s_S.Match[9] == 0 && s_S.MatchRel[9] == 0);

	// Closest frequency
	c = Cfg(3);
	assert(PWMInit(&dev, &c) && dev.Freq == 3 && s_S.Match[0] == 333332);
	c = Cfg(600000);
	assert(PWMInit(&dev, &c) && dev.Freq == 500000 && s_S.Match[0] == 1);
	c = Cfg(0xFFFFFFFFU);
	assert(PWMInit(&dev, &c) && dev.Freq == 500000 && s_S.Match[0] == 1);
	c = Cfg(2000001);
	SystemCoreClock = 220000000;
	assert(PWMInit(&dev, &c) && dev.Freq == 2000000 && s_S.Match[0] == 109);
	SystemCoreClock = 1000000;

	// Running from Init by the owner is no obstacle for Init again, which
	// closes the open channels
	c = Cfg();
	PwmChanCfg_t led = Chan(0, 3, 14, 1, PWM_POL_LOW);

	assert(PWMInit(&dev, &c));
	assert(PWMOpenChannel(&dev, &led, 1) && Func(3, 14) == 1);
	assert(PWMStart(&dev, 0) && !Halted());
	GpioReset();
	assert(PWMInit(&dev, &c) && Halted());
	assert(GpioOut(3, 14, 1));

	// Taken over, the channels of the first owner closed
	assert(PWMOpenChannel(&dev, &led, 1) && Func(3, 14) == 1);
	GpioReset();
	assert(PWMInit(&other, &c));
	assert(GpioOut(3, 14, 1));
	assert(!PWMStart(&dev, 0) && !PWMEnable(&dev) && !PWMSetFrequency(&dev, 10) && !PWMSetDutyCycle(&dev, 0, 5));
	PwmChanCfg_t ch = Chan(0, 3, 14, 1, PWM_POL_LOW);
	assert(!PWMOpenChannel(&dev, &ch, 1));
	PWMStop(&dev);
	PWMDisable(&dev);
	PWMCloseChannel(&dev, 0);
	assert(PWMStart(&other, 0));
	assert(!PWMStart(&other, 10));
	printf("init: PASS\n");
}

static void TestOpen()
{
	ChipReset();

	PwmDev_t dev = {};
	PwmCfg_t c = Cfg();
	PwmChanCfg_t ch[3];

	assert(PWMInit(&dev, &c));
	ch[0] = Chan(-1, 3, 14, 1, PWM_POL_LOW);
	assert(!PWMOpenChannel(&dev, ch, 1));
	ch[0] = Chan(8, 3, 14, 1, PWM_POL_LOW);
	assert(!PWMOpenChannel(&dev, ch, 1));
	// No SCT output on the pin, or not with that function
	ch[0] = Chan(0, 0, 0, 1, PWM_POL_LOW);
	assert(!PWMOpenChannel(&dev, ch, 1));
	ch[0] = Chan(0, 3, 14, 2, PWM_POL_LOW);
	assert(!PWMOpenChannel(&dev, ch, 1));
	ch[0] = Chan(0, 3, 14, 1, PWM_POL_LOW);
	ch[0].PinOp = IOPINOP_GPIO;
	assert(!PWMOpenChannel(&dev, ch, 1));
	ch[0] = Chan(0, 3, 14, 1, PWM_POL_LOW);
	assert(!PWMOpenChannel(&dev, nullptr, 1) && !PWMOpenChannel(&dev, ch, 0));
	// Same channel, or two pins of SCT0_OUT4, in one call
	ch[1] = Chan(0, 3, 28, 2, PWM_POL_HIGH);
	assert(!PWMOpenChannel(&dev, ch, 2));
	ch[1] = Chan(1, 0, 23, 4, PWM_POL_HIGH);
	assert(!PWMOpenChannel(&dev, ch, 2));
	assert(s_S.OutSet[OUT_LED1] == 0 && Func(3, 14) == 0);

	ch[1] = Chan(1, 3, 28, 2, PWM_POL_HIGH);
	assert(PWMOpenChannel(&dev, ch, 2));
	assert(Func(3, 14) == 1 && Func(3, 28) == 2);
	assert(IOCON->PIO[3][14] & IOCON_PIO_DIGIMODE_MASK);
	// Inactive while stopped: LED1 active low is high
	assert(s_S.Output == (1UL << OUT_LED1));
	assert(s_S.OutSet[OUT_LED1] == (1UL << 1) && s_S.OutClr[OUT_LED1] == (1UL << 9));
	assert(s_S.OutSet[OUT_J13] == (1UL << 9) && s_S.OutClr[OUT_J13] == (1UL << 2));
	assert(((s_S.Res >> (OUT_LED1 * 2)) & 3U) == 1U && ((s_S.Res >> (OUT_J13 * 2)) & 3U) == 2U);
	assert(s_S.EvState[1] == 1U && s_S.EvCtrl[1] == (SCT_EV_CTRL_MATCHSEL(1) | SCT_EV_CTRL_COMBMODE(1)));
	assert(s_S.EvState[2] == 1U && s_S.EvCtrl[2] == (SCT_EV_CTRL_MATCHSEL(2) | SCT_EV_CTRL_COMBMODE(1)));

	// SCT0_OUT4 open on channel 0: refused on channel 2
	ch[2] = Chan(2, 0, 23, 4, PWM_POL_HIGH);
	assert(!PWMOpenChannel(&dev, &ch[2], 1));
	// Channel 0 to the other SCT0_OUT4 pin, set again in the call
	ch[2] = Chan(0, 0, 23, 4, PWM_POL_HIGH);
	assert(PWMOpenChannel(&dev, &ch[2], 1));
	assert(GpioOut(3, 14, 1) && Func(0, 23) == 4);
	assert(s_S.OutSet[OUT_LED1] == (1UL << 9) && s_S.OutClr[OUT_LED1] == (1UL << 1));
	assert((s_S.Output & (1UL << OUT_LED1)) == 0);

	// Outputs swapped between two channels in one call
	ch[0] = Chan(0, 3, 28, 2, PWM_POL_HIGH);
	ch[1] = Chan(1, 0, 23, 4, PWM_POL_LOW);
	assert(PWMOpenChannel(&dev, ch, 2));
	assert(s_S.OutClr[OUT_J13] == (1UL << 1) && s_S.OutSet[OUT_LED1] == (1UL << 2));

	// Close: pin to GPIO at the inactive level, output and event released
	PWMCloseChannel(&dev, 1);
	assert(GpioOut(0, 23, 1));
	assert(s_S.OutSet[OUT_LED1] == 0 && s_S.OutClr[OUT_LED1] == 0 && s_S.EvState[2] == 0);
	assert(((s_S.Res >> (OUT_LED1 * 2)) & 3U) == 0);
	// SCT0_OUT4 free again
	ch[2] = Chan(3, 3, 14, 1, PWM_POL_LOW);
	assert(PWMOpenChannel(&dev, &ch[2], 1));
	PWMCloseChannel(&dev, 3);
	PWMCloseChannel(&dev, 1);
	PWMCloseChannel(&dev, -1);
	PWMCloseChannel(&dev, 8);
	GpioReset();
	PWMCloseChannel(&dev, 0);
	assert(GpioOut(3, 28, 0));

	// Eight channels at once, duty cycles set before the channels open
	PwmChanCfg_t all[8];
	static const int pins[8][4] = {
		{ 0, 2, 3, 0 }, { 0, 3, 3, 1 }, { 0, 15, 4, 2 }, { 3, 10, 1, 3 }, { 3, 14, 1, 4 },
		{ 3, 18, 5, 5 }, { 2, 2, 3, 6 }, { 2, 22, 2, 7 },
	};

	for (int i = 0; i < 8; i++)
	{
		all[i] = Chan(i, pins[i][0], pins[i][1], pins[i][2], PWM_POL_HIGH);
		assert(PWMSetDutyCycle(&dev, i, 10 * (i + 1)));
	}
	assert(PWMOpenChannel(&dev, all, 8));
	assert(PWMStart(&dev, 0));

	uint32_t high[10];

	// From the first period after Start
	Period(1000, high);
	for (int i = 0; i < 8; i++)
	{
		assert(high[pins[i][3]] == 100U * (i + 1));
	}
	Period(1000, high);
	for (int i = 0; i < 8; i++)
	{
		assert(high[pins[i][3]] == 100U * (i + 1));
		assert(s_S.OutClr[pins[i][3]] == (1UL << (i + 1)));
	}
	assert(high[8] == 0 && high[9] == 0);
	// SCT0_OUT9 also on P0_30, P2_24, P3_13
	PwmChanCfg_t o9 = Chan(0, 3, 13, 1, PWM_POL_HIGH);

	assert(PWMOpenChannel(&dev, &o9, 1));
	assert(GpioOut(0, 2, 0) && Func(3, 13) == 1);
	printf("open: PASS\n");
}

static void TestWave()
{
	ChipReset();

	PwmDev_t dev = {};
	PwmCfg_t c = Cfg();
	PwmChanCfg_t ch[2] = { Chan(0, 3, 14, 1, PWM_POL_LOW), Chan(1, 3, 28, 2, PWM_POL_HIGH) };
	uint32_t high[10];

	assert(PWMInit(&dev, &c));
	assert(PWMOpenChannel(&dev, ch, 2));
	assert(PWMSetDutyCycle(&dev, 0, 50) && PWMSetDutyCycle(&dev, 1, 25));
	// Stopped: the counter does not run, outputs inactive
	for (int i = 0; i < 100; i++)
	{
		Clock();
	}
	assert(s_S.Output == (1UL << OUT_LED1));

	assert(PWMStart(&dev, 0));
	Period(1000, high);
	Period(1000, high);
	// LED1 active low: 500 low counts
	assert(high[OUT_LED1] == 500 && high[OUT_J13] == 250);

	// Duty at the next period
	assert(PWMSetDutyCycle(&dev, 1, 33));
	Period(1000, high);
	assert(high[OUT_J13] == 250);
	Period(1000, high);
	assert(high[OUT_J13] == 330);

	// 0 and 100 percent exactly, on both polarities
	assert(PWMSetDutyCycle(&dev, 0, 0) && PWMSetDutyCycle(&dev, 1, 0));
	Period(1000, high);
	Period(1000, high);
	assert(high[OUT_LED1] == 1000 && high[OUT_J13] == 0);
	assert(PWMSetDutyCycle(&dev, 0, 100) && PWMSetDutyCycle(&dev, 1, 100));
	Period(1000, high);
	Period(1000, high);
	assert(high[OUT_LED1] == 0 && high[OUT_J13] == 1000);
	assert(PWMSetDutyCycle(&dev, 0, -5) && PWMSetDutyCycle(&dev, 1, 150));
	Period(1000, high);
	Period(1000, high);
	assert(high[OUT_LED1] == 1000 && high[OUT_J13] == 1000);
	assert(PWMSetDutyCycle(&dev, 1, 30) && PWMSetDutyCycle(&dev, 1, 300));
	Period(1000, high);
	Period(1000, high);
	assert(high[OUT_J13] == 1000);
	assert(PWMSetDutyCycle(&dev, 1, 1));
	Period(1000, high);
	Period(1000, high);
	assert(high[OUT_J13] == 10);
	assert(PWMSetDutyCycle(&dev, 1, 99));
	Period(1000, high);
	Period(1000, high);
	assert(high[OUT_J13] == 990);
	assert(!PWMSetDutyCycle(&dev, 8, 5) && !PWMSetDutyCycle(&dev, -1, 5));

	// Stop in the active part of a period: inactive and halted, Start again
	for (int i = 0; i < 100; i++)
	{
		Clock();
	}
	assert(s_S.Output & (1UL << OUT_J13));
	PWMStop(&dev);
	assert(Halted() && s_S.Output == (1UL << OUT_LED1));
	assert(PWMSetDutyCycle(&dev, 0, 20) && PWMSetDutyCycle(&dev, 1, 70));
	assert(PWMStart(&dev, 0));
	Period(1000, high);
	Period(1000, high);
	assert(high[OUT_LED1] == 800 && high[OUT_J13] == 700);

	// New frequency at once, the counter beyond the new period
	for (int i = 0; i < 700; i++)
	{
		Clock();
	}
	assert(PWMSetFrequency(&dev, 2000) && dev.Freq == 2000);
	assert(s_S.Count == 0);
	Period(500, high);
	assert(high[OUT_LED1] == 400 && high[OUT_J13] == 350);
	assert(!PWMSetFrequency(&dev, 0) && dev.Freq == 2000);

	// Rounding of a short period: 3 counts, 50 percent is 2 active
	assert(PWMSetFrequency(&dev, 333333) && dev.Freq == 333333);
	assert(PWMSetDutyCycle(&dev, 1, 50) && PWMSetDutyCycle(&dev, 0, 16));
	Period(3, high);
	Period(3, high);
	assert(high[OUT_J13] == 2 && high[OUT_LED1] == 3);

	// Frequency while stopped, loaded at Start
	PWMDisable(&dev);
	assert(Halted());
	assert(PWMSetFrequency(&dev, 500) && dev.Freq == 500);
	assert(Halted() && s_S.Match[0] == 1999 && s_S.Match[2] == 1000);
	assert(PWMSetDutyCycle(&dev, 1, 10) && s_S.Match[2] == 200);
	assert(PWMStart(&dev, 0));
	Period(2000, high);
	Period(2000, high);
	assert(high[OUT_J13] == 200 && high[OUT_LED1] == 2000 - 320);

	// Channel opened while running: inactive until its first period
	PwmChanCfg_t led3 = Chan(2, 2, 2, 3, PWM_POL_LOW);

	assert(PWMSetDutyCycle(&dev, 2, 75));
	for (int i = 0; i < 700; i++)
	{
		Clock();
	}
	assert(PWMOpenChannel(&dev, &led3, 1) && !Halted());
	assert(s_S.Output & (1UL << 6));
	Period(2000, high);
	Period(2000, high);
	assert(high[6] == 500 && high[OUT_J13] == 200);

	// Enable gives the clock back
	SysWrite(AHB_CLR, 1, SYSCON_AHBCLKCTRL_SCT0_MASK);
	assert(PWMEnable(&dev) && (s_Ahb[1] & SYSCON_AHBCLKCTRL_SCT0_MASK));
	printf("wave: PASS\n");
}

int main()
{
	TestInit();
	TestOpen();
	TestWave();
	printf("pwm: PASS\n");
	return 0;
}
