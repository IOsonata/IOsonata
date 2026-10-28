// Host regression of the LPC546xx ADC0 driver with a register model.
//
// Model: the ADC registers need the ADC0 bus clock (AHBCLKCTRL0) and ADC0 out
// of reset (PRESETCTRL0). ADC_ENA is set at least 10 us after ADC0 is powered,
// with VDDA, VD2_ANA and VREFP on, and is cleared when ADC0 powers down.
// ADC_INIT is not set in the write that sets ADC_ENA, it ends after 6 ADC
// clocks. CALREQD reads 1 until a calibration ends since the chip power up,
// a calibration (CALIB) runs 81 ADC clocks at 30 MHz or less. Conversions
// need a calibration or ADC_INIT since ADC0 power up, an ADC clock of 60 MHz
// or less and 12 bits; one takes 15 + TSAMP ADC clocks. SEQ_CTRL: channels
// and trigger change only while SEQ_ENA is 0, START and BURST not in the
// write that sets SEQ_ENA. START runs one pass over the channels in order,
// BURST repeats them back to back until BURST or SEQ_ENA is cleared, the
// conversion in progress then ends. A data register read clears DATAVALID
// and OVERRUN. A pin input converts its voltage when the pin is analog
// (IOCON 0, GPIO input) and the sampling time covers what the test asks,
// otherwise 0xBAD. The temperature sensor (INSEL 3) needs TS powered 100 us
// before, TSAMP 7, and gives its value from the 9th conversion of a burst.
// Erratum ADC.1: all ADC pins are digital while the ADC is set up and when
// ADC0 powers down.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "LPC546xx.h"

uint32_t SystemCoreClock = 220000000;

static uint64_t s_Ns = 0;
static uint64_t s_DelayUs = 0;

alignas(8) static uint8_t s_IoconMem[sizeof(IOCON_Type)];
alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];
static IOCON_Type * const s_Iocon = (IOCON_Type *)s_IoconMem;
static GPIO_Type * const s_Gpio = (GPIO_Type *)s_GpioMem;

static const int s_Pin[12][2] = {
	{ 0, 10 }, { 0, 11 }, { 0, 12 }, { 0, 15 }, { 0, 16 }, { 0, 31 },
	{ 1, 0 }, { 2, 0 }, { 2, 1 }, { 3, 21 }, { 3, 22 }, { 0, 23 },
};

#define SUPPLY		(SYSCON_PDRUNCFG_PDEN_VDDA_MASK | SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK | SYSCON_PDRUNCFG_PDEN_VREFP_MASK)
#define BAD_CODE	0xBADU

enum { CONV_PASS, CONV_BURST };

struct AdcM {
	uint32_t Ctrl = 0, Insel = 0, Seq = 0, Dat[12] = {}, Startup = 0;
	bool Ready = false;			// Calibration or ADC_INIT since ADC0 power up
	bool ChipCal = false;		// Calibration since the chip power up
	int CalCount = 0, InitCount = 0;
	bool CalBusy = false, InitBusy = false;
	uint64_t CalEnd = 0, InitEnd = 0;
	bool Conv = false, StopAfter = false;
	int Mode = CONV_PASS, Ch = 0, BurstCount = 0;
	uint32_t Mask = 0, ConvCtrl = 0, TempCtrl = 0, PinCtrl = 0;
	uint64_t ConvEnd = 0, ConvStart = 0;
	uint64_t AdcOn = 0, TsOn = 0;
	bool StuckConv = false, StuckCal = false;
	uint32_t Code[12] = {};
	uint32_t Inten = 0;			// INTEN, the driver polls
	uint32_t TempCode = 0;
	double MinSampUs[12] = {};
	int Convs[12] = {};
	int Passes = 0, TempConvs = 0, Overruns = 0;
};
static AdcM s_A;

static uint32_t s_Ahb[3], s_Pd[2], s_Rst[3];
static int s_PdDownAdc = 0;

static bool PinAnalog(int Input)
{
	int p = s_Pin[Input][0], n = s_Pin[Input][1];

	return s_Iocon->PIO[p][n] == 0 && (s_Gpio->DIR[p] & (1UL << n)) == 0;
}

static void ErrataAllDigital()
{
	for (int i = 0; i < 12; i++)
	{
		assert(s_Iocon->PIO[s_Pin[i][0]][s_Pin[i][1]] & IOCON_PIO_DIGIMODE_MASK);
	}
}

static uint32_t Div(uint32_t Ctrl) { return (Ctrl & ADC_CTRL_CLKDIV_MASK) + 1U; }
static uint32_t Tsamp(uint32_t Ctrl) { return (Ctrl & ADC_CTRL_TSAMP_MASK) >> ADC_CTRL_TSAMP_SHIFT; }
static double ClkNs(uint32_t Ctrl) { return 1e9 * Div(Ctrl) / SystemCoreClock; }
static double AdcHz(uint32_t Ctrl) { return (double)SystemCoreClock / Div(Ctrl); }

static void StartConv(int Ch, uint64_t Start)
{
	uint32_t c = s_A.Ctrl;

	assert(s_A.Ready && (s_A.Startup & ADC_STARTUP_ADC_ENA_MASK));
	assert(s_A.Inten == 0);
	assert((s_Pd[0] & (SUPPLY | SYSCON_PDRUNCFG_PDEN_ADC0_MASK)) == 0);
	assert(((c & ADC_CTRL_RESOL_MASK) >> ADC_CTRL_RESOL_SHIFT) == 3U);
	assert((c & (ADC_CTRL_ASYNMODE_MASK | ADC_CTRL_BYPASSCAL_MASK)) == 0);
	assert(AdcHz(c) <= 60e6);
	if (Ch == 0 && s_A.Insel == 3U)
	{
		assert((s_Pd[0] & SYSCON_PDRUNCFG_PDEN_TS_MASK) == 0 && Start - s_A.TsOn >= 100000U);
		assert(Tsamp(c) == 7U);
	}
	s_A.Conv = true;
	s_A.Ch = Ch;
	s_A.ConvCtrl = c;
	s_A.ConvStart = Start;
	s_A.ConvEnd = Start + (uint64_t)((15U + Tsamp(c)) * ClkNs(c) + 0.5);
}

static void Complete()
{
	int ch = s_A.Ch;
	uint32_t c = s_A.ConvCtrl;
	uint32_t code;

	s_A.Convs[ch]++;
	if (ch == 0 && s_A.Insel == 3U)
	{
		int k = s_A.Mode == CONV_BURST ? ++s_A.BurstCount : 1;

		s_A.TempConvs++;
		s_A.TempCtrl = c;
		code = k >= 9 ? s_A.TempCode : s_A.TempCode - 300U;
	}
	else
	{
		double samp = (2.5 + Tsamp(c)) * ClkNs(c);

		s_A.PinCtrl = c;

		code = PinAnalog(ch) && samp >= s_A.MinSampUs[ch] * 1000.0 - 1e-3 ? s_A.Code[ch] : BAD_CODE;
	}

	uint32_t ovr = 0;

	if (s_A.Dat[ch] & ADC_DAT_DATAVALID_MASK)
	{
		ovr = ADC_DAT_OVERRUN_MASK;
		s_A.Overruns++;
	}
	s_A.Dat[ch] = ADC_DAT_DATAVALID_MASK | ovr | ADC_DAT_CHANNEL(ch) | ADC_DAT_RESULT(code);
}

static int NextChan()
{
	if (s_A.StopAfter)
	{
		s_A.StopAfter = false;
		if (s_A.Mode == CONV_PASS)
		{
			s_A.Passes++;
		}
		return -1;
	}
	for (int i = s_A.Ch + 1; i < 12; i++)
	{
		if (s_A.Mask & (1UL << i))
		{
			return i;
		}
	}
	if (s_A.Mode == CONV_PASS)
	{
		s_A.Passes++;
		return -1;
	}
	for (int i = 0; i < 12; i++)
	{
		if (s_A.Mask & (1UL << i))
		{
			return i;
		}
	}
	return -1;
}

static void Process()
{
	if (s_A.CalBusy && !s_A.StuckCal && s_Ns >= s_A.CalEnd)
	{
		s_A.CalBusy = false;
		s_A.ChipCal = true;
		s_A.Ready = true;
		s_A.CalCount++;
	}
	if (s_A.InitBusy && s_Ns >= s_A.InitEnd)
	{
		s_A.InitBusy = false;
		s_A.Startup &= ~ADC_STARTUP_ADC_INIT_MASK;
		s_A.Ready = true;
		s_A.InitCount++;
	}
	while (s_A.Conv && !s_A.StuckConv && s_Ns >= s_A.ConvEnd)
	{
		uint64_t t = s_A.ConvEnd;

		Complete();
		s_A.Conv = false;

		int next = NextChan();

		if (next >= 0)
		{
			StartConv(next, t);
		}
	}
}

static void AdcReset()
{
	uint32_t ch = s_A.ChipCal;

	s_A.Ctrl = s_A.Insel = s_A.Seq = s_A.Startup = s_A.Inten = 0;
	memset(s_A.Dat, 0, sizeof(s_A.Dat));
	s_A.Ready = s_A.Conv = s_A.CalBusy = s_A.InitBusy = s_A.StopAfter = false;
	s_A.ChipCal = ch;
}

static void AdcPowerDown()
{
	s_A.Startup = 0;
	s_A.Ready = false;
	s_A.Conv = false;
	s_A.CalBusy = s_A.InitBusy = false;
}

enum { AHB_SET, AHB_CLR, PD_SET, PD_CLR, RST_SET, RST_CLR };

static void SysWrite(int Kind, int Idx, uint32_t v)
{
	Process();
	switch (Kind)
	{
		case AHB_SET: s_Ahb[Idx] |= v; break;
		case AHB_CLR: s_Ahb[Idx] &= ~v; break;
		case PD_SET:
			if (Idx == 0 && (v & SYSCON_PDRUNCFG_PDEN_ADC0_MASK) && !(s_Pd[0] & SYSCON_PDRUNCFG_PDEN_ADC0_MASK))
			{
				ErrataAllDigital();
				assert(!s_A.Conv);
				AdcPowerDown();
				s_PdDownAdc++;
			}
			s_Pd[Idx] |= v;
			break;
		case PD_CLR:
			if (Idx == 0 && (v & SYSCON_PDRUNCFG_PDEN_ADC0_MASK) && (s_Pd[0] & SYSCON_PDRUNCFG_PDEN_ADC0_MASK))
			{
				s_A.AdcOn = s_Ns;
			}
			if (Idx == 0 && (v & SYSCON_PDRUNCFG_PDEN_TS_MASK) && (s_Pd[0] & SYSCON_PDRUNCFG_PDEN_TS_MASK))
			{
				s_A.TsOn = s_Ns;
			}
			s_Pd[Idx] &= ~v;
			break;
		case RST_SET:
			if (Idx == 0 && (v & SYSCON_PRESETCTRL_ADC0_RST_MASK))
			{
				AdcReset();
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
struct PdR {
	int Idx;
	operator uint32_t() const { return s_Pd[Idx]; }
};
struct SysRegs {
	WReg AHBCLKCTRLSET[3] = { { AHB_SET, 0 }, { AHB_SET, 1 }, { AHB_SET, 2 } };
	WReg AHBCLKCTRLCLR[3] = { { AHB_CLR, 0 }, { AHB_CLR, 1 }, { AHB_CLR, 2 } };
	PdR PDRUNCFG[2] = { { 0 }, { 1 } };
	WReg PDRUNCFGSET[2] = { { PD_SET, 0 }, { PD_SET, 1 } };
	WReg PDRUNCFGCLR[2] = { { PD_CLR, 0 }, { PD_CLR, 1 } };
	WReg PRESETCTRLSET[3] = { { RST_SET, 0 }, { RST_SET, 1 }, { RST_SET, 2 } };
	WReg PRESETCTRLCLR[3] = { { RST_CLR, 0 }, { RST_CLR, 1 }, { RST_CLR, 2 } };
};
static SysRegs s_SysRegs;

static void Access()
{
	assert(s_Ahb[0] & SYSCON_AHBCLKCTRL_ADC0_MASK);
	assert((s_Rst[0] & SYSCON_PRESETCTRL_ADC0_RST_MASK) == 0);
	s_Ns += 20;
	Process();
}

struct CtrlR {
	operator uint32_t() const { Access(); return s_A.Ctrl; }
	CtrlR &operator=(uint32_t v)
	{
		Access();
		// Changed only while the ADC is idle
		assert(!s_A.Conv);
		assert(s_A.StuckCal || (!s_A.CalBusy && !s_A.InitBusy));
		s_A.Ctrl = v;
		return *this;
	}
};
struct InselR {
	operator uint32_t() const { Access(); return s_A.Insel; }
	InselR &operator=(uint32_t v) { Access(); assert(!s_A.Conv); s_A.Insel = v & 3U; return *this; }
};
struct SeqR {
	operator uint32_t() const { Access(); return s_A.Seq; }
	SeqR &operator=(uint32_t v)
	{
		Access();
		uint32_t prev = s_A.Seq;
		uint32_t cfg = ADC_SEQ_CTRL_CHANNELS_MASK | ADC_SEQ_CTRL_TRIGGER_MASK | ADC_SEQ_CTRL_TRIGPOL_MASK |
					   ADC_SEQ_CTRL_SYNCBYPASS_MASK | ADC_SEQ_CTRL_SINGLESTEP_MASK | ADC_SEQ_CTRL_LOWPRIO_MASK |
					   ADC_SEQ_CTRL_MODE_MASK;
		bool rise = (v & ADC_SEQ_CTRL_SEQ_ENA_MASK) && !(prev & ADC_SEQ_CTRL_SEQ_ENA_MASK);

		// Channels and trigger change while the sequence is off, or in the
		// write that turns it on or off
		if ((v ^ prev) & cfg)
		{
			assert(!(prev & ADC_SEQ_CTRL_SEQ_ENA_MASK) || !(v & ADC_SEQ_CTRL_SEQ_ENA_MASK));
		}
		assert(!(rise && (v & (ADC_SEQ_CTRL_START_MASK | ADC_SEQ_CTRL_BURST_MASK))));
		assert(!((v & ADC_SEQ_CTRL_START_MASK) && (v & ADC_SEQ_CTRL_BURST_MASK)));
		if (v & ADC_SEQ_CTRL_SEQ_ENA_MASK)
		{
			assert((v & ADC_SEQ_CTRL_TRIGGER_MASK) == 0 && (v & ADC_SEQ_CTRL_TRIGPOL_MASK));
		}
		s_A.Seq = v & ~ADC_SEQ_CTRL_START_MASK;

		uint32_t mask = v & ADC_SEQ_CTRL_CHANNELS_MASK;

		if (!(v & ADC_SEQ_CTRL_SEQ_ENA_MASK))
		{
			// Halted at the end of the conversion in progress
			if (s_A.Conv)
			{
				s_A.StopAfter = true;
			}
			return *this;
		}
		if (v & ADC_SEQ_CTRL_BURST_MASK)
		{
			if (!(prev & ADC_SEQ_CTRL_BURST_MASK))
			{
				assert(!s_A.Conv && mask);
				s_A.Mode = CONV_BURST;
				s_A.Mask = mask;
				s_A.BurstCount = 0;
				StartConv(__builtin_ctz(mask), s_Ns);
			}
		}
		else if (prev & ADC_SEQ_CTRL_BURST_MASK)
		{
			if (s_A.Conv)
			{
				s_A.StopAfter = true;
			}
		}
		if (v & ADC_SEQ_CTRL_START_MASK)
		{
			assert(!s_A.Conv && mask);
			s_A.Mode = CONV_PASS;
			s_A.Mask = mask;
			StartConv(__builtin_ctz(mask), s_Ns);
		}
		return *this;
	}
};
struct DatR {
	int Ch;
	operator uint32_t() const
	{
		Access();
		uint32_t v = s_A.Dat[Ch];
		s_A.Dat[Ch] &= ~(ADC_DAT_DATAVALID_MASK | ADC_DAT_OVERRUN_MASK);
		return v;
	}
};
struct StartupR {
	operator uint32_t() const { Access(); return s_A.Startup; }
	StartupR &operator=(uint32_t v)
	{
		Access();
		if ((v & ADC_STARTUP_ADC_ENA_MASK) && !(s_A.Startup & ADC_STARTUP_ADC_ENA_MASK))
		{
			// Not with ADC_INIT, 10 us after the power up, during the set up
			assert(!(v & ADC_STARTUP_ADC_INIT_MASK));
			assert((s_Pd[0] & (SUPPLY | SYSCON_PDRUNCFG_PDEN_ADC0_MASK)) == 0);
			assert(s_Ns - s_A.AdcOn >= 10000U);
			ErrataAllDigital();
			s_A.Startup |= ADC_STARTUP_ADC_ENA_MASK;
		}
		if (v & ADC_STARTUP_ADC_INIT_MASK)
		{
			assert(s_A.Startup & ADC_STARTUP_ADC_ENA_MASK);
			assert(!s_A.Ready);
			ErrataAllDigital();
			s_A.Startup |= ADC_STARTUP_ADC_INIT_MASK;
			s_A.InitBusy = true;
			s_A.InitEnd = s_Ns + (uint64_t)(6U * ClkNs(s_A.Ctrl) + 0.5);
		}
		return *this;
	}
};
struct CalibR {
	operator uint32_t() const
	{
		Access();
		uint32_t req = !s_A.ChipCal && !(s_A.Ctrl & ADC_CTRL_BYPASSCAL_MASK) ? ADC_CALIB_CALREQD_MASK : 0U;
		return (s_A.CalBusy ? ADC_CALIB_CALIB_MASK : 0U) | req;
	}
	CalibR &operator=(uint32_t v)
	{
		Access();
		if (v & ADC_CALIB_CALIB_MASK)
		{
			assert(s_A.Startup & ADC_STARTUP_ADC_ENA_MASK);
			assert(!(s_A.Ctrl & ADC_CTRL_BYPASSCAL_MASK));
			assert(AdcHz(s_A.Ctrl) <= 30e6);
			assert(!s_A.Conv && !s_A.CalBusy);
			if (!s_A.Ready)
			{
				ErrataAllDigital();
			}
			s_A.CalBusy = true;
			s_A.CalEnd = s_Ns + (uint64_t)(81U * ClkNs(s_A.Ctrl) + 0.5);
		}
		return *this;
	}
};
struct AdcRegs {
	CtrlR CTRL;
	InselR INSEL;
	SeqR SEQ_CTRL[1];
	DatR DAT[12] = { { 0 }, { 1 }, { 2 }, { 3 }, { 4 }, { 5 }, { 6 }, { 7 }, { 8 }, { 9 }, { 10 }, { 11 } };
	StartupR STARTUP;
	CalibR CALIB;
};
static AdcRegs s_AdcRegs;

void ModelDelayUs(uint32_t Us)
{
	s_Ns += (uint64_t)Us * 1000U;
	s_DelayUs += Us;
	Process();
}

#undef SYSCON
#undef IOCON
#undef GPIO
#undef ADC0
#define SYSCON (&s_SysRegs)
#define IOCON s_Iocon
#define GPIO s_Gpio
#define ADC0 (&s_AdcRegs)

#include "../../../src/device.cpp"
#include "../../../src/device_intrf.cpp"
#include "../../../ARM/NXP/LPC546xx/src/adc_lpc546xx.cpp"

static const AdcRefVolt_t s_Ref[] = { { ADC_REFVOLT_TYPE_SUPPLY, 3.3f, -1 } };

static AdcCfg_t Cfg()
{
	AdcCfg_t c = {};

	c.Mode = ADC_CONV_MODE_SINGLE;
	c.pRefVolt = s_Ref;
	c.NbRefVolt = 1;
	c.Resolution = 12;
	return c;
}

static AdcChanCfg_t Chan(int Chan, int Input, int AcqUs = 0)
{
	AdcChanCfg_t c = {};

	c.Chan = Chan;
	c.RefVoltIdx = 0;
	c.Type = ADC_CHAN_TYPE_SINGLE_ENDED;
	c.Gain = 0x100;
	c.AcqTime = AcqUs;
	c.PinP = { Input, ADC_PIN_CONN_NONE };
	c.PinN = { -1, ADC_PIN_CONN_NONE };
	return c;
}

static float Volt(uint32_t Code)
{
	return Code * 3.3f / 4095.0f;
}

static void CheckData(const AdcData_t &d, int Chan, uint32_t Code)
{
	assert(d.Chan == Chan && d.Timestamp == 0);
	assert(fabsf(d.Data - Volt(Code)) < 1e-4f);
}

// Chip after power up: supplies and ADC0 off, ADC pins analog except pin 1
// (SWCLK, function 6), the system oscillator off
static void ChipReset()
{
	memset(s_IoconMem, 0, sizeof(s_IoconMem));
	memset(s_GpioMem, 0, sizeof(s_GpioMem));
	memset(s_Ahb, 0, sizeof(s_Ahb));
	memset(s_Rst, 0, sizeof(s_Rst));
	s_Pd[0] = SUPPLY | SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK;
	s_Pd[1] = SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK;
	s_A = AdcM();
	s_Iocon->PIO[0][11] = IOCON_PIO_FUNC(6) | IOCON_PIO_DIGIMODE_MASK;
	SystemCoreClock = 220000000;
}

static void TestInit()
{
	ChipReset();

	AdcLpc546xx adc, other;
	AdcCfg_t c = Cfg();
	AdcRefVolt_t ref[1] = { { ADC_REFVOLT_TYPE_SUPPLY, 3.3f, -1 } };

	// Refused configurations touch nothing
	c.bInterrupt = true;
	assert(!adc.Init(c));
	c = Cfg();
	c.Mode = ADC_CONV_MODE_CONTINUOUS;
	assert(!adc.Init(c));
	c = Cfg();
	c.OvrSample = 2;
	assert(!adc.Init(c));
	c = Cfg();
	c.OvrSample = -1;
	assert(!adc.Init(c));
	c = Cfg();
	c.NbRefVolt = 2;
	assert(!adc.Init(c));
	c = Cfg();
	c.pRefVolt = nullptr;
	assert(!adc.Init(c));
	c = Cfg();
	ref[0].Type = ADC_REFVOLT_TYPE_INTERNAL;
	c.pRefVolt = ref;
	assert(!adc.Init(c));
	ref[0].Type = ADC_REFVOLT_TYPE_EXTERNAL;
	ref[0].Voltage = 0;
	assert(!adc.Init(c));
	ref[0].Voltage = NAN;
	assert(!adc.Init(c));
	c = Cfg();
	assert(!adc.Init(c, nullptr, (DeviceIntrf *)&other));
	SystemCoreClock = 0;
	assert(!adc.Init(c));
	SystemCoreClock = 220000000;
	assert(s_Ahb[0] == 0 && s_Pd[0] == (SUPPLY | SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK));
	assert(s_Iocon->PIO[0][10] == 0);

	// Interrupts left on by earlier software, the reset of Init clears them
	s_A.Inten = ADC_INTEN_SEQA_INTEN_MASK;
	c.OvrSample = 1;
	assert(adc.Init(c));
	assert(adc.Valid() && s_A.Inten == 0);
	assert(s_Ahb[0] & SYSCON_AHBCLKCTRL_ADC0_MASK);
	assert(s_Pd[0] == SYSCON_PDRUNCFG_PDEN_TS_MASK);
	assert(s_A.CalCount == 1 && s_A.InitCount == 0 && s_A.Ready);
	// Conversions at 220 MHz / 4, 55 MHz
	assert(s_A.Ctrl == (ADC_CTRL_CLKDIV(3) | ADC_CTRL_RESOL(3)));
	assert(s_A.Insel == 0);
	// Erratum ADC.1: the pins with the digital buffer off got it, others kept
	for (int i = 0; i < 12; i++)
	{
		uint32_t v = s_Iocon->PIO[s_Pin[i][0]][s_Pin[i][1]];

		assert(v == (i == 1 ? (IOCON_PIO_FUNC(6) | IOCON_PIO_DIGIMODE_MASK) : IOCON_PIO_DIGIMODE_MASK));
	}

	assert(!other.Init(Cfg()));
	assert(adc.Resolution(8) == 12 && adc.Resolution() == 12);
	assert(adc.Enable());
	assert(s_A.CalCount == 1);

	// Init again by the owner
	assert(adc.Init(Cfg()));
	assert(adc.Valid());
	// The chip calibration is kept, ADC_INIT this time
	assert(s_A.CalCount == 1 && s_A.InitCount == 1);
	adc.PowerOff();
	assert(!adc.Valid());
	assert(other.Init(Cfg()));
	other.PowerOff();
	printf("init: PASS\n");
}

static void TestOpen()
{
	ChipReset();

	AdcLpc546xx adc;
	AdcChanCfg_t c[3];

	assert(!adc.OpenChannel(c, 0));
	assert(adc.Init(Cfg()));

	uint32_t before[12];

	for (int i = 0; i < 12; i++)
	{
		before[i] = s_Iocon->PIO[s_Pin[i][0]][s_Pin[i][1]];
	}

	c[0] = Chan(-1, 4);
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(12, 4);
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, -1);
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 13);
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4);
	c[0].RefVoltIdx = 1;
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4);
	c[0].Type = ADC_CHAN_TYPE_DIFFERENTIAL;
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4);
	c[0].Gain = 0x80;
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4);
	c[0].PinP.Conn = ADC_PIN_CONN_PULLUP;
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4);
	c[0].BurstMode = true;
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4);
	c[0].FifoMemSize = 16;
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4);
	c[0].pFifoMem = (uint8_t *)before;
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4, -1);
	assert(!adc.OpenChannel(c, 1));
	// 9.5 clocks at 220 MHz / 256 is 11.05 us
	c[0] = Chan(0, 4, 12);
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 4);
	assert(!adc.OpenChannel(nullptr, 1));
	AdcChanCfg_t all[13];

	for (int i = 0; i < 13; i++)
	{
		all[i] = Chan(i % 12, i % 12);
	}
	assert(!adc.OpenChannel(all, 13));
	all[12] = Chan(0, ADC_LPC546XX_INPUT_TEMP);
	assert(!adc.OpenChannel(all, 13));
	c[1] = Chan(0, 5);
	assert(!adc.OpenChannel(c, 2));
	c[1] = Chan(1, 4);
	assert(!adc.OpenChannel(c, 2));
	c[0] = Chan(0, 0);
	c[1] = Chan(1, ADC_LPC546XX_INPUT_TEMP);
	assert(!adc.OpenChannel(c, 2));
	c[2] = Chan(2, 4);
	c[1] = Chan(1, 5);
	c[0] = Chan(0, 4, 12);
	assert(!adc.OpenChannel(c, 2));

	for (int i = 0; i < 12; i++)
	{
		assert(s_Iocon->PIO[s_Pin[i][0]][s_Pin[i][1]] == before[i]);
	}
	assert(!adc.StartConversion());

	c[0] = Chan(0, 4, 11);
	assert(adc.OpenChannel(c, 1));
	assert(PinAnalog(4));
	// 11 us: 2 * 11 * 220 / 19 rounded up, divider 255
	assert(adc.StartConversion());
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(254) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(7)));

	// Temperature sensor and input 0 share ADC channel 0
	c[0] = Chan(3, ADC_LPC546XX_INPUT_TEMP);
	assert(adc.OpenChannel(c, 1));
	assert((s_Pd[0] & SYSCON_PDRUNCFG_PDEN_TS_MASK) == 0 && s_A.Insel == 3U);
	c[0] = Chan(4, 0);
	assert(!adc.OpenChannel(c, 1));
	// The same input on another logical channel
	c[0] = Chan(5, 4);
	assert(!adc.OpenChannel(c, 1));
	// Set again on the logical channel of the sensor
	c[0] = Chan(3, 0);
	assert(adc.OpenChannel(c, 1));
	assert((s_Pd[0] & SYSCON_PDRUNCFG_PDEN_TS_MASK) && s_A.Insel == 0);
	assert(PinAnalog(0));
	// Inputs swapped between two logical channels in one call
	c[0] = Chan(0, 5);
	c[1] = Chan(3, 4);
	assert(adc.OpenChannel(c, 2));
	assert(PinAnalog(4) && PinAnalog(5));
	assert(s_Iocon->PIO[0][10] == IOCON_PIO_DIGIMODE_MASK);
	adc.CloseChannel(0);
	adc.CloseChannel(3);
	assert(s_Iocon->PIO[0][16] == IOCON_PIO_DIGIMODE_MASK && s_Iocon->PIO[0][31] == IOCON_PIO_DIGIMODE_MASK);
	// The sampling time of a closed channel no longer counts
	c[0] = Chan(1, 9);
	assert(adc.OpenChannel(c, 1));
	assert(adc.StartConversion());
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(3) | ADC_CTRL_RESOL(3)));
	c[0] = Chan(2, 8, 11);
	assert(adc.OpenChannel(c, 1));
	assert(adc.StartConversion());
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(254) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(7)));
	adc.CloseChannel(2);
	assert(adc.StartConversion());
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(3) | ADC_CTRL_RESOL(3)));
	adc.CloseChannel(1);
	adc.CloseChannel(3);
	adc.CloseChannel(-1);
	adc.CloseChannel(12);
	adc.PowerOff();
	printf("open: PASS\n");
}

static void TestConvert()
{
	ChipReset();

	AdcLpc546xx adc;
	AdcChanCfg_t c[3];
	AdcData_t d[4];

	// P0_16 GPIO output with pull up before the ADC
	s_Iocon->PIO[0][16] = IOCON_PIO_MODE(2) | IOCON_PIO_DIGIMODE_MASK;
	s_Gpio->DIR[0] = 1UL << 16;
	assert(adc.Init(Cfg()));

	c[0] = Chan(0, 4);
	c[1] = Chan(1, 5);
	c[2] = Chan(2, 11);
	assert(adc.OpenChannel(c, 3));
	assert(PinAnalog(4) && PinAnalog(5) && PinAnalog(11));
	s_A.Code[4] = 1000;
	s_A.Code[5] = 4095;
	s_A.Code[11] = 0;
	// Stale result before the conversion
	s_A.Dat[4] = ADC_DAT_DATAVALID_MASK | ADC_DAT_RESULT(77);

	assert(!adc.UpdateData());
	assert(adc.StartConversion());
	assert(adc.UpdateData());
	assert(s_A.Passes == 1 && s_A.Convs[4] == 1 && s_A.Convs[5] == 1 && s_A.Convs[11] == 1);
	assert(!s_A.Conv && s_A.Seq == 0 && s_A.Overruns == 0);
	assert(adc.Read(d, 2) == 2);
	CheckData(d[0], 0, 1000);
	CheckData(d[1], 1, 4095);
	assert(adc.Read(d, 4) == 1);
	CheckData(d[0], 2, 0);
	assert(adc.Read(d, 4) == 0);
	assert(!adc.UpdateData());
	assert(adc.Read(nullptr, 4) == 0 && adc.Read(d, 0) == 0);

	// Read of a channel without a new result converts it alone
	s_A.Code[5] = 2048;
	assert(adc.Read(1, &d[0]));
	CheckData(d[0], 1, 2048);
	assert(s_A.Passes == 2 && s_A.Convs[5] == 2 && s_A.Convs[4] == 1);
	assert(!adc.Read(3, &d[0]) && !adc.Read(-1, &d[0]) && !adc.Read(12, &d[0]) && !adc.Read(1, nullptr));

	// StopConversion drops the results not read
	assert(adc.StartConversion());
	adc.StopConversion();
	assert(!adc.UpdateData() && adc.Read(d, 4) == 0);

	// Continuous mode is not supported, StartConversion drops the results
	assert(adc.StartConversion());
	adc.Mode(ADC_CONV_MODE_CONTINUOUS);
	assert(!adc.StartConversion() && !adc.UpdateData());
	adc.Mode(ADC_CONV_MODE_SINGLE);

	// Pin made analog after Init: back with the digital buffer on
	s_Iocon->PIO[2][1] = 0;
	c[0] = Chan(3, 8);
	assert(adc.OpenChannel(c, 1));
	adc.CloseChannel(3);
	assert(s_Iocon->PIO[2][1] == IOCON_PIO_DIGIMODE_MASK);

	// The pin back as it was, digital buffer on
	adc.CloseChannel(0);
	assert(s_Iocon->PIO[0][16] == (IOCON_PIO_MODE(2) | IOCON_PIO_DIGIMODE_MASK));
	assert(s_Gpio->DIR[0] == (1UL << 16));
	assert(adc.StartConversion());
	assert(adc.Read(d, 4) == 2);
	CheckData(d[0], 1, 2048);
	CheckData(d[1], 2, 0);

	// Sampling time: input 5 needs 1 us
	s_A.MinSampUs[5] = 1.0;
	assert(adc.StartConversion());
	assert(adc.Read(d, 4) == 2);
	CheckData(d[0], 1, BAD_CODE);
	c[0] = Chan(1, 5, 1);
	assert(adc.OpenChannel(c, 1));
	assert(adc.StartConversion());
	// 2 * 1 * 220 / 19 rounded up, divider 24, TSAMP 7: 1.036 us
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(23) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(7)));
	assert(adc.Read(d, 4) == 2);
	CheckData(d[0], 1, 2048);
	adc.PowerOff();

	// 12 MHz: divider 1 without a sampling time
	ChipReset();
	SystemCoreClock = 12000000;
	assert(adc.Init(Cfg()));
	assert(s_A.Ctrl == (ADC_CTRL_CLKDIV(0) | ADC_CTRL_RESOL(3)));
	s_A.MinSampUs[6] = 100.0;
	s_A.Code[6] = 321;
	c[0] = Chan(0, 6, 100);
	assert(adc.OpenChannel(c, 1));
	assert(adc.Read(0, &d[0]));
	CheckData(d[0], 0, 321);
	// 2 * 100 * 12 / 19 rounded up, divider 127, TSAMP 7
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(126) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(7)));
	s_A.MinSampUs[6] = 1.0;
	c[0] = Chan(0, 6, 1);
	assert(adc.OpenChannel(c, 1));
	assert(adc.Read(0, &d[0]));
	CheckData(d[0], 0, 321);
	// 9.5 clocks of 12 MHz are too short, divider 2: 1 us is 12 half clocks
	// of 6 MHz, TSAMP (12 - 5 + 1) / 2 = 4, 6.5 clocks, 1.083 us
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(1) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(4)));
	adc.PowerOff();
	printf("convert: PASS\n");
}

static void TestTemp()
{
	ChipReset();

	AdcLpc546xx adc;
	AdcChanCfg_t c[2];
	AdcData_t d[2];

	assert(adc.Init(Cfg()));
	c[0] = Chan(0, 4);
	c[1] = Chan(1, ADC_LPC546XX_INPUT_TEMP);
	uint64_t us = s_DelayUs;
	assert(adc.OpenChannel(c, 2));
	assert(s_DelayUs - us >= 100);
	assert(s_A.Insel == 3U && (s_Pd[0] & SYSCON_PDRUNCFG_PDEN_TS_MASK) == 0);
	s_A.Code[4] = 1500;
	s_A.TempCode = 720;

	assert(adc.StartConversion());
	assert(s_A.TempConvs >= 9 && !s_A.Conv && s_A.Seq == 0);
	assert((s_A.Dat[0] & ADC_DAT_DATAVALID_MASK) == 0);
	assert(s_A.Ctrl == (ADC_CTRL_CLKDIV(3) | ADC_CTRL_RESOL(3)));
	assert(adc.Read(d, 2) == 2);
	CheckData(d[0], 0, 1500);
	CheckData(d[1], 1, 720);

	// Sensor alone
	int n = s_A.TempConvs;
	assert(adc.Read(1, &d[0]));
	CheckData(d[0], 1, 720);
	assert(s_A.TempConvs - n >= 9 && s_A.Convs[4] == 1);

	assert(s_A.TempCtrl == (ADC_CTRL_CLKDIV(3) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(7)));

	// Sensor with 1 us of sampling: its own divider, 24 at 220 MHz, the pin
	// conversions keep theirs
	c[0] = Chan(1, ADC_LPC546XX_INPUT_TEMP, 1);
	assert(adc.OpenChannel(c, 1));
	assert(s_A.Ctrl == (ADC_CTRL_CLKDIV(3) | ADC_CTRL_RESOL(3)));
	assert(adc.StartConversion());
	assert(s_A.TempCtrl == (ADC_CTRL_CLKDIV(23) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(7)));
	assert(s_A.Ctrl == (ADC_CTRL_CLKDIV(3) | ADC_CTRL_RESOL(3)));
	assert(adc.Read(d, 2) == 2);
	CheckData(d[0], 0, 1500);
	CheckData(d[1], 1, 720);

	adc.CloseChannel(1);
	assert((s_Pd[0] & SYSCON_PDRUNCFG_PDEN_TS_MASK) && s_A.Insel == 0);
	adc.PowerOff();
	printf("temp: PASS\n");
}

// Slow system clocks: calibration and ADC_INIT waits, the largest divider,
// TSAMP rounding
static void TestClock()
{
	AdcLpc546xx adc;
	AdcChanCfg_t c[1];
	AdcData_t d;

	ChipReset();
	SystemCoreClock = 400000;
	assert(adc.Init(Cfg()));
	assert(s_A.CalCount == 1);
	assert(adc.Init(Cfg()));
	assert(s_A.InitCount == 1);
	c[0] = Chan(0, 7);
	assert(adc.OpenChannel(c, 1));
	s_A.Code[7] = 1234;
	assert(adc.Read(0, &d));
	CheckData(d, 0, 1234);
	adc.PowerOff();

	// 19 MHz: 128 us needs 9.5 clocks of 19 MHz / 256
	ChipReset();
	SystemCoreClock = 19000000;
	assert(adc.Init(Cfg()));
	c[0] = Chan(0, 7, 129);
	assert(!adc.OpenChannel(c, 1));
	c[0] = Chan(0, 7, 128);
	assert(adc.OpenChannel(c, 1));
	s_A.Code[7] = 4000;
	s_A.MinSampUs[7] = 128.0;
	assert(adc.Read(0, &d));
	CheckData(d, 0, 4000);
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(255) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(7)));
	adc.PowerOff();

	// 3.75 MHz: 1 us is 7.5 half clocks, TSAMP 2, 4.5 clocks, 1.2 us
	ChipReset();
	SystemCoreClock = 3750000;
	assert(adc.Init(Cfg()));
	c[0] = Chan(0, 7, 1);
	assert(adc.OpenChannel(c, 1));
	s_A.Code[7] = 17;
	s_A.MinSampUs[7] = 1.0;
	assert(adc.Read(0, &d));
	CheckData(d, 0, 17);
	assert(s_A.PinCtrl == (ADC_CTRL_CLKDIV(0) | ADC_CTRL_RESOL(3) | ADC_CTRL_TSAMP(2)));
	adc.PowerOff();
	printf("clock: PASS\n");
}

static void TestPower()
{
	ChipReset();

	AdcLpc546xx adc;
	AdcChanCfg_t c[2];
	AdcData_t d[2];

	assert(adc.Init(Cfg()));
	c[0] = Chan(0, 9);
	c[1] = Chan(1, ADC_LPC546XX_INPUT_TEMP);
	assert(adc.OpenChannel(c, 2));
	s_A.Code[9] = 99;
	s_A.TempCode = 700;

	int down = s_PdDownAdc;

	adc.Disable();
	assert(s_PdDownAdc == down + 1);
	assert(s_Pd[0] & SYSCON_PDRUNCFG_PDEN_ADC0_MASK && s_Pd[0] & SYSCON_PDRUNCFG_PDEN_TS_MASK);
	assert(s_Iocon->PIO[3][21] == IOCON_PIO_DIGIMODE_MASK);
	adc.Disable();
	assert(s_PdDownAdc == down + 1);

	// Open while off: the pin stays digital until the ADC is powered
	c[0] = Chan(2, 10);
	assert(adc.OpenChannel(c, 1));
	assert(s_Iocon->PIO[3][22] & IOCON_PIO_DIGIMODE_MASK);
	s_A.Code[10] = 55;

	// Conversion powers it again, ADC_INIT as the chip is calibrated
	assert(adc.StartConversion());
	assert(s_A.CalCount == 1 && s_A.InitCount == 1);
	assert(PinAnalog(9) && PinAnalog(10) && s_A.Insel == 3U);
	assert(adc.Read(d, 2) == 2);
	CheckData(d[0], 0, 99);
	CheckData(d[1], 1, 700);
	assert(adc.Read(2, &d[0]));
	CheckData(d[0], 2, 55);

	adc.Disable();
	assert(adc.Enable());
	assert(s_A.InitCount == 2);
	assert(adc.Enable());
	assert(s_A.InitCount == 2);

	// Calibrate at 30 MHz or less, CTRL back after it
	uint32_t ctrl = s_A.Ctrl;

	assert(adc.Calibrate());
	assert(s_A.CalCount == 2 && s_A.Ctrl == ctrl);

	// Conversion that does not end: invalid until Reset
	s_A.StuckConv = true;
	assert(!adc.StartConversion());
	assert(!adc.Valid() && s_A.Seq == 0);
	assert(adc.Read(d, 2) == 0 && !adc.Read(0, &d[0]) && !adc.UpdateData());
	assert(!adc.StartConversion() && !adc.Calibrate());
	assert(!adc.OpenChannel(c, 1));
	s_A.StuckConv = false;
	s_A.Conv = false;
	adc.Reset();
	assert(adc.Valid() && s_A.Insel == 3U && PinAnalog(9));
	assert(adc.StartConversion());
	assert(adc.Read(d, 2) == 2);
	CheckData(d[0], 0, 99);
	CheckData(d[1], 1, 700);

	// Temperature burst that does not end
	s_A.StuckConv = true;
	assert(!adc.Read(1, &d[0]));
	assert(!adc.Valid());
	s_A.StuckConv = false;
	s_A.Conv = false;
	adc.Reset();
	assert(adc.Read(1, &d[0]));
	CheckData(d[0], 1, 700);

	// Calibration that does not end
	s_A.StuckCal = true;
	assert(!adc.Calibrate());
	assert(!adc.Valid());
	s_A.StuckCal = false;
	adc.Reset();
	assert(adc.Valid());

	// PowerOff: pins back, ADC0 and the sensor off, the supplies Init
	// powered off too, VD2_ANA as the system oscillator is off
	adc.PowerOff();
	assert(s_Iocon->PIO[3][21] == IOCON_PIO_DIGIMODE_MASK && s_Iocon->PIO[3][22] == IOCON_PIO_DIGIMODE_MASK);
	assert(s_Pd[0] == (SUPPLY | SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK));
	assert((s_Ahb[0] & SYSCON_AHBCLKCTRL_ADC0_MASK) == 0);
	assert(!adc.StartConversion() && adc.Read(d, 2) == 0 && !adc.Enable() && !adc.Calibrate());
	adc.Disable();
	adc.Reset();
	adc.StopConversion();
	adc.CloseChannel(0);

	// Supplies on before Init stay on
	s_Pd[0] = SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK;
	s_Pd[1] = 0;
	assert(adc.Init(Cfg()));
	adc.PowerOff();
	assert(s_Pd[0] == (SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK));

	// VD2_ANA powered by Init, then the system oscillator started: kept on
	s_Pd[0] = SUPPLY | SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK;
	s_Pd[1] = SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK;
	assert(adc.Init(Cfg()));
	s_Pd[1] = 0;
	adc.PowerOff();
	assert(s_Pd[0] == (SYSCON_PDRUNCFG_PDEN_VDDA_MASK | SYSCON_PDRUNCFG_PDEN_VREFP_MASK |
					   SYSCON_PDRUNCFG_PDEN_ADC0_MASK | SYSCON_PDRUNCFG_PDEN_TS_MASK));

	// Calibration that does not end at Init: refused, ADC0 free
	ChipReset();
	s_A.StuckCal = true;
	assert(!adc.Init(Cfg()));
	assert((s_Ahb[0] & SYSCON_AHBCLKCTRL_ADC0_MASK) == 0 && (s_Pd[0] & SYSCON_PDRUNCFG_PDEN_ADC0_MASK));
	s_A.StuckCal = false;
	{
		AdcLpc546xx tmp;

		assert(tmp.Init(Cfg()));
		c[0] = Chan(0, 7);
		assert(tmp.OpenChannel(c, 1));
		assert(PinAnalog(7));
	}
	// The destructor powered it off
	assert(s_Iocon->PIO[2][0] == IOCON_PIO_DIGIMODE_MASK && (s_Ahb[0] & SYSCON_AHBCLKCTRL_ADC0_MASK) == 0);
	assert(adc.Init(Cfg()));
	adc.PowerOff();
	printf("power: PASS\n");
}

int main()
{
	TestInit();
	TestOpen();
	TestConvert();
	TestTemp();
	TestClock();
	TestPower();
	printf("adc: PASS\n");
	return 0;
}
