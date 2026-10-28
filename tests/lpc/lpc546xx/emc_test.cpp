// Host regression of the LPC546xx EMC SDRAM set up (SdramInit of
// coredev/sdram.h) with a register model.
//
// Model: the EMC registers need the EMC bus clock (AHBCLKCTRL2) and the EMC
// out of reset (PRESETCTRL2), the reset clears them. EMCCLKDIV is set before
// the EMC is enabled. The SDRAM checks the power up sequence: the address
// map and latencies of one chip select, the read strategy and timings
// written before the first command, NOP 100 us or more after the EMC is enabled, precharge all 100 us
// or more after it, 8 or more refreshes at the DYNAMICREFRESH rate (16
// clocks each unit) before the mode register command, then one read whose
// row address is the mode value with bank 0, then normal mode, then the
// buffer enable. No other SDRAM access during the sequence. The EMC clock is
// the CPU clock / (EMCCLKDIV + 1).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "LPC546xx.h"

uint32_t SystemCoreClock = 220000000;
uint32_t g_IrqMasked = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { (void)n; (void)p; }
void NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
void NVIC_DisableIRQ(IRQn_Type n) { (void)n; }

static double s_Ns = 0;

alignas(8) static uint8_t s_IoconMem[sizeof(IOCON_Type)];
alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];
alignas(8) static uint8_t s_PintMem[sizeof(PINT_Type)];
alignas(8) static uint8_t s_InputMuxMem[sizeof(INPUTMUX_Type)];

static uint32_t s_Ahb[3], s_Rst[3];
static uint32_t s_ClkDiv = 0, s_SysCtrl = 0xFFFFFFFFU, s_DlyCtrl = 0x1010U;
static bool s_DivSet = false;
static int s_EmcResets = 0;

enum { ST_OFF, ST_ON, ST_NOP, ST_PALL, ST_MODE, ST_READ, ST_NORMAL };

struct EmcM {
	uint32_t Control = 0, Config = 0, DynCtrl = 0, Refresh = 0, ReadCfg = 0;
	uint32_t Rp = 0, Ras = 0, Srex = 0, Apr = 0, Dal = 0, Wr = 0, Rc = 0, Rfc = 0, Xsr = 0, Rrd = 0, Mrd = 0;
	uint32_t DynCfg[4] = {}, RasCas[4] = {};
	int Chip = -1;					// Chip select set up
	uint32_t Written = 0;			// Bit per timing register written
	int State = ST_OFF;
	double OnNs = 0, NopNs = 0, PallNs = 0, RefNs = 0;
	double Refreshes = 0;
	uint32_t ModeAddr = 0;
	int Reads = 0;
	bool BufBeforeNormal = false;
};
static EmcM s_E;

static double EmcClkHz()
{
	return (double)SystemCoreClock / (s_ClkDiv + 1U);
}

// Refreshes since the last change of the rate, during the sequence
static void Advance()
{
	if (s_E.State == ST_PALL && s_E.Refresh != 0)
	{
		double period = 16.0 * s_E.Refresh / EmcClkHz() * 1e9;
		s_E.Refreshes += (s_Ns - s_E.RefNs) / period;
	}
	s_E.RefNs = s_Ns;
}

void ModelDelayUs(uint32_t Us)
{
	Advance();
	s_Ns += Us * 1000.0;
	Advance();
}

static void Access()
{
	assert(s_Ahb[2] & SYSCON_AHBCLKCTRL_EMC_MASK);
	assert((s_Rst[2] & SYSCON_PRESETCTRL_EMC_RESET_MASK) == 0);
	s_Ns += 5;
}

enum { AHB_SET, AHB_CLR, RST_SET, RST_CLR };

static void SysWrite(int Kind, int Idx, uint32_t v)
{
	switch (Kind)
	{
		case AHB_SET: s_Ahb[Idx] |= v; break;
		case AHB_CLR: s_Ahb[Idx] &= ~v; break;
		case RST_SET:
			if (Idx == 2 && (v & SYSCON_PRESETCTRL_EMC_RESET_MASK))
			{
				s_E = EmcM();
				s_EmcResets++;
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
struct ClkDivR {
	ClkDivR &operator=(uint32_t v)
	{
		assert((v & ~SYSCON_EMCCLKDIV_DIV_MASK) == 0);
		assert(s_E.State == ST_OFF);
		s_ClkDiv = v;
		s_DivSet = true;
		return *this;
	}
};
struct SysCtrlR {
	SysCtrlR &operator=(uint32_t v) { s_SysCtrl = v; return *this; }
};
struct DlyCtrlR {
	DlyCtrlR &operator=(uint32_t v) { s_DlyCtrl = v; return *this; }
};
struct SysRegs {
	AhbR AHBCLKCTRL[3] = { { 0 }, { 1 }, { 2 } };
	WReg AHBCLKCTRLSET[3] = { { AHB_SET, 0 }, { AHB_SET, 1 }, { AHB_SET, 2 } };
	WReg AHBCLKCTRLCLR[3] = { { AHB_CLR, 0 }, { AHB_CLR, 1 }, { AHB_CLR, 2 } };
	WReg PRESETCTRLSET[3] = { { RST_SET, 0 }, { RST_SET, 1 }, { RST_SET, 2 } };
	WReg PRESETCTRLCLR[3] = { { RST_CLR, 0 }, { RST_CLR, 1 }, { RST_CLR, 2 } };
	ClkDivR EMCCLKDIV;
	SysCtrlR EMCSYSCTRL;
	DlyCtrlR EMCDLYCTRL;
};
static SysRegs s_SysRegs;

// Timing and configuration registers, written before the first command
enum { W_RP, W_RAS, W_SREX, W_APR, W_DAL, W_WR, W_RC, W_RFC, W_XSR, W_RRD, W_MRD, W_READ, W_DYNCFG, W_RASCAS, W_ALL };

struct TimR {
	uint32_t *pVal;
	int Bit;
	uint32_t Mask;
	operator uint32_t() const { Access(); return *pVal; }
	TimR &operator=(uint32_t v)
	{
		Access();
		assert(s_E.State == ST_ON);
		assert((v & ~Mask) == 0);
		*pVal = v;
		s_E.Written |= 1U << Bit;
		return *this;
	}
};
struct ControlR {
	operator uint32_t() const { Access(); return s_E.Control; }
	ControlR &operator=(uint32_t v)
	{
		Access();
		assert(v == EMC_CONTROL_E_MASK && s_DivSet);
		s_E.Control = v;
		if (s_E.State == ST_OFF)
		{
			s_E.State = ST_ON;
			s_E.OnNs = s_Ns;
		}
		return *this;
	}
};
struct ConfigR {
	ConfigR &operator=(uint32_t v) { Access(); assert(s_E.State == ST_OFF); s_E.Config = v; return *this; }
};
struct DynCtrlR {
	DynCtrlR &operator=(uint32_t v)
	{
		Access();
		Advance();
		uint32_t cmd = v & EMC_DYNAMICCONTROL_I_MASK;
		bool clk = (v & (EMC_DYNAMICCONTROL_CE_MASK | EMC_DYNAMICCONTROL_CS_MASK)) ==
				   (EMC_DYNAMICCONTROL_CE_MASK | EMC_DYNAMICCONTROL_CS_MASK);
		switch (s_E.State)
		{
			case ST_ON:
				assert(cmd == 0x180U && clk);
				assert(s_E.Written == (1U << W_ALL) - 1U);
				assert(s_Ns - s_E.OnNs >= 100000.0);
				s_E.State = ST_NOP;
				s_E.NopNs = s_Ns;
				break;
			case ST_NOP:
				assert(cmd == 0x100U && clk);
				assert(s_Ns - s_E.NopNs >= 100000.0);
				s_E.State = ST_PALL;
				s_E.PallNs = s_Ns;
				s_E.Refreshes = 0;
				break;
			case ST_PALL:
				assert(cmd == 0x080U && clk);
				assert(s_E.Refreshes >= 8.0);
				s_E.State = ST_MODE;
				break;
			case ST_READ:
				assert(v == 0);
				s_E.State = ST_NORMAL;
				break;
			default:
				assert(false);
		}
		s_E.DynCtrl = v;
		return *this;
	}
};
struct RefreshR {
	operator uint32_t() const { Access(); return s_E.Refresh; }
	RefreshR &operator=(uint32_t v)
	{
		Access();
		Advance();
		assert(v >= 1 && v <= EMC_DYNAMICREFRESH_REFRESH_MASK);
		s_E.Refresh = v;
		return *this;
	}
};
// DYNAMICCONFIG and DYNAMICRASCAS of one chip select, the same one
static void ChipWrite(int Idx)
{
	assert(s_E.State == ST_ON);
	assert(s_E.Chip < 0 || s_E.Chip == Idx);
	s_E.Chip = Idx;
}

struct DynCfgR {
	int Idx;
	operator uint32_t() const { Access(); return s_E.DynCfg[Idx]; }
	DynCfgR &operator|=(uint32_t v) { return *this = (uint32_t)*this | v; }
	DynCfgR &operator=(uint32_t v)
	{
		Access();
		if (v & EMC_DYNAMIC_DYNAMICCONFIG_B_MASK)
		{
			// The buffer after normal mode, the configuration unchanged
			assert(s_E.State == ST_NORMAL && Idx == s_E.Chip);
			assert((v & ~EMC_DYNAMIC_DYNAMICCONFIG_B_MASK) == s_E.DynCfg[Idx]);
		}
		else
		{
			ChipWrite(Idx);
			assert((v & ~(EMC_DYNAMIC_DYNAMICCONFIG_AM0_MASK | EMC_DYNAMIC_DYNAMICCONFIG_AM1_MASK)) == 0);
			s_E.Written |= 1U << W_DYNCFG;
		}
		s_E.DynCfg[Idx] = v;
		return *this;
	}
};
struct RasCasR {
	int Idx;
	RasCasR &operator=(uint32_t v)
	{
		Access();
		ChipWrite(Idx);
		assert((v & ~(EMC_DYNAMIC_DYNAMICRASCAS_RAS_MASK | EMC_DYNAMIC_DYNAMICRASCAS_CAS_MASK)) == 0);
		s_E.RasCas[Idx] = v;
		s_E.Written |= 1U << W_RASCAS;
		return *this;
	}
};
struct NoR {
	NoR &operator=(uint32_t) { assert(false); return *this; }
	operator uint32_t() const { assert(false); return 0; }
};
struct DynChip {
	DynCfgR DYNAMICCONFIG;
	RasCasR DYNAMICRASCAS;
};
struct EmcRegs {
	ControlR CONTROL;
	NoR STATUS;
	ConfigR CONFIG;
	DynCtrlR DYNAMICCONTROL;
	RefreshR DYNAMICREFRESH;
	TimR DYNAMICREADCONFIG = { &s_E.ReadCfg, W_READ, EMC_DYNAMICREADCONFIG_RD_MASK };
	TimR DYNAMICRP = { &s_E.Rp, W_RP, EMC_DYNAMICRP_TRP_MASK };
	TimR DYNAMICRAS = { &s_E.Ras, W_RAS, EMC_DYNAMICRAS_TRAS_MASK };
	TimR DYNAMICSREX = { &s_E.Srex, W_SREX, EMC_DYNAMICSREX_TSREX_MASK };
	TimR DYNAMICAPR = { &s_E.Apr, W_APR, EMC_DYNAMICAPR_TAPR_MASK };
	TimR DYNAMICDAL = { &s_E.Dal, W_DAL, EMC_DYNAMICDAL_TDAL_MASK };
	TimR DYNAMICWR = { &s_E.Wr, W_WR, EMC_DYNAMICWR_TWR_MASK };
	TimR DYNAMICRC = { &s_E.Rc, W_RC, EMC_DYNAMICRC_TRC_MASK };
	TimR DYNAMICRFC = { &s_E.Rfc, W_RFC, EMC_DYNAMICRFC_TRFC_MASK };
	TimR DYNAMICXSR = { &s_E.Xsr, W_XSR, EMC_DYNAMICXSR_TXSR_MASK };
	TimR DYNAMICRRD = { &s_E.Rrd, W_RRD, EMC_DYNAMICRRD_TRRD_MASK };
	TimR DYNAMICMRD = { &s_E.Mrd, W_MRD, EMC_DYNAMICMRD_TMRD_MASK };
	DynChip DYNAMIC[4] = { { { 0 }, { 0 } }, { { 1 }, { 1 } }, { { 2 }, { 2 } }, { { 3 }, { 3 } } };
};
static EmcRegs s_EmcRegs;

// The read in mode register state
static uint32_t ModelRead32(uint32_t Addr)
{
	assert(s_E.State == ST_MODE);
	s_E.ModeAddr = Addr;
	s_E.Reads++;
	s_E.State = ST_READ;
	return 0;
}

#undef IOCON
#undef GPIO
#undef SYSCON
#undef PINT
#undef INPUTMUX
#undef EMC
#define IOCON ((IOCON_Type *)s_IoconMem)
#define GPIO ((GPIO_Type *)s_GpioMem)
#define SYSCON (&s_SysRegs)
#define PINT ((PINT_Type *)s_PintMem)
#define INPUTMUX ((INPUTMUX_Type *)s_InputMuxMem)
#define EMC (&s_EmcRegs)
#define LPC546XX_EMC_READ32(Addr) ModelRead32(Addr)

#include "../../../ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/emc_lpc546xx.cpp"


// The pins of the LPCXpresso54628 SDRAM (NXP SDK emc sdram example)
static const IOPinCfg_t s_Pins[] = {
	{0, 2, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},		// D0
	{0, 15, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// WE
	{1, 11, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CLK0
	{3, 25, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// A14
};

static const uint32_t s_Base[4] = { 0xA0000000U, 0xA8000000U, 0xB0000000U, 0xB8000000U };

// The board part, MT48LC8M16A2B4-6A, NXP SDK board.c
static SdramCfg_t Board()
{
	SdramCfg_t c = {};

	c.DevNo = 0;
	c.pIOPinMap = s_Pins;
	c.NbIOPins = sizeof(s_Pins) / sizeof(s_Pins[0]);
	c.Rate = 75000000;
	c.BusWidth = 16;
	c.DevWidth = 16;
	c.NbBanks = 4;
	c.RowBits = 12;
	c.ColBits = 9;
	c.CasLatency = 3;
	c.TwrClk = 1;
	c.TmrdClk = 2;
	c.RefreshNs = 64000000 / 4096;
	c.TrcdNs = 18;
	c.TrpNs = 18;
	c.TrasNs = 42;
	c.TrcNs = 60;
	c.TrfcNs = 60;
	c.TrrdNs = 12;
	c.TwrNs = 6;
	c.TaprNs = 18;
	c.TsrexNs = 67;
	c.TxsrNs = 67;
	return c;
}

static uint32_t Cyc(double Clk, double Ns)
{
	return (uint32_t)ceil(Clk * Ns / 1e9 - 1e-9);
}

static uint32_t Reg(double Clk, double Ns, uint32_t Max)
{
	uint32_t c = Cyc(Clk, Ns);

	c = c > 0 ? c - 1 : 0;
	return c > Max ? Max : c;
}

// The parts of the user manual address mapping table: width, banks, rows,
// columns
struct Part { uint8_t Width, Banks, Rows, Cols; };
static const Part s_Parts[] = {
	{ 8, 2, 11, 9 }, { 16, 2, 11, 8 },
	{ 8, 4, 12, 9 }, { 16, 4, 12, 8 }, { 32, 4, 11, 8 },
	{ 8, 4, 12, 10 }, { 16, 4, 12, 9 }, { 32, 4, 12, 8 },
	{ 8, 4, 13, 10 }, { 16, 4, 13, 9 }, { 32, 4, 13, 8 },
	{ 8, 4, 13, 11 }, { 16, 4, 13, 10 },
};

// AM0 of a part from its capacity and width: size 16, 64, 128, 256, 512 Mb
// is 0 to 4 in bits 4:2, width x8, x16, x32 is 0 to 2 in bits 1:0
static uint32_t Am0(const SdramCfg_t &C)
{
	uint64_t mbit = ((uint64_t)C.NbBanks << (C.RowBits + C.ColBits)) * C.DevWidth >> 20;
	uint32_t size = mbit == 16 ? 0 : mbit == 64 ? 1 : mbit == 128 ? 2 : mbit == 256 ? 3 : 4;
	assert(mbit == 16 || mbit == 64 || mbit == 128 || mbit == 256 || mbit == 512);
	uint32_t width = C.DevWidth == 8 ? 0 : C.DevWidth == 16 ? 1 : 2;

	return (size << 2) | width;
}

static bool Run(const SdramCfg_t &Cfg, SdramDev_t &Dev)
{
	s_Ns += 1e6;
	return SdramInit(&Dev, &Cfg);
}

// The registers and the memory set up for C
static void Check(const SdramCfg_t &C, const SdramDev_t &D)
{
	int cs = C.DevNo;
	bool bus32 = C.BusWidth == 32;

	assert(s_E.State == ST_NORMAL && s_E.Reads == 1 && s_E.Chip == cs);
	assert(s_E.DynCfg[cs] & EMC_DYNAMIC_DYNAMICCONFIG_B_MASK);
	for (int i = 0; i < 4; i++)
	{
		assert(i == cs || (s_E.DynCfg[i] == 0 && s_E.RasCas[i] == 0));
	}
	assert(s_E.Config == 0 && s_E.Control == EMC_CONTROL_E_MASK && s_SysCtrl == 0);
	assert(s_DlyCtrl == 0x1010U);
	assert(D.Rate == SystemCoreClock / (s_ClkDiv + 1U));
	assert((s_E.DynCfg[cs] & ~EMC_DYNAMIC_DYNAMICCONFIG_B_MASK) == ((Am0(C) << 7) | ((bus32 ? 1U : 0U) << 14)));

	double clk = D.Rate;
	uint32_t ras = Cyc(clk, C.TrcdNs);
	ras = ras < 1 ? 1 : (ras > 3 ? 3 : ras);
	assert(s_E.RasCas[cs] == (ras | ((uint32_t)C.CasLatency << 8)));
	assert(s_E.ReadCfg == 1);
	assert(s_E.Rp == Reg(clk, C.TrpNs, 15) && s_E.Ras == Reg(clk, C.TrasNs, 15));
	assert(s_E.Srex == Reg(clk, C.TsrexNs, 15) && s_E.Apr == Reg(clk, C.TaprNs, 15));
	uint32_t wr = Cyc(clk, C.TwrNs) + C.TwrClk;
	assert(s_E.Wr == (wr > 16 ? 15 : (wr > 0 ? wr - 1 : 0)));
	uint32_t dal = wr + Cyc(clk, C.TrpNs);
	assert(s_E.Dal == (dal > 15 ? 15 : dal));
	assert(s_E.Rc == Reg(clk, C.TrcNs, 31) && s_E.Rfc == Reg(clk, C.TrfcNs, 31));
	assert(s_E.Xsr == Reg(clk, C.TxsrNs, 31) && s_E.Rrd == Reg(clk, C.TrrdNs, 15));
	assert(s_E.Mrd == C.TmrdClk - 1U);
	uint32_t refresh = Cyc(clk, C.RefreshNs) / 16;
	refresh = refresh < 1 ? 1 : (refresh > 0x7FF ? 0x7FF : refresh);
	assert(s_E.Refresh == refresh);

	// Mode register: CAS latency, sequential burst of 8 (16 bit bus) or 4,
	// on the row address of bank 0: above the byte lanes, the columns and
	// the bank bits
	uint32_t mode = ((uint32_t)C.CasLatency << 4) | (bus32 ? 2U : 3U);
	uint32_t row = (bus32 ? 2U : 1U) + C.ColBits + (C.NbBanks == 2 ? 1U : 2U);
	assert(s_E.ModeAddr == s_Base[cs] + (mode << row));

	assert(D.pBase == (void *)(uintptr_t)s_Base[cs]);
	assert(D.Size == ((uint32_t)C.NbBanks << (C.RowBits + C.ColBits)) * (C.BusWidth / 8U));
	assert(D.Cfg.DevNo == C.DevNo && D.Cfg.Rate == C.Rate && D.Cfg.pIOPinMap == C.pIOPinMap);
}

static void TestBoard()
{
	SdramCfg_t c = Board();
	SdramDev_t d = {};
	int resets = s_EmcResets;

	assert(Run(c, d));
	assert(d.Rate == 73333333U && s_ClkDiv == 2 && s_EmcResets == resets + 1);
	Check(c, d);
	// The values of the NXP SDK board at this clock: address map 0x09, RAS
	// 2, mode register 0x33
	assert(s_E.DynCfg[0] == ((0x09U << 7) | EMC_DYNAMIC_DYNAMICCONFIG_B_MASK) && s_E.RasCas[0] == 0x302U);
	assert(s_E.Rp == 1 && s_E.Ras == 3 && s_E.Srex == 4 && s_E.Apr == 1 && s_E.Wr == 1 && s_E.Dal == 4);
	assert(s_E.Rc == 4 && s_E.Rfc == 4 && s_E.Xsr == 4 && s_E.Rrd == 0 && s_E.Mrd == 1 && s_E.Refresh == 71);
	assert(s_E.ModeAddr == EMC_DYCS0_BASE + (0x33U << 12));
	assert(d.pBase == (void *)EMC_DYCS0_BASE && d.Size == 16UL * 1024UL * 1024UL);
	for (const IOPinCfg_t &p : s_Pins)
	{
		uint32_t v = IOCON->PIO[p.PortNo][p.PinNo];
		assert((v & IOCON_PIO_FUNC_MASK) == IOCON_PIO_FUNC(6) && (v & IOCON_PIO_SLEW_MASK));
	}
	assert(s_Ahb[2] & SYSCON_AHBCLKCTRL_EMC_MASK);

	// The C++ class
	Sdram ram;
	assert(ram.Base() == nullptr && ram.Size() == 0 && ram.Rate() == 0);
	assert(ram.Init(c));
	assert(ram.Base() == (void *)EMC_DYCS0_BASE && ram.Size() == 16UL * 1024UL * 1024UL && ram.Rate() == 73333333U);
	printf("board: PASS\n");
}

static void TestClock()
{
	SdramCfg_t c = Board();
	SdramDev_t d = {};
	struct { uint32_t Req, Div; } t[] = {
		{ 75000000, 3 }, { 110000000, 2 }, { 100000000, 2 }, { 165000000, 2 }, { 165000001, 1 },
		{ 300000000, 1 }, { 0, 256 }, { 500000, 256 }, { 1000000, 220 }, { 60000000, 4 },
	};
	for (auto &e : t)
	{
		c.Rate = e.Req;
		assert(Run(c, d));
		assert(s_ClkDiv == e.Div - 1U && d.Rate == 220000000U / e.Div);
		Check(c, d);
	}
	printf("clock: PASS\n");
}

static void TestMaps()
{
	SdramDev_t d = {};
	int n = 0;

	for (const Part &p : s_Parts)
	{
		for (uint8_t bus = 16; bus <= 32; bus += 16)
		{
			for (uint8_t cas = 1; cas <= 3; cas++)
			{
				SdramCfg_t c = Board();

				c.DevWidth = p.Width;
				c.NbBanks = p.Banks;
				c.RowBits = p.Rows;
				c.ColBits = p.Cols;
				c.BusWidth = bus;
				c.CasLatency = cas;
				c.DevNo = n++ % 4;

				uint64_t size = ((uint64_t)p.Banks << (p.Rows + p.Cols)) * (bus / 8);
				bool ok = p.Width <= bus && size <= 0x08000000U;

				assert(Run(c, d) == ok);
				if (ok)
				{
					Check(c, d);
				}
			}
		}
	}
	printf("address maps: PASS\n");
}

static void TestTimings()
{
	SdramCfg_t c = Board();
	SdramDev_t d = {};

	// Values above the fields take their largest value
	c.TrcdNs = c.TrpNs = c.TrasNs = c.TsrexNs = c.TaprNs = c.TwrNs = c.TrcNs = c.TrfcNs = c.TxsrNs = c.TrrdNs = 65535;
	c.TwrClk = 255;
	c.TmrdClk = 16;
	c.RefreshNs = 0xFFFFFFFFU;
	assert(Run(c, d));
	Check(c, d);
	assert(s_E.Rp == 15 && s_E.Ras == 15 && s_E.Srex == 15 && s_E.Wr == 15 && s_E.Dal == 15);
	assert(s_E.Rc == 31 && s_E.Xsr == 31 && s_E.Mrd == 15 && s_E.Refresh == 0x7FF && (s_E.RasCas[0] & 3U) == 3);

	// Zero, tRCD one clock at least
	c = Board();
	c.TrcdNs = c.TrpNs = c.TrasNs = c.TsrexNs = c.TaprNs = c.TwrNs = c.TrcNs = c.TrfcNs = c.TxsrNs = c.TrrdNs = 0;
	c.TwrClk = 0;
	c.TmrdClk = 1;
	c.RefreshNs = 0;
	assert(Run(c, d));
	Check(c, d);
	assert(s_E.Rp == 0 && s_E.Wr == 0 && s_E.Dal == 0 && s_E.Mrd == 0 && s_E.Refresh == 1 && (s_E.RasCas[0] & 3U) == 1);

	// A different value for each
	c = Board();
	c.TrcdNs = 20;
	c.TrpNs = 20;
	c.TrasNs = 45;
	c.TsrexNs = 100;
	c.TaprNs = 30;
	c.TwrNs = 9;
	c.TrcNs = 160;
	c.TrfcNs = 210;
	c.TxsrNs = 260;
	c.TrrdNs = 40;
	assert(Run(c, d));
	Check(c, d);
	assert(s_E.Rp == 1 && s_E.Ras == 3 && s_E.Srex == 7 && s_E.Apr == 2 && s_E.Rc == 11 && s_E.Rfc == 15 &&
		   s_E.Xsr == 19 && s_E.Rrd == 2 && (s_E.RasCas[0] & 3U) == 2);

	// tRCD at 1, 2, 3 clocks and above
	static const struct { uint16_t Ns; uint32_t Ras; } rcd[] = { { 13, 1 }, { 14, 2 }, { 27, 2 }, { 28, 3 }, { 41, 3 }, { 42, 3 } };
	for (auto &e : rcd)
	{
		c = Board();
		c.TrcdNs = e.Ns;
		assert(Run(c, d));
		Check(c, d);
		assert((s_E.RasCas[0] & 3U) == e.Ras);
	}

	// A slow clock still gives 8 refreshes before the mode register
	c = Board();
	c.Rate = 1000000;
	assert(Run(c, d));
	Check(c, d);
	c.Rate = 0;
	assert(Run(c, d));
	Check(c, d);
	SystemCoreClock = 12000000;
	assert(Run(c, d));
	assert(d.Rate == 46875U);
	Check(c, d);
	SystemCoreClock = 220000000;
	printf("timings: PASS\n");
}

static void TestRefused()
{
	SdramCfg_t c;
	SdramDev_t d = {};
	int resets = s_EmcResets;

	d.Rate = 12345;
	assert(SdramInit(NULL, NULL) == false);
	c = Board(); assert(SdramInit(NULL, &c) == false);
	assert(SdramInit(&d, NULL) == false);
	c = Board(); c.pIOPinMap = NULL; assert(Run(c, d) == false);
	c = Board(); c.NbIOPins = 0; assert(Run(c, d) == false);
	c = Board(); c.DevNo = -1; assert(Run(c, d) == false);
	c = Board(); c.DevNo = 4; assert(Run(c, d) == false);
	c = Board(); c.BusWidth = 8; c.DevWidth = 8; assert(Run(c, d) == false);
	c = Board(); c.BusWidth = 24; assert(Run(c, d) == false);
	c = Board(); c.BusWidth = 64; assert(Run(c, d) == false);
	c = Board(); c.DevWidth = 32; c.RowBits = 12; c.ColBits = 8; assert(Run(c, d) == false);
	c = Board(); c.DevWidth = 4; assert(Run(c, d) == false);
	c = Board(); c.NbBanks = 2; assert(Run(c, d) == false);
	c = Board(); c.NbBanks = 8; assert(Run(c, d) == false);
	c = Board(); c.RowBits = 14; assert(Run(c, d) == false);
	c = Board(); c.ColBits = 7; assert(Run(c, d) == false);
	c = Board(); c.BusWidth = 32; c.DevWidth = 8; c.RowBits = 13; c.ColBits = 11; assert(Run(c, d) == false);
	c = Board(); c.CasLatency = 0; assert(Run(c, d) == false);
	c = Board(); c.CasLatency = 4; assert(Run(c, d) == false);
	c = Board(); c.TmrdClk = 0; assert(Run(c, d) == false);
	c = Board(); c.TmrdClk = 17; assert(Run(c, d) == false);
	assert(s_EmcResets == resets && d.Rate == 12345);

	// The limits themselves
	c = Board(); c.DevNo = 3; assert(Run(c, d) && d.pBase == (void *)EMC_DYCS3_BASE);
	c = Board(); c.CasLatency = 1; c.TmrdClk = 16; assert(Run(c, d));
	c = Board(); c.BusWidth = 32; c.DevWidth = 16; c.RowBits = 13; c.ColBits = 10;
	assert(Run(c, d) && d.Size == 0x08000000U);
	printf("refused: PASS\n");
}

int main()
{
	TestBoard();
	TestClock();
	TestMaps();
	TestTimings();
	TestRefused();
	printf("emc: PASS\n");

	return 0;
}
