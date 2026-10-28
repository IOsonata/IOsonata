// Host regression of the LPC546xx SPIFI driver with a model of the SPIFI and
// of a serial NOR flash, then the generic Nvm and Flash drivers on it.
//
// SPIFI model: the registers need the SPIFI bus clock (AHBCLKCTRL0) and the
// SPIFI out of reset (PRESETCTRL0), the reset clears it. CTRL, ADDR and IDATA
// are written only when no command runs, CTRL never with MODE3, RFCLK and
// FBCLK all set, nor DMAEN or INTEN. CMD starts a command when none runs, in
// command mode (MCINIT 0), with the SPIFI clock from the main clock
// (SPIFICLKSEL 0) and its divider written, POLL 0, FRAMEFORM 1 to 5 (opcode,
// 0 to 4 address bytes), opcode serial. FIELDFORM puts the data (1), or the
// address, intermediate bytes and data (2), on the dual or quad lines, CTRL
// DUAL selects 2 or 4 lines. Each DATA byte access moves one byte of DATALEN,
// read for an input command, written for an output command, and waits for
// the byte on the lines (the clock is the main clock / (DIV + 1)). An access
// beyond DATALEN or in the wrong direction would hold the bus forever, it
// fails the test. The command ends when its clocks are out and its data
// moved; STAT CMD is set until then. STAT RESET aborts the command and the
// memory mode, it reads 1 two more times. MCMD and CLIMIT are not used.
// Faults: a command that never ends, a RESET that never ends.
//
// Flash model, Winbond W25Q like: JEDEC id, status registers 1 and 2 (QE bit
// 1 of register 2, or bit 6 of register 1 for a Macronix like part), the
// write enable latch, 3 or 4 byte addresses (EN4B), reads 0x03, 0x0B, 0x3B,
// 0xBB, 0x6B and 0xEB with their address lines, dummy cycles and data lines,
// programs 0x02, 0x32 and 0x38 wrapping in their 256 byte page (the last
// bytes of a longer one stay) and only clearing bits, erases 0x20, 0xD8 and 0xC7, the 4 byte address forms. A
// command with a wrong address size, wrong lines, wrong dummy cycles, a quad
// command without QE, a write without the latch, a command other than a
// status read or write disable while busy (the part ignores them), or 0xEB
// with the continuous read mode bits, fails the test.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

#include "LPC546xx.h"
#include "coredev/spi.h"
#include "storage/nvm.h"
#include "storage/flash.h"

uint32_t SystemCoreClock = 220000000;
uint32_t g_IrqMasked = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { (void)n; (void)p; }
void NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
void NVIC_DisableIRQ(IRQn_Type n) { (void)n; }

static uint64_t s_Ns = 0;

void ModelDelayUs(uint32_t Us)
{
	s_Ns += (uint64_t)Us * 1000U;
}

alignas(8) static uint8_t s_IoconMem[sizeof(IOCON_Type)];
alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];
alignas(8) static uint8_t s_PintMem[sizeof(PINT_Type)];
alignas(8) static uint8_t s_InputMuxMem[sizeof(INPUTMUX_Type)];

// ---- Flash ----

#define FLASH_MEM_SIZE		(32U * 1024U * 1024U)

static uint8_t s_Mem[FLASH_MEM_SIZE];

struct FlashM {
	uint32_t Size = 16U * 1024U * 1024U;
	uint8_t Id[3] = { 0xEF, 0x40, 0x18 };
	uint8_t Sr1 = 0, Sr2 = 0;
	bool QeSr2 = true;			// QE in status register 2 bit 1, else 1 bit 6
	bool Wel = false, Addr4 = false, ResetEn = false;
	uint64_t BusyEnd = 0;
	int Programs = 0, Erases = 0;
	int Ops[256] = {};
};
static FlashM s_F;

static bool FlashBusy() { return s_Ns < s_F.BusyEnd; }

static bool FlashQe()
{
	return s_F.QeSr2 ? (s_F.Sr2 & 2U) != 0 : (s_F.Sr1 & 0x40U) != 0;
}

enum { DIR_NONE, DIR_IN, DIR_OUT };

struct Expect {
	int AddrBytes, AddrLines, Dummy, DataLines, Dir;
	bool Quad;
};

// What the flash expects of an opcode, AddrBytes -1 for the current mode
static bool FlashExpect(uint8_t Op, Expect &E)
{
	int a = -1;

	switch (Op)
	{
		case 0x9F: case 0x05: case 0x35: E = { 0, 1, 0, 1, DIR_IN, false }; return true;
		case 0x01: case 0x31: E = { 0, 1, 0, 1, DIR_OUT, false }; return true;
		case 0x06: case 0x04: case 0x66: case 0x99: case 0xB7: case 0xE9: case 0xC7: case 0x60:
			E = { 0, 1, 0, 1, DIR_NONE, false }; return true;
		case 0x03: E = { a, 1, 0, 1, DIR_IN, false }; return true;
		case 0x0B: E = { a, 1, 8, 1, DIR_IN, false }; return true;
		case 0x3B: E = { a, 1, 8, 2, DIR_IN, false }; return true;
		case 0xBB: E = { a, 2, 4, 2, DIR_IN, false }; return true;
		case 0x6B: E = { a, 1, 8, 4, DIR_IN, true }; return true;
		case 0xEB: E = { a, 4, 6, 4, DIR_IN, true }; return true;
		case 0x02: E = { a, 1, 0, 1, DIR_OUT, false }; return true;
		case 0x32: E = { a, 1, 0, 4, DIR_OUT, true }; return true;
		case 0x38: E = { a, 4, 0, 4, DIR_OUT, true }; return true;
		case 0x20: case 0x52: case 0xD8: E = { a, 1, 0, 1, DIR_NONE, false }; return true;
		// 4 byte address forms
		case 0x13: E = { 4, 1, 0, 1, DIR_IN, false }; return true;
		case 0x6C: E = { 4, 1, 8, 4, DIR_IN, true }; return true;
		case 0xEC: E = { 4, 4, 6, 4, DIR_IN, true }; return true;
		case 0x12: E = { 4, 1, 0, 1, DIR_OUT, false }; return true;
		case 0x34: E = { 4, 1, 0, 4, DIR_OUT, true }; return true;
		case 0x3E: E = { 4, 4, 0, 4, DIR_OUT, true }; return true;
		case 0x21: case 0xDC: E = { 4, 1, 0, 1, DIR_NONE, false }; return true;
		default: return false;
	}
}

static uint8_t FlashSr1()
{
	return (uint8_t)((s_F.Sr1 & 0xFCU) | (s_F.Wel ? 2U : 0U) | (FlashBusy() ? 1U : 0U));
}

static void FlashStartBusy(uint32_t Us)
{
	s_F.BusyEnd = s_Ns + (uint64_t)Us * 1000U;
	s_F.Wel = false;
}

static void FlashErase(uint32_t Addr, uint32_t Size)
{
	assert(s_F.Wel);
	Addr &= ~(Size - 1U);
	assert(Addr + Size <= s_F.Size);
	memset(s_Mem + Addr, 0xFF, Size);
	s_F.Erases++;
	FlashStartBusy(Size >= 65536U ? 2000U : 300U);
}

// A command without data, at its end
static void FlashExec(uint8_t Op, uint32_t Addr)
{
	switch (Op)
	{
		case 0x05: break;				// write disable ignored while busy
		case 0x06: s_F.Wel = true; break;
		case 0x04: s_F.Wel = false; break;
		case 0x66: s_F.ResetEn = true; break;
		case 0x99:
			if (s_F.ResetEn)
			{
				s_F.Wel = false;
			}
			break;
		case 0xB7: s_F.Addr4 = true; break;
		case 0xE9: s_F.Addr4 = false; break;
		case 0x20: case 0x21: FlashErase(Addr, 4096U); break;
		case 0x52: FlashErase(Addr, 32768U); break;
		case 0xD8: case 0xDC: FlashErase(Addr, 65536U); break;
		case 0xC7: case 0x60: FlashErase(0, s_F.Size); break;
		default: assert(false);
	}
	if (Op != 0x66)
	{
		s_F.ResetEn = false;
	}
}

// The data of an output command, at its end
static void FlashWrite(uint8_t Op, uint32_t Addr, const uint8_t *pData, uint32_t Len)
{
	assert(s_F.Wel);
	switch (Op)
	{
		case 0x01:
			assert(Len >= 1 && Len <= 2);
			s_F.Sr1 = pData[0] & 0xFCU;
			if (Len > 1)
			{
				s_F.Sr2 = pData[1];
			}
			FlashStartBusy(20);
			break;
		case 0x31:
			assert(Len == 1);
			s_F.Sr2 = pData[0];
			FlashStartBusy(20);
			break;
		default:
		{
			uint32_t page = Addr & ~0xFFU;
			uint8_t buf[256];
			bool set[256] = {};

			// More than a page wraps, the last bytes stay
			assert(page < s_F.Size);
			for (uint32_t i = 0; i < Len; i++)
			{
				buf[(Addr + i) & 0xFFU] = pData[i];
				set[(Addr + i) & 0xFFU] = true;
			}
			for (uint32_t i = 0; i < 256U; i++)
			{
				if (set[i])
				{
					s_Mem[page + i] &= buf[i];
				}
			}
			s_F.Programs++;
			FlashStartBusy(100);
			break;
		}
	}
}

static uint8_t FlashRead(uint8_t Op, uint32_t Addr, uint32_t Idx)
{
	switch (Op)
	{
		case 0x9F: return s_F.Id[Idx % 3U];
		case 0x05: return FlashSr1();
		case 0x35: return s_F.Sr2;
		default: return s_Mem[(Addr + Idx) % s_F.Size];
	}
}

// ---- SPIFI ----

struct CmdRec {
	uint8_t Op;
	int AddrBytes, AddrLines, Dummy, DataLines;
	uint32_t Addr, Len;
	bool Dout;
};

struct SpifiM {
	uint32_t Ctrl = 0, Addr = 0, Idata = 0;
	bool Cmd = false, Mcinit = false, Intrq = false;
	int ResetLeft = 0;
	// Command in progress
	uint8_t Op = 0;
	uint32_t Len = 0, Done = 0, CmdAddr = 0;
	bool Dout = false;
	int DataLines = 1;
	uint64_t Start = 0, Pre = 0, End = 0;	// time, clocks before the data
	uint8_t Out[16384];
	// Faults
	bool Stuck = false, ResetStuck = false;
	bool Hang = false;			// The command in progress never ends
	int Cmds = 0, Aborts = 0;
	std::vector<CmdRec> Log;
};
static SpifiM s_P;

static uint32_t s_Ahb[3], s_Rst[3];
static uint32_t s_ClkSel = 7, s_ClkDiv = 0, s_AhbClkDiv = 0;
static bool s_DivSet = false;
static int s_SpifiResets = 0;

static double ClkNs()
{
	return 1e9 * (s_ClkDiv + 1U) / ((double)SystemCoreClock * (s_AhbClkDiv + 1U));
}

static void SpifiReset()
{
	s_P.Ctrl = 0;
	s_P.Addr = 0;
	s_P.Idata = 0;
	s_P.Cmd = false;
	s_P.Mcinit = false;
	s_P.Intrq = false;
	s_P.ResetLeft = 0;
	s_SpifiResets++;
}

static void Process()
{
	if (s_P.Cmd && s_P.Done == s_P.Len && s_Ns >= s_P.End && s_P.Stuck == false && s_P.Hang == false)
	{
		s_P.Cmd = false;
		s_P.Intrq = true;
		if (s_P.Len == 0)
		{
			FlashExec(s_P.Op, s_P.CmdAddr);
		}
		else if (s_P.Dout)
		{
			FlashWrite(s_P.Op, s_P.CmdAddr, s_P.Out, s_P.Len);
		}
	}
}

static void Access()
{
	assert(s_Ahb[0] & SYSCON_AHBCLKCTRL_SPIFI_MASK);
	assert((s_Rst[0] & SYSCON_PRESETCTRL_SPIFI_RST_MASK) == 0);
	s_Ns += 5;
	Process();
}

static void CmdWrite(uint32_t v)
{
	Access();
	assert(!s_P.Cmd && !s_P.Mcinit && s_P.ResetLeft == 0);
	assert(s_ClkSel == 0 && s_DivSet);

	uint32_t len = v & SPIFI_CMD_DATALEN_MASK;
	uint32_t intlen = (v & SPIFI_CMD_INTLEN_MASK) >> SPIFI_CMD_INTLEN_SHIFT;
	uint32_t form = (v & SPIFI_CMD_FIELDFORM_MASK) >> SPIFI_CMD_FIELDFORM_SHIFT;
	uint32_t frame = (v & SPIFI_CMD_FRAMEFORM_MASK) >> SPIFI_CMD_FRAMEFORM_SHIFT;
	uint8_t op = (uint8_t)(v >> SPIFI_CMD_OPCODE_SHIFT);
	int multi = (s_P.Ctrl & SPIFI_CTRL_DUAL_MASK) ? 2 : 4;

	assert((v & SPIFI_CMD_POLL_MASK) == 0);
	assert(frame >= 1 && frame <= 5);
	assert(form != 3);				// opcode serial, the flash is not in QPI

	CmdRec r;
	r.Op = op;
	r.AddrBytes = (int)frame - 1;
	r.AddrLines = form >= 2 ? multi : 1;
	r.Dummy = (int)intlen * 8 / r.AddrLines;
	r.DataLines = form >= 1 ? multi : 1;
	r.Addr = r.AddrBytes == 4 ? s_P.Addr : s_P.Addr & ((1U << (8 * r.AddrBytes)) - 1U);
	r.Len = len;
	r.Dout = (v & SPIFI_CMD_DOUT_MASK) != 0;
	s_P.Log.push_back(r);

	// What the flash expects
	Expect e;
	assert(FlashExpect(op, e));
	if (e.AddrBytes < 0)
	{
		e.AddrBytes = s_F.Addr4 ? 4 : 3;
	}
	if (FlashBusy())
	{
		// The part ignores the others. The legacy Flash driver sends write
		// disable right after a program, which does no harm: the latch is
		// cleared at the end of the program anyway.
		assert(op == 0x05 || op == 0x35 || op == 0x04);
		if (op == 0x04)
		{
			s_F.Ops[op]++;
			s_P.Cmds++;
			s_P.Cmd = true;
			s_P.Op = 0x05;	// ends as a command with nothing to run
			s_P.Len = 0;
			s_P.Done = 0;
			s_P.Start = s_Ns;
			s_P.Pre = 8;
			s_P.End = s_Ns + (uint64_t)(8 * ClkNs() + 0.5);
			return;
		}
	}
	assert(r.AddrBytes == e.AddrBytes);
	if (e.AddrBytes > 0)
	{
		assert(r.AddrLines == e.AddrLines);
	}
	assert(r.Dummy == e.Dummy && (int)intlen * 8 == r.Dummy * r.AddrLines);
	if (e.Dir == DIR_NONE)
	{
		assert(len == 0);
	}
	else
	{
		assert(len > 0 && r.DataLines == e.DataLines && r.Dout == (e.Dir == DIR_OUT));
	}
	if (e.Quad)
	{
		assert(FlashQe());
	}
	if (op == 0xEB || op == 0xEC)
	{
		// Continuous read mode not entered
		assert((s_P.Idata & 0x30U) != 0x20U);
	}
	if (e.Dir == DIR_OUT || (e.Dir == DIR_NONE && op != 0x05))
	{
		// A write, erase or register write: the latch is checked at its end
	}
	s_F.Ops[op]++;

	s_P.Cmd = true;
	s_P.Op = op;
	s_P.Len = len;
	s_P.Done = 0;
	s_P.Dout = r.Dout;
	s_P.DataLines = r.DataLines;
	s_P.CmdAddr = r.Addr;
	s_P.Start = s_Ns;
	s_P.Pre = 8U + (uint64_t)r.AddrBytes * 8U / r.AddrLines + r.Dummy;
	s_P.End = s_Ns + (uint64_t)((s_P.Pre + (uint64_t)len * 8U / r.DataLines) * ClkNs() + 0.5);
	s_P.Cmds++;
}

// Time of the end of data byte Idx
static uint64_t ByteNs(uint32_t Idx)
{
	return s_P.Start + (uint64_t)((s_P.Pre + (uint64_t)(Idx + 1U) * 8U / s_P.DataLines) * ClkNs() + 0.5);
}

static uint8_t DataRead()
{
	Access();
	assert(s_P.Cmd && !s_P.Dout && s_P.Done < s_P.Len);
	uint64_t t = ByteNs(s_P.Done);
	if (s_Ns < t)
	{
		s_Ns = t;
	}
	uint8_t d = FlashRead(s_P.Op, s_P.CmdAddr, s_P.Done);
	s_P.Done++;
	Process();
	return d;
}

static void DataWrite(uint8_t v)
{
	Access();
	assert(s_P.Cmd && s_P.Dout && s_P.Done < s_P.Len);
	uint64_t t = ByteNs(s_P.Done);
	if (s_Ns < t)
	{
		s_Ns = t;
	}
	s_P.Out[s_P.Done++] = v;
	Process();
}

struct CtrlR {
	operator uint32_t() const { Access(); return s_P.Ctrl; }
	CtrlR &operator=(uint32_t v)
	{
		Access();
		assert(!s_P.Cmd);
		uint32_t edge = SPIFI_CTRL_MODE3_MASK | SPIFI_CTRL_RFCLK_MASK | SPIFI_CTRL_FBCLK_MASK;
		assert((v & edge) != edge);
		assert((v & (SPIFI_CTRL_DMAEN_MASK | SPIFI_CTRL_INTEN_MASK)) == 0);
		s_P.Ctrl = v;
		return *this;
	}
};
struct CmdR {
	CmdR &operator=(uint32_t v) { CmdWrite(v); return *this; }
};
struct AddrR {
	AddrR &operator=(uint32_t v) { Access(); assert(!s_P.Cmd); s_P.Addr = v; return *this; }
};
struct IdataR {
	IdataR &operator=(uint32_t v) { Access(); assert(!s_P.Cmd); s_P.Idata = v; return *this; }
};
struct StatR {
	operator uint32_t() const
	{
		Access();
		uint32_t st = (s_P.Mcinit ? SPIFI_STAT_MCINIT_MASK : 0U) | (s_P.Cmd ? SPIFI_STAT_CMD_MASK : 0U) |
					  (s_P.ResetLeft > 0 ? SPIFI_STAT_RESET_MASK : 0U) | (s_P.Intrq ? SPIFI_STAT_INTRQ_MASK : 0U);
		if (s_P.ResetLeft > 0 && s_P.ResetStuck == false)
		{
			s_P.ResetLeft--;
		}
		return st;
	}
	StatR &operator=(uint32_t v)
	{
		Access();
		if (v & SPIFI_STAT_RESET_MASK)
		{
			// The command, if any, is cut: the flash does not run it
			s_P.Cmd = false;
			s_P.Hang = false;
			s_P.Mcinit = false;
			s_P.ResetLeft = 2;
			s_P.Aborts++;
		}
		if (v & SPIFI_STAT_INTRQ_MASK)
		{
			s_P.Intrq = false;
		}
		return *this;
	}
};
struct NoR {
	NoR &operator=(uint32_t) { assert(false); return *this; }
	operator uint32_t() const { assert(false); return 0; }
};
struct SpifiRegs {
	CtrlR CTRL;
	CmdR CMD;
	AddrR ADDR;
	IdataR IDATA;
	NoR CLIMIT;
	NoR DATA;
	NoR MCMD;
	StatR STAT;
};
static SpifiRegs s_SpifiRegs;

struct Data8R {
	operator uint8_t() const { return DataRead(); }
	Data8R &operator=(uint8_t v) { DataWrite(v); return *this; }
};
static Data8R s_Data8;

enum { AHB_SET, AHB_CLR, RST_SET, RST_CLR };

static void SysWrite(int Kind, int Idx, uint32_t v)
{
	switch (Kind)
	{
		case AHB_SET: s_Ahb[Idx] |= v; break;
		case AHB_CLR: s_Ahb[Idx] &= ~v; break;
		case RST_SET:
			if (Idx == 0 && (v & SYSCON_PRESETCTRL_SPIFI_RST_MASK))
			{
				SpifiReset();
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
struct ClkSelR {
	operator uint32_t() const { return s_ClkSel; }
	ClkSelR &operator=(uint32_t v) { s_ClkSel = v & SYSCON_SPIFICLKSEL_SEL_MASK; return *this; }
};
struct ClkDivR {
	operator uint32_t() const { return s_ClkDiv; }
	ClkDivR &operator=(uint32_t v)
	{
		// The divider changes only while no command runs
		assert(!s_P.Cmd);
		assert((v & ~SYSCON_SPIFICLKDIV_DIV_MASK) == 0);
		s_ClkDiv = v;
		s_DivSet = true;
		return *this;
	}
};
struct AhbDivR {
	operator uint32_t() const { return s_AhbClkDiv; }
};
struct SysRegs {
	AhbR AHBCLKCTRL[3] = { { 0 }, { 1 }, { 2 } };
	WReg AHBCLKCTRLSET[3] = { { AHB_SET, 0 }, { AHB_SET, 1 }, { AHB_SET, 2 } };
	WReg AHBCLKCTRLCLR[3] = { { AHB_CLR, 0 }, { AHB_CLR, 1 }, { AHB_CLR, 2 } };
	WReg PRESETCTRLSET[3] = { { RST_SET, 0 }, { RST_SET, 1 }, { RST_SET, 2 } };
	WReg PRESETCTRLCLR[3] = { { RST_CLR, 0 }, { RST_CLR, 1 }, { RST_CLR, 2 } };
	ClkSelR SPIFICLKSEL;
	ClkDivR SPIFICLKDIV;
	AhbDivR AHBCLKDIV;
};
static SysRegs s_SysRegs;

#undef IOCON
#undef GPIO
#undef SYSCON
#undef PINT
#undef INPUTMUX
#undef SPIFI0
#define IOCON ((IOCON_Type *)s_IoconMem)
#define GPIO ((GPIO_Type *)s_GpioMem)
#define SYSCON (&s_SysRegs)
#define PINT ((PINT_Type *)s_PintMem)
#define INPUTMUX ((INPUTMUX_Type *)s_InputMuxMem)
#define SPIFI0 (&s_SpifiRegs)
#define LPC546XX_SPIFI_DATA8 s_Data8
#define LPC546XX_SPIFI_WAIT_MAX 2000U

#include "../../../ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/spifi_lpc546xx.cpp"

// SPIInit of spi_lpc546xx.cpp gives DevNo 10 to the SPIFI
bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	return Lpc546xxSpifiInit(pDev, pCfgData);
}

// ---- Tests ----

static const IOPinCfg_t s_Pins[] = {
	{ 0, 26, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL },	// SCK
	{ 0, 24, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL },		// IO0
	{ 0, 25, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL },		// IO1
	{ 0, 28, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL },		// IO2
	{ 0, 27, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL },		// IO3
	{ 0, 23, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL },	// CS
};

static SPICfg_t Cfg(SPIPHY Phy = SPIPHY_QUAD_SDR, uint32_t Rate = 80000000)
{
	SPICfg_t c = {};

	c.DevNo = LPC546XX_SPIFI_DEVNO;
	c.Phy = Phy;
	c.Mode = SPIMODE_MASTER;
	c.pIOPinMap = s_Pins;
	c.NbIOPins = 6;
	c.Rate = Rate;
	c.DataSize = 8;
	c.MaxRetry = 5;
	c.BitOrder = SPIDATABIT_MSB;
	c.DataPhase = SPIDATAPHASE_FIRST_CLK;
	c.ClkPol = SPICLKPOL_HIGH;
	c.ChipSel = SPICSEL_AUTO;
	return c;
}

// A flash at its reset state, the SPIFI model idle
static void Fresh(bool QeSr2 = true)
{
	s_F = FlashM();
	s_F.QeSr2 = QeSr2;
	memset(s_Mem, 0xFF, sizeof(s_Mem));
	s_P.Log.clear();
	s_P.Stuck = false;
	s_P.ResetStuck = false;
	s_P.Cmd = false;
	s_P.Mcinit = false;
	s_P.ResetLeft = 0;
	s_Ns += 10000000;
}

static uint32_t Iocon(int Port, int Pin)
{
	return IOCON->PIO[Port][Pin];
}

static int Cmd(SPIDev_t *pDev, uint8_t Op, uint32_t Addr, uint8_t AddrLen, uint8_t *pRx, const uint8_t *pTx,
			   uint32_t Len, uint8_t Dummy)
{
	assert(SPIStartRx(pDev, 0));
	if (QuadSPISendCmd(pDev, Op, Addr, AddrLen, Len, Dummy) == false)
	{
		SPIStopRx(pDev);
		return -1;
	}
	int n = 0;
	if (pRx != NULL)
	{
		n = SPIRxData(pDev, pRx, (int)Len);
	}
	else if (pTx != NULL)
	{
		n = SPITxData(pDev, pTx, (int)Len);
	}
	SPIStopRx(pDev);
	return n;
}

static void WaitFlash(SPIDev_t *pDev)
{
	uint8_t sr;

	do {
		assert(Cmd(pDev, 0x05, (uint32_t)-1, 0, &sr, NULL, 1, 0) == 1);
	} while (sr & 1U);
}

static void TestInit()
{
	SPI spi;
	SPICfg_t c;

	// Refused
	c = Cfg(); c.DevNo = 3; assert(!Lpc546xxSpifiInit(spi, &c));
	c = Cfg(); c.Mode = SPIMODE_SLAVE; assert(!spi.Init(c));
	c = Cfg(SPIPHY_QUAD_DDR); assert(!spi.Init(c));
	c = Cfg(SPIPHY_3WIRE); assert(!spi.Init(c));
	c = Cfg(); c.NbIOPins = 5; assert(!spi.Init(c));
	c = Cfg(); c.pIOPinMap = NULL; assert(!spi.Init(c));
	c = Cfg(); c.ChipSel = SPICSEL_MAN; assert(!spi.Init(c));
	c = Cfg(); c.DataSize = 16; assert(!spi.Init(c));
	c = Cfg(); c.BitOrder = SPIDATABIT_LSB; assert(!spi.Init(c));
	c = Cfg(); c.bIntEn = true; assert(!spi.Init(c));
	c = Cfg(); c.bDmaEn = true; assert(!spi.Init(c));
	c = Cfg(); c.ClkPol = SPICLKPOL_LOW; assert(!spi.Init(c));
	c = Cfg(); c.DataPhase = SPIDATAPHASE_SECOND_CLK; assert(!spi.Init(c));
	assert(!Lpc546xxSpifiInit(NULL, &c));
	assert(!Lpc546xxSpifiInit(spi, NULL));
	assert(s_SpifiResets == 0 && s_Ahb[0] == 0);

	// Mode 0
	Fresh();
	int resets = s_SpifiResets;
	c = Cfg();
	assert(spi.Init(c));
	assert(s_SpifiResets == resets + 1 && (s_Rst[0] & SYSCON_PRESETCTRL_SPIFI_RST_MASK) == 0);
	assert(s_Ahb[0] & SYSCON_AHBCLKCTRL_SPIFI_MASK);
	assert(s_ClkSel == 0 && s_ClkDiv == 2 && spi.Rate() == 73333333U);
	assert(s_P.Ctrl == (SPIFI_CTRL_TIMEOUT(0xFFFFU) | SPIFI_CTRL_CSHIGH(0xFU) | SPIFI_CTRL_RFCLK_MASK |
						SPIFI_CTRL_FBCLK_MASK));
	for (const IOPinCfg_t &p : s_Pins)
	{
		uint32_t v = Iocon(p.PortNo, p.PinNo);
		assert((v & IOCON_PIO_FUNC_MASK) == IOCON_PIO_FUNC(6));
		assert((v & IOCON_PIO_SLEW_MASK) && (v & IOCON_PIO_DIGIMODE_MASK) && (v & IOCON_PIO_FILTEROFF_MASK));
	}
	assert(((DevIntrf_t *)spi)->Type == DEVINTRF_TYPE_QSPI);
	assert(SPIGetHandle(spi) == (SPIDev_t *)spi);

	// Mode 3
	c.ClkPol = SPICLKPOL_LOW;
	c.DataPhase = SPIDATAPHASE_SECOND_CLK;
	assert(spi.Init(c));
	assert(s_P.Ctrl == (SPIFI_CTRL_TIMEOUT(0xFFFFU) | SPIFI_CTRL_CSHIGH(0xFU) | SPIFI_CTRL_MODE3_MASK |
						SPIFI_CTRL_FBCLK_MASK));
	printf("init: PASS\n");
}

static void TestRate()
{
	SPI spi;
	SPICfg_t c = Cfg();

	Fresh();
	assert(spi.Init(c));
	struct { uint32_t Req, Div; } t[] = {
		{ 96000000, 2 }, { 80000000, 3 }, { 60000000, 4 }, { 1000000, 220 }, { 500000, 256 },
		{ 0, 256 }, { 300000000, 1 }, { 220000000, 1 }, { 110000001, 2 }, { 164999999, 2 }, { 165000000, 2 }, { 165000001, 1 },
	};
	for (auto &e : t)
	{
		uint32_t r = spi.Rate(e.Req);
		assert(s_ClkDiv == e.Div - 1U && r == 220000000U / e.Div && spi.Rate() == r);
		assert(((SPIDev_t *)spi)->Cfg.Rate == r);
	}
	// The main clock is the core clock times the AHB divider
	SystemCoreClock = 110000000;
	s_AhbClkDiv = 1;
	assert(spi.Rate(73000000) == 73333333U && s_ClkDiv == 2);
	SystemCoreClock = 220000000;
	s_AhbClkDiv = 0;
	printf("rate: PASS\n");
}

static void TestCmd()
{
	SPI spi;
	SPIDev_t *dev;
	uint8_t id[4], b[16];

	Fresh();
	assert(spi.Init(Cfg()));
	dev = spi;

	// Id: opcode only, 3 bytes in
	memset(id, 0, sizeof(id));
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, id, NULL, 3, 0) == 3);
	assert(id[0] == 0xEF && id[1] == 0x40 && id[2] == 0x18);
	CmdRec r = s_P.Log.back();
	assert(r.Op == 0x9F && r.AddrBytes == 0 && r.Len == 3 && !r.Dout && r.DataLines == 1);

	// A command without data runs at once
	assert(SPIStartTx(dev, 0));
	assert(QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	SPIStopTx(dev);
	assert(s_F.Wel && !s_P.Cmd);
	assert(QuadSPISendCmd(dev, 0x04, (uint32_t)-1, 0, 0, 0) && !s_F.Wel);

	// Quad enable in status register 2
	b[0] = 0x02;
	assert(QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(Cmd(dev, 0x31, (uint32_t)-1, 0, NULL, b, 1, 0) == 1);
	WaitFlash(dev);
	assert(FlashQe());

	// The line modes of the reads, 3 byte address
	for (uint32_t i = 0; i < 64; i++)
	{
		s_Mem[0x123400 + i] = (uint8_t)(i * 7 + 1);
	}
	struct { uint8_t Op, Dummy; int AddrLines, DataLines; uint32_t Dual; } m[] = {
		{ 0x03, 0, 1, 1, 0 }, { 0x0B, 8, 1, 1, 0 }, { 0x3B, 8, 1, 2, SPIFI_CTRL_DUAL_MASK },
		{ 0xBB, 4, 2, 2, SPIFI_CTRL_DUAL_MASK }, { 0x6B, 8, 1, 4, 0 }, { 0xEB, 6, 4, 4, 0 },
	};
	for (auto &e : m)
	{
		memset(b, 0, sizeof(b));
		assert(Cmd(dev, e.Op, 0x123405, 3, b, NULL, 9, e.Dummy) == 9);
		for (int i = 0; i < 9; i++)
		{
			assert(b[i] == (uint8_t)((5 + i) * 7 + 1));
		}
		r = s_P.Log.back();
		assert(r.Op == e.Op && r.AddrBytes == 3 && r.Addr == 0x123405 && r.AddrLines == e.AddrLines &&
			   r.Dummy == e.Dummy && r.DataLines == e.DataLines && r.Len == 9);
		assert((s_P.Ctrl & SPIFI_CTRL_DUAL_MASK) == e.Dual);
	}

	// Programs and erase
	static const uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	struct { uint8_t Op; int AddrLines, DataLines; } w[] = { { 0x02, 1, 1 }, { 0x32, 1, 4 }, { 0x38, 4, 4 } };
	uint32_t addr = 0x200000;
	for (auto &e : w)
	{
		assert(QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
		assert(Cmd(dev, e.Op, addr, 3, NULL, data, 8, 0) == 8);
		r = s_P.Log.back();
		assert(r.Op == e.Op && r.Dout && r.AddrLines == e.AddrLines && r.DataLines == e.DataLines);
		WaitFlash(dev);
		assert(memcmp(s_Mem + addr, data, 8) == 0);
		addr += 0x100;
	}
	assert(QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(QuadSPISendCmd(dev, 0x20, 0x200010, 3, 0, 0));
	r = s_P.Log.back();
	assert(r.Op == 0x20 && r.AddrBytes == 3 && r.Addr == 0x200010 && r.Len == 0);
	WaitFlash(dev);
	assert(s_Mem[0x200000] == 0xFF && s_F.Erases == 1);

	// Address size from the memory size when the command gives none
	assert(QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(QuadSPISendCmd(dev, 0xB7, (uint32_t)-1, 0, 0, 0) && s_F.Addr4);
	s_F.Size = 32U * 1024U * 1024U;
	QuadSPISetMemSize(dev, 32U * 1024U);
	s_Mem[0x1800000] = 0x5A;
	assert(Cmd(dev, 0x03, 0x1800000, 0, b, NULL, 1, 0) == 1 && b[0] == 0x5A);
	assert(s_P.Log.back().AddrBytes == 4 && s_P.Log.back().Addr == 0x1800000);
	assert(Cmd(dev, 0xEC, 0x1800000, 4, b, NULL, 1, 6) == 1 && b[0] == 0x5A);
	QuadSPISetMemSize(dev, 16U * 1024U);
	assert(QuadSPISendCmd(dev, 0xE9, (uint32_t)-1, 0, 0, 0) && !s_F.Addr4);
	assert(Cmd(dev, 0x03, 0x000001, 0, b, NULL, 1, 0) == 1 && s_P.Log.back().AddrBytes == 3);
	s_F.Size = 16U * 1024U * 1024U;

	// Refused: dummy cycles that are not whole bytes or more than 7 bytes,
	// lines above the Phy, address size
	int cmds = s_P.Cmds;
	assert(!QuadSPISendCmd(dev, 0x6B, 0, 3, 4, 10));
	assert(!QuadSPISendCmd(dev, 0x0B, 0, 3, 4, 4));
	assert(!QuadSPISendCmd(dev, 0xEB, 0, 3, 4, 5));
	assert(!QuadSPISendCmd(dev, 0xEB, 0, 3, 4, 16));
	assert(QuadSPISendCmd(dev, 0xEB, 0, 3, 4, 14));
	assert(!QuadSPISendCmd(dev, 0x03, 0, 5, 4, 0));
	assert(SPIRxData(dev, b, 4) == 0);
	spi.Phy(SPIPHY_NORMAL);
	assert(!QuadSPISendCmd(dev, 0x6B, 0, 3, 4, 8));
	assert(!QuadSPISendCmd(dev, 0x3B, 0, 3, 4, 8));
	spi.Phy(SPIPHY_DUAL);
	assert(!QuadSPISendCmd(dev, 0x6B, 0, 3, 4, 8));
	assert(!QuadSPISendCmd(dev, 0x38, 0, 3, 4, 0));
	assert(Cmd(dev, 0x3B, 0x123405, 3, b, NULL, 2, 8) == 2 && b[0] == 5 * 7 + 1);
	assert(Cmd(dev, 0xBB, 0x123405, 3, b, NULL, 2, 4) == 2 && b[1] == 6 * 7 + 1);
	spi.Phy(SPIPHY_QUAD_SDR);
	assert(s_P.Cmds == cmds + 2);

	// Data calls without a command, more than the command, other chip selects
	assert(SPIRxData(dev, b, 4) == 0);
	assert(SPITxData(dev, data, 4) == 0);
	assert(QuadSPISendCmd(dev, 0x03, 0x123405, 3, 2, 0));
	assert(SPIRxData(dev, NULL, 2) == 0);
	assert(SPIRxData(dev, b, 0) == 0);
	assert(SPIRxData(dev, b, 8) == 2 && b[1] == 6 * 7 + 1);
	assert(SPIRxData(dev, b, 8) == 0);
	assert(QuadSPISendCmd(dev, 0x03, 0x123405, 3, 2, 0));
	SPIStopRx(dev);
	assert(SPIRxData(dev, b, 2) == 0);
	assert(QuadSPISendCmd(dev, 0x03, 0x123405, 3, 2, 0));
	assert(SPIStartRx(dev, 0));
	assert(SPIRxData(dev, b, 2) == 0);
	SPIStopRx(dev);
	assert(!SPIStartRx(dev, 1));
	assert(!SPIStartTx(dev, 1));
	assert(QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(QuadSPISendCmd(dev, 0x02, 0x200000, 3, 4, 0));
	assert(SPITxData(dev, NULL, 4) == 0);
	assert(QuadSPISendCmd(dev, 0x02, 0x200000, 3, 4, 0));
	assert(SPITxData(dev, data, 8) == 4);
	WaitFlash(dev);
	assert(memcmp(s_Mem + 0x200000, data, 4) == 0 && s_Mem[0x200004] == 0xFF);

	// Not the SPIFI
	SPIDev_t other = {};
	other.Cfg.DevNo = 3;
	assert(!QuadSPISendCmd(&other, 0x06, (uint32_t)-1, 0, 0, 0));
	QuadSPISetMemSize(&other, 64U * 1024U);
	assert(s_Lpc546xxSpifiDev.AddrLen == 3);
	SPIDev_t copy = {};
	copy.Cfg = dev->Cfg;
	assert(!QuadSPISendCmd(&copy, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(!QuadSPISendCmd(NULL, 0x06, (uint32_t)-1, 0, 0, 0));
	printf("commands: PASS\n");
}

static void TestLong()
{
	SPI spi;
	SPIDev_t *dev;
	static uint8_t b[40000];

	Fresh();
	assert(spi.Init(Cfg()));
	dev = spi;
	for (uint32_t i = 0; i < sizeof(b); i++)
	{
		s_Mem[0x10000 + i] = (uint8_t)(i ^ (i >> 8));
	}

	// Split in commands of 16383 bytes at the following addresses
	size_t first = s_P.Log.size();
	memset(b, 0, sizeof(b));
	assert(Cmd(dev, 0x03, 0x10000, 3, b, NULL, sizeof(b), 0) == (int)sizeof(b));
	assert(memcmp(b, s_Mem + 0x10000, sizeof(b)) == 0);
	assert(s_P.Log.size() == first + 3);
	assert(s_P.Log[first].Len == 16383 && s_P.Log[first].Addr == 0x10000);
	assert(s_P.Log[first + 1].Len == 16383 && s_P.Log[first + 1].Addr == 0x10000 + 16383);
	assert(s_P.Log[first + 2].Len == 40000 - 2 * 16383 && s_P.Log[first + 2].Addr == 0x10000 + 2 * 16383);

	// A read without address is one command
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, b, NULL, 20000, 0) == 16383);
	assert(s_P.Log.back().Len == 16383 && b[16382] == s_F.Id[16382 % 3]);

	// A write is one command
	assert(QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(QuadSPISendCmd(dev, 0x02, 0x300000, 3, 20000, 0));
	first = s_P.Log.size();
	assert(SPITxData(dev, b, 20000) == 16383);
	assert(s_P.Log.size() == first + 1 && s_P.Log.back().Len == 16383);
	WaitFlash(dev);
	assert(memcmp(s_Mem + 0x300000, b + 16383 - 256 + 1, 255) == 0 && s_Mem[0x300000 + 255] == b[16383 - 256]);

	// A write that does not end
	assert(QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(QuadSPISendCmd(dev, 0x02, 0x300000, 3, 16, 0));
	s_P.Stuck = true;
	int aborts = s_P.Aborts;
	assert(SPITxData(dev, b, 16) == 0);
	assert(s_P.Aborts == aborts + 1);
	s_P.Stuck = false;
	printf("long transfers: PASS\n");
}

static void TestFault()
{
	SPI spi;
	SPIDev_t *dev;
	uint8_t b[4];

	Fresh();
	assert(spi.Init(Cfg()));
	dev = spi;

	// A command that does not end is aborted, the next one runs
	s_P.Stuck = true;
	int aborts = s_P.Aborts;
	assert(!QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(s_P.Aborts == aborts + 1 && !s_F.Wel);
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, b, NULL, 3, 0) == 0);
	assert(s_P.Aborts == aborts + 2);
	s_P.Stuck = false;
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, b, NULL, 3, 0) == 3 && b[0] == 0xEF);

	// Memory mode found active is ended before a command
	s_P.Mcinit = true;
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, b, NULL, 3, 0) == 3 && b[1] == 0x40);
	assert(s_P.Aborts == aborts + 3);

	// A reset that does not end: no command is started
	s_P.Mcinit = true;
	s_P.ResetStuck = true;
	int cmds = s_P.Cmds;
	assert(!QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, b, NULL, 3, 0) == 0);
	assert(s_P.Cmds == cmds);
	s_P.ResetStuck = false;
	s_P.ResetLeft = 0;
	s_P.Mcinit = false;

	// A command left running is waited for, then aborted: before a command
	// and before a rate change
	s_P.Cmd = true;
	s_P.Hang = true;
	s_P.Len = 0;
	s_P.Done = 0;
	aborts = s_P.Aborts;
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, b, NULL, 3, 0) == 3 && b[0] == 0xEF);
	assert(s_P.Aborts == aborts + 1);
	s_P.Cmd = true;
	s_P.Hang = true;
	assert(spi.Rate(55000000) == 55000000U && s_P.Aborts == aborts + 2);

	// Init again: no command kept, 3 address bytes
	assert(QuadSPISendCmd(dev, 0x03, 0, 3, 2, 0));
	QuadSPISetMemSize(dev, 32U * 1024U);
	assert(spi.Init(Cfg()));
	assert(SPIRxData(dev, b, 2) == 0);
	assert(Cmd(dev, 0x03, 0x10, 0, b, NULL, 1, 0) == 1 && s_P.Log.back().AddrBytes == 3);

	// Reset of the interface aborts
	aborts = s_P.Aborts;
	assert(QuadSPISendCmd(dev, 0x03, 0, 3, 2, 0));
	DeviceIntrfReset(spi);
	assert(s_P.Aborts == aborts + 1 && SPIRxData(dev, b, 2) == 0);
	printf("faults: PASS\n");
}

static void TestPower()
{
	SPI spi;
	SPIDev_t *dev;
	uint8_t b[4];

	Fresh();
	assert(spi.Init(Cfg()));
	dev = spi;

	spi.Disable();
	assert((s_Ahb[0] & SYSCON_AHBCLKCTRL_SPIFI_MASK) == 0);
	assert(!QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	assert(!SPIStartRx(dev, 0));
	spi.Disable();
	spi.Enable();
	assert(s_Ahb[0] & SYSCON_AHBCLKCTRL_SPIFI_MASK);
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, b, NULL, 3, 0) == 3 && b[2] == 0x18);

	assert(QuadSPISendCmd(dev, 0x03, 0, 3, 2, 0));
	spi.Disable();
	spi.Enable();
	assert(SPIRxData(dev, b, 2) == 0);

	// The rate is set while the clock is off
	spi.Disable();
	assert(spi.Rate(55000000) == 55000000U && s_ClkDiv == 3);
	spi.Enable();

	int resets = s_SpifiResets;
	DeviceIntrfPowerOff(spi);
	assert(s_SpifiResets == resets + 1 && (s_Ahb[0] & SYSCON_AHBCLKCTRL_SPIFI_MASK) == 0);
	assert((s_Rst[0] & SYSCON_PRESETCTRL_SPIFI_RST_MASK) == 0 && s_ClkSel == 7);
	for (const IOPinCfg_t &p : s_Pins)
	{
		assert(Iocon(p.PortNo, p.PinNo) == 0);
	}
	assert(!QuadSPISendCmd(dev, 0x06, (uint32_t)-1, 0, 0, 0));
	DeviceIntrfPowerOff(spi);
	assert(spi.Init(Cfg()));
	assert(Cmd(dev, 0x9F, (uint32_t)-1, 0, b, NULL, 3, 0) == 3);
	printf("power: PASS\n");
}

// The generic Nvm driver on the SPIFI
static void NvmRun(const NvmCfg_t &Ncfg, SPIPHY Phy, bool QeSr2, uint8_t RdOp, uint8_t WrOp)
{
	SPI spi;
	Nvm nvm;
	static uint8_t wr[9000], rd[9000];

	Fresh(QeSr2);
	assert(spi.Init(Cfg(Phy)));
	assert(nvm.Init(Ncfg, &spi));
	if (Ncfg.QuadEnable.Reg != 0)
	{
		assert(FlashQe());
	}
	for (uint32_t i = 0; i < sizeof(wr); i++)
	{
		wr[i] = (uint8_t)(i * 13 + 5);
	}
	uint32_t off = 0x7F0F3;
	assert(nvm.Erase(off & ~4095U, 16384) == 0);
	assert(nvm.Write(off, wr, sizeof(wr)) == (int)sizeof(wr));
	assert(memcmp(s_Mem + off, wr, sizeof(wr)) == 0);
	memset(rd, 0, sizeof(rd));
	assert(nvm.Read(off, rd, sizeof(rd)) == (int)sizeof(rd));
	assert(memcmp(rd, wr, sizeof(rd)) == 0);
	assert(s_F.Ops[RdOp] > 0 && s_F.Ops[WrOp] > 0);
	assert(nvm.ReadId(3) == Ncfg.DevId);
	printf("nvm 0x%02X/0x%02X: PASS\n", RdOp, WrOp);
}

static void TestNvm()
{
	NvmCfg_t c = {};

	c.TotalSize = 16U * 1024U * 1024U;
	c.EraseSize = 4096;
	c.PageSize = 256;
	c.AddrSize = 3;
	c.DevId = 0x1840EF;
	c.DevIdSize = 3;
	c.WrProtMask = 0x7C;
	c.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };

	// Single line
	NvmRun(c, SPIPHY_NORMAL, true, 0x03, 0x02);

	// Winbond quad: 0x6B and 0x32, QE in status register 2
	c.RdCmd = { 0x6B, 8 };
	c.WrCmd = { 0x32, 0 };
	c.QuadEnable = { 2, 1 };
	NvmRun(c, SPIPHY_QUAD_SDR, true, 0x6B, 0x32);

	// Quad I/O: 0xEB and 0x38, QE in status register 1
	c.RdCmd = { 0xEB, 6 };
	c.WrCmd = { 0x38, 0 };
	c.QuadEnable = { 1, 6 };
	NvmRun(c, SPIPHY_QUAD_SDR, false, 0xEB, 0x38);

	// Dual reads
	c.RdCmd = { 0xBB, 4 };
	c.WrCmd = { 0x02, 0 };
	c.QuadEnable = { 0, 0 };
	NvmRun(c, SPIPHY_DUAL, true, 0xBB, 0x02);
}

// The legacy Flash driver on the SPIFI, the part already in quad mode
static void TestFlash()
{
	SPI spi;
	FlashDev_t fl;
	FlashCfg_t c = {};
	static uint8_t wr[4096], rd[4096];

	Fresh();
	s_F.Sr2 = 0x02;
	assert(spi.Init(Cfg()));
	c.TotalSize = 16U * 1024U;
	c.SectSize = 4096;
	c.BlkSize = 65536;
	c.PageSize = 256;
	c.AddrSize = 3;
	c.DevId = 0x1840EF;
	c.DevIdSize = 3;
	c.RdCmd = { 0x6B, 8 };
	c.WrCmd = { 0x32, 0 };
	assert(FlashInit(&fl, &c, spi));
	for (uint32_t i = 0; i < sizeof(wr); i++)
	{
		wr[i] = (uint8_t)(i * 3 + 11);
	}
	FlashEraseSector(&fl, 5, 1);
	assert(s_Mem[5 * 4096] == 0xFF);
	assert(FlashSectWrite(&fl, 5, wr));
	assert(FlashSectRead(&fl, 5, rd));
	assert(memcmp(rd, wr, sizeof(rd)) == 0 && memcmp(s_Mem + 5 * 4096, wr, sizeof(wr)) == 0);
	assert(s_F.Ops[0x6B] > 0 && s_F.Ops[0x32] == 16);
	printf("flash: PASS\n");
}

int main()
{
	TestInit();
	TestRate();
	TestCmd();
	TestLong();
	TestFault();
	TestPower();
	TestNvm();
	TestFlash();
	printf("spifi: PASS\n");

	return 0;
}
