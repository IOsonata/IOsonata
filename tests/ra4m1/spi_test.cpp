/**-------------------------------------------------------------------------
@file	spi_test.cpp

@brief	Host model test of spi_ra4m1.cpp.

		Two RSPI units are modeled at frame level and wired together: unit
		0 or 1 acts as master, the other as slave. The model has the transmit
		buffer and shift register of the RSPI, SPRF/SPTEF/IDLNF, the error
		flags, SCKASE clock stop, the SPRI, SPII and SPEI events through a
		simple ICU, and PRIMASK. Time passes when the driver reads a status
		register, re-enables interrupts, or the test waits. The model runs
		the interrupt handlers as if the CPU were infinitely fast, so it
		checks the register sequence and the data flow, not the timing of the
		slave against a fast master.

		The real DeviceIntrf and SPI generic code are used. The slave select
		of the model is the level of the master chip select pin.

		Not hardware validation: no exact timing, no electrical test, and the
		frame end behavior of the RSPI slave is a model assumption.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "interrupt_ra4m1.h"
#include "iopinctrl.h"

// ---------------------------------------------------------------------------
// Model
// ---------------------------------------------------------------------------

uint32_t g_TestPrimask;
R_SPI0_Type g_TestSpi[2];
R_SYSTEM_Type g_TestSystem;
R_MSTP_Type g_TestMstp;

typedef struct {
	uint8_t spcr, sslp, sppcr, spbr, spdcr, spckd, sslnd, spnd, spcr2;
	uint16_t spcmd;
	uint8_t err;
	bool rf;
	uint16_t rx;
	bool shiftValid, txValid, inFrame;
	uint16_t shift, tx;
} Unit_t;

static Unit_t g_U[2];
static uint16_t g_Prcr;
static uint32_t g_Mstp = 0xFFFFFFFFU;
static int g_Violations;
static bool g_Loopback;			// MISO = MOSI, no slave
static bool g_IdleFirst;		// deliver the idle event before the receive event
static uint8_t g_Pin[10][16];
static int g_CsEdges[2];		// [0] clear (select), [1] set (deselect)
static const int CS_PORT = 1, CS_PIN = 3;
static unsigned g_Frames;		// frames clocked by the master
static bool g_Hold;				// the master clock does not run
static bool g_Verbose;			// SPI_TEST_VERBOSE in the environment

#define BIT(x)	(1U << (x))

static int UnitOf(const void *p)
{
	const char *c = (const char *)p;

	for (int i = 0; i < 2; i++)
	{
		if (c >= (const char *)&g_TestSpi[i] && c < (const char *)(&g_TestSpi[i] + 1))
		{
			return i;
		}
	}

	return -1;
}

static bool IsMaster(const Unit_t &u)
{
	return (u.spcr & R_SPI0_SPCR_MSTR_Msk) != 0;
}

static bool Spe(const Unit_t &u)
{
	return (u.spcr & R_SPI0_SPCR_SPE_Msk) != 0;
}

static bool CsLow(void)
{
	return g_Pin[CS_PORT][CS_PIN] == 0;
}

static int Bits(const Unit_t &u)
{
	int spb = (u.spcmd >> R_SPI0_SPCMD_SPB_Pos) & 0xF;

	return spb >= 8 ? spb + 1 : 8;
}

// ICU model
typedef struct {
	bool used;
	uint8_t evt;
	Ra4m1IRQHandler_t handler;
	void *ctx;
	bool pending;
} Irq_t;

static Irq_t g_Irq[8];
static bool g_InIrq;

IRQn_Type Ra4m1RegisterIntHandler(uint8_t EvtId, int Prio, Ra4m1IRQHandler_t pHandler, void *pCtx)
{
	(void)Prio;

	for (int i = 0; i < 8; i++)
	{
		if (!g_Irq[i].used)
		{
			g_Irq[i] = {true, EvtId, pHandler, pCtx, false};

			return i;
		}
	}

	return -1;
}

void Ra4m1UnregisterIntHandler(IRQn_Type IrqNo)
{
	if (IrqNo >= 0 && IrqNo < 8)
	{
		g_Irq[IrqNo] = {};
	}
}

bool Ra4m1AcknowledgeInt(IRQn_Type IrqNo)
{
	return IrqNo >= 0 && IrqNo < 8 && g_Irq[IrqNo].used;
}

static void Raise(int unit, int off)
{
	uint8_t evt = (uint8_t)((unit == 0 ? RA4M1_EVTID_SPI0_SPRI : RA4M1_EVTID_SPI1_SPRI) + off);

	for (int i = 0; i < 8; i++)
	{
		if (g_Irq[i].used && g_Irq[i].evt == evt)
		{
			g_Irq[i].pending = true;
		}
	}
}

static bool Deliver(void)
{
	if (g_TestPrimask || g_InIrq)
	{
		return false;
	}

	for (int pass = 0; pass < 8; pass++)
	{
		int pick = -1;

		for (int i = 0; i < 8; i++)
		{
			if (!g_Irq[i].used || !g_Irq[i].pending)
			{
				continue;
			}

			bool idle = (g_Irq[i].evt == RA4M1_EVTID_SPI0_SPII ||
						 g_Irq[i].evt == RA4M1_EVTID_SPI1_SPII);

			if (pick < 0 || (g_IdleFirst && idle))
			{
				pick = i;
			}
		}

		if (pick < 0)
		{
			return pass > 0;
		}

		Irq_t irq = g_Irq[pick];

		g_Irq[pick].pending = false;
		g_InIrq = true;
		g_TestPrimask = 1;
		irq.handler(pick, irq.ctx);
		g_TestPrimask = 0;
		g_InIrq = false;
	}

	return true;
}

static bool SlaveFor(int m, int *pS)
{
	int s = 1 - m;

	*pS = s;

	return Spe(g_U[s]) && !IsMaster(g_U[s]) && CsLow();
}

// One master frame
static bool StepMaster(void)
{
	for (int m = 0; m < 2; m++)
	{
		Unit_t &u = g_U[m];

		if (!IsMaster(u) || !Spe(u) || !u.inFrame || u.rf)
		{
			continue;
		}

		uint16_t mask = (uint16_t)((1UL << Bits(u)) - 1UL);
		uint16_t out = u.shift & mask;
		uint16_t in = mask;		// MISO pulled up
		int s;

		g_Frames++;

		if (g_Loopback)
		{
			in = out;
		}
		else if (SlaveFor(m, &s))
		{
			Unit_t &sl = g_U[s];
			uint16_t smask = (uint16_t)((1UL << Bits(sl)) - 1UL);

			if (sl.shiftValid)
			{
				in = sl.shift & smask;
			}
			else
			{
				in = 0;
				if (g_Verbose)
				{
					printf("model: slave underrun at master frame %u\n", g_Frames);
				}
				sl.err |= R_SPI0_SPSR_UDRF_Msk | R_SPI0_SPSR_MODF_Msk;
				if (sl.spcr & R_SPI0_SPCR_SPEIE_Msk)
				{
					Raise(s, 3);
				}
			}

			if (sl.rf)
			{
				if (g_Verbose)
				{
					printf("model: slave overrun at master frame %u\n", g_Frames);
				}
				sl.err |= R_SPI0_SPSR_OVRF_Msk;
				if (sl.spcr & R_SPI0_SPCR_SPEIE_Msk)
				{
					Raise(s, 3);
				}
			}
			else
			{
				sl.rx = out & smask;
				sl.rf = true;
				if (sl.spcr & R_SPI0_SPCR_SPRIE_Msk)
				{
					Raise(s, 0);
				}
			}

			sl.shiftValid = sl.txValid;
			sl.shift = sl.tx;
			sl.txValid = false;
		}

		u.rx = in;
		u.rf = true;
		if (u.spcr & R_SPI0_SPCR_SPRIE_Msk)
		{
			Raise(m, 0);
		}

		u.shiftValid = u.txValid;
		u.inFrame = u.txValid;
		u.shift = u.tx;
		u.txValid = false;

		return true;
	}

	return false;
}

void TestIrqPoint(void)
{
	// Time passes while interrupts are enabled
	for (int guard = 0; guard < 100000; guard++)
	{
		bool did = Deliver();

		if (!g_TestPrimask && !g_InIrq && !g_Hold)
		{
			did |= StepMaster();
		}

		if (!did)
		{
			break;
		}
	}
}

static void CheckMstp(int unit)
{
	if (unit >= 0 && (g_Mstp & BIT(unit == 0 ? 19 : 18)))
	{
		g_Violations++;
		printf("VIOLATION: SPI%d register access with module stopped\n", unit);
	}
}

static void ResetEngine(Unit_t &u)
{
	u.shiftValid = u.txValid = u.inFrame = false;
}

uint32_t TestRegRead(const void *pReg, int Id)
{
	int n = UnitOf(pReg);

	if (Id == SYSREG_PRCR)
	{
		return g_Prcr;
	}

	if (Id == SYSREG_MSTPCRB)
	{
		return g_Mstp;
	}

	CheckMstp(n);

	Unit_t &u = g_U[n];
	uint32_t val = 0;

	switch (Id)
	{
	case SPIREG_SPCR:
		return u.spcr;
	case SPIREG_SSLP:
		return u.sslp;
	case SPIREG_SPPCR:
		return u.sppcr;
	case SPIREG_SPBR:
		return u.spbr;
	case SPIREG_SPDCR:
		return u.spdcr;
	case SPIREG_SPCKD:
		return u.spckd;
	case SPIREG_SSLND:
		return u.sslnd;
	case SPIREG_SPND:
		return u.spnd;
	case SPIREG_SPCR2:
		return u.spcr2;
	case SPIREG_SPCMD:
		return u.spcmd;
	case SPIREG_SPSR:
		// Time passes for the polling code, not inside a handler or a
		// masked section: the model CPU is infinitely fast there.
		if (!g_TestPrimask && !g_InIrq && !g_Hold)
		{
			StepMaster();
			Deliver();
		}
		val = u.err;
		if (u.rf)
		{
			val |= R_SPI0_SPSR_SPRF_Msk;
		}
		if (Spe(u) && !u.txValid)
		{
			val |= R_SPI0_SPSR_SPTEF_Msk;
		}
		if (Spe(u) && (IsMaster(u) ? (u.inFrame || u.txValid) : CsLow()))
		{
			val |= R_SPI0_SPSR_IDLNF_Msk;
		}
		return val;
	case SPIREG_SPDR_BY:
	case SPIREG_SPDR_HA:
		{
			uint16_t v = u.rx;

			u.rf = false;
			Deliver();

			return Id == SPIREG_SPDR_BY ? (v & 0xFF) : v;
		}
	}

	return 0;
}

void TestRegWrite(const void *pReg, int Id, uint32_t Val)
{
	int n = UnitOf(pReg);

	if (Id == SYSREG_PRCR)
	{
		g_Prcr = (uint16_t)Val;

		return;
	}

	if (Id == SYSREG_MSTPCRB)
	{
		if ((g_Prcr & R_SYSTEM_PRCR_PRC1_Msk) == 0 || (g_Prcr >> 8) != 0xA5)
		{
			g_Violations++;
			printf("VIOLATION: MSTPCRB written without PRC1 unlock\n");
		}
		else
		{
			g_Mstp = Val;
		}

		return;
	}

	CheckMstp(n);

	Unit_t &u = g_U[n];

	switch (Id)
	{
	case SPIREG_SPCR:
		if (Spe(u) && !(Val & R_SPI0_SPCR_SPE_Msk))
		{
			ResetEngine(u);
		}
		u.spcr = (uint8_t)Val;
		break;
	case SPIREG_SSLP:
		u.sslp = (uint8_t)Val;
		break;
	case SPIREG_SPPCR:
		u.sppcr = (uint8_t)Val;
		break;
	case SPIREG_SPBR:
		u.spbr = (uint8_t)Val;
		break;
	case SPIREG_SPDCR:
		u.spdcr = (uint8_t)Val;
		break;
	case SPIREG_SPCKD:
		u.spckd = (uint8_t)Val;
		break;
	case SPIREG_SSLND:
		u.sslnd = (uint8_t)Val;
		break;
	case SPIREG_SPND:
		u.spnd = (uint8_t)Val;
		break;
	case SPIREG_SPCR2:
		u.spcr2 = (uint8_t)Val;
		break;
	case SPIREG_SPCMD:
		u.spcmd = (uint16_t)Val;
		break;
	case SPIREG_SPSR:
		// Writing 0 to a flag clears it
		u.err &= (uint8_t)Val;
		break;
	case SPIREG_SPDR_BY:
	case SPIREG_SPDR_HA:
		if (!Spe(u))
		{
			g_Violations++;
			printf("VIOLATION: SPDR written with SPE = 0\n");
		}
		else if (!u.shiftValid)
		{
			u.shift = (uint16_t)Val;
			u.shiftValid = true;
			u.inFrame = IsMaster(u);
		}
		else if (!u.txValid)
		{
			u.tx = (uint16_t)Val;
			u.txValid = true;
		}
		else
		{
			g_Violations++;
			printf("VIOLATION: SPDR written with SPTEF = 0\n");
		}
		break;
	}
}

// Platform functions
extern "C" uint32_t SystemCoreClockGet(void)
{
	return 48000000U;
}

extern "C" uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx == 0 ? 48000000U : 0U;
}

extern "C" void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)PortNo; (void)PinNo; (void)PinOp; (void)Dir; (void)Resistor; (void)Type;
}

extern "C" void IOPinDisable(int PortNo, int PinNo)
{
	(void)PortNo; (void)PinNo;
}

extern "C" int IOPinRead(int PortNo, int PinNo)
{
	return g_Pin[PortNo][PinNo];
}

extern "C" void IOPinSet(int PortNo, int PinNo)
{
	bool rising = g_Pin[PortNo][PinNo] == 0;

	g_Pin[PortNo][PinNo] = 1;

	if (PortNo == CS_PORT && PinNo == CS_PIN && rising)
	{
		g_CsEdges[1]++;

		for (int i = 0; i < 2; i++)
		{
			if (!IsMaster(g_U[i]) && Spe(g_U[i]) && (g_U[i].spcr2 & R_SPI0_SPCR2_SPIIE_Msk))
			{
				Raise(i, 2);
			}
		}
	}
}

extern "C" void IOPinClear(int PortNo, int PinNo)
{
	if (PortNo == CS_PORT && PinNo == CS_PIN && g_Pin[PortNo][PinNo])
	{
		g_CsEdges[0]++;
	}

	g_Pin[PortNo][PinNo] = 0;
}

// ---------------------------------------------------------------------------
// Test code
// ---------------------------------------------------------------------------

static int g_Fail;

#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); g_Fail++; } } while (0)

static IOPinCfg_t s_MPins[4] = {
	{1, 2, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// SCK P102
	{1, 0, IOPINOP_FUNC5, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// MISO P100
	{1, 1, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// MOSI P101
	{1, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// CS P103
};
static IOPinCfg_t s_SPins[4] = {
	{1, 11, IOPINOP_FUNC5, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// SCK P111
	{1, 10, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// MISO P110
	{1, 9, IOPINOP_FUNC5, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// MOSI P109
	{1, 12, IOPINOP_FUNC5, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// SSL0 P112
};

static uint8_t s_SlaveRx[129];
static uint8_t s_SlaveTx[129];
static int s_SlaveRxLen = 129;
static unsigned s_Arms, s_Completions;
static int s_SlaveCount;
static bool s_MasterDone;
static int s_MasterCount;
static uint8_t *s_MasterBuf;
static unsigned s_MasterEvents;
SPIDev_t g_Slave, g_Master;

static int SlaveEvt(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuf, int Len)
{
	(void)pDev; (void)pBuf;

	if (Evt == DEVINTRF_EVT_STATECHG)
	{
		s_Arms++;
		SPISetSlaveRxBuffer(&g_Slave, 0, s_SlaveRx, s_SlaveRxLen);
		SPISetSlaveTxData(&g_Slave, 0, s_SlaveTx, sizeof(s_SlaveTx));
	}
	else if (Evt == DEVINTRF_EVT_COMPLETED)
	{
		s_Completions++;
		s_SlaveCount = Len;
	}

	return 0;
}

static int MasterEvt(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuf, int Len)
{
	(void)pDev;

	if (Evt == DEVINTRF_EVT_COMPLETED)
	{
		s_MasterEvents++;
		s_MasterDone = true;
		s_MasterCount = Len;
		s_MasterBuf = pBuf;
	}

	return 0;
}

static SPICfg_t MasterCfg(int dev, uint32_t rate, int mode, bool bInt, uint32_t bits = 8)
{
	SPICfg_t c = {dev, SPIPHY_NORMAL, SPIMODE_MASTER, s_MPins, 4, rate, bits, 0, SPIDATABIT_MSB,
		(mode & 1) ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK,
		(mode & 2) ? SPICLKPOL_LOW : SPICLKPOL_HIGH, SPICSEL_AUTO, true, bInt, 6, 0xFF, MasterEvt};

	return c;
}

static SPICfg_t SlaveCfg(int dev, int mode)
{
	SPICfg_t c = {dev, SPIPHY_NORMAL, SPIMODE_SLAVE, s_SPins, 4, 100000, 8, 0, SPIDATABIT_MSB,
		(mode & 1) ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK,
		(mode & 2) ? SPICLKPOL_LOW : SPICLKPOL_HIGH, SPICSEL_AUTO, true, true, 6, 0xFF, SlaveEvt};

	return c;
}

static void Reset(void)
{
	memset(g_U, 0, sizeof(g_U));
	memset(g_Irq, 0, sizeof(g_Irq));
	memset(g_Pin, 0, sizeof(g_Pin));
	memset((void *)&g_Master, 0, sizeof(g_Master));
	memset((void *)&g_Slave, 0, sizeof(g_Slave));
	g_Mstp = 0xFFFFFFFFU;
	g_TestPrimask = 0;
	g_InIrq = false;
	g_Loopback = false;
	g_IdleFirst = false;
	g_Hold = false;
	g_CsEdges[0] = g_CsEdges[1] = 0;
	s_Arms = s_Completions = 0;
	s_MasterEvents = 0;
	s_SlaveRxLen = 129;
	for (unsigned i = 0; i < sizeof(s_SlaveTx); i++)
	{
		s_SlaveTx[i] = (uint8_t)(0xA0 ^ i);
	}
}

static void Idle(void)
{
	TestIrqPoint();
}

static void TestRates(void)
{
	Reset();

	struct { uint32_t req, want; } t[] = {
		{1000000, 1000000}, {24000000, 24000000}, {100000000, 24000000},
		{3000000, 3000000}, {0, 1000000}, {100, 11718}, {400000, 400000},
		{7500000, 8000000},
	};

	for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++)
	{
		SPICfg_t c = MasterCfg(0, t[i].req, 0, false);

		CHECK(SPIInit(&g_Master, &c));
		if (g_Master.Cfg.Rate != t[i].want)
		{
			printf("rate %u -> %u, expected %u\n", t[i].req, g_Master.Cfg.Rate, t[i].want);
			g_Fail++;
		}
		CHECK((uint32_t)SPISetRate(&g_Master, 1000000) == 1000000U);
		CHECK((uint32_t)SPIGetRate(&g_Master) == 1000000U);
		CHECK((uint32_t)SPISetRate(&g_Master, t[i].req) == t[i].want);
		DeviceIntrfPowerOff(&g_Master.DevIntrf);
	}
}

static void TestInitRejects(void)
{
	Reset();

	SPICfg_t c = MasterCfg(2, 1000000, 0, false);
	CHECK(!SPIInit(&g_Master, &c));

	c = MasterCfg(0, 1000000, 0, false, 7);
	CHECK(!SPIInit(&g_Master, &c));

	c = MasterCfg(0, 1000000, 0, false, 17);
	CHECK(!SPIInit(&g_Master, &c));

	// Master chip select must be a GPIO output
	IOPinCfg_t bad[4];
	memcpy(bad, s_MPins, sizeof(bad));
	bad[3].PinOp = IOPINOP_FUNC5;
	c = MasterCfg(0, 1000000, 0, false);
	c.pIOPinMap = bad;
	CHECK(!SPIInit(&g_Master, &c));

	// SCK of SPI1 on a SPI0 pin
	c = MasterCfg(1, 1000000, 0, false);
	CHECK(!SPIInit(&g_Master, &c));

	// Wrong pin function
	memcpy(bad, s_MPins, sizeof(bad));
	bad[0].PinOp = IOPINOP_FUNC4;
	c = MasterCfg(0, 1000000, 0, false);
	c.pIOPinMap = bad;
	CHECK(!SPIInit(&g_Master, &c));

	// Slave: CPHA = 0, no interrupt, 16 bit, SSL on a wrong pin
	c = SlaveCfg(1, 0);
	CHECK(!SPIInit(&g_Slave, &c));
	c = SlaveCfg(1, 2);
	CHECK(!SPIInit(&g_Slave, &c));
	c = SlaveCfg(1, 1);
	c.bIntEn = false;
	CHECK(!SPIInit(&g_Slave, &c));
	c = SlaveCfg(1, 1);
	c.DataSize = 16;
	CHECK(!SPIInit(&g_Slave, &c));
	memcpy(bad, s_SPins, sizeof(bad));
	bad[3].PortNo = 1;
	bad[3].PinNo = 4;
	c = SlaveCfg(1, 1);
	c.pIOPinMap = bad;
	CHECK(!SPIInit(&g_Slave, &c));

	// Nothing may be left running by the failed calls
	CHECK(g_Mstp == 0xFFFFFFFFU || (g_Mstp & (BIT(19) | BIT(18))) == (BIT(19) | BIT(18)));

	// A good slave is accepted in modes 1 and 3
	c = SlaveCfg(1, 1);
	CHECK(SPIInit(&g_Slave, &c));
	CHECK(s_Arms == 1);
	DeviceIntrfPowerOff(&g_Slave.DevIntrf);
	c = SlaveCfg(1, 3);
	CHECK(SPIInit(&g_Slave, &c));
	DeviceIntrfPowerOff(&g_Slave.DevIntrf);
	CHECK((g_Mstp & BIT(18)) != 0);
}

static void TestMasterLoopback(void)
{
	Reset();
	g_Loopback = true;

	SPICfg_t c = MasterCfg(0, 1000000, 0, false);
	CHECK(SPIInit(&g_Master, &c));
	CHECK((g_Mstp & BIT(19)) == 0);
	CHECK(g_Pin[1][3] == 1);		// CS idles high
	g_CsEdges[0] = g_CsEdges[1] = 0;

	uint8_t tx[4] = {0xA5, 1, 2, 3};
	uint8_t rx[4] = {};

	CHECK(SPITx(&g_Master, 0, tx, 4) == 4);
	CHECK(g_Master.FirstRdData == 0xA5);
	CHECK(g_CsEdges[0] == 1 && g_CsEdges[1] == 1);
	CHECK(g_Pin[1][3] == 1);

	// Command phase then receive phase in one chip select frame
	g_CsEdges[0] = g_CsEdges[1] = 0;
	uint8_t cmd[2] = {0x9F, 0x11};
	CHECK(DeviceIntrfRead(&g_Master.DevIntrf, 0, cmd, 2, rx, 4) == 4);
	CHECK(rx[0] == 0xFF && rx[3] == 0xFF);	// dummy byte is looped back
	CHECK(g_Master.FirstRdData == 0x9F);
	CHECK(g_CsEdges[0] == 1 && g_CsEdges[1] == 1);

	// Chip select out of range
	CHECK(SPITx(&g_Master, 1, tx, 4) == 0);

	// Command and data written as one chip select frame
	g_CsEdges[0] = g_CsEdges[1] = 0;
	CHECK(DeviceIntrfWrite(&g_Master.DevIntrf, 0, cmd, 2, tx, 4) == 4);
	CHECK(g_CsEdges[0] == 1 && g_CsEdges[1] == 1);
	DeviceIntrfPowerOff(&g_Master.DevIntrf);
	CHECK((g_Mstp & BIT(19)) != 0);

	// 12 bit frames use two bytes
	c = MasterCfg(0, 1000000, 0, false, 12);
	CHECK(SPIInit(&g_Master, &c));
	uint8_t tx12[4] = {0xAB, 0xFF, 0xCD, 0x0E};
	CHECK(SPITx(&g_Master, 0, tx12, 4) == 4);
	CHECK(g_Master.FirstRdData == 0xFAB);
	CHECK(SPITx(&g_Master, 0, tx12, 3) == 0);	// odd length
	CHECK(SPITx(&g_Master, 0, tx12, 4) == 4);	// still usable after the error
	DeviceIntrfPowerOff(&g_Master.DevIntrf);

	// Manual chip select leaves the pin alone
	c = MasterCfg(0, 1000000, 0, false);
	c.ChipSel = SPICSEL_MAN;
	c.NbIOPins = 3;
	CHECK(SPIInit(&g_Master, &c));
	g_CsEdges[0] = g_CsEdges[1] = 0;
	CHECK(SPITx(&g_Master, 0, tx, 4) == 4);
	CHECK(g_CsEdges[0] == 0 && g_CsEdges[1] == 0);
	DeviceIntrfPowerOff(&g_Master.DevIntrf);

	// Master on unit 1
	static IOPinCfg_t m1[4] = {
		{1, 11, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{1, 10, IOPINOP_FUNC5, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
		{1, 9, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{1, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	};
	c = MasterCfg(1, 1000000, 0, false);
	c.pIOPinMap = m1;
	CHECK(SPIInit(&g_Master, &c));
	CHECK((g_Mstp & BIT(18)) == 0);
	CHECK(SPITx(&g_Master, 0, tx, 4) == 4);
	DeviceIntrfPowerOff(&g_Master.DevIntrf);
}

// Same sequence as the shared example
static bool Frame(int len, uint8_t seed, bool receive, bool masterInt)
{
	uint8_t mtx[129], mrx[129] = {};
	int count;

	for (int i = 0; i < len; i++)
	{
		mtx[i] = (uint8_t)(seed + i);
	}

	memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
	s_MasterDone = false;
	unsigned before = s_Completions;

	if (receive)
	{
		count = SPIRx(&g_Master, 0, mrx, len);
	}
	else
	{
		count = SPITx(&g_Master, 0, mtx, len);
		// The driver must have its own copy of asynchronous TX data
		memset(mtx, 0x5A, sizeof(mtx));
		for (int i = 0; i < len; i++)
		{
			mtx[i] = 0x5A;
		}
	}

	if (count < 0)
	{
		CHECK(masterInt);
		Idle();
		CHECK(s_MasterDone);
		count = s_MasterDone ? s_MasterCount : 0;
	}
	else
	{
		Idle();
	}

	bool pass = count == len && s_Completions == before + 1 && s_SlaveCount == len;

	for (int i = 0; i < len; i++)
	{
		uint8_t exp = receive ? 0xFF : (uint8_t)(seed + i);

		if (s_SlaveRx[i] != exp || (receive && mrx[i] != s_SlaveTx[i]))
		{
			pass = false;
		}
	}

	if (!pass)
	{
		printf("Frame len=%d seed=%02x rx=%d count=%d slave=%d completions %u->%u\n", len, seed,
			receive, count, s_SlaveCount, before, s_Completions);
	}

	return pass;
}

static void PairRun(int mode, bool masterInt, bool idleFirst)
{
	Reset();
	g_IdleFirst = idleFirst;

	SPICfg_t m = MasterCfg(0, 1000000, mode, masterInt);
	SPICfg_t s = SlaveCfg(1, mode);

	CHECK(SPIInit(&g_Master, &m));
	CHECK(SPIInit(&g_Slave, &s));
	CHECK(!g_Master.DevIntrf.bDma && !g_Slave.DevIntrf.bDma);
	CHECK(g_Slave.DevIntrf.bIntEn);
	CHECK(s_Arms == 1);

	const int lens[] = {1, 16, 129};

	for (int l : lens)
	{
		CHECK(Frame(l, 0x30, false, masterInt));
		CHECK(Frame(l, 0x50, false, masterInt));
		CHECK(Frame(l, 0, true, masterInt));
		if (l > 1)
		{
			// A short frame must discard the unused TX prefetch
			CHECK(Frame(1, 0x70, false, masterInt));
			CHECK(Frame(l, 0x90, true, masterInt));
			CHECK(Frame(l, 0x90, false, masterInt));
		}
	}

	if (masterInt)
	{
		CHECK(s_MasterEvents > 0);
	}
	else
	{
		CHECK(s_MasterEvents == 0);
	}

	// The slave is armed again after every frame
	CHECK(s_Arms == 1 + s_Completions);
	CHECK(g_Master.DevIntrf.bTxReady);

	DeviceIntrfPowerOff(&g_Master.DevIntrf);
	DeviceIntrfPowerOff(&g_Slave.DevIntrf);
	CHECK((g_Mstp & (BIT(19) | BIT(18))) == (BIT(19) | BIT(18)));
	for (int i = 0; i < 8; i++)
	{
		CHECK(!g_Irq[i].used);
	}
}

static void TestSlaveErrors(void)
{
	Reset();

	SPICfg_t m = MasterCfg(0, 1000000, 1, false);
	SPICfg_t s = SlaveCfg(1, 1);

	CHECK(SPIInit(&g_Master, &m));
	CHECK(SPIInit(&g_Slave, &s));

	// Frame longer than the slave RX buffer
	s_SlaveRxLen = 16;
	DeviceIntrfReset(&g_Slave.DevIntrf);
	{
		uint8_t tx[32];

		for (int i = 0; i < 32; i++)
		{
			tx[i] = (uint8_t)i;
		}

		unsigned before = s_Completions;

		CHECK(SPITx(&g_Master, 0, tx, 32) == 32);
		Idle();
		CHECK(s_Completions == before + 1);
		CHECK(s_SlaveCount == -1);
		CHECK(s_SlaveRx[15] == 15);
	}

	// The slave rearmed itself and takes the new capacity at once
	s_SlaveRxLen = 129;
	CHECK(Frame(16, 0x10, false, false));

	// Reset of the slave rearms it
	unsigned arms = s_Arms;
	DeviceIntrfReset(&g_Slave.DevIntrf);
	CHECK(s_Arms == arms + 1);
	CHECK(Frame(16, 0x20, false, false));

	// Disable and enable
	DeviceIntrfDisable(&g_Slave.DevIntrf);
	CHECK(g_Slave.DevIntrf.EnCnt == 0);
	DeviceIntrfEnable(&g_Slave.DevIntrf);
	CHECK(Frame(16, 0x30, false, false));

	DeviceIntrfPowerOff(&g_Master.DevIntrf);
	DeviceIntrfPowerOff(&g_Slave.DevIntrf);
}

static int UsedIrqs(void)
{
	int n = 0;

	for (int i = 0; i < 8; i++)
	{
		n += g_Irq[i].used ? 1 : 0;
	}

	return n;
}

// Init again without PowerOff, as the shared example does for every mode
static void TestReinit(void)
{
	Reset();

	for (int mode = 1; mode <= 3; mode += 2)
	{
		SPICfg_t m = MasterCfg(0, 1000000, mode, true);
		SPICfg_t s = SlaveCfg(1, mode);

		CHECK(SPIInit(&g_Master, &m));
		CHECK(SPIInit(&g_Slave, &s));
		CHECK(UsedIrqs() == 5);
		CHECK(Frame(16, 0x10, false, true));
		CHECK(Frame(16, 0, true, true));
	}

	DeviceIntrfPowerOff(&g_Master.DevIntrf);
	DeviceIntrfPowerOff(&g_Slave.DevIntrf);
	CHECK(UsedIrqs() == 0);
}

static void TestMasterAsyncMisc(void)
{
	Reset();

	SPICfg_t m = MasterCfg(0, 1000000, 1, true);
	SPICfg_t s = SlaveCfg(1, 1);

	CHECK(SPIInit(&g_Master, &m));
	CHECK(SPIInit(&g_Slave, &s));

	// One frame is polled, the busy flag is free on return
	uint8_t tx[4] = {1, 2, 3, 4};

	CHECK(SPITx(&g_Master, 0, tx, 1) == 1);
	CHECK(SPITx(&g_Master, 0, tx, 4) == -1);

	// The interface is busy until the completion
	g_Hold = true;
	s_MasterDone = false;
	CHECK(SPITx(&g_Master, 0, tx, 4) == -1);
	CHECK(!s_MasterDone);
	CHECK(!g_Master.DevIntrf.bTxReady);
	CHECK(!SPIStartTx(&g_Master, 0));
	g_Hold = false;
	Idle();
	CHECK(s_MasterDone && s_MasterCount == 4);
	CHECK(g_Master.DevIntrf.bTxReady);
	CHECK(SPIStartTx(&g_Master, 0));
	SPIStopTx(&g_Master);

	// The driver keeps its own copy of the TX data while the transfer runs
	{
		uint8_t t2[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
		uint8_t keep[8];

		memcpy(keep, t2, sizeof(t2));
		memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
		g_Hold = true;
		s_MasterDone = false;
		CHECK(SPITx(&g_Master, 0, t2, 8) == -1);
		memset(t2, 0x5A, sizeof(t2));
		g_Hold = false;
		Idle();
		CHECK(s_MasterDone && s_MasterCount == 8);
		CHECK(memcmp(s_SlaveRx, keep, sizeof(keep)) == 0);
	}

	// Longer than the copy buffer: polled, count returned
	static uint8_t big[300];
	CHECK(SPITx(&g_Master, 0, big, 300) == 300);

	// Disable during an asynchronous transfer releases the busy flag and
	// reports nothing
	g_Hold = true;
	s_MasterDone = false;
	CHECK(SPITx(&g_Master, 0, tx, 4) == -1);
	DeviceIntrfDisable(&g_Master.DevIntrf);
	g_Hold = false;
	Idle();
	CHECK(!s_MasterDone);
	CHECK(g_Pin[1][3] == 1);
	CHECK(!SPIStartTx(&g_Master, 0));	// disabled
	DeviceIntrfEnable(&g_Master.DevIntrf);
	CHECK(SPIStartTx(&g_Master, 0));
	SPIStopTx(&g_Master);
	CHECK(SPITx(&g_Master, 0, tx, 4) == -1);
	Idle();
	CHECK(s_MasterDone && s_MasterCount == 4);

	DeviceIntrfPowerOff(&g_Master.DevIntrf);
	DeviceIntrfPowerOff(&g_Slave.DevIntrf);
}

int main(void)
{
	g_Verbose = getenv("SPI_TEST_VERBOSE") != NULL;
	TestRates();
	TestInitRejects();
	TestMasterLoopback();

	for (int mode = 1; mode <= 3; mode += 2)
	{
		for (int mint = 0; mint < 2; mint++)
		{
			for (int idle = 0; idle < 2; idle++)
			{
				PairRun(mode, mint != 0, idle != 0);
			}
		}
	}

	TestSlaveErrors();
	TestReinit();
	TestMasterAsyncMisc();

	CHECK(g_Violations == 0);

	if (g_Fail == 0)
	{
		printf("PASS: SPI model checks complete (%u frames)\n", g_Frames);

		return 0;
	}

	printf("FAIL: %d checks failed\n", g_Fail);

	return 1;
}
