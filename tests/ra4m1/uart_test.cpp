/**-------------------------------------------------------------------------
@file	uart_test.cpp

@brief	Host test of the RA4M1 UART implementation, uart_ra4m1.cpp.

		The driver source is compiled with a model of the SCI registers,
		module stop, PRCR and the ICU (uart_include). It runs with the real
		CFIFO and DeviceIntrf code, through the public UART functions.

		Line model : one step moves one character each way. TX shifts out the
		character in the shift register and loads the next from TDR or the TX
		FIFO, TDR written with the line idle goes to the shift register right
		away. RX receives the next character of RxIn. RXI (RIE and RDRF, or
		RDF or DR with the FIFO) and ERI (RIE and ORER, FER or PER) are
		levels. TXI is latched when TDRE goes to 1, TDFE when the TX FIFO goes
		down to the trigger level, with TIE set, or when TE and TIE are set by
		the same write, and cleared when it is taken. Interrupts are taken
		when interrupts are enabled again and after each step. An interrupt
		that stays requested after 200 calls fails the test.

		Checked on every access : the SCI module is running, SMR, SEMR, BRR,
		MDDR, SCMR and the FIFO resets are written with TE and RE cleared,
		status flags are cleared only after being read as 1, the RX FIFO is
		not read empty, the TX FIFO and TDR are not written full, MSTPCRB is
		written with PRCR.PRC1 set.

		Not a timing or electrical test, not a test of the ICU edge handling.

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
#include <math.h>
#include <stdio.h>
#include <deque>
#include <vector>

#include "../../ARM/Renesas/RA4M1/src/uart_ra4m1.cpp"

#define CHECK(c)	do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_FailCnt++; } } while (0)

#define B_TDRE		R_SCI0_SSR_TDRE_Msk		// TDFE with the FIFO
#define B_RDRF		R_SCI0_SSR_RDRF_Msk		// RDF with the FIFO
#define B_ORER		R_SCI0_SSR_ORER_Msk
#define B_FER		R_SCI0_SSR_FER_Msk
#define B_PER		R_SCI0_SSR_PER_Msk
#define B_TEND		R_SCI0_SSR_TEND_Msk
#define B_DR		R_SCI0_SSR_FIFO_DR_Msk
#define B_RXERR		(B_ORER | B_FER | B_PER)

#define C_FER		R_SCI0_FRDRHL_FER_Msk	// Error bits of a character in RxIn
#define C_PER		R_SCI0_FRDRHL_PER_Msk

#define TE			R_SCI0_SCR_TE_Msk
#define RE			R_SCI0_SCR_RE_Msk
#define TIE			R_SCI0_SCR_TIE_Msk
#define RIE			R_SCI0_SCR_RIE_Msk

typedef struct {
	uint8_t Smr, Brr, Scr, Scmr, Semr, Snfr, Simr1, Spmr, Mddr, Dccr, Sptr;
	uint16_t Fcr;
	uint8_t Flags;					// SSR
	uint8_t Seen;					// Flags read as 1 since the last SSR write
	std::deque<uint16_t> RxIn;		// Characters still to arrive, with C_FER, C_PER
	std::deque<uint16_t> RxFifo;
	uint8_t Rdr;
	std::deque<uint8_t> TxFifo;
	uint8_t Tdr;
	bool bTdrFull;
	int Tsr;						// Character in the TX shift register, -1 none
	int Idle;						// Steps without RX character
	bool bTxiIr;					// TXI latched in the ICU
	bool bRxdLow;					// RXD held low, break
	std::vector<uint8_t> Wire;		// Characters sent
	unsigned StaleRdr;				// RDR read without RDRF or an error flag
	unsigned TxIgnored;				// TDR or FTDRL written with TE cleared
	unsigned RxLost;				// Characters lost by the receiver
} SciModel_t;

typedef struct {
	int Evt;
	Ra4m1IRQHandler_t Handler;
	void *pCtx;
} IcuSlot_t;

static int s_FailCnt;
static SciModel_t s_Sci[4];
static uint16_t s_Prcr;
static uint32_t s_Mstpcrb = 0xFFFFFFFFU;
static const uint32_t s_MstpBit[4] = { 1UL << 31, 1UL << 30, 1UL << 29, 1UL << 22 };
static const int s_EvtRxi[4] = { RA4M1_EVTID_SCI0_RXI, RA4M1_EVTID_SCI1_RXI, RA4M1_EVTID_SCI2_RXI, RA4M1_EVTID_SCI9_RXI };
static uint32_t s_Pclk = 48000000;
static IcuSlot_t s_Slot[32];
static bool s_bInIsr;
static unsigned s_IrqPoints;
static unsigned s_StepEvery;		// Line step every n interrupt points, 0 none
static unsigned s_IsrCnt;
static int s_PinCfgCnt;
static int s_PinDisCnt;

R_SCI0_Type g_TestSci[4];
R_SYSTEM_Type g_TestSystem;
R_MSTP_Type g_TestMstp;
uint32_t g_TestPrimask;
uint32_t SystemCoreClock = 48000000;

uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx == 0 ? s_Pclk : 0;
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	s_PinCfgCnt++;
}

void IOPinDisable(int PortNo, int PinNo)
{
	s_PinDisCnt++;
}

static bool Fifo(const SciModel_t &m)
{
	return (m.Fcr & R_SCI0_FCR_FM_Msk) != 0;
}

static unsigned RxTrigger(const SciModel_t &m)
{
	return (m.Fcr & R_SCI0_FCR_RTRG_Msk) >> R_SCI0_FCR_RTRG_Pos;
}

static unsigned TxTrigger(const SciModel_t &m)
{
	return (m.Fcr & R_SCI0_FCR_TTRG_Msk) >> R_SCI0_FCR_TTRG_Pos;
}

// TDRE, or TDFE with the FIFO, goes to 1. With TIE set this is a TXI.
static void SetTxFlag(SciModel_t &m)
{
	if ((m.Flags & B_TDRE) == 0)
	{
		m.Flags |= B_TDRE;
		if (m.Scr & TIE)
		{
			m.bTxiIr = true;
		}
	}
}

static int SciUnit(const void *pReg)
{
	int u = (int)(((const char *)pReg - (const char *)g_TestSci) / (long)sizeof(R_SCI0_Type));

	CHECK(u >= 0 && u < 4);
	CHECK((s_Mstpcrb & s_MstpBit[u]) == 0);

	return u;
}

uint32_t TestRegRead(const void *pReg, int Id)
{
	if (Id == SYSREG_PRCR)
	{
		return s_Prcr;
	}
	if (Id == SYSREG_MSTPCRB)
	{
		return s_Mstpcrb;
	}

	SciModel_t &m = s_Sci[SciUnit(pReg)];

	switch (Id)
	{
		case SCIREG_SMR: return m.Smr;
		case SCIREG_BRR: return m.Brr;
		case SCIREG_SCR: return m.Scr;
		case SCIREG_SCMR: return m.Scmr;
		case SCIREG_SEMR: return m.Semr;
		case SCIREG_SNFR: return m.Snfr;
		case SCIREG_SIMR1: return m.Simr1;
		case SCIREG_SPMR: return m.Spmr;
		case SCIREG_MDDR: return m.Mddr;
		case SCIREG_DCCR: return m.Dccr;
		case SCIREG_FCR: return m.Fcr;
		case SCIREG_SPTR: return (m.Sptr & ~R_SCI0_SPTR_RXDMON_Msk) | (m.bRxdLow ? 0 : R_SCI0_SPTR_RXDMON_Msk);
		case SCIREG_SSR:
			m.Seen |= m.Flags;
			return m.Flags;
		case SCIREG_RDR:
			CHECK(!Fifo(m));
			if ((m.Flags & (B_RDRF | B_RXERR)) == 0)
			{
				m.StaleRdr++;
			}
			m.Flags &= ~B_RDRF;
			return m.Rdr;
		case SCIREG_FRDRHL:
		{
			CHECK(Fifo(m) && !m.RxFifo.empty());
			if (m.RxFifo.empty())
			{
				return 0;
			}
			uint16_t d = m.RxFifo.front();
			m.RxFifo.pop_front();
			return d;
		}
		case SCIREG_FDR:
			return (uint16_t)((m.TxFifo.size() << R_SCI0_FDR_T_Pos) | m.RxFifo.size());
		default:
			CHECK(false);
			return 0;
	}
}

void TestRegWrite(const void *pReg, int Id, uint32_t Val)
{
	if (Id == SYSREG_PRCR)
	{
		CHECK((Val & 0xFF00U) == 0xA500U);
		s_Prcr = Val & 0xFU;
		return;
	}
	if (Id == SYSREG_MSTPCRB)
	{
		CHECK(s_Prcr & R_SYSTEM_PRCR_PRC1_Msk);
		s_Mstpcrb = Val;
		return;
	}

	SciModel_t &m = s_Sci[SciUnit(pReg)];
	bool off = (m.Scr & (TE | RE)) == 0;
	uint8_t v = (uint8_t)Val;

	switch (Id)
	{
		case SCIREG_SMR: CHECK(off); m.Smr = v; break;
		case SCIREG_BRR: CHECK(off); m.Brr = v; break;
		case SCIREG_SCMR: CHECK(off); m.Scmr = v; break;
		case SCIREG_SEMR: CHECK(off); m.Semr = v; break;
		case SCIREG_MDDR: CHECK(off && v >= 128); m.Mddr = v; break;
		case SCIREG_SNFR: m.Snfr = v; break;
		case SCIREG_SIMR1: m.Simr1 = v; break;
		case SCIREG_SPMR: m.Spmr = v; break;
		case SCIREG_DCCR: m.Dccr = v; break;
		case SCIREG_SPTR: m.Sptr = v; break;
		case SCIREG_FCR:
			CHECK(off);
			if (Val & R_SCI0_FCR_RFRST_Msk)
			{
				m.RxFifo.clear();
			}
			if (Val & R_SCI0_FCR_TFRST_Msk)
			{
				m.TxFifo.clear();
			}
			m.Fcr = (uint16_t)(Val & ~(R_SCI0_FCR_RFRST_Msk | R_SCI0_FCR_TFRST_Msk));
			break;
		case SCIREG_SCR:
		{
			uint8_t old = m.Scr;
			m.Scr = v;
			if ((v & TE) && !(old & TE))
			{
				if (!Fifo(m) || m.TxFifo.size() <= TxTrigger(m))
				{
					m.Flags |= B_TDRE;
				}
				m.Flags |= B_TEND;
				if (v & TIE)
				{
					// TE and TIE set by the same write
					m.bTxiIr = true;
				}
			}
			if (!(v & TE))
			{
				// TX stopped, TDR and the shift register are dropped
				m.bTdrFull = false;
				m.Tsr = -1;
				if (!Fifo(m))
				{
					m.Flags |= B_TDRE;
				}
			}
			break;
		}
		case SCIREG_SSR:
		{
			// Written 0 to a flag that is 1 : cleared, only after it was read
			uint8_t clr = (uint8_t)(~v & m.Flags);
			uint8_t clearable = Fifo(m) ? (B_TDRE | B_RDRF | B_RXERR | B_DR) : B_RXERR;

			CHECK((clr & ~m.Seen) == 0);
			if (!Fifo(m))
			{
				// RDRF and TDRE are left to the hardware by the driver
				CHECK((~v & (B_TDRE | B_RDRF)) == 0);
			}
			m.Flags &= ~(clr & clearable);
			m.Seen = 0;
			break;
		}
		case SCIREG_TDR:
			CHECK(!Fifo(m));
			if (!(m.Scr & TE))
			{
				m.TxIgnored++;
				break;
			}
			CHECK(m.Flags & B_TDRE);
			m.Flags &= ~(B_TDRE | B_TEND);
			if (m.Tsr < 0)
			{
				// Line idle, TDR goes to the shift register right away
				m.Tsr = v;
				SetTxFlag(m);
			}
			else
			{
				m.Tdr = v;
				m.bTdrFull = true;
			}
			break;
		case SCIREG_FTDRL:
			CHECK(Fifo(m));
			if (!(m.Scr & TE))
			{
				m.TxIgnored++;
				break;
			}
			CHECK(m.TxFifo.size() < 16);
			m.TxFifo.push_back(v);
			m.Flags &= ~B_TEND;
			break;
		default:
			CHECK(false);
			break;
	}
}

// A character on RXD
static void Arrive(SciModel_t &m, uint16_t c)
{
	uint16_t err = c & (C_FER | C_PER);

	if (Fifo(m))
	{
		if (m.RxFifo.size() >= 16)
		{
			m.Flags |= B_ORER;
			m.RxLost++;
			return;
		}
		m.RxFifo.push_back(c);
		if (err & C_FER)
		{
			m.Flags |= B_FER;
		}
		if (err & C_PER)
		{
			m.Flags |= B_PER;
		}
		if (m.RxFifo.size() >= RxTrigger(m))
		{
			m.Flags |= B_RDRF;
		}
		return;
	}

	if (m.Flags & B_RXERR)
	{
		// Reception stopped until the error flags are cleared
		m.RxLost++;
		return;
	}
	if (m.Flags & B_RDRF)
	{
		m.Flags |= B_ORER;
		m.RxLost++;
		return;
	}
	m.Rdr = (uint8_t)c;
	if (err)
	{
		m.Flags |= (err & C_FER) ? B_FER : 0;
		m.Flags |= (err & C_PER) ? B_PER : 0;
	}
	else
	{
		m.Flags |= B_RDRF;
	}
}

static void LineStep(int u)
{
	SciModel_t &m = s_Sci[u];

	if (s_Mstpcrb & s_MstpBit[u])
	{
		return;
	}

	if (m.Tsr >= 0)
	{
		m.Wire.push_back((uint8_t)m.Tsr);
		m.Tsr = -1;
	}
	if (m.Scr & TE)
	{
		if (Fifo(m))
		{
			if (!m.TxFifo.empty())
			{
				// TDFE set when the FIFO goes down to the trigger level only
				bool cross = m.TxFifo.size() == TxTrigger(m) + 1;
				m.Tsr = m.TxFifo.front();
				m.TxFifo.pop_front();
				if (cross)
				{
					SetTxFlag(m);
				}
			}
		}
		else if (m.bTdrFull)
		{
			m.Tsr = m.Tdr;
			m.bTdrFull = false;
			SetTxFlag(m);
		}
		if (m.Tsr < 0)
		{
			m.Flags |= B_TEND;
		}
	}

	if ((m.Scr & RE) && !m.RxIn.empty())
	{
		Arrive(m, m.RxIn.front());
		m.RxIn.pop_front();
		m.Idle = 0;
	}
	else if (Fifo(m) && (m.Scr & RE) && !m.RxFifo.empty() && m.RxFifo.size() < RxTrigger(m))
	{
		if (++m.Idle >= 2)
		{
			m.Flags |= B_DR;
		}
	}
}

static bool Requested(int Slot)
{
	int u = -1;
	int kind = 0;

	for (int i = 0; i < 4; i++)
	{
		int k = s_Slot[Slot].Evt - s_EvtRxi[i];
		if (k == 0 || k == 1 || k == 3)
		{
			u = i;
			kind = k;
		}
	}
	if (u < 0 || (s_Mstpcrb & s_MstpBit[u]))
	{
		return false;
	}

	SciModel_t &m = s_Sci[u];

	switch (kind)
	{
		case 0: return (m.Scr & RIE) && (m.Flags & (Fifo(m) ? (B_RDRF | B_DR) : B_RDRF));
		case 1: return m.bTxiIr;
		default: return (m.Scr & RIE) && (m.Flags & B_RXERR);
	}
}

static void Dispatch(void)
{
	if (s_bInIsr)
	{
		return;
	}
	for (int n = 0; ; n++)
	{
		int slot = -1;
		for (int i = 0; i < 32 && slot < 0; i++)
		{
			if (s_Slot[i].Handler && Requested(i))
			{
				slot = i;
			}
		}
		if (slot < 0)
		{
			return;
		}
		if (n >= 200)
		{
			CHECK(n < 200);
			return;
		}
		for (int i = 0; i < 4; i++)
		{
			if (s_Slot[slot].Evt == s_EvtRxi[i] + 1)
			{
				// TXI taken, its ICU flag is cleared
				s_Sci[i].bTxiIr = false;
			}
		}
		uint32_t pm = g_TestPrimask;
		s_bInIsr = true;
		g_TestPrimask = 1;
		s_Slot[slot].Handler(slot, s_Slot[slot].pCtx);
		g_TestPrimask = pm;
		s_bInIsr = false;
		s_IsrCnt++;
	}
}

static void RunLine(int Steps)
{
	for (int i = 0; i < Steps; i++)
	{
		for (int u = 0; u < 4; u++)
		{
			LineStep(u);
		}
		Dispatch();
	}
}

void TestIrqPoint(void)
{
	if (s_bInIsr)
	{
		return;
	}
	s_IrqPoints++;
	if (s_StepEvery && (s_IrqPoints % s_StepEvery) == 0)
	{
		for (int u = 0; u < 4; u++)
		{
			LineStep(u);
		}
	}
	Dispatch();
}

IRQn_Type Ra4m1RegisterIntHandler(uint8_t EvtId, int Prio, Ra4m1IRQHandler_t pHandler, void *pCtx)
{
	for (int i = 0; i < 32; i++)
	{
		if (s_Slot[i].Handler && s_Slot[i].Evt == EvtId)
		{
			// Same as the ICU driver : an event has one slot
			return (IRQn_Type)-1;
		}
	}
	for (int i = 0; i < 32; i++)
	{
		if (s_Slot[i].Handler == NULL)
		{
			s_Slot[i].Evt = EvtId;
			s_Slot[i].Handler = pHandler;
			s_Slot[i].pCtx = pCtx;
			return (IRQn_Type)i;
		}
	}
	return (IRQn_Type)-1;
}

void Ra4m1UnregisterIntHandler(IRQn_Type IrqNo)
{
	if ((unsigned)IrqNo < 32)
	{
		s_Slot[IrqNo].Handler = NULL;
		s_Slot[IrqNo].Evt = 0;
	}
}

bool Ra4m1AcknowledgeInt(IRQn_Type IrqNo)
{
	return (unsigned)IrqNo < 32 && s_Slot[IrqNo].Handler != NULL;
}

static int SlotCnt(void)
{
	int n = 0;
	for (int i = 0; i < 32; i++)
	{
		n += s_Slot[i].Handler != NULL;
	}
	return n;
}

// Test setup

static const IOPinCfg_t s_Pins[2] = {
	{ 1, 0, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 1, 1, IOPINOP_FUNC3, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
};

alignas(4) static uint8_t s_RxMem[CFIFO_MEMSIZE(256)];
alignas(4) static uint8_t s_TxMem[CFIFO_MEMSIZE(256)];

static UARTDev_t s_Uart;
static unsigned s_EvtRx;
static unsigned s_EvtTx;
static unsigned s_EvtLine;
static uint8_t s_LastLine;
static int s_RefillLeft;
static const uint8_t *s_pRefill;

static int EvtHandler(UARTDev_t * const pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	switch (EvtId)
	{
		case UART_EVT_RXDATA:
			s_EvtRx++;
			CHECK(BufferLen == CFifoUsed(pDev->hRxFifo));
			break;
		case UART_EVT_TXREADY:
			s_EvtTx++;
			CHECK(BufferLen == CFifoAvail(pDev->hTxFifo));
			if (s_RefillLeft > 0)
			{
				// More data from the interrupt, as a stream source would
				int n = UARTTx(pDev, (uint8_t *)s_pRefill, s_RefillLeft);
				s_pRefill += n;
				s_RefillLeft -= n;
			}
			break;
		case UART_EVT_LINESTATE:
			s_EvtLine++;
			CHECK(BufferLen == 1 && pBuffer != NULL);
			s_LastLine = pBuffer ? *pBuffer : 0;
			break;
		default:
			break;
	}
	return 0;
}

static void ResetModel(void)
{
	for (int u = 0; u < 4; u++)
	{
		if (s_Uart.DevIntrf.pDevData && ((Ra4m1UartDev_t *)s_Uart.DevIntrf.pDevData)->DevNo == u &&
			(s_Mstpcrb & s_MstpBit[u]) == 0)
		{
			UARTPowerOff(&s_Uart);
		}
	}
	for (int u = 0; u < 4; u++)
	{
		s_Sci[u] = SciModel_t();
		s_Sci[u].Tsr = -1;
		s_Sci[u].Scmr = 0xF2;
		s_Sci[u].Mddr = 0xFF;
		s_Sci[u].Dccr = 0x40;
		s_Sci[u].Flags = B_TDRE | B_TEND;
	}
	for (int i = 0; i < 4; i++)
	{
		s_Ra4m1UartDev[i].pUartDev = NULL;
		Ra4m1UartReleaseIRQs(&s_Ra4m1UartDev[i]);
	}
	for (int i = 0; i < 32; i++)
	{
		s_Slot[i] = IcuSlot_t();
	}
	s_Mstpcrb = 0xFFFFFFFFU;
	s_Prcr = 0;
	s_StepEvery = 0;
	s_EvtRx = s_EvtTx = s_EvtLine = 0;
	s_LastLine = 0;
	s_RefillLeft = 0;
	memset((void *)&s_Uart, 0, sizeof(s_Uart));
}

static UARTCfg_t Cfg(int DevNo, bool bIntMode)
{
	UARTCfg_t cfg = {};

	cfg.DevNo = DevNo;
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 1000000;
	cfg.DataBits = 8;
	cfg.Parity = UART_PARITY_NONE;
	cfg.StopBits = 1;
	cfg.FlowControl = UART_FLWCTRL_NONE;
	cfg.bIntMode = bIntMode;
	cfg.IntPrio = 1;
	cfg.EvtCallback = NULL;
	cfg.bFifoBlocking = true;
	cfg.RxMemSize = sizeof(s_RxMem);
	cfg.pRxMem = s_RxMem;
	cfg.TxMemSize = sizeof(s_TxMem);
	cfg.pTxMem = s_TxMem;
	cfg.bDMAMode = false;
	cfg.Duplex = UART_DUPLEX_FULL;
	cfg.Mode = UART_MODE_UART;

	return cfg;
}

static void Pattern(std::vector<uint8_t> &d, int n, uint8_t seed)
{
	d.resize(n);
	uint8_t x = seed;
	for (int i = 0; i < n; i++)
	{
		x = (uint8_t)(x * 5 + 17);
		d[i] = x;
	}
}

// All of the data out with UARTTx, len bytes per call
static void SendAll(const uint8_t *p, int Len, int Chunk)
{
	int sent = 0;
	int guard = 0;

	while (sent < Len)
	{
		int l = Len - sent < Chunk ? Len - sent : Chunk;
		int n = UARTTx(&s_Uart, (uint8_t *)p + sent, l);
		sent += n;
		if (n == 0)
		{
			RunLine(1);
		}
		if (++guard > 1000000)
		{
			CHECK(false);
			return;
		}
	}
}

static void Drain(int u, size_t Len)
{
	for (int i = 0; i < 100000 && s_Sci[u].Wire.size() < Len; i++)
	{
		RunLine(1);
		if (!s_Uart.DevIntrf.bIntEn)
		{
			// Without the interrupt TX moves only in the driver calls, as an
			// application calling RxData would make it move
			uint32_t state = DisableInterrupt();
			Ra4m1UartPoll(&s_Ra4m1UartDev[u]);
			EnableInterrupt(state);
		}
	}
}

// Tests

static void TestInit(void)
{
	UARTCfg_t cfg;

	ResetModel();
	cfg = Cfg(-1, true); CHECK(UARTInit(&s_Uart, &cfg) == false);
	cfg = Cfg(4, true); CHECK(UARTInit(&s_Uart, &cfg) == false);
	cfg = Cfg(0, true); cfg.pIOPinMap = NULL; CHECK(UARTInit(&s_Uart, &cfg) == false);
	cfg = Cfg(0, true); cfg.NbIOPins = 1; CHECK(UARTInit(&s_Uart, &cfg) == false);
	cfg = Cfg(0, true); cfg.DataBits = 9; CHECK(UARTInit(&s_Uart, &cfg) == false);
	cfg = Cfg(0, true); cfg.Parity = UART_PARITY_MARK; CHECK(UARTInit(&s_Uart, &cfg) == false);
	cfg = Cfg(0, true); cfg.FlowControl = UART_FLWCTRL_HW; CHECK(UARTInit(&s_Uart, &cfg) == false);
	cfg = Cfg(0, true); cfg.Rate = 0; CHECK(UARTInit(&s_Uart, &cfg) == false);
	cfg = Cfg(0, true); cfg.StopBits = 3; CHECK(UARTInit(&s_Uart, &cfg) == false);
	CHECK(SlotCnt() == 0);

	for (int u = 0; u < 4; u++)
	{
		for (int intmode = 0; intmode < 2; intmode++)
		{
			ResetModel();
			cfg = Cfg(u, intmode);
			cfg.bDMAMode = true;
			cfg.StopBits = 2;
			cfg.Parity = UART_PARITY_ODD;
			s_PinCfgCnt = 0;
			CHECK(UARTInit(&s_Uart, &cfg) == true);
			SciModel_t &m = s_Sci[u];
			CHECK((s_Mstpcrb & s_MstpBit[u]) == 0);
			CHECK(m.Scr == (TE | RE | (intmode ? RIE : 0)));
			CHECK(Fifo(m) == (u < 2));
			CHECK((m.Smr & (R_SCI0_SMR_STOP_Msk | R_SCI0_SMR_PE_Msk | R_SCI0_SMR_PM_Msk | R_SCI0_SMR_CHR_Msk)) ==
				  (R_SCI0_SMR_STOP_Msk | R_SCI0_SMR_PE_Msk | R_SCI0_SMR_PM_Msk));
			CHECK(m.Scmr == 0xF2);
			CHECK(s_Uart.Rate == 1000000);
			CHECK(s_Uart.DevIntrf.bDma == false);
			CHECK(s_Uart.DevIntrf.bIntEn == (intmode != 0));
			CHECK(s_Uart.DevIntrf.EnCnt == 1);
			CHECK(s_Uart.bTxReady == true);
			CHECK(SlotCnt() == (intmode ? 3 : 0));
			CHECK(s_PinCfgCnt == 2);
			CHECK(UARTGetInstance(u) == &s_Uart);
			CHECK(UARTGetDevHandle(&s_Uart.DevIntrf) == &s_Uart);
		}
	}

	// A second UARTInit releases the interrupts of the first
	ResetModel();
	cfg = Cfg(0, true);
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	CHECK(SlotCnt() == 3);

	// A running instance is not taken by another UART object
	UARTDev_t other = {};
	CHECK(UARTInit(&other, &cfg) == false);
	CHECK(s_Ra4m1UartDev[0].pUartDev == &s_Uart);

	// Interrupt registration failure
	ResetModel();
	for (int i = 0; i < 31; i++)
	{
		s_Slot[i].Handler = (Ra4m1IRQHandler_t)1;
		s_Slot[i].Evt = 1;
	}
	cfg = Cfg(1, true);
	CHECK(UARTInit(&s_Uart, &cfg) == false);
	CHECK(s_Slot[31].Handler == NULL);
	CHECK((s_Mstpcrb & s_MstpBit[1]) != 0);
	for (int i = 0; i < 31; i++)
	{
		s_Slot[i] = IcuSlot_t();
	}
}

// Rate from the registers, as the SCI makes it
static double ModelRate(const SciModel_t &m)
{
	double div = (m.Semr & R_SCI0_SEMR_ABCSE_Msk) ? 6 : (m.Semr & R_SCI0_SEMR_BGDM_Msk) ?
				 ((m.Semr & R_SCI0_SEMR_ABCS_Msk) ? 8 : 16) : 32;
	double mm = (m.Semr & R_SCI0_SEMR_BRME_Msk) ? m.Mddr : 256;

	return (double)s_Pclk * mm / (256.0 * div * (double)(1 << (2 * (m.Smr & 3))) * (m.Brr + 1));
}

// Closest rate the SCI can make, all settings tried
static double BestRate(double Rate)
{
	static const double divs[4] = { 32, 16, 8, 6 };
	double best = 0;

	for (int i = 0; i < 4; i++)
	{
		for (int n = 0; n < 4; n++)
		{
			for (int b = 1; b <= 256; b++)
			{
				double r0 = (double)s_Pclk / (divs[i] * (1 << (2 * n)) * b);
				for (int mm = 128; mm <= 256; mm++)
				{
					double r = r0 * mm / 256;
					if (fabs(r - Rate) < fabs(best - Rate))
					{
						best = r;
					}
				}
			}
		}
	}
	return best;
}

static void TestSetRate(void)
{
	static const uint32_t rates[] = { 50, 300, 9600, 19200, 57600, 115200, 230400, 460800, 921600,
									  1000000, 2000000, 3000000, 6000000, 10000000, 100000000 };
	UARTCfg_t cfg;

	for (int p = 0; p < 2; p++)
	{
		s_Pclk = p ? 24000000 : 48000000;
		ResetModel();
		cfg = Cfg(2, true);
		CHECK(UARTInit(&s_Uart, &cfg) == true);
		for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
		{
			uint32_t r = UARTSetRate(&s_Uart, rates[i]);
			double model = ModelRate(s_Sci[2]);
			double best = BestRate(rates[i]);
			CHECK(fabs((double)r - model) <= 1.0);
			CHECK(s_Uart.Rate == (int)r);
			// Within 1 Hz of the closest any setting gives, integer rounding
			CHECK(fabs(model - rates[i]) <= fabs(best - rates[i]) + 1.0);
			if (fabs(model - rates[i]) > fabs(best - rates[i]) + 1.0)
			{
				printf("  SetRate pclk %u rate %u : %u, closest %.2f\n", s_Pclk, rates[i], r, best);
			}
			CHECK(s_Sci[2].Scr == (TE | RE | RIE));
		}
		CHECK(UARTSetRate(&s_Uart, 1000000) == 1000000);
		uint8_t brr = s_Sci[2].Brr;
		CHECK(UARTSetRate(&s_Uart, 0) == 0);
		CHECK(s_Sci[2].Brr == brr);
	}
	s_Pclk = 48000000;
}

// TX of Len bytes, Chunk bytes per UARTTx call
static void TestTx(int u, bool bIntMode, int Len, int Chunk)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, bIntMode);
	CHECK(UARTInit(&s_Uart, &cfg) == true);

	std::vector<uint8_t> d;
	Pattern(d, Len, (uint8_t)(u + Chunk));
	s_StepEvery = 2;
	SendAll(d.data(), Len, Chunk);
	Drain(u, Len);
	RunLine(4);

	CHECK(s_Sci[u].Wire == d);
	CHECK(s_Sci[u].TxIgnored == 0);
	CHECK(s_Uart.bTxReady == true);
	CHECK((s_Sci[u].Scr & TIE) == 0);
	CHECK(CFifoUsed(s_Uart.hTxFifo) == 0);
}

// RX of Len bytes, read Chunk bytes per UARTRx call
static void TestRx(int u, bool bIntMode, int Len, int Chunk)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, bIntMode);
	CHECK(UARTInit(&s_Uart, &cfg) == true);

	std::vector<uint8_t> d, r;
	Pattern(d, Len, (uint8_t)(0x40 + u));
	for (int i = 0; i < Len; i++)
	{
		s_Sci[u].RxIn.push_back(d[i]);
	}
	// A SCI without FIFO holds one character. Polling it, the line moves only
	// between the driver calls, as an application reading in a loop.
	s_StepEvery = bIntMode || u < 2 ? 3 : 0;
	for (int i = 0; i < 100000 && (int)r.size() < Len; i++)
	{
		// The application reads what has arrived, Chunk bytes per call
		uint8_t buf[64];
		int n;
		do {
			n = UARTRx(&s_Uart, buf, Chunk);
			r.insert(r.end(), buf, buf + n);
		} while (n > 0 && (int)r.size() < Len);
		RunLine(1);
	}
	CHECK(r == d);
	CHECK(s_Sci[u].RxLost == 0);
	CHECK(s_Sci[u].StaleRdr == 0);
	CHECK(s_Uart.RxOvrErrCnt == 0 && s_Uart.RxDropCnt == 0);
	CHECK(s_Uart.bRxReady == false);
	if (r != d)
	{
		printf("  TestRx unit %d int %d chunk %d : %zu of %d, lost %u, ovr %u, drop %u\n", u, bIntMode, Chunk,
			   r.size(), Len, s_Sci[u].RxLost, s_Uart.RxOvrErrCnt, s_Uart.RxDropCnt);
	}
}

// Echo of Len bytes as exemples/uart/uart_loopback.cpp does it
static void TestLoopback(int u, bool bIntMode, int Len)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, bIntMode);
	CHECK(UARTInit(&s_Uart, &cfg) == true);

	std::vector<uint8_t> d;
	Pattern(d, Len, (uint8_t)(0x80 + u));
	for (int i = 0; i < Len; i++)
	{
		s_Sci[u].RxIn.push_back(d[i]);
	}
	s_StepEvery = bIntMode || u < 2 ? 5 : 0;
	for (int i = 0; i < 200000 && s_Sci[u].Wire.size() < (size_t)Len; i++)
	{
		uint8_t buf[16];
		int n = UARTRx(&s_Uart, buf, sizeof(buf));
		if (n > 0)
		{
			SendAll(buf, n, n);
		}
		RunLine(1);
	}
	CHECK(s_Sci[u].Wire == d);
	CHECK(s_Sci[u].RxLost == 0);
	CHECK(s_Sci[u].StaleRdr == 0);
	CHECK(s_Uart.RxOvrErrCnt == 0 && s_Uart.RxDropCnt == 0);
}

static void TestErrors(void)
{
	UARTCfg_t cfg;
	uint8_t buf[64];

	// FIFO, polling : framing and parity errors of single entries
	ResetModel();
	cfg = Cfg(0, false);
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	s_Sci[0].RxIn = { 'A', (uint16_t)('B' | C_FER), (uint16_t)('C' | C_PER), 'D' };
	RunLine(6);
	int n = UARTRx(&s_Uart, buf, sizeof(buf));
	CHECK(n == 2 && buf[0] == 'A' && buf[1] == 'D');
	CHECK(s_Uart.FramErrCnt == 1 && s_Uart.ParErrCnt == 1 && s_Uart.RxDropCnt == 2);
	CHECK((s_Uart.LineState & (UART_LINESTATE_FRMERR | UART_LINESTATE_PARERR)) ==
		  (UART_LINESTATE_FRMERR | UART_LINESTATE_PARERR));
	CHECK((s_Sci[0].Flags & B_RXERR) == 0);

	// FIFO, polling : overrun keeps the 16 characters in the FIFO
	for (int i = 0; i < 20; i++)
	{
		s_Sci[0].RxIn.push_back((uint16_t)i);
	}
	RunLine(22);
	n = UARTRx(&s_Uart, buf, sizeof(buf));
	CHECK(n == 16 && buf[0] == 0 && buf[15] == 15);
	CHECK(s_Uart.RxOvrErrCnt == 1);
	CHECK((s_Sci[0].Flags & B_RXERR) == 0);

	// No FIFO, polling : overrun keeps the character in RDR, framing error drops
	ResetModel();
	cfg = Cfg(2, false);
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	s_Sci[2].RxIn = { 'A', 'B' };
	RunLine(2);
	n = UARTRx(&s_Uart, buf, sizeof(buf));
	CHECK(n == 1 && buf[0] == 'A');
	CHECK(s_Uart.RxOvrErrCnt == 1);
	s_Sci[2].RxIn = { (uint16_t)('C' | C_FER), 'E' };
	RunLine(1);
	n = UARTRx(&s_Uart, buf, sizeof(buf));
	RunLine(1);
	n += UARTRx(&s_Uart, buf + n, sizeof(buf) - n);
	CHECK(n == 1 && buf[0] == 'E');
	CHECK(s_Uart.FramErrCnt == 1);
	CHECK(s_Sci[2].StaleRdr == 0);

	// No FIFO, interrupt : ERI, line state callback, break
	ResetModel();
	cfg = Cfg(3, true);
	cfg.EvtCallback = EvtHandler;
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	s_Sci[3].bRxdLow = true;
	s_Sci[3].RxIn = { (uint16_t)(0 | C_FER) };
	RunLine(1);
	s_Sci[3].bRxdLow = false;
	s_Sci[3].RxIn = { 'Z' };
	RunLine(2);
	n = UARTRx(&s_Uart, buf, sizeof(buf));
	CHECK(n == 1 && buf[0] == 'Z');
	CHECK(s_EvtLine == 1);
	CHECK(s_LastLine == (UART_LINESTATE_FRMERR | UART_LINESTATE_BRK));
	CHECK(s_Uart.FramErrCnt == 1);
	CHECK(s_Sci[3].StaleRdr == 0);

	// FIFO, interrupt : overrun reported through ERI
	ResetModel();
	cfg = Cfg(1, true);
	cfg.EvtCallback = EvtHandler;
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	s_Sci[1].Scr &= ~RIE;			// Interrupt held off, the FIFO fills
	for (int i = 0; i < 18; i++)
	{
		s_Sci[1].RxIn.push_back((uint16_t)(0x30 + i));
	}
	for (int i = 0; i < 18; i++)
	{
		LineStep(1);
	}
	s_Sci[1].Scr |= RIE;
	Dispatch();
	n = UARTRx(&s_Uart, buf, sizeof(buf));
	CHECK(n == 16 && buf[0] == 0x30 && buf[15] == 0x3F);
	CHECK(s_Uart.RxOvrErrCnt == 1 && s_EvtLine == 1 && (s_LastLine & UART_LINESTATE_OVR));
}

// RX CFIFO full, blocking and not blocking
static void TestRxFull(void)
{
	alignas(4) static uint8_t small[CFIFO_MEMSIZE(8)];

	for (int blocking = 0; blocking < 2; blocking++)
	{
		for (int u = 0; u < 4; u += 2)
		{
			ResetModel();
			UARTCfg_t cfg = Cfg(u, true);
			cfg.pRxMem = small;
			cfg.RxMemSize = sizeof(small);
			cfg.bFifoBlocking = blocking;
			CHECK(UARTInit(&s_Uart, &cfg) == true);
			for (int i = 0; i < 20; i++)
			{
				s_Sci[u].RxIn.push_back((uint16_t)i);
			}
			RunLine(24);
			uint8_t buf[32];
			int n = UARTRx(&s_Uart, buf, sizeof(buf));
			CHECK(n == 8);
			CHECK(buf[0] == (blocking ? 0 : 12) && buf[7] == (blocking ? 7 : 19));
			CHECK(s_Uart.RxDropCnt == (blocking ? 12U : 0U));
			CHECK(s_Sci[u].RxLost == 0);
		}
	}
}

static void TestSevenBit(void)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(0, true);
	cfg.DataBits = 7;
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	CHECK(s_Sci[0].Smr & R_SCI0_SMR_CHR_Msk);
	s_Sci[0].RxIn = { 0xC1, 0x42 };
	RunLine(6);
	uint8_t buf[4];
	int n = UARTRx(&s_Uart, buf, sizeof(buf));
	CHECK(n == 2 && buf[0] == 0x41 && buf[1] == 0x42);
}

static void TestCallbacks(void)
{
	for (int u = 0; u < 4; u += 2)
	{
		ResetModel();
		UARTCfg_t cfg = Cfg(u, true);
		cfg.EvtCallback = EvtHandler;
		CHECK(UARTInit(&s_Uart, &cfg) == true);

		// RX data events
		s_Sci[u].RxIn = { 1, 2, 3 };
		RunLine(6);
		CHECK(s_EvtRx > 0);
		uint8_t buf[8];
		CHECK(UARTRx(&s_Uart, buf, sizeof(buf)) == 3);

		// TX refilled from the TXREADY event
		std::vector<uint8_t> d;
		Pattern(d, 3000, 7);
		int n = UARTTx(&s_Uart, d.data(), 100);
		s_pRefill = d.data() + n;
		s_RefillLeft = 3000 - n;
		for (int i = 0; i < 20000 && s_Sci[u].Wire.size() < d.size(); i++)
		{
			RunLine(1);
		}
		CHECK(s_RefillLeft == 0);
		CHECK(s_Sci[u].Wire == d);
		CHECK(s_EvtTx > 0);
		CHECK(s_Uart.bTxReady == true);
	}
}

static void TestEnableDisable(void)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(0, true);
	CHECK(UARTInit(&s_Uart, &cfg) == true);

	s_Sci[0].RxIn = { 'a', 'b' };
	RunLine(6);
	UARTDisable(&s_Uart);
	CHECK(s_Sci[0].Scr == 0);
	CHECK(s_Uart.DevIntrf.EnCnt == 0);
	UARTEnable(&s_Uart);
	CHECK(s_Sci[0].Scr == (TE | RE | RIE));
	uint8_t buf[4];
	CHECK(UARTRx(&s_Uart, buf, sizeof(buf)) == 2 && buf[0] == 'a');

	std::vector<uint8_t> d;
	Pattern(d, 100, 3);
	SendAll(d.data(), 100, 10);
	Drain(0, 100);
	CHECK(s_Sci[0].Wire == d);

	// PowerOff, then a new UARTInit
	s_PinDisCnt = 0;
	UARTPowerOff(&s_Uart);
	CHECK((s_Mstpcrb & s_MstpBit[0]) != 0);
	CHECK(SlotCnt() == 0);
	CHECK(s_PinDisCnt == 2);
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	CHECK(SlotCnt() == 3);
	s_Sci[0].Wire.clear();
	SendAll(d.data(), 100, 7);
	Drain(0, 100);
	CHECK(s_Sci[0].Wire == d);
}

int main()
{
	TestInit();
	TestSetRate();
	for (int u = 0; u < 4; u++)
	{
		for (int intmode = 0; intmode < 2; intmode++)
		{
			TestTx(u, intmode, 1000, 1);
			TestTx(u, intmode, 1000, 16);
			TestTx(u, intmode, 1000, 300);
			TestRx(u, intmode, 1000, 1);
			TestRx(u, intmode, 1000, 16);
			TestRx(u, intmode, 1000, 64);
			TestLoopback(u, intmode, 2000);
		}
	}
	TestErrors();
	TestRxFull();
	TestSevenBit();
	TestCallbacks();
	TestEnableDisable();
	ResetModel();

	if (s_FailCnt)
	{
		printf("uart_test: %d failures\n", s_FailCnt);
		return 1;
	}
	printf("uart_test: PASS (%u interrupts)\n", s_IsrCnt);

	return 0;
}
