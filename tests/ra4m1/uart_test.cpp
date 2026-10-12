/**-------------------------------------------------------------------------
@file	uart_test.cpp

@brief	Host test of the RA4M1 UART implementation, uart_ra4m1.cpp.

		The driver source is compiled with a model of the SCI registers,
		module stop, PRCR and the ICU (uart_include). It runs with the real
		CFIFO and DeviceIntrf code, through the public UART functions.

		Line model : one step moves one character each way. TX shifts out the
		character in the shift register and loads the next from TDR or the TX
		FIFO, TDR written with the line idle goes to the shift register right
		away. RX receives the next character of RxIn, C_IDLE in RxIn is one
		step without a character.

		ICU model : each slot has its IR flag. RXI (RIE and RDRF, or RDF or DR
		with the FIFO) and ERI (RIE and ORER, FER or PER) set it when their
		request goes from 0 to 1. TXI sets it when TDRE goes to 1, TDFE when
		the TX FIFO goes down to the trigger level, with TIE set, or when TE
		and TIE are set by the same write. An event with IR already set is
		lost. Ra4m1AcknowledgeInt clears IR, otherwise it is cleared when the
		handler returns, as the ICU dispatcher of interrupt_ra4m1.cpp does.
		The handler is called while IR is set, when interrupts are enabled
		again and after each step. With s_IsrStepEvery the line also moves
		inside the handler, every n register accesses.

		FIFO flags : RDF is cleared only below the RX trigger, DR only with the
		RX FIFO empty, TDFE only above the TX trigger, FER and PER only with no
		such character left in the FIFO. Reception stops while ORER is set.

		Checked on every access : the SCI module is running, SMR, SEMR, BRR,
		MDDR, SCMR and the FIFO resets are written with TE and RE cleared,
		status flags are cleared only after being read as 1, the RX FIFO is
		not read empty, the TX FIFO and TDR are not written full, MSTPCRB is
		written with PRCR.PRC1 set.

		Not a timing or electrical test.

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
#include <stdlib.h>
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
#define C_IDLE		0x8000U					// RxIn : one step without a character

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
	bool bTxiEvt;					// TXI event, moved to the ICU slot
	bool bRxdLow;					// RXD held low, break
	std::vector<uint8_t> Wire;		// Characters sent
	unsigned StaleRdr;				// RDR read without RDRF or an error flag
	unsigned TxIgnored;				// TDR or FTDRL written with TE cleared
	unsigned RxLost;				// Characters lost by the receiver
	unsigned TxGap;					// Steps with the line idle and TX CFIFO bytes waiting
} SciModel_t;

typedef struct {
	int Evt;
	Ra4m1IRQHandler_t Handler;
	void *pCtx;
	bool bIr;						// ICU IR flag
	bool bReq;						// Request level of RXI and ERI, last seen
	bool bAcked;					// Ra4m1AcknowledgeInt called by the handler
	int Prio;						// NVIC priority given at registration
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
static bool s_bIrqOff;			// Interrupts held off by the application
static unsigned s_IrqPoints;
static unsigned s_StepEvery;		// Line step every n interrupt points, 0 none
static unsigned s_IsrStepEvery;		// Line step after n register accesses in the handler, 0 none
static unsigned s_IsrAccess;		// Register accesses in the handler since the last step
static unsigned s_LostEvt;			// Events that found IR already set
static unsigned s_IsrCnt;
static int s_PinCfgCnt;
static int s_PinDisCnt;

// DMAC model. Normal mode, 8 bit, source incremented, destination fixed,
// activated by the event in ICU.DELSR, the only setting the driver uses.
typedef struct {
	uint32_t Sar, Dar, Cra, Crb, Ofr;
	uint16_t Tmd, Amd;
	uint8_t Int, Cnt, Sts;
	bool bPend;						// Request held in DELSR while DTE is cleared
	uint8_t Late;					// Request taken, moved after this many register accesses, at a line step
	unsigned Moved;					// Bytes moved
} DmacModel_t;

#define DMAC_MSTP_BIT	(1UL << 22)

static DmacModel_t s_Dmac[4];
static uint32_t s_Delsr[4];
static uint8_t s_Dmast;
static uint32_t s_Mstpcra = 0xFFFFFFFFU;
static bool s_DmaHold;				// A request while DTE is cleared waits in DELSR, otherwise it is lost
static unsigned s_DmaLate;			// Register accesses before the channel moves a request it took, 0 at once
static bool s_StepAtCra;			// The line moves right after each read of DMCRA
static bool s_StepAtErrClear;		// The line moves right after a write clearing a receive error flag
static bool s_bDmaWrite;			// SCI register written by the DMAC
static UARTDev_t s_Uart;

R_SCI0_Type g_TestSci[4];
R_SYSTEM_Type g_TestSystem;
R_MSTP_Type g_TestMstp;
R_DMAC0_Type g_TestDmac[4];
R_DMA_Type g_TestDma;
R_ICU_Type g_TestIcu;
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
			m.bTxiEvt = true;
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

static void LineStep(int u);
static void UpdateIcu(void);

// Host address of a 32 bit DMAC address register value. The test data and
// the SCI register objects are in the same 4 GB of the host address space.
static uintptr_t HostAddr(uint32_t a)
{
	return ((uintptr_t)g_TestSci & ~(uintptr_t)0xFFFFFFFFU) | a;
}

static int DmacUnit(const void *pReg)
{
	int ch = (int)(((const char *)pReg - (const char *)g_TestDmac) / (long)sizeof(R_DMAC0_Type));

	CHECK(ch >= 0 && ch < 4);
	CHECK((s_Mstpcra & DMAC_MSTP_BIT) == 0);

	return ch;
}

static int DelsrIdx(const void *pReg)
{
	int i = (int)(((const char *)pReg - (const char *)g_TestIcu.DELSR) / (long)sizeof(g_TestIcu.DELSR[0]));

	CHECK(i >= 0 && i < 4);

	return i;
}

static void DmaTransfer(int Ch);

static uint32_t RegRead(const void *pReg, int Id)
{
	if (Id == SYSREG_PRCR)
	{
		return s_Prcr;
	}
	if (Id == SYSREG_MSTPCRA)
	{
		return s_Mstpcra;
	}
	if (Id == SYSREG_MSTPCRB)
	{
		return s_Mstpcrb;
	}
	if (Id == ICUREG_DELSR)
	{
		int i = DelsrIdx(pReg);
		return s_Delsr[i] | (s_Dmac[i].bPend ? R_ICU_DELSR_IR_Msk : 0);
	}
	if (Id == DMAREG_DMAST)
	{
		CHECK((s_Mstpcra & DMAC_MSTP_BIT) == 0);
		return s_Dmast;
	}
	if (Id >= DMAREG_SAR && Id <= DMAREG_STS)
	{
		DmacModel_t &d = s_Dmac[DmacUnit(pReg)];
		switch (Id)
		{
			case DMAREG_SAR: return d.Sar;
			case DMAREG_DAR: return d.Dar;
			case DMAREG_CRA: return d.Cra;
			case DMAREG_CRB: return d.Crb;
			case DMAREG_TMD: return d.Tmd;
			case DMAREG_INT: return d.Int;
			case DMAREG_AMD: return d.Amd;
			case DMAREG_OFR: return d.Ofr;
			case DMAREG_CNT: return d.Cnt;
			default: return d.Sts | (d.Late ? R_DMAC0_DMSTS_ACT_Msk : 0);
		}
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

static void RegWrite(const void *pReg, int Id, uint32_t Val)
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
	if (Id == SYSREG_MSTPCRA)
	{
		CHECK(s_Prcr & R_SYSTEM_PRCR_PRC1_Msk);
		s_Mstpcra = Val;
		return;
	}
	if (Id == ICUREG_DELSR)
	{
		// A held request goes with the write, IR is written 0
		int i = DelsrIdx(pReg);
		s_Delsr[i] = Val & R_ICU_DELSR_DELS_Msk;
		s_Dmac[i].bPend = false;
		return;
	}
	if (Id == DMAREG_DMAST)
	{
		CHECK((s_Mstpcra & DMAC_MSTP_BIT) == 0);
		s_Dmast = (uint8_t)Val;
		return;
	}
	if (Id >= DMAREG_SAR && Id <= DMAREG_STS)
	{
		int ch = DmacUnit(pReg);
		DmacModel_t &d = s_Dmac[ch];
		if (Id != DMAREG_CNT && Id != DMAREG_STS)
		{
			// Channel settings are written with DTE cleared and no transfer
			// in progress (DMSTS.ACT)
			CHECK((d.Cnt & R_DMAC0_DMCNT_DTE_Msk) == 0 && d.Late == 0);
		}
		switch (Id)
		{
			case DMAREG_SAR: d.Sar = Val; break;
			case DMAREG_DAR: d.Dar = Val; break;
			case DMAREG_CRA: d.Cra = Val; break;
			case DMAREG_CRB: d.Crb = Val; break;
			case DMAREG_TMD: d.Tmd = (uint16_t)Val; break;
			case DMAREG_INT: d.Int = (uint8_t)Val; break;
			case DMAREG_AMD: d.Amd = (uint16_t)Val; break;
			case DMAREG_OFR: d.Ofr = Val; break;
			case DMAREG_STS: d.Sts &= (uint8_t)Val; break;
			default:
				// Normal mode with a count of 0 transfers with no end, not used
				CHECK((Val & R_DMAC0_DMCNT_DTE_Msk) == 0 || d.Cra != 0);
				d.Cnt = (uint8_t)(Val & R_DMAC0_DMCNT_DTE_Msk);
				if ((d.Cnt & R_DMAC0_DMCNT_DTE_Msk) && d.bPend && (s_Dmast & R_DMA_DMAST_DMST_Msk))
				{
					// The held request is taken
					d.bPend = false;
					DmaTransfer(ch);
				}
				break;
		}
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
			if (Fifo(m) && (v & TE) && (old & TE) && (v & TIE) && !(old & TIE) && (m.Flags & B_TDRE))
			{
				// With the FIFO, TIE set while TDFE is 1 gives a TXI, the
				// Renesas FSP starts its transfers that way
				m.bTxiEvt = true;
			}
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
					m.bTxiEvt = true;
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
			if (Fifo(m))
			{
				// FIFO flags are cleared only once their condition is gone
				bool fer = false;
				bool per = false;
				for (uint16_t c : m.RxFifo)
				{
					fer |= (c & C_FER) != 0;
					per |= (c & C_PER) != 0;
				}
				if (m.RxFifo.size() >= RxTrigger(m))
				{
					clearable &= ~B_RDRF;
				}
				if (!m.RxFifo.empty())
				{
					clearable &= ~B_DR;
				}
				if (m.TxFifo.size() <= TxTrigger(m))
				{
					clearable &= ~B_TDRE;
				}
				if (fer)
				{
					clearable &= ~B_FER;
				}
				if (per)
				{
					clearable &= ~B_PER;
				}
			}
			else
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
			if (s_bDmaWrite && m.TxFifo.size() > TxTrigger(m))
			{
				// A write by the DMAC clears TDFE once above the trigger level
				m.Flags &= ~B_TDRE;
			}
			break;
		default:
			CHECK(false);
			break;
	}
}

// With s_DmaLate, a request a channel took is moved s_DmaLate register accesses
// later, or at the next line step. It finishes even when DTE was cleared
// meanwhile, DMSTS.ACT is set until then.
static void DmaLateTake(bool bStep)
{
	for (int ch = 0; ch < 4; ch++)
	{
		DmacModel_t &d = s_Dmac[ch];
		if (d.Late && (bStep || --d.Late == 0))
		{
			d.Late = 0;
			DmaTransfer(ch);
		}
	}
}

// Line state after a register access. In the handler the line moves once
// s_IsrStepEvery accesses are made since the last step, the time the handler
// takes.
static void AccessDone(void)
{
	DmaLateTake(false);
	UpdateIcu();
	if (s_bInIsr && s_IsrStepEvery && ++s_IsrAccess >= s_IsrStepEvery)
	{
		for (int u = 0; u < 4; u++)
		{
			LineStep(u);
		}
	}
}

uint32_t TestRegRead(const void *pReg, int Id)
{
	uint32_t v = RegRead(pReg, Id);

	AccessDone();
	if (s_StepAtCra && Id == DMAREG_CRA)
	{
		for (int u = 0; u < 4; u++)
		{
			LineStep(u);
		}
	}

	return v;
}

void TestRegWrite(const void *pReg, int Id, uint32_t Val)
{
	RegWrite(pReg, Id, Val);
	AccessDone();
	if (s_StepAtErrClear && Id == SCIREG_SSR && (~Val & B_RXERR))
	{
		for (int u = 0; u < 4; u++)
		{
			LineStep(u);
		}
	}
}

// A character on RXD
static void Arrive(SciModel_t &m, uint16_t c)
{
	uint16_t err = c & (C_FER | C_PER);

	if (Fifo(m))
	{
		if (m.Flags & B_ORER)
		{
			// Reception stopped until ORER is cleared
			m.RxLost++;
			return;
		}
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

	if (u == 0)
	{
		// One step of all units is one character time
		s_IsrAccess = 0;
		DmaLateTake(true);
	}
	if (s_Mstpcrb & s_MstpBit[u])
	{
		return;
	}
	bool idle = !m.RxIn.empty() && m.RxIn.front() == C_IDLE;
	if (idle)
	{
		m.RxIn.pop_front();
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
			if (s_Uart.DevIntrf.pDevData == &s_Ra4m1UartDev[u] && s_Uart.hTxFifo && CFifoUsed(s_Uart.hTxFifo) > 0)
			{
				m.TxGap++;
			}
		}
	}

	if ((m.Scr & RE) && !idle && !m.RxIn.empty())
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
	UpdateIcu();
}

// Unit and kind of a slot : 0 RXI, 1 TXI, 3 ERI. -1 for another event.
static int SlotUnit(int Slot, int *pKind)
{
	for (int i = 0; i < 4; i++)
	{
		int k = s_Slot[Slot].Evt - s_EvtRxi[i];
		if (k == 0 || k == 1 || k == 3)
		{
			*pKind = k;
			return (s_Mstpcrb & s_MstpBit[i]) ? -1 : i;
		}
	}
	return -1;
}

// One byte moved by channel Ch for a request
static void DmaTransfer(int Ch)
{
	DmacModel_t &d = s_Dmac[Ch];
	int u = -1;
	int id = 0;

	CHECK((d.Tmd & (R_DMAC0_DMTMD_MD_Msk | R_DMAC0_DMTMD_SZ_Msk | R_DMAC0_DMTMD_DCTG_Msk)) ==
		  (1U << R_DMAC0_DMTMD_DCTG_Pos));
	CHECK((d.Amd & (R_DMAC0_DMAMD_SM_Msk | R_DMAC0_DMAMD_DM_Msk)) == (2U << R_DMAC0_DMAMD_SM_Pos));
	CHECK(d.Cra > 0);
	for (int i = 0; i < 4; i++)
	{
		if (d.Dar == (uint32_t)(uintptr_t)&g_TestSci[i].FTDRL)
		{
			u = i;
			id = SCIREG_FTDRL;
		}
		if (d.Dar == (uint32_t)(uintptr_t)&g_TestSci[i].TDR)
		{
			u = i;
			id = SCIREG_TDR;
		}
	}
	CHECK(u >= 0);
	if (u < 0 || d.Cra == 0)
	{
		d.Cnt = 0;
		return;
	}

	uint8_t v = *(const uint8_t *)HostAddr(d.Sar);

	s_bDmaWrite = true;
	RegWrite(&g_TestSci[u], id, v);
	s_bDmaWrite = false;
	d.Sar++;
	d.Moved++;
	if (--d.Cra == 0)
	{
		d.Cnt = 0;
		d.Sts |= R_DMAC0_DMSTS_DTIF_Msk;
	}
}

// IR set by the events of the SCI and the DMAC. A TXI linked to a DMAC channel
// in DELSR goes to the channel instead.
static void UpdateIcu(void)
{
	// A transfer can give the next TXI, a TDR write to an idle line
	for (bool any = true; any; )
	{
		any = false;
		for (int u = 0; u < 4; u++)
		{
			for (int ch = 0; ch < 4 && s_Sci[u].bTxiEvt; ch++)
			{
				if (s_Delsr[ch] != (uint32_t)(s_EvtRxi[u] + 1))
				{
					continue;
				}
				DmacModel_t &d = s_Dmac[ch];
				s_Sci[u].bTxiEvt = false;
				any = true;
				if ((d.Cnt & R_DMAC0_DMCNT_DTE_Msk) && (s_Dmast & R_DMA_DMAST_DMST_Msk))
				{
					if (s_DmaLate)
					{
						if (d.Late)
						{
							// One transfer at a time, the one in progress ends first
							d.Late = 0;
							DmaTransfer(ch);
						}
						if (d.Cnt & R_DMAC0_DMCNT_DTE_Msk)
						{
							d.Late = (uint8_t)s_DmaLate;
						}
						else if (s_DmaHold)
						{
							d.bPend = true;
						}
					}
					else
					{
						DmaTransfer(ch);
					}
				}
				else if (s_DmaHold)
				{
					d.bPend = true;
				}
			}
		}
	}

	for (int i = 0; i < 32; i++)
	{
		int kind = 0;
		int u = s_Slot[i].Handler ? SlotUnit(i, &kind) : -1;
		int ev = s_Slot[i].Evt;

		if (s_Slot[i].Handler && ev >= (int)RA4M1_EVTID_DMAC0_INT && ev <= (int)RA4M1_EVTID_DMAC3_INT)
		{
			// Transfer end of a DMAC channel
			DmacModel_t &d = s_Dmac[ev - RA4M1_EVTID_DMAC0_INT];
			bool req = (d.Sts & R_DMAC0_DMSTS_DTIF_Msk) && (d.Int & R_DMAC0_DMINT_DTIE_Msk);
			if (req && !s_Slot[i].bReq)
			{
				if (s_Slot[i].bIr)
				{
					s_LostEvt++;
				}
				s_Slot[i].bIr = true;
			}
			s_Slot[i].bReq = req;
			continue;
		}
		if (u < 0)
		{
			continue;
		}

		SciModel_t &m = s_Sci[u];
		bool evt = false;

		if (kind == 1)
		{
			evt = m.bTxiEvt;
			m.bTxiEvt = false;
		}
		else
		{
			bool req = (m.Scr & RIE) && (kind == 0 ? (m.Flags & (Fifo(m) ? (B_RDRF | B_DR) : B_RDRF)) :
										 (m.Flags & B_RXERR));
			evt = req && !s_Slot[i].bReq;
			s_Slot[i].bReq = req;
		}
		if (evt)
		{
			if (s_Slot[i].bIr)
			{
				s_LostEvt++;
			}
			s_Slot[i].bIr = true;
		}
	}
}

static void Dispatch(void)
{
	if (s_bInIsr || s_bIrqOff)
	{
		return;
	}
	UpdateIcu();
	for (int n = 0; ; n++)
	{
		int slot = -1;
		for (int i = 0; i < 32 && slot < 0; i++)
		{
			if (s_Slot[i].Handler && s_Slot[i].bIr)
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
		uint32_t pm = g_TestPrimask;
		s_bInIsr = true;
		g_TestPrimask = 1;
		s_Slot[slot].bAcked = false;
		s_Slot[slot].Handler(slot, s_Slot[slot].pCtx);
		if (!s_Slot[slot].bAcked)
		{
			// The dispatcher clears IR after a handler that did not
			s_Slot[slot].bIr = false;
		}
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
	for (int i = 0; i < 4; i++)
	{
		if (s_Delsr[i] == EvtId)
		{
			// Same as the ICU driver : an event linked to a DMAC channel
			return (IRQn_Type)-1;
		}
	}
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
			s_Slot[i] = IcuSlot_t();
			s_Slot[i].Evt = EvtId;
			s_Slot[i].Handler = pHandler;
			s_Slot[i].pCtx = pCtx;
			s_Slot[i].Prio = Prio;
			return (IRQn_Type)i;
		}
	}
	return (IRQn_Type)-1;
}

void Ra4m1UnregisterIntHandler(IRQn_Type IrqNo)
{
	if ((unsigned)IrqNo < 32)
	{
		s_Slot[IrqNo] = IcuSlot_t();
	}
}

bool Ra4m1AcknowledgeInt(IRQn_Type IrqNo)
{
	if ((unsigned)IrqNo >= 32 || s_Slot[IrqNo].Handler == NULL)
	{
		return false;
	}
	s_Slot[IrqNo].bIr = false;
	s_Slot[IrqNo].bAcked = true;

	return true;
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
	s_Mstpcra = 0xFFFFFFFFU;
	for (int i = 0; i < 4; i++)
	{
		s_Dmac[i] = DmacModel_t();
		s_Delsr[i] = 0;
	}
	s_Dmast = 0;
	s_Prcr = 0;
	s_StepEvery = 0;
	s_IsrStepEvery = 0;
	s_IsrAccess = 0;
	s_bIrqOff = false;
	s_StepAtCra = false;
	s_StepAtErrClear = false;
	s_EvtRx = s_EvtTx = s_EvtLine = 0;
	s_LastLine = 0;
	s_RefillLeft = 0;
	memset((void *)&s_Uart, 0, sizeof(s_Uart));
}

static UARTCfg_t Cfg(int DevNo, bool bIntMode, bool bDma = false)
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
	cfg.bDMAMode = bDma;
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
		for (int mode = 0; mode < 4; mode++)
		{
			bool intmode = mode & 1;
			bool dma = mode & 2;
			ResetModel();
			cfg = Cfg(u, intmode, dma);
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
			CHECK(s_Uart.DevIntrf.bDma == dma);
			CHECK(s_Uart.DevIntrf.bIntEn == intmode);
			CHECK(s_Uart.DevIntrf.EnCnt == 1);
			CHECK(s_Uart.bTxReady == true);
			CHECK(SlotCnt() == (intmode ? 3 : 0));
			int txslot = -1;
			for (int i = 0; i < 32; i++)
			{
				if (s_Slot[i].Handler && s_Slot[i].Evt == (int)(dma ? RA4M1_EVTID_DMAC0_INT + u : s_EvtRxi[u] + 1))
				{
					txslot = i;
				}
			}
			CHECK((txslot >= 0) == intmode);
			if (dma)
			{
				// TXI linked to DMAC channel DevNo, the DMAC running
				CHECK(s_Delsr[u] == (uint32_t)(s_EvtRxi[u] + 1));
				CHECK((s_Mstpcra & DMAC_MSTP_BIT) == 0 && (s_Dmast & R_DMA_DMAST_DMST_Msk));
				CHECK((s_Dmac[u].Int & R_DMAC0_DMINT_DTIE_Msk) == (intmode ? R_DMAC0_DMINT_DTIE_Msk : 0));
				CHECK(u >= 2 || TxTrigger(m) == 0);
			}
			else
			{
				CHECK(s_Delsr[u] == 0);
				CHECK(u >= 2 || TxTrigger(m) == 8);
			}
			CHECK(s_PinCfgCnt == 2);
			CHECK(UARTGetInstance(u) == &s_Uart);
			CHECK(UARTGetDevHandle(&s_Uart.DevIntrf) == &s_Uart);
		}
	}

	// DMA needs a blocking FIFO and a free DMAC channel. Without DMA, a link
	// left by an earlier UARTInit is removed and TXI goes to the CPU again.
	ResetModel();
	cfg = Cfg(1, true, true);
	cfg.bFifoBlocking = false;
	CHECK(UARTInit(&s_Uart, &cfg) == false);
	s_Delsr[1] = RA4M1_EVTID_DMAC0_INT;
	cfg = Cfg(1, true, true);
	CHECK(UARTInit(&s_Uart, &cfg) == false);
	s_Delsr[1] = 0;
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	CHECK(s_Delsr[1] == (uint32_t)(s_EvtRxi[1] + 1));
	cfg = Cfg(1, true);
	CHECK(UARTInit(&s_Uart, &cfg) == true);
	CHECK(s_Delsr[1] == 0 && SlotCnt() == 3);

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
static void TestTx(int u, bool bIntMode, int Len, int Chunk, bool bDma = false)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, bIntMode, bDma);
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
	if (bDma)
	{
		// The CPU fills the SCI while the channel is idle, the DMAC moves the
		// rest
		CHECK((s_Dmac[u].Cnt & R_DMAC0_DMCNT_DTE_Msk) == 0);
		CHECK(s_Dmac[u].Moved > 0);
		if (bIntMode)
		{
			// The line is idle only while the first UARTTx copies its data,
			// a line step every second 8 byte piece. Blocks follow each other
			// with no idle.
			int first = Chunk < 256 ? Chunk : 256;
			CHECK(s_Sci[u].TxGap <= (unsigned)((first + 15) / 16 + 1));
		}
	}
	if (s_Sci[u].Wire != d || (bDma && bIntMode && s_Sci[u].TxGap > (unsigned)(((Chunk < 256 ? Chunk : 256) + 15) / 16 + 1)))
	{
		printf("  TestTx unit %d int %d dma %d hold %d chunk %d : %zu of %d sent, TX FIFO %zu, TIE %d, TDFE %d, "
			   "DMA moved %u, gaps %u\n", u, bIntMode, bDma, s_DmaHold, Chunk, s_Sci[u].Wire.size(), Len,
			   s_Sci[u].TxFifo.size(), (s_Sci[u].Scr & TIE) != 0, (s_Sci[u].Flags & B_TDRE) != 0, s_Dmac[u].Moved,
			   s_Sci[u].TxGap);
	}
}

// TX DMA with the end of a block seen late: the line moves Late steps between
// UARTTx calls, with interrupts held off in interrupt mode. The SCI asks for
// data while the channel is stopped, with the request held in DELSR it must
// not move a byte into a full TX FIFO or TDR when the next block is armed.
static void TestTxLate(int u, bool bIntMode, int Late, int Chunk)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, bIntMode, true);
	if (Chunk < 16)
	{
		cfg.pTxMem = NULL;
		cfg.TxMemSize = 0;
	}
	CHECK(UARTInit(&s_Uart, &cfg) == true);

	std::vector<uint8_t> d;
	Pattern(d, 2000, (uint8_t)(u + Late + Chunk));
	int sent = 0;
	for (int guard = 0; sent < (int)d.size() && guard < 100000; guard++)
	{
		int l = (int)d.size() - sent < Chunk ? (int)d.size() - sent : Chunk;
		sent += UARTTx(&s_Uart, d.data() + sent, l);
		s_bIrqOff = true;
		RunLine(Late);
		s_bIrqOff = false;
		Dispatch();
	}
	Drain(u, d.size());
	RunLine(4);

	CHECK(s_Sci[u].Wire == d);
	CHECK(s_Uart.bTxReady == true);
	CHECK((s_Sci[u].Scr & TIE) == 0);
	CHECK((s_Dmac[u].Cnt & R_DMAC0_DMCNT_DTE_Msk) == 0);
	if (s_Sci[u].Wire != d)
	{
		printf("  TestTxLate unit %d int %d hold %d late %d chunk %d : %zu of %zu sent, DMA moved %u\n", u,
			   bIntMode, s_DmaHold, Late, Chunk, s_Sci[u].Wire.size(), d.size(), s_Dmac[u].Moved);
	}
}

// SCI2/9 block start: the line moves right after the CPU reads DMCRA and the
// channel moves the byte 1 to 3 register accesses after it took the request,
// the last one of a 1 byte block, while the CPU checks TDR and stops the
// channel. With DMCRA 0, DTE set again would be a transfer with no end (CHECK
// in the DMCNT write), DMCRA read before the transfer ends is not its count
// (settings written during a transfer are a CHECK).
static void TestTxDmaLastByte(void)
{
	bool hold = s_DmaHold;
	unsigned late = s_DmaLate;

	s_DmaHold = true;
	for (int u = 2; u < 4; u++)
	{
		for (int intmode = 0; intmode < 2; intmode++)
		{
			for (unsigned every = 1; every <= 4; every *= 2)
			{
				for (s_DmaLate = 1; s_DmaLate <= 3; s_DmaLate++)
				{
					ResetModel();
					UARTCfg_t cfg = Cfg(u, intmode, true);
					cfg.pTxMem = NULL;
					cfg.TxMemSize = 0;
					CHECK(UARTInit(&s_Uart, &cfg) == true);
					s_StepAtCra = true;
					s_StepEvery = every;

					std::vector<uint8_t> d;
					Pattern(d, 500, (uint8_t)(u + every + s_DmaLate));
					SendAll(d.data(), (int)d.size(), 1);
					Drain(u, d.size());
					RunLine(4);
					CHECK(s_Sci[u].Wire == d);
					CHECK((s_Dmac[u].Cnt & R_DMAC0_DMCNT_DTE_Msk) == 0);
					if (s_Sci[u].Wire != d)
					{
						printf("  TestTxDmaLastByte unit %d int %d step every %u late %u : %zu of %zu sent\n", u,
							   intmode, every, s_DmaLate, s_Sci[u].Wire.size(), d.size());
					}
				}
			}
		}
	}
	s_StepAtCra = false;
	s_DmaHold = hold;
	s_DmaLate = late;
}

// SCI2/9, interrupt mode: one overrun while interrupts are held off for two
// characters, then the next character arrives while that error is handled
// (the line moves right after the error flags are cleared). Only the
// character of the overrun is lost: the RXI of the next one is not dropped
// with the error, which would leave it in RDR for the following one to
// overrun, and so on.
static void TestRxOverrunOnce(void)
{
	for (int u = 2; u < 4; u++)
	{
		for (int dma = 0; dma < 2; dma++)
		{
			ResetModel();
			UARTCfg_t cfg = Cfg(u, true, dma);
			CHECK(UARTInit(&s_Uart, &cfg) == true);

			std::vector<uint8_t> d;
			// Fewer than the 256 byte RX CFIFO, read at the end
			Pattern(d, 200, (uint8_t)(u + dma));
			for (uint8_t c : d)
			{
				s_Sci[u].RxIn.push_back(c);
			}
			RunLine(50);
			s_StepAtErrClear = true;
			s_bIrqOff = true;
			RunLine(2);
			s_bIrqOff = false;
			Dispatch();
			RunLine(200);
			s_StepAtErrClear = false;

			std::vector<uint8_t> r;
			uint8_t buf[64];
			int n;
			while ((n = UARTRx(&s_Uart, buf, sizeof(buf))) > 0)
			{
				r.insert(r.end(), buf, buf + n);
			}
			std::vector<uint8_t> e = d;
			e.erase(e.begin() + 51);
			CHECK(s_Sci[u].RxLost == 1);
			CHECK(s_Sci[u].StaleRdr == 0);
			CHECK(s_Uart.RxOvrErrCnt == 1);
			CHECK(r == e);
			if (r != e || s_Sci[u].RxLost != 1)
			{
				printf("  TestRxOverrunOnce unit %d dma %d : %zu of %zu received, lost %u, overruns %u\n", u, dma,
					   r.size(), d.size(), s_Sci[u].RxLost, (unsigned)s_Uart.RxOvrErrCnt);
			}
		}
	}
}

// Interrupt priorities: RXI and ERI at IntPrio. SCI2/9 have no RX FIFO, their
// TXI or DMA transfer end is one level lower (a larger number) so that RXI can
// preempt it, unless IntPrio is already the lowest. SCI0/1 use IntPrio for all.
static void TestIntPrio(void)
{
	for (int u = 0; u < 4; u++)
	{
		for (int dma = 0; dma < 2; dma++)
		{
			for (int prio = 1; prio <= 15; prio += 14)
			{
				ResetModel();
				UARTCfg_t cfg = Cfg(u, true, dma);
				cfg.IntPrio = prio;
				CHECK(UARTInit(&s_Uart, &cfg) == true);
				Ra4m1UartDev_t *dev = &s_Ra4m1UartDev[u];
				int tx = dma ? dev->IrqDma : dev->IrqTxi;
				int txprio = u >= 2 && prio < 15 ? prio + 1 : prio;
				CHECK(s_Slot[dev->IrqRxi].Prio == prio);
				CHECK(s_Slot[dev->IrqEri].Prio == prio);
				CHECK(tx >= 0 && s_Slot[tx].Prio == txprio);
			}
		}
	}
}

// RX of Len bytes, read Chunk bytes per UARTRx call
static void TestRx(int u, bool bIntMode, int Len, int Chunk, bool bDma = false)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, bIntMode, bDma);
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
static void TestLoopback(int u, bool bIntMode, int Len, bool bDma = false)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, bIntMode, bDma);
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

// Echo as exemples/uart/uart_loopback.cpp does it, the characters arriving
// in bursts of 1 to 40 with 0 to 4 idle steps between them. IsrStep : line
// step after n register accesses in the handler, 0 none.
static void TestLoopbackBursts(int u, bool bIntMode, int Len, unsigned IsrStep, unsigned Seed, bool bDma = false)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, bIntMode, bDma);
	CHECK(UARTInit(&s_Uart, &cfg) == true);

	std::vector<uint8_t> d;
	Pattern(d, Len, (uint8_t)(0x20 + u));
	unsigned x = Seed;
	for (int i = 0; i < Len; )
	{
		x = x * 1103515245U + 12345U;
		int burst = 1 + (int)((x >> 16) % 40);
		for (int k = 0; k < burst && i < Len; k++, i++)
		{
			s_Sci[u].RxIn.push_back(d[i]);
		}
		x = x * 1103515245U + 12345U;
		for (int k = (int)((x >> 16) % 5); k > 0; k--)
		{
			s_Sci[u].RxIn.push_back(C_IDLE);
		}
	}
	s_StepEvery = bIntMode || u < 2 ? 5 : 0;
	s_IsrStepEvery = IsrStep;
	unsigned lostevt = s_LostEvt;
	size_t last = 0;
	int still = 0;
	// Ends when all is echoed, or nothing moved for 2000 steps
	while (s_Sci[u].Wire.size() < (size_t)Len && still < 2000)
	{
		uint8_t buf[16];
		int n = UARTRx(&s_Uart, buf, sizeof(buf));
		if (n > 0)
		{
			SendAll(buf, n, n);
		}
		RunLine(1);
		still = s_Sci[u].Wire.size() == last ? still + 1 : 0;
		last = s_Sci[u].Wire.size();
	}
	CHECK(s_Sci[u].Wire == d);
	CHECK(s_Sci[u].RxLost == 0);
	CHECK(s_Sci[u].StaleRdr == 0);
	CHECK(s_Uart.RxOvrErrCnt == 0 && s_Uart.RxDropCnt == 0);
	if (s_Sci[u].Wire != d && getenv("UART_TEST_DUMP"))
	{
		size_t k = 0;
		while (k < d.size() && k < s_Sci[u].Wire.size() && s_Sci[u].Wire[k] == d[k]) k++;
		printf("  first difference at %zu :", k);
		for (size_t j = k > 3 ? k - 3 : 0; j < k + 8 && j < s_Sci[u].Wire.size(); j++) printf(" %02x", s_Sci[u].Wire[j]);
		printf("  expected");
		for (size_t j = k > 3 ? k - 3 : 0; j < k + 8 && j < d.size(); j++) printf(" %02x", d[j]);
		printf("\n");
	}
	if (s_Sci[u].Wire != d || s_Sci[u].RxLost)
	{
		printf("  TestLoopbackBursts unit %d int %d dma %d hold %d isr step %u seed %u : %zu of %d echoed, lost %u, "
			   "ovr %u, events lost %u\n", u, bIntMode, bDma, s_DmaHold, IsrStep, Seed, s_Sci[u].Wire.size(), Len,
			   s_Sci[u].RxLost, s_Uart.RxOvrErrCnt, s_LostEvt - lostevt);
	}
}

// Byte mode as exemples/uart/uart_prbs_tx.cpp, one UARTTx call per byte, with
// the 16 byte default CFIFOs. The line moves one character every StepEvery
// interrupt points: the application as fast as the line, 2 or 4 times faster.
// Returns the number of interrupts, the line must never be idle.
static unsigned TxByteMode(int u, bool bDma, unsigned StepEvery, unsigned IsrStep)
{
	ResetModel();
	UARTCfg_t cfg = Cfg(u, true, bDma);
	cfg.pTxMem = NULL;
	cfg.TxMemSize = 0;
	cfg.pRxMem = NULL;
	cfg.RxMemSize = 0;
	CHECK(UARTInit(&s_Uart, &cfg) == true);

	std::vector<uint8_t> d;
	Pattern(d, 2000, (uint8_t)(StepEvery + u));
	s_StepEvery = StepEvery;
	s_IsrStepEvery = IsrStep;
	unsigned isr = s_IsrCnt;
	SendAll(d.data(), (int)d.size(), 1);
	Drain(u, d.size());
	CHECK(s_Sci[u].Wire == d);
	CHECK(s_Sci[u].TxGap == 0);
	if (s_Sci[u].Wire != d || s_Sci[u].TxGap)
	{
		printf("  TxByteMode unit %d dma %d hold %d step every %u isr step %u : %zu sent, gaps %u\n", u, bDma,
			   s_DmaHold, StepEvery, IsrStep, s_Sci[u].Wire.size(), s_Sci[u].TxGap);
	}

	return s_IsrCnt - isr;
}

// DMA mode takes no more interrupts than interrupt mode. The CPU fills the SCI
// while the channel is idle, an application writing a byte at a time no faster
// than the line takes none.
static void TestTxByteMode(void)
{
	for (int u = 0; u < 4; u++)
	{
		for (unsigned every = 1; every <= 4; every *= 2)
		{
			for (unsigned isrstep = 0; isrstep <= 8; isrstep += 8)
			{
				unsigned irq = TxByteMode(u, false, every, isrstep);
				unsigned dmairq = TxByteMode(u, true, every, isrstep);
				CHECK(dmairq <= irq);
				CHECK(every > 1 || dmairq == 0);
				if (dmairq > irq || (every == 1 && dmairq))
				{
					printf("  TestTxByteMode unit %d hold %d step every %u isr step %u : interrupts %u, with DMA %u\n",
						   u, s_DmaHold, every, isrstep, irq, dmairq);
				}
			}
		}
	}
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
	TestIntPrio();
	// Without DMA, then with DMA for a request made while DTE is cleared held
	// in DELSR and lost, then held with the channel taking each request one
	// register access or line step later
	for (int run = 0; run < 4; run++)
	{
		bool dma = run > 0;
		s_DmaHold = run == 1 || run == 3;
		s_DmaLate = run == 3 ? 1 : 0;
		for (int u = 0; u < 4; u++)
		{
			for (int intmode = 0; intmode < 2; intmode++)
			{
				TestTx(u, intmode, 1000, 1, dma);
				TestTx(u, intmode, 1000, 16, dma);
				TestTx(u, intmode, 1000, 300, dma);
				TestRx(u, intmode, 1000, 1, dma);
				TestRx(u, intmode, 1000, 16, dma);
				TestRx(u, intmode, 1000, 64, dma);
				TestLoopback(u, intmode, 2000, dma);
				// Without FIFO, a handler slower than one character overruns RDR.
				// With DMA, TXI goes to the DMAC and only RXI drains the RX FIFO:
				// with 4 register accesses per character an RXI takes about 7
				// characters and the 16 byte FIFO overruns. Without DMA the TXI
				// handler drains it too.
				for (unsigned isrstep = 0; isrstep < 20; isrstep += (isrstep == 0 && (u >= 2 || dma)) ? 8 : 4)
				{
					for (unsigned seed = 1; seed <= 3; seed++)
					{
						TestLoopbackBursts(u, intmode, 3000, isrstep, seed, dma);
					}
				}
			}
		}
		if (dma)
		{
			TestTxByteMode();
			for (int u = 0; u < 4; u++)
			{
				for (int intmode = 0; intmode < 2; intmode++)
				{
					// 60 : the line is idle before each UARTTx call
					static const int late[] = { 1, 2, 3, 4, 12, 20, 60 };
					for (int l : late)
					{
						TestTxLate(u, intmode, l, 1);
						TestTxLate(u, intmode, l, 40);
					}
				}
			}
		}
	}
	s_DmaHold = false;
	s_DmaLate = 0;
	TestTxDmaLastByte();
	TestRxOverrunOnce();
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
