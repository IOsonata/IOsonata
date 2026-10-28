// Host regression of the LPC546xx I2S driver on Flexcomm 6 and 7, in polling,
// interrupt and DMA modes, transmit, receive and full duplex, master and
// slave.
//
// Bus model. A Flexcomm in its I2S function (PSELID PERSEL 4 transmit, 5
// receive) with MAINENABLE takes part in the frames. The master (CFG1
// MSTSLVCFG 3) makes a frame every 32 SCK, SCK = function clock / (DIV + 1),
// the function clock the audio PLL (FCLKSEL 2 required); in slave mode
// (MSTSLVCFG 0 on every Flexcomm) the test makes the frames at its own rate.
// A Flexcomm takes part in the frames that start after its MAINENABLE: a
// slave of a full duplex enabled after the master misses frames, which the
// test sees. Each frame: a transmit Flexcomm takes the first entry of its TX
// FIFO, or sends 0 and sets TXERR when it is empty (TXI2SE0 required); a
// receive Flexcomm pushes the frame of the codec into its RX FIFO, or sets
// RXERR when it is full. The codec sends a counting pattern, or in loopback
// the frame it receives. Frames need CFG1 MODE 0 (classic I2S), DATALEN 15,
// CFG2 FRAMELEN 31, POSITION 0. FIFOs have 8 entries; a write to a full TX
// FIFO or a read of an empty RX FIFO fails the test. DMA requests: TX while
// the TX FIFO is not full with DMATX, RX while the RX FIFO is not empty with
// DMARX. Interrupts: FIFOINTSTAT TXERR, RXERR, TXLVL (TX entries at most
// TXLVL), RXLVL (RX entries above RXLVL), masked by FIFOINTENSET.
//
// Time: 0.2 us per FIFOSTAT read, the test runs the time on in steps of 1 us.
// Each step the frames due are made, the DMA moves its values and the
// pending interrupts not masked run, one level deep.
//
// The DMA moves the first value of a channel when it is started (dma_model.h
// Immediate), as the controller does while the CPU goes on.
//
// The audio PLL is a stub here: it gives the frequency asked, or an offset,
// or fails (system_LPC546xx.c has its own test).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

#include "LPC546xx.h"
#include "dma_model.h"

uint32_t g_IrqMasked = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { s_NvicPrio[n & 63] = p; }
void NVIC_EnableIRQ(IRQn_Type n) { s_NvicEn |= 1UL << (n & 31); }
void NVIC_DisableIRQ(IRQn_Type n) { s_NvicEn &= ~(1UL << (n & 31)); }
void __DMB(void) {}

static double s_Us = 0;

// ---- Audio PLL stub ----

static uint32_t s_PllFreq = 0;
static uint32_t s_PllAsk = 0;
static int s_PllStarts = 0;
static bool s_PllFail = false;
static int32_t s_PllOffset = 0;

extern "C" uint32_t Lpc546xxAudioPllStart(uint32_t Freq)
{
	s_PllStarts++;
	s_PllAsk = Freq;
	s_PllFreq = s_PllFail || Freq == 0 ? 0 : (uint32_t)((int32_t)Freq + s_PllOffset);
	return s_PllFreq;
}
extern "C" void Lpc546xxAudioPllStop(void) { s_PllFreq = 0; }
extern "C" uint32_t Lpc546xxAudioPllFreq(void) { return s_PllFreq; }

// ---- Flexcomm and I2S ----

static void FcReset(const void *pPsel);
struct Psel {
	uint32_t value = 0x00000070U;
	operator uint32_t() const { return value; }
	Psel &operator=(uint32_t v) { value = (value & ~0xFU) | (v & 0xFU); FcReset(this); return *this; }
};
struct FcRegs {
	uint8_t Reserved[0xFF8];
	Psel PSELID;
	uint32_t PID;
};

#define FIFO_DEPTH	8U

struct I2sM {
	uint32_t Cfg1 = 0, Cfg2 = 0, Div = 0, FifoCfg = 0, FifoTrig = 0, IntEn = 0;
	std::deque<uint32_t> Tx, Rx;
	bool TxErr = false, RxErr = false;
	bool Run = false;
	double StartUs = 0;			// Master: time of MAINENABLE
	uint64_t FirstFrame = 0;	// First frame it takes part in
	std::vector<uint32_t> Sent;	// Transmit: entries on the bus
	int Underruns = 0, Overruns = 0;
	int SlaveLate = 0;			// Slave enabled after the master
};
static I2sM s_M[10];

static int RegNo(const void *p);
static void Tick(double Us);

struct Cfg1R {
	operator uint32_t() const;
	Cfg1R &operator=(uint32_t v);
	Cfg1R &operator|=(uint32_t v) { return *this = (uint32_t)*this | v; }
	Cfg1R &operator&=(uint32_t v) { return *this = (uint32_t)*this & v; }
};
struct PlainR {
	uint32_t I2sM::*f;
	operator uint32_t() const;
	PlainR &operator=(uint32_t v);
};
struct FifoCfgR {
	operator uint32_t() const;
	FifoCfgR &operator=(uint32_t v);
};
struct FifoStatR { operator uint32_t() const; FifoStatR &operator=(uint32_t v); };
struct FifoIntSetR { operator uint32_t() const; FifoIntSetR &operator=(uint32_t v); };
struct FifoIntClrR { FifoIntClrR &operator=(uint32_t v); };
struct FifoIntStatR { operator uint32_t() const; };
// Word aligned, the DMA takes their address
struct alignas(4) FifoWrR { uint32_t pad = 0; FifoWrR &operator=(uint32_t v); };
struct alignas(4) FifoRdR { uint32_t pad = 0; operator uint32_t() const; };

struct I2sRegs {
	Cfg1R CFG1;
	PlainR CFG2 = { &I2sM::Cfg2 };
	uint32_t STAT = 0;
	PlainR DIV = { &I2sM::Div };
	FifoCfgR FIFOCFG;
	FifoStatR FIFOSTAT;
	PlainR FIFOTRIG = { &I2sM::FifoTrig };
	FifoIntSetR FIFOINTENSET;
	FifoIntClrR FIFOINTENCLR;
	FifoIntStatR FIFOINTSTAT;
	FifoWrR FIFOWR;
	FifoRdR FIFORD;
};

static I2sRegs s_I2sRegs[2];			// Flexcomm 6 and 7
alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
static FcRegs s_Fc[10];

static int RegNo(const void *p)
{
	for (int i = 0; i < 2; i++)
	{
		if ((const uint8_t *)p >= (const uint8_t *)&s_I2sRegs[i] && (const uint8_t *)p < (const uint8_t *)&s_I2sRegs[i + 1])
		{
			return 6 + i;
		}
	}
	assert(false);
	return -1;
}

static uint32_t Persel(int Fc) { return s_Fc[Fc].PSELID.value & 7U; }
static bool IsTx(int Fc) { return Persel(Fc) == 4U; }
static bool IsRx(int Fc) { return Persel(Fc) == 5U; }
static bool IsMaster(int Fc) { return ((s_M[Fc].Cfg1 & I2S_CFG1_MSTSLVCFG_MASK) >> I2S_CFG1_MSTSLVCFG_SHIFT) == 3U; }

static void FcReset(const void *pPsel)
{
	for (int i = 0; i < 10; i++)
	{
		if (pPsel == &s_Fc[i].PSELID)
		{
			std::vector<uint32_t> sent = s_M[i].Sent;
			s_M[i] = I2sM();
			s_M[i].Sent = sent;
			return;
		}
	}
	assert(false);
}

// Frames made by the master, or by the test in slave mode
static double s_ExtRate = 0;			// Slave mode: frames per second
static double s_ExtStartUs = 0;
static uint64_t s_Frame = 0;			// Frames made
static bool s_Loopback = false;
static std::vector<uint32_t> s_CodecRx;	// Frames the codec received

static uint32_t Pattern(uint64_t n)
{
	return (uint32_t)(uint16_t)(n * 3U + 1U) | ((uint32_t)(uint16_t)(~n * 5U) << 16);
}

static double FcClk()
{
	return (double)s_PllFreq;
}

static int MasterFc()
{
	for (int fc = 6; fc <= 7; fc++)
	{
		if (s_M[fc].Run && IsMaster(fc))
		{
			return fc;
		}
	}
	return -1;
}

static void CheckFrameCfg(int Fc)
{
	const I2sM &m = s_M[Fc];

	assert(((m.Cfg1 & I2S_CFG1_MODE_MASK) >> I2S_CFG1_MODE_SHIFT) == 0U);
	assert(((m.Cfg1 & I2S_CFG1_DATALEN_MASK) >> I2S_CFG1_DATALEN_SHIFT) == 15U);
	assert((m.Cfg1 & (I2S_CFG1_ONECHANNEL_MASK | I2S_CFG1_PDMDATA_MASK | I2S_CFG1_SCK_POL_MASK |
					  I2S_CFG1_WS_POL_MASK | I2S_CFG1_LEFTJUST_MASK | I2S_CFG1_RIGHTLOW_MASK)) == 0U);
	assert(((m.Cfg1 & I2S_CFG1_PAIRCOUNT_MASK) >> I2S_CFG1_PAIRCOUNT_SHIFT) == 0U);
	assert(m.Cfg2 == I2S_CFG2_FRAMELEN(31));
	assert((((SYSCON_Type *)s_SysconMem)->FCLKSEL[Fc] & 7U) == 2U);
}

static void Frame(void)
{
	uint32_t tx = 0;

	for (int fc = 6; fc <= 7; fc++)
	{
		I2sM &m = s_M[fc];

		if (!m.Run || s_Frame < m.FirstFrame || !IsTx(fc))
		{
			continue;
		}
		CheckFrameCfg(fc);
		assert(m.FifoCfg & I2S_FIFOCFG_ENABLETX_MASK);
		if (m.Tx.empty())
		{
			assert(m.FifoCfg & I2S_FIFOCFG_TXI2SE0_MASK);
			m.TxErr = true;
			m.Underruns++;
			tx = 0;
		}
		else
		{
			tx = m.Tx.front();
			m.Tx.pop_front();
		}
		m.Sent.push_back(tx);
	}
	s_CodecRx.push_back(tx);

	uint32_t rx = s_Loopback ? tx : Pattern(s_Frame);

	for (int fc = 6; fc <= 7; fc++)
	{
		I2sM &m = s_M[fc];

		if (!m.Run || s_Frame < m.FirstFrame || !IsRx(fc))
		{
			continue;
		}
		CheckFrameCfg(fc);
		assert(m.FifoCfg & I2S_FIFOCFG_ENABLERX_MASK);
		if (m.Rx.size() >= FIFO_DEPTH)
		{
			m.RxErr = true;
			m.Overruns++;
		}
		else
		{
			m.Rx.push_back(rx);
		}
	}
	s_Frame++;
}

// Frames due at the current time
static void Bus(void)
{
	int mfc = MasterFc();
	double start, rate;

	if (mfc >= 0)
	{
		start = s_M[mfc].StartUs;
		rate = FcClk() / (s_M[mfc].Div + 1U) / 32.0;
	}
	else if (s_ExtRate > 0)
	{
		start = s_ExtStartUs;
		rate = s_ExtRate;
	}
	else
	{
		return;
	}
	assert(rate > 0);

	// Frames counted from the start of the clock source
	static double s_Start = -1;
	static uint64_t s_Base = 0;
	if (s_Start != start)
	{
		s_Start = start;
		s_Base = s_Frame;
	}
	uint64_t due = s_Base + (uint64_t)floor((s_Us - start) * rate / 1e6);

	while (s_Frame < due)
	{
		Frame();
	}
}

Cfg1R::operator uint32_t() const { return s_M[RegNo(this)].Cfg1; }
Cfg1R &Cfg1R::operator=(uint32_t v)
{
	int fc = RegNo(this);
	I2sM &m = s_M[fc];
	bool on = (v & I2S_CFG1_MAINENABLE_MASK) != 0;

	assert(IsTx(fc) || IsRx(fc));
	if (on && !m.Run)
	{
		Bus();
		m.Run = true;
		m.FirstFrame = s_Frame;
		m.Cfg1 = v;
		if (IsMaster(fc))
		{
			m.StartUs = s_Us;
			// A slave Flexcomm set up for the frames runs already
			for (int o = 6; o <= 7; o++)
			{
				if (o != fc && (IsTx(o) || IsRx(o)) && (s_M[o].FifoCfg & (I2S_FIFOCFG_ENABLETX_MASK | I2S_FIFOCFG_ENABLERX_MASK)) &&
					!s_M[o].Run)
				{
					s_M[o].SlaveLate++;
				}
			}
		}
		return *this;
	}
	if (!on && m.Run)
	{
		Bus();
		m.Run = false;
	}
	// The configuration changes with MAINENABLE off
	assert(!m.Run || (v & ~I2S_CFG1_MAINENABLE_MASK) == (m.Cfg1 & ~I2S_CFG1_MAINENABLE_MASK));
	m.Cfg1 = v;
	return *this;
}
PlainR::operator uint32_t() const { return s_M[RegNo(this)].*f; }
PlainR &PlainR::operator=(uint32_t v)
{
	I2sM &m = s_M[RegNo(this)];
	assert(!m.Run || f == &I2sM::FifoTrig);
	m.*f = v;
	return *this;
}
FifoCfgR::operator uint32_t() const { return s_M[RegNo(this)].FifoCfg; }
FifoCfgR &FifoCfgR::operator=(uint32_t v)
{
	I2sM &m = s_M[RegNo(this)];
	m.FifoCfg = v & (I2S_FIFOCFG_ENABLETX_MASK | I2S_FIFOCFG_ENABLERX_MASK | I2S_FIFOCFG_TXI2SE0_MASK |
					 I2S_FIFOCFG_DMATX_MASK | I2S_FIFOCFG_DMARX_MASK | I2S_FIFOCFG_PACK48_MASK);
	assert((v & I2S_FIFOCFG_PACK48_MASK) == 0);
	if (v & I2S_FIFOCFG_EMPTYTX_MASK)
	{
		m.Tx.clear();
	}
	if (v & I2S_FIFOCFG_EMPTYRX_MASK)
	{
		m.Rx.clear();
	}
	return *this;
}
static uint32_t FifoStat(int Fc)
{
	const I2sM &m = s_M[Fc];
	return (m.TxErr ? I2S_FIFOSTAT_TXERR_MASK : 0U) | (m.RxErr ? I2S_FIFOSTAT_RXERR_MASK : 0U) |
		   (m.Tx.empty() ? I2S_FIFOSTAT_TXEMPTY_MASK : 0U) | (m.Tx.size() < FIFO_DEPTH ? I2S_FIFOSTAT_TXNOTFULL_MASK : 0U) |
		   (!m.Rx.empty() ? I2S_FIFOSTAT_RXNOTEMPTY_MASK : 0U) | (m.Rx.size() >= FIFO_DEPTH ? I2S_FIFOSTAT_RXFULL_MASK : 0U) |
		   I2S_FIFOSTAT_TXLVL(m.Tx.size()) | I2S_FIFOSTAT_RXLVL(m.Rx.size());
}
FifoStatR::operator uint32_t() const
{
	int fc = RegNo(this);
	Tick(0.2);
	return FifoStat(fc);
}
FifoStatR &FifoStatR::operator=(uint32_t v)
{
	I2sM &m = s_M[RegNo(this)];
	if (v & I2S_FIFOSTAT_TXERR_MASK)
	{
		m.TxErr = false;
	}
	if (v & I2S_FIFOSTAT_RXERR_MASK)
	{
		m.RxErr = false;
	}
	return *this;
}
FifoIntSetR::operator uint32_t() const { return s_M[RegNo(this)].IntEn; }
FifoIntSetR &FifoIntSetR::operator=(uint32_t v) { s_M[RegNo(this)].IntEn |= v; return *this; }
FifoIntClrR &FifoIntClrR::operator=(uint32_t v) { s_M[RegNo(this)].IntEn &= ~v; return *this; }
static uint32_t IntStat(int Fc)
{
	const I2sM &m = s_M[Fc];
	uint32_t s = (m.TxErr ? I2S_FIFOINTSTAT_TXERR_MASK : 0U) | (m.RxErr ? I2S_FIFOINTSTAT_RXERR_MASK : 0U);
	uint32_t trig = m.FifoTrig;

	if ((trig & I2S_FIFOTRIG_TXLVLENA_MASK) && m.Tx.size() <= ((trig & I2S_FIFOTRIG_TXLVL_MASK) >> I2S_FIFOTRIG_TXLVL_SHIFT))
	{
		s |= I2S_FIFOINTSTAT_TXLVL_MASK;
	}
	if ((trig & I2S_FIFOTRIG_RXLVLENA_MASK) && m.Rx.size() > ((trig & I2S_FIFOTRIG_RXLVL_MASK) >> I2S_FIFOTRIG_RXLVL_SHIFT))
	{
		s |= I2S_FIFOINTSTAT_RXLVL_MASK;
	}
	return s & m.IntEn;
}
FifoIntStatR::operator uint32_t() const { return IntStat(RegNo(this)); }
static void Push(int Fc, uint32_t v)
{
	I2sM &m = s_M[Fc];
	assert(IsTx(Fc) && (m.FifoCfg & I2S_FIFOCFG_ENABLETX_MASK));
	assert(m.Tx.size() < FIFO_DEPTH);
	m.Tx.push_back(v);
}
static uint32_t Pop(int Fc)
{
	I2sM &m = s_M[Fc];
	assert(IsRx(Fc) && !m.Rx.empty());
	uint32_t v = m.Rx.front();
	m.Rx.pop_front();
	return v;
}
FifoWrR &FifoWrR::operator=(uint32_t v) { Push(RegNo(this), v); return *this; }
FifoRdR::operator uint32_t() const { return Pop(RegNo(this)); }

#undef SYSCON
#undef FLEXCOMM0
#undef FLEXCOMM1
#undef FLEXCOMM2
#undef FLEXCOMM3
#undef FLEXCOMM4
#undef FLEXCOMM5
#undef FLEXCOMM6
#undef FLEXCOMM7
#undef FLEXCOMM8
#undef FLEXCOMM9
#undef I2S0
#undef I2S1
#undef DMA0
#define SYSCON ((SYSCON_Type *)s_SysconMem)
#define FLEXCOMM_Type FcRegs
#define FLEXCOMM0 (&s_Fc[0])
#define FLEXCOMM1 (&s_Fc[1])
#define FLEXCOMM2 (&s_Fc[2])
#define FLEXCOMM3 (&s_Fc[3])
#define FLEXCOMM4 (&s_Fc[4])
#define FLEXCOMM5 (&s_Fc[5])
#define FLEXCOMM6 (&s_Fc[6])
#define FLEXCOMM7 (&s_Fc[7])
#define FLEXCOMM8 (&s_Fc[8])
#define FLEXCOMM9 (&s_Fc[9])
#define I2S_Type I2sRegs
#define I2S0 (&s_I2sRegs[0])
#define I2S1 (&s_I2sRegs[1])
#define DMA_Type DmaRegs
#define DMA0 (&s_DmaRegs)
#define LPC546XX_I2S_DATA_POLLS 20000U

#include "dma_lpc546xx.h"

static bool DmaReqOn(int Ch)
{
	for (int fc = 6; fc <= 7; fc++)
	{
		const I2sM &m = s_M[fc];
		if (Ch == LPC546XX_DMA_CH_FC_TX(fc))
		{
			return IsTx(fc) && (m.FifoCfg & I2S_FIFOCFG_DMATX_MASK) && m.Tx.size() < FIFO_DEPTH;
		}
		if (Ch == LPC546XX_DMA_CH_FC_RX(fc))
		{
			return IsRx(fc) && (m.FifoCfg & I2S_FIFOCFG_DMARX_MASK) && !m.Rx.empty();
		}
	}
	return false;
}

static bool DmaPeriphRead(uint32_t Addr, uint32_t *pVal)
{
	for (int i = 0; i < 2; i++)
	{
		if (Addr == (uint32_t)(uintptr_t)&s_I2sRegs[i].FIFORD)
		{
			*pVal = Pop(6 + i);
			return true;
		}
	}
	return false;
}

static bool DmaPeriphWrite(uint32_t Addr, uint32_t Val)
{
	for (int i = 0; i < 2; i++)
	{
		if (Addr == (uint32_t)(uintptr_t)&s_I2sRegs[i].FIFOWR)
		{
			Push(6 + i, Val);
			return true;
		}
	}
	return false;
}

#include "coredev/iopincfg.h"

static int s_PinCfg = 0;

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)PortNo; (void)PinNo; (void)PinOp; (void)Dir; (void)Resistor; (void)Type;
	s_PinCfg++;
}
void IOPinDisable(int PortNo, int PinNo) { (void)PortNo; (void)PinNo; }

#include "../../../src/coredev/shared_intrf.cpp"
#include "../../../ARM/NXP/LPC546xx/src/flexcomm_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/dma_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/i2s_lpc546xx.cpp"

static void (* const s_FcVector[2])(void) = { FLEXCOMM6_IRQHandler, FLEXCOMM7_IRQHandler };

static bool FcIrqPending(int Fc)
{
	return (s_NvicEn & (1UL << (Lpc546xxFlexcommIrqNo(Fc) & 31))) && IntStat(Fc) != 0;
}

static int s_InTick = 0;
static int s_IrqCalls = 0;

static void Tick(double Us)
{
	s_Us += Us;
	Bus();
	if (s_InTick)
	{
		// The DMA keeps moving while an interrupt runs
		while (DmaStep(false) > 0);
		return;
	}
	s_InTick = 1;
	for (int r = 0; r < 4; r++)
	{
		while (DmaStep(false) > 0);
		if (g_IrqMasked == 0)
		{
			if (DmaIrqPending())
			{
				DMA0_IRQHandler();
			}
			for (int fc = 6; fc <= 7; fc++)
			{
				if (g_IrqMasked == 0 && FcIrqPending(fc))
				{
					s_IrqCalls++;
					s_FcVector[fc - 6]();
				}
			}
		}
	}
	s_InTick = 0;
}

static void Run(double Us)
{
	for (double t = 0; t < Us; t += 1.0)
	{
		Tick(1.0);
	}
}

// ---- Tests ----

// LPCXpresso54628: Flexcomm 6 to the codec, Flexcomm 7 from it, MCLK P3_11
static const IOPinCfg_t s_Duplex[] = {
	{4, 1, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// FC6 SCK
	{4, 3, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// FC6 WS
	{2, 19, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// FC7 DATA
	{4, 2, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// FC6 DATA
	{3, 11, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// MCLK
	{2, 18, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// FC7 SCK
	{2, 20, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// FC7 WS
};
static const IOPinCfg_t s_TxOnly[] = {
	{4, 1, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{4, 3, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{4, 2, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};
static const IOPinCfg_t s_RxOnly[] = {
	{2, 18, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{2, 20, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{2, 19, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_RxMem[CFIFO_TOTAL_MEMSIZE(512, 4)];
alignas(4) static uint8_t s_TxMem[CFIFO_TOTAL_MEMSIZE(512, 4)];

static int s_Evt[16];
static int s_LastEvtCount;
static std::vector<int> s_EvtLog;
static void (*s_EvtHook)(DEVINTRF_EVT Evt) = nullptr;

static int AudioEvt(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Count)
{
	(void)pDev;
	(void)pData;
	s_Evt[Evt & 15]++;
	s_LastEvtCount = Count;
	s_EvtLog.push_back(Evt);
	if (s_EvtHook)
	{
		s_EvtHook(Evt);
	}
	return 0;
}

static I2SCfg_t Cfg(int DevNo, const IOPinCfg_t *pPins, int NbPins, bool bInt, bool bDma, bool bRxMem, bool bTxMem)
{
	I2SCfg_t c;

	memset(&c, 0, sizeof(c));
	c.DevNo = DevNo;
	c.pIOPinMap = pPins;
	c.NbIOPins = NbPins;
	c.Mode = I2SMODE_MASTER;
	c.Format = I2SFMT_NORMAL;
	c.SampleWidth = I2SSWIDTH_16BITS;
	c.Chan = I2SCHAN_STEREO;
	c.Freq = 48000;
	c.MaxRetry = 0;
	c.bDmaEn = bDma;
	c.bIntEn = bInt;
	c.IntPrio = 6;
	c.EvtCB = AudioEvt;
	c.pRxFifoMem = bRxMem ? s_RxMem : NULL;
	c.RxFifoMemSize = bRxMem ? sizeof(s_RxMem) : 0;
	c.pTxFifoMem = bTxMem ? s_TxMem : NULL;
	c.TxFifoMemSize = bTxMem ? sizeof(s_TxMem) : 0;
	return c;
}

static void Fresh(void)
{
	for (int i = 0; i < 10; i++)
	{
		s_M[i] = I2sM();
		s_Fc[i].PSELID.value = (i == 6 || i == 7) ? 0xF0U : 0x70U;
	}
	memset(s_SysconMem, 0, sizeof(s_SysconMem));
	DmaModelInit();
	s_DmaM.Immediate = true;
	s_NvicEn = 0;
	memset(s_NvicPrio, 0, sizeof(s_NvicPrio));
	s_PinCfg = 0;
	s_IrqCalls = 0;
	s_EvtLog.clear();
	s_EvtHook = nullptr;
	s_ExtRate = 0;
	s_Loopback = false;
	s_CodecRx.clear();
	s_PllFail = false;
	s_PllOffset = 0;
	memset(s_Evt, 0, sizeof(s_Evt));
	g_IrqMasked = 0;
	s_Us += 1000;
}

static SYSCON_Type *Sys() { return (SYSCON_Type *)s_SysconMem; }

// The TX frames on the bus from the first one equal to First
static size_t Find(const std::vector<uint32_t> &v, uint32_t First, size_t From = 0)
{
	for (size_t i = From; i < v.size(); i++)
	{
		if (v[i] == First)
		{
			return i;
		}
	}
	return v.size();
}

static uint32_t Stereo(uint32_t n)
{
	return (uint32_t)(uint16_t)(n * 7U + 0x101U) | ((uint32_t)(uint16_t)(n * 11U + 0x8001U) << 16);
}

static void TestInitChecks()
{
	I2S i2s;
	I2SCfg_t c;

	Fresh();
	c = Cfg(5, s_Duplex, 7, false, false, false, false);
	assert(!i2s.Init(c));
	c = Cfg(8, s_Duplex, 7, false, false, false, false);
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 7, false, false, false, false);
	c.Format = I2SFMT_ALIGN_LEFT;
	assert(!i2s.Init(c));
	c.Format = I2SFMT_NORMAL;
	c.SampleWidth = I2SSWIDTH_24BITS;
	assert(!i2s.Init(c));
	c.SampleWidth = I2SSWIDTH_8BITS;
	assert(!i2s.Init(c));
	c.SampleWidth = I2SSWIDTH_16BITS;
	c.Chan = (I2SCHAN)3;
	assert(!i2s.Init(c));
	c.Chan = I2SCHAN_STEREO;
	c.IntPrio = 8;
	assert(!i2s.Init(c));
	c.IntPrio = -1;
	assert(!i2s.Init(c));
	// Pins: too few, too many, no data pin, no SCK or WS, duplex without the
	// receive clocks
	c = Cfg(6, s_Duplex, 2, false, false, false, false);
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 8, false, false, false, false);
	assert(!i2s.Init(c));
	IOPinCfg_t p[7];
	memcpy(p, s_Duplex, sizeof(p));
	p[2].PortNo = -1;
	p[3].PortNo = -1;
	c = Cfg(6, p, 7, false, false, false, false);
	assert(!i2s.Init(c));
	memcpy(p, s_Duplex, sizeof(p));
	p[0].PortNo = -1;
	c = Cfg(6, p, 7, false, false, false, false);
	assert(!i2s.Init(c));
	memcpy(p, s_Duplex, sizeof(p));
	p[1].PortNo = -1;
	c = Cfg(6, p, 7, false, false, false, false);
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 6, false, false, false, false);
	assert(!i2s.Init(c));
	memcpy(p, s_Duplex, sizeof(p));
	p[6].PortNo = -1;
	c = Cfg(6, p, 7, false, false, false, false);
	assert(!i2s.Init(c));
	// FIFO memory: needed with interrupts or DMA, aligned, a frame or more,
	// for a direction used, not overlapping
	c = Cfg(6, s_Duplex, 7, true, false, false, true);
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 7, true, false, true, false);
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 7, false, true, true, false);
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 7, true, false, true, true);
	c.pRxFifoMem = s_RxMem + 1;
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 7, true, false, true, true);
	c.pTxFifoMem = s_TxMem + 2;
	c.TxFifoMemSize = 64;
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 7, true, false, true, true);
	c.RxFifoMemSize = sizeof(CFifo_t) + 3;
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 7, true, false, true, true);
	c.TxFifoMemSize = sizeof(CFifo_t) + 3;
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 7, true, false, true, true);
	c.pTxFifoMem = s_RxMem + 4;
	c.TxFifoMemSize = 64;
	assert(!i2s.Init(c));
	c = Cfg(6, s_TxOnly, 4, false, false, true, false);
	assert(!i2s.Init(c));
	c = Cfg(7, s_RxOnly, 3, false, false, false, true);
	assert(!i2s.Init(c));
	// RX memory inside the TX memory, duplex without the receive SCK, no pin
	c = Cfg(6, s_Duplex, 7, true, false, true, true);
	c.pRxFifoMem = s_TxMem + 4;
	c.RxFifoMemSize = 64;
	assert(!i2s.Init(c));
	memcpy(p, s_Duplex, sizeof(p));
	p[5].PortNo = -1;
	c = Cfg(6, p, 7, false, false, false, false);
	assert(!i2s.Init(c));
	c = Cfg(6, s_Duplex, 0, false, false, false, false);
	assert(!i2s.Init(c));
	c.NbIOPins = -1;
	assert(!i2s.Init(c));
	// Receive DMA: the FIFO memory holds a block of 32 frames
	c = Cfg(7, s_RxOnly, 3, true, true, true, false);
	c.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(31, 4);
	assert(!i2s.Init(c));
	c.Chan = I2SCHAN_LEFT;
	c.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(31, 2);
	assert(!i2s.Init(c));
	assert(s_PllStarts == 0 && s_PinCfg == 0);
	c.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(32, 2);
	assert(i2s.Init(c));
	i2s.PowerOff();
	s_PllStarts = 0;
	// Flexcomm 6 or 7 only, even with an I2S function present
	Fresh();
	s_Fc[5].PSELID.value = 0xF0U;
	c = Cfg(5, s_TxOnly, 4, false, false, false, false);
	assert(!i2s.Init(c) && s_PllStarts == 0 && Persel(5) == 0U);
	// Duplex with Flexcomm 7 not in I2S. SYSCON is memory here: the last
	// clock turned off is seen, Flexcomm 7.
	Fresh();
	s_Fc[7].PSELID.value = 0x70U;
	c = Cfg(6, s_Duplex, 7, false, false, false, false);
	assert(!i2s.Init(c) && Sys()->AHBCLKCTRLCLR[1] == SYSCON_AHBCLKCTRL_FLEXCOMM7_MASK);
	// Not an I2S Flexcomm
	Fresh();
	s_Fc[6].PSELID.value = 0x70U;
	c = Cfg(6, s_TxOnly, 4, false, false, false, false);
	assert(!i2s.Init(c));
	assert((Sys()->AHBCLKCTRL[1] & SYSCON_AHBCLKCTRL_FLEXCOMM6_MASK) == 0);
	// The PLL fails: all released
	Fresh();
	s_PllFail = true;
	c = Cfg(6, s_Duplex, 7, true, true, true, true);
	assert(!i2s.Init(c));
	assert(s_PllFreq == 0 && (s_NvicEn & (1UL << FLEXCOMM6_IRQn)) == 0 && g_SharedIntrf[6].pIntrf == NULL);
	assert((Sys()->FCLKSEL[6] & 7U) == 7U && (Sys()->FCLKSEL[7] & 7U) == 7U);
	assert(Lpc546xxDmaChClaim(LPC546XX_DMA_CH_FC_TX(6), NULL, NULL, 0, false));
	Lpc546xxDmaChFree(LPC546XX_DMA_CH_FC_TX(6));
	// A DMA channel already taken
	Fresh();
	assert(Lpc546xxDmaChClaim(LPC546XX_DMA_CH_FC_TX(6), NULL, NULL, 0, false));
	c = Cfg(6, s_Duplex, 7, true, true, true, true);
	assert(!i2s.Init(c));
	assert(Lpc546xxDmaChClaim(LPC546XX_DMA_CH_FC_RX(7), NULL, NULL, 0, false));
	Lpc546xxDmaChFree(LPC546XX_DMA_CH_FC_RX(7));
	Lpc546xxDmaChFree(LPC546XX_DMA_CH_FC_TX(6));
	// The receive channel taken
	Fresh();
	assert(Lpc546xxDmaChClaim(LPC546XX_DMA_CH_FC_RX(7), NULL, NULL, 0, false));
	assert(!i2s.Init(c));
	assert(Lpc546xxDmaChClaim(LPC546XX_DMA_CH_FC_TX(6), NULL, NULL, 0, false));
	Lpc546xxDmaChFree(LPC546XX_DMA_CH_FC_TX(6));
	Lpc546xxDmaChFree(LPC546XX_DMA_CH_FC_RX(7));
	assert(g_SharedIntrf[6].pIntrf == NULL && g_SharedIntrf[7].pIntrf == NULL);
	assert(s_PllStarts == 1);
	s_PllStarts = 0;

	// One device at a time
	Fresh();
	I2S other;
	c = Cfg(6, s_TxOnly, 4, false, false, false, false);
	assert(i2s.Init(c));
	c = Cfg(7, s_RxOnly, 3, false, false, false, false);
	assert(!other.Init(c));
	i2s.PowerOff();
	assert(other.Init(c));
	other.PowerOff();
	printf("init checks: PASS\n");
}

static void TestRates()
{
	static const struct { uint32_t Req, Pll, K, Rate; } r[] = {
		{ 48000, 12288000, 1, 48000 }, { 44100, 22579200, 2, 44100 }, { 32000, 16384000, 2, 32000 },
		{ 16000, 12288000, 3, 16000 }, { 8000, 12288000, 6, 8000 }, { 96000, 24576000, 1, 96000 },
		{ 0, 12288000, 3, 16000 }, { 15625, 12000000, 3, 15625 }, { 100, 6553600, 256, 100 },
		{ 10, 655360, 256, 10 },
	};

	for (auto &e : r)
	{
		I2S i2s;

		Fresh();
		I2SCfg_t c = Cfg(6, s_Duplex, 7, false, false, false, false);
		c.Freq = e.Req;
		assert(i2s.Init(c));
		if (!(s_PllAsk == e.Pll && i2s.Rate() == e.Rate)) fprintf(stderr, "req %u ask %u rate %u\n", e.Req, s_PllAsk, i2s.Rate());
		assert(s_PllAsk == e.Pll && i2s.Rate() == e.Rate);
		assert(s_M[6].Div == 8 * e.K - 1 && ((I2SDev_t *)i2s)->MClkFreq == e.Pll / e.K);
		assert(Sys()->MCLKCLKSEL == 1 && Sys()->MCLKDIV == e.K - 1 && Sys()->MCLKIO == 1);
		assert((Sys()->FCLKSEL[6] & 7U) == 2U && (Sys()->FCLKSEL[7] & 7U) == 2U);
		i2s.PowerOff();
	}

	// The PLL a bit off: the rate it makes
	I2S i2s;
	Fresh();
	s_PllOffset = 256;
	I2SCfg_t c = Cfg(6, s_Duplex, 7, false, false, false, false);
	assert(i2s.Init(c) && i2s.Rate() == 48001);
	// Rate changes while running
	Run(2000);
	assert(i2s.Rate(44100) == 44100 && s_M[6].Div == 15 && s_M[6].Run && s_M[7].Run);
	assert(i2s.Rate(0) == 16000 && s_M[6].Div == 23);
	s_PllFail = true;
	assert(i2s.Rate(48000) == 0 && !s_M[6].Run);
	i2s.PowerOff();
	// Without the MCLK pin, slave mode
	Fresh();
	c = Cfg(6, s_TxOnly, 4, false, false, false, false);
	Sys()->MCLKIO = 0;
	Sys()->MCLKCLKSEL = 7;
	assert(i2s.Init(c) && Sys()->MCLKCLKSEL == 7 && Sys()->MCLKIO == 0 && ((I2SDev_t *)i2s)->MClkFreq == 12288000);
	i2s.PowerOff();
	Fresh();
	c = Cfg(6, s_Duplex, 7, false, false, false, false);
	c.Mode = I2SMODE_SLAVE;
	assert(i2s.Init(c) && ((I2SDev_t *)i2s)->MClkFreq == 0 && !IsMaster(6) && !IsMaster(7));
	assert(Sys()->MCLKIO == 1);
	i2s.PowerOff();
	// Rates the 32 bit products do not hold: the PLL asked its most
	Fresh();
	c = Cfg(6, s_TxOnly, 4, false, false, false, false);
	c.Freq = 1U << 24;
	assert(i2s.Init(c) && s_PllAsk == 0xFFFFFFFFU && i2s.Rate() == 0xFFFFFFFFU / 256U);
	assert(i2s.Rate(0xFFFFFFFFU) == 0xFFFFFFFFU / 256U && s_M[6].Div == 7);
	i2s.PowerOff();
	printf("rates: PASS\n");
}

// Stereo or mono frames sent, in each mode
static void TestTx()
{
	static const struct { bool Int, Dma, Mem; } m[] = {
		{ false, false, false }, { true, false, true }, { false, false, true }, { true, true, true }, { false, true, true },
	};
	static const I2SCHAN chans[] = { I2SCHAN_STEREO, I2SCHAN_LEFT, I2SCHAN_RIGHT };

	for (auto &e : m)
	{
		for (I2SCHAN ch : chans)
		{
			I2S i2s;

			Fresh();
			I2SCfg_t c = Cfg(6, s_TxOnly, 4, e.Int, e.Dma, false, e.Mem);
			c.Chan = ch;
			assert(i2s.Init(c));
			assert(IsTx(6) && IsMaster(6) && s_M[6].Run && !s_M[7].Run);
			unsigned fb = ch == I2SCHAN_STEREO ? 4 : 2;
			uint8_t buf[256 * 4];
			for (uint32_t n = 0; n < 256; n++)
			{
				uint32_t v = Stereo(n);
				if (fb == 4)
				{
					memcpy(buf + n * 4, &v, 4);
				}
				else
				{
					uint16_t s = (uint16_t)v;
					memcpy(buf + n * 2, &s, 2);
				}
			}
			// A frame cut is refused
			assert(i2s.Tx(0, buf, (int)fb + 1) == 0);
			// A time without frames: silence
			Run(1000);
			unsigned sent = 0;
			for (int t = 0; t < 20000 && sent < 256 * fb; t++)
			{
				int n = i2s.Tx(0, buf + sent, 256 * fb - sent);
				assert(n >= 0 && n % (int)fb == 0);
				sent += n;
				Run(20);
			}
			assert(sent == 256 * fb);
			// Silence after the frames: without interrupt the frames move
			// when the application calls
			static uint8_t zero[64 * 4];
			for (int t = 0; t < 300; t++)
			{
				if (!e.Int)
				{
					i2s.Tx(0, zero, 64 * fb);
				}
				Run(50);
			}
			// On the bus in order, after the silence of the start
			uint32_t first = fb == 4 ? Stereo(0) : (ch == I2SCHAN_LEFT ? (uint16_t)Stereo(0) : (uint32_t)(uint16_t)Stereo(0) << 16);
			const std::vector<uint32_t> &b = s_M[6].Sent;
			size_t at = Find(b, first);
			if (!(at < b.size() && b.size() - at >= 256))
			{
				fprintf(stderr, "int %d dma %d mem %d ch %d: sent %zu at %zu under %d\n", e.Int, e.Dma, e.Mem, ch, b.size(), at, s_M[6].Underruns);
				for (size_t i = 0; i < b.size() && i < 40; i++) fprintf(stderr, " %08x", b[i]);
				fprintf(stderr, "\n");
			}
			assert(at < b.size() && b.size() - at >= 256);
			for (size_t i = 0; i < at; i++)
			{
				assert(b[i] == 0);
			}
			for (uint32_t n = 0; n < 256; n++)
			{
				uint32_t v = Stereo(n);
				uint32_t want = fb == 4 ? v : (ch == I2SCHAN_LEFT ? (v & 0xFFFFU) : (v << 16));
				assert(b[at + n] == want);
			}
			// Silence after the frames
			for (size_t i = at + 256; i < b.size(); i++)
			{
				assert(b[i] == 0);
			}
			if (e.Mem && (e.Int || e.Dma))
			{
				assert(s_Evt[DEVINTRF_EVT_TX_READY] > 0);
			}
			if (e.Dma && e.Int)
			{
				assert(s_Evt[DEVINTRF_EVT_TX_FIFO_EMPTY] > 0);
			}
			if (!e.Int)
			{
				// Without interrupt: the FIFO ran empty between the calls, no
				// failure
				assert(s_M[6].Underruns > 0 && s_Evt[DEVINTRF_EVT_STATECHG] == 0);
			}
			if (e.Int && !e.Dma)
			{
				assert(s_M[6].Underruns == 0);
			}
			// No frame to move: no event
			int ready = s_Evt[DEVINTRF_EVT_TX_READY], empty = s_Evt[DEVINTRF_EVT_TX_FIFO_EMPTY];
			Run(3000);
			assert(s_Evt[DEVINTRF_EVT_TX_READY] == ready && s_Evt[DEVINTRF_EVT_TX_FIFO_EMPTY] == empty);
			assert(s_Evt[DEVINTRF_EVT_RX_DATA] == 0);
			// No Flexcomm interrupt with the DMA
			if (e.Dma)
			{
				assert(s_IrqCalls == 0);
			}
			// Pins, interrupt priority, the interrupt on with bIntEn only, TX
			// interrupt or DMA at 4 entries or less
			assert(s_PinCfg == 3 && s_NvicPrio[FLEXCOMM6_IRQn] == 6);
			assert(((s_NvicEn >> (FLEXCOMM6_IRQn & 31)) & 1U) == (e.Int ? 1U : 0U));
			assert(s_M[6].FifoTrig == (I2S_FIFOTRIG_TXLVLENA_MASK | I2S_FIFOTRIG_TXLVL(4)));
			// Receive on a transmit device: refused, the stream goes on
			assert(i2s.Rx(0, buf, 4) == 0 && s_M[6].Run && s_Evt[DEVINTRF_EVT_RX_TIMEOUT] == 0);
			i2s.PowerOff();
			assert((Sys()->FCLKSEL[6] & 7U) == 7U && !s_M[6].Run && s_PllFreq == 0);
		}
	}
	printf("transmit: PASS\n");
}

// Frames received in order from the first one taken, in each mode
static void TestRx()
{
	static const struct { bool Int, Dma, Mem; } m[] = {
		{ false, false, false }, { true, false, true }, { false, false, true }, { true, true, true }, { false, true, true },
	};
	static const I2SCHAN chans[] = { I2SCHAN_STEREO, I2SCHAN_LEFT, I2SCHAN_RIGHT };

	for (auto &e : m)
	{
		for (I2SCHAN ch : chans)
		{
			I2S i2s;

			Fresh();
			I2SCfg_t c = Cfg(7, s_RxOnly, 3, e.Int, e.Dma, e.Mem, false);
			c.Chan = ch;
			assert(i2s.Init(c));
			assert(IsRx(7) && IsMaster(7) && s_M[7].Run && !s_M[6].Run);
			unsigned fb = ch == I2SCHAN_STEREO ? 4 : 2;
			uint8_t buf[600 * 4];
			unsigned got = 0;
			// A frame cut is refused
			assert(i2s.Rx(0, buf, (int)fb + 1) == 0);
			// About 5 frames between the calls
			for (int t = 0; t < 20000 && got < 600 * fb; t++)
			{
				int n = i2s.Rx(0, buf + got, (int)(600 * fb - got));
				assert(n >= 0 && n % (int)fb == 0);
				got += n;
				if (e.Mem)
				{
					Run(100);
				}
			}
			assert(got == 600 * fb);
			// Consecutive frames of the pattern
			uint32_t s0;
			uint16_t h;
			if (fb == 4)
			{
				memcpy(&s0, buf, 4);
			}
			else
			{
				memcpy(&h, buf, 2);
				s0 = h;
			}
			uint64_t base = 0;
			for (base = 0; base < s_Frame; base++)
			{
				uint32_t p = Pattern(base);
				uint32_t want = fb == 4 ? p : (ch == I2SCHAN_LEFT ? (p & 0xFFFFU) : (p >> 16));
				if (want == s0)
				{
					break;
				}
			}
			assert(base < s_Frame);
			for (uint32_t n = 0; n < 600; n++)
			{
				uint32_t p = Pattern(base + n);
				if (fb == 4)
				{
					uint32_t v;
					memcpy(&v, buf + n * 4, 4);
					assert(v == p);
				}
				else
				{
					memcpy(&h, buf + n * 2, 2);
					assert(h == (ch == I2SCHAN_LEFT ? (uint16_t)p : (uint16_t)(p >> 16)));
				}
			}
			if (e.Mem && (e.Int || e.Dma))
			{
				assert(s_Evt[DEVINTRF_EVT_RX_DATA] > 0 && s_M[7].Overruns == 0);
			}
			assert(s_Evt[DEVINTRF_EVT_STATECHG] == 0 && s_Evt[DEVINTRF_EVT_RX_FIFO_FULL] == 0);
			assert(s_M[7].FifoTrig == (I2S_FIFOTRIG_RXLVLENA_MASK | I2S_FIFOTRIG_RXLVL(3)));
			if (e.Dma)
			{
				assert(s_IrqCalls == 0);
			}
			assert(s_PinCfg == 3 && s_NvicPrio[FLEXCOMM7_IRQn] == 6);
			assert(i2s.Tx(0, buf, 4) == 0 && s_M[7].Run);
			// No call for 2 ms. Without interrupt or DMA, frames are lost
			// and the stream goes on. Without interrupt, the DMA stops at
			// the end of its block and the receive FIFO overruns: the stream
			// stops.
			if (e.Mem && !e.Int)
			{
				Run(2000);
				if (e.Dma)
				{
					assert(i2s.Rx(0, buf, 4 * (int)fb) == 0 && s_Evt[DEVINTRF_EVT_STATECHG] == 1 && !s_M[7].Run);
				}
				else
				{
					int n = 0;
					for (int t = 0; t < 10 && n == 0; t++)
					{
						n = i2s.Rx(0, buf, 4 * (int)fb);
						Run(100);
					}
					assert(n > 0 && s_Evt[DEVINTRF_EVT_STATECHG] == 0 && s_M[7].Run && s_M[7].Overruns > 0);
				}
			}
			i2s.PowerOff();
		}
	}
	printf("receive: PASS\n");
}

// Flexcomm 6 sends, Flexcomm 7 receives the codec loopback
static void TestDuplex()
{
	static const struct { bool Int, Dma; I2SMODE Mode; int Dev; } m[] = {
		{ true, false, I2SMODE_MASTER, 6 }, { true, true, I2SMODE_MASTER, 6 }, { false, true, I2SMODE_MASTER, 6 },
		{ true, false, I2SMODE_SLAVE, 6 }, { true, true, I2SMODE_SLAVE, 6 }, { true, true, I2SMODE_MASTER, 7 },
	};

	for (auto &e : m)
	{
		I2S i2s;

		Fresh();
		s_Loopback = true;
		I2SCfg_t c = Cfg(e.Dev, s_Duplex, 7, e.Int, e.Dma, true, true);
		c.Mode = e.Mode;
		assert(i2s.Init(c));
		if (e.Mode == I2SMODE_SLAVE)
		{
			s_ExtRate = 48000;
			s_ExtStartUs = s_Us;
		}
		int tx = e.Dev, rx = 13 - e.Dev;
		assert(IsTx(tx) && IsRx(rx) && IsMaster(tx) == (e.Mode == I2SMODE_MASTER) && !IsMaster(rx));
		assert(s_M[6].Run && s_M[7].Run && s_M[7].SlaveLate == 0 && s_M[6].SlaveLate == 0);
		assert(s_PinCfg == 7 && s_NvicPrio[FLEXCOMM6_IRQn] == 6 && s_NvicPrio[FLEXCOMM7_IRQn] == 6);
		// Frames sent come back, after the silence of the start
		static uint8_t out[1000 * 4], in[1600 * 4];
		for (uint32_t n = 0; n < 1000; n++)
		{
			uint32_t v = Stereo(n);
			memcpy(out + n * 4, &v, 4);
		}
		unsigned sent = 0, got = 0;
		for (int t = 0; t < 40000 && got < sizeof(in); t++)
		{
			if (sent < sizeof(out))
			{
				sent += i2s.Tx(0, out + sent, sizeof(out) - sent);
			}
			else
			{
				static uint8_t zero[64 * 4];
				i2s.Tx(0, zero, sizeof(zero));
			}
			got += i2s.Rx(0, in + got, sizeof(in) - got);
			Run(20);
		}
		assert(got == sizeof(in));
		unsigned at = 0;
		while (at < got && memcmp(in + at, out, 4) != 0)
		{
			assert(memcmp(in + at, "\0\0\0\0", 4) == 0);
			at += 4;
		}
		assert(at + sizeof(out) <= got && memcmp(in + at, out, sizeof(out)) == 0);
		assert(s_Evt[DEVINTRF_EVT_STATECHG] == 0);
		i2s.PowerOff();
		assert(!s_M[6].Run && !s_M[7].Run && (Sys()->FCLKSEL[7] & 7U) == 7U && Sys()->MCLKIO == 0);
		assert(Lpc546xxDmaChClaim(LPC546XX_DMA_CH_FC_RX(rx), NULL, NULL, 0, false));
		Lpc546xxDmaChFree(LPC546XX_DMA_CH_FC_RX(rx));
	}
	printf("full duplex: PASS\n");
}

static void TestFail()
{
	I2S i2s;
	uint8_t buf[64];

	// Receive FIFO overrun with the interrupts held off
	for (int dma = 0; dma < 2; dma++)
	{
		Fresh();
		I2SCfg_t c = Cfg(6, s_Duplex, 7, true, dma != 0, true, true);
		assert(i2s.Init(c));
		Run(500);
		g_IrqMasked = 1;
		Run(dma ? 2000 : 400);
		g_IrqMasked = 0;
		Run(100);
		assert(s_Evt[DEVINTRF_EVT_STATECHG] == 1 && !s_M[6].Run && !s_M[7].Run);
		assert(i2s.Rx(0, buf, 4) == 0 && i2s.Tx(0, buf, 4) == 0);
		// Disable then Enable restarts
		i2s.Disable();
		i2s.Enable();
		assert(s_M[6].Run && s_M[7].Run);
		Run(2000);
		assert(i2s.Rx(0, buf, 64) == 64 && s_Evt[DEVINTRF_EVT_STATECHG] == 1);
		// Again, then Reset restarts
		g_IrqMasked = 1;
		Run(dma ? 2000 : 400);
		g_IrqMasked = 0;
		Run(100);
		assert(s_Evt[DEVINTRF_EVT_STATECHG] == 2 && !s_M[6].Run && !s_M[7].Run);
		i2s.Reset();
		assert(s_M[6].Run && s_M[7].Run);
		Run(2000);
		assert(i2s.Rx(0, buf, 64) == 64 && s_Evt[DEVINTRF_EVT_STATECHG] == 2);
		i2s.PowerOff();
	}

	// Receive CFIFO full while sending: nothing moves after the failure
	I2SCfg_t c;
	for (int dma = 0; dma < 2; dma++)
	{
		Fresh();
		c = Cfg(6, s_Duplex, 7, true, dma != 0, true, true);
		c.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(dma ? 48 : 16, 4);
		assert(i2s.Init(c));
		static uint8_t frames[128 * 4];
		for (uint32_t i = 0; i < 128; i++)
		{
			uint32_t v = Stereo(i);
			memcpy(frames + i * 4, &v, 4);
		}
		assert(i2s.Tx(0, frames, sizeof(frames)) == sizeof(frames));
		Run(2000);
		assert(s_Evt[DEVINTRF_EVT_RX_FIFO_FULL] == 1 && !s_M[6].Run && !s_M[7].Run);
		assert(s_EvtLog.back() == DEVINTRF_EVT_RX_FIFO_FULL && s_M[6].Tx.empty());
		i2s.Reset();
		assert(s_M[7].Run && i2s.Rx(0, buf, 64) == 0);
		Run(dma ? 800 : 250);
		int n = i2s.Rx(0, buf, 64);
		assert(n >= 32 && n % 4 == 0 && s_Evt[DEVINTRF_EVT_RX_FIFO_FULL] == 1);
		i2s.PowerOff();
	}

	// DMA error, transmit and receive
	Fresh();
	c = Cfg(6, s_TxOnly, 4, true, true, false, true);
	assert(i2s.Init(c));
	Run(500);
	s_DmaM.ErrCh = LPC546XX_DMA_CH_FC_TX(6);
	Run(2000);
	assert(s_Evt[DEVINTRF_EVT_STATECHG] == 1 && !s_M[6].Run);
	i2s.PowerOff();
	Fresh();
	c = Cfg(7, s_RxOnly, 3, true, true, true, false);
	assert(i2s.Init(c));
	Run(500);
	s_DmaM.ErrCh = LPC546XX_DMA_CH_FC_RX(7);
	// Seen at the error, before the receive FIFO could overrun (8 frames)
	Run(100);
	assert(s_Evt[DEVINTRF_EVT_STATECHG] == 1 && !s_M[7].Run && s_M[7].Overruns == 0);
	i2s.PowerOff();

	// Polling transfers with no clock: time out
	Fresh();
	c = Cfg(7, s_RxOnly, 3, false, false, false, false);
	c.Mode = I2SMODE_SLAVE;
	assert(i2s.Init(c));
	assert(i2s.Rx(0, buf, 8) == 0 && s_Evt[DEVINTRF_EVT_RX_TIMEOUT] == 1);
	i2s.Reset();
	s_ExtRate = 48000;
	s_ExtStartUs = s_Us;
	assert(i2s.Rx(0, buf, 8) == 8);
	i2s.PowerOff();
	Fresh();
	c = Cfg(6, s_TxOnly, 4, false, false, false, false);
	c.Mode = I2SMODE_SLAVE;
	assert(i2s.Init(c));
	uint8_t big[16 * 4] = {};
	// The FIFO is full of the silence of the start: nothing goes
	assert(i2s.Tx(0, big, sizeof(big)) == 0 && s_Evt[DEVINTRF_EVT_TX_TIMEOUT] == 1);
	i2s.PowerOff();
	printf("failures: PASS\n");
}


// TX frames given, checked at each TX_FIFO_EMPTY: all of them in the hardware
// FIFO or on the bus
static size_t s_TxGiven;
static int s_EmptyBad;

static void EmptyCheck(DEVINTRF_EVT Evt)
{
	if (Evt != DEVINTRF_EVT_TX_FIFO_EMPTY)
	{
		return;
	}
	size_t n = 0;
	for (uint32_t v : s_M[6].Sent)
	{
		n += v != 0;
	}
	for (uint32_t v : s_M[6].Tx)
	{
		n += v != 0;
	}
	s_EmptyBad += n != s_TxGiven;
}

static void TestDma()
{
	I2S i2s;

	// Bursts of 40 frames, 1.25 blocks, each out before the next (up to 4
	// blocks, 2.7 ms), at times that vary which buffer ends a burst
	Fresh();
	I2SCfg_t c = Cfg(6, s_TxOnly, 4, true, true, false, true);
	assert(i2s.Init(c));
	s_TxGiven = 0;
	s_EmptyBad = 0;
	s_EvtHook = EmptyCheck;
	static const int gap[] = { 3000, 3300, 3700, 3900, 3100, 3500, 4100, 3600 };
	uint32_t k = 0;
	for (int b : gap)
	{
		uint8_t out[40 * 4];
		for (int i = 0; i < 40; i++, k++)
		{
			uint32_t v = Stereo(k);
			memcpy(out + i * 4, &v, 4);
		}
		assert(i2s.Tx(0, out, sizeof(out)) == sizeof(out));
		s_TxGiven += 40;
		Run(b);
	}
	assert(s_EmptyBad == 0 && s_Evt[DEVINTRF_EVT_TX_FIFO_EMPTY] == 8);
	// All frames in order, silence between the bursts
	const std::vector<uint32_t> &bus = s_M[6].Sent;
	size_t at = 0;
	for (uint32_t n = 0; n < k; n++)
	{
		while (at < bus.size() && bus[at] == 0)
		{
			at++;
		}
		assert(at < bus.size() && bus[at] == Stereo(n));
		at++;
	}
	s_EvtHook = nullptr;
	i2s.PowerOff();

	// Disable and Enable in the middle of a stream, with both DMA channels on
	// their second buffer: the stream starts again from the first, frames
	// before Disable are not sent again
	Fresh();
	s_Loopback = true;
	c = Cfg(6, s_Duplex, 7, true, true, true, true);
	assert(i2s.Init(c));
	static uint8_t a[300 * 4], bf[200 * 4], in[1200 * 4];
	for (uint32_t n = 0; n < 300; n++)
	{
		uint32_t v = Stereo(n);
		memcpy(a + n * 4, &v, 4);
	}
	for (uint32_t n = 0; n < 200; n++)
	{
		uint32_t v = Stereo(1000 + n);
		memcpy(bf + n * 4, &v, 4);
	}
	assert(i2s.Tx(0, a, sizeof(a)) == sizeof(a));
	uint8_t tmp[256];
	Run(2500);
	while (i2s.Rx(0, tmp, sizeof(tmp)) > 0);
	int t = 0;
	for (; t < 2000 && !(s_Lpc546xxI2s.Rx.Index == 1 && s_Lpc546xxI2s.Tx.Index == 1); t++)
	{
		Run(5);
		while (i2s.Rx(0, tmp, sizeof(tmp)) > 0);
	}
	assert(t < 2000 && s_M[6].Sent.size() < 300);
	i2s.Disable();
	// Both channels stopped, the DMA requests off
	uint32_t chs = (1UL << LPC546XX_DMA_CH_FC_TX(6)) | (1UL << LPC546XX_DMA_CH_FC_RX(7));
	assert((s_DmaM.Trig & chs) == 0 && (s_DmaRegs.COMMON[0].ACTIVE & chs) == 0);
	assert((s_M[6].FifoCfg & I2S_FIFOCFG_DMATX_MASK) == 0 && (s_M[7].FifoCfg & I2S_FIFOCFG_DMARX_MASK) == 0);
	size_t sentBefore = s_M[6].Sent.size();
	i2s.Enable();
	assert(s_M[6].Run && s_M[7].Run);
	unsigned sent = 0, got = 0;
	for (t = 0; t < 40000 && got < sizeof(in); t++)
	{
		if (sent < sizeof(bf))
		{
			sent += i2s.Tx(0, bf + sent, sizeof(bf) - sent);
		}
		got += i2s.Rx(0, in + got, sizeof(in) - got);
		Run(20);
	}
	assert(got == sizeof(in));
	// Nothing of the first stream on the bus after Enable
	for (size_t i = sentBefore; i < s_M[6].Sent.size(); i++)
	{
		uint32_t v = s_M[6].Sent[i];
		for (uint32_t n = 0; n < 300; n++)
		{
			assert(v != Stereo(n));
		}
	}
	// Silence then the second stream, received in order
	unsigned p = 0;
	while (p < got && memcmp(in + p, bf, 4) != 0)
	{
		assert(memcmp(in + p, "\0\0\0\0", 4) == 0);
		p += 4;
	}
	assert(p + sizeof(bf) <= got && memcmp(in + p, bf, sizeof(bf)) == 0);
	assert(s_Evt[DEVINTRF_EVT_STATECHG] == 0);
	i2s.PowerOff();

	// Rate change with frames queued, the transmit DMA on its second buffer:
	// the frames in flight are lost, the others follow in order, none twice
	Fresh();
	c = Cfg(6, s_TxOnly, 4, true, true, false, true);
	assert(i2s.Init(c));
	assert(i2s.Tx(0, a, sizeof(a)) == sizeof(a));
	// After about 100 frames sent
	Run(3000);
	for (t = 0; t < 2000 && !(s_Lpc546xxI2s.Tx.Index == 1 && s_Lpc546xxI2s.Tx.Bytes[0] > 0); t++)
	{
		Run(5);
	}
	assert(t < 2000);
	assert(i2s.Rate(44100) == 44100 && s_M[6].Run);
	Run(12000);
	uint32_t last = 0;
	bool any = false;
	int gaps = 0;
	for (uint32_t v : s_M[6].Sent)
	{
		if (v == 0)
		{
			continue;
		}
		uint32_t n = 0;
		while (n < 300 && Stereo(n) != v)
		{
			n++;
		}
		assert(n < 300 && (!any || n > last));
		gaps += any && n != last + 1;
		last = n;
		any = true;
	}
	assert(last == 299 && gaps == 1);
	// The DMA interrupt held off: the transmit FIFO runs low, no Flexcomm
	// interrupt with the DMA
	s_IrqCalls = 0;
	s_NvicEn &= ~(1UL << (DMA0_IRQn & 31));
	Run(1000);
	s_NvicEn |= 1UL << (DMA0_IRQn & 31);
	Run(1000);
	assert(s_IrqCalls == 0 && s_M[6].Underruns > 0 && s_Evt[DEVINTRF_EVT_STATECHG] == 0 && s_M[6].Run);
	i2s.PowerOff();
	printf("dma: PASS\n");
}

static void TestLifecycle()
{
	I2S i2s;
	uint8_t buf[64];

	Fresh();
	I2SCfg_t c = Cfg(6, s_Duplex, 7, true, false, true, true);
	assert(i2s.Init(c));
	Run(1000);
	assert(i2s.Rx(0, buf, 64) == 64);
	// Disable: stopped, the FIFOs emptied
	i2s.Disable();
	assert(!s_M[6].Run && !s_M[7].Run && s_M[6].Tx.empty() && s_M[7].Rx.empty());
	assert(i2s.Rx(0, buf, 64) == 0 && i2s.Tx(0, buf, 64) == 0);
	assert((s_M[6].IntEn | s_M[7].IntEn) == 0);
	i2s.Enable();
	assert(s_M[6].Run && s_M[7].Run && s_M[7].SlaveLate == 0);
	Run(1000);
	assert(i2s.Rx(0, buf, 64) == 64);
	// Not started when stopped
	i2s.Disable();
	assert(!i2s.StartRx(0) && !i2s.StartTx(0));
	i2s.Enable();
	assert(i2s.StartRx(0));
	i2s.StopRx();
	// The DevIntrf Enable while running does not set it up again
	DevIntrf_t *di = i2s;
	double start = s_M[6].StartUs;
	Run(100);
	di->Enable(di);
	assert(s_M[6].Run && s_M[7].Run && s_M[6].StartUs == start);
	// TX frames held in the CFIFO when Disable is called are not sent
	g_IrqMasked = 1;
	static uint8_t held[32 * 4];
	for (uint32_t n = 0; n < 32; n++)
	{
		uint32_t v = Stereo(500 + n);
		memcpy(held + n * 4, &v, 4);
	}
	assert(i2s.Tx(0, held, sizeof(held)) == sizeof(held));
	i2s.Disable();
	g_IrqMasked = 0;
	size_t sentBefore = s_M[6].Sent.size();
	i2s.Enable();
	Run(3000);
	for (size_t i = sentBefore; i < s_M[6].Sent.size(); i++)
	{
		assert(s_M[6].Sent[i] == 0);
	}
	// Init again: the same device set up again
	c.Freq = 44100;
	assert(i2s.Init(c) && i2s.Rate() == 44100 && s_M[6].Run);
	// PowerOff: stopped, the Flexcomms, their clocks and the audio PLL
	// released (the last clock turned off is seen)
	Sys()->AHBCLKCTRLCLR[1] = 0;
	i2s.PowerOff();
	assert(!s_M[6].Run && !s_M[7].Run && (s_M[6].IntEn | s_M[7].IntEn) == 0 && s_PllFreq == 0);
	assert((Sys()->FCLKSEL[6] & 7U) == 7U && (Sys()->FCLKSEL[7] & 7U) == 7U);
	assert(Sys()->AHBCLKCTRLCLR[1] == SYSCON_AHBCLKCTRL_FLEXCOMM7_MASK);
	// Twice, and an object never set up (DevIntrf PowerOff is checked)
	i2s.PowerOff();
	I2S none;
	none.PowerOff();
	// Set up again after PowerOff
	assert(i2s.Init(c) && i2s.Rate() == 44100);
	Run(1000);
	assert(i2s.Rx(0, buf, 64) == 64);
	i2s.PowerOff();

	// An object set up then powered off does nothing to the one set up after
	I2S b;
	Fresh();
	c = Cfg(7, s_RxOnly, 3, false, false, false, false);
	assert(b.Init(c));
	b.PowerOff();
	c = Cfg(6, s_Duplex, 7, true, false, true, true);
	assert(i2s.Init(c) && i2s.Rate() == 48000);
	Run(500);
	assert(b.Rate() == 0 && b.Rate(8000) == 0 && i2s.Rate() == 48000 && s_M[6].Div == 7);
	assert(b.Rx(0, buf, 4) == 0 && b.Tx(0, buf, 4) == 0);
	b.Disable();
	b.Enable();
	b.Reset();
	DevIntrf_t *bi = b;
	assert(bi->GetHandle(bi) == NULL);
	b.PowerOff();
	assert(s_M[6].Run && s_M[7].Run && s_PllFreq != 0);
	Run(1000);
	assert(i2s.Rx(0, buf, 64) == 64);
	i2s.PowerOff();
	printf("lifecycle: PASS\n");
}

int main()
{
	setvbuf(stdout, NULL, _IONBF, 0);
	TestInitChecks();
	TestRates();
	TestTx();
	TestRx();
	TestDuplex();
	TestFail();
	TestDma();
	TestLifecycle();
	printf("i2s: PASS\n");

	return 0;
}
