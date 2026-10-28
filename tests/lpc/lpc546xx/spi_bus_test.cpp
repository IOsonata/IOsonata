// Host regression of the LPC546xx Flexcomm SPI master and slave, in every
// master mode (polling, interrupt, DMA, DMA with interrupt) and slave mode
// (interrupt, DMA), with the sequence of exemples/spi/spi_master_slave.cpp.
//
// Bus model: a master Flexcomm (CFG MASTER and ENABLE) shifts one frame per
// step while its TX FIFO has an entry, and while its RX FIFO has room unless
// the entry has RXIGNORE, as the device header describes the master halt. The
// master chip select is a GPIO wired to the SSEL of the slave Flexcomm: its
// edges set SSA and SSD of the slave at once. A selected and enabled slave
// sends its TX FIFO entry, or 0xFF with TXERR when it is empty, and takes the
// master byte into its RX FIFO, or sets RXERR when it is full. FIFOs have 8
// entries (FIFOCFG SIZE 1).
//
// FIFOWR: a 32 bit write pushes the data with its control bits and keeps
// them, a halfword write to the control half pushes nothing. A DMA byte write
// pushes the data with the control bits kept. DMA requests: TX while the TX
// FIFO is not full with DMATX, RX while the RX FIFO is not empty with DMARX.
//
// Time passes on each FIFOSTAT and STAT read: the frames shift, the DMA
// moves one value per channel and the enabled interrupts run when they are
// not masked. Inside an interrupt only the DMA moves.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <string>
#include <pthread.h>

#include "LPC546xx.h"
#include "dma_model.h"

// Flexcomm PSELID, the present bits are read only. Selecting a function
// follows the Flexcomm reset, the SPI model is reset with it.
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

#define FIFO_DEPTH	8

struct SpiM {
	uint32_t FifoCfg = 0;
	uint32_t IntEn = 0;
	uint32_t FifoIntEn = 0;
	std::deque<uint32_t> Tx;
	std::deque<uint32_t> Rx;	// Data, SOT on the first byte after SSEL asserted
	bool Sot = false;			// Next byte received is the first of a frame
	bool TxErr = false;
	bool RxErr = false;
	bool Ssa = false;
	bool Ssd = false;
	bool Sel = false;
	int Frames = 0;
	std::string Log;			// Master: MOSI bytes, slave: MISO bytes
};

static SpiM s_S[10];
static int RegNo(const void *p);
static void Tick(void);

struct SpiFifoCfgR {
	operator uint32_t() const;
	SpiFifoCfgR &operator=(uint32_t v);
	SpiFifoCfgR &operator|=(uint32_t v) { return *this = (uint32_t)*this | v; }
	SpiFifoCfgR &operator&=(uint32_t v) { return *this = (uint32_t)*this & v; }
};
struct SpiFifoStatR { operator uint32_t() const; SpiFifoStatR &operator=(uint32_t v); };
struct SpiStatR { operator uint32_t() const; SpiStatR &operator=(uint32_t v); };
struct SpiIntSetR { operator uint32_t() const; SpiIntSetR &operator=(uint32_t v); };
struct SpiIntClrR { SpiIntClrR &operator=(uint32_t v); };
struct SpiFifoIntSetR { operator uint32_t() const; SpiFifoIntSetR &operator=(uint32_t v); };
struct SpiFifoIntClrR { SpiFifoIntClrR &operator=(uint32_t v); };
// The control half is plain memory, written by the halfword access
struct SpiFifoWrR {
	uint16_t lo = 0;
	volatile uint16_t hi = 0;
	SpiFifoWrR &operator=(uint32_t v);
};
struct SpiFifoRdR { operator uint32_t() const; };

struct SpiRegs {
	uint32_t CFG = 0, DLY = 0;
	SpiStatR STAT;
	SpiIntSetR INTENSET;
	SpiIntClrR INTENCLR;
	uint32_t DIV = 0, INTSTAT = 0;
	SpiFifoCfgR FIFOCFG;
	SpiFifoStatR FIFOSTAT;
	uint32_t FIFOTRIG = 0;
	SpiFifoIntSetR FIFOINTENSET;
	SpiFifoIntClrR FIFOINTENCLR;
	uint32_t FIFOINTSTAT = 0;
	SpiFifoWrR FIFOWR;
	SpiFifoRdR FIFORD;
	uint32_t FIFORDNOPOP = 0;
};

static SpiRegs s_Regs[10];

static int RegNo(const void *p)
{
	for (int i = 0; i < 10; i++)
	{
		if ((const uint8_t *)p >= (const uint8_t *)&s_Regs[i] && (const uint8_t *)p < (const uint8_t *)&s_Regs[i + 1])
		{
			return i;
		}
	}
	assert(false);
	return -1;
}

static void Push(int No, uint32_t v)
{
	SpiM &m = s_S[No];

	assert(m.FifoCfg & SPI_FIFOCFG_ENABLETX_MASK);
	if (m.Tx.size() >= FIFO_DEPTH)
	{
		m.TxErr = true;
		return;
	}
	// One byte frames
	assert((v & SPI_FIFOWR_LEN_MASK) == SPI_FIFOWR_LEN(7));
	m.Tx.push_back(v);
}

static uint32_t Pop(int No)
{
	SpiM &m = s_S[No];

	assert(!m.Rx.empty());
	uint32_t d = m.Rx.front();
	m.Rx.pop_front();
	return d;
}

SpiFifoCfgR::operator uint32_t() const
{
	return s_S[RegNo(this)].FifoCfg | SPI_FIFOCFG_SIZE(1);
}
SpiFifoCfgR &SpiFifoCfgR::operator=(uint32_t v)
{
	SpiM &m = s_S[RegNo(this)];
	m.FifoCfg = v & (SPI_FIFOCFG_ENABLETX_MASK | SPI_FIFOCFG_ENABLERX_MASK | SPI_FIFOCFG_DMATX_MASK |
					 SPI_FIFOCFG_DMARX_MASK);
	if (v & SPI_FIFOCFG_EMPTYTX_MASK)
	{
		m.Tx.clear();
	}
	if (v & SPI_FIFOCFG_EMPTYRX_MASK)
	{
		m.Rx.clear();
	}
	return *this;
}

static uint32_t FifoStat(int No)
{
	const SpiM &m = s_S[No];
	return (m.TxErr ? SPI_FIFOSTAT_TXERR_MASK : 0U) | (m.RxErr ? SPI_FIFOSTAT_RXERR_MASK : 0U) |
		   (m.Tx.empty() ? SPI_FIFOSTAT_TXEMPTY_MASK : 0U) |
		   (m.Tx.size() < FIFO_DEPTH ? SPI_FIFOSTAT_TXNOTFULL_MASK : 0U) |
		   (!m.Rx.empty() ? SPI_FIFOSTAT_RXNOTEMPTY_MASK : 0U) |
		   (m.Rx.size() >= FIFO_DEPTH ? SPI_FIFOSTAT_RXFULL_MASK : 0U) |
		   SPI_FIFOSTAT_TXLVL(m.Tx.size()) | SPI_FIFOSTAT_RXLVL(m.Rx.size());
}

SpiFifoStatR::operator uint32_t() const
{
	int no = RegNo(this);
	Tick();
	return FifoStat(no);
}
SpiFifoStatR &SpiFifoStatR::operator=(uint32_t v)
{
	SpiM &m = s_S[RegNo(this)];
	if (v & SPI_FIFOSTAT_TXERR_MASK)
	{
		m.TxErr = false;
	}
	if (v & SPI_FIFOSTAT_RXERR_MASK)
	{
		m.RxErr = false;
	}
	return *this;
}

static bool IsMaster(int No)
{
	return (s_Regs[No].CFG & SPI_CFG_MASTER_MASK) != 0;
}

static uint32_t StatValue(int No)
{
	const SpiM &m = s_S[No];
	return (m.Ssa ? SPI_STAT_SSA_MASK : 0U) | (m.Ssd ? SPI_STAT_SSD_MASK : 0U) |
		   (IsMaster(No) && m.Tx.empty() ? SPI_STAT_MSTIDLE_MASK : 0U);
}

SpiStatR::operator uint32_t() const
{
	int no = RegNo(this);
	Tick();
	return StatValue(no);
}
SpiStatR &SpiStatR::operator=(uint32_t v)
{
	SpiM &m = s_S[RegNo(this)];
	if (v & SPI_STAT_SSA_MASK)
	{
		m.Ssa = false;
	}
	if (v & SPI_STAT_SSD_MASK)
	{
		m.Ssd = false;
	}
	return *this;
}
SpiIntSetR::operator uint32_t() const { return s_S[RegNo(this)].IntEn; }
SpiIntSetR &SpiIntSetR::operator=(uint32_t v) { s_S[RegNo(this)].IntEn |= v; return *this; }
SpiIntClrR &SpiIntClrR::operator=(uint32_t v) { s_S[RegNo(this)].IntEn &= ~v; return *this; }
SpiFifoIntSetR::operator uint32_t() const { return s_S[RegNo(this)].FifoIntEn; }
SpiFifoIntSetR &SpiFifoIntSetR::operator=(uint32_t v) { s_S[RegNo(this)].FifoIntEn |= v; return *this; }
SpiFifoIntClrR &SpiFifoIntClrR::operator=(uint32_t v) { s_S[RegNo(this)].FifoIntEn &= ~v; return *this; }
SpiFifoWrR &SpiFifoWrR::operator=(uint32_t v)
{
	hi = (uint16_t)(v >> 16);
	Push(RegNo(this), v);
	return *this;
}
SpiFifoRdR::operator uint32_t() const { return Pop(RegNo(this)); }

// GPIO with SET and CLR acting on PIN, the chip select edges reach the slave
static void CsEdge(void);
struct GpioWr {
	uint32_t *pPin = nullptr;
	bool bSet = true;
	GpioWr &operator=(uint32_t v)
	{
		*pPin = bSet ? (*pPin | v) : (*pPin & ~v);
		CsEdge();
		return *this;
	}
};
struct GpioRegs {
	uint8_t B[6][32] = {};
	uint32_t DIR[6] = {}, PIN[6] = {}, NOT[6] = {}, DIRSET[6] = {}, DIRCLR[6] = {};
	GpioWr SET[6], CLR[6];
	GpioRegs()
	{
		for (int i = 0; i < 6; i++)
		{
			SET[i].pPin = &PIN[i];
			CLR[i].pPin = &PIN[i];
			CLR[i].bSet = false;
		}
	}
};

alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
static FcRegs s_Fc[10];
static GpioRegs s_Gpio;

// Registers and FIFOs at their reset values, the control half of FIFOWR 0
static void FcReset(const void *pPsel)
{
	for (int i = 0; i < 10; i++)
	{
		if (pPsel == &s_Fc[i].PSELID)
		{
			s_S[i] = SpiM();
			s_Regs[i].CFG = 0;
			s_Regs[i].DLY = 0;
			s_Regs[i].DIV = 0;
			s_Regs[i].FIFOTRIG = 0;
			s_Regs[i].FIFOWR.hi = 0;
			return;
		}
	}
	assert(false);
}
uint32_t g_IrqMasked = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { s_NvicPrio[n & 63] = p; }
void NVIC_EnableIRQ(IRQn_Type n) { s_NvicEn |= 1UL << (n & 31); }
void NVIC_DisableIRQ(IRQn_Type n) { s_NvicEn &= ~(1UL << (n & 31)); }
void __DMB(void) {}

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
#undef SPI0
#undef SPI1
#undef SPI2
#undef SPI3
#undef SPI4
#undef SPI5
#undef SPI6
#undef SPI7
#undef SPI8
#undef SPI9
#undef GPIO
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
#define SPI_Type SpiRegs
#define SPI0 (&s_Regs[0])
#define SPI1 (&s_Regs[1])
#define SPI2 (&s_Regs[2])
#define SPI3 (&s_Regs[3])
#define SPI4 (&s_Regs[4])
#define SPI5 (&s_Regs[5])
#define SPI6 (&s_Regs[6])
#define SPI7 (&s_Regs[7])
#define SPI8 (&s_Regs[8])
#define SPI9 (&s_Regs[9])
#define GPIO (&s_Gpio)
#define DMA_Type DmaRegs
#define DMA0 (&s_DmaRegs)

#include "dma_lpc546xx.h"

static bool IsSpi(int No)
{
	return (s_Fc[No].PSELID.value & 0xFU) == 2U;
}

static bool DmaReqOn(int Ch)
{
	for (int i = 0; i < 10; i++)
	{
		if (!IsSpi(i))
		{
			continue;
		}
		const SpiM &m = s_S[i];
		if (Ch == LPC546XX_DMA_CH_FC_TX(i))
		{
			return (m.FifoCfg & SPI_FIFOCFG_DMATX_MASK) && m.Tx.size() < FIFO_DEPTH;
		}
		if (Ch == LPC546XX_DMA_CH_FC_RX(i))
		{
			return (m.FifoCfg & SPI_FIFOCFG_DMARX_MASK) && !m.Rx.empty();
		}
	}
	return false;
}

static bool DmaPeriphRead(uint32_t Addr, uint32_t *pVal)
{
	for (int i = 0; i < 10; i++)
	{
		if (Addr == (uint32_t)(uintptr_t)&s_Regs[i].FIFORD)
		{
			*pVal = Pop(i);
			return true;
		}
	}
	return false;
}

static bool DmaPeriphWrite(uint32_t Addr, uint32_t Val)
{
	for (int i = 0; i < 10; i++)
	{
		if (Addr == (uint32_t)(uintptr_t)&s_Regs[i].FIFOWR)
		{
			Push(i, (Val & 0xFFU) | ((uint32_t)s_Regs[i].FIFOWR.hi << 16));
			return true;
		}
	}
	return false;
}

#include "coredev/iopincfg.h"
#include "coredev/spi.h"

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)PortNo; (void)PinNo; (void)PinOp; (void)Dir; (void)Resistor; (void)Type;
}
void IOPinDisable(int PortNo, int PinNo) { (void)PortNo; (void)PinNo; }

#include "../../../src/coredev/shared_intrf.cpp"
#include "../../../src/coredev/spi.cpp"
#include "../../../ARM/NXP/LPC546xx/src/flexcomm_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/dma_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/spi_lpc546xx.cpp"
#include "../../../src/device_intrf.cpp"

// SPIInit hands DevNo 10 to the SPIFI driver, not linked here
bool Lpc546xxSpifiInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	(void)pDev;
	(void)pCfgData;
	return false;
}

#define MASTER_FC	3
#define SLAVE_FC	9
#define CS_PORT		0
#define CS_PIN		1

static bool CsLow(void)
{
	return (s_Gpio.PIN[CS_PORT] & (1UL << CS_PIN)) == 0;
}

// The master chip select is the SSEL of the slave Flexcomm
static void CsEdge(void)
{
	SpiM &s = s_S[SLAVE_FC];
	bool low = CsLow();

	if (low && !s.Sel)
	{
		s.Sel = true;
		s.Ssa = true;
		s.Sot = true;
	}
	else if (!low && s.Sel)
	{
		s.Sel = false;
		s.Ssd = true;
	}
	// The SSEL pin as the GPIO reads it
	s_Gpio.B[3][30] = low ? 0 : 1;
}

static void (* const s_FcVector[10])(void) = {
	FLEXCOMM0_IRQHandler, FLEXCOMM1_IRQHandler, FLEXCOMM2_IRQHandler, FLEXCOMM3_IRQHandler,
	FLEXCOMM4_IRQHandler, FLEXCOMM5_IRQHandler, FLEXCOMM6_IRQHandler, FLEXCOMM7_IRQHandler,
	FLEXCOMM8_IRQHandler, FLEXCOMM9_IRQHandler,
};

static bool FcIrqPending(int No)
{
	const SpiM &m = s_S[No];
	uint32_t trig = s_Regs[No].FIFOTRIG;
	uint32_t fint = (m.TxErr ? SPI_FIFOINTENSET_TXERR_MASK : 0U) | (m.RxErr ? SPI_FIFOINTENSET_RXERR_MASK : 0U);

	if ((trig & SPI_FIFOTRIG_TXLVLENA_MASK) &&
		m.Tx.size() <= ((trig & SPI_FIFOTRIG_TXLVL_MASK) >> SPI_FIFOTRIG_TXLVL_SHIFT))
	{
		fint |= SPI_FIFOINTENSET_TXLVL_MASK;
	}
	if ((trig & SPI_FIFOTRIG_RXLVLENA_MASK) &&
		m.Rx.size() > ((trig & SPI_FIFOTRIG_RXLVL_MASK) >> SPI_FIFOTRIG_RXLVL_SHIFT))
	{
		fint |= SPI_FIFOINTENSET_RXLVL_MASK;
	}
	return (s_NvicEn & (1UL << (Lpc546xxFlexcommIrqNo(No) & 31))) &&
		   ((fint & m.FifoIntEn) || (StatValue(No) & m.IntEn));
}

// One frame of a master, with the slave selected by the chip select
static bool Shift(int No)
{
	SpiM &m = s_S[No];
	SpiRegs &r = s_Regs[No];

	if (!IsSpi(No) || !IsMaster(No) || (r.CFG & SPI_CFG_ENABLE_MASK) == 0 || m.Tx.empty())
	{
		return false;
	}
	uint32_t e = m.Tx.front();
	bool ignore = (e & SPI_FIFOWR_RXIGNORE_MASK) != 0;
	if (!ignore && m.Rx.size() >= FIFO_DEPTH)
	{
		// The master halts on a full RX FIFO
		return false;
	}
	m.Tx.pop_front();
	m.Frames++;

	uint8_t mosi = (uint8_t)e;
	uint8_t miso = 0xFF;
	char t[4];

	snprintf(t, sizeof(t), "%02X", mosi);
	m.Log += t;

	SpiM &s = s_S[SLAVE_FC];
	SpiRegs &sr = s_Regs[SLAVE_FC];
	if (IsSpi(SLAVE_FC) && !IsMaster(SLAVE_FC) && (sr.CFG & SPI_CFG_ENABLE_MASK) && s.Sel)
	{
		// Both ends in the same SPI mode
		uint32_t mode = SPI_CFG_CPOL_MASK | SPI_CFG_CPHA_MASK | SPI_CFG_LSBF_MASK;
		assert((r.CFG & mode) == (sr.CFG & mode));
		if (s.Tx.empty())
		{
			s.TxErr = true;
		}
		else
		{
			miso = (uint8_t)s.Tx.front();
			s.Tx.pop_front();
		}
		if (s.Rx.size() >= FIFO_DEPTH)
		{
			s.RxErr = true;
		}
		else
		{
			s.Rx.push_back(mosi | (s.Sot ? SPI_FIFORD_SOT_MASK : 0U));
		}
		s.Sot = false;
		s.Frames++;
		snprintf(t, sizeof(t), "%02X", miso);
		s.Log += t;
	}
	if (!ignore)
	{
		m.Rx.push_back(miso);
	}
	return true;
}


static int s_InTick = 0;

static void Tick(void)
{
	if (s_InTick)
	{
		// The DMA keeps moving while an interrupt runs
		DmaStep(false);
		return;
	}
	s_InTick = 1;
	for (int r = 0; r < 4; r++)
	{
		if (g_IrqMasked == 0)
		{
			if (DmaIrqPending())
			{
				DMA0_IRQHandler();
			}
			for (int i = 0; i < 10; i++)
			{
				if (g_IrqMasked == 0 && IsSpi(i) && FcIrqPending(i))
				{
					s_FcVector[i]();
				}
			}
		}
		DmaStep(false);
		for (int i = 0; i < 10; i++)
		{
			Shift(i);
		}
	}
	s_InTick = 0;
}

// Slave of exemples/spi/spi_master_slave.cpp
static SPIDev_t s_Master;
static SPIDev_t s_Slave;
static uint8_t s_SlaveTx[2048];
static uint8_t s_SlaveRx[2048];
static int s_SlaveRxSize = 129;
static int s_SlaveTxSize = 129;
static bool s_Done = false;
static int s_Count = 0;
static unsigned s_Completions = 0;
static unsigned s_Arms = 0;
static int s_CountHist[8];			// Counts of the last completions
static bool s_MasterDone = false;
static int s_MasterCount = 0;
static int s_MasterEvt = 0;

static int SlaveEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event, uint8_t *pBuffer, int Length)
{
	(void)pDev;
	if (Event == DEVINTRF_EVT_STATECHG)
	{
		s_Arms++;
		SPISetSlaveRxBuffer(&s_Slave, 0, s_SlaveRx, s_SlaveRxSize);
		SPISetSlaveTxData(&s_Slave, 0, s_SlaveTx, s_SlaveTxSize);
	}
	else if (Event == DEVINTRF_EVT_COMPLETED)
	{
		assert(pBuffer == s_SlaveRx);
		s_Count = Length;
		s_CountHist[s_Completions & 7] = Length;
		s_Completions++;
		s_Done = true;
	}
	return 0;
}

static int MasterEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event, uint8_t *pBuffer, int Length)
{
	(void)pDev; (void)pBuffer;
	if (Event == DEVINTRF_EVT_COMPLETED)
	{
		// The chip select is released before the completion
		assert(!CsLow() && s_Master.DevIntrf.bTxReady);
		s_MasterCount = Length;
		s_MasterDone = true;
		s_MasterEvt++;
	}
	return 0;
}

static void Settle(void)
{
	for (int i = 0; i < 100; i++)
	{
		Tick();
	}
}

static bool WaitFlag(const bool *pFlag)
{
	for (int i = 0; i < 2000000 && !*pFlag; i++)
	{
		Tick();
	}
	return *pFlag;
}

// The interface is free and the chip select released
static void BusFree(void)
{
	assert(!atomic_flag_test_and_set(&s_Master.DevIntrf.bBusy));
	atomic_flag_clear(&s_Master.DevIntrf.bBusy);
	assert(!CsLow());
}

static int MasterTx(const uint8_t *p, int Len)
{
	s_MasterDone = false;
	int c = SPITx(&s_Master, 0, (uint8_t *)p, Len);
	if (c < 0)
	{
		c = WaitFlag(&s_MasterDone) ? s_MasterCount : -2;
	}
	return c;
}

static int MasterRx(uint8_t *p, int Len)
{
	s_MasterDone = false;
	int c = SPIRx(&s_Master, 0, p, Len);
	if (c < 0)
	{
		c = WaitFlag(&s_MasterDone) ? s_MasterCount : -2;
	}
	return c;
}

static bool Frame(int Length, uint8_t Seed, bool Receive = false)
{
	uint8_t masterTx[2048];
	uint8_t masterRx[2048] = {};
	unsigned before = s_Completions;

	assert(Length <= (int)sizeof(masterTx));
	s_Done = false;
	for (int i = 0; i < Length; i++)
	{
		masterTx[i] = (uint8_t)(Seed + i);
	}
	int count = Receive ? MasterRx(masterRx, Length) : MasterTx(masterTx, Length);
	bool done = WaitFlag(&s_Done);
	Settle();
	BusFree();

	bool pass = count == Length && done && s_Completions == before + 1 && s_Count == Length;
	for (int i = 0; i < Length; i++)
	{
		uint8_t want = Receive ? s_Master.Cfg.DummyByte : (uint8_t)(Seed + i);
		if ((Receive && masterRx[i] != s_SlaveTx[i]) || s_SlaveRx[i] != want)
		{
			pass = false;
		}
	}
	if (!pass)
	{
		printf("Frame %d seed %02x rx %d: count %d done %d completions %u->%u slave count %d\n",
			   Length, Seed, Receive, count, done, before, s_Completions, s_Count);
	}
	return pass;
}

static const IOPinCfg_t s_MasterPins[] = {
	{0, 0, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 2, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{0, 3, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{CS_PORT, CS_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const IOPinCfg_t s_SlavePins[] = {
	{3, 20, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{3, 22, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{3, 21, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{3, 30, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static SPICfg_t MasterCfg(bool bDma, bool bInt, int Mode)
{
	SPICfg_t c = {};

	c.DevNo = MASTER_FC;
	c.Phy = SPIPHY_NORMAL;
	c.Mode = SPIMODE_MASTER;
	c.pIOPinMap = s_MasterPins;
	c.NbIOPins = 4;
	c.Rate = 1000000;
	c.DataSize = 8;
	c.BitOrder = SPIDATABIT_MSB;
	c.ClkPol = (Mode & 2) ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
	c.DataPhase = (Mode & 1) ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
	c.ChipSel = SPICSEL_AUTO;
	c.bDmaEn = bDma;
	c.bIntEn = bInt;
	c.IntPrio = 6;
	c.DummyByte = 0xFF;
	c.EvtCB = MasterEvent;
	return c;
}

static SPICfg_t SlaveCfg(bool bDma, int Mode)
{
	SPICfg_t c = MasterCfg(bDma, true, Mode);

	c.DevNo = SLAVE_FC;
	c.Mode = SPIMODE_SLAVE;
	c.pIOPinMap = s_SlavePins;
	c.Rate = 100000;
	c.EvtCB = SlaveEvent;
	return c;
}

static const char *ModeName(bool bDma, bool bInt)
{
	return bDma ? (bInt ? "DMA+interrupt" : "DMA") : (bInt ? "interrupt" : "polling");
}

static void InitPair(const SPICfg_t &Mc, const SPICfg_t &Sc)
{
	assert(SPIInit(&s_Master, &Mc));
	unsigned arms = s_Arms;
	assert(SPIInit(&s_Slave, &Sc));
	// The slave asks for its buffers at once
	assert(s_Arms == arms + 1);
	assert(s_Slave.DevIntrf.bIntEn && s_Slave.DevIntrf.bDma == Sc.bDmaEn);
	assert(IsMaster(MASTER_FC) && !IsMaster(SLAVE_FC));
}

static void RunCombo(int Mm, int Sm)
{
	bool mdma = Mm >= 2;
	bool mint = Mm == 1 || Mm == 3;
	int dmavalues = s_DmaM.Values;
	int evts = s_MasterEvt;

	for (int mode = 0; mode < 4; mode++)
	{
		SPICfg_t mc = MasterCfg(mdma, mint, mode);
		SPICfg_t sc = SlaveCfg(Sm == 1, mode);

		s_SlaveRxSize = 129;
		s_SlaveTxSize = 129;
		for (int i = 0; i < 129; i++)
		{
			s_SlaveTx[i] = (uint8_t)(0xA0 ^ i);
		}
		InitPair(mc, sc);

		const int lengths[] = { 1, 16, 129 };
		for (int n = 0; n < 3; n++)
		{
			int len = lengths[n];

			memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
			assert(Frame(len, 0x30));
			// No reset: the end of the frame prepares byte zero again
			memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
			assert(Frame(len, 0x50));
			assert(Frame(len, 0, true));
			if (len > 1)
			{
				// A short frame must drop the TX data loaded and not sent
				assert(Frame(1, 0x70) && Frame(len, 0x90));
			}
		}
	}

	if (mdma || Sm == 1)
	{
		assert(s_DmaM.Values > dmavalues);
	}
	else
	{
		assert(s_DmaM.Values == dmavalues);
	}
	if (mint)
	{
		assert(s_MasterEvt > evts);
	}
	else
	{
		assert(s_MasterEvt == evts);
	}

	printf("lpc546xx spi master %s, slave %s: PASS\n", ModeName(mdma, mint), Sm == 1 ? "DMA" : "interrupt");
}

static void ExtraTests(void)
{
	// Long frames: more than one DMA block of 1024 on the master, the slave
	// DMA takes 1024 bytes, the interrupt the rest
	for (int sm = 0; sm < 2; sm++)
	{
		for (int mm = 0; mm < 4; mm++)
		{
			InitPair(MasterCfg(mm >= 2, mm == 1 || mm == 3, 0), SlaveCfg(sm == 1, 0));
			s_SlaveRxSize = 2000;
			s_SlaveTxSize = 2000;
			for (int i = 0; i < 2000; i++)
			{
				s_SlaveTx[i] = (uint8_t)(i * 3 + (i >> 8));
			}
			DeviceIntrfReset(&s_Slave.DevIntrf);
			memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
			int values = s_DmaM.Values;
			assert(Frame(1500, 0x11));
			if (sm == 1 && mm < 2)
			{
				// Only the slave uses the DMA: 1024 bytes each way
				assert(s_DmaM.Values - values >= 2048);
			}
			assert(Frame(1500, 0, true));
		}
	}

	// An overrun of the slave RX FIFO is reported as -1
	for (int sm = 0; sm < 2; sm++)
	{
		InitPair(MasterCfg(false, false, 0), SlaveCfg(sm == 1, 0));
		s_SlaveRxSize = 129;
		s_SlaveTxSize = 129;
		DeviceIntrfReset(&s_Slave.DevIntrf);
		uint8_t tx[16] = {};
		uint32_t irq = 1UL << (Lpc546xxFlexcommIrqNo(SLAVE_FC) & 31);
		uint32_t dmarx = s_S[SLAVE_FC].FifoCfg & SPI_FIFOCFG_DMARX_MASK;
		// The slave interrupt and its RX channel held off for the frame
		s_NvicEn &= ~irq;
		s_S[SLAVE_FC].FifoCfg &= ~SPI_FIFOCFG_DMARX_MASK;
		s_Done = false;
		assert(SPITx(&s_Master, 0, tx, 16) == 16);
		s_S[SLAVE_FC].FifoCfg |= dmarx;
		s_NvicEn |= irq;
		assert(WaitFlag(&s_Done) && s_Count == -1);
		Settle();
		assert(Frame(16, 0x61));
	}

	// A frame longer than the slave buffer is reported as -1, the next one
	// is good again. A frame longer than the slave data gets the DummyByte.
	for (int sm = 0; sm < 2; sm++)
	{
		InitPair(MasterCfg(false, false, 0), SlaveCfg(sm == 1, 0));
		s_SlaveRxSize = 10;
		s_SlaveTxSize = 4;
		DeviceIntrfReset(&s_Slave.DevIntrf);
		uint8_t tx[16];
		uint8_t rx[16];
		for (int i = 0; i < 16; i++)
		{
			tx[i] = (uint8_t)(0x40 + i);
		}
		s_Done = false;
		assert(MasterTx(tx, 16) == 16);
		assert(WaitFlag(&s_Done) && s_Count == -1);
		assert(memcmp(s_SlaveRx, tx, 10) == 0);
		Settle();
		s_Done = false;
		memset(rx, 0, sizeof(rx));
		assert(MasterRx(rx, 8) == 8);
		assert(WaitFlag(&s_Done) && s_Count == 8);
		assert(memcmp(rx, s_SlaveTx, 4) == 0 && rx[4] == 0xFF && rx[7] == 0xFF);
		Settle();
	}

	// Interrupt: the command phase of a read runs to its end with the chip
	// select held, the receive reports by the event
	{
		InitPair(MasterCfg(false, true, 0), SlaveCfg(false, 0));
		s_SlaveRxSize = 129;
		s_SlaveTxSize = 129;
		for (int i = 0; i < 129; i++)
		{
			s_SlaveTx[i] = (uint8_t)(0xA0 ^ i);
		}
		DeviceIntrfReset(&s_Slave.DevIntrf);
		uint8_t cmd[2] = { 0x9F, 0x01 };
		uint8_t rx[8];
		s_Done = false;
		s_MasterDone = false;
		s_S[MASTER_FC].Log.clear();
		int c = DeviceIntrfRead(&s_Master.DevIntrf, 0, cmd, 2, rx, 8);
		assert(c == -1);
		assert(WaitFlag(&s_MasterDone) && s_MasterCount == 8);
		assert(WaitFlag(&s_Done) && s_Count == 10);
		Settle();
		BusFree();
		assert(s_S[MASTER_FC].Log == "9F01FFFFFFFFFFFFFFFF");
		assert(memcmp(rx, &s_SlaveTx[2], 8) == 0 && s_SlaveRx[0] == 0x9F && s_SlaveRx[1] == 0x01);

		// The data of an interrupt transmit is copied, the caller may reuse
		// its buffer once TxData returns
		uint8_t tx[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
		s_Done = false;
		s_MasterDone = false;
		assert(SPITx(&s_Master, 0, tx, 9) == -1);
		memset(tx, 0xEE, sizeof(tx));
		assert(WaitFlag(&s_MasterDone) && s_MasterCount == 9);
		assert(WaitFlag(&s_Done) && s_Count == 9 && s_SlaveRx[0] == 1 && s_SlaveRx[8] == 9);
		Settle();
		BusFree();

		// Disable then enable prepares the slave again
		unsigned arms = s_Arms;
		SPIDisable(&s_Slave);
		SPIEnable(&s_Slave);
		assert(s_Arms == arms + 1);
		assert(Frame(16, 0x21));
	}

	// A DMA error on the first frame ends the polling DMA transfer with
	// nothing done, DeviceIntrfTx tries again
	{
		InitPair(MasterCfg(true, false, 0), SlaveCfg(false, 0));
		uint8_t tx[100];
		for (int i = 0; i < 100; i++)
		{
			tx[i] = (uint8_t)i;
		}
		unsigned before = s_Completions;
		s_DmaM.ErrCh = LPC546XX_DMA_CH_FC_TX(MASTER_FC);
		assert(MasterTx(tx, 100) == 100);
		Settle();
		BusFree();
		// The frame without a byte is not reported, then the frame of the retry
		assert(s_Completions == before + 1 && s_Count == 100 && memcmp(s_SlaveRx, tx, 100) == 0);
	}

	// A wait with the master interrupt held off (a caller with interrupts
	// masked or a priority as high) moves the transfer itself
	for (int mm = 1; mm < 4; mm += 2)
	{
		InitPair(MasterCfg(mm == 3, true, 0), SlaveCfg(false, 0));
		s_SlaveRxSize = 129;
		s_SlaveTxSize = 129;
		DeviceIntrfReset(&s_Slave.DevIntrf);
		uint8_t tx[100];
		for (int i = 0; i < 100; i++)
		{
			tx[i] = (uint8_t)(i + 1);
		}
		uint32_t irq = 1UL << (Lpc546xxFlexcommIrqNo(MASTER_FC) & 31);
		uint32_t dirq = 1UL << DMA0_IRQn;
		uint32_t en = s_NvicEn;
		s_NvicEn &= ~(irq | dirq);
		s_Done = false;
		assert(SPITx(&s_Master, 0, tx, 100) == 100);
		s_NvicEn = en;
		assert(WaitFlag(&s_Done) && s_Count == 100 && memcmp(s_SlaveRx, tx, 100) == 0);
		Settle();
		BusFree();
	}

	// Two frames with the slave interrupt held off: the first is reported
	// with its bytes, the second, begun before the first was taken, as -1.
	// Then a frame begun while the slave prepares is -1, the next is good.
	for (int sm = 0; sm < 2; sm++)
	{
		InitPair(MasterCfg(false, false, 0), SlaveCfg(sm == 1, 0));
		s_SlaveRxSize = 129;
		s_SlaveTxSize = 129;
		DeviceIntrfReset(&s_Slave.DevIntrf);
		uint8_t a[4] = { 0x10, 0x11, 0x12, 0x13 };
		uint8_t b[4] = { 0x20, 0x21, 0x22, 0x23 };
		uint32_t irq = 1UL << (Lpc546xxFlexcommIrqNo(SLAVE_FC) & 31);
		uint32_t dirq = 1UL << DMA0_IRQn;
		uint32_t dmarx = s_S[SLAVE_FC].FifoCfg & SPI_FIFOCFG_DMARX_MASK;
		unsigned before = s_Completions;
		s_NvicEn &= ~(irq | dirq);
		s_S[SLAVE_FC].FifoCfg &= ~SPI_FIFOCFG_DMARX_MASK;
		assert(SPITx(&s_Master, 0, a, 4) == 4);
		assert(SPITx(&s_Master, 0, b, 4) == 4);
		s_S[SLAVE_FC].FifoCfg |= dmarx;
		s_NvicEn |= irq | dirq;
		Settle();
		assert(s_Completions == before + 2 && s_CountHist[(before + 1) & 7] == -1);
		if (sm == 0)
		{
			// Interrupt: SOT splits the frames, the first is reported with
			// its bytes
			assert(s_CountHist[before & 7] == 4 && memcmp(s_SlaveRx, a, 4) == 0);
		}

		// SSEL asserted while the slave is not ready yet: that frame is -1
		before = s_Completions;
		s_NvicEn &= ~irq;
		s_Gpio.CLR[CS_PORT] = 1UL << CS_PIN;
		DeviceIntrfReset(&s_Slave.DevIntrf);
		s_NvicEn |= irq;
		s_Gpio.SET[CS_PORT] = 1UL << CS_PIN;
		Settle();
		assert(s_Completions == before + 1 && s_Count == -1);
		assert(SPITx(&s_Master, 0, a, 4) == 4);
		Settle();
		assert(s_Completions == before + 2 && s_Count == 4);
		assert(Frame(16, 0x41));
	}

	// SSEL asserted and deasserted with no clock: not reported
	{
		InitPair(MasterCfg(false, false, 0), SlaveCfg(false, 0));
		unsigned before = s_Completions;
		unsigned arms = s_Arms;
		s_Gpio.CLR[CS_PORT] = 1UL << CS_PIN;
		Settle();
		s_Gpio.SET[CS_PORT] = 1UL << CS_PIN;
		Settle();
		assert(s_Completions == before && s_Arms == arms);
		assert(Frame(8, 0x51));
	}

	// PowerOff gives everything back
	InitPair(MasterCfg(true, true, 0), SlaveCfg(true, 0));
	assert(s_DmaRegs.CTRL != 0);
	DeviceIntrfPowerOff(&s_Master.DevIntrf);
	DeviceIntrfPowerOff(&s_Slave.DevIntrf);
	assert(s_DmaRegs.CTRL == 0 && (s_NvicEn & (1UL << DMA0_IRQn)) == 0);
	assert((s_NvicEn & (1UL << (Lpc546xxFlexcommIrqNo(MASTER_FC) & 31))) == 0);
	assert((s_NvicEn & (1UL << (Lpc546xxFlexcommIrqNo(SLAVE_FC) & 31))) == 0);
	assert(s_S[SLAVE_FC].IntEn == 0);

	// Back to polling: the DMA channels are given back
	InitPair(MasterCfg(false, false, 0), SlaveCfg(false, 0));
	assert(s_DmaRegs.CTRL == 0);
	assert(Frame(16, 0x31));

	printf("lpc546xx spi bus: long DMA frames, short buffers, read, copy, DMA error, release: PASS\n");
}

// The DMA descriptors hold 32 bit addresses. The test runs on a stack in the
// static data, below 4 GB with -no-pie like all the other test memory.
alignas(64) static uint8_t s_Stack[1 << 20];

static void *TestMain(void *)
{
	memset(s_SysconMem, 0, sizeof(s_SysconMem));
	DmaModelInit();
	// Chip select high
	s_Gpio.PIN[CS_PORT] = 1UL << CS_PIN;

	for (int sm = 0; sm < 2; sm++)
	{
		for (int mm = 0; mm < 4; mm++)
		{
			RunCombo(mm, sm);
		}
	}
	ExtraTests();

	return nullptr;
}

int main()
{
	pthread_attr_t attr;
	pthread_t t;

	assert((uintptr_t)s_Stack + sizeof(s_Stack) < 0x100000000ULL);
	pthread_attr_init(&attr);
	pthread_attr_setstack(&attr, s_Stack, sizeof(s_Stack));
	assert(pthread_create(&t, &attr, TestMain, nullptr) == 0);
	pthread_join(t, nullptr);

	return 0;
}
