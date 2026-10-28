// Host regression of the LPC546xx Flexcomm I2C master and slave, in every
// master mode (polling, interrupt, DMA, DMA with interrupt) and slave mode
// (interrupt, DMA), with the sequence of exemples/i2c/i2c_master_slave.cpp.
//
// Bus model: the Flexcomms in the I2C function share one bus. A master
// START sends the address byte, a matching slave address (SLVADR0 to 3) makes
// the slave pending in the address state and the master waits, as with the
// clock stretched, until the slave answers with SLVCONTINUE. Data bytes go the
// same way, each one waits for the side that has to answer. STOP, and the
// NACK of the master after its last read byte, deselect the slave (SLVDESEL).
// A repeated START to the selected slave keeps it selected.
//
// MSTDMA and SLVDMA: the master and slave pending states of data are served
// by the DMA request of the channel (Flexcomm TX channel for the master, RX
// channel for the slave) and hidden from MSTPENDING and SLVPENDING. A DMA
// write of MSTDAT sends the byte, a DMA read acknowledges it and continues, as
// the MSTDMA and SLVDMA descriptions of the device header say.
//
// Time passes on each STAT read: the DMA moves one value per channel and the
// enabled interrupts run when they are not masked.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <pthread.h>

#include "LPC546xx.h"
#include "dma_model.h"

// Flexcomm PSELID, the present bits are read only
struct Psel {
	uint32_t value = 0x00000070U;
	operator uint32_t() const { return value; }
	Psel &operator=(uint32_t v) { value = (value & ~0xFU) | (v & 0xFU); return *this; }
};

struct FcRegs {
	uint8_t Reserved[0xFF8];
	Psel PSELID;
	uint32_t PID;
};

#define MST_IDLE	0
#define MST_RXRDY	1
#define MST_TXRDY	2
#define MST_NACKADR	3
#define MST_NACKDAT	4
#define SLV_ADDR	0
#define SLV_RX		1
#define SLV_TX		2

struct I2cM {
	// Master
	bool mEn = false;
	bool mPending = true;		// Master needs service or is idle
	int mState = MST_IDLE;
	bool mDma = false;
	uint8_t mDat = 0;
	bool mOwn = false;			// START sent, no STOP yet
	int mTgt = -1;				// Slave addressed
	bool mRead = false;
	// Slave
	bool sEn = false;
	bool sSel = false;
	bool sDesel = false;
	bool sPending = false;
	int sState = SLV_ADDR;
	bool sDma = false;
	uint8_t sDat = 0;
	int sIdx = 0;
	int sMst = -1;				// Master of the request
	uint32_t IntEn = 0;
	std::string Log;			// Bus seen by this master
	int AutoNack = -1;			// Model slave, not a driver: NACK of this data byte
	int AutoCnt = 0;
};

static I2cM s_M[10];
static int RegNo(const void *p);

struct I2cCfgR { operator uint32_t() const; I2cCfgR &operator=(uint32_t v); };
struct I2cStatR { operator uint32_t() const; I2cStatR &operator=(uint32_t v); };
struct I2cIntSetR { operator uint32_t() const; I2cIntSetR &operator=(uint32_t v); };
struct I2cIntClrR { I2cIntClrR &operator=(uint32_t v); };
struct I2cMstCtlR { operator uint32_t() const; I2cMstCtlR &operator=(uint32_t v); };
struct I2cMstDatR { operator uint32_t() const; I2cMstDatR &operator=(uint32_t v); };
struct I2cSlvCtlR { operator uint32_t() const; I2cSlvCtlR &operator=(uint32_t v); };
struct I2cSlvDatR { operator uint32_t() const; I2cSlvDatR &operator=(uint32_t v); };

struct I2cRegs {
	I2cCfgR CFG;
	I2cStatR STAT;
	I2cIntSetR INTENSET;
	I2cIntClrR INTENCLR;
	uint32_t TIMEOUT = 0, CLKDIV = 0, INTSTAT = 0;
	I2cMstCtlR MSTCTL;
	uint32_t MSTTIME = 0;
	I2cMstDatR MSTDAT;
	I2cSlvCtlR SLVCTL;
	I2cSlvDatR SLVDAT;
	uint32_t SLVADR[4] = {};
	uint32_t SLVQUAL0 = 0;
};

static I2cRegs s_Regs[10];

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

static void Tick(void);

// Slave with an enabled address matching Addr
static int FindSlave(int Self, int Addr, int *pIdx)
{
	for (int i = 0; i < 10; i++)
	{
		if (i == Self || s_M[i].sEn == false)
		{
			continue;
		}
		for (int k = 0; k < 4; k++)
		{
			uint32_t a = s_Regs[i].SLVADR[k];
			if ((a & I2C_SLVADR_SADISABLE_MASK) == 0 && (int)((a >> 1) & 0x7F) == Addr)
			{
				*pIdx = k;
				return i;
			}
		}
	}
	return -1;
}

static void SlvDeselect(int t)
{
	I2cM &s = s_M[t];

	if (s.sSel)
	{
		s.sSel = false;
		s.sDesel = true;
	}
	s.sPending = false;
	s.sMst = -1;
}

static void Hex(std::string &l, uint8_t b)
{
	char t[4];
	snprintf(t, sizeof(t), "%02X", b);
	l += t;
}

// The master sends MSTDAT, the slave receives it
static void MstSendByte(int No)
{
	I2cM &m = s_M[No];

	assert(m.mPending && m.mState == MST_TXRDY && m.mTgt >= 0);
	Hex(m.Log, m.mDat);
	I2cM &s = s_M[m.mTgt];
	s.sPending = true;
	s.sState = SLV_RX;
	s.sDat = m.mDat;
	m.mPending = false;
}

// The master acknowledges the byte read, the slave sends the next one
static void MstAckRead(int No)
{
	I2cM &m = s_M[No];

	assert(m.mPending && m.mState == MST_RXRDY && m.mTgt >= 0);
	m.Log += "A";
	I2cM &s = s_M[m.mTgt];
	s.sPending = true;
	s.sState = SLV_TX;
	m.mPending = false;
}

static void MstCtl(int No, uint32_t v)
{
	I2cM &m = s_M[No];

	m.mDma = (v & I2C_MSTCTL_MSTDMA_MASK) != 0;
	if (m.mEn == false)
	{
		return;
	}

	if (v & I2C_MSTCTL_MSTSTART_MASK)
	{
		assert(m.mPending);
		if (m.mOwn && m.mState == MST_RXRDY)
		{
			// NACK of the last byte read
			m.Log += "N";
			if (m.mTgt >= 0)
			{
				SlvDeselect(m.mTgt);
			}
		}
		m.Log += m.mOwn ? "Sr" : "S";
		uint8_t a = m.mDat;
		m.Log += (a & 1) ? "R" : "W";
		int idx = 0;
		int t = FindSlave(No, a >> 1, &idx);
		if (m.mTgt >= 0 && m.mTgt != t)
		{
			SlvDeselect(m.mTgt);
		}
		m.mOwn = true;
		m.mRead = a & 1;
		m.mTgt = t;
		if (t < 0)
		{
			m.Log += "n";
			m.mState = MST_NACKADR;
			m.mPending = true;
		}
		else
		{
			I2cM &s = s_M[t];
			s.sMst = No;
			s.sPending = true;
			s.sState = SLV_ADDR;
			s.sDat = a;
			s.sIdx = idx;
			m.mPending = false;
		}
	}
	else if (v & I2C_MSTCTL_MSTSTOP_MASK)
	{
		assert(m.mPending);
		if (m.mOwn && m.mState == MST_RXRDY)
		{
			m.Log += "N";
		}
		m.Log += "P";
		if (m.mTgt >= 0)
		{
			SlvDeselect(m.mTgt);
		}
		m.mTgt = -1;
		m.mOwn = false;
		m.mState = MST_IDLE;
		m.mPending = true;
	}
	else if (v & I2C_MSTCTL_MSTCONTINUE_MASK)
	{
		assert(m.mPending && m.mOwn);
		if (m.mState == MST_TXRDY)
		{
			MstSendByte(No);
		}
		else if (m.mState == MST_RXRDY)
		{
			MstAckRead(No);
		}
	}
}

// The slave answers the pending state
static void SlvContinue(int No)
{
	I2cM &s = s_M[No];

	assert(s.sPending && s.sMst >= 0);
	I2cM &m = s_M[s.sMst];
	s.sPending = false;
	switch (s.sState)
	{
		case SLV_ADDR:
			s.sSel = true;
			m.Log += "a";
			if (m.mRead)
			{
				s.sPending = true;
				s.sState = SLV_TX;
			}
			else
			{
				m.mState = MST_TXRDY;
				m.mPending = true;
			}
			break;
		case SLV_RX:
			m.mState = MST_TXRDY;
			m.mPending = true;
			break;
		case SLV_TX:
			Hex(m.Log, s.sDat);
			m.mDat = s.sDat;
			m.mState = MST_RXRDY;
			m.mPending = true;
			break;
	}
}

static void SlvCtl(int No, uint32_t v)
{
	I2cM &s = s_M[No];

	s.sDma = (v & I2C_SLVCTL_SLVDMA_MASK) != 0;
	if (v & I2C_SLVCTL_SLVCONTINUE_MASK)
	{
		// SLVCONTINUE is not set unless SLVPENDING
		assert(s.sPending);
		SlvContinue(No);
	}
	else if (v & I2C_SLVCTL_SLVNACK_MASK)
	{
		assert(s.sPending);
		I2cM &m = s_M[s.sMst];
		m.Log += "n";
		m.mState = s.sState == SLV_ADDR ? MST_NACKADR : MST_NACKDAT;
		m.mPending = true;
		s.sPending = false;
	}
}

static bool MstVisible(const I2cM &m)
{
	return m.mPending && !(m.mDma && (m.mState == MST_RXRDY || m.mState == MST_TXRDY));
}

static bool SlvVisible(const I2cM &s)
{
	return s.sPending && !(s.sDma && (s.sState == SLV_RX || s.sState == SLV_TX));
}

static uint32_t StatValue(int No)
{
	const I2cM &m = s_M[No];
	uint32_t v = 0;

	if (m.mEn)
	{
		v |= (MstVisible(m) ? I2C_STAT_MSTPENDING_MASK : 0U) | ((uint32_t)m.mState << I2C_STAT_MSTSTATE_SHIFT);
	}
	if (m.sEn)
	{
		v |= (SlvVisible(m) ? I2C_STAT_SLVPENDING_MASK : 0U) | ((uint32_t)m.sState << I2C_STAT_SLVSTATE_SHIFT) |
			 ((uint32_t)m.sIdx << I2C_STAT_SLVIDX_SHIFT) | (m.sSel ? I2C_STAT_SLVSEL_MASK : 0U);
	}
	v |= m.sDesel ? I2C_STAT_SLVDESEL_MASK : 0U;
	return v;
}

I2cCfgR::operator uint32_t() const
{
	const I2cM &m = s_M[RegNo(this)];
	return (m.mEn ? I2C_CFG_MSTEN_MASK : 0U) | (m.sEn ? I2C_CFG_SLVEN_MASK : 0U);
}
I2cCfgR &I2cCfgR::operator=(uint32_t v)
{
	I2cM &m = s_M[RegNo(this)];
	m.mEn = (v & I2C_CFG_MSTEN_MASK) != 0;
	m.sEn = (v & I2C_CFG_SLVEN_MASK) != 0;
	if (!m.mEn)
	{
		m.mPending = true;
		m.mState = MST_IDLE;
		m.mOwn = false;
		m.mTgt = -1;
	}
	if (!m.sEn)
	{
		m.sPending = false;
		m.sSel = false;
	}
	return *this;
}
I2cStatR::operator uint32_t() const
{
	int no = RegNo(this);
	Tick();
	return StatValue(no);
}
I2cStatR &I2cStatR::operator=(uint32_t v)
{
	if (v & I2C_STAT_SLVDESEL_MASK)
	{
		s_M[RegNo(this)].sDesel = false;
	}
	return *this;
}
I2cIntSetR::operator uint32_t() const { return s_M[RegNo(this)].IntEn; }
I2cIntSetR &I2cIntSetR::operator=(uint32_t v) { s_M[RegNo(this)].IntEn |= v; return *this; }
I2cIntClrR &I2cIntClrR::operator=(uint32_t v) { s_M[RegNo(this)].IntEn &= ~v; return *this; }
I2cMstCtlR::operator uint32_t() const { return s_M[RegNo(this)].mDma ? I2C_MSTCTL_MSTDMA_MASK : 0U; }
I2cMstCtlR &I2cMstCtlR::operator=(uint32_t v) { MstCtl(RegNo(this), v); return *this; }
I2cMstDatR::operator uint32_t() const { return s_M[RegNo(this)].mDat; }
I2cMstDatR &I2cMstDatR::operator=(uint32_t v) { s_M[RegNo(this)].mDat = (uint8_t)v; return *this; }
I2cSlvCtlR::operator uint32_t() const { return s_M[RegNo(this)].sDma ? I2C_SLVCTL_SLVDMA_MASK : 0U; }
I2cSlvCtlR &I2cSlvCtlR::operator=(uint32_t v) { SlvCtl(RegNo(this), v); return *this; }
I2cSlvDatR::operator uint32_t() const { return s_M[RegNo(this)].sDat; }
I2cSlvDatR &I2cSlvDatR::operator=(uint32_t v) { s_M[RegNo(this)].sDat = (uint8_t)v; return *this; }

alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
static FcRegs s_Fc[10];
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
#undef I2C0
#undef I2C1
#undef I2C2
#undef I2C3
#undef I2C4
#undef I2C5
#undef I2C6
#undef I2C7
#undef I2C8
#undef I2C9
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
#define I2C_Type I2cRegs
#define I2C0 (&s_Regs[0])
#define I2C1 (&s_Regs[1])
#define I2C2 (&s_Regs[2])
#define I2C3 (&s_Regs[3])
#define I2C4 (&s_Regs[4])
#define I2C5 (&s_Regs[5])
#define I2C6 (&s_Regs[6])
#define I2C7 (&s_Regs[7])
#define I2C8 (&s_Regs[8])
#define I2C9 (&s_Regs[9])
#define DMA_Type DmaRegs
#define DMA0 (&s_DmaRegs)

#include "dma_lpc546xx.h"

static bool IsI2c(int No)
{
	return (s_Fc[No].PSELID.value & 0xFU) == 3U;
}

// Master request on the Flexcomm TX channel, slave request on the RX one
static bool DmaReqOn(int Ch)
{
	for (int i = 0; i < 10; i++)
	{
		if (!IsI2c(i))
		{
			continue;
		}
		const I2cM &m = s_M[i];
		if (Ch == LPC546XX_DMA_CH_FC_TX(i))
		{
			return m.mEn && m.mPending && m.mDma && (m.mState == MST_TXRDY || m.mState == MST_RXRDY);
		}
		if (Ch == LPC546XX_DMA_CH_FC_RX(i))
		{
			return m.sEn && m.sPending && m.sDma && (m.sState == SLV_RX || m.sState == SLV_TX);
		}
	}
	return false;
}

static bool DmaPeriphRead(uint32_t Addr, uint32_t *pVal)
{
	for (int i = 0; i < 10; i++)
	{
		if (Addr == (uint32_t)(uintptr_t)&s_Regs[i].MSTDAT)
		{
			// The master acknowledges the byte and continues
			assert(s_M[i].mDma && s_M[i].mState == MST_RXRDY);
			*pVal = s_M[i].mDat;
			MstAckRead(i);
			return true;
		}
		if (Addr == (uint32_t)(uintptr_t)&s_Regs[i].SLVDAT)
		{
			assert(s_M[i].sDma && s_M[i].sState == SLV_RX);
			*pVal = s_M[i].sDat;
			SlvContinue(i);
			return true;
		}
	}
	return false;
}

static bool DmaPeriphWrite(uint32_t Addr, uint32_t Val)
{
	for (int i = 0; i < 10; i++)
	{
		if (Addr == (uint32_t)(uintptr_t)&s_Regs[i].MSTDAT)
		{
			assert(s_M[i].mDma && s_M[i].mState == MST_TXRDY);
			s_M[i].mDat = (uint8_t)Val;
			MstSendByte(i);
			return true;
		}
		if (Addr == (uint32_t)(uintptr_t)&s_Regs[i].SLVDAT)
		{
			assert(s_M[i].sDma && s_M[i].sState == SLV_TX);
			s_M[i].sDat = (uint8_t)Val;
			SlvContinue(i);
			return true;
		}
	}
	return false;
}

#include "coredev/iopincfg.h"
#include "coredev/i2c.h"

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)PortNo; (void)PinNo; (void)PinOp; (void)Dir; (void)Resistor; (void)Type;
}
void IOPinDisable(int PortNo, int PinNo) { (void)PortNo; (void)PinNo; }
void I2CBusReset(I2CDev_t * const pDev) { (void)pDev; }
void I2CSetReadRqstData(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pData, int DataLen)
{
	pDev->pRRData[SlaveIdx] = pData;
	pDev->RRDataLen[SlaveIdx] = DataLen;
}
void I2CSetWriteRqstBuffer(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pBuff, int BuffLen)
{
	pDev->pTRBuff[SlaveIdx] = pBuff;
	pDev->TRBuffLen[SlaveIdx] = BuffLen;
}

#include "../../../src/coredev/shared_intrf.cpp"
#include "../../../ARM/NXP/LPC546xx/src/flexcomm_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/dma_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/i2c_lpc546xx.cpp"
#include "../../../src/device_intrf.cpp"

static void (* const s_FcVector[10])(void) = {
	FLEXCOMM0_IRQHandler, FLEXCOMM1_IRQHandler, FLEXCOMM2_IRQHandler, FLEXCOMM3_IRQHandler,
	FLEXCOMM4_IRQHandler, FLEXCOMM5_IRQHandler, FLEXCOMM6_IRQHandler, FLEXCOMM7_IRQHandler,
	FLEXCOMM8_IRQHandler, FLEXCOMM9_IRQHandler,
};

static bool FcIrqPending(int No)
{
	uint32_t mask = I2C_INTENSET_MSTPENDINGEN_MASK | I2C_INTENSET_MSTARBLOSSEN_MASK |
					I2C_INTENSET_MSTSTSTPERREN_MASK | I2C_INTENSET_SLVPENDINGEN_MASK | I2C_INTENSET_SLVDESELEN_MASK;
	return (s_NvicEn & (1UL << (Lpc546xxFlexcommIrqNo(No) & 31))) && (StatValue(No) & s_M[No].IntEn & mask);
}

static int s_InTick = 0;
static unsigned long s_Ticks = 0;

// A Flexcomm with AutoNack set answers by itself, as a slave that takes
// AutoNack bytes then refuses the next one
static void AutoSlave(void)
{
	for (int i = 0; i < 10; i++)
	{
		I2cM &s = s_M[i];

		if (s.AutoNack < 0 || s.sPending == false)
		{
			continue;
		}
		if (s.sState == SLV_ADDR)
		{
			s.AutoCnt = 0;
			SlvCtl(i, I2C_SLVCTL_SLVCONTINUE_MASK);
		}
		else if (s.sState == SLV_RX)
		{
			SlvCtl(i, s.AutoCnt++ == s.AutoNack ? I2C_SLVCTL_SLVNACK_MASK : I2C_SLVCTL_SLVCONTINUE_MASK);
		}
	}
}

static void Tick(void)
{
	if (s_InTick)
	{
		return;
	}
	s_InTick = 1;
	s_Ticks++;
	AutoSlave();
	for (int r = 0; r < 4; r++)
	{
		DmaStep(false);
		if (g_IrqMasked == 0)
		{
			if (DmaIrqPending())
			{
				DMA0_IRQHandler();
			}
			for (int i = 0; i < 10; i++)
			{
				if (g_IrqMasked == 0 && IsI2c(i) && FcIrqPending(i))
				{
					s_FcVector[i]();
				}
			}
		}
	}
	s_InTick = 0;
}

// Slave of exemples/i2c/i2c_master_slave.cpp
#define SLAVE_ADDR		0x22
#define NACK_ADDR		0x23
#define BUFF_SIZE		20
#define SLAVE_MAX_XFER	255
#define LONG_LEN		1500
#define WRITE_TEST_CMD	0xFF

static I2CDev_t s_Master;
static I2CDev_t s_Slave;
static uint8_t s_ReadRqstData[BUFF_SIZE];
static uint8_t s_WriteRqstData[LONG_LEN + 8];
static int s_WriteRqstSize = SLAVE_MAX_XFER;
static int s_SlaveWriteLen = 0;
static bool s_bWriteRqst = false;
static int s_Offset = 0;
static bool s_MasterCompleted = false;
static int s_MasterCount = 0;
static int s_MasterEvt = 0;
static int s_SlaveCompleted = 0;

static int MasterEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	(void)pDev; (void)pBuffer;
	if (EvtId == DEVINTRF_EVT_COMPLETED)
	{
		// The bus is released before the completion
		assert(s_Master.DevIntrf.bTxReady);
		s_MasterCount = Len;
		s_MasterCompleted = true;
		s_MasterEvt++;
	}
	return 0;
}

static int SlaveEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	(void)pDev; (void)pBuffer;
	switch (EvtId)
	{
		case DEVINTRF_EVT_READ_RQST:
			if (s_bWriteRqst)
			{
				if (Len > 0)
				{
					s_Offset = s_WriteRqstData[0];
				}
				s_bWriteRqst = false;
			}
			I2CSetReadRqstData(&s_Slave, 0, &s_ReadRqstData[s_Offset], BUFF_SIZE - s_Offset);
			break;
		case DEVINTRF_EVT_WRITE_RQST:
			s_bWriteRqst = true;
			I2CSetWriteRqstBuffer(&s_Slave, 0, s_WriteRqstData, s_WriteRqstSize);
			break;
		case DEVINTRF_EVT_COMPLETED:
			s_SlaveCompleted++;
			if (s_bWriteRqst == false)
			{
				s_Offset += Len;
				if (s_Offset >= BUFF_SIZE)
				{
					s_Offset = 0;
				}
			}
			else
			{
				s_SlaveWriteLen = Len;
				if (Len > 1 && s_WriteRqstData[0] < BUFF_SIZE)
				{
					int n = Len - 1;
					if (n > BUFF_SIZE - s_WriteRqstData[0])
					{
						n = BUFF_SIZE - s_WriteRqstData[0];
					}
					memcpy(&s_ReadRqstData[s_WriteRqstData[0]], &s_WriteRqstData[1], n);
				}
			}
			s_bWriteRqst = false;
			break;
		default:
			break;
	}
	return BUFF_SIZE;
}

static int WaitMaster(void)
{
	for (int i = 0; i < 2000000 && !s_MasterCompleted; i++)
	{
		Tick();
	}
	return s_MasterCompleted ? s_MasterCount : -1;
}

static void Settle(void)
{
	for (int i = 0; i < 200; i++)
	{
		Tick();
	}
}

static int MasterWrite(uint8_t Addr, uint8_t Cmd, const uint8_t *pData, int Len)
{
	s_MasterCompleted = false;
	int c = I2CWrite(&s_Master, Addr, &Cmd, 1, (uint8_t *)pData, Len);
	if (s_Master.DevIntrf.bIntEn)
	{
		assert(c == 0);
		int total = Addr == NACK_ADDR ? 0 : WaitMaster();
		c = total >= 1 ? total - 1 : 0;
	}
	Settle();
	assert(!atomic_flag_test_and_set(&s_Master.DevIntrf.bBusy));
	atomic_flag_clear(&s_Master.DevIntrf.bBusy);
	return c;
}

static int MasterRead(uint8_t Offset, uint8_t *pBuff, int Len)
{
	s_MasterCompleted = false;
	int c = I2CRead(&s_Master, SLAVE_ADDR, &Offset, 1, pBuff, Len);
	if (c < 0)
	{
		c = WaitMaster();
	}
	Settle();
	assert(!atomic_flag_test_and_set(&s_Master.DevIntrf.bBusy));
	atomic_flag_clear(&s_Master.DevIntrf.bBusy);
	return c;
}

static int MasterRx(uint8_t *pBuff, int Len)
{
	s_MasterCompleted = false;
	int c = I2CRx(&s_Master, SLAVE_ADDR, pBuff, Len);
	if (c < 0)
	{
		c = WaitMaster();
	}
	Settle();
	assert(!atomic_flag_test_and_set(&s_Master.DevIntrf.bBusy));
	atomic_flag_clear(&s_Master.DevIntrf.bBusy);
	return c;
}

static const IOPinCfg_t s_Pins[] = {
	{3, 23, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{3, 24, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

static uint8_t s_WriteTestData[LONG_LEN];

static bool WriteTest(int Len)
{
	for (int i = 0; i < Len; i++)
	{
		s_WriteTestData[i] = (uint8_t)(i * 7 + 3);
	}
	memset(s_WriteRqstData, 0, sizeof(s_WriteRqstData));
	s_SlaveWriteLen = 0;
	int c = MasterWrite(SLAVE_ADDR, WRITE_TEST_CMD, Len > 0 ? s_WriteTestData : NULL, Len);
	int got = s_SlaveWriteLen - 1;
	return c == Len && got == Len && s_WriteRqstData[0] == WRITE_TEST_CMD &&
		   memcmp(&s_WriteRqstData[1], s_WriteTestData, Len) == 0;
}

static const char *ModeName(bool bDma, bool bInt)
{
	return bDma ? (bInt ? "DMA+interrupt" : "DMA") : (bInt ? "interrupt" : "polling");
}

static void RunCombo(int Mm, int Sm)
{
	I2CCfg_t mc = {};
	I2CCfg_t sc = {};

	mc.DevNo = 1;
	mc.Mode = I2CMODE_MASTER;
	mc.pIOPinMap = s_Pins;
	mc.NbIOPins = 2;
	mc.Rate = 100000;
	mc.MaxRetry = 5;
	mc.bDmaEn = Mm >= 2;
	mc.bIntEn = Mm == 1 || Mm == 3;
	mc.IntPrio = 7;
	mc.EvtCB = MasterEvt;

	sc = mc;
	sc.DevNo = 2;
	sc.Mode = I2CMODE_SLAVE;
	sc.NbSlaveAddr = 1;
	sc.SlaveAddr[0] = SLAVE_ADDR;
	sc.bDmaEn = Sm == 1;
	sc.bIntEn = true;
	sc.EvtCB = SlaveEvt;

	assert(I2CInit(&s_Master, &mc) && I2CInit(&s_Slave, &sc));
	assert(s_Slave.DevIntrf.bIntEn && s_Slave.DevIntrf.bDma == sc.bDmaEn);
	assert((s_Regs[2].SLVADR[0] >> 1) == SLAVE_ADDR && (s_Regs[2].SLVADR[1] & I2C_SLVADR_SADISABLE_MASK));
	assert(s_M[2].sEn && !s_M[2].mEn && s_M[1].mEn && !s_M[1].sEn);

	for (int i = 0; i < BUFF_SIZE; i++)
	{
		s_ReadRqstData[i] = (uint8_t)i;
	}
	memset(s_WriteRqstData, 0, sizeof(s_WriteRqstData));
	s_Offset = 0;
	s_bWriteRqst = false;
	s_WriteRqstSize = SLAVE_MAX_XFER;
	int dmavalues = s_DmaM.Values;
	int evts = s_MasterEvt;

	const uint8_t wr[8] = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7 };
	uint8_t buff[BUFF_SIZE];
	const uint8_t offset = 4;

	// Register write of 8 bytes at offset 4
	s_M[1].Log.clear();
	assert(MasterWrite(SLAVE_ADDR, offset, wr, 8) == 8);
	assert(s_M[1].Log == "SWa04A0A1A2A3A4A5A6A7P");
	assert(memcmp(&s_ReadRqstData[4], wr, 8) == 0);

	// Register reads of 1, 2 and 8 bytes
	const int readlen[] = { 1, 2, 8 };
	for (int i = 0; i < 3; i++)
	{
		memset(buff, 0xFF, sizeof(buff));
		s_M[1].Log.clear();
		assert(MasterRead(offset, buff, readlen[i]) == readlen[i]);
		assert(memcmp(buff, wr, readlen[i]) == 0);
	}
	assert(s_M[1].Log == "SWa04SrRaA0AA1AA2AA3AA4AA5AA6AA7NP");

	// Receives without a command continue after the last read
	memset(buff, 0xFF, sizeof(buff));
	assert(MasterRx(buff, 4) == 4 && memcmp(buff, &s_ReadRqstData[12], 4) == 0);
	memset(buff, 0xFF, sizeof(buff));
	s_M[1].Log.clear();
	assert(MasterRx(buff, 1) == 1 && buff[0] == s_ReadRqstData[16]);
	assert(s_M[1].Log == "SRa10NP");

	// Writes of 0, 1, 2, 8 and 254 data bytes
	const int writelen[] = { 0, 1, 2, 8, SLAVE_MAX_XFER - 1 };
	for (int i = 0; i < 5; i++)
	{
		assert(WriteTest(writelen[i]));
	}

	// Address NACK, then a register read
	s_M[1].Log.clear();
	assert(MasterWrite(NACK_ADDR, offset, wr, 8) == 0);
	assert(s_M[1].Log == "SWnPSWnPSWnPSWnPSWnPSWnP");
	memset(buff, 0xFF, sizeof(buff));
	assert(MasterRead(offset, buff, 8) == 8 && memcmp(buff, wr, 8) == 0);

	// A write longer than the slave buffer: acknowledged and dropped past it
	s_WriteRqstSize = 100;
	for (int i = 0; i < 200; i++)
	{
		s_WriteTestData[i] = (uint8_t)(i * 5 + 1);
	}
	s_SlaveWriteLen = 0;
	assert(MasterWrite(SLAVE_ADDR, WRITE_TEST_CMD, s_WriteTestData, 199) == 199);
	assert(s_SlaveWriteLen == 100 && memcmp(&s_WriteRqstData[1], s_WriteTestData, 99) == 0);

	// A long write, more than one DMA block of both sides
	s_WriteRqstSize = LONG_LEN + 1;
	assert(WriteTest(LONG_LEN));

	// Interrupt and DMA: the data is copied, the caller may reuse its buffer
	// once TxData returns
	if (mc.bIntEn)
	{
		uint8_t tx[9] = { WRITE_TEST_CMD, 1, 2, 3, 4, 5, 6, 7, 8 };
		s_WriteRqstSize = 20;
		s_SlaveWriteLen = 0;
		s_MasterCompleted = false;
		assert(I2CTx(&s_Master, SLAVE_ADDR, tx, 9) == -1);
		memset(tx, 0xEE, sizeof(tx));
		assert(WaitMaster() == 9);
		Settle();
		assert(s_SlaveWriteLen == 9 && s_WriteRqstData[1] == 1 && s_WriteRqstData[8] == 8);
	}

	// A data NACK from a slave on the first data byte, then on the third.
	// The slave at 0x40 is the model itself, Flexcomm 3 is not initialized.
	s_M[3].sEn = true;
	s_Regs[3].SLVADR[0] = 0x40U << 1;
	for (int k = 1; k < 4; k++)
	{
		s_Regs[3].SLVADR[k] = I2C_SLVADR_SADISABLE_MASK;
	}
	s_M[3].AutoNack = 0;
	for (int k = 0; k < 2; k++)
	{
		uint8_t tx[5] = { 0x10, 0x11, 0x12, 0x13, 0x14 };
		s_MasterCompleted = false;
		s_M[1].Log.clear();
		int c = I2CTx(&s_Master, 0x40, tx, 5);
		if (c < 0)
		{
			c = WaitMaster();
		}
		Settle();
		assert(c == s_M[3].AutoNack);
		if (k == 0)
		{
			// Nothing acknowledged: a transfer that returns its count is
			// tried again by DeviceIntrfTx, MaxRetry more times
			std::string one = "SWa10nP";
			std::string all = one;
			for (int r = 0; r < 5 && !mc.bIntEn; r++)
			{
				all += one;
			}
			assert(s_M[1].Log == all);
		}
		else
		{
			assert(s_M[1].Log == "SWa101112nP");
		}
		s_M[3].AutoNack = 2;
	}
	s_M[3].AutoNack = -1;
	s_M[3].sEn = false;

	// A read past the slave data gets 0xFF, not counted by the slave
	s_Offset = 0;
	s_SlaveCompleted = 0;
	uint8_t big[30];
	memset(big, 0, sizeof(big));
	int c = MasterRx(big, 30);
	assert(c == 30 && memcmp(big, s_ReadRqstData, BUFF_SIZE) == 0 && big[BUFF_SIZE] == 0xFF && big[29] == 0xFF);
	assert(s_SlaveCompleted == 1 && s_Offset == 0);

	if (mc.bDmaEn || sc.bDmaEn)
	{
		assert(s_DmaM.Values > dmavalues);
	}
	else
	{
		assert(s_DmaM.Values == dmavalues);
	}
	if (mc.bIntEn)
	{
		assert(s_MasterEvt > evts);
	}
	else
	{
		assert(s_MasterEvt == evts);
	}

	printf("lpc546xx i2c master %s, slave %s: PASS\n", ModeName(mc.bDmaEn, mc.bIntEn), sc.bDmaEn ? "DMA" : "interrupt");
}

// The DMA descriptors hold 32 bit addresses. The test runs on a stack in the
// static data, below 4 GB with -no-pie like all the other test memory, so
// that DeviceIntrfWrite's buffer on the stack can be a DMA source.
alignas(64) static uint8_t s_Stack[1 << 20];

static void *TestMain(void *)
{
	memset(s_SysconMem, 0, sizeof(s_SysconMem));
	DmaModelInit();

	for (int sm = 0; sm < 2; sm++)
	{
		for (int mm = 0; mm < 4; mm++)
		{
			RunCombo(mm, sm);
		}
	}

	// A DMA error ends the master transfer with the bytes moved
	I2CCfg_t mc = {};
	mc.DevNo = 1;
	mc.Mode = I2CMODE_MASTER;
	mc.pIOPinMap = s_Pins;
	mc.NbIOPins = 2;
	mc.Rate = 400000;
	mc.bDmaEn = true;
	mc.EvtCB = MasterEvt;
	assert(I2CInit(&s_Master, &mc));
	s_WriteRqstSize = 50;
	s_DmaM.ErrCh = LPC546XX_DMA_CH_FC_TX(1);
	uint8_t d[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
	s_M[1].Log.clear();
	// The retry of DeviceIntrfTx sends it again
	assert(I2CTx(&s_Master, SLAVE_ADDR, d, 10) == 10);
	assert(s_M[1].Log == "SWaPSWa0102030405060708090AP");
	Settle();

	// Polling: a write, then a write after a repeated START. The slave
	// reports the first one when the second address comes.
	mc.bDmaEn = false;
	mc.Rate = 100000;
	assert(I2CInit(&s_Master, &mc));
	{
		I2CCfg_t scfg = mc;
		scfg.DevNo = 2;
		scfg.Mode = I2CMODE_SLAVE;
		scfg.NbSlaveAddr = 1;
		scfg.SlaveAddr[0] = SLAVE_ADDR;
		scfg.EvtCB = SlaveEvt;
		assert(I2CInit(&s_Slave, &scfg));
	}
	s_WriteRqstSize = 20;
	s_SlaveCompleted = 0;
	s_M[1].Log.clear();
	uint8_t w1 = 0x05, w2[2] = { 0x06, 0x07 };
	assert(I2CStartTx(&s_Master, SLAVE_ADDR));
	s_Master.DevIntrf.bNoStop = true;
	assert(I2CTxData(&s_Master, &w1, 1) == 1);
	assert(s_Master.DevIntrf.StartTx(&s_Master.DevIntrf, SLAVE_ADDR));
	s_Master.DevIntrf.bNoStop = false;
	assert(I2CTxData(&s_Master, w2, 2) == 2);
	I2CStopTx(&s_Master);
	Settle();
	assert(s_M[1].Log == "SWa05SrWa0607P");
	assert(s_SlaveCompleted == 2 && s_SlaveWriteLen == 2 && s_WriteRqstData[0] == 6);

	// Back to polling: the DMA channel is given back
	I2CCfg_t sc = mc;
	sc.DevNo = 2;
	sc.Mode = I2CMODE_SLAVE;
	sc.NbSlaveAddr = 1;
	sc.SlaveAddr[0] = SLAVE_ADDR;
	sc.EvtCB = SlaveEvt;
	assert(I2CInit(&s_Slave, &sc));
	assert(s_DmaRegs.CTRL == 0);
	assert(I2CTx(&s_Master, SLAVE_ADDR, d, 4) == 4);

	printf("lpc546xx i2c bus: DMA error, release of the DMA channel: PASS\n");
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
