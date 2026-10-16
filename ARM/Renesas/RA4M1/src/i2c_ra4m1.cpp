/**-------------------------------------------------------------------------
@file	i2c_ra4m1.cpp

@brief	RA4M1 IIC0/IIC1 I2C master and slave implementation.

			DevNo 0 selects IIC0 and DevNo 1 selects IIC1. Standard 7-bit
			addressing up to 400 kHz. The master runs polled or interrupt
			driven from one shared service routine, so both modes follow the
			same register sequence. The slave is interrupt driven and answers
			up to three addresses.

			The IIC clock is PCLKB. StartTx/StartRx only record the target;
			the bus activity starts in TxData/RxData. A TxSrData transfer keeps
			the bus and the following RxData issues the repeated start.
			Interrupt mode copies the transmit data, returns -1 and reports the
			result with DEVINTRF_EVT_COMPLETED.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <stdint.h>
#include <string.h>

#include "ra4m1xxx.h"

#include "coredev/i2c.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "interrupt_ra4m1.h"
#include "i2c_ra4m1.h"

#define RA4M1_I2C_COUNT				2
#define RA4M1_I2C_MAX_RATE			400000U
#define RA4M1_I2C_MAX_SLAVE_ADDR	3
#define RA4M1_I2C_TXBUF_SIZE		64
#define RA4M1_I2C_ICBR_RESERVED		0xE0U
#define RA4M1_I2C_ICMR1_BASE		0x08U
#define RA4M1_I2C_ICMR2_MASTER		0x06U
#define RA4M1_I2C_ICMR2_SLAVE		0x06U
#define RA4M1_I2C_ICFER_MASTER		0x7AU
#define RA4M1_I2C_ICFER_SLAVE		0x5DU

// ICCR2 request bits. They are set with a read-modify-write so that MST and
// TRS are kept while a request is issued.
#define RA4M1_I2C_ICCR2_ST			0x02U
#define RA4M1_I2C_ICCR2_RS			0x04U
#define RA4M1_I2C_ICCR2_SP			0x08U

// ICSR2 flags. A flag is cleared by writing 0, writing 1 has no effect.
#define RA4M1_I2C_SR2_TMOF			0x01U
#define RA4M1_I2C_SR2_AL			0x02U
#define RA4M1_I2C_SR2_START			0x04U
#define RA4M1_I2C_SR2_STOP			0x08U
#define RA4M1_I2C_SR2_NACKF			0x10U
#define RA4M1_I2C_SR2_RDRF			0x20U
#define RA4M1_I2C_SR2_TEND			0x40U
#define RA4M1_I2C_SR2_TDRE			0x80U
#define RA4M1_I2C_SR2_EVENTS		(RA4M1_I2C_SR2_TMOF | RA4M1_I2C_SR2_AL | \
									 RA4M1_I2C_SR2_START | RA4M1_I2C_SR2_STOP | \
									 RA4M1_I2C_SR2_NACKF)

// Other register bits
#define RA4M1_I2C_CR1_ICE			0x80U
#define RA4M1_I2C_CR1_IICRST		0x40U
#define RA4M1_I2C_CR1_SOWP			0x10U
#define RA4M1_I2C_CR2_BBSY			0x80U
#define RA4M1_I2C_FER_TMOE			0x01U
#define RA4M1_I2C_MR3_NF2			0x01U
#define RA4M1_I2C_MR3_ACKBT			0x08U
#define RA4M1_I2C_MR3_ACKWP			0x10U
#define RA4M1_I2C_MR3_WAIT			0x40U

// ICIER interrupt enables
#define RA4M1_I2C_IER_TMOIE			0x01U
#define RA4M1_I2C_IER_ALIE			0x02U
#define RA4M1_I2C_IER_STIE			0x04U
#define RA4M1_I2C_IER_SPIE			0x08U
#define RA4M1_I2C_IER_NAKIE			0x10U
#define RA4M1_I2C_IER_RIE			0x20U
#define RA4M1_I2C_IER_TEIE			0x40U
#define RA4M1_I2C_IER_TIE			0x80U
#define RA4M1_I2C_IER_BUS			(RA4M1_I2C_IER_TMOIE | RA4M1_I2C_IER_ALIE | \
									 RA4M1_I2C_IER_STIE | RA4M1_I2C_IER_SPIE | \
									 RA4M1_I2C_IER_NAKIE)
#define RA4M1_I2C_IER_MASTER		(RA4M1_I2C_IER_BUS | RA4M1_I2C_IER_RIE | \
									 RA4M1_I2C_IER_TEIE)
// TIE is only on while the master reads. TDRE is set whenever ICDRT is empty,
// so an enabled TIE outside a read would keep the transmit event pending.
#define RA4M1_I2C_IER_SLAVE_IDLE	(RA4M1_I2C_IER_RIE | \
									 RA4M1_I2C_IER_TMOIE | RA4M1_I2C_IER_ALIE)
#define RA4M1_I2C_IER_SLAVE_WRITE	(RA4M1_I2C_IER_RIE | RA4M1_I2C_IER_BUS)
#define RA4M1_I2C_IER_SLAVE_READ	(RA4M1_I2C_IER_RIE | RA4M1_I2C_IER_TIE | RA4M1_I2C_IER_BUS)

// Master transfer states
typedef enum
{
	RA4M1_I2C_M_IDLE,		// no transfer
	RA4M1_I2C_M_HELD,		// transmit done, bus kept for a repeated start
	RA4M1_I2C_M_START,		// START or RESTART requested
	RA4M1_I2C_M_TX,			// address and data bytes going out
	RA4M1_I2C_M_RXADDR,		// receive address sent, waiting for dummy read
	RA4M1_I2C_M_RX,			// data bytes coming in
	RA4M1_I2C_M_STOP,		// STOP requested, waiting for it
	RA4M1_I2C_M_DONE		// finished, result is in the fields
} Ra4m1I2cMState_t;

// Slave transfer states
typedef enum
{
	RA4M1_I2C_S_IDLE,		// not addressed
	RA4M1_I2C_S_WRITE,		// master writes, slave receives
	RA4M1_I2C_S_READ		// master reads, slave transmits
} Ra4m1I2cSState_t;

typedef struct
{
	int DevNo;
	R_IIC0_Type *pReg;
	uint32_t MstpMask;
	I2CDev_t *pI2cDev;
	uint8_t Cks;
	uint8_t Brl;
	uint8_t Brh;
	bool bEnabled;
	bool bOpen;				// StartTx/StartRx done, StopTx/StopRx not yet
	bool bRegistered;		// interrupt routes are installed
	IRQn_Type IrqRxi;
	IRQn_Type IrqTxi;
	IRQn_Type IrqTei;
	IRQn_Type IrqEei;

	// Master
	volatile Ra4m1I2cMState_t MState;
	uint8_t Addr;
	bool bRxDir;			// current transfer direction
	bool bStopReq;			// issue STOP at the end of the transfer
	bool bAsync;			// interrupt driven, completion by callback
	bool bRestart;			// transfer started with a repeated START
	bool bAddrPhase;		// address byte not yet acknowledged
	bool bNack;				// a NACK ended the transfer
	bool bError;			// timeout, arbitration loss or bus failure
	const uint8_t *pTx;
	uint8_t *pRx;
	int Len;
	int Sent;				// bytes loaded in ICDRT
	int Acked;				// bytes acknowledged by the slave
	int Count;				// bytes received
	uint8_t TxBuf[RA4M1_I2C_TXBUF_SIZE];

	// Slave
	volatile Ra4m1I2cSState_t SState;
	int SlaveIdx;
	int SlaveCount;
	int PriorCount;			// write count kept over a repeated start
	bool bSlaveNack;		// master NACK seen while transmitting
	bool bSlaveLastReal;	// last byte loaded in ICDRT came from the buffer

} Ra4m1I2cDev_t;

#if RA4M1_I2C_TRACE
static Ra4m1I2cTrace_t s_Ra4m1I2cTrace[RA4M1_I2C_COUNT][RA4M1_I2C_TRACE_DEPTH];
static uint32_t s_Ra4m1I2cTraceCnt[RA4M1_I2C_COUNT];

static void Ra4m1I2cTrace(const Ra4m1I2cDev_t * const pDev, uint8_t Code, uint8_t Info)
{
	const R_IIC0_Type *reg = pDev->pReg;
	uint32_t n = s_Ra4m1I2cTraceCnt[pDev->DevNo];

	// Keep the first events and stop when the table is full, so the first
	// call of a run (and any refusal in it) is never overwritten by retries.
	if (n >= RA4M1_I2C_TRACE_DEPTH)
	{
		return;
	}

	s_Ra4m1I2cTraceCnt[pDev->DevNo] = n + 1U;

	Ra4m1I2cTrace_t *rec = &s_Ra4m1I2cTrace[pDev->DevNo][n];

	rec->Seq = n + 1U;
	rec->Code = Code;
	rec->Info = Info;
	rec->MState = (uint8_t)pDev->MState;
	rec->ICCR1 = reg->ICCR1;
	rec->ICCR2 = reg->ICCR2;
	rec->ICSR2 = reg->ICSR2;
	rec->ICFER = reg->ICFER;
	rec->ICMR3 = reg->ICMR3;
	rec->ICIER = reg->ICIER;
	rec->ICSER = reg->ICSER;

	// The other IIC unit is read only when a transfer is abandoned. The
	// transfer is dead by then, so the read cannot change its outcome.
	rec->PeerCR1 = 0U;
	rec->PeerSR1 = 0U;
	rec->PeerSR2 = 0U;
	rec->PeerIER = 0U;
	rec->SdaPfs = 0U;
	rec->SclPfs = 0U;
	rec->PeerSdaPfs = 0U;
	rec->PeerSclPfs = 0U;

	if (Code == RA4M1_I2C_TR_ABORT)
	{
		const bool peer0 = pDev->DevNo != 0;
		const R_IIC0_Type *peer = peer0 ? R_IIC0 : R_IIC1;
		const IOPinCfg_t *sda = &pDev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX];
		const IOPinCfg_t *scl = &pDev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX];

		rec->SdaPfs = R_PFS->PORT[sda->PortNo].PIN[sda->PinNo].PmnPFS;
		rec->SclPfs = R_PFS->PORT[scl->PortNo].PIN[scl->PinNo].PmnPFS;

		if ((R_MSTP->MSTPCRB & (1UL << (peer0 ? 9 : 8))) == 0U)
		{
			rec->PeerCR1 = peer->ICCR1;
			rec->PeerSR1 = peer->ICSR1;
			rec->PeerSR2 = peer->ICSR2;
			rec->PeerIER = peer->ICIER;

			const Ra4m1I2cDev_t *other = &s_Ra4m1I2cDev[1 - pDev->DevNo];

			if (other->pI2cDev != NULL && other->pI2cDev->Cfg.pIOPinMap != NULL)
			{
				const IOPinCfg_t *peerSda =
					&other->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX];
				const IOPinCfg_t *peerScl =
					&other->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX];

				rec->PeerSdaPfs =
					R_PFS->PORT[peerSda->PortNo].PIN[peerSda->PinNo].PmnPFS;
				rec->PeerSclPfs =
					R_PFS->PORT[peerScl->PortNo].PIN[peerScl->PinNo].PmnPFS;
			}
		}
	}
}

uint32_t Ra4m1I2cTraceCount(int DevNo)
{
	return (DevNo >= 0 && DevNo < RA4M1_I2C_COUNT) ? s_Ra4m1I2cTraceCnt[DevNo] : 0U;
}

bool Ra4m1I2cTraceGet(int DevNo, uint32_t Idx, Ra4m1I2cTrace_t *pRec)
{
	if (DevNo < 0 || DevNo >= RA4M1_I2C_COUNT || pRec == NULL)
	{
		return false;
	}

	uint32_t cnt = s_Ra4m1I2cTraceCnt[DevNo];
	uint32_t first = cnt > RA4M1_I2C_TRACE_DEPTH ? cnt - RA4M1_I2C_TRACE_DEPTH : 0U;

	if (Idx >= cnt - first)
	{
		return false;
	}

	*pRec = s_Ra4m1I2cTrace[DevNo][(first + Idx) % RA4M1_I2C_TRACE_DEPTH];

	return true;
}
#else
#define Ra4m1I2cTrace(pDev, Code, Info)		((void)0)

uint32_t Ra4m1I2cTraceCount(int DevNo)
{
	(void)DevNo;
	return 0U;
}

bool Ra4m1I2cTraceGet(int DevNo, uint32_t Idx, Ra4m1I2cTrace_t *pRec)
{
	(void)DevNo;
	(void)Idx;
	(void)pRec;
	return false;
}
#endif

static Ra4m1I2cDev_t s_Ra4m1I2cDev[RA4M1_I2C_COUNT] = {
	{
		.DevNo = 0,
		.pReg = R_IIC0,
		.MstpMask = (1UL << 9),
		.IrqRxi = (IRQn_Type)-1,
		.IrqTxi = (IRQn_Type)-1,
		.IrqTei = (IRQn_Type)-1,
		.IrqEei = (IRQn_Type)-1,
	},
	{
		.DevNo = 1,
		.pReg = R_IIC1,
		.MstpMask = (1UL << 8),
		.IrqRxi = (IRQn_Type)-1,
		.IrqTxi = (IRQn_Type)-1,
		.IrqTei = (IRQn_Type)-1,
		.IrqEei = (IRQn_Type)-1,
	},
};

static Ra4m1I2cDev_t *Ra4m1I2cHandle(DevIntrf_t * const pIntrf)
{
	if (pIntrf == NULL)
	{
		return NULL;
	}

	Ra4m1I2cDev_t *dev = (Ra4m1I2cDev_t *)pIntrf->pDevData;

	if (dev == NULL || dev->pI2cDev == NULL)
	{
		return NULL;
	}

	if (&dev->pI2cDev->DevIntrf != pIntrf)
	{
		return NULL;
	}

	return dev;
}

static void Ra4m1I2cModuleStart(Ra4m1I2cDev_t * const pDev, bool bStart)
{
	uint16_t prcr = R_SYSTEM->PRCR & 0x0FU;

	R_SYSTEM->PRCR = 0xA500U | prcr | R_SYSTEM_PRCR_PRC1_Msk;

	if (bStart)
	{
		R_MSTP->MSTPCRB &= ~pDev->MstpMask;
	}
	else
	{
		R_MSTP->MSTPCRB |= pDev->MstpMask;
	}

	(void)R_MSTP->MSTPCRB;
	R_SYSTEM->PRCR = 0xA500U | prcr;
}

// Number of idle polling passes allowed before a transfer is declared stuck.
static int32_t Ra4m1I2cTimeout(const Ra4m1I2cDev_t * const pDev)
{
	uint32_t core = SystemCoreClockGet();
	uint32_t rate = pDev->pI2cDev->Cfg.Rate;

	if (core == 0U || rate == 0U)
	{
		return 1000000;
	}

	// About 64 byte times of polling passes without progress
	uint64_t ticks = ((uint64_t)core / rate) * 64U + 256U;

	return ticks > INT32_MAX ? INT32_MAX : (int32_t)ticks;
}

static bool Ra4m1I2cRate(uint32_t Rate, uint8_t *pCks, uint8_t *pBrl,
	uint8_t *pBrh, uint32_t *pActual)
{
	if (pCks == NULL || pBrl == NULL || pBrh == NULL || pActual == NULL)
	{
		return false;
	}

	uint32_t clk = SystemPeriphClockGet(1);

	if (Rate == 0U || Rate > RA4M1_I2C_MAX_RATE || clk == 0U)
	{
		return false;
	}

	uint32_t lowNs = Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U ?
		I2C_SCL_TLOW_STD_MODE_MIN : I2C_SCL_TLOW_FAST_MODE_MIN;
	uint32_t highNs = Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U ?
		I2C_SCL_THIGH_STD_MODE_MIN : I2C_SCL_THIGH_FAST_MODE_MIN;
	uint32_t bestTicks = UINT32_MAX;
	uint8_t bestCks = 0U;
	uint8_t bestBrl = 0U;
	uint8_t bestBrh = 0U;

	// The master runs with SCLE and NFE enabled and two noise filter stages.
	// With CKS=0 the IIC adds five IIC-phi clocks to each programmed high/low
	// count, with a divided IIC-phi it adds four. Rise and fall time are
	// treated as zero, a real open-drain bus only lengthens the period, so the
	// result is at or below the requested rate and never violates the low/high
	// minima.
	for (uint8_t cks = 0U; cks < 8U; cks++)
	{
		uint32_t offset = cks == 0U ? 5U : 4U;
		uint32_t divider = 1UL << cks;

		for (uint8_t brl = 0U; brl < 32U; brl++)
		{
			uint32_t lowTicks = ((uint32_t)brl + offset) * divider;

			if ((uint64_t)lowTicks * 1000000000ULL < (uint64_t)lowNs * clk)
			{
				continue;
			}

			for (uint8_t brh = 0U; brh < 32U; brh++)
			{
				uint32_t highTicks = ((uint32_t)brh + offset) * divider;
				uint32_t ticks = lowTicks + highTicks;

				if ((uint64_t)highTicks * 1000000000ULL < (uint64_t)highNs * clk)
				{
					continue;
				}

				if ((uint64_t)ticks * Rate < clk)
				{
					continue;
				}

				if (ticks < bestTicks)
				{
					bestTicks = ticks;
					bestCks = cks;
					bestBrl = brl;
					bestBrh = brh;
				}
			}
		}
	}

	if (bestTicks == UINT32_MAX)
	{
		return false;
	}

	*pCks = bestCks;
	*pBrl = bestBrl;
	*pBrh = bestBrh;
	*pActual = clk / bestTicks;

	return *pActual != 0U;
}

// Write 0 to the selected flags only. A read-modify-write would also clear a
// flag that rises between the read and the write.
static inline void Ra4m1I2cClearFlags(R_IIC0_Type * const pReg, uint8_t Mask)
{
	pReg->ICSR2 = (uint8_t)~Mask;
}

// ACKBT is only writable while ACKWP is set.
static void Ra4m1I2cAck(R_IIC0_Type * const pReg, bool bNack)
{
	pReg->ICMR3 |= RA4M1_I2C_MR3_ACKWP;

	if (bNack)
	{
		pReg->ICMR3 |= RA4M1_I2C_MR3_ACKBT;
	}
	else
	{
		pReg->ICMR3 &= (uint8_t)~RA4M1_I2C_MR3_ACKBT;
	}

	pReg->ICMR3 &= (uint8_t)~RA4M1_I2C_MR3_ACKWP;
}

static inline void Ra4m1I2cSetWait(R_IIC0_Type * const pReg, bool bWait)
{
	if (bWait)
	{
		pReg->ICMR3 |= RA4M1_I2C_MR3_WAIT;
	}
	else
	{
		pReg->ICMR3 &= (uint8_t)~RA4M1_I2C_MR3_WAIT;
	}
}

// Reading ICDRR also releases SCL and clears RDRF, so a discarded read must
// still be issued.
static inline uint8_t Ra4m1I2cReadData(R_IIC0_Type * const pReg)
{
	return pReg->ICDRR;
}

static inline void Ra4m1I2cIceOff(R_IIC0_Type * const pReg)
{
	pReg->ICCR1 &= (uint8_t)~RA4M1_I2C_CR1_ICE;
}

static inline bool Ra4m1I2cBusy(const R_IIC0_Type * const pReg)
{
	return (pReg->ICCR2 & RA4M1_I2C_CR2_BBSY) != 0U;
}

static void Ra4m1I2cConfigure(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;
	const I2CCfg_t *cfg = &pDev->pI2cDev->Cfg;
	const bool slave = cfg->Mode == I2CMODE_SLAVE;

	// Use the vendor reset sequence with explicit ICCR1 values. Do not
	// preserve output-control bits from a previous transfer during recovery.
	reg->ICIER = 0U;
	reg->ICCR1 = 0U;
	reg->ICCR1 = 0x40U;

	int32_t count = Ra4m1I2cTimeout(pDev);

	while ((reg->ICCR1 & RA4M1_I2C_CR1_IICRST) == 0U && --count > 0)
	{
		__NOP();
	}

	pDev->MState = RA4M1_I2C_M_IDLE;
	pDev->SState = RA4M1_I2C_S_IDLE;
	pDev->bAsync = false;
	pDev->SlaveCount = 0;
	pDev->PriorCount = 0;
	pDev->bSlaveNack = false;
	pDev->bSlaveLastReal = false;

	if (count <= 0)
	{
		// An unresponsive block is left disabled.
		pDev->bError = true;
		reg->ICCR1 = 0U;
		return;
	}

	reg->ICCR1 = 0xC0U;

	for (int i = 0; i < RA4M1_I2C_MAX_SLAVE_ADDR; i++)
	{
		reg->SAR[i].L = 0U;
		reg->SAR[i].U = 0U;
	}

	reg->ICSER = 0U;

	if (slave)
	{
		uint8_t en = 0U;

		for (int i = 0; i < cfg->NbSlaveAddr && i < RA4M1_I2C_MAX_SLAVE_ADDR; i++)
		{
			reg->SAR[i].L = (uint8_t)(cfg->SlaveAddr[i] << 1U);
			en |= (uint8_t)(1U << i);
		}

		reg->ICSER = en;
	}

	reg->ICBRL = (uint8_t)(RA4M1_I2C_ICBR_RESERVED | pDev->Brl);
	reg->ICBRH = (uint8_t)(RA4M1_I2C_ICBR_RESERVED | pDev->Brh);
	reg->ICMR1 = (uint8_t)(RA4M1_I2C_ICMR1_BASE | (pDev->Cks << 4U));
	reg->ICMR2 = slave ? RA4M1_I2C_ICMR2_SLAVE : RA4M1_I2C_ICMR2_MASTER;

	// Do not request slave clock stretching before address reception.
	// Renesas configures WAIT as an optional clock-stretching feature. In the
	// loopback bring-up, leave WAIT clear to distinguish clock stretching from
	// an external SCL-low condition. Master RX configures WAIT as needed.
	// Two noise filter stages, as the RE01 driver on the same IIC block.
	reg->ICMR3 = RA4M1_I2C_MR3_NF2 | RA4M1_I2C_MR3_ACKWP;
	Ra4m1I2cAck(reg, false);

	reg->ICFER = slave ? RA4M1_I2C_ICFER_SLAVE : RA4M1_I2C_ICFER_MASTER;
	Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_EVENTS);
	reg->ICCR1 = 0x9FU;

	if (!pDev->bEnabled)
	{
		Ra4m1I2cIceOff(reg);
	}
	else if (slave && pDev->bRegistered)
	{
		reg->ICIER = RA4M1_I2C_IER_SLAVE_IDLE;
	}

	Ra4m1I2cTrace(pDev, RA4M1_I2C_TR_CONFIG, 0U);
}

// ---------------------------------------------------------------------------
// Master
// ---------------------------------------------------------------------------

// Wait for the bus free time. BBSY can stay set after noise on the lines
// while the pins were being configured, so one reset of the block is tried
// before giving up. A bus that is really held still reads busy afterwards.
static bool Ra4m1I2cWaitIdle(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;
	int32_t count = Ra4m1I2cTimeout(pDev);

	while (Ra4m1I2cBusy(reg) && count > 0)
	{
		count--;
	}

	if (Ra4m1I2cBusy(reg))
	{
		Ra4m1I2cTrace(pDev, RA4M1_I2C_TR_IDLE_BUSY, 0U);
		Ra4m1I2cConfigure(pDev);
	}

	if (Ra4m1I2cBusy(reg))
	{
		Ra4m1I2cTrace(pDev, RA4M1_I2C_TR_IDLE_FAIL, 0U);

		return false;
	}

	return true;
}

static void Ra4m1I2cMasterAbort(Ra4m1I2cDev_t * const pDev, uint8_t Why)
{
	Ra4m1I2cTrace(pDev, RA4M1_I2C_TR_ABORT, Why);
	(void)Why;
	pDev->bError = true;
	Ra4m1I2cConfigure(pDev);
	pDev->MState = RA4M1_I2C_M_DONE;
}

// Advance the transfer as far as the current flags allow. Called from the
// polling loop and from the interrupt handlers, so both follow one sequence.
static void Ra4m1I2cMasterService(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;
	const uint8_t sr = reg->ICSR2;

	switch (pDev->MState)
	{
		case RA4M1_I2C_M_START:
		case RA4M1_I2C_M_TX:
		case RA4M1_I2C_M_RXADDR:
		case RA4M1_I2C_M_RX:
		case RA4M1_I2C_M_STOP:
			break;

		default:
			return;
	}

	if ((sr & (RA4M1_I2C_SR2_TMOF | RA4M1_I2C_SR2_AL)) != 0U)
	{
		Ra4m1I2cMasterAbort(pDev, 1U);
		return;
	}

	if (pDev->MState == RA4M1_I2C_M_STOP)
	{
		if ((sr & RA4M1_I2C_SR2_STOP) != 0U)
		{
			Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_STOP | RA4M1_I2C_SR2_NACKF |
				RA4M1_I2C_SR2_START);
			Ra4m1I2cSetWait(reg, false);
			Ra4m1I2cAck(reg, false);
			pDev->MState = RA4M1_I2C_M_DONE;
		}

		return;
	}

	if ((sr & RA4M1_I2C_SR2_NACKF) != 0U &&
		pDev->MState != RA4M1_I2C_M_START && pDev->MState != RA4M1_I2C_M_RX)
	{
		// The slave refused the address or a data byte. With the master
		// still owning the bus a STOP is required to release it.
		Ra4m1I2cTrace(pDev, RA4M1_I2C_TR_NACK, (uint8_t)pDev->Sent);
		Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_NACKF);
		pDev->bNack = true;
		reg->ICCR2 |= RA4M1_I2C_ICCR2_SP;
		pDev->MState = RA4M1_I2C_M_STOP;
		return;
	}

	switch (pDev->MState)
	{
		case RA4M1_I2C_M_START:
			// TDRE can be set before the START went out, so wait for the
			// START flag. After a plain START, BBSY also says it went out.
			if (((sr & RA4M1_I2C_SR2_START) != 0U ||
				 (!pDev->bRestart && Ra4m1I2cBusy(reg))) &&
				(sr & RA4M1_I2C_SR2_TDRE) != 0U)
			{
				Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_START);
				reg->ICDRT = (uint8_t)((pDev->Addr << 1U) | (pDev->bRxDir ? 1U : 0U));
				Ra4m1I2cTrace(pDev, RA4M1_I2C_TR_ADDR, pDev->bRxDir ? 1U : 0U);
				pDev->MState = pDev->bRxDir ?
					RA4M1_I2C_M_RXADDR : RA4M1_I2C_M_TX;
			}
			break;

		case RA4M1_I2C_M_TX:
			if ((sr & RA4M1_I2C_SR2_TEND) == 0U)
			{
				break;
			}

			// TEND means the byte just sent was acknowledged.
			if (pDev->bAddrPhase)
			{
				pDev->bAddrPhase = false;
			}
			else
			{
				pDev->Acked = pDev->Sent;
			}

			if (pDev->Sent < pDev->Len)
			{
				reg->ICDRT = pDev->pTx[pDev->Sent++];
			}
			else if (pDev->bStopReq)
			{
				reg->ICCR2 |= RA4M1_I2C_ICCR2_SP;
				pDev->MState = RA4M1_I2C_M_STOP;
			}
			else
			{
				pDev->MState = RA4M1_I2C_M_HELD;
			}
			break;

		case RA4M1_I2C_M_RXADDR:
			if ((sr & RA4M1_I2C_SR2_RDRF) == 0U)
			{
				break;
			}

			// The first RDRF after the address is a dummy read that starts
			// the receive clocks. One and two byte reads need WAIT so the
			// NACK and STOP can be set up in time.
			if (pDev->Len <= 2)
			{
				Ra4m1I2cSetWait(reg, true);
			}

			if (pDev->Len == 1)
			{
				Ra4m1I2cAck(reg, true);
			}

			(void)Ra4m1I2cReadData(reg);
			pDev->MState = RA4M1_I2C_M_RX;
			break;

		case RA4M1_I2C_M_RX:
			if ((sr & RA4M1_I2C_SR2_RDRF) != 0U)
			{
				const int remain = pDev->Len - pDev->Count;

				if (remain == 3)
				{
					Ra4m1I2cSetWait(reg, true);
				}

				if (remain == 2)
				{
					Ra4m1I2cAck(reg, true);
				}

				if (remain == 1)
				{
					reg->ICCR2 |= RA4M1_I2C_ICCR2_SP;
				}

				pDev->pRx[pDev->Count++] = reg->ICDRR;

				if (pDev->Count >= pDev->Len)
				{
					Ra4m1I2cSetWait(reg, false);
					pDev->MState = RA4M1_I2C_M_STOP;
				}
			}
			break;

		default:
			break;
	}
}

// Prepare and request START, or RESTART when the bus is held.
static bool Ra4m1I2cMasterBegin(Ra4m1I2cDev_t * const pDev, bool bRx,
	bool bStop, bool bAsync)
{
	R_IIC0_Type *reg = pDev->pReg;
	const bool restart = pDev->MState == RA4M1_I2C_M_HELD;

	if (!restart && !Ra4m1I2cWaitIdle(pDev))
	{
		return false;
	}

	pDev->bRxDir = bRx;
	pDev->bRestart = restart;
	pDev->bStopReq = bStop;
	pDev->bAsync = bAsync;
	pDev->bAddrPhase = true;
	pDev->bNack = false;
	pDev->bError = false;
	pDev->Sent = 0;
	pDev->Acked = 0;
	pDev->Count = 0;

	Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_EVENTS);
	pDev->MState = RA4M1_I2C_M_START;
	Ra4m1I2cTrace(pDev, RA4M1_I2C_TR_BEGIN, (uint8_t)((bRx ? 1U : 0U) | (restart ? 2U : 0U)));

	// Polling mode reads the flags and leaves the interrupt enables off.
	reg->ICIER = bAsync ? RA4M1_I2C_IER_MASTER : 0U;

	if (restart)
	{
		reg->ICCR2 |= RA4M1_I2C_ICCR2_RS;
	}
	else
	{
		reg->ICCR2 |= RA4M1_I2C_ICCR2_ST;
	}

	return true;
}

// Polling driver of the shared service routine.
static void Ra4m1I2cMasterRun(Ra4m1I2cDev_t * const pDev)
{
	const int32_t budget = Ra4m1I2cTimeout(pDev);
	int32_t left = budget;
	int lastSent = -1;
	int lastCount = -1;
	Ra4m1I2cMState_t last = RA4M1_I2C_M_IDLE;

	while (pDev->MState != RA4M1_I2C_M_DONE && pDev->MState != RA4M1_I2C_M_HELD)
	{
		Ra4m1I2cMasterService(pDev);

		if (pDev->MState != last || pDev->Sent != lastSent || pDev->Count != lastCount)
		{
			last = pDev->MState;
			lastSent = pDev->Sent;
			lastCount = pDev->Count;
			left = budget;
		}
		else if (--left <= 0)
		{
			Ra4m1I2cMasterAbort(pDev, 2U);
		}
		else if (left == budget / 2)
		{
			// One look at the registers while the transfer is stuck
			Ra4m1I2cTrace(pDev, RA4M1_I2C_TR_SAMPLE, 0U);
		}
	}
}

// Take the finished transfer result and return the engine to idle.
static int Ra4m1I2cMasterResult(Ra4m1I2cDev_t * const pDev)
{
	const int count = pDev->bRxDir ? pDev->Count : pDev->Acked;

	if (pDev->MState == RA4M1_I2C_M_DONE)
	{
		pDev->MState = RA4M1_I2C_M_IDLE;
	}

	pDev->bAsync = false;
	pDev->pReg->ICIER = 0U;

	return count;
}

// Called when an interrupt driven transfer reaches DONE.
static void Ra4m1I2cMasterAsyncDone(Ra4m1I2cDev_t * const pDev)
{
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;
	const bool rx = pDev->bRxDir;

	pDev->pReg->ICIER = 0U;

	const int count = Ra4m1I2cMasterResult(pDev);

	if (rx)
	{
		DeviceIntrfStopRx(intrf);
	}
	else
	{
		DeviceIntrfTxComplete(intrf);
	}

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, NULL, count);
	}
}

// Issue STOP for a held bus or a transfer left open, then go idle.
static void Ra4m1I2cMasterRelease(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;

	if (pDev->MState == RA4M1_I2C_M_HELD)
	{
		reg->ICCR2 |= RA4M1_I2C_ICCR2_SP;
		pDev->MState = RA4M1_I2C_M_STOP;
		Ra4m1I2cMasterRun(pDev);
	}

	if (pDev->MState == RA4M1_I2C_M_DONE)
	{
		pDev->MState = RA4M1_I2C_M_IDLE;
	}

	reg->ICIER = 0U;
}

static void Ra4m1I2cStop(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER)
	{
		return;
	}

	if (!dev->bAsync)
	{
		Ra4m1I2cMasterRelease(dev);
	}

	dev->bOpen = false;
	pIntrf->bTxReady = true;
}

// StartTx/StartRx only record the target. The bus is touched by the data
// call, which lets a repeated start follow a held write without a gap.
static bool Ra4m1I2cStart(DevIntrf_t * const pIntrf, uint32_t Addr, bool bRx)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || !dev->bEnabled || Addr > 0x7FU ||
		dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER)
	{
#if RA4M1_I2C_TRACE
		if (dev != NULL)
		{
			Ra4m1I2cTrace(dev, RA4M1_I2C_TR_START_REFUSED,
				(uint8_t)((dev->bEnabled ? 0U : RA4M1_I2C_INFO_NOT_ENABLED) |
				(Addr > 0x7FU ? RA4M1_I2C_INFO_BAD_ADDR : 0U) |
				(dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER ? RA4M1_I2C_INFO_NOT_MASTER : 0U)));
		}
#endif
		return false;
	}

	if (!dev->bOpen && dev->MState != RA4M1_I2C_M_HELD &&
		!Ra4m1I2cWaitIdle(dev))
	{
		return false;
	}

	dev->Addr = (uint8_t)Addr;
	dev->bRxDir = bRx;
	dev->bOpen = true;

	return true;
}

static bool Ra4m1I2cStartTx(DevIntrf_t * const pIntrf, uint32_t Addr)
{
	return Ra4m1I2cStart(pIntrf, Addr, false);
}

static bool Ra4m1I2cStartRx(DevIntrf_t * const pIntrf, uint32_t Addr)
{
	return Ra4m1I2cStart(pIntrf, Addr, true);
}

static int Ra4m1I2cTx(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || !dev->bOpen || dev->bRxDir || dev->bAsync ||
		pData == NULL || Len <= 0)
	{
#if RA4M1_I2C_TRACE
		if (dev != NULL)
		{
			Ra4m1I2cTrace(dev, RA4M1_I2C_TR_TX_REFUSED,
				(uint8_t)((dev->bOpen ? 0U : RA4M1_I2C_INFO_NOT_OPEN) |
				(dev->bRxDir ? RA4M1_I2C_INFO_WRONG_DIR : 0U) |
				(dev->bAsync ? RA4M1_I2C_INFO_ASYNC_BUSY : 0U) |
				((pData == NULL || Len <= 0) ? RA4M1_I2C_INFO_BAD_ARG : 0U)));
		}
#endif
		return 0;
	}

	// Interrupt mode owns a copy of the data and reports at the end. A
	// transfer that keeps the bus, or one larger than the buffer, is run to
	// the end here.
	const bool async = pIntrf->bIntEn && !pIntrf->bNoStop && Len <= RA4M1_I2C_TXBUF_SIZE;

	if (async)
	{
		memcpy(dev->TxBuf, pData, Len);
		dev->pTx = dev->TxBuf;
	}
	else
	{
		dev->pTx = pData;
	}

	dev->Len = Len;
	pIntrf->bTxReady = false;

	if (!Ra4m1I2cMasterBegin(dev, false, !pIntrf->bNoStop, async))
	{
		pIntrf->bTxReady = true;
		return 0;
	}

	if (async)
	{
		return -1;
	}

	Ra4m1I2cMasterRun(dev);

	const int count = Ra4m1I2cMasterResult(dev);

	pIntrf->bTxReady = true;

	return count;
}

static int Ra4m1I2cRx(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || !dev->bOpen || !dev->bRxDir || dev->bAsync ||
		pBuff == NULL || Len <= 0)
	{
#if RA4M1_I2C_TRACE
		if (dev != NULL)
		{
			Ra4m1I2cTrace(dev, RA4M1_I2C_TR_RX_REFUSED,
				(uint8_t)((dev->bOpen ? 0U : RA4M1_I2C_INFO_NOT_OPEN) |
				(dev->bRxDir ? 0U : RA4M1_I2C_INFO_WRONG_DIR) |
				(dev->bAsync ? RA4M1_I2C_INFO_ASYNC_BUSY : 0U) |
				((pBuff == NULL || Len <= 0) ? RA4M1_I2C_INFO_BAD_ARG : 0U)));
		}
#endif
		return 0;
	}

	const bool async = pIntrf->bIntEn;

	dev->pRx = pBuff;
	dev->Len = Len;

	if (!Ra4m1I2cMasterBegin(dev, true, true, async))
	{
		return 0;
	}

	if (async)
	{
		return -1;
	}

	Ra4m1I2cMasterRun(dev);

	return Ra4m1I2cMasterResult(dev);
}

// ---------------------------------------------------------------------------
// Slave
// ---------------------------------------------------------------------------

static void Ra4m1I2cSlaveEvent(Ra4m1I2cDev_t * const pDev, DEVINTRF_EVT Evt, int Len)
{
	I2CDev_t *i2c = pDev->pI2cDev;

	if (i2c->DevIntrf.EvtCB != NULL)
	{
		i2c->DevIntrf.EvtCB(&i2c->DevIntrf, Evt, NULL, Len);
	}
}

// End the transfer in progress and report the byte count. A master NACK
// leaves one byte loaded in ICDRT that never went out, so it is not counted.
static void Ra4m1I2cSlaveComplete(Ra4m1I2cDev_t * const pDev)
{
	int count = pDev->SlaveCount;

	if (pDev->SState == RA4M1_I2C_S_READ && pDev->bSlaveNack &&
		pDev->bSlaveLastReal && count > 0)
	{
		count--;
	}

	pDev->SState = RA4M1_I2C_S_IDLE;
	pDev->SlaveCount = 0;
	pDev->bSlaveNack = false;
	pDev->bSlaveLastReal = false;
	Ra4m1I2cSlaveEvent(pDev, DEVINTRF_EVT_COMPLETED, count);
}

// Handle STOP, repeated START and error flags.
static void Ra4m1I2cSlaveEnd(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;
	const uint8_t sr = reg->ICSR2;

	if ((sr & (RA4M1_I2C_SR2_TMOF | RA4M1_I2C_SR2_AL)) != 0U)
	{
		const bool active = pDev->SState != RA4M1_I2C_S_IDLE;
		const int prior = pDev->PriorCount;

		if (active)
		{
			Ra4m1I2cSlaveComplete(pDev);
		}
		else if (prior > 0)
		{
			Ra4m1I2cSlaveEvent(pDev, DEVINTRF_EVT_COMPLETED, prior);
		}

		Ra4m1I2cConfigure(pDev);
		return;
	}

	if ((sr & RA4M1_I2C_SR2_STOP) != 0U)
	{
		const int prior = pDev->PriorCount;

		Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_STOP | RA4M1_I2C_SR2_NACKF);
		pDev->PriorCount = 0;
		Ra4m1I2cAck(reg, false);

		if (pDev->SState != RA4M1_I2C_S_IDLE)
		{
			Ra4m1I2cSlaveComplete(pDev);
		}
		else if (prior > 0)
		{
			Ra4m1I2cSlaveEvent(pDev, DEVINTRF_EVT_COMPLETED, prior);
		}

		reg->ICIER = RA4M1_I2C_IER_SLAVE_IDLE;
		return;
	}

	if ((sr & RA4M1_I2C_SR2_START) != 0U && pDev->SState != RA4M1_I2C_S_IDLE)
	{
		// Repeated START. A write phase hands its count to the following
		// read request, a read phase simply ends.
		Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_START | RA4M1_I2C_SR2_NACKF);
		Ra4m1I2cAck(reg, false);

		if (pDev->SState == RA4M1_I2C_S_WRITE)
		{
			pDev->PriorCount = pDev->SlaveCount;
			pDev->SState = RA4M1_I2C_S_IDLE;
			pDev->SlaveCount = 0;
		}
		else
		{
			Ra4m1I2cSlaveComplete(pDev);
		}

		reg->ICIER = RA4M1_I2C_IER_SLAVE_WRITE;
	}
}

// The first byte received after an address match is the address itself.
static void Ra4m1I2cSlaveAddress(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;
	I2CDev_t *i2c = pDev->pI2cDev;
	const uint8_t sr1 = reg->ICSR1;
	const uint8_t addr = reg->ICDRR;
	const bool read = (addr & 1U) != 0U;

	const int prior = pDev->PriorCount;
	int idx = 0;

	if ((sr1 & 0x01U) != 0U)
	{
		idx = 0;
	}
	else if ((sr1 & 0x02U) != 0U)
	{
		idx = 1;
	}
	else if ((sr1 & 0x04U) != 0U)
	{
		idx = 2;
	}

	// The match START and any stale STOP are consumed here, so a later
	// START flag can only be a repeated START.
	Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_START | RA4M1_I2C_SR2_STOP |
		RA4M1_I2C_SR2_NACKF);

	pDev->SlaveIdx = idx;
	pDev->SlaveCount = 0;
	pDev->PriorCount = 0;
	pDev->bSlaveNack = false;
	pDev->bSlaveLastReal = false;
	pDev->SState = read ? RA4M1_I2C_S_READ : RA4M1_I2C_S_WRITE;
	reg->ICIER = read ? RA4M1_I2C_IER_SLAVE_READ : RA4M1_I2C_IER_SLAVE_WRITE;

	// The application installs the buffer again for every request.
	if (read)
	{
		i2c->pRRData[idx] = NULL;
		i2c->RRDataLen[idx] = 0;
		Ra4m1I2cSlaveEvent(pDev, DEVINTRF_EVT_READ_RQST, prior);
	}
	else
	{
		i2c->pTRBuff[idx] = NULL;
		i2c->TRBuffLen[idx] = 0;

		if (prior > 0)
		{
			Ra4m1I2cSlaveEvent(pDev, DEVINTRF_EVT_COMPLETED, prior);
		}

		Ra4m1I2cSlaveEvent(pDev, DEVINTRF_EVT_WRITE_RQST, 0);

		// With no room, NACK the first data byte.
		if (i2c->pTRBuff[idx] == NULL || i2c->TRBuffLen[idx] <= 0)
		{
			Ra4m1I2cAck(reg, true);
		}
	}
}

static void Ra4m1I2cSlaveReceive(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;
	I2CDev_t *i2c = pDev->pI2cDev;
	const uint8_t value = reg->ICDRR;
	uint8_t *buf = i2c->pTRBuff[pDev->SlaveIdx];

	const int len = i2c->TRBuffLen[pDev->SlaveIdx];

	if (buf != NULL && pDev->SlaveCount < len)
	{
		buf[pDev->SlaveCount++] = value;
	}

	// The acknowledge of the next byte is decided now, so a full buffer
	// is refused on the following byte.
	if (buf == NULL || pDev->SlaveCount >= len)
	{
		Ra4m1I2cAck(reg, true);
	}
}

static void Ra4m1I2cSlaveTransmit(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;
	I2CDev_t *i2c = pDev->pI2cDev;
	const uint8_t *data = i2c->pRRData[pDev->SlaveIdx];
	const int len = i2c->RRDataLen[pDev->SlaveIdx];

	if (data != NULL && pDev->SlaveCount < len)
	{
		reg->ICDRT = data[pDev->SlaveCount++];
		pDev->bSlaveLastReal = true;
	}
	else
	{
		reg->ICDRT = 0xFFU;
		pDev->bSlaveLastReal = false;
	}
}

static void Ra4m1I2cSlaveService(Ra4m1I2cDev_t * const pDev)
{
	R_IIC0_Type *reg = pDev->pReg;

	if (!pDev->bEnabled)
	{
		return;
	}

	Ra4m1I2cSlaveEnd(pDev);

	if ((reg->ICSR2 & RA4M1_I2C_SR2_RDRF) != 0U)
	{
		switch (pDev->SState)
		{
			case RA4M1_I2C_S_IDLE:
				Ra4m1I2cSlaveAddress(pDev);
				break;

			case RA4M1_I2C_S_WRITE:
				Ra4m1I2cSlaveReceive(pDev);
				break;

			default:
				(void)Ra4m1I2cReadData(reg);
				break;
		}
	}

	if (pDev->SState == RA4M1_I2C_S_READ)
	{
		const uint8_t sr = reg->ICSR2;

		if ((sr & RA4M1_I2C_SR2_NACKF) != 0U)
		{
			// The master ended the read. The dummy read releases SCL.
			(void)Ra4m1I2cReadData(reg);
			Ra4m1I2cClearFlags(reg, RA4M1_I2C_SR2_NACKF);
			reg->ICIER &= (uint8_t)~(RA4M1_I2C_IER_NAKIE | RA4M1_I2C_IER_TIE);
			pDev->bSlaveNack = true;
		}
		else if ((sr & RA4M1_I2C_SR2_TDRE) != 0U)
		{
			Ra4m1I2cSlaveTransmit(pDev);
		}
	}
}

// ---------------------------------------------------------------------------
// Interrupt routing
// ---------------------------------------------------------------------------

// The four events are edges. The request is cleared first, so an edge that
// arrives while the service runs raises the handler again instead of being
// lost. The service itself works from the flags, not from which event fired.
static void Ra4m1I2cIrqHandler(int IntNo, void *pCtx)
{
	Ra4m1I2cDev_t *dev = (Ra4m1I2cDev_t *)pCtx;

	Ra4m1AcknowledgeInt((IRQn_Type)IntNo);

	if (dev->pI2cDev == NULL)
	{
		return;
	}

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		Ra4m1I2cSlaveService(dev);
	}
	else if (dev->bAsync)
	{
		Ra4m1I2cMasterService(dev);

		if (dev->MState == RA4M1_I2C_M_DONE)
		{
			Ra4m1I2cMasterAsyncDone(dev);
		}
	}
}

static void Ra4m1I2cUnregister(Ra4m1I2cDev_t * const pDev)
{
	Ra4m1UnregisterIntHandler(pDev->IrqRxi);
	Ra4m1UnregisterIntHandler(pDev->IrqTxi);
	Ra4m1UnregisterIntHandler(pDev->IrqTei);
	Ra4m1UnregisterIntHandler(pDev->IrqEei);
	pDev->IrqRxi = (IRQn_Type)-1;
	pDev->IrqTxi = (IRQn_Type)-1;
	pDev->IrqTei = (IRQn_Type)-1;
	pDev->IrqEei = (IRQn_Type)-1;
	pDev->bRegistered = false;
}

// Each IIC has four ICU events: RXI, TXI, TEI and EEI in consecutive ids.
static bool Ra4m1I2cRegister(Ra4m1I2cDev_t * const pDev, int Priority)
{
	uint8_t event = pDev->DevNo == 0 ?
		RA4M1_EVTID_IIC0_RXI : RA4M1_EVTID_IIC1_RXI;

	pDev->IrqRxi = Ra4m1RegisterIntHandler(
		event, Priority, Ra4m1I2cIrqHandler, pDev);
	if (pDev->IrqRxi >= 0)
	{
		pDev->IrqTxi = Ra4m1RegisterIntHandler(
			event + 1U, Priority, Ra4m1I2cIrqHandler, pDev);
	}
	if (pDev->IrqTxi >= 0)
	{
		pDev->IrqTei = Ra4m1RegisterIntHandler(
			event + 2U, Priority, Ra4m1I2cIrqHandler, pDev);
	}
	if (pDev->IrqTei >= 0)
	{
		pDev->IrqEei = Ra4m1RegisterIntHandler(
			event + 3U, Priority, Ra4m1I2cIrqHandler, pDev);
	}

	if (pDev->IrqEei < 0)
	{
		Ra4m1I2cUnregister(pDev);
		return false;
	}

	pDev->bRegistered = true;

	return true;
}

// ---------------------------------------------------------------------------
// DeviceIntrf control
// ---------------------------------------------------------------------------

static void Ra4m1I2cDisable(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	dev->pReg->ICIER = 0U;

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_MASTER)
	{
		if (!dev->bAsync)
		{
			Ra4m1I2cMasterRelease(dev);
		}
	}
	else if (dev->SState != RA4M1_I2C_S_IDLE)
	{
		Ra4m1I2cSlaveComplete(dev);
	}

	Ra4m1I2cIceOff(dev->pReg);
	dev->bEnabled = false;
}

// The peripheral owns SCL/SDA when PMR is enabled. Respect the pin
// directions requested by the common IOsonata example instead of forcing
// both lines to input (PDR=0), which can prevent master SCL drive.
static void Ra4m1I2cPinConfig(const I2CCfg_t * const pCfg)
{
	if (pCfg->pIOPinMap == NULL)
	{
		return;
	}

	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		const IOPinCfg_t *pin = &pCfg->pIOPinMap[i];

		// Release open-drain lines before switching the pad to IIC mode.
		// The PFS driver preserves PODR, so a reset-low output latch must
		// not be carried into an active master SCL output.
		IOPinSet(pin->PortNo, pin->PinNo);
		// RA4M1 IIC bus needs a high idle level. Request the internal
		// pull-up for both SDA and SCL. The IIC block drives the pad by
		// itself, and the pull-up does not act while PDR is set, so PDR is
		// kept clear whatever direction the pin map lists (the loopback log
		// showed SCLI low at idle for an IOPINDIR_OUTPUT SCL entry while the
		// SDA entry with PDR clear read high).
		IOPinConfig(pin->PortNo, pin->PinNo, pin->PinOp, IOPINDIR_BI,
			IOPINRES_PULLUP, pin->Type);
	}
}

static void Ra4m1I2cEnable(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	Ra4m1I2cPinConfig(&dev->pI2cDev->Cfg);
	dev->bEnabled = true;
	Ra4m1I2cConfigure(dev);
}

static bool Ra4m1I2cSclHigh(Ra4m1I2cDev_t * const pDev,
	const IOPinCfg_t * const pScl)
{
	int32_t count = Ra4m1I2cTimeout(pDev);

	while (count > 0)
	{
		if (IOPinRead(pScl->PortNo, pScl->PinNo) != 0)
		{
			return true;
		}

		count--;
	}

	return false;
}

static void Ra4m1I2cBusDelay(void)
{
	int32_t count = (int32_t)((SystemCoreClockGet() + 199999U) / 200000U);

	if (count == 0)
	{
		count = 1;
	}

	while (count > 0)
	{
		__NOP();
		count--;
	}
}

// Free a bus held by a stuck slave: up to nine SCL pulses on the pins as
// open-drain GPIO, then a STOP, then back to the peripheral function.
static void Ra4m1I2cReset(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	const bool enabled = dev->bEnabled;

	dev->pReg->ICIER = 0U;
	Ra4m1I2cIceOff(dev->pReg);
	dev->bOpen = false;
	dev->bAsync = false;
	dev->MState = RA4M1_I2C_M_IDLE;
	dev->SState = RA4M1_I2C_S_IDLE;

	const IOPinCfg_t *sda = &dev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX];
	const IOPinCfg_t *scl = &dev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX];

	IOPinSet(sda->PortNo, sda->PinNo);
	IOPinSet(scl->PortNo, scl->PinNo);

	IOPinConfig(sda->PortNo, sda->PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT,
		sda->Res, IOPINTYPE_OPENDRAIN);
	IOPinConfig(scl->PortNo, scl->PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT,
		scl->Res, IOPINTYPE_OPENDRAIN);

	bool high = Ra4m1I2cSclHigh(dev, scl);

	for (int i = 0; high && i < 9 && IOPinRead(sda->PortNo, sda->PinNo) == 0; i++)
	{
		IOPinClear(scl->PortNo, scl->PinNo);
		Ra4m1I2cBusDelay();
		IOPinSet(scl->PortNo, scl->PinNo);
		high = Ra4m1I2cSclHigh(dev, scl);
		Ra4m1I2cBusDelay();
	}

	if (high)
	{
		IOPinClear(scl->PortNo, scl->PinNo);
		IOPinClear(sda->PortNo, sda->PinNo);
		Ra4m1I2cBusDelay();
		IOPinSet(scl->PortNo, scl->PinNo);

		if (Ra4m1I2cSclHigh(dev, scl))
		{
			Ra4m1I2cBusDelay();
			IOPinSet(sda->PortNo, sda->PinNo);
			Ra4m1I2cBusDelay();
		}
	}

	IOPinSet(sda->PortNo, sda->PinNo);
	IOPinSet(scl->PortNo, scl->PinNo);
	Ra4m1I2cPinConfig(&dev->pI2cDev->Cfg);

	dev->bEnabled = enabled;
	Ra4m1I2cConfigure(dev);
}

static void Ra4m1I2cPowerOff(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	Ra4m1I2cDisable(pIntrf);

	if (dev->bRegistered)
	{
		Ra4m1I2cUnregister(dev);
	}

	IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);

	uint32_t state = DisableInterrupt();
	Ra4m1I2cModuleStart(dev, false);
	dev->pI2cDev = NULL;
	pIntrf->pDevData = NULL;
	pIntrf->EnCnt = 0;
	EnableInterrupt(state);
}

static void *Ra4m1I2cGetHandle(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	return dev != NULL ? dev->pI2cDev : NULL;
}

static uint32_t Ra4m1I2cGetRate(DevIntrf_t * const pIntrf)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	return dev != NULL ? dev->pI2cDev->Cfg.Rate : 0U;
}

static uint32_t Ra4m1I2cSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	Ra4m1I2cDev_t *dev = Ra4m1I2cHandle(pIntrf);

	if (dev == NULL || dev->bOpen || dev->bAsync ||
		dev->MState != RA4M1_I2C_M_IDLE || dev->SState != RA4M1_I2C_S_IDLE)
	{
		return 0U;
	}

	uint8_t cks = 0U;
	uint8_t brl = 0U;
	uint8_t brh = 0U;
	uint32_t actual = 0U;

	if (!Ra4m1I2cRate(Rate, &cks, &brl, &brh, &actual))
	{
		return 0U;
	}

	dev->Cks = cks;
	dev->Brl = brl;
	dev->Brh = brh;
	dev->pI2cDev->Cfg.Rate = actual;
	Ra4m1I2cConfigure(dev);

	return actual;
}

static bool Ra4m1I2cPin(Ra4m1I2cDev_t * const pDev, const IOPinCfg_t * const pPin,
	bool bScl)
{
	if (!Ra4m1PinValid(pPin->PortNo, pPin->PinNo))
	{
		return false;
	}

	if (pPin->PinOp != IOPINOP_FUNC6 || pPin->Type != IOPINTYPE_OPENDRAIN)
	{
		return false;
	}

	if (pPin->Res != IOPINRES_NONE && pPin->Res != IOPINRES_PULLUP)
	{
		return false;
	}

	if (pDev->DevNo == 0)
	{
		if (bScl)
		{
			return (pPin->PortNo == 2 && pPin->PinNo == 4) ||
				   (pPin->PortNo == 4 && pPin->PinNo == 0) ||
				   (pPin->PortNo == 4 && pPin->PinNo == 8);
		}

		return (pPin->PortNo == 4 && pPin->PinNo == 1) ||
			   (pPin->PortNo == 4 && pPin->PinNo == 7);
	}

	if (bScl)
	{
		return (pPin->PortNo == 1 && pPin->PinNo == 0) ||
			   (pPin->PortNo == 2 && pPin->PinNo == 5);
	}

	return (pPin->PortNo == 1 && pPin->PinNo == 1) ||
		   (pPin->PortNo == 2 && pPin->PinNo == 6);
}

static bool Ra4m1I2cPinMap(Ra4m1I2cDev_t * const pDev, const I2CCfg_t * const pCfg)
{
	const IOPinCfg_t *sda = &pCfg->pIOPinMap[I2C_SDA_IOPIN_IDX];
	const IOPinCfg_t *scl = &pCfg->pIOPinMap[I2C_SCL_IOPIN_IDX];

	if (sda->PortNo == scl->PortNo && sda->PinNo == scl->PinNo)
	{
		return false;
	}

	return Ra4m1I2cPin(pDev, sda, false) &&
		   Ra4m1I2cPin(pDev, scl, true);
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= RA4M1_I2C_COUNT ||
		pCfg->Type != I2CTYPE_STANDARD ||
		(pCfg->Mode != I2CMODE_MASTER && pCfg->Mode != I2CMODE_SLAVE) ||
		pCfg->AddrType != I2CADDR_TYPE_NORMAL ||
		pCfg->bDmaEn ||
		pCfg->pIOPinMap == NULL ||
		pCfg->NbIOPins != 2 ||
		pCfg->MaxRetry < 0)
	{
		return false;
	}

	if (pCfg->Mode == I2CMODE_SLAVE)
	{
		// The slave is serviced by interrupt and owns one to three
		// distinct 7-bit addresses, one per slave address register.
		if (!pCfg->bIntEn || pCfg->NbSlaveAddr < 1 ||
			pCfg->NbSlaveAddr > RA4M1_I2C_MAX_SLAVE_ADDR)
		{
			return false;
		}

		for (int i = 0; i < pCfg->NbSlaveAddr; i++)
		{
			if (pCfg->SlaveAddr[i] < 0x08U || pCfg->SlaveAddr[i] > 0x77U)
			{
				return false;
			}

			for (int j = 0; j < i; j++)
			{
				if (pCfg->SlaveAddr[i] == pCfg->SlaveAddr[j])
				{
					return false;
				}
			}
		}
	}

	Ra4m1I2cDev_t *dev = &s_Ra4m1I2cDev[pCfg->DevNo];

	if (!Ra4m1I2cPinMap(dev, pCfg))
	{
		return false;
	}

	uint8_t cks = 0U;
	uint8_t brl = 0U;
	uint8_t brh = 0U;
	uint32_t actual = 0U;

	if (!Ra4m1I2cRate(pCfg->Rate, &cks, &brl, &brh, &actual))
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	if ((dev->pI2cDev != NULL && dev->pI2cDev != pDev) ||
		dev->bOpen ||
		s_Ra4m1I2cDev[1 - pCfg->DevNo].pI2cDev == pDev)
	{
		EnableInterrupt(state);
		return false;
	}

	dev->pI2cDev = pDev;
	dev->Cks = cks;
	dev->Brl = brl;
	dev->Brh = brh;
	dev->bEnabled = true;
	dev->bOpen = false;
	dev->bRegistered = false;
	dev->bError = false;
	dev->bAsync = false;
	dev->MState = RA4M1_I2C_M_IDLE;
	dev->SState = RA4M1_I2C_S_IDLE;
	Ra4m1I2cModuleStart(dev, true);
	EnableInterrupt(state);

	pDev->Cfg = *pCfg;
	pDev->Cfg.Rate = actual;

	for (int i = 0; i < I2C_SLAVEMODE_MAX_ADDR; i++)
	{
		pDev->pRRData[i] = NULL;
		pDev->RRDataLen[i] = 0;
		pDev->pTRBuff[i] = NULL;
		pDev->TRBuffLen[i] = 0;
	}

	Ra4m1I2cPinConfig(pCfg);

	DevIntrf_t *intrf = &pDev->DevIntrf;
	intrf->pDevData = dev;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->EnCnt = 1;
	intrf->bDma = false;
	intrf->bIntEn = pCfg->bIntEn;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = Ra4m1I2cEnable;
	intrf->Disable = Ra4m1I2cDisable;
	intrf->PowerOff = Ra4m1I2cPowerOff;
	intrf->Reset = Ra4m1I2cReset;
	intrf->GetHandle = Ra4m1I2cGetHandle;
	intrf->GetRate = Ra4m1I2cGetRate;
	intrf->SetRate = Ra4m1I2cSetRate;
	intrf->StartRx = Ra4m1I2cStartRx;
	intrf->StartTx = Ra4m1I2cStartTx;
	intrf->StopRx = Ra4m1I2cStop;
	intrf->StopTx = Ra4m1I2cStop;
	intrf->RxData = Ra4m1I2cRx;
	intrf->TxData = Ra4m1I2cTx;
	intrf->TxSrData = Ra4m1I2cTx;
	atomic_flag_clear(&intrf->bBusy);

	if (pCfg->bIntEn && !Ra4m1I2cRegister(dev, pCfg->IntPrio))
	{
		dev->bEnabled = false;
		dev->pReg->ICIER = 0U;
		Ra4m1I2cIceOff(dev->pReg);
		IOPinDis(pCfg->pIOPinMap, pCfg->NbIOPins);
		dev->pI2cDev = NULL;
		intrf->pDevData = NULL;
		intrf->EnCnt = 0;
		Ra4m1I2cModuleStart(dev, false);
		return false;
	}

	Ra4m1I2cConfigure(dev);

	if (dev->bError || (dev->pReg->ICCR1 & RA4M1_I2C_CR1_ICE) == 0U)
	{
		dev->pReg->ICIER = 0U;
		Ra4m1I2cIceOff(dev->pReg);

		if (dev->bRegistered)
		{
			Ra4m1I2cUnregister(dev);
		}

		IOPinDis(pCfg->pIOPinMap, pCfg->NbIOPins);
		dev->bEnabled = false;
		dev->pI2cDev = NULL;
		intrf->pDevData = NULL;
		intrf->EnCnt = 0;

		uint32_t irqState = DisableInterrupt();
		Ra4m1I2cModuleStart(dev, false);
		EnableInterrupt(irqState);
		return false;
	}

	return true;
}
