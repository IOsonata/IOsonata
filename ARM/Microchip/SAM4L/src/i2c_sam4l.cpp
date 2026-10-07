/**-------------------------------------------------------------------------
@file	i2c_sam4l.cpp

@brief	I2C implementation for the SAM4L TWIM/TWIS controllers.

The SAM4L separates master (TWIM) and slave (TWIS) functions. IOsonata keeps
one I2CDev_t API and selects the matching register block from I2CCfg_t::Mode.

Master mode supports polling, interrupt-driven, and PDCA-backed transfers,
7-bit and 10-bit addressing, transfer lengths beyond the 8-bit NBYTES field,
and the write/repeated-start/read sequence used by DeviceIntrfRead(). Interrupt TX
copies the complete caller transfer into controller-owned static storage before
returning so no caller buffer is retained by the ISR.

Slave mode is interrupt driven, with optional PDCA data movement. The TWIS
address-match clock stretching is used so application callbacks can install the
read/write buffers before the transfer continues.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#include "sam4lxxx.h"
#include "component/component_pm.h"
#include "component/component_pdca.h"
#include "component/component_twim.h"
#include "component/component_twis.h"

#include "coredev/i2c.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"

#define SAM4L_I2C_MASTER_COUNT		4
#define SAM4L_I2C_SLAVE_COUNT		2
#define SAM4L_I2C_MAX_NBYTES		255
#define SAM4L_I2C_INT_TX_BUFFER_SIZE	256
#define SAM4L_I2C_WAIT_COUNT		1000000U

// PDCA channels 0..3 are reserved by the SAM4L UART port for USART0..3 TX.
// I2C master uses a fixed RX/TX pair per TWIM instance from channels 4..11.
#define SAM4L_I2C_PDCA_RX_CHAN(n)	(4U + ((uint32_t)(n) << 1U))
#define SAM4L_I2C_PDCA_TX_CHAN(n)	(5U + ((uint32_t)(n) << 1U))
#define SAM4L_I2C_PDCA_SLAVE_RX_CHAN(n)	(12U + ((uint32_t)(n) << 1U))
#define SAM4L_I2C_PDCA_SLAVE_TX_CHAN(n)	(13U + ((uint32_t)(n) << 1U))
#define SAM4L_PDCA_PID_TWIM_RX(n)	(5U + (uint32_t)(n))
#define SAM4L_PDCA_PID_TWIM_TX(n)	(23U + (uint32_t)(n))
#define SAM4L_PDCA_PID_TWIS_RX(n)	(9U + (uint32_t)(n))
#define SAM4L_PDCA_PID_TWIS_TX(n)	(27U + (uint32_t)(n))
#define SAM4L_PDCA_MAX_COUNT		0xFFFFU

#define SAM4L_TWIM_ERROR_MASK \
	(TWIM_SR_ANAK | TWIM_SR_DNAK | TWIM_SR_ARBLST | TWIM_SR_TOUT | TWIM_SR_PECERR)

#define SAM4L_TWIM_ERROR_IRQ_MASK \
	(TWIM_IER_ANAK | TWIM_IER_DNAK | TWIM_IER_ARBLST | TWIM_IER_TOUT | TWIM_IER_PECERR)

#define SAM4L_TWIM_TX_IRQ_MASK \
	(TWIM_IER_TXRDY | TWIM_IER_CCOMP | SAM4L_TWIM_ERROR_IRQ_MASK)

#define SAM4L_TWIM_RX_IRQ_MASK \
	(TWIM_IER_RXRDY | TWIM_IER_CCOMP | SAM4L_TWIM_ERROR_IRQ_MASK)

#define SAM4L_TWIM_DMA_IRQ_MASK \
	(TWIM_IER_CCOMP | SAM4L_TWIM_ERROR_IRQ_MASK)

#define SAM4L_TWIS_ERROR_MASK \
	(TWIS_SR_URUN | TWIS_SR_ORUN | TWIS_SR_SMBTOUT | TWIS_SR_SMBPECERR | TWIS_SR_BUSERR)

#define SAM4L_TWIS_BASE_IRQ_MASK \
	(TWIS_IER_SAM | TWIS_IER_TCOMP | TWIS_IER_REP | \
	 TWIS_IER_URUN | TWIS_IER_ORUN | TWIS_IER_SMBTOUT | TWIS_IER_SMBPECERR | TWIS_IER_BUSERR)

typedef enum {
	SAM4L_I2C_OP_NONE,
	SAM4L_I2C_OP_MASTER_TX,
	SAM4L_I2C_OP_MASTER_RX_PREAMBLE,
	SAM4L_I2C_OP_MASTER_RX,
} SAM4L_I2C_OP;

typedef struct {
	int DevNo;
	uint32_t MasterPbaMask;
	uint32_t SlavePbaMask;
	IRQn_Type MasterIrq;
	IRQn_Type SlaveIrq;
	Twim *pMReg;
	Twis *pSReg;
	I2CDev_t *pI2cDev;
	uint8_t TxBuffer[SAM4L_I2C_INT_TX_BUFFER_SIZE];

	uint16_t DevAddr;
	const uint8_t *pTxData;
	uint8_t *pRxData;
	int TxRemain;
	int RxRemain;
	int ChunkRemain;
	int ChunkTotal;
	int TxCount;
	int RxCount;
	SAM4L_I2C_OP Op;
	bool NeedStart;
	bool BusHeld;
	bool LastError;
	bool CommandPhase;
	bool CombinedRead;
	int SrLen;
	bool TenBitReadReady;

	int SlaveRxCount;
	int SlaveTxCount;
	bool SlaveRxActive;
	bool SlaveTxActive;
} SAM4L_I2CDEV;

static SAM4L_I2CDEV s_Sam4lI2CDev[SAM4L_I2C_MASTER_COUNT] = {
	{
		.DevNo = 0,
		.MasterPbaMask = PM_PBAMASK_TWIM0,
		.SlavePbaMask = PM_PBAMASK_TWIS0,
		.MasterIrq = TWIM0_IRQn,
		.SlaveIrq = TWIS0_IRQn,
		.pMReg = SAM4L_TWIM0,
		.pSReg = SAM4L_TWIS0,
	},
	{
		.DevNo = 1,
		.MasterPbaMask = PM_PBAMASK_TWIM1,
		.SlavePbaMask = PM_PBAMASK_TWIS1,
		.MasterIrq = TWIM1_IRQn,
		.SlaveIrq = TWIS1_IRQn,
		.pMReg = SAM4L_TWIM1,
		.pSReg = SAM4L_TWIS1,
	},
	{
		.DevNo = 2,
		.MasterPbaMask = PM_PBAMASK_TWIM2,
		.SlavePbaMask = 0,
		.MasterIrq = TWIM2_IRQn,
		.SlaveIrq = (IRQn_Type)-1,
		.pMReg = SAM4L_TWIM2,
		.pSReg = nullptr,
	},
	{
		.DevNo = 3,
		.MasterPbaMask = PM_PBAMASK_TWIM3,
		.SlavePbaMask = 0,
		.MasterIrq = TWIM3_IRQn,
		.SlaveIrq = (IRQn_Type)-1,
		.pMReg = SAM4L_TWIM3,
		.pSReg = nullptr,
	},
};

static void Sam4lI2CMasterRecover(SAM4L_I2CDEV *dev);

static inline void Sam4lPmWrite(volatile uint32_t *pReg, uint32_t Value)
{
	const uint32_t offset = (uint32_t)pReg - (uint32_t)SAM4L_PM;
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAU) | PM_UNLOCK_ADDR(offset);
	*pReg = Value;
}

static void Sam4lI2CPdcaClockEnable(void)
{
	Sam4lPmWrite(&SAM4L_PM->PM_HSBMASK,
		SAM4L_PM->PM_HSBMASK | PM_HSBMASK_PDCA);
	Sam4lPmWrite(&SAM4L_PM->PM_PBBMASK,
		SAM4L_PM->PM_PBBMASK | PM_PBBMASK_PDCA);
}

static PdcaChannel *Sam4lI2CPdcaRxChannel(const SAM4L_I2CDEV *dev)
{
	return &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_I2C_PDCA_RX_CHAN(dev->DevNo)];
}

static PdcaChannel *Sam4lI2CPdcaTxChannel(const SAM4L_I2CDEV *dev)
{
	return &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_I2C_PDCA_TX_CHAN(dev->DevNo)];
}

static PdcaChannel *Sam4lI2CSlavePdcaRxChannel(const SAM4L_I2CDEV *dev)
{
	return &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_I2C_PDCA_SLAVE_RX_CHAN(dev->DevNo)];
}

static PdcaChannel *Sam4lI2CSlavePdcaTxChannel(const SAM4L_I2CDEV *dev)
{
	return &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_I2C_PDCA_SLAVE_TX_CHAN(dev->DevNo)];
}

static void Sam4lI2CPdcaChannelInit(PdcaChannel *chan, uint32_t PeripheralId)
{
	chan->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
	chan->PDCA_PSR = PDCA_PSR_PID(PeripheralId);
	chan->PDCA_MR = PDCA_MR_SIZE_BYTE;
	chan->PDCA_MAR = 0U;
	chan->PDCA_TCR = 0U;
	chan->PDCA_MARR = 0U;
	chan->PDCA_TCRR = 0U;
	chan->PDCA_IDR = 0xFFFFFFFFU;
	(void)chan->PDCA_ISR;
}

static void Sam4lI2CPdcaInit(SAM4L_I2CDEV *dev)
{
	Sam4lI2CPdcaClockEnable();
	Sam4lI2CPdcaChannelInit(Sam4lI2CPdcaRxChannel(dev),
		SAM4L_PDCA_PID_TWIM_RX(dev->DevNo));
	Sam4lI2CPdcaChannelInit(Sam4lI2CPdcaTxChannel(dev),
		SAM4L_PDCA_PID_TWIM_TX(dev->DevNo));
}

static void Sam4lI2CSlavePdcaInit(SAM4L_I2CDEV *dev)
{
	Sam4lI2CPdcaClockEnable();
	Sam4lI2CPdcaChannelInit(Sam4lI2CSlavePdcaRxChannel(dev),
		SAM4L_PDCA_PID_TWIS_RX(dev->DevNo));
	Sam4lI2CPdcaChannelInit(Sam4lI2CSlavePdcaTxChannel(dev),
		SAM4L_PDCA_PID_TWIS_TX(dev->DevNo));

	const IRQn_Type irq = (IRQn_Type)(PDCA_0_IRQn +
		SAM4L_I2C_PDCA_SLAVE_RX_CHAN(dev->DevNo));
	NVIC_ClearPendingIRQ(irq);
	NVIC_SetPriority(irq, dev->pI2cDev->Cfg.IntPrio);
	NVIC_EnableIRQ(irq);
}

static void Sam4lI2CSlavePdcaStop(SAM4L_I2CDEV *dev)
{
	PdcaChannel *rx = Sam4lI2CSlavePdcaRxChannel(dev);
	PdcaChannel *tx = Sam4lI2CSlavePdcaTxChannel(dev);
	rx->PDCA_CR = PDCA_CR_TDIS;
	tx->PDCA_CR = PDCA_CR_TDIS;
	rx->PDCA_IDR = 0xFFFFFFFFU;
	tx->PDCA_IDR = 0xFFFFFFFFU;
}

static void Sam4lI2CPdcaStop(SAM4L_I2CDEV *dev)
{
	PdcaChannel *rx = Sam4lI2CPdcaRxChannel(dev);
	PdcaChannel *tx = Sam4lI2CPdcaTxChannel(dev);
	rx->PDCA_CR = PDCA_CR_TDIS;
	tx->PDCA_CR = PDCA_CR_TDIS;
	rx->PDCA_IDR = 0xFFFFFFFFU;
	tx->PDCA_IDR = 0xFFFFFFFFU;
}

static bool Sam4lI2CPdcaWait(SAM4L_I2CDEV *dev, PdcaChannel *chan)
{
	uint32_t timeout = SAM4L_I2C_WAIT_COUNT;

	do
	{
		const uint32_t isr = chan->PDCA_ISR;
		if ((isr & PDCA_ISR_TERR) != 0U)
		{
			chan->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
			Sam4lI2CMasterRecover(dev);
			return false;
		}

		const uint32_t sr = dev->pMReg->TWIM_SR;
		if ((sr & SAM4L_TWIM_ERROR_MASK) != 0U)
		{
			chan->PDCA_CR = PDCA_CR_TDIS;
			Sam4lI2CMasterRecover(dev);
			return false;
		}

		if (chan->PDCA_TCR == 0U)
			return true;
	} while (--timeout != 0U);

	chan->PDCA_CR = PDCA_CR_TDIS;
	Sam4lI2CMasterRecover(dev);
	return false;
}

static inline uint32_t Sam4lI2CClockMask(const SAM4L_I2CDEV *dev)
{
	return dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE ?
		dev->SlavePbaMask : dev->MasterPbaMask;
}

static void Sam4lI2CClockEnable(SAM4L_I2CDEV *dev)
{
	const uint32_t mask = Sam4lI2CClockMask(dev);
	Sam4lPmWrite(&SAM4L_PM->PM_PBAMASK, SAM4L_PM->PM_PBAMASK | mask);
}

static void Sam4lI2CClockDisable(SAM4L_I2CDEV *dev)
{
	const uint32_t mask = Sam4lI2CClockMask(dev);
	Sam4lPmWrite(&SAM4L_PM->PM_PBAMASK, SAM4L_PM->PM_PBAMASK & ~mask);
}

static inline bool Sam4lI2CAddressValid(const I2CDev_t *i2c, uint32_t Address)
{
	if (i2c->Cfg.AddrType == I2CADDR_TYPE_EXT)
		return Address <= 0x3FFU;
	return Address <= 0x7FU;
}

static uint32_t Sam4lI2CGetRate(DevIntrf_t * const pDev)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	return dev->pI2cDev->Cfg.Rate;
}

static uint32_t Sam4lI2CSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	I2CDev_t *i2c = dev->pI2cDev;
	const uint32_t pba = SystemPeriphClockGet(0);

	if (Rate == 0U || pba == 0U || Rate > I2C_SCL_FAST_MODE_PLUS_MAX_SPEED * 1000U)
		return 0U;

	uint32_t setupNs;
	uint32_t lowNs;
	uint32_t highNs;
	uint32_t staStoNs;

	if (Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U)
	{
		setupNs = I2C_TSUDAT_STDMODE_MIN;
		lowNs = I2C_SCL_TLOW_STD_MODE_MIN;
		highNs = I2C_SCL_THIGH_STD_MODE_MIN;
		staStoNs = 4700U; // max(tHD;STA, tSU;STA, tSU;STO) in Standard mode
	}
	else if (Rate <= I2C_SCL_FAST_MODE_MAX_SPEED * 1000U)
	{
		setupNs = I2C_TSUDAT_FASTMODE_MIN;
		lowNs = I2C_SCL_TLOW_FAST_MODE_MIN;
		highNs = I2C_SCL_THIGH_FAST_MODE_MIN;
		staStoNs = 600U;
	}
	else
	{
		setupNs = I2C_TSUDAT_FASTMODEPLUS_MIN;
		lowNs = I2C_SCL_TLOW_FAST_MODE_PLUS_MIN;
		highNs = I2C_SCL_THIGH_FAST_MODE_PLUS_MIN;
		staStoNs = 260U;
	}

	if (i2c->Cfg.Mode == I2CMODE_SLAVE)
	{
		// TWIS SUDAT is counted directly from CLK_TWIS, not the EXP prescaler.
		uint32_t cycles = (uint32_t)(((uint64_t)pba * setupNs + 999999999ULL) /
			1000000000ULL);
		if (cycles < 2U)
			cycles = 2U;
		if (cycles > 255U)
			cycles = 255U;

		dev->pSReg->TWIS_TR = TWIS_TR_SUDAT(cycles);
		dev->pSReg->TWIS_NBYTES = 0U; // required in plain I2C mode
		i2c->Cfg.Rate = Rate;
		return Rate;
	}

	// CWGR counters run from fPBA / 2^(EXP+1). DATA is used for both
	// tHD;DAT and tSU;DAT, so both DATA intervals contribute to SCL low time.
	for (uint32_t exp = 0U; exp <= 7U; ++exp)
	{
		const uint32_t prescale = 1U << (exp + 1U);
		const uint64_t nsDenom = 1000000000ULL * prescale;

		uint32_t data = (uint32_t)(((uint64_t)pba * setupNs + nsDenom - 1ULL) /
			nsDenom);
		uint32_t low = (uint32_t)(((uint64_t)pba * lowNs + nsDenom - 1ULL) /
			nsDenom);
		uint32_t high = (uint32_t)(((uint64_t)pba * highNs + nsDenom - 1ULL) /
			nsDenom);
		uint32_t staSto = (uint32_t)(((uint64_t)pba * staStoNs + nsDenom - 1ULL) /
			nsDenom);

		if (data == 0U) data = 1U;
		if (low == 0U) low = 1U;
		if (high == 0U) high = 1U;
		if (staSto == 0U) staSto = 1U;

		if (data > 15U || low > 255U || high > 255U || staSto > 255U)
			continue;

		const uint64_t rateDenom = (uint64_t)Rate * prescale;
		uint32_t target = (uint32_t)(((uint64_t)pba + rateDenom - 1ULL) /
			rateDenom);
		uint32_t total = low + high + (data << 1);

		// Do not exceed the requested SCL rate. Put any spare cycles into the
		// LOW/HIGH phases while preserving their minimum bus timings.
		if (total < target)
		{
			uint32_t extra = target - total;
			low += (extra + 1U) >> 1U;
			high += extra >> 1U;
		}

		if (low > 255U || high > 255U)
			continue;

		dev->pMReg->TWIM_CWGR =
			TWIM_CWGR_LOW(low) |
			TWIM_CWGR_HIGH(high) |
			TWIM_CWGR_STASTO(staSto) |
			TWIM_CWGR_DATA(data) |
			TWIM_CWGR_EXP(exp);

		const uint32_t actual = pba /
			(prescale * (low + high + (data << 1)));
		i2c->Cfg.Rate = actual;
		return actual;
	}

	return 0U;
}

static void Sam4lI2CMasterHwReset(SAM4L_I2CDEV *dev)
{
	Twim *reg = dev->pMReg;
	reg->TWIM_IDR = 0xFFFFFFFFU;
	reg->TWIM_CR = TWIM_CR_MEN;
	reg->TWIM_CR = TWIM_CR_SWRST;
	reg->TWIM_CR = TWIM_CR_MDIS;
	reg->TWIM_CMDR = 0;
	reg->TWIM_NCMDR = 0;
	reg->TWIM_SCR = 0xFFFFFFFFU;
	(void)Sam4lI2CSetRate(&dev->pI2cDev->DevIntrf, dev->pI2cDev->Cfg.Rate);
	reg->TWIM_CR = TWIM_CR_MEN;
}

static void Sam4lI2CMasterRecover(SAM4L_I2CDEV *dev)
{
	if (dev->pI2cDev != nullptr && dev->pI2cDev->DevIntrf.bDma)
		Sam4lI2CPdcaStop(dev);

	dev->LastError = true;
	dev->BusHeld = false;
	dev->NeedStart = true;
	dev->TenBitReadReady = false;
	dev->Op = SAM4L_I2C_OP_NONE;
	Sam4lI2CMasterHwReset(dev);
}

static bool Sam4lI2CMasterWait(SAM4L_I2CDEV *dev, uint32_t Mask)
{
	uint32_t timeout = SAM4L_I2C_WAIT_COUNT;
	do
	{
		const uint32_t sr = dev->pMReg->TWIM_SR;
		if ((sr & SAM4L_TWIM_ERROR_MASK) != 0U)
		{
			Sam4lI2CMasterRecover(dev);
			return false;
		}
		if ((sr & Mask) == Mask)
			return true;
	} while (--timeout != 0U);

	Sam4lI2CMasterRecover(dev);
	return false;
}

static uint32_t Sam4lI2CMasterCommand(SAM4L_I2CDEV *dev, bool Read,
	int Count, bool Start, bool Stop, bool AckLast, bool RepSame)
{
	uint32_t cmd = TWIM_CMDR_SADR(dev->DevAddr) |
		TWIM_CMDR_NBYTES((uint32_t)Count) |
		TWIM_CMDR_VALID;

	if (Read)
		cmd |= TWIM_CMDR_READ;
	if (Start)
		cmd |= TWIM_CMDR_START;
	if (Stop)
		cmd |= TWIM_CMDR_STOP;
	if (AckLast)
		cmd |= TWIM_CMDR_ACKLAST;
	if (dev->pI2cDev->Cfg.AddrType == I2CADDR_TYPE_EXT)
	{
		cmd |= TWIM_CMDR_TENBIT;
		if (RepSame)
			cmd |= TWIM_CMDR_REPSAME;
	}
	return cmd;
}

static void Sam4lI2CMasterIssue(SAM4L_I2CDEV *dev, uint32_t Command)
{
	dev->pMReg->TWIM_SCR = TWIM_SCR_CCOMP |
		TWIM_SCR_ANAK | TWIM_SCR_DNAK | TWIM_SCR_ARBLST |
		TWIM_SCR_TOUT | TWIM_SCR_PECERR;
	dev->pMReg->TWIM_CMDR = Command;
}

static bool Sam4lI2CMasterWaitCommand(SAM4L_I2CDEV *dev)
{
	if (!Sam4lI2CMasterWait(dev, TWIM_SR_CCOMP))
		return false;
	dev->pMReg->TWIM_SCR = TWIM_SCR_CCOMP;
	return true;
}

static bool Sam4lI2CMasterStartTxChunk(SAM4L_I2CDEV *dev)
{
	if (dev->TxRemain <= 0)
		return false;

	const int chunk = dev->TxRemain > SAM4L_I2C_MAX_NBYTES ?
		SAM4L_I2C_MAX_NBYTES : dev->TxRemain;
	const bool final = dev->TxRemain <= SAM4L_I2C_MAX_NBYTES;
	const bool stop = final && !dev->pI2cDev->DevIntrf.bNoStop;
	const uint32_t cmd = Sam4lI2CMasterCommand(dev, false, chunk,
		dev->NeedStart, stop, false, false);

	dev->ChunkRemain = chunk;
	dev->ChunkTotal = chunk;
	dev->NeedStart = false;
	Sam4lI2CMasterIssue(dev, cmd);
	return true;
}

static bool Sam4lI2CMasterStartRxChunk(SAM4L_I2CDEV *dev)
{
	if (dev->RxRemain <= 0)
		return false;

	const int chunk = dev->RxRemain > SAM4L_I2C_MAX_NBYTES ?
		SAM4L_I2C_MAX_NBYTES : dev->RxRemain;
	const bool final = dev->RxRemain <= SAM4L_I2C_MAX_NBYTES;
	const bool stop = final && !dev->pI2cDev->DevIntrf.bNoStop;
	const bool ackLast = !final;
	const bool repSame = dev->NeedStart && dev->TenBitReadReady;
	const uint32_t cmd = Sam4lI2CMasterCommand(dev, true, chunk,
		dev->NeedStart, stop, ackLast, repSame);

	dev->ChunkRemain = chunk;
	dev->NeedStart = false;
	Sam4lI2CMasterIssue(dev, cmd);
	return true;
}

static bool Sam4lI2CMasterStartRxPreamble(SAM4L_I2CDEV *dev)
{
	const uint32_t cmd = Sam4lI2CMasterCommand(dev, false, 0, true, false, false, false);
	Sam4lI2CMasterIssue(dev, cmd);
	return true;
}

static bool Sam4lI2CStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER ||
		!Sam4lI2CAddressValid(dev->pI2cDev, DevAddr))
		return false;

	dev->DevAddr = (uint16_t)DevAddr;
	dev->LastError = false;
	dev->CommandPhase = false;
	dev->CombinedRead = false;
	dev->SrLen = 0;
	dev->TenBitReadReady = false;
	dev->BusHeld = false;
	dev->NeedStart = true;
	dev->pMReg->TWIM_SCR = 0xFFFFFFFFU;
	return true;
}

static int Sam4lI2CMasterTxPolling(DevIntrf_t * const pDev,
	const uint8_t *pData, int DataLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	dev->pTxData = pData;
	dev->TxRemain = DataLen;
	dev->TxCount = 0;

	while (dev->TxRemain > 0)
	{
		if (!Sam4lI2CMasterStartTxChunk(dev))
			break;

		while (dev->ChunkRemain > 0)
		{
			if (!Sam4lI2CMasterWait(dev, TWIM_SR_TXRDY))
				return dev->TxCount;
			dev->pMReg->TWIM_THR = *dev->pTxData++;
			--dev->ChunkRemain;
			--dev->TxRemain;
		}

		if (!Sam4lI2CMasterWaitCommand(dev))
			return dev->TxCount;
		dev->TxCount += dev->ChunkTotal;
	}

	dev->BusHeld = pDev->bNoStop && !dev->LastError;
	return dev->TxCount;
}

static bool Sam4lI2CMasterStartTxDmaChunk(SAM4L_I2CDEV *dev)
{
	if (dev->TxRemain <= 0)
		return false;

	const int chunk = dev->TxRemain > SAM4L_I2C_MAX_NBYTES ?
		SAM4L_I2C_MAX_NBYTES : dev->TxRemain;
	const bool final = dev->TxRemain <= SAM4L_I2C_MAX_NBYTES;
	const bool stop = final && !dev->pI2cDev->DevIntrf.bNoStop;
	const uint32_t cmd = Sam4lI2CMasterCommand(dev, false, chunk,
		dev->NeedStart, stop, false, false);

	PdcaChannel *chan = Sam4lI2CPdcaTxChannel(dev);
	chan->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
	chan->PDCA_PSR = PDCA_PSR_PID(SAM4L_PDCA_PID_TWIM_TX(dev->DevNo));
	chan->PDCA_MAR = (uint32_t)dev->pTxData;
	chan->PDCA_TCR = (uint32_t)chunk;

	dev->ChunkRemain = chunk;
	dev->ChunkTotal = chunk;
	dev->NeedStart = false;
	Sam4lI2CMasterIssue(dev, cmd);
	chan->PDCA_CR = PDCA_CR_TEN;
	return true;
}

static bool Sam4lI2CMasterStartRxDmaChunk(SAM4L_I2CDEV *dev)
{
	if (dev->RxRemain <= 0)
		return false;

	const int chunk = dev->RxRemain > SAM4L_I2C_MAX_NBYTES ?
		SAM4L_I2C_MAX_NBYTES : dev->RxRemain;
	const bool final = dev->RxRemain <= SAM4L_I2C_MAX_NBYTES;
	const bool stop = final && !dev->pI2cDev->DevIntrf.bNoStop;
	const bool ackLast = !final;
	const bool repSame = dev->NeedStart && dev->TenBitReadReady;
	const uint32_t cmd = Sam4lI2CMasterCommand(dev, true, chunk,
		dev->NeedStart, stop, ackLast, repSame);

	PdcaChannel *chan = Sam4lI2CPdcaRxChannel(dev);
	chan->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
	chan->PDCA_PSR = PDCA_PSR_PID(SAM4L_PDCA_PID_TWIM_RX(dev->DevNo));
	chan->PDCA_MAR = (uint32_t)dev->pRxData;
	chan->PDCA_TCR = (uint32_t)chunk;

	dev->ChunkRemain = chunk;
	dev->ChunkTotal = chunk;
	dev->NeedStart = false;
	Sam4lI2CMasterIssue(dev, cmd);
	chan->PDCA_CR = PDCA_CR_TEN;
	return true;
}

static int Sam4lI2CMasterTxDma(DevIntrf_t * const pDev,
	const uint8_t *pData, int DataLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;

	if (pDev->bIntEn && !pDev->bNoStop &&
		DataLen <= SAM4L_I2C_INT_TX_BUFFER_SIZE)
	{
		// Async TX must own the source bytes after this call returns.
		memcpy(dev->TxBuffer, pData, DataLen);
		dev->pTxData = dev->TxBuffer;
		dev->TxRemain = DataLen;
		dev->TxCount = 0;
		dev->Op = SAM4L_I2C_OP_MASTER_TX;
		pDev->bTxReady = false;

		if (!Sam4lI2CMasterStartTxDmaChunk(dev))
		{
			pDev->bTxReady = true;
			return 0;
		}

		dev->pMReg->TWIM_IER = SAM4L_TWIM_DMA_IRQ_MASK;
		return -1;
	}

	// Oversized async transfers and no-stop transfers remain synchronous so
	// the DMA engine never retains caller-owned TX memory after return.
	dev->pTxData = pData;
	dev->TxRemain = DataLen;
	dev->TxCount = 0;

	while (dev->TxRemain > 0)
	{
		if (!Sam4lI2CMasterStartTxDmaChunk(dev))
			break;

		PdcaChannel *chan = Sam4lI2CPdcaTxChannel(dev);
		if (!Sam4lI2CPdcaWait(dev, chan))
			return dev->TxCount;
		chan->PDCA_CR = PDCA_CR_TDIS;

		if (!Sam4lI2CMasterWaitCommand(dev))
			return dev->TxCount;

		dev->pTxData += dev->ChunkTotal;
		dev->TxRemain -= dev->ChunkTotal;
		dev->TxCount += dev->ChunkTotal;
	}

	dev->BusHeld = pDev->bNoStop && !dev->LastError;
	return dev->TxCount;
}
static int Sam4lI2CMasterTxAsync(DevIntrf_t * const pDev,
	const uint8_t *pData, int DataLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;

	// DeviceIntrf TX buffers are immediate-use. Interrupt mode must therefore
	// take ownership of the bytes before returning. Larger transfers fall back
	// to the synchronous path instead of retaining the caller's pointer.
	if (DataLen > SAM4L_I2C_INT_TX_BUFFER_SIZE)
		return Sam4lI2CMasterTxPolling(pDev, pData, DataLen);

	memcpy(dev->TxBuffer, pData, DataLen);
	dev->pTxData = dev->TxBuffer;
	dev->TxRemain = DataLen;
	dev->TxCount = 0;
	dev->Op = SAM4L_I2C_OP_MASTER_TX;
	pDev->bTxReady = false;

	if (!Sam4lI2CMasterStartTxChunk(dev))
	{
		pDev->bTxReady = true;
		return 0;
	}

	dev->pMReg->TWIM_IER = SAM4L_TWIM_TX_IRQ_MASK;
	return -1;
}

static int Sam4lI2CTxData(DevIntrf_t * const pDev,
	const uint8_t *pData, int DataLen)
{
	if (pData == nullptr || DataLen <= 0)
		return 0;

	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER)
		return 0;

	if (pDev->bDma)
		return Sam4lI2CMasterTxDma(pDev, pData, DataLen);

	return pDev->bIntEn ?
		Sam4lI2CMasterTxAsync(pDev, pData, DataLen) :
		Sam4lI2CMasterTxPolling(pDev, pData, DataLen);
}

static int Sam4lI2CTxSrData(DevIntrf_t * const pDev,
	const uint8_t *pData, int DataLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;

	if (pData == nullptr || DataLen <= 0 ||
		DataLen > SAM4L_I2C_MAX_NBYTES ||
		DataLen > SAM4L_I2C_INT_TX_BUFFER_SIZE)
	{
		dev->CommandPhase = false;
		dev->CombinedRead = false;
		dev->SrLen = 0;
		return 0;
	}

	// SAM4L combined write/read transfers must have the read command queued in
	// NCMDR before the write command finishes. Stage the write/address bytes
	// here; RxData() knows the receive length and can then program CMDR+NCMDR
	// as one connected transfer. No caller pointer is retained.
	memcpy(dev->TxBuffer, pData, DataLen);
	dev->SrLen = DataLen;
	dev->CommandPhase = true;
	dev->CombinedRead = false;
	return DataLen;
}

static bool Sam4lI2CStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER ||
		!Sam4lI2CAddressValid(dev->pI2cDev, DevAddr))
		return false;

	const bool fromCommand = dev->CommandPhase;
	dev->CommandPhase = false;
	dev->CombinedRead = fromCommand && dev->SrLen > 0;
	dev->DevAddr = (uint16_t)DevAddr;
	dev->NeedStart = true;
	dev->TenBitReadReady = false;

	if (!dev->CombinedRead)
	{
		dev->LastError = false;
		dev->BusHeld = false;
		dev->pMReg->TWIM_SCR = 0xFFFFFFFFU;
	}

	return !dev->LastError;
}

static int Sam4lI2CMasterReadCombinedDma(DevIntrf_t * const pDev,
	uint8_t *pBuff, int BuffLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	Twim *reg = dev->pMReg;
	PdcaChannel *rx = Sam4lI2CPdcaRxChannel(dev);

	if (dev->SrLen <= 0 || dev->SrLen > SAM4L_I2C_MAX_NBYTES ||
		pBuff == nullptr || BuffLen <= 0)
		return 0;

	dev->LastError = false;
	dev->BusHeld = false;
	dev->NeedStart = true;
	dev->TenBitReadReady = false;
	dev->RxCount = 0;

	reg->TWIM_IDR = 0xFFFFFFFFU;
	reg->TWIM_SCR = 0xFFFFFFFFU;

	int activeChunk = BuffLen > SAM4L_I2C_MAX_NBYTES ?
		SAM4L_I2C_MAX_NBYTES : BuffLen;
	int remain = BuffLen;
	const bool firstFinal = BuffLen <= SAM4L_I2C_MAX_NBYTES;

	rx->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
	rx->PDCA_PSR = PDCA_PSR_PID(SAM4L_PDCA_PID_TWIM_RX(dev->DevNo));
	rx->PDCA_MAR = (uint32_t)pBuff;
	rx->PDCA_TCR = (uint32_t)activeChunk;

	const uint32_t txcmd = Sam4lI2CMasterCommand(dev, false,
		dev->SrLen, true, false, false, false);
	const uint32_t rxcmd = Sam4lI2CMasterCommand(dev, true,
		activeChunk, true, firstFinal && !pDev->bNoStop, !firstFinal,
		dev->pI2cDev->Cfg.AddrType == I2CADDR_TYPE_EXT);

	reg->TWIM_CMDR = txcmd;
	reg->TWIM_NCMDR = rxcmd;
	rx->PDCA_CR = PDCA_CR_TEN;

	for (int i = 0; i < dev->SrLen; ++i)
	{
		if (!Sam4lI2CMasterWait(dev, TWIM_SR_TXRDY))
			goto done;
		reg->TWIM_THR = dev->TxBuffer[i];
	}

	// The queued read is promoted immediately when the write command completes.
	if (!Sam4lI2CMasterWaitCommand(dev))
		goto done;

	while (remain > 0)
	{
		const int after = remain - activeChunk;
		int nextChunk = 0;

		if (after > 0)
		{
			nextChunk = after > SAM4L_I2C_MAX_NBYTES ?
				SAM4L_I2C_MAX_NBYTES : after;
			const bool nextFinal = after <= SAM4L_I2C_MAX_NBYTES;
			const uint32_t next = Sam4lI2CMasterCommand(dev, true,
				nextChunk, false, nextFinal && !pDev->bNoStop,
				!nextFinal, false);
			reg->TWIM_NCMDR = next;
		}

		if (!Sam4lI2CPdcaWait(dev, rx))
			goto done;
		rx->PDCA_CR = PDCA_CR_TDIS;

		if (!Sam4lI2CMasterWaitCommand(dev))
			goto done;

		dev->RxCount += activeChunk;
		pBuff += activeChunk;
		remain -= activeChunk;

		if (remain <= 0)
			break;

		activeChunk = nextChunk;
		rx->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
		rx->PDCA_MAR = (uint32_t)pBuff;
		rx->PDCA_TCR = (uint32_t)activeChunk;
		rx->PDCA_CR = PDCA_CR_TEN;
	}

done:
	rx->PDCA_CR = PDCA_CR_TDIS;
	dev->CombinedRead = false;
	dev->SrLen = 0;
	dev->BusHeld = pDev->bNoStop && !dev->LastError;
	return dev->RxCount;
}

static int Sam4lI2CMasterReadCombinedPolling(DevIntrf_t * const pDev,
	uint8_t *pBuff, int BuffLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	Twim *reg = dev->pMReg;

	if (dev->SrLen <= 0 || dev->SrLen > SAM4L_I2C_MAX_NBYTES ||
		pBuff == nullptr || BuffLen <= 0)
		return 0;

	dev->LastError = false;
	dev->BusHeld = false;
	dev->NeedStart = true;
	dev->TenBitReadReady = false;
	dev->pRxData = pBuff;
	dev->RxRemain = BuffLen;
	dev->RxCount = 0;

	// Polling owns the peripheral while the connected transfer is active.
	reg->TWIM_IDR = 0xFFFFFFFFU;
	reg->TWIM_SCR = 0xFFFFFFFFU;

	const uint32_t txcmd = Sam4lI2CMasterCommand(dev, false,
		dev->SrLen, true, false, false, false);

	int activeChunk = BuffLen > SAM4L_I2C_MAX_NBYTES ?
		SAM4L_I2C_MAX_NBYTES : BuffLen;
	bool final = BuffLen <= SAM4L_I2C_MAX_NBYTES;
	uint32_t rxcmd = Sam4lI2CMasterCommand(dev, true, activeChunk,
		true, final && !pDev->bNoStop, !final,
		dev->pI2cDev->Cfg.AddrType == I2CADDR_TYPE_EXT);

	// Section 27.8.7.3: queue the read in NCMDR before the write finishes.
	reg->TWIM_CMDR = txcmd;
	reg->TWIM_NCMDR = rxcmd;

	for (int i = 0; i < dev->SrLen; ++i)
	{
		if (!Sam4lI2CMasterWait(dev, TWIM_SR_TXRDY))
			goto done;
		reg->TWIM_THR = dev->TxBuffer[i];
	}

	// This completes the write command; hardware has already promoted NCMDR
	// and generates the repeated START without releasing the bus.
	if (!Sam4lI2CMasterWaitCommand(dev))
		goto done;

	while (dev->RxRemain > 0)
	{
		int remainingAfterActive = dev->RxRemain - activeChunk;
		int queuedChunk = 0;

		if (remainingAfterActive > 0)
		{
			queuedChunk = remainingAfterActive > SAM4L_I2C_MAX_NBYTES ?
				SAM4L_I2C_MAX_NBYTES : remainingAfterActive;
			const bool queuedFinal =
				remainingAfterActive <= SAM4L_I2C_MAX_NBYTES;
			const uint32_t next = Sam4lI2CMasterCommand(dev, true,
				queuedChunk, false,
				queuedFinal && !pDev->bNoStop, !queuedFinal, false);
			reg->TWIM_NCMDR = next;
		}

		for (int i = 0; i < activeChunk; ++i)
		{
			if (!Sam4lI2CMasterWait(dev, TWIM_SR_RXRDY))
				goto done;
			*dev->pRxData++ = (uint8_t)reg->TWIM_RHR;
			--dev->RxRemain;
			++dev->RxCount;
		}

		if (!Sam4lI2CMasterWaitCommand(dev))
			goto done;

		if (dev->RxRemain <= 0)
			break;
		activeChunk = queuedChunk;
	}

done:
	dev->CombinedRead = false;
	dev->SrLen = 0;
	dev->BusHeld = pDev->bNoStop && !dev->LastError;
	return dev->RxCount;
}

static int Sam4lI2CMasterRxPolling(DevIntrf_t * const pDev,
	uint8_t *pBuff, int BuffLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->LastError)
		return 0;

	dev->pRxData = pBuff;
	dev->RxRemain = BuffLen;
	dev->RxCount = 0;

	if (dev->pI2cDev->Cfg.AddrType == I2CADDR_TYPE_EXT && !dev->BusHeld)
	{
		Sam4lI2CMasterStartRxPreamble(dev);
		if (!Sam4lI2CMasterWaitCommand(dev))
			return 0;
		dev->BusHeld = true;
		dev->NeedStart = true;
		dev->TenBitReadReady = true;
	}

	while (dev->RxRemain > 0)
	{
		if (!Sam4lI2CMasterStartRxChunk(dev))
			break;

		while (dev->ChunkRemain > 0)
		{
			if (!Sam4lI2CMasterWait(dev, TWIM_SR_RXRDY))
				return dev->RxCount;
			*dev->pRxData++ = (uint8_t)dev->pMReg->TWIM_RHR;
			--dev->ChunkRemain;
			--dev->RxRemain;
			++dev->RxCount;
		}

		if (!Sam4lI2CMasterWaitCommand(dev))
			return dev->RxCount;
	}

	dev->BusHeld = pDev->bNoStop && !dev->LastError;
	return dev->RxCount;
}

static int Sam4lI2CMasterRxDma(DevIntrf_t * const pDev,
	uint8_t *pBuff, int BuffLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->LastError)
		return 0;

	dev->pRxData = pBuff;
	dev->RxRemain = BuffLen;
	dev->RxCount = 0;

	if (pDev->bIntEn)
	{
		if (dev->pI2cDev->Cfg.AddrType == I2CADDR_TYPE_EXT && !dev->BusHeld)
		{
			dev->Op = SAM4L_I2C_OP_MASTER_RX_PREAMBLE;
			Sam4lI2CMasterStartRxPreamble(dev);
		}
		else
		{
			dev->Op = SAM4L_I2C_OP_MASTER_RX;
			Sam4lI2CMasterStartRxDmaChunk(dev);
		}
		dev->pMReg->TWIM_IER = SAM4L_TWIM_DMA_IRQ_MASK;
		return -1;
	}

	if (dev->pI2cDev->Cfg.AddrType == I2CADDR_TYPE_EXT && !dev->BusHeld)
	{
		Sam4lI2CMasterStartRxPreamble(dev);
		if (!Sam4lI2CMasterWaitCommand(dev))
			return 0;
		dev->BusHeld = true;
		dev->NeedStart = true;
		dev->TenBitReadReady = true;
	}

	while (dev->RxRemain > 0)
	{
		if (!Sam4lI2CMasterStartRxDmaChunk(dev))
			break;

		PdcaChannel *chan = Sam4lI2CPdcaRxChannel(dev);
		if (!Sam4lI2CPdcaWait(dev, chan))
			return dev->RxCount;
		chan->PDCA_CR = PDCA_CR_TDIS;

		if (!Sam4lI2CMasterWaitCommand(dev))
			return dev->RxCount;

		dev->pRxData += dev->ChunkTotal;
		dev->RxRemain -= dev->ChunkTotal;
		dev->RxCount += dev->ChunkTotal;
	}

	dev->BusHeld = pDev->bNoStop && !dev->LastError;
	return dev->RxCount;
}
static int Sam4lI2CMasterRxAsync(DevIntrf_t * const pDev,
	uint8_t *pBuff, int BuffLen)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->LastError)
		return 0;

	dev->pRxData = pBuff;
	dev->RxRemain = BuffLen;
	dev->RxCount = 0;

	if (dev->pI2cDev->Cfg.AddrType == I2CADDR_TYPE_EXT && !dev->BusHeld)
	{
		dev->Op = SAM4L_I2C_OP_MASTER_RX_PREAMBLE;
		Sam4lI2CMasterStartRxPreamble(dev);
	}
	else
	{
		dev->Op = SAM4L_I2C_OP_MASTER_RX;
		Sam4lI2CMasterStartRxChunk(dev);
	}

	dev->pMReg->TWIM_IER = SAM4L_TWIM_RX_IRQ_MASK;
	return -1;
}

static int Sam4lI2CRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == nullptr || BuffLen <= 0)
		return 0;

	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER)
		return 0;

	if (dev->CombinedRead)
		return pDev->bDma ?
			Sam4lI2CMasterReadCombinedDma(pDev, pBuff, BuffLen) :
			Sam4lI2CMasterReadCombinedPolling(pDev, pBuff, BuffLen);

	if (pDev->bDma)
		return Sam4lI2CMasterRxDma(pDev, pBuff, BuffLen);

	return pDev->bIntEn ?
		Sam4lI2CMasterRxAsync(pDev, pBuff, BuffLen) :
		Sam4lI2CMasterRxPolling(pDev, pBuff, BuffLen);
}

static void Sam4lI2CStopTx(DevIntrf_t * const pDev)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER)
		return;

	if (dev->BusHeld && !pDev->bNoStop)
	{
		dev->pMReg->TWIM_CR = TWIM_CR_STOP;
		(void)Sam4lI2CMasterWait(dev, TWIM_SR_IDLE);
		dev->BusHeld = false;
	}
}

static void Sam4lI2CStopRx(DevIntrf_t * const pDev)
{
	Sam4lI2CStopTx(pDev);
}

static void Sam4lI2CMasterAsyncComplete(SAM4L_I2CDEV *dev)
{
	DevIntrf_t *intrf = &dev->pI2cDev->DevIntrf;
	dev->pMReg->TWIM_IDR = 0xFFFFFFFFU;

	if (dev->Op == SAM4L_I2C_OP_MASTER_TX)
	{
		const int count = dev->TxCount;
		dev->BusHeld = intrf->bNoStop && !dev->LastError;
		dev->Op = SAM4L_I2C_OP_NONE;

		if (intrf->bNoStop)
		{
			intrf->bTxReady = true;
			return;
		}

		DeviceIntrfTxComplete(intrf);
		if (intrf->EvtCB)
			intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, nullptr, count);
	}
	else
	{
		const int count = dev->RxCount;
		dev->BusHeld = intrf->bNoStop && !dev->LastError;
		dev->Op = SAM4L_I2C_OP_NONE;
		DeviceIntrfStopRx(intrf);
		if (intrf->EvtCB)
			intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, nullptr, count);
	}
}

static void Sam4lI2CMasterIrqHandler(SAM4L_I2CDEV *dev)
{
	Twim *reg = dev->pMReg;
	const uint32_t sr = reg->TWIM_SR;
	const uint32_t pending = sr & reg->TWIM_IMR;

	if ((sr & SAM4L_TWIM_ERROR_MASK) != 0U)
	{
		const SAM4L_I2C_OP op = dev->Op;
		Sam4lI2CMasterRecover(dev);
		dev->Op = op;
		if (op == SAM4L_I2C_OP_MASTER_TX && dev->pI2cDev->DevIntrf.bNoStop)
		{
			dev->pI2cDev->DevIntrf.bTxReady = true;
			dev->Op = SAM4L_I2C_OP_NONE;
			return;
		}
		Sam4lI2CMasterAsyncComplete(dev);
		return;
	}

	if ((pending & TWIM_SR_TXRDY) != 0U &&
		dev->Op == SAM4L_I2C_OP_MASTER_TX && dev->ChunkRemain > 0)
	{
		reg->TWIM_THR = *dev->pTxData++;
		--dev->ChunkRemain;
		--dev->TxRemain;
		if (dev->ChunkRemain == 0)
			reg->TWIM_IDR = TWIM_IDR_TXRDY;
	}

	if ((pending & TWIM_SR_RXRDY) != 0U &&
		dev->Op == SAM4L_I2C_OP_MASTER_RX && dev->ChunkRemain > 0)
	{
		*dev->pRxData++ = (uint8_t)reg->TWIM_RHR;
		--dev->ChunkRemain;
		--dev->RxRemain;
		++dev->RxCount;
	}

	if ((pending & TWIM_SR_CCOMP) == 0U)
		return;

	reg->TWIM_SCR = TWIM_SCR_CCOMP;

	if (dev->Op == SAM4L_I2C_OP_MASTER_RX_PREAMBLE)
	{
		dev->BusHeld = true;
		dev->NeedStart = true;
		dev->TenBitReadReady = true;
		dev->Op = SAM4L_I2C_OP_MASTER_RX;
		if (dev->pI2cDev->DevIntrf.bDma)
			Sam4lI2CMasterStartRxDmaChunk(dev);
		else
			Sam4lI2CMasterStartRxChunk(dev);
		return;
	}

	if (dev->Op == SAM4L_I2C_OP_MASTER_TX)
	{
		if (dev->pI2cDev->DevIntrf.bDma)
		{
			PdcaChannel *chan = Sam4lI2CPdcaTxChannel(dev);
			if (chan->PDCA_TCR != 0U)
			{
				Sam4lI2CMasterRecover(dev);
				Sam4lI2CMasterAsyncComplete(dev);
				return;
			}
			chan->PDCA_CR = PDCA_CR_TDIS;
			dev->pTxData += dev->ChunkTotal;
			dev->TxRemain -= dev->ChunkTotal;
		}

		dev->TxCount += dev->ChunkTotal;
		if (dev->TxRemain > 0)
		{
			if (dev->pI2cDev->DevIntrf.bDma)
				Sam4lI2CMasterStartTxDmaChunk(dev);
			else
			{
				Sam4lI2CMasterStartTxChunk(dev);
				reg->TWIM_IER = TWIM_IER_TXRDY;
			}
			return;
		}
		Sam4lI2CMasterAsyncComplete(dev);
		return;
	}

	if (dev->Op == SAM4L_I2C_OP_MASTER_RX)
	{
		if (dev->pI2cDev->DevIntrf.bDma)
		{
			PdcaChannel *chan = Sam4lI2CPdcaRxChannel(dev);
			if (chan->PDCA_TCR != 0U)
			{
				Sam4lI2CMasterRecover(dev);
				Sam4lI2CMasterAsyncComplete(dev);
				return;
			}
			chan->PDCA_CR = PDCA_CR_TDIS;
			dev->pRxData += dev->ChunkTotal;
			dev->RxRemain -= dev->ChunkTotal;
			dev->RxCount += dev->ChunkTotal;
		}

		if (dev->RxRemain > 0)
		{
			if (dev->pI2cDev->DevIntrf.bDma)
				Sam4lI2CMasterStartRxDmaChunk(dev);
			else
				Sam4lI2CMasterStartRxChunk(dev);
			return;
		}
		Sam4lI2CMasterAsyncComplete(dev);
	}
}

static int Sam4lI2CSlaveRxDmaCount(SAM4L_I2CDEV *dev)
{
	I2CDev_t *i2c = dev->pI2cDev;
	if (i2c->pTRBuff[0] == nullptr || i2c->TRBuffLen[0] <= 0)
		return dev->SlaveRxCount;

	int dmaLen = i2c->TRBuffLen[0] - 1;
	if (dmaLen < 0)
		dmaLen = 0;
	uint32_t remain = Sam4lI2CSlavePdcaRxChannel(dev)->PDCA_TCR;
	if (remain > (uint32_t)dmaLen)
		remain = (uint32_t)dmaLen;
	const int dmaCount = dmaLen - (int)remain;
	return dev->SlaveRxCount > dmaCount ? dev->SlaveRxCount : dmaCount;
}

static int Sam4lI2CSlaveTxDmaCount(SAM4L_I2CDEV *dev)
{
	I2CDev_t *i2c = dev->pI2cDev;
	if (i2c->pRRData[0] == nullptr || i2c->RRDataLen[0] <= 0)
		return dev->SlaveTxCount;

	const int dmaLen = i2c->RRDataLen[0];
	uint32_t remain = Sam4lI2CSlavePdcaTxChannel(dev)->PDCA_TCR;
	if (remain > (uint32_t)dmaLen)
		remain = (uint32_t)dmaLen;
	const int dmaCount = dmaLen - (int)remain;
	return dev->SlaveTxCount > dmaCount ? dev->SlaveTxCount : dmaCount;
}

static void Sam4lI2CSlaveRxPdcaIrqHandler(SAM4L_I2CDEV *dev)
{
	PdcaChannel *chan = Sam4lI2CSlavePdcaRxChannel(dev);
	const uint32_t isr = chan->PDCA_ISR;

	if ((isr & (PDCA_ISR_TRC | PDCA_ISR_TERR)) == 0U)
		return;

	dev->SlaveRxCount = Sam4lI2CSlaveRxDmaCount(dev);
	chan->PDCA_CR = PDCA_CR_TDIS;
	chan->PDCA_IDR = PDCA_IER_TRC | PDCA_IER_TERR;

	if ((isr & PDCA_ISR_TERR) != 0U)
		chan->PDCA_CR = PDCA_CR_ECLR;

	// The datasheet specifies receive DMA size-1. Once that region is full,
	// leave the last byte to the TWIS ISR; STREN prevents RHR overrun while
	// the interrupt takes over.
	if (dev->SlaveRxActive)
		dev->pSReg->TWIS_IER = TWIS_IER_RXRDY;
}

static void Sam4lI2CSlavePrimeTx(SAM4L_I2CDEV *dev)
{
	if (dev->pI2cDev->DevIntrf.bDma)
		return;

	if (!dev->SlaveTxActive || dev->pSReg == nullptr)
		return;

	I2CDev_t *i2c = dev->pI2cDev;
	if (dev->SlaveTxCount >= i2c->RRDataLen[0] || i2c->pRRData[0] == nullptr)
	{
		// Leave THR empty. With STREN set, TWIS stretches SCL until the
		// application supplies data through I2CSetReadRqstData().
		return;
	}

	// The first byte may need to be loaded while SOAM is still holding the
	// address phase. TXRDY is not guaranteed to be asserted yet at that point.
	// After the first byte, only refill an empty THR.
	if (dev->SlaveTxCount != 0 &&
		(dev->pSReg->TWIS_SR & TWIS_SR_TXRDY) == 0U)
		return;

	dev->pSReg->TWIS_THR = i2c->pRRData[0][dev->SlaveTxCount++];
}

static void Sam4lI2CSlaveFinish(SAM4L_I2CDEV *dev)
{
	I2CDev_t *i2c = dev->pI2cDev;
	int count;

	if (i2c->DevIntrf.bDma)
	{
		if (dev->SlaveTxActive)
		{
			count = Sam4lI2CSlaveTxDmaCount(dev);
			Sam4lI2CSlavePdcaTxChannel(dev)->PDCA_CR = PDCA_CR_TDIS;
		}
		else
		{
			count = Sam4lI2CSlaveRxDmaCount(dev);
			Sam4lI2CSlavePdcaRxChannel(dev)->PDCA_CR = PDCA_CR_TDIS;
		}
	}
	else
	{
		count = dev->SlaveTxActive ? dev->SlaveTxCount : dev->SlaveRxCount;
	}

	dev->SlaveRxActive = false;
	dev->SlaveTxActive = false;
	dev->SlaveRxCount = 0;
	dev->SlaveTxCount = 0;
	dev->pSReg->TWIS_IDR = TWIS_IER_RXRDY | TWIS_IER_BTF;
	dev->pSReg->TWIS_CR &= ~TWIS_CR_ACK;

	if (i2c->DevIntrf.EvtCB)
		i2c->DevIntrf.EvtCB(&i2c->DevIntrf, DEVINTRF_EVT_COMPLETED, nullptr, count);
}
static void Sam4lI2CSlaveIrqHandler(SAM4L_I2CDEV *dev)
{
	Twis *reg = dev->pSReg;
	I2CDev_t *i2c = dev->pI2cDev;
	const uint32_t sr = reg->TWIS_SR;
	const uint32_t pending = sr & reg->TWIS_IMR;

	if ((pending & SAM4L_TWIS_ERROR_MASK) != 0U)
	{
		reg->TWIS_SCR = pending & SAM4L_TWIS_ERROR_MASK;
		Sam4lI2CSlaveFinish(dev);
		return;
	}

	if ((pending & TWIS_SR_SAM) != 0U)
	{
		// TRA is specified to update one CLK_TWIS cycle after SAM. Do not use
		// the SR snapshot taken at ISR entry.
		const bool transmit = (reg->TWIS_SR & TWIS_SR_TRA) != 0U;
		reg->TWIS_CR &= ~TWIS_CR_ACK;
		reg->TWIS_NBYTES = 0U;

		if (transmit)
		{
			int priorRx = 0;
			if (dev->SlaveRxActive)
			{
				priorRx = i2c->DevIntrf.bDma ?
					Sam4lI2CSlaveRxDmaCount(dev) : dev->SlaveRxCount;
				if (i2c->DevIntrf.bDma)
					Sam4lI2CSlavePdcaRxChannel(dev)->PDCA_CR = PDCA_CR_TDIS;
			}

			dev->SlaveRxActive = false;
			dev->SlaveTxActive = true;
			dev->SlaveTxCount = 0;
			reg->TWIS_IDR = TWIS_IER_RXRDY | TWIS_IER_BTF;

			// A response buffer belongs to this read request. Do not silently
			// reuse one installed for an earlier transaction.
			i2c->pRRData[0] = nullptr;
			i2c->RRDataLen[0] = 0;

			if (i2c->DevIntrf.EvtCB)
				i2c->DevIntrf.EvtCB(&i2c->DevIntrf,
					DEVINTRF_EVT_READ_RQST, nullptr, priorRx);

			if (!i2c->DevIntrf.bDma)
			{
				reg->TWIS_IER = TWIS_IER_BTF;
				Sam4lI2CSlavePrimeTx(dev);
			}
		}
		else
		{
			if (dev->SlaveTxActive && i2c->DevIntrf.EvtCB)
			{
				const int priorTx = i2c->DevIntrf.bDma ?
					Sam4lI2CSlaveTxDmaCount(dev) : dev->SlaveTxCount;
				i2c->DevIntrf.EvtCB(&i2c->DevIntrf,
					DEVINTRF_EVT_COMPLETED, nullptr, priorTx);
			}
			if (i2c->DevIntrf.bDma)
				Sam4lI2CSlavePdcaTxChannel(dev)->PDCA_CR = PDCA_CR_TDIS;

			dev->SlaveTxActive = false;
			dev->SlaveRxActive = true;
			dev->SlaveRxCount = 0;
			reg->TWIS_IDR = TWIS_IER_RXRDY | TWIS_IER_BTF;

			// Receive storage is request-scoped for the same reason.
			i2c->pTRBuff[0] = nullptr;
			i2c->TRBuffLen[0] = 0;

			if (i2c->DevIntrf.EvtCB)
				i2c->DevIntrf.EvtCB(&i2c->DevIntrf,
					DEVINTRF_EVT_WRITE_RQST, nullptr, 0);

			if (!i2c->DevIntrf.bDma)
				reg->TWIS_IER = TWIS_IER_RXRDY;
		}

		// SOAM keeps SCL low until SAM is cleared. Request callbacks above can
		// arm PDCA or install the interrupt-mode buffer before bus progress.
		reg->TWIS_SCR = TWIS_SCR_SAM;
	}

	if ((pending & TWIS_SR_RXRDY) != 0U && dev->SlaveRxActive)
	{
		if (i2c->pTRBuff[0] == nullptr)
		{
			// Leave RHR full. STREN holds SCL low until the application installs
			// the receive buffer through I2CSetWriteRqstBuffer().
			reg->TWIS_IDR = TWIS_IER_RXRDY;
		}
		else
		{
			if (dev->SlaveRxCount >= i2c->TRBuffLen[0])
				reg->TWIS_CR |= TWIS_CR_ACK;
			const uint8_t value = (uint8_t)reg->TWIS_RHR;
			if (dev->SlaveRxCount < i2c->TRBuffLen[0])
				i2c->pTRBuff[0][dev->SlaveRxCount++] = value;
		}
	}

	if ((pending & TWIS_SR_BTF) != 0U && dev->SlaveTxActive)
	{
		if ((sr & TWIS_SR_NAK) != 0U)
		{
			reg->TWIS_SCR = TWIS_SCR_BTF | TWIS_SCR_NAK;
			reg->TWIS_IDR = TWIS_IER_BTF;
		}
		else
		{
			reg->TWIS_SCR = TWIS_SCR_BTF;
			Sam4lI2CSlavePrimeTx(dev);
		}
	}

	if ((pending & TWIS_SR_REP) != 0U)
		reg->TWIS_SCR = TWIS_SCR_REP;

	if ((pending & TWIS_SR_TCOMP) != 0U)
	{
		reg->TWIS_SCR = TWIS_SCR_TCOMP | TWIS_SCR_STO | TWIS_SCR_NAK | TWIS_SCR_BTF;
		Sam4lI2CSlaveFinish(dev);
	}
}

static void Sam4lI2CDisable(DevIntrf_t * const pDev)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		dev->pSReg->TWIS_IDR = 0xFFFFFFFFU;
		if (pDev->bDma)
			Sam4lI2CSlavePdcaStop(dev);
		dev->pSReg->TWIS_CR &= ~TWIS_CR_SEN;
	}
	else
	{
		dev->pMReg->TWIM_IDR = 0xFFFFFFFFU;
		if (pDev->bDma)
			Sam4lI2CPdcaStop(dev);
		dev->pMReg->TWIM_CR = TWIM_CR_MDIS;
	}
	Sam4lI2CClockDisable(dev);
}

static void Sam4lI2CEnable(DevIntrf_t * const pDev)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	Sam4lI2CClockEnable(dev);
	IOPinCfg(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		dev->pSReg->TWIS_IER = SAM4L_TWIS_BASE_IRQ_MASK;
		dev->pSReg->TWIS_CR |= TWIS_CR_SEN;
	}
	else
	{
		dev->pMReg->TWIM_CR = TWIM_CR_MEN;
	}
}

static void Sam4lI2CReset(DevIntrf_t * const pDev)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;

	I2CBusReset(dev->pI2cDev);
	if (dev->pI2cDev->Cfg.Mode == I2CMODE_MASTER)
	{
		Sam4lI2CMasterHwReset(dev);
	}
	else
	{
		if (pDev->Cfg.bDmaEn)
			Sam4lI2CSlavePdcaInit(dev);

		Twis *reg = dev->pSReg;
		reg->TWIS_IDR = 0xFFFFFFFFU;
		if (pDev->bDma)
			Sam4lI2CSlavePdcaStop(dev);
		reg->TWIS_CR = TWIS_CR_SWRST;
		reg->TWIS_SCR = 0xFFFFFFFFU;
		reg->TWIS_NBYTES = 0U;
		uint32_t cr = TWIS_CR_SMATCH | TWIS_CR_STREN | TWIS_CR_SOAM |
			TWIS_CR_ADR(dev->pI2cDev->Cfg.SlaveAddr[0]);
		if (pDev->bDma)
			cr |= TWIS_CR_CUP;
		if (dev->pI2cDev->Cfg.AddrType == I2CADDR_TYPE_EXT)
			cr |= TWIS_CR_TENBIT;
		reg->TWIS_CR = cr | TWIS_CR_SEN;
		(void)Sam4lI2CSetRate(pDev, dev->pI2cDev->Cfg.Rate);
		reg->TWIS_IER = SAM4L_TWIS_BASE_IRQ_MASK;
	}
}

static void Sam4lI2CPowerOff(DevIntrf_t * const pDev)
{
	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->pDevData;
	Sam4lI2CDisable(pDev);
	IOPinDisable(dev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PortNo,
		dev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PinNo);
	IOPinDisable(dev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PortNo,
		dev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PinNo);
}

static void *Sam4lI2CGetHandle(DevIntrf_t * const pDev)
{
	return ((SAM4L_I2CDEV *)pDev->pDevData)->pI2cDev;
}

void I2CSetReadRqstData(I2CDev_t * const pDev, int SlaveIdx,
	uint8_t * const pData, int DataLen)
{
	if (pDev == nullptr || SlaveIdx != 0 || DataLen < 0)
		return;

	if (DataLen > (int)SAM4L_PDCA_MAX_COUNT)
		DataLen = (int)SAM4L_PDCA_MAX_COUNT;

	pDev->pRRData[0] = pData;
	pDev->RRDataLen[0] = DataLen;

	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->DevIntrf.pDevData;
	if (dev == nullptr || pDev->Cfg.Mode != I2CMODE_SLAVE ||
		!dev->SlaveTxActive || pData == nullptr || DataLen <= 0)
		return;

	if (pDev->DevIntrf.bDma)
	{
		PdcaChannel *chan = Sam4lI2CSlavePdcaTxChannel(dev);
		chan->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
		chan->PDCA_PSR = PDCA_PSR_PID(SAM4L_PDCA_PID_TWIS_TX(dev->DevNo));
		chan->PDCA_MAR = (uint32_t)pData;
		chan->PDCA_TCR = (uint32_t)DataLen;
		chan->PDCA_IDR = 0xFFFFFFFFU;
		chan->PDCA_CR = PDCA_CR_TEN;
	}
	else
	{
		Sam4lI2CSlavePrimeTx(dev);
	}
}

void I2CSetWriteRqstBuffer(I2CDev_t * const pDev, int SlaveIdx,
	uint8_t * const pBuff, int BuffLen)
{
	if (pDev == nullptr || SlaveIdx != 0 || BuffLen < 0)
		return;

	if (BuffLen > (int)SAM4L_PDCA_MAX_COUNT)
		BuffLen = (int)SAM4L_PDCA_MAX_COUNT;

	pDev->pTRBuff[0] = pBuff;
	pDev->TRBuffLen[0] = BuffLen;

	SAM4L_I2CDEV *dev = (SAM4L_I2CDEV *)pDev->DevIntrf.pDevData;
	if (dev == nullptr || pDev->Cfg.Mode != I2CMODE_SLAVE ||
		!dev->SlaveRxActive || pBuff == nullptr || BuffLen <= 0)
		return;

	Twis *reg = dev->pSReg;
	if (pDev->DevIntrf.bDma)
	{
		PdcaChannel *chan = Sam4lI2CSlavePdcaRxChannel(dev);
		const uint32_t dmaLen = BuffLen > 1 ? (uint32_t)(BuffLen - 1) : 0U;

		chan->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
		chan->PDCA_PSR = PDCA_PSR_PID(SAM4L_PDCA_PID_TWIS_RX(dev->DevNo));
		chan->PDCA_MAR = (uint32_t)pBuff;
		chan->PDCA_TCR = dmaLen;
		chan->PDCA_IDR = 0xFFFFFFFFU;
		dev->SlaveRxCount = 0;

		reg->TWIS_IDR = TWIS_IER_RXRDY;
		if (dmaLen > 0U)
		{
			chan->PDCA_IER = PDCA_IER_TRC | PDCA_IER_TERR;
			chan->PDCA_CR = PDCA_CR_TEN;
		}
		else
		{
			reg->TWIS_IER = TWIS_IER_RXRDY;
		}
		return;
	}

	if ((reg->TWIS_SR & TWIS_SR_RXRDY) != 0U &&
		dev->SlaveRxCount < BuffLen)
	{
		pBuff[dev->SlaveRxCount++] = (uint8_t)reg->TWIS_RHR;
	}
	reg->TWIS_IER = TWIS_IER_RXRDY;
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	if (pDev == nullptr || pCfgData == nullptr ||
		pCfgData->pIOPinMap == nullptr || pCfgData->NbIOPins < 2 ||
		pCfgData->DevNo < 0 || pCfgData->DevNo >= SAM4L_I2C_MASTER_COUNT ||
		pCfgData->Type != I2CTYPE_STANDARD ||
		pCfgData->Rate == 0U)
	{
		return false;
	}

	if (pCfgData->Mode == I2CMODE_SLAVE)
	{
		// The generic slave-address array is byte-sized, so the public API
		// cannot represent a SAM4L 10-bit own address without changing the
		// shared interface. Keep the target correct and reject that mode.
		if (pCfgData->DevNo >= SAM4L_I2C_SLAVE_COUNT ||
			pCfgData->NbSlaveAddr != 1 ||
			pCfgData->AddrType != I2CADDR_TYPE_NORMAL ||
			pCfgData->SlaveAddr[0] > 0x7FU)
			return false;
	}

	SAM4L_I2CDEV *dev = &s_Sam4lI2CDev[pCfgData->DevNo];
	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));
	if (pDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		// Slave timing is controlled by the external master; TWIS therefore
		// always uses its interrupt path regardless of the requested setting.
		pDev->Cfg.bIntEn = true;
	}

	memset(pDev->pRRData, 0, sizeof(pDev->pRRData));
	memset(pDev->RRDataLen, 0, sizeof(pDev->RRDataLen));
	memset(pDev->pTRBuff, 0, sizeof(pDev->pTRBuff));
	memset(pDev->TRBuffLen, 0, sizeof(pDev->TRBuffLen));

	dev->pI2cDev = pDev;
	dev->Op = SAM4L_I2C_OP_NONE;
	dev->NeedStart = true;
	dev->BusHeld = false;
	dev->LastError = false;
	dev->CommandPhase = false;
	dev->TenBitReadReady = false;
	dev->SlaveRxCount = 0;
	dev->SlaveTxCount = 0;
	dev->SlaveRxActive = false;
	dev->SlaveTxActive = false;

	pDev->DevIntrf.pDevData = dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.bDma = pDev->Cfg.bDmaEn;
	pDev->DevIntrf.bIntEn = pDev->Cfg.bIntEn;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.Disable = Sam4lI2CDisable;
	pDev->DevIntrf.Enable = Sam4lI2CEnable;
	pDev->DevIntrf.GetRate = Sam4lI2CGetRate;
	pDev->DevIntrf.SetRate = Sam4lI2CSetRate;
	pDev->DevIntrf.StartRx = Sam4lI2CStartRx;
	pDev->DevIntrf.RxData = Sam4lI2CRxData;
	pDev->DevIntrf.StopRx = Sam4lI2CStopRx;
	pDev->DevIntrf.StartTx = Sam4lI2CStartTx;
	pDev->DevIntrf.TxData = Sam4lI2CTxData;
	pDev->DevIntrf.TxSrData = Sam4lI2CTxSrData;
	pDev->DevIntrf.StopTx = Sam4lI2CStopTx;
	pDev->DevIntrf.Reset = Sam4lI2CReset;
	pDev->DevIntrf.PowerOff = Sam4lI2CPowerOff;
	pDev->DevIntrf.GetHandle = Sam4lI2CGetHandle;
	pDev->DevIntrf.IntPrio = pDev->Cfg.IntPrio;
	pDev->DevIntrf.EvtCB = pDev->Cfg.EvtCB;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	Sam4lI2CClockEnable(dev);
	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);

	if (pDev->Cfg.Mode == I2CMODE_MASTER)
	{
		if (pDev->Cfg.bDmaEn)
			Sam4lI2CPdcaInit(dev);
		Sam4lI2CMasterHwReset(dev);
		if (pDev->Cfg.bIntEn)
		{
			NVIC_ClearPendingIRQ(dev->MasterIrq);
			NVIC_SetPriority(dev->MasterIrq, pDev->Cfg.IntPrio);
			NVIC_EnableIRQ(dev->MasterIrq);
		}
	}
	else
	{
		Twis *reg = dev->pSReg;
		reg->TWIS_IDR = 0xFFFFFFFFU;
		reg->TWIS_CR = TWIS_CR_SWRST;
		reg->TWIS_SCR = 0xFFFFFFFFU;
		reg->TWIS_NBYTES = 0U;

		uint32_t cr = TWIS_CR_SMATCH | TWIS_CR_STREN | TWIS_CR_SOAM |
			TWIS_CR_ADR(pDev->Cfg.SlaveAddr[0]);
		if (pDev->Cfg.bDmaEn)
			cr |= TWIS_CR_CUP;
		if (pDev->Cfg.AddrType == I2CADDR_TYPE_EXT)
			cr |= TWIS_CR_TENBIT;
		reg->TWIS_CR = cr;

		if (Sam4lI2CSetRate(&pDev->DevIntrf, pDev->Cfg.Rate) == 0U)
			return false;

		reg->TWIS_IER = SAM4L_TWIS_BASE_IRQ_MASK;
		reg->TWIS_CR |= TWIS_CR_SEN;

		NVIC_ClearPendingIRQ(dev->SlaveIrq);
		NVIC_SetPriority(dev->SlaveIrq, pDev->Cfg.IntPrio);
		NVIC_EnableIRQ(dev->SlaveIrq);
	}

	return pDev->Cfg.Rate != 0U;
}

extern "C" void TWIM0_Handler(void)
{
	Sam4lI2CMasterIrqHandler(&s_Sam4lI2CDev[0]);
}

extern "C" void TWIM1_Handler(void)
{
	Sam4lI2CMasterIrqHandler(&s_Sam4lI2CDev[1]);
}

extern "C" void TWIM2_Handler(void)
{
	Sam4lI2CMasterIrqHandler(&s_Sam4lI2CDev[2]);
}

extern "C" void TWIM3_Handler(void)
{
	Sam4lI2CMasterIrqHandler(&s_Sam4lI2CDev[3]);
}

extern "C" void TWIS0_Handler(void)
{
	Sam4lI2CSlaveIrqHandler(&s_Sam4lI2CDev[0]);
}

extern "C" void TWIS1_Handler(void)
{
	Sam4lI2CSlaveIrqHandler(&s_Sam4lI2CDev[1]);
}

extern "C" void PDCA_12_Handler(void)
{
	Sam4lI2CSlaveRxPdcaIrqHandler(&s_Sam4lI2CDev[0]);
}

extern "C" void PDCA_14_Handler(void)
{
	Sam4lI2CSlaveRxPdcaIrqHandler(&s_Sam4lI2CDev[1]);
}
