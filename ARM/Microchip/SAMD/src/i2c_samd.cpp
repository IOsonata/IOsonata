/**-------------------------------------------------------------------------
@file	i2c_samd.cpp

@brief	SAM D11/D21 SERCOM I2C master and slave

		DevNo is the SERCOM number. SDA is pad 0, SCL pad 1, on pins with the
		I2C pad drive (see SamdSercomI2cPin). 7-bit addresses, standard I2C.

		Master: polling or interrupt. The SCL rate is set from the main clock
		with an assumed 215 ns rise time, from 1 MHz down to about 90 kHz at
		48 MHz; a request out of that range gets the closest rate. In
		interrupt mode, TxData and RxData return -1 and report the transfer
		with DEVINTRF_EVT_COMPLETED. The command phase of a read (bNoStop)
		is sent by polling. Data to transmit is copied to a buffer shared by
		the instances; data longer than the buffer, or sent while another
		instance uses it, is sent by polling and reported the same way.
		A START waits for the bus to be idle, up to about 15 ms: past it, or
		after a polling transfer timeout, the SERCOM is restarted and the bus
		state set to idle. A slave holding SDA low is not clocked free.

		Master with bDmaEn: one DMAC channel, on the SERCOM TX (MB) or RX (SB)
		trigger, moves the data bytes in place of the CPU, taken at I2CInit
		(I2CInit fails without a free channel), given back at PowerOff:
		transfers return 0 until I2CInit is called again.
		The CPU sends the address and checks its acknowledge, the channel
		the bytes of a transmit, then the CPU waits for the end of the last
		one; the channel reads all the bytes of a receive but the last one,
		each read acknowledging the byte and starting the next (smart mode),
		the CPU refuses the last byte and sends the STOP. ADDR.LENEN is not
		used: the transfer has any length, the STOP or the repeated START is
		set by the CPU as without DMA. Same modes as without DMA: with bIntEn
		the DMAC interrupt ends the block and the SERCOM interrupt the
		transfer, the cases sent by polling above are sent by DMA with the
		CPU waiting, as all transfers without bIntEn. A transmit byte refused
		by the slave does not stop the channel: the count is the length, less
		one when the slave refused the last byte. A receive of one byte is
		done by the CPU.

		Slave: interrupt, one address. DEVINTRF_EVT_WRITE_RQST and
		DEVINTRF_EVT_READ_RQST at the address match, the handler sets the
		buffer with I2CSetWriteRqstBuffer or I2CSetReadRqstData. A read
		request after a write in the same transfer gets the count of bytes
		written. DEVINTRF_EVT_COMPLETED at the STOP, or at a repeated START
		that turns a read into a write, with the count of bytes moved. Bytes
		past the write buffer are refused, a read past the data gets 0xFF.
		No DMA: I2CInit fails with bDmaEn.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "samdxxx.h"

#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/i2c.h"
#include "coredev/shared_intrf.h"
#include "clock_samd.h"
#include "sercom_samd.h"
#include "dmac_samd.h"

// Bound of the register and bus waits, about 15 ms at 48 MHz
#define SAMD_I2C_WAIT_LIMIT			100000UL

#define SAMD_I2C_RATE_MAX			1000000U	// Fast mode plus
#define SAMD_I2C_TRISE_NS			215U		// SCL rise time of the rate setting

// Interrupt transmit buffer, shared by the instances
#if SAMD_SERCOM_CNT > 3
#define SAMD_I2C_TXBUF_SIZE			256
#else
#define SAMD_I2C_TXBUF_SIZE			64
#endif

// Master CTRLB.CMD
#define SAMD_I2CM_CMD_STOP			3U		// Acknowledge action then STOP

// Slave CTRLB.CMD
#define SAMD_I2CS_CMD_WAIT			2U		// Transfer done, wait for START or STOP
#define SAMD_I2CS_CMD_NEXT			3U		// Acknowledge action then next byte

#define SAMD_I2CM_BUSSTATE_UNKNOWN	0U
#define SAMD_I2CM_BUSSTATE_IDLE		1U
#define SAMD_I2CM_BUSSTATE_OWNER	2U

#define SAMD_I2CM_ERRORS			(SERCOM_I2CM_STATUS_BUSERR_Msk | SERCOM_I2CM_STATUS_ARBLOST_Msk | \
									 SERCOM_I2CM_STATUS_LOWTOUT_Msk | SERCOM_I2CM_STATUS_MEXTTOUT_Msk | \
									 SERCOM_I2CM_STATUS_SEXTTOUT_Msk | SERCOM_I2CM_STATUS_LENERR_Msk)
#define SAMD_I2CM_BUS_ERRORS		(SERCOM_I2CM_STATUS_BUSERR_Msk | SERCOM_I2CM_STATUS_ARBLOST_Msk)
#define SAMD_I2CS_ERRORS			(SERCOM_I2CS_STATUS_BUSERR_Msk | SERCOM_I2CS_STATUS_COLL_Msk | \
									 SERCOM_I2CS_STATUS_LOWTOUT_Msk | SERCOM_I2CS_STATUS_SEXTTOUT_Msk)

#define SAMD_I2CM_INTS				(SERCOM_I2CM_INTENSET_MB_Msk | SERCOM_I2CM_INTENSET_SB_Msk | \
									 SERCOM_I2CM_INTENSET_ERROR_Msk)
#define SAMD_I2CS_INTS				(SERCOM_I2CS_INTENSET_PREC_Msk | SERCOM_I2CS_INTENSET_AMATCH_Msk | \
									 SERCOM_I2CS_INTENSET_DRDY_Msk | SERCOM_I2CS_INTENSET_ERROR_Msk)

typedef enum {
	SAMD_I2C_OP_NONE,
	SAMD_I2C_OP_TX,
	SAMD_I2C_OP_RX,
} SAMD_I2C_OP;

#pragma pack(push, 4)

typedef struct {
	int DevNo;						//!< SERCOM number
	sercom_registers_t *pReg;
	I2CDev_t *pI2cDev;				//!< Generic I2C device data
	uint32_t CtrlA;					//!< CTRLA without ENABLE
	uint32_t Baud;					//!< Master BAUD
	uint8_t DevAddr;				//!< Master target address
	bool bNeedStart;				//!< Master: the next byte goes after the address
	bool bBusHeld;					//!< Master owns the bus, no STOP sent
	volatile SAMD_I2C_OP Op;		//!< Master interrupt transfer
	bool bAddrPhase;				//!< Master interrupt transmit: address sent, not acknowledged
	const uint8_t *pTxData;
	uint8_t *pRxData;
	int Remain;
	int Count;
	bool bSlaveTx;					//!< Slave transmit, master read
	bool bSlaveRx;					//!< Slave receive, master write
	bool bSlaveFirst;				//!< First data interrupt of a slave transmit
	bool bDma;						//!< DmaChan taken
	int SlaveCount;
	volatile bool bDmaRun;			//!< Master DMA block ended by the DMAC interrupt
	int8_t DmaChan;					//!< DMAC channel of the master, both directions
	uint16_t DmaLen;				//!< Bytes of the running block
} SamdI2cDev_t;

#pragma pack(pop)

static SamdI2cDev_t s_SamdI2cDev[SAMD_SERCOM_CNT];

static uint8_t s_SamdI2cTxBuf[SAMD_I2C_TXBUF_SIZE];
static SamdI2cDev_t *s_pSamdI2cTxOwner;

static bool SamdI2cSync(sercom_registers_t *pReg, uint32_t Mask)
{
	for (uint32_t i = 0; i < SAMD_I2C_WAIT_LIMIT; i++)
	{
		if ((pReg->I2CM.SERCOM_SYNCBUSY & Mask) == 0U)
		{
			return true;
		}
	}

	return false;
}

static inline uint32_t SamdI2cmBusState(sercom_i2cm_registers_t *pReg)
{
	return ((uint32_t)pReg->SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos;
}

// Master CTRLB, smart mode: reading DATA sends the acknowledge action
static void SamdI2cmCtrlb(SamdI2cDev_t *pDev, uint32_t Cmd, bool bNack)
{
	pDev->pReg->I2CM.SERCOM_CTRLB = SERCOM_I2CM_CTRLB_SMEN_Msk | SERCOM_I2CM_CTRLB_CMD(Cmd) |
									(bNack ? SERCOM_I2CM_CTRLB_ACKACT_Msk : 0U);
	(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
}

static void SamdI2cmStop(SamdI2cDev_t *pDev)
{
	if (pDev->bBusHeld)
	{
		SamdI2cmCtrlb(pDev, SAMD_I2CM_CMD_STOP, true);
		pDev->bBusHeld = false;
	}
}

// Bus state idle, false on timeout
static bool SamdI2cmWaitIdle(sercom_i2cm_registers_t *pReg)
{
	for (uint32_t i = 0; i < SAMD_I2C_WAIT_LIMIT; i++)
	{
		if (SamdI2cmBusState(pReg) == SAMD_I2CM_BUSSTATE_IDLE)
		{
			return true;
		}
	}

	return false;
}

// After a bus error, an arbitration loss or a timeout. An unknown bus state
// is set to idle. Restart, from a polling transfer or before a START: a bus
// that does not go idle, busy from a transfer that did not end, gets the
// SERCOM disabled and enabled with its settings and the bus state set to
// idle. Not in the interrupt: after an arbitration loss the bus is busy with
// another master until its STOP.
static void SamdI2cmRecover(SamdI2cDev_t *pDev, bool bRestart)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;

	if (SamdI2cmBusState(reg) == SAMD_I2CM_BUSSTATE_OWNER)
	{
		SamdI2cmCtrlb(pDev, SAMD_I2CM_CMD_STOP, true);
	}
	reg->SERCOM_STATUS = SAMD_I2CM_ERRORS;
	reg->SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_ERROR_Msk;
	(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
	pDev->bBusHeld = false;

	uint32_t state = SamdI2cmBusState(reg);

	if (state != SAMD_I2CM_BUSSTATE_UNKNOWN &&
		(!bRestart || state == SAMD_I2CM_BUSSTATE_IDLE || SamdI2cmWaitIdle(reg)))
	{
		return;
	}

	if (state != SAMD_I2CM_BUSSTATE_UNKNOWN)
	{
		reg->SERCOM_CTRLA = pDev->CtrlA;
		(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);
		reg->SERCOM_CTRLA = pDev->CtrlA | SERCOM_I2CM_CTRLA_ENABLE_Msk;
		(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);
	}

	// The bus state is unknown after enable
	reg->SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE(SAMD_I2CM_BUSSTATE_IDLE);
	(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
}

// Before a START that is not a repeated START, from the caller of the
// transfer: the bus idle, the STOP of the previous transfer on the bus. An
// address written before the STOP is out makes a repeated START, one
// written on a busy bus waits without a bound.
static void SamdI2cmWaitFree(SamdI2cDev_t *pDev)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;

	if (pDev->bBusHeld)
	{
		return;
	}

	if (SamdI2cmBusState(reg) == SAMD_I2CM_BUSSTATE_UNKNOWN || !SamdI2cmWaitIdle(reg))
	{
		SamdI2cmRecover(pDev, true);
	}
}

// MB or SB, 0 on timeout
static uint8_t SamdI2cmWait(SamdI2cDev_t *pDev)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;

	for (uint32_t i = 0; i < SAMD_I2C_WAIT_LIMIT; i++)
	{
		uint8_t flags = reg->SERCOM_INTFLAG & (SERCOM_I2CM_INTFLAG_MB_Msk | SERCOM_I2CM_INTFLAG_SB_Msk);

		if (flags != 0U)
		{
			return flags;
		}
	}

	return 0;
}

static inline void SamdI2cmSendAddress(SamdI2cDev_t *pDev, bool bRead)
{
	pDev->pReg->I2CM.SERCOM_ADDR = SERCOM_I2CM_ADDR_ADDR(((uint32_t)pDev->DevAddr << 1) | (bRead ? 1U : 0U));
	(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
	pDev->bNeedStart = false;
}

// START, or repeated START on a held bus, and the address. A read has its
// first byte received when this returns true.
static bool SamdI2cmAddress(SamdI2cDev_t *pDev, bool bRead)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;

	SamdI2cmWaitFree(pDev);
	SamdI2cmCtrlb(pDev, 0, false);
	SamdI2cmSendAddress(pDev, bRead);

	uint8_t flags = SamdI2cmWait(pDev);
	uint16_t status = reg->SERCOM_STATUS;

	if (flags == 0U || (status & SAMD_I2CM_BUS_ERRORS) != 0U)
	{
		SamdI2cmRecover(pDev, flags == 0U);

		return false;
	}

	pDev->bBusHeld = true;

	// Address refused: MB with RXNACK instead of the first read byte
	if ((status & SERCOM_I2CM_STATUS_RXNACK_Msk) != 0U || (bRead && (flags & SERCOM_I2CM_INTFLAG_SB_Msk) == 0U))
	{
		SamdI2cmStop(pDev);

		return false;
	}

	return true;
}

// Bytes acknowledged by the slave
static int SamdI2cmTxPolling(SamdI2cDev_t *pDev, const uint8_t *pData, int DataLen)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;
	int count = 0;

	if (pDev->bNeedStart && !SamdI2cmAddress(pDev, false))
	{
		return 0;
	}

	while (count < DataLen)
	{
		reg->SERCOM_DATA = pData[count];
		(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);

		uint8_t flags = SamdI2cmWait(pDev);
		uint16_t status = reg->SERCOM_STATUS;

		if (flags == 0U || (status & SAMD_I2CM_BUS_ERRORS) != 0U)
		{
			SamdI2cmRecover(pDev, flags == 0U);
			break;
		}
		if ((status & SERCOM_I2CM_STATUS_RXNACK_Msk) != 0U)
		{
			SamdI2cmStop(pDev);
			break;
		}
		count++;
	}

	return count;
}

// The last byte is refused and followed by the STOP
static int SamdI2cmRxPolling(SamdI2cDev_t *pDev, uint8_t *pBuff, int BuffLen)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;
	int count = 0;

	if (!SamdI2cmAddress(pDev, true))
	{
		return 0;
	}

	while (count < BuffLen)
	{
		if (count > 0)
		{
			uint8_t flags = SamdI2cmWait(pDev);

			if ((flags & SERCOM_I2CM_INTFLAG_SB_Msk) == 0U || (reg->SERCOM_STATUS & SAMD_I2CM_BUS_ERRORS) != 0U)
			{
				SamdI2cmRecover(pDev, flags == 0U);
				break;
			}
		}

		if (count == BuffLen - 1)
		{
			SamdI2cmCtrlb(pDev, SAMD_I2CM_CMD_STOP, true);
			pDev->bBusHeld = false;
		}

		// Smart mode: acknowledges and receives the next byte
		(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
		pBuff[count++] = reg->SERCOM_DATA;
	}

	return count;
}

// Master DMA block on the TX trigger (MB) with pTx, the RX trigger (SB)
// otherwise. A request already set is taken at once.
static void SamdI2cmDmaStart(SamdI2cDev_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Len)
{
	volatile uint8_t *data = &pDev->pReg->I2CM.SERCOM_DATA;

	pDev->DmaLen = (uint16_t)Len;
	if (pTx != NULL)
	{
		SamdDmacTrigger(pDev->DmaChan, SAMD_DMAC_TRIG_SERCOM_TX(pDev->DevNo));
		SamdDmacStart(pDev->DmaChan, pTx, true, data, false, (uint32_t)Len);
	}
	else
	{
		SamdDmacTrigger(pDev->DmaChan, SAMD_DMAC_TRIG_SERCOM_RX(pDev->DevNo));
		SamdDmacStart(pDev->DmaChan, data, false, pRx, true, (uint32_t)Len);
	}
}

// Wait of a DMA block, the CPU polling, ended by a bus error. Bounded at
// SAMD_I2C_WAIT_LIMIT passes, as one byte by polling, plus the core clocks
// of the block at 9 SCL periods a byte: a pass takes several clocks, a slave
// holding SCL low ends the wait after about 15 ms plus a few times the
// transfer time. Returns the bytes the channel did not move.
static int SamdI2cmDmaWait(SamdI2cDev_t *pDev, int Len)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;
	uint32_t rate = pDev->pI2cDev->Cfg.Rate;
	uint64_t limit = SAMD_I2C_WAIT_LIMIT + (uint64_t)Len * 9U * SystemCoreClockGet() / (rate ? rate : 1U);

	for (uint64_t i = 0; i < limit && SamdDmacBusy(pDev->DmaChan); i++)
	{
		if ((reg->SERCOM_STATUS & SAMD_I2CM_BUS_ERRORS) != 0U)
		{
			break;
		}
	}

	return (int)SamdDmacStop(pDev->DmaChan);
}

// Transmit by DMA, the CPU waiting: the address by the CPU, the bytes by the
// channel in blocks of at most SAMD_DMAC_COUNT_MAX, then the end of the last
// byte. Returns the bytes sent, less the last one when it was refused.
static int SamdI2cmTxDma(SamdI2cDev_t *pDev, const uint8_t *pData, int DataLen)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;
	int count = 0;

	if (pDev->bNeedStart && !SamdI2cmAddress(pDev, false))
	{
		return 0;
	}

	while (count < DataLen)
	{
		int len = DataLen - count < SAMD_DMAC_COUNT_MAX ? DataLen - count : SAMD_DMAC_COUNT_MAX;

		SamdI2cmDmaStart(pDev, pData + count, NULL, len);

		int left = SamdI2cmDmaWait(pDev, len);

		count += len - left;
		if (left != 0)
		{
			// The byte on the bus at the error is not counted
			SamdI2cmRecover(pDev, (reg->SERCOM_STATUS & SAMD_I2CM_BUS_ERRORS) == 0U);

			return count > 0 ? count - 1 : 0;
		}
	}

	uint8_t flags = SamdI2cmWait(pDev);
	uint16_t status = reg->SERCOM_STATUS;

	if (flags == 0U || (status & SAMD_I2CM_BUS_ERRORS) != 0U)
	{
		SamdI2cmRecover(pDev, flags == 0U);

		return count - 1;
	}
	if ((status & SERCOM_I2CM_STATUS_RXNACK_Msk) != 0U)
	{
		SamdI2cmStop(pDev);

		return count - 1;
	}

	return count;
}

// Receive by DMA, the CPU waiting: the address and the first byte by the
// CPU, the channel reads all but the last byte, the CPU refuses the last
// one and sends the STOP. One byte by polling.
static int SamdI2cmRxDma(SamdI2cDev_t *pDev, uint8_t *pBuff, int BuffLen)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;
	int count = 0;

	if (BuffLen < 2)
	{
		return SamdI2cmRxPolling(pDev, pBuff, BuffLen);
	}

	if (!SamdI2cmAddress(pDev, true))
	{
		return 0;
	}

	while (count < BuffLen - 1)
	{
		int len = BuffLen - 1 - count < SAMD_DMAC_COUNT_MAX ? BuffLen - 1 - count : SAMD_DMAC_COUNT_MAX;

		SamdI2cmDmaStart(pDev, NULL, pBuff + count, len);

		int left = SamdI2cmDmaWait(pDev, len);

		count += len - left;
		if (left != 0)
		{
			SamdI2cmRecover(pDev, (reg->SERCOM_STATUS & SAMD_I2CM_BUS_ERRORS) == 0U);

			return count;
		}
	}

	uint8_t flags = SamdI2cmWait(pDev);

	if ((flags & SERCOM_I2CM_INTFLAG_SB_Msk) == 0U || (reg->SERCOM_STATUS & SAMD_I2CM_BUS_ERRORS) != 0U)
	{
		SamdI2cmRecover(pDev, flags == 0U);

		return count;
	}

	SamdI2cmCtrlb(pDev, SAMD_I2CM_CMD_STOP, true);
	pDev->bBusHeld = false;
	(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
	pBuff[count++] = reg->SERCOM_DATA;

	return count;
}

// Transfers with the CPU waiting: DMA with bDmaEn, polling otherwise
static int SamdI2cmTxSync(SamdI2cDev_t *pDev, const uint8_t *pData, int DataLen)
{
	return pDev->bDma ? SamdI2cmTxDma(pDev, pData, DataLen) : SamdI2cmTxPolling(pDev, pData, DataLen);
}

static int SamdI2cmRxSync(SamdI2cDev_t *pDev, uint8_t *pBuff, int BuffLen)
{
	return pDev->bDma ? SamdI2cmRxDma(pDev, pBuff, BuffLen) : SamdI2cmRxPolling(pDev, pBuff, BuffLen);
}

// Running DMA block of an interrupt transfer stopped, the bytes it moved
// counted. bError: the transmit byte on the bus at an error is not counted.
static void SamdI2cmDmaEnd(SamdI2cDev_t *pDev, bool bError)
{
	uint32_t state = DisableInterrupt();

	if (pDev->bDmaRun)
	{
		pDev->bDmaRun = false;

		int moved = pDev->DmaLen - (int)SamdDmacStop(pDev->DmaChan);

		if (pDev->Op == SAMD_I2C_OP_TX)
		{
			if (bError && moved > 0)
			{
				moved--;
			}
		}
		else
		{
			pDev->pRxData += moved;
			pDev->Remain -= moved;
		}
		pDev->Count += moved;
	}

	EnableInterrupt(state);
}

// Interrupt transfer by DMA of the bytes left, the SERCOM interrupt on
// errors only until the DMAC interrupt ends the block: transmit of all the
// bytes, receive of all but the last. The state is set before the channel
// starts: its interrupt can come at once.
static void SamdI2cmDmaRun(SamdI2cDev_t *pDev)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;

	reg->SERCOM_INTENCLR = SERCOM_I2CM_INTENCLR_MB_Msk | SERCOM_I2CM_INTENCLR_SB_Msk;
	reg->SERCOM_INTENSET = SERCOM_I2CM_INTENSET_ERROR_Msk;
	pDev->bDmaRun = true;
	if (pDev->Op == SAMD_I2C_OP_TX)
	{
		const uint8_t *p = pDev->pTxData;
		int len = pDev->Remain;

		pDev->pTxData += len;
		pDev->Remain = 0;
		SamdI2cmDmaStart(pDev, p, NULL, len);
	}
	else
	{
		SamdI2cmDmaStart(pDev, NULL, pDev->pRxData, pDev->Remain - 1);
	}
}

// End of a master interrupt transfer taken by the first of the DMAC
// interrupt, the SERCOM interrupt and the caller: SERCOM interrupts off, a
// running DMA block stopped and counted. Returns the operation, none when
// another path took it. The STOP of a transmit is sent by StopTx from
// DeviceIntrfTxComplete.
static SAMD_I2C_OP SamdI2cmTake(SamdI2cDev_t *pDev, bool bError)
{
	uint32_t state = DisableInterrupt();
	SAMD_I2C_OP op = pDev->Op;

	if (op != SAMD_I2C_OP_NONE)
	{
		pDev->pReg->I2CM.SERCOM_INTENCLR = SAMD_I2CM_INTS;
		SamdI2cmDmaEnd(pDev, bError);
		pDev->Op = SAMD_I2C_OP_NONE;
		if (s_pSamdI2cTxOwner == pDev)
		{
			s_pSamdI2cTxOwner = NULL;
		}
	}
	EnableInterrupt(state);

	return op;
}

// Transfer reported, after SamdI2cmTake
static void SamdI2cmFinish(SamdI2cDev_t *pDev, SAMD_I2C_OP Op)
{
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;

	if (Op == SAMD_I2C_OP_TX)
	{
		DeviceIntrfTxComplete(intrf);
	}
	else
	{
		DeviceIntrfStopRx(intrf);
	}

	if (intrf->EvtCB)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, NULL, pDev->Count);
	}
}

static void SamdI2cmComplete(SamdI2cDev_t *pDev)
{
	SAMD_I2C_OP op = SamdI2cmTake(pDev, false);

	if (op != SAMD_I2C_OP_NONE)
	{
		SamdI2cmFinish(pDev, op);
	}
}

// After a bus error: the transfer taken first, so that a DMAC interrupt
// in the recovery does not end it again
static void SamdI2cmAbort(SamdI2cDev_t *pDev)
{
	SAMD_I2C_OP op = SamdI2cmTake(pDev, true);

	if (op != SAMD_I2C_OP_NONE)
	{
		SamdI2cmRecover(pDev, false);
		SamdI2cmFinish(pDev, op);
	}
}

static int SamdI2cmTxInt(SamdI2cDev_t *pDev, const uint8_t *pData, int DataLen)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;
	uint32_t state = DisableInterrupt();
	bool buff = s_pSamdI2cTxOwner == NULL && DataLen <= SAMD_I2C_TXBUF_SIZE;

	if (buff)
	{
		s_pSamdI2cTxOwner = pDev;
	}
	EnableInterrupt(state);

	intrf->bTxReady = false;

	if (!buff)
	{
		pDev->Count = SamdI2cmTxSync(pDev, pData, DataLen);
		pDev->Op = SAMD_I2C_OP_TX;
		SamdI2cmComplete(pDev);

		return -1;
	}

	// The caller data does not outlive the call
	memcpy(s_SamdI2cTxBuf, pData, (size_t)DataLen);
	pDev->pTxData = s_SamdI2cTxBuf;
	pDev->Remain = DataLen;
	pDev->Count = 0;
	pDev->Op = SAMD_I2C_OP_TX;

	if (pDev->bNeedStart)
	{
		pDev->bAddrPhase = true;
		SamdI2cmWaitFree(pDev);
		SamdI2cmCtrlb(pDev, 0, false);
		SamdI2cmSendAddress(pDev, false);
	}
	else if (pDev->bDma)
	{
		// Bus held, MB set by the last byte: the channel starts at once
		uint32_t state = DisableInterrupt();

		pDev->bAddrPhase = false;
		SamdI2cmDmaRun(pDev);
		EnableInterrupt(state);

		return -1;
	}
	else
	{
		pDev->bAddrPhase = false;
		pDev->Remain--;
		reg->SERCOM_DATA = *pDev->pTxData++;
		(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
	}

	reg->SERCOM_INTENSET = SAMD_I2CM_INTS;

	return -1;
}

static int SamdI2cmRxInt(SamdI2cDev_t *pDev, uint8_t *pBuff, int BuffLen)
{
	if (pDev->bDma && BuffLen - 1 > SAMD_DMAC_COUNT_MAX)
	{
		// Longer than a block
		pDev->Count = SamdI2cmRxSync(pDev, pBuff, BuffLen);
		pDev->Op = SAMD_I2C_OP_RX;
		SamdI2cmComplete(pDev);

		return -1;
	}

	pDev->pRxData = pBuff;
	pDev->Remain = BuffLen;
	pDev->Count = 0;
	pDev->Op = SAMD_I2C_OP_RX;

	SamdI2cmWaitFree(pDev);
	SamdI2cmCtrlb(pDev, 0, false);
	SamdI2cmSendAddress(pDev, true);

	pDev->pReg->I2CM.SERCOM_INTENSET = SAMD_I2CM_INTS;

	return -1;
}

// DMAC interrupt of the master channel: the end of the last byte by the
// SERCOM interrupt, MB of a transmit, SB of the last receive byte
static void SamdI2cmDmaHandler(int Chan, uint32_t Flags, void *pCtx)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pCtx;
	bool err = (Flags & DMAC_CHINTFLAG_TERR_Msk) != 0U;

	(void)Chan;

	if (!dev->bDmaRun)
	{
		return;
	}

	if (err)
	{
		SamdI2cmAbort(dev);

		return;
	}

	uint32_t state = DisableInterrupt();

	if (dev->bDmaRun && dev->Op != SAMD_I2C_OP_NONE)
	{
		SamdI2cmDmaEnd(dev, false);
		dev->pReg->I2CM.SERCOM_INTENSET = SAMD_I2CM_INTS;
	}
	EnableInterrupt(state);
}

static void SamdI2cmIrq(SamdI2cDev_t *pDev)
{
	sercom_i2cm_registers_t *reg = &pDev->pReg->I2CM;
	// MB and SB are left to a running DMA block, their interrupts off
	uint8_t flags = reg->SERCOM_INTFLAG & reg->SERCOM_INTENSET;
	uint16_t status = reg->SERCOM_STATUS;

	if (pDev->Op == SAMD_I2C_OP_NONE)
	{
		reg->SERCOM_INTENCLR = SAMD_I2CM_INTS;

		return;
	}

	if ((flags & SERCOM_I2CM_INTFLAG_ERROR_Msk) != 0U || (status & SAMD_I2CM_BUS_ERRORS) != 0U)
	{
		SamdI2cmAbort(pDev);

		return;
	}

	if (pDev->Op == SAMD_I2C_OP_TX)
	{
		if ((flags & SERCOM_I2CM_INTFLAG_MB_Msk) == 0U)
		{
			return;
		}

		pDev->bBusHeld = true;

		// Address or data refused. With DMA the refused byte is the last
		// one the channel sent.
		if ((status & SERCOM_I2CM_STATUS_RXNACK_Msk) != 0U)
		{
			if (pDev->bDma && !pDev->bAddrPhase && pDev->Count > 0)
			{
				pDev->Count--;
			}
			SamdI2cmStop(pDev);
			SamdI2cmComplete(pDev);

			return;
		}

		if (pDev->bAddrPhase)
		{
			pDev->bAddrPhase = false;
			if (pDev->bDma)
			{
				SamdI2cmDmaRun(pDev);

				return;
			}
		}
		else if (!pDev->bDma)
		{
			pDev->Count++;
		}

		if (pDev->Remain > 0)
		{
			pDev->Remain--;
			reg->SERCOM_DATA = *pDev->pTxData++;
			(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);

			return;
		}

		SamdI2cmComplete(pDev);

		return;
	}

	// Receive: MB is the address refused
	if ((flags & SERCOM_I2CM_INTFLAG_MB_Msk) != 0U)
	{
		pDev->bBusHeld = true;
		SamdI2cmStop(pDev);
		SamdI2cmComplete(pDev);

		return;
	}

	if ((flags & SERCOM_I2CM_INTFLAG_SB_Msk) != 0U)
	{
		pDev->bBusHeld = true;

		// First byte in: the channel reads all but the last one
		if (pDev->bDma && pDev->Count == 0 && pDev->Remain >= 2)
		{
			SamdI2cmDmaRun(pDev);

			return;
		}

		if (pDev->Remain <= 1)
		{
			SamdI2cmCtrlb(pDev, SAMD_I2CM_CMD_STOP, true);
			pDev->bBusHeld = false;
		}

		(void)SamdI2cSync(pDev->pReg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
		*pDev->pRxData++ = reg->SERCOM_DATA;
		pDev->Count++;
		pDev->Remain--;

		if (pDev->Remain <= 0)
		{
			SamdI2cmComplete(pDev);
		}
	}
}

// Slave CTRLB in the address and data interrupts. SAM D erratum: a CTRLB
// write there can reset the slave state machine, STATUS is written first and
// both writes are not interrupted.
static void SamdI2csCtrlb(sercom_i2cs_registers_t *pReg, uint32_t Cmd, bool bNack)
{
	uint32_t state = DisableInterrupt();

	pReg->SERCOM_STATUS = 0;
	pReg->SERCOM_CTRLB = SERCOM_I2CS_CTRLB_SMEN_Msk | SERCOM_I2CS_CTRLB_CMD(Cmd) |
						 (bNack ? SERCOM_I2CS_CTRLB_ACKACT_Msk : 0U);

	EnableInterrupt(state);
}

static void SamdI2csFinish(SamdI2cDev_t *pDev)
{
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;
	bool active = pDev->bSlaveTx || pDev->bSlaveRx;
	int count = pDev->SlaveCount;

	pDev->bSlaveTx = false;
	pDev->bSlaveRx = false;
	pDev->SlaveCount = 0;

	if (active && intrf->EvtCB)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, NULL, count);
	}
}

static void SamdI2csIrq(SamdI2cDev_t *pDev)
{
	sercom_i2cs_registers_t *reg = &pDev->pReg->I2CS;
	I2CDev_t *i2c = pDev->pI2cDev;
	DevIntrf_t *intrf = &i2c->DevIntrf;
	uint8_t flags = reg->SERCOM_INTFLAG & reg->SERCOM_INTENSET;

	if ((flags & SERCOM_I2CS_INTFLAG_ERROR_Msk) != 0U)
	{
		reg->SERCOM_STATUS = SAMD_I2CS_ERRORS;
		reg->SERCOM_INTFLAG = SERCOM_I2CS_INTFLAG_ERROR_Msk;
		SamdI2csFinish(pDev);
	}

	// A STOP pending with a new address ends the previous transfer
	if ((flags & SERCOM_I2CS_INTFLAG_PREC_Msk) != 0U)
	{
		reg->SERCOM_INTFLAG = SERCOM_I2CS_INTFLAG_PREC_Msk;
		SamdI2csFinish(pDev);
	}

	if ((flags & SERCOM_I2CS_INTFLAG_AMATCH_Msk) != 0U)
	{
		if ((reg->SERCOM_STATUS & SERCOM_I2CS_STATUS_DIR_Msk) != 0U)
		{
			// Master read, after the bytes of a write when repeated START
			int prior = pDev->bSlaveRx ? pDev->SlaveCount : 0;

			pDev->bSlaveRx = false;
			pDev->bSlaveTx = true;
			pDev->bSlaveFirst = true;
			pDev->SlaveCount = 0;

			if (intrf->EvtCB)
			{
				intrf->EvtCB(intrf, DEVINTRF_EVT_READ_RQST, NULL, prior);
			}
		}
		else
		{
			if (pDev->bSlaveTx && intrf->EvtCB)
			{
				intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, NULL, pDev->SlaveCount);
			}

			pDev->bSlaveTx = false;
			pDev->bSlaveRx = true;
			pDev->SlaveCount = 0;

			if (intrf->EvtCB)
			{
				intrf->EvtCB(intrf, DEVINTRF_EVT_WRITE_RQST, NULL, 0);
			}
		}

		// Address acknowledged, the transfer continues
		SamdI2csCtrlb(reg, SAMD_I2CS_CMD_NEXT, false);

		return;
	}

	if ((flags & SERCOM_I2CS_INTFLAG_DRDY_Msk) == 0U)
	{
		return;
	}

	if (pDev->bSlaveTx)
	{
		if (!pDev->bSlaveFirst && (reg->SERCOM_STATUS & SERCOM_I2CS_STATUS_RXNACK_Msk) != 0U)
		{
			// The master refused the last byte, end of the read
			SamdI2csCtrlb(reg, SAMD_I2CS_CMD_WAIT, false);
		}
		else
		{
			uint8_t data = 0xFF;

			if (i2c->pRRData[0] != NULL && pDev->SlaveCount < i2c->RRDataLen[0])
			{
				data = i2c->pRRData[0][pDev->SlaveCount];
			}
			pDev->bSlaveFirst = false;
			pDev->SlaveCount++;
			reg->SERCOM_DATA = data;
		}
	}
	else if (pDev->bSlaveRx && i2c->pTRBuff[0] != NULL && pDev->SlaveCount < i2c->TRBuffLen[0])
	{
		// Smart mode: reading DATA acknowledges the byte
		i2c->pTRBuff[0][pDev->SlaveCount++] = reg->SERCOM_DATA;
	}
	else
	{
		// No room, the byte is refused
		SamdI2csCtrlb(reg, 0, true);
		(void)reg->SERCOM_DATA;
	}
}

// Master DMAC channel, the interrupt with bIntEn
static bool SamdI2cDmaTake(SamdI2cDev_t *pDev)
{
	I2CDev_t *i2c = pDev->pI2cDev;

	pDev->DmaChan = (int8_t)SamdDmacOpen(SAMD_DMAC_TRIG_SERCOM_TX(pDev->DevNo),
										 i2c->Cfg.bIntEn ? SamdI2cmDmaHandler : NULL, pDev, i2c->Cfg.IntPrio);
	pDev->bDma = pDev->DmaChan >= 0;

	return pDev->bDma;
}

static void SamdI2cDmaGive(SamdI2cDev_t *pDev)
{
	if (pDev->bDma)
	{
		pDev->bDma = false;
		pDev->bDmaRun = false;
		SamdDmacClose(pDev->DmaChan);
		pDev->DmaChan = -1;
	}
}

static void SamdI2cIrqHandler(int DevNo, DevIntrf_t * const pDev)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;

	(void)DevNo;

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		SamdI2csIrq(dev);
	}
	else
	{
		SamdI2cmIrq(dev);
	}
}

static uint64_t SamdI2cCeilDiv(uint64_t Num, uint64_t Den)
{
	return (Num + Den - 1U) / Den;
}

// Speed and SDA hold of CTRLA and the master BAUD of a rate. Returns the SCL
// rate with the assumed rise time, the closest to the request.
static uint32_t SamdI2cTiming(uint32_t Rate, uint32_t *pCtrla, uint32_t *pBaud)
{
	uint32_t fclk = SystemPeriphClockGet(SAMD_GCLK_MAIN);
	uint32_t tlow;
	uint32_t thigh;

	if (Rate == 0U || fclk == 0U)
	{
		return 0;
	}
	if (Rate > SAMD_I2C_RATE_MAX)
	{
		Rate = SAMD_I2C_RATE_MAX;
	}

	if (Rate > I2C_SCL_FAST_MODE_MAX_SPEED * 1000U)
	{
		tlow = I2C_SCL_TLOW_FAST_MODE_PLUS_MIN;
		thigh = I2C_SCL_THIGH_FAST_MODE_PLUS_MIN;
		// Fast mode plus, 50 to 100 ns SDA hold
		*pCtrla = SERCOM_I2CM_CTRLA_SPEED(1U) | SERCOM_I2CM_CTRLA_SDAHOLD(1U);
	}
	else
	{
		if (Rate > I2C_SCL_STD_MODE_MAX_SPEED * 1000U)
		{
			tlow = I2C_SCL_TLOW_FAST_MODE_MIN;
			thigh = I2C_SCL_THIGH_FAST_MODE_MIN;
		}
		else
		{
			tlow = I2C_SCL_TLOW_STD_MODE_MIN;
			thigh = I2C_SCL_THIGH_STD_MODE_MIN;
		}
		// 300 to 600 ns SDA hold
		*pCtrla = SERCOM_I2CM_CTRLA_SPEED(0U) | SERCOM_I2CM_CTRLA_SDAHOLD(2U);
	}

	// fSCL = fclk / (10 + BAUD + BAUDLOW + fclk * Trise)
	// Tlow = (BAUDLOW + 5) / fclk, Thigh = (BAUD + 5) / fclk, the rise time
	// apart: the high time counts from SCL seen high
	int32_t rise = (int32_t)SamdI2cCeilDiv((uint64_t)fclk * SAMD_I2C_TRISE_NS, 1000000000ULL);
	int32_t period = (int32_t)((fclk + Rate / 2U) / Rate);
	int32_t low = (int32_t)SamdI2cCeilDiv((uint64_t)fclk * tlow, 1000000000ULL) - 5;
	int32_t high = (int32_t)SamdI2cCeilDiv((uint64_t)fclk * thigh, 1000000000ULL) - 5;
	int32_t avail = period - 10 - rise;

	if (low < 1)
	{
		low = 1;
	}
	if (high < 1)
	{
		high = 1;
	}

	// The time left over goes to both phases
	if (avail > low + high)
	{
		low += (avail - low - high) / 2;
		high = avail - low;
	}
	if (low > 255)
	{
		low = 255;
	}
	if (high > 255)
	{
		high = 255;
	}

	*pBaud = SERCOM_I2CM_BAUD_BAUD((uint32_t)high) | SERCOM_I2CM_BAUD_BAUDLOW((uint32_t)low);

	return fclk / (uint32_t)(10 + low + high + rise);
}

// Configuration, from the SERCOM reset. Interrupts disabled.
static bool SamdI2cHwInit(SamdI2cDev_t *pDev)
{
	sercom_registers_t *reg = pDev->pReg;
	I2CDev_t *i2c = pDev->pI2cDev;
	IRQn_Type irq = SamdSercomIrq(pDev->DevNo);
	uint32_t timing;
	uint32_t baud;

	NVIC_DisableIRQ(irq);

	reg->I2CM.SERCOM_CTRLA = SERCOM_I2CM_CTRLA_SWRST_Msk;
	if (!SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_SWRST_Msk))
	{
		return false;
	}

	uint32_t rate = SamdI2cTiming(i2c->Cfg.Rate, &timing, &baud);

	if (rate == 0U)
	{
		return false;
	}

	if (i2c->Cfg.Mode == I2CMODE_MASTER)
	{
		pDev->CtrlA = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER | timing;
		pDev->Baud = baud;
		i2c->Cfg.Rate = rate;

		reg->I2CM.SERCOM_CTRLA = pDev->CtrlA;
		reg->I2CM.SERCOM_BAUD = pDev->Baud;
		reg->I2CM.SERCOM_CTRLB = SERCOM_I2CM_CTRLB_SMEN_Msk;
		reg->I2CM.SERCOM_CTRLA = pDev->CtrlA | SERCOM_I2CM_CTRLA_ENABLE_Msk;
		if (!SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_ENABLE_Msk | SERCOM_I2CM_SYNCBUSY_SYSOP_Msk))
		{
			return false;
		}

		// The bus state is unknown after enable
		reg->I2CM.SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE(SAMD_I2CM_BUSSTATE_IDLE);
		if (!SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk))
		{
			return false;
		}
	}
	else
	{
		// The speed of the master, the slave follows its clock
		pDev->CtrlA = SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE | timing;
		i2c->Cfg.Rate = i2c->Cfg.Rate > SAMD_I2C_RATE_MAX ? SAMD_I2C_RATE_MAX : i2c->Cfg.Rate;

		reg->I2CS.SERCOM_CTRLA = pDev->CtrlA;
		reg->I2CS.SERCOM_CTRLB = SERCOM_I2CS_CTRLB_SMEN_Msk;
		reg->I2CS.SERCOM_ADDR = SERCOM_I2CS_ADDR_ADDR(i2c->Cfg.SlaveAddr[0]);
		reg->I2CS.SERCOM_INTFLAG = SAMD_I2CS_INTS;
		reg->I2CS.SERCOM_INTENSET = SAMD_I2CS_INTS;
		reg->I2CS.SERCOM_CTRLA = pDev->CtrlA | SERCOM_I2CS_CTRLA_ENABLE_Msk;
		if (!SamdI2cSync(reg, SERCOM_I2CS_SYNCBUSY_ENABLE_Msk))
		{
			return false;
		}
	}

	if (i2c->DevIntrf.bIntEn)
	{
		NVIC_ClearPendingIRQ(irq);
		NVIC_SetPriority(irq, SamdIrqPrio(i2c->Cfg.IntPrio));
		NVIC_EnableIRQ(irq);
	}

	return true;
}

static uint32_t SamdI2cGetRate(DevIntrf_t * const pDev)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;

	return dev->pI2cDev->Cfg.Rate;
}

// The SERCOM is stopped for the change. Returns the closest rate.
static uint32_t SamdI2cSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;
	sercom_registers_t *reg = dev->pReg;
	I2CDev_t *i2c = dev->pI2cDev;
	uint32_t timing;
	uint32_t baud;
	uint32_t rate = SamdI2cTiming(Rate, &timing, &baud);

	if (rate == 0U || dev->Op != SAMD_I2C_OP_NONE)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	bool enabled = (reg->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_ENABLE_Msk) != 0U;
	uint32_t mode = dev->CtrlA & SERCOM_I2CM_CTRLA_MODE_Msk;

	reg->I2CM.SERCOM_CTRLA = dev->CtrlA;
	(void)SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);

	dev->CtrlA = mode | timing;
	reg->I2CM.SERCOM_CTRLA = dev->CtrlA;

	if (i2c->Cfg.Mode == I2CMODE_MASTER)
	{
		dev->Baud = baud;
		reg->I2CM.SERCOM_BAUD = baud;
		i2c->Cfg.Rate = rate;
	}
	else
	{
		i2c->Cfg.Rate = Rate > SAMD_I2C_RATE_MAX ? SAMD_I2C_RATE_MAX : Rate;
	}

	if (enabled)
	{
		reg->I2CM.SERCOM_CTRLA = dev->CtrlA | SERCOM_I2CM_CTRLA_ENABLE_Msk;
		(void)SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);
		if (i2c->Cfg.Mode == I2CMODE_MASTER)
		{
			reg->I2CM.SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE(SAMD_I2CM_BUSSTATE_IDLE);
			(void)SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
		}
		dev->bBusHeld = false;
	}

	EnableInterrupt(state);

	return i2c->Cfg.Rate;
}

static bool SamdI2cStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;

	if (dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER || DevAddr > 0x7FU || dev->Op != SAMD_I2C_OP_NONE)
	{
		return false;
	}

	dev->DevAddr = (uint8_t)DevAddr;
	dev->bNeedStart = true;

	return true;
}

// Also the repeated START of DeviceIntrfRead
static bool SamdI2cStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return SamdI2cStartTx(pDev, DevAddr);
}

static int SamdI2cTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;

	// bDmaEn without the channel: given back by PowerOff
	if (pData == NULL || DataLen <= 0 || dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER ||
		dev->pI2cDev->Cfg.bDmaEn != dev->bDma)
	{
		return 0;
	}

	if (pDev->bIntEn && !pDev->bNoStop)
	{
		return SamdI2cmTxInt(dev, pData, DataLen);
	}

	return SamdI2cmTxSync(dev, pData, DataLen);
}

static int SamdI2cRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;

	if (pBuff == NULL || BuffLen <= 0 || dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER ||
		dev->pI2cDev->Cfg.bDmaEn != dev->bDma)
	{
		return 0;
	}

	return pDev->bIntEn ? SamdI2cmRxInt(dev, pBuff, BuffLen) : SamdI2cmRxSync(dev, pBuff, BuffLen);
}

static void SamdI2cStopTx(DevIntrf_t * const pDev)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_MASTER && !pDev->bNoStop)
	{
		SamdI2cmStop(dev);
	}
}

static void SamdI2cStopRx(DevIntrf_t * const pDev)
{
	SamdI2cStopTx(pDev);
}

static void SamdI2cDisable(DevIntrf_t * const pDev)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;
	sercom_registers_t *reg = dev->pReg;

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_MASTER)
	{
		// A channel running is stopped, its transfer is not reported: Reset
		// ends it
		SamdI2cmDmaEnd(dev, true);
		SamdI2cmStop(dev);
		reg->I2CM.SERCOM_INTENCLR = SAMD_I2CM_INTS;
	}
	else
	{
		reg->I2CS.SERCOM_INTENCLR = SAMD_I2CS_INTS;
	}

	reg->I2CM.SERCOM_CTRLA = dev->CtrlA;
	(void)SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);
	SamdSercomClockDisable(dev->DevNo);
}

static void SamdI2cEnable(DevIntrf_t * const pDev)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;
	sercom_registers_t *reg = dev->pReg;
	I2CDev_t *i2c = dev->pI2cDev;

	if (!SamdSercomClockEnable(dev->DevNo))
	{
		return;
	}
	IOPinCfg(i2c->Cfg.pIOPinMap, 2);

	reg->I2CM.SERCOM_CTRLA = dev->CtrlA | SERCOM_I2CM_CTRLA_ENABLE_Msk;
	(void)SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);

	if (i2c->Cfg.Mode == I2CMODE_MASTER)
	{
		reg->I2CM.SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE(SAMD_I2CM_BUSSTATE_IDLE);
		(void)SamdI2cSync(reg, SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
		dev->bBusHeld = false;
	}
	else
	{
		reg->I2CS.SERCOM_INTENSET = SAMD_I2CS_INTS;
	}
}

// Bus clocked free, then the SERCOM configured again
static void SamdI2cReset(DevIntrf_t * const pDev)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;

	I2CBusReset(dev->pI2cDev);

	uint32_t state = DisableInterrupt();

	SamdI2cmDmaEnd(dev, true);
	dev->Op = SAMD_I2C_OP_NONE;
	dev->bBusHeld = false;
	dev->bSlaveTx = false;
	dev->bSlaveRx = false;
	if (s_pSamdI2cTxOwner == dev)
	{
		s_pSamdI2cTxOwner = NULL;
	}
	(void)SamdI2cHwInit(dev);

	EnableInterrupt(state);
}

static void SamdI2cPowerOff(DevIntrf_t * const pDev)
{
	SamdI2cDev_t *dev = (SamdI2cDev_t *)pDev->pDevData;

	SamdI2cDisable(pDev);

	uint32_t state = DisableInterrupt();

	SamdI2cDmaGive(dev);
	EnableInterrupt(state);

	IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, 2);
}

static void *SamdI2cGetHandle(DevIntrf_t * const pDev)
{
	return ((SamdI2cDev_t *)pDev->pDevData)->pI2cDev;
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL || pCfgData->pIOPinMap == NULL || pCfgData->NbIOPins < 2 ||
		pCfgData->DevNo < 0 || pCfgData->DevNo >= SAMD_SERCOM_CNT || pCfgData->Type != I2CTYPE_STANDARD ||
		pCfgData->AddrType != I2CADDR_TYPE_NORMAL || pCfgData->Rate == 0U ||
		(pCfgData->Mode != I2CMODE_MASTER && pCfgData->Mode != I2CMODE_SLAVE) ||
		(pCfgData->bDmaEn && pCfgData->Mode == I2CMODE_SLAVE))
	{
		return false;
	}

	if (pCfgData->Mode == I2CMODE_SLAVE && (pCfgData->NbSlaveAddr != 1 || pCfgData->SlaveAddr[0] > 0x7FU))
	{
		return false;
	}

	// The slave runs on interrupts
	bool intEn = pCfgData->Mode == I2CMODE_SLAVE || pCfgData->bIntEn;

	if (intEn && pCfgData->IntPrio < 0)
	{
		return false;
	}

	const IOPinCfg_t *sda = &pCfgData->pIOPinMap[I2C_SDA_IOPIN_IDX];
	const IOPinCfg_t *scl = &pCfgData->pIOPinMap[I2C_SCL_IOPIN_IDX];

	if (SamdSercomPad(pCfgData->DevNo, sda) != 0 || SamdSercomPad(pCfgData->DevNo, scl) != 1 ||
		!SamdSercomI2cPin(sda) || !SamdSercomI2cPin(scl))
	{
		return false;
	}

	SamdI2cDev_t *dev = &s_SamdI2cDev[pCfgData->DevNo];
	uint32_t state = DisableInterrupt();

	if (SamdSercomOwned(pCfgData->DevNo, &pDev->DevIntrf) || !SamdSercomClockEnable(pCfgData->DevNo))
	{
		EnableInterrupt(state);

		return false;
	}

	if (s_pSamdI2cTxOwner == dev)
	{
		s_pSamdI2cTxOwner = NULL;
	}

	// Channel of an earlier I2CInit given back
	SamdI2cDmaGive(dev);
	memset(dev, 0, sizeof(SamdI2cDev_t));
	dev->DevNo = pCfgData->DevNo;
	dev->pReg = SamdSercomGetReg(pCfgData->DevNo);
	dev->pI2cDev = pDev;
	dev->DmaChan = -1;

	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));
	pDev->Cfg.bIntEn = intEn;

	if (pCfgData->bDmaEn && !SamdI2cDmaTake(dev))
	{
		// The SERCOM released as by a failed configuration below
		SharedIntrfSetIrqHandler(pCfgData->DevNo, NULL, NULL);
		SamdSercomClockDisable(pCfgData->DevNo);
		EnableInterrupt(state);

		return false;
	}
	memset(pDev->pRRData, 0, sizeof(pDev->pRRData));
	memset(pDev->RRDataLen, 0, sizeof(pDev->RRDataLen));
	memset(pDev->pTRBuff, 0, sizeof(pDev->pTRBuff));
	memset(pDev->TRBuffLen, 0, sizeof(pDev->TRBuffLen));

	pDev->DevIntrf.pDevData = dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.bIntEn = intEn;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.Disable = SamdI2cDisable;
	pDev->DevIntrf.Enable = SamdI2cEnable;
	pDev->DevIntrf.GetRate = SamdI2cGetRate;
	pDev->DevIntrf.SetRate = SamdI2cSetRate;
	pDev->DevIntrf.StartRx = SamdI2cStartRx;
	pDev->DevIntrf.RxData = SamdI2cRxData;
	pDev->DevIntrf.StopRx = SamdI2cStopRx;
	pDev->DevIntrf.StartTx = SamdI2cStartTx;
	pDev->DevIntrf.TxData = SamdI2cTxData;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.StopTx = SamdI2cStopTx;
	pDev->DevIntrf.Reset = SamdI2cReset;
	pDev->DevIntrf.PowerOff = SamdI2cPowerOff;
	pDev->DevIntrf.GetHandle = SamdI2cGetHandle;
	pDev->DevIntrf.IntPrio = (int)SamdIrqPrio(pCfgData->IntPrio);
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	SharedIntrfSetIrqHandler(pCfgData->DevNo, &pDev->DevIntrf, SamdI2cIrqHandler);

	IOPinCfg(pCfgData->pIOPinMap, 2);

	bool res = SamdI2cHwInit(dev);

	if (!res)
	{
		SamdI2cDmaGive(dev);
		SharedIntrfSetIrqHandler(pCfgData->DevNo, NULL, NULL);
		IOPinDis(pCfgData->pIOPinMap, 2);
		SamdSercomClockDisable(pCfgData->DevNo);
	}

	EnableInterrupt(state);

	return res;
}
