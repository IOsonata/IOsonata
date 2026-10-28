/**-------------------------------------------------------------------------
@file	i2c_lpc546xx.cpp

@brief	LPC546xx Flexcomm I2C implementation

		DevNo is the Flexcomm number, 0 to 9. The function clock is the
		12 MHz FRO. 7 bit addresses, SMBus and 10 bit addresses fail
		I2CInit.

		Master. StartTx and StartRx send the START, or the repeated START
		after an earlier phase, with the address and return false when it is
		not acknowledged. They wait for the address phase in every mode.
		StopTx and StopRx send the STOP when the bus is still held.

		Polling (bIntEn and bDmaEn false): TxData and RxData move the data
		bytes. A read ends with a NACK on its last byte, sent with the STOP
		or the next START, so a receive can take several RxData calls.

		Interrupt and DMA: TxData and RxData are each one complete transfer
		ending with the STOP. A transmit of a command phase before a read
		(the interface bNoStop set) always runs as in polling. With bIntEn
		they return -1, the Flexcomm interrupt moves the bytes, and the end
		is reported with DEVINTRF_EVT_COMPLETED and the byte count after the
		bus is released (DeviceIntrfTxComplete or DeviceIntrfStopRx). A
		transmit of up to LPC546XX_I2C_TXBUF_SIZE bytes is copied first, a
		longer one waits until its last byte is taken. With bDmaEn the data
		bytes are moved by the DMA channel of the Flexcomm master request
		(MSTDMA), the CPU takes the last byte of a receive and sends the
		STOP. Without bIntEn the call waits for the end of the transfer.

		Slave. Interrupt driven, bIntEn is forced. One to four 7 bit
		addresses (SLVADR0 to 3). The slave stretches SCL until the
		interrupt has answered. The sequence follows the other IOsonata I2C
		slaves: a master write raises DEVINTRF_EVT_WRITE_RQST, the
		application sets the receive buffer with SetWriteRqstBuffer, the
		bytes are stored and the end is reported with
		DEVINTRF_EVT_COMPLETED and the byte count. A master read raises
		DEVINTRF_EVT_READ_RQST with the byte count of a write that came
		before it with a repeated START, the application sets the data with
		SetReadRqstData; bytes past the data are sent as 0xFF and not
		counted. Bytes received past the buffer are acknowledged and dropped,
		not counted. The end of a request is the slave deselect (STOP, the
		NACK of the master at the end of a read, or another address). With
		bDmaEn the DMA channel of the Flexcomm slave request moves the data
		up to the buffer length, the interrupt takes the bytes past it.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "LPC546xx.h"

#include "istddef.h"
#include "coredev/iopincfg.h"
#include "coredev/i2c.h"
#include "coredev/interrupt.h"
#include "coredev/shared_intrf.h"
#include "flexcomm_lpc546xx.h"
#include "dma_lpc546xx.h"

#define LPC546XX_I2C_WAIT_MAX		1000000UL	// STAT reads before a wait gives up
#define LPC546XX_I2C_ADDR_MAX		0x7FU		// 7 bit address
#define LPC546XX_I2C_READ			1U			// Address R/W bit for a read
#define LPC546XX_I2C_DIV_MAX		0x10000U	// CLKDIV DIVVAL + 1
#define LPC546XX_I2C_SCL_CLK_MIN	2U			// MSTSCLLOW and MSTSCLHIGH + 2
#define LPC546XX_I2C_SCL_CLK_MAX	9U
#define LPC546XX_I2C_SLVADR_CNT		4			// SLVADR0 to SLVADR3
#define LPC546XX_I2C_FILLER			0xFFU		// Slave byte sent past the read data

// Transmit bytes copied by an interrupt or DMA TxData, see Lpc546xxI2CTxData
#define LPC546XX_I2C_TXBUF_SIZE		64

// MSTSTATE values
#define LPC546XX_I2C_MST_IDLE		0
#define LPC546XX_I2C_MST_RXRDY		1
#define LPC546XX_I2C_MST_TXRDY		2
#define LPC546XX_I2C_MST_NACKADR	3
#define LPC546XX_I2C_MST_NACKDAT	4

// SLVSTATE values
#define LPC546XX_I2C_SLV_ADDR		0
#define LPC546XX_I2C_SLV_RX			1
#define LPC546XX_I2C_SLV_TX			2

// Request served by the slave
#define LPC546XX_I2C_S_IDLE			0
#define LPC546XX_I2C_S_WRITE		1		// The master writes, the slave receives
#define LPC546XX_I2C_S_READ			2		// The master reads, the slave transmits

// Bus errors, cleared by writing 1
#define LPC546XX_I2C_STAT_ERR		(I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK)

#define LPC546XX_I2C_MST_INTS		(I2C_INTENSET_MSTPENDINGEN_MASK | I2C_INTENSET_MSTARBLOSSEN_MASK | \
									 I2C_INTENSET_MSTSTSTPERREN_MASK)
#define LPC546XX_I2C_SLV_INTS		(I2C_INTENSET_SLVPENDINGEN_MASK | I2C_INTENSET_SLVDESELEN_MASK)

#pragma pack(push, 4)

typedef struct __Lpc546xx_I2C_Dev {
	I2C_Type *pReg;					//!< Flexcomm I2C registers
	int DevNo;						//!< Flexcomm number
	uint32_t FClk;					//!< Function clock frequency
	I2CDev_t *pI2cDev;				//!< Generic I2C device data
	bool bActive;					//!< Address acknowledged, transfer in progress
	bool bRxData;					//!< A received byte is waiting in MSTDAT
	bool bDmaClaimed;				//!< DMA channel taken by this instance
	// Master transfer of the interrupt and DMA modes
	volatile bool bRun;				//!< Transfer running
	bool bAsync;					//!< Ended by the interrupt
	bool bRx;						//!< Direction of the transfer
	bool bStop;						//!< STOP sent, the transfer ends when the master is idle
	volatile bool bDmaRun;			//!< DMA moving the data
	const uint8_t *pTx;				//!< Data sent
	uint8_t *pRx;					//!< Receive buffer
	int Len;						//!< Bytes of the transfer
	int Done;						//!< Bytes moved, acknowledged ones at the end of a transmit
	int DmaLen;						//!< Bytes of the running DMA block
	// Slave
	uint8_t SState;					//!< Request served, LPC546XX_I2C_S_xxx
	uint8_t SIdx;					//!< Index of the address matched
	bool bSDmaRun;					//!< DMA moving the data of the request
	int SCount;						//!< Bytes of the request
	int SDmaLen;					//!< Bytes of the running DMA block
	uint8_t TxBuf[LPC546XX_I2C_TXBUF_SIZE];	//!< Copy of the data of a transmit
} Lpc546xxI2CDev_t;

#pragma pack(pop)

// Registers of each Flexcomm, the instance data is set by I2CInit
static I2C_Type * const s_Lpc546xxI2CReg[] = {
	I2C0, I2C1, I2C2, I2C3, I2C4, I2C5, I2C6, I2C7, I2C8, I2C9,
};

static const int s_NbI2CDev = sizeof(s_Lpc546xxI2CReg) / sizeof(I2C_Type *);

static Lpc546xxI2CDev_t s_Lpc546xxI2CDev[sizeof(s_Lpc546xxI2CReg) / sizeof(I2C_Type *)];

static int Lpc546xxI2CWait(Lpc546xxI2CDev_t * const pDev);
static bool Lpc546xxI2CStart(DevIntrf_t * const pDev, uint32_t DevAddr, uint32_t Rw);
static void Lpc546xxI2CStop(DevIntrf_t * const pDev);
static int Lpc546xxI2CDmaCh(Lpc546xxI2CDev_t * const pDev);
static void Lpc546xxI2CMstDmaStart(Lpc546xxI2CDev_t * const pDev);
static void Lpc546xxI2CMstDmaDone(Lpc546xxI2CDev_t * const pDev);
static void Lpc546xxI2CMstEnd(Lpc546xxI2CDev_t * const pDev);
static void Lpc546xxI2CMstAbort(Lpc546xxI2CDev_t * const pDev, uint32_t Stat);
static void Lpc546xxI2CMstService(Lpc546xxI2CDev_t * const pDev);
static int Lpc546xxI2CMstRun(Lpc546xxI2CDev_t * const pDev, bool bRx, const uint8_t *pTx, uint8_t *pRx, int Len);
static void Lpc546xxI2CSlvEvent(Lpc546xxI2CDev_t * const pDev, DEVINTRF_EVT Evt, int Len);
static void Lpc546xxI2CSlvDmaEnd(Lpc546xxI2CDev_t * const pDev);
static void Lpc546xxI2CSlvAddress(Lpc546xxI2CDev_t * const pDev, uint32_t Stat);
static void Lpc546xxI2CSlvService(Lpc546xxI2CDev_t * const pDev);
static void Lpc546xxI2CIrqHandler(int DevNo, DevIntrf_t * const pDev);
static void Lpc546xxI2CDmaIrq(int Ch, void *pCtx);

static uint32_t Lpc546xxI2CGetRate(DevIntrf_t * const pDev);
static uint32_t Lpc546xxI2CSetRate(DevIntrf_t * const pDev, uint32_t Rate);
static bool Lpc546xxI2CStartRx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Lpc546xxI2CRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen);
static bool Lpc546xxI2CStartTx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Lpc546xxI2CTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen);
static void Lpc546xxI2CDisable(DevIntrf_t * const pDev);
static void Lpc546xxI2CEnable(DevIntrf_t * const pDev);
static void Lpc546xxI2CPowerOff(DevIntrf_t * const pDev);
static void Lpc546xxI2CReset(DevIntrf_t * const pDev);
static void *Lpc546xxI2CGetHandle(DevIntrf_t * const pDev);

// Wait until the master needs service. Returns the master state, or -1 on
// timeout, arbitration loss or START/STOP error.
static int Lpc546xxI2CWait(Lpc546xxI2CDev_t * const pDev)
{
	I2C_Type *reg = pDev->pReg;

	for (uint32_t n = LPC546XX_I2C_WAIT_MAX; n > 0; n--)
	{
		uint32_t stat = reg->STAT;

		if (stat & LPC546XX_I2C_STAT_ERR)
		{
			reg->STAT = stat & LPC546XX_I2C_STAT_ERR;

			return -1;
		}
		if (stat & I2C_STAT_MSTPENDING_MASK)
		{
			return (int)((stat & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT);
		}
	}

	return -1;
}

// START or repeated START with the address. On NACK the bus is released.
static bool Lpc546xxI2CStart(DevIntrf_t * const pDev, uint32_t DevAddr, uint32_t Rw)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	I2C_Type *reg = dev->pReg;

	dev->bRxData = false;

	if (dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER || dev->bRun ||
		DevAddr > LPC546XX_I2C_ADDR_MAX || Lpc546xxI2CWait(dev) < 0)
	{
		dev->bActive = false;

		return false;
	}

	reg->MSTDAT = (DevAddr << 1) | Rw;
	reg->MSTCTL = I2C_MSTCTL_MSTSTART_MASK;

	int state = Lpc546xxI2CWait(dev);

	if (state == (Rw == LPC546XX_I2C_READ ? LPC546XX_I2C_MST_RXRDY : LPC546XX_I2C_MST_TXRDY))
	{
		dev->bActive = true;
		dev->bRxData = Rw == LPC546XX_I2C_READ;

		return true;
	}

	if (state == LPC546XX_I2C_MST_NACKADR || state == LPC546XX_I2C_MST_NACKDAT)
	{
		reg->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
		(void)Lpc546xxI2CWait(dev);
	}
	dev->bActive = false;

	return false;
}

// STOP when the master holds the bus. After a read it is preceded by the
// NACK of the last byte. Nothing to do after an interrupt or DMA transfer,
// which ends with its STOP.
static void Lpc546xxI2CStop(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;

	if (dev->pI2cDev->Cfg.Mode != I2CMODE_MASTER || dev->bRun)
	{
		return;
	}

	if (Lpc546xxI2CWait(dev) > LPC546XX_I2C_MST_IDLE)
	{
		dev->pReg->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
		(void)Lpc546xxI2CWait(dev);
	}

	dev->bActive = false;
	dev->bRxData = false;
}

// Master and slave requests of the Flexcomm have their own channel
static int Lpc546xxI2CDmaCh(Lpc546xxI2CDev_t * const pDev)
{
	return pDev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE ? LPC546XX_DMA_CH_FC_RX(pDev->DevNo) :
		   LPC546XX_DMA_CH_FC_TX(pDev->DevNo);
}

// Next DMA block of the master transfer, from Done. A receive leaves its
// last byte to the CPU.
static void Lpc546xxI2CMstDmaStart(Lpc546xxI2CDev_t * const pDev)
{
	int ch = Lpc546xxI2CDmaCh(pDev);
	int end = pDev->bRx ? pDev->Len - 1 : pDev->Len;
	int len = end - pDev->Done;

	if (len > (int)LPC546XX_DMA_XFER_MAX)
	{
		len = LPC546XX_DMA_XFER_MAX;
	}

	pDev->DmaLen = len;
	pDev->bDmaRun = true;

	if (pDev->bRx)
	{
		Lpc546xxDmaChSetup(ch, false, &pDev->pReg->MSTDAT, 0);
		Lpc546xxDmaChStart(ch, &pDev->pRx[pDev->Done], (uint32_t)len);
	}
	else
	{
		Lpc546xxDmaChSetup(ch, true, &pDev->pReg->MSTDAT, 0);
		Lpc546xxDmaChStart(ch, &pDev->pTx[pDev->Done], (uint32_t)len);
	}
}

// End of a master DMA block: the next one, or MSTDMA cleared so that the
// master is pending for the last byte of a receive or the STOP. A DMA error
// ends the transfer with the bytes moved.
static void Lpc546xxI2CMstDmaDone(Lpc546xxI2CDev_t * const pDev)
{
	int ch = Lpc546xxI2CDmaCh(pDev);

	if (pDev->bDmaRun == false)
	{
		return;
	}

	uint32_t flags = Lpc546xxDmaChFlags(ch);

	if ((flags & (LPC546XX_DMA_FLAG_DONE | LPC546XX_DMA_FLAG_ERR)) == 0U)
	{
		return;
	}

	pDev->bDmaRun = false;

	if (flags & LPC546XX_DMA_FLAG_ERR)
	{
		pDev->Done += pDev->DmaLen - (int)Lpc546xxDmaChStopRemain(ch);
		pDev->pReg->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
		pDev->bStop = true;

		return;
	}

	Lpc546xxDmaChClear(ch);
	pDev->Done += pDev->DmaLen;

	if (pDev->Done < (pDev->bRx ? pDev->Len - 1 : pDev->Len))
	{
		Lpc546xxI2CMstDmaStart(pDev);
	}
	else
	{
		pDev->pReg->MSTCTL = 0;
	}
}

// The master is idle after the STOP of the transfer
static void Lpc546xxI2CMstEnd(Lpc546xxI2CDev_t * const pDev)
{
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;
	int count = pDev->Done;
	bool rx = pDev->bRx;

	pDev->pReg->INTENCLR = LPC546XX_I2C_MST_INTS;
	pDev->bRun = false;
	pDev->bStop = false;
	pDev->bActive = false;
	pDev->bRxData = false;

	if (pDev->bAsync == false)
	{
		return;
	}

	pDev->bAsync = false;
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
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, rx ? pDev->pRx : NULL, count);
	}
}

// A transfer that stopped moving: the bytes of the DMA block are counted, a
// pending master sends the STOP, a busy one (SCL held by the slave) accepts no
// command and its function is reset. Called with interrupts disabled, Stat is
// the last STAT read.
static void Lpc546xxI2CMstAbort(Lpc546xxI2CDev_t * const pDev, uint32_t Stat)
{
	I2C_Type *reg = pDev->pReg;

	if (pDev->bRun == false)
	{
		return;
	}
	if (pDev->bDmaRun)
	{
		pDev->Done += pDev->DmaLen - (int)Lpc546xxDmaChStopRemain(Lpc546xxI2CDmaCh(pDev));
		pDev->bDmaRun = false;
	}
	if (Stat & I2C_STAT_MSTPENDING_MASK)
	{
		reg->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
	}
	else
	{
		reg->CFG = 0;
		reg->CFG = I2C_CFG_MSTEN_MASK;
	}
	Lpc546xxI2CMstEnd(pDev);
}

// One step of the master transfer when the master is pending, from the
// interrupt or the wait of a call without bIntEn
static void Lpc546xxI2CMstService(Lpc546xxI2CDev_t * const pDev)
{
	I2C_Type *reg = pDev->pReg;
	uint32_t stat = reg->STAT;

	if (pDev->bRun == false)
	{
		return;
	}

	if (stat & LPC546XX_I2C_STAT_ERR)
	{
		// The master has stopped driving the bus and is idle
		reg->STAT = stat & LPC546XX_I2C_STAT_ERR;
		if (pDev->bDmaRun)
		{
			pDev->Done += pDev->DmaLen - (int)Lpc546xxDmaChStopRemain(Lpc546xxI2CDmaCh(pDev));
			pDev->bDmaRun = false;
		}
		reg->MSTCTL = 0;
		Lpc546xxI2CMstEnd(pDev);

		return;
	}

	if ((stat & I2C_STAT_MSTPENDING_MASK) == 0U)
	{
		return;
	}

	int state = (int)((stat & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT);

	if (pDev->bStop)
	{
		if (state == LPC546XX_I2C_MST_IDLE)
		{
			Lpc546xxI2CMstEnd(pDev);
		}

		return;
	}

	if (pDev->bDmaRun)
	{
		// The DMA does not handle a NACK, the master is pending for it
		pDev->Done += pDev->DmaLen - (int)Lpc546xxDmaChStopRemain(Lpc546xxI2CDmaCh(pDev));
		pDev->bDmaRun = false;
	}

	switch (state)
	{
		case LPC546XX_I2C_MST_TXRDY:
			if (pDev->bRx == false && pDev->Done < pDev->Len)
			{
				reg->MSTDAT = pDev->pTx[pDev->Done++];
				reg->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK;

				return;
			}
			break;

		case LPC546XX_I2C_MST_RXRDY:
			if (pDev->bRx && pDev->Done < pDev->Len)
			{
				pDev->pRx[pDev->Done++] = (uint8_t)reg->MSTDAT;
				if (pDev->Done < pDev->Len)
				{
					reg->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK;

					return;
				}
			}
			break;

		case LPC546XX_I2C_MST_NACKDAT:
			// The last byte written was not acknowledged
			if (pDev->bRx == false && pDev->Done > 0)
			{
				pDev->Done--;
			}
			break;

		case LPC546XX_I2C_MST_IDLE:
			// Not expected while the transfer runs, the bus is free
			reg->MSTCTL = 0;
			Lpc546xxI2CMstEnd(pDev);

			return;

		default:
			break;
	}

	// The STOP, preceded by the NACK of the last byte of a receive. It also
	// clears MSTDMA.
	reg->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
	pDev->bStop = true;
}

// Interrupt or DMA transfer, from the address phase done by StartTx or
// StartRx to the STOP. Returns -1 with bIntEn, the end comes from the
// interrupt, otherwise the bytes moved.
static int Lpc546xxI2CMstRun(Lpc546xxI2CDev_t * const pDev, bool bRx, const uint8_t *pTx, uint8_t *pRx, int Len)
{
	I2C_Type *reg = pDev->pReg;
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;
	bool async = intrf->bIntEn;

	pDev->bRx = bRx;
	pDev->pTx = pTx;
	pDev->pRx = pRx;
	pDev->Len = Len;
	pDev->Done = 0;
	pDev->bStop = false;
	pDev->bDmaRun = false;
	pDev->bAsync = async;
	pDev->bRun = true;

	if (bRx == false)
	{
		atomic_store(&intrf->bTxReady, false);
	}

	uint32_t state = DisableInterrupt();

	// A DMA receive of one byte has nothing for the DMA, the CPU takes it
	if (intrf->bDma && (bRx == false || Len > 1))
	{
		Lpc546xxI2CMstDmaStart(pDev);
		reg->MSTCTL = I2C_MSTCTL_MSTDMA_MASK;
	}

	if (async)
	{
		// The master is already pending, the interrupt comes at once
		reg->INTENSET = LPC546XX_I2C_MST_INTS;
		EnableInterrupt(state);

		return -1;
	}

	EnableInterrupt(state);

	uint32_t left = LPC546XX_I2C_WAIT_MAX;
	int prior = -1;

	while (pDev->bRun)
	{
		Lpc546xxI2CMstDmaDone(pDev);
		Lpc546xxI2CMstService(pDev);

		// The wait is bounded from the last progress
		int progress = __atomic_load_n(&pDev->Done, __ATOMIC_RELAXED) + (pDev->bStop ? 0x10000 : 0);

		if (progress != prior)
		{
			prior = progress;
			left = LPC546XX_I2C_WAIT_MAX;
		}
		else if (--left == 0)
		{
			Lpc546xxI2CMstAbort(pDev, reg->STAT);
		}
	}

	if (bRx == false)
	{
		atomic_store(&intrf->bTxReady, true);
	}

	return pDev->Done;
}

static void Lpc546xxI2CSlvEvent(Lpc546xxI2CDev_t * const pDev, DEVINTRF_EVT Evt, int Len)
{
	DevIntrf_t *intrf = &pDev->pI2cDev->DevIntrf;

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, Evt, NULL, Len);
	}
}

// The DMA block of the request ends: the bytes it moved are the count, the
// interrupt takes the bytes past it.
static void Lpc546xxI2CSlvDmaEnd(Lpc546xxI2CDev_t * const pDev)
{
	if (pDev->bSDmaRun == false)
	{
		return;
	}

	pDev->SCount = pDev->SDmaLen - (int)Lpc546xxDmaChStopRemain(Lpc546xxI2CDmaCh(pDev));
	pDev->bSDmaRun = false;

	// SLVDMA off, nothing else written
	pDev->pReg->SLVCTL = 0;
}

// Address matched, read from SLVDAT with the direction bit. A repeated
// START comes here without a deselect before it.
static void Lpc546xxI2CSlvAddress(Lpc546xxI2CDev_t * const pDev, uint32_t Stat)
{
	I2C_Type *reg = pDev->pReg;
	I2CDev_t *i2c = pDev->pI2cDev;
	bool read = (reg->SLVDAT & LPC546XX_I2C_READ) != 0U;
	int idx = (int)((Stat & I2C_STAT_SLVIDX_MASK) >> I2C_STAT_SLVIDX_SHIFT);
	int prior = 0;

	Lpc546xxI2CSlvDmaEnd(pDev);

	if (pDev->SState == LPC546XX_I2C_S_WRITE)
	{
		prior = pDev->SCount;
	}
	else if (pDev->SState == LPC546XX_I2C_S_READ)
	{
		Lpc546xxI2CSlvEvent(pDev, DEVINTRF_EVT_COMPLETED, pDev->SCount);
	}

	pDev->SIdx = (uint8_t)idx;
	pDev->SCount = 0;
	pDev->SState = read ? LPC546XX_I2C_S_READ : LPC546XX_I2C_S_WRITE;

	// The application installs the buffer again for every request
	uint8_t *buf;
	int len;

	if (read)
	{
		i2c->pRRData[idx] = NULL;
		i2c->RRDataLen[idx] = 0;
		Lpc546xxI2CSlvEvent(pDev, DEVINTRF_EVT_READ_RQST, prior);
		buf = i2c->pRRData[idx];
		len = i2c->RRDataLen[idx];
	}
	else
	{
		i2c->pTRBuff[idx] = NULL;
		i2c->TRBuffLen[idx] = 0;
		if (prior != 0)
		{
			Lpc546xxI2CSlvEvent(pDev, DEVINTRF_EVT_COMPLETED, prior);
		}
		Lpc546xxI2CSlvEvent(pDev, DEVINTRF_EVT_WRITE_RQST, 0);
		buf = i2c->pTRBuff[idx];
		len = i2c->TRBuffLen[idx];
	}

	uint32_t ctl = I2C_SLVCTL_SLVCONTINUE_MASK;

	if (i2c->DevIntrf.bDma && buf != NULL && len > 0)
	{
		int ch = Lpc546xxI2CDmaCh(pDev);

		if (len > (int)LPC546XX_DMA_XFER_MAX)
		{
			len = LPC546XX_DMA_XFER_MAX;
		}
		Lpc546xxDmaChSetup(ch, read, &reg->SLVDAT, 0);
		if (Lpc546xxDmaChStart(ch, buf, (uint32_t)len))
		{
			pDev->SDmaLen = len;
			pDev->bSDmaRun = true;
			ctl |= I2C_SLVCTL_SLVDMA_MASK;
		}
	}

	// The address is acknowledged
	reg->SLVCTL = ctl;
}

static void Lpc546xxI2CSlvService(Lpc546xxI2CDev_t * const pDev)
{
	I2C_Type *reg = pDev->pReg;
	I2CDev_t *i2c = pDev->pI2cDev;
	uint32_t stat = reg->STAT;

	// End of the request: STOP, the NACK of the master after its last byte
	// or another address
	if (stat & I2C_STAT_SLVDESEL_MASK)
	{
		reg->STAT = I2C_STAT_SLVDESEL_MASK;
		Lpc546xxI2CSlvDmaEnd(pDev);
		if (pDev->SState != LPC546XX_I2C_S_IDLE)
		{
			int count = pDev->SCount;

			pDev->SState = LPC546XX_I2C_S_IDLE;
			pDev->SCount = 0;
			Lpc546xxI2CSlvEvent(pDev, DEVINTRF_EVT_COMPLETED, count);
		}
		stat = reg->STAT;
	}

	if ((stat & I2C_STAT_SLVPENDING_MASK) == 0U)
	{
		return;
	}

	switch ((stat & I2C_STAT_SLVSTATE_MASK) >> I2C_STAT_SLVSTATE_SHIFT)
	{
		case LPC546XX_I2C_SLV_ADDR:
			Lpc546xxI2CSlvAddress(pDev, stat);
			break;

		case LPC546XX_I2C_SLV_RX:
			{
				uint8_t d = (uint8_t)reg->SLVDAT;
				uint8_t *buf = i2c->pTRBuff[pDev->SIdx];

				if (pDev->SState == LPC546XX_I2C_S_WRITE && buf != NULL &&
					pDev->SCount < i2c->TRBuffLen[pDev->SIdx])
				{
					buf[pDev->SCount++] = d;
				}
				reg->SLVCTL = I2C_SLVCTL_SLVCONTINUE_MASK;
			}
			break;

		case LPC546XX_I2C_SLV_TX:
			{
				const uint8_t *data = i2c->pRRData[pDev->SIdx];

				if (pDev->SState == LPC546XX_I2C_S_READ && data != NULL &&
					pDev->SCount < i2c->RRDataLen[pDev->SIdx])
				{
					reg->SLVDAT = data[pDev->SCount++];
				}
				else
				{
					reg->SLVDAT = LPC546XX_I2C_FILLER;
				}
				reg->SLVCTL = I2C_SLVCTL_SLVCONTINUE_MASK;
			}
			break;

		default:
			reg->SLVCTL = I2C_SLVCTL_SLVCONTINUE_MASK;
			break;
	}
}

// Flexcomm interrupt, the shared interrupt entry passes the interface
static void Lpc546xxI2CIrqHandler(int DevNo, DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	(void)DevNo;

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		Lpc546xxI2CSlvService(dev);
	}
	else if (dev->bRun)
	{
		Lpc546xxI2CMstService(dev);
	}
	else
	{
		dev->pReg->INTENCLR = LPC546XX_I2C_MST_INTS;
	}

	EnableInterrupt(state);
}

// DMA interrupt of the channel, the context is the instance
static void Lpc546xxI2CDmaIrq(int Ch, void *pCtx)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pCtx;
	uint32_t state = DisableInterrupt();

	(void)Ch;

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		// The buffer is used up, the interrupt takes the bytes past it
		if (dev->bSDmaRun &&
			(Lpc546xxDmaChFlags(Lpc546xxI2CDmaCh(dev)) & (LPC546XX_DMA_FLAG_DONE | LPC546XX_DMA_FLAG_ERR)))
		{
			Lpc546xxI2CSlvDmaEnd(dev);
		}
	}
	else
	{
		Lpc546xxI2CMstDmaDone(dev);
	}

	EnableInterrupt(state);
}

static uint32_t Lpc546xxI2CGetRate(DevIntrf_t * const pDev)
{
	return ((Lpc546xxI2CDev_t *)pDev->pDevData)->pI2cDev->Cfg.Rate;
}

// Master: closest SCL rate with the SCL low and high times of the bus mode
// of the requested rate: standard mode up to 100 kHz, fast mode up to
// 400 kHz, fast mode plus above. Slave: the master clocks the bus, the rate
// sets the divider of the data setup time of the bus mode.
static uint32_t Lpc546xxI2CSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	uint32_t fclk = dev->FClk;

	if (Rate == 0 || fclk == 0)
	{
		return 0;
	}

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		uint32_t tsu = I2C_TSUDAT_FASTMODEPLUS_MIN;

		if (Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U)
		{
			tsu = I2C_TSUDAT_STDMODE_MIN;
		}
		else if (Rate <= I2C_SCL_FAST_MODE_MAX_SPEED * 1000U)
		{
			tsu = I2C_TSUDAT_FASTMODE_MIN;
		}

		uint32_t div = (uint32_t)(((uint64_t)fclk * tsu) / 1000000000U);

		if (div >= LPC546XX_I2C_DIV_MAX)
		{
			div = LPC546XX_I2C_DIV_MAX - 1U;
		}
		dev->pReg->CLKDIV = I2C_CLKDIV_DIVVAL(div);
		dev->pI2cDev->Cfg.Rate = Rate;

		return Rate;
	}

	uint32_t tlow = I2C_SCL_TLOW_FAST_MODE_PLUS_MIN;
	uint32_t thigh = I2C_SCL_THIGH_FAST_MODE_PLUS_MIN;

	if (Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U)
	{
		tlow = I2C_SCL_TLOW_STD_MODE_MIN;
		thigh = I2C_SCL_THIGH_STD_MODE_MIN;
	}
	else if (Rate <= I2C_SCL_FAST_MODE_MAX_SPEED * 1000U)
	{
		tlow = I2C_SCL_TLOW_FAST_MODE_MIN;
		thigh = I2C_SCL_THIGH_FAST_MODE_MIN;
	}

	uint32_t bestdiv = 0;
	uint32_t bestlow = LPC546XX_I2C_SCL_CLK_MAX;
	uint32_t besthigh = LPC546XX_I2C_SCL_CLK_MAX;
	uint32_t bestrate = 0;
	uint32_t besterr = UINT32_MAX;

	// SCL period is 4 to 18 function clocks after the divider
	uint64_t divmin = (uint64_t)fclk / ((uint64_t)Rate * 2U * LPC546XX_I2C_SCL_CLK_MAX);
	uint64_t divmax = (uint64_t)fclk / ((uint64_t)Rate * 2U * LPC546XX_I2C_SCL_CLK_MIN) + 1U;

	if (divmin < 1U)
	{
		divmin = 1U;
	}
	if (divmax > LPC546XX_I2C_DIV_MAX)
	{
		divmax = LPC546XX_I2C_DIV_MAX;
	}

	for (uint32_t div = (uint32_t)divmin; div <= (uint32_t)divmax; div++)
	{
		uint32_t f = fclk / div;
		uint32_t clocks = (f + Rate / 2U) / Rate;
		uint32_t lmin = (uint32_t)(((uint64_t)tlow * f + 999999999U) / 1000000000U);
		uint32_t hmin = (uint32_t)(((uint64_t)thigh * f + 999999999U) / 1000000000U);

		if (lmin < LPC546XX_I2C_SCL_CLK_MIN)
		{
			lmin = LPC546XX_I2C_SCL_CLK_MIN;
		}
		if (hmin < LPC546XX_I2C_SCL_CLK_MIN)
		{
			hmin = LPC546XX_I2C_SCL_CLK_MIN;
		}
		if (lmin > LPC546XX_I2C_SCL_CLK_MAX || hmin > LPC546XX_I2C_SCL_CLK_MAX)
		{
			continue;
		}
		if (clocks < lmin + hmin)
		{
			clocks = lmin + hmin;
		}
		if (clocks > 2U * LPC546XX_I2C_SCL_CLK_MAX)
		{
			clocks = 2U * LPC546XX_I2C_SCL_CLK_MAX;
		}

		// Extra clocks go to the low time first
		uint32_t low = clocks - hmin;

		if (low > LPC546XX_I2C_SCL_CLK_MAX)
		{
			low = LPC546XX_I2C_SCL_CLK_MAX;
		}

		uint32_t high = clocks - low;
		uint32_t rate = fclk / (div * clocks);
		uint32_t err = rate > Rate ? rate - Rate : Rate - rate;

		if (err < besterr)
		{
			besterr = err;
			bestdiv = div;
			bestlow = low;
			besthigh = high;
			bestrate = rate;
		}
	}

	if (bestdiv == 0)
	{
		return 0;
	}

	dev->pReg->CLKDIV = I2C_CLKDIV_DIVVAL(bestdiv - 1U);
	dev->pReg->MSTTIME = I2C_MSTTIME_MSTSCLLOW(bestlow - LPC546XX_I2C_SCL_CLK_MIN) |
						 I2C_MSTTIME_MSTSCLHIGH(besthigh - LPC546XX_I2C_SCL_CLK_MIN);

	dev->pI2cDev->Cfg.Rate = bestrate;

	return bestrate;
}

static bool Lpc546xxI2CStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return Lpc546xxI2CStart(pDev, DevAddr, LPC546XX_I2C_READ);
}

// Polling: bytes received are acknowledged when the next one is requested.
// The last one is left for the NACK sent with StopRx or the next start.
// Interrupt or DMA: one transfer to the STOP, see Lpc546xxI2CMstRun.
static int Lpc546xxI2CRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	I2C_Type *reg = dev->pReg;
	int cnt = 0;

	if (dev->bActive == false || dev->bRun || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	// StartRx leaves the first byte in MSTDAT, the transfer ends with this
	// call
	if (pDev->bIntEn || pDev->bDma)
	{
		return Lpc546xxI2CMstRun(dev, true, NULL, pBuff, BuffLen);
	}

	while (cnt < BuffLen)
	{
		if (dev->bRxData == false)
		{
			reg->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK;
		}

		if (Lpc546xxI2CWait(dev) != LPC546XX_I2C_MST_RXRDY)
		{
			dev->bRxData = false;
			break;
		}

		pBuff[cnt++] = (uint8_t)reg->MSTDAT;
		dev->bRxData = false;
	}

	return cnt;
}

static bool Lpc546xxI2CStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return Lpc546xxI2CStart(pDev, DevAddr, 0);
}

// Returns the number of bytes acknowledged. Interrupt or DMA, without
// bNoStop: one transfer to the STOP, see Lpc546xxI2CMstRun. The data is
// copied to TxBuf when it fits, DeviceIntrfWrite gives a buffer that does not
// live past the call. A longer transmit returns once its last byte is taken.
static int Lpc546xxI2CTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	I2C_Type *reg = dev->pReg;
	int cnt = 0;

	if (dev->bActive == false || dev->bRun || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	if ((pDev->bIntEn || pDev->bDma) && pDev->bNoStop == false)
	{
		if (Lpc546xxI2CWait(dev) != LPC546XX_I2C_MST_TXRDY)
		{
			return 0;
		}

		const uint8_t *src = pData;

		if (pDev->bIntEn && DataLen <= LPC546XX_I2C_TXBUF_SIZE)
		{
			memcpy(dev->TxBuf, pData, DataLen);
			src = dev->TxBuf;
		}

		int res = Lpc546xxI2CMstRun(dev, false, src, NULL, DataLen);

		if (res < 0 && src == pData)
		{
			// The caller data is read until the last byte is taken. The bound
			// counts STAT reads from the last byte taken, as in Lpc546xxI2CWait.
			uint32_t left = LPC546XX_I2C_WAIT_MAX;
			int prior = -1;

			while (dev->bRun && (__atomic_load_n(&dev->Done, __ATOMIC_RELAXED) < DataLen || dev->bDmaRun))
			{
				int done = __atomic_load_n(&dev->Done, __ATOMIC_RELAXED);
				uint32_t stat = reg->STAT;

				if (done != prior)
				{
					prior = done;
					left = LPC546XX_I2C_WAIT_MAX;
				}
				else if (--left == 0)
				{
					uint32_t state = DisableInterrupt();

					Lpc546xxI2CMstAbort(dev, stat);
					EnableInterrupt(state);
					break;
				}
			}
		}

		return res;
	}

	int state = Lpc546xxI2CWait(dev);

	while (cnt < DataLen && state == LPC546XX_I2C_MST_TXRDY)
	{
		reg->MSTDAT = pData[cnt++];
		reg->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK;
		state = Lpc546xxI2CWait(dev);
	}

	// The last byte written was not acknowledged
	if (state != LPC546XX_I2C_MST_TXRDY && cnt > 0)
	{
		cnt--;
	}

	return cnt;
}

static void Lpc546xxI2CDisable(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->INTENCLR = LPC546XX_I2C_MST_INTS | LPC546XX_I2C_SLV_INTS;
	if (dev->bDmaClaimed)
	{
		Lpc546xxDmaChStop(Lpc546xxI2CDmaCh(dev));
	}
	dev->pReg->CFG = 0;
	dev->bActive = false;
	dev->bRxData = false;
	dev->bRun = false;
	dev->bAsync = false;
	dev->bStop = false;
	dev->bDmaRun = false;
	dev->bSDmaRun = false;
	dev->SState = LPC546XX_I2C_S_IDLE;
	dev->SCount = 0;
	Lpc546xxFlexcommClock(dev->DevNo, false);

	EnableInterrupt(state);
}

static void Lpc546xxI2CEnable(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	bool slave = dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE;
	uint32_t state = DisableInterrupt();

	Lpc546xxFlexcommClock(dev->DevNo, true);
	IOPinCfg(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
	dev->pReg->STAT = LPC546XX_I2C_STAT_ERR | I2C_STAT_SLVDESEL_MASK;
	dev->pReg->CFG = slave ? I2C_CFG_SLVEN_MASK : I2C_CFG_MSTEN_MASK;
	if (slave)
	{
		dev->pReg->INTENSET = LPC546XX_I2C_SLV_INTS;
	}

	EnableInterrupt(state);
}

static void Lpc546xxI2CPowerOff(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;

	Lpc546xxI2CDisable(pDev);
	IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
}

// Bus recovery with clock pulses on SCL, then Enable restores the pins
static void Lpc546xxI2CReset(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;

	I2CBusReset(dev->pI2cDev);
}

static void *Lpc546xxI2CGetHandle(DevIntrf_t * const pDev)
{
	return ((Lpc546xxI2CDev_t *)pDev->pDevData)->pI2cDev;
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL || pCfgData->DevNo < 0 || pCfgData->DevNo >= s_NbI2CDev ||
		pCfgData->pIOPinMap == NULL || pCfgData->NbIOPins < 2 || pCfgData->Rate == 0)
	{
		return false;
	}

	// 7 bit addresses, standard I2C
	if (pCfgData->Type != I2CTYPE_STANDARD || pCfgData->AddrType != I2CADDR_TYPE_NORMAL ||
		(pCfgData->Mode != I2CMODE_MASTER && pCfgData->Mode != I2CMODE_SLAVE))
	{
		return false;
	}

	bool slave = pCfgData->Mode == I2CMODE_SLAVE;

	if (slave)
	{
		if (pCfgData->NbSlaveAddr < 1 || pCfgData->NbSlaveAddr > LPC546XX_I2C_SLVADR_CNT)
		{
			return false;
		}
		for (int i = 0; i < pCfgData->NbSlaveAddr; i++)
		{
			if (pCfgData->SlaveAddr[i] > LPC546XX_I2C_ADDR_MAX)
			{
				return false;
			}
		}
	}

	Lpc546xxI2CDev_t *dev = &s_Lpc546xxI2CDev[pCfgData->DevNo];

	dev->pReg = s_Lpc546xxI2CReg[pCfgData->DevNo];
	dev->DevNo = pCfgData->DevNo;

	IRQn_Type irq = Lpc546xxFlexcommIrqNo(dev->DevNo);
	uint32_t state = DisableInterrupt();

	NVIC_DisableIRQ(irq);
	if (dev->bDmaClaimed)
	{
		// The channel of the earlier mode is given back, taken again below
		Lpc546xxDmaChFree(Lpc546xxI2CDmaCh(dev));
		dev->bDmaClaimed = false;
	}

	// Bus clock on, Flexcomm reset with the I2C function selected. Fails if
	// the Flexcomm is locked to the USART or SPI.
	dev->FClk = Lpc546xxFlexcommSelect(dev->DevNo, LPC546XX_FLEXCOMM_I2C);
	if (dev->FClk == 0)
	{
		EnableInterrupt(state);

		return false;
	}

	SharedIntrfSetIrqHandler(dev->DevNo, NULL, NULL);

	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));
	if (pDev->Cfg.MaxRetry <= 0)
	{
		pDev->Cfg.MaxRetry = I2C_MAX_RETRY;
	}
	if (slave)
	{
		// The master drives the slave transfers, only the interrupt can follow
		pDev->Cfg.bIntEn = true;
	}

	dev->pI2cDev = pDev;
	dev->bActive = false;
	dev->bRxData = false;
	dev->bRun = false;
	dev->bAsync = false;
	dev->bStop = false;
	dev->bDmaRun = false;
	dev->bSDmaRun = false;
	dev->SState = LPC546XX_I2C_S_IDLE;
	dev->SCount = 0;

	if (pCfgData->bDmaEn)
	{
		if (Lpc546xxDmaChClaim(Lpc546xxI2CDmaCh(dev), Lpc546xxI2CDmaIrq, dev, pCfgData->IntPrio,
							   pDev->Cfg.bIntEn) == false)
		{
			Lpc546xxFlexcommClock(dev->DevNo, false);
			EnableInterrupt(state);

			return false;
		}
		dev->bDmaClaimed = true;
	}

	pDev->DevIntrf.pDevData = dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.bIntEn = pDev->Cfg.bIntEn;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.GetHandle = Lpc546xxI2CGetHandle;
	pDev->DevIntrf.Disable = Lpc546xxI2CDisable;
	pDev->DevIntrf.Enable = Lpc546xxI2CEnable;
	pDev->DevIntrf.PowerOff = Lpc546xxI2CPowerOff;
	pDev->DevIntrf.Reset = Lpc546xxI2CReset;
	pDev->DevIntrf.GetRate = Lpc546xxI2CGetRate;
	pDev->DevIntrf.SetRate = Lpc546xxI2CSetRate;
	pDev->DevIntrf.StartRx = Lpc546xxI2CStartRx;
	pDev->DevIntrf.RxData = Lpc546xxI2CRxData;
	pDev->DevIntrf.StopRx = Lpc546xxI2CStop;
	pDev->DevIntrf.StartTx = Lpc546xxI2CStartTx;
	pDev->DevIntrf.TxData = Lpc546xxI2CTxData;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.StopTx = Lpc546xxI2CStop;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	for (int i = 0; i < pCfgData->NbSlaveAddr && i < I2C_SLAVEMODE_MAX_ADDR; i++)
	{
		pDev->pRRData[i] = NULL;
		pDev->RRDataLen[i] = 0;
		pDev->pTRBuff[i] = NULL;
		pDev->TRBuffLen[i] = 0;
	}

	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	if (Lpc546xxI2CSetRate(&pDev->DevIntrf, pCfgData->Rate) == 0)
	{
		if (dev->bDmaClaimed)
		{
			Lpc546xxDmaChFree(Lpc546xxI2CDmaCh(dev));
			dev->bDmaClaimed = false;
		}
		Lpc546xxFlexcommClock(dev->DevNo, false);
		EnableInterrupt(state);

		return false;
	}

	dev->pReg->INTENCLR = LPC546XX_I2C_MST_INTS | LPC546XX_I2C_SLV_INTS;

	if (slave)
	{
		for (int i = 0; i < LPC546XX_I2C_SLVADR_CNT; i++)
		{
			dev->pReg->SLVADR[i] = i < pCfgData->NbSlaveAddr ?
								   I2C_SLVADR_SLVADR(pCfgData->SlaveAddr[i]) : I2C_SLVADR_SADISABLE_MASK;
		}
		dev->pReg->SLVQUAL0 = 0;
	}

	if (pDev->Cfg.bIntEn)
	{
		SharedIntrfSetIrqHandler(dev->DevNo, &pDev->DevIntrf, Lpc546xxI2CIrqHandler);
		NVIC_ClearPendingIRQ(irq);
		NVIC_SetPriority(irq, pCfgData->IntPrio);
		NVIC_EnableIRQ(irq);
	}

	dev->pReg->STAT = LPC546XX_I2C_STAT_ERR | I2C_STAT_SLVDESEL_MASK;
	if (slave)
	{
		dev->pReg->CFG = I2C_CFG_SLVEN_MASK;
		dev->pReg->INTENSET = LPC546XX_I2C_SLV_INTS;
	}
	else
	{
		dev->pReg->CFG = I2C_CFG_MSTEN_MASK;
	}

	EnableInterrupt(state);

	return true;
}
