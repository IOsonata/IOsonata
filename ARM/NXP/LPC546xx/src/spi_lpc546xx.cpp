/**-------------------------------------------------------------------------
@file	spi_lpc546xx.cpp

@brief	LPC546xx Flexcomm SPI master implementation

		DevNo is the Flexcomm number, 0 to 9. The function clock is the
		12 MHz FRO, SCK is up to 12 MHz. Master mode with polling, 4 to 8
		bit frames on the standard 4 wire bus. Slave mode, other PHY,
		interrupt and DMA configurations fail SPIInit.

		Chip selects are GPIO pins, from index SPI_CS_IOPIN_IDX of the pin
		map. The DevAddr of StartRx and StartTx is the zero based chip
		select. The hardware SSEL outputs are not used.

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
#include "iopinctrl.h"
#include "coredev/spi.h"
#include "coredev/shared_intrf.h"
#include "flexcomm_lpc546xx.h"

#define LPC546XX_SPI_WAIT_MAX		1000000UL	// Status reads before a wait gives up
#define LPC546XX_SPI_DIV_MAX		0x10000U	// DIVVAL + 1
#define LPC546XX_SPI_DATASIZE_MIN	4U
#define LPC546XX_SPI_DATASIZE_MAX	8U

// Frames in flight in the FIFOs, lower than their depth so the RX FIFO
// cannot overflow
#define LPC546XX_SPI_INFLIGHT		4

// FIFOWR control bits of each frame, hardware SSEL outputs deasserted
#define LPC546XX_SPI_TXSSEL_OFF		(SPI_FIFOWR_TXSSEL0_N_MASK | SPI_FIFOWR_TXSSEL1_N_MASK | \
									 SPI_FIFOWR_TXSSEL2_N_MASK | SPI_FIFOWR_TXSSEL3_N_MASK)

#pragma pack(push, 4)

typedef struct __Lpc546xx_SPI_Dev {
	SPI_Type *pReg;					//!< Flexcomm SPI registers
	int DevNo;						//!< Flexcomm number
	uint32_t FClk;					//!< Function clock frequency
	uint32_t TxCtrl;				//!< FIFOWR control bits of each frame
	SPIDev_t *pSpiDev;				//!< Generic SPI device data
} Lpc546xxSPIDev_t;

#pragma pack(pop)

static Lpc546xxSPIDev_t s_Lpc546xxSPIDev[] = {
	{ .pReg = SPI0, .DevNo = 0, },
	{ .pReg = SPI1, .DevNo = 1, },
	{ .pReg = SPI2, .DevNo = 2, },
	{ .pReg = SPI3, .DevNo = 3, },
	{ .pReg = SPI4, .DevNo = 4, },
	{ .pReg = SPI5, .DevNo = 5, },
	{ .pReg = SPI6, .DevNo = 6, },
	{ .pReg = SPI7, .DevNo = 7, },
	{ .pReg = SPI8, .DevNo = 8, },
	{ .pReg = SPI9, .DevNo = 9, },
};

static const int s_NbSPIDev = sizeof(s_Lpc546xxSPIDev) / sizeof(Lpc546xxSPIDev_t);

static int Lpc546xxSPIXfer(Lpc546xxSPIDev_t * const pDev, const uint8_t *pTx, uint8_t *pRx, int Len);
static bool Lpc546xxSPIStart(DevIntrf_t * const pDev, uint32_t DevCs);
static void Lpc546xxSPIStop(DevIntrf_t * const pDev);

static uint32_t Lpc546xxSPIGetRate(DevIntrf_t * const pDev);
static uint32_t Lpc546xxSPISetRate(DevIntrf_t * const pDev, uint32_t Rate);
static int Lpc546xxSPIRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen);
static int Lpc546xxSPITxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen);
static void Lpc546xxSPIDisable(DevIntrf_t * const pDev);
static void Lpc546xxSPIEnable(DevIntrf_t * const pDev);
static void Lpc546xxSPIPowerOff(DevIntrf_t * const pDev);
static void Lpc546xxSPIReset(DevIntrf_t * const pDev);
static void *Lpc546xxSPIGetHandle(DevIntrf_t * const pDev);

// Polled transfer of Len frames. pTx NULL sends the dummy byte, pRx NULL
// ignores the received data. Returns the number of frames transferred.
static int Lpc546xxSPIXfer(Lpc546xxSPIDev_t * const pDev, const uint8_t *pTx, uint8_t *pRx, int Len)
{
	SPI_Type *reg = pDev->pReg;
	uint32_t ctrl = pDev->TxCtrl | (pRx == NULL ? SPI_FIFOWR_RXIGNORE_MASK : 0U);
	uint32_t dummy = pDev->pSpiDev->Cfg.DummyByte;
	int txcnt = 0;
	int rxcnt = 0;
	uint32_t wait = LPC546XX_SPI_WAIT_MAX;

	reg->FIFOCFG |= SPI_FIFOCFG_EMPTYRX_MASK;
	reg->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

	while ((pRx != NULL ? rxcnt : txcnt) < Len && wait > 0)
	{
		uint32_t stat = reg->FIFOSTAT;

		if (txcnt < Len && (stat & SPI_FIFOSTAT_TXNOTFULL_MASK) &&
			(pRx == NULL || txcnt - rxcnt < LPC546XX_SPI_INFLIGHT))
		{
			reg->FIFOWR = (pTx != NULL ? pTx[txcnt] : dummy) | ctrl;
			txcnt++;
			wait = LPC546XX_SPI_WAIT_MAX;
		}
		else if (pRx != NULL && (stat & SPI_FIFOSTAT_RXNOTEMPTY_MASK))
		{
			pRx[rxcnt++] = (uint8_t)reg->FIFORD;
			wait = LPC546XX_SPI_WAIT_MAX;
		}
		else
		{
			wait--;
		}
	}

	// TX only, the last frames must be on the bus before the chip select is
	// released
	if (pRx == NULL)
	{
		for (wait = LPC546XX_SPI_WAIT_MAX; wait > 0; wait--)
		{
			if ((reg->FIFOSTAT & SPI_FIFOSTAT_TXEMPTY_MASK) && (reg->STAT & SPI_STAT_MSTIDLE_MASK))
			{
				break;
			}
		}

		return wait > 0 ? txcnt : 0;
	}

	return rxcnt;
}

static bool Lpc546xxSPIStart(DevIntrf_t * const pDev, uint32_t DevCs)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.ChipSel == SPICSEL_MAN)
	{
		return true;
	}

	if ((int)DevCs < 0 || (int)DevCs >= spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
	{
		return false;
	}

	spi->CurDevCs = (int)DevCs;
	IOPinClear(spi->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo, spi->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);

	return true;
}

static void Lpc546xxSPIStop(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(spi->Cfg.pIOPinMap[spi->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				 spi->Cfg.pIOPinMap[spi->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

static uint32_t Lpc546xxSPIGetRate(DevIntrf_t * const pDev)
{
	return ((Lpc546xxSPIDev_t *)pDev->pDevData)->pSpiDev->Cfg.Rate;
}

// Closest SCK rate, function clock / (DIVVAL + 1)
static uint32_t Lpc546xxSPISetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;

	if (Rate == 0 || dev->FClk == 0)
	{
		return 0;
	}

	uint32_t div = (uint32_t)(((uint64_t)dev->FClk + Rate / 2U) / Rate);

	if (div < 1U)
	{
		div = 1U;
	}
	else if (div > LPC546XX_SPI_DIV_MAX)
	{
		div = LPC546XX_SPI_DIV_MAX;
	}

	dev->pReg->DIV = SPI_DIV_DIVVAL(div - 1U);
	dev->pSpiDev->Cfg.Rate = dev->FClk / div;

	return dev->pSpiDev->Cfg.Rate;
}

static int Lpc546xxSPIRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	return Lpc546xxSPIXfer((Lpc546xxSPIDev_t *)pDev->pDevData, NULL, pBuff, BuffLen);
}

static int Lpc546xxSPITxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	if (pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	return Lpc546xxSPIXfer((Lpc546xxSPIDev_t *)pDev->pDevData, pData, NULL, DataLen);
}

static void Lpc546xxSPIDisable(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;

	dev->pReg->CFG &= ~SPI_CFG_ENABLE_MASK;
	Lpc546xxFlexcommClock(dev->DevNo, false);
}

static void Lpc546xxSPIEnable(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;

	Lpc546xxFlexcommClock(dev->DevNo, true);
	IOPinCfg(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);
	dev->pReg->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
	dev->pReg->CFG |= SPI_CFG_ENABLE_MASK;
}

static void Lpc546xxSPIPowerOff(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;

	Lpc546xxSPIDisable(pDev);
	IOPinDis(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);
}

// FIFOs emptied, errors cleared. The configuration is kept.
static void Lpc546xxSPIReset(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;

	dev->pReg->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
	dev->pReg->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;
}

static void *Lpc546xxSPIGetHandle(DevIntrf_t * const pDev)
{
	return ((Lpc546xxSPIDev_t *)pDev->pDevData)->pSpiDev;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL || pCfgData->DevNo < 0 || pCfgData->DevNo >= s_NbSPIDev ||
		pCfgData->pIOPinMap == NULL || pCfgData->Rate == 0)
	{
		return false;
	}

	// Polling master on the standard 4 wire bus, 4 to 8 bit frames
	if (pCfgData->Mode != SPIMODE_MASTER || pCfgData->Phy != SPIPHY_NORMAL ||
		pCfgData->bDmaEn || pCfgData->bIntEn ||
		pCfgData->DataSize < LPC546XX_SPI_DATASIZE_MIN || pCfgData->DataSize > LPC546XX_SPI_DATASIZE_MAX ||
		(pCfgData->ChipSel != SPICSEL_AUTO && pCfgData->ChipSel != SPICSEL_MAN) ||
		pCfgData->NbIOPins < (pCfgData->ChipSel == SPICSEL_AUTO ? SPI_CS_IOPIN_IDX + 1 : SPI_CS_IOPIN_IDX))
	{
		return false;
	}

	Lpc546xxSPIDev_t *dev = &s_Lpc546xxSPIDev[pCfgData->DevNo];

	// Bus clock on, Flexcomm reset with the SPI function selected. Fails if
	// the Flexcomm is locked to the USART or I2C.
	dev->FClk = Lpc546xxFlexcommSelect(dev->DevNo, LPC546XX_FLEXCOMM_SPI);
	if (dev->FClk == 0)
	{
		return false;
	}

	NVIC_DisableIRQ(Lpc546xxFlexcommIrqNo(dev->DevNo));
	SharedIntrfSetIrqHandler(dev->DevNo, NULL, NULL);

	memcpy(&pDev->Cfg, pCfgData, sizeof(SPICfg_t));
	if (pDev->Cfg.MaxRetry <= 0)
	{
		pDev->Cfg.MaxRetry = SPI_MAX_RETRY;
	}

	dev->pSpiDev = pDev;
	dev->TxCtrl = LPC546XX_SPI_TXSSEL_OFF | SPI_FIFOWR_LEN(pCfgData->DataSize - 1U);

	pDev->FirstRdData = 0;
	pDev->CurDevCs = 0;
	pDev->DevIntrf.pDevData = dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.GetHandle = Lpc546xxSPIGetHandle;
	pDev->DevIntrf.Disable = Lpc546xxSPIDisable;
	pDev->DevIntrf.Enable = Lpc546xxSPIEnable;
	pDev->DevIntrf.PowerOff = Lpc546xxSPIPowerOff;
	pDev->DevIntrf.Reset = Lpc546xxSPIReset;
	pDev->DevIntrf.GetRate = Lpc546xxSPIGetRate;
	pDev->DevIntrf.SetRate = Lpc546xxSPISetRate;
	pDev->DevIntrf.StartRx = Lpc546xxSPIStart;
	pDev->DevIntrf.RxData = Lpc546xxSPIRxData;
	pDev->DevIntrf.StopRx = Lpc546xxSPIStop;
	pDev->DevIntrf.StartTx = Lpc546xxSPIStart;
	pDev->DevIntrf.TxData = Lpc546xxSPITxData;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.StopTx = Lpc546xxSPIStop;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	// Chip selects deasserted
	for (int i = SPI_CS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
	{
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
	}

	uint32_t cfg = SPI_CFG_MASTER_MASK;

	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cfg |= SPI_CFG_LSBF_MASK;
	}
	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cfg |= SPI_CFG_CPHA_MASK;
	}
	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
		cfg |= SPI_CFG_CPOL_MASK;
	}

	dev->pReg->CFG = cfg;
	dev->pReg->DLY = 0;
	dev->pReg->INTENCLR = 0xFFFFFFFFU;
	dev->pReg->FIFOCFG = SPI_FIFOCFG_ENABLETX_MASK | SPI_FIFOCFG_ENABLERX_MASK |
						 SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
	dev->pReg->FIFOTRIG = 0;
	dev->pReg->FIFOINTENCLR = 0xFFFFFFFFU;

	if (Lpc546xxSPISetRate(&pDev->DevIntrf, pCfgData->Rate) == 0)
	{
		Lpc546xxFlexcommClock(dev->DevNo, false);

		return false;
	}

	dev->pReg->CFG = cfg | SPI_CFG_ENABLE_MASK;

	return true;
}
