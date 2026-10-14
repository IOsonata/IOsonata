/**-------------------------------------------------------------------------
@file	spi_stm32wba.cpp

@brief	STM32WBA SPI master implementation

		DevNo 0 to 2 are SPI1, SPI2 and SPI3. SPI1 is clocked from PCLK2,
		SPI2 from PCLK1 and SPI3 from PCLK7. The kernel clock selection is
		PCLK. The rate is set with the MBR divider of CFG1 (2 to 256). SetRate
		and SPIInit return and store the rate actually set, the closest the
		divider gives: a request is not rejected for being out of range.

		Master mode, full duplex, polling. Slave mode, 3 wire and other
		physical interfaces, interrupt and DMA are not implemented: SPIInit
		returns false for them. The data size is 4 to 16 bits. Up to 8 bits
		the data is one byte per item, above 8 bits it is two bytes per item,
		low byte first.

		This is the SPI v2 peripheral with CFG1 and CFG2, which has a
		transfer size register. Each TxData and RxData call is one hardware
		transfer of TSIZE items, started with CSTART, as HAL_SPI_Transmit and
		HAL_SPI_TransmitReceive do, in chunks of up to 65535 items. The chip
		select is a GPIO of the pin map from index SPI_CS_IOPIN_IDX, driven
		low by StartTx and StartRx and high by StopTx and StopRx, so it stays
		low between the command phase and the data phase of a read. The
		hardware NSS is not used. With ChipSel SPICSEL_MAN, or a pin map that
		has no chip select pin, the application drives the chip select.

		Receive clocks out the DummyByte of the configuration. Data received
		during a transmit is discarded. AFCNTR keeps the pins driven by the
		peripheral between two transfers, so SCK stays at its idle level.

		An underrun, an overrun or a mode fault ends the transfer and returns
		the number of bytes done. Waits are bounded.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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
#include <string.h>
#include <stdint.h>

#include "stm32wbaxx.h"

#include "istddef.h"
#include "coredev/spi.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"

#define WBA_SPI_MAXDEV				3

// Kernel clock of the SPI is the APB clock of the instance (CCIPR field 0)
#define WBA_SPI_CLKSEL_PCLK			0

#define WBA_SPI_TSIZE_MAX			0xFFFFU

// Data bytes the transmit FIFO holds in flight. SPI3 has 8, SPI1 and SPI2 have
// 16, the smaller value is used for all.
#define WBA_SPI_FIFO_BYTES			8

#define WBA_SPI_MBR_MAX				7

#define WBA_SPI_DATASIZE_MIN		4
#define WBA_SPI_DATASIZE_MAX		16

#define WBA_SPI_RATE_DEFAULT		1000000

// 16 bit access to the data registers
typedef volatile uint16_t __attribute__((may_alias)) WbaSpiReg16_t;

#define WBA_SPI_ERR_FLAGS			(SPI_SR_UDR | SPI_SR_OVR | SPI_SR_MODF)
#define WBA_SPI_IFCR_ALL			(SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC | SPI_IFCR_OVRC | \
									 SPI_IFCR_CRCEC | SPI_IFCR_TIFREC | SPI_IFCR_MODFC | SPI_IFCR_SUSPC)

#pragma pack(push, 4)

typedef struct {
	int DevNo;
	SPIDev_t *pSpiDev;
	SPI_TypeDef *pReg;
	bool bCsDrive;			//!< The driver drives the chip select pin
	bool bCsLow;			//!< Chip select pin driven low by StartTx or StartRx
} WbaSpiDev_t;

#pragma pack(pop)

static WbaSpiDev_t s_WbaSpiDev[WBA_SPI_MAXDEV] = {
	{ 0, NULL, SPI1, false, false },
	{ 1, NULL, SPI2, false, false },
	{ 2, NULL, SPI3, false, false },
};

static WbaSpiDev_t *WbaSpiHandle(DevIntrf_t * const pIntrf)
{
	WbaSpiDev_t *dev = pIntrf ? (WbaSpiDev_t*)pIntrf->pDevData : NULL;

	return dev && dev->pSpiDev && &dev->pSpiDev->DevIntrf == pIntrf ? dev : NULL;
}

// RCC kernel clock selection and clock enable of one instance.
// SPI1 is on APB2, SPI2 on APB1 and SPI3 on APB7.
static void WbaSpiClkEnable(int DevNo, bool bEn)
{
	switch (DevNo)
	{
		case 0:
			if (bEn)
			{
				RCC->CCIPR1 &= ~RCC_CCIPR1_SPI1SEL_Msk;
				RCC->CCIPR1 |= WBA_SPI_CLKSEL_PCLK << RCC_CCIPR1_SPI1SEL_Pos;
				RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
			}
			else
			{
				RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
			}
			break;
		case 1:
			if (bEn)
			{
				RCC->CCIPR1 &= ~RCC_CCIPR1_SPI2SEL_Msk;
				RCC->CCIPR1 |= WBA_SPI_CLKSEL_PCLK << RCC_CCIPR1_SPI2SEL_Pos;
				RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
			}
			else
			{
				RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI2EN;
			}
			break;
		case 2:
			if (bEn)
			{
				RCC->CCIPR3 &= ~RCC_CCIPR3_SPI3SEL_Msk;
				RCC->CCIPR3 |= WBA_SPI_CLKSEL_PCLK << RCC_CCIPR3_SPI3SEL_Pos;
				RCC->APB7ENR |= RCC_APB7ENR_SPI3EN;
			}
			else
			{
				RCC->APB7ENR &= ~RCC_APB7ENR_SPI3EN;
			}
			break;
	}
}

static void WbaSpiRccReset(int DevNo)
{
	switch (DevNo)
	{
		case 0:
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
			break;
		case 1:
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST;
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST;
			break;
		case 2:
			RCC->APB7RSTR |= RCC_APB7RSTR_SPI3RST;
			RCC->APB7RSTR &= ~RCC_APB7RSTR_SPI3RST;
			break;
	}
}

// Kernel clock in Hz: PCLK2 for SPI1, PCLK1 for SPI2, PCLK7 for SPI3
static uint32_t WbaSpiClock(int DevNo)
{
	static const int idx[WBA_SPI_MAXDEV] = { 1, 0, 2 };

	return SystemPeriphClockGet(idx[DevNo]);
}

// Bytes of one data item
static inline int WbaSpiItemSize(WbaSpiDev_t *pDev)
{
	return pDev->pSpiDev->Cfg.DataSize > 8 ? 2 : 1;
}

// Wait bound in loop counts without progress: the time of a data item plus
// a margin.
static uint32_t WbaSpiTimeout(WbaSpiDev_t *pDev)
{
	uint32_t rate = pDev->pSpiDev->Cfg.Rate;

	if (rate == 0)
	{
		rate = 1;
	}

	return (SystemCoreClock / rate + 1U) * (pDev->pSpiDev->Cfg.DataSize + 2U) + 256U;
}

// Close a transfer as HAL SPI_CloseTransfer does: stop the peripheral, clear
// the flags, collect the errors. Interrupts and DMA are never enabled by this
// driver. Return false when an error flag was set.
static bool WbaSpiClose(WbaSpiDev_t *pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	uint32_t sr = reg->SR;

	reg->CR1 &= ~SPI_CR1_SPE;
	reg->IFCR = WBA_SPI_IFCR_ALL;

	return (sr & WBA_SPI_ERR_FLAGS) == 0;
}

// One hardware transfer of Count items, at most WBA_SPI_TSIZE_MAX. pTx is the
// data to send, or NULL to send the dummy value. pRx is the buffer for the
// data received, or NULL to discard it. Return the number of items done.
static uint32_t WbaSpiXfer(WbaSpiDev_t *pDev, const uint8_t *pTx, uint8_t *pRx, uint32_t Count)
{
	SPI_TypeDef *reg = pDev->pReg;
	const bool b16 = pDev->pSpiDev->Cfg.DataSize > 8;
	const uint32_t fifo = WBA_SPI_FIFO_BYTES / (b16 ? 2 : 1);
	const uint32_t dummy = b16 ? ((uint32_t)pDev->pSpiDev->Cfg.DummyByte * 0x0101U) :
								 (uint32_t)pDev->pSpiDev->Cfg.DummyByte;
	const uint32_t tmo = WbaSpiTimeout(pDev);
	uint32_t txn = 0;
	uint32_t rxn = 0;
	uint32_t n = tmo;
	bool bOk = true;

	reg->CR1 &= ~SPI_CR1_SPE;
	reg->IFCR = WBA_SPI_IFCR_ALL;
	reg->CR2 = Count;
	reg->CR1 |= SPI_CR1_SPE;
	reg->CR1 |= SPI_CR1_CSTART;

	while (txn < Count || rxn < Count)
	{
		uint32_t sr = reg->SR;
		bool bProgress = false;

		if (sr & WBA_SPI_ERR_FLAGS)
		{
			bOk = false;
			break;
		}

		if ((sr & SPI_SR_TXP) && txn < Count && (txn - rxn) < fifo)
		{
			if (b16)
			{
				uint16_t d = pTx ? (uint16_t)(pTx[txn * 2] | ((uint16_t)pTx[txn * 2 + 1] << 8)) : (uint16_t)dummy;

				*(WbaSpiReg16_t*)&reg->TXDR = d;
			}
			else
			{
				*(volatile uint8_t*)&reg->TXDR = pTx ? pTx[txn] : (uint8_t)dummy;
			}
			txn++;
			bProgress = true;
		}

		if ((sr & SPI_SR_RXP) && rxn < Count)
		{
			if (b16)
			{
				uint16_t d = *(WbaSpiReg16_t*)&reg->RXDR;

				if (pRx)
				{
					pRx[rxn * 2] = (uint8_t)d;
					pRx[rxn * 2 + 1] = (uint8_t)(d >> 8);
				}
			}
			else
			{
				uint8_t d = *(volatile uint8_t*)&reg->RXDR;

				if (pRx)
				{
					pRx[rxn] = d;
				}
			}
			rxn++;
			bProgress = true;
		}

		if (bProgress)
		{
			n = tmo;
		}
		else if (n-- == 0)
		{
			bOk = false;
			break;
		}
	}

	if (bOk)
	{
		// All items are in and out, wait for the end of transfer flag
		n = tmo;
		while ((reg->SR & SPI_SR_EOT) == 0)
		{
			if (n-- == 0)
			{
				bOk = false;
				break;
			}
		}
	}

	if (WbaSpiClose(pDev) == false)
	{
		bOk = false;
	}

	return bOk ? Count : rxn;
}

// Run a transfer of Len bytes in chunks. Return the number of bytes done.
static int WbaSpiTransfer(WbaSpiDev_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Len)
{
	const int isz = WbaSpiItemSize(pDev);
	uint32_t items = (uint32_t)Len / isz;
	uint32_t done = 0;

	while (items > 0)
	{
		uint32_t chunk = items > WBA_SPI_TSIZE_MAX ? WBA_SPI_TSIZE_MAX : items;
		uint32_t d = WbaSpiXfer(pDev, pTx ? pTx + done * isz : NULL, pRx ? pRx + done * isz : NULL, chunk);

		done += d;
		if (d < chunk)
		{
			break;
		}
		items -= chunk;
	}

	return (int)(done * isz);
}

// Set the closest rate the MBR divider gives, 2 to 256. Return the rate set.
static uint32_t WbaSpiSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev == NULL)
	{
		return 0;
	}

	uint32_t clk = WbaSpiClock(dev->DevNo);

	if (Rate == 0)
	{
		Rate = WBA_SPI_RATE_DEFAULT;
	}

	// Closest rate, the lower one when two are as close
	uint32_t best = 0;
	uint32_t bestdiff = 0xFFFFFFFFU;

	for (uint32_t mbr = 0; mbr <= WBA_SPI_MBR_MAX; mbr++)
	{
		uint32_t r = clk >> (mbr + 1);
		uint32_t diff = r > Rate ? r - Rate : Rate - r;

		if (diff <= bestdiff)
		{
			bestdiff = diff;
			best = mbr;
		}
	}

	SPI_TypeDef *reg = dev->pReg;
	bool spe = (reg->CR1 & SPI_CR1_SPE) != 0;

	// MBR can only change with the peripheral disabled
	reg->CR1 &= ~SPI_CR1_SPE;
	reg->CFG1 = (reg->CFG1 & ~(SPI_CFG1_MBR_Msk | SPI_CFG1_BPASS)) | (best << SPI_CFG1_MBR_Pos);
	if (spe)
	{
		reg->CR1 |= SPI_CR1_SPE;
	}
	dev->pSpiDev->Cfg.Rate = clk >> (best + 1);

	return dev->pSpiDev->Cfg.Rate;
}

static uint32_t WbaSpiGetRate(DevIntrf_t * const pIntrf)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	return dev ? dev->pSpiDev->Cfg.Rate : 0;
}

static void WbaSpiReset(DevIntrf_t * const pIntrf)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	// SPE low resets the state machine, the configuration is kept
	dev->pReg->CR1 &= ~SPI_CR1_SPE;
	dev->pReg->IFCR = WBA_SPI_IFCR_ALL;
}

static void WbaSpiEnable(DevIntrf_t * const pIntrf)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev)
	{
		WbaSpiClkEnable(dev->DevNo, true);
	}
}

static void WbaSpiDisable(DevIntrf_t * const pIntrf)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev)
	{
		dev->pReg->CR1 &= ~SPI_CR1_SPE;
		WbaSpiClkEnable(dev->DevNo, false);
	}
}

static void WbaSpiPowerOff(DevIntrf_t * const pIntrf)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev)
	{
		dev->pReg->CR1 &= ~SPI_CR1_SPE;
		WbaSpiClkEnable(dev->DevNo, false);
		IOPinDis(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);
	}
}

static void WbaSpiStop(DevIntrf_t * const pIntrf);

// Open a transfer: select the chip. StartRx can follow StartTx for the data
// phase of a read, the chip select then stays low.
static bool WbaSpiStart(DevIntrf_t * const pIntrf, uint32_t DevCs)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev == NULL)
	{
		return false;
	}

	if (dev->bCsDrive)
	{
		const SPICfg_t *cfg = &dev->pSpiDev->Cfg;

		if (DevCs >= (uint32_t)(cfg->NbIOPins - SPI_CS_IOPIN_IDX))
		{
			return false;
		}
		if (dev->bCsLow && dev->pSpiDev->CurDevCs == (int)DevCs)
		{
			// Data phase of a read, the chip select is already low
			return true;
		}
		if (dev->bCsLow)
		{
			// Another device was still selected
			WbaSpiStop(pIntrf);
		}

		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(cfg->pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
				   cfg->pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);
		dev->bCsLow = true;
	}

	return true;
}

// Close a transfer: release the chip select
static void WbaSpiStop(DevIntrf_t * const pIntrf)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev && dev->bCsLow)
	{
		const SPICfg_t *cfg = &dev->pSpiDev->Cfg;
		int cs = dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX;

		IOPinSet(cfg->pIOPinMap[cs].PortNo, cfg->pIOPinMap[cs].PinNo);
		dev->bCsLow = false;
	}
}

static int WbaSpiTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev == NULL || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	return WbaSpiTransfer(dev, pData, NULL, DataLen);
}

static int WbaSpiRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	WbaSpiDev_t *dev = WbaSpiHandle(pIntrf);

	if (dev == NULL || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	return WbaSpiTransfer(dev, NULL, pBuff, BuffLen);
}

// Master mode, full duplex, polling. DevNo 0 to 2 are SPI1 to SPI3.
bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= WBA_SPI_MAXDEV)
	{
		return false;
	}

	if (pCfgData->Mode != SPIMODE_MASTER || pCfgData->Phy != SPIPHY_NORMAL ||
		pCfgData->bIntEn || pCfgData->bDmaEn ||
		pCfgData->DataSize < WBA_SPI_DATASIZE_MIN || pCfgData->DataSize > WBA_SPI_DATASIZE_MAX ||
		pCfgData->pIOPinMap == NULL || pCfgData->NbIOPins < SPI_CS_IOPIN_IDX)
	{
		return false;
	}

	WbaSpiDev_t *dev = &s_WbaSpiDev[pCfgData->DevNo];
	SPI_TypeDef *reg = dev->pReg;

	memcpy(&pDev->Cfg, pCfgData, sizeof(SPICfg_t));

	dev->pSpiDev = pDev;
	dev->bCsDrive = pCfgData->ChipSel != SPICSEL_MAN && pCfgData->NbIOPins > SPI_CS_IOPIN_IDX;
	dev->bCsLow = false;
	if (dev->bCsDrive)
	{
		pDev->Cfg.ChipSel = SPICSEL_DRIVER;
	}
	pDev->CurDevCs = 0;
	pDev->FirstRdData = 0;

	pDev->DevIntrf.pDevData = (void*)dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Reset = WbaSpiReset;
	pDev->DevIntrf.Disable = WbaSpiDisable;
	pDev->DevIntrf.Enable = WbaSpiEnable;
	pDev->DevIntrf.PowerOff = WbaSpiPowerOff;
	pDev->DevIntrf.GetRate = WbaSpiGetRate;
	pDev->DevIntrf.SetRate = WbaSpiSetRate;
	pDev->DevIntrf.StartRx = WbaSpiStart;
	pDev->DevIntrf.RxData = WbaSpiRxData;
	pDev->DevIntrf.StopRx = WbaSpiStop;
	pDev->DevIntrf.StartTx = WbaSpiStart;
	pDev->DevIntrf.TxData = WbaSpiTxData;
	pDev->DevIntrf.StopTx = WbaSpiStop;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	WbaSpiClkEnable(pCfgData->DevNo, true);
	WbaSpiRccReset(pCfgData->DevNo);

	// Chip selects idle high before the pins are configured
	for (int i = SPI_CS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
	{
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
	}

	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);
	for (int i = 0; i < pDev->Cfg.NbIOPins; i++)
	{
		IOPinSetSpeed(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	// The internal slave select is held inactive (SSM and SSI), so the NSS pin
	// is not used and no mode fault occurs. AFCNTR keeps the pins driven by
	// the peripheral when it is disabled.
	uint32_t cfg2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR;

	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
		cfg2 |= SPI_CFG2_CPOL;
	}
	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cfg2 |= SPI_CFG2_CPHA;
	}
	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cfg2 |= SPI_CFG2_LSBFRST;
	}

	reg->CR1 = SPI_CR1_SSI;
	reg->CR2 = 0;
	reg->IER = 0;
	reg->AUTOCR = 0;
	reg->CFG1 = (pCfgData->DataSize - 1) << SPI_CFG1_DSIZE_Pos;
	reg->CFG2 = cfg2;
	reg->IFCR = WBA_SPI_IFCR_ALL;

	WbaSpiSetRate(&pDev->DevIntrf, pCfgData->Rate);

	return true;
}
