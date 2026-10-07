/**-------------------------------------------------------------------------
@file	spi_sam4l.cpp

@brief	SAM4L SPI master implementation.

The SAM4L has one dedicated SPI controller. This port implements the
IOsonata polling master path using software-controlled GPIO chip selects.
The hardware peripheral-select field is kept on CSR0 so every device attached
to one IOsonata SPI object uses the same configured transfer format and rate.

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
#include "component/component_spi.h"

#include "coredev/spi.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"

#define SAM4L_SPI_WAIT_COUNT	100000U
#define SAM4L_SPI_PCS0		0x0EU

typedef struct __Sam4l_Spi_Dev
{
	Spi *pReg;
	SPIDev_t *pSpiDev;
} Sam4lSpiDev_t;

static Sam4lSpiDev_t s_SpiDev = {
	.pReg = SAM4L_SPI,
	.pSpiDev = nullptr,
};

static inline void Sam4lSpiPmWrite(volatile uint32_t *pReg, uint32_t Value)
{
	const uint32_t offset = static_cast<uint32_t>(
		reinterpret_cast<uintptr_t>(pReg) - reinterpret_cast<uintptr_t>(SAM4L_PM));
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAU) | PM_UNLOCK_ADDR(offset);
	*pReg = Value;
}

static inline void Sam4lSpiClockEnable(void)
{
	Sam4lSpiPmWrite(&SAM4L_PM->PM_PBAMASK,
		SAM4L_PM->PM_PBAMASK | PM_PBAMASK_SPI);
}

static inline void Sam4lSpiClockDisable(void)
{
	Sam4lSpiPmWrite(&SAM4L_PM->PM_PBAMASK,
		SAM4L_PM->PM_PBAMASK & ~PM_PBAMASK_SPI);
}

static bool Sam4lSpiWait(Spi *pReg, uint32_t Mask)
{
	uint32_t timeout = SAM4L_SPI_WAIT_COUNT;
	do
	{
		if ((pReg->SPI_SR & Mask) == Mask)
			return true;
	} while (--timeout != 0U);
	return false;
}

static uint32_t Sam4lSpiGetRate(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	return dev != nullptr && dev->pSpiDev != nullptr ? dev->pSpiDev->Cfg.Rate : 0U;
}

static uint32_t Sam4lSpiSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	if (dev == nullptr || dev->pSpiDev == nullptr || Rate == 0U)
		return 0U;

	const uint32_t pba = SystemPeriphClockGet(0);
	if (pba == 0U)
		return 0U;

	uint32_t div = pba / Rate;
	if ((pba % Rate) != 0U)
		++div;
	if (div == 0U)
		div = 1U;
	else if (div > 255U)
		div = 255U;

	uint32_t csr = dev->pReg->SPI_CSR[0] & ~SPI_CSR_SCBR_Msk;
	csr |= SPI_CSR_SCBR(div);

	// Keep every CSR identical. Besides making all software chip-selects use
	// one format, this avoids the SAM4L SCBR=1/mixed-divider erratum.
	for (unsigned i = 0U; i < 4U; ++i)
		dev->pReg->SPI_CSR[i] = csr;

	dev->pSpiDev->Cfg.Rate = pba / div;
	return dev->pSpiDev->Cfg.Rate;
}

static void Sam4lSpiConfigure(Sam4lSpiDev_t *dev)
{
	Spi *reg = dev->pReg;
	const SPICfg_t &cfg = dev->pSpiDev->Cfg;

	reg->SPI_IDR = 0xFFFFFFFFU;
	reg->SPI_CR = SPI_CR_SPIDIS;
	reg->SPI_CR = SPI_CR_SWRST;

	// Variable peripheral select lets TDR.PCS select CSR0 for every character.
	// Physical slave selection is handled by the IOsonata GPIO CS array.
	reg->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PS |
		SPI_MR_PCS(SAM4L_SPI_PCS0);

	uint32_t csr = SPI_CSR_BITS(cfg.DataSize - 8U);
	if (cfg.ClkPol == SPICLKPOL_LOW)
		csr |= SPI_CSR_CPOL;
	// SAM4L NCPHA is the inverse of the conventional CPHA encoding:
	// NCPHA=1 captures on the leading edge (generic FIRST_CLK).
	if (cfg.DataPhase == SPIDATAPHASE_FIRST_CLK)
		csr |= SPI_CSR_NCPHA;

	for (unsigned i = 0U; i < 4U; ++i)
		reg->SPI_CSR[i] = csr;

	(void)Sam4lSpiSetRate(&dev->pSpiDev->DevIntrf, cfg.Rate);

	// Drop any stale receive characters before enabling a new session.
	while ((reg->SPI_SR & SPI_SR_RDRF) != 0U)
		(void)reg->SPI_RDR;

	reg->SPI_CR = SPI_CR_SPIEN;
}

static void Sam4lSpiDisable(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	(void)Sam4lSpiWait(dev->pReg, SPI_SR_TXEMPTY);
	dev->pReg->SPI_CR = SPI_CR_SPIDIS;
	Sam4lSpiClockDisable();
}

static void Sam4lSpiEnable(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	Sam4lSpiClockEnable();
	dev->pReg->SPI_CR = SPI_CR_SPIEN;
}

static void Sam4lSpiReset(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	Sam4lSpiClockEnable();
	Sam4lSpiConfigure(dev);
}

static void Sam4lSpiPowerOff(DevIntrf_t * const pDev)
{
	Sam4lSpiDisable(pDev);
}

static bool Sam4lSpiSelect(DevIntrf_t * const pDev, uint32_t DevCs)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	const uint32_t count = static_cast<uint32_t>(spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX);
	if (DevCs >= count)
		return false;

	spi->CurDevCs = static_cast<int>(DevCs);
	const IOPinCfg_t &pin = spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + DevCs];
	IOPinClear(pin.PortNo, pin.PinNo);
	return true;
}

static void Sam4lSpiDeselect(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	SPIDev_t *spi = dev->pSpiDev;

	(void)Sam4lSpiWait(dev->pReg, SPI_SR_TXEMPTY);
	if (spi->Cfg.ChipSel != SPICSEL_MAN && spi->CurDevCs >= 0)
	{
		const IOPinCfg_t &pin =
			spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi->CurDevCs];
		IOPinSet(pin.PortNo, pin.PinNo);
	}
}

static inline bool Sam4lSpiStartRx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	return Sam4lSpiSelect(pDev, DevCs);
}

static inline void Sam4lSpiStopRx(DevIntrf_t * const pDev)
{
	Sam4lSpiDeselect(pDev);
}

static inline bool Sam4lSpiStartTx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	return Sam4lSpiSelect(pDev, DevCs);
}

static inline void Sam4lSpiStopTx(DevIntrf_t * const pDev)
{
	Sam4lSpiDeselect(pDev);
}

static bool Sam4lSpiTransferWord(Sam4lSpiDev_t *dev, uint16_t Tx, uint16_t *pRx)
{
	Spi *reg = dev->pReg;
	if (!Sam4lSpiWait(reg, SPI_SR_TDRE))
		return false;

	reg->SPI_TDR = SPI_TDR_TD(Tx) | SPI_TDR_PCS(SAM4L_SPI_PCS0);
	if (!Sam4lSpiWait(reg, SPI_SR_RDRF))
		return false;

	const uint16_t rx = static_cast<uint16_t>(reg->SPI_RDR & SPI_RDR_RD_Msk);
	if (pRx != nullptr)
		*pRx = rx;
	return true;
}

static int Sam4lSpiTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	if (pData == nullptr || DataLen <= 0)
		return 0;

	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	const bool wide = dev->pSpiDev->Cfg.DataSize > 8U;
	int count = 0;

	while (DataLen >= (wide ? 2 : 1))
	{
		const uint16_t tx = wide ?
			static_cast<uint16_t>(pData[0] | (static_cast<uint16_t>(pData[1]) << 8U)) :
			pData[0];
		if (!Sam4lSpiTransferWord(dev, tx, nullptr))
			break;

		const int step = wide ? 2 : 1;
		pData += step;
		DataLen -= step;
		count += step;
	}
	return count;
}

static int Sam4lSpiRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == nullptr || BuffLen <= 0)
		return 0;

	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	const bool wide = dev->pSpiDev->Cfg.DataSize > 8U;
	const uint16_t dummy = wide ?
		static_cast<uint16_t>(dev->pSpiDev->Cfg.DummyByte |
			(static_cast<uint16_t>(dev->pSpiDev->Cfg.DummyByte) << 8U)) :
		dev->pSpiDev->Cfg.DummyByte;
	int count = 0;

	while (BuffLen >= (wide ? 2 : 1))
	{
		uint16_t rx;
		if (!Sam4lSpiTransferWord(dev, dummy, &rx))
			break;

		pBuff[0] = static_cast<uint8_t>(rx);
		if (wide)
			pBuff[1] = static_cast<uint8_t>(rx >> 8U);

		const int step = wide ? 2 : 1;
		pBuff += step;
		BuffLen -= step;
		count += step;
	}
	return count;
}

static void *Sam4lSpiGetHandle(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	return dev->pSpiDev;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	if (pDev == nullptr || pCfgData == nullptr ||
		pCfgData->DevNo != 0 ||
		pCfgData->Mode != SPIMODE_MASTER ||
		pCfgData->Phy != SPIPHY_NORMAL ||
		pCfgData->BitOrder != SPIDATABIT_MSB ||
		pCfgData->DataSize < 8U || pCfgData->DataSize > 16U ||
		pCfgData->Rate == 0U ||
		pCfgData->pIOPinMap == nullptr || pCfgData->NbIOPins < SPI_CS_IOPIN_IDX ||
		pCfgData->bDmaEn || pCfgData->bIntEn)
	{
		return false;
	}
	if (pCfgData->ChipSel != SPICSEL_MAN &&
		pCfgData->NbIOPins <= SPI_CS_IOPIN_IDX)
	{
		return false;
	}

	pDev->Cfg = *pCfgData;
	pDev->CurDevCs = -1;
	pDev->FirstRdData = -1;
	s_SpiDev.pSpiDev = pDev;
	pDev->DevIntrf.pDevData = &s_SpiDev;

	Sam4lSpiClockEnable();
	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);
	if (pDev->Cfg.ChipSel != SPICSEL_MAN)
	{
		for (int i = SPI_CS_IOPIN_IDX; i < pDev->Cfg.NbIOPins; ++i)
			IOPinSet(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo);
	}

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Disable = Sam4lSpiDisable;
	pDev->DevIntrf.Enable = Sam4lSpiEnable;
	pDev->DevIntrf.GetRate = Sam4lSpiGetRate;
	pDev->DevIntrf.SetRate = Sam4lSpiSetRate;
	pDev->DevIntrf.StartRx = Sam4lSpiStartRx;
	pDev->DevIntrf.RxData = Sam4lSpiRxData;
	pDev->DevIntrf.StopRx = Sam4lSpiStopRx;
	pDev->DevIntrf.StartTx = Sam4lSpiStartTx;
	pDev->DevIntrf.TxData = Sam4lSpiTxData;
	pDev->DevIntrf.TxSrData = Sam4lSpiTxData;
	pDev->DevIntrf.StopTx = Sam4lSpiStopTx;
	pDev->DevIntrf.Reset = Sam4lSpiReset;
	pDev->DevIntrf.PowerOff = Sam4lSpiPowerOff;
	pDev->DevIntrf.GetHandle = Sam4lSpiGetHandle;
	pDev->DevIntrf.IntPrio = pDev->Cfg.IntPrio;
	pDev->DevIntrf.EvtCB = pDev->Cfg.EvtCB;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	Sam4lSpiConfigure(&s_SpiDev);
	return pDev->Cfg.Rate != 0U;
}
