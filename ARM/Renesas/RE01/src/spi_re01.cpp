/**-------------------------------------------------------------------------
@file	spi_re01.cpp
@brief	RE01 1500KB SPI0/1 polling master implementation.

Uses the existing SPIDev_t/DevIntrf_t C interface and SPI C++ wrapper.
Supports normal SPI, 8-16 bit frames, all four clock modes, MSB/LSB first,
and GPIO or application controlled chip selects. Register encodings and
clocking follow Renesas re-driver-package, r_spi_cmsis_api.c.

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
#include "re01xxx.h"
#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"

typedef struct {
	SPI0_Type *pReg;
	SPIDev_t *pSpiDev;
	uint32_t StopMask;
	bool bEnabled;
	bool bSelected;
	bool bError;
} Re01SpiDev_t;

static Re01SpiDev_t s_Re01SpiDev[] = {
	{SPI0, NULL, MSTP_MSTPCRB_MSTPB19_Msk, false, false, false},
	{(SPI0_Type*)SPI1, NULL, MSTP_MSTPCRB_MSTPB18_Msk, false, false, false},
};

#define RE01_SPI_ERRORS	(SPI0_SPSR_OVRF_Msk | SPI0_SPSR_MODF_Msk | \
						 SPI0_SPSR_PERF_Msk | SPI0_SPSR_UDRF_Msk)

static Re01SpiDev_t *Re01SpiHandle(DevIntrf_t *pIntrf)
{
	Re01SpiDev_t *dev = pIntrf ? (Re01SpiDev_t*)pIntrf->pDevData : NULL;
	return dev && dev->pSpiDev && &dev->pSpiDev->DevIntrf == pIntrf ? dev : NULL;
}

static uint32_t Re01SpiTimeout(Re01SpiDev_t *dev)
{
	// At least 50 ms plus a complete frame, even at the slowest divisor.
	return SystemCoreClock / 20U +
		   (SystemCoreClock / dev->pSpiDev->Cfg.Rate) * 16U + 256U;
}

static bool Re01SpiWait(Re01SpiDev_t *dev, uint8_t Mask, uint8_t Value)
{
	uint32_t n = Re01SpiTimeout(dev);
	do {
		uint8_t status = dev->pReg->SPSR;
		if (status & RE01_SPI_ERRORS)
			break;
		if ((status & Mask) == Value)
			return true;
	} while (n--);
	dev->bError = true;
	return false;
}

static bool Re01SpiRate(uint32_t Rate, uint8_t *pSpbr, uint16_t *pBrdv,
						uint32_t *pActual)
{
	uint32_t clk = SystemPeriphClockGet(0); // ICLK/PCLKA, not PCLKB.
	if (Rate == 0 || clk == 0)
		return false;
	// fRSPCK = PCLKA / (2 * (SPBR + 1) * 2^BRDV).
	for (unsigned div = 0; div < 4; div++)
	{
		uint64_t denom = (uint64_t)Rate * (2U << div);
		uint32_t count = (uint32_t)(((uint64_t)clk + denom - 1) / denom);
		if (count == 0)
			count = 1;
		if (count <= 256)
		{
			*pSpbr = (uint8_t)(count - 1);
			*pBrdv = (uint16_t)(div << SPI0_SPCMD0_BRDV_Pos);
			*pActual = clk / ((2U << div) * count);
			return *pActual != 0;
		}
	}
	return false;
}

static uint32_t Re01SpiGetRate(DevIntrf_t *pIntrf)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	return dev ? dev->pSpiDev->Cfg.Rate : 0;
}

static uint32_t Re01SpiSetRate(DevIntrf_t *pIntrf, uint32_t Rate)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	uint8_t spbr;
	uint16_t brdv;
	uint32_t actual;
	if (!dev || dev->bSelected || !Re01SpiRate(Rate, &spbr, &brdv, &actual))
		return 0;
	uint8_t control = dev->pReg->SPCR;
	dev->pReg->SPCR = control & ~SPI0_SPCR_SPE_Msk;
	dev->pReg->SPBR = spbr;
	dev->pReg->SPCMD0 = (dev->pReg->SPCMD0 & ~SPI0_SPCMD0_BRDV_Msk) | brdv;
	dev->pReg->SPCR = control;
	dev->pSpiDev->Cfg.Rate = actual;
	return actual;
}

static void Re01SpiClear(Re01SpiDev_t *dev)
{
	// SPE=0 resets the transfer engine. Clear status after reading it,
	// and drain the receive register with the configured access width.
	dev->pReg->SPCR &= ~SPI0_SPCR_SPE_Msk;
	(void)dev->pReg->SPSR;
	dev->pReg->SPSR = 0xA0;
	if (dev->pSpiDev->Cfg.DataSize == 8)
		(void)dev->pReg->SPDR_HH;
	else
		(void)dev->pReg->SPDR_HA;
	if (dev->bEnabled)
		dev->pReg->SPCR |= SPI0_SPCR_SPE_Msk;
}

static void Re01SpiStop(DevIntrf_t *pIntrf)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	if (!dev || !dev->bSelected)
		return;
	if (dev->bError || !Re01SpiWait(dev, SPI0_SPSR_IDLNF_Msk, 0))
		Re01SpiClear(dev);
	SPIDev_t *spi = dev->pSpiDev;
	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi->CurDevCs];
		IOPinSet(pin->PortNo, pin->PinNo);
	}
	dev->bSelected = false;
	dev->bError = false;
	pIntrf->bTxReady = true;
}

static bool Re01SpiStart(DevIntrf_t *pIntrf, uint32_t DevCs)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	if (!dev || !dev->bEnabled)
		return false;
	SPIDev_t *spi = dev->pSpiDev;
	if (spi->Cfg.ChipSel == SPICSEL_AUTO &&
		DevCs >= (uint32_t)(spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX))
	{
		dev->bError = dev->bSelected;
		return false;
	}
	if (dev->bSelected)
	{
		// DeviceIntrfRead changes direction without releasing the transfer.
		if (spi->Cfg.ChipSel == SPICSEL_AUTO && DevCs != (uint32_t)spi->CurDevCs)
			dev->bError = true;
		return !dev->bError;
	}
	dev->bError = false;
	if (!Re01SpiWait(dev, SPI0_SPSR_IDLNF_Msk, 0))
	{
		Re01SpiClear(dev);
		dev->bError = false;
		return false;
	}
	spi->CurDevCs = (int)DevCs;
	spi->FirstRdData = -1;
	dev->bSelected = true;
	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + DevCs];
		IOPinClear(pin->PortNo, pin->PinNo);
	}
	return true;
}

static int Re01SpiTransfer(DevIntrf_t *pIntrf, const uint8_t *pTx,
						 uint8_t *pRx, int Len)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	if (!dev || !dev->bSelected || dev->bError || Len <= 0)
		return 0;
	SPIDev_t *spi = dev->pSpiDev;
	int bytes = spi->Cfg.DataSize == 8 ? 1 : 2;
	if (Len % bytes)
	{
		dev->bError = true;
		return 0;
	}
	int count = 0;
	uint16_t mask = (uint16_t)((1UL << spi->Cfg.DataSize) - 1);
	pIntrf->bTxReady = false;
	while (count < Len)
	{
		uint16_t data = spi->Cfg.DummyByte;
		if (bytes == 2)
			data |= data << 8;
		if (pTx)
		{
			data = pTx[count];
			if (bytes == 2)
				data |= (uint16_t)pTx[count + 1] << 8;
		}
		if (!Re01SpiWait(dev, SPI0_SPSR_SPTEF_Msk, SPI0_SPSR_SPTEF_Msk))
			break;
		if (bytes == 1)
			dev->pReg->SPDR_HH = (uint8_t)data;
		else
			dev->pReg->SPDR_HA = data;
		if (!Re01SpiWait(dev, SPI0_SPSR_SPRF_Msk, SPI0_SPSR_SPRF_Msk))
			break;
		data = (bytes == 1 ? dev->pReg->SPDR_HH : dev->pReg->SPDR_HA) & mask;
		if (spi->FirstRdData < 0)
			spi->FirstRdData = data;
		if (pRx)
		{
			pRx[count] = (uint8_t)data;
			if (bytes == 2)
				pRx[count + 1] = (uint8_t)(data >> 8);
		}
		count += bytes;
	}
	if (!dev->bError)
		(void)Re01SpiWait(dev, SPI0_SPSR_IDLNF_Msk, 0);
	pIntrf->bTxReady = true;
	return count;
}

static int Re01SpiTx(DevIntrf_t *pIntrf, const uint8_t *pData, int Len)
{
	return pData ? Re01SpiTransfer(pIntrf, pData, NULL, Len) : 0;
}

static int Re01SpiRx(DevIntrf_t *pIntrf, uint8_t *pBuff, int Len)
{
	return pBuff ? Re01SpiTransfer(pIntrf, NULL, pBuff, Len) : 0;
}

static void Re01SpiDisable(DevIntrf_t *pIntrf)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	if (!dev)
		return;
	Re01SpiStop(pIntrf);
	dev->pReg->SPCR &= ~SPI0_SPCR_SPE_Msk;
	dev->bEnabled = false;
}

static void Re01SpiEnable(DevIntrf_t *pIntrf)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	if (!dev)
		return;
	dev->bEnabled = true;
	Re01SpiClear(dev);
}

static void Re01SpiReset(DevIntrf_t *pIntrf)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	if (dev)
	{
		Re01SpiStop(pIntrf);
		Re01SpiClear(dev);
	}
}

static void Re01SpiPowerOff(DevIntrf_t *pIntrf)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	if (!dev)
		return;
	Re01SpiDisable(pIntrf);
	for (int i = 0; i < dev->pSpiDev->Cfg.NbIOPins; i++)
	{
		const IOPinCfg_t *pin = &dev->pSpiDev->Cfg.pIOPinMap[i];
		IOPinDisable(pin->PortNo, pin->PinNo);
	}
	uint32_t state = DisableInterrupt();
	MSTP->MSTPCRB |= dev->StopMask;
	dev->pSpiDev = NULL;
	pIntrf->pDevData = NULL;
	pIntrf->EnCnt = 0;
	EnableInterrupt(state);
}

static void *Re01SpiGetHandle(DevIntrf_t *pIntrf)
{
	Re01SpiDev_t *dev = Re01SpiHandle(pIntrf);
	return dev ? dev->pSpiDev : NULL;
}

SPIPHY SPISetPhy(SPIDev_t * const pDev, SPIPHY Phy)
{
	// This target supports only the normal, separate MOSI/MISO bus.
	(void)Phy;
	return pDev ? pDev->Cfg.Phy : SPIPHY_NORMAL;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfg)
{
	if (!pDev || !pCfg || pCfg->DevNo < 0 || pCfg->DevNo >= 2 ||
		pCfg->Mode != SPIMODE_MASTER || pCfg->Phy != SPIPHY_NORMAL ||
		pCfg->bIntEn || pCfg->bDmaEn || pCfg->DataSize < 8 || pCfg->DataSize > 16 ||
		(pCfg->ChipSel != SPICSEL_AUTO && pCfg->ChipSel != SPICSEL_MAN) ||
		(pCfg->BitOrder != SPIDATABIT_MSB && pCfg->BitOrder != SPIDATABIT_LSB) ||
		(pCfg->ClkPol != SPICLKPOL_HIGH && pCfg->ClkPol != SPICLKPOL_LOW) ||
		(pCfg->DataPhase != SPIDATAPHASE_FIRST_CLK &&
		 pCfg->DataPhase != SPIDATAPHASE_SECOND_CLK) ||
		!pCfg->pIOPinMap || pCfg->NbIOPins < 3 || pCfg->MaxRetry < 0 ||
		(pCfg->ChipSel == SPICSEL_AUTO && pCfg->NbIOPins < 4))
		return false;
	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		const IOPinCfg_t *pin = &pCfg->pIOPinMap[i];
		if (pin->PortNo < 0 || pin->PortNo >= 9 || pin->PinNo < 0 || pin->PinNo >= 16 ||
			(i < 3 && (pin->PinOp <= 0 || pin->PinOp >= IOPINOP_FUNC31)) ||
			(i >= 3 && (pin->PinOp != IOPINOP_GPIO || pin->PinDir != IOPINDIR_OUTPUT)))
			return false;
		for (int j = 0; j < i; j++)
			if (pin->PortNo == pCfg->pIOPinMap[j].PortNo &&
				pin->PinNo == pCfg->pIOPinMap[j].PinNo)
				return false;
	}
	uint8_t spbr;
	uint16_t brdv;
	uint32_t actual;
	if (!Re01SpiRate(pCfg->Rate, &spbr, &brdv, &actual))
		return false;
	uint32_t state = DisableInterrupt();
	Re01SpiDev_t *dev = &s_Re01SpiDev[pCfg->DevNo];
	if ((dev->pSpiDev && dev->pSpiDev != pDev) || dev->bSelected ||
		(s_Re01SpiDev[1 - pCfg->DevNo].pSpiDev == pDev))
	{
		EnableInterrupt(state);
		return false;
	}
	dev->pSpiDev = pDev;
	dev->bEnabled = true;
	dev->bSelected = dev->bError = false;
	MSTP->MSTPCRB &= ~dev->StopMask;
	(void)MSTP->MSTPCRB;
	EnableInterrupt(state);
	pDev->Cfg = *pCfg;
	pDev->Cfg.Rate = actual;
	pDev->FirstRdData = -1;
	pDev->CurDevCs = 0;
	for (int i = SPI_CS_IOPIN_IDX; i < pCfg->NbIOPins; i++)
		IOPinSet(pCfg->pIOPinMap[i].PortNo, pCfg->pIOPinMap[i].PinNo);
	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	SPI0_Type *reg = dev->pReg;
	reg->SPCR = SPI0_SPCR_MSTR_Msk | SPI0_SPCR_SPMS_Msk;
	reg->SSLP = 0;
	reg->SPPCR = 0;
	reg->SPSCR = 0;
	reg->SPCR2 = 0;
	reg->SPCKD = 0;
	reg->SSLND = 0;
	reg->SPND = 0;
	reg->SPBR = spbr;
	reg->SPDCR = pCfg->DataSize == 8 ? SPI0_SPDCR_SPBYT_Msk : 0;
	uint16_t cmd = brdv | (uint16_t)((pCfg->DataSize == 8 ? 4 : pCfg->DataSize - 1)
									 << SPI0_SPCMD0_SPB_Pos);
	if (pCfg->BitOrder == SPIDATABIT_LSB)
		cmd |= SPI0_SPCMD0_LSBF_Msk;
	if (pCfg->ClkPol == SPICLKPOL_LOW)
		cmd |= SPI0_SPCMD0_CPOL_Msk;
	if (pCfg->DataPhase == SPIDATAPHASE_SECOND_CLK)
		cmd |= SPI0_SPCMD0_CPHA_Msk;
	reg->SPCMD0 = cmd;
	DevIntrf_t *intrf = &pDev->DevIntrf;
	intrf->pDevData = dev;
	intrf->Type = DEVINTRF_TYPE_SPI;
	intrf->EnCnt = 1;
	intrf->bDma = intrf->bIntEn = false;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = Re01SpiEnable;
	intrf->Disable = Re01SpiDisable;
	intrf->PowerOff = Re01SpiPowerOff;
	intrf->Reset = Re01SpiReset;
	intrf->GetHandle = Re01SpiGetHandle;
	intrf->GetRate = Re01SpiGetRate;
	intrf->SetRate = Re01SpiSetRate;
	intrf->StartRx = intrf->StartTx = Re01SpiStart;
	intrf->StopRx = intrf->StopTx = Re01SpiStop;
	intrf->RxData = Re01SpiRx;
	intrf->TxData = intrf->TxSrData = Re01SpiTx;
	atomic_flag_clear(&intrf->bBusy);
	Re01SpiClear(dev);
	return true;
}
