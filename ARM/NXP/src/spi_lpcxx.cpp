/*--------------------------------------------------------------------------
File   : spi_lpcxx.c

Author : Hoang Nguyen Hoan          Dec. 25, 2011

Desc   : Synchronous Serial Port(SSP) implementation on LPC series MCU

Copyright (c) 2011, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by         Date            Description
Hoan				Feb. 20, 2015	New EHAL
----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "coredev/iopincfg.h"
#include "spi_lpcxx.h"
#include "iopinctrl.h"
#include "coredev/interrupt.h"

#define LPCSSP_WAIT_COUNT 100000U
#define LPCSSP_FIFO_DEPTH 8

static uint32_t LpcSSPTimeout(SSPDEV *pDev);
static bool LpcSSPStart(DevIntrf_t *pDev, uint32_t DevCs);
static void LpcSSPStop(DevIntrf_t *pDev);
static int LpcSSPTransfer(DevIntrf_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Len);


uint32_t LpcSSPGetRate(DevIntrf_t *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((SSPDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// SCK = PCLK / (CPSDVSR * (SCR + 1)). CPSDVSR is even, 2..254,
// and SCR is 0..255. For each prescaler, only the two divisors bracketing
// the requested rate can be nearest. Zero selects the minimum rate.
uint32_t LpcSSPSetRate(DevIntrf_t *pDev, uint32_t DataRate)
{
	if (pDev == NULL || pDev->pDevData == NULL)
	{
		return 0;
	}
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;

	if (dev->pSpiDev == NULL || dev->pSspReg == NULL || dev->PClkFreq == 0)
	{
		return 0;
	}
	uint32_t bestDiv = 0;
	uint32_t bestPrescale = 2;
	uint32_t bestSerial = 1;
	uint64_t bestError = 0;

	for (uint32_t prescale = 2; prescale <= 254; prescale += 2)
	{
		uint64_t ideal = DataRate == 0 ? 256 :
			dev->PClkFreq / ((uint64_t)DataRate * prescale);

		for (uint32_t next = 0; next < 2; next++)
		{
			uint64_t serial = ideal + next;

			if (serial < 1)
			{
				serial = 1;
			}
			if (serial > 256)
			{
				serial = 256;
			}
			uint32_t div = prescale * (uint32_t)serial;
			uint64_t target = (uint64_t)DataRate * div;
			uint64_t error = target > dev->PClkFreq ?
				target - dev->PClkFreq : dev->PClkFreq - target;

			// Compare exact errors before rounding to integer Hz. Both
			// divisors are below 65536, so these products fit in 64 bits.
			if (bestDiv == 0 || error * bestDiv < bestError * div)
			{
				bestDiv = div;
				bestError = error;
				bestPrescale = prescale;
				bestSerial = (uint32_t)serial;
			}
		}
	}

	uint32_t state = DisableInterrupt();

	// Keep an in-flight frame and any queued TX frames at their old rate.
	if (dev->bSelected ||
		(dev->pSspReg->SR & (LPCSSP_SR_BSY | LPCSSP_SR_TFE)) != LPCSSP_SR_TFE)
	{
		EnableInterrupt(state);
		return dev->pSpiDev->Cfg.Rate;
	}
	uint32_t cr1 = dev->pSspReg->CR1;

	dev->pSspReg->CR1 = cr1 & ~LPCSSP_CR1_SSP_EN;
	dev->pSspReg->CPSR = bestPrescale;
	dev->pSspReg->CR0 = (dev->pSspReg->CR0 & ~LPCSSP_CR0_SCR_MASK) |
		((bestSerial - 1U) << 8);
	dev->pSspReg->CR1 = cr1;
	dev->pSpiDev->Cfg.Rate = (uint32_t)(((uint64_t)dev->PClkFreq + bestDiv / 2U) / bestDiv);
	EnableInterrupt(state);

	return dev->pSpiDev->Cfg.Rate;
}

// Include at least one CPU cycle per wire bit at the selected rate.
static uint32_t LpcSSPTimeout(SSPDEV *pDev)
{
	uint32_t rate = pDev->pSpiDev->Cfg.Rate;
	uint64_t wait = LPCSSP_WAIT_COUNT;

	if (rate != 0)
	{
		wait += ((uint64_t)SystemCoreClock * pDev->pSpiDev->Cfg.DataSize + rate - 1U) / rate;
	}
	return wait > INT32_MAX ? INT32_MAX : (uint32_t)wait;
}

static bool LpcSSPStart(DevIntrf_t *pDev, uint32_t DevCs)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;

	if (!dev->bEnabled)
	{
		return false;
	}
	if (spi->Cfg.ChipSel == SPICSEL_AUTO &&
		DevCs >= (uint32_t)(spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX))
	{
		dev->bError = dev->bSelected;
		return false;
	}
	if (dev->bSelected)
	{
		// DeviceIntrfRead calls StartRx directly after its command phase.
		// Preserve the selection, first response and any command error.
		if (spi->Cfg.ChipSel == SPICSEL_AUTO && (uint32_t)spi->CurDevCs != DevCs)
		{
			dev->bError = true;
		}
		return !dev->bError;
	}
	if (dev->bError || !LpcSSPWaitBusy(dev, LpcSSPTimeout(dev)))
	{
		pDev->Reset(pDev);
		return false;
	}
	for (int i = 0; i < LPCSSP_FIFO_DEPTH &&
		(LPCSSP_READ(dev->pSspReg, SR) & LPCSSP_SR_RNE); i++)
	{
		(void)LPCSSP_READ(dev->pSspReg, DR);
	}
	LPCSSP_WRITE(dev->pSspReg, ICR, 3);
	spi->FirstRdData = -1;
	spi->CurDevCs = (int)DevCs;
	dev->bSelected = true;
	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + DevCs];

		IOPinClear(pin->PortNo, pin->PinNo);
	}
	return true;
}

static void LpcSSPStop(DevIntrf_t *pDev)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;

	if (!dev->bSelected)
	{
		return;
	}
	if (dev->bError || !LpcSSPWaitBusy(dev, LpcSSPTimeout(dev)))
	{
		// The family reset aborts the shifter and queued frames before CS
		// is released. It also restores the configured format and rate.
		pDev->Reset(pDev);
		return;
	}
	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi->CurDevCs];

		IOPinSet(pin->PortNo, pin->PinNo);
	}
	dev->bSelected = false;
	spi->CurDevCs = -1;
	pDev->bTxReady = true;
}

static int LpcSSPTransfer(DevIntrf_t *pDev, const uint8_t *pTx, uint8_t *pRx, int Len)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;
	int step = spi->Cfg.DataSize > 8 ? 2 : 1;
	int count = 0;

	if (!dev->bEnabled || !dev->bSelected || dev->bError)
	{
		return 0;
	}
	if (Len <= 0 || Len % step != 0 || (pTx == NULL && pRx == NULL))
	{
		dev->bError = true;
		return 0;
	}
	uint32_t mask = (1U << spi->Cfg.DataSize) - 1U;
	int timeout = (int)LpcSSPTimeout(dev);

	pDev->bTxReady = false;
	while (count < Len)
	{
		uint32_t data = spi->Cfg.DummyByte;

		if (step == 2)
		{
			data |= data << 8;
		}
		if (pTx != NULL)
		{
			data = pTx[count];
			if (step == 2)
			{
				data |= (uint32_t)pTx[count + 1] << 8;
			}
		}
		if (!LpcSSPWaitTxFifo(dev, timeout))
		{
			dev->bError = true;
			break;
		}
		LPCSSP_WRITE(dev->pSspReg, DR, data & mask);
		if (!LpcSSPWaitRxFifo(dev, timeout))
		{
			dev->bError = true;
			break;
		}
		data = LPCSSP_READ(dev->pSspReg, DR) & mask;
		if (spi->FirstRdData < 0)
		{
			spi->FirstRdData = (int)data;
		}
		if (pRx != NULL)
		{
			pRx[count] = (uint8_t)data;
			if (step == 2)
			{
				pRx[count + 1] = (uint8_t)(data >> 8);
			}
		}
		count += step;
	}
	if (!dev->bError && !LpcSSPWaitBusy(dev, timeout))
	{
		dev->bError = true;
		// A received frame is not proof that SCK returned to idle.
		count = 0;
	}
	pDev->bTxReady = true;
	return count;
}

bool LpcSSPStartRx(DevIntrf_t *pDev, uint32_t DevAddr)
{
	return LpcSSPStart(pDev, DevAddr);
}

bool LpcSSPStartTx(DevIntrf_t *pDev, uint32_t DevAddr)
{
	return LpcSSPStart(pDev, DevAddr);
}

int LpcSSPRxData(DevIntrf_t *pDev, uint8_t *pBuff, int BuffLen)
{
	return LpcSSPTransfer(pDev, NULL, pBuff, BuffLen);
}

int LpcSSPTxData(DevIntrf_t *pDev, const uint8_t *pData, int DataLen)
{
	return LpcSSPTransfer(pDev, pData, NULL, DataLen);
}

void LpcSSPStopRx(DevIntrf_t *pDev)
{
	LpcSSPStop(pDev);
}

void LpcSSPStopTx(DevIntrf_t *pDev)
{
	LpcSSPStop(pDev);
}

int LpcSSPRx(DevIntrf_t *pDev, int DevAddr, uint8_t *pBuff, int BuffLen)
{
	int count = 0;

	if (LpcSSPStartRx(pDev, (uint32_t)DevAddr))
	{
		count = LpcSSPRxData(pDev, pBuff, BuffLen);
		LpcSSPStopRx(pDev);
	}
	return count;
}

int LpcSSPTx(DevIntrf_t *pDev, uint32_t DevAddr, uint8_t *pData, int DataLen)
{
	int count = 0;

	if (LpcSSPStartTx(pDev, DevAddr))
	{
		count = LpcSSPTxData(pDev, pData, DataLen);
		LpcSSPStopTx(pDev);
	}
	return count;
}

void LpcSSPDisable(DevIntrf_t *pDev)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;

	LpcSSPStop(pDev);
	dev->pSspReg->CR1 &= ~LPCSSP_CR1_SSP_EN;
	dev->bEnabled = false;
}

void LpcSSPEnable(DevIntrf_t *pDev)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;

	dev->pSspReg->CR1 |= LPCSSP_CR1_SSP_EN;
	dev->bEnabled = true;
}

void *LpcSSPGetHandle(DevIntrf_t *pDev)
{
	return ((SSPDEV *)pDev->pDevData)->pSpiDev;
}

bool LpcSSPWaitBusy(SSPDEV *pDev, int TimeoutCnt)
{
	while (TimeoutCnt-- > 0)
	{
		if ((LPCSSP_READ(pDev->pSspReg, SR) & (LPCSSP_SR_BSY | LPCSSP_SR_TFE)) == LPCSSP_SR_TFE)
		{
			return true;
		}
	}
	return false;
}

bool LpcSSPWaitRxFifo(SSPDEV *pDev, int TimeoutCnt)
{
	while (TimeoutCnt-- > 0)
	{
		if (LPCSSP_READ(pDev->pSspReg, SR) & LPCSSP_SR_RNE)
		{
			return true;
		}
	}
	return false;
}

bool LpcSSPWaitTxFifo(SSPDEV *pDev, int TimeoutCnt)
{
	while (TimeoutCnt-- > 0)
	{
		if (LPCSSP_READ(pDev->pSspReg, SR) & LPCSSP_SR_TNF)
		{
			return true;
		}
	}
	return false;
}
