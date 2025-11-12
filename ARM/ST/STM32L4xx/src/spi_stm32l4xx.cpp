/**-------------------------------------------------------------------------
@file	spi_stm32l4xx.cpp

@brief	SPI implementation on STM32L4xx series MCU

@author	Hoang Nguyen Hoan
@date	July 26, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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
#include "stm32l4xx.h"

#include "istddef.h"
#include "spi_stm32l4xx.h"
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"
#include "storage/diskio_flash.h"

#ifdef STM32L4S9xx
bool STM32L4xxOctoSPIInit(SPIDev_t * const pDev, const SPICFG *pCfgData);
#else
bool STM32L4xxQuadSPIInit(SPIDev_t * const pDev, const SPICFG *pCfgData);
#endif

STM32L4XX_SPIDev_t s_STM32L4xxSPIDev[STM32L4XX_SPI_MAXDEV] = {
	{
		.DevNo = 0, .pSpiDev = NULL, .pReg = SPI1
	},
	{
		.DevNo = 1, .pSpiDev = NULL, .pReg = SPI2
	},
	{
		.DevNo = 2, .pSpiDev = NULL, .pReg = SPI3
	},
#ifdef STM32L4S9xx
	{
		.DevNo = 3, .pSpiDev = NULL, .pOReg = OCTOSPI1
	},
	{
		.DevNo = 4, .pSpiDev = NULL, .pOReg = OCTOSPI2
	}
#else
	{
		.DevNo = 3, .pSpiDev = NULL, .pQReg = QUADSPI
	}
#endif
};

const int g_NbSTM32L4xxSPIDev = sizeof(s_STM32L4xxSPIDev) / sizeof(STM32L4XX_SPIDev_t);

static bool STM32L4xxSPIWaitRxReady(STM32L4XX_SPIDev_t * const pDev, uint32_t Timeout)
{
	do {
        if (pDev->pReg->SR & SPI_SR_RXNE)
        {
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}

static bool STM32L4xxSPIWaitTxFifo(STM32L4XX_SPIDev_t * const pDev, uint32_t Timeout)
{
	do {
        if ((pDev->pReg->SR & SPI_SR_FTLVL) != SPI_SR_FTLVL)
        {
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}

static bool STM32L4xxSPIWaitTx(STM32L4XX_SPIDev_t * const pDev, uint32_t Timeout)
{
	do {
        if (pDev->pReg->SR & SPI_SR_TXE)
        {
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}

static bool STM32L4xxSPIWaitBusy(STM32L4XX_SPIDev_t * const pDev, uint32_t Timeout)
{
	do {
        if ((pDev->pReg->SR & SPI_SR_BSY) == 0)
        {
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}

uint32_t STM32L4xxSPIGetRate(DevIntrf_t * const pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((STM32L4XX_SPIDev_t*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
static uint32_t STM32L4xxSPISetRate(DevIntrf_t * const pDev, uint32_t DataRate)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev->pDevData;

	uint32_t pclk;

	if (dev->DevNo == 0)
	{
		// SPI1 is on PCLK2
		pclk = SystemPeriphClockGet(1);
	}
	else
	{
		// SPI2 & SPI3 is on PCLK1
		pclk = SystemPeriphClockGet(0);
	}

	uint32_t div = (pclk + (DataRate >> 1)) / DataRate;

	dev->pReg->CR1 &= ~SPI_CR1_BR_Msk;
	if (div < 4)
	{
		dev->pSpiDev->Cfg.Rate = pclk >> 1;
	}
	else if (div < 8)
	{
		dev->pReg->CR1 |= 1 << SPI_CR1_BR_Pos;
		dev->pSpiDev->Cfg.Rate = pclk >> 2;
	}
	else if (div < 16)
	{
		dev->pReg->CR1 |= 2 << SPI_CR1_BR_Pos;
		dev->pSpiDev->Cfg.Rate = pclk >> 3;
	}
	else if (div < 32)
	{
		dev->pReg->CR1 |= 3 << SPI_CR1_BR_Pos;
		dev->pSpiDev->Cfg.Rate = pclk >> 4;
	}
	else if (div < 64)
	{
		dev->pReg->CR1 |= 4 << SPI_CR1_BR_Pos;
		dev->pSpiDev->Cfg.Rate = pclk >> 5;
	}
	else if (div < 128)
	{
		dev->pReg->CR1 |= 5 << SPI_CR1_BR_Pos;
		dev->pSpiDev->Cfg.Rate = pclk >> 6;
	}
	else if (div < 256)
	{
		dev->pReg->CR1 |= 6 << SPI_CR1_BR_Pos;
		dev->pSpiDev->Cfg.Rate = pclk >> 7;
	}
	else
	{
		dev->pReg->CR1 |= 7 << SPI_CR1_BR_Pos;
		dev->pSpiDev->Cfg.Rate = pclk >> 8;
	}

	return dev->pSpiDev->Cfg.Rate;
}

void STM32L4xxSPIDisable(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev->pDevData;
	int32_t timout = 100000;

	while ((dev->pReg->SR & SPI_SR_FTLVL));
    while ((dev->pReg->SR & SPI_SR_BSY) && timout-- > 0);

    dev->pReg->CR1 &= ~SPI_CR1_SPE;
}

static void STM32L4xxSPIEnable(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev->pDevData;

    dev->pReg->CR1 |= SPI_CR1_SPE;
}

static void STM32L4xxSPIReset(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev->pDevData;

	if (dev->DevNo > 0)
	{
		RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN << (dev->DevNo - 1);
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST << (dev->DevNo - 1);
		msDelay(1);
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_SPI2RST << (dev->DevNo - 1));
	}
	else
	{
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
		msDelay(1);
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	}
}

static void STM32L4xxSPIPowerOff(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev->pDevData;

}

// Initial receive
static bool STM32L4xxSPIStartRx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev->pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
		return false;

	STM32L4xxSPIWaitBusy(dev, 100000);

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		// Handle multi-chipsel manually
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);
	}

	return true;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxSPIRxDataDma(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxSPIRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev-> pDevData;
    int cnt = 0;
    uint16_t d = 0;

    while (BuffLen > 0)
    {
    	uint16_t x = 0;

		dev->pReg->CR1 |= SPI_CR1_BIDIOE;
        if (dev->pSpiDev->Cfg.DataSize > 8)
        {
            *(uint16_t*)&dev->pReg->DR = d;	// Dummy write
        	if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
        	{
        		STM32L4xxSPIWaitTx(dev, 100000);
        		dev->pReg->CR1 &= ~SPI_CR1_BIDIOE;
        	}

            STM32L4xxSPIWaitRxReady(dev, 100000);
            uint16_t x = *(uint16_t*)&dev->pReg->DR;
        	pBuff[0] = x & 0xff;
        	pBuff[1] = (x >> 8) & 0xFF;
        	BuffLen -= 2;
        	pBuff += 2;
        	cnt += 2;
        }
        else
        {
            *(uint8_t*)&dev->pReg->DR = d;	// Dummy write
        	if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
        	{
        		STM32L4xxSPIWaitTx(dev, 100000);
        		dev->pReg->CR1 &= ~SPI_CR1_BIDIOE;
        	}
			STM32L4xxSPIWaitRxReady(dev, 100000);
			*pBuff = dev->pReg->DR;
        	BuffLen--;
        	pBuff++;
        	cnt++;
        }
    }

    return cnt;
}

// Stop receive
static void STM32L4xxSPIStopRx(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
static bool STM32L4xxSPIStartTx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev-> pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
		return false;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);
	}

	return true;
}

// Transmit Data only, no Start/Stop condition
static int STM32L4xxSPITxDataDma(DevIntrf_t * const pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Send Data only, no Start/Stop condition
static int STM32L4xxSPITxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t*)pDev->pDevData;
    int cnt = 0;
    uint16_t d;

    while (DataLen > 0)
    {
        if (STM32L4xxSPIWaitTxFifo(dev, 100000) == false)
        {
            break;
        }

        if (dev->pSpiDev->Cfg.DataSize > 8)
    	{
        	*(uint16_t*)&dev->pReg->DR = ((uint16_t)pData[1] << 8) | (uint16_t)pData[0];
    		pData += 2;
            DataLen -= 2;
            cnt += 2;
            STM32L4xxSPIWaitRxReady(dev, 100000);
            d = dev->pReg->DR;
    	}
        else
        {
			*(uint8_t*)&dev->pReg->DR = *pData;
			pData++;
			DataLen--;
			cnt++;
	        STM32L4xxSPIWaitRxReady(dev, 100000);
			d = dev->pReg->DR;
        }
    }

    return cnt;
}

// Stop transmit
static void STM32L4xxSPIStopTx(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev-> pDevData;

	STM32L4xxSPIWaitBusy(dev, 100000);

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

void SPIIrqHandler(int DevNo)
{
	STM32L4XX_SPIDev_t *dev = &s_STM32L4xxSPIDev[DevNo];
	uint32_t flag = dev->pReg->SR;
	uint16_t d;

	if (dev->pReg->SR & SPI_SR_RXNE)
	{
		d = dev->pReg->DR;
		if (dev->pSpiDev->Cfg.EvtCB)
		{
			dev->pSpiDev->Cfg.EvtCB(&dev->pSpiDev->DevIntrf, DEVINTRF_EVT_RX_DATA, (uint8_t*)&d, 1);
		}
	}
	if (dev->pReg->SR & SPI_SR_TXE)
	{
		if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
		{
			dev->pReg->CR1 &= ~SPI_CR1_BIDIOE;
		}
		if (dev->pSpiDev->Cfg.EvtCB)
		{
			dev->pSpiDev->Cfg.EvtCB(&dev->pSpiDev->DevIntrf, DEVINTRF_EVT_RX_DATA, (uint8_t*)&d, 1);
		}
	}
	if (dev->pReg->SR & SPI_SR_OVR)
	{

	}
	if (dev->pReg->SR & SPI_SR_FRE)
	{

	}
}

SPIPHY STM32L4xxSPIPhy(SPIDEV * const pDev, SPIPHY Phy)
{
	switch (Phy)
	{
		case SPIPHY_3WIRE:
			if (pDev->Cfg.DevNo < STM32L4XX_SPI_MAXDEV - 2)
			{
				SPI_TypeDef *reg;

				reg = s_STM32L4xxSPIDev[pDev->Cfg.DevNo].pReg;
				reg->CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;

				pDev->Cfg.Phy = SPIPHY_3WIRE;
			}
			break;
		case SPIPHY_NORMAL:
			if (pDev->Cfg.DevNo < STM32L4XX_SPI_MAXDEV - 2)
			{
				SPI_TypeDef *reg;

				reg = s_STM32L4xxSPIDev[pDev->Cfg.DevNo].pReg;
				reg->CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_SPE);
				reg->CR1 |= SPI_CR1_BIDIOE | SPI_CR1_SPE;

				pDev->Cfg.Phy = SPIPHY_NORMAL;
			}
			break;
	}

	return Phy;
}

bool STM32L4xxSPIInit(SPIDEV * const pDev, const SPICFG *pCfgData)
{
	SPI_TypeDef *reg;
	uint32_t cr1reg = 0;
	uint32_t tmp = 0;

	// Get the correct register map
	reg = s_STM32L4xxSPIDev[pCfgData->DevNo].pReg;

	if (pCfgData->DevNo > 0)
	{
		RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN << (pCfgData->DevNo - 1);
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST << (pCfgData->DevNo - 1);
		msDelay(1);
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_SPI2RST << (pCfgData->DevNo - 1));
	}
	else
	{
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
		msDelay(1);
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	}


	for (int i = SPI_CS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
	{
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
	}

	if (pCfgData->ChipSel == SPICSEL_AUTO)
	{
		if (pCfgData->NbIOPins > ((SPI_CS_IOPIN_IDX + 1)))
		{
			s_STM32L4xxSPIDev[pCfgData->DevNo].pSpiDev->Cfg.ChipSel = SPICSEL_DRIVER;
		}
		else
		{
			tmp |= SPI_CR2_SSOE | SPI_CR2_NSSP;
		}
	}
	tmp |= ((pCfgData->DataSize - 1) & 0xF) << SPI_CR2_DS_Pos;

	if (pCfgData->DataSize <= 8)
	{
		// Don't use packing for 8bits data length
		tmp |= SPI_CR2_FRXTH;
	}

	reg->CR2 = tmp;

	if (pCfgData->Mode == SPIMODE_MASTER)
	{
		cr1reg |= SPI_CR1_MSTR;
		if (s_STM32L4xxSPIDev[pCfgData->DevNo].pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
		{
			cr1reg |= SPI_CR1_SSM | SPI_CR1_SSI;
		}
	}
	else
	{
		cr1reg |= SPI_CR1_RXONLY;
	}

	if (pCfgData->ClkPol == SPICLKPOL_HIGH)
	{
		IOPinClear(0, pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}
	else
	{
		cr1reg |= SPI_CR1_CPOL;
		IOPinSet(0, pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}

	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cr1reg |= SPI_CR1_CPHA;
	}

	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cr1reg |= SPI_CR1_LSBFIRST;
	}

	if (pCfgData->Phy == SPIPHY_3WIRE)
	{
		cr1reg |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
	}
	else
	{
		cr1reg |= SPI_CR1_BIDIOE;
	}

	reg->CR1 = cr1reg;

	// Note : this function call will modify CR1 register
	STM32L4xxSPISetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Disable = STM32L4xxSPIDisable;
	pDev->DevIntrf.Enable = STM32L4xxSPIEnable;
	pDev->DevIntrf.GetRate = STM32L4xxSPIGetRate;
	pDev->DevIntrf.SetRate = STM32L4xxSPISetRate;
	pDev->DevIntrf.StartRx = STM32L4xxSPIStartRx;
	pDev->DevIntrf.RxData = STM32L4xxSPIRxData;
	pDev->DevIntrf.StopRx = STM32L4xxSPIStopRx;
	pDev->DevIntrf.StartTx = STM32L4xxSPIStartTx;
	pDev->DevIntrf.TxData = STM32L4xxSPITxData;
	pDev->DevIntrf.StopTx = STM32L4xxSPIStopTx;
	pDev->DevIntrf.Reset = STM32L4xxSPIReset;
	pDev->DevIntrf.PowerOff = STM32L4xxSPIPowerOff;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->CR1 |= SPI_CR1_SPE;

	return true;
}

SPIPHY SPISetPhy(SPIDEV * const pDev, SPIPHY Phy)
{
	if (Phy != pDev->Cfg.Phy)
	{
		if (pDev->Cfg.DevNo == STM32L4XX_SPI_MAXDEV - 1)
		{
			if (Phy == SPIPHY_QUAD_DDR)
			{
				STM32L4XX_SPIDev_t *dev = (STM32L4XX_SPIDev_t *)pDev->DevIntrf.pDevData;
#ifdef STM32L4S9xx
#else
				dev->CcrReg = QUADSPI_CCR_DDRM;
#endif
			}
		}
		else
		{
			STM32L4xxSPIPhy(pDev, Phy);
		}
	}

	return pDev->Cfg.Phy;
}

bool SPIInit(SPIDEV * const pDev, const SPICFG *pCfgData)
{
	bool retval = false;

	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->DevNo >= STM32L4XX_SPI_MAXDEV)
	{
		return false;
	}

	pDev->Cfg = *pCfgData;
	s_STM32L4xxSPIDev[pCfgData->DevNo].pSpiDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_STM32L4xxSPIDev[pCfgData->DevNo];

	// Configure I/O pins
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	for (int i = 0; i < pCfgData->NbIOPins; i++)
	{
		IOPinSetSpeed(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	if (pCfgData->DevNo < STM32L4XX_SPI_MAXDEV - 1)
	{
		// SPI only
		retval = STM32L4xxSPIInit(pDev, pCfgData);
	}
	else
	{
#ifdef STM32L4S9xx
		// Octo SPI
		retval = STM32L4xxOctoSPIInit(pDev, pCfgData);
#else
		// Quad SPI
		retval = STM32L4xxQuadSPIInit(pDev, pCfgData);
#endif
	}

	if (retval == false)
	{
		return false;
	}

    if (pCfgData->bIntEn && pCfgData->Mode == SPIMODE_SLAVE)
    {
    	SPI_TypeDef *reg;

    	reg = s_STM32L4xxSPIDev[pCfgData->DevNo].pReg;

    	reg->CR2 |= SPI_CR2_RXNEIE | SPI_CR2_ERRIE;

    	switch (pCfgData->DevNo)
    	{
    		case 0:
                NVIC_ClearPendingIRQ(SPI1_IRQn);
                NVIC_SetPriority(SPI1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPI1_IRQn);
                break;
    	    case 1:
                NVIC_ClearPendingIRQ(SPI2_IRQn);
                NVIC_SetPriority(SPI2_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPI2_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(SPI3_IRQn);
                NVIC_SetPriority(SPI3_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPI3_IRQn);
                break;
#ifdef STM32L4S9xx
    	    case 3:
                NVIC_ClearPendingIRQ(OCTOSPI1_IRQn);
                NVIC_SetPriority(OCTOSPI1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(OCTOSPI1_IRQn);
                break;
    	    case 4:
                NVIC_ClearPendingIRQ(OCTOSPI2_IRQn);
                NVIC_SetPriority(OCTOSPI2_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(OCTOSPI2_IRQn);
                break;
#else
    	    case 3:
                NVIC_ClearPendingIRQ(QUADSPI_IRQn);
                NVIC_SetPriority(QUADSPI_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(QUADSPI_IRQn);
                break;
#endif
    	}
    }

	return true;
}

extern "C" void SPI1_IRQHandler(void)
{
	SPIIrqHandler(0);
    NVIC_ClearPendingIRQ(SPI1_IRQn);
}

extern "C" void SPI2_IRQHandler(void)
{
	SPIIrqHandler(1);
    NVIC_ClearPendingIRQ(SPI2_IRQn);
}

extern "C" void SPI3_IRQHandler(void)
{
	SPIIrqHandler(2);
    NVIC_ClearPendingIRQ(SPI3_IRQn);
}



