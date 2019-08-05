/**-------------------------------------------------------------------------
@file	spi_stm32l4xx.cpp

@brief	SPI implementation on STM32L4xx series MCU

@author	Hoang Nguyen Hoan
@date	July 26, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#include "stm32l4xx.h"

#include "istddef.h"
#include "coredev/spi.h"
#include "iopinctrl.h"
#include "system_core_clock.h"
#include "idelay.h"

#define STM32L4XX_SPI_MAXDEV		4

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	union {
		SPI_TypeDef	*pReg;
		QUADSPI_TypeDef	*pQReg;
	};
} STM32L4XX_SPIDEV;
#pragma pack(pop)

static STM32L4XX_SPIDEV s_STM32L4xxSPIDev[STM32L4XX_SPI_MAXDEV] = {
	{
		0, NULL, .pReg = SPI1
	},
	{
		1, NULL, .pReg = SPI2
	},
	{
		2, NULL, .pReg = SPI3
	},
	{
		3, NULL, .pQReg = QUADSPI
	}
};

static bool STM32L4xxSPIWaitRxReady(STM32L4XX_SPIDEV * const pDev, uint32_t Timeout)
{
	do {
        if (pDev->pReg->SR & SPI_SR_RXNE)
        {
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}

static bool STM32L4xxSPIWaitTxFifo(STM32L4XX_SPIDEV * const pDev, uint32_t Timeout)
{
	do {
        if ((pDev->pReg->SR & SPI_SR_FTLVL) != SPI_SR_FTLVL)
        {
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}

static bool STM32L4xxSPIWaitBusy(STM32L4XX_SPIDEV * const pDev, uint32_t Timeout)
{
	do {
        if ((pDev->pReg->SR & SPI_SR_BSY) == 0)
        {
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}

static int STM32L4xxSPIGetRate(DEVINTRF * const pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((STM32L4XX_SPIDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
static int STM32L4xxSPISetRate(DEVINTRF * const pDev, int DataRate)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	uint32_t pclk = SystemPeriphClockGet();
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

void STM32L4xxSPIDisable(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;
	int32_t timout = 100000;

	while ((dev->pReg->SR & SPI_SR_FTLVL));
    while ((dev->pReg->SR & SPI_SR_BSY) && timout-- > 0);

    dev->pReg->CR1 &= ~SPI_CR1_SPE;
}

static void STM32L4xxSPIEnable(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

    dev->pReg->CR1 |= SPI_CR1_SPE;
}

static void STM32L4xxSPIPowerOff(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

}

// Initial receive
static bool STM32L4xxSPIStartRx(DEVINTRF * const pDev, int DevCs)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	STM32L4xxSPIWaitBusy(dev, 100000);

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		// Handle multi-chipsel manually
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PinNo);
	}

	return true;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxSPIRxDataDma(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxSPIRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;
    int cnt = 0;
    uint16_t d = 0xFFFF;

    while (BuffLen > 0)
    {
    	uint16_t x = 0;

        if (dev->pSpiDev->Cfg.DataSize > 8)
        {
            *(uint16_t*)&dev->pReg->DR = d;
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
            *(uint8_t*)&dev->pReg->DR = d;
            STM32L4xxSPIWaitRxReady(dev, 100000);
        	*pBuff = dev->pReg->DR & 0xff;
        	BuffLen--;
        	pBuff++;
        	cnt++;
        }
    }

    return cnt;
}

// Stop receive
static void STM32L4xxSPIStopRx(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PortNo,
				 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
static bool STM32L4xxSPIStartTx(DEVINTRF * const pDev, int DevCs)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PinNo);
	}

	return true;
}

// Transmit Data only, no Start/Stop condition
static int STM32L4xxSPITxDataDma(DEVINTRF * const pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Send Data only, no Start/Stop condition
static int STM32L4xxSPITxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV*)pDev->pDevData;
    int cnt = 0;
    uint16_t d;

    if (pData == NULL)
    {
        return 0;
    }

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
static void STM32L4xxSPIStopTx(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

	STM32L4xxSPIWaitBusy(dev, 100000);

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PinNo);
	}
}

void SPIIrqHandler(int DevNo)
{
	STM32L4XX_SPIDEV *dev = &s_STM32L4xxSPIDev[DevNo];
	uint32_t flag = dev->pReg->SR;

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


	if (pCfgData->ChipSel == SPICSEL_AUTO)
	{
		if (pCfgData->NbIOPins > ((SPI_SS_IOPIN_IDX + 1)))
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

	if (pCfgData->Type == SPITYPE_MASTER)
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

	if (pCfgData->Mode == SPIMODE_3WIRE)
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
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.PowerOff = STM32L4xxSPIPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->CR1 |= SPI_CR1_SPE;

	return true;
}

bool STM32L4xxQuadSPIInit(SPIDEV * const pDev, const SPICFG *pCfgData)
{
	QUADSPI_TypeDef *reg;
	uint32_t cfgreg = 0;

	if (pCfgData->DevNo != (STM32L4XX_SPI_MAXDEV -1))
	{
		return false;
	}

	// Get the correct register map
	reg = s_STM32L4xxSPIDev[pCfgData->DevNo].pQReg;

	return true;
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

	for (int i = SPI_SS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
	{
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
	}

	for (int i = 0; i < pCfgData->NbIOPins; i++)
	{
		IOPinSetSpeed(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	if (pCfgData->Mode == SPIMODE_QUAD_SDR)
	{
		retval = STM32L4xxQuadSPIInit(pDev, pCfgData);
	}
	else
	{
		retval = STM32L4xxSPIInit(pDev, pCfgData);
	}

	if (retval == false)
	{
		return false;
	}

    if (pCfgData->bIntEn)
    {
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
    	    case 3:
                NVIC_ClearPendingIRQ(QUADSPI_IRQn);
                NVIC_SetPriority(QUADSPI_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(QUADSPI_IRQn);
                break;
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

extern "C" void QUADSPI_IRQHandler(void)
{
	SPIIrqHandler(3);
    NVIC_ClearPendingIRQ(QUADSPI_IRQn);
}

