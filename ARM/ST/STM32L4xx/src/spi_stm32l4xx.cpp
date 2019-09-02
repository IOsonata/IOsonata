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
#include "diskio_flash.h"

#define STM32L4XX_SPI_MAXDEV		4

#pragma pack(push, 4)

typedef enum {
	QSPI_PHASE_IDLE,
	QSPI_PHASE_INST,
	QSPI_PHASE_DATA
} QSPI_PHASE;

typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	union {
		SPI_TypeDef	*pReg;
		QUADSPI_TypeDef	*pQReg;
	};
	QSPI_PHASE QPhase;
	int AdSize;
	uint32_t CcrReg;	// used by QuadSPI only
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

static bool STM32L4xxSPIWaitTx(STM32L4XX_SPIDEV * const pDev, uint32_t Timeout)
{
	do {
        if (pDev->pReg->SR & SPI_SR_TXE)
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
    uint16_t d = 0;

    while (BuffLen > 0)
    {
    	uint16_t x = 0;

		dev->pReg->CR1 |= SPI_CR1_BIDIOE;
        if (dev->pSpiDev->Cfg.DataSize > 8)
        {
            *(uint16_t*)&dev->pReg->DR = d;	// Dummy write
        	if (dev->pSpiDev->Cfg.Mode == SPIMODE_3WIRE)
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
        	if (dev->pSpiDev->Cfg.Mode == SPIMODE_3WIRE)
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
static void STM32L4xxSPIStopRx(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
static bool STM32L4xxSPIStartTx(DEVINTRF * const pDev, int DevCs)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

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
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

void SPIIrqHandler(int DevNo)
{
	STM32L4XX_SPIDEV *dev = &s_STM32L4xxSPIDev[DevNo];
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
		if (dev->pSpiDev->Cfg.Mode == SPIMODE_3WIRE)
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

SPIMODE STM32L4xxSPIMode(SPIDEV * const pDev, SPIMODE Mode)
{
	switch (Mode)
	{
		case SPIMODE_3WIRE:
			if (pDev->Cfg.DevNo < STM32L4XX_SPI_MAXDEV - 2)
			{
				SPI_TypeDef *reg;

				reg = s_STM32L4xxSPIDev[pDev->Cfg.DevNo].pReg;
				reg->CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;

				pDev->Cfg.Mode = SPIMODE_3WIRE;
			}
			break;
		case SPIMODE_NORMAL:
			if (pDev->Cfg.DevNo < STM32L4XX_SPI_MAXDEV - 2)
			{
				SPI_TypeDef *reg;

				reg = s_STM32L4xxSPIDev[pDev->Cfg.DevNo].pReg;
				reg->CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_SPE);
				reg->CR1 |= SPI_CR1_BIDIOE | SPI_CR1_SPE;

				pDev->Cfg.Mode = SPIMODE_NORMAL;
			}
			break;
	}

	return Mode;
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

bool STM32L4xxQSPIWaitBusy(STM32L4XX_SPIDEV *pDev, int Timeout)
{
	do {
		if ((pDev->pQReg->SR & QUADSPI_SR_BUSY) == 0)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool STM32L4xxQSPIWaitTxComplete(STM32L4XX_SPIDEV *pDev, int Timeout)
{
	do {
		if ((pDev->pQReg->SR & QUADSPI_SR_TCF))
		{
			pDev->pQReg->FCR |= QUADSPI_FCR_CTCF;
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool STM32L4xxQSPIWaitFifo(STM32L4XX_SPIDEV *pDev, int Timeout)
{
	do {
		if ((pDev->pQReg->SR & QUADSPI_SR_FLEVEL_Msk))
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

// Set data rate in bits/sec (Hz)
// return actual rate
static int STM32L4xxQSPISetRate(DEVINTRF * const pDev, int DataRate)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	uint32_t hclk = SystemHFClockGet();
	int32_t div = (hclk + (DataRate >> 1)) / DataRate - 1;

	if (div < 0)
	{
		div = 0;
	}

	dev->pQReg->CR &= ~QUADSPI_CR_PRESCALER_Msk;
	dev->pQReg->CR |= (div & 0xFF) << QUADSPI_CR_PRESCALER_Pos;
	dev->pSpiDev->Cfg.Rate = hclk / div;

	return dev->pSpiDev->Cfg.Rate;
}

void STM32L4xxQSPIDisable(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	STM32L4xxQSPIWaitBusy(dev, 100000);

    dev->pQReg->CR &= ~QUADSPI_CR_EN;
}

static void STM32L4xxQSPIEnable(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

    dev->pQReg->CR |= QUADSPI_CR_EN;
}

static void STM32L4xxQSPIPowerOff(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

}

bool STM32L4xxQSPISendCmd(DEVINTRF * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	dev->CcrReg &= ~(QUADSPI_CCR_INSTRUCTION_Msk | QUADSPI_CCR_IMODE_Msk | QUADSPI_CCR_DMODE_Msk |
					QUADSPI_CCR_FMODE_Msk | QUADSPI_CCR_ADMODE_Msk | QUADSPI_CCR_DCYC_Msk);
	dev->CcrReg |= Cmd | QUADSPI_CCR_IMODE_0;

	if (DataLen > 0)
	{
		dev->pQReg->DLR = DataLen - 1;
		switch (Cmd)
		{
			case FLASH_CMD_DREAD:
				dev->CcrReg |= QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_DMODE_1 | (DummyCycle << QUADSPI_CCR_DCYC_Pos);
				break;
			case FLASH_CMD_QREAD:
				dev->CcrReg |= QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_DMODE_Msk | (DummyCycle << QUADSPI_CCR_DCYC_Pos);
				break;
			case FLASH_CMD_2READ:
				dev->CcrReg |= QUADSPI_CCR_ADMODE_1 | QUADSPI_CCR_DMODE_1 | (DummyCycle << QUADSPI_CCR_DCYC_Pos);
				break;
			case FLASH_CMD_QWRITE:
				dev->CcrReg |= QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_DMODE_Msk;// | (DummyCycle << QUADSPI_CCR_DCYC_Pos);
				break;
			case FLASH_CMD_4READ:
				dev->CcrReg |= (DummyCycle << QUADSPI_CCR_DCYC_Pos);
			case FLASH_CMD_4WRITE:
			case FLASH_CMD_E4WRITE:
				dev->CcrReg |= QUADSPI_CCR_ADMODE_Msk | QUADSPI_CCR_DMODE_Msk;
				break;
			default:
				if (Addr != -1)
				{
					dev->CcrReg |= QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_DMODE_0;
				}
				else
				{
					dev->CcrReg |= QUADSPI_CCR_DMODE_0;
				}
		}
	}
	else
	{
		dev->pQReg->DLR = 0;
	}

	dev->pQReg->CCR = dev->CcrReg;
	if (Addr != -1)
	{
		dev->pQReg->AR = Addr;
	}

	STM32L4xxQSPIWaitFifo(dev, 100000);
	//STM32L4xxQSPIWaitTxComplete(dev, 100000);

	return true;
}

// Initial receive
static bool STM32L4xxQSPIStartRx(DEVINTRF * const pDev, int DevCs)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - QSPI_CS_IOPIN_IDX)
		return false;

	if (STM32L4xxQSPIWaitBusy(dev, 100000) == false)
		return false;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		// Handle multi-chipsel manually
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}

	dev->QPhase = QSPI_PHASE_INST;

	return true;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxQSPIRxDataDma(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;
	int cnt = 0;

	return cnt;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxQSPIRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;
	QUADSPI_TypeDef *reg = dev->pQReg;
    int cnt = 0;
    uint16_t d = 0;

	reg->DLR = BuffLen - 1;
	reg->CCR |= QUADSPI_CCR_FMODE_0;
	reg->AR = reg->AR;

    while (BuffLen > 0)
    {
    	uint16_t x = 0;

        if (dev->pSpiDev->Cfg.DataSize > 8)
        {

            STM32L4xxQSPIWaitFifo(dev, 100000);
            uint16_t x = *(uint16_t*)&reg->DR;
        	pBuff[0] = x & 0xff;
        	pBuff[1] = (x >> 8) & 0xFF;
        	BuffLen -= 2;
        	pBuff += 2;
        	cnt += 2;
        }
        else
        {
            STM32L4xxQSPIWaitFifo(dev, 100000);
			*pBuff = *(uint8_t*)&reg->DR;
        	BuffLen--;
        	pBuff++;
        	cnt++;
        }
    }

    return cnt;
}

// Stop receive
static void STM32L4xxQSPIStopRx(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + QSPI_CS_IOPIN_IDX].PortNo,
				 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
static bool STM32L4xxQSPIStartTx(DEVINTRF * const pDev, int DevCs)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - QSPI_CS_IOPIN_IDX)
		return false;

	if (STM32L4xxSPIWaitBusy(dev, 100000) == false)
		return false;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}

	dev->QPhase = QSPI_PHASE_INST;

	return true;
}

// Transmit Data only, no Start/Stop condition
static int STM32L4xxQSPITxDataDma(DEVINTRF * const pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Send Data only, no Start/Stop condition
static int STM32L4xxQSPITxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV*)pDev->pDevData;
	QUADSPI_TypeDef *reg = dev->pQReg;
    int cnt = 0;
    uint16_t d;

	reg->DLR = DataLen - 1;
	dev->CcrReg &= ~QUADSPI_CCR_FMODE_Msk;
	reg->CCR = dev->CcrReg;

    while (DataLen > 0)
    {
        if (dev->pSpiDev->Cfg.DataSize > 8)
    	{
        	*(uint16_t*)&reg->DR = ((uint16_t)pData[1] << 8) | (uint16_t)pData[0];
    		pData += 2;
            DataLen -= 2;
            cnt += 2;
    	}
        else
        {
			*(uint8_t*)&reg->DR = *pData;
			pData++;
			DataLen--;
			cnt++;
        }
    }

    return cnt;
}

// Stop transmit
static void STM32L4xxQSPIStopTx(DEVINTRF * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;
	QUADSPI_TypeDef *reg = dev->pQReg;

	STM32L4xxQSPIWaitTxComplete(dev, 100000);
//	STM32L4xxSPIWaitBusy(dev, 100000);

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

bool STM32L4xxQuadSPIInit(SPIDEV * const pDev, const SPICFG *pCfgData)
{
	QUADSPI_TypeDef *reg;
	uint32_t ctrlreg = 0;

	if (pCfgData->DevNo != (STM32L4XX_SPI_MAXDEV - 1))
	{
		return false;
	}

	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->DevIntrf.pDevData;

	// Get the correct register map
	reg = s_STM32L4xxSPIDev[pCfgData->DevNo].pQReg;

	RCC->AHB3RSTR |= RCC_AHB3RSTR_QSPIRST;
	RCC->AHB3RSTR &= ~RCC_AHB3RSTR_QSPIRST;

	msDelay(1);

	RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
	RCC->AHB3SMENR &= ~RCC_AHB3SMENR_QSPISMEN;

	reg->CR &= ~(QUADSPI_CR_FTHRES_Msk | QUADSPI_CR_SSHIFT);
	//reg->CR |= 0 << QUADSPI_CR_FTHRES_Pos | QUADSPI_CR_SSHIFT;

	if (pCfgData->Mode == SPIMODE_QUAD_DDR)
	{
		dev->CcrReg = QUADSPI_CCR_DDRM;
	}

	STM32L4xxQSPISetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_QSPI;
	pDev->DevIntrf.Disable = STM32L4xxQSPIDisable;
	pDev->DevIntrf.Enable = STM32L4xxQSPIEnable;
	pDev->DevIntrf.GetRate = STM32L4xxSPIGetRate;
	pDev->DevIntrf.SetRate = STM32L4xxQSPISetRate;
	pDev->DevIntrf.StartRx = STM32L4xxQSPIStartRx;
	pDev->DevIntrf.RxData = STM32L4xxQSPIRxData;
	pDev->DevIntrf.StopRx = STM32L4xxQSPIStopRx;
	pDev->DevIntrf.StartTx = STM32L4xxQSPIStartTx;
	pDev->DevIntrf.TxData = STM32L4xxQSPITxData;
	pDev->DevIntrf.StopTx = STM32L4xxQSPIStopTx;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.PowerOff = STM32L4xxQSPIPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	//pDev->SendCmd = STM32L4xxQSPISendCmd;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->FCR = reg->FCR;	// Clear all flags
    reg->CR |= QUADSPI_CR_EN;

	return true;
}

/**
 * @brief	Set Quad SPI Flash size
 */
void QuadSPISetMemSize(SPIDEV * const pDev, uint32_t Size)
{
	if (pDev->Cfg.DevNo == (STM32L4XX_SPI_MAXDEV - 1))
	{
		STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->DevIntrf.pDevData;

		STM32L4xxQSPIWaitBusy(dev, 100000);

		uint32_t dcr = QUADSPI->DCR & ~QUADSPI_DCR_FSIZE_Msk;

#ifdef __ICCARM__
		int32_t n = (41 - __CLZ(Size)) & 0x1F;
#else
		int32_t n = (41 - __builtin_clzl(Size)) & 0x1F;
#endif

		dcr |= (n & 0x1F) << QUADSPI_DCR_FSIZE_Pos;
		QUADSPI->DCR = dcr;

		n = (n - 1) / 8;
		if (n < 0)
			n = 0;
		dev->AdSize = n;
		dev->CcrReg &= ~QUADSPI_CCR_ADSIZE_Msk;
		dev->CcrReg |= n << QUADSPI_CCR_ADSIZE_Pos;
	}
}

bool QuadSPISendCmd(SPIDEV * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	if (pDev->Cfg.DevNo == STM32L4XX_SPI_MAXDEV - 1)
	{
		return STM32L4xxQSPISendCmd(&pDev->DevIntrf, Cmd, Addr, AddrLen, DataLen, DummyCycle);
	}

	return false;
}

SPIMODE SPISetMode(SPIDEV * const pDev, SPIMODE Mode)
{
	if (Mode != pDev->Cfg.Mode)
	{
		if (pDev->Cfg.DevNo == STM32L4XX_SPI_MAXDEV - 1)
		{
			if (Mode == SPIMODE_QUAD_DDR)
			{
				STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->DevIntrf.pDevData;
				dev->CcrReg = QUADSPI_CCR_DDRM;
			}
		}
		else
		{
			STM32L4xxSPIMode(pDev, Mode);
		}
	}

	return pDev->Cfg.Mode;
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
		// Quad SPI
		retval = STM32L4xxQuadSPIInit(pDev, pCfgData);
	}

	if (retval == false)
	{
		return false;
	}

    if (pCfgData->bIntEn && pCfgData->Type == SPITYPE_SLAVE)
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
    	    case 3:
                NVIC_ClearPendingIRQ(QUADSPI_IRQn);
                NVIC_SetPriority(QUADSPI_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(QUADSPI_IRQn);
                break;
    	}
    }

	return true;
}

void QSPIIrqHandler()
{

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
	QSPIIrqHandler();
    NVIC_ClearPendingIRQ(QUADSPI_IRQn);
}


