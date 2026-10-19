/**-------------------------------------------------------------------------
@file	quadspi_stm32.cpp

@brief	STM32 QUADSPI implementation

		QUADSPI is the SPI device DevNo 3 (STM32_QOSPI_DEVNO_START), SPIInit of
		spi_stm32.cpp calls Stm32QospiInit for it. Master in polling mode,
		indirect mode, the commands of QuadSPISendCmd. The chip select is a
		GPIO of the pin map from index QSPI_CS_IOPIN_IDX when ChipSel is
		SPICSEL_DRIVER. The rate is HCLK divided by 1 to 256 (PRESCALER).

		Moved from STM32L4xx/src/quadspi_stm32l4xx.cpp.

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
#include "stm32.h"

#if defined(QUADSPI)

#include <stddef.h>

#include "istddef.h"
#include "qospi_stm32.h"
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"
#include "storage/diskio_flash.h"

#pragma pack(push, 4)

typedef struct {
	SPIDev_t *pSpiDev;
	QUADSPI_TypeDef *pReg;
	QOSPI_PHASE QOPhase;
	int AdSize;
	uint32_t CcrReg;
} Stm32QspiDev_t;

#pragma pack(pop)

static Stm32QspiDev_t s_Stm32QspiDev = {
	NULL, QUADSPI, QOSPI_PHASE_IDLE, 0, 0
};

static bool Stm32QspiIsDev(SPIDev_t * const pDev)
{
	return pDev->Cfg.DevNo == STM32_QOSPI_DEVNO_START && s_Stm32QspiDev.pSpiDev == pDev;
}

static bool Stm32QspiWaitBusy(Stm32QspiDev_t *pDev, int Timeout)
{
	do {
		if ((pDev->pReg->SR & QUADSPI_SR_BUSY) == 0)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static bool Stm32QspiWaitTxComplete(Stm32QspiDev_t *pDev, int Timeout)
{
	do {
		if ((pDev->pReg->SR & QUADSPI_SR_TCF))
		{
			pDev->pReg->FCR |= QUADSPI_FCR_CTCF;
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static bool Stm32QspiWaitFifo(Stm32QspiDev_t *pDev, int Timeout)
{
	do {
		if ((pDev->pReg->SR & QUADSPI_SR_FLEVEL_Msk))
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static uint32_t Stm32QspiGetRate(DevIntrf_t * const pDev)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	return dev->pSpiDev->Cfg.Rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
static uint32_t Stm32QspiSetRate(DevIntrf_t * const pDev, uint32_t DataRate)
{
	if (DataRate == 0)
	{
		return 0;
	}

	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;
	uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos;
	uint32_t hclk = tmp & 8 ? SystemCoreClock >> ((tmp & 7) + 1) : SystemCoreClock;

	// SCK = HCLK / (PRESCALER + 1)
	int32_t div = (hclk + (DataRate >> 1)) / DataRate - 1;

	if (div < 0)
	{
		div = 0;
	}

	if (div > 255)
	{
		div = 255;
	}

	dev->pReg->CR &= ~QUADSPI_CR_PRESCALER_Msk;
	dev->pReg->CR |= (div & 0xFF) << QUADSPI_CR_PRESCALER_Pos;
	dev->pSpiDev->Cfg.Rate = hclk / (div + 1);

	return dev->pSpiDev->Cfg.Rate;
}

static void Stm32QspiDisable(DevIntrf_t * const pDev)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	Stm32QspiWaitBusy(dev, 100000);

	dev->pReg->CR &= ~QUADSPI_CR_EN;
}

static void Stm32QspiEnable(DevIntrf_t * const pDev)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	dev->pReg->CR |= QUADSPI_CR_EN;
}

static void Stm32QspiPowerOff(DevIntrf_t * const pDev)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	Stm32QspiDisable(pDev);

	RCC->AHB3ENR &= ~RCC_AHB3ENR_QSPIEN;

	for (int i = 0; i < dev->pSpiDev->Cfg.NbIOPins; i++)
	{
		IOPinConfig(dev->pSpiDev->Cfg.pIOPinMap[i].PortNo, dev->pSpiDev->Cfg.pIOPinMap[i].PinNo,
					0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	}
}

static bool Stm32QspiSendCmd(DevIntrf_t * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	dev->CcrReg &= ~(QUADSPI_CCR_INSTRUCTION_Msk | QUADSPI_CCR_IMODE_Msk | QUADSPI_CCR_DMODE_Msk |
					QUADSPI_CCR_FMODE_Msk | QUADSPI_CCR_ADMODE_Msk | QUADSPI_CCR_DCYC_Msk);
	dev->CcrReg |= Cmd | QUADSPI_CCR_IMODE_0;

	if (DataLen > 0)
	{
		dev->pReg->DLR = DataLen - 1;
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
				dev->CcrReg |= QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_DMODE_Msk;
				break;
			case FLASH_CMD_4READ:
				dev->CcrReg |= (DummyCycle << QUADSPI_CCR_DCYC_Pos);
				// fall through
			case FLASH_CMD_4WRITE:
			case FLASH_CMD_E4WRITE:
				dev->CcrReg |= QUADSPI_CCR_ADMODE_Msk | QUADSPI_CCR_DMODE_Msk;
				break;
			default:
				if (Addr != (uint32_t)-1)
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
		if (Addr != (uint32_t)-1)
		{
			dev->CcrReg |= QUADSPI_CCR_ADMODE_0;
		}
		dev->pReg->DLR = 0;
	}

	dev->pReg->CCR = dev->CcrReg;
	if (Addr != (uint32_t)-1)
	{
		dev->pReg->AR = Addr;
	}

	Stm32QspiWaitFifo(dev, 100000);

	return true;
}

// Initiate receive or transmit, chip select low
static bool Stm32QspiStart(DevIntrf_t * const pDev, uint32_t DevCs)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	if ((int)DevCs < 0 || (int)DevCs >= dev->pSpiDev->Cfg.NbIOPins - QSPI_CS_IOPIN_IDX)
	{
		return false;
	}

	if (Stm32QspiWaitBusy(dev, 100000) == false)
	{
		return false;
	}

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		// Handle multi-chipsel manually
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}

	dev->QOPhase = QOSPI_PHASE_INST;

	return true;
}

// Chip select high
static void Stm32QspiCsHigh(Stm32QspiDev_t *dev)
{
	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + QSPI_CS_IOPIN_IDX].PortNo,
				 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}
}

// Receive Data only, no Start/Stop condition
static int Stm32QspiRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;
	QUADSPI_TypeDef *reg = dev->pReg;
	// DR accessed as a half word for data above 8 bits
	volatile uint16_t *dr16 = (volatile uint16_t *)((uintptr_t)reg + offsetof(QUADSPI_TypeDef, DR));
	int cnt = 0;

	reg->DLR = BuffLen - 1;
	reg->CCR |= QUADSPI_CCR_FMODE_0;
	reg->AR = reg->AR;

	while (BuffLen > 0)
	{
		Stm32QspiWaitFifo(dev, 100000);

		if (dev->pSpiDev->Cfg.DataSize > 8)
		{
			uint16_t x = *dr16;
			pBuff[0] = x & 0xff;
			pBuff[1] = (x >> 8) & 0xFF;
			BuffLen -= 2;
			pBuff += 2;
			cnt += 2;
		}
		else
		{
			*pBuff = *(volatile uint8_t*)&reg->DR;
			BuffLen--;
			pBuff++;
			cnt++;
		}
	}

	return cnt;
}

// Stop receive
static void Stm32QspiStopRx(DevIntrf_t * const pDev)
{
	Stm32QspiCsHigh((Stm32QspiDev_t *)pDev->pDevData);
}

// Send Data only, no Start/Stop condition
static int Stm32QspiTxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t*)pDev->pDevData;
	QUADSPI_TypeDef *reg = dev->pReg;
	// DR accessed as a half word for data above 8 bits
	volatile uint16_t *dr16 = (volatile uint16_t *)((uintptr_t)reg + offsetof(QUADSPI_TypeDef, DR));
	int cnt = 0;

	reg->DLR = DataLen - 1;
	dev->CcrReg &= ~QUADSPI_CCR_FMODE_Msk;
	reg->CCR = dev->CcrReg;

	while (DataLen > 0)
	{
		if (dev->pSpiDev->Cfg.DataSize > 8)
		{
			*dr16 = ((uint16_t)pData[1] << 8) | (uint16_t)pData[0];
			pData += 2;
			DataLen -= 2;
			cnt += 2;
		}
		else
		{
			*(volatile uint8_t*)&reg->DR = *pData;
			pData++;
			DataLen--;
			cnt++;
		}
	}

	return cnt;
}

// Stop transmit
static void Stm32QspiStopTx(DevIntrf_t * const pDev)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	Stm32QspiWaitTxComplete(dev, 100000);
	Stm32QspiCsHigh(dev);
}

bool Stm32QospiInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	if (pCfgData->DevNo != STM32_QOSPI_DEVNO_START || pCfgData->Mode != SPIMODE_MASTER ||
		pCfgData->bIntEn || pCfgData->bDmaEn)
	{
		return false;
	}

	Stm32QspiDev_t *dev = &s_Stm32QspiDev;
	QUADSPI_TypeDef *reg = dev->pReg;

	pDev->Cfg = *pCfgData;
	dev->pSpiDev = pDev;
	pDev->DevIntrf.pDevData = (void*)dev;

	// Configure I/O pins
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	for (int i = 0; i < pCfgData->NbIOPins; i++)
	{
		IOPinSetSpeed(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	RCC->AHB3RSTR |= RCC_AHB3RSTR_QSPIRST;
	RCC->AHB3RSTR &= ~RCC_AHB3RSTR_QSPIRST;

	msDelay(1);

	RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
	RCC->AHB3SMENR &= ~RCC_AHB3SMENR_QSPISMEN;

	reg->CR &= ~(QUADSPI_CR_FTHRES_Msk | QUADSPI_CR_SSHIFT);

	dev->CcrReg &= ~QUADSPI_CCR_DDRM;
	if (pCfgData->Phy == SPIPHY_QUAD_DDR)
	{
		dev->CcrReg |= QUADSPI_CCR_DDRM;
	}

	Stm32QspiSetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_QSPI;
	pDev->DevIntrf.Disable = Stm32QspiDisable;
	pDev->DevIntrf.Enable = Stm32QspiEnable;
	pDev->DevIntrf.GetRate = Stm32QspiGetRate;
	pDev->DevIntrf.SetRate = Stm32QspiSetRate;
	pDev->DevIntrf.StartRx = Stm32QspiStart;
	pDev->DevIntrf.RxData = Stm32QspiRxData;
	pDev->DevIntrf.StopRx = Stm32QspiStopRx;
	pDev->DevIntrf.StartTx = Stm32QspiStart;
	pDev->DevIntrf.TxData = Stm32QspiTxData;
	pDev->DevIntrf.StopTx = Stm32QspiStopTx;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.PowerOff = Stm32QspiPowerOff;
	pDev->DevIntrf.EnCnt = 1;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->FCR = reg->FCR;	// Clear all flags
	reg->CR |= QUADSPI_CR_EN;

	return true;
}

SPIPHY SPISetPhy(SPIDev_t * const pDev, SPIPHY Phy)
{
	if (Stm32QspiIsDev(pDev))
	{
		s_Stm32QspiDev.CcrReg &= ~QUADSPI_CCR_DDRM;
		if (Phy == SPIPHY_QUAD_DDR)
		{
			s_Stm32QspiDev.CcrReg |= QUADSPI_CCR_DDRM;
		}
	}

	pDev->Cfg.Phy = Phy;

	return pDev->Cfg.Phy;
}

/**
 * @brief	Set Quad SPI Flash size
 */
void QuadSPISetMemSize(SPIDev_t * const pDev, uint32_t Size)
{
	if (Stm32QspiIsDev(pDev) == false)
	{
		return;
	}

	Stm32QspiDev_t *dev = &s_Stm32QspiDev;

	Stm32QspiWaitBusy(dev, 100000);

	uint32_t dcr = dev->pReg->DCR & ~QUADSPI_DCR_FSIZE_Msk;

#ifdef __ICCARM__
	int32_t n = (41 - __CLZ(Size)) & 0x1F;
#else
	int32_t n = (41 - __builtin_clzl(Size)) & 0x1F;
#endif

	dcr |= (n & 0x1F) << QUADSPI_DCR_FSIZE_Pos;
	dev->pReg->DCR = dcr;

	n = (n - 1) / 8;
	if (n < 0)
	{
		n = 0;
	}
	dev->AdSize = n;
	dev->CcrReg &= ~QUADSPI_CCR_ADSIZE_Msk;
	dev->CcrReg |= n << QUADSPI_CCR_ADSIZE_Pos;
}

bool QuadSPISendCmd(SPIDev_t * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	if (Stm32QspiIsDev(pDev))
	{
		return Stm32QspiSendCmd(&pDev->DevIntrf, Cmd, Addr, AddrLen, DataLen, DummyCycle);
	}

	return false;
}

#endif // QUADSPI
