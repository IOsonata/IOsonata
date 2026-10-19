/**-------------------------------------------------------------------------
@file	octospi_stm32.cpp

@brief	STM32 OCTOSPI implementation

		OCTOSPI1 and OCTOSPI2 are the SPI devices DevNo 3 and 4
		(STM32_QOSPI_DEVNO_START), SPIInit of spi_stm32.cpp calls
		Stm32QospiInit for them. Master in polling mode, indirect mode, the
		commands of QuadSPISendCmd. The chip select is a GPIO of the pin map
		from index OSPI_CS_IOPIN_IDX when ChipSel is SPICSEL_DRIVER. The rate
		is HCLK divided by 1 to 256 (PRESCALER of DCR2).

		Moved from STM32L4xx/src/octospi_stm32l4xx.cpp.

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

#if defined(OCTOSPI1)

#include <stddef.h>

#include "istddef.h"
#include "qospi_stm32.h"
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"
#include "storage/diskio_flash.h"

#ifdef OCTOSPI2
#define STM32_OSPI_MAXDEV			2
#else
#define STM32_OSPI_MAXDEV			1
#endif

#pragma pack(push, 4)

typedef struct {
	int Idx;				// 0 OCTOSPI1, 1 OCTOSPI2
	SPIDev_t *pSpiDev;
	OCTOSPI_TypeDef *pReg;
	QOSPI_PHASE QOPhase;
	int AdSize;
	uint32_t CcrReg;
} Stm32OspiDev_t;

#pragma pack(pop)

static Stm32OspiDev_t s_Stm32OspiDev[STM32_OSPI_MAXDEV] = {
	{ 0, NULL, OCTOSPI1, QOSPI_PHASE_IDLE, 0, 0 },
#ifdef OCTOSPI2
	{ 1, NULL, OCTOSPI2, QOSPI_PHASE_IDLE, 0, 0 },
#endif
};

// The device of pDev, NULL if pDev is not an initialized OCTOSPI device
static Stm32OspiDev_t *Stm32OspiDev(SPIDev_t * const pDev)
{
	int idx = pDev->Cfg.DevNo - STM32_QOSPI_DEVNO_START;

	if (idx < 0 || idx >= STM32_OSPI_MAXDEV || s_Stm32OspiDev[idx].pSpiDev != pDev)
	{
		return NULL;
	}

	return &s_Stm32OspiDev[idx];
}

static bool Stm32OspiWaitBusy(Stm32OspiDev_t *pDev, int Timeout)
{
	do {
		if ((pDev->pReg->SR & OCTOSPI_SR_BUSY) == 0)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static bool Stm32OspiWaitTxComplete(Stm32OspiDev_t *pDev, int Timeout)
{
	do {
		if ((pDev->pReg->SR & OCTOSPI_SR_TCF))
		{
			pDev->pReg->FCR |= OCTOSPI_FCR_CTCF;
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static bool Stm32OspiWaitFifo(Stm32OspiDev_t *pDev, int Timeout)
{
	do {
		if ((pDev->pReg->SR & OCTOSPI_SR_FLEVEL_Msk))
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static uint32_t Stm32OspiGetRate(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	return dev->pSpiDev->Cfg.Rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
static uint32_t Stm32OspiSetRate(DevIntrf_t * const pDev, uint32_t DataRate)
{
	if (DataRate == 0)
	{
		return 0;
	}

	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;
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

	dev->pReg->DCR2 &= ~OCTOSPI_DCR2_PRESCALER_Msk;
	dev->pReg->DCR2 |= (div & 0xFF) << OCTOSPI_DCR2_PRESCALER_Pos;
	dev->pSpiDev->Cfg.Rate = hclk / (div + 1);

	return dev->pSpiDev->Cfg.Rate;
}

static void Stm32OspiDisable(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	Stm32OspiWaitBusy(dev, 100000);

	dev->pReg->CR &= ~OCTOSPI_CR_EN;
}

static void Stm32OspiEnable(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	dev->pReg->CR |= OCTOSPI_CR_EN;
}

static void Stm32OspiReset(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	RCC->AHB3RSTR |= RCC_AHB3RSTR_OSPI1RST_Msk << dev->Idx;
	msDelay(1);
	RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI1RST_Msk << dev->Idx);
}

static void Stm32OspiPowerOff(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	Stm32OspiDisable(pDev);

	RCC->AHB3ENR &= ~(RCC_AHB3ENR_OSPI1EN << dev->Idx);

	for (int i = 0; i < dev->pSpiDev->Cfg.NbIOPins; i++)
	{
		IOPinConfig(dev->pSpiDev->Cfg.pIOPinMap[i].PortNo, dev->pSpiDev->Cfg.pIOPinMap[i].PinNo,
					0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	}
}

static bool Stm32OspiSendCmd(Stm32OspiDev_t *dev, uint32_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	dev->CcrReg &= ~(OCTOSPI_CCR_IMODE_Msk | OCTOSPI_CCR_DMODE_Msk | OCTOSPI_CCR_ADMODE_Msk);
	dev->pReg->TCR &= ~OCTOSPI_TCR_DCYC_Msk;

	if (DataLen > 0)
	{
		dev->pReg->DLR = DataLen - 1;
		switch (Cmd)
		{
			case FLASH_CMD_DREAD:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_DMODE_1;
				dev->pReg->TCR |= (DummyCycle << OCTOSPI_TCR_DCYC_Pos);
				break;
			case FLASH_CMD_QREAD:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_DMODE_Msk;
				dev->pReg->TCR |= (DummyCycle << OCTOSPI_TCR_DCYC_Pos);
				break;
			case FLASH_CMD_2READ:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_1 | OCTOSPI_CCR_DMODE_1;
				dev->pReg->TCR |= (DummyCycle << OCTOSPI_TCR_DCYC_Pos);
				break;
			case FLASH_CMD_QWRITE:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_DMODE_Msk;
				break;
			case FLASH_CMD_4READ:
				dev->pReg->TCR |= (DummyCycle << OCTOSPI_TCR_DCYC_Pos);
				// fall through
			case FLASH_CMD_4WRITE:
			case FLASH_CMD_E4WRITE:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_Msk | OCTOSPI_CCR_DMODE_Msk;
				break;
			default:
				if (Addr != (uint32_t)-1)
				{
					dev->CcrReg |= OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_DMODE_0;
				}
				else
				{
					dev->CcrReg |= OCTOSPI_CCR_DMODE_0;
				}
		}
	}
	else
	{
		if (Addr != (uint32_t)-1)
		{
			dev->CcrReg |= OCTOSPI_CCR_ADMODE_0;
		}
		dev->pReg->DLR = 0;
	}

	dev->pReg->CCR = dev->CcrReg;
	if (Addr != (uint32_t)-1)
	{
		dev->pReg->AR = Addr;
	}

	dev->pReg->IR = Cmd;

	Stm32OspiWaitFifo(dev, 100000);

	return true;
}

// Initiate receive or transmit, chip select low
static bool Stm32OspiStart(DevIntrf_t * const pDev, uint32_t DevCs)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	if ((int)DevCs < 0 || (int)DevCs >= dev->pSpiDev->Cfg.NbIOPins - OSPI_CS_IOPIN_IDX)
	{
		return false;
	}

	if (Stm32OspiWaitBusy(dev, 100000) == false)
	{
		return false;
	}

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		// Handle multi-chipsel manually
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + OSPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + OSPI_CS_IOPIN_IDX].PinNo);
	}

	dev->QOPhase = QOSPI_PHASE_INST;

	return true;
}

// Chip select high
static void Stm32OspiCsHigh(Stm32OspiDev_t *dev)
{
	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + OSPI_CS_IOPIN_IDX].PortNo,
				 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + OSPI_CS_IOPIN_IDX].PinNo);
	}
}

// Receive Data only, no Start/Stop condition
static int Stm32OspiRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;
	OCTOSPI_TypeDef *reg = dev->pReg;
	// DR accessed as a half word for data above 8 bits
	volatile uint16_t *dr16 = (volatile uint16_t *)((uintptr_t)reg + offsetof(OCTOSPI_TypeDef, DR));
	int cnt = 0;

	reg->DLR = BuffLen - 1;
	reg->CR |= OCTOSPI_CR_FMODE_0;
	reg->AR = reg->AR;

	while (BuffLen > 0)
	{
		Stm32OspiWaitFifo(dev, 100000);

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
static void Stm32OspiStopRx(DevIntrf_t * const pDev)
{
	Stm32OspiCsHigh((Stm32OspiDev_t *)pDev->pDevData);
}

// Send Data only, no Start/Stop condition
static int Stm32OspiTxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t*)pDev->pDevData;
	OCTOSPI_TypeDef *reg = dev->pReg;
	// DR accessed as a half word for data above 8 bits
	volatile uint16_t *dr16 = (volatile uint16_t *)((uintptr_t)reg + offsetof(OCTOSPI_TypeDef, DR));
	int cnt = 0;

	reg->DLR = DataLen - 1;
	reg->CR &= ~OCTOSPI_CR_FMODE_Msk;
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
static void Stm32OspiStopTx(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	Stm32OspiWaitTxComplete(dev, 100000);
	Stm32OspiCsHigh(dev);
}

bool Stm32QospiInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	int idx = pCfgData->DevNo - STM32_QOSPI_DEVNO_START;

	if (idx < 0 || idx >= STM32_OSPI_MAXDEV || pCfgData->Mode != SPIMODE_MASTER ||
		pCfgData->bIntEn || pCfgData->bDmaEn)
	{
		return false;
	}

	Stm32OspiDev_t *dev = &s_Stm32OspiDev[idx];
	OCTOSPI_TypeDef *reg = dev->pReg;

	pDev->Cfg = *pCfgData;
	dev->pSpiDev = pDev;
	pDev->DevIntrf.pDevData = (void*)dev;

	// Configure I/O pins
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	for (int i = 0; i < pCfgData->NbIOPins; i++)
	{
		IOPinSetSpeed(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	Stm32OspiReset(&pDev->DevIntrf);

	RCC->AHB3ENR |= RCC_AHB3ENR_OSPI1EN << dev->Idx;
	RCC->AHB3SMENR &= ~(RCC_AHB3SMENR_OSPI1SMEN << dev->Idx);

	reg->CR &= ~OCTOSPI_CR_FTHRES_Msk;
	reg->TCR &= ~OCTOSPI_TCR_SSHIFT;

	dev->CcrReg &= ~OCTOSPI_CCR_DDTR;
	if (pCfgData->Phy == SPIPHY_OCTO_DDR)
	{
		dev->CcrReg |= OCTOSPI_CCR_DDTR;
	}

	Stm32OspiSetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_QSPI;
	pDev->DevIntrf.Disable = Stm32OspiDisable;
	pDev->DevIntrf.Enable = Stm32OspiEnable;
	pDev->DevIntrf.GetRate = Stm32OspiGetRate;
	pDev->DevIntrf.SetRate = Stm32OspiSetRate;
	pDev->DevIntrf.StartRx = Stm32OspiStart;
	pDev->DevIntrf.RxData = Stm32OspiRxData;
	pDev->DevIntrf.StopRx = Stm32OspiStopRx;
	pDev->DevIntrf.StartTx = Stm32OspiStart;
	pDev->DevIntrf.TxData = Stm32OspiTxData;
	pDev->DevIntrf.StopTx = Stm32OspiStopTx;
	pDev->DevIntrf.Reset = Stm32OspiReset;
	pDev->DevIntrf.PowerOff = Stm32OspiPowerOff;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.EnCnt = 1;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->FCR = reg->FCR;	// Clear all flags
	reg->CR |= OCTOSPI_CR_EN;

	return true;
}

/**
 * @brief	Set Quad SPI Flash size
 */
void QuadSPISetMemSize(SPIDev_t * const pDev, uint32_t Size)
{
	Stm32OspiDev_t *dev = Stm32OspiDev(pDev);

	if (dev == NULL)
	{
		return;
	}

	Stm32OspiWaitBusy(dev, 100000);

	uint32_t dcr = dev->pReg->DCR1 & ~OCTOSPI_DCR1_DEVSIZE_Msk;

#ifdef __ICCARM__
	int32_t n = (41 - __CLZ(Size)) & 0x1F;
#else
	int32_t n = (41 - __builtin_clzl(Size)) & 0x1F;
#endif

	dcr |= (n & 0x1F) << OCTOSPI_DCR1_DEVSIZE_Pos;
	dev->pReg->DCR1 = dcr;

	n = (n - 1) / 8;
	if (n < 0)
	{
		n = 0;
	}
	dev->AdSize = n;
	dev->CcrReg &= ~OCTOSPI_CCR_ADSIZE_Msk;
	dev->CcrReg |= n << OCTOSPI_CCR_ADSIZE_Pos;
}

bool QuadSPISendCmd(SPIDev_t * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	Stm32OspiDev_t *dev = Stm32OspiDev(pDev);

	if (dev == NULL)
	{
		return false;
	}

	return Stm32OspiSendCmd(dev, Cmd, Addr, AddrLen, DataLen, DummyCycle);
}

#endif // OCTOSPI1
