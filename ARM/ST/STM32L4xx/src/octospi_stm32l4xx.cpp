/**-------------------------------------------------------------------------
@file	octospi_stm32l4xx.cpp

@brief	OCTOSPI implementation on STM32L4xx series MCU

@author	Hoang Nguyen Hoan
@date	Apr. 13, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#include "diskio_flash.h"

extern STM32L4XX_SPIDEV s_STM32L4xxSPIDev[STM32L4XX_SPI_MAXDEV];
extern const int g_NbSTM32L4xxSPIDev = sizeof(s_STM32L4xxSPIDev) / sizeof(STM32L4XX_SPIDEV);

bool STM32L4xxOSPIWaitBusy(STM32L4XX_SPIDEV *pDev, int Timeout)
{
	do {
		if ((pDev->pOReg->SR & OCTOSPI_SR_BUSY) == 0)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool STM32L4xxOSPIWaitTxComplete(STM32L4XX_SPIDEV *pDev, int Timeout)
{
	do {
		if ((pDev->pOReg->SR & OCTOSPI_SR_TCF))
		{
			pDev->pOReg->FCR |= OCTOSPI_FCR_CTCF;
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool STM32L4xxOSPIWaitFifo(STM32L4XX_SPIDEV *pDev, int Timeout)
{
	do {
		if ((pDev->pOReg->SR & OCTOSPI_SR_FLEVEL_Msk))
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

// Set data rate in bits/sec (Hz)
// return actual rate
static uint32_t STM32L4xxOSPISetRate(DevIntrf_t * const pDev, uint32_t DataRate)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;
	uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos;
	uint32_t hclk = tmp & 8 ? SystemCoreClock >> ((tmp & 7) + 1) : SystemCoreClock;

	//uint32_t hclk = SystemHFClockGet();
	int32_t div = (hclk + (DataRate >> 1)) / DataRate - 1;

	if (div < 0)
	{
		div = 0;
	}

	dev->pOReg->DCR2 &= ~OCTOSPI_DCR2_PRESCALER_Msk;
	dev->pOReg->DCR2 |= (div & 0xFF) << OCTOSPI_DCR2_PRESCALER_Pos;
	dev->pSpiDev->Cfg.Rate = hclk / div;

	return dev->pSpiDev->Cfg.Rate;
}

void STM32L4xxOSPIDisable(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	STM32L4xxOSPIWaitBusy(dev, 100000);

    dev->pOReg->CR &= ~OCTOSPI_CR_EN;
}

static void STM32L4xxOSPIEnable(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

    dev->pOReg->CR |= OCTOSPI_CR_EN;
}

static void STM32L4xxOSPIPowerOff(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	STM32L4xxOSPIDisable(pDev);

	RCC->AHB3ENR &= ~(1 << (RCC_AHB3ENR_OSPI1EN + dev->DevNo - STM32L4XX_OSPI_DEVNO_START));

	for (int i = 0; i < dev->pSpiDev->Cfg.NbIOPins; i++)
	{
		IOPinConfig(dev->pSpiDev->Cfg.pIOPinMap[i].PortNo, dev->pSpiDev->Cfg.pIOPinMap[i].PinNo,
					0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	}
}

bool STM32L4xxOSPISendCmd(DevIntrf_t * const pDev, uint32_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	//dev->CcrReg &= ~(OCTOSPI_CCR_INSTRUCTION_Msk | OCTOSPI_CCR_IMODE_Msk | OCTOSPI_CCR_DMODE_Msk |
	//				OCTOSPI_CCR_FMODE_Msk | OCTOSPI_CCR_ADMODE_Msk | OCTOSPI_CCR_DCYC_Msk);
	//dev->CcrReg |= Cmd | OCTOSPI_CCR_IMODE_0;
	dev->CcrReg &= ~(OCTOSPI_CCR_IMODE_Msk | OCTOSPI_CCR_DMODE_Msk | OCTOSPI_CCR_ADMODE_Msk);
	dev->pOReg->TCR &= ~OCTOSPI_TCR_DCYC_Msk;

	if (DataLen > 0)
	{
		dev->pOReg->DLR = DataLen - 1;
		switch (Cmd)
		{
			case FLASH_CMD_DREAD:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_DMODE_1;
				dev->pOReg->TCR |= (DummyCycle << OCTOSPI_TCR_DCYC_Pos);
				break;
			case FLASH_CMD_QREAD:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_DMODE_Msk;
				dev->pOReg->TCR |= (DummyCycle << OCTOSPI_TCR_DCYC_Pos);
				break;
			case FLASH_CMD_2READ:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_1 | OCTOSPI_CCR_DMODE_1;
				dev->pOReg->TCR |= (DummyCycle << OCTOSPI_TCR_DCYC_Pos);
				break;
			case FLASH_CMD_QWRITE:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_DMODE_Msk;// | (DummyCycle << QUADSPI_CCR_DCYC_Pos);
				break;
			case FLASH_CMD_4READ:
				dev->pOReg->TCR |= (DummyCycle << OCTOSPI_TCR_DCYC_Pos);
			case FLASH_CMD_4WRITE:
			case FLASH_CMD_E4WRITE:
				dev->CcrReg |= OCTOSPI_CCR_ADMODE_Msk | OCTOSPI_CCR_DMODE_Msk;
				break;
			default:
				if (Addr != -1)
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
		if (Addr != -1)
		{
			dev->CcrReg |= OCTOSPI_CCR_ADMODE_0;
		}
		dev->pOReg->DLR = 0;
	}

	dev->pOReg->CCR = dev->CcrReg;
	if (Addr != -1)
	{
		dev->pOReg->AR = Addr;
	}

	dev->pOReg->IR = Cmd;

	STM32L4xxOSPIWaitFifo(dev, 100000);
	//STM32L4xxOSPIWaitTxComplete(dev, 100000);

	return true;
}

// Initial receive
static bool STM32L4xxOSPIStartRx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - OSPI_CS_IOPIN_IDX)
		return false;

	if (STM32L4xxOSPIWaitBusy(dev, 100000) == false)
		return false;

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

// Receive Data only, no Start/Stop condition
static int STM32L4xxOSPIRxDataDma(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;
	int cnt = 0;

	return cnt;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxOSPIRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;
	OCTOSPI_TypeDef *reg = dev->pOReg;
    int cnt = 0;
    uint16_t d = 0;

	reg->DLR = BuffLen - 1;
	reg->CR |= OCTOSPI_CR_FMODE_0;
	reg->AR = reg->AR;

    while (BuffLen > 0)
    {
    	uint16_t x = 0;

        if (dev->pSpiDev->Cfg.DataSize > 8)
        {

            STM32L4xxOSPIWaitFifo(dev, 100000);
            uint16_t x = *(uint16_t*)&reg->DR;
        	pBuff[0] = x & 0xff;
        	pBuff[1] = (x >> 8) & 0xFF;
        	BuffLen -= 2;
        	pBuff += 2;
        	cnt += 2;
        }
        else
        {
            STM32L4xxOSPIWaitFifo(dev, 100000);
			*pBuff = *(uint8_t*)&reg->DR;
        	BuffLen--;
        	pBuff++;
        	cnt++;
        }
    }

    return cnt;
}

// Stop receive
static void STM32L4xxOSPIStopRx(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + OSPI_CS_IOPIN_IDX].PortNo,
				 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + OSPI_CS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
static bool STM32L4xxOSPIStartTx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - OSPI_CS_IOPIN_IDX)
		return false;

	if (STM32L4xxOSPIWaitBusy(dev, 100000) == false)
		return false;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + OSPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + OSPI_CS_IOPIN_IDX].PinNo);
	}

	dev->QOPhase = QOSPI_PHASE_INST;

	return true;
}

// Transmit Data only, no Start/Stop condition
static int STM32L4xxOSPITxDataDma(DevIntrf_t * const pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Send Data only, no Start/Stop condition
static int STM32L4xxOSPITxData(DevIntrf_t *pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV*)pDev->pDevData;
	OCTOSPI_TypeDef *reg = dev->pOReg;
    int cnt = 0;
    uint16_t d;

	reg->DLR = DataLen - 1;
	//dev->CcrReg &= ~OCTOSPI_CR_FMODE_Msk;
	dev->pOReg->CR &= ~OCTOSPI_CR_FMODE_Msk;
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
static void STM32L4xxOSPIStopTx(DevIntrf_t * const pDev)
{
	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->pDevData;
	OCTOSPI_TypeDef *reg = dev->pOReg;

	STM32L4xxOSPIWaitTxComplete(dev, 100000);
//	STM32L4xxSPIWaitBusy(dev, 100000);

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

bool STM32L4xxQuadSPIInit(SPIDEV * const pDev, const SPICFG *pCfgData)
{
	OCTOSPI_TypeDef *reg;
	uint32_t ctrlreg = 0;

	if (pCfgData->DevNo != (STM32L4XX_SPI_MAXDEV - 1))
	{
		return false;
	}

	STM32L4XX_SPIDEV *dev = (STM32L4XX_SPIDEV *)pDev->DevIntrf.pDevData;

	// Get the correct register map
	reg = s_STM32L4xxSPIDev[pCfgData->DevNo].pOReg;

	RCC->AHB3RSTR |= 1 << (RCC_AHB3RSTR_OSPI1RST_Pos + dev->DevNo - STM32L4XX_OSPI_DEVNO_START);
	RCC->AHB3RSTR &= ~(1 << (RCC_AHB3RSTR_OSPI1RST_Pos + dev->DevNo - STM32L4XX_OSPI_DEVNO_START));

	msDelay(1);

	RCC->AHB3ENR |= 1 << (RCC_AHB3ENR_OSPI1EN + dev->DevNo - STM32L4XX_OSPI_DEVNO_START);
	RCC->AHB3SMENR &= ~(1 << (RCC_AHB3ENR_OSPI1EN + dev->DevNo - STM32L4XX_OSPI_DEVNO_START));

	reg->CR &= ~OCTOSPI_CR_FTHRES_Msk;
	reg->TCR &= ~OCTOSPI_TCR_SSHIFT;
	//reg->CR |= 0 << QUADSPI_CR_FTHRES_Pos | QUADSPI_CR_SSHIFT;

	if (pCfgData->Phy == SPIPHY_OCTO_DDR)
	{
		dev->CcrReg = OCTOSPI_CCR_DDTR;
	}

	STM32L4xxOSPISetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_QSPI;
	pDev->DevIntrf.Disable = STM32L4xxOSPIDisable;
	pDev->DevIntrf.Enable = STM32L4xxOSPIEnable;
	pDev->DevIntrf.GetRate = STM32L4xxSPIGetRate;
	pDev->DevIntrf.SetRate = STM32L4xxOSPISetRate;
	pDev->DevIntrf.StartRx = STM32L4xxOSPIStartRx;
	pDev->DevIntrf.RxData = STM32L4xxOSPIRxData;
	pDev->DevIntrf.StopRx = STM32L4xxOSPIStopRx;
	pDev->DevIntrf.StartTx = STM32L4xxOSPIStartTx;
	pDev->DevIntrf.TxData = STM32L4xxOSPITxData;
	pDev->DevIntrf.StopTx = STM32L4xxOSPIStopTx;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.PowerOff = STM32L4xxOSPIPowerOff;
	pDev->DevIntrf.EnCnt = 1;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->FCR = reg->FCR;	// Clear all flags
    reg->CR |= OCTOSPI_CR_EN;

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

		STM32L4xxOSPIWaitBusy(dev, 100000);

		uint32_t dcr = dev->pOReg->DCR1 & ~OCTOSPI_DCR1_DEVSIZE_Msk;

#ifdef __ICCARM__
		int32_t n = (41 - __CLZ(Size)) & 0x1F;
#else
		int32_t n = (41 - __builtin_clzl(Size)) & 0x1F;
#endif

		dcr |= (n & 0x1F) << OCTOSPI_DCR1_DEVSIZE_Pos;
		dev->pOReg->DCR1 = dcr;

		n = (n - 1) / 8;
		if (n < 0)
			n = 0;
		dev->AdSize = n;
		dev->CcrReg &= ~OCTOSPI_CCR_ADSIZE_Msk;
		dev->CcrReg |= n << OCTOSPI_CCR_ADSIZE_Pos;
	}
}

bool QuadSPISendCmd(SPIDEV * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	if (pDev->Cfg.DevNo == STM32L4XX_SPI_MAXDEV - 1)
	{
		return STM32L4xxOSPISendCmd(&pDev->DevIntrf, Cmd, Addr, AddrLen, DataLen, DummyCycle);
	}

	return false;
}

void OSPIIrqHandler(int DevNo)
{

}

extern "C" void OCTOSPI1_IRQHandler(void)
{
	OSPIIrqHandler(STM32L4XX_OSPI_DEVNO_START);
    NVIC_ClearPendingIRQ(OCTOSPI1_IRQn);
}

extern "C" void OCTOSPI2_IRQHandler(void)
{
	OSPIIrqHandler(STM32L4XX_OSPI_DEVNO_START + 1);
    NVIC_ClearPendingIRQ(OCTOSPI2_IRQn);
}



