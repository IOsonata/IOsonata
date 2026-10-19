/**-------------------------------------------------------------------------
@file	quadspi_stm32.cpp

@brief	STM32 QUADSPI implementation

		QUADSPI is the SPI device DevNo 3 (STM32_QOSPI_DEVNO_START), SPIInit of
		spi_stm32.cpp calls Stm32QospiInit for it. Master in polling mode,
		indirect mode. QuadSPISendCmd gives the command, its address, dummy
		cycles and data length; a command without data runs at once, one with
		data is started by RxData or TxData. The flash drivers (Flash, Nvm)
		describe any part this way. The chip select is a
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
	uint32_t CcrReg;		// ADSIZE and DDRM, kept for every command
	// Command with a data phase given by QuadSPISendCmd, started by RxData
	// (indirect read) or TxData (indirect write)
	uint32_t PendCcr;
	uint32_t PendAddr;
	uint32_t PendLen;
	bool bPendAddr;
	bool bPend;
} Stm32QspiDev_t;

#pragma pack(pop)

static Stm32QspiDev_t s_Stm32QspiDev = {
	NULL, QUADSPI, QOSPI_PHASE_IDLE, 0, 0, 0, 0, 0, false, false
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

// Stop a transfer that did not end, the controller is ready for the next one
static void Stm32QspiAbort(Stm32QspiDev_t *pDev)
{
	pDev->pReg->CR |= QUADSPI_CR_ABORT;

	for (int i = 0; i < 100000 && (pDev->pReg->CR & QUADSPI_CR_ABORT); i++);

	pDev->pReg->FCR = QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;
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

// Line modes of the address and data phases of a command: 1 single, 2 dual,
// 4 quad lines, as the CCR ADMODE and DMODE values 1, 2 and 3
static void Stm32QspiModes(uint8_t Cmd, uint32_t &AdMode, uint32_t &DMode)
{
	switch (Cmd)
	{
		case FLASH_CMD_DREAD:		// 1-1-2
			AdMode = 1;
			DMode = 2;
			break;
		case FLASH_CMD_2READ:		// 1-2-2
			AdMode = 2;
			DMode = 2;
			break;
		case FLASH_CMD_QREAD:		// 1-1-4
		case FLASH_CMD_QWRITE:
			AdMode = 1;
			DMode = 3;
			break;
		case FLASH_CMD_4READ:		// 1-4-4
		case FLASH_CMD_4WRITE:
		case FLASH_CMD_E4WRITE:
			AdMode = 3;
			DMode = 3;
			break;
		default:					// 1-1-1
			AdMode = 1;
			DMode = 1;
	}
}

// Indirect mode. The controller starts a transfer on the last register of
// its sequence: CCR when there is no address, AR otherwise; an indirect write
// then takes its data from the FIFO. FMODE is in CCR, so a command with a data
// phase is kept here and started by RxData or TxData, which know the
// direction. A command without data runs now.
static bool Stm32QspiSendCmd(DevIntrf_t * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;
	QUADSPI_TypeDef *reg = dev->pReg;
	uint32_t admode, dmode;
	bool baddr = Addr != (uint32_t)-1;

	Stm32QspiModes(Cmd, admode, dmode);

	uint32_t ccr = (dev->CcrReg & QUADSPI_CCR_DDRM) | Cmd | QUADSPI_CCR_IMODE_0;

	if (baddr)
	{
		// Address size of the command, or the one of the memory size
		if (AddrLen > 0 && AddrLen <= 4)
		{
			ccr |= (uint32_t)(AddrLen - 1) << QUADSPI_CCR_ADSIZE_Pos;
		}
		else
		{
			ccr |= dev->CcrReg & QUADSPI_CCR_ADSIZE_Msk;
		}
		ccr |= admode << QUADSPI_CCR_ADMODE_Pos;
	}

	if (Stm32QspiWaitBusy(dev, 100000) == false)
	{
		Stm32QspiAbort(dev);
	}

	if (DataLen > 0)
	{
		dev->PendCcr = ccr | (dmode << QUADSPI_CCR_DMODE_Pos) |
					   ((uint32_t)DummyCycle << QUADSPI_CCR_DCYC_Pos);
		dev->PendAddr = Addr;
		dev->bPendAddr = baddr;
		dev->PendLen = DataLen;
		dev->bPend = true;

		return true;
	}

	dev->bPend = false;
	reg->FCR = QUADSPI_FCR_CTCF;
	reg->CCR = ccr;					// indirect write, no data
	if (baddr)
	{
		reg->AR = Addr;
	}

	if (Stm32QspiWaitTxComplete(dev, 100000) == false)
	{
		Stm32QspiAbort(dev);

		return false;
	}

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
		Stm32QspiAbort(dev);
	}

	dev->bPend = false;

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

// Start the command kept by QuadSPISendCmd with the data phase length Len
static void Stm32QspiStartData(Stm32QspiDev_t *dev, uint32_t Fmode, uint32_t Len)
{
	QUADSPI_TypeDef *reg = dev->pReg;

	reg->FCR = QUADSPI_FCR_CTCF;
	reg->DLR = Len - 1;
	reg->CCR = dev->PendCcr | Fmode;
	if (dev->bPendAddr)
	{
		reg->AR = dev->PendAddr;
	}
	dev->bPend = false;
	dev->QOPhase = QOSPI_PHASE_DATA;
}

// Receive the data phase of the command given by QuadSPISendCmd
static int Stm32QspiRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;
	QUADSPI_TypeDef *reg = dev->pReg;
	int cnt = 0;

	if (dev->bPend == false || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	uint32_t len = (uint32_t)BuffLen < dev->PendLen ? (uint32_t)BuffLen : dev->PendLen;

	Stm32QspiStartData(dev, QUADSPI_CCR_FMODE_0, len);	// indirect read

	while (cnt < (int)len)
	{
		int timeout = 100000;

		while ((reg->SR & QUADSPI_SR_FLEVEL_Msk) == 0 && timeout-- > 0);

		if (timeout < 0)
		{
			Stm32QspiAbort(dev);

			return cnt;
		}
		pBuff[cnt++] = *(volatile uint8_t *)&reg->DR;
	}

	if (Stm32QspiWaitTxComplete(dev, 100000) == false)
	{
		Stm32QspiAbort(dev);
	}

	return cnt;
}

// Stop receive
static void Stm32QspiStopRx(DevIntrf_t * const pDev)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	dev->bPend = false;
	dev->QOPhase = QOSPI_PHASE_IDLE;
	Stm32QspiCsHigh(dev);
}

// Send the data phase of the command given by QuadSPISendCmd
static int Stm32QspiTxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t*)pDev->pDevData;
	QUADSPI_TypeDef *reg = dev->pReg;
	int cnt = 0;

	if (dev->bPend == false || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	uint32_t len = (uint32_t)DataLen < dev->PendLen ? (uint32_t)DataLen : dev->PendLen;

	Stm32QspiStartData(dev, 0, len);	// indirect write

	while (cnt < (int)len)
	{
		int timeout = 100000;

		// FTF: room in the FIFO, the threshold is 1
		while ((reg->SR & QUADSPI_SR_FTF) == 0 && timeout-- > 0);

		if (timeout < 0)
		{
			Stm32QspiAbort(dev);

			return cnt;
		}
		*(volatile uint8_t *)&reg->DR = pData[cnt++];
	}

	if (Stm32QspiWaitTxComplete(dev, 100000) == false)
	{
		Stm32QspiAbort(dev);
	}

	return cnt;
}

// Stop transmit
static void Stm32QspiStopTx(DevIntrf_t * const pDev)
{
	Stm32QspiDev_t *dev = (Stm32QspiDev_t *)pDev->pDevData;

	dev->bPend = false;
	dev->QOPhase = QOSPI_PHASE_IDLE;
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
