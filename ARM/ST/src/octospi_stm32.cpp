/**-------------------------------------------------------------------------
@file	octospi_stm32.cpp

@brief	STM32 OCTOSPI implementation

		OCTOSPI1 and OCTOSPI2 are the SPI devices DevNo 3 and 4
		(STM32_QOSPI_DEVNO_START), SPIInit of spi_stm32.cpp calls
		Stm32QospiInit for them. Master in polling mode, indirect mode.
		QuadSPISendCmd gives the command, its address, dummy cycles and data
		length; a command without data runs at once, one with data is started
		by RxData or TxData. The flash drivers (Flash, Nvm) describe any part
		this way. The chip select is a GPIO of the pin map
		from index OSPI_CS_IOPIN_IDX when ChipSel is SPICSEL_DRIVER. The rate
		is HCLK divided by 1 to 256 (PRESCALER of DCR2).

		Moved from STM32L4xx/src/octospi_stm32l4xx.cpp.

@author	Hoang Nguyen Hoan
@date	July 26, 2019

@license

MIT License

Copyright (c) 2019, I-SYST inc., all rights reserved

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
	uint32_t CcrReg;		// ADSIZE and DDTR, kept for every command
	// Command with a data phase given by QuadSPISendCmd, started by RxData
	// (indirect read) or TxData (indirect write)
	uint32_t PendCmd;
	uint32_t PendCcr;
	uint32_t PendDcyc;
	uint32_t PendAddr;
	uint32_t PendLen;
	bool bPendAddr;
	bool bPend;
} Stm32OspiDev_t;

#pragma pack(pop)

static Stm32OspiDev_t s_Stm32OspiDev[STM32_OSPI_MAXDEV] = {
	{ 0, NULL, OCTOSPI1, QOSPI_PHASE_IDLE, 0, 0, 0, 0, 0, 0, 0, false, false },
#ifdef OCTOSPI2
	{ 1, NULL, OCTOSPI2, QOSPI_PHASE_IDLE, 0, 0, 0, 0, 0, 0, 0, false, false },
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

// Stop a transfer that did not end, the controller is ready for the next one
static void Stm32OspiAbort(Stm32OspiDev_t *pDev)
{
	pDev->pReg->CR |= OCTOSPI_CR_ABORT;

	for (int i = 0; i < 100000 && (pDev->pReg->CR & OCTOSPI_CR_ABORT); i++);

	pDev->pReg->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
}

// The SPI device, what SPIGetHandle returns
static void *Stm32OspiGetHandle(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	return dev != NULL ? dev->pSpiDev : NULL;
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

// Line modes of the address and data phases of a command: 1 single, 2 dual,
// 3 quad lines, the CCR ADMODE and DMODE values
static void Stm32OspiModes(uint32_t Cmd, uint32_t &AdMode, uint32_t &DMode)
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

// Indirect mode. With FMODE set in CR, the controller starts a transfer on
// the last register of its sequence: IR when there is no address, AR
// otherwise; an indirect write then takes its data from the FIFO. A command
// with a data phase is kept here and started by RxData or TxData, which know
// the direction. A command without data runs now.
static bool Stm32OspiSendCmd(Stm32OspiDev_t *dev, uint32_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	OCTOSPI_TypeDef *reg = dev->pReg;
	uint32_t admode, dmode;
	bool baddr = Addr != (uint32_t)-1;

	Stm32OspiModes(Cmd, admode, dmode);

	uint32_t ccr = (dev->CcrReg & OCTOSPI_CCR_DDTR) | OCTOSPI_CCR_IMODE_0;

	if (baddr)
	{
		// Address size of the command, or the one of the memory size
		if (AddrLen > 0 && AddrLen <= 4)
		{
			ccr |= (uint32_t)(AddrLen - 1) << OCTOSPI_CCR_ADSIZE_Pos;
		}
		else
		{
			ccr |= dev->CcrReg & OCTOSPI_CCR_ADSIZE_Msk;
		}
		ccr |= admode << OCTOSPI_CCR_ADMODE_Pos;
	}

	if (Stm32OspiWaitBusy(dev, 100000) == false)
	{
		Stm32OspiAbort(dev);
	}

	if (DataLen > 0)
	{
		dev->PendCmd = Cmd;
		dev->PendCcr = ccr | (dmode << OCTOSPI_CCR_DMODE_Pos);
		dev->PendDcyc = (uint32_t)DummyCycle << OCTOSPI_TCR_DCYC_Pos;
		dev->PendAddr = Addr;
		dev->bPendAddr = baddr;
		dev->PendLen = DataLen;
		dev->bPend = true;

		return true;
	}

	dev->bPend = false;
	reg->FCR = OCTOSPI_FCR_CTCF;
	reg->CR &= ~OCTOSPI_CR_FMODE_Msk;	// indirect write, no data
	reg->CCR = ccr;
	reg->TCR &= ~OCTOSPI_TCR_DCYC_Msk;
	reg->IR = Cmd;
	if (baddr)
	{
		reg->AR = Addr;
	}

	if (Stm32OspiWaitTxComplete(dev, 100000) == false)
	{
		Stm32OspiAbort(dev);

		return false;
	}

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
		Stm32OspiAbort(dev);
	}

	dev->bPend = false;

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

// Start the command kept by QuadSPISendCmd with the data phase length Len
static void Stm32OspiStartData(Stm32OspiDev_t *dev, uint32_t Fmode, uint32_t Len)
{
	OCTOSPI_TypeDef *reg = dev->pReg;

	reg->FCR = OCTOSPI_FCR_CTCF;
	reg->CR = (reg->CR & ~OCTOSPI_CR_FMODE_Msk) | Fmode;
	reg->DLR = Len - 1;
	reg->CCR = dev->PendCcr;
	reg->TCR = (reg->TCR & ~OCTOSPI_TCR_DCYC_Msk) | dev->PendDcyc;
	reg->IR = dev->PendCmd;
	if (dev->bPendAddr)
	{
		reg->AR = dev->PendAddr;
	}
	dev->bPend = false;
	dev->QOPhase = QOSPI_PHASE_DATA;
}

// Receive the data phase of the command given by QuadSPISendCmd
static int Stm32OspiRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;
	OCTOSPI_TypeDef *reg = dev->pReg;
	int cnt = 0;

	if (dev->bPend == false || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	uint32_t len = (uint32_t)BuffLen < dev->PendLen ? (uint32_t)BuffLen : dev->PendLen;

	Stm32OspiStartData(dev, OCTOSPI_CR_FMODE_0, len);	// indirect read

	while (cnt < (int)len)
	{
		int timeout = 100000;

		while ((reg->SR & OCTOSPI_SR_FLEVEL_Msk) == 0 && timeout-- > 0);

		if (timeout < 0)
		{
			Stm32OspiAbort(dev);

			return cnt;
		}
		pBuff[cnt++] = *(volatile uint8_t *)&reg->DR;
	}

	if (Stm32OspiWaitTxComplete(dev, 100000) == false)
	{
		Stm32OspiAbort(dev);
	}

	return cnt;
}

// Stop receive
static void Stm32OspiStopRx(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	dev->bPend = false;
	dev->QOPhase = QOSPI_PHASE_IDLE;
	Stm32OspiCsHigh(dev);
}

// Send the data phase of the command given by QuadSPISendCmd
static int Stm32OspiTxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t*)pDev->pDevData;
	OCTOSPI_TypeDef *reg = dev->pReg;
	int cnt = 0;

	if (dev->bPend == false || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	uint32_t len = (uint32_t)DataLen < dev->PendLen ? (uint32_t)DataLen : dev->PendLen;

	Stm32OspiStartData(dev, 0, len);	// indirect write

	while (cnt < (int)len)
	{
		int timeout = 100000;

		// FTF: room in the FIFO, the threshold is 1
		while ((reg->SR & OCTOSPI_SR_FTF) == 0 && timeout-- > 0);

		if (timeout < 0)
		{
			Stm32OspiAbort(dev);

			return cnt;
		}
		*(volatile uint8_t *)&reg->DR = pData[cnt++];
	}

	if (Stm32OspiWaitTxComplete(dev, 100000) == false)
	{
		Stm32OspiAbort(dev);
	}

	return cnt;
}

// Stop transmit
static void Stm32OspiStopTx(DevIntrf_t * const pDev)
{
	Stm32OspiDev_t *dev = (Stm32OspiDev_t *)pDev->pDevData;

	dev->bPend = false;
	dev->QOPhase = QOSPI_PHASE_IDLE;
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
	pDev->DevIntrf.GetHandle = Stm32OspiGetHandle;
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
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bNoStop = false;
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
