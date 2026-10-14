/**-------------------------------------------------------------------------
@file	spi_lpc11uxx.c

@brief	SSP/SPI implementation on LPC11Uxx

		 Current implementation
		 	 Master mode
		 	 Polling

@author	Hoang Nguyen Hoan
@date	Feb. 20, 2015

@license

Copyright (c) 2015, I-SYST inc., all rights reserved

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
#include "LPC11Uxx.h"

#include "spi_lpcxx.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"

#define LPC11U_SYSAHBCLKCTRL_SSP0_EN		(1 << 11)
#define LPC11U_SYSAHBCLKCTRL_SSP1_EN		(1 << 18)
#define LPC11U_SYSAHBCLKCTRL_GPIO_EN		(1 << 6)

#define LPC11U_PRESETCTRL_SSP0_RST			(1 << 0)
#define LPC11U_PRESETCTRL_SSP1_RST			(1 << 2)

#define LPC11U_MAX_SSPDEV		2

extern uint32_t SystemMainClkFreq;

SSPDEV g_SspDev[LPC11U_MAX_SSPDEV] = {
		{0, 0, LPC_SSP0, NULL, false, false, false},
		{1, 0, LPC_SSP1, NULL, false, false, false},
};

// UM10462 chapter 7: routes are MCU-specific; transfers remain shared.
static bool Lpc11SSPSignalPin(int DevNo, int Signal, const IOPinCfg_t *pPin)
{
	if (DevNo == 0)
	{
		switch (Signal)
		{
			case SPI_SCK_IOPIN_IDX:
				return (pPin->PortNo == 0 && (pPin->PinNo == 6 || pPin->PinNo == 10) &&
					pPin->PinOp == IOPINOP_FUNC2) ||
					(pPin->PortNo == 1 && pPin->PinNo == 29 && pPin->PinOp == IOPINOP_FUNC1);
			case SPI_MISO_IOPIN_IDX:
				return pPin->PortNo == 0 && pPin->PinNo == 8 && pPin->PinOp == IOPINOP_FUNC1;
			case SPI_MOSI_IOPIN_IDX:
				return pPin->PortNo == 0 && pPin->PinNo == 9 && pPin->PinOp == IOPINOP_FUNC1;
		}
	}
	else
	{
		switch (Signal)
		{
			case SPI_SCK_IOPIN_IDX:
				return pPin->PortNo == 1 &&
					((pPin->PinNo == 15 && pPin->PinOp == IOPINOP_FUNC3) ||
					(pPin->PinNo == 20 && pPin->PinOp == IOPINOP_FUNC2));
			case SPI_MISO_IOPIN_IDX:
				return (pPin->PortNo == 0 && pPin->PinNo == 22 && pPin->PinOp == IOPINOP_FUNC3) ||
					(pPin->PortNo == 1 && pPin->PinNo == 21 && pPin->PinOp == IOPINOP_FUNC2);
			case SPI_MOSI_IOPIN_IDX:
				return pPin->PinOp == IOPINOP_FUNC2 &&
					((pPin->PortNo == 0 && pPin->PinNo == 21) ||
					(pPin->PortNo == 1 && pPin->PinNo == 22));
		}
	}
	return false;
}

static void Lpc11SSPReset(DevIntrf_t *pDev)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;
	uint32_t cr0 = dev->pSspReg->CR0;
	uint32_t cpsr = dev->pSspReg->CPSR;
	uint32_t reset = dev->DevNo == 0 ? LPC11U_PRESETCTRL_SSP0_RST : LPC11U_PRESETCTRL_SSP1_RST;
	uint32_t state = DisableInterrupt();

	// Reset aborts outstanding frames and empties both FIFOs.
	LPC_SYSCON->PRESETCTRL &= ~reset;
	(void)LPC_SYSCON->PRESETCTRL;
	LPC_SYSCON->PRESETCTRL |= reset;
	dev->pSspReg->CR1 = 0;
	dev->pSspReg->CR0 = cr0;
	dev->pSspReg->CPSR = cpsr;
	dev->pSspReg->IMSC = 0;
	dev->pSspReg->ICR = 3;
	if (dev->bSelected && spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi->CurDevCs];

		IOPinSet(pin->PortNo, pin->PinNo);
	}
	dev->bSelected = false;
	dev->bError = false;
	spi->CurDevCs = -1;
	pDev->bTxReady = true;
	if (dev->bEnabled)
	{
		dev->pSspReg->CR1 = LPCSSP_CR1_SSP_EN;
	}
	EnableInterrupt(state);
}

static void Lpc11SSPPowerOff(DevIntrf_t *pDev)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;
	uint32_t clock = dev->DevNo == 0 ? LPC11U_SYSAHBCLKCTRL_SSP0_EN : LPC11U_SYSAHBCLKCTRL_SSP1_EN;

	LpcSSPDisable(pDev);
	LPC_SYSCON->SYSAHBCLKCTRL &= ~clock;
}

SPIPHY SPISetPhy(SPIDev_t *pDev, SPIPHY Phy)
{
	(void)Phy;
	return pDev->Cfg.Phy;
}

bool SPIInit(SPIDev_t *pDev, const SPICfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL || pCfgData->DevNo < 0 ||
		pCfgData->DevNo >= LPC11U_MAX_SSPDEV || SystemMainClkFreq == 0 ||
		pCfgData->Mode != SPIMODE_MASTER || pCfgData->Phy != SPIPHY_NORMAL ||
		pCfgData->bDmaEn || pCfgData->bIntEn || pCfgData->DataSize < 4 ||
		pCfgData->DataSize > 16 || pCfgData->BitOrder != SPIDATABIT_MSB ||
		(pCfgData->ClkPol != SPICLKPOL_HIGH && pCfgData->ClkPol != SPICLKPOL_LOW) ||
		(pCfgData->DataPhase != SPIDATAPHASE_FIRST_CLK && pCfgData->DataPhase != SPIDATAPHASE_SECOND_CLK) ||
		(pCfgData->ChipSel != SPICSEL_AUTO && pCfgData->ChipSel != SPICSEL_MAN) ||
		pCfgData->pIOPinMap == NULL || pCfgData->NbIOPins < SPI_CS_IOPIN_IDX ||
		(pCfgData->ChipSel == SPICSEL_AUTO && pCfgData->NbIOPins <= SPI_CS_IOPIN_IDX) ||
		pCfgData->MaxRetry < 0)
	{
		return false;
	}
	uint32_t used[2] = {0, 0};

	for (int i = 0; i < pCfgData->NbIOPins; i++)
	{
		const IOPinCfg_t *pin = &pCfgData->pIOPinMap[i];

		if (!Lpc11GpioValidPin(pin->PortNo, pin->PinNo) ||
			pin->PinOp < IOPINOP_GPIO || pin->PinOp > IOPINOP_FUNC7)
		{
			return false;
		}
		if (i < SPI_CS_IOPIN_IDX && !Lpc11SSPSignalPin(pCfgData->DevNo, i, pin))
		{
			return false;
		}
		uint32_t mask = 1UL << pin->PinNo;

		// A later pin entry must not disconnect a signal or alias another CS.
		if (used[pin->PortNo] & mask)
		{
			return false;
		}
		used[pin->PortNo] |= mask;
		// Automatic CS is driven by the GPIO helpers.
		if (i >= SPI_CS_IOPIN_IDX && pCfgData->ChipSel == SPICSEL_AUTO &&
			(pin->PinOp != IOPINOP_GPIO || pin->PinDir != IOPINDIR_OUTPUT))
		{
			return false;
		}
	}
	SSPDEV *dev = &g_SspDev[pCfgData->DevNo];
	uint32_t state = DisableInterrupt();

	if (dev->bSelected || (dev->pSpiDev != NULL && dev->pSpiDev != pDev) ||
		g_SspDev[1 - pCfgData->DevNo].pSpiDev == pDev)
	{
		EnableInterrupt(state);
		return false;
	}
	dev->pSpiDev = pDev;
	dev->bEnabled = false;
	dev->bSelected = false;
	dev->bError = false;
	pDev->Cfg = *pCfgData;
	pDev->FirstRdData = -1;
	pDev->CurDevCs = -1;
	pDev->DevIntrf.pDevData = dev;
	LPC_SYSCON->SYSAHBCLKCTRL |= LPC11U_SYSAHBCLKCTRL_GPIO_EN;
	if (pCfgData->DevNo == 0)
	{
		LPC_SYSCON->SYSAHBCLKCTRL |= LPC11U_SYSAHBCLKCTRL_SSP0_EN;
		LPC_SYSCON->SSP0CLKDIV = 1;
		NVIC_DisableIRQ(SSP0_IRQn);
	}
	else
	{
		LPC_SYSCON->SYSAHBCLKCTRL |= LPC11U_SYSAHBCLKCTRL_SSP1_EN;
		LPC_SYSCON->SSP1CLKDIV = 1;
		NVIC_DisableIRQ(SSP1_IRQn);
	}
	uint32_t div = pCfgData->DevNo == 0 ? LPC_SYSCON->SSP0CLKDIV : LPC_SYSCON->SSP1CLKDIV;

	dev->PClkFreq = SystemMainClkFreq / (div & 0xFFU);
	Lpc11SSPReset(&pDev->DevIntrf);
	EnableInterrupt(state);

	// Preload high before making the GPIO chip selects outputs.
	if (pCfgData->ChipSel == SPICSEL_AUTO)
	{
		for (int i = SPI_CS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
		{
			IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
		}
	}
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);
	uint32_t cr0 = pCfgData->DataSize - 1U;

	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cr0 |= LPCSSP_CR0_CPHA_SECOND;
	}
	// Generic HIGH means active high: CPOL=0, idle low.
	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
		cr0 |= LPCSSP_CR0_CPOL_HI;
	}
	dev->pSspReg->CR0 = cr0;
	dev->pSspReg->CPSR = 2;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.Disable = LpcSSPDisable;
	pDev->DevIntrf.Enable = LpcSSPEnable;
	pDev->DevIntrf.GetRate = LpcSSPGetRate;
	pDev->DevIntrf.SetRate = LpcSSPSetRate;
	pDev->DevIntrf.StartRx = LpcSSPStartRx;
	pDev->DevIntrf.RxData = LpcSSPRxData;
	pDev->DevIntrf.StopRx = LpcSSPStopRx;
	pDev->DevIntrf.StartTx = LpcSSPStartTx;
	pDev->DevIntrf.TxData = LpcSSPTxData;
	pDev->DevIntrf.TxSrData = LpcSSPTxData;
	pDev->DevIntrf.StopTx = LpcSSPStopTx;
	pDev->DevIntrf.Reset = Lpc11SSPReset;
	pDev->DevIntrf.PowerOff = Lpc11SSPPowerOff;
	pDev->DevIntrf.GetHandle = LpcSSPGetHandle;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	LpcSSPSetRate(&pDev->DevIntrf, pCfgData->Rate);
	LpcSSPEnable(&pDev->DevIntrf);
	return true;
}
