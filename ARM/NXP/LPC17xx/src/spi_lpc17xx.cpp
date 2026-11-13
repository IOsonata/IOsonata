/**-------------------------------------------------------------------------
@file	spi_lpc17xx.c

@brief	SSP/SPI implementation on LPC17xx

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
#include "LPC17xx.h"

#include "spi_lpcxx.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"

#define LPC17_MAX_SSPDEV 2
#define LPC17_SSP_DRAIN_WAIT (100000 + 9 * 16 * 254 * 256)

static SSPDEV g_SspDev[LPC17_MAX_SSPDEV] = {
	{0, 0, LPC_SSP0, NULL, false, false, false},
	{1, 0, LPC_SSP1, NULL, false, false, false},
};
static const uint32_t s_PowerMask[] = {1UL << 21, 1UL << 10};
static const IRQn_Type s_Irq[] = {SSP0_IRQn, SSP1_IRQn};
static const int s_Ssp0Port0Pins[] = {15, 17, 18};
static const int s_Ssp0Port1Pins[] = {20, 23, 24};

// UM10360 chapter 8. Package bonding may expose fewer pins.
static bool Lpc17SSPPinValid(int Port, int Pin)
{
	if ((unsigned)Pin >= 32U)
	{
		return false;
	}
	switch (Port)
	{
		case 0:
			return Pin <= 11 || (Pin >= 15 && Pin <= 30);
		case 1:
			return Pin <= 1 || Pin == 4 || (Pin >= 8 && Pin <= 10) || Pin >= 14;
		case 2:
			return Pin <= 13;
		case 3:
			return Pin == 25 || Pin == 26;
		case 4:
			return Pin == 28 || Pin == 29;
	}
	return false;
}

static bool Lpc17SSPSignalPin(int DevNo, int Signal, const IOPinCfg_t *pPin)
{
	if (DevNo == 0)
	{
		return (pPin->PortNo == 0 && pPin->PinNo == s_Ssp0Port0Pins[Signal] && pPin->PinOp == IOPINOP_FUNC2) ||
			(pPin->PortNo == 1 && pPin->PinNo == s_Ssp0Port1Pins[Signal] && pPin->PinOp == IOPINOP_FUNC3);
	}
	return pPin->PinOp == IOPINOP_FUNC2 &&
		((pPin->PortNo == 0 && pPin->PinNo == 7 + Signal) ||
		 (Signal == SPI_SCK_IOPIN_IDX && pPin->PortNo == 1 && pPin->PinNo == 31));
}

static void Lpc17SSPReset(DevIntrf_t *pDev)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;
	// PCONP gates clocks; it does not reset SSP FIFOs. Finish queued frames
	// at the existing rate, then disable and drain RX. PCLK is CCLK here.
	bool idle = LpcSSPWaitBusy(dev, LPC17_SSP_DRAIN_WAIT);
	uint32_t state = DisableInterrupt();

	dev->pSspReg->CR1 = 0;
	dev->pSspReg->IMSC = 0;
	dev->pSspReg->DMACR = 0;
	for (int i = 0; i < 8 && (LPCSSP_READ(dev->pSspReg, SR) & LPCSSP_SR_RNE); i++)
	{
		(void)LPCSSP_READ(dev->pSspReg, DR);
	}
	LPCSSP_WRITE(dev->pSspReg, ICR, 3);
	NVIC_ClearPendingIRQ(s_Irq[dev->DevNo]);
	if (dev->bSelected && spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi->CurDevCs];

		IOPinSet(pin->PortNo, pin->PinNo);
	}
	dev->bSelected = false;
	dev->bError = !idle;
	if (!idle)
	{
		dev->bEnabled = false;
	}
	spi->CurDevCs = -1;
	pDev->bTxReady = true;
	if (dev->bEnabled && idle)
	{
		dev->pSspReg->CR1 = LPCSSP_CR1_SSP_EN;
	}
	EnableInterrupt(state);
}

static void Lpc17SSPEnable(DevIntrf_t *pDev)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;

	if (!dev->bError)
	{
		LpcSSPEnable(pDev);
	}
}

static void Lpc17SSPPowerOff(DevIntrf_t *pDev)
{
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;

	LpcSSPDisable(pDev);
	uint32_t state = DisableInterrupt();

	LPC_SC->PCONP &= ~s_PowerMask[dev->DevNo];
	EnableInterrupt(state);
}

SPIPHY SPISetPhy(SPIDev_t *pDev, SPIPHY Phy)
{
	(void)Phy;
	return pDev->Cfg.Phy;
}

bool SPIInit(SPIDev_t *pDev, const SPICfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL || pCfgData->DevNo < 0 ||
		pCfgData->DevNo >= LPC17_MAX_SSPDEV || SystemCoreClock == 0 ||
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
	uint32_t used[5] = {0};

	for (int i = 0; i < pCfgData->NbIOPins; i++)
	{
		const IOPinCfg_t *pin = &pCfgData->pIOPinMap[i];

		if (!Lpc17SSPPinValid(pin->PortNo, pin->PinNo) ||
			pin->PinOp < IOPINOP_GPIO || pin->PinOp > IOPINOP_FUNC3)
		{
			return false;
		}
		if (i < SPI_CS_IOPIN_IDX && !Lpc17SSPSignalPin(pCfgData->DevNo, i, pin))
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
	LPC_SC->PCONP |= s_PowerMask[pCfgData->DevNo] | (1UL << 15);
	(void)LPC_SC->PCONP;
	// Do not take over a controller with a pending frame or queued TX data.
	if ((LPCSSP_READ(dev->pSspReg, SR) & (LPCSSP_SR_BSY | LPCSSP_SR_TFE)) != LPCSSP_SR_TFE)
	{
		EnableInterrupt(state);
		return false;
	}
	dev->pSspReg->CR1 = 0;
	if (pCfgData->DevNo == 0)
	{
		LPC_SC->PCLKSEL1 = (LPC_SC->PCLKSEL1 & ~LPC_PCLKSEL1_SSP0_MASK) | LPC_PCLKSEL1_SSP0_DIV1;
	}
	else
	{
		LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & ~LPC_PCLKSEL0_SSP1_MASK) | LPC_PCLKSEL0_SSP1_DIV1;
	}
	NVIC_DisableIRQ(s_Irq[pCfgData->DevNo]);
	dev->pSpiDev = pDev;
	dev->bEnabled = false;
	dev->bSelected = false;
	dev->bError = false;
	pDev->Cfg = *pCfgData;
	pDev->FirstRdData = -1;
	pDev->CurDevCs = -1;
	pDev->DevIntrf.pDevData = dev;
	dev->PClkFreq = SystemCoreClock;
	Lpc17SSPReset(&pDev->DevIntrf);
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
	pDev->DevIntrf.Enable = Lpc17SSPEnable;
	pDev->DevIntrf.GetRate = LpcSSPGetRate;
	pDev->DevIntrf.SetRate = LpcSSPSetRate;
	pDev->DevIntrf.StartRx = LpcSSPStartRx;
	pDev->DevIntrf.RxData = LpcSSPRxData;
	pDev->DevIntrf.StopRx = LpcSSPStopRx;
	pDev->DevIntrf.StartTx = LpcSSPStartTx;
	pDev->DevIntrf.TxData = LpcSSPTxData;
	pDev->DevIntrf.TxSrData = LpcSSPTxData;
	pDev->DevIntrf.StopTx = LpcSSPStopTx;
	pDev->DevIntrf.Reset = Lpc17SSPReset;
	pDev->DevIntrf.PowerOff = Lpc17SSPPowerOff;
	pDev->DevIntrf.GetHandle = LpcSSPGetHandle;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	LpcSSPSetRate(&pDev->DevIntrf, pCfgData->Rate);
	LpcSSPEnable(&pDev->DevIntrf);
	return true;
}
