/**-------------------------------------------------------------------------
@file	spi_r9a02.cpp

@brief	R9A02G021 SPI master on the serial array units (simplified SPI)

R9A02G021 User's Manual R01UH1036EJ0110, 23.5. One SAU channel per SPI, in
master transmission and reception mode (23.5.3):

	DevNo 0 : SPI00, SAU0 channel 0, SCK00, SI00, SO00
	DevNo 1 : SPI01, SAU0 channel 1, SCK01, SI01, SO01
	DevNo 2 : SPI10, SAU0 channel 2, SCK10, SI10, SO10
	DevNo 3 : SPI11, SAU0 channel 3, SCK11, SI11, SO11
	DevNo 4 : SPI20, SAU1 channel 0, SCK20, SI20, SO20
	DevNo 5 : SPI21, SAU1 channel 1, SCK21, SI21, SO21

An SPI and a UART cannot use the same channel: SPI00 and SPI01 are the
channels of UART0, SPI10 and SPI11 those of UART1, SPI20 and SPI21 those of
UART2. The channels are claimed through sau_r9a02.cpp, SPIInit fails when the
UART has them. Which SPIs have pins depends on the package (tables 23.1 to
23.3).

Master, polling, 7 or 8 bit frames, MSB or LSB first, the four clock modes,
chip selects on GPIO pins driven by the driver (SPICSEL_AUTO) or by the
application (SPICSEL_MAN). No slave mode, no 3-wire mode, no interrupt or DMA
transfer, these fail SPIInit.

Each byte is written to SDRmn, the transfer end event (SAUm_ENDIn, single
transfer mode, SMRmn.MD0 = 0) is routed to an IELSR without a CPU interrupt
and its IR flag is polled, then the received byte is read from SDRmn. The SSR
flags alone cannot tell a byte waiting to start from a received one in this
mode (23.3.11).

Transfer clock = fMCK / (2 x (STCLK + 1)), at most PCLKB / 4, PCLKB / 2 for
SPI00 (23.5). SetRate sets the closest clock and returns it.

Clock modes, DCP[1:0] of SCRmn (figure 23.3):

	CPOL 0, CPHA 0 : type 4, DCP 11b
	CPOL 0, CPHA 1 : type 2, DCP 01b
	CPOL 1, CPHA 0 : type 3, DCP 10b
	CPOL 1, CPHA 1 : type 1, DCP 00b

Pins: PinOp R9A02_PSEL_SAU (IOPINOP_FUNC16) for SCK, MISO (SI) and MOSI
(SO), SCK and MOSI IOPINDIR_OUTPUT and IOPINTYPE_NORMAL, MISO IOPINDIR_INPUT:
IOPinConfig then sets NCODR = 1 on the SI input (16.5.1). Chip selects are
GPIO outputs. Other pin settings fail SPIInit.

SPI00 can run at PCLKB / 2, but its serial clock cycle is at least 62.5 ns
(16 MHz) from VCC 4.0 V and 83.3 ns (12 MHz) from VCC 2.7 V in high-speed
mode (table 37.32), the other SPIs per table 37.33: the application keeps
the rate within the limit of its supply voltage.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdint.h>
#include <stddef.h>

#include "r9a02g021.h"
#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "sau_r9a02.h"

#pragma pack(push, 4)
typedef struct __R9a02_Spi_Dev {
	uint8_t Unit;					// SAU unit
	uint8_t Ch;						// SAU channel
	uint8_t EvtEnd;					// Transfer end event
	uint8_t PclkDiv;				// Highest transfer clock is PCLKB / PclkDiv
	SPIDev_t *pSpiDev;				// Generic SPI device data
	int IntNo;						// Polled IELSR route, -1 when none
	uint32_t ByteLoops;				// Poll loops allowed for one byte
	uint16_t DataMask;				// 0x7F with 7 bit frames
	bool bEnabled;
	bool bSelected;
	bool bError;
} R9a02SpiDev_t;
#pragma pack(pop)

static R9a02SpiDev_t s_R9a02SpiDev[] = {
	{ 0, 0, R9A02_EVT_SAU0_ENDI0, 2, NULL, -1, 0, 0xFF, false, false, false },
	{ 0, 1, R9A02_EVT_SAU0_ENDI1, 4, NULL, -1, 0, 0xFF, false, false, false },
	{ 0, 2, R9A02_EVT_SAU0_ENDI2, 4, NULL, -1, 0, 0xFF, false, false, false },
	{ 0, 3, R9A02_EVT_SAU0_ENDI3, 4, NULL, -1, 0, 0xFF, false, false, false },
	{ 1, 0, R9A02_EVT_SAU1_ENDI0, 4, NULL, -1, 0, 0xFF, false, false, false },
	{ 1, 1, R9A02_EVT_SAU1_ENDI1, 4, NULL, -1, 0, 0xFF, false, false, false },
};

static const int s_NbR9a02SpiDev = sizeof(s_R9a02SpiDev) / sizeof(s_R9a02SpiDev[0]);

static R9a02SpiDev_t *R9a02SpiHandle(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = pIntrf ? (R9a02SpiDev_t *)pIntrf->pDevData : NULL;

	return dev && dev->pSpiDev && &dev->pSpiDev->DevIntrf == pIntrf ? dev : NULL;
}

// Poll loops for one byte: the byte time in CPU cycles, one or more cycles
// per loop, plus a margin
static void R9a02SpiSetTimeout(R9a02SpiDev_t * const pDev)
{
	uint32_t rate = pDev->pSpiDev->Cfg.Rate ? pDev->pSpiDev->Cfg.Rate : 1U;

	pDev->ByteLoops = (SystemCoreClock / rate + 1U) * 8U + 1000U;
}

// Channel stopped, output disabled, SO and SCK at their idle levels
static void R9a02SpiStop(R9a02SpiDev_t * const pDev)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint16_t bit = (uint16_t)(1U << pDev->Ch);

	reg->ST = bit;
	reg->SOE = (uint16_t)(reg->SOE & ~bit);
}

// Initial output levels and output enable (table 23.38, steps 6 and 7). The
// serial clock output starts at its idle level: CKO = 1 with DCP[0] = 0, 0
// with DCP[0] = 1 (table 23.35).
static void R9a02SpiOutputs(R9a02SpiDev_t * const pDev)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint16_t bit = (uint16_t)(1U << pDev->Ch);
	uint16_t cko = R9A02_SAU_SO_CKO(pDev->Ch);
	uint16_t so = (uint16_t)(reg->SO | bit);

	if (reg->SCR[pDev->Ch] & R9A02_SAU_SCR_DCP_CLK_REV)
	{
		so &= (uint16_t)~cko;
	}
	else
	{
		so |= cko;
	}

	reg->SIR[pDev->Ch] = R9A02_SAU_SIR_OVCT;
	reg->SO = so;
	reg->SOE = (uint16_t)(reg->SOE | bit);
}

// Channel start (table 23.38, step 9), the transfer end flag cleared
static void R9a02SpiRun(R9a02SpiDev_t * const pDev)
{
	R9a02SauReg(pDev->Unit)->SS = (uint16_t)(1U << pDev->Ch);
	if (pDev->IntNo >= 0)
	{
		R9a02EventFlagClear(pDev->IntNo);
	}
}

static void R9a02SpiStart(R9a02SpiDev_t * const pDev)
{
	R9a02SpiOutputs(pDev);
	R9a02SpiRun(pDev);
}

static uint32_t R9a02SpiGetRate(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	return dev ? dev->pSpiDev->Cfg.Rate : 0;
}

static uint32_t R9a02SpiSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL || dev->bSelected || Rate == 0U)
	{
		return 0;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);
	uint16_t bit = (uint16_t)(1U << dev->Ch);
	R9a02SauClk_t clk;
	uintptr_t state = DisableInterrupt();

	// SPS, SMR and STCLK are written with the channel stopped
	R9a02SpiStop(dev);

	uint32_t best = R9a02SauSetRate(dev->Unit, bit, Rate, 1U, SystemPeriphClockGet(0) / dev->PclkDiv, &clk);

	if (best != 0U)
	{
		reg->SMR[dev->Ch] = (uint16_t)((reg->SMR[dev->Ch] & ~R9A02_SAU_SMR_CKS) | clk.Cks);
		reg->SDR[dev->Ch] = clk.Sdr;
		dev->pSpiDev->Cfg.Rate = best;
		R9a02SpiSetTimeout(dev);
	}

	if (dev->bEnabled)
	{
		R9a02SpiStart(dev);
	}

	EnableInterrupt(state);

	return best;
}

// A channel left with an error or a byte that did not end is restarted
static void R9a02SpiRecover(R9a02SpiDev_t * const pDev)
{
	uintptr_t state = DisableInterrupt();

	R9a02SpiStop(pDev);
	if (pDev->bEnabled)
	{
		R9a02SpiStart(pDev);
	}

	EnableInterrupt(state);
}

static void R9a02SpiStopXfer(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL || !dev->bSelected)
	{
		return;
	}

	SPIDev_t *spi = dev->pSpiDev;
	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);
	uint32_t n = dev->ByteLoops;

	while ((reg->SSR[dev->Ch] & R9A02_SAU_SSR_TSF) && --n > 0U)
	{
	}

	// Chip select released before a restart, which can move SCK to its idle
	// level
	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi->CurDevCs];

		IOPinSet(pin->PortNo, pin->PinNo);
	}

	if (dev->bError || n == 0U)
	{
		R9a02SpiRecover(dev);
	}

	dev->bSelected = false;
	dev->bError = false;
	pIntrf->bTxReady = true;
}

static bool R9a02SpiStartXfer(DevIntrf_t * const pIntrf, uint32_t DevCs)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL || !dev->bEnabled)
	{
		return false;
	}

	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.ChipSel == SPICSEL_AUTO && DevCs >= (uint32_t)(spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX))
	{
		dev->bError = dev->bSelected;

		return false;
	}

	if (dev->bSelected)
	{
		// DeviceIntrfRead changes direction without releasing the transfer
		if (spi->Cfg.ChipSel == SPICSEL_AUTO && DevCs != (uint32_t)spi->CurDevCs)
		{
			dev->bError = true;
		}

		return !dev->bError;
	}

	dev->bError = false;
	spi->CurDevCs = (int)DevCs;
	spi->FirstRdData = -1;
	dev->bSelected = true;

	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + DevCs];

		IOPinClear(pin->PortNo, pin->PinNo);
	}

	return true;
}

// One byte out and one in per frame. pTx NULL sends the dummy byte, pRx NULL
// drops the received bytes.
static int R9a02SpiTransfer(DevIntrf_t * const pIntrf, const uint8_t *pTx, uint8_t *pRx, int Len)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL || !dev->bSelected || dev->bError || Len <= 0)
	{
		return 0;
	}

	SPIDev_t *spi = dev->pSpiDev;
	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);
	uint32_t ch = dev->Ch;
	int cnt = 0;

	pIntrf->bTxReady = false;

	while (cnt < Len)
	{
		uint16_t d = pTx ? pTx[cnt] : spi->Cfg.DummyByte;
		uint32_t n = dev->ByteLoops;

		R9a02EventFlagClear(dev->IntNo);

		// During operation only DAT is written, STCLK is kept (23.3.8)
		reg->SDR[ch] = (uint16_t)(d & dev->DataMask);

		while (!R9a02EventFlag(dev->IntNo) && --n > 0U)
		{
		}

		if (n == 0U)
		{
			dev->bError = true;
			break;
		}

		uint16_t ssr = reg->SSR[ch];

		d = (uint16_t)(reg->SDR[ch] & dev->DataMask);

		if (ssr & R9A02_SAU_SSR_OVF)
		{
			reg->SIR[ch] = R9A02_SAU_SIR_OVCT;
			dev->bError = true;
			break;
		}

		if (spi->FirstRdData < 0)
		{
			spi->FirstRdData = d;
		}
		if (pRx)
		{
			pRx[cnt] = (uint8_t)d;
		}
		cnt++;
	}

	pIntrf->bTxReady = true;

	return cnt;
}

static int R9a02SpiTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	return pData ? R9a02SpiTransfer(pIntrf, pData, NULL, Len) : 0;
}

static int R9a02SpiRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	return pBuff ? R9a02SpiTransfer(pIntrf, NULL, pBuff, Len) : 0;
}

static void R9a02SpiDisable(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	R9a02SpiStopXfer(pIntrf);

	uintptr_t state = DisableInterrupt();

	R9a02SpiStop(dev);
	dev->bEnabled = false;

	EnableInterrupt(state);
}

static void R9a02SpiEnable(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();

	dev->bEnabled = true;
	R9a02SpiStop(dev);
	R9a02SpiStart(dev);

	EnableInterrupt(state);
}

static void R9a02SpiReset(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev != NULL)
	{
		R9a02SpiStopXfer(pIntrf);
		R9a02SpiRecover(dev);
	}
}

// Channel stopped and released, route removed, pins disconnected. A new
// SPIInit is needed to use it again.
static void R9a02SpiPowerOff(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	R9a02SpiDisable(pIntrf);

	uintptr_t state = DisableInterrupt();
	SPIDev_t *spi = dev->pSpiDev;

	R9a02UnregisterIntHandler(dev->IntNo);
	dev->IntNo = -1;
	IOPinDis(spi->Cfg.pIOPinMap, spi->Cfg.NbIOPins);
	R9a02SauRelease(dev->Unit, (uint16_t)(1U << dev->Ch), dev);
	dev->pSpiDev = NULL;
	pIntrf->pDevData = NULL;
	pIntrf->EnCnt = 0;

	EnableInterrupt(state);
}

static void *R9a02SpiGetHandle(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	return dev ? dev->pSpiDev : NULL;
}

// The simplified SPI has separate SI and SO lines only
SPIPHY SPISetPhy(SPIDev_t * const pDev, SPIPHY Phy)
{
	return pDev ? pDev->Cfg.Phy : SPIPHY_NORMAL;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo < 0 || pCfg->DevNo >= s_NbR9a02SpiDev ||
		pCfg->Mode != SPIMODE_MASTER || pCfg->Phy != SPIPHY_NORMAL || pCfg->bIntEn || pCfg->bDmaEn ||
		(pCfg->DataSize != 7 && pCfg->DataSize != 8) || pCfg->Rate == 0U ||
		(pCfg->ChipSel != SPICSEL_AUTO && pCfg->ChipSel != SPICSEL_MAN) ||
		(pCfg->BitOrder != SPIDATABIT_MSB && pCfg->BitOrder != SPIDATABIT_LSB) ||
		(pCfg->ClkPol != SPICLKPOL_HIGH && pCfg->ClkPol != SPICLKPOL_LOW) ||
		(pCfg->DataPhase != SPIDATAPHASE_FIRST_CLK && pCfg->DataPhase != SPIDATAPHASE_SECOND_CLK) ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins < SPI_CS_IOPIN_IDX || pCfg->MaxRetry < 0 ||
		(pCfg->ChipSel == SPICSEL_AUTO && pCfg->NbIOPins <= SPI_CS_IOPIN_IDX))
	{
		return false;
	}

	// SCK and SO CMOS outputs, SI an input, NCODR = 1 then (16.5.1)
	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		const IOPinCfg_t *pin = &pCfg->pIOPinMap[i];
		IOPINDIR dir = i == SPI_MISO_IOPIN_IDX ? IOPINDIR_INPUT : IOPINDIR_OUTPUT;

		if (!R9a02PinValid(pin->PortNo, pin->PinNo) || pin->PinDir != dir ||
			(i < SPI_CS_IOPIN_IDX && (pin->PinOp != R9A02_PSEL_SAU || pin->Type != IOPINTYPE_NORMAL)) ||
			(i >= SPI_CS_IOPIN_IDX && pin->PinOp != IOPINOP_GPIO))
		{
			return false;
		}
		for (int j = 0; j < i; j++)
		{
			if (pin->PortNo == pCfg->pIOPinMap[j].PortNo && pin->PinNo == pCfg->pIOPinMap[j].PinNo)
			{
				return false;
			}
		}
	}

	R9a02SpiDev_t *dev = &s_R9a02SpiDev[pCfg->DevNo];
	uint16_t bit = (uint16_t)(1U << dev->Ch);
	uintptr_t state = DisableInterrupt();

	// An SPI object is not used for two instances
	for (int i = 0; i < s_NbR9a02SpiDev; i++)
	{
		if (i != pCfg->DevNo && s_R9a02SpiDev[i].pSpiDev == pDev)
		{
			EnableInterrupt(state);

			return false;
		}
	}

	// An instance in use by another SPI object is not taken over. The claim
	// fails when the UART has the channel, it also starts the SAU module
	// clock, register writes before it are ignored.
	if ((dev->pSpiDev != NULL && dev->pSpiDev != pDev) || dev->bSelected ||
		!R9a02SauClaim(dev->Unit, bit, dev))
	{
		EnableInterrupt(state);

		return false;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);

	R9a02SpiStop(dev);

	if (dev->IntNo < 0)
	{
		dev->IntNo = R9a02RouteEvent(dev->EvtEnd);
	}
	if (dev->IntNo < 0)
	{
		R9a02SauRelease(dev->Unit, bit, dev);
		EnableInterrupt(state);

		return false;
	}

	dev->pSpiDev = pDev;
	dev->bEnabled = false;
	dev->bSelected = false;
	dev->bError = false;
	dev->DataMask = pCfg->DataSize == 7 ? 0x7FU : 0xFFU;

	pDev->Cfg = *pCfg;
	pDev->FirstRdData = -1;
	pDev->CurDevCs = 0;

	// Mode and format, written with the channel stopped (tables 23.32 and
	// 23.33). The operation clock, SMR.CKS, is set by R9a02SpiSetRate.
	uint16_t scr = (uint16_t)(R9A02_SAU_SCR_TRXE_TXRX |
							  (pCfg->DataSize == 7 ? R9A02_SAU_SCR_DLS_7BIT : R9A02_SAU_SCR_DLS_8BIT));

	if (pCfg->BitOrder == SPIDATABIT_LSB)
	{
		scr |= R9A02_SAU_SCR_DIR_LSB;
	}
	if (pCfg->DataPhase == SPIDATAPHASE_FIRST_CLK)
	{
		scr |= R9A02_SAU_SCR_DCP_DATA_EARLY;
	}
	if (pCfg->ClkPol == SPICLKPOL_HIGH)
	{
		scr |= R9A02_SAU_SCR_DCP_CLK_REV;
	}

	reg->SMR[dev->Ch] = (uint16_t)(R9A02_SAU_SMR_FIXED | R9A02_SAU_SMR_MD_SPI);
	reg->SCR[dev->Ch] = scr;

	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = dev;

	if (R9a02SpiSetRate(intrf, pCfg->Rate) == 0U)
	{
		R9a02UnregisterIntHandler(dev->IntNo);
		dev->IntNo = -1;
		dev->pSpiDev = NULL;
		intrf->pDevData = NULL;
		R9a02SauRelease(dev->Unit, bit, dev);
		EnableInterrupt(state);

		return false;
	}

	// Chip selects inactive before their pins become outputs
	for (int i = SPI_CS_IOPIN_IDX; i < pCfg->NbIOPins; i++)
	{
		IOPinSet(pCfg->pIOPinMap[i].PortNo, pCfg->pIOPinMap[i].PinNo);
	}

	intrf->Type = DEVINTRF_TYPE_SPI;
	intrf->bDma = false;
	intrf->bIntEn = false;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = R9a02SpiEnable;
	intrf->Disable = R9a02SpiDisable;
	intrf->PowerOff = R9a02SpiPowerOff;
	intrf->Reset = R9a02SpiReset;
	intrf->GetHandle = R9a02SpiGetHandle;
	intrf->GetRate = R9a02SpiGetRate;
	intrf->SetRate = R9a02SpiSetRate;
	intrf->StartRx = R9a02SpiStartXfer;
	intrf->StartTx = R9a02SpiStartXfer;
	intrf->StopRx = R9a02SpiStopXfer;
	intrf->StopTx = R9a02SpiStopXfer;
	intrf->RxData = R9a02SpiRxData;
	intrf->TxData = R9a02SpiTxData;
	intrf->TxSrData = R9a02SpiTxData;
	intrf->EnCnt = 1;
	atomic_flag_clear(&intrf->bBusy);

	// Output levels and output enable before the pins are connected, then
	// the channel starts (table 23.38)
	dev->bEnabled = true;
	R9a02SpiOutputs(dev);
	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	R9a02SpiRun(dev);

	EnableInterrupt(state);

	return true;
}
