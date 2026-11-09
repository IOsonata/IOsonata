/**-------------------------------------------------------------------------
@file	spi_esp32.cpp

@brief	GP-SPI2 polling master for ESP32-C2/C3/C5/C6/C61, SPI device 0.

Normal 8-bit SPI supports all four clock modes and both bit orders.
Transfers use the 64-byte hardware buffer. GPIO chip select remains asserted
across chunks and the generic command/read sequence. Manual chip select is
owned by the application. Board pin maps select the GPIO matrix signals.
SCK and MISO may be omitted with port/pin -1 for MOSI-only devices.
Receive operations require MISO; MOSI is always required.
Slave, DMA, interrupt, three-wire and multi-line transfers are not implemented.
The XTAL supplies the clock. Rate selects the closest divider; zero selects
its minimum rate. GPIO matrix timing and board wiring limit usable speed.

@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026

@license

MIT

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
#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "esp32xx_spi.h"

#define SPI_BASE ESP32_SPI2_BASE

typedef struct {
	SPIDev_t *pOwner;
	uint32_t Clock;
	bool bEnabled;
	bool bOpen;
	bool bError;
} Esp32SpiDev_t;

static Esp32SpiDev_t s_Spi;

static Esp32SpiDev_t *Esp32SpiHandle(DevIntrf_t *pIntrf)
{
	if (pIntrf == NULL || s_Spi.pOwner == NULL ||
		pIntrf != &s_Spi.pOwner->DevIntrf || pIntrf->pDevData != &s_Spi)
	{
		return NULL;
	}
	return &s_Spi;
}

static void Esp32SpiClock(bool Enable)
{
	uintptr_t state;

	state = DisableInterrupt();
#if defined(ESP32C2) || defined(ESP32C3)
	if (Enable)
	{
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x10U) |= 1UL << 6;
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x18U) |= 1UL << 6;
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x18U) &= ~(1UL << 6);
	}
	else
	{
		ESP32_SPI_CLK_GATE_REG(SPI_BASE) = 0;
		ESP32_REG32(ESP32_SYSTEM_BASE + 0x10U) &= ~(1UL << 6);
	}
#else
	if (Enable)
	{
		ESP32_REG32(ESP32_PCR_BASE + ESP32_SPI_PCR_CONF_OFFSET) |=
			ESP32_SPI_PCR_CLK_EN_Msk | ESP32_SPI_PCR_RST_EN_Msk;
		ESP32_REG32(ESP32_PCR_BASE + ESP32_SPI_PCR_CONF_OFFSET) &= ~ESP32_SPI_PCR_RST_EN_Msk;
		// Clock source zero selects XTAL on C5/C6/C61.
		ESP32_REG32(ESP32_PCR_BASE + ESP32_SPI_PCR_CLKM_CONF_OFFSET) = ESP32_SPI_PCR_CLKM_EN_Msk;
	}
	else
	{
		ESP32_SPI_CLK_GATE_REG(SPI_BASE) = 0;
		ESP32_REG32(ESP32_PCR_BASE + ESP32_SPI_PCR_CLKM_CONF_OFFSET) = 0;
		ESP32_REG32(ESP32_PCR_BASE + ESP32_SPI_PCR_CONF_OFFSET) &= ~ESP32_SPI_PCR_CLK_EN_Msk;
	}
#endif
	EnableInterrupt(state);
}

static void Esp32SpiConfigure(Esp32SpiDev_t *pDev)
{
	const SPICfg_t *cfg;
	uint32_t user;
	uint32_t misc;
	uint32_t ctrl;

	cfg = &pDev->pOwner->Cfg;
	Esp32SpiClock(true);
#if defined(ESP32C2) || defined(ESP32C3)
	// MST_CLK_SEL = 0 selects XTAL, matching the divider source.
	ESP32_SPI_CLK_GATE_REG(SPI_BASE) = ESP32_SPI_CLK_GATE_CLK_EN_Msk |
		ESP32_SPI_CLK_GATE_MST_CLK_ACTIVE_Msk;
#else
	ESP32_SPI_CLK_GATE_REG(SPI_BASE) = ESP32_SPI_CLK_GATE_CLK_EN_Msk |
		ESP32_SPI_CLK_GATE_MST_CLK_ACTIVE_Msk | ESP32_SPI_CLK_GATE_MST_CLK_SEL_Msk;
#endif
	user = ESP32_SPI_USER_USR_MOSI_Msk;
	if (cfg->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo >= 0)
	{
		user |= ESP32_SPI_USER_DOUTDIN_Msk | ESP32_SPI_USER_USR_MISO_Msk;
	}
	misc = 0x3FU;
	ctrl = 0;
	if (cfg->ClkPol == SPICLKPOL_LOW)
	{
		misc |= ESP32_SPI_MISC_CK_IDLE_EDGE_Msk;
	}
	if ((cfg->ClkPol == SPICLKPOL_LOW) != (cfg->DataPhase == SPIDATAPHASE_SECOND_CLK))
	{
		user |= ESP32_SPI_USER_CK_OUT_EDGE_Msk;
	}
	if (cfg->BitOrder == SPIDATABIT_LSB)
	{
		ctrl = (1UL << ESP32_SPI_CTRL_RD_BIT_ORDER_Pos) | (1UL << ESP32_SPI_CTRL_WR_BIT_ORDER_Pos);
	}
	ESP32_SPI_CTRL_REG(SPI_BASE) = ctrl;
	ESP32_SPI_USER_REG(SPI_BASE) = user;
	ESP32_SPI_USER1_REG(SPI_BASE) = 0;
	ESP32_SPI_USER2_REG(SPI_BASE) = 0;
	ESP32_SPI_MISC_REG(SPI_BASE) = misc;
	ESP32_SPI_SLAVE_REG(SPI_BASE) = 0;
	ESP32_SPI_DMA_CONF_REG(SPI_BASE) = 0;
	ESP32_SPI_DMA_INT_ENA_REG(SPI_BASE) = 0;
	ESP32_SPI_DMA_INT_CLR_REG(SPI_BASE) = UINT32_MAX;
	ESP32_SPI_DIN_MODE_REG(SPI_BASE) = 0;
	ESP32_SPI_DIN_NUM_REG(SPI_BASE) = 0;
	ESP32_SPI_DOUT_MODE_REG(SPI_BASE) = 0;
	ESP32_SPI_CLOCK_REG(SPI_BASE) = pDev->Clock;
}

static uint32_t Esp32SpiGetRate(DevIntrf_t *pIntrf)
{
	Esp32SpiDev_t *dev;

	dev = Esp32SpiHandle(pIntrf);
	return dev ? dev->pOwner->Cfg.Rate : 0;
}

static uint32_t Esp32SpiSetRate(DevIntrf_t *pIntrf, uint32_t Rate)
{
	Esp32SpiDev_t *dev;
	uint32_t source;
	uint32_t pre;
	uint32_t n;
	uint32_t div;
	uint32_t bestDiv;
	uint32_t clock;
	uint64_t target;
	uint64_t error;
	uint64_t bestError;

	dev = Esp32SpiHandle(pIntrf);
	if (dev == NULL)
	{
		return 0;
	}
	if (dev->bOpen)
	{
		return dev->pOwner->Cfg.Rate;
	}
	source = GetHighFreqOscFreq();
	if (source == 0)
	{
		return 0;
	}
	bestDiv = 1;
	bestError = Rate > source ? Rate - source : source - Rate;
	clock = ESP32_SPI_CLOCK_CLK_EQU_SYSCLK_Msk;
	for (pre = 1; pre <= 16U; pre++)
	{
		for (n = 2; n <= 64U; n++)
		{
			div = pre * n;
			target = (uint64_t)Rate * div;
			error = target > source ? target - source : source - target;
			// Compare exact frequency errors before rounding to integer Hz.
			if (error * bestDiv < bestError * div)
			{
				bestDiv = div;
				bestError = error;
				clock = ((pre - 1U) << ESP32_SPI_CLOCK_CLKDIV_PRE_Pos) |
					((n - 1U) << ESP32_SPI_CLOCK_CLKCNT_N_Pos) |
					((n / 2U - 1U) << ESP32_SPI_CLOCK_CLKCNT_H_Pos) | (n - 1U);
			}
		}
	}
	dev->Clock = clock;
	dev->pOwner->Cfg.Rate = (source + bestDiv / 2U) / bestDiv;
	if (dev->bEnabled)
	{
		ESP32_SPI_CLOCK_REG(SPI_BASE) = clock;
	}
	return dev->pOwner->Cfg.Rate;
}

static bool Esp32SpiWait(Esp32SpiDev_t *pDev, uint32_t Mask, int Bytes)
{
	uint64_t budget;
	uint32_t loops;

	// Include wire time and a conservative CPU polling allowance.
	budget = (uint64_t)SystemCoreClockGet() * (Bytes * 8U + 1U) / pDev->pOwner->Cfg.Rate;
	budget += SystemCoreClockGet() / 10U + 1U;
	loops = budget > UINT32_MAX ? UINT32_MAX : (uint32_t)budget;
	while (loops-- > 0)
	{
		if ((ESP32_SPI_CMD_REG(SPI_BASE) & Mask) == 0)
		{
			return true;
		}
	}
	pDev->bError = true;
	return false;
}

static void Esp32SpiStop(DevIntrf_t *pIntrf)
{
	Esp32SpiDev_t *dev;
	const IOPinCfg_t *pin;

	dev = Esp32SpiHandle(pIntrf);
	if (dev == NULL || !dev->bEnabled)
	{
		return;
	}
	if (dev->bError)
	{
		// Abort a timed-out transfer before releasing chip select.
		Esp32SpiConfigure(dev);
	}
	if (dev->bOpen && dev->pOwner->Cfg.ChipSel == SPICSEL_AUTO)
	{
		pin = &dev->pOwner->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + dev->pOwner->CurDevCs];
		IOPinSet(pin->PortNo, pin->PinNo);
	}
	dev->bOpen = false;
	dev->bError = false;
	dev->pOwner->CurDevCs = -1;
	pIntrf->bTxReady = true;
}

static bool Esp32SpiStart(DevIntrf_t *pIntrf, uint32_t DevCs)
{
	Esp32SpiDev_t *dev;
	SPIDev_t *spi;
	const IOPinCfg_t *pin;

	dev = Esp32SpiHandle(pIntrf);
	if (dev == NULL || !dev->bEnabled)
	{
		return false;
	}
	spi = dev->pOwner;
	if ((spi->Cfg.ChipSel == SPICSEL_AUTO && DevCs >= (uint32_t)(spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)) ||
		(spi->Cfg.ChipSel == SPICSEL_MAN && DevCs != 0))
	{
		dev->bError = dev->bOpen;
		return false;
	}
	if (dev->bOpen)
	{
		// Generic Read changes direction without releasing CS or command errors.
		if ((uint32_t)spi->CurDevCs != DevCs)
		{
			dev->bError = true;
		}
		return !dev->bError;
	}
	spi->CurDevCs = DevCs;
	spi->FirstRdData = -1;
	dev->bOpen = true;
	dev->bError = false;
	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + DevCs];
		IOPinClear(pin->PortNo, pin->PinNo);
	}
	return true;
}

static int Esp32SpiTransfer(DevIntrf_t *pIntrf, const uint8_t *pTx, uint8_t *pRx, int Length)
{
	Esp32SpiDev_t *dev;
	int count;
	int chunk;
	int i;
	int j;
	uint32_t word;
	uint8_t value;

	dev = Esp32SpiHandle(pIntrf);
	if (dev == NULL || !dev->bEnabled || !dev->bOpen || dev->bError)
	{
		return 0;
	}
	if (Length <= 0 || (pTx == NULL && pRx == NULL) ||
		(pRx && dev->pOwner->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo < 0))
	{
		dev->bError = true;
		return 0;
	}
	count = 0;
	pIntrf->bTxReady = false;
	while (count < Length)
	{
		chunk = Length - count;
		if (chunk > (int)ESP32_SPI_W_FIFO_BYTES)
		{
			chunk = ESP32_SPI_W_FIFO_BYTES;
		}
		for (i = 0; i < chunk; i += 4)
		{
			word = 0;
			for (j = 0; j < 4 && i + j < chunk; j++)
			{
				value = pTx ? pTx[count + i + j] : dev->pOwner->Cfg.DummyByte;
				word |= (uint32_t)value << (8 * j);
			}
			ESP32_SPI_W_REG(SPI_BASE, i / 4) = word;
		}
		ESP32_SPI_MS_DLEN_REG(SPI_BASE) = chunk * 8U - 1U;
		ESP32_SPI_DMA_INT_CLR_REG(SPI_BASE) = UINT32_MAX;
		ESP32_SPI_CMD_REG(SPI_BASE) = ESP32_SPI_CMD_UPDATE_Msk;
		if (!Esp32SpiWait(dev, ESP32_SPI_CMD_UPDATE_Msk, 0))
		{
			break;
		}
		ESP32_SPI_CMD_REG(SPI_BASE) = ESP32_SPI_CMD_USR_Msk;
		if (!Esp32SpiWait(dev, ESP32_SPI_CMD_USR_Msk, chunk))
		{
			break;
		}
		if (dev->pOwner->FirstRdData < 0 && dev->pOwner->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo >= 0)
		{
			dev->pOwner->FirstRdData = ESP32_SPI_W_REG(SPI_BASE, 0) & 255U;
		}
		if (pRx)
		{
			for (i = 0; i < chunk; i += 4)
			{
				word = ESP32_SPI_W_REG(SPI_BASE, i / 4);
				for (j = 0; j < 4 && i + j < chunk; j++)
				{
					pRx[count + i + j] = word >> (8 * j);
				}
			}
		}
		count += chunk;
	}
	pIntrf->bTxReady = true;
	return count;
}

static int Esp32SpiRxData(DevIntrf_t *pIntrf, uint8_t *pData, int Length)
{
	return Esp32SpiTransfer(pIntrf, NULL, pData, Length);
}

static int Esp32SpiTxData(DevIntrf_t *pIntrf, const uint8_t *pData, int Length)
{
	return Esp32SpiTransfer(pIntrf, pData, NULL, Length);
}

static void Esp32SpiEnable(DevIntrf_t *pIntrf)
{
	Esp32SpiDev_t *dev;
	const IOPinCfg_t *pins;
	int i;

	dev = Esp32SpiHandle(pIntrf);
	if (dev == NULL || dev->bEnabled)
	{
		return;
	}
	Esp32SpiConfigure(dev);
	// Commit the idle clock level before connecting SCK to the pad.
	ESP32_SPI_CMD_REG(SPI_BASE) = ESP32_SPI_CMD_UPDATE_Msk;
	if (!Esp32SpiWait(dev, ESP32_SPI_CMD_UPDATE_Msk, 0))
	{
		Esp32SpiClock(false);
		return;
	}
	pins = dev->pOwner->Cfg.pIOPinMap;
	if (dev->pOwner->Cfg.ChipSel == SPICSEL_AUTO)
	{
		for (i = SPI_CS_IOPIN_IDX; i < dev->pOwner->Cfg.NbIOPins; i++)
		{
			IOPinSet(pins[i].PortNo, pins[i].PinNo);
		}
	}
	IOPinCfg(pins, dev->pOwner->Cfg.NbIOPins);
	dev->bError = false;
	dev->bEnabled = true;
}

static void Esp32SpiDisable(DevIntrf_t *pIntrf)
{
	Esp32SpiDev_t *dev;
	const IOPinCfg_t *pins;
	int i;

	dev = Esp32SpiHandle(pIntrf);
	if (dev == NULL || !dev->bEnabled)
	{
		return;
	}
	Esp32SpiStop(pIntrf);
	pins = dev->pOwner->Cfg.pIOPinMap;
	for (i = 0; i < dev->pOwner->Cfg.NbIOPins; i++)
	{
		if (pins[i].PinNo >= 0)
		{
			IOPinConfig(pins[i].PortNo, pins[i].PinNo, IOPINOP_GPIO, IOPINDIR_INPUT, pins[i].Res, IOPINTYPE_NORMAL);
		}
	}
	Esp32SpiClock(false);
	dev->bEnabled = false;
}

static void Esp32SpiPowerOff(DevIntrf_t *pIntrf)
{
	if (Esp32SpiHandle(pIntrf))
	{
		Esp32SpiDisable(pIntrf);
		s_Spi.pOwner = NULL;
		pIntrf->pDevData = NULL;
		pIntrf->EnCnt = 0;
	}
}

static void *Esp32SpiGetHandle(DevIntrf_t *pIntrf)
{
	Esp32SpiDev_t *dev;

	dev = Esp32SpiHandle(pIntrf);
	return dev ? dev->pOwner : NULL;
}

SPIPHY SPISetPhy(SPIDev_t * const pDev, SPIPHY Phy)
{
	(void)Phy;
	return pDev ? pDev->Cfg.Phy : SPIPHY_NORMAL;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfg)
{
	const int signals[] = {ESP32_PINOP_SPI2_SCK, ESP32_PINOP_SPI2_MISO, ESP32_PINOP_SPI2_MOSI};
	DevIntrf_t *intrf;
	const IOPinCfg_t *pins;
	uintptr_t state;
	uint32_t used;
	uint32_t mask;
	int op;
	int i;

	if (pDev == NULL || pCfg == NULL || pCfg->DevNo != 0 || pCfg->Phy != SPIPHY_NORMAL ||
		pCfg->Mode != SPIMODE_MASTER || pCfg->DataSize != 8 || pCfg->bDmaEn || pCfg->bIntEn ||
		pCfg->MaxRetry < 0 || pCfg->pIOPinMap == NULL || pCfg->NbIOPins < 3 ||
		pCfg->NbIOPins > (int)ESP32_GPIO_PIN_COUNT || GetHighFreqOscFreq() == 0 ||
		(pCfg->ChipSel != SPICSEL_AUTO && pCfg->ChipSel != SPICSEL_MAN) ||
		(pCfg->ChipSel == SPICSEL_AUTO && pCfg->NbIOPins < 4) ||
		(pCfg->ChipSel == SPICSEL_MAN && pCfg->NbIOPins != 3) ||
		(pCfg->BitOrder != SPIDATABIT_MSB && pCfg->BitOrder != SPIDATABIT_LSB) ||
		(pCfg->ClkPol != SPICLKPOL_HIGH && pCfg->ClkPol != SPICLKPOL_LOW) ||
		(pCfg->DataPhase != SPIDATAPHASE_FIRST_CLK && pCfg->DataPhase != SPIDATAPHASE_SECOND_CLK))
	{
		return false;
	}
	pins = pCfg->pIOPinMap;
	used = 0;
	for (i = 0; i < pCfg->NbIOPins; i++)
	{
		if (i < SPI_MOSI_IOPIN_IDX && pins[i].PortNo == -1 && pins[i].PinNo == -1)
		{
			continue;
		}
		op = i < SPI_CS_IOPIN_IDX ? signals[i] : IOPINOP_GPIO;
		if (pins[i].PortNo != 0 || pins[i].PinNo < 0 || pins[i].PinNo > ESP32_GPIO_PIN_MAX ||
			pins[i].Type != IOPINTYPE_NORMAL || pins[i].PinOp != op ||
			pins[i].PinDir != (i == SPI_MISO_IOPIN_IDX ? IOPINDIR_INPUT : IOPINDIR_OUTPUT))
		{
			return false;
		}
		mask = 1UL << pins[i].PinNo;
		if (used & mask)
		{
			return false;
		}
		used |= mask;
	}
	state = DisableInterrupt();
	if (s_Spi.pOwner && (s_Spi.pOwner != pDev || s_Spi.bOpen || pDev->DevIntrf.EnCnt > 1))
	{
		EnableInterrupt(state);
		return false;
	}
	if (s_Spi.pOwner)
	{
		Esp32SpiDisable(&pDev->DevIntrf);
	}
	s_Spi.pOwner = pDev;
	EnableInterrupt(state);
	pDev->Cfg = *pCfg;
	pDev->FirstRdData = -1;
	pDev->CurDevCs = -1;
	pDev->QuadSetMemSize = NULL;
	pDev->QuadSendCmd = NULL;
	intrf = &pDev->DevIntrf;
	intrf->pDevData = &s_Spi;
	intrf->Type = DEVINTRF_TYPE_SPI;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->bDma = false;
	intrf->bIntEn = false;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	atomic_flag_clear(&intrf->bBusy);
	intrf->EnCnt = 1;
	intrf->Enable = Esp32SpiEnable;
	intrf->Disable = Esp32SpiDisable;
	intrf->Reset = Esp32SpiStop;
	intrf->PowerOff = Esp32SpiPowerOff;
	intrf->GetRate = Esp32SpiGetRate;
	intrf->SetRate = Esp32SpiSetRate;
	intrf->StartRx = Esp32SpiStart;
	intrf->StartTx = Esp32SpiStart;
	intrf->StopRx = Esp32SpiStop;
	intrf->StopTx = Esp32SpiStop;
	intrf->RxData = Esp32SpiRxData;
	intrf->TxData = Esp32SpiTxData;
	intrf->TxSrData = Esp32SpiTxData;
	intrf->GetHandle = Esp32SpiGetHandle;
	Esp32SpiSetRate(intrf, pCfg->Rate);
	Esp32SpiEnable(intrf);
	if (!s_Spi.bEnabled)
	{
		Esp32SpiPowerOff(intrf);
		return false;
	}
	return true;
}
