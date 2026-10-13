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

Master, 7 or 8 bit frames, MSB or LSB first, the four clock modes, chip
selects on GPIO pins driven by the driver (SPICSEL_AUTO) or by the
application (SPICSEL_MAN). No slave mode and no 3-wire mode, these fail
SPIInit.

Each byte is written to SDRmn, the transfer end event (SAUm_ENDIn, single
transfer mode, SMRmn.MD0 = 0) marks its end, then the received byte is read
from SDRmn. The SSR flags alone cannot tell a byte waiting to start from a
received one in this mode (23.3.11).

Transfer modes:

	bIntEn and bDmaEn false: polling. SAUm_ENDIn is routed to an IELSR
	without a CPU interrupt and its IR flag is polled.

	bIntEn: RxData starts the reception and returns -1. The SAUm_ENDIn
	interrupt stores each byte and sends the next dummy byte. At the end the
	chip select and the interface are released (DeviceIntrfStopRx), then the
	event handler gets DEVINTRF_EVT_COMPLETED with the buffer and the count.
	Reset cancels a reception in progress, without event.

	bDmaEn: the DTC receives (dtc_r9a02.h). For each SAUm_ENDIn it reads
	the byte from SDRmn (byte access, as the DTC example of 14.6.1) and
	writes the next dummy byte (16-bit access, 23.3.8), Len - 1 times. The
	CPU reads the last byte: the DTC end comes when that byte starts. With
	bIntEn too this is done in the interrupt and RxData returns -1 as above,
	without it RxData waits and returns the count. One byte is received as
	without bDmaEn.

	Transmission is polled in every mode, TxData returns when the bytes are
	sent and does not keep the caller buffer (DeviceIntrfWrite passes a
	copy on the stack). The DTC would have to write SDRmn by bytes, which
	the manual does not describe. With bIntEn, except for the command of a
	read (bNoStop), TxData then ends as a background transfer: chip select
	released, DEVINTRF_EVT_COMPLETED, and -1 returned. Every Tx, Rx and
	Read then ends with one event.

The interrupt runs at IntPrio. Its CLIC input is on only while a reception
runs in the background, the polled parts use the IR flag.

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
#include "dtc_r9a02.h"

#pragma pack(push, 4)
typedef struct __R9a02_Spi_Dev {
	R9a02DtcInfo_t Dtc[2];			// Reception records: byte read, dummy write
	uint8_t Unit;					// SAU unit
	uint8_t Ch;						// SAU channel
	uint8_t EvtEnd;					// Transfer end event
	uint8_t PclkDiv;				// Highest transfer clock is PCLKB / PclkDiv
	SPIDev_t *pSpiDev;				// Generic SPI device data
	int IntNo;						// IELSR route, -1 when none
	uint32_t ByteLoops;				// Poll loops allowed for one byte
	uint32_t StartLoops;			// Poll loops in which a byte written to SDRmn starts
	uint8_t *pRxBuf;				// Reception in progress
	int RxLen;
	volatile int RxCnt;
	uint16_t DataMask;				// 0x7F with 7 bit frames
	uint16_t Dummy;					// Dummy byte written by the DTC
	volatile bool bAsync;			// Reception in the background
	bool bDtcXfer;					// Reception by the DTC
	bool bEnabled;
	bool bSelected;
	bool bError;
} R9a02SpiDev_t;
#pragma pack(pop)

static R9a02SpiDev_t s_R9a02SpiDev[] = {
	{ {}, 0, 0, R9A02_EVT_SAU0_ENDI0, 2, NULL, -1, 0, 0, NULL, 0, 0, 0xFF, 0xFF, false, false, false, false, false },
	{ {}, 0, 1, R9A02_EVT_SAU0_ENDI1, 4, NULL, -1, 0, 0, NULL, 0, 0, 0xFF, 0xFF, false, false, false, false, false },
	{ {}, 0, 2, R9A02_EVT_SAU0_ENDI2, 4, NULL, -1, 0, 0, NULL, 0, 0, 0xFF, 0xFF, false, false, false, false, false },
	{ {}, 0, 3, R9A02_EVT_SAU0_ENDI3, 4, NULL, -1, 0, 0, NULL, 0, 0, 0xFF, 0xFF, false, false, false, false, false },
	{ {}, 1, 0, R9A02_EVT_SAU1_ENDI0, 4, NULL, -1, 0, 0, NULL, 0, 0, 0xFF, 0xFF, false, false, false, false, false },
	{ {}, 1, 1, R9A02_EVT_SAU1_ENDI1, 4, NULL, -1, 0, 0, NULL, 0, 0, 0xFF, 0xFF, false, false, false, false, false },
};

static const int s_NbR9a02SpiDev = sizeof(s_R9a02SpiDev) / sizeof(s_R9a02SpiDev[0]);

static R9a02SpiDev_t *R9a02SpiHandle(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = pIntrf ? (R9a02SpiDev_t *)pIntrf->pDevData : NULL;

	return dev && dev->pSpiDev && &dev->pSpiDev->DevIntrf == pIntrf ? dev : NULL;
}

// Poll loops for one byte: the byte time in CPU cycles, one or more cycles
// per loop, plus a margin. A byte written to SDRmn starts within a few fMCK
// cycles, one transfer clock period is allowed.
static void R9a02SpiSetTimeout(R9a02SpiDev_t * const pDev, uint32_t FmckHz)
{
	uint32_t rate = pDev->pSpiDev->Cfg.Rate ? pDev->pSpiDev->Cfg.Rate : 1U;

	pDev->ByteLoops = (SystemCoreClock / rate + 1U) * 8U + 1000U;
	pDev->StartLoops = SystemCoreClock / rate + (FmckHz ? 4U * (SystemCoreClock / FmckHz) : 0U) + 16U;
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
		R9a02SpiSetTimeout(dev, clk.FmckHz);
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

// Background reception stopped without event, the route kept. Returns true
// when one was running.
static bool R9a02SpiCancel(R9a02SpiDev_t * const pDev)
{
	uintptr_t state = DisableInterrupt();
	bool running = pDev->bAsync;

	if (pDev->IntNo >= 0 && (running || pDev->bDtcXfer))
	{
		R9a02IntEnable(pDev->IntNo, false);
		if (pDev->bDtcXfer)
		{
			R9a02DtcDisable(pDev->IntNo);
		}
		R9a02ClearPendingInt(pDev->IntNo);

		// A byte can be on the line, the channel is restarted at the stop
		pDev->bError = true;
	}
	pDev->bAsync = false;
	pDev->bDtcXfer = false;

	EnableInterrupt(state);

	return running;
}

static void R9a02SpiStopXfer(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL || !dev->bSelected)
	{
		return;
	}

	R9a02SpiCancel(dev);

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

// Polled in every mode. With bIntEn, except for the command of a read, it
// ends as a background transfer: the interface released, the event, then
// -1, so that every Tx ends with one DEVINTRF_EVT_COMPLETED.
static int R9a02SpiTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL || pData == NULL || Len <= 0 || !dev->bSelected || dev->bError || dev->bAsync)
	{
		return 0;
	}

	int cnt = R9a02SpiTransfer(pIntrf, pData, NULL, Len);

	if (!pIntrf->bIntEn || pIntrf->bNoStop)
	{
		return cnt;
	}

	DeviceIntrfTxComplete(pIntrf);

	if (pIntrf->EvtCB != NULL)
	{
		pIntrf->EvtCB(pIntrf, DEVINTRF_EVT_COMPLETED, NULL, cnt);
	}

	return -1;
}

// Last byte of a DTC reception. The last DTC transfer wrote its dummy byte,
// the byte is received when TSF is 0 and BFF is 1 after its start, or when
// no start is seen in the time a start takes: it was over before. TSF and
// BFF read the same before the start and after the end (23.3.11).
static void R9a02SpiDtcLast(R9a02SpiDev_t * const pDev)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint32_t ch = pDev->Ch;
	uint32_t n = pDev->ByteLoops;
	uint32_t start = pDev->StartLoops;
	bool started = false;
	bool done = false;

	while (!done && --n > 0U)
	{
		uint16_t ssr = reg->SSR[ch];

		if ((ssr & R9A02_SAU_SSR_TSF) || (ssr & R9A02_SAU_SSR_BFF) == 0U)
		{
			started = true;
		}
		else if (started || --start == 0U)
		{
			done = true;
		}
	}

	uint16_t ssr = reg->SSR[ch];
	uint8_t d = (uint8_t)(reg->SDR[ch] & pDev->DataMask);

	pDev->RxCnt = pDev->RxLen - 1;

	if (ssr & R9A02_SAU_SSR_OVF)
	{
		reg->SIR[ch] = R9A02_SAU_SIR_OVCT;
		pDev->bError = true;
	}
	else if (!done)
	{
		pDev->bError = true;
	}
	else
	{
		pDev->pRxBuf[pDev->RxLen - 1] = d;
		pDev->RxCnt = pDev->RxLen;
	}

	// The DTC copied SDRmn [7:0], the 8th bit of 7 bit frames is removed
	if (pDev->DataMask != 0xFFU)
	{
		for (int i = 0; i < pDev->RxCnt; i++)
		{
			pDev->pRxBuf[i] &= (uint8_t)pDev->DataMask;
		}
	}

	// The event of the last byte
	R9a02ClearPendingInt(pDev->IntNo);
}

// End of a background reception: the interface is released, then the event
static void R9a02SpiRxEnd(R9a02SpiDev_t * const pDev)
{
	SPIDev_t *spi = pDev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	uint8_t *buf = pDev->pRxBuf;
	int cnt = pDev->RxCnt;

	R9a02IntEnable(pDev->IntNo, false);
	pDev->bAsync = false;
	pDev->bDtcXfer = false;

	if (cnt > 0 && spi->FirstRdData < 0)
	{
		spi->FirstRdData = buf[0];
	}

	DeviceIntrfStopRx(intrf);

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, buf, cnt);
	}
}

// SAUm_ENDIn: one byte received, or the DTC end
static void R9a02SpiIrqHandler(int IntNo, void *pCtx)
{
	R9a02SpiDev_t *dev = (R9a02SpiDev_t *)pCtx;

	if (!dev->bAsync || dev->pSpiDev == NULL)
	{
		return;
	}

	if (dev->bDtcXfer)
	{
		R9a02IntEnable(IntNo, false);
		R9a02SpiDtcLast(dev);
		R9a02SpiRxEnd(dev);

		return;
	}

	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);
	uint32_t ch = dev->Ch;
	uint16_t ssr = reg->SSR[ch];
	uint8_t d = (uint8_t)(reg->SDR[ch] & dev->DataMask);

	if (ssr & R9A02_SAU_SSR_OVF)
	{
		reg->SIR[ch] = R9A02_SAU_SIR_OVCT;
		dev->bError = true;
		R9a02SpiRxEnd(dev);

		return;
	}

	dev->pRxBuf[dev->RxCnt] = d;
	dev->RxCnt = dev->RxCnt + 1;

	if (dev->RxCnt < dev->RxLen)
	{
		reg->SDR[ch] = dev->Dummy;

		return;
	}

	R9a02SpiRxEnd(dev);
}

// DTC records: SDRmn [7:0] to the buffer, then the dummy byte to SDRmn
// (chain), Len - 1 times. The last transfer ends the chain (14.4.6).
static bool R9a02SpiDtcSetup(R9a02SpiDev_t * const pDev, uint8_t *pBuff, int Len)
{
	R9A02_SAU_Type *reg = R9a02SauReg(pDev->Unit);
	uint32_t sdr = (uint32_t)(uintptr_t)&reg->SDR[pDev->Ch];
	uint16_t cnt = (uint16_t)(Len - 1);			// 0 is 65536
	R9a02DtcInfo_t *d = pDev->Dtc;

	d[0].Rsv = 0U;
	d[0].Mra = R9A02_DTC_MRA_MD_NORMAL | R9A02_DTC_MRA_SZ_BYTE | R9A02_DTC_MRA_SM_FIXED;
	d[0].Mrb = R9A02_DTC_MRB_CHNE | R9A02_DTC_MRB_DM_INC;
	d[0].Sar = sdr;
	d[0].Dar = (uint32_t)(uintptr_t)pBuff;
	d[0].Crb = 0U;
	d[0].Cra = cnt;

	d[1].Rsv = 0U;
	d[1].Mra = R9A02_DTC_MRA_MD_NORMAL | R9A02_DTC_MRA_SZ_HALF | R9A02_DTC_MRA_SM_FIXED;
	d[1].Mrb = R9A02_DTC_MRB_DM_FIXED;
	d[1].Sar = (uint32_t)(uintptr_t)&pDev->Dummy;
	d[1].Dar = sdr;
	d[1].Crb = 0U;
	d[1].Cra = cnt;

	return R9a02DtcSet(pDev->IntNo, d);
}

static int R9a02SpiRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL || pBuff == NULL || Len <= 0)
	{
		return 0;
	}

	// The DTC counts up to 65536 transfers, one byte is taken by the CPU
	bool dtc = pIntrf->bDma && Len >= 2 && Len - 1 <= 0x10000;

	if (!dtc && !pIntrf->bIntEn)
	{
		return R9a02SpiTransfer(pIntrf, NULL, pBuff, Len);
	}

	if (!dev->bSelected || dev->bError || dev->bAsync)
	{
		return 0;
	}

	SPIDev_t *spi = dev->pSpiDev;
	R9A02_SAU_Type *reg = R9a02SauReg(dev->Unit);

	dev->pRxBuf = pBuff;
	dev->RxLen = Len;
	dev->RxCnt = 0;
	dev->Dummy = (uint16_t)(spi->Cfg.DummyByte & dev->DataMask);

	if (dtc && !R9a02SpiDtcSetup(dev, pBuff, Len))
	{
		return 0;
	}

	uintptr_t state = DisableInterrupt();

	dev->bDtcXfer = dtc;
	dev->bAsync = pIntrf->bIntEn;

	if (dtc)
	{
		R9a02DtcEnable(dev->IntNo);
	}
	else
	{
		R9a02EventFlagClear(dev->IntNo);
	}
	if (pIntrf->bIntEn)
	{
		R9a02IntEnable(dev->IntNo, true);
	}

	// First byte by the CPU, the others follow its end
	reg->SDR[dev->Ch] = dev->Dummy;

	EnableInterrupt(state);

	if (pIntrf->bIntEn)
	{
		return -1;
	}

	// DTC reception, the caller waits for its end, DTCE back at 0
	uint64_t loops = (uint64_t)dev->ByteLoops * (uint32_t)Len;
	uint32_t n = loops > 0xFFFFFFFFULL ? 0xFFFFFFFFU : (uint32_t)loops;

	while (R9a02DtcEnabled(dev->IntNo) && --n > 0U)
	{
	}

	if (n == 0U)
	{
		R9a02SpiCancel(dev);

		return 0;
	}

	R9a02SpiDtcLast(dev);
	dev->bDtcXfer = false;

	if (dev->RxCnt > 0 && spi->FirstRdData < 0)
	{
		spi->FirstRdData = pBuff[0];
	}

	return dev->RxCnt;
}

static void R9a02SpiDisable(DevIntrf_t * const pIntrf)
{
	R9a02SpiDev_t *dev = R9a02SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	// A background reception stopped here releases the interface
	bool running = R9a02SpiCancel(dev);

	R9a02SpiStopXfer(pIntrf);
	if (running)
	{
		atomic_flag_clear(&pIntrf->bBusy);
	}

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
		// A background reception is dropped without event and the interface
		// released, as after a timeout
		bool running = R9a02SpiCancel(dev);

		R9a02SpiStopXfer(pIntrf);
		R9a02SpiRecover(dev);
		if (running)
		{
			atomic_flag_clear(&pIntrf->bBusy);
		}
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

	R9a02DtcClear(dev->IntNo);
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
		pCfg->Mode != SPIMODE_MASTER || pCfg->Phy != SPIPHY_NORMAL ||
		(pCfg->bIntEn && (pCfg->IntPrio < 0 || pCfg->IntPrio > (int)R9A02_CLIC_PRIO_MAX)) ||
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

	// Route for the mode: a handler with bIntEn, its CLIC input on only for a
	// background reception, the IR flag polled otherwise
	if (dev->IntNo >= 0)
	{
		R9a02DtcClear(dev->IntNo);
		R9a02UnregisterIntHandler(dev->IntNo);
		dev->IntNo = -1;
	}
	if (pCfg->bIntEn)
	{
		dev->IntNo = R9a02RegisterIntHandler(dev->EvtEnd, pCfg->IntPrio, R9a02SpiIrqHandler, dev);
		R9a02IntEnable(dev->IntNo, false);
	}
	else
	{
		dev->IntNo = R9a02RouteEvent(dev->EvtEnd);
	}
	if (dev->IntNo < 0)
	{
		dev->pSpiDev = NULL;
		pDev->DevIntrf.pDevData = NULL;
		pDev->DevIntrf.EnCnt = 0;
		R9a02SauRelease(dev->Unit, bit, dev);
		EnableInterrupt(state);

		return false;
	}

	dev->pSpiDev = pDev;
	dev->bEnabled = false;
	dev->bSelected = false;
	dev->bError = false;
	dev->bAsync = false;
	dev->bDtcXfer = false;
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

	// Noise filter of the channel pair off for the simplified SPI (23.3.27),
	// the UART of the pair cannot run with this channel claimed
	R9A02_PORGA_SNFEN = (uint8_t)(R9A02_PORGA_SNFEN & ~(1U << ((dev->Unit * 4U + dev->Ch) & ~1U)));

	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = dev;

	if (R9a02SpiSetRate(intrf, pCfg->Rate) == 0U)
	{
		R9a02UnregisterIntHandler(dev->IntNo);
		dev->IntNo = -1;
		dev->pSpiDev = NULL;
		intrf->pDevData = NULL;
		intrf->EnCnt = 0;
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
	intrf->bDma = pCfg->bDmaEn;
	intrf->bIntEn = pCfg->bIntEn;
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
