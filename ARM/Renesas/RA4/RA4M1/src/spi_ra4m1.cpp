/**-------------------------------------------------------------------------
@file	spi_ra4m1.cpp

@brief	RA4M1 RSPI0/RSPI1 SPI master and slave implementation.

			DevNo 0 selects RSPI0 and DevNo 1 selects RSPI1. The RSPI clock is
			PCLKA.

			Master: full duplex, 8 to 16 bit frames, modes 0 to 3, MSB or LSB
			first. The master runs in clock synchronous (3 wire) mode and the
			chip select is a GPIO driven by the driver (SPICSEL_AUTO) or by the
			application (SPICSEL_MAN). Transfers are polled. With bIntEn set,
			a TxData or RxData of more than one frame runs from the receive
			interrupt: TX data is copied into a 256 byte driver buffer, the
			call returns -1 and the result is reported with
			DEVINTRF_EVT_COMPLETED. Longer TX data and every TxSrData
			command phase are polled. Rate selection returns the closest
			supported rate.

			Slave: interrupt driven, 8 bit frames, chip select on SSL0 of the
			unit. The RSPI slave does not support CPHA = 0, so only modes 1
			and 3 (SPIDATAPHASE_SECOND_CLK) are accepted and the master must
			release the chip select between frames. The slave needs the next
			TX frame loaded before the master clocks it, so the data of frame
			k + 2 is written when frame k is received and two frames are loaded
			at the start. The SCK rate is set by the master and must leave time
			for one interrupt per frame. There is no DMA: bDmaEn is accepted and
			ignored, DevIntrf_t::bDma reports false.

			Slave frame sequence: DEVINTRF_EVT_STATECHG (set the buffers with
			SetSlaveRxBuffer and SetSlaveTxData, index 0), master frame,
			DEVINTRF_EVT_COMPLETED with the number of bytes received, or -1
			after an error. The frame ends when the RSPI goes idle (SPII),
			which is the chip select returning high. The slave is then
			rearmed from the same interrupt.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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
#include <string.h>

#include "ra4m1xxx.h"

#include "coredev/spi.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "interrupt_ra4m1.h"

#define RA4M1_SPI_COUNT			2
#define RA4M1_SPI_ASYNC_MAX		256		// TX copy buffer for interrupt transfers
#define RA4M1_SPI_ISR_SPIN		256		// bounded wait inside an interrupt
#define RA4M1_SPI_DEFAULT_RATE	1000000U	// used when the requested rate is 0

#define RA4M1_SPI_ERRORS	(R_SPI0_SPSR_OVRF_Msk | R_SPI0_SPSR_MODF_Msk | \
							 R_SPI0_SPSR_PERF_Msk | R_SPI0_SPSR_UDRF_Msk)

// Pin tables, LQFP100 package, function PSEL 6 (IOPINOP_FUNC5).
typedef struct {
	uint8_t Port;
	uint8_t Pin;
} Ra4m1SpiPin_t;

static const Ra4m1SpiPin_t s_Sck[RA4M1_SPI_COUNT][2] = {
	{{4, 12}, {1, 2}},		// RSPCKA P412, P102
	{{1, 11}, {2, 4}},		// RSPCKB P111, P204
};
static const Ra4m1SpiPin_t s_Miso[RA4M1_SPI_COUNT][2] = {
	{{4, 10}, {1, 0}},		// MISOA P410, P100
	{{1, 10}, {2, 2}},		// MISOB P110, P202
};
static const Ra4m1SpiPin_t s_Mosi[RA4M1_SPI_COUNT][2] = {
	{{4, 11}, {1, 1}},		// MOSIA P411, P101
	{{1, 9}, {2, 3}},		// MOSIB P109, P203
};
static const Ra4m1SpiPin_t s_Ssl0[RA4M1_SPI_COUNT][3] = {
	{{4, 13}, {1, 3}, {0, 0}},		// SSLA0 P413, P103
	{{1, 8}, {1, 12}, {2, 5}},		// SSLB0 P108, P112, P205
};

typedef struct {
	R_SPI0_Type *pReg;
	SPIDev_t *pSpiDev;
	uint32_t MstpMask;
	uint8_t EvtId;				// ICU event id of SPRI, SPTI/SPII/SPEI follow
	IRQn_Type IrqRx;
	IRQn_Type IrqIdle;
	IRQn_Type IrqErr;
	uint8_t Bytes;				// bytes per frame, 1 or 2
	volatile bool bRegistered;
	volatile bool bEnabled;
	volatile bool bSelected;
	volatile bool bError;
	volatile bool bAsync;
	// Master transfer
	const uint8_t *pTx;
	uint8_t *pRx;
	int Len;
	volatile int TxCnt;
	volatile int RxCnt;
	// Slave frame
	volatile bool bSlaveReady;
	volatile bool bSlaveError;
	volatile int SlaveCount;
	uint8_t AsyncBuf[RA4M1_SPI_ASYNC_MAX];
} Ra4m1SpiDev_t;

static Ra4m1SpiDev_t s_Ra4m1SpiDev[RA4M1_SPI_COUNT] = {
	{R_SPI0, NULL, (1UL << 19), RA4M1_EVTID_SPI0_SPRI, (IRQn_Type)-1, (IRQn_Type)-1, (IRQn_Type)-1},
	{R_SPI1, NULL, (1UL << 18), RA4M1_EVTID_SPI1_SPRI, (IRQn_Type)-1, (IRQn_Type)-1, (IRQn_Type)-1},
};

static void Ra4m1SpiSlaveArm(Ra4m1SpiDev_t * const pDev);

static Ra4m1SpiDev_t *Ra4m1SpiHandle(DevIntrf_t * const pIntrf)
{
	Ra4m1SpiDev_t *dev = pIntrf != NULL ? (Ra4m1SpiDev_t *)pIntrf->pDevData : NULL;

	if (dev == NULL || dev->pSpiDev == NULL || &dev->pSpiDev->DevIntrf != pIntrf)
	{
		return NULL;
	}

	return dev;
}

static inline bool Ra4m1SpiIsSlave(const Ra4m1SpiDev_t * const pDev)
{
	return pDev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE;
}

static void Ra4m1SpiModuleStart(Ra4m1SpiDev_t * const pDev, bool bStart)
{
	uint16_t prcr = R_SYSTEM->PRCR & 0x0FU;

	R_SYSTEM->PRCR = 0xA500U | prcr | R_SYSTEM_PRCR_PRC1_Msk;

	if (bStart)
	{
		R_MSTP->MSTPCRB &= ~pDev->MstpMask;
	}
	else
	{
		R_MSTP->MSTPCRB |= pDev->MstpMask;
	}

	(void)R_MSTP->MSTPCRB;
	R_SYSTEM->PRCR = 0xA500U | prcr;
}

// ---------------------------------------------------------------------------
// Rate and register helpers
// ---------------------------------------------------------------------------

// fRSPCK = PCLKA / (2 * (SPBR + 1) * 2^BRDV). The closest supported rate is
// returned, a request outside the range selects the nearest end.
static bool Ra4m1SpiRate(uint32_t Rate, uint8_t *pSpbr, uint16_t *pBrdv,
	uint32_t *pActual)
{
	uint32_t clk = SystemPeriphClockGet(0);

	if (clk == 0U)
	{
		return false;
	}

	if (Rate == 0U)
	{
		Rate = RA4M1_SPI_DEFAULT_RATE;
	}

	uint32_t want = (uint32_t)(((uint64_t)clk + Rate / 2U) / Rate);

	if (want < 2U)
	{
		want = 2U;
	}
	else if (want > 4096U)
	{
		want = 4096U;
	}

	for (unsigned brdv = 0; brdv < 4U; brdv++)
	{
		uint32_t step = 2U << brdv;
		uint32_t cnt = (want + step / 2U) / step;

		if (cnt < 1U)
		{
			cnt = 1U;
		}

		if (cnt <= 256U)
		{
			*pSpbr = (uint8_t)(cnt - 1U);
			*pBrdv = (uint16_t)(brdv << R_SPI0_SPCMD_BRDV_Pos);
			*pActual = clk / (step * cnt);

			return *pActual != 0U;
		}
	}

	return false;
}

static inline void Ra4m1SpiPut(const Ra4m1SpiDev_t * const pDev, uint16_t Val)
{
	if (pDev->Bytes == 1U)
	{
		pDev->pReg->SPDR_BY = (uint8_t)Val;
	}
	else
	{
		pDev->pReg->SPDR_HA = Val;
	}
}

static inline uint16_t Ra4m1SpiGet(const Ra4m1SpiDev_t * const pDev)
{
	return pDev->Bytes == 1U ? pDev->pReg->SPDR_BY : pDev->pReg->SPDR_HA;
}

// Master TX frame at byte position Pos. Without TX data the dummy byte is sent.
static inline uint16_t Ra4m1SpiTxFrame(const Ra4m1SpiDev_t * const pDev, int Pos)
{
	uint16_t val = pDev->pSpiDev->Cfg.DummyByte;

	if (pDev->pTx != NULL)
	{
		val = pDev->pTx[Pos];

		if (pDev->Bytes == 2U)
		{
			val |= (uint16_t)((uint16_t)pDev->pTx[Pos + 1] << 8);
		}
	}
	else if (pDev->Bytes == 2U)
	{
		val |= (uint16_t)(val << 8);
	}

	return val;
}

static inline void Ra4m1SpiRxFrame(Ra4m1SpiDev_t * const pDev, int Pos, uint16_t Val)
{
	SPIDev_t *spi = pDev->pSpiDev;

	Val &= (uint16_t)((1UL << spi->Cfg.DataSize) - 1UL);

	if (spi->FirstRdData < 0)
	{
		spi->FirstRdData = Val;
	}

	if (pDev->pRx != NULL)
	{
		pDev->pRx[Pos] = (uint8_t)Val;

		if (pDev->Bytes == 2U)
		{
			pDev->pRx[Pos + 1] = (uint8_t)(Val >> 8);
		}
	}
}

static uint32_t Ra4m1SpiTimeout(const Ra4m1SpiDev_t * const pDev)
{
	uint32_t core = SystemCoreClockGet();
	uint32_t rate = pDev->pSpiDev->Cfg.Rate != 0U ? pDev->pSpiDev->Cfg.Rate : 1U;

	// At least 50 ms plus a complete frame, even at the slowest divisor.
	return core / 20U + (core / rate) * 16U + 256U;
}

static bool Ra4m1SpiWait(Ra4m1SpiDev_t * const pDev, uint8_t Mask, uint8_t Value)
{
	uint32_t n = Ra4m1SpiTimeout(pDev);

	do
	{
		uint8_t status = pDev->pReg->SPSR;

		if (status & RA4M1_SPI_ERRORS)
		{
			break;
		}

		if ((status & Mask) == Value)
		{
			return true;
		}
	} while (n--);

	pDev->bError = true;

	return false;
}

// SPE = 0 resets the transfer engine. The status register is read before the
// flags are cleared, and the receive register is drained. The master keeps
// the interface enabled afterwards.
static void Ra4m1SpiMasterClear(Ra4m1SpiDev_t * const pDev)
{
	R_SPI0_Type *reg = pDev->pReg;

	reg->SPCR &= (uint8_t)~R_SPI0_SPCR_SPE_Msk;
	(void)reg->SPSR;
	reg->SPSR = 0U;

	if (pDev->Bytes == 1U)
	{
		(void)reg->SPDR_BY;
	}
	else
	{
		(void)reg->SPDR_HA;
	}

	if (pDev->bEnabled)
	{
		reg->SPCR |= R_SPI0_SPCR_SPE_Msk;
	}
}

// Program the registers from the configuration with the unit disabled.
static void Ra4m1SpiConfigure(Ra4m1SpiDev_t * const pDev, uint8_t Spbr, uint16_t Brdv)
{
	SPIDev_t *spi = pDev->pSpiDev;
	R_SPI0_Type *reg = pDev->pReg;
	uint16_t cmd = Brdv;

	pDev->Bytes = spi->Cfg.DataSize <= 8U ? 1U : 2U;

	reg->SPCR = 0U;
	(void)reg->SPSR;
	reg->SPSR = 0U;
	reg->SSLP = 0U;
	reg->SPPCR = 0U;
	reg->SPBR = Spbr;
	reg->SPCKD = 0U;
	reg->SSLND = 0U;
	reg->SPND = 0U;
	reg->SPDCR = pDev->Bytes == 1U ? R_SPI0_SPDCR_SPBYT_Msk : 0U;

	cmd |= (uint16_t)((spi->Cfg.DataSize - 1U) << R_SPI0_SPCMD_SPB_Pos);

	if (spi->Cfg.BitOrder == SPIDATABIT_LSB)
	{
		cmd |= R_SPI0_SPCMD_LSBF_Msk;
	}

	if (spi->Cfg.ClkPol == SPICLKPOL_LOW)
	{
		cmd |= R_SPI0_SPCMD_CPOL_Msk;
	}

	if (spi->Cfg.DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cmd |= R_SPI0_SPCMD_CPHA_Msk;
	}

	reg->SPCMD[0] = cmd;

	if (Ra4m1SpiIsSlave(pDev))
	{
		// 4 wire mode, SSL0 frames the transfer. Idle interrupt ends a frame.
		reg->SPCR2 = R_SPI0_SPCR2_SPIIE_Msk;
		reg->SPCR = R_SPI0_SPCR_SPRIE_Msk | R_SPI0_SPCR_SPEIE_Msk;
	}
	else
	{
		// Clock synchronous (3 wire) master, chip select is a GPIO.
		// SCK stops while the receive register is full.
		reg->SPCR2 = R_SPI0_SPCR2_SCKASE_Msk;
		reg->SPCR = R_SPI0_SPCR_SPMS_Msk | R_SPI0_SPCR_MSTR_Msk;
	}
}

// ---------------------------------------------------------------------------
// Master
// ---------------------------------------------------------------------------

static void Ra4m1SpiMasterCancel(Ra4m1SpiDev_t * const pDev)
{
	uint32_t state = DisableInterrupt();

	pDev->pReg->SPCR &= (uint8_t)~(R_SPI0_SPCR_SPRIE_Msk | R_SPI0_SPCR_SPEIE_Msk);

	if (pDev->bAsync)
	{
		DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
		bool rx = pDev->pRx != NULL;

		pDev->bAsync = false;
		pDev->bError = true;
		intrf->bTxReady = true;
		EnableInterrupt(state);

		// The generic stop helpers release the busy flag.
		if (rx)
		{
			DeviceIntrfStopRx(intrf);
		}
		else
		{
			DeviceIntrfStopTx(intrf);
		}

		return;
	}

	EnableInterrupt(state);
}

static void Ra4m1SpiStop(DevIntrf_t * const pIntrf)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL || !dev->bSelected)
	{
		return;
	}

	if (dev->bError || !Ra4m1SpiWait(dev, R_SPI0_SPSR_IDLNF_Msk, 0U))
	{
		Ra4m1SpiMasterClear(dev);
	}

	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		const IOPinCfg_t *pin = &spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi->CurDevCs];

		IOPinSet(pin->PortNo, pin->PinNo);
	}

	dev->bSelected = false;
	dev->bError = false;
	pIntrf->bTxReady = true;
}

static bool Ra4m1SpiStart(DevIntrf_t * const pIntrf, uint32_t DevCs)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL || !dev->bEnabled || Ra4m1SpiIsSlave(dev))
	{
		return false;
	}

	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.ChipSel == SPICSEL_AUTO &&
		DevCs >= (uint32_t)(spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX))
	{
		dev->bError = dev->bSelected;
		return false;
	}

	if (dev->bSelected)
	{
		// DeviceIntrfRead changes direction without releasing the transfer.
		if (spi->Cfg.ChipSel == SPICSEL_AUTO && DevCs != (uint32_t)spi->CurDevCs)
		{
			dev->bError = true;
		}

		return !dev->bError;
	}

	dev->bError = false;

	if (!Ra4m1SpiWait(dev, R_SPI0_SPSR_IDLNF_Msk, 0U))
	{
		Ra4m1SpiMasterClear(dev);
		dev->bError = false;

		return false;
	}

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

// Polled transfer. Returns the number of bytes moved.
static int Ra4m1SpiPoll(Ra4m1SpiDev_t * const pDev, const uint8_t *pTx,
	uint8_t *pRx, int Len)
{
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
	int bytes = pDev->Bytes;
	int count = 0;

	if (!pDev->bSelected || pDev->bError || Len <= 0)
	{
		return 0;
	}

	if (Len % bytes)
	{
		pDev->bError = true;

		return 0;
	}

	pDev->pTx = pTx;
	pDev->pRx = pRx;
	intrf->bTxReady = false;

	while (count < Len)
	{
		if (!Ra4m1SpiWait(pDev, R_SPI0_SPSR_SPTEF_Msk, R_SPI0_SPSR_SPTEF_Msk))
		{
			break;
		}

		Ra4m1SpiPut(pDev, Ra4m1SpiTxFrame(pDev, count));

		if (!Ra4m1SpiWait(pDev, R_SPI0_SPSR_SPRF_Msk, R_SPI0_SPSR_SPRF_Msk))
		{
			break;
		}

		Ra4m1SpiRxFrame(pDev, count, Ra4m1SpiGet(pDev));
		count += bytes;
	}

	if (!pDev->bError)
	{
		(void)Ra4m1SpiWait(pDev, R_SPI0_SPSR_IDLNF_Msk, 0U);
	}

	intrf->bTxReady = true;

	return count;
}

// Complete an interrupt transfer. Runs in the interrupt.
static void Ra4m1SpiMasterFinish(Ra4m1SpiDev_t * const pDev)
{
	R_SPI0_Type *reg = pDev->pReg;
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
	uint8_t *buffer = pDev->pRx;
	bool rx = pDev->pRx != NULL;
	int count = pDev->bError ? -1 : pDev->RxCnt;

	reg->SPCR &= (uint8_t)~(R_SPI0_SPCR_SPRIE_Msk | R_SPI0_SPCR_SPEIE_Msk);

	for (int i = 0; i < RA4M1_SPI_ISR_SPIN &&
		(reg->SPSR & R_SPI0_SPSR_IDLNF_Msk) != 0U; i++)
	{
	}

	pDev->bAsync = false;

	// The generic stop helpers own the busy flag. Release before the callback.
	if (rx)
	{
		DeviceIntrfStopRx(intrf);
	}
	else
	{
		DeviceIntrfTxComplete(intrf);
	}

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, buffer, count);
	}
}

static void Ra4m1SpiMasterService(Ra4m1SpiDev_t * const pDev)
{
	R_SPI0_Type *reg = pDev->pReg;
	int bytes = pDev->Bytes;

	if (reg->SPSR & RA4M1_SPI_ERRORS)
	{
		pDev->bError = true;
	}
	else
	{
		while ((reg->SPSR & R_SPI0_SPSR_SPRF_Msk) && pDev->RxCnt < pDev->Len)
		{
			Ra4m1SpiRxFrame(pDev, pDev->RxCnt, Ra4m1SpiGet(pDev));
			pDev->RxCnt += bytes;

			// Keep two frames in flight.
			while (pDev->TxCnt < pDev->Len &&
				   (pDev->TxCnt - pDev->RxCnt) < 2 * bytes &&
				   (reg->SPSR & R_SPI0_SPSR_SPTEF_Msk))
			{
				Ra4m1SpiPut(pDev, Ra4m1SpiTxFrame(pDev, pDev->TxCnt));
				pDev->TxCnt += bytes;
			}
		}
	}

	if (pDev->bError || pDev->RxCnt >= pDev->Len)
	{
		Ra4m1SpiMasterFinish(pDev);
	}
}

// Start an interrupt transfer. Returns -1, the result arrives with the
// completion event.
static int Ra4m1SpiAsyncBegin(Ra4m1SpiDev_t * const pDev, const uint8_t *pTx,
	uint8_t *pRx, int Len)
{
	R_SPI0_Type *reg = pDev->pReg;
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
	int bytes = pDev->Bytes;

	pDev->pTx = pTx;
	pDev->pRx = pRx;
	pDev->Len = Len;
	pDev->TxCnt = 0;
	pDev->RxCnt = 0;
	pDev->bAsync = true;
	intrf->bTxReady = false;

	// The write of the first frame starts the clock. Hold the interrupt
	// off until two frames are loaded, as the receive interrupt feeds the
	// following frames.
	uint32_t state = DisableInterrupt();

	reg->SPCR |= R_SPI0_SPCR_SPRIE_Msk | R_SPI0_SPCR_SPEIE_Msk;
	Ra4m1SpiPut(pDev, Ra4m1SpiTxFrame(pDev, 0));
	pDev->TxCnt = bytes;

	if (pDev->TxCnt < Len)
	{
		for (int i = 0; i < RA4M1_SPI_ISR_SPIN &&
			(reg->SPSR & R_SPI0_SPSR_SPTEF_Msk) == 0U; i++)
		{
		}

		if (reg->SPSR & R_SPI0_SPSR_SPTEF_Msk)
		{
			Ra4m1SpiPut(pDev, Ra4m1SpiTxFrame(pDev, pDev->TxCnt));
			pDev->TxCnt += bytes;
		}
	}

	EnableInterrupt(state);

	return -1;
}

static int Ra4m1SpiTx(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL || pData == NULL || Len <= 0)
	{
		return 0;
	}

	if (dev->bSelected && !dev->bError && dev->bRegistered &&
		Len > dev->Bytes && Len <= RA4M1_SPI_ASYNC_MAX && (Len % dev->Bytes) == 0)
	{
		// The caller's buffer may go away before the interrupt transfer ends.
		memcpy(dev->AsyncBuf, pData, (size_t)Len);

		return Ra4m1SpiAsyncBegin(dev, dev->AsyncBuf, NULL, Len);
	}

	return Ra4m1SpiPoll(dev, pData, NULL, Len);
}

// Command phase of a read. Always polled, the receive phase follows.
static int Ra4m1SpiTxSr(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL || pData == NULL)
	{
		return 0;
	}

	return Ra4m1SpiPoll(dev, pData, NULL, Len);
}

static int Ra4m1SpiRx(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL || pBuff == NULL || Len <= 0)
	{
		return 0;
	}

	if (dev->bSelected && !dev->bError && dev->bRegistered &&
		Len > dev->Bytes && (Len % dev->Bytes) == 0)
	{
		return Ra4m1SpiAsyncBegin(dev, NULL, pBuff, Len);
	}

	return Ra4m1SpiPoll(dev, NULL, pBuff, Len);
}

// ---------------------------------------------------------------------------
// Slave
// ---------------------------------------------------------------------------

static inline uint8_t Ra4m1SpiSlaveTxByte(const Ra4m1SpiDev_t * const pDev, int Idx)
{
	const SPIDev_t *spi = pDev->pSpiDev;

	if (spi->pTxData[0] != NULL && Idx < spi->TxDataLen[0])
	{
		return spi->pTxData[0][Idx];
	}

	return spi->Cfg.DummyByte;
}

// Store the received byte and write the TX frame two positions ahead.
static void Ra4m1SpiSlaveRx(Ra4m1SpiDev_t * const pDev)
{
	R_SPI0_Type *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	uint8_t data = reg->SPDR_BY;
	int idx = pDev->SlaveCount;

	if (spi->pRxBuff[0] != NULL && idx < spi->RxBuffLen[0])
	{
		spi->pRxBuff[0][idx] = data;
	}
	else
	{
		pDev->bSlaveError = true;
	}

	if (idx < 0x7FFFFFFF)
	{
		pDev->SlaveCount = idx + 1;
	}

	if (reg->SPSR & R_SPI0_SPSR_SPTEF_Msk)
	{
		reg->SPDR_BY = Ra4m1SpiSlaveTxByte(pDev, idx + 2);
	}
}

static void Ra4m1SpiSlaveArm(Ra4m1SpiDev_t * const pDev)
{
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
	R_SPI0_Type *reg = pDev->pReg;

	pDev->bSlaveReady = false;

	// The application supplies the buffers for the coming frame.
	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_STATECHG, NULL, 0);
	}

	pDev->SlaveCount = 0;
	pDev->bSlaveError = false;

	// SPE = 0 flushes the unused TX prefetch of the previous frame.
	reg->SPCR &= (uint8_t)~R_SPI0_SPCR_SPE_Msk;
	(void)reg->SPSR;
	reg->SPSR = 0U;
	(void)reg->SPDR_BY;

	reg->SPCR |= R_SPI0_SPCR_SPE_Msk;
	reg->SPDR_BY = Ra4m1SpiSlaveTxByte(pDev, 0);
	reg->SPDR_BY = Ra4m1SpiSlaveTxByte(pDev, 1);
	pDev->bSlaveReady = true;
}

static void Ra4m1SpiSlaveEnd(Ra4m1SpiDev_t * const pDev)
{
	R_SPI0_Type *reg = pDev->pReg;
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
	SPIDev_t *spi = pDev->pSpiDev;

	// A received byte can still be waiting when the idle event is taken.
	while (reg->SPSR & R_SPI0_SPSR_SPRF_Msk)
	{
		Ra4m1SpiSlaveRx(pDev);
	}

	int count = pDev->bSlaveError ? -1 : pDev->SlaveCount;
	uint8_t *buffer = spi->pRxBuff[0];

	pDev->bSlaveReady = false;

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, buffer, count);
	}

	if (pDev->bEnabled && !pDev->bSlaveReady)
	{
		Ra4m1SpiSlaveArm(pDev);
	}
}

static void Ra4m1SpiSlaveService(Ra4m1SpiDev_t * const pDev, bool bIdle)
{
	R_SPI0_Type *reg = pDev->pReg;
	uint8_t status = reg->SPSR;

	if (!pDev->bSlaveReady)
	{
		// Not armed: discard whatever is pending.
		(void)reg->SPSR;
		reg->SPSR = 0U;

		if (status & R_SPI0_SPSR_SPRF_Msk)
		{
			(void)reg->SPDR_BY;
		}

		return;
	}

	if (status & RA4M1_SPI_ERRORS)
	{
		pDev->bSlaveError = true;
		reg->SPSR = 0U;

		if ((reg->SPCR & R_SPI0_SPCR_SPE_Msk) == 0U)
		{
			// The unit stopped, no idle event will follow.
			Ra4m1SpiSlaveEnd(pDev);

			return;
		}
	}

	if (status & R_SPI0_SPSR_SPRF_Msk)
	{
		Ra4m1SpiSlaveRx(pDev);
	}

	// An idle event with no byte and no error is not a frame.
	if (bIdle && (pDev->SlaveCount > 0 || pDev->bSlaveError))
	{
		Ra4m1SpiSlaveEnd(pDev);
	}
}

// ---------------------------------------------------------------------------
// Interrupt registration
// ---------------------------------------------------------------------------

static void Ra4m1SpiIrqHandler(int IntNo, void *pCtx)
{
	Ra4m1SpiDev_t *dev = (Ra4m1SpiDev_t *)pCtx;

	Ra4m1AcknowledgeInt((IRQn_Type)IntNo);

	if (dev->pSpiDev == NULL)
	{
		return;
	}

	if (Ra4m1SpiIsSlave(dev))
	{
		Ra4m1SpiSlaveService(dev, IntNo == (int)dev->IrqIdle);
	}
	else if (dev->bAsync)
	{
		Ra4m1SpiMasterService(dev);
	}
}

static void Ra4m1SpiUnregister(Ra4m1SpiDev_t * const pDev)
{
	Ra4m1UnregisterIntHandler(pDev->IrqRx);
	Ra4m1UnregisterIntHandler(pDev->IrqIdle);
	Ra4m1UnregisterIntHandler(pDev->IrqErr);
	pDev->IrqRx = (IRQn_Type)-1;
	pDev->IrqIdle = (IRQn_Type)-1;
	pDev->IrqErr = (IRQn_Type)-1;
	pDev->bRegistered = false;
}

// RSPI events: SPRI, SPTI, SPII, SPEI, SPTEND in consecutive ids.
static bool Ra4m1SpiRegister(Ra4m1SpiDev_t * const pDev, int Priority, bool bSlave)
{
	pDev->IrqRx = Ra4m1RegisterIntHandler(pDev->EvtId, Priority,
		Ra4m1SpiIrqHandler, pDev);

	if (pDev->IrqRx >= 0 && bSlave)
	{
		pDev->IrqIdle = Ra4m1RegisterIntHandler((uint8_t)(pDev->EvtId + 2U),
			Priority, Ra4m1SpiIrqHandler, pDev);
	}

	if (pDev->IrqRx >= 0 && (!bSlave || pDev->IrqIdle >= 0))
	{
		pDev->IrqErr = Ra4m1RegisterIntHandler((uint8_t)(pDev->EvtId + 3U),
			Priority, Ra4m1SpiIrqHandler, pDev);
	}

	if (pDev->IrqErr < 0)
	{
		Ra4m1SpiUnregister(pDev);

		return false;
	}

	pDev->bRegistered = true;

	return true;
}

// ---------------------------------------------------------------------------
// DeviceIntrf control
// ---------------------------------------------------------------------------

static uint32_t Ra4m1SpiGetRate(DevIntrf_t * const pIntrf)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	return dev != NULL ? dev->pSpiDev->Cfg.Rate : 0U;
}

static uint32_t Ra4m1SpiSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);
	uint8_t spbr = 0U;
	uint16_t brdv = 0U;
	uint32_t actual = 0U;

	if (dev == NULL || dev->bSelected || dev->bAsync)
	{
		return 0U;
	}

	if (Ra4m1SpiIsSlave(dev))
	{
		// The master sets the SCK rate.
		return dev->pSpiDev->Cfg.Rate;
	}

	if (!Ra4m1SpiRate(Rate, &spbr, &brdv, &actual))
	{
		return 0U;
	}

	R_SPI0_Type *reg = dev->pReg;
	uint8_t control = reg->SPCR;

	reg->SPCR = control & (uint8_t)~R_SPI0_SPCR_SPE_Msk;
	reg->SPBR = spbr;
	reg->SPCMD[0] = (uint16_t)((reg->SPCMD[0] & ~R_SPI0_SPCMD_BRDV_Msk) | brdv);
	reg->SPCR = control;
	dev->pSpiDev->Cfg.Rate = actual;

	return actual;
}

static void Ra4m1SpiDisable(DevIntrf_t * const pIntrf)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	if (Ra4m1SpiIsSlave(dev))
	{
		dev->bSlaveReady = false;
	}
	else
	{
		Ra4m1SpiMasterCancel(dev);
		Ra4m1SpiStop(pIntrf);
	}

	dev->pReg->SPCR &= (uint8_t)~(R_SPI0_SPCR_SPE_Msk | R_SPI0_SPCR_SPRIE_Msk |
		R_SPI0_SPCR_SPEIE_Msk);
	dev->bEnabled = false;
}

static void Ra4m1SpiEnable(DevIntrf_t * const pIntrf)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	dev->bEnabled = true;

	if (Ra4m1SpiIsSlave(dev))
	{
		dev->pReg->SPCR |= R_SPI0_SPCR_SPRIE_Msk | R_SPI0_SPCR_SPEIE_Msk;
		uint32_t state = DisableInterrupt();
		Ra4m1SpiSlaveArm(dev);
		EnableInterrupt(state);
	}
	else
	{
		Ra4m1SpiMasterClear(dev);
	}
}

static void Ra4m1SpiReset(DevIntrf_t * const pIntrf)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	if (Ra4m1SpiIsSlave(dev))
	{
		if (dev->bEnabled)
		{
			uint32_t state = DisableInterrupt();
			Ra4m1SpiSlaveArm(dev);
			EnableInterrupt(state);
		}
	}
	else
	{
		Ra4m1SpiMasterCancel(dev);
		Ra4m1SpiStop(pIntrf);
		Ra4m1SpiMasterClear(dev);
	}
}

static void Ra4m1SpiPowerOff(DevIntrf_t * const pIntrf)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

	Ra4m1SpiDisable(pIntrf);

	if (dev->bRegistered)
	{
		Ra4m1SpiUnregister(dev);
	}

	IOPinDis(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);

	uint32_t state = DisableInterrupt();

	Ra4m1SpiModuleStart(dev, false);
	dev->pSpiDev = NULL;
	pIntrf->pDevData = NULL;
	pIntrf->EnCnt = 0;
	EnableInterrupt(state);
}

static void *Ra4m1SpiGetHandle(DevIntrf_t * const pIntrf)
{
	Ra4m1SpiDev_t *dev = Ra4m1SpiHandle(pIntrf);

	return dev != NULL ? dev->pSpiDev : NULL;
}

SPIPHY SPISetPhy(SPIDev_t * const pDev, SPIPHY Phy)
{
	// This target supports only the normal, separate MOSI/MISO bus.
	(void)Phy;

	return pDev != NULL ? pDev->Cfg.Phy : SPIPHY_NORMAL;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

static bool Ra4m1SpiPinIn(const Ra4m1SpiPin_t *pTable, int Nb, const IOPinCfg_t * const pPin)
{
	if (pPin->PinOp != IOPINOP_FUNC5)
	{
		return false;
	}

	for (int i = 0; i < Nb; i++)
	{
		if (pTable[i].Port == pPin->PortNo && pTable[i].Pin == pPin->PinNo)
		{
			return true;
		}
	}

	return false;
}

static bool Ra4m1SpiPinMap(int DevNo, const SPICfg_t * const pCfg)
{
	const IOPinCfg_t *map = pCfg->pIOPinMap;
	bool slave = pCfg->Mode == SPIMODE_SLAVE;

	if (!Ra4m1SpiPinIn(s_Sck[DevNo], 2, &map[SPI_SCK_IOPIN_IDX]) ||
		!Ra4m1SpiPinIn(s_Miso[DevNo], 2, &map[SPI_MISO_IOPIN_IDX]) ||
		!Ra4m1SpiPinIn(s_Mosi[DevNo], 2, &map[SPI_MOSI_IOPIN_IDX]))
	{
		return false;
	}

	if (slave)
	{
		// The slave select input is SSL0 of the unit.
		if (!Ra4m1SpiPinIn(s_Ssl0[DevNo], DevNo == 0 ? 2 : 3, &map[SPI_CS_IOPIN_IDX]))
		{
			return false;
		}
	}
	else if (pCfg->ChipSel == SPICSEL_AUTO)
	{
		// Master chip selects are GPIO outputs.
		for (int i = SPI_CS_IOPIN_IDX; i < pCfg->NbIOPins; i++)
		{
			if (map[i].PinOp != IOPINOP_GPIO || map[i].PinDir != IOPINDIR_OUTPUT)
			{
				return false;
			}
		}
	}

	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		if (!Ra4m1PinValid(map[i].PortNo, map[i].PinNo))
		{
			return false;
		}

		for (int j = 0; j < i; j++)
		{
			if (map[i].PortNo == map[j].PortNo && map[i].PinNo == map[j].PinNo)
			{
				return false;
			}
		}
	}

	return true;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL ||
		pCfg->DevNo < 0 || pCfg->DevNo >= RA4M1_SPI_COUNT ||
		(pCfg->Mode != SPIMODE_MASTER && pCfg->Mode != SPIMODE_SLAVE) ||
		pCfg->Phy != SPIPHY_NORMAL ||
		pCfg->DataSize < 8U || pCfg->DataSize > 16U ||
		(pCfg->ChipSel != SPICSEL_AUTO && pCfg->ChipSel != SPICSEL_MAN) ||
		(pCfg->BitOrder != SPIDATABIT_MSB && pCfg->BitOrder != SPIDATABIT_LSB) ||
		(pCfg->ClkPol != SPICLKPOL_HIGH && pCfg->ClkPol != SPICLKPOL_LOW) ||
		(pCfg->DataPhase != SPIDATAPHASE_FIRST_CLK &&
		 pCfg->DataPhase != SPIDATAPHASE_SECOND_CLK) ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins < 3 || pCfg->MaxRetry < 0)
	{
		return false;
	}

	bool slave = pCfg->Mode == SPIMODE_SLAVE;

	if (slave)
	{
		// RSPI slave: interrupt driven, 8 bit frames, CPHA = 1 only,
		// SSL0 as the select input.
		if (!pCfg->bIntEn || pCfg->DataSize != 8U || pCfg->NbIOPins < 4 ||
			pCfg->DataPhase != SPIDATAPHASE_SECOND_CLK)
		{
			return false;
		}
	}
	else if (pCfg->ChipSel == SPICSEL_AUTO && pCfg->NbIOPins < 4)
	{
		return false;
	}

	if (!Ra4m1SpiPinMap(pCfg->DevNo, pCfg))
	{
		return false;
	}

	uint8_t spbr = 0U;
	uint16_t brdv = 0U;
	uint32_t actual = pCfg->Rate;

	if (!slave && !Ra4m1SpiRate(pCfg->Rate, &spbr, &brdv, &actual))
	{
		return false;
	}

	Ra4m1SpiDev_t *dev = &s_Ra4m1SpiDev[pCfg->DevNo];
	uint32_t state = DisableInterrupt();

	if ((dev->pSpiDev != NULL && dev->pSpiDev != pDev) || dev->bSelected ||
		dev->bAsync || s_Ra4m1SpiDev[1 - pCfg->DevNo].pSpiDev == pDev)
	{
		EnableInterrupt(state);

		return false;
	}

	if (dev->pSpiDev == pDev)
	{
		// Initialized again without PowerOff: stop the unit and release
		// its interrupt slots before the new configuration is applied.
		dev->pReg->SPCR = 0U;
		dev->bSlaveReady = false;

		if (dev->bRegistered)
		{
			Ra4m1SpiUnregister(dev);
		}
	}

	dev->pSpiDev = pDev;
	dev->bEnabled = true;
	dev->bSelected = false;
	dev->bError = false;
	dev->bAsync = false;
	dev->bSlaveReady = false;
	dev->bSlaveError = false;
	dev->SlaveCount = 0;
	Ra4m1SpiModuleStart(dev, true);
	EnableInterrupt(state);

	pDev->Cfg = *pCfg;
	pDev->Cfg.Rate = actual;
	pDev->FirstRdData = -1;
	pDev->CurDevCs = 0;

	for (int i = 0; i < SPI_SLAVEMODE_MAX_DEV; i++)
	{
		pDev->pRxBuff[i] = NULL;
		pDev->RxBuffLen[i] = 0;
		pDev->pTxData[i] = NULL;
		pDev->TxDataLen[i] = 0;
	}

	// Chip selects idle high before they become outputs.
	if (!slave && pCfg->ChipSel == SPICSEL_AUTO)
	{
		for (int i = SPI_CS_IOPIN_IDX; i < pCfg->NbIOPins; i++)
		{
			IOPinSet(pCfg->pIOPinMap[i].PortNo, pCfg->pIOPinMap[i].PinNo);
		}
	}

	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);

	Ra4m1SpiConfigure(dev, spbr, brdv);

	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = dev;
	intrf->Type = DEVINTRF_TYPE_SPI;
	intrf->EnCnt = 1;
	intrf->bDma = false;
	intrf->bIntEn = pCfg->bIntEn;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = Ra4m1SpiEnable;
	intrf->Disable = Ra4m1SpiDisable;
	intrf->PowerOff = Ra4m1SpiPowerOff;
	intrf->Reset = Ra4m1SpiReset;
	intrf->GetHandle = Ra4m1SpiGetHandle;
	intrf->GetRate = Ra4m1SpiGetRate;
	intrf->SetRate = Ra4m1SpiSetRate;
	intrf->StartRx = Ra4m1SpiStart;
	intrf->StartTx = Ra4m1SpiStart;
	intrf->StopRx = Ra4m1SpiStop;
	intrf->StopTx = Ra4m1SpiStop;
	intrf->RxData = Ra4m1SpiRx;
	intrf->TxData = Ra4m1SpiTx;
	intrf->TxSrData = Ra4m1SpiTxSr;
	atomic_flag_clear(&intrf->bBusy);

	if (pCfg->bIntEn && !Ra4m1SpiRegister(dev, pCfg->IntPrio, slave))
	{
		dev->pReg->SPCR = 0U;
		IOPinDis(pCfg->pIOPinMap, pCfg->NbIOPins);
		Ra4m1SpiModuleStart(dev, false);
		dev->pSpiDev = NULL;
		intrf->pDevData = NULL;
		intrf->EnCnt = 0;

		return false;
	}

	if (slave)
	{
		state = DisableInterrupt();
		Ra4m1SpiSlaveArm(dev);
		EnableInterrupt(state);
	}
	else
	{
		Ra4m1SpiMasterClear(dev);
	}

	return true;
}
