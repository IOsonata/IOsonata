/**-------------------------------------------------------------------------
@file	spi_sam4l.cpp

@brief	SAM4L SPI master and interrupt/PDCA slave implementation.

The SAM4L has one dedicated SPI controller. This port implements the
IOsonata polling, interrupt and PDCA master paths using GPIO chip selects.
The hardware peripheral-select field is kept on CSR0 so every device attached
to one IOsonata SPI object uses the same configured transfer format and rate.
With INT enabled, RX and TX up to 256 bytes return -1 and complete by callback.
Larger TX and Read command phases finish synchronously to preserve TX ownership.
PDCA moves bounded halfword chunks; INT selects asynchronous completion.
Slave mode supports 8-bit interrupt/PDCA transfers, framed by NPCS0.
Slave PDCA uses byte buffers up to 65535 bytes; larger frames use interrupts.
STATECHG supplies buffers before clocks start; COMPLETED reports RX at NSS rising.
Slave buffers remain owned by the application until COMPLETED. Update buffers
in STATECHG, or Reset while NSS is high to apply foreground buffer changes.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <limits.h>

#include "sam4lxxx.h"
#include "component/component_pm.h"
#include "component/component_pdca.h"
#include "component/component_spi.h"

#include "coredev/spi.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"

#define SAM4L_SPI_WAIT_COUNT	100000U
#define SAM4L_SPI_PCS0		0x0EU
#define SAM4L_SPI_DMA_WORDS	64
#define SAM4L_SPI_TX_SIZE		256
#define SAM4L_SPI_ERRORS		(SPI_SR_MODF | SPI_SR_OVRES)

// UART owns channels 0..3; I2C owns 4..11 (master/slave share per DevNo).
#define SAM4L_SPI_RX_CHAN		12
#define SAM4L_SPI_TX_CHAN		13

typedef struct __Sam4l_Spi_Dev
{
	Spi *pReg;
	SPIDev_t *pSpiDev;
	const uint8_t *pTx;
	uint8_t *pRx;
	int Length;
	int Count;
	int Step;
	int ChunkWords;
	bool Active;
	bool Async;
	bool DmaInitialized;
	bool SlaveReady;
	bool SlaveError;
	bool SlaveDmaFrame;
	bool SlaveRxDma;
	int SlaveTxLength;
	uint8_t TxBuffer[SAM4L_SPI_TX_SIZE];
	uint16_t DmaTx[SAM4L_SPI_DMA_WORDS];
	uint16_t DmaRx[SAM4L_SPI_DMA_WORDS];
} Sam4lSpiDev_t;

static Sam4lSpiDev_t s_SpiDev = {
	.pReg = SAM4L_SPI,
	.pSpiDev = nullptr,
};

static void Sam4lSpiCancel(Sam4lSpiDev_t *dev);
static void Sam4lSpiSlaveArm(Sam4lSpiDev_t *dev);

static PdcaChannel *Sam4lSpiRxChannel(void)
{
	return &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_SPI_RX_CHAN];
}

static PdcaChannel *Sam4lSpiTxChannel(void)
{
	return &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_SPI_TX_CHAN];
}

static void Sam4lSpiDmaStop(Sam4lSpiDev_t *dev)
{
	if (!dev->DmaInitialized)
		return;
	Sam4lSpiRxChannel()->PDCA_IDR = 0xFFFFFFFFU;
	Sam4lSpiTxChannel()->PDCA_IDR = 0xFFFFFFFFU;
	Sam4lSpiRxChannel()->PDCA_CR = PDCA_CR_TDIS;
	Sam4lSpiTxChannel()->PDCA_CR = PDCA_CR_TDIS;
	// SAM4L erratum: allow two cycles after stopping PDCA before SPI disable.
	__NOP();
	__NOP();
}

static inline void Sam4lSpiPmWrite(volatile uint32_t *pReg, uint32_t Value)
{
	const uint32_t offset = static_cast<uint32_t>(
		reinterpret_cast<uintptr_t>(pReg) - reinterpret_cast<uintptr_t>(SAM4L_PM));
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAU) | PM_UNLOCK_ADDR(offset);
	*pReg = Value;
}

static inline void Sam4lSpiClockEnable(void)
{
	Sam4lSpiPmWrite(&SAM4L_PM->PM_PBAMASK,
		SAM4L_PM->PM_PBAMASK | PM_PBAMASK_SPI);
}

static inline void Sam4lSpiClockDisable(void)
{
	Sam4lSpiPmWrite(&SAM4L_PM->PM_PBAMASK,
		SAM4L_PM->PM_PBAMASK & ~PM_PBAMASK_SPI);
}

static bool Sam4lSpiWait(Spi *pReg, uint32_t Mask)
{
	uint32_t timeout = SAM4L_SPI_WAIT_COUNT;
	do
	{
		if ((pReg->SPI_SR & Mask) == Mask)
			return true;
	} while (--timeout != 0U);
	return false;
}

static uint32_t Sam4lSpiGetRate(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	return dev != nullptr && dev->pSpiDev != nullptr ? dev->pSpiDev->Cfg.Rate : 0U;
}

static uint32_t Sam4lSpiSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	if (dev == nullptr || dev->pSpiDev == nullptr || Rate == 0U)
		return 0U;
	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
		return dev->pSpiDev->Cfg.Rate; // The external master owns SCK.

	const uint32_t pba = SystemPeriphClockGet(0);
	if (pba == 0U)
		return 0U;

	uint32_t div = pba / Rate;
	if ((pba % Rate) != 0U)
		++div;
	if (div == 0U)
		div = 1U;
	else if (div > 255U)
		div = 255U;

	uint32_t csr = dev->pReg->SPI_CSR[0] & ~SPI_CSR_SCBR_Msk;
	csr |= SPI_CSR_SCBR(div);

	// Keep every CSR identical. Besides making all software chip-selects use
	// one format, this avoids the SAM4L SCBR=1/mixed-divider erratum.
	for (unsigned i = 0U; i < 4U; ++i)
		dev->pReg->SPI_CSR[i] = csr;

	dev->pSpiDev->Cfg.Rate = pba / div;
	return dev->pSpiDev->Cfg.Rate;
}

static void Sam4lSpiConfigure(Sam4lSpiDev_t *dev)
{
	Spi *reg = dev->pReg;
	const SPICfg_t &cfg = dev->pSpiDev->Cfg;

	reg->SPI_IDR = 0xFFFFFFFFU;
	Sam4lSpiDmaStop(dev);
	// Slave-disable erratum: read pending receive data before software reset.
	if (cfg.Mode == SPIMODE_SLAVE)
		for (unsigned i = 0; i < 4U && (reg->SPI_SR & SPI_SR_RDRF) != 0U; ++i)
			(void)reg->SPI_RDR;
	reg->SPI_CR = SPI_CR_SPIDIS;
	reg->SPI_CR = SPI_CR_SWRST;

	// Fixed peripheral select keeps CSR0 selected for halfword PDCA writes.
	// Physical slave selection is handled by the IOsonata GPIO CS array.
	reg->SPI_MR = cfg.Mode == SPIMODE_MASTER ?
		SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PCS(SAM4L_SPI_PCS0) : 0U;

	uint32_t csr = SPI_CSR_BITS(cfg.DataSize - 8U);
	if (cfg.ClkPol == SPICLKPOL_LOW)
		csr |= SPI_CSR_CPOL;
	// SAM4L NCPHA is the inverse of the conventional CPHA encoding:
	// NCPHA=1 captures on the leading edge (generic FIRST_CLK).
	if (cfg.DataPhase == SPIDATAPHASE_FIRST_CLK)
		csr |= SPI_CSR_NCPHA;

	for (unsigned i = 0U; i < 4U; ++i)
		reg->SPI_CSR[i] = csr;

	if (cfg.Mode == SPIMODE_MASTER)
		(void)Sam4lSpiSetRate(&dev->pSpiDev->DevIntrf, cfg.Rate);

	// Drop any stale receive characters before enabling a new session.
	if ((reg->SPI_SR & SPI_SR_RDRF) != 0U)
		(void)reg->SPI_RDR;

	reg->SPI_CR = SPI_CR_SPIEN;
}

static void Sam4lSpiDisable(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	Sam4lSpiCancel(dev);
	if (dev->pSpiDev->Cfg.Mode == SPIMODE_MASTER)
		(void)Sam4lSpiWait(dev->pReg, SPI_SR_TXEMPTY);
	dev->pReg->SPI_CR = SPI_CR_SPIDIS;
	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		for (unsigned i = 0; i < 4U && (dev->pReg->SPI_SR & SPI_SR_RDRF) != 0U; ++i)
			(void)dev->pReg->SPI_RDR;
		dev->pReg->SPI_CR = SPI_CR_SWRST;
	}
	Sam4lSpiClockDisable();
}

static void Sam4lSpiEnable(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	Sam4lSpiClockEnable();
	dev->pReg->SPI_CR = SPI_CR_SPIEN;
	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		Sam4lSpiConfigure(dev);
		Sam4lSpiSlaveArm(dev);
	}
}

static void Sam4lSpiReset(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	Sam4lSpiClockEnable();
	Sam4lSpiCancel(dev);
	Sam4lSpiConfigure(dev);
	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
		Sam4lSpiSlaveArm(dev);
}

static void Sam4lSpiPowerOff(DevIntrf_t * const pDev)
{
	Sam4lSpiDisable(pDev);
}

static bool Sam4lSpiSelect(DevIntrf_t * const pDev, uint32_t DevCs)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.Mode == SPIMODE_SLAVE)
		return false;
	if (spi->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	const uint32_t count = static_cast<uint32_t>(spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX);
	if (DevCs >= count)
		return false;

	spi->CurDevCs = static_cast<int>(DevCs);
	const IOPinCfg_t &pin = spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + DevCs];
	IOPinClear(pin.PortNo, pin.PinNo);
	return true;
}

static void Sam4lSpiDeselect(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	SPIDev_t *spi = dev->pSpiDev;

	if (spi->Cfg.Mode == SPIMODE_SLAVE)
		return;
	(void)Sam4lSpiWait(dev->pReg, SPI_SR_TXEMPTY);
	if (spi->Cfg.ChipSel != SPICSEL_MAN && spi->CurDevCs >= 0)
	{
		const IOPinCfg_t &pin =
			spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi->CurDevCs];
		IOPinSet(pin.PortNo, pin.PinNo);
	}
	spi->CurDevCs = -1;
}

static inline bool Sam4lSpiStartRx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	return Sam4lSpiSelect(pDev, DevCs);
}

static inline void Sam4lSpiStopRx(DevIntrf_t * const pDev)
{
	Sam4lSpiDeselect(pDev);
}

static inline bool Sam4lSpiStartTx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	return Sam4lSpiSelect(pDev, DevCs);
}

static inline void Sam4lSpiStopTx(DevIntrf_t * const pDev)
{
	Sam4lSpiDeselect(pDev);
}

static bool Sam4lSpiTransferWord(Sam4lSpiDev_t *dev, uint16_t Tx, uint16_t *pRx)
{
	Spi *reg = dev->pReg;
	if (!Sam4lSpiWait(reg, SPI_SR_TDRE))
		return false;

	reg->SPI_TDR = SPI_TDR_TD(Tx) | SPI_TDR_PCS(SAM4L_SPI_PCS0);
	if (!Sam4lSpiWait(reg, SPI_SR_RDRF))
		return false;

	const uint16_t rx = static_cast<uint16_t>(reg->SPI_RDR & SPI_RDR_RD_Msk);
	if (pRx != nullptr)
		*pRx = rx;
	return true;
}

static int Sam4lSpiCommand(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	if (pData == nullptr || DataLen <= 0)
		return 0;

	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	if (dev->pSpiDev->Cfg.Mode != SPIMODE_MASTER)
		return 0;
	const bool wide = dev->pSpiDev->Cfg.DataSize > 8U;
	int count = 0;

	while (DataLen >= (wide ? 2 : 1))
	{
		const uint16_t tx = wide ?
			static_cast<uint16_t>(pData[0] | (static_cast<uint16_t>(pData[1]) << 8U)) :
			pData[0];
		if (!Sam4lSpiTransferWord(dev, tx, nullptr))
			break;

		const int step = wide ? 2 : 1;
		pData += step;
		DataLen -= step;
		count += step;
	}
	return count;
}

static int Sam4lSpiPollingRx(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == nullptr || BuffLen <= 0)
		return 0;

	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	const bool wide = dev->pSpiDev->Cfg.DataSize > 8U;
	const uint16_t dummy = wide ?
		static_cast<uint16_t>(dev->pSpiDev->Cfg.DummyByte |
			(static_cast<uint16_t>(dev->pSpiDev->Cfg.DummyByte) << 8U)) :
		dev->pSpiDev->Cfg.DummyByte;
	int count = 0;

	while (BuffLen >= (wide ? 2 : 1))
	{
		uint16_t rx;
		if (!Sam4lSpiTransferWord(dev, dummy, &rx))
			break;

		pBuff[0] = static_cast<uint8_t>(rx);
		if (wide)
			pBuff[1] = static_cast<uint8_t>(rx >> 8U);

		const int step = wide ? 2 : 1;
		pBuff += step;
		BuffLen -= step;
		count += step;
	}
	return count;
}

static void Sam4lSpiDmaInit(Sam4lSpiDev_t *dev)
{
	Sam4lSpiPmWrite(&SAM4L_PM->PM_HSBMASK,
		SAM4L_PM->PM_HSBMASK | PM_HSBMASK_PDCA);
	Sam4lSpiPmWrite(&SAM4L_PM->PM_PBBMASK,
		SAM4L_PM->PM_PBBMASK | PM_PBBMASK_PDCA);
	dev->DmaInitialized = true;
	Sam4lSpiDmaStop(dev);
	for (int i = 0; i < 2; ++i)
	{
		PdcaChannel *chan = i == 0 ? Sam4lSpiRxChannel() : Sam4lSpiTxChannel();
		chan->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
		chan->PDCA_PSR = PDCA_PSR_PID(i == 0 ? 4U : 22U);
		chan->PDCA_MR = PDCA_MR_SIZE(1U); // Halfword; SPI data is 8..16 bits.
		chan->PDCA_TCR = 0U;
		chan->PDCA_MARR = 0U;
		chan->PDCA_TCRR = 0U;
	}
}

static uint16_t Sam4lSpiTxWord(Sam4lSpiDev_t *dev, int Offset)
{
	if (dev->pTx != nullptr)
	{
		uint16_t word = dev->pTx[Offset];
		if (dev->Step == 2)
			word |= static_cast<uint16_t>(dev->pTx[Offset + 1]) << 8U;
		return word;
	}
	const uint16_t dummy = dev->pSpiDev->Cfg.DummyByte;
	return dev->Step == 2 ? dummy | (dummy << 8U) : dummy;
}

static void Sam4lSpiRxWord(Sam4lSpiDev_t *dev, uint16_t Word)
{
	if (dev->pRx != nullptr)
	{
		dev->pRx[dev->Count] = static_cast<uint8_t>(Word);
		if (dev->Step == 2)
			dev->pRx[dev->Count + 1] = static_cast<uint8_t>(Word >> 8U);
	}
	dev->Count += dev->Step;
}

static void Sam4lSpiFinish(Sam4lSpiDev_t *dev, bool Error)
{
	DevIntrf_t *intrf = &dev->pSpiDev->DevIntrf;
	const bool async = dev->Async;
	const bool read = dev->pRx != nullptr;
	uint8_t *buffer = dev->pRx;
	const int count = dev->Count;
	dev->pReg->SPI_IDR = 0xFFFFFFFFU;
	Sam4lSpiDmaStop(dev);
	if (Error)
		Sam4lSpiConfigure(dev);
	dev->Active = false;
	intrf->bTxReady = true;
	if (!async)
		return;

	// The generic stop helpers own the busy flag. Release before callbacks.
	if (read)
		DeviceIntrfStopRx(intrf);
	else
		DeviceIntrfTxComplete(intrf);
	if (intrf->EvtCB != nullptr)
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, buffer, count);
}

static void Sam4lSpiCancel(Sam4lSpiDev_t *dev)
{
	// Cancellation is silent: caller is resetting/disabling the interface.
	// Mask the whole transfer before inspecting state shared with the ISR.
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	dev->pReg->SPI_IDR = 0xFFFFFFFFU;
	Sam4lSpiDmaStop(dev);
	dev->SlaveReady = false;
	if (dev->Active)
	{
		DevIntrf_t *intrf = &dev->pSpiDev->DevIntrf;
		const bool async = dev->Async;
		const bool read = dev->pRx != nullptr;
		dev->Active = false;
		Sam4lSpiConfigure(dev);
		intrf->bTxReady = true;
		if (async)
		{
			if (read)
				DeviceIntrfStopRx(intrf);
			else
				DeviceIntrfStopTx(intrf);
		}
	}
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_ClearPendingIRQ(PDCA_12_IRQn);
	NVIC_ClearPendingIRQ(PDCA_13_IRQn);
	__set_PRIMASK(primask);
}

static void Sam4lSpiDmaChunk(Sam4lSpiDev_t *dev)
{
	int words = (dev->Length - dev->Count) / dev->Step;
	if (words > SAM4L_SPI_DMA_WORDS)
		words = SAM4L_SPI_DMA_WORDS;
	dev->ChunkWords = words;
	for (int i = 0; i < words; ++i)
		dev->DmaTx[i] = Sam4lSpiTxWord(dev, dev->Count + i * dev->Step);

	PdcaChannel *rx = Sam4lSpiRxChannel();
	PdcaChannel *tx = Sam4lSpiTxChannel();
	rx->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
	tx->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
	rx->PDCA_MAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(dev->DmaRx));
	tx->PDCA_MAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(dev->DmaTx));
	rx->PDCA_TCR = words;
	tx->PDCA_TCR = words;
	__DMB();
	if (dev->Async)
	{
		rx->PDCA_IER = PDCA_IER_TRC | PDCA_IER_TERR;
		tx->PDCA_IER = PDCA_IER_TERR;
	}
	// Every transmitted word produces receive data, even for a TX-only API.
	// Arm RX first so the first character cannot overrun RDR.
	rx->PDCA_CR = PDCA_CR_TEN;
	tx->PDCA_CR = PDCA_CR_TEN;
}

static void Sam4lSpiDmaCollect(Sam4lSpiDev_t *dev)
{
	Sam4lSpiDmaStop(dev);
	__DMB();
	int words = dev->ChunkWords - static_cast<int>(Sam4lSpiRxChannel()->PDCA_TCR);
	if (words < 0 || words > dev->ChunkWords)
		words = 0;
	for (int i = 0; i < words; ++i)
		Sam4lSpiRxWord(dev, dev->DmaRx[i]);
	dev->ChunkWords = 0;
}

static int Sam4lSpiTransfer(Sam4lSpiDev_t *dev, const uint8_t *Tx,
						   uint8_t *Rx, int Length, bool Async)
{
	if (dev->Active || Length <= 0)
		return 0;
	dev->Step = dev->pSpiDev->Cfg.DataSize > 8U ? 2 : 1;
	dev->Length = Length - Length % dev->Step;
	if (dev->Length == 0)
		return 0;
	dev->Count = 0;
	dev->ChunkWords = 0;
	dev->pRx = Rx;
	dev->pTx = Tx;
	dev->Async = Async;
	if (Async && Tx != nullptr)
	{
		memcpy(dev->TxBuffer, Tx, dev->Length);
		dev->pTx = dev->TxBuffer;
	}
	dev->Active = true;
	dev->pSpiDev->DevIntrf.bTxReady = false;

	if (Async)
		dev->pReg->SPI_IER = SAM4L_SPI_ERRORS;
	if (dev->pSpiDev->Cfg.bDmaEn)
	{
		do
		{
			Sam4lSpiDmaChunk(dev);
			if (Async)
				return -1;
			uint32_t timeout = SAM4L_SPI_WAIT_COUNT;
			bool error = false;
			do
			{
				error = ((Sam4lSpiRxChannel()->PDCA_ISR |
					Sam4lSpiTxChannel()->PDCA_ISR) & PDCA_ISR_TERR) != 0U ||
					(dev->pReg->SPI_SR & SAM4L_SPI_ERRORS) != 0U;
				if (error || Sam4lSpiRxChannel()->PDCA_TCR == 0U)
					break;
			} while (--timeout != 0U);
			Sam4lSpiDmaCollect(dev);
			if (error || timeout == 0U)
			{
				Sam4lSpiFinish(dev, true);
				return dev->Count;
			}
		} while (dev->Count < dev->Length);
		Sam4lSpiFinish(dev, !Sam4lSpiWait(dev->pReg, SPI_SR_TXEMPTY));
		return dev->Count;
	}
	// Interrupt mode allows only one outstanding character. RDRF advances it.
	dev->pReg->SPI_IER = SPI_IER_RDRF;
	dev->pReg->SPI_TDR = Sam4lSpiTxWord(dev, 0);
	return -1;
}

static int Sam4lSpiTxData(DevIntrf_t * const pDev, const uint8_t *Data, int Length)
{
	if (Data == nullptr || Length <= 0)
		return 0;
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	if (dev->pSpiDev->Cfg.Mode != SPIMODE_MASTER)
		return 0;
	// Long TX stays synchronous so no caller buffer survives this call.
	const bool async = pDev->bIntEn && Length <= SAM4L_SPI_TX_SIZE;
	if (!pDev->bDma && !async)
		return Sam4lSpiCommand(pDev, Data, Length);
	return Sam4lSpiTransfer(dev, Data, nullptr, Length, async);
}

static int Sam4lSpiRxData(DevIntrf_t * const pDev, uint8_t *Buffer, int Length)
{
	if (Buffer == nullptr || Length <= 0)
		return 0;
	if (static_cast<Sam4lSpiDev_t *>(pDev->pDevData)->pSpiDev->Cfg.Mode != SPIMODE_MASTER)
		return 0;
	if (!pDev->bDma && !pDev->bIntEn)
		return Sam4lSpiPollingRx(pDev, Buffer, Length);
	return Sam4lSpiTransfer(static_cast<Sam4lSpiDev_t *>(pDev->pDevData),
		nullptr, Buffer, Length, pDev->bIntEn);
}

static void Sam4lSpiSlaveArm(Sam4lSpiDev_t *dev)
{
	SPIDev_t *spi = dev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	dev->SlaveReady = false;
	if (intrf->EvtCB != nullptr)
		intrf->EvtCB(intrf, DEVINTRF_EVT_STATECHG, nullptr, 0);
	if (intrf->EnCnt == 0 || spi->Cfg.Mode != SPIMODE_SLAVE)
		return;
	dev->pRx = spi->pRxBuff[0];
	dev->Length = dev->pRx != nullptr && spi->RxBuffLen[0] > 0 ?
		spi->RxBuffLen[0] : 0;
	dev->pTx = spi->pTxData[0];
	dev->SlaveTxLength = dev->pTx != nullptr && spi->TxDataLen[0] > 0 ?
		spi->TxDataLen[0] : 0;
	dev->Count = 0;
	dev->SlaveError = false;
	dev->SlaveReady = true;
	dev->SlaveDmaFrame = spi->Cfg.bDmaEn &&
		dev->Length <= 65535 && dev->SlaveTxLength <= 65535;
	dev->SlaveRxDma = false;
	// PDCA owns every TX byte in DMA mode, including the first one.
	// A CPU preload as well would transmit byte zero twice.
	if (!dev->SlaveDmaFrame || dev->SlaveTxLength == 0)
		dev->pReg->SPI_TDR = dev->SlaveTxLength > 0 ?
			dev->pTx[0] : spi->Cfg.DummyByte;
	uint32_t mask = SPI_IER_NSSR | SPI_IER_OVRES | SPI_IER_UNDES;
	if (dev->SlaveDmaFrame)
	{
		PdcaChannel *rx = Sam4lSpiRxChannel();
		PdcaChannel *tx = Sam4lSpiTxChannel();
		rx->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
		tx->PDCA_CR = PDCA_CR_TDIS | PDCA_CR_ECLR;
		rx->PDCA_MR = PDCA_MR_SIZE_BYTE;
		tx->PDCA_MR = PDCA_MR_SIZE_BYTE;
		rx->PDCA_TCR = dev->Length;
		tx->PDCA_TCR = dev->SlaveTxLength;
		rx->PDCA_MAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(dev->pRx));
		tx->PDCA_MAR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(dev->pTx));
		__DMB();
		if (dev->Length > 0)
		{
			dev->SlaveRxDma = true;
			rx->PDCA_IER = PDCA_IER_TRC | PDCA_IER_TERR;
			rx->PDCA_CR = PDCA_CR_TEN;
		}
		else
			mask |= SPI_IER_RDRF;
		if (dev->SlaveTxLength > 0)
		{
			tx->PDCA_IER = PDCA_IER_TRC | PDCA_IER_TERR;
			tx->PDCA_CR = PDCA_CR_TEN;
		}
		else
			mask |= SPI_IER_TDRE;
	}
	else
		mask |= SPI_IER_RDRF;
	dev->pReg->SPI_IER = mask;
}

// Stop before sampling TCR so CPU and PDCA never consume the same RDR.
static void Sam4lSpiSlaveDmaCollect(Sam4lSpiDev_t *dev)
{
	if (!dev->SlaveRxDma)
		return;
	PdcaChannel *rx = Sam4lSpiRxChannel();
	rx->PDCA_IDR = 0xFFFFFFFFU;
	rx->PDCA_CR = PDCA_CR_TDIS;
	__NOP();
	__NOP();
	__DMB();
	dev->Count = dev->Length - static_cast<int>(rx->PDCA_TCR);
	dev->SlaveRxDma = false;
}

static void Sam4lSpiSlaveIrq(Sam4lSpiDev_t *dev, uint32_t Status)
{
	if (!dev->SlaveReady)
		return;
	uint32_t events = Status;
	if (dev->SlaveDmaFrame)
	{
		if ((events & SPI_SR_NSSR) != 0U)
		{
			if (((Sam4lSpiRxChannel()->PDCA_ISR |
				Sam4lSpiTxChannel()->PDCA_ISR) & PDCA_ISR_TERR) != 0U)
				dev->SlaveError = true;
			Sam4lSpiDmaStop(dev);
			Sam4lSpiSlaveDmaCollect(dev);
			// The captured RDRF may have been serviced by PDCA already.
			Status = dev->pReg->SPI_SR;
			events |= Status;
		}
		else if (dev->SlaveRxDma)
			Status &= ~SPI_SR_RDRF;
	}
	bool received = false;
	// The receive FIFO holds four characters. NSSR can arrive alongside
	// several unread characters; drain them before retiring the frame.
	for (unsigned i = 0; i < 4U && (Status & SPI_SR_RDRF) != 0U; ++i)
	{
		const uint8_t data = static_cast<uint8_t>(dev->pReg->SPI_RDR);
		received = true;
		if (dev->pRx != nullptr && dev->Count < dev->Length)
			dev->pRx[dev->Count] = data;
		else if (dev->pRx != nullptr)
			dev->SlaveError = true;
		if (dev->Count < INT_MAX)
			++dev->Count;
		else
			dev->SlaveError = true;

		Status = dev->pReg->SPI_SR;
		events |= Status; // SR reads clear NSSR and error flags.
	}
	if ((events & (SPI_SR_OVRES | SPI_SR_UNDES)) != 0U)
		dev->SlaveError = true;
	if ((events & SPI_SR_NSSR) == 0U)
	{
		if (!dev->SlaveDmaFrame && received)
			dev->pReg->SPI_TDR = dev->Count < dev->SlaveTxLength ?
				dev->pTx[dev->Count] : dev->pSpiDev->Cfg.DummyByte;
		if (dev->SlaveDmaFrame && (events & dev->pReg->SPI_IMR & SPI_SR_TDRE) != 0U)
			dev->pReg->SPI_TDR = dev->pSpiDev->Cfg.DummyByte;
		return;
	}

	DevIntrf_t *intrf = &dev->pSpiDev->DevIntrf;
	uint8_t *buffer = dev->pRx;
	const int count = dev->SlaveError ? -1 : dev->Count;
	dev->SlaveReady = false;
	dev->pReg->SPI_IDR = 0xFFFFFFFFU;
	// Discard the unused preloaded character at a frame boundary. The next
	// frame must start at TX byte zero, including after a short transaction.
	Sam4lSpiConfigure(dev);
	if (intrf->EvtCB != nullptr)
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, buffer, count);
	if (intrf->EnCnt > 0 && dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE &&
		!dev->SlaveReady)
		Sam4lSpiSlaveArm(dev);
}

extern "C" void SPI_Handler(void)
{
	Sam4lSpiDev_t *dev = &s_SpiDev;
	const uint32_t status = dev->pReg->SPI_SR;
	const uint32_t pending = status & dev->pReg->SPI_IMR;
	if (dev->pSpiDev != nullptr && dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		Sam4lSpiSlaveIrq(dev, status);
		return;
	}
	if (!dev->Active || !dev->Async)
		return;
	if ((pending & SAM4L_SPI_ERRORS) != 0U)
	{
		if (dev->pSpiDev->Cfg.bDmaEn && dev->ChunkWords > 0)
			Sam4lSpiDmaCollect(dev);
		Sam4lSpiFinish(dev, true);
		return;
	}
	if ((pending & SPI_SR_RDRF) != 0U)
	{
		Sam4lSpiRxWord(dev, static_cast<uint16_t>(dev->pReg->SPI_RDR));
		if (dev->Count < dev->Length)
			dev->pReg->SPI_TDR = Sam4lSpiTxWord(dev, dev->Count);
		else
		{
			dev->pReg->SPI_IDR = SPI_IDR_RDRF;
			dev->pReg->SPI_IER = SPI_IER_TXEMPTY;
		}
		return; // Re-read TXEMPTY on the next IRQ, never use stale status.
	}
	if ((pending & SPI_SR_TXEMPTY) != 0U)
		Sam4lSpiFinish(dev, false);
}

static void Sam4lSpiDmaIrq(void)
{
	Sam4lSpiDev_t *dev = &s_SpiDev;
	const uint32_t rx = Sam4lSpiRxChannel()->PDCA_ISR & Sam4lSpiRxChannel()->PDCA_IMR;
	const uint32_t tx = Sam4lSpiTxChannel()->PDCA_ISR & Sam4lSpiTxChannel()->PDCA_IMR;
	if (dev->pSpiDev != nullptr && dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		if (!dev->SlaveReady || !dev->SlaveDmaFrame)
			return;
		if (((rx | tx) & PDCA_ISR_TERR) != 0U)
		{
			dev->SlaveError = true;
			Sam4lSpiDmaStop(dev);
			Sam4lSpiSlaveDmaCollect(dev);
			dev->pReg->SPI_IER = SPI_IER_RDRF | SPI_IER_TDRE;
			return; // Report error at NSS rising, once per frame.
		}
		if ((rx & PDCA_ISR_TRC) != 0U)
		{
			Sam4lSpiSlaveDmaCollect(dev);
			dev->pReg->SPI_IER = SPI_IER_RDRF; // Detect excess clocks safely.
		}
		if ((tx & PDCA_ISR_TRC) != 0U)
		{
			Sam4lSpiTxChannel()->PDCA_IDR = 0xFFFFFFFFU;
			Sam4lSpiTxChannel()->PDCA_CR = PDCA_CR_TDIS;
			dev->pReg->SPI_IER = SPI_IER_TDRE; // Dummy bytes after TX ends.
		}
		return;
	}
	if (!dev->Active || !dev->Async)
		return;
	if (((rx | tx) & PDCA_ISR_TERR) != 0U)
	{
		Sam4lSpiDmaCollect(dev);
		Sam4lSpiFinish(dev, true);
		return;
	}
	if ((rx & PDCA_ISR_TRC) == 0U)
		return;
	Sam4lSpiDmaCollect(dev);
	if (dev->Count < dev->Length)
		Sam4lSpiDmaChunk(dev);
	else
		dev->pReg->SPI_IER = SPI_IER_TXEMPTY;
}

extern "C" void PDCA_12_Handler(void)
{
	Sam4lSpiDmaIrq();
}

extern "C" void PDCA_13_Handler(void)
{
	Sam4lSpiDmaIrq();
}

static void *Sam4lSpiGetHandle(DevIntrf_t * const pDev)
{
	Sam4lSpiDev_t *dev = static_cast<Sam4lSpiDev_t *>(pDev->pDevData);
	return dev->pSpiDev;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	if (pDev == nullptr || pCfgData == nullptr ||
		pCfgData->DevNo != 0 ||
		(pCfgData->Mode != SPIMODE_MASTER && pCfgData->Mode != SPIMODE_SLAVE) ||
		pCfgData->Phy != SPIPHY_NORMAL ||
		pCfgData->BitOrder != SPIDATABIT_MSB ||
		pCfgData->DataSize < 8U || pCfgData->DataSize > 16U ||
		pCfgData->Rate == 0U ||
		pCfgData->pIOPinMap == nullptr || pCfgData->NbIOPins < SPI_CS_IOPIN_IDX ||
		s_SpiDev.Active)
	{
		return false;
	}
	// Slave wide words need their own external-clock validation.
	if (pCfgData->Mode == SPIMODE_SLAVE &&
		(pCfgData->DataSize != 8U ||
		 pCfgData->NbIOPins <= SPI_CS_IOPIN_IDX))
		return false;
	if (pCfgData->ChipSel != SPICSEL_MAN &&
		pCfgData->NbIOPins <= SPI_CS_IOPIN_IDX)
	{
		return false;
	}

	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_DisableIRQ(PDCA_12_IRQn);
	NVIC_DisableIRQ(PDCA_13_IRQn);
	Sam4lSpiDmaStop(&s_SpiDev);
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_ClearPendingIRQ(PDCA_12_IRQn);
	NVIC_ClearPendingIRQ(PDCA_13_IRQn);
	pDev->Cfg = *pCfgData;
	if (pDev->Cfg.Mode == SPIMODE_SLAVE)
		pDev->Cfg.bIntEn = true;
	pDev->CurDevCs = -1;
	pDev->FirstRdData = -1;
	s_SpiDev.pSpiDev = pDev;
	pDev->DevIntrf.pDevData = &s_SpiDev;

	Sam4lSpiClockEnable();
	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);
	if (pDev->Cfg.Mode == SPIMODE_MASTER && pDev->Cfg.ChipSel != SPICSEL_MAN)
	{
		for (int i = SPI_CS_IOPIN_IDX; i < pDev->Cfg.NbIOPins; ++i)
			IOPinSet(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo);
	}

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Disable = Sam4lSpiDisable;
	pDev->DevIntrf.Enable = Sam4lSpiEnable;
	pDev->DevIntrf.GetRate = Sam4lSpiGetRate;
	pDev->DevIntrf.SetRate = Sam4lSpiSetRate;
	pDev->DevIntrf.StartRx = Sam4lSpiStartRx;
	pDev->DevIntrf.RxData = Sam4lSpiRxData;
	pDev->DevIntrf.StopRx = Sam4lSpiStopRx;
	pDev->DevIntrf.StartTx = Sam4lSpiStartTx;
	pDev->DevIntrf.TxData = Sam4lSpiTxData;
	// The generic Read command phase must finish before it starts RX.
	pDev->DevIntrf.TxSrData = Sam4lSpiCommand;
	pDev->DevIntrf.StopTx = Sam4lSpiStopTx;
	pDev->DevIntrf.Reset = Sam4lSpiReset;
	pDev->DevIntrf.PowerOff = Sam4lSpiPowerOff;
	pDev->DevIntrf.GetHandle = Sam4lSpiGetHandle;
	pDev->DevIntrf.IntPrio = pDev->Cfg.IntPrio;
	pDev->DevIntrf.EvtCB = pDev->Cfg.EvtCB;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.bDma = pDev->Cfg.bDmaEn;
	pDev->DevIntrf.bIntEn = pDev->Cfg.bIntEn;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	Sam4lSpiConfigure(&s_SpiDev);
	if (pDev->Cfg.bDmaEn)
		Sam4lSpiDmaInit(&s_SpiDev);
	if (pDev->Cfg.Mode == SPIMODE_SLAVE)
		Sam4lSpiSlaveArm(&s_SpiDev);
	if (pDev->Cfg.bIntEn)
	{
		NVIC_SetPriority(SPI_IRQn, pDev->Cfg.IntPrio);
		NVIC_SetPriority(PDCA_12_IRQn, pDev->Cfg.IntPrio);
		NVIC_SetPriority(PDCA_13_IRQn, pDev->Cfg.IntPrio);
		NVIC_EnableIRQ(SPI_IRQn);
		if (pDev->Cfg.bDmaEn)
		{
			NVIC_EnableIRQ(PDCA_12_IRQn);
			NVIC_EnableIRQ(PDCA_13_IRQn);
		}
	}
	return pDev->Cfg.Rate != 0U;
}

