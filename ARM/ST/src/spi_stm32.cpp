/**-------------------------------------------------------------------------
@file	spi_stm32.cpp

@brief	Generic STM32 SPI implementation, master and slave

		Two register layouts, chosen from the CMSIS names. The SPI with the
		CFG1, CFG2 and TSIZE registers (WBA; the same IP is in the U5, H5 and
		N6 families), and the SPI with CR1, CR2, SR and DR and a 32 bit FIFO
		each way (L4; the same IP is in the F0, F3, G4 and WB families). The
		engine is the same for both, the instance table holds what differs:
		the RCC enable, reset and kernel clock selection, the interrupt and
		the DMA requests. A family is added with its table.

		CR1/CR2 layout (L4). DevNo 0 to 2 are SPI1 to SPI3, an instance the
		part does not have (SPI2 of the STM32L43x and L44x) fails SPIInit.
		SPI1 is clocked from PCLK2, SPI2 and SPI3 from PCLK1, the rate is set
		with the BR divider of CR1 (2 to 256). The peripheral has no transfer
		size: a master transfer ends when its last item is received. With
		bIntEn the receive interrupt moves the items, at most a FIFO of items
		in flight. The master keeps SPE set between transfers so that SCK
		stays at its idle level. The slave has no underrun value: its
		transmit FIFO is kept filled, with the DummyByte after the data.
		Clearing SPE leaves the FIFO content, so the slave is reset through
		RCC before each frame. QUADSPI is not handled here. DMA is not
		available on this layout yet, SPIInit returns false for bDmaEn.
		What follows describes the CFG1/CFG2 layout; master chip select,
		events, the slave frame sequence and the NSS pin event are the same
		on both.

		DevNo 0 to 2 are SPI1 to SPI3 of the WBA. SPI1 is clocked from PCLK2,
		SPI2 from PCLK1 and SPI3 from PCLK7. The kernel clock selection is
		PCLK. The rate is set with the MBR divider of CFG1 (2 to 256). SetRate
		and SPIInit return and store the rate actually set, the closest the
		divider gives: a request is not rejected for being out of range.

		The data size is 4 to 16 bits. Up to 8 bits the data is one byte per
		item, above 8 bits it is two bytes per item, low byte first. Lengths
		are in bytes. Only the standard 4 wire interface (SPIPHY_NORMAL) is
		implemented.

		Master. Full duplex. Each TxData and RxData call is one transfer of
		TSIZE items started with CSTART, as HAL_SPI_Transmit and
		HAL_SPI_TransmitReceive do, in chunks of up to 65535 items. The chip
		select is a GPIO of the pin map from index SPI_CS_IOPIN_IDX, driven
		low by StartTx and StartRx and high by StopTx and StopRx, so it stays
		low between the command phase and the data phase of a read. The
		hardware NSS is not used. With ChipSel SPICSEL_MAN, or a pin map that
		has no chip select pin, the application drives the chip select.
		Receive clocks out the DummyByte of the configuration. Data received
		during a transmit is discarded. AFCNTR keeps the pins driven by the
		peripheral between two transfers, so SCK stays at its idle level.

		Polling, interrupt and DMA run the same engine. Without bIntEn the
		call waits for the end of the transfer. With bIntEn a receive, and a
		transmit of up to SPI_STM32_TXBUF_SIZE bytes, return -1 at once and the
		end is reported with DEVINTRF_EVT_COMPLETED and the byte count; the
		transmit data is copied first. With bDmaEn the data is moved by two
		GPDMA channels, one for each direction; a direction with no data of
		the application runs on the dummy value or into a sink. DMA is
		available on the families that have a channel table (WBA); SPIInit
		returns false for bDmaEn on the others. The channels are taken at
		SPIInit and kept until PowerOff.

		An underrun, an overrun or a mode fault ends the transfer and returns
		the number of bytes done. Waits are bounded in polling mode.

		Slave. Interrupt driven, with or without DMA. The pin map is SCK, MISO,
		MOSI and the NSS pin at index SPI_CS_IOPIN_IDX, which the hardware
		uses (SSM off). The frame sequence follows the other IOsonata SPI
		slaves: DEVINTRF_EVT_STATECHG before each frame lets the handler give
		the buffers (SetSlaveRxBuffer and SetSlaveTxData, index 0), the frame,
		then DEVINTRF_EVT_COMPLETED with the number of bytes of the frame, or
		-1 after an overrun or a frame longer than the receive buffer. The
		next frame is then prepared. After the transmit data the DummyByte is
		sent (UDRDR). The frame ends when NSS goes inactive. With DMA the
		receive count is the number of bytes the channel has moved when NSS
		rises.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2026

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
#include <string.h>
#include <stdint.h>

#include "stm32.h"

#if defined(IOSONATA_STM32_WBA)
#include "gpdma_stm32wba.h"
#define STM32_SPI_DMA
#endif

#include "istddef.h"
#include "coredev/spi.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"

#define STM32_SPI_MAXDEV			3

// Register layout of the SPI, from the CMSIS names
#if defined(SPI_CFG1_MBR)
// CFG1, CFG2, TSIZE, TXDR and RXDR
#define STM32_SPI_LAYOUT_CFG
#elif defined(SPI_CR2_DS)
// CR1, CR2, SR and DR with a 32 bit FIFO each way
#define STM32_SPI_LAYOUT_CR
#else
#error "spi_stm32.cpp: SPI register layout not supported"
#endif

#ifdef STM32_SPI_LAYOUT_CFG
#define STM32_SPI_TSIZE_MAX			0xFFFFU

#define SPI_STM32_TXBUF_SIZE		256

// Data bytes the transmit FIFO holds in flight. SPI3 has 8, SPI1 and SPI2 have
// 16, the smaller value is used for all.
#define STM32_SPI_FIFO_BYTES		8

#define STM32_SPI_MBR_MAX			7

#define STM32_SPI_DATASIZE_MIN		4
#define STM32_SPI_DATASIZE_MAX		16

#define STM32_SPI_RATE_DEFAULT		1000000

// Kernel clock of the SPI is the APB clock of the instance (CCIPR field 0)
#define STM32_SPI_CLKSEL_PCLK		0

// 16 bit access to the data registers
typedef volatile uint16_t __attribute__((may_alias)) Stm32SpiReg16_t;

#define STM32_SPI_ERR_FLAGS			(SPI_SR_UDR | SPI_SR_OVR | SPI_SR_MODF)
#define STM32_SPI_IFCR_ALL			(SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC | SPI_IFCR_OVRC | \
									 SPI_IFCR_CRCEC | SPI_IFCR_TIFREC | SPI_IFCR_MODFC | SPI_IFCR_SUSPC)
#define STM32_SPI_CFG1_DMA			(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN)
#define STM32_SPI_IER_MASTER		(SPI_IER_EOTIE | SPI_IER_UDRIE | SPI_IER_OVRIE | SPI_IER_MODFIE)
#define STM32_SPI_IER_DATA			(SPI_IER_TXPIE | SPI_IER_RXPIE)
#define STM32_SPI_IER_SLAVE			(SPI_IER_EOTIE | SPI_IER_OVRIE)
#else
#define SPI_STM32_TXBUF_SIZE		256

// Data bytes in flight, the 32 bit FIFO
#define STM32_SPI_FIFO_BYTES		4

// BR divider of CR1, 2 to 256
#define STM32_SPI_MBR_MAX			7

#define STM32_SPI_DATASIZE_MIN		4
#define STM32_SPI_DATASIZE_MAX		16

#define STM32_SPI_RATE_DEFAULT		1000000

// 16 bit access to the data register
typedef volatile uint16_t __attribute__((may_alias)) Stm32SpiReg16_t;

// FRE is for the TI mode, not used
#define STM32_SPI_ERR_FLAGS			(SPI_SR_OVR | SPI_SR_MODF)
#define STM32_SPI_CR2_INT			(SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE)
#endif

#ifdef STM32_SPI_LAYOUT_CFG
#define STM32_SPI_SR_RXNE			SPI_SR_RXP
#else
#define STM32_SPI_SR_RXNE			SPI_SR_RXNE
#endif

#pragma pack(push, 4)

/// What differs between the instances: registers of the RCC, interrupt, DMA
typedef struct {
	SPI_TypeDef *pReg;
	volatile uint32_t *pEnReg;		//!< RCC clock enable register
	uint32_t EnMask;
	volatile uint32_t *pRstReg;		//!< RCC reset register
	uint32_t RstMask;
	volatile uint32_t *pSelReg;		//!< RCC kernel clock selection register
	uint32_t SelMask;				//!< Selection field, written with the PCLK value
	int ClkIdx;						//!< SystemPeriphClockGet index of the APB clock
	IRQn_Type Irq;
	uint8_t TxReq;					//!< GPDMA request of the transmit, 0 none
	uint8_t RxReq;
} Stm32SpiInfo_t;

typedef struct {
	int DevNo;
	const Stm32SpiInfo_t *pInfo;
	SPIDev_t *pSpiDev;
	SPI_TypeDef *pReg;
	bool bCsDrive;			//!< The driver drives the chip select pin
	bool bCsLow;			//!< Chip select pin driven low by StartTx or StartRx
	bool bSlave;
	bool bFrame;			//!< Slave: NSS seen active, the frame is running
	bool bNssInt;			//!< Slave: the NSS pin event is enabled
	int NssLine;			//!< Slave: EXTI line of the NSS pin
	// Master transfer
	bool bRun;				//!< Transfer engine running
	bool bAsync;			//!< Running transfer ends in the interrupt and reports with the event
	bool bFail;				//!< Underrun, overrun or mode fault
	bool bRx;				//!< Direction of an asynchronous transfer, for the completion call
	bool bDma;				//!< Data moved by DMA
	int DmaTx;				//!< GPDMA channel of the transmit, -1 none
	int DmaRx;
	const uint8_t *pTx;
	uint8_t *pRx;
	uint32_t Items;			//!< Items of the transfer
	uint32_t Done;			//!< Items of the chunks completed
	uint32_t Chunk;			//!< Items of the running chunk
	uint32_t TxN;			//!< Items written in the running chunk
	uint32_t RxN;			//!< Items read in the running chunk
	int Result;				//!< Bytes done of the last transfer
	// Slave frame
	bool bBad;				//!< Overrun or frame longer than the receive buffer
	uint32_t SRx;			//!< Items received
	uint32_t STx;			//!< Items written
	uint32_t SRxMax;		//!< Items the receive buffer holds
	uint32_t STxMax;		//!< Items of transmit data
	uint16_t Sink;			//!< Receive data nobody wants
	uint16_t Dummy;			//!< Transmit value when there is no data
#ifdef STM32_SPI_LAYOUT_CR
	uint32_t Cr1;			//!< CR1 of the configuration, without SPE
	uint32_t Cr2;			//!< CR2 of the configuration, without the interrupt enables
#endif
	uint8_t TxBuf[SPI_STM32_TXBUF_SIZE];
} Stm32SpiDev_t;

#pragma pack(pop)

#if defined(IOSONATA_STM32_WBA)
static const Stm32SpiInfo_t s_Stm32SpiInfo[STM32_SPI_MAXDEV] = {
	{ SPI1, &RCC->APB2ENR, RCC_APB2ENR_SPI1EN, &RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST,
	  &RCC->CCIPR1, RCC_CCIPR1_SPI1SEL_Msk, 1, SPI1_IRQn,
	  WBA_GPDMA_REQ_SPI1_TX, WBA_GPDMA_REQ_SPI1_RX },
	{ SPI2, &RCC->APB1ENR1, RCC_APB1ENR1_SPI2EN, &RCC->APB1RSTR1, RCC_APB1RSTR1_SPI2RST,
	  &RCC->CCIPR1, RCC_CCIPR1_SPI2SEL_Msk, 0, SPI2_IRQn,
	  WBA_GPDMA_REQ_SPI2_TX, WBA_GPDMA_REQ_SPI2_RX },
	{ SPI3, &RCC->APB7ENR, RCC_APB7ENR_SPI3EN, &RCC->APB7RSTR, RCC_APB7RSTR_SPI3RST,
	  &RCC->CCIPR3, RCC_CCIPR3_SPI3SEL_Msk, 2, SPI3_IRQn,
	  WBA_GPDMA_REQ_SPI3_TX, WBA_GPDMA_REQ_SPI3_RX },
};
#elif defined(IOSONATA_STM32_L4)
// SPI1 on APB2 (PCLK2), SPI2 and SPI3 on APB1 (PCLK1). The kernel clock is the
// APB clock, there is no selection. The STM32L43x and L44x have no SPI2.
static const Stm32SpiInfo_t s_Stm32SpiInfo[STM32_SPI_MAXDEV] = {
	{ SPI1, &RCC->APB2ENR, RCC_APB2ENR_SPI1EN, &RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST,
	  NULL, 0, 1, SPI1_IRQn, 0, 0 },
#ifdef SPI2
	{ SPI2, &RCC->APB1ENR1, RCC_APB1ENR1_SPI2EN, &RCC->APB1RSTR1, RCC_APB1RSTR1_SPI2RST,
	  NULL, 0, 0, SPI2_IRQn, 0, 0 },
#else
	{ NULL, NULL, 0, NULL, 0, NULL, 0, 0, (IRQn_Type)0, 0, 0 },
#endif
	{ SPI3, &RCC->APB1ENR1, RCC_APB1ENR1_SPI3EN, &RCC->APB1RSTR1, RCC_APB1RSTR1_SPI3RST,
	  NULL, 0, 0, SPI3_IRQn, 0, 0 },
};
#else
#error "spi_stm32.cpp: no SPI instance table for this STM32 family"
#endif

static Stm32SpiDev_t s_Stm32SpiDev[STM32_SPI_MAXDEV];

static Stm32SpiDev_t *Stm32SpiHandle(DevIntrf_t * const pIntrf)
{
	Stm32SpiDev_t *dev = pIntrf ? (Stm32SpiDev_t*)pIntrf->pDevData : NULL;

	return dev && dev->pSpiDev && &dev->pSpiDev->DevIntrf == pIntrf ? dev : NULL;
}

static void Stm32SpiClkEnable(Stm32SpiDev_t * const pDev, bool bEn)
{
	const Stm32SpiInfo_t *info = pDev->pInfo;

	if (bEn)
	{
#ifdef STM32_SPI_LAYOUT_CFG
		*info->pSelReg &= ~info->SelMask;
#endif
		*info->pEnReg |= info->EnMask;
		(void)*info->pEnReg;
	}
	else
	{
		*info->pEnReg &= ~info->EnMask;
	}
}

static void Stm32SpiRccReset(Stm32SpiDev_t * const pDev)
{
	const Stm32SpiInfo_t *info = pDev->pInfo;

	*info->pRstReg |= info->RstMask;
	*info->pRstReg &= ~info->RstMask;
}

// Bytes of one data item
static inline int Stm32SpiItemSize(Stm32SpiDev_t * const pDev)
{
	return pDev->pSpiDev->Cfg.DataSize > 8 ? 2 : 1;
}

// Wait bound in loop counts without progress: the time of a data item plus
// a margin.
static uint32_t Stm32SpiTimeout(Stm32SpiDev_t * const pDev)
{
	uint32_t rate = pDev->pSpiDev->Cfg.Rate;

	if (rate == 0)
	{
		rate = 1;
	}

	return (SystemCoreClock / rate + 1U) * (pDev->pSpiDev->Cfg.DataSize + 2U) + 256U;
}

#ifdef STM32_SPI_LAYOUT_CR
// Empty the receive FIFO, one byte at a time
static void Stm32SpiFlushRx(SPI_TypeDef * const pReg)
{
	for (int i = 0; i < 8 && (pReg->SR & SPI_SR_RXNE); i++)
	{
		(void)*(volatile uint8_t*)&pReg->DR;
	}
}
#endif

// ---------------------------------------------------------------------------
// DMA
// ---------------------------------------------------------------------------

#ifdef STM32_SPI_DMA
static void Stm32SpiDmaIrq(int Ch, void *pCtx);
static void Stm32SpiService(Stm32SpiDev_t * const pDev);
static void Stm32SpiDmaGive(Stm32SpiDev_t * const pDev);

// Take the two channels. Both or none.
static bool Stm32SpiDmaTake(Stm32SpiDev_t * const pDev, bool bIrq, int IntPrio)
{
	const Stm32SpiInfo_t *info = pDev->pInfo;
	const int width = Stm32SpiItemSize(pDev) - 1;

	if (pDev->DmaTx < 0 || pDev->DmaRx < 0)
	{
		Stm32SpiDmaGive(pDev);
		pDev->DmaTx = WbaGpdmaAlloc(Stm32SpiDmaIrq, pDev, IntPrio, bIrq);
		if (pDev->DmaTx < 0)
		{
			return false;
		}
		pDev->DmaRx = WbaGpdmaAlloc(Stm32SpiDmaIrq, pDev, IntPrio, bIrq);
		if (pDev->DmaRx < 0)
		{
			WbaGpdmaFree(pDev->DmaTx);
			pDev->DmaTx = -1;

			return false;
		}
	}

	WbaGpdmaSetup(pDev->DmaTx, info->TxReq, true, &pDev->pReg->TXDR, width, bIrq);
	WbaGpdmaSetup(pDev->DmaRx, info->RxReq, false, &pDev->pReg->RXDR, width, bIrq);

	return true;
}

static void Stm32SpiDmaGive(Stm32SpiDev_t * const pDev)
{
	if (pDev->DmaTx >= 0)
	{
		WbaGpdmaFree(pDev->DmaTx);
		pDev->DmaTx = -1;
	}
	if (pDev->DmaRx >= 0)
	{
		WbaGpdmaFree(pDev->DmaRx);
		pDev->DmaRx = -1;
	}
}

static void Stm32SpiDmaIrq(int Ch, void *pCtx)
{
	Stm32SpiDev_t *dev = (Stm32SpiDev_t *)pCtx;

	WbaGpdmaAck(Ch);
	// A master transfer that the caller waits for is followed by its polling loop
	if (dev->bAsync || dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		Stm32SpiService(dev);
	}
}

static void Stm32SpiDmaStop(Stm32SpiDev_t * const pDev)
{
	if (pDev->DmaTx >= 0)
	{
		WbaGpdmaStop(pDev->DmaTx);
	}
	if (pDev->DmaRx >= 0)
	{
		WbaGpdmaStop(pDev->DmaRx);
	}
}
#endif

// ---------------------------------------------------------------------------
// Master engine
// ---------------------------------------------------------------------------

// Bytes done of a transfer that failed
static int Stm32SpiMasterCount(Stm32SpiDev_t * const pDev)
{
	const int isz = Stm32SpiItemSize(pDev);
	uint32_t items = pDev->RxN;

#ifdef STM32_SPI_DMA
	if (pDev->bDma)
	{
		items = pDev->Chunk - WbaGpdmaRemain(pDev->DmaRx) / isz;
	}
#endif

	return (int)((pDev->Done + items) * isz);
}

#ifdef STM32_SPI_LAYOUT_CFG
// End of the transfer. Interrupts and DMA off, flags cleared, the result
// stored.
static void Stm32SpiMasterEnd(Stm32SpiDev_t * const pDev, bool bad)
{
	SPI_TypeDef *reg = pDev->pReg;

	pDev->Result = bad ? Stm32SpiMasterCount(pDev) : (int)(pDev->Items * Stm32SpiItemSize(pDev));
	reg->IER = 0;
	reg->CR1 &= ~SPI_CR1_SPE;
	reg->IFCR = STM32_SPI_IFCR_ALL;
#ifdef STM32_SPI_DMA
	if (pDev->bDma)
	{
		reg->CFG1 &= ~STM32_SPI_CFG1_DMA;
		Stm32SpiDmaStop(pDev);
	}
#endif
	pDev->bRun = false;
}

// Start the next chunk of the transfer
static void Stm32SpiMasterChunk(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	const int isz = Stm32SpiItemSize(pDev);
	uint32_t left = pDev->Items - pDev->Done;
	uint32_t max = STM32_SPI_TSIZE_MAX;

#ifdef STM32_SPI_DMA
	if (pDev->bDma && max > 0xFFFFU / isz)
	{
		max = 0xFFFFU / isz;
	}
#endif
	pDev->Chunk = left > max ? max : left;
	pDev->TxN = 0;
	pDev->RxN = 0;

	reg->CR1 &= ~SPI_CR1_SPE;
	reg->IFCR = STM32_SPI_IFCR_ALL;
	reg->CR2 = pDev->Chunk;

#ifdef STM32_SPI_DMA
	if (pDev->bDma)
	{
		const uint32_t bytes = pDev->Chunk * isz;

		if (pDev->pTx)
		{
			WbaGpdmaMemInc(pDev->DmaTx, true);
			WbaGpdmaStart(pDev->DmaTx, pDev->pTx + pDev->Done * isz, bytes);
		}
		else
		{
			WbaGpdmaMemInc(pDev->DmaTx, false);
			WbaGpdmaStart(pDev->DmaTx, &pDev->Dummy, bytes);
		}
		if (pDev->pRx)
		{
			WbaGpdmaMemInc(pDev->DmaRx, true);
			WbaGpdmaStart(pDev->DmaRx, pDev->pRx + pDev->Done * isz, bytes);
		}
		else
		{
			WbaGpdmaMemInc(pDev->DmaRx, false);
			WbaGpdmaStart(pDev->DmaRx, &pDev->Sink, bytes);
		}
		reg->CFG1 |= STM32_SPI_CFG1_DMA;
	}
#endif
	if (pDev->bAsync)
	{
		reg->IER = STM32_SPI_IER_MASTER | (pDev->bDma ? 0U : STM32_SPI_IER_DATA);
	}
	reg->CR1 |= SPI_CR1_SPE;
	reg->CR1 |= SPI_CR1_CSTART;
}

// Move the items the flags allow
static void Stm32SpiMasterPump(Stm32SpiDev_t * const pDev, uint32_t Sr)
{
	SPI_TypeDef *reg = pDev->pReg;
	const bool b16 = pDev->pSpiDev->Cfg.DataSize > 8;
	const uint32_t fifo = STM32_SPI_FIFO_BYTES / (b16 ? 2 : 1);
	const uint32_t first = pDev->Done;

	if ((Sr & SPI_SR_TXP) && pDev->TxN < pDev->Chunk && (pDev->TxN - pDev->RxN) < fifo)
	{
		const uint32_t i = first + pDev->TxN;

		if (b16)
		{
			uint16_t d = pDev->pTx ? (uint16_t)(pDev->pTx[i * 2] | ((uint16_t)pDev->pTx[i * 2 + 1] << 8)) :
									 pDev->Dummy;

			*(Stm32SpiReg16_t*)&reg->TXDR = d;
		}
		else
		{
			*(volatile uint8_t*)&reg->TXDR = pDev->pTx ? pDev->pTx[i] : (uint8_t)pDev->Dummy;
		}
		pDev->TxN++;
	}

	if ((Sr & SPI_SR_RXP) && pDev->RxN < pDev->Chunk)
	{
		const uint32_t i = first + pDev->RxN;

		if (b16)
		{
			uint16_t d = *(Stm32SpiReg16_t*)&reg->RXDR;

			if (pDev->pRx)
			{
				pDev->pRx[i * 2] = (uint8_t)d;
				pDev->pRx[i * 2 + 1] = (uint8_t)(d >> 8);
			}
		}
		else
		{
			uint8_t d = *(volatile uint8_t*)&reg->RXDR;

			if (pDev->pRx)
			{
				pDev->pRx[i] = d;
			}
		}
		pDev->RxN++;
	}
}

// One step of the transfer from the flags. Called by the polling loop and by
// the interrupt.
static void Stm32SpiMasterService(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	uint32_t sr = reg->SR;

#ifdef STM32_SPI_DMA
	if (pDev->bDma && ((WbaGpdmaFlags(pDev->DmaTx) | WbaGpdmaFlags(pDev->DmaRx)) & WBA_GPDMA_FLAG_ERR))
	{
		pDev->bFail = true;
		Stm32SpiMasterEnd(pDev, true);

		return;
	}
#endif

	if (sr & STM32_SPI_ERR_FLAGS)
	{
		pDev->bFail = true;
		Stm32SpiMasterEnd(pDev, true);

		return;
	}

	if (pDev->bDma == false)
	{
		Stm32SpiMasterPump(pDev, sr);
		sr = reg->SR;
	}

	if (sr & SPI_SR_EOT)
	{
		bool done;

		if (pDev->bDma)
		{
#ifdef STM32_SPI_DMA
			done = (WbaGpdmaFlags(pDev->DmaRx) & WBA_GPDMA_FLAG_TC) != 0U;
#else
			done = false;
#endif
		}
		else
		{
			done = pDev->TxN >= pDev->Chunk && pDev->RxN >= pDev->Chunk;
		}
		if (done)
		{
			pDev->Done += pDev->Chunk;
			if (pDev->Done < pDev->Items)
			{
				Stm32SpiMasterChunk(pDev);
			}
			else
			{
				Stm32SpiMasterEnd(pDev, false);
			}
		}
	}
}

#else
// End of the transfer. Interrupts off, the result stored. After an error the
// receive FIFO is emptied and the peripheral restarted, so that the next
// transfer starts clean.
static void Stm32SpiMasterEnd(Stm32SpiDev_t * const pDev, bool bad)
{
	SPI_TypeDef *reg = pDev->pReg;

	pDev->Result = bad ? Stm32SpiMasterCount(pDev) : (int)(pDev->Items * Stm32SpiItemSize(pDev));
	reg->CR2 &= ~STM32_SPI_CR2_INT;
	if (bad)
	{
		reg->CR1 &= ~SPI_CR1_SPE;
		Stm32SpiFlushRx(reg);
		(void)reg->SR;
		reg->CR1 = pDev->Cr1 | SPI_CR1_SPE;
	}
	pDev->bRun = false;
}

// Move the items the flags allow. Received items are read first: each one
// read makes room for one more in flight. At most a FIFO of items is in
// flight, so the receive FIFO cannot overflow.
static void Stm32SpiMasterPump(Stm32SpiDev_t * const pDev, uint32_t Sr)
{
	SPI_TypeDef *reg = pDev->pReg;
	const bool b16 = pDev->pSpiDev->Cfg.DataSize > 8;
	const uint32_t fifo = STM32_SPI_FIFO_BYTES / (b16 ? 2 : 1);
	const uint32_t first = pDev->Done;

	while ((Sr & SPI_SR_RXNE) && pDev->RxN < pDev->Chunk)
	{
		const uint32_t i = first + pDev->RxN;

		if (b16)
		{
			uint16_t d = *(Stm32SpiReg16_t*)&reg->DR;

			if (pDev->pRx)
			{
				pDev->pRx[i * 2] = (uint8_t)d;
				pDev->pRx[i * 2 + 1] = (uint8_t)(d >> 8);
			}
		}
		else
		{
			uint8_t d = *(volatile uint8_t*)&reg->DR;

			if (pDev->pRx)
			{
				pDev->pRx[i] = d;
			}
		}
		pDev->RxN++;
		Sr = reg->SR;
	}

	while ((Sr & SPI_SR_TXE) && pDev->TxN < pDev->Chunk && (pDev->TxN - pDev->RxN) < fifo)
	{
		const uint32_t i = first + pDev->TxN;

		if (b16)
		{
			uint16_t d = pDev->pTx ? (uint16_t)(pDev->pTx[i * 2] | ((uint16_t)pDev->pTx[i * 2 + 1] << 8)) :
									 pDev->Dummy;

			*(Stm32SpiReg16_t*)&reg->DR = d;
		}
		else
		{
			*(volatile uint8_t*)&reg->DR = pDev->pTx ? pDev->pTx[i] : (uint8_t)pDev->Dummy;
		}
		pDev->TxN++;
		Sr = reg->SR;
	}
}

// Start the transfer. The peripheral has no transfer size: the whole transfer
// is one chunk, it ends when the last item is received. With the interrupt
// the first items are loaded here, the receive interrupt then keeps the
// transfer going.
static void Stm32SpiMasterChunk(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;

	pDev->Chunk = pDev->Items - pDev->Done;
	pDev->TxN = 0;
	pDev->RxN = 0;

	if ((reg->CR1 & SPI_CR1_SPE) == 0)
	{
		reg->CR1 |= SPI_CR1_SPE;
	}
	if (pDev->bAsync)
	{
		Stm32SpiMasterPump(pDev, reg->SR);
		reg->CR2 |= SPI_CR2_RXNEIE | SPI_CR2_ERRIE;
	}
}

// One step of the transfer from the flags. Called by the polling loop and by
// the interrupt.
static void Stm32SpiMasterService(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	uint32_t sr = reg->SR;

	if (sr & STM32_SPI_ERR_FLAGS)
	{
		pDev->bFail = true;
		Stm32SpiMasterEnd(pDev, true);

		return;
	}

	Stm32SpiMasterPump(pDev, sr);

	if (pDev->RxN >= pDev->Chunk)
	{
		// The last item is in, the clock ends with it: BSY clears within a
		// bit time
		for (uint32_t n = Stm32SpiTimeout(pDev); n > 0 && (reg->SR & SPI_SR_BSY); n--)
		{
		}
		pDev->Done += pDev->Chunk;
		Stm32SpiMasterEnd(pDev, false);
	}
}
#endif

// Polling loop of the engine. The wait restarts whenever the transfer
// progresses.
static void Stm32SpiMasterRun(Stm32SpiDev_t * const pDev)
{
	const uint32_t budget = Stm32SpiTimeout(pDev);
	uint32_t left = budget;
	uint32_t last = 0xFFFFFFFFU;

	while (pDev->bRun)
	{
		Stm32SpiMasterService(pDev);

		uint32_t sig = pDev->Done * 3U + pDev->TxN * 5U + pDev->RxN * 7U;

#ifdef STM32_SPI_DMA
		if (pDev->bDma && pDev->bRun)
		{
			sig += WbaGpdmaRemain(pDev->DmaRx) * 11U + WbaGpdmaRemain(pDev->DmaTx) * 13U;
		}
#endif
		if (sig != last)
		{
			last = sig;
			left = budget;
		}
		else if (--left == 0)
		{
			pDev->bFail = true;
			Stm32SpiMasterEnd(pDev, true);
		}
	}
}

// An interrupt driven transfer ended
static void Stm32SpiAsyncDone(Stm32SpiDev_t * const pDev)
{
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
	const int count = pDev->Result;

	pDev->bAsync = false;
	if (pDev->bRx)
	{
		DeviceIntrfStopRx(intrf);
	}
	else
	{
		DeviceIntrfTxComplete(intrf);
	}
	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, NULL, count);
	}
}

// Run a transfer of Len bytes. pTx is the data to send, or NULL to send the
// dummy value. pRx is the buffer for the data received, or NULL to discard
// it. Return the bytes done, or -1 when the end is reported by the event.
static int Stm32SpiTransfer(Stm32SpiDev_t * const pDev, const uint8_t *pTx, uint8_t *pRx, int Len, bool bAsync)
{
	const int isz = Stm32SpiItemSize(pDev);
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;

	if (pDev->bRun || Len < isz)
	{
		return 0;
	}

	pDev->pTx = pTx;
	pDev->pRx = pRx;
	pDev->Items = (uint32_t)Len / isz;
	pDev->Done = 0;
	pDev->bFail = false;
	pDev->bAsync = bAsync;
	pDev->bRx = pTx == NULL;
	pDev->bRun = true;
	pDev->Result = 0;
#ifdef STM32_SPI_DMA
	pDev->bDma = intrf->bDma && pDev->DmaTx >= 0 && pDev->DmaRx >= 0;
#else
	pDev->bDma = false;
#endif
	if (bAsync)
	{
		atomic_store(&intrf->bTxReady, false);
	}

	Stm32SpiMasterChunk(pDev);
	if (bAsync)
	{
		return -1;
	}

	Stm32SpiMasterRun(pDev);

	return pDev->Result;
}

// ---------------------------------------------------------------------------
// Slave engine
// ---------------------------------------------------------------------------

#ifdef STM32_SPI_LAYOUT_CFG
static void Stm32SpiSlaveWrite(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	const bool b16 = spi->Cfg.DataSize > 8;
	const uint8_t *data = spi->pTxData[0];

	while ((reg->SR & SPI_SR_TXP) && pDev->STx < pDev->STxMax)
	{
		const uint32_t i = pDev->STx;

		if (b16)
		{
			*(Stm32SpiReg16_t*)&reg->TXDR = (uint16_t)(data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8));
		}
		else
		{
			*(volatile uint8_t*)&reg->TXDR = data[i];
		}
		pDev->STx++;
	}
	if (pDev->STx >= pDev->STxMax)
	{
		reg->IER &= ~SPI_IER_TXPIE;
	}
}

static void Stm32SpiSlaveRead(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	const bool b16 = spi->Cfg.DataSize > 8;
	uint8_t *buf = spi->pRxBuff[0];

	while (reg->SR & SPI_SR_RXP)
	{
		uint16_t d;

		if (b16)
		{
			d = *(Stm32SpiReg16_t*)&reg->RXDR;
		}
		else
		{
			d = *(volatile uint8_t*)&reg->RXDR;
		}
		if (pDev->SRx < pDev->SRxMax)
		{
			if (b16)
			{
				buf[pDev->SRx * 2] = (uint8_t)d;
				buf[pDev->SRx * 2 + 1] = (uint8_t)(d >> 8);
			}
			else
			{
				buf[pDev->SRx] = (uint8_t)d;
			}
		}
		else
		{
			pDev->bBad = true;
		}
		pDev->SRx++;
	}
}

// Prepare the next frame: the handler gives the buffers, the peripheral is
// enabled and waits for NSS.
static void Stm32SpiSlaveArm(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	const int isz = Stm32SpiItemSize(pDev);

	spi->pRxBuff[0] = NULL;
	spi->RxBuffLen[0] = 0;
	spi->pTxData[0] = NULL;
	spi->TxDataLen[0] = 0;
	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_STATECHG, NULL, 0);
	}

	pDev->bBad = false;
	pDev->bFrame = false;
	pDev->SRx = 0;
	pDev->STx = 0;
	pDev->SRxMax = spi->pRxBuff[0] != NULL && spi->RxBuffLen[0] > 0 ? (uint32_t)spi->RxBuffLen[0] / isz : 0;
	pDev->STxMax = spi->pTxData[0] != NULL && spi->TxDataLen[0] > 0 ? (uint32_t)spi->TxDataLen[0] / isz : 0;

	reg->CR1 &= ~SPI_CR1_SPE;
	reg->IFCR = STM32_SPI_IFCR_ALL;
	reg->CR2 = 0;
	reg->CFG1 &= ~STM32_SPI_CFG1_DMA;

	uint32_t ier = STM32_SPI_IER_SLAVE;

#ifdef STM32_SPI_DMA
	if (pDev->bDma)
	{
		if (pDev->SRxMax > 0)
		{
			WbaGpdmaMemInc(pDev->DmaRx, true);
			WbaGpdmaStart(pDev->DmaRx, spi->pRxBuff[0], pDev->SRxMax * isz);
			reg->CFG1 |= SPI_CFG1_RXDMAEN;
		}
		else
		{
			ier |= SPI_IER_RXPIE;
		}
		if (pDev->STxMax > 0)
		{
			WbaGpdmaMemInc(pDev->DmaTx, true);
			WbaGpdmaStart(pDev->DmaTx, spi->pTxData[0], pDev->STxMax * isz);
			reg->CFG1 |= SPI_CFG1_TXDMAEN;
		}
	}
	else
#endif
	{
		ier |= SPI_IER_RXPIE;
		if (pDev->STxMax > 0)
		{
			ier |= SPI_IER_TXPIE;
		}
	}

	reg->IER = ier;
	reg->CR1 |= SPI_CR1_SPE;
	if (pDev->bDma == false && pDev->STxMax > 0)
	{
		Stm32SpiSlaveWrite(pDev);
	}
}

// NSS went inactive: the frame is over
static void Stm32SpiSlaveEnd(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	const int isz = Stm32SpiItemSize(pDev);
	int count;

	if (pDev->bDma)
	{
#ifdef STM32_SPI_DMA
		// The channel takes what the FIFO still holds
		for (uint32_t n = Stm32SpiTimeout(pDev); n > 0 && (reg->SR & SPI_SR_RXP); n--)
		{
		}
		if (pDev->SRxMax > 0)
		{
			pDev->SRx = pDev->SRxMax - WbaGpdmaRemain(pDev->DmaRx) / isz;
			if (reg->SR & SPI_SR_RXP)
			{
				// Items are left that the channel did not take: the frame
				// is longer than the receive buffer
				pDev->bBad = true;
			}
		}
		else
		{
			Stm32SpiSlaveRead(pDev);
		}
#endif
	}
	else
	{
		Stm32SpiSlaveRead(pDev);
	}

	if (reg->SR & SPI_SR_OVR)
	{
		pDev->bBad = true;
	}
	count = pDev->bBad ? -1 : (int)(pDev->SRx * isz);

	reg->IER = 0;
	reg->CR1 &= ~SPI_CR1_SPE;
	reg->IFCR = STM32_SPI_IFCR_ALL;
#ifdef STM32_SPI_DMA
	if (pDev->bDma)
	{
		Stm32SpiDmaStop(pDev);
	}
#endif

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, spi->pRxBuff[0], count);
	}

	Stm32SpiSlaveArm(pDev);
}

// TXC stays set while the transmit FIFO is empty and EOTIE raises the
// interrupt for it, so the handler would never leave. One underrun item in the
// FIFO clears TXC until it is sent: the interrupt then comes once per item.
static bool Stm32SpiSlaveTxDone(Stm32SpiDev_t * const pDev)
{
#ifdef STM32_SPI_DMA
	if (pDev->bDma && pDev->STxMax > 0)
	{
		return WbaGpdmaRemain(pDev->DmaTx) == 0U;
	}
#endif

	return pDev->bDma ? true : pDev->STx >= pDev->STxMax;
}

static void Stm32SpiSlaveFeed(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;

	if (pDev->pSpiDev->Cfg.DataSize > 8)
	{
		*(Stm32SpiReg16_t*)&reg->TXDR = pDev->Dummy;
	}
	else
	{
		*(volatile uint8_t*)&reg->TXDR = (uint8_t)pDev->Dummy;
	}
}

static void Stm32SpiSlaveService(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	uint32_t sr = reg->SR;

	if (sr & SPI_SR_OVR)
	{
		pDev->bBad = true;
		reg->IFCR = SPI_IFCR_OVRC;
	}
	if (sr & SPI_SR_UDR)
	{
		// The DummyByte goes out after the transmit data
		reg->IFCR = SPI_IFCR_UDRC;
	}

	if (pDev->bDma == false)
	{
		if (sr & SPI_SR_RXP)
		{
			Stm32SpiSlaveRead(pDev);
		}
		if (sr & SPI_SR_TXP)
		{
			if (pDev->STx < pDev->STxMax)
			{
				Stm32SpiSlaveWrite(pDev);
			}
			else
			{
				// Nothing left to load, TXP stays set while the FIFO has room
				reg->IER &= ~SPI_IER_TXPIE;
			}
		}
	}
	else if (pDev->SRxMax == 0 && (sr & SPI_SR_RXP))
	{
		Stm32SpiSlaveRead(pDev);
	}

	if (sr & SPI_SR_EOT)
	{
		Stm32SpiSlaveEnd(pDev);
	}
	else if ((sr & SPI_SR_TXC) && Stm32SpiSlaveTxDone(pDev))
	{
		Stm32SpiSlaveFeed(pDev);
	}
}

#else
// Load the transmit FIFO up to its threshold: the data of the frame, then
// the DummyByte. The peripheral has no underrun value, the FIFO is kept
// filled so that the DummyByte goes out after the data.
static void Stm32SpiSlaveWrite(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	const bool b16 = spi->Cfg.DataSize > 8;
	const uint8_t *data = spi->pTxData[0];

	while (reg->SR & SPI_SR_TXE)
	{
		uint16_t d = pDev->Dummy;

		if (pDev->STx < pDev->STxMax)
		{
			const uint32_t i = pDev->STx;

			d = b16 ? (uint16_t)(data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8)) : data[i];
			pDev->STx++;
		}
		if (b16)
		{
			*(Stm32SpiReg16_t*)&reg->DR = d;
		}
		else
		{
			*(volatile uint8_t*)&reg->DR = (uint8_t)d;
		}
	}
}

static void Stm32SpiSlaveRead(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	const bool b16 = spi->Cfg.DataSize > 8;
	uint8_t *buf = spi->pRxBuff[0];

	while (reg->SR & SPI_SR_RXNE)
	{
		uint16_t d;

		if (b16)
		{
			d = *(Stm32SpiReg16_t*)&reg->DR;
		}
		else
		{
			d = *(volatile uint8_t*)&reg->DR;
		}
		if (pDev->SRx < pDev->SRxMax)
		{
			if (b16)
			{
				buf[pDev->SRx * 2] = (uint8_t)d;
				buf[pDev->SRx * 2 + 1] = (uint8_t)(d >> 8);
			}
			else
			{
				buf[pDev->SRx] = (uint8_t)d;
			}
		}
		else
		{
			pDev->bBad = true;
		}
		pDev->SRx++;
	}
}

// Prepare the next frame: the handler gives the buffers, the peripheral is
// enabled with the transmit FIFO loaded and waits for NSS.
static void Stm32SpiSlaveArm(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	const int isz = Stm32SpiItemSize(pDev);

	spi->pRxBuff[0] = NULL;
	spi->RxBuffLen[0] = 0;
	spi->pTxData[0] = NULL;
	spi->TxDataLen[0] = 0;
	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_STATECHG, NULL, 0);
	}

	pDev->bBad = false;
	pDev->bFrame = false;
	pDev->SRx = 0;
	pDev->STx = 0;
	pDev->SRxMax = spi->pRxBuff[0] != NULL && spi->RxBuffLen[0] > 0 ? (uint32_t)spi->RxBuffLen[0] / isz : 0;
	pDev->STxMax = spi->pTxData[0] != NULL && spi->TxDataLen[0] > 0 ? (uint32_t)spi->TxDataLen[0] / isz : 0;

	// Clearing SPE leaves the FIFO content: the items loaded for the frame
	// that ended would go out in the next one. The RCC reset empties the
	// FIFOs, the registers are written again.
	Stm32SpiRccReset(pDev);
	reg->CR1 = pDev->Cr1;
	reg->CR2 = pDev->Cr2;
	reg->CR1 |= SPI_CR1_SPE;
	Stm32SpiSlaveWrite(pDev);
	reg->CR2 |= STM32_SPI_CR2_INT;
}

// NSS went inactive: the frame is over
static void Stm32SpiSlaveEnd(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	const int isz = Stm32SpiItemSize(pDev);
	int count;

	Stm32SpiSlaveRead(pDev);
	if (reg->SR & SPI_SR_OVR)
	{
		pDev->bBad = true;
	}
	count = pDev->bBad ? -1 : (int)(pDev->SRx * isz);

	reg->CR2 &= ~STM32_SPI_CR2_INT;
	reg->CR1 &= ~SPI_CR1_SPE;

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, spi->pRxBuff[0], count);
	}

	Stm32SpiSlaveArm(pDev);
}

static void Stm32SpiSlaveService(Stm32SpiDev_t * const pDev)
{
	SPI_TypeDef *reg = pDev->pReg;
	uint32_t sr = reg->SR;

	if (sr & SPI_SR_OVR)
	{
		// Cleared by reading DR then SR
		pDev->bBad = true;
		Stm32SpiFlushRx(reg);
		(void)reg->SR;
	}
	if (sr & SPI_SR_RXNE)
	{
		Stm32SpiSlaveRead(pDev);
	}
	if (sr & SPI_SR_TXE)
	{
		Stm32SpiSlaveWrite(pDev);
	}
}
#endif

// The peripheral raises no EOT when NSS goes inactive while the transfer
// size is open, so the pin event ends the frame.
static bool Stm32SpiSlaveActive(Stm32SpiDev_t * const pDev)
{
	bool active = pDev->bFrame || pDev->SRx > 0U || (pDev->pReg->SR & STM32_SPI_SR_RXNE) != 0U;

#ifdef STM32_SPI_DMA
	if (pDev->bDma && pDev->SRxMax > 0U)
	{
		active = active || WbaGpdmaRemain(pDev->DmaRx) < pDev->SRxMax * (uint32_t)Stm32SpiItemSize(pDev);
	}
#endif

	return active;
}

static void Stm32SpiNssEvt(int IntNo, void *pCtx)
{
	Stm32SpiDev_t *dev = (Stm32SpiDev_t*)pCtx;
	const IOPinCfg_t &cs = dev->pSpiDev->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX];

	(void)IntNo;
	if (IOPinRead(cs.PortNo, cs.PinNo) == 0)
	{
		dev->bFrame = true;
	}
	else if (Stm32SpiSlaveActive(dev))
	{
		Stm32SpiSlaveEnd(dev);
	}
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

static void Stm32SpiService(Stm32SpiDev_t * const pDev)
{
	if (pDev->pSpiDev == NULL)
	{
		return;
	}

	if (pDev->bSlave)
	{
		Stm32SpiSlaveService(pDev);

		return;
	}

	if (pDev->bRun == false)
	{
		return;
	}

	Stm32SpiMasterService(pDev);
	if (pDev->bRun == false && pDev->bAsync)
	{
		Stm32SpiAsyncDone(pDev);
	}
}

// ---------------------------------------------------------------------------
// Device interface
// ---------------------------------------------------------------------------

// Set the closest rate the MBR divider gives, 2 to 256. Return the rate set.
static uint32_t Stm32SpiSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return 0;
	}

	uint32_t clk = SystemPeriphClockGet(dev->pInfo->ClkIdx);

	if (Rate == 0)
	{
		Rate = STM32_SPI_RATE_DEFAULT;
	}

	// Closest rate, the lower one when two are as close
	uint32_t best = 0;
	uint32_t bestdiff = 0xFFFFFFFFU;

	for (uint32_t mbr = 0; mbr <= STM32_SPI_MBR_MAX; mbr++)
	{
		uint32_t r = clk >> (mbr + 1);
		uint32_t diff = r > Rate ? r - Rate : Rate - r;

		if (diff <= bestdiff)
		{
			bestdiff = diff;
			best = mbr;
		}
	}

	SPI_TypeDef *reg = dev->pReg;
	bool spe = (reg->CR1 & SPI_CR1_SPE) != 0;

#ifdef STM32_SPI_LAYOUT_CFG
	// MBR can only change with the peripheral disabled
	reg->CR1 &= ~SPI_CR1_SPE;
	reg->CFG1 = (reg->CFG1 & ~(SPI_CFG1_MBR_Msk | SPI_CFG1_BPASS)) | (best << SPI_CFG1_MBR_Pos);
#else
	// BR can only change with the peripheral disabled
	reg->CR1 &= ~SPI_CR1_SPE;
	reg->CR1 = (reg->CR1 & ~SPI_CR1_BR_Msk) | (best << SPI_CR1_BR_Pos);
	dev->Cr1 = (dev->Cr1 & ~SPI_CR1_BR_Msk) | (best << SPI_CR1_BR_Pos);
#endif
	if (spe)
	{
		reg->CR1 |= SPI_CR1_SPE;
	}
	dev->pSpiDev->Cfg.Rate = clk >> (best + 1);

	return dev->pSpiDev->Cfg.Rate;
}

static uint32_t Stm32SpiGetRate(DevIntrf_t * const pIntrf)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	return dev ? dev->pSpiDev->Cfg.Rate : 0;
}

static void Stm32SpiReset(DevIntrf_t * const pIntrf)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev == NULL)
	{
		return;
	}

#ifdef STM32_SPI_LAYOUT_CFG
	// SPE low resets the state machine, the configuration is kept
	dev->pReg->IER = 0;
	dev->pReg->CR1 &= ~SPI_CR1_SPE;
	dev->pReg->CFG1 &= ~STM32_SPI_CFG1_DMA;
	dev->pReg->IFCR = STM32_SPI_IFCR_ALL;
#else
	// Interrupts off, the receive FIFO emptied and the errors cleared. The
	// master sets SPE again, the slave is armed below.
	dev->pReg->CR2 &= ~STM32_SPI_CR2_INT;
	dev->pReg->CR1 &= ~SPI_CR1_SPE;
	Stm32SpiFlushRx(dev->pReg);
	(void)dev->pReg->SR;
	if (dev->bSlave == false)
	{
		dev->pReg->CR1 = dev->Cr1 | SPI_CR1_SPE;
	}
#endif
#ifdef STM32_SPI_DMA
	Stm32SpiDmaStop(dev);
#endif
	dev->bRun = false;
	dev->bAsync = false;
	atomic_store(&pIntrf->bTxReady, true);
	if (dev->bSlave)
	{
		Stm32SpiSlaveArm(dev);
	}
}

static void Stm32SpiEnable(DevIntrf_t * const pIntrf)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev)
	{
		Stm32SpiClkEnable(dev, true);
	}
}

static void Stm32SpiDisable(DevIntrf_t * const pIntrf)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev && dev->bSlave == false)
	{
		dev->pReg->CR1 &= ~SPI_CR1_SPE;
		Stm32SpiClkEnable(dev, false);
	}
}

static void Stm32SpiPowerOff(DevIntrf_t * const pIntrf)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev)
	{
#ifdef STM32_SPI_LAYOUT_CFG
		dev->pReg->IER = 0;
#else
		dev->pReg->CR2 &= ~STM32_SPI_CR2_INT;
#endif
		dev->pReg->CR1 &= ~SPI_CR1_SPE;
		NVIC_DisableIRQ(dev->pInfo->Irq);
		if (dev->bNssInt)
		{
			IOPinDisableInterrupt(dev->NssLine);
			dev->bNssInt = false;
		}
#ifdef STM32_SPI_DMA
		Stm32SpiDmaGive(dev);
#endif
		Stm32SpiClkEnable(dev, false);
		IOPinDis(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);
	}
}

static void *Stm32SpiGetHandle(DevIntrf_t * const pIntrf)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	return dev != NULL ? dev->pSpiDev : NULL;
}

static void Stm32SpiStop(DevIntrf_t * const pIntrf);

// Open a transfer: select the chip. StartRx can follow StartTx for the data
// phase of a read, the chip select then stays low.
static bool Stm32SpiStart(DevIntrf_t * const pIntrf, uint32_t DevCs)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev == NULL || dev->bSlave || dev->bRun)
	{
		return false;
	}

	if (dev->bCsDrive)
	{
		const SPICfg_t *cfg = &dev->pSpiDev->Cfg;

		if (DevCs >= (uint32_t)(cfg->NbIOPins - SPI_CS_IOPIN_IDX))
		{
			return false;
		}
		if (dev->bCsLow && dev->pSpiDev->CurDevCs == (int)DevCs)
		{
			// Data phase of a read, the chip select is already low
			return true;
		}
		if (dev->bCsLow)
		{
			// Another device was still selected
			Stm32SpiStop(pIntrf);
		}

		dev->pSpiDev->CurDevCs = DevCs;
		IOPinClear(cfg->pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
				   cfg->pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);
		dev->bCsLow = true;
	}

	return true;
}

// Close a transfer: release the chip select
static void Stm32SpiStop(DevIntrf_t * const pIntrf)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev && dev->bCsLow)
	{
		const SPICfg_t *cfg = &dev->pSpiDev->Cfg;
		int cs = dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX;

		IOPinSet(cfg->pIOPinMap[cs].PortNo, cfg->pIOPinMap[cs].PinNo);
		dev->bCsLow = false;
	}
}

static int Stm32SpiTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev == NULL || dev->bSlave || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	if (pIntrf->bIntEn && DataLen <= SPI_STM32_TXBUF_SIZE && dev->bRun == false)
	{
		memcpy(dev->TxBuf, pData, DataLen);

		return Stm32SpiTransfer(dev, dev->TxBuf, NULL, DataLen, true);
	}

	return Stm32SpiTransfer(dev, pData, NULL, DataLen, false);
}

static int Stm32SpiRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	Stm32SpiDev_t *dev = Stm32SpiHandle(pIntrf);

	if (dev == NULL || dev->bSlave || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	return Stm32SpiTransfer(dev, NULL, pBuff, BuffLen, pIntrf->bIntEn);
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= STM32_SPI_MAXDEV)
	{
		return false;
	}

#ifdef STM32_SPI_LAYOUT_CR
	// An instance the part does not have has no register block
	if (s_Stm32SpiInfo[pCfgData->DevNo].pReg == NULL)
	{
		return false;
	}
#endif

	const bool slave = pCfgData->Mode == SPIMODE_SLAVE;

	if ((pCfgData->Mode != SPIMODE_MASTER && slave == false) || pCfgData->Phy != SPIPHY_NORMAL ||
		pCfgData->DataSize < STM32_SPI_DATASIZE_MIN || pCfgData->DataSize > STM32_SPI_DATASIZE_MAX ||
		pCfgData->pIOPinMap == NULL || pCfgData->NbIOPins < SPI_CS_IOPIN_IDX + (slave ? 1 : 0))
	{
		return false;
	}

	Stm32SpiDev_t *dev = &s_Stm32SpiDev[pCfgData->DevNo];
	SPI_TypeDef *reg = s_Stm32SpiInfo[pCfgData->DevNo].pReg;

	// An instance running for another SPI object is not taken over
	if (dev->pSpiDev != NULL && dev->pSpiDev != pDev && (reg->CR1 & SPI_CR1_SPE))
	{
		return false;
	}

	if (dev->pSpiDev == NULL)
	{
		dev->DmaTx = -1;
		dev->DmaRx = -1;
	}

	dev->DevNo = pCfgData->DevNo;
	dev->pInfo = &s_Stm32SpiInfo[pCfgData->DevNo];
	dev->pReg = reg;
	if (dev->bNssInt)
	{
		IOPinDisableInterrupt(dev->NssLine);
	}
	dev->bNssInt = false;
	dev->bFrame = false;
	dev->bSlave = slave;
	dev->Dummy = pCfgData->DataSize > 8 ? (uint16_t)(pCfgData->DummyByte * 0x0101U) : pCfgData->DummyByte;
	dev->Sink = 0;

	memcpy(&pDev->Cfg, pCfgData, sizeof(SPICfg_t));
	if (slave)
	{
		// The slave is driven by its interrupts
		pDev->Cfg.bIntEn = true;
	}

	dev->pSpiDev = pDev;
	dev->bCsDrive = slave == false && pCfgData->ChipSel != SPICSEL_MAN && pCfgData->NbIOPins > SPI_CS_IOPIN_IDX;
	dev->bCsLow = false;
	dev->bRun = false;
	dev->bAsync = false;
	dev->bBad = false;
	if (dev->bCsDrive)
	{
		pDev->Cfg.ChipSel = SPICSEL_DRIVER;
	}
	pDev->CurDevCs = 0;
	pDev->FirstRdData = 0;
	memset(pDev->pRxBuff, 0, sizeof(pDev->pRxBuff));
	memset(pDev->RxBuffLen, 0, sizeof(pDev->RxBuffLen));
	memset(pDev->pTxData, 0, sizeof(pDev->pTxData));
	memset(pDev->TxDataLen, 0, sizeof(pDev->TxDataLen));

	pDev->DevIntrf.pDevData = (void*)dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Reset = Stm32SpiReset;
	pDev->DevIntrf.Disable = Stm32SpiDisable;
	pDev->DevIntrf.Enable = Stm32SpiEnable;
	pDev->DevIntrf.PowerOff = Stm32SpiPowerOff;
	pDev->DevIntrf.GetHandle = Stm32SpiGetHandle;
	pDev->DevIntrf.GetRate = Stm32SpiGetRate;
	pDev->DevIntrf.SetRate = Stm32SpiSetRate;
	pDev->DevIntrf.StartRx = Stm32SpiStart;
	pDev->DevIntrf.RxData = Stm32SpiRxData;
	pDev->DevIntrf.StopRx = Stm32SpiStop;
	pDev->DevIntrf.StartTx = Stm32SpiStart;
	pDev->DevIntrf.TxData = Stm32SpiTxData;
	pDev->DevIntrf.StopTx = Stm32SpiStop;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bIntEn = pDev->Cfg.bIntEn;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	atomic_store(&pDev->DevIntrf.bTxReady, true);

	Stm32SpiClkEnable(dev, true);
	Stm32SpiRccReset(dev);

#ifdef STM32_SPI_DMA
	if (pCfgData->bDmaEn)
	{
		if (Stm32SpiDmaTake(dev, pDev->Cfg.bIntEn, pCfgData->IntPrio) == false)
		{
			return false;
		}
		pDev->DevIntrf.bDma = true;
	}
	else
	{
		Stm32SpiDmaGive(dev);
	}
#else
	if (pCfgData->bDmaEn)
	{
		return false;
	}
#endif
	dev->bDma = pDev->DevIntrf.bDma;

	// Chip selects idle high before the pins are configured
	if (slave == false)
	{
		for (int i = SPI_CS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
		{
			IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
		}
	}

	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);
	for (int i = 0; i < pDev->Cfg.NbIOPins; i++)
	{
		IOPinSetSpeed(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

#ifdef STM32_SPI_LAYOUT_CFG
	// Master: the internal slave select is held inactive (SSM and SSI), so
	// the NSS pin is not used and no mode fault occurs. Slave: the NSS pin is
	// the frame select. AFCNTR keeps the pins driven by the peripheral when it
	// is disabled.
	uint32_t cfg2 = SPI_CFG2_AFCNTR;

	if (slave == false)
	{
		cfg2 |= SPI_CFG2_MASTER | SPI_CFG2_SSM;
	}
	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
		cfg2 |= SPI_CFG2_CPOL;
	}
	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cfg2 |= SPI_CFG2_CPHA;
	}
	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cfg2 |= SPI_CFG2_LSBFRST;
	}

	reg->CR1 = slave ? 0U : SPI_CR1_SSI;
	reg->CR2 = 0;
	reg->IER = 0;
	reg->AUTOCR = 0;
	reg->CFG1 = (pCfgData->DataSize - 1) << SPI_CFG1_DSIZE_Pos;
	reg->CFG2 = cfg2;
	reg->UDRDR = dev->Dummy;
	reg->IFCR = STM32_SPI_IFCR_ALL;

	Stm32SpiSetRate(&pDev->DevIntrf, pCfgData->Rate);
#else
	// Master: the internal slave select is held inactive (SSM and SSI), so
	// the NSS pin is not used and no mode fault occurs. Slave: the NSS pin is
	// the frame select. Data of 8 bits or less: RXNE at each byte (FRXTH).
	uint32_t cr1 = 0;
	uint32_t cr2 = ((pCfgData->DataSize - 1) << SPI_CR2_DS_Pos) | (pCfgData->DataSize <= 8 ? SPI_CR2_FRXTH : 0U);

	if (slave == false)
	{
		cr1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
	}
	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
		cr1 |= SPI_CR1_CPOL;
	}
	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cr1 |= SPI_CR1_CPHA;
	}
	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cr1 |= SPI_CR1_LSBFIRST;
	}

	reg->CR1 = 0;
	reg->CR2 = cr2;
	reg->CR1 = cr1;
	dev->Cr1 = cr1;
	dev->Cr2 = cr2;

	Stm32SpiSetRate(&pDev->DevIntrf, pCfgData->Rate);
	if (slave == false)
	{
		// SPE set: the peripheral drives SCK at its idle level between
		// transfers
		reg->CR1 |= SPI_CR1_SPE;
	}
#endif

	if (pDev->Cfg.bIntEn)
	{
		NVIC_ClearPendingIRQ(dev->pInfo->Irq);
		NVIC_SetPriority(dev->pInfo->Irq, pCfgData->IntPrio);
		NVIC_EnableIRQ(dev->pInfo->Irq);
	}

	if (slave)
	{
		if (pCfgData->NbIOPins > SPI_CS_IOPIN_IDX)
		{
			const IOPinCfg_t &cs = pCfgData->pIOPinMap[SPI_CS_IOPIN_IDX];

			if (cs.PortNo >= 0 && cs.PinNo >= 0 &&
				IOPinEnableInterrupt(cs.PinNo, pCfgData->IntPrio, cs.PortNo, cs.PinNo,
									 IOPINSENSE_TOGGLE, Stm32SpiNssEvt, dev))
			{
				dev->NssLine = cs.PinNo;
				dev->bNssInt = true;
			}
		}
		Stm32SpiSlaveArm(dev);
	}

	return true;
}

// Interrupt vectors of the instances
extern "C" void SPI1_IRQHandler(void)
{
	Stm32SpiService(&s_Stm32SpiDev[0]);
}

#ifdef SPI2
extern "C" void SPI2_IRQHandler(void)
{
	Stm32SpiService(&s_Stm32SpiDev[1]);
}
#endif

extern "C" void SPI3_IRQHandler(void)
{
	Stm32SpiService(&s_Stm32SpiDev[2]);
}
