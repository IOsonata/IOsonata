/**-------------------------------------------------------------------------
@file	spi_lpc546xx.cpp

@brief	LPC546xx Flexcomm SPI implementation, master and slave

		DevNo is the Flexcomm number, 0 to 9. The function clock is the
		12 MHz FRO, the master SCK is up to 12 MHz. 4 to 8 bit frames, one
		byte per frame, on the standard 4 wire bus (SPIPHY_NORMAL). Other
		PHY fail SPIInit. DevNo 10 (LPC546XX_SPIFI_DEVNO) is the SPIFI,
		spifi_lpc546xx.cpp.

		Master. Chip selects are GPIO pins, from index SPI_CS_IOPIN_IDX of
		the pin map. The DevAddr of StartRx and StartTx is the zero based
		chip select, driven low by StartTx and StartRx and high by StopTx
		and StopRx. The hardware SSEL outputs are not used. Receive clocks
		out the DummyByte of the configuration, data received during a
		transmit is discarded.

		Polling (bIntEn and bDmaEn false): TxData and RxData move the frames
		with at most LPC546XX_SPI_INFLIGHT in the FIFOs.

		Interrupt and DMA: each TxData and RxData call is one transfer. With
		bIntEn a receive, and a transmit of up to LPC546XX_SPI_TXBUF_SIZE
		bytes, return -1 at once and the end is reported with
		DEVINTRF_EVT_COMPLETED and the byte count after the chip select is
		released (DeviceIntrfTxComplete or DeviceIntrfStopRx); the transmit
		data is copied first. A longer transmit, and a transmit of a command
		phase before a read (the interface bNoStop set), wait for the end and
		return the count, as the STM32 driver does. The interrupt moves the
		frames on the RX FIFO level, the TX FIFO is filled as frames come
		back, so no more than a FIFO of frames is in flight. With bDmaEn two
		DMA channels move the frames, the Flexcomm TX request writes FIFOWR
		and the RX request reads FIFORD, in blocks of up to
		LPC546XX_DMA_XFER_MAX frames. A direction with no application data
		runs on the DummyByte or into a sink byte. The end of a block is the
		end of its receive. Without bIntEn the call waits for the end of the
		transfer. Waits are bounded from the last progress. A wait with
		bIntEn moves the transfer itself when the interrupt makes no
		progress, as when the caller runs with interrupts masked (an event
		handler) or at a priority as high. The chip select is released once
		the master is idle.

		Slave. Interrupt driven, bIntEn is forced, with or without DMA. The
		pin map is SCK, MISO, MOSI and the SSEL pin of the Flexcomm at index
		SPI_CS_IOPIN_IDX, active low. The frame sequence follows the other
		IOsonata SPI slaves: DEVINTRF_EVT_STATECHG before each frame lets
		the handler give the buffers (SetSlaveRxBuffer and SetSlaveTxData,
		index 0), the frame, then DEVINTRF_EVT_COMPLETED with the number of
		bytes of the frame, or -1 after an overrun or a frame longer than the
		receive buffer. After the transmit data the DummyByte is sent. The
		frame ends when SSEL is deasserted (SSD). The SPI is then disabled
		and its FIFOs emptied, which drops the transmit data loaded but not
		sent, and the next frame is prepared. With DMA the receive channel
		takes the frame into the buffer and the transmit channel loads the
		data, the interrupt takes over past them. SSEL asserted and
		deasserted with no byte is not reported. A frame that begins before
		the previous one is taken (the SSD interrupt late) is reported -1:
		bytes the CPU reads are split by the SOT flag of FIFORD, a frame
		whose SSEL is already asserted when it is prepared is marked bad.
		Bytes the receive channel has already moved are not split, the two
		frames are then reported as one. The interrupt handlers run with interrupts
		masked: the priorities order the interrupts pending, a handler is
		not preempted.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "LPC546xx.h"

#include "istddef.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"
#include "coredev/spi.h"
#include "coredev/shared_intrf.h"
#include "flexcomm_lpc546xx.h"
#include "dma_lpc546xx.h"
#include "spifi_lpc546xx.h"

#define LPC546XX_SPI_WAIT_MAX		1000000UL	// Status reads before a wait gives up
#define LPC546XX_SPI_DIV_MAX		0x10000U	// DIVVAL + 1
#define LPC546XX_SPI_DATASIZE_MIN	4U
#define LPC546XX_SPI_DATASIZE_MAX	8U

// Frames in flight in the FIFOs in polling, lower than their depth so the RX
// FIFO cannot overflow
#define LPC546XX_SPI_INFLIGHT		4

// Most bytes of an interrupt transmit copied to return at once
#define LPC546XX_SPI_TXBUF_SIZE		64

// FIFO depth when FIFOCFG SIZE reads 0
#define LPC546XX_SPI_FIFO_DEFAULT	8

// Passes of a wait without progress from the interrupt before the waiting
// code moves the transfer itself: the interrupt cannot run when the caller
// has interrupts masked or a priority as high
#define LPC546XX_SPI_STALL			256U

// STAT reads while the master finishes its last frame
#define LPC546XX_SPI_IDLE_READS		1000

// FIFOWR control bits of each frame, hardware SSEL outputs deasserted
#define LPC546XX_SPI_TXSSEL_OFF		(SPI_FIFOWR_TXSSEL0_N_MASK | SPI_FIFOWR_TXSSEL1_N_MASK | \
									 SPI_FIFOWR_TXSSEL2_N_MASK | SPI_FIFOWR_TXSSEL3_N_MASK)

#define LPC546XX_SPI_FIFO_INTS		(SPI_FIFOINTENSET_TXERR_MASK | SPI_FIFOINTENSET_RXERR_MASK | \
									 SPI_FIFOINTENSET_TXLVL_MASK | SPI_FIFOINTENSET_RXLVL_MASK)

#pragma pack(push, 4)

typedef struct __Lpc546xx_SPI_Dev {
	SPI_Type *pReg;					//!< Flexcomm SPI registers
	int DevNo;						//!< Flexcomm number
	uint32_t FClk;					//!< Function clock frequency
	uint32_t TxCtrl;				//!< FIFOWR control bits of each frame
	SPIDev_t *pSpiDev;				//!< Generic SPI device data
	int FifoSize;					//!< Entries of each FIFO
	bool bSlave;					//!< Slave mode
	bool bDmaClaimed;				//!< The two DMA channels are taken
	// Master transfer of the interrupt and DMA modes
	volatile bool bRun;				//!< Transfer running
	bool bAsync;					//!< The end is reported by the event
	bool bRx;						//!< Receive transfer, StopRx at its end
	const uint8_t *pTx;				//!< Data sent, NULL for the DummyByte
	uint8_t *pRx;					//!< Data received, NULL to discard it
	int Len;						//!< Frames of the transfer
	int TxCnt;						//!< Frames written to the TX FIFO
	int RxCnt;						//!< Frames received, the count reported
	int DmaLen;						//!< Frames of the DMA block running, 0 none
	// Slave frame
	int SRx;						//!< Bytes received in the frame
	int SRxMax;						//!< Receive buffer length
	int STx;						//!< Bytes of the transmit data loaded
	int STxMax;						//!< Transmit data length
	int SRxDmaLen;					//!< Bytes of the receive DMA, 0 none running
	int STxDmaLen;					//!< Bytes of the transmit DMA, 0 none running
	uint8_t *pSRx;					//!< Receive buffer of the frame
	const uint8_t *pSTx;			//!< Transmit data of the frame
	bool bSBad;						//!< Overrun or frame longer than the buffer
	bool bSNext;					//!< The next frame began before this one was taken
	uint8_t Dummy;					//!< DMA source of the DummyByte
	uint8_t Sink;					//!< DMA destination of the bytes discarded
	uint8_t TxBuf[LPC546XX_SPI_TXBUF_SIZE];	//!< Copy of an interrupt transmit
} Lpc546xxSPIDev_t;

#pragma pack(pop)

// Registers of each Flexcomm, the instance data is set by SPIInit
static SPI_Type * const s_Lpc546xxSPIReg[] = {
	SPI0, SPI1, SPI2, SPI3, SPI4, SPI5, SPI6, SPI7, SPI8, SPI9,
};

static const int s_NbSPIDev = sizeof(s_Lpc546xxSPIReg) / sizeof(SPI_Type *);

static Lpc546xxSPIDev_t s_Lpc546xxSPIDev[sizeof(s_Lpc546xxSPIReg) / sizeof(SPI_Type *)];

static int Lpc546xxSPIXfer(Lpc546xxSPIDev_t * const pDev, const uint8_t *pTx, uint8_t *pRx, int Len);
static void Lpc546xxSPIMstService(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPIMstDmaBlock(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPIMstDmaDone(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPIMstEnd(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPIMstAbort(Lpc546xxSPIDev_t * const pDev);
static int Lpc546xxSPIMstRun(Lpc546xxSPIDev_t * const pDev, const uint8_t *pTx, uint8_t *pRx, int Len, bool bAsync);
static void Lpc546xxSPISlvWrite(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPISlvRead(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPISlvArm(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPISlvEnd(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPISlvDma(Lpc546xxSPIDev_t * const pDev);
static void Lpc546xxSPIIrqHandler(int DevNo, DevIntrf_t * const pDev);
static void Lpc546xxSPIDmaIrq(int Ch, void *pCtx);
static void Lpc546xxSPIDmaFree(Lpc546xxSPIDev_t * const pDev);
static bool Lpc546xxSPIStart(DevIntrf_t * const pDev, uint32_t DevCs);
static void Lpc546xxSPIStop(DevIntrf_t * const pDev);

static uint32_t Lpc546xxSPIGetRate(DevIntrf_t * const pDev);
static uint32_t Lpc546xxSPISetRate(DevIntrf_t * const pDev, uint32_t Rate);
static int Lpc546xxSPIRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen);
static int Lpc546xxSPITxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen);
static void Lpc546xxSPIDisable(DevIntrf_t * const pDev);
static void Lpc546xxSPIEnable(DevIntrf_t * const pDev);
static void Lpc546xxSPIPowerOff(DevIntrf_t * const pDev);
static void Lpc546xxSPIReset(DevIntrf_t * const pDev);
static void *Lpc546xxSPIGetHandle(DevIntrf_t * const pDev);

// Polled transfer of Len frames. pTx NULL sends the dummy byte, pRx NULL
// ignores the received data. Returns the number of frames transferred.
static int Lpc546xxSPIXfer(Lpc546xxSPIDev_t * const pDev, const uint8_t *pTx, uint8_t *pRx, int Len)
{
	SPI_Type *reg = pDev->pReg;
	uint32_t ctrl = pDev->TxCtrl | (pRx == NULL ? SPI_FIFOWR_RXIGNORE_MASK : 0U);
	uint32_t dummy = pDev->pSpiDev->Cfg.DummyByte;
	int txcnt = 0;
	int rxcnt = 0;
	uint32_t wait = LPC546XX_SPI_WAIT_MAX;

	reg->FIFOCFG |= SPI_FIFOCFG_EMPTYRX_MASK;
	reg->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

	while ((pRx != NULL ? rxcnt : txcnt) < Len && wait > 0)
	{
		uint32_t stat = reg->FIFOSTAT;

		if (txcnt < Len && (stat & SPI_FIFOSTAT_TXNOTFULL_MASK) &&
			(pRx == NULL || txcnt - rxcnt < LPC546XX_SPI_INFLIGHT))
		{
			reg->FIFOWR = (pTx != NULL ? pTx[txcnt] : dummy) | ctrl;
			txcnt++;
			wait = LPC546XX_SPI_WAIT_MAX;
		}
		else if (pRx != NULL && (stat & SPI_FIFOSTAT_RXNOTEMPTY_MASK))
		{
			pRx[rxcnt++] = (uint8_t)reg->FIFORD;
			wait = LPC546XX_SPI_WAIT_MAX;
		}
		else
		{
			wait--;
		}
	}

	// TX only, the last frames must be on the bus before the chip select is
	// released. The wait is bounded from the last change of the TX level.
	if (pRx == NULL)
	{
		uint32_t lvl = 0xFFFFFFFFU;

		for (wait = LPC546XX_SPI_WAIT_MAX; wait > 0; wait--)
		{
			uint32_t stat = reg->FIFOSTAT;

			if ((stat & SPI_FIFOSTAT_TXEMPTY_MASK) && (reg->STAT & SPI_STAT_MSTIDLE_MASK))
			{
				break;
			}
			if ((stat & SPI_FIFOSTAT_TXLVL_MASK) != lvl)
			{
				lvl = stat & SPI_FIFOSTAT_TXLVL_MASK;
				wait = LPC546XX_SPI_WAIT_MAX;
			}
		}

		return wait > 0 ? txcnt : 0;
	}

	return rxcnt;
}

// Interrupt master: take the frames received, then write as many frames as
// came back, a FIFO of frames at most in flight. Every frame is received so
// that the end of the last one ends the transfer.
static void Lpc546xxSPIMstService(Lpc546xxSPIDev_t * const pDev)
{
	SPI_Type *reg = pDev->pReg;
	uint32_t dummy = pDev->pSpiDev->Cfg.DummyByte;

	if (pDev->bRun == false || pDev->DmaLen > 0)
	{
		return;
	}

	while (pDev->RxCnt < pDev->Len && (reg->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK))
	{
		uint8_t d = (uint8_t)reg->FIFORD;

		if (pDev->pRx != NULL)
		{
			pDev->pRx[pDev->RxCnt] = d;
		}
		pDev->RxCnt++;
	}

	while (pDev->TxCnt < pDev->Len && pDev->TxCnt - pDev->RxCnt < pDev->FifoSize &&
		   (reg->FIFOSTAT & SPI_FIFOSTAT_TXNOTFULL_MASK))
	{
		reg->FIFOWR = (pDev->pTx != NULL ? pDev->pTx[pDev->TxCnt] : dummy) | pDev->TxCtrl;
		pDev->TxCnt++;
	}

	if (pDev->RxCnt >= pDev->Len)
	{
		Lpc546xxSPIMstEnd(pDev);
	}
}

// Next DMA block of the master transfer, from RxCnt
static void Lpc546xxSPIMstDmaBlock(Lpc546xxSPIDev_t * const pDev)
{
	SPI_Type *reg = pDev->pReg;
	int rxch = LPC546XX_DMA_CH_FC_RX(pDev->DevNo);
	int txch = LPC546XX_DMA_CH_FC_TX(pDev->DevNo);
	int n = pDev->Len - pDev->RxCnt;

	if (n > (int)LPC546XX_DMA_XFER_MAX)
	{
		n = LPC546XX_DMA_XFER_MAX;
	}

	// The control bits of the frames the channel writes. A halfword write
	// to the control half of FIFOWR pushes nothing, the byte writes of the
	// channel push the data with them.
	*(volatile uint16_t *)((uintptr_t)&reg->FIFOWR + 2U) = (uint16_t)(pDev->TxCtrl >> 16);

	Lpc546xxDmaChMemInc(rxch, pDev->pRx != NULL);
	Lpc546xxDmaChMemInc(txch, pDev->pTx != NULL);

	if (Lpc546xxDmaChStart(rxch, pDev->pRx != NULL ? &pDev->pRx[pDev->RxCnt] : &pDev->Sink, (uint32_t)n) == false ||
		Lpc546xxDmaChStart(txch, pDev->pTx != NULL ? &pDev->pTx[pDev->RxCnt] : &pDev->Dummy, (uint32_t)n) == false)
	{
		Lpc546xxDmaChStop(rxch);
		Lpc546xxDmaChStop(txch);
		Lpc546xxSPIMstEnd(pDev);

		return;
	}

	pDev->DmaLen = n;
	reg->FIFOCFG |= SPI_FIFOCFG_DMATX_MASK | SPI_FIFOCFG_DMARX_MASK;
}

// End of a DMA block: the receive channel done, or an error on either. An
// error ends the transfer with the frames received.
static void Lpc546xxSPIMstDmaDone(Lpc546xxSPIDev_t * const pDev)
{
	int rxch = LPC546XX_DMA_CH_FC_RX(pDev->DevNo);
	int txch = LPC546XX_DMA_CH_FC_TX(pDev->DevNo);

	if (pDev->bRun == false || pDev->DmaLen == 0)
	{
		return;
	}

	uint32_t rf = Lpc546xxDmaChFlags(rxch);
	uint32_t tf = Lpc546xxDmaChFlags(txch);

	if ((rf | tf) & LPC546XX_DMA_FLAG_ERR)
	{
		uint32_t remain = Lpc546xxDmaChStopRemain(rxch);

		Lpc546xxDmaChStop(txch);
		pDev->RxCnt += pDev->DmaLen - (int)remain;
		pDev->DmaLen = 0;
		pDev->pReg->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
		Lpc546xxSPIMstEnd(pDev);

		return;
	}

	if ((rf & LPC546XX_DMA_FLAG_DONE) == 0)
	{
		return;
	}

	Lpc546xxDmaChClear(rxch);
	Lpc546xxDmaChClear(txch);
	pDev->RxCnt += pDev->DmaLen;
	pDev->DmaLen = 0;

	if (pDev->RxCnt < pDev->Len)
	{
		Lpc546xxSPIMstDmaBlock(pDev);
	}
	else
	{
		Lpc546xxSPIMstEnd(pDev);
	}
}

// The transfer is over. An asynchronous one releases the chip select and
// reports the count.
static void Lpc546xxSPIMstEnd(Lpc546xxSPIDev_t * const pDev)
{
	SPI_Type *reg = pDev->pReg;
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
	bool rx = pDev->bRx;

	reg->FIFOINTENCLR = LPC546XX_SPI_FIFO_INTS;
	reg->FIFOCFG &= ~(SPI_FIFOCFG_DMATX_MASK | SPI_FIFOCFG_DMARX_MASK);

	// The last frame is received before its last clock edge with CPHA 0, the
	// chip select is released once the master is idle
	for (int i = 0; i < LPC546XX_SPI_IDLE_READS && (reg->STAT & SPI_STAT_MSTIDLE_MASK) == 0; i++)
	{
	}

	pDev->bRun = false;

	if (pDev->bAsync == false)
	{
		return;
	}

	pDev->bAsync = false;
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
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, rx ? pDev->pRx : NULL, pDev->RxCnt);
	}
}

// A transfer that stopped moving, the frames received are counted.
// Called with interrupts disabled.
static void Lpc546xxSPIMstAbort(Lpc546xxSPIDev_t * const pDev)
{
	if (pDev->bRun == false)
	{
		return;
	}

	SPI_Type *reg = pDev->pReg;

	if (pDev->DmaLen > 0)
	{
		uint32_t remain = Lpc546xxDmaChStopRemain(LPC546XX_DMA_CH_FC_RX(pDev->DevNo));

		Lpc546xxDmaChStop(LPC546XX_DMA_CH_FC_TX(pDev->DevNo));
		pDev->RxCnt += pDev->DmaLen - (int)remain;
		pDev->DmaLen = 0;
	}
	else
	{
		// The frames received and not yet taken
		while (pDev->RxCnt < pDev->Len && (reg->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK))
		{
			uint8_t d = (uint8_t)reg->FIFORD;

			if (pDev->pRx != NULL)
			{
				pDev->pRx[pDev->RxCnt] = d;
			}
			pDev->RxCnt++;
		}
	}
	reg->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
	Lpc546xxSPIMstEnd(pDev);
}

// Interrupt or DMA master transfer of Len frames. bAsync returns -1, the end
// is reported by the event. Otherwise the call waits for the end and
// returns the frames received.
static int Lpc546xxSPIMstRun(Lpc546xxSPIDev_t * const pDev, const uint8_t *pTx, uint8_t *pRx, int Len, bool bAsync)
{
	SPI_Type *reg = pDev->pReg;
	DevIntrf_t *intrf = &pDev->pSpiDev->DevIntrf;
	uint32_t state = DisableInterrupt();

	if (pDev->bRun)
	{
		EnableInterrupt(state);

		return 0;
	}

	pDev->pTx = pTx;
	pDev->pRx = pRx;
	pDev->Len = Len;
	pDev->TxCnt = 0;
	pDev->RxCnt = 0;
	pDev->DmaLen = 0;
	pDev->bRx = pTx == NULL;
	pDev->bAsync = bAsync;
	pDev->bRun = true;
	pDev->Dummy = pDev->pSpiDev->Cfg.DummyByte;

	if (pDev->bRx == false)
	{
		atomic_store(&intrf->bTxReady, false);
	}

	reg->FIFOCFG |= SPI_FIFOCFG_EMPTYRX_MASK;
	reg->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

	if (intrf->bDma)
	{
		Lpc546xxSPIMstDmaBlock(pDev);
	}
	else
	{
		Lpc546xxSPIMstService(pDev);
		if (pDev->bRun && intrf->bIntEn)
		{
			reg->FIFOINTENSET = SPI_FIFOINTENSET_RXLVL_MASK;
		}
	}

	EnableInterrupt(state);

	if (bAsync)
	{
		return -1;
	}

	uint32_t left = LPC546XX_SPI_WAIT_MAX;
	uint32_t prior = 0xFFFFFFFFU;
	uint32_t priorlvl = 0;

	while (pDev->bRun)
	{
		// Without the interrupt, or when it makes no progress
		if (intrf->bIntEn == false || left < LPC546XX_SPI_WAIT_MAX - LPC546XX_SPI_STALL)
		{
			state = DisableInterrupt();
			if (intrf->bDma)
			{
				Lpc546xxSPIMstDmaDone(pDev);
			}
			else
			{
				Lpc546xxSPIMstService(pDev);
			}
			EnableInterrupt(state);
		}

		// The wait is bounded from the last progress: frames received, the
		// count left of the DMA block or the FIFO levels
		uint32_t progress = (uint32_t)__atomic_load_n(&pDev->RxCnt, __ATOMIC_RELAXED) * 2048U;
		uint32_t lvl = reg->FIFOSTAT & (SPI_FIFOSTAT_TXLVL_MASK | SPI_FIFOSTAT_RXLVL_MASK);

		if (pDev->DmaLen > 0)
		{
			progress -= Lpc546xxDmaChRemain(LPC546XX_DMA_CH_FC_RX(pDev->DevNo));
		}
		if (progress != prior || lvl != priorlvl)
		{
			prior = progress;
			priorlvl = lvl;
			left = LPC546XX_SPI_WAIT_MAX;
		}
		else if (--left == 0)
		{
			state = DisableInterrupt();
			Lpc546xxSPIMstAbort(pDev);
			EnableInterrupt(state);
		}
	}

	if (pDev->bRx == false)
	{
		atomic_store(&intrf->bTxReady, true);
	}

	return pDev->RxCnt;
}

// Fill the slave TX FIFO: the transmit data, then the DummyByte
static void Lpc546xxSPISlvWrite(Lpc546xxSPIDev_t * const pDev)
{
	SPI_Type *reg = pDev->pReg;

	while (reg->FIFOSTAT & SPI_FIFOSTAT_TXNOTFULL_MASK)
	{
		uint32_t d = pDev->Dummy;

		if (pDev->STx < pDev->STxMax)
		{
			d = pDev->pSTx[pDev->STx++];
		}
		reg->FIFOWR = d | pDev->TxCtrl;
	}
}

// Take the slave RX FIFO, bytes past the buffer mark the frame bad. SOT
// marks the first byte after SSEL was asserted: one after the first byte of
// the frame belongs to the next frame, which began before this one was
// taken, it and the bytes after it are dropped.
static void Lpc546xxSPISlvRead(Lpc546xxSPIDev_t * const pDev)
{
	SPI_Type *reg = pDev->pReg;

	while (reg->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK)
	{
		uint32_t d = reg->FIFORD;

		if ((d & SPI_FIFORD_SOT_MASK) && pDev->SRx > 0)
		{
			pDev->bSNext = true;
		}
		if (pDev->bSNext)
		{
			continue;
		}
		if (pDev->SRx < pDev->SRxMax)
		{
			pDev->pSRx[pDev->SRx] = (uint8_t)d;
		}
		else
		{
			pDev->bSBad = true;
		}
		pDev->SRx++;
	}
}

// Prepare the next frame. The SPI is disabled and its FIFOs emptied, the
// handler gives the buffers, the TX FIFO is loaded or the channels started,
// then the SPI waits for SSEL. Called with interrupts disabled.
static void Lpc546xxSPISlvArm(Lpc546xxSPIDev_t * const pDev)
{
	SPI_Type *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	int rxch = LPC546XX_DMA_CH_FC_RX(pDev->DevNo);
	int txch = LPC546XX_DMA_CH_FC_TX(pDev->DevNo);

	reg->CFG &= ~SPI_CFG_ENABLE_MASK;
	reg->FIFOINTENCLR = LPC546XX_SPI_FIFO_INTS;
	if (pDev->bDmaClaimed)
	{
		Lpc546xxDmaChStop(rxch);
		Lpc546xxDmaChStop(txch);
	}
	pDev->SRxDmaLen = 0;
	pDev->STxDmaLen = 0;
	reg->FIFOCFG = (reg->FIFOCFG & ~(SPI_FIFOCFG_DMATX_MASK | SPI_FIFOCFG_DMARX_MASK)) |
				   SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
	reg->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;
	reg->STAT = SPI_STAT_SSA_MASK | SPI_STAT_SSD_MASK;

	spi->pRxBuff[0] = NULL;
	spi->RxBuffLen[0] = 0;
	spi->pTxData[0] = NULL;
	spi->TxDataLen[0] = 0;
	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_STATECHG, NULL, 0);
	}

	pDev->bSBad = false;
	pDev->bSNext = false;
	pDev->SRx = 0;
	pDev->STx = 0;
	pDev->pSRx = spi->pRxBuff[0];
	pDev->pSTx = spi->pTxData[0];
	pDev->SRxMax = pDev->pSRx != NULL && spi->RxBuffLen[0] > 0 ? spi->RxBuffLen[0] : 0;
	pDev->STxMax = pDev->pSTx != NULL && spi->TxDataLen[0] > 0 ? spi->TxDataLen[0] : 0;
	pDev->Dummy = spi->Cfg.DummyByte;

	uint32_t fint = SPI_FIFOINTENSET_RXERR_MASK | SPI_FIFOINTENSET_RXLVL_MASK | SPI_FIFOINTENSET_TXLVL_MASK;

	if (intrf->bDma && pDev->bDmaClaimed)
	{
		if (pDev->SRxMax > 0)
		{
			int n = pDev->SRxMax > (int)LPC546XX_DMA_XFER_MAX ? (int)LPC546XX_DMA_XFER_MAX : pDev->SRxMax;

			Lpc546xxDmaChMemInc(rxch, true);
			if (Lpc546xxDmaChStart(rxch, spi->pRxBuff[0], (uint32_t)n))
			{
				pDev->SRxDmaLen = n;
				reg->FIFOCFG |= SPI_FIFOCFG_DMARX_MASK;
				fint &= ~SPI_FIFOINTENSET_RXLVL_MASK;
			}
		}
		if (pDev->STxMax > 0)
		{
			int n = pDev->STxMax > (int)LPC546XX_DMA_XFER_MAX ? (int)LPC546XX_DMA_XFER_MAX : pDev->STxMax;

			*(volatile uint16_t *)((uintptr_t)&reg->FIFOWR + 2U) = (uint16_t)(pDev->TxCtrl >> 16);
			Lpc546xxDmaChMemInc(txch, true);
			if (Lpc546xxDmaChStart(txch, spi->pTxData[0], (uint32_t)n))
			{
				pDev->STxDmaLen = n;
				reg->FIFOCFG |= SPI_FIFOCFG_DMATX_MASK;
				fint &= ~SPI_FIFOINTENSET_TXLVL_MASK;
			}
		}
	}

	// TX level: the DMA request while the FIFO is not full, the interrupt at
	// half the FIFO
	reg->FIFOTRIG = (reg->FIFOTRIG & ~SPI_FIFOTRIG_TXLVL_MASK) |
					SPI_FIFOTRIG_TXLVL(pDev->STxDmaLen > 0 ? pDev->FifoSize - 1 : pDev->FifoSize / 2);

	if (fint & SPI_FIFOINTENSET_TXLVL_MASK)
	{
		Lpc546xxSPISlvWrite(pDev);
	}
	reg->FIFOINTENSET = fint;
	reg->CFG |= SPI_CFG_ENABLE_MASK;

	// SSEL already asserted: the frame began before the slave was ready, it
	// is reported bad
	const IOPinCfg_t &cs = spi->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX];

	if (IOPinRead(cs.PortNo, cs.PinNo) == 0)
	{
		pDev->bSBad = true;
	}
}

// SSEL deasserted: the frame is over. Called with interrupts disabled.
static void Lpc546xxSPISlvEnd(Lpc546xxSPIDev_t * const pDev)
{
	SPI_Type *reg = pDev->pReg;
	SPIDev_t *spi = pDev->pSpiDev;
	DevIntrf_t *intrf = &spi->DevIntrf;
	int rxch = LPC546XX_DMA_CH_FC_RX(pDev->DevNo);

	// SSEL asserted then deasserted with no byte: nothing to report, the
	// frame stays prepared, as the other IOsonata slaves
	if (pDev->SRx == 0 && pDev->bSBad == false && (reg->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK) == 0 &&
		(pDev->SRxDmaLen == 0 || Lpc546xxDmaChRemain(rxch) == (uint32_t)pDev->SRxDmaLen))
	{
		return;
	}

	if (pDev->SRxDmaLen > 0)
	{
		// The bytes the channel has moved, the CPU takes those still in the
		// FIFO after them
		if (Lpc546xxDmaChFlags(rxch) & LPC546XX_DMA_FLAG_ERR)
		{
			pDev->bSBad = true;
		}
		pDev->SRx = pDev->SRxDmaLen - (int)Lpc546xxDmaChStopRemain(rxch);
		pDev->SRxDmaLen = 0;
		reg->FIFOCFG &= ~SPI_FIFOCFG_DMARX_MASK;
	}

	Lpc546xxSPISlvRead(pDev);

	if (reg->FIFOSTAT & SPI_FIFOSTAT_RXERR_MASK)
	{
		pDev->bSBad = true;
	}

	int count = pDev->bSBad ? -1 : pDev->SRx;
	bool next = pDev->bSNext;

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, pDev->pSRx, count);
	}

	Lpc546xxSPISlvArm(pDev);

	// The next frame began before this one was taken and is already over:
	// reported bad, then the frame after it is prepared. Still running, Arm
	// has marked it bad.
	if (next && pDev->bSBad == false)
	{
		if (intrf->EvtCB != NULL)
		{
			intrf->EvtCB(intrf, DEVINTRF_EVT_COMPLETED, pDev->pSRx, -1);
		}
		Lpc546xxSPISlvArm(pDev);
	}
}

// A slave channel done or in error: the interrupt takes over the direction.
// Called with interrupts disabled.
static void Lpc546xxSPISlvDma(Lpc546xxSPIDev_t * const pDev)
{
	SPI_Type *reg = pDev->pReg;
	int rxch = LPC546XX_DMA_CH_FC_RX(pDev->DevNo);
	int txch = LPC546XX_DMA_CH_FC_TX(pDev->DevNo);

	if (pDev->SRxDmaLen > 0)
	{
		uint32_t f = Lpc546xxDmaChFlags(rxch);

		if (f & (LPC546XX_DMA_FLAG_DONE | LPC546XX_DMA_FLAG_ERR))
		{
			if (f & LPC546XX_DMA_FLAG_ERR)
			{
				pDev->bSBad = true;
			}
			pDev->SRx = pDev->SRxDmaLen - (int)Lpc546xxDmaChStopRemain(rxch);
			pDev->SRxDmaLen = 0;
			reg->FIFOCFG &= ~SPI_FIFOCFG_DMARX_MASK;
			reg->FIFOINTENSET = SPI_FIFOINTENSET_RXLVL_MASK;
		}
	}

	if (pDev->STxDmaLen > 0)
	{
		uint32_t f = Lpc546xxDmaChFlags(txch);

		if (f & (LPC546XX_DMA_FLAG_DONE | LPC546XX_DMA_FLAG_ERR))
		{
			pDev->STx = pDev->STxDmaLen - (int)Lpc546xxDmaChStopRemain(txch);
			pDev->STxDmaLen = 0;
			reg->FIFOCFG &= ~SPI_FIFOCFG_DMATX_MASK;
			reg->FIFOTRIG = (reg->FIFOTRIG & ~SPI_FIFOTRIG_TXLVL_MASK) | SPI_FIFOTRIG_TXLVL(pDev->FifoSize / 2);
			Lpc546xxSPISlvWrite(pDev);
			reg->FIFOINTENSET = SPI_FIFOINTENSET_TXLVL_MASK;
		}
	}
}

static void Lpc546xxSPIIrqHandler(int DevNo, DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	SPI_Type *reg = dev->pReg;
	uint32_t state = DisableInterrupt();

	(void)DevNo;

	if (dev->bSlave)
	{
		uint32_t fstat = reg->FIFOSTAT;

		if (dev->SRxDmaLen == 0)
		{
			Lpc546xxSPISlvRead(dev);
		}
		if (fstat & SPI_FIFOSTAT_RXERR_MASK)
		{
			// Overrun, the frame is reported bad at its end
			dev->bSBad = true;
			reg->FIFOSTAT = SPI_FIFOSTAT_RXERR_MASK;
		}
		if (fstat & SPI_FIFOSTAT_TXERR_MASK)
		{
			// Underrun, the master got a byte the slave did not load
			reg->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK;
		}
		if (dev->STxDmaLen == 0)
		{
			Lpc546xxSPISlvWrite(dev);
		}
		if (reg->STAT & SPI_STAT_SSD_MASK)
		{
			reg->STAT = SPI_STAT_SSD_MASK;
			Lpc546xxSPISlvEnd(dev);
		}
	}
	else if (dev->bRun && pDev->bDma == false)
	{
		Lpc546xxSPIMstService(dev);
	}
	else
	{
		reg->FIFOINTENCLR = LPC546XX_SPI_FIFO_INTS;
	}

	EnableInterrupt(state);
}

// DMA interrupt of a channel, the context is the instance
static void Lpc546xxSPIDmaIrq(int Ch, void *pCtx)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pCtx;
	uint32_t state = DisableInterrupt();

	(void)Ch;

	if (dev->bSlave)
	{
		Lpc546xxSPISlvDma(dev);
	}
	else
	{
		Lpc546xxSPIMstDmaDone(dev);
	}

	EnableInterrupt(state);
}

static void Lpc546xxSPIDmaFree(Lpc546xxSPIDev_t * const pDev)
{
	if (pDev->bDmaClaimed)
	{
		Lpc546xxDmaChFree(LPC546XX_DMA_CH_FC_RX(pDev->DevNo));
		Lpc546xxDmaChFree(LPC546XX_DMA_CH_FC_TX(pDev->DevNo));
		pDev->bDmaClaimed = false;
	}
}

static bool Lpc546xxSPIStart(DevIntrf_t * const pDev, uint32_t DevCs)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;

	if (dev->bSlave || dev->bRun)
	{
		return false;
	}

	if (spi->Cfg.ChipSel == SPICSEL_MAN)
	{
		return true;
	}

	if ((int)DevCs < 0 || (int)DevCs >= spi->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
	{
		return false;
	}

	spi->CurDevCs = (int)DevCs;
	IOPinClear(spi->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo, spi->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);

	return true;
}

static void Lpc546xxSPIStop(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	SPIDev_t *spi = dev->pSpiDev;

	if (dev->bSlave == false && spi->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(spi->Cfg.pIOPinMap[spi->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				 spi->Cfg.pIOPinMap[spi->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

static uint32_t Lpc546xxSPIGetRate(DevIntrf_t * const pDev)
{
	return ((Lpc546xxSPIDev_t *)pDev->pDevData)->pSpiDev->Cfg.Rate;
}

// Closest SCK rate, function clock / (DIVVAL + 1). The slave follows the
// master clock, the divider is not used.
static uint32_t Lpc546xxSPISetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;

	if (Rate == 0 || dev->FClk == 0)
	{
		return 0;
	}

	uint32_t div = (uint32_t)(((uint64_t)dev->FClk + Rate / 2U) / Rate);

	if (div < 1U)
	{
		div = 1U;
	}
	else if (div > LPC546XX_SPI_DIV_MAX)
	{
		div = LPC546XX_SPI_DIV_MAX;
	}

	dev->pReg->DIV = SPI_DIV_DIVVAL(div - 1U);
	dev->pSpiDev->Cfg.Rate = dev->FClk / div;

	return dev->pSpiDev->Cfg.Rate;
}

static int Lpc546xxSPIRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;

	if (dev->bSlave || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	if (pDev->bIntEn == false && pDev->bDma == false)
	{
		return Lpc546xxSPIXfer(dev, NULL, pBuff, BuffLen);
	}

	return Lpc546xxSPIMstRun(dev, NULL, pBuff, BuffLen, pDev->bIntEn && pDev->bNoStop == false);
}

static int Lpc546xxSPITxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;

	if (dev->bSlave || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	if (pDev->bIntEn == false && pDev->bDma == false)
	{
		return Lpc546xxSPIXfer(dev, pData, NULL, DataLen);
	}

	if (pDev->bIntEn && pDev->bNoStop == false && DataLen <= LPC546XX_SPI_TXBUF_SIZE && dev->bRun == false)
	{
		memcpy(dev->TxBuf, pData, DataLen);

		return Lpc546xxSPIMstRun(dev, dev->TxBuf, NULL, DataLen, true);
	}

	return Lpc546xxSPIMstRun(dev, pData, NULL, DataLen, false);
}

static void Lpc546xxSPIDisable(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	IRQn_Type irq = Lpc546xxFlexcommIrqNo(dev->DevNo);
	uint32_t state = DisableInterrupt();

	Lpc546xxSPIMstAbort(dev);

	// No interrupt while the clock is off
	NVIC_DisableIRQ(irq);
	NVIC_ClearPendingIRQ(irq);
	dev->pReg->FIFOINTENCLR = LPC546XX_SPI_FIFO_INTS;
	if (dev->bDmaClaimed)
	{
		Lpc546xxDmaChStop(LPC546XX_DMA_CH_FC_RX(dev->DevNo));
		Lpc546xxDmaChStop(LPC546XX_DMA_CH_FC_TX(dev->DevNo));
	}
	dev->SRxDmaLen = 0;
	dev->STxDmaLen = 0;
	dev->pReg->CFG &= ~SPI_CFG_ENABLE_MASK;
	Lpc546xxFlexcommClock(dev->DevNo, false);

	EnableInterrupt(state);
}

static void Lpc546xxSPIEnable(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	Lpc546xxFlexcommClock(dev->DevNo, true);
	IOPinCfg(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);
	if (dev->bSlave)
	{
		Lpc546xxSPISlvArm(dev);
	}
	else
	{
		dev->pReg->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
		dev->pReg->CFG |= SPI_CFG_ENABLE_MASK;
	}
	if (pDev->bIntEn)
	{
		NVIC_EnableIRQ(Lpc546xxFlexcommIrqNo(dev->DevNo));
	}

	EnableInterrupt(state);
}

// Everything given back: interrupt handler, DMA channels, pins. SPIInit
// is needed again.
static void Lpc546xxSPIPowerOff(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->INTENCLR = 0xFFFFFFFFU;
	Lpc546xxSPIDisable(pDev);
	SharedIntrfSetIrqHandler(dev->DevNo, NULL, NULL);
	Lpc546xxSPIDmaFree(dev);
	EnableInterrupt(state);

	IOPinDis(dev->pSpiDev->Cfg.pIOPinMap, dev->pSpiDev->Cfg.NbIOPins);
}

// Master: FIFOs emptied, errors cleared, the configuration is kept. Slave:
// the next frame is prepared again.
static void Lpc546xxSPIReset(DevIntrf_t * const pDev)
{
	Lpc546xxSPIDev_t *dev = (Lpc546xxSPIDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	if (dev->bSlave)
	{
		Lpc546xxSPISlvArm(dev);
	}
	else
	{
		Lpc546xxSPIMstAbort(dev);
		dev->pReg->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
		dev->pReg->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;
	}

	EnableInterrupt(state);
}

static void *Lpc546xxSPIGetHandle(DevIntrf_t * const pDev)
{
	return ((Lpc546xxSPIDev_t *)pDev->pDevData)->pSpiDev;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	// The SPIFI, the DevNo after the Flexcomms
	if (pCfgData != NULL && pCfgData->DevNo == LPC546XX_SPIFI_DEVNO)
	{
		return Lpc546xxSpifiInit(pDev, pCfgData);
	}

	if (pDev == NULL || pCfgData == NULL || pCfgData->DevNo < 0 || pCfgData->DevNo >= s_NbSPIDev ||
		pCfgData->pIOPinMap == NULL || pCfgData->Rate == 0)
	{
		return false;
	}

	bool slave = pCfgData->Mode == SPIMODE_SLAVE;

	// Master or slave on the standard 4 wire bus, 4 to 8 bit frames. The
	// slave needs its SSEL pin.
	if ((pCfgData->Mode != SPIMODE_MASTER && slave == false) || pCfgData->Phy != SPIPHY_NORMAL ||
		pCfgData->DataSize < LPC546XX_SPI_DATASIZE_MIN || pCfgData->DataSize > LPC546XX_SPI_DATASIZE_MAX ||
		(pCfgData->ChipSel != SPICSEL_AUTO && pCfgData->ChipSel != SPICSEL_MAN) ||
		pCfgData->NbIOPins < (slave || pCfgData->ChipSel == SPICSEL_AUTO ? SPI_CS_IOPIN_IDX + 1 : SPI_CS_IOPIN_IDX))
	{
		return false;
	}

	Lpc546xxSPIDev_t *dev = &s_Lpc546xxSPIDev[pCfgData->DevNo];

	dev->pReg = s_Lpc546xxSPIReg[pCfgData->DevNo];
	dev->DevNo = pCfgData->DevNo;

	IRQn_Type irq = Lpc546xxFlexcommIrqNo(dev->DevNo);
	uint32_t state = DisableInterrupt();

	// A transfer or a frame of the earlier configuration is stopped and its
	// channels given back before the Flexcomm reset
	NVIC_DisableIRQ(irq);
	dev->bRun = false;
	dev->bAsync = false;
	dev->DmaLen = 0;
	dev->SRxDmaLen = 0;
	dev->STxDmaLen = 0;
	Lpc546xxSPIDmaFree(dev);

	// Bus clock on, Flexcomm reset with the SPI function selected. Fails if
	// the Flexcomm is locked to the USART or I2C.
	dev->FClk = Lpc546xxFlexcommSelect(dev->DevNo, LPC546XX_FLEXCOMM_SPI);
	if (dev->FClk == 0)
	{
		EnableInterrupt(state);

		return false;
	}

	SharedIntrfSetIrqHandler(dev->DevNo, NULL, NULL);

	memcpy(&pDev->Cfg, pCfgData, sizeof(SPICfg_t));
	if (pDev->Cfg.MaxRetry <= 0)
	{
		pDev->Cfg.MaxRetry = SPI_MAX_RETRY;
	}
	if (slave)
	{
		// The master drives the frames, only the interrupt can follow
		pDev->Cfg.bIntEn = true;
	}

	dev->pSpiDev = pDev;
	dev->bSlave = slave;
	dev->TxCtrl = LPC546XX_SPI_TXSSEL_OFF | SPI_FIFOWR_LEN(pCfgData->DataSize - 1U);

	if (pCfgData->bDmaEn)
	{
		int rxch = LPC546XX_DMA_CH_FC_RX(dev->DevNo);
		int txch = LPC546XX_DMA_CH_FC_TX(dev->DevNo);

		if (Lpc546xxDmaChClaim(rxch, Lpc546xxSPIDmaIrq, dev, pCfgData->IntPrio, pDev->Cfg.bIntEn) == false)
		{
			Lpc546xxFlexcommClock(dev->DevNo, false);
			EnableInterrupt(state);

			return false;
		}
		if (Lpc546xxDmaChClaim(txch, Lpc546xxSPIDmaIrq, dev, pCfgData->IntPrio, pDev->Cfg.bIntEn) == false)
		{
			Lpc546xxDmaChFree(rxch);
			Lpc546xxFlexcommClock(dev->DevNo, false);
			EnableInterrupt(state);

			return false;
		}
		dev->bDmaClaimed = true;
		Lpc546xxDmaChSetup(rxch, false, &dev->pReg->FIFORD, 0);
		Lpc546xxDmaChSetup(txch, true, &dev->pReg->FIFOWR, 0);
	}

	pDev->FirstRdData = 0;
	pDev->CurDevCs = 0;
	memset(pDev->pRxBuff, 0, sizeof(pDev->pRxBuff));
	memset(pDev->RxBuffLen, 0, sizeof(pDev->RxBuffLen));
	memset(pDev->pTxData, 0, sizeof(pDev->pTxData));
	memset(pDev->TxDataLen, 0, sizeof(pDev->TxDataLen));
	pDev->DevIntrf.pDevData = dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.bIntEn = pDev->Cfg.bIntEn;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.GetHandle = Lpc546xxSPIGetHandle;
	pDev->DevIntrf.Disable = Lpc546xxSPIDisable;
	pDev->DevIntrf.Enable = Lpc546xxSPIEnable;
	pDev->DevIntrf.PowerOff = Lpc546xxSPIPowerOff;
	pDev->DevIntrf.Reset = Lpc546xxSPIReset;
	pDev->DevIntrf.GetRate = Lpc546xxSPIGetRate;
	pDev->DevIntrf.SetRate = Lpc546xxSPISetRate;
	pDev->DevIntrf.StartRx = Lpc546xxSPIStart;
	pDev->DevIntrf.RxData = Lpc546xxSPIRxData;
	pDev->DevIntrf.StopRx = Lpc546xxSPIStop;
	pDev->DevIntrf.StartTx = Lpc546xxSPIStart;
	pDev->DevIntrf.TxData = Lpc546xxSPITxData;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.StopTx = Lpc546xxSPIStop;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	if (slave == false)
	{
		// Chip selects deasserted
		for (int i = SPI_CS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
		{
			IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
		}
	}

	uint32_t cfg = slave ? 0U : SPI_CFG_MASTER_MASK;

	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cfg |= SPI_CFG_LSBF_MASK;
	}
	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cfg |= SPI_CFG_CPHA_MASK;
	}
	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
		cfg |= SPI_CFG_CPOL_MASK;
	}

	SPI_Type *reg = dev->pReg;

	reg->CFG = cfg;
	reg->DLY = 0;
	reg->INTENCLR = 0xFFFFFFFFU;
	reg->FIFOCFG = SPI_FIFOCFG_ENABLETX_MASK | SPI_FIFOCFG_ENABLERX_MASK |
				   SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
	reg->FIFOINTENCLR = 0xFFFFFFFFU;

	int size = (int)((reg->FIFOCFG & SPI_FIFOCFG_SIZE_MASK) >> SPI_FIFOCFG_SIZE_SHIFT) * 8;

	dev->FifoSize = size > 0 ? size : LPC546XX_SPI_FIFO_DEFAULT;

	// RX level trigger on each frame. TX level of the slave refill at half
	// the FIFO, of the DMA request while the FIFO is not full.
	reg->FIFOTRIG = SPI_FIFOTRIG_RXLVLENA_MASK | SPI_FIFOTRIG_RXLVL(0) | SPI_FIFOTRIG_TXLVLENA_MASK |
					SPI_FIFOTRIG_TXLVL(pCfgData->bDmaEn ? dev->FifoSize - 1 : dev->FifoSize / 2);

	if (Lpc546xxSPISetRate(&pDev->DevIntrf, pCfgData->Rate) == 0)
	{
		Lpc546xxSPIDmaFree(dev);
		Lpc546xxFlexcommClock(dev->DevNo, false);
		EnableInterrupt(state);

		return false;
	}

	if (pDev->Cfg.bIntEn)
	{
		SharedIntrfSetIrqHandler(dev->DevNo, &pDev->DevIntrf, Lpc546xxSPIIrqHandler);
		NVIC_ClearPendingIRQ(irq);
		NVIC_SetPriority(irq, pCfgData->IntPrio);
		NVIC_EnableIRQ(irq);
	}

	if (slave)
	{
		reg->INTENSET = SPI_INTENSET_SSDEN_MASK;
		Lpc546xxSPISlvArm(dev);
	}
	else
	{
		reg->CFG = cfg | SPI_CFG_ENABLE_MASK;
	}

	EnableInterrupt(state);

	return true;
}
