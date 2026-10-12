/**-------------------------------------------------------------------------
@file	uart_ra4m1.cpp

@brief	RA4M1 SCI UART implementation.

		UARTCfg_t.DevNo 0, 1, 2 and 3 select SCI0, SCI1, SCI2 and SCI9.
		SCI0 and SCI1 use their 16 byte hardware FIFOs, SCI2 and SCI9 use
		RDR and TDR. Asynchronous full duplex, 7 or 8 data bits, parity none,
		odd or even, 1 or 2 stop bits. The pin map is RX then TX.

		TxData puts the data in the TX CFIFO, then starts TX if it is ready.
		RxData reads from the RX CFIFO. Without the interrupt, TxData and
		RxData both move what the hardware holds, in both directions.

		bIntMode false : polling, TxData and RxData move the CFIFO data to and
		                 from the SCI
		bIntMode true  : RXI, TXI and ERI interrupts drain the TX CFIFO and
		                 fill the RX CFIFO
		bDMAMode true  : TX DMA sends blocks from the TX CFIFO, DMAC channel
		                 DevNo activated by the TXI of the SCI. RX as without
		                 DMA: the SCI has no idle line event to move a DMA
		                 ring to the RX CFIFO after a short burst. Without
		                 bIntMode, TxData and RxData retire the finished
		                 block and start the next one. Needs a blocking FIFO.

		The SCI clock is PCLKA, read with SystemPeriphClockGet(0).

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
#include <string.h>

#include "ra4m1xxx.h"

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "interrupt_ra4m1.h"

// Hardware FIFO size of SCI0 and SCI1
#define RA4M1_SCI_HWFIFO_SIZE		16

// FIFO mode. RXI when the RX FIFO holds RX_TRIGGER characters, or with fewer
// once the line is idle for 1.5 frames (receive data ready, FCR.DRES = 0).
// TXI when the TX FIFO holds TX_TRIGGER characters or less. With DMA, TXI
// when it is empty, as the Renesas FSP sets it with a transfer instance: each
// TXI moves one byte, its write clears TDFE.
#define RA4M1_SCI_RX_TRIGGER		8
#define RA4M1_SCI_TX_TRIGGER		8
#define RA4M1_SCI_TX_TRIGGER_DMA	0

#define RA4M1_SCI_SSR_RXERR			(R_SCI0_SSR_ORER_Msk | R_SCI0_SSR_FER_Msk | R_SCI0_SSR_PER_Msk)

// SCMR reset value : CHR1 = 1 (SMR.CHR selects 7 or 8 bits), LSB first,
// no inversion, asynchronous mode. The other bits are reserved and read 1.
#define RA4M1_SCI_SCMR_ASYNC		0xF2U

// Default CFIFO size when the configuration gives no memory
#ifndef RA4M1_UART_CFIFO_SIZE
#define RA4M1_UART_CFIFO_SIZE		16
#endif
#define RA4M1_UART_CFIFO_MEMSIZE	CFIFO_MEMSIZE(RA4M1_UART_CFIFO_SIZE)

// Most bytes TxData and RxData copy to or from a CFIFO with interrupts
// disabled, to keep the RX interrupt latency short
#define RA4M1_UART_COPY_MAX			8

// DMAC and DTC module stop bit in MSTPCRA
#define RA4M1_DMAC_MSTP_MASK		(1U << 22)

// Most bytes of one TX DMA block, the normal mode count of DMCRA
#define RA4M1_UART_DMA_MAX			0xFFFF

// Caller of Ra4m1UartRxHw. Without the FIFO it decides when RDR is read.
typedef enum {
	RA4M1_UART_RX_POLL,				// Polling, RDR read with RDRF or an error flag
	RA4M1_UART_RX_RXI,				// RXI, RDR read, RDRF may already be cleared
	RA4M1_UART_RX_OTHER,			// ERI or TXI, RDR read only with an error flag
} RA4M1_UART_RX;

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	R_SCI0_Type *pReg;
	uint32_t MstpMask;				// SCI module stop bit in MSTPCRB
	uint8_t EvtRxi;					// ICU event of RXI. TXI, TEI and ERI follow it.
	bool bHwFifo;					// 16 byte FIFOs (SCI0 and SCI1)
	uint8_t RxDataMask;				// 0x7F with 7 data bits
	uint32_t SyncCnt;				// Wait loops for 2 PCLKA cycles
	UARTDev_t *pUartDev;			// Pointer to generic UART dev. data
	const IOPinCfg_t *pIOPinMap;	// RX, TX
	int NbIOPins;
	IRQn_Type IrqRxi;
	IRQn_Type IrqTxi;
	IRQn_Type IrqEri;
	R_DMAC0_Type *pDma;				// DMAC channel of the TX DMA
	uint8_t EvtDma;					// ICU event of its transfer end
	IRQn_Type IrqDma;
	bool bDmaActive;				// TX DMA block running
	uint32_t DmaLen;				// TX CFIFO bytes of the running block
	alignas(4) uint8_t RxFifoMem[RA4M1_UART_CFIFO_MEMSIZE];
	alignas(4) uint8_t TxFifoMem[RA4M1_UART_CFIFO_MEMSIZE];
} Ra4m1UartDev_t;
#pragma pack(pop)

static Ra4m1UartDev_t s_Ra4m1UartDev[] = {
	{
		.DevNo = 0,
		.pReg = R_SCI0,
		.MstpMask = 1UL << 31,
		.EvtRxi = RA4M1_EVTID_SCI0_RXI,
		.bHwFifo = true,
		.IrqRxi = (IRQn_Type)-1,
		.IrqTxi = (IRQn_Type)-1,
		.IrqEri = (IRQn_Type)-1,
		.pDma = R_DMAC0,
		.EvtDma = RA4M1_EVTID_DMAC0_INT,
		.IrqDma = (IRQn_Type)-1,
	},
	{
		.DevNo = 1,
		.pReg = R_SCI1,
		.MstpMask = 1UL << 30,
		.EvtRxi = RA4M1_EVTID_SCI1_RXI,
		.bHwFifo = true,
		.IrqRxi = (IRQn_Type)-1,
		.IrqTxi = (IRQn_Type)-1,
		.IrqEri = (IRQn_Type)-1,
		.pDma = R_DMAC1,
		.EvtDma = RA4M1_EVTID_DMAC1_INT,
		.IrqDma = (IRQn_Type)-1,
	},
	{
		.DevNo = 2,
		.pReg = R_SCI2,
		.MstpMask = 1UL << 29,
		.EvtRxi = RA4M1_EVTID_SCI2_RXI,
		.bHwFifo = false,
		.IrqRxi = (IRQn_Type)-1,
		.IrqTxi = (IRQn_Type)-1,
		.IrqEri = (IRQn_Type)-1,
		.pDma = R_DMAC2,
		.EvtDma = RA4M1_EVTID_DMAC2_INT,
		.IrqDma = (IRQn_Type)-1,
	},
	{
		.DevNo = 3,
		.pReg = R_SCI9,
		.MstpMask = 1UL << 22,
		.EvtRxi = RA4M1_EVTID_SCI9_RXI,
		.bHwFifo = false,
		.IrqRxi = (IRQn_Type)-1,
		.IrqTxi = (IRQn_Type)-1,
		.IrqEri = (IRQn_Type)-1,
		.pDma = R_DMAC3,
		.EvtDma = RA4M1_EVTID_DMAC3_INT,
		.IrqDma = (IRQn_Type)-1,
	},
};

static const int s_NbRa4m1UartDev = sizeof(s_Ra4m1UartDev) / sizeof(s_Ra4m1UartDev[0]);

// Baud = PCLKA * M / (256 * Div * 4^CKS * (BRR + 1)). Div from SEMR.BGDM, ABCS
// and ABCSE. M is MDDR with modulation (SEMR.BRME), 256 without.
static const struct {
	uint32_t Div;
	uint8_t Semr;
} s_Ra4m1UartClkDiv[] = {
	{32, 0},
	{16, R_SCI0_SEMR_BGDM_Msk},
	{8, R_SCI0_SEMR_BGDM_Msk | R_SCI0_SEMR_ABCS_Msk},
	{6, R_SCI0_SEMR_ABCSE_Msk},
};

// Read back and 2 PCLKA cycles after the last register write, before the ICU
// IR flag of the event is cleared.
static void Ra4m1UartSync(Ra4m1UartDev_t * const pDev)
{
	(void)pDev->pReg->SCR;
	__DSB();
	for (uint32_t i = 0; i < pDev->SyncCnt; i++)
	{
		__NOP();
	}
}

// SCI module stop bit, written with PRCR.PRC1 set
static void Ra4m1UartModuleStart(Ra4m1UartDev_t * const pDev, bool bStart)
{
	uint16_t prcr = R_SYSTEM->PRCR & 0xFU;

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

static void Ra4m1UartReleaseIRQs(Ra4m1UartDev_t * const pDev)
{
	Ra4m1UnregisterIntHandler(pDev->IrqRxi);
	Ra4m1UnregisterIntHandler(pDev->IrqTxi);
	Ra4m1UnregisterIntHandler(pDev->IrqEri);
	Ra4m1UnregisterIntHandler(pDev->IrqDma);
	pDev->IrqRxi = pDev->IrqTxi = pDev->IrqEri = pDev->IrqDma = (IRQn_Type)-1;
}

// Received characters into the RX CFIFO. The hardware is read first, a full
// CFIFO drops the character (RxDropCnt). A character with a framing or parity
// error is dropped. Returns the number of characters read from the hardware,
// pErr the line state errors seen. Interrupts disabled or from the interrupt.
//
// Src : caller, it decides when RDR is read without the FIFO. RXI tells that
// RDR holds a character, RDRF may already be cleared by then. A character read
// on another interrupt would be put again by its own RXI.
static int Ra4m1UartRxHw(Ra4m1UartDev_t * const pDev, RA4M1_UART_RX Src, uint8_t *pErr)
{
	UARTDev_t *udev = pDev->pUartDev;
	R_SCI0_Type *reg = pDev->pReg;
	int cnt = 0;
	uint8_t err = 0;

	if (pDev->bHwFifo)
	{
		// The FIFO is read until it is empty, then the flags are read and
		// cleared, as the Renesas FSP does. DR is cleared only with the FIFO
		// empty and RDF only below the trigger level. A flag left set gives no
		// new RXI, the next characters would wait for an overrun. Characters
		// arriving during a pass are read in one more pass. Bounded at twice
		// the FIFO size, the line does not fill it faster than it is read.
		int n = (reg->FDR & R_SCI0_FDR_R_Msk) >> R_SCI0_FDR_R_Pos;

		do {
			// Each FIFO entry has its own framing and parity status
			for (; n > 0; n--, cnt++)
			{
				uint16_t d = reg->FRDRHL;

				if (d & (R_SCI0_FRDRHL_FER_Msk | R_SCI0_FRDRHL_PER_Msk))
				{
					if (d & R_SCI0_FRDRHL_FER_Msk)
					{
						udev->FramErrCnt++;
						err |= UART_LINESTATE_FRMERR;
					}
					if (d & R_SCI0_FRDRHL_PER_Msk)
					{
						udev->ParErrCnt++;
						err |= UART_LINESTATE_PARERR;
					}
					udev->RxDropCnt++;
				}
				else if (CFifoPutByte(udev->hRxFifo, (uint8_t)d & pDev->RxDataMask))
				{
					udev->bRxReady = true;
				}
				else
				{
					udev->RxDropCnt++;
				}
			}

			uint8_t status = reg->SSR_FIFO;

			// Overrun loses the arriving characters until ORER is cleared, the
			// FIFO content is kept
			if (status & R_SCI0_SSR_FIFO_ORER_Msk)
			{
				udev->RxOvrErrCnt++;
				err |= UART_LINESTATE_OVR;
			}

			// Flags cleared by writing 0 to them only
			uint8_t clr = status & (R_SCI0_SSR_FIFO_RDF_Msk | R_SCI0_SSR_FIFO_ORER_Msk |
									R_SCI0_SSR_FIFO_FER_Msk | R_SCI0_SSR_FIFO_PER_Msk |
									R_SCI0_SSR_FIFO_DR_Msk);
			if (clr)
			{
				reg->SSR_FIFO = (uint8_t)~clr;
			}
			n = (reg->FDR & R_SCI0_FDR_R_Msk) >> R_SCI0_FDR_R_Pos;
		} while (n > 0 && cnt < 2 * RA4M1_SCI_HWFIFO_SIZE);
	}
	else
	{
		uint8_t status = reg->SSR;

		if (status & RA4M1_SCI_SSR_RXERR)
		{
			// RDR holds the character received with the error. After an
			// overrun alone it holds the last good character, the arriving
			// one is lost. That character is put only if RDRF says it was
			// not read yet, an RXI may have read it already.
			uint8_t d = reg->RDR;

			if (status & R_SCI0_SSR_ORER_Msk)
			{
				udev->RxOvrErrCnt++;
				err |= UART_LINESTATE_OVR;
			}
			if (status & R_SCI0_SSR_FER_Msk)
			{
				udev->FramErrCnt++;
				err |= UART_LINESTATE_FRMERR;
			}
			if (status & R_SCI0_SSR_PER_Msk)
			{
				udev->ParErrCnt++;
				err |= UART_LINESTATE_PARERR;
			}
			if (status & (R_SCI0_SSR_FER_Msk | R_SCI0_SSR_PER_Msk))
			{
				udev->RxDropCnt++;
			}
			else if ((status & R_SCI0_SSR_RDRF_Msk) == 0)
			{
				// Overrun alone, RDR already read
			}
			else if (CFifoPutByte(udev->hRxFifo, d & pDev->RxDataMask))
			{
				udev->bRxReady = true;
			}
			else
			{
				udev->RxDropCnt++;
			}
			// Error flags cleared by writing 0 to them only
			reg->SSR = (uint8_t)~(status & RA4M1_SCI_SSR_RXERR);
			cnt = 1;
		}
		else if (Src == RA4M1_UART_RX_RXI || (Src == RA4M1_UART_RX_POLL && (status & R_SCI0_SSR_RDRF_Msk)))
		{
			// Reading RDR clears RDRF
			if (CFifoPutByte(udev->hRxFifo, reg->RDR & pDev->RxDataMask))
			{
				udev->bRxReady = true;
			}
			else
			{
				udev->RxDropCnt++;
			}
			cnt = 1;
		}
	}

	if (err)
	{
		if ((err & UART_LINESTATE_FRMERR) && (reg->SPTR & R_SCI0_SPTR_RXDMON_Msk) == 0)
		{
			// RXD still low after a framing error
			err |= UART_LINESTATE_BRK;
		}
		udev->LineState = err;
	}
	*pErr = err;

	return cnt;
}

// TX CFIFO bytes into the SCI while it has room. Returns the number of bytes
// moved. Interrupts disabled or from the interrupt.
static int Ra4m1UartTxHw(Ra4m1UartDev_t * const pDev)
{
	R_SCI0_Type *reg = pDev->pReg;
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;
	int cnt = 0;

	if (pDev->bHwFifo)
	{
		int room = RA4M1_SCI_HWFIFO_SIZE - (int)((reg->FDR & R_SCI0_FDR_T_Msk) >> R_SCI0_FDR_T_Pos);

		for (; cnt < room; cnt++)
		{
			int c = CFifoGetByte(fifo);
			if (c < 0)
			{
				break;
			}
			reg->FTDRL = (uint8_t)c;
		}

		// TDFE is cleared by writing 0 after reading 1. It is set again when
		// the FIFO goes down to the trigger level, which gives the next TXI.
		if (reg->SSR_FIFO & R_SCI0_SSR_FIFO_TDFE_Msk)
		{
			reg->SSR_FIFO = (uint8_t)~R_SCI0_SSR_FIFO_TDFE_Msk;
		}
	}
	else if (reg->SSR & R_SCI0_SSR_TDRE_Msk)
	{
		int c = CFifoGetByte(fifo);
		if (c >= 0)
		{
			// Writing TDR clears TDRE, TXI comes when TDR moves to the shift
			// register
			reg->TDR = (uint8_t)c;
			cnt = 1;
		}
	}

	return cnt;
}

// TX DMA. DMAC channel DevNo moves one byte to FTDRL or TDR for each TXI of
// the SCI, linked through ICU.DELSR. A block is the first contiguous run of
// the TX CFIFO, sent in place: its bytes stay in the FIFO, the producer cannot
// write over them, until the channel has moved all of them. The transfer end
// interrupt, or TxData and RxData without bIntMode, release them and start
// the next block.

// DMAC module running and its transfers enabled. Not stopped by this driver,
// other drivers may use the DMAC or the DTC.
static void Ra4m1UartDmacStart(void)
{
	uint16_t prcr = R_SYSTEM->PRCR & 0xFU;

	R_SYSTEM->PRCR = 0xA500U | prcr | R_SYSTEM_PRCR_PRC1_Msk;
	R_MSTP->MSTPCRA &= ~RA4M1_DMAC_MSTP_MASK;
	(void)R_MSTP->MSTPCRA;
	R_SYSTEM->PRCR = 0xA500U | prcr;
	R_DMA->DMAST = R_DMA_DMAST_DMST_Msk;
}

// Channel stopped and unlinked from the TXI. A request pending in DELSR goes
// with the link. The bytes of a running block are not released, Enable
// flushes the TX CFIFO.
static void Ra4m1UartDmaStop(Ra4m1UartDev_t * const pDev)
{
	R_ICU->DELSR[pDev->DevNo] = 0;
	pDev->pDma->DMCNT = 0;
	pDev->bDmaActive = false;
	pDev->DmaLen = 0;
}

// Channel set for TX and linked to the TXI: normal mode, 8 bit, source
// incremented, destination FTDRL or TDR fixed, transfer end interrupt with
// bIntMode. Started by Ra4m1UartDmaTxStart.
static void Ra4m1UartDmaSetup(Ra4m1UartDev_t * const pDev)
{
	R_DMAC0_Type *dma = pDev->pDma;
	R_SCI0_Type *reg = pDev->pReg;

	Ra4m1UartDmaStop(pDev);
	dma->DMTMD = 1U << R_DMAC0_DMTMD_DCTG_Pos;
	dma->DMAMD = 2U << R_DMAC0_DMAMD_SM_Pos;
	dma->DMDAR = pDev->bHwFifo ? (uint32_t)(uintptr_t)&reg->FTDRL : (uint32_t)(uintptr_t)&reg->TDR;
	dma->DMOFR = 0;
	dma->DMCRB = 0;
	dma->DMINT = pDev->pUartDev->DevIntrf.bIntEn ? R_DMAC0_DMINT_DTIE_Msk : 0;
	dma->DMSTS = 0;
	R_ICU->DELSR[pDev->DevNo] = pDev->EvtRxi + 1U;
}

// Start a block with the first contiguous run of the TX CFIFO, at most half
// of the FIFO: the producer fills the other half meanwhile and the next block
// is ready when this one ends. Returns false when the FIFO is empty.
// Interrupts disabled or from an interrupt.
//
// SCI0/1: the channel is armed, then TIE is set again. Setting TIE gives a
// TXI when the TX FIFO is already empty, the Renesas FSP starts its transfers
// that way, otherwise the TXI comes when the last byte of the previous block
// leaves the FIFO.
//
// SCI2/9: setting TIE gives no TXI, it comes when TDR moves to the shift
// register. TIE is set first, then the channel is armed for the whole block:
// the byte left in TDR by the previous block gives the TXI, and a byte the
// channel writes to an idle line goes on to the shift register at once and
// gives the next TXI. When TDR is empty with bytes left and the channel
// stopped, no TXI is coming: the TXI came before the channel was armed, or the
// line is idle. The request it may have left in DELSR is cleared and the next
// byte goes to TDR from the CPU, as the Renesas FSP starts a transfer, the
// channel takes the rest. A block of one byte is written to TDR alone and the
// next run is taken. That happens at most twice in a row: after a TDR write
// TDR is full unless the line was idle.
static bool Ra4m1UartDmaTxStart(Ra4m1UartDev_t * const pDev)
{
	R_SCI0_Type *reg = pDev->pReg;
	R_DMAC0_Type *dma = pDev->pDma;
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;

	int half = (CFifoUsed(fifo) + CFifoAvail(fifo)) / 2;

	if (half < 1)
	{
		half = 1;
	}
	if (half > RA4M1_UART_DMA_MAX)
	{
		half = RA4M1_UART_DMA_MAX;
	}

	for (;;)
	{
		int cnt = half;
		uint8_t *p = CFifoPeekMultiple(fifo, &cnt);

		if (p == NULL)
		{
			return false;
		}

		pDev->DmaLen = (uint32_t)cnt;
		pDev->bDmaActive = true;
		dma->DMSAR = (uint32_t)(uintptr_t)p;
		dma->DMCRA = (uint32_t)cnt;
		// The CFIFO bytes are written before the channel reads them
		__DSB();

		if (pDev->bHwFifo)
		{
			dma->DMCNT = R_DMAC0_DMCNT_DTE_Msk;
			reg->SCR &= (uint8_t)~R_SCI0_SCR_TIE_Msk;
			reg->SCR |= R_SCI0_SCR_TIE_Msk;

			return true;
		}

		reg->SCR |= R_SCI0_SCR_TIE_Msk;
		dma->DMCNT = R_DMAC0_DMCNT_DTE_Msk;

		uint32_t left;
		for (;;)
		{
			// Time for the channel to take a request pending before DTE was set
			Ra4m1UartSync(pDev);
			left = dma->DMCRA;
			if (left == 0 || (reg->SSR & R_SCI0_SSR_TDRE_Msk) == 0)
			{
				// All moved, or TDR full and its move gives the next TXI
				return true;
			}
			dma->DMCNT = 0;
			if (dma->DMCRA == left)
			{
				break;
			}
			// The channel moved a byte meanwhile
			dma->DMCNT = R_DMAC0_DMCNT_DTE_Msk;
		}

		// TDR empty, channel stopped with bytes left
		R_ICU->DELSR[pDev->DevNo] = 0;
		R_ICU->DELSR[pDev->DevNo] = pDev->EvtRxi + 1U;

		uint8_t *q = p + (cnt - (int)left);

		if (left > 1)
		{
			dma->DMSAR = (uint32_t)(uintptr_t)(q + 1);
			dma->DMCRA = left - 1;
			dma->DMCNT = R_DMAC0_DMCNT_DTE_Msk;
			reg->TDR = *q;

			return true;
		}
		reg->TDR = *q;
		CFifoRelease(fifo, cnt);
		pDev->bDmaActive = false;
		pDev->DmaLen = 0;
	}
}

// A block is done once the channel cleared DTE after its last byte. Its bytes
// are released and the next block takes what was queued meanwhile. With the
// FIFO empty TX is ready again and the application is told. From the transfer
// end interrupt, or from TxData and RxData without bIntMode. Returns false if
// no block was done.
static bool Ra4m1UartDmaService(Ra4m1UartDev_t * const pDev)
{
	UARTDev_t *udev = pDev->pUartDev;
	R_DMAC0_Type *dma = pDev->pDma;

	if (!pDev->bDmaActive || (dma->DMCNT & R_DMAC0_DMCNT_DTE_Msk))
	{
		return false;
	}

	dma->DMSTS = 0;
	pDev->bDmaActive = false;
	CFifoRelease(udev->hTxFifo, (int)pDev->DmaLen);
	pDev->DmaLen = 0;

	if (Ra4m1UartDmaTxStart(pDev))
	{
		return true;
	}

	// No TXI for the channel while TX is idle
	pDev->pReg->SCR &= (uint8_t)~R_SCI0_SCR_TIE_Msk;
	udev->bTxReady = true;
	if (udev->DevIntrf.bIntEn && udev->EvtCallback)
	{
		// Length is the free space in the TX FIFO
		udev->EvtCallback(udev, UART_EVT_TXREADY, NULL, CFifoAvail(udev->hTxFifo));
	}

	return true;
}

// Transfer end of the TX DMA channel, an edge event cleared first
static void Ra4m1UartDmaIRQHandler(int IntNo, void *pCtx)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pCtx;

	Ra4m1AcknowledgeInt((IRQn_Type)IntNo);
	Ra4m1UartDmaService(dev);
}

// Without the interrupt, RxData and TxData move what the hardware holds, in
// both directions. With DMA the finished TX block is retired and the next one
// started. Interrupts disabled.
static void Ra4m1UartPoll(Ra4m1UartDev_t * const pDev)
{
	uint8_t err;

	Ra4m1UartRxHw(pDev, RA4M1_UART_RX_POLL, &err);
	if (pDev->pUartDev->DevIntrf.bDma)
	{
		Ra4m1UartDmaService(pDev);
		if (pDev->pUartDev->bTxReady && Ra4m1UartDmaTxStart(pDev))
		{
			pDev->pUartDev->bTxReady = false;
		}
	}
	else
	{
		Ra4m1UartTxHw(pDev);
	}
}

// RXI, TXI and ERI of one SCI. RX is serviced first whatever the event.
static void Ra4m1UartIRQHandler(int IntNo, void *pCtx)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;
	R_SCI0_Type *reg = dev->pReg;
	uint8_t err = 0;
	bool txevt = false;
	bool eri = IntNo == (int)dev->IrqEri;

	// RXI and TXI are edge events. Their ICU IR flag is cleared first, as the
	// Renesas FSP does: an event while the handler runs sets it again and the
	// handler runs once more. Cleared at the end, that event would be lost,
	// RDF or DR left set with no RXI, or TDRE set again by a TDR write to an
	// idle line with no TXI.
	if (!eri)
	{
		Ra4m1AcknowledgeInt((IRQn_Type)IntNo);
	}

	int rxcnt = Ra4m1UartRxHw(dev, IntNo == (int)dev->IrqRxi ? RA4M1_UART_RX_RXI : RA4M1_UART_RX_OTHER, &err);

	// With DMA, TIE is for the channel and TXI goes to it
	if (!udev->DevIntrf.bDma && (reg->SCR & R_SCI0_SCR_TIE_Msk))
	{
		Ra4m1UartTxHw(dev);
		if (CFifoUsed(udev->hTxFifo) == 0)
		{
			// TX CFIFO empty, TxData starts the next transmission
			reg->SCR &= (uint8_t)~R_SCI0_SCR_TIE_Msk;
			udev->bTxReady = true;
		}
		txevt = true;
	}

	if (eri || err)
	{
		// The error flags are cleared, the ERI request is negated and its ICU
		// IR flag can be cleared. Without the FIFO, RXI may also be pending
		// for the character just read with the error.
		Ra4m1UartSync(dev);
		Ra4m1AcknowledgeInt(dev->IrqEri);
		if (err && !dev->bHwFifo)
		{
			Ra4m1AcknowledgeInt(dev->IrqRxi);
		}
	}

	if (udev->EvtCallback)
	{
		if (err)
		{
			udev->EvtCallback(udev, UART_EVT_LINESTATE, &err, 1);
		}
		if (rxcnt > 0)
		{
			udev->EvtCallback(udev, UART_EVT_RXDATA, NULL, CFifoUsed(udev->hRxFifo));
		}
		if (txevt)
		{
			udev->EvtCallback(udev, UART_EVT_TXREADY, NULL, CFifoAvail(udev->hTxFifo));
		}
	}
}

static uint32_t Ra4m1UartGetRate(DevIntrf_t * const pDev)
{
	return ((Ra4m1UartDev_t *)pDev->pDevData)->pUartDev->Rate;
}

// Sets the closest rate the SCI can make and returns it. 0 is the automatic
// rate, which the SCI does not have.
static uint32_t Ra4m1UartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pDev->pDevData;
	R_SCI0_Type *reg = dev->pReg;
	uint32_t pclk = SystemPeriphClockGet(0);

	if (Rate == 0 || pclk == 0)
	{
		return 0;
	}

	uint32_t baud = 0;
	uint32_t diff = UINT32_MAX;
	uint8_t brr = 0;
	uint8_t cks = 0;
	uint8_t semr = 0;
	uint8_t mddr = 0xFF;

	for (unsigned i = 0; i < sizeof(s_Ra4m1UartClkDiv) / sizeof(s_Ra4m1UartClkDiv[0]); i++)
	{
		for (unsigned n = 0; n < 4; n++)
		{
			// PCLKA cycles per bit with BRR = 0
			uint64_t clk = (uint64_t)s_Ra4m1UartClkDiv[i].Div << (2 * n);

			// Without modulation, BRR + 1 closest to the rate
			uint64_t div = ((uint64_t)pclk + clk * Rate / 2) / (clk * Rate);
			if (div < 1)
			{
				div = 1;
			}
			if (div > 256)
			{
				div = 256;
			}
			uint32_t actual = (uint32_t)(((uint64_t)pclk + clk * div / 2) / (clk * div));
			uint32_t e = actual > Rate ? actual - Rate : Rate - actual;
			if (e < diff || (e == diff && (semr & R_SCI0_SEMR_BRME_Msk)))
			{
				diff = e;
				baud = actual;
				brr = (uint8_t)(div - 1);
				cks = (uint8_t)n;
				semr = s_Ra4m1UartClkDiv[i].Semr;
				mddr = 0xFF;
			}

			// Modulation lowers the rate by M / 256, M from 128 to 255. Every
			// BRR + 1 giving a rate from the one wanted to twice it is tried,
			// with the closest M.
			uint64_t dmax = (uint64_t)pclk / (clk * Rate);
			uint64_t dmin = ((uint64_t)pclk + clk * Rate * 2 - 1) / (clk * Rate * 2);
			if (dmax > 256)
			{
				dmax = 256;
			}
			if (dmin < 1)
			{
				dmin = 1;
			}
			for (div = dmin; div <= dmax; div++)
			{
				uint64_t m = ((uint64_t)Rate * clk * div * 256 + pclk / 2) / pclk;
				if (m < 128 || m > 255)
				{
					continue;
				}
				actual = (uint32_t)(((uint64_t)pclk * m + 128 * clk * div) / (256 * clk * div));
				e = actual > Rate ? actual - Rate : Rate - actual;
				if (e < diff)
				{
					diff = e;
					baud = actual;
					brr = (uint8_t)(div - 1);
					cks = (uint8_t)n;
					semr = s_Ra4m1UartClkDiv[i].Semr | R_SCI0_SEMR_BRME_Msk;
					mddr = (uint8_t)m;
				}
			}
		}
	}

	uint32_t state = DisableInterrupt();

	// SMR, SEMR, BRR and MDDR are written with TE and RE cleared
	uint8_t scr = reg->SCR;
	reg->SCR = 0;
	reg->SMR = (uint8_t)((reg->SMR & ~R_SCI0_SMR_CKS_Msk) | cks);
	reg->SEMR = (uint8_t)((reg->SEMR & ~(R_SCI0_SEMR_BGDM_Msk | R_SCI0_SEMR_ABCS_Msk |
						R_SCI0_SEMR_ABCSE_Msk | R_SCI0_SEMR_BRME_Msk)) | semr);
	reg->BRR = brr;
	reg->MDDR = mddr;
	dev->pUartDev->Rate = baud;

	if (scr != 0)
	{
		// One bit time for the baud rate generator before TE and RE are set
		// again
		for (uint32_t wait = SystemCoreClock / baud + 1; wait > 0; wait--)
		{
			__NOP();
		}
		reg->SCR = scr;
	}

	EnableInterrupt(state);

	return baud;
}

static bool Ra4m1UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int Ra4m1UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	hCFifo_t fifo = udev->hRxFifo;
	bool poll = !pDev->bIntEn;
	int cnt = 0;

	if (pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

	// The data comes from the RX CFIFO. CFifoGetMultiple frees the slots
	// before they are copied, interrupts are disabled until the copy is done
	// so that the interrupt cannot put over them. In pieces of at most
	// RA4M1_UART_COPY_MAX bytes. Without the interrupt the SCI is serviced
	// before each piece, the characters received go in the CFIFO after the
	// ones already there.
	while (cnt < Bufflen)
	{
		int l = Bufflen - cnt < RA4M1_UART_COPY_MAX ? Bufflen - cnt : RA4M1_UART_COPY_MAX;
		uint32_t state = DisableInterrupt();
		if (poll)
		{
			Ra4m1UartPoll(dev);
		}
		uint8_t *p = CFifoGetMultiple(fifo, &l);
		if (p != NULL)
		{
			memcpy(pBuff + cnt, p, l);
		}
		EnableInterrupt(state);
		if (p == NULL)
		{
			break;
		}
		cnt += l;
	}

	// Cleared first, then set again if a byte is left. A byte put by the
	// interrupt in between sets it too.
	udev->bRxReady = false;
	if (CFifoUsed(fifo) != 0)
	{
		udev->bRxReady = true;
	}

	if (poll)
	{
		uint32_t state = DisableInterrupt();
		Ra4m1UartPoll(dev);
		EnableInterrupt(state);
	}

	return cnt;
}

static void Ra4m1UartStopRx(DevIntrf_t * const pDev)
{
}

static bool Ra4m1UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

// Several bytes into the TX CFIFO. CFifoPutMultiple publishes the slots
// before they are written, so interrupts are disabled for the copy, in pieces
// of at most RA4M1_UART_COPY_MAX bytes. Without the interrupt the SCI is
// serviced before each piece. Not inlined in TxData, which keeps the one byte
// path short. Returns the number of bytes put.
__attribute__((noinline)) static int Ra4m1UartTxPut(Ra4m1UartDev_t * const pDev, uint8_t const *pData, int Datalen)
{
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;
	bool poll = !pDev->pUartDev->DevIntrf.bIntEn;
	int cnt = 0;

	while (Datalen > 0)
	{
		int l = Datalen < RA4M1_UART_COPY_MAX ? Datalen : RA4M1_UART_COPY_MAX;
		uint32_t state = DisableInterrupt();
		if (poll)
		{
			Ra4m1UartPoll(pDev);
		}
		uint8_t *p = CFifoPutMultiple(fifo, &l);
		if (p != NULL)
		{
			memcpy(p, pData, l);
		}
		EnableInterrupt(state);
		if (p == NULL)
		{
			break;
		}
		Datalen -= l;
		pData += l;
		cnt += l;
	}

	return cnt;
}

static int Ra4m1UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	int cnt = 0;

	if (pData == NULL || Datalen <= 0)
	{
		return 0;
	}

	// The data goes in the TX CFIFO first. CFifoPutByte writes the byte
	// before it is published.
	if (Datalen == 1)
	{
		cnt = CFifoPutByte(udev->hTxFifo, *pData) ? 1 : 0;
	}
	else
	{
		cnt = Ra4m1UartTxPut(dev, pData, Datalen);
	}

	if (!pDev->bIntEn)
	{
		// Without the interrupt what the SCI takes now is sent, the next
		// TxData or RxData call sends the rest. With DMA the finished block is
		// retired and the next one started.
		uint32_t state = DisableInterrupt();
		Ra4m1UartPoll(dev);
		EnableInterrupt(state);
	}
	else if (pDev->bDma)
	{
		// A DMA block from the FIFO, the transfer end interrupt sends the
		// rest. Checked again with interrupts disabled, TxData may also be
		// called from an interrupt.
		if (udev->bTxReady)
		{
			uint32_t state = DisableInterrupt();
			if (udev->bTxReady && Ra4m1UartDmaTxStart(dev))
			{
				udev->bTxReady = false;
			}
			EnableInterrupt(state);
		}
	}
	else if (udev->bTxReady)
	{
		// TX is started with what the SCI takes, the TXI interrupt sends the
		// rest. Checked again with interrupts disabled, TxData may also be
		// called from an interrupt.
		uint32_t state = DisableInterrupt();
		if (udev->bTxReady)
		{
			if (dev->bHwFifo)
			{
				// TXI comes when the full TX FIFO goes down to the trigger
				// level. Nothing left in the TX CFIFO, no TXI needed.
				Ra4m1UartTxHw(dev);
				if (CFifoUsed(udev->hTxFifo) != 0)
				{
					udev->bTxReady = false;
					dev->pReg->SCR |= R_SCI0_SCR_TIE_Msk;
				}
			}
			else
			{
				// TXI comes when TDR moves to the shift register, right after
				// the write when the line is idle. TIE is set before TDR is
				// written so that this TXI is not missed.
				udev->bTxReady = false;
				dev->pReg->SCR |= R_SCI0_SCR_TIE_Msk;
				Ra4m1UartTxHw(dev);
			}
		}
		EnableInterrupt(state);
	}

	return cnt;
}

static void Ra4m1UartStopTx(DevIntrf_t * const pDev)
{
}

// TE, RE and their interrupts off. Registers, pins and CFIFOs are kept,
// Enable restarts the UART.
static void Ra4m1UartDisable(DevIntrf_t * const pDev)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->SCR = 0;
	if (pDev->bDma)
	{
		Ra4m1UartDmaStop(dev);
	}

	EnableInterrupt(state);
}

static void Ra4m1UartEnable(DevIntrf_t * const pDev)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pDev->pDevData;
	R_SCI0_Type *reg = dev->pReg;
	uint32_t state = DisableInterrupt();

	dev->pUartDev->RxOvrErrCnt = 0;
	dev->pUartDev->ParErrCnt = 0;
	dev->pUartDev->FramErrCnt = 0;
	dev->pUartDev->RxDropCnt = 0;
	dev->pUartDev->TxDropCnt = 0;

	CFifoFlush(dev->pUartDev->hTxFifo);
	dev->pUartDev->bTxReady = true;

	// TE and RE are cleared here, the hardware FIFOs can be reset
	reg->SCR = 0;
	if (dev->bHwFifo)
	{
		reg->FCR |= R_SCI0_FCR_RFRST_Msk | R_SCI0_FCR_TFRST_Msk;
	}
	uint8_t status = reg->SSR;
	if (status & RA4M1_SCI_SSR_RXERR)
	{
		reg->SSR = (uint8_t)~(status & RA4M1_SCI_SSR_RXERR);
	}
	if (pDev->bDma)
	{
		Ra4m1UartDmaSetup(dev);
	}

	// TE and RE set in one write. TXI is enabled by TxData when the TX CFIFO
	// has data. RIE enables RXI and ERI.
	reg->SCR = R_SCI0_SCR_TE_Msk | R_SCI0_SCR_RE_Msk |
			   (pDev->bIntEn ? R_SCI0_SCR_RIE_Msk : 0);

	EnableInterrupt(state);
}

// Interrupts released, pins disconnected and the SCI module stopped. A new
// UARTInit is needed to use it again.
static void Ra4m1UartPowerOff(DevIntrf_t * const pDev)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->SCR = 0;
	if (pDev->bDma)
	{
		Ra4m1UartDmaStop(dev);
	}
	Ra4m1UartReleaseIRQs(dev);
	IOPinDis(dev->pIOPinMap, dev->NbIOPins);
	Ra4m1UartModuleStart(dev, false);

	EnableInterrupt(state);
}

static void *Ra4m1UartGetHandle(DevIntrf_t * const pDev)
{
	return ((Ra4m1UartDev_t *)pDev->pDevData)->pUartDev;
}

UARTDev_t const *UARTGetInstance(int DevNo)
{
	return DevNo >= 0 && DevNo < s_NbRa4m1UartDev ? s_Ra4m1UartDev[DevNo].pUartDev : NULL;
}

// No modem control lines on this implementation
void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
}

bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->pIOPinMap == NULL || pCfg->NbIOPins < 2)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbRa4m1UartDev)
	{
		return false;
	}

	// Asynchronous full duplex without flow control, 7 or 8 data bits,
	// parity none, odd or even. TX DMA reads the TX CFIFO in place, a put on
	// a full non-blocking FIFO would write over the running block.
	if ((pCfg->bDMAMode && !pCfg->bFifoBlocking) ||
		pCfg->Mode != UART_MODE_UART || pCfg->Duplex != UART_DUPLEX_FULL || pCfg->bIrDAMode ||
		pCfg->FlowControl != UART_FLWCTRL_NONE || (pCfg->DataBits != 7 && pCfg->DataBits != 8) ||
		(pCfg->Parity != UART_PARITY_NONE && pCfg->Parity != UART_PARITY_ODD &&
		 pCfg->Parity != UART_PARITY_EVEN) ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) || pCfg->Rate <= 0)
	{
		return false;
	}

	Ra4m1UartDev_t *dev = &s_Ra4m1UartDev[pCfg->DevNo];
	R_SCI0_Type *reg = dev->pReg;
	uint32_t state = DisableInterrupt();

	// An instance running for another UART object is not taken over
	if (dev->pUartDev != NULL && dev->pUartDev != pDev && (R_MSTP->MSTPCRB & dev->MstpMask) == 0 &&
		(reg->SCR & (R_SCI0_SCR_TE_Msk | R_SCI0_SCR_RE_Msk)))
	{
		EnableInterrupt(state);

		return false;
	}

	// DMAC channel DevNo, free or linked to the TXI of this SCI by an earlier
	// UARTInit. Linked to another event, another driver uses it.
	uint32_t dels = R_ICU->DELSR[pCfg->DevNo] & R_ICU_DELSR_DELS_Msk;
	if (pCfg->bDMAMode && dels != 0 && dels != dev->EvtRxi + 1U)
	{
		EnableInterrupt(state);

		return false;
	}

	// Interrupts and TX DMA of a previous UARTInit. Without DMA the TXI goes
	// to the CPU again.
	Ra4m1UartReleaseIRQs(dev);
	if (dels == dev->EvtRxi + 1U)
	{
		Ra4m1UartDmaStop(dev);
	}

	// Module clock first, register writes before it are ignored
	Ra4m1UartModuleStart(dev, true);
	reg->SCR = 0;
	if (pCfg->bDMAMode)
	{
		Ra4m1UartDmacStart();
	}

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(dev->RxFifoMem, RA4M1_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(dev->TxFifoMem, RA4M1_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	// CFifoInit returns NULL when the supplied memory is too small
	if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
	{
		Ra4m1UartModuleStart(dev, false);
		EnableInterrupt(state);

		return false;
	}

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;
	dev->pIOPinMap = (const IOPinCfg_t *)pCfg->pIOPinMap;
	dev->NbIOPins = 2;
	dev->RxDataMask = pCfg->DataBits == 7 ? 0x7F : 0xFF;

	uint32_t pclk = SystemPeriphClockGet(0);
	dev->SyncCnt = pclk ? 2 * ((SystemCoreClock + pclk - 1) / pclk) + 2 : 4;

	if (dev->bHwFifo)
	{
		// FIFO mode, both FIFOs reset. RSTRG (RTS level) is not used.
		uint16_t ttrg = pCfg->bDMAMode ? RA4M1_SCI_TX_TRIGGER_DMA : RA4M1_SCI_TX_TRIGGER;
		reg->FCR = R_SCI0_FCR_FM_Msk | R_SCI0_FCR_RFRST_Msk | R_SCI0_FCR_TFRST_Msk |
				   (RA4M1_SCI_RX_TRIGGER << R_SCI0_FCR_RTRG_Pos) |
				   (ttrg << R_SCI0_FCR_TTRG_Pos) | R_SCI0_FCR_RSTRG_Msk;
	}

	// TXD held at mark level while TE is cleared
	reg->SPTR = R_SCI0_SPTR_SPB2IO_Msk | R_SCI0_SPTR_SPB2DT_Msk;
	reg->SIMR1 = 0;
	reg->SPMR = 0;
	reg->SNFR = 0;
	reg->SCMR = RA4M1_SCI_SCMR_ASYNC;

	uint8_t smr = 0;
	if (pCfg->DataBits == 7)
	{
		smr |= R_SCI0_SMR_CHR_Msk;
	}
	if (pCfg->StopBits == 2)
	{
		smr |= R_SCI0_SMR_STOP_Msk;
	}
	if (pCfg->Parity != UART_PARITY_NONE)
	{
		smr |= R_SCI0_SMR_PE_Msk;
		if (pCfg->Parity == UART_PARITY_ODD)
		{
			smr |= R_SCI0_SMR_PM_Msk;
		}
	}
	reg->SMR = smr;

	pDev->Rate = (int)Ra4m1UartSetRate(&pDev->DevIntrf, (uint32_t)pCfg->Rate);
	if (pDev->Rate == 0)
	{
		Ra4m1UartModuleStart(dev, false);
		EnableInterrupt(state);

		return false;
	}

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->Mode = pCfg->Mode;
	pDev->Duplex = pCfg->Duplex;
	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->Parity = pCfg->Parity;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->LineState = 0;
	pDev->bRxReady = false;
	pDev->bTxReady = true;
	pDev->RxOvrErrCnt = 0;
	pDev->ParErrCnt = 0;
	pDev->FramErrCnt = 0;
	pDev->RxDropCnt = 0;
	pDev->TxDropCnt = 0;
	pDev->DevIntrf.IntPrio = pCfg->IntPrio;
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
	pDev->DevIntrf.bDma = pCfg->bDMAMode;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = NULL;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.Reset = NULL;
	pDev->DevIntrf.GetHandle = Ra4m1UartGetHandle;
	pDev->DevIntrf.Disable = Ra4m1UartDisable;
	pDev->DevIntrf.Enable = Ra4m1UartEnable;
	pDev->DevIntrf.GetRate = Ra4m1UartGetRate;
	pDev->DevIntrf.SetRate = Ra4m1UartSetRate;
	pDev->DevIntrf.StartRx = Ra4m1UartStartRx;
	pDev->DevIntrf.RxData = Ra4m1UartRxData;
	pDev->DevIntrf.StopRx = Ra4m1UartStopRx;
	pDev->DevIntrf.StartTx = Ra4m1UartStartTx;
	pDev->DevIntrf.TxData = Ra4m1UartTxData;
	pDev->DevIntrf.StopTx = Ra4m1UartStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = Ra4m1UartPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	if (pCfg->bIntMode)
	{
		// One handler for RXI, TXI and ERI, TEI is not used. With DMA the TXI
		// goes to the DMAC channel, whose transfer end has its own handler.
		dev->IrqRxi = Ra4m1RegisterIntHandler(dev->EvtRxi, pCfg->IntPrio, Ra4m1UartIRQHandler, dev);
		dev->IrqEri = Ra4m1RegisterIntHandler(dev->EvtRxi + 3, pCfg->IntPrio, Ra4m1UartIRQHandler, dev);
		IRQn_Type irqtx;
		if (pCfg->bDMAMode)
		{
			irqtx = dev->IrqDma = Ra4m1RegisterIntHandler(dev->EvtDma, pCfg->IntPrio, Ra4m1UartDmaIRQHandler, dev);
		}
		else
		{
			irqtx = dev->IrqTxi = Ra4m1RegisterIntHandler(dev->EvtRxi + 1, pCfg->IntPrio, Ra4m1UartIRQHandler, dev);
		}
		if (dev->IrqRxi == (IRQn_Type)-1 || irqtx == (IRQn_Type)-1 || dev->IrqEri == (IRQn_Type)-1)
		{
			Ra4m1UartReleaseIRQs(dev);
			IOPinDis(dev->pIOPinMap, dev->NbIOPins);
			Ra4m1UartModuleStart(dev, false);
			EnableInterrupt(state);

			return false;
		}
	}

	Ra4m1UartEnable(&pDev->DevIntrf);

	EnableInterrupt(state);

	return true;
}
