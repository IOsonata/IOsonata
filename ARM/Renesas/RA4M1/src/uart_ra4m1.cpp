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
		bDMAMode       : not implemented, ignored

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
// TXI when the TX FIFO holds TX_TRIGGER characters or less.
#define RA4M1_SCI_RX_TRIGGER		8
#define RA4M1_SCI_TX_TRIGGER		8

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
	pDev->IrqRxi = pDev->IrqTxi = pDev->IrqEri = (IRQn_Type)-1;
}

// Received characters into the RX CFIFO. The hardware is read first, a full
// CFIFO drops the character (RxDropCnt). A character with a framing or parity
// error is dropped. Returns the number of characters read from the hardware,
// pErr the line state errors seen. Interrupts disabled or from the interrupt.
//
// bRxi : called for the RXI of a SCI without FIFO, RDR holds a character. In
// all other cases RDR is read only with RDRF or an error flag set.
static int Ra4m1UartRxHw(Ra4m1UartDev_t * const pDev, bool bRxi, uint8_t *pErr)
{
	UARTDev_t *udev = pDev->pUartDev;
	R_SCI0_Type *reg = pDev->pReg;
	int cnt = 0;
	uint8_t err = 0;

	if (pDev->bHwFifo)
	{
		uint8_t status = reg->SSR_FIFO;
		int n = (reg->FDR & R_SCI0_FDR_R_Msk) >> R_SCI0_FDR_R_Pos;

		// Each FIFO entry has its own framing and parity status
		for (; cnt < n; cnt++)
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

		// Overrun loses the arriving character, the FIFO content is kept
		if (status & R_SCI0_SSR_FIFO_ORER_Msk)
		{
			udev->RxOvrErrCnt++;
			err |= UART_LINESTATE_OVR;
		}

		// Flags cleared by writing 0 to them only. RDF, ORER, FER and PER
		// once the FIFO is read, DR only once it is empty.
		uint8_t clr = status & (R_SCI0_SSR_FIFO_RDF_Msk | R_SCI0_SSR_FIFO_ORER_Msk |
								R_SCI0_SSR_FIFO_FER_Msk | R_SCI0_SSR_FIFO_PER_Msk);
		if ((reg->FDR & R_SCI0_FDR_R_Msk) == 0)
		{
			clr |= status & R_SCI0_SSR_FIFO_DR_Msk;
		}
		if (clr)
		{
			reg->SSR_FIFO = (uint8_t)~clr;
		}
	}
	else
	{
		uint8_t status = reg->SSR;

		if (status & RA4M1_SCI_SSR_RXERR)
		{
			// RDR holds the character received with the error. After an
			// overrun alone it holds the last good character, the arriving
			// one is lost.
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
		else if (bRxi || (status & R_SCI0_SSR_RDRF_Msk))
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

// Without the interrupt, RxData and TxData move what the hardware holds, in
// both directions. Interrupts disabled.
static void Ra4m1UartPoll(Ra4m1UartDev_t * const pDev)
{
	uint8_t err;

	Ra4m1UartRxHw(pDev, false, &err);
	Ra4m1UartTxHw(pDev);
}

// RXI, TXI and ERI of one SCI. RX is serviced first whatever the event.
static void Ra4m1UartIRQHandler(int IntNo, void *pCtx)
{
	Ra4m1UartDev_t *dev = (Ra4m1UartDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;
	R_SCI0_Type *reg = dev->pReg;
	uint8_t err = 0;
	bool txevt = false;

	int rxcnt = Ra4m1UartRxHw(dev, IntNo == (int)dev->IrqRxi, &err);

	if (reg->SCR & R_SCI0_SCR_TIE_Msk)
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

	// The SCI request is negated, its ICU IR flag can be cleared
	Ra4m1UartSync(dev);
	if (err && !dev->bHwFifo)
	{
		// RXI and ERI may both be pending for the character just read
		Ra4m1AcknowledgeInt(dev->IrqRxi);
		Ra4m1AcknowledgeInt(dev->IrqEri);
	}
	Ra4m1AcknowledgeInt((IRQn_Type)IntNo);

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
		// TxData or RxData call sends the rest
		uint32_t state = DisableInterrupt();
		Ra4m1UartPoll(dev);
		EnableInterrupt(state);
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

	dev->pReg->SCR = 0;
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
	// parity none, odd or even
	if (pCfg->Mode != UART_MODE_UART || pCfg->Duplex != UART_DUPLEX_FULL || pCfg->bIrDAMode ||
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

	// Interrupts of a previous UARTInit
	Ra4m1UartReleaseIRQs(dev);

	// Module clock first, register writes before it are ignored
	Ra4m1UartModuleStart(dev, true);
	reg->SCR = 0;

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
		reg->FCR = R_SCI0_FCR_FM_Msk | R_SCI0_FCR_RFRST_Msk | R_SCI0_FCR_TFRST_Msk |
				   (RA4M1_SCI_RX_TRIGGER << R_SCI0_FCR_RTRG_Pos) |
				   (RA4M1_SCI_TX_TRIGGER << R_SCI0_FCR_TTRG_Pos) | R_SCI0_FCR_RSTRG_Msk;
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
	pDev->DevIntrf.bDma = false;
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
		// One handler for RXI, TXI and ERI. TEI is not used.
		dev->IrqRxi = Ra4m1RegisterIntHandler(dev->EvtRxi, pCfg->IntPrio, Ra4m1UartIRQHandler, dev);
		dev->IrqTxi = Ra4m1RegisterIntHandler(dev->EvtRxi + 1, pCfg->IntPrio, Ra4m1UartIRQHandler, dev);
		dev->IrqEri = Ra4m1RegisterIntHandler(dev->EvtRxi + 3, pCfg->IntPrio, Ra4m1UartIRQHandler, dev);
		if (dev->IrqRxi == (IRQn_Type)-1 || dev->IrqTxi == (IRQn_Type)-1 || dev->IrqEri == (IRQn_Type)-1)
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
