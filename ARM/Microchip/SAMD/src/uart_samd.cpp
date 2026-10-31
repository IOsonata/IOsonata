/**-------------------------------------------------------------------------
@file	uart_samd.cpp

@brief	SAM D11/D21 SERCOM UART implementation

		DevNo is the SERCOM number: 0 to 2 on SAM D11, 0 to 5 on SAM D21.

		The pad routing comes from the pin map. Each pin is a SERCOM pad
		with PinOp IOPINOP_PERIPHC (SERCOM) or IOPINOP_PERIPHD (SERCOM-ALT):
		TX on pad 0 or 2, RX on any other pad. With hardware flow control TX
		is on pad 0, RTS on pad 2, CTS on pad 3 and RX on pad 1. A pin that
		is not a pad of the SERCOM fails UARTInit.

		5 to 8 data bits, no, even or odd parity, 1 or 2 stop bits. The
		SERCOM runs on the main clock, the baud rate is the closest of the
		arithmetic and fractional generators with 16x, 8x or 3x oversampling.

		TxData puts the data in the TX CFIFO, then starts TX if it is ready.
		RxData reads from the RX CFIFO. Without the interrupt, TxData and
		RxData both move what the hardware holds, in both directions.

		bIntMode false : polling, TxData and RxData move the CFIFO data to and
		                 from the DATA register
		bIntMode true  : SERCOM interrupts drain the TX CFIFO and fill the
		                 RX CFIFO
		bDMAMode true  : TX by a DMAC channel on the SERCOM TX trigger (DRE),
		                 taken at UARTInit (UARTInit fails without a free
		                 channel, and with SAMD_DMAC_NONE in the library
		                 project), given back at PowerOff: TxData returns 0
		                 until UARTInit. The CPU writes what DATA takes when the
		                 channel is idle, the channel sends the rest of the
		                 TX CFIFO in blocks of at most half its size: in
		                 place with a blocking CFIFO, from a copy of
		                 SAMD_UART_DMA_COPY bytes with a non-blocking one. A
		                 block is released when the channel has written its
		                 last byte to DATA. RX as without DMA: the USART has
		                 no idle line event to move a DMA ring after a short
		                 burst. With bIntMode the DMAC interrupt ends the
		                 blocks and sends UART_EVT_TXREADY when the TX CFIFO
		                 is empty; without it TxData and RxData do.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "samdxxx.h"

#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "coredev/shared_intrf.h"
#include "clock_samd.h"
#include "sercom_samd.h"
#include "dmac_samd.h"

#define SAMD_UART_CFIFO_SIZE		16
#define SAMD_UART_CFIFO_MEMSIZE		CFIFO_MEMSIZE(SAMD_UART_CFIFO_SIZE)

// Most bytes copied to or from a CFIFO with interrupts disabled. The RX
// interrupt reads DATA within two character times, 960 cycles at 1 Mbaud
// on 48 MHz.
#define SAMD_UART_COPY_MAX			8

#define SAMD_UART_SYNC_POLL_LIMIT	100000UL

// TX DMA block of a non-blocking CFIFO, copied out of the FIFO: a put on a
// full non-blocking CFIFO writes over its oldest bytes
#if SAMD_SERCOM_CNT > 3
#define SAMD_UART_DMA_COPY			16
#else
#define SAMD_UART_DMA_COPY			8
#endif

#define SAMD_USART_RX_ERRORS		(SERCOM_USART_INT_STATUS_PERR_Msk | SERCOM_USART_INT_STATUS_FERR_Msk | \
									 SERCOM_USART_INT_STATUS_BUFOVF_Msk)

// SAMPR sample rate selections
#define SAMD_USART_SAMPR_16X		0U
#define SAMD_USART_SAMPR_16X_FRAC	1U
#define SAMD_USART_SAMPR_8X			2U
#define SAMD_USART_SAMPR_8X_FRAC	3U
#define SAMD_USART_SAMPR_3X			4U

// TXPO pad selections
#define SAMD_USART_TXPO_PAD0		0U		// TX pad 0
#define SAMD_USART_TXPO_PAD2		1U		// TX pad 2
#define SAMD_USART_TXPO_FLOW		2U		// TX pad 0, RTS pad 2, CTS pad 3

#pragma pack(push, 4)

typedef struct {
	int DevNo;							//!< SERCOM number
	sercom_usart_int_registers_t *pReg;
	UARTDev_t *pUartDev;				//!< Generic UART device data
	const IOPinCfg_t *pIOPinMap;		//!< Pins in use
	int NbIOPins;
	uint8_t RxDataMask;					//!< Data bits of a character
	int8_t DmaChan;						//!< TX DMAC channel, -1 without DMA
	volatile bool bDmaActive;			//!< TX block running
	uint16_t DmaLen;					//!< TX CFIFO bytes of the running block, 0 for a copy
	uint16_t DmaMax;					//!< Largest block, half the TX CFIFO
	alignas(4) uint8_t RxFifoMem[SAMD_UART_CFIFO_MEMSIZE];
	alignas(4) uint8_t TxFifoMem[SAMD_UART_CFIFO_MEMSIZE];
	uint8_t TxDmaCopy[SAMD_UART_DMA_COPY];	//!< Block of a non-blocking TX CFIFO
} SamdUartDev_t;

#pragma pack(pop)

// TX by DMA, false with SAMD_DMAC_NONE: the DMA code is left out
static inline __attribute__((always_inline)) bool SamdUartDmaOn(const SamdUartDev_t * const pDev)
{
	return SAMD_DMAC_ON && pDev->DmaChan >= 0;
}

static SamdUartDev_t s_SamdUartDev[SAMD_SERCOM_CNT] = {
	{ .DevNo = 0, .pReg = &SERCOM0_REGS->USART_INT, .DmaChan = -1, },
	{ .DevNo = 1, .pReg = &SERCOM1_REGS->USART_INT, .DmaChan = -1, },
	{ .DevNo = 2, .pReg = &SERCOM2_REGS->USART_INT, .DmaChan = -1, },
#if SAMD_SERCOM_CNT > 3
	{ .DevNo = 3, .pReg = &SERCOM3_REGS->USART_INT, .DmaChan = -1, },
	{ .DevNo = 4, .pReg = &SERCOM4_REGS->USART_INT, .DmaChan = -1, },
	{ .DevNo = 5, .pReg = &SERCOM5_REGS->USART_INT, .DmaChan = -1, },
#endif
};

static bool SamdUartSync(SamdUartDev_t * const pDev, uint32_t Mask)
{
	uint32_t timeout = SAMD_UART_SYNC_POLL_LIMIT;

	while ((pDev->pReg->SERCOM_SYNCBUSY & Mask) != 0U)
	{
		if (--timeout == 0U)
		{
			return false;
		}
	}

	return true;
}

// USART enabled or disabled. CTRLA, BAUD and most of CTRLB change only while
// disabled.
static bool SamdUartRun(SamdUartDev_t * const pDev, bool bRun)
{
	if (bRun)
	{
		pDev->pReg->SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;
	}
	else
	{
		pDev->pReg->SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE_Msk;
	}

	return SamdUartSync(pDev, SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk);
}

// DATA characters go in the RX CFIFO while there is one. STATUS belongs to
// the character at the head of DATA: a parity or framing error drops it,
// an overflow means a character before it was lost. Interrupts disabled.
static void SamdUartPollRxRead(SamdUartDev_t * const pDev)
{
	sercom_usart_int_registers_t *reg = pDev->pReg;
	UARTDev_t *dev = pDev->pUartDev;

	while (true)
	{
		uint16_t status = reg->SERCOM_STATUS & SAMD_USART_RX_ERRORS;
		uint8_t flags = reg->SERCOM_INTFLAG;

		if (status != 0U)
		{
			if (status & SERCOM_USART_INT_STATUS_BUFOVF_Msk)
			{
				dev->RxOvrErrCnt++;
			}
			if (status & SERCOM_USART_INT_STATUS_PERR_Msk)
			{
				dev->ParErrCnt++;
			}
			if (status & SERCOM_USART_INT_STATUS_FERR_Msk)
			{
				dev->FramErrCnt++;
			}
			reg->SERCOM_STATUS = status;
			reg->SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_ERROR_Msk;

			if (status & (SERCOM_USART_INT_STATUS_PERR_Msk | SERCOM_USART_INT_STATUS_FERR_Msk))
			{
				if (flags & SERCOM_USART_INT_INTFLAG_RXC_Msk)
				{
					(void)reg->SERCOM_DATA;
				}
				continue;
			}
		}

		if ((flags & SERCOM_USART_INT_INTFLAG_RXC_Msk) == 0U)
		{
			break;
		}

		// A full FIFO drops the character
		uint8_t c = (uint8_t)reg->SERCOM_DATA & pDev->RxDataMask;
		if (CFifoPutByte(dev->hRxFifo, c))
		{
			dev->bRxReady = true;
		}
		else
		{
			dev->RxDropCnt++;
		}
	}
}

// SamdUartPollRxRead when a character or an error is there. Interrupts
// disabled.
static inline __attribute__((always_inline)) void SamdUartPollRx(SamdUartDev_t * const pDev)
{
	if (pDev->pReg->SERCOM_INTFLAG & (SERCOM_USART_INT_INTFLAG_RXC_Msk | SERCOM_USART_INT_INTFLAG_ERROR_Msk))
	{
		SamdUartPollRxRead(pDev);
	}
}

// TX CFIFO bytes to DATA while it is empty. Interrupts disabled. True when
// the TX CFIFO was found empty.
static inline __attribute__((always_inline)) bool SamdUartPollTx(SamdUartDev_t * const pDev)
{
	sercom_usart_int_registers_t *reg = pDev->pReg;

	while (reg->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk)
	{
		int c = CFifoGetByte(pDev->pUartDev->hTxFifo);
		if (c < 0)
		{
			return true;
		}
		reg->SERCOM_DATA = (uint16_t)c;
	}

	return false;
}

// TX DMA block from the TX CFIFO: the first contiguous run, at most DmaMax
// bytes, in place with a blocking CFIFO (released when the block is done);
// with a non-blocking one, up to SAMD_UART_DMA_COPY bytes taken out of the
// FIFO and copied, in pieces of at most SAMD_UART_COPY_MAX bytes. Interrupts
// disabled. False when the FIFO is empty.
static bool SamdUartDmaTxStart(SamdUartDev_t * const pDev)
{
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;
	uint8_t *p;
	int len;

	if (CFifoIsBlocking(fifo))
	{
		len = pDev->DmaMax;
		p = CFifoPeekMultiple(fifo, &len);
		if (p == NULL)
		{
			return false;
		}
		pDev->DmaLen = (uint16_t)len;
	}
	else
	{
		len = 0;
		while (len < SAMD_UART_DMA_COPY)
		{
			int l = SAMD_UART_DMA_COPY - len < SAMD_UART_COPY_MAX ? SAMD_UART_DMA_COPY - len : SAMD_UART_COPY_MAX;
			uint8_t *s = CFifoGetMultiple(fifo, &l);

			if (s == NULL)
			{
				break;
			}
			memcpy(pDev->TxDmaCopy + len, s, l);
			len += l;
		}
		if (len == 0)
		{
			return false;
		}
		p = pDev->TxDmaCopy;
		pDev->DmaLen = 0;
	}

	pDev->bDmaActive = true;
	SamdDmacStart(pDev->DmaChan, p, true, &pDev->pReg->SERCOM_DATA, false, (uint32_t)len);

	return true;
}

// The CPU writes what DATA takes, the channel takes the rest. Interrupts
// disabled, channel idle. True when a block runs.
static bool SamdUartDmaTxKick(SamdUartDev_t * const pDev)
{
	return !SamdUartPollTx(pDev) && SamdUartDmaTxStart(pDev);
}

// End of the running block: bytes released, unsent ones of a transfer error
// counted as dropped, then the next block. Interrupts disabled. True when a
// block still runs. Out of line: the TX paths without DMA stay small.
__attribute__((noinline)) static bool SamdUartDmaService(SamdUartDev_t * const pDev, bool bError)
{
	UARTDev_t *udev = pDev->pUartDev;

	if (pDev->bDmaActive)
	{
		if (!bError && SamdDmacBusy(pDev->DmaChan))
		{
			return true;
		}

		udev->TxDropCnt += (int)SamdDmacStop(pDev->DmaChan);
		CFifoRelease(udev->hTxFifo, pDev->DmaLen);
		pDev->DmaLen = 0;
		pDev->bDmaActive = false;
	}

	return SamdUartDmaTxKick(pDev);
}

// DMAC interrupt of the TX channel, interrupt mode
static void SamdUartDmaHandler(int Chan, uint32_t Flags, void *pCtx)
{
	SamdUartDev_t *dev = (SamdUartDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;

	(void)Chan;

	if (udev == NULL || !dev->bDmaActive)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	bool run = SamdUartDmaService(dev, (Flags & DMAC_CHINTFLAG_TERR_Msk) != 0U);

	if (!run)
	{
		udev->bTxReady = true;
	}
	EnableInterrupt(state);

	if (!run && udev->EvtCallback)
	{
		// Length is the free space in the TX FIFO
		udev->EvtCallback(udev, UART_EVT_TXREADY, NULL, CFifoAvail(udev->hTxFifo));
	}
}

// Channel stopped, the bytes of its block not released: Enable flushes the
// TX CFIFO
static void SamdUartDmaStop(SamdUartDev_t * const pDev)
{
	if (SamdUartDmaOn(pDev))
	{
		(void)SamdDmacStop(pDev->DmaChan);
	}
	pDev->bDmaActive = false;
	pDev->DmaLen = 0;
}

// TX channel given back
static void SamdUartDmaClose(SamdUartDev_t * const pDev)
{
	if (SamdUartDmaOn(pDev))
	{
		SamdDmacClose(pDev->DmaChan);
	}
	pDev->DmaChan = -1;
}

static void SamdUartIrqHandler(int DevNo, DevIntrf_t * const pDev)
{
	SamdUartDev_t *dev = (SamdUartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	sercom_usart_int_registers_t *reg = dev->pReg;

	(void)DevNo;

	// The SERCOM may be in polling mode
	if (udev == NULL || !pDev->bIntEn)
	{
		return;
	}

	uint8_t flags = reg->SERCOM_INTFLAG & reg->SERCOM_INTENSET;

	if (flags & (SERCOM_USART_INT_INTFLAG_RXC_Msk | SERCOM_USART_INT_INTFLAG_ERROR_Msk))
	{
		SamdUartPollRxRead(dev);
		if (udev->EvtCallback)
		{
			udev->EvtCallback(udev, UART_EVT_RXDATA, NULL, CFifoUsed(udev->hRxFifo));
		}
	}

	if (flags & SERCOM_USART_INT_INTFLAG_DRE_Msk)
	{
		int c = CFifoGetByte(udev->hTxFifo);

		if (c >= 0)
		{
			// Writing DATA clears DRE
			reg->SERCOM_DATA = (uint16_t)c;
		}
		else
		{
			// FIFO empty, DRE disabled so that it does not keep interrupting
			reg->SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_DRE_Msk;
			udev->bTxReady = true;
			if (udev->EvtCallback)
			{
				// Length is the free space in the TX FIFO
				udev->EvtCallback(udev, UART_EVT_TXREADY, NULL, CFifoAvail(udev->hTxFifo));
			}
		}
	}
}

static uint32_t SamdUartGetRate(DevIntrf_t * const pDev)
{
	return (uint32_t)((SamdUartDev_t *)pDev->pDevData)->pUartDev->Rate;
}

// Closest rate of the arithmetic and fractional generators with 16x, 8x
// and 3x oversampling. 16x is kept for equal rates, it has the best noise
// margin.
static uint32_t SamdUartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	static const uint8_t sampr[] = {
		SAMD_USART_SAMPR_16X, SAMD_USART_SAMPR_16X_FRAC, SAMD_USART_SAMPR_8X,
		SAMD_USART_SAMPR_8X_FRAC, SAMD_USART_SAMPR_3X
	};
	static const uint8_t over[] = { 16, 16, 8, 8, 3 };
	SamdUartDev_t *dev = (SamdUartDev_t *)pDev->pDevData;
	sercom_usart_int_registers_t *reg = dev->pReg;
	uint32_t fref = SystemPeriphClockGet(SAMD_GCLK_MAIN);
	uint32_t best = 0;
	uint32_t besterr = UINT32_MAX;
	uint32_t bestbaud = 0;
	uint32_t bestsampr = 0;

	if (Rate == 0 || fref == 0)
	{
		return 0;
	}

	for (unsigned i = 0; i < sizeof(sampr); i++)
	{
		uint64_t s = over[i];
		uint32_t baud;
		uint32_t rate;

		if (sampr[i] & 1U)
		{
			// Fractional: fref / (S x (BAUD + FP / 8)), BAUD 1 to 8191
			uint64_t div8 = ((uint64_t)fref * 8U + (s * Rate) / 2U) / (s * Rate);

			if (div8 < 8U)
			{
				div8 = 8U;
			}
			else if (div8 > 8191U * 8U + 7U)
			{
				div8 = 8191U * 8U + 7U;
			}
			baud = (uint32_t)((div8 >> 3) | ((div8 & 7U) << 13));
			rate = (uint32_t)(((uint64_t)fref * 8U + s * div8 / 2U) / (s * div8));
		}
		else
		{
			// Arithmetic: fref x (65536 - BAUD) / (65536 x S)
			uint64_t n = ((uint64_t)65536U * s * Rate + fref / 2U) / fref;

			if (n > 65536U)
			{
				n = 65536U;
			}
			else if (n == 0U)
			{
				n = 1U;
			}
			baud = (uint32_t)(65536U - n);
			rate = (uint32_t)(((uint64_t)fref * n + 32768U * s) / (65536U * s));
		}

		uint32_t err = rate > Rate ? rate - Rate : Rate - rate;
		if (err < besterr)
		{
			besterr = err;
			best = rate;
			bestbaud = baud;
			bestsampr = sampr[i];
		}
	}

	// CTRLA and BAUD change with the USART disabled. The interrupt handler
	// does not write them.
	uint32_t state = DisableInterrupt();
	bool run = (reg->SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_ENABLE_Msk) != 0U;

	if (run)
	{
		(void)SamdUartRun(dev, false);
	}

	reg->SERCOM_CTRLA = (reg->SERCOM_CTRLA & ~SERCOM_USART_INT_CTRLA_SAMPR_Msk) |
						SERCOM_USART_INT_CTRLA_SAMPR(bestsampr);
	reg->SERCOM_BAUD = (uint16_t)bestbaud;

	if (run)
	{
		(void)SamdUartRun(dev, true);
	}

	EnableInterrupt(state);

	dev->pUartDev->Rate = (int)best;

	return best;
}

static bool SamdUartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	(void)DevAddr;

	return true;
}

// Without the interrupt, DATA goes in the RX CFIFO and the TX CFIFO bytes to
// DATA, before and after the copy, so that both directions keep going while
// the application only reads. Interrupts disabled. Inline in the one byte
// TxData path, where at 1 Mbaud the call cost about 25 of the 480 cycles of
// a character at 48 MHz; SamdUartPoll for the other callers.
static inline __attribute__((always_inline)) void SamdUartPollInline(SamdUartDev_t * const pDev)
{
	// DATA checked again after the TX writes, which can take two
	// characters. With DMA the channel sends the TX CFIFO bytes DATA does
	// not take.
	SamdUartPollRx(pDev);
	if (!SamdUartDmaOn(pDev))
	{
		(void)SamdUartPollTx(pDev);
	}
	else
	{
		(void)SamdUartDmaService(pDev, false);
	}
	SamdUartPollRx(pDev);
}

static void SamdUartPoll(SamdUartDev_t * const pDev)
{
	SamdUartPollInline(pDev);
}

static int SamdUartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	SamdUartDev_t *dev = (SamdUartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	hCFifo_t fifo = udev->hRxFifo;
	bool poll = !pDev->bIntEn;
	int cnt = 0;

	if (pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

	if (poll)
	{
		uint32_t state = DisableInterrupt();
		SamdUartPoll(dev);
		EnableInterrupt(state);
	}

	// The data comes from the RX CFIFO. CFifoGetMultiple frees the slots
	// before they are copied, interrupts stay disabled until the copy is
	// done, in pieces of at most SAMD_UART_COPY_MAX bytes.
	while (cnt < Bufflen)
	{
		int l = Bufflen - cnt < SAMD_UART_COPY_MAX ? Bufflen - cnt : SAMD_UART_COPY_MAX;
		uint32_t state = DisableInterrupt();

		if (poll)
		{
			SamdUartPoll(dev);
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
		SamdUartPoll(dev);
		EnableInterrupt(state);
	}

	return cnt;
}

static void SamdUartStopRx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static bool SamdUartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	(void)DevAddr;

	return true;
}

// Several bytes into the TX CFIFO, in pieces of at most SAMD_UART_COPY_MAX
// bytes with interrupts disabled. In polling mode DATA is serviced before
// each piece. Returns the number of bytes put.
__attribute__((noinline)) static int SamdUartTxPut(SamdUartDev_t * const pDev, uint8_t const *pData, int Datalen)
{
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;
	bool poll = !pDev->pUartDev->DevIntrf.bIntEn;
	int cnt = 0;

	while (Datalen > 0)
	{
		int l = Datalen < SAMD_UART_COPY_MAX ? Datalen : SAMD_UART_COPY_MAX;
		uint32_t state = DisableInterrupt();

		if (poll)
		{
			SamdUartPoll(pDev);
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

static int SamdUartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	SamdUartDev_t *dev = (SamdUartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	hCFifo_t fifo = udev->hTxFifo;
	int cnt;

	// bDMAMode without the channel: given back by PowerOff
	if (pData == NULL || Datalen <= 0 || (pDev->bDma && !SamdUartDmaOn(dev)))
	{
		return 0;
	}

	// The data goes in the TX CFIFO first
	if (Datalen == 1)
	{
		cnt = CFifoPutByte(fifo, *pData) ? 1 : 0;
	}
	else
	{
		cnt = SamdUartTxPut(dev, pData, Datalen);
	}

	if (!pDev->bIntEn)
	{
		// Without the interrupt TX stays ready: what DATA takes now is sent,
		// the next TxData or RxData call sends the rest
		uint32_t state = DisableInterrupt();

		SamdUartPollInline(dev);

		EnableInterrupt(state);
	}
	else if (udev->bTxReady)
	{
		// TX starts if it is ready, otherwise the running interrupt takes
		// the bytes just put. Checked again with interrupts disabled, TxData
		// may be called from an interrupt.
		uint32_t state = DisableInterrupt();

		if (udev->bTxReady && SamdUartDmaOn(dev))
		{
			// DMA: the CPU fills DATA, a block for the rest, ended by the
			// DMAC interrupt
			if (SamdUartDmaService(dev, false))
			{
				udev->bTxReady = false;
			}
		}
		else if (udev->bTxReady)
		{
			// TX ready means DATA is empty. The first byte goes to DATA,
			// the DRE interrupt sends the rest.
			int c = CFifoGetByte(fifo);
			if (c >= 0)
			{
				udev->bTxReady = false;
				dev->pReg->SERCOM_DATA = (uint16_t)c;
				dev->pReg->SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_DRE_Msk;
			}
		}

		EnableInterrupt(state);
	}

	return cnt;
}

static void SamdUartStopTx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

// Registers keep their content while the clock is off, Enable restarts the
// UART without a new UARTInit.
static void SamdUartDisable(DevIntrf_t * const pDev)
{
	SamdUartDev_t *dev = (SamdUartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_Msk;
	SamdUartDmaStop(dev);
	(void)SamdUartRun(dev, false);
	SamdSercomClockDisable(dev->DevNo);

	IOPinDis(dev->pIOPinMap, dev->NbIOPins);

	EnableInterrupt(state);
}

// Disabled, the TX channel given back: a new UARTInit is needed
static void SamdUartPowerOff(DevIntrf_t * const pDev)
{
	SamdUartDev_t *dev = (SamdUartDev_t *)pDev->pDevData;

	SamdUartDisable(pDev);

	uint32_t state = DisableInterrupt();

	SamdUartDmaClose(dev);
	EnableInterrupt(state);
}

static void SamdUartEnable(DevIntrf_t * const pDev)
{
	SamdUartDev_t *dev = (SamdUartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	uint32_t state = DisableInterrupt();

	udev->RxOvrErrCnt = 0;
	udev->ParErrCnt = 0;
	udev->FramErrCnt = 0;
	udev->RxDropCnt = 0;
	udev->TxDropCnt = 0;

	SamdUartDmaStop(dev);
	CFifoFlush(udev->hTxFifo);
	udev->bTxReady = true;

	if (SamdSercomClockEnable(dev->DevNo))
	{
		IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

		if (pDev->bIntEn)
		{
			dev->pReg->SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_RXC_Msk | SERCOM_USART_INT_INTENSET_ERROR_Msk;
		}
		(void)SamdUartRun(dev, true);
	}

	EnableInterrupt(state);
}

// Software reset, all registers back to their reset value
static void SamdUartReset(DevIntrf_t * const pDev)
{
	SamdUartDev_t *dev = (SamdUartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	SamdUartDmaStop(dev);
	dev->pReg->SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
	(void)SamdUartSync(dev, SERCOM_USART_INT_SYNCBUSY_SWRST_Msk);

	EnableInterrupt(state);
}

static void *SamdUartGetHandle(DevIntrf_t * const pDev)
{
	return ((SamdUartDev_t *)pDev->pDevData)->pUartDev;
}

UARTDev_t const *UARTGetInstance(int DevNo)
{
	return DevNo >= 0 && DevNo < SAMD_SERCOM_CNT ? s_SamdUartDev[DevNo].pUartDev : NULL;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
	// RTS is driven by the SERCOM, there are no other modem lines
	pDev->LineState = LineState;
}

// CTRLA pad selection from the pin map, 0 for a pin map the SERCOM cannot
// take. TXPO and RXPO are returned in their CTRLA fields.
static uint32_t SamdUartPads(int DevNo, const IOPinCfg_t *pPins, int NbPins, bool bFlowCtrl)
{
	int rx;
	int tx;
	uint32_t txpo;

	if (NbPins < 2 || (bFlowCtrl && NbPins <= UARTPIN_RTS_IDX))
	{
		return 0;
	}

	rx = SamdSercomPad(DevNo, &pPins[UARTPIN_RX_IDX]);
	tx = SamdSercomPad(DevNo, &pPins[UARTPIN_TX_IDX]);

	if (rx < 0 || tx < 0 || rx == tx)
	{
		return 0;
	}

	if (bFlowCtrl)
	{
		if (tx != 0 || rx != 1 || SamdSercomPad(DevNo, &pPins[UARTPIN_RTS_IDX]) != 2 ||
			SamdSercomPad(DevNo, &pPins[UARTPIN_CTS_IDX]) != 3)
		{
			return 0;
		}
		txpo = SAMD_USART_TXPO_FLOW;
	}
	else if (tx == 0)
	{
		txpo = SAMD_USART_TXPO_PAD0;
	}
	else if (tx == 2)
	{
		txpo = SAMD_USART_TXPO_PAD2;
	}
	else
	{
		return 0;
	}

	return SERCOM_USART_INT_CTRLA_TXPO(txpo) | SERCOM_USART_INT_CTRLA_RXPO((uint32_t)rx);
}

bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->pIOPinMap == NULL || pCfg->NbIOPins <= 0)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= SAMD_SERCOM_CNT)
	{
		return false;
	}

	// Asynchronous UART, polling or interrupt, TX DMA. No IrDA or half
	// duplex.
	if (pCfg->bIrDAMode || pCfg->Mode != UART_MODE_UART ||
		pCfg->Duplex != UART_DUPLEX_FULL ||
		(pCfg->FlowControl != UART_FLWCTRL_NONE && pCfg->FlowControl != UART_FLWCTRL_HW) ||
		(pCfg->Parity != UART_PARITY_NONE && pCfg->Parity != UART_PARITY_EVEN &&
		 pCfg->Parity != UART_PARITY_ODD) ||
		pCfg->DataBits < 5 || pCfg->DataBits > 8 ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) || pCfg->Rate <= 0 ||
		(pCfg->bIntMode && pCfg->IntPrio < 0))
	{
		return false;
	}

	// Reject undersized caller buffers before changing the instance
	if ((pCfg->pRxMem != NULL && pCfg->RxMemSize > 0 && pCfg->RxMemSize < (int)CFIFO_MEMSIZE(1)) ||
		(pCfg->pTxMem != NULL && pCfg->TxMemSize > 0 && pCfg->TxMemSize < (int)CFIFO_MEMSIZE(1)))
	{
		return false;
	}

	const IOPinCfg_t *pins = (const IOPinCfg_t *)pCfg->pIOPinMap;
	bool flowctrl = pCfg->FlowControl == UART_FLWCTRL_HW;
	uint32_t pads = SamdUartPads(pCfg->DevNo, pins, pCfg->NbIOPins, flowctrl);

	if (pads == 0)
	{
		return false;
	}

	SamdUartDev_t *dev = &s_SamdUartDev[pCfg->DevNo];
	sercom_usart_int_registers_t *reg = dev->pReg;
	uint32_t state = DisableInterrupt();

	// Reject another running owner of the SERCOM: UART, I2C or SPI
	if (SamdSercomOwned(pCfg->DevNo, &pDev->DevIntrf))
	{
		EnableInterrupt(state);

		return false;
	}

	if (!SamdSercomClockEnable(pCfg->DevNo))
	{
		EnableInterrupt(state);

		return false;
	}

	// A channel of an earlier UARTInit given back, a new one taken. None with
	// SAMD_DMAC_NONE: the init fails.
	SamdUartDmaClose(dev);
	dev->bDmaActive = false;
	dev->DmaLen = 0;
	if (pCfg->bDMAMode)
	{
		if (SAMD_DMAC_ON)
		{
			dev->DmaChan = (int8_t)SamdDmacOpen(SAMD_DMAC_TRIG_SERCOM_TX(pCfg->DevNo),
												pCfg->bIntMode ? SamdUartDmaHandler : NULL, dev, pCfg->IntPrio);
		}
		if (dev->DmaChan < 0)
		{
			SamdSercomClockDisable(pCfg->DevNo);
			EnableInterrupt(state);

			return false;
		}
	}

	// All registers at their reset value, USART disabled
	NVIC_DisableIRQ(SamdSercomIrq(pCfg->DevNo));
	reg->SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
	if (!SamdUartSync(dev, SERCOM_USART_INT_SYNCBUSY_SWRST_Msk))
	{
		SamdUartDmaClose(dev);
		SamdSercomClockDisable(pCfg->DevNo);
		EnableInterrupt(state);

		return false;
	}

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(dev->RxFifoMem, SAMD_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(dev->TxFifoMem, SAMD_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	// CFifoInit returns NULL when the supplied memory is too small
	if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
	{
		SamdUartDmaClose(dev);
		SamdSercomClockDisable(pCfg->DevNo);
		EnableInterrupt(state);

		return false;
	}

	// Half the TX CFIFO per DMA block, the producer fills the other half
	int txsize = (CFifoUsed(pDev->hTxFifo) + CFifoAvail(pDev->hTxFifo)) / 2;

	dev->DmaMax = (uint16_t)(txsize < 1 ? 1 : txsize > SAMD_DMAC_COUNT_MAX ? SAMD_DMAC_COUNT_MAX : txsize);

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;
	dev->pIOPinMap = pins;
	dev->NbIOPins = flowctrl ? UARTPIN_RTS_IDX + 1 : UARTPIN_TX_IDX + 1;
	dev->RxDataMask = (uint8_t)((1U << pCfg->DataBits) - 1U);

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

	// Internal clock, asynchronous, LSB first
	uint32_t ctrla = SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK | SERCOM_USART_INT_CTRLA_DORD_Msk | pads;
	uint32_t ctrlb = SERCOM_USART_INT_CTRLB_TXEN_Msk | SERCOM_USART_INT_CTRLB_RXEN_Msk;

	if (pCfg->Parity != UART_PARITY_NONE)
	{
		ctrla |= SERCOM_USART_INT_CTRLA_FORM(1U);
		if (pCfg->Parity == UART_PARITY_ODD)
		{
			ctrlb |= SERCOM_USART_INT_CTRLB_PMODE_Msk;
		}
	}

	// CHSIZE 0 is 8 bits, 5 to 7 are 5 to 7 bits
	ctrlb |= SERCOM_USART_INT_CTRLB_CHSIZE(pCfg->DataBits == 8 ? 0U : (uint32_t)pCfg->DataBits);

	if (pCfg->StopBits == 2)
	{
		ctrlb |= SERCOM_USART_INT_CTRLB_SBMODE_Msk;
	}

	reg->SERCOM_CTRLA = ctrla;

	pDev->Rate = (int)SamdUartSetRate(&pDev->DevIntrf, (uint32_t)pCfg->Rate);

	reg->SERCOM_CTRLB = ctrlb;

	if (pDev->Rate == 0 || !SamdUartSync(dev, SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk))
	{
		SamdUartDmaClose(dev);
		dev->pUartDev = NULL;
		IOPinDis(dev->pIOPinMap, dev->NbIOPins);
		SamdSercomClockDisable(pCfg->DevNo);
		EnableInterrupt(state);

		return false;
	}

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
	pDev->bRxReady = false;
	pDev->bTxReady = true;
	pDev->RxOvrErrCnt = 0;
	pDev->ParErrCnt = 0;
	pDev->FramErrCnt = 0;
	pDev->RxDropCnt = 0;
	pDev->TxDropCnt = 0;
	pDev->DevIntrf.IntPrio = (int)SamdIrqPrio(pCfg->IntPrio);
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
	pDev->DevIntrf.bDma = pCfg->bDMAMode;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = NULL;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.GetHandle = SamdUartGetHandle;
	pDev->DevIntrf.Reset = SamdUartReset;
	pDev->DevIntrf.Disable = SamdUartDisable;
	pDev->DevIntrf.Enable = SamdUartEnable;
	pDev->DevIntrf.GetRate = SamdUartGetRate;
	pDev->DevIntrf.SetRate = SamdUartSetRate;
	pDev->DevIntrf.StartRx = SamdUartStartRx;
	pDev->DevIntrf.RxData = SamdUartRxData;
	pDev->DevIntrf.StopRx = SamdUartStopRx;
	pDev->DevIntrf.StartTx = SamdUartStartTx;
	pDev->DevIntrf.TxData = SamdUartTxData;
	pDev->DevIntrf.StopTx = SamdUartStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = SamdUartPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	SharedIntrfSetIrqHandler(pCfg->DevNo, &pDev->DevIntrf, SamdUartIrqHandler);

	if (pCfg->bIntMode)
	{
		// DRE is enabled by SamdUartTxData when the TX FIFO has data
		reg->SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_RXC_Msk | SERCOM_USART_INT_INTENSET_ERROR_Msk;

		NVIC_ClearPendingIRQ(SamdSercomIrq(pCfg->DevNo));
		NVIC_SetPriority(SamdSercomIrq(pCfg->DevNo), SamdIrqPrio(pCfg->IntPrio));
		NVIC_EnableIRQ(SamdSercomIrq(pCfg->DevNo));
	}

	bool res = SamdUartRun(dev, true);

	EnableInterrupt(state);

	return res;
}
