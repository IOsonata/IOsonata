/**-------------------------------------------------------------------------
@file	uarta_r9a02.cpp

@brief	R9A02G021 UART driver on the serial interface UARTA

R9A02G021 User's Manual R01UH1036EJ0110, chapter 25. Called by UARTInit for
DevNo 3 (UARTA0, TxDA0 P009, RxDA0 P010) and DevNo 4 (UARTA1, TxDA1 P403,
RxDA1 P008). The pins exist on the 48-pin package only.

Format: 5, 7 or 8 data bits, no, odd, even or space (0) parity, 1 or 2 stop
bits, LSB first, no flow control, no DMA. Unsupported settings fail
UARTInit.

Clock: the UARTA does not use PCLKB. fSEL is the HOCO when it runs, else the
EXTAL clock; the MOCO is not taken, its untrimmed accuracy is too low for a
UART (37.3). fSEL is common to both channels (UTA0CK.SEL): when the other
channel is in use, its fSEL is kept. Rate = fSEL / 2^CK / (2 x k), CK 0 to
6, k (BRGCAn) 2 to 255. The rate is limited to the 200 to 153600 bps of table
37.36, SetRate sets the closest rate and returns it. Of equal rates the one
with the larger k is taken, it has the wider receive margin (table 25.13).
UARTInit fails for a rate outside these limits or more than 2 % away from
the one that can be made, below about 1500 bps with the HOCO at 48 MHz.
UTA0CK is written with the channel stopped. The manual does not say whether
the other channel must be stopped too when UARTA0 changes its CK (25.2.8).

The UARTA has a one character buffer each way. Every received character,
with or without error, raises RX_ENDI (ASIMAn0.ISRMA = 1), its errors are
read from ASISAn. With interrupts, transmit uses the buffer empty interrupt
(ISSMA = 1) one level below IntPrio. Without interrupts RX_ENDI is routed to
an IELSR without a CPU interrupt and its IR flag tells RxData that a
character is there: the UARTA has no receive data flag.

Pins: PinOp R9A02_PSEL_UARTA (IOPINOP_FUNC5), RX input, TX output.

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

#include "r9a02g021.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "uarta_r9a02.h"

// Default CFIFO size when the configuration gives no memory
#ifndef R9A02_UARTA_CFIFO_SIZE
#define R9A02_UARTA_CFIFO_SIZE		16
#endif
#define R9A02_UARTA_CFIFO_MEMSIZE	CFIFO_MEMSIZE(R9A02_UARTA_CFIFO_SIZE)

// Most bytes TxData and RxData copy to or from a CFIFO with interrupts
// disabled, to keep the RX interrupt latency short
#define R9A02_UARTA_COPY_MAX		8

#define R9A02_UARTA_BRGCA_MAX		255U
#define R9A02_UARTA_RATE_TOL		50U			// Rate error at most 1/50 of the rate
#define R9A02_UARTA_FRAME_BITS		12U			// Longest character, start, 8 data, parity, 2 stop

#pragma pack(push, 4)
typedef struct __R9a02_Uarta_Dev {
	R9A02_UARTA_Unit_Type *pReg;
	volatile uint8_t *pUtaCk;		// UTA0CK or UTA1CK
	uint8_t UlbsMask;				// Loopback bit in ULBS
	uint8_t EvtTx;					// ICU events
	uint8_t EvtRx;
	uint16_t RxDataMask;			// 0x1F, 0x7F or 0xFF
	uint32_t FutaHz;				// Operation clock fUTAn
	UARTDev_t *pUartDev;			// Generic UART device data
	const IOPinCfg_t *pIOPinMap;	// RX, TX
	int IntTx;						// IELSR numbers, -1 when not routed
	int IntRx;						// Interrupt or polled route
	bool bInUse;					// Initialized and not powered off
} R9a02UartaDev_t;
#pragma pack(pop)

// Default CFIFO memory, RX then TX, of each channel
alignas(4) static uint8_t s_R9a02UartaFifoMem[R9A02_UART_UARTA_CNT][2][R9A02_UARTA_CFIFO_MEMSIZE];

// fSEL of UTA0CK.SEL, common to both channels, set by the first channel in
// use
static uint8_t s_R9a02UartaSel = R9A02_UTA0CK_SEL_HOCO;

static R9a02UartaDev_t s_R9a02UartaDev[R9A02_UART_UARTA_CNT] = {
	{ &R9A02_UARTA->UNIT[0], &R9A02_UARTA->UTA0CK, R9A02_ULBS_UARTA0, R9A02_EVT_UARTA_TX_ENDI0,
	  R9A02_EVT_UARTA_RX_ENDI0, 0xFF, 0, NULL, NULL, -1, -1, false },
	{ &R9A02_UARTA->UNIT[1], &R9A02_UARTA->UTA1CK, R9A02_ULBS_UARTA1, R9A02_EVT_UARTA_TX_ENDI1,
	  R9A02_EVT_UARTA_RX_ENDI1, 0xFF, 0, NULL, NULL, -1, -1, false },
};

static void R9a02UartaReleaseInts(R9a02UartaDev_t * const pDev)
{
	R9a02UnregisterIntHandler(pDev->IntTx);
	R9a02UnregisterIntHandler(pDev->IntRx);
	pDev->IntTx = -1;
	pDev->IntRx = -1;
}

// Wait n cycles of fUTAn
static void R9a02UartaWaitClk(R9a02UartaDev_t * const pDev, uint32_t Cycles)
{
	uint32_t hz = pDev->FutaHz ? pDev->FutaHz : R9A02_LOCO_FREQ;

	R9a02DelayUs((uint32_t)(((uint64_t)Cycles * 1000000ULL + hz - 1U) / hz));
}

// Transmit and receive off, EN kept. The characters in TXBAn and the shift
// register go out first, TXEA = 0 during a transmission makes it undefined
// (25.2.6). TXEA and RXEA can be set again 2 fUTAn cycles later (25.2.3).
static void R9a02UartaStop(R9a02UartaDev_t * const pDev)
{
	R9A02_UARTA_Unit_Type *reg = pDev->pReg;

	if (reg->ASIMA0 & R9A02_ASIMA0_TXEA)
	{
		uint32_t rate = pDev->pUartDev != NULL && pDev->pUartDev->Rate > 0 ?
						(uint32_t)pDev->pUartDev->Rate : R9A02_UARTA_RATE_MIN;
		uint32_t us = 2U * R9A02_UARTA_FRAME_BITS * 1000000U / rate + 10U;

		while ((reg->ASISA & (R9A02_ASISA_TXBFA | R9A02_ASISA_TXSFA)) && us-- > 0U)
		{
			R9a02DelayUs(1);
		}
	}

	reg->ASIMA0 = (uint8_t)(reg->ASIMA0 & ~(R9A02_ASIMA0_TXEA | R9A02_ASIMA0_RXEA));
	R9a02UartaWaitClk(pDev, 2);
}

// EN, then TXEA and RXEA in a second write (table 25.3). TXBAn is written 1
// fUTAn cycle after TXEA at the earliest.
static void R9a02UartaStart(R9a02UartaDev_t * const pDev)
{
	R9A02_UARTA_Unit_Type *reg = pDev->pReg;
	uint8_t asim0 = (uint8_t)(reg->ASIMA0 | R9A02_ASIMA0_EN);

	reg->ASIMA0 = asim0;
	reg->ASCTA = R9A02_ASISA_ERR_Msk;
	reg->ASIMA0 = (uint8_t)(asim0 | R9A02_ASIMA0_TXEA | R9A02_ASIMA0_RXEA);
	R9a02UartaWaitClk(pDev, 1);
	if (pDev->IntRx >= 0)
	{
		R9a02ClearPendingInt(pDev->IntRx);
	}
}

// The other channel is initialized, running or not: fSEL is kept for it
static bool R9a02UartaOtherInUse(R9a02UartaDev_t * const pDev)
{
	R9a02UartaDev_t *other = pDev == &s_R9a02UartaDev[0] ? &s_R9a02UartaDev[1] : &s_R9a02UartaDev[0];

	return other->bInUse;
}

// fSEL source of UTA0CK.SEL, 0 when it does not run
static uint32_t R9a02UartaSelHz(uint8_t Sel)
{
	switch (Sel & R9A02_UTA0CK_SEL_Msk)
	{
		case R9A02_UTA0CK_SEL_HOCO:
			return R9a02HocoClockGet();
		case R9A02_UTA0CK_SEL_EXTAL:
			return R9a02ExtalClockGet();
		case R9A02_UTA0CK_SEL_MOCO:
			return R9A02_MOCO_FREQ;
		default:
			return 0U;
	}
}

// One character received, with its error flags (25.4.4). The IR flag of the
// route is cleared by the caller before. After an overrun RXBAn holds the
// character before the lost one, it is kept. Returns the number of bytes put
// in the RX CFIFO.
static int R9a02UartaRxHw(R9a02UartaDev_t * const pDev, uint8_t *pErr)
{
	UARTDev_t *udev = pDev->pUartDev;
	R9A02_UARTA_Unit_Type *reg = pDev->pReg;
	uint8_t err = reg->ASISA & R9A02_ASISA_ERR_Msk;
	uint8_t d = reg->RXBA;

	if (err)
	{
		reg->ASCTA = err;
		if (err & R9A02_ASISA_OVEA)
		{
			udev->RxOvrErrCnt++;
			*pErr |= UART_LINESTATE_OVR;
		}
		if (err & R9A02_ASISA_PEA)
		{
			udev->ParErrCnt++;
			*pErr |= UART_LINESTATE_PARERR;
		}
		if (err & R9A02_ASISA_FEA)
		{
			udev->FramErrCnt++;
			*pErr |= UART_LINESTATE_FRMERR;
		}
		if (err & (R9A02_ASISA_PEA | R9A02_ASISA_FEA))
		{
			udev->RxDropCnt++;

			return 0;
		}
	}

	if (CFifoPutByte(udev->hRxFifo, (uint8_t)(d & pDev->RxDataMask)))
	{
		udev->bRxReady = true;

		return 1;
	}

	udev->RxDropCnt++;

	return 0;
}

// Next byte of the TX CFIFO into TXBAn. Returns 1 when a byte was written, 0
// when the CFIFO is empty, -1 when TXBAn is full. The buffer empty interrupt
// does not check TXBFA (25.2.6).
static int R9a02UartaTxHw(R9a02UartaDev_t * const pDev, bool bCheckFull)
{
	R9A02_UARTA_Unit_Type *reg = pDev->pReg;

	if (bCheckFull && (reg->ASISA & R9A02_ASISA_TXBFA))
	{
		return -1;
	}

	uint8_t *p = CFifoGet(pDev->pUartDev->hTxFifo);

	if (p == NULL)
	{
		return 0;
	}

	reg->TXBA = *p;

	return 1;
}

// Without interrupts, everything in the TX CFIFO is sent now. Each byte waits
// at most two character times for TXBAn (table 25.6).
static void R9a02UartaTxDrain(R9a02UartaDev_t * const pDev)
{
	R9A02_UARTA_Unit_Type *reg = pDev->pReg;
	UARTDev_t *udev = pDev->pUartDev;
	uint32_t rate = udev->Rate > 0 ? (uint32_t)udev->Rate : 1U;
	uint32_t timeout = (SystemCoreClock / rate + 1U) * 24U + 1000U;

	if ((reg->ASIMA0 & R9A02_ASIMA0_TXEA) == 0U)
	{
		return;
	}

	while (CFifoUsed(udev->hTxFifo) > 0)
	{
		uint32_t n = timeout;

		while ((reg->ASISA & R9A02_ASISA_TXBFA) && --n > 0U)
		{
		}
		if (n == 0U || R9a02UartaTxHw(pDev, true) <= 0)
		{
			break;
		}
	}
}

// Without interrupts: a character is there when the RX_ENDI route has its IR
// flag set
static void R9a02UartaRxPoll(R9a02UartaDev_t * const pDev)
{
	if (pDev->IntRx >= 0 && R9a02EventFlag(pDev->IntRx))
	{
		uint8_t err = 0;

		R9a02EventFlagClear(pDev->IntRx);
		R9a02UartaRxHw(pDev, &err);
	}
}

static void R9a02UartaRxIRQHandler(int IntNo, void *pCtx)
{
	R9a02UartaDev_t *dev = (R9a02UartaDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;
	uint8_t err = 0;
	int cnt = R9a02UartaRxHw(dev, &err);

	if (udev->EvtCallback)
	{
		if (err)
		{
			udev->EvtCallback(udev, UART_EVT_LINESTATE, &err, 1);
		}
		if (cnt > 0)
		{
			udev->EvtCallback(udev, UART_EVT_RXDATA, NULL, CFifoUsed(udev->hRxFifo));
		}
	}
}

// Transmit buffer empty interrupt: the byte in TXBAn went to the shift register
static void R9a02UartaTxIRQHandler(int IntNo, void *pCtx)
{
	R9a02UartaDev_t *dev = (R9a02UartaDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;
	uintptr_t state = DisableInterrupt();

	if (R9a02UartaTxHw(dev, false) == 0)
	{
		// TX CFIFO empty, TxData starts the next transmission
		udev->bTxReady = true;
	}

	EnableInterrupt(state);

	if (udev->EvtCallback)
	{
		udev->EvtCallback(udev, UART_EVT_TXREADY, NULL, CFifoAvail(udev->hTxFifo));
	}
}

static uint32_t R9a02UartaGetRate(DevIntrf_t * const pDev)
{
	return ((R9a02UartaDev_t *)pDev->pDevData)->pUartDev->Rate;
}

static uint32_t R9a02UartaSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	R9a02UartaDev_t *dev = (R9a02UartaDev_t *)pDev->pDevData;
	R9A02_UARTA_Unit_Type *reg = dev->pReg;

	if (Rate == 0U)
	{
		return 0;
	}
	if (Rate < R9A02_UARTA_RATE_MIN)
	{
		Rate = R9A02_UARTA_RATE_MIN;
	}
	if (Rate > R9A02_UARTA_RATE_MAX)
	{
		Rate = R9A02_UARTA_RATE_MAX;
	}

	uintptr_t state = DisableInterrupt();
	bool otheruse = R9a02UartaOtherInUse(dev);
	uint8_t sel = s_R9a02UartaSel;
	uint32_t fsel;

	if (otheruse)
	{
		// fSEL is common, it is kept
		fsel = R9a02UartaSelHz(sel);
	}
	else
	{
		fsel = R9a02HocoClockGet();
		sel = R9A02_UTA0CK_SEL_HOCO;
		if (fsel == 0U)
		{
			fsel = R9a02ExtalClockGet();
			sel = R9A02_UTA0CK_SEL_EXTAL;
		}
	}

	uint32_t best = 0;
	uint32_t besterr = UINT32_MAX;
	uint32_t bestck = 0;
	uint32_t bestk = 0;

	for (uint32_t ck = 0; fsel != 0U && ck <= R9A02_UTACK_CK_MAX; ck++)
	{
		uint32_t futa = fsel >> ck;
		uint32_t k = (futa + Rate) / (2U * Rate);

		if (k < R9A02_UARTA_BRGCA_MIN)
		{
			k = R9A02_UARTA_BRGCA_MIN;
		}
		if (k > R9A02_UARTA_BRGCA_MAX)
		{
			k = R9A02_UARTA_BRGCA_MAX;
		}

		uint32_t actual = (futa + k) / (2U * k);
		uint32_t err = actual > Rate ? actual - Rate : Rate - actual;

		// Strictly better only: of equal errors the smaller CK, larger k
		if (err < besterr)
		{
			besterr = err;
			best = actual;
			bestck = ck;
			bestk = k;
		}
	}

	if (best == 0U)
	{
		EnableInterrupt(state);

		return 0;
	}

	// UTAnCK and BRGCAn are written with TXEA and RXEA off (25.2.5, 25.2.8, 25.2.9)
	bool running = (reg->ASIMA0 & (R9A02_ASIMA0_TXEA | R9A02_ASIMA0_RXEA)) != 0U;

	if (running)
	{
		R9a02UartaStop(dev);
	}

	if (dev == &s_R9a02UartaDev[0])
	{
		R9A02_UARTA->UTA0CK = (uint8_t)(sel | bestck);
	}
	else
	{
		if (!otheruse)
		{
			R9A02_UARTA->UTA0CK = (uint8_t)((R9A02_UARTA->UTA0CK & R9A02_UTACK_CK_Msk) | sel);
		}
		R9A02_UARTA->UTA1CK = (uint8_t)bestck;
	}
	s_R9a02UartaSel = sel;
	reg->BRGCA = (uint8_t)bestk;
	dev->FutaHz = fsel >> bestck;
	dev->pUartDev->Rate = (int)best;

	if (running)
	{
		R9a02UartaStart(dev);
	}

	EnableInterrupt(state);

	return best;
}

static bool R9a02UartaStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int R9a02UartaRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	R9a02UartaDev_t *dev = (R9a02UartaDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	hCFifo_t fifo = udev->hRxFifo;
	bool poll = !pDev->bIntEn;
	int cnt = 0;

	if (pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

	// CFifoGetMultiple frees the slots before they are copied, interrupts
	// are disabled until the copy is done so that the interrupt cannot put
	// over them. Without interrupts the receive buffer is read first.
	while (cnt < Bufflen)
	{
		int l = Bufflen - cnt < R9A02_UARTA_COPY_MAX ? Bufflen - cnt : R9A02_UARTA_COPY_MAX;
		uintptr_t state = DisableInterrupt();

		if (poll)
		{
			R9a02UartaRxPoll(dev);
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

	return cnt;
}

static void R9a02UartaStopRx(DevIntrf_t * const pDev)
{
}

static bool R9a02UartaStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

// Several bytes into the TX CFIFO, in pieces of at most R9A02_UARTA_COPY_MAX
// bytes with interrupts disabled. Returns the number of bytes put.
__attribute__((noinline)) static int R9a02UartaTxPut(R9a02UartaDev_t * const pDev, uint8_t const *pData, int Datalen)
{
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;
	int cnt = 0;

	while (Datalen > 0)
	{
		int l = Datalen < R9A02_UARTA_COPY_MAX ? Datalen : R9A02_UARTA_COPY_MAX;
		uintptr_t state = DisableInterrupt();
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

static int R9a02UartaTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	R9a02UartaDev_t *dev = (R9a02UartaDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	int cnt = 0;

	if (pData == NULL || Datalen <= 0)
	{
		return 0;
	}

	if (Datalen == 1)
	{
		cnt = CFifoPutByte(udev->hTxFifo, *pData) ? 1 : 0;
	}
	else
	{
		cnt = R9a02UartaTxPut(dev, pData, Datalen);
	}

	if (!pDev->bIntEn)
	{
		R9a02UartaTxDrain(dev);
	}
	else if (udev->bTxReady)
	{
		// No transmission running: the first byte starts it, the buffer
		// empty interrupt sends the rest. TxData may also be called from an
		// interrupt.
		uintptr_t state = DisableInterrupt();

		if (udev->bTxReady && R9a02UartaTxHw(dev, true) > 0)
		{
			udev->bTxReady = false;
		}
		EnableInterrupt(state);
	}

	return cnt;
}

static void R9a02UartaStopTx(DevIntrf_t * const pDev)
{
}

// Transmit and receive off. Registers, pins and CFIFOs are kept, Enable
// restarts the UART.
static void R9a02UartaDisable(DevIntrf_t * const pDev)
{
	R9a02UartaDev_t *dev = (R9a02UartaDev_t *)pDev->pDevData;
	uintptr_t state = DisableInterrupt();

	R9a02UartaStop(dev);

	EnableInterrupt(state);
}

static void R9a02UartaEnable(DevIntrf_t * const pDev)
{
	R9a02UartaDev_t *dev = (R9a02UartaDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	uintptr_t state = DisableInterrupt();

	udev->RxOvrErrCnt = 0;
	udev->ParErrCnt = 0;
	udev->FramErrCnt = 0;
	udev->RxDropCnt = 0;
	udev->TxDropCnt = 0;

	CFifoFlush(udev->hTxFifo);
	udev->bTxReady = true;

	R9a02UartaStop(dev);
	R9a02UartaStart(dev);

	EnableInterrupt(state);
}

// Transmit, receive and the operation clock off, interrupts released, pins
// disconnected, the module stopped when the other channel is not used. A new
// UARTInit is needed to use it again.
static void R9a02UartaPowerOff(DevIntrf_t * const pDev)
{
	R9a02UartaDev_t *dev = (R9a02UartaDev_t *)pDev->pDevData;
	R9a02UartaDev_t *other = dev == &s_R9a02UartaDev[0] ? &s_R9a02UartaDev[1] : &s_R9a02UartaDev[0];
	uintptr_t state = DisableInterrupt();

	R9a02UartaStop(dev);
	dev->pReg->ASIMA0 = 0U;
	R9a02UartaReleaseInts(dev);
	IOPinDis(dev->pIOPinMap, 2);
	dev->bInUse = false;
	if (!other->bInUse)
	{
		R9a02ModuleStop(R9A02_MSTPCR_B, R9A02_MSTPCRB_UARTA);
	}

	EnableInterrupt(state);
}

static void *R9a02UartaGetHandle(DevIntrf_t * const pDev)
{
	return ((R9a02UartaDev_t *)pDev->pDevData)->pUartDev;
}

UARTDev_t const *R9a02UartaGetInstance(int Chan)
{
	return Chan >= 0 && Chan < R9A02_UART_UARTA_CNT ? s_R9a02UartaDev[Chan].pUartDev : NULL;
}

bool R9a02UartaInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	int chan = pCfg != NULL ? pCfg->DevNo - R9A02_UART_SAU_CNT : -1;

	if (pDev == NULL || pCfg == NULL || pCfg->pIOPinMap == NULL || pCfg->NbIOPins < 2 ||
		chan < 0 || chan >= R9A02_UART_UARTA_CNT)
	{
		return false;
	}

	if (pCfg->bDMAMode || pCfg->Mode != UART_MODE_UART || pCfg->Duplex != UART_DUPLEX_FULL ||
		pCfg->bIrDAMode || pCfg->FlowControl != UART_FLWCTRL_NONE ||
		(pCfg->DataBits != 5 && pCfg->DataBits != 7 && pCfg->DataBits != 8) ||
		pCfg->Parity == UART_PARITY_MARK || (pCfg->StopBits != 1 && pCfg->StopBits != 2) ||
		pCfg->Rate <= 0 || (pCfg->bIntMode && (pCfg->IntPrio < 0 || pCfg->IntPrio > (int)R9A02_CLIC_PRIO_MAX)))
	{
		return false;
	}

	const IOPinCfg_t *pins = (const IOPinCfg_t *)pCfg->pIOPinMap;

	for (int i = 0; i < 2; i++)
	{
		if (!R9a02PinValid(pins[i].PortNo, pins[i].PinNo))
		{
			return false;
		}
	}

	uint8_t asim1 = R9A02_ASIMA1_DIR_LSB;

	switch (pCfg->Parity)
	{
		case UART_PARITY_NONE:
			break;
		case UART_PARITY_SPACE:
			asim1 |= R9A02_ASIMA1_PS_ZERO;
			break;
		case UART_PARITY_ODD:
			asim1 |= R9A02_ASIMA1_PS_ODD;
			break;
		case UART_PARITY_EVEN:
			asim1 |= R9A02_ASIMA1_PS_EVEN;
			break;
		default:
			return false;
	}

	switch (pCfg->DataBits)
	{
		case 5:
			asim1 |= R9A02_ASIMA1_CL_5BIT;
			break;
		case 7:
			asim1 |= R9A02_ASIMA1_CL_7BIT;
			break;
		default:
			asim1 |= R9A02_ASIMA1_CL_8BIT;
			break;
	}

	if (pCfg->StopBits == 2)
	{
		asim1 |= R9A02_ASIMA1_SL_2BIT;
	}

	R9a02UartaDev_t *dev = &s_R9a02UartaDev[chan];
	R9A02_UARTA_Unit_Type *reg = dev->pReg;
	uintptr_t state = DisableInterrupt();

	// An instance running for another UART object is not taken over. The
	// module clock comes first, register writes before it are ignored.
	R9a02ModuleStart(R9A02_MSTPCR_B, R9A02_MSTPCRB_UARTA);
	if (dev->bInUse && dev->pUartDev != pDev && (reg->ASIMA0 & (R9A02_ASIMA0_TXEA | R9A02_ASIMA0_RXEA)))
	{
		EnableInterrupt(state);

		return false;
	}

	R9a02UartaReleaseInts(dev);
	if (reg->ASIMA0 & (R9A02_ASIMA0_TXEA | R9A02_ASIMA0_RXEA))
	{
		R9a02UartaStop(dev);
	}
	reg->ASIMA0 = 0U;

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_R9a02UartaFifoMem[chan][0], R9A02_UARTA_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_R9a02UartaFifoMem[chan][1], R9A02_UARTA_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	R9a02UartaDev_t *other = &s_R9a02UartaDev[chan ^ 1];

	if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
	{
		if (!dev->bInUse && !other->bInUse)
		{
			R9a02ModuleStop(R9A02_MSTPCR_B, R9A02_MSTPCRB_UARTA);
		}
		EnableInterrupt(state);

		return false;
	}

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;
	dev->pIOPinMap = pins;
	dev->RxDataMask = pCfg->DataBits == 5 ? 0x1FU : pCfg->DataBits == 7 ? 0x7FU : 0xFFU;

	// Format and interrupt sources with EN, TXEA and RXEA off (table 25.3).
	// Every receive error on RX_ENDI, transmit on buffer empty.
	reg->ASIMA1 = asim1;
	reg->ASIMA0 = (uint8_t)(R9A02_ASIMA0_ISRMA | (pCfg->bIntMode ? R9A02_ASIMA0_ISSMA : 0U));
	R9A02_PORGA_ULBS = (uint8_t)(R9A02_PORGA_ULBS & ~dev->UlbsMask);

	// A rate outside the characterized range or more than 2 % away from the
	// one made is not supported
	uint32_t req = (uint32_t)pCfg->Rate;
	uint32_t rate = req >= R9A02_UARTA_RATE_MIN && req <= R9A02_UARTA_RATE_MAX ?
					R9a02UartaSetRate(&pDev->DevIntrf, req) : 0U;

	if (rate == 0U || (rate > req ? rate - req : req - rate) * R9A02_UARTA_RATE_TOL > req)
	{
		if (!dev->bInUse && !other->bInUse)
		{
			R9a02ModuleStop(R9A02_MSTPCR_B, R9A02_MSTPCRB_UARTA);
		}
		EnableInterrupt(state);

		return false;
	}

	// The RX pin goes to its idle level before EN (25.5)
	IOPinCfg(dev->pIOPinMap, 2);

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
	pDev->DevIntrf.GetHandle = R9a02UartaGetHandle;
	pDev->DevIntrf.Disable = R9a02UartaDisable;
	pDev->DevIntrf.Enable = R9a02UartaEnable;
	pDev->DevIntrf.GetRate = R9a02UartaGetRate;
	pDev->DevIntrf.SetRate = R9a02UartaSetRate;
	pDev->DevIntrf.StartRx = R9a02UartaStartRx;
	pDev->DevIntrf.RxData = R9a02UartaRxData;
	pDev->DevIntrf.StopRx = R9a02UartaStopRx;
	pDev->DevIntrf.StartTx = R9a02UartaStartTx;
	pDev->DevIntrf.TxData = R9a02UartaTxData;
	pDev->DevIntrf.StopTx = R9a02UartaStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = R9a02UartaPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	if (pCfg->bIntMode)
	{
		int txprio = pCfg->IntPrio > 0 ? pCfg->IntPrio - 1 : 0;

		dev->IntRx = R9a02RegisterIntHandler(dev->EvtRx, pCfg->IntPrio, R9a02UartaRxIRQHandler, dev);
		dev->IntTx = R9a02RegisterIntHandler(dev->EvtTx, txprio, R9a02UartaTxIRQHandler, dev);
	}
	else
	{
		dev->IntRx = R9a02RouteEvent(dev->EvtRx);
	}

	if (dev->IntRx < 0 || (pCfg->bIntMode && dev->IntTx < 0))
	{
		R9a02UartaReleaseInts(dev);
		IOPinDis(dev->pIOPinMap, 2);
		if (!dev->bInUse && !other->bInUse)
		{
			R9a02ModuleStop(R9A02_MSTPCR_B, R9A02_MSTPCRB_UARTA);
		}
		EnableInterrupt(state);

		return false;
	}

	dev->bInUse = true;
	R9a02UartaEnable(&pDev->DevIntrf);

	EnableInterrupt(state);

	return true;
}
