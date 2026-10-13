/**-------------------------------------------------------------------------
@file	uart_r9a02.cpp

@brief	R9A02G021 UART driver on the serial array units (SAU)

R9A02G021 User's Manual R01UH1036EJ0110, chapter 23. Each UART uses two SAU
channels, the even one transmits, the odd one receives:

	DevNo 0 : UART0, SAU0 channels 0 and 1, TxD0 and RxD0
	DevNo 1 : UART1, SAU0 channels 2 and 3, TxD1 and RxD1
	DevNo 2 : UART2, SAU1 channels 0 and 1, TxD2 and RxD2

DevNo 3 and 4, UARTA0 and UARTA1, are handed to uarta_r9a02.cpp when the
application links it (uarta_r9a02.h).

Format: 7 or 8 data bits, no, odd, even or space (0) parity, 1 or 2 stop
bits, LSB first, no flow control, no DMA. Unsupported settings fail
UARTInit.

Rate: fMCK = PCLKB / 2^PRS (PRS 0 to 15, fMCK at most 32 MHz, table 23.5),
rate = fMCK / (2 * (STCLK + 1)), STCLK 2 to 127. SetRate sets the closest
rate and returns it. The operation clock is chosen by sau_r9a02.cpp: the
prescaler SPSm is shared by the channels of a unit and is not changed while
another channel of the unit runs or uses it, the rate is then set with STCLK
only. The UART claims its two channels there, a channel cannot be used by the
UART and the SPI at the same time.

The SAU has a one character buffer each way. With interrupts, transmit uses
the buffer empty interrupt (SMR.MD0 = 1), receive the transfer end
interrupt and the error interrupt (SCR.EOC = 1). The transmit interrupt
runs one level below IntPrio so that the receive interrupt can preempt it.
Without interrupts, TxData waits for each character to go out and RxData
reads the receive buffer when called.

Pins: PinOp R9A02_PSEL_SAU (IOPINOP_FUNC16). IOPinConfig puts the RX pin in
input mode with NCODR = 1 (16.5.1). The RxD noise filter (SNFEN) is on.

@author	Hoang Nguyen Hoan
@date	May 12, 2026

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
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "sau_r9a02.h"
#include "uarta_r9a02.h"

// Default CFIFO size when the configuration gives no memory
#ifndef R9A02_UART_CFIFO_SIZE
#define R9A02_UART_CFIFO_SIZE		16
#endif
#define R9A02_UART_CFIFO_MEMSIZE	CFIFO_MEMSIZE(R9A02_UART_CFIFO_SIZE)

// Most bytes TxData and RxData copy to or from a CFIFO with interrupts
// disabled, to keep the RX interrupt latency short
#define R9A02_UART_COPY_MAX			8

#define R9A02_SAU_SIR_ALL			(R9A02_SAU_SIR_OVCT | R9A02_SAU_SIR_PECT | R9A02_SAU_SIR_FECT)

// UARTA is referenced weakly, it is linked only when the application asks for
// it (uarta_r9a02.h) and stays out of the images that do not use it, the DFU
// boot among them. Without it DevNo 3 and 4 fail UARTInit.
bool R9a02UartaInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg) __attribute__((weak));
UARTDev_t const *R9a02UartaGetInstance(int Chan) __attribute__((weak));

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	R9A02_SAU_Type *pReg;
	uint8_t Unit;					// SAU unit
	uint8_t TxCh;					// Transmit channel, receive is TxCh + 1
	uint8_t SnfenMask;				// RxD noise filter bit in SNFEN
	uint8_t UlbsMask;				// Loopback bit in ULBS
	uint8_t EvtTx;					// ICU events
	uint8_t EvtRx;
	uint8_t EvtErr;
	uint16_t RxDataMask;			// 0x7F with 7 data bits
	uint32_t FmckHz;				// Operation clock of the channels
	UARTDev_t *pUartDev;			// Generic UART device data
	const IOPinCfg_t *pIOPinMap;	// RX, TX
	int NbIOPins;
	int IntTx;						// IELSR numbers, -1 when not routed
	int IntRx;
	int IntErr;
} R9a02UartDev_t;
#pragma pack(pop)

// Default CFIFO memory, RX then TX, of each UART
alignas(4) static uint8_t s_R9a02UartFifoMem[3][2][R9A02_UART_CFIFO_MEMSIZE];

static R9a02UartDev_t s_R9a02UartDev[] = {
	{
		.DevNo = 0,
		.pReg = R9A02_SAU0,
		.Unit = 0,
		.TxCh = 0,
		.SnfenMask = R9A02_SNFEN_SNFEN00,
		.UlbsMask = R9A02_ULBS_UART0,
		.EvtTx = R9A02_EVT_SAU0_ENDI0,
		.EvtRx = R9A02_EVT_SAU0_ENDI1,
		.EvtErr = R9A02_EVT_SAU0_INTSRE0,
		.RxDataMask = 0xFF,
		.FmckHz = 0,
		.pUartDev = NULL,
		.pIOPinMap = NULL,
		.NbIOPins = 0,
		.IntTx = -1,
		.IntRx = -1,
		.IntErr = -1,
	},
	{
		.DevNo = 1,
		.pReg = R9A02_SAU0,
		.Unit = 0,
		.TxCh = 2,
		.SnfenMask = R9A02_SNFEN_SNFEN10,
		.UlbsMask = R9A02_ULBS_UART1,
		.EvtTx = R9A02_EVT_SAU0_ENDI2,
		.EvtRx = R9A02_EVT_SAU0_ENDI3,
		.EvtErr = R9A02_EVT_SAU0_INTSRE1,
		.RxDataMask = 0xFF,
		.FmckHz = 0,
		.pUartDev = NULL,
		.pIOPinMap = NULL,
		.NbIOPins = 0,
		.IntTx = -1,
		.IntRx = -1,
		.IntErr = -1,
	},
	{
		.DevNo = 2,
		.pReg = R9A02_SAU1,
		.Unit = 1,
		.TxCh = 0,
		.SnfenMask = R9A02_SNFEN_SNFEN20,
		.UlbsMask = R9A02_ULBS_UART2,
		.EvtTx = R9A02_EVT_SAU1_ENDI0,
		.EvtRx = R9A02_EVT_SAU1_ENDI1,
		.EvtErr = R9A02_EVT_SAU1_INTSRE2,
		.RxDataMask = 0xFF,
		.FmckHz = 0,
		.pUartDev = NULL,
		.pIOPinMap = NULL,
		.NbIOPins = 0,
		.IntTx = -1,
		.IntRx = -1,
		.IntErr = -1,
	},
};

static const int s_NbR9a02UartDev = sizeof(s_R9a02UartDev) / sizeof(s_R9a02UartDev[0]);

static_assert(sizeof(s_R9a02UartFifoMem) / sizeof(s_R9a02UartFifoMem[0]) == sizeof(s_R9a02UartDev) / sizeof(s_R9a02UartDev[0]),
			  "One default CFIFO memory per UART");
static_assert(sizeof(s_R9a02UartDev) / sizeof(s_R9a02UartDev[0]) == R9A02_UART_SAU_CNT, "SAU UART DevNo 0 to 2");

static void R9a02UartReleaseInts(R9a02UartDev_t * const pDev)
{
	R9a02UnregisterIntHandler(pDev->IntTx);
	R9a02UnregisterIntHandler(pDev->IntRx);
	R9a02UnregisterIntHandler(pDev->IntErr);
	pDev->IntTx = -1;
	pDev->IntRx = -1;
	pDev->IntErr = -1;
}

// Both channels stopped, TxD held at the idle level by SO (23.6.1)
static void R9a02UartStop(R9a02UartDev_t * const pDev)
{
	R9A02_SAU_Type *reg = pDev->pReg;
	uint16_t txbit = (uint16_t)(1U << pDev->TxCh);

	reg->ST = (uint16_t)(3U << pDev->TxCh);
	reg->SOE = (uint16_t)(reg->SOE & ~txbit);
	reg->SO = (uint16_t)(reg->SO | txbit);
}

// Both channels started. The receive channel needs 4 fMCK cycles between its
// SCR write and SS (23.6.2).
static void R9a02UartStart(R9a02UartDev_t * const pDev)
{
	R9A02_SAU_Type *reg = pDev->pReg;
	uint16_t txbit = (uint16_t)(1U << pDev->TxCh);

	reg->SIR[pDev->TxCh + 1] = R9A02_SAU_SIR_ALL;
	reg->SO = (uint16_t)(reg->SO | txbit);
	reg->SOE = (uint16_t)(reg->SOE | txbit);
	R9a02DelayUs(4000000UL / (pDev->FmckHz ? pDev->FmckHz : 1U) + 1U);
	reg->SS = (uint16_t)(3U << pDev->TxCh);
}

// The receive buffer, with its error flags (23.6.5). Data first, then the
// flags read and cleared with the value read. A framing error restarts the
// receive channel. Returns the number of bytes put in the RX CFIFO.
static int R9a02UartRxHw(R9a02UartDev_t * const pDev, uint8_t *pErr)
{
	UARTDev_t *udev = pDev->pUartDev;
	R9A02_SAU_Type *reg = pDev->pReg;
	uint32_t ch = pDev->TxCh + 1U;
	uint16_t ssr = reg->SSR[ch];

	if ((ssr & (R9A02_SAU_SSR_BFF | R9A02_SAU_SSR_ERR_Msk)) == 0U)
	{
		return 0;
	}

	uint16_t d = reg->SDR[ch];
	uint16_t err = reg->SSR[ch] & R9A02_SAU_SSR_ERR_Msk;

	if (err)
	{
		reg->SIR[ch] = err;
		if (err & R9A02_SAU_SSR_OVF)
		{
			udev->RxOvrErrCnt++;
			*pErr |= UART_LINESTATE_OVR;
		}
		if (err & R9A02_SAU_SSR_PEF)
		{
			udev->ParErrCnt++;
			*pErr |= UART_LINESTATE_PARERR;
		}
		if (err & R9A02_SAU_SSR_FEF)
		{
			udev->FramErrCnt++;
			*pErr |= UART_LINESTATE_FRMERR;
			reg->ST = (uint16_t)(1U << ch);
			reg->SS = (uint16_t)(1U << ch);
		}
		if (err & (R9A02_SAU_SSR_PEF | R9A02_SAU_SSR_FEF))
		{
			// The character is bad. After an overrun alone SDR holds the
			// last character received, the one before it is lost.
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

// Next byte of the TX CFIFO into SDR. SDR takes a byte only when empty, a
// write with BFF set is lost (23.3.11). Returns 1 when a byte was written, 0
// when the CFIFO is empty, -1 when SDR is full.
static int R9a02UartTxHw(R9a02UartDev_t * const pDev)
{
	R9A02_SAU_Type *reg = pDev->pReg;

	if (reg->SSR[pDev->TxCh] & R9A02_SAU_SSR_BFF)
	{
		return -1;
	}

	uint8_t *p = CFifoGet(pDev->pUartDev->hTxFifo);

	if (p == NULL)
	{
		return 0;
	}

	reg->SDR[pDev->TxCh] = *p;

	return 1;
}

// Without interrupts, everything in the TX CFIFO is sent now. Each byte waits
// at most two character times for SDR.
static void R9a02UartTxDrain(R9a02UartDev_t * const pDev)
{
	R9A02_SAU_Type *reg = pDev->pReg;
	UARTDev_t *udev = pDev->pUartDev;
	uint32_t rate = udev->Rate > 0 ? (uint32_t)udev->Rate : 1U;
	uint32_t timeout = (SystemCoreClock / rate + 1U) * 24U + 1000U;

	if ((reg->SE & (1U << pDev->TxCh)) == 0U)
	{
		return;
	}

	while (CFifoUsed(udev->hTxFifo) > 0)
	{
		uint32_t n = timeout;

		while ((reg->SSR[pDev->TxCh] & R9A02_SAU_SSR_BFF) && --n > 0U)
		{
		}
		if (n == 0U || R9a02UartTxHw(pDev) <= 0)
		{
			break;
		}
	}
}

// Receive end and receive error interrupts
static void R9a02UartRxIRQHandler(int IntNo, void *pCtx)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;
	uint8_t err = 0;
	int cnt = R9a02UartRxHw(dev, &err);

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

// Transmit buffer empty interrupt: the byte in SDR went to the shift register
static void R9a02UartTxIRQHandler(int IntNo, void *pCtx)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;
	uintptr_t state = DisableInterrupt();

	if (R9a02UartTxHw(dev) == 0)
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

static uint32_t R9a02UartGetRate(DevIntrf_t * const pDev)
{
	return ((R9a02UartDev_t *)pDev->pDevData)->pUartDev->Rate;
}

static uint32_t R9a02UartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pDev->pDevData;
	R9A02_SAU_Type *reg = dev->pReg;
	uint16_t chmsk = (uint16_t)(3U << dev->TxCh);
	R9a02SauClk_t clk;

	if (Rate == 0U)
	{
		return 0;
	}

	uintptr_t state = DisableInterrupt();

	// SPS, SMR and STCLK are written with the channels stopped
	bool running = (reg->SE & chmsk) != 0U;

	if (running)
	{
		reg->ST = chmsk;
	}

	// Rate = fMCK / (2 x (STCLK + 1)), STCLK from 2 (23.3.8). Both channels
	// use the same operation clock (table 23.86).
	uint32_t best = R9a02SauSetRate(dev->Unit, chmsk, Rate, R9A02_SAU_SDR_STCLK_UART_MIN + 1U,
									UINT32_MAX, &clk);

	if (best != 0U)
	{
		for (uint32_t ch = dev->TxCh; ch < dev->TxCh + 2U; ch++)
		{
			reg->SMR[ch] = (uint16_t)((reg->SMR[ch] & ~R9A02_SAU_SMR_CKS) | clk.Cks);
			reg->SDR[ch] = clk.Sdr;
		}
		dev->FmckHz = clk.FmckHz;
		dev->pUartDev->Rate = (int)best;
	}

	if (running)
	{
		R9a02UartStart(dev);
	}

	EnableInterrupt(state);

	return best;
}

static bool R9a02UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int R9a02UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pDev->pDevData;
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
		int l = Bufflen - cnt < R9A02_UART_COPY_MAX ? Bufflen - cnt : R9A02_UART_COPY_MAX;
		uintptr_t state = DisableInterrupt();

		if (poll)
		{
			uint8_t err = 0;

			R9a02UartRxHw(dev, &err);
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

static void R9a02UartStopRx(DevIntrf_t * const pDev)
{
}

static bool R9a02UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

// Several bytes into the TX CFIFO. CFifoPutMultiple publishes the slots
// before they are written, so interrupts are disabled for the copy, in pieces
// of at most R9A02_UART_COPY_MAX bytes. Returns the number of bytes put.
__attribute__((noinline)) static int R9a02UartTxPut(R9a02UartDev_t * const pDev, uint8_t const *pData, int Datalen)
{
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;
	int cnt = 0;

	while (Datalen > 0)
	{
		int l = Datalen < R9A02_UART_COPY_MAX ? Datalen : R9A02_UART_COPY_MAX;
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

static int R9a02UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pDev->pDevData;
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
		cnt = R9a02UartTxPut(dev, pData, Datalen);
	}

	if (!pDev->bIntEn)
	{
		R9a02UartTxDrain(dev);
	}
	else if (udev->bTxReady)
	{
		// The first byte starts the transmission, the buffer empty interrupt
		// sends the rest. Checked again with interrupts disabled, TxData may
		// also be called from an interrupt.
		uintptr_t state = DisableInterrupt();

		if (udev->bTxReady && R9a02UartTxHw(dev) > 0)
		{
			udev->bTxReady = false;
		}
		EnableInterrupt(state);
	}

	return cnt;
}

static void R9a02UartStopTx(DevIntrf_t * const pDev)
{
}

// Channels stopped. Registers, pins and CFIFOs are kept, Enable restarts the
// UART.
static void R9a02UartDisable(DevIntrf_t * const pDev)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pDev->pDevData;
	uintptr_t state = DisableInterrupt();

	R9a02UartStop(dev);

	EnableInterrupt(state);
}

static void R9a02UartEnable(DevIntrf_t * const pDev)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	uintptr_t state = DisableInterrupt();

	udev->RxOvrErrCnt = 0;
	udev->ParErrCnt = 0;
	udev->FramErrCnt = 0;
	udev->RxDropCnt = 0;
	udev->TxDropCnt = 0;

	CFifoFlush(udev->hTxFifo);
	udev->bTxReady = true;

	R9a02UartStop(dev);
	R9a02UartStart(dev);

	EnableInterrupt(state);
}

// Interrupts released, pins disconnected, the SAU stopped when none of its
// channels runs. A new UARTInit is needed to use it again.
static void R9a02UartPowerOff(DevIntrf_t * const pDev)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pDev->pDevData;
	uintptr_t state = DisableInterrupt();

	R9a02UartStop(dev);
	R9a02UartReleaseInts(dev);
	IOPinDis(dev->pIOPinMap, dev->NbIOPins);
	R9a02SauRelease(dev->Unit, (uint16_t)(3U << dev->TxCh), dev);

	EnableInterrupt(state);
}

static void *R9a02UartGetHandle(DevIntrf_t * const pDev)
{
	return ((R9a02UartDev_t *)pDev->pDevData)->pUartDev;
}

UARTDev_t const *UARTGetInstance(int DevNo)
{
	if (DevNo >= R9A02_UART_SAU_CNT)
	{
		return R9a02UartaGetInstance != NULL ? R9a02UartaGetInstance(DevNo - R9A02_UART_SAU_CNT) : NULL;
	}

	return DevNo >= 0 && DevNo < s_NbR9a02UartDev ? s_R9a02UartDev[DevNo].pUartDev : NULL;
}

// No modem control lines on the SAU UART
void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
}

bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->pIOPinMap == NULL || pCfg->NbIOPins < 2)
	{
		return false;
	}

	if (pCfg->DevNo >= R9A02_UART_SAU_CNT && pCfg->DevNo < R9A02_UART_DEV_CNT)
	{
		return R9a02UartaInit != NULL && R9a02UartaInit(pDev, pCfg);
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbR9a02UartDev)
	{
		return false;
	}

	if (pCfg->bDMAMode || pCfg->Mode != UART_MODE_UART || pCfg->Duplex != UART_DUPLEX_FULL ||
		pCfg->bIrDAMode || pCfg->FlowControl != UART_FLWCTRL_NONE ||
		(pCfg->DataBits != 7 && pCfg->DataBits != 8) || pCfg->Parity == UART_PARITY_MARK ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) || pCfg->Rate <= 0 ||
		(pCfg->bIntMode && (pCfg->IntPrio < 0 || pCfg->IntPrio > (int)R9A02_CLIC_PRIO_MAX)))
	{
		return false;
	}

	uint16_t ptc;

	switch (pCfg->Parity)
	{
		case UART_PARITY_NONE:
			ptc = 0U;
			break;
		case UART_PARITY_SPACE:
			ptc = R9A02_SAU_SCR_PTC_ZERO;
			break;
		case UART_PARITY_EVEN:
			ptc = R9A02_SAU_SCR_PTC_EVEN;
			break;
		case UART_PARITY_ODD:
			ptc = R9A02_SAU_SCR_PTC_ODD;
			break;
		default:
			return false;
	}

	R9a02UartDev_t *dev = &s_R9a02UartDev[pCfg->DevNo];
	R9A02_SAU_Type *reg = dev->pReg;
	uint16_t chmsk = (uint16_t)(3U << dev->TxCh);
	uintptr_t state = DisableInterrupt();

	// The claim fails when another driver, the SPI, has one of the channels.
	// It also starts the SAU module clock, register writes before it are
	// ignored. An instance running for another UART object is not taken
	// over, the channels then stay claimed by this instance.
	if (!R9a02SauClaim(dev->Unit, chmsk, dev) ||
		(dev->pUartDev != NULL && dev->pUartDev != pDev && (reg->SE & chmsk)))
	{
		EnableInterrupt(state);

		return false;
	}

	R9a02UartReleaseInts(dev);
	R9a02UartStop(dev);

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_R9a02UartFifoMem[pCfg->DevNo][0], R9A02_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_R9a02UartFifoMem[pCfg->DevNo][1], R9A02_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
	{
		R9a02SauRelease(dev->Unit, chmsk, dev);
		EnableInterrupt(state);

		return false;
	}

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;
	dev->pIOPinMap = (const IOPinCfg_t *)pCfg->pIOPinMap;
	dev->NbIOPins = 2;
	dev->RxDataMask = pCfg->DataBits == 7 ? 0x7FU : 0xFFU;

	// Mode and format, written with the channels stopped (23.3.2 to 23.3.7).
	// The operation clock, SMR.CKS, is set by R9a02UartSetRate.
	uint16_t fmt = (uint16_t)(R9A02_SAU_SCR_DIR_LSB | ptc |
							  (pCfg->DataBits == 7 ? R9A02_SAU_SCR_DLS_7BIT : R9A02_SAU_SCR_DLS_8BIT));

	reg->SMR[dev->TxCh] = (uint16_t)(R9A02_SAU_SMR_FIXED | R9A02_SAU_SMR_MD_UART | R9A02_SAU_SMR_MD0);
	reg->SMR[dev->TxCh + 1] = (uint16_t)(R9A02_SAU_SMR_FIXED | R9A02_SAU_SMR_MD_UART | R9A02_SAU_SMR_STS);
	reg->SCR[dev->TxCh] = (uint16_t)(R9A02_SAU_SCR_TRXE_TX | fmt |
									 (pCfg->StopBits == 2 ? R9A02_SAU_SCR_SLC_2BIT : R9A02_SAU_SCR_SLC_1BIT));
	reg->SCR[dev->TxCh + 1] = (uint16_t)(R9A02_SAU_SCR_TRXE_RX | fmt | R9A02_SAU_SCR_SLC_1BIT |
										 (pCfg->bIntMode ? R9A02_SAU_SCR_EOC : 0U));
	reg->SOL = (uint16_t)(reg->SOL & ~(1U << dev->TxCh));

	// RxD noise filter on (23.3.27), loopback off (23.3.28)
	R9A02_PORGA_SNFEN = (uint8_t)(R9A02_PORGA_SNFEN | dev->SnfenMask);
	R9A02_PORGA_ULBS = (uint8_t)(R9A02_PORGA_ULBS & ~dev->UlbsMask);

	if (R9a02UartSetRate(&pDev->DevIntrf, (uint32_t)pCfg->Rate) == 0U)
	{
		R9a02SauRelease(dev->Unit, chmsk, dev);
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
	pDev->DevIntrf.GetHandle = R9a02UartGetHandle;
	pDev->DevIntrf.Disable = R9a02UartDisable;
	pDev->DevIntrf.Enable = R9a02UartEnable;
	pDev->DevIntrf.GetRate = R9a02UartGetRate;
	pDev->DevIntrf.SetRate = R9a02UartSetRate;
	pDev->DevIntrf.StartRx = R9a02UartStartRx;
	pDev->DevIntrf.RxData = R9a02UartRxData;
	pDev->DevIntrf.StopRx = R9a02UartStopRx;
	pDev->DevIntrf.StartTx = R9a02UartStartTx;
	pDev->DevIntrf.TxData = R9a02UartTxData;
	pDev->DevIntrf.StopTx = R9a02UartStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = R9a02UartPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	if (pCfg->bIntMode)
	{
		int txprio = pCfg->IntPrio > 0 ? pCfg->IntPrio - 1 : 0;

		dev->IntRx = R9a02RegisterIntHandler(dev->EvtRx, pCfg->IntPrio, R9a02UartRxIRQHandler, dev);
		dev->IntErr = R9a02RegisterIntHandler(dev->EvtErr, pCfg->IntPrio, R9a02UartRxIRQHandler, dev);
		dev->IntTx = R9a02RegisterIntHandler(dev->EvtTx, txprio, R9a02UartTxIRQHandler, dev);

		if (dev->IntRx < 0 || dev->IntErr < 0 || dev->IntTx < 0)
		{
			R9a02UartReleaseInts(dev);
			IOPinDis(dev->pIOPinMap, dev->NbIOPins);
			R9a02SauRelease(dev->Unit, chmsk, dev);
			EnableInterrupt(state);

			return false;
		}
	}

	R9a02UartEnable(&pDev->DevIntrf);

	EnableInterrupt(state);

	return true;
}
