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
bits, LSB first, no flow control. Unsupported settings fail
UARTInit.

Rate: fMCK = PCLKB / 2^PRS (PRS 0 to 15, fMCK at most 32 MHz, table 23.5),
rate = fMCK / (2 * (STCLK + 1)), STCLK 2 to 127. SetRate sets the closest
rate and returns it. The operation clock is chosen by sau_r9a02.cpp: the
prescaler SPSm is shared by the channels of a unit and is not changed while
another channel of the unit runs or uses it, the rate is then set with STCLK
only. The UART claims its two channels there, a channel cannot be used by the
UART and the SPI at the same time.

The SAU has a one character buffer each way. With interrupts, an isolated
transmit byte uses transfer end to let following bytes accumulate. Queued
transmission uses buffer empty (SMR.MD0 = 1). Receive uses transfer end
and the error interrupt (SCR.EOC = 1). The transmit interrupt
runs one level below IntPrio so that the receive interrupt can preempt it.
Without interrupts, TxData and RxData service both directions without
waiting for a character on the wire. The application keeps calling them
while queued bytes remain, as with the STM32 polling implementation.

bDMAMode selects DTC transmit in private blocks, as in the SAM4L port.
RX retains the interrupt/polling path, including per-character errors and
short-burst callbacks. The DTC writes SDR by halfwords; the CPU starts each
block, then the buffer-empty events transfer its remaining characters.
An isolated CPU byte uses transfer end instead, allowing the producer to
queue more bytes before the next block is selected, as in the STM32 TC path.
The TX completion route is enabled in the CLIC also with bIntMode false
(manual 14.5); in that mode it chains blocks without application callbacks.

Pins: PinOp R9A02_PSEL_SAU (IOPINOP_FUNC16). IOPinConfig puts the RX pin in
input mode with NCODR = 1 (16.5.1). The RxD noise filter (SNFEN) is on.

Reception in Snooze, UART0 and UART2 (23.6.3), with R9a02UartSnooze
(sau_r9a02.h). The UART then runs on the operation clock of table 23.95,
PCLKB / 2^5 at 4800 and PCLKB / 2^4 at 9600, so that only STCLK of the
receive channel changes for Snooze and no other channel of the unit is
touched. A Software Standby hook sets the receive channel for Snooze just
before WFI (figure 23.41 steps 1 to 3: SCRm1.EOC = 1 and SSCm.SSEC = 1, an
error sends the MCU back to Software Standby). The character that ends
Snooze wakes the CPU through SELSR0 (R9a02SnoozeSet), its interrupt reads
it while SSCm.SWC is still 1 (figure 23.37 note 1) and puts the channel
back in normal operation (steps 10 to 12), the hook does it after a
wake-up by another source. The transmit channel stays stopped in Software
Standby, after the TX CFIFO was sent.

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
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "interrupt_r9a02.h"
#include "dtc_r9a02.h"
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

// Private TX staging, as in SAM4L. SDR requires halfword writes (23.3.8).
#define R9A02_UART_DMA_SIZE			16

#define R9A02_SAU_SIR_ALL			(R9A02_SAU_SIR_OVCT | R9A02_SAU_SIR_PECT | R9A02_SAU_SIR_FECT)

// UARTA is referenced weakly, it is linked only when the application asks for
// it (uarta_r9a02.h) and stays out of the images that do not use it, the DFU
// boot among them. Without it DevNo 3 and 4 fail UARTInit.
bool R9a02UartaInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg) __attribute__((weak));
UARTDev_t const *R9a02UartaGetInstance(int Chan) __attribute__((weak));

struct __R9a02_Uart_Snz_Ops;

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
	const struct __R9a02_Uart_Snz_Ops *pSnz;	// Reception in Snooze set up, NULL when not
	R9a02DtcInfo_t TxDtc;
	uint16_t TxDmaMem[R9A02_UART_DMA_SIZE];
} R9a02UartDev_t;

// Reception in Snooze, reached through R9a02UartDev_t.pSnz, which only
// R9a02UartSnooze sets: an image that does not call it does not link this
// code and its data, the DFU boot among them.
typedef struct __R9a02_Uart_Snz_Ops {
	bool (*Clock)(R9a02UartDev_t * const pDev, uint32_t Rate);
	void (*Off)(R9a02UartDev_t * const pDev);
} R9a02UartSnzOps_t;

// State of the reception in Snooze. Snooze has one request source (10.8.1),
// so one UART at a time.
typedef struct {
	R9a02UartDev_t *pDev;			// UART set up
	uint16_t SdrNormal;				// Receive channel SDR, STCLK in normal operation
	uint16_t SdrSnz;				// and in Snooze (table 23.95)
	volatile bool bArmed;			// Receive channel set for Snooze, SSCm.SWC = 1
} R9a02UartSnz_t;
#pragma pack(pop)

// Default CFIFO memory, RX then TX, of each UART
alignas(4) static uint8_t s_R9a02UartFifoMem[3][2][R9A02_UART_CFIFO_MEMSIZE];

static R9a02UartSnz_t s_R9a02UartSnz;

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
		.pSnz = NULL,
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
		.pSnz = NULL,
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
		.pSnz = NULL,
	},
};

static const int s_NbR9a02UartDev = sizeof(s_R9a02UartDev) / sizeof(s_R9a02UartDev[0]);

static_assert(sizeof(s_R9a02UartFifoMem) / sizeof(s_R9a02UartFifoMem[0]) == sizeof(s_R9a02UartDev) / sizeof(s_R9a02UartDev[0]),
			  "One default CFIFO memory per UART");
static_assert(sizeof(s_R9a02UartDev) / sizeof(s_R9a02UartDev[0]) == R9A02_UART_SAU_CNT, "SAU UART DevNo 0 to 2");

static void R9a02UartReleaseInts(R9a02UartDev_t * const pDev)
{
	R9a02DtcClear(pDev->IntTx);
	pDev->TxDtc.Cra = 0U;
	R9a02UnregisterIntHandler(pDev->IntTx);
	R9a02UnregisterIntHandler(pDev->IntRx);
	R9a02UnregisterIntHandler(pDev->IntErr);
	pDev->IntTx = -1;
	pDev->IntRx = -1;
	pDev->IntErr = -1;
}

// Both channels stopped, TxD held at the idle level by SO (23.6.1).
// ST retains the pin level; clear SOE before writing SO (23.3.15, 23.3.19).
static void R9a02UartStop(R9a02UartDev_t * const pDev)
{
	R9A02_SAU_Type *reg = pDev->pReg;
	uint16_t txbit = (uint16_t)(1U << pDev->TxCh);

	if (pDev->IntTx >= 0 && pDev->pUartDev->DevIntrf.bDma)
	{
		R9a02DtcDisable(pDev->IntTx);
	}
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
static int R9a02UartRxRead(R9a02UartDev_t * const pDev, uint8_t *pErr)
{
	UARTDev_t *udev = pDev->pUartDev;
	R9A02_SAU_Type *reg = pDev->pReg;
	uint32_t ch = pDev->TxCh + 1U;
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

// The empty check stays inline in the streaming path, as in the STM32 port.
// Error handling and FIFO insertion are only entered when RX needs service.
static inline __attribute__((always_inline)) int R9a02UartRxHw(R9a02UartDev_t * const pDev, uint8_t *pErr)
{
	if ((pDev->pReg->SSR[pDev->TxCh + 1U] & (R9A02_SAU_SSR_BFF | R9A02_SAU_SSR_ERR_Msk)) == 0U)
	{
		return 0;
	}

	return R9a02UartRxRead(pDev, pErr);
}

// Next byte of the TX CFIFO into SDR. SDR takes a byte only when empty, a
// write with BFF set is lost (23.3.11), a stopped channel takes none: the
// bytes wait in the CFIFO. Returns 1 when a byte was written, 0 when the
// CFIFO is empty, -1 when SDR is full or the channel stopped.
static inline __attribute__((always_inline)) int R9a02UartTxHw(R9a02UartDev_t * const pDev, bool bTransferEnd = false)
{
	R9A02_SAU_Type *reg = pDev->pReg;
	uint32_t ch = pDev->TxCh;
	volatile uint16_t *sdr = &reg->SDR[ch];

	if ((reg->SSR[ch] & R9A02_SAU_SSR_BFF) || (reg->SE & (1U << ch)) == 0U)
	{
		return -1;
	}

	int c = CFifoGetByte(pDev->pUartDev->hTxFifo);

	if (c < 0)
	{
		return 0;
	}

	if (bTransferEnd)
	{
		// Let bytes accumulate during this character before servicing
		// the next data, as the STM32 TC path does. MD0 may change while
		// running (figure 23.33); set it before writing this character.
		reg->SMR[ch] = (uint16_t)(reg->SMR[ch] & ~R9A02_SAU_SMR_MD0);
	}
	*sdr = (uint8_t)c;

	return 1;
}

// DTC drains a private block; CFIFO storage may be reused immediately.
// A normal transfer ends when its last halfword enters SDR, not on wire
// completion. Stage the next block while that byte is still transmitting,
// then leave the buffer-empty request to the CPU to arm it. Arming while
// SDR is full could lose an event while DTCE is set.
// Called with interrupts disabled, from TxData, polling or the TX handler.
static void R9a02UartTxDma(R9a02UartDev_t * const pDev)
{
	UARTDev_t *udev = pDev->pUartDev;
	R9A02_SAU_Type *reg = pDev->pReg;
	volatile R9a02DtcInfo_t *info = &pDev->TxDtc;

	if (R9a02DtcEnabled(pDev->IntTx) || (reg->SE & (1U << pDev->TxCh)) == 0U)
	{
		return;
	}
	if (info->Cra == 0U)
	{
		int cnt = 0;

		while (cnt < R9A02_UART_DMA_SIZE)
		{
			int len = R9A02_UART_DMA_SIZE - cnt;
			uint8_t *p = CFifoGetMultiple(udev->hTxFifo, &len);

			if (p == NULL)
			{
				break;
			}
			for (int i = 0; i < len; i++)
			{
				pDev->TxDmaMem[cnt + i] = p[i];
			}
			cnt += len;
		}
		if (cnt == 0)
		{
			// Ready still means SDR is available, not just an empty CFIFO.
			if ((reg->SSR[pDev->TxCh] & R9A02_SAU_SSR_BFF) == 0U)
			{
				udev->bTxReady = true;
			}

			return;
		}
		info->Sar = (uint32_t)(uintptr_t)pDev->TxDmaMem;
		info->Cra = (uint16_t)cnt;
	}
	udev->bTxReady = false;
	if (reg->SSR[pDev->TxCh] & R9A02_SAU_SSR_BFF)
	{
		return;
	}

	// Blocks use buffer-empty requests so the DTC fills SDR while the
	// preceding character is shifting. A lone CPU byte used transfer end.
	reg->SMR[pDev->TxCh] = (uint16_t)(reg->SMR[pDev->TxCh] | R9A02_SAU_SMR_MD0);

	uint16_t first = *(const uint16_t *)(uintptr_t)info->Sar;

	info->Sar = info->Sar + sizeof(uint16_t);
	info->Cra = info->Cra - 1U;
	// SDR stays empty until the first write below. DtcEnable clears the old
	// CPU request before arming; do not repeat full route cleanup here.
	if (info->Cra != 0U)
	{
		R9a02DtcEnable(pDev->IntTx);
	}
	else
	{
		R9a02EventFlagClear(pDev->IntTx);
	}
	reg->SDR[pDev->TxCh] = first;
}

// Called only when TX is idle, after its CPU event, or after Stop. Single
// bytes use the register directly with transfer-end notification; queued
// bursts use buffer-empty events and, when selected, the DTC. The running
// transfer owns the FIFO until it drains, as in STM32 and SAM4L.
static inline __attribute__((always_inline)) void R9a02UartTxKick(R9a02UartDev_t * const pDev, bool bDma)
{
	UARTDev_t *udev = pDev->pUartDev;

	if (bDma &&
		(((volatile R9a02DtcInfo_t *)&pDev->TxDtc)->Cra != 0U || CFifoUsed(udev->hTxFifo) > 1))
	{
		R9a02UartTxDma(pDev);
	}
	else
	{
		// An isolated byte waits until transfer end so the producer can
		// queue its successor. Bursts continue on buffer-empty events.
		int cnt = R9a02UartTxHw(pDev, bDma || CFifoUsed(udev->hTxFifo) == 1);

		if (cnt >= 0)
		{
			udev->bTxReady = cnt == 0;
		}
	}
}

// Poll completion only when a CPU event is pending. DTCE protects a live
// block even if its IR is briefly set during a DTC activation.
static inline __attribute__((always_inline)) void R9a02UartTxDmaPoll(R9a02UartDev_t * const pDev)
{
	if (!pDev->pUartDev->bTxReady)
	{
		uint32_t evt = R9A02_ICU->IELSR[pDev->IntTx];

		if ((evt & (R9A02_IELSR_IR | R9A02_IELSR_DTCE)) != R9A02_IELSR_IR)
		{
			return;
		}
		R9a02EventFlagClear(pDev->IntTx);
	}
	R9a02UartTxKick(pDev, true);
}

// Without interrupts, service both directions as the handlers would. Fill
// the shift register and SDR when ready, but never wait for a character on
// the wire. Read RX between TX writes so full duplex reception keeps going.
static void R9a02UartPoll(R9a02UartDev_t * const pDev)
{
	uint8_t err = 0;

	R9a02UartRxHw(pDev, &err);
	if (pDev->pUartDev->DevIntrf.bDma)
	{
		R9a02UartTxDmaPoll(pDev);
		R9a02UartRxHw(pDev, &err);

		return;
	}
	for (int i = 0; i < 2; i++)
	{
		if (R9a02UartTxHw(pDev) <= 0)
		{
			break;
		}
		R9a02UartRxHw(pDev, &err);
	}
}

// After the channels stopped and started again: SDR lost what it held and
// no buffer empty interrupt follows, the next byte of the TX CFIFO starts
// the transmission again. Called with interrupts disabled.
static void R9a02UartTxResume(R9a02UartDev_t * const pDev)
{
	UARTDev_t *udev = pDev->pUartDev;

	if (udev->DevIntrf.bDma)
	{
		R9a02UartTxKick(pDev, true);
	}
	else if (udev->DevIntrf.bIntEn)
	{
		udev->bTxReady = R9a02UartTxHw(pDev) <= 0;
	}
}

// The TX CFIFO sent and the last character out, before the channels stop.
// At most the time of the bytes queued and two more, counted in CPU cycles:
// when the TX interrupt cannot run (interrupts disabled, or a call from an
// interrupt at its level or above) the CFIFO does not empty and the wait
// ends there.
static void R9a02UartTxWait(R9a02UartDev_t * const pDev)
{
	UARTDev_t *udev = pDev->pUartDev;
	R9A02_SAU_Type *reg = pDev->pReg;
	uint32_t rate = udev->Rate > 0 ? (uint32_t)udev->Rate : 1U;
	uint32_t staged = udev->DevIntrf.bDma ? R9A02_UART_DMA_SIZE : 0U;
	uint32_t limit = (SystemCoreClock / rate + 1U) * 12U *
					 ((uint32_t)CFifoUsed(udev->hTxFifo) + staged + 2U);
	uint32_t start;

	__asm volatile ("csrr %0, mcycle" : "=r"(start));

	while (((reg->SSR[pDev->TxCh] & R9A02_SAU_SSR_TSF) || !udev->bTxReady) &&
		   (reg->SE & (1U << pDev->TxCh)) != 0U)
	{
		uint32_t now;

		if (udev->DevIntrf.bDma)
		{
			uintptr_t state = DisableInterrupt();

			R9a02UartTxDma(pDev);
			EnableInterrupt(state);
		}
		__asm volatile ("csrr %0, mcycle" : "=r"(now));
		if ((uint32_t)(now - start) >= limit)
		{
			break;
		}
	}
}

// Receive channel back to normal operation, figure 23.41 steps 10 to 12:
// stopped, SWC and SSEC at 0, normal STCLK, both channels started again
// unless the UART was disabled meanwhile. Its interrupts are back on
// without the requests left from Snooze. Called with interrupts disabled.
static void R9a02UartSnzRestore(R9a02UartDev_t * const pDev)
{
	R9a02UartSnz_t *snz = &s_R9a02UartSnz;
	R9A02_SAU_Type *reg = pDev->pReg;
	uint32_t rx = pDev->TxCh + 1U;
	bool run = (reg->SE & (1U << rx)) != 0U;

	reg->ST = (uint16_t)(1U << rx);
	reg->SSC = 0U;
	reg->SDR[rx] = snz->SdrNormal;
	snz->bArmed = false;

	R9a02ClearPendingInt(pDev->IntRx);
	R9a02ClearPendingInt(pDev->IntErr);
	R9a02IntEnable(pDev->IntRx, true);
	R9a02IntEnable(pDev->IntErr, true);

	if (run)
	{
		R9a02UartStart(pDev);
		R9a02UartTxResume(pDev);
	}
}

// Snooze ended by the reception end (SELSR0, R9a02SnoozeSet). The
// character is read while SSCm.SWC is still 1 (figure 23.37 note 1): its
// error flags are not set (SSEC = 1) and with SWC = 1 the manual does not
// say that BFF follows, the reception end alone tells that it is there.
// The receive channel goes back to normal operation at once, for the next
// character, before UART_EVT_RXDATA.
static void R9a02UartSnzWake(void *pCtx)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pCtx;
	R9a02UartSnz_t *snz = &s_R9a02UartSnz;
	UARTDev_t *udev = dev->pUartDev;
	uintptr_t state = DisableInterrupt();
	bool put = false;

	if (snz->bArmed && snz->pDev == dev)
	{
		uint8_t d = (uint8_t)(dev->pReg->SDR[dev->TxCh + 1U] & dev->RxDataMask);

		R9a02UartSnzRestore(dev);

		put = CFifoPutByte(udev->hRxFifo, d);
		if (put)
		{
			udev->bRxReady = true;
		}
		else
		{
			udev->RxDropCnt++;
		}
	}

	EnableInterrupt(state);

	if (put && udev->EvtCallback)
	{
		udev->EvtCallback(udev, UART_EVT_RXDATA, NULL, CFifoUsed(udev->hRxFifo));
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

// The callback is outside the byte path, so its call state is not saved
// around every SDR write.
__attribute__((noinline)) static void R9a02UartTxNotify(UARTDev_t * const pDev)
{
	if (pDev->DevIntrf.bIntEn && pDev->EvtCallback)
	{
		pDev->EvtCallback(pDev, UART_EVT_TXREADY, NULL, CFifoAvail(pDev->hTxFifo));
	}
}

// Byte service has no DMA selection or descriptor state. Keep the hardware
// guards: a higher-priority RX callback can stop or restart TX after the
// dispatcher acknowledges this event and before this handler runs.
static void R9a02UartTxIRQHandler(int IntNo, void *pCtx)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;
	uintptr_t state = DisableInterrupt();

	// The isolated startup byte used transfer end. Subsequent bytes
	// fill SDR ahead of the shift register through buffer-empty events.
	R9A02_SAU_Type *reg = dev->pReg;

	reg->SMR[dev->TxCh] = (uint16_t)(reg->SMR[dev->TxCh] | R9A02_SAU_SMR_MD0);

	int cnt = R9a02UartTxHw(dev);

	if (cnt >= 0)
	{
		udev->bTxReady = cnt == 0;
	}
	EnableInterrupt(state);

	if (cnt == 0 && udev->bTxReady)
	{
		R9a02UartTxNotify(udev);
	}
}

static void R9a02UartTxDmaIRQHandler(int IntNo, void *pCtx)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pCtx;
	UARTDev_t *udev = dev->pUartDev;
	uintptr_t state = DisableInterrupt();
	uint8_t err = 0;

	// This IRQ also runs with bIntMode false. Service the one-byte RX
	// buffer around TX work, as in the polling path; foreground polling
	// cannot run while the next DMA block is staged here.
	if (!udev->DevIntrf.bIntEn)
	{
		R9a02UartRxHw(dev, &err);
	}

	R9a02UartTxKick(dev, true);

	if (!udev->DevIntrf.bIntEn)
	{
		R9a02UartRxHw(dev, &err);
	}

	EnableInterrupt(state);

	if (udev->bTxReady && udev->DevIntrf.bIntEn && udev->EvtCallback)
	{
		udev->EvtCallback(udev, UART_EVT_TXREADY, NULL, CFifoAvail(udev->hTxFifo));
	}
}

static uint32_t R9a02UartGetRate(DevIntrf_t * const pDev)
{
	return ((R9a02UartDev_t *)pDev->pDevData)->pUartDev->Rate;
}

// Operation clock of table 23.95 for reception in Snooze: PCLKB the HOCO at
// 24 or 32 MHz without divider, fMCK = PCLKB / 2^5 at 4800, PCLKB / 2^4 at
// 9600, STCLK 79 or 106 in Snooze. In normal operation the STCLK closest to
// the rate (9615 and 4807.7 bps). The TX CFIFO is sent first, the channels
// are stopped and started again when they ran.
static bool R9a02UartSnoozeClock(R9a02UartDev_t * const pDev, uint32_t Rate)
{
	uint32_t src = R9a02SauSnoozeSrcHz();

	// The rate kept by the driver is the one set, 9615 for 9600: 2 % around
	// 4800 and 9600 selects them
	if (Rate >= 4704U && Rate <= 4896U)
	{
		Rate = 4800U;
	}
	else if (Rate >= 9408U && Rate <= 9792U)
	{
		Rate = 9600U;
	}
	else
	{
		return false;
	}

	if ((R9A02_SYSC_SCKSCR & R9A02_SCKSCR_CKSEL_Msk) != R9A02_SCKSCR_HOCO || SystemPeriphClockGet(0) != src ||
		(src != 24000000UL && src != 32000000UL))
	{
		return false;
	}

	R9A02_SAU_Type *reg = pDev->pReg;
	uint16_t chmsk = (uint16_t)(3U << pDev->TxCh);
	R9a02SauClk_t clk;

	R9a02UartTxWait(pDev);

	uintptr_t state = DisableInterrupt();
	bool running = (reg->SE & chmsk) != 0U;

	if (running)
	{
		R9a02UartStop(pDev);
	}

	bool ok = R9a02SauSetPrs(pDev->Unit, chmsk, Rate == 4800U ? 5U : 4U, &clk);

	if (ok)
	{
		uint32_t div = (clk.FmckHz + Rate) / (2U * Rate);

		R9a02UartSnz_t *snz = &s_R9a02UartSnz;

		snz->SdrNormal = (uint16_t)((div - 1U) << R9A02_SAU_SDR_STCLK_Pos);
		snz->SdrSnz = (uint16_t)((src == 32000000UL ? 106U : 79U) << R9A02_SAU_SDR_STCLK_Pos);
		for (uint32_t ch = pDev->TxCh; ch < pDev->TxCh + 2U; ch++)
		{
			reg->SMR[ch] = (uint16_t)((reg->SMR[ch] & ~R9A02_SAU_SMR_CKS) | clk.Cks);
			reg->SDR[ch] = snz->SdrNormal;
		}
		pDev->FmckHz = clk.FmckHz;
		pDev->pUartDev->Rate = (int)(clk.FmckHz / (2U * div));
	}

	if (running)
	{
		R9a02UartStart(pDev);
		R9a02UartTxResume(pDev);
	}

	EnableInterrupt(state);

	return ok;
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

	// With reception in Snooze the rate stays on the clock of table 23.95
	if (dev->pSnz != NULL)
	{
		return dev->pSnz->Clock(dev, Rate) ? (uint32_t)dev->pUartDev->Rate : 0U;
	}

	uintptr_t state = DisableInterrupt();

	// SPS, SMR and STCLK are written with the channels stopped
	bool running = (reg->SE & chmsk) != 0U;

	if (running)
	{
		R9a02UartStop(dev);
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
		R9a02UartTxResume(dev);
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
	bool held = !poll && CFifoIsBlocking(fifo);
	int cnt = 0;

	if (pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

	// A blocking FIFO keeps the inspected bytes until the copy is released,
	// so RX can interrupt the copy. A non-blocking producer can overwrite
	// old bytes: keep its copy protected. Polling services both directions.
	while (cnt < Bufflen)
	{
		int l = Bufflen - cnt < R9A02_UART_COPY_MAX ? Bufflen - cnt : R9A02_UART_COPY_MAX;
		uintptr_t state = 0;

		if (!held)
		{
			state = DisableInterrupt();
		}

		if (poll)
		{
			R9a02UartPoll(dev);
		}

		uint8_t *p = held ? CFifoPeekMultiple(fifo, &l) : CFifoGetMultiple(fifo, &l);

		if (p != NULL)
		{
			memcpy(pBuff + cnt, p, l);
			if (held)
			{
				CFifoRelease(fifo, l);
			}
		}
		if (!held)
		{
			EnableInterrupt(state);
		}

		if (p == NULL)
		{
			break;
		}
		cnt += l;
	}

	if (poll)
	{
		uintptr_t state = DisableInterrupt();

		R9a02UartPoll(dev);
		EnableInterrupt(state);
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

// Reserve, fill, then publish a blocking FIFO span. DeviceIntrf serializes
// the producer; the TX interrupt cannot read it before publication. Polling
// and non-blocking FIFOs keep the protected, short-copy path.
__attribute__((noinline)) static int R9a02UartTxPut(R9a02UartDev_t * const pDev, uint8_t const *pData, int Datalen)
{
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;
	bool poll = !pDev->pUartDev->DevIntrf.bIntEn;
	bool held = !poll && CFifoIsBlocking(fifo);
	int cnt = 0;

	while (Datalen > 0)
	{
		int l = held || Datalen < R9A02_UART_COPY_MAX ? Datalen : R9A02_UART_COPY_MAX;
		uintptr_t state = 0;

		if (!held)
		{
			state = DisableInterrupt();
		}
		if (poll)
		{
			R9a02UartPoll(pDev);
		}
		uint8_t *p = held ? CFifoResvMultiple(fifo, &l) : CFifoPutMultiple(fifo, &l);

		if (p != NULL)
		{
			memcpy(p, pData, l);
			if (held)
			{
				CFifoPutMultiple(fifo, &l);
			}
		}
		if (!held)
		{
			EnableInterrupt(state);
		}

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

// Init selects the byte or DMA entry. Compile the same enqueue logic with
// a fixed mode so byte writes do not carry DMA calls and their saved state.
static inline __attribute__((always_inline)) int R9a02UartTxDataMode(DevIntrf_t * const pDev,
		uint8_t const *pData, int Datalen, bool bDma)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	int cnt = 0;

	if (pData == NULL || Datalen <= 0)
	{
		return 0;
	}

	// Keep the one-byte producer short, as in the STM32 port. Buffer
	// copies use TxPut so their loop state is not saved for every byte.
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
		// DTC completion has a CPU route even in polling mode. Accepted
		// writes to a busy blocking FIFO leave TX to that route, just as
		// in interrupt mode. A full FIFO still polls, so a masked TX IRQ
		// cannot prevent progress. Non-blocking FIFOs always poll since
		// their puts never report full. Check RX on every call.
		if (!bDma || udev->bTxReady || cnt == 0 || !CFifoIsBlocking(udev->hTxFifo) ||
			(dev->pReg->SSR[dev->TxCh + 1U] & (R9A02_SAU_SSR_BFF | R9A02_SAU_SSR_ERR_Msk)))
		{
			uintptr_t state = DisableInterrupt();

			R9a02UartPoll(dev);
			EnableInterrupt(state);
		}
	}
	else if (udev->bTxReady)
	{
		uintptr_t state = DisableInterrupt();

		if (udev->bTxReady)
		{
			R9a02UartTxKick(dev, bDma);
		}
		EnableInterrupt(state);
	}

	return cnt;
}

static int R9a02UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	return R9a02UartTxDataMode(pDev, pData, Datalen, false);
}

static int R9a02UartTxDmaData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	return R9a02UartTxDataMode(pDev, pData, Datalen, true);
}

static void R9a02UartStopTx(DevIntrf_t * const pDev)
{
}

// Software Standby hook of the reception in Snooze. Before WFI, figure 23.41
// steps 1 to 3: the transmission and a reception in progress end, both
// channels stop, the error flags are cleared and SDR read (SSEC note of
// 23.6.3), Snooze STCLK, SWC and SSEC, then the receive channel starts. Its
// interrupts stay off while it is set for Snooze, the Snooze end interrupt
// takes the character. After a wake-up by another source, steps 10 to 12.
static void R9a02UartSnzHook(bool bEnter, void *pCtx)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pCtx;
	R9a02UartSnz_t *snz = &s_R9a02UartSnz;
	R9A02_SAU_Type *reg = dev->pReg;
	uint32_t rx = dev->TxCh + 1U;
	uint16_t rxbit = (uint16_t)(1U << rx);

	if (!bEnter)
	{
		uintptr_t state = DisableInterrupt();

		if (snz->bArmed && snz->pDev == dev)
		{
			R9a02UartSnzRestore(dev);
		}

		EnableInterrupt(state);

		return;
	}

	// Only for an enabled UART whose Snooze set up is still the current one
	if (dev->pSnz == NULL || (reg->SE & rxbit) == 0U || !R9a02SnoozeIsSet(R9a02UartSnzWake, dev))
	{
		return;
	}

	R9a02UartTxWait(dev);

	uint32_t rate = dev->pUartDev->Rate > 0 ? (uint32_t)dev->pUartDev->Rate : 1U;
	uint32_t n = (SystemCoreClock / rate + 1U) * 24U + 1000U;

	while ((reg->SSR[rx] & R9A02_SAU_SSR_TSF) && --n > 0U)
	{
	}

	uintptr_t state = DisableInterrupt();
	uint8_t err = 0;

	// Checked again: an interrupt during the waits may have turned Snooze
	// off or disabled the UART
	if (dev->pSnz == NULL || (reg->SE & rxbit) == 0U || !R9a02SnoozeIsSet(R9a02UartSnzWake, dev))
	{
		EnableInterrupt(state);

		return;
	}

	// A character already received goes to the RX CFIFO first, its pending
	// interrupt is dropped so that it is not taken for the character of the
	// Snooze
	int cnt = R9a02UartRxHw(dev, &err);

	R9a02IntEnable(dev->IntRx, false);
	R9a02IntEnable(dev->IntErr, false);
	R9a02ClearPendingInt(dev->IntRx);
	R9a02ClearPendingInt(dev->IntErr);

	R9a02UartStop(dev);
	reg->SIR[rx] = R9A02_SAU_SIR_ALL;
	(void)reg->SDR[rx];
	reg->SDR[rx] = snz->SdrSnz;
	reg->SSC = R9A02_SAU_SSC_SWC | R9A02_SAU_SSC_SSEC;
	snz->bArmed = true;
	reg->SS = rxbit;

	EnableInterrupt(state);

	// An answer sent from here waits in the TX CFIFO until the wake-up
	UARTDev_t *udev = dev->pUartDev;

	if (udev->EvtCallback != NULL)
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

// Snooze set up of the UART removed, the receive channel back to normal
// operation when it was set for Snooze. The Snooze registers are cleared
// only when the set up is still the UART's.
static void R9a02UartSnoozeOff(R9a02UartDev_t * const pDev)
{
	R9a02UartSnz_t *snz = &s_R9a02UartSnz;
	uintptr_t state = DisableInterrupt();

	if (snz->bArmed && snz->pDev == pDev)
	{
		R9a02UartSnzRestore(pDev);
	}

	EnableInterrupt(state);

	R9a02StandbyHookSet(R9a02UartSnzHook, pDev, false);
	if (R9a02SnoozeIsSet(R9a02UartSnzWake, pDev))
	{
		R9a02SnoozeSet(NULL);
	}
	pDev->pSnz = NULL;
	if (snz->pDev == pDev)
	{
		snz->pDev = NULL;
	}
}

static const R9a02UartSnzOps_t s_R9a02UartSnzOps = {
	R9a02UartSnoozeClock,
	R9a02UartSnoozeOff,
};

bool R9a02UartSnooze(DevIntrf_t * const pDev, bool bEnable)
{
	if (pDev == NULL || pDev->Type != DEVINTRF_TYPE_UART)
	{
		return false;
	}

	R9a02UartDev_t *dev = NULL;

	// A SAU UART, not a UARTA
	for (int i = 0; i < s_NbR9a02UartDev; i++)
	{
		if (pDev->pDevData == &s_R9a02UartDev[i] && s_R9a02UartDev[i].pUartDev != NULL &&
			&s_R9a02UartDev[i].pUartDev->DevIntrf == pDev)
		{
			dev = &s_R9a02UartDev[i];
		}
	}

	if (dev == NULL)
	{
		return false;
	}

	if (!bEnable)
	{
		if (dev->pSnz != NULL)
		{
			dev->pSnz->Off(dev);
		}

		return true;
	}

	// UART0 and UART2 only, receive interrupt
	if ((dev->DevNo != 0 && dev->DevNo != 2) || !pDev->bIntEn)
	{
		return false;
	}

	// One UART at a time: the other one is turned off
	R9a02UartDev_t *prev = s_R9a02UartSnz.pDev;

	if (prev != NULL && prev != dev && prev->pSnz != NULL)
	{
		prev->pSnz->Off(prev);
	}

	// Clock of table 23.95
	if (!R9a02UartSnoozeClock(dev, (uint32_t)dev->pUartDev->Rate))
	{
		return false;
	}

	R9a02SnoozeCfg_t cfg = {
		.Request = dev->Unit ? R9A02_SNZREQ_RXD2 : R9A02_SNZREQ_RXD0,
		.End = (uint16_t)(dev->Unit ? R9A02_SNZEND_SAU1_ERR : R9A02_SNZEND_SAU0_ERR),
		.WakeEvt = dev->EvtRx,
		.bDtc = false,
		.Handler = R9a02UartSnzWake,
		.pCtx = dev,
		.IntPrio = pDev->IntPrio,
	};

	if (!R9a02StandbyHookSet(R9a02UartSnzHook, dev, true))
	{
		return false;
	}
	if (!R9a02SnoozeSet(&cfg))
	{
		R9a02StandbyHookSet(R9a02UartSnzHook, dev, false);

		return false;
	}

	s_R9a02UartSnz.pDev = dev;
	s_R9a02UartSnz.bArmed = false;
	dev->pSnz = &s_R9a02UartSnzOps;

	return true;
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

	R9a02UartStop(dev);
	dev->TxDtc.Cra = 0U;
	CFifoFlush(udev->hTxFifo);
	udev->bTxReady = true;

	R9a02UartStart(dev);

	EnableInterrupt(state);
}

// Interrupts released, pins disconnected, the SAU stopped when none of its
// channels runs. A new UARTInit is needed to use it again.
static void R9a02UartPowerOff(DevIntrf_t * const pDev)
{
	R9a02UartDev_t *dev = (R9a02UartDev_t *)pDev->pDevData;

	if (dev->pSnz != NULL)
	{
		dev->pSnz->Off(dev);
	}

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

	if (pCfg->Mode != UART_MODE_UART || pCfg->Duplex != UART_DUPLEX_FULL ||
		pCfg->bIrDAMode || pCfg->FlowControl != UART_FLWCTRL_NONE ||
		(pCfg->DataBits != 7 && pCfg->DataBits != 8) || pCfg->Parity == UART_PARITY_MARK ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) || pCfg->Rate <= 0 ||
		((pCfg->bIntMode || pCfg->bDMAMode) && (pCfg->IntPrio < 0 || pCfg->IntPrio > (int)R9A02_CLIC_PRIO_MAX)))
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

	// A Snooze set up of a previous UARTInit is removed
	if (dev->pSnz != NULL)
	{
		dev->pSnz->Off(dev);
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

	// Pin configuration passes through GPIO output before selecting SAU.
	// Preload the idle level so the reset PODR value cannot drive TX low.
	const IOPinCfg_t *txpin = &dev->pIOPinMap[UARTPIN_TX_IDX];

	IOPinSet(txpin->PortNo, txpin->PinNo);
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
	pDev->DevIntrf.GetHandle = R9a02UartGetHandle;
	pDev->DevIntrf.Disable = R9a02UartDisable;
	pDev->DevIntrf.Enable = R9a02UartEnable;
	pDev->DevIntrf.GetRate = R9a02UartGetRate;
	pDev->DevIntrf.SetRate = R9a02UartSetRate;
	pDev->DevIntrf.StartRx = R9a02UartStartRx;
	pDev->DevIntrf.RxData = R9a02UartRxData;
	pDev->DevIntrf.StopRx = R9a02UartStopRx;
	pDev->DevIntrf.StartTx = R9a02UartStartTx;
	pDev->DevIntrf.TxData = pCfg->bDMAMode ? R9a02UartTxDmaData : R9a02UartTxData;
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
		dev->IntTx = R9a02RegisterIntHandler(dev->EvtTx, txprio,
				pCfg->bDMAMode ? R9a02UartTxDmaIRQHandler : R9a02UartTxIRQHandler, dev);
	}
	else if (pCfg->bDMAMode)
	{
		// The DTC activation source must be enabled in the CLIC (14.5).
		dev->IntTx = R9a02RegisterIntHandler(dev->EvtTx, pCfg->IntPrio, R9a02UartTxDmaIRQHandler, dev);
	}

	bool ok = !pCfg->bIntMode || (dev->IntRx >= 0 && dev->IntErr >= 0 && dev->IntTx >= 0);

	if (pCfg->bDMAMode)
	{
		memset(&dev->TxDtc, 0, sizeof(dev->TxDtc));
		dev->TxDtc.Mra = R9A02_DTC_MRA_MD_NORMAL | R9A02_DTC_MRA_SZ_HALF | R9A02_DTC_MRA_SM_INC;
		dev->TxDtc.Dar = (uint32_t)(uintptr_t)&reg->SDR[dev->TxCh];
		ok = ok && R9a02DtcSet(dev->IntTx, &dev->TxDtc);
	}
	if (!ok)
	{
		R9a02UartReleaseInts(dev);
		IOPinDis(dev->pIOPinMap, dev->NbIOPins);
		R9a02SauRelease(dev->Unit, chmsk, dev);
		EnableInterrupt(state);

		return false;
	}

	R9a02UartEnable(&pDev->DevIntrf);

	EnableInterrupt(state);

	return true;
}
