/**-------------------------------------------------------------------------
@file	uart_nrfx.cpp

@brief	Nordic nRF UART implementation

One implementation for every nRF family. The register layout changed with the
nRF54 generation and the peripheral set varies per part, so nothing here keys
on a part name. The MDK headers already declare what the target has and this
file reads those declarations:

	UART_PRESENT / UARTE_PRESENT		legacy byte mode, DMA mode
	UARTE_DMA_RX_PTR_PTR_Msk		DMA register block, nRF54 generation
	UARTE_CONFIG_PARITYTYPE_Msk		odd / even parity selection
	UARTE_CONFIG_STOP_Msk			stop bit selection
	UARTE_CONFIG_FRAMESIZE_Msk		4 to 9 bit data frame
	UARTE_EVENTS_FRAMETIMEOUT_...		frame timeout event
	UARTEnn_CORE_FREQUENCY			baud generator clock, per instance

Only the instance table below names parts, because the DevNo ordering is a
choice of this driver rather than something the MDK can state.

Verified against MDK for nRF52, nRF53, nRF91, nRF54L, nRF54H.

DevNo mapping

	nRF52832	0 UART0
	nRF52840	0 UART0, 1 UARTE1
	nRF5340 App	0 UARTE0, 1 UARTE1, 2 UARTE2, 3 UARTE3
	nRF5340 Net	0 UARTE0
	nRF9160		0 UARTE0, 1 UARTE1, 2 UARTE2, 3 UARTE3
	nRF9120		0 UARTE0, 1 UARTE1, 2 UARTE2, 3 UARTE3
	nRF54L15	0 UARTE30, 1 UARTE20, 2 UARTE21, 3 UARTE22, 4 UARTE00
	nRF54LM20A	0 UARTE30, 1 UARTE20, 2 UARTE21, 3 UARTE22,
			4 UARTE23, 5 UARTE24, 6 UARTE00
	nRF54H20 App	0 UARTE136, 1 UARTE130, 2 UARTE131, 3 UARTE132,
			4 UARTE133, 5 UARTE134, 6 UARTE135, 7 UARTE137,
			8 UARTE120

@author	Hoang Nguyen Hoan
@date	Aug. 30, 2015

@license

MIT License

Copyright (c) 2015, I-SYST inc., all rights reserved

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
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include "nrf.h"
#include "nrf_peripherals.h"

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "coredev/shared_intrf.h"

//
// Capabilities of the target, taken from the MDK headers. No part name.
//

// nRF54 generation moved the EasyDMA pointers into a DMA register block and
// renamed the start/stop tasks and the end events with them.
#ifdef UARTE_DMA_RX_PTR_PTR_Msk
#define NRFX_UART_DMAREG			1
#else
#define NRFX_UART_DMAREG			0
#endif

#ifdef UARTE_CONFIG_PARITYTYPE_Msk
#define NRFX_UART_PARITYTYPE		1
#else
#define NRFX_UART_PARITYTYPE		0
#endif

#ifdef UARTE_CONFIG_STOP_Msk
#define NRFX_UART_STOPMODE			1
#else
#define NRFX_UART_STOPMODE			0
#endif

#ifdef UARTE_CONFIG_FRAMESIZE_Msk
#define NRFX_UART_FRAMESIZE			1
#else
#define NRFX_UART_FRAMESIZE			0
#endif

#ifdef UARTE_EVENTS_FRAMETIMEOUT_EVENTS_FRAMETIMEOUT_Msk
#define NRFX_UART_FRAMETIMEOUT		1
#else
#define NRFX_UART_FRAMETIMEOUT		0
#endif

// UARTE shares a vector with SPIM and TWIM on these targets, so the vector
// lives in shared_intrf_nrfx.cpp and dispatches on DevNo. On nRF52 and nRF51
// the UART has a vector of its own, defined at the bottom of this file.
#if NRFX_UART_DMAREG || defined(NRF91_SERIES) || defined(NRF53_SERIES)
#define NRFX_UART_SHAREDIRQ			1
#else
#define NRFX_UART_SHAREDIRQ			0
#endif

//
// Register naming. The only place in this file where the two generations are
// spelled differently.
//
#if NRFX_UART_DMAREG

#define NRFX_UART_TASK_STARTRX(r)	(r)->TASKS_DMA.RX.START
#define NRFX_UART_TASK_STOPRX(r)	(r)->TASKS_DMA.RX.STOP
#define NRFX_UART_TASK_STARTTX(r)	(r)->TASKS_DMA.TX.START
#define NRFX_UART_TASK_STOPTX(r)	(r)->TASKS_DMA.TX.STOP
#define NRFX_UART_EVT_ENDRX(r)		(r)->EVENTS_DMA.RX.END
#define NRFX_UART_EVT_ENDTX(r)		(r)->EVENTS_DMA.TX.END
#define NRFX_UART_RXD_PTR(r)		(r)->DMA.RX.PTR
#define NRFX_UART_RXD_MAXCNT(r)		(r)->DMA.RX.MAXCNT
#define NRFX_UART_RXD_AMOUNT(r)		(r)->DMA.RX.AMOUNT
#define NRFX_UART_TXD_PTR(r)		(r)->DMA.TX.PTR
#define NRFX_UART_TXD_MAXCNT(r)		(r)->DMA.TX.MAXCNT
#define NRFX_UART_INTEN_ENDRX		UARTE_INTENSET_DMARXEND_Msk
#define NRFX_UART_INTEN_ENDTX		UARTE_INTENSET_DMATXEND_Msk

#else

#define NRFX_UART_TASK_STARTRX(r)	(r)->TASKS_STARTRX
#define NRFX_UART_TASK_STOPRX(r)	(r)->TASKS_STOPRX
#define NRFX_UART_TASK_STARTTX(r)	(r)->TASKS_STARTTX
#define NRFX_UART_TASK_STOPTX(r)	(r)->TASKS_STOPTX
#define NRFX_UART_EVT_ENDRX(r)		(r)->EVENTS_ENDRX
#define NRFX_UART_EVT_ENDTX(r)		(r)->EVENTS_ENDTX
#define NRFX_UART_RXD_PTR(r)		(r)->RXD.PTR
#define NRFX_UART_RXD_MAXCNT(r)		(r)->RXD.MAXCNT
#define NRFX_UART_RXD_AMOUNT(r)		(r)->RXD.AMOUNT
#define NRFX_UART_TXD_PTR(r)		(r)->TXD.PTR
#define NRFX_UART_TXD_MAXCNT(r)		(r)->TXD.MAXCNT
#define NRFX_UART_INTEN_ENDRX		UARTE_INTENSET_ENDRX_Msk
#define NRFX_UART_INTEN_ENDTX		UARTE_INTENSET_ENDTX_Msk

#endif

// Common bitfields for both DMA & non DMA error registers
#define NRFX_UART_ERRORSRC_BREAK_Pos (3UL) /*!< Position of BREAK field. */
#define NRFX_UART_ERRORSRC_BREAK_Msk (0x1UL << NRFX_UART_ERRORSRC_BREAK_Pos) /*!< Bit mask of BREAK field. */

/* Bit 2 : Framing error occurred */
#define NRFX_UART_ERRORSRC_FRAMING_Pos (2UL) /*!< Position of FRAMING field. */
#define NRFX_UART_ERRORSRC_FRAMING_Msk (0x1UL << NRFX_UART_ERRORSRC_FRAMING_Pos) /*!< Bit mask of FRAMING field. */

/* Bit 1 : Parity error */
#define NRFX_UART_ERRORSRC_PARITY_Pos (1UL) /*!< Position of PARITY field. */
#define NRFX_UART_ERRORSRC_PARITY_Msk (0x1UL << NRFX_UART_ERRORSRC_PARITY_Pos) /*!< Bit mask of PARITY field. */

/* Bit 0 : Overrun error */
#define NRFX_UART_ERRORSRC_OVERRUN_Pos (0UL) /*!< Position of OVERRUN field. */
#define NRFX_UART_ERRORSRC_OVERRUN_Msk (0x1UL << NRFX_UART_ERRORSRC_OVERRUN_Pos) /*!< Bit mask of OVERRUN field. */

// There is no indication in the datasheet about how many hardware fifo
// this value seems to produce best performance
#if NRFX_UART_DMAREG
#define NRFX_UART_HWFIFO_SIZE		8
#else
#define NRFX_UART_HWFIFO_SIZE		4
#endif

// Default fifo size if one is not provided in the config.
#define NRFX_UART_RXDMA_SIZE		(NRFX_UART_HWFIFO_SIZE)
#define NRFX_UART_TXDMA_SIZE		(4 * NRFX_UART_HWFIFO_SIZE)

#define NRFX_UART_CFIFO_SIZE		CFIFO_MEMSIZE(4 * NRFX_UART_HWFIFO_SIZE)

// Frame timeout in bit periods, used only where the peripheral has the event.
#define NRFX_UART_FRAMETIMEOUT_BITS	8

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct __nRF_UART_Dev {
	int DevNo;				// UART interface number
	union {
#ifdef UART_PRESENT
		NRF_UART_Type *pReg;		// UART registers
#endif
#ifdef UARTE_PRESENT
		NRF_UARTE_Type *pDmaReg;	// UART registers
#endif
	};
	IRQn_Type IrqNo;			// Vector this instance raises
	uint32_t CoreFreq;			// Baud generator clock in Hz
	UARTDev_t *pUartDev;			// Pointer to generic UART dev. data
	uint32_t RxPin;
	uint32_t TxPin;
	uint32_t CtsPin;
	uint32_t RtsPin;
	uint32_t RxDmaCnt;
	uint8_t RxFifoMem[NRFX_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[NRFX_UART_CFIFO_SIZE];
	uint8_t RxDmaMem[NRFX_UART_RXDMA_SIZE];
	uint8_t TxDmaMem[NRFX_UART_TXDMA_SIZE];
} nRFUartDev_t;
#pragma pack(pop)

// Instance table. The DevNo order is a choice of this driver, so this is the
// one block that names instances. Everything else is read from the MDK.
#if defined(NRF54L_SERIES)

// UARTE00 sits on the fast domain and is listed last so the general purpose
// instances keep the low DevNo values.
#ifdef NRF_UARTE23
#define NRFX_UART_DEVNO_00		6
#else
#define NRFX_UART_DEVNO_00		4
#endif

alignas(4) static nRFUartDev_t s_nRFxUARTDev[] = {
	{	// On P0
		.DevNo = 0,
		.pDmaReg = NRF_UARTE30,
		.IrqNo = UARTE30_IRQn,
		.CoreFreq = UARTE30_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// On P1
		.DevNo = 1,
		.pDmaReg = NRF_UARTE20,
		.IrqNo = UARTE20_IRQn,
		.CoreFreq = UARTE20_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// On P1
		.DevNo = 2,
		.pDmaReg = NRF_UARTE21,
		.IrqNo = UARTE21_IRQn,
		.CoreFreq = UARTE21_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// On P1
		.DevNo = 3,
		.pDmaReg = NRF_UARTE22,
		.IrqNo = UARTE22_IRQn,
		.CoreFreq = UARTE22_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#ifdef NRF_UARTE23
	{	// nRF54LM20A and up
		.DevNo = 4,
		.pDmaReg = NRF_UARTE23,
		.IrqNo = UARTE23_IRQn,
		.CoreFreq = UARTE23_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// nRF54LM20A and up
		.DevNo = 5,
		.pDmaReg = NRF_UARTE24,
		.IrqNo = UARTE24_IRQn,
		.CoreFreq = UARTE24_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#endif
	{	// On P2, fast domain
		.DevNo = NRFX_UART_DEVNO_00,
		.pDmaReg = NRF_UARTE00,
		.IrqNo = UARTE00_IRQn,
		.CoreFreq = UARTE00_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
};

#elif defined(NRF54H_SERIES)

// UARTE120 sits on the fast domain and is listed last, same reason as above.
alignas(4) static nRFUartDev_t s_nRFxUARTDev[] = {
	{
		.DevNo = 0,
		.pDmaReg = NRF_UARTE130,
		.IrqNo = UARTE130_IRQn,
		.CoreFreq = UARTE130_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 1,
		.pDmaReg = NRF_UARTE131,
		.IrqNo = UARTE131_IRQn,
		.CoreFreq = UARTE131_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 2,
		.pDmaReg = NRF_UARTE132,
		.IrqNo = UARTE132_IRQn,
		.CoreFreq = UARTE132_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 3,
		.pDmaReg = NRF_UARTE133,
		.IrqNo = UARTE133_IRQn,
		.CoreFreq = UARTE133_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 4,
		.pDmaReg = NRF_UARTE134,
		.IrqNo = UARTE134_IRQn,
		.CoreFreq = UARTE134_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 5,
		.pDmaReg = NRF_UARTE135,
		.IrqNo = UARTE135_IRQn,
		.CoreFreq = UARTE135_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 6,
		.pDmaReg = NRF_UARTE136,
		.IrqNo = UARTE136_IRQn,
		.CoreFreq = UARTE136_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 7,
		.pDmaReg = NRF_UARTE137,
		.IrqNo = UARTE137_IRQn,
		.CoreFreq = UARTE137_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// Fast domain, 320 MHz baud generator
		.DevNo = 8,
		.pDmaReg = NRF_UARTE120,
		.IrqNo = UARTE120_IRQn,
		.CoreFreq = UARTE120_CORE_FREQUENCY * 1000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
};

#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)

// These families predate the plain NRF_UARTEn alias, so the security state has
// to be picked here. The nRF54 tables above do not need this.
#if defined(NRF_TRUSTZONE_NONSECURE) || defined(NRF5340_XXAA_NETWORK)
#define NRFX_UART_REG(n)		NRF_UARTE##n##_NS
#else
#define NRFX_UART_REG(n)		NRF_UARTE##n##_S
#endif

alignas(4) static nRFUartDev_t s_nRFxUARTDev[] = {
	{
		.DevNo = 0,
		.pDmaReg = NRFX_UART_REG(0),
#ifdef NRF91_SERIES
		.IrqNo = UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn,
#else
		.IrqNo = SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn,
#endif
		.CoreFreq = 16000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#ifdef NRF_UARTE1_S
	{
		.DevNo = 1,
		.pDmaReg = NRFX_UART_REG(1),
#ifdef NRF91_SERIES
		.IrqNo = UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn,
#else
		.IrqNo = SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn,
#endif
		.CoreFreq = 16000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#endif
#ifdef NRF_UARTE2_S
	{
		.DevNo = 2,
		.pDmaReg = NRFX_UART_REG(2),
#ifdef NRF91_SERIES
		.IrqNo = UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn,
#else
		.IrqNo = SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn,
#endif
		.CoreFreq = 16000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#endif
#ifdef NRF_UARTE3_S
	{
		.DevNo = 3,
		.pDmaReg = NRFX_UART_REG(3),
#ifdef NRF91_SERIES
		.IrqNo = UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn,
#else
		.IrqNo = SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn,
#endif
		.CoreFreq = 16000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#endif
};

#else

alignas(4) static nRFUartDev_t s_nRFxUARTDev[] = {
	{
		.DevNo = 0,
		.pReg = NRF_UART0,
#ifdef NRF51
		.IrqNo = UART0_IRQn,
#else
		.IrqNo = UARTE0_UART0_IRQn,
#endif
		.CoreFreq = 16000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#ifdef NRF_UARTE1
	{
		.DevNo = 1,
		.pDmaReg = NRF_UARTE1,
		.IrqNo = UARTE1_IRQn,
		.CoreFreq = 16000000UL,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#endif
};

#endif

static const int s_NbUartDev = sizeof(s_nRFxUARTDev) / sizeof(nRFUartDev_t);

bool nRFUARTWaitForRxReady(nRFUartDev_t * const pDev, uint32_t Timeout)
{
#ifdef UART_PRESENT
	NRF_UART_Type *reg = pDev->pReg;
#else
	NRF_UARTE_Type *reg = pDev->pDmaReg;
#endif

	do {
		if (reg->EVENTS_RXDRDY || pDev->pUartDev->bRxReady)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool nRFUARTWaitForTxReady(nRFUartDev_t * const pDev, uint32_t Timeout)
{
#ifdef UART_PRESENT
	NRF_UART_Type *reg = pDev->pReg;
#else
	NRF_UARTE_Type *reg = pDev->pDmaReg;
#endif

	do {
		if (reg->EVENTS_TXDRDY || pDev->pUartDev->bTxReady == true)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

//static void UartIrqHandler(nRFUartDev_t * const pDev)
static void UartIrqHandler(int DevNo, DevIntrf_t * const pDev)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;
	int len = 0;
	int cnt = 0;

#ifdef UART_PRESENT
	NRF_UART_Type *reg = dev->pReg;
#else
	NRF_UARTE_Type *reg = dev->pDmaReg;
#endif

#if NRFX_UART_FRAMETIMEOUT
	// A partial frame has gone quiet. Stop the receiver so the DMA end event
	// hands over what arrived instead of waiting for a full buffer.
	if (dev->pDmaReg->EVENTS_FRAMETIMEOUT)
	{
		dev->pDmaReg->EVENTS_FRAMETIMEOUT = 0;
		NRFX_UART_TASK_STOPRX(dev->pDmaReg) = 1;
	}
#endif

	if (reg->EVENTS_RXTO)
	{
#ifdef UARTE_PRESENT
		dev->pDmaReg->TASKS_FLUSHRX = 1;
#endif
		reg->EVENTS_RXTO = 0;
		if (dev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(dev->pUartDev->hRxFifo);
			cnt = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXTIMEOUT, NULL, len);
		}
	}

#ifdef UARTE_PRESENT
	if (NRFX_UART_EVT_ENDRX(dev->pDmaReg))
	{
		// No DMA support for RX, just clear the event
		dev->pDmaReg->EVENTS_RXDRDY = 0;
		NRFX_UART_EVT_ENDRX(dev->pDmaReg) = 0;
		dev->RxDmaCnt = 0;

		int l = NRFX_UART_RXD_AMOUNT(dev->pDmaReg);
		uint8_t *p = CFifoPutMultiple(dev->pUartDev->hRxFifo, &l);
		if (p)
		{
			memcpy(p, dev->RxDmaMem, l);
		}
		else
		{
			dev->pUartDev->RxDropCnt++;
		}

		dev->pUartDev->bRxReady = false;
		NRFX_UART_TASK_STARTRX(dev->pDmaReg) = 1;
	}
	else
#endif

	if (reg->EVENTS_RXDRDY)
	{
#ifdef UART_PRESENT
		uint8_t *d;

		if (pDev->bDma == false)
		{
			cnt = 0;
			dev->pUartDev->bRxReady = false;

			do {
				reg->EVENTS_RXDRDY = 0;

				d = CFifoPut(dev->pUartDev->hRxFifo);
				if (d == NULL)
				{
					dev->pUartDev->bRxReady = true;
					dev->pUartDev->RxDropCnt++;
					break;
				}
				*d = reg->RXD;
				cnt++;
			} while (reg->EVENTS_RXDRDY && cnt < NRFX_UART_HWFIFO_SIZE) ;
		}
		else
#endif
		{
			reg->EVENTS_RXDRDY = 0;
			dev->RxDmaCnt++;
		}
		if (dev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(dev->pUartDev->hRxFifo);
			cnt = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, len);
		}
	}


	if (reg->EVENTS_TXDRDY)
	{

		reg->EVENTS_TXDRDY = 0;
		cnt = 0;

#ifdef UART_PRESENT
		if (dev->pUartDev->DevIntrf.bDma == false)
		{
			do {
				dev->pReg->EVENTS_TXDRDY = 0;

				uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
				if (p == NULL)
				{
					dev->pUartDev->bTxReady = true;
					break;
				}
				dev->pUartDev->bTxReady = false;
				dev->pReg->TXD = *p;
				cnt++;
			} while (dev->pReg->EVENTS_TXDRDY && cnt < NRFX_UART_HWFIFO_SIZE);

			if (dev->pUartDev->EvtCallback)
			{
				len = CFifoAvail(dev->pUartDev->hTxFifo);
				len = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, len);
			}
		}
#endif
	}

#ifdef UARTE_PRESENT
	if (NRFX_UART_EVT_ENDTX(dev->pDmaReg) || dev->pDmaReg->EVENTS_TXSTOPPED)
	{
		NRFX_UART_EVT_ENDTX(dev->pDmaReg) = 0;
		dev->pDmaReg->EVENTS_TXSTOPPED = 0;

		int l = NRFX_UART_TXDMA_SIZE;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
		if (p)
		{
			dev->pUartDev->bTxReady = false;

			// Transfer to tx cache before sending as CFifo will immediately make the memory
			// block available for reuse in the Put request. This could cause an overwrite
			// if uart tx has not completed in time.
			memcpy(dev->TxDmaMem, p, l);

			NRFX_UART_TXD_MAXCNT(dev->pDmaReg) = l;
			NRFX_UART_TXD_PTR(dev->pDmaReg) = (uint32_t)dev->TxDmaMem;
			NRFX_UART_TASK_STARTTX(dev->pDmaReg) = 1;
		}
		else
		{
			dev->pUartDev->bTxReady = true;
		}
		if (dev->pUartDev->EvtCallback)
		{
			len = CFifoAvail(dev->pUartDev->hTxFifo);
			len = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, len);
		}
	}
#endif

	// Handle errors
	if (reg->EVENTS_ERROR)
	{
		uint32_t err = reg->ERRORSRC;
		reg->EVENTS_ERROR = 0;

		if (err & NRFX_UART_ERRORSRC_OVERRUN_Msk)
		{
			dev->pUartDev->RxOvrErrCnt++;
			len = 0;

#ifdef UARTE_PRESENT
			if (pDev->bDma == true)
			{
				NRFX_UART_TASK_STOPRX(dev->pDmaReg) = 1;
			}
			else
#endif
			{
#ifdef UART_PRESENT
				cnt = NRFX_UART_HWFIFO_SIZE;
				do {
					dev->pReg->EVENTS_RXDRDY = 0;
					uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
					if (p == NULL)
					{
						dev->pUartDev->bRxReady = true;
						break;
					}
					dev->pUartDev->bRxReady = false;
					*p = dev->pReg->RXD;
					cnt++;
				} while (dev->pReg->EVENTS_RXDRDY && --cnt > 0);
				if (dev->pUartDev->EvtCallback)
				{
					len = CFifoUsed(dev->pUartDev->hRxFifo);
					dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, len);
				}
#endif
			}
		}
		if (err & NRFX_UART_ERRORSRC_FRAMING_Msk)
		{
			dev->pUartDev->FramErrCnt++;
		}
		if (err & NRFX_UART_ERRORSRC_PARITY_Msk)
		{
			dev->pUartDev->ParErrCnt++;
		}
		if (err & NRFX_UART_ERRORSRC_BREAK_Msk)
		{
		}
		reg->ERRORSRC = err;
		len = 0;
		if (dev->pUartDev->EvtCallback)
		{
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, NULL, len);
		}
		NRFX_UART_TASK_STARTRX(reg) = 1;
	}

	if (reg->EVENTS_CTS)
	{
		reg->EVENTS_CTS = 0;
		dev->pUartDev->LineState &= ~UART_LINESTATE_CTS;
		if (dev->pUartDev->EvtCallback)
		{
			uint8_t buff = 0;
			len = 1;
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, &buff, len);
		}
	}

	if (reg->EVENTS_NCTS)
	{
		reg->EVENTS_NCTS = 0;
		dev->pUartDev->LineState |= UART_LINESTATE_CTS;
		if (dev->pUartDev->EvtCallback)
		{
			uint8_t buff = UART_LINESTATE_CTS;
			len = 1;
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, &buff, len);
		}
	}
}

static uint32_t nRFUARTGetRate(DevIntrf_t * const pDev)
{
	return ((nRFUartDev_t*)pDev->pDevData)->pUartDev->Rate;
}

static uint32_t nRFUARTSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;
	uint32_t f = dev->CoreFreq;

	if (f == 0 || Rate == 0)
	{
		return 0;
	}

	// NOTE: Thanks to hmolesworth for the Baudrate calculation posted on devzone.
	// https://devzone.nordicsemi.com/f/nordic-q-a/43280/technical-question-regarding-uart-baud-rate-generator-baudrate-register-offset-0x524/458422
	// BAUDRATE holds Rate * 2^32 / CoreFreq. CoreFreq is 16 MHz everywhere up
	// to nRF53, and per instance from the MDK on nRF54 and later, where the
	// fast domain instances run at 64, 128 or 320 MHz.

	uint32_t regval = (uint32_t)(((((uint64_t)Rate << 32ULL) + (uint64_t)(f >> 1)) / (uint64_t)f) + 0x800ULL) & 0xFFFFF000UL;

#ifdef UARTE_PRESENT
	dev->pDmaReg->BAUDRATE = regval;
#else
	dev->pReg->BAUDRATE = regval;
#endif

	return (uint32_t)(((uint64_t)regval * (uint64_t)f) >> 32ULL);
}

static bool nRFUARTStartRx(DevIntrf_t * const pSerDev, uint32_t DevAddr)
{
	return true;
}

static int nRFUARTRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Bufflen)
	{
		int l  = Bufflen;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &l);
		if (p == NULL)
		{
#if defined(UARTE_PRESENT) && !NRFX_UART_FRAMETIMEOUT
			// Without a frame timeout event the receiver has to be stopped by
			// hand to release a partially filled DMA buffer.
			if (pDev->bDma == true && CFifoUsed(dev->pUartDev->hRxFifo) <= 0)
			{
				if (dev->RxDmaCnt > 0)
				{
					NRFX_UART_TASK_STOPRX(dev->pDmaReg) = 1;
				}
			}
#endif
			break;
		}
		memcpy(pBuff, p, l);
		cnt += l;
		pBuff += l;
		Bufflen -= l;
	}
	EnableInterrupt(state);

	if (dev->pUartDev->bRxReady)
	{
#ifdef UARTE_PRESENT
		if (pDev->bDma == true)
		{
			dev->pUartDev->bRxReady = false;
			NRFX_UART_RXD_MAXCNT(dev->pDmaReg) = NRFX_UART_RXDMA_SIZE;
			NRFX_UART_RXD_PTR(dev->pDmaReg) = (uint32_t)dev->RxDmaMem;
			NRFX_UART_TASK_STARTRX(dev->pDmaReg) = 1;
		}
		else
#endif
		{
#ifdef UART_PRESENT
			uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
			if (p)
			{
				dev->pReg->EVENTS_RXDRDY = 0;
				dev->pUartDev->bRxReady = false;
				*p = dev->pReg->RXD;
			}
#endif
		}
	}

	return cnt;
}

static void nRFUARTStopRx(DevIntrf_t * const pDev)
{
}

static bool nRFUARTStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int nRFUARTTxData(DevIntrf_t * const pDev, const uint8_t *pData, int Datalen)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;
	int cnt = 0;
	int rtry = pDev->MaxRetry;

	while (Datalen > 0 && rtry-- > 0)
	{
		uint32_t state = DisableInterrupt();

		while (Datalen > 0)
		{
			int l = Datalen;
			uint8_t *p = CFifoPutMultiple(dev->pUartDev->hTxFifo, &l);
			if (p == NULL)
			{
				break;
			}
			memcpy(p, pData, l);
			Datalen -= l;
			pData += l;
			cnt += l;
		}
		EnableInterrupt(state);

		if (dev->pUartDev->bTxReady)
		{
#ifdef UARTE_PRESENT
			if (pDev->bDma == true)
			{
				int l = NRFX_UART_TXDMA_SIZE;
				uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
				if (p)
				{
					dev->pUartDev->bTxReady = false;

					// Transfer to tx cache before sending as CFifo will immediately make the memory
					// block available for reuse in the Put request. This could cause an overwrite
					// if uart tx has not completed in time.
					memcpy(dev->TxDmaMem, p, l);

					NRFX_UART_TXD_MAXCNT(dev->pDmaReg) = l;
					NRFX_UART_TXD_PTR(dev->pDmaReg) = (uint32_t)dev->TxDmaMem;
					NRFX_UART_TASK_STARTTX(dev->pDmaReg) = 1;
				}
			}
			else
#endif
			{
#ifdef UART_PRESENT
				dev->pReg->EVENTS_TXDRDY = 0;
				dev->pUartDev->bTxReady = true;
				uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
				if (p)
				{
					dev->pUartDev->bTxReady = false;
					dev->pReg->TXD = *p;
				}
#endif
			}
		}
	}

	// Datalen is what is left and cnt is what was accepted, both moved by the
	// same loop above, so the drop is Datalen. Taking cnt off it underflowed
	// the unsigned counter whenever more was accepted than remained, and the
	// old rtry test only ran when the FIFO refused, so with a large FIFO this
	// reported 0 no matter what was lost.
	if (Datalen > 0)
	{
		dev->pUartDev->TxDropCnt += (uint32_t)Datalen;
	}

	return cnt;
}

static void nRFUARTStopTx(DevIntrf_t * const pDev)
{
}

static void nRFUARTDisable(DevIntrf_t * const pDev)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;
#ifdef UART_PRESENT
	NRF_UART_Type *reg = dev->pReg;
	reg->TASKS_STOPRX = 1;
	reg->TASKS_STOPTX = 1;
#ifdef NRF52805_XXAA
	reg->PSEL.RXD = -1;
	reg->PSEL.TXD = -1;
	reg->PSEL.RTS = -1;
	reg->PSEL.CTS = -1;
#else
	reg->PSELRXD = -1;
	reg->PSELTXD = -1;
	reg->PSELRTS = -1;
	reg->PSELCTS = -1;
#endif
#else
	NRF_UARTE_Type *reg = dev->pDmaReg;
	NRFX_UART_TASK_STOPRX(reg) = 1;
	NRFX_UART_TASK_STOPTX(reg) = 1;
	reg->PSEL.RXD = -1;
	reg->PSEL.TXD = -1;
	reg->PSEL.RTS = -1;
	reg->PSEL.CTS = -1;
#endif
	reg->ENABLE = 0;
}

static void nRFUARTEnable(DevIntrf_t * const pDev)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;

	dev->pUartDev->RxOvrErrCnt = 0;
	dev->pUartDev->ParErrCnt = 0;
	dev->pUartDev->FramErrCnt = 0;
	dev->pUartDev->RxDropCnt = 0;
	dev->pUartDev->TxDropCnt = 0;
	dev->RxDmaCnt = 0;

	CFifoFlush(dev->pUartDev->hTxFifo);

#ifdef UARTE_PRESENT
	if (pDev->bDma == true)
	{
		dev->pDmaReg->PSEL.RXD = dev->RxPin;
		dev->pDmaReg->PSEL.TXD = dev->TxPin;
		dev->pDmaReg->PSEL.CTS = dev->CtsPin;
		dev->pDmaReg->PSEL.RTS = dev->RtsPin;

		dev->pDmaReg->ENABLE |= (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);
		NRFX_UART_RXD_MAXCNT(dev->pDmaReg) = NRFX_UART_RXDMA_SIZE;
		NRFX_UART_RXD_PTR(dev->pDmaReg) = (uint32_t)dev->RxDmaMem;
		NRFX_UART_EVT_ENDRX(dev->pDmaReg) = 0;
		NRFX_UART_TASK_STARTRX(dev->pDmaReg) = 1;
	}
	else
#endif
	{
#ifdef UART_PRESENT
#ifdef NRF52805_XXAA
		dev->pReg->PSEL.RXD = dev->RxPin;
		dev->pReg->PSEL.TXD = dev->TxPin;
		dev->pReg->PSEL.CTS = dev->CtsPin;
		dev->pReg->PSEL.RTS = dev->RtsPin;
#else
		dev->pReg->PSELRXD = dev->RxPin;
		dev->pReg->PSELTXD = dev->TxPin;
		dev->pReg->PSELCTS = dev->CtsPin;
		dev->pReg->PSELRTS = dev->RtsPin;
#endif
		dev->pReg->ENABLE |= (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
		dev->pReg->TASKS_STARTRX = 1;
		dev->pReg->TASKS_STARTTX = 1;
#endif
	}

	dev->pUartDev->bTxReady = true;
}

void nRFUARTPowerOff(DevIntrf_t * const pDev)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;

#if !NRFX_UART_DMAREG
#ifdef UART_PRESENT
	NRF_UART_Type *reg = dev->pReg;
#else
	NRF_UARTE_Type *reg = dev->pDmaReg;
#endif

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
	// The register does not exist on nRF54 and later.
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 0;
#endif

	if (dev->CtsPin != -1)
	{
		IOPinDisable(dev->CtsPin >> 5, dev->CtsPin & 0x1F);
	}
	if (dev->RtsPin != -1)
	{
		IOPinDisable(dev->RtsPin >> 5, dev->RtsPin & 0x1F);
	}
	if (dev->RxPin != -1)
	{
		IOPinDisable(dev->RxPin >> 5, dev->RxPin & 0x1F);
	}
	if (dev->TxPin != -1)
	{
		IOPinDisable(dev->TxPin >> 5, dev->TxPin & 0x1F);
	}
}

void *nRFUARTGetHandle(DevIntrf_t * const pDev)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;

	return dev->pUartDev;
}

static void apply_workaround_for_enable_anomaly(nRFUartDev_t * const pDev)
{
#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || defined(NRF91_SERIES)
	// Apply workaround for anomalies:
	// - nRF91 series - anomaly 23
	// - nRF5340 - anomaly 44
	volatile uint32_t const * rxenable_reg =
		(volatile uint32_t *)(((uint32_t)pDev->pDmaReg) + 0x564);
	volatile uint32_t const * txenable_reg =
		(volatile uint32_t *)(((uint32_t)pDev->pDmaReg) + 0x568);

	if (*txenable_reg == 1)
	{
		pDev->pDmaReg->TASKS_STOPTX = 1;
	}

	if (*rxenable_reg == 1)
	{
		pDev->pDmaReg->ENABLE = UARTE_ENABLE_ENABLE_Msk;
		pDev->pDmaReg->TASKS_STOPRX = 1;

		while (*rxenable_reg) {}

		pDev->pDmaReg->ERRORSRC = pDev->pDmaReg->ERRORSRC;

		pDev->pDmaReg->ENABLE = 0;
	}
#endif
}

#ifdef UARTE_PRESENT
// Compose the CONFIG value. Each field is written only where the MDK says the
// peripheral has it, so the same code serves every family.
static uint32_t nRFUARTCfgVal(const UARTCfg_t *pCfg)
{
	uint32_t cnf = 0;

#if NRFX_UART_FRAMESIZE
	uint32_t fsz = pCfg->DataBits;

	if (fsz < UARTE_CONFIG_FRAMESIZE_Min || fsz > UARTE_CONFIG_FRAMESIZE_Max)
	{
		fsz = UARTE_CONFIG_FRAMESIZE_8bit;
	}
	cnf |= (fsz << UARTE_CONFIG_FRAMESIZE_Pos) & UARTE_CONFIG_FRAMESIZE_Msk;
#endif

	if (pCfg->Parity == UART_PARITY_NONE)
	{
		cnf |= UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos;
	}
	else
	{
		cnf |= UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos;
#if NRFX_UART_PARITYTYPE
		cnf |= pCfg->Parity == UART_PARITY_ODD ?
					(UARTE_CONFIG_PARITYTYPE_Odd << UARTE_CONFIG_PARITYTYPE_Pos) :
					(UARTE_CONFIG_PARITYTYPE_Even << UARTE_CONFIG_PARITYTYPE_Pos);
#endif
	}

#if NRFX_UART_STOPMODE
	cnf |= pCfg->StopBits == 2 ?
				(UARTE_CONFIG_STOP_Two << UARTE_CONFIG_STOP_Pos) :
				(UARTE_CONFIG_STOP_One << UARTE_CONFIG_STOP_Pos);
#endif

	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		cnf |= UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos;
	}

#ifdef UARTE_CONFIG_ENDIAN_Msk
	cnf |= UARTE_CONFIG_ENDIAN_LSB << UARTE_CONFIG_ENDIAN_Pos;
#endif

#if NRFX_UART_FRAMETIMEOUT
	cnf |= UARTE_CONFIG_FRAMETIMEOUT_Enabled << UARTE_CONFIG_FRAMETIMEOUT_Pos;
#endif

	return cnf;
}
#endif

bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	// Config I/O pins
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->pIOPinMap == NULL || pCfg->NbIOPins <= 0)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbUartDev)
	{
		return false;
	}

	int devno = pCfg->DevNo;
#ifdef UART_PRESENT
	NRF_UART_Type *reg = s_nRFxUARTDev[devno].pReg;
#else
	NRF_UARTE_Type *reg = s_nRFxUARTDev[devno].pDmaReg;
#endif

#if !NRFX_UART_DMAREG
	// Force power on in case it was powered off previously. Undocumented
	// register, absent on nRF54 and later.
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;
#endif

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_nRFxUARTDev[devno].RxFifoMem, NRFX_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_nRFxUARTDev[devno].TxFifoMem, NRFX_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	IOPinCfg_t *pincfg = (IOPinCfg_t*)pCfg->pIOPinMap;

	IOPinSet(pincfg[UARTPIN_TX_IDX].PortNo, pincfg[UARTPIN_TX_IDX].PinNo);
	IOPinCfg(pincfg, pCfg->NbIOPins);

	IOPinSetStrength(pincfg[UARTPIN_TX_IDX].PortNo, pincfg[UARTPIN_TX_IDX].PinNo, IOPINSTRENGTH_STRONG);

	pDev->DevIntrf.pDevData = &s_nRFxUARTDev[devno];
	s_nRFxUARTDev[devno].pUartDev = pDev;

#ifndef UARTE_PRESENT
	pDev->DevIntrf.bDma = false;	// DMA not avail on nRF51

	s_nRFxUARTDev[devno].RxPin = pincfg[UARTPIN_RX_IDX].PinNo;
	s_nRFxUARTDev[devno].TxPin = pincfg[UARTPIN_TX_IDX].PinNo;
	s_nRFxUARTDev[devno].pReg->PSELRXD = pincfg[UARTPIN_RX_IDX].PinNo;
	s_nRFxUARTDev[devno].pReg->PSELTXD = pincfg[UARTPIN_TX_IDX].PinNo;

	s_nRFxUARTDev[devno].pReg->CONFIG &= ~(UART_CONFIG_PARITY_Msk << UART_CONFIG_PARITY_Pos);
	if (pCfg->Parity == UART_PARITY_NONE)
	{
		s_nRFxUARTDev[devno].pReg->CONFIG |= UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos;
	}
	else
	{
		s_nRFxUARTDev[devno].pReg->CONFIG |= UART_CONFIG_PARITY_Included << UART_CONFIG_PARITY_Pos;
	}

	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		s_nRFxUARTDev[devno].pReg->CONFIG |= (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
		s_nRFxUARTDev[devno].CtsPin = pincfg[UARTPIN_CTS_IDX].PinNo;
		s_nRFxUARTDev[devno].RtsPin = pincfg[UARTPIN_RTS_IDX].PinNo;
		s_nRFxUARTDev[devno].pReg->PSELCTS = s_nRFxUARTDev[devno].CtsPin;
		s_nRFxUARTDev[devno].pReg->PSELRTS = s_nRFxUARTDev[devno].RtsPin;
	}
	else
	{
		s_nRFxUARTDev[devno].pReg->CONFIG &= ~(UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
		s_nRFxUARTDev[devno].pReg->PSELRTS = -1;
		s_nRFxUARTDev[devno].pReg->PSELCTS = -1;
		s_nRFxUARTDev[devno].CtsPin = -1;
		s_nRFxUARTDev[devno].RtsPin = -1;
	}
#else
#ifndef UART_PRESENT
	// DMA mode avail only, force DMA
	pDev->DevIntrf.bDma = true;
#else
	pDev->DevIntrf.bDma = pCfg->bDMAMode;
#endif
	s_nRFxUARTDev[devno].RxPin = (pincfg[UARTPIN_RX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RX_IDX].PortNo << 5);
	s_nRFxUARTDev[devno].TxPin = (pincfg[UARTPIN_TX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_TX_IDX].PortNo << 5);
	s_nRFxUARTDev[devno].pDmaReg->PSEL.RXD = s_nRFxUARTDev[devno].RxPin;
	s_nRFxUARTDev[devno].pDmaReg->PSEL.TXD = s_nRFxUARTDev[devno].TxPin;

	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		s_nRFxUARTDev[devno].CtsPin = (pincfg[UARTPIN_CTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_CTS_IDX].PortNo << 5);
		s_nRFxUARTDev[devno].RtsPin = (pincfg[UARTPIN_RTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RTS_IDX].PortNo << 5);
		s_nRFxUARTDev[devno].pDmaReg->PSEL.CTS = s_nRFxUARTDev[devno].CtsPin;
		s_nRFxUARTDev[devno].pDmaReg->PSEL.RTS = s_nRFxUARTDev[devno].RtsPin;

		IOPinClear(pincfg[UARTPIN_CTS_IDX].PortNo, pincfg[UARTPIN_CTS_IDX].PinNo);
		IOPinClear(pincfg[UARTPIN_RTS_IDX].PortNo, pincfg[UARTPIN_RTS_IDX].PinNo);
	}
	else
	{
		s_nRFxUARTDev[devno].pDmaReg->PSEL.RTS = -1;
		s_nRFxUARTDev[devno].pDmaReg->PSEL.CTS = -1;
		s_nRFxUARTDev[devno].CtsPin = -1;
		s_nRFxUARTDev[devno].RtsPin = -1;
	}

	s_nRFxUARTDev[devno].pDmaReg->CONFIG = nRFUARTCfgVal(pCfg);
#endif

	// Set baud
	pDev->Rate = nRFUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

#if NRFX_UART_FRAMETIMEOUT
	reg->FRAMETIMEOUT = NRFX_UART_FRAMETIMEOUT_BITS;
	reg->EVENTS_FRAMETIMEOUT = 0;
#endif

	reg->EVENTS_RXDRDY = 0;
	reg->EVENTS_TXDRDY = 0;
	reg->EVENTS_ERROR = 0;
	reg->EVENTS_RXTO = 0;
	reg->EVENTS_CTS = 0;
	reg->ERRORSRC = reg->ERRORSRC;

#ifdef UARTE_PRESENT
	if (pDev->DevIntrf.bDma == true)
	{
		s_nRFxUARTDev[devno].pDmaReg->EVENTS_TXSTOPPED = 0;
	}
#endif

	s_nRFxUARTDev[devno].pUartDev->bRxReady = false;
	s_nRFxUARTDev[devno].pUartDev->bTxReady = true;
	s_nRFxUARTDev[devno].pUartDev->RxOvrErrCnt = 0;
	s_nRFxUARTDev[devno].pUartDev->ParErrCnt = 0;
	s_nRFxUARTDev[devno].pUartDev->FramErrCnt = 0;
	s_nRFxUARTDev[devno].pUartDev->RxDropCnt = 0;
	s_nRFxUARTDev[devno].pUartDev->TxDropCnt = 0;
	s_nRFxUARTDev[devno].RxDmaCnt = 0;

	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->Parity = pCfg->Parity;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->DevIntrf.Disable = nRFUARTDisable;
	pDev->DevIntrf.Enable = nRFUARTEnable;
	pDev->DevIntrf.GetRate = nRFUARTGetRate;
	pDev->DevIntrf.SetRate = nRFUARTSetRate;
	pDev->DevIntrf.StartRx = nRFUARTStartRx;
	pDev->DevIntrf.RxData = nRFUARTRxData;
	pDev->DevIntrf.StopRx = nRFUARTStopRx;
	pDev->DevIntrf.StartTx = nRFUARTStartTx;
	pDev->DevIntrf.TxData = nRFUARTTxData;
	pDev->DevIntrf.StopTx = nRFUARTStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = nRFUARTPowerOff;
	pDev->DevIntrf.GetHandle = nRFUARTGetHandle;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	apply_workaround_for_enable_anomaly(&s_nRFxUARTDev[devno]);

#ifdef UARTE_PRESENT
	if (pDev->DevIntrf.bDma == true)
	{
		s_nRFxUARTDev[devno].pDmaReg->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

		// We need to transfer only 1 byte at a time for Rx. Otherwise, it will not interrupt
		// until buffer is filled. It will be blocked.
		// The RX timeout logic of the nRF series is implemented wrong. We cannot use it.
		NRFX_UART_RXD_MAXCNT(s_nRFxUARTDev[devno].pDmaReg) = NRFX_UART_RXDMA_SIZE;
		NRFX_UART_RXD_PTR(s_nRFxUARTDev[devno].pDmaReg) = (uint32_t)s_nRFxUARTDev[devno].RxDmaMem;
		NRFX_UART_EVT_ENDRX(s_nRFxUARTDev[devno].pDmaReg) = 0;
		NRFX_UART_TASK_STARTRX(s_nRFxUARTDev[devno].pDmaReg) = 1;
	}
	else
#endif
	{
#ifdef UART_PRESENT
		s_nRFxUARTDev[devno].pReg->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
		s_nRFxUARTDev[devno].pReg->TASKS_STARTTX = 1;
		s_nRFxUARTDev[devno].pReg->TASKS_STARTRX = 1;
#endif
	}

	reg->INTENCLR = 0xffffffffUL;

	if (pCfg->bIntMode)
	{
#if NRFX_UART_SHAREDIRQ
		SharedIntrfSetIrqHandler(pCfg->DevNo, &pDev->DevIntrf, UartIrqHandler);
#endif

#ifdef UARTE_PRESENT
		if (pDev->DevIntrf.bDma == true)
		{
			s_nRFxUARTDev[devno].pDmaReg->INTENSET = UARTE_INTENSET_RXDRDY_Msk |
								  UARTE_INTENSET_RXTO_Msk |
								  UARTE_INTENSET_TXDRDY_Msk |
								  UARTE_INTENSET_ERROR_Msk |
								  UARTE_INTENSET_CTS_Msk |
								  UARTE_INTENSET_NCTS_Msk |
								  NRFX_UART_INTEN_ENDTX |
								  NRFX_UART_INTEN_ENDRX
#if NRFX_UART_FRAMETIMEOUT
								  | UARTE_INTENSET_FRAMETIMEOUT_Msk
#endif
								  ;
		}
		else
#endif
		{
#ifdef UART_PRESENT
			s_nRFxUARTDev[devno].pReg->INTENSET = UART_INTENSET_RXDRDY_Msk |
								  UART_INTENSET_RXTO_Msk |
								  UART_INTENSET_TXDRDY_Msk |
								  UART_INTENSET_ERROR_Msk |
								  UART_INTENSET_CTS_Msk |
								  UART_INTENSET_NCTS_Msk;
#endif
		}

		NVIC_ClearPendingIRQ(s_nRFxUARTDev[devno].IrqNo);
		NVIC_SetPriority(s_nRFxUARTDev[devno].IrqNo, pCfg->IntPrio);
		NVIC_EnableIRQ(s_nRFxUARTDev[devno].IrqNo);
	}

	return true;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
}

UARTDev_t const *UARTGetInstance(int DevNo)
{
	return s_nRFxUARTDev[DevNo].pUartDev;
}

#if !NRFX_UART_SHAREDIRQ
// nRF51 and nRF52 give the UART a vector of its own. Keep C linkage so these
// override the weak defaults in the vector table.

#ifdef UART_PRESENT
extern "C" void UART0_IRQHandler()
{
	UartIrqHandler(0, &s_nRFxUARTDev[0].pUartDev->DevIntrf);
}
#endif

#ifdef NRF_UARTE1
extern "C" void UARTE1_IRQHandler()
{
	UartIrqHandler(1, &s_nRFxUARTDev[1].pUartDev->DevIntrf);
}
#endif

#endif
