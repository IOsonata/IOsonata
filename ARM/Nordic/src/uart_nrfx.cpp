/**-------------------------------------------------------------------------
@file	uart_nrfx.cpp

@brief	nRF5x & nRF54x UART implementation

Single implementation for all nRF series.

nRF51        : UART0 only, no DMA
nRF52        : UART0/UARTE0 (+UARTE1 on nRF52840), DMA optional
nRF53, nRF91 : UARTE0..3, DMA only
nRF54L       : new UARTE register block (DMA.RX/DMA.TX), DMA only,
               hardware frame timeout

nRF54L DevNo mapping

DevNo		hardware		domain
0			UARTE30			LP  (P0)
1			UARTE20			PERI(P1)
2			UARTE21			PERI(P1)
3			UARTE22			PERI(P1)
4			UARTE00			MCU (P2), runs on the fast clock
5			UARTE23			PERI(P1), nRF54LM20x only
6			UARTE24			PERI(P1), nRF54LM20x only

@author	Hoang Nguyen Hoan
@date	Aug. 30, 2015

@license

Copyright (c) 2015, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

// Defining common bitfields for both DMA & non DMA registers
#define NRFX_UART_ERRORSRC_BREAK_Pos (3UL) /*!< Position of BREAK field. */
#define NRFX_UART_ERRORSRC_BREAK_Msk (0x1UL << NRFX_UART_ERRORSRC_BREAK_Pos) /*!< Bit mask of BREAK field. */
#define NRFX_UART_ERRORSRC_BREAK_NotPresent (0UL) /*!< Read: error not present */
#define NRFX_UART_ERRORSRC_BREAK_Present (1UL) /*!< Read: error present */

/* Bit 2 : Framing error occurred */
#define NRFX_UART_ERRORSRC_FRAMING_Pos (2UL) /*!< Position of FRAMING field. */
#define NRFX_UART_ERRORSRC_FRAMING_Msk (0x1UL << NRFX_UART_ERRORSRC_FRAMING_Pos) /*!< Bit mask of FRAMING field. */
#define NRFX_UART_ERRORSRC_FRAMING_NotPresent (0UL) /*!< Read: error not present */
#define NRFX_UART_ERRORSRC_FRAMING_Present (1UL) /*!< Read: error present */

/* Bit 1 : Parity error */
#define NRFX_UART_ERRORSRC_PARITY_Pos (1UL) /*!< Position of PARITY field. */
#define NRFX_UART_ERRORSRC_PARITY_Msk (0x1UL << NRFX_UART_ERRORSRC_PARITY_Pos) /*!< Bit mask of PARITY field. */
#define NRFX_UART_ERRORSRC_PARITY_NotPresent (0UL) /*!< Read: error not present */
#define NRFX_UART_ERRORSRC_PARITY_Present (1UL) /*!< Read: error present */

/* Bit 0 : Overrun error */
#define NRFX_UART_ERRORSRC_OVERRUN_Pos (0UL) /*!< Position of OVERRUN field. */
#define NRFX_UART_ERRORSRC_OVERRUN_Msk (0x1UL << NRFX_UART_ERRORSRC_OVERRUN_Pos) /*!< Bit mask of OVERRUN field. */
#define NRFX_UART_ERRORSRC_OVERRUN_NotPresent (0UL) /*!< Read: error not present */
#define NRFX_UART_ERRORSRC_OVERRUN_Present (1UL) /*!< Read: error present */

// The nRF54 series moved the UARTE DMA controls into the DMA.RX/DMA.TX
// sub-blocks and renamed the task/event registers. Everything else in the
// transfer logic is identical, so the differences are folded into the
// accessors below and the shared code reads through them.
#if defined(NRF54L_SERIES)
#define NRFX_UARTE_NEW_DMA_REG		1

#define NRFX_UARTE_RXD_PTR(r)		((r)->DMA.RX.PTR)
#define NRFX_UARTE_RXD_MAXCNT(r)	((r)->DMA.RX.MAXCNT)
#define NRFX_UARTE_RXD_AMOUNT(r)	((r)->DMA.RX.AMOUNT)
#define NRFX_UARTE_TXD_PTR(r)		((r)->DMA.TX.PTR)
#define NRFX_UARTE_TXD_MAXCNT(r)	((r)->DMA.TX.MAXCNT)
#define NRFX_UARTE_EVENTS_ENDRX(r)	((r)->EVENTS_DMA.RX.END)
#define NRFX_UARTE_EVENTS_ENDTX(r)	((r)->EVENTS_DMA.TX.END)
#define NRFX_UARTE_TASKS_STARTRX(r)	((r)->TASKS_DMA.RX.START)
#define NRFX_UARTE_TASKS_STOPRX(r)	((r)->TASKS_DMA.RX.STOP)
#define NRFX_UARTE_TASKS_STARTTX(r)	((r)->TASKS_DMA.TX.START)
#define NRFX_UARTE_TASKS_STOPTX(r)	((r)->TASKS_DMA.TX.STOP)

// There is no indication in the datasheet about how many hardware fifo
// this value seems to produce best performance
#define NRFX_UART_HWFIFO_SIZE		8
#elif defined(UARTE_PRESENT)
#define NRFX_UARTE_RXD_PTR(r)		((r)->RXD.PTR)
#define NRFX_UARTE_RXD_MAXCNT(r)	((r)->RXD.MAXCNT)
#define NRFX_UARTE_RXD_AMOUNT(r)	((r)->RXD.AMOUNT)
#define NRFX_UARTE_TXD_PTR(r)		((r)->TXD.PTR)
#define NRFX_UARTE_TXD_MAXCNT(r)	((r)->TXD.MAXCNT)
#define NRFX_UARTE_EVENTS_ENDRX(r)	((r)->EVENTS_ENDRX)
#define NRFX_UARTE_EVENTS_ENDTX(r)	((r)->EVENTS_ENDTX)
#define NRFX_UARTE_TASKS_STARTRX(r)	((r)->TASKS_STARTRX)
#define NRFX_UARTE_TASKS_STOPRX(r)	((r)->TASKS_STOPRX)
#define NRFX_UARTE_TASKS_STARTTX(r)	((r)->TASKS_STARTTX)
#define NRFX_UARTE_TASKS_STOPTX(r)	((r)->TASKS_STOPTX)

// There is no indication in the datasheet about how many hardware fifo
// this value seems to produce best performance
#define NRFX_UART_HWFIFO_SIZE		4
#else
#define NRFX_UART_HWFIFO_SIZE		4
#endif

#define NRFX_UART_MAXDEV			UARTE_COUNT

// Default fifo size if one is not provided in the config.
#define NRFX_UART_RXDMA_SIZE		(NRFX_UART_HWFIFO_SIZE)
#define NRFX_UART_TXDMA_SIZE		(4 * NRFX_UART_HWFIFO_SIZE)

#define NRFX_UART_CFIFO_SIZE		CFIFO_MEMSIZE(4 * NRFX_UART_HWFIFO_SIZE)

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
	UARTDev_t *pUartDev;				// Pointer to generic UART dev. data
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

alignas(4) static nRFUartDev_t s_nRFxUARTDev[] = {
#if defined(NRF54L_SERIES)
	{	// On P0
		.DevNo = 0,
		.pDmaReg = NRF_UARTE30_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// On P1
		.DevNo = 1,
		.pDmaReg = NRF_UARTE20_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// On P1
		.DevNo = 2,
		.pDmaReg = NRF_UARTE21_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// On P1
		.DevNo = 3,
		.pDmaReg = NRF_UARTE22_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// On P2
		.DevNo = 4,
		.pDmaReg = NRF_UARTE00_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#if UARTE_COUNT > 5
	{	// On P1
		.DevNo = 5,
		.pDmaReg = NRF_UARTE23_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{	// On P1
		.DevNo = 6,
		.pDmaReg = NRF_UARTE24_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#endif
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	{
		.DevNo = 0,
		.pDmaReg = NRF_UARTE0_NS,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#else
	{
		.DevNo = 0,
		.pDmaReg = NRF_UARTE0_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 1,
		.pDmaReg = NRF_UARTE1_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 2,
		.pDmaReg = NRF_UARTE2_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
	{
		.DevNo = 3,
		.pDmaReg = NRF_UARTE3_S,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#endif
#else
	{
		.DevNo = 0,
		.pReg = NRF_UART0,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#if NRFX_UART_MAXDEV > 1
	{
		.DevNo = 1,
		.pDmaReg = NRF_UARTE1,
		.pUartDev = NULL,
		.RxPin = (uint32_t)-1,
		.TxPin = (uint32_t)-1,
		.CtsPin = (uint32_t)-1,
		.RtsPin = (uint32_t)-1,
	},
#endif
#endif
};

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

#if defined(UARTE_CONFIG_FRAMETIMEOUT_Msk)
	if (dev->pDmaReg->EVENTS_FRAMETIMEOUT)
	{
		// The hardware frame timeout replaces the manual DMA stop used on
		// the older series : stopping the RX DMA forces the END event which
		// drains the partial buffer into the fifo.
		dev->pDmaReg->EVENTS_FRAMETIMEOUT = 0;
		NRFX_UARTE_TASKS_STOPRX(dev->pDmaReg) = 1;
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
	if (NRFX_UARTE_EVENTS_ENDRX(dev->pDmaReg))
	{
		dev->pDmaReg->EVENTS_RXDRDY = 0;
		NRFX_UARTE_EVENTS_ENDRX(dev->pDmaReg) = 0;
		dev->RxDmaCnt = 0;

		int l = NRFX_UARTE_RXD_AMOUNT(dev->pDmaReg);
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
		NRFX_UARTE_TASKS_STARTRX(dev->pDmaReg) = 1;
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
	if (NRFX_UARTE_EVENTS_ENDTX(dev->pDmaReg) || dev->pDmaReg->EVENTS_TXSTOPPED)
	{
		NRFX_UARTE_EVENTS_ENDTX(dev->pDmaReg) = 0;
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

			NRFX_UARTE_TXD_MAXCNT(dev->pDmaReg) = l;
			NRFX_UARTE_TXD_PTR(dev->pDmaReg) = (uint32_t)dev->TxDmaMem;
			NRFX_UARTE_TASKS_STARTTX(dev->pDmaReg) = 1;
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
				NRFX_UARTE_TASKS_STOPRX(dev->pDmaReg) = 1;
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
		// Write back the captured bits (write 1 to clear). Re-reading the
		// register here could clear error bits latched after the capture
		// without them ever being counted.
		reg->ERRORSRC = err;
		len = 0;
		if (dev->pUartDev->EvtCallback)
		{
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, NULL, len);
		}
#ifdef UARTE_PRESENT
		if (pDev->bDma == true)
		{
			NRFX_UARTE_TASKS_STARTRX(dev->pDmaReg) = 1;
		}
		else
#endif
		{
#ifdef UART_PRESENT
			dev->pReg->TASKS_STARTRX = 1;
#endif
		}
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

	int rate = 0;

	// NOTE: Thanks to hmolesworth for the Baudrate calculation posted on devzone.
	// https://devzone.nordicsemi.com/f/nordic-q-a/43280/technical-question-regarding-uart-baud-rate-generator-baudrate-register-offset-0x524/458422

	uint32_t regval = (uint32_t)(((((uint64_t)Rate << 32ULL) + 8000000ULL) / 16000000ULL) + 0x800ULL) & 0xFFFFF000;
	rate = (uint32_t)(((uint64_t)regval * 16000000ULL) >> 32ULL);

#if defined(NRF54L_SERIES)
	if (dev->pDmaReg == NRF_UARTE00_S)
	{
		// UARTE00 sits in the MCU domain and runs on the fast clock, not
		// the 16MHz the baud formula assumes.
		uint32_t f = SystemCoreClock / 16000000;
		dev->pDmaReg->BAUDRATE = regval / f;
	}
	else
	{
		dev->pDmaReg->BAUDRATE = regval;
	}
#elif defined(UARTE_PRESENT)
	dev->pDmaReg->BAUDRATE = regval;
#else
	dev->pReg->BAUDRATE = regval;
#endif

	return rate;
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
#if defined(UARTE_PRESENT) && !defined(UARTE_CONFIG_FRAMETIMEOUT_Msk)
			// Force the partial DMA buffer out into the fifo. On chips with
			// the hardware frame timeout this is done by the FRAMETIMEOUT
			// event instead.
			if (pDev->bDma == true && CFifoUsed(dev->pUartDev->hRxFifo) <= 0)
			{
				if (dev->RxDmaCnt > 0)
				{
					NRFX_UARTE_TASKS_STOPRX(dev->pDmaReg) = 1;
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
			NRFX_UARTE_RXD_MAXCNT(dev->pDmaReg) = NRFX_UART_RXDMA_SIZE;
			NRFX_UARTE_RXD_PTR(dev->pDmaReg) = (uint32_t)dev->RxDmaMem;
			NRFX_UARTE_TASKS_STARTRX(dev->pDmaReg) = 1;
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

					NRFX_UARTE_TXD_MAXCNT(dev->pDmaReg) = l;
					NRFX_UARTE_TXD_PTR(dev->pDmaReg) = (uint32_t)dev->TxDmaMem;
					NRFX_UARTE_TASKS_STARTTX(dev->pDmaReg) = 1;
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
	NRFX_UARTE_TASKS_STOPRX(reg) = 1;
	NRFX_UARTE_TASKS_STOPTX(reg) = 1;
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
		NRFX_UARTE_RXD_MAXCNT(dev->pDmaReg) = NRFX_UART_RXDMA_SIZE;
		NRFX_UARTE_RXD_PTR(dev->pDmaReg) = (uint32_t)dev->RxDmaMem;
		NRFX_UARTE_EVENTS_ENDRX(dev->pDmaReg) = 0;
		NRFX_UARTE_TASKS_STARTRX(dev->pDmaReg) = 1;
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
#ifdef UART_PRESENT
	NRF_UART_Type *reg = dev->pReg;
#else
	NRF_UARTE_Type *reg = dev->pDmaReg;
#endif

#if !defined(NRF54L_SERIES)
	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
	// Not present on the nRF54 series.
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 0;
#else
	(void)reg;
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
#endif // defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || defined(NRF91_SERIES)
}

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

#if !defined(NRF54L_SERIES)
	// Force power on in case it was powered off previously
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
#endif

#if defined(UARTE_CONFIG_FRAMESIZE_Msk)
	// New UARTE (nRF54 series) : full frame configuration in one register
	// write. FRAMETIMEOUT counts idle bit periods after the last received
	// frame before flagging the timeout event.
	{
		uint32_t cnf = ((uint32_t)pCfg->DataBits << UARTE_CONFIG_FRAMESIZE_Pos) & UARTE_CONFIG_FRAMESIZE_Msk;

		if (pCfg->Parity != UART_PARITY_NONE)
		{
			cnf |= UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos;
			cnf |= pCfg->Parity == UART_PARITY_ODD ? (UARTE_CONFIG_PARITYTYPE_Odd << UARTE_CONFIG_PARITYTYPE_Pos) :
								  (UARTE_CONFIG_PARITYTYPE_Even << UARTE_CONFIG_PARITYTYPE_Pos);
		}

		cnf |= pCfg->StopBits == 2 ? UARTE_CONFIG_STOP_Two << UARTE_CONFIG_STOP_Pos : UARTE_CONFIG_STOP_One << UARTE_CONFIG_STOP_Pos;

		if (pCfg->FlowControl == UART_FLWCTRL_HW)
		{
			cnf |= (UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
		}

		cnf |= UARTE_CONFIG_ENDIAN_LSB << UARTE_CONFIG_ENDIAN_Pos;
		cnf |= UARTE_CONFIG_FRAMETIMEOUT_Msk;

		s_nRFxUARTDev[devno].pDmaReg->CONFIG = cnf;
		s_nRFxUARTDev[devno].pDmaReg->FRAMETIMEOUT = 8;
	}
#elif defined(UARTE_PRESENT)
	s_nRFxUARTDev[devno].pDmaReg->CONFIG &= ~(UARTE_CONFIG_PARITY_Msk << UARTE_CONFIG_PARITY_Pos);

	if (pCfg->Parity == UART_PARITY_NONE)
	{
		s_nRFxUARTDev[devno].pDmaReg->CONFIG |= UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos;
	}
	else
	{
		s_nRFxUARTDev[devno].pDmaReg->CONFIG |= UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos;
	}
#endif

    // Set baud
    pDev->Rate = nRFUARTSetRate(&pDev->DevIntrf, pCfg->Rate);


	reg->EVENTS_RXDRDY = 0;
	reg->EVENTS_TXDRDY = 0;
	reg->EVENTS_ERROR = 0;
	reg->EVENTS_RXTO = 0;
	reg->ERRORSRC = reg->ERRORSRC;
	reg->EVENTS_CTS = 0;
#if defined(UARTE_CONFIG_FRAMETIMEOUT_Msk)
	reg->EVENTS_FRAMETIMEOUT = 0;
	reg->EVENTS_TXSTOPPED = 0;
#endif

#if defined(UARTE_PRESENT) && !defined(NRFX_UARTE_NEW_DMA_REG)
	if (pDev->DevIntrf.bDma == true)
	{
		s_nRFxUARTDev[devno].pDmaReg->EVENTS_RXSTARTED = 0;
		s_nRFxUARTDev[devno].pDmaReg->EVENTS_TXSTARTED = 0;
		s_nRFxUARTDev[devno].pDmaReg->EVENTS_TXSTOPPED = 0;
	}
#endif

#if defined(UARTE_CONFIG_FRAMESIZE_Msk)
	// Flow control pins. HWFC itself is already set in CONFIG above.
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
#elif defined(UART_PRESENT)
    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	s_nRFxUARTDev[devno].pReg->CONFIG |= (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
    	s_nRFxUARTDev[devno].CtsPin = (pincfg[UARTPIN_CTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_CTS_IDX].PortNo << 5);
    	s_nRFxUARTDev[devno].RtsPin = (pincfg[UARTPIN_RTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RTS_IDX].PortNo << 5);
#ifdef NRF52805_XXAA
    	s_nRFxUARTDev[devno].pReg->PSEL.CTS = s_nRFxUARTDev[devno].CtsPin;
    	s_nRFxUARTDev[devno].pReg->PSEL.RTS = s_nRFxUARTDev[devno].RtsPin;
#else
    	s_nRFxUARTDev[devno].pReg->PSELCTS = s_nRFxUARTDev[devno].CtsPin;
    	s_nRFxUARTDev[devno].pReg->PSELRTS = s_nRFxUARTDev[devno].RtsPin;
#endif
	}
	else
	{
		s_nRFxUARTDev[devno].pReg->CONFIG &= ~(UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
#ifdef NRF52805_XXAA
		s_nRFxUARTDev[devno].pReg->PSEL.RTS = -1;
		s_nRFxUARTDev[devno].pReg->PSEL.CTS = -1;
#else
		s_nRFxUARTDev[devno].pReg->PSELRTS = -1;
		s_nRFxUARTDev[devno].pReg->PSELCTS = -1;
#endif
		s_nRFxUARTDev[devno].CtsPin = -1;
		s_nRFxUARTDev[devno].RtsPin = -1;
	}
#else
    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	s_nRFxUARTDev[devno].pDmaReg->CONFIG |= (UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
    	s_nRFxUARTDev[devno].CtsPin = (pincfg[UARTPIN_CTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_CTS_IDX].PortNo << 5);
    	s_nRFxUARTDev[devno].RtsPin = (pincfg[UARTPIN_RTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RTS_IDX].PortNo << 5);
    	s_nRFxUARTDev[devno].pDmaReg->PSEL.CTS = s_nRFxUARTDev[devno].CtsPin;
    	s_nRFxUARTDev[devno].pDmaReg->PSEL.RTS = s_nRFxUARTDev[devno].RtsPin;

    	IOPinClear(pincfg[UARTPIN_CTS_IDX].PortNo, pincfg[UARTPIN_CTS_IDX].PinNo);
        IOPinClear(pincfg[UARTPIN_RTS_IDX].PortNo, pincfg[UARTPIN_RTS_IDX].PinNo);
	}
	else
	{
		s_nRFxUARTDev[devno].pDmaReg->CONFIG &= ~(UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
		s_nRFxUARTDev[devno].pDmaReg->PSEL.RTS = -1;
		s_nRFxUARTDev[devno].pDmaReg->PSEL.CTS = -1;
		s_nRFxUARTDev[devno].CtsPin = -1;
		s_nRFxUARTDev[devno].RtsPin = -1;
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
		NRFX_UARTE_RXD_MAXCNT(s_nRFxUARTDev[devno].pDmaReg) = NRFX_UART_RXDMA_SIZE;
		NRFX_UARTE_RXD_PTR(s_nRFxUARTDev[devno].pDmaReg) = (uint32_t)s_nRFxUARTDev[devno].RxDmaMem;
		NRFX_UARTE_EVENTS_ENDRX(s_nRFxUARTDev[devno].pDmaReg) = 0;
		NRFX_UARTE_TASKS_STARTRX(s_nRFxUARTDev[devno].pDmaReg) = 1;
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
#if defined(NRF91_SERIES) || defined(NRF53_SERIES) || defined(NRF54L_SERIES)
		SharedIntrfSetIrqHandler(pCfg->DevNo, &pDev->DevIntrf, UartIrqHandler);
#endif

#ifdef UARTE_PRESENT
		if (pDev->DevIntrf.bDma == true)
		{
#if defined(UARTE_INTENSET_DMARXEND_Msk)
			s_nRFxUARTDev[devno].pDmaReg->INTENSET = (UARTE_INTENSET_RXDRDY_Set << UARTE_INTENSET_RXDRDY_Pos) |
							  (UARTE_INTENSET_RXTO_Set << UARTE_INTENSET_RXTO_Pos) |
							  (UARTE_INTENSET_TXDRDY_Set << UARTE_INTENSET_TXDRDY_Pos) |
							  (UARTE_INTENSET_ERROR_Set << UARTE_INTENSET_ERROR_Pos) |
							  (UARTE_INTENSET_CTS_Set << UARTE_INTENSET_CTS_Pos) |
							  (UARTE_INTENSET_NCTS_Set << UARTE_INTENSET_NCTS_Pos) |
							  (UARTE_INTENSET_DMARXEND_Set << UARTE_INTENSET_DMARXEND_Pos) |
							  (UARTE_INTENSET_DMATXEND_Set << UARTE_INTENSET_DMATXEND_Pos) |
							  (UARTE_INTEN_FRAMETIMEOUT_Enabled << UARTE_INTEN_FRAMETIMEOUT_Pos);
#else
			s_nRFxUARTDev[devno].pDmaReg->INTENSET = (UARTE_INTENSET_RXDRDY_Set << UARTE_INTENSET_RXDRDY_Pos) |
							  (UARTE_INTENSET_RXTO_Set << UARTE_INTENSET_RXTO_Pos) |
							  (UARTE_INTENSET_TXDRDY_Set << UARTE_INTENSET_TXDRDY_Pos) |
							  (UARTE_INTENSET_ERROR_Set << UARTE_INTENSET_ERROR_Pos) |
							  (UARTE_INTENSET_CTS_Set << UARTE_INTENSET_CTS_Pos) |
							  (UARTE_INTENSET_NCTS_Set << UARTE_INTENSET_NCTS_Pos) |
							  (UARTE_INTENSET_ENDTX_Set << UARTE_INTENSET_ENDTX_Pos) |
							  (UARTE_INTENSET_ENDRX_Set << UARTE_INTENSET_ENDRX_Pos);
#endif
		}
		else
#endif
		{
#ifdef UART_PRESENT
			s_nRFxUARTDev[devno].pReg->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
							  (UART_INTENSET_RXTO_Set << UART_INTENSET_RXTO_Pos) |
							  (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
							  (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos) |
							  (UART_INTENSET_CTS_Set << UART_INTENSET_CTS_Pos) |
							  (UART_INTENSET_NCTS_Set << UART_INTENSET_NCTS_Pos);
#endif
		}

		switch (devno)
		{
#if defined(NRF54L_SERIES)
		case 0:
			NVIC_ClearPendingIRQ(SERIAL30_IRQn);
			NVIC_SetPriority(SERIAL30_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SERIAL30_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(SERIAL20_IRQn);
			NVIC_SetPriority(SERIAL20_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SERIAL20_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(SERIAL21_IRQn);
			NVIC_SetPriority(SERIAL21_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SERIAL21_IRQn);
			break;
		case 3:
			NVIC_ClearPendingIRQ(SERIAL22_IRQn);
			NVIC_SetPriority(SERIAL22_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SERIAL22_IRQn);
			break;
		case 4:
			NVIC_ClearPendingIRQ(SERIAL00_IRQn);
			NVIC_SetPriority(SERIAL00_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SERIAL00_IRQn);
			break;
#if UARTE_COUNT > 5
		case 5:
			NVIC_ClearPendingIRQ(SERIAL23_IRQn);
			NVIC_SetPriority(SERIAL23_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SERIAL23_IRQn);
			break;
		case 6:
			NVIC_ClearPendingIRQ(SERIAL24_IRQn);
			NVIC_SetPriority(SERIAL24_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SERIAL24_IRQn);
			break;
#endif
#elif defined(NRF91_SERIES)
		case 0:
			NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
			NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
			NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
			NVIC_SetPriority(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
			break;
		case 3:
			NVIC_ClearPendingIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
			NVIC_SetPriority(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
			break;
#elif defined(NRF53_SERIES)
    		case 0:
                NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn, pCfg->IntPrio);
                NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                break;
#ifdef NRF5340_XXAA_APPLICATION
    	    case 1:
                NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
                NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn, pCfg->IntPrio);
                NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
                NVIC_SetPriority(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn, pCfg->IntPrio);
                NVIC_EnableIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
                break;
    	    case 3:
                NVIC_ClearPendingIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
                NVIC_SetPriority(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn, pCfg->IntPrio);
                NVIC_EnableIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
                break;
#endif
#else
			case 0:
#ifdef NRF51
				NVIC_ClearPendingIRQ(UART0_IRQn);
				NVIC_SetPriority(UART0_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UART0_IRQn);
#else
				NVIC_ClearPendingIRQ(UARTE0_UART0_IRQn);
				NVIC_SetPriority(UARTE0_UART0_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UARTE0_UART0_IRQn);
#endif
				break;
#ifdef NRF52840_XXAA
			case 1:
				NVIC_ClearPendingIRQ(UARTE1_IRQn);
				NVIC_SetPriority(UARTE1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UARTE1_IRQn);
				break;
#endif
#endif
        }
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

// Direct vector handlers for the devices whose UART interrupt is not routed
// through the shared interface (nRF51 & nRF52). The IOsonata nRF52 vector
// tables name the entry UARTE0_UART0_IRQHandler; nRF51 names it
// UART0_IRQHandler.
#if defined(NRF51)
extern "C" void UART0_IRQHandler()
{
	UartIrqHandler(0, &s_nRFxUARTDev[0].pUartDev->DevIntrf);
}
#elif defined(NRF52_SERIES)
extern "C" void UARTE0_UART0_IRQHandler()
{
	UartIrqHandler(0, &s_nRFxUARTDev[0].pUartDev->DevIntrf);
}
#endif

#if defined(NRF52840_XXAA)
extern "C" void UARTE1_IRQHandler()
{
	UartIrqHandler(1, &s_nRFxUARTDev[1].pUartDev->DevIntrf);
}
#endif
