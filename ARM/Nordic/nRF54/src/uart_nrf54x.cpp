/**-------------------------------------------------------------------------
@file	uart_nrf54x.c

@brief	nRF54x UART implementation

nRF54L15 UART mapping

DevNo		nRF54L15 hardware		Max Rate (Baud)
0			UART30					1M
1			UART20					1M
2			UART21					1M
3			UART22					1M
4			UART00					1M


@author	Hoang Nguyen Hoan
@date	Feb. 10, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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
#include "interrupt.h"
#include "coredev/shared_intrf.h"

// There is no indication in the datasheet about how many hardware fifo
// this value seems to produce best performance
#define NRFX_UART_HWFIFO_SIZE		8

#define NRFX_UART_MAXDEV			UARTE_COUNT
#define NRFX_UART_DMA_MAXCNT		((1<<UARTE0_EASYDMA_MAXCNT_SIZE)-1)

// Default fifo size if one is not provided is not provided in the config.
#define NRFX_UART_RXDMA_SIZE		(NRFX_UART_HWFIFO_SIZE)
#define NRFX_UART_TXDMA_SIZE		(4 * NRFX_UART_HWFIFO_SIZE)

#define NRFX_UART_CFIFO_SIZE		CFIFO_MEMSIZE(4 * NRFX_UART_HWFIFO_SIZE)

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct __nRF_UART_Dev {
	int DevNo;				// UART interface number
	NRF_UARTE_Type *pDmaReg;	// UART registers
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
/*
typedef struct {
	int Baud;
	int RegVal;
} nRFUartBaud_t;
*/
#pragma pack(pop)
/*
alignas(4) static const nRFUartBaud_t s_nRFxBaudrate[] = {
	{1200, UARTE_BAUDRATE_BAUDRATE_Baud1200},
	{2400, UARTE_BAUDRATE_BAUDRATE_Baud2400},
	{4800, UARTE_BAUDRATE_BAUDRATE_Baud4800},
	{9600, UARTE_BAUDRATE_BAUDRATE_Baud9600},
	{14400, UARTE_BAUDRATE_BAUDRATE_Baud14400},
	{19200, UARTE_BAUDRATE_BAUDRATE_Baud19200},
	{28800, UARTE_BAUDRATE_BAUDRATE_Baud28800},
	{31250, UARTE_BAUDRATE_BAUDRATE_Baud31250},
	{38400, UARTE_BAUDRATE_BAUDRATE_Baud38400},
	{56000, UARTE_BAUDRATE_BAUDRATE_Baud56000},
	{57600, UARTE_BAUDRATE_BAUDRATE_Baud57600},
	{76800, UARTE_BAUDRATE_BAUDRATE_Baud76800},
	{115200, UARTE_BAUDRATE_BAUDRATE_Baud115200},
	{230400, UARTE_BAUDRATE_BAUDRATE_Baud230400},
	{250000, UARTE_BAUDRATE_BAUDRATE_Baud250000},
	{460800, UARTE_BAUDRATE_BAUDRATE_Baud460800},
	{921600, UARTE_BAUDRATE_BAUDRATE_Baud921600},
	{1000000, UARTE_BAUDRATE_BAUDRATE_Baud1M}
};

static const int s_NbBaudrate = sizeof(s_nRFxBaudrate) / sizeof(nRFUartBaud_t);
*/
alignas(4) static nRFUartDev_t s_nRFxUARTDev[] = {
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
};

static const int s_NbUartDev = sizeof(s_nRFxUARTDev) / sizeof(nRFUartDev_t);

bool nRFUARTWaitForRxReady(nRFUartDev_t * const pDev, uint32_t Timeout)
{
	do {
		if (pDev->pDmaReg->EVENTS_RXDRDY || pDev->pUartDev->bRxReady)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool nRFUARTWaitForTxReady(nRFUartDev_t * const pDev, uint32_t Timeout)
{
	do {
		if (pDev->pDmaReg->EVENTS_TXDRDY || pDev->pUartDev->bTxReady == true)
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

	if (dev->pDmaReg->EVENTS_FRAMETIMEOUT)
	{
		dev->pDmaReg->EVENTS_FRAMETIMEOUT = 0;
		dev->pDmaReg->TASKS_DMA.RX.STOP = 1;
	}

	if (dev->pDmaReg->EVENTS_RXTO)
	{
		dev->pDmaReg->TASKS_FLUSHRX = 1;
		dev->pDmaReg->EVENTS_RXTO = 0;
	}

	if (dev->pDmaReg->EVENTS_DMA.RX.END)
	{
		// No DMA support for RX, just clear the event
		dev->pDmaReg->EVENTS_RXDRDY = 0;
		dev->pDmaReg->EVENTS_DMA.RX.END = 0;
		dev->RxDmaCnt = 0;

		int l = dev->pDmaReg->DMA.RX.AMOUNT;
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
		//dev->pDmaReg->RXD.MAXCNT = NRFX_UART_BUFF_SIZE;
		//dev->pDmaReg->RXD.PTR = (uint32_t)dev->RxDmaMem;
		dev->pDmaReg->TASKS_DMA.RX.START = 1;
	}

	if (dev->pDmaReg->EVENTS_RXDRDY)
	{
		uint8_t *d;

		dev->pDmaReg->EVENTS_RXDRDY = 0;
		dev->RxDmaCnt++;
		if (dev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(dev->pUartDev->hRxFifo);
			cnt = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, len);
		}
	}

	if (dev->pDmaReg->EVENTS_TXDRDY)
	{

		dev->pDmaReg->EVENTS_TXDRDY = 0;
		cnt = 0;
	}

	if (dev->pDmaReg->EVENTS_DMA.TX.END || dev->pDmaReg->EVENTS_TXSTOPPED)
	{
		dev->pDmaReg->EVENTS_DMA.TX.END = 0;
		dev->pDmaReg->EVENTS_TXSTOPPED = 0;

		int l = NRFX_UART_TXDMA_SIZE;//min(CFifoUsed(pDev->pUartDev->hTxFifo), NRF52_UART_DMA_MAX_LEN);
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
		if (p)
		{
			dev->pUartDev->bTxReady = false;

			// Transfer to tx cache before sending as CFifo will immediately make the memory
			// block available for reuse in the Put request. This could cause an overwrite
			// if uart tx has not completed in time.
			memcpy(&dev->TxDmaMem, p, l);

			dev->pDmaReg->DMA.TX.MAXCNT = l;
			dev->pDmaReg->DMA.TX.PTR = (uint32_t)dev->TxDmaMem;
			dev->pDmaReg->TASKS_DMA.TX.START = 1;
		}
		else
		{
			dev->pUartDev->bTxReady = true;
		}
		if (dev->pUartDev->EvtCallback)
		{
			//uint8_t buff[NRFUART_CFIFO_SIZE];

			//len = min(NRFUART_CFIFO_SIZE, CFifoAvail(s_nRFxUARTDev.pUartDev->hTxFifo));
			len = CFifoAvail(dev->pUartDev->hTxFifo);
			len = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, len);
			if (len > 0)
			{
				//s_nRFxUARTDev.bTxReady = false;
				//nRFUARTTxData(&s_nRFxUARTDev.pUartDev->SerIntrf, buff, len);
			}
		}
	}

	// Handle errors
	if (dev->pDmaReg->EVENTS_ERROR)
	{
		uint32_t err = dev->pDmaReg->ERRORSRC;
		dev->pDmaReg->EVENTS_ERROR = 0;

		if (err & UARTE_ERRORSRC_OVERRUN_Msk)
		{
			dev->pUartDev->RxOvrErrCnt++;
			len = 0;

			dev->pDmaReg->TASKS_DMA.RX.STOP = 1;
		}
		if (err & UARTE_ERRORSRC_FRAMING_Msk)
		{
			dev->pUartDev->FramErrCnt++;
		}
		if (err & UARTE_ERRORSRC_PARITY_Msk)
		{
			dev->pUartDev->ParErrCnt++;
		}
		if (err & UARTE_ERRORSRC_BREAK_Msk)
		{
		}
		dev->pDmaReg->ERRORSRC = err;
		len = 0;
		if (dev->pUartDev->EvtCallback)
		{
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, NULL, len);
		}
		dev->pDmaReg->TASKS_DMA.RX.START = 1;
	}

	if (dev->pDmaReg->EVENTS_CTS)
	{
		dev->pDmaReg->EVENTS_CTS = 0;
		dev->pUartDev->LineState &= ~UART_LINESTATE_CTS;
        if (dev->pUartDev->EvtCallback)
        {
            uint8_t buff = 0;
            len = 1;
            dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, &buff, len);
        }
		//NRF_UART0->TASKS_STARTTX = 1;
		//s_nRFxUARTDev.bTxReady = true;
	}

	if (dev->pDmaReg->EVENTS_NCTS)
	{
		dev->pDmaReg->EVENTS_NCTS = 0;
		dev->pUartDev->LineState |= UART_LINESTATE_CTS;
		//NRF_UART0->TASKS_STOPTX = 1;
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

	if (dev->pDmaReg == NRF_UARTE00)
	{
		// Patch for UARTE00 clock
		uint32_t f = SystemCoreClock / 16000000;
		dev->pDmaReg->BAUDRATE = regval / f;
//		rate = s_nRFxBaudrate[i].Baud;
	}
	else
	{
		dev->pDmaReg->BAUDRATE = regval;
//		rate = s_nRFxBaudrate[i].Baud;
	}
#if 0
	for (int i = 0; i < s_NbBaudrate; i++)
	{
		if (s_nRFxBaudrate[i].Baud >= Rate)
		{
			if (dev->pDmaReg == NRF_UARTE00)
			{
				// Patch for UARTE00 clock
				uint32_t f = SystemCoreClock / 16000000;
				dev->pDmaReg->BAUDRATE = s_nRFxBaudrate[i].RegVal / f;
				rate = s_nRFxBaudrate[i].Baud;
			}
			else
			{
				dev->pDmaReg->BAUDRATE = s_nRFxBaudrate[i].RegVal;
				rate = s_nRFxBaudrate[i].Baud;
			}
		    break;
		}
	}

	return rate;
#endif
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
		{/*
			if (pDev->bDma == true && CFifoUsed(dev->pUartDev->hRxFifo) <= 0)
			{
				if (dev->RxDmaCnt > 0)
				{
					dev->pDmaReg->TASKS_DMA.RX.STOP = 1;
				}
			}*/
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
		dev->pUartDev->bRxReady = false;
		dev->pDmaReg->DMA.RX.MAXCNT = NRFX_UART_RXDMA_SIZE;
		dev->pDmaReg->DMA.RX.PTR = (uint32_t)dev->RxDmaMem;
		dev->pDmaReg->TASKS_DMA.RX.START = 1;
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

static int nRFUARTTxData(DevIntrf_t * const pDev, uint8_t *pData, int Datalen)
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
//            	dev->pUartDev->TxDropCnt++;
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
			int l = NRFX_UART_TXDMA_SIZE;//min(CFifoUsed(dev->pUartDev->hTxFifo), NRF52_UART_DMA_MAX_LEN);
			uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
			if (p)
			{
				dev->pUartDev->bTxReady = false;

				// Transfer to tx cache before sending as CFifo will immediately make the memory
				// block available for reuse in the Put request. This could cause an overwrite
				// if uart tx has not completed in time.
				memcpy(dev->TxDmaMem, p, l);

				dev->pDmaReg->DMA.TX.MAXCNT = l;
				dev->pDmaReg->DMA.TX.PTR = (uint32_t)dev->TxDmaMem;
				dev->pDmaReg->TASKS_DMA.TX.START = 1;
        	}
        }
    }

    if (rtry <= 0)
    {
    	dev->pUartDev->TxDropCnt += Datalen- cnt;
    }

    return cnt;
}

static void nRFUARTStopTx(DevIntrf_t * const pDev)
{
}

static void nRFUARTDisable(DevIntrf_t * const pDev)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;
	NRF_UARTE_Type *reg = dev->pDmaReg;

	reg->TASKS_DMA.RX.STOP = 1;
	reg->TASKS_DMA.TX.STOP = 1;
	reg->PSEL.RXD = -1;
	reg->PSEL.TXD = -1;
	reg->PSEL.RTS = -1;
	reg->PSEL.CTS = -1;
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

	dev->pDmaReg->PSEL.RXD = dev->RxPin;
	dev->pDmaReg->PSEL.TXD = dev->TxPin;
	dev->pDmaReg->PSEL.CTS = dev->CtsPin;
	dev->pDmaReg->PSEL.RTS = dev->RtsPin;

	dev->pDmaReg->ENABLE |= (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);
	dev->pDmaReg->DMA.RX.MAXCNT = NRFX_UART_RXDMA_SIZE;
	dev->pDmaReg->DMA.RX.PTR = (uint32_t)dev->RxDmaMem;
	dev->pDmaReg->EVENTS_DMA.RX.END = 0;
	dev->pDmaReg->TASKS_DMA.RX.START = 1;
	dev->pUartDev->bTxReady = true;
}

void nRFUARTPowerOff(DevIntrf_t * const pDev)
{
	nRFUartDev_t *dev = (nRFUartDev_t *)pDev->pDevData;
	NRF_UARTE_Type *reg = dev->pDmaReg;

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
//	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
//	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;
//	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 0;

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
#if 0
	// Start HF clock if needed
	if (NRF_CLOCK->XO.STAT == 0)
	{
		NRF_CLOCK->TASKS_XOSTART = 1;
		int timout = 1000000;
		while (timout-- > 0 && NRF_CLOCK->EVENTS_XOSTARTED == 0);

		if (timout <= 0)
			return false;

		NRF_CLOCK->EVENTS_XOSTARTED = 0;
	}
#endif

	int devno = pCfg->DevNo;
	NRF_UARTE_Type *reg = s_nRFxUARTDev[devno].pDmaReg;

	// Force power on in case it was powered off previously
	//*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	//*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;

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

	pDev->DevIntrf.pDevData = &s_nRFxUARTDev[devno];
	s_nRFxUARTDev[devno].pUartDev = pDev;

	// DMA mode avail only, force DMA
	pDev->DevIntrf.bDma = true;

	s_nRFxUARTDev[devno].RxPin = (pincfg[UARTPIN_RX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RX_IDX].PortNo << 5);
	s_nRFxUARTDev[devno].TxPin = (pincfg[UARTPIN_TX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_TX_IDX].PortNo << 5);
	s_nRFxUARTDev[devno].pDmaReg->PSEL.RXD = s_nRFxUARTDev[devno].RxPin;
	s_nRFxUARTDev[devno].pDmaReg->PSEL.TXD = s_nRFxUARTDev[devno].TxPin;
    //s_nRFxUARTDev[devno].pDmaReg->CONFIG &= ~(UARTE_CONFIG_PARITY_Msk << UARTE_CONFIG_PARITY_Pos);

    uint32_t cnf = (pCfg->DataBits << UARTE_CONFIG_FRAMESIZE_Pos) & UARTE_CONFIG_FRAMESIZE_Msk;


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
//    	s_nRFxUARTDev[devno].pDmaReg->CONFIG |= (UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
    	s_nRFxUARTDev[devno].CtsPin = (pincfg[UARTPIN_CTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_CTS_IDX].PortNo << 5);
    	s_nRFxUARTDev[devno].RtsPin = (pincfg[UARTPIN_RTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RTS_IDX].PortNo << 5);
    	s_nRFxUARTDev[devno].pDmaReg->PSEL.CTS = s_nRFxUARTDev[devno].CtsPin;
    	s_nRFxUARTDev[devno].pDmaReg->PSEL.RTS = s_nRFxUARTDev[devno].RtsPin;

    	IOPinClear(pincfg[UARTPIN_CTS_IDX].PortNo, pincfg[UARTPIN_CTS_IDX].PinNo);
        IOPinClear(pincfg[UARTPIN_RTS_IDX].PortNo, pincfg[UARTPIN_RTS_IDX].PinNo);
	}
	else
	{
//		s_nRFxUARTDev[devno].pDmaReg->CONFIG &= ~(UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
		s_nRFxUARTDev[devno].pDmaReg->PSEL.RTS = -1;
		s_nRFxUARTDev[devno].pDmaReg->PSEL.CTS = -1;
		s_nRFxUARTDev[devno].CtsPin = -1;
		s_nRFxUARTDev[devno].RtsPin = -1;
	}

    cnf |= UARTE_CONFIG_ENDIAN_LSB << UARTE_CONFIG_ENDIAN_Pos;
    cnf |= UARTE_CONFIG_FRAMETIMEOUT_Msk;

    reg->CONFIG = cnf;

    // Set baud
    pDev->Rate = nRFUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

    reg->FRAMETIMEOUT = 8;
    reg->EVENTS_RXDRDY = 0;
	reg->EVENTS_TXDRDY = 0;
	reg->EVENTS_ERROR = 0;
	reg->EVENTS_RXTO = 0;
	reg->EVENTS_TXSTOPPED = 0;
	reg->EVENTS_CTS = 0;
	reg->ERRORSRC = 0xFFFFFFFF;


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
//	pDev->DevIntrf.bDma = pCfg->bDmaEn;
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

	s_nRFxUARTDev[devno].pDmaReg->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

	s_nRFxUARTDev[devno].pDmaReg->DMA.RX.MAXCNT = NRFX_UART_RXDMA_SIZE;
	s_nRFxUARTDev[devno].pDmaReg->DMA.RX.PTR = (uint32_t)s_nRFxUARTDev[devno].RxDmaMem;
	s_nRFxUARTDev[devno].pDmaReg->EVENTS_DMA.RX.END = 0;
	s_nRFxUARTDev[devno].pDmaReg->TASKS_DMA.RX.START = 1;

    reg->INTENCLR = 0xffffffffUL;

	if (pCfg->bIntMode)
	{
		SharedIntrfSetIrqHandler(pCfg->DevNo, &pDev->DevIntrf, UartIrqHandler);

		if (pDev->DevIntrf.bDma == true)
		{
			s_nRFxUARTDev[devno].pDmaReg->INTENSET = (UARTE_INTENSET_RXDRDY_Set << UARTE_INTENSET_RXDRDY_Pos) |
							  (UARTE_INTENSET_RXTO_Set << UARTE_INTENSET_RXTO_Pos) |
							  (UARTE_INTENSET_TXDRDY_Set << UARTE_INTENSET_TXDRDY_Pos) |
							  (UARTE_INTENSET_ERROR_Set << UARTE_INTENSET_ERROR_Pos) |
							  (UARTE_INTENSET_CTS_Set << UARTE_INTENSET_CTS_Pos) |
							  (UARTE_INTENSET_NCTS_Set << UARTE_INTENSET_NCTS_Pos) |
							  (UARTE_INTENSET_DMARXEND_Set << UARTE_INTENSET_DMARXEND_Pos) |
							  (UARTE_INTENSET_DMATXEND_Set << UARTE_INTENSET_DMATXEND_Pos) |
							  (UARTE_INTEN_FRAMETIMEOUT_Enabled << UARTE_INTEN_FRAMETIMEOUT_Pos);
		}

		switch (devno)
		{
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
        }
    }


	return true;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
//	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->SerIntrf.pDevData;

}

UARTDev_t * const UARTGetInstance(int DevNo)
{
	return s_nRFxUARTDev[DevNo].pUartDev;
}
