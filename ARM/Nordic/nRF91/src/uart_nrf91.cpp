/**-------------------------------------------------------------------------
@file	uart_nrf91.cpp

@brief	nRF91 UART implementation

@author	Nguyen Hoan Hoang
@date	Mar. 12, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "interrupt.h"
#include "shared_irq_nrf91.h"

#define NRF91_UART_DMA_MAX_LEN		(0x1FFF)

// Default fifo size if one is not provided is not provided in the config.
#define NRF91_UART_BUFF_SIZE		(32)

#define NRF91_UART_CFIFO_SIZE		CFIFO_MEMSIZE(NRF91_UART_BUFF_SIZE)

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct _nRF_UART_Dev {
	int DevNo;				// UART interface number
	NRF_UARTE_Type *pReg;		// UART registers
	UARTDEV	*pUartDev;				// Pointer to generic UART dev. data
	uint32_t RxPin;
	uint32_t TxPin;
	uint32_t CtsPin;
	uint32_t RtsPin;
	uint8_t RxFifoMem[NRF91_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[NRF91_UART_CFIFO_SIZE];
	uint8_t TxDmaCache[NRF91_UART_BUFF_SIZE];
	uint8_t RxDmaCache;
} NRF91_UARTDEV;

typedef struct {
	uint32_t Baud;
	uint32_t nRFBaud;
} NRFRATECVT;

#pragma pack(pop)

const NRFRATECVT s_BaudnRF[] = {
	{1200, UARTE_BAUDRATE_BAUDRATE_Baud1200 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{2400, UARTE_BAUDRATE_BAUDRATE_Baud2400 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{4800, UARTE_BAUDRATE_BAUDRATE_Baud4800 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{9600, UARTE_BAUDRATE_BAUDRATE_Baud9600 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{14400, UARTE_BAUDRATE_BAUDRATE_Baud14400 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{19200, UARTE_BAUDRATE_BAUDRATE_Baud19200 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{28800, UARTE_BAUDRATE_BAUDRATE_Baud28800 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{38400, UARTE_BAUDRATE_BAUDRATE_Baud38400 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{57600, UARTE_BAUDRATE_BAUDRATE_Baud57600 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{76800, UARTE_BAUDRATE_BAUDRATE_Baud76800 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{115200, UARTE_BAUDRATE_BAUDRATE_Baud115200 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{230400, UARTE_BAUDRATE_BAUDRATE_Baud230400 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{250000, UARTE_BAUDRATE_BAUDRATE_Baud250000 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{460800, UARTE_BAUDRATE_BAUDRATE_Baud460800 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{921600, UARTE_BAUDRATE_BAUDRATE_Baud921600 << UARTE_BAUDRATE_BAUDRATE_Pos},
	{1000000, UARTE_BAUDRATE_BAUDRATE_Baud1M << UARTE_BAUDRATE_BAUDRATE_Pos}
};

static const int s_NbBaudnRF = sizeof(s_BaudnRF) / sizeof(NRFRATECVT);

static NRF91_UARTDEV s_nRFUartDev[] = {
	{
		.DevNo = 0,
		.pReg = NRF_UARTE0_S,
		.pUartDev = NULL,
		0, 0, 0,
	},
	{
		.DevNo = 1,
		.pReg = NRF_UARTE1_S,
		.pUartDev = NULL,
		0, 0, 0,
	},
	{
		.DevNo = 2,
		.pReg = NRF_UARTE2_S,
		.pUartDev = NULL,
		0, 0, 0,
	},
	{
		.DevNo = 3,
		.pReg = NRF_UARTE3_S,
		.pUartDev = NULL,
		0, 0, 0,
	},
};

static const int s_NbUartDev = sizeof(s_nRFUartDev) / sizeof(NRF91_UARTDEV);

bool nRFUARTWaitForRxReady(NRF91_UARTDEV * const pDev, uint32_t Timeout)
{
	do {
		if (pDev->pReg->EVENTS_RXDRDY || pDev->pUartDev->bRxReady)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool nRFUARTWaitForTxReady(NRF91_UARTDEV * const pDev, uint32_t Timeout)
{
	do {
		if (pDev->pReg->EVENTS_TXDRDY || pDev->pUartDev->bTxReady == true)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

void UartIrqHandler(int DevNo, DEVINTRF * const pDev)
{
	NRF91_UARTDEV *dev = (NRF91_UARTDEV *)pDev-> pDevData;
	int len = 0;
	int cnt = 0;

	uint8_t rxto = dev->pReg->EVENTS_RXTO;

	if (rxto)
	{
		dev->pReg->TASKS_FLUSHRX = 1;
		dev->pReg->EVENTS_RXTO = 0;
	}

	if (dev->pReg->EVENTS_RXDRDY)
	{
		dev->pReg->EVENTS_RXDRDY = 0;
	}

	if (dev->pReg->EVENTS_ENDRX)
	{
		dev->pReg->EVENTS_ENDRX = 0;

		uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
		if (p)
		{
			*p = dev->RxDmaCache;
		}
		else
		{
			dev->pUartDev->RxDropCnt++;
		}

		// We need to transfer only 1 byte at a time for Rx. Otherwise, it will not interrupt
		// until buffer is filled. It will be blocked.
		// The RX timeout logic of the nRF series is implemented wrong. We cannot use it.
		dev->pUartDev->bRxReady = false;
		dev->pReg->RXD.MAXCNT = 1;
		dev->pReg->RXD.PTR = (uint32_t)&dev->RxDmaCache;
		dev->pReg->TASKS_STARTRX = 1;

		if (dev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(dev->pUartDev->hRxFifo);
			cnt = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, len);
		}
	}

	if (dev->pReg->EVENTS_TXDRDY)
	{

		dev->pReg->EVENTS_TXDRDY = 0;
	}

	if (dev->pReg->EVENTS_ENDTX || dev->pReg->EVENTS_TXSTOPPED)
	{
		dev->pReg->EVENTS_ENDTX = 0;
		dev->pReg->TASKS_STOPTX = 1;
		dev->pReg->EVENTS_TXSTOPPED = 0;

		int l = NRF91_UART_BUFF_SIZE;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
		if (p)
		{
			dev->pUartDev->bTxReady = false;

			// Transfer to tx cache before sending as CFifo will immediately make the memory
			// block available for reuse in the Put request. Direct use of fifo buffer could
			// cause an overwrite if uart tx has not completed in time.
			memcpy(&dev->TxDmaCache, p, l);

			dev->pReg->TXD.MAXCNT = l;
			dev->pReg->TXD.PTR = (uint32_t)dev->TxDmaCache;
			dev->pReg->TASKS_STARTTX = 1;
		}
		else
		{
			dev->pUartDev->bTxReady = true;
		}
		if (dev->pUartDev->EvtCallback)
		{
			//uint8_t buff[NRFUART_CFIFO_SIZE];

			//len = min(NRFUART_CFIFO_SIZE, CFifoAvail(s_nRFUartDev.pUartDev->hTxFifo));
			len = CFifoAvail(dev->pUartDev->hTxFifo);
			len = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, len);
			if (len > 0)
			{
				//s_nRFUartDev.bTxReady = false;
				//nRFUARTTxData(&s_nRFUartDev.pUartDev->SerIntrf, buff, len);
			}
		}
	}

	if (dev->pReg->EVENTS_ERROR)
	{
		dev->pReg->EVENTS_ERROR = 0;
		uint32_t err = dev->pReg->ERRORSRC;
		if (err & UARTE_ERRORSRC_OVERRUN_Msk)
		{
			dev->pReg->TASKS_FLUSHRX;
			dev->pUartDev->RxOvrErrCnt++;

		}
		if (err & UARTE_ERRORSRC_PARITY_Msk)
		{
			// Parity error
			dev->pUartDev->ParErrCnt++;
		}
		if (err & UARTE_ERRORSRC_FRAMING_Msk)
		{
			// Framing error
			dev->pUartDev->FramErrCnt++;
		}
		dev->pReg->ERRORSRC = dev->pReg->ERRORSRC;
		len = 0;
		if (dev->pUartDev->EvtCallback)
		{
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, NULL, len);
		}
		dev->pReg->TASKS_STARTRX = 1;
	}

	if (dev->pReg->EVENTS_CTS)
	{
		dev->pReg->EVENTS_CTS = 0;
		dev->pUartDev->LineState &= ~UART_LINESTATE_CTS;
        if (dev->pUartDev->EvtCallback)
        {
            uint8_t buff = 0;
            len = 1;
            dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, &buff, len);
        }
		//NRF_UART0->TASKS_STARTTX = 1;
		//s_nRFUartDev.bTxReady = true;
	}

	if (dev->pReg->EVENTS_NCTS)
	{
		dev->pReg->EVENTS_NCTS = 0;
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

static inline int nRFUARTGetRate(DEVINTRF * const pDev) {
	return ((NRF91_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

static int nRFUARTSetRate(DEVINTRF * const pDev, int Rate)
{
	NRF91_UARTDEV *dev = (NRF91_UARTDEV *)pDev->pDevData;

	int rate = 0;

	for (int i = 0; i < s_NbBaudnRF; i++)
	{
		if (s_BaudnRF[i].Baud >= Rate)
		{
		    dev->pReg->BAUDRATE = s_BaudnRF[i].nRFBaud;

		    rate = s_BaudnRF[i].Baud;
		    break;
		}
	}

	return rate;
}

static inline bool nRFUARTStartRx(DEVINTRF * const pSerDev, int DevAddr) {
	return true;
}

static int nRFUARTRxData(DEVINTRF * const pDev, uint8_t *pBuff, int Bufflen)
{
	NRF91_UARTDEV *dev = (NRF91_UARTDEV *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Bufflen)
	{
		int l  = Bufflen;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &l);
		if (p == NULL)
			break;
		memcpy(pBuff, p, l);
		cnt += l;
		pBuff += l;
		Bufflen -= l;
	}
	EnableInterrupt(state);

	if (dev->pUartDev->bRxReady)
	{
		dev->pUartDev->bRxReady = false;
		dev->pReg->RXD.MAXCNT = 1;
		dev->pReg->RXD.PTR = (uint32_t)&dev->RxDmaCache;
		dev->pReg->TASKS_STARTRX = 1;
	}

	return cnt;
}

static inline void nRFUARTStopRx(DEVINTRF * const pDev) {
}

static inline bool nRFUARTStartTx(DEVINTRF * const pDev, int DevAddr) {
	return true;
}

static int nRFUARTTxData(DEVINTRF * const pDev, uint8_t *pData, int Datalen)
{
	NRF91_UARTDEV *dev = (NRF91_UARTDEV *)pDev->pDevData;
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
                break;
            memcpy(p, pData, l);
            Datalen -= l;
            pData += l;
            cnt += l;
        }
        EnableInterrupt(state);

        if (dev->pUartDev->bTxReady)
        {

			int l = NRF91_UART_BUFF_SIZE;//min(CFifoUsed(dev->pUartDev->hTxFifo), NRF52_UART_DMA_MAX_LEN);
			uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
			if (p)
			{
				dev->pUartDev->bTxReady = false;

				// Transfer to tx cache before sending as CFifo will immediately make the memory
				// block available for reuse in the Put request. This could cause an overwrite
				// if uart tx has not completed in time.
				memcpy(dev->TxDmaCache, p, l);

				dev->pReg->TXD.MAXCNT = l;
				dev->pReg->TXD.PTR = (uint32_t)dev->TxDmaCache;
				dev->pReg->TASKS_STARTTX = 1;
			}
        }
    }
    return cnt;
}

static inline void nRFUARTStopTx(DEVINTRF * const pDev) {
}

static void nRFUARTDisable(DEVINTRF * const pDev)
{
	NRF91_UARTDEV *dev = (NRF91_UARTDEV *)pDev->pDevData;

	dev->pReg->TASKS_STOPRX = 1;
	dev->pReg->TASKS_STOPTX = 1;

	dev->pReg->PSEL.RXD = -1;
	dev->pReg->PSEL.TXD = -1;
	dev->pReg->PSEL.RTS = -1;
	dev->pReg->PSEL.CTS = -1;

	dev->pReg->ENABLE = 0;
}

static void nRFUARTEnable(DEVINTRF * const pDev)
{
	NRF91_UARTDEV *dev = (NRF91_UARTDEV *)pDev->pDevData;

	dev->pUartDev->RxOvrErrCnt = 0;
	dev->pUartDev->ParErrCnt = 0;
	dev->pUartDev->FramErrCnt = 0;
	dev->pUartDev->RxDropCnt = 0;
	dev->pUartDev->TxDropCnt = 0;

	dev->pReg->PSEL.RXD = dev->RxPin;
	dev->pReg->PSEL.TXD = dev->TxPin;
	dev->pReg->PSEL.CTS = dev->CtsPin;
	dev->pReg->PSEL.RTS = dev->RtsPin;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;
	dev->pUartDev->bRxReady = false;

	dev->pReg->ENABLE |= (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

	dev->pReg->RXD.MAXCNT = 1;
	dev->pReg->RXD.PTR = (uint32_t)&dev->RxDmaCache;
	dev->pReg->TASKS_STARTRX = 1;
}

void nRFUARTPowerOff(DEVINTRF * const pDev)
{
	NRF91_UARTDEV *dev = (NRF91_UARTDEV *)pDev->pDevData;

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 0;
}

static void apply_workaround_for_enable_anomaly(NRF91_UARTDEV * const pDev)
{
#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || defined(NRF9160_XXAA)
    // Apply workaround for anomalies:
    // - nRF9160 - anomaly 23
    // - nRF5340 - anomaly 44
    volatile uint32_t const * rxenable_reg =
        (volatile uint32_t *)(((uint32_t)pDev->pReg) + 0x564);
    volatile uint32_t const * txenable_reg =
        (volatile uint32_t *)(((uint32_t)pDev->pReg) + 0x568);

    if (*txenable_reg == 1)
    {
    	pDev->pReg->TASKS_STOPTX = 1;
    }

    if (*rxenable_reg == 1)
    {
    	pDev->pReg->ENABLE = UARTE_ENABLE_ENABLE_Msk;
    	pDev->pReg->TASKS_STOPRX = 1;

        while (*rxenable_reg) {}

        pDev->pReg->ERRORSRC = pDev->pReg->ERRORSRC;

        pDev->pReg->ENABLE = 0;
    }
#endif // defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || defined(NRF9160_XXAA)
}

bool UARTInit(UARTDEV * const pDev, const UARTCFG *pCfg)
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

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)s_nRFUartDev[devno].pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)s_nRFUartDev[devno].pReg + 0xFFC) = 1;

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_nRFUartDev[devno].RxFifoMem, NRF91_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_nRFUartDev[devno].TxFifoMem, NRF91_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	IOPINCFG *pincfg = (IOPINCFG*)pCfg->pIOPinMap;

	IOPinSet(pincfg[UARTPIN_TX_IDX].PortNo, pincfg[UARTPIN_TX_IDX].PinNo);
	IOPinCfg(pincfg, pCfg->NbIOPins);

	pDev->DevIntrf.pDevData = &s_nRFUartDev[devno];
	s_nRFUartDev[devno].pUartDev = pDev;

	// nRF91 only supports DMA mode
	pDev->DevIntrf.bDma = true;

	s_nRFUartDev[devno].RxPin = (pincfg[UARTPIN_RX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RX_IDX].PortNo << 5);
	s_nRFUartDev[devno].TxPin = (pincfg[UARTPIN_TX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_TX_IDX].PortNo << 5);
	s_nRFUartDev[devno].pReg->PSEL.RXD = s_nRFUartDev[devno].RxPin;
	s_nRFUartDev[devno].pReg->PSEL.TXD = s_nRFUartDev[devno].TxPin;

    // Set baud
    pDev->Rate = nRFUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

    s_nRFUartDev[devno].pReg->CONFIG &= ~(UARTE_CONFIG_PARITY_Msk << UARTE_CONFIG_PARITY_Pos);
	if (pCfg->Parity == UART_PARITY_NONE)
	{
		s_nRFUartDev[devno].pReg->CONFIG |= UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos;
	}
	else
	{
		s_nRFUartDev[devno].pReg->CONFIG |= UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos;
	}

	s_nRFUartDev[devno].pReg->EVENTS_RXDRDY = 0;
	s_nRFUartDev[devno].pReg->EVENTS_TXDRDY = 0;
	s_nRFUartDev[devno].pReg->EVENTS_ERROR = 0;
	s_nRFUartDev[devno].pReg->EVENTS_RXTO = 0;
	s_nRFUartDev[devno].pReg->ERRORSRC = NRF_UARTE0_S->ERRORSRC;
	s_nRFUartDev[devno].pReg->EVENTS_CTS = 0;

	s_nRFUartDev[devno].pReg->EVENTS_RXSTARTED = 0;
	s_nRFUartDev[devno].pReg->EVENTS_TXSTARTED = 0;
	s_nRFUartDev[devno].pReg->EVENTS_TXSTOPPED = 0;

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	s_nRFUartDev[devno].pReg->CONFIG |= (UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
    	s_nRFUartDev[devno].CtsPin = (pincfg[UARTPIN_CTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_CTS_IDX].PortNo << 5);
    	s_nRFUartDev[devno].RtsPin = (pincfg[UARTPIN_RTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RTS_IDX].PortNo << 5);
    	s_nRFUartDev[devno].pReg->PSEL.CTS = s_nRFUartDev[devno].CtsPin;
    	s_nRFUartDev[devno].pReg->PSEL.RTS = s_nRFUartDev[devno].RtsPin;
		NRF_P0_S->OUTCLR = (1 << pincfg[UARTPIN_CTS_IDX].PinNo);
		NRF_P0_S->OUTCLR = (1 << pincfg[UARTPIN_RTS_IDX].PinNo);
	}
	else
	{
		s_nRFUartDev[devno].pReg->CONFIG &= ~(UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
		s_nRFUartDev[devno].pReg->PSEL.RTS = -1;
		s_nRFUartDev[devno].pReg->PSEL.CTS = -1;
		s_nRFUartDev[devno].CtsPin = -1;
		s_nRFUartDev[devno].RtsPin = -1;
	}


	s_nRFUartDev[devno].pUartDev->bRxReady = false;
	s_nRFUartDev[devno].pUartDev->bTxReady = true;
	s_nRFUartDev[devno].pUartDev->RxOvrErrCnt = 0;
	s_nRFUartDev[devno].pUartDev->ParErrCnt = 0;
	s_nRFUartDev[devno].pUartDev->FramErrCnt = 0;
	s_nRFUartDev[devno].pUartDev->RxDropCnt = 0;
	s_nRFUartDev[devno].pUartDev->TxDropCnt = 0;

	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->Parity = pCfg->Parity;
	pDev->bIntMode = pCfg->bIntMode;
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
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	apply_workaround_for_enable_anomaly(&s_nRFUartDev[devno]);

	s_nRFUartDev[devno].pReg->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

	// We need to transfer only 1 byte at a time for Rx. Otherwise, it will not interrupt
	// until buffer is filled. It will be blocked.
	// The RX timeout logic of the nRF series is implemented wrong. We cannot use it.
	s_nRFUartDev[devno].pReg->RXD.MAXCNT = 1;
	s_nRFUartDev[devno].pReg->RXD.PTR = (uint32_t)&s_nRFUartDev[devno].RxDmaCache;

	s_nRFUartDev[devno].pReg->EVENTS_ENDRX = 0;
	s_nRFUartDev[devno].pReg->TASKS_STARTRX = 1;
//	s_nRFUartDev[devno].pReg->TASKS_STARTTX = 1;

    s_nRFUartDev[devno].pReg->INTENCLR = 0xffffffffUL;

	if (pCfg->bIntMode)
	{
		SetSharedIntHandler(pCfg->DevNo, &pDev->DevIntrf, UartIrqHandler);

		s_nRFUartDev[devno].pReg->INTENSET = (UARTE_INTENSET_RXDRDY_Set << UARTE_INTENSET_RXDRDY_Pos) |
							  (UARTE_INTENSET_RXTO_Set << UARTE_INTENSET_RXTO_Pos) |
							  (UARTE_INTENSET_TXDRDY_Set << UARTE_INTENSET_TXDRDY_Pos) |
							  (UARTE_INTENSET_ERROR_Set << UARTE_INTENSET_ERROR_Pos) |
							  (UARTE_INTENSET_CTS_Set << UARTE_INTENSET_CTS_Pos) |
							  (UARTE_INTENSET_NCTS_Set << UARTE_INTENSET_NCTS_Pos) |
							  (UARTE_INTENSET_ENDTX_Set << UARTE_INTENSET_ENDTX_Pos) |
							  (UARTE_INTENSET_ENDRX_Set << UARTE_INTENSET_ENDRX_Pos);

		switch (devno)
		{
			case 0:
				NVIC_ClearPendingIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
				NVIC_SetPriority(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
				break;
			case 1:
				NVIC_ClearPendingIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
				NVIC_SetPriority(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
				break;
			case 2:
				NVIC_ClearPendingIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
				NVIC_SetPriority(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
				break;
			case 3:
				NVIC_ClearPendingIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
				NVIC_SetPriority(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
				break;
        }
    }

	return true;
}

void UARTSetCtrlLineState(UARTDEV * const pDev, uint32_t LineState)
{
//	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->SerIntrf.pDevData;

}

UARTDEV * const UARTGetInstance(int DevNo)
{
	return s_nRFUartDev[DevNo].pUartDev;
}

