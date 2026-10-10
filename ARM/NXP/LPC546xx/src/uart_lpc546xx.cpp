/**-------------------------------------------------------------------------
@file	uart_lpc546xx.cpp

@brief	LPC546xx Flexcomm USART implementation

		DevNo is the Flexcomm number, 0 to 9. The function clock is the
		12 MHz FRO, independent of the core clock. Polling and interrupt
		modes. Both use the 16 entry hardware FIFOs. In interrupt mode the
		CFIFOs are filled and drained by the Flexcomm interrupt. TX DMA is not
		supported, a configuration with bDMAMode fails UARTInit.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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

#include "LPC546xx.h"

#include "istddef.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "coredev/shared_intrf.h"
#include "flexcomm_lpc546xx.h"

// Default CFIFO memory of each instance, used when the configuration does
// not supply its own
#define LPC546XX_UART_BUFF_SIZE		16
#define LPC546XX_UART_CFIFO_SIZE	CFIFO_MEMSIZE(LPC546XX_UART_BUFF_SIZE)

// Baud rate = function clock / ((OSRVAL + 1) * (BRGVAL + 1))
#define LPC546XX_USART_OSRVAL_MIN	4U			// 5 clocks per bit
#define LPC546XX_USART_OSRVAL_MAX	15U			// 16 clocks per bit
#define LPC546XX_USART_BRG_MAX		0x10000U	// BRGVAL + 1

// CFG field values
#define LPC546XX_USART_DATALEN_7	0U
#define LPC546XX_USART_DATALEN_8	1U
#define LPC546XX_USART_PARITY_EVEN	2U
#define LPC546XX_USART_PARITY_ODD	3U

// The TX FIFO interrupt comes when the FIFO is down to this number of
// entries, the RX FIFO interrupt on each received character.
#define LPC546XX_USART_TXLVL		8U
#define LPC546XX_USART_RXLVL		0U

// Receive errors reported with each character in FIFORD
#define LPC546XX_USART_FIFORD_ERR	(USART_FIFORD_FRAMERR_MASK | USART_FIFORD_PARITYERR_MASK)

#pragma pack(push, 4)

/// Device driver data require by low level functions
typedef struct __Lpc546xx_Uart_Dev {
	USART_Type *pReg;				//!< Flexcomm USART registers
	int DevNo;						//!< Flexcomm number
	uint32_t FClk;					//!< Function clock frequency
	UARTDev_t *pUartDev;			//!< Pointer to generic UART dev. data
	const IOPinCfg_t *pIOPinMap;	//!< Pins configured again by Enable
	int NbIOPins;					//!< Number of pins in pIOPinMap
	alignas(4) uint8_t RxFifoMem[LPC546XX_UART_CFIFO_SIZE];	//!< Default RX CFIFO memory
	alignas(4) uint8_t TxFifoMem[LPC546XX_UART_CFIFO_SIZE];	//!< Default TX CFIFO memory
} Lpc546xxUartDev_t;

#pragma pack(pop)

static Lpc546xxUartDev_t s_Lpc546xxUartDev[] = {
	{ .pReg = USART0, .DevNo = 0, },
	{ .pReg = USART1, .DevNo = 1, },
	{ .pReg = USART2, .DevNo = 2, },
	{ .pReg = USART3, .DevNo = 3, },
	{ .pReg = USART4, .DevNo = 4, },
	{ .pReg = USART5, .DevNo = 5, },
	{ .pReg = USART6, .DevNo = 6, },
	{ .pReg = USART7, .DevNo = 7, },
	{ .pReg = USART8, .DevNo = 8, },
	{ .pReg = USART9, .DevNo = 9, },
};

static const int s_NbUartDev = sizeof(s_Lpc546xxUartDev) / sizeof(Lpc546xxUartDev_t);

static uint8_t Lpc546xxUartRxRead(Lpc546xxUartDev_t * const pDev, uint32_t *pData);
static void Lpc546xxUartTxFill(Lpc546xxUartDev_t * const pDev);
static void Lpc546xxUartIrqHandler(int DevNo, DevIntrf_t * const pDev);

static uint32_t Lpc546xxUartGetRate(DevIntrf_t * const pDev);
static uint32_t Lpc546xxUartSetRate(DevIntrf_t * const pDev, uint32_t Rate);
static bool Lpc546xxUartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Lpc546xxUartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen);
static void Lpc546xxUartStopRx(DevIntrf_t * const pDev);
static bool Lpc546xxUartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Lpc546xxUartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen);
static void Lpc546xxUartStopTx(DevIntrf_t * const pDev);
static void Lpc546xxUartDisable(DevIntrf_t * const pDev);
static void Lpc546xxUartEnable(DevIntrf_t * const pDev);
static void Lpc546xxUartReset(DevIntrf_t * const pDev);
static void *Lpc546xxUartGetHandle(DevIntrf_t * const pDev);

// Read one character from the RX FIFO, which must not be empty. Returns the
// FIFORD error flags, 0 when the character is good. Frame and parity errors
// are counted.
static uint8_t Lpc546xxUartRxRead(Lpc546xxUartDev_t * const pDev, uint32_t *pData)
{
	uint32_t d = pDev->pReg->FIFORD;

	if (d & USART_FIFORD_FRAMERR_MASK)
	{
		pDev->pUartDev->FramErrCnt++;
	}
	if (d & USART_FIFORD_PARITYERR_MASK)
	{
		pDev->pUartDev->ParErrCnt++;
	}

	*pData = d & (pDev->pUartDev->DataBits == 7 ? 0x7FU : 0xFFU);

	return (d & LPC546XX_USART_FIFORD_ERR) != 0U;
}

// Move TX CFIFO data to the hardware FIFO until one of them is full or
// empty. Called with interrupts disabled or from the interrupt handler.
static void Lpc546xxUartTxFill(Lpc546xxUartDev_t * const pDev)
{
	USART_Type *reg = pDev->pReg;

	while (reg->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK)
	{
		uint8_t *p = CFifoGet(pDev->pUartDev->hTxFifo);
		if (p == NULL)
		{
			break;
		}
		reg->FIFOWR = *p;
	}
}

static void Lpc546xxUartIrqHandler(int DevNo, DevIntrf_t * const pDev)
{
	Lpc546xxUartDev_t *dev = (Lpc546xxUartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	USART_Type *reg = dev->pReg;
	uint32_t stat = reg->FIFOSTAT;

	(void)DevNo;

	// RXERR is an RX FIFO overflow, TXERR a write to a full TX FIFO. Both
	// are cleared by writing 1.
	if (stat & USART_FIFOSTAT_RXERR_MASK)
	{
		udev->RxOvrErrCnt++;
	}
	if (stat & (USART_FIFOSTAT_RXERR_MASK | USART_FIFOSTAT_TXERR_MASK))
	{
		reg->FIFOSTAT = stat & (USART_FIFOSTAT_RXERR_MASK | USART_FIFOSTAT_TXERR_MASK);
	}

	if (stat & USART_FIFOSTAT_RXNOTEMPTY_MASK)
	{
		do {
			uint32_t d;

			if (Lpc546xxUartRxRead(dev, &d) == 0U)
			{
				uint8_t *p = CFifoPut(udev->hRxFifo);
				if (p != NULL)
				{
					*p = (uint8_t)d;
					udev->bRxReady = true;
				}
				else
				{
					udev->RxDropCnt++;
				}
			}
		} while (reg->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK);

		if (udev->EvtCallback)
		{
			udev->EvtCallback(udev, UART_EVT_RXDATA, NULL, CFifoUsed(udev->hRxFifo));
		}
	}

	if (reg->FIFOINTENSET & USART_FIFOINTENSET_TXLVL_MASK)
	{
		Lpc546xxUartTxFill(dev);

		if (CFifoUsed(udev->hTxFifo) == 0)
		{
			// Nothing left to send, the TX level interrupt is off until
			// Lpc546xxUartTxData has data again.
			reg->FIFOINTENCLR = USART_FIFOINTENCLR_TXLVL_MASK;
			udev->bTxReady = true;
			if (udev->EvtCallback)
			{
				udev->EvtCallback(udev, UART_EVT_TXREADY, NULL, 0);
			}
		}
	}
}

static uint32_t Lpc546xxUartGetRate(DevIntrf_t * const pDev)
{
	return ((Lpc546xxUartDev_t *)pDev->pDevData)->pUartDev->Rate;
}

// Select the closest rate the function clock can make and return it.
// Oversampling by 16 is kept unless a lower one is closer.
static uint32_t Lpc546xxUartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Lpc546xxUartDev_t *dev = (Lpc546xxUartDev_t *)pDev->pDevData;
	USART_Type *reg = dev->pReg;
	uint32_t fclk = dev->FClk;

	if (Rate == 0 || fclk == 0)
	{
		return 0;
	}

	uint32_t bestosr = LPC546XX_USART_OSRVAL_MAX;
	uint32_t bestbrg = 1;
	uint32_t bestrate = 0;
	uint32_t besterr = UINT32_MAX;

	for (uint32_t osr = LPC546XX_USART_OSRVAL_MAX; osr >= LPC546XX_USART_OSRVAL_MIN; osr--)
	{
		uint64_t div = (uint64_t)(osr + 1U) * Rate;
		uint64_t brg = ((uint64_t)fclk + (div >> 1)) / div;

		if (brg < 1U)
		{
			brg = 1U;
		}
		else if (brg > LPC546XX_USART_BRG_MAX)
		{
			brg = LPC546XX_USART_BRG_MAX;
		}

		uint32_t rate = (uint32_t)(fclk / ((osr + 1U) * brg));
		uint32_t err = rate > Rate ? rate - Rate : Rate - rate;

		if (err < besterr)
		{
			besterr = err;
			bestosr = osr;
			bestbrg = (uint32_t)brg;
			bestrate = rate;
		}
	}

	// OSR and BRG are changed with the USART disabled. The interrupt handler
	// uses the USART too, so the sequence is not interrupted.
	uint32_t state = DisableInterrupt();
	uint32_t cfg = reg->CFG;

	reg->CFG = cfg & ~USART_CFG_ENABLE_MASK;
	reg->OSR = USART_OSR_OSRVAL(bestosr);
	reg->BRG = USART_BRG_BRGVAL(bestbrg - 1U);
	reg->CFG = cfg;

	EnableInterrupt(state);

	dev->pUartDev->Rate = bestrate;

	return bestrate;
}

static bool Lpc546xxUartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int Lpc546xxUartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	Lpc546xxUartDev_t *dev = (Lpc546xxUartDev_t *)pDev->pDevData;
	USART_Type *reg = dev->pReg;
	int cnt = 0;

	if (pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

	if (!pDev->bIntEn)
	{
		if (reg->FIFOSTAT & USART_FIFOSTAT_RXERR_MASK)
		{
			dev->pUartDev->RxOvrErrCnt++;
			reg->FIFOSTAT = USART_FIFOSTAT_RXERR_MASK;
		}

		// Characters with an error are dropped
		while (cnt < Bufflen && (reg->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK))
		{
			uint32_t d;

			if (Lpc546xxUartRxRead(dev, &d) == 0U)
			{
				pBuff[cnt++] = (uint8_t)d;
			}
		}

		return cnt;
	}

	uint32_t state = DisableInterrupt();

	while (cnt < Bufflen)
	{
		int len = Bufflen - cnt;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &len);
		if (p == NULL)
		{
			break;
		}
		memcpy(&pBuff[cnt], p, len);
		cnt += len;
	}
	dev->pUartDev->bRxReady = CFifoUsed(dev->pUartDev->hRxFifo) != 0;

	EnableInterrupt(state);

	return cnt;
}

static void Lpc546xxUartStopRx(DevIntrf_t * const pDev)
{
}

static bool Lpc546xxUartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int Lpc546xxUartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	Lpc546xxUartDev_t *dev = (Lpc546xxUartDev_t *)pDev->pDevData;
	USART_Type *reg = dev->pReg;
	int cnt = 0;

	if (pData == NULL || Datalen <= 0 || (reg->CFG & USART_CFG_ENABLE_MASK) == 0)
	{
		return 0;
	}

	if (!pDev->bIntEn)
	{
		while (cnt < Datalen && (reg->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK))
		{
			reg->FIFOWR = pData[cnt++];
		}

		return cnt;
	}

	int rtry = pDev->MaxRetry;

	while (Datalen > 0 && rtry-- > 0)
	{
		uint32_t state = DisableInterrupt();

		if ((reg->CFG & USART_CFG_ENABLE_MASK) == 0)
		{
			EnableInterrupt(state);
			break;
		}

		while (Datalen > 0)
		{
			int l = Datalen;
			uint8_t *p = l == 1 ? CFifoPut(dev->pUartDev->hTxFifo) :
						 CFifoPutMultiple(dev->pUartDev->hTxFifo, &l);
			if (p == NULL)
			{
				break;
			}
			if (l == 1)
			{
				*p = *pData;
			}
			else
			{
				memcpy(p, pData, l);
			}
			Datalen -= l;
			pData += l;
			cnt += l;
		}

		// With the TX level interrupt off, the hardware FIFO is filled here.
		// Older data is always taken from the CFIFO first, so the order is
		// kept.
		if (dev->pUartDev->bTxReady)
		{
			Lpc546xxUartTxFill(dev);
		}

		if (CFifoUsed(dev->pUartDev->hTxFifo) > 0)
		{
			dev->pUartDev->bTxReady = false;
			reg->FIFOINTENSET = USART_FIFOINTENSET_TXLVL_MASK;
		}

		EnableInterrupt(state);
	}

	// Datalen is what the CFIFO did not take after all retries.
	if (Datalen > 0)
	{
		dev->pUartDev->TxDropCnt += (uint32_t)Datalen;
	}

	return cnt;
}

static void Lpc546xxUartStopTx(DevIntrf_t * const pDev)
{
}

// Also the PowerOff hook. Registers keep their content while the clock is
// off, Enable restarts the USART without a new UARTInit.
static void Lpc546xxUartDisable(DevIntrf_t * const pDev)
{
	Lpc546xxUartDev_t *dev = (Lpc546xxUartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->FIFOINTENCLR = USART_FIFOINTENCLR_TXLVL_MASK;
	dev->pReg->CFG &= ~USART_CFG_ENABLE_MASK;
	Lpc546xxFlexcommClock(dev->DevNo, false);

	IOPinDis(dev->pIOPinMap, dev->NbIOPins);

	EnableInterrupt(state);
}

static void Lpc546xxUartEnable(DevIntrf_t * const pDev)
{
	Lpc546xxUartDev_t *dev = (Lpc546xxUartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pUartDev->RxOvrErrCnt = 0;
	dev->pUartDev->ParErrCnt = 0;
	dev->pUartDev->FramErrCnt = 0;
	dev->pUartDev->RxDropCnt = 0;
	dev->pUartDev->TxDropCnt = 0;

	CFifoFlush(dev->pUartDev->hTxFifo);
	dev->pUartDev->bTxReady = true;

	Lpc546xxFlexcommClock(dev->DevNo, true);

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

	dev->pReg->FIFOCFG |= USART_FIFOCFG_EMPTYTX_MASK | USART_FIFOCFG_EMPTYRX_MASK;
	dev->pReg->FIFOSTAT = USART_FIFOSTAT_TXERR_MASK | USART_FIFOSTAT_RXERR_MASK;
	dev->pReg->CFG |= USART_CFG_ENABLE_MASK;

	EnableInterrupt(state);
}

// Flexcomm reset, all USART registers back to their reset value with the
// USART function still selected.
static void Lpc546xxUartReset(DevIntrf_t * const pDev)
{
	Lpc546xxUartDev_t *dev = (Lpc546xxUartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	Lpc546xxFlexcommSelect(dev->DevNo, LPC546XX_FLEXCOMM_USART);
	dev->pUartDev->bTxReady = true;

	EnableInterrupt(state);
}

static void *Lpc546xxUartGetHandle(DevIntrf_t * const pDev)
{
	return ((Lpc546xxUartDev_t *)pDev->pDevData)->pUartDev;
}

UARTDev_t const *UARTGetInstance(int DevNo)
{
	return DevNo >= 0 && DevNo < s_NbUartDev ? s_Lpc546xxUartDev[DevNo].pUartDev : NULL;
}

bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
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

	// Polling and interrupt modes, 7 or 8 data bits, no IrDA, no single wire
	// half duplex. RTS and CTS are handled by the USART with
	// UART_FLWCTRL_HW.
	if (pCfg->bDMAMode || pCfg->bIrDAMode || pCfg->Mode != UART_MODE_UART ||
		pCfg->Duplex != UART_DUPLEX_FULL ||
		(pCfg->FlowControl != UART_FLWCTRL_NONE && pCfg->FlowControl != UART_FLWCTRL_HW) ||
		(pCfg->Parity != UART_PARITY_NONE && pCfg->Parity != UART_PARITY_EVEN && pCfg->Parity != UART_PARITY_ODD) ||
		(pCfg->DataBits != 7 && pCfg->DataBits != 8) ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) || pCfg->Rate <= 0)
	{
		return false;
	}

	Lpc546xxUartDev_t *dev = &s_Lpc546xxUartDev[pCfg->DevNo];
	USART_Type *reg = dev->pReg;
	IRQn_Type irq = Lpc546xxFlexcommIrqNo(dev->DevNo);
	uint32_t state = DisableInterrupt();

	// An instance running for another UART object is not taken over. The bus
	// clock is turned on first, Disable may have turned it off.
	Lpc546xxFlexcommClock(dev->DevNo, true);
	if (dev->pUartDev != NULL && dev->pUartDev != pDev && (reg->CFG & USART_CFG_ENABLE_MASK))
	{
		EnableInterrupt(state);

		return false;
	}

	// Bus clock on, Flexcomm reset with the USART function selected. Fails
	// if the Flexcomm is locked to SPI or I2C.
	dev->FClk = Lpc546xxFlexcommSelect(dev->DevNo, LPC546XX_FLEXCOMM_USART);
	if (dev->FClk == 0)
	{
		EnableInterrupt(state);

		return false;
	}

	NVIC_DisableIRQ(irq);

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;
	dev->pIOPinMap = (const IOPinCfg_t *)pCfg->pIOPinMap;
	dev->NbIOPins = pCfg->NbIOPins;

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(dev->RxFifoMem, LPC546XX_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(dev->TxFifoMem, LPC546XX_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	// CFifoInit returns NULL when the supplied memory is too small.
	if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
	{
		Lpc546xxFlexcommClock(dev->DevNo, false);
		EnableInterrupt(state);

		return false;
	}

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

	pDev->Rate = (int)Lpc546xxUartSetRate(&pDev->DevIntrf, (uint32_t)pCfg->Rate);

	uint32_t cfg = USART_CFG_DATALEN(pCfg->DataBits == 7 ? LPC546XX_USART_DATALEN_7 : LPC546XX_USART_DATALEN_8);

	if (pCfg->Parity == UART_PARITY_EVEN)
	{
		cfg |= USART_CFG_PARITYSEL(LPC546XX_USART_PARITY_EVEN);
	}
	else if (pCfg->Parity == UART_PARITY_ODD)
	{
		cfg |= USART_CFG_PARITYSEL(LPC546XX_USART_PARITY_ODD);
	}
	if (pCfg->StopBits == 2)
	{
		cfg |= USART_CFG_STOPLEN_MASK;
	}
	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		cfg |= USART_CFG_CTSEN_MASK;
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
	pDev->DevIntrf.IntPrio = pCfg->IntPrio;
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = NULL;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.GetHandle = Lpc546xxUartGetHandle;
	pDev->DevIntrf.Reset = Lpc546xxUartReset;
	pDev->DevIntrf.Disable = Lpc546xxUartDisable;
	pDev->DevIntrf.Enable = Lpc546xxUartEnable;
	pDev->DevIntrf.GetRate = Lpc546xxUartGetRate;
	pDev->DevIntrf.SetRate = Lpc546xxUartSetRate;
	pDev->DevIntrf.StartRx = Lpc546xxUartStartRx;
	pDev->DevIntrf.RxData = Lpc546xxUartRxData;
	pDev->DevIntrf.StopRx = Lpc546xxUartStopRx;
	pDev->DevIntrf.StartTx = Lpc546xxUartStartTx;
	pDev->DevIntrf.TxData = Lpc546xxUartTxData;
	pDev->DevIntrf.StopTx = Lpc546xxUartStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = Lpc546xxUartDisable;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	// Both FIFOs on and empty, level triggers for the interrupt mode
	reg->FIFOCFG = USART_FIFOCFG_ENABLETX_MASK | USART_FIFOCFG_ENABLERX_MASK |
				   USART_FIFOCFG_EMPTYTX_MASK | USART_FIFOCFG_EMPTYRX_MASK;
	reg->FIFOTRIG = USART_FIFOTRIG_TXLVLENA_MASK | USART_FIFOTRIG_TXLVL(LPC546XX_USART_TXLVL) |
					USART_FIFOTRIG_RXLVLENA_MASK | USART_FIFOTRIG_RXLVL(LPC546XX_USART_RXLVL);
	reg->FIFOINTENCLR = USART_FIFOINTENCLR_TXERR_MASK | USART_FIFOINTENCLR_RXERR_MASK |
						USART_FIFOINTENCLR_TXLVL_MASK | USART_FIFOINTENCLR_RXLVL_MASK;
	reg->INTENCLR = 0xFFFFFFFFU;

	if (pCfg->bIntMode)
	{
		// The TX level interrupt is enabled by Lpc546xxUartTxData when the TX
		// CFIFO has data.
		SharedIntrfSetIrqHandler(dev->DevNo, &pDev->DevIntrf, Lpc546xxUartIrqHandler);
		reg->FIFOINTENSET = USART_FIFOINTENSET_RXLVL_MASK | USART_FIFOINTENSET_RXERR_MASK |
							USART_FIFOINTENSET_TXERR_MASK;

		NVIC_ClearPendingIRQ(irq);
		NVIC_SetPriority(irq, pCfg->IntPrio);
		NVIC_EnableIRQ(irq);
	}
	else
	{
		SharedIntrfSetIrqHandler(dev->DevNo, NULL, NULL);
	}

	reg->CFG = cfg | USART_CFG_ENABLE_MASK;

	EnableInterrupt(state);

	return true;
}

// Line state is not controlled by software. With UART_FLWCTRL_HW the USART
// drives RTS and follows CTS.
void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
}
