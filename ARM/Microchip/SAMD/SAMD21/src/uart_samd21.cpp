/**-------------------------------------------------------------------------
@file	uart_samd21.cpp

@brief	SAM D21 SERCOM USART polling driver

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "sercom_clock_samd21.h"

#if !defined(__SAMD21J18A__)
#error Unsupported SAM D device
#endif

#define SAMD21_SERCOM_BASE(n)	(0x42000800UL + ((uint32_t)(n) * 0x400UL))
#define SAMD21_SERCOM_CTRLA		0x00UL
#define SAMD21_SERCOM_CTRLB		0x04UL
#define SAMD21_SERCOM_BAUD		0x0CUL
#define SAMD21_SERCOM_INTENCLR	0x14UL
#define SAMD21_SERCOM_INTENSET	0x16UL
#define SAMD21_SERCOM_INTFLAG	0x18UL
#define SAMD21_SERCOM_DATA		0x28UL
#define SAMD21_SERCOM_STATUS		0x1AUL
#define SAMD21_SERCOM_SYNCBUSY	0x1CUL

#define SAMD21_USART_ENABLE		(1UL << 1)
#define SAMD21_USART_MODE_INT	(1UL << 2)
#define SAMD21_USART_DORD		(1UL << 30)
#define SAMD21_USART_RXPO_1		(1UL << 20)
#define SAMD21_USART_TXPO_0		0UL
#define SAMD21_USART_RXEN		(1UL << 17)
#define SAMD21_USART_TXEN		(1UL << 16)
#define SAMD21_USART_DRE		(1U << 0)
#define SAMD21_USART_RXC		(1U << 2)
#define SAMD21_USART_TXC		(1U << 1)
#define SAMD21_USART_ERROR		(1U << 7)
#define SAMD21_USART_STATUS_PERR	(1U << 0)
#define SAMD21_USART_STATUS_FERR	(1U << 2)
#define SAMD21_USART_STATUS_BUFOVF	(1U << 3)
#define SAMD21_USART_WAIT		100000UL
#define SAMD21_UART_FIFO_SIZE	16
#define SAMD21_UART_FIFO_MEMSIZE CFIFO_MEMSIZE(SAMD21_UART_FIFO_SIZE)
#define SAMD21_SYNCBUSY_ENABLE	(1UL << 1)
#define SAMD21_SYNCBUSY_CTRLB	(1UL << 2)

typedef struct {
	UARTDev_t *pUartDev;
	uint32_t Base;
	uint8_t RxFifoMem[SAMD21_UART_FIFO_MEMSIZE];
	uint8_t TxFifoMem[SAMD21_UART_FIFO_MEMSIZE];
} Samd21UartDev_t;

static Samd21UartDev_t s_UartDev[6] = {
	{ NULL, SAMD21_SERCOM_BASE(0) },
	{ NULL, SAMD21_SERCOM_BASE(1) },
	{ NULL, SAMD21_SERCOM_BASE(2) },
	{ NULL, SAMD21_SERCOM_BASE(3) },
	{ NULL, SAMD21_SERCOM_BASE(4) },
	{ NULL, SAMD21_SERCOM_BASE(5) }
};

static volatile uint32_t *Samd21UartReg32(Samd21UartDev_t *pDev, uint32_t Offset)
{
	return (volatile uint32_t *)(uintptr_t)(pDev->Base + Offset);
}

static volatile uint16_t *Samd21UartReg16(Samd21UartDev_t *pDev, uint32_t Offset)
{
	return (volatile uint16_t *)(uintptr_t)(pDev->Base + Offset);
}

static volatile uint8_t *Samd21UartReg8(Samd21UartDev_t *pDev, uint32_t Offset)
{
	return (volatile uint8_t *)(uintptr_t)(pDev->Base + Offset);
}

static bool Samd21UartWait(Samd21UartDev_t *pDev, uint32_t Mask)
{
	uint32_t timeout;

	timeout = SAMD21_USART_WAIT;
	while ((*Samd21UartReg32(pDev, SAMD21_SERCOM_SYNCBUSY) & Mask) != 0UL)
	{
		if (--timeout == 0UL)
		{
			return false;
		}
	}

	return true;
}

static uint32_t Samd21UartGetRate(DevIntrf_t * const pIntrf)
{
	Samd21UartDev_t *pDev;

	pDev = (Samd21UartDev_t *)pIntrf->pDevData;
	return pDev->pUartDev->Rate;
}

static uint32_t Samd21UartSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	Samd21UartDev_t *pDev;
	uint32_t clock;
	uint32_t baud;
	uint32_t actual;
	uint32_t control;
	bool enabled;

	pDev = (Samd21UartDev_t *)pIntrf->pDevData;
	clock = SystemCoreClockGet();

	if (Rate == 0UL || clock == 0UL || Rate > clock / 16UL)
	{
		return 0UL;
	}

	baud = 65536UL - (uint32_t)(((uint64_t)65536UL * 16UL * Rate +
		clock / 2UL) / clock);
	if (baud > 65535UL)
	{
		return 0UL;
	}

	control = *Samd21UartReg32(pDev, SAMD21_SERCOM_CTRLA);
	enabled = (control & SAMD21_USART_ENABLE) != 0UL;
	if (enabled)
	{
		// Do not stop a transmission before the last stop bit.
		if ((*Samd21UartReg8(pDev, SAMD21_SERCOM_INTFLAG) &
			SAMD21_USART_TXC) == 0U)
		{
			return 0UL;
		}
		*Samd21UartReg32(pDev, SAMD21_SERCOM_CTRLA) =
			control & ~SAMD21_USART_ENABLE;
		if (!Samd21UartWait(pDev, SAMD21_SYNCBUSY_ENABLE))
		{
			return 0UL;
		}
	}

	*Samd21UartReg16(pDev, SAMD21_SERCOM_BAUD) = (uint16_t)baud;
	if (enabled)
	{
		*Samd21UartReg32(pDev, SAMD21_SERCOM_CTRLA) = control;
		if (!Samd21UartWait(pDev, SAMD21_SYNCBUSY_ENABLE))
		{
			return 0UL;
		}
	}
	actual = (uint32_t)(((uint64_t)(65536UL - baud) * clock + 524288UL) /
		1048576UL);
	pDev->pUartDev->Rate = (int)actual;

	return actual;
}

static bool Samd21UartStart(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf;
	(void)DevAddr;
	return true;
}

static void Samd21UartStop(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
}

static void Samd21UartIrq(Samd21UartDev_t *pDev)
{
	UARTDev_t *pUart;
	uint8_t flags;
	int data;
	uint8_t *p;

	pUart = pDev->pUartDev;
	if (pUart == NULL || !pUart->DevIntrf.bIntEn)
	{
		return;
	}

	flags = *Samd21UartReg8(pDev, SAMD21_SERCOM_INTFLAG);
	if ((flags & SAMD21_USART_RXC) != 0U)
	{
		data = (int)*Samd21UartReg16(pDev, SAMD21_SERCOM_DATA);
		if (!CFifoPutByte(pUart->hRxFifo, (uint8_t)data))
		{
			pUart->RxDropCnt++;
		}
		if (pUart->EvtCallback != NULL)
		{
			pUart->EvtCallback(pUart, UART_EVT_RXDATA, NULL,
				CFifoUsed(pUart->hRxFifo));
		}
	}

	if ((flags & SAMD21_USART_DRE) != 0U)
	{
		p = CFifoGet(pUart->hTxFifo);
		if (p != NULL)
		{
			*Samd21UartReg16(pDev, SAMD21_SERCOM_DATA) = *p;
		}
		else
		{
			*Samd21UartReg8(pDev, SAMD21_SERCOM_INTENCLR) =
				SAMD21_USART_DRE;
			pUart->bTxReady = true;
			if (pUart->EvtCallback != NULL)
			{
				pUart->EvtCallback(pUart, UART_EVT_TXREADY, NULL,
					CFifoAvail(pUart->hTxFifo));
			}
		}
	}
}

extern "C" void SERCOM0_Handler(void) { Samd21UartIrq(&s_UartDev[0]); }
extern "C" void SERCOM1_Handler(void) { Samd21UartIrq(&s_UartDev[1]); }
extern "C" void SERCOM2_Handler(void) { Samd21UartIrq(&s_UartDev[2]); }
extern "C" void SERCOM3_Handler(void) { Samd21UartIrq(&s_UartDev[3]); }
extern "C" void SERCOM4_Handler(void) { Samd21UartIrq(&s_UartDev[4]); }
extern "C" void SERCOM5_Handler(void) { Samd21UartIrq(&s_UartDev[5]); }

static int Samd21UartRx(DevIntrf_t * const pIntrf, uint8_t *pBuff, int Len)
{
	Samd21UartDev_t *pDev;
	int count;
	uint16_t status;

	pDev = (Samd21UartDev_t *)pIntrf->pDevData;
	count = 0;
	if (pIntrf->bIntEn)
	{
		while (count < Len)
		{
			int value = CFifoGetByte(pDev->pUartDev->hRxFifo);
			if (value < 0)
			{
				break;
			}
			pBuff[count++] = (uint8_t)value;
		}
		return count;
	}
	while (count < Len)
	{
		if ((*Samd21UartReg8(pDev, SAMD21_SERCOM_INTFLAG) &
			SAMD21_USART_RXC) == 0U)
		{
			break;
		}
		status = *Samd21UartReg16(pDev, SAMD21_SERCOM_STATUS);
		if ((status & SAMD21_USART_STATUS_BUFOVF) != 0U)
		{
			pDev->pUartDev->RxOvrErrCnt++;
		}
		if ((status & SAMD21_USART_STATUS_FERR) != 0U)
		{
			pDev->pUartDev->FramErrCnt++;
		}
		if ((status & SAMD21_USART_STATUS_PERR) != 0U)
		{
			pDev->pUartDev->ParErrCnt++;
		}
		if (status & (SAMD21_USART_STATUS_BUFOVF |
			SAMD21_USART_STATUS_FERR | SAMD21_USART_STATUS_PERR))
		{
			(void)*Samd21UartReg16(pDev, SAMD21_SERCOM_DATA);
			*Samd21UartReg16(pDev, SAMD21_SERCOM_STATUS) =
				status & (SAMD21_USART_STATUS_BUFOVF |
				SAMD21_USART_STATUS_FERR | SAMD21_USART_STATUS_PERR);
			*Samd21UartReg8(pDev, SAMD21_SERCOM_INTFLAG) =
				SAMD21_USART_ERROR;
			continue;
		}
		pBuff[count] = (uint8_t)*Samd21UartReg16(pDev, SAMD21_SERCOM_DATA);
		count++;
	}
	return count;
}

static int Samd21UartTx(DevIntrf_t * const pIntrf, const uint8_t *pData, int Len)
{
	Samd21UartDev_t *pDev;
	int count;
	uint32_t timeout;

	pDev = (Samd21UartDev_t *)pIntrf->pDevData;
	count = 0;
	if (pIntrf->bIntEn)
	{
		while (count < Len)
		{
			if (!CFifoPutByte(pDev->pUartDev->hTxFifo, pData[count]))
			{
				break;
			}
			count++;
		}
		if (count > 0)
		{
			*Samd21UartReg8(pDev, SAMD21_SERCOM_INTENSET) =
				SAMD21_USART_DRE;
		}
		return count;
	}

	while (count < Len)
	{
		timeout = SAMD21_USART_WAIT;
		while ((*Samd21UartReg8(pDev, SAMD21_SERCOM_INTFLAG) &
			SAMD21_USART_DRE) == 0U)
		{
			if (--timeout == 0UL)
			{
				return count;
			}
		}
		*Samd21UartReg16(pDev, SAMD21_SERCOM_DATA) = pData[count++];
	}
	return count;
}

static void Samd21UartDisable(DevIntrf_t * const pIntrf)
{
	Samd21UartDev_t *pDev;

	pDev = (Samd21UartDev_t *)pIntrf->pDevData;
	*Samd21UartReg32(pDev, SAMD21_SERCOM_CTRLA) &=
		~SAMD21_USART_ENABLE;
	(void)Samd21UartWait(pDev, SAMD21_SYNCBUSY_ENABLE);
}

static void Samd21UartEnable(DevIntrf_t * const pIntrf)
{
	Samd21UartDev_t *pDev;

	pDev = (Samd21UartDev_t *)pIntrf->pDevData;
	*Samd21UartReg32(pDev, SAMD21_SERCOM_CTRLA) |=
		SAMD21_USART_ENABLE;
	(void)Samd21UartWait(pDev, SAMD21_SYNCBUSY_ENABLE);
}

static void Samd21UartReset(DevIntrf_t * const pIntrf)
{
	Samd21UartDev_t *pDev;

	pDev = (Samd21UartDev_t *)pIntrf->pDevData;
	*Samd21UartReg32(pDev, SAMD21_SERCOM_CTRLA) &=
		~SAMD21_USART_ENABLE;
	if (!Samd21UartWait(pDev, SAMD21_SYNCBUSY_ENABLE))
	{
		return;
	}

	*Samd21UartReg32(pDev, SAMD21_SERCOM_CTRLA) |=
		SAMD21_USART_ENABLE;
	(void)Samd21UartWait(pDev, SAMD21_SYNCBUSY_ENABLE);
}

static void *Samd21UartGetHandle(DevIntrf_t * const pIntrf)
{
	return pIntrf->pDevData;
}

UARTDev_t const *UARTGetInstance(int DevNo)
{
	if (DevNo < 0 || DevNo >= 6)
	{
		return NULL;
	}
	return s_UartDev[DevNo].pUartDev;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
	pDev->LineState = LineState;
}

bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	Samd21UartDev_t *pHw;
	DevIntrf_t *pIntrf;
	uint32_t baud;

	if (pDev == NULL || pCfg == NULL || pCfg->DevNo < 0 ||
		pCfg->DevNo >= 6 || pCfg->Rate <= 0 ||
		pCfg->DataBits != 8 || pCfg->StopBits != 1 ||
		pCfg->Parity != UART_PARITY_NONE ||
		pCfg->FlowControl != UART_FLWCTRL_NONE ||
		pCfg->bDMAMode ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins < 2)
	{
		return false;
	}

	pHw = &s_UartDev[pCfg->DevNo];
	if (pHw->pUartDev != NULL && pHw->pUartDev != pDev)
	{
		return false;
	}

	if (!Samd21SercomClockEnable(pCfg->DevNo))
	{
		return false;
	}

	// This initial mapping requires RX on PAD1 and TX on PAD0.
	IOPinCfg((const IOPinCfg_t *)pCfg->pIOPinMap, 2);

	*Samd21UartReg32(pHw, SAMD21_SERCOM_CTRLA) = 0UL;
	if (!Samd21UartWait(pHw, SAMD21_SYNCBUSY_ENABLE))
	{
		return false;
	}
	*Samd21UartReg32(pHw, SAMD21_SERCOM_CTRLA) =
		SAMD21_USART_DORD | SAMD21_USART_MODE_INT |
		SAMD21_USART_RXPO_1 | SAMD21_USART_TXPO_0;
	*Samd21UartReg32(pHw, SAMD21_SERCOM_CTRLB) =
		SAMD21_USART_TXEN | SAMD21_USART_RXEN;
	if (!Samd21UartWait(pHw, SAMD21_SYNCBUSY_CTRLB))
	{
		return false;
	}

	pIntrf = &pDev->DevIntrf;
	pIntrf->pDevData = pHw;
	pHw->pUartDev = pDev;
	baud = Samd21UartSetRate(pIntrf, (uint32_t)pCfg->Rate);
	if (baud == 0UL)
	{
		pHw->pUartDev = NULL;
		return false;
	}

	pDev->Rate = (int)baud;
	pDev->DataBits = 8;
	pDev->StopBits = 1;
	pDev->Parity = UART_PARITY_NONE;
	pDev->FlowControl = UART_FLWCTRL_NONE;
	pDev->EvtCallback = pCfg->EvtCallback;
	if (pCfg->bIntMode)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem != NULL ?
			pCfg->pRxMem : pHw->RxFifoMem,
			pCfg->pRxMem != NULL ? pCfg->RxMemSize : SAMD21_UART_FIFO_MEMSIZE,
			1, pCfg->bFifoBlocking);
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem != NULL ?
			pCfg->pTxMem : pHw->TxFifoMem,
			pCfg->pTxMem != NULL ? pCfg->TxMemSize : SAMD21_UART_FIFO_MEMSIZE,
			1, true);
		if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
		{
			pHw->pUartDev = NULL;
			return false;
		}
	}
	pIntrf->Type = DEVINTRF_TYPE_UART;
	pIntrf->GetRate = Samd21UartGetRate;
	pIntrf->SetRate = Samd21UartSetRate;
	pIntrf->StartRx = Samd21UartStart;
	pIntrf->RxData = Samd21UartRx;
	pIntrf->StopRx = Samd21UartStop;
	pIntrf->StartTx = Samd21UartStart;
	pIntrf->TxData = Samd21UartTx;
	pIntrf->StopTx = Samd21UartStop;
	pIntrf->TxSrData = Samd21UartTx;
	pIntrf->Disable = Samd21UartDisable;
	pIntrf->Enable = Samd21UartEnable;
	pIntrf->PowerOff = Samd21UartDisable;
	pIntrf->Reset = Samd21UartReset;
	pIntrf->GetHandle = Samd21UartGetHandle;
	pIntrf->MaxRetry = UART_RETRY_MAX;
	pIntrf->bIntEn = pCfg->bIntMode;
	pIntrf->bDma = false;
	atomic_flag_clear(&pIntrf->bBusy);
	atomic_store(&pIntrf->EnCnt, 1);
	Samd21UartEnable(pIntrf);
	if ((*Samd21UartReg32(pHw, SAMD21_SERCOM_SYNCBUSY) &
		SAMD21_SYNCBUSY_ENABLE) != 0UL ||
		(*Samd21UartReg32(pHw, SAMD21_SERCOM_CTRLA) &
		SAMD21_USART_ENABLE) == 0UL)
	{
		pHw->pUartDev = NULL;
		return false;
	}

	if (pCfg->bIntMode)
	{
		*Samd21UartReg8(pHw, SAMD21_SERCOM_INTENSET) =
			SAMD21_USART_RXC;
		NVIC_ClearPendingIRQ((IRQn_Type)(9 + pCfg->DevNo));
		NVIC_SetPriority((IRQn_Type)(9 + pCfg->DevNo), pCfg->IntPrio);
		NVIC_EnableIRQ((IRQn_Type)(9 + pCfg->DevNo));
	}

	return true;
}
