/**-------------------------------------------------------------------------
@file	uart_stm32.cpp

@brief	Shared STM32 UART transport and family-specific peripheral setup.

		The FIFO, polling, interrupt, and public UART interface are shared.
		MCU-family differences are limited to clock, reset, baud-rate, frame,
		and IRQ selection helpers.

@author	Hoang Nguyen Hoan
@date	October 10, 2026

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
#include <stdbool.h>
#include <string.h>

#include "stm32.h"

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"


#if defined(IOSONATA_STM32_F0)

#define STM32_UART_BUFF_SIZE			16

#elif defined(IOSONATA_STM32_L4)

#define STM32_UART_BUFF_SIZE			32

#define RCC_CCIPR_USARTSEL_PCLK			0U
#define RCC_CCIPR_USARTSEL_SYSCLK		1U
#define RCC_CCIPR_USARTSEL_HSI16		2U
#define RCC_CCIPR_USARTSEL_LSE			3U
#define RCC_CCIPR_USARTSEL(n, clk)		((uint32_t)(clk) << (((n) - 1U) << 1))

#else
#error "uart_stm32.cpp: STM32 UART family adapter not implemented"
#endif

#define STM32_UART_CFIFO_SIZE			CFIFO_MEMSIZE(STM32_UART_BUFF_SIZE)

#if defined(USART_ISR_RXNE_RXFNE)
#define STM32_UART_ISR_RXREADY			USART_ISR_RXNE_RXFNE
#else
#define STM32_UART_ISR_RXREADY			USART_ISR_RXNE
#endif

#if defined(USART_ISR_TXE_TXFNF)
#define STM32_UART_ISR_TXREADY			USART_ISR_TXE_TXFNF
#else
#define STM32_UART_ISR_TXREADY			USART_ISR_TXE
#endif

#if defined(USART_CR1_RXNEIE_RXFNEIE)
#define STM32_UART_CR1_RXIE				USART_CR1_RXNEIE_RXFNEIE
#else
#define STM32_UART_CR1_RXIE				USART_CR1_RXNEIE
#endif

#if defined(USART_CR1_TXEIE_TXFNFIE)
#define STM32_UART_CR1_TXIE				USART_CR1_TXEIE_TXFNFIE
#else
#define STM32_UART_CR1_TXIE				USART_CR1_TXEIE
#endif


typedef struct {
	int DevNo;
	USART_TypeDef *pReg;
	UARTDev_t *pUartDev;
	const IOPinCfg_t *pIOPinMap;
	int NbPins;
	uint32_t RxDropCnt;
	uint32_t RxTimeoutCnt;
	uint32_t ErrCnt;
	uint32_t ErrFlag;
	uint8_t RxFifoMem[STM32_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[STM32_UART_CFIFO_SIZE];
} Stm32UartDev_t;


#if defined(IOSONATA_STM32_F0)

static Stm32UartDev_t s_UartDev[] = {
	{ 0, USART1 },
#if defined(USART2)
	{ 1, USART2 },
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
	{ 2, USART3 },
	{ 3, USART4 },
#endif
#if defined(STM32F030xC)
	{ 4, USART5 },
	{ 5, USART6 },
#endif
};

#elif defined(IOSONATA_STM32_L4)

static Stm32UartDev_t s_UartDev[] = {
	{ 0, LPUART1 },
	{ 1, USART1 },
	{ 2, USART2 },
	{ 3, USART3 },
	{ 4, UART4 },
	{ 5, UART5 },
};

#endif

static const int s_NbUartDev = sizeof(s_UartDev) / sizeof(s_UartDev[0]);


static int Stm32UartRxFifoRead(UARTDev_t *pDev, uint8_t *pBuff, int Bufflen);
static void Stm32UartClearErrors(Stm32UartDev_t *pDev, uint32_t Flags);
static void Stm32UartIRQ(Stm32UartDev_t *pDev);

static uint32_t Stm32UartGetRate(DevIntrf_t * const pDev);
static uint32_t Stm32UartSetRate(DevIntrf_t * const pDev, uint32_t Rate);
static bool Stm32UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Stm32UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen);
static void Stm32UartStopRx(DevIntrf_t * const pDev);
static bool Stm32UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Stm32UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen);
static void Stm32UartStopTx(DevIntrf_t * const pDev);
static void Stm32UartDisable(DevIntrf_t * const pDev);
static void Stm32UartEnable(DevIntrf_t * const pDev);
static void Stm32UartPowerOff(DevIntrf_t * const pDev);
static void Stm32UartReset(DevIntrf_t * const pDev);
static void *Stm32UartGetHandle(DevIntrf_t * const pDev);

static bool Stm32UartHwEnableClock(Stm32UartDev_t *pDev);
static void Stm32UartHwDisableClock(Stm32UartDev_t *pDev);
static void Stm32UartHwReset(Stm32UartDev_t *pDev);
static int Stm32UartHwIRQ(const Stm32UartDev_t *pDev);
static bool Stm32UartHwEnableIRQ(const Stm32UartDev_t *pDev, int Priority);
static uint32_t Stm32UartHwSetRate(Stm32UartDev_t *pDev, uint32_t Rate);
static bool Stm32UartHwConfigureFrame(Stm32UartDev_t *pDev, const UARTCfg_t *pCfg);


static int Stm32UartRxFifoRead(UARTDev_t *pDev, uint8_t *pBuff, int Bufflen)
{
	int cnt = 0;

	while (Bufflen > 0)
	{
		int len = Bufflen;
		uint8_t *p = CFifoGetMultiple(pDev->hRxFifo, &len);

		if (p == NULL)
		{
			break;
		}

		memcpy(pBuff, p, len);
		pBuff += len;
		Bufflen -= len;
		cnt += len;
	}

	pDev->bRxReady = CFifoUsed(pDev->hRxFifo) != 0;

	return cnt;
}

static void Stm32UartClearErrors(Stm32UartDev_t *pDev, uint32_t Flags)
{
	uint32_t clear = 0U;

	if (Flags & USART_ISR_PE)
	{
		clear |= USART_ICR_PECF;
	}
	if (Flags & USART_ISR_FE)
	{
		clear |= USART_ICR_FECF;
	}
	if (Flags & USART_ISR_ORE)
	{
		clear |= USART_ICR_ORECF;
	}
	if (Flags & USART_ISR_NE)
	{
		clear |= USART_ICR_NCF;
	}

	if (clear != 0U)
	{
		pDev->pReg->ICR = clear;
	}
}

static void Stm32UartIRQ(Stm32UartDev_t *pDev)
{
	UARTDev_t *dev = pDev->pUartDev;

	if (dev == NULL || !dev->DevIntrf.bIntEn)
	{
		return;
	}

	uint32_t iflag = pDev->pReg->ISR;
	uint32_t cr1 = pDev->pReg->CR1;
	uint32_t err = iflag & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);

	if (err != 0U)
	{
		pDev->ErrCnt++;
		pDev->ErrFlag = err;
		Stm32UartClearErrors(pDev, err);

		if (iflag & STM32_UART_ISR_RXREADY)
		{
			(void)pDev->pReg->RDR;
		}

		iflag &= ~STM32_UART_ISR_RXREADY;
	}

	if ((iflag & STM32_UART_ISR_RXREADY) && (cr1 & STM32_UART_CR1_RXIE))
	{
		uint8_t data = (uint8_t)pDev->pReg->RDR;

		if (dev->DataBits == 7)
		{
			data &= 0x7FU;
		}

		uint8_t *p = CFifoPut(dev->hRxFifo);

		if (p != NULL)
		{
			*p = data;
			dev->bRxReady = true;
		}
		else
		{
			pDev->RxDropCnt++;
			dev->RxDropCnt++;
		}

		if (dev->EvtCallback != NULL)
		{
			dev->EvtCallback(dev, UART_EVT_RXDATA, NULL, CFifoUsed(dev->hRxFifo));
		}
	}

	if ((iflag & STM32_UART_ISR_TXREADY) && (cr1 & STM32_UART_CR1_TXIE))
	{
		uint8_t *p = CFifoGet(dev->hTxFifo);

		if (p != NULL)
		{
			dev->bTxReady = false;
			pDev->pReg->TDR = *p;
		}
		else
		{
			pDev->pReg->CR1 &= ~STM32_UART_CR1_TXIE;
			dev->bTxReady = true;

			if (dev->EvtCallback != NULL)
			{
				dev->EvtCallback(dev, UART_EVT_TXREADY, NULL, 0);
			}
		}
	}

#if defined(USART_ISR_RTOF) && defined(USART_CR1_RTOIE) && defined(USART_ICR_RTOCF)
	if ((iflag & USART_ISR_RTOF) && (cr1 & USART_CR1_RTOIE))
	{
		pDev->pReg->ICR = USART_ICR_RTOCF;
		pDev->RxTimeoutCnt++;

		if (dev->EvtCallback != NULL)
		{
			dev->EvtCallback(dev, UART_EVT_RXTIMEOUT, NULL, CFifoUsed(dev->hRxFifo));
		}
	}
#endif
}

static uint32_t Stm32UartGetRate(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	return dev != NULL && dev->pUartDev != NULL ? dev->pUartDev->Rate : 0U;
}

static uint32_t Stm32UartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	if (dev == NULL || dev->pUartDev == NULL)
	{
		return 0U;
	}

	return Stm32UartHwSetRate(dev, Rate);
}

static bool Stm32UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	(void)DevAddr;

	return true;
}

static int Stm32UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	if (dev == NULL || pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

	if (!pDev->bIntEn)
	{
		int cnt = 0;

		while (cnt < Bufflen)
		{
			uint32_t flags = dev->pReg->ISR;
			uint32_t err = flags & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);

			if (err != 0U)
			{
				dev->ErrCnt++;
				dev->ErrFlag = err;
				Stm32UartClearErrors(dev, err);

				if (flags & STM32_UART_ISR_RXREADY)
				{
					(void)dev->pReg->RDR;
				}
				break;
			}

			if ((flags & STM32_UART_ISR_RXREADY) == 0U)
			{
				break;
			}

			uint8_t data = (uint8_t)dev->pReg->RDR;

			if (dev->pUartDev->DataBits == 7)
			{
				data &= 0x7FU;
			}

			pBuff[cnt++] = data;
		}

		return cnt;
	}

	uint32_t state = DisableInterrupt();
	int cnt = Stm32UartRxFifoRead(dev->pUartDev, pBuff, Bufflen);
	EnableInterrupt(state);

	return cnt;
}

static void Stm32UartStopRx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static bool Stm32UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	(void)DevAddr;

	return true;
}

static int Stm32UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	if (dev == NULL || pData == NULL || Datalen <= 0 ||
		(dev->pReg->CR1 & USART_CR1_UE) == 0U)
	{
		return 0;
	}

	int cnt = 0;

	if (!pDev->bIntEn)
	{
		while (cnt < Datalen && (dev->pReg->ISR & STM32_UART_ISR_TXREADY))
		{
			dev->pReg->TDR = pData[cnt++];

			// TDR is a device write. Complete it before a fast Release build
			// can re-enter UARTTx and sample TXE/TXFNF again.
			__DSB();
		}

		return cnt;
	}

	int rtry = pDev->MaxRetry;

	while (Datalen > 0 && rtry-- > 0)
	{
		uint32_t state = DisableInterrupt();

		if ((dev->pReg->CR1 & USART_CR1_UE) == 0U)
		{
			EnableInterrupt(state);
			break;
		}

		while (Datalen > 0)
		{
			int len = Datalen;
			uint8_t *p = CFifoPutMultiple(dev->pUartDev->hTxFifo, &len);

			if (p == NULL)
			{
				break;
			}

			memcpy(p, pData, len);
			Datalen -= len;
			pData += len;
			cnt += len;
		}

		if (dev->pUartDev->bTxReady &&
			(dev->pReg->ISR & STM32_UART_ISR_TXREADY))
		{
			uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);

			if (p != NULL)
			{
				dev->pUartDev->bTxReady = false;
				dev->pReg->TDR = *p;
				dev->pReg->CR1 |= STM32_UART_CR1_TXIE;
			}
		}

		if (CFifoUsed(dev->pUartDev->hTxFifo) > 0)
		{
			dev->pUartDev->bTxReady = false;
			dev->pReg->CR1 |= STM32_UART_CR1_TXIE;
		}

		EnableInterrupt(state);
	}

	return cnt;
}

static void Stm32UartStopTx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static void Stm32UartDisable(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	if (dev == NULL)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	dev->pReg->CR1 &= ~(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
#if defined(USART_CR2_RTOEN)
	dev->pReg->CR2 &= ~USART_CR2_RTOEN;
#endif

	EnableInterrupt(state);
}

static void Stm32UartEnable(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	if (dev == NULL || dev->pUartDev == NULL)
	{
		return;
	}

	if (!Stm32UartHwEnableClock(dev))
	{
		return;
	}

	if (dev->pIOPinMap != NULL && dev->NbPins > 0)
	{
		IOPinCfg(dev->pIOPinMap, dev->NbPins);
	}

	dev->ErrCnt = 0;
	dev->ErrFlag = 0;
	dev->RxTimeoutCnt = 0;
	dev->RxDropCnt = 0;

	CFifoFlush(dev->pUartDev->hTxFifo);
	dev->pUartDev->bTxReady = true;
	atomic_flag_clear(&pDev->bBusy);

	dev->pReg->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
}

static void Stm32UartPowerOff(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	if (dev == NULL)
	{
		return;
	}

	Stm32UartDisable(pDev);

	if (dev->pIOPinMap != NULL && dev->NbPins > 0)
	{
		IOPinDis(dev->pIOPinMap, dev->NbPins);
	}

	Stm32UartHwDisableClock(dev);
}

static void Stm32UartReset(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	if (dev == NULL)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	Stm32UartHwReset(dev);
	EnableInterrupt(state);
}

static void *Stm32UartGetHandle(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	return dev != NULL ? dev->pUartDev : NULL;
}


#if defined(IOSONATA_STM32_F0)

static bool Stm32UartHwEnableClock(Stm32UartDev_t *pDev)
{
	switch (pDev->DevNo)
	{
		case 0:
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			(void)RCC->APB2ENR;
#ifdef RCC_CFGR3_USART1SW_Msk
			RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;
#endif
			break;

#if defined(USART2)
		case 1:
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
			(void)RCC->APB1ENR;
			break;
#endif

#if defined(STM32F070xB) || defined(STM32F030xC)
		case 2:
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
			(void)RCC->APB1ENR;
			break;

		case 3:
			RCC->APB1ENR |= RCC_APB1ENR_USART4EN;
			(void)RCC->APB1ENR;
			break;
#endif

#if defined(STM32F030xC)
		case 4:
			RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
			(void)RCC->APB1ENR;
			break;

		case 5:
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
			(void)RCC->APB2ENR;
			break;
#endif

		default:
			return false;
	}

	return true;
}

static void Stm32UartHwDisableClock(Stm32UartDev_t *pDev)
{
	switch (pDev->DevNo)
	{
		case 0:
			RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
			break;

#if defined(USART2)
		case 1:
			RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
			break;
#endif

#if defined(STM32F070xB) || defined(STM32F030xC)
		case 2:
			RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
			break;

		case 3:
			RCC->APB1ENR &= ~RCC_APB1ENR_USART4EN;
			break;
#endif

#if defined(STM32F030xC)
		case 4:
			RCC->APB1ENR &= ~RCC_APB1ENR_USART5EN;
			break;

		case 5:
			RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
			break;
#endif

		default:
			break;
	}
}

static void Stm32UartHwReset(Stm32UartDev_t *pDev)
{
	switch (pDev->DevNo)
	{
		case 0:
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
			(void)RCC->APB2RSTR;
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
			(void)RCC->APB2RSTR;
			break;

#if defined(USART2)
		case 1:
			RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
			(void)RCC->APB1RSTR;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
			(void)RCC->APB1RSTR;
			break;
#endif

#if defined(STM32F070xB) || defined(STM32F030xC)
		case 2:
			RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
			(void)RCC->APB1RSTR;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
			(void)RCC->APB1RSTR;
			break;

		case 3:
			RCC->APB1RSTR |= RCC_APB1RSTR_USART4RST;
			(void)RCC->APB1RSTR;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART4RST;
			(void)RCC->APB1RSTR;
			break;
#endif

#if defined(STM32F030xC)
		case 4:
			RCC->APB1RSTR |= RCC_APB1RSTR_USART5RST;
			(void)RCC->APB1RSTR;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART5RST;
			(void)RCC->APB1RSTR;
			break;

		case 5:
			RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST;
			(void)RCC->APB2RSTR;
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
			(void)RCC->APB2RSTR;
			break;
#endif

		default:
			break;
	}
}

static int Stm32UartHwIRQ(const Stm32UartDev_t *pDev)
{
	switch (pDev->DevNo)
	{
		case 0:
			return USART1_IRQn;

#if defined(USART2)
		case 1:
			return USART2_IRQn;
#endif

#if defined(STM32F070xB) || defined(STM32F030xC)
		default:
#if defined(STM32F030xC)
			return USART3_6_IRQn;
#else
			return USART3_4_IRQn;
#endif
#else
		default:
			break;
#endif
	}

	return -1;
}

static uint32_t Stm32UartHwSetRate(Stm32UartDev_t *pDev, uint32_t Rate)
{
	uint32_t fclkfreq = SystemPeriphClockGet(0);

	if (Rate == 0U || fclkfreq == 0U || Rate > fclkfreq / 16U)
	{
		return 0U;
	}

	uint32_t div = (fclkfreq + (Rate >> 1)) / Rate;

	if (div < 16U || div > 0xFFFFU)
	{
		return 0U;
	}

	uint32_t cr1 = pDev->pReg->CR1;
	uint32_t newcr1 = cr1 & ~(USART_CR1_UE | USART_CR1_OVER8);

	pDev->pReg->CR1 = newcr1;
	pDev->pReg->BRR = div;
	pDev->pReg->CR1 = newcr1 | (cr1 & USART_CR1_UE);
	pDev->pUartDev->Rate = fclkfreq / div;

	return pDev->pUartDev->Rate;
}

static bool Stm32UartHwConfigureFrame(Stm32UartDev_t *pDev, const UARTCfg_t *pCfg)
{
	int wordbits = pCfg->DataBits + (pCfg->Parity != UART_PARITY_NONE ? 1 : 0);
	uint32_t cr1 = pDev->pReg->CR1;

	cr1 &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);

#ifdef USART_CR1_M1
	cr1 &= ~USART_CR1_M1;

	if (wordbits == 7)
	{
		cr1 |= USART_CR1_M1;
	}
#else
	if (wordbits == 7)
	{
		return false;
	}
#endif

	if (pCfg->Parity != UART_PARITY_NONE)
	{
		cr1 |= USART_CR1_PCE;
	}
	if (pCfg->Parity == UART_PARITY_ODD)
	{
		cr1 |= USART_CR1_PS;
	}
	if (wordbits == 9)
	{
		cr1 |= USART_CR1_M;
	}

	pDev->pReg->CR1 = cr1;

	return true;
}

#elif defined(IOSONATA_STM32_L4)

static bool Stm32UartHwEnableClock(Stm32UartDev_t *pDev)
{
	uint32_t ccipr = RCC->CCIPR;

	switch (pDev->DevNo)
	{
		case 0:
			if (GetLowFreqOscType() == OSC_TYPE_XTAL)
			{
				ccipr = (ccipr & ~RCC_CCIPR_LPUART1SEL_Msk) |
						RCC_CCIPR_USARTSEL(6U, RCC_CCIPR_USARTSEL_LSE);
			}
			else
			{
				ccipr = (ccipr & ~RCC_CCIPR_LPUART1SEL_Msk) |
						RCC_CCIPR_USARTSEL(6U, RCC_CCIPR_USARTSEL_HSI16);
			}
			RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
			(void)RCC->APB1ENR2;
			break;

		case 1:
			ccipr = (ccipr & ~RCC_CCIPR_USART1SEL_Msk) |
					RCC_CCIPR_USARTSEL(1U, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			(void)RCC->APB2ENR;
			break;

		case 2:
			ccipr = (ccipr & ~RCC_CCIPR_USART2SEL_Msk) |
					RCC_CCIPR_USARTSEL(2U, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
			(void)RCC->APB1ENR1;
			break;

		case 3:
			ccipr = (ccipr & ~RCC_CCIPR_USART3SEL_Msk) |
					RCC_CCIPR_USARTSEL(3U, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
			(void)RCC->APB1ENR1;
			break;

		case 4:
			ccipr = (ccipr & ~RCC_CCIPR_UART4SEL_Msk) |
					RCC_CCIPR_USARTSEL(4U, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
			(void)RCC->APB1ENR1;
			break;

		case 5:
			ccipr = (ccipr & ~RCC_CCIPR_UART5SEL_Msk) |
					RCC_CCIPR_USARTSEL(5U, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
			(void)RCC->APB1ENR1;
			break;

		default:
			return false;
	}

	RCC->CCIPR = ccipr;
	(void)RCC->CCIPR;

	return true;
}

static void Stm32UartHwDisableClock(Stm32UartDev_t *pDev)
{
	switch (pDev->DevNo)
	{
		case 0:
			RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN;
			break;

		case 1:
			RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
			break;

		case 2:
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;
			break;

		case 3:
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART3EN;
			break;

		case 4:
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART4EN;
			break;

		case 5:
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART5EN;
			break;

		default:
			break;
	}
}

static void Stm32UartHwReset(Stm32UartDev_t *pDev)
{
	switch (pDev->DevNo)
	{
		case 0:
			RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST;
			(void)RCC->APB1RSTR2;
			RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_LPUART1RST;
			(void)RCC->APB1RSTR2;
			break;

		case 1:
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
			(void)RCC->APB2RSTR;
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
			(void)RCC->APB2RSTR;
			break;

		case 2:
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST;
			(void)RCC->APB1RSTR1;
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART2RST;
			(void)RCC->APB1RSTR1;
			break;

		case 3:
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART3RST;
			(void)RCC->APB1RSTR1;
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART3RST;
			(void)RCC->APB1RSTR1;
			break;

		case 4:
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_UART4RST;
			(void)RCC->APB1RSTR1;
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_UART4RST;
			(void)RCC->APB1RSTR1;
			break;

		case 5:
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_UART5RST;
			(void)RCC->APB1RSTR1;
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_UART5RST;
			(void)RCC->APB1RSTR1;
			break;

		default:
			break;
	}
}

static int Stm32UartHwIRQ(const Stm32UartDev_t *pDev)
{
	switch (pDev->DevNo)
	{
		case 0: return LPUART1_IRQn;
		case 1: return USART1_IRQn;
		case 2: return USART2_IRQn;
		case 3: return USART3_IRQn;
		case 4: return UART4_IRQn;
		case 5: return UART5_IRQn;
		default: break;
	}

	return -1;
}

static uint32_t Stm32UartHwSetRate(Stm32UartDev_t *pDev, uint32_t Rate)
{
	if (Rate == 0U)
	{
		return 0U;
	}

	uint32_t cr1 = pDev->pReg->CR1;
	uint32_t newcr1 = cr1 & ~USART_CR1_UE;
	uint32_t div = 0U;

	pDev->pReg->CR1 = newcr1;

	if (pDev->DevNo == 0)
	{
		uint64_t fclk256;
		uint32_t ccipr = RCC->CCIPR;

		if (GetLowFreqOscType() == OSC_TYPE_XTAL && Rate < 10922U)
		{
			fclk256 = (uint64_t)32768U << 8;
			ccipr = (ccipr & ~RCC_CCIPR_LPUART1SEL_Msk) |
					RCC_CCIPR_USARTSEL(6U, RCC_CCIPR_USARTSEL_LSE);
		}
		else
		{
			fclk256 = (uint64_t)16000000U << 8;
			ccipr = (ccipr & ~RCC_CCIPR_LPUART1SEL_Msk) |
					RCC_CCIPR_USARTSEL(6U, RCC_CCIPR_USARTSEL_HSI16);
		}

		div = (uint32_t)((fclk256 + (Rate >> 1)) / Rate);

		if (div < 0x300U || div > 0xFFFFFU)
		{
			pDev->pReg->CR1 = cr1;
			return 0U;
		}

		RCC->CCIPR = ccipr;
		pDev->pReg->BRR = div;
		pDev->pUartDev->Rate = (uint32_t)(fclk256 / div);
	}
	else
	{
		uint32_t fclk = SystemCoreClock;

		if (fclk == 0U)
		{
			pDev->pReg->CR1 = cr1;
			return 0U;
		}

		uint32_t div16 = (fclk + (Rate >> 1)) / Rate;
		uint32_t actual16 = div16 != 0U ? fclk / div16 : 0U;
		uint32_t err16 = actual16 > Rate ? actual16 - Rate : Rate - actual16;

		uint32_t fclk2 = fclk << 1;
		uint32_t div8 = (fclk2 + (Rate >> 1)) / Rate;
		uint32_t actual8 = div8 != 0U ? fclk2 / div8 : 0U;
		uint32_t err8 = actual8 > Rate ? actual8 - Rate : Rate - actual8;

		if (div8 >= 16U && div8 <= 0xFFFFU && err8 < err16)
		{
			newcr1 |= USART_CR1_OVER8;
			div = ((div8 & 0xFU) >> 1) | (div8 & 0xFFF0U);
			pDev->pUartDev->Rate = actual8;
		}
		else
		{
			if (div16 < 16U || div16 > 0xFFFFU)
			{
				pDev->pReg->CR1 = cr1;
				return 0U;
			}

			newcr1 &= ~USART_CR1_OVER8;
			div = div16;
			pDev->pUartDev->Rate = actual16;
		}

		pDev->pReg->BRR = div;
	}

	pDev->pReg->CR1 = newcr1 | (cr1 & USART_CR1_UE);

	return pDev->pUartDev->Rate;
}

static bool Stm32UartHwConfigureFrame(Stm32UartDev_t *pDev, const UARTCfg_t *pCfg)
{
	int wordbits = pCfg->DataBits + (pCfg->Parity != UART_PARITY_NONE ? 1 : 0);

	if (pDev->DevNo == 0 && wordbits == 7)
	{
		return false;
	}

	uint32_t cr1 = pDev->pReg->CR1;

	cr1 &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M0 | USART_CR1_M1);

	if (pCfg->Parity != UART_PARITY_NONE)
	{
		cr1 |= USART_CR1_PCE;
	}
	if (pCfg->Parity == UART_PARITY_ODD)
	{
		cr1 |= USART_CR1_PS;
	}
	if (wordbits == 9)
	{
		cr1 |= USART_CR1_M0;
	}
	else if (wordbits == 7)
	{
		cr1 |= USART_CR1_M1;
	}

	pDev->pReg->CR1 = cr1;

	return true;
}

#endif


static bool Stm32UartHwEnableIRQ(const Stm32UartDev_t *pDev, int Priority)
{
	int irqno = Stm32UartHwIRQ(pDev);

	if (irqno < 0)
	{
		return false;
	}

	IRQn_Type irq = (IRQn_Type)irqno;

	NVIC_ClearPendingIRQ(irq);
	NVIC_SetPriority(irq, Priority);
	NVIC_EnableIRQ(irq);

	return true;
}


UARTDev_t const *UARTGetInstance(int DevNo)
{
	return DevNo >= 0 && DevNo < s_NbUartDev ? s_UartDev[DevNo].pUartDev : NULL;
}

bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins <= 0 ||
		pCfg->DevNo < 0 || pCfg->DevNo >= s_NbUartDev)
	{
		return false;
	}

	// Hardware DMA is intentionally not exposed until a family implementation
	// is validated. Never accept bDMAMode and silently fall back to IRQ mode.
	if (pCfg->bDMAMode ||
		pCfg->bIrDAMode ||
		pCfg->Mode != UART_MODE_UART ||
		(pCfg->FlowControl != UART_FLWCTRL_NONE && pCfg->FlowControl != UART_FLWCTRL_HW) ||
		(pCfg->Parity != UART_PARITY_NONE && pCfg->Parity != UART_PARITY_EVEN &&
		 pCfg->Parity != UART_PARITY_ODD) ||
		(pCfg->DataBits != 7 && pCfg->DataBits != 8) ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) ||
		pCfg->Rate <= 0)
	{
		return false;
	}

	Stm32UartDev_t *dev = &s_UartDev[pCfg->DevNo];
	USART_TypeDef *reg = dev->pReg;

	dev->pUartDev = pDev;
	dev->pIOPinMap = (const IOPinCfg_t *)pCfg->pIOPinMap;
	dev->NbPins = pCfg->NbIOPins;

	pDev->DevIntrf.pDevData = dev;

	if (!Stm32UartHwEnableClock(dev))
	{
		return false;
	}

	Stm32UartHwReset(dev);
	reg->CR1 &= ~USART_CR1_UE;

	pDev->hRxFifo = pCfg->pRxMem != NULL && pCfg->RxMemSize > 0 ?
			CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking) :
			CFifoInit(dev->RxFifoMem, STM32_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);

	pDev->hTxFifo = pCfg->pTxMem != NULL && pCfg->TxMemSize > 0 ?
			CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking) :
			CFifoInit(dev->TxFifoMem, STM32_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);

	if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
	{
		Stm32UartHwDisableClock(dev);
		return false;
	}

	IOPinCfg(dev->pIOPinMap, dev->NbPins);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = NULL;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.GetHandle = Stm32UartGetHandle;
	pDev->DevIntrf.Reset = Stm32UartReset;
	pDev->DevIntrf.Disable = Stm32UartDisable;
	pDev->DevIntrf.Enable = Stm32UartEnable;
	pDev->DevIntrf.GetRate = Stm32UartGetRate;
	pDev->DevIntrf.SetRate = Stm32UartSetRate;
	pDev->DevIntrf.StartRx = Stm32UartStartRx;
	pDev->DevIntrf.RxData = Stm32UartRxData;
	pDev->DevIntrf.StopRx = Stm32UartStopRx;
	pDev->DevIntrf.StartTx = Stm32UartStartTx;
	pDev->DevIntrf.TxData = Stm32UartTxData;
	pDev->DevIntrf.StopTx = Stm32UartStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = Stm32UartPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	pDev->Mode = pCfg->Mode;
	pDev->Duplex = pCfg->Duplex;
	pDev->DataBits = pCfg->DataBits;
	pDev->Parity = pCfg->Parity;
	pDev->StopBits = pCfg->StopBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->bRxReady = false;
	pDev->bTxReady = true;
	pDev->RxDropCnt = 0;
	pDev->TxDropCnt = 0;

	dev->RxDropCnt = 0;
	dev->RxTimeoutCnt = 0;
	dev->ErrCnt = 0;
	dev->ErrFlag = 0;

	if (Stm32UartHwSetRate(dev, (uint32_t)pCfg->Rate) == 0U ||
		!Stm32UartHwConfigureFrame(dev, pCfg))
	{
		Stm32UartHwDisableClock(dev);
		return false;
	}

	reg->CR2 &= ~USART_CR2_STOP_Msk;

	if (pCfg->StopBits == 2)
	{
		reg->CR2 |= USART_CR2_STOP_1;
	}

	reg->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR |
				  USART_CR3_CTSE | USART_CR3_RTSE | USART_CR3_EIE);
#ifdef USART_CR3_CTSIE
	reg->CR3 &= ~USART_CR3_CTSIE;
#endif

	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		reg->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
	}

	reg->CR3 &= ~USART_CR3_HDSEL;

	if (pCfg->Duplex == UART_DUPLEX_HALF)
	{
		reg->CR3 |= USART_CR3_HDSEL;
	}

	uint32_t clear = USART_ICR_PECF | USART_ICR_FECF |
					 USART_ICR_ORECF | USART_ICR_NCF;
#if defined(USART_ICR_RTOCF)
	clear |= USART_ICR_RTOCF;
#endif
	reg->ICR = clear;

#if defined(USART_RQR_RXFRQ)
	reg->RQR = USART_RQR_RXFRQ;
#endif
	(void)reg->RDR;

	uint32_t cr1 = reg->CR1;
	uint32_t intmask = USART_CR1_PEIE | STM32_UART_CR1_TXIE |
					   STM32_UART_CR1_RXIE | USART_CR1_TCIE | USART_CR1_IDLEIE;
#if defined(USART_CR1_RTOIE)
	intmask |= USART_CR1_RTOIE;
#endif

	cr1 &= ~intmask;

	if (pCfg->bIntMode)
	{
		cr1 |= USART_CR1_PEIE | STM32_UART_CR1_RXIE;
		reg->CR3 |= USART_CR3_EIE;

		if (!Stm32UartHwEnableIRQ(dev, pCfg->IntPrio))
		{
			Stm32UartHwDisableClock(dev);
			return false;
		}
	}

#if defined(USART_CR2_RTOEN)
	reg->CR2 &= ~USART_CR2_RTOEN;
#endif

	reg->CR1 = cr1 | USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	return true;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
	if (pDev != NULL)
	{
		pDev->LineState = LineState;
	}
}


#if defined(IOSONATA_STM32_F0)

extern "C" void USART1_IRQHandler()
{
	Stm32UartIRQ(&s_UartDev[0]);
}

#if defined(USART2)
extern "C" void USART2_IRQHandler()
{
	Stm32UartIRQ(&s_UartDev[1]);
}
#endif

#if defined(STM32F070xB) || defined(STM32F030xC)
#if defined(STM32F030xC)
extern "C" void USART3_6_IRQHandler()
#else
extern "C" void USART3_4_IRQHandler()
#endif
{
	for (int i = 2; i < s_NbUartDev; i++)
	{
		Stm32UartIRQ(&s_UartDev[i]);
	}
}
#endif

#elif defined(IOSONATA_STM32_L4)

extern "C" void USART1_IRQHandler()
{
	Stm32UartIRQ(&s_UartDev[1]);
}

extern "C" void USART2_IRQHandler()
{
	Stm32UartIRQ(&s_UartDev[2]);
}

extern "C" void USART3_IRQHandler()
{
	Stm32UartIRQ(&s_UartDev[3]);
}

extern "C" void UART4_IRQHandler()
{
	Stm32UartIRQ(&s_UartDev[4]);
}

extern "C" void UART5_IRQHandler()
{
	Stm32UartIRQ(&s_UartDev[5]);
}

extern "C" void LPUART1_IRQHandler()
{
	Stm32UartIRQ(&s_UartDev[0]);
}

#endif
