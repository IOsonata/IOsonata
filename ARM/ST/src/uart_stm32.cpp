/**-------------------------------------------------------------------------
@file	uart_stm32.cpp

@brief	Common STM32 UART interrupt transport and family-specific peripheral setup.

@author	Hoang Nguyen Hoan
@date	June 7, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

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


#include "stm32.h"

#include "coredev/uart.h"
static int Stm32UartRxFifoRead(UARTDEV *pDev, uint8_t *pBuff, int Bufflen)
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


#if defined(IOSONATA_STM32_F0)

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"


#define STM32F0X_UART_HWFIFO_SIZE		6
#define STM32F0X_UART_RXTIMEOUT			15
#define STM32F0X_UART_BUFF_SIZE			16
#define STM32F0X_UART_CFIFO_SIZE		CFIFO_MEMSIZE(STM32F0X_UART_BUFF_SIZE)

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct _STM32F0X_UART_Dev {
	int DevNo;				// UART interface number
	USART_TypeDef *pReg;	// UART registers
	UARTDEV	*pUartDev;		// Pointer to generic UART dev. data
	uint32_t RxDropCnt;
	uint32_t RxTimeoutCnt;
	uint32_t TxDropCnt;
	uint32_t ErrCnt;
	uint32_t RxPin;
	uint32_t TxPin;
	uint32_t CtsPin;
	uint32_t RtsPin;
	DMA_Channel_TypeDef *pTxDma;
	uint32_t TxDmaShift;
	IRQn_Type TxDmaIrq;
	bool DmaOwned;
	alignas(4) uint8_t TxDmaCache[STM32F0X_UART_BUFF_SIZE];
	uint8_t RxFifoMem[STM32F0X_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[STM32F0X_UART_CFIFO_SIZE];
} STM32F0X_UARTDEV;

#pragma pack(pop)

//static uint32_t s_FclkFreq = SYSTEM_CORE_CLOCK;		// FCLK frequency in Hz

static STM32F0X_UARTDEV s_Stm32f03xUartDev[] = {
	{
		.DevNo = 0,
		.pReg = USART1,
#ifdef STM32F030x8
		.pTxDma = DMA1_Channel2,
		.TxDmaShift = 4,
		.TxDmaIrq = DMA1_Channel2_3_IRQn,
#endif
	},
#if defined(USART2)
	{
		.DevNo = 1,
		.pReg = USART2,
#ifdef STM32F030x8
		.pTxDma = DMA1_Channel4,
		.TxDmaShift = 12,
		.TxDmaIrq = DMA1_Channel4_5_IRQn,
#endif
	},
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
	{
		.DevNo = 2,
		.pReg = USART3,
	},
	{
		.DevNo = 3,
		.pReg = USART4,
	},
#endif
#ifdef STM32F030xC
	{
		.DevNo = 4,
		.pReg = USART5,
	},
	{
		.DevNo = 5,
		.pReg = USART6,
	},
#endif
};

static const int s_NbUartDev = sizeof(s_Stm32f03xUartDev) / sizeof(STM32F0X_UARTDEV);

UARTDEV const *UARTGetInstance(int DevNo)
{
	return DevNo >= 0 && DevNo < s_NbUartDev ? s_Stm32f03xUartDev[DevNo].pUartDev : NULL;
}

// The channel is disabled before either caller fills the fixed TX cache.
// CPAR and CMAR are constant and are set once during initialization.
static void STM32F03xUARTDmaArm(STM32F0X_UARTDEV *dev, int Count)
{
	dev->pTxDma->CNDTR = Count;
	dev->pUartDev->bTxReady = false;
	__DMB();
	dev->pTxDma->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN;
}

// Call with interrupts masked. Copy before the producer can reuse the span.
static void STM32F03xUARTDmaStart(STM32F0X_UARTDEV *dev)
{
	int len = STM32F0X_UART_BUFF_SIZE;
	uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &len);
	if (p == NULL)
	{
		dev->pUartDev->bTxReady = true;
		return;
	}
	if (len == 1) dev->TxDmaCache[0] = *p;
	else memcpy(dev->TxDmaCache, p, len);
	STM32F03xUARTDmaArm(dev, len);
}

static void STM32F03xUARTDmaStop(STM32F0X_UARTDEV *dev)
{
	if (!dev->DmaOwned) return;
	dev->pReg->CR3 &= ~USART_CR3_DMAT;
	dev->pTxDma->CCR = 0;
	DMA1->IFCR = DMA_IFCR_CGIF1 << dev->TxDmaShift;
	dev->pTxDma->CNDTR = 0;
	dev->pUartDev->bTxReady = true;
}

#ifdef STM32F030x8
static void STM32F03xUARTDmaIrq(STM32F0X_UARTDEV *dev)
{
	uint32_t state = DisableInterrupt();
	if (!dev->DmaOwned || !(dev->pTxDma->CCR & (DMA_CCR_TCIE | DMA_CCR_TEIE)))
	{
		EnableInterrupt(state);
		return;
	}
	uint32_t flags = (DMA1->ISR >> dev->TxDmaShift) & (DMA_ISR_TCIF1 | DMA_ISR_TEIF1);
	if (flags == 0)
	{
		EnableInterrupt(state);
		return;
	}
	dev->pTxDma->CCR = 0;
	if (flags & DMA_ISR_TEIF1)
	{
		dev->ErrCnt++;
		dev->pUartDev->TxDropCnt += dev->pTxDma->CNDTR;
	}
	DMA1->IFCR = DMA_IFCR_CGIF1 << dev->TxDmaShift;
	STM32F03xUARTDmaStart(dev);
	bool ready = dev->pUartDev->bTxReady;
	EnableInterrupt(state);
	if (ready && dev->pUartDev->EvtCallback)
		dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, 0);
}

// Only UART's channel flags are acknowledged. Channels 3 and 5 are unused
// by this driver; future users must share these vector handlers.
extern "C" void DMA1_Channel2_3_IRQHandler()
{
	STM32F03xUARTDmaIrq(&s_Stm32f03xUartDev[0]);
}

extern "C" void DMA1_Channel4_5_IRQHandler()
{
	STM32F03xUARTDmaIrq(&s_Stm32f03xUartDev[1]);
}
#endif

static void UART_IRQHandler(STM32F0X_UARTDEV * const pDev)
{
	UARTDEV *dev = (UARTDEV *)pDev->pUartDev;
	if (dev == NULL || !dev->DevIntrf.bIntEn)
	{
		return;
	}
	uint32_t iflag = pDev->pReg->ISR;
	uint32_t cr1 = pDev->pReg->CR1;

	// RX errors: clear only the error flags in ICR, then drain RDR to
	// release RXNE and let the line recover.
	if (iflag & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE))
	{
		pDev->ErrCnt++;
		pDev->pReg->ICR = iflag & (USART_ICR_PECF | USART_ICR_FECF |
		                           USART_ICR_ORECF | USART_ICR_NCF);
		if (iflag & USART_ISR_RXNE)
		{
			(void)pDev->pReg->RDR;
		}
		iflag &= ~USART_ISR_RXNE;
	}

	if ((iflag & USART_ISR_RXNE) && (cr1 & USART_CR1_RXNEIE))
	{
		uint8_t c = (uint8_t)pDev->pReg->RDR & (dev->DataBits == 7 ? 0x7F : 0xFF);    // read clears RXNE
		uint8_t *p = CFifoPut(dev->hRxFifo);
		if (p != NULL)
		{
			*p = c;
			dev->bRxReady = true;
		}
		else
		{
			pDev->RxDropCnt++;
		}
		if (dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_RXDATA, NULL, CFifoUsed(dev->hRxFifo));
		}
	}

	if ((iflag & USART_ISR_TXE) && (cr1 & USART_CR1_TXEIE))
	{
		uint8_t *p = CFifoGet(dev->hTxFifo);
		if (p != NULL)
		{
			pDev->pReg->TDR = *p;             // write clears TXE
		}
		else
		{
			// FIFO empty: mask TXEIE so we don't storm on TXE.
			pDev->pReg->CR1 &= ~USART_CR1_TXEIE;
			dev->bTxReady = true;
			if (dev->EvtCallback)
			{
				dev->EvtCallback(dev, UART_EVT_TXREADY, NULL, 0);
			}
		}
	}

	if ((iflag & USART_ISR_RTOF) && (cr1 & USART_CR1_RTOIE))
	{
		pDev->pReg->ICR = USART_ICR_RTOCF;
		if (dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_RXTIMEOUT, NULL, CFifoUsed(dev->hRxFifo));
		}
	}

	// Note: TXE and RXNE are not ICR-clearable. No blanket "ICR = iflag".
}

extern "C" void USART1_IRQHandler()
{
	UART_IRQHandler(&s_Stm32f03xUartDev[0]);
}

#if defined(USART2)
extern "C" void USART2_IRQHandler()
{
	UART_IRQHandler(&s_Stm32f03xUartDev[1]);
}
#endif

#if defined(STM32F070xB) || defined(STM32F030xC)
#ifdef STM32F030xC
extern "C" void USART3_6_IRQHandler()
#else
extern "C" void USART3_4_IRQHandler()
#endif
{
	for (int i = 2; i < s_NbUartDev; i++)
	{
		UART_IRQHandler(&s_Stm32f03xUartDev[i]);
	}
}
#endif

static uint32_t STM32F03xUARTGetRate(DevIntrf_t * const pDev)
{
	return ((STM32F0X_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

static uint32_t STM32F03xUARTSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
	uint32_t fclkfreq = SystemPeriphClockGet(0);
	if (Rate == 0 || Rate > fclkfreq / 16)
	{
		return 0;
	}
	uint32_t div = (fclkfreq + (Rate >> 1)) / Rate;
	if (div < 16 || div > 0xFFFF)
	{
		return 0;
	}
	// Do not change the USART clock divider during an active DMA burst.
	if (dev->DmaOwned && !dev->pUartDev->bTxReady) return 0;

	uint32_t cr1 = dev->pReg->CR1;
	dev->pReg->CR1 = cr1 & ~USART_CR1_UE;
	dev->pReg->CR1 &= ~USART_CR1_OVER8;
	dev->pReg->BRR = div;
	dev->pReg->CR1 = cr1 & ~USART_CR1_OVER8;
	dev->pUartDev->Rate = fclkfreq / div;

	return dev->pUartDev->Rate;
}

static bool STM32F03xUARTStartRx(DevIntrf_t * const pSerDev, uint32_t DevAddr)
{
	return true;
}

static int STM32F03xUARTRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
	int cnt = 0;

	if (!pDev->bIntEn)
	{
		while (cnt < Bufflen)
		{
			uint32_t flags = dev->pReg->ISR;
			if (flags & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE))
			{
				dev->pReg->ICR = flags & (USART_ICR_PECF | USART_ICR_FECF |
						USART_ICR_ORECF | USART_ICR_NCF);
				if (flags & USART_ISR_RXNE) (void)dev->pReg->RDR;
				break;
			}
			if (!(flags & USART_ISR_RXNE)) break;
			pBuff[cnt++] = (uint8_t)dev->pReg->RDR & (dev->pUartDev->DataBits == 7 ? 0x7F : 0xFF);
		}
		return cnt;
	}

	uint32_t state = DisableInterrupt();
	cnt = Stm32UartRxFifoRead(dev->pUartDev, pBuff, Bufflen);
	EnableInterrupt(state);

	return cnt;
}

static void STM32F03xUARTStopRx(DevIntrf_t * const pDev) {
}

static bool STM32F03xUARTStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int STM32F03xUARTTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
	if (pData == NULL || Datalen <= 0 || !(dev->pReg->CR1 & USART_CR1_UE)) return 0;
    int cnt = 0;
    int rtry = pDev->MaxRetry;

	if (!pDev->bIntEn)
	{
		// In polling mode wait for TDR availability rather than returning
		// immediately between consecutive bytes. Preserve partial writes on
		// a stalled peripheral and never overwrite TDR while TXE is clear.
		while (cnt < Datalen)
		{
			int retry = pDev->MaxRetry;
			while ((dev->pReg->ISR & USART_ISR_TXE) == 0U && retry-- > 0)
			{
				if ((dev->pReg->CR1 & USART_CR1_UE) == 0U)
				{
					return cnt;
				}
			}
			if ((dev->pReg->ISR & USART_ISR_TXE) == 0U)
			{
				break;
			}
			dev->pReg->TDR = pData[cnt++];
		}
		return cnt;
	}

    while (Datalen > 0 && rtry-- > 0)
    {
        uint32_t state = DisableInterrupt();
		if (!(dev->pReg->CR1 & USART_CR1_UE))
		{
			EnableInterrupt(state);
			break;
		}

		// Ready means both the DMA cache and queued FIFO are empty.
		// Avoid a FIFO round trip when the caller can fill the cache directly.
		if (pDev->bDma && dev->pUartDev->bTxReady)
		{
			int len = Datalen < STM32F0X_UART_BUFF_SIZE ? Datalen : STM32F0X_UART_BUFF_SIZE;
			if (len == 1) dev->TxDmaCache[0] = *pData;
			else memcpy(dev->TxDmaCache, pData, len);
			STM32F03xUARTDmaArm(dev, len);
			Datalen -= len;
			pData += len;
			cnt += len;
		}

        while (Datalen > 0)
        {
            int l = Datalen;
            uint8_t *p = l == 1 ? CFifoPut(dev->pUartDev->hTxFifo) :
				CFifoPutMultiple(dev->pUartDev->hTxFifo, &l);
            if (p == NULL)
                break;
			if (l == 1) *p = *pData;
			else memcpy(p, pData, l);
            Datalen -= l;
            pData += l;
            cnt += l;
        }

		if (pDev->bDma)
		{
			EnableInterrupt(state);
			continue;
		}

        // Kick off TX from inside the critical section so the ISR cannot
        // double-fetch the same byte we just queued and overwrite TDR.
        if (dev->pUartDev->bTxReady && (dev->pReg->ISR & USART_ISR_TXE))
        {
            uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
            if (p != NULL)
            {
                dev->pUartDev->bTxReady = false;
                dev->pReg->TDR = *p;
                dev->pReg->CR1 |= USART_CR1_TXEIE;
            }
        }

        if (CFifoUsed(dev->pUartDev->hTxFifo) > 0)
        {
            dev->pUartDev->bTxReady = false;
            dev->pReg->CR1 |= USART_CR1_TXEIE;
        }

        EnableInterrupt(state);
    }
    return cnt;
}

static inline void STM32F03xUARTStopTx(DevIntrf_t * const pDev) {
}

static void STM32F03xUARTDisable(DevIntrf_t * const pDev)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
	uint32_t state = DisableInterrupt();
	STM32F03xUARTDmaStop(dev);

	dev->pReg->CR1 &= ~(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
	dev->pReg->CR2 &= ~USART_CR2_RTOEN;

	EnableInterrupt(state);
}

static void STM32F03xUARTEnable(DevIntrf_t * const pDev)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
	uint32_t state = DisableInterrupt();
	STM32F03xUARTDmaStop(dev);

	dev->ErrCnt = 0;
	dev->RxTimeoutCnt = 0;
	dev->RxDropCnt = 0;
	dev->TxDropCnt = 0;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;

	if (dev->DevNo == 0)
	{
		RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;
	}
	if (dev->DmaOwned) dev->pReg->CR3 |= USART_CR3_DMAT;
	dev->pReg->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	dev->pReg->CR2 &= ~USART_CR2_RTOEN;

	EnableInterrupt(state);
}

static void STM32F03xUARTPowerOff(DevIntrf_t * const pDev)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;

	STM32F03xUARTDisable(pDev);

	switch (dev->DevNo)
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
#ifdef STM32F030xC
		case 4:
			RCC->APB1ENR &= ~RCC_APB1ENR_USART5EN;
			break;
		case 5:
			RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
			break;
#endif
	}

	dev->pReg->CR2 &= ~USART_CR2_CLKEN;
}

static void STM32F03xUARTReset(DevIntrf_t * const pDev)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
	uint32_t state = DisableInterrupt();
	STM32F03xUARTDmaStop(dev);
	if (dev->DmaOwned) CFifoFlush(dev->pUartDev->hTxFifo);

	switch (dev->DevNo)
	{
		case 0:
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
			usDelay(100);
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
			break;
#if defined(USART2)
		case 1:
			RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
			usDelay(100);
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
			break;
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
		case 2:
			RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
			usDelay(100);
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
			break;
		case 3:
			RCC->APB1RSTR |= RCC_APB1RSTR_USART4RST;
			usDelay(100);
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART4RST;
			break;
#endif
#ifdef STM32F030xC
		case 4:
			RCC->APB1RSTR |= RCC_APB1RSTR_USART5RST;
			usDelay(100);
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART5RST;
			break;
		case 5:
			RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST;
			usDelay(100);
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
			break;
#endif
	}
	EnableInterrupt(state);
}

static void *STM32F03xUARTGetHandle(DevIntrf_t * const pDev)
{
	return ((STM32F0X_UARTDEV *)pDev->pDevData)->pUartDev;
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

	// TX DMA uses completion interrupts; RX remains a character stream.
	// DMA routing is provided for STM32F030x8 only.
	if ((pCfg->bDMAMode && !pCfg->bIntMode) || pCfg->bIrDAMode || pCfg->Mode != UART_MODE_UART ||
		(pCfg->FlowControl != UART_FLWCTRL_NONE && pCfg->FlowControl != UART_FLWCTRL_HW) ||
		(pCfg->Parity != UART_PARITY_NONE && pCfg->Parity != UART_PARITY_EVEN && pCfg->Parity != UART_PARITY_ODD) ||
		(pCfg->DataBits != 7 && pCfg->DataBits != 8) ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) || pCfg->Rate <= 0)
	{
		return false;
	}
	int wordbits = pCfg->DataBits + (pCfg->Parity != UART_PARITY_NONE ? 1 : 0);
#ifndef USART_CR1_M1
	if (wordbits == 7) return false;
#endif
	uint32_t clock = SystemPeriphClockGet(0);
	uint32_t divisor = (clock + ((uint32_t)pCfg->Rate >> 1)) / (uint32_t)pCfg->Rate;
	if (divisor < 16 || divisor > 0xFFFF || (uint32_t)pCfg->Rate > clock / 16)
	{
		return false;
	}

	int devno = pCfg->DevNo;
	STM32F0X_UARTDEV *dev = &s_Stm32f03xUartDev[devno];
	USART_TypeDef *reg = dev->pReg;
	uint32_t state = DisableInterrupt();
	// Do not take an active UART DMA channel from another device object.
	if ((dev->DmaOwned && dev->pUartDev != pDev) ||
		(pCfg->bDMAMode && !dev->DmaOwned && dev->pTxDma->CCR != 0))
	{
		EnableInterrupt(state);
		return false;
	}
	STM32F03xUARTDmaStop(dev);
	dev->DmaOwned = false;

	pDev->DevIntrf.pDevData = &s_Stm32f03xUartDev[devno];
	s_Stm32f03xUartDev[devno].pUartDev = pDev;

	// Enable peripheral clock FIRST. Any register access before this is
	// silently dropped by the bus matrix.
	switch (devno)
	{
		case 0:
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			break;
#if defined(USART2)
		case 1:
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
			break;
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
		case 2:
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
			break;
		case 3:
			RCC->APB1ENR |= RCC_APB1ENR_USART4EN;
			break;
#endif
#ifdef STM32F030xC
		case 4:
			RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
			break;
		case 5:
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
			break;
#endif
	}

	// Select PCLK for USART1 too, so all baud divisors use the APB clock.
	if (devno == 0)
	{
		RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;
	}

	STM32F03xUARTReset(&pDev->DevIntrf);

	// Disable UART so OVER8/M/parity/stop-bit fields are writable.
	reg->CR1 &= ~USART_CR1_UE;

	// Do NOT set CR2.CLKEN here — that would enable the synchronous CK
	// output on PA8 (AF1) for smartcard/sync modes. Async UART uses no
	// clock pin.


	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_Stm32f03xUartDev[devno].RxFifoMem, STM32F0X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_Stm32f03xUartDev[devno].TxFifoMem, STM32F0X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	// Configure UART pins
	IOPINCFG *pincfg = (IOPINCFG*)pCfg->pIOPinMap;

	IOPinCfg(pincfg, pCfg->NbIOPins);

    // Set baud
    pDev->Rate = STM32F03xUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

	reg->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
#ifdef USART_CR1_M1
	reg->CR1 &= ~USART_CR1_M1;
	if (wordbits == 7) reg->CR1 |= USART_CR1_M1;
#endif
	if (pCfg->Parity != UART_PARITY_NONE) reg->CR1 |= USART_CR1_PCE;
	if (pCfg->Parity == UART_PARITY_ODD) reg->CR1 |= USART_CR1_PS;
	if (wordbits == 9) reg->CR1 |= USART_CR1_M;

	reg->CR2 &= ~USART_CR2_STOP_Msk;
	if (pCfg->StopBits == 2) reg->CR2 |= USART_CR2_STOP_1;

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	reg->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
	}
	else
	{
    	reg->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	}


    s_Stm32f03xUartDev[devno].pUartDev->bRxReady = false;
    s_Stm32f03xUartDev[devno].pUartDev->bTxReady = true;
    s_Stm32f03xUartDev[devno].ErrCnt = 0;
    s_Stm32f03xUartDev[devno].RxTimeoutCnt = 0;
    s_Stm32f03xUartDev[devno].RxDropCnt = 0;
    s_Stm32f03xUartDev[devno].TxDropCnt = 0;
	pDev->TxDropCnt = 0;

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
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = NULL;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.GetHandle = STM32F03xUARTGetHandle;
	pDev->DevIntrf.Reset = STM32F03xUARTReset;
	pDev->DevIntrf.Disable = STM32F03xUARTDisable;
	pDev->DevIntrf.Enable = STM32F03xUARTEnable;
	pDev->DevIntrf.GetRate = STM32F03xUARTGetRate;
	pDev->DevIntrf.SetRate = STM32F03xUARTSetRate;
	pDev->DevIntrf.StartRx = STM32F03xUARTStartRx;
	pDev->DevIntrf.RxData = STM32F03xUARTRxData;
	pDev->DevIntrf.StopRx = STM32F03xUARTStopRx;
	pDev->DevIntrf.StartTx = STM32F03xUARTStartTx;
	pDev->DevIntrf.TxData = STM32F03xUARTTxData;
	pDev->DevIntrf.StopTx = STM32F03xUARTStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = STM32F03xUARTPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_CTSIE | USART_CR3_EIE);

	// Reference transport: USART interrupt TX and RX, no DMA requests.
	// DMA mode selection is accepted with interrupts for baseline comparison.
	reg->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

	// Select duplex mode
	reg->CR3 &= ~USART_CR3_HDSEL;
	if (pDev->Duplex == UART_DUPLEX_HALF)
	{
		reg->CR3 |= USART_CR3_HDSEL;
	}

	uint32_t tmp = reg->CR1;

	// Disable all USART interrupts while we configure.
	tmp &= ~(USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_TCIE |
	         USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_RTOIE);

	if (pCfg->bIntMode)
	{
		// Enable only the interrupts the ISR handles. TXEIE is armed on
		// demand by STM32F03xUARTTxData, not here, so the ISR doesn't
		// fire on a permanently-empty FIFO at boot. TCIE, IDLEIE and
		// RTOIE are NOT enabled — their flags don't auto-clear and would
		// cause an interrupt storm.
		tmp |= USART_CR1_RXNEIE | USART_CR1_PEIE;

		// CTSE/RTSE perform hardware flow control without a CTS interrupt.
		reg->CR3 |= USART_CR3_EIE;   // FE, NE, ORE error IRQ via CR3

		switch (devno)
		{
			case 0:
				NVIC_ClearPendingIRQ(USART1_IRQn);
				NVIC_SetPriority(USART1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART1_IRQn);
				break;
#if defined(USART2)
			case 1:
				NVIC_ClearPendingIRQ(USART2_IRQn);
				NVIC_SetPriority(USART2_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART2_IRQn);
				break;
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
			default:
#ifdef STM32F030xC
				NVIC_SetPriority(USART3_6_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART3_6_IRQn);
#else
				NVIC_SetPriority(USART3_4_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART3_4_IRQn);
#endif
				break;
#endif
		}
    }

	// Enable USART
	tmp |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	reg->CR1 = tmp;
	// Note: RTOEN intentionally not set. Receiver timeout needs RTOR
	// programmed with a sensible bit count; the demo doesn't use it.

	EnableInterrupt(state);
	return true;
}

void UARTSetCtrlLineState(UARTDEV * const pDev, uint32_t LineState)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->DevIntrf.pDevData;

}




#endif // IOSONATA_STM32_F0

#if defined(IOSONATA_STM32_L4)

#include "istddef.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "coredev/interrupt.h"

#define RCC_CCIPR_USARTSEL_PCLK			(0)
#define RCC_CCIPR_USARTSEL_SYSCLK		(1)
#define RCC_CCIPR_USARTSEL_HSI16		(2)
#define RCC_CCIPR_USARTSEL_LSE			(3)

#define RCC_CCIPR_USARTSEL(n, clk)		(clk<<((n-1) << 1))

#ifdef STM32L4S9xx
#define USART_ISR_RXNE			USART_ISR_RXNE_RXFNE
#define USART_ISR_TXE			USART_ISR_TXE_TXFNF
#define USART_CR1_TXEIE			USART_CR1_TXEIE_TXFNFIE
#define USART_CR1_RXNEIE		USART_CR1_RXNEIE_RXFNEIE
#endif

#define STM32L4X_UART_HWFIFO_SIZE		8
#define STM32L4X_UART_RXTIMEOUT			15
#define STM32L4X_UART_BUFF_SIZE			(4 * STM32L4X_UART_HWFIFO_SIZE)
#define STM32L4X_UART_CFIFO_SIZE		CFIFO_MEMSIZE(STM32L4X_UART_BUFF_SIZE)

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct _STM32L4X_UART_Dev {
	int DevNo;				// UART interface number
	USART_TypeDef *pReg;	// UART registers
	UARTDev_t	*pUartDev;		// Pointer to generic UART dev. data
	uint32_t RxDropCnt;
	uint32_t RxTimeoutCnt;
	uint32_t ErrCnt;
	const IOPinCfg_t *pIOPinMap;
	int NbPins;
	uint8_t TxDmaCache[STM32L4X_UART_BUFF_SIZE];
	uint8_t RxFifoMem[STM32L4X_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[STM32L4X_UART_CFIFO_SIZE];
	uint32_t ErrFlag;
} STM32L4X_UARTDEV;

#pragma pack(pop)

static uint32_t s_FclkFreq = SystemCoreClock;		// FCLK frequency in Hz

static STM32L4X_UARTDEV s_Stm32l4xUartDev[] = {
	{
		.DevNo = 0,
		.pReg = LPUART1,
	},
	{
		.DevNo = 1,
		.pReg = USART1,
	},
	{
		.DevNo = 2,
		.pReg = USART2,
	},
	{
		.DevNo = 3,
		.pReg = USART3,
	},
	{
		.DevNo = 4,
		.pReg = UART4,
	},
	{
		.DevNo = 5,
		.pReg = UART5,
	},
};

static const int s_NbUartDev = sizeof(s_Stm32l4xUartDev) / sizeof(STM32L4X_UARTDEV);

UARTDev_t const *UARTGetInstance(int DevNo)
{
	return s_Stm32l4xUartDev[DevNo].pUartDev;
}

bool STM32L4xUARTWaitForRxReady(STM32L4X_UARTDEV * const pDev, uint32_t Timeout)
{
	return false;
}

bool STM32L4xUARTWaitForTxReady(STM32L4X_UARTDEV * const pDev, uint32_t Timeout)
{
	return false;
}

static void UART_IRQHandler(STM32L4X_UARTDEV * const pDev)
{
	UARTDev_t *dev = (UARTDev_t *)pDev->pUartDev;
	int len = 0;
	int cnt = 0;
	uint32_t iflag = pDev->pReg->ISR;

	if (iflag & USART_ISR_RXNE)
	{
		int cnt = STM32L4X_UART_HWFIFO_SIZE;
		do {
			// RDR belongs to the ISR, including when the software FIFO is full.
			uint8_t value = (uint16_t)pDev->pReg->RDR & (dev->DataBits == 7 ? 0x7F : 0xFF);
			uint8_t *p = CFifoPut(dev->hRxFifo);
			if (p)
			{
				*p = value;
				dev->bRxReady = true;
			}
			else
			{
				pDev->RxDropCnt++;
			}
		} while (pDev->pReg->ISR & USART_ISR_RXNE && cnt-- > 0);
		if (dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_RXDATA, NULL, CFifoUsed(dev->hRxFifo));
		}
	}
	if (iflag & USART_ISR_TXE)// | USART_ISR_TC))
	{
		//pDev->pReg->ICR = USART_ISR_TXE;
		int cnt = STM32L4X_UART_HWFIFO_SIZE;

		do {
			uint8_t *p = CFifoGet(dev->hTxFifo);
			if (p == NULL)
			{
				dev->bTxReady = true;
				break;
			}
			dev->bTxReady = false;
			pDev->pReg->TDR = *p;
		} while (pDev->pReg->ISR & USART_ISR_TXE && cnt-- > 0);

		if (dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_TXREADY, NULL, CFifoUsed(dev->hRxFifo));
		}
	}
	if (iflag & USART_ISR_RTOF)
	{
		if (dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_RXTIMEOUT, NULL, CFifoUsed(dev->hRxFifo));
		}
		//pDev->pReg->ICR = USART_ISR_RTOF;
	}

	if (iflag & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE))
	{
		// error
		pDev->ErrCnt++;
		pDev->ErrFlag = iflag & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
		//pDev->pReg->ICR = (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
	}
	pDev->pReg->ICR = iflag & 0x121BDF;
}

extern "C" void USART1_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[1]);
	NVIC_ClearPendingIRQ(USART1_IRQn);
}

extern "C" void USART2_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[2]);
	NVIC_ClearPendingIRQ(USART2_IRQn);
}

extern "C" void USART3_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[3]);
	NVIC_ClearPendingIRQ(USART3_IRQn);
}

extern "C" void UART4_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[4]);
	NVIC_ClearPendingIRQ(UART4_IRQn);
}

extern "C" void UART5_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[5]);
	NVIC_ClearPendingIRQ(UART5_IRQn);
}

extern "C" void LPUART1_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[0]);
	NVIC_ClearPendingIRQ(LPUART1_IRQn);
}

static uint32_t STM32L4xUARTGetRate(DevIntrf_t * const pDev)
{
	return ((STM32L4X_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

static uint32_t STM32L4xUARTSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;
	uint32_t div;

	if (dev->DevNo == 0)
	{
		uint64_t fclk2 = 16000000 << 8;
		uint32_t tmp = RCC->CCIPR;
		if (GetLowFreqOscType() == OSC_TYPE_XTAL && Rate < 10922)
		{
			fclk2 = 32768 << 8;
			tmp = (tmp & ~RCC_CCIPR_LPUART1SEL_Msk) | RCC_CCIPR_USARTSEL(6, RCC_CCIPR_USARTSEL_LSE);
		}
		else if (Rate < 5333333)
		{
			tmp = (tmp & ~RCC_CCIPR_LPUART1SEL_Msk) | RCC_CCIPR_USARTSEL(6, RCC_CCIPR_USARTSEL_HSI16);
		}
		div = (fclk2 + (uint64_t)(Rate >> 1ULL)) / Rate;
		dev->pUartDev->Rate = fclk2 / div;
		RCC->CCIPR = tmp;
	}
	else
	{
		uint32_t fclk2 = s_FclkFreq << 1;
		uint32_t rem8 = fclk2 % Rate;
		uint32_t rem16 = s_FclkFreq % Rate;

		if (rem8 < rem16)
		{
			// /8 better
			s_Stm32l4xUartDev[dev->DevNo].pReg->CR1 |= USART_CR1_OVER8;

			div = (fclk2 + (Rate >> 1)) / Rate;
			dev->pUartDev->Rate = (fclk2 + (div >> 1)) / div;
			div = ((div & 0xf) >> 1) | (div & 0xFFFFFFF0);
		}
		else
		{
			// /16 better
			s_Stm32l4xUartDev[dev->DevNo].pReg->CR1 &= ~USART_CR1_OVER8;

			div = (s_FclkFreq + (Rate >> 1)) / Rate;
			dev->pUartDev->Rate = (s_FclkFreq + (div >> 1)) / div;
		}
	}
	s_Stm32l4xUartDev[dev->DevNo].pReg->BRR = div;

	return dev->pUartDev->Rate;
}

static bool STM32L4xUARTStartRx(DevIntrf_t * const pSerDev, uint32_t DevAddr)
{
	return true;
}

static int STM32L4xUARTRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	cnt = Stm32UartRxFifoRead(dev->pUartDev, pBuff, Bufflen);
	EnableInterrupt(state);

	return cnt;
}

static void STM32L4xUARTStopRx(DevIntrf_t * const pDev) {
}

static bool STM32L4xUARTStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int STM32L4xUARTTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;
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
        	uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
        	if (p != NULL)
        	{
        		dev->pUartDev->bTxReady = false;
        		dev->pReg->TDR = *p;
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

static void STM32L4xUARTStopTx(DevIntrf_t * const pDev)
{
}

static void STM32L4xUARTDisable(DevIntrf_t * const pDev)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;

	dev->pReg->CR1 &= ~(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
	dev->pReg->CR2 &= ~USART_CR2_RTOEN;

	if (dev->DevNo == 1)
	{
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	}
	else if (dev->DevNo == 0)
	{
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN;
	}
	else if (dev->DevNo >= 2)
	{
		RCC->APB1ENR1 &= ~(RCC_APB1ENR1_USART2EN << (dev->DevNo - 2));
	}

	IOPinDis(dev->pIOPinMap, dev->NbPins);
}

static void STM32L4xUARTEnable(DevIntrf_t * const pDev)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;

	dev->ErrCnt = 0;
	dev->RxTimeoutCnt = 0;
	dev->RxDropCnt = 0;
	atomic_flag_clear(&pDev->bBusy);

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;

	if (dev->DevNo == 1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	}
	else if (dev->DevNo == 0)
	{
		RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	}
	else if (dev->DevNo >= 2)
	{
		RCC->APB1ENR1 |= (RCC_APB1ENR1_USART2EN << (dev->DevNo - 2));
	}

	IOPinCfg(dev->pIOPinMap, dev->NbPins);
	for (int i = 0; i < dev->NbPins; i++)
	{
		if (dev->pIOPinMap[i].PortNo >= 0)
		{
			IOPinSetSpeed(dev->pIOPinMap[i].PortNo, dev->pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
		}
	}

	dev->pReg->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	dev->pReg->CR2 |= USART_CR2_RTOEN;
}

static void STM32L4xUARTPowerOff(DevIntrf_t * const pDev)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;

	STM32L4xUARTDisable(pDev);
	// DevNo 0 is LPUART1; the remaining entries are USART1 through UART5.
	if (dev->DevNo == 0)
	{
		RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL_Msk;
	}
	else
	{
		RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL_Msk << ((dev->DevNo - 1) << 1));
	}
}

static void STM32L4xUARTReset(DevIntrf_t * const pDev)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;

	if (dev->DevNo == 1)
	{
		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		usDelay(100);
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
	}
	else if (dev->DevNo == 0)
	{
		RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST;
		usDelay(100);
		RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_LPUART1RST;
	}
	else if (dev->DevNo >= 2)
	{
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST << (dev->DevNo - 2);
		usDelay(100);
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_USART2RST << (dev->DevNo - 2));
	}
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

	// The byte-oriented API supports seven or eight payload bits.
	if ((pCfg->DataBits != 7 && pCfg->DataBits != 8) ||
		(pCfg->Parity != UART_PARITY_NONE && pCfg->Parity != UART_PARITY_EVEN && pCfg->Parity != UART_PARITY_ODD) ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) ||
		(pCfg->DevNo == 0 && pCfg->DataBits == 7 && pCfg->Parity == UART_PARITY_NONE))
	{
		return false;
	}

	int devno = pCfg->DevNo;
	USART_TypeDef *reg = s_Stm32l4xUartDev[devno].pReg;
	uint32_t tmp;

	pDev->DevIntrf.pDevData = &s_Stm32l4xUartDev[devno];
	s_Stm32l4xUartDev[devno].pUartDev = pDev;
	s_Stm32l4xUartDev[devno].pIOPinMap = (const IOPinCfg_t*)pCfg->pIOPinMap;
	s_Stm32l4xUartDev[devno].NbPins = pCfg->NbIOPins;
	STM32L4xUARTReset(&pDev->DevIntrf);

	// Disable UART first because some field can't be set is it is already enabled
	reg->CR1 &= ~USART_CR1_UE;

	// Enable clock
	tmp = RCC->CCIPR;
	switch (devno)
	{
		case 0:
			if (GetLowFreqOscType() == OSC_TYPE_XTAL)
			{
				tmp = (tmp & ~RCC_CCIPR_LPUART1SEL_Msk) | RCC_CCIPR_USARTSEL(6, RCC_CCIPR_USARTSEL_LSE);
			}
			else
			{
				tmp = (tmp & ~RCC_CCIPR_LPUART1SEL_Msk) | RCC_CCIPR_USARTSEL(6, RCC_CCIPR_USARTSEL_HSI16);
			}
			RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
			break;
		case 1:
			tmp = (tmp & ~RCC_CCIPR_USART1SEL_Msk) | RCC_CCIPR_USARTSEL(1, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			break;
		case 2:
			tmp = (tmp & ~RCC_CCIPR_USART2SEL_Msk) | RCC_CCIPR_USARTSEL(2, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
			break;
		case 3:
			tmp = (tmp & ~RCC_CCIPR_USART3SEL_Msk) | RCC_CCIPR_USARTSEL(3, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
			break;
		case 4:
			tmp = (tmp & ~RCC_CCIPR_UART4SEL_Msk) | RCC_CCIPR_USARTSEL(4, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
			break;
		case 5:
			tmp = (tmp & ~RCC_CCIPR_UART5SEL_Msk) | RCC_CCIPR_USARTSEL(5, RCC_CCIPR_USARTSEL_SYSCLK);
			RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
			break;
	}
	RCC->CCIPR = tmp;

	msDelay(1);

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_Stm32l4xUartDev[devno].RxFifoMem, STM32L4X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_Stm32l4xUartDev[devno].TxFifoMem, STM32L4X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	// Configure UART pins
	IOPinCfg_t *pincfg = (IOPinCfg_t*)pCfg->pIOPinMap;

	IOPinCfg(pincfg, pCfg->NbIOPins);

	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		if (pincfg[i].PortNo >= 0)
		{
			IOPinSetSpeed(pincfg[i].PortNo, pincfg[i].PinNo, IOPINSPEED_TURBO);
		}
	}

	msDelay(1);

    // Set baud
    pDev->Rate = STM32L4xUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

    msDelay(1);

	// Hardware word length includes the parity bit.
	int wordbits = pCfg->DataBits + (pCfg->Parity != UART_PARITY_NONE ? 1 : 0);
	reg->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M0 | USART_CR1_M1);
	if (pCfg->Parity != UART_PARITY_NONE) reg->CR1 |= USART_CR1_PCE;
	if (pCfg->Parity == UART_PARITY_ODD) reg->CR1 |= USART_CR1_PS;
	if (wordbits == 9) reg->CR1 |= USART_CR1_M0;
	else if (wordbits == 7) reg->CR1 |= USART_CR1_M1;

	reg->CR2 &= ~USART_CR2_STOP_Msk;
	if (pCfg->StopBits == 2) reg->CR2 |= USART_CR2_STOP_1;

	// Swap
	//reg->CR2 |= USART_CR2_SWAP;

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	reg->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
	}
	else
	{
    	reg->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	}

    s_Stm32l4xUartDev[devno].pUartDev->bRxReady = false;
    s_Stm32l4xUartDev[devno].pUartDev->bTxReady = true;
    s_Stm32l4xUartDev[devno].ErrCnt = 0;
    s_Stm32l4xUartDev[devno].RxTimeoutCnt = 0;
    s_Stm32l4xUartDev[devno].RxDropCnt = 0;

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
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.Reset = STM32L4xUARTReset;
	pDev->DevIntrf.Disable = STM32L4xUARTDisable;
	pDev->DevIntrf.Enable = STM32L4xUARTEnable;
	pDev->DevIntrf.GetRate = STM32L4xUARTGetRate;
	pDev->DevIntrf.SetRate = STM32L4xUARTSetRate;
	pDev->DevIntrf.StartRx = STM32L4xUARTStartRx;
	pDev->DevIntrf.RxData = STM32L4xUARTRxData;
	pDev->DevIntrf.StopRx = STM32L4xUARTStopRx;
	pDev->DevIntrf.StartTx = STM32L4xUARTStartTx;
	pDev->DevIntrf.TxData = STM32L4xUARTTxData;
	pDev->DevIntrf.StopTx = STM32L4xUARTStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = STM32L4xUARTPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);


	if (false && pCfg->bDMAMode)
	{
/*		if (devno == 0)
		{
			SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
		}
		if (devno == 2)
		{
			// USART3_DMA_RMP
			SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART3_DMA_RMP;
		}*/
		// Not using DMA transfer on Rx. It is useless on UART as we need to process 1 char at a time
		// cannot wait until DMA buffer is filled.
		reg->CR3 |= USART_CR3_DMAT;
	}
	else
	{
/*		if (devno == 0)
		{
			SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART1TX_DMA_RMP | SYSCFG_CFGR1_USART1RX_DMA_RMP);
		}
		if (devno == 2)
		{
			SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART3_DMA_RMP);
		}
*/
		reg->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
	}

	// Select duplex mode
	reg->CR3 &= ~USART_CR3_HDSEL;
	if (pDev->Duplex == UART_DUPLEX_HALF)
	{
		reg->CR3 |= USART_CR3_HDSEL;
	}

	tmp = reg->CR1;

	reg->ICR = 0xFFFFFFFF;
	reg->RQR |= USART_RQR_RXFRQ;

	(void)reg->RDR;

	// Disable all interrupts
	tmp &= ~(USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_RTOIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE);

	if (pCfg->bIntMode)
	{
		tmp |= (USART_CR1_PEIE | USART_CR1_TCIE | USART_CR1_RXNEIE);

		if (pCfg->FlowControl == UART_FLWCTRL_HW)
	    {
			reg->CR3 |= USART_CR3_CTSIE;
	    }
		reg->CR3 |= USART_CR3_EIE;

		switch (devno)
		{
			case 1:
				NVIC_ClearPendingIRQ(USART1_IRQn);
				NVIC_SetPriority(USART1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART1_IRQn);
				break;
			case 2:
				NVIC_ClearPendingIRQ(USART2_IRQn);
				NVIC_SetPriority(USART2_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART2_IRQn);
				break;
			case 3:
				NVIC_ClearPendingIRQ(USART3_IRQn);
				NVIC_SetPriority(USART3_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART3_IRQn);
				break;

			case 4:
				NVIC_ClearPendingIRQ(UART4_IRQn);
				NVIC_SetPriority(UART4_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UART4_IRQn);
				break;
			case 5:
				NVIC_ClearPendingIRQ(UART5_IRQn);
				NVIC_SetPriority(UART5_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UART5_IRQn);
				break;

			case 0:
				NVIC_ClearPendingIRQ(LPUART1_IRQn);
				NVIC_SetPriority(LPUART1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(LPUART1_IRQn);
				break;
		}
    }

	msDelay(1);

	// Enable USART
	tmp |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	reg->CR1 = tmp;
	reg->CR2 |= USART_CR2_RTOEN;

	return true;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->DevIntrf.pDevData;

}


#endif // IOSONATA_STM32_L4
