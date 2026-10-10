/**-------------------------------------------------------------------------
@file	uart_stm32f0x.cpp

@brief	STM32F0x UART implementation

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
//#include <atomic>

#include "stm32f0xx.h"

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"


#define STM32F0X_UART_HWFIFO_SIZE		6
#define STM32F0X_UART_RXTIMEOUT			15
#define STM32F0X_UART_BUFF_SIZE			16
#define STM32F0X_UART_RXDMA_SIZE		64
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
	DMA_Channel_TypeDef *pRxDma;
	uint32_t TxDmaShift;
	uint32_t RxDmaShift;
	uint16_t RxDmaPos;
	IRQn_Type TxDmaIrq;
	bool DmaOwned;
	alignas(4) uint8_t TxDmaCache[STM32F0X_UART_BUFF_SIZE];
	alignas(4) uint8_t RxDmaCache[STM32F0X_UART_RXDMA_SIZE];
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
		.pRxDma = DMA1_Channel3,
		.TxDmaShift = 4,
		.RxDmaShift = 8,
		.TxDmaIrq = DMA1_Channel2_3_IRQn,
#endif
	},
#if defined(USART2)
	{
		.DevNo = 1,
		.pReg = USART2,
#ifdef STM32F030x8
		.pTxDma = DMA1_Channel4,
		.pRxDma = DMA1_Channel5,
		.TxDmaShift = 12,
		.RxDmaShift = 16,
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
	uint32_t ctrl = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_EN;
	if (dev->pUartDev->DevIntrf.bIntEn)
	{
		ctrl |= DMA_CCR_TCIE | DMA_CCR_TEIE;
	}
	dev->pTxDma->CCR = ctrl;
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
/**
 * @brief Move RX bytes from circular DMA storage into the UART FIFO.
 *
 * Call at least once per half buffer period while DMA is active.
 * Without interrupts, the caller must poll often enough to avoid a full
 * circular wrap between checks.
 */
static void STM32F03xUARTRxDmaService(STM32F0X_UARTDEV *dev)
{
	if (!dev->DmaOwned || dev->pUartDev == NULL)
	{
		return;
	}

	uint32_t flags = (DMA1->ISR >> dev->RxDmaShift) &
					 (DMA_ISR_HTIF1 | DMA_ISR_TCIF1 | DMA_ISR_TEIF1);
	uint16_t pos = (uint16_t)(STM32F0X_UART_RXDMA_SIZE - dev->pRxDma->CNDTR);

	if (pos >= STM32F0X_UART_RXDMA_SIZE)
	{
		pos = 0U;
	}

	int count = 0;
	while (dev->RxDmaPos != pos)
	{
		uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
		if (p != NULL)
		{
			*p = dev->RxDmaCache[dev->RxDmaPos];
			count++;
		}
		else
		{
			dev->pUartDev->RxDropCnt++;
		}
		dev->RxDmaPos++;
		if (dev->RxDmaPos >= STM32F0X_UART_RXDMA_SIZE)
		{
			dev->RxDmaPos = 0U;
		}
	}

	if (flags != 0U)
	{
		DMA1->IFCR = DMA_IFCR_CGIF1 << dev->RxDmaShift;
	}
	if ((flags & DMA_ISR_TEIF1) != 0U)
	{
		dev->ErrCnt++;
		dev->pReg->CR3 &= ~USART_CR3_DMAR;
		dev->pRxDma->CCR = 0U;
		dev->pRxDma->CNDTR = STM32F0X_UART_RXDMA_SIZE;
		dev->RxDmaPos = 0U;
		dev->pRxDma->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN |
						 (dev->pUartDev->DevIntrf.bIntEn ?
						  (DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE) : 0U);
		dev->pReg->CR3 |= USART_CR3_DMAR;
	}

	dev->pUartDev->bRxReady = CFifoUsed(dev->pUartDev->hRxFifo) != 0;
	if (count > 0 && dev->pUartDev->DevIntrf.bIntEn &&
		dev->pUartDev->EvtCallback != NULL)
	{
		dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA,
								 NULL, CFifoUsed(dev->pUartDev->hRxFifo));
	}
}

static void STM32F03xUARTRxDmaStop(STM32F0X_UARTDEV *dev)
{
	if (!dev->DmaOwned)
	{
		return;
	}
	dev->pReg->CR3 &= ~USART_CR3_DMAR;
	dev->pRxDma->CCR = 0U;
	DMA1->IFCR = DMA_IFCR_CGIF1 << dev->RxDmaShift;
	dev->RxDmaPos = 0U;
}

static void STM32F03xUARTRxDmaStart(STM32F0X_UARTDEV *dev)
{
	dev->pRxDma->CCR = 0U;
	DMA1->IFCR = DMA_IFCR_CGIF1 << dev->RxDmaShift;
	dev->pRxDma->CPAR = (uint32_t)(uintptr_t)&dev->pReg->RDR;
	dev->pRxDma->CMAR = (uint32_t)(uintptr_t)dev->RxDmaCache;
	dev->pRxDma->CNDTR = STM32F0X_UART_RXDMA_SIZE;
	dev->RxDmaPos = 0U;
	dev->pRxDma->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN |
					(dev->pUartDev->DevIntrf.bIntEn ?
					 (DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE) : 0U);
	dev->pReg->CR3 |= USART_CR3_DMAR;
}

static void STM32F03xUARTTxDmaService(STM32F0X_UARTDEV *dev)
{
	if (!dev->DmaOwned || (dev->pTxDma->CCR & DMA_CCR_EN) == 0U)
	{
		return;
	}
	uint32_t flags = (DMA1->ISR >> dev->TxDmaShift) &
					 (DMA_ISR_TCIF1 | DMA_ISR_TEIF1);
	if (flags == 0U)
	{
		return;
	}
	dev->pTxDma->CCR = 0U;
	if ((flags & DMA_ISR_TEIF1) != 0U)
	{
		dev->ErrCnt++;
		dev->pUartDev->TxDropCnt += dev->pTxDma->CNDTR;
	}
	DMA1->IFCR = DMA_IFCR_CGIF1 << dev->TxDmaShift;
	STM32F03xUARTDmaStart(dev);
	if (dev->pUartDev->bTxReady && dev->pUartDev->DevIntrf.bIntEn &&
		dev->pUartDev->EvtCallback != NULL)
	{
		dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, 0);
	}
}

static void STM32F03xUARTDmaIrq(STM32F0X_UARTDEV *dev)
{
	uint32_t state = DisableInterrupt();
	if (dev->DmaOwned)
	{
		STM32F03xUARTRxDmaService(dev);
		STM32F03xUARTTxDmaService(dev);
	}
	EnableInterrupt(state);
}

extern "C" void DMA1_Channel2_3_IRQHandler(void)
{
	STM32F03xUARTDmaIrq(&s_Stm32f03xUartDev[0]);
}

extern "C" void DMA1_Channel4_5_IRQHandler(void)
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
		if ((iflag & USART_ISR_RXNE) && !pDev->DmaOwned)
		{
			(void)pDev->pReg->RDR;
		}
		iflag &= ~USART_ISR_RXNE;
	}

	if ((iflag & USART_ISR_RXNE) && (cr1 & USART_CR1_RXNEIE) && !pDev->DmaOwned)
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

#ifdef STM32F030x8
	if (pDev->DmaOwned && (iflag & USART_ISR_IDLE) &&
		(cr1 & USART_CR1_IDLEIE))
	{
		pDev->pReg->ICR = USART_ICR_IDLECF;
		STM32F03xUARTRxDmaService(pDev);
	}
#endif
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

	if (!pDev->bIntEn && !pDev->bDma)
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
#ifdef STM32F030x8
	if (pDev->bDma)
	{
		STM32F03xUARTRxDmaService(dev);
	}
#endif
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
	dev->pUartDev->bRxReady = CFifoUsed(dev->pUartDev->hRxFifo) != 0;
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
		// A partial return lets the caller retry without an unbounded wait.
		while (cnt < Datalen && (dev->pReg->ISR & USART_ISR_TXE))
		{
			dev->pReg->TDR = pData[cnt++];
		}
		return cnt;
	}

    while (Datalen > 0 && rtry-- > 0)
    {
        uint32_t state = DisableInterrupt();
#ifdef STM32F030x8
		if (pDev->bDma)
		{
			STM32F03xUARTTxDmaService(dev);
		}
#endif
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
#ifdef STM32F030x8
	STM32F03xUARTRxDmaStop(dev);
#endif

	dev->pReg->CR1 &= ~(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
	dev->pReg->CR2 &= ~USART_CR2_RTOEN;

	EnableInterrupt(state);
}

static void STM32F03xUARTEnable(DevIntrf_t * const pDev)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
	uint32_t state = DisableInterrupt();
	STM32F03xUARTDmaStop(dev);
#ifdef STM32F030x8
	STM32F03xUARTRxDmaStop(dev);
#endif

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
	if (dev->DmaOwned)
	{
		dev->pReg->CR3 |= USART_CR3_DMAT;
#ifdef STM32F030x8
		STM32F03xUARTRxDmaStart(dev);
#endif
	}
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
#ifdef STM32F030x8
	STM32F03xUARTRxDmaStop(dev);
#endif
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

	// DMA must be available in both directions. IRQs are optional.
	if ((pCfg->bDMAMode &&
		(s_Stm32f03xUartDev[pCfg->DevNo].pTxDma == NULL ||
		 s_Stm32f03xUartDev[pCfg->DevNo].pRxDma == NULL)) || pCfg->bIrDAMode || pCfg->Mode != UART_MODE_UART ||
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
		(pCfg->bDMAMode && !dev->DmaOwned && (dev->pTxDma->CCR != 0 || dev->pRxDma->CCR != 0)))
	{
		EnableInterrupt(state);
		return false;
	}
	STM32F03xUARTDmaStop(dev);
#ifdef STM32F030x8
	STM32F03xUARTRxDmaStop(dev);
#endif
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
	pDev->DevIntrf.bDma = pCfg->bDMAMode;
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

	if (pCfg->bDMAMode)
	{
		RCC->AHBENR |= RCC_AHBENR_DMAEN;
		(void)RCC->AHBENR;
#ifdef STM32F030x8
		// RM0360: USART1_TX -> channel 2; USART2_TX -> channel 4.
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
		(void)RCC->APB2ENR;
		if (devno == 0) SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1TX_DMA_RMP;
#endif
		dev->DmaOwned = true;
		STM32F03xUARTDmaStop(dev);
		dev->pTxDma->CPAR = (uint32_t)(uintptr_t)&reg->TDR;
		dev->pTxDma->CMAR = (uint32_t)(uintptr_t)dev->TxDmaCache;
		reg->CR3 |= USART_CR3_DMAT;
#ifdef STM32F030x8
		STM32F03xUARTRxDmaStart(dev);
#endif
		if (pCfg->bIntMode)
		{
			NVIC_ClearPendingIRQ(dev->TxDmaIrq);
			NVIC_SetPriority(dev->TxDmaIrq, pCfg->IntPrio);
			NVIC_EnableIRQ(dev->TxDmaIrq);
		}
	}

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
		tmp |= USART_CR1_PEIE;
		if (pCfg->bDMAMode)
		{
			tmp |= USART_CR1_IDLEIE;
		}
		else
		{
			tmp |= USART_CR1_RXNEIE;
		}

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



