/**-------------------------------------------------------------------------
@file	uart_stm32.cpp

@brief	STM32 UART implementation.

		One implementation for the STM32 USART and LPUART with the ISR, ICR,
		TDR and RDR register set, using the ST CMSIS definitions selected by
		stm32.h. The transfer paths, the interrupt handler and the baud rate
		divider are common. What follows from the MCU model is the instance
		table, the USART kernel clock source and the interrupt vectors.

		Families served

		F0 : USART1 to USART6 depending on the part, all on PCLK
		L4 : LPUART1, USART1 to USART3, UART4, UART5, USART and UART on
		     SYSCLK, LPUART1 on the LSE crystal or HSI16

		Not served

		F4  : SR/DR register set
		WBA : instance table and kernel clock source not written yet

		Operating modes

		bIntMode false : polling, TxData and RxData access TDR and RDR
		bIntMode true  : USART interrupts drain the TX CFIFO and fill the
		                 RX CFIFO
		bDMAMode true  : not implemented, UARTInit fails

		DevNo mapping

		F0 : 0 USART1, 1 USART2, 2 USART3, 3 USART4, 4 USART5, 5 USART6
		L4 : 0 LPUART1, 1 USART1, 2 USART2, 3 USART3, 4 UART4, 5 UART5

@author	Hoang Nguyen Hoan
@date	Jun. 7, 2019

@license

MIT License

Copyright (c) 2019, I-SYST inc., all rights reserved

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

#include "stm32.h"

#include "istddef.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"

#if !defined(IOSONATA_STM32_F0) && !defined(IOSONATA_STM32_L4)
#error "uart_stm32.cpp: no USART instance table for this STM32 family"
#endif

// USARTs with a hardware FIFO name the TXE and RXNE bits after their FIFO
// meaning. With FIFOEN cleared, its reset value kept by this driver, they are
// the plain TXE and RXNE flags.
#ifdef USART_ISR_TXE
#define ST_USART_ISR_TXE		USART_ISR_TXE
#define ST_USART_ISR_RXNE		USART_ISR_RXNE
#define ST_USART_CR1_TXEIE		USART_CR1_TXEIE
#define ST_USART_CR1_RXNEIE		USART_CR1_RXNEIE
#else
#define ST_USART_ISR_TXE		USART_ISR_TXE_TXFNF
#define ST_USART_ISR_RXNE		USART_ISR_RXNE_RXFNE
#define ST_USART_CR1_TXEIE		USART_CR1_TXEIE_TXFNFIE
#define ST_USART_CR1_RXNEIE		USART_CR1_RXNEIE_RXFNEIE
#endif

#ifdef USART_ICR_NECF
#define ST_USART_ICR_NECF		USART_ICR_NECF
#else
#define ST_USART_ICR_NECF		USART_ICR_NCF
#endif

// Word length bit for 9 bit words. Parts with a single bit call it M.
#ifdef USART_CR1_M0
#define ST_USART_CR1_M0			USART_CR1_M0
#else
#define ST_USART_CR1_M0			USART_CR1_M
#endif

// Receive errors. The ICR clear bits sit at the same positions as the ISR
// flags, so the flags read from ISR select what is cleared.
#define ST_USART_ISR_RXERR		(USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)
#define ST_USART_ICR_RXERR		(USART_ICR_PECF | USART_ICR_FECF | ST_USART_ICR_NECF | USART_ICR_ORECF)

// Values of the 2 bit RCC kernel clock selection field. PCLK and SYSCLK are
// the same on every family. HSI16 and LSE are the L4 encoding.
#define ST_UART_CLKSEL_MSK		3U
#define ST_UART_CLKSEL_PCLK		0U
#define ST_UART_CLKSEL_SYSCLK	1U
#define ST_UART_CLKSEL_HSI16	2U
#define ST_UART_CLKSEL_LSE		3U

#define ST_UART_HSI16_FREQ		16000000U

// USART divider range. Oversampling by 16 uses BRR = fck / baud, by 8 uses
// USARTDIV = 2 x fck / baud. Both must be at least 16.
#define ST_USART_DIV_MIN		16U
#define ST_USART_DIV_MAX		0xFFFFU

// LPUART BRR = 256 x fck / baud and its valid range.
#define ST_LPUART_BRR_MIN		0x300U
#define ST_LPUART_BRR_MAX		0xFFFFFU

// Default CFIFO memory of each instance, used when the configuration does
// not supply its own.
#define ST_UART_BUFF_SIZE		16
#define ST_UART_CFIFO_SIZE		CFIFO_MEMSIZE(ST_UART_BUFF_SIZE)

#pragma pack(push, 4)

/// Device driver data require by low level functions
typedef struct __Stm32_Uart_Dev {
	int DevNo;						//!< UART interface number
	USART_TypeDef *pReg;			//!< USART or LPUART registers
	IRQn_Type IrqNo;				//!< Interrupt number, shared by several instances on some parts
	volatile uint32_t *pClkEnReg;	//!< RCC peripheral clock enable register
	uint32_t ClkEnMask;				//!< Peripheral clock enable bit
	volatile uint32_t *pRstReg;		//!< RCC peripheral reset register
	uint32_t RstMask;				//!< Peripheral reset bit
	volatile uint32_t *pClkSelReg;	//!< RCC kernel clock selection register, NULL when the instance has none
	uint32_t ClkSelPos;				//!< Position of the kernel clock selection field in pClkSelReg
	UARTDev_t *pUartDev;			//!< Pointer to generic UART dev. data
	const IOPinCfg_t *pIOPinMap;	//!< Pins configured again by Enable
	int NbIOPins;					//!< Number of pins in pIOPinMap
	alignas(4) uint8_t RxFifoMem[ST_UART_CFIFO_SIZE];	//!< Default RX CFIFO memory
	alignas(4) uint8_t TxFifoMem[ST_UART_CFIFO_SIZE];	//!< Default TX CFIFO memory
} Stm32UartDev_t;

#pragma pack(pop)

static Stm32UartDev_t s_Stm32UartDev[] = {
#if defined(IOSONATA_STM32_F0)
	{
		.DevNo = 0,
		.pReg = USART1,
		.IrqNo = USART1_IRQn,
		.pClkEnReg = &RCC->APB2ENR,
		.ClkEnMask = RCC_APB2ENR_USART1EN,
		.pRstReg = &RCC->APB2RSTR,
		.RstMask = RCC_APB2RSTR_USART1RST,
		.pClkSelReg = &RCC->CFGR3,
		.ClkSelPos = RCC_CFGR3_USART1SW_Pos,
	},
#ifdef USART2
	{
		.DevNo = 1,
		.pReg = USART2,
		.IrqNo = USART2_IRQn,
		.pClkEnReg = &RCC->APB1ENR,
		.ClkEnMask = RCC_APB1ENR_USART2EN,
		.pRstReg = &RCC->APB1RSTR,
		.RstMask = RCC_APB1RSTR_USART2RST,
		.pClkSelReg = NULL,
		.ClkSelPos = 0,
	},
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
	{
		.DevNo = 2,
		.pReg = USART3,
#ifdef STM32F030xC
		.IrqNo = USART3_6_IRQn,
#else
		.IrqNo = USART3_4_IRQn,
#endif
		.pClkEnReg = &RCC->APB1ENR,
		.ClkEnMask = RCC_APB1ENR_USART3EN,
		.pRstReg = &RCC->APB1RSTR,
		.RstMask = RCC_APB1RSTR_USART3RST,
		.pClkSelReg = NULL,
		.ClkSelPos = 0,
	},
	{
		.DevNo = 3,
		.pReg = USART4,
#ifdef STM32F030xC
		.IrqNo = USART3_6_IRQn,
#else
		.IrqNo = USART3_4_IRQn,
#endif
		.pClkEnReg = &RCC->APB1ENR,
		.ClkEnMask = RCC_APB1ENR_USART4EN,
		.pRstReg = &RCC->APB1RSTR,
		.RstMask = RCC_APB1RSTR_USART4RST,
		.pClkSelReg = NULL,
		.ClkSelPos = 0,
	},
#endif
#ifdef STM32F030xC
	{
		.DevNo = 4,
		.pReg = USART5,
		.IrqNo = USART3_6_IRQn,
		.pClkEnReg = &RCC->APB1ENR,
		.ClkEnMask = RCC_APB1ENR_USART5EN,
		.pRstReg = &RCC->APB1RSTR,
		.RstMask = RCC_APB1RSTR_USART5RST,
		.pClkSelReg = NULL,
		.ClkSelPos = 0,
	},
	{
		.DevNo = 5,
		.pReg = USART6,
		.IrqNo = USART3_6_IRQn,
		.pClkEnReg = &RCC->APB2ENR,
		.ClkEnMask = RCC_APB2ENR_USART6EN,
		.pRstReg = &RCC->APB2RSTR,
		.RstMask = RCC_APB2RSTR_USART6RST,
		.pClkSelReg = NULL,
		.ClkSelPos = 0,
	},
#endif
#elif defined(IOSONATA_STM32_L4)
	{
		.DevNo = 0,
		.pReg = LPUART1,
		.IrqNo = LPUART1_IRQn,
		.pClkEnReg = &RCC->APB1ENR2,
		.ClkEnMask = RCC_APB1ENR2_LPUART1EN,
		.pRstReg = &RCC->APB1RSTR2,
		.RstMask = RCC_APB1RSTR2_LPUART1RST,
		.pClkSelReg = &RCC->CCIPR,
		.ClkSelPos = RCC_CCIPR_LPUART1SEL_Pos,
	},
	{
		.DevNo = 1,
		.pReg = USART1,
		.IrqNo = USART1_IRQn,
		.pClkEnReg = &RCC->APB2ENR,
		.ClkEnMask = RCC_APB2ENR_USART1EN,
		.pRstReg = &RCC->APB2RSTR,
		.RstMask = RCC_APB2RSTR_USART1RST,
		.pClkSelReg = &RCC->CCIPR,
		.ClkSelPos = RCC_CCIPR_USART1SEL_Pos,
	},
	{
		.DevNo = 2,
		.pReg = USART2,
		.IrqNo = USART2_IRQn,
		.pClkEnReg = &RCC->APB1ENR1,
		.ClkEnMask = RCC_APB1ENR1_USART2EN,
		.pRstReg = &RCC->APB1RSTR1,
		.RstMask = RCC_APB1RSTR1_USART2RST,
		.pClkSelReg = &RCC->CCIPR,
		.ClkSelPos = RCC_CCIPR_USART2SEL_Pos,
	},
	{
		.DevNo = 3,
		.pReg = USART3,
		.IrqNo = USART3_IRQn,
		.pClkEnReg = &RCC->APB1ENR1,
		.ClkEnMask = RCC_APB1ENR1_USART3EN,
		.pRstReg = &RCC->APB1RSTR1,
		.RstMask = RCC_APB1RSTR1_USART3RST,
		.pClkSelReg = &RCC->CCIPR,
		.ClkSelPos = RCC_CCIPR_USART3SEL_Pos,
	},
	{
		.DevNo = 4,
		.pReg = UART4,
		.IrqNo = UART4_IRQn,
		.pClkEnReg = &RCC->APB1ENR1,
		.ClkEnMask = RCC_APB1ENR1_UART4EN,
		.pRstReg = &RCC->APB1RSTR1,
		.RstMask = RCC_APB1RSTR1_UART4RST,
		.pClkSelReg = &RCC->CCIPR,
		.ClkSelPos = RCC_CCIPR_UART4SEL_Pos,
	},
	{
		.DevNo = 5,
		.pReg = UART5,
		.IrqNo = UART5_IRQn,
		.pClkEnReg = &RCC->APB1ENR1,
		.ClkEnMask = RCC_APB1ENR1_UART5EN,
		.pRstReg = &RCC->APB1RSTR1,
		.RstMask = RCC_APB1RSTR1_UART5RST,
		.pClkSelReg = &RCC->CCIPR,
		.ClkSelPos = RCC_CCIPR_UART5SEL_Pos,
	},
#endif
};

static const int s_NbUartDev = sizeof(s_Stm32UartDev) / sizeof(Stm32UartDev_t);

static void Stm32UartIrqHandler(Stm32UartDev_t * const pDev);
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
static uint32_t Stm32UartClkSrc(Stm32UartDev_t * const pDev, uint32_t Rate, uint32_t * const pFreq);

static void Stm32UartIrqHandler(Stm32UartDev_t * const pDev)
{
	UARTDev_t *dev = pDev->pUartDev;

	if (dev == NULL || !dev->DevIntrf.bIntEn)
	{
		return;
	}

	USART_TypeDef *reg = pDev->pReg;
	uint32_t iflag = reg->ISR;
	uint32_t cr1 = reg->CR1;

	// RX errors: clear only the error flags seen, then drain RDR to release
	// RXNE and let the line recover.
	if (iflag & ST_USART_ISR_RXERR)
	{
		if (iflag & USART_ISR_ORE)
		{
			dev->RxOvrErrCnt++;
		}
		if (iflag & USART_ISR_PE)
		{
			dev->ParErrCnt++;
		}
		if (iflag & USART_ISR_FE)
		{
			dev->FramErrCnt++;
		}
		reg->ICR = iflag & ST_USART_ICR_RXERR;
		if (iflag & ST_USART_ISR_RXNE)
		{
			(void)reg->RDR;
		}
		iflag &= ~ST_USART_ISR_RXNE;
	}

	if ((iflag & ST_USART_ISR_RXNE) && (cr1 & ST_USART_CR1_RXNEIE))
	{
		// Reading RDR clears RXNE
		uint8_t c = (uint8_t)reg->RDR & (dev->DataBits == 7 ? 0x7F : 0xFF);
		uint8_t *p = CFifoPut(dev->hRxFifo);
		if (p != NULL)
		{
			*p = c;
			dev->bRxReady = true;
		}
		else
		{
			dev->RxDropCnt++;
		}
		if (dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_RXDATA, NULL, CFifoUsed(dev->hRxFifo));
		}
	}

	if ((iflag & ST_USART_ISR_TXE) && (cr1 & ST_USART_CR1_TXEIE))
	{
		uint8_t *p = CFifoGet(dev->hTxFifo);
		if (p != NULL)
		{
			// Writing TDR clears TXE
			reg->TDR = *p;
		}
		else
		{
			// FIFO empty, mask TXEIE so TXE does not keep interrupting.
			reg->CR1 &= ~ST_USART_CR1_TXEIE;
			dev->bTxReady = true;
			if (dev->EvtCallback)
			{
				dev->EvtCallback(dev, UART_EVT_TXREADY, NULL, 0);
			}
		}
	}

	// TXE and RXNE have no ICR clear bit. They are cleared by the TDR write
	// and the RDR read above.
}

static uint32_t Stm32UartGetRate(DevIntrf_t * const pDev)
{
	return ((Stm32UartDev_t *)pDev->pDevData)->pUartDev->Rate;
}

// Select the closest rate the instance can make and return it.
static uint32_t Stm32UartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	USART_TypeDef *reg = dev->pReg;

	if (Rate == 0)
	{
		return 0;
	}

	// The kernel clock source, OVER8 and BRR are changed with UE cleared.
	// The interrupt handler writes CR1 too, so the sequence is not
	// interrupted.
	uint32_t state = DisableInterrupt();
	uint32_t cr1 = reg->CR1;
	reg->CR1 = cr1 & ~USART_CR1_UE;

	uint32_t fclk = 0;
	uint32_t clksel = Stm32UartClkSrc(dev, Rate, &fclk);

	if (fclk == 0)
	{
		reg->CR1 = cr1;
		EnableInterrupt(state);

		return 0;
	}

	if (dev->pClkSelReg != NULL)
	{
		*dev->pClkSelReg = (*dev->pClkSelReg & ~(ST_UART_CLKSEL_MSK << dev->ClkSelPos)) |
						   (clksel << dev->ClkSelPos);
	}

	uint32_t brr = 0;
	uint32_t rate = 0;

	cr1 &= ~USART_CR1_OVER8;

#ifdef LPUART1
	if (reg == LPUART1)
	{
		uint64_t f = (uint64_t)fclk << 8;
		uint64_t div = (f + (Rate >> 1)) / Rate;

		if (div < ST_LPUART_BRR_MIN)
		{
			div = ST_LPUART_BRR_MIN;
		}
		else if (div > ST_LPUART_BRR_MAX)
		{
			div = ST_LPUART_BRR_MAX;
		}
		brr = (uint32_t)div;
		rate = (uint32_t)((f + (div >> 1)) / div);
	}
	else
#endif
	{
		uint32_t div16 = (fclk + (Rate >> 1)) / Rate;
		uint32_t div8 = ((fclk << 1) + (Rate >> 1)) / Rate;

		if (div16 < ST_USART_DIV_MIN)
		{
			div16 = ST_USART_DIV_MIN;
		}
		else if (div16 > ST_USART_DIV_MAX)
		{
			div16 = ST_USART_DIV_MAX;
		}
		if (div8 < ST_USART_DIV_MIN)
		{
			div8 = ST_USART_DIV_MIN;
		}
		else if (div8 > ST_USART_DIV_MAX)
		{
			div8 = ST_USART_DIV_MAX;
		}

		uint32_t rate16 = (fclk + (div16 >> 1)) / div16;
		uint32_t rate8 = ((fclk << 1) + (div8 >> 1)) / div8;
		uint32_t err16 = rate16 > Rate ? rate16 - Rate : Rate - rate16;
		uint32_t err8 = rate8 > Rate ? rate8 - Rate : Rate - rate8;

		// Oversampling by 16 has the better noise margin and is kept unless
		// oversampling by 8 is closer, which is the case above fck / 16.
		if (err8 < err16)
		{
			cr1 |= USART_CR1_OVER8;
			brr = (div8 & 0xFFF0U) | ((div8 & 0xFU) >> 1);
			rate = rate8;
		}
		else
		{
			brr = div16;
			rate = rate16;
		}
	}

	reg->CR1 = cr1 & ~USART_CR1_UE;
	reg->BRR = brr;
	reg->CR1 = cr1;

	EnableInterrupt(state);

	dev->pUartDev->Rate = rate;

	return rate;
}

static bool Stm32UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int Stm32UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	USART_TypeDef *reg = dev->pReg;
	int cnt = 0;

	if (!pDev->bIntEn)
	{
		while (cnt < Bufflen)
		{
			uint32_t flags = reg->ISR;

			if (flags & ST_USART_ISR_RXERR)
			{
				if (flags & USART_ISR_ORE)
				{
					dev->pUartDev->RxOvrErrCnt++;
				}
				if (flags & USART_ISR_PE)
				{
					dev->pUartDev->ParErrCnt++;
				}
				if (flags & USART_ISR_FE)
				{
					dev->pUartDev->FramErrCnt++;
				}
				reg->ICR = flags & ST_USART_ICR_RXERR;
				if (flags & ST_USART_ISR_RXNE)
				{
					(void)reg->RDR;
				}
				break;
			}
			if ((flags & ST_USART_ISR_RXNE) == 0)
			{
				break;
			}
			pBuff[cnt++] = (uint8_t)reg->RDR & (dev->pUartDev->DataBits == 7 ? 0x7F : 0xFF);
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

static void Stm32UartStopRx(DevIntrf_t * const pDev)
{
}

static bool Stm32UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int Stm32UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	USART_TypeDef *reg = dev->pReg;
	int cnt = 0;

	if (pData == NULL || Datalen <= 0 || (reg->CR1 & USART_CR1_UE) == 0)
	{
		return 0;
	}

	if (!pDev->bIntEn)
	{
		// TDR holds the next frame while the shift register sends the
		// current one. TXE set means TDR is free. Write only while it is
		// free and return the count, the caller retries the rest.
		while (cnt < Datalen && (reg->ISR & ST_USART_ISR_TXE))
		{
			reg->TDR = pData[cnt++];
		}

		return cnt;
	}

	int rtry = pDev->MaxRetry;

	while (Datalen > 0 && rtry-- > 0)
	{
		uint32_t state = DisableInterrupt();

		if ((reg->CR1 & USART_CR1_UE) == 0)
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

		// Start TX inside the critical section so the interrupt cannot fetch
		// the same byte again and overwrite TDR.
		if (dev->pUartDev->bTxReady && (reg->ISR & ST_USART_ISR_TXE))
		{
			uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
			if (p != NULL)
			{
				dev->pUartDev->bTxReady = false;
				reg->TDR = *p;
				reg->CR1 |= ST_USART_CR1_TXEIE;
			}
		}

		if (CFifoUsed(dev->pUartDev->hTxFifo) > 0)
		{
			dev->pUartDev->bTxReady = false;
			reg->CR1 |= ST_USART_CR1_TXEIE;
		}

		EnableInterrupt(state);
	}

	// Datalen is what the FIFO did not take after all retries.
	if (Datalen > 0)
	{
		dev->pUartDev->TxDropCnt += (uint32_t)Datalen;
	}

	return cnt;
}

static void Stm32UartStopTx(DevIntrf_t * const pDev)
{
}

static void Stm32UartDisable(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->CR1 &= ~(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);

	// Registers keep their content while the peripheral clock is off.
	*dev->pClkEnReg &= ~dev->ClkEnMask;

	IOPinDis(dev->pIOPinMap, dev->NbIOPins);

	EnableInterrupt(state);
}

static void Stm32UartEnable(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pUartDev->RxOvrErrCnt = 0;
	dev->pUartDev->ParErrCnt = 0;
	dev->pUartDev->FramErrCnt = 0;
	dev->pUartDev->RxDropCnt = 0;
	dev->pUartDev->TxDropCnt = 0;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;

	*dev->pClkEnReg |= dev->ClkEnMask;
	(void)*dev->pClkEnReg;

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

	dev->pReg->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	EnableInterrupt(state);
}

static void Stm32UartPowerOff(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;

	Stm32UartDisable(pDev);

	// Return the kernel clock selection to its reset value, PCLK.
	if (dev->pClkSelReg != NULL)
	{
		*dev->pClkSelReg &= ~(ST_UART_CLKSEL_MSK << dev->ClkSelPos);
	}
}

static void Stm32UartReset(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	*dev->pRstReg |= dev->RstMask;
	(void)*dev->pRstReg;
	*dev->pRstReg &= ~dev->RstMask;

	EnableInterrupt(state);
}

static void *Stm32UartGetHandle(DevIntrf_t * const pDev)
{
	return ((Stm32UartDev_t *)pDev->pDevData)->pUartDev;
}

#if defined(IOSONATA_STM32_F0)

// Every F0 instance runs from PCLK. USART1 also has its own selection field,
// set back to PCLK through the instance table.
static uint32_t Stm32UartClkSrc(Stm32UartDev_t * const pDev, uint32_t Rate, uint32_t * const pFreq)
{
	*pFreq = SystemPeriphClockGet(0);

	return ST_UART_CLKSEL_PCLK;
}

#elif defined(IOSONATA_STM32_L4)

// USART and UART instances run from SYSCLK. LPUART1 needs fck between 3 and
// 4096 times the rate, so it runs from the LSE crystal up to LSE / 3, from
// HSI16 up to 16 MHz / 3 and from SYSCLK above that.
static uint32_t Stm32UartClkSrc(Stm32UartDev_t * const pDev, uint32_t Rate, uint32_t * const pFreq)
{
	if (pDev->pReg == LPUART1)
	{
		if (GetLowFreqOscType() == OSC_TYPE_XTAL && Rate <= GetLowFreqOscFreq() / 3U)
		{
			*pFreq = GetLowFreqOscFreq();

			return ST_UART_CLKSEL_LSE;
		}

		if (Rate <= ST_UART_HSI16_FREQ / 3U)
		{
			RCC->CR |= RCC_CR_HSION;
			while ((RCC->CR & RCC_CR_HSIRDY) == 0);

			*pFreq = ST_UART_HSI16_FREQ;

			return ST_UART_CLKSEL_HSI16;
		}
	}

	*pFreq = SystemCoreClockGet();

	return ST_UART_CLKSEL_SYSCLK;
}

#endif

UARTDev_t const *UARTGetInstance(int DevNo)
{
	return DevNo >= 0 && DevNo < s_NbUartDev ? s_Stm32UartDev[DevNo].pUartDev : NULL;
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

	// Polling and USART interrupt operation. DMA is not implemented, so a
	// DMA request fails here instead of running in another mode.
	if (pCfg->bDMAMode || pCfg->bIrDAMode || pCfg->Mode != UART_MODE_UART ||
		(pCfg->FlowControl != UART_FLWCTRL_NONE && pCfg->FlowControl != UART_FLWCTRL_HW) ||
		(pCfg->Parity != UART_PARITY_NONE && pCfg->Parity != UART_PARITY_EVEN && pCfg->Parity != UART_PARITY_ODD) ||
		(pCfg->DataBits != 7 && pCfg->DataBits != 8) ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) || pCfg->Rate <= 0)
	{
		return false;
	}

	// The hardware word length includes the parity bit.
	int wordbits = pCfg->DataBits + (pCfg->Parity != UART_PARITY_NONE ? 1 : 0);

#ifndef USART_CR1_M1
	if (wordbits == 7)
	{
		return false;
	}
#endif

	Stm32UartDev_t *dev = &s_Stm32UartDev[pCfg->DevNo];
	USART_TypeDef *reg = dev->pReg;
	uint32_t state = DisableInterrupt();

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;
	dev->pIOPinMap = (const IOPinCfg_t *)pCfg->pIOPinMap;
	dev->NbIOPins = pCfg->NbIOPins;

	// Peripheral clock first. Register writes before it are ignored.
	*dev->pClkEnReg |= dev->ClkEnMask;
	(void)*dev->pClkEnReg;

	// All USART registers at their reset value, UE cleared.
	Stm32UartReset(&pDev->DevIntrf);

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(dev->RxFifoMem, ST_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(dev->TxFifoMem, ST_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	// CFifoInit returns NULL when the supplied memory is too small.
	if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
	{
		*dev->pClkEnReg &= ~dev->ClkEnMask;
		EnableInterrupt(state);

		return false;
	}

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

	pDev->Rate = (int)Stm32UartSetRate(&pDev->DevIntrf, (uint32_t)pCfg->Rate);
	if (pDev->Rate == 0)
	{
		*dev->pClkEnReg &= ~dev->ClkEnMask;
		EnableInterrupt(state);

		return false;
	}

	// Keep the OVER8 selection made by Stm32UartSetRate.
	uint32_t cr1 = reg->CR1 & USART_CR1_OVER8;
	uint32_t cr3 = 0;

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
		cr1 |= ST_USART_CR1_M0;
	}
#ifdef USART_CR1_M1
	else if (wordbits == 7)
	{
		cr1 |= USART_CR1_M1;
	}
#endif

	reg->CR2 = pCfg->StopBits == 2 ? USART_CR2_STOP_1 : 0;

	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		cr3 |= USART_CR3_CTSE | USART_CR3_RTSE;
	}
	if (pCfg->Duplex == UART_DUPLEX_HALF)
	{
		cr3 |= USART_CR3_HDSEL;
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

	if (pCfg->bIntMode)
	{
		// Only the interrupts the handler services. TXEIE is set by
		// Stm32UartTxData when the TX FIFO has data. TCIE, IDLEIE and RTOIE
		// stay off, their flags are not cleared by the handler.
		cr1 |= ST_USART_CR1_RXNEIE | USART_CR1_PEIE;

		// FE, NE and ORE
		cr3 |= USART_CR3_EIE;

		NVIC_ClearPendingIRQ(dev->IrqNo);
		NVIC_SetPriority(dev->IrqNo, pCfg->IntPrio);
		NVIC_EnableIRQ(dev->IrqNo);
	}

	reg->CR3 = cr3;
	reg->CR1 = cr1 | USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	EnableInterrupt(state);

	return true;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
}

#if defined(IOSONATA_STM32_F0)

extern "C" void USART1_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[0]);
}

#ifdef USART2
extern "C" void USART2_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[1]);
}
#endif

#if defined(STM32F070xB) || defined(STM32F030xC)
// USART3 and up share one vector.
#ifdef STM32F030xC
extern "C" void USART3_6_IRQHandler()
#else
extern "C" void USART3_4_IRQHandler()
#endif
{
	for (int i = 2; i < s_NbUartDev; i++)
	{
		Stm32UartIrqHandler(&s_Stm32UartDev[i]);
	}
}
#endif

#elif defined(IOSONATA_STM32_L4)

extern "C" void LPUART1_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[0]);
}

extern "C" void USART1_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[1]);
}

extern "C" void USART2_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[2]);
}

extern "C" void USART3_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[3]);
}

extern "C" void UART4_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[4]);
}

extern "C" void UART5_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[5]);
}

#endif
