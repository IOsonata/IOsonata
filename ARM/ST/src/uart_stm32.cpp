/**-------------------------------------------------------------------------
@file	uart_stm32.cpp

@brief	STM32 UART implementation.

		One driver for the STM32 USART and LPUART. The engine is the same on
		every family. What differs is a few register and bit names, mapped
		below from the CMSIS device header selected by stm32.h:

		- SR and DR on F1, F2 and F4 in place of ISR, ICR, TDR and RDR
		- the FIFO names of the TXE and RXNE flags
		- the RCC register holding the APB1 clock enables
		- the LPUART clock registers
		- the shared USART3 and up vector of the F0 parts

		The instance table and the interrupt vectors list every instance the
		device header defines. DevNo is the position in that list:

		LPUART1, USART1, USART2, USART3, USART4 or UART4, USART5 or UART5,
		USART6

		F030x8 : 0 USART1, 1 USART2
		F401   : 0 USART1, 1 USART2, 2 USART6
		L4     : 0 LPUART1, 1 USART1, 2 USART2, 3 USART3, 4 UART4, 5 UART5

		USART kernel clock is PCLK, the reset value of its selection field,
		read with SystemPeriphClockGet. LPUART1 runs from the LSE crystal or
		HSI16 for the rates they can make.

		bIntMode false : polling, TxData and RxData access the data register
		bIntMode true  : USART interrupts drain the TX CFIFO and fill the
		                 RX CFIFO
		bDMAMode true  : not implemented, UARTInit fails

		Not mapped yet: the RCC names of WBA.

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

// Status, data and receive error clear. Most families have ISR, ICR, TDR and
// RDR. F1, F2 and F4 have SR and one DR, and clear the receive errors when DR
// is read after SR, which the receive paths do since an error comes with
// RXNE. The flags sit at the same bit positions in ISR and SR.
#if defined(USART_ISR_TXE) || defined(USART_ISR_TXE_TXFNF)
#define ST_USART_ISR(Reg)				((Reg)->ISR)
#define ST_USART_RDR(Reg)				((Reg)->RDR)
#define ST_USART_TDR(Reg)				((Reg)->TDR)
#define ST_USART_RXERR_CLEAR(Reg, Flags)	((Reg)->ICR = (Flags) & ST_USART_ICR_RXERR)
#define ST_USART_ISR_PE					USART_ISR_PE
#define ST_USART_ISR_FE					USART_ISR_FE
#define ST_USART_ISR_NE					USART_ISR_NE
#define ST_USART_ISR_ORE				USART_ISR_ORE
#ifdef USART_ISR_TXE
#define ST_USART_ISR_TXE				USART_ISR_TXE
#define ST_USART_ISR_TC				USART_ISR_TC
#define ST_USART_ISR_RXNE				USART_ISR_RXNE
#define ST_USART_CR1_TXEIE				USART_CR1_TXEIE
#define ST_USART_CR1_RXNEIE				USART_CR1_RXNEIE
#else
// Hardware FIFO names. With FIFOEN at its reset value, cleared, they are the
// plain TXE and RXNE flags.
#define ST_USART_ISR_TXE				USART_ISR_TXE_TXFNF
#define ST_USART_ISR_TC				USART_ISR_TC
#define ST_USART_ISR_RXNE				USART_ISR_RXNE_RXFNE
#define ST_USART_CR1_TXEIE				USART_CR1_TXEIE_TXFNFIE
#define ST_USART_CR1_RXNEIE				USART_CR1_RXNEIE_RXFNEIE
#endif
#ifdef USART_ICR_NECF
#define ST_USART_ICR_NECF				USART_ICR_NECF
#else
#define ST_USART_ICR_NECF				USART_ICR_NCF
#endif
// The ICR clear bits sit at the same positions as the ISR flags.
#define ST_USART_ICR_RXERR				(USART_ICR_PECF | USART_ICR_FECF | ST_USART_ICR_NECF | USART_ICR_ORECF)
#elif defined(USART_SR_TXE)
#define ST_USART_ISR(Reg)				((Reg)->SR)
#define ST_USART_RDR(Reg)				((Reg)->DR)
#define ST_USART_TDR(Reg)				((Reg)->DR)
#define ST_USART_RXERR_CLEAR(Reg, Flags)
#define ST_USART_ISR_PE					USART_SR_PE
#define ST_USART_ISR_FE					USART_SR_FE
#define ST_USART_ISR_NE					USART_SR_NE
#define ST_USART_ISR_ORE				USART_SR_ORE
#define ST_USART_ISR_TXE				USART_SR_TXE
#define ST_USART_ISR_TC				USART_SR_TC
#define ST_USART_ISR_RXNE				USART_SR_RXNE
#define ST_USART_CR1_TXEIE				USART_CR1_TXEIE
#define ST_USART_CR1_RXNEIE				USART_CR1_RXNEIE
#else
#error "uart_stm32.cpp: USART register names not mapped"
#endif

#define ST_USART_ISR_RXERR		(ST_USART_ISR_PE | ST_USART_ISR_FE | ST_USART_ISR_NE | ST_USART_ISR_ORE)

// 9 bit word. Parts with a single word length bit call it M.
#ifdef USART_CR1_M0
#define ST_USART_CR1_M0			USART_CR1_M0
#else
#define ST_USART_CR1_M0			USART_CR1_M
#endif

// RCC register holding the APB1 clock enables, split in two on L4. The reset
// bit of a peripheral has the same position as its enable bit.
#if defined(RCC_APB1ENR1_PWREN)
#define ST_RCC_APB1ENR			APB1ENR1
#define ST_RCC_APB1RSTR			APB1RSTR1
#define ST_RCC_APB1EN(Inst)		RCC_APB1ENR1_##Inst##EN
#elif defined(RCC_APB1ENR_PWREN)
#define ST_RCC_APB1ENR			APB1ENR
#define ST_RCC_APB1RSTR			APB1RSTR
#define ST_RCC_APB1EN(Inst)		RCC_APB1ENR_##Inst##EN
#else
#error "uart_stm32.cpp: RCC APB1 register names not mapped"
#endif

// SystemPeriphClockGet index of the APB2 instances. Parts with a single APB
// prescaler have one PCLK.
#ifdef RCC_CFGR_PPRE2
#define ST_PCLK_APB2			1
#else
#define ST_PCLK_APB2			0
#endif

// LPUART1 clock enable and kernel clock selection
#if defined(LPUART1) && defined(RCC_APB1ENR2_LPUART1EN)
#define ST_RCC_LPUART1ENR		APB1ENR2
#define ST_RCC_LPUART1RSTR		APB1RSTR2
#define ST_RCC_LPUART1EN		RCC_APB1ENR2_LPUART1EN
#define ST_RCC_LPUART1SELR		CCIPR
#define ST_RCC_LPUART1SEL_Pos	RCC_CCIPR_LPUART1SEL_Pos
#elif defined(LPUART1)
#error "uart_stm32.cpp: LPUART1 RCC names not mapped"
#endif

// F0 parts route USART3 and up to one vector. The device header names it
// USART3_4 on every part that has it.
#if defined(USART3_4_IRQn) || defined(USART3_6_IRQn)
#define ST_USART3_IRQn			USART3_4_IRQn
#define ST_USART6_IRQn			USART3_4_IRQn
#else
#define ST_USART3_IRQn			USART3_IRQn
#define ST_USART6_IRQn			USART6_IRQn
#endif

// LPUART kernel clock selection values and the HSI16 frequency
#define ST_LPUART_CLKSEL_MSK	3U
#define ST_LPUART_CLKSEL_PCLK	0U
#define ST_LPUART_CLKSEL_HSI16	2U
#define ST_LPUART_CLKSEL_LSE	3U
#define ST_HSI16_FREQ			16000000U

// USART divider range. Oversampling by 16 uses BRR = fck / baud, by 8 uses
// USARTDIV = 2 x fck / baud. Both must be at least 16.
#define ST_USART_DIV_MIN		16U
#define ST_USART_DIV_MAX		0xFFFFU

// LPUART BRR = 256 x fck / baud and its valid range
#define ST_LPUART_BRR_MIN		0x300U
#define ST_LPUART_BRR_MAX		0xFFFFFU

// Default CFIFO memory of each instance, used when the configuration does
// not supply its own
#define ST_UART_BUFF_SIZE		16
#define ST_UART_CFIFO_SIZE		CFIFO_MEMSIZE(ST_UART_BUFF_SIZE)

#pragma pack(push, 4)

/// Device driver data require by low level functions
typedef struct __Stm32_Uart_Dev {
	USART_TypeDef *pReg;			//!< USART or LPUART registers
	IRQn_Type IrqNo;				//!< Interrupt number
	volatile uint32_t *pRccEnReg;	//!< RCC clock enable register
	volatile uint32_t *pRccRstReg;	//!< RCC reset register
	uint32_t RccMask;				//!< Enable bit, also the reset bit
	int PclkIdx;					//!< SystemPeriphClockGet index of the instance bus
	UARTDev_t *pUartDev;			//!< Pointer to generic UART dev. data
	const IOPinCfg_t *pIOPinMap;	//!< Pins configured again by Enable
	int NbIOPins;					//!< Number of pins in pIOPinMap
	bool PollTxStarted;				//!< Polling frame already started
	alignas(4) uint8_t RxFifoMem[ST_UART_CFIFO_SIZE];	//!< Default RX CFIFO memory
	alignas(4) uint8_t TxFifoMem[ST_UART_CFIFO_SIZE];	//!< Default TX CFIFO memory
} Stm32UartDev_t;

#pragma pack(pop)

static Stm32UartDev_t s_Stm32UartDev[] = {
#ifdef LPUART1
	{
		.pReg = LPUART1,
		.IrqNo = LPUART1_IRQn,
		.pRccEnReg = &RCC->ST_RCC_LPUART1ENR,
		.pRccRstReg = &RCC->ST_RCC_LPUART1RSTR,
		.RccMask = ST_RCC_LPUART1EN,
		.PclkIdx = 0,
	},
#endif
	{
		.pReg = USART1,
		.IrqNo = USART1_IRQn,
		.pRccEnReg = &RCC->APB2ENR,
		.pRccRstReg = &RCC->APB2RSTR,
		.RccMask = RCC_APB2ENR_USART1EN,
		.PclkIdx = ST_PCLK_APB2,
	},
#ifdef USART2
	{
		.pReg = USART2,
		.IrqNo = USART2_IRQn,
		.pRccEnReg = &RCC->ST_RCC_APB1ENR,
		.pRccRstReg = &RCC->ST_RCC_APB1RSTR,
		.RccMask = ST_RCC_APB1EN(USART2),
		.PclkIdx = 0,
	},
#endif
#ifdef USART3
	{
		.pReg = USART3,
		.IrqNo = ST_USART3_IRQn,
		.pRccEnReg = &RCC->ST_RCC_APB1ENR,
		.pRccRstReg = &RCC->ST_RCC_APB1RSTR,
		.RccMask = ST_RCC_APB1EN(USART3),
		.PclkIdx = 0,
	},
#endif
#ifdef USART4
	{
		.pReg = USART4,
		.IrqNo = USART3_4_IRQn,
		.pRccEnReg = &RCC->ST_RCC_APB1ENR,
		.pRccRstReg = &RCC->ST_RCC_APB1RSTR,
		.RccMask = ST_RCC_APB1EN(USART4),
		.PclkIdx = 0,
	},
#endif
#ifdef UART4
	{
		.pReg = UART4,
		.IrqNo = UART4_IRQn,
		.pRccEnReg = &RCC->ST_RCC_APB1ENR,
		.pRccRstReg = &RCC->ST_RCC_APB1RSTR,
		.RccMask = ST_RCC_APB1EN(UART4),
		.PclkIdx = 0,
	},
#endif
#ifdef USART5
	{
		.pReg = USART5,
		.IrqNo = USART3_4_IRQn,
		.pRccEnReg = &RCC->ST_RCC_APB1ENR,
		.pRccRstReg = &RCC->ST_RCC_APB1RSTR,
		.RccMask = ST_RCC_APB1EN(USART5),
		.PclkIdx = 0,
	},
#endif
#ifdef UART5
	{
		.pReg = UART5,
		.IrqNo = UART5_IRQn,
		.pRccEnReg = &RCC->ST_RCC_APB1ENR,
		.pRccRstReg = &RCC->ST_RCC_APB1RSTR,
		.RccMask = ST_RCC_APB1EN(UART5),
		.PclkIdx = 0,
	},
#endif
#ifdef USART6
	{
		.pReg = USART6,
		.IrqNo = ST_USART6_IRQn,
		.pRccEnReg = &RCC->APB2ENR,
		.pRccRstReg = &RCC->APB2RSTR,
		.RccMask = RCC_APB2ENR_USART6EN,
		.PclkIdx = ST_PCLK_APB2,
	},
#endif
};

static const int s_NbUartDev = sizeof(s_Stm32UartDev) / sizeof(Stm32UartDev_t);

static void Stm32UartIrqHandler(USART_TypeDef * const pReg);
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
static void Stm32UartReset(DevIntrf_t * const pDev);
static void *Stm32UartGetHandle(DevIntrf_t * const pDev);

static void Stm32UartIrqHandler(USART_TypeDef * const pReg)
{
	UARTDev_t *dev = NULL;

	for (int i = 0; i < s_NbUartDev; i++)
	{
		if (s_Stm32UartDev[i].pReg == pReg)
		{
			dev = s_Stm32UartDev[i].pUartDev;
			break;
		}
	}

	if (dev == NULL || !dev->DevIntrf.bIntEn)
	{
		return;
	}

	uint32_t iflag = ST_USART_ISR(pReg);
	uint32_t cr1 = pReg->CR1;

	// RX errors: clear only the error flags seen, then drain RDR to release
	// RXNE and let the line recover.
	if (iflag & ST_USART_ISR_RXERR)
	{
		if (iflag & ST_USART_ISR_ORE)
		{
			dev->RxOvrErrCnt++;
		}
		if (iflag & ST_USART_ISR_PE)
		{
			dev->ParErrCnt++;
		}
		if (iflag & ST_USART_ISR_FE)
		{
			dev->FramErrCnt++;
		}
		ST_USART_RXERR_CLEAR(pReg, iflag);
		if (iflag & ST_USART_ISR_RXNE)
		{
			(void)ST_USART_RDR(pReg);
		}
		iflag &= ~ST_USART_ISR_RXNE;
	}

	if ((iflag & ST_USART_ISR_RXNE) && (cr1 & ST_USART_CR1_RXNEIE))
	{
		// Reading RDR clears RXNE
		uint8_t c = (uint8_t)ST_USART_RDR(pReg) & (dev->DataBits == 7 ? 0x7F : 0xFF);
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
			ST_USART_TDR(pReg) = *p;
		}
		else
		{
			// FIFO empty, mask TXEIE so TXE does not keep interrupting.
			pReg->CR1 &= ~ST_USART_CR1_TXEIE;
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
	uint32_t fclk = SystemPeriphClockGet(dev->PclkIdx);

	if (Rate == 0 || fclk == 0)
	{
		return 0;
	}

	// The kernel clock, OVER8 and BRR are changed with UE cleared. The
	// interrupt handler writes CR1 too, so the sequence is not interrupted.
	uint32_t state = DisableInterrupt();
	uint32_t cr1 = reg->CR1 & ~USART_CR1_OVER8;
	uint32_t brr = 0;
	uint32_t rate = 0;

	reg->CR1 = cr1 & ~USART_CR1_UE;

#ifdef LPUART1
	if (reg == LPUART1)
	{
		// fck must be between 3 and 4096 times the rate. The LSE crystal
		// keeps the LPUART running in low power modes, HSI16 covers the
		// rates up to 16 MHz / 3, PCLK the ones above.
		uint32_t clksel = ST_LPUART_CLKSEL_PCLK;

		if (GetLowFreqOscType() == OSC_TYPE_XTAL && Rate <= GetLowFreqOscFreq() / 3U)
		{
			clksel = ST_LPUART_CLKSEL_LSE;
			fclk = GetLowFreqOscFreq();
		}
		else if (Rate <= ST_HSI16_FREQ / 3U)
		{
			RCC->CR |= RCC_CR_HSION;
			while ((RCC->CR & RCC_CR_HSIRDY) == 0);

			clksel = ST_LPUART_CLKSEL_HSI16;
			fclk = ST_HSI16_FREQ;
		}

		RCC->ST_RCC_LPUART1SELR = (RCC->ST_RCC_LPUART1SELR & ~(ST_LPUART_CLKSEL_MSK << ST_RCC_LPUART1SEL_Pos)) |
								  (clksel << ST_RCC_LPUART1SEL_Pos);

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

	if (pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

	if (!pDev->bIntEn)
	{
		while (cnt < Bufflen)
		{
			uint32_t flags = ST_USART_ISR(reg);

			if (flags & ST_USART_ISR_RXERR)
			{
				if (flags & ST_USART_ISR_ORE)
				{
					dev->pUartDev->RxOvrErrCnt++;
				}
				if (flags & ST_USART_ISR_PE)
				{
					dev->pUartDev->ParErrCnt++;
				}
				if (flags & ST_USART_ISR_FE)
				{
					dev->pUartDev->FramErrCnt++;
				}
				ST_USART_RXERR_CLEAR(reg, flags);
				if (flags & ST_USART_ISR_RXNE)
				{
					(void)ST_USART_RDR(reg);
				}
				break;
			}
			if ((flags & ST_USART_ISR_RXNE) == 0)
			{
				break;
			}
			pBuff[cnt++] = (uint8_t)ST_USART_RDR(reg) & (dev->pUartDev->DataBits == 7 ? 0x7F : 0xFF);
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
		// The first write waits for TXE, the next ones for TC of the
		// previous frame, so frames are not sent back to back. With TXE
		// only, Release builds dropped data on the F030 PRBS test.
		while (cnt < Datalen)
		{
			uint32_t ready = dev->PollTxStarted ? ST_USART_ISR_TC : ST_USART_ISR_TXE;
			if ((ST_USART_ISR(reg) & ready) == 0U)
			{
				break;
			}
			ST_USART_TDR(reg) = pData[cnt++];
			dev->PollTxStarted = true;
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
		if (dev->pUartDev->bTxReady && (ST_USART_ISR(reg) & ST_USART_ISR_TXE))
		{
			uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
			if (p != NULL)
			{
				dev->pUartDev->bTxReady = false;
				ST_USART_TDR(reg) = *p;
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

// Also the PowerOff hook. Registers keep their content while the clock is
// off, Enable restarts the UART without a new UARTInit.
static void Stm32UartDisable(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	dev->pReg->CR1 &= ~(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
	*dev->pRccEnReg &= ~dev->RccMask;

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

	dev->PollTxStarted = false;
	dev->pUartDev->bTxReady = true;

	*dev->pRccEnReg |= dev->RccMask;
	(void)*dev->pRccEnReg;

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

	dev->pReg->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	EnableInterrupt(state);
}

// RCC reset of the instance, all its registers back to their reset value.
static void Stm32UartReset(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

	*dev->pRccRstReg |= dev->RccMask;
	(void)*dev->pRccRstReg;
	*dev->pRccRstReg &= ~dev->RccMask;
	dev->PollTxStarted = false;

	EnableInterrupt(state);
}

static void *Stm32UartGetHandle(DevIntrf_t * const pDev)
{
	return ((Stm32UartDev_t *)pDev->pDevData)->pUartDev;
}

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

	// An instance running for another UART object is not taken over.
	if (dev->pUartDev != NULL && dev->pUartDev != pDev && (reg->CR1 & USART_CR1_UE))
	{
		EnableInterrupt(state);

		return false;
	}

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;
	dev->pIOPinMap = (const IOPinCfg_t *)pCfg->pIOPinMap;
	dev->NbIOPins = pCfg->NbIOPins;

	// Peripheral clock first. Register writes before it are ignored.
	*dev->pRccEnReg |= dev->RccMask;
	(void)*dev->pRccEnReg;

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
		*dev->pRccEnReg &= ~dev->RccMask;
		EnableInterrupt(state);

		return false;
	}

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

	pDev->Rate = (int)Stm32UartSetRate(&pDev->DevIntrf, (uint32_t)pCfg->Rate);
	if (pDev->Rate == 0)
	{
		*dev->pRccEnReg &= ~dev->RccMask;
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
	pDev->DevIntrf.PowerOff = Stm32UartDisable;
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

#ifdef LPUART1
extern "C" void LPUART1_IRQHandler()
{
	Stm32UartIrqHandler(LPUART1);
}
#endif

extern "C" void USART1_IRQHandler()
{
	Stm32UartIrqHandler(USART1);
}

#ifdef USART2
extern "C" void USART2_IRQHandler()
{
	Stm32UartIrqHandler(USART2);
}
#endif

#if defined(USART3_4_IRQn) || defined(USART3_6_IRQn)
// One vector for USART3 and up, named USART3_4 by the device header on every
// part that has it.
extern "C" void USART3_4_IRQHandler()
{
	Stm32UartIrqHandler(USART3);
#ifdef USART4
	Stm32UartIrqHandler(USART4);
#endif
#ifdef USART5
	Stm32UartIrqHandler(USART5);
#endif
#ifdef USART6
	Stm32UartIrqHandler(USART6);
#endif
}
#else
#ifdef USART3
extern "C" void USART3_IRQHandler()
{
	Stm32UartIrqHandler(USART3);
}
#endif

#ifdef UART4
extern "C" void UART4_IRQHandler()
{
	Stm32UartIrqHandler(UART4);
}
#endif

#ifdef UART5
extern "C" void UART5_IRQHandler()
{
	Stm32UartIrqHandler(UART5);
}
#endif

#ifdef USART6
extern "C" void USART6_IRQHandler()
{
	Stm32UartIrqHandler(USART6);
}
#endif
#endif
