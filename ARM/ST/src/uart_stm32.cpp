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

		TxData puts the data in the TX CFIFO, then starts TX if it is ready.
		RxData reads from the RX CFIFO. Without the interrupt, TxData and
		RxData both move what the hardware holds, in both directions.

		bIntMode false : polling, TxData and RxData move the CFIFO data to and
		                 from the data register
		bIntMode true  : USART interrupts drain the TX CFIFO and fill the
		                 RX CFIFO
		bDMAMode true  : STM32F030x8 only, TX DMA sends blocks from the TX
		                 CFIFO, circular RX DMA ring moved into the RX CFIFO

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
#define ST_USART_ISR_RXNE				USART_ISR_RXNE
#define ST_USART_CR1_TXEIE				USART_CR1_TXEIE
#define ST_USART_CR1_RXNEIE				USART_CR1_RXNEIE
#else
// Hardware FIFO names. With FIFOEN at its reset value, cleared, they are the
// plain TXE and RXNE flags.
#define ST_USART_ISR_TXE				USART_ISR_TXE_TXFNF
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
#define ST_UART_DMA_TX_SIZE		256
#define ST_UART_DMA_RX_SIZE		256
#define ST_UART_DMA_CFIFO_SIZE	CFIFO_MEMSIZE(ST_UART_DMA_TX_SIZE)
#define ST_UART_DMA_RX_CFIFO_SIZE	CFIFO_MEMSIZE(ST_UART_DMA_RX_SIZE)
// RX DMA ring, power of 2. Moved into the RX CFIFO at half and full ring and
// on IDLE, or by RxData without the interrupt.
#define ST_UART_DMA_RX_RING		64
// Most bytes TxData and RxData copy to or from a CFIFO with interrupts
// disabled. Most USARTs have a single RDR, the RX interrupt must read it
// within one character time or the next character is lost: 480 cycles at
// 1 Mbaud on a 48 MHz Cortex-M0. With 8 bytes the copy keeps interrupts
// disabled for at most about 260 cycles there.
#define ST_UART_COPY_MAX		8

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
	uint8_t RxDataMask;				//!< RDR data bits, 0x7F with 7 data bits
#if defined(STM32F030x8)
	DMA_Channel_TypeDef *pRxDma;	//!< RX DMA channel
	uint32_t RxDmaShift;		//!< Channel flags shift
	uint16_t RxDmaRead;		//!< Ring index of the next byte to move to the RX CFIFO
	alignas(4) uint8_t RxDmaRing[ST_UART_DMA_RX_RING];
	alignas(4) uint8_t RxDmaFifoMem[ST_UART_DMA_RX_CFIFO_SIZE];	//!< Default RX CFIFO memory in DMA mode
	DMA_Channel_TypeDef *pTxDma;
	IRQn_Type TxDmaIrq;
	uint32_t TxDmaShift;
	bool DmaActive;					//!< TX DMA transfer running
	uint32_t TxDmaLength;			//!< TX CFIFO bytes owned by the running transfer
	alignas(4) uint8_t TxDmaAlign[4];	//!< Bytes up to the next aligned FIFO address
	alignas(4) uint8_t TxDmaFifoMem[ST_UART_DMA_CFIFO_SIZE];	//!< Default TX CFIFO memory in DMA mode
#endif
	alignas(4) uint8_t RxFifoMem[ST_UART_CFIFO_SIZE];	//!< Default RX CFIFO memory
	alignas(4) uint8_t TxFifoMem[ST_UART_CFIFO_SIZE];	//!< Default TX CFIFO memory
} Stm32UartDev_t;

#pragma pack(pop)

// Position of each instance in s_Stm32UartDev, in the same order and under
// the same conditions as the table. The vectors pass their entry directly.
enum {
#ifdef LPUART1
	ST_LPUART1_IDX,
#endif
	ST_USART1_IDX,
#ifdef USART2
	ST_USART2_IDX,
#endif
#ifdef USART3
	ST_USART3_IDX,
#endif
#ifdef USART4
	ST_USART4_IDX,
#endif
#ifdef UART4
	ST_UART4_IDX,
#endif
#ifdef USART5
	ST_USART5_IDX,
#endif
#ifdef UART5
	ST_UART5_IDX,
#endif
#ifdef USART6
	ST_USART6_IDX,
#endif
};

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
#if defined(STM32F030x8)
		.pRxDma = DMA1_Channel3,
		.RxDmaShift = 8,
		.pTxDma = DMA1_Channel2,
		.TxDmaIrq = DMA1_Channel2_3_IRQn,
		.TxDmaShift = 4,
#endif
	},
#ifdef USART2
	{
		.pReg = USART2,
		.IrqNo = USART2_IRQn,
		.pRccEnReg = &RCC->ST_RCC_APB1ENR,
		.pRccRstReg = &RCC->ST_RCC_APB1RSTR,
		.RccMask = ST_RCC_APB1EN(USART2),
		.PclkIdx = 0,
#if defined(STM32F030x8)
		.pRxDma = DMA1_Channel5,
		.RxDmaShift = 16,
		.pTxDma = DMA1_Channel4,
		.TxDmaIrq = DMA1_Channel4_5_IRQn,
		.TxDmaShift = 12,
#endif
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

static void Stm32UartIrqHandler(Stm32UartDev_t * const pDev);
#if defined(STM32F030x8)
static int Stm32UartDmaRxMove(Stm32UartDev_t * const pDev);
static void Stm32UartDmaIrq(Stm32UartDev_t * const pDev);
static void Stm32UartDmaRxStop(Stm32UartDev_t * const pDev);
static void Stm32UartDmaSetup(Stm32UartDev_t * const pDev);
static bool Stm32UartDmaTxStart(Stm32UartDev_t * const pDev);
static bool Stm32UartDmaService(Stm32UartDev_t * const pDev);
static void Stm32UartDmaStop(Stm32UartDev_t * const pDev);
#endif
static uint32_t Stm32UartGetRate(DevIntrf_t * const pDev);
static uint32_t Stm32UartSetRate(DevIntrf_t * const pDev, uint32_t Rate);
static bool Stm32UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Stm32UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen);
static void Stm32UartStopRx(DevIntrf_t * const pDev);
static bool Stm32UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Stm32UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen);
static void Stm32UartPollRxRead(Stm32UartDev_t * const pDev);
static void Stm32UartStopTx(DevIntrf_t * const pDev);
static void Stm32UartDisable(DevIntrf_t * const pDev);
static void Stm32UartEnable(DevIntrf_t * const pDev);
static void Stm32UartReset(DevIntrf_t * const pDev);
static void *Stm32UartGetHandle(DevIntrf_t * const pDev);

#if defined(STM32F030x8)
// RX DMA runs in circular mode over RxDmaRing, CNDTR gives the write
// position. Move what it wrote since the last call into the RX CFIFO, as
// nRFUARTRxDmaEnd does with its DMA buffer. A full FIFO drops the rest.
// One caller context: the DMA and IDLE interrupts with the interrupt
// enabled, RxData without it. The bytes are written before they are
// published, the consumer may read the FIFO at the same time. The DMA
// copies RDR as is, with 7 data bits the 8th bit is masked here. Returns
// the number of bytes moved.
static int Stm32UartDmaRxMove(Stm32UartDev_t * const pDev)
{
	UARTDev_t *dev = pDev->pUartDev;
	uint32_t write = (ST_UART_DMA_RX_RING - pDev->pRxDma->CNDTR) & (ST_UART_DMA_RX_RING - 1U);
	int cnt = 0;

	while (pDev->RxDmaRead != write)
	{
		int l = (write > pDev->RxDmaRead ? write : ST_UART_DMA_RX_RING) - pDev->RxDmaRead;
		uint8_t *p = CFifoResvMultiple(dev->hRxFifo, &l);
		if (p == NULL)
		{
			dev->RxDropCnt += (write - pDev->RxDmaRead) & (ST_UART_DMA_RX_RING - 1U);
			pDev->RxDmaRead = write;
			break;
		}
		memcpy(p, &pDev->RxDmaRing[pDev->RxDmaRead], l);
		if (pDev->RxDataMask != 0xFFU)
		{
			for (int i = 0; i < l; i++)
			{
				p[i] &= pDev->RxDataMask;
			}
		}
		CFifoPutMultiple(dev->hRxFifo, &l);
		pDev->RxDmaRead = (pDev->RxDmaRead + l) & (ST_UART_DMA_RX_RING - 1U);
		cnt += l;
	}
	if (cnt > 0)
	{
		dev->bRxReady = true;
	}

	return cnt;
}

static void Stm32UartDmaRxStop(Stm32UartDev_t * const pDev)
{
	if (pDev->pRxDma == NULL)
	{
		return;
	}
	pDev->pReg->CR3 &= ~USART_CR3_DMAR;
	pDev->pReg->CR1 &= ~USART_CR1_IDLEIE;
	pDev->pRxDma->CCR &= ~DMA_CCR_EN;
	DMA1->IFCR = DMA_IFCR_CGIF1 << pDev->RxDmaShift;
	pDev->RxDmaRead = 0;
}

// Program both channels: RX circular over its ring, TX idle until a block is
// started. Used by UARTInit and Enable, since Disable stops both channels.
static void Stm32UartDmaSetup(Stm32UartDev_t * const pDev)
{
	USART_TypeDef *reg = pDev->pReg;
	bool intmode = pDev->pUartDev->DevIntrf.bIntEn;

	pDev->pRxDma->CCR = 0;
	DMA1->IFCR = DMA_IFCR_CGIF1 << pDev->RxDmaShift;
	pDev->pRxDma->CPAR = (uint32_t)(uintptr_t)&ST_USART_RDR(reg);
	pDev->pRxDma->CMAR = (uint32_t)(uintptr_t)pDev->RxDmaRing;
	pDev->pRxDma->CNDTR = ST_UART_DMA_RX_RING;
	pDev->pRxDma->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN |
						(intmode ? DMA_CCR_HTIE | DMA_CCR_TCIE : 0U);
	pDev->RxDmaRead = 0;

	pDev->pTxDma->CCR = 0;
	DMA1->IFCR = DMA_IFCR_CGIF1 << pDev->TxDmaShift;
	pDev->pTxDma->CPAR = (uint32_t)(uintptr_t)&ST_USART_TDR(reg);
	pDev->pTxDma->CMAR = 0;
	pDev->pTxDma->CNDTR = 0;
	pDev->pTxDma->CCR = DMA_CCR_DIR | DMA_CCR_MINC | (intmode ? DMA_CCR_TEIE : 0U);
	pDev->DmaActive = false;
	pDev->TxDmaLength = 0;

	reg->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
	if (intmode)
	{
		reg->CR1 |= USART_CR1_IDLEIE;
	}
}

// Start a DMA block with the first contiguous run of the TX CFIFO, sent in
// place. The bytes stay in the FIFO, so the producer cannot write over them,
// until the block is done. The DMA reads from an aligned address: when the
// FIFO read position is not aligned, the bytes up to the next aligned
// address are copied to TxDmaAlign and sent first, the next block continues
// aligned. A longer block is cut to a multiple of 4 so that the next one
// starts aligned too, the rest goes in the next block. TC is cleared first
// so that it is set again by the last character of this block. Returns false
// when the FIFO is empty.
static bool Stm32UartDmaTxStart(Stm32UartDev_t * const pDev)
{
	int cnt = 0xFFFC;
	uint8_t *p = CFifoPeekMultiple(pDev->pUartDev->hTxFifo, &cnt);

	if (p == NULL)
	{
		return false;
	}

	uint32_t len = (uint32_t)cnt;
	uint32_t head = (uint32_t)(uintptr_t)p & 3U;

	if (head != 0U)
	{
		head = 4U - head;
		if (len > head)
		{
			len = head;
		}
		memcpy(pDev->TxDmaAlign, p, len);
		p = pDev->TxDmaAlign;
	}
	else if (len > 4U && (len & 3U) != 0U)
	{
		len &= ~3U;
	}
	pDev->TxDmaLength = len;
	pDev->DmaActive = true;
	pDev->pReg->ICR = USART_ICR_TCCF;
	if (pDev->pUartDev->DevIntrf.bIntEn)
	{
		pDev->pReg->CR1 |= USART_CR1_TCIE;
	}
	pDev->pTxDma->CMAR = (uint32_t)(uintptr_t)p;
	pDev->pTxDma->CNDTR = len;
	DMA1->IFCR = DMA_IFCR_CGIF1 << pDev->TxDmaShift;
	// Last, the interrupt has little left to do once the block runs
	pDev->pTxDma->CCR |= DMA_CCR_EN;

	return true;
}

// A block is done when the DMA has moved all of it and TC is set, its last
// character has left the line, or when the DMA stopped on a transfer error.
// The DMA transfer complete flag is not used, it is set as soon as the last
// byte is in TDR. Holding TX busy until TC lets the bytes queued meanwhile go
// in the next block. The block bytes are released and the next block takes
// what was queued. With the FIFO empty TX is ready again and the application
// is told. From the TC and DMA interrupts, or from TxData in polling mode.
// Returns false if no block was done.
static bool Stm32UartDmaService(Stm32UartDev_t * const pDev)
{
	UARTDev_t *dev = pDev->pUartDev;

	if (dev == NULL || !pDev->DmaActive)
	{
		return false;
	}

	if (pDev->pTxDma->CNDTR != 0U)
	{
		if ((DMA1->ISR & (DMA_ISR_TEIF1 << pDev->TxDmaShift)) == 0U)
		{
			return false;
		}
	}
	else if ((ST_USART_ISR(pDev->pReg) & USART_ISR_TC) == 0U)
	{
		return false;
	}

	pDev->pTxDma->CCR &= ~DMA_CCR_EN;
	// Not sent when a transfer error stopped the block
	dev->TxDropCnt += pDev->pTxDma->CNDTR;
	DMA1->IFCR = DMA_IFCR_CGIF1 << pDev->TxDmaShift;
	pDev->DmaActive = false;
	CFifoRelease(dev->hTxFifo, (int)pDev->TxDmaLength);
	pDev->TxDmaLength = 0;

	if (Stm32UartDmaTxStart(pDev))
	{
		return true;
	}

	pDev->pReg->CR1 &= ~USART_CR1_TCIE;
	dev->bTxReady = true;
	if (dev->DevIntrf.bIntEn && dev->EvtCallback)
	{
		// Length is the free space in the TX FIFO
		dev->EvtCallback(dev, UART_EVT_TXREADY, NULL, CFifoAvail(dev->hTxFifo));
	}

	return true;
}

// DMA interrupt of an instance: RX ring half or full, TX transfer error
static void Stm32UartDmaIrq(Stm32UartDev_t * const pDev)
{
	UARTDev_t *dev = pDev->pUartDev;

	if (dev == NULL || !dev->DevIntrf.bDma)
	{
		return;
	}

	if (DMA1->ISR & ((DMA_ISR_HTIF1 | DMA_ISR_TCIF1) << pDev->RxDmaShift))
	{
		DMA1->IFCR = DMA_IFCR_CGIF1 << pDev->RxDmaShift;
		if (Stm32UartDmaRxMove(pDev) > 0 && dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_RXDATA, NULL, CFifoUsed(dev->hRxFifo));
		}
	}

	Stm32UartDmaService(pDev);
}

static void Stm32UartDmaStop(Stm32UartDev_t * const pDev)
{
	if (pDev->pTxDma == NULL)
	{
		return;
	}
	pDev->pReg->CR3 &= ~USART_CR3_DMAT;
	pDev->pReg->CR1 &= ~USART_CR1_TCIE;
	pDev->pTxDma->CCR = 0;
	DMA1->IFCR = DMA_IFCR_CGIF1 << pDev->TxDmaShift;
	pDev->DmaActive = false;
	pDev->TxDmaLength = 0;
}
#endif

// The vector passes its instance.
static void Stm32UartIrqHandler(Stm32UartDev_t * const pDev)
{
	UARTDev_t *dev = pDev->pUartDev;

	// Instances sharing a vector may be unused or in polling mode
	if (dev == NULL || !dev->DevIntrf.bIntEn)
	{
		return;
	}

	USART_TypeDef *reg = pDev->pReg;
	uint32_t iflag = ST_USART_ISR(reg);
	uint32_t cr1 = reg->CR1;

#if defined(STM32F030x8)
	if ((cr1 & USART_CR1_IDLEIE) && (iflag & USART_ISR_IDLE))
	{
		// End of an RX burst, what the DMA received goes in the RX CFIFO
		reg->ICR = USART_ICR_IDLECF;
		if (Stm32UartDmaRxMove(pDev) > 0 && dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_RXDATA, NULL, CFifoUsed(dev->hRxFifo));
		}
	}
#endif

	// RX errors: clear only the error flags seen. PE, FE and NE belong to the
	// character in RDR, which is read and dropped. ORE means the character
	// after it was lost, the one in RDR is valid and is read below. With RX
	// DMA, RXNEIE is off and RDR is left to the DMA.
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
		ST_USART_RXERR_CLEAR(reg, iflag);
		if ((iflag & (ST_USART_ISR_PE | ST_USART_ISR_FE | ST_USART_ISR_NE)) &&
			(cr1 & ST_USART_CR1_RXNEIE))
		{
			if (iflag & ST_USART_ISR_RXNE)
			{
				(void)ST_USART_RDR(reg);
			}
			iflag &= ~ST_USART_ISR_RXNE;
		}
	}

	if ((iflag & ST_USART_ISR_RXNE) && (cr1 & ST_USART_CR1_RXNEIE))
	{
		// Reading RDR clears RXNE
		uint8_t c = (uint8_t)ST_USART_RDR(reg) & pDev->RxDataMask;
		CFifo_t *fifo = dev->hRxFifo;

		if (CFifoPutByte(fifo, c))
		{
			dev->bRxReady = true;
		}
		else
		{
			dev->RxDropCnt++;
		}
		if (dev->EvtCallback)
		{
			dev->EvtCallback(dev, UART_EVT_RXDATA, NULL, CFifoUsed(fifo));
		}
	}

#if defined(STM32F030x8)
	if ((iflag & USART_ISR_TC) && (cr1 & USART_CR1_TCIE))
	{
		// End of a TX DMA block. If the DMA is still behind, TC is cleared and
		// set again by the last character.
		if (!Stm32UartDmaService(pDev))
		{
			reg->ICR = USART_ICR_TCCF;
		}
	}
#endif

	if ((iflag & ST_USART_ISR_TXE) && (cr1 & ST_USART_CR1_TXEIE))
	{
		int c = CFifoGetByte(dev->hTxFifo);

		if (c >= 0)
		{
			// Writing TDR clears TXE
			ST_USART_TDR(reg) = (uint8_t)c;
		}
		else
		{
			// FIFO empty, mask TXEIE so TXE does not keep interrupting.
			reg->CR1 &= ~ST_USART_CR1_TXEIE;
			dev->bTxReady = true;
			if (dev->EvtCallback)
			{
				// Length is the free space in the TX FIFO
				dev->EvtCallback(dev, UART_EVT_TXREADY, NULL, CFifoAvail(dev->hTxFifo));
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

// Without the interrupt, RDR goes in the RX CFIFO as the interrupt would put
// it, while a character is there. Interrupts disabled.
static void Stm32UartPollRxRead(Stm32UartDev_t * const pDev)
{
	UARTDev_t *dev = pDev->pUartDev;
	USART_TypeDef *reg = pDev->pReg;

	while (true)
	{
		uint32_t flags = ST_USART_ISR(reg);

		if (flags & ST_USART_ISR_RXERR)
		{
			if (flags & ST_USART_ISR_ORE)
			{
				dev->RxOvrErrCnt++;
			}
			if (flags & ST_USART_ISR_PE)
			{
				dev->ParErrCnt++;
			}
			if (flags & ST_USART_ISR_FE)
			{
				dev->FramErrCnt++;
			}
			ST_USART_RXERR_CLEAR(reg, flags);
			// The character in RDR is valid after an overrun alone
			if (flags & (ST_USART_ISR_PE | ST_USART_ISR_FE | ST_USART_ISR_NE))
			{
				if (flags & ST_USART_ISR_RXNE)
				{
					(void)ST_USART_RDR(reg);
				}
				continue;
			}
		}
		if ((flags & ST_USART_ISR_RXNE) == 0)
		{
			break;
		}

		// RDR is read first, as in the RXNE interrupt: a full FIFO drops the
		// character
		uint8_t c = (uint8_t)ST_USART_RDR(reg) & pDev->RxDataMask;
		if (CFifoPutByte(dev->hRxFifo, c))
		{
			dev->bRxReady = true;
		}
		else
		{
			dev->RxDropCnt++;
		}
	}
}

// Stm32UartPollRxRead when a character or an RX error is there. The flag
// check alone is cheap enough for the TX path. Interrupts disabled.
static inline __attribute__((always_inline)) void Stm32UartPollRx(Stm32UartDev_t * const pDev)
{
	if (ST_USART_ISR(pDev->pReg) & (ST_USART_ISR_RXNE | ST_USART_ISR_RXERR))
	{
		Stm32UartPollRxRead(pDev);
	}
}

// Without the interrupt, TDR takes the TX CFIFO bytes while it is free. TXE
// set means TDR is free while the shift register still sends the previous
// character. Interrupts disabled.
static void Stm32UartPollTx(Stm32UartDev_t * const pDev)
{
	USART_TypeDef *reg = pDev->pReg;

	while (ST_USART_ISR(reg) & ST_USART_ISR_TXE)
	{
		int c = CFifoGetByte(pDev->pUartDev->hTxFifo);
		if (c < 0)
		{
			break;
		}
		ST_USART_TDR(reg) = (uint8_t)c;
	}
}

// Without the interrupt, RxData services the hardware in both directions, as
// the interrupts would: RDR goes in the RX CFIFO and TDR takes the TX CFIFO
// bytes, or with DMA the RX ring is moved, the finished TX block retired and
// the next one started. Bytes written earlier go out while the application
// only reads. TxData services TX, and in polling mode reads RDR between the
// pieces it copies. Interrupts disabled.
static void Stm32UartPoll(Stm32UartDev_t * const pDev)
{
#if defined(STM32F030x8)
	if (pDev->pUartDev->DevIntrf.bDma)
	{
		Stm32UartDmaRxMove(pDev);
		Stm32UartDmaService(pDev);
		if (pDev->pUartDev->bTxReady && Stm32UartDmaTxStart(pDev))
		{
			pDev->pUartDev->bTxReady = false;
		}
		return;
	}
#endif
	// RDR checked again after the TDR writes, which can take two
	// characters
	Stm32UartPollRx(pDev);
	Stm32UartPollTx(pDev);
	Stm32UartPollRx(pDev);
}

static int Stm32UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	UARTDev_t *udev = dev->pUartDev;
	hCFifo_t fifo = udev->hRxFifo;
	int cnt = 0;

	if (pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

	// Without the interrupt, the hardware is serviced before the copy: the
	// characters received go in the FIFO after the ones already there, so
	// the order is kept, and RDR is free for the next one while the copy
	// runs. At 1 Mbaud RDR must be read every 480 cycles on a 48 MHz
	// Cortex-M0.
	if (!pDev->bIntEn)
	{
		uint32_t state = DisableInterrupt();
		Stm32UartPoll(dev);
		EnableInterrupt(state);
	}

	// The data comes from the RX CFIFO. CFifoGetMultiple frees the slots
	// before they are copied, interrupts are disabled until the copy is done
	// so that the interrupt cannot put over them. In pieces of at most
	// ST_UART_COPY_MAX bytes to keep the RX interrupt latency short. In
	// polling mode RDR is read and TDR written before each piece, which keeps
	// both directions running at the line rate.
	bool poll = !pDev->bIntEn && !pDev->bDma;

	while (cnt < Bufflen)
	{
		int l = Bufflen - cnt < ST_UART_COPY_MAX ? Bufflen - cnt : ST_UART_COPY_MAX;
		uint32_t state = DisableInterrupt();
		if (poll)
		{
			Stm32UartPollRx(dev);
			Stm32UartPollTx(dev);
			Stm32UartPollRx(dev);
		}
		uint8_t *p = CFifoGetMultiple(fifo, &l);
		if (p != NULL)
		{
			memcpy(pBuff + cnt, p, l);
		}
		EnableInterrupt(state);
		if (p == NULL)
		{
			break;
		}
		cnt += l;
	}

	// Cleared first, then set again if a byte is left. A byte put by the
	// interrupt in between sets it too.
	udev->bRxReady = false;
	if (CFifoUsed(fifo) != 0)
	{
		udev->bRxReady = true;
	}

	// Then what the hardware received goes in the FIFO for the next read.
	// Without the interrupt, Stm32UartPoll does it and sends the TX CFIFO
	// bytes. With the interrupt and DMA, the DMA half, full and IDLE
	// interrupts move the ring only at 32 or 64 bytes or once the line is
	// idle, the ring is moved here too so that the next read gets the bytes
	// received since. With the RXNE interrupt alone there is nothing to move.
	if (!pDev->bIntEn)
	{
		uint32_t state = DisableInterrupt();
		Stm32UartPoll(dev);
		EnableInterrupt(state);
	}
#if defined(STM32F030x8)
	else if (pDev->bDma)
	{
		uint32_t state = DisableInterrupt();
		Stm32UartDmaRxMove(dev);
		EnableInterrupt(state);
	}
#endif

	return cnt;
}

static void Stm32UartStopRx(DevIntrf_t * const pDev)
{
}

static bool Stm32UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

// Several bytes into the TX CFIFO. CFifoPutMultiple publishes the slots
// before they are written, so interrupts are disabled for the copy, in
// pieces of at most ST_UART_COPY_MAX bytes to keep the RX interrupt latency
// short. In polling mode RDR is read and TDR written before each piece, RDR
// read again at the end. Not inlined in TxData, which keeps the one byte path
// short. Returns the number of bytes put.
__attribute__((noinline)) static int Stm32UartTxPut(Stm32UartDev_t * const pDev, uint8_t const *pData, int Datalen)
{
	hCFifo_t fifo = pDev->pUartDev->hTxFifo;
	bool poll = !pDev->pUartDev->DevIntrf.bIntEn && !pDev->pUartDev->DevIntrf.bDma;
	int cnt = 0;

	while (Datalen > 0)
	{
		int l = Datalen < ST_UART_COPY_MAX ? Datalen : ST_UART_COPY_MAX;
		uint32_t state = DisableInterrupt();
		if (poll)
		{
			Stm32UartPollRx(pDev);
			Stm32UartPollTx(pDev);
			Stm32UartPollRx(pDev);
		}
		uint8_t *p = CFifoPutMultiple(fifo, &l);
		if (p != NULL)
		{
			memcpy(p, pData, l);
		}
		EnableInterrupt(state);
		if (p == NULL)
		{
			break;
		}
		Datalen -= l;
		pData += l;
		cnt += l;
	}

	if (poll)
	{
		uint32_t state = DisableInterrupt();
		Stm32UartPollRx(pDev);
		EnableInterrupt(state);
	}

	return cnt;
}

static int Stm32UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int Datalen)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	USART_TypeDef *reg = dev->pReg;
	hCFifo_t fifo = dev->pUartDev->hTxFifo;
	uint32_t state;
	int cnt = 0;

	if (pData == NULL || Datalen <= 0)
	{
		return 0;
	}

	// The data goes in the TX CFIFO first. CFifoPutByte writes the byte
	// before it is published.
	if (Datalen == 1)
	{
		cnt = CFifoPutByte(fifo, *pData) ? 1 : 0;
	}
	else
	{
		cnt = Stm32UartTxPut(dev, pData, Datalen);
	}

	// Then start TX if it is ready. Otherwise the running interrupt or DMA
	// transfer takes the bytes just put. Checked again with interrupts
	// disabled, TxData may also be called from an interrupt. Without the
	// interrupt, what the hardware takes now is sent, the next TxData or
	// RxData call sends the rest.
#if defined(STM32F030x8)
	if (pDev->bDma)
	{
		// DMA: a block from the FIFO, the TC interrupt sends the rest.
		// Without the interrupt the finished block is retired here.
		if (!pDev->bIntEn || dev->pUartDev->bTxReady)
		{
			state = DisableInterrupt();
			if (!pDev->bIntEn)
			{
				Stm32UartDmaService(dev);
			}
			if (dev->pUartDev->bTxReady && Stm32UartDmaTxStart(dev))
			{
				dev->pUartDev->bTxReady = false;
			}
			EnableInterrupt(state);
		}

		return cnt;
	}
#endif

	// Without the interrupt TX stays ready
	if (dev->pUartDev->bTxReady)
	{
		state = DisableInterrupt();
		if (dev->pUartDev->bTxReady)
		{
			if (pDev->bIntEn)
			{
				// The first byte goes to TDR, the TXE interrupt sends the rest.
				// TX ready means TDR is empty.
				int c = CFifoGetByte(fifo);
				if (c >= 0)
				{
					dev->pUartDev->bTxReady = false;
					ST_USART_TDR(reg) = (uint8_t)c;
					reg->CR1 |= ST_USART_CR1_TXEIE;
				}
			}
			else
			{
				// RDR before and after the TDR writes, the next RxData call
				// may come later than one character time
				Stm32UartPollRx(dev);
				Stm32UartPollTx(dev);
				Stm32UartPollRx(dev);
			}
		}
		EnableInterrupt(state);
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

#if defined(STM32F030x8)
	if (pDev->bDma)
	{
		Stm32UartDmaRxStop(dev);
		Stm32UartDmaStop(dev);
	}
#endif
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

#if defined(STM32F030x8)
	if (pDev->bDma)
	{
		Stm32UartDmaRxStop(dev);
		Stm32UartDmaStop(dev);
	}
#endif
	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;

	*dev->pRccEnReg |= dev->RccMask;
	(void)*dev->pRccEnReg;

	IOPinCfg(dev->pIOPinMap, dev->NbIOPins);

#if defined(STM32F030x8)
	if (pDev->bDma)
	{
		Stm32UartDmaSetup(dev);
	}
#endif
	dev->pReg->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	EnableInterrupt(state);
}

// RCC reset of the instance, all its registers back to their reset value.
static void Stm32UartReset(DevIntrf_t * const pDev)
{
	Stm32UartDev_t *dev = (Stm32UartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();

#if defined(STM32F030x8)
	if (pDev->bDma)
	{
		Stm32UartDmaRxStop(dev);
		Stm32UartDmaStop(dev);
	}
#endif
	*dev->pRccRstReg |= dev->RccMask;
	(void)*dev->pRccRstReg;
	*dev->pRccRstReg &= ~dev->RccMask;

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

	// Select polling, USART interrupt or DMA. DMA is supported only on the
	// F030x8 instances with mapped DMA channels.
	if ((pCfg->bDMAMode && !pCfg->bFifoBlocking) ||
		pCfg->bIrDAMode || pCfg->Mode != UART_MODE_UART ||
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
#if defined(STM32F030x8)
	if (pCfg->bDMAMode && dev->pTxDma == NULL)
	{
		return false;
	}
	if (pCfg->bDMAMode && dev->pTxDma->CCR != 0U &&
		(dev->pUartDev == NULL || dev->pUartDev != pDev))
	{
		return false;
	}
#else
	if (pCfg->bDMAMode)
	{
		return false;
	}
#endif
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
#if defined(STM32F030x8)
	else if (pCfg->bDMAMode)
	{
		pDev->hRxFifo = CFifoInit(dev->RxDmaFifoMem, ST_UART_DMA_RX_CFIFO_SIZE, 1,
								 pCfg->bFifoBlocking);
	}
#endif
	else
	{
		pDev->hRxFifo = CFifoInit(dev->RxFifoMem, ST_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
#if defined(STM32F030x8)
	else if (pCfg->bDMAMode)
	{
		pDev->hTxFifo = CFifoInit(dev->TxDmaFifoMem, ST_UART_DMA_CFIFO_SIZE, 1,
								 pCfg->bFifoBlocking);
	}
#endif
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
	dev->RxDataMask = pCfg->DataBits == 7 ? 0x7F : 0xFF;
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
	pDev->DevIntrf.bDma = pCfg->bDMAMode;
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

#if defined(STM32F030x8)
	if (pCfg->bDMAMode)
	{
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		(void)RCC->AHBENR;
		// USART1 uses the default DMA request mapping on channel 3.
		if (reg == USART1)
		{
			RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
			SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1RX_DMA_RMP;
		}
		Stm32UartDmaSetup(dev);
		cr3 |= USART_CR3_DMAT | USART_CR3_DMAR;
		if (pCfg->bIntMode)
		{
			NVIC_ClearPendingIRQ(dev->TxDmaIrq);
			NVIC_SetPriority(dev->TxDmaIrq, pCfg->IntPrio);
			NVIC_EnableIRQ(dev->TxDmaIrq);
		}
	}
#endif
	if (pCfg->bIntMode)
	{
		// Only the interrupts the handler services. TXEIE is set by
		// Stm32UartTxData when the TX FIFO has data, TCIE by the TX DMA start.
		// IDLEIE is used with RX DMA. RTOIE stays off.
		if (pCfg->bDMAMode)
		{
			cr1 |= USART_CR1_IDLEIE | USART_CR1_PEIE;
		}
		else
		{
			cr1 |= ST_USART_CR1_RXNEIE | USART_CR1_PEIE;
		}

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

#if defined(STM32F030x8)
extern "C" void DMA1_Channel2_3_IRQHandler()
{
	uint32_t state = DisableInterrupt();
	Stm32UartDmaIrq(&s_Stm32UartDev[ST_USART1_IDX]);
	EnableInterrupt(state);
}

extern "C" void DMA1_Channel4_5_IRQHandler()
{
	uint32_t state = DisableInterrupt();
	Stm32UartDmaIrq(&s_Stm32UartDev[ST_USART2_IDX]);
	EnableInterrupt(state);
}
#endif

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
}

#ifdef LPUART1
extern "C" void LPUART1_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_LPUART1_IDX]);
}
#endif

extern "C" void USART1_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_USART1_IDX]);
}

#ifdef USART2
extern "C" void USART2_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_USART2_IDX]);
}
#endif

#if defined(USART3_4_IRQn) || defined(USART3_6_IRQn)
// One vector for USART3 and up, named USART3_4 by the device header on every
// part that has it.
extern "C" void USART3_4_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_USART3_IDX]);
#ifdef USART4
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_USART4_IDX]);
#endif
#ifdef USART5
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_USART5_IDX]);
#endif
#ifdef USART6
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_USART6_IDX]);
#endif
}
#else
#ifdef USART3
extern "C" void USART3_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_USART3_IDX]);
}
#endif

#ifdef UART4
extern "C" void UART4_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_UART4_IDX]);
}
#endif

#ifdef UART5
extern "C" void UART5_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_UART5_IDX]);
}
#endif

#ifdef USART6
extern "C" void USART6_IRQHandler()
{
	Stm32UartIrqHandler(&s_Stm32UartDev[ST_USART6_IDX]);
}
#endif
#endif
