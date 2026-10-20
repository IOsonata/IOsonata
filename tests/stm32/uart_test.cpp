/**-------------------------------------------------------------------------
@file	uart_test.cpp

@brief	Host test of the STM32 UART driver, ARM/ST/src/uart_stm32.cpp.

		The driver is compiled with the ST device header of the part, the
		host core header of tests/stm32/shim, and runs with the real CFIFO
		and DeviceIntrf code. The peripheral address ranges are mapped in the
		test process. The USART, DMA and RCC pages are protected: each access
		of the driver stops on SIGSEGV, runs as a single step with the trap
		flag, then the model below applies its side effects.

		USART model : RDR read clears RXNE and loads the next character the
		host sent. ICR write clears its ISR flags. TDR write puts the
		character on the wire and clears TC. TXE stays set. SR and DR parts
		(F4): DR read clears RXNE and the error and IDLE flags seen by the
		last SR read, SR write clears TC and RXNE written 0.

		DMA model : IFCR, LIFCR, HIFCR and the GPDMA CFCR clear their flags.
		A GPDMA channel ignores EN written 0, SUSP sets SUSPF and IDLEF,
		RESET clears EN and SUSP. A GPDMA transfer register written with EN
		set is counted as a violation. The transfers are moved by the test:
		TX blocks go to the wire, then TC is set, RX bytes go in the ring
		with the half and full flags, the circular reload or the GPDMA linked
		list node.

		RCC model : a write setting the reset bit of an instance is counted.

		Checks for each instance, in the DevNo order of the driver: RCC
		enable and reset bits, kernel clock selection, vector and NVIC
		number, the PCLK used, polling and interrupt TX and RX, RX errors,
		full FIFO. DMA where the driver has it: channel or stream and its
		request, peripheral addresses, circular RX, TX blocks, RX ring, a
		transfer error, polled DMA, Disable and Enable, a channel already in
		use, interrupt mode after DMA. Instances without DMA refuse it.
		Framing on USART1, LPUART1 clock selection.

		Linux x86-64 only, the single step uses the trap flag. Not a timing
		or hardware test.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <ucontext.h>

#include "stm32.h"
#if defined(STM32WBA)
#include "gpdma_stm32wba.h"
#elif defined(STM32L4)
#include "dma_stm32.h"
#endif
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"

#define CHECK(c)	do { s_CheckCnt++; if (!(c)) { printf("FAIL %s:%d [%s] %s\n", __FILE__, __LINE__, s_pName, #c); s_FailCnt++; } } while (0)

static int s_FailCnt;
static int s_CheckCnt;
static unsigned s_AccCnt;
static const char *s_pName = "";

// ---- Register names of the part --------------------------------------------

#if defined(USART_ISR_TXE) || defined(USART_ISR_TXE_TXFNF)
#define T_SR_TYPE		0
#define T_ISR(r)		((r)->ISR)
#define T_RDR(r)		((r)->RDR)
#define T_TDR(r)		((r)->TDR)
#ifdef USART_ISR_TXE
#define T_TXE			USART_ISR_TXE
#define T_RXNE			USART_ISR_RXNE
#define T_TXEIE			USART_CR1_TXEIE
#define T_RXNEIE		USART_CR1_RXNEIE
#else
#define T_TXE			USART_ISR_TXE_TXFNF
#define T_RXNE			USART_ISR_RXNE_RXFNE
#define T_TXEIE			USART_CR1_TXEIE_TXFNFIE
#define T_RXNEIE		USART_CR1_RXNEIE_RXFNEIE
#endif
#define T_TC			USART_ISR_TC
#define T_IDLE			USART_ISR_IDLE
#define T_ORE			USART_ISR_ORE
#define T_FE			USART_ISR_FE
#define T_PE			USART_ISR_PE
#define T_NE			USART_ISR_NE
#else
#define T_SR_TYPE		1
#define T_ISR(r)		((r)->SR)
#define T_RDR(r)		((r)->DR)
#define T_TDR(r)		((r)->DR)
#define T_TXE			USART_SR_TXE
#define T_RXNE			USART_SR_RXNE
#define T_TXEIE			USART_CR1_TXEIE
#define T_RXNEIE		USART_CR1_RXNEIE
#define T_TC			USART_SR_TC
#define T_IDLE			USART_SR_IDLE
#define T_ORE			USART_SR_ORE
#define T_FE			USART_SR_FE
#define T_PE			USART_SR_PE
#define T_NE			USART_SR_NE
#endif
#define T_RXERR			(T_PE | T_FE | T_NE | T_ORE)

#ifdef USART_CR1_M0
#define T_M0			USART_CR1_M0
#else
#define T_M0			USART_CR1_M
#endif

#if defined(DMA_SxCR_CHSEL)
#define T_DMA_STREAM
typedef DMA_Stream_TypeDef TChan_t;
#elif defined(DMA_CTR2_REQSEL)
#define T_DMA_GPDMA
typedef DMA_Channel_TypeDef TChan_t;
#else
typedef DMA_Channel_TypeDef TChan_t;
#endif

// DMA flags given to the test helpers
#define F_TC			1U
#define F_HT			2U
#define F_TE			4U

// ---- Expected instances, from the reference manuals -------------------------

typedef struct {
	void *pChan;					// Channel or stream, NULL none
	void *pCtl;						// Controller
	int Idx;						// Channel number - 1, stream or GPDMA channel number
	int Req;						// CSELR, DMAMUX1, CHSEL or GPDMA request
	int IrqNo;
	void (*Vector)(void);
} TestDma_t;

typedef struct {
	const char *pName;
	USART_TypeDef *pReg;
	void (*Vector)(void);
	int IrqNo;
	volatile uint32_t *pEnReg;
	volatile uint32_t *pRstReg;
	uint32_t Bit;					// Enable and reset bit
	volatile uint32_t *pSelReg;		// Kernel clock selection, NULL none
	uint32_t SelMsk;
	int PclkIdx;					// SystemPeriphClockGet index of its bus
	bool bLpuart;
	TestDma_t Rx;
	TestDma_t Tx;
} TestInst_t;

#define MAX_INST		8

static TestInst_t s_Inst[MAX_INST];
static int s_NbInst;

extern "C" void USART1_IRQHandler(void);
extern "C" void USART2_IRQHandler(void);
extern "C" void USART3_IRQHandler(void);
extern "C" void USART3_4_IRQHandler(void);
extern "C" void UART4_IRQHandler(void);
extern "C" void UART5_IRQHandler(void);
extern "C" void USART6_IRQHandler(void);
extern "C" void LPUART1_IRQHandler(void);
extern "C" void DMA1_Channel2_3_IRQHandler(void);
extern "C" void DMA1_Channel4_5_IRQHandler(void);
extern "C" void DMA1_Channel1_IRQHandler(void);
extern "C" void DMA1_Channel2_IRQHandler(void);
extern "C" void DMA1_Channel3_IRQHandler(void);
extern "C" void DMA1_Channel4_IRQHandler(void);
extern "C" void DMA1_Channel5_IRQHandler(void);
extern "C" void DMA1_Channel6_IRQHandler(void);
extern "C" void DMA1_Channel7_IRQHandler(void);
extern "C" void DMA2_Channel1_IRQHandler(void);
extern "C" void DMA2_Channel2_IRQHandler(void);
extern "C" void DMA2_Channel3_IRQHandler(void);
extern "C" void DMA2_Channel4_IRQHandler(void);
extern "C" void DMA2_Channel5_IRQHandler(void);
extern "C" void DMA2_Channel6_IRQHandler(void);
extern "C" void DMA2_Channel7_IRQHandler(void);
extern "C" void DMA1_Stream0_IRQHandler(void);
extern "C" void DMA1_Stream1_IRQHandler(void);
extern "C" void DMA1_Stream2_IRQHandler(void);
extern "C" void DMA1_Stream3_IRQHandler(void);
extern "C" void DMA1_Stream4_IRQHandler(void);
extern "C" void DMA1_Stream5_IRQHandler(void);
extern "C" void DMA1_Stream6_IRQHandler(void);
extern "C" void DMA1_Stream7_IRQHandler(void);
extern "C" void DMA2_Stream1_IRQHandler(void);
extern "C" void DMA2_Stream5_IRQHandler(void);
extern "C" void DMA2_Stream6_IRQHandler(void);
extern "C" void DMA2_Stream7_IRQHandler(void);
extern "C" void GPDMA1_Channel0_IRQHandler(void);
extern "C" void GPDMA1_Channel1_IRQHandler(void);
extern "C" void GPDMA1_Channel2_IRQHandler(void);
extern "C" void GPDMA1_Channel3_IRQHandler(void);
extern "C" void GPDMA1_Channel4_IRQHandler(void);
extern "C" void GPDMA1_Channel5_IRQHandler(void);
extern "C" void GPDMA1_Channel6_IRQHandler(void);
extern "C" void GPDMA1_Channel7_IRQHandler(void);

#define TDMA(Ch, Ctl, Idx, Req, Irq, Vec)	TestDma_t{ (void *)(Ch), (void *)(Ctl), Idx, Req, (int)(Irq), Vec }

static TestInst_t &AddInst(const char *pName, USART_TypeDef *pReg, void (*Vector)(void), int IrqNo,
						   volatile uint32_t *pEn, volatile uint32_t *pRst, uint32_t Bit, int PclkIdx)
{
	TestInst_t &t = s_Inst[s_NbInst++];

	memset(&t, 0, sizeof(t));
	t.pName = pName;
	t.pReg = pReg;
	t.Vector = Vector;
	t.IrqNo = IrqNo;
	t.pEnReg = pEn;
	t.pRstReg = pRst;
	t.Bit = Bit;
	t.PclkIdx = PclkIdx;

	return t;
}

// Same order as the instance table of the driver: LPUART1, USART1, USART2,
// USART3, USART4 or UART4, USART5 or UART5, USART6
static void BuildInst(void)
{
#if defined(STM32F0)
	// One APB, PCLK index 0. USART3 and up share one vector, no DMA there.
	{
		TestInst_t &t = AddInst("USART1", USART1, USART1_IRQHandler, USART1_IRQn, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_USART1EN, 0);
#ifdef RCC_CFGR3_USART1SW
		t.pSelReg = &RCC->CFGR3;
		t.SelMsk = RCC_CFGR3_USART1SW;
#endif
		t.Rx = TDMA(DMA1_Channel3, DMA1, 2, 8, DMA1_Channel2_3_IRQn, DMA1_Channel2_3_IRQHandler);
		t.Tx = TDMA(DMA1_Channel2, DMA1, 1, 8, DMA1_Channel2_3_IRQn, DMA1_Channel2_3_IRQHandler);
	}
#ifdef USART2
	{
		TestInst_t &t = AddInst("USART2", USART2, USART2_IRQHandler, USART2_IRQn, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_USART2EN, 0);
#ifdef RCC_CFGR3_USART2SW
		t.pSelReg = &RCC->CFGR3;
		t.SelMsk = RCC_CFGR3_USART2SW;
#endif
		t.Rx = TDMA(DMA1_Channel5, DMA1, 4, 9, DMA1_Channel4_5_IRQn, DMA1_Channel4_5_IRQHandler);
		t.Tx = TDMA(DMA1_Channel4, DMA1, 3, 9, DMA1_Channel4_5_IRQn, DMA1_Channel4_5_IRQHandler);
	}
#endif
#ifdef USART3
	{
		TestInst_t &t = AddInst("USART3", USART3, USART3_4_IRQHandler, USART3_4_IRQn, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_USART3EN, 0);
#ifdef RCC_CFGR3_USART3SW
		t.pSelReg = &RCC->CFGR3;
		t.SelMsk = RCC_CFGR3_USART3SW;
#endif
	}
#endif
#ifdef USART4
	AddInst("USART4", USART4, USART3_4_IRQHandler, USART3_4_IRQn, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_USART4EN, 0);
#endif
#ifdef USART5
	AddInst("USART5", USART5, USART3_4_IRQHandler, USART3_4_IRQn, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_USART5EN, 0);
#endif
#ifdef USART6
	AddInst("USART6", USART6, USART3_4_IRQHandler, USART3_4_IRQn, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_USART6EN, 0);
#endif

#elif defined(STM32L4)
	// DMA1 and DMA2 channels, request in CSELR, or DMAMUX1 numbers of the
	// L4R and L4S on L4+
#if defined(DMAMUX1)
#define L4REQ(Csel, Mux)	(Mux)
#else
#define L4REQ(Csel, Mux)	(Csel)
#endif
#ifdef LPUART1
	{
		TestInst_t &t = AddInst("LPUART1", LPUART1, LPUART1_IRQHandler, LPUART1_IRQn, &RCC->APB1ENR2, &RCC->APB1RSTR2, RCC_APB1ENR2_LPUART1EN, 0);
		t.bLpuart = true;
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_LPUART1SEL;
		t.Rx = TDMA(DMA2_Channel7, DMA2, 6, L4REQ(4, 34), DMA2_Channel7_IRQn, DMA2_Channel7_IRQHandler);
		t.Tx = TDMA(DMA2_Channel6, DMA2, 5, L4REQ(4, 35), DMA2_Channel6_IRQn, DMA2_Channel6_IRQHandler);
	}
#endif
	{
		TestInst_t &t = AddInst("USART1", USART1, USART1_IRQHandler, USART1_IRQn, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_USART1EN, 1);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_USART1SEL;
		t.Rx = TDMA(DMA1_Channel5, DMA1, 4, L4REQ(2, 24), DMA1_Channel5_IRQn, DMA1_Channel5_IRQHandler);
		t.Tx = TDMA(DMA1_Channel4, DMA1, 3, L4REQ(2, 25), DMA1_Channel4_IRQn, DMA1_Channel4_IRQHandler);
	}
#ifdef USART2
	{
		TestInst_t &t = AddInst("USART2", USART2, USART2_IRQHandler, USART2_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_USART2EN, 0);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_USART2SEL;
		t.Rx = TDMA(DMA1_Channel6, DMA1, 5, L4REQ(2, 26), DMA1_Channel6_IRQn, DMA1_Channel6_IRQHandler);
		t.Tx = TDMA(DMA1_Channel7, DMA1, 6, L4REQ(2, 27), DMA1_Channel7_IRQn, DMA1_Channel7_IRQHandler);
	}
#endif
#ifdef USART3
	{
		TestInst_t &t = AddInst("USART3", USART3, USART3_IRQHandler, USART3_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_USART3EN, 0);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_USART3SEL;
		t.Rx = TDMA(DMA1_Channel3, DMA1, 2, L4REQ(2, 28), DMA1_Channel3_IRQn, DMA1_Channel3_IRQHandler);
		t.Tx = TDMA(DMA1_Channel2, DMA1, 1, L4REQ(2, 29), DMA1_Channel2_IRQn, DMA1_Channel2_IRQHandler);
	}
#endif
#ifdef UART4
	{
		TestInst_t &t = AddInst("UART4", UART4, UART4_IRQHandler, UART4_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_UART4EN, 0);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_UART4SEL;
		t.Rx = TDMA(DMA2_Channel5, DMA2, 4, L4REQ(2, 30), DMA2_Channel5_IRQn, DMA2_Channel5_IRQHandler);
		t.Tx = TDMA(DMA2_Channel3, DMA2, 2, L4REQ(2, 31), DMA2_Channel3_IRQn, DMA2_Channel3_IRQHandler);
	}
#endif
#ifdef UART5
	{
		TestInst_t &t = AddInst("UART5", UART5, UART5_IRQHandler, UART5_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_UART5EN, 0);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_UART5SEL;
		t.Rx = TDMA(DMA2_Channel2, DMA2, 1, L4REQ(2, 32), DMA2_Channel2_IRQn, DMA2_Channel2_IRQHandler);
		t.Tx = TDMA(DMA2_Channel1, DMA2, 0, L4REQ(2, 33), DMA2_Channel1_IRQn, DMA2_Channel1_IRQHandler);
	}
#endif

#elif defined(STM32G4)
	// DMA1 and DMA2 channels 1 to 6 routed by DMAMUX1, request numbers of RM0440
	{
		TestInst_t &t = AddInst("LPUART1", LPUART1, LPUART1_IRQHandler, LPUART1_IRQn, &RCC->APB1ENR2, &RCC->APB1RSTR2, RCC_APB1ENR2_LPUART1EN, 0);
		t.bLpuart = true;
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_LPUART1SEL;
		t.Rx = TDMA(DMA2_Channel5, DMA2, 4, 34, DMA2_Channel5_IRQn, DMA2_Channel5_IRQHandler);
		t.Tx = TDMA(DMA2_Channel6, DMA2, 5, 35, DMA2_Channel6_IRQn, DMA2_Channel6_IRQHandler);
	}
	{
		TestInst_t &t = AddInst("USART1", USART1, USART1_IRQHandler, USART1_IRQn, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_USART1EN, 1);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_USART1SEL;
		t.Rx = TDMA(DMA1_Channel1, DMA1, 0, 24, DMA1_Channel1_IRQn, DMA1_Channel1_IRQHandler);
		t.Tx = TDMA(DMA1_Channel2, DMA1, 1, 25, DMA1_Channel2_IRQn, DMA1_Channel2_IRQHandler);
	}
	{
		TestInst_t &t = AddInst("USART2", USART2, USART2_IRQHandler, USART2_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_USART2EN, 0);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_USART2SEL;
		t.Rx = TDMA(DMA1_Channel3, DMA1, 2, 26, DMA1_Channel3_IRQn, DMA1_Channel3_IRQHandler);
		t.Tx = TDMA(DMA1_Channel4, DMA1, 3, 27, DMA1_Channel4_IRQn, DMA1_Channel4_IRQHandler);
	}
	{
		TestInst_t &t = AddInst("USART3", USART3, USART3_IRQHandler, USART3_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_USART3EN, 0);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_USART3SEL;
		t.Rx = TDMA(DMA1_Channel5, DMA1, 4, 28, DMA1_Channel5_IRQn, DMA1_Channel5_IRQHandler);
		t.Tx = TDMA(DMA1_Channel6, DMA1, 5, 29, DMA1_Channel6_IRQn, DMA1_Channel6_IRQHandler);
	}
#ifdef UART4
	{
		TestInst_t &t = AddInst("UART4", UART4, UART4_IRQHandler, UART4_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_UART4EN, 0);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_UART4SEL;
		t.Rx = TDMA(DMA2_Channel1, DMA2, 0, 30, DMA2_Channel1_IRQn, DMA2_Channel1_IRQHandler);
		t.Tx = TDMA(DMA2_Channel2, DMA2, 1, 31, DMA2_Channel2_IRQn, DMA2_Channel2_IRQHandler);
	}
#endif
#ifdef UART5
	{
		TestInst_t &t = AddInst("UART5", UART5, UART5_IRQHandler, UART5_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_UART5EN, 0);
		t.pSelReg = &RCC->CCIPR;
		t.SelMsk = RCC_CCIPR_UART5SEL;
		t.Rx = TDMA(DMA2_Channel3, DMA2, 2, 32, DMA2_Channel3_IRQn, DMA2_Channel3_IRQHandler);
		t.Tx = TDMA(DMA2_Channel4, DMA2, 3, 33, DMA2_Channel4_IRQn, DMA2_Channel4_IRQHandler);
	}
#endif

#elif defined(STM32F4)
	// DMA streams, the request is the channel number in CR
	{
		TestInst_t &t = AddInst("USART1", USART1, USART1_IRQHandler, USART1_IRQn, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_USART1EN, 1);
		t.Rx = TDMA(DMA2_Stream5, DMA2, 5, 4, DMA2_Stream5_IRQn, DMA2_Stream5_IRQHandler);
		t.Tx = TDMA(DMA2_Stream7, DMA2, 7, 4, DMA2_Stream7_IRQn, DMA2_Stream7_IRQHandler);
	}
#ifdef USART2
	{
		TestInst_t &t = AddInst("USART2", USART2, USART2_IRQHandler, USART2_IRQn, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_USART2EN, 0);
		t.Rx = TDMA(DMA1_Stream5, DMA1, 5, 4, DMA1_Stream5_IRQn, DMA1_Stream5_IRQHandler);
		t.Tx = TDMA(DMA1_Stream6, DMA1, 6, 4, DMA1_Stream6_IRQn, DMA1_Stream6_IRQHandler);
	}
#endif
#ifdef USART3
	{
		TestInst_t &t = AddInst("USART3", USART3, USART3_IRQHandler, USART3_IRQn, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_USART3EN, 0);
		t.Rx = TDMA(DMA1_Stream1, DMA1, 1, 4, DMA1_Stream1_IRQn, DMA1_Stream1_IRQHandler);
		t.Tx = TDMA(DMA1_Stream3, DMA1, 3, 4, DMA1_Stream3_IRQn, DMA1_Stream3_IRQHandler);
	}
#endif
#ifdef UART4
	{
		TestInst_t &t = AddInst("UART4", UART4, UART4_IRQHandler, UART4_IRQn, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_UART4EN, 0);
		t.Rx = TDMA(DMA1_Stream2, DMA1, 2, 4, DMA1_Stream2_IRQn, DMA1_Stream2_IRQHandler);
		t.Tx = TDMA(DMA1_Stream4, DMA1, 4, 4, DMA1_Stream4_IRQn, DMA1_Stream4_IRQHandler);
	}
#endif
#ifdef UART5
	{
		TestInst_t &t = AddInst("UART5", UART5, UART5_IRQHandler, UART5_IRQn, &RCC->APB1ENR, &RCC->APB1RSTR, RCC_APB1ENR_UART5EN, 0);
		t.Rx = TDMA(DMA1_Stream0, DMA1, 0, 4, DMA1_Stream0_IRQn, DMA1_Stream0_IRQHandler);
#if defined(STM32F413xx) || defined(STM32F423xx)
		t.Tx = TDMA(DMA1_Stream7, DMA1, 7, 8, DMA1_Stream7_IRQn, DMA1_Stream7_IRQHandler);
#else
		t.Tx = TDMA(DMA1_Stream7, DMA1, 7, 4, DMA1_Stream7_IRQn, DMA1_Stream7_IRQHandler);
#endif
	}
#endif
#ifdef USART6
	{
		TestInst_t &t = AddInst("USART6", USART6, USART6_IRQHandler, USART6_IRQn, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_USART6EN, 1);
		t.Rx = TDMA(DMA2_Stream1, DMA2, 1, 5, DMA2_Stream1_IRQn, DMA2_Stream1_IRQHandler);
		t.Tx = TDMA(DMA2_Stream6, DMA2, 6, 5, DMA2_Stream6_IRQn, DMA2_Stream6_IRQHandler);
	}
#endif

#elif defined(STM32WBA)
	// GPDMA1, any request on any channel, two channels for each instance
	{
		TestInst_t &t = AddInst("LPUART1", LPUART1, LPUART1_IRQHandler, LPUART1_IRQn, &RCC->APB7ENR, &RCC->APB7RSTR, RCC_APB7ENR_LPUART1EN, 2);
		t.bLpuart = true;
		t.pSelReg = &RCC->CCIPR3;
		t.SelMsk = RCC_CCIPR3_LPUART1SEL;
		t.Rx = TDMA(GPDMA1_Channel4, GPDMA1, 4, 15, GPDMA1_Channel4_IRQn, GPDMA1_Channel4_IRQHandler);
		t.Tx = TDMA(GPDMA1_Channel5, GPDMA1, 5, 16, GPDMA1_Channel5_IRQn, GPDMA1_Channel5_IRQHandler);
	}
	{
		TestInst_t &t = AddInst("USART1", USART1, USART1_IRQHandler, USART1_IRQn, &RCC->APB2ENR, &RCC->APB2RSTR, RCC_APB2ENR_USART1EN, 1);
		t.pSelReg = &RCC->CCIPR1;
		t.SelMsk = RCC_CCIPR1_USART1SEL;
		t.Rx = TDMA(GPDMA1_Channel0, GPDMA1, 0, 11, GPDMA1_Channel0_IRQn, GPDMA1_Channel0_IRQHandler);
		t.Tx = TDMA(GPDMA1_Channel1, GPDMA1, 1, 12, GPDMA1_Channel1_IRQn, GPDMA1_Channel1_IRQHandler);
	}
#ifdef USART2
	{
		TestInst_t &t = AddInst("USART2", USART2, USART2_IRQHandler, USART2_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_USART2EN, 0);
		t.pSelReg = &RCC->CCIPR1;
		t.SelMsk = RCC_CCIPR1_USART2SEL;
		t.Rx = TDMA(GPDMA1_Channel2, GPDMA1, 2, 13, GPDMA1_Channel2_IRQn, GPDMA1_Channel2_IRQHandler);
		t.Tx = TDMA(GPDMA1_Channel3, GPDMA1, 3, 14, GPDMA1_Channel3_IRQn, GPDMA1_Channel3_IRQHandler);
	}
#endif
#ifdef USART3
	{
		TestInst_t &t = AddInst("USART3", USART3, USART3_IRQHandler, USART3_IRQn, &RCC->APB1ENR1, &RCC->APB1RSTR1, RCC_APB1ENR1_USART3EN, 0);
		t.pSelReg = &RCC->CCIPR1;
		t.SelMsk = RCC_CCIPR1_USART3SEL;
		t.Rx = TDMA(GPDMA1_Channel6, GPDMA1, 6, 65, GPDMA1_Channel6_IRQn, GPDMA1_Channel6_IRQHandler);
		t.Tx = TDMA(GPDMA1_Channel7, GPDMA1, 7, 66, GPDMA1_Channel7_IRQn, GPDMA1_Channel7_IRQHandler);
	}
#endif
#else
#error "uart_test.cpp: family not listed"
#endif
}

// ---- Stubs of the functions the driver calls --------------------------------

McuOsc_t g_McuOsc = { { OSC_TYPE_RC, 16000000, 20, 0 }, { OSC_TYPE_RC, 32768, 20, 0 }, false };

// PCLK of each SystemPeriphClockGet index, all different so that the rate
// shows which one an instance uses
static uint32_t s_Pclk[3] = { 24000000, 48000000, 12000000 };

static int s_PinCfgCnt;
static int s_PinDisCnt;
static bool s_NvicEn[256];
static uint32_t s_NvicPrio[256];

extern "C" uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx >= 0 && Idx < 3 ? s_Pclk[Idx] : 0;
}

extern "C" void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	s_PinCfgCnt++;
}

extern "C" void IOPinDisable(int PortNo, int PinNo)
{
	s_PinDisCnt++;
}

extern "C" void TestNvic(int Op, int IrqNo, uint32_t Prio)
{
	if (IrqNo < 0 || IrqNo >= 256)
	{
		return;
	}
	if (Op == 0)
	{
		s_NvicEn[IrqNo] = true;
	}
	else if (Op == 1)
	{
		s_NvicEn[IrqNo] = false;
	}
	else if (Op == 3)
	{
		s_NvicPrio[IrqNo] = Prio;
	}
}

// ---- Peripheral model ------------------------------------------------------

#define MAX_PAGE		8
#define WIRE_MAX		16384
#define RXQ_MAX			1024

static uintptr_t s_Page[MAX_PAGE];
static int s_NbPage;
static uintptr_t s_AccAddr;
static bool s_bAccWr;
static uint32_t s_AccOld;
#if T_SR_TYPE
static uint32_t s_SrSeen[MAX_INST];
#endif
static uint8_t s_Wire[MAX_INST][WIRE_MAX];
static int s_WireLen[MAX_INST];
static uint8_t s_RxQ[MAX_INST][RXQ_MAX];
static int s_RxQRd[MAX_INST];
static int s_RxQWr[MAX_INST];
static int s_RstCnt[MAX_INST];
static int s_Violation;

static void AddPage(uintptr_t Addr)
{
	uintptr_t p = Addr & ~(uintptr_t)0xFFF;

	for (int i = 0; i < s_NbPage; i++)
	{
		if (s_Page[i] == p)
		{
			return;
		}
	}
	if (s_NbPage < MAX_PAGE)
	{
		s_Page[s_NbPage++] = p;
	}
}

static bool IsPage(uintptr_t Page)
{
	for (int i = 0; i < s_NbPage; i++)
	{
		if (s_Page[i] == Page)
		{
			return true;
		}
	}

	return false;
}

static void Unprot(void)
{
	for (int i = 0; i < s_NbPage; i++)
	{
		mprotect((void *)s_Page[i], 4096, PROT_READ | PROT_WRITE);
	}
}

static void Prot(void)
{
	for (int i = 0; i < s_NbPage; i++)
	{
		mprotect((void *)s_Page[i], 4096, PROT_NONE);
	}
}

// Register access by the test, without the model
template <typename T> static uint32_t Rd(volatile T &Reg)
{
	Unprot();
	uint32_t v = Reg;
	Prot();

	return v;
}

template <typename T> static void Wr(volatile T &Reg, uint32_t Val)
{
	Unprot();
	Reg = (T)Val;
	Prot();
}

template <typename T> static void Set(volatile T &Reg, uint32_t Bits)
{
	Unprot();
	Reg = (T)(Reg | Bits);
	Prot();
}

template <typename T> static void Clr(volatile T &Reg, uint32_t Bits)
{
	Unprot();
	Reg = (T)(Reg & ~Bits);
	Prot();
}

static inline volatile uint32_t &At(uintptr_t Addr)
{
	return *(volatile uint32_t *)Addr;
}

template <typename T> static uintptr_t Ad(volatile T &Reg)
{
	return (uintptr_t)&Reg;
}

// Next character from the host into RDR, with the page open
static void RxFeed(int Inst)
{
	USART_TypeDef *reg = s_Inst[Inst].pReg;

	if (s_RxQRd[Inst] != s_RxQWr[Inst] && (T_ISR(reg) & T_RXNE) == 0)
	{
		T_RDR(reg) = s_RxQ[Inst][s_RxQRd[Inst]++];
		T_ISR(reg) |= T_RXNE;
	}
}

static void WirePut(int Inst, uint8_t c)
{
	if (s_WireLen[Inst] < WIRE_MAX)
	{
		s_Wire[Inst][s_WireLen[Inst]++] = c;
	}
}

// Before the access, the page open
static void ModelBefore(uintptr_t Addr, bool bWr)
{
#if T_SR_TYPE
	for (int i = 0; i < s_NbInst; i++)
	{
		if (Addr == Ad(T_ISR(s_Inst[i].pReg)) && !bWr)
		{
			s_SrSeen[i] = T_ISR(s_Inst[i].pReg);
		}
	}
#endif
}

// After the access, the page still open
static void ModelAfter(uintptr_t Addr, bool bWr, uint32_t Old)
{
	uint32_t val = At(Addr);

	bWr = bWr || val != Old;

	for (int i = 0; i < s_NbInst; i++)
	{
		USART_TypeDef *reg = s_Inst[i].pReg;

		if (Addr == Ad(*s_Inst[i].pRstReg) && bWr && (val & s_Inst[i].Bit) && !(Old & s_Inst[i].Bit))
		{
			// Instance registers back to their reset value
			uintptr_t pg = (uintptr_t)reg & ~(uintptr_t)0xFFF;
			bool other = pg != (Addr & ~(uintptr_t)0xFFF);

			s_RstCnt[i]++;
			if (other)
			{
				mprotect((void *)pg, 4096, PROT_READ | PROT_WRITE);
			}
			reg->CR1 = 0;
			reg->CR2 = 0;
			reg->CR3 = 0;
			reg->BRR = 0;
			T_ISR(reg) = T_TXE | T_TC;
			if (other)
			{
				mprotect((void *)pg, 4096, PROT_NONE);
			}
		}
		if (Addr < (uintptr_t)reg || Addr >= (uintptr_t)reg + sizeof(USART_TypeDef))
		{
			continue;
		}
#if T_SR_TYPE
		if (Addr == Ad(reg->DR))
		{
			if (bWr)
			{
				WirePut(i, (uint8_t)val);
				reg->SR &= ~T_TC;
			}
			else
			{
				reg->SR &= ~(T_RXNE | (s_SrSeen[i] & (T_RXERR | T_IDLE)));
				RxFeed(i);
			}
		}
		else if (Addr == Ad(reg->SR) && bWr)
		{
			// TC, RXNE, LBD and CTS cleared by writing 0, the others read only
			const uint32_t w0 = USART_SR_TC | USART_SR_RXNE | USART_SR_LBD | USART_SR_CTS;
			reg->SR = Old & (val | ~w0);
		}
#else
		if (Addr == Ad(reg->RDR) && !bWr)
		{
			reg->ISR &= ~T_RXNE;
			RxFeed(i);
		}
		else if (Addr == Ad(reg->TDR) && bWr)
		{
			WirePut(i, (uint8_t)val);
			reg->ISR &= ~T_TC;
		}
		else if (Addr == Ad(reg->ICR) && bWr)
		{
			reg->ISR &= ~(val & ~(T_RXNE | T_TXE));
		}
#endif
		return;
	}

#if defined(T_DMA_STREAM)
	DMA_TypeDef *dmas[2] = { DMA1, DMA2 };
	for (DMA_TypeDef *d : dmas)
	{
		if (Addr == Ad(d->LIFCR) && bWr)
		{
			d->LISR &= ~val;
		}
		else if (Addr == Ad(d->HIFCR) && bWr)
		{
			d->HISR &= ~val;
		}
	}
#elif defined(T_DMA_GPDMA)
	// Only the page of the access is open
	for (int n = 0; n < 8 && (Addr & ~(uintptr_t)0xFFF) == (GPDMA1_BASE & ~(uintptr_t)0xFFF); n++)
	{
		DMA_Channel_TypeDef *ch = (DMA_Channel_TypeDef *)(GPDMA1_BASE + 0x50 + 0x80 * n);

		if (Addr == Ad(ch->CFCR) && bWr)
		{
			ch->CSR &= ~val;
		}
		else if (Addr == Ad(ch->CCR) && bWr)
		{
			if (val & DMA_CCR_RESET)
			{
				ch->CCR = val & ~(DMA_CCR_EN | DMA_CCR_SUSP | DMA_CCR_RESET);
				ch->CSR |= DMA_CSR_IDLEF;
			}
			else
			{
				if ((Old & DMA_CCR_EN) && !(val & DMA_CCR_EN))
				{
					// EN written 0 is ignored
					val |= DMA_CCR_EN;
					s_Violation++;
					printf("GPDMA channel %d: EN written 0\n", n);
				}
				if ((val & DMA_CCR_SUSP) && !(Old & DMA_CCR_SUSP) && (Old & DMA_CCR_EN))
				{
					ch->CSR |= DMA_CSR_SUSPF | DMA_CSR_IDLEF;
				}
				if ((val & DMA_CCR_EN) && !(Old & DMA_CCR_EN))
				{
					ch->CSR &= ~DMA_CSR_IDLEF;
				}
				ch->CCR = val;
			}
		}
		else if (bWr && (Addr == Ad(ch->CTR1) || Addr == Ad(ch->CTR2) || Addr == Ad(ch->CBR1) ||
				 Addr == Ad(ch->CSAR) || Addr == Ad(ch->CDAR) || Addr == Ad(ch->CLLR) || Addr == Ad(ch->CLBAR)) &&
				 (ch->CCR & DMA_CCR_EN))
		{
			// Read only with EN set
			At(Addr) = Old;
			s_Violation++;
			printf("GPDMA channel %d: register written with EN set\n", n);
		}
	}
#else
	DMA_TypeDef *dmas[2] = { DMA1,
#ifdef DMA2
		DMA2
#else
		DMA1
#endif
	};
	for (DMA_TypeDef *d : dmas)
	{
		if (Addr == Ad(d->IFCR) && bWr)
		{
			// CGIFx clears the 4 flags of channel x
			uint32_t clr = 0;
			for (int c = 0; c < 8; c++)
			{
				uint32_t g = (val >> (4 * c)) & 0xFU;
				clr |= ((g & 1U) ? 0xFU : g) << (4 * c);
			}
			d->ISR &= ~clr;
		}
	}
#endif
}

static void OnSegv(int Sig, siginfo_t *pInfo, void *pCtx)
{
	ucontext_t *uc = (ucontext_t *)pCtx;
	uintptr_t a = (uintptr_t)pInfo->si_addr;
	uintptr_t page = a & ~(uintptr_t)0xFFF;

	if (!IsPage(page))
	{
		// A real fault
		signal(SIGSEGV, SIG_DFL);
		return;
	}
	s_AccAddr = a & ~(uintptr_t)3;
	s_bAccWr = (uc->uc_mcontext.gregs[REG_ERR] & 2) != 0;
	mprotect((void *)page, 4096, PROT_READ | PROT_WRITE);
	s_AccOld = At(s_AccAddr);
	ModelBefore(s_AccAddr, s_bAccWr);
	uc->uc_mcontext.gregs[REG_EFL] |= 0x100;
}

static void OnTrap(int Sig, siginfo_t *pInfo, void *pCtx)
{
	ucontext_t *uc = (ucontext_t *)pCtx;

	uc->uc_mcontext.gregs[REG_EFL] &= ~0x100LL;
	s_AccCnt++;
	ModelAfter(s_AccAddr, s_bAccWr, s_AccOld);
	mprotect((void *)(s_AccAddr & ~(uintptr_t)0xFFF), 4096, PROT_NONE);
}

static void MapRange(uintptr_t Addr, size_t Len)
{
	void *p = mmap((void *)Addr, Len, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

	if (p != (void *)Addr)
	{
		printf("mmap %lx failed\n", (unsigned long)Addr);
		exit(2);
	}
}

// ---- Host side of the line ---------------------------------------------------

static void RxSend(int Inst, const uint8_t *p, int Len)
{
	for (int k = 0; k < Len && s_RxQWr[Inst] < RXQ_MAX; k++)
	{
		s_RxQ[Inst][s_RxQWr[Inst]++] = p[k];
	}
	Unprot();
	RxFeed(Inst);
	Prot();
}

static void RxQReset(int Inst)
{
	s_RxQRd[Inst] = 0;
	s_RxQWr[Inst] = 0;
}

static void WireReset(int Inst)
{
	s_WireLen[Inst] = 0;
}

static bool WireIs(int Inst, const uint8_t *p, int Len)
{
	return s_WireLen[Inst] == Len && memcmp(s_Wire[Inst], p, Len) == 0;
}

// ---- DMA helpers, by controller --------------------------------------------

static TChan_t *Ch(const TestDma_t &d)
{
	return (TChan_t *)d.pChan;
}

#if defined(T_DMA_STREAM)
static uint32_t DmaCnt(const TestDma_t &d) { return Rd(Ch(d)->NDTR); }
static void DmaSetCnt(const TestDma_t &d, uint32_t v) { Wr(Ch(d)->NDTR, v); }
static uint32_t DmaMem(const TestDma_t &d, bool bTx) { return Rd(Ch(d)->M0AR); }
static uint32_t DmaPeriph(const TestDma_t &d, bool bTx) { return Rd(Ch(d)->PAR); }
static bool DmaOn(const TestDma_t &d) { return Rd(Ch(d)->CR) & DMA_SxCR_EN; }
static uint32_t DmaCtrl(const TestDma_t &d) { return Rd(Ch(d)->CR); }
static void DmaSetCtrl(const TestDma_t &d, uint32_t v) { Wr(Ch(d)->CR, v); }
static uint32_t DmaInts(const TestDma_t &d)
{
	uint32_t cr = Rd(Ch(d)->CR);
	return ((cr & DMA_SxCR_TCIE) ? F_TC : 0) | ((cr & DMA_SxCR_HTIE) ? F_HT : 0) | ((cr & DMA_SxCR_TEIE) ? F_TE : 0);
}
static void DmaFlags(const TestDma_t &d, uint32_t f)
{
	static const uint8_t pos[4] = { 0, 6, 16, 22 };
	uint32_t v = ((f & F_TC) ? 0x20U : 0) | ((f & F_HT) ? 0x10U : 0) | ((f & F_TE) ? 0x08U : 0);
	DMA_TypeDef *c = (DMA_TypeDef *)d.pCtl;

	if (d.Idx < 4)
	{
		Set(c->LISR, v << pos[d.Idx]);
	}
	else
	{
		Set(c->HISR, v << pos[d.Idx & 3]);
	}
}
static uint32_t DmaGetFlags(const TestDma_t &d)
{
	static const uint8_t pos[4] = { 0, 6, 16, 22 };
	DMA_TypeDef *c = (DMA_TypeDef *)d.pCtl;
	uint32_t v = (d.Idx < 4 ? Rd(c->LISR) : Rd(c->HISR)) >> pos[d.Idx & 3];

	return ((v & 0x20U) ? F_TC : 0) | ((v & 0x10U) ? F_HT : 0) | ((v & 0x08U) ? F_TE : 0);
}
// A stream clears EN at the end of a normal transfer and on an error
static void DmaEnd(const TestDma_t &d) { Clr(Ch(d)->CR, DMA_SxCR_EN); }
static void DmaStopErr(const TestDma_t &d) { Clr(Ch(d)->CR, DMA_SxCR_EN); }
static void DmaTxDone(const TestDma_t &d, uint32_t n) {}
static uint8_t *DmaRxPos(const TestDma_t &d, uint32_t Ring)
{
	return (uint8_t *)(uintptr_t)Rd(Ch(d)->M0AR) + Ring - DmaCnt(d);
}
static void DmaRxWrap(const TestDma_t &d, uint32_t Ring) { DmaSetCnt(d, Ring); }
static bool DmaCirc(const TestDma_t &d, uint32_t Ring, uint32_t Base) { return (Rd(Ch(d)->CR) & DMA_SxCR_CIRC) != 0 && Rd(Ch(d)->M0AR) == Base; }
static void DmaRoute(const TestDma_t &d, bool bTx)
{
	CHECK(((Rd(Ch(d)->CR) & DMA_SxCR_CHSEL) >> DMA_SxCR_CHSEL_Pos) == (uint32_t)d.Req);
	CHECK(((Rd(Ch(d)->CR) & DMA_SxCR_DIR) == DMA_SxCR_DIR_0) == bTx);
}
static void DmaInUse(const TestDma_t &d) { Wr(Ch(d)->CR, DMA_SxCR_MINC); }

#elif defined(T_DMA_GPDMA)
static uint32_t DmaCnt(const TestDma_t &d) { return Rd(Ch(d)->CBR1) & DMA_CBR1_BNDT; }
static void DmaSetCnt(const TestDma_t &d, uint32_t v) { Wr(Ch(d)->CBR1, (Rd(Ch(d)->CBR1) & ~DMA_CBR1_BNDT) | v); }
static uint32_t DmaMem(const TestDma_t &d, bool bTx) { return bTx ? Rd(Ch(d)->CSAR) : Rd(Ch(d)->CDAR); }
static uint32_t DmaPeriph(const TestDma_t &d, bool bTx) { return bTx ? Rd(Ch(d)->CDAR) : Rd(Ch(d)->CSAR); }
static bool DmaOn(const TestDma_t &d) { return Rd(Ch(d)->CCR) & DMA_CCR_EN; }
static uint32_t DmaCtrl(const TestDma_t &d) { return Rd(Ch(d)->CCR) | Rd(Ch(d)->CTR1) | Rd(Ch(d)->CTR2); }
static void DmaSetCtrl(const TestDma_t &d, uint32_t v) { Wr(Ch(d)->CCR, v); Wr(Ch(d)->CTR1, v); Wr(Ch(d)->CTR2, v); }
static uint32_t DmaInts(const TestDma_t &d)
{
	uint32_t cr = Rd(Ch(d)->CCR);
	return ((cr & DMA_CCR_TCIE) ? F_TC : 0) | ((cr & DMA_CCR_HTIE) ? F_HT : 0) | ((cr & DMA_CCR_DTEIE) ? F_TE : 0);
}
static void DmaFlags(const TestDma_t &d, uint32_t f)
{
	Set(Ch(d)->CSR, ((f & F_TC) ? DMA_CSR_TCF : 0) | ((f & F_HT) ? DMA_CSR_HTF : 0) | ((f & F_TE) ? DMA_CSR_DTEF : 0));
}
static uint32_t DmaGetFlags(const TestDma_t &d)
{
	uint32_t v = Rd(Ch(d)->CSR);

	return ((v & DMA_CSR_TCF) ? F_TC : 0) | ((v & DMA_CSR_HTF) ? F_HT : 0) | ((v & DMA_CSR_DTEF) ? F_TE : 0);
}
// The channel goes idle at the end of its last block and on an error
static void DmaEnd(const TestDma_t &d) { Clr(Ch(d)->CCR, DMA_CCR_EN); Set(Ch(d)->CSR, DMA_CSR_IDLEF); }
static void DmaStopErr(const TestDma_t &d) { DmaEnd(d); }
// The source address moves with SINC
static void DmaTxDone(const TestDma_t &d, uint32_t n) { Wr(Ch(d)->CSAR, Rd(Ch(d)->CSAR) + n); }
static uint8_t *DmaRxPos(const TestDma_t &d, uint32_t Ring)
{
	uint32_t a = Rd(Ch(d)->CDAR);

	Wr(Ch(d)->CDAR, a + 1);

	return (uint8_t *)(uintptr_t)a;
}
// End of the block: the linked list node is loaded, by its update bits
static void DmaRxWrap(const TestDma_t &d, uint32_t Ring)
{
	TChan_t *ch = Ch(d);
	uint32_t llr = Rd(ch->CLLR);

	if (llr == 0)
	{
		DmaEnd(d);
		return;
	}

	const uint32_t *w = (const uint32_t *)(uintptr_t)((Rd(ch->CLBAR) & 0xFFFF0000U) | (llr & 0xFFFCU));

	if (llr & DMA_CLLR_UT1) Wr(ch->CTR1, *w++);
	if (llr & DMA_CLLR_UT2) Wr(ch->CTR2, *w++);
	if (llr & DMA_CLLR_UB1) Wr(ch->CBR1, *w++);
	if (llr & DMA_CLLR_USA) Wr(ch->CSAR, *w++);
	if (llr & DMA_CLLR_UDA) Wr(ch->CDAR, *w++);
	Wr(ch->CLLR, (llr & DMA_CLLR_ULL) ? *w : 0U);
}
// Circular: the node loads the ring again and links to itself
static bool DmaCirc(const TestDma_t &d, uint32_t Ring, uint32_t Base)
{
	TChan_t *ch = Ch(d);
	uint32_t llr = Rd(ch->CLLR);
	uint32_t upd = DMA_CLLR_UB1 | DMA_CLLR_UDA | DMA_CLLR_ULL;

	if ((llr & (upd | DMA_CLLR_UT1 | DMA_CLLR_UT2 | DMA_CLLR_USA)) != upd)
	{
		return false;
	}

	const uint32_t *w = (const uint32_t *)(uintptr_t)((Rd(ch->CLBAR) & 0xFFFF0000U) | (llr & 0xFFFCU));

	return (w[0] & DMA_CBR1_BNDT) == Ring && w[1] == Base && w[2] == llr;
}
static void DmaRoute(const TestDma_t &d, bool bTx)
{
	CHECK((Rd(Ch(d)->CTR2) & DMA_CTR2_REQSEL) == (uint32_t)d.Req);
	CHECK(((Rd(Ch(d)->CTR2) & DMA_CTR2_DREQ) != 0) == bTx);
	CHECK((Rd(Ch(d)->CTR1) & (bTx ? DMA_CTR1_SINC : DMA_CTR1_DINC)) != 0);
}
static void DmaInUse(const TestDma_t &d) { Wr(Ch(d)->CTR1, DMA_CTR1_SINC); }

#else
static uint32_t DmaCnt(const TestDma_t &d) { return Rd(Ch(d)->CNDTR); }
static void DmaSetCnt(const TestDma_t &d, uint32_t v) { Wr(Ch(d)->CNDTR, v); }
static uint32_t DmaMem(const TestDma_t &d, bool bTx) { return Rd(Ch(d)->CMAR); }
static uint32_t DmaPeriph(const TestDma_t &d, bool bTx) { return Rd(Ch(d)->CPAR); }
static bool DmaOn(const TestDma_t &d) { return Rd(Ch(d)->CCR) & DMA_CCR_EN; }
static uint32_t DmaCtrl(const TestDma_t &d) { return Rd(Ch(d)->CCR); }
static void DmaSetCtrl(const TestDma_t &d, uint32_t v) { Wr(Ch(d)->CCR, v); }
static uint32_t DmaInts(const TestDma_t &d)
{
	uint32_t cr = Rd(Ch(d)->CCR);
	return ((cr & DMA_CCR_TCIE) ? F_TC : 0) | ((cr & DMA_CCR_HTIE) ? F_HT : 0) | ((cr & DMA_CCR_TEIE) ? F_TE : 0);
}
static void DmaFlags(const TestDma_t &d, uint32_t f)
{
	uint32_t v = 1U | ((f & F_TC) ? 2U : 0) | ((f & F_HT) ? 4U : 0) | ((f & F_TE) ? 8U : 0);

	Set(((DMA_TypeDef *)d.pCtl)->ISR, v << (4 * d.Idx));
}
static uint32_t DmaGetFlags(const TestDma_t &d)
{
	uint32_t v = Rd(((DMA_TypeDef *)d.pCtl)->ISR) >> (4 * d.Idx);

	return ((v & 2U) ? F_TC : 0) | ((v & 4U) ? F_HT : 0) | ((v & 8U) ? F_TE : 0);
}
// A channel keeps EN after a normal transfer, clears it on an error
static void DmaEnd(const TestDma_t &d) {}
static void DmaStopErr(const TestDma_t &d) { Clr(Ch(d)->CCR, DMA_CCR_EN); }
static void DmaTxDone(const TestDma_t &d, uint32_t n) {}
static uint8_t *DmaRxPos(const TestDma_t &d, uint32_t Ring)
{
	return (uint8_t *)(uintptr_t)Rd(Ch(d)->CMAR) + Ring - DmaCnt(d);
}
static void DmaRxWrap(const TestDma_t &d, uint32_t Ring) { DmaSetCnt(d, Ring); }
static bool DmaCirc(const TestDma_t &d, uint32_t Ring, uint32_t Base) { return (Rd(Ch(d)->CCR) & DMA_CCR_CIRC) != 0 && Rd(Ch(d)->CMAR) == Base; }
static void DmaRoute(const TestDma_t &d, bool bTx)
{
	CHECK(((Rd(Ch(d)->CCR) & DMA_CCR_DIR) != 0) == bTx);
	CHECK((Rd(Ch(d)->CCR) & DMA_CCR_MINC) != 0);
#if defined(DMAMUX1)
	// DMA2 channel 1 is on the DMAMUX1 channel after the DMA1 ones, RM0432
	// and RM0440
#if defined(DMA1_Channel8)
	uint32_t n = d.Idx + ((DMA_TypeDef *)d.pCtl == DMA1 ? 0U : 8U);
#elif defined(DMA1_Channel7)
	uint32_t n = d.Idx + ((DMA_TypeDef *)d.pCtl == DMA1 ? 0U : 7U);
#else
	uint32_t n = d.Idx + ((DMA_TypeDef *)d.pCtl == DMA1 ? 0U : 6U);
#endif
	CHECK((Rd(DMAMUX1_Channel0[n].CCR) & DMAMUX_CxCR_DMAREQ_ID) == (uint32_t)d.Req);
#elif defined(DMA1_CSELR)
	DMA_Request_TypeDef *sel = (DMA_TypeDef *)d.pCtl == DMA1 ? DMA1_CSELR : DMA2_CSELR;
	CHECK(((Rd(sel->CSELR) >> (4 * d.Idx)) & 0xFU) == (uint32_t)d.Req);
#elif defined(DMA_CSELR_C1S_Msk)
	CHECK(((Rd(((DMA_TypeDef *)d.pCtl)->CSELR) >> (4 * d.Idx)) & 0xFU) == (uint32_t)d.Req);
#endif
}
static void DmaInUse(const TestDma_t &d) { Wr(Ch(d)->CCR, DMA_CCR_MINC); }
#endif

// ---- Test helpers -------------------------------------------------------------

static const IOPinCfg_t s_Pins[] = {
	{ 0, 10, 1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 9, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
};

static UARTDev_t s_Uart[MAX_INST];
static int s_TxReadyCnt;
static int s_RxDataCnt;
static uint32_t s_Ring[MAX_INST];
static uint32_t s_RingBase[MAX_INST];

static int EvtHandler(UARTDev_t * const pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	if (EvtId == UART_EVT_TXREADY)
	{
		s_TxReadyCnt++;
	}
	else if (EvtId == UART_EVT_RXDATA)
	{
		s_RxDataCnt++;
	}

	return 0;
}

static UARTCfg_t Cfg(int DevNo, int Rate, bool bInt, bool bDma)
{
	UARTCfg_t c;

	memset(&c, 0, sizeof(c));
	c.DevNo = DevNo;
	c.pIOPinMap = s_Pins;
	c.NbIOPins = 2;
	c.Rate = Rate;
	c.DataBits = 8;
	c.Parity = UART_PARITY_NONE;
	c.StopBits = 1;
	c.FlowControl = UART_FLWCTRL_NONE;
	c.bIntMode = bInt;
	c.IntPrio = 3;
	c.EvtCallback = EvtHandler;
	c.bFifoBlocking = true;
	c.bDMAMode = bDma;
	c.Duplex = UART_DUPLEX_FULL;
	c.Mode = UART_MODE_UART;

	return c;
}

// The rate an instance makes from fck with oversampling by 16, BRR = fck / rate
static uint32_t Rate16(uint32_t Fck, uint32_t Rate)
{
	uint32_t div = (Fck + Rate / 2) / Rate;

	return (Fck + div / 2) / div;
}

// LPUART BRR = 256 x fck / rate, 0x300 to 0xFFFFF
static uint32_t LpBrr(uint32_t Fck, uint32_t Rate)
{
	uint64_t f = (uint64_t)Fck << 8;
	uint64_t div = (f + Rate / 2) / Rate;

	if (div < 0x300)
	{
		div = 0x300;
	}
	else if (div > 0xFFFFF)
	{
		div = 0xFFFFF;
	}

	return (uint32_t)div;
}

static uint32_t LpRate(uint32_t Fck, uint32_t Brr)
{
	return (uint32_t)((((uint64_t)Fck << 8) + Brr / 2) / Brr);
}

static uint32_t SelVal(const TestInst_t &t)
{
	return (Rd(*t.pSelReg) & t.SelMsk) / (t.SelMsk & (~t.SelMsk + 1));
}

// TX DMA block moved to the wire, then the last character leaves the line:
// TC set. Returns the bytes moved.
static int DmaTxRun(int i)
{
	const TestDma_t &d = s_Inst[i].Tx;

	if (!DmaOn(d))
	{
		return 0;
	}

	uint32_t n = DmaCnt(d);

	if (n == 0)
	{
		return 0;
	}

	const uint8_t *p = (const uint8_t *)(uintptr_t)DmaMem(d, true);

	for (uint32_t k = 0; k < n; k++)
	{
		WirePut(i, p[k]);
	}
	DmaSetCnt(d, 0);
	DmaTxDone(d, n);
	DmaFlags(d, F_TC | F_HT);
	DmaEnd(d);
	Set(T_ISR(s_Inst[i].pReg), T_TC);

	return (int)n;
}

// RX bytes written by the DMA in the ring. The half and full flags call the
// RX DMA vector in interrupt mode, as the NVIC would with HTIE and TCIE.
static void DmaRxPut(int i, const uint8_t *p, int Len, bool bInt)
{
	const TestDma_t &d = s_Inst[i].Rx;
	uint32_t ring = s_Ring[i];

	for (int k = 0; k < Len; k++)
	{
		if (!DmaOn(d))
		{
			// Channel stopped, the bytes are lost
			return;
		}

		uint8_t *dst = DmaRxPos(d, ring);

		if ((uintptr_t)dst < s_RingBase[i] || (uintptr_t)dst >= s_RingBase[i] + ring)
		{
			CHECK((uintptr_t)dst >= s_RingBase[i] && (uintptr_t)dst < s_RingBase[i] + ring);
			return;
		}
		*dst = p[k];

		uint32_t cnt = DmaCnt(d) - 1;
		uint32_t f = cnt == ring / 2 ? F_HT : cnt == 0 ? F_TC : 0U;

		DmaSetCnt(d, cnt);
		if (cnt == 0)
		{
			DmaRxWrap(d, ring);
		}
		if (f)
		{
			DmaFlags(d, f);
			if (bInt && (DmaInts(d) & f))
			{
				d.Vector();
			}
		}
	}
}

// End of an RX burst: IDLE, the USART interrupt clears it
static void RxIdle(int i)
{
	Set(T_ISR(s_Inst[i].pReg), T_IDLE);
	s_Inst[i].Vector();
	CHECK((Rd(T_ISR(s_Inst[i].pReg)) & T_IDLE) == 0);
}

static int RxAll(int i, uint8_t *pBuff, int Max)
{
	int cnt = 0;

	while (cnt < Max)
	{
		int l = UARTRx(&s_Uart[i], pBuff + cnt, Max - cnt);
		if (l <= 0)
		{
			break;
		}
		cnt += l;
	}

	return cnt;
}

// False after 50 calls without a change of Val: the loop is stuck
static bool Progress(int Val)
{
	static int s_Last = -1;
	static int s_Same;

	if (Val != s_Last)
	{
		s_Last = Val;
		s_Same = 0;

		return true;
	}

	return ++s_Same < 50;
}

// ---- Tests -------------------------------------------------------------------

// Polling: clocks, kernel clock selection, rate on the instance bus, TX and RX
static void TestPolling(int i)
{
	TestInst_t &t = s_Inst[i];
	USART_TypeDef *reg = t.pReg;
	UARTDev_t &u = s_Uart[i];

	if (t.pSelReg != NULL && !t.bLpuart)
	{
		Set(*t.pSelReg, t.SelMsk);
	}
	s_RstCnt[i] = 0;
	memset(s_NvicEn, 0, sizeof(s_NvicEn));
	s_PinCfgCnt = 0;

	UARTCfg_t c = Cfg(i, 9600, false, false);
	CHECK(UARTInit(&u, &c));
	CHECK(UARTGetInstance(i) == &u);
	CHECK(Rd(*t.pEnReg) & t.Bit);
	CHECK(s_RstCnt[i] == 1 && (Rd(*t.pRstReg) & t.Bit) == 0);
	if (t.pSelReg != NULL && !t.bLpuart)
	{
		CHECK(SelVal(t) == 0);
	}
	CHECK(s_PinCfgCnt == 2);
	CHECK((Rd(reg->CR1) & (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE)) == (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE));
	CHECK((Rd(reg->CR1) & (T_RXNEIE | T_TXEIE | USART_CR1_IDLEIE | USART_CR1_TCIE)) == 0);
	CHECK(!s_NvicEn[t.IrqNo]);
	if (!t.bLpuart)
	{
		uint32_t fck = s_Pclk[t.PclkIdx];
		CHECK(Rd(reg->BRR) == fck / 9600);
		CHECK((uint32_t)UARTGetRate(&u) == Rate16(fck, 9600));
		CHECK((Rd(reg->CR1) & USART_CR1_OVER8) == 0);
	}

	// TXE stays set: TxData writes the CFIFO bytes to TDR at once
	WireReset(i);
	uint8_t tx[5] = { 'h', 'e', 'l', 'l', 'o' };
	CHECK(UARTTx(&u, tx, 5) == 5);
	CHECK(WireIs(i, tx, 5));

	// RxData reads RDR while RXNE is set
	uint8_t rx[16];
	RxQReset(i);
	RxSend(i, (const uint8_t *)"polled", 6);
	CHECK(RxAll(i, rx, 16) == 6 && memcmp(rx, "polled", 6) == 0);
	CHECK(UARTRx(&u, rx, 16) == 0);

	// An error character is dropped, ORE keeps the character in RDR
	Unprot();
	T_RDR(reg) = 0x55;
	T_ISR(reg) |= T_RXNE | T_FE;
	Prot();
	CHECK(UARTRx(&u, rx, 16) == 0);
	CHECK(u.FramErrCnt == 1);
	CHECK((Rd(T_ISR(reg)) & (T_RXNE | T_FE)) == 0);

	s_PinDisCnt = 0;
	UARTDisable(&u);
	CHECK((Rd(*t.pEnReg) & t.Bit) == 0);
	CHECK((Rd(reg->CR1) & USART_CR1_UE) == 0);
	CHECK(s_PinDisCnt == 2);
	UARTEnable(&u);
	CHECK(Rd(*t.pEnReg) & t.Bit);
	CHECK(Rd(reg->CR1) & USART_CR1_UE);
}

// Interrupt mode: vector, NVIC, RX and TX through the handler, RX errors,
// full RX FIFO
static void TestInterrupt(int i)
{
	TestInst_t &t = s_Inst[i];
	USART_TypeDef *reg = t.pReg;
	UARTDev_t &u = s_Uart[i];
	uint8_t rx[32];

	memset(s_NvicEn, 0, sizeof(s_NvicEn));
	UARTCfg_t c = Cfg(i, 115200, true, false);
	CHECK(UARTInit(&u, &c));
	CHECK(s_NvicEn[t.IrqNo] && s_NvicPrio[t.IrqNo] == 3);
	CHECK(Rd(reg->CR1) & T_RXNEIE);
	CHECK(Rd(reg->CR3) & USART_CR3_EIE);
	CHECK((Rd(reg->CR3) & (USART_CR3_DMAT | USART_CR3_DMAR)) == 0);

	// RX through the vector
	RxQReset(i);
	s_RxDataCnt = 0;
	RxSend(i, (const uint8_t *)"ab", 2);
	t.Vector();
	t.Vector();
	CHECK(s_RxDataCnt == 2);
	CHECK(RxAll(i, rx, 32) == 2 && rx[0] == 'a' && rx[1] == 'b');

	// FE: dropped. ORE: counted, the character in RDR kept.
	Unprot();
	T_RDR(reg) = 0x55;
	T_ISR(reg) |= T_RXNE | T_FE;
	Prot();
	t.Vector();
	CHECK(u.FramErrCnt == 1);
	CHECK((Rd(T_ISR(reg)) & (T_RXNE | T_FE)) == 0);
	CHECK(UARTRx(&u, rx, 32) == 0);
	Unprot();
	T_RDR(reg) = 0x66;
	T_ISR(reg) |= T_RXNE | T_ORE;
	Prot();
	t.Vector();
	CHECK(u.RxOvrErrCnt == 1);
	CHECK(UARTRx(&u, rx, 32) == 1 && rx[0] == 0x66);
	CHECK((Rd(T_ISR(reg)) & (T_RXNE | T_RXERR)) == 0);

	// Blocking FIFO full: the new characters are dropped and counted
	uint8_t many[20];
	for (int k = 0; k < 20; k++)
	{
		many[k] = (uint8_t)(0x30 + k);
	}
	RxQReset(i);
	RxSend(i, many, 20);
	for (int k = 0; k < 20; k++)
	{
		t.Vector();
	}
	int n = RxAll(i, rx, 32);
	CHECK(n + (int)u.RxDropCnt == 20 && u.RxDropCnt > 0);
	CHECK(n > 0 && memcmp(rx, many, n) == 0);

	// TX: the first byte goes to TDR, TXE interrupts send the rest
	WireReset(i);
	s_TxReadyCnt = 0;
	uint8_t tx[5] = { '1', '2', '3', '4', '5' };
	CHECK(UARTTx(&u, tx, 5) == 5);
	CHECK(WireIs(i, tx, 1));
	CHECK(Rd(reg->CR1) & T_TXEIE);
	for (int k = 0; k < 5; k++)
	{
		t.Vector();
	}
	CHECK(WireIs(i, tx, 5));
	CHECK((Rd(reg->CR1) & T_TXEIE) == 0);
	CHECK(s_TxReadyCnt == 1 && u.bTxReady);

	// 7 data bits: the 8th bit is cleared
	c.DataBits = 7;
	c.Parity = UART_PARITY_EVEN;
	CHECK(UARTInit(&u, &c));
	RxQReset(i);
	uint8_t ff = 0xFF;
	RxSend(i, &ff, 1);
	t.Vector();
	CHECK(UARTRx(&u, rx, 32) == 1 && rx[0] == 0x7F);
}

// DMA: route, setup, TX blocks, RX ring, error, polled DMA, Disable and
// Enable, channel in use, interrupt mode after DMA
static void TestDma(int i)
{
	TestInst_t &t = s_Inst[i];
	USART_TypeDef *reg = t.pReg;
	UARTDev_t &u = s_Uart[i];
	UARTCfg_t c = Cfg(i, 115200, true, true);

	if (t.Tx.pChan == NULL)
	{
		CHECK(!UARTInit(&u, &c));
		c.bIntMode = false;
		CHECK(!UARTInit(&u, &c));
		return;
	}

#ifdef SYSCFG_CFGR1_USART1RX_DMA_RMP
	if (reg == USART1)
	{
		Wr(SYSCFG->CFGR1, SYSCFG_CFGR1_USART1TX_DMA_RMP | SYSCFG_CFGR1_USART1RX_DMA_RMP | SYSCFG_CFGR1_ADC_DMA_RMP);
	}
#endif
	memset(s_NvicEn, 0, sizeof(s_NvicEn));
	CHECK(UARTInit(&u, &c));
#ifdef SYSCFG_CFGR1_USART1RX_DMA_RMP
	if (reg == USART1)
	{
		CHECK(Rd(SYSCFG->CFGR1) == SYSCFG_CFGR1_ADC_DMA_RMP);
	}
#endif
	DmaRoute(t.Rx, false);
	DmaRoute(t.Tx, true);
	CHECK(DmaPeriph(t.Rx, false) == (uint32_t)(uintptr_t)&T_RDR(reg));
	CHECK(DmaPeriph(t.Tx, true) == (uint32_t)(uintptr_t)&T_TDR(reg));
	CHECK(DmaOn(t.Rx) && !DmaOn(t.Tx));
	s_Ring[i] = DmaCnt(t.Rx);
	s_RingBase[i] = DmaMem(t.Rx, false);
	CHECK(s_Ring[i] >= 16 && (s_Ring[i] & (s_Ring[i] - 1)) == 0);
	CHECK(DmaCirc(t.Rx, s_Ring[i], s_RingBase[i]));
	CHECK(DmaInts(t.Rx) == (F_HT | F_TC));
	CHECK(DmaInts(t.Tx) == F_TE);
	CHECK(s_NvicEn[t.IrqNo] && s_NvicEn[t.Rx.IrqNo] && s_NvicEn[t.Tx.IrqNo]);
	CHECK(s_NvicPrio[t.Rx.IrqNo] == 3 && s_NvicPrio[t.Tx.IrqNo] == 3);
	CHECK((Rd(reg->CR3) & (USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE)) == (USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE));
	CHECK((Rd(reg->CR1) & (T_RXNEIE | USART_CR1_IDLEIE)) == USART_CR1_IDLEIE);

	// TX: blocks from the FIFO, the next one started by the TC interrupt.
	// The caller buffer may change once Tx returns.
	static uint8_t data[1500];
	for (int k = 0; k < 1500; k++)
	{
		data[k] = (uint8_t)(k * 73 + 19);
	}
	uint8_t buf[64];
	WireReset(i);
	s_TxReadyCnt = 0;
	int sent = 0;
	for (int guard = 0; guard < 5000 && (sent < 300 || !u.bTxReady) && Progress(sent + s_WireLen[i]); guard++)
	{
		if (sent < 300)
		{
			int l = 300 - sent < 64 ? 300 - sent : 64;
			memcpy(buf, data + sent, l);
			int n = UARTTx(&u, buf, l);
			memset(buf, 0, sizeof(buf));
			sent += n;
		}
		if (DmaTxRun(i) > 0)
		{
			t.Vector();
		}
	}
	CHECK(WireIs(i, data, 300));
	CHECK(s_TxReadyCnt >= 1 && u.bTxReady);
	CHECK((Rd(reg->CR1) & USART_CR1_TCIE) == 0);

	// One byte at a time, FIFO wrap and back pressure
	WireReset(i);
	sent = 0;
	for (int guard = 0; guard < 100000 && (sent < 1500 || !u.bTxReady) && Progress(sent + s_WireLen[i]); guard++)
	{
		if (sent < 1500 && UARTTx(&u, (uint8_t *)&data[sent], 1) == 1)
		{
			sent++;
			if (sent % 11)
			{
				continue;
			}
		}
		if (DmaTxRun(i) > 0)
		{
			t.Vector();
		}
	}
	CHECK(WireIs(i, data, 1500));

	// Transfer error: the bytes of the block not sent are counted
	WireReset(i);
	CHECK(UARTTx(&u, data, 10) == 10);
	uint32_t left = DmaCnt(t.Tx);
	CHECK(left > 0 && left <= 10);
	DmaFlags(t.Tx, F_TE);
	DmaStopErr(t.Tx);
	t.Tx.Vector();
	CHECK(u.TxDropCnt == left);
	CHECK((DmaGetFlags(t.Tx) & F_TE) == 0);
	for (int guard = 0; guard < 100 && !u.bTxReady; guard++)
	{
		if (DmaTxRun(i) > 0)
		{
			t.Vector();
		}
	}
	CHECK(u.bTxReady && WireIs(i, data + left, 10 - left));

	// RX ring: bursts, half and full interrupts, IDLE at the end of each
	static uint8_t rxd[1200];
	uint8_t got[1200];
	int total = 0, recv = 0;
	for (int k = 0; k < 1200; k++)
	{
		rxd[k] = (uint8_t)(k * 31 + 7);
	}
	s_RxDataCnt = 0;
	for (int b = 1; total < 1200; b = b % 45 + 1)
	{
		int l = 1200 - total < b ? 1200 - total : b;
		DmaRxPut(i, rxd + total, l, true);
		total += l;
		RxIdle(i);
		recv += RxAll(i, got + recv, 1200 - recv);
	}
	CHECK(recv == 1200 && memcmp(got, rxd, 1200) == 0);
	CHECK(s_RxDataCnt > 0 && u.RxDropCnt == 0);
	CHECK(DmaOn(t.Rx) && DmaCirc(t.Rx, s_Ring[i], s_RingBase[i]));

	// RX error with the character taken by the DMA: counted, flag cleared
	Set(T_ISR(reg), T_FE);
	t.Vector();
	CHECK(u.FramErrCnt == 1);
	CHECK((Rd(T_ISR(reg)) & T_FE) == 0);

	// Disable stops both channels, Enable sets them up again
	CHECK(UARTTx(&u, data, 20) == 20);
	CHECK(DmaOn(t.Tx));
	UARTDisable(&u);
	CHECK(DmaCtrl(t.Tx) == 0 && !DmaOn(t.Rx));
	CHECK((Rd(reg->CR3) & (USART_CR3_DMAT | USART_CR3_DMAR)) == 0);
	CHECK((Rd(*t.pEnReg) & t.Bit) == 0);
	UARTEnable(&u);
	CHECK(DmaOn(t.Rx) && DmaCnt(t.Rx) == s_Ring[i] && !DmaOn(t.Tx));
	CHECK(CFifoUsed(u.hTxFifo) == 0 && u.bTxReady);
	CHECK((Rd(reg->CR3) & (USART_CR3_DMAT | USART_CR3_DMAR)) == (USART_CR3_DMAT | USART_CR3_DMAR));
	// The half ring interrupt moves the first half, its flag cleared
	WireReset(i);
	s_RxDataCnt = 0;
	DmaRxPut(i, rxd, s_Ring[i] / 2 + 8, true);
	CHECK(s_RxDataCnt == 1 && CFifoUsed(u.hRxFifo) == (int)s_Ring[i] / 2);
	CHECK(DmaGetFlags(t.Rx) == 0);
	RxIdle(i);
	recv = RxAll(i, got, 100);
	CHECK(recv == (int)s_Ring[i] / 2 + 8 && memcmp(got, rxd, recv) == 0);

	// Polled DMA: TxData and RxData service the channels
	c.bIntMode = false;
	memset(s_NvicEn, 0, sizeof(s_NvicEn));
	CHECK(UARTInit(&u, &c));
	CHECK(!s_NvicEn[t.Rx.IrqNo] && !s_NvicEn[t.IrqNo]);
	CHECK(DmaInts(t.Rx) == 0 && DmaInts(t.Tx) == 0);
	CHECK((Rd(reg->CR1) & (T_RXNEIE | USART_CR1_IDLEIE)) == 0);
	WireReset(i);
	sent = 0;
	recv = 0;
	total = 0;
	for (int guard = 0; guard < 5000 && (sent < 600 || recv < 600 || !u.bTxReady) &&
		 Progress(sent + recv + total + s_WireLen[i]); guard++)
	{
		if (sent < 600)
		{
			int l = 600 - sent < 50 ? 600 - sent : 50;
			sent += UARTTx(&u, (uint8_t *)data + sent, l);
		}
		if (total < 600)
		{
			int l = 600 - total < 37 ? 600 - total : 37;
			DmaRxPut(i, rxd + total, l, false);
			total += l;
		}
		recv += RxAll(i, got + recv, 600 - recv);
		DmaTxRun(i);
	}
	CHECK(WireIs(i, data, 600));
	CHECK(recv == 600 && memcmp(got, rxd, 600) == 0);

	// A channel set up by another driver is not taken
	UARTDisable(&u);
	UARTDev_t other = {};
	DmaInUse(t.Tx);
	uint32_t ctrl = DmaCtrl(t.Tx);
	CHECK(!UARTInit(&other, &c));
	CHECK(DmaCtrl(t.Tx) == ctrl);
	DmaSetCtrl(t.Tx, 0);

	// Interrupt mode after DMA: both channels stopped, RXNE interrupt
	c.bIntMode = true;
	CHECK(UARTInit(&u, &c));
	CHECK(UARTTx(&u, data, 8) == 8);
	CHECK(DmaOn(t.Tx));
	c.bDMAMode = false;
	CHECK(UARTInit(&u, &c));
	CHECK(DmaCtrl(t.Tx) == 0 && !DmaOn(t.Rx));
	CHECK((Rd(reg->CR3) & (USART_CR3_DMAT | USART_CR3_DMAR)) == 0);
	CHECK((Rd(reg->CR1) & (T_RXNEIE | USART_CR1_IDLEIE)) == T_RXNEIE);
	// A late DMA interrupt does nothing
	DmaFlags(t.Rx, F_TC);
	t.Rx.Vector();
	CHECK(!DmaOn(t.Tx) && u.bTxReady);
}

// Interrupt mode: TX DMA blocks run to the end, the TC interrupt starts the
// next one
static void DmaTxDrain(int i)
{
	for (int guard = 0; guard < 1000 && !s_Uart[i].bTxReady && DmaTxRun(i) > 0; guard++)
	{
		s_Inst[i].Vector();
	}
}

// DMA, blocking and non-blocking CFIFO. Blocking: the block is read in place.
// Non-blocking: the block is a copy taken out of the FIFO, a put on a full
// FIFO drops the oldest bytes still in the FIFO, the running block is sent as
// it was. A full RX FIFO drops its oldest bytes.
static void TestDmaDrop(int i)
{
	TestInst_t &t = s_Inst[i];
	UARTDev_t &u = s_Uart[i];

	if (t.Tx.pChan == NULL)
	{
		return;
	}

	static uint8_t data[1500];
	static uint8_t expect[600];
	for (int k = 0; k < 1500; k++)
	{
		data[k] = (uint8_t)(k * 57 + 3);
	}

	// Blocking: in place from the start of the FIFO, cut to a multiple of 4
	UARTCfg_t c = Cfg(i, 115200, true, true);
	CHECK(UARTInit(&u, &c));
	WireReset(i);
	CHECK(UARTTx(&u, data, 10) == 10);
	CHECK(DmaMem(t.Tx, true) == (uint32_t)(uintptr_t)u.hTxFifo->pMemStart && DmaCnt(t.Tx) == 8);
	DmaTxDrain(i);
	CHECK(WireIs(i, data, 10));

	c.bFifoBlocking = false;
	CHECK(UARTInit(&u, &c));
	CHECK(!CFifoIsBlocking(u.hTxFifo) && !CFifoIsBlocking(u.hRxFifo));

	// The block is out of the FIFO, the DMA reads an aligned copy
	WireReset(i);
	s_TxReadyCnt = 0;
	CHECK(UARTTx(&u, data, 10) == 10);
	uint32_t mem = DmaMem(t.Tx, true);
	uintptr_t fbeg = (uintptr_t)u.hTxFifo->pMemStart;
	uintptr_t fend = fbeg + (uintptr_t)u.hTxFifo->MaxIdxCnt;
	CHECK(DmaOn(t.Tx) && DmaCnt(t.Tx) == 10 && (mem & 3U) == 0);
	CHECK(mem + 10 <= fbeg || mem >= fend);
	CHECK(CFifoUsed(u.hTxFifo) == 0 && !u.bTxReady);

	// The FIFO filled past its size while the block runs: Tx takes all,
	// the oldest queued bytes are dropped, the block goes out unchanged
	int size = CFifoAvail(u.hTxFifo);
	int over = 40;
	CHECK(size > 64 && size + over <= 600);
	CHECK(UARTTx(&u, data + 100, size + over) == size + over);
	CHECK(u.hTxFifo->DropCnt == (uint32_t)over && CFifoUsed(u.hTxFifo) == size);
	DmaTxDrain(i);
	memcpy(expect, data, 10);
	memcpy(expect + 10, data + 100 + over, size);
	CHECK(WireIs(i, expect, 10 + size));
	CHECK(u.bTxReady && s_TxReadyCnt == 1);
	CHECK((Rd(s_Inst[i].pReg->CR1) & USART_CR1_TCIE) == 0);

	// A stream across the FIFO wrap, no drop: at most 32 bytes per block
	WireReset(i);
	int sent = 0;
	for (int guard = 0; guard < 5000 && (sent < 1500 || !u.bTxReady) && Progress(sent + s_WireLen[i]); guard++)
	{
		if (sent < 1500)
		{
			int l = 1500 - sent < 23 ? 1500 - sent : 23;
			CHECK(UARTTx(&u, data + sent, l) == l);
			sent += l;
		}
		uint32_t n = DmaOn(t.Tx) ? DmaCnt(t.Tx) : 0;
		CHECK(n <= 32);
		if (DmaTxRun(i) > 0)
		{
			t.Vector();
		}
	}
	CHECK(WireIs(i, data, 1500));
	CHECK(u.hTxFifo->DropCnt == (uint32_t)over);

	// Transfer error: the copy not sent is counted
	WireReset(i);
	u.TxDropCnt = 0;
	CHECK(UARTTx(&u, data, 10) == 10);
	CHECK(DmaCnt(t.Tx) == 10);
	DmaFlags(t.Tx, F_TE);
	DmaStopErr(t.Tx);
	t.Tx.Vector();
	CHECK(u.TxDropCnt == 10 && u.bTxReady && s_WireLen[i] == 0);

	// RX FIFO full: the oldest bytes are dropped, the newest kept
	static uint8_t rxd[300];
	uint8_t got[300];
	for (int k = 0; k < 300; k++)
	{
		rxd[k] = (uint8_t)(k * 13 + 5);
	}
	int rsize = CFifoAvail(u.hRxFifo);
	CHECK(rsize < 300);
	for (int total = 0; total < 300; total += 25)
	{
		DmaRxPut(i, rxd + total, 25, true);
		RxIdle(i);
	}
	CHECK(RxAll(i, got, 300) == rsize && memcmp(got, rxd + 300 - rsize, rsize) == 0);
	CHECK(u.RxDropCnt == 0 && u.hRxFifo->DropCnt == (uint32_t)(300 - rsize));

	// Polled DMA: TxData and RxData service the channels
	c.bIntMode = false;
	CHECK(UARTInit(&u, &c));
	WireReset(i);
	sent = 0;
	int recv = 0, total = 0;
	for (int guard = 0; guard < 5000 && (sent < 600 || recv < 300 || !u.bTxReady) &&
		 Progress(sent + recv + total + s_WireLen[i]); guard++)
	{
		if (sent < 600)
		{
			int l = 600 - sent < 25 ? 600 - sent : 25;
			CHECK(UARTTx(&u, data + sent, l) == l);
			sent += l;
		}
		if (total < 300)
		{
			int l = 300 - total < 19 ? 300 - total : 19;
			DmaRxPut(i, rxd + total, l, false);
			total += l;
		}
		if (recv < 300)
		{
			recv += RxAll(i, got + recv, 300 - recv);
		}
		else
		{
			// RxData services TX too
			uint8_t b;
			CHECK(UARTRx(&u, &b, 1) == 0);
		}
		DmaTxRun(i);
	}
	CHECK(WireIs(i, data, 600));
	CHECK(recv == 300 && memcmp(got, rxd, 300) == 0);
	CHECK(u.hTxFifo->DropCnt == 0 && u.hRxFifo->DropCnt == 0);
}

#if defined(STM32WBA) || defined(STM32L4)
static bool TestDmaClaim(int Ch, void *pCtx)
{
#if defined(STM32WBA)
	return WbaGpdmaClaim(Ch, NULL, pCtx, 6, false);
#else
	return Stm32DmaChClaim(Ch, NULL, pCtx, 6, false);
#endif
}

static void TestDmaFree(int Ch)
{
#if defined(STM32WBA)
	WbaGpdmaFree(Ch);
#else
	Stm32DmaChFree(Ch);
#endif
}

static int TestDmaCh(const TestDma_t &Dma)
{
#if defined(STM32WBA)
	return Dma.Idx;
#else
	return Dma.Idx + (Dma.pCtl == DMA1 ? 0 : STM32_DMA_CTRL_NBCH);
#endif
}

static void CheckAllDmaFree(void)
{
#if defined(STM32WBA)
	const int nb = WBA_GPDMA_NBCH;
#else
	const int nb = STM32_DMA_NBCH;
#endif

	for (int ch = 0; ch < nb; ch++)
	{
		bool claimed = TestDmaClaim(ch, &s_CheckCnt);

		CHECK(claimed);
		if (claimed)
		{
			TestDmaFree(ch);
		}
	}
}

// Run before successful initialization reserves any of the shared channels.
static void TestInitFailure(void)
{
	int i = 0;
	UARTCfg_t c;
	uint8_t tiny[1];
	uint32_t clock;
	int tx;

	// LPUART can select HSI16 independently of PCLK. Use a USART for
	// the deliberately unusable-clock cases.
	while (s_Inst[i].bLpuart)
	{
		i++;
	}
	c = Cfg(i, 115200, true, true);
	clock = s_Pclk[s_Inst[i].PclkIdx];
	tx = TestDmaCh(s_Inst[i].Tx);

	c.pRxMem = tiny;
	c.RxMemSize = sizeof(tiny);
	CHECK(!UARTInit(&s_Uart[i], &c));
	CheckAllDmaFree();
	c = Cfg(i, 115200, true, true);
	c.pTxMem = tiny;
	c.TxMemSize = sizeof(tiny);
	CHECK(!UARTInit(&s_Uart[i], &c));
	CheckAllDmaFree();
	c = Cfg(i, 115200, true, true);

	// Both the unavailable clock and a clock whose baud rounds to zero
	// fail after the channels have been claimed.
	for (uint32_t hz = 0; hz < 2; hz++)
	{
		s_Pclk[s_Inst[i].PclkIdx] = hz;
		CHECK(!UARTInit(&s_Uart[i], &c));
		CheckAllDmaFree();
	}
	s_Pclk[s_Inst[i].PclkIdx] = clock;

	// Failure of the second claim returns the first channel to the pool.
	CHECK(TestDmaClaim(tx, &c));
	CHECK(!UARTInit(&s_Uart[i], &c));
	TestDmaFree(tx);
	CheckAllDmaFree();
}

static void TestReinitFailure(int i)
{
	UARTCfg_t c = Cfg(i, 115200, true, true);
	UARTDev_t &u = s_Uart[i];
	uint8_t tiny[1];
	hCFifo_t rx = u.hRxFifo;
	hCFifo_t tx = u.hTxFifo;
	uint32_t cr1 = Rd(s_Inst[i].pReg->CR1);
	uint32_t clock = s_Pclk[s_Inst[i].PclkIdx];

	c.pRxMem = tiny;
	c.RxMemSize = sizeof(tiny);
	CHECK(!UARTInit(&u, &c));
	CHECK(u.hRxFifo == rx && u.hTxFifo == tx);
	CHECK(Rd(s_Inst[i].pReg->CR1) == cr1);

	// A clock failure on the same object keeps its existing reservation.
	c = Cfg(i, 115200, true, true);
	s_Pclk[s_Inst[i].PclkIdx] = 0;
	CHECK(!UARTInit(&u, &c));
	CHECK(!TestDmaClaim(TestDmaCh(s_Inst[i].Rx), &c));
	CHECK(!TestDmaClaim(TestDmaCh(s_Inst[i].Tx), &c));
	s_Pclk[s_Inst[i].PclkIdx] = clock;
	CHECK(UARTInit(&u, &c));
}
#endif

// Framing on the first USART, not LPUART
static void TestFraming(int i)
{
	USART_TypeDef *reg = s_Inst[i].pReg;
	UARTDev_t &u = s_Uart[i];
	static const UART_PARITY par[3] = { UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD };

	for (int bits = 7; bits <= 8; bits++)
	{
		for (int p = 0; p < 3; p++)
		{
			for (int stop = 1; stop <= 2; stop++)
			{
				UARTCfg_t c = Cfg(i, 115200, false, false);
				c.DataBits = bits;
				c.Parity = par[p];
				c.StopBits = stop;

				int word = bits + (p != 0 ? 1 : 0);
				bool ok = UARTInit(&u, &c);
#ifndef USART_CR1_M1
				if (word == 7)
				{
					CHECK(!ok);
					continue;
				}
#endif
				CHECK(ok);
				uint32_t cr1 = Rd(reg->CR1);
				CHECK(((cr1 & USART_CR1_PCE) != 0) == (p != 0));
				CHECK(((cr1 & USART_CR1_PS) != 0) == (p == 2));
				CHECK(((cr1 & T_M0) != 0) == (word == 9));
#ifdef USART_CR1_M1
				CHECK(((cr1 & USART_CR1_M1) != 0) == (word == 7));
#endif
				CHECK((Rd(reg->CR2) & USART_CR2_STOP) == (stop == 2 ? USART_CR2_STOP_1 : 0U));
			}
		}
	}

	UARTCfg_t c = Cfg(i, 115200, false, false);
	c.DataBits = 9;
	CHECK(!UARTInit(&u, &c));
	c.DataBits = 6;
	CHECK(!UARTInit(&u, &c));
	c.DataBits = 8;
	c.Rate = 0;
	CHECK(!UARTInit(&u, &c));
	c.Rate = 115200;
	c.DevNo = s_NbInst;
	CHECK(!UARTInit(&u, &c));
}

// LPUART1 kernel clock: LSE crystal, HSI16, then PCLK for the rates above
// HSI16 / 3
static void TestLpuart(int i)
{
	TestInst_t &t = s_Inst[i];
	USART_TypeDef *reg = t.pReg;
	UARTDev_t &u = s_Uart[i];

	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_XTAL;
	UARTCfg_t c = Cfg(i, 9600, false, false);
	CHECK(UARTInit(&u, &c));
	CHECK(SelVal(t) == 3);
	CHECK(Rd(reg->BRR) == LpBrr(32768, 9600));
	CHECK((uint32_t)UARTGetRate(&u) == LpRate(32768, LpBrr(32768, 9600)));

	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_RC;
	CHECK(UARTInit(&u, &c));
	CHECK(SelVal(t) == 2);
	CHECK(Rd(reg->BRR) == LpBrr(16000000, 9600));
	CHECK((uint32_t)UARTGetRate(&u) == LpRate(16000000, LpBrr(16000000, 9600)));

	c.Rate = 6000000;
	CHECK(UARTInit(&u, &c));
	uint32_t fck = s_Pclk[t.PclkIdx];
	CHECK(SelVal(t) == 0);
	CHECK(Rd(reg->BRR) == LpBrr(fck, 6000000));
	CHECK((uint32_t)UARTGetRate(&u) == LpRate(fck, LpBrr(fck, 6000000)));
}

int main()
{
	struct sigaction sa;

	MapRange(0x40000000, 0x30000);
#if defined(STM32WBA)
	MapRange(0x46000000, 0x30000);
#endif

	BuildInst();

	for (int i = 0; i < s_NbInst; i++)
	{
		AddPage((uintptr_t)s_Inst[i].pReg);
		AddPage((uintptr_t)s_Inst[i].pRstReg);
		if (s_Inst[i].Rx.pCtl)
		{
			AddPage((uintptr_t)s_Inst[i].Rx.pCtl);
		}
	}

	memset(&sa, 0, sizeof(sa));
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = OnSegv;
	sigaction(SIGSEGV, &sa, NULL);
	sa.sa_sigaction = OnTrap;
	sigaction(SIGTRAP, &sa, NULL);

	// TXE set, HSI16 ready
	for (int i = 0; i < s_NbInst; i++)
	{
		T_ISR(s_Inst[i].pReg) = T_TXE | T_TC;
	}
	RCC->CR |= RCC_CR_HSIRDY;
	Prot();

#if defined(STM32WBA) || defined(STM32L4)
	TestInitFailure();
#endif

	int first = -1;
	for (int i = 0; i < s_NbInst; i++)
	{
		s_pName = s_Inst[i].pName;
		if (first < 0 && !s_Inst[i].bLpuart)
		{
			first = i;
		}
		TestPolling(i);
		TestInterrupt(i);
		TestDma(i);
		TestDmaDrop(i);
#if defined(STM32WBA) || defined(STM32L4)
		TestReinitFailure(i);
#endif
		if (s_Inst[i].bLpuart)
		{
			TestLpuart(i);
		}
		UARTDisable(&s_Uart[i]);
	}
	s_pName = s_Inst[first].pName;
	TestFraming(first);

	s_pName = "model";
	CHECK(s_Violation == 0);

	printf("%d instances, %d checks, %u register accesses modelled, %d failures\n", s_NbInst, s_CheckCnt, s_AccCnt, s_FailCnt);

	return s_FailCnt ? 1 : 0;
}
