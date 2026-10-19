/**-------------------------------------------------------------------------
@file	dma_stm32.cpp

@brief	STM32 DMA channel manager, DMA controllers made of channels

		See dma_stm32.h. The manager owns the DMA1 and DMA2 channel interrupt
		vectors of the L4 and L4+ parts and gives each channel to one driver
		at a time.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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
#include <stddef.h>

#include "dma_stm32.h"

// TC and TE of the group of 4 flags of a channel in ISR, GIF is bit 0
#define STM32_DMA_ISR_TC			2U
#define STM32_DMA_ISR_TE			8U

typedef struct {
	Stm32DmaChHandler_t Handler;
	void *pCtx;
	bool bUsed;
} Stm32DmaSlot_t;

static Stm32DmaSlot_t s_Stm32DmaSlot[STM32_DMA_NBCH];
static uint32_t s_Stm32DmaLatch[STM32_DMA_NBCH];	// flags taken by Stm32DmaChAck
static uint8_t s_Stm32DmaWidth[STM32_DMA_NBCH];		// transfer width, log2 of bytes

static DMA_Channel_TypeDef * const s_Stm32DmaCh[STM32_DMA_NBCH] = {
	DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6, DMA1_Channel7,
	DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, DMA2_Channel5, DMA2_Channel6, DMA2_Channel7,
};

static const IRQn_Type s_Stm32DmaIrq[STM32_DMA_NBCH] = {
	DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn, DMA1_Channel4_IRQn,
	DMA1_Channel5_IRQn, DMA1_Channel6_IRQn, DMA1_Channel7_IRQn,
	DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn, DMA2_Channel4_IRQn,
	DMA2_Channel5_IRQn, DMA2_Channel6_IRQn, DMA2_Channel7_IRQn,
};

static inline DMA_TypeDef *Stm32DmaCtrl(int Ch)
{
	return Ch < STM32_DMA_CTRL_NBCH ? DMA1 : DMA2;
}

// Position of the flag group of the channel in ISR and IFCR
static inline uint32_t Stm32DmaShift(int Ch)
{
	return (uint32_t)(Ch % STM32_DMA_CTRL_NBCH) * 4U;
}

static inline uint32_t Stm32DmaIsr(int Ch)
{
	return (Stm32DmaCtrl(Ch)->ISR >> Stm32DmaShift(Ch)) & 0xFU;
}

// Controller clock on, and DMAMUX1 when the part has it
static void Stm32DmaClock(int Ch)
{
	RCC->AHB1ENR |= Stm32DmaCtrl(Ch) == DMA1 ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
#ifdef RCC_AHB1ENR_DMAMUX1EN
	RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
#endif
	(void)RCC->AHB1ENR;
}

// Request routed to the channel: DMAMUX1 (L4+) or CSELR (L4)
static void Stm32DmaRoute(int Ch, uint32_t Req)
{
#if defined(DMAMUX1)
	// DMA1 channel n on DMAMUX1 channel n - 1, DMA2 channel n after the 7 DMA1
	// channels
	DMAMUX1_Channel0[Ch].CCR = Req;
#else
	DMA_Request_TypeDef *sel = Stm32DmaCtrl(Ch) == DMA1 ? DMA1_CSELR : DMA2_CSELR;
	uint32_t pos = Stm32DmaShift(Ch);

	sel->CSELR = (sel->CSELR & ~(DMA_CSELR_C1S_Msk << pos)) | ((Req & DMA_CSELR_C1S_Msk) << pos);
#endif
}

static void Stm32DmaTake(int Ch, Stm32DmaChHandler_t Handler, void *pCtx, int IntPrio, bool bIrq)
{
	s_Stm32DmaSlot[Ch].Handler = Handler;
	s_Stm32DmaSlot[Ch].pCtx = pCtx;
	s_Stm32DmaSlot[Ch].bUsed = true;

	Stm32DmaClock(Ch);
	Stm32DmaChStop(Ch);

	NVIC_ClearPendingIRQ(s_Stm32DmaIrq[Ch]);
	if (bIrq)
	{
		NVIC_SetPriority(s_Stm32DmaIrq[Ch], IntPrio);
		NVIC_EnableIRQ(s_Stm32DmaIrq[Ch]);
	}
	else
	{
		NVIC_DisableIRQ(s_Stm32DmaIrq[Ch]);
	}
}

bool Stm32DmaChClaim(int Ch, Stm32DmaChHandler_t Handler, void *pCtx, int IntPrio, bool bIrq)
{
	if (Ch < 0 || Ch >= STM32_DMA_NBCH)
	{
		return false;
	}

	uint32_t pm = __get_PRIMASK();
	bool ok = false;

	__disable_irq();
	if (s_Stm32DmaSlot[Ch].bUsed == false ||
		(s_Stm32DmaSlot[Ch].Handler == Handler && s_Stm32DmaSlot[Ch].pCtx == pCtx))
	{
		Stm32DmaTake(Ch, Handler, pCtx, IntPrio, bIrq);
		ok = true;
	}
	__set_PRIMASK(pm);

	return ok;
}

void Stm32DmaChFree(int Ch)
{
	if (Ch < 0 || Ch >= STM32_DMA_NBCH)
	{
		return;
	}

	uint32_t pm = __get_PRIMASK();

	__disable_irq();
	NVIC_DisableIRQ(s_Stm32DmaIrq[Ch]);
	Stm32DmaChStop(Ch);
	s_Stm32DmaCh[Ch]->CCR = 0;
	s_Stm32DmaSlot[Ch].Handler = NULL;
	s_Stm32DmaSlot[Ch].pCtx = NULL;
	s_Stm32DmaSlot[Ch].bUsed = false;
	__set_PRIMASK(pm);
}

void Stm32DmaChSetup(int Ch, uint32_t Req, bool bToPeriph, volatile void *pPeriph, int WidthLog2, bool bIrq)
{
	DMA_Channel_TypeDef *ch = s_Stm32DmaCh[Ch];
	const uint32_t w = (uint32_t)WidthLog2 & 3U;

	Stm32DmaChStop(Ch);
	ch->CCR = 0;
	Stm32DmaRoute(Ch, Req);
	s_Stm32DmaWidth[Ch] = (uint8_t)w;
	ch->CPAR = (uint32_t)(uintptr_t)pPeriph;
	ch->CCR = (w << DMA_CCR_PSIZE_Pos) | (w << DMA_CCR_MSIZE_Pos) | DMA_CCR_MINC |
			  (bToPeriph ? DMA_CCR_DIR : 0U) | (bIrq ? DMA_CCR_TCIE | DMA_CCR_TEIE : 0U);
}

void Stm32DmaChMemInc(int Ch, bool bInc)
{
	DMA_Channel_TypeDef *ch = s_Stm32DmaCh[Ch];

	if (bInc)
	{
		ch->CCR |= DMA_CCR_MINC;
	}
	else
	{
		ch->CCR &= ~DMA_CCR_MINC;
	}
}

void Stm32DmaChStart(int Ch, const void *pMem, uint32_t Count)
{
	DMA_Channel_TypeDef *ch = s_Stm32DmaCh[Ch];

	// CMAR and CNDTR can only be written with the channel disabled
	ch->CCR &= ~DMA_CCR_EN;
	Stm32DmaChClear(Ch);
	ch->CMAR = (uint32_t)(uintptr_t)pMem;
	ch->CNDTR = Count >> s_Stm32DmaWidth[Ch];
	ch->CCR |= DMA_CCR_EN;
}

void Stm32DmaChStop(int Ch)
{
	s_Stm32DmaCh[Ch]->CCR &= ~DMA_CCR_EN;
	Stm32DmaChClear(Ch);
}

uint32_t Stm32DmaChStopRemain(int Ch)
{
	DMA_Channel_TypeDef *ch = s_Stm32DmaCh[Ch];

	// CNDTR keeps the count left when the channel is disabled
	ch->CCR &= ~DMA_CCR_EN;

	uint32_t remain = (ch->CNDTR & 0xFFFFU) << s_Stm32DmaWidth[Ch];

	Stm32DmaChClear(Ch);

	return remain;
}

uint32_t Stm32DmaChRemain(int Ch)
{
	return (s_Stm32DmaCh[Ch]->CNDTR & 0xFFFFU) << s_Stm32DmaWidth[Ch];
}

static uint32_t Stm32DmaFlagsOf(uint32_t Isr)
{
	return ((Isr & STM32_DMA_ISR_TC) ? STM32_DMA_FLAG_TC : 0U) |
		   ((Isr & STM32_DMA_ISR_TE) ? STM32_DMA_FLAG_ERR : 0U);
}

uint32_t Stm32DmaChFlags(int Ch)
{
	return Stm32DmaFlagsOf(Stm32DmaIsr(Ch)) | s_Stm32DmaLatch[Ch];
}

void Stm32DmaChClear(int Ch)
{
	s_Stm32DmaLatch[Ch] = 0U;
	// CGIF clears the 4 flags of the channel
	Stm32DmaCtrl(Ch)->IFCR = DMA_IFCR_CGIF1 << Stm32DmaShift(Ch);
}

void Stm32DmaChAck(int Ch)
{
	const uint32_t flags = Stm32DmaFlagsOf(Stm32DmaIsr(Ch));

	Stm32DmaCtrl(Ch)->IFCR = DMA_IFCR_CGIF1 << Stm32DmaShift(Ch);
	s_Stm32DmaLatch[Ch] |= flags;
}

static void Stm32DmaChIrq(int Ch)
{
	if (s_Stm32DmaSlot[Ch].Handler)
	{
		s_Stm32DmaSlot[Ch].Handler(Ch, s_Stm32DmaSlot[Ch].pCtx);
	}
	else
	{
		Stm32DmaChClear(Ch);
	}
}

extern "C" void DMA1_Channel1_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(1, 1)); }
extern "C" void DMA1_Channel2_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(1, 2)); }
extern "C" void DMA1_Channel3_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(1, 3)); }
extern "C" void DMA1_Channel4_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(1, 4)); }
extern "C" void DMA1_Channel5_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(1, 5)); }
extern "C" void DMA1_Channel6_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(1, 6)); }
extern "C" void DMA1_Channel7_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(1, 7)); }
extern "C" void DMA2_Channel1_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(2, 1)); }
extern "C" void DMA2_Channel2_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(2, 2)); }
extern "C" void DMA2_Channel3_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(2, 3)); }
extern "C" void DMA2_Channel4_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(2, 4)); }
extern "C" void DMA2_Channel5_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(2, 5)); }
extern "C" void DMA2_Channel6_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(2, 6)); }
extern "C" void DMA2_Channel7_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_CH(2, 7)); }
