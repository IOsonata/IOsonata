/**-------------------------------------------------------------------------
@file	dma_stm32.cpp

@brief	STM32 DMA channel and stream manager

		See dma_stm32.h. The manager owns the DMA1 and DMA2 channel interrupt
		vectors of L4/L4+ and stream vectors of H753 and gives each channel to one driver
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
#include <string.h>

#include "dma_stm32.h"

#if defined(STM32H753xx)
#define STM32_DMA_ISR_TC			0x20U
#define STM32_DMA_ISR_TE			0x0DU // FEIF, DMEIF and TEIF
#else
// TC and TE of the group of 4 flags of a channel in ISR, GIF is bit 0
#define STM32_DMA_ISR_TC			2U
#define STM32_DMA_ISR_TE			8U
#endif

typedef struct {
	Stm32DmaChHandler_t Handler;
	void *pCtx;
	bool bUsed;
} Stm32DmaSlot_t;

static Stm32DmaSlot_t s_Stm32DmaSlot[STM32_DMA_NBCH];
static uint32_t s_Stm32DmaLatch[STM32_DMA_NBCH];	// flags taken by Stm32DmaChAck
static uint8_t s_Stm32DmaWidth[STM32_DMA_NBCH];		// transfer width, log2 of bytes

#if defined(STM32H753xx)
static DMA_Stream_TypeDef * const s_Stm32DmaCh[16] = {
	DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
	DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
	DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
	DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7,
};
static const IRQn_Type s_Stm32DmaIrq[STM32_DMA_NBCH] = {
	DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
	DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
	DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
	DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn,
	BDMA_Channel0_IRQn, BDMA_Channel1_IRQn, BDMA_Channel2_IRQn, BDMA_Channel3_IRQn,
	BDMA_Channel4_IRQn, BDMA_Channel5_IRQn, BDMA_Channel6_IRQn, BDMA_Channel7_IRQn,
};

static BDMA_Channel_TypeDef * const s_Stm32BdmaCh[8] = {
	BDMA_Channel0, BDMA_Channel1, BDMA_Channel2, BDMA_Channel3,
	BDMA_Channel4, BDMA_Channel5, BDMA_Channel6, BDMA_Channel7,
};

// EN is cleared only after the current beat finishes. Do not reprogram or
// release a stream that failed to stop. No controller reset: other owners
// can still be using the other seven streams.
static bool Stm32DmaDisable(int Ch)
{
	if (Ch >= 16)
	{
		s_Stm32BdmaCh[Ch - 16]->CCR &= ~BDMA_CCR_EN;
		for (unsigned i = 0; i < 100000U; i++)
		{
			if ((s_Stm32BdmaCh[Ch - 16]->CCR & BDMA_CCR_EN) == 0) { __DSB(); return true; }
		}
		s_Stm32DmaLatch[Ch] |= STM32_DMA_FLAG_ERR;
		return false;
	}
	DMA_Stream_TypeDef *ch = s_Stm32DmaCh[Ch];
	ch->CR &= ~DMA_SxCR_EN;
	for (unsigned i = 0; i < 100000U; i++)
	{
		if ((ch->CR & DMA_SxCR_EN) == 0) { __DSB(); return true; }
	}
	s_Stm32DmaLatch[Ch] |= STM32_DMA_FLAG_ERR;
	return false;
}
#else
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

#endif

static inline DMA_TypeDef *Stm32DmaCtrl(int Ch)
{
	return Ch < STM32_DMA_CTRL_NBCH ? DMA1 : DMA2;
}

// Position of the flag group of the channel in ISR and IFCR
static inline uint32_t Stm32DmaShift(int Ch)
{
#if defined(STM32H753xx)
	static const uint8_t shift[4] = { 0, 6, 16, 22 };
	return shift[Ch & 3];
#else
	return (uint32_t)(Ch % STM32_DMA_CTRL_NBCH) * 4U;
#endif
}

static inline uint32_t Stm32DmaIsr(int Ch)
{
#if defined(STM32H753xx)
	if (Ch >= 16)
	{
		uint32_t f = (BDMA->ISR >> ((Ch - 16) * 4)) & 15U;
		return ((f & 2) ? STM32_DMA_ISR_TC : 0U) | ((f & 8) ? STM32_DMA_ISR_TE : 0U);
	}
	DMA_TypeDef *dma = Stm32DmaCtrl(Ch);
	return ((Ch & 4 ? dma->HISR : dma->LISR) >> Stm32DmaShift(Ch)) & 0x3DU;
#else
	return (Stm32DmaCtrl(Ch)->ISR >> Stm32DmaShift(Ch)) & 0xFU;
#endif
}

// Controller clock on, and DMAMUX1 when the part has it
static void Stm32DmaClock(int Ch)
{
#if defined(STM32H753xx)
	if (Ch >= 16)
	{
		RCC->AHB4ENR |= RCC_AHB4ENR_BDMAEN;
		(void)RCC->AHB4ENR;
		return;
	}
#endif
	RCC->AHB1ENR |= Stm32DmaCtrl(Ch) == DMA1 ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
#ifdef RCC_AHB1ENR_DMAMUX1EN
	RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
#endif
	(void)RCC->AHB1ENR;
}

// Request routed to the channel: DMAMUX1 (L4+) or CSELR (L4)
static void Stm32DmaRoute(int Ch, uint32_t Req)
{
#if defined(STM32H753xx)
	if (Ch >= 16) { DMAMUX2_Channel0[Ch - 16].CCR = Req; return; }
#endif
#if defined(DMAMUX1) || defined(STM32H753xx)
	// DMA1 channel n on DMAMUX1 channel n - 1, DMA2 channel n after the 7 DMA1
	// channels
	DMAMUX1_Channel0[Ch].CCR = Req;
#else
	DMA_Request_TypeDef *sel = Stm32DmaCtrl(Ch) == DMA1 ? DMA1_CSELR : DMA2_CSELR;
	uint32_t pos = Stm32DmaShift(Ch);

	sel->CSELR = (sel->CSELR & ~(DMA_CSELR_C1S_Msk << pos)) | ((Req & DMA_CSELR_C1S_Msk) << pos);
#endif
}

static bool Stm32DmaTake(int Ch, Stm32DmaChHandler_t Handler, void *pCtx, int IntPrio, bool bIrq)
{
#if defined(STM32H753xx)
	Stm32DmaClock(Ch);
	if (!Stm32DmaDisable(Ch)) { return false; }
	s_Stm32DmaWidth[Ch] = 0xFF; // Setup must precede Start.
#endif
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
	return true;
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
		ok = Stm32DmaTake(Ch, Handler, pCtx, IntPrio, bIrq);
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
#if defined(STM32H753xx)
	if (!Stm32DmaDisable(Ch))
	{
		__set_PRIMASK(pm);
		return;
	}
	if (Ch >= 16) { s_Stm32BdmaCh[Ch - 16]->CCR = 0; }
	else { s_Stm32DmaCh[Ch]->CR = 0; }
	Stm32DmaRoute(Ch, 0);
#else
	s_Stm32DmaCh[Ch]->CCR = 0;
#endif
	s_Stm32DmaSlot[Ch].Handler = NULL;
	s_Stm32DmaSlot[Ch].pCtx = NULL;
	s_Stm32DmaSlot[Ch].bUsed = false;
	__set_PRIMASK(pm);
}

#if defined(STM32H753xx)
void Stm32DmaChSetup(int Ch, uint32_t Req, bool bToPeriph, volatile void *pPeriph, int WidthLog2, bool bIrq)
{
	if (!Stm32DmaDisable(Ch)) { return; }
	Stm32DmaChClear(Ch);
	s_Stm32DmaWidth[Ch] = 0xFF;
	// Peripheral requests only; no request-generator or synchronization mode.
	if (WidthLog2 < 0 || WidthLog2 > 2 || Req < 9 || Req > (Ch >= 16 ? 20U : 115U) ||
		pPeriph == NULL || ((uintptr_t)pPeriph & ((1U << WidthLog2) - 1U)))
	{
		s_Stm32DmaLatch[Ch] = STM32_DMA_FLAG_ERR;
		return;
	}
	Stm32DmaRoute(Ch, Req);
	s_Stm32DmaWidth[Ch] = (uint8_t)WidthLog2;
	if (Ch >= 16)
	{
		BDMA_Channel_TypeDef *ch = s_Stm32BdmaCh[Ch - 16];
		ch->CPAR = (uint32_t)(uintptr_t)pPeriph;
		ch->CCR = ((uint32_t)WidthLog2 << BDMA_CCR_PSIZE_Pos) |
			((uint32_t)WidthLog2 << BDMA_CCR_MSIZE_Pos) | BDMA_CCR_MINC |
			(bToPeriph ? BDMA_CCR_DIR : 0U) | (bIrq ? BDMA_CCR_TCIE | BDMA_CCR_TEIE : 0U);
		return;
	}
	DMA_Stream_TypeDef *ch = s_Stm32DmaCh[Ch];
	ch->PAR = (uint32_t)(uintptr_t)pPeriph;
	ch->FCR = 0; // Direct mode, single beats; NDTR counts peripheral items.
	ch->CR = ((uint32_t)WidthLog2 << DMA_SxCR_PSIZE_Pos) |
		((uint32_t)WidthLog2 << DMA_SxCR_MSIZE_Pos) | DMA_SxCR_MINC |
		(bToPeriph ? DMA_SxCR_DIR_0 : 0U) |
		(bIrq ? DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE : 0U);
}

void Stm32DmaChMemInc(int Ch, bool bInc)
{
	if (!Stm32DmaDisable(Ch)) { return; }
	if (Ch >= 16)
	{
		if (bInc) { s_Stm32BdmaCh[Ch - 16]->CCR |= BDMA_CCR_MINC; }
		else { s_Stm32BdmaCh[Ch - 16]->CCR &= ~BDMA_CCR_MINC; }
		return;
	}
	if (bInc) { s_Stm32DmaCh[Ch]->CR |= DMA_SxCR_MINC; }
	else { s_Stm32DmaCh[Ch]->CR &= ~DMA_SxCR_MINC; }
}

void Stm32DmaChStart(int Ch, const void *pMem, uint32_t Count)
{
	if (!Stm32DmaDisable(Ch)) { return; }
	Stm32DmaChClear(Ch);
	uint32_t w = s_Stm32DmaWidth[Ch];
	uintptr_t addr = (uintptr_t)pMem;
	bool inc = Ch >= 16 ? (s_Stm32BdmaCh[Ch - 16]->CCR & BDMA_CCR_MINC) != 0 :
		(s_Stm32DmaCh[Ch]->CR & DMA_SxCR_MINC) != 0;
	uint32_t span = inc ? Count : (w <= 2 ? 1U << w : 0);
	// The H753 linker puts ordinary data in DTCM, which DMA1/2 cannot reach.
	if (w > 2 || Count == 0 || Count >> w > 65535 ||
		(Count & ((1U << w) - 1U)) || (addr & ((1U << w) - 1U)) ||
		addr < 0x08000000UL || addr > 0xFFFFFFFFUL - span ||
		(addr < 0x20020000UL && addr + span > 0x20000000UL) ||
		(Ch >= 16 && (addr < 0x38000000UL || addr + span > 0x38010000UL)))
	{
		s_Stm32DmaLatch[Ch] = STM32_DMA_FLAG_ERR;
		return;
	}
	if (Ch >= 16)
	{
		BDMA_Channel_TypeDef *ch = s_Stm32BdmaCh[Ch - 16];
		ch->CM0AR = (uint32_t)addr;
		ch->CNDTR = Count >> w;
		__DMB();
		ch->CCR |= BDMA_CCR_EN;
		return;
	}
	DMA_Stream_TypeDef *ch = s_Stm32DmaCh[Ch];
	ch->M0AR = (uint32_t)addr;
	ch->NDTR = Count >> w;
	__DMB();
	ch->CR |= DMA_SxCR_EN;
}

void Stm32DmaChStop(int Ch)
{
	if (Stm32DmaDisable(Ch)) { Stm32DmaChClear(Ch); }
}

uint32_t Stm32DmaChStopRemain(int Ch)
{
	bool stopped = Stm32DmaDisable(Ch);
	uint32_t remain = Stm32DmaChRemain(Ch);
	if (stopped) { Stm32DmaChClear(Ch); }
	return remain;
}

uint32_t Stm32DmaChRemain(int Ch)
{
	uint32_t w = s_Stm32DmaWidth[Ch];
	uint32_t n = Ch >= 16 ? s_Stm32BdmaCh[Ch - 16]->CNDTR : s_Stm32DmaCh[Ch]->NDTR;
	return w <= 2 ? (n & 0xFFFFU) << w : 0;
}

#else
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

#endif

static uint32_t Stm32DmaFlagsOf(uint32_t Isr)
{
	return ((Isr & STM32_DMA_ISR_TC) ? STM32_DMA_FLAG_TC : 0U) |
		   ((Isr & STM32_DMA_ISR_TE) ? STM32_DMA_FLAG_ERR : 0U);
}

uint32_t Stm32DmaChFlags(int Ch)
{
	return Stm32DmaFlagsOf(Stm32DmaIsr(Ch)) | s_Stm32DmaLatch[Ch];
}

static void Stm32DmaClearHw(int Ch)
{
#if defined(STM32H753xx)
	if (Ch >= 16) { BDMA->IFCR = 15U << ((Ch - 16) * 4); return; }
	DMA_TypeDef *dma = Stm32DmaCtrl(Ch);
	uint32_t flags = 0x3DU << Stm32DmaShift(Ch);
	if (Ch & 4) { dma->HIFCR = flags; }
	else { dma->LIFCR = flags; }
#else
	Stm32DmaCtrl(Ch)->IFCR = DMA_IFCR_CGIF1 << Stm32DmaShift(Ch);
#endif
}

void Stm32DmaChClear(int Ch)
{
	s_Stm32DmaLatch[Ch] = 0U;
	Stm32DmaClearHw(Ch);
}

void Stm32DmaChAck(int Ch)
{
	const uint32_t flags = Stm32DmaFlagsOf(Stm32DmaIsr(Ch));

	Stm32DmaClearHw(Ch);
	s_Stm32DmaLatch[Ch] |= flags;
}

#if defined(STM32H753xx)
// Staging owns complete cache lines. Application memory can be unaligned or
// in DTCM. SRAM4 is reachable by both DMA1/2 and BDMA.
#define STM32_DMA_BUFFER_SIZE 512U
static uint8_t s_Stm32DmaBuffer[STM32_DMA_NBCH][STM32_DMA_BUFFER_SIZE]
	__attribute__((section(".bus_dma"), aligned(32)));
typedef struct {
	uint8_t *pMem;
	uint32_t Count;
	uint32_t Done;
	uint32_t Chunk;
	uint32_t Flags;
	bool bActive;
	bool bTx;
	bool bInc;
} Stm32DmaBuffer_t;
static Stm32DmaBuffer_t s_Stm32DmaBufferState[STM32_DMA_NBCH];

int Stm32DmaChAlloc(bool bBdma, Stm32DmaChHandler_t Handler, void *pCtx, int IntPrio, bool bIrq)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	int found = -1;
	for (int ch = bBdma ? 16 : 0; ch < (bBdma ? 24 : 16); ch++)
	{
		if (!s_Stm32DmaSlot[ch].bUsed && Stm32DmaChClaim(ch, Handler, pCtx, IntPrio, bIrq))
		{
			found = ch;
			break;
		}
	}
	__set_PRIMASK(pm);
	return found;
}

static void Stm32DmaBufferChunk(int Ch)
{
	Stm32DmaBuffer_t *s = &s_Stm32DmaBufferState[Ch];
	uint8_t *buf = s_Stm32DmaBuffer[Ch];
	uint32_t left = s->Count - s->Done;
	s->Chunk = left < STM32_DMA_BUFFER_SIZE ? left : STM32_DMA_BUFFER_SIZE;
	if (s->bTx)
	{
		memcpy(buf, s->pMem + (s->bInc ? s->Done : 0), s->bInc ? s->Chunk : 1U << s_Stm32DmaWidth[Ch]);
	}
	if (SCB->CCR & SCB_CCR_DC_Msk)
	{
		if (s->bTx) { SCB_CleanDCache_by_Addr((uint32_t*)buf, STM32_DMA_BUFFER_SIZE); }
		else { SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buf, STM32_DMA_BUFFER_SIZE); }
	}
	Stm32DmaChStart(Ch, buf, s->Chunk);
	s->Flags |= Stm32DmaChFlags(Ch) & STM32_DMA_FLAG_ERR;
}

static void Stm32DmaBufferCopy(int Ch, uint32_t Count)
{
	Stm32DmaBuffer_t *s = &s_Stm32DmaBufferState[Ch];
	if (!s->bTx && Count)
	{
		uint8_t *buf = s_Stm32DmaBuffer[Ch];
		if (SCB->CCR & SCB_CCR_DC_Msk)
		{
			SCB_InvalidateDCache_by_Addr((uint32_t*)buf, STM32_DMA_BUFFER_SIZE);
		}
		memcpy(s->pMem + (s->bInc ? s->Done : 0), buf, s->bInc ? Count : 1U << s_Stm32DmaWidth[Ch]);
	}
}

// Called with interrupts masked. Intermediate TC belongs to the staging
// engine, not the owner's logical transfer.
static bool Stm32DmaBufferService(int Ch)
{
	Stm32DmaBuffer_t *s = &s_Stm32DmaBufferState[Ch];
	if (!s->bActive) { return true; }
	uint32_t f = Stm32DmaChFlags(Ch);
	if (f & STM32_DMA_FLAG_ERR)
	{
		s->Flags |= STM32_DMA_FLAG_ERR;
		Stm32DmaChAck(Ch);
		return true;
	}
	if (f & STM32_DMA_FLAG_TC)
	{
		Stm32DmaBufferCopy(Ch, s->Chunk);
		s->Done += s->Chunk;
		Stm32DmaChClear(Ch);
		if (s->Done < s->Count)
		{
			Stm32DmaBufferChunk(Ch);
			return s->Flags != 0;
		}
		s->Chunk = 0;
		s->bActive = false;
		s->Flags |= STM32_DMA_FLAG_TC;
		return true;
	}
	return false;
}

void Stm32DmaChBufferStart(int Ch, const void *pMem, uint32_t Count)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	Stm32DmaChBufferStopRemain(Ch);
	Stm32DmaBuffer_t *s = &s_Stm32DmaBufferState[Ch];
	if (s->bActive) { __set_PRIMASK(pm); return; }
	memset(s, 0, sizeof(*s));
	s->pMem = (uint8_t*)pMem;
	s->Count = Count;
	uint32_t w = s_Stm32DmaWidth[Ch];
	if (!pMem || !Count || w > 2 || (Count & ((1U << w) - 1U)))
	{
		s->Flags = STM32_DMA_FLAG_ERR;
		__set_PRIMASK(pm);
		return;
	}
	if (Ch >= 16)
	{
		s->bTx = (s_Stm32BdmaCh[Ch - 16]->CCR & BDMA_CCR_DIR) != 0;
		s->bInc = (s_Stm32BdmaCh[Ch - 16]->CCR & BDMA_CCR_MINC) != 0;
	}
	else
	{
		s->bTx = (s_Stm32DmaCh[Ch]->CR & DMA_SxCR_DIR_0) != 0;
		s->bInc = (s_Stm32DmaCh[Ch]->CR & DMA_SxCR_MINC) != 0;
	}
	s->bActive = true;
	Stm32DmaBufferChunk(Ch);
	__set_PRIMASK(pm);
}

uint32_t Stm32DmaChBufferFlags(int Ch)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	Stm32DmaBufferService(Ch);
	uint32_t f = s_Stm32DmaBufferState[Ch].Flags;
	__set_PRIMASK(pm);
	return f;
}

uint32_t Stm32DmaChBufferRemain(int Ch)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	Stm32DmaBufferService(Ch);
	Stm32DmaBuffer_t *s = &s_Stm32DmaBufferState[Ch];
	uint32_t n = s->Count - s->Done;
	if (s->bActive)
	{
		uint32_t left = Stm32DmaChRemain(Ch);
		if (left <= s->Chunk) { n -= s->Chunk - left; }
		else { s->Flags |= STM32_DMA_FLAG_ERR; }
	}
	__set_PRIMASK(pm);
	return n;
}

uint32_t Stm32DmaChBufferStopRemain(int Ch)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	Stm32DmaBuffer_t *s = &s_Stm32DmaBufferState[Ch];
	if (s->bActive)
	{
		s->Flags |= Stm32DmaChFlags(Ch) & STM32_DMA_FLAG_ERR;
		if (Stm32DmaDisable(Ch))
		{
			uint32_t left = Stm32DmaChRemain(Ch);
			uint32_t n = left <= s->Chunk ? s->Chunk - left : 0;
			if (left > s->Chunk) { s->Flags |= STM32_DMA_FLAG_ERR; }
			Stm32DmaBufferCopy(Ch, n);
			s->Done += n;
			s->Chunk = 0;
			s->bActive = false;
			Stm32DmaChClear(Ch);
		}
		else { s->Flags |= STM32_DMA_FLAG_ERR; }
	}
	uint32_t n = s->Count - s->Done;
	__set_PRIMASK(pm);
	return n;
}
#endif

static void Stm32DmaChIrq(int Ch)
{
#if defined(STM32H753xx)
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	bool report = Stm32DmaBufferService(Ch);
	__set_PRIMASK(pm);
	if (!report) { return; }
#endif
	if (s_Stm32DmaSlot[Ch].Handler)
	{
		s_Stm32DmaSlot[Ch].Handler(Ch, s_Stm32DmaSlot[Ch].pCtx);
	}
	else
	{
		Stm32DmaChClear(Ch);
	}
}

#if defined(STM32H753xx)
extern "C" void BDMA_Channel0_IRQHandler(void) { Stm32DmaChIrq(16); }
extern "C" void BDMA_Channel1_IRQHandler(void) { Stm32DmaChIrq(17); }
extern "C" void BDMA_Channel2_IRQHandler(void) { Stm32DmaChIrq(18); }
extern "C" void BDMA_Channel3_IRQHandler(void) { Stm32DmaChIrq(19); }
extern "C" void BDMA_Channel4_IRQHandler(void) { Stm32DmaChIrq(20); }
extern "C" void BDMA_Channel5_IRQHandler(void) { Stm32DmaChIrq(21); }
extern "C" void BDMA_Channel6_IRQHandler(void) { Stm32DmaChIrq(22); }
extern "C" void BDMA_Channel7_IRQHandler(void) { Stm32DmaChIrq(23); }
extern "C" void DMA1_Stream0_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(1, 0)); }
extern "C" void DMA1_Stream1_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(1, 1)); }
extern "C" void DMA1_Stream2_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(1, 2)); }
extern "C" void DMA1_Stream3_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(1, 3)); }
extern "C" void DMA1_Stream4_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(1, 4)); }
extern "C" void DMA1_Stream5_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(1, 5)); }
extern "C" void DMA1_Stream6_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(1, 6)); }
extern "C" void DMA1_Stream7_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(1, 7)); }
extern "C" void DMA2_Stream0_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(2, 0)); }
extern "C" void DMA2_Stream1_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(2, 1)); }
extern "C" void DMA2_Stream2_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(2, 2)); }
extern "C" void DMA2_Stream3_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(2, 3)); }
extern "C" void DMA2_Stream4_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(2, 4)); }
extern "C" void DMA2_Stream5_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(2, 5)); }
extern "C" void DMA2_Stream6_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(2, 6)); }
extern "C" void DMA2_Stream7_IRQHandler(void) { Stm32DmaChIrq(STM32_DMA_STREAM(2, 7)); }
#else
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


#endif
