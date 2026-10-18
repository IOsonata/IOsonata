/**-------------------------------------------------------------------------
@file	gpdma_stm32wba.cpp

@brief	STM32WBA GPDMA1 channel manager

		Owns the 8 GPDMA1 channel vectors and the table of who holds which
		channel. See gpdma_stm32wba.h.

		The channel transfers are single beats of 1, 2 or 4 bytes, both sides
		on port 0, the setup the ST HAL uses for a peripheral data register.
		A stopped channel is suspended and then reset, as the HAL does, because
		a channel ignores EN written 0 while it runs.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2026

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

#include "stm32wbaxx.h"

#include "gpdma_stm32wba.h"

#define WBA_GPDMA_STOP_POLLS		1000

typedef struct {
	WbaGpdmaHandler_t Handler;
	void *pCtx;
	bool bUsed;
} WbaGpdmaSlot_t;

static WbaGpdmaSlot_t s_WbaGpdma[WBA_GPDMA_NBCH];
static uint32_t s_WbaGpdmaIe[WBA_GPDMA_NBCH];		// interrupt enables of the channel
static uint32_t s_WbaGpdmaLatch[WBA_GPDMA_NBCH];	// flags taken by WbaGpdmaAck

static DMA_Channel_TypeDef * const s_WbaGpdmaCh[WBA_GPDMA_NBCH] = {
	GPDMA1_Channel0, GPDMA1_Channel1, GPDMA1_Channel2, GPDMA1_Channel3,
	GPDMA1_Channel4, GPDMA1_Channel5, GPDMA1_Channel6, GPDMA1_Channel7,
};

static const IRQn_Type s_WbaGpdmaIrq[WBA_GPDMA_NBCH] = {
	GPDMA1_Channel0_IRQn, GPDMA1_Channel1_IRQn, GPDMA1_Channel2_IRQn, GPDMA1_Channel3_IRQn,
	GPDMA1_Channel4_IRQn, GPDMA1_Channel5_IRQn, GPDMA1_Channel6_IRQn, GPDMA1_Channel7_IRQn,
};

static void WbaGpdmaClock(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA1EN;
	(void)RCC->AHB1ENR;
}

static void WbaGpdmaTake(int Ch, WbaGpdmaHandler_t Handler, void *pCtx, int IntPrio, bool bIrq)
{
	s_WbaGpdma[Ch].Handler = Handler;
	s_WbaGpdma[Ch].pCtx = pCtx;
	s_WbaGpdma[Ch].bUsed = true;

	WbaGpdmaClock();
	WbaGpdmaStop(Ch);

	NVIC_ClearPendingIRQ(s_WbaGpdmaIrq[Ch]);
	if (bIrq)
	{
		NVIC_SetPriority(s_WbaGpdmaIrq[Ch], IntPrio);
		NVIC_EnableIRQ(s_WbaGpdmaIrq[Ch]);
	}
	else
	{
		NVIC_DisableIRQ(s_WbaGpdmaIrq[Ch]);
	}
}

int WbaGpdmaAlloc(WbaGpdmaHandler_t Handler, void *pCtx, int IntPrio, bool bIrq)
{
	uint32_t pm = __get_PRIMASK();
	int ch = -1;

	__disable_irq();
	for (int i = WBA_GPDMA_NBCH - 1; i >= 0; i--)
	{
		if (s_WbaGpdma[i].bUsed == false)
		{
			WbaGpdmaTake(i, Handler, pCtx, IntPrio, bIrq);
			ch = i;
			break;
		}
	}
	__set_PRIMASK(pm);

	return ch;
}

bool WbaGpdmaClaim(int Ch, WbaGpdmaHandler_t Handler, void *pCtx, int IntPrio, bool bIrq)
{
	if (Ch < 0 || Ch >= WBA_GPDMA_NBCH)
	{
		return false;
	}

	uint32_t pm = __get_PRIMASK();
	bool ok = false;

	__disable_irq();
	if (s_WbaGpdma[Ch].bUsed == false || (s_WbaGpdma[Ch].Handler == Handler && s_WbaGpdma[Ch].pCtx == pCtx))
	{
		WbaGpdmaTake(Ch, Handler, pCtx, IntPrio, bIrq);
		ok = true;
	}
	__set_PRIMASK(pm);

	return ok;
}

void WbaGpdmaFree(int Ch)
{
	if (Ch < 0 || Ch >= WBA_GPDMA_NBCH)
	{
		return;
	}

	uint32_t pm = __get_PRIMASK();

	__disable_irq();
	NVIC_DisableIRQ(s_WbaGpdmaIrq[Ch]);
	WbaGpdmaStop(Ch);
	s_WbaGpdmaCh[Ch]->CCR = 0;
	s_WbaGpdmaCh[Ch]->CTR1 = 0;
	s_WbaGpdmaCh[Ch]->CTR2 = 0;
	s_WbaGpdma[Ch].Handler = NULL;
	s_WbaGpdma[Ch].pCtx = NULL;
	s_WbaGpdma[Ch].bUsed = false;
	__set_PRIMASK(pm);
}

void WbaGpdmaSetup(int Ch, uint32_t Req, bool bToPeriph, volatile void *pPeriph, int WidthLog2, bool bIrq)
{
	DMA_Channel_TypeDef *ch = s_WbaGpdmaCh[Ch];
	uint32_t w = ((uint32_t)WidthLog2 << DMA_CTR1_SDW_LOG2_Pos) | ((uint32_t)WidthLog2 << DMA_CTR1_DDW_LOG2_Pos);

	WbaGpdmaStop(Ch);
	ch->CTR1 = w | (bToPeriph ? DMA_CTR1_SINC : DMA_CTR1_DINC);
	ch->CTR2 = (Req << DMA_CTR2_REQSEL_Pos) | (bToPeriph ? DMA_CTR2_DREQ : 0U);
	ch->CBR1 = 0;
	ch->CLLR = 0;
	if (bToPeriph)
	{
		ch->CDAR = (uint32_t)(uintptr_t)pPeriph;
	}
	else
	{
		ch->CSAR = (uint32_t)(uintptr_t)pPeriph;
	}
	s_WbaGpdmaIe[Ch] = bIrq ? (DMA_CCR_TCIE | DMA_CCR_DTEIE | DMA_CCR_ULEIE | DMA_CCR_USEIE) : 0U;
	ch->CCR = s_WbaGpdmaIe[Ch];
}

void WbaGpdmaMemInc(int Ch, bool bInc)
{
	DMA_Channel_TypeDef *ch = s_WbaGpdmaCh[Ch];
	const uint32_t bit = (ch->CTR2 & DMA_CTR2_DREQ) ? DMA_CTR1_SINC : DMA_CTR1_DINC;

	if (bInc)
	{
		ch->CTR1 |= bit;
	}
	else
	{
		ch->CTR1 &= ~bit;
	}
}

void WbaGpdmaStart(int Ch, const void *pMem, uint32_t Count)
{
	DMA_Channel_TypeDef *ch = s_WbaGpdmaCh[Ch];

	ch->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF |
			   DMA_CFCR_SUSPF | DMA_CFCR_TOF;
	if (ch->CTR2 & DMA_CTR2_DREQ)
	{
		ch->CSAR = (uint32_t)(uintptr_t)pMem;
	}
	else
	{
		ch->CDAR = (uint32_t)(uintptr_t)pMem;
	}
	ch->CBR1 = Count;
	s_WbaGpdmaLatch[Ch] = 0U;
	// The interrupt enables are written again: the reset at the end of the
	// previous transfer may have cleared them
	ch->CCR |= s_WbaGpdmaIe[Ch] | DMA_CCR_EN;
}

void WbaGpdmaStop(int Ch)
{
	DMA_Channel_TypeDef *ch = s_WbaGpdmaCh[Ch];

	if (ch->CCR & DMA_CCR_EN)
	{
		ch->CCR |= DMA_CCR_SUSP;
		for (int i = 0; i < WBA_GPDMA_STOP_POLLS && (ch->CSR & DMA_CSR_IDLEF) == 0U; i++)
		{
		}
		ch->CCR = (ch->CCR & ~(DMA_CCR_EN | DMA_CCR_SUSP)) | DMA_CCR_RESET;
	}
	WbaGpdmaClear(Ch);
}

uint32_t WbaGpdmaStopRemain(int Ch)
{
	DMA_Channel_TypeDef *ch = s_WbaGpdmaCh[Ch];
	uint32_t remain;

	if (ch->CCR & DMA_CCR_EN)
	{
		ch->CCR |= DMA_CCR_SUSP;
		for (int i = 0; i < WBA_GPDMA_STOP_POLLS && (ch->CSR & DMA_CSR_IDLEF) == 0U; i++)
		{
		}
		remain = ch->CBR1 & DMA_CBR1_BNDT;
		ch->CCR = (ch->CCR & ~(DMA_CCR_EN | DMA_CCR_SUSP)) | DMA_CCR_RESET;
	}
	else
	{
		remain = ch->CBR1 & DMA_CBR1_BNDT;
	}
	WbaGpdmaClear(Ch);

	return remain;
}

uint32_t WbaGpdmaRemain(int Ch)
{
	return s_WbaGpdmaCh[Ch]->CBR1 & DMA_CBR1_BNDT;
}

static uint32_t WbaGpdmaFlagsOf(uint32_t Csr)
{
	return ((Csr & DMA_CSR_TCF) ? WBA_GPDMA_FLAG_TC : 0U) |
		   ((Csr & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF)) ? WBA_GPDMA_FLAG_ERR : 0U);
}

uint32_t WbaGpdmaFlags(int Ch)
{
	return WbaGpdmaFlagsOf(s_WbaGpdmaCh[Ch]->CSR) | s_WbaGpdmaLatch[Ch];
}

void WbaGpdmaClear(int Ch)
{
	s_WbaGpdmaLatch[Ch] = 0U;
	s_WbaGpdmaCh[Ch]->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF |
							 DMA_CFCR_SUSPF | DMA_CFCR_TOF;
}

void WbaGpdmaAck(int Ch)
{
	const uint32_t flags = WbaGpdmaFlagsOf(s_WbaGpdmaCh[Ch]->CSR);

	s_WbaGpdmaCh[Ch]->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF |
							 DMA_CFCR_SUSPF | DMA_CFCR_TOF;
	s_WbaGpdmaLatch[Ch] |= flags;
}

static void WbaGpdmaIrq(int Ch)
{
	if (s_WbaGpdma[Ch].Handler)
	{
		s_WbaGpdma[Ch].Handler(Ch, s_WbaGpdma[Ch].pCtx);
	}
	else
	{
		WbaGpdmaClear(Ch);
	}
}

extern "C" void GPDMA1_Channel0_IRQHandler(void) { WbaGpdmaIrq(0); }
extern "C" void GPDMA1_Channel1_IRQHandler(void) { WbaGpdmaIrq(1); }
extern "C" void GPDMA1_Channel2_IRQHandler(void) { WbaGpdmaIrq(2); }
extern "C" void GPDMA1_Channel3_IRQHandler(void) { WbaGpdmaIrq(3); }
extern "C" void GPDMA1_Channel4_IRQHandler(void) { WbaGpdmaIrq(4); }
extern "C" void GPDMA1_Channel5_IRQHandler(void) { WbaGpdmaIrq(5); }
extern "C" void GPDMA1_Channel6_IRQHandler(void) { WbaGpdmaIrq(6); }
extern "C" void GPDMA1_Channel7_IRQHandler(void) { WbaGpdmaIrq(7); }
