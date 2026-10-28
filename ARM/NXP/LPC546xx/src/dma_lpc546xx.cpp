/**-------------------------------------------------------------------------
@file	dma_lpc546xx.cpp

@brief	LPC546xx DMA controller channels

		One transfer descriptor per channel in the channel table, no linked
		descriptors. The peripheral request of the channel paces the
		transfer, it is started by a software trigger and the trigger is
		cleared when the descriptor is done. Each transfer sets INTA at its
		end, INTA is the done flag with or without the interrupt.

@author	Hoang Nguyen Hoan
@date	Oct. 26, 2026

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

#include "LPC546xx.h"

#include "coredev/interrupt.h"
#include "dma_lpc546xx.h"

// Channel table alignment required by SRAMBASE
#define LPC546XX_DMA_TABLE_ALIGN	FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE

// XFERCOUNT reads this value once the descriptor is exhausted
#define LPC546XX_DMA_XFERCOUNT_DONE	0x3FFU

// Reads of BUSY while a disabled channel finishes its current value
#define LPC546XX_DMA_BUSY_POLLS		10000

// Width value of a channel not set up
#define LPC546XX_DMA_WIDTH_NONE		0xFFU

#pragma pack(push, 4)

/// Channel descriptor of the channel table, read by the controller
typedef struct {
	volatile uint32_t XferCfg;		//!< For a linked descriptor, the register holds it for this one
	volatile uint32_t SrcEnd;		//!< Address of the last source value
	volatile uint32_t DstEnd;		//!< Address of the last destination value
	volatile uint32_t Link;			//!< Next descriptor, 0 none
} Lpc546xxDmaDesc_t;

typedef struct {
	Lpc546xxDmaChHandler_t Handler;	//!< Called from the DMA interrupt
	void *pCtx;						//!< Context passed to Handler
	volatile void *pPeriph;			//!< Peripheral data register
	uint8_t Latch;					//!< Flags taken by the interrupt
	int8_t IntPrio;					//!< Interrupt priority asked
	uint8_t Width;					//!< Log2 of the value size in bytes
	bool bUsed;						//!< Channel taken
	bool bIrq;						//!< Interrupt enabled
	bool bToPeriph;					//!< Memory to peripheral
	bool bMemInc;					//!< Memory address increments
	bool bStarted;					//!< A transfer was started since the last stop
} Lpc546xxDmaSlot_t;

#pragma pack(pop)

alignas(LPC546XX_DMA_TABLE_ALIGN) static Lpc546xxDmaDesc_t s_Lpc546xxDmaDesc[LPC546XX_DMA_NBCH];
static Lpc546xxDmaSlot_t s_Lpc546xxDmaSlot[LPC546XX_DMA_NBCH];
static int s_Lpc546xxDmaUsedCnt = 0;

static void Lpc546xxDmaCtrlOn(void);
static void Lpc546xxDmaCtrlOff(void);
static void Lpc546xxDmaIrqPrio(void);
static bool Lpc546xxDmaDisable(int Ch);
static uint32_t Lpc546xxDmaHwFlags(int Ch);
static void Lpc546xxDmaHwClear(int Ch);

// Clock on, reset, channel table and controller enabled
static void Lpc546xxDmaCtrlOn(void)
{
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;
	SYSCON->PRESETCTRLSET[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;
	SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;

	DMA0->SRAMBASE = (uint32_t)(uintptr_t)s_Lpc546xxDmaDesc;
	DMA0->CTRL = DMA_CTRL_ENABLE_MASK;
}

static void Lpc546xxDmaCtrlOff(void)
{
	NVIC_DisableIRQ(DMA0_IRQn);
	DMA0->CTRL = 0;
	SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_DMA_MASK;
}

// One vector for all channels, its priority is the most urgent one asked by
// the channels using the interrupt. Off when none does.
static void Lpc546xxDmaIrqPrio(void)
{
	int prio = -1;

	for (int i = 0; i < LPC546XX_DMA_NBCH; i++)
	{
		if (s_Lpc546xxDmaSlot[i].bUsed && s_Lpc546xxDmaSlot[i].bIrq &&
			(prio < 0 || s_Lpc546xxDmaSlot[i].IntPrio < prio))
		{
			prio = s_Lpc546xxDmaSlot[i].IntPrio;
		}
	}

	if (prio < 0)
	{
		NVIC_DisableIRQ(DMA0_IRQn);
	}
	else
	{
		NVIC_SetPriority(DMA0_IRQn, (uint32_t)prio);
		NVIC_EnableIRQ(DMA0_IRQn);
	}
}

// Disable the channel and wait for the value in progress to finish. Returns
// false if the channel is still busy after LPC546XX_DMA_BUSY_POLLS reads.
static bool Lpc546xxDmaDisable(int Ch)
{
	uint32_t bit = 1UL << Ch;

	DMA0->COMMON[0].ENABLECLR = bit;

	for (int i = 0; i < LPC546XX_DMA_BUSY_POLLS; i++)
	{
		if ((DMA0->COMMON[0].BUSY & bit) == 0U)
		{
			return true;
		}
	}

	s_Lpc546xxDmaSlot[Ch].Latch |= LPC546XX_DMA_FLAG_ERR;

	return false;
}

static uint32_t Lpc546xxDmaHwFlags(int Ch)
{
	uint32_t bit = 1UL << Ch;

	return ((DMA0->COMMON[0].INTA & bit) ? LPC546XX_DMA_FLAG_DONE : 0U) |
		   ((DMA0->COMMON[0].ERRINT & bit) ? LPC546XX_DMA_FLAG_ERR : 0U);
}

// INTA and ERRINT are cleared by writing 1
static void Lpc546xxDmaHwClear(int Ch)
{
	uint32_t bit = 1UL << Ch;

	DMA0->COMMON[0].INTA = bit;
	DMA0->COMMON[0].ERRINT = bit;
}

bool Lpc546xxDmaChClaim(int Ch, Lpc546xxDmaChHandler_t Handler, void *pCtx, int IntPrio, bool bIrq)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return false;
	}

	Lpc546xxDmaSlot_t *slot = &s_Lpc546xxDmaSlot[Ch];
	uint32_t state = DisableInterrupt();

	if (slot->bUsed && (slot->Handler != Handler || slot->pCtx != pCtx))
	{
		EnableInterrupt(state);

		return false;
	}

	if (slot->bUsed == false)
	{
		if (s_Lpc546xxDmaUsedCnt == 0)
		{
			Lpc546xxDmaCtrlOn();
		}
		s_Lpc546xxDmaUsedCnt++;
		slot->Width = LPC546XX_DMA_WIDTH_NONE;
	}

	slot->Handler = Handler;
	slot->pCtx = pCtx;
	slot->bUsed = true;
	slot->bIrq = bIrq;
	slot->IntPrio = (int8_t)IntPrio;

	Lpc546xxDmaChStop(Ch);

	if (bIrq)
	{
		DMA0->COMMON[0].INTENSET = 1UL << Ch;
	}
	else
	{
		DMA0->COMMON[0].INTENCLR = 1UL << Ch;
	}
	Lpc546xxDmaIrqPrio();

	EnableInterrupt(state);

	return true;
}

void Lpc546xxDmaChFree(int Ch)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return;
	}

	Lpc546xxDmaSlot_t *slot = &s_Lpc546xxDmaSlot[Ch];
	uint32_t state = DisableInterrupt();

	if (slot->bUsed)
	{
		DMA0->COMMON[0].INTENCLR = 1UL << Ch;
		Lpc546xxDmaChStop(Ch);
		DMA0->CHANNEL[Ch].CFG = 0;

		slot->Handler = NULL;
		slot->pCtx = NULL;
		slot->pPeriph = NULL;
		slot->bUsed = false;
		slot->bIrq = false;
		slot->Width = LPC546XX_DMA_WIDTH_NONE;

		if (--s_Lpc546xxDmaUsedCnt == 0)
		{
			Lpc546xxDmaCtrlOff();
		}
		else
		{
			Lpc546xxDmaIrqPrio();
		}
	}

	EnableInterrupt(state);
}

void Lpc546xxDmaChSetup(int Ch, bool bToPeriph, volatile void *pPeriph, int WidthLog2)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH || s_Lpc546xxDmaSlot[Ch].bUsed == false)
	{
		return;
	}

	Lpc546xxDmaSlot_t *slot = &s_Lpc546xxDmaSlot[Ch];

	Lpc546xxDmaChStop(Ch);

	slot->Width = LPC546XX_DMA_WIDTH_NONE;

	if (WidthLog2 < 0 || WidthLog2 > 2 || pPeriph == NULL ||
		((uintptr_t)pPeriph & ((1U << WidthLog2) - 1U)) != 0U)
	{
		slot->Latch = LPC546XX_DMA_FLAG_ERR;

		return;
	}

	slot->pPeriph = pPeriph;
	slot->bToPeriph = bToPeriph;
	slot->bMemInc = true;
	slot->Width = (uint8_t)WidthLog2;

	// Peripheral request on, no hardware trigger, priority 0
	DMA0->CHANNEL[Ch].CFG = DMA_CHANNEL_CFG_PERIPHREQEN_MASK;
}

void Lpc546xxDmaChMemInc(int Ch, bool bInc)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return;
	}

	s_Lpc546xxDmaSlot[Ch].bMemInc = bInc;
}

bool Lpc546xxDmaChStart(int Ch, const void *pMem, uint32_t Count)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return false;
	}

	Lpc546xxDmaSlot_t *slot = &s_Lpc546xxDmaSlot[Ch];
	uint32_t w = slot->Width;
	uintptr_t addr = (uintptr_t)pMem;

	uint32_t state = DisableInterrupt();

	if (slot->bUsed == false || w > 2U || pMem == NULL || Count == 0U ||
		(Count & ((1U << w) - 1U)) != 0U || (Count >> w) > LPC546XX_DMA_XFER_MAX ||
		(addr & ((1U << w) - 1U)) != 0U)
	{
		slot->Latch |= LPC546XX_DMA_FLAG_ERR;
		EnableInterrupt(state);

		return false;
	}

	if (Lpc546xxDmaDisable(Ch) == false)
	{
		EnableInterrupt(state);

		return false;
	}
	DMA0->COMMON[0].ABORT = 1UL << Ch;
	Lpc546xxDmaChClear(Ch);

	uint32_t inc = slot->bMemInc ? 1U : 0U;
	uint32_t memend = (uint32_t)addr + (inc ? Count - (1U << w) : 0U);
	uint32_t periph = (uint32_t)(uintptr_t)slot->pPeriph;
	uint32_t cfg = DMA_CHANNEL_XFERCFG_CFGVALID_MASK | DMA_CHANNEL_XFERCFG_CLRTRIG_MASK |
				   DMA_CHANNEL_XFERCFG_SETINTA_MASK | DMA_CHANNEL_XFERCFG_WIDTH(w) |
				   DMA_CHANNEL_XFERCFG_XFERCOUNT((Count >> w) - 1U);
	Lpc546xxDmaDesc_t *desc = &s_Lpc546xxDmaDesc[Ch];

	if (slot->bToPeriph)
	{
		cfg |= DMA_CHANNEL_XFERCFG_SRCINC(inc);
		desc->SrcEnd = memend;
		desc->DstEnd = periph;
	}
	else
	{
		cfg |= DMA_CHANNEL_XFERCFG_DSTINC(inc);
		desc->SrcEnd = periph;
		desc->DstEnd = memend;
	}
	desc->XferCfg = cfg;
	desc->Link = 0;

	slot->bStarted = true;

	// The descriptor is in memory before the controller reads it
	__DMB();

	DMA0->CHANNEL[Ch].XFERCFG = cfg;
	DMA0->COMMON[0].ENABLESET = 1UL << Ch;
	DMA0->CHANNEL[Ch].XFERCFG = cfg | DMA_CHANNEL_XFERCFG_SWTRIG_MASK;

	EnableInterrupt(state);

	return true;
}

void Lpc546xxDmaChStop(int Ch)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	if (Lpc546xxDmaDisable(Ch))
	{
		DMA0->COMMON[0].ABORT = 1UL << Ch;
		Lpc546xxDmaChClear(Ch);
	}
	s_Lpc546xxDmaSlot[Ch].bStarted = false;

	EnableInterrupt(state);
}

uint32_t Lpc546xxDmaChStopRemain(int Ch)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	bool stopped = Lpc546xxDmaDisable(Ch);
	uint32_t remain = Lpc546xxDmaChRemain(Ch);

	if (stopped)
	{
		DMA0->COMMON[0].ABORT = 1UL << Ch;
		Lpc546xxDmaChClear(Ch);
	}
	s_Lpc546xxDmaSlot[Ch].bStarted = false;

	EnableInterrupt(state);

	return remain;
}

// XFERCOUNT is the number of values left minus 1. It reads
// LPC546XX_DMA_XFERCOUNT_DONE when the descriptor is done, which is also the
// value of a transfer of LPC546XX_DMA_XFER_MAX values not begun. The done
// flag tells them apart, and once the flags are cleared the channel is no
// longer active (ACTIVE is set from the trigger to the end or the abort).
uint32_t Lpc546xxDmaChRemain(int Ch)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return 0;
	}

	Lpc546xxDmaSlot_t *slot = &s_Lpc546xxDmaSlot[Ch];

	if (slot->bStarted == false || slot->Width > 2U ||
		(Lpc546xxDmaChFlags(Ch) & LPC546XX_DMA_FLAG_DONE) != 0U)
	{
		return 0;
	}

	uint32_t cnt = (DMA0->CHANNEL[Ch].XFERCFG & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) >>
				   DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT;

	if (cnt == LPC546XX_DMA_XFERCOUNT_DONE && (DMA0->COMMON[0].ACTIVE & (1UL << Ch)) == 0U)
	{
		return 0;
	}

	return (cnt + 1U) << slot->Width;
}

uint32_t Lpc546xxDmaChFlags(int Ch)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return 0;
	}

	return Lpc546xxDmaHwFlags(Ch) | s_Lpc546xxDmaSlot[Ch].Latch;
}

void Lpc546xxDmaChClear(int Ch)
{
	if (Ch < 0 || Ch >= LPC546XX_DMA_NBCH)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	s_Lpc546xxDmaSlot[Ch].Latch = 0;
	Lpc546xxDmaHwClear(Ch);

	EnableInterrupt(state);
}

// The flags of each channel pending are taken into its latch and cleared in
// the controller before its handler runs. INTA is pending for the channels
// with the interrupt enabled. ERRINT is taken for every channel: the error
// interrupt does not depend on INTENSET (ACTIVEERRINT is any error, the NXP
// SDK clears it without looking at INTENSET), an error of a channel without
// interrupt would otherwise enter this vector again and again. Its handler
// is not called, the flags tell its owner.
extern "C" void DMA0_IRQHandler(void)
{
	uint32_t pend = (DMA0->COMMON[0].INTA & DMA0->COMMON[0].INTENSET) | DMA0->COMMON[0].ERRINT;

	while (pend != 0U)
	{
		int ch = __builtin_ctz(pend);
		Lpc546xxDmaSlot_t *slot = &s_Lpc546xxDmaSlot[ch];

		pend &= pend - 1U;

		slot->Latch |= Lpc546xxDmaHwFlags(ch);
		Lpc546xxDmaHwClear(ch);

		if (slot->bIrq && slot->Handler != NULL)
		{
			slot->Handler(ch, slot->pCtx);
		}
	}
}
