/**-------------------------------------------------------------------------
@file	dmac_samd.c

@brief	SAM D11/D21 DMAC channels

See dmac_samd.h. The channel registers are reached through CHID: every
access sequence runs with interrupts disabled.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "samdxxx.h"

#include "coredev/interrupt.h"
#include "dmac_samd.h"

#define SAMD_DMAC_STOP_LIMIT		100000U
#define SAMD_DMAC_CH_INTS			(DMAC_CHINTFLAG_TERR_Msk | DMAC_CHINTFLAG_TCMPL_Msk | DMAC_CHINTFLAG_SUSP_Msk)

// Descriptors and write-back section, apart as Harmony, ASF and Zephyr have
// them. BASEADDR and WRBADDR need 64-bit alignment (SAM D11 errata, data
// sheet clarification), 128-bit is kept.
static dmac_descriptor_registers_t s_SamdDmacDesc[DMAC_CH_NUM] __attribute__((aligned(16)));
static dmac_descriptor_registers_t s_SamdDmacWrb[DMAC_CH_NUM] __attribute__((aligned(16)));

static SamdDmacHandler_t s_SamdDmacHandler[DMAC_CH_NUM];
static void *s_pSamdDmacCtx[DMAC_CH_NUM];
static int8_t s_SamdDmacPrio[DMAC_CH_NUM];
static uint16_t s_SamdDmacUsed;

// Bits of CHCTRLA of the selected channel cleared, bounded wait
static void SamdDmacChanWait(uint8_t Mask)
{
	for (uint32_t i = 0; i < SAMD_DMAC_STOP_LIMIT && (DMAC_REGS->DMAC_CHCTRLA & Mask) != 0U; i++)
	{
	}
}

// DMAC on, the descriptor sections set while it is disabled
static void SamdDmacOn(void)
{
	PM_REGS->PM_AHBMASK |= PM_AHBMASK_DMAC_Msk;
	PM_REGS->PM_APBBMASK |= PM_APBBMASK_DMAC_Msk;

	if ((DMAC_REGS->DMAC_CTRL & DMAC_CTRL_DMAENABLE_Msk) != 0U &&
		DMAC_REGS->DMAC_BASEADDR == (uint32_t)s_SamdDmacDesc)
	{
		return;
	}

	// Reset ignored and the sections write protected until the DMAC and the
	// CRC read disabled (a bootloader may leave them on)
	DMAC_REGS->DMAC_CTRL = 0;
	for (uint32_t i = 0; i < SAMD_DMAC_STOP_LIMIT &&
		 (DMAC_REGS->DMAC_CTRL & (DMAC_CTRL_DMAENABLE_Msk | DMAC_CTRL_CRCENABLE_Msk)); i++)
	{
	}
	DMAC_REGS->DMAC_CTRL = DMAC_CTRL_SWRST_Msk;
	for (uint32_t i = 0; i < SAMD_DMAC_STOP_LIMIT && (DMAC_REGS->DMAC_CTRL & DMAC_CTRL_SWRST_Msk); i++)
	{
	}
	DMAC_REGS->DMAC_BASEADDR = (uint32_t)s_SamdDmacDesc;
	DMAC_REGS->DMAC_WRBADDR = (uint32_t)s_SamdDmacWrb;
	DMAC_REGS->DMAC_CTRL = DMAC_CTRL_DMAENABLE_Msk | DMAC_CTRL_LVLEN0_Msk | DMAC_CTRL_LVLEN1_Msk |
						   DMAC_CTRL_LVLEN2_Msk | DMAC_CTRL_LVLEN3_Msk;
}

// Most urgent priority of the open channels with a handler, a closed
// channel has none
static void SamdDmacIrqPrio(void)
{
	int prio = -1;

	for (int i = 0; i < DMAC_CH_NUM; i++)
	{
		if (s_SamdDmacHandler[i] != NULL && (prio < 0 || s_SamdDmacPrio[i] < prio))
		{
			prio = s_SamdDmacPrio[i];
		}
	}

	if (prio >= 0)
	{
		NVIC_SetPriority(DMAC_IRQn, SamdIrqPrio(prio));
		NVIC_EnableIRQ(DMAC_IRQn);
	}
}

int SamdDmacOpen(int Trig, SamdDmacHandler_t Handler, void *pCtx, int IntPrio)
{
	int chan = -1;

	if (Trig <= 0 || Trig >= DMAC_TRIG_NUM || (Handler != NULL && (IntPrio < 0 || IntPrio > INT8_MAX)))
	{
		return -1;
	}

	uint32_t state = DisableInterrupt();

	for (int i = 0; i < DMAC_CH_NUM; i++)
	{
		if ((s_SamdDmacUsed & (1U << i)) == 0U)
		{
			s_SamdDmacUsed |= (uint16_t)(1U << i);
			chan = i;
			break;
		}
	}

	if (chan >= 0)
	{
		SamdDmacOn();

		s_SamdDmacHandler[chan] = Handler;
		s_pSamdDmacCtx[chan] = pCtx;
		s_SamdDmacPrio[chan] = (int8_t)(Handler != NULL ? IntPrio : 0);
		s_SamdDmacWrb[chan].DMAC_BTCNT = 0;

		// Channel reset, ignored while ENABLE reads 1, then a beat per
		// trigger at the lowest level
		DMAC_REGS->DMAC_CHID = (uint8_t)chan;
		DMAC_REGS->DMAC_CHCTRLA = 0;
		SamdDmacChanWait(DMAC_CHCTRLA_ENABLE_Msk);
		DMAC_REGS->DMAC_CHCTRLA = DMAC_CHCTRLA_SWRST_Msk;
		SamdDmacChanWait(DMAC_CHCTRLA_SWRST_Msk);
		DMAC_REGS->DMAC_CHCTRLB = DMAC_CHCTRLB_TRIGSRC((uint32_t)Trig) | DMAC_CHCTRLB_TRIGACT_BEAT;
		DMAC_REGS->DMAC_CHINTFLAG = SAMD_DMAC_CH_INTS;
		if (Handler != NULL)
		{
			DMAC_REGS->DMAC_CHINTENSET = DMAC_CHINTENSET_TCMPL_Msk | DMAC_CHINTENSET_TERR_Msk;
		}
		else
		{
			DMAC_REGS->DMAC_CHINTENCLR = SAMD_DMAC_CH_INTS;
		}
		SamdDmacIrqPrio();
	}

	EnableInterrupt(state);

	return chan;
}

void SamdDmacClose(int Chan)
{
	if (Chan < 0 || Chan >= DMAC_CH_NUM || (s_SamdDmacUsed & (1U << Chan)) == 0U)
	{
		return;
	}

	(void)SamdDmacStop(Chan);

	uint32_t state = DisableInterrupt();

	DMAC_REGS->DMAC_CHID = (uint8_t)Chan;
	DMAC_REGS->DMAC_CHINTENCLR = SAMD_DMAC_CH_INTS;
	DMAC_REGS->DMAC_CHINTFLAG = SAMD_DMAC_CH_INTS;
	s_SamdDmacHandler[Chan] = NULL;
	s_SamdDmacUsed &= (uint16_t)~(1U << Chan);
	SamdDmacIrqPrio();

	EnableInterrupt(state);
}

void SamdDmacTrigger(int Chan, int Trig)
{
	if (Chan < 0 || Chan >= DMAC_CH_NUM || Trig <= 0 || Trig >= DMAC_TRIG_NUM)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	// CHCTRLB is written with the channel disabled
	DMAC_REGS->DMAC_CHID = (uint8_t)Chan;
	if ((DMAC_REGS->DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) == 0U)
	{
		DMAC_REGS->DMAC_CHCTRLB = DMAC_CHCTRLB_TRIGSRC((uint32_t)Trig) | DMAC_CHCTRLB_TRIGACT_BEAT;
	}

	EnableInterrupt(state);
}

void SamdDmacStart(int Chan, const volatile void *pSrc, bool bSrcInc, volatile void *pDst, bool bDstInc,
				   uint32_t Count)
{
	if (Chan < 0 || Chan >= DMAC_CH_NUM || Count == 0U || Count > SAMD_DMAC_COUNT_MAX)
	{
		return;
	}

	// An incremented address is the end of the block: start + count
	dmac_descriptor_registers_t *desc = &s_SamdDmacDesc[Chan];
	uint32_t src = (uint32_t)pSrc + (bSrcInc ? Count : 0U);
	uint32_t dst = (uint32_t)pDst + (bDstInc ? Count : 0U);

	uint32_t state = DisableInterrupt();

	desc->DMAC_BTCTRL = (uint16_t)(DMAC_BTCTRL_VALID_Msk | DMAC_BTCTRL_BLOCKACT(DMAC_BTCTRL_BLOCKACT_INT_Val) |
								   DMAC_BTCTRL_BEATSIZE(DMAC_BTCTRL_BEATSIZE_BYTE_Val) |
								   (bSrcInc ? DMAC_BTCTRL_SRCINC_Msk : 0U) | (bDstInc ? DMAC_BTCTRL_DSTINC_Msk : 0U));
	desc->DMAC_BTCNT = (uint16_t)Count;
	desc->DMAC_SRCADDR = src;
	desc->DMAC_DSTADDR = dst;
	desc->DMAC_DESCADDR = 0;
	__DMB();

	DMAC_REGS->DMAC_CHID = (uint8_t)Chan;
	DMAC_REGS->DMAC_CHINTFLAG = SAMD_DMAC_CH_INTS;
	DMAC_REGS->DMAC_CHCTRLA = DMAC_CHCTRLA_ENABLE_Msk;

	EnableInterrupt(state);
}

bool SamdDmacBusy(int Chan)
{
	if (Chan < 0 || Chan >= DMAC_CH_NUM)
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	DMAC_REGS->DMAC_CHID = (uint8_t)Chan;
	bool busy = (DMAC_REGS->DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0U;

	EnableInterrupt(state);

	return busy;
}

uint32_t SamdDmacStop(int Chan)
{
	if (Chan < 0 || Chan >= DMAC_CH_NUM)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();

	DMAC_REGS->DMAC_CHID = (uint8_t)Chan;
	if (DMAC_REGS->DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk)
	{
		// ENABLE reads 1 until the beat in progress is done, then the
		// write-back section holds the count left
		DMAC_REGS->DMAC_CHCTRLA = 0;
		SamdDmacChanWait(DMAC_CHCTRLA_ENABLE_Msk);
	}
	DMAC_REGS->DMAC_CHINTFLAG = SAMD_DMAC_CH_INTS;

	uint32_t left = s_SamdDmacWrb[Chan].DMAC_BTCNT;

	EnableInterrupt(state);

	return left;
}

void DMAC_Handler(void)
{
	// INTPEND gives the lowest channel with a flag, its ID selects the
	// channel of the flags written back
	for (int n = 0; n < DMAC_CH_NUM * 2; n++)
	{
		uint16_t pend = DMAC_REGS->DMAC_INTPEND;
		uint32_t flags = pend & (DMAC_INTPEND_TERR_Msk | DMAC_INTPEND_TCMPL_Msk | DMAC_INTPEND_SUSP_Msk);
		int chan = (int)((pend & DMAC_INTPEND_ID_Msk) >> DMAC_INTPEND_ID_Pos);

		if (flags == 0U)
		{
			break;
		}

		DMAC_REGS->DMAC_INTPEND = (uint16_t)((pend & DMAC_INTPEND_ID_Msk) | flags);

		if (chan < DMAC_CH_NUM && s_SamdDmacHandler[chan] != NULL)
		{
			// INTPEND flags are the CHINTFLAG bits shifted by 8
			s_SamdDmacHandler[chan](chan, flags >> DMAC_INTPEND_TERR_Pos, s_pSamdDmacCtx[chan]);
		}
	}
}
