/**-------------------------------------------------------------------------
@file	evsys_samd.c

@brief	SAM D11/D21 event system channels

A channel is allocated for one generator and at most one user. The user
multiplexer is set before the channel, as the datasheet asks. The
synchronous and resynchronized paths run on the channel generic clock on
generator 0. The event detected flag of those paths calls the handler of
the channel from the EVSYS interrupt.

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
#include "clock_samd.h"
#include "evsys_samd.h"

typedef struct {
	SamdEvsysHandler_t Handler;
	void *pCtx;
} SamdEvsysChan_t;

static SamdEvsysChan_t s_SamdEvsysChan[EVSYS_CHANNELS];
static uint32_t s_SamdEvsysUsed;

// Overrun and event detected flags of a channel: channels 0 to 7 at bits 0
// and 8, channels 8 to 11 at bits 16 and 24
static inline uint32_t SamdEvsysPos(int Chan)
{
	return Chan < 8 ? (uint32_t)Chan : (uint32_t)Chan + 8U;
}

static inline uint32_t SamdEvsysFlags(int Chan)
{
	return 0x101UL << SamdEvsysPos(Chan);
}

static inline uint32_t SamdEvsysEvd(int Chan)
{
	return 0x100UL << SamdEvsysPos(Chan);
}

int SamdEvsysOpen(int Gen, int Path, int Edge, int User, SamdEvsysHandler_t Handler, void *pCtx, int IntPrio)
{
	uint32_t state;
	int chan = -1;

	// The asynchronous path has no edge detection and no interrupt
	if (Gen <= 0 || Gen > (int)(EVSYS_CHANNEL_EVGEN_Msk >> EVSYS_CHANNEL_EVGEN_Pos) ||
		Path < SAMD_EVSYS_PATH_SYNC || Path > SAMD_EVSYS_PATH_ASYNC ||
		Edge < SAMD_EVSYS_EDGE_NONE || Edge > SAMD_EVSYS_EDGE_BOTH ||
		(Path == SAMD_EVSYS_PATH_ASYNC && (Edge != SAMD_EVSYS_EDGE_NONE || Handler != NULL)) ||
		(Path != SAMD_EVSYS_PATH_ASYNC && Edge == SAMD_EVSYS_EDGE_NONE) ||
		User >= (int)EVSYS_USERS || IntPrio < 0)
	{
		return -1;
	}

	state = DisableInterrupt();
	for (int i = 0; i < EVSYS_CHANNELS; i++)
	{
		if ((s_SamdEvsysUsed & (1UL << i)) == 0U)
		{
			s_SamdEvsysUsed |= 1UL << i;
			chan = i;
			break;
		}
	}
	EnableInterrupt(state);

	if (chan < 0)
	{
		return -1;
	}

	PM_REGS->PM_APBCMASK |= PM_APBCMASK_EVSYS_Msk;

	if (Path != SAMD_EVSYS_PATH_ASYNC &&
		!SamdGclkChannelEnable((uint16_t)(EVSYS_GCLK_ID_0 + chan), SAMD_GCLK_MAIN))
	{
		state = DisableInterrupt();
		s_SamdEvsysUsed &= ~(1UL << chan);
		EnableInterrupt(state);

		return -1;
	}

	state = DisableInterrupt();

	s_SamdEvsysChan[chan].Handler = Handler;
	s_SamdEvsysChan[chan].pCtx = pCtx;

	// The user multiplexer before the channel, each in a single write
	if (User >= 0)
	{
		EVSYS_REGS->EVSYS_USER = (uint16_t)(EVSYS_USER_CHANNEL((uint16_t)(chan + 1)) | EVSYS_USER_USER((uint16_t)User));
	}
	EVSYS_REGS->EVSYS_CHANNEL = EVSYS_CHANNEL_CHANNEL((uint32_t)chan) | EVSYS_CHANNEL_EVGEN((uint32_t)Gen) |
								EVSYS_CHANNEL_PATH((uint32_t)Path) | EVSYS_CHANNEL_EDGSEL((uint32_t)Edge);

	EVSYS_REGS->EVSYS_INTFLAG = SamdEvsysFlags(chan);
	if (Handler != NULL)
	{
		EVSYS_REGS->EVSYS_INTENSET = SamdEvsysEvd(chan);

		NVIC_ClearPendingIRQ(EVSYS_IRQn);
		NVIC_SetPriority(EVSYS_IRQn, SamdIrqPrio(IntPrio));
		NVIC_EnableIRQ(EVSYS_IRQn);
	}

	EnableInterrupt(state);

	return chan;
}

void SamdEvsysClose(int Chan, int User)
{
	uint32_t state;

	if (Chan < 0 || Chan >= EVSYS_CHANNELS || User >= (int)EVSYS_USERS)
	{
		return;
	}

	state = DisableInterrupt();

	if ((s_SamdEvsysUsed & (1UL << Chan)) != 0U)
	{
		EVSYS_REGS->EVSYS_INTENCLR = SamdEvsysFlags(Chan);

		// No channel on the user, no generator on the channel
		if (User >= 0)
		{
			EVSYS_REGS->EVSYS_USER = EVSYS_USER_USER((uint16_t)User);
		}
		EVSYS_REGS->EVSYS_CHANNEL = EVSYS_CHANNEL_CHANNEL((uint32_t)Chan);
		EVSYS_REGS->EVSYS_INTFLAG = SamdEvsysFlags(Chan);

		s_SamdEvsysChan[Chan].Handler = NULL;
		s_SamdEvsysChan[Chan].pCtx = NULL;
		s_SamdEvsysUsed &= ~(1UL << Chan);
	}

	EnableInterrupt(state);
}

// Each flag is cleared before its handler, an event during the handler
// stays pending
void EVSYS_Handler(void)
{
	uint32_t flags = EVSYS_REGS->EVSYS_INTFLAG;

	for (int i = 0; i < EVSYS_CHANNELS; i++)
	{
		uint32_t mask = SamdEvsysFlags(i);

		if ((flags & mask) == 0U)
		{
			continue;
		}
		EVSYS_REGS->EVSYS_INTFLAG = flags & mask;

		if ((flags & SamdEvsysEvd(i) & EVSYS_REGS->EVSYS_INTENSET) != 0U &&
			s_SamdEvsysChan[i].Handler != NULL)
		{
			s_SamdEvsysChan[i].Handler(i, s_SamdEvsysChan[i].pCtx);
		}
	}
}
