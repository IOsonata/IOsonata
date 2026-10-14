/**-------------------------------------------------------------------------
@file	sau_r9a02.cpp

@brief	R9A02G021 serial array unit channel sharing

Channel ownership, module stop and the shared operation clocks of the SAU
units. See sau_r9a02.h.

The prescaler register SPSm must not be written while a channel of the unit
runs (R9A02G021 User's Manual R01UH1036EJ0110, 23.3.1). A prescaler is also
kept while another claimed channel uses it, stopped or not, so that its
transfer clock does not change when it starts again.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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

#include "r9a02g021.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "system_r9a02.h"
#include "sau_r9a02.h"

#define R9A02_SAU_NO_CK				-1

typedef struct __R9a02_Sau_Unit {
	uint16_t ChValid;				// Channels of the unit
	uint32_t MstpMask;				// MSTPCRB bit
	const void *pOwner[R9A02_SAU_CHAN_CNT];
	int8_t Ck[R9A02_SAU_CHAN_CNT];	// Operation clock used, 0, 1 or R9A02_SAU_NO_CK
} R9a02SauUnit_t;

static R9a02SauUnit_t s_R9a02SauUnit[R9A02_SAU_UNIT_CNT] = {
	{ 0x0FU, R9A02_MSTPCRB_SAU0, { NULL, NULL, NULL, NULL },
	  { R9A02_SAU_NO_CK, R9A02_SAU_NO_CK, R9A02_SAU_NO_CK, R9A02_SAU_NO_CK } },
	{ 0x03U, R9A02_MSTPCRB_SAU1, { NULL, NULL, NULL, NULL },
	  { R9A02_SAU_NO_CK, R9A02_SAU_NO_CK, R9A02_SAU_NO_CK, R9A02_SAU_NO_CK } },
};

static bool R9a02SauAnyClaimed(const R9a02SauUnit_t *pUnit)
{
	for (uint32_t ch = 0; ch < R9A02_SAU_CHAN_CNT; ch++)
	{
		if (pUnit->pOwner[ch] != NULL)
		{
			return true;
		}
	}

	return false;
}

bool R9a02SauClaim(uint32_t Unit, uint16_t ChMask, const void *pOwner)
{
	if (Unit >= R9A02_SAU_UNIT_CNT || pOwner == NULL || ChMask == 0U)
	{
		return false;
	}

	R9a02SauUnit_t *unit = &s_R9a02SauUnit[Unit];

	if ((ChMask & ~unit->ChValid) != 0U)
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();

	for (uint32_t ch = 0; ch < R9A02_SAU_CHAN_CNT; ch++)
	{
		if ((ChMask & (1U << ch)) && unit->pOwner[ch] != NULL && unit->pOwner[ch] != pOwner)
		{
			EnableInterrupt(state);

			return false;
		}
	}

	// Module clock first, register writes before it are ignored
	if (!R9a02SauAnyClaimed(unit))
	{
		R9a02ModuleStart(R9A02_MSTPCR_B, unit->MstpMask);
	}

	for (uint32_t ch = 0; ch < R9A02_SAU_CHAN_CNT; ch++)
	{
		if (ChMask & (1U << ch))
		{
			unit->pOwner[ch] = pOwner;
		}
	}

	EnableInterrupt(state);

	return true;
}

void R9a02SauRelease(uint32_t Unit, uint16_t ChMask, const void *pOwner)
{
	if (Unit >= R9A02_SAU_UNIT_CNT || pOwner == NULL)
	{
		return;
	}

	R9a02SauUnit_t *unit = &s_R9a02SauUnit[Unit];
	uintptr_t state = DisableInterrupt();
	bool had = R9a02SauAnyClaimed(unit);

	for (uint32_t ch = 0; ch < R9A02_SAU_CHAN_CNT; ch++)
	{
		if ((ChMask & (1U << ch)) && unit->pOwner[ch] == pOwner)
		{
			unit->pOwner[ch] = NULL;
			unit->Ck[ch] = R9A02_SAU_NO_CK;
		}
	}

	if (had && !R9a02SauAnyClaimed(unit))
	{
		R9a02ModuleStop(R9A02_MSTPCR_B, unit->MstpMask);
	}

	EnableInterrupt(state);
}

uint32_t R9a02SauSetRate(uint32_t Unit, uint16_t ChMask, uint32_t Rate, uint32_t MinDiv,
						 uint32_t MaxRate, R9a02SauClk_t *pClk)
{
	uint32_t pclk = SystemPeriphClockGet(0);

	if (Unit >= R9A02_SAU_UNIT_CNT || pClk == NULL || ChMask == 0U || Rate == 0U || pclk == 0U ||
		MinDiv == 0U || MinDiv > R9A02_SAU_DIV_MAX)
	{
		return 0;
	}

	R9a02SauUnit_t *unit = &s_R9a02SauUnit[Unit];
	R9A02_SAU_Type *reg = R9a02SauReg(Unit);
	uintptr_t state = DisableInterrupt();
	uint16_t sps = reg->SPS;
	bool othersrun = (reg->SE & (uint16_t)~ChMask) != 0U;
	uint32_t best = 0;
	uint32_t besterr = UINT32_MAX;
	uint32_t bestck = 0;
	uint32_t bestprs = 0;
	uint32_t bestdiv = 0;
	bool bestkeep = false;

	for (uint32_t ck = 0; ck < 2U; ck++)
	{
		uint32_t shift = ck ? R9A02_SAU_SPS_PRS1_Pos : R9A02_SAU_SPS_PRS0_Pos;
		uint32_t cur = (sps >> shift) & R9A02_SAU_SPS_PRS_Msk;
		bool locked = othersrun;

		for (uint32_t ch = 0; ch < R9A02_SAU_CHAN_CNT; ch++)
		{
			if ((ChMask & (1U << ch)) == 0U && unit->pOwner[ch] != NULL && unit->Ck[ch] == (int8_t)ck)
			{
				locked = true;
			}
		}

		for (uint32_t prs = 0; prs <= R9A02_SAU_PRS_MAX; prs++)
		{
			uint32_t fmck = pclk >> prs;

			if ((locked && prs != cur) || fmck > R9A02_SAU_FMCK_MAX)
			{
				continue;
			}

			// Div rounded to the closest rate, then raised until the rate
			// is within MaxRate
			uint32_t div = (fmck + Rate) / (2U * Rate);

			if (div < MinDiv)
			{
				div = MinDiv;
			}
			if (div > R9A02_SAU_DIV_MAX)
			{
				div = R9A02_SAU_DIV_MAX;
			}

			uint32_t actual = (fmck + div) / (2U * div);

			while (actual > MaxRate && div < R9A02_SAU_DIV_MAX)
			{
				div++;
				actual = (fmck + div) / (2U * div);
			}

			if (actual == 0U || actual > MaxRate)
			{
				continue;
			}

			uint32_t err = actual > Rate ? actual - Rate : Rate - actual;
			bool keep = prs == cur;

			// The closest rate, on equal error a prescaler that does not
			// change
			if (err < besterr || (err == besterr && keep && !bestkeep))
			{
				besterr = err;
				best = actual;
				bestck = ck;
				bestprs = prs;
				bestdiv = div;
				bestkeep = keep;
			}
		}
	}

	if (best != 0U)
	{
		uint32_t shift = bestck ? R9A02_SAU_SPS_PRS1_Pos : R9A02_SAU_SPS_PRS0_Pos;

		if (!bestkeep)
		{
			reg->SPS = (uint16_t)((sps & ~(R9A02_SAU_SPS_PRS_Msk << shift)) | (bestprs << shift));
		}

		for (uint32_t ch = 0; ch < R9A02_SAU_CHAN_CNT; ch++)
		{
			if (ChMask & (1U << ch))
			{
				unit->Ck[ch] = (int8_t)bestck;
			}
		}

		pClk->Cks = bestck ? R9A02_SAU_SMR_CKS : 0U;
		pClk->Sdr = (uint16_t)((bestdiv - 1U) << R9A02_SAU_SDR_STCLK_Pos);
		pClk->FmckHz = pclk >> bestprs;
	}

	EnableInterrupt(state);

	return best;
}

bool R9a02SauSetPrs(uint32_t Unit, uint16_t ChMask, uint32_t Prs, R9a02SauClk_t *pClk)
{
	uint32_t pclk = SystemPeriphClockGet(0);

	if (Unit >= R9A02_SAU_UNIT_CNT || pClk == NULL || ChMask == 0U || Prs > R9A02_SAU_PRS_MAX ||
		pclk == 0U || (pclk >> Prs) > R9A02_SAU_FMCK_MAX)
	{
		return false;
	}

	R9a02SauUnit_t *unit = &s_R9a02SauUnit[Unit];
	R9A02_SAU_Type *reg = R9a02SauReg(Unit);
	uintptr_t state = DisableInterrupt();
	uint16_t sps = reg->SPS;
	bool othersrun = (reg->SE & (uint16_t)~ChMask) != 0U;
	int found = R9A02_SAU_NO_CK;
	bool keep = false;

	// A clock already at Prs first, then one whose prescaler can change
	for (uint32_t ck = 0; ck < 2U && !keep; ck++)
	{
		uint32_t shift = ck ? R9A02_SAU_SPS_PRS1_Pos : R9A02_SAU_SPS_PRS0_Pos;
		bool locked = othersrun;

		for (uint32_t ch = 0; ch < R9A02_SAU_CHAN_CNT; ch++)
		{
			if ((ChMask & (1U << ch)) == 0U && unit->pOwner[ch] != NULL && unit->Ck[ch] == (int8_t)ck)
			{
				locked = true;
			}
		}

		if (((sps >> shift) & R9A02_SAU_SPS_PRS_Msk) == Prs)
		{
			found = (int)ck;
			keep = true;
		}
		else if (!locked && found == R9A02_SAU_NO_CK)
		{
			found = (int)ck;
		}
	}

	if (found != R9A02_SAU_NO_CK)
	{
		uint32_t shift = found ? R9A02_SAU_SPS_PRS1_Pos : R9A02_SAU_SPS_PRS0_Pos;

		if (!keep)
		{
			reg->SPS = (uint16_t)((sps & ~(R9A02_SAU_SPS_PRS_Msk << shift)) | (Prs << shift));
		}

		for (uint32_t ch = 0; ch < R9A02_SAU_CHAN_CNT; ch++)
		{
			if (ChMask & (1U << ch))
			{
				unit->Ck[ch] = (int8_t)found;
			}
		}

		pClk->Cks = found ? R9A02_SAU_SMR_CKS : 0U;
		pClk->Sdr = 0U;
		pClk->FmckHz = pclk >> Prs;
	}

	EnableInterrupt(state);

	return found != R9A02_SAU_NO_CK;
}

// CPU and PCLKB from the HOCO or the MOCO, EXTAL stopped (23.3.25, table
// 10.2 note 1)
uint32_t R9a02SauSnoozeSrcHz(void)
{
	if (R9a02ExtalClockGet() != 0U)
	{
		return 0U;
	}

	switch (R9A02_SYSC_SCKSCR & R9A02_SCKSCR_CKSEL_Msk)
	{
		case R9A02_SCKSCR_HOCO:
			return R9a02HocoClockGet();
		case R9A02_SCKSCR_MOCO:
			return R9A02_MOCO_FREQ;
		default:
			return 0U;
	}
}
