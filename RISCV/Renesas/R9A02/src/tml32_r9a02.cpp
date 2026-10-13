/**-------------------------------------------------------------------------
@file	tml32_r9a02.cpp

@brief	R9A02G021 32-bit interval timer (TML32)

See tml32_r9a02.h. R9A02G021 User's Manual R01UH1036EJ0110, chapter 19.

Channel 0 of the 16-bit mode (ITL000 + ITL001) or of the 32-bit mode is used,
the other flags are masked (ITLMKF0). Start, table 19.9: module, mode, clock,
divider, compare, flags, ITLCTL0.EN0. Stop, table 19.10: flag masked and
cleared, EN0 = 0, one count clock to stop.

The interval flag ITF00 holds TML32_OUTI high (19.3.3). It is cleared by
writing 0 to it and 1 to the other bits, so that no other flag is lost
(19.2.8). In the DTC modes a repeat transfer writes 0x00 to ITLS0, as
19.3.7, the other flags are masked and never set.

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
#include <stdint.h>
#include <stddef.h>

#include "r9a02g021.h"
#include "coredev/interrupt.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "dtc_r9a02.h"
#include "tml32_r9a02.h"

#define R9A02_TML32_CMP16_MAX		0x10000ULL		// Compare + 1 in 16-bit mode
#define R9A02_TML32_CMP32_MAX		0x100000000ULL	// Compare + 1 in 32-bit mode

typedef struct __R9a02_Tml32_Dev {
	R9a02DtcInfo_t Dtc;				// ITLS0 = 0x00, repeat
	R9a02Tml32Handler_t Handler;
	void *pCtx;
	uint64_t nsPeriod;				// Interval set
	uint32_t ClkHz;					// Count clock before the divider
	int IntNo;						// TML32_OUTI route, -1 when none
	int SnzIntNo;					// SYSTEM_SNZREQ route, -1 when none
	uint8_t Ctl0;					// ITLCTL0 with EN0
	bool bInit;
} R9a02Tml32Dev_t;

static R9a02Tml32Dev_t s_R9a02Tml32 = {
	{ 0, 0, 0, 0, 0, 0, 0 }, NULL, NULL, 0, 0, -1, -1, 0, false
};

// Source of the DTC write to ITLS0, in SRAM
static uint8_t s_R9a02Tml32Zero = 0;

static void R9a02Tml32IrqHandler(int IntNo, void *pCtx)
{
	R9A02_TML32_Type *reg = R9A02_TML32;
	uint8_t f = reg->ITLS0;

	// The flags seen are cleared, a flag set in between is kept (19.2.8)
	reg->ITLS0 = (uint8_t)(~f & R9A02_ITLS0_Msk);

	if ((f & R9A02_ITLS0_ITF00) && s_R9a02Tml32.Handler != NULL)
	{
		s_R9a02Tml32.Handler(s_R9a02Tml32.pCtx);
	}
}

static uint32_t R9a02Tml32ClkHz(R9A02_TML32_CLK Clk)
{
	switch (Clk)
	{
		case R9A02_TML32_CLK_HOCO:
			return R9a02HocoClockGet();
		case R9A02_TML32_CLK_MOCO:
			return R9A02_MOCO_FREQ;
		case R9A02_TML32_CLK_EXTAL:
			return R9a02ExtalClockGet();
		case R9A02_TML32_CLK_LF:
			return R9A02_LOCO_FREQ;
		default:
			return 0;
	}
}

// Flags masked and cleared, channels off (table 19.10), then one count clock
static void R9a02Tml32Halt(uint32_t ClkHz)
{
	R9A02_TML32_Type *reg = R9A02_TML32;

	reg->ITLMKF0 = R9A02_ITLMKF0_Msk;
	reg->ITLS0 = 0U;
	reg->ITLCTL0 = (uint8_t)(reg->ITLCTL0 & ~(R9A02_ITLCTL0_EN0 | R9A02_ITLCTL0_EN1 |
											  R9A02_ITLCTL0_EN2 | R9A02_ITLCTL0_EN3));
	R9a02DelayUs(ClkHz ? 1000000UL / ClkHz + 1U : 31U);
}

// Routes and DTC records removed
static void R9a02Tml32Unroute(void)
{
	if (s_R9a02Tml32.IntNo >= 0)
	{
		R9a02DtcClear(s_R9a02Tml32.IntNo);
		R9a02WakeupSet(R9A02_EVT_TML32_OUTI, false);
		R9a02UnregisterIntHandler(s_R9a02Tml32.IntNo);
		s_R9a02Tml32.IntNo = -1;
	}
	if (s_R9a02Tml32.SnzIntNo >= 0)
	{
		R9a02DtcClear(s_R9a02Tml32.SnzIntNo);
		R9a02UnregisterIntHandler(s_R9a02Tml32.SnzIntNo);
		s_R9a02Tml32.SnzIntNo = -1;
	}
}

// The DTC writes 0x00 to ITLS0 at each event of the slot, repeat mode, both
// addresses fixed, no CPU interrupt (19.3.7)
static bool R9a02Tml32DtcRoute(int IntNo)
{
	return IntNo >= 0 && R9a02DtcSet(IntNo, &s_R9a02Tml32.Dtc) && R9a02DtcEnable(IntNo);
}

bool R9a02Tml32Init(const R9a02Tml32Cfg_t *pCfg)
{
	if (pCfg == NULL || pCfg->nsPeriod == 0U ||
		(pCfg->Mode != R9A02_TML32_MODE_INT && pCfg->Mode != R9A02_TML32_MODE_DTC &&
		 pCfg->Mode != R9A02_TML32_MODE_SNOOZE) ||
		(pCfg->Mode == R9A02_TML32_MODE_INT && (pCfg->IntPrio < 0 || pCfg->IntPrio > (int)R9A02_CLIC_PRIO_MAX)) ||
		(pCfg->bWakeup && (pCfg->Mode != R9A02_TML32_MODE_INT || pCfg->ClkSrc != R9A02_TML32_CLK_LF)) ||
		(pCfg->Mode == R9A02_TML32_MODE_SNOOZE && pCfg->ClkSrc != R9A02_TML32_CLK_LF))
	{
		return false;
	}

	uint32_t hz = R9a02Tml32ClkHz(pCfg->ClkSrc);

	if (hz == 0U)
	{
		return false;
	}

	// Clocks of the period, whole seconds apart so that the product does not
	// overflow, then the divider 2^n and the mode
	uint64_t sec = pCfg->nsPeriod / 1000000000ULL;
	uint64_t ticks = (sec > 0xFFFFFFFFULL ? 0xFFFFFFFFULL : sec) * hz +
					 ((pCfg->nsPeriod % 1000000000ULL) * hz + 500000000ULL) / 1000000000ULL;
	uint32_t div = 0;
	uint64_t cnt = 0;
	bool m32 = false;

	if (ticks < 2U)
	{
		ticks = 2U;
	}

	for (div = 0; div <= R9A02_ITLFDIV_MAX; div++)
	{
		cnt = (ticks + ((1ULL << div) >> 1)) >> div;
		if (cnt <= R9A02_TML32_CMP16_MAX)
		{
			break;
		}
	}

	if (div > R9A02_ITLFDIV_MAX)
	{
		m32 = true;
		for (div = 0; div < R9A02_ITLFDIV_MAX; div++)
		{
			cnt = (ticks + ((1ULL << div) >> 1)) >> div;
			if (cnt <= R9A02_TML32_CMP32_MAX)
			{
				break;
			}
		}
		cnt = (ticks + ((1ULL << div) >> 1)) >> div;
		if (cnt > R9A02_TML32_CMP32_MAX)
		{
			cnt = R9A02_TML32_CMP32_MAX;
		}
	}

	if (cnt < 2U)
	{
		cnt = 2U;
	}

	uint32_t cmp = (uint32_t)(cnt - 1U);

	// 32-bit mode: neither half of the compare value is 0 (19.2.1)
	if (m32 && (cmp & 0xFFFFU) == 0U)
	{
		cmp |= 1U;
		cnt = (uint64_t)cmp + 1U;
	}

	uintptr_t state = DisableInterrupt();

	R9a02ModuleStart(R9A02_MSTPCR_D, R9A02_MSTPCRD_TML32);
	R9a02Tml32Unroute();
	R9a02Tml32Halt(s_R9a02Tml32.bInit ? s_R9a02Tml32.ClkHz : 0U);

	R9A02_TML32_Type *reg = R9A02_TML32;
	uint8_t md = m32 ? R9A02_ITLCTL0_MD_32BIT : R9A02_ITLCTL0_MD_16BIT;

	// Mode and clock with every channel off (19.2.3, 19.2.4)
	reg->ITLCTL0 = md;
	reg->ITLCSEL0 = (uint8_t)pCfg->ClkSrc;
	reg->ITLFDIV00 = (uint8_t)(div << R9A02_ITLFDIV_FDIV0_Pos);
	reg->ITLFDIV01 = 0U;
	reg->ITLCC0 = 0U;
	reg->ITLCMP00 = (uint16_t)cmp;
	reg->ITLCMP01 = m32 ? (uint16_t)(cmp >> 16) : 0xFFFFU;
	reg->ITLS0 = 0U;
	reg->ITLMKF0 = (uint8_t)(R9A02_ITLMKF0_Msk & ~R9A02_ITLS0_ITF00);

	s_R9a02Tml32.Dtc.Rsv = 0U;
	s_R9a02Tml32.Dtc.Mra = R9A02_DTC_MRA_MD_REPEAT | R9A02_DTC_MRA_SZ_BYTE | R9A02_DTC_MRA_SM_FIXED;
	s_R9a02Tml32.Dtc.Mrb = R9A02_DTC_MRB_DM_FIXED;
	s_R9a02Tml32.Dtc.Sar = (uint32_t)(uintptr_t)&s_R9a02Tml32Zero;
	s_R9a02Tml32.Dtc.Dar = (uint32_t)(uintptr_t)&reg->ITLS0;
	s_R9a02Tml32.Dtc.Crb = 0U;
	s_R9a02Tml32.Dtc.Cra = 0x0101U;
	s_R9a02Tml32.Handler = pCfg->Handler;
	s_R9a02Tml32.pCtx = pCfg->pCtx;

	bool ok;

	switch (pCfg->Mode)
	{
		case R9A02_TML32_MODE_INT:
			s_R9a02Tml32.IntNo = R9a02RegisterIntHandler(R9A02_EVT_TML32_OUTI, pCfg->IntPrio,
														 R9a02Tml32IrqHandler, NULL);
			ok = s_R9a02Tml32.IntNo >= 0 && (!pCfg->bWakeup || R9a02WakeupSet(R9A02_EVT_TML32_OUTI, true));
			break;
		case R9A02_TML32_MODE_DTC:
			s_R9a02Tml32.IntNo = R9a02RouteEvent(R9A02_EVT_TML32_OUTI);
			ok = R9a02Tml32DtcRoute(s_R9a02Tml32.IntNo);
			break;
		default:
			// The Snooze request is TML32_OUTI itself, the CPU wake-up is off,
			// it would take precedence (10.2.11). The flag is cleared at each
			// interval and again when Snooze starts, the DTC runs then.
			s_R9a02Tml32.IntNo = R9a02RouteEvent(R9A02_EVT_TML32_OUTI);
			s_R9a02Tml32.SnzIntNo = R9a02RouteEvent(R9A02_EVT_SYSTEM_SNZREQ);
			ok = R9a02Tml32DtcRoute(s_R9a02Tml32.IntNo) && R9a02Tml32DtcRoute(s_R9a02Tml32.SnzIntNo);
			break;
	}

	if (!ok)
	{
		R9a02Tml32Unroute();
		R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_TML32);
		s_R9a02Tml32.bInit = false;
		s_R9a02Tml32.nsPeriod = 0U;
		EnableInterrupt(state);

		return false;
	}

	s_R9a02Tml32.ClkHz = hz;
	uint64_t t = cnt << div;

	s_R9a02Tml32.nsPeriod = (t / hz) * 1000000000ULL + ((t % hz) * 1000000000ULL + hz / 2U) / hz;
	s_R9a02Tml32.Ctl0 = (uint8_t)(md | R9A02_ITLCTL0_EN0);
	s_R9a02Tml32.bInit = true;

	reg->ITLCTL0 = s_R9a02Tml32.Ctl0;

	EnableInterrupt(state);

	return true;
}

uint64_t R9a02Tml32Period(void)
{
	return s_R9a02Tml32.bInit ? s_R9a02Tml32.nsPeriod : 0U;
}

bool R9a02Tml32Start(void)
{
	if (!s_R9a02Tml32.bInit)
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();
	R9A02_TML32_Type *reg = R9A02_TML32;

	// The counter is cleared only by EN0 going to 0 (19.2.3), stopped first
	R9a02Tml32Halt(s_R9a02Tml32.ClkHz);
	reg->ITLMKF0 = (uint8_t)(R9A02_ITLMKF0_Msk & ~R9A02_ITLS0_ITF00);
	reg->ITLCTL0 = s_R9a02Tml32.Ctl0;

	EnableInterrupt(state);

	return true;
}

void R9a02Tml32Stop(void)
{
	if (!s_R9a02Tml32.bInit)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();

	R9a02Tml32Halt(s_R9a02Tml32.ClkHz);
	if (s_R9a02Tml32.IntNo >= 0 && !R9a02DtcEnabled(s_R9a02Tml32.IntNo))
	{
		R9a02ClearPendingInt(s_R9a02Tml32.IntNo);
	}

	EnableInterrupt(state);
}

void R9a02Tml32PowerOff(void)
{
	uintptr_t state = DisableInterrupt();

	if (R9a02ModuleRunning(R9A02_MSTPCR_D, R9A02_MSTPCRD_TML32))
	{
		R9a02Tml32Halt(s_R9a02Tml32.bInit ? s_R9a02Tml32.ClkHz : 0U);
	}
	R9a02Tml32Unroute();
	R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_TML32);
	s_R9a02Tml32.bInit = false;
	s_R9a02Tml32.nsPeriod = 0U;

	EnableInterrupt(state);
}
