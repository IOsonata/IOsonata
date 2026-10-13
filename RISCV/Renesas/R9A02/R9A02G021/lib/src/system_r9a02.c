/**-------------------------------------------------------------------------
@file	system_r9a02.c

@brief	R9A02G021 system initialization and clock control

Implements SystemInit, SystemCoreClockUpdate and the system_core_clock.h
functions for the Renesas R9A02G021 (RISC-V, CLIC), from the R9A02G021
User's Manual: Hardware R01UH1036EJ0110 chapters 8, 10 and 11.

After reset ICLK and PCLKB are MOCO / 16 (500 kHz) in middle-speed mode.
SystemInit sets the core clock from g_McuOsc.CoreOsc with ICLK = PCLKB =
the source, and the subsystem clock from g_McuOsc.LowPwrOsc. See
system_r9a02.h for the supported oscillators.

Operating power mode: high-speed above 24 MHz, middle-speed otherwise. The
code flash needs one wait cycle (MEMWAIT, FLDWAITR) above 32 MHz. With the
internal clock supply type B (OFS1.ICSATS = 0) ICLK and PCLKB are always
equal and HOCO 48 MHz is not allowed (6.2.2).

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2025

@license

MIT License

Copyright (c) 2025, I-SYST inc., all rights reserved

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
#include <stdbool.h>
#include <stddef.h>

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "r9a02g021.h"
#include "system_r9a02.h"
#include "interrupt_r9a02.h"

#define R9A02_EXTAL_FREQ_MIN		1000000UL	//!< Datasheet 2.3.3
#define R9A02_LOCO_START_US			100U		//!< tLOCO, manual 37.3
#define R9A02_MOCO_START_US			1U			//!< tMOCO, manual 37.3
#define R9A02_SOSC_STOP_US			200U		//!< 5 SOSC cycles before WUTMMCK0 = 1

#define R9A02_PRCR_MSK				(R9A02_PRCR_PRC0 | R9A02_PRCR_PRC1 | R9A02_PRCR_PRC3)

/// Core clock setting computed by SystemCoreClockSelect
typedef struct __R9a02_Clock_Plan {
	uint32_t SrcHz;			//!< Source frequency, ICLK = SrcHz
	uint8_t Cksel;			//!< SCKSCR.CKSEL
	uint8_t Hcfrq;			//!< HOCOCR2 value when Cksel is HOCO
	uint8_t Opcm;			//!< Operating power control mode
} R9a02ClockPlan_t;

// ICLK after reset is MOCO / 16. ResetEntry initializes this storage before
// calling SystemInit, including an application's strong g_McuOsc.
uint32_t SystemCoreClock = 500000UL;
uint64_t SystemCoreClockPeriodns = 2000ULL;
uint64_t SystemCoreClockPeriodus = 1ULL;
volatile uint32_t g_R9a02StartupError;

__attribute__((weak)) McuOsc_t g_McuOsc = {
	{ OSC_TYPE_RC, R9A02_HOCO_48MHZ, 0U, 0U },
	{ OSC_TYPE_RC, R9A02_LOCO_FREQ, 0U, 0U },
	false
};

// Nominal EXTAL frequency, the registers do not give it
static uint32_t s_ExtalFreq;

// PCLKB divider relative to ICLK, as a power of 2, from SystemPeriphClockSet
static uint8_t s_PclkbShift;

static inline uint32_t R9a02Mcycle(void)
{
	uint32_t v;

	__asm volatile ("csrr %0, mcycle" : "=r"(v));

	return v;
}

static bool R9a02Wait8(volatile uint8_t *pReg, uint8_t Mask, uint8_t Value)
{
	uint32_t n = R9A02_STARTUP_TIMEOUT;

	do {
		if ((*pReg & Mask) == Value)
		{
			return true;
		}
	} while (--n != 0U);

	return false;
}

static uint16_t R9a02Unprotect(uint16_t Bits)
{
	uint16_t prev = R9A02_SYSC_PRCR & R9A02_PRCR_MSK;

	R9A02_SYSC_PRCR = R9A02_PRCR_KEY | prev | Bits;

	return prev;
}

static void R9a02Protect(uint16_t Prev)
{
	R9A02_SYSC_PRCR = R9A02_PRCR_KEY | Prev;
}

// Internal clock supply type A lets ICLK and PCLKB differ, OFS1.ICSATS
static bool R9a02ClockTypeA(void)
{
	const volatile uint32_t *ofs1 = (const volatile uint32_t *)R9A02_OFS1_ADDR;

	// OFS1 is at 0x404. Hide the constant address from the compiler, which
	// takes an access below 4 KB for a null pointer access.
	__asm volatile ("" : "+r"(ofs1));

	return (*ofs1 & R9A02_OFS1_ICSATS) != 0U;
}

static uint32_t R9a02HocoHz(void)
{
	switch (R9A02_SYSC_HOCOCR2 & R9A02_HOCOCR2_HCFRQ1_Msk)
	{
		case R9A02_HOCOCR2_24MHZ:
			return R9A02_HOCO_24MHZ;
		case R9A02_HOCOCR2_32MHZ:
			return R9A02_HOCO_32MHZ;
		case R9A02_HOCOCR2_48MHZ:
			return R9A02_HOCO_48MHZ;
		default:
			return 0U;
	}
}

static uint32_t R9a02SourceHz(void)
{
	switch (R9A02_SYSC_SCKSCR & R9A02_SCKSCR_CKSEL_Msk)
	{
		case R9A02_SCKSCR_HOCO:
			return R9a02HocoHz();
		case R9A02_SCKSCR_MOCO:
			return R9A02_MOCO_FREQ;
		case R9A02_SCKSCR_LOCO:
			return R9A02_LOCO_FREQ;
		case R9A02_SCKSCR_EXTAL:
			return s_ExtalFreq;
		case R9A02_SCKSCR_SOSC:
			return R9A02_SOSC_FREQ;
		default:
			return 0U;
	}
}

static uint32_t R9a02IclkHz(void)
{
	uint32_t ick = (R9A02_SYSC_SCKDIVCR & R9A02_SCKDIVCR_ICK_Msk) >> R9A02_SCKDIVCR_ICK_Pos;

	return R9a02SourceHz() >> ick;
}

uint32_t R9a02HocoClockGet(void)
{
	if ((R9A02_SYSC_HOCOCR & R9A02_HOCOCR_HCSTP) || (R9A02_SYSC_OSCSF & R9A02_OSCSF_HOCOSF) == 0U)
	{
		return 0U;
	}

	return R9a02HocoHz();
}

uint32_t R9a02ExtalClockGet(void)
{
	return (R9A02_SYSC_MOSCCR & R9A02_MOSCCR_MOSTP) ? 0U : s_ExtalFreq;
}

void R9a02DelayUs(uint32_t Us)
{
	uint32_t hz = R9a02IclkHz();

	if (hz == 0U)
	{
		// Unknown EXTAL frequency, wait for the fastest ICLK
		hz = R9A02_ICLK_MAX;
	}

	uint64_t cycles = ((uint64_t)Us * hz + 999999ULL) / 1000000ULL;
	uint32_t start = R9a02Mcycle();

	// Wait in parts so the 32-bit counter difference never wraps
	while (cycles > 0U)
	{
		uint32_t part = cycles > 0x40000000ULL ? 0x40000000UL : (uint32_t)cycles;

		while ((uint32_t)(R9a02Mcycle() - start) < part)
		{
		}
		start += part;
		cycles -= part;
	}
}

static volatile uint32_t *R9a02MstpReg(R9A02_MSTPCR Reg)
{
	switch (Reg)
	{
		case R9A02_MSTPCR_A:
			return &R9A02_SYSC_MSTPCRA;
		case R9A02_MSTPCR_B:
			return &R9A02_MSTP_MSTPCRB;
		case R9A02_MSTPCR_C:
			return &R9A02_MSTP_MSTPCRC;
		case R9A02_MSTPCR_D:
			return &R9A02_MSTP_MSTPCRD;
		default:
			return NULL;
	}
}

void R9a02ModuleStart(R9A02_MSTPCR Reg, uint32_t Mask)
{
	volatile uint32_t *p = R9a02MstpReg(Reg);

	if (p == NULL)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	uint16_t prev = R9a02Unprotect(R9A02_PRCR_PRC1);

	// Reserved bits read as 1 and keep their required value 1
	*p &= ~Mask;
	(void)*p;

	R9a02Protect(prev);
	EnableInterrupt(state);
}

void R9a02ModuleStop(R9A02_MSTPCR Reg, uint32_t Mask)
{
	volatile uint32_t *p = R9a02MstpReg(Reg);

	if (p == NULL)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	uint16_t prev = R9a02Unprotect(R9A02_PRCR_PRC1);

	*p |= Mask;
	(void)*p;

	R9a02Protect(prev);
	EnableInterrupt(state);
}

bool R9a02ModuleRunning(R9A02_MSTPCR Reg, uint32_t Mask)
{
	volatile uint32_t *p = R9a02MstpReg(Reg);

	return p != NULL && (*p & Mask) == 0U;
}

static bool R9a02MakePlan(OSC_TYPE Type, uint32_t Freq, R9a02ClockPlan_t *pPlan)
{
	switch (Type)
	{
		case OSC_TYPE_RC:
			pPlan->SrcHz = Freq;
			pPlan->Cksel = R9A02_SCKSCR_HOCO;
			if (Freq == R9A02_HOCO_24MHZ)
			{
				pPlan->Hcfrq = R9A02_HOCOCR2_24MHZ;
			}
			else if (Freq == R9A02_HOCO_32MHZ)
			{
				pPlan->Hcfrq = R9A02_HOCOCR2_32MHZ;
			}
			else if (Freq == R9A02_HOCO_48MHZ && R9a02ClockTypeA())
			{
				pPlan->Hcfrq = R9A02_HOCOCR2_48MHZ;
			}
			else if (Freq == R9A02_MOCO_FREQ)
			{
				pPlan->Cksel = R9A02_SCKSCR_MOCO;
				pPlan->Hcfrq = 0U;
			}
			else
			{
				return false;
			}
			break;

		case OSC_TYPE_TCXO:
			if (Freq < R9A02_EXTAL_FREQ_MIN || Freq > R9A02_EXTAL_FREQ_MAX)
			{
				return false;
			}
			pPlan->SrcHz = Freq;
			pPlan->Cksel = R9A02_SCKSCR_EXTAL;
			pPlan->Hcfrq = 0U;
			break;

		default:
			// No main crystal oscillator on this part
			return false;
	}

	pPlan->Opcm = pPlan->SrcHz > R9A02_ICLK_MIDDLE_MAX ? R9A02_OPCCR_HIGH_SPEED : R9A02_OPCCR_MIDDLE_SPEED;

	return true;
}

// Operating power control mode change, manual 10.2.6 and 10.5.1
static bool R9a02SetOpcm(uint8_t Mode)
{
	if ((R9A02_SYSC_OPCCR & R9A02_OPCCR_OPCM_Msk) == Mode)
	{
		return true;
	}

	// OPCCR must not be written while the HOCO stabilization counter runs
	if ((R9A02_SYSC_HOCOCR & R9A02_HOCOCR_HCSTP) == 0U &&
		!R9a02Wait8(&R9A02_SYSC_OSCSF, R9A02_OSCSF_HOCOSF, R9A02_OSCSF_HOCOSF))
	{
		return false;
	}

	if (!R9a02Wait8(&R9A02_SYSC_OPCCR, R9A02_OPCCR_OPCMTSF, 0U))
	{
		return false;
	}

	R9A02_SYSC_OPCCR = Mode;

	return R9a02Wait8(&R9A02_SYSC_OPCCR, R9A02_OPCCR_OPCMTSF, 0U) &&
		   (R9A02_SYSC_OPCCR & R9A02_OPCCR_OPCM_Msk) == Mode;
}

static bool R9a02LeaveSubosc(void)
{
	if ((R9A02_SYSC_SOPCCR & R9A02_SOPCCR_SOPCM) == 0U)
	{
		return true;
	}

	if (!R9a02Wait8(&R9A02_SYSC_SOPCCR, R9A02_SOPCCR_SOPCMTSF, 0U))
	{
		return false;
	}

	R9A02_SYSC_SOPCCR = 0U;

	return R9a02Wait8(&R9A02_SYSC_SOPCCR, R9A02_SOPCCR_SOPCMTSF | R9A02_SOPCCR_SOPCM, 0U);
}

static bool R9a02StartMoco(void)
{
	if ((R9A02_SYSC_MOCOCR & R9A02_MOCOCR_MCSTP) == 0U)
	{
		return true;
	}

	R9A02_SYSC_MOCOCR = 0U;
	if (R9A02_SYSC_MOCOCR != 0U)
	{
		return false;
	}
	R9a02DelayUs(R9A02_MOCO_START_US);

	return true;
}

// HOCO must be stable before it is stopped and must read stopped before it
// is started again (8.2.8).
static bool R9a02StopHoco(void)
{
	if ((R9A02_SYSC_HOCOCR & R9A02_HOCOCR_HCSTP) == 0U)
	{
		if (!R9a02Wait8(&R9A02_SYSC_OSCSF, R9A02_OSCSF_HOCOSF, R9A02_OSCSF_HOCOSF))
		{
			return false;
		}
		R9A02_SYSC_HOCOCR = R9A02_HOCOCR_HCSTP;
	}

	return R9a02Wait8(&R9A02_SYSC_OSCSF, R9A02_OSCSF_HOCOSF, 0U);
}

static bool R9a02SetMemWait(bool bWait)
{
	uint8_t v = bWait ? 1U : 0U;

	R9A02_SYSC_MEMWAIT = v;
	R9A02_FLCN_FLDWAITR = v;

	return R9A02_SYSC_MEMWAIT == v && R9A02_FLCN_FLDWAITR == v;
}

// Called with interrupts disabled and PRC0, PRC1 write enabled. Every step
// keeps a valid clock: the operating mode is raised first and lowered last,
// the flash wait cycle is set before ICLK goes above 32 MHz and cleared after
// it comes back down, the source is switched before the divider so ICLK never
// goes above the target.
static R9a02StartupError_t R9a02ApplyPlan(const R9a02ClockPlan_t *pPlan)
{
	uint8_t cur = R9A02_SYSC_SCKSCR & R9A02_SCKSCR_CKSEL_Msk;
	bool typea = R9a02ClockTypeA();

	if (!R9a02LeaveSubosc())
	{
		return R9A02_STARTUP_POWER_MODE;
	}

	if (pPlan->Opcm < (R9A02_SYSC_OPCCR & R9A02_OPCCR_OPCM_Msk) && !R9a02SetOpcm(pPlan->Opcm))
	{
		return R9A02_STARTUP_POWER_MODE;
	}

	if (pPlan->Cksel == R9A02_SCKSCR_HOCO && (R9A02_SYSC_HOCOCR & R9A02_HOCOCR_HCSTP) == 0U &&
		(R9A02_SYSC_HOCOCR2 & R9A02_HOCOCR2_HCFRQ1_Msk) != pPlan->Hcfrq)
	{
		// The frequency of a running HOCO cannot change. Move off it first.
		if (cur == R9A02_SCKSCR_HOCO)
		{
			if (!R9a02StartMoco())
			{
				return R9A02_STARTUP_CLOCK;
			}
			R9A02_SYSC_SCKSCR = R9A02_SCKSCR_MOCO;
			if ((R9A02_SYSC_SCKSCR & R9A02_SCKSCR_CKSEL_Msk) != R9A02_SCKSCR_MOCO)
			{
				return R9A02_STARTUP_CLOCK;
			}
		}
		if (!R9a02StopHoco())
		{
			return R9A02_STARTUP_HOCO;
		}
	}

	switch (pPlan->Cksel)
	{
		case R9A02_SCKSCR_HOCO:
			if (R9A02_SYSC_HOCOCR & R9A02_HOCOCR_HCSTP)
			{
				if (!R9a02Wait8(&R9A02_SYSC_OSCSF, R9A02_OSCSF_HOCOSF, 0U))
				{
					return R9A02_STARTUP_HOCO;
				}
				R9A02_SYSC_HOCOCR2 = pPlan->Hcfrq;
				if ((R9A02_SYSC_HOCOCR2 & R9A02_HOCOCR2_HCFRQ1_Msk) != pPlan->Hcfrq)
				{
					return R9A02_STARTUP_HOCO;
				}
				R9A02_SYSC_HOCOCR = 0U;
			}
			if (!R9a02Wait8(&R9A02_SYSC_OSCSF, R9A02_OSCSF_HOCOSF, R9A02_OSCSF_HOCOSF))
			{
				return R9A02_STARTUP_HOCO;
			}
			break;

		case R9A02_SCKSCR_MOCO:
			if (!R9a02StartMoco())
			{
				return R9A02_STARTUP_CLOCK;
			}
			break;

		case R9A02_SCKSCR_EXTAL:
			if (R9A02_SYSC_MOSCCR & R9A02_MOSCCR_MOSTP)
			{
				R9A02_SYSC_MOSCCR = 0U;
				if (R9A02_SYSC_MOSCCR != 0U)
				{
					return R9A02_STARTUP_CLOCK;
				}
			}
			s_ExtalFreq = pPlan->SrcHz;
			break;

		default:
			return R9A02_STARTUP_BAD_OSC;
	}

	if (pPlan->SrcHz > R9A02_ICLK_NOWAIT_MAX && !R9a02SetMemWait(true))
	{
		return R9A02_STARTUP_FLASH_WAIT;
	}

	R9A02_SYSC_SCKSCR = pPlan->Cksel;
	if ((R9A02_SYSC_SCKSCR & R9A02_SCKSCR_CKSEL_Msk) != pPlan->Cksel)
	{
		return R9A02_STARTUP_CLOCK;
	}

	// Type B needs ICK and PCKB equal
	uint32_t pckb = typea ? s_PclkbShift : 0U;
	uint32_t divcr = (pckb << R9A02_SCKDIVCR_PCKB_Pos) | R9A02_SCKDIVCR_FIXED;

	R9A02_SYSC_SCKDIVCR = divcr;
	if (R9A02_SYSC_SCKDIVCR != divcr)
	{
		return R9A02_STARTUP_CLOCK;
	}

	if (pPlan->SrcHz <= R9A02_ICLK_NOWAIT_MAX && (R9A02_SYSC_MEMWAIT & R9A02_MEMWAIT_MEMWAIT) &&
		!R9a02SetMemWait(false))
	{
		return R9A02_STARTUP_FLASH_WAIT;
	}

	// Stop the previous source when it was the HOCO or EXTAL and is not used
	if (cur == R9A02_SCKSCR_HOCO && pPlan->Cksel != R9A02_SCKSCR_HOCO)
	{
		R9A02_SYSC_HOCOCR = R9A02_HOCOCR_HCSTP;
		(void)R9A02_SYSC_HOCOCR;
	}
	if (cur == R9A02_SCKSCR_EXTAL && pPlan->Cksel != R9A02_SCKSCR_EXTAL)
	{
		R9A02_SYSC_MOSCCR = R9A02_MOSCCR_MOSTP;
		(void)R9A02_SYSC_MOSCCR;
	}

	if (pPlan->Opcm > (R9A02_SYSC_OPCCR & R9A02_OPCCR_OPCM_Msk) && !R9a02SetOpcm(pPlan->Opcm))
	{
		return R9A02_STARTUP_POWER_MODE;
	}

	return R9A02_STARTUP_OK;
}

void SystemCoreClockUpdate(void)
{
	uint32_t hz = R9a02IclkHz();

	if (hz == 0U)
	{
		return;
	}

	SystemCoreClock = hz;
	SystemCoreClockPeriodus = (hz + 500000UL) / 1000000UL;
	SystemCoreClockPeriodns = (1000000000ULL + hz / 2U) / hz;
}

uint32_t SystemCoreClockGet(void)
{
	return R9a02IclkHz();
}

uint32_t SystemPeriphClockGet(int Idx)
{
	if (Idx != 0)
	{
		return 0U;
	}

	return R9a02SourceHz() >> ((R9A02_SYSC_SCKDIVCR & R9A02_SCKDIVCR_PCKB_Msk) >> R9A02_SCKDIVCR_PCKB_Pos);
}

// Idx 0 is PCLKB, the only peripheral clock. Sets the closest PCLKB at or
// below ICLK with ICLK : PCLKB = 2^n : 1 (8.2.1) and returns it. Peripherals
// already running keep their divider settings, set PCLKB before their Init.
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	if (Idx != 0 || Freq == 0U)
	{
		return 0U;
	}

	uintptr_t state = DisableInterrupt();
	uint32_t src = R9a02SourceHz();
	uint32_t ick = (R9A02_SYSC_SCKDIVCR & R9A02_SCKDIVCR_ICK_Msk) >> R9A02_SCKDIVCR_ICK_Pos;

	if (src != 0U && R9a02ClockTypeA())
	{
		uint32_t best = ick;
		uint32_t besterr = UINT32_MAX;

		for (uint32_t k = ick; k <= R9A02_SCKDIVCR_DIV_MAX; k++)
		{
			uint32_t f = src >> k;
			uint32_t err = f > Freq ? f - Freq : Freq - f;

			if (err < besterr)
			{
				best = k;
				besterr = err;
			}
		}

		uint16_t prev = R9a02Unprotect(R9A02_PRCR_PRC0);

		R9A02_SYSC_SCKDIVCR = (ick << R9A02_SCKDIVCR_ICK_Pos) | (best << R9A02_SCKDIVCR_PCKB_Pos) |
							  R9A02_SCKDIVCR_FIXED;
		R9a02Protect(prev);

		s_PclkbShift = (uint8_t)(best - ick);
	}

	uint32_t actual = SystemPeriphClockGet(0);

	EnableInterrupt(state);

	return actual;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	R9a02ClockPlan_t plan;

	if (!R9a02MakePlan(ClkSrc, OscFreq, &plan))
	{
		g_R9a02StartupError = R9A02_STARTUP_BAD_OSC;

		return false;
	}

	uintptr_t state = DisableInterrupt();
	uint16_t prev = R9a02Unprotect(R9A02_PRCR_PRC0 | R9A02_PRCR_PRC1);
	R9a02StartupError_t err = R9A02_STARTUP_PROTECT;

	if ((R9A02_SYSC_PRCR & (R9A02_PRCR_PRC0 | R9A02_PRCR_PRC1)) == (R9A02_PRCR_PRC0 | R9A02_PRCR_PRC1))
	{
		err = R9a02ApplyPlan(&plan);
	}

	R9a02Protect(prev);

	if (err == R9A02_STARTUP_OK)
	{
		g_McuOsc.CoreOsc.Type = ClkSrc;
		g_McuOsc.CoreOsc.Freq = OscFreq;
	}
	g_R9a02StartupError = err;
	SystemCoreClockUpdate();
	EnableInterrupt(state);

	return err == R9A02_STARTUP_OK;
}

// Clock of RTC, TML32, UARTA and REMC (OSMCR.WUTMMCK0, 8.2.18). Change it
// only while those are stopped, a running RTC is stopped first and
// R9a02RtcInit starts it again. An RTC counting on the SOSC keeps it: the RTC
// is not reset by a reset other than the power-on reset (table 20.2), the
// LOCO request of a warm start would stop the calendar. Table 5.3 lists RTCC0
// as reset by every reset, RTCE then reads 0 and the RTC starts again.
static void R9a02LfClockRtcStop(void)
{
	if (R9A02_RTC->RTCC0 & R9A02_RTCC0_RTCE)
	{
		R9A02_RTC->RTCC0 = (uint8_t)(R9A02_RTC->RTCC0 & ~R9A02_RTCC0_RTCE);
	}
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if (OscFreq != R9A02_LOCO_FREQ || (ClkSrc != OSC_TYPE_RC && ClkSrc != OSC_TYPE_XTAL))
	{
		g_R9a02StartupError = R9A02_STARTUP_BAD_LF_OSC;

		return false;
	}

	if (ClkSrc == OSC_TYPE_RC && (R9A02_RTC->RTCC0 & R9A02_RTCC0_RTCE) &&
		(R9A02_SYSC_OSMCR & R9A02_OSMCR_WUTMMCK0) == 0U && (R9A02_SYSC_SOSCCR & R9A02_SOSCCR_SOSTP) == 0U)
	{
		ClkSrc = OSC_TYPE_XTAL;
	}

	uintptr_t state = DisableInterrupt();
	uint16_t prev = R9a02Unprotect(R9A02_PRCR_PRC0);
	bool ok = (R9A02_SYSC_PRCR & R9A02_PRCR_PRC0) != 0U;

	if (ok && ClkSrc == OSC_TYPE_RC)
	{
		if (R9A02_SYSC_LOCOCR & R9A02_LOCOCR_LCSTP)
		{
			R9A02_SYSC_LOCOCR = 0U;
			ok = R9A02_SYSC_LOCOCR == 0U;
			if (ok)
			{
				R9a02DelayUs(R9A02_LOCO_START_US);
			}
		}
		if (ok && (R9A02_SYSC_OSMCR & R9A02_OSMCR_WUTMMCK0) == 0U)
		{
			// LOCO is not selectable while SOSC oscillates
			if ((R9A02_SYSC_SOSCCR & R9A02_SOSCCR_SOSTP) == 0U)
			{
				ok = (R9A02_SYSC_SCKSCR & R9A02_SCKSCR_CKSEL_Msk) != R9A02_SCKSCR_SOSC;
				if (ok)
				{
					R9A02_SYSC_SOSCCR = R9A02_SOSCCR_SOSTP;
					ok = R9A02_SYSC_SOSCCR == R9A02_SOSCCR_SOSTP;
					R9a02DelayUs(R9A02_SOSC_STOP_US);
				}
			}
			if (ok)
			{
				R9a02LfClockRtcStop();
				R9A02_SYSC_OSMCR = R9A02_OSMCR_WUTMMCK0;
				ok = R9A02_SYSC_OSMCR == R9A02_OSMCR_WUTMMCK0;
			}
		}
	}
	else if (ok)
	{
		if (R9A02_SYSC_SOSCCR & R9A02_SOSCCR_SOSTP)
		{
			// Normal drive. SOMCR is written only while SOSC is stopped.
			R9A02_SYSC_SOMCR = 0U;
			R9A02_SYSC_SOSCCR = 0U;
			ok = R9A02_SYSC_SOSCCR == 0U;
			if (ok)
			{
				R9a02DelayUs(R9A02_SOSC_STARTUP_US);
			}
		}
		if (ok && (R9A02_SYSC_OSMCR & R9A02_OSMCR_WUTMMCK0))
		{
			R9a02LfClockRtcStop();
			R9A02_SYSC_OSMCR = 0U;
			ok = R9A02_SYSC_OSMCR == 0U;
		}
	}

	R9a02Protect(prev);

	if (ok)
	{
		g_McuOsc.LowPwrOsc.Type = ClkSrc;
		g_McuOsc.LowPwrOsc.Freq = OscFreq;
	}
	g_R9a02StartupError = ok ? R9A02_STARTUP_OK : R9A02_STARTUP_LF_CLOCK;
	EnableInterrupt(state);

	return ok;
}

// Applies g_McuOsc: core clock, then subsystem clock
void SystemOscInit(void)
{
	if (SystemCoreClockSelect(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq))
	{
		SystemLowFreqClockSelect(g_McuOsc.LowPwrOsc.Type, g_McuOsc.LowPwrOsc.Freq);
	}
}

void *SystemRamStart(void)
{
	extern unsigned long __data_start__;

	return (void *)&__data_start__;
}

// Module-stop function for an unused circuit, manual 10.9.13
static void R9a02UnusedCircuitInit(void)
{
	uint16_t prev = R9a02Unprotect(R9A02_PRCR_PRC1);

	R9A02_MSTP_MSTPCRC &= ~R9A02_MSTPCRC_TRNG;

	// Wait 3 PCLKB cycles, each peripheral read takes at least one
	(void)R9A02_PORT(1)->PODR;
	(void)R9A02_PORT(1)->PODR;
	(void)R9A02_PORT(1)->PODR;

	R9A02_MSTP_MSTPCRC |= R9A02_MSTPCRC_TRNG;

	R9a02Protect(prev);
}

static void R9a02StartupStop(void)
{
	__asm volatile ("csrci mstatus, 8" ::: "memory");
	for (;;)
	{
		__asm volatile ("wfi");
	}
}

void SystemInit(void)
{
	g_R9a02StartupError = R9A02_STARTUP_OK;

	// Trap and interrupt entries first, so a fault in the rest of startup
	// stops in R9a02ExceptionHandler.
	R9a02TrapInit();

	R9a02UnusedCircuitInit();

	SystemOscInit();
	if (g_R9a02StartupError != R9A02_STARTUP_OK)
	{
		R9a02StartupStop();
	}

	SystemCoreClockUpdate();

	// mstatus.MPP must hold 11b (2.7.2.3.6). Interrupts on, as on Cortex-M
	// after reset: every CLIC input is still disabled.
	__asm volatile (
		"li		t0, 0x1800\n"
		"csrs	mstatus, t0\n"
		"csrsi	mstatus, 8\n"
		::: "t0", "memory");
}

// Software Standby wake-up enable bit of an ICU event (table 10.3, 12.2.7,
// 12.2.8). Returns false for an event that cannot end Software Standby.
static bool R9a02WakeupBit(uint8_t EvtId, volatile uint32_t **ppReg, uint32_t *pBit)
{
	volatile uint32_t *wup0 = &R9A02_ICU->WUPEN0;
	volatile uint32_t *wup1 = &R9A02_ICU->WUPEN1;

	if (EvtId >= R9A02_EVT_PORT_IRQ0 && EvtId <= R9A02_EVT_PORT_IRQ7)
	{
		*ppReg = wup0;
		*pBit = R9A02_WUPEN0_IRQ(EvtId - R9A02_EVT_PORT_IRQ0);

		return true;
	}

	switch (EvtId)
	{
		case R9A02_EVT_IWDT_NMIUNDF:
			*ppReg = wup0;
			*pBit = R9A02_WUPEN0_IWDT;
			break;
		case R9A02_EVT_KEY_INTKR:
			*ppReg = wup0;
			*pBit = R9A02_WUPEN0_KEY;
			break;
		case R9A02_EVT_LVD_LVD1:
			*ppReg = wup0;
			*pBit = R9A02_WUPEN0_LVD1;
			break;
		case R9A02_EVT_LVD_LVD2:
			*ppReg = wup0;
			*pBit = R9A02_WUPEN0_LVD2;
			break;
		case R9A02_EVT_RTC_ALM_OR_PRD:
			*ppReg = wup0;
			*pBit = R9A02_WUPEN0_RTC;
			break;
		case R9A02_EVT_TML32_OUTI:
			*ppReg = wup0;
			*pBit = R9A02_WUPEN0_TML32;
			break;
		case R9A02_EVT_IIC0_ENDI:
			*ppReg = wup1;
			*pBit = R9A02_WUPEN1_IICA0;
			break;
		case R9A02_EVT_IIC1_ENDI:
			*ppReg = wup1;
			*pBit = R9A02_WUPEN1_IICA1;
			break;
		case R9A02_EVT_UARTA_RX_ENDI0:
			*ppReg = wup1;
			*pBit = R9A02_WUPEN1_UARTA_RX_ENDI0;
			break;
		case R9A02_EVT_UARTA_RX_ERI0:
			*ppReg = wup1;
			*pBit = R9A02_WUPEN1_UARTA_RX_ERI0;
			break;
		case R9A02_EVT_UARTA_RX_ENDI1:
			*ppReg = wup1;
			*pBit = R9A02_WUPEN1_UARTA_RX_ENDI1;
			break;
		case R9A02_EVT_UARTA_RX_ERI1:
			*ppReg = wup1;
			*pBit = R9A02_WUPEN1_UARTA_RX_ERI1;
			break;
		case R9A02_EVT_COMP_DET0:
			*ppReg = wup1;
			*pBit = R9A02_WUPEN1_COMP_DET0;
			break;
		case R9A02_EVT_COMP_DET1:
			*ppReg = wup1;
			*pBit = R9A02_WUPEN1_COMP_DET1;
			break;
		default:
			return false;
	}

	return true;
}

bool R9a02WakeupSet(uint8_t EvtId, bool bEnable)
{
	volatile uint32_t *reg;
	uint32_t bit;

	if (!R9a02WakeupBit(EvtId, &reg, &bit))
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();

	*reg = bEnable ? (*reg | bit) : (*reg & ~bit);

	EnableInterrupt(state);

	return true;
}

// Sleep or Software Standby until an interrupt (10.6, 10.7). SBYCR.SSBY is
// set back to 0 after the wake-up, it stays 1 otherwise (10.2.1).
bool R9a02LowPowerEnter(bool bStandby)
{
	// No SYSTEM register write during flash P/E (10.9.1)
	if (R9A02_FLCN_FENTRYR != 0U)
	{
		return false;
	}

	// Software Standby: HOCOSF at 1 for a running HOCO, at 0 for a stopped
	// one (8.2.8)
	uint8_t hocosf = (R9A02_SYSC_HOCOCR & R9A02_HOCOCR_HCSTP) ? 0U : R9A02_OSCSF_HOCOSF;

	if ((bStandby && !R9a02Wait8(&R9A02_SYSC_OSCSF, R9A02_OSCSF_HOCOSF, hocosf)) ||
		!R9a02Wait8(&R9A02_SYSC_OPCCR, R9A02_OPCCR_OPCMTSF, 0U) ||
		!R9a02Wait8(&R9A02_SYSC_SOPCCR, R9A02_SOPCCR_SOPCMTSF, 0U))
	{
		return false;
	}

	uint16_t sbycr = bStandby ? R9A02_SBYCR_SSBY : 0U;
	uint16_t prev = R9a02Unprotect(R9A02_PRCR_PRC1);

	if (R9A02_SYSC_SBYCR != sbycr)
	{
		R9A02_SYSC_SBYCR = sbycr;
	}
	R9a02Protect(prev);

	// The last register write is done before WFI (10.9.5)
	(void)R9A02_SYSC_SBYCR;
	(void)R9A02_SYSC_PRCR;

	__asm volatile ("wfi" ::: "memory");

	if (bStandby)
	{
		prev = R9a02Unprotect(R9A02_PRCR_PRC1);
		R9A02_SYSC_SBYCR = 0U;
		R9a02Protect(prev);
	}

	return true;
}
