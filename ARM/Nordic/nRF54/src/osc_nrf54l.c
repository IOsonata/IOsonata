/**-------------------------------------------------------------------------
@file	osc_nf54l.c

@brief	Oscillator configuration

Oscillator settings & configuration.  Setup internal capacitor value

@author	Nguyen Hoan Hoang
@date	Dec. 21, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Softare.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include "nrf.h"
#include "hal/nrf_ficr.h"

#include "coredev/system_core_clock.h"

// Overload this variable in application firmware to change oscillator
__WEAK McuOsc_t g_McuOsc = {
#if 0
	// BLYSTL15
	.CoreOsc = { OSC_TYPE_XTAL,	32000000, 10, 100},
	.LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 70},
#else
	// Nordic nRF54L15 DK and IBK-NRF54L15: same crystal specifications.
	// HFXO: 32 MHz, 8 pF; LFXO: 32.768 kHz, 9 pF.
	.CoreOsc = { OSC_TYPE_XTAL,	32000000, 20, 80},
	.LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 90},
#endif
	.bUSBClk = false
};

// nrfx before 4.7.0 does not declare the FICR capacitance helpers.
// Match Nordic's signed trim decoding and femtofarad calculations.
#if !defined(FICR_HFXO_CAP_CALCULATE) || !defined(FICR_LFXO_CAP_CALCULATE)
static int32_t nRF54LTrimSlope(uint32_t Trim, uint32_t Mask, uint32_t Shift)
{
	uint32_t field;
	uint32_t mask;
	uint32_t sign;

	field = (Trim & Mask) >> Shift;
	mask = Mask >> Shift;
	sign = mask - (mask >> 1);

	return (int32_t)(field ^ sign) - (int32_t)sign;
}
#endif

#if !defined(FICR_HFXO_CAP_CALCULATE)
static uint32_t nRF54LHfxoCapCalculate(uint32_t CapFf)
{
	uint32_t trim;
	uint32_t offset;
	int32_t slope;
	int64_t scaled;

	trim = NRF_FICR->XOSC32MTRIM;
	slope = nRF54LTrimSlope(trim, FICR_XOSC32MTRIM_SLOPE_Msk,
		FICR_XOSC32MTRIM_SLOPE_Pos);
	offset = (trim & FICR_XOSC32MTRIM_OFFSET_Msk) >>
		FICR_XOSC32MTRIM_OFFSET_Pos;
	scaled = ((int64_t)CapFf - 5500LL) * (slope + 791) +
		(int64_t)(offset << 2) * 1000LL;
	if (scaled <= 0LL)
	{
		return 0U;
	}
	scaled >>= 8;

	return (uint32_t)((scaled + 500LL) / 1000LL);
}
#endif

#if !defined(FICR_LFXO_CAP_CALCULATE)
static uint32_t nRF54LLfxoCapCalculate(uint32_t CapFf)
{
	uint32_t trim;
	uint32_t offset;
	int32_t slope;
	int64_t scaled;

	trim = NRF_FICR->XOSC32KTRIM;
	slope = nRF54LTrimSlope(trim, FICR_XOSC32KTRIM_SLOPE_Msk,
		FICR_XOSC32KTRIM_SLOPE_Pos);
	offset = (trim & FICR_XOSC32KTRIM_OFFSET_Msk) >>
		FICR_XOSC32KTRIM_OFFSET_Pos;
	scaled = (2LL * CapFf - 12000LL) * (slope + 392) +
		(int64_t)(offset << 3) * 1000LL;
	if (scaled <= 0LL)
	{
		return 0U;
	}

	return (uint32_t)((scaled + 256000LL) / 512000LL);
}
#endif

void SystemOscInit(void)
{
	uint32_t hfCap;
	uint32_t lfCap;
	uint32_t lfStat;
	bool lfRunning;

	// The board describes load capacitance in tenths of a pF.
	// Nordic FICR calibration takes femtofarads: 0.1 pF = 100 fF.
	// An unpopulated/internal-capacitor-free board uses zero.
	if (g_McuOsc.CoreOsc.Type == OSC_TYPE_XTAL)
	{
		hfCap = 0U;
		if (g_McuOsc.CoreOsc.LoadCap != 0U)
		{
#if defined(FICR_HFXO_CAP_CALCULATE)
			hfCap = nrf_ficr_hfxo_cap_calculate(NRF_FICR,
				g_McuOsc.CoreOsc.LoadCap * 100U);
#else
			hfCap = nRF54LHfxoCapCalculate(
				g_McuOsc.CoreOsc.LoadCap * 100U);
#endif
		}

		// Do not modify an already-running HFXO owned by a bootloader
		// or another clock user. This call does not start an oscillator.
#if defined(CLOCK_XO_STAT_STATE_Msk)
		if ((NRF_CLOCK_S->XO.STAT & CLOCK_XO_STAT_STATE_Msk) == 0U &&
			NRF_OSCILLATORS->XOSC32M.CONFIG.INTCAP != hfCap)
#else
		if (NRF_OSCILLATORS->XOSC32M.CONFIG.INTCAP != hfCap)
#endif
		{
			NRF_OSCILLATORS->XOSC32M.CONFIG.INTCAP = hfCap;
		}
	}

	if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL)
	{
		lfCap = 0U;
		if (g_McuOsc.LowPwrOsc.LoadCap != 0U)
		{
#if defined(FICR_LFXO_CAP_CALCULATE)
			lfCap = nrf_ficr_lfxo_cap_calculate(NRF_FICR,
				g_McuOsc.LowPwrOsc.LoadCap * 100U);
#else
			lfCap = nRF54LLfxoCapCalculate(
				g_McuOsc.LowPwrOsc.LoadCap * 100U);
#endif
		}

		// GRTC can run directly from LFXO while system LFCLK is off.
		// Preserve the capacitor configuration if either user is active.
		lfStat = NRF_CLOCK_S->LFCLK.STAT;
		lfRunning = (lfStat & CLOCK_LFCLK_STAT_STATE_Msk) != 0U &&
			(lfStat & CLOCK_LFCLK_STAT_SRC_Msk) ==
			(CLOCK_LFCLK_SRC_SRC_LFXO << CLOCK_LFCLK_STAT_SRC_Pos);
		if ((NRF_GRTC->MODE & GRTC_MODE_SYSCOUNTEREN_Msk) != 0U &&
			((NRF_GRTC->CLKCFG & GRTC_CLKCFG_CLKSEL_Msk) >>
			GRTC_CLKCFG_CLKSEL_Pos) == GRTC_CLKCFG_CLKSEL_LFXO)
		{
			lfRunning = true;
		}
		if (!lfRunning && NRF_OSCILLATORS->XOSC32KI.INTCAP != lfCap)
		{
			NRF_OSCILLATORS->XOSC32KI.INTCAP = lfCap;
		}
	}
}
