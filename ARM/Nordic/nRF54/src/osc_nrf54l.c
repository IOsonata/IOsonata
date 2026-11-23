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
	// Nordic DK
	// 32MHz, 8pF
	// 32.768kHz, 9pF
	.CoreOsc = { OSC_TYPE_XTAL,	32000000, 20, 80},
	.LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 90},
#endif
	.bUSBClk = false
};

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
			hfCap = nrf_ficr_hfxo_cap_calculate(NRF_FICR,
				g_McuOsc.CoreOsc.LoadCap * 100U);
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
			lfCap = nrf_ficr_lfxo_cap_calculate(NRF_FICR,
				g_McuOsc.LowPwrOsc.LoadCap * 100U);
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
