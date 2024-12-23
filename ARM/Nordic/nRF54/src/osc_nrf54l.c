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
#include "nrf_oscillators.h"

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
//	 INTCAP = (((CAPACITANCE-5.5)*(FICR->XOSC32MTRIM.SLOPE+791)) +
//	           FICR->XOSC32MTRIM.OFFSET*4)/256
	/* As specified in the nRF54L15 PS:
	 * CAPVALUE = (((CAPACITANCE-5.5)*(FICR->XOSC32MTRIM.SLOPE+791)) +
	 *              FICR->XOSC32MTRIM.OFFSET<<2)>>8;
	 * where CAPACITANCE is the desired total load capacitance value in pF,
	 * holding any value between 4.0 pF and 17.0 pF in 0.25 pF steps.
	 */
	uint32_t intcap = 0;
	if (g_McuOsc.CoreOsc.LoadCap > 0)
	{
		int32_t slope = (NRF_FICR->XOSC32MTRIM & FICR_XOSC32MTRIM_SLOPE_Msk) >> FICR_XOSC32MTRIM_SLOPE_Pos;
		slope = ((-(slope>>8))<<8) | slope;
		uint32_t offset = (NRF_FICR->XOSC32MTRIM & FICR_XOSC32MTRIM_OFFSET_Msk) >> FICR_XOSC32MTRIM_OFFSET_Pos;
		intcap = (uint32_t)((((g_McuOsc.CoreOsc.LoadCap * 2 - 40) - 55) * (slope + 791)) / 10 + (offset << 2))>>8;
	}
//	NRF_OSCILLATORS->XOSC32M.CONFIG.INTCAP = 32;//intcap;
	//intcap = NRF_OSCILLATORS->XOSC32M.CONFIG.INTCAP;

	//intcap = OSCILLATORS_HFXO_CAP_CALCULATE(NRF_FICR, (12));
	NRF_OSCILLATORS->XOSC32M.CONFIG.INTCAP = intcap;

	intcap = 0;
	if (g_McuOsc.LowPwrOsc.LoadCap > 0)
	{
		/* As specified in the nRF54L15 PS:
		 * CAPVALUE = round( (CAPACITANCE - 4) * (FICR->XOSC32KTRIM.SLOPE + 0.765625 * 2^9)/(2^9)
		 *            + FICR->XOSC32KTRIM.OFFSET/(2^6) );
		 * where CAPACITANCE is the desired capacitor value in pF, holding any
		 * value between 4 pF and 18 pF in 0.5 pF steps.
		 */
		int32_t slope = (NRF_FICR->XOSC32KTRIM & FICR_XOSC32KTRIM_SLOPE_Msk) >> FICR_XOSC32KTRIM_SLOPE_Pos;
		slope = ((-(slope>>8))<<8) | slope;
		uint32_t offset = (NRF_FICR->XOSC32KTRIM & FICR_XOSC32KTRIM_OFFSET_Msk) >> FICR_XOSC32KTRIM_OFFSET_Pos;

		intcap = (((g_McuOsc.LowPwrOsc.LoadCap - 40UL) * (uint32_t)(slope + 392) + 5) / 5120) + (offset >> 6);
	}
	NRF_OSCILLATORS->XOSC32KI.INTCAP = 20;//intcap;
	intcap = NRF_OSCILLATORS->XOSC32KI.INTCAP;
}

