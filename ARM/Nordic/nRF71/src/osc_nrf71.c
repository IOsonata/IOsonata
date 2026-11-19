/**-------------------------------------------------------------------------
@file	osc_nrf71.c

@brief	Oscillator configuration of the nRF71

Oscillator settings and configuration. Sets the internal load capacitance of
the 32.768 kHz crystal oscillator (LFXO): 0 to 25 pF in 1 pF steps, CLOAD
VAL0 holds 0 to 15 pF and VAL1 adds 10 pF, the encoding of nrf_lfxo_cload_set
in the nrfx HAL (hal/nrf_lfxo.h). The MDK has no load capacitance register
for the 64 MHz crystal oscillator (HFXO64M).

@author	Hoang Nguyen Hoan
@date	Nov. 17, 2026

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
#include "nrf.h"

#include "coredev/system_core_clock.h"

// Highest internal load of the LFXO in pF
#define NRF71_LFXO_CLOAD_MAX		25UL

// Overload this variable in application firmware to change oscillator
__WEAK McuOsc_t g_McuOsc = {
	// Nordic nRF7120 DK
	// 64MHz
	// 32.768kHz, 14pF internal load
	.CoreOsc = { OSC_TYPE_XTAL,	64000000, 20, 0},
	.LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 140},
	.bUSBClk = false
};

void SystemOscInit(void)
{
	// LoadCap is in 10 x pF, 0 for a crystal with external capacitors
	uint32_t cap = (g_McuOsc.LowPwrOsc.LoadCap + 5UL) / 10UL;
	uint32_t cload = 0;

	if (cap > NRF71_LFXO_CLOAD_MAX)
	{
		cap = NRF71_LFXO_CLOAD_MAX;
	}

	if (cap >= 10UL)
	{
		cload = LFXO_CLOAD_VAL1_Msk;
		cap -= 10UL;
	}

	cload |= (cap << LFXO_CLOAD_VAL0_Pos) & LFXO_CLOAD_VAL0_Msk;

	NRF_LFXO->CLOAD = cload;
}
