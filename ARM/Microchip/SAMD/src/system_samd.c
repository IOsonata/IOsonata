/**-------------------------------------------------------------------------
@file	system_samd.c

@brief	SAM D11/D21 system clock initialization

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <stdint.h>

#include "coredev/system_core_clock.h"

#if !defined(__SAMD11D14A__) && !defined(__SAMD21J18A__)
#error Unsupported SAM D device
#endif

#define SAMD_SYSCTRL_OSC8M		(*(volatile uint32_t *)0x40000820UL)
#define SAMD_OSC8M_PRESC_MASK	(3UL << 8)
#define SAMD_OSC8M_ENABLE		(1UL << 1)
#define SAMD_OSC8M_FREQ			8000000UL

// CPU clock after reset is OSC8M divided by eight.
uint32_t SystemCoreClock = 1000000UL;
uint32_t SystemnsDelayFactor = 1UL;

__attribute__((weak)) McuOsc_t g_McuOsc = {
	{ OSC_TYPE_RC, 1000000UL, 0UL, 0UL },
	{ OSC_TYPE_RC, 32768UL, 0UL, 0UL },
	false
};

void SystemInit(void)
{
	// The initial implementation retains the reset clock configuration.
	// External oscillators and USB clocks are not configured here.
}

void SystemCoreClockUpdate(void)
{
	uint32_t osc8m;
	uint32_t prescaler;

	osc8m = SAMD_SYSCTRL_OSC8M;
	if ((osc8m & SAMD_OSC8M_ENABLE) == 0UL)
	{
		return;
	}

	prescaler = (osc8m & SAMD_OSC8M_PRESC_MASK) >> 8;
	SystemCoreClock = SAMD_OSC8M_FREQ >> prescaler;
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

uint32_t SystemPeriphClockGet(int Idx)
{
	(void)Idx;
	return 0UL;
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	(void)Idx;
	(void)Freq;
	return 0UL;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	(void)ClkSrc;
	(void)OscFreq;
	return false;
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	(void)ClkSrc;
	(void)OscFreq;
	return false;
}

void SystemOscInit(void)
{
}
