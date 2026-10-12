/**-------------------------------------------------------------------------
@file	clock_test.c

@brief	LPC11Uxx clock decoding regression using vendor CMSIS registers.

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
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "LPC11Uxx.h"
#include "coredev/system_core_clock.h"

extern uint32_t SystemMainClkFreq;

static void Check(uint32_t Main, uint32_t Core)
{
	LPC_SYSCON_Type before;

	memcpy(&before, (const void *)LPC_SYSCON, sizeof(before));
	SystemCoreClockUpdate();
	assert(SystemMainClkFreq == Main);
	assert(SystemCoreClockGet() == Core);
	assert(memcmp(&before, (const void *)LPC_SYSCON, sizeof(before)) == 0);
}

int main(void)
{
	assert(mmap((void *)0x40048000, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x40048000);

	// Reset IRC source must report 12 MHz, not 6 MHz.
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	Check(12000000, 12000000);
	for (unsigned div = 1; div <= 255; div++)
	{
		LPC_SYSCON->SYSAHBCLKDIV = div;
		Check(12000000, 12000000 / div);
	}

	// PLL bypass ignores M and P. Both supported inputs are 12 MHz here.
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLIN;
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	LPC_SYSCON->SYSPLLCTRL = 0x7f;
	for (unsigned source = 0; source < 2; source++)
	{
		LPC_SYSCON->SYSPLLCLKSEL = source;
		Check(12000000, 12000000);
		LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLOUT;
		for (unsigned p = 0; p < 4; p++)
		{
			// Decode the full field range, including configurations above CPU limits.
			for (unsigned m = 1; m <= 32; m++)
			{
				LPC_SYSCON->SYSPLLCTRL = (p << 5) | (m - 1);
				Check(12000000 * m, 12000000 * m);
			}
		}
		LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLIN;
	}
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLOUT;
	LPC_SYSCON->SYSPLLCTRL = 0x23;
	LPC_SYSCON->SYSAHBCLKDIV = 2;
	Check(48000000, 24000000);

	// WDT uses its own analog source and divider, independently of SYSPLL.
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_WDT;
	LPC_SYSCON->WDTOSCCTRL = 1U << 5;
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	Check(300000, 300000);
	LPC_SYSCON->WDTOSCCTRL = (1U << 5) | 31U;
	Check(9375, 9375);
	LPC_SYSCON->WDTOSCCTRL = (2U << 5) | 2U;
	LPC_SYSCON->SYSAHBCLKDIV = 7;
	Check(175000, 25000);
	LPC_SYSCON->WDTOSCCTRL = 15U << 5;
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	Check(2300000, 2300000);
	LPC_SYSCON->WDTOSCCTRL = 0;
	Check(0, 0);

	// Disabled divider and reserved PLL sources cannot retain stale rates.
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_IRC;
	LPC_SYSCON->SYSAHBCLKDIV = 0;
	Check(12000000, 0);
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	for (unsigned source = 2; source < 4; source++)
	{
		LPC_SYSCON->SYSPLLCLKSEL = source;
		LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLIN;
		Check(0, 0);
		LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLOUT;
		Check(0, 0);
	}
	puts("LPC11Uxx clock decoding: PASS");
}
