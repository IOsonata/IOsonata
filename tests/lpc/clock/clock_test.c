/**-------------------------------------------------------------------------
@file	clock_test.c

@brief	LPC11Uxx CPU clock from the stored main clock, vendor CMSIS registers.

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

uint32_t SystemMicroSecLoopCnt;

extern uint32_t SystemMainClkFreq;

// The CPU clock is the stored main clock divided by SYSAHBCLKDIV. No register
// is written and the clock source registers are not read.
static void Check(uint32_t Main, uint32_t Div)
{
	LPC_SYSCON_Type before;
	uint32_t loops;
	uint32_t core;

	core = Div != 0U ? Main / Div : 0U;
	SystemMainClkFreq = Main;
	LPC_SYSCON->SYSAHBCLKDIV = Div;
	memcpy(&before, (const void *)LPC_SYSCON, sizeof(before));
	loops = SystemMicroSecLoopCnt;
	SystemCoreClockUpdate();
	assert(SystemMainClkFreq == Main);
	assert(SystemCoreClockGet() == core);
	// ArmDelayClockSetDivisors keeps the delay factors of the last running
	// clock when the core clock is stopped.
	if (core == 0)
	{
		assert(SystemMicroSecLoopCnt == loops);
	}
	else
	{
		assert(SystemMicroSecLoopCnt == ((uint64_t)core * 1000000ULL) / 16000000U);
	}
	assert(SystemPeriphClockGet(0) == core);
	assert(SystemPeriphClockGet(-1) == 0);
	assert(SystemPeriphClockGet(1) == 0);
	// The frequency in effect, the bus clock does not change
	assert(SystemPeriphClockSet(0, 1000000) == core);
	assert(SystemPeriphClockSet(0, 0) == core);
	assert(SystemPeriphClockSet(1, 1000000) == 0);
	assert(SystemPeriphClockSet(-1, 1000000) == 0);
	assert(SystemCoreClockGet() == core);
	assert(memcmp(&before, (const void *)LPC_SYSCON, sizeof(before)) == 0);
}

int main(void)
{
	static const uint32_t mains[] = { 12000000, 48000000, 96000000, 9375 };

	assert(mmap((void *)0x40048000, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x40048000);

	assert(GetHighFreqOscType() == OSC_TYPE_XTAL);
	assert(GetHighFreqOscFreq() == 12000000);
	assert(g_McuOsc.bUSBClk);
	// Reset value: the IRC at 12 MHz, divider 1
	assert(SystemMainClkFreq == 12000000);
	assert(SystemCoreClock == 12000000);

	for (unsigned i = 0; i < sizeof(mains) / sizeof(mains[0]); i++)
	{
		for (unsigned div = 0; div <= 255; div++)
		{
			Check(mains[i], div);
		}
	}

	// Clock source registers do not change the stored frequency
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_WDT;
	LPC_SYSCON->WDTOSCCTRL = 1U << 5;
	LPC_SYSCON->PDRUNCFG = PDRUNCFG_IRC_PD | PDRUNCFG_IRCOUT_PD | PDRUNCFG_SYSPLL_PD;
	Check(48000000, 2);
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLOUT;
	LPC_SYSCON->SYSPLLCTRL = 0x1f;
	Check(48000000, 1);
	Check(0, 1);
	puts("LPC11Uxx clock from the stored main clock: PASS");
}
