/**-------------------------------------------------------------------------
@file	clock_link_test.c

@brief	C and C++ linkage regression for LPC11Uxx clock setters.

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
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "LPC11Uxx.h"

typedef uint32_t (*ClockSetter_t)(bool, int);

static ClockSetter_t const s_Setters[] = {
	SystemSetCoreClock,
	SystemCoreClockSet,
};

int main(void)
{
	void *regs = mmap((void *)0x40048000, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	uint32_t locked = 1;

	assert(regs == (void *)0x40048000);
	for (unsigned i = 0; i < sizeof(s_Setters) / sizeof(s_Setters[0]); i++)
	{
		for (unsigned crystal = 0; crystal < 2; crystal++)
		{
			memset(regs, 0, 0x1000);
			// Supply lock indications in the model; no analog PLL is simulated.
			memcpy((char *)regs + offsetof(LPC_SYSCON_Type, SYSPLLSTAT), &locked, sizeof(locked));
			memcpy((char *)regs + offsetof(LPC_SYSCON_Type, USBPLLSTAT), &locked, sizeof(locked));
			assert(s_Setters[i](crystal != 0, 12000000) == 48000000);
			assert(LPC_SYSCON->MAINCLKSEL == MAINCLKSEL_PLLOUT);
			assert(LPC_SYSCON->SYSAHBCLKDIV == 2);
			assert(LPC_SYSCON->SYSPLLCLKSEL == crystal);
			assert(SystemCoreClock == 48000000);
			SystemCoreClockUpdate();
			assert(SystemCoreClock == 48000000);
		}
	}
	puts("LPC11Uxx clock setter linkage: PASS");
}
