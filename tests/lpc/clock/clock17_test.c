/**-------------------------------------------------------------------------
@file	clock17_test.c

@brief	LPC17xx clock decoding regression using vendor CMSIS registers.

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
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "LPC17xx.h"

extern uint32_t SystemClkFreq;
uint32_t SystemMicroSecLoopCnt;
static unsigned s_Errors;

static void Check(unsigned Source, uint32_t Input, uint32_t Status,
				  unsigned Div, uint32_t Expected)
{
	LPC_SC_TypeDef before;

	SystemClkFreq = Input;
	LPC_SC->CLKSRCSEL = Source;
	LPC_SC->CCLKCFG = Div - 1;
	memcpy((uint8_t *)LPC_SC + offsetof(LPC_SC_TypeDef, PLL0STAT), &Status, sizeof(Status));
	memcpy(&before, (const void *)LPC_SC, sizeof(before));
	SystemCoreClockUpdate();
	if (SystemCoreClock != Expected)
	{
		if (s_Errors < 8)
		{
			printf("source=%u input=%u status=%08x div=%u expected=%u actual=%u\n",
				Source, Input, Status, Div, Expected, SystemCoreClock);
		}
		s_Errors++;
	}
	assert(SystemMicroSecLoopCnt == SystemCoreClock / 16000000U);
	assert(memcmp(&before, (const void *)LPC_SC, sizeof(before)) == 0);
}

int main(void)
{
	assert(mmap((void *)LPC_SC_BASE, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)LPC_SC_BASE);

	// Valid main-oscillator settings: 9.6 GHz intermediate, 400 MHz CCO.
	Check(CLKSRCSEL_MAIN, 12000000, 0x03000000U | (23U << 16) | 399U, 4, 100000000);
	// UM10360 table 21 RTC multiplier: M=4272, N=1.
	Check(CLKSRCSEL_RTC, 0, 0x03000000U | 4271U, 4, 69992448);
	// Bit 15 is reserved and must not contribute to M.
	Check(CLKSRCSEL_IRC, 0, 0x03008000U | 59U, 4, 120000000);

	for (unsigned div = 1; div <= 256; div++)
	{
		// PLL contributes only when both enable and connect are set.
		for (unsigned state = 0; state < 3; state++)
		{
			Check(CLKSRCSEL_IRC, 0, (state << 24) | 59U, div, 4000000U / div);
			Check(CLKSRCSEL_MAIN, 16000000, state << 24, div, 16000000U / div);
			Check(CLKSRCSEL_RTC, 0, state << 24, div, 32768U / div);
		}
		Check(CLKSRCSEL_IRC, 0, 0x03000000U | 59U, div, 480000000U / div);
	}
	// Decode every M bit and N encoding independently of operating limits.
	for (unsigned bit = 0; bit < 15; bit++)
	{
		for (unsigned n = 1; n <= 256; n++)
		{
			uint32_t msel = 1U << bit;
			uint32_t expected = (uint32_t)(2ULL * 12000000U * (msel + 1) / n / 256);

			Check(CLKSRCSEL_MAIN, 12000000, 0x03000000U | ((n - 1) << 16) | msel,
				256, expected);
		}
	}
	printf("LPC17xx clock decoding: %u errors\n", s_Errors);
	return s_Errors != 0;
}
