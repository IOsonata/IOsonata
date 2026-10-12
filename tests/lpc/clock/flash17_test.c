/**-------------------------------------------------------------------------
@file	flash17_test.c

@brief	LPC17xx flash timing regression using vendor CMSIS registers.

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
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "flash17_shim.h"

uint32_t SystemMicroSecLoopCnt;
static LPC_SC_TypeDef s_Regs;
static unsigned s_Unsafe;
static unsigned s_Checks;
static unsigned s_OscWait;
static unsigned s_OscReadyAfter;
static unsigned s_EarlyClockChange;
static const uint32_t s_Rates[] = {
	20000000, 22000000, 40000000, 42000000, 60000000, 62000000,
	80000000, 82000000, 99000000, 100000000, 101000000, 102000000,
	119000000, 120000000
};
static const int s_InvalidInputs[] = {
	0, -1, INT_MIN, 1, 32767, 32769, 999999, 25000001, INT_MAX
};
extern uint32_t SystemClkFreq;
extern uint32_t SystemCoreClockSet(uint32_t CoreFreq, int ClkFreq);

LPC_SC_TypeDef *TestSystemRegs(void)
{
	uint32_t status = s_Regs.PLL0CFG | (7U << 24);

	if ((s_Regs.SCS & 0x60U) == 0x20U)
	{
		if (s_Regs.CCLKCFG != 0 || s_Regs.CLKSRCSEL != 0 || s_Regs.PLL0CON != 0)
		{
			s_EarlyClockChange++;
		}
		s_OscWait++;
		assert(s_OscWait < 2000000U);
		if (s_OscReadyAfter != 0 && s_OscWait >= s_OscReadyAfter)
		{
			s_Regs.SCS |= 0x40U;
		}
	}

	// Model ready status from configuration; analog locking/feed are not modeled.
	memcpy((uint8_t *)&s_Regs + offsetof(LPC_SC_TypeDef, PLL0STAT), &status, sizeof(status));
	if (s_Regs.CCLKCFG != 0 || s_Regs.PLL0CON != 0)
	{
		s_Checks++;
		if ((s_Regs.FLASHCFG & 0xF000U) != 0x5000U)
		{
			s_Unsafe++;
		}
	}
	return &s_Regs;
}

int main(void)
{
	unsigned errors = 0;

	// Invalid physical oscillator inputs must fail before register/global changes.
	for (unsigned i = 0; i < sizeof(s_InvalidInputs) / sizeof(s_InvalidInputs[0]); i++)
	{
		LPC_SC_TypeDef before;
		uint32_t core = SystemCoreClock;
		uint32_t source = SystemClkFreq;
		uint32_t delay = SystemMicroSecLoopCnt;

		memcpy(&before, &s_Regs, sizeof(before));
		assert(SystemCoreClockSet(120000000, s_InvalidInputs[i]) == 0);
		assert(memcmp(&before, &s_Regs, sizeof(before)) == 0);
		assert(SystemCoreClock == core);
		assert(SystemClkFreq == source);
		assert(SystemMicroSecLoopCnt == delay);
	}

	for (unsigned i = 0; i < sizeof(s_Rates) / sizeof(s_Rates[0]); i++)
	{
		uint32_t rate;
		uint32_t timing;

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.FLASHCFG = 0xA500003AU;
		s_Unsafe = 0;
		s_Checks = 0;
		rate = SystemCoreClockSet(s_Rates[i], 4000000);
		assert(rate == s_Rates[i]);
		if (rate == 99000000)
		{
			assert(s_Regs.PLL0CFG == ((7U << 16) | 296U));
			assert(s_Regs.CCLKCFG == 2U);
		}
		if (rate == 101000000)
		{
			assert(s_Regs.PLL0CFG == ((7U << 16) | 302U));
			assert(s_Regs.CCLKCFG == 2U);
		}
		if (rate == 119000000)
		{
			assert(s_Regs.PLL0CFG == ((7U << 16) | 356U));
			assert(s_Regs.CCLKCFG == 2U);
		}
		timing = (rate - 1U) / 20000000U;
		if (timing > 5U)
		{
			timing = 5U;
		}
		if (s_Unsafe != 0 || (s_Regs.FLASHCFG & 0xF000U) != timing << 12)
		{
			printf("rate=%u unsafe accesses=%u final=%x expected=%x\n",
				rate, s_Unsafe, s_Regs.FLASHCFG & 0xF000U, timing << 12);
			errors++;
		}
		assert(s_Checks != 0);
		assert((s_Regs.FLASHCFG & ~0xF000U) == 0xA500003AU);
	}
	// Check oscillator readiness in both range selections, with delayed status.
	for (unsigned input = 12000000; input <= 16000000; input += 4000000)
	{
		for (unsigned delay = 1; delay <= 4; delay += 3)
		{
			memset(&s_Regs, 0, sizeof(s_Regs));
			s_Regs.FLASHCFG = 0x303AU;
			s_Unsafe = 0;
			s_OscWait = 0;
			s_OscReadyAfter = delay;
			s_EarlyClockChange = 0;
			assert(SystemCoreClockSet(120000000, input) == 120000000);
			assert(s_Unsafe == 0);
			assert(s_Regs.SCS == (input == 12000000 ? 0x60U : 0x70U));
			if (s_EarlyClockChange != 0)
			{
				printf("input=%u delay=%u clock changed before oscillator ready\n", input, delay);
				errors++;
			}
			assert(s_OscWait == delay);
		}
	}
	// Main oscillator inputs below the IRC rate are not the RTC source.
	for (unsigned input = 1000000; input <= 3000000; input += 1000000)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.FLASHCFG = 0x303AU;
		s_Unsafe = 0;
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		s_EarlyClockChange = 0;
		assert(SystemCoreClockSet(120000000, input) == 120000000);
		assert(s_Regs.CLKSRCSEL == CLKSRCSEL_MAIN);
		assert(s_Regs.SCS == 0x60U);
		assert(s_OscWait == 4);
		assert(s_EarlyClockChange == 0);
		assert(s_Unsafe == 0);
	}
	// Missing oscillator: return without selecting it or programming PLL0.
	for (unsigned input = 12000000; input <= 16000000; input += 4000000)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.FLASHCFG = 0x303AU;
		SystemCoreClock = 4000000;
		SystemClkFreq = 4000000;
		SystemMicroSecLoopCnt = 7;
		s_OscWait = 0;
		s_OscReadyAfter = 0;
		s_EarlyClockChange = 0;
		assert(SystemCoreClockSet(120000000, input) == 0);
		assert(s_OscWait != 0);
		assert(s_EarlyClockChange == 0);
		assert(s_Regs.CLKSRCSEL == CLKSRCSEL_IRC);
		assert(s_Regs.CCLKCFG == 0);
		assert(s_Regs.PLL0CFG == 0);
		assert(s_Regs.PLL0CON == 0);
		assert(s_Regs.FLASHCFG == 0x503AU);
		assert(SystemCoreClock == 4000000);
		assert(SystemClkFreq == 4000000);
		assert(SystemMicroSecLoopCnt == 7);

		// A later ready indication permits a successful retry.
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		s_Unsafe = 0;
		assert(SystemCoreClockSet(120000000, input) == 120000000);
		assert(s_EarlyClockChange == 0);
		assert(s_Unsafe == 0);
		assert(SystemClkFreq == input);
	}
	// UM10360 table 21: supported RTC multipliers with MSEL bit 12 set.
	memset(&s_Regs, 0, sizeof(s_Regs));
	s_Regs.FLASHCFG = 0x303AU;
	s_Unsafe = 0;
	assert(SystemCoreClockSet(72007680, 32768) == 72007680);
	assert(s_Regs.PLL0CFG == 4394U);
	assert(s_Regs.CCLKCFG == 3U);
	assert(s_Unsafe == 0);

	memset(&s_Regs, 0, sizeof(s_Regs));
	s_Regs.FLASHCFG = 0x303AU;
	s_Unsafe = 0;
	assert(SystemCoreClockSet(84000768, 32768) == 84000768);
	assert(s_Regs.PLL0CFG == 5126U);
	assert(s_Regs.CCLKCFG == 3U);
	assert(s_Unsafe == 0);

	printf("LPC17xx flash timing and oscillator wait: %u errors\n", errors);
	return errors != 0;
}
