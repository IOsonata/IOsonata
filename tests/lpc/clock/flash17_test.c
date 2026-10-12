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
static unsigned s_PllWait;
static int s_FailPllLock;
static int s_FailPllConnect;
static int s_StuckDisconnect;
static unsigned s_ConnectSeen;
static unsigned s_ConnectWait;
static uint32_t s_Primask;
static unsigned s_FeedGuards;
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

uint32_t TestGetPrimask(void)
{
	return s_Primask;
}

void TestDisableIRQ(void)
{
	s_Primask = 1;
}

void TestSetPrimask(uint32_t Mask)
{
	assert(s_Primask == 1);
	assert(s_Regs.PLL0FEED == 0x55 || s_Regs.PLL1FEED == 0x55);
	s_FeedGuards++;
	s_Primask = Mask;
}

LPC_SC_TypeDef *TestSystemRegs(void)
{
	uint32_t status = s_Regs.PLL0CFG | ((s_Regs.PLL0CON & 3U) << 24);

	if (s_Regs.PLL0FEED == 0xAA || s_Regs.PLL1FEED == 0xAA)
	{
		assert(s_Primask == 1);
	}

	if (s_FailPllConnect)
	{
		if ((s_Regs.PLL0CON & 2U) != 0)
		{
			s_ConnectSeen = 1;
			status &= ~(1U << 25);
		}
		else if (s_ConnectSeen && s_StuckDisconnect)
		{
			// Late connection that does not acknowledge cancellation.
			status |= 1U << 25;
		}
		if (s_ConnectSeen)
		{
			s_ConnectWait++;
			assert(s_ConnectWait < 3000000U);
		}
	}

	if (s_FailPllLock && (s_Regs.PLL0CON & 1U) != 0)
	{
		s_PllWait++;
		assert(s_PllWait < 2000000U);
		assert((s_Regs.PLL0CON & 2U) == 0);
	}
	else
	{
		status |= 1U << 26;
	}

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
	// Lock failure leaves the CPU on the selected input through its divider.
	for (unsigned input = 4000000; input <= 12000000; input += 8000000)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.FLASHCFG = 0x303AU;
		s_Regs.USBCLKCFG = 9;
		SystemCoreClock = 120000000;
		SystemMicroSecLoopCnt = 7;
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		s_EarlyClockChange = 0;
		s_PllWait = 0;
		s_FailPllLock = 1;
		s_Unsafe = 0;
		assert(SystemCoreClockSet(120000000, input) == 0);
		assert(s_PllWait != 0);
		assert(s_Regs.PLL0CON == 1);
		assert(s_Regs.CCLKCFG == 3);
		assert(s_Regs.USBCLKCFG == 9);
		assert(s_Regs.FLASHCFG == 0x503AU);
		assert(SystemCoreClock == input / 4);
		assert(SystemClkFreq == input);
		assert(SystemMicroSecLoopCnt == 0);
		assert(s_Unsafe == 0);
		s_FailPllLock = 0;
	}
	// Connection failure must cancel the request without stopping a possible CPU clock.
	for (unsigned stuck = 0; stuck < 2; stuck++)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.FLASHCFG = 0x303AU;
		s_Regs.USBCLKCFG = 9;
		s_FailPllConnect = 1;
		s_StuckDisconnect = stuck;
		s_ConnectSeen = 0;
		s_ConnectWait = 0;
		s_Unsafe = 0;
		assert(SystemCoreClockSet(120000000, 4000000) == 0);
		assert(s_ConnectSeen != 0);
		assert(s_Regs.PLL0CON == 1);
		assert(s_Regs.PLL0FEED == 0x55);
		assert(s_Regs.USBCLKCFG == 9);
		assert(s_Regs.FLASHCFG == 0x503AU);
		assert(SystemCoreClock == (stuck ? 120000000U : 1000000U));
		assert(SystemMicroSecLoopCnt == SystemCoreClock / 16000000U);
		assert(s_Unsafe == 0);
		s_FailPllConnect = 0;
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

	// Both PLL feed paths preserve enabled and already-disabled IRQ state.
	for (uint32_t mask = 0; mask <= 1; mask++)
	{
		uint32_t status = 7U << 8;

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Primask = mask;
		s_FeedGuards = 0;
		assert(SystemCoreClockSet(120000000, 4000000) == 120000000);
		assert(s_Primask == mask);
		assert(s_FeedGuards == 3);

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.CLKSRCSEL = CLKSRCSEL_MAIN;
		SystemClkFreq = 12000000;
		memcpy((uint8_t *)&s_Regs + offsetof(LPC_SC_TypeDef, PLL1STAT), &status, sizeof(status));
		s_FeedGuards = 0;
		SystemConnectPLL1();
		assert(s_Primask == mask);
		assert(s_FeedGuards == 3);
	}
	printf("LPC17xx flash timing and oscillator wait: %u errors\n", errors);
	return errors != 0;
}
