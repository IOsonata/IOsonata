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
#include "coredev/system_core_clock.h"

uint32_t SystemMicroSecLoopCnt;
TestScb_t g_TestScb;
void (* const __Vectors[])(void) = {0};
static LPC_SC_TypeDef s_Regs;
static unsigned s_Unsafe;
static unsigned s_Checks;
static unsigned s_OscWait;
static unsigned s_OscReadyAfter;
static unsigned s_EarlyClockChange;
static unsigned s_PllWait;
static int s_FailPllLock;
static int s_UnreliablePllLock;
static unsigned s_SettleCycles;
static int s_FailPllConnect;
static int s_StuckDisconnect;
static unsigned s_ConnectSeen;
static unsigned s_ConnectWait;
static uint32_t s_Primask;
static unsigned s_FeedGuards;
static int s_UsbModel = 1;
static int s_CheckUsbStop;
static int s_BlockUsbStop;
static uint32_t s_UsbPreviousCfg;
static uint32_t s_UsbPreviousCon;
static unsigned s_UsbStopWait;
static unsigned s_UsbFailure;
static unsigned s_UsbWait;
static unsigned s_CpuStopMode;
static unsigned s_CpuStopWait;
static uint32_t s_CpuPreviousStatus;
static int s_CpuStopDone;
static const uint32_t s_Rates[] = {
	20000000, 22000000, 40000000, 42000000, 60000000, 62000000,
	80000000, 82000000, 99000000, 100000000, 101000000, 102000000,
	119000000, 120000000
};
static const uint32_t s_BypassCases[][2] = {
	{4000000, 4000000}, {4000000, 2000000},
	{12000000, 12000000}, {12000000, 6000000},
	{12000000, 3000000}, {24000000, 24000000}
};
static const uint32_t s_ExternalInputs[] = {
	1000000, 4000000, 12000000, 15000000, 15000001, 25000000
};
static const int s_InvalidInputs[] = {
	0, -1, INT_MIN, 1, 32767, 32769, 999999, 25000001, INT_MAX
};
static const uint32_t s_UsbInputs[] = {12000000, 16000000, 24000000};
static const uint32_t s_UsbInvalidInputs[] = {0, 32768, 4000000, 10000000, 20000000, 25000000};
// Input Hz, requested CPU Hz, expected USB divider (7 is the retained sentinel).
static const uint32_t s_UsbDividerCases[][3] = {
	{12000000, 72000000, 5},
	{12000000, 76800000, 7},
	{12000000, 120000000, 9},
	{12000000, 100000000, 7},
	{12000000, 84000000, 7},
	{12000000, 1000000, 7},
	{12000000, 12345678, 7},
	{4000000, 120000000, 7},
	{32768, 72007680, 7}
};
static const uint32_t s_SettleCases[][3] = {
	{32768, 72007680, 50},
	{32768, 71999488, 100},
	{25000000, 100000000, 4167}
};
// Independent exhaustive oracle uses the hardware's supported RTC values.
static const uint16_t s_RtcM[] = {
	4272, 4395, 4578, 4725, 4807, 5127, 5188, 5400, 5493, 5859,
	6042, 6075, 6104, 6409, 6592, 6750, 6836, 6866, 6958, 7050,
	7324, 7425, 7690, 7813, 7935, 8057, 8100, 8545, 8789, 9155,
	9613, 10254, 10376, 10986, 11719, 12085, 12207, 12817, 13184, 13672,
	13733, 13916, 14099, 14420, 14648, 15381, 15564, 15625, 15869, 16113,
	16479, 17578, 18127, 18311, 19226, 19775, 20508, 20599, 20874, 21149,
	21973, 23071, 23438, 23804, 24170
};
#ifdef TEST_CLOCK_LIMIT
static const uint32_t s_LimitRequests[] = {99000000, 100000000, 101000000, 120000000, UINT32_MAX};
#endif
static const uint32_t s_ClosestCases[][2] = {
	{4000000, 0}, {4000000, UINT32_MAX}, {4000000, 12345678},
	{4000000, 1000000}, {12000000, 12345678}, {25000000, UINT32_MAX},
	{32768, 72000000}, {32768, 1}, {32768, 123456789}
};
extern uint32_t SystemClkFreq;

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

void TestClockWaitCycle(void)
{
	assert(s_Regs.PLL0CON == 1);
	assert(s_Regs.PLL0FEED == 0x55);
	s_SettleCycles++;
}

LPC_SC_TypeDef *TestSystemRegs(void)
{
	uint32_t status = s_Regs.PLL0CFG | ((s_Regs.PLL0CON & 3U) << 24);

	if (s_CpuStopMode && !s_CpuStopDone)
	{
		if (s_Regs.PLL0CFG != 19U || s_Regs.CCLKCFG != 7U ||
			s_Regs.CLKSRCSEL != CLKSRCSEL_MAIN)
		{
			assert((s_CpuPreviousStatus & (3U << 24)) == 0);
			assert(s_CpuStopMode == 1);
			s_CpuStopDone = 1;
		}
		else if (s_CpuStopMode == 2)
		{
			assert(s_Regs.PLL0CON != 0);
			status |= 3U << 24;
		}
		else if (s_CpuStopMode == 3)
		{
			status |= 1U << 24;
		}
		s_CpuPreviousStatus = status;
		s_CpuStopWait++;
		assert(s_CpuStopWait < 2000000U);
	}

	if (s_UsbModel)
	{
		uint32_t usbstatus = s_Regs.PLL1CFG | ((s_Regs.PLL1CON & 3U) << 8);

		if ((s_Regs.PLL1CON & 1U) != 0)
		{
			s_UsbWait++;
			assert(s_UsbWait < 2000000U);
			if (s_UsbFailure == 1)
			{
				assert((s_Regs.PLL1CON & 2U) == 0);
			}
			else
			{
				usbstatus |= 1U << 10;
			}
			if (s_UsbFailure == 2)
			{
				usbstatus &= ~(1U << 9);
			}
		}
		if (s_CheckUsbStop)
		{
			if (s_Regs.PLL1CFG != s_UsbPreviousCfg)
			{
				assert(s_UsbPreviousCon == 0);
				assert(!s_BlockUsbStop);
			}
			s_UsbPreviousCfg = s_Regs.PLL1CFG;
			s_UsbPreviousCon = s_Regs.PLL1CON;
			if (s_BlockUsbStop && s_Regs.PLL1CON == 0)
			{
				s_UsbStopWait++;
				assert(s_UsbStopWait < 2000000U);
				usbstatus |= 3U << 8;
			}
		}
		memcpy((uint8_t *)&s_Regs + offsetof(LPC_SC_TypeDef, PLL1STAT), &usbstatus, sizeof(usbstatus));
	}

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

	if (s_UnreliablePllLock)
	{
		status &= ~(1U << 26);
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

static uint64_t ClockError(uint64_t Numerator, uint32_t Denominator, uint32_t Request)
{
	uint64_t target = (uint64_t)Request * Denominator;

	return Numerator > target ? Numerator - target : target - Numerator;
}

static void CheckClosest(uint32_t Input, uint32_t Request)
{
	uint32_t div = (s_Regs.CCLKCFG & 255U) + 1U;
	uint32_t m = (s_Regs.PLL0CFG & 32767U) + 1U;
	uint32_t n = ((s_Regs.PLL0CFG >> 16) & 255U) + 1U;
	uint64_t numerator = Input;
	uint32_t denominator = div;
	uint64_t error;

	if (s_Regs.PLL0CON == 3)
	{
		int supported = m >= 6 && m <= 512;

		if (Input == 32768)
		{
			supported = 0;
			for (unsigned i = 0; i < sizeof(s_RtcM) / sizeof(s_RtcM[0]); i++)
			{
				supported |= m == s_RtcM[i];
			}
		}
		assert(supported && n <= 32);
		numerator = (uint64_t)Input * 2U * m;
		denominator *= n;
		assert(numerator >= 275000000ULL * n);
		assert(numerator <= 550000000ULL * n);
	}
	else
	{
		assert(s_Regs.PLL0CON == 0);
	}
	assert(numerator <= (uint64_t)LPC17_MAX_CORE_FREQ * denominator);
	assert(SystemCoreClock == numerator / denominator);
	error = ClockError(numerator, denominator, Request);
	// Enumerate every divider, unlike the production bracketing search.
	for (uint32_t d = 1; d <= 256; d++)
	{
		assert(ClockError(Input, d, Request) * denominator >= error * d);
	}
	for (uint32_t pre = 1; pre <= 32; pre++)
	{
		unsigned count = Input == 32768 ? sizeof(s_RtcM) / sizeof(s_RtcM[0]) : 507;

		for (unsigned i = 0; i < count; i++)
		{
			uint32_t mul = Input == 32768 ? s_RtcM[i] : i + 6U;
			uint64_t num = (uint64_t)Input * 2U * mul;

			if (num < 275000000ULL * pre || num > 550000000ULL * pre)
			{
				continue;
			}
			for (uint32_t d = 3; d <= 256; d++)
			{
				uint32_t den = pre * d;

				if (num <= (uint64_t)LPC17_MAX_CORE_FREQ * den)
				{
					assert(ClockError(num, den, Request) * denominator >= error * den);
				}
			}
		}
	}
}

void TestDSB(void)
{
}

void TestISB(void)
{
}

int main(void)
{
	unsigned errors = 0;

	// Exact input/divider rates must not require PLL lock, even on USB CCO ties.
	for (unsigned i = 0; i < sizeof(s_BypassCases) / sizeof(s_BypassCases[0]); i++)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		s_FailPllLock = 1;
		s_PllWait = 0;
		s_Regs.USBCLKCFG = 7;
		assert(SystemCoreClockSet(s_BypassCases[i][1], s_BypassCases[i][0]) == s_BypassCases[i][1]);
		assert(s_Regs.PLL0CON == 0);
		assert(s_Regs.CCLKCFG + 1U == s_BypassCases[i][0] / s_BypassCases[i][1]);
		assert(s_Regs.USBCLKCFG == 7);
		assert(s_PllWait == 0);
		CheckClosest(s_BypassCases[i][0], s_BypassCases[i][1]);
	}
	s_FailPllLock = 0;

	// Startup recovery must work even when the PLL cannot lock or connect.
	for (unsigned failure = 0; failure < 4; failure++)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_OscWait = 0;
		s_OscReadyAfter = failure == 0 ? 0 : 4;
		s_FailPllLock = failure == 1;
		s_FailPllConnect = failure == 2;
		s_PllWait = 0;
		s_ConnectSeen = 0;
		s_ConnectWait = 0;
		g_McuOsc.CoreOsc.Type = OSC_TYPE_XTAL;
		g_McuOsc.CoreOsc.Freq = failure == 3 ? 32768 : 12000000;
		g_McuOsc.CoreOsc.Accuracy = 20;
		g_McuOsc.CoreOsc.LoadCap = 120;
		SystemInit();
		assert(s_Regs.CLKSRCSEL == CLKSRCSEL_IRC);
		assert(s_Regs.PLL0CON == 0);
		assert(s_Regs.CCLKCFG == 0);
		assert(SystemCoreClock == 4000000);
		assert(SystemClkFreq == 4000000);
		assert(g_McuOsc.CoreOsc.Type == OSC_TYPE_RC);
		assert(g_McuOsc.CoreOsc.Freq == 4000000);
		assert(GetHighFreqOscAcc() == 10000);
		assert(GetHighFreqLoadCap() == 0);
		assert((s_Regs.FLASHCFG & 0xF000U) == 0);
	}
	s_FailPllLock = 0;
	s_FailPllConnect = 0;
	s_ConnectSeen = 0;
	s_OscWait = 0;
	memset(&s_Regs, 0, sizeof(s_Regs));

	// Startup must consume the descriptor rather than hard-code IRC.
	g_McuOsc.CoreOsc.Type = OSC_TYPE_XTAL;
	g_McuOsc.CoreOsc.Freq = 12000000;
	g_McuOsc.CoreOsc.Accuracy = 20;
	g_McuOsc.CoreOsc.LoadCap = 120;
	s_OscReadyAfter = 4;
	SystemInit();
	assert(s_Regs.CLKSRCSEL == CLKSRCSEL_MAIN);
	assert(SystemClkFreq == 12000000);
	assert(GetHighFreqOscAcc() == 20);
	assert(GetHighFreqLoadCap() == 120);
	CheckClosest(12000000, CORE_FREQ);

	// Externally driven XTAL1 uses MAIN, including at the IRC's nominal rate.
	for (unsigned i = 0; i < sizeof(s_ExternalInputs) / sizeof(s_ExternalInputs[0]); i++)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		assert(SystemCoreClockSelect(OSC_TYPE_TCXO, s_ExternalInputs[i]));
		assert(s_Regs.CLKSRCSEL == CLKSRCSEL_MAIN);
		assert((s_Regs.SCS & 0x70U) == (s_ExternalInputs[i] > 15000000U ? 0x70U : 0x60U));
		assert(g_McuOsc.CoreOsc.Type == OSC_TYPE_TCXO);
		assert(g_McuOsc.CoreOsc.Freq == s_ExternalInputs[i]);
		CheckClosest(s_ExternalInputs[i], CORE_FREQ);
	}
	memset(&s_Regs, 0, sizeof(s_Regs));
	s_OscWait = 0;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_TCXO;
	g_McuOsc.CoreOsc.Freq = 12000000;
	SystemInit();
	assert(s_Regs.CLKSRCSEL == CLKSRCSEL_MAIN);
	assert(SystemClkFreq == 12000000);
	assert(g_McuOsc.CoreOsc.Type == OSC_TYPE_TCXO);
	CheckClosest(12000000, CORE_FREQ);

	// Explicit source selection must distinguish a 4 MHz crystal from IRC.
	for (unsigned crystal = 0; crystal <= 1; crystal++)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		assert(SystemCoreClockSelect(crystal ? OSC_TYPE_XTAL : OSC_TYPE_RC, 4000000));
		assert(s_Regs.CLKSRCSEL == (crystal ? CLKSRCSEL_MAIN : CLKSRCSEL_IRC));
		assert(g_McuOsc.CoreOsc.Type == (crystal ? OSC_TYPE_XTAL : OSC_TYPE_RC));
		assert(g_McuOsc.CoreOsc.Freq == 4000000);
		CheckClosest(4000000, CORE_FREQ);
	}
	{
		LPC_SC_TypeDef before;
		McuOsc_t desc = g_McuOsc;

		memcpy(&before, &s_Regs, sizeof(before));
		assert(!SystemCoreClockSelect(OSC_TYPE_RC, 12000000));
		assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 32768));
		assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, UINT32_MAX));
		assert(!SystemCoreClockSelect(OSC_TYPE_TCXO, 999999));
		assert(!SystemCoreClockSelect(OSC_TYPE_TCXO, 25000001));
		assert(!SystemCoreClockSelect(OSC_TYPE_TCXO, UINT32_MAX));
		assert(!SystemCoreClockSelect((OSC_TYPE)99, 4000000));
		assert(memcmp(&before, &s_Regs, sizeof(before)) == 0);
		assert(memcmp(&desc, &g_McuOsc, sizeof(desc)) == 0);
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_OscReadyAfter = 0;
		assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000));
		assert(memcmp(&desc, &g_McuOsc, sizeof(desc)) == 0);
		s_OscWait = 0;
		assert(!SystemCoreClockSelect(OSC_TYPE_TCXO, 12000000));
		assert(memcmp(&desc, &g_McuOsc, sizeof(desc)) == 0);
		assert(s_Regs.CLKSRCSEL == CLKSRCSEL_IRC);
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		assert(SystemCoreClockSelect(OSC_TYPE_TCXO, 12000000));
		assert(g_McuOsc.CoreOsc.Type == OSC_TYPE_TCXO);
	}


	for (unsigned i = 0; i < sizeof(s_ClosestCases) / sizeof(s_ClosestCases[0]); i++)
	{
		uint32_t input = s_ClosestCases[i][0];
		uint32_t request = s_ClosestCases[i][1];

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		assert(SystemCoreClockSet(request, input) != 0);
		CheckClosest(input, request);
	}


#ifdef TEST_CLOCK_LIMIT
	assert(LPC17_MAX_CORE_FREQ == TEST_CLOCK_LIMIT);
	for (unsigned i = 0; i < sizeof(s_LimitRequests) / sizeof(s_LimitRequests[0]); i++)
	{
		uint32_t expected = s_LimitRequests[i] > TEST_CLOCK_LIMIT ? TEST_CLOCK_LIMIT : s_LimitRequests[i];

		memset(&s_Regs, 0, sizeof(s_Regs));
		assert(SystemCoreClockSet(s_LimitRequests[i], 4000000) == expected);
	}
	puts("LPC17xx device speed limit and closest selection: PASS");
	return 0;
#endif

	for (unsigned i = 0; i < sizeof(s_UsbDividerCases) / sizeof(s_UsbDividerCases[0]); i++)
	{
		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.USBCLKCFG = i == 1 ? 5U : 7U;
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		assert(SystemCoreClockSet(s_UsbDividerCases[i][1], s_UsbDividerCases[i][0]) != 0);
		assert(s_Regs.USBCLKCFG == s_UsbDividerCases[i][2]);
		if (i < 3)
		{
			uint32_t m = (s_Regs.PLL0CFG & 32767U) + 1U;
			uint32_t n = ((s_Regs.PLL0CFG >> 16) & 255U) + 1U;

			assert((uint64_t)s_UsbDividerCases[i][0] * 2U * m ==
				48000000ULL * n * (s_Regs.USBCLKCFG + 1U));
			assert((s_Regs.USBCLKCFG & 1U) == 1U);
		}
	}

	// Unreliable references must wait despite premature lock or absent lock status.
	for (unsigned i = 0; i < sizeof(s_SettleCases) / sizeof(s_SettleCases[0]); i++)
	{
		for (unsigned missing = 0; missing <= 1; missing++)
		{
			for (uint32_t mask = 0; mask <= 1; mask++)
			{
				memset(&s_Regs, 0, sizeof(s_Regs));
				s_OscWait = 0;
				s_OscReadyAfter = 4;
				s_SettleCycles = 0;
				s_UnreliablePllLock = missing;
				s_Primask = mask;
				assert(SystemCoreClockSet(s_SettleCases[i][1], s_SettleCases[i][0]) == s_SettleCases[i][1]);
				assert(s_SettleCycles == s_SettleCases[i][2]);
				assert(s_Primask == mask);
				s_UnreliablePllLock = 0;
			}
		}
	}

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
			assert(s_Regs.PLL0CFG == ((1U << 16) | 98U));
			assert(s_Regs.CCLKCFG == 3U);
		}
		if (rate == 101000000)
		{
			assert(s_Regs.PLL0CFG == ((1U << 16) | 100U));
			assert(s_Regs.CCLKCFG == 3U);
		}
		if (rate == 119000000)
		{
			assert(s_Regs.PLL0CFG == ((1U << 16) | 118U));
			assert(s_Regs.CCLKCFG == 3U);
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
		assert(s_FeedGuards == 4);

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.CLKSRCSEL = CLKSRCSEL_MAIN;
		s_Regs.SCS = 0x60;
		SystemClkFreq = 12000000;
		memcpy((uint8_t *)&s_Regs + offsetof(LPC_SC_TypeDef, PLL1STAT), &status, sizeof(status));
		s_FeedGuards = 0;
		SystemConnectPLL1();
		assert(s_Primask == mask);
		assert(s_FeedGuards == 4);
	}
	// PLL1 needs exactly 48 MHz output and P=2 for its 192 MHz CCO.
	for (unsigned i = 0; i < sizeof(s_UsbInputs) / sizeof(s_UsbInputs[0]); i++)
	{
		uint32_t status = 7U << 8;

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.CLKSRCSEL = CLKSRCSEL_MAIN;
		s_Regs.SCS = 0x60;
		SystemClkFreq = s_UsbInputs[i];
		memcpy((uint8_t *)&s_Regs + offsetof(LPC_SC_TypeDef, PLL1STAT), &status, sizeof(status));
		SystemConnectPLL1();
		assert(s_Regs.PLL1CFG == ((48000000U / s_UsbInputs[i] - 1U) | 0x20U));
		assert(s_Regs.PLL1CON == 3);
	}

	for (unsigned i = 0; i < sizeof(s_UsbInvalidInputs) / sizeof(s_UsbInvalidInputs[0]); i++)
	{
		LPC_SC_TypeDef before;
		uint32_t status = 7U << 8;

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.CLKSRCSEL = CLKSRCSEL_MAIN;
		s_Regs.SCS = 0x60;
		SystemClkFreq = s_UsbInvalidInputs[i];
		memcpy((uint8_t *)&s_Regs + offsetof(LPC_SC_TypeDef, PLL1STAT), &status, sizeof(status));
		TestSystemRegs();
		memcpy(&before, &s_Regs, sizeof(before));
		SystemConnectPLL1();
		assert(memcmp(&before, &s_Regs, sizeof(before)) == 0);
	}

	// A valid frequency alone is insufficient without a known, ready MAIN source.
	for (unsigned source = 0; source < 3; source++)
	{
		LPC_SC_TypeDef before;

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_Regs.CLKSRCSEL = source;
		s_Regs.SCS = source == CLKSRCSEL_MAIN ? 0 : 0x60;
		SystemClkFreq = 12000000;
		TestSystemRegs();
		memcpy(&before, &s_Regs, sizeof(before));
		SystemConnectPLL1();
		assert(memcmp(&before, &s_Regs, sizeof(before)) == 0);
	}
	// PLL1 failures must return with the USB PLL disabled, allowing a retry.
	for (unsigned i = 0; i < sizeof(s_UsbInputs) / sizeof(s_UsbInputs[0]); i++)
	{
		for (unsigned failure = 1; failure <= 2; failure++)
		{
			memset(&s_Regs, 0, sizeof(s_Regs));
			s_Regs.CLKSRCSEL = CLKSRCSEL_MAIN;
			s_Regs.SCS = 0x60;
			s_Regs.USBCLKCFG = 9;
			SystemClkFreq = s_UsbInputs[i];
			SystemCoreClock = 72000000;
			s_Primask = failure - 1;
			s_UsbModel = 1;
			s_UsbFailure = failure;
			s_UsbWait = 0;
			SystemConnectPLL1();
			assert(s_UsbWait != 0);
			assert(s_Regs.PLL1CON == 0);
			assert(s_Regs.PLL1FEED == 0x55);
			assert(s_Regs.USBCLKCFG == 9);
			assert(SystemCoreClock == 72000000);
			assert(s_Primask == failure - 1);

			s_UsbFailure = 0;
			s_UsbWait = 0;
			SystemConnectPLL1();
			assert(s_Regs.PLL1CON == 3);
			assert((s_Regs.PLL1STAT & (7U << 8)) == (7U << 8));
			assert(s_Primask == failure - 1);
			s_UsbModel = 0;
		}
	}
	// Reconfigure an already enabled/connected USB PLL only after shutdown.
	for (unsigned blocked = 0; blocked < 2; blocked++)
	{
		for (uint32_t mask = 0; mask <= 1; mask++)
		{
			memset(&s_Regs, 0, sizeof(s_Regs));
			s_Regs.CLKSRCSEL = CLKSRCSEL_MAIN;
			s_Regs.SCS = 0x60;
			s_Regs.PLL1CON = 3;
			s_Regs.PLL1CFG = 0x22;
			SystemClkFreq = 12000000;
			s_Primask = mask;
			s_UsbModel = 1;
			s_UsbFailure = 0;
			s_UsbWait = 0;
			s_CheckUsbStop = 1;
			s_BlockUsbStop = blocked;
			s_UsbPreviousCon = s_Regs.PLL1CON;
			s_UsbPreviousCfg = s_Regs.PLL1CFG;
			s_UsbStopWait = 0;
			SystemConnectPLL1();
			assert(s_Primask == mask);
			assert(s_Regs.PLL1CFG == (blocked ? 0x22U : 0x23U));
			assert(s_Regs.PLL1CON == (blocked ? 0U : 3U));
			if (blocked)
			{
				assert(s_UsbStopWait != 0);
			}
			s_CheckUsbStop = 0;
		}
	}
	// An active CPU PLL must disconnect before disable or configuration changes.
	for (unsigned mode = 1; mode <= 3; mode++)
	{
		for (uint32_t mask = 0; mask <= 1; mask++)
		{
			memset(&s_Regs, 0, sizeof(s_Regs));
			s_Regs.FLASHCFG = 0x503A;
			s_Regs.CLKSRCSEL = CLKSRCSEL_MAIN;
			s_Regs.CCLKCFG = 7;
			s_Regs.PLL0CFG = 19;
			s_Regs.PLL0CON = 3;
			s_Regs.USBCLKCFG = 9;
			SystemClkFreq = 16000000;
			SystemCoreClock = 80000000;
			s_Primask = mask;
			s_CpuStopMode = mode;
			s_CpuStopWait = 0;
			s_CpuStopDone = 0;
			s_CpuPreviousStatus = 3U << 24;
			assert(SystemCoreClockSet(120000000, 4000000) ==
				(mode == 1 ? 120000000U : 0U));
			assert(s_Primask == mask);
			if (mode == 1)
			{
				assert(s_CpuStopDone);
				assert(SystemClkFreq == 4000000);
				assert(s_Regs.PLL0CON == 3);
			}
			else
			{
				assert(!s_CpuStopDone);
				assert(s_Regs.PLL0CON == (mode == 2 ? 1U : 0U));
				assert(s_Regs.USBCLKCFG == 9);
				assert(SystemClkFreq == 16000000);
				assert(SystemCoreClock == (mode == 2 ? 80000000U : 2000000U));
				assert(SystemMicroSecLoopCnt == SystemCoreClock / 16000000U);
				assert(s_Regs.FLASHCFG == 0x503A);
			}
			s_CpuStopMode = 0;
		}
	}
	// The legacy API takes selectors, not oscillator frequencies.
	for (int selector = CLKSRCSEL_IRC; selector <= CLKSRCSEL_RTC; selector++)
	{
		uint32_t target = selector == CLKSRCSEL_RTC ? 72007680U : 120000000U;

		memset(&s_Regs, 0, sizeof(s_Regs));
		s_OscWait = 0;
		s_OscReadyAfter = 4;
		assert(SystemSetCoreClock(target, selector) == target);
		assert(s_Regs.CLKSRCSEL == (uint32_t)selector);
		assert(SystemClkFreq == (selector == CLKSRCSEL_IRC ? 4000000U :
			(selector == CLKSRCSEL_MAIN ? 16000000U : 32768U)));
	}
	{
		LPC_SC_TypeDef before;
		uint32_t core = SystemCoreClock;
		uint32_t source = SystemClkFreq;

		memcpy(&before, &s_Regs, sizeof(before));
		assert(SystemSetCoreClock(120000000, -1) == 0);
		assert(SystemSetCoreClock(120000000, 3) == 0);
		assert(SystemSetCoreClock(120000000, 16000000) == 0);
		assert(memcmp(&before, &s_Regs, sizeof(before)) == 0);
		assert(SystemCoreClock == core);
		assert(SystemClkFreq == source);
	}
	printf("LPC17xx flash timing and oscillator wait: %u errors\n", errors);
	return errors != 0;
}
