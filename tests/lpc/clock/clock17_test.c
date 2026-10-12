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
#include "coredev/system_core_clock.h"
#include <limits.h>

extern uint32_t SystemClkFreq;
uint32_t SystemMicroSecLoopCnt;
static unsigned s_Errors;
static uint32_t s_Primask;
static const uint32_t s_PclkRequests[] = {0, 1, 12000000, 16000000, 18000000,
	20000000, 24000000, 36000000, 48000000, 72000000, 96000000, UINT32_MAX};
static const unsigned s_PclkFields[][2] = {
	{LPC17_PCLK_WDT, 0},
	{LPC17_PCLK_TIMER0, 1},
	{LPC17_PCLK_TIMER1, 2},
	{LPC17_PCLK_UART0, 3},
	{LPC17_PCLK_UART1, 4},
	{LPC17_PCLK_PWM1, 6},
	{LPC17_PCLK_I2C0, 7},
	{LPC17_PCLK_SPI, 8},
	{LPC17_PCLK_SSP1, 10},
	{LPC17_PCLK_DAC, 11},
	{LPC17_PCLK_ADC, 12},
	{LPC17_PCLK_CAN1, 13},
	{LPC17_PCLK_CAN2, 14},
	{LPC17_PCLK_ACF, 15},
	{LPC17_PCLK_QEI, 16},
	{LPC17_PCLK_GPIOINT, 17},
	{LPC17_PCLK_PCB, 18},
	{LPC17_PCLK_I2C1, 19},
	{LPC17_PCLK_SSP0, 21},
	{LPC17_PCLK_TIMER2, 22},
	{LPC17_PCLK_TIMER3, 23},
	{LPC17_PCLK_UART2, 24},
	{LPC17_PCLK_UART3, 25},
	{LPC17_PCLK_I2C2, 26},
	{LPC17_PCLK_I2S, 27},
	{LPC17_PCLK_RIT, 29},
	{LPC17_PCLK_SYSCON, 30},
	{LPC17_PCLK_MCPWM, 31},
};
static const unsigned s_PclkDividers[] = {4, 1, 2, 8};
static const int s_InvalidPclks[] = {INT_MIN, -1, 5, 9, 20, 28, 33, INT_MAX};


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
	s_Primask = Mask;
}

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
	SystemCoreClock = 96000001;
	for (unsigned i = 0; i < sizeof(s_PclkFields) / sizeof(s_PclkFields[0]); i++)
	{
		unsigned field = s_PclkFields[i][1];
		unsigned shift = 2U * (field % 16U);
		volatile uint32_t *reg = field < 16 ? &LPC_SC->PCLKSEL0 : &LPC_SC->PCLKSEL1;

		for (unsigned sel = 0; sel < 4; sel++)
		{
			LPC_SC_TypeDef before;
			unsigned div = sel == 3 && field >= 13 && field <= 15 ? 6 : s_PclkDividers[sel];

			LPC_SC->PCLKSEL0 = 0xAAAAAAAA;
			LPC_SC->PCLKSEL1 = 0x55555555;
			*reg = (*reg & ~(3U << shift)) | (sel << shift);
			memcpy(&before, (const void *)LPC_SC, sizeof(before));
			assert(SystemPeriphClockGet(s_PclkFields[i][0]) == SystemCoreClock / div);
			assert(memcmp(&before, (const void *)LPC_SC, sizeof(before)) == 0);
		}
	}
	assert(SystemPeriphClockGet(LPC17_PCLK_RTC) == SystemCoreClock / 8U);
	for (unsigned i = 0; i < sizeof(s_InvalidPclks) / sizeof(s_InvalidPclks[0]); i++)
	{
		assert(SystemPeriphClockGet(s_InvalidPclks[i]) == 0);
	}
	SystemCoreClock = 96000000;
	for (unsigned i = 0; i < sizeof(s_PclkFields) / sizeof(s_PclkFields[0]); i++)
	{
		unsigned field = s_PclkFields[i][1];
		int can = field >= 13 && field <= 15;

		for (unsigned j = 0; j < sizeof(s_PclkRequests) / sizeof(s_PclkRequests[0]); j++)
		{
			for (unsigned irq = 0; irq <= 1; irq++)
			{
				LPC_SC_TypeDef before;
				uint32_t mask = can ? 0xFC000000U : 3U << (2U * (field % 16U));
				uint32_t rate;
				uint32_t expected = 0;
				uint64_t error = UINT64_MAX;

				// Enumerate the four actual rates and prefer the lower rate on ties.
				for (unsigned sel = 0; sel < 4; sel++)
				{
					uint32_t div = can && sel == 3 ? 6 : s_PclkDividers[sel];
					uint32_t candidate = SystemCoreClock / div;
					uint64_t diff = candidate > s_PclkRequests[j] ?
						(uint64_t)candidate - s_PclkRequests[j] : (uint64_t)s_PclkRequests[j] - candidate;

					if (diff < error || (diff == error && candidate < expected))
					{
						error = diff;
						expected = candidate;
					}
				}
				LPC_SC->PCLKSEL0 = 0xAAAAAAAA;
				LPC_SC->PCLKSEL1 = 0x55555555;
				memcpy(&before, (const void *)LPC_SC, sizeof(before));
				s_Primask = irq;
				rate = SystemPeriphClockSet(s_PclkFields[i][0], s_PclkRequests[j]);
				assert(rate == expected);
				assert(SystemPeriphClockGet(s_PclkFields[i][0]) == rate);
				assert(s_Primask == irq);
				if (can)
				{
					assert(SystemPeriphClockGet(LPC17_PCLK_CAN1) == rate);
					assert(SystemPeriphClockGet(LPC17_PCLK_CAN2) == rate);
					assert(SystemPeriphClockGet(LPC17_PCLK_ACF) == rate);
				}
				if (field < 16)
				{
					assert((LPC_SC->PCLKSEL0 & ~mask) == (before.PCLKSEL0 & ~mask));
					before.PCLKSEL0 = LPC_SC->PCLKSEL0;
				}
				else
				{
					assert((LPC_SC->PCLKSEL1 & ~mask) == (before.PCLKSEL1 & ~mask));
					before.PCLKSEL1 = LPC_SC->PCLKSEL1;
				}
				assert(memcmp(&before, (const void *)LPC_SC, sizeof(before)) == 0);
			}
		}
	}
	{
		LPC_SC_TypeDef before;

		memcpy(&before, (const void *)LPC_SC, sizeof(before));
		for (unsigned i = 0; i < sizeof(s_InvalidPclks) / sizeof(s_InvalidPclks[0]); i++)
		{
			assert(SystemPeriphClockSet(s_InvalidPclks[i], UINT32_MAX) == 0);
		}
		assert(SystemPeriphClockSet(LPC17_PCLK_RTC, 1) == 12000000);
		assert(SystemPeriphClockSet(LPC17_PCLK_RTC, UINT32_MAX) == 12000000);
		SystemCoreClock = 0;
		assert(SystemPeriphClockSet(LPC17_PCLK_UART0, 1000000) == 0);
		assert(memcmp(&before, (const void *)LPC_SC, sizeof(before)) == 0);
	}
	// Fractional midpoint: comparing truncated Hz would choose the wrong divider.
	SystemCoreClock = 96000006;
	assert(SystemPeriphClockSet(LPC17_PCLK_UART0, 18000001) == 12000000);
	assert(((LPC_SC->PCLKSEL0 >> 6) & 3U) == 3U);
	printf("LPC17xx clock decoding: %u errors\n", s_Errors);
	return s_Errors != 0;
}
