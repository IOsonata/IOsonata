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
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "LPC11Uxx.h"
#include "coredev/system_core_clock.h"

typedef uint32_t (*ClockSetter_t)(bool, int);

// Strong board definition must override the driver's weak default in C and C++.
McuOsc_t g_McuOsc = {
	{OSC_TYPE_XTAL, 24000000, 20, 0},
	{OSC_TYPE_RC, 0, 0, 0},
	false,
};

static ClockSetter_t const s_Setters[] = {
	SystemSetCoreClock,
	SystemCoreClockSet,
};

static const int s_InvalidInputs[] = {
	0, -1, INT_MIN, INT_MAX, 9999999, 25000001, 10000000, 13000000, 25000000,
};

static const int s_CrystalInputs[] = {
	12000000, 16000000, 19200000, 24000000,
};

extern uint32_t SystemMainClkFreq;

static void CheckRejected(ClockSetter_t Setter, void *Regs)
{
	uint8_t before[sizeof(LPC_SYSCON_Type)];
	uint32_t core = SystemCoreClock;
	uint32_t mainclk = SystemMainClkFreq;

	memcpy(&before, Regs, sizeof(before));
	for (unsigned i = 0; i < sizeof(s_InvalidInputs) / sizeof(s_InvalidInputs[0]); i++)
	{
		assert(Setter(true, s_InvalidInputs[i]) == 0);
		assert(memcmp(&before, Regs, sizeof(before)) == 0);
		assert(SystemCoreClock == core);
		assert(SystemMainClkFreq == mainclk);
	}
}

int main(void)
{
	void *regs = mmap((void *)0x40048000, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	uint32_t locked = 1;

	assert(regs == (void *)0x40048000);
	assert(mmap((void *)0x4003c000, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x4003c000);
	LPC_FLASHCTRL->FLASHCFG = 0xa5a50000U;
	memcpy((char *)regs + offsetof(LPC_SYSCON_Type, SYSPLLSTAT), &locked, sizeof(locked));
	SystemInit();
	assert(GetHighFreqOscFreq() == 24000000);
	assert(SystemCoreClock == 48000000);
	assert(((LPC_SYSCON->SYSPLLCTRL & 31U) + 1U) * 24000000U /
		LPC_SYSCON->SYSAHBCLKDIV == 48000000);
	assert(LPC_SYSCON->USBCLKDIV == 0);
	assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPLL_PD);
	assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPAD_PD);

	// Exercise each board oscillator type with USB requested and disabled.
	for (unsigned type = OSC_TYPE_RC; type <= OSC_TYPE_TCXO; type++)
	{
		for (unsigned usb = 0; usb < 2; usb++)
		{
			g_McuOsc.CoreOsc.Type = (OSC_TYPE)type;
			g_McuOsc.CoreOsc.Freq = type == OSC_TYPE_RC ? 12000000U : 24000000U;
			g_McuOsc.bUSBClk = usb != 0;
			memcpy((char *)regs + offsetof(LPC_SYSCON_Type, USBPLLSTAT), &locked, sizeof(locked));
			SystemInit();
			assert(SystemCoreClock == 48000000);
			assert(GetHighFreqOscType() == (OSC_TYPE)type);
			assert(LPC_SYSCON->SYSPLLCLKSEL == (type == OSC_TYPE_RC ? 0U : 1U));
			if (type != OSC_TYPE_RC)
			{
				assert((LPC_SYSCON->SYSOSCCTRL & 3U) == (type == OSC_TYPE_TCXO ? 3U : 2U));
			}
			bool usbEnabled = type != OSC_TYPE_RC && usb != 0;

			assert((LPC_SYSCON->USBCLKDIV != 0) == usbEnabled);
			assert(((LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPLL_PD) == 0) == usbEnabled);
			assert(((LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPAD_PD) == 0) == usbEnabled);
		}
	}
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
			assert(LPC_FLASHCTRL->FLASHCFG == 0xa5a50002U);
			if (crystal == 0)
			{
				assert(LPC_SYSCON->USBCLKDIV == 0);
				assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPAD_PD);
				assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPLL_PD);
			}
			SystemCoreClockUpdate();
			assert(SystemCoreClock == 48000000);
		}
		for (unsigned input = 0; input < sizeof(s_CrystalInputs) / sizeof(s_CrystalInputs[0]); input++)
		{
			uint32_t freq = (uint32_t)s_CrystalInputs[input];

			assert(s_Setters[i](true, (int)freq) == 48000000);
			uint32_t ctrl = LPC_SYSCON->SYSPLLCTRL;
			uint32_t output = freq * ((ctrl & 31U) + 1U);
			uint64_t cco = (uint64_t)output * (2U << ((ctrl >> 5) & 3U));

			assert((ctrl & ~127U) == 0);
			assert(cco >= 156000000 && cco <= 320000000);
			assert(output / LPC_SYSCON->SYSAHBCLKDIV == 48000000);
			assert(LPC_SYSCON->USBPLLCTRL == ctrl);
			assert(output / LPC_SYSCON->USBCLKDIV == 48000000);
			SystemCoreClockUpdate();
			assert(SystemCoreClock == 48000000);
			assert((LPC_SYSCON->SYSOSCCTRL & 3U) == (freq > 20000000U ? 2U : 0U));
			CheckRejected(s_Setters[i], regs);
			SystemCoreClockUpdate();
			assert(SystemCoreClock == 48000000);
		}
		// Reconfigure from an active PLL, then inject each PLL lock failure.
		for (unsigned usbFailure = 0; usbFailure < 2; usbFailure++)
		{
			uint32_t unlocked = 0;

			memcpy((char *)regs + offsetof(LPC_SYSCON_Type, SYSPLLSTAT), &locked, sizeof(locked));
			memcpy((char *)regs + offsetof(LPC_SYSCON_Type, USBPLLSTAT), &locked, sizeof(locked));
			assert(s_Setters[i](true, 12000000) == 48000000);
			size_t offset = usbFailure ? offsetof(LPC_SYSCON_Type, USBPLLSTAT) :
				offsetof(LPC_SYSCON_Type, SYSPLLSTAT);

			memcpy((char *)regs + offset, &unlocked, sizeof(unlocked));
			assert(s_Setters[i](true, 12000000) == 0);
			assert(LPC_SYSCON->MAINCLKSEL == MAINCLKSEL_IRC);
			assert(LPC_SYSCON->SYSAHBCLKDIV == 1);
			assert(SystemMainClkFreq == 12000000 && SystemCoreClock == 12000000);
			assert((LPC_SYSCON->PDRUNCFG & (PDRUNCFG_SYSPLL_PD | PDRUNCFG_USBPLL_PD)) ==
				(PDRUNCFG_SYSPLL_PD | PDRUNCFG_USBPLL_PD));
			assert(LPC_SYSCON->USBCLKDIV == 0);
			assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPAD_PD);
			memcpy((char *)regs + offset, &locked, sizeof(locked));
			assert(s_Setters[i](true, 12000000) == 48000000);
		}
	}
	// Generic entry point must link in C/C++, report success and update metadata.
	g_McuOsc.bUSBClk = false;
	assert(SystemCoreClockSelect(OSC_TYPE_TCXO, 24000000));
	assert(GetHighFreqOscType() == OSC_TYPE_TCXO);
	assert(GetHighFreqOscFreq() == 24000000);
	assert((LPC_SYSCON->SYSOSCCTRL & 3U) == 3U);
	assert(SystemCoreClock == 48000000 && LPC_SYSCON->USBCLKDIV == 0);
	uint8_t before[sizeof(LPC_SYSCON_Type)];

	memcpy(before, regs, sizeof(before));
	assert(!SystemCoreClockSelect(OSC_TYPE_RC, 4000000));
	assert(!SystemCoreClockSelect((OSC_TYPE)99, 12000000));
	assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 13000000));
	assert(memcmp(before, regs, sizeof(before)) == 0);
	assert(GetHighFreqOscType() == OSC_TYPE_TCXO);
	assert(GetHighFreqOscFreq() == 24000000);
	assert(SystemCoreClock == 48000000);
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 12000000));
	assert(GetHighFreqOscType() == OSC_TYPE_RC);
	assert(GetHighFreqOscFreq() == 12000000);
	assert(LPC_SYSCON->SYSPLLCLKSEL == SYSPLLCLKSEL_IRC);

	// A lock failure must not publish an oscillator selection that failed.
	g_McuOsc.bUSBClk = true;
	uint32_t unlocked = 0;

	memcpy((char *)regs + offsetof(LPC_SYSCON_Type, USBPLLSTAT), &unlocked, sizeof(unlocked));
	assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 16000000));
	assert(GetHighFreqOscType() == OSC_TYPE_RC);
	assert(GetHighFreqOscFreq() == 12000000);
	assert(SystemCoreClock == 12000000);
	memcpy((char *)regs + offsetof(LPC_SYSCON_Type, USBPLLSTAT), &locked, sizeof(locked));
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 16000000));
	assert(GetHighFreqOscType() == OSC_TYPE_XTAL);
	assert(GetHighFreqOscFreq() == 16000000);
	assert(SystemCoreClock == 48000000 && LPC_SYSCON->USBCLKDIV != 0);
	puts("LPC11Uxx clock setters, PLL selection and lock recovery: PASS");
}
