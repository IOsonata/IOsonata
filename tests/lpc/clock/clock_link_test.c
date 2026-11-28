/**-------------------------------------------------------------------------
@file	clock_link_test.c

@brief	LPC11Uxx clock setup in SystemInit, linked from C and C++.

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

uint32_t SystemMicroSecLoopCnt;

// Strong board definition must override the driver's weak default in C and C++.
McuOsc_t g_McuOsc = {
	{OSC_TYPE_XTAL, 24000000, 20, 0},
	{OSC_TYPE_RC, 0, 0, 0},
	false,
};

// Inputs with no 48 MHz PLL setting, or outside the 10 to 25 MHz range
static const uint32_t s_InvalidInputs[] = {
	0, 9999999, 25000001, 10000000, 13000000, 25000000, UINT32_MAX,
};

static const uint32_t s_CrystalInputs[] = {
	12000000, 16000000, 19200000, 24000000,
};

extern uint32_t SystemMainClkFreq;

static void *s_Regs;

// Register model after a reset, with the PLL lock indications given
static void ResetModel(bool SysLock, bool UsbLock)
{
	uint32_t locked;

	memset(s_Regs, 0, 0x1000);
	LPC_SYSCON->PDRUNCFG = PDRUNCFG_SYSOSC_PD | PDRUNCFG_WDTOSC_PD | PDRUNCFG_SYSPLL_PD |
		PDRUNCFG_USBPLL_PD | PDRUNCFG_USBPAD_PD;
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	LPC_FLASHCTRL->FLASHCFG = 0xa5a50000U;
	locked = SysLock ? 1U : 0U;
	memcpy((char *)s_Regs + offsetof(LPC_SYSCON_Type, SYSPLLSTAT), &locked, sizeof(locked));
	locked = UsbLock ? 1U : 0U;
	memcpy((char *)s_Regs + offsetof(LPC_SYSCON_Type, USBPLLSTAT), &locked, sizeof(locked));
}

static void SetCore(OSC_TYPE Type, uint32_t Freq, bool Usb)
{
	g_McuOsc.CoreOsc.Type = Type;
	g_McuOsc.CoreOsc.Freq = Freq;
	g_McuOsc.CoreOsc.Accuracy = 20;
	g_McuOsc.CoreOsc.LoadCap = 0;
	g_McuOsc.bUSBClk = Usb;
}

// 48 MHz from the IRC, USB off, and the descriptor of the IRC
static void CheckIrcPll(void)
{
	assert(SystemCoreClock == 48000000);
	assert(LPC_SYSCON->MAINCLKSEL == MAINCLKSEL_PLLOUT);
	assert(LPC_SYSCON->SYSPLLCLKSEL == SYSPLLCLKSEL_IRC);
	assert(LPC_SYSCON->USBCLKDIV == 0);
	assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPLL_PD);
	assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPAD_PD);
	assert(GetHighFreqOscType() == OSC_TYPE_RC);
	assert(GetHighFreqOscFreq() == 12000000);
	assert(SystemMainClkFreq / LPC_SYSCON->SYSAHBCLKDIV == 48000000);
}

int main(void)
{
	s_Regs = mmap((void *)0x40048000, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	assert(s_Regs == (void *)0x40048000);
	assert(mmap((void *)0x4003c000, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x4003c000);

	ResetModel(true, true);
	SystemInit();
	SystemCoreClockUpdate();
	assert(GetHighFreqOscType() == OSC_TYPE_XTAL);
	assert(GetHighFreqOscFreq() == 24000000);
	assert(SystemCoreClock == 48000000);
	assert(SystemCoreClockGet() == 48000000);
	assert(((LPC_SYSCON->SYSPLLCTRL & 31U) + 1U) * 24000000U /
		LPC_SYSCON->SYSAHBCLKDIV == 48000000);
	assert(LPC_FLASHCTRL->FLASHCFG == 0xa5a50002U);
	assert(LPC_SYSCON->USBCLKDIV == 0);
	assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPLL_PD);
	assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPAD_PD);
	// The watchdog oscillator stays off without a low frequency descriptor
	assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_WDTOSC_PD);
	assert(GetLowFreqOscFreq() == 0);

	// Each board oscillator type with USB requested and not
	for (unsigned type = OSC_TYPE_RC; type <= OSC_TYPE_TCXO; type++)
	{
		for (unsigned usb = 0; usb < 2; usb++)
		{
			bool usbEnabled;

			ResetModel(true, true);
			SetCore((OSC_TYPE)type, type == OSC_TYPE_RC ? 12000000U : 24000000U, usb != 0);
			SystemInit();
			assert(SystemCoreClock == 48000000);
			assert(GetHighFreqOscType() == (OSC_TYPE)type);
			assert(LPC_SYSCON->SYSPLLCLKSEL == (type == OSC_TYPE_RC ? 0U : 1U));
			if (type != OSC_TYPE_RC)
			{
				assert((LPC_SYSCON->SYSOSCCTRL & 3U) == (type == OSC_TYPE_TCXO ? 3U : 2U));
			}
			usbEnabled = type != OSC_TYPE_RC && usb != 0;
			assert((LPC_SYSCON->USBCLKDIV != 0) == usbEnabled);
			assert(((LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPLL_PD) == 0) == usbEnabled);
			assert(((LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPAD_PD) == 0) == usbEnabled);
		}
	}

	// Crystal inputs with a 48 MHz setting for the CPU and USB
	for (unsigned i = 0; i < sizeof(s_CrystalInputs) / sizeof(s_CrystalInputs[0]); i++)
	{
		uint32_t freq = s_CrystalInputs[i];
		uint32_t ctrl;
		uint32_t output;
		uint64_t cco;

		ResetModel(true, true);
		SetCore(OSC_TYPE_XTAL, freq, true);
		SystemInit();
		ctrl = LPC_SYSCON->SYSPLLCTRL;
		output = freq * ((ctrl & 31U) + 1U);
		cco = (uint64_t)output * (2U << ((ctrl >> 5) & 3U));
		assert((ctrl & ~127U) == 0);
		assert(cco >= 156000000 && cco <= 320000000);
		assert(output / LPC_SYSCON->SYSAHBCLKDIV == 48000000);
		assert(SystemMainClkFreq == output);
		assert(LPC_SYSCON->USBPLLCTRL == ctrl);
		assert(output / LPC_SYSCON->USBCLKDIV == 48000000);
		assert(SystemCoreClock == 48000000);
		assert((LPC_SYSCON->SYSOSCCTRL & 3U) == (freq > 20000000U ? 2U : 0U));
		assert(GetHighFreqOscType() == OSC_TYPE_XTAL && GetHighFreqOscFreq() == freq);
	}

	// An input with no 48 MHz setting: 48 MHz from the IRC
	for (unsigned i = 0; i < sizeof(s_InvalidInputs) / sizeof(s_InvalidInputs[0]); i++)
	{
		ResetModel(true, true);
		SetCore(OSC_TYPE_XTAL, s_InvalidInputs[i], true);
		g_McuOsc.CoreOsc.LoadCap = 9;
		SystemInit();
		CheckIrcPll();
		assert(g_McuOsc.CoreOsc.Accuracy == 10000 && g_McuOsc.CoreOsc.LoadCap == 0);
	}
	ResetModel(true, true);
	SetCore((OSC_TYPE)99, 12000000, true);
	SystemInit();
	CheckIrcPll();

	// The USB PLL does not lock: the crystal setup fails, the IRC one does
	// not use the USB PLL
	ResetModel(true, false);
	SetCore(OSC_TYPE_XTAL, 12000000, true);
	SystemInit();
	CheckIrcPll();

	// The system PLL does not lock: the CPU stays on the IRC, 12 MHz
	ResetModel(false, true);
	SetCore(OSC_TYPE_XTAL, 12000000, true);
	SystemInit();
	assert(SystemCoreClock == 12000000 && SystemMainClkFreq == 12000000);
	assert(LPC_SYSCON->MAINCLKSEL == MAINCLKSEL_IRC);
	assert(LPC_SYSCON->SYSAHBCLKDIV == 1);
	assert((LPC_SYSCON->PDRUNCFG & (PDRUNCFG_SYSPLL_PD | PDRUNCFG_USBPLL_PD)) ==
		(PDRUNCFG_SYSPLL_PD | PDRUNCFG_USBPLL_PD));
	assert(LPC_SYSCON->USBCLKDIV == 0);
	assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_USBPAD_PD);
	// The descriptor keeps the oscillator that did not start
	assert(GetHighFreqOscType() == OSC_TYPE_XTAL);
	assert(GetHighFreqOscFreq() == 12000000);

	// An RC descriptor keeps the application accuracy
	ResetModel(true, true);
	SetCore(OSC_TYPE_RC, 0, true);
	g_McuOsc.CoreOsc.Accuracy = 3000;
	SystemInit();
	CheckIrcPll();
	assert(g_McuOsc.CoreOsc.Accuracy == 3000);

	// Watchdog oscillator from the low frequency descriptor
	{
		static const uint32_t requests[] = { 1, UINT32_MAX, 32768 };
		static const uint32_t rates[] = { 9375, 2300000, 32812 };

		for (unsigned i = 0; i < sizeof(requests) / sizeof(requests[0]); i++)
		{
			ResetModel(true, true);
			SetCore(OSC_TYPE_XTAL, 12000000, false);
			g_McuOsc.LowPwrOsc.Type = OSC_TYPE_RC;
			g_McuOsc.LowPwrOsc.Freq = requests[i];
			SystemInit();
			assert(GetLowFreqOscFreq() == rates[i]);
			assert(GetLowFreqOscAcc() == 400000);
			assert((LPC_SYSCON->PDRUNCFG & PDRUNCFG_WDTOSC_PD) == 0);
			assert(LPC_SYSCON->WDTOSCCTRL != 0);
			// The watchdog is not started, its bus clock stays off
			assert((LPC_SYSCON->SYSAHBCLKCTRL & (1U << 15)) == 0);
		}
		// No watchdog oscillator for a crystal descriptor
		ResetModel(true, true);
		g_McuOsc.LowPwrOsc.Type = OSC_TYPE_XTAL;
		g_McuOsc.LowPwrOsc.Freq = 32768;
		SystemInit();
		assert(LPC_SYSCON->PDRUNCFG & PDRUNCFG_WDTOSC_PD);
		assert(LPC_SYSCON->WDTOSCCTRL == 0);
		assert(GetLowFreqOscFreq() == 32768);
	}

	// The stored main clock, not the registers, after the setup
	ResetModel(true, true);
	SetCore(OSC_TYPE_XTAL, 12000000, false);
	SystemInit();
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_WDT;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 48000000);
	// Shared clock requests count the users of clocks SystemInit started
	ResetModel(true, true);
	SetCore(OSC_TYPE_XTAL, 12000000, false);
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_RC;
	g_McuOsc.LowPwrOsc.Freq = 0;
	SystemInit();
	SystemHighFreqClockDisable();
	assert(SystemHighFreqClockEnable());
	assert(SystemHighFreqClockEnable());
	SystemHighFreqClockDisable();
	SystemHighFreqClockDisable();
	assert(SystemHighFreqClockEnable());
	SystemHighFreqClockDisable();
	assert(SystemHighFreqClockStartTime() == 0);
	// No watchdog oscillator without a low frequency descriptor
	assert(!SystemLowFreqClockEnable());
	SystemLowFreqClockDisable();
	g_McuOsc.LowPwrOsc.Freq = 32768;
	SystemInit();
	assert(SystemLowFreqClockEnable());
	assert(SystemLowFreqClockEnable());
	SystemLowFreqClockDisable();
	SystemLowFreqClockDisable();
	SystemLowFreqClockDisable();
	assert(SystemLowFreqClockEnable());
	SystemLowFreqClockDisable();
	assert(SystemLowFreqClockStartTime() == 0);

	// Running modes: LOW_POWER doubles SYSAHBCLKDIV, the flash access time
	// follows the system clock, nothing else changes
	{
		static const struct { OSC_TYPE Type; uint32_t Freq; bool SysLock; uint32_t Div; } cases[] = {
			{ OSC_TYPE_XTAL, 12000000, true, 2 },	// 96 MHz PLL output
			{ OSC_TYPE_TCXO, 24000000, true, 2 },	// 96 MHz PLL output
			{ OSC_TYPE_XTAL, 12000000, false, 1 },	// 12 MHz IRC, no PLL
		};
		uint8_t before[sizeof(LPC_SYSCON_Type)];

		for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
		{
			uint32_t perf;

			ResetModel(cases[i].SysLock, true);
			SetCore(cases[i].Type, cases[i].Freq, false);
			SystemInit();
			perf = SystemCoreClock;
			assert(LPC_SYSCON->SYSAHBCLKDIV == cases[i].Div);
			assert(SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE);
			assert(SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE));
			assert(SystemCoreClock == perf);
			memcpy(before, s_Regs, sizeof(before));
			assert(SystemPowerModeSet(SYSTEM_POWER_MODE_LOW_POWER));
			assert(SystemPowerModeGet() == SYSTEM_POWER_MODE_LOW_POWER);
			assert(LPC_SYSCON->SYSAHBCLKDIV == cases[i].Div * 2U);
			assert(SystemCoreClock == perf / 2U && SystemCoreClockGet() == perf / 2U);
			assert(SystemPeriphClockGet(0) == perf / 2U);
			assert(SystemMicroSecLoopCnt == ((uint64_t)SystemCoreClock * 1000000ULL) / 16000000U);
			assert(LPC_FLASHCTRL->FLASHCFG == (0xa5a50000U | (perf / 2U > 20000000U ? 1U : 0U)));
			LPC_SYSCON->SYSAHBCLKDIV = cases[i].Div;
			assert(memcmp(before, s_Regs, sizeof(before)) == 0);
			LPC_SYSCON->SYSAHBCLKDIV = cases[i].Div * 2U;
			assert(SystemPowerModeSet(SYSTEM_POWER_MODE_LOW_POWER));
			assert(LPC_SYSCON->SYSAHBCLKDIV == cases[i].Div * 2U);
			assert(SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE));
			assert(SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE);
			assert(LPC_SYSCON->SYSAHBCLKDIV == cases[i].Div);
			assert(SystemCoreClock == perf);
			assert(LPC_FLASHCTRL->FLASHCFG == (0xa5a50000U | (perf > 40000000U ? 2U : (perf > 20000000U ? 1U : 0U))));
			assert(memcmp(before, s_Regs, sizeof(before)) == 0);
			assert(!SystemPowerModeSet((SYSTEM_POWER_MODE)2));
			assert(SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE);
			assert(memcmp(before, s_Regs, sizeof(before)) == 0);
		}
	}
	puts("LPC11Uxx SystemInit clock setup, IRC fallback, watchdog oscillator, clock requests and running modes: PASS");
}
