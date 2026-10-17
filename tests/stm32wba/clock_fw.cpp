/**-------------------------------------------------------------------------
@file	clock_fw.cpp

@brief	STM32WBA core clock test firmware

Runs on the instruction set emulator of run_clock.py, which models the RCC,
PWR and flash registers used by the clock setup. Each step calls the
library clock functions and records the result in g_ClkTestResult.
ClkTestMark tells the emulator which step starts.

Not hardware validation.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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
#include <stdint.h>

#include "coredev/system_core_clock.h"

extern "C" void SystemInit(void);

#define CLK_TEST_MAX_RESULT		64

extern "C" volatile int32_t g_ClkTestResult[CLK_TEST_MAX_RESULT];
extern "C" volatile uint32_t g_ClkTestDone;

extern "C" void ClkTestMark(int Id) __attribute__((noinline));
extern "C" void ClkTestDoneHook(void) __attribute__((noinline));

volatile int32_t g_ClkTestResult[CLK_TEST_MAX_RESULT];
volatile uint32_t g_ClkTestDone;

extern "C" void ClkTestMark(int Id)
{
	__asm volatile ("" : : "r" (Id) : "memory");
}

extern "C" void ClkTestDoneHook(void)
{
	__asm volatile ("" ::: "memory");
}

extern "C" uint32_t SystemCoreClock;

static int s_R;

static void Rec(int32_t v)
{
	g_ClkTestResult[s_R++] = v;
}

static void RecClocks(void)
{
	Rec((int32_t)SystemCoreClock);
	Rec((int32_t)SystemPeriphClockGet(0));
	Rec((int32_t)SystemPeriphClockGet(1));
	Rec((int32_t)SystemPeriphClockGet(2));
}

extern "C" int main(void)
{
	// 0 : default, HSI16 as the PLL source
	ClkTestMark(0);
	SystemInit();
	RecClocks();
	Rec((int32_t)g_McuOsc.CoreOsc.Type);

	// 1 : 32 MHz crystal, HSE as the PLL source
	ClkTestMark(1);
	Rec(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000) ? 1 : 0);
	RecClocks();
	Rec((int32_t)g_McuOsc.CoreOsc.Type);

	// 2 : a crystal frequency the part does not take
	ClkTestMark(2);
	Rec(SystemCoreClockSelect(OSC_TYPE_XTAL, 24000000) ? 1 : 0);
	Rec((int32_t)SystemCoreClock);

	// 3 : back to the internal RC
	ClkTestMark(3);
	Rec(SystemCoreClockSelect(OSC_TYPE_RC, 0) ? 1 : 0);
	RecClocks();

	// 4 : crystal that does not start, the emulator keeps HSERDY low
	ClkTestMark(4);
	Rec(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000) ? 1 : 0);
	RecClocks();
	Rec((int32_t)g_McuOsc.CoreOsc.Type);
	Rec((int32_t)g_McuOsc.CoreOsc.Freq);

	// 5 : peripheral clock prescalers
	ClkTestMark(5);
	Rec((int32_t)SystemPeriphClockSet(0, 50000000));
	Rec((int32_t)SystemPeriphClockSet(1, 25000000));
	Rec((int32_t)SystemPeriphClockSet(2, 0));
	Rec((int32_t)SystemPeriphClockSet(3, 1000));
	RecClocks();

	// Voltage acknowledgements must arrive before increasing the clock.
	ClkTestMark(6);
	SystemInit();
	RecClocks();

	ClkTestMark(7);
	SystemInit();
	RecClocks();

	// Restore PLL operation before refusing the switch back to HSI.
	ClkTestMark(8);
	SystemInit();
	RecClocks();

	ClkTestMark(9);
	SystemInit();
	RecClocks();

	// Refuse PLL shutdown after the core has returned to HSI.
	ClkTestMark(10);
	SystemInit();
	RecClocks();

	ClkTestMark(11);
	g_ClkTestResult[CLK_TEST_MAX_RESULT - 1] = s_R;
	g_ClkTestDone = 1;
	ClkTestDoneHook();

	while (1)
	{
	}

	return 0;
}

// Minimal start: stack, data and bss, then main. SystemInit is not needed.
extern "C" uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

extern "C" void ResetHandler(void)
{
	uint32_t *src = &_sidata;

	for (uint32_t *dst = &_sdata; dst < &_edata; )
	{
		*dst++ = *src++;
	}
	for (uint32_t *dst = &_sbss; dst < &_ebss; )
	{
		*dst++ = 0;
	}
	main();
	while (1)
	{
	}
}

extern "C" __attribute__((section(".isr_vector"), used))
void (* const __Vectors[16 + 128])(void) = {
	(void (*)(void))&_estack,
	ResetHandler,
};
