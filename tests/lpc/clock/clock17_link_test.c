/*--------------------------------------------------------------------------
File   : clock17_link_test.c

Desc   : C and C++ callers for the LPC17xx public clock APIs

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#include <assert.h>
#include "system_LPC17xx.h"
#include "coredev/system_core_clock.h"

// Application definition must override the port default in C and C++.
McuOsc_t g_McuOsc = {{OSC_TYPE_RC, 4000000, 123, 0},
	{OSC_TYPE_XTAL, 32768, 20, 0}, false};

#ifdef __cplusplus
extern "C" {
#endif
int TestClock17Run(void);
#ifdef __cplusplus
}
#endif

int main(void)
{
	int result;

	assert(g_McuOsc.CoreOsc.Type == OSC_TYPE_RC);
	assert(g_McuOsc.CoreOsc.Freq == 4000000U);
	assert(!SystemCoreClockSelect(OSC_TYPE_RC, 0));
	assert(SystemCoreClockGet() == 4000000U);
	assert(SystemPeriphClockGet(-1) == 0);
	assert(SystemPeriphClockSet(-1, UINT32_MAX) == 0);
	// Both entry points must link from the public header without local declarations.
	assert(SystemSetCoreClock(120000000, -1) == 0);
	assert(SystemCoreClockSet(120000000, 0) == 0);

	result = TestClock17Run();
	assert(g_McuOsc.CoreOsc.Accuracy == 123);
	assert(SystemCoreClockGet() == SystemCoreClock);
	assert(SystemPeriphClockGet(LPC17_PCLK_RTC) == SystemCoreClock / 8U);

	return result;
}
