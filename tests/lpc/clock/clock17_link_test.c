/*--------------------------------------------------------------------------
File   : clock17_link_test.c

Desc   : C and C++ callers for the LPC17xx public clock APIs

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#include <assert.h>
#include "system_LPC17xx.h"
#include "coredev/system_core_clock.h"

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

	assert(SystemCoreClockGet() == 4000000U);
	assert(SystemPeriphClockGet(-1) == 0);
	// Both entry points must link from the public header without local declarations.
	assert(SystemSetCoreClock(120000000, -1) == 0);
	assert(SystemCoreClockSet(120000000, 0) == 0);

	result = TestClock17Run();
	assert(SystemCoreClockGet() == SystemCoreClock);
	assert(SystemPeriphClockGet(LPC17_PCLK_RTC) == SystemCoreClock / 8U);

	return result;
}
