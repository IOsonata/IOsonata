/*--------------------------------------------------------------------------
File   : clock17_link_test.c

Desc   : C and C++ callers for the LPC17xx public clock APIs

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#include <assert.h>
#include "system_LPC17xx.h"

#ifdef __cplusplus
extern "C" {
#endif
int TestClock17Run(void);
#ifdef __cplusplus
}
#endif

int main(void)
{
	// Both entry points must link from the public header without local declarations.
	assert(SystemSetCoreClock(120000000, -1) == 0);
	assert(SystemCoreClockSet(120000000, 0) == 0);

	return TestClock17Run();
}
