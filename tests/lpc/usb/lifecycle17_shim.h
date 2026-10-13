/* LPC1769 lifecycle interrupt model. MIT License, I-SYST, 2026. */
#pragma once
#include "../startup/system17_shim.h"
#define __INTERRUPT_H__
static inline uint32_t DisableInterrupt(void)
{
	uint32_t state = TestGetPrimask();
	TestDisableIRQ();
	return state;
}
static inline void EnableInterrupt(uint32_t State)
{
	TestSetPrimask(State);
}
