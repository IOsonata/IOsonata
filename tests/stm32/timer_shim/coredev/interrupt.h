// Host replacement of coredev/interrupt.h for tests/stm32/timer_tim_test.cpp.
// Records the interrupts disabled state, the test checks each
// DisableInterrupt is matched by EnableInterrupt.
#ifndef __TEST_INTERRUPT_H__
#define __TEST_INTERRUPT_H__

#include <stdint.h>

extern uint32_t g_TestIrqState;

static inline uint32_t DisableInterrupt(void)
{
	uint32_t old = g_TestIrqState;

	g_TestIrqState = 1;

	return old;
}

static inline void EnableInterrupt(uint32_t State)
{
	g_TestIrqState = State;
}

#endif // __TEST_INTERRUPT_H__
