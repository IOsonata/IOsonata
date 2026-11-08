#ifndef TEST_NRF53_INTERRUPT_H
#define TEST_NRF53_INTERRUPT_H
#include <stdint.h>
extern int g_TestCriticalDepth;
static inline uint32_t DisableInterrupt(void)
{
	g_TestCriticalDepth++;
	return 0U;
}
static inline void EnableInterrupt(uint32_t State)
{
	(void)State;
	g_TestCriticalDepth--;
}
#endif
