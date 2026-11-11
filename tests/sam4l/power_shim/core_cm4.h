// Host syntax shim, not a CMSIS implementation.
#pragma once
#include <stdint.h>
#define __INTERRUPT_H__
struct PowerTestScb { uint32_t SCR; };
extern PowerTestScb g_PowerTestScb;
#define SCB (&g_PowerTestScb)
#define SCB_SCR_SLEEPDEEP_Msk (1UL << 2)
extern uint32_t g_PowerTestMask;
void PowerTestWfi(void);
static inline uint32_t DisableInterrupt(void)
{
	uint32_t state = g_PowerTestMask;
	g_PowerTestMask = 1;
	return state;
}
static inline void EnableInterrupt(uint32_t State) { g_PowerTestMask = State; }
static inline void __DSB(void) {}
static inline void __WFI(void) { PowerTestWfi(); }
