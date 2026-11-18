// Host syntax shim, not a CMSIS implementation.
#pragma once
#include <stdint.h>
#define __INTERRUPT_H__
struct PowerTestScb { uint32_t SCR; };
extern PowerTestScb g_PowerTestScb;
#define SCB (&g_PowerTestScb)
#define SCB_SCR_SLEEPONEXIT_Msk (1UL << 1)
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
static inline uint32_t __get_IPSR(void) { return 0U; }
static inline void __DSB(void) {}
static inline void __WFI(void) { PowerTestWfi(); }

static inline void __ISB(void) {}
static inline void __NOP(void) {}

extern uint32_t g_PowerTestIrqEnabled;
extern uint32_t g_PowerTestIrqPending;
static inline uint32_t NVIC_GetEnableIRQ(IRQn_Type) { return g_PowerTestIrqEnabled; }
static inline uint32_t NVIC_GetPendingIRQ(IRQn_Type) { return g_PowerTestIrqPending; }
static inline void NVIC_EnableIRQ(IRQn_Type) { g_PowerTestIrqEnabled = 1; }
static inline void NVIC_DisableIRQ(IRQn_Type) { g_PowerTestIrqEnabled = 0; }
static inline void NVIC_ClearPendingIRQ(IRQn_Type) { g_PowerTestIrqPending = 0; }
