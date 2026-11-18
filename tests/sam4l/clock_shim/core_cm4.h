// Host register tests only; not a CMSIS implementation.
#pragma once
#include <stdint.h>
extern uint32_t g_TestMask;
extern uint32_t g_TestIpsr;
void ClockTestHalt(void);
static inline uint32_t __get_PRIMASK(void) { return g_TestMask; }
static inline uint32_t __get_IPSR(void) { return g_TestIpsr; }
static inline void __disable_irq(void) { g_TestMask = 1; }
static inline void __set_PRIMASK(uint32_t Mask) { g_TestMask = Mask; }
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __NOP(void) { ClockTestHalt(); }
