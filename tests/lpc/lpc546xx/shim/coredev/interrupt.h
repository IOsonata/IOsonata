// Host stand-in for the interrupt masking functions
#pragma once
#include <stdint.h>
extern uint32_t g_IrqMasked;
static inline uint32_t DisableInterrupt(void) { uint32_t old = g_IrqMasked; g_IrqMasked = 1; return old; }
static inline void EnableInterrupt(uint32_t State) { g_IrqMasked = State; }
