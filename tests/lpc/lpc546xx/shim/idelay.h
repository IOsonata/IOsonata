// Host stand-in for the delay functions: the test advances its model time
#pragma once
#include <stdint.h>
void ModelDelayUs(uint32_t Us);
static inline void usDelay(uint32_t Us) { ModelDelayUs(Us); }
static inline void msDelay(uint32_t Ms) { ModelDelayUs(Ms * 1000U); }
