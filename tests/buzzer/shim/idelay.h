#pragma once
#include <stdint.h>
extern uint64_t g_DelayUs;
inline void usDelay(uint32_t us) { g_DelayUs += us; }
