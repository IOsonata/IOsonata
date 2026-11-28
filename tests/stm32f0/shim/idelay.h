// Host stand-in for the delay functions: delays return at once
#pragma once
#include <stdint.h>
#include "coredev/system_core_clock.h"

static inline void usDelay(uint32_t n) { (void)n; }
static inline void nsDelay(uint32_t n) { (void)n; }
static inline void msDelay(uint32_t n) { (void)n; }

extern uint32_t SystemMicroSecLoopCnt;
extern uint32_t SystemnsDelayFactor;

// The ARM delay factor arithmetic, ArmDelayClockSetDivisors. The ARM
// idelay.h it includes is replaced by this file.
#define __IDELAY_H__
#include "../../../ARM/include/system_core_clock_calibration.h"
