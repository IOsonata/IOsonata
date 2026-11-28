// Host stand-in for the delay functions: the test advances its model time
#pragma once
#include <stdint.h>
#include "coredev/system_core_clock.h"

void ModelDelayUs(uint32_t Us);
static inline void usDelay(uint32_t Us) { ModelDelayUs(Us); }
static inline void nsDelay(uint32_t Ns) { ModelDelayUs(Ns / 1000U); }
static inline void msDelay(uint32_t Ms) { ModelDelayUs(Ms * 1000U); }

extern uint32_t SystemMicroSecLoopCnt;
extern uint32_t SystemnsDelayFactor;

// The ARM delay factor arithmetic, ArmDelayClockSetDivisors. The ARM
// idelay.h it includes is replaced by this file.
#define __IDELAY_H__
#include "../../../../ARM/include/system_core_clock_calibration.h"
