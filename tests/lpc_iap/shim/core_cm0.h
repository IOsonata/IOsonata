// Host substitutes for CMSIS interrupt intrinsics; peripheral types remain vendor CMSIS.
#pragma once
#include "../../lpc_uart/shim/core_cm0.h"
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t Mask);
