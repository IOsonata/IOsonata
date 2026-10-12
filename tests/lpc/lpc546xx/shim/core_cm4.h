// Host stand-in for the CMSIS Cortex-M4 core header. The NVIC operations go
// to the register model of the test.
#pragma once
#include <stdint.h>
#define __IO volatile
#define __I volatile const
#define __O volatile
#define __IM volatile const
#define __OM volatile
#define __IOM volatile
#define __STATIC_INLINE static inline
#define __WEAK __attribute__((weak))
void NVIC_ClearPendingIRQ(IRQn_Type IRQn);
void NVIC_SetPendingIRQ(IRQn_Type IRQn);
void NVIC_SetPriority(IRQn_Type IRQn, uint32_t Priority);
void NVIC_EnableIRQ(IRQn_Type IRQn);
void NVIC_DisableIRQ(IRQn_Type IRQn);
