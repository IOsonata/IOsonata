#pragma once
#include <stdint.h>
#define __IO volatile
#define __I volatile const
#define __O volatile
#define __STATIC_INLINE static inline
#define __STATIC_FORCEINLINE static inline
#define __WEAK __attribute__((weak))
static inline void NVIC_ClearPendingIRQ(IRQn_Type n) {(void)n;}
static inline void NVIC_SetPriority(IRQn_Type n, uint32_t p) {(void)n;(void)p;}
static inline void NVIC_EnableIRQ(IRQn_Type n) {(void)n;}
static inline void NVIC_DisableIRQ(IRQn_Type n) {(void)n;}
