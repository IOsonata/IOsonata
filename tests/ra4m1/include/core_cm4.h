/* REDUCED TEST SHIM ONLY. Production builds use the project's CMSIS Core. */
#ifndef TEST_CORE_CM4_H
#define TEST_CORE_CM4_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
#define TEST_STATIC_ASSERT static_assert
#else
#define TEST_STATIC_ASSERT _Static_assert
#endif
#define __WEAK __attribute__((weak))
typedef struct { volatile uint32_t CPUID, ICSR, VTOR, AIRCR, SCR, CCR;
 volatile uint8_t SHP[12]; volatile uint32_t SHCSR, CFSR, HFSR, DFSR, MMFAR,
 BFAR, AFSR, PFR[2], DFR, ADR, MMFR[4], ISAR[5], RESERVED0[5], CPACR;
} SCB_Type;
TEST_STATIC_ASSERT(offsetof(SCB_Type, VTOR)==8, "VTOR test layout");
TEST_STATIC_ASSERT(offsetof(SCB_Type, CPACR)==0x88, "CPACR test layout");
#ifdef RA4M1_HOST_TEST
extern SCB_Type test_scb;
extern uint32_t test_primask, test_disabled, test_cleared;
#define SCB (&test_scb)
static inline void __NOP(void) {}
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline uint32_t __get_PRIMASK(void) {return test_primask;}
static inline void __disable_irq(void) {test_primask=1;}
static inline void __set_PRIMASK(uint32_t v) {test_primask=v;}
#ifdef RA4M1_IO_TEST
extern uint32_t test_enabled, test_active, test_pending;
extern uint32_t test_priority[32];
static inline uint32_t NVIC_GetEnableIRQ(IRQn_Type n) {return (test_enabled >> (unsigned)n) & 1U;}
static inline uint32_t NVIC_GetActive(IRQn_Type n) {return (test_active >> (unsigned)n) & 1U;}
static inline void NVIC_EnableIRQ(IRQn_Type n) {test_enabled |= 1UL << (unsigned)n;}
static inline void NVIC_SetPriority(IRQn_Type n, uint32_t p) {test_priority[(unsigned)n]=p;}
static inline void NVIC_DisableIRQ(IRQn_Type n) {test_disabled |= 1UL << (unsigned)n; test_enabled &= ~(1UL << (unsigned)n);}
#else
static inline void NVIC_DisableIRQ(IRQn_Type n) {test_disabled |= 1UL << (unsigned)n;}
#endif
static inline void NVIC_ClearPendingIRQ(IRQn_Type n) {test_cleared |= 1UL << (unsigned)n;
#ifdef RA4M1_IO_TEST
 test_pending &= ~(1UL << (unsigned)n);
#endif
}
#else
static inline uint32_t NVIC_GetEnableIRQ(IRQn_Type n) {return (*(volatile uint32_t *)0xE000E100UL >> (unsigned)n) & 1U;}
static inline uint32_t NVIC_GetActive(IRQn_Type n) {return (*(volatile uint32_t *)0xE000E300UL >> (unsigned)n) & 1U;}
static inline void NVIC_EnableIRQ(IRQn_Type n) {*(volatile uint32_t *)0xE000E100UL=1UL << (unsigned)n;}
static inline void NVIC_SetPriority(IRQn_Type n, uint32_t p) {((volatile uint8_t *)0xE000E400UL)[(unsigned)n]=(uint8_t)(p << (8U-__NVIC_PRIO_BITS));}
#define SCB ((SCB_Type *)0xE000ED00UL)
static inline void __NOP(void) {__asm volatile("nop");}
static inline void __DSB(void) {__asm volatile("dsb 0xF":::"memory");}
static inline void __ISB(void) {__asm volatile("isb 0xF":::"memory");}
static inline uint32_t __get_PRIMASK(void) {uint32_t v;__asm volatile("mrs %0, primask":"=r"(v));return v;}
static inline void __disable_irq(void) {__asm volatile("cpsid i":::"memory");}
static inline void __set_PRIMASK(uint32_t v) {__asm volatile("msr primask, %0"::"r"(v):"memory");}
static inline void NVIC_DisableIRQ(IRQn_Type n) {*(volatile uint32_t *)0xE000E180UL=1UL<<(unsigned)n;}
static inline void NVIC_ClearPendingIRQ(IRQn_Type n) {*(volatile uint32_t *)0xE000E280UL=1UL<<(unsigned)n;}
#endif
#endif
