// Host replacement of the CMSIS core header for the tests of tests/stm32 and
// the STM32 family tests.
// Register qualifiers, barriers and the NVIC functions the drivers call. The
// NVIC calls are recorded by the test.
#ifndef __TEST_CORE_H__
#define __TEST_CORE_H__

#include <stdint.h>

#define __IO	volatile
#define __I		volatile const
#define __O		volatile
#define __IM	volatile const
#define __OM	volatile
#define __IOM	volatile

#define __STATIC_INLINE			static inline
#define __STATIC_FORCEINLINE	static inline
#define __WEAK					__attribute__((weak))

static inline void __DMB(void) { __asm__ volatile("" ::: "memory"); }
static inline void __DSB(void) { __asm__ volatile("" ::: "memory"); }
static inline void __ISB(void) { __asm__ volatile("" ::: "memory"); }
static inline void __NOP(void) {}
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t v) { (void)v; }
static inline void __disable_irq(void) {}

#ifdef __cplusplus
extern "C" {
#endif
// Op : 0 enable, 1 disable, 2 clear pending, 3 set priority, 4 set pending
void TestNvic(int Op, int IrqNo, uint32_t Prio);
uint32_t TestNvicGetEnable(int IrqNo);
#ifdef __cplusplus
}
#endif

#define NVIC_EnableIRQ(n)			TestNvic(0, (int)(n), 0)
#define NVIC_DisableIRQ(n)			TestNvic(1, (int)(n), 0)
#define NVIC_GetEnableIRQ(n)		TestNvicGetEnable((int)(n))
#define NVIC_ClearPendingIRQ(n)		TestNvic(2, (int)(n), 0)
#define NVIC_SetPriority(n, p)		TestNvic(3, (int)(n), (uint32_t)(p))
#define NVIC_SetPendingIRQ(n)		TestNvic(4, (int)(n), 0)

#endif
