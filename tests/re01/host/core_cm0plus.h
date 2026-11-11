/* Host-only CMSIS shim. The generated RE01 register definitions remain real. */
#ifndef RE01_TEST_CORE_CM0PLUS_H
#define RE01_TEST_CORE_CM0PLUS_H
#include <stdint.h>
#define __I volatile const
#define __O volatile
#define __IO volatile
#define __IM volatile const
#define __OM volatile
#define __IOM volatile
#define __WEAK __attribute__((weak))
#define __NOP() ((void)0)
#define __WFE() ((void)0)
#define __CLZ(n) ((n) ? __builtin_clz((uint32_t)(n)) : 32U)
#define __DSB() ((void)0)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t Mask) { (void)Mask; }
static inline void __disable_irq(void) {}
static inline void NVIC_EnableIRQ(IRQn_Type Irq) { (void)Irq; }
static inline void NVIC_DisableIRQ(IRQn_Type Irq) { (void)Irq; }
static inline void NVIC_ClearPendingIRQ(IRQn_Type Irq) { (void)Irq; }
static inline void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority) { (void)Irq; (void)Priority; }
#endif
