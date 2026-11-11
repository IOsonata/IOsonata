// Host syntax check of the production SAM4E headers, not a CMSIS implementation.
#ifndef SAM4E_POWER_TEST_CORE_H
#define SAM4E_POWER_TEST_CORE_H
#include <stdint.h>
#define __I volatile const
#define __O volatile
#define __IO volatile
#define __WEAK __attribute__((weak))
#define __INTERRUPT_H__
static inline uint32_t DisableInterrupt(void) { return 0; }
static inline void EnableInterrupt(uint32_t State) { (void)State; }
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __WFE(void) {}
#endif
