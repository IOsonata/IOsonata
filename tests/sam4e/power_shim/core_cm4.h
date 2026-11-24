// Host syntax check of the production SAM4E headers, not a CMSIS implementation.
#ifndef SAM4E_POWER_TEST_CORE_H
#define SAM4E_POWER_TEST_CORE_H
#include <stdint.h>
#define __I volatile const
#define __O volatile
#define __IO volatile
#define __WEAK __attribute__((weak))
#define __INTERRUPT_H__
typedef struct { uint32_t SCR; } TestScb;
extern TestScb s_TestScb;
#define SCB (&s_TestScb)
#define SCB_SCR_SLEEPDEEP_Msk 4U
#define SCB_SCR_SLEEPONEXIT_Msk 2U
static inline unsigned __get_IPSR(void) { return 0; }
static inline unsigned __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(unsigned state) { (void)state; }
static inline void __WFI(void) {}
static inline uint32_t DisableInterrupt(void) { return 0; }
static inline void EnableInterrupt(uint32_t State) { (void)State; }
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __WFE(void) {}
#endif
