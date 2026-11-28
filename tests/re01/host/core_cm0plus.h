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
#define __SEV() ((void)0)
#define __WFE() ((void)0)
#define __CLZ(n) ((n) ? __builtin_clz((uint32_t)(n)) : 32U)
#define __DSB() ((void)0)
#define __ISB() ((void)0)
// The parts of SCB, SysTick and NVIC that SystemSleep uses, in host memory.
// WFI calls the test, which records the sleep selection.
typedef struct { volatile uint32_t CPUID, ICSR, VTOR, AIRCR, SCR, CCR; } SCB_Type;
typedef struct { volatile uint32_t CTRL, LOAD, VAL, CALIB; } SysTick_Type;
typedef struct { volatile uint32_t ISER[1], ICER[1], ISPR[1], ICPR[1]; } NVIC_Type;
#ifdef __cplusplus
extern "C" {
#endif
extern SCB_Type g_TestScb;
extern SysTick_Type g_TestSysTick;
extern NVIC_Type g_TestNvic;
void TestWfi(void);
#ifdef __cplusplus
}
#endif
#define SCB (&g_TestScb)
#define SysTick (&g_TestSysTick)
#define NVIC (&g_TestNvic)
#define SCB_SCR_SLEEPONEXIT_Msk (1UL << 1)
#define SCB_SCR_SLEEPDEEP_Msk (1UL << 2)
#define SCB_ICSR_PENDSTCLR_Msk (1UL << 25)
#define SCB_ICSR_PENDSTSET_Msk (1UL << 26)
#define SCB_ICSR_PENDSVCLR_Msk (1UL << 27)
#define SCB_ICSR_PENDSVSET_Msk (1UL << 28)
#define SysTick_CTRL_TICKINT_Msk (1UL << 1)
#define __WFI() TestWfi()
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t Mask) { (void)Mask; }
static inline void __disable_irq(void) {}
static inline void NVIC_EnableIRQ(IRQn_Type Irq) { (void)Irq; }
static inline void NVIC_DisableIRQ(IRQn_Type Irq) { (void)Irq; }
static inline void NVIC_ClearPendingIRQ(IRQn_Type Irq) { (void)Irq; }
static inline void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority) { (void)Irq; (void)Priority; }
#endif
