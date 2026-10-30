// CMSIS interrupt state modeled by the SDC host test.
#ifndef __NRF_STUB_H__
#define __NRF_STUB_H__
#include <stdint.h>
typedef int IRQn_Type;
#define SWI5_EGU5_IRQn 25
#define __NVIC_PRIO_BITS 3
uint32_t NVIC_GetPriority(IRQn_Type Irq);
uint32_t __get_IPSR(void);
uint32_t __get_BASEPRI(void);
void __set_BASEPRI_MAX(uint32_t Value);
void __set_BASEPRI(uint32_t Value);
static inline void __DSB(void) {}
static inline void __ISB(void) {}
#endif
