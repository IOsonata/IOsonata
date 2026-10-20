
#pragma once
#include <stdint.h>
#define __IO volatile
#define __I volatile const
#define __IM volatile const
#define __O volatile
inline void NVIC_DisableIRQ(int) {}
inline void NVIC_EnableIRQ(int) {}
inline void NVIC_SetPriority(int, int) {}
inline void NVIC_ClearPendingIRQ(int) {}
inline void NVIC_SetPendingIRQ(int) {}

void __DSB();
inline void __ISB() {}
struct ScbMock { uint32_t CCR, ICSR, VTOR; };
extern ScbMock scb_mock;
#define SCB (&scb_mock)
#define SCB_CCR_DC_Msk (1U << 16)
#define SCB_CCR_IC_Msk (1U << 17)
#define SCB_ICSR_PENDSTCLR_Msk (1U << 25)
void SCB_InvalidateDCache_by_Addr(uint32_t *, int32_t);
void SCB_InvalidateICache();
inline void SCB_DisableDCache() {}
inline void SCB_DisableICache() {}
inline void NVIC_SystemReset() {}
struct NvicMock { uint32_t ICER[8], ICPR[8]; };
extern NvicMock nvic_mock;
#define NVIC (&nvic_mock)
struct SysTickMock { uint32_t CTRL; };
extern SysTickMock systick_mock;
#define SysTick (&systick_mock)
inline void __set_CONTROL(unsigned) {}
inline void __set_BASEPRI(unsigned) {}
inline void __set_FAULTMASK(unsigned) {}
