/**-------------------------------------------------------------------------
@file	vectors_stm32wba65.c

@brief	STM32WBA65 interrupt vector table for IOsonata ResetEntry.

@author	Hoang Nguyen Hoan
@date	October 10, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/

#include <stdint.h>
#include "stm32wbaxx.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);

void Default_Handler(void)
{
    for (;;) { __NOP(); }
}

void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SecureFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));
void WWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_S_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TAMP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RAMCFG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_S_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GTZC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_S_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI8_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI9_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI11_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI12_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI13_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI14_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI15_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void IWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SAES_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPDMA1_Channel0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPDMA1_Channel1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPDMA1_Channel2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPDMA1_Channel3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPDMA1_Channel4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPDMA1_Channel5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPDMA1_Channel6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPDMA1_Channel7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPUART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPTIM1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPTIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM16_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM17_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void COMP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SAI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TSC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void AES_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PKA_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ICACHE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RADIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HSEM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HSEM_S_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WKUP_S_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_AUDIOSYNC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OTG_HS_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C4_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C4_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI19_RADIO_IO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI20_RADIO_IO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/* The generic ARM linker script keeps .ivector at the flash origin. */
__attribute__((used, section(".ivector"), aligned(512)))
void (* const __Vectors[])(void) = {
    (void (*)(void))&__StackTop,
    ResetEntry,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    SecureFault_Handler,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    WWDG_IRQHandler,
    PVD_IRQHandler,
    RTC_IRQHandler,
    RTC_S_IRQHandler,
    TAMP_IRQHandler,
    RAMCFG_IRQHandler,
    FLASH_IRQHandler,
    FLASH_S_IRQHandler,
    GTZC_IRQHandler,
    RCC_IRQHandler,
    RCC_S_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    EXTI5_IRQHandler,
    EXTI6_IRQHandler,
    EXTI7_IRQHandler,
    EXTI8_IRQHandler,
    EXTI9_IRQHandler,
    EXTI10_IRQHandler,
    EXTI11_IRQHandler,
    EXTI12_IRQHandler,
    EXTI13_IRQHandler,
    EXTI14_IRQHandler,
    EXTI15_IRQHandler,
    IWDG_IRQHandler,
    SAES_IRQHandler,
    GPDMA1_Channel0_IRQHandler,
    GPDMA1_Channel1_IRQHandler,
    GPDMA1_Channel2_IRQHandler,
    GPDMA1_Channel3_IRQHandler,
    GPDMA1_Channel4_IRQHandler,
    GPDMA1_Channel5_IRQHandler,
    GPDMA1_Channel6_IRQHandler,
    GPDMA1_Channel7_IRQHandler,
    TIM1_BRK_IRQHandler,
    TIM1_UP_IRQHandler,
    TIM1_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    SPI1_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    LPUART1_IRQHandler,
    LPTIM1_IRQHandler,
    LPTIM2_IRQHandler,
    TIM16_IRQHandler,
    TIM17_IRQHandler,
    COMP_IRQHandler,
    I2C3_EV_IRQHandler,
    I2C3_ER_IRQHandler,
    SAI1_IRQHandler,
    TSC_IRQHandler,
    AES_IRQHandler,
    RNG_IRQHandler,
    FPU_IRQHandler,
    HASH_IRQHandler,
    PKA_IRQHandler,
    SPI3_IRQHandler,
    ICACHE_IRQHandler,
    ADC4_IRQHandler,
    RADIO_IRQHandler,
    WKUP_IRQHandler,
    HSEM_IRQHandler,
    HSEM_S_IRQHandler,
    WKUP_S_IRQHandler,
    RCC_AUDIOSYNC_IRQHandler,
    TIM4_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI2_IRQHandler,
    OTG_HS_IRQHandler,
    I2C4_EV_IRQHandler,
    I2C4_ER_IRQHandler,
    USART3_IRQHandler,
    EXTI19_RADIO_IO_IRQHandler,
    EXTI20_RADIO_IO_IRQHandler,
};

/* Guard interrupt ordering against an incompatible CMSIS device header. */
_Static_assert(WWDG_IRQn == 0, "Incorrect STM32WBA65 vector map");
_Static_assert(EXTI20_RADIO_IO_IRQn == 81,
               "Incorrect STM32WBA65 vector map");
