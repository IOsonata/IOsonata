/**-------------------------------------------------------------------------
@file	vector_nf54l15.c

@brief	Interrupt Vectors table for ARM Cortex-M33 specific to nRF54L15.

CMSIS & GCC compiler
linker section name .Vectors is used for the table

@author	Nguyen Hoan Hoang
@date	Nov. 24, 2023

@license

MIT License

Copyright (c) 2023 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Softare.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include "nrf.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);
extern void Reset_Handler(void);
extern char Image$$ER_ZI$$Base[];
extern char Image$$ARM_LIB_STACK$$ZI$$Base[];

void DEF_IRQHandler(void) { while(1); }

__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void HardFault_Handler(void) { while(1); }
__attribute__((weak, alias("DEF_IRQHandler"))) void MemoryManagement_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BusFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SecureFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMon_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void SysTick_Handler(void) {}

__attribute__((weak, alias("DEF_IRQHandler"))) void SWI00_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI01_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI02_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI03_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPU00_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MPC00_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AAR00_CCM00_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECB00_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CRACEN_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL00_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RRAMC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void VPR00_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CTRLAP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CM33SS_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER00_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPU10_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER10_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void RTC10_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU10_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RADIO_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RADIO_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPU20_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL20_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL21_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL22_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU20_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER20_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER21_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER22_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER23_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER24_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDM20_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDM21_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM20_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM21_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM22_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAADC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void NFCT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TEMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE20_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE20_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TAMPC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S20_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QDEC20_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QDEC21_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GRTC_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GRTC_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GRTC_2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GRTC_3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPU30_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL30_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CLOCK_POWER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void COMP_LPCOMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT30_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT31_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE30_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE30_1_IRQHandler(void);

/**
 * This interrupt vector is by default located in FLASH. Though it can not be
 * changed at runtime. All functions in the vector are weak.  it can be
 * overloaded by application function
 *
 */
#ifdef __ICCARM__
__attribute__ ((section(".intvec"), used))
void (* const __vector_table[])(void) = {
#else
__attribute__ ((section(".vectors"), used))
void (* const __Vectors[])(void) = {
#endif
#if defined ( __ARMCC_VERSION )
	(void (*)(void) )((uint32_t)0x20000000 + 0x10000),
	Reset_Handler,
#else
	(void (*)(void) )((uint32_t)&__StackTop),
	ResetEntry,
#endif
    NMI_Handler,
    HardFault_Handler,
    MemoryManagement_Handler,
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
/* Device specific interrupt handlers */
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SWI00_IRQHandler,
    SWI01_IRQHandler,
    SWI02_IRQHandler,
    SWI03_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SPU00_IRQHandler,
    MPC00_IRQHandler,
    0,
    0,
    0,
    0,
    AAR00_CCM00_IRQHandler,
    ECB00_IRQHandler,
    CRACEN_IRQHandler,
    0,
    SERIAL00_IRQHandler,
    RRAMC_IRQHandler,
    VPR00_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    CTRLAP_IRQHandler,
    CM33SS_IRQHandler,
    0,
    TIMER00_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SPU10_IRQHandler,
    0,
    0,
    0,
    0,
    TIMER10_IRQHandler,
    RTC10_IRQHandler,
    EGU10_IRQHandler,
    0,
    0,
    RADIO_0_IRQHandler,
    RADIO_1_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SPU20_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    SERIAL20_IRQHandler,
    SERIAL21_IRQHandler,
    SERIAL22_IRQHandler,
    EGU20_IRQHandler,
    TIMER20_IRQHandler,
    TIMER21_IRQHandler,
    TIMER22_IRQHandler,
    TIMER23_IRQHandler,
    TIMER24_IRQHandler,
    0,
    PDM20_IRQHandler,
    PDM21_IRQHandler,
    PWM20_IRQHandler,
    PWM21_IRQHandler,
    PWM22_IRQHandler,
    SAADC_IRQHandler,
    NFCT_IRQHandler,
    TEMP_IRQHandler,
    0,
    0,
    GPIOTE20_0_IRQHandler,
    GPIOTE20_1_IRQHandler,
    TAMPC_IRQHandler,
    I2S20_IRQHandler,
    0,
    0,
    QDEC20_IRQHandler,
    QDEC21_IRQHandler,
    GRTC_0_IRQHandler,
    GRTC_1_IRQHandler,
    GRTC_2_IRQHandler,
    GRTC_3_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SPU30_IRQHandler,
    0,
    0,
    0,
    SERIAL30_IRQHandler,
    CLOCK_POWER_IRQHandler,
    COMP_LPCOMP_IRQHandler,
    0,
    WDT30_IRQHandler,
    WDT31_IRQHandler,
    0,
    0,
    GPIOTE30_0_IRQHandler,
    GPIOTE30_1_IRQHandler,
};

