/**-------------------------------------------------------------------------
@file	Vector_nRF52805.c

@brief	Interrupt Vectors table for nRF52805

		CMSIS & GCC compiler
		linker section name .Vectors is used for the table

@author	Hoang Nguyen Hoan
@date	Oct. 9, 2023

@lincense

MIT

Copyright (c) 2023, I-SYST inc., all rights reserved

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
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMonitor_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void POWER_CLOCK_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RADIO_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UARTE0_UART0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWIM0_TWIS0_TWI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM0_SPIS0_SPI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAADC_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER0_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER1_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void TIMER2_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void RTC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TEMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RNG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CCM_AAR_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void RTC1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QDEC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI0_EGU0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI1_EGU1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI5_IRQHandler(void);

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
void (* const __Vectors[100])(void) = {
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
	0, 0, 0, 0,
	SVC_Handler,
	DebugMonitor_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,

/* External Interrupts */
    POWER_CLOCK_IRQHandler,
    RADIO_IRQHandler,
	UARTE0_UART0_IRQHandler,
	TWIM0_TWIS0_TWI0_IRQHandler,
	SPIM0_SPIS0_SPI0_IRQHandler,
	0,
    GPIOTE_IRQHandler,
	SAADC_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    RTC0_IRQHandler,
    TEMP_IRQHandler,
    RNG_IRQHandler,
    ECB_IRQHandler,
    CCM_AAR_IRQHandler,
    WDT_IRQHandler,
    RTC1_IRQHandler,
    QDEC_IRQHandler,
	0,
	SWI0_EGU0_IRQHandler,
	SWI1_EGU1_IRQHandler,
	SWI2_IRQHandler,
	SWI3_IRQHandler,
	SWI4_IRQHandler,
	SWI5_IRQHandler,
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
	0
};
