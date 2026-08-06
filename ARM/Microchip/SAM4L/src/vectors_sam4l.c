/**-------------------------------------------------------------------------
@file	Vector_SAM4L.c

@brief	Interrupt Vectors table for ARM Cortex-M4 specific to SAM4L
		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

@author	Hoang Nguyen Hoan
@date	June 30, 2021

@licanse

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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

#include "sam4lxxx.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);
extern void Reset_Handler(void);
extern char Image$$ER_ZI$$Base[];
extern char Image$$ARM_LIB_STACK$$ZI$$Base[];

void DEF_IRQHandler(void) { while(1); }
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void HardFault_Handler(void) { while(1); }
__attribute__((weak, alias("DEF_IRQHandler"))) void MemManage_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BusFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMon_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak/*, alias("DNT_IRQHandler")*/)) void SysTick_Handler(void) {}

__attribute__((weak, alias("DEF_IRQHandler"))) void HFLASHC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_3_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_4_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_5_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_6_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_7_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_8_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_9_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_10_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_11_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_12_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_13_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_14_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDCA_15_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CRCCU_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USBC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PEVC_TR_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PEVC_OV_Handler(void);
#ifdef AESA
__attribute__((weak, alias("DEF_IRQHandler"))) void AESA_Handler(void);
#endif
__attribute__((weak, alias("DEF_IRQHandler"))) void PM_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SCIF_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FREQM_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_3_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_4_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_5_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_6_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_7_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_8_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_9_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_10_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_11_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BPM_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BSCIF_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AST_ALARM_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AST_PER_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AST_OVF_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AST_READY_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AST_CLKREADY_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_3_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_4_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_5_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_6_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_7_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_8_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IISC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC00_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC01_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC02_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC10_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC11_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC12_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWIM0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWIS0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWIM1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWIS1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART3_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADCIFE_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DACC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ACIFC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ABDACB_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TRNG_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PARC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CATB_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWIM2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWIM3_Handler(void);
#ifdef LCDCA
__attribute__((weak, alias("DEF_IRQHandler"))) void LCDCA_Handler(void);
#endif
__attribute__((weak, alias("DEF_IRQHandler"))) void Dummy_Handler(void);


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
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0, 0, 0, 0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,

// External Interrupts
	HFLASHC_Handler,      // 0
	PDCA_0_Handler,       // 1
	PDCA_1_Handler,       // 2
	PDCA_2_Handler,       // 3
	PDCA_3_Handler,       // 4
	PDCA_4_Handler,       // 5
	PDCA_5_Handler,       // 6
	PDCA_6_Handler,       // 7
	PDCA_7_Handler,       // 8
	PDCA_8_Handler,       // 9
	PDCA_9_Handler,       // 10
	PDCA_10_Handler,      // 11
	PDCA_11_Handler,      // 12
	PDCA_12_Handler,      // 13
	PDCA_13_Handler,      // 14
	PDCA_14_Handler,      // 15
	PDCA_15_Handler,      // 16
	CRCCU_Handler,        // 17
	USBC_Handler,         // 18
	PEVC_TR_Handler,      // 19
	PEVC_OV_Handler,      // 20
	#ifdef AESA
	AESA_Handler,         // 21
	#else
	Dummy_Handler,
	#endif
	PM_Handler,           // 22
	SCIF_Handler,         // 23
	FREQM_Handler,        // 24
	GPIO_0_Handler,       // 25
	GPIO_1_Handler,       // 26
	GPIO_2_Handler,       // 27
	GPIO_3_Handler,       // 28
	GPIO_4_Handler,       // 29
	GPIO_5_Handler,       // 30
	GPIO_6_Handler,       // 31
	GPIO_7_Handler,       // 32
	GPIO_8_Handler,       // 33
	GPIO_9_Handler,       // 34
	GPIO_10_Handler,      // 35
	GPIO_11_Handler,      // 36
	BPM_Handler,          // 37
	BSCIF_Handler,        // 38
	AST_ALARM_Handler,    // 39
	AST_PER_Handler,      // 40
	AST_OVF_Handler,      // 41
	AST_READY_Handler,    // 42
	AST_CLKREADY_Handler, // 43
	WDT_Handler,          // 44
	EIC_1_Handler,        // 45
	EIC_2_Handler,        // 46
	EIC_3_Handler,        // 47
	EIC_4_Handler,        // 48
	EIC_5_Handler,        // 49
	EIC_6_Handler,        // 50
	EIC_7_Handler,        // 51
	EIC_8_Handler,        // 52
	IISC_Handler,         // 53
	SPI_Handler,          // 54
	TC00_Handler,         // 55
	TC01_Handler,         // 56
	TC02_Handler,         // 57
	TC10_Handler,         // 58
	TC11_Handler,         // 59
	TC12_Handler,         // 60
	TWIM0_Handler,        // 61
	TWIS0_Handler,        // 62
	TWIM1_Handler,        // 63
	TWIS1_Handler,        // 64
	USART0_Handler,       // 65
	USART1_Handler,       // 66
	USART2_Handler,       // 67
	USART3_Handler,       // 68
	ADCIFE_Handler,       // 69
	DACC_Handler,         // 70
	ACIFC_Handler,        // 71
	ABDACB_Handler,       // 72
	TRNG_Handler,         // 73
	PARC_Handler,         // 74
	CATB_Handler,         // 75
	Dummy_Handler,        // one not used
	TWIM2_Handler,        // 77
	TWIM3_Handler,        // 78
	#ifdef LCDCA
	LCDCA_Handler         // 79
	#else
	Dummy_Handler
	#endif
};

