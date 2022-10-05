/**-------------------------------------------------------------------------
@file	vectors_re01.c

@brief	Interrupt Vectors table for ARM Cortex-M0+ specific to Renesas RE01
		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

@author	Hoang Nguyen Hoan
@date	Nov. 10, 2021

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

#include "RE01xxx.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);
extern void Reset_Handler(void);
extern char Image$$ER_ZI$$Base[];
extern char Image$$ARM_LIB_STACK$$ZI$$Base[];

void DEF_IRQHandler(void) { while(1); }
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void HardFault_Handler(void) { while(1); }
//__attribute__((weak, alias("DEF_IRQHandler"))) void MemManage_Handler(void);
//__attribute__((weak, alias("DEF_IRQHandler"))) void BusFault_Handler(void);
//__attribute__((weak, alias("DEF_IRQHandler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
//__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMon_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);

// These handlers are implemented with new interrupt registration scheme in
// interrupt_re01.cpp
//
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL0_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL1_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL2_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL3_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL4_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL5_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL6_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL7_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL8_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL9_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL10_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL11_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL12_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL13_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL14_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL15_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL16_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL17_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL18_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL19_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL20_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL21_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL22_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL23_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL24_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL25_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL26_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL27_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL28_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL29_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL30_IRQHandler(void);
/*__attribute__((weak, alias("DEF_IRQHandler"))) */void IEL31_IRQHandler(void);


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
	0,
	0,
	0,
	0, 0, 0, 0,
	SVC_Handler,
	0,
	0,
	PendSV_Handler,
	SysTick_Handler,

// External Interrupts
	IEL0_IRQHandler,
	IEL1_IRQHandler,
	IEL2_IRQHandler,
	IEL3_IRQHandler,
	IEL4_IRQHandler,
	IEL5_IRQHandler,
	IEL6_IRQHandler,
	IEL7_IRQHandler,
	IEL8_IRQHandler,
	IEL9_IRQHandler,
	IEL10_IRQHandler,
	IEL11_IRQHandler,
	IEL12_IRQHandler,
	IEL13_IRQHandler,
	IEL14_IRQHandler,
	IEL15_IRQHandler,
	IEL16_IRQHandler,
	IEL17_IRQHandler,
	IEL18_IRQHandler,
	IEL19_IRQHandler,
	IEL20_IRQHandler,
	IEL21_IRQHandler,
	IEL22_IRQHandler,
	IEL23_IRQHandler,
	IEL24_IRQHandler,
	IEL25_IRQHandler,
	IEL26_IRQHandler,
	IEL27_IRQHandler,
	IEL28_IRQHandler,
	IEL29_IRQHandler,
	IEL30_IRQHandler,
	IEL31_IRQHandler,
};

