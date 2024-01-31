/**-------------------------------------------------------------------------
@file	Vector_nRF5340_Net.c

@brief	Interrupt Vectors table for ARM Cortex-M33 specific to nRF5340 Net.

CMSIS & GCC compiler
linker section name .Vectors is used for the table


@author	Hoang Nguyen Hoan
@date	Mar. 17, 2020

@license

Copyright (c) 2020, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include "nrf.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);

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
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CLOCK_POWER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RADIO_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RNG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_IRQHandler(void);
__attribute__((weak, /*alias("DEF_IRQHandler")*/)) void TIMER0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AAR_CCM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TEMP_IRQHandler(void);
__attribute__((weak, /*alias("DEF_IRQHandler")*/)) void RTC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IPC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU0_IRQHandler(void);
__attribute__((weak, /*alias("DEF_IRQHandler")*/)) void RTC1_IRQHandler(void);
__attribute__((weak, /*alias("DEF_IRQHandler")*/)) void TIMER1_IRQHandler(void);
__attribute__((weak, /*alias("DEF_IRQHandler")*/)) void TIMER2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI3_IRQHandler(void);


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
	SecureFault_Handler,
	0, 0, 0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,

/* External Interrupts */
	0, 0, 0, 0,	0,
	CLOCK_POWER_IRQHandler,
	0, 0,
	RADIO_IRQHandler,
	RNG_IRQHandler,
	GPIOTE_IRQHandler,
	WDT_IRQHandler,
	TIMER0_IRQHandler,
	ECB_IRQHandler,
	AAR_CCM_IRQHandler,
	0,
	TEMP_IRQHandler,
	RTC0_IRQHandler,
	IPC_IRQHandler,
	SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQHandler,
	EGU0_IRQHandler,
	0,
	RTC1_IRQHandler,
	0,
	TIMER1_IRQHandler,
	TIMER2_IRQHandler,
	SWI0_IRQHandler,
	SWI1_IRQHandler,
	SWI2_IRQHandler,
	SWI3_IRQHandler,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

