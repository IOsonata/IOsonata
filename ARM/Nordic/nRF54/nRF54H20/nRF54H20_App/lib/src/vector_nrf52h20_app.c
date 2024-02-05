/**-------------------------------------------------------------------------
@file	vector_nf54h20_app.c

@brief	Interrupt Vectors table for ARM Cortex-M33 specific to nRF54H20 App core.

CMSIS & GCC compiler
linker section name .Vectors is used for the table

@author	Nguyen Hoan Hoang
@date	Mov. 22, 2023

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
__attribute__((weak/*, alias("DNT_IRQHandler")*/)) void SysTick_Handler(void) {}

__attribute__((weak, alias("DEF_IRQHandler"))) void SPU000_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MPC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CPUC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MVDMA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPU010_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT010_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT011_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IPCT_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IPCT_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BELLBOARD_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BELLBOARD_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BELLBOARD_2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BELLBOARD_3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE130_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE130_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GRTC_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GRTC_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GRTC_2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TBM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USBHS_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MRAMC110_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MRAMC111_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXMIF_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTPC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void VPR120_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IPCT120_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I3C120_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void VPR121_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN120_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MVDMA120_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I3C121_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER120_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER121_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM120_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIS120_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM120_UARTE120_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM121_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void VPR130_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IPCT130_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC130_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC131_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT131_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT132_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU130_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RESETHUB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAADC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void COMP_LPCOMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TEMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void NFCT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TDM130_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QDEC130_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QDEC131_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SIMIF130_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TDM131_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER130_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER131_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM130_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER132_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER133_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM131_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER134_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER135_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM132_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER136_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER137_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM133_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERIAL7_IRQHandler(void);


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
void (* const __Vectors[500])(void) = {
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
	SPU000_IRQHandler,
	MPC_IRQHandler,
	CPUC_IRQHandler,
	MVDMA_IRQHandler,
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
	SPU010_IRQHandler,
	0, 0, 0,
	WDT010_IRQHandler,
	WDT011_IRQHandler,
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
	IPCT_0_IRQHandler,
	IPCT_1_IRQHandler,
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
	SWI0_IRQHandler,
	SWI1_IRQHandler,
	SWI2_IRQHandler,
	SWI3_IRQHandler,
	SWI4_IRQHandler,
	SWI5_IRQHandler,
	SWI6_IRQHandler,
	SWI7_IRQHandler,
	BELLBOARD_0_IRQHandler,
	BELLBOARD_1_IRQHandler,
	BELLBOARD_2_IRQHandler,
	BELLBOARD_3_IRQHandler,
	0, 0, 0, 0,
	GPIOTE130_0_IRQHandler,
	GPIOTE130_1_IRQHandler,
	0, 0,
	GRTC_0_IRQHandler,
	GRTC_1_IRQHandler,
	GRTC_2_IRQHandler,
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
	TBM_IRQHandler,
	0, 0, 0, 0, 0, 0,
	USBHS_IRQHandler,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	MRAMC110_IRQHandler,
	MRAMC111_IRQHandler,
	0,
	EXMIF_IRQHandler,
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
    OTPC_IRQHandler,
    0,
    0,
    VPR120_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    IPCT120_0_IRQHandler,
    0,
    I3C120_IRQHandler,
    VPR121_IRQHandler,
    0,
    0,
    0,
    CAN120_IRQHandler,
    MVDMA120_IRQHandler,
    0,
    0,
    0,
    0,
    I3C121_IRQHandler,
    0,
    0,
    0,
    TIMER120_IRQHandler,
    TIMER121_IRQHandler,
    PWM120_IRQHandler,
    SPIS120_IRQHandler,
    SPIM120_UARTE120_IRQHandler,
    SPIM121_IRQHandler,
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
    VPR130_IRQHandler,
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
    IPCT130_0_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    RTC130_IRQHandler,
    RTC131_IRQHandler,
    0,
    WDT131_IRQHandler,
    WDT132_IRQHandler,
    EGU130_IRQHandler,
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
    RESETHUB_IRQHandler,
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
    0,
    0,
    0,
    0,
    SAADC_IRQHandler,
    COMP_LPCOMP_IRQHandler,
    TEMP_IRQHandler,
    NFCT_IRQHandler,
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
    TDM130_IRQHandler,
    PDM_IRQHandler,
    QDEC130_IRQHandler,
    QDEC131_IRQHandler,
    SIMIF130_IRQHandler,
    TDM131_IRQHandler,
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
    TIMER130_IRQHandler,
    TIMER131_IRQHandler,
    PWM130_IRQHandler,
    SERIAL0_IRQHandler,
    SERIAL1_IRQHandler,
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
    TIMER132_IRQHandler,
    TIMER133_IRQHandler,
    PWM131_IRQHandler,
    SERIAL2_IRQHandler,
    SERIAL3_IRQHandler,
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
    TIMER134_IRQHandler,
    TIMER135_IRQHandler,
    PWM132_IRQHandler,
    SERIAL4_IRQHandler,
    SERIAL5_IRQHandler,
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
    TIMER136_IRQHandler,
    TIMER137_IRQHandler,
    PWM133_IRQHandler,
    SERIAL6_IRQHandler,
    SERIAL7_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};
