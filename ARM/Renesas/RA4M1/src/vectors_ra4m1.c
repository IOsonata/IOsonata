/**-------------------------------------------------------------------------
@file	vectors_ra4m1.c

@brief	RA4M1 Cortex-M4 exception vectors and boot option words.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include "ra4m1xxx.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);

void DEF_IRQHandler(void)
{
	for (;;) { __NOP(); }
}

__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HardFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MemManage_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BusFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMon_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);

/* Keep IOsonata's overridable SysTick default; startup does not start it. */
__attribute__((weak)) void SysTick_Handler(void) {}

/* Defined by interrupt_ra4m1.cpp, as in the RE01 port. Keep these references
 * strong so static-library extraction cannot select local weak trap aliases.
 */
extern void IEL0_IRQHandler(void);
extern void IEL1_IRQHandler(void);
extern void IEL2_IRQHandler(void);
extern void IEL3_IRQHandler(void);
extern void IEL4_IRQHandler(void);
extern void IEL5_IRQHandler(void);
extern void IEL6_IRQHandler(void);
extern void IEL7_IRQHandler(void);
extern void IEL8_IRQHandler(void);
extern void IEL9_IRQHandler(void);
extern void IEL10_IRQHandler(void);
extern void IEL11_IRQHandler(void);
extern void IEL12_IRQHandler(void);
extern void IEL13_IRQHandler(void);
extern void IEL14_IRQHandler(void);
extern void IEL15_IRQHandler(void);
extern void IEL16_IRQHandler(void);
extern void IEL17_IRQHandler(void);
extern void IEL18_IRQHandler(void);
extern void IEL19_IRQHandler(void);
extern void IEL20_IRQHandler(void);
extern void IEL21_IRQHandler(void);
extern void IEL22_IRQHandler(void);
extern void IEL23_IRQHandler(void);
extern void IEL24_IRQHandler(void);
extern void IEL25_IRQHandler(void);
extern void IEL26_IRQHandler(void);
extern void IEL27_IRQHandler(void);
extern void IEL28_IRQHandler(void);
extern void IEL29_IRQHandler(void);
extern void IEL30_IRQHandler(void);
extern void IEL31_IRQHandler(void);

__attribute__((section(".vectors"), used, aligned(256)))
void (* const __Vectors[RA4M1_VECTOR_COUNT])(void) = {
	(void (*)(void))&__StackTop,
	ResetEntry,
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

/* Normal boot options: OFS0, OFS1, then the security-MPU option area.
 * Do not emit access-window or debugger-ID configuration at 0x01010000.
 * The linker reserves all bytes through 0x43B, including reserved padding.
 */
__attribute__((section(".option_setting"), used, aligned(4)))
const uint32_t g_Ra4m1OptionSetting[15] = {
	RA4M1_OFS0_VALUE,
	RA4M1_OFS1_VALUE,
	0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
	0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
	0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
	0xFFFFFFFFUL
};

_Static_assert(sizeof(__Vectors) == 48U * sizeof(void (*)(void)),
	"RA4M1 requires 16 core vectors and 32 ICU event-link vectors");
_Static_assert(sizeof(g_Ra4m1OptionSetting) == 0x3CU,
	"RA4M1 option-setting area must cover 0x400..0x43B");
