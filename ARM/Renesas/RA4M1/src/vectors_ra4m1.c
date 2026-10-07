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

/* Strong definitions supplied by a future RA4M1 interrupt manager or an
 * application replace these defaults without changing the vector table.
 */
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL8_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL9_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL10_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL11_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL12_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL13_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL14_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL15_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL16_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL17_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL18_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL19_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL20_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL21_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL22_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL23_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL24_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL25_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL26_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL27_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL28_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL29_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL30_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IEL31_IRQHandler(void);

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
