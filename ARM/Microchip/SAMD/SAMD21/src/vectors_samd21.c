/**-------------------------------------------------------------------------
@file	vectors_samd21.c

@brief	SAM D Cortex-M0+ interrupt vectors

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
#include "samd21j18a.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);

void DEF_IRQHandler(void)
{
	for (;;)
	{
		__asm volatile("nop");
	}
}

__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HardFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak)) void SysTick_Handler(void) {}

__attribute__((weak, alias("DEF_IRQHandler"))) void PM_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SYSCTRL_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EIC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void NVMCTRL_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMAC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EVSYS_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERCOM0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERCOM1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERCOM2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERCOM3_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERCOM4_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SERCOM5_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TCC0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TCC1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TCC2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC3_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC4_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC5_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC6_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC7_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DAC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PTC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S_Handler(void);

__attribute__((section(".vectors"), used, aligned(256)))
void (* const __Vectors[44])(void) = {
	(void (*)(void))&__StackTop,
	ResetEntry,
	NMI_Handler,
	HardFault_Handler,
	0, 0, 0, 0, 0, 0, 0,
	SVC_Handler,
	0, 0,
	PendSV_Handler,
	SysTick_Handler,
	PM_Handler, // IRQ 0
	SYSCTRL_Handler, // IRQ 1
	WDT_Handler, // IRQ 2
	RTC_Handler, // IRQ 3
	EIC_Handler, // IRQ 4
	NVMCTRL_Handler, // IRQ 5
	DMAC_Handler, // IRQ 6
	USB_Handler, // IRQ 7
	EVSYS_Handler, // IRQ 8
	SERCOM0_Handler, // IRQ 9
	SERCOM1_Handler, // IRQ 10
	SERCOM2_Handler, // IRQ 11
	SERCOM3_Handler, // IRQ 12
	SERCOM4_Handler, // IRQ 13
	SERCOM5_Handler, // IRQ 14
	TCC0_Handler, // IRQ 15
	TCC1_Handler, // IRQ 16
	TCC2_Handler, // IRQ 17
	TC3_Handler, // IRQ 18
	TC4_Handler, // IRQ 19
	TC5_Handler, // IRQ 20
	TC6_Handler, // IRQ 21
	TC7_Handler, // IRQ 22
	ADC_Handler, // IRQ 23
	AC_Handler, // IRQ 24
	DAC_Handler, // IRQ 25
	PTC_Handler, // IRQ 26
	I2S_Handler, // IRQ 27
};

_Static_assert(sizeof(__Vectors) == 44U * sizeof(void (*)(void)),
	"SAMD21 vector table must contain 16 core and 28 external entries");

_Static_assert(PERIPH_MAX_IRQn + 17 == 44,
	"Vector table does not match CMSIS interrupt numbering");
