/**-------------------------------------------------------------------------
@file	vectors_samd11.c

@brief	SAMD11D14A Cortex-M0+ interrupt vectors, using IOsonata ResetEntry.

@license
MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>

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
__attribute__((weak, alias("DEF_IRQHandler"))) void TCC0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DAC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PTC_Handler(void);

__attribute__((section(".vectors"), used, aligned(256)))
void (* const __Vectors[35])(void) = {
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
	TCC0_Handler, // IRQ 12
	TC1_Handler, // IRQ 13
	TC2_Handler, // IRQ 14
	ADC_Handler, // IRQ 15
	AC_Handler, // IRQ 16
	DAC_Handler, // IRQ 17
	PTC_Handler, // IRQ 18
};

_Static_assert(sizeof(__Vectors) == 35U * sizeof(void (*)(void)),
	"SAMD11 vector table must contain 16 core and 19 external entries");
