/**-------------------------------------------------------------------------
@file	Vector_STM32L152RExx.c

@brief	Interrupt Vectors table for ARM Cortex-M7 STM32L152RExx.

		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

@license

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include "stm32l1xx.h"
extern unsigned long __StackTop;
extern void ResetEntry(void);
void DEF_IRQHandler(void)
{
	while (1)
	{
	}
}
__attribute__((weak)) void Stm32TimerIRQHandler(int IrqNo);
void Stm32TimerVector(void)
{
	if (Stm32TimerIRQHandler)
	{
		Stm32TimerIRQHandler((int)(__get_IPSR() & 0x1FFU) - 16);
	}
	else
	{
		DEF_IRQHandler();
	}
}
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HardFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MemManage_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BusFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMon_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WWDG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PVD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TAMPER_STAMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLASH_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RCC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB_HP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB_LP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DAC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void COMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI9_5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LCD_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM9_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM10_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM11_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM2_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM3_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C1_EV_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C1_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C2_EV_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C2_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI15_10_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_Alarm_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB_FS_WKUP_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM6_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM7_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void COMP_ACQ_IRQHandler(void);

__attribute__((used, section(".vectors"), aligned(512)))
void (* const __Vectors[])(void) = {
	(void (*)(void))&__StackTop,
	ResetEntry,
	NMI_Handler,
	HardFault_Handler,
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0,
	0,
	0,
	0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,
	WWDG_IRQHandler,
	PVD_IRQHandler,
	TAMPER_STAMP_IRQHandler,
	RTC_WKUP_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_IRQHandler,
	EXTI1_IRQHandler,
	EXTI2_IRQHandler,
	EXTI3_IRQHandler,
	EXTI4_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_IRQHandler,
	DMA1_Channel3_IRQHandler,
	DMA1_Channel4_IRQHandler,
	DMA1_Channel5_IRQHandler,
	DMA1_Channel6_IRQHandler,
	DMA1_Channel7_IRQHandler,
	ADC1_IRQHandler,
	USB_HP_IRQHandler,
	USB_LP_IRQHandler,
	DAC_IRQHandler,
	COMP_IRQHandler,
	EXTI9_5_IRQHandler,
	LCD_IRQHandler,
	TIM9_IRQHandler,
	TIM10_IRQHandler,
	TIM11_IRQHandler,
	TIM2_IRQHandler,
	TIM3_IRQHandler,
	TIM4_IRQHandler,
	I2C1_EV_IRQHandler,
	I2C1_ER_IRQHandler,
	I2C2_EV_IRQHandler,
	I2C2_ER_IRQHandler,
	SPI1_IRQHandler,
	SPI2_IRQHandler,
	USART1_IRQHandler,
	USART2_IRQHandler,
	USART3_IRQHandler,
	EXTI15_10_IRQHandler,
	RTC_Alarm_IRQHandler,
	USB_FS_WKUP_IRQHandler,
	TIM6_IRQHandler,
	TIM7_IRQHandler,
	0,
	TIM5_IRQHandler,
	SPI3_IRQHandler,
	UART4_IRQHandler,
	UART5_IRQHandler,
	DMA2_Channel1_IRQHandler,
	DMA2_Channel2_IRQHandler,
	DMA2_Channel3_IRQHandler,
	DMA2_Channel4_IRQHandler,
	DMA2_Channel5_IRQHandler,
	0,
	COMP_ACQ_IRQHandler,
};
