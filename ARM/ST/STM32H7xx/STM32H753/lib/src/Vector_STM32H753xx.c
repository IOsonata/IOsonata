/**-------------------------------------------------------------------------
@file	Vector_STM32H753xx.c

@brief	Interrupt Vectors table for ARM Cortex-M7 STM32H753xx.

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
#include "stm32h7xx.h"
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
__attribute__((weak, alias("DEF_IRQHandler"))) void PVD_AVD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TAMP_STAMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLASH_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RCC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FDCAN1_IT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FDCAN2_IT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FDCAN1_IT1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FDCAN2_IT1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI9_5_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM1_BRK_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM1_UP_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM1_TRG_COM_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM1_CC_IRQHandler(void);
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
__attribute__((weak, alias("Stm32TimerVector"))) void TIM8_BRK_TIM12_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM8_UP_TIM13_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM8_TRG_COM_TIM14_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM8_CC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FMC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SDMMC1_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART5_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM6_DAC_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ETH_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ETH_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FDCAN_CAL_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C3_EV_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C3_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_HS_EP1_OUT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_HS_EP1_IN_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_HS_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_HS_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DCMI_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CRYP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HASH_RNG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FPU_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART8_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LTDC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LTDC_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2D_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QUADSPI_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTIM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CEC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C4_EV_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C4_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPDIF_RX_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_FS_EP1_OUT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_FS_EP1_IN_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_FS_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_FS_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMAMUX1_OVR_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HRTIM1_Master_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HRTIM1_TIMA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HRTIM1_TIMB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HRTIM1_TIMC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HRTIM1_TIMD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HRTIM1_TIME_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HRTIM1_FLT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DFSDM1_FLT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DFSDM1_FLT1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DFSDM1_FLT2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DFSDM1_FLT3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWPMI1_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM15_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM16_IRQHandler(void);
__attribute__((weak, alias("Stm32TimerVector"))) void TIM17_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MDIOS_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MDIOS_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void JPEG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MDMA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SDMMC2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HSEM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMAMUX2_OVR_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BDMA_Channel0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BDMA_Channel1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BDMA_Channel2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BDMA_Channel3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BDMA_Channel4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BDMA_Channel5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BDMA_Channel6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BDMA_Channel7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void COMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTIM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTIM3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTIM4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTIM5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPUART1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CRS_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAI4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WAKEUP_PIN_IRQHandler(void);

__attribute__((section(".vectors"), used, aligned(1024)))
void (* const __Vectors[])(void) = {
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
	WWDG_IRQHandler,	// IRQ 0
	PVD_AVD_IRQHandler,	// IRQ 1
	TAMP_STAMP_IRQHandler,	// IRQ 2
	RTC_WKUP_IRQHandler,	// IRQ 3
	FLASH_IRQHandler,	// IRQ 4
	RCC_IRQHandler,	// IRQ 5
	EXTI0_IRQHandler,	// IRQ 6
	EXTI1_IRQHandler,	// IRQ 7
	EXTI2_IRQHandler,	// IRQ 8
	EXTI3_IRQHandler,	// IRQ 9
	EXTI4_IRQHandler,	// IRQ 10
	DMA1_Stream0_IRQHandler,	// IRQ 11
	DMA1_Stream1_IRQHandler,	// IRQ 12
	DMA1_Stream2_IRQHandler,	// IRQ 13
	DMA1_Stream3_IRQHandler,	// IRQ 14
	DMA1_Stream4_IRQHandler,	// IRQ 15
	DMA1_Stream5_IRQHandler,	// IRQ 16
	DMA1_Stream6_IRQHandler,	// IRQ 17
	ADC_IRQHandler,	// IRQ 18
	FDCAN1_IT0_IRQHandler,	// IRQ 19
	FDCAN2_IT0_IRQHandler,	// IRQ 20
	FDCAN1_IT1_IRQHandler,	// IRQ 21
	FDCAN2_IT1_IRQHandler,	// IRQ 22
	EXTI9_5_IRQHandler,	// IRQ 23
	TIM1_BRK_IRQHandler,	// IRQ 24
	TIM1_UP_IRQHandler,	// IRQ 25
	TIM1_TRG_COM_IRQHandler,	// IRQ 26
	TIM1_CC_IRQHandler,	// IRQ 27
	TIM2_IRQHandler,	// IRQ 28
	TIM3_IRQHandler,	// IRQ 29
	TIM4_IRQHandler,	// IRQ 30
	I2C1_EV_IRQHandler,	// IRQ 31
	I2C1_ER_IRQHandler,	// IRQ 32
	I2C2_EV_IRQHandler,	// IRQ 33
	I2C2_ER_IRQHandler,	// IRQ 34
	SPI1_IRQHandler,	// IRQ 35
	SPI2_IRQHandler,	// IRQ 36
	USART1_IRQHandler,	// IRQ 37
	USART2_IRQHandler,	// IRQ 38
	USART3_IRQHandler,	// IRQ 39
	EXTI15_10_IRQHandler,	// IRQ 40
	RTC_Alarm_IRQHandler,	// IRQ 41
	0,	// IRQ 42
	TIM8_BRK_TIM12_IRQHandler,	// IRQ 43
	TIM8_UP_TIM13_IRQHandler,	// IRQ 44
	TIM8_TRG_COM_TIM14_IRQHandler,	// IRQ 45
	TIM8_CC_IRQHandler,	// IRQ 46
	DMA1_Stream7_IRQHandler,	// IRQ 47
	FMC_IRQHandler,	// IRQ 48
	SDMMC1_IRQHandler,	// IRQ 49
	TIM5_IRQHandler,	// IRQ 50
	SPI3_IRQHandler,	// IRQ 51
	UART4_IRQHandler,	// IRQ 52
	UART5_IRQHandler,	// IRQ 53
	TIM6_DAC_IRQHandler,	// IRQ 54
	TIM7_IRQHandler,	// IRQ 55
	DMA2_Stream0_IRQHandler,	// IRQ 56
	DMA2_Stream1_IRQHandler,	// IRQ 57
	DMA2_Stream2_IRQHandler,	// IRQ 58
	DMA2_Stream3_IRQHandler,	// IRQ 59
	DMA2_Stream4_IRQHandler,	// IRQ 60
	ETH_IRQHandler,	// IRQ 61
	ETH_WKUP_IRQHandler,	// IRQ 62
	FDCAN_CAL_IRQHandler,	// IRQ 63
	0,	// IRQ 64
	0,	// IRQ 65
	0,	// IRQ 66
	0,	// IRQ 67
	DMA2_Stream5_IRQHandler,	// IRQ 68
	DMA2_Stream6_IRQHandler,	// IRQ 69
	DMA2_Stream7_IRQHandler,	// IRQ 70
	USART6_IRQHandler,	// IRQ 71
	I2C3_EV_IRQHandler,	// IRQ 72
	I2C3_ER_IRQHandler,	// IRQ 73
	OTG_HS_EP1_OUT_IRQHandler,	// IRQ 74
	OTG_HS_EP1_IN_IRQHandler,	// IRQ 75
	OTG_HS_WKUP_IRQHandler,	// IRQ 76
	OTG_HS_IRQHandler,	// IRQ 77
	DCMI_IRQHandler,	// IRQ 78
	CRYP_IRQHandler,	// IRQ 79
	HASH_RNG_IRQHandler,	// IRQ 80
	FPU_IRQHandler,	// IRQ 81
	UART7_IRQHandler,	// IRQ 82
	UART8_IRQHandler,	// IRQ 83
	SPI4_IRQHandler,	// IRQ 84
	SPI5_IRQHandler,	// IRQ 85
	SPI6_IRQHandler,	// IRQ 86
	SAI1_IRQHandler,	// IRQ 87
	LTDC_IRQHandler,	// IRQ 88
	LTDC_ER_IRQHandler,	// IRQ 89
	DMA2D_IRQHandler,	// IRQ 90
	SAI2_IRQHandler,	// IRQ 91
	QUADSPI_IRQHandler,	// IRQ 92
	LPTIM1_IRQHandler,	// IRQ 93
	CEC_IRQHandler,	// IRQ 94
	I2C4_EV_IRQHandler,	// IRQ 95
	I2C4_ER_IRQHandler,	// IRQ 96
	SPDIF_RX_IRQHandler,	// IRQ 97
	OTG_FS_EP1_OUT_IRQHandler,	// IRQ 98
	OTG_FS_EP1_IN_IRQHandler,	// IRQ 99
	OTG_FS_WKUP_IRQHandler,	// IRQ 100
	OTG_FS_IRQHandler,	// IRQ 101
	DMAMUX1_OVR_IRQHandler,	// IRQ 102
	HRTIM1_Master_IRQHandler,	// IRQ 103
	HRTIM1_TIMA_IRQHandler,	// IRQ 104
	HRTIM1_TIMB_IRQHandler,	// IRQ 105
	HRTIM1_TIMC_IRQHandler,	// IRQ 106
	HRTIM1_TIMD_IRQHandler,	// IRQ 107
	HRTIM1_TIME_IRQHandler,	// IRQ 108
	HRTIM1_FLT_IRQHandler,	// IRQ 109
	DFSDM1_FLT0_IRQHandler,	// IRQ 110
	DFSDM1_FLT1_IRQHandler,	// IRQ 111
	DFSDM1_FLT2_IRQHandler,	// IRQ 112
	DFSDM1_FLT3_IRQHandler,	// IRQ 113
	SAI3_IRQHandler,	// IRQ 114
	SWPMI1_IRQHandler,	// IRQ 115
	TIM15_IRQHandler,	// IRQ 116
	TIM16_IRQHandler,	// IRQ 117
	TIM17_IRQHandler,	// IRQ 118
	MDIOS_WKUP_IRQHandler,	// IRQ 119
	MDIOS_IRQHandler,	// IRQ 120
	JPEG_IRQHandler,	// IRQ 121
	MDMA_IRQHandler,	// IRQ 122
	0,	// IRQ 123
	SDMMC2_IRQHandler,	// IRQ 124
	HSEM1_IRQHandler,	// IRQ 125
	0,	// IRQ 126
	ADC3_IRQHandler,	// IRQ 127
	DMAMUX2_OVR_IRQHandler,	// IRQ 128
	BDMA_Channel0_IRQHandler,	// IRQ 129
	BDMA_Channel1_IRQHandler,	// IRQ 130
	BDMA_Channel2_IRQHandler,	// IRQ 131
	BDMA_Channel3_IRQHandler,	// IRQ 132
	BDMA_Channel4_IRQHandler,	// IRQ 133
	BDMA_Channel5_IRQHandler,	// IRQ 134
	BDMA_Channel6_IRQHandler,	// IRQ 135
	BDMA_Channel7_IRQHandler,	// IRQ 136
	COMP_IRQHandler,	// IRQ 137
	LPTIM2_IRQHandler,	// IRQ 138
	LPTIM3_IRQHandler,	// IRQ 139
	LPTIM4_IRQHandler,	// IRQ 140
	LPTIM5_IRQHandler,	// IRQ 141
	LPUART1_IRQHandler,	// IRQ 142
	0,	// IRQ 143
	CRS_IRQHandler,	// IRQ 144
	ECC_IRQHandler,	// IRQ 145
	SAI4_IRQHandler,	// IRQ 146
	0,	// IRQ 147
	0,	// IRQ 148
	WAKEUP_PIN_IRQHandler,	// IRQ 149
};
const uint32_t g_iVectorSize = sizeof(__Vectors);
