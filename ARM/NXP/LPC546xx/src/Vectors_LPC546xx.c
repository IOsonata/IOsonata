/**-------------------------------------------------------------------------
@file	Vector_LPC546xx.c

@brief	Interrupt vector for the LPC546xx

Interrupt Vectors table for LPC546xx
		 CMSIS & GCC compiler
		 linker section name .vectors is used for the table

@author	Hoang Nguyen Hoan
@date	July 10, 2020

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

extern unsigned long __StackTop;
extern void ResetEntry(void);

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

__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_BOD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GINT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GINT1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIN_INT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIN_INT1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIN_INT2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIN_INT3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UTICK0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MRT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CTIMER0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CTIMER1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SCT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CTIMER3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC0_SEQA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC0_SEQB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC0_THCMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMIC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HWVAD0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB0_NEEDCLK_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIN_INT4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIN_INT5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIN_INT6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIN_INT7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CTIMER2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CTIMER4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RIT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIFI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM8_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEXCOMM9_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SDIO_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN0_IRQ0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN0_IRQ1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN1_IRQ0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN1_IRQ1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB1_NEEDCLK_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ETHERNET_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ETHERNET_PMT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ETHERNET_MACLP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EEPROM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LCD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SHA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SMARTCARD0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SMARTCARD1_IRQHandler(void);

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
	0,
	(void (*) )-1,		// Checksum value
	0, 0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,

/* External Interrupts */
	WDT_BOD_IRQHandler,					// Windowed watchdog timer, Brownout detect
	DMA0_IRQHandler,             		// DMA controller
	GINT0_IRQHandler,             		// GPIO group 0
	GINT1_IRQHandler,             		// GPIO group 1
	PIN_INT0_IRQHandler,             	// Pin interrupt 0 or pattern match engine slice 0
	PIN_INT1_IRQHandler,             	// Pin interrupt 1 or pattern match engine slice 1
	PIN_INT2_IRQHandler,             	// Pin interrupt 2 or pattern match engine slice 2
	PIN_INT3_IRQHandler,             	// Pin interrupt 3 or pattern match engine slice 3
	UTICK0_IRQHandler,                 	// Micro-tick Timer
	MRT0_IRQHandler,					// Multi-rate timer
	CTIMER0_IRQHandler,					// Standard counter/timer CTIMER0
	CTIMER1_IRQHandler,					// Standard counter/timer CTIMER0
	SCT0_IRQHandler,					// SCTimer/PWM
	CTIMER3_IRQHandler,					// Standard counter/timer CTIMER3
	FLEXCOMM0_IRQHandler,				// Flexcomm Interface 0 (USART, SPI, I2C, FLEXCOMM)
	FLEXCOMM1_IRQHandler,				// Flexcomm Interface 1 (USART, SPI, I2C, FLEXCOMM)
	FLEXCOMM2_IRQHandler,             	// Flexcomm Interface 2 (USART, SPI, I2C, FLEXCOMM)
	FLEXCOMM3_IRQHandler,             	// Flexcomm Interface 3 (USART, SPI, I2C, FLEXCOMM)
	FLEXCOMM4_IRQHandler,             	// Flexcomm Interface 4 (USART, SPI, I2C, FLEXCOMM)
	FLEXCOMM5_IRQHandler,               // Flexcomm Interface 5 (USART, SPI, I2C, FLEXCOMM)
	FLEXCOMM6_IRQHandler,               // Flexcomm Interface 6 (USART, SPI, I2C, FLEXCOMM)
	FLEXCOMM7_IRQHandler,               // Flexcomm Interface 7 (USART, SPI, I2C, FLEXCOMM)
	ADC0_SEQA_IRQHandler,               // ADC0 sequence A completion.
	ADC0_SEQB_IRQHandler,               // ADC0 sequence B completion.
	ADC0_THCMP_IRQHandler,				// ADC0 threshold compare and error.
	DMIC0_IRQHandler,					// Digital microphone and DMIC subsystem
	HWVAD0_IRQHandler,					// Hardware Voice Activity Detector
	USB0_NEEDCLK_IRQHandler,			// USB Activity Wake-up Interrupt
	USB0_IRQHandler,					// USB device
	RTC_IRQHandler,						// RTC alarm and wake-up interrupts
	0,
	0,
	PIN_INT4_IRQHandler,				// Pin interrupt 4 or pattern match engine slice 4 int
	PIN_INT5_IRQHandler,				// Pin interrupt 5 or pattern match engine slice 5 int
	PIN_INT6_IRQHandler,				// Pin interrupt 6 or pattern match engine slice 6 int
	PIN_INT7_IRQHandler,				// Pin interrupt 7 or pattern match engine slice 7 int
	CTIMER2_IRQHandler,					// Standard counter/timer CTIMER2
	CTIMER4_IRQHandler,					// Standard counter/timer CTIMER4
	RIT_IRQHandler,						// Repetitive Interrupt Timer
	SPIFI0_IRQHandler,					// SPI flash interface */
	FLEXCOMM8_IRQHandler,				// Flexcomm Interface 8 (USART, SPI, I2C, FLEXCOMM)
	FLEXCOMM9_IRQHandler,				// Flexcomm Interface 9 (USART, SPI, I2C, FLEXCOMM)
	SDIO_IRQHandler,					// SD/MMC
	CAN0_IRQ0_IRQHandler,				// CAN0 interrupt0
	CAN0_IRQ1_IRQHandler,				// CAN0 interrupt1
	CAN1_IRQ0_IRQHandler,				// CAN1 interrupt0
	CAN1_IRQ1_IRQHandler,				// CAN1 interrupt1
	USB1_IRQHandler,					// USB1 interrupt
	USB1_NEEDCLK_IRQHandler,			// USB1 activity
	ETHERNET_IRQHandler,				// Ethernet
	ETHERNET_PMT_IRQHandler,			// Ethernet power management interrupt
	ETHERNET_MACLP_IRQHandler,			// Ethernet MAC interrupt
	EEPROM_IRQHandler,					// EEPROM interrupt
	LCD_IRQHandler,						// LCD interrupt
	SHA_IRQHandler,						// SHA interrupt
	SMARTCARD0_IRQHandler,				// Smart card 0 interrupt
	SMARTCARD1_IRQHandler,				// Smart card 1 interrupt
};

#ifdef __ICCARM__
const uint32_t g_VectorSize = sizeof(__vector_table) + 4;
#else
const uint32_t g_VectorSize = sizeof(__Vectors) + 4;
#endif
