/*--------------------------------------------------------------------------
File   : vector17_test.c

Desc   : LPC17xx vector positions checked against vendor CMSIS IRQ numbers

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdio.h>
#include "LPC17xx.h"

extern void (* const g_Vectors[])(void);
extern void (* const __Vectors[])(void);
unsigned long __StackTop;
static volatile int s_LastIrq;

void ResetEntry(void)
{
}

// Distinct strong handlers verify both slot order and weak-symbol override.
void WDT_IRQHandler(void)
{
	s_LastIrq = WDT_IRQn;
}

void TIMER0_IRQHandler(void)
{
	s_LastIrq = TIMER0_IRQn;
}

void TIMER1_IRQHandler(void)
{
	s_LastIrq = TIMER1_IRQn;
}

void TIMER2_IRQHandler(void)
{
	s_LastIrq = TIMER2_IRQn;
}

void TIMER3_IRQHandler(void)
{
	s_LastIrq = TIMER3_IRQn;
}

void UART0_IRQHandler(void)
{
	s_LastIrq = UART0_IRQn;
}

void UART1_IRQHandler(void)
{
	s_LastIrq = UART1_IRQn;
}

void UART2_IRQHandler(void)
{
	s_LastIrq = UART2_IRQn;
}

void UART3_IRQHandler(void)
{
	s_LastIrq = UART3_IRQn;
}

void PWM_IRQHandler(void)
{
	s_LastIrq = PWM1_IRQn;
}

void I2C0_IRQHandler(void)
{
	s_LastIrq = I2C0_IRQn;
}

void I2C1_IRQHandler(void)
{
	s_LastIrq = I2C1_IRQn;
}

void I2C2_IRQHandler(void)
{
	s_LastIrq = I2C2_IRQn;
}

void SPI_IRQHandler(void)
{
	s_LastIrq = SPI_IRQn;
}

void SSP0_IRQHandler(void)
{
	s_LastIrq = SSP0_IRQn;
}

void SSP1_IRQHandler(void)
{
	s_LastIrq = SSP1_IRQn;
}

void PLL0_IRQHandler(void)
{
	s_LastIrq = PLL0_IRQn;
}

void RTC_IRQHandler(void)
{
	s_LastIrq = RTC_IRQn;
}

void EINT0_IRQHandler(void)
{
	s_LastIrq = EINT0_IRQn;
}

void EINT1_IRQHandler(void)
{
	s_LastIrq = EINT1_IRQn;
}

void EINT2_IRQHandler(void)
{
	s_LastIrq = EINT2_IRQn;
}

void EINT3_IRQHandler(void)
{
	s_LastIrq = EINT3_IRQn;
}

void ADC_IRQHandler(void)
{
	s_LastIrq = ADC_IRQn;
}

void BOD_IRQHandler(void)
{
	s_LastIrq = BOD_IRQn;
}

void USB_IRQHandler(void)
{
	s_LastIrq = USB_IRQn;
}

void CAN_IRQHandler(void)
{
	s_LastIrq = CAN_IRQn;
}

void DMA_IRQHandler(void)
{
	s_LastIrq = DMA_IRQn;
}

void I2S_IRQHandler(void)
{
	s_LastIrq = I2S_IRQn;
}

void ENET_IRQHandler(void)
{
	s_LastIrq = ENET_IRQn;
}

void RIT_IRQHandler(void)
{
	s_LastIrq = RIT_IRQn;
}

void MCPWM_IRQHandler(void)
{
	s_LastIrq = MCPWM_IRQn;
}

void QEI_IRQHandler(void)
{
	s_LastIrq = QEI_IRQn;
}

void PLL1_IRQHandler(void)
{
	s_LastIrq = PLL1_IRQn;
}

void USBActivity_IRQHandler(void)
{
	s_LastIrq = USBActivity_IRQn;
}

void CANActivity_IRQHandler(void)
{
	s_LastIrq = CANActivity_IRQn;
}

int main(void)
{
	assert(&__Vectors[0] == &g_Vectors[0]);
	assert(g_Vectors[0] == (void (*)(void))&__StackTop);
	assert(g_Vectors[1] == ResetEntry);
	for (int irq = 0; irq <= CANActivity_IRQn; irq++)
	{
		s_LastIrq = -1;
		g_Vectors[16 + irq]();
		assert(s_LastIrq == irq);
	}
	puts("LPC17xx CMSIS vector positions and handler overrides: PASS");
	return 0;
}
