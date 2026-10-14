/* LPC11Uxx pin interrupt host checks. Copyright (c) 2026 I-SYST inc.
 * MIT License. Models NVIC state and the PINT set/clear aliases at NVIC calls.
 */
#ifndef LPC_GPIO_IRQ_MODEL_H
#define LPC_GPIO_IRQ_MODEL_H
#include <assert.h>
#include "LPC11Uxx.h"

static unsigned s_Enabled;
static unsigned s_Pending;
static unsigned s_Priority[8];
static unsigned s_CallbackCount;
static int s_LastChannel;
static void *s_LastContext;
static bool s_Reedge;
static bool s_Release;

void NVIC_DisableIRQ(IRQn_Type Irq)
{
	assert((unsigned)Irq < 8U);
	s_Enabled &= ~(1U << Irq);
}

void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	unsigned mask = 1U << Irq;
	assert((LPC_SYSCON->SYSAHBCLKCTRL & (1U << 19)) != 0);
	assert((LPC_GPIO_PIN_INT->ISEL & mask) == 0);
	assert(LPC_GPIO_PIN_INT->IST == mask);
	LPC_GPIO_PIN_INT->IENR &= ~LPC_GPIO_PIN_INT->CIENR;
	LPC_GPIO_PIN_INT->IENF &= ~LPC_GPIO_PIN_INT->CIENF;
	LPC_GPIO_PIN_INT->CIENR = 0;
	LPC_GPIO_PIN_INT->CIENF = 0;
	LPC_GPIO_PIN_INT->IST = 0;
	s_Pending &= ~mask;
}

void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	assert(Priority < 4U);
	s_Priority[Irq] = Priority;
}

void NVIC_EnableIRQ(IRQn_Type Irq)
{
	assert((LPC_GPIO_PIN_INT->ISEL & (1U << Irq)) == 0);
	LPC_GPIO_PIN_INT->IENR |= LPC_GPIO_PIN_INT->SIENR;
	LPC_GPIO_PIN_INT->IENF |= LPC_GPIO_PIN_INT->SIENF;
	LPC_GPIO_PIN_INT->SIENR = 0;
	LPC_GPIO_PIN_INT->SIENF = 0;
	s_Enabled |= 1U << Irq;
}

static void PinEvent(int IntNo, void *Context)
{
	unsigned mask = 1U << IntNo;
	// Driver must acknowledge exactly this channel before calling us.
	assert(LPC_GPIO_PIN_INT->IST == mask);
	LPC_GPIO_PIN_INT->IST = 0;
	s_CallbackCount++;
	s_LastChannel = IntNo;
	s_LastContext = Context;
	if (s_Reedge)
	{
		LPC_GPIO_PIN_INT->IST = mask;
	}
	if (s_Release)
	{
		IOPinDisableInterrupt(IntNo);
	}
}
#endif
