/**-------------------------------------------------------------------------
@file	iopincfg_stm32.cpp

@brief	Shared STM32 GPIO configuration and EXTI implementation.

		This implementation uses the ST CMSIS GPIO_TypeDef, RCC, EXTI and
		SYSCFG definitions selected by stm32.h. Family differences are kept
		local to this source. No board wiring is defined here.

@author	Hoang Nguyen Hoan
@date	October 10, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdbool.h>

#include "iopinctrl.h"

#define IOPIN_MAX_INT			16

typedef struct {
	IOPinEvtHandler_t Handler;
	void *pContext;
	uint8_t PortNo;
} Stm32ExtiHook_t;

static Stm32ExtiHook_t s_ExtiHook[IOPIN_MAX_INT];

static bool Stm32GpioEnableClock(int PortNo)
{
	if (Stm32Gpio(PortNo) == NULL)
	{
		return false;
	}

#if defined(IOSONATA_STM32_F0)
	uint32_t mask = 0U;

	switch (PortNo)
	{
#ifdef RCC_AHBENR_GPIOAEN
		case IOPORTA: mask = RCC_AHBENR_GPIOAEN; break;
#endif
#ifdef RCC_AHBENR_GPIOBEN
		case IOPORTB: mask = RCC_AHBENR_GPIOBEN; break;
#endif
#ifdef RCC_AHBENR_GPIOCEN
		case IOPORTC: mask = RCC_AHBENR_GPIOCEN; break;
#endif
#ifdef RCC_AHBENR_GPIODEN
		case IOPORTD: mask = RCC_AHBENR_GPIODEN; break;
#endif
#ifdef RCC_AHBENR_GPIOFEN
		case IOPORTF: mask = RCC_AHBENR_GPIOFEN; break;
#endif
		default: break;
	}

	if (mask == 0U)
	{
		return false;
	}

	RCC->AHBENR |= mask;
	(void)RCC->AHBENR;

#elif defined(IOSONATA_STM32_F4)
	uint32_t mask = 0U;

	switch (PortNo)
	{
#ifdef RCC_AHB1ENR_GPIOAEN
		case IOPORTA: mask = RCC_AHB1ENR_GPIOAEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIOBEN
		case IOPORTB: mask = RCC_AHB1ENR_GPIOBEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIOCEN
		case IOPORTC: mask = RCC_AHB1ENR_GPIOCEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIODEN
		case IOPORTD: mask = RCC_AHB1ENR_GPIODEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIOEEN
		case IOPORTE: mask = RCC_AHB1ENR_GPIOEEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIOHEN
		case IOPORTH: mask = RCC_AHB1ENR_GPIOHEN; break;
#endif
		default: break;
	}

	if (mask == 0U)
	{
		return false;
	}

	RCC->AHB1ENR |= mask;
	(void)RCC->AHB1ENR;

#elif defined(IOSONATA_STM32_L4) || defined(IOSONATA_STM32_WBA)
	uint32_t mask = 0U;

	switch (PortNo)
	{
#ifdef RCC_AHB2ENR_GPIOAEN
		case IOPORTA: mask = RCC_AHB2ENR_GPIOAEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIOBEN
		case IOPORTB: mask = RCC_AHB2ENR_GPIOBEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIOCEN
		case IOPORTC: mask = RCC_AHB2ENR_GPIOCEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIODEN
		case IOPORTD: mask = RCC_AHB2ENR_GPIODEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIOEEN
		case IOPORTE: mask = RCC_AHB2ENR_GPIOEEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIOFEN
		case IOPORTF: mask = RCC_AHB2ENR_GPIOFEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIOGEN
		case IOPORTG: mask = RCC_AHB2ENR_GPIOGEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIOHEN
		case IOPORTH: mask = RCC_AHB2ENR_GPIOHEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIOIEN
		case IOPORTI: mask = RCC_AHB2ENR_GPIOIEN; break;
#endif
#ifdef RCC_AHB2ENR_GPIOJEN
		case IOPORTJ: mask = RCC_AHB2ENR_GPIOJEN; break;
#endif
		default: break;
	}

	if (mask == 0U)
	{
		return false;
	}

#if defined(IOSONATA_STM32_L4)
	if (PortNo == IOPORTG)
	{
#if defined(RCC_APB1ENR1_PWREN) && defined(PWR_CR2_IOSV)
		RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
		(void)RCC->APB1ENR1;
		PWR->CR2 |= PWR_CR2_IOSV;
#endif
	}
#endif

	RCC->AHB2ENR |= mask;
	(void)RCC->AHB2ENR;

#else
	return false;
#endif

	return true;
}

static IRQn_Type Stm32ExtiIRQ(int PinNo)
{
#if defined(IOSONATA_STM32_F0)
	if (PinNo < 2)
	{
		return EXTI0_1_IRQn;
	}
	if (PinNo < 4)
	{
		return EXTI2_3_IRQn;
	}
	return EXTI4_15_IRQn;

#elif defined(IOSONATA_STM32_WBA)
	switch (PinNo)
	{
		case 0: return EXTI0_IRQn;
		case 1: return EXTI1_IRQn;
		case 2: return EXTI2_IRQn;
		case 3: return EXTI3_IRQn;
		case 4: return EXTI4_IRQn;
		case 5: return EXTI5_IRQn;
		case 6: return EXTI6_IRQn;
		case 7: return EXTI7_IRQn;
		case 8: return EXTI8_IRQn;
		case 9: return EXTI9_IRQn;
		case 10: return EXTI10_IRQn;
		case 11: return EXTI11_IRQn;
		case 12: return EXTI12_IRQn;
		case 13: return EXTI13_IRQn;
		case 14: return EXTI14_IRQn;
		case 15: return EXTI15_IRQn;
		default: return EXTI0_IRQn;
	}

#else
	switch (PinNo)
	{
		case 0: return EXTI0_IRQn;
		case 1: return EXTI1_IRQn;
		case 2: return EXTI2_IRQn;
		case 3: return EXTI3_IRQn;
		case 4: return EXTI4_IRQn;
		default: return PinNo < 10 ? EXTI9_5_IRQn : EXTI15_10_IRQn;
	}
#endif
}

static unsigned Stm32ExtiGroupStart(int PinNo)
{
#if defined(IOSONATA_STM32_F0)
	return PinNo < 2 ? 0U : PinNo < 4 ? 2U : 4U;
#elif defined(IOSONATA_STM32_WBA)
	return (unsigned)PinNo;
#else
	return PinNo < 5 ? (unsigned)PinNo : PinNo < 10 ? 5U : 10U;
#endif
}

static unsigned Stm32ExtiGroupEnd(int PinNo)
{
#if defined(IOSONATA_STM32_F0)
	return PinNo < 2 ? 1U : PinNo < 4 ? 3U : 15U;
#elif defined(IOSONATA_STM32_WBA)
	return (unsigned)PinNo;
#else
	return PinNo < 5 ? (unsigned)PinNo : PinNo < 10 ? 9U : 15U;
#endif
}

static bool Stm32ExtiGroupUsed(int PinNo)
{
	for (unsigned i = Stm32ExtiGroupStart(PinNo);
		 i <= Stm32ExtiGroupEnd(PinNo); i++)
	{
		if (s_ExtiHook[i].Handler != NULL)
		{
			return true;
		}
	}

	return false;
}

static uint32_t Stm32ExtiPending(void)
{
#if defined(IOSONATA_STM32_WBA)
	return (EXTI->RPR1 | EXTI->FPR1) & EXTI->IMR1;
#elif defined(IOSONATA_STM32_L4)
	return EXTI->PR1 & EXTI->IMR1;
#else
	return EXTI->PR & EXTI->IMR;
#endif
}

static void Stm32ExtiClear(uint32_t Mask)
{
#if defined(IOSONATA_STM32_WBA)
	EXTI->RPR1 = Mask;
	EXTI->FPR1 = Mask;
#elif defined(IOSONATA_STM32_L4)
	EXTI->PR1 = Mask;
#else
	EXTI->PR = Mask;
#endif
}

static void Stm32ExtiMask(uint32_t Mask, bool Enable)
{
#if defined(IOSONATA_STM32_WBA) || defined(IOSONATA_STM32_L4)
	if (Enable)
	{
		EXTI->IMR1 |= Mask;
	}
	else
	{
		EXTI->IMR1 &= ~Mask;
	}
#else
	if (Enable)
	{
		EXTI->IMR |= Mask;
	}
	else
	{
		EXTI->IMR &= ~Mask;
	}
#endif
}

static void Stm32ExtiEdges(uint32_t Mask, IOPINSENSE Sense)
{
	uint32_t rising = (Sense == IOPINSENSE_HIGH_TRANSITION ||
					   Sense == IOPINSENSE_TOGGLE) ? Mask : 0U;
	uint32_t falling = (Sense == IOPINSENSE_LOW_TRANSITION ||
						Sense == IOPINSENSE_TOGGLE) ? Mask : 0U;

#if defined(IOSONATA_STM32_WBA) || defined(IOSONATA_STM32_L4)
	EXTI->RTSR1 = (EXTI->RTSR1 & ~Mask) | rising;
	EXTI->FTSR1 = (EXTI->FTSR1 & ~Mask) | falling;
#else
	EXTI->RTSR = (EXTI->RTSR & ~Mask) | rising;
	EXTI->FTSR = (EXTI->FTSR & ~Mask) | falling;
#endif
}

static void Stm32ExtiSelect(int PortNo, int PinNo)
{
	uint32_t index = (unsigned)PinNo >> 2;

#if defined(IOSONATA_STM32_WBA)
	uint32_t shift = ((unsigned)PinNo & 3U) * EXTI_EXTICR1_EXTI1_Pos;
	uint32_t mask = 0x0FUL << shift;

	EXTI->EXTICR[index] = (EXTI->EXTICR[index] & ~mask) |
						  (((unsigned)PortNo & 0x0FU) << shift);
#else
	uint32_t shift = ((unsigned)PinNo & 3U) * 4U;
	uint32_t mask = 0x0FUL << shift;

	SYSCFG->EXTICR[index] = (SYSCFG->EXTICR[index] & ~mask) |
							(((unsigned)PortNo & 0x0FU) << shift);
#endif
}

static void Stm32ExtiDispatch(unsigned First, unsigned Last)
{
	uint32_t pending = Stm32ExtiPending();

	for (unsigned i = First; i <= Last; i++)
	{
		uint32_t mask = 1UL << i;

		if ((pending & mask) != 0U)
		{
			Stm32ExtiClear(mask);

			if (s_ExtiHook[i].Handler != NULL)
			{
				s_ExtiHook[i].Handler((int)i, s_ExtiHook[i].pContext);
			}
		}
	}
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir,
				 IOPINRES Resistor, IOPINTYPE Type)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg == NULL || (unsigned)PinNo >= 16U ||
		Stm32GpioEnableClock(PortNo) == false)
	{
		return;
	}

	uint32_t shift = (unsigned)PinNo * 2U;
	uint32_t mode = Dir == IOPINDIR_OUTPUT ? 1U : 0U;

	if (PinOp >= IOPINOP_FUNC0 && PinOp <= IOPINOP_FUNC15)
	{
		uint32_t afshift = ((unsigned)PinNo & 7U) * 4U;
		uint32_t af = (unsigned)(PinOp - IOPINOP_FUNC0);
		uint32_t index = (unsigned)PinNo >> 3;

		mode = 2U;
		reg->AFR[index] = (reg->AFR[index] & ~(0x0FUL << afshift)) |
						  (af << afshift);
	}
	else if (PinOp != IOPINOP_GPIO)
	{
		mode = 3U;
	}

	uint32_t pull = 0U;

	if (Resistor == IOPINRES_PULLDOWN)
	{
		pull = 2U;
	}
	else if (Resistor == IOPINRES_PULLUP || Resistor == IOPINRES_FOLLOW)
	{
		pull = 1U;
	}

	uint32_t mask = 1UL << (unsigned)PinNo;

	reg->OTYPER = (reg->OTYPER & ~mask) |
				   (Type == IOPINTYPE_OPENDRAIN ? mask : 0U);
	reg->PUPDR = (reg->PUPDR & ~(3UL << shift)) | (pull << shift);
	reg->MODER = (reg->MODER & ~(3UL << shift)) | (mode << shift);
}

void IOPinDisable(int PortNo, int PinNo)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg == NULL || (unsigned)PinNo >= 16U)
	{
		return;
	}

	uint32_t shift = (unsigned)PinNo * 2U;

	reg->MODER |= 3UL << shift;
	reg->PUPDR &= ~(3UL << shift);
}

void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	(void)PortNo;
	(void)PinNo;
	(void)Strength;
}

void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg == NULL || (unsigned)PinNo >= 16U)
	{
		return;
	}

	uint32_t shift = (unsigned)PinNo * 2U;
	uint32_t value = Speed == IOPINSPEED_LOW ? 0U :
					 Speed == IOPINSPEED_MEDIUM ? 1U :
					 Speed == IOPINSPEED_HIGH ? 2U : 3U;

	reg->OSPEEDR = (reg->OSPEEDR & ~(3UL << shift)) | (value << shift);
}

void IOPinDisableInterrupt(int IntNo)
{
	if ((unsigned)IntNo >= IOPIN_MAX_INT ||
		s_ExtiHook[IntNo].Handler == NULL)
	{
		return;
	}

	uint32_t mask = 1UL << (unsigned)IntNo;
	IRQn_Type irq = Stm32ExtiIRQ(IntNo);

	Stm32ExtiMask(mask, false);
	Stm32ExtiEdges(mask, IOPINSENSE_DISABLE);
	Stm32ExtiClear(mask);

	s_ExtiHook[IntNo].Handler = NULL;
	s_ExtiHook[IntNo].pContext = NULL;
	s_ExtiHook[IntNo].PortNo = 0U;

	if (Stm32ExtiGroupUsed(IntNo) == false)
	{
		NVIC_DisableIRQ(irq);
		NVIC_ClearPendingIRQ(irq);
	}
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo,
						  uint32_t PinNo, IOPINSENSE Sense,
						  IOPinEvtHandler_t Handler, void *pContext)
{
	if (PinNo >= IOPIN_MAX_INT || IntNo != (int)PinNo ||
		Stm32Gpio((int)PortNo) == NULL || Handler == NULL ||
		(unsigned)IntPrio >= (1UL << __NVIC_PRIO_BITS) ||
		(Sense != IOPINSENSE_LOW_TRANSITION &&
		 Sense != IOPINSENSE_HIGH_TRANSITION &&
		 Sense != IOPINSENSE_TOGGLE) ||
		s_ExtiHook[PinNo].Handler != NULL)
	{
		return false;
	}

#if defined(IOSONATA_STM32_F0)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	(void)RCC->APB2ENR;
#elif defined(IOSONATA_STM32_F4) || defined(IOSONATA_STM32_L4)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	(void)RCC->APB2ENR;
#endif

	uint32_t mask = 1UL << PinNo;
	IRQn_Type irq = Stm32ExtiIRQ((int)PinNo);

	Stm32ExtiMask(mask, false);
	Stm32ExtiSelect((int)PortNo, (int)PinNo);
	Stm32ExtiEdges(mask, Sense);
	Stm32ExtiClear(mask);

	s_ExtiHook[PinNo].PortNo = (uint8_t)PortNo;
	s_ExtiHook[PinNo].pContext = pContext;
	s_ExtiHook[PinNo].Handler = Handler;

	NVIC_SetPriority(irq, (uint32_t)IntPrio);
	NVIC_ClearPendingIRQ(irq);
	Stm32ExtiMask(mask, true);
	NVIC_EnableIRQ(irq);

	return true;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo,
						   IOPINSENSE Sense, IOPinEvtHandler_t Handler,
						   void *pContext)
{
	if ((unsigned)PinNo >= IOPIN_MAX_INT)
	{
		return -1;
	}

	return IOPinEnableInterrupt(PinNo, IntPrio, (uint32_t)PortNo,
								(uint32_t)PinNo, Sense, Handler,
								pContext) ? PinNo : -1;
}

void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	if ((unsigned)PinNo >= IOPIN_MAX_INT ||
		s_ExtiHook[PinNo].Handler == NULL ||
		s_ExtiHook[PinNo].PortNo != (uint8_t)PortNo)
	{
		return;
	}

	Stm32ExtiEdges(1UL << (unsigned)PinNo, Sense);
}

extern "C" {

#if defined(IOSONATA_STM32_F0)

void EXTI0_1_IRQHandler(void) { Stm32ExtiDispatch(0U, 1U); }
void EXTI2_3_IRQHandler(void) { Stm32ExtiDispatch(2U, 3U); }
void EXTI4_15_IRQHandler(void) { Stm32ExtiDispatch(4U, 15U); }

#elif defined(IOSONATA_STM32_WBA)

void EXTI0_IRQHandler(void)  { Stm32ExtiDispatch(0U, 0U); }
void EXTI1_IRQHandler(void)  { Stm32ExtiDispatch(1U, 1U); }
void EXTI2_IRQHandler(void)  { Stm32ExtiDispatch(2U, 2U); }
void EXTI3_IRQHandler(void)  { Stm32ExtiDispatch(3U, 3U); }
void EXTI4_IRQHandler(void)  { Stm32ExtiDispatch(4U, 4U); }
void EXTI5_IRQHandler(void)  { Stm32ExtiDispatch(5U, 5U); }
void EXTI6_IRQHandler(void)  { Stm32ExtiDispatch(6U, 6U); }
void EXTI7_IRQHandler(void)  { Stm32ExtiDispatch(7U, 7U); }
void EXTI8_IRQHandler(void)  { Stm32ExtiDispatch(8U, 8U); }
void EXTI9_IRQHandler(void)  { Stm32ExtiDispatch(9U, 9U); }
void EXTI10_IRQHandler(void) { Stm32ExtiDispatch(10U, 10U); }
void EXTI11_IRQHandler(void) { Stm32ExtiDispatch(11U, 11U); }
void EXTI12_IRQHandler(void) { Stm32ExtiDispatch(12U, 12U); }
void EXTI13_IRQHandler(void) { Stm32ExtiDispatch(13U, 13U); }
void EXTI14_IRQHandler(void) { Stm32ExtiDispatch(14U, 14U); }
void EXTI15_IRQHandler(void) { Stm32ExtiDispatch(15U, 15U); }

#else

void EXTI0_IRQHandler(void) { Stm32ExtiDispatch(0U, 0U); }
void EXTI1_IRQHandler(void) { Stm32ExtiDispatch(1U, 1U); }
void EXTI2_IRQHandler(void) { Stm32ExtiDispatch(2U, 2U); }
void EXTI3_IRQHandler(void) { Stm32ExtiDispatch(3U, 3U); }
void EXTI4_IRQHandler(void) { Stm32ExtiDispatch(4U, 4U); }
void EXTI9_5_IRQHandler(void) { Stm32ExtiDispatch(5U, 9U); }
void EXTI15_10_IRQHandler(void) { Stm32ExtiDispatch(10U, 15U); }

#endif

}
