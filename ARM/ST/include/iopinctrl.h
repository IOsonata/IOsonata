/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	Shared STM32 fast I/O pin control using ST CMSIS GPIO_TypeDef.

		This header contains only the fast inline GPIO access functions.
		GPIO configuration, peripheral clock enabling, EXTI allocation and
		interrupt dispatch are owned by iopincfg_stm32.cpp.

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>

#include "stm32.h"
#include "coredev/iopincfg.h"

static inline GPIO_TypeDef *Stm32Gpio(int PortNo)
{
	switch (PortNo)
	{
#ifdef GPIOA
		case IOPORTA:
			return GPIOA;
#endif
#ifdef GPIOB
		case IOPORTB:
			return GPIOB;
#endif
#ifdef GPIOC
		case IOPORTC:
			return GPIOC;
#endif
#ifdef GPIOD
		case IOPORTD:
			return GPIOD;
#endif
#ifdef GPIOE
		case IOPORTE:
			return GPIOE;
#endif
#ifdef GPIOF
		case IOPORTF:
			return GPIOF;
#endif
#ifdef GPIOG
		case IOPORTG:
			return GPIOG;
#endif
#ifdef GPIOH
		case IOPORTH:
			return GPIOH;
#endif
#ifdef GPIOI
		case IOPORTI:
			return GPIOI;
#endif
#ifdef GPIOJ
		case IOPORTJ:
			return GPIOJ;
#endif
		default:
			return (GPIO_TypeDef *)0;
	}
}

static inline void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg == NULL || (unsigned)PinNo >= 16U)
	{
		return;
	}

	uint32_t shift = (unsigned)PinNo * 2U;

	reg->MODER = (reg->MODER & ~(3UL << shift)) |
				 ((Dir == IOPINDIR_OUTPUT ? 1UL : 0UL) << shift);
}

static inline int IOPinRead(int PortNo, int PinNo)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg == NULL || (unsigned)PinNo >= 16U)
	{
		return 0;
	}

	return (int)((reg->IDR >> (unsigned)PinNo) & 1UL);
}

static inline void IOPinSet(int PortNo, int PinNo)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg != NULL && (unsigned)PinNo < 16U)
	{
		reg->BSRR = 1UL << (unsigned)PinNo;
	}
}

static inline void IOPinClear(int PortNo, int PinNo)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg != NULL && (unsigned)PinNo < 16U)
	{
		reg->BSRR = 1UL << ((unsigned)PinNo + 16U);
	}
}

static inline void IOPinToggle(int PortNo, int PinNo)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg != NULL && (unsigned)PinNo < 16U)
	{
		uint32_t mask = 1UL << (unsigned)PinNo;

		reg->BSRR = (reg->ODR & mask) != 0U ? mask << 16U : mask;
	}
}

static inline uint32_t IOPinReadPort(int PortNo)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	return reg != NULL ? reg->IDR & 0xFFFFU : 0U;
}

static inline void IOPinWritePort(int PortNo, uint32_t Data)
{
	GPIO_TypeDef *reg = Stm32Gpio(PortNo);

	if (reg != NULL)
	{
		reg->ODR = Data & 0xFFFFU;
	}
}

#endif // __IOPINCTRL_H__
