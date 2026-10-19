/**-------------------------------------------------------------------------
@file	sai_stm32wba.cpp

@brief	STM32WBA SAI1 ownership and interrupt dispatch


@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

#include "stm32wbaxx.h"
#include "sai_stm32wba.h"

static void *s_pOwner;
static WbaSaiHandler_t s_Handler;

bool WbaSaiClaim(void *pOwner, WbaSaiHandler_t Handler)
{
	uint32_t irq = __get_PRIMASK();
	bool result = false;

	__disable_irq();
	if (pOwner && Handler && !s_pOwner && !(RCC->APB2ENR & RCC_APB2ENR_SAI1EN))
	{
		s_pOwner = pOwner;
		s_Handler = Handler;
		result = true;
	}
	__set_PRIMASK(irq);
	return result;
}

void WbaSaiRelease(void *pOwner)
{
	uint32_t irq = __get_PRIMASK();

	__disable_irq();
	if (s_pOwner == pOwner)
	{
		NVIC_DisableIRQ(SAI1_IRQn);
		NVIC_ClearPendingIRQ(SAI1_IRQn);
		s_Handler = nullptr;
		s_pOwner = nullptr;
	}
	__set_PRIMASK(irq);
}

extern "C" void SAI1_IRQHandler()
{
	if (s_Handler)
	{
		s_Handler();
	}
}
