/**-------------------------------------------------------------------------
@file	stm32wbaxx_ll_rcc.h

@brief	RCC LL names used by the STM32CubeWBA BLE platform files.

linklayer_plat.c includes stm32wbaxx_ll_rcc.h by name, and ll_sys_if.c uses the
same items through main.h. IOsonata does not use the ST LL drivers. This
header gives those files the radio sleep timer clock source and the HSE ready
flag with CMSIS register access. See stm32wbaxx_hal.h.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#ifndef __STM32WBAXX_LL_RCC_H__
#define __STM32WBAXX_LL_RCC_H__

#include <stdint.h>

#include "stm32wbaxx.h"

// 2.4 GHz RADIO sleep timer clock source, RCC_BDCR1 RADIOSTSEL field
#define LL_RCC_RADIOSLEEPSOURCE_NONE		0U
#define LL_RCC_RADIOSLEEPSOURCE_LSE			RCC_BDCR1_RADIOSTSEL_0
#define LL_RCC_RADIOSLEEPSOURCE_LSI			RCC_BDCR1_RADIOSTSEL_1
#define LL_RCC_RADIOSLEEPSOURCE_HSE_DIV1000	RCC_BDCR1_RADIOSTSEL

#ifdef __cplusplus
extern "C" {
#endif

// 1 when the HSE oscillator is ready
static inline uint32_t LL_RCC_HSE_IsReady(void)
{
	return (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RCC_CR_HSERDY) ? 1UL : 0UL;
}

// Selected 2.4 GHz RADIO sleep timer clock, one of LL_RCC_RADIOSLEEPSOURCE_*
static inline uint32_t LL_RCC_RADIO_GetSleepTimerClockSource(void)
{
	return READ_BIT(RCC->BDCR1, RCC_BDCR1_RADIOSTSEL);
}

// 1 when the 2.4 GHz RADIO sleep timer clock is on, RCC_RADIOENR STRADIOCLKON
static inline uint32_t LL_RCC_RADIO_IsEnabledSleepTimerClock(void)
{
	return (READ_BIT(RCC->RADIOENR, RCC_RADIOENR_STRADIOCLKON) == RCC_RADIOENR_STRADIOCLKON) ? 1UL : 0UL;
}

#ifdef __cplusplus
}
#endif

#endif // __STM32WBAXX_LL_RCC_H__
