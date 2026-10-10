/**-------------------------------------------------------------------------
@file	system_stm32wbaxx.c

@brief	STM32WBA common Cortex-M33 system initialization for IOsonata ResetEntry.

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
#include "stm32wbaxx.h"
#include "system_stm32wbaxx.h"

extern void (* const __Vectors[])(void);

uint32_t SystemCoreClock = 16000000U;
const uint8_t AHBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
const uint8_t APBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
const uint8_t AHB5PrescTable[8] = {1U, 1U, 1U, 1U, 2U, 3U, 4U, 6U};

void SystemInit(void)
{
    /* WBA65 reset runs from HSI16. Do not change clock mux, flash latency,
       TrustZone or peripheral ownership until the MCU clock port is present. */
    SCB->VTOR = (uint32_t)__Vectors;
    __DSB();
    __ISB();
    SystemCoreClock = 16000000U;
}

/* Valid for initial HSI16 bring-up only. Replace with live RCC decoding
 * before enabling PLL/HSE or changing AHB prescalers.
 */
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 16000000U;
}
