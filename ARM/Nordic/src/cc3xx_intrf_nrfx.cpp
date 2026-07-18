/**-------------------------------------------------------------------------
@file	cc3xx_intrf_nrfx.cpp

@brief	Nordic implementation of the CC3xx interface vendor surface.

		Supplies the register file base and the CryptoCell wrapper power
		operations for Nordic targets (nRF52840 CC310, nRF5340 application
		core CC312). This is the only CC3xx file that touches Nordic
		registers.

@author	Hoang Nguyen Hoan
@date	Jul. 12, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "nrf.h"
#include "cc3xx_intrf.h"

#if defined(NRF_CRYPTOCELL)
#define CC3XX_WRAPPER				NRF_CRYPTOCELL
#elif defined(NRF_CRYPTOCELL_S)
#define CC3XX_WRAPPER				NRF_CRYPTOCELL_S
#else
#error CC3xx wrapper is not defined for the selected Nordic target
#endif

#ifndef CC3XX_POWERUP_SPINS
#define CC3XX_POWERUP_SPINS			256U
#endif

uintptr_t Cc3xxBase(void)
{
	return 0x5002B000UL;
}

bool Cc3xxEnable(void)
{
	CC3XX_WRAPPER->ENABLE = 1U;

	for (uint32_t idx = 0U; idx < CC3XX_POWERUP_SPINS; idx++)
	{
		__asm volatile("");
	}

	return CC3XX_WRAPPER->ENABLE != 0U;
}

void Cc3xxDisable(void)
{
	CC3XX_WRAPPER->ENABLE = 0U;
}
