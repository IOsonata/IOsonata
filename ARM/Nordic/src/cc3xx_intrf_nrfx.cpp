/**-------------------------------------------------------------------------
@file	cc3xx_intrf_nrfx.cpp

@brief	Nordic implementation of the CC3xx interface vendor surface.

		Supplies the register file base and the CryptoCell wrapper power
		operations for Nordic targets (nRF52840 CC310, nRF5340 application
		core CC312). This is the only CC3xx file that touches Nordic
		registers.

@author	Hoang Nguyen Hoan
@date	Jul. 17, 2026

@license MIT, (c) 2026 I-SYST. See crypto.h for full text.
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
