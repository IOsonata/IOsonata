/**-------------------------------------------------------------------------
@file	cc3xx_port.h

@brief	Nordic target hooks for the generic Arm CC3xx crypto provider.

		This header supplies only the target-specific wrapper enable, disable,
		and power-up settling needed before the generic ARM CC3xx source
		accesses the CryptoCell core registers.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CC3XX_PORT_H__
#define __CC3XX_PORT_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"

#if defined(NRF_CRYPTOCELL)
#define CC3XX_PORT_WRAPPER		NRF_CRYPTOCELL
#elif defined(NRF_CRYPTOCELL_S)
#define CC3XX_PORT_WRAPPER		NRF_CRYPTOCELL_S
#else
#error CC3xx wrapper is not defined for the selected Nordic target
#endif

#ifndef CC3XX_PORT_POWERUP_SPINS
#define CC3XX_PORT_POWERUP_SPINS		256U
#endif

static inline bool Cc3xxPortEnable(void)
{
	CC3XX_PORT_WRAPPER->ENABLE = 1U;

	for (uint32_t idx = 0U; idx < CC3XX_PORT_POWERUP_SPINS; idx++)
	{
		__asm volatile("");
	}

	return CC3XX_PORT_WRAPPER->ENABLE != 0U;
}

static inline void Cc3xxPortDisable(void)
{
	CC3XX_PORT_WRAPPER->ENABLE = 0U;
}

#endif // __CC3XX_PORT_H__
