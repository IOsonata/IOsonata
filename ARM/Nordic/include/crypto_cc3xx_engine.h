/**-------------------------------------------------------------------------
@file	crypto_cc3xx_engine.h

@brief	nRF52840 CryptoCell CC310 target configuration.

		Defines the CC310 register base, PKA SRAM size, and target-specific
		hardware enable and disable operations used by ARM/src/crypto_cc3xx.cpp.
		The P-256 driver itself is target-independent and contains no Nordic
		register dependency.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_CC3XX_ENGINE_H__
#define __CRYPTO_CC3XX_ENGINE_H__

#include <stdint.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "nrf.h"

// CC310 core register file and PKA SRAM on the nRF52840.
#define CC3XX_BASE_ADDRESS			(0x5002B000UL)
#define CC3XX_PKA_SRAM_SIZE			(0x1000UL)

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

static inline bool Cc3xxEnable(void)
{
	CC3XX_WRAPPER->ENABLE = 1U;

	for (uint32_t idx = 0U; idx < CC3XX_POWERUP_SPINS; idx++)
	{
		__asm volatile("");
	}

	return CC3XX_WRAPPER->ENABLE != 0U;
}

static inline void Cc3xxDisable(void)
{
	CC3XX_WRAPPER->ENABLE = 0U;
}

#endif // __CRYPTO_CC3XX_ENGINE_H__
