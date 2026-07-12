/**-------------------------------------------------------------------------
@file	crypto_cc3xx.h

@brief	nRF52840 CryptoCell CC310 configuration for the Arm CC3xx driver.

		This target-specific header contains both the imported CC3xx driver
		configuration and the hardware enable/disable operations required before
		accessing the CryptoCell core registers.

		Randomness comes from IOsonata RngGet through ARM/src/crypto_cc3xx.cpp;
		the CC3xx TRNG, entropy conditioner, and DRBG are not built.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_CC3XX_H__
#define __CRYPTO_CC3XX_H__

#include <stdint.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "nrf.h"

//----------------------------------------------------------------------------
// Hardware instance
//----------------------------------------------------------------------------

// CC310 register file on the nRF52840.
#define CC3XX_CONFIG_BASE_ADDRESS			(0x5002B000UL)

#ifndef CC3XX_CONFIG_HW_VERSION_CC310
#define CC3XX_CONFIG_HW_VERSION_CC310
#endif

#if defined(NRF_CRYPTOCELL)
#define CC3XX_WRAPPER			NRF_CRYPTOCELL
#elif defined(NRF_CRYPTOCELL_S)
#define CC3XX_WRAPPER			NRF_CRYPTOCELL_S
#else
#error CC3xx wrapper is not defined for the selected Nordic target
#endif

#ifndef CC3XX_POWERUP_SPINS
#define CC3XX_POWERUP_SPINS		256U
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

//----------------------------------------------------------------------------
// Elliptic curve: P-256 Weierstrass on the PKA
//----------------------------------------------------------------------------

#define CC3XX_CONFIG_EC_CURVE_TYPE_WEIERSTRASS_ENABLE
#define CC3XX_CONFIG_EC_CURVE_SECP_256_R1_ENABLE

#define CC3XX_CONFIG_ECDH_ENABLE
#define CC3XX_CONFIG_ECDSA_KEYGEN_ENABLE

#define CC3XX_CONFIG_PKA_CALC_NP_ENABLE
#define CC3XX_CONFIG_PKA_INLINE_FOR_PERFORMANCE
#define CC3XX_CONFIG_PKA_ALIGN_FOR_PERFORMANCE
#ifndef CC3XX_CONFIG_PKA_MAX_VIRT_REG_AMOUNT
#define CC3XX_CONFIG_PKA_MAX_VIRT_REG_AMOUNT	64
#endif

//----------------------------------------------------------------------------
// Randomness: sourced from IOsonata RngGet, not the CC3xx DRBG
//----------------------------------------------------------------------------

#define CC3XX_CONFIG_RNG_ENABLE

#ifndef CC3XX_CONFIG_RNG_MAX_ATTEMPTS
#define CC3XX_CONFIG_RNG_MAX_ATTEMPTS		100U
#endif

#endif // __CRYPTO_CC3XX_H__
