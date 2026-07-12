/**-------------------------------------------------------------------------
@file	cc3xx_config.h

@brief	Temporary build compatibility for the previous CC3xx source set.

		ARM/src/crypto_cc3xx.cpp no longer uses tf-psa-crypto-drivers. The
		nRF52840 Eclipse project still links the old external source files, so
		this header preserves their compile-time configuration until those
		linked resources and include paths are removed. Delete this file with
		the project cleanup.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CC3XX_CONFIG_H__
#define __CC3XX_CONFIG_H__

#include "crypto_cc3xx.h"

#define CC3XX_CONFIG_BASE_ADDRESS			CC3XX_BASE_ADDRESS
#define CC3XX_CONFIG_HW_VERSION_CC310
#define CC3XX_CONFIG_EC_CURVE_TYPE_WEIERSTRASS_ENABLE
#define CC3XX_CONFIG_EC_CURVE_SECP_256_R1_ENABLE
#define CC3XX_CONFIG_ECDH_ENABLE
#define CC3XX_CONFIG_ECDSA_KEYGEN_ENABLE
#define CC3XX_CONFIG_PKA_CALC_NP_ENABLE
#define CC3XX_CONFIG_PKA_INLINE_FOR_PERFORMANCE
#define CC3XX_CONFIG_PKA_ALIGN_FOR_PERFORMANCE
#define CC3XX_CONFIG_PKA_MAX_VIRT_REG_AMOUNT	64
#define CC3XX_CONFIG_RNG_ENABLE
#define CC3XX_CONFIG_RNG_MAX_ATTEMPTS		100U

#endif // __CC3XX_CONFIG_H__
