/**-------------------------------------------------------------------------
@file	cc3xx_rng.cpp

@brief	IOsonata RNG adapter for the Arm CC3xx low level driver.

		This file replaces the upstream cc3xx_rng.c from
		tf-psa-crypto-drivers commit 27c8ccd,
		vendor/arm/cc3xx/low_level_driver/src. The upstream implementation
		provides a TRNG noise source, entropy conditioning, and a CTR or HMAC
		DRBG. IOsonata already provides the RngGet cryptographic random-byte
		service, so the three CC3xx RNG entry points are routed to RngGet.

		The imported cc3xx_rng.h provides C-linkage guards, so this C++
		translation unit exports the C symbols expected by the CC3xx driver.

		CC3XX_CONFIG_DPA_MITIGATIONS_ENABLE is disabled in this build. The
		live random-byte caller is private-key scalar generation in cc3xx_pka.c.
		The permutation entry point therefore returns the identity permutation,
		matching the disabled-mitigation path in the upstream implementation.

		Re-check this replacement against the upstream CC3xx RNG interface on
		every tf-psa-crypto-drivers update.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#include "crypto/crypto.h"

#ifndef CC3XX_CONFIG_FILE
#include "cc3xx_config.h"
#else
#include CC3XX_CONFIG_FILE
#endif

#include "cc3xx_rng.h"
#include "cc3xx_error.h"

#ifndef CC3XX_CONFIG_RNG_MAX_ATTEMPTS
#define CC3XX_CONFIG_RNG_MAX_ATTEMPTS		100U
#endif

cc3xx_err_t cc3xx_lowlevel_rng_get_random(uint8_t *pBuff, size_t len,
										  enum cc3xx_rng_quality_t quality)
{
	(void)quality;

	if (pBuff == NULL)
	{
		if (len != 0U)
		{
			return CC3XX_ERR_RNG_INVALID_RNG;
		}
		return CC3XX_ERR_SUCCESS;
	}

	if (RngGet(pBuff, len) == false)
	{
		return CC3XX_ERR_RNG_INVALID_RNG;
	}

	return CC3XX_ERR_SUCCESS;
}

cc3xx_err_t cc3xx_lowlevel_rng_get_random_uint(uint32_t bound,
										   uint32_t *pValue,
										   enum cc3xx_rng_quality_t quality)
{
	uint32_t value;
	uint32_t attempts = 0U;
	uint32_t mask;
	cc3xx_err_t err;

	assert(bound != 0U);
	if (bound == 0U || pValue == NULL)
	{
		return CC3XX_ERR_RNG_INVALID_RNG;
	}

	if ((bound & (bound - 1U)) == 0U)
	{
		mask = bound - 1U;
	}
	else
	{
		mask = UINT32_MAX >> __builtin_clz(bound);
	}

	do
	{
		err = cc3xx_lowlevel_rng_get_random((uint8_t *)&value, sizeof(value),
											quality);
		if (err != CC3XX_ERR_SUCCESS)
		{
			return err;
		}

		value &= mask;

		if (attempts++ >= CC3XX_CONFIG_RNG_MAX_ATTEMPTS)
		{
			return CC3XX_ERR_RNG_TOO_MANY_ATTEMPTS;
		}
	} while (value >= bound);

	*pValue = value;
	return CC3XX_ERR_SUCCESS;
}

void cc3xx_lowlevel_rng_get_random_permutation(uint8_t *pPermutation,
											   size_t len)
{
	assert(pPermutation != NULL);
	assert(len <= 256U);
	if (pPermutation == NULL || len > 256U)
	{
		return;
	}

	for (size_t idx = 0U; idx < len; idx++)
	{
		pPermutation[idx] = (uint8_t)idx;
	}
}
