/**-------------------------------------------------------------------------
@file	cc3xx_rng.c

@brief	Nordic platform hooks for the Arm CC3xx low level driver.

		This is a lean replacement for the upstream cc3xx_rng.c
		(tf-psa-crypto-drivers, vendor/arm/cc3xx/low_level_driver/src). The
		upstream file implements a full SP800-90A random path: a TRNG noise
		source, an entropy conditioner, and a CTR or HMAC DRBG, pulling in
		cc3xx_drbg*.c, cc3xx_entropy.c, cc3xx_noise_source.c, and, through the
		DRBG derivation function, cc3xx_aes.c, cc3xx_hash.c, cc3xx_hmac.c,
		cc3xx_kdf.c and cc3xx_chacha.c. That is roughly 4500 lines whose only
		job is to turn hardware entropy into random bytes.

		IOsonata already has that path: RngGet is the crypto RBG interface,
		backed by the target hardware entropy driver, and it is the single
		RBG the rest of the crypto layer uses. This file routes the three
		CC3xx RNG entry points to RngGet, so the CC3xx driver reduces to the
		PKA and elliptic curve arithmetic it uniquely provides, and the whole
		DRBG and entropy stack drops out of the build under gc-sections.

		The same linked translation unit implements the generic CC3xx target
		wrapper hooks. cc3xx_nrfx.h selects the verified Nordic MDK wrapper
		symbol for the target; no numeric peripheral address is duplicated.

		Only these three entry points are part of the CC3xx RNG public API
		(cc3xx_rng.h). With CC3XX_CONFIG_DPA_MITIGATIONS_ENABLE off, the EC
		and init side channel uses are compiled out, so the only live caller
		is the private key scalar generation in cc3xx_pka.c
		(pka_set_to_random), which draws cryptographic random bytes: exactly
		what RngGet provides.

		Keep this file in place of the upstream cc3xx_rng.c: compile this one
		and exclude the external cc3xx_rng.c, the same override pattern used
		for the local cc3xx_aes.c.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>

#ifndef CC3XX_CONFIG_FILE
#include "cc3xx_config.h"
#else
#include CC3XX_CONFIG_FILE
#endif

#include "cc3xx_rng.h"
#include "cc3xx_error.h"
#include "cc3xx_port.h"
#include "cc3xx_nrfx.h"

// RngGet is the IOsonata crypto RBG interface, declared in crypto.h. It is
// declared here directly rather than including the C++ crypto.h so this C
// translation unit stays free of the C++ header. The definition has C linkage
// in the IOsonata crypto layer.
#ifdef __cplusplus
extern "C" {
#endif
bool RngGet(uint8_t *pBuff, size_t Len);
#ifdef __cplusplus
}
#endif

#ifndef CC3XX_CONFIG_RNG_MAX_ATTEMPTS
#define CC3XX_CONFIG_RNG_MAX_ATTEMPTS		100
#endif

bool Cc3xxPortEnable(void)
{
	CC3XX_NRFX_WRAPPER->ENABLE = 1;

	return CC3XX_NRFX_WRAPPER->ENABLE != 0;
}

void Cc3xxPortDisable(void)
{
	CC3XX_NRFX_WRAPPER->ENABLE = 0;
}

// Fill a buffer with cryptographic random bytes from RngGet. The quality
// argument from the CC3xx API is ignored: RngGet is always the hardware
// backed RBG, so there is no lower quality fast path to select. A NULL buffer
// with zero length is a valid no-op, matching the upstream behavior.
cc3xx_err_t cc3xx_lowlevel_rng_get_random(uint8_t *buf, size_t length,
										  enum cc3xx_rng_quality_t quality)
{
	(void)quality;

	if (buf == NULL)
	{
		if (length != 0)
		{
			return CC3XX_ERR_RNG_INVALID_RNG;
		}
		return CC3XX_ERR_SUCCESS;
	}

	if (RngGet(buf, length) == false)
	{
		return CC3XX_ERR_RNG_INVALID_RNG;
	}

	return CC3XX_ERR_SUCCESS;
}

// Uniform random value in [0, bound) by rejection sampling, unchanged in
// method from upstream (SP800-90A A.5.1): mask down to the next power of two
// above bound, then discard values at or above bound. Only the byte source
// differs, RngGet instead of the DRBG.
cc3xx_err_t cc3xx_lowlevel_rng_get_random_uint(uint32_t bound, uint32_t *uint,
										   enum cc3xx_rng_quality_t quality)
{
	uint32_t value;
	uint32_t attempts = 0;
	uint32_t mask;
	cc3xx_err_t err;

	// Runtime validation: with NDEBUG the assert vanishes, and bound == 0
	// would reach (bound - 1) wraparound and __builtin_clz(0), which is
	// undefined. The upstream file has only the assert; this replacement
	// keeps the API safe unconditionally.
	assert(bound != 0);
	if (bound == 0 || uint == NULL)
	{
		return CC3XX_ERR_RNG_INVALID_RNG;
	}

	if ((bound & (bound - 1)) == 0)
	{
		// Single bit set: mask is bound minus one.
		mask = bound - 1;
	}
	else
	{
		mask = UINT32_MAX >> __builtin_clz(bound);
	}

	do {
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

	*uint = value;
	return CC3XX_ERR_SUCCESS;
}

// Identity permutation. The upstream shuffle is a DPA side channel
// countermeasure and only runs under CC3XX_CONFIG_DPA_MITIGATIONS_ENABLE,
// which is off in this build, so upstream also returns the identity here.
void cc3xx_lowlevel_rng_get_random_permutation(uint8_t *permutation_buf,
											   size_t len)
{
	size_t idx;

	// The entries are uint8_t, so indices past 256 would wrap; the driver
	// callers stay under 256 words per copy (asserted in cc3xx_pka.c). Guard
	// here too so this local replacement is safe on its own.
	assert(permutation_buf != NULL);
	assert(len <= 256U);
	if (permutation_buf == NULL || len > 256U)
	{
		return;
	}

	for (idx = 0; idx < len; idx++)
	{
		permutation_buf[idx] = (uint8_t)idx;
	}
}
