/**-------------------------------------------------------------------------
@file	rng_nrfx.cpp

@brief	Random number generator implementation on Nordic nRF series.

		Provides the RngInit/RngGet declared in crypto/crypto.h. There is no
		software default: a part without an RNG peripheral does not link. Any
		code that calls RngGet (crypto engines for key generation, SMP, etc.)
		gets hardware entropy here, with no knowledge of the underlying engine.

		Two hardware paths:
		- nRF54L / nRF54H: CRACEN. Random is taken from the NIST SP800-90A
		  CTR-DRBG that nrfx seeds from the CRACEN TRNG. The CRACEN lock is
		  shared with the direct hardware crypto provider.
		- nRF51/52/53/91: the legacy RNG peripheral with bias correction
		  (DERCEN) enabled.

@author	Hoang Nguyen Hoan
@date	Aug. 9, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>

#include "nrf.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#include "crypto_rng_nrf.h"

#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
#include "nrfx_cracen.h"
#include "crypto_cracen.h"
#define RNG_USE_CRACEN		1

static volatile bool s_CracenBusy;

bool CracenTryAcquire(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	bool acquired = !s_CracenBusy;
	if (acquired)
	{
		s_CracenBusy = true;
		__DMB();
	}

	if (primask == 0)
	{
		__enable_irq();
	}

	return acquired;
}

void CracenRelease(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	__DMB();
	s_CracenBusy = false;

	if (primask == 0)
	{
		__enable_irq();
	}
}
#endif

static bool s_RngReady;

bool RngInit(void)
{
#if defined(RNG_USE_CRACEN)
	if (!CracenTryAcquire())
	{
		return false;
	}

	// Bring up the CRACEN CTR-DRBG. nrfx_cracen_init returns 0 on success or
	// -EALREADY when it was initialized by an earlier caller.
	int r = nrfx_cracen_init();
	CracenRelease();

	if (r != 0 && r != -EALREADY)
	{
		s_RngReady = false;
		return false;
	}

	s_RngReady = true;
	return true;
#else
	// Legacy RNG peripheral needs no persistent init; RngGet drives it per call.
	s_RngReady = true;
	return true;
#endif
}

static bool RngHwGet(uint8_t *pBuff, size_t Len)
{
	if (pBuff == NULL || Len == 0)
	{
		return false;
	}

#if defined(RNG_USE_CRACEN)
	if (!s_RngReady)
	{
		if (!RngInit())
		{
			return false;
		}
	}

	if (!CracenTryAcquire())
	{
		return false;
	}

	// CTR-DRBG output, seeded and reseeded from the CRACEN hardware TRNG.
	int r = nrfx_cracen_ctr_drbg_random_get(pBuff, Len);
	CracenRelease();
	return r == 0;
#else
 #if defined(NRF91_SERIES) || defined(NRF53_SERIES)
  #ifdef NRF5340_XXAA_NETWORK
	NRF_RNG_Type *reg = NRF_RNG_NS;
  #else
	NRF_RNG_Type *reg = NRF_RNG_S;
  #endif
 #else
	NRF_RNG_Type *reg = NRF_RNG;
 #endif

 #if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	// While the SoftDevice is enabled the RNG peripheral is SoftDevice owned;
	// direct register access asserts the stack. Draw from the SoftDevice
	// entropy pool instead. The pool refills at a finite rate, so take what is
	// available and wait for more until the request is filled.
	uint8_t sdEnabled = 0;

	(void)sd_softdevice_is_enabled(&sdEnabled);

	if (sdEnabled)
	{
		size_t idx = 0;

		while (idx < Len)
		{
			uint8_t avail = 0;

			(void)sd_rand_application_bytes_available_get(&avail);

			if (avail == 0)
			{
				continue;
			}

			size_t n = Len - idx;

			if (n > avail)
			{
				n = avail;
			}

			if (sd_rand_application_vector_get(&pBuff[idx], (uint8_t)n) == NRF_SUCCESS)
			{
				idx += n;
			}
		}

		return true;
	}
 #endif

	// Enable bias correction so the byte stream is unbiased.
	reg->CONFIG = RNG_CONFIG_DERCEN_Enabled;
	reg->TASKS_START = 1;

	for (size_t i = 0; i < Len; i++)
	{
		reg->EVENTS_VALRDY = 0;
		while (reg->EVENTS_VALRDY == 0);
		pBuff[i] = (uint8_t)reg->VALUE;
	}

	reg->TASKS_STOP = 1;
	reg->CONFIG = RNG_CONFIG_DERCEN_Disabled;

	return true;
#endif
}


//-----------------------------------------------------------------------------
// OO engine wrapper. CryptoRngNrf presents the hardware RNG as a security-grade
// RngEngine; the raw draw and init above are unchanged. The singleton lives in
// internal static storage so there is no allocation.
//-----------------------------------------------------------------------------
bool CryptoRngNrf::Enable(void)
{
	vbValid = RngInit();
	return vbValid;
}

CRYPTO_STATUS CryptoRngNrf::Random(uint8_t *pOut, size_t Len)
{
	if (pOut == nullptr)
	{
		return (Len == 0) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
	return RngHwGet(pOut, Len) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CryptoRngNrf *CryptoRngNrfInstance(void)
{
	static CryptoRngNrf s_Instance;
	return &s_Instance;
}

//-----------------------------------------------------------------------------
// Compatibility shim. Existing callers (SMP, the direct hardware crypto
// providers) call the free function RngGet; it forwards to the singleton engine
// so there is one hardware path. Retire these shims as callers move to the
// RngEngine facet.
//-----------------------------------------------------------------------------
bool RngGet(uint8_t *pBuff, size_t Len)
{
	return RngHwGet(pBuff, Len);
}
