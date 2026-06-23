/**-------------------------------------------------------------------------
@file	crypto_rng.cpp

@brief	Crypto engine: hardware RNG via the platform RngGet driver.

Generic RNG-only crypto engine. Wraps the platform hardware random number
generator (RngGet, e.g. rng_nrfx.c driving the Nordic RNG / CRACEN peripheral,
including its secure/non-secure variants). Advertises only CRYPTO_CAP_RNG, so
it composes as the RNG slot for a consumer (SMP, TLS) whose other slots are
filled by an ECDH/AES engine, so single-capability engines combine
into a full set.

This is independent of any BLE controller: the entropy comes from the MCU's
dedicated RNG hardware, available whether or not a radio is running.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include "crypto/crypto.h"

// Platform hardware RNG driver (rng_nrfx.c on Nordic; provide on other platforms).
extern "C" bool RngGet(uint8_t *pBuff, size_t Len);

static void RngHwRand(CryptoDev_t * const pDev, uint8_t *pBuf, size_t Len)
{
	(void)pDev;
	RngGet(pBuf, Len);
}

extern "C" bool CryptoRngHwInit(CryptoDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}
	pDev->pDevData       = nullptr;
	pDev->pName          = "rng-hw";
	pDev->Cap            = CRYPTO_CAP_RNG;	// RNG only
	pDev->EvtCB          = nullptr;			// synchronous
	pDev->Aes128Ecb      = nullptr;
	pDev->EcdhP256KeyGen = nullptr;
	pDev->EcdhP256       = nullptr;
	pDev->Rand           = RngHwRand;
	pDev->SelfTest       = nullptr;
	return true;
}
