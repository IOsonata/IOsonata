/**-------------------------------------------------------------------------
@file	crypto_uecc.cpp

@brief	Crypto engine: micro-ecc (uECC) software ECDH over P-256.

Generic software crypto engine implementing the ECDH capability of CryptoDev_t
via micro-ecc. ECDH only - no symmetric cipher, no RNG of its own: uECC needs
an RNG registered, which this engine borrows from the platform via the RNG
function passed at registration (CryptoUeccSetRng), so it composes with a
separate RNG engine to cover the RNG capability it does not provide itself.

Like a sensor that supports only one axis-type, this engine advertises only
CRYPTO_CAP_ECDH_P256. A consumer needing AES or RNG composes it with another
engine that supplies those (see bt_smp's three-slot Init).

Byte order: built with uECC defaults (big-endian), matching the SMP toolbox
(f4/f5/f6) and the over-the-air Public Key PDU, so keys/secret pass straight
through with no reversal.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"

#if defined(CRYPTO_HAS_UECC) || \
	(defined(__has_include) && __has_include("uECC.h"))

#include "uECC.h"

// Our P-256 private key, retained between keygen and the DH computation.
static uint8_t s_PrivKey[32];

// RNG source for uECC. micro-ecc requires an RNG before key generation; it is
// not part of this engine. The application (or the composing consumer) provides
// one - typically the platform hardware RNG (RngGet) - via CryptoUeccSetRng.
extern "C" bool RngGet(uint8_t *pBuff, size_t Len);

static int UeccRngAdapter(uint8_t *pDest, unsigned Size)
{
	return RngGet(pDest, Size) ? 1 : 0;
}

static CRYPTO_STATUS UeccEcdhKeyGen(CryptoDev_t * const pDev, uint8_t pPubKey[64],
									void *pCtx)
{
	(void)pDev; (void)pCtx;

	if (uECC_get_rng() == nullptr)
	{
		uECC_set_rng(UeccRngAdapter);
	}

	if (uECC_make_key(pPubKey, s_PrivKey, uECC_secp256r1()) != 1)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS UeccEcdh(CryptoDev_t * const pDev,
							  const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
							  void *pCtx)
{
	(void)pDev; (void)pCtx;

	uint8_t secret[32];
	if (uECC_shared_secret(pPeerPubKey, s_PrivKey, secret, uECC_secp256r1()) != 1)
	{
		return CRYPTO_STATUS_FAIL;
	}
	memcpy(pDhKey, secret, 32);	// DHKey = X coordinate, big-endian
	return CRYPTO_STATUS_OK;
}

// Self-test: BLE spec P-256 DH known vector (Vol 3 Part H 2.3.5.6.1).
static int UeccSelfTest(CryptoDev_t * const pDev)
{
	(void)pDev;
	static const uint8_t privA[32] = {
		0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
		0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd };
	static const uint8_t pubB[64] = {
		0x1e,0xa1,0xf0,0xf0,0x1f,0xaf,0x1d,0x96,0x09,0x59,0x22,0x84,0xf1,0x9e,0x4c,0x00,
		0x47,0xb5,0x8a,0xfd,0x86,0x15,0xa6,0x9f,0x55,0x90,0x77,0xb2,0x2f,0xaa,0xa1,0x90,
		0x4c,0x55,0xf3,0x3e,0x42,0x9d,0xad,0x37,0x73,0x56,0x70,0x3a,0x9a,0xb8,0x51,0x60,
		0x47,0x2d,0x11,0x30,0xe2,0x8e,0x36,0x76,0x5f,0x89,0xaf,0xf9,0x15,0xb1,0x21,0x4a };
	static const uint8_t dhExpect[32] = {
		0xec,0x02,0x34,0xa3,0x57,0xc8,0xad,0x05,0x34,0x10,0x10,0xa6,0x0a,0x39,0x7d,0x9b,
		0x99,0x79,0x6b,0x13,0xb4,0xf8,0x66,0xf1,0x86,0x8d,0x34,0xf3,0x73,0xbf,0xa6,0x98 };

	uint8_t dh[32];
	if (uECC_shared_secret(pubB, privA, dh, uECC_secp256r1()) != 1)
	{
		return -1;
	}
	return memcmp(dh, dhExpect, 32) == 0 ? 0 : -2;
}

extern "C" bool CryptoUeccInit(CryptoDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}
	pDev->pDevData       = nullptr;
	pDev->pName          = "uecc";
	pDev->Cap            = CRYPTO_CAP_ECDH_P256;	// ECDH only
	pDev->EvtCB          = nullptr;					// synchronous
	pDev->Aes128Ecb      = nullptr;					// not provided
	pDev->EcdhP256KeyGen = UeccEcdhKeyGen;
	pDev->EcdhP256       = UeccEcdh;
	pDev->Rand           = nullptr;					// not provided
	pDev->SelfTest       = UeccSelfTest;
	return true;
}

#else  // uECC not available on this target

extern "C" bool CryptoUeccInit(CryptoDev_t * const pDev)
{
	(void)pDev;
	return false;	// micro-ecc not present in this build
}

#endif // CRYPTO_HAS_UECC || __has_include("uECC.h")
