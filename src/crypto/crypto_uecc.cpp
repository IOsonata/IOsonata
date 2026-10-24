/**-------------------------------------------------------------------------
@file	crypto_uecc.cpp

@brief	Software P-256 crypto engine over micro-ecc.

		micro-ecc exposes one process-wide RNG callback. All calls are therefore
		serialized here so two engine objects, or main and interrupt context, cannot
		rebind the callback while another operation is active.

@author	Hoang Nguyen Hoan
@date	May 31, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#include <stdint.h>
#include <string.h>
#include <atomic>
#include <new>

#include "crypto/crypto_uecc.h"
#include "crypto/crypto.h"
#include "uECC.h"
#include "uECC_vli.h"

#if defined(uECC_VLI_NATIVE_LITTLE_ENDIAN) && uECC_VLI_NATIVE_LITTLE_ENDIAN
#error "crypto_uecc requires the uECC default big-endian build"
#endif

static void UeccWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0U)
	{
		*p++ = 0U;
	}
}

// micro-ecc's co-Z ladder degenerates for 1 and n - 1. Run its normal
// multiplication with 2 for those scalars, then select the exact result.
// Fixed-length masks avoid a private-key-dependent shortcut and keep ECDH's
// randomized initial Z and RNG failure handling in use.
static const uint8_t s_UeccOrder[32] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xBC,0xE6,0xFA,0xAD,0xA7,0x17,0x9E,0x84,0xF3,0xB9,0xCA,0xC2,0xFC,0x63,0x25,0x51,
};
static const uint8_t s_UeccGenerator[64] = {
	0x6B,0x17,0xD1,0xF2,0xE1,0x2C,0x42,0x47,0xF8,0xBC,0xE6,0xE5,0x63,0xA4,0x40,0xF2,
	0x77,0x03,0x7D,0x81,0x2D,0xEB,0x33,0xA0,0xF4,0xA1,0x39,0x45,0xD8,0x98,0xC2,0x96,
	0x4F,0xE3,0x42,0xE2,0xFE,0x1A,0x7F,0x9B,0x8E,0xE7,0xEB,0x4A,0x7C,0x0F,0x9E,0x16,
	0x2B,0xCE,0x33,0x57,0x6B,0x31,0x5E,0xCE,0xCB,0xB6,0x40,0x68,0x37,0xBF,0x51,0xF5,
};
static const uint8_t s_UeccNegativeGy[32] = {
	0xB0,0x1C,0xBD,0x1C,0x01,0xE5,0x80,0x65,0x71,0x18,0x14,0xB5,0x83,0xF0,0x61,0xE9,
	0xD4,0x31,0xCC,0xA9,0x94,0xCE,0xA1,0x31,0x34,0x49,0xBF,0x97,0xC8,0x40,0xAE,0x0A,
};

static uint8_t UeccPrepareScalar(const uint8_t *pKey, uint8_t *pScalar,
								uint8_t &NegativeMask)
{
	uint32_t oneDiff = 0U, negativeDiff = 0U;
	for (size_t i = 0; i < 32U; i++)
	{
		oneDiff |= pKey[i] ^ (i == 31U ? 1U : 0U);
		negativeDiff |= pKey[i] ^ (s_UeccOrder[i] - (i == 31U ? 1U : 0U));
	}
	const uint8_t oneMask = (uint8_t)((oneDiff - 1U) >> 8);
	NegativeMask = (uint8_t)((negativeDiff - 1U) >> 8);
	const uint8_t mask = oneMask | NegativeMask;
	for (size_t i = 0; i < 32U; i++)
	{
		pScalar[i] = (pKey[i] & (uint8_t)~mask) | ((i == 31U ? 2U : 0U) & mask);
	}
	return mask;
}

static int UeccPublicKey(const uint8_t *pKey, uint8_t *pPub)
{
	uint8_t scalar[32], negativeMask;
	const uint8_t mask = UeccPrepareScalar(pKey, scalar, negativeMask);
	int ok = uECC_compute_public_key(scalar, pPub, uECC_secp256r1());
	for (size_t i = 0; i < 64U; i++)
	{
		uint8_t edge = s_UeccGenerator[i];
		if (i >= 32U)
		{
			edge = (edge & (uint8_t)~negativeMask) |
				(s_UeccNegativeGy[i - 32U] & negativeMask);
		}
		pPub[i] = (pPub[i] & (uint8_t)~mask) | (edge & mask);
	}
	UeccWipe(scalar, sizeof(scalar));
	return ok;
}

static std::atomic_flag s_UeccBusy = ATOMIC_FLAG_INIT;
static RngEngine *s_pActiveRng;
static uECC_RNG_Function s_PreviousRng;
static CRYPTO_STATUS s_UeccRngStatus;

static int UeccRngAdapter(uint8_t *pDest, unsigned Size)
{
	if (s_pActiveRng == nullptr)
	{
		s_UeccRngStatus = CRYPTO_STATUS_UNSUPPORTED;
		return 0;
	}
	s_UeccRngStatus = s_pActiveRng->Random(pDest, (size_t)Size);
	return s_UeccRngStatus == CRYPTO_STATUS_OK ? 1 : 0;
}

static CRYPTO_STATUS UeccBegin(RngEngine *pRng, bool bNeedRng)
{
	if (std::atomic_flag_test_and_set(&s_UeccBusy))
	{
		return CRYPTO_STATUS_BUSY;
	}

	s_PreviousRng = uECC_get_rng();
	if (bNeedRng && (pRng == nullptr || !pRng->IsSecure()))
	{
		s_PreviousRng = nullptr;
		std::atomic_flag_clear(&s_UeccBusy);
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	s_UeccRngStatus = CRYPTO_STATUS_OK;
	s_pActiveRng = bNeedRng ? pRng : nullptr;
	uECC_set_rng(bNeedRng ? UeccRngAdapter : nullptr);
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS UeccEnd(void)
{
	CRYPTO_STATUS status = s_UeccRngStatus;
	uECC_set_rng(s_PreviousRng);
	s_PreviousRng = nullptr;
	s_pActiveRng = nullptr;
	s_UeccRngStatus = CRYPTO_STATUS_OK;
	std::atomic_flag_clear(&s_UeccBusy);
	return status;
}

static void UeccKeyReset(KeyAgreeEngine *pOwner, void *pKeyCtx)
{
	if (pOwner != nullptr)
	{
		pOwner->KeyReset(pKeyCtx);
	}
	else if (pKeyCtx != nullptr)
	{
		UeccWipe(pKeyCtx, sizeof(CryptoUecc::KeyCtx));
	}
}

void CryptoUecc::KeyReset(void *pKeyCtx)
{
	if (pKeyCtx != nullptr)
	{
		UeccWipe(pKeyCtx, sizeof(KeyCtx));
	}
}

static CRYPTO_STATUS UeccKeyGen(RngEngine *pRng, KeyAgreeEngine *pOwner, CRYPTO_CURVE Curve, void *pKeyCtx,
								 uint8_t *pPubKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr || pPubKey == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	CryptoUecc::KeyCtx *pk = (CryptoUecc::KeyCtx *)pKeyCtx;
	UeccKeyReset(pOwner, pk);
	memset(pPubKey, 0, 64U);

	if (pRng == nullptr || !pRng->IsSecure())
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	CRYPTO_STATUS begin = UeccBegin(pRng, true);
	if (begin != CRYPTO_STATUS_OK)
	{
		return begin;
	}

	int ok = uECC_make_key(pPubKey, pk->PrivKey, uECC_secp256r1());
	CRYPTO_STATUS rngStatus = UeccEnd();
	if (ok != 1)
	{
		UeccKeyReset(pOwner, pk);
		memset(pPubKey, 0, 64U);
		return rngStatus == CRYPTO_STATUS_BUSY ? CRYPTO_STATUS_BUSY :
			CRYPTO_STATUS_FAIL;
	}
	pk->bKeyValid = true;
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS UeccKeyImport(KeyAgreeEngine *pOwner, CRYPTO_CURVE Curve, void *pKeyCtx,
									const uint8_t *pPrivKey, uint8_t *pPubKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
		pPrivKey == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	CryptoUecc::KeyCtx *pk = (CryptoUecc::KeyCtx *)pKeyCtx;
	UeccKeyReset(pOwner, pk);
	if (pPubKey != nullptr)
	{
		memset(pPubKey, 0, 64U);
	}
	if (!P256ScalarInRange(pPrivKey))
	{
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t pub[64] = {};
	CRYPTO_STATUS begin = UeccBegin(nullptr, false);
	if (begin != CRYPTO_STATUS_OK)
	{
		return begin;
	}
	int ok = UeccPublicKey(pPrivKey, pub);
	UeccEnd();
	if (ok != 1)
	{
		UeccWipe(pub, sizeof(pub));
		return CRYPTO_STATUS_FAIL;
	}

	memcpy(pk->PrivKey, pPrivKey, 32U);
	pk->bKeyValid = true;
	if (pPubKey != nullptr)
	{
		memcpy(pPubKey, pub, sizeof(pub));
	}
	UeccWipe(pub, sizeof(pub));
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS UeccAgree(RngEngine *pRng, KeyAgreeEngine *pOwner, CRYPTO_CURVE Curve, void *pKeyCtx,
								const uint8_t *pPeerPubKey,
								uint8_t *pSharedX, bool bKeepKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
		pPeerPubKey == nullptr || pSharedX == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	CryptoUecc::KeyCtx *pk = (CryptoUecc::KeyCtx *)pKeyCtx;
	if (!pk->bKeyValid)
	{
		memset(pSharedX, 0, 32U);
		return CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS begin = UeccBegin(pRng, true);
	if (begin != CRYPTO_STATUS_OK)
	{
		memset(pSharedX, 0, 32U);
		if (begin == CRYPTO_STATUS_BUSY)
		{
			return CRYPTO_STATUS_BUSY;
		}
		UeccKeyReset(pOwner, pk);
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t secret[32] = {};
	uint8_t scalar[32], negativeMask;
	const uint8_t mask = UeccPrepareScalar(pk->PrivKey, scalar, negativeMask);
	int valid = uECC_valid_public_key(pPeerPubKey, uECC_secp256r1());
	int ok = valid == 1 ?
		uECC_shared_secret(pPeerPubKey, scalar, secret,
						   uECC_secp256r1()) : 0;
	CRYPTO_STATUS rngStatus = UeccEnd();
	UeccWipe(scalar, sizeof(scalar));
	for (size_t i = 0; i < sizeof(secret); i++)
	{
		// x(-Q) = x(Q); only release this after peer/RNG validation succeeds.
		secret[i] = (secret[i] & (uint8_t)~mask) | (pPeerPubKey[i] & mask);
	}

	if (ok == 1)
	{
		memcpy(pSharedX, secret, sizeof(secret));
	}
	else
	{
		memset(pSharedX, 0, 32U);
	}
	UeccWipe(secret, sizeof(secret));

	if (ok == 1)
	{
		if (!bKeepKey)
		{
			UeccKeyReset(pOwner, pk);
		}
		return CRYPTO_STATUS_OK;
	}
	if (rngStatus == CRYPTO_STATUS_BUSY)
	{
		return CRYPTO_STATUS_BUSY;
	}
	UeccKeyReset(pOwner, pk);
	return CRYPTO_STATUS_FAIL;
}

static CRYPTO_STATUS UeccSign(RngEngine *pRng, CRYPTO_CURVE Curve, const CryptoKey &Key,
							   const uint8_t *pHash, size_t HashLen,
							   uint8_t *pSig)
{
	if (Curve != CRYPTO_CURVE_P256 || Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		Key.Type != CRYPTO_KEY_ECC_P256 ||
		(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U ||
		pHash == nullptr || pSig == nullptr || Key.Plain.pData == nullptr ||
		Key.Plain.Len != 32U)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (pRng == nullptr || !pRng->IsSecure())
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	CRYPTO_STATUS begin = UeccBegin(pRng, true);
	if (begin != CRYPTO_STATUS_OK)
	{
		return begin;
	}
	int ok = uECC_sign(Key.Plain.pData, pHash, (unsigned)HashLen, pSig,
					   uECC_secp256r1());
	CRYPTO_STATUS rngStatus = UeccEnd();
	if (ok == 1)
	{
		return CRYPTO_STATUS_OK;
	}
	memset(pSig, 0, 64U);
	return rngStatus == CRYPTO_STATUS_BUSY ? CRYPTO_STATUS_BUSY :
		CRYPTO_STATUS_FAIL;
}

// For Q = G or -G, ECDSA verification reduces to a single multiplication:
// R = ((h +/- r) / s mod n) G. Avoid micro-ecc's degenerate G + Q Shamir
// table. All arithmetic here uses public signature/hash data.
static int UeccVerifyBoundary(const uint8_t *pHash, size_t HashLen,
							 const uint8_t *pSig, bool Negative)
{
	if (!P256ScalarInRange(pSig) || !P256ScalarInRange(pSig + 32U))
	{
		return 0;
	}
	const uECC_Curve curve = uECC_secp256r1();
	const wordcount_t words = 32U / uECC_WORD_SIZE;
	const uECC_word_t *order = uECC_curve_n(curve);
	uECC_word_t h[32U / uECC_WORD_SIZE], r[32U / uECC_WORD_SIZE];
	uECC_word_t s[32U / uECC_WORD_SIZE], k[32U / uECC_WORD_SIZE];
	uint8_t hash[32] = {}, scalar[32], pub[64] = {};
	const size_t len = HashLen < sizeof(hash) ? HashLen : sizeof(hash);
	memcpy(hash + sizeof(hash) - len, pHash, len);
	uECC_vli_bytesToNative(h, hash, sizeof(hash));
	uECC_vli_bytesToNative(r, pSig, 32U);
	uECC_vli_bytesToNative(s, pSig + 32U, 32U);
	if (uECC_vli_cmp(h, order, words) >= 0)
	{
		uECC_vli_sub(h, h, order, words);
	}
	if (Negative)
	{
		uECC_vli_modSub(h, h, r, order, words);
	}
	else
	{
		uECC_vli_modAdd(h, h, r, order, words);
	}
	uECC_vli_modInv(k, s, order, words);
	uECC_vli_modMult(k, k, h, order, words);
	if (uECC_vli_isZero(k, words))
	{
		return 0; // R is the point at infinity.
	}
	uECC_vli_nativeToBytes(scalar, sizeof(scalar), k);
	if (UeccPublicKey(scalar, pub) != 1)
	{
		return 0;
	}
	uECC_vli_bytesToNative(k, pub, 32U);
	if (uECC_vli_cmp(k, order, words) >= 0)
	{
		uECC_vli_sub(k, k, order, words);
	}
	return uECC_vli_equal(k, r, words);
}

static CRYPTO_STATUS UeccVerify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
								const uint8_t *pHash, size_t HashLen,
								const uint8_t *pSig)
{
	CRYPTO_STATUS begin;
	int ok;
	bool sameX;
	bool positive;
	bool negative;

	if (Curve != CRYPTO_CURVE_P256 || pPubKey == nullptr ||
		pHash == nullptr || pSig == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	begin = UeccBegin(nullptr, false);
	if (begin != CRYPTO_STATUS_OK)
	{
		return begin;
	}
	if (uECC_valid_public_key(pPubKey, uECC_secp256r1()) != 1)
	{
		(void)UeccEnd();
		return CRYPTO_STATUS_FAIL;
	}
	sameX = memcmp(pPubKey, s_UeccGenerator, 32U) == 0;
	positive = sameX && memcmp(pPubKey + 32U, s_UeccGenerator + 32U, 32U) == 0;
	negative = sameX && memcmp(pPubKey + 32U, s_UeccNegativeGy, 32U) == 0;
	if (positive || negative)
	{
		ok = UeccVerifyBoundary(pHash, HashLen, pSig, negative);
	}
	else
	{
		ok = uECC_verify(pPubKey, pHash, (unsigned)HashLen, pSig, uECC_secp256r1());
	}
	(void)UeccEnd();
	return ok == 1 ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS CryptoUecc::Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
								 const uint8_t *pHash, size_t HashLen,
								 const uint8_t *pSig)
{
	return UeccVerify(Curve, pPubKey, pHash, HashLen, pSig);
}

CRYPTO_STATUS CryptoUeccVerify::Verify(CRYPTO_CURVE Curve,
									   const uint8_t *pPubKey,
									   const uint8_t *pHash, size_t HashLen,
									   const uint8_t *pSig)
{
	return UeccVerify(Curve, pPubKey, pHash, HashLen, pSig);
}

int CryptoUecc::SelfTest()
{
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

	if (UeccBegin(nullptr, false) != CRYPTO_STATUS_OK)
	{
		return -1;
	}
	uint8_t dh[32] = {};
	int ok = uECC_shared_secret(pubB, privA, dh, uECC_secp256r1());
	(void)UeccEnd();
	int rc = ok == 1 && memcmp(dh, dhExpect, sizeof(dh)) == 0 ? 0 : -1;
	UeccWipe(dh, sizeof(dh));
	return rc;
}

CryptoUecc *CryptoUeccCreate(void *pMem, size_t MemSize, RngEngine *pRng)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoUecc) ||
		((uintptr_t)pMem & (alignof(CryptoUecc) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoUecc *p = new (pMem) CryptoUecc();
	p->SetRng(pRng);
	p->Enable();
	return p;
}

CryptoUeccVerify *CryptoUeccVerifyCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoUeccVerify) ||
		((uintptr_t)pMem & (alignof(CryptoUeccVerify) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoUeccVerify *p = new (pMem) CryptoUeccVerify();
	p->Enable();
	return p;
}

// Generic software defaults share primitives; they do not instantiate CryptoUecc.
CRYPTO_STATUS CryptoUecc::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
								 uint8_t *pPubKey)
{
	return UeccKeyGen(vpRng, this, Curve, pKeyCtx, pPubKey);
}


CRYPTO_STATUS CryptoUecc::KeyImport(CRYPTO_CURVE Curve, void *pKeyCtx,
									const uint8_t *pPrivKey, uint8_t *pPubKey)
{
	return UeccKeyImport(this, Curve, pKeyCtx, pPrivKey, pPubKey);
}


CRYPTO_STATUS CryptoUecc::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
								const uint8_t *pPeerPubKey,
								uint8_t *pSharedX, bool bKeepKey)
{
	return UeccAgree(vpRng, this, Curve, pKeyCtx, pPeerPubKey, pSharedX, bKeepKey);
}


CRYPTO_STATUS CryptoUecc::Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
							   const uint8_t *pHash, size_t HashLen,
							   uint8_t *pSig)
{
	return UeccSign(vpRng, Curve, Key, pHash, HashLen, pSig);
}


__attribute__((weak)) CRYPTO_STATUS Crypto::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx, uint8_t *pPubKey)
{
	if (!vbValid)
	{
		if (pPubKey != nullptr && Curve == CRYPTO_CURVE_P256)
		{
			CryptoSecureWipe(pPubKey, 64U);
		}
		if (pKeyCtx != nullptr)
		{
			KeyReset(pKeyCtx);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return UeccKeyGen(this, nullptr, Curve, pKeyCtx, pPubKey);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::KeyImport(CRYPTO_CURVE Curve, void *pKeyCtx,
			const uint8_t *pPrivKey, uint8_t *pPubKey)
{
	if (!vbValid)
	{
		if (pPubKey != nullptr && Curve == CRYPTO_CURVE_P256)
		{
			CryptoSecureWipe(pPubKey, 64U);
		}
		if (pKeyCtx != nullptr)
		{
			KeyReset(pKeyCtx);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return UeccKeyImport(nullptr, Curve, pKeyCtx, pPrivKey, pPubKey);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
			const uint8_t *pPeerPubKey, uint8_t *pSharedX, bool bKeepKey)
{
	if (!vbValid)
	{
		if (pSharedX != nullptr && Curve == CRYPTO_CURVE_P256)
		{
			CryptoSecureWipe(pSharedX, 32U);
		}
		if (pKeyCtx != nullptr)
		{
			KeyReset(pKeyCtx);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return UeccAgree(this, nullptr, Curve, pKeyCtx, pPeerPubKey, pSharedX, bKeepKey);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
			const uint8_t *pHash, size_t HashLen, uint8_t *pSig)
{
	if (!vbValid)
	{
		if (pSig != nullptr && Curve == CRYPTO_CURVE_P256)
		{
			CryptoSecureWipe(pSig, 64U);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return UeccSign(this, Curve, Key, pHash, HashLen, pSig);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
			const uint8_t *pHash, size_t HashLen, const uint8_t *pSig)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return UeccVerify(Curve, pPubKey, pHash, HashLen, pSig);
}

__attribute__((weak)) void Crypto::KeyReset(void *pKeyCtx)
{
	UeccKeyReset(nullptr, pKeyCtx);
}

__attribute__((weak)) size_t Crypto::KeyCtxSize() const
{
	if (!vbValid)
	{
		return 0U;
	}
	return sizeof(CryptoUecc::KeyCtx);
}

__attribute__((weak)) size_t Crypto::KeyCtxAlign() const
{
	if (!vbValid)
	{
		return 1U;
	}
	return alignof(CryptoUecc::KeyCtx);
}
