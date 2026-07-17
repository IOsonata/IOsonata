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
#include <stdatomic.h>
#include <new>

#include "crypto/crypto_uecc.h"
#include "uECC.h"

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

static atomic_flag s_UeccBusy = ATOMIC_FLAG_INIT;
static RngEngine *s_pActiveRng;
static uECC_RNG_Function s_PreviousRng;

static int UeccRngAdapter(uint8_t *pDest, unsigned Size)
{
	return s_pActiveRng != nullptr &&
		s_pActiveRng->Random(pDest, (size_t)Size) == CRYPTO_STATUS_OK ? 1 : 0;
}

static CRYPTO_STATUS UeccBegin(RngEngine *pRng, bool bNeedRng)
{
	if (atomic_flag_test_and_set(&s_UeccBusy))
	{
		return CRYPTO_STATUS_BUSY;
	}

	s_PreviousRng = uECC_get_rng();
	if (bNeedRng && (pRng == nullptr || !pRng->IsSecure()))
	{
		s_PreviousRng = nullptr;
		atomic_flag_clear(&s_UeccBusy);
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	s_pActiveRng = bNeedRng ? pRng : nullptr;
	uECC_set_rng(bNeedRng ? UeccRngAdapter : nullptr);
	return CRYPTO_STATUS_OK;
}

static void UeccEnd(void)
{
	uECC_set_rng(s_PreviousRng);
	s_PreviousRng = nullptr;
	s_pActiveRng = nullptr;
	atomic_flag_clear(&s_UeccBusy);
}

void CryptoUecc::KeyReset(void *pKeyCtx)
{
	if (pKeyCtx != nullptr)
	{
		UeccWipe(pKeyCtx, sizeof(KeyCtx));
	}
}

CRYPTO_STATUS CryptoUecc::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
								 uint8_t *pPubKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr || pPubKey == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	KeyCtx *pk = (KeyCtx *)pKeyCtx;
	KeyReset(pk);
	memset(pPubKey, 0, 64U);

	if (vpRng == nullptr || !vpRng->IsSecure())
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	CRYPTO_STATUS begin = UeccBegin(vpRng, true);
	if (begin != CRYPTO_STATUS_OK)
	{
		return begin;
	}

	int ok = uECC_make_key(pPubKey, pk->PrivKey, uECC_secp256r1());
	UeccEnd();
	if (ok != 1)
	{
		KeyReset(pk);
		memset(pPubKey, 0, 64U);
		return CRYPTO_STATUS_FAIL;
	}
	pk->bKeyValid = true;
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoUecc::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
								const uint8_t *pPeerPubKey,
								uint8_t *pSharedX, bool bKeepKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
		pPeerPubKey == nullptr || pSharedX == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	KeyCtx *pk = (KeyCtx *)pKeyCtx;
	if (!pk->bKeyValid)
	{
		memset(pSharedX, 0, 32U);
		return CRYPTO_STATUS_FAIL;
	}
	// micro-ecc applies its random initial-Z point randomization only while an
	// RNG callback is bound. Bind the engine RNG so the multiply over the
	// private scalar runs with that protection; fail closed without it.
	CRYPTO_STATUS begin = UeccBegin(vpRng, true);
	if (begin != CRYPTO_STATUS_OK)
	{
		if (begin != CRYPTO_STATUS_BUSY)
		{
			KeyReset(pk);
		}
		memset(pSharedX, 0, 32U);
		return begin;
	}

	uint8_t secret[32] = {};
	int valid = uECC_valid_public_key(pPeerPubKey, uECC_secp256r1());
	int ok = valid == 1 ?
		uECC_shared_secret(pPeerPubKey, pk->PrivKey, secret,
						   uECC_secp256r1()) : 0;
	UeccEnd();

	if (ok == 1)
	{
		memcpy(pSharedX, secret, sizeof(secret));
	}
	else
	{
		memset(pSharedX, 0, 32U);
	}
	UeccWipe(secret, sizeof(secret));

	if (!bKeepKey || ok != 1)
	{
		KeyReset(pk);
	}
	return ok == 1 ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS CryptoUecc::Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
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
	if (vpRng == nullptr || !vpRng->IsSecure())
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	CRYPTO_STATUS begin = UeccBegin(vpRng, true);
	if (begin != CRYPTO_STATUS_OK)
	{
		return begin;
	}
	int ok = uECC_sign(Key.Plain.pData, pHash, (unsigned)HashLen, pSig,
					   uECC_secp256r1());
	UeccEnd();
	return ok == 1 ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS CryptoUecc::Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
								 const uint8_t *pHash, size_t HashLen,
								 const uint8_t *pSig)
{
	if (Curve != CRYPTO_CURVE_P256 || pPubKey == nullptr ||
		pHash == nullptr || pSig == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	CRYPTO_STATUS begin = UeccBegin(nullptr, false);
	if (begin != CRYPTO_STATUS_OK)
	{
		return begin;
	}
	int ok = uECC_verify(pPubKey, pHash, (unsigned)HashLen, pSig,
						 uECC_secp256r1());
	UeccEnd();
	return ok == 1 ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
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
	UeccEnd();
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
