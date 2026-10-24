/**-------------------------------------------------------------------------
@file	crypto.cpp
@brief	Default crypto engines selected by the target library.

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
#include "crypto/crypto.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_softrng.h"
#include "crypto/crypto_uecc.h"
#include <string.h>

Crypto::Crypto()
{
	vbValid = false;
	Seed(0U, 0U);
}

__attribute__((weak)) bool Crypto::Enable()
{
	return Init();
}

__attribute__((weak)) void Crypto::Disable()
{
	vbValid = false;
}

__attribute__((weak)) void Crypto::Reset()
{
	Disable();
	Seed(0U, 0U);
	(void)Init();
}

__attribute__((weak)) int Crypto::SelfTest()
{
	static const uint8_t aes[16] = {0x66,0xE9,0x4B,0xD4,0xEF,0x8A,0x2C,0x3B,0x88,0x4C,0xFA,0x59,0xCA,0x34,0x2B,0x2E};
	static const uint8_t sha[32] = {0xE3,0xB0,0xC4,0x42,0x98,0xFC,0x1C,0x14,0x9A,0xFB,0xF4,0xC8,0x99,0x6F,0xB9,0x24,0x27,0xAE,0x41,0xE4,0x64,0x9B,0x93,0x4C,0xA4,0x95,0x99,0x1B,0x78,0x52,0xB8,0x55};
	uint8_t zero[16] = {};
	uint8_t out[32];
	CryptoKey key = {};
	bool ok;
	if (!vbValid)
	{
		return -1;
	}
	key.Type = CRYPTO_KEY_AES_128;
	key.Loc = CRYPTO_KEY_LOC_PLAIN;
	key.Usage = CRYPTO_KEY_USE_ENCRYPT;
	key.Plain.pData = zero;
	key.Plain.Len = sizeof(zero);
	ok = Cipher(CRYPTO_CIPHER_ECB, 1, key, nullptr, 0, zero, 16, out) == CRYPTO_STATUS_OK &&
		memcmp(out, aes, 16) == 0 && Hash(CRYPTO_HASH_SHA256, nullptr, 0, out) == CRYPTO_STATUS_OK &&
		memcmp(out, sha, 32) == 0;
	CryptoSecureWipe(out, sizeof(out));
	return ok ? 0 : -1;
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
			const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	CryptoSoftAes software;

	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	return software.Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen)
{
	CryptoSoftAes software;

	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		if (pTag != nullptr)
		{
			CryptoSecureWipe(pTag, TagLen);
		}
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	return software.Seal(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen)
{
	CryptoSoftAes software;

	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	return software.Open(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
}

// The hash-only object avoids retaining the separate SHA/HMAC self-test and
// multiple-inheritance tables in firmware that uses the generic Crypto object.
__attribute__((weak)) CRYPTO_STATUS Crypto::Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len, uint8_t *pDigest)
{
	CryptoSoftSha256Hash software;

	if (!vbValid)
	{
		if (pDigest != nullptr && Alg == CRYPTO_HASH_SHA256)
		{
			CryptoSecureWipe(pDigest, 32U);
		}
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	return software.Hash(Alg, pMsg, Len, pDigest);
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

__attribute__((weak)) void Crypto::KeyReset(void *pKeyCtx)
{
	CryptoUecc software;

	software.KeyReset(pKeyCtx);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx, uint8_t *pPubKey)
{
	CryptoUecc software;

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
	software.Enable();
	software.SetRng(this);
	return software.KeyGen(Curve, pKeyCtx, pPubKey);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::KeyImport(CRYPTO_CURVE Curve, void *pKeyCtx,
			const uint8_t *pPrivKey, uint8_t *pPubKey)
{
	CryptoUecc software;

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
	software.Enable();
	software.SetRng(this);
	return software.KeyImport(Curve, pKeyCtx, pPrivKey, pPubKey);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
			const uint8_t *pPeerPubKey, uint8_t *pSharedX, bool bKeepKey)
{
	CryptoUecc software;

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
	software.Enable();
	software.SetRng(this);
	return software.Agree(Curve, pKeyCtx, pPeerPubKey, pSharedX, bKeepKey);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
			const uint8_t *pHash, size_t HashLen, uint8_t *pSig)
{
	CryptoUecc software;

	if (!vbValid)
	{
		if (pSig != nullptr && Curve == CRYPTO_CURVE_P256)
		{
			CryptoSecureWipe(pSig, 64U);
		}
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	software.SetRng(this);
	return software.Sign(Curve, Key, pHash, HashLen, pSig);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
			const uint8_t *pHash, size_t HashLen, const uint8_t *pSig)
{
	CryptoUecc software;

	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	software.SetRng(this);
	return software.Verify(Curve, pPubKey, pHash, HashLen, pSig);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
			const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen)
{
	CryptoSoftAes software;

	if (!vbValid)
	{
		if (pMac != nullptr)
		{
			CryptoSecureWipe(pMac, MacLen);
		}
		return CRYPTO_STATUS_FAIL;
	}
	if (Alg == CRYPTO_MAC_HMAC)
	{
		return CryptoHmacSha256(this, Alg, Key, pMsg, Len, pMac, MacLen);
	}
	software.Enable();
	return software.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::Random(uint8_t *pOut, size_t Len)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return CryptoSoftRngGenerate(vSoftState[0], vSoftState[1], pOut, Len);
}

__attribute__((weak)) bool Crypto::IsSecure() const
{
	if (!vbValid)
	{
		return false;
	}
	return false;
}

__attribute__((weak)) size_t Crypto::HashCtxSize() const
{
	CryptoSoftSha256Hash software;

	if (!vbValid)
	{
		return 0U;
	}
	return software.HashCtxSize();
}

__attribute__((weak)) size_t Crypto::HashCtxAlign() const
{
	CryptoSoftSha256Hash software;

	if (!vbValid)
	{
		return 1U;
	}
	return software.HashCtxAlign();
}

__attribute__((weak)) CRYPTO_STATUS Crypto::HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx)
{
	CryptoSoftSha256Hash software;

	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	return software.HashInit(Alg, pHashCtx);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::HashUpdate(void *pHashCtx, const uint8_t *pMsg, size_t Len)
{
	CryptoSoftSha256Hash software;

	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	return software.HashUpdate(pHashCtx, pMsg, Len);
}

__attribute__((weak)) CRYPTO_STATUS Crypto::HashFinal(void *pHashCtx, uint8_t *pDigest)
{
	CryptoSoftSha256Hash software;

	if (!vbValid)
	{
		if (pDigest != nullptr)
		{
			CryptoSecureWipe(pDigest, 32U);
		}
		return CRYPTO_STATUS_FAIL;
	}
	software.Enable();
	return software.HashFinal(pHashCtx, pDigest);
}


void Crypto::Seed(uint64_t Seed0, uint64_t Seed1)
{
	vSoftState[0] = Seed0;
	vSoftState[1] = Seed1;
	if ((Seed0 | Seed1) == 0U)
	{
		vSoftState[0] = 0x9E3779B97F4A7C15ULL;
		vSoftState[1] = 0xBF58476D1CE4E5B9ULL;
	}
}
