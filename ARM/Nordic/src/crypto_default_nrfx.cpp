/**-------------------------------------------------------------------------
@file	crypto_default_nrfx.cpp
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
#include "nrf.h"
#include "crypto_rng_nrf.h"
#if defined(NRF54H20_XXAA)
// No crypto hardware or RNG on the nRF54H20 application and radio cores. The
// nRF54H20 libraries do not link this file and use the software Crypto
// implementation in src/crypto/crypto.cpp.
#error "crypto_default_nrfx.cpp: no crypto hardware on the nRF54H20 application and radio cores"
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#include "cracen_intrf.h"
#include "crypto/cryptomaster.h"
#include "crypto/ba414ep.h"
#define CRYPTO_TARGET_AES CryptoMaster
#define CRYPTO_TARGET_PKA Ba414ep
#elif defined(NRF52840_XXAA) || defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)
#include "crypto_cc3xx_sym.h"
#define CRYPTO_TARGET_AES CryptoCc3xxSym
#define CRYPTO_TARGET_CC_HASH
#elif defined(NRF5340_XXAA_NETWORK) || defined(NRF52_SERIES) || \
	defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || \
	defined(NRF52832_XXAA) || defined(NRF52840_XXAA)
#include "crypto_ecb_nrf.h"
#define CRYPTO_TARGET_AES CryptoEcbNrf
#endif
#if defined(NRF52840_XXAA) || defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)
#include "crypto_cc3xx.h"
#define CRYPTO_TARGET_PKA CryptoCc3xx
#endif

#include "crypto/crypto_softsha256.h"

static RngEngine *s_Rng;
#ifdef CRYPTO_TARGET_AES
static CRYPTO_TARGET_AES s_Aes;
#endif
#ifdef CRYPTO_TARGET_PKA
static CRYPTO_TARGET_PKA s_Pka;
#endif

bool Crypto::Init()
{
	if (vbValid)
	{
		return true;
	}
	s_Rng = CryptoRngNrfInstance();
	if (s_Rng == nullptr)
	{
		return false;
	}
#ifdef CRYPTO_TARGET_AES
#if defined(CRYPTO_TARGET_CC_HASH)
	if (!s_Aes.Init(Cc3xxIntrfInstance()))
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	if (!s_Aes.Init(CracenIntrfInstance()))
#else
	if (!s_Aes.Enable())
#endif
	{
		return false;
	}
#endif
#ifdef CRYPTO_TARGET_PKA
#ifdef CRYPTO_TARGET_CC_HASH
	if (!s_Pka.Init(Cc3xxIntrfInstance(), s_Rng))
#else
	if (!s_Pka.Init(CracenIntrfInstance(), s_Rng))
#endif
	{
		return false;
	}
#endif
	vbValid = true;
	return true;
}

#if defined(CRYPTO_TARGET_AES)
CRYPTO_STATUS Crypto::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
			const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return s_Aes.Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
}

CRYPTO_STATUS Crypto::Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen)
{
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
	return s_Aes.Seal(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
}

CRYPTO_STATUS Crypto::Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return s_Aes.Open(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
}

CRYPTO_STATUS Crypto::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
			const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen)
{
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
	return s_Aes.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
}

#endif

#if defined(CRYPTO_TARGET_CC_HASH)
CRYPTO_STATUS Crypto::Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len, uint8_t *pDigest)
{
	if (!vbValid)
	{
		if (pDigest != nullptr && Alg == CRYPTO_HASH_SHA256)
		{
			CryptoSecureWipe(pDigest, 32U);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return s_Aes.Hash(Alg, pMsg, Len, pDigest);
}

size_t Crypto::HashCtxSize() const
{
	if (!vbValid)
	{
		return 0U;
	}
	return s_Aes.HashCtxSize();
}

size_t Crypto::HashCtxAlign() const
{
	if (!vbValid)
	{
		return 1U;
	}
	return s_Aes.HashCtxAlign();
}

CRYPTO_STATUS Crypto::HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return s_Aes.HashInit(Alg, pHashCtx);
}

CRYPTO_STATUS Crypto::HashUpdate(void *pHashCtx, const uint8_t *pMsg, size_t Len)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return s_Aes.HashUpdate(pHashCtx, pMsg, Len);
}

CRYPTO_STATUS Crypto::HashFinal(void *pHashCtx, uint8_t *pDigest)
{
	if (!vbValid)
	{
		if (pDigest != nullptr)
		{
			CryptoSecureWipe(pDigest, 32U);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return s_Aes.HashFinal(pHashCtx, pDigest);
}

#endif

#if defined(CRYPTO_TARGET_PKA)
size_t Crypto::KeyCtxSize() const
{
	if (!vbValid)
	{
		return 0U;
	}
	return s_Pka.KeyCtxSize();
}

size_t Crypto::KeyCtxAlign() const
{
	if (!vbValid)
	{
		return 1U;
	}
	return s_Pka.KeyCtxAlign();
}

void Crypto::KeyReset(void *pKeyCtx)
{
	s_Pka.KeyReset(pKeyCtx);
}

CRYPTO_STATUS Crypto::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx, uint8_t *pPubKey)
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
	return s_Pka.KeyGen(Curve, pKeyCtx, pPubKey);
}

CRYPTO_STATUS Crypto::KeyImport(CRYPTO_CURVE Curve, void *pKeyCtx,
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
	return s_Pka.KeyImport(Curve, pKeyCtx, pPrivKey, pPubKey);
}

CRYPTO_STATUS Crypto::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
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
	return s_Pka.Agree(Curve, pKeyCtx, pPeerPubKey, pSharedX, bKeepKey);
}

#endif

CRYPTO_STATUS Crypto::Random(uint8_t *pOut, size_t Len)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return (*s_Rng).Random(pOut, Len);
}

bool Crypto::IsSecure() const
{
	if (!vbValid)
	{
		return false;
	}
	return (*s_Rng).IsSecure();
}

