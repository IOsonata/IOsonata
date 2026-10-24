/**-------------------------------------------------------------------------
@file	crypto_default_stm32.cpp
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
#include "stm32.h"
#if defined(STM32WBA65xx)
#include "crypto/crypto_stm32wba.h"
#define CRYPTO_TARGET_AES CryptoWbaAes
#define CRYPTO_TARGET_PKA CryptoWbaPka
#elif (defined(STM32H753xx) || defined(STM32L4S9xx))
#include "crypto_stm32h7.h"
#define CRYPTO_TARGET_AES CryptoStm32H7Aes
#endif
#if IOSONATA_STM32_HAS_RNG && !defined(STM32WBA65xx)
#include "crypto_rng_stm32.h"
#endif

#include "crypto/crypto_softsha256.h"

#if IOSONATA_STM32_HAS_RNG || defined(STM32WBA65xx)
static RngEngine *s_Rng;
#endif
#ifdef CRYPTO_TARGET_AES
static CRYPTO_TARGET_AES s_Aes;
#endif
#ifdef CRYPTO_TARGET_PKA
static CRYPTO_TARGET_PKA s_Pka;
static CryptoWbaRng s_WbaRng;
#endif
#if defined(STM32H753xx) || defined(STM32L4S9xx)
static CryptoStm32H7Hash s_Hash;
#endif

bool Crypto::Init()
{
	if (vbValid)
	{
		return true;
	}
#if defined(STM32WBA65xx)
	s_Rng = &s_WbaRng;
	if (!s_WbaRng.Enable())
	{
		return false;
	}
	s_Pka.SetRng(s_Rng);
	if (!s_Pka.Enable())
	{
		return false;
	}
#elif IOSONATA_STM32_HAS_RNG
	s_Rng = CryptoRngStm32Instance();
	if (s_Rng == nullptr)
	{
		return false;
	}
#endif
#ifdef CRYPTO_TARGET_AES
	if (!s_Aes.Enable())
	{
		return false;
	}
#endif
#if defined(STM32H753xx) || defined(STM32L4S9xx)
	if (!s_Hash.Enable())
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
#if defined(STM32H753xx) || defined(STM32L4S9xx)
		return s_Hash.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
#else
		return CryptoHmacSha256(this, Alg, Key, pMsg, Len, pMac, MacLen);
#endif
	}
	return s_Aes.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
}

#endif

#if defined(STM32H753xx) || defined(STM32L4S9xx)
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
	return s_Hash.Hash(Alg, pMsg, Len, pDigest);
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

CRYPTO_STATUS Crypto::Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
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
	return s_Pka.Sign(Curve, Key, pHash, HashLen, pSig);
}

CRYPTO_STATUS Crypto::Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
			const uint8_t *pHash, size_t HashLen, const uint8_t *pSig)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return s_Pka.Verify(Curve, pPubKey, pHash, HashLen, pSig);
}

#endif

#if IOSONATA_STM32_HAS_RNG || defined(STM32WBA65xx)
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

#endif

