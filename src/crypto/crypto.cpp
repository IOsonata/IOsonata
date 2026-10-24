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

Crypto::Crypto()
{
	CryptoUecc::SetRng(this);
}

bool Crypto::Enable()
{
	return Init();
}

void Crypto::Disable()
{
	vbValid = false;
}

void Crypto::Reset()
{
	Disable();
	CryptoSoftRng::Reset();
	(void)Init();
}

int Crypto::SelfTest()
{
	if (!vbValid)
	{
		return -1;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->SelfTest();
	}
	if (CryptoSoftAes::SelfTest() != 0 || CryptoSoftSha256::SelfTest() != 0 ||
		CryptoUecc::SelfTest() != 0)
	{
		return -1;
	}
	return 0;
}

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
	if (vpHardware != nullptr)
	{
		return vpHardware->Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
	}
	return CryptoSoftAes::Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
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
	if (vpHardware != nullptr)
	{
		return vpHardware->Seal(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
	}
	return CryptoSoftAes::Seal(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
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
	if (vpHardware != nullptr)
	{
		return vpHardware->Open(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
	}
	return CryptoSoftAes::Open(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
}

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
	if (vpHardware != nullptr)
	{
		return vpHardware->Hash(Alg, pMsg, Len, pDigest);
	}
	return CryptoSoftSha256::Hash(Alg, pMsg, Len, pDigest);
}

size_t Crypto::KeyCtxSize() const
{
	if (!vbValid)
	{
		return 0U;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->KeyCtxSize();
	}
	return CryptoUecc::KeyCtxSize();
}

size_t Crypto::KeyCtxAlign() const
{
	if (!vbValid)
	{
		return 1U;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->KeyCtxAlign();
	}
	return CryptoUecc::KeyCtxAlign();
}

void Crypto::KeyReset(void *pKeyCtx)
{
	if (vpHardware != nullptr)
	{
		vpHardware->KeyReset(pKeyCtx);
		return;
	}
	CryptoUecc::KeyReset(pKeyCtx);
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
	if (vpHardware != nullptr)
	{
		return vpHardware->KeyGen(Curve, pKeyCtx, pPubKey);
	}
	return CryptoUecc::KeyGen(Curve, pKeyCtx, pPubKey);
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
	if (vpHardware != nullptr)
	{
		return vpHardware->KeyImport(Curve, pKeyCtx, pPrivKey, pPubKey);
	}
	return CryptoUecc::KeyImport(Curve, pKeyCtx, pPrivKey, pPubKey);
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
	if (vpHardware != nullptr)
	{
		return vpHardware->Agree(Curve, pKeyCtx, pPeerPubKey, pSharedX, bKeepKey);
	}
	return CryptoUecc::Agree(Curve, pKeyCtx, pPeerPubKey, pSharedX, bKeepKey);
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
	if (vpHardware != nullptr)
	{
		return vpHardware->Sign(Curve, Key, pHash, HashLen, pSig);
	}
	return CryptoUecc::Sign(Curve, Key, pHash, HashLen, pSig);
}

CRYPTO_STATUS Crypto::Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
			const uint8_t *pHash, size_t HashLen, const uint8_t *pSig)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->Verify(Curve, pPubKey, pHash, HashLen, pSig);
	}
	return CryptoUecc::Verify(Curve, pPubKey, pHash, HashLen, pSig);
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
	if (vpHardware != nullptr)
	{
		return vpHardware->Mac(Alg, Key, pMsg, Len, pMac, MacLen);
	}
	if (Alg == CRYPTO_MAC_HMAC)
	{
		return CryptoSoftSha256::Mac(Alg, Key, pMsg, Len, pMac, MacLen);
	}
	return CryptoSoftAes::Mac(Alg, Key, pMsg, Len, pMac, MacLen);
}

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
	if (vpHardware != nullptr)
	{
		return vpHardware->Random(pOut, Len);
	}
	return CryptoSoftRng::Random(pOut, Len);
}

bool Crypto::IsSecure() const
{
	if (!vbValid)
	{
		return false;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->IsSecure();
	}
	return CryptoSoftRng::IsSecure();
}

size_t Crypto::HashCtxSize() const
{
	if (!vbValid)
	{
		return 0U;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->HashCtxSize();
	}
	return CryptoSoftSha256::HashCtxSize();
}

size_t Crypto::HashCtxAlign() const
{
	if (!vbValid)
	{
		return 1U;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->HashCtxAlign();
	}
	return CryptoSoftSha256::HashCtxAlign();
}

CRYPTO_STATUS Crypto::HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->HashInit(Alg, pHashCtx);
	}
	return CryptoSoftSha256::HashInit(Alg, pHashCtx);
}

CRYPTO_STATUS Crypto::HashUpdate(void *pHashCtx, const uint8_t *pMsg, size_t Len)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (vpHardware != nullptr)
	{
		return vpHardware->HashUpdate(pHashCtx, pMsg, Len);
	}
	return CryptoSoftSha256::HashUpdate(pHashCtx, pMsg, Len);
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
	if (vpHardware != nullptr)
	{
		return vpHardware->HashFinal(pHashCtx, pDigest);
	}
	return CryptoSoftSha256::HashFinal(pHashCtx, pDigest);
}

