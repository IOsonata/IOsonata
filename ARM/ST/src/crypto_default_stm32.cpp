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
#elif defined(STM32H753xx)
#include "crypto_stm32h7.h"
#define CRYPTO_TARGET_AES CryptoStm32H7Aes
#endif
#if IOSONATA_STM32_HAS_RNG && !defined(STM32WBA65xx)
#include "crypto_rng_stm32.h"
#endif

// Library-owned hardware implementation. Application Crypto objects borrow it;
// their Disable and destruction do not stop other users of these peripherals.
class CryptoStm32 : public Crypto {
public:
	bool Enable() override
	{
		if (vbValid)
		{
			return true;
		}
#if defined(STM32WBA65xx)
		vpRng = &vRng;
		vbRngStarted = true;
		if (!vRng.Enable())
		{
			return false;
		}
		vPka.SetRng(this);
		vbPkaStarted = true;
		if (!vPka.Enable())
		{
			return false;
		}
#elif IOSONATA_STM32_HAS_RNG
		vpRng = CryptoRngStm32Instance();
		if (vpRng == nullptr)
		{
			return false;
		}
#endif
#ifdef CRYPTO_TARGET_AES
		if (!vAes.Enable())
		{
			return false;
		}
#endif
#if defined(STM32H753xx)
		if (!vHash.Enable())
		{
			return false;
		}
#endif
		vbValid = true;
		return true;
	}

	void Disable() override
	{
#ifdef CRYPTO_TARGET_AES
		if (vAes.Valid())
		{
			vAes.Disable();
		}
#endif
#ifdef CRYPTO_TARGET_PKA
		if (vbPkaStarted)
		{
			vPka.Disable();
			vbPkaStarted = false;
		}
#endif
#if defined(STM32H753xx)
		if (vHash.Valid())
		{
			vHash.Disable();
		}
#elif defined(STM32WBA65xx)
		if (vbRngStarted)
		{
			vRng.Disable();
			vbRngStarted = false;
		}
#endif
		// Platform RNG singletons may be shared with other devices.
		vpRng = nullptr;
		Crypto::Disable();
	}

	void Reset() override
	{
		Disable();
		CryptoSoftRng::Reset();
		(void)Enable();
	}

	int SelfTest() override
	{
		if (!vbValid || Crypto::SelfTest() != 0)
		{
			return -1;
		}
#ifdef CRYPTO_TARGET_AES
		if (vAes.SelfTest() != 0)
		{
			return -1;
		}
#endif
#ifdef CRYPTO_TARGET_PKA
		if (vPka.SelfTest() != 0)
		{
			return -1;
		}
#endif
		return 0;
	}

	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override
	{
		if (vpRng != nullptr)
		{
			return vpRng->Random(pOut, Len);
		}
#if IOSONATA_STM32_HAS_RNG || defined(STM32WBA65xx)
		return CRYPTO_STATUS_FAIL;
#else
		return Crypto::Random(pOut, Len);
#endif
	}

	bool IsSecure() const override
	{
		return vpRng != nullptr && vpRng->IsSecure();
	}

#ifdef CRYPTO_TARGET_AES
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
			const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut) override
	{
		return vAes.Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
	}

	CRYPTO_STATUS Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen) override
	{
		return vAes.Seal(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
	}

	CRYPTO_STATUS Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen) override
	{
		return vAes.Open(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
	}

#endif
	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
			const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen) override
	{
#if defined(STM32H753xx)
		if (Alg == CRYPTO_MAC_HMAC)
		{
			return vHash.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
		}
#endif
#ifdef CRYPTO_TARGET_AES
		if (Alg == CRYPTO_MAC_CMAC)
		{
			return vAes.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
		}
#endif
		return Crypto::Mac(Alg, Key, pMsg, Len, pMac, MacLen);
	}

#if defined(STM32H753xx)
	CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len, uint8_t *pDigest) override
	{
		return vHash.Hash(Alg, pMsg, Len, pDigest);
	}

#endif
#ifdef CRYPTO_TARGET_PKA
	size_t KeyCtxSize() const override
	{
		return vPka.KeyCtxSize();
	}

	size_t KeyCtxAlign() const override
	{
		return vPka.KeyCtxAlign();
	}

	void KeyReset(void *pKeyCtx) override
	{
		vPka.KeyReset(pKeyCtx);
	}

	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx, uint8_t *pPubKey) override
	{
		return vPka.KeyGen(Curve, pKeyCtx, pPubKey);
	}

	CRYPTO_STATUS KeyImport(CRYPTO_CURVE Curve, void *pKeyCtx,
			const uint8_t *pPrivKey, uint8_t *pPubKey) override
	{
		return vPka.KeyImport(Curve, pKeyCtx, pPrivKey, pPubKey);
	}

	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
			const uint8_t *pPeerPubKey, uint8_t *pSharedX, bool bKeepKey = false) override
	{
		return vPka.Agree(Curve, pKeyCtx, pPeerPubKey, pSharedX, bKeepKey);
	}

#endif
#if defined(STM32WBA65xx)
	CRYPTO_STATUS Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
			const uint8_t *pHash, size_t HashLen, uint8_t *pSig) override
	{
		return vPka.Sign(Curve, Key, pHash, HashLen, pSig);
	}

	CRYPTO_STATUS Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
			const uint8_t *pHash, size_t HashLen, const uint8_t *pSig) override
	{
		return vPka.Verify(Curve, pPubKey, pHash, HashLen, pSig);
	}

#endif
private:
	RngEngine *vpRng = nullptr;
#ifdef CRYPTO_TARGET_AES
	CRYPTO_TARGET_AES vAes;
#endif
#ifdef CRYPTO_TARGET_PKA
	CRYPTO_TARGET_PKA vPka;
#endif
#if defined(STM32H753xx)
	CryptoStm32H7Hash vHash;
#elif defined(STM32WBA65xx)
	CryptoWbaRng vRng;
	bool vbRngStarted = false;
	bool vbPkaStarted = false;
#endif
};

bool Crypto::Init()
{
#if defined(CRYPTO_TARGET_AES) || IOSONATA_STM32_HAS_RNG
	static CryptoStm32 hardware;

	if (vbValid)
	{
		return true;
	}
	if (!hardware.Enable())
	{
		hardware.Disable();
		return false;
	}
	vpHardware = &hardware;
#else
	vpHardware = nullptr;
#endif
	vbValid = true;
	return true;
}
