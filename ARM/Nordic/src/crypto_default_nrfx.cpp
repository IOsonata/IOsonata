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
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA) || \
	defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#include "cracen_intrf.h"
#include "crypto/cryptomaster.h"
#include "crypto/ba414ep.h"
#define CRYPTO_TARGET_AES CryptoMaster
#define CRYPTO_TARGET_PKA Ba414ep
#elif defined(NRF5340_XXAA_NETWORK) || defined(NRF52_SERIES) || \
	defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || \
	defined(NRF52832_XXAA) || defined(NRF52840_XXAA)
#include "../nRF53/include/crypto_ecb_nrf53.h"
#define CRYPTO_TARGET_AES CryptoEcbNrf53
#endif
#if defined(NRF52840_XXAA) || defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)
#include "crypto_cc3xx.h"
#define CRYPTO_TARGET_PKA CryptoCc3xx
#endif

// Library-owned hardware implementation. Application Crypto objects borrow it;
// their Disable and destruction do not stop other users of these peripherals.
class CryptoNordic : public Crypto {
public:
	bool Enable() override
	{
		if (vbValid)
		{
			return true;
		}
		vpRng = CryptoRngNrfInstance();
		if (vpRng == nullptr)
		{
			return false;
		}
#ifdef CRYPTO_TARGET_AES
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA) || \
	defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		if (!vAes.Init(CracenIntrfInstance()))
#else
		if (!vAes.Enable())
#endif
		{
			return false;
		}
#endif
#ifdef CRYPTO_TARGET_PKA
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA) || \
	defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		if (!vPka.Init(CracenIntrfInstance(), this))
#else
		if (!vPka.Init(Cc3xxIntrfInstance(), this))
#endif
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
		vAes.Disable();
#endif
#ifdef CRYPTO_TARGET_PKA
		vPka.Disable();
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
		return CRYPTO_STATUS_FAIL;
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
#ifdef CRYPTO_TARGET_AES
		if (Alg == CRYPTO_MAC_CMAC)
		{
			return vAes.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
		}
#endif
		return Crypto::Mac(Alg, Key, pMsg, Len, pMac, MacLen);
	}

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
private:
	RngEngine *vpRng = nullptr;
#ifdef CRYPTO_TARGET_AES
	CRYPTO_TARGET_AES vAes;
#endif
#ifdef CRYPTO_TARGET_PKA
	CRYPTO_TARGET_PKA vPka;
#endif
};

bool Crypto::Init()
{
	static CryptoNordic hardware;

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
	vbValid = true;
	return true;
}
