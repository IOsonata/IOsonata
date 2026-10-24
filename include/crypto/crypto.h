/**-------------------------------------------------------------------------
@file	crypto.h
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
#ifndef __CRYPTO_H__
#define __CRYPTO_H__

#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_softrng.h"
#include "crypto/crypto_uecc.h"

/// Directly declared crypto device, for example Crypto g_Crypto.
/// Construction initializes software state only. Call Init() after system
/// startup to bind the MCU library implementation; no allocation is performed.
/// Hardware engines are shared inside that library. Disable affects this
/// object's access only; peripheral recovery and locking remain in the drivers.
/// Keys and hash contexts are caller-owned. Initialize before sizing contexts,
/// and finish operations before Disable, Reset or destruction.
class Crypto : public CryptoSoftAes, public CryptoSoftSha256,
			   public CryptoUecc, public CryptoSoftRng {
public:
	Crypto();
	Crypto(const Crypto &) = delete;
	Crypto &operator=(const Crypto &) = delete;

	/// Bind and enable this object. False on hardware initialization failure;
	/// a failed hardware path never selects software instead. Startup calls
	/// must be serialized, before concurrent operation access begins.
	bool Init();
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	int SelfTest() override;
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
			const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut) override;
	CRYPTO_STATUS Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen) override;
	CRYPTO_STATUS Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen) override;
	CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len, uint8_t *pDigest) override;
	size_t KeyCtxSize() const override;
	size_t KeyCtxAlign() const override;
	void KeyReset(void *pKeyCtx) override;
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx, uint8_t *pPubKey) override;
	CRYPTO_STATUS KeyImport(CRYPTO_CURVE Curve, void *pKeyCtx,
			const uint8_t *pPrivKey, uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
			const uint8_t *pPeerPubKey, uint8_t *pSharedX, bool bKeepKey = false) override;
	CRYPTO_STATUS Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
			const uint8_t *pHash, size_t HashLen, uint8_t *pSig) override;
	CRYPTO_STATUS Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
			const uint8_t *pHash, size_t HashLen, const uint8_t *pSig) override;
	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
			const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen) override;
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;
	bool IsSecure() const override;
	size_t HashCtxSize() const override;
	size_t HashCtxAlign() const override;
	CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx) override;
	CRYPTO_STATUS HashUpdate(void *pHashCtx, const uint8_t *pMsg, size_t Len) override;
	CRYPTO_STATUS HashFinal(void *pHashCtx, uint8_t *pDigest) override;

private:
	// Borrowed target implementation with library lifetime. Null on software
	// targets and inside the target implementation itself.
	Crypto *vpHardware = nullptr;
};

#endif
