/**-------------------------------------------------------------------------
@file	crypto_uecc.h

@brief	Software P-256 crypto engine over micro-ecc.

@author	Hoang Nguyen Hoan
@date	Jul. 15, 2026

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
#ifndef __CRYPTO_UECC_H__
#define __CRYPTO_UECC_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

class CryptoUecc : public KeyAgreeEngine, public SignEngine {
public:
	struct KeyCtx {
		uint8_t PrivKey[32];
		bool bKeyValid;
	};

	CryptoUecc() { vbValid = false; vpRng = nullptr; }
	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	void KeyReset(void *pKeyCtx) override;
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPubKey, uint8_t *pSharedX,
						bool bKeepKey = false) override;

	CRYPTO_STATUS Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
					   const uint8_t *pHash, size_t HashLen,
					   uint8_t *pSig) override;
	CRYPTO_STATUS Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
						 const uint8_t *pHash, size_t HashLen,
						 const uint8_t *pSig) override;

	int SelfTest() override;

private:
	RngEngine *vpRng;
};

static_assert(sizeof(CryptoUecc::KeyCtx) <= CRYPTO_KEYCTX_MAX,
			  "CryptoUecc key context exceeds the common consumer storage");

#define CRYPTO_UECC_MEMSIZE		sizeof(CryptoUecc)

CryptoUecc *CryptoUeccCreate(void *pMem, size_t MemSize, RngEngine *pRng);

/** @} */

#endif // __CRYPTO_UECC_H__
