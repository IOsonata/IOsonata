/**-------------------------------------------------------------------------
@file	crypto_cc3xx.h

@brief	CryptoCc3xx: P-256 ECDH on the Arm CryptoCell CC3xx PKA.

@author	Hoang Nguyen Hoan
@date	Jul 2026

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
#ifndef __CRYPTO_CC3XX_H__
#define __CRYPTO_CC3XX_H__

#include <stdint.h>

#include "crypto/icrypto.h"
#include "cc3xx_intrf.h"

#ifdef __cplusplus

class CryptoCc3xx : public KeyAgreeEngine {
public:
	CryptoCc3xx()
	{
		vbValid = false;
		vbIntrfEnabled = false;
		vpRng = nullptr;
	}

	struct KeyCtx {
		uint8_t PrivKey[32];
		bool bKeyValid;
	};

	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	bool Init(Cc3xxIntrf * const pIntrf, RngEngine *pRng)
	{
		return Init((DeviceIntrf *)pIntrf, pRng);
	}

	bool Enable() override;
	void Disable() override;
	void Reset() override;
	int SelfTest() override;

	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	size_t KeyCtxAlign() const override { return alignof(KeyCtx); }
	void KeyReset(void *pKeyCtx) override
	{
		if (pKeyCtx != nullptr)
		{
			CryptoSecureWipe(pKeyCtx, sizeof(KeyCtx));
		}
	}
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPubKey, uint8_t *pSharedX,
						bool bKeepKey = false) override;

protected:
	RngEngine *vpRng;
	// Whether this engine owns one interface user count. Separate from
	// vbValid: a failed re-Enable leaves the engine non-operational while
	// the count is still owned, and the count must be released exactly once.
	bool vbIntrfEnabled;

private:
	bool Init(DeviceIntrf * const pIntrf, RngEngine *pRng);
};

static_assert(sizeof(CryptoCc3xx::KeyCtx) <= CRYPTO_KEYCTX_MAX,
			  "CC3xx key context exceeds the common consumer storage");

#define CRYPTO_CC3XX_MEMSIZE		sizeof(CryptoCc3xx)

CryptoCc3xx *CryptoCc3xxCreate(void *pMem, size_t MemSize, RngEngine *pRng);

#endif // __cplusplus

#endif // __CRYPTO_CC3XX_H__
