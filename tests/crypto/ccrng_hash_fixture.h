/**-------------------------------------------------------------------------
@file	ccrng_hash_fixture.h

@brief	Host test support: ccrng hash fixture.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#pragma once
#define OPENSSL_SUPPRESS_DEPRECATED
#include <openssl/sha.h>
#include "crypto/crypto_softsha256.h"
#include "cc3xx_intrf.h"
extern int g_HashFailAfter;
class CryptoCc3xxSym : public HashEngine {
public:
	bool Init(Cc3xxIntrf *)
	{
		return true;
	}
	void Disable() override
	{
	}
	bool Enable() override
	{
		return true;
	}
	void Reset() override
	{
	}
	size_t HashCtxSize() const override { return sizeof(SHA256_CTX); }
	size_t HashCtxAlign() const override { return alignof(SHA256_CTX); }
	CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG, void *p) override {
		if (g_HashFailAfter == 0)
			return CRYPTO_STATUS_FAIL;
		if (g_HashFailAfter > 0)
			--g_HashFailAfter;
		return SHA256_Init((SHA256_CTX *)p) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS HashUpdate(void *p, const uint8_t *in, size_t n) override {
		return SHA256_Update((SHA256_CTX *)p, in, n) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS HashFinal(void *p, uint8_t *out) override {
		return SHA256_Final(out, (SHA256_CTX *)p) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
};
