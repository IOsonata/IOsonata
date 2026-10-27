/**-------------------------------------------------------------------------
@file	sha256_test.cpp

@brief	Host test support: sha256 test.

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

// SHA/HMAC differential tests and synchronous hash failure injection.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include "crypto/crypto_softsha256.h"

class FailingHash : public CryptoSoftSha256Hash {
public:
	unsigned Calls = 0;
	unsigned FailAt = 0;
	CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG Alg, void *pCtx) override
	{
		return ++Calls == FailAt ? CRYPTO_STATUS_FAIL : CryptoSoftSha256Hash::HashInit(Alg, pCtx);
	}
	CRYPTO_STATUS HashUpdate(void *pCtx, const uint8_t *pMsg, size_t Len) override
	{
		return ++Calls == FailAt ? CRYPTO_STATUS_FAIL : CryptoSoftSha256Hash::HashUpdate(pCtx, pMsg, Len);
	}
	CRYPTO_STATUS HashFinal(void *pCtx, uint8_t *pDigest) override
	{
		return ++Calls == FailAt ? CRYPTO_STATUS_FAIL : CryptoSoftSha256Hash::HashFinal(pCtx, pDigest);
	}
};

int main()
{
	CryptoSoftSha256 engine;
	FailingHash failing;
	uint8_t msg[4097], key[131], expected[32], actual[34], overlap[4097];
	alignas(CRYPTO_HASHCTX_ALIGN_MAX) uint8_t ctx[CRYPTO_HASHCTX_MAX];
	CryptoKey k{CRYPTO_KEY_HMAC, CRYPTO_KEY_LOC_PLAIN, CRYPTO_KEY_USE_SIGN, {}};
	const size_t keySizes[] = {0, 1, 32, 63, 64, 65, 131};
	unsigned digestLen, calls;
	size_t i, len, off, count, keyLen, keyIndex, macLen;

	engine.Enable();
	for (i = 0; i < sizeof(msg); i++)
		msg[i] = (uint8_t)(i * 17U + (i >> 5));
	for (i = 0; i < sizeof(key); i++)
		key[i] = (uint8_t)(i * 31U);
	k.Plain.pData = key;
	for (len = 0; len <= 4096; len++)
	{
		assert(EVP_Digest(msg + 1, len, expected, &digestLen, EVP_sha256(), nullptr) == 1);
		memset(actual, 0xA5, sizeof(actual));
		assert(engine.Hash(CRYPTO_HASH_SHA256, msg + 1, len, actual + 1) == CRYPTO_STATUS_OK);
		assert(memcmp(actual + 1, expected, 32) == 0 && actual[0] == 0xA5 && actual[33] == 0xA5);
		assert(engine.HashInit(CRYPTO_HASH_SHA256, ctx) == CRYPTO_STATUS_OK);
		for (off = 0; off < len; off += count)
		{
			count = 1U + off % 137U;
			if (count > len - off)
				count = len - off;
			assert(engine.HashUpdate(ctx, msg + 1 + off, count) == CRYPTO_STATUS_OK);
		}
		assert(engine.HashFinal(ctx, actual + 1) == CRYPTO_STATUS_OK);
		assert(memcmp(actual + 1, expected, 32) == 0);
		for (i = 0; i < engine.HashCtxSize(); i++)
			assert(ctx[i] == 0);
		assert(engine.HashUpdate(ctx, msg, 1) != CRYPTO_STATUS_OK);
		for (keyIndex = 0; keyIndex < sizeof(keySizes) / sizeof(keySizes[0]); keyIndex++)
		{
			keyLen = keySizes[keyIndex];
			k.Plain.Len = keyLen;
			assert(HMAC(EVP_sha256(), key, (int)keyLen, msg + 1, len, expected, &digestLen) != nullptr);
			macLen = 1U + len % 32U;
			memset(actual, 0xA5, sizeof(actual));
			assert(engine.Mac(CRYPTO_MAC_HMAC, k, msg + 1, len, actual + 1, macLen) == CRYPTO_STATUS_OK);
			assert(memcmp(actual + 1, expected, macLen) == 0);
			assert(actual[0] == 0xA5 && actual[macLen + 1] == 0xA5);
			memcpy(overlap, msg + 1, len);
			assert(engine.Mac(CRYPTO_MAC_HMAC, k, overlap, len, overlap, 32) == CRYPTO_STATUS_OK);
			assert(memcmp(overlap, expected, 32) == 0);
		}
	}
	for (keyIndex = 0; keyIndex < sizeof(keySizes) / sizeof(keySizes[0]); keyIndex++)
	{
		keyLen = keySizes[keyIndex];
		k.Plain.Len = keyLen;
		failing.Calls = 0;
		failing.FailAt = 0;
		assert(CryptoHmacSha256(&failing, CRYPTO_MAC_HMAC, k, msg, sizeof(msg), actual + 1, 32) == CRYPTO_STATUS_OK);
		calls = failing.Calls;
		for (i = 1; i <= calls; i++)
		{
			failing.Calls = 0;
			failing.FailAt = i;
			memset(actual, 0xA5, sizeof(actual));
			assert(CryptoHmacSha256(&failing, CRYPTO_MAC_HMAC, k, msg, sizeof(msg), actual + 1, 32) == CRYPTO_STATUS_FAIL);
			assert(failing.Calls == i);
			assert(actual[0] == 0xA5 && actual[33] == 0xA5);
			for (off = 1; off <= 32; off++)
				assert(actual[off] == 0);
		}
	}
	puts("SHA-256/HMAC: lengths 0..4096, streaming, truncation, overlap and every hash failure passed");
}
