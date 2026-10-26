/**-------------------------------------------------------------------------
@file	ccsym_model.cpp

@brief	Host test support: ccsym model.

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

#include "crypto_cc3xx_sym.h"
#include "crypto/crypto.h"
#define OPENSSL_SUPPRESS_DEPRECATED
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <openssl/rand.h>
#include <cassert>
#include <cstring>
#include <cstdio>

#if defined(NRF52840_XXAA) || defined(NRF9160_XXAA) || defined(NRF9120_XXAA)
#define MODEL_DOUT 0xC00U
#else
#define MODEL_DOUT 0xD00U
#endif
static uint32_t s_Reg[1024];
static uint8_t s_Input[64], s_Output[16];
static unsigned s_In, s_Out, s_AesBlocks, s_HashBlocks, s_KeyLoads;
static bool s_Timeout;
static bool s_SetupTimeout;
static bool s_ReadFailure;
static bool s_CleanupFailure;
uintptr_t Cc3xxBase()
{
	return (uintptr_t)s_Reg;
}
bool Cc3xxEnable()
{
	return true;
}
void Cc3xxDisable()
{
	memset(s_Reg, 0, sizeof(s_Reg));
	s_In = s_Out = 0;
}

class ModelSym : public CryptoCc3xxSym
{
public:
	int Read(uint8_t *addr, int len, uint8_t *out, int count) override
	{
		uint32_t offset, value;
		assert(len == 4 && count == 4);
		memcpy(&offset, addr, 4);
		if (s_ReadFailure)
			return 0;
		assert(offset < sizeof(s_Reg));
		value = s_Reg[offset/4];
		if (offset == 0xC50)
			value = s_Timeout ? 0 : 1;
		if (offset == 0x910)
			value = s_SetupTimeout ? 1 : 0;
		if (offset == 0x91C)
			value = 0;
		if (offset == 0xD50)
			value = s_Out >= 16;
		if (offset == MODEL_DOUT)
		{
			assert(s_Out < 16);
			memcpy(&value, s_Output+s_Out, 4);
			s_Out += 4;
		}
		memcpy(out, &value, 4);
		return 4;
	}
	int Write(uint8_t *addr, int len, const uint8_t *in, int count) override
	{
		uint32_t offset, value;
		assert(len == 4 && count > 0 && count % 4 == 0);
		if (count > 4)
		{
			memcpy(&offset, addr, 4);
			for (int i = 0; i < count; i += 4)
			{
				uint32_t next = offset + i;
				if (Write((uint8_t *)&next, 4, in + i, 4) != 4)
					return i;
			}
			return count;
		}
		memcpy(&offset, addr, 4);
		memcpy(&value, in, 4);
		assert(offset < sizeof(s_Reg));
		if (s_CleanupFailure && offset == 0x900 && value == 0)
			return 0;
		s_Reg[offset/4] = value;
		if (offset == 0x4C0)
			s_KeyLoads++;
		if (offset == 0xC58 || offset == 0xC48)
			s_In = 0;
		if (offset == 0xC00)
		{
			assert(s_In < 64);
			memcpy(s_Input+s_In, &value, 4);
			s_In += 4;
			if (s_Reg[0x900/4] == 1 && s_In == 16)
			{
				uint32_t control = s_Reg[0x4C0/4];
				int n, final;
				EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
				assert(ctx);
				assert(EVP_CipherInit_ex(ctx, (control & 0x2000) ? EVP_aes_256_ecb() : EVP_aes_128_ecb(),
					nullptr, (uint8_t *)&s_Reg[0x400/4], nullptr, !(control & 1)) == 1);
				assert(EVP_CIPHER_CTX_set_padding(ctx, 0) == 1);
				assert(EVP_CipherUpdate(ctx, s_Output, &n, s_Input, 16) == 1);
				assert(EVP_CipherFinal_ex(ctx, s_Output+n, &final) == 1);
				EVP_CIPHER_CTX_free(ctx);
				s_Out = 0;
				s_AesBlocks++;
			}
			if (s_Reg[0x900/4] == 7 && s_In == 64)
			{
				SHA256_CTX ctx;
				assert(SHA256_Init(&ctx) == 1);
				assert(s_Reg[0x7C0/4] == 2 && s_Reg[0x684/4] == 0);
				memcpy(ctx.h, &s_Reg[0x640/4], 32);
				SHA256_Transform(&ctx, s_Input);
				memcpy(&s_Reg[0x640/4], ctx.h, 32);
				s_HashBlocks++;
			}
		}
		return 4;
	}
};
static ModelSym s_Sym;
bool Crypto::Init()
{
	vbValid = s_Sym.Init(Cc3xxIntrfInstance());
	return vbValid;
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
	return s_Sym.Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
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
	return s_Sym.Seal(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
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
	return s_Sym.Open(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
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
	return s_Sym.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
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
	return s_Sym.Hash(Alg, pMsg, Len, pDigest);
}

size_t Crypto::HashCtxSize() const
{
	if (!vbValid)
	{
		return 0U;
	}
	return s_Sym.HashCtxSize();
}

size_t Crypto::HashCtxAlign() const
{
	if (!vbValid)
	{
		return 1U;
	}
	return s_Sym.HashCtxAlign();
}

CRYPTO_STATUS Crypto::HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return s_Sym.HashInit(Alg, pHashCtx);
}

CRYPTO_STATUS Crypto::HashUpdate(void *pHashCtx, const uint8_t *pMsg, size_t Len)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return s_Sym.HashUpdate(pHashCtx, pMsg, Len);
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
	return s_Sym.HashFinal(pHashCtx, pDigest);
}

bool Crypto::IsSecure() const
{
	return vbValid;
}
CRYPTO_STATUS Crypto::Random(uint8_t *out, size_t len)
{
#ifdef CRACEN_RNG_MODEL
	return CryptoRngNrfInstance()->Random(out, len);
#else
	return RAND_bytes(out, len) == 1 ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
#endif
}
extern int CryptoEngineTest();
int main()
{
	Crypto c;
	uint8_t key[32] = {}, input[64] = {}, out[34];
	CryptoKey k = {};
	int owner;
	ModelSym second;
	alignas(16) uint8_t ctxA[CRYPTO_HASHCTX_MAX], ctxB[CRYPTO_HASHCTX_MAX];
	uint8_t msgA[137], msgB[137], digestA[32], digestB[32], ref[32];
	uint8_t nonce[12] = {}, tag[16], cipher[16];
	unsigned i;
	assert(CryptoEngineTest() == 0);
	assert(s_AesBlocks > 1000 && s_HashBlocks > 1000);
	assert(c.Init());
#if defined(NRF52840_XXAA) || defined(NRF9160_XXAA) || defined(NRF9120_XXAA)
	// CC310 AES-256 must use software without starting a hardware block.
	{
		CryptoSoftAes software;
		CryptoKey wide = {};
		uint8_t expected[16];
		uint8_t actual[16];
		unsigned blocks = s_AesBlocks;

		assert(software.Enable());
		wide.Type = CRYPTO_KEY_AES_256;
		wide.Loc = CRYPTO_KEY_LOC_PLAIN;
		wide.Usage = CRYPTO_KEY_USE_ENCRYPT;
		wide.Plain.pData = key;
		wide.Plain.Len = sizeof(key);
		assert(software.Cipher(CRYPTO_CIPHER_ECB, 1, wide, nullptr, 0,
			input, sizeof(expected), expected) == CRYPTO_STATUS_OK);
		assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, wide, nullptr, 0,
			input, sizeof(actual), actual) == CRYPTO_STATUS_OK);
		assert(memcmp(expected, actual, sizeof(actual)) == 0);
		assert(s_AesBlocks == blocks);
	}
#endif
#if defined(NRF52840_XXAA) || defined(NRF9160_XXAA) || defined(NRF9120_XXAA)
	k.Type = CRYPTO_KEY_AES_128;
#else
	k.Type = CRYPTO_KEY_AES_256;
#endif
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT;
	k.Plain.pData = key;
	k.Plain.Len = k.Type == CRYPTO_KEY_AES_128 ? 16U : sizeof(key);
	assert(Cc3xxIntrfInstance()->OpHold(&owner));
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_BUSY);
	assert(c.Hash(CRYPTO_HASH_SHA256, input, 64, out+1) == CRYPTO_STATUS_BUSY);
	assert(Cc3xxIntrfInstance()->OpRelease(&owner));
	{
		uint8_t multi[64];
		unsigned loads = s_KeyLoads;
		assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 64, multi) == CRYPTO_STATUS_OK);
		assert(s_KeyLoads == loads + 1);
		key[0] = 1;
		assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 64, multi) == CRYPTO_STATUS_OK);
		assert(s_KeyLoads == loads + 2);
		key[0] = 0;
	}
	s_Timeout = true;
	memset(out, 0xA5, sizeof(out));
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_FAIL);
	assert(out[0] == 0xA5 && out[17] == 0xA5);
	for (unsigned i = 1; i <= 16; i++)
		assert(out[i] == 0);
	memset(out, 0xA5, sizeof(out));
	assert(c.Hash(CRYPTO_HASH_SHA256, input, 64, out+1) == CRYPTO_STATUS_FAIL);
	for (unsigned i = 1; i <= 32; i++)
		assert(out[i] == 0);
	assert(out[0] == 0xA5 && out[33] == 0xA5);
	s_Timeout = false;
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_OK);
	assert(c.Hash(CRYPTO_HASH_SHA256, input, 64, out+1) == CRYPTO_STATUS_OK);
	assert(second.Init(Cc3xxIntrfInstance()));
	assert(s_Sym.HashInit(CRYPTO_HASH_SHA256, ctxA) == CRYPTO_STATUS_OK);
	assert(second.HashInit(CRYPTO_HASH_SHA256, ctxB) == CRYPTO_STATUS_OK);
	for (i = 0; i < sizeof(msgA); i++)
	{
		msgA[i] = (uint8_t)i;
		msgB[i] = (uint8_t)(255U-i);
		assert(s_Sym.HashUpdate(ctxA, msgA+i, 1) == CRYPTO_STATUS_OK);
		assert(second.HashUpdate(ctxB, msgB+i, 1) == CRYPTO_STATUS_OK);
		assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_OK);
	}
	assert(s_Sym.HashFinal(ctxA, digestA) == CRYPTO_STATUS_OK);
	assert(second.HashFinal(ctxB, digestB) == CRYPTO_STATUS_OK);
	assert(SHA256(msgA, sizeof(msgA), ref));
	assert(memcmp(ref, digestA, 32) == 0);
	assert(SHA256(msgB, sizeof(msgB), ref));
	assert(memcmp(ref, digestB, 32) == 0);
	for (i = 0; i < s_Sym.HashCtxSize(); i++)
		assert(ctxA[i] == 0 && ctxB[i] == 0);
	s_SetupTimeout = true;
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_FAIL);
	assert(c.Hash(CRYPTO_HASH_SHA256, input, 64, out+1) == CRYPTO_STATUS_FAIL);
	s_SetupTimeout = false;
	s_ReadFailure = true;
	assert(c.Hash(CRYPTO_HASH_SHA256, input, 64, out+1) == CRYPTO_STATUS_FAIL);
	s_ReadFailure = false;
	assert(Cc3xxIntrfInstance()->OpHold(&owner));
	assert(second.Hash(CRYPTO_HASH_SHA256, input, 64, out+1) == CRYPTO_STATUS_BUSY);
	assert(Cc3xxIntrfInstance()->OpRelease(&owner));
	assert(c.Hash(CRYPTO_HASH_SHA256, input, 64, out+1) == CRYPTO_STATUS_OK);
	for (i = 0; i < 8; i++)
		assert(s_Reg[0x400/4+i] == 0 && s_Reg[0x640/4+i] == 0);
	k.Type = CRYPTO_KEY_AES_128;
	k.Plain.Len = 16;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT | CRYPTO_KEY_USE_SIGN;
	assert(c.Seal(CRYPTO_AEAD_AES_GCM, k, nonce, sizeof(nonce), nullptr, 0, input, 16, cipher, tag, 16) == CRYPTO_STATUS_OK);
	s_CleanupFailure = true;
	assert(c.Open(CRYPTO_AEAD_AES_GCM, k, nonce, sizeof(nonce), nullptr, 0, cipher, 16, out+1, tag, 16) == CRYPTO_STATUS_FAIL);
	for (i = 1; i <= 16; i++)
		assert(out[i] == 0);
	assert(c.Seal(CRYPTO_AEAD_AES_GCM, k, nonce, sizeof(nonce), nullptr, 0, input, 16, out+1, tag, 16) == CRYPTO_STATUS_FAIL);
	for (i = 0; i < 16; i++)
		assert(tag[i] == 0 && out[i+1] == 0);
	assert(c.Mac(CRYPTO_MAC_CMAC, k, input, 16, out+1, 16) == CRYPTO_STATUS_FAIL);
	for (i = 1; i <= 16; i++)
		assert(out[i] == 0);
	s_CleanupFailure = false;
	second.Disable();
	printf("CC3xx FIFO model: %u AES and %u SHA blocks; shared exclusion and timeouts passed\n", s_AesBlocks, s_HashBlocks);
}
