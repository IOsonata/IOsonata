/**-------------------------------------------------------------------------
@file	crypto_softsha256.cpp

@brief	Software SHA-256 crypto engine (FIPS 180-4).

@author	Hoang Nguyen Hoan
@date	Jul. 16, 2026

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
#include <stdint.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softsha256.h"

static const uint32_t s_Sha256K[64] = {
	0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,
	0x923f82a4u,0xab1c5ed5u,0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,
	0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,0xe49b69c1u,0xefbe4786u,
	0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
	0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,
	0x06ca6351u,0x14292967u,0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,
	0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,0xa2bfe8a1u,0xa81a664bu,
	0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
	0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,
	0x5b9cca4fu,0x682e6ff3u,0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,
	0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u
};

static uint32_t Sha256Ror(uint32_t x, int n)
{
	return (x >> n) | (x << (32 - n));
}

static void Sha256Wipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0U) *p++ = 0U;
}

static void Sha256Block(uint32_t H[8], const uint8_t block[64])
{
	uint32_t W[64];
	for (int t = 0; t < 16; t++)
	{
		W[t] = ((uint32_t)block[4*t] << 24) |
			   ((uint32_t)block[4*t+1] << 16) |
			   ((uint32_t)block[4*t+2] << 8) |
			   (uint32_t)block[4*t+3];
	}
	for (int t = 16; t < 64; t++)
	{
		uint32_t s0 = Sha256Ror(W[t-15],7) ^ Sha256Ror(W[t-15],18) ^ (W[t-15] >> 3);
		uint32_t s1 = Sha256Ror(W[t-2],17) ^ Sha256Ror(W[t-2],19) ^ (W[t-2] >> 10);
		W[t] = W[t-16] + s0 + W[t-7] + s1;
	}
	uint32_t a=H[0], b=H[1], c=H[2], d=H[3];
	uint32_t e=H[4], f=H[5], g=H[6], h=H[7];
	for (int t = 0; t < 64; t++)
	{
		uint32_t s1 = Sha256Ror(e,6) ^ Sha256Ror(e,11) ^ Sha256Ror(e,25);
		uint32_t ch = (e & f) ^ (~e & g);
		uint32_t t1 = h + s1 + ch + s_Sha256K[t] + W[t];
		uint32_t s0 = Sha256Ror(a,2) ^ Sha256Ror(a,13) ^ Sha256Ror(a,22);
		uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
		uint32_t t2 = s0 + maj;
		h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
	}
	H[0]+=a; H[1]+=b; H[2]+=c; H[3]+=d;
	H[4]+=e; H[5]+=f; H[6]+=g; H[7]+=h;
	Sha256Wipe(W, sizeof(W));
}

// The magic marks an initialized, not yet finalized context: HashUpdate and
// HashFinal on arbitrary caller memory must not trust an uninitialized
// BufLen, and a finalized context must not be reusable.
#define SHA256_CTX_MAGIC	0x35326853UL

typedef struct {
	uint32_t Magic;
	uint32_t H[8];
	uint64_t Total;
	uint8_t Buf[64];
	size_t BufLen;
} Sha256Ctx;

static bool Sha256CtxLive(const void *pCtx)
{
	return pCtx != nullptr &&
		   ((uintptr_t)pCtx & (alignof(Sha256Ctx) - 1U)) == 0U &&
		   ((const Sha256Ctx *)pCtx)->Magic == SHA256_CTX_MAGIC;
}

static void Sha256Init(Sha256Ctx *c)
{
	c->Magic = SHA256_CTX_MAGIC;
	static const uint32_t initial[8] = {
		0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
		0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u
	};
	memcpy(c->H, initial, sizeof(initial));
	c->Total = 0U;
	c->BufLen = 0U;
}

static void Sha256Update(Sha256Ctx *c, const uint8_t *pData, size_t Len)
{
	c->Total += Len;
	while (Len > 0U)
	{
		size_t count = 64U - c->BufLen;
		if (count > Len) count = Len;
		memcpy(c->Buf + c->BufLen, pData, count);
		c->BufLen += count;
		pData += count;
		Len -= count;
		if (c->BufLen == 64U)
		{
			Sha256Block(c->H, c->Buf);
			c->BufLen = 0U;
		}
	}
}

static void Sha256Final(Sha256Ctx *c, uint8_t Digest[32])
{
	uint64_t bits = c->Total * 8U;
	c->Buf[c->BufLen++] = 0x80U;
	if (c->BufLen > 56U)
	{
		memset(c->Buf + c->BufLen, 0, 64U - c->BufLen);
		Sha256Block(c->H, c->Buf);
		c->BufLen = 0U;
	}
	memset(c->Buf + c->BufLen, 0, 56U - c->BufLen);
	for (int i = 0; i < 8; i++)
	{
		c->Buf[56+i] = (uint8_t)(bits >> (56 - 8*i));
	}
	Sha256Block(c->H, c->Buf);
	for (int i = 0; i < 8; i++)
	{
		Digest[4*i] = (uint8_t)(c->H[i] >> 24);
		Digest[4*i+1] = (uint8_t)(c->H[i] >> 16);
		Digest[4*i+2] = (uint8_t)(c->H[i] >> 8);
		Digest[4*i+3] = (uint8_t)c->H[i];
	}
}

CRYPTO_STATUS CryptoSoftSha256::Hash(CRYPTO_HASH_ALG Alg,
									 const uint8_t *pMsg, size_t Len,
									 uint8_t *pDigest)
{
	if (Alg != CRYPTO_HASH_SHA256 || pDigest == nullptr ||
		(pMsg == nullptr && Len != 0U))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	Sha256Ctx context;
	Sha256Init(&context);
	if (Len > 0U) Sha256Update(&context, pMsg, Len);
	Sha256Final(&context, pDigest);
	Sha256Wipe(&context, sizeof(context));
	return CRYPTO_STATUS_OK;
}

// Streaming facet: the caller context is a Sha256Ctx. The static_assert in
// this file keeps the class HashCtxSize claim in step with the struct.
static_assert(sizeof(Sha256Ctx) <= CRYPTO_HASHCTX_MAX,
			  "SHA-256 context exceeds the common consumer storage");

size_t CryptoSoftSha256::HashCtxSize() const
{
	return sizeof(Sha256Ctx);
}

size_t CryptoSoftSha256::HashCtxAlign() const
{
	return alignof(Sha256Ctx);
}

CRYPTO_STATUS CryptoSoftSha256::HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx)
{
	if (Alg != CRYPTO_HASH_SHA256 || pHashCtx == nullptr ||
		((uintptr_t)pHashCtx & (alignof(Sha256Ctx) - 1U)) != 0U)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	Sha256Init((Sha256Ctx *)pHashCtx);
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoSoftSha256::HashUpdate(void *pHashCtx,
										   const uint8_t *pMsg, size_t Len)
{
	if (!Sha256CtxLive(pHashCtx) || (pMsg == nullptr && Len != 0U))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (Len > 0U)
	{
		Sha256Update((Sha256Ctx *)pHashCtx, pMsg, Len);
	}
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoSoftSha256::HashFinal(void *pHashCtx, uint8_t *pDigest)
{
	if (!Sha256CtxLive(pHashCtx) || pDigest == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	Sha256Final((Sha256Ctx *)pHashCtx, pDigest);
	// The wipe clears the magic: a finalized context is dead until the next
	// HashInit.
	Sha256Wipe(pHashCtx, sizeof(Sha256Ctx));
	return CRYPTO_STATUS_OK;
}

#define HMAC_BLOCK			64U
#define SHA256_DIGEST		32U

// HMAC-SHA-256 (RFC 2104) over the virtual streaming hash calls, so a
// hardware subclass that overrides the streaming trio provides the digest
// work. The key context storage is bounded by CRYPTO_HASHCTX_MAX, the same
// bound every in-tree consumer reserves.
CRYPTO_STATUS CryptoSoftSha256::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
									const uint8_t *pMsg, size_t Len,
									uint8_t *pMac, size_t MacLen)
{
	// The generic construction chains the streaming hash synchronously on
	// stack storage; an asynchronous hash engine must provide its own HMAC.
	if (IsAsync() || Alg != CRYPTO_MAC_HMAC || Key.Type != CRYPTO_KEY_HMAC ||
		Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U ||
		(Key.Plain.pData == nullptr && Key.Plain.Len != 0U) ||
		pMac == nullptr || MacLen == 0U || MacLen > SHA256_DIGEST ||
		(pMsg == nullptr && Len != 0U) ||
		HashCtxSize() > CRYPTO_HASHCTX_MAX || HashCtxAlign() == 0U ||
		HashCtxAlign() > CRYPTO_HASHCTX_ALIGN_MAX)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	// Block-sized key: a longer key is first hashed (RFC 2104), a shorter
	// one zero padded.
	uint8_t key[HMAC_BLOCK] = {};
	alignas(CRYPTO_HASHCTX_ALIGN_MAX) uint8_t ctx[CRYPTO_HASHCTX_MAX];
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;
	if (Key.Plain.Len > HMAC_BLOCK)
	{
		if (HashInit(CRYPTO_HASH_SHA256, ctx) != CRYPTO_STATUS_OK ||
			HashUpdate(ctx, Key.Plain.pData, Key.Plain.Len) !=
				CRYPTO_STATUS_OK ||
			HashFinal(ctx, key) != CRYPTO_STATUS_OK)
		{
			st = CRYPTO_STATUS_FAIL;
		}
	}
	else if (Key.Plain.Len > 0U)
	{
		memcpy(key, Key.Plain.pData, Key.Plain.Len);
	}

	uint8_t pad[HMAC_BLOCK];
	uint8_t inner[SHA256_DIGEST];
	if (st == CRYPTO_STATUS_OK)
	{
		// Inner digest: H((K xor ipad) || message).
		for (size_t i = 0; i < HMAC_BLOCK; i++) pad[i] = key[i] ^ 0x36U;
		if (HashInit(CRYPTO_HASH_SHA256, ctx) != CRYPTO_STATUS_OK ||
			HashUpdate(ctx, pad, sizeof(pad)) != CRYPTO_STATUS_OK ||
			HashUpdate(ctx, pMsg, Len) != CRYPTO_STATUS_OK ||
			HashFinal(ctx, inner) != CRYPTO_STATUS_OK)
		{
			st = CRYPTO_STATUS_FAIL;
		}
	}
	if (st == CRYPTO_STATUS_OK)
	{
		// Outer digest: H((K xor opad) || inner).
		uint8_t full[SHA256_DIGEST];
		for (size_t i = 0; i < HMAC_BLOCK; i++) pad[i] = key[i] ^ 0x5CU;
		if (HashInit(CRYPTO_HASH_SHA256, ctx) != CRYPTO_STATUS_OK ||
			HashUpdate(ctx, pad, sizeof(pad)) != CRYPTO_STATUS_OK ||
			HashUpdate(ctx, inner, sizeof(inner)) != CRYPTO_STATUS_OK ||
			HashFinal(ctx, full) != CRYPTO_STATUS_OK)
		{
			st = CRYPTO_STATUS_FAIL;
		}
		else
		{
			memcpy(pMac, full, MacLen);
		}
		Sha256Wipe(full, sizeof(full));
	}

	Sha256Wipe(key, sizeof(key));
	Sha256Wipe(pad, sizeof(pad));
	Sha256Wipe(inner, sizeof(inner));
	Sha256Wipe(ctx, sizeof(ctx));
	if (st != CRYPTO_STATUS_OK)
	{
		memset(pMac, 0, MacLen);
	}
	return st;
}

int CryptoSoftSha256::SelfTest()
{
	static const uint8_t msg[3] = {'a','b','c'};
	static const uint8_t expected[32] = {
		0xba,0x78,0x16,0xbf,0x8f,0x01,0xcf,0xea,0x41,0x41,0x40,0xde,0x5d,0xae,0x22,0x23,
		0xb0,0x03,0x61,0xa3,0x96,0x17,0x7a,0x9c,0xb4,0x10,0xff,0x61,0xf2,0x00,0x15,0xad
	};
	static const uint8_t empty[32] = {
		0xe3,0xb0,0xc4,0x42,0x98,0xfc,0x1c,0x14,0x9a,0xfb,0xf4,0xc8,0x99,0x6f,0xb9,0x24,
		0x27,0xae,0x41,0xe4,0x64,0x9b,0x93,0x4c,0xa4,0x95,0x99,0x1b,0x78,0x52,0xb8,0x55
	};
	uint8_t digest[32];
	if (Hash(CRYPTO_HASH_SHA256, msg, sizeof(msg), digest) != CRYPTO_STATUS_OK ||
		memcmp(digest, expected, sizeof(digest)) != 0)
	{
		return -1;
	}
	if (Hash(CRYPTO_HASH_SHA256, nullptr, 0, digest) != CRYPTO_STATUS_OK ||
		memcmp(digest, empty, sizeof(digest)) != 0)
	{
		return -2;
	}

	// RFC 4231 test case 2: key "Jefe", message "what do ya want for nothing?".
	static const uint8_t hmacKey[4] = {'J','e','f','e'};
	static const uint8_t hmacMsg[28] = {
		'w','h','a','t',' ','d','o',' ','y','a',' ','w','a','n','t',' ',
		'f','o','r',' ','n','o','t','h','i','n','g','?'
	};
	static const uint8_t hmacExpected[32] = {
		0x5b,0xdc,0xc1,0x46,0xbf,0x60,0x75,0x4e,0x6a,0x04,0x24,0x26,0x08,0x95,0x75,0xc7,
		0x5a,0x00,0x3f,0x08,0x9d,0x27,0x39,0x83,0x9d,0xec,0x58,0xb9,0x64,0xec,0x38,0x43
	};
	CryptoKey mk{CRYPTO_KEY_HMAC, CRYPTO_KEY_LOC_PLAIN, CRYPTO_KEY_USE_SIGN, {}};
	mk.Plain.pData = hmacKey;
	mk.Plain.Len = sizeof(hmacKey);
	if (Mac(CRYPTO_MAC_HMAC, mk, hmacMsg, sizeof(hmacMsg), digest, 32U) !=
			CRYPTO_STATUS_OK ||
		memcmp(digest, hmacExpected, sizeof(hmacExpected)) != 0)
	{
		return -3;
	}
	return 0;
}

CryptoSoftSha256 *CryptoSoftSha256Create(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoSoftSha256) ||
		((uintptr_t)pMem & (alignof(CryptoSoftSha256) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoSoftSha256 *p = new (pMem) CryptoSoftSha256();
	p->Enable();
	return p;
}
