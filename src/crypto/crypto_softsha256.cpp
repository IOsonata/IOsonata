/**-------------------------------------------------------------------------
@file	crypto_softsha256.cpp

@brief	Software SHA-256 crypto engine (FIPS 180-4).

		Implements the HashEngine facet with a self-contained SHA-256 core. No
		key, no external dependency; one engine object serves any number of
		messages.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
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
	0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u,
};

static uint32_t Sha256Ror(uint32_t x, int n)
{
	return (x >> n) | (x << (32 - n));
}

static void Sha256Block(uint32_t H[8], const uint8_t blk[64])
{
	uint32_t W[64];
	for (int t = 0; t < 16; t++)
	{
		W[t] = ((uint32_t)blk[4 * t] << 24) | ((uint32_t)blk[4 * t + 1] << 16) |
			   ((uint32_t)blk[4 * t + 2] << 8) | (uint32_t)blk[4 * t + 3];
	}
	for (int t = 16; t < 64; t++)
	{
		uint32_t s0 = Sha256Ror(W[t - 15], 7) ^ Sha256Ror(W[t - 15], 18) ^ (W[t - 15] >> 3);
		uint32_t s1 = Sha256Ror(W[t - 2], 17) ^ Sha256Ror(W[t - 2], 19) ^ (W[t - 2] >> 10);
		W[t] = W[t - 16] + s0 + W[t - 7] + s1;
	}
	uint32_t a = H[0], b = H[1], c = H[2], d = H[3];
	uint32_t e = H[4], f = H[5], g = H[6], h = H[7];
	for (int t = 0; t < 64; t++)
	{
		uint32_t S1 = Sha256Ror(e, 6) ^ Sha256Ror(e, 11) ^ Sha256Ror(e, 25);
		uint32_t ch = (e & f) ^ (~e & g);
		uint32_t t1 = h + S1 + ch + s_Sha256K[t] + W[t];
		uint32_t S0 = Sha256Ror(a, 2) ^ Sha256Ror(a, 13) ^ Sha256Ror(a, 22);
		uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
		uint32_t t2 = S0 + maj;
		h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
	}
	H[0] += a; H[1] += b; H[2] += c; H[3] += d;
	H[4] += e; H[5] += f; H[6] += g; H[7] += h;
}

typedef struct {
	uint32_t H[8];
	uint64_t Total;
	uint8_t  Buf[64];
	size_t   BufLen;
} Sha256Ctx;

static void Sha256Init(Sha256Ctx *c)
{
	static const uint32_t H0[8] = {
		0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
		0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u
	};
	memcpy(c->H, H0, sizeof(H0));
	c->Total = 0;
	c->BufLen = 0;
}

static void Sha256Update(Sha256Ctx *c, const uint8_t *p, size_t n)
{
	c->Total += n;
	while (n > 0)
	{
		size_t take = 64 - c->BufLen;
		if (take > n)
		{
			take = n;
		}
		memcpy(c->Buf + c->BufLen, p, take);
		c->BufLen += take;
		p += take;
		n -= take;
		if (c->BufLen == 64)
		{
			Sha256Block(c->H, c->Buf);
			c->BufLen = 0;
		}
	}
}

static void Sha256Final(Sha256Ctx *c, uint8_t Digest[32])
{
	uint64_t bits = c->Total * 8;
	c->Buf[c->BufLen++] = 0x80;
	if (c->BufLen > 56)
	{
		memset(c->Buf + c->BufLen, 0, 64 - c->BufLen);
		Sha256Block(c->H, c->Buf);
		c->BufLen = 0;
	}
	memset(c->Buf + c->BufLen, 0, 56 - c->BufLen);
	for (int i = 0; i < 8; i++)
	{
		c->Buf[56 + i] = (uint8_t)((bits >> (56 - 8 * i)) & 0xFF);
	}
	Sha256Block(c->H, c->Buf);
	for (int i = 0; i < 8; i++)
	{
		Digest[4 * i]     = (uint8_t)(c->H[i] >> 24);
		Digest[4 * i + 1] = (uint8_t)(c->H[i] >> 16);
		Digest[4 * i + 2] = (uint8_t)(c->H[i] >> 8);
		Digest[4 * i + 3] = (uint8_t)(c->H[i]);
	}
}

CRYPTO_STATUS CryptoSoftSha256::Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg,
									 size_t Len, uint8_t *pDigest)
{
	if (Alg != CRYPTO_HASH_SHA256 || pDigest == nullptr ||
		(pMsg == nullptr && Len != 0))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	Sha256Ctx c;
	Sha256Init(&c);
	if (Len > 0)
	{
		Sha256Update(&c, pMsg, Len);
	}
	Sha256Final(&c, pDigest);
	return CRYPTO_STATUS_OK;
}

int CryptoSoftSha256::SelfTest()
{
	// FIPS 180-4 example: SHA-256("abc").
	static const uint8_t msg[3] = { 'a', 'b', 'c' };
	static const uint8_t expected[32] = {
		0xba,0x78,0x16,0xbf,0x8f,0x01,0xcf,0xea,0x41,0x41,0x40,0xde,0x5d,0xae,0x22,0x23,
		0xb0,0x03,0x61,0xa3,0x96,0x17,0x7a,0x9c,0xb4,0x10,0xff,0x61,0xf2,0x00,0x15,0xad,
	};

	uint8_t digest[32];
	if (Hash(CRYPTO_HASH_SHA256, msg, sizeof(msg), digest) != CRYPTO_STATUS_OK)
	{
		return -1;
	}
	if (memcmp(digest, expected, sizeof(expected)) != 0)
	{
		return -1;
	}

	// Empty message vector.
	static const uint8_t expectedEmpty[32] = {
		0xe3,0xb0,0xc4,0x42,0x98,0xfc,0x1c,0x14,0x9a,0xfb,0xf4,0xc8,0x99,0x6f,0xb9,0x24,
		0x27,0xae,0x41,0xe4,0x64,0x9b,0x93,0x4c,0xa4,0x95,0x99,0x1b,0x78,0x52,0xb8,0x55,
	};
	if (Hash(CRYPTO_HASH_SHA256, nullptr, 0, digest) != CRYPTO_STATUS_OK)
	{
		return -1;
	}
	if (memcmp(digest, expectedEmpty, sizeof(expectedEmpty)) != 0)
	{
		return -1;
	}
	return 0;
}

CryptoSoftSha256 *CryptoSoftSha256Create(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoSoftSha256))
	{
		return nullptr;
	}
	return new (pMem) CryptoSoftSha256();
}
