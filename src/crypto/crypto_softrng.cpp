/**-------------------------------------------------------------------------
@file	crypto_softrng.cpp

@brief	Software pseudo-random generator on the OO engine tree.

		Implements RngEngine with a xorshift128+ generator. Deterministic from
		its seed, good statistical quality, no hardware entropy. IsSecure() is
		false so key generation declines to use it. Statistical and non-security
		use only.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softrng.h"

// xorshift128+ step: advances the state and returns 64 pseudo-random bits.
static inline uint64_t Xorshift128p(uint64_t *pS0, uint64_t *pS1)
{
	uint64_t s1 = *pS0;
	uint64_t s0 = *pS1;
	*pS0 = s0;
	s1 ^= s1 << 23;
	s1 ^= s1 >> 17;
	s1 ^= s0;
	s1 ^= s0 >> 26;
	*pS1 = s1;
	return s1 + s0;
}

void CryptoSoftRng::Seed(uint64_t Seed0, uint64_t Seed1)
{
	// The state must never be all-zero; remap a zero seed to the default.
	if (Seed0 == 0 && Seed1 == 0)
	{
		Seed0 = 0x9E3779B97F4A7C15ULL;
		Seed1 = 0xBF58476D1CE4E5B9ULL;
	}
	vState0 = Seed0;
	vState1 = Seed1;
}

CRYPTO_STATUS CryptoSoftRng::Random(uint8_t *pOut, size_t Len)
{
	if (pOut == nullptr)
	{
		return (Len == 0) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}

	size_t off = 0;
	while (off < Len)
	{
		uint64_t r = Xorshift128p(&vState0, &vState1);
		size_t n = (Len - off < sizeof(r)) ? (Len - off) : sizeof(r);
		for (size_t j = 0; j < n; j++)
		{
			pOut[off + j] = (uint8_t)(r >> (8 * j));
		}
		off += n;
	}
	return CRYPTO_STATUS_OK;
}

CryptoSoftRng *CryptoSoftRngCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoSoftRng))
	{
		return nullptr;
	}
	CryptoSoftRng *p = new (pMem) CryptoSoftRng();
	p->Enable();
	return p;
}
