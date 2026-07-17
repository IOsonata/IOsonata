/**-------------------------------------------------------------------------
@file	crypto_softrng.cpp

@brief	Software pseudo-random generator on the OO engine tree.

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
#include <stdint.h>
#include <new>

#include "crypto/crypto_softrng.h"

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
	if (Seed0 == 0U && Seed1 == 0U)
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
		return Len == 0U ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}

	for (size_t off = 0; off < Len; )
	{
		uint64_t value = Xorshift128p(&vState0, &vState1);
		size_t count = Len - off < sizeof(value) ? Len - off : sizeof(value);
		for (size_t i = 0; i < count; i++)
		{
			pOut[off + i] = (uint8_t)(value >> (8U * i));
		}
		off += count;
	}
	return CRYPTO_STATUS_OK;
}

CryptoSoftRng *CryptoSoftRngCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoSoftRng) ||
		((uintptr_t)pMem & (alignof(CryptoSoftRng) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoSoftRng *p = new (pMem) CryptoSoftRng();
	p->Enable();
	return p;
}
