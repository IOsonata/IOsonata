/**-------------------------------------------------------------------------
@file	crypto_p256.cpp

@brief	Portable P-256 scalar and field helpers shared by the ECC engines.

		Small curve-independent utilities the software and hardware P-256 engines
		reuse: zero and range tests over big-endian field elements and scalars,
		a rejection-sampled private scalar, and a constant-time scalar
		regularization for a fixed-window multiply. These are plain byte-array
		operations against the P-256 group order; they hold no engine state.

		Randomness for a private scalar comes from an injected RngEngine, provided
		by the target random driver. A scalar is accepted only when it is a
		valid private key: non-zero and strictly less than the group order n.

@author	Hoang Nguyen Hoan
@date	Jul. 13, 2026

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
#include <stddef.h>
#include <string.h>

#include "crypto/icrypto.h"

static const uint8_t s_P256Order[P256_BYTES] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xBC,0xE6,0xFA,0xAD,0xA7,0x17,0x9E,0x84,0xF3,0xB9,0xCA,0xC2,0xFC,0x63,0x25,0x51 };

static const uint8_t s_P256Prime[P256_BYTES] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

extern "C" {

void CryptoSecureWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0)
	{
		*p++ = 0;
	}
}

bool P256IsZero(const uint8_t *pData, size_t Len)
{
	uint8_t acc = 0;
	for (size_t i = 0; i < Len; i++)
	{
		acc |= pData[i];
	}
	return acc == 0;
}

bool P256LessBe(const uint8_t *pA, const uint8_t *pB, size_t Len)
{
	int lt = 0;
	int gt = 0;
	for (size_t i = 0; i < Len; i++)
	{
		int a = pA[i];
		int b = pB[i];
		int curLt = (a < b) & 1;
		int curGt = (a > b) & 1;
		int decided = lt | gt;
		lt |= curLt & (decided ^ 1);
		gt |= curGt & (decided ^ 1);
	}
	return lt != 0;
}

bool P256NonzeroFieldElement(const uint8_t Coord[P256_BYTES])
{
	if (P256IsZero(Coord, P256_BYTES))
	{
		return false;
	}
	return P256LessBe(Coord, s_P256Prime, P256_BYTES);
}

bool P256ScalarInRange(const uint8_t Scalar[P256_BYTES])
{
	if (P256IsZero(Scalar, P256_BYTES))
	{
		return false;
	}
	return P256LessBe(Scalar, s_P256Order, P256_BYTES);
}

CRYPTO_STATUS P256RandomScalarStatus(RngEngine *pRng,
									uint8_t Scalar[P256_BYTES])
{
	if (Scalar == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (pRng == nullptr || !pRng->IsSecure())
	{
		CryptoSecureWipe(Scalar, P256_BYTES);
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	for (int attempt = 0; attempt < 64; attempt++)
	{
		CRYPTO_STATUS status = pRng->Random(Scalar, P256_BYTES);
		if (status != CRYPTO_STATUS_OK)
		{
			CryptoSecureWipe(Scalar, P256_BYTES);
			return status;
		}
		if (P256ScalarInRange(Scalar))
		{
			return CRYPTO_STATUS_OK;
		}
	}
	CryptoSecureWipe(Scalar, P256_BYTES);
	return CRYPTO_STATUS_FAIL;
}

bool P256RandomScalar(RngEngine *pRng, uint8_t Scalar[P256_BYTES])
{
	return P256RandomScalarStatus(pRng, Scalar) == CRYPTO_STATUS_OK;
}

static uint8_t P256AddBe(const uint8_t A[P256_BYTES], const uint8_t B[P256_BYTES],
						 uint8_t Out[P256_BYTES])
{
	uint32_t carry = 0U;
	for (int idx = (int)P256_BYTES - 1; idx >= 0; idx--)
	{
		const uint32_t sum = (uint32_t)A[idx] + B[idx] + carry;
		Out[idx] = (uint8_t)sum;
		carry = sum >> 8;
	}
	return (uint8_t)carry;
}

void P256RegularizeScalar(const uint8_t K[P256_BYTES], uint8_t R[P256_BYTES + 1U])
{
	uint8_t kPlusN[P256_BYTES];
	uint8_t kPlus2N[P256_BYTES];
	const uint8_t carry1 = P256AddBe(K, s_P256Order, kPlusN);
	(void)P256AddBe(kPlusN, s_P256Order, kPlus2N);
	const uint8_t mask = (uint8_t)(0U - (uint32_t)carry1);
	R[0] = 1U;
	for (size_t idx = 0U; idx < P256_BYTES; idx++)
	{
		R[idx + 1U] = (uint8_t)((kPlusN[idx] & mask) |
								(kPlus2N[idx] & (uint8_t)~mask));
	}
	CryptoSecureWipe(kPlusN, sizeof(kPlusN));
	CryptoSecureWipe(kPlus2N, sizeof(kPlus2N));
}

uint32_t P256RegularBit(const uint8_t Scalar[P256_BYTES + 1U], uint32_t BitNo)
{
	const uint32_t byteFromEnd = BitNo / 8U;
	return (Scalar[P256_BYTES - byteFromEnd] >> (BitNo & 7U)) & 1U;
}

} // extern "C"
