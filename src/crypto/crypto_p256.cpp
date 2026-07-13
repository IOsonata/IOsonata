/**-------------------------------------------------------------------------
@file	crypto_p256.cpp

@brief	NIST P-256 (secp256r1) constants and byte-level helpers.

		See crypto_p256.h. No hardware or PKA dependency.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <assert.h>

#include "crypto/crypto_p256.h"
#include "crypto/crypto.h"		// RngGet, CryptoSecureWipe

#ifndef P256_RANDOM_ATTEMPTS
#define P256_RANDOM_ATTEMPTS	100U
#endif

static const uint8_t s_P256Order[P256_BYTES] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xBC,0xE6,0xFA,0xAD,0xA7,0x17,0x9E,0x84,0xF3,0xB9,0xCA,0xC2,0xFC,0x63,0x25,0x51,
};

static const uint8_t s_P256Field[P256_BYTES] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};

bool P256IsZero(const uint8_t *pData, size_t Len)
{
	uint8_t value = 0U;
	for (size_t idx = 0U; idx < Len; idx++)
	{
		value |= pData[idx];
	}
	return value == 0U;
}

bool P256LessBe(const uint8_t *pA, const uint8_t *pB, size_t Len)
{
	uint32_t less = 0U;
	uint32_t greater = 0U;
	for (size_t idx = 0U; idx < Len; idx++)
	{
		const uint32_t a = pA[idx];
		const uint32_t b = pB[idx];
		const uint32_t undecided = 1U ^ (less | greater);
		less |= undecided & ((a - b) >> 31);
		greater |= undecided & ((b - a) >> 31);
	}
	return less != 0U;
}

uint8_t P256AddBe(const uint8_t A[P256_BYTES], const uint8_t B[P256_BYTES],
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

bool P256RandomScalar(uint8_t Scalar[P256_BYTES])
{
	for (uint32_t attempt = 0U; attempt < P256_RANDOM_ATTEMPTS; attempt++)
	{
		if (RngGet(Scalar, P256_BYTES) == false)
		{
			CryptoSecureWipe(Scalar, P256_BYTES);
			return false;
		}
		if (!P256IsZero(Scalar, P256_BYTES) &&
			P256LessBe(Scalar, s_P256Order, P256_BYTES))
		{
			return true;
		}
	}
	CryptoSecureWipe(Scalar, P256_BYTES);
	return false;
}

bool P256ScalarInRange(const uint8_t Scalar[P256_BYTES])
{
	return !P256IsZero(Scalar, P256_BYTES) &&
		   P256LessBe(Scalar, s_P256Order, P256_BYTES);
}

bool P256NonzeroFieldElement(const uint8_t Coord[P256_BYTES])
{
	return !P256IsZero(Coord, P256_BYTES) &&
		   P256LessBe(Coord, s_P256Field, P256_BYTES);
}

void P256RegularizeScalar(const uint8_t K[P256_BYTES], uint8_t R[P256_BYTES + 1U])
{
	uint8_t kPlusN[P256_BYTES];
	uint8_t kPlus2N[P256_BYTES];
	const uint8_t carry1 = P256AddBe(K, s_P256Order, kPlusN);
	const uint8_t carry2 = P256AddBe(kPlusN, s_P256Order, kPlus2N);
	assert(carry1 != 0U || carry2 != 0U);
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
