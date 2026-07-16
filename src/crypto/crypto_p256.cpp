/**-------------------------------------------------------------------------
@file	crypto_p256.cpp

@brief	Portable P-256 scalar and field helpers shared by the ECC engines.

		Small curve-independent utilities the software and hardware P-256 engines
		reuse: zero and range tests over big-endian field elements and scalars,
		a rejection-sampled private scalar, and a constant-time scalar
		regularization for a fixed-window multiply. These are plain byte-array
		operations against the P-256 group order; they hold no engine state.

		Randomness for a private scalar comes from the platform RngGet, provided
		by the target random driver. A scalar is accepted only when it is a
		valid private key: non-zero and strictly less than the group order n.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "crypto/icrypto.h"

// Platform random driver. Provided by the target (Nordic rng_nrfx, ST rng_stm32).
#ifdef __cplusplus
extern "C" {
#endif
bool RngGet(uint8_t *pBuff, size_t Len);
#ifdef __cplusplus
}
#endif

// P-256 group order n, big-endian (SEC2 secp256r1). Public constant.
static const uint8_t s_P256Order[P256_BYTES] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xBC,0xE6,0xFA,0xAD,0xA7,0x17,0x9E,0x84,0xF3,0xB9,0xCA,0xC2,0xFC,0x63,0x25,0x51 };

// P-256 field prime p, big-endian. Public constant.
static const uint8_t s_P256Prime[P256_BYTES] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

extern "C" {

// Zeroize helper. Volatile-through so the compiler cannot drop the writes.
void CryptoSecureWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0)
	{
		*p++ = 0;
	}
}

// True when every byte is zero. Reads all bytes so the timing does not reveal
// the position of the first non-zero byte.
bool P256IsZero(const uint8_t *pData, size_t Len)
{
	uint8_t acc = 0;
	for (size_t i = 0; i < Len; i++)
	{
		acc |= pData[i];
	}
	return acc == 0;
}

// True when big-endian A < B. Constant-time over the full width: it does not
// stop at the first differing byte.
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
		// Take the most significant differing byte: only update while equal so
		// far (neither lt nor gt yet decided).
		int decided = lt | gt;
		lt |= curLt & (decided ^ 1);
		gt |= curGt & (decided ^ 1);
	}
	return lt != 0;
}

// True when Coord is a valid non-zero field element: non-zero and less than p.
bool P256NonzeroFieldElement(const uint8_t Coord[P256_BYTES])
{
	if (P256IsZero(Coord, P256_BYTES))
	{
		return false;
	}
	return P256LessBe(Coord, s_P256Prime, P256_BYTES);
}

// True when Scalar is a valid private scalar: non-zero and strictly less than n.
bool P256ScalarInRange(const uint8_t Scalar[P256_BYTES])
{
	if (P256IsZero(Scalar, P256_BYTES))
	{
		return false;
	}
	return P256LessBe(Scalar, s_P256Order, P256_BYTES);
}

// Draw a uniform private scalar in [1, n-1] by rejection sampling: pull 32 fresh
// bytes and keep the first that is in range. Bounded attempts; the rejection
// probability per draw is far below one half.
bool P256RandomScalar(uint8_t Scalar[P256_BYTES])
{
	for (int attempt = 0; attempt < 64; attempt++)
	{
		if (!RngGet(Scalar, P256_BYTES))
		{
			return false;
		}
		if (P256ScalarInRange(Scalar))
		{
			return true;
		}
	}
	memset(Scalar, 0, P256_BYTES);
	return false;
}

// Regularize a scalar for a fixed-length window multiply: produce R such that
// its top bit is set, so the multiply runs a constant number of iterations
// independent of the scalar bit length. R is one byte wider than K. When K is
// odd, R = K; when K is even, R = K + n, which is odd because n is odd. The
// multiply routine accounts for the added order.
void P256RegularizeScalar(const uint8_t K[P256_BYTES], uint8_t R[P256_BYTES + 1U])
{
	int carry = 0;
	int odd = K[P256_BYTES - 1U] & 1;

	// R = K, or R = K + n when K is even. Add n conditionally, constant time.
	for (int i = (int)P256_BYTES - 1; i >= 0; i--)
	{
		int addend = odd ? 0 : s_P256Order[i];
		int sum = (int)K[i] + addend + carry;
		R[i + 1] = (uint8_t)(sum & 0xFF);
		carry = sum >> 8;
	}
	R[0] = (uint8_t)carry;
}

} // extern "C"
