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
			// RngGet may have partially filled the buffer with real entropy;
			// wipe it so no key material leaks on failure.
			CryptoSecureWipe(Scalar, P256_BYTES);
			return false;
		}
		if (P256ScalarInRange(Scalar))
		{
			return true;
		}
	}
	// Exhausted attempts. Wipe through volatile so the clear is not optimized
	// away (a plain memset here is dead-store eliminable).
	CryptoSecureWipe(Scalar, P256_BYTES);
	return false;
}

// Big-endian add: Out = A + B, returns the carry out of the top byte.
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

// Regularize a scalar to a fixed 257-bit length with bit 256 always set, for a
// constant-time fixed-position ladder that seeds its accumulator with P (the
// implicit set bit 256). For a scalar k in [1, n), exactly one of k+n and k+2n
// lands in [2^256, 2^257): add n once, and if that did not carry into bit 256,
// add n again. R holds the low 256 bits of the chosen value with R[0] = 1
// marking the set bit 256, so the ladder computes k*P and not (2^256 + k)*P.
// Constant time: both sums are always computed and selected by a mask.
void P256RegularizeScalar(const uint8_t K[P256_BYTES], uint8_t R[P256_BYTES + 1U])
{
	uint8_t kPlusN[P256_BYTES];
	uint8_t kPlus2N[P256_BYTES];
	const uint8_t carry1 = P256AddBe(K, s_P256Order, kPlusN);
	(void)P256AddBe(kPlusN, s_P256Order, kPlus2N);
	// For k in [1, n) one of the two sums always carries into bit 256; carry1
	// selects k+n, otherwise k+2n.
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

// Extract bit BitNo of a regularized big-endian scalar (one extra leading byte).
uint32_t P256RegularBit(const uint8_t Scalar[P256_BYTES + 1U], uint32_t BitNo)
{
	const uint32_t byteFromEnd = BitNo / 8U;
	return (Scalar[P256_BYTES - byteFromEnd] >> (BitNo & 7U)) & 1U;
}

} // extern "C"
