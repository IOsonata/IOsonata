/**-------------------------------------------------------------------------
@file	crypto_p256.h

@brief	NIST P-256 (secp256r1) constants and byte-level helpers.

		Curve parameters and constant-time byte-string helpers shared by
		P-256 engines that drive a hardware public-key accelerator directly
		(no vendor multi-curve framework). All values are big-endian 32-byte
		strings, matching the SEC1 uncompressed public-key encoding the
		CryptoDev_t interface uses.

		This unit contains no hardware or PKA register dependency and exports
		no curve constants: the order and field prime are file-local, reached
		only through the scalar and field helpers. The curve point arithmetic
		(double, add, scalar multiply) and the base point stay in each engine,
		because both are specific to the accelerator register model.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_P256_H__
#define __CRYPTO_P256_H__

#include <stdint.h>
#include <stddef.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define P256_BYTES		32U		//!< P-256 field element / scalar size in bytes

// True when every byte of the buffer is zero. Runs over the whole buffer
// (no early return) so timing does not depend on where the first set byte is.
bool P256IsZero(const uint8_t *pData, size_t Len);

// Constant-time big-endian compare: true when pA < pB over Len bytes. Timing
// does not depend on the operand values.
bool P256LessBe(const uint8_t *pA, const uint8_t *pB, size_t Len);

// Big-endian 256-bit add: Out = A + B, returns the carry out (0 or 1). Out may
// alias A or B.
uint8_t P256AddBe(const uint8_t A[P256_BYTES], const uint8_t B[P256_BYTES],
				  uint8_t Out[P256_BYTES]);

// Draw a uniform scalar in [1, n-1] into Scalar via RngGet with rejection
// sampling (reject zero and any value >= n). Returns false and wipes Scalar
// when RngGet fails or the attempt budget is exhausted.
bool P256RandomScalar(uint8_t Scalar[P256_BYTES]);

// True when Scalar is a valid private scalar: non-zero and less than the order
// n. Constant-time.
bool P256ScalarInRange(const uint8_t Scalar[P256_BYTES]);

// True when Coord is a non-zero field element: 0 < Coord < p. Constant-time.
// Used for a projective randomization factor, which must not be zero.
bool P256NonzeroFieldElement(const uint8_t Coord[P256_BYTES]);

// Regularize a scalar to a fixed-length representation for a constant-time
// ladder: R is 33 bytes, R[0] is always 1, and R[1..32] hold K + n or K + 2n
// (whichever sets the carry) big-endian. Both (K+n) and (K+2n) reduce to K
// times a point of order n, so the choice does not change the result. Wipes its
// own temporaries.
void P256RegularizeScalar(const uint8_t K[P256_BYTES], uint8_t R[P256_BYTES + 1U]);

// Bit BitNo (0 = least significant) of a 33-byte regularized scalar from
// P256RegularizeScalar.
uint32_t P256RegularBit(const uint8_t Scalar[P256_BYTES + 1U], uint32_t BitNo);

#ifdef __cplusplus
}
#endif

#endif // __CRYPTO_P256_H__
