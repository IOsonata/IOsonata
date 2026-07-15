/**-------------------------------------------------------------------------
@file	crypto_softrng.h

@brief	Software pseudo-random generator on the OO engine tree.

		Declares CryptoSoftRng, the software implementation of the RngEngine
		facet. It is a PRNG: a deterministic algorithm with no hardware entropy
		source, so it makes no security claim and IsSecure() returns false. It
		is suitable for statistical and non-security use only.

		A target MCU with a hardware entropy source provides a separate RngEngine
		whose Random draws from a DRBG or TRNG and whose IsSecure() returns true.
		Security key generation must use that hardware engine, never this one: a
		consumer generating keys checks IsSecure() and refuses to run when false.

		The generator state is caller-visible so its seed can be set for
		reproducible sequences; reseeding does not make it security grade.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_SOFTRNG_H__
#define __CRYPTO_SOFTRNG_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

/// @brief	Software PRNG implementing RngEngine. Not security grade.
///
/// Backed by a xorshift128+ generator: fast, good statistical quality, and
/// wholly deterministic from its seed. IsSecure() is false, so key generation
/// declines to use it. Seed defaults to a fixed nonzero constant; Seed() sets
/// it for reproducible test streams.
class CryptoSoftRng : public RngEngine {
public:
	CryptoSoftRng() { vbValid = false; vState0 = 0x9E3779B97F4A7C15ULL; vState1 = 0xBF58476D1CE4E5B9ULL; }

	// Device lifecycle (software engine).
	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override { vState0 = 0x9E3779B97F4A7C15ULL; vState1 = 0xBF58476D1CE4E5B9ULL; }

	// RngEngine: fill pOut with Len pseudo-random bytes.
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;

	// Not security grade: this is a PRNG with no entropy source.
	bool IsSecure() const override { return false; }

	// Set the generator seed for a reproducible sequence. A zero seed is
	// remapped to the default so the state is never all-zero.
	void Seed(uint64_t Seed0, uint64_t Seed1);

private:
	uint64_t vState0;
	uint64_t vState1;
};

/// Bytes of storage CryptoSoftRngCreate needs.
#define CRYPTO_SOFTRNG_MEMSIZE		sizeof(CryptoSoftRng)

/// @brief	Construct a CryptoSoftRng in caller-provided storage (no allocation).
CryptoSoftRng *CryptoSoftRngCreate(void *pMem, size_t MemSize);

/** @} */

#endif // __CRYPTO_SOFTRNG_H__
