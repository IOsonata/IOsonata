/**-------------------------------------------------------------------------
@file	crypto_rng_nrf.h

@brief	Nordic nRF hardware random generator on the OO engine tree.

		Declares CryptoRngNrf, the Nordic implementation of the RngEngine facet.
		It is security grade (IsSecure() returns true): on nRF54L/nRF54H it draws
		from the CRACEN NIST SP800-90A CTR-DRBG seeded by the CRACEN TRNG; on
		nRF51/52/53/91 it draws from the RNG peripheral with bias correction, or
		from the SoftDevice entropy pool while the stack is enabled.

		A part with no RNG peripheral does not provide this engine; on such a
		part only the software CryptoSoftRng (a PRNG) is available, and security
		key generation must not run.

		The free function RngGet is kept as a thin shim over the singleton
		instance so existing callers (SMP, the direct hardware crypto providers)
		keep working during the migration to the engine object.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_RNG_NRF_H__
#define __CRYPTO_RNG_NRF_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

/// @brief	Nordic hardware random generator implementing RngEngine.
///
/// Security grade: IsSecure() is true. Random draws hardware entropy through
/// the per-part path selected at build time. Stateless beyond the Device
/// lifecycle; the underlying peripheral holds the state.
class CryptoRngNrf : public RngEngine {
public:
	CryptoRngNrf() { vbValid = false; }

	// Device lifecycle. Enable brings up the hardware RNG (CTR-DRBG init on the
	// CRACEN parts); it is idempotent.
	bool Enable() override;
	void Disable() override {}
	void Reset() override {}

	// RngEngine: fill pOut with Len hardware random bytes.
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;

	// Hardware entropy: security grade.
	bool IsSecure() const override { return true; }
};

/// @brief	Singleton accessor for the Nordic hardware RNG engine. Constructs on
///			first use in internal static storage (no allocation) and returns the
///			same instance thereafter. Returns the engine even before Enable; the
///			caller or RngGet enables it on first draw.
CryptoRngNrf *CryptoRngNrfInstance(void);

//-----------------------------------------------------------------------------
// Compatibility C-shim. Existing callers (SMP, the direct hardware crypto
// providers) use these free functions. They forward to the singleton engine so
// there is one hardware path. Declared here so the RNG unit needs only this
// header; the same names are also declared in the legacy crypto.h for callers
// that have not moved to the engine facet. Retire as callers migrate.
//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

/// Bring up the hardware RNG. Idempotent. Returns false when unavailable.
bool RngInit(void);

/// Fill pBuff with Len hardware random bytes. Returns false on failure.
bool RngGet(uint8_t *pBuff, size_t Len);

#ifdef __cplusplus
}
#endif

/** @} */

#endif // __CRYPTO_RNG_NRF_H__
