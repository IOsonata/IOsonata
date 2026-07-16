/**-------------------------------------------------------------------------
@file	crypto_rng_stm32.h

@brief	STM32 hardware random generator implementing RngEngine.

		OO engine over the STM32 RNG peripheral, direct CMSIS register access
		(no STM32 HAL or LL). Mirrors the Nordic CryptoRngNrf: a singleton
		engine plus a C-shim (RngGet) for callers not yet on the facet.

		STM32F0/F030 has no hardware RNG and must not link the implementation.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_RNG_STM32_H__
#define __CRYPTO_RNG_STM32_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

#ifdef __cplusplus

/// @brief	STM32 hardware random generator implementing RngEngine.
///
/// Security grade: IsSecure() is true. Random draws hardware entropy from the
/// STM32 RNG peripheral. Stateless beyond the Device lifecycle; the peripheral
/// holds the state.
class CryptoRngStm32 : public RngEngine {
public:
	CryptoRngStm32() { vbValid = false; }

	// Device lifecycle. Enable brings up the RNG peripheral clock and block;
	// it is idempotent.
	bool Enable() override;
	void Disable() override {}
	void Reset() override {}

	// RngEngine: fill pOut with Len hardware random bytes.
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;

	// Hardware entropy: security grade.
	bool IsSecure() const override { return true; }
};

/// @brief	Singleton accessor for the STM32 hardware RNG engine. Constructs on
///			first use in internal static storage (no allocation) and returns the
///			same instance thereafter. Returns the engine even before Enable; the
///			caller or RngGet enables it on first draw.
CryptoRngStm32 *CryptoRngStm32Instance(void);

#endif // __cplusplus

//-----------------------------------------------------------------------------
// Compatibility C-shim. Callers that have not moved to the engine facet use
// these free functions; they forward to the singleton engine so there is one
// hardware path.
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

#endif // __CRYPTO_RNG_STM32_H__
