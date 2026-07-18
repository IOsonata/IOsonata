/**-------------------------------------------------------------------------
@file	crypto_rng_stm32.h

@brief	STM32 hardware random generator implementing RngEngine.

		OO engine over the STM32 RNG peripheral, direct CMSIS register access
		(no STM32 HAL or LL). Mirrors the Nordic CryptoRngNrf: a singleton
		engine on the RngEngine facet; entropy is drawn through Random.

		STM32F0/F030 has no hardware RNG and must not link the implementation.

@author	Hoang Nguyen Hoan
@date	Jul 2026

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
///			caller or the first Random draw enables it.
CryptoRngStm32 *CryptoRngStm32Instance(void);

#endif // __cplusplus

//-----------------------------------------------------------------------------
// The RNG is reached through the RngEngine facet: hold a CryptoRngStm32 (or
// the singleton CryptoRngStm32Instance) and call Random. Consumers that need
// entropy take an injected RngEngine pointer, so a software generator can
// stand in for the hardware one. There is no free-function entropy shim.
//-----------------------------------------------------------------------------

/** @} */

#endif // __CRYPTO_RNG_STM32_H__
