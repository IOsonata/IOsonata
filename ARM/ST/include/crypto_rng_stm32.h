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

class CryptoRngStm32 : public RngEngine {
public:
	CryptoRngStm32() : vbEnabled(false) {
		vbValid = false;
		atomic_flag_clear(&vOpBusy);
	}

	bool Enable() override;
	void Disable() override;
	void Reset() override;

	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;
	bool IsSecure() const override { return true; }

private:
	atomic_flag vOpBusy;
	bool vbEnabled;
};

/// @brief	Return the enabled STM32 hardware RNG singleton, or nullptr when the
///			peripheral cannot be initialized.
CryptoRngStm32 *CryptoRngStm32Instance(void);

#endif // __cplusplus

/** @} */

#endif // __CRYPTO_RNG_STM32_H__
