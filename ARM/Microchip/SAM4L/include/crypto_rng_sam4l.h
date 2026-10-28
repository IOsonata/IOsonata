/**-------------------------------------------------------------------------
@file	crypto_rng_sam4l.h

@brief	SAM4L TRNG random number engine.

Every SAM4L part has the TRNG. The SAM4LC library uses it next to the AESA
engine (crypto_sam4l.cpp), for Crypto::Random and the AESA countermeasure
seed. SAM4LS parts have no AESA: the SAM4LS library uses this engine for
Crypto::Random and the software engines for the other operations
(crypto_default_sam4ls.cpp). Both libraries share one engine object.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __CRYPTO_RNG_SAM4L_H__
#define __CRYPTO_RNG_SAM4L_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

#ifdef __cplusplus

class CryptoRngSam4l : public RngEngine {
public:
	CryptoRngSam4l()
	{
		vbValid = false;
	}
	/// Clock the TRNG through the PBA bridge, start it and wait for its first
	/// word. False when the clock does not start or no word arrives.
	bool Enable() override;
	/// Stop the TRNG.
	void Disable() override;
	void Reset() override;
	/// Fill pOut with TRNG words. The whole buffer is wiped and the request
	/// fails when a word does not arrive in time; BUSY while another request
	/// uses the TRNG.
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;
	bool IsSecure() const override
	{
		return vbValid;
	}
};

/// @brief	The TRNG engine of the library. The crypto provider enables it.
CryptoRngSam4l &CryptoRngSam4lInstance(void);

#endif // __cplusplus

/** @} */

#endif // __CRYPTO_RNG_SAM4L_H__
