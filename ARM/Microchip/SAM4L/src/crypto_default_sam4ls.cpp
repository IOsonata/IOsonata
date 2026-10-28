/**-------------------------------------------------------------------------
@file	crypto_default_sam4ls.cpp

@brief	SAM4LS crypto: TRNG random numbers, software algorithms.

SAM4LS parts have the SAM4L TRNG but no AESA engine: the Microchip SAM4L
device pack defines the AESA instance and its interrupt (21) on SAM4LC parts
only. Crypto::Random uses the TRNG through CryptoRngSam4l, so IsSecure is
true and key generation and signing are available. Cipher, Seal, Open, Mac,
Hash and the P-256 operations are the inherited software implementations,
AES-128 and AES-256 included. Init fails when the TRNG does not start; it
never selects a software random generator instead.

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
#include "crypto/crypto.h"
#include "crypto_rng_sam4l.h"

bool Crypto::Init()
{
	if (vbValid)
	{
		return true;
	}
	if (!CryptoRngSam4lInstance().Enable())
	{
		return false;
	}
	vbValid = true;
	return true;
}

CRYPTO_STATUS Crypto::Random(uint8_t *pOut, size_t Len)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return CryptoRngSam4lInstance().Random(pOut, Len);
}

bool Crypto::IsSecure() const
{
	if (!vbValid)
	{
		return false;
	}
	return CryptoRngSam4lInstance().IsSecure();
}
