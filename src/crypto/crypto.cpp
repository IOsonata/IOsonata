/**-------------------------------------------------------------------------
@file	crypto.cpp
@brief	Default crypto engines selected by the target library.

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
#include "crypto/crypto.h"

Crypto::Crypto()
{
	CryptoUecc::SetRng(this);
}

bool Crypto::Enable()
{
	vbValid = true;
	return true;
}

void Crypto::Disable()
{
	vbValid = false;
}

void Crypto::Reset()
{
	CryptoSoftRng::Reset();
}

int Crypto::SelfTest()
{
	if (CryptoSoftAes::SelfTest() != 0 || CryptoSoftSha256::SelfTest() != 0 ||
		CryptoUecc::SelfTest() != 0)
	{
		return -1;
	}
	return 0;
}

CRYPTO_STATUS Crypto::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
						 const uint8_t *pMsg, size_t Len,
						 uint8_t *pMac, size_t MacLen)
{
	if (Alg == CRYPTO_MAC_HMAC)
	{
		return CryptoSoftSha256::Mac(Alg, Key, pMsg, Len, pMac, MacLen);
	}
	return CryptoSoftAes::Mac(Alg, Key, pMsg, Len, pMac, MacLen);
}
