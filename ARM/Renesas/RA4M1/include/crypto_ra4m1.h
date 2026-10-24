/**-------------------------------------------------------------------------
@file	crypto_ra4m1.h
@brief	RA4M1 SCE5 AES and secure random overrides.

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
#ifndef __CRYPTO_RA4M1_H__
#define __CRYPTO_RA4M1_H__
#include "crypto/crypto.h"

// SCE5 AES/TRNG with inherited software SHA and P-256.
class CryptoRa4m1 : public Crypto {
public:
	bool Enable() override;
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
		const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len,
		uint8_t *pOut) override;
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;
	bool IsSecure() const override;
protected:
	bool AesOpBegin() override;
	void AesOpEnd() override;
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
		uint8_t Out[16]) override;
private:
	bool Block(const uint8_t *pKey, size_t KeyLen, bool Encrypt,
		const uint8_t In[16], uint8_t Out[16]);
};
#endif
