/**-------------------------------------------------------------------------
@file	crypto_ra4m1.h

@brief	RA4M1 SCE5 crypto engines: AES-128/256 and TRNG.

Two engines on the one SCE5 block: CryptoRa4m1Rng (TRNG) and CryptoRa4m1Aes
(AES-128/256 ECB, CBC, CTR both directions; CMAC, CCM and GCM composed over
the block). The SCE5 has no documented register interface; the access
sequences come from the Renesas plain-key primitives in crypto_sce5.c.

@author	Hoang Nguyen Hoan
@date	Oct. 25, 2026

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
#ifndef __CRYPTO_RA4M1_H__
#define __CRYPTO_RA4M1_H__

#include <stdint.h>
#include <stddef.h>
#include <atomic>

#include "crypto/crypto_softaes.h"

/** @addtogroup Crypto
  * @{
  */

// The SCE5 is one peripheral shared by the random source and the AES engine.
// Both objects serialize on one lock; a random draw cannot run inside an AES
// operation and an AES operation cannot start during a random draw. SHA-256
// and P-256 have no SCE5 primitive on this part and stay in software.

class CryptoRa4m1Rng : public RngEngine
{
public:
	CryptoRa4m1Rng()
	{
		vbValid = false;
	}
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;
	bool IsSecure() const override;
	int SelfTest() override;
};

#define CRYPTO_RA4M1_RNG_MEMSIZE	sizeof(CryptoRa4m1Rng)

CryptoRa4m1Rng *CryptoRa4m1RngCreate(void *pMem, size_t MemSize);

// Complete AES accelerator: ECB, CBC and CTR with AES-128 and AES-256 keys in
// both directions on the SCE5; CMAC, CCM and GCM are composed by
// CryptoAesModes over the ECB block inside one operation.
class CryptoRa4m1Aes : public CryptoAesModes
{
public:
	CryptoRa4m1Aes()
	{
		vbValid = false;
	}
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len,
						 uint8_t *pOut) override;
	int SelfTest() override;
protected:
	bool AesOpBegin() override;
	void AesOpEnd() override;
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
					   uint8_t Out[16]) override;
};

#define CRYPTO_RA4M1_AES_MEMSIZE	sizeof(CryptoRa4m1Aes)

CryptoRa4m1Aes *CryptoRa4m1AesCreate(void *pMem, size_t MemSize);

/** @} */

#endif // __CRYPTO_RA4M1_H__
