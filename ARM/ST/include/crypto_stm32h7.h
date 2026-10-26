/**-------------------------------------------------------------------------
@file	crypto_stm32h7.h
@brief	STM32H7 CRYP AES-128/256 and HASH SHA-256 engines.
@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026
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
#ifndef __CRYPTO_STM32H7_H__
#define __CRYPTO_STM32H7_H__

#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"

// Shared H753 CRYP / L4S9 AES implementation. Historical class names retained.
// Polled peripheral: ECB/CBC/CTR accept AES-128/256. AES-128
// CMAC/CCM/GCM reuse the software base's constructions
// over the hardware block primitive. Separate instances share peripheral locks.
class CryptoStm32H7Aes : public CryptoAesModes
{
public:
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen, const uint8_t *pIn,
						 size_t Len, uint8_t *pOut) override;
protected:
	bool AesOpBegin() override;
	void AesOpEnd() override;
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
					   uint8_t Out[16]) override;
};

#define CRYPTO_STM32H7_AES_MEMSIZE sizeof(CryptoStm32H7Aes)
CryptoStm32H7Aes *CryptoStm32H7AesCreate(void *pMem, size_t MemSize);

// One-shot SHA-256 and HMAC use HASH. Streaming retains the software base:
// the peripheral's full context does not fit CRYPTO_HASHCTX_MAX. This keeps
// caller-owned contexts, interleaved streams and the existing API intact.
class CryptoStm32H7Hash : public CryptoSoftSha256 {
public:
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len,
					   uint8_t *pDigest) override;
	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
					  const uint8_t *pMsg, size_t Len,
					  uint8_t *pMac, size_t MacLen) override;
};

#define CRYPTO_STM32H7_HASH_MEMSIZE sizeof(CryptoStm32H7Hash)
CryptoStm32H7Hash *CryptoStm32H7HashCreate(void *pMem, size_t MemSize);

#endif
