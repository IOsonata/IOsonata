/**-------------------------------------------------------------------------
@file	crypto.h
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
#ifndef __CRYPTO_H__
#define __CRYPTO_H__

#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_softrng.h"
#include "crypto/crypto_uecc.h"

/// Software crypto device. Target libraries derive from this class and replace
/// accelerated operations. Keys and streaming contexts remain caller-owned.
/// The software RNG is deterministic: key generation and signing require a
/// target with secure entropy. Verify does not require secure entropy.
class Crypto : public CryptoSoftAes, public CryptoSoftSha256,
			   public CryptoUecc, public CryptoSoftRng {
public:
	Crypto();
	Crypto(const Crypto &) = delete;
	Crypto &operator=(const Crypto &) = delete;

	bool Enable() override;
	void Disable() override;
	void Reset() override;
	int SelfTest() override;
	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
					  const uint8_t *pMsg, size_t Len,
					  uint8_t *pMac, size_t MacLen) override;
};

/// Common caller-owned arena for the largest in-tree target crypto object.
/// Each target checks its size and alignment at compile time. No heap or
/// singleton is used. Construct after system initialization; do not copy the
/// object or reuse its arena while operations or caller contexts are active.
#define CRYPTO_MEMSIZE		1024U
#define CRYPTO_MEMALIGN		alignof(uint64_t)

/// Construct and enable the MCU library's derived Crypto object in pMem.
/// Returns nullptr for invalid storage or initialization failure. Hardware
/// initialization failure never selects software instead. Call Disable() and
/// the virtual destructor pCrypto->~Crypto() before reusing caller storage.
Crypto *CryptoCreate(void *pMem, size_t MemSize);

#endif
