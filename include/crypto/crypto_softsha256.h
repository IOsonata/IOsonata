/**-------------------------------------------------------------------------
@file	crypto_softsha256.h

@brief	Software SHA-256 crypto engine: digest, streaming and HMAC.

		Declares CryptoSoftSha256, the software implementation of the HashEngine
		facet (SHA-256, FIPS 180-4, one-shot and streaming) and the MacEngine
		facet (HMAC-SHA-256, RFC 2104). HMAC is computed over the virtual
		streaming hash calls, so a hardware digest block that overrides them
		gets hardware-backed HMAC with no further code. A build with no
		accelerator uses this class directly.

		The digest is a 32 byte string in the natural order, matching the other
		engines and the DFU and signature-verify consumers.

@author	Hoang Nguyen Hoan
@date	Jul. 16, 2026

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
#ifndef __CRYPTO_SOFTSHA256_H__
#define __CRYPTO_SOFTSHA256_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

/// @brief	Software SHA-256 engine implementing HashEngine and HMAC.
///
/// Stateless beyond the Device lifecycle: one-shot Hash and HMAC carry no
/// state between calls; streaming digests live in caller-provided context
/// storage of HashCtxSize() bytes, so one engine object serves any number of
/// messages and interleaved streams.
class CryptoSoftSha256 : public HashEngine, public MacEngine {
public:
	CryptoSoftSha256() { vbValid = false; }

	// Device lifecycle (software engine).
	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	// HashEngine: one-shot SHA-256 digest.
	CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg,
					   size_t Len, uint8_t *pDigest) override;

	// HashEngine: streaming SHA-256 over caller context storage.
	size_t HashCtxSize() const override;
	CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx) override;
	CRYPTO_STATUS HashUpdate(void *pHashCtx, const uint8_t *pMsg,
							 size_t Len) override;
	CRYPTO_STATUS HashFinal(void *pHashCtx, uint8_t *pDigest) override;

	// MacEngine: HMAC-SHA-256 (RFC 2104) over the virtual streaming hash.
	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
					  const uint8_t *pMsg, size_t Len,
					  uint8_t *pMac, size_t MacLen) override;

	// Known-answer self-test: FIPS 180-4 SHA-256 and RFC 4231 HMAC vectors.
	int SelfTest() override;
};

/// Bytes of storage CryptoSoftSha256Create needs.
#define CRYPTO_SOFTSHA256_MEMSIZE		sizeof(CryptoSoftSha256)

/// @brief	Construct a CryptoSoftSha256 in caller-provided storage (no
///			allocation).
/// @param	pMem	Buffer of at least CRYPTO_SOFTSHA256_MEMSIZE bytes, aligned.
/// @param	MemSize	Size of pMem.
/// @return	Ready engine pointer, or nullptr on a too-small buffer.
CryptoSoftSha256 *CryptoSoftSha256Create(void *pMem, size_t MemSize);

/** @} */

#endif // __CRYPTO_SOFTSHA256_H__
