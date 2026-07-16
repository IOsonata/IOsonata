/**-------------------------------------------------------------------------
@file	crypto_softsha256.h

@brief	Software SHA-256 crypto engine.

		Declares CryptoSoftSha256, the software implementation of the HashEngine
		facet (SHA-256, FIPS 180-4). Unkeyed digest: no CryptoKey, one engine
		object serves any number of messages. A hardware digest block can offer
		the same facet and override Hash; a build with no accelerator uses this
		class directly.

		The digest is a 32 byte string in the natural order, matching the other
		engines and the DFU and signature-verify consumers.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_SOFTSHA256_H__
#define __CRYPTO_SOFTSHA256_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

/// @brief	Software SHA-256 engine implementing the HashEngine facet.
///
/// Stateless beyond the Device lifecycle: each Hash call runs a full one-shot
/// digest, so one engine object serves any number of messages.
class CryptoSoftSha256 : public HashEngine {
public:
	CryptoSoftSha256() { vbValid = false; }

	// Device lifecycle (software engine).
	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	// HashEngine: one-shot SHA-256 digest.
	CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg,
					   size_t Len, uint8_t *pDigest) override;

	// Known-answer self-test: FIPS 180-4 SHA-256 vectors.
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
