/**-------------------------------------------------------------------------
@file	crypto_softaes.h

@brief	Software AES-128 crypto engine: cipher and CMAC.

		Declares CryptoSoftAes, the software implementation of the CipherEngine
		(AES-128 ECB/CTR/CBC) and MacEngine (AES-CMAC) facets. It is the software
		base of the symmetric facets: a hardware AES block (Silex CryptoMaster,
		Arm CryptoCell) inherits the same facets and overrides Cipher, and the
		inherited software CMAC then runs over that hardware AES through the
		virtual Cipher call. A build with no accelerator uses this class directly.

		AES-128 only. All values are byte strings in the natural order (the AES
		state is column-major per FIPS-197), matching the SMP toolbox and the
		other engines so keys and blocks pass through with no reordering.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_SOFTAES_H__
#define __CRYPTO_SOFTAES_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

/// @brief	Software AES-128 engine implementing Cipher and CMAC.
///
/// Stateless beyond the Device lifecycle: the key travels with each call in the
/// CryptoKey, so one engine object serves any number of keys. CMAC is computed
/// over the virtual Cipher call, so a subclass that overrides Cipher with
/// hardware AES gets hardware-backed CMAC with no further code.
class CryptoSoftAes : public CipherEngine, public MacEngine {
public:
	CryptoSoftAes() { vbValid = false; }

	// Device lifecycle (software engine).
	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	// CipherEngine: software AES-128 ECB/CTR/CBC.
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len, uint8_t *pOut) override;

	// MacEngine: AES-CMAC (RFC 4493) over this->Cipher.
	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
					  const uint8_t *pMsg, size_t Len,
					  uint8_t *pMac, size_t MacLen) override;

	// Known-answer self-test: FIPS-197 AES-128 ECB and RFC 4493 CMAC vectors.
	int SelfTest() override;
};

/// Bytes of storage CryptoSoftAesCreate needs.
#define CRYPTO_SOFTAES_MEMSIZE		sizeof(CryptoSoftAes)

/// @brief	Construct a CryptoSoftAes in caller-provided storage (no allocation).
/// @param	pMem	Buffer of at least CRYPTO_SOFTAES_MEMSIZE bytes, aligned.
/// @param	MemSize	Size of pMem.
/// @return	Ready engine pointer, or nullptr on a too-small buffer.
CryptoSoftAes *CryptoSoftAesCreate(void *pMem, size_t MemSize);

/** @} */

#endif // __CRYPTO_SOFTAES_H__
