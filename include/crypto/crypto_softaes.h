/**-------------------------------------------------------------------------
@file	crypto_softaes.h

@brief	Software AES-128 crypto engine: cipher, CMAC, CCM and GCM.

		Declares CryptoSoftAes, the software implementation of the CipherEngine
		(AES-128 ECB/CTR/CBC), MacEngine (AES-CMAC) and AeadEngine (AES-CCM,
		AES-GCM) facets. It is the software base of the symmetric facets: a
		hardware AES block (Silex CryptoMaster, Arm CryptoCell) inherits the
		same facets, overrides Cipher and the AesEcbEncrypt block primitive,
		and the inherited CMAC, CCM and GCM then run over that hardware AES
		inside one AesOpBegin/AesOpEnd bracket per operation. A build with no
		accelerator uses this class directly.

		AES-128 only. All values are byte strings in the natural order (the AES
		state is column-major per FIPS-197), matching the SMP toolbox and the
		other engines so keys and blocks pass through with no reordering.

@author	Hoang Nguyen Hoan
@date	Jul. 15, 2026

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
/// over the virtual AesEcbEncrypt primitive between the AesOpBegin/AesOpEnd hooks,
/// so a hardware subclass gets hardware-backed CMAC under one operation-level
/// acquisition by overriding those three.
class CryptoSoftAes : public CipherEngine, public MacEngine,
					  public AeadEngine {
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

	// AeadEngine: AES-CCM (RFC 3610) over the AesEcbEncrypt primitive inside
	// one AesOpBegin/AesOpEnd bracket, so a hardware subclass runs the whole
	// seal or open under one acquisition. Nonce 7..13 bytes, tag an even
	// 4..16 bytes, AAD shorter than 65280 bytes.
	CRYPTO_STATUS Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
					   const uint8_t *pNonce, size_t NonceLen,
					   const uint8_t *pAad, size_t AadLen,
					   const uint8_t *pMsg, size_t Len,
					   uint8_t *pOut, uint8_t *pTag, size_t TagLen) override;
	CRYPTO_STATUS Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
					   const uint8_t *pNonce, size_t NonceLen,
					   const uint8_t *pAad, size_t AadLen,
					   const uint8_t *pMsg, size_t Len,
					   uint8_t *pOut, const uint8_t *pTag,
					   size_t TagLen) override;

	// Known-answer self-test: FIPS-197 AES-128 ECB and RFC 4493 CMAC vectors.
	int SelfTest() override;

private:
	// CCM authentication: the CBC-MAC over B0, AAD and message, masked with
	// the A0 keystream. Runs inside the caller's AesOpBegin/AesOpEnd bracket.
	bool CcmMacTag(const uint8_t *pKey, const uint8_t *pNonce, size_t NonceLen,
				   const uint8_t *pAad, size_t AadLen,
				   const uint8_t *pMsg, size_t Len, size_t TagLen,
				   uint8_t Tag[16]);

protected:
	// Hooks for a hardware subclass to bracket the whole CMAC in one
	// operation-level acquisition. Mac calls AesOpBegin once before any block,
	// AesOpEnd once on every exit after a successful AesOpBegin. The software
	// base needs neither.
	virtual bool AesOpBegin() { return true; }
	virtual void AesOpEnd() {}

	// One AES-128 ECB block encrypt for the inherited CMAC, called only
	// between AesOpBegin and AesOpEnd. The base encrypts in software; a hardware
	// subclass overrides it and may assume the acquisition its AesOpBegin took
	// is still owned.
	virtual bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
							 uint8_t Out[16]);
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
