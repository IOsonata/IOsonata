/**-------------------------------------------------------------------------
@file	crypto_uecc.h

@brief	Software P-256 crypto engine over micro-ecc.

		Declares CryptoUecc, the software implementation of the KeyAgreeEngine
		(ECDH) and SignEngine (ECDSA) facets. A hardware public-key engine
		(Silex BA414EP, Arm CryptoCell) inherits the same facets and overrides
		the operations; a build with no accelerator uses this class directly.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_UECC_H__
#define __CRYPTO_UECC_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup Crypto
  * @{
  */

/// @brief	Software P-256 engine (micro-ecc) implementing ECDH and ECDSA.
///
/// The per-instance private key lives in caller-provided key context storage
/// (KeyCtxSize() bytes), so several exchanges can run against one engine object.
/// The private key is single-use: it is wiped on Agree and on every failure.
class CryptoUecc : public KeyAgreeEngine, public SignEngine {
public:
	/// Per-instance key context the caller provides to KeyGen/Agree.
	struct KeyCtx {
		uint8_t PrivKey[32];	//!< P-256 private key, retained keygen to DH
		bool    bKeyValid;		//!< true only while PrivKey holds a usable key
	};

	CryptoUecc() { vbValid = false; }

	// Device lifecycle (software engine).
	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	// KeyAgreeEngine.
	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPubKey, uint8_t *pSharedX) override;

	// SignEngine.
	CRYPTO_STATUS Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
					   const uint8_t *pHash, size_t HashLen,
					   uint8_t *pSig) override;
	CRYPTO_STATUS Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
						 const uint8_t *pHash, size_t HashLen,
						 const uint8_t *pSig) override;

	// Known-answer self-test: BLE spec P-256 DH vector.
	int SelfTest() override;
};

/// Bytes of storage CryptoUeccCreate needs.
#define CRYPTO_UECC_MEMSIZE		sizeof(CryptoUecc)

/// @brief	Construct a CryptoUecc in caller-provided storage (no allocation).
/// @param	pMem	Buffer of at least CRYPTO_UECC_MEMSIZE bytes, 8-byte aligned.
/// @param	MemSize	Size of pMem.
/// @return	Ready engine pointer, or nullptr on a too-small buffer.
CryptoUecc *CryptoUeccCreate(void *pMem, size_t MemSize);

/** @} */

#endif // __CRYPTO_UECC_H__
