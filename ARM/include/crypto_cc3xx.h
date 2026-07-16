/**-------------------------------------------------------------------------
@file	crypto_cc3xx.h

@brief	CryptoCc3xx: P-256 ECDH on the Arm CryptoCell CC3xx PKA.

		A KeyAgreeEngine on the OO crypto tree, driving the CC3xx public-key
		accelerator directly (no vendor runtime blob). The current target is the
		nRF52840 CryptoCell CC310; the target supplies crypto_cc3xx.h with the
		register base and the enable and disable operations.

		The private scalar lives in caller key context (KeyCtxSize() bytes),
		single use by default and multi-peer with bKeepKey, matching the other
		P-256 engines (CryptoUecc, Ba414ep).

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_CC3XX_H__
#define __CRYPTO_CC3XX_H__

#include <stdint.h>

#include "crypto/icrypto.h"

/// Storage the caller reserves for a CryptoCc3xx engine object (placement-new
/// target for CryptoCc3xxCreate). A static assert in the implementation checks
/// it against the real object size.
#define CRYPTO_CC3XX_MEMSIZE		64U

#ifdef __cplusplus

/// @brief	CryptoCell CC3xx P-256 ECDH as a KeyAgreeEngine.
///
/// KeyGen generates a local key pair (generator multiply); Agree derives the
/// shared secret (peer point multiply). The private scalar is held in caller
/// key context between the two. SelfTest runs a fixed known-answer P-256 DH.
class CryptoCc3xx : public KeyAgreeEngine {
public:
	/// Per-instance key context: the P-256 private scalar and a validity flag.
	struct KeyCtx {
		uint8_t PrivKey[32];	//!< P-256 private scalar, KeyGen to Agree
		bool    bKeyValid;		//!< true while PrivKey holds a usable key
	};

	bool Enable() override;
	void Disable() override;
	void Reset() override;

	int SelfTest() override;

	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPubKey, uint8_t *pSharedX,
						bool bKeepKey = false) override;
};

/// @brief	Construct a CryptoCc3xx engine in caller storage (no allocation).
///
/// @param	pMem	Caller storage, at least CRYPTO_CC3XX_MEMSIZE bytes, uint32_t
///					aligned.
/// @param	MemSize	Size of pMem in bytes.
///
/// @return	The engine, or nullptr on a bad arena or if CC3xx bring-up fails.
CryptoCc3xx *CryptoCc3xxCreate(void *pMem, size_t MemSize);

#endif // __cplusplus

#endif // __CRYPTO_CC3XX_H__
