/**-------------------------------------------------------------------------
@file	crypto_cc3xx.h

@brief	CryptoCc3xx: P-256 ECDH on the Arm CryptoCell CC3xx PKA.

		A KeyAgreeEngine on the OO crypto tree, driving the CC3xx public-key
		accelerator directly (no vendor runtime blob). The current target is the
		nRF52840 CryptoCell CC310; the vendor implements the cc3xx_intrf.h surface with the
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

#ifdef __cplusplus

/// @brief	CryptoCell CC3xx P-256 ECDH as a KeyAgreeEngine.
///
/// KeyGen generates a local key pair (generator multiply); Agree derives the
/// shared secret (peer point multiply). The private scalar is held in caller
/// key context between the two. SelfTest runs a fixed known-answer P-256 DH.
class CryptoCc3xx : public KeyAgreeEngine {
public:
	CryptoCc3xx() { vpRng = nullptr; }

	/// Per-instance key context: the P-256 private scalar and a validity flag.
	struct KeyCtx {
		uint8_t PrivKey[32];	//!< P-256 private scalar, KeyGen to Agree
		bool    bKeyValid;		//!< true while PrivKey holds a usable key
	};

	/// Inject the security-grade random source. Key generation and point
	/// blinding draw from it; without a secure engine those operations fail
	/// closed.
	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	/// @brief	Initialise the engine on a crypto interface.
	///
	/// Binds the transport (normally Cc310IntrfInstance) and the random
	/// source, then enables the hardware.
	bool Init(DeviceIntrf * const pIntrf, RngEngine *pRng);

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

protected:
	RngEngine *vpRng;			//!< Security-grade random source
};

/// Storage the caller reserves for a CryptoCc3xx engine object (placement-new
/// target for CryptoCc3xxCreate). Defined from the class itself so it can
/// never fall behind the real object size.
#define CRYPTO_CC3XX_MEMSIZE		sizeof(CryptoCc3xx)

/// @brief	Construct a CryptoCc3xx engine in caller storage (no allocation).
///
/// @param	pMem	Caller storage, at least CRYPTO_CC3XX_MEMSIZE bytes, aligned
///					to alignof(CryptoCc3xx): declare the arena alignas(CryptoCc3xx).
/// @param	MemSize	Size of pMem in bytes.
/// @param	pRng	Security-grade random source for key generation and blinding.
///
/// @return	The engine, or nullptr on a bad arena or if CC3xx bring-up fails.
CryptoCc3xx *CryptoCc3xxCreate(void *pMem, size_t MemSize, RngEngine *pRng);

#endif // __cplusplus

#endif // __CRYPTO_CC3XX_H__
