/**-------------------------------------------------------------------------
@file	crypto_uecc.h

@brief	Software P-256 crypto engine over micro-ecc.

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

class CryptoUecc : public KeyAgreeEngine, public SignEngine {
public:
	struct KeyCtx {
		uint8_t PrivKey[32];
		bool    bKeyValid;
	};

	CryptoUecc() { vbValid = false; vpRng = nullptr; }

	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	void KeyReset(void *pKeyCtx) override;
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPubKey, uint8_t *pSharedX,
						bool bKeepKey = false) override;

	CRYPTO_STATUS Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
					   const uint8_t *pHash, size_t HashLen,
					   uint8_t *pSig) override;
	CRYPTO_STATUS Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
						 const uint8_t *pHash, size_t HashLen,
						 const uint8_t *pSig) override;

	int SelfTest() override;

private:
	RngEngine *vpRng;
};

#define CRYPTO_UECC_MEMSIZE		sizeof(CryptoUecc)

CryptoUecc *CryptoUeccCreate(void *pMem, size_t MemSize, RngEngine *pRng);

/** @} */

#endif // __CRYPTO_UECC_H__
