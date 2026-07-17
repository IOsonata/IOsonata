/**-------------------------------------------------------------------------
@file	crypto_cc3xx.h

@brief	CryptoCc3xx: P-256 ECDH on the Arm CryptoCell CC3xx PKA.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_CC3XX_H__
#define __CRYPTO_CC3XX_H__

#include <stdint.h>

#include "crypto/icrypto.h"
#include "cc3xx_intrf.h"

#ifdef __cplusplus

class CryptoCc3xx : public KeyAgreeEngine {
public:
	CryptoCc3xx() { vbValid = false; vpRng = nullptr; }

	struct KeyCtx {
		uint8_t PrivKey[32];
		bool bKeyValid;
	};

	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	// The engine needs OpHold/OpRelease in addition to the DeviceIntrf transfer
	// operations, so construction accepts only the concrete CC3xx interface.
	bool Init(Cc3xxIntrf * const pIntrf, RngEngine *pRng)
	{
		return Init((DeviceIntrf *)pIntrf, pRng);
	}

	bool Enable() override;
	void Disable() override;
	void Reset() override;
	int SelfTest() override;

	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	void KeyReset(void *pKeyCtx) override
	{
		if (pKeyCtx != nullptr)
		{
			CryptoSecureWipe(pKeyCtx, sizeof(KeyCtx));
		}
	}
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
						const uint8_t *pPeerPubKey, uint8_t *pSharedX,
						bool bKeepKey = false) override;

protected:
	RngEngine *vpRng;

private:
	// Implementation entry retained for the translation unit. External callers
	// cannot pass an unrelated DeviceIntrf and rely on an unchecked downcast.
	bool Init(DeviceIntrf * const pIntrf, RngEngine *pRng);
};

#define CRYPTO_CC3XX_MEMSIZE		sizeof(CryptoCc3xx)

CryptoCc3xx *CryptoCc3xxCreate(void *pMem, size_t MemSize, RngEngine *pRng);

#endif // __cplusplus

#endif // __CRYPTO_CC3XX_H__
