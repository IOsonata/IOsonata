/**-------------------------------------------------------------------------
@file	crypto_stm32wba.h

@brief	Crypto engines for the STM32WBA hardware blocks: AES, PKA and RNG.

Three independent engines, one per peripheral, each a plain driver of its
block with no Bluetooth code or header:

 - CryptoWbaRng	true random source (RngEngine)
 - CryptoWbaAes	AES-128 ECB, CBC and CTR in the peripheral; CMAC, CCM and
				GCM built on the single block primitive through the
				CryptoSoftAes hooks (CipherEngine, MacEngine, AeadEngine)
 - CryptoWbaPka	P-256 key generation, ECDH, ECDSA sign and verify in the
				public key accelerator (KeyAgreeEngine, SignEngine)

All three run in polled mode. They use the AES, PKA and RNG instance aliases of
the device header, the view the build selects, and leave the secure AES (SAES)
alone.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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
#ifndef __CRYPTO_STM32WBA_H__
#define __CRYPTO_STM32WBA_H__

#include <stdint.h>
#include <stddef.h>
#include <atomic>

#include "crypto/icrypto.h"
#include "crypto/crypto_softaes.h"

/** @addtogroup Crypto
  * @{
  */

/// @brief	STM32WBA true random number generator.
///
/// Runs the generator from the 16 MHz HSI kernel clock with the NIST
/// compliant configuration of the reference HAL. A seed or clock error
/// restarts the generator and the read is retried; when the generator does
/// not recover the call fails and the output is cleared.
class CryptoWbaRng : public RngEngine {
public:
	CryptoWbaRng() { vbValid = false; std::atomic_flag_clear(&vOpBusy); }

	bool Enable() override;
	void Disable() override;
	void Reset() override;

	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override;
	bool IsSecure() const override { return true; }

	/// Sanity check of the live source: two draws must differ and neither may
	/// be constant. 0 on pass.
	int SelfTest() override;

private:
	std::atomic_flag vOpBusy;
};

#define CRYPTO_WBA_RNG_MEMSIZE		sizeof(CryptoWbaRng)

/// Construct a CryptoWbaRng in caller storage and enable it. Null on a
/// too-small or misaligned buffer, or when the generator does not start.
CryptoWbaRng *CryptoWbaRngCreate(void *pMem, size_t MemSize);

/// @brief	STM32WBA AES-128 engine.
///
/// ECB, CBC and CTR run in the peripheral, decryption of ECB and CBC
/// included (the software base has no decryption of those two). CMAC, CCM and
/// GCM are the constructions of CryptoSoftAes over the peripheral block
/// primitive, held under one acquisition per operation.
class CryptoWbaAes : public CryptoSoftAes {
public:
	CryptoWbaAes() { vbValid = false; std::atomic_flag_clear(&vOpBusy); }

	bool Enable() override;
	void Disable() override;
	void Reset() override;

	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len,
						 uint8_t *pOut) override;

	/// Known-answer test on the peripheral: FIPS-197 and RFC 4493 (software
	/// base), then SP 800-38A ECB, CBC and CTR in both directions. 0 on pass.
	int SelfTest() override;

protected:
	bool AesOpBegin() override;
	void AesOpEnd() override;
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
					   uint8_t Out[16]) override;

private:
	bool OpAcquire() { return std::atomic_flag_test_and_set(&vOpBusy) == false; }
	void OpRelease() { std::atomic_flag_clear(&vOpBusy); }

	std::atomic_flag vOpBusy;
};

#define CRYPTO_WBA_AES_MEMSIZE		sizeof(CryptoWbaAes)

/// Construct a CryptoWbaAes in caller storage and enable it. Null on a
/// too-small or misaligned buffer.
CryptoWbaAes *CryptoWbaAesCreate(void *pMem, size_t MemSize);

/// @brief	STM32WBA public key accelerator, NIST P-256.
///
/// KeyGen and ECDSA signing need a secure RngEngine set with SetRng; the
/// private scalar and the ECDSA nonce come from it. Agree checks the peer
/// point against the curve in the accelerator before it is used. The
/// accelerator RAM is cleared after every operation.
class CryptoWbaPka : public KeyAgreeEngine, public SignEngine {
public:
	struct KeyCtx {
		uint8_t PrivKey[32];
		bool bKeyValid;
	};

	CryptoWbaPka() {
		vbValid = false;
		vpRng = nullptr;
		std::atomic_flag_clear(&vOpBusy);
	}
	void SetRng(RngEngine *pRng) { vpRng = pRng; }

	bool Enable() override;
	void Disable() override;
	void Reset() override;

	size_t KeyCtxSize() const override { return sizeof(KeyCtx); }
	size_t KeyCtxAlign() const override { return alignof(KeyCtx); }
	void KeyReset(void *pKeyCtx) override;
	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
						 uint8_t *pPubKey) override;
	CRYPTO_STATUS KeyImport(CRYPTO_CURVE Curve, void *pKeyCtx,
							const uint8_t *pPrivKey,
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

	/// Known-answer test on the accelerator: ECDH, point check, ECDSA with a
	/// fixed nonce (RFC 6979 A.2.5) and verify. 0 on pass.
	int SelfTest() override;

	/// Bring-up probe of the accelerator, independent of the key and
	/// signature paths. Words must be at least 15. Out holds:
	/// 0 point check of a valid point, accelerator Montgomery parameter
	///   (1 on the curve, 0 off it, -1 block failure, -2 no parameter)
	/// 1 error word of that check
	/// 2 the same check with the constant R^2 mod p
	/// 3 error word of that check
	/// 4 scalar multiplication of a point other than the base point
	///   (1 result as expected, 0 not, -1 block failure)
	/// 5 verification of a known signature, no point check before it
	/// 6 1 when the accelerator's R^2 mod p equals the constant
	/// 7..14 the accelerator's R^2 mod p, least significant word first
	/// Returns 0, or -1 when the probe could not run.
	int Diag(uint32_t *pOut, size_t Words);

private:
	bool OpAcquire() { return std::atomic_flag_test_and_set(&vOpBusy) == false; }
	void OpRelease() { std::atomic_flag_clear(&vOpBusy); }

	RngEngine *vpRng;
	std::atomic_flag vOpBusy;
};

static_assert(sizeof(CryptoWbaPka::KeyCtx) <= CRYPTO_KEYCTX_MAX,
			  "CryptoWbaPka key context exceeds the common consumer storage");

#define CRYPTO_WBA_PKA_MEMSIZE		sizeof(CryptoWbaPka)

/// Construct a CryptoWbaPka in caller storage and enable it. Null on a
/// too-small or misaligned buffer, or when the accelerator does not start.
CryptoWbaPka *CryptoWbaPkaCreate(void *pMem, size_t MemSize, RngEngine *pRng);

/** @} */

#endif // __CRYPTO_STM32WBA_H__
