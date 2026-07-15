/**-------------------------------------------------------------------------
@file	crypto_uecc.cpp

@brief	Software P-256 crypto engine over micro-ecc, on the OO engine tree.

		Implements the KeyAgreeEngine (ECDH) and SignEngine (ECDSA) facets in
		software with micro-ecc. This is the software base of the public-key
		facets: a hardware engine (Silex BA414EP, Arm CryptoCell) inherits the
		same facets and overrides these operations, while a build with no
		accelerator uses this class directly.

		micro-ecc needs random bytes for key generation and signing; it obtains
		them from RngGet, the per-MCU entropy seam. RngGet is not a crypto
		capability and is not part of this engine.

		Byte order: built with the micro-ecc defaults (big-endian), matching the
		SMP toolbox (f4/f5/f6) and the over-the-air Public Key PDU, so keys and
		shared secret pass through with no reversal. A native little-endian
		micro-ecc build silently breaks every value and is refused below.

		The private key is retained between KeyGen and Agree in caller storage
		and is single-use: it is wiped on Agree and on every failure path, so no
		private material survives a completed or failed exchange.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <new>

#include "crypto/crypto_uecc.h"

#include "uECC.h"

#if defined(uECC_VLI_NATIVE_LITTLE_ENDIAN) && uECC_VLI_NATIVE_LITTLE_ENDIAN
#error "crypto_uecc requires the uECC default (big-endian) build; remove uECC_VLI_NATIVE_LITTLE_ENDIAN=1 from the project defines"
#endif

// Zeroize helper. Volatile-through so the wipe is not optimized away.
static void UeccWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0)
	{
		*p++ = 0;
	}
}

// micro-ecc takes a global RNG hook, not a per-call context, so the active
// engine's secure random source is pointed at by this file-scope handle for the
// duration of a make_key or sign call. Only a security-grade engine is ever
// installed here; the callers below enforce IsSecure() before binding it.
static RngEngine *s_pActiveRng;

static int UeccRngAdapter(uint8_t *pDest, unsigned Size)
{
	if (s_pActiveRng == nullptr)
	{
		return 0;
	}
	return (s_pActiveRng->Random(pDest, (size_t)Size) == CRYPTO_STATUS_OK) ? 1 : 0;
}

// Install the bound RNG as the micro-ecc source for one operation. Returns false
// when no secure RNG is bound, so key generation and signing fail closed rather
// than emit key material from a non-secure or absent source.
static bool UeccArmSecureRng(RngEngine *pRng)
{
	if (pRng == nullptr || !pRng->IsSecure())
	{
		return false;
	}
	s_pActiveRng = pRng;
	uECC_set_rng(UeccRngAdapter);
	return true;
}

CRYPTO_STATUS CryptoUecc::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
								 uint8_t *pPubKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr || pPubKey == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	KeyCtx *pk = (KeyCtx *)pKeyCtx;

	// Clean state: no valid key until make_key succeeds.
	pk->bKeyValid = false;
	UeccWipe(pk->PrivKey, sizeof(pk->PrivKey));

	// Fail closed unless a security-grade RNG is bound: a PRNG-derived private
	// scalar is a broken key.
	if (!UeccArmSecureRng(vpRng))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	if (uECC_make_key(pPubKey, pk->PrivKey, uECC_secp256r1()) != 1)
	{
		UeccWipe(pk->PrivKey, sizeof(pk->PrivKey));
		return CRYPTO_STATUS_FAIL;
	}
	pk->bKeyValid = true;
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoUecc::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
								const uint8_t *pPeerPubKey, uint8_t *pSharedX)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
		pPeerPubKey == nullptr || pSharedX == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	KeyCtx *pk = (KeyCtx *)pKeyCtx;

	// Fail closed: Agree before KeyGen, or a second Agree after the single-use
	// key was consumed and wiped.
	if (!pk->bKeyValid)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Reject a peer public key not on the P-256 curve before the DH, closing the
	// invalid-curve attack (CVE-2018-5383). Single-use key: wipe here too.
	if (uECC_valid_public_key(pPeerPubKey, uECC_secp256r1()) != 1)
	{
		UeccWipe(pk->PrivKey, sizeof(pk->PrivKey));
		pk->bKeyValid = false;
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t secret[32];
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;
	if (uECC_shared_secret(pPeerPubKey, pk->PrivKey, secret,
						   uECC_secp256r1()) != 1)
	{
		st = CRYPTO_STATUS_FAIL;
	}
	else
	{
		memcpy(pSharedX, secret, 32);	// shared X coordinate, big-endian
	}

	// Single-use: wipe the shared secret and the private key on every exit.
	UeccWipe(secret, sizeof(secret));
	UeccWipe(pk->PrivKey, sizeof(pk->PrivKey));
	pk->bKeyValid = false;
	return st;
}

CRYPTO_STATUS CryptoUecc::Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
							   const uint8_t *pHash, size_t HashLen,
							   uint8_t *pSig)
{
	if (Curve != CRYPTO_CURVE_P256 || Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0 ||
		pHash == nullptr || pSig == nullptr || Key.Plain.pData == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	// The ECDSA per-signature nonce must come from a security-grade RNG; a weak
	// nonce leaks the private key. Fail closed otherwise.
	if (!UeccArmSecureRng(vpRng))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	int ok = uECC_sign(Key.Plain.pData, pHash, (unsigned)HashLen, pSig,
					   uECC_secp256r1());
	return (ok == 1) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS CryptoUecc::Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
								 const uint8_t *pHash, size_t HashLen,
								 const uint8_t *pSig)
{
	if (Curve != CRYPTO_CURVE_P256 || pPubKey == nullptr ||
		pHash == nullptr || pSig == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	int ok = uECC_verify(pPubKey, pHash, (unsigned)HashLen, pSig,
						 uECC_secp256r1());
	return (ok == 1) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

int CryptoUecc::SelfTest()
{
	// BLE spec P-256 DH known vector (Vol 3 Part H 2.3.5.6.1).
	static const uint8_t privA[32] = {
		0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
		0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd };
	static const uint8_t pubB[64] = {
		0x1e,0xa1,0xf0,0xf0,0x1f,0xaf,0x1d,0x96,0x09,0x59,0x22,0x84,0xf1,0x9e,0x4c,0x00,
		0x47,0xb5,0x8a,0xfd,0x86,0x15,0xa6,0x9f,0x55,0x90,0x77,0xb2,0x2f,0xaa,0xa1,0x90,
		0x4c,0x55,0xf3,0x3e,0x42,0x9d,0xad,0x37,0x73,0x56,0x70,0x3a,0x9a,0xb8,0x51,0x60,
		0x47,0x2d,0x11,0x30,0xe2,0x8e,0x36,0x76,0x5f,0x89,0xaf,0xf9,0x15,0xb1,0x21,0x4a };
	static const uint8_t dhExpect[32] = {
		0xec,0x02,0x34,0xa3,0x57,0xc8,0xad,0x05,0x34,0x10,0x10,0xa6,0x0a,0x39,0x7d,0x9b,
		0x99,0x79,0x6b,0x13,0xb4,0xf8,0x66,0xf1,0x86,0x8d,0x34,0xf3,0x73,0xbf,0xa6,0x98 };

	uint8_t dh[32];
	int rc;
	if (uECC_shared_secret(pubB, privA, dh, uECC_secp256r1()) != 1)
	{
		rc = -1;
	}
	else
	{
		rc = (memcmp(dh, dhExpect, 32) == 0) ? 0 : -2;
	}
	UeccWipe(dh, sizeof(dh));
	return rc;
}

CryptoUecc *CryptoUeccCreate(void *pMem, size_t MemSize, RngEngine *pRng)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoUecc))
	{
		return nullptr;
	}
	CryptoUecc *p = new (pMem) CryptoUecc();
	p->SetRng(pRng);
	p->Enable();
	return p;
}
