/**-------------------------------------------------------------------------
@file	crypto_uecc.cpp

@brief	Crypto engine: micro-ecc (uECC) software ECDH over P-256.

Generic software crypto engine implementing the ECDH capability of CryptoDev_t
via micro-ecc. ECDH only - no symmetric cipher. uECC needs random bytes for key
generation and obtains them from the coredev RngGet service (coredev/rng.h);
RNG is not a crypto engine or capability.

Like a sensor that supports only one axis-type, this engine advertises only
CRYPTO_CAP_ECDH_P256. A consumer needing AES composes it with an AES engine.

Byte order: built with uECC defaults (big-endian), matching the SMP toolbox
(f4/f5/f6) and the over-the-air Public Key PDU, so keys/secret pass straight
through with no reversal.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"
#include "coredev/rng.h"

#if defined(CRYPTO_HAS_UECC) || \
	(defined(__has_include) && __has_include("uECC.h"))

#include "uECC.h"

// Per-instance secret state. Lives in App-owned pMem (CryptoCfg_t), reached
// via pDev->pDevData, or via pKeyCtx when a Cryptor shares one engine across
// instances. No file-static key: each instance is independent.
typedef struct {
	uint8_t PrivKey[32];	// P-256 private key, retained between keygen and DH
	bool    bKeyValid;	// true only while PrivKey holds a usable single-use key
} CryptoUeccData_t;

static_assert(sizeof(CryptoUeccData_t) <= CRYPTO_MEMSIZE_UECC,
			  "CRYPTO_MEMSIZE_UECC too small for CryptoUeccData_t");

// Resolve the per-instance key context: a caller-supplied pKeyCtx (a Cryptor
// sharing one engine) overrides this engine's own pDevData (dedicated engine).
static inline CryptoUeccData_t *UeccData(CryptoDev_t * const pDev, void *pKeyCtx)
{
	return (CryptoUeccData_t *)(pKeyCtx != nullptr ? pKeyCtx : pDev->pDevData);
}

// RNG source for uECC. micro-ecc requires an RNG before key generation; it is
// not part of this engine. The application (or the composing consumer) provides
// one - typically the platform hardware RNG (RngGet), declared in coredev/rng.h.

static int UeccRngAdapter(uint8_t *pDest, unsigned Size)
{
	return RngGet(pDest, Size) ? 1 : 0;
}

static CRYPTO_STATUS UeccEcdhKeyGen(CryptoDev_t * const pDev, void *pKeyCtx,
									uint8_t pPubKey[64], void *pOpCtx)
{
	(void)pOpCtx;
	CryptoUeccData_t *pd = UeccData(pDev, pKeyCtx);
	if (pd == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Start from a clean state: no valid key until make_key succeeds.
	pd->bKeyValid = false;
	CryptoSecureWipe(pd->PrivKey, sizeof(pd->PrivKey));

	if (uECC_get_rng() == nullptr)
	{
		uECC_set_rng(UeccRngAdapter);
	}

	if (uECC_make_key(pPubKey, pd->PrivKey, uECC_secp256r1()) != 1)
	{
		// make_key may have written partial data into PrivKey; wipe it.
		CryptoSecureWipe(pd->PrivKey, sizeof(pd->PrivKey));
		return CRYPTO_STATUS_FAIL;
	}
	pd->bKeyValid = true;
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS UeccEcdh(CryptoDev_t * const pDev, void *pKeyCtx,
								  const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
								  void *pOpCtx)
{
	(void)pOpCtx;
	CryptoUeccData_t *pd = UeccData(pDev, pKeyCtx);
	if (pd == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Fail closed when no valid private key is present: Ecdh called before
	// KeyGen, or a second Ecdh after the single-use key was consumed and wiped.
	if (!pd->bKeyValid)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Reject a peer public key that is not on the P-256 curve before the DH.
	// Without this the engine is open to the invalid-curve attack (CVE-2018-5383).
	// The private key is single-use, so wipe it on this exit path too.
	if (uECC_valid_public_key(pPeerPubKey, uECC_secp256r1()) != 1)
	{
		CryptoSecureWipe(pd->PrivKey, sizeof(pd->PrivKey));
		pd->bKeyValid = false;
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t secret[32];
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;
	if (uECC_shared_secret(pPeerPubKey, pd->PrivKey, secret, uECC_secp256r1()) != 1)
	{
		st = CRYPTO_STATUS_FAIL;
	}
	else
	{
		memcpy(pDhKey, secret, 32);	// DHKey = X coordinate, big-endian
	}

	// Ephemeral key is single-use: wipe the shared secret and the private key
	// on every exit so no secret material survives the call.
	CryptoSecureWipe(secret, sizeof(secret));
	CryptoSecureWipe(pd->PrivKey, sizeof(pd->PrivKey));
	pd->bKeyValid = false;
	return st;
}

// Self-test: BLE spec P-256 DH known vector (Vol 3 Part H 2.3.5.6.1).
static int UeccSelfTest(CryptoDev_t * const pDev)
{
	(void)pDev;
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
	CryptoSecureWipe(dh, sizeof(dh));	// wipe the test shared secret
	return rc;
}

static CRYPTO_STATUS UeccEcdsaVerify(CryptoDev_t * const pDev, const uint8_t PubKey[64],
									const uint8_t Hash[32], const uint8_t Sig[64],
									void *pCtx)
{
	(void)pDev;
	(void)pCtx;
	// uECC_verify returns 1 for a valid signature. PubKey is X||Y and Sig is
	// r||s, both big-endian, matching the interface byte order.
	int ok = uECC_verify(PubKey, Hash, 32, Sig, uECC_secp256r1());
	return (ok == 1) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

static CRYPTO_STATUS UeccEcdsaSign(CryptoDev_t * const pDev, const uint8_t PrivKey[32],
								  const uint8_t Hash[32], uint8_t Sig[64], void *pCtx)
{
	(void)pDev;
	(void)pCtx;
	if (uECC_get_rng() == nullptr)
	{
		uECC_set_rng(UeccRngAdapter);	// per-signature nonce from RngGet
	}
	int ok = uECC_sign(PrivKey, Hash, 32, Sig, uECC_secp256r1());
	return (ok == 1) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

bool CryptoUeccInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}
	if (pCfg->pMem == nullptr || pCfg->MemSize < sizeof(CryptoUeccData_t))
	{
		return false;	// caller must supply per-instance state
	}
	uint32_t caps = CRYPTO_CAP_ECDH_P256 | CRYPTO_CAP_SHA256 | CRYPTO_CAP_HMAC_SHA256 |
					CRYPTO_CAP_ECDSA_P256_SIGN | CRYPTO_CAP_ECDSA_P256_VERIFY;
	if ((pCfg->ReqCaps & ~caps) != 0)
	{
		return false;	// requested capability not provided by this engine
	}

	memset(pCfg->pMem, 0, sizeof(CryptoUeccData_t));
	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "uecc";
	pDev->Cap            = caps;					// ECDH + ECDSA verify; SHA/HMAC via base
	pDev->Props          = CRYPTO_PROP_PLAIN_KEYCTX | CRYPTO_PROP_SYNC;	// plain zeroable key ctx; synchronous
	pDev->KeyCtxSize     = sizeof(CryptoUeccData_t);
	pDev->EvtCB          = pCfg->EvtCB;
	pDev->Aes128Ecb      = nullptr;					// not provided
	pDev->Cmac           = nullptr;					// no AES, no CMAC
	pDev->Ccm            = nullptr;					// no AES, no CCM
	pDev->Gcm            = nullptr;					// no AES, no GCM
	pDev->Sha256         = nullptr;					// software SHA-256 in the base
	pDev->EcdhP256KeyGen = UeccEcdhKeyGen;
	pDev->EcdhP256       = UeccEcdh;
	pDev->EcdsaP256Verify = UeccEcdsaVerify;
	pDev->EcdsaP256Sign  = UeccEcdsaSign;
	pDev->SelfTest       = UeccSelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) && UeccSelfTest(pDev) != 0)
	{
		return false;
	}
	return true;
}

#else  // uECC not available on this target

bool CryptoUeccInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	(void)pDev; (void)pCfg;
	return false;	// micro-ecc not present in this build
}

#endif // CRYPTO_HAS_UECC || __has_include("uECC.h")
