/**-------------------------------------------------------------------------
@file	crypto_cc310.cpp

@brief	SMP crypto provider: CryptoCell CC310 hardware via the nrf_cc310
		runtime library (nRF52840).

		Implements the CryptoHwInit public symbol (CryptoDev_t hardware engine)
		directly on the CRYS API of the nrf_cc310 runtime library (SaSi_LibInit,
		CRYS_ECPKI_*, CRYS_ECDH_SVDP_DH, CRYS_RND_*). The nRF5 SDK nrf_crypto
		layer is not used; libnrf_cc310 is the only dependency. This is the
		nRF52840 counterpart of crypto_psa_bm.cpp (sdk-nrf-bm PSA over CRACEN on
		nRF54L). The CC310 accelerator is only present on nRF52840; parts
		without it (nRF52832 and smaller) have no ECC hardware, so this file is
		not added to their lib projects and the software uECC engine is used
		instead.

		ECDH P-256 only. The private key stays inside the CRYS key structure in
		App-owned memory; the raw private scalar never crosses the interface.
		AES for SMP comes from the BLE controller (BtCryptoCtlrSdcInit), so this
		engine does not provide it. The key context is a structured CRYS
		object, not plain bytes, so the engine does NOT set
		CRYPTO_CAP_PLAIN_KEYCTX; a Cryptor composes it with pMem NULL, like the
		mbedTLS and PSA engines.

		Byte order: the engine interface is big-endian (crypto.h). CRYS import
		and export forms are big-endian PC||X||Y with the 0x04 uncompressed
		marker byte, and the ECDH shared secret is the big-endian X coordinate,
		so the only conversion is adding or stripping the marker byte.

		Peer public key validation: the peer key is imported with
		CRYS_ECPKI_BuildPublKeyFullCheck, which verifies the point is on the
		P-256 curve before the DH, closing the invalid-curve attack
		(CVE-2018-5383).

		Power: the CryptoCell peripheral is enabled only around library init
		and each operation (NRF_CRYPTOCELL->ENABLE), the same pattern as the
		SDK CC310 usage.

		Requires: libnrf_cc310 linked in the example project. The RNG for key
		generation is the CC310 TRNG-seeded DRBG (CRYS_RND), instantiated once
		at engine init.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nrf.h"

#include "crypto/crypto.h"

// This engine is available on a target when this file is added to the MCU lib
// project. It needs the nrf_cc310 headers; if they do not resolve the build
// fails and reports it. A lib links exactly one file defining CryptoHwInit:
// this one, crypto_psa_bm.cpp, or crypto_hw_none.cpp.
#include "sns_silib.h"
#include "crys_rnd.h"
#include "crys_ecpki_types.h"
#include "crys_ecpki_domain.h"
#include "crys_ecpki_build.h"
#include "crys_ecpki_kg.h"
#include "crys_ecpki_dh.h"

// Per-instance key context: the CRYS private key structure plus a valid flag.
typedef struct {
	CRYS_ECPKI_UserPrivKey_t PrivKey;
	bool                     bKeyValid;	// true only while PrivKey is a live single-use key
} CryptoCc310Data_t;

static_assert(sizeof(CryptoCc310Data_t) <= CRYPTO_MEMSIZE_HW,
			  "CRYPTO_MEMSIZE_HW too small for CryptoCc310Data_t");

// DRBG state and its instantiation scratch. The work buffer is also used for
// reseeding, so both stay resident, the same as the SDK CC310 usage.
static CRYS_RND_State_t    s_RndState;
static CRYS_RND_WorkBuff_t s_RndWorkBuff;
static bool                s_bCcInit;

// Resolve the key context: the Cryptor-supplied context, else the engine own.
static inline CryptoCc310Data_t *Cc310Data(CryptoDev_t * const pDev, void *pKeyCtx)
{
	return (CryptoCc310Data_t *)(pKeyCtx != nullptr ? pKeyCtx : pDev->pDevData);
}

// The CryptoCell peripheral must be enabled while the library runs.
static inline void Cc310On(void)
{
	NRF_CRYPTOCELL->ENABLE = 1;
}

static inline void Cc310Off(void)
{
	NRF_CRYPTOCELL->ENABLE = 0;
}

// One-time library bring-up: runtime init and DRBG instantiation from the
// CC310 TRNG.
static CRYPTO_STATUS EnsureCc310(void)
{
	if (!s_bCcInit)
	{
		Cc310On();

		if (SaSi_LibInit() != SA_SILIB_RET_OK)
		{
			Cc310Off();
			return CRYPTO_STATUS_FAIL;
		}

		if (CRYS_RndInit(&s_RndState, &s_RndWorkBuff) != CRYS_OK)
		{
			SaSi_LibFini();
			Cc310Off();
			return CRYPTO_STATUS_FAIL;
		}

		Cc310Off();
		s_bCcInit = true;
	}
	return CRYPTO_STATUS_OK;
}

// Wipe the instance private key and clear the valid flag. CRYS has no key
// release call; overwriting the structure is the release.
static void Cc310KeyReset(CryptoCc310Data_t *pd)
{
	CryptoSecureWipe(&pd->PrivKey, sizeof(pd->PrivKey));
	pd->bKeyValid = false;
}

static CRYPTO_STATUS Cc310EcdhKeyGen(CryptoDev_t * const pDev, void *pKeyCtx,
									 uint8_t pPubKey[64], void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCc310Data_t *pd = Cc310Data(pDev, pKeyCtx);
	if (pd == nullptr || EnsureCc310() != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Clean start: wipe any prior key so a re-keygen cannot leak key material.
	Cc310KeyReset(pd);

	const CRYS_ECPKI_Domain_t *pDom = CRYS_ECPKI_GetEcDomain(CRYS_ECPKI_DomainID_secp256r1);
	if (pDom == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	CRYS_ECPKI_UserPublKey_t pub;
	CRYS_ECPKI_KG_TempData_t kgTemp;

	Cc310On();
	CRYSError_t r = CRYS_ECPKI_GenKeyPair(&s_RndState,
										  (SaSiRndGenerateVectWorkFunc_t)CRYS_RND_GenerateVector,
										  pDom, &pd->PrivKey, &pub, &kgTemp, nullptr);

	// Export as uncompressed PC||X||Y (65 bytes, big-endian); the interface
	// form is X||Y without the marker byte.
	uint8_t  raw[65];
	uint32_t rawLen = sizeof(raw);
	if (r == CRYS_OK)
	{
		r = CRYS_ECPKI_ExportPublKey(&pub, CRYS_EC_PointUncompressed, raw, &rawLen);
	}
	Cc310Off();

	if (r != CRYS_OK || rawLen != 65 || raw[0] != 0x04)
	{
		Cc310KeyReset(pd);
		return CRYPTO_STATUS_FAIL;
	}

	memcpy(pPubKey, &raw[1], 64);
	pd->bKeyValid = true;
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS Cc310Ecdh(CryptoDev_t * const pDev, void *pKeyCtx,
							   const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
							   void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCc310Data_t *pd = Cc310Data(pDev, pKeyCtx);
	if (pd == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Fail closed when no valid private key is present: Ecdh before KeyGen, or a
	// second Ecdh after the single-use key was consumed.
	if (!pd->bKeyValid)
	{
		return CRYPTO_STATUS_FAIL;
	}

	const CRYS_ECPKI_Domain_t *pDom = CRYS_ECPKI_GetEcDomain(CRYS_ECPKI_DomainID_secp256r1);
	if (pDom == nullptr)
	{
		Cc310KeyReset(pd);
		return CRYPTO_STATUS_FAIL;
	}

	// Import the peer public key with full validation: the point is checked to
	// be on the curve before the DH (invalid-curve attack, CVE-2018-5383).
	uint8_t peerRaw[65];
	peerRaw[0] = 0x04;
	memcpy(&peerRaw[1], pPeerPubKey, 64);

	CRYS_ECPKI_UserPublKey_t    peer;
	CRYS_ECPKI_BUILD_TempData_t buildTemp;

	Cc310On();
	CRYSError_t r = CRYS_ECPKI_BuildPublKeyFullCheck(pDom, peerRaw, sizeof(peerRaw),
													 &peer, &buildTemp);

	uint8_t  secret[32];
	uint32_t secretLen = sizeof(secret);
	if (r == CRYS_OK)
	{
		CRYS_ECDH_TempData_t dhTemp;

		r = CRYS_ECDH_SVDP_DH(&peer, &pd->PrivKey, secret, &secretLen, &dhTemp);
	}
	Cc310Off();

	CRYPTO_STATUS rc = (r == CRYS_OK && secretLen == 32) ?
					   CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	if (rc == CRYPTO_STATUS_OK)
	{
		memcpy(pDhKey, secret, 32);	// DHKey = X coordinate, big-endian
	}

	// Ephemeral key is single-use: wipe the shared secret and the private key
	// and clear the valid flag on every exit.
	CryptoSecureWipe(secret, sizeof(secret));
	Cc310KeyReset(pd);
	return rc;
}

// Known-answer test with the BLE Core spec P-256 vector (Vol 3 Part H
// 2.3.5.6.1): import the spec private key, compute the DH against the spec
// peer public key and compare against the spec DHKey. Exercises key import,
// full point validation and the DH itself.
static int Cc310SelfTest(CryptoDev_t * const pDev)
{
	(void)pDev;
	static const uint8_t privA[32] = {
		0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
		0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd
	};
	static const uint8_t pubB[64] = {
		0x1e,0xa1,0xf0,0xf0,0x1f,0xaf,0x1d,0x96,0x09,0x59,0x22,0x84,0xf1,0x9e,0x4c,0x00,
		0x47,0xb5,0x8a,0xfd,0x86,0x15,0xa6,0x9f,0x55,0x90,0x77,0xb2,0x2f,0xaa,0xa1,0x90,
		0x4c,0x55,0xf3,0x3e,0x42,0x9d,0xad,0x37,0x73,0x56,0x70,0x3a,0x9a,0xb8,0x51,0x60,
		0x47,0x2d,0x11,0x30,0xe2,0x8e,0x36,0x76,0x5f,0x89,0xaf,0xf9,0x15,0xb1,0x21,0x4a
	};
	static const uint8_t dhExpect[32] = {
		0xec,0x02,0x34,0xa3,0x57,0xc8,0xad,0x05,0x34,0x10,0x10,0xa6,0x0a,0x39,0x7d,0x9b,
		0x99,0x79,0x6b,0x13,0xb4,0xf8,0x66,0xf1,0x86,0x8d,0x34,0xf3,0x73,0xbf,0xa6,0x98
	};

	if (EnsureCc310() != CRYPTO_STATUS_OK)
	{
		return -1;
	}

	const CRYS_ECPKI_Domain_t *pDom = CRYS_ECPKI_GetEcDomain(CRYS_ECPKI_DomainID_secp256r1);
	if (pDom == nullptr)
	{
		return -1;
	}

	CRYS_ECPKI_UserPrivKey_t    priv;
	CRYS_ECPKI_UserPublKey_t    pub;
	CRYS_ECPKI_BUILD_TempData_t buildTemp;

	uint8_t pubRaw[65];
	pubRaw[0] = 0x04;
	memcpy(&pubRaw[1], pubB, 64);

	Cc310On();
	CRYSError_t r = CRYS_ECPKI_BuildPrivKey(pDom, privA, sizeof(privA), &priv);
	if (r == CRYS_OK)
	{
		r = CRYS_ECPKI_BuildPublKeyFullCheck(pDom, pubRaw, sizeof(pubRaw),
											 &pub, &buildTemp);
	}

	uint8_t  dh[32];
	uint32_t dhLen = sizeof(dh);
	if (r == CRYS_OK)
	{
		CRYS_ECDH_TempData_t dhTemp;

		r = CRYS_ECDH_SVDP_DH(&pub, &priv, dh, &dhLen, &dhTemp);
	}
	Cc310Off();

	int rc;
	if (r != CRYS_OK || dhLen != 32)
	{
		rc = -1;
	}
	else
	{
		rc = (memcmp(dh, dhExpect, 32) == 0) ? 0 : -2;
	}

	CryptoSecureWipe(dh, sizeof(dh));		// wipe the test shared secret
	CryptoSecureWipe(&priv, sizeof(priv));	// wipe the test private key
	return rc;
}

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}
	if (pCfg->pMem == nullptr || pCfg->MemSize < sizeof(CryptoCc310Data_t))
	{
		return false;	// caller must supply per-instance state
	}
	if (((uintptr_t)pCfg->pMem & (alignof(uint32_t) - 1)) != 0)
	{
		return false;	// arena must be word aligned (see CryptoCfg_t pMem)
	}
	if ((pCfg->ReqCaps & ~(uint32_t)CRYPTO_CAP_ECDH_P256) != 0)
	{
		return false;	// this engine provides P-256 ECDH only
	}
	if (EnsureCc310() != CRYPTO_STATUS_OK)
	{
		return false;	// CC310 runtime not available on this target
	}

	memset(pDev, 0, sizeof(*pDev));
	memset(pCfg->pMem, 0, sizeof(CryptoCc310Data_t));
	CryptoCc310Data_t *pd = (CryptoCc310Data_t *)pCfg->pMem;
	pd->bKeyValid = false;

	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "cc310-hw";
	pDev->Cap            = CRYPTO_CAP_ECDH_P256;	// ECDH only; structured key ctx, no PLAIN_KEYCTX
	pDev->KeyCtxSize     = sizeof(CryptoCc310Data_t);
	pDev->Props          = CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC;
	pDev->EvtCB          = pCfg->EvtCB;			// synchronous engine
	pDev->Aes128Ecb      = nullptr;				// AES via the BLE controller on this path
	pDev->EcdhP256KeyGen = Cc310EcdhKeyGen;
	pDev->EcdhP256       = Cc310Ecdh;
	pDev->SelfTest       = Cc310SelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) && Cc310SelfTest(pDev) != 0)
	{
		return false;
	}
	return true;
}
