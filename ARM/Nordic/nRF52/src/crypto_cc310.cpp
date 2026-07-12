/**-------------------------------------------------------------------------
@file	crypto_cc310.cpp

@brief	CryptoCell CC310 P-256 provider for the nRF5 SDK runtime.

		This backend uses the nRF5 SDK nrf_cc310 CRYS ABI. The nrfxlib SDC
		build uses crypto_cc310_sdc.cpp instead. Keeping the two translation
		units separate prevents the SDK and nrfxlib CryptoCell ABIs from being
		mixed in one library configuration.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"

#if !defined(NRFXLIB_SDC)

#include "nrf.h"
#include "sns_silib.h"
#include "crys_rnd.h"
#include "crys_ecpki_types.h"
#include "crys_ecpki_domain.h"
#include "crys_ecpki_build.h"
#include "crys_ecpki_kg.h"
#include "crys_ecpki_dh.h"

typedef struct {
	CRYS_ECPKI_UserPrivKey_t PrivKey;
	bool                     bKeyValid;
} CryptoCc310Data_t;

static_assert(sizeof(CryptoCc310Data_t) <= CRYPTO_MEMSIZE_HW,
			  "CRYPTO_MEMSIZE_HW too small for CryptoCc310Data_t");

static CRYS_RND_State_t    s_RndState;
static CRYS_RND_WorkBuff_t s_RndWorkBuff;
static bool                s_bCcInit;

static inline CryptoCc310Data_t *Cc310Data(CryptoDev_t * const pDev,
										  void *pKeyCtx)
{
	return (CryptoCc310Data_t *)(pKeyCtx != nullptr ? pKeyCtx : pDev->pDevData);
}

static inline void Cc310On(void)
{
	NRF_CRYPTOCELL->ENABLE = 1;
}

static inline void Cc310Off(void)
{
	NRF_CRYPTOCELL->ENABLE = 0;
}

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

static void Cc310KeyReset(CryptoCc310Data_t *pData)
{
	CryptoSecureWipe(&pData->PrivKey, sizeof(pData->PrivKey));
	pData->bKeyValid = false;
}

static CRYPTO_STATUS Cc310EcdhKeyGen(CryptoDev_t * const pDev,
									 void *pKeyCtx,
									 uint8_t pPubKey[64],
									 void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCc310Data_t *pData = Cc310Data(pDev, pKeyCtx);
	if (pData == nullptr || EnsureCc310() != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	Cc310KeyReset(pData);
	const CRYS_ECPKI_Domain_t *pDomain =
		CRYS_ECPKI_GetEcDomain(CRYS_ECPKI_DomainID_secp256r1);
	if (pDomain == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	CRYS_ECPKI_UserPublKey_t pub;
	CRYS_ECPKI_KG_TempData_t temp;
	uint8_t raw[65];
	uint32_t rawLen = sizeof(raw);

	Cc310On();
	CRYSError_t result = CRYS_ECPKI_GenKeyPair(
		&s_RndState,
		(SaSiRndGenerateVectWorkFunc_t)CRYS_RND_GenerateVector,
		pDomain, &pData->PrivKey, &pub, &temp, nullptr);
	if (result == CRYS_OK)
	{
		result = CRYS_ECPKI_ExportPublKey(&pub, CRYS_EC_PointUncompressed,
										 raw, &rawLen);
	}
	Cc310Off();

	if (result != CRYS_OK || rawLen != sizeof(raw) || raw[0] != 0x04)
	{
		CryptoSecureWipe(raw, sizeof(raw));
		Cc310KeyReset(pData);
		return CRYPTO_STATUS_FAIL;
	}

	memcpy(pPubKey, &raw[1], 64);
	CryptoSecureWipe(raw, sizeof(raw));
	pData->bKeyValid = true;
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS Cc310Ecdh(CryptoDev_t * const pDev,
							   void *pKeyCtx,
							   const uint8_t pPeerPubKey[64],
							   uint8_t pDhKey[32],
							   void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCc310Data_t *pData = Cc310Data(pDev, pKeyCtx);
	if (pData == nullptr || !pData->bKeyValid)
	{
		return CRYPTO_STATUS_FAIL;
	}

	const CRYS_ECPKI_Domain_t *pDomain =
		CRYS_ECPKI_GetEcDomain(CRYS_ECPKI_DomainID_secp256r1);
	if (pDomain == nullptr)
	{
		Cc310KeyReset(pData);
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t peerRaw[65];
	peerRaw[0] = 0x04;
	memcpy(&peerRaw[1], pPeerPubKey, 64);

	CRYS_ECPKI_UserPublKey_t peer;
	CRYS_ECPKI_BUILD_TempData_t buildTemp;
	uint8_t secret[32];
	uint32_t secretLen = sizeof(secret);

	Cc310On();
	CRYSError_t result = CRYS_ECPKI_BuildPublKeyFullCheck(
		pDomain, peerRaw, sizeof(peerRaw), &peer, &buildTemp);
	if (result == CRYS_OK)
	{
		CRYS_ECDH_TempData_t dhTemp;
		result = CRYS_ECDH_SVDP_DH(&peer, &pData->PrivKey,
									 secret, &secretLen, &dhTemp);
	}
	Cc310Off();

	CRYPTO_STATUS status =
		(result == CRYS_OK && secretLen == sizeof(secret)) ?
		CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	if (status == CRYPTO_STATUS_OK)
	{
		memcpy(pDhKey, secret, sizeof(secret));
	}

	CryptoSecureWipe(peerRaw, sizeof(peerRaw));
	CryptoSecureWipe(secret, sizeof(secret));
	Cc310KeyReset(pData);
	return status;
}

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
	static const uint8_t expected[32] = {
		0xec,0x02,0x34,0xa3,0x57,0xc8,0xad,0x05,0x34,0x10,0x10,0xa6,0x0a,0x39,0x7d,0x9b,
		0x99,0x79,0x6b,0x13,0xb4,0xf8,0x66,0xf1,0x86,0x8d,0x34,0xf3,0x73,0xbf,0xa6,0x98
	};

	if (EnsureCc310() != CRYPTO_STATUS_OK)
	{
		return -1;
	}
	const CRYS_ECPKI_Domain_t *pDomain =
		CRYS_ECPKI_GetEcDomain(CRYS_ECPKI_DomainID_secp256r1);
	if (pDomain == nullptr)
	{
		return -1;
	}

	CRYS_ECPKI_UserPrivKey_t priv;
	CRYS_ECPKI_UserPublKey_t pub;
	CRYS_ECPKI_BUILD_TempData_t buildTemp;
	uint8_t rawPub[65];
	rawPub[0] = 0x04;
	memcpy(&rawPub[1], pubB, sizeof(pubB));
	uint8_t secret[32];
	uint32_t secretLen = sizeof(secret);

	Cc310On();
	CRYSError_t result = CRYS_ECPKI_BuildPrivKey(
		pDomain, privA, sizeof(privA), &priv);
	if (result == CRYS_OK)
	{
		result = CRYS_ECPKI_BuildPublKeyFullCheck(
			pDomain, rawPub, sizeof(rawPub), &pub, &buildTemp);
	}
	if (result == CRYS_OK)
	{
		CRYS_ECDH_TempData_t dhTemp;
		result = CRYS_ECDH_SVDP_DH(&pub, &priv,
									 secret, &secretLen, &dhTemp);
	}
	Cc310Off();

	int status = -1;
	if (result == CRYS_OK && secretLen == sizeof(secret))
	{
		status = (memcmp(secret, expected, sizeof(secret)) == 0) ? 0 : -2;
	}
	CryptoSecureWipe(rawPub, sizeof(rawPub));
	CryptoSecureWipe(secret, sizeof(secret));
	CryptoSecureWipe(&priv, sizeof(priv));
	return status;
}

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pMem == nullptr ||
		pCfg->MemSize < sizeof(CryptoCc310Data_t))
	{
		return false;
	}
	if ((pCfg->ReqCaps & ~(uint32_t)CRYPTO_CAP_ECDH_P256) != 0 ||
		EnsureCc310() != CRYPTO_STATUS_OK)
	{
		return false;
	}

	memset(pDev, 0, sizeof(*pDev));
	memset(pCfg->pMem, 0, sizeof(CryptoCc310Data_t));
	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "cc310-hw";
	pDev->Cap            = CRYPTO_CAP_ECDH_P256;
	pDev->Props          = CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC;
	pDev->KeyCtxSize     = sizeof(CryptoCc310Data_t);
	pDev->EvtCB          = pCfg->EvtCB;
	pDev->Aes128Ecb      = nullptr;
	pDev->EcdhP256KeyGen = Cc310EcdhKeyGen;
	pDev->EcdhP256       = Cc310Ecdh;
	pDev->SelfTest       = Cc310SelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) && Cc310SelfTest(pDev) != 0)
	{
		return false;
	}
	return true;
}

#endif // !NRFXLIB_SDC
