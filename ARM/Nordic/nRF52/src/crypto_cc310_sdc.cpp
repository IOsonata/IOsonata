/**-------------------------------------------------------------------------
@file	crypto_cc310_sdc.cpp

@brief	CryptoCell CC310 P-256 provider for the nrfxlib SDC build.

		Implements CryptoHwInit through the nrfxlib CC3xx platform and PSA
		driver entry points. This is separate from crypto_cc310.cpp, which uses
		the nRF5 SDK nrf_cc310 CRYS runtime. The nrfxlib platform owns CryptoCell
		power, entropy and DRBG initialization; this file never writes
		NRF_CRYPTOCELL->ENABLE directly.

		Only ECDH P-256 is exposed. The generated private scalar is retained in
		App-owned CryptoDev_t state and wiped after one ECDH attempt. Public keys
		use SEC1 uncompressed form inside the driver and the IOsonata interface
		uses X||Y, both big-endian.

		Requires the nrfxlib nrf_cc310 platform, core and psa_crypto libraries.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"

#if defined(NRFXLIB_SDC)

#include "psa/crypto.h"
#include "nrf_cc3xx_platform.h"
#include "cc3xx_psa_key_generation.h"
#include "cc3xx_psa_key_agreement.h"

#define CC310_P256_PRIV_KEY_SIZE	32U
#define CC310_P256_PUB_KEY_SIZE		64U
#define CC310_SEC1_PUB_KEY_SIZE		65U

typedef struct {
	uint8_t PrivKey[CC310_P256_PRIV_KEY_SIZE];
	size_t  PrivKeyLen;
	bool    bKeyValid;
} CryptoCc310SdcData_t;

static_assert(sizeof(CryptoCc310SdcData_t) <= CRYPTO_MEMSIZE_HW,
			  "CRYPTO_MEMSIZE_HW too small for CryptoCc310SdcData_t");

static bool s_bCc310SdcInit;

static inline CryptoCc310SdcData_t *Cc310SdcData(CryptoDev_t * const pDev,
											 void *pKeyCtx)
{
	return (CryptoCc310SdcData_t *)(pKeyCtx != nullptr ? pKeyCtx : pDev->pDevData);
}

static void Cc310SdcKeyReset(CryptoCc310SdcData_t *pData)
{
	if (pData != nullptr)
	{
		CryptoSecureWipe(pData->PrivKey, sizeof(pData->PrivKey));
		pData->PrivKeyLen = 0;
		pData->bKeyValid = false;
	}
}

static CRYPTO_STATUS EnsureCc310Sdc(void)
{
	if (!s_bCc310SdcInit)
	{
		if (nrf_cc3xx_platform_init() != 0)
		{
			return CRYPTO_STATUS_FAIL;
		}
		s_bCc310SdcInit = true;
	}
	return CRYPTO_STATUS_OK;
}

static psa_key_attributes_t Cc310SdcP256Attributes(void)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;

	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
	psa_set_key_type(&attr,
					 PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
	psa_set_key_bits(&attr, 256);

	return attr;
}

static CRYPTO_STATUS Cc310SdcEcdhKeyGen(CryptoDev_t * const pDev,
										void *pKeyCtx,
										uint8_t pPubKey[CC310_P256_PUB_KEY_SIZE],
										void *pOpCtx)
{
	(void)pOpCtx;

	CryptoCc310SdcData_t *pData = Cc310SdcData(pDev, pKeyCtx);
	if (pData == nullptr || EnsureCc310Sdc() != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	Cc310SdcKeyReset(pData);

	psa_key_attributes_t attr = Cc310SdcP256Attributes();
	size_t privLen = 0;
	psa_status_t status = cc3xx_generate_key(&attr,
										  pData->PrivKey,
										  sizeof(pData->PrivKey),
										  &privLen);
	if (status != PSA_SUCCESS || privLen != CC310_P256_PRIV_KEY_SIZE)
	{
		Cc310SdcKeyReset(pData);
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t rawPub[CC310_SEC1_PUB_KEY_SIZE];
	size_t rawPubLen = 0;
	status = cc3xx_export_public_key(&attr,
									 pData->PrivKey, privLen,
									 rawPub, sizeof(rawPub), &rawPubLen);
	if (status != PSA_SUCCESS || rawPubLen != sizeof(rawPub) || rawPub[0] != 0x04)
	{
		CryptoSecureWipe(rawPub, sizeof(rawPub));
		Cc310SdcKeyReset(pData);
		return CRYPTO_STATUS_FAIL;
	}

	memcpy(pPubKey, &rawPub[1], CC310_P256_PUB_KEY_SIZE);
	CryptoSecureWipe(rawPub, sizeof(rawPub));
	pData->PrivKeyLen = privLen;
	pData->bKeyValid = true;

	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS Cc310SdcEcdh(CryptoDev_t * const pDev,
								  void *pKeyCtx,
								  const uint8_t pPeerPubKey[CC310_P256_PUB_KEY_SIZE],
								  uint8_t pDhKey[32],
								  void *pOpCtx)
{
	(void)pOpCtx;

	CryptoCc310SdcData_t *pData = Cc310SdcData(pDev, pKeyCtx);
	if (pData == nullptr || !pData->bKeyValid ||
		pData->PrivKeyLen != CC310_P256_PRIV_KEY_SIZE)
	{
		return CRYPTO_STATUS_FAIL;
	}

	psa_key_attributes_t attr = Cc310SdcP256Attributes();
	uint8_t peer[CC310_SEC1_PUB_KEY_SIZE];
	peer[0] = 0x04;
	memcpy(&peer[1], pPeerPubKey, CC310_P256_PUB_KEY_SIZE);

	uint8_t secret[32];
	size_t secretLen = 0;
	psa_status_t status = cc3xx_key_agreement(&attr,
										   pData->PrivKey, pData->PrivKeyLen,
										   peer, sizeof(peer),
										   secret, sizeof(secret), &secretLen,
										   PSA_ALG_ECDH);

	CRYPTO_STATUS result =
		(status == PSA_SUCCESS && secretLen == sizeof(secret)) ?
		CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	if (result == CRYPTO_STATUS_OK)
	{
		memcpy(pDhKey, secret, sizeof(secret));
	}

	CryptoSecureWipe(peer, sizeof(peer));
	CryptoSecureWipe(secret, sizeof(secret));
	Cc310SdcKeyReset(pData);

	return result;
}

static int Cc310SdcSelfTest(CryptoDev_t * const pDev)
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
	static const uint8_t dhExpected[32] = {
		0xec,0x02,0x34,0xa3,0x57,0xc8,0xad,0x05,0x34,0x10,0x10,0xa6,0x0a,0x39,0x7d,0x9b,
		0x99,0x79,0x6b,0x13,0xb4,0xf8,0x66,0xf1,0x86,0x8d,0x34,0xf3,0x73,0xbf,0xa6,0x98
	};

	if (EnsureCc310Sdc() != CRYPTO_STATUS_OK)
	{
		return -1;
	}

	psa_key_attributes_t attr = Cc310SdcP256Attributes();
	uint8_t peer[CC310_SEC1_PUB_KEY_SIZE];
	peer[0] = 0x04;
	memcpy(&peer[1], pubB, sizeof(pubB));

	uint8_t secret[32];
	size_t secretLen = 0;
	psa_status_t status = cc3xx_key_agreement(&attr,
										   privA, sizeof(privA),
										   peer, sizeof(peer),
										   secret, sizeof(secret), &secretLen,
										   PSA_ALG_ECDH);

	int result = -1;
	if (status == PSA_SUCCESS && secretLen == sizeof(secret))
	{
		result = (memcmp(secret, dhExpected, sizeof(secret)) == 0) ? 0 : -2;
	}

	CryptoSecureWipe(peer, sizeof(peer));
	CryptoSecureWipe(secret, sizeof(secret));
	return result;
}

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pMem == nullptr ||
		pCfg->MemSize < sizeof(CryptoCc310SdcData_t))
	{
		return false;
	}
	if ((pCfg->ReqCaps & ~(uint32_t)CRYPTO_CAP_ECDH_P256) != 0)
	{
		return false;
	}
	if (EnsureCc310Sdc() != CRYPTO_STATUS_OK)
	{
		return false;
	}

	memset(pDev, 0, sizeof(*pDev));
	memset(pCfg->pMem, 0, sizeof(CryptoCc310SdcData_t));

	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "cc310-sdc";
	pDev->Cap            = CRYPTO_CAP_ECDH_P256;
	pDev->Props          = CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC;
	pDev->KeyCtxSize     = sizeof(CryptoCc310SdcData_t);
	pDev->EvtCB          = pCfg->EvtCB;
	pDev->Aes128Ecb      = nullptr;
	pDev->EcdhP256KeyGen = Cc310SdcEcdhKeyGen;
	pDev->EcdhP256       = Cc310SdcEcdh;
	pDev->SelfTest       = Cc310SdcSelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) && Cc310SdcSelfTest(pDev) != 0)
	{
		return false;
	}

	return true;
}

#endif // NRFXLIB_SDC
