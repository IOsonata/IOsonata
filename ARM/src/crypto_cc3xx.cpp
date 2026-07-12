/**-------------------------------------------------------------------------
@file	crypto_cc3xx.cpp

@brief	CryptoDev_t hardware engine on the Arm CC3xx low level driver.

		Generic P-256 ECDH provider for Arm targets containing a compatible
		CryptoCell CC3xx engine. The implementation is built directly on the
		open Arm register level driver from tf-psa-crypto-drivers, with no
		vendor binary or PSA core dependency.

		The selected target supplies cc3xx_port.h. That header implements only
		the target-specific clock, power, reset, wrapper, and readiness steps.
		CC3xx register initialization, serialization, key handling, and crypto
		operations remain in this generic ARM source.

		The driver reads and writes PKA registers with a per-word endianness
		swap, so its uint32_t buffers hold big-endian byte strings. The
		CryptoDev_t interface uses SEC1 big-endian X||Y public keys and a
		big-endian X-coordinate DHKey. Interface byte buffers are copied
		through aligned local buffers.

		The generic cc3xx_rng.c adapter routes the CC3xx random API to IOsonata
		RngGet. The CC3xx noise source, entropy conditioner, and DRBG modules
		are not built.

		Peer public keys are checked to be on the P-256 curve with
		cc3xx_lowlevel_ec_allocate_point_from_data before ECDH, closing the
		invalid-curve attack described by CVE-2018-5383.

		The driver has global engine state, so one operation runs at a time. A
		PRIMASK-protected try-acquire guard returns CRYPTO_STATUS_FAIL to a
		concurrent caller instead of corrupting the PKA state.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "crypto/crypto.h"
#include "coredev/interrupt.h"
#include "cc3xx_port.h"

// The CC3xx headers manage their own C linkage.
#include "cc3xx_ecdh.h"
#include "cc3xx_ecdsa.h"
#include "cc3xx_ec.h"
#include "cc3xx_ec_curve_data.h"
#include "cc3xx_dev.h"
#include "cc3xx_ahbm.h"
#include "cc3xx_engine_state.h"

#define CC3XX_P256_BYTES		32
#define CC3XX_P256_WORDS		(CC3XX_P256_BYTES / sizeof(uint32_t))

typedef struct {
	uint32_t PrivKey[CC3XX_P256_WORDS];
	bool     bKeyValid;
} CryptoCc3xxData_t;

static_assert(sizeof(CryptoCc3xxData_t) <= CRYPTO_MEMSIZE_HW,
			  "CRYPTO_MEMSIZE_HW too small for CryptoCc3xxData_t");

static bool s_bCcInit;
static volatile bool s_bCc3xxBusy;

static bool Cc3xxAcquire(void)
{
	uint32_t intState = DisableInterrupt();

	bool acquired = (s_bCc3xxBusy == false);
	if (acquired)
	{
		s_bCc3xxBusy = true;
	}

	EnableInterrupt(intState);
	return acquired;
}

static inline void Cc3xxRelease(void)
{
	s_bCc3xxBusy = false;
}

static CryptoCc3xxData_t *Cc3xxData(CryptoDev_t * const pDev, void *pKeyCtx)
{
	void *p = pKeyCtx != nullptr ? pKeyCtx :
			  (pDev != nullptr ? pDev->pDevData : nullptr);

	if (p == nullptr || ((uintptr_t)p & (alignof(CryptoCc3xxData_t) - 1)) != 0)
	{
		return nullptr;
	}

	return (CryptoCc3xxData_t *)p;
}

static CRYPTO_STATUS EnsureCc3xx(void)
{
	if (s_bCcInit)
	{
		return CRYPTO_STATUS_OK;
	}

	if (Cc3xxAcquire() == false)
	{
		return CRYPTO_STATUS_FAIL;
	}

	if (s_bCcInit)
	{
		Cc3xxRelease();
		return CRYPTO_STATUS_OK;
	}

	if (Cc3xxPortEnable() == false)
	{
		Cc3xxRelease();
		return CRYPTO_STATUS_FAIL;
	}

	// Mirror the upstream feature check for CC3XX_CONFIG_RNG_ENABLE. The
	// hardware RNG is not used by this provider; random bytes come from RngGet.
	if ((P_CC3XX->host_rgf.host_boot & (1U << 11)) == 0U)
	{
		Cc3xxPortDisable();
		Cc3xxRelease();
		return CRYPTO_STATUS_FAIL;
	}

	P_CC3XX->host_rgf.host_rgf_endian = 0x0U;

	cc3xx_engine_in_use = CC3XX_ENGINE_NONE;
	P_CC3XX->cc_ctl.crypto_ctl = CC3XX_ENGINE_NONE;

	P_CC3XX->misc.dma_clk_enable = 0x1U;
	P_CC3XX->ahb.ahbm_hprot = CC3XX_AHBM_HPROT_DATA |
							  CC3XX_AHBM_HPROT_PRIVILEGED |
							  CC3XX_AHBM_HPROT_CACHEABLE;
	P_CC3XX->ahb.ahbm_singles = CC3XX_AHBM_BURST_INCR4_TRANSACTIONS;
	P_CC3XX->ahb.ahbm_hnonsec = CC3XX_AHBM_SECURE_TRANSACTIONS;
	P_CC3XX->misc.dma_clk_enable = 0x0U;

	s_bCcInit = true;
	Cc3xxRelease();
	return CRYPTO_STATUS_OK;
}

static void Cc3xxKeyReset(CryptoCc3xxData_t *pd)
{
	CryptoSecureWipe(pd->PrivKey, sizeof(pd->PrivKey));
	pd->bKeyValid = false;
}

static CRYPTO_STATUS Cc3xxEcdhKeyGen(CryptoDev_t * const pDev, void *pKeyCtx,
									 uint8_t pPubKey[64], void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCc3xxData_t *pd = Cc3xxData(pDev, pKeyCtx);
	if (pd == nullptr || pPubKey == nullptr || EnsureCc3xx() != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	if (Cc3xxAcquire() == false)
	{
		return CRYPTO_STATUS_FAIL;
	}

	Cc3xxKeyReset(pd);

	uint32_t pubX[CC3XX_P256_WORDS];
	uint32_t pubY[CC3XX_P256_WORDS];
	size_t privSize = 0U;
	size_t pubXSize = 0U;
	size_t pubYSize = 0U;

	cc3xx_err_t r = cc3xx_lowlevel_ecdsa_genkey(CC3XX_EC_CURVE_SECP_256_R1,
											pd->PrivKey, sizeof(pd->PrivKey),
											&privSize);
	if (r == CC3XX_ERR_SUCCESS)
	{
		r = cc3xx_lowlevel_ecdsa_getpub(CC3XX_EC_CURVE_SECP_256_R1,
									pd->PrivKey, sizeof(pd->PrivKey),
									pubX, sizeof(pubX), &pubXSize,
									pubY, sizeof(pubY), &pubYSize);
	}

	if (r != CC3XX_ERR_SUCCESS ||
		privSize != CC3XX_P256_BYTES ||
		pubXSize != CC3XX_P256_BYTES ||
		pubYSize != CC3XX_P256_BYTES)
	{
		Cc3xxKeyReset(pd);
		Cc3xxRelease();
		return CRYPTO_STATUS_FAIL;
	}

	memcpy(&pPubKey[0], pubX, CC3XX_P256_BYTES);
	memcpy(&pPubKey[32], pubY, CC3XX_P256_BYTES);

	pd->bKeyValid = true;
	Cc3xxRelease();
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS Cc3xxEcdhRaw(const uint32_t privKey[CC3XX_P256_WORDS],
								  const uint8_t pPeerPubKey[64],
								  uint8_t pDhKey[32])
{
	if (privKey == nullptr || pPeerPubKey == nullptr || pDhKey == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	uint32_t peerX[CC3XX_P256_WORDS];
	uint32_t peerY[CC3XX_P256_WORDS];
	memcpy(peerX, &pPeerPubKey[0], CC3XX_P256_BYTES);
	memcpy(peerY, &pPeerPubKey[32], CC3XX_P256_BYTES);

	cc3xx_ec_curve_t curve;
	cc3xx_ec_point_affine peerPt;
	cc3xx_err_t r = cc3xx_lowlevel_ec_init(CC3XX_EC_CURVE_SECP_256_R1, &curve);
	if (r == CC3XX_ERR_SUCCESS)
	{
		r = cc3xx_lowlevel_ec_allocate_point_from_data(&curve,
											   peerX, CC3XX_P256_BYTES,
											   peerY, CC3XX_P256_BYTES,
											   &peerPt);
		cc3xx_lowlevel_ec_uninit();
	}
	if (r != CC3XX_ERR_SUCCESS)
	{
		return CRYPTO_STATUS_FAIL;
	}

	uint32_t secret[CC3XX_P256_WORDS];
	size_t secretSize = 0U;

	r = cc3xx_lowlevel_ecdh(CC3XX_EC_CURVE_SECP_256_R1,
								privKey, CC3XX_P256_BYTES,
								peerX, CC3XX_P256_BYTES,
								peerY, CC3XX_P256_BYTES,
								secret, sizeof(secret), &secretSize);

	CRYPTO_STATUS rc = (r == CC3XX_ERR_SUCCESS && secretSize == CC3XX_P256_BYTES) ?
					   CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;

	if (rc == CRYPTO_STATUS_OK)
	{
		memcpy(pDhKey, secret, CC3XX_P256_BYTES);
	}

	CryptoSecureWipe(secret, sizeof(secret));
	return rc;
}

static CRYPTO_STATUS Cc3xxEcdh(CryptoDev_t * const pDev, void *pKeyCtx,
							   const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
							   void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCc3xxData_t *pd = Cc3xxData(pDev, pKeyCtx);
	if (pd == nullptr || pd->bKeyValid == false)
	{
		return CRYPTO_STATUS_FAIL;
	}

	if (Cc3xxAcquire() == false)
	{
		return CRYPTO_STATUS_FAIL;
	}

	CRYPTO_STATUS rc = Cc3xxEcdhRaw(pd->PrivKey, pPeerPubKey, pDhKey);
	Cc3xxKeyReset(pd);
	Cc3xxRelease();
	return rc;
}

static int Cc3xxSelfTest(CryptoDev_t * const pDev)
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

	if (EnsureCc3xx() != CRYPTO_STATUS_OK)
	{
		return -1;
	}

	uint32_t priv[CC3XX_P256_WORDS];
	uint8_t dh[32];
	memcpy(priv, privA, sizeof(privA));

	if (Cc3xxAcquire() == false)
	{
		CryptoSecureWipe(priv, sizeof(priv));
		return -1;
	}

	CRYPTO_STATUS rc = Cc3xxEcdhRaw(priv, pubB, dh);
	Cc3xxRelease();

	int ret;
	if (rc != CRYPTO_STATUS_OK)
	{
		ret = -1;
	}
	else
	{
		ret = (memcmp(dh, dhExpect, sizeof(dhExpect)) == 0) ? 0 : -2;
	}

	CryptoSecureWipe(dh, sizeof(dh));
	CryptoSecureWipe(priv, sizeof(priv));
	return ret;
}

extern "C" bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}
	if (pCfg->pMem == nullptr || pCfg->MemSize < sizeof(CryptoCc3xxData_t))
	{
		return false;
	}
	if (((uintptr_t)pCfg->pMem & (alignof(uint32_t) - 1)) != 0U)
	{
		return false;
	}
	if ((pCfg->ReqCaps & ~(uint32_t)CRYPTO_CAP_ECDH_P256) != 0U)
	{
		return false;
	}
	if (EnsureCc3xx() != CRYPTO_STATUS_OK)
	{
		return false;
	}

	memset(pDev, 0, sizeof(*pDev));
	memset(pCfg->pMem, 0, sizeof(CryptoCc3xxData_t));
	CryptoCc3xxData_t *pd = (CryptoCc3xxData_t *)pCfg->pMem;
	pd->bKeyValid = false;

	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "cc3xx-hw";
	pDev->Cap            = CRYPTO_CAP_ECDH_P256;
	pDev->KeyCtxSize     = sizeof(CryptoCc3xxData_t);
	pDev->Props          = CRYPTO_PROP_PLAIN_KEYCTX |
						   CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC;
	pDev->EvtCB          = pCfg->EvtCB;
	pDev->Aes128Ecb      = nullptr;
	pDev->EcdhP256KeyGen = Cc3xxEcdhKeyGen;
	pDev->EcdhP256       = Cc3xxEcdh;
	pDev->SelfTest       = Cc3xxSelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) != 0U && Cc3xxSelfTest(pDev) != 0)
	{
		return false;
	}
	return true;
}
