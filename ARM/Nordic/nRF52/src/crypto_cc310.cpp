/**-------------------------------------------------------------------------
@file	crypto_cc310.cpp

@brief	SMP crypto provider: architecture hardware via the nRF5 SDK nrf_crypto
		library over the CryptoCell CC310 accelerator (nRF52840).

		Implements the CryptoHwInit public symbol (CryptoDev_t hardware engine)
		on the nRF5 SDK, whose crypto surface is nrf_crypto, not PSA. This is
		the nRF52840 counterpart of crypto_psa.cpp (sdk-nrf-bm PSA over CRACEN on
		nRF54L). The CC310 accelerator is only present on nRF52840;
		parts without it (nRF52832 and smaller) have no ECC hardware, so this
		file stays inert there and the software uECC engine is used instead.

		ECDH P-256 only. The private key stays inside the nrf_crypto key object
		in App-owned memory; the raw private scalar never crosses the interface.
		AES for SMP comes from the BLE controller (BtCryptoCtlrSdcInit), so this
		engine does not provide it. The key context is a structured nrf_crypto
		object, not plain bytes, so the engine does NOT set CRYPTO_CAP_PLAIN_KEYCTX;
		a Cryptor composes it with pMem NULL, like the mbedTLS and PSA engines.

		Byte order: the SMP toolbox is big-endian. nrf_crypto raw ECC keys are
		X||Y big-endian and the raw ECDH result is the big-endian X coordinate,
		so the buffers map straight through with no marker byte.

		Requires: nrf_crypto with the CC310 driver enabled in sdk_config.h
		(NRF_CRYPTO_ENABLED, NRF_CRYPTO_BACKEND_CC310_ENABLED and its
		secp256r1 option). Random bytes for key generation come from the CC310
		DRBG inside nrf_crypto.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"

// This engine is available on a target when this file is added to the MCU lib
// project. It needs the nRF5 SDK nrf_crypto headers; if they do not resolve the
// build fails and reports it. A lib links exactly one file defining
// CryptoHwInit: this one, crypto_psa.cpp, or crypto_hw_none.cpp.
//
// This engine and crypto_psa.cpp both define CryptoHwInit, so it stays inert
// whenever PSA is also available: PSA takes precedence (it dispatches to the
// same CC3xx/CRACEN hardware), and the two files never both emit CryptoHwInit
// in one build. So both may be added to any Nordic lib without a collision.
#include "nrf_crypto.h"

// Per-instance key context: the nrf_crypto private key object plus a valid flag.
typedef struct {
	nrf_crypto_ecc_private_key_t PrivKey;
	bool                         bKeyValid;	// true only while PrivKey is a live single-use key
} CryptoCc310Data_t;

static_assert(sizeof(CryptoCc310Data_t) <= CRYPTO_MEMSIZE_HW,
			  "CRYPTO_MEMSIZE_HW too small for CryptoCc310Data_t");

// Resolve the key context: the Cryptor-supplied context, else the engine own.
static inline CryptoCc310Data_t *Cc310Data(CryptoDev_t * const pDev, void *pKeyCtx)
{
	return (CryptoCc310Data_t *)(pKeyCtx != nullptr ? pKeyCtx : pDev->pDevData);
}

// One-time nrf_crypto bring-up. Already-initialized is treated as success, so
// this coexists with other SDK code that also calls nrf_crypto_init.
static CRYPTO_STATUS EnsureCc310(void)
{
	static bool s_Init = false;
	if (!s_Init)
	{
		ret_code_t r = nrf_crypto_init();
		if (r != NRF_SUCCESS && r != NRF_ERROR_MODULE_ALREADY_INITIALIZED)
		{
			return CRYPTO_STATUS_FAIL;
		}
		s_Init = true;
	}
	return CRYPTO_STATUS_OK;
}

// Free the instance private key and clear the valid flag.
static void Cc310KeyReset(CryptoCc310Data_t *pd)
{
	if (pd->bKeyValid)
	{
		(void)nrf_crypto_ecc_private_key_free(&pd->PrivKey);
	}
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

	// Clean start: free any prior key so a re-keygen cannot leak the key object.
	Cc310KeyReset(pd);

	nrf_crypto_ecc_key_pair_generate_context_t genCtx;
	nrf_crypto_ecc_public_key_t                pub;
	if (nrf_crypto_ecc_key_pair_generate(&genCtx,
										 &g_nrf_crypto_ecc_secp256r1_curve_info,
										 &pd->PrivKey, &pub) != NRF_SUCCESS)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Export the public key as raw X||Y (64 bytes, big-endian). The public key
	// object is only needed to read the raw form, so release it here.
	uint8_t raw[64];
	size_t  rawLen = sizeof(raw);
	ret_code_t r = nrf_crypto_ecc_public_key_to_raw(&pub, raw, &rawLen);
	(void)nrf_crypto_ecc_public_key_free(&pub);
	if (r != NRF_SUCCESS || rawLen != 64)
	{
		(void)nrf_crypto_ecc_private_key_free(&pd->PrivKey);
		return CRYPTO_STATUS_FAIL;
	}
	memcpy(pPubKey, raw, 64);
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

	// Import the peer public key from raw X||Y. nrf_crypto validates the point
	// is on the curve, closing the invalid-curve attack (CVE-2018-5383).
	nrf_crypto_ecc_public_key_t peer;
	if (nrf_crypto_ecc_public_key_from_raw(&g_nrf_crypto_ecc_secp256r1_curve_info,
										   &peer, pPeerPubKey, 64) != NRF_SUCCESS)
	{
		(void)nrf_crypto_ecc_private_key_free(&pd->PrivKey);
		pd->bKeyValid = false;
		return CRYPTO_STATUS_FAIL;
	}

	nrf_crypto_ecdh_context_t ecdhCtx;
	uint8_t secret[32];
	size_t  secretLen = sizeof(secret);
	ret_code_t r = nrf_crypto_ecdh_compute(&ecdhCtx, &pd->PrivKey, &peer,
										   secret, &secretLen);
	(void)nrf_crypto_ecc_public_key_free(&peer);

	CRYPTO_STATUS rc = (r == NRF_SUCCESS && secretLen == 32) ?
					   CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	if (rc == CRYPTO_STATUS_OK)
	{
		memcpy(pDhKey, secret, 32);	// DHKey = X coordinate, big-endian
	}

	// Ephemeral key is single-use: wipe the shared secret, free the key and
	// clear the valid flag on every exit.
	CryptoSecureWipe(secret, sizeof(secret));
	(void)nrf_crypto_ecc_private_key_free(&pd->PrivKey);
	pd->bKeyValid = false;
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
	if ((pCfg->ReqCaps & ~(uint32_t)CRYPTO_CAP_ECDH_P256) != 0)
	{
		return false;	// this engine provides P-256 ECDH only
	}
	if (EnsureCc310() != CRYPTO_STATUS_OK)
	{
		return false;	// nrf_crypto / CC310 not available on this target
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
	pDev->SelfTest       = nullptr;				// optional KAT not provided
	return true;
}

