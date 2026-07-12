/**-------------------------------------------------------------------------
@file	crypto_psa_bm.cpp

@brief	SMP crypto provider: architecture hardware via the PSA Crypto API.

		Not the portable PSA engine: src/crypto/crypto_psa.cpp implements the
		CryptoPsaInit provider slot over any PSA implementation; this file is
		the sdk-nrf-bm hardware slot.

		Implements the CryptoHwInit public symbol (CryptoDev_t hardware engine)
		on top of the ARM PSA Crypto API. This targets Nordic sdk-nrf-bm (the
		bare-metal SDK, not Zephyr), which ships a PSA Crypto implementation over
		CRACEN on nRF54L; the stock sdk-nrf-bm nrf_ble_lesc used the same PSA
		calls. On any platform whose lib provides psa/crypto.h with a hardware
		driver (e.g. CC3xx), the same source uses that accelerator. The private key stays inside the
		PSA keystore (a psa_key_id_t handle), so key material never crosses the
		CryptoDev_t interface and a secure engine can keep it in its own domain.

		The per-instance key context is a PSA key id plus a valid flag, not
		plain bytes, so this engine does NOT set CRYPTO_CAP_PLAIN_KEYCTX. A
		Cryptor composes it with pMem NULL (like the mbedTLS engine) and it then
		runs on its own single context.

		Every hook is SYNCHRONOUS: keygen, ecdh and aes complete in-call. PSA
		supplies its own DRBG (seeded from the platform TRNG through the PSA
		driver), so this engine does not call RngGet.

		Byte order: the SMP toolbox is big-endian. PSA exports and imports ECC
		public keys as SEC1 uncompressed (0x04 || X || Y, big-endian) and the
		raw ECDH result is the big-endian X coordinate, so the buffers map
		straight through after adding or dropping the 0x04 marker byte. AES ECB
		is endian-neutral on the block.

		Requires: psa/crypto.h and a linked PSA implementation with a P-256 and
		AES-128-ECB driver (nrf_security / CRACEN / cc3xx, or mbedTLS PSA).

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "crypto/crypto.h"

// This engine is available on a target when this file is added to the MCU lib
// project. It needs the PSA Crypto headers; if they do not resolve the build
// fails and reports it. A lib links exactly one file defining CryptoHwInit:
// this one, crypto_cc310.cpp, or crypto_hw_none.cpp.
#include "psa/crypto.h"

// Per-instance key context: PSA owns the private key inside its keystore, so
// this holds only the opaque key id and a validity flag.
typedef struct {
	psa_key_id_t KeyId;
	bool         bKeyValid;	// true only while KeyId names a live single-use key
} CryptoPsaData_t;

static_assert(sizeof(CryptoPsaData_t) <= CRYPTO_MEMSIZE_HW,
			  "CRYPTO_MEMSIZE_HW too small for CryptoPsaData_t");

// Resolve the key context: the Cryptor-supplied context, else the engine own.
// Resolve the key context: the Cryptor-supplied context, else the engine own.
// A caller-supplied context is validated like the Init pMem check: non-null
// and word aligned. Returns nullptr on a bad context so the operation fails
// closed.
static CryptoPsaData_t *PsaData(CryptoDev_t * const pDev, void *pKeyCtx)
{
	void *p = pKeyCtx != nullptr ? pKeyCtx :
			  (pDev != nullptr ? pDev->pDevData : nullptr);

	if (p == nullptr || ((uintptr_t)p & (alignof(CryptoPsaData_t) - 1)) != 0)
	{
		return nullptr;
	}

	return (CryptoPsaData_t *)p;
}

// One-time PSA bring-up. psa_crypto_init is idempotent and cheap on repeat.
static CRYPTO_STATUS EnsurePsa(void)
{
	static bool s_Init = false;
	if (!s_Init)
	{
		if (psa_crypto_init() != PSA_SUCCESS)
		{
			return CRYPTO_STATUS_FAIL;
		}
		s_Init = true;
	}
	return CRYPTO_STATUS_OK;
}

// Destroy the instance key and clear the valid flag.
static void PsaKeyReset(CryptoPsaData_t *pd)
{
	if (pd->bKeyValid)
	{
		(void)psa_destroy_key(pd->KeyId);
	}
	pd->KeyId     = PSA_KEY_ID_NULL;
	pd->bKeyValid = false;
}

static CRYPTO_STATUS PsaAes128Ecb(CryptoDev_t * const pDev, const uint8_t Key[16],
								   const uint8_t In[16], uint8_t Out[16], void *pCtx)
{
	(void)pDev; (void)pCtx;
	if (Key == nullptr || In == nullptr || Out == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (EnsurePsa() != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
	psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 128);

	psa_key_id_t kid = PSA_KEY_ID_NULL;
	if (psa_import_key(&attr, Key, 16, &kid) != PSA_SUCCESS)
	{
		return CRYPTO_STATUS_FAIL;
	}

	size_t outLen = 0;
	psa_status_t st = psa_cipher_encrypt(kid, PSA_ALG_ECB_NO_PADDING,
										 In, 16, Out, 16, &outLen);
	(void)psa_destroy_key(kid);
	return (st == PSA_SUCCESS && outLen == 16) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

static CRYPTO_STATUS PsaEcdhKeyGen(CryptoDev_t * const pDev, void *pKeyCtx,
								   uint8_t pPubKey[64], void *pOpCtx)
{
	(void)pOpCtx;
	CryptoPsaData_t *pd = PsaData(pDev, pKeyCtx);
	if (pd == nullptr || pPubKey == nullptr || EnsurePsa() != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Clean start: destroy any prior or partial key so a re-keygen cannot leak
	// a keystore slot.
	PsaKeyReset(pd);

	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
	psa_set_key_bits(&attr, 256);

	psa_key_id_t kid = PSA_KEY_ID_NULL;
	if (psa_generate_key(&attr, &kid) != PSA_SUCCESS)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// PSA exports the public key SEC1 uncompressed: 0x04 || X || Y (65 bytes).
	// The interface is raw X||Y (64), so drop the marker byte.
	uint8_t raw[65];
	size_t  rawLen = 0;
	psa_status_t st = psa_export_public_key(kid, raw, sizeof(raw), &rawLen);
	if (st != PSA_SUCCESS || rawLen != 65 || raw[0] != 0x04)
	{
		(void)psa_destroy_key(kid);
		CryptoSecureWipe(raw, sizeof(raw));
		return CRYPTO_STATUS_FAIL;
	}
	memcpy(pPubKey, &raw[1], 64);
	CryptoSecureWipe(raw, sizeof(raw));
	pd->KeyId     = kid;
	pd->bKeyValid = true;
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS PsaEcdh(CryptoDev_t * const pDev, void *pKeyCtx,
							 const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
							 void *pOpCtx)
{
	(void)pOpCtx;
	CryptoPsaData_t *pd = PsaData(pDev, pKeyCtx);
	if (pd == nullptr || pPeerPubKey == nullptr || pDhKey == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Fail closed when no valid private key is present: Ecdh before KeyGen, or a
	// second Ecdh after the single-use key was consumed.
	if (!pd->bKeyValid)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Peer key to SEC1 uncompressed (0x04 || X || Y). psa_raw_key_agreement
	// rejects a point that is not on the curve, closing the invalid-curve
	// attack (CVE-2018-5383).
	uint8_t peer[65];
	peer[0] = 0x04;
	memcpy(&peer[1], pPeerPubKey, 64);

	uint8_t secret[32];
	size_t  secretLen = 0;
	psa_status_t st = psa_raw_key_agreement(PSA_ALG_ECDH, pd->KeyId,
											peer, sizeof(peer),
											secret, sizeof(secret), &secretLen);
	CRYPTO_STATUS rc = (st == PSA_SUCCESS && secretLen == 32) ?
					   CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	if (rc == CRYPTO_STATUS_OK)
	{
		memcpy(pDhKey, secret, 32);	// DHKey = X coordinate, big-endian
	}

	// Ephemeral key is single-use: wipe the shared secret, destroy the key and
	// clear the valid flag on every exit.
	CryptoSecureWipe(secret, sizeof(secret));
	PsaKeyReset(pd);
	return rc;
}

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}
	if (pCfg->pMem == nullptr || pCfg->MemSize < sizeof(CryptoPsaData_t))
	{
		return false;	// caller must supply per-instance state
	}
	if (((uintptr_t)pCfg->pMem & (alignof(uint32_t) - 1)) != 0)
	{
		return false;	// arena must be word aligned (see CryptoCfg_t pMem)
	}
	if ((pCfg->ReqCaps & ~(uint32_t)(CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_ECDH_P256)) != 0)
	{
		return false;	// this engine provides AES-128 ECB and P-256 ECDH only
	}
	if (EnsurePsa() != CRYPTO_STATUS_OK)
	{
		return false;	// PSA driver not available on this target
	}

	memset(pDev, 0, sizeof(*pDev));
	memset(pCfg->pMem, 0, sizeof(CryptoPsaData_t));
	CryptoPsaData_t *pd = (CryptoPsaData_t *)pCfg->pMem;
	pd->KeyId     = PSA_KEY_ID_NULL;
	pd->bKeyValid = false;

	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "psa-hw";
	pDev->Cap            = CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_ECDH_P256;	// structured key ctx: no PLAIN_KEYCTX
	pDev->KeyCtxSize     = sizeof(CryptoPsaData_t);
	pDev->Props          = CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SECURE_DOMAIN | CRYPTO_PROP_SYNC;
	pDev->EvtCB          = pCfg->EvtCB;			// synchronous engine
	pDev->Aes128Ecb      = PsaAes128Ecb;
	pDev->EcdhP256KeyGen = PsaEcdhKeyGen;
	pDev->EcdhP256       = PsaEcdh;
	pDev->SelfTest       = nullptr;				// optional KAT not provided
	return true;
}

