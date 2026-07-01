/**-------------------------------------------------------------------------
@file	crypto.cpp

@brief	Base crypto layer: the Cryptor instance and the provider registry.

This file holds the generic, provider-independent part of the crypto module:

  - Cryptor: the per-use-case instance over one or more engines. It references
	engine objects (CryptoDev_t) and presents its own CryptoDev_t handle that
	forwards each operation to the engine providing that capability, using the
	Cryptor's own per-instance key state. Several use cases can share one engine
	(each Cryptor keeps a separate key arena), or one use case can compose
	several single-capability engines (software ECDH, controller AES).

  - Weak fail-closed provider inits. An application links only the providers its
	target ships (crypto_uecc.cpp, crypto_mbedtls.cpp, a port CryptoHwInit). A
	reference to a provider that is not linked resolves to the weak definition
	here, which returns false, so a missing provider fails closed instead of
	breaking the link. The real provider's strong definition overrides the weak
	one when linked.

  - CryptoInit: the default config-driven selector. Weak, so a port can install
	a different selection policy.

Per-instance key state: a Cryptor created with a pMem arena (CryptoCfg_t)
forwards the ECDH operations with that arena as the engine key context,
overriding the engine's own context, so two Cryptors over one engine do not
collide. The
arena holds the ECDH private key, the only per-instance secret today; AES is stateless and ignores it. Size the arena with the ECDH engine's
CRYPTO_MEMSIZE_* macro. The arena must be a plain-byte context that is valid
when zeroed (the micro-ecc engine, and slot-handle hardware engines). An engine
whose per-instance context needs structured init (mbedTLS) is composed with
pMem NULL: the Cryptor then forwards NULL and that engine uses its own
initialized context as a single shared instance.

The ECDH ops carry a separate pKeyCtx (per-instance key context) and pOpCtx
(operation/completion context), so the Cryptor forwards its key arena as pKeyCtx
without consuming the operation context. Async completion correlation through a
Cryptor is a later refinement that arrives with the first async hardware engine.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"

//-----------------------------------------------------------------------------
// Secure wipe: volatile store so the compiler cannot elide the clear. Used for
// key material and intermediate secrets across the providers.
//-----------------------------------------------------------------------------

void CryptoSecureWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len--)
	{
		*p++ = 0;
	}
}

//-----------------------------------------------------------------------------
// Weak fail-closed provider inits. Overridden by the real provider when linked.
//-----------------------------------------------------------------------------

__attribute__((weak)) bool CryptoUeccInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	(void)pDev; (void)pCfg;
	return false;	// micro-ecc engine not linked
}

__attribute__((weak)) bool CryptoMbedtlsInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	(void)pDev; (void)pCfg;
	return false;	// mbedTLS engine not linked
}

__attribute__((weak)) bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	(void)pDev; (void)pCfg;
	return false;	// no hardware engine linked for this target
}

// Weak: 0 when the mbedTLS provider is not linked. The real provider overrides
// this with sizeof(CryptoMbedtlsData_t) so an App can size pMem exactly.
__attribute__((weak)) size_t CryptoMbedtlsMemSize(void)
{
	return 0;
}

// Default selector. AUTO tries hardware first, then software unless the caller
// set CRYPTO_FLAG_NO_FALLBACK. An explicit Provider selects one directly. Each
// call resolves to the strong provider init when linked, or the weak false
// above when not, so an absent provider is simply skipped.
__attribute__((weak)) bool CryptoInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	switch (pCfg->Provider)
	{
	case CRYPTO_PROVIDER_HW:
		return CryptoHwInit(pDev, pCfg);
	case CRYPTO_PROVIDER_MBEDTLS:
		return CryptoMbedtlsInit(pDev, pCfg);
	case CRYPTO_PROVIDER_UECC:
		return CryptoUeccInit(pDev, pCfg);
	case CRYPTO_PROVIDER_AUTO:
	default:
		if (CryptoHwInit(pDev, pCfg))
		{
			return true;
		}
		if ((pCfg->Flags & CRYPTO_FLAG_NO_FALLBACK) == 0)
		{
			if (CryptoMbedtlsInit(pDev, pCfg))
			{
				return true;
			}
			if (CryptoUeccInit(pDev, pCfg))
			{
				return true;
			}
		}
		return false;
	}
}

//-----------------------------------------------------------------------------
// Cryptor: per-use-case instance over one or more engines.
//-----------------------------------------------------------------------------

// Find the first composed engine that provides every bit in Cap.
static CryptoDev_t *CryptorPick(Cryptor_t * const pInst, uint32_t Cap)
{
	for (int i = 0; i < pInst->NbEng; i++)
	{
		CryptoDev_t *pEng = pInst->pEng[i];
		if (pEng != nullptr && (pEng->Cap & Cap) == Cap)
		{
			return pEng;
		}
	}
	return nullptr;
}

static CRYPTO_STATUS CryptorAes128Ecb(CryptoDev_t * const pDev,
									  const uint8_t Key[16], const uint8_t In[16],
									  uint8_t Out[16], void *pCtx)
{
	Cryptor_t *pInst = (Cryptor_t *)pDev->pDevData;
	CryptoDev_t *pEng = CryptorPick(pInst, CRYPTO_CAP_AES128_ECB);
	if (pEng == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	return CryptoAes128Ecb(pEng, Key, In, Out, pCtx);
}

static CRYPTO_STATUS CryptorEcdhP256KeyGen(CryptoDev_t * const pDev,
										   void *pKeyCtx, uint8_t pPubKey[64],
										   void *pOpCtx)
{
	Cryptor_t *pInst = (Cryptor_t *)pDev->pDevData;
	CryptoDev_t *pEng = CryptorPick(pInst, CRYPTO_CAP_ECDH_P256);
	if (pEng == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	void *pKey = (pKeyCtx != nullptr) ? pKeyCtx : pInst->pMem;
	return CryptoEcdhP256KeyGen(pEng, pKey, pPubKey, pOpCtx);
}

static CRYPTO_STATUS CryptorEcdhP256(CryptoDev_t * const pDev, void *pKeyCtx,
									 const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
									 void *pOpCtx)
{
	Cryptor_t *pInst = (Cryptor_t *)pDev->pDevData;
	CryptoDev_t *pEng = CryptorPick(pInst, CRYPTO_CAP_ECDH_P256);
	if (pEng == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	void *pKey = (pKeyCtx != nullptr) ? pKeyCtx : pInst->pMem;
	return CryptoEcdhP256(pEng, pKey, pPeerPubKey, pDhKey, pOpCtx);
}

static int CryptorSelfTest(CryptoDev_t * const pDev)
{
	Cryptor_t *pInst = (Cryptor_t *)pDev->pDevData;
	for (int i = 0; i < pInst->NbEng; i++)
	{
		if (pInst->pEng[i] != nullptr)
		{
			int rc = CryptoSelfTest(pInst->pEng[i]);
			if (rc != 0)
			{
				return rc;
			}
		}
	}
	return 0;
}

// Union of capabilities across the composed engines.
static uint32_t CryptorCapUnion(CryptoDev_t * const pEng[], int NbEng)
{
	uint32_t cap = 0;
	for (int i = 0; i < NbEng; i++)
	{
		if (pEng[i] != nullptr)
		{
			cap |= pEng[i]->Cap;
		}
	}
	return cap;
}

// Build the forwarding handle. Each operation pointer is set only where a
// composed engine covers it, so the presented Cap and the pointers agree.
static bool CryptorBuild(Cryptor_t * const pInst, const CryptoCfg_t *pCfg,
						 CryptoDev_t * const pEng[], int NbEng)
{
	if (pInst == nullptr || pCfg == nullptr || pEng == nullptr ||
		NbEng <= 0 || NbEng > CRYPTOR_MAX_ENGINE)
	{
		return false;
	}

	uint32_t have = CryptorCapUnion(pEng, NbEng);
	uint32_t req  = (pCfg->ReqCaps != 0) ? pCfg->ReqCaps : have;
	if ((have & req) != req)
	{
		return false;	// a required capability is not covered; fail closed
	}

	// A supplied key arena (pMem) is forwarded to the ECDH engine as its per-
	// instance key context. That is valid only when the engine key context is
	// plain zeroable bytes (CRYPTO_CAP_PLAIN_KEYCTX). An engine needing structured
	// context init (mbedTLS) must be composed with pMem NULL so it uses its own
	// initialized context. Fail closed otherwise.
	if (pCfg->pMem != nullptr)
	{
		CryptoDev_t *pEcdh = nullptr;
		for (int i = 0; i < NbEng; i++)
		{
			if (pEng[i] != nullptr && (pEng[i]->Cap & CRYPTO_CAP_ECDH_P256) != 0)
			{
				pEcdh = pEng[i];
				break;
			}
		}
		if (pEcdh != nullptr)
		{
			// pMem becomes this engine per-instance key context. It must be a
			// plain zeroable context (CRYPTO_CAP_PLAIN_KEYCTX), and the arena
			// must be at least the engine key-context size (KeyCtxSize) so a
			// keyed op does not write past pMem.
			if ((pEcdh->Cap & CRYPTO_CAP_PLAIN_KEYCTX) == 0)
			{
				return false;
			}
			if (pEcdh->KeyCtxSize == 0 || pCfg->MemSize < pEcdh->KeyCtxSize)
			{
				return false;
			}
		}
	}

	memset(pInst, 0, sizeof(Cryptor_t));
	for (int i = 0; i < NbEng; i++)
	{
		pInst->pEng[i] = pEng[i];
	}
	pInst->NbEng   = NbEng;
	pInst->ReqCaps = req;
	pInst->pMem    = pCfg->pMem;
	pInst->MemSize = pCfg->MemSize;

	// A plain-byte key arena (micro-ecc, slot-handle engines) is valid zeroed.
	// An engine needing structured context init is composed with pMem NULL.
	if (pInst->pMem != nullptr)
	{
		memset(pInst->pMem, 0, pInst->MemSize);
	}

	CryptoDev_t *d = &pInst->Dev;
	d->pDevData       = pInst;
	d->pName          = "cryptor";
	d->Cap            = have & ~(uint32_t)CRYPTO_CAP_PLAIN_KEYCTX;	// property bit, not a queryable cap
	d->EvtCB          = pCfg->EvtCB;
	d->Aes128Ecb      = (CryptorPick(pInst, CRYPTO_CAP_AES128_ECB) != nullptr) ? CryptorAes128Ecb : nullptr;
	d->EcdhP256KeyGen = (CryptorPick(pInst, CRYPTO_CAP_ECDH_P256) != nullptr) ? CryptorEcdhP256KeyGen : nullptr;
	d->EcdhP256       = (CryptorPick(pInst, CRYPTO_CAP_ECDH_P256) != nullptr) ? CryptorEcdhP256 : nullptr;
	d->SelfTest       = CryptorSelfTest;
	return true;
}

bool CryptorInit(Cryptor_t * const pInst, const CryptoCfg_t *pCfg,
							CryptoDev_t * const pEng)
{
	CryptoDev_t *one[1] = { pEng };
	return CryptorBuild(pInst, pCfg, one, 1);
}

bool CryptorComposeInit(Cryptor_t * const pInst, const CryptoCfg_t *pCfg,
								   CryptoDev_t * const pEng[], int NbEng)
{
	return CryptorBuild(pInst, pCfg, pEng, NbEng);
}

CryptoDev_t * CryptorHandle(Cryptor_t * const pInst)
{
	return (pInst != nullptr) ? &pInst->Dev : nullptr;
}
