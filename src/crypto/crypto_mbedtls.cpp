/**-------------------------------------------------------------------------
@file	crypto_mbedtls.cpp

@brief	SMP crypto provider: software mbedTLS (AES + P-256 ECDH).

Portable provider (generic, not platform-specific). Provides CryptoMbedtlsInit
(App-owned instance): the application initializes a CryptoDev_t and injects it
into the subsystem. Compiled
into the lib alongside any targeted provider without symbol collision (the
implementations are file-static; only the const vtable instance is exported).

This is the natural choice where nRF Connect SDK links nrf_security / PSA over
CC3xx (nRF52840, nRF54): on those targets mbedTLS calls are transparently
accelerated by the CryptoCell / CRACEN driver, so the same source uses
hardware where it exists and software where it does not.

Every hook here is SYNCHRONOUS: keygen and ecdh complete in-call and return
CRYPTO_STATUS_OK, so bt_smp.cpp advances without waiting for a controller
event.

Byte order: the SMP toolbox is big-endian. mbedTLS MPIs read/write big-endian
binary, and AES ECB is endian-neutral on the block, so no reversal is needed -
the buffers map straight through.

Requires: mbedtls/aes.h, mbedtls/ecdh.h, mbedtls/ecp.h, mbedtls/bignum.h, and
an entropy/DRBG seeded from the platform TRNG (RngGet). On nRF this is
satisfied by nrf_security (CC3xx) or the vanilla mbedtls module.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"

// This engine is available on a target when this file is added to the MCU lib
// project. It needs the mbedTLS headers and library; if they do not resolve the
// build fails and reports it, which is the intended report.
#include "mbedtls/aes.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#include "mbedtls/bignum.h"

// mbedTLS 3.x hides struct fields behind MBEDTLS_PRIVATE(); 2.x (including the
// mbedTLS bundled with the nRF5 SDK) exposes them directly and does not define
// the macro. Fall back to the bare field name so the same source builds on both.
#ifndef MBEDTLS_PRIVATE
#define MBEDTLS_PRIVATE(member)		member
#endif

// IOsonata TRNG (rng_nrfx.c on Nordic; provide on other platforms).

//-----------------------------------------------------------------------------
// DRBG seeded from the platform TRNG. mbedTLS wants an f_rng for the
// side-channel countermeasures in ecp_mul; we back it with RngGet.
//-----------------------------------------------------------------------------

static int RngForMbed(void *ctx, unsigned char *buf, size_t len)
{
	(void)ctx;
	return RngGet(buf, len) ? 0 : -1;
}

// ECDH state held between keygen and the shared-secret computation. The SMP
// layer drives one SC pairing at a time per device, so a single context is
// sufficient; size up to an array indexed by link if concurrent SC pairings
// on one local device are ever needed.
// The P-256 group parameters are read-only after load and shared by every
// instance (one copy). Only the per-instance secret state (private key d and
// public point Q) lives in App-owned pMem, reached via pDev->pDevData or via a
// Cryptor-supplied pCtx. The control structs sit in pMem; mbedTLS allocates
// MPI limbs through its own allocator, so the real working set also depends on
// the mbedTLS heap configuration.
static mbedtls_ecp_group s_Grp;
static bool              s_GrpInit = false;

typedef struct {
	mbedtls_mpi       PrivD;	// our private key d
	mbedtls_ecp_point PubQ;		// our public key Q
	bool              bInit;	// keygen has run for this instance
} CryptoMbedtlsData_t;

static CryptoMbedtlsData_t *MbedData(CryptoDev_t * const pDev, void *pKeyCtx)
{
	return (CryptoMbedtlsData_t *)CryptoResolveKeyCtx(pDev, pKeyCtx,
														 alignof(CryptoMbedtlsData_t));
}

// Free and reinit an instance key context (private key d and public point Q)
// and mark it not-keyed. Used to start keygen from a clean state, to wipe a
// partially generated key on failure, and to wipe the single-use ephemeral key
// after a DH.
static void MbedKeyReset(CryptoMbedtlsData_t *pd)
{
	mbedtls_mpi_free(&pd->PrivD);
	mbedtls_mpi_init(&pd->PrivD);
	mbedtls_ecp_point_free(&pd->PubQ);
	mbedtls_ecp_point_init(&pd->PubQ);
	pd->bInit = false;
}

static int EnsureGroup(void)
{
	if (s_GrpInit)
	{
		return 0;
	}
	mbedtls_ecp_group_init(&s_Grp);
	if (mbedtls_ecp_group_load(&s_Grp, MBEDTLS_ECP_DP_SECP256R1) != 0)
	{
		return -1;
	}
	s_GrpInit = true;
	return 0;
}

//-----------------------------------------------------------------------------
// Provider implementation (file-static; exposed only via CryptoMbedtlsInit)
//-----------------------------------------------------------------------------

static CRYPTO_STATUS MbedAes128Ecb(CryptoDev_t * const pDev,
								   const uint8_t Key[16], const uint8_t In[16],
								   uint8_t Out[16], void *pCtx)
{
	(void)pDev; (void)pCtx;

	mbedtls_aes_context aes;
	mbedtls_aes_init(&aes);
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;
	if (mbedtls_aes_setkey_enc(&aes, Key, 128) != 0 ||
		mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, In, Out) != 0)
	{
		memset(Out, 0, 16);		// fail loud
		st = CRYPTO_STATUS_FAIL;
	}
	mbedtls_aes_free(&aes);
	return st;
}

static CRYPTO_STATUS MbedEcdhKeyGen(CryptoDev_t * const pDev, void *pKeyCtx,
									uint8_t pPubKey[64], void *pOpCtx)
{
	(void)pOpCtx;
	CryptoMbedtlsData_t *pd = MbedData(pDev, pKeyCtx);
	if (pd == nullptr || EnsureGroup() != 0)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Start from a clean context: wipe any prior or partial key state so a
	// re-keygen does not leak the previous d/Q allocations.
	MbedKeyReset(pd);

	// Generate d and Q = d*G into this instance's context.
	if (mbedtls_ecdh_gen_public(&s_Grp, &pd->PrivD, &pd->PubQ,
								RngForMbed, nullptr) != 0)
	{
		MbedKeyReset(pd);
		return CRYPTO_STATUS_FAIL;
	}

	// Export Q as raw X||Y (32 + 32), big-endian, matching the SMP toolbox.
	if (mbedtls_mpi_write_binary(&pd->PubQ.MBEDTLS_PRIVATE(X), &pPubKey[0],  32) != 0 ||
		mbedtls_mpi_write_binary(&pd->PubQ.MBEDTLS_PRIVATE(Y), &pPubKey[32], 32) != 0)
	{
		MbedKeyReset(pd);	// wipe the partially generated key
		return CRYPTO_STATUS_FAIL;
	}
	pd->bInit = true;
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS MbedEcdh(CryptoDev_t * const pDev, void *pKeyCtx,
					const uint8_t pPeerPubKey[64], uint8_t pDhKey[32], void *pOpCtx)
{
	(void)pOpCtx;
	CryptoMbedtlsData_t *pd = MbedData(pDev, pKeyCtx);
	if (pd == nullptr || !pd->bInit)
	{
		return CRYPTO_STATUS_FAIL;	// keygen must run first
	}

	mbedtls_ecp_point peerQ;
	mbedtls_mpi       z;
	mbedtls_ecp_point_init(&peerQ);
	mbedtls_mpi_init(&z);

	CRYPTO_STATUS rc = CRYPTO_STATUS_FAIL;

	// Load peer Q from raw X||Y. Z = 1 (affine). Reject an off-curve peer key
	// before the DH (invalid-curve attack, CVE-2018-5383); compute_shared does
	// not perform this check.
	if (mbedtls_mpi_read_binary(&peerQ.MBEDTLS_PRIVATE(X), &pPeerPubKey[0],  32) == 0 &&
		mbedtls_mpi_read_binary(&peerQ.MBEDTLS_PRIVATE(Y), &pPeerPubKey[32], 32) == 0 &&
		mbedtls_mpi_lset(&peerQ.MBEDTLS_PRIVATE(Z), 1) == 0 &&
		mbedtls_ecp_check_pubkey(&s_Grp, &peerQ) == 0)
	{
		if (mbedtls_ecdh_compute_shared(&s_Grp, &z, &peerQ, &pd->PrivD,
										RngForMbed, nullptr) == 0)
		{
			// DHKey is the X coordinate of the shared point, 32 bytes BE.
			if (mbedtls_mpi_write_binary(&z, pDhKey, 32) == 0)
			{
				rc = CRYPTO_STATUS_OK;
			}
		}
	}

	mbedtls_mpi_free(&z);
	mbedtls_ecp_point_free(&peerQ);

	// Ephemeral key is single-use: wipe d and Q and mark the instance not-keyed.
	MbedKeyReset(pd);
	return rc;
}

// Self-test: BLE spec P-256 DH known vector (Vol 3 Part H 2.3.5.6.1). Loads the
// fixed private key A and peer public key B, computes the shared secret, and
// checks it against the expected DHKey. Returns 0 on PASS. All values big-
// endian as the spec writes them. Verifies the mbedTLS ECDH path and byte
// order on the target (including a CC3xx-accelerated build).
static int MbedSelfTest(CryptoDev_t * const pDev)
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

	if (EnsureGroup() != 0)
	{
		return -1;
	}

	mbedtls_mpi       d;
	mbedtls_ecp_point peerQ;
	mbedtls_mpi       z;
	mbedtls_mpi_init(&d);
	mbedtls_ecp_point_init(&peerQ);
	mbedtls_mpi_init(&z);

	int rc = -1;
	uint8_t dh[32];

	if (mbedtls_mpi_read_binary(&d, privA, 32) == 0 &&
		mbedtls_mpi_read_binary(&peerQ.MBEDTLS_PRIVATE(X), &pubB[0],  32) == 0 &&
		mbedtls_mpi_read_binary(&peerQ.MBEDTLS_PRIVATE(Y), &pubB[32], 32) == 0 &&
		mbedtls_mpi_lset(&peerQ.MBEDTLS_PRIVATE(Z), 1) == 0)
	{
		if (mbedtls_ecdh_compute_shared(&s_Grp, &z, &peerQ, &d,
										RngForMbed, nullptr) == 0 &&
			mbedtls_mpi_write_binary(&z, dh, 32) == 0)
		{
			rc = (memcmp(dh, dhExpect, 32) == 0) ? 0 : -2;	// -2 = wrong DHKey
		}
	}

	CryptoSecureWipe(dh, sizeof(dh));	// wipe the test shared secret
	mbedtls_mpi_free(&z);
	mbedtls_ecp_point_free(&peerQ);
	mbedtls_mpi_free(&d);
	return rc;
}

//-----------------------------------------------------------------------------
// Exported provider instance
//-----------------------------------------------------------------------------

bool CryptoMbedtlsInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (!CryptoCfgValidate(pDev, pCfg, sizeof(CryptoMbedtlsData_t),
						   CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_ECDH_P256))
	{
		return false;
	}

	memset(pDev, 0, sizeof(*pDev));

	// Lifecycle rule: INIT ONCE per pMem. Call CryptoMbedtlsInit with fresh
	// App-owned memory that does not already hold an initialized mbedTLS
	// context. A second call on the same live pMem leaks the prior d/Q
	// allocations: this init does not free them first. The memset only clears
	// the field bytes (a clean start for fresh, even non-zeroed, memory); it
	// does not release any mbedTLS-internal buffers a prior init attached. To
	// bind a new instance to the same arena, obtain fresh memory. In-place
	// reinit is not supported (there is no Deinit path today).
	CryptoMbedtlsData_t *pd = (CryptoMbedtlsData_t *)pCfg->pMem;
	memset(pd, 0, sizeof(*pd));
	mbedtls_mpi_init(&pd->PrivD);
	mbedtls_ecp_point_init(&pd->PubQ);
	pd->bInit = false;

	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "mbedtls";
	pDev->Cap            = CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_ECDH_P256;
	pDev->KeyCtxSize     = sizeof(CryptoMbedtlsData_t);
	pDev->Props          = CRYPTO_PROP_SYNC;
	pDev->EvtCB          = pCfg->EvtCB;			// synchronous engine
	pDev->Aes128Ecb      = MbedAes128Ecb;
	pDev->EcdhP256KeyGen = MbedEcdhKeyGen;
	pDev->EcdhP256       = MbedEcdh;
	pDev->SelfTest       = MbedSelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) && MbedSelfTest(pDev) != 0)
	{
		return false;
	}
	return true;
}

size_t CryptoMbedtlsMemSize(void)
{
	return sizeof(CryptoMbedtlsData_t);
}

