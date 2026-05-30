/**-------------------------------------------------------------------------
@file	bt_smp_crypto_mbedtls.cpp

@brief	SMP crypto provider: software mbedTLS (AES + P-256 ECDH).

Portable fallback for controllers that do NOT expose the LE crypto HCI
commands, and the natural choice where nRF Connect SDK already links
nrf_security / PSA over CC3xx (nRF52840, nRF54): on those targets mbedTLS
calls are transparently accelerated by the CryptoCell / CRACEN driver, so
the same source uses hardware where it exists and software where it does
not.

Unlike the HCI-offload provider, every hook here is SYNCHRONOUS: keygen and
ecdh complete in-call and return BT_SMP_CRYPTO_OK, so bt_smp.cpp advances
without waiting for a controller event.

Byte order: the SMP toolbox is big-endian. mbedTLS MPIs read/write big-
endian binary, and AES ECB is endian-neutral on the block, so no reversal is
needed - the buffers map straight through.

Selection: link THIS file instead of bt_smp_crypto_sdc.cpp. The two define
the same weak-overriding symbols; linking both is a multiple-definition
error by design, so a target picks exactly one.

Requires: mbedtls/aes.h, mbedtls/ecdh.h, mbedtls/ecp.h, mbedtls/bignum.h,
and an entropy/DRBG seeded from the platform TRNG (RngGet). On nRF this is
satisfied by nrf_security (CC3xx) or the vanilla mbedtls module.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "bluetooth/bt_smp_crypto.h"

#include "mbedtls/aes.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#include "mbedtls/bignum.h"

// mbedTLS 3.x hides struct fields behind MBEDTLS_PRIVATE(); 2.x (including
// the mbedTLS bundled with the nRF5 SDK) exposes them directly and does not
// define the macro. Fall back to the bare field name so the same source
// builds on both.
#ifndef MBEDTLS_PRIVATE
#define MBEDTLS_PRIVATE(member)		member
#endif

// IOsonata TRNG (rng_nrfx.c on Nordic; provide on other platforms).
extern "C" bool RngGet(uint8_t *pBuff, size_t Len);

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
static mbedtls_ecp_group s_Grp;
static mbedtls_mpi       s_PrivD;	// our private key d
static mbedtls_ecp_point s_PubQ;	// our public key Q
static bool              s_Init = false;

static int EnsureGroup(void)
{
	if (s_Init)
	{
		return 0;
	}
	mbedtls_ecp_group_init(&s_Grp);
	mbedtls_mpi_init(&s_PrivD);
	mbedtls_ecp_point_init(&s_PubQ);
	if (mbedtls_ecp_group_load(&s_Grp, MBEDTLS_ECP_DP_SECP256R1) != 0)
	{
		return -1;
	}
	s_Init = true;
	return 0;
}

//-----------------------------------------------------------------------------
// Provider hooks
//-----------------------------------------------------------------------------

extern "C" void BtSmpCryptoAes128(BtHciDevice_t * const pDev,
								  const uint8_t Key[16], const uint8_t In[16],
								  uint8_t Out[16])
{
	(void)pDev;

	mbedtls_aes_context aes;
	mbedtls_aes_init(&aes);
	if (mbedtls_aes_setkey_enc(&aes, Key, 128) != 0 ||
		mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, In, Out) != 0)
	{
		memset(Out, 0, 16);		// fail loud
	}
	mbedtls_aes_free(&aes);
}

extern "C" int BtSmpCryptoP256KeyGen(BtHciDevice_t * const pDev, uint8_t pPubKey[64])
{
	(void)pDev;

	if (EnsureGroup() != 0)
	{
		return BT_SMP_CRYPTO_FAIL;
	}

	// Generate d and Q = d*G.
	if (mbedtls_ecdh_gen_public(&s_Grp, &s_PrivD, &s_PubQ,
								RngForMbed, nullptr) != 0)
	{
		return BT_SMP_CRYPTO_FAIL;
	}

	// Export Q as raw X||Y (32 + 32), big-endian, matching the SMP toolbox.
	if (mbedtls_mpi_write_binary(&s_PubQ.MBEDTLS_PRIVATE(X), &pPubKey[0],  32) != 0 ||
		mbedtls_mpi_write_binary(&s_PubQ.MBEDTLS_PRIVATE(Y), &pPubKey[32], 32) != 0)
	{
		return BT_SMP_CRYPTO_FAIL;
	}
	return BT_SMP_CRYPTO_OK;
}

extern "C" int BtSmpCryptoEcdh(BtHciDevice_t * const pDev,
							   const uint8_t pPeerPubKey[64], uint8_t pDhKey[32])
{
	(void)pDev;

	if (!s_Init)
	{
		return BT_SMP_CRYPTO_FAIL;	// keygen must run first
	}

	mbedtls_ecp_point peerQ;
	mbedtls_mpi       z;
	mbedtls_ecp_point_init(&peerQ);
	mbedtls_mpi_init(&z);

	int rc = BT_SMP_CRYPTO_FAIL;

	// Load peer Q from raw X||Y. Z = 1 (affine).
	if (mbedtls_mpi_read_binary(&peerQ.MBEDTLS_PRIVATE(X), &pPeerPubKey[0],  32) == 0 &&
		mbedtls_mpi_read_binary(&peerQ.MBEDTLS_PRIVATE(Y), &pPeerPubKey[32], 32) == 0 &&
		mbedtls_mpi_lset(&peerQ.MBEDTLS_PRIVATE(Z), 1) == 0)
	{
		if (mbedtls_ecdh_compute_shared(&s_Grp, &z, &peerQ, &s_PrivD,
										RngForMbed, nullptr) == 0)
		{
			// DHKey is the X coordinate of the shared point, 32 bytes BE.
			if (mbedtls_mpi_write_binary(&z, pDhKey, 32) == 0)
			{
				rc = BT_SMP_CRYPTO_OK;
			}
		}
	}

	mbedtls_mpi_free(&z);
	mbedtls_ecp_point_free(&peerQ);
	return rc;
}

extern "C" void BtSmpCryptoRand(uint8_t *pBuf, size_t Len)
{
	RngGet(pBuf, Len);
}
