/**-------------------------------------------------------------------------
@file	crypto_cc3xx.cpp

@brief	CryptoDev_t hardware engine on the Arm CC3xx low level driver

		P-256 ECDH for LE Secure Connections on the nRF52840 CryptoCell
		CC310, built directly on the open Arm register level driver
		(Mbed-TLS/tf-psa-crypto-drivers, vendor/arm/cc3xx/low_level_driver)
		with no vendor blob, no PSA core, no nRF5 SDK. It is an alternative
		to crypto_cc310.cpp (CRYS blob) and to the PSA-over-CC310 path; a lib
		links exactly one file defining CryptoHwInit: this one,
		crypto_cc310.cpp, crypto_psa_bm.cpp, or crypto_hw_none.cpp.

		Key and point byte order: the driver reads and writes its PKA
		registers with a per-word endianness swap, so its uint32_t buffers
		hold big-endian byte strings. The CryptoDev_t interface uses SEC1
		big-endian X||Y public keys and a big-endian X-coordinate DHKey, so
		buffers map straight through with no reordering. The uint32_t buffers
		are word aligned as required; the interface byte buffers are copied
		through aligned locals rather than cast, so no unaligned access is
		assumed on the interface side.

		Randomness: the driver RNG entry points are provided by the local
		cc3xx_rng.c over IOsonata RngGet, so the private key scalar drawn
		inside cc3xx_lowlevel_ecdsa_genkey comes from RngGet. The CC3xx noise
		source, entropy conditioner and DRBG modules are not built.

		Peer public key validation: the peer point is checked to be on the
		P-256 curve with cc3xx_lowlevel_ec_allocate_point_from_data (NIST
		SP800-186 D.1 point check) before the DH, closing the invalid-curve
		attack (CVE-2018-5383). The driver ECDH itself loads the raw
		coordinates without validation, so this engine performs the check,
		the same responsibility crypto_uecc.cpp and crypto_cc310.cpp take.

		Peripheral enable: the CryptoCell wrapper enable register
		(NRF_CRYPTOCELL->ENABLE) is turned on at init and left on. This is a
		deliberate power tradeoff, not the gating crypto_cc310.cpp uses (that
		engine disables ENABLE after each operation): the Nordic wrapper has
		no ready flag and toggling ENABLE per operation raced the block
		power-up (see Cc3xxOn). Measure the idle current with ENABLE set
		before shipping; a pairing-scoped enable/disable is the planned
		refinement. The driver is polling, so no interrupt setup is needed.

		Single instance: the driver holds global engine state (PKA register
		file), so one operation runs at a time. A PRIMASK based try-acquire
		guard enforces this: a second concurrent operation returns
		CRYPTO_STATUS_FAIL instead of corrupting the engine state. On the
		nRF52840 SDC path the SoftDevice and SDC use the ECB and CCM
		peripherals, never the CryptoCell, so there is no contention. The per-instance key
		context holds only the generated private key bytes and a valid flag,
		supplied by the caller through the CryptoCfg_t arena, as with the
		CRYS engine.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "crypto/crypto.h"

// The cc3xx headers manage their own C linkage, so they are included directly.
#include "cc3xx_ecdh.h"
#include "cc3xx_ecdsa.h"
#include "cc3xx_ec.h"
#include "cc3xx_ec_curve_data.h"
#include "cc3xx_dev.h"			// P_CC3XX register map
#include "cc3xx_ahbm.h"			// AHB manager configuration constants
#include "cc3xx_engine_state.h"	// cc3xx_engine_in_use, CC3XX_ENGINE_NONE

// P-256 field element size, the modulus size the driver uses for secp256r1.
#define CC3XX_P256_BYTES		32
// Word count for a 32-byte big-endian value passed to the driver.
#define CC3XX_P256_WORDS		(CC3XX_P256_BYTES / sizeof(uint32_t))

// Per-instance key context: the 32-byte private key plus a valid flag. Held
// as words for direct handoff to the driver, which takes uint32_t buffers.
typedef struct {
	uint32_t PrivKey[CC3XX_P256_WORDS];
	bool     bKeyValid;	// true only while PrivKey is a live single-use key
} CryptoCc3xxData_t;

static_assert(sizeof(CryptoCc3xxData_t) <= CRYPTO_MEMSIZE_HW,
			  "CRYPTO_MEMSIZE_HW too small for CryptoCc3xxData_t");

static bool s_bCcInit;

// Single-operation guard for the driver global engine state (PKA register
// file). Try-acquire under PRIMASK: a second concurrent operation fails
// instead of trampling the engine. Every exit path of an operation that
// acquired the guard must release it.
static volatile bool s_bCc3xxBusy;

static bool Cc3xxAcquire(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	bool acquired = (s_bCc3xxBusy == false);
	if (acquired)
	{
		s_bCc3xxBusy = true;
	}

	if (primask == 0)
	{
		__enable_irq();
	}

	return acquired;
}

static inline void Cc3xxRelease(void)
{
	s_bCc3xxBusy = false;
}

// Resolve the key context: the Cryptor-supplied context, else the engine own.
static inline CryptoCc3xxData_t *Cc3xxData(CryptoDev_t * const pDev, void *pKeyCtx)
{
	return (CryptoCc3xxData_t *)(pKeyCtx != nullptr ? pKeyCtx : pDev->pDevData);
}

// Enable the CryptoCell subsystem. The Nordic wrapper exposes only an ENABLE
// register with no ready flag, and the CC310 internal blocks (PKA, DMA) need
// a settling period after ENABLE is asserted before they respond. Enabling
// once at init and leaving it on avoids racing that power-up on every
// operation. Toggling ENABLE per operation produced a timing dependent
// failure: the first PKA or DMA access raced the wake-up at full speed, while
// single stepping gave the block time to settle and masked it.
static inline void Cc3xxOn(void)
{
	if (NRF_CRYPTOCELL->ENABLE == 0)
	{
		NRF_CRYPTOCELL->ENABLE = 1;
		// Allow the subsystem to settle before first access. There is no
		// ready flag to poll, so a short fixed spin covers the power-up.
		for (volatile uint32_t i = 0; i < 256; i++)
		{
		}
	}
}

// One-time driver bring-up: the subset of the upstream cc3xx_lowlevel_init
// that applies to this build (endian select, engine reset, AHBM setup; the
// DFA setup returns early on CC310, and no DPA, DMA remap, TRNG or DRBG init
// is configured). Verified against tf-psa-crypto-drivers commit 27c8ccd;
// re-check this function against cc3xx_lowlevel_init on every driver update.
// Guarded by the flag so repeat calls are cheap.
static CRYPTO_STATUS EnsureCc3xx(void)
{
	if (s_bCcInit)
	{
		return CRYPTO_STATUS_OK;
	}

	Cc3xxOn();

	// CryptoCell bring-up, the subset of cc3xx_lowlevel_init that applies to
	// this build (CC310, DPA mitigations off, EC/PKA and RNG only).

	// Mirror of the upstream check_features assert for CC3XX_CONFIG_RNG_ENABLE.
	// The CC3xx RNG hardware itself is unused (randomness comes from RngGet);
	// this only confirms the expected CC310 feature set is present.
	if ((P_CC3XX->host_rgf.host_boot & (1U << 11)) == 0)	// RNG_EXISTS_LOCAL
	{
		NRF_CRYPTOCELL->ENABLE = 0;	// do not leave the block powered on failure
		return CRYPTO_STATUS_FAIL;
	}

	// Configure the whole engine little-endian.
	P_CC3XX->host_rgf.host_rgf_endian = 0x0U;

	// Reset the crypto engine to passthrough / idle.
	cc3xx_engine_in_use = CC3XX_ENGINE_NONE;
	P_CC3XX->cc_ctl.crypto_ctl = CC3XX_ENGINE_NONE;

	// Set up the AHB5 manager interface. The AHBM registers require the DMA
	// clock enabled while they are written, then it is disabled to save power.
	P_CC3XX->misc.dma_clk_enable = 0x1U;
	P_CC3XX->ahb.ahbm_hprot = CC3XX_AHBM_HPROT_DATA |
							  CC3XX_AHBM_HPROT_PRIVILEGED |
							  CC3XX_AHBM_HPROT_CACHEABLE;
	P_CC3XX->ahb.ahbm_singles = CC3XX_AHBM_BURST_INCR4_TRANSACTIONS;
	P_CC3XX->ahb.ahbm_hnonsec = CC3XX_AHBM_SECURE_TRANSACTIONS;
	P_CC3XX->misc.dma_clk_enable = 0x0U;

	// CC310 has no always-on interface, so there is no DFA countermeasure
	// setup to perform (cc3xx_lowlevel_init returns early on CC310 here).


	s_bCcInit = true;
	return CRYPTO_STATUS_OK;
}

// Wipe the instance private key and clear the valid flag.
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
	if (pd == nullptr || EnsureCc3xx() != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	if (Cc3xxAcquire() == false)
	{
		return CRYPTO_STATUS_FAIL;	// engine busy with another operation
	}

	// Clean start: wipe any prior key so a re-keygen cannot leak key material.
	Cc3xxKeyReset(pd);

	uint32_t pubX[CC3XX_P256_WORDS];
	uint32_t pubY[CC3XX_P256_WORDS];
	size_t   privSize = 0;
	size_t   pubXSize = 0;
	size_t   pubYSize = 0;

	Cc3xxOn();
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
	// On success the driver always reports and writes the full P-256 element
	// size (verified upstream); the size checks guard driver drift the same
	// way the ECDH path checks secretSize.
	if (r != CC3XX_ERR_SUCCESS ||
		privSize != CC3XX_P256_BYTES ||
		pubXSize != CC3XX_P256_BYTES ||
		pubYSize != CC3XX_P256_BYTES)
	{
		Cc3xxKeyReset(pd);
		Cc3xxRelease();
		return CRYPTO_STATUS_FAIL;
	}

	// The driver writes big-endian byte strings; the interface public key is
	// SEC1 X||Y big-endian, so copy the register words out as bytes directly.
	memcpy(&pPubKey[0],  pubX, CC3XX_P256_BYTES);
	memcpy(&pPubKey[32], pubY, CC3XX_P256_BYTES);

	pd->bKeyValid = true;
	Cc3xxRelease();
	return CRYPTO_STATUS_OK;
}

// Core ECDH over a supplied private key and SEC1 peer public key. Used by both
// the interface Ecdh (instance key) and the self test (spec key).
static CRYPTO_STATUS Cc3xxEcdhRaw(const uint32_t privKey[CC3XX_P256_WORDS],
								  const uint8_t pPeerPubKey[64],
								  uint8_t pDhKey[32])
{
	// Peer key halves are SEC1 big-endian X then Y. Copy through aligned
	// locals so no unaligned word access is assumed on the byte buffer.
	uint32_t peerX[CC3XX_P256_WORDS];
	uint32_t peerY[CC3XX_P256_WORDS];
	memcpy(peerX, &pPeerPubKey[0],  CC3XX_P256_BYTES);
	memcpy(peerY, &pPeerPubKey[32], CC3XX_P256_BYTES);

	if (Cc3xxAcquire() == false)
	{
		return CRYPTO_STATUS_FAIL;	// engine busy with another operation
	}

	Cc3xxOn();

	// Reject a peer public key that is not on the P-256 curve before the DH
	// (invalid-curve attack, CVE-2018-5383). The driver ECDH does not validate
	// its input point: cc3xx_lowlevel_ecdh loads the raw coordinates straight
	// into PKA registers and multiplies. The validating allocator
	// cc3xx_lowlevel_ec_allocate_point_from_data runs the full NIST SP800-186
	// D.1 point check (coordinates in range, non zero, on curve).
	// cc3xx_lowlevel_ec_uninit releases the whole PKA state whether or not the
	// check passed, so no explicit point free is needed on either path.
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
		Cc3xxRelease();
		return CRYPTO_STATUS_FAIL;
	}

	uint32_t secret[CC3XX_P256_WORDS];
	size_t   secretSize = 0;

	r = cc3xx_lowlevel_ecdh(CC3XX_EC_CURVE_SECP_256_R1,
							privKey, CC3XX_P256_BYTES,
							peerX, CC3XX_P256_BYTES,
							peerY, CC3XX_P256_BYTES,
							secret, sizeof(secret), &secretSize);
	CRYPTO_STATUS rc = (r == CC3XX_ERR_SUCCESS && secretSize == CC3XX_P256_BYTES) ?
					   CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;

	if (rc == CRYPTO_STATUS_OK)
	{
		// DHKey is the big-endian X coordinate.
		memcpy(pDhKey, secret, CC3XX_P256_BYTES);
	}

	CryptoSecureWipe(secret, sizeof(secret));
	Cc3xxRelease();
	return rc;
}

static CRYPTO_STATUS Cc3xxEcdh(CryptoDev_t * const pDev, void *pKeyCtx,
							   const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
							   void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCc3xxData_t *pd = Cc3xxData(pDev, pKeyCtx);
	if (pd == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Fail closed when no valid private key is present: Ecdh before KeyGen, or
	// a second Ecdh after the single-use key was consumed.
	if (!pd->bKeyValid)
	{
		return CRYPTO_STATUS_FAIL;
	}

	CRYPTO_STATUS rc = Cc3xxEcdhRaw(pd->PrivKey, pPeerPubKey, pDhKey);

	// The private key is single use: consume it whether or not the DH
	// succeeded, so a stale key cannot be reused.
	Cc3xxKeyReset(pd);

	return rc;
}

// Known-answer test with the BLE Core spec P-256 vector (Vol 3 Part H
// 2.3.5.6.1): compute the DH between the spec private key and the spec peer
// public key and compare against the spec DHKey. The driver validates the peer
// point and runs the same PKA path as a live pairing.
static int Cc3xxSelfTest(CryptoDev_t * const pDev)
{
	(void)pDev;

	// Private key A, big-endian (spec sample data).
	static const uint8_t privA[32] = {
		0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
		0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd
	};
	// Peer public key B, SEC1 X||Y big-endian.
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

	// Load the spec private key into a word buffer (big-endian byte string).
	uint32_t priv[CC3XX_P256_WORDS];
	memcpy(priv, privA, sizeof(privA));

	uint8_t dh[32];
	CRYPTO_STATUS rc = Cc3xxEcdhRaw(priv, pubB, dh);

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

// Provider entry point. Referenced unconditionally by CryptoInit, so the
// linker extracts this object from the archive when this file is the selected
// hw slot. Fills only the ECDH capability; AES on this path comes from the
// BLE controller AES engine (bt_crypto_ctlr_sdc.cpp), so this engine leaves
// the AES slot null.
extern "C" bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}
	if (pCfg->pMem == nullptr || pCfg->MemSize < sizeof(CryptoCc3xxData_t))
	{
		return false;	// caller must supply per-instance state
	}
	if (((uintptr_t)pCfg->pMem & (alignof(uint32_t) - 1)) != 0)
	{
		// The key context holds uint32_t words handed to the driver, and the
		// driver secret register reads require word aligned buffers. Fail
		// closed on a misaligned arena (see CryptoCfg_t pMem in crypto.h).
		return false;
	}
	if ((pCfg->ReqCaps & ~(uint32_t)CRYPTO_CAP_ECDH_P256) != 0)
	{
		return false;	// this engine provides P-256 ECDH only
	}
	if (EnsureCc3xx() != CRYPTO_STATUS_OK)
	{
		return false;	// CC310 driver not available on this target
	}

	memset(pDev, 0, sizeof(*pDev));
	memset(pCfg->pMem, 0, sizeof(CryptoCc3xxData_t));
	CryptoCc3xxData_t *pd = (CryptoCc3xxData_t *)pCfg->pMem;
	pd->bKeyValid = false;

	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "cc3xx-hw";
	pDev->Cap            = CRYPTO_CAP_ECDH_P256;
	pDev->KeyCtxSize     = sizeof(CryptoCc3xxData_t);
	pDev->Props          = CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC;
	pDev->EvtCB          = pCfg->EvtCB;			// synchronous engine
	pDev->Aes128Ecb      = nullptr;				// AES via the BLE controller
	pDev->EcdhP256KeyGen = Cc3xxEcdhKeyGen;
	pDev->EcdhP256       = Cc3xxEcdh;
	pDev->SelfTest       = Cc3xxSelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) && Cc3xxSelfTest(pDev) != 0)
	{
		return false;
	}
	return true;
}
