/**-------------------------------------------------------------------------
@file	crypto.h

@brief	Generic cryptographic engine interface.

A crypto engine abstraction in the same spirit as DeviceIntrf (device_intrf.h):
a thin behavioral interface that says WHAT cryptographic operation is wanted,
never HOW or WHERE it runs. A consumer (Bluetooth SMP, TLS over LTE/Wi-Fi,
secure DFU, encrypted storage) holds a CryptoDev_t* and calls it, blind to
whether the work is done in software, in a hardware accelerator (Arm
CryptoCell CC310/CC312, STM32 PKA/CRYP), or forwarded across a TrustZone
boundary to a secure-domain engine.

Composition, exactly as SLIP-over-UART composes onto DeviceIntrf: an engine is
a CryptoDev_t; a secure-domain proxy is ALSO a CryptoDev_t that holds, in its
private pDevData, a reference to the real engine on the far side and forwards
each operation. The consumer cannot tell the difference, the same way a sensor
cannot tell SPI from I2C from SLIP. Keys that must never leave a secure domain
never cross the interface: the OPERATION crosses, not the key.

C-struct-first (CryptoDev_t) for C/C++ compatibility, like DevIntrf_t. A C++
wrapper class may be layered on top later; the struct is the canonical form
and is what the C SMP core consumes.

Async tolerance: like DeviceIntrf, an operation need not complete in-call. A
software or direct-hardware engine completes synchronously and returns
CRYPTO_STATUS_OK. An engine that offloads (controller HCI, secure service,
modem) may return CRYPTO_STATUS_PENDING and signal completion later via the
event callback, so the consumer parks rather than blocks.

Capability query: not every engine implements every primitive (uECC has no
symmetric cipher; a modem-TLS offload may not expose raw AES). A consumer asks
what an engine provides via the capability bitmask, the way a bus consumer
queries rate; an unsupported operation returns CRYPTO_STATUS_UNSUPPORTED, never
undefined behavior.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __CRYPTO_H__
#define __CRYPTO_H__

#include <stdint.h>
#include <stddef.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

/** @addtogroup Crypto
  * @{
  */

/// Operation status. OK / PENDING / FAIL mirror the SMP crypto return model;
/// UNSUPPORTED lets capability-limited engines decline an operation cleanly.
typedef enum __Crypto_Status {
	CRYPTO_STATUS_OK          = 0,	//!< Completed synchronously, output valid
	CRYPTO_STATUS_PENDING     = 1,	//!< Result will arrive via the event callback
	CRYPTO_STATUS_FAIL        = -1,	//!< Operation attempted but failed
	CRYPTO_STATUS_UNSUPPORTED = -2	//!< Engine does not provide this primitive
} CRYPTO_STATUS;

/// Capability bits. A consumer checks these (CryptoIsCapable) before relying on
/// an operation. The set grows as consumers need more primitives; today the
/// SMP consumer needs AES-128 ECB and P-256 ECDH. RNG is not a crypto
/// capability: it is a target driver (RngGet, declared below) that engines call.
#define CRYPTO_CAP_AES128_ECB		(1U << 0)	//!< Single-block AES-128 ECB encrypt
#define CRYPTO_CAP_ECDH_P256		(1U << 1)	//!< P-256 key generation + ECDH
#define CRYPTO_CAP_AES_CMAC			(1U << 3)	//!< AES-CMAC (RFC 4493); available on any AES-128 ECB engine
#define CRYPTO_CAP_AES_CCM			(1U << 4)	//!< AES-CCM AEAD (RFC 3610); available on any AES-128 ECB engine
#define CRYPTO_CAP_AES_GCM			(1U << 5)	//!< AES-GCM AEAD (SP 800-38D); available on any AES-128 ECB engine
#define CRYPTO_CAP_SHA256			(1U << 6)	//!< SHA-256 (base software core; native via the Sha256 slot)
#define CRYPTO_CAP_HMAC_SHA256		(1U << 7)	//!< HMAC-SHA-256 (RFC 2104), computed over SHA-256
#define CRYPTO_CAP_ECDSA_P256_SIGN	(1U << 8)	//!< ECDSA P-256 signature generation
#define CRYPTO_CAP_ECDSA_P256_VERIFY	(1U << 9)	//!< ECDSA P-256 signature verify (DFU image signature)
// Reserved for future consumers (TLS, DFU): AES-GCM/CTR/CBC, SHA-256, HMAC,
// ECDSA, X.509. Append here; do not renumber existing bits.

// Provider/context properties. These describe the engine, not operations, and
// are NOT part of the Cap operation mask a consumer queries. A Cryptor and the
// selector read them through pDev->Props (see CryptoHasProp).
#define CRYPTO_PROP_PLAIN_KEYCTX	(1U << 0)	//!< Per-instance key ctx is plain zeroable bytes (Cryptor may hand it pMem)
#define CRYPTO_PROP_HARDWARE		(1U << 1)	//!< Backed by a hardware accelerator
#define CRYPTO_PROP_SECURE_DOMAIN	(1U << 2)	//!< Key stays inside a secure domain / keystore
#define CRYPTO_PROP_SYNC			(1U << 3)	//!< Engine is always synchronous (no PENDING completion)

typedef struct __Crypto_Dev CryptoDev_t;

/**
 * @brief	Async completion event. Called by an engine that returned PENDING
 *			when the deferred operation finishes. Normally invoked from an
 *			interrupt or a secure-service callback; avoid blocking.
 *
 * @param	pDev	The engine that completed.
 * @param	Op		Which operation completed (one of the CRYPTO_CAP_* bits).
 * @param	Status	CRYPTO_STATUS_OK or _FAIL.
 * @param	pCtx	Consumer context passed through from the call.
 */
typedef void (*CryptoEvtHandler_t)(CryptoDev_t * const pDev, uint32_t Op,
								   CRYPTO_STATUS Status, void *pCtx);

/// Provider selector for the optional config-driven CryptoInit. The explicit
/// provider inits (CryptoUeccInit, CryptoPsaInit, CryptoHwInit) bypass this
/// and are the primary path; CryptoInit is sugar that picks one for the App.
typedef enum __Crypto_Provider {
	CRYPTO_PROVIDER_AUTO    = 0,	//!< CryptoInit picks: HW, then PSA, then uECC
	CRYPTO_PROVIDER_HW      = 1,	//!< Architecture hardware engine (CryptoHwInit)
	CRYPTO_PROVIDER_PSA     = 2,	//!< PSA Crypto engine (CryptoPsaInit)
	CRYPTO_PROVIDER_UECC    = 3,	//!< micro-ecc engine (CryptoUeccInit)
} CRYPTO_PROVIDER;

// Policy bits for CryptoCfg_t.Flags.
#define CRYPTO_FLAG_SYNC		(1U << 0)	//!< Reject an async provider for this instance
#define CRYPTO_FLAG_SELFTEST	(1U << 1)	//!< Run the engine KAT at Init; fail Init on KAT fail
#define CRYPTO_FLAG_NO_FALLBACK	(1U << 2)	//!< AUTO must not fall back to a software provider
// All current generic providers are synchronous; CRYPTO_FLAG_SYNC is not yet
// enforced by the selector and is reserved for future async hardware providers.

/// Per-instance configuration. The App fills this and passes it to an engine
/// Init. pMem/MemSize is App-owned RAM that holds the engine's per-instance
/// secret state (no heap); a separate buffer per instance keeps instances
/// independent. pMem must be at least uint32_t (4-byte) aligned: engines keep
/// word-typed key state in it and hand those words to hardware drivers, so
/// declare the arena with alignas(uint32_t). Engine Inits fail closed on a
/// misaligned arena. ReqCaps is validated by the engine at Init (an engine that
/// cannot meet it returns false); ReqCaps 0 means the provider's full native
/// set. DevNo selects a hardware block for HW providers and is ignored by the
/// software providers.
typedef struct __Crypto_Cfg {
	int                DevNo;	//!< HW block index (HW providers only; ignored by software)
	CRYPTO_PROVIDER    Provider;	//!< Requested provider (CryptoInit selector only)
	uint32_t           ReqCaps;	//!< Required capability bits (CRYPTO_CAP_*); 0 = full native set
	uint32_t           Flags;	//!< CRYPTO_FLAG_* policy bits
	int                IntPrio;	//!< Interrupt priority for an async HW engine (IRQ_PRIO_*)
	CryptoEvtHandler_t EvtCB;	//!< Completion callback for async ops; NULL for synchronous
	void              *pMem;	//!< App-owned per-instance state arena (no heap), uint32_t aligned
	size_t             MemSize;	//!< Size of pMem in bytes
} CryptoCfg_t;

// Per-instance state arena sizes, for declaring the App-owned pMem buffer.
// CRYPTO_MEMSIZE_UECC covers CryptoUeccData_t; a compile-time assert in
// crypto_uecc.cpp keeps it in sync. CRYPTO_MEMSIZE_PSA covers the per-instance
// control structs only; a PSA implementation keeps key material in its own
// keystore, so the real working set also depends on that implementation. Each
// engine Init re-checks MemSize against its true sizeof and fails closed.
#define CRYPTO_MEMSIZE_UECC		40U
#define CRYPTO_MEMSIZE_PSA		256U
#define CRYPTO_MEMSIZE_HW		1024U	// Hardware engine (CryptoHwInit) per-instance arena. Covers the
									// small PSA key-id context and the large nrf_crypto/CC310 key
									// object (about 828 bytes measured). A per-engine static_assert
									// validates the exact size at compile time.
// Arena for an ECDH engine picked at runtime by CryptoInit(AUTO): sized for
// whichever engine wins, hardware or software uECC.
#define CRYPTO_MEMSIZE_ECDH \
	((CRYPTO_MEMSIZE_HW) > (CRYPTO_MEMSIZE_PSA) ? \
	 ((CRYPTO_MEMSIZE_HW) > (CRYPTO_MEMSIZE_UECC) ? (CRYPTO_MEMSIZE_HW) : (CRYPTO_MEMSIZE_UECC)) : \
	 ((CRYPTO_MEMSIZE_PSA) > (CRYPTO_MEMSIZE_UECC) ? (CRYPTO_MEMSIZE_PSA) : (CRYPTO_MEMSIZE_UECC)))

/// @brief	Crypto engine interface (vtable). Canonical C form, like DevIntrf_t.
///
/// Implementer fills pDevData and the function pointers. Application/consumer
/// code treats this as an opaque handle and must not touch members directly;
/// it calls the Crypto* inline wrappers below.
struct __Crypto_Dev {
	void       *pDevData;	//!< Private engine data (software ctx, HW handle, or far-side ref for a proxy)
	const char *pName;		//!< Engine name for trace ("psa", "uecc", "sdc", "cc3xx", "proxy")
	uint32_t    Cap;		//!< Capability bitmask (CRYPTO_CAP_*), operations only
	uint32_t    Props;	//!< Provider/context properties (CRYPTO_PROP_*)
	size_t      KeyCtxSize;	//!< Bytes this engine needs for one per-instance key context in App pMem; 0 if the engine keeps no forwardable key context
	CryptoEvtHandler_t EvtCB;	//!< Completion callback for PENDING ops; NULL if engine is always synchronous

	/**
	 * @brief	AES-128 single-block ECB encrypt. Out = AES-128(Key, In).
	 *			16-byte buffers, big-endian per the SMP toolbox convention.
	 */
	CRYPTO_STATUS (*Aes128Ecb)(CryptoDev_t * const pDev,
							   const uint8_t Key[16], const uint8_t In[16],
							   uint8_t Out[16], void *pCtx);

	/**
	 * @brief	Optional native AES-CMAC (RFC 4493). NULL when the engine has
	 *			no native MAC; CryptoCmac then computes it over Aes128Ecb.
	 */
	CRYPTO_STATUS (*Cmac)(CryptoDev_t * const pDev, const uint8_t Key[16],
						  const uint8_t *pMsg, size_t Len, uint8_t Mac[16],
						  void *pCtx);

	/**
	 * @brief	Optional native AES-CCM AEAD (RFC 3610). bEncrypt selects the
	 *			direction; on decrypt pTag is the tag to verify and the call
	 *			fails on mismatch. NULL when the engine has no native AEAD;
	 *			CryptoCcm* then compute it over Aes128Ecb.
	 */
	CRYPTO_STATUS (*Ccm)(CryptoDev_t * const pDev, int bEncrypt,
						 const uint8_t Key[16], const uint8_t *pNonce, size_t NonceLen,
						 const uint8_t *pAad, size_t AadLen,
						 const uint8_t *pIn, size_t Len, uint8_t *pOut,
						 uint8_t *pTag, size_t TagLen, void *pCtx);

	/**
	 * @brief	Optional native AES-GCM AEAD (SP 800-38D). bEncrypt selects the
	 *			direction; on decrypt pTag is the tag to verify and the call
	 *			fails on mismatch. NULL when the engine has no native AEAD;
	 *			CryptoGcm* then compute it over Aes128Ecb.
	 */
	CRYPTO_STATUS (*Gcm)(CryptoDev_t * const pDev, int bEncrypt,
						 const uint8_t Key[16], const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pAad, size_t AadLen,
						 const uint8_t *pIn, size_t Len, uint8_t *pOut,
						 uint8_t *pTag, size_t TagLen, void *pCtx);

	/**
	 * @brief	Optional native SHA-256. NULL when the engine has no native
	 *			hash; CryptoSha256 then uses the built-in software core.
	 */
	CRYPTO_STATUS (*Sha256)(CryptoDev_t * const pDev, const uint8_t *pMsg,
							size_t Len, uint8_t Digest[32], void *pCtx);

	/**
	 * @brief	Generate a local P-256 key pair; return the 64-byte public key
	 *			(X||Y, big-endian). The private key is retained by the engine
	 *			for the matching Ecdh call (and never crosses the interface, so
	 *			a secure engine can keep it inside its domain).
	 */
	CRYPTO_STATUS (*EcdhP256KeyGen)(CryptoDev_t * const pDev, void *pKeyCtx,
									uint8_t pPubKey[64], void *pOpCtx);

	/**
	 * @brief	ECDH shared secret from the peer public key, using the private
	 *			key from the preceding EcdhP256KeyGen. Writes 32-byte DHKey
	 *			(X coordinate, big-endian).
	 */
	CRYPTO_STATUS (*EcdhP256)(CryptoDev_t * const pDev, void *pKeyCtx,
							  const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
							  void *pOpCtx);

	/**
	 * @brief	Optional ECDSA P-256 signature verify. PubKey is 64 bytes
	 *			(X||Y, big-endian), Hash is the 32-byte message hash, Sig is 64
	 *			bytes (r||s, big-endian). Returns CRYPTO_STATUS_OK when valid,
	 *			CRYPTO_STATUS_FAIL when not.
	 */
	CRYPTO_STATUS (*EcdsaP256Verify)(CryptoDev_t * const pDev,
									 const uint8_t PubKey[64], const uint8_t Hash[32],
									 const uint8_t Sig[64], void *pCtx);

	/**
	 * @brief	Optional ECDSA P-256 sign. PrivKey is the 32-byte private
	 *			scalar (big-endian), Hash is the 32-byte message hash. Writes the
	 *			64-byte signature (r||s, big-endian). Needs an RNG for the
	 *			per-signature nonce. OK on success, FAIL otherwise.
	 */
	CRYPTO_STATUS (*EcdsaP256Sign)(CryptoDev_t * const pDev,
								   const uint8_t PrivKey[32], const uint8_t Hash[32],
								   uint8_t Sig[64], void *pCtx);

	/**
	 * @brief	Optional known-answer self-test. 0 = PASS, nonzero = FAIL.
	 *			May be NULL.
	 */
	int (*SelfTest)(CryptoDev_t * const pDev);
};

// The Crypto* wrappers below are static inline: they have internal linkage and
// manage their own, so they are not fenced in an extern "C" block. Only the
// exported engine and Cryptor functions further down take C linkage.

/**
 * @brief	Capabilities callable through this handle.
 *
 * Derived operations are included when their required primitive is present.
 * SHA-256 and HMAC-SHA-256 use the built-in software implementation when the
 * handle has no native SHA function.
 */
static inline uint32_t CryptoEffectiveCaps(CryptoDev_t * const pDev) {
	if (pDev == NULL)
	{
		return 0;
	}

	uint32_t cap = CRYPTO_CAP_SHA256 | CRYPTO_CAP_HMAC_SHA256;
	if (pDev->Aes128Ecb != NULL)
	{
		cap |= CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_AES_CMAC |
			   CRYPTO_CAP_AES_CCM | CRYPTO_CAP_AES_GCM;
	}
	if (pDev->Cmac != NULL) { cap |= CRYPTO_CAP_AES_CMAC; }
	if (pDev->Ccm != NULL) { cap |= CRYPTO_CAP_AES_CCM; }
	if (pDev->Gcm != NULL) { cap |= CRYPTO_CAP_AES_GCM; }
	if (pDev->EcdhP256KeyGen != NULL && pDev->EcdhP256 != NULL)
	{
		cap |= CRYPTO_CAP_ECDH_P256;
	}
	if (pDev->EcdsaP256Sign != NULL) { cap |= CRYPTO_CAP_ECDSA_P256_SIGN; }
	if (pDev->EcdsaP256Verify != NULL) { cap |= CRYPTO_CAP_ECDSA_P256_VERIFY; }
	return cap;
}

/**
 * @brief	True if every capability in Mask is callable through the handle.
 */
static inline bool CryptoIsCapable(CryptoDev_t * const pDev, uint32_t Mask) {
	return pDev != NULL && (CryptoEffectiveCaps(pDev) & Mask) == Mask;
}

/**
 * @brief	True if the engine sets every property in Mask (CRYPTO_PROP_*).
 */
static inline bool CryptoHasProp(CryptoDev_t * const pDev, uint32_t Mask) {
	return pDev != NULL && (pDev->Props & Mask) == Mask;
}

static inline const char * CryptoName(CryptoDev_t * const pDev) {
	return (pDev != NULL && pDev->pName != NULL) ? pDev->pName : "null";
}

static inline CRYPTO_STATUS CryptoAes128Ecb(CryptoDev_t * const pDev,
		const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16], void *pCtx) {
	if (pDev == NULL || pDev->Aes128Ecb == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (Key == NULL || In == NULL || Out == NULL)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return pDev->Aes128Ecb(pDev, Key, In, Out, pCtx);
}

static inline CRYPTO_STATUS CryptoEcdhP256KeyGen(CryptoDev_t * const pDev,
		void *pKeyCtx, uint8_t pPubKey[64], void *pOpCtx) {
	if (pDev == NULL || pDev->EcdhP256KeyGen == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (pPubKey == NULL)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return pDev->EcdhP256KeyGen(pDev, pKeyCtx, pPubKey, pOpCtx);
}

static inline CRYPTO_STATUS CryptoEcdhP256(CryptoDev_t * const pDev,
		void *pKeyCtx, const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
		void *pOpCtx) {
	if (pDev == NULL || pDev->EcdhP256 == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (pPeerPubKey == NULL || pDhKey == NULL)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return pDev->EcdhP256(pDev, pKeyCtx, pPeerPubKey, pDhKey, pOpCtx);
}

/**
 * @brief	ECDSA P-256 verify. OK when the signature is valid, FAIL when
 *			not, UNSUPPORTED when the engine has no verify.
 */
static inline CRYPTO_STATUS CryptoEcdsaP256Verify(CryptoDev_t * const pDev,
		const uint8_t PubKey[64], const uint8_t Hash[32], const uint8_t Sig[64],
		void *pCtx) {
	if (pDev == NULL || pDev->EcdsaP256Verify == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (PubKey == NULL || Hash == NULL || Sig == NULL)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return pDev->EcdsaP256Verify(pDev, PubKey, Hash, Sig, pCtx);
}

/**
 * @brief	ECDSA P-256 sign. OK on success, FAIL on error, UNSUPPORTED when
 *			the engine has no sign.
 */
static inline CRYPTO_STATUS CryptoEcdsaP256Sign(CryptoDev_t * const pDev,
		const uint8_t PrivKey[32], const uint8_t Hash[32], uint8_t Sig[64],
		void *pCtx) {
	if (pDev == NULL || pDev->EcdsaP256Sign == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (PrivKey == NULL || Hash == NULL || Sig == NULL)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return pDev->EcdsaP256Sign(pDev, PrivKey, Hash, Sig, pCtx);
}

static inline int CryptoSelfTest(CryptoDev_t * const pDev) {
	if (pDev == NULL || pDev->SelfTest == NULL)
	{
		return 0;	// no self-test available is not a failure
	}
	return pDev->SelfTest(pDev);
}

//-----------------------------------------------------------------------------
// Engine init. The library provides the implementation; the APPLICATION owns
// the CryptoDev_t instance and calls the engine's Init to populate it, then
// passes its pointer into a subsystem (e.g. BtSmpInit) - exactly as the App
// owns an SPI/I2C object, calls Init(cfg), and injects it into a sensor.
//
//   CryptoDev_t g_BleEcdh;                       // App-owned instance
//   static uint8_t g_BleMem[CRYPTO_MEMSIZE_UECC];
//   CryptoCfg_t cfg = { 0 };
//   cfg.Provider = CRYPTO_PROVIDER_UECC;
//   cfg.ReqCaps  = CRYPTO_CAP_ECDH_P256;
//   cfg.pMem     = g_BleMem; cfg.MemSize = sizeof(g_BleMem);
//   CryptoUeccInit(&g_BleEcdh, &cfg);             // library configures it
//   BtSmpInit(&g_BleEcdh, &g_BleAes);             // inject into the subsystem
//
// A given lib provides only the Init functions whose dependencies its platform
// ships (guarded on header availability). Each Init returns true on success,
// false if the engine cannot be brought up on this target.
//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
// Cryptographic random bit generator.
//
// Source of random bits for key material: SMP pairing nonces, IRK, CSRK,
// passkeys, ECDH scalars and the seeding of the crypto engines. RngGet returns
// bits that an adversary cannot predict or reproduce. NIST calls this a random
// bit generator (RBG): either an entropy source feeding a deterministic
// generator (SP 800-90A/B/C), or a hardware block providing both. The Bluetooth
// core specification (Vol 2, Part H, section 2) requires a generator compliant
// with FIPS PUB 140-2 or a later update, and the Security Manager (Vol 3,
// Part H) refers back to it.
//
// The implementation is a per-MCU driver over the RNG peripheral, for example
// rng_nrfx.cpp on Nordic (legacy RNG block, or CRACEN on nRF54) and rng_stm32.c
// on ST. There is no generic software default, on purpose. Software alone
// cannot produce an RBG: a deterministic generator stretches entropy, it never
// creates it, and entropy comes from physical noise that only the target has. A
// software fallback would compile on any part and hand every device the same
// key. A target without an RNG peripheral must supply its own driver, or the
// link fails and names the missing symbol.
//
// Statistical randomness (test patterns, dithering, back-off delays) is a
// different service with opposite requirements. Use the C library directly:
// rand_r with a caller owned seed. Never use RngGet for it, and never use the C
// library for key material.
//-----------------------------------------------------------------------------

/**
 * @brief	Initialise the RNG peripheral.
 *
 * The drivers seed on first use, so an explicit call is not required.
 *
 * @return	true on success.
 */
bool RngInit(void);

/**
 * @brief	Fill a buffer with cryptographically strong random bytes.
 *
 * Suitable for key material. A caller on a security path must check the result
 * and abort on failure rather than proceed with the buffer contents.
 *
 * @param	pBuff	Destination buffer.
 * @param	Len		Number of bytes.
 *
 * @return	true on success.
 */
bool RngGet(uint8_t *pBuff, size_t Len);

// Engine providers fill a CryptoDev_t. Public names are provider-class, never
// target-specific: a port implements CryptoHwInit for its architecture (CRACEN,
// PKA, ESP blocks), but the public symbol stays CryptoHwInit. Target-specific
// init functions exist only inside the port implementation, file-local. Each
// Init returns true on success, false if the engine cannot be brought up or
// cannot meet pCfg->ReqCaps on this target.
bool CryptoUeccInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg);		//!< Software ECDH P-256 (micro-ecc)
bool CryptoPsaInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg);	//!< AES+ECDH over the PSA Crypto API (TF-PSA-Crypto, or a platform PSA driver)
bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg);		//!< Architecture hardware engine; provided by the selected port lib
bool CryptoInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg);			//!< Config-driven selector over the providers above

// Zeroize sensitive memory so the compiler cannot elide the clear. Use for key
// material and intermediate secret buffers instead of plain memset.
void CryptoSecureWipe(void *pData, size_t Len);

// Resolve the per-instance key context an engine operation should use: the
// Cryptor-supplied context when non-null, else the engine's own pDevData. The
// resolved pointer is validated non-null and aligned to Align (the key context
// struct alignment), so an engine fails closed on a bad or misaligned context
// instead of faulting. Returns nullptr on failure. Align must be a power of two.
void *CryptoResolveKeyCtx(CryptoDev_t * const pDev, void *pKeyCtx, size_t Align);

// Validate an engine Init configuration: non-null pDev and pCfg, a per-instance
// arena of at least CtxSize bytes aligned to at least uint32_t, and a requested
// capability set within CapMask (the engine's supported CRYPTO_CAP_* bits).
// Returns true when the configuration is usable. Every Crypto*Init runs this
// first so the boundary checks live in one place.
bool CryptoCfgValidate(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg,
					   size_t CtxSize, uint32_t CapMask);

/**
 * @brief	AES-CMAC (RFC 4493) with a 16-byte key over Len bytes; writes
 *			the 16-byte MAC. Uses the engine native Cmac if present, else
 *			computes it over the engine Aes128Ecb. UNSUPPORTED if the engine
 *			has neither.
 */
CRYPTO_STATUS CryptoCmac(CryptoDev_t * const pDev, const uint8_t Key[16],
						 const uint8_t *pMsg, size_t Len, uint8_t Mac[16],
						 void *pCtx);

/**
 * @brief	AES-CCM AEAD encrypt (RFC 3610). NonceLen 7..13, TagLen even
 *			4..16. Writes Len ciphertext bytes to pCipher and TagLen tag bytes
 *			to pTag. Uses a native Ccm if present, else computes it over
 *			Aes128Ecb. AAD length must be below 0xFF00.
 */
CRYPTO_STATUS CryptoCcmEncrypt(CryptoDev_t * const pDev, const uint8_t Key[16],
							   const uint8_t *pNonce, size_t NonceLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pPlain, size_t Len, uint8_t *pCipher,
							   uint8_t *pTag, size_t TagLen, void *pCtx);

/**
 * @brief	AES-CCM AEAD decrypt and verify (RFC 3610). Writes Len plaintext
 *			bytes to pPlain only if the tag verifies; on mismatch it wipes
 *			pPlain and returns CRYPTO_STATUS_FAIL.
 */
CRYPTO_STATUS CryptoCcmDecrypt(CryptoDev_t * const pDev, const uint8_t Key[16],
							   const uint8_t *pNonce, size_t NonceLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pCipher, size_t Len, uint8_t *pPlain,
							   const uint8_t *pTag, size_t TagLen, void *pCtx);

/**
 * @brief	SHA-256 (FIPS 180-4) over Len bytes; writes the 32-byte digest.
 *			Uses the engine native Sha256 if present, else a built-in software
 *			core. Always available; pDev may be NULL for the software path.
 */
/**
 * @brief	AES-GCM AEAD encrypt (SP 800-38D). IvLen any (12 is the fast
 *			path), TagLen 4..16. Writes Len ciphertext bytes and TagLen tag
 *			bytes. Uses a native Gcm if present, else over Aes128Ecb.
 */
CRYPTO_STATUS CryptoGcmEncrypt(CryptoDev_t * const pDev, const uint8_t Key[16],
							   const uint8_t *pIv, size_t IvLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pPlain, size_t Len, uint8_t *pCipher,
							   uint8_t *pTag, size_t TagLen, void *pCtx);

/**
 * @brief	AES-GCM AEAD decrypt and verify (SP 800-38D). Writes plaintext
 *			only if the tag verifies; on mismatch it wipes pPlain and returns
 *			CRYPTO_STATUS_FAIL.
 */
CRYPTO_STATUS CryptoGcmDecrypt(CryptoDev_t * const pDev, const uint8_t Key[16],
							   const uint8_t *pIv, size_t IvLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pCipher, size_t Len, uint8_t *pPlain,
							   const uint8_t *pTag, size_t TagLen, void *pCtx);

CRYPTO_STATUS CryptoSha256(CryptoDev_t * const pDev, const uint8_t *pMsg,
						   size_t Len, uint8_t Digest[32], void *pCtx);

/**
 * @brief	HMAC-SHA-256 (RFC 2104) with a KeyLen-byte key; writes the
 *			32-byte MAC. Computed over the SHA-256 core.
 */
CRYPTO_STATUS CryptoHmacSha256(CryptoDev_t * const pDev,
							   const uint8_t *pKey, size_t KeyLen,
							   const uint8_t *pMsg, size_t Len, uint8_t Mac[32],
							   void *pCtx);

// Exact per-instance arena size for the PSA engine on this build, for
// sizing an App-owned pMem where CRYPTO_MEMSIZE_PSA would be a guess.
// Lifecycle: CryptoPsaInit is init-once per pMem. Supply fresh memory that
// does not already hold an initialized PSA context; in-place reinit over a
// live context leaks its allocations (there is no Deinit path today).
size_t CryptoPsaMemSize(void);

//-----------------------------------------------------------------------------
// Cryptor: the per-use-case instance, like a motion fusion object over sensors.
// A Cryptor references one or more engines (CryptoDev_t) and presents its own
// CryptoDev_t handle (CryptorHandle) that forwards each operation to the engine
// providing that capability, using the Cryptor's own per-instance key state in
// pMem. Build a Cryptor only when several use cases must share one engine, or
// one use case must compose several engines; for a dedicated engine, pass the
// engine handle straight to the subsystem. Forwarding lives in crypto.cpp.
//-----------------------------------------------------------------------------
#define CRYPTOR_MAX_ENGINE	4	//!< Capability-routing engine slots per Cryptor

typedef struct __Cryptor {
	CryptoDev_t  Dev;						//!< Forwarding handle this instance presents
	CryptoDev_t *pEng[CRYPTOR_MAX_ENGINE];	//!< Referenced engine(s); routed by capability
	int          NbEng;						//!< Number of referenced engines
	uint32_t     ReqCaps;					//!< Capabilities this instance requires
	void        *pMem;						//!< Per-instance key state arena (App-owned)
	size_t       MemSize;					//!< Size of pMem in bytes
} Cryptor_t;

bool CryptorInit(Cryptor_t * const pInst, const CryptoCfg_t *pCfg,
				 CryptoDev_t * const pEng);
bool CryptorComposeInit(Cryptor_t * const pInst, const CryptoCfg_t *pCfg,
						CryptoDev_t * const pEng[], int NbEng);
CryptoDev_t * CryptorHandle(Cryptor_t * const pInst);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/// C++ facade over the CryptoDev_t handle, like DeviceIntrf over DevIntrf_t.
/// Engines and Cryptor instances both present a CryptoDev_t; this wraps it so
/// C++ firmware can call methods and pass the object where a CryptoDev_t* is
/// wanted. The subsystem cannot tell an engine from a forwarding instance.
class CryptoDevice {
public:
	virtual operator CryptoDev_t * const () = 0;	// concrete returns its embedded handle
	virtual uint32_t Capability() { return ((CryptoDev_t*)*this)->Cap; }
	virtual bool IsCapable(uint32_t Mask) { return CryptoIsCapable(*this, Mask); }
	virtual bool HasProp(uint32_t Mask) { return CryptoHasProp(*this, Mask); }
	virtual const char *Name() { return CryptoName(*this); }
	virtual CRYPTO_STATUS Aes128Ecb(const uint8_t Key[16], const uint8_t In[16],
									uint8_t Out[16], void *pCtx = nullptr) {
		return CryptoAes128Ecb(*this, Key, In, Out, pCtx);
	}
	virtual CRYPTO_STATUS Cmac(const uint8_t Key[16], const uint8_t *pMsg,
							   size_t Len, uint8_t Mac[16], void *pCtx = nullptr) {
		return CryptoCmac(*this, Key, pMsg, Len, Mac, pCtx);
	}
	virtual CRYPTO_STATUS CcmEncrypt(const uint8_t Key[16], const uint8_t *pNonce,
			size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pPlain, size_t Len, uint8_t *pCipher,
			uint8_t *pTag, size_t TagLen, void *pCtx = nullptr) {
		return CryptoCcmEncrypt(*this, Key, pNonce, NonceLen, pAad, AadLen, pPlain, Len, pCipher, pTag, TagLen, pCtx);
	}
	virtual CRYPTO_STATUS CcmDecrypt(const uint8_t Key[16], const uint8_t *pNonce,
			size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pCipher, size_t Len, uint8_t *pPlain,
			const uint8_t *pTag, size_t TagLen, void *pCtx = nullptr) {
		return CryptoCcmDecrypt(*this, Key, pNonce, NonceLen, pAad, AadLen, pCipher, Len, pPlain, pTag, TagLen, pCtx);
	}
	virtual CRYPTO_STATUS GcmEncrypt(const uint8_t Key[16], const uint8_t *pIv,
			size_t IvLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pPlain, size_t Len, uint8_t *pCipher,
			uint8_t *pTag, size_t TagLen, void *pCtx = nullptr) {
		return CryptoGcmEncrypt(*this, Key, pIv, IvLen, pAad, AadLen, pPlain, Len, pCipher, pTag, TagLen, pCtx);
	}
	virtual CRYPTO_STATUS GcmDecrypt(const uint8_t Key[16], const uint8_t *pIv,
			size_t IvLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pCipher, size_t Len, uint8_t *pPlain,
			const uint8_t *pTag, size_t TagLen, void *pCtx = nullptr) {
		return CryptoGcmDecrypt(*this, Key, pIv, IvLen, pAad, AadLen, pCipher, Len, pPlain, pTag, TagLen, pCtx);
	}
	virtual CRYPTO_STATUS Sha256(const uint8_t *pMsg, size_t Len,
								 uint8_t Digest[32], void *pCtx = nullptr) {
		return CryptoSha256(*this, pMsg, Len, Digest, pCtx);
	}
	virtual CRYPTO_STATUS HmacSha256(const uint8_t *pKey, size_t KeyLen,
			const uint8_t *pMsg, size_t Len, uint8_t Mac[32], void *pCtx = nullptr) {
		return CryptoHmacSha256(*this, pKey, KeyLen, pMsg, Len, Mac, pCtx);
	}
	virtual CRYPTO_STATUS EcdhP256KeyGen(uint8_t Pub[64], void *pKeyCtx = nullptr,
										 void *pOpCtx = nullptr) {
		return CryptoEcdhP256KeyGen(*this, pKeyCtx, Pub, pOpCtx);
	}
	virtual CRYPTO_STATUS EcdhP256(const uint8_t Peer[64], uint8_t Dh[32],
								   void *pKeyCtx = nullptr, void *pOpCtx = nullptr) {
		return CryptoEcdhP256(*this, pKeyCtx, Peer, Dh, pOpCtx);
	}
	virtual CRYPTO_STATUS EcdsaP256Verify(const uint8_t PubKey[64],
			const uint8_t Hash[32], const uint8_t Sig[64], void *pCtx = nullptr) {
		return CryptoEcdsaP256Verify(*this, PubKey, Hash, Sig, pCtx);
	}
	virtual CRYPTO_STATUS EcdsaP256Sign(const uint8_t PrivKey[32],
			const uint8_t Hash[32], uint8_t Sig[64], void *pCtx = nullptr) {
		return CryptoEcdsaP256Sign(*this, PrivKey, Hash, Sig, pCtx);
	}
	virtual int SelfTest() { return CryptoSelfTest(*this); }
	virtual ~CryptoDevice() {}
};

/// micro-ecc engine object. Owns its per-instance key arena, so multiple
/// CryptoUecc objects are independent without the App sizing pMem by hand.
class CryptoUecc : public CryptoDevice {
public:
	bool Init() {
		CryptoCfg_t c = {};
		c.Provider = CRYPTO_PROVIDER_UECC; c.ReqCaps = CRYPTO_CAP_ECDH_P256;
		c.pMem = vMem; c.MemSize = sizeof(vMem);
		return CryptoUeccInit(&vDev, &c);
	}
	bool Init(const CryptoCfg_t &Cfg) {
		CryptoCfg_t c = Cfg;
		if (c.pMem == nullptr) { c.pMem = vMem; c.MemSize = sizeof(vMem); }
		return CryptoUeccInit(&vDev, &c);
	}
	operator CryptoDev_t * const () { return &vDev; }
private:
	CryptoDev_t vDev {};
	uint8_t     vMem[CRYPTO_MEMSIZE_UECC] {};
};

/// PSA Crypto engine object. Owns its per-instance control-struct arena.
class CryptoPsa : public CryptoDevice {
public:
	bool Init() {
		CryptoCfg_t c = {};
		c.Provider = CRYPTO_PROVIDER_PSA;
		c.ReqCaps = CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_ECDH_P256;
		c.pMem = vMem; c.MemSize = sizeof(vMem);
		return CryptoPsaInit(&vDev, &c);
	}
	bool Init(const CryptoCfg_t &Cfg) {
		CryptoCfg_t c = Cfg;
		if (c.pMem == nullptr) { c.pMem = vMem; c.MemSize = sizeof(vMem); }
		return CryptoPsaInit(&vDev, &c);
	}
	operator CryptoDev_t * const () { return &vDev; }
private:
	CryptoDev_t vDev {};
	uint8_t     vMem[CRYPTO_MEMSIZE_PSA] {};
};

/// Use-case instance: references engine(s) and forwards. Pass it where a
/// CryptoDev_t* is wanted; the conversion returns the forwarding handle.
class Cryptor {
public:
	bool Init(const CryptoCfg_t &Cfg, CryptoDevice &Eng) {
		return CryptorInit(&vInst, &Cfg, Eng);
	}
	bool Init(const CryptoCfg_t &Cfg, CryptoDev_t * const Eng[], int NbEng) {
		return CryptorComposeInit(&vInst, &Cfg, Eng, NbEng);
	}
	operator CryptoDev_t * const () { return CryptorHandle(&vInst); }
private:
	Cryptor_t vInst {};
};

#endif // __cplusplus

/** @} end group Crypto */

#endif // __CRYPTO_H__
