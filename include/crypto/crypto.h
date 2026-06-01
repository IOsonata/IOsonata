/**-------------------------------------------------------------------------
@file	crypto.h

@brief	Generic cryptographic engine interface.

A crypto engine abstraction in the same spirit as DeviceIntrf (device_intrf.h):
a thin behavioral contract that says WHAT cryptographic operation is wanted,
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
/// SMP consumer needs only AES-128 ECB, P-256 ECDH, and RNG.
#define CRYPTO_CAP_AES128_ECB		(1U << 0)	//!< Single-block AES-128 ECB encrypt
#define CRYPTO_CAP_ECDH_P256		(1U << 1)	//!< P-256 key generation + ECDH
#define CRYPTO_CAP_RNG				(1U << 2)	//!< Cryptographic random
// Reserved for future consumers (TLS, DFU): AES-GCM/CTR/CBC, SHA-256, HMAC,
// ECDSA, X.509. Append here; do not renumber existing bits.

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

/// @brief	Crypto engine interface (vtable). Canonical C form, like DevIntrf_t.
///
/// Implementer fills pDevData and the function pointers. Application/consumer
/// code treats this as an opaque handle and must not touch members directly;
/// it calls the Crypto* inline wrappers below.
struct __Crypto_Dev {
	void       *pDevData;	//!< Private engine data (software ctx, HW handle, or far-side ref for a proxy)
	const char *pName;		//!< Engine name for trace ("mbedtls", "sdc", "cc3xx", "proxy")
	uint32_t    Cap;		//!< Capability bitmask (CRYPTO_CAP_*)
	CryptoEvtHandler_t EvtCB;	//!< Completion callback for PENDING ops; NULL if engine is always synchronous

	/**
	 * @brief	AES-128 single-block ECB encrypt. Out = AES-128(Key, In).
	 *			16-byte buffers, big-endian per the SMP toolbox convention.
	 */
	CRYPTO_STATUS (*Aes128Ecb)(CryptoDev_t * const pDev,
							   const uint8_t Key[16], const uint8_t In[16],
							   uint8_t Out[16], void *pCtx);

	/**
	 * @brief	Generate a local P-256 key pair; return the 64-byte public key
	 *			(X||Y, big-endian). The private key is retained by the engine
	 *			for the matching Ecdh call (and never crosses the interface, so
	 *			a secure engine can keep it inside its domain).
	 */
	CRYPTO_STATUS (*EcdhP256KeyGen)(CryptoDev_t * const pDev, uint8_t pPubKey[64],
									void *pCtx);

	/**
	 * @brief	ECDH shared secret from the peer public key, using the private
	 *			key from the preceding EcdhP256KeyGen. Writes 32-byte DHKey
	 *			(X coordinate, big-endian).
	 */
	CRYPTO_STATUS (*EcdhP256)(CryptoDev_t * const pDev,
							  const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
							  void *pCtx);

	/**
	 * @brief	Fill a buffer with cryptographically strong random bytes.
	 *			Synchronous by contract (callers treat it as a pure source).
	 */
	void (*Rand)(CryptoDev_t * const pDev, uint8_t *pBuf, size_t Len);

	/**
	 * @brief	Optional known-answer self-test. 0 = PASS, nonzero = FAIL.
	 *			May be NULL.
	 */
	int (*SelfTest)(CryptoDev_t * const pDev);
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	True if the engine provides every capability in Mask.
 */
static inline bool CryptoIsCapable(CryptoDev_t * const pDev, uint32_t Mask) {
	return pDev != NULL && (pDev->Cap & Mask) == Mask;
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
	return pDev->Aes128Ecb(pDev, Key, In, Out, pCtx);
}

static inline CRYPTO_STATUS CryptoEcdhP256KeyGen(CryptoDev_t * const pDev,
		uint8_t pPubKey[64], void *pCtx) {
	if (pDev == NULL || pDev->EcdhP256KeyGen == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	return pDev->EcdhP256KeyGen(pDev, pPubKey, pCtx);
}

static inline CRYPTO_STATUS CryptoEcdhP256(CryptoDev_t * const pDev,
		const uint8_t pPeerPubKey[64], uint8_t pDhKey[32], void *pCtx) {
	if (pDev == NULL || pDev->EcdhP256 == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	return pDev->EcdhP256(pDev, pPeerPubKey, pDhKey, pCtx);
}

static inline void CryptoRand(CryptoDev_t * const pDev, uint8_t *pBuf, size_t Len) {
	if (pDev != NULL && pDev->Rand != NULL)
	{
		pDev->Rand(pDev, pBuf, Len);
	}
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
//   CryptoDev_t g_Ecdh;                  // App-owned instance
//   CryptoUeccInit(&g_Ecdh);             // library configures it
//   ...
//   BtSmpInit(&g_Ecdh, &g_Aes, &g_Rng);  // inject into the subsystem
//
// A given lib provides only the Init functions whose dependencies its platform
// ships (guarded on header availability). Each Init returns true on success,
// false if the engine cannot be brought up on this target.
//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

bool CryptoUeccInit(CryptoDev_t * const pDev);		//!< Software ECDH P-256 (micro-ecc)
bool CryptoMbedtlsInit(CryptoDev_t * const pDev);	//!< Software AES+ECDH+RNG (mbedTLS); HW-accel via platform mbedTLS
bool CryptoRngHwInit(CryptoDev_t * const pDev);		//!< Hardware RNG (platform RngGet)

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
}
#endif

/** @} end group Crypto */

#endif // __CRYPTO_H__
