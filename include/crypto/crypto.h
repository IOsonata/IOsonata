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
/// capability: it is a coredev service (coredev/rng.h) that crypto engines call.
#define CRYPTO_CAP_AES128_ECB		(1U << 0)	//!< Single-block AES-128 ECB encrypt
#define CRYPTO_CAP_ECDH_P256		(1U << 1)	//!< P-256 key generation + ECDH
// Property bit (not an operation): the engine per-instance key context is
// plain bytes that are valid when zeroed, so a Cryptor may hand it App-owned
// pMem. A structured key-context engine (mbedTLS) does NOT set this and must be
// composed with pMem NULL. High bit, clear of the operation caps a consumer queries.
#define CRYPTO_CAP_PLAIN_KEYCTX		(1U << 31)	//!< Per-instance key ctx is plain zeroable bytes
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

/// Provider selector for the optional config-driven CryptoInit. The explicit
/// provider inits (CryptoUeccInit, CryptoMbedtlsInit, CryptoHwInit) bypass this
/// and are the primary path; CryptoInit is sugar that picks one for the App.
typedef enum __Crypto_Provider {
	CRYPTO_PROVIDER_AUTO    = 0,	//!< CryptoInit picks: HW, then mbedTLS, then uECC
	CRYPTO_PROVIDER_HW      = 1,	//!< Architecture hardware engine (CryptoHwInit)
	CRYPTO_PROVIDER_MBEDTLS = 2,	//!< mbedTLS engine (CryptoMbedtlsInit)
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
/// independent. ReqCaps is validated by the engine at Init (an engine that
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
	void              *pMem;	//!< App-owned per-instance state arena (no heap)
	size_t             MemSize;	//!< Size of pMem in bytes
} CryptoCfg_t;

// Per-instance state arena sizes, for declaring the App-owned pMem buffer.
// CRYPTO_MEMSIZE_UECC is exact. CRYPTO_MEMSIZE_MBEDTLS covers the per-instance
// control structs only; mbedTLS allocates MPI limbs through its own allocator,
// so the real working set also depends on the mbedTLS heap configuration. Each
// engine Init re-checks MemSize against its true sizeof and fails closed.
#define CRYPTO_MEMSIZE_UECC		32U
#define CRYPTO_MEMSIZE_MBEDTLS	256U

/// @brief	Crypto engine interface (vtable). Canonical C form, like DevIntrf_t.
///
/// Implementer fills pDevData and the function pointers. Application/consumer
/// code treats this as an opaque handle and must not touch members directly;
/// it calls the Crypto* inline wrappers below.
struct __Crypto_Dev {
	void       *pDevData;	//!< Private engine data (software ctx, HW handle, or far-side ref for a proxy)
	const char *pName;		//!< Engine name for trace ("mbedtls", "sdc", "cc3xx", "proxy")
	uint32_t    Cap;		//!< Capability bitmask (CRYPTO_CAP_*)
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
	 * @brief	Optional known-answer self-test. 0 = PASS, nonzero = FAIL.
	 *			May be NULL.
	 */
	int (*SelfTest)(CryptoDev_t * const pDev);
};

// The Crypto* wrappers below are static inline: they have internal linkage and
// manage their own, so they are not fenced in an extern "C" block. Only the
// exported engine and Cryptor functions further down take C linkage.

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
		void *pKeyCtx, uint8_t pPubKey[64], void *pOpCtx) {
	if (pDev == NULL || pDev->EcdhP256KeyGen == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
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
	return pDev->EcdhP256(pDev, pKeyCtx, pPeerPubKey, pDhKey, pOpCtx);
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

// Engine providers fill a CryptoDev_t. Public names are provider-class, never
// target-specific: a port implements CryptoHwInit for its architecture (CRACEN,
// PKA, ESP blocks), but the public symbol stays CryptoHwInit. Target-specific
// init functions exist only inside the port implementation, file-local. Each
// Init returns true on success, false if the engine cannot be brought up or
// cannot meet pCfg->ReqCaps on this target.
bool CryptoUeccInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg);		//!< Software ECDH P-256 (micro-ecc)
bool CryptoMbedtlsInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg);	//!< Software AES+ECDH (mbedTLS); HW-accel via platform mbedTLS
bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg);		//!< Architecture hardware engine; provided by the selected port lib
bool CryptoInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg);			//!< Config-driven selector over the providers above

// Zeroize sensitive memory so the compiler cannot elide the clear. Use for key
// material and intermediate secret buffers instead of plain memset.
void CryptoSecureWipe(void *pData, size_t Len);

// Exact per-instance arena size for the mbedTLS provider on this build, for
// sizing an App-owned pMem where CRYPTO_MEMSIZE_MBEDTLS would be a guess.
// Lifecycle: CryptoMbedtlsInit is init-once per pMem. Supply fresh memory that
// does not already hold an initialized mbedTLS context; in-place reinit over a
// live context leaks its allocations (there is no Deinit path today).
size_t CryptoMbedtlsMemSize(void);

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
	virtual const char *Name() { return CryptoName(*this); }
	virtual CRYPTO_STATUS Aes128Ecb(const uint8_t Key[16], const uint8_t In[16],
									uint8_t Out[16], void *pCtx = nullptr) {
		return CryptoAes128Ecb(*this, Key, In, Out, pCtx);
	}
	virtual CRYPTO_STATUS EcdhP256KeyGen(uint8_t Pub[64], void *pKeyCtx = nullptr,
										 void *pOpCtx = nullptr) {
		return CryptoEcdhP256KeyGen(*this, pKeyCtx, Pub, pOpCtx);
	}
	virtual CRYPTO_STATUS EcdhP256(const uint8_t Peer[64], uint8_t Dh[32],
								   void *pKeyCtx = nullptr, void *pOpCtx = nullptr) {
		return CryptoEcdhP256(*this, pKeyCtx, Peer, Dh, pOpCtx);
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

/// mbedTLS engine object. Owns its per-instance control-struct arena.
class CryptoMbedtls : public CryptoDevice {
public:
	bool Init() {
		CryptoCfg_t c = {};
		c.Provider = CRYPTO_PROVIDER_MBEDTLS;
		c.ReqCaps = CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_ECDH_P256;
		c.pMem = vMem; c.MemSize = sizeof(vMem);
		return CryptoMbedtlsInit(&vDev, &c);
	}
	bool Init(const CryptoCfg_t &Cfg) {
		CryptoCfg_t c = Cfg;
		if (c.pMem == nullptr) { c.pMem = vMem; c.MemSize = sizeof(vMem); }
		return CryptoMbedtlsInit(&vDev, &c);
	}
	operator CryptoDev_t * const () { return &vDev; }
private:
	CryptoDev_t vDev {};
	uint8_t     vMem[CRYPTO_MEMSIZE_MBEDTLS] {};
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
