/**-------------------------------------------------------------------------
@file	icrypto.h

@brief	Object-oriented crypto engine tree, key object, and result status.

		A crypto engine is modelled on the IOsonata Device base: a controllable
		entity with a lifecycle and capability facets, not a byte transport.
		Each capability facet (cipher, mac, hash, key agreement, signature,
		key derivation, rng) is a base class with a working software body. A
		hardware driver inherits the facets it accelerates and overrides only
		those operations; anything it does not override runs the software base.
		The base tree is, by itself, a complete software crypto library.

		A real block multiply-inherits the facets it implements, each facet
		virtual public CryptoEngine so the shared base subobject is single.
		That single subobject is what makes an inherited software algorithm (for
		example CMAC in MacEngine) call the hardware override of another facet
		(Cipher) on a combined block, through ordinary virtual dispatch. This is
		the Sensor : virtual public Device diamond discipline.

		No RTTI, no dynamic allocation. Facet presence is a compile-time base,
		not a runtime query. Per-instance and streaming-operation state is
		caller-provided static memory.

		A key is data with attributes, not a Device: it carries the key type,
		where the material lives (plain bytes, an accelerator slot, or an opaque
		secure handle), and what it may be used for. Operations take a CryptoKey
		by reference, never a raw pointer, so a plaintext software key, an
		accelerator-slot key, and a never-exportable secure-element key all cross
		the same interface: the operation crosses, not the key.

		The file name is i-prefixed by IOsonata convention (isha256.h, iatomic.h,
		istddef.h) so it does not collide with a bare crypto.h from a toolchain
		or vendor SDK. This header stands alone: it defines its own result status
		and depends only on device.h, not on any earlier crypto layer.

		This header defines the base and the facets BLE Secure Connections needs
		(cipher, mac, key agreement, rng) on the full tree, plus the shared P-256
		scalar and field helpers every ECDH path uses. The remaining facets
		(aead, hash, signature, kdf) follow the same shape and are added as
		LoRaWAN, Wi-Fi and storage consumers land.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __ICRYPTO_H__
#define __ICRYPTO_H__

#include <stdint.h>
#include <stddef.h>

#include "device.h"

/** @addtogroup Crypto
  * @{
  */

//-----------------------------------------------------------------------------
// Result status.
//
// Shared primitive of the crypto layer. OK / PENDING / FAIL mirror the SMP
// crypto return model; UNSUPPORTED lets a capability-limited engine decline an
// operation cleanly rather than fault. An engine that offloads (controller HCI,
// secure service, modem) may return PENDING and signal completion through the
// Device event callback.
//-----------------------------------------------------------------------------
typedef enum __Crypto_Status {
	CRYPTO_STATUS_OK          = 0,	//!< Completed synchronously, output valid
	CRYPTO_STATUS_PENDING     = 1,	//!< Result will arrive via the event callback
	CRYPTO_STATUS_FAIL        = -1,	//!< Operation attempted but failed
	CRYPTO_STATUS_UNSUPPORTED = -2	//!< Engine does not provide this primitive
} CRYPTO_STATUS;

//-----------------------------------------------------------------------------
// Key object.
//-----------------------------------------------------------------------------

/// Key type. Selects the algorithm family and, for symmetric keys, the size.
typedef enum __Crypto_Key_Type {
	CRYPTO_KEY_AES_128,			//!< 16-byte AES key
	CRYPTO_KEY_AES_256,			//!< 32-byte AES key
	CRYPTO_KEY_ECC_P256,		//!< P-256 private scalar or public point
	CRYPTO_KEY_ECC_P384,		//!< P-384 private scalar or public point
	CRYPTO_KEY_HMAC,			//!< Arbitrary-length MAC key
	CRYPTO_KEY_DERIVE			//!< Input keying material for a KDF
} CRYPTO_KEY_TYPE;

/// Where the key material lives. This axis is what lets one interface serve a
/// software engine and a secure element without the consumer knowing which.
/// Only the plain location needs an implementation today; slot and opaque are
/// defined now so adding a secure element later needs a new engine that accepts
/// those locations, not a change to any operation signature.
typedef enum __Crypto_Key_Location {
	CRYPTO_KEY_LOC_PLAIN,		//!< Bytes in application RAM (software or HW accel)
	CRYPTO_KEY_LOC_SLOT,		//!< Loaded into an accelerator key slot by index
	CRYPTO_KEY_LOC_OPAQUE		//!< Never leaves a secure domain; referenced by id
} CRYPTO_KEY_LOCATION;

/// Usage policy bitmask. The engine enforces it; the caller does not.
#define CRYPTO_KEY_USE_ENCRYPT		(1U << 0)
#define CRYPTO_KEY_USE_DECRYPT		(1U << 1)
#define CRYPTO_KEY_USE_SIGN			(1U << 2)
#define CRYPTO_KEY_USE_VERIFY		(1U << 3)
#define CRYPTO_KEY_USE_DERIVE		(1U << 4)
#define CRYPTO_KEY_USE_AGREE		(1U << 5)

/// @brief	Crypto key descriptor.
///
/// A plain key holds a pointer and length into caller memory. A slot key holds
/// an accelerator slot index. An opaque key holds a secure-domain identifier.
/// The union keeps the object small and allocation-free.
typedef struct __Crypto_Key {
	CRYPTO_KEY_TYPE     Type;		//!< Key type
	CRYPTO_KEY_LOCATION Loc;		//!< Where the material lives
	uint32_t            Usage;		//!< CRYPTO_KEY_USE_* bitmask
	union {
		struct {
			const uint8_t *pData;	//!< Key bytes (caller-owned)
			size_t         Len;		//!< Key length in bytes
		} Plain;					//!< CRYPTO_KEY_LOC_PLAIN
		uint32_t SlotIndex;			//!< CRYPTO_KEY_LOC_SLOT
		uint32_t OpaqueId;			//!< CRYPTO_KEY_LOC_OPAQUE
	};
} CryptoKey;

//-----------------------------------------------------------------------------
// Algorithm identifiers, passed to the facet operations.
//-----------------------------------------------------------------------------

/// Cipher mode (CipherEngine).
typedef enum __Crypto_Cipher_Alg {
	CRYPTO_CIPHER_ECB,			//!< AES-ECB (single or multi block)
	CRYPTO_CIPHER_CTR,			//!< AES-CTR
	CRYPTO_CIPHER_CBC,			//!< AES-CBC
	CRYPTO_CIPHER_XTS			//!< AES-XTS (storage)
} CRYPTO_CIPHER_ALG;

/// MAC algorithm (MacEngine).
typedef enum __Crypto_Mac_Alg {
	CRYPTO_MAC_CMAC,			//!< AES-CMAC (RFC 4493)
	CRYPTO_MAC_HMAC,			//!< HMAC (RFC 2104), hash selected by key/config
	CRYPTO_MAC_GMAC				//!< GMAC (GHASH-based)
} CRYPTO_MAC_ALG;

/// Elliptic curve (KeyAgreeEngine, SignEngine).
typedef enum __Crypto_Curve {
	CRYPTO_CURVE_P256,			//!< NIST P-256 (secp256r1)
	CRYPTO_CURVE_P384			//!< NIST P-384 (secp384r1)
} CRYPTO_CURVE;

//-----------------------------------------------------------------------------
// Shared P-256 scalar and field helpers.
//
// Byte-level hygiene shared by every P-256 path: the software point-multiply
// base (micro-ecc), the hardware overrides (Silex BA414EP, Arm CryptoCell), and
// their countermeasures. These are facts about the curve order and field, not
// point arithmetic: no double, add, or scalar multiply, and no register
// dependency. Point arithmetic lives in each KeyAgreeEngine implementation,
// software or hardware. All values are big-endian 32-byte strings matching the
// SEC1 uncompressed public-key encoding. C linkage so a C engine can call them.
//-----------------------------------------------------------------------------
#define P256_BYTES		32U		//!< P-256 field element / scalar size in bytes

#ifdef __cplusplus
extern "C" {
#endif

/// True when every byte of the buffer is zero, timing-independent of position.
bool P256IsZero(const uint8_t *pData, size_t Len);

/// True when big-endian A is strictly less than big-endian B over Len bytes.
bool P256LessBe(const uint8_t *pA, const uint8_t *pB, size_t Len);

/// Draw a uniform private scalar in [1, n-1] via the RNG with rejection.
bool P256RandomScalar(uint8_t Scalar[P256_BYTES]);

/// True when Scalar is a valid private scalar: non-zero and less than the order.
bool P256ScalarInRange(const uint8_t Scalar[P256_BYTES]);

/// True when the coordinate is a non-zero field element (below the field prime).
bool P256NonzeroFieldElement(const uint8_t Coord[P256_BYTES]);

/// Regularize a scalar to a fixed bit length for a constant-time ladder.
void P256RegularizeScalar(const uint8_t K[P256_BYTES], uint8_t R[P256_BYTES + 1U]);

/// Platform random source: fill pBuff with Len cryptographically strong bytes.
/// Provided by the target random driver (Nordic rng_nrfx, ST rng_stm32). A
/// security-path caller must check the result and abort on failure. This is the
/// same free function the RngEngine facet wraps; it is kept here so portable
/// crypto and SMP code can draw randomness without a platform header.
bool RngGet(uint8_t *pBuff, size_t Len);

/// Zeroize a buffer so the write is not optimized away. Use to clear key
/// material, shared secrets, and other secrets on every exit path.
void CryptoSecureWipe(void *pData, size_t Len);

#ifdef __cplusplus
}
#endif

//-----------------------------------------------------------------------------
// CryptoEngine: common base for every engine.
//
// Inherits Device for the lifecycle (Enable/Disable/Reset/PowerOff), the event
// callback, and the held interface used by an offload or secure-domain proxy.
// A software engine implements Enable/Disable/Reset trivially; a hardware
// engine powers its block.
//-----------------------------------------------------------------------------
/// @brief	Which operation a completion callback is reporting.
///
/// An engine that returns CRYPTO_STATUS_PENDING from a facet op signals its
/// completion later through the CryptoEngine completion handler, tagging the
/// event with the operation that finished so a consumer driving several
/// operations (for example SMP running key generation then key agreement) can
/// tell them apart.
typedef enum __Crypto_Op {
	CRYPTO_OP_NONE = 0,			//!< No operation
	CRYPTO_OP_CIPHER,			//!< CipherEngine::Cipher completed
	CRYPTO_OP_MAC,				//!< MacEngine::Mac completed
	CRYPTO_OP_KEYGEN,			//!< KeyAgreeEngine::KeyGen completed
	CRYPTO_OP_AGREE,			//!< KeyAgreeEngine::Agree completed
	CRYPTO_OP_SIGN,				//!< SignEngine::Sign completed
	CRYPTO_OP_VERIFY,			//!< SignEngine::Verify completed
	CRYPTO_OP_RANDOM			//!< RngEngine::Random completed
} CRYPTO_OP;

class CryptoEngine;

/// @brief	Completion callback for an asynchronous crypto operation.
///
/// Called by the engine when an operation that returned CRYPTO_STATUS_PENDING
/// finishes (from the engine's interrupt or worker context). Op identifies the
/// finished operation, Status is CRYPTO_STATUS_OK or a failure, and pCtx is the
/// caller context handed to the facet call. The output buffer the facet call
/// named holds the result on OK.
typedef void (*CryptoCompleteHandler_t)(CryptoEngine * const pEngine,
										CRYPTO_OP Op, CRYPTO_STATUS Status,
										void *pCtx);

class CryptoEngine : virtual public Device {
public:
	virtual ~CryptoEngine() {}

	/**
	 * @brief	Optional known-answer self-test. 0 = pass, nonzero = fail.
	 *			The base reports pass; an engine with test vectors overrides.
	 */
	virtual int SelfTest() { return 0; }

	/**
	 * @brief	Bind the completion handler for asynchronous operations.
	 *
	 * A synchronous engine ignores this: its facet ops return OK or FAIL
	 * directly and never call the handler. An asynchronous engine (a hardware
	 * block driven under interrupt, or an off-die secure element) returns
	 * CRYPTO_STATUS_PENDING and calls this handler on completion. Binding is
	 * optional; an engine with no handler runs an operation to completion
	 * before returning even if it could have deferred.
	 */
	void SetCompleteHandler(CryptoCompleteHandler_t Handler, void *pCtx)
	{
		vCompleteHandler = Handler;
		vpCompleteCtx = pCtx;
	}

	/**
	 * @brief	True if this engine may return CRYPTO_STATUS_PENDING. A
	 *			synchronous engine returns false; a consumer that requires a
	 *			synchronous result (no event loop) checks this before use.
	 */
	virtual bool IsAsync() const { return false; }

	// Enable(), Disable(), Reset(), PowerOff(), EvtHandler(), Interface() are
	// inherited from Device.

protected:
	/// Engines call this to report an async completion to the bound handler.
	/// A synchronous engine never calls it. Safe to call with no handler bound.
	void Complete(CRYPTO_OP Op, CRYPTO_STATUS Status)
	{
		if (vCompleteHandler != nullptr)
		{
			vCompleteHandler(this, Op, Status, vpCompleteCtx);
		}
	}

	CryptoCompleteHandler_t vCompleteHandler = nullptr;	//!< Async completion callback
	void                   *vpCompleteCtx    = nullptr;	//!< Caller context for the callback
};

//-----------------------------------------------------------------------------
// CipherEngine: block cipher facet. Base is software AES; a hardware driver
// overrides Cipher with its accelerator.
//-----------------------------------------------------------------------------
class CipherEngine : virtual public CryptoEngine {
public:
	/**
	 * @brief	One-shot cipher. Encrypts or decrypts Len bytes under Key.
	 *
	 * @param	Alg		Cipher mode (CRYPTO_CIPHER_*).
	 * @param	bEncrypt Nonzero to encrypt, zero to decrypt.
	 * @param	Key		Key descriptor; must permit the direction.
	 * @param	pIv		Initial value / counter, or NULL for ECB.
	 * @param	IvLen	Length of pIv in bytes (0 for ECB).
	 * @param	pIn		Input buffer of Len bytes.
	 * @param	Len		Byte length; a multiple of 16 for block modes.
	 * @param	pOut	Output buffer of Len bytes; may equal pIn.
	 *
	 * @return	CRYPTO_STATUS_OK on success. Base runs software AES.
	 */
	virtual CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
								 const CryptoKey &Key,
								 const uint8_t *pIv, size_t IvLen,
								 const uint8_t *pIn, size_t Len, uint8_t *pOut) {
		(void)Alg; (void)bEncrypt; (void)Key; (void)pIv; (void)IvLen;
		(void)pIn; (void)Len; (void)pOut;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

//-----------------------------------------------------------------------------
// MacEngine: message authentication facet. Base is software CMAC/HMAC. CMAC is
// built over the cipher facet, so on a combined block that overrode Cipher the
// inherited software CMAC uses the hardware AES automatically. A hardware
// driver with a native MAC overrides Mac.
//-----------------------------------------------------------------------------
class MacEngine : virtual public CryptoEngine {
public:
	/**
	 * @brief	One-shot MAC over a full message.
	 *
	 * @param	Alg		MAC algorithm (CRYPTO_MAC_*).
	 * @param	Key		Key descriptor; must permit sign use.
	 * @param	pMsg	Message buffer of Len bytes.
	 * @param	Len		Message length in bytes.
	 * @param	pMac	Output tag buffer of MacLen bytes.
	 * @param	MacLen	Requested tag length (16 for full AES-CMAC).
	 *
	 * @return	CRYPTO_STATUS_OK on success.
	 */
	virtual CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
							  const uint8_t *pMsg, size_t Len,
							  uint8_t *pMac, size_t MacLen) {
		(void)Alg; (void)Key; (void)pMsg; (void)Len; (void)pMac; (void)MacLen;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

//-----------------------------------------------------------------------------
// KeyAgreeEngine: ECDH facet. The base does the point multiply in software
// (micro-ecc); a hardware driver overrides both calls to drive its public-key
// core (Silex BA414EP, Arm CryptoCell). Both the software base and the hardware
// override use the shared crypto_p256 scalar and field hygiene helpers; those
// helpers do not perform point arithmetic, so they are not the base by
// themselves.
//
// The base operations decline (CRYPTO_STATUS_UNSUPPORTED) until a concrete
// engine overrides them: the software engine CryptoUecc, or a hardware engine.
//
// The private key generated by KeyGen is retained by the engine in the caller's
// pKeyCtx and never crosses the interface, so a secure engine can keep it in its
// domain. pKeyCtx is caller-provided static storage sized by KeyCtxSize().
//-----------------------------------------------------------------------------
class KeyAgreeEngine : virtual public CryptoEngine {
public:
	/**
	 * @brief	Bytes of per-instance key context the caller must provide for
	 *			pKeyCtx. Zero when the engine keeps no forwardable context.
	 */
	virtual size_t KeyCtxSize() const { return 0; }

	/**
	 * @brief	Generate a local key pair; return the public key. The private
	 *			key is retained in pKeyCtx for the matching Agree call.
	 *
	 * @param	Curve	Curve (CRYPTO_CURVE_*).
	 * @param	pKeyCtx	Caller storage of KeyCtxSize() bytes.
	 * @param	pPubKey	Output public key, 64 bytes for P-256 (X||Y big-endian).
	 *
	 * @return	CRYPTO_STATUS_OK on success.
	 */
	virtual CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
								 uint8_t *pPubKey) {
		(void)Curve; (void)pKeyCtx; (void)pPubKey;
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	/**
	 * @brief	ECDH shared secret from the peer public key, using the private
	 *			key from the preceding KeyGen. The peer point is validated on the
	 *			curve before use.
	 *
	 * @param	Curve		Curve (CRYPTO_CURVE_*).
	 * @param	pKeyCtx		Same storage passed to KeyGen.
	 * @param	pPeerPubKey	Peer public key, 64 bytes for P-256 (X||Y).
	 * @param	pSharedX	Output shared X coordinate, 32 bytes for P-256.
	 * @param	bKeepKey	When false (default) the private key is single use:
	 *						it is wiped after this call, so a second Agree fails.
	 *						When true the key survives for another Agree, which a
	 *						caller running one ephemeral key pair against several
	 *						concurrent peers (LE Secure Connections) needs; that
	 *						caller wipes the key itself by generating a new pair.
	 *
	 * @return	CRYPTO_STATUS_OK on success, CRYPTO_STATUS_FAIL on an invalid
	 *			peer point.
	 */
	virtual CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
								const uint8_t *pPeerPubKey, uint8_t *pSharedX,
								bool bKeepKey = false) {
		(void)Curve; (void)pKeyCtx; (void)pPeerPubKey; (void)pSharedX;
		(void)bKeepKey;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

//-----------------------------------------------------------------------------
// SignEngine: digital signature facet. Base declines; the software engine
// (CryptoUecc) or a hardware public-key core overrides.
//-----------------------------------------------------------------------------
class SignEngine : virtual public CryptoEngine {
public:
	/**
	 * @brief	Sign a message hash. Needs an RNG for the per-signature nonce.
	 *
	 * @param	Curve	Curve (CRYPTO_CURVE_*).
	 * @param	Key		Private key descriptor; must permit sign use.
	 * @param	pHash	Message hash.
	 * @param	HashLen	Hash length in bytes.
	 * @param	pSig	Output signature, 64 bytes for P-256 (r||s big-endian).
	 */
	virtual CRYPTO_STATUS Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
							   const uint8_t *pHash, size_t HashLen,
							   uint8_t *pSig) {
		(void)Curve; (void)Key; (void)pHash; (void)HashLen; (void)pSig;
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	/**
	 * @brief	Verify a signature over a message hash against a public key.
	 *
	 * @param	Curve		Curve (CRYPTO_CURVE_*).
	 * @param	pPubKey		Public key, 64 bytes for P-256 (X||Y).
	 * @param	pHash		Message hash.
	 * @param	HashLen		Hash length in bytes.
	 * @param	pSig		Signature, 64 bytes for P-256 (r||s).
	 *
	 * @return	CRYPTO_STATUS_OK when valid, CRYPTO_STATUS_FAIL when not.
	 */
	virtual CRYPTO_STATUS Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
								 const uint8_t *pHash, size_t HashLen,
								 const uint8_t *pSig) {
		(void)Curve; (void)pPubKey; (void)pHash; (void)HashLen; (void)pSig;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

//-----------------------------------------------------------------------------
// RngEngine: random generator facet. The base is a software PRNG: deterministic,
// no security claim, usable for statistical needs. A target MCU with a hardware
// entropy source overrides Random with a DRBG. No hardware entropy means no
// DRBG; the derived class is not instantiated and the PRNG base stands.
//
// Security key generation must resolve to the DRBG override, never the PRNG
// base. A consumer that needs security-grade randomness checks IsSecure().
//-----------------------------------------------------------------------------
class RngEngine : virtual public CryptoEngine {
public:
	/**
	 * @brief	Fill pOut with Len random bytes.
	 *
	 * @return	CRYPTO_STATUS_OK on success.
	 */
	virtual CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) {
		(void)pOut; (void)Len;
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	/**
	 * @brief	True when this generator is security grade (a hardware DRBG).
	 *			The PRNG base returns false; a DRBG override returns true. A
	 *			consumer generating keys must refuse to run when this is false.
	 */
	virtual bool IsSecure() const { return false; }
};

/** @} */

#endif // __ICRYPTO_H__
