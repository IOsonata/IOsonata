/**-------------------------------------------------------------------------
@file	icrypto.h

@brief	Object-oriented crypto engine facets, key descriptor and status values.

@author	Hoang Nguyen Hoan
@date	Jul. 15, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/
#ifndef __ICRYPTO_H__
#define __ICRYPTO_H__

#include <stdint.h>
#include <stddef.h>

#include "device.h"

/** @addtogroup Crypto
  * @{
  */

typedef enum __Crypto_Status {
	CRYPTO_STATUS_OK          = 0,
	CRYPTO_STATUS_PENDING     = 1,
	CRYPTO_STATUS_BUSY        = 2,	//!< Transient contention: outputs are
									//!< cleared, a caller-held single-use key
									//!< is preserved for retry
	CRYPTO_STATUS_FAIL        = -1,
	CRYPTO_STATUS_UNSUPPORTED = -2
} CRYPTO_STATUS;

typedef enum __Crypto_Key_Type {
	CRYPTO_KEY_AES_128,
	CRYPTO_KEY_AES_256,
	CRYPTO_KEY_ECC_P256,
	CRYPTO_KEY_ECC_P384,
	CRYPTO_KEY_HMAC,
	CRYPTO_KEY_DERIVE
} CRYPTO_KEY_TYPE;

typedef enum __Crypto_Key_Location {
	CRYPTO_KEY_LOC_PLAIN,
	CRYPTO_KEY_LOC_SLOT,
	CRYPTO_KEY_LOC_OPAQUE
} CRYPTO_KEY_LOCATION;

#define CRYPTO_KEY_USE_ENCRYPT		(1U << 0)
#define CRYPTO_KEY_USE_DECRYPT		(1U << 1)
#define CRYPTO_KEY_USE_SIGN			(1U << 2)
#define CRYPTO_KEY_USE_VERIFY		(1U << 3)
#define CRYPTO_KEY_USE_DERIVE		(1U << 4)
#define CRYPTO_KEY_USE_AGREE		(1U << 5)

typedef struct __Crypto_Key {
	CRYPTO_KEY_TYPE Type;
	CRYPTO_KEY_LOCATION Loc;
	uint32_t Usage;
	union {
		struct {
			const uint8_t *pData;
			size_t Len;
		} Plain;
		uint32_t SlotIndex;
		uint32_t OpaqueId;
	};
} CryptoKey;

typedef enum __Crypto_Cipher_Alg {
	CRYPTO_CIPHER_ECB,
	CRYPTO_CIPHER_CTR,
	CRYPTO_CIPHER_CBC,
	CRYPTO_CIPHER_XTS
} CRYPTO_CIPHER_ALG;

typedef enum __Crypto_Mac_Alg {
	CRYPTO_MAC_CMAC,
	CRYPTO_MAC_HMAC,
	CRYPTO_MAC_GMAC
} CRYPTO_MAC_ALG;

typedef enum __Crypto_Hash_Alg {
	CRYPTO_HASH_SHA256
} CRYPTO_HASH_ALG;

typedef enum __Crypto_Aead_Alg {
	CRYPTO_AEAD_AES_CCM,
	CRYPTO_AEAD_AES_GCM
} CRYPTO_AEAD_ALG;

typedef enum __Crypto_Curve {
	CRYPTO_CURVE_P256,
	CRYPTO_CURVE_P384
} CRYPTO_CURVE;

#define P256_BYTES	32U
#define CRYPTO_KEYCTX_MAX	64U
#define CRYPTO_HASHCTX_MAX	128U
#define CRYPTO_KEYCTX_ALIGN_MAX	16U
#define CRYPTO_HASHCTX_ALIGN_MAX	16U

#if defined(__GNUC__)
#define CRYPTO_ALIGNED(n)	__attribute__((aligned(n)))
#else
#define CRYPTO_ALIGNED(n)
#endif

#ifdef __cplusplus
class CryptoEngine;
class CipherEngine;
class MacEngine;
class HashEngine;
class AeadEngine;
class KeyAgreeEngine;
class SignEngine;
class RngEngine;
extern "C" {
#else
typedef struct CryptoEngine CryptoEngine;
typedef struct CipherEngine CipherEngine;
typedef struct MacEngine MacEngine;
typedef struct HashEngine HashEngine;
typedef struct AeadEngine AeadEngine;
typedef struct KeyAgreeEngine KeyAgreeEngine;
typedef struct SignEngine SignEngine;
typedef struct RngEngine RngEngine;
#endif

bool P256IsZero(const uint8_t *pData, size_t Len);
bool P256LessBe(const uint8_t *pA, const uint8_t *pB, size_t Len);
bool P256ScalarInRange(const uint8_t Scalar[P256_BYTES]);
bool P256NonzeroFieldElement(const uint8_t Coord[P256_BYTES]);
void P256RegularizeScalar(const uint8_t K[P256_BYTES],
						  uint8_t R[P256_BYTES + 1U]);
uint32_t P256RegularBit(const uint8_t Scalar[P256_BYTES + 1U],
						uint32_t BitNo);
void CryptoSecureWipe(void *pData, size_t Len);
CRYPTO_STATUS P256RandomScalar(RngEngine *pRng,
							  uint8_t Scalar[P256_BYTES]);

#ifdef __cplusplus
}

typedef enum __Crypto_Op {
	CRYPTO_OP_NONE = 0,
	CRYPTO_OP_CIPHER,
	CRYPTO_OP_MAC,
	CRYPTO_OP_KEYGEN,
	CRYPTO_OP_AGREE,
	CRYPTO_OP_SIGN,
	CRYPTO_OP_VERIFY,
	CRYPTO_OP_HASH,
	CRYPTO_OP_RANDOM,
	CRYPTO_OP_SEAL,
	CRYPTO_OP_OPEN
} CRYPTO_OP;

typedef void (*CryptoCompleteHandler_t)(CryptoEngine * const pEngine,
										CRYPTO_OP Op,
										CRYPTO_STATUS Status,
										void *pCtx);

class CryptoEngine : virtual public Device {
public:
	virtual ~CryptoEngine() {}
	// Known-answer self-test: 0 on pass. The default reports not implemented
	// as a failure, so an engine that claims a facet without providing its
	// own test cannot report healthy hardware it never exercised.
	virtual int SelfTest() { return -1; }
	void SetCompleteHandler(CryptoCompleteHandler_t Handler, void *pCtx)
	{
		vCompleteHandler = Handler;
		vpCompleteCtx = pCtx;
	}
	virtual bool IsAsync() const { return false; }

protected:
	void Complete(CRYPTO_OP Op, CRYPTO_STATUS Status)
	{
		if (vCompleteHandler != nullptr)
		{
			vCompleteHandler(this, Op, Status, vpCompleteCtx);
		}
	}
	CryptoCompleteHandler_t vCompleteHandler = nullptr;
	void *vpCompleteCtx = nullptr;
};

class CipherEngine : virtual public CryptoEngine {
public:
	virtual CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
								 const CryptoKey &Key,
								 const uint8_t *pIv, size_t IvLen,
								 const uint8_t *pIn, size_t Len,
								 uint8_t *pOut)
	{
		(void)Alg; (void)bEncrypt; (void)Key; (void)pIv; (void)IvLen;
		(void)pIn; (void)Len; (void)pOut;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

class AeadEngine : virtual public CryptoEngine {
public:
	// Authenticated encryption with associated data. Seal encrypts pMsg and
	// emits the tag; Open decrypts, verifies the tag in constant time, and
	// zeroes pOut on any failure. Seal enforces CRYPTO_KEY_USE_ENCRYPT and
	// Open CRYPTO_KEY_USE_DECRYPT. pOut may equal pMsg for in-place work.
	virtual CRYPTO_STATUS Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
							   const uint8_t *pNonce, size_t NonceLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pMsg, size_t Len,
							   uint8_t *pOut, uint8_t *pTag, size_t TagLen)
	{
		(void)Alg; (void)Key; (void)pNonce; (void)NonceLen; (void)pAad;
		(void)AadLen; (void)pMsg; (void)Len; (void)pOut; (void)pTag;
		(void)TagLen;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	virtual CRYPTO_STATUS Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
							   const uint8_t *pNonce, size_t NonceLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pMsg, size_t Len,
							   uint8_t *pOut, const uint8_t *pTag,
							   size_t TagLen)
	{
		(void)Alg; (void)Key; (void)pNonce; (void)NonceLen; (void)pAad;
		(void)AadLen; (void)pMsg; (void)Len; (void)pOut; (void)pTag;
		(void)TagLen;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

class MacEngine : virtual public CryptoEngine {
public:
	virtual CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
							 const uint8_t *pMsg, size_t Len,
							 uint8_t *pMac, size_t MacLen)
	{
		(void)Alg; (void)Key; (void)pMsg; (void)Len; (void)pMac; (void)MacLen;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

class HashEngine : virtual public CryptoEngine {
public:
	virtual CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg,
							  size_t Len, uint8_t *pDigest)
	{
		(void)Alg; (void)pMsg; (void)Len; (void)pDigest;
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	// Streaming digest over caller-provided context storage of HashCtxSize()
	// bytes aligned to HashCtxAlign() (all in-tree consumers reserve
	// CRYPTO_HASHCTX_MAX with alignas(uint64_t)). HashInit begins, HashUpdate
	// appends any number of times, HashFinal emits the digest and
	// invalidates the context; an uninitialized, finalized or misaligned
	// context is rejected. HMAC and multi-part consumers (signatures over
	// large images, TLS transcripts) build on these.
	virtual size_t HashCtxSize() const { return 0; }
	virtual size_t HashCtxAlign() const { return alignof(uint64_t); }
	virtual CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx)
	{
		(void)Alg; (void)pHashCtx;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	virtual CRYPTO_STATUS HashUpdate(void *pHashCtx, const uint8_t *pMsg,
									 size_t Len)
	{
		(void)pHashCtx; (void)pMsg; (void)Len;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	virtual CRYPTO_STATUS HashFinal(void *pHashCtx, uint8_t *pDigest)
	{
		(void)pHashCtx; (void)pDigest;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

class KeyAgreeEngine : virtual public CryptoEngine {
public:
	// All in-tree consumers reserve CRYPTO_KEYCTX_MAX bytes. A provider that
	// needs more storage is not compatible with those fixed embedded consumers.
	virtual size_t KeyCtxSize() const { return 0; }
	virtual size_t KeyCtxAlign() const { return 1U; }
	virtual void KeyReset(void *pKeyCtx) { (void)pKeyCtx; }
	virtual CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
								 uint8_t *pPubKey)
	{
		(void)Curve; (void)pKeyCtx; (void)pPubKey;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	virtual CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
								const uint8_t *pPeerPubKey,
								uint8_t *pSharedX,
								bool bKeepKey = false)
	{
		(void)Curve; (void)pKeyCtx; (void)pPeerPubKey; (void)pSharedX;
		(void)bKeepKey;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

class SignEngine : virtual public CryptoEngine {
public:
	virtual CRYPTO_STATUS Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
							  const uint8_t *pHash, size_t HashLen,
							  uint8_t *pSig)
	{
		(void)Curve; (void)Key; (void)pHash; (void)HashLen; (void)pSig;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	virtual CRYPTO_STATUS Verify(CRYPTO_CURVE Curve,
								const uint8_t *pPubKey,
								const uint8_t *pHash, size_t HashLen,
								const uint8_t *pSig)
	{
		(void)Curve; (void)pPubKey; (void)pHash; (void)HashLen; (void)pSig;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
};

class RngEngine : virtual public CryptoEngine {
public:
	virtual CRYPTO_STATUS Random(uint8_t *pOut, size_t Len)
	{
		(void)pOut; (void)Len;
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	virtual bool IsSecure() const { return false; }
};

#endif // __cplusplus

/** @} */

#endif // __ICRYPTO_H__
