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
// AES-CMAC (RFC 4493) computed over an engine Aes128Ecb. Any engine that
// provides AES-128 ECB gets CMAC through this path; an engine with a native
// MAC sets pDev->Cmac and CryptoCmac dispatches to it instead.
//-----------------------------------------------------------------------------

static void CmacLeftShift(const uint8_t In[16], uint8_t Out[16])
{
	uint8_t carry = 0;
	for (int i = 15; i >= 0; i--)
	{
		uint8_t b = In[i];
		Out[i] = (uint8_t)((b << 1) | carry);
		carry = (uint8_t)((b >> 7) & 1);
	}
}

// Subkey: Out = shift(In), then XOR Rb (0x87) into the last byte if MSB(In) set.
static void CmacSubkey(const uint8_t In[16], uint8_t Out[16])
{
	uint8_t msb = (uint8_t)(In[0] >> 7);
	CmacLeftShift(In, Out);
	if (msb)
	{
		Out[15] ^= 0x87;
	}
}

CRYPTO_STATUS CryptoCmac(CryptoDev_t * const pDev, const uint8_t Key[16],
						 const uint8_t *pMsg, size_t Len, uint8_t Mac[16],
						 void *pCtx)
{
	if (pDev == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (pDev->Cmac != NULL)
	{
		return pDev->Cmac(pDev, Key, pMsg, Len, Mac, pCtx);
	}
	if (pDev->Aes128Ecb == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	// L = AES(Key, 0); derive subkeys K1, K2.
	uint8_t zero[16] = { 0 };
	uint8_t L[16], K1[16], K2[16];
	if (CryptoAes128Ecb(pDev, Key, zero, L, pCtx) != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}
	CmacSubkey(L, K1);
	CmacSubkey(K1, K2);

	size_t nBlocks = (Len + 15) / 16;
	bool lastComplete = (Len != 0) && ((Len & 15) == 0);
	if (nBlocks == 0)
	{
		nBlocks = 1;	// empty message: one padded block
	}

	uint8_t X[16] = { 0 };
	uint8_t Y[16];
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;

	// All but the last block: X = AES(Key, X XOR M_i).
	for (size_t i = 0; i + 1 < nBlocks; i++)
	{
		for (int j = 0; j < 16; j++)
		{
			Y[j] = (uint8_t)(X[j] ^ pMsg[i * 16 + j]);
		}
		if (CryptoAes128Ecb(pDev, Key, Y, X, pCtx) != CRYPTO_STATUS_OK)
		{
			st = CRYPTO_STATUS_FAIL;
			break;
		}
	}

	if (st == CRYPTO_STATUS_OK)
	{
		// Last block: complete -> XOR K1; incomplete or empty -> pad(10*) XOR K2.
		uint8_t last[16];
		size_t base = (nBlocks - 1) * 16;
		if (lastComplete)
		{
			for (int j = 0; j < 16; j++)
			{
				last[j] = (uint8_t)(pMsg[base + j] ^ K1[j]);
			}
		}
		else
		{
			size_t rem = Len - base;	// 0 for empty, 1..15 otherwise
			for (int j = 0; j < 16; j++)
			{
				uint8_t mb = ((size_t)j < rem) ? pMsg[base + j] :
							 ((size_t)j == rem ? 0x80 : 0x00);
				last[j] = (uint8_t)(mb ^ K2[j]);
			}
		}
		for (int j = 0; j < 16; j++)
		{
			Y[j] = (uint8_t)(X[j] ^ last[j]);
		}
		if (CryptoAes128Ecb(pDev, Key, Y, Mac, pCtx) != CRYPTO_STATUS_OK)
		{
			st = CRYPTO_STATUS_FAIL;
		}
	}

	CryptoSecureWipe(L, sizeof(L));
	CryptoSecureWipe(K1, sizeof(K1));
	CryptoSecureWipe(K2, sizeof(K2));
	CryptoSecureWipe(X, sizeof(X));
	CryptoSecureWipe(Y, sizeof(Y));
	return st;
}

//-----------------------------------------------------------------------------
// AES-CCM (RFC 3610 / NIST SP 800-38C) computed over an engine Aes128Ecb.
// CBC-MAC for authentication, CTR for encryption, both over the same block
// cipher. An engine with a native AEAD sets pDev->Ccm and CryptoCcm* dispatch
// to it. Only the 2-byte AAD length encoding is supported (AadLen < 0xFF00).
//-----------------------------------------------------------------------------

// CBC-MAC over B0 || (2-byte len || AAD, padded) || (message, padded). Writes T.
static CRYPTO_STATUS CcmMac(CryptoDev_t * const pDev, const uint8_t Key[16],
							const uint8_t *pNonce, size_t NonceLen,
							const uint8_t *pAad, size_t AadLen,
							const uint8_t *pMsg, size_t MsgLen,
							size_t TagLen, uint8_t T[16], void *pCtx)
{
	size_t L = 15 - NonceLen;
	uint8_t B[16], X[16], Y[16];

	B[0] = (uint8_t)((AadLen > 0 ? 0x40 : 0) |
					 (uint8_t)(((TagLen - 2) / 2) << 3) | (uint8_t)(L - 1));
	memcpy(&B[1], pNonce, NonceLen);
	for (size_t i = 0; i < L; i++)
	{
		B[15 - i] = (uint8_t)((MsgLen >> (8 * i)) & 0xFF);
	}
	if (CryptoAes128Ecb(pDev, Key, B, X, pCtx) != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	if (AadLen > 0)
	{
		uint8_t blk[16];
		size_t p = 0;
		blk[p++] = (uint8_t)((AadLen >> 8) & 0xFF);
		blk[p++] = (uint8_t)(AadLen & 0xFF);
		size_t a = 0;
		while (a < AadLen)
		{
			blk[p++] = pAad[a++];
			if (p == 16)
			{
				for (int j = 0; j < 16; j++) { Y[j] = (uint8_t)(X[j] ^ blk[j]); }
				if (CryptoAes128Ecb(pDev, Key, Y, X, pCtx) != CRYPTO_STATUS_OK)
				{
					return CRYPTO_STATUS_FAIL;
				}
				p = 0;
			}
		}
		if (p > 0)
		{
			while (p < 16) { blk[p++] = 0; }
			for (int j = 0; j < 16; j++) { Y[j] = (uint8_t)(X[j] ^ blk[j]); }
			if (CryptoAes128Ecb(pDev, Key, Y, X, pCtx) != CRYPTO_STATUS_OK)
			{
				return CRYPTO_STATUS_FAIL;
			}
		}
	}

	size_t m = 0;
	while (m < MsgLen)
	{
		uint8_t blk[16];
		size_t n = (MsgLen - m < 16) ? (MsgLen - m) : 16;
		for (int j = 0; j < 16; j++)
		{
			blk[j] = ((size_t)j < n) ? pMsg[m + j] : 0;
		}
		for (int j = 0; j < 16; j++) { Y[j] = (uint8_t)(X[j] ^ blk[j]); }
		if (CryptoAes128Ecb(pDev, Key, Y, X, pCtx) != CRYPTO_STATUS_OK)
		{
			return CRYPTO_STATUS_FAIL;
		}
		m += n;
	}
	memcpy(T, X, 16);
	CryptoSecureWipe(X, sizeof(X));
	CryptoSecureWipe(Y, sizeof(Y));
	return CRYPTO_STATUS_OK;
}

// CTR keystream. S0 (counter 0) is returned for tag masking; data uses i >= 1.
static CRYPTO_STATUS CcmCtr(CryptoDev_t * const pDev, const uint8_t Key[16],
							const uint8_t *pNonce, size_t NonceLen,
							const uint8_t *pIn, size_t Len, uint8_t *pOut,
							uint8_t S0[16], void *pCtx)
{
	size_t L = 15 - NonceLen;
	uint8_t A[16], S[16];

	A[0] = (uint8_t)(L - 1);
	memcpy(&A[1], pNonce, NonceLen);
	for (size_t i = 0; i < L; i++) { A[15 - i] = 0; }
	if (CryptoAes128Ecb(pDev, Key, A, S0, pCtx) != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	size_t off = 0;
	uint64_t ctr = 1;
	while (off < Len)
	{
		for (size_t i = 0; i < L; i++)
		{
			A[15 - i] = (uint8_t)((ctr >> (8 * i)) & 0xFF);
		}
		if (CryptoAes128Ecb(pDev, Key, A, S, pCtx) != CRYPTO_STATUS_OK)
		{
			CryptoSecureWipe(S, sizeof(S));
			return CRYPTO_STATUS_FAIL;
		}
		size_t n = (Len - off < 16) ? (Len - off) : 16;
		for (size_t j = 0; j < n; j++)
		{
			pOut[off + j] = (uint8_t)(pIn[off + j] ^ S[j]);
		}
		off += n;
		ctr++;
	}
	CryptoSecureWipe(S, sizeof(S));
	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS CcmValidate(size_t NonceLen, size_t TagLen, size_t Len,
								 size_t AadLen)
{
	if (NonceLen < 7 || NonceLen > 13)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (TagLen < 4 || TagLen > 16 || (TagLen & 1))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (AadLen >= 0xFF00)
	{
		return CRYPTO_STATUS_FAIL;	// only the 2-byte AAD length encoding
	}
	size_t L = 15 - NonceLen;
	if (L < sizeof(size_t))
	{
		size_t maxMsg = ((size_t)1 << (8 * L)) - 1;
		if (Len > maxMsg)
		{
			return CRYPTO_STATUS_FAIL;
		}
	}
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoCcmEncrypt(CryptoDev_t * const pDev, const uint8_t Key[16],
							   const uint8_t *pNonce, size_t NonceLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pPlain, size_t Len, uint8_t *pCipher,
							   uint8_t *pTag, size_t TagLen, void *pCtx)
{
	if (pDev == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (pDev->Ccm != NULL)
	{
		return pDev->Ccm(pDev, 1, Key, pNonce, NonceLen, pAad, AadLen,
						 pPlain, Len, pCipher, pTag, TagLen, pCtx);
	}
	if (pDev->Aes128Ecb == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (CcmValidate(NonceLen, TagLen, Len, AadLen) != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t T[16], S0[16];
	CRYPTO_STATUS st = CcmMac(pDev, Key, pNonce, NonceLen, pAad, AadLen,
							  pPlain, Len, TagLen, T, pCtx);
	if (st == CRYPTO_STATUS_OK)
	{
		st = CcmCtr(pDev, Key, pNonce, NonceLen, pPlain, Len, pCipher, S0, pCtx);
	}
	if (st == CRYPTO_STATUS_OK)
	{
		for (size_t i = 0; i < TagLen; i++)
		{
			pTag[i] = (uint8_t)(T[i] ^ S0[i]);
		}
	}
	CryptoSecureWipe(T, sizeof(T));
	CryptoSecureWipe(S0, sizeof(S0));
	return st;
}

CRYPTO_STATUS CryptoCcmDecrypt(CryptoDev_t * const pDev, const uint8_t Key[16],
							   const uint8_t *pNonce, size_t NonceLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pCipher, size_t Len, uint8_t *pPlain,
							   const uint8_t *pTag, size_t TagLen, void *pCtx)
{
	if (pDev == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (pDev->Ccm != NULL)
	{
		return pDev->Ccm(pDev, 0, Key, pNonce, NonceLen, pAad, AadLen,
						 pCipher, Len, pPlain, (uint8_t *)pTag, TagLen, pCtx);
	}
	if (pDev->Aes128Ecb == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (CcmValidate(NonceLen, TagLen, Len, AadLen) != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t T[16], S0[16];
	CRYPTO_STATUS st = CcmCtr(pDev, Key, pNonce, NonceLen, pCipher, Len, pPlain, S0, pCtx);
	if (st == CRYPTO_STATUS_OK)
	{
		st = CcmMac(pDev, Key, pNonce, NonceLen, pAad, AadLen, pPlain, Len, TagLen, T, pCtx);
	}

	uint8_t diff = 0;
	if (st == CRYPTO_STATUS_OK)
	{
		for (size_t i = 0; i < TagLen; i++)
		{
			diff |= (uint8_t)((T[i] ^ S0[i]) ^ pTag[i]);
		}
	}
	else
	{
		diff = 1;
	}

	CryptoSecureWipe(T, sizeof(T));
	CryptoSecureWipe(S0, sizeof(S0));
	if (diff != 0)
	{
		CryptoSecureWipe(pPlain, Len);	// do not release unverified plaintext
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

//-----------------------------------------------------------------------------
// AES-GCM (NIST SP 800-38D) computed over an engine Aes128Ecb. CTR for
// encryption, GHASH over GF(2^128) for authentication. An engine with a native
// AEAD sets pDev->Gcm and CryptoGcm* dispatch to it.
//-----------------------------------------------------------------------------

// Y = Y * H in GF(2^128), GCM bit convention (reduction R = 0xe1 << 120).
static void GcmMul(uint8_t Y[16], const uint8_t H[16])
{
	uint8_t Z[16], V[16];
	memset(Z, 0, 16);
	memcpy(V, H, 16);
	for (int i = 0; i < 128; i++)
	{
		if ((Y[i >> 3] >> (7 - (i & 7))) & 1)
		{
			for (int j = 0; j < 16; j++) { Z[j] ^= V[j]; }
		}
		uint8_t lsb = (uint8_t)(V[15] & 1);
		for (int j = 15; j > 0; j--)
		{
			V[j] = (uint8_t)((V[j] >> 1) | (V[j - 1] << 7));
		}
		V[0] = (uint8_t)(V[0] >> 1);
		if (lsb) { V[0] ^= 0xe1; }
	}
	memcpy(Y, Z, 16);
}

// GHASH accumulate: for each 16-byte block (zero-padded), Y = (Y ^ B) * H.
static void GcmGhash(uint8_t Y[16], const uint8_t H[16], const uint8_t *pData, size_t Len)
{
	size_t off = 0;
	while (off < Len)
	{
		size_t n = (Len - off < 16) ? (Len - off) : 16;
		for (size_t j = 0; j < 16; j++)
		{
			Y[j] ^= (j < n) ? pData[off + j] : 0;
		}
		GcmMul(Y, H);
		off += n;
	}
}

static void GcmInc32(uint8_t Ctr[16])
{
	for (int i = 15; i >= 12; i--)
	{
		if (++Ctr[i] != 0) { break; }
	}
}

// Shared GCM core. GHASH runs over AAD then the ciphertext; on decrypt the
// ciphertext (pIn) is hashed before the CTR pass so in-place decrypt is safe.
static CRYPTO_STATUS GcmCrypt(CryptoDev_t * const pDev, int bEncrypt,
							  const uint8_t Key[16], const uint8_t *pIv, size_t IvLen,
							  const uint8_t *pAad, size_t AadLen,
							  const uint8_t *pIn, size_t Len, uint8_t *pOut,
							  uint8_t *pTag, size_t TagLen, void *pCtx)
{
	if (IvLen == 0 || TagLen < 4 || TagLen > 16)
	{
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t H[16], J0[16], EJ0[16], ctr[16], ks[16], Y[16], zero[16];
	memset(zero, 0, 16);
	if (CryptoAes128Ecb(pDev, Key, zero, H, pCtx) != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	if (IvLen == 12)
	{
		memcpy(J0, pIv, 12);
		J0[12] = 0; J0[13] = 0; J0[14] = 0; J0[15] = 1;
	}
	else
	{
		uint8_t lenblk[16];
		memset(J0, 0, 16);
		GcmGhash(J0, H, pIv, IvLen);
		memset(lenblk, 0, 16);
		uint64_t ivbits = (uint64_t)IvLen * 8;
		for (int i = 0; i < 8; i++)
		{
			lenblk[8 + i] = (uint8_t)((ivbits >> (56 - 8 * i)) & 0xFF);
		}
		for (int j = 0; j < 16; j++) { J0[j] ^= lenblk[j]; }
		GcmMul(J0, H);
	}

	if (CryptoAes128Ecb(pDev, Key, J0, EJ0, pCtx) != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}

	memset(Y, 0, 16);
	GcmGhash(Y, H, pAad, AadLen);
	if (!bEncrypt)
	{
		GcmGhash(Y, H, pIn, Len);		// decrypt: hash ciphertext before CTR
	}

	memcpy(ctr, J0, 16);
	size_t off = 0;
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;
	while (off < Len)
	{
		GcmInc32(ctr);
		if (CryptoAes128Ecb(pDev, Key, ctr, ks, pCtx) != CRYPTO_STATUS_OK)
		{
			st = CRYPTO_STATUS_FAIL;
			break;
		}
		size_t n = (Len - off < 16) ? (Len - off) : 16;
		for (size_t j = 0; j < n; j++)
		{
			pOut[off + j] = (uint8_t)(pIn[off + j] ^ ks[j]);
		}
		off += n;
	}

	CRYPTO_STATUS rc = st;
	if (st == CRYPTO_STATUS_OK)
	{
		uint8_t lenblk[16];
		if (bEncrypt)
		{
			GcmGhash(Y, H, pOut, Len);	// encrypt: hash ciphertext after CTR
		}
		uint64_t abits = (uint64_t)AadLen * 8;
		uint64_t cbits = (uint64_t)Len * 8;
		for (int i = 0; i < 8; i++)
		{
			lenblk[i]     = (uint8_t)((abits >> (56 - 8 * i)) & 0xFF);
			lenblk[8 + i] = (uint8_t)((cbits >> (56 - 8 * i)) & 0xFF);
		}
		for (int j = 0; j < 16; j++) { Y[j] ^= lenblk[j]; }
		GcmMul(Y, H);
		for (int j = 0; j < 16; j++) { Y[j] ^= EJ0[j]; }	// T = GHASH XOR E(J0)

		if (bEncrypt)
		{
			memcpy(pTag, Y, TagLen);
		}
		else
		{
			uint8_t diff = 0;
			for (size_t i = 0; i < TagLen; i++) { diff |= (uint8_t)(Y[i] ^ pTag[i]); }
			if (diff != 0)
			{
				CryptoSecureWipe(pOut, Len);	// do not release unverified plaintext
				rc = CRYPTO_STATUS_FAIL;
			}
		}
	}

	CryptoSecureWipe(H, sizeof(H));
	CryptoSecureWipe(EJ0, sizeof(EJ0));
	CryptoSecureWipe(ks, sizeof(ks));
	CryptoSecureWipe(Y, sizeof(Y));
	return rc;
}

CRYPTO_STATUS CryptoGcmEncrypt(CryptoDev_t * const pDev, const uint8_t Key[16],
							   const uint8_t *pIv, size_t IvLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pPlain, size_t Len, uint8_t *pCipher,
							   uint8_t *pTag, size_t TagLen, void *pCtx)
{
	if (pDev == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (pDev->Gcm != NULL)
	{
		return pDev->Gcm(pDev, 1, Key, pIv, IvLen, pAad, AadLen,
						 pPlain, Len, pCipher, pTag, TagLen, pCtx);
	}
	if (pDev->Aes128Ecb == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	return GcmCrypt(pDev, 1, Key, pIv, IvLen, pAad, AadLen, pPlain, Len, pCipher,
					pTag, TagLen, pCtx);
}

CRYPTO_STATUS CryptoGcmDecrypt(CryptoDev_t * const pDev, const uint8_t Key[16],
							   const uint8_t *pIv, size_t IvLen,
							   const uint8_t *pAad, size_t AadLen,
							   const uint8_t *pCipher, size_t Len, uint8_t *pPlain,
							   const uint8_t *pTag, size_t TagLen, void *pCtx)
{
	if (pDev == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (pDev->Gcm != NULL)
	{
		return pDev->Gcm(pDev, 0, Key, pIv, IvLen, pAad, AadLen,
						 pCipher, Len, pPlain, (uint8_t *)pTag, TagLen, pCtx);
	}
	if (pDev->Aes128Ecb == NULL)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	return GcmCrypt(pDev, 0, Key, pIv, IvLen, pAad, AadLen, pCipher, Len, pPlain,
					(uint8_t *)pTag, TagLen, pCtx);
}

//-----------------------------------------------------------------------------
// SHA-256 (FIPS 180-4) software core, and HMAC-SHA-256 (RFC 2104) over it.
// CryptoSha256 uses a native engine hash (pDev->Sha256) when present, else this
// core, and is always available. HMAC is computed with the software core.
//-----------------------------------------------------------------------------

static const uint32_t s_Sha256K[64] = {
	0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,0x923f82a4u,0xab1c5ed5u,
	0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,
	0xe49b69c1u,0xefbe4786u,0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
	0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,0x06ca6351u,0x14292967u,
	0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,
	0xa2bfe8a1u,0xa81a664bu,0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
	0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,0x5b9cca4fu,0x682e6ff3u,
	0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u
};

static uint32_t Sha256Ror(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

static void Sha256Block(uint32_t H[8], const uint8_t blk[64])
{
	uint32_t W[64];
	for (int t = 0; t < 16; t++)
	{
		W[t] = ((uint32_t)blk[4 * t] << 24) | ((uint32_t)blk[4 * t + 1] << 16) |
			   ((uint32_t)blk[4 * t + 2] << 8) | (uint32_t)blk[4 * t + 3];
	}
	for (int t = 16; t < 64; t++)
	{
		uint32_t s0 = Sha256Ror(W[t - 15], 7) ^ Sha256Ror(W[t - 15], 18) ^ (W[t - 15] >> 3);
		uint32_t s1 = Sha256Ror(W[t - 2], 17) ^ Sha256Ror(W[t - 2], 19) ^ (W[t - 2] >> 10);
		W[t] = W[t - 16] + s0 + W[t - 7] + s1;
	}
	uint32_t a = H[0], b = H[1], c = H[2], d = H[3], e = H[4], f = H[5], g = H[6], h = H[7];
	for (int t = 0; t < 64; t++)
	{
		uint32_t S1 = Sha256Ror(e, 6) ^ Sha256Ror(e, 11) ^ Sha256Ror(e, 25);
		uint32_t ch = (e & f) ^ (~e & g);
		uint32_t t1 = h + S1 + ch + s_Sha256K[t] + W[t];
		uint32_t S0 = Sha256Ror(a, 2) ^ Sha256Ror(a, 13) ^ Sha256Ror(a, 22);
		uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
		uint32_t t2 = S0 + maj;
		h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
	}
	H[0] += a; H[1] += b; H[2] += c; H[3] += d; H[4] += e; H[5] += f; H[6] += g; H[7] += h;
}

typedef struct {
	uint32_t H[8];
	uint64_t Total;
	uint8_t  Buf[64];
	size_t   BufLen;
} Sha256Ctx;

static void Sha256Init(Sha256Ctx *c)
{
	static const uint32_t H0[8] = { 0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
									0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u };
	memcpy(c->H, H0, sizeof(H0));
	c->Total = 0;
	c->BufLen = 0;
}

static void Sha256Update(Sha256Ctx *c, const uint8_t *p, size_t n)
{
	c->Total += n;
	while (n > 0)
	{
		size_t take = 64 - c->BufLen;
		if (take > n) { take = n; }
		memcpy(c->Buf + c->BufLen, p, take);
		c->BufLen += take;
		p += take;
		n -= take;
		if (c->BufLen == 64)
		{
			Sha256Block(c->H, c->Buf);
			c->BufLen = 0;
		}
	}
}

static void Sha256Final(Sha256Ctx *c, uint8_t Digest[32])
{
	uint64_t bits = c->Total * 8;
	c->Buf[c->BufLen++] = 0x80;
	if (c->BufLen > 56)
	{
		memset(c->Buf + c->BufLen, 0, 64 - c->BufLen);
		Sha256Block(c->H, c->Buf);
		c->BufLen = 0;
	}
	memset(c->Buf + c->BufLen, 0, 56 - c->BufLen);
	for (int i = 0; i < 8; i++)
	{
		c->Buf[56 + i] = (uint8_t)((bits >> (56 - 8 * i)) & 0xFF);
	}
	Sha256Block(c->H, c->Buf);
	for (int i = 0; i < 8; i++)
	{
		Digest[4 * i]     = (uint8_t)(c->H[i] >> 24);
		Digest[4 * i + 1] = (uint8_t)(c->H[i] >> 16);
		Digest[4 * i + 2] = (uint8_t)(c->H[i] >> 8);
		Digest[4 * i + 3] = (uint8_t)(c->H[i]);
	}
}

static void Sha256Sw(const uint8_t *pMsg, size_t Len, uint8_t Digest[32])
{
	Sha256Ctx c;
	Sha256Init(&c);
	if (Len > 0)
	{
		Sha256Update(&c, pMsg, Len);
	}
	Sha256Final(&c, Digest);
}

CRYPTO_STATUS CryptoSha256(CryptoDev_t * const pDev, const uint8_t *pMsg,
						   size_t Len, uint8_t Digest[32], void *pCtx)
{
	if (pDev != NULL && pDev->Sha256 != NULL)
	{
		return pDev->Sha256(pDev, pMsg, Len, Digest, pCtx);
	}
	Sha256Sw(pMsg, Len, Digest);
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoHmacSha256(CryptoDev_t * const pDev,
							   const uint8_t *pKey, size_t KeyLen,
							   const uint8_t *pMsg, size_t Len, uint8_t Mac[32],
							   void *pCtx)
{
	uint8_t k0[64];
	if (KeyLen > 64)
	{
		// Reduce a long key with SHA-256 (native if the engine provides it).
		if (CryptoSha256(pDev, pKey, KeyLen, k0, pCtx) != CRYPTO_STATUS_OK)
		{
			return CRYPTO_STATUS_FAIL;
		}
		memset(k0 + 32, 0, 32);
	}
	else
	{
		memcpy(k0, pKey, KeyLen);
		memset(k0 + KeyLen, 0, 64 - KeyLen);
	}

	uint8_t ipad[64], opad[64], inner[32];
	for (int i = 0; i < 64; i++)
	{
		ipad[i] = (uint8_t)(k0[i] ^ 0x36);
		opad[i] = (uint8_t)(k0[i] ^ 0x5c);
	}

	Sha256Ctx c;
	Sha256Init(&c);
	Sha256Update(&c, ipad, 64);
	if (Len > 0)
	{
		Sha256Update(&c, pMsg, Len);
	}
	Sha256Final(&c, inner);

	Sha256Init(&c);
	Sha256Update(&c, opad, 64);
	Sha256Update(&c, inner, 32);
	Sha256Final(&c, Mac);

	CryptoSecureWipe(k0, sizeof(k0));
	CryptoSecureWipe(ipad, sizeof(ipad));
	CryptoSecureWipe(opad, sizeof(opad));
	CryptoSecureWipe(inner, sizeof(inner));
	return CRYPTO_STATUS_OK;
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
	// plain zeroable bytes (CRYPTO_PROP_PLAIN_KEYCTX). An engine needing structured
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
			// plain zeroable context (CRYPTO_PROP_PLAIN_KEYCTX), and the arena
			// must be at least the engine key-context size (KeyCtxSize) so a
			// keyed op does not write past pMem.
			if ((pEcdh->Props & CRYPTO_PROP_PLAIN_KEYCTX) == 0)
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
	d->Cap            = have;	// operations only; PLAIN_KEYCTX is a Props bit, never in Cap
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
