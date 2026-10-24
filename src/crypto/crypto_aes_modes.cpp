/**-------------------------------------------------------------------------
@file	crypto_aes_modes.cpp

@brief	AES modes, authentication algorithms and software engine adapter.

		CMAC is computed through the virtual Cipher operation. A hardware subclass
		therefore accelerates inherited CMAC without duplicating the MAC code. The
		public operations enforce CryptoKey usage policy; CMAC uses an internal key
		descriptor for its block-cipher calls after validating SIGN permission.

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
#include <stdint.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softaes.h"

#define AES_BLOCK		16U
#define AES128_ROUNDS	10U
#define AES128_RK_WORDS	44U

static void AesWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0U)
	{
		*p++ = 0U;
	}
}

void CryptoAesSoftEncryptBlock(const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16]);
void CryptoAesSoftDecryptBlock(const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16]);

static bool AesKeyMaterialOk(const CryptoKey &Key)
{
	return Key.Type == CRYPTO_KEY_AES_128 &&
		Key.Loc == CRYPTO_KEY_LOC_PLAIN &&
		Key.Plain.pData != nullptr && Key.Plain.Len == AES_BLOCK;
}

CRYPTO_STATUS CryptoSoftAes::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
									const CryptoKey &Key,
									const uint8_t *pIv, size_t IvLen,
									const uint8_t *pIn, size_t Len,
									uint8_t *pOut)
{
	uint32_t required = bEncrypt != 0 ? CRYPTO_KEY_USE_ENCRYPT :
									 CRYPTO_KEY_USE_DECRYPT;
	if (!AesKeyMaterialOk(Key) || (Key.Usage & required) == 0U ||
		pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	// Encryption uses the virtual block primitive, allowing a hardware
	// subclass to accelerate ECB without duplicating the cipher loop.
	if (Alg == CRYPTO_CIPHER_ECB && bEncrypt != 0)
	{
		if ((Len & (AES_BLOCK - 1U)) != 0U)
		{
			return CRYPTO_STATUS_FAIL;
		}
		if (!AesOpBegin())
		{
			if (Len > 0U) memset(pOut, 0, Len);
			return CRYPTO_STATUS_BUSY;
		}
		CRYPTO_STATUS status = CRYPTO_STATUS_OK;
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			if (!AesEcbEncrypt(Key.Plain.pData, pIn + off, pOut + off))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
		}
		AesOpEnd();
		if (status != CRYPTO_STATUS_OK && Len > 0U)
		{
			memset(pOut, 0, Len);
		}
		return status;
	}

	// CTR and CBC encryption only require an AES encryption block.
	// Hardware subclasses reuse these modes through AesEcbEncrypt.
	if (Alg == CRYPTO_CIPHER_CTR ||
		(Alg == CRYPTO_CIPHER_CBC && bEncrypt != 0))
	{
		if (pIv == nullptr || IvLen != AES_BLOCK ||
			(Alg == CRYPTO_CIPHER_CBC && (Len & (AES_BLOCK - 1U)) != 0U))
		{
			return CRYPTO_STATUS_FAIL;
		}
		if (!AesOpBegin())
		{
			if (Len > 0U) memset(pOut, 0, Len);
			return CRYPTO_STATUS_BUSY;
		}
		uint8_t counter[AES_BLOCK];
		uint8_t block[AES_BLOCK];
		memcpy(counter, pIv, AES_BLOCK);
		CRYPTO_STATUS status = CRYPTO_STATUS_OK;
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			if (Alg == CRYPTO_CIPHER_CBC)
			{
				for (size_t i = 0; i < AES_BLOCK; i++)
				{
					block[i] = (uint8_t)(pIn[off + i] ^ counter[i]);
				}
				if (!AesEcbEncrypt(Key.Plain.pData, block, pOut + off))
				{
					status = CRYPTO_STATUS_FAIL;
					break;
				}
				memcpy(counter, pOut + off, AES_BLOCK);
			}
			else
			{
				if (!AesEcbEncrypt(Key.Plain.pData, counter, block))
				{
					status = CRYPTO_STATUS_FAIL;
					break;
				}
				size_t n = Len - off < AES_BLOCK ? Len - off : AES_BLOCK;
				for (size_t i = 0; i < n; i++)
				{
					pOut[off + i] = (uint8_t)(pIn[off + i] ^ block[i]);
				}
				for (int i = AES_BLOCK - 1; i >= 0; i--)
				{
					if (++counter[i] != 0U)
					{
						break;
					}
				}
			}
		}
		AesWipe(block, sizeof(block));
		AesWipe(counter, sizeof(counter));
		AesOpEnd();
		if (status != CRYPTO_STATUS_OK && Len > 0U)
		{
			memset(pOut, 0, Len);
		}
		return status;
	}

	// All encryption and CTR operations have already used the virtual ECB
	// primitive. Only software ECB/CBC decryption remains here.
	if (bEncrypt != 0 || (Alg != CRYPTO_CIPHER_ECB &&
		Alg != CRYPTO_CIPHER_CBC))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if ((Len & (AES_BLOCK - 1U)) != 0U ||
		(Alg == CRYPTO_CIPHER_CBC &&
		(pIv == nullptr || IvLen != AES_BLOCK)))
	{
		return CRYPTO_STATUS_FAIL;
	}

	if (Alg == CRYPTO_CIPHER_ECB)
	{
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			CryptoAesSoftDecryptBlock(Key.Plain.pData, pIn + off, pOut + off);
		}
	}
	else
	{
		uint8_t chain[AES_BLOCK], block[AES_BLOCK];
		memcpy(chain, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			memcpy(block, pIn + off, AES_BLOCK);
			CryptoAesSoftDecryptBlock(Key.Plain.pData, block, pOut + off);
			for (size_t i = 0; i < AES_BLOCK; i++)
			{
				pOut[off + i] ^= chain[i];
			}
			memcpy(chain, block, AES_BLOCK);
		}
		AesWipe(chain, sizeof(chain));
		AesWipe(block, sizeof(block));
	}
	return CRYPTO_STATUS_OK;
}

static void CmacSubkey(const uint8_t In[16], uint8_t Out[16])
{
	uint8_t carry = 0U;
	uint8_t msb = In[0] >> 7;
	for (int i = 15; i >= 0; i--)
	{
		uint8_t b = In[i];
		Out[i] = (uint8_t)((b << 1) | carry);
		carry = (b >> 7) & 1U;
	}
	if (msb != 0U) Out[15] ^= 0x87U;
}

// One software AES-128 ECB block for the inherited CMAC. A hardware subclass
// overrides this with its block primitive and brackets the whole CMAC in one
// acquisition through AesOpBegin/AesOpEnd.
bool CryptoSoftAes::AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16])
{
	CryptoAesSoftEncryptBlock(Key, In, Out);
	return true;
}

CRYPTO_STATUS CryptoSoftAes::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
								 const uint8_t *pMsg, size_t Len,
								 uint8_t *pMac, size_t MacLen)
{
	if (Alg != CRYPTO_MAC_CMAC || !AesKeyMaterialOk(Key) ||
		(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U || pMac == nullptr ||
		MacLen == 0U || MacLen > AES_BLOCK || (Len > 0U && pMsg == nullptr))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	// One acquisition for the whole CMAC: the block primitive runs between
	// AesOpBegin and AesOpEnd, so a hardware subclass holds its operation once
	// instead of once per block and no other user interleaves mid-MAC.
	if (!AesOpBegin())
	{
		// Fail closed on a refused bracket (hardware contention): the caller
		// must never read a stale tag, and BUSY marks the refusal retryable.
		memset(pMac, 0, MacLen);
		return CRYPTO_STATUS_BUSY;
	}

	const uint8_t *key = Key.Plain.pData;
	uint8_t zero[AES_BLOCK] = {};
	uint8_t L[AES_BLOCK] = {}, K1[AES_BLOCK] = {}, K2[AES_BLOCK] = {};
	uint8_t X[AES_BLOCK] = {};
	uint8_t Y[AES_BLOCK] = {};
	CRYPTO_STATUS st = AesEcbEncrypt(key, zero, L) ? CRYPTO_STATUS_OK
												 : CRYPTO_STATUS_FAIL;
	if (st == CRYPTO_STATUS_OK)
	{
		CmacSubkey(L, K1);
		CmacSubkey(K1, K2);

		size_t blocks = (Len + AES_BLOCK - 1U) / AES_BLOCK;
		bool complete = Len != 0U && (Len & (AES_BLOCK - 1U)) == 0U;
		if (blocks == 0U) blocks = 1U;

		for (size_t i = 0; i + 1U < blocks; i++)
		{
			for (size_t j = 0; j < AES_BLOCK; j++) Y[j] = X[j] ^ pMsg[i*AES_BLOCK+j];
			if (!AesEcbEncrypt(key, Y, X))
			{
				st = CRYPTO_STATUS_FAIL;
				break;
			}
		}

		if (st == CRYPTO_STATUS_OK)
		{
			uint8_t last[AES_BLOCK];
			size_t base = (blocks - 1U) * AES_BLOCK;
			if (complete)
			{
				for (size_t j = 0; j < AES_BLOCK; j++) last[j] = pMsg[base+j] ^ K1[j];
			}
			else
			{
				size_t rem = Len - base;
				for (size_t j = 0; j < AES_BLOCK; j++)
				{
					uint8_t value = j < rem ? pMsg[base+j] : (j == rem ? 0x80U : 0U);
					last[j] = value ^ K2[j];
				}
			}
			for (size_t j = 0; j < AES_BLOCK; j++) Y[j] = X[j] ^ last[j];
			uint8_t full[AES_BLOCK];
			if (!AesEcbEncrypt(key, Y, full))
			{
				st = CRYPTO_STATUS_FAIL;
			}
			else
			{
				memcpy(pMac, full, MacLen);
			}
			AesWipe(last, sizeof(last));
			AesWipe(full, sizeof(full));
		}
	}
	AesOpEnd();

	AesWipe(L, sizeof(L));
	AesWipe(K1, sizeof(K1));
	AesWipe(K2, sizeof(K2));
	AesWipe(X, sizeof(X));
	AesWipe(Y, sizeof(Y));
	if (st != CRYPTO_STATUS_OK)
	{
		memset(pMac, 0, MacLen);
	}
	return st;
}

// AES-CCM (RFC 3610). One AesOpBegin/AesOpEnd bracket wraps the whole seal
// or open, the CBC-MAC and CTR streams both running on the AesEcbEncrypt
// primitive so a hardware subclass provides every block operation.

// GHASH (SP 800-38D): constant-time bit-serial GF(2^128) multiply. No table,
// no data-dependent branch; the per-bit select is a mask. Software on every
// engine: a hardware AES subclass supplies the block cipher through
// AesEcbEncrypt while the field multiply stays here.
static void GhashMul(uint8_t Z[16], const uint8_t H[16])
{
	uint8_t v[16], z[16];
	memcpy(v, H, 16);
	memset(z, 0, 16);
	for (int i = 0; i < 128; i++)
	{
		uint8_t bit = (uint8_t)((Z[i >> 3] >> (7 - (i & 7))) & 1U);
		uint8_t mask = (uint8_t)(0U - bit);
		for (int j = 0; j < 16; j++)
		{
			z[j] ^= (uint8_t)(v[j] & mask);
		}
		uint8_t lsb = (uint8_t)(v[15] & 1U);
		for (int j = 15; j > 0; j--)
		{
			v[j] = (uint8_t)((v[j] >> 1) | (v[j - 1] << 7));
		}
		v[0] >>= 1;
		v[0] ^= (uint8_t)((0U - lsb) & 0xE1U);
	}
	memcpy(Z, z, 16);
	AesWipe(v, sizeof(v));
	AesWipe(z, sizeof(z));
}

// Absorb a byte string into the GHASH state, zero padded to full blocks.
static void GhashAbsorb(uint8_t Y[16], const uint8_t H[16],
						const uint8_t *pData, size_t Len)
{
	for (size_t idx = 0; idx < Len; idx += 16U)
	{
		size_t count = Len - idx < 16U ? Len - idx : 16U;
		for (size_t i = 0; i < count; i++)
		{
			Y[i] ^= pData[idx + i];
		}
		GhashMul(Y, H);
	}
}

static bool GcmParamsOk(size_t NonceLen, size_t TagLen)
{
	// The 96 bit IV form only (J0 = IV | 0x00000001). Tag lengths per
	// SP 800-38D: 96 to 128 bits, plus the 32 and 64 bit special sizes.
	return NonceLen == 12U &&
		   ((TagLen >= 12U && TagLen <= 16U) || TagLen == 8U || TagLen == 4U);
}

static bool CcmParamsOk(size_t NonceLen, size_t TagLen, size_t AadLen,
						size_t Len)
{
	size_t l = 15U - NonceLen;
	if (NonceLen < 7U || NonceLen > 13U ||
		TagLen < 4U || TagLen > 16U || (TagLen & 1U) != 0U ||
		AadLen >= 65280U)
	{
		return false;
	}
	// The message length must fit in the L length octets.
	if (l < (size_t)sizeof(size_t) && (Len >> (8U * l)) != 0U)
	{
		return false;
	}
	return true;
}

// T = CBC-MAC(B0 | encoded AAD | message), Tag = T xor E(K, A0). The full
// 16 byte masked block is written; the first TagLen bytes are the tag.
// Assumes validated parameters and an owned AesOpBegin bracket.
bool CryptoSoftAes::CcmMacTag(const uint8_t *pKey, const uint8_t *pNonce,
							  size_t NonceLen, const uint8_t *pAad,
							  size_t AadLen, const uint8_t *pMsg, size_t Len,
							  size_t TagLen, uint8_t Tag[16])
{
	size_t l = 15U - NonceLen;
	uint8_t x[AES_BLOCK], b[AES_BLOCK];
	bool ok;

	// B0: flags, nonce, message length.
	b[0] = (uint8_t)(((AadLen != 0U) ? 0x40U : 0U) |
					 (((TagLen - 2U) / 2U) << 3) | (l - 1U));
	memcpy(&b[1], pNonce, NonceLen);
	for (size_t i = 0; i < l; i++)
	{
		b[15U - i] = (uint8_t)(Len >> (8U * i));
	}
	ok = AesEcbEncrypt(pKey, b, x);

	// AAD: two byte length then the data, zero padded to the block.
	if (ok && AadLen != 0U)
	{
		memset(b, 0, sizeof(b));
		b[0] = (uint8_t)(AadLen >> 8);
		b[1] = (uint8_t)AadLen;
		size_t fill = AadLen < 14U ? AadLen : 14U;
		memcpy(&b[2], pAad, fill);
		size_t idx = fill;
		for (size_t i = 0; i < AES_BLOCK; i++) b[i] ^= x[i];
		ok = AesEcbEncrypt(pKey, b, x);
		while (ok && idx < AadLen)
		{
			memset(b, 0, sizeof(b));
			size_t count = AadLen - idx < AES_BLOCK ? AadLen - idx : AES_BLOCK;
			memcpy(b, &pAad[idx], count);
			idx += count;
			for (size_t i = 0; i < AES_BLOCK; i++) b[i] ^= x[i];
			ok = AesEcbEncrypt(pKey, b, x);
		}
	}

	// Message into the CBC-MAC.
	for (size_t idx = 0; ok && idx < Len; idx += AES_BLOCK)
	{
		memset(b, 0, sizeof(b));
		size_t count = Len - idx < AES_BLOCK ? Len - idx : AES_BLOCK;
		memcpy(b, &pMsg[idx], count);
		for (size_t i = 0; i < AES_BLOCK; i++) b[i] ^= x[i];
		ok = AesEcbEncrypt(pKey, b, x);
	}

	// Mask with the A0 keystream.
	if (ok)
	{
		memset(b, 0, sizeof(b));
		b[0] = (uint8_t)(l - 1U);
		memcpy(&b[1], pNonce, NonceLen);
		uint8_t s0[AES_BLOCK];
		ok = AesEcbEncrypt(pKey, b, s0);
		if (ok)
		{
			for (size_t i = 0; i < AES_BLOCK; i++)
			{
				Tag[i] = x[i] ^ s0[i];
			}
		}
		AesWipe(s0, sizeof(s0));
	}
	AesWipe(x, sizeof(x));
	AesWipe(b, sizeof(b));
	return ok;
}

CRYPTO_STATUS CryptoSoftAes::Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
								  const uint8_t *pNonce, size_t NonceLen,
								  const uint8_t *pAad, size_t AadLen,
								  const uint8_t *pMsg, size_t Len,
								  uint8_t *pOut, uint8_t *pTag, size_t TagLen)
{
	if (Alg == CRYPTO_AEAD_AES_GCM)
	{
		if (!AesKeyMaterialOk(Key) ||
			(Key.Usage & CRYPTO_KEY_USE_ENCRYPT) == 0U ||
			pNonce == nullptr || pTag == nullptr ||
			(pMsg == nullptr && Len != 0U) || (pOut == nullptr && Len != 0U) ||
			(pAad == nullptr && AadLen != 0U) ||
			!GcmParamsOk(NonceLen, TagLen))
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}
		if (!AesOpBegin())
		{
			// Refused bracket: fail closed and retryable.
			if (Len > 0U) memset(pOut, 0, Len);
			memset(pTag, 0, TagLen);
			return CRYPTO_STATUS_BUSY;
		}
		const uint8_t *key = Key.Plain.pData;
		uint8_t h[16] = {}, j0[16], ej0[16], a[16], stream[16], y[16] = {};
		bool ok = AesEcbEncrypt(key, h, h);
		memcpy(j0, pNonce, 12U);
		j0[12] = 0U; j0[13] = 0U; j0[14] = 0U; j0[15] = 1U;
		ok = ok && AesEcbEncrypt(key, j0, ej0);
		for (size_t idx = 0, ctr = 2; ok && idx < Len; idx += 16U, ctr++)
		{
			memcpy(a, j0, 12U);
			a[12] = (uint8_t)(ctr >> 24);
			a[13] = (uint8_t)(ctr >> 16);
			a[14] = (uint8_t)(ctr >> 8);
			a[15] = (uint8_t)ctr;
			ok = AesEcbEncrypt(key, a, stream);
			size_t count = Len - idx < 16U ? Len - idx : 16U;
			for (size_t i = 0; ok && i < count; i++)
			{
				pOut[idx + i] = pMsg[idx + i] ^ stream[i];
			}
		}
		AesOpEnd();
		if (ok)
		{
			GhashAbsorb(y, h, pAad, AadLen);
			GhashAbsorb(y, h, pOut, Len);
			uint64_t abits = (uint64_t)AadLen * 8U;
			uint64_t cbits = (uint64_t)Len * 8U;
			uint8_t lens[16];
			for (int i = 0; i < 8; i++)
			{
				lens[i] = (uint8_t)(abits >> (56 - 8 * i));
				lens[8 + i] = (uint8_t)(cbits >> (56 - 8 * i));
			}
			GhashAbsorb(y, h, lens, sizeof(lens));
			for (size_t i = 0; i < TagLen; i++)
			{
				pTag[i] = y[i] ^ ej0[i];
			}
		}
		else
		{
			if (Len > 0U) memset(pOut, 0, Len);
			memset(pTag, 0, TagLen);
		}
		AesWipe(h, sizeof(h));
		AesWipe(j0, sizeof(j0));
		AesWipe(ej0, sizeof(ej0));
		AesWipe(a, sizeof(a));
		AesWipe(stream, sizeof(stream));
		AesWipe(y, sizeof(y));
		return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
	if (Alg != CRYPTO_AEAD_AES_CCM || !AesKeyMaterialOk(Key) ||
		(Key.Usage & CRYPTO_KEY_USE_ENCRYPT) == 0U ||
		pNonce == nullptr || pTag == nullptr ||
		(pMsg == nullptr && Len != 0U) || (pOut == nullptr && Len != 0U) ||
		(pAad == nullptr && AadLen != 0U) ||
		!CcmParamsOk(NonceLen, TagLen, AadLen, Len))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!AesOpBegin())
	{
		// Refused bracket: fail closed and retryable.
		if (Len > 0U) memset(pOut, 0, Len);
		memset(pTag, 0, TagLen);
		return CRYPTO_STATUS_BUSY;
	}

	const uint8_t *key = Key.Plain.pData;
	size_t l = 15U - NonceLen;
	uint8_t tag[AES_BLOCK];
	uint8_t a[AES_BLOCK], stream[AES_BLOCK];
	bool ok = CcmMacTag(key, pNonce, NonceLen, pAad, AadLen, pMsg, Len,
						TagLen, tag);

	// Payload: CTR with counters A1 onward.
	for (size_t idx = 0, ctr = 1; ok && idx < Len; idx += AES_BLOCK, ctr++)
	{
		memset(a, 0, sizeof(a));
		a[0] = (uint8_t)(l - 1U);
		memcpy(&a[1], pNonce, NonceLen);
		for (size_t i = 0; i < l; i++)
		{
			a[15U - i] = (uint8_t)(ctr >> (8U * i));
		}
		ok = AesEcbEncrypt(key, a, stream);
		size_t count = Len - idx < AES_BLOCK ? Len - idx : AES_BLOCK;
		for (size_t i = 0; ok && i < count; i++)
		{
			pOut[idx + i] = pMsg[idx + i] ^ stream[i];
		}
	}
	AesOpEnd();

	if (ok)
	{
		memcpy(pTag, tag, TagLen);
	}
	else
	{
		if (Len > 0U) memset(pOut, 0, Len);
		memset(pTag, 0, TagLen);
	}
	AesWipe(tag, sizeof(tag));
	AesWipe(a, sizeof(a));
	AesWipe(stream, sizeof(stream));
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS CryptoSoftAes::Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
								  const uint8_t *pNonce, size_t NonceLen,
								  const uint8_t *pAad, size_t AadLen,
								  const uint8_t *pMsg, size_t Len,
								  uint8_t *pOut, const uint8_t *pTag,
								  size_t TagLen)
{
	if (Alg == CRYPTO_AEAD_AES_GCM)
	{
		if (!AesKeyMaterialOk(Key) ||
			(Key.Usage & CRYPTO_KEY_USE_DECRYPT) == 0U ||
			pNonce == nullptr || pTag == nullptr ||
			(pMsg == nullptr && Len != 0U) || (pOut == nullptr && Len != 0U) ||
			(pAad == nullptr && AadLen != 0U) ||
			!GcmParamsOk(NonceLen, TagLen))
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}
		if (!AesOpBegin())
		{
			// Refused bracket: fail closed and retryable.
			if (Len > 0U) memset(pOut, 0, Len);
			return CRYPTO_STATUS_BUSY;
		}

		// Verify first over the received ciphertext, decrypt only on an
		// authentic tag, constant-time compare.
		const uint8_t *key = Key.Plain.pData;
		uint8_t h[16] = {}, j0[16], ej0[16], a[16], stream[16], y[16] = {};
		bool ok = AesEcbEncrypt(key, h, h);
		memcpy(j0, pNonce, 12U);
		j0[12] = 0U; j0[13] = 0U; j0[14] = 0U; j0[15] = 1U;
		ok = ok && AesEcbEncrypt(key, j0, ej0);
		if (ok)
		{
			GhashAbsorb(y, h, pAad, AadLen);
			GhashAbsorb(y, h, pMsg, Len);
			uint64_t abits = (uint64_t)AadLen * 8U;
			uint64_t cbits = (uint64_t)Len * 8U;
			uint8_t lens[16];
			for (int i = 0; i < 8; i++)
			{
				lens[i] = (uint8_t)(abits >> (56 - 8 * i));
				lens[8 + i] = (uint8_t)(cbits >> (56 - 8 * i));
			}
			GhashAbsorb(y, h, lens, sizeof(lens));
			uint8_t diff = 0U;
			for (size_t i = 0; i < TagLen; i++)
			{
				diff |= (uint8_t)((y[i] ^ ej0[i]) ^ pTag[i]);
			}
			ok = diff == 0U;
		}
		for (size_t idx = 0, ctr = 2; ok && idx < Len; idx += 16U, ctr++)
		{
			memcpy(a, j0, 12U);
			a[12] = (uint8_t)(ctr >> 24);
			a[13] = (uint8_t)(ctr >> 16);
			a[14] = (uint8_t)(ctr >> 8);
			a[15] = (uint8_t)ctr;
			ok = AesEcbEncrypt(key, a, stream);
			size_t count = Len - idx < 16U ? Len - idx : 16U;
			for (size_t i = 0; ok && i < count; i++)
			{
				pOut[idx + i] = pMsg[idx + i] ^ stream[i];
			}
		}
		AesOpEnd();
		if (!ok && Len > 0U)
		{
			memset(pOut, 0, Len);
		}
		AesWipe(h, sizeof(h));
		AesWipe(j0, sizeof(j0));
		AesWipe(ej0, sizeof(ej0));
		AesWipe(a, sizeof(a));
		AesWipe(stream, sizeof(stream));
		AesWipe(y, sizeof(y));
		return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
	if (Alg != CRYPTO_AEAD_AES_CCM || !AesKeyMaterialOk(Key) ||
		(Key.Usage & CRYPTO_KEY_USE_DECRYPT) == 0U ||
		pNonce == nullptr || pTag == nullptr ||
		(pMsg == nullptr && Len != 0U) || (pOut == nullptr && Len != 0U) ||
		(pAad == nullptr && AadLen != 0U) ||
		!CcmParamsOk(NonceLen, TagLen, AadLen, Len))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!AesOpBegin())
	{
		// Refused bracket: fail closed and retryable.
		if (Len > 0U) memset(pOut, 0, Len);
		return CRYPTO_STATUS_BUSY;
	}

	const uint8_t *key = Key.Plain.pData;
	size_t l = 15U - NonceLen;
	uint8_t a[AES_BLOCK], stream[AES_BLOCK];
	bool ok = true;

	// Recover the plaintext with the CTR stream, counters A1 onward.
	for (size_t idx = 0, ctr = 1; ok && idx < Len; idx += AES_BLOCK, ctr++)
	{
		memset(a, 0, sizeof(a));
		a[0] = (uint8_t)(l - 1U);
		memcpy(&a[1], pNonce, NonceLen);
		for (size_t i = 0; i < l; i++)
		{
			a[15U - i] = (uint8_t)(ctr >> (8U * i));
		}
		ok = AesEcbEncrypt(key, a, stream);
		size_t count = Len - idx < AES_BLOCK ? Len - idx : AES_BLOCK;
		for (size_t i = 0; ok && i < count; i++)
		{
			pOut[idx + i] = pMsg[idx + i] ^ stream[i];
		}
	}

	// Recompute the seal tag over the recovered plaintext and compare in
	// constant time.
	uint8_t tag[AES_BLOCK];
	if (ok)
	{
		ok = CcmMacTag(key, pNonce, NonceLen, pAad, AadLen, pOut, Len,
					   TagLen, tag);
	}
	AesOpEnd();

	if (ok)
	{
		uint8_t diff = 0U;
		for (size_t i = 0; i < TagLen; i++)
		{
			diff |= (uint8_t)(tag[i] ^ pTag[i]);
		}
		ok = diff == 0U;
	}
	AesWipe(tag, sizeof(tag));
	AesWipe(a, sizeof(a));
	AesWipe(stream, sizeof(stream));
	if (!ok && Len > 0U)
	{
		memset(pOut, 0, Len);
	}
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

int CryptoSoftAes::SelfTest()
{
	static const uint8_t aesKey[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
	static const uint8_t aesIn[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
	static const uint8_t aesExp[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a };
	static const uint8_t cmacKey[16] = {
		0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
		0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
	static const uint8_t cmacExp[16] = {
		0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,
		0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46 };

	CryptoKey cipherKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
						CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	cipherKey.Plain.pData = aesKey;
	cipherKey.Plain.Len = sizeof(aesKey);
	uint8_t out[16];
	if (Cipher(CRYPTO_CIPHER_ECB, 1, cipherKey, nullptr, 0,
			   aesIn, sizeof(aesIn), out) != CRYPTO_STATUS_OK ||
		memcmp(out, aesExp, sizeof(out)) != 0)
	{
		return -1;
	}

	if (Cipher(CRYPTO_CIPHER_ECB, 0, cipherKey, nullptr, 0,
			   aesExp, sizeof(aesExp), out) != CRYPTO_STATUS_OK ||
		memcmp(out, aesIn, sizeof(out)) != 0)
	{
		return -3;
	}

	CryptoKey macKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_SIGN, {}};
	macKey.Plain.pData = cmacKey;
	macKey.Plain.Len = sizeof(cmacKey);
	uint8_t mac[16];
	if (Mac(CRYPTO_MAC_CMAC, macKey, nullptr, 0, mac, sizeof(mac)) !=
		CRYPTO_STATUS_OK || memcmp(mac, cmacExp, sizeof(mac)) != 0)
	{
		return -2;
	}
	return 0;
}

CryptoSoftAes *CryptoSoftAesCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoSoftAes) ||
		((uintptr_t)pMem & (alignof(CryptoSoftAes) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoSoftAes *p = new (pMem) CryptoSoftAes();
	p->Enable();
	return p;
}
