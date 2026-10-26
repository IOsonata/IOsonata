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
void CryptoAesSoftEncryptBlock256(const uint8_t Key[32], const uint8_t In[16], uint8_t Out[16]);
void CryptoAesSoftDecryptBlock256(const uint8_t Key[32], const uint8_t In[16], uint8_t Out[16]);

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
	bool wide = Key.Type == CRYPTO_KEY_AES_256;
	bool material = AesKeyMaterialOk(Key) || (wide && Key.Loc == CRYPTO_KEY_LOC_PLAIN &&
		Key.Plain.pData != nullptr && Key.Plain.Len == 32U);
	bool encrypt = bEncrypt != 0 || Alg == CRYPTO_CIPHER_CTR;
	bool acquire = encrypt && !wide;
	uint32_t required = bEncrypt != 0 ? CRYPTO_KEY_USE_ENCRYPT : CRYPTO_KEY_USE_DECRYPT;
	CRYPTO_STATUS status = CRYPTO_STATUS_OK;
	uint8_t chain[AES_BLOCK];
	uint8_t block[AES_BLOCK];
	uint8_t saved[AES_BLOCK];
	size_t off;
	size_t count;
	size_t i;
	int carry;

	if (!material || (Key.Usage & required) == 0U ||
		pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (Alg != CRYPTO_CIPHER_ECB && Alg != CRYPTO_CIPHER_CBC &&
		Alg != CRYPTO_CIPHER_CTR)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if ((Alg != CRYPTO_CIPHER_CTR && (Len & (AES_BLOCK - 1U)) != 0U) ||
		(Alg != CRYPTO_CIPHER_ECB && (pIv == nullptr || IvLen != AES_BLOCK)))
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Only AES-128 encryption uses the hardware hooks. CTR uses encryption
	// in both directions; AES-256 and ECB/CBC decryption stay in software.
	if (acquire && !AesOpBegin())
	{
		if (Len > 0U)
		{
			memset(pOut, 0, Len);
		}
		return CRYPTO_STATUS_BUSY;
	}
	if (Alg != CRYPTO_CIPHER_ECB)
	{
		memcpy(chain, pIv, AES_BLOCK);
	}
	for (off = 0; off < Len; off += count)
	{
		count = Len - off < AES_BLOCK ? Len - off : AES_BLOCK;
		if (Alg == CRYPTO_CIPHER_CTR)
		{
			memcpy(block, chain, AES_BLOCK);
		}
		else
		{
			memcpy(block, pIn + off, AES_BLOCK);
			if (Alg == CRYPTO_CIPHER_CBC)
			{
				if (encrypt)
				{
					for (i = 0; i < AES_BLOCK; i++)
					{
						block[i] ^= chain[i];
					}
				}
				else
				{
					memcpy(saved, block, AES_BLOCK);
				}
			}
		}

		if (!encrypt)
		{
			if (wide)
			{
				CryptoAesSoftDecryptBlock256(Key.Plain.pData, block, block);
			}
			else
			{
				CryptoAesSoftDecryptBlock(Key.Plain.pData, block, block);
			}
		}
		else if (wide)
		{
			CryptoAesSoftEncryptBlock256(Key.Plain.pData, block, block);
		}
		else if (!AesEcbEncrypt(Key.Plain.pData, block, block))
		{
			status = CRYPTO_STATUS_FAIL;
			break;
		}

		if (Alg == CRYPTO_CIPHER_CTR)
		{
			for (i = 0; i < count; i++)
			{
				pOut[off + i] = pIn[off + i] ^ block[i];
			}
			for (carry = AES_BLOCK - 1; carry >= 0; carry--)
			{
				if (++chain[carry] != 0U)
				{
					break;
				}
			}
		}
		else
		{
			if (Alg == CRYPTO_CIPHER_CBC)
			{
				if (encrypt)
				{
					memcpy(chain, block, AES_BLOCK);
				}
				else
				{
					for (i = 0; i < AES_BLOCK; i++)
					{
						block[i] ^= chain[i];
					}
					memcpy(chain, saved, AES_BLOCK);
				}
			}
			memcpy(pOut + off, block, AES_BLOCK);
		}
	}
	AesWipe(chain, sizeof(chain));
	AesWipe(block, sizeof(block));
	AesWipe(saved, sizeof(saved));
	if (acquire)
	{
		AesOpEnd();
	}
	if (status != CRYPTO_STATUS_OK && Len > 0U)
	{
		memset(pOut, 0, Len);
	}
	return status;
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
	if (msb != 0U)
		Out[15] ^= 0x87U;
}

// One software AES-128 ECB block for the inherited CMAC. A hardware subclass
// overrides this with its block primitive and brackets the whole CMAC in one
// acquisition through AesOpBegin/AesOpEnd.
bool CryptoSoftAes::AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16])
{
	CryptoAesSoftEncryptBlock(Key, In, Out);
	return true;
}

CRYPTO_STATUS CryptoAesModes::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
								 const uint8_t *pMsg, size_t Len,
								 uint8_t *pMac, size_t MacLen)
{
	const uint8_t *key = Key.Plain.pData;
	uint8_t subkey[AES_BLOCK] = {};
	uint8_t chain[AES_BLOCK] = {};
	uint8_t block[AES_BLOCK];
	CRYPTO_STATUS st;
	size_t off = 0U;
	size_t count;
	size_t i;
	bool last;

	if (Alg != CRYPTO_MAC_CMAC || !AesKeyMaterialOk(Key) ||
		(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U || pMac == nullptr ||
		MacLen == 0U || MacLen > AES_BLOCK || (Len > 0U && pMsg == nullptr))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	// Hold the hardware once for subkey generation and every message block.
	if (!AesOpBegin())
	{
		memset(pMac, 0, MacLen);
		return CRYPTO_STATUS_BUSY;
	}
	st = AesEcbEncrypt(key, subkey, subkey) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	if (st == CRYPTO_STATUS_OK)
	{
		CmacSubkey(subkey, subkey);
		if (Len == 0U || (Len & (AES_BLOCK - 1U)) != 0U)
		{
			CmacSubkey(subkey, subkey);
		}
		// Process the final block in the same loop, including an empty message.
		do
		{
			count = Len - off < AES_BLOCK ? Len - off : AES_BLOCK;
			last = Len - off <= AES_BLOCK;
			memset(block, 0, sizeof(block));
			if (count != 0U)
			{
				memcpy(block, pMsg + off, count);
			}
			if (count < AES_BLOCK)
			{
				block[count] = 0x80U;
			}
			for (i = 0U; i < AES_BLOCK; i++)
			{
				block[i] ^= chain[i] ^ (last ? subkey[i] : 0U);
			}
			if (!AesEcbEncrypt(key, block, chain))
			{
				st = CRYPTO_STATUS_FAIL;
				break;
			}
			off += count;
		} while (!last);
	}
	AesOpEnd();
	if (st == CRYPTO_STATUS_OK)
	{
		memcpy(pMac, chain, MacLen);
	}
	else
	{
		memset(pMac, 0, MacLen);
	}
	AesWipe(subkey, sizeof(subkey));
	AesWipe(chain, sizeof(chain));
	AesWipe(block, sizeof(block));
	return st;
}

// AES-CCM (RFC 3610). One AesOpBegin/AesOpEnd bracket wraps the whole seal
// or open, the CBC-MAC and CTR streams both running on the AesEcbEncrypt
// primitive so a hardware subclass provides every block operation.

// GHASH (SP 800-38D): fixed 128-step multiply using four 32-bit words.
// Masked selection and reduction need no secret-indexed table or branch.
// Explicit big-endian loads/stores also support unaligned byte buffers.
static void GhashMul(uint8_t Z[16], const uint8_t H[16])
{
	uint32_t v[4], z[4] = {};
	uint32_t mask, reduce;
	unsigned i;

	for (i = 0U; i < 4U; i++)
	{
		v[i] = ((uint32_t)H[4U * i] << 24) |
			((uint32_t)H[4U * i + 1U] << 16) |
			((uint32_t)H[4U * i + 2U] << 8) | H[4U * i + 3U];
	}
	for (i = 0U; i < 128U; i++)
	{
		mask = 0U - ((Z[i >> 3] >> (7U - (i & 7U))) & 1U);
		z[0] ^= v[0] & mask;
		z[1] ^= v[1] & mask;
		z[2] ^= v[2] & mask;
		z[3] ^= v[3] & mask;
		reduce = (0U - (v[3] & 1U)) & 0xE1000000U;
		v[3] = (v[3] >> 1) | (v[2] << 31);
		v[2] = (v[2] >> 1) | (v[1] << 31);
		v[1] = (v[1] >> 1) | (v[0] << 31);
		v[0] = (v[0] >> 1) ^ reduce;
	}
	for (i = 0U; i < 4U; i++)
	{
		Z[4U * i] = (uint8_t)(z[i] >> 24);
		Z[4U * i + 1U] = (uint8_t)(z[i] >> 16);
		Z[4U * i + 2U] = (uint8_t)(z[i] >> 8);
		Z[4U * i + 3U] = (uint8_t)z[i];
	}
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
bool CryptoAesModes::CcmMacTag(const uint8_t *pKey, const uint8_t *pNonce,
							  size_t NonceLen, const uint8_t *pAad,
							  size_t AadLen, const uint8_t *pMsg, size_t Len,
							  size_t TagLen, uint8_t Tag[16])
{
	size_t l = 15U - NonceLen;
	size_t value = Len;
	uint8_t x[AES_BLOCK], b[AES_BLOCK];
	const uint8_t *data;
	size_t remaining, prefix, count, i;
	unsigned part;
	bool ok;

	// B0: flags, nonce, message length.
	b[0] = (uint8_t)(((AadLen != 0U) ? 0x40U : 0U) |
					 (((TagLen - 2U) / 2U) << 3) | (l - 1U));
	memcpy(&b[1], pNonce, NonceLen);
	// L may exceed sizeof(size_t); shift one byte at a time to zero extend.
	for (i = 0; i < l; i++)
	{
		b[15U - i] = (uint8_t)value;
		value >>= 8U;
	}
	ok = AesEcbEncrypt(pKey, b, x);

	// AAD and message have independent block padding, but share the same
	// CBC-MAC loop. Only the AAD's first block carries a two-byte length.
	for (part = 0U; ok && part < 2U; part++)
	{
		data = part == 0U ? pAad : pMsg;
		remaining = part == 0U ? AadLen : Len;
		prefix = part == 0U ? 2U : 0U;

		while (ok && remaining != 0U)
		{
			memset(b, 0, sizeof(b));
			if (prefix != 0U)
			{
				b[0] = (uint8_t)(AadLen >> 8);
				b[1] = (uint8_t)AadLen;
			}
			count = remaining < AES_BLOCK - prefix ? remaining : AES_BLOCK - prefix;
			memcpy(b + prefix, data, count);
			data += count;
			remaining -= count;
			prefix = 0U;
			for (i = 0; i < AES_BLOCK; i++)
			{
				b[i] ^= x[i];
			}
			ok = AesEcbEncrypt(pKey, b, x);
		}
	}

	// Mask with the A0 keystream.
	if (ok)
	{
		memset(b, 0, sizeof(b));
		b[0] = (uint8_t)(l - 1U);
		memcpy(&b[1], pNonce, NonceLen);
		ok = AesEcbEncrypt(pKey, b, b);
		if (ok)
		{
			for (i = 0; i < AES_BLOCK; i++)
			{
				Tag[i] = x[i] ^ b[i];
			}
		}
	}
	AesWipe(x, sizeof(x));
	AesWipe(b, sizeof(b));
	return ok;
}

// Seal and open use the same counter stream and tag construction. GCM open
// authenticates ciphertext before any plaintext is produced; CCM open wipes
// the recovered plaintext if its recomputed tag does not match.
static void GcmTag(const uint8_t H[16], const uint8_t Mask[16],
	const uint8_t *pAad, size_t AadLen, const uint8_t *pMsg, size_t Len,
	uint8_t Tag[16])
{
	uint8_t lens[16];
	uint64_t abits = (uint64_t)AadLen * 8U;
	uint64_t cbits = (uint64_t)Len * 8U;
	unsigned i;

	memset(Tag, 0, 16U);
	GhashAbsorb(Tag, H, pAad, AadLen);
	GhashAbsorb(Tag, H, pMsg, Len);
	for (i = 0U; i < 8U; i++)
	{
		lens[7U - i] = (uint8_t)abits;
		lens[15U - i] = (uint8_t)cbits;
		abits >>= 8U;
		cbits >>= 8U;
	}
	GhashAbsorb(Tag, H, lens, sizeof(lens));
	for (i = 0U; i < 16U; i++)
	{
		Tag[i] ^= Mask[i];
	}
}

static bool AesTagEqual(const uint8_t *pLeft, const uint8_t *pRight, size_t Len)
{
	uint8_t diff = 0U;
	size_t i;

	for (i = 0U; i < Len; i++)
	{
		diff |= pLeft[i] ^ pRight[i];
	}
	return diff == 0U;
}

CRYPTO_STATUS CryptoAesModes::Aead(bool Encrypt, CRYPTO_AEAD_ALG Alg,
	const CryptoKey &Key, const uint8_t *pNonce, size_t NonceLen,
	const uint8_t *pAad, size_t AadLen, const uint8_t *pMsg, size_t Len,
	uint8_t *pOut, const uint8_t *pTag, uint8_t *pNewTag, size_t TagLen)
{
	bool gcm = Alg == CRYPTO_AEAD_AES_GCM;
	bool ok;
	uint32_t usage = Encrypt ? CRYPTO_KEY_USE_ENCRYPT : CRYPTO_KEY_USE_DECRYPT;
	const uint8_t *key = Key.Plain.pData;
	uint8_t tag[16], a[16] = {}, stream[16], h[16] = {}, mask[16];
	size_t l, idx, ctr, count, i, value;

	if (!AesKeyMaterialOk(Key) || (Key.Usage & usage) == 0U ||
		pNonce == nullptr || (Encrypt ? pNewTag == nullptr : pTag == nullptr) ||
		(pMsg == nullptr && Len != 0U) || (pOut == nullptr && Len != 0U) ||
		(pAad == nullptr && AadLen != 0U) ||
		(gcm ? !GcmParamsOk(NonceLen, TagLen) :
		 (Alg != CRYPTO_AEAD_AES_CCM || !CcmParamsOk(NonceLen, TagLen, AadLen, Len))))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!AesOpBegin())
	{
		if (Len > 0U)
			memset(pOut, 0, Len);
		if (Encrypt)
			memset(pNewTag, 0, TagLen);
		return CRYPTO_STATUS_BUSY;
	}

	l = gcm ? 4U : 15U - NonceLen;
	if (gcm)
	{
		memcpy(a, pNonce, NonceLen);
		a[15] = 1U;
		ok = AesEcbEncrypt(key, h, h) && AesEcbEncrypt(key, a, mask);
		if (ok && !Encrypt)
		{
			GcmTag(h, mask, pAad, AadLen, pMsg, Len, tag);
			ok = AesTagEqual(tag, pTag, TagLen);
		}
	}
	else
	{
		a[0] = (uint8_t)(l - 1U);
		memcpy(a + 1U, pNonce, NonceLen);
		ok = !Encrypt || CcmMacTag(key, pNonce, NonceLen, pAad, AadLen,
			pMsg, Len, TagLen, tag);
	}

	for (idx = 0U, ctr = gcm ? 2U : 1U; ok && idx < Len; idx += count, ctr++)
	{
		value = ctr;
		for (i = 0U; i < l; i++)
		{
			a[15U - i] = (uint8_t)value;
			value >>= 8U;
		}
		ok = AesEcbEncrypt(key, a, stream);
		count = Len - idx < 16U ? Len - idx : 16U;
		for (i = 0U; ok && i < count; i++)
		{
			pOut[idx + i] = pMsg[idx + i] ^ stream[i];
		}
	}
	if (ok && !gcm && !Encrypt)
	{
		ok = CcmMacTag(key, pNonce, NonceLen, pAad, AadLen, pOut, Len, TagLen, tag);
	}
	AesOpEnd();
	if (ok && gcm && Encrypt)
	{
		GcmTag(h, mask, pAad, AadLen, pOut, Len, tag);
	}
	if (ok && !gcm && !Encrypt)
	{
		ok = AesTagEqual(tag, pTag, TagLen);
	}
	if (ok && Encrypt)
	{
		memcpy(pNewTag, tag, TagLen);
	}
	if (!ok)
	{
		if (Len > 0U)
			memset(pOut, 0, Len);
		if (Encrypt)
			memset(pNewTag, 0, TagLen);
	}
	AesWipe(tag, sizeof(tag));
	AesWipe(a, sizeof(a));
	AesWipe(stream, sizeof(stream));
	AesWipe(h, sizeof(h));
	AesWipe(mask, sizeof(mask));
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS CryptoAesModes::Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
	const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
	const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen)
{
	return Aead(true, Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len,
		pOut, nullptr, pTag, TagLen);
}

CRYPTO_STATUS CryptoAesModes::Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
	const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
	const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen)
{
	return Aead(false, Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len,
		pOut, pTag, nullptr, TagLen);
}

int CryptoAesModes::SelfTest()
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
