/**-------------------------------------------------------------------------
@file	crypto_stm32h7.cpp
@brief	STM32H7 CRYP and HASH register drivers; no HAL dependency.
@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026
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
#include <atomic>
#include <new>
#include <string.h>
#include "stm32.h"
#include "crypto_stm32h7.h"

#ifndef CRYPTO_STM32H7_POLL_MAX
#define CRYPTO_STM32H7_POLL_MAX 1000000U
#endif

static std::atomic_flag s_AesBusy = ATOMIC_FLAG_INIT;
static std::atomic_flag s_HashBusy = ATOMIC_FLAG_INIT;

static uint32_t LoadBe32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
		   ((uint32_t)p[2] << 8) | p[3];
}

static void StoreBe32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

static void CrypReset()
{
	RCC->AHB2RSTR |= RCC_AHB2RSTR_CRYPRST;
	RCC->AHB2RSTR &= ~(uint32_t)RCC_AHB2RSTR_CRYPRST;
}

static bool CrypWait(uint32_t Mask, uint32_t Value)
{
	for (uint32_t i = 0; i < CRYPTO_STM32H7_POLL_MAX; i++)
	{
		if ((CRYP->SR & Mask) == Value) return true;
	}
	return false;
}

// DATATYPE=0: explicit big-endian loads/stores also allow unaligned buffers.
static bool CrypStart(const uint8_t *pKey, size_t KeyLen, bool Decrypt)
{
	const uint32_t keySize = KeyLen == 32U ? CRYP_CR_KEYSIZE_1 : 0U;
	CRYP->CR = keySize | CRYP_CR_FFLUSH;
	if (KeyLen == 32U)
	{
		CRYP->K0LR = LoadBe32(pKey); CRYP->K0RR = LoadBe32(pKey + 4);
		CRYP->K1LR = LoadBe32(pKey + 8); CRYP->K1RR = LoadBe32(pKey + 12);
		pKey += 16;
	}
	CRYP->K2LR = LoadBe32(pKey); CRYP->K2RR = LoadBe32(pKey + 4);
	CRYP->K3LR = LoadBe32(pKey + 8); CRYP->K3RR = LoadBe32(pKey + 12);
	if (Decrypt)
	{
		CRYP->CR = keySize | CRYP_CR_ALGOMODE_AES_KEY | CRYP_CR_CRYPEN;
		if (!CrypWait(CRYP_SR_BUSY, 0U)) return false;
	}
	CRYP->CR = keySize | CRYP_CR_ALGOMODE_AES_ECB |
		(Decrypt ? CRYP_CR_ALGODIR : 0U) | CRYP_CR_CRYPEN;
	return true;
}

static bool CrypBlock(const uint8_t *pIn, uint8_t *pOut)
{
	for (size_t i = 0; i < 16U; i += 4U)
	{
		if (!CrypWait(CRYP_SR_IFNF, CRYP_SR_IFNF)) return false;
		CRYP->DIN = LoadBe32(pIn + i);
	}
	for (size_t i = 0; i < 16U; i += 4U)
	{
		if (!CrypWait(CRYP_SR_OFNE, CRYP_SR_OFNE)) return false;
		StoreBe32(pOut + i, CRYP->DOUT);
	}
	return true;
}

bool CryptoStm32H7Aes::Enable()
{
	if (s_AesBusy.test_and_set()) return false;
	RCC->AHB2ENR |= RCC_AHB2ENR_CRYPEN;
	(void)RCC->AHB2ENR;
	CrypReset();
	vbValid = true;
	s_AesBusy.clear();
	return true;
}

void CryptoStm32H7Aes::Disable()
{
	if (s_AesBusy.test_and_set()) return;
	CrypReset();
	vbValid = false;
	// Clock remains enabled: another engine object may still use this block.
	s_AesBusy.clear();
}

void CryptoStm32H7Aes::Reset()
{
	if (s_AesBusy.test_and_set()) return;
	if (vbValid) CrypReset();
	s_AesBusy.clear();
}

bool CryptoStm32H7Aes::AesOpBegin()
{
	return vbValid && !s_AesBusy.test_and_set();
}

void CryptoStm32H7Aes::AesOpEnd()
{
	// Reset clears key, IV, FIFO and internal key schedule on every exit.
	CrypReset();
	s_AesBusy.clear();
}

bool CryptoStm32H7Aes::AesEcbEncrypt(const uint8_t Key[16],
								  const uint8_t In[16], uint8_t Out[16])
{
	CRYP->CR = 0U;
	return CrypStart(Key, 16U, false) && CrypBlock(In, Out);
}

CRYPTO_STATUS CryptoStm32H7Aes::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
		const CryptoKey &Key, const uint8_t *pIv, size_t IvLen,
		const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	const uint32_t usage = bEncrypt ? CRYPTO_KEY_USE_ENCRYPT : CRYPTO_KEY_USE_DECRYPT;
	if (Key.Loc != CRYPTO_KEY_LOC_PLAIN || Key.Plain.pData == nullptr ||
		!((Key.Type == CRYPTO_KEY_AES_128 && Key.Plain.Len == 16U) ||
		  (Key.Type == CRYPTO_KEY_AES_256 && Key.Plain.Len == 32U)) ||
		(Key.Usage & usage) == 0U || pIn == nullptr || pOut == nullptr ||
		(Alg != CRYPTO_CIPHER_ECB && Alg != CRYPTO_CIPHER_CBC && Alg != CRYPTO_CIPHER_CTR))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if ((Alg != CRYPTO_CIPHER_CTR && (Len & 15U) != 0U) ||
		(Alg != CRYPTO_CIPHER_ECB && (pIv == nullptr || IvLen != 16U)))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!vbValid)
	{
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_FAIL;
	}
	if (!AesOpBegin())
	{
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_BUSY;
	}
	uint8_t chain[16] = {}, block[16] = {}, result[16] = {};
	if (Alg != CRYPTO_CIPHER_ECB) memcpy(chain, pIv, sizeof(chain));
	bool ok = CrypStart(Key.Plain.pData, Key.Plain.Len, !bEncrypt && Alg != CRYPTO_CIPHER_CTR);
	for (size_t off = 0; ok && off < Len; )
	{
		const size_t count = Len - off < 16U ? Len - off : 16U;
		if (Alg == CRYPTO_CIPHER_CTR)
		{
			ok = CrypBlock(chain, result);
			if (ok)
			{
				for (size_t j = 0; j < count; j++) pOut[off + j] = pIn[off + j] ^ result[j];
				// Full 128-bit counter, including carry and wrap.
				for (int j = 15; j >= 0; j--) if (++chain[j] != 0U) break;
			}
		}
		else
		{
			memcpy(block, pIn + off, 16U);
			if (Alg == CRYPTO_CIPHER_CBC && bEncrypt)
				for (size_t j = 0; j < 16U; j++) block[j] ^= chain[j];
			ok = CrypBlock(block, result);
			if (ok)
			{
				if (Alg == CRYPTO_CIPHER_CBC)
				{
					if (!bEncrypt) for (size_t j = 0; j < 16U; j++) result[j] ^= chain[j];
					memcpy(chain, bEncrypt ? result : block, 16U);
				}
				memcpy(pOut + off, result, 16U);
			}
		}
		off += count;
	}
	AesOpEnd();
	CryptoSecureWipe(chain, sizeof(chain));
	CryptoSecureWipe(block, sizeof(block));
	CryptoSecureWipe(result, sizeof(result));
	if (!ok) memset(pOut, 0, Len);
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CryptoStm32H7Aes *CryptoStm32H7AesCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoStm32H7Aes) ||
		((uintptr_t)pMem & (alignof(CryptoStm32H7Aes) - 1U)) != 0U) return nullptr;
	auto *p = new (pMem) CryptoStm32H7Aes();
	if (p->Enable()) return p;
	p->~CryptoStm32H7Aes();
	return nullptr;
}

static void HashReset()
{
	RCC->AHB2RSTR |= RCC_AHB2RSTR_HASHRST;
	RCC->AHB2RSTR &= ~(uint32_t)RCC_AHB2RSTR_HASHRST;
}

static bool HashWait(uint32_t Mask, uint32_t Value)
{
	for (uint32_t i = 0; i < CRYPTO_STM32H7_POLL_MAX; i++)
	{
		if ((HASH->SR & Mask) == Value) return true;
	}
	return false;
}

bool CryptoStm32H7Hash::Enable()
{
	if (s_HashBusy.test_and_set()) return false;
	RCC->AHB2ENR |= RCC_AHB2ENR_HASHEN;
	(void)RCC->AHB2ENR;
	HashReset();
	vbValid = true;
	s_HashBusy.clear();
	return true;
}

void CryptoStm32H7Hash::Disable()
{
	if (s_HashBusy.test_and_set()) return;
	HashReset();
	vbValid = false;
	s_HashBusy.clear();
}

void CryptoStm32H7Hash::Reset()
{
	if (s_HashBusy.test_and_set()) return;
	if (vbValid) HashReset();
	s_HashBusy.clear();
}

// Caller owns s_HashBusy. Optional pad is exactly one SHA-256 block.
static bool HashDigest(const uint8_t *pPad, const uint8_t *pMsg, size_t Len,
		uint8_t *pDigest)
{
	HashReset();
	// A full HMAC pad precedes the message without copying either buffer.
	// SHA-256, byte-swap input words. Assemble partial words without overread.
	HASH->CR = HASH_CR_ALGO | HASH_CR_DATATYPE_1 | HASH_CR_INIT;
	HASH->STR = (uint32_t)(Len & 3U) * 8U;
	bool ok = true;
	if (pPad != nullptr)
	{
		for (size_t off = 0; ok && off < 64U; off += 4U)
		{
			ok = HashWait(HASH_SR_BUSY, 0U);
			if (ok)
			{
				uint32_t word = 0U;
				for (size_t j = 0; j < 4U; j++) word |= (uint32_t)pPad[off + j] << (8U * j);
				HASH->DIN = word;
			}
		}
	}
	for (size_t off = 0; ok && off < Len; )
	{
		ok = HashWait(HASH_SR_BUSY, 0U);
		if (!ok) break;
		uint32_t word = 0U;
		const size_t count = Len - off < 4U ? Len - off : 4U;
		for (size_t j = 0; j < count; j++) word |= (uint32_t)pMsg[off + j] << (8U * j);
		HASH->DIN = word;
		off += count;
	}
	if (ok)
	{
		HASH->STR |= HASH_STR_DCAL;
		ok = HashWait(HASH_SR_DCIS, HASH_SR_DCIS);
	}
	if (ok)
	{
		for (size_t i = 0; i < 8U; i++)
			StoreBe32(pDigest + 4U * i, i < 5U ? HASH->HR[i] : HASH_DIGEST->HR[i]);
	}
	else memset(pDigest, 0, 32U);
	return ok;
}

CRYPTO_STATUS CryptoStm32H7Hash::Hash(CRYPTO_HASH_ALG Alg,
		const uint8_t *pMsg, size_t Len, uint8_t *pDigest)
{
	if (Alg != CRYPTO_HASH_SHA256 || pDigest == nullptr ||
		(pMsg == nullptr && Len != 0U)) return CRYPTO_STATUS_UNSUPPORTED;
	if (!vbValid)
	{
		memset(pDigest, 0, 32U);
		return CRYPTO_STATUS_FAIL;
	}
	if (s_HashBusy.test_and_set())
	{
		memset(pDigest, 0, 32U);
		return CRYPTO_STATUS_BUSY;
	}
	bool ok = HashDigest(nullptr, pMsg, Len, pDigest);
	HashReset();
	s_HashBusy.clear();
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS CryptoStm32H7Hash::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
		const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen)
{
	if (Alg != CRYPTO_MAC_HMAC || Key.Type != CRYPTO_KEY_HMAC ||
		Key.Loc != CRYPTO_KEY_LOC_PLAIN || (Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U ||
		(Key.Plain.pData == nullptr && Key.Plain.Len != 0U) ||
		pMac == nullptr || MacLen == 0U || MacLen > 32U ||
		(pMsg == nullptr && Len != 0U)) return CRYPTO_STATUS_UNSUPPORTED;
	if (!vbValid)
	{
		memset(pMac, 0, MacLen);
		return CRYPTO_STATUS_FAIL;
	}
	if (s_HashBusy.test_and_set())
	{
		memset(pMac, 0, MacLen);
		return CRYPTO_STATUS_BUSY;
	}
	// RFC 2104: normalize the key, then H(opad || H(ipad || message)).
	// Keep ownership across all digests; each digest resets peripheral state.
	uint8_t key[64] = {}, pad[64], inner[32], full[32];
	bool ok = true;
	if (Key.Plain.Len > sizeof(key))
		ok = HashDigest(nullptr, Key.Plain.pData, Key.Plain.Len, key);
	else if (Key.Plain.Len != 0U)
		memcpy(key, Key.Plain.pData, Key.Plain.Len);
	if (ok)
	{
		for (size_t i = 0; i < sizeof(pad); i++) pad[i] = key[i] ^ 0x36U;
		ok = HashDigest(pad, pMsg, Len, inner);
	}
	if (ok)
	{
		for (size_t i = 0; i < sizeof(pad); i++) pad[i] = key[i] ^ 0x5cU;
		ok = HashDigest(pad, inner, sizeof(inner), full);
	}
	if (ok) memcpy(pMac, full, MacLen);
	else memset(pMac, 0, MacLen);
	CryptoSecureWipe(key, sizeof(key));
	CryptoSecureWipe(pad, sizeof(pad));
	CryptoSecureWipe(inner, sizeof(inner));
	CryptoSecureWipe(full, sizeof(full));
	HashReset();
	s_HashBusy.clear();
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

CryptoStm32H7Hash *CryptoStm32H7HashCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoStm32H7Hash) ||
		((uintptr_t)pMem & (alignof(CryptoStm32H7Hash) - 1U)) != 0U) return nullptr;
	auto *p = new (pMem) CryptoStm32H7Hash();
	if (p->Enable()) return p;
	p->~CryptoStm32H7Hash();
	return nullptr;
}
