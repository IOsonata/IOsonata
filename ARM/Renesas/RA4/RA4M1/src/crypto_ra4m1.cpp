/**-------------------------------------------------------------------------
@file	crypto_ra4m1.cpp

@brief	RA4M1 SCE5 crypto engines: AES-128/256 and TRNG.

See crypto_ra4m1.h. The SCE5 primitives (key index generation, AES
init/update/final, random number, self checks) are in crypto_sce5.c; this
file owns the module clock, the shared lock and the engine objects.

@author	Hoang Nguyen Hoan
@date	Oct. 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <string.h>
#include <new>

#include "ra4m1xxx.h"
#include "coredev/interrupt.h"
#include "crypto/crypto.h"
#include "crypto_ra4m1.h"
#include "crypto_sce5.h"

//=============================================================================
// Shared SCE5 block
//
// One lock for the random source and the AES engine: the SCE5 primitives
// run one at a time and a random draw inside an AES session is a resource
// conflict in the engine. The module clock is started once; the engine stays
// enabled for the lifetime of the library.
//=============================================================================

static std::atomic_flag s_SceBusy = ATOMIC_FLAG_INIT;
static bool s_SceClocked = false;

static bool SceAcquire()
{
	return std::atomic_flag_test_and_set(&s_SceBusy) == false;
}

static void SceRelease()
{
	std::atomic_flag_clear(&s_SceBusy);
}

// Caller holds the SCE lock.
static bool SceStart()
{
	uintptr_t state;
	uint16_t protect;

	if (!s_SceClocked)
	{
		state = DisableInterrupt();
		protect = R_SYSTEM->PRCR & 0x0FU;
		R_SYSTEM->PRCR = 0xA500U | protect | R_SYSTEM_PRCR_PRC1_Msk;
		R_MSTP->MSTPCRC &= ~(uint32_t)(1UL << 31U);
		(void)R_MSTP->MSTPCRC;
		R_SYSTEM->PRCR = 0xA500U | protect;
		EnableInterrupt(state);
		s_SceClocked = true;
	}
	return Sce5Init();
}

//=============================================================================
// TRNG
//=============================================================================

bool CryptoRa4m1Rng::Enable()
{
	if (vbValid && Sce5Ready())
	{
		return true;
	}
	if (!SceAcquire())
	{
		return false;
	}
	vbValid = SceStart();
	SceRelease();
	return vbValid;
}

void CryptoRa4m1Rng::Disable()
{
	// The block is shared; only this object's access is dropped.
	vbValid = false;
}

void CryptoRa4m1Rng::Reset()
{
	vbValid = false;
	(void)Enable();
}

bool CryptoRa4m1Rng::IsSecure() const
{
	return vbValid && Sce5Ready();
}

CRYPTO_STATUS CryptoRa4m1Rng::Random(uint8_t *pOut, size_t Len)
{
	uint32_t words[4] = {};
	size_t off;
	size_t count;
	bool ok = true;

	if (pOut == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (!vbValid)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	if (!SceAcquire())
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}
	for (off = 0U; off < Len; off += count)
	{
		if (!Sce5RandomWords(words))
		{
			ok = false;
			break;
		}
		count = Len - off < sizeof(words) ? Len - off : sizeof(words);
		memcpy(pOut + off, words, count);
	}
	CryptoSecureWipe(words, sizeof(words));
	SceRelease();
	if (!ok)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

int CryptoRa4m1Rng::SelfTest()
{
	uint8_t a[32];
	uint8_t b[32];
	int rc = 0;
	bool same = true;
	size_t i;

	if (Random(a, sizeof(a)) != CRYPTO_STATUS_OK ||
		Random(b, sizeof(b)) != CRYPTO_STATUS_OK)
	{
		return -1;
	}
	for (i = 1U; i < sizeof(a); i++)
	{
		if (a[i] != a[0])
		{
			same = false;
		}
	}
	if (memcmp(a, b, sizeof(a)) == 0)
	{
		rc = -2;
	}
	else if (same)
	{
		rc = -3;
	}
	CryptoSecureWipe(a, sizeof(a));
	CryptoSecureWipe(b, sizeof(b));
	return rc;
}

CryptoRa4m1Rng *CryptoRa4m1RngCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoRa4m1Rng) ||
		((uintptr_t)pMem & (alignof(CryptoRa4m1Rng) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoRa4m1Rng *p = new (pMem) CryptoRa4m1Rng();
	if (!p->Enable())
	{
		p->~CryptoRa4m1Rng();
		return nullptr;
	}
	return p;
}

//=============================================================================
// AES
//
// One operation (a Cipher call, or a CMAC/CCM/GCM computation of
// CryptoAesModes) runs between AesOpBegin and AesOpEnd under the SCE lock.
// The key index is generated and the engine initialized by the first block
// of the operation; the following blocks only run the update primitive. The
// public driver method checks finalization through FinishOperation before
// returning, then releases the lock. AesOpEnd leaves that work to its caller.
// The SCE5 works on single ECB blocks here;
// CBC and CTR chaining is done in this file.
//=============================================================================

bool CryptoRa4m1Aes::Enable()
{
	if (vbValid && Sce5Ready())
	{
		return true;
	}
	if (!SceAcquire())
	{
		return false;
	}
	vbValid = SceStart();
	SceRelease();
	return vbValid;
}

void CryptoRa4m1Aes::Disable()
{
	vbValid = false;
}

void CryptoRa4m1Aes::Reset()
{
	vbValid = false;
	(void)Enable();
}

bool CryptoRa4m1Aes::AesOpBegin()
{
	return vbValid && SceAcquire();
}

void CryptoRa4m1Aes::AesOpEnd()
{
	// Keep ownership until the public driver operation checks finalization.
	// The inherited composition routines cannot return status from this hook.
}

CRYPTO_STATUS CryptoRa4m1Aes::FinishOperation(CRYPTO_STATUS Status)
{
	bool finished = Sce5AesEnd();
	SceRelease();
	return finished ? Status : CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS CryptoRa4m1Aes::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
	const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen)
{
	CRYPTO_STATUS status = CryptoAesModes::Mac(Alg, Key, pMsg, Len, pMac, MacLen);

	// The generic modes return UNSUPPORTED before acquisition and BUSY when
	// acquisition fails. Only OK/FAIL have an owned operation to finish.
	if (status == CRYPTO_STATUS_OK || status == CRYPTO_STATUS_FAIL)
	{
		status = FinishOperation(status);
		if (status != CRYPTO_STATUS_OK)
		{
			CryptoSecureWipe(pMac, MacLen);
		}
	}
	return status;
}

CRYPTO_STATUS CryptoRa4m1Aes::Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
	const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
	const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen)
{
	CRYPTO_STATUS status = CryptoAesModes::Seal(Alg, Key, pNonce, NonceLen,
		pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);

	if (status == CRYPTO_STATUS_OK || status == CRYPTO_STATUS_FAIL)
	{
		status = FinishOperation(status);
		if (status != CRYPTO_STATUS_OK)
		{
			CryptoSecureWipe(pOut, Len);
			CryptoSecureWipe(pTag, TagLen);
		}
	}
	return status;
}

CRYPTO_STATUS CryptoRa4m1Aes::Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
	const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
	const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen)
{
	CRYPTO_STATUS status = CryptoAesModes::Open(Alg, Key, pNonce, NonceLen,
		pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);

	if (status == CRYPTO_STATUS_OK || status == CRYPTO_STATUS_FAIL)
	{
		status = FinishOperation(status);
		if (status != CRYPTO_STATUS_OK)
		{
			CryptoSecureWipe(pOut, Len);
		}
	}
	return status;
}

bool CryptoRa4m1Aes::AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
								   uint8_t Out[16])
{
	return Sce5AesBegin(Key, 16U, true) && Sce5AesUpdate(In, Out);
}

CRYPTO_STATUS CryptoRa4m1Aes::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
									 const CryptoKey &Key,
									 const uint8_t *pIv, size_t IvLen,
									 const uint8_t *pIn, size_t Len,
									 uint8_t *pOut)
{
	uint8_t chain[16] = {};
	uint8_t input[16] = {};
	uint8_t output[16] = {};
	size_t off;
	size_t i;
	size_t count;
	int digit;
	bool ok;
	bool encrypt = bEncrypt != 0;
	size_t keyLen = Key.Type == CRYPTO_KEY_AES_256 ? 32U : 16U;
	uint32_t usage = encrypt ? CRYPTO_KEY_USE_ENCRYPT : CRYPTO_KEY_USE_DECRYPT;

	if ((Key.Type != CRYPTO_KEY_AES_128 && Key.Type != CRYPTO_KEY_AES_256) ||
		Key.Loc != CRYPTO_KEY_LOC_PLAIN || Key.Plain.pData == nullptr ||
		Key.Plain.Len != keyLen || (Key.Usage & usage) == 0U ||
		pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (Alg != CRYPTO_CIPHER_ECB && Alg != CRYPTO_CIPHER_CBC && Alg != CRYPTO_CIPHER_CTR)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if ((Alg != CRYPTO_CIPHER_CTR && (Len & 15U) != 0U) ||
		(Alg != CRYPTO_CIPHER_ECB && (pIv == nullptr || IvLen != 16U)))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!AesOpBegin())
	{
		CryptoSecureWipe(pOut, Len);
		return vbValid ? CRYPTO_STATUS_BUSY : CRYPTO_STATUS_FAIL;
	}
	if (Alg != CRYPTO_CIPHER_ECB)
	{
		memcpy(chain, pIv, sizeof(chain));
	}
	// CTR always runs the block in the encrypt direction.
	ok = Sce5AesBegin(Key.Plain.pData, keyLen, encrypt || Alg == CRYPTO_CIPHER_CTR);
	for (off = 0U; ok && off < Len; off += count)
	{
		count = Len - off < 16U ? Len - off : 16U;
		memcpy(input, pIn + off, count);
		if (Alg == CRYPTO_CIPHER_CBC && encrypt)
		{
			for (i = 0U; i < 16U; i++)
			{
				input[i] ^= chain[i];
			}
		}
		ok = Sce5AesUpdate(Alg == CRYPTO_CIPHER_CTR ? chain : input, output);
		if (!ok)
		{
			break;
		}
		if (Alg == CRYPTO_CIPHER_CTR)
		{
			for (i = 0U; i < count; i++)
			{
				output[i] ^= input[i];
			}
			for (digit = 15; digit >= 0; digit--)
			{
				if (++chain[digit] != 0U)
				{
					break;
				}
			}
		}
		else if (Alg == CRYPTO_CIPHER_CBC)
		{
			if (!encrypt)
			{
				for (i = 0U; i < 16U; i++)
				{
					output[i] ^= chain[i];
				}
			}
			memcpy(chain, encrypt ? output : input, sizeof(chain));
		}
		memcpy(pOut + off, output, count);
	}
	ok = FinishOperation(ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL) == CRYPTO_STATUS_OK;
	CryptoSecureWipe(chain, sizeof(chain));
	CryptoSecureWipe(input, sizeof(input));
	CryptoSecureWipe(output, sizeof(output));
	if (!ok)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

int CryptoRa4m1Aes::SelfTest()
{
	// NIST SP 800-38A F.1.1/F.1.2, F.2.1/F.2.2, F.5.1/F.5.2 (AES-128) and
	// F.1.5/F.1.6 (AES-256), first two blocks of each.
	static const uint8_t key128[16] = {
		0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
	static const uint8_t key256[32] = {
		0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81,
		0x1f,0x35,0x2c,0x07,0x3b,0x61,0x08,0xd7,0x2d,0x98,0x10,0xa3,0x09,0x14,0xdf,0xf4 };
	static const uint8_t pt[32] = {
		0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
		0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51 };
	static const uint8_t ecb128[32] = {
		0x3a,0xd7,0x7b,0xb4,0x0d,0x7a,0x36,0x60,0xa8,0x9e,0xca,0xf3,0x24,0x66,0xef,0x97,
		0xf5,0xd3,0xd5,0x85,0x03,0xb9,0x69,0x9d,0xe7,0x85,0x89,0x5a,0x96,0xfd,0xba,0xaf };
	static const uint8_t ecb256[32] = {
		0xf3,0xee,0xd1,0xbd,0xb5,0xd2,0xa0,0x3c,0x06,0x4b,0x5a,0x7e,0x3d,0xb1,0x81,0xf8,
		0x59,0x1c,0xcb,0x10,0xd4,0x10,0xed,0x26,0xdc,0x5b,0xa7,0x4a,0x31,0x36,0x28,0x70 };
	static const uint8_t cbcIv[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
	static const uint8_t cbc128[32] = {
		0x76,0x49,0xab,0xac,0x81,0x19,0xb2,0x46,0xce,0xe9,0x8e,0x9b,0x12,0xe9,0x19,0x7d,
		0x50,0x86,0xcb,0x9b,0x50,0x72,0x19,0xee,0x95,0xdb,0x11,0x3a,0x91,0x76,0x78,0xb2 };
	static const uint8_t ctrIv[16] = {
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff };
	static const uint8_t ctr128[32] = {
		0x87,0x4d,0x61,0x91,0xb6,0x20,0xe3,0x26,0x1b,0xef,0x68,0x64,0x99,0x0d,0xb6,0xce,
		0x98,0x06,0xf6,0x6b,0x79,0x70,0xfd,0xff,0x86,0x17,0x18,0x7b,0xb9,0xff,0xfd,0xff };
	uint8_t out[32];
	uint8_t back[32];
	int rc = CryptoAesModes::SelfTest();
	CryptoKey k = {};

	if (rc != 0)
	{
		return rc;
	}
	k.Type = CRYPTO_KEY_AES_128;
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT;
	k.Plain.pData = key128;
	k.Plain.Len = sizeof(key128);
	if (Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, pt, sizeof(pt), out) != CRYPTO_STATUS_OK ||
		memcmp(out, ecb128, sizeof(ecb128)) != 0)
	{
		rc = -3;
	}
	else if (Cipher(CRYPTO_CIPHER_ECB, 0, k, nullptr, 0, ecb128, sizeof(ecb128), back) != CRYPTO_STATUS_OK ||
		memcmp(back, pt, sizeof(pt)) != 0)
	{
		rc = -4;
	}
	else if (Cipher(CRYPTO_CIPHER_CBC, 1, k, cbcIv, sizeof(cbcIv), pt, sizeof(pt), out) != CRYPTO_STATUS_OK ||
		memcmp(out, cbc128, sizeof(cbc128)) != 0)
	{
		rc = -5;
	}
	else if (Cipher(CRYPTO_CIPHER_CBC, 0, k, cbcIv, sizeof(cbcIv), cbc128, sizeof(cbc128), back) != CRYPTO_STATUS_OK ||
		memcmp(back, pt, sizeof(pt)) != 0)
	{
		rc = -6;
	}
	else if (Cipher(CRYPTO_CIPHER_CTR, 1, k, ctrIv, sizeof(ctrIv), pt, sizeof(pt), out) != CRYPTO_STATUS_OK ||
		memcmp(out, ctr128, sizeof(ctr128)) != 0)
	{
		rc = -7;
	}
	else if (Cipher(CRYPTO_CIPHER_CTR, 0, k, ctrIv, sizeof(ctrIv), ctr128, sizeof(ctr128), back) != CRYPTO_STATUS_OK ||
		memcmp(back, pt, sizeof(pt)) != 0)
	{
		rc = -8;
	}
	else
	{
		k.Type = CRYPTO_KEY_AES_256;
		k.Plain.pData = key256;
		k.Plain.Len = sizeof(key256);
		if (Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, pt, sizeof(pt), out) != CRYPTO_STATUS_OK ||
			memcmp(out, ecb256, sizeof(ecb256)) != 0)
		{
			rc = -9;
		}
		else if (Cipher(CRYPTO_CIPHER_ECB, 0, k, nullptr, 0, ecb256, sizeof(ecb256), back) != CRYPTO_STATUS_OK ||
			memcmp(back, pt, sizeof(pt)) != 0)
		{
			rc = -10;
		}
	}
	CryptoSecureWipe(out, sizeof(out));
	CryptoSecureWipe(back, sizeof(back));
	return rc;
}

CryptoRa4m1Aes *CryptoRa4m1AesCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoRa4m1Aes) ||
		((uintptr_t)pMem & (alignof(CryptoRa4m1Aes) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoRa4m1Aes *p = new (pMem) CryptoRa4m1Aes();
	if (!p->Enable())
	{
		p->~CryptoRa4m1Aes();
		return nullptr;
	}
	return p;
}
