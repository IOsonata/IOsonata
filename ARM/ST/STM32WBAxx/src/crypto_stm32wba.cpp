/**-------------------------------------------------------------------------
@file	crypto_stm32wba.cpp

@brief	Crypto engines for the STM32WBA hardware blocks: AES, PKA and RNG.

See crypto_stm32wba.h. Register sequences follow the ST reference HAL for
the series (stm32wbaxx_hal_cryp.c, stm32wbaxx_hal_pka.c, stm32wbaxx_hal_rng.c);
the AES and PKA data layout is described next to the helpers that use it.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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
#include <stdint.h>
#include <string.h>
#include <new>

#include "stm32wbaxx.h"

#include "crypto/crypto_stm32wba.h"

#define AES_BLOCK				16U

// Loop counts of the busy waits. Each loop is a few cycles, the limits are far
// above the duration of one operation at any supported core clock.
#ifndef CRYPTO_WBA_AES_POLL_MAX
#define CRYPTO_WBA_AES_POLL_MAX		200000U
#endif
#ifndef CRYPTO_WBA_PKA_POLL_MAX
#define CRYPTO_WBA_PKA_POLL_MAX		30000000U
#endif
#ifndef CRYPTO_WBA_PKA_INIT_POLL_MAX
#define CRYPTO_WBA_PKA_INIT_POLL_MAX	1000000U
#endif
#ifndef CRYPTO_WBA_RNG_POLL_MAX
#define CRYPTO_WBA_RNG_POLL_MAX		200000U
#endif
#define CRYPTO_WBA_RNG_RETRY		4
#define CRYPTO_WBA_SIGN_RETRY		4

static inline uint32_t LoadBe32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
		   ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static inline void StoreBe32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24);
	p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8);
	p[3] = (uint8_t)v;
}

//=============================================================================
// RNG
//=============================================================================

// Bring the generator up: HSI16 kernel clock, NIST compliant configuration and
// health test value of the reference HAL, conditioning reset, then enable.
// Clock error detection is switched off as in the platform layer of the
// Bluetooth port, the kernel clock is the internal HSI16.
static bool WbaRngStart(void)
{
	uint32_t n;

	SET_BIT(RCC->CR, RCC_CR_HSION);
	n = CRYPTO_WBA_RNG_POLL_MAX;
	while ((RCC->CR & RCC_CR_HSIRDY) == 0U && n > 0U)
	{
		n--;
	}
	if (n == 0U)
	{
		return false;
	}

	MODIFY_REG(RCC->CCIPR2, RCC_CCIPR2_RNGSEL, RCC_CCIPR2_RNGSEL_1);
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_RNGEN);
	(void)RCC->AHB2ENR;

	CLEAR_BIT(RNG->CR, RNG_CR_RNGEN);
	WRITE_REG(RNG->CR, RNG_CR_NIST_VALUE | RNG_CR_CONDRST | RNG_CR_CED);
	WRITE_REG(RNG->HTCR[0], RNG_HTCR_NIST_VALUE);
	CLEAR_BIT(RNG->CR, RNG_CR_CONDRST);
	n = CRYPTO_WBA_RNG_POLL_MAX;
	while ((RNG->CR & RNG_CR_CONDRST) != 0U && n > 0U)
	{
		n--;
	}
	if (n == 0U)
	{
		return false;
	}

	SET_BIT(RNG->CR, RNG_CR_RNGEN);
	if ((RNG->SR & RNG_SR_SEIS) != 0U)
	{
		return false;
	}
	n = CRYPTO_WBA_RNG_POLL_MAX;
	while ((RNG->SR & RNG_SR_DRDY) == 0U && n > 0U)
	{
		n--;
	}
	return n > 0U;
}

static void WbaRngStop(void)
{
	CLEAR_BIT(RNG->CR, RNG_CR_RNGEN);
	CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_RNGEN);
}

// One 32 bit word. A seed or clock error, or a timeout, restarts the
// generator and the read is tried again.
static bool WbaRngWord(uint32_t *pWord)
{
	const uint32_t errMask = RNG_SR_SEIS | RNG_SR_CEIS | RNG_SR_SECS | RNG_SR_CECS;

	for (int retry = 0; retry < CRYPTO_WBA_RNG_RETRY; retry++)
	{
		uint32_t n = CRYPTO_WBA_RNG_POLL_MAX;

		while (n > 0U)
		{
			uint32_t sr = RNG->SR;

			if ((sr & errMask) != 0U)
			{
				break;
			}
			if ((sr & RNG_SR_DRDY) != 0U)
			{
				uint32_t w = RNG->DR;

				if ((RNG->SR & errMask) == 0U)
				{
					*pWord = w;
					return true;
				}
				break;
			}
			n--;
		}

		CLEAR_BIT(RNG->SR, RNG_SR_SEIS | RNG_SR_CEIS);
		WbaRngStop();
		(void)WbaRngStart();
	}
	return false;
}

bool CryptoWbaRng::Enable()
{
	if (vbValid)
	{
		return true;
	}
	if (std::atomic_flag_test_and_set(&vOpBusy))
	{
		return false;
	}
	vbValid = WbaRngStart();
	std::atomic_flag_clear(&vOpBusy);
	return vbValid;
}

void CryptoWbaRng::Disable()
{
	if (std::atomic_flag_test_and_set(&vOpBusy))
	{
		return;
	}
	vbValid = false;
	WbaRngStop();
	std::atomic_flag_clear(&vOpBusy);
}

void CryptoWbaRng::Reset()
{
	if (std::atomic_flag_test_and_set(&vOpBusy))
	{
		return;
	}
	WbaRngStop();
	vbValid = WbaRngStart();
	std::atomic_flag_clear(&vOpBusy);
}

CRYPTO_STATUS CryptoWbaRng::Random(uint8_t *pOut, size_t Len)
{
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
	if (std::atomic_flag_test_and_set(&vOpBusy))
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}

	bool ok = true;
	size_t off = 0U;

	while (ok && off < Len)
	{
		uint32_t w;

		ok = WbaRngWord(&w);
		if (ok)
		{
			size_t n = Len - off < 4U ? Len - off : 4U;

			memcpy(pOut + off, &w, n);
			off += n;
		}
		w = 0U;
	}
	std::atomic_flag_clear(&vOpBusy);

	if (!ok)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

int CryptoWbaRng::SelfTest()
{
	uint8_t a[32], b[32];

	if (Random(a, sizeof(a)) != CRYPTO_STATUS_OK ||
		Random(b, sizeof(b)) != CRYPTO_STATUS_OK)
	{
		return -1;
	}
	int rc = 0;
	if (memcmp(a, b, sizeof(a)) == 0)
	{
		rc = -2;
	}
	else
	{
		bool same = true;

		for (size_t i = 1U; i < sizeof(a); i++)
		{
			if (a[i] != a[0])
			{
				same = false;
			}
		}
		if (same)
		{
			rc = -3;
		}
	}
	CryptoSecureWipe(a, sizeof(a));
	CryptoSecureWipe(b, sizeof(b));
	return rc;
}

CryptoWbaRng *CryptoWbaRngCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoWbaRng) ||
		((uintptr_t)pMem & (alignof(CryptoWbaRng) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoWbaRng *p = new (pMem) CryptoWbaRng();
	if (!p->Enable())
	{
		p->~CryptoWbaRng();
		return nullptr;
	}
	return p;
}

//=============================================================================
// AES
//
// Data layout. The peripheral is used with DATATYPE 0 (no swap). A 16 byte
// block in standard (FIPS-197) byte order is written to DINR as four words,
// the first four bytes as a big-endian word first, and DOUTR returns it the
// same way. KEYR3 holds key bytes 0 to 3 and KEYR0 bytes 12 to 15. IVR3 holds
// IV (counter) bytes 0 to 3 and IVR0 bytes 12 to 15.
//=============================================================================

#define WBA_AES_MODE_ENC		0U
#define WBA_AES_MODE_KEYDER		AES_CR_MODE_0
#define WBA_AES_MODE_DEC		AES_CR_MODE_1
#define WBA_AES_CHMOD_ECB		0U
#define WBA_AES_CHMOD_CBC		AES_CR_CHMOD_0
#define WBA_AES_CHMOD_CTR		AES_CR_CHMOD_1

static void WbaAesWipeRegs(void)
{
	AES->KEYR0 = 0U;
	AES->KEYR1 = 0U;
	AES->KEYR2 = 0U;
	AES->KEYR3 = 0U;
	AES->IVR0 = 0U;
	AES->IVR1 = 0U;
	AES->IVR2 = 0U;
	AES->IVR3 = 0U;
}

static bool WbaAesWaitCcf(void)
{
	for (uint32_t n = 0U; n < CRYPTO_WBA_AES_POLL_MAX; n++)
	{
		uint32_t sr = AES->SR;

		if ((sr & (AES_SR_RDERR | AES_SR_WRERR)) != 0U)
		{
			return false;
		}
		if ((sr & AES_SR_CCF) != 0U)
		{
			return true;
		}
	}
	return false;
}

static void WbaAesLoadKey(const uint8_t *pKey)
{
	AES->KEYR3 = LoadBe32(pKey);
	AES->KEYR2 = LoadBe32(pKey + 4);
	AES->KEYR1 = LoadBe32(pKey + 8);
	AES->KEYR0 = LoadBe32(pKey + 12);
}

static void WbaAesLoadIv(const uint8_t *pIv)
{
	AES->IVR3 = LoadBe32(pIv);
	AES->IVR2 = LoadBe32(pIv + 4);
	AES->IVR1 = LoadBe32(pIv + 8);
	AES->IVR0 = LoadBe32(pIv + 12);
}

// Run Blocks 16 byte blocks through the peripheral. Mode is ENC or DEC,
// Chmod ECB, CBC or CTR. pIv is the IV or the first counter block (null for
// ECB). pOut may equal pIn. ECB and CBC decryption first derive the
// decryption key (mode 01) and then switch to mode 10, as the reference HAL
// does; the peripheral and the key registers are cleared at the end.
static bool WbaAesRun(uint32_t Mode, uint32_t Chmod, const uint8_t *pKey,
					  const uint8_t *pIv, const uint8_t *pIn, uint8_t *pOut,
					  size_t Blocks)
{
	bool ok = true;

	AES->CR = 0U;
	AES->ICR = AES_ICR_CCF | AES_ICR_RWEIF | AES_ICR_KEIF;

	if (Mode == WBA_AES_MODE_DEC && Chmod != WBA_AES_CHMOD_CTR)
	{
		AES->CR = Chmod | WBA_AES_MODE_KEYDER;
		WbaAesLoadKey(pKey);
		SET_BIT(AES->CR, AES_CR_EN);
		ok = WbaAesWaitCcf();
		AES->ICR = AES_ICR_CCF;
		MODIFY_REG(AES->CR, AES_CR_MODE, WBA_AES_MODE_DEC);
	}
	else
	{
		AES->CR = Chmod | Mode;
		WbaAesLoadKey(pKey);
	}
	if (ok && pIv != nullptr)
	{
		WbaAesLoadIv(pIv);
	}
	if (ok)
	{
		SET_BIT(AES->CR, AES_CR_EN);
	}

	for (size_t b = 0U; ok && b < Blocks; b++)
	{
		const uint8_t *s = pIn + b * AES_BLOCK;
		uint8_t *d = pOut + b * AES_BLOCK;

		AES->DINR = LoadBe32(s);
		AES->DINR = LoadBe32(s + 4);
		AES->DINR = LoadBe32(s + 8);
		AES->DINR = LoadBe32(s + 12);
		ok = WbaAesWaitCcf();
		if (ok)
		{
			uint32_t w0 = AES->DOUTR;
			uint32_t w1 = AES->DOUTR;
			uint32_t w2 = AES->DOUTR;
			uint32_t w3 = AES->DOUTR;

			StoreBe32(d, w0);
			StoreBe32(d + 4, w1);
			StoreBe32(d + 8, w2);
			StoreBe32(d + 12, w3);
		}
		AES->ICR = AES_ICR_CCF;
	}

	if ((AES->ISR & (AES_ISR_KEIF | AES_ISR_RWEIF)) != 0U)
	{
		ok = false;
	}
	AES->CR = 0U;
	AES->ICR = AES_ICR_CCF | AES_ICR_RWEIF | AES_ICR_KEIF;
	WbaAesWipeRegs();
	return ok;
}

// Add Blocks to the 128 bit big-endian counter.
static void WbaCtrAdd(uint8_t Ctr[AES_BLOCK], uint32_t Blocks)
{
	uint64_t sum = (uint64_t)LoadBe32(Ctr + 12) + Blocks;

	StoreBe32(Ctr + 12, (uint32_t)sum);
	uint32_t carry = (uint32_t)(sum >> 32);
	for (int i = 11; i >= 0 && carry != 0U; i--)
	{
		uint32_t v = (uint32_t)Ctr[i] + carry;

		Ctr[i] = (uint8_t)v;
		carry = v >> 8;
	}
}

bool CryptoWbaAes::Enable()
{
	if (vbValid)
	{
		return true;
	}
	if (!OpAcquire())
	{
		return false;
	}
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_AESEN);
	(void)RCC->AHB2ENR;
	SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_AESRST);
	CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_AESRST);
	AES->CR = 0U;
	vbValid = true;
	OpRelease();
	return true;
}

void CryptoWbaAes::Disable()
{
	if (!OpAcquire())
	{
		return;
	}
	vbValid = false;
	AES->CR = 0U;
	WbaAesWipeRegs();
	CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_AESEN);
	OpRelease();
}

void CryptoWbaAes::Reset()
{
	if (!OpAcquire())
	{
		return;
	}
	SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_AESRST);
	CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_AESRST);
	AES->CR = 0U;
	OpRelease();
}

bool CryptoWbaAes::AesOpBegin()
{
	return vbValid && OpAcquire();
}

void CryptoWbaAes::AesOpEnd()
{
	OpRelease();
}

bool CryptoWbaAes::AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
								 uint8_t Out[16])
{
	return WbaAesRun(WBA_AES_MODE_ENC, WBA_AES_CHMOD_ECB, Key, nullptr, In,
					 Out, 1U);
}

CRYPTO_STATUS CryptoWbaAes::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
								   const CryptoKey &Key,
								   const uint8_t *pIv, size_t IvLen,
								   const uint8_t *pIn, size_t Len,
								   uint8_t *pOut)
{
	uint32_t required = bEncrypt != 0 ? CRYPTO_KEY_USE_ENCRYPT :
									 CRYPTO_KEY_USE_DECRYPT;

	// The base implements key sizes this accelerator does not provide.
	if (Key.Type == CRYPTO_KEY_AES_256)
	{
		return CryptoSoftAes::Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
	}

	if (Key.Type != CRYPTO_KEY_AES_128 || Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		Key.Plain.pData == nullptr || Key.Plain.Len != AES_BLOCK ||
		(Key.Usage & required) == 0U || pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (Alg != CRYPTO_CIPHER_ECB && Alg != CRYPTO_CIPHER_CTR &&
		Alg != CRYPTO_CIPHER_CBC)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if ((Alg == CRYPTO_CIPHER_CTR || Alg == CRYPTO_CIPHER_CBC) &&
		(pIv == nullptr || IvLen != AES_BLOCK))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (Alg != CRYPTO_CIPHER_CTR && (Len & (AES_BLOCK - 1U)) != 0U)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!vbValid)
	{
		if (Len > 0U)
		{
			memset(pOut, 0, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (!OpAcquire())
	{
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_BUSY;
	}

	const uint8_t *key = Key.Plain.pData;
	const uint32_t mode = bEncrypt != 0 ? WBA_AES_MODE_ENC : WBA_AES_MODE_DEC;
	bool ok = true;

	if (Alg == CRYPTO_CIPHER_ECB)
	{
		ok = WbaAesRun(mode, WBA_AES_CHMOD_ECB, key, nullptr, pIn, pOut,
					   Len / AES_BLOCK);
	}
	else if (Alg == CRYPTO_CIPHER_CBC)
	{
		ok = WbaAesRun(mode, WBA_AES_CHMOD_CBC, key, pIv, pIn, pOut,
					   Len / AES_BLOCK);
	}
	else
	{
		// The peripheral counter is only relied on below the wrap of its low
		// 32 bits. A run stops at that boundary and the counter is carried
		// into the upper 96 bits here, which gives the full 128 bit counter
		// of the software engine.
		uint8_t ctr[AES_BLOCK];
		size_t off = 0U;

		memcpy(ctr, pIv, AES_BLOCK);
		while (ok && off < Len)
		{
			size_t remain = Len - off;
			size_t full = remain / AES_BLOCK;

			if (full == 0U)
			{
				uint8_t in[AES_BLOCK] = {};
				uint8_t out[AES_BLOCK];

				memcpy(in, pIn + off, remain);
				ok = WbaAesRun(mode, WBA_AES_CHMOD_CTR, key, ctr, in, out, 1U);
				if (ok)
				{
					memcpy(pOut + off, out, remain);
				}
				CryptoSecureWipe(in, sizeof(in));
				CryptoSecureWipe(out, sizeof(out));
				off += remain;
			}
			else
			{
				uint64_t toWrap = 0x100000000ULL - (uint64_t)LoadBe32(ctr + 12);
				size_t n = (uint64_t)full < toWrap ? full : (size_t)toWrap;

				ok = WbaAesRun(mode, WBA_AES_CHMOD_CTR, key, ctr, pIn + off,
							   pOut + off, n);
				WbaCtrAdd(ctr, (uint32_t)n);
				off += n * AES_BLOCK;
			}
		}
		CryptoSecureWipe(ctr, sizeof(ctr));
	}
	OpRelease();

	if (!ok)
	{
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

int CryptoWbaAes::SelfTest()
{
	// NIST SP 800-38A, AES-128, four blocks.
	static const uint8_t key[16] = {
		0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
	static const uint8_t pt[64] = {
		0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
		0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
		0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
		0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10 };
	static const uint8_t ecb[64] = {
		0x3a,0xd7,0x7b,0xb4,0x0d,0x7a,0x36,0x60,0xa8,0x9e,0xca,0xf3,0x24,0x66,0xef,0x97,
		0xf5,0xd3,0xd5,0x85,0x03,0xb9,0x69,0x9d,0xe7,0x85,0x89,0x5a,0x96,0xfd,0xba,0xaf,
		0x43,0xb1,0xcd,0x7f,0x59,0x8e,0xce,0x23,0x88,0x1b,0x00,0xe3,0xed,0x03,0x06,0x88,
		0x7b,0x0c,0x78,0x5e,0x27,0xe8,0xad,0x3f,0x82,0x23,0x20,0x71,0x04,0x72,0x5d,0xd4 };
	static const uint8_t cbcIv[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
	static const uint8_t cbc[64] = {
		0x76,0x49,0xab,0xac,0x81,0x19,0xb2,0x46,0xce,0xe9,0x8e,0x9b,0x12,0xe9,0x19,0x7d,
		0x50,0x86,0xcb,0x9b,0x50,0x72,0x19,0xee,0x95,0xdb,0x11,0x3a,0x91,0x76,0x78,0xb2,
		0x73,0xbe,0xd6,0xb8,0xe3,0xc1,0x74,0x3b,0x71,0x16,0xe6,0x9e,0x22,0x22,0x95,0x16,
		0x3f,0xf1,0xca,0xa1,0x68,0x1f,0xac,0x09,0x12,0x0e,0xca,0x30,0x75,0x86,0xe1,0xa7 };
	static const uint8_t ctrIv[16] = {
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff };
	static const uint8_t ctr[64] = {
		0x87,0x4d,0x61,0x91,0xb6,0x20,0xe3,0x26,0x1b,0xef,0x68,0x64,0x99,0x0d,0xb6,0xce,
		0x98,0x06,0xf6,0x6b,0x79,0x70,0xfd,0xff,0x86,0x17,0x18,0x7b,0xb9,0xff,0xfd,0xff,
		0x5a,0xe4,0xdf,0x3e,0xdb,0xd5,0xd3,0x5e,0x5b,0x4f,0x09,0x02,0x0d,0xb0,0x3e,0xab,
		0x1e,0x03,0x1d,0xda,0x2f,0xbe,0x03,0xd1,0x79,0x21,0x70,0xa0,0xf3,0x00,0x9c,0xee };

	int rc = CryptoSoftAes::SelfTest();
	if (rc != 0)
	{
		return rc;
	}

	CryptoKey k{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
				CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	k.Plain.pData = key;
	k.Plain.Len = sizeof(key);
	uint8_t out[64];
	uint8_t back[64];

	if (Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, pt, sizeof(pt), out) !=
		CRYPTO_STATUS_OK || memcmp(out, ecb, sizeof(ecb)) != 0)
	{
		return -3;
	}
	if (Cipher(CRYPTO_CIPHER_ECB, 0, k, nullptr, 0, ecb, sizeof(ecb), back) !=
		CRYPTO_STATUS_OK || memcmp(back, pt, sizeof(pt)) != 0)
	{
		return -4;
	}
	if (Cipher(CRYPTO_CIPHER_CBC, 1, k, cbcIv, sizeof(cbcIv), pt, sizeof(pt),
			   out) != CRYPTO_STATUS_OK || memcmp(out, cbc, sizeof(cbc)) != 0)
	{
		return -5;
	}
	if (Cipher(CRYPTO_CIPHER_CBC, 0, k, cbcIv, sizeof(cbcIv), cbc, sizeof(cbc),
			   back) != CRYPTO_STATUS_OK || memcmp(back, pt, sizeof(pt)) != 0)
	{
		return -6;
	}
	if (Cipher(CRYPTO_CIPHER_CTR, 1, k, ctrIv, sizeof(ctrIv), pt, sizeof(pt),
			   out) != CRYPTO_STATUS_OK || memcmp(out, ctr, sizeof(ctr)) != 0)
	{
		return -7;
	}
	if (Cipher(CRYPTO_CIPHER_CTR, 0, k, ctrIv, sizeof(ctrIv), ctr, sizeof(ctr),
			   back) != CRYPTO_STATUS_OK || memcmp(back, pt, sizeof(pt)) != 0)
	{
		return -8;
	}
	return 0;
}

CryptoWbaAes *CryptoWbaAesCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoWbaAes) ||
		((uintptr_t)pMem & (alignof(CryptoWbaAes) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoWbaAes *p = new (pMem) CryptoWbaAes();
	p->Enable();
	return p;
}

//=============================================================================
// PKA
//
// Data layout. Every operand is a big-endian byte string of 32 bytes. In the
// accelerator RAM it is stored as eight 32 bit words, least significant word
// first, followed by two zero words. The result word of a mode equals
// 0xD60D when the operation completed without error.
//=============================================================================

#define WBA_PKA_NO_ERROR		0xD60DU
#define WBA_PKA_BITS			256U

#define WBA_PKA_MODE_MONTGOMERY	0x01U
#define WBA_PKA_MODE_ECC_MUL	0x20U
#define WBA_PKA_MODE_ECDSA_SIGN	0x24U
#define WBA_PKA_MODE_ECDSA_VER	0x26U
#define WBA_PKA_MODE_PT_CHECK	0x28U

#define WBA_PKA_FLAGS_ERR	(PKA_SR_RAMERRF | PKA_SR_ADDRERRF | PKA_SR_OPERRF)
#define WBA_PKA_CLR_ALL		(PKA_CLRFR_PROCENDFC | PKA_CLRFR_RAMERRFC | \
							 PKA_CLRFR_ADDRERRFC | PKA_CLRFR_OPERRFC)

// NIST P-256 (secp256r1): y^2 = x^3 + a x + b mod p with a = -3.
static const uint8_t s_P[32] = {
	0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff };
static const uint8_t s_AbsA[32] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03 };
static const uint8_t s_B[32] = {
	0x5a,0xc6,0x35,0xd8,0xaa,0x3a,0x93,0xe7,0xb3,0xeb,0xbd,0x55,0x76,0x98,0x86,0xbc,
	0x65,0x1d,0x06,0xb0,0xcc,0x53,0xb0,0xf6,0x3b,0xce,0x3c,0x3e,0x27,0xd2,0x60,0x4b };
static const uint8_t s_N[32] = {
	0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xbc,0xe6,0xfa,0xad,0xa7,0x17,0x9e,0x84,0xf3,0xb9,0xca,0xc2,0xfc,0x63,0x25,0x51 };
static const uint8_t s_Gx[32] = {
	0x6b,0x17,0xd1,0xf2,0xe1,0x2c,0x42,0x47,0xf8,0xbc,0xe6,0xe5,0x63,0xa4,0x40,0xf2,
	0x77,0x03,0x7d,0x81,0x2d,0xeb,0x33,0xa0,0xf4,0xa1,0x39,0x45,0xd8,0x98,0xc2,0x96 };
static const uint8_t s_Gy[32] = {
	0x4f,0xe3,0x42,0xe2,0xfe,0x1a,0x7f,0x9b,0x8e,0xe7,0xeb,0x4a,0x7c,0x0f,0x9e,0x16,
	0x2b,0xce,0x33,0x57,0x6b,0x31,0x5e,0xce,0xcb,0xb6,0x40,0x68,0x37,0xbf,0x51,0xf5 };

static void WbaPkaPut(uint32_t Idx, const uint8_t *pBe)
{
	for (uint32_t i = 0U; i < 8U; i++)
	{
		PKA->RAM[Idx + i] = LoadBe32(pBe + 28U - 4U * i);
	}
	PKA->RAM[Idx + 8U] = 0U;
	PKA->RAM[Idx + 9U] = 0U;
}

static void WbaPkaGet(uint8_t *pBe, uint32_t Idx)
{
	for (uint32_t i = 0U; i < 8U; i++)
	{
		StoreBe32(pBe + 28U - 4U * i, PKA->RAM[Idx + i]);
	}
}

static void WbaPkaPutWords(uint32_t Idx, const uint32_t *pWords)
{
	for (uint32_t i = 0U; i < 8U; i++)
	{
		PKA->RAM[Idx + i] = pWords[i];
	}
	PKA->RAM[Idx + 8U] = 0U;
	PKA->RAM[Idx + 9U] = 0U;
}

static void WbaPkaWipeRam(void)
{
	for (uint32_t i = 0U; i < sizeof(PKA->RAM) / sizeof(PKA->RAM[0]); i++)
	{
		PKA->RAM[i] = 0U;
	}
}

// Clock on, reset, enable and wait for the RAM erase to end.
static bool WbaPkaStart(void)
{
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_PKAEN);
	(void)RCC->AHB2ENR;
	SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_PKARST);
	CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_PKARST);

	uint32_t n = CRYPTO_WBA_PKA_INIT_POLL_MAX;
	bool en = false;
	while (!en && n > 0U)
	{
		PKA->CR = PKA_CR_EN;
		en = (PKA->CR & PKA_CR_EN) != 0U;
		n--;
	}
	if (!en)
	{
		return false;
	}
	n = CRYPTO_WBA_PKA_INIT_POLL_MAX;
	while ((PKA->SR & PKA_SR_INITOK) == 0U && n > 0U)
	{
		n--;
	}
	if (n == 0U)
	{
		return false;
	}
	PKA->CLRFR = WBA_PKA_CLR_ALL;
	return true;
}

// Start the mode and wait for the end of the operation. Interrupts of the
// block stay off.
static bool WbaPkaRun(uint32_t Mode)
{
	MODIFY_REG(PKA->CR, PKA_CR_MODE_Msk | PKA_CR_PROCENDIE | PKA_CR_RAMERRIE |
			   PKA_CR_ADDRERRIE | PKA_CR_OPERRIE, Mode << PKA_CR_MODE_Pos);
	SET_BIT(PKA->CR, PKA_CR_START);

	bool done = false;

	for (uint32_t n = 0U; n < CRYPTO_WBA_PKA_POLL_MAX; n++)
	{
		if ((PKA->SR & PKA_SR_PROCENDF) != 0U)
		{
			done = true;
			break;
		}
	}
	uint32_t sr = PKA->SR;

	PKA->CLRFR = WBA_PKA_CLR_ALL;
	if (!done)
	{
		// Abort the operation that does not end and arm the block again.
		(void)WbaPkaStart();
		return false;
	}
	return (sr & WBA_PKA_FLAGS_ERR) == 0U;
}

// Scalar multiplication K * (Px, Py) on P-256.
static bool WbaPkaScalarMul(const uint8_t K[32], const uint8_t Px[32],
							const uint8_t Py[32], uint8_t Rx[32],
							uint8_t Ry[32])
{
	PKA->RAM[PKA_ECC_SCALAR_MUL_IN_EXP_NB_BITS] = WBA_PKA_BITS;
	PKA->RAM[PKA_ECC_SCALAR_MUL_IN_OP_NB_BITS] = WBA_PKA_BITS;
	PKA->RAM[PKA_ECC_SCALAR_MUL_IN_A_COEFF_SIGN] = 1U;
	WbaPkaPut(PKA_ECC_SCALAR_MUL_IN_A_COEFF, s_AbsA);
	WbaPkaPut(PKA_ECC_SCALAR_MUL_IN_B_COEFF, s_B);
	WbaPkaPut(PKA_ECC_SCALAR_MUL_IN_MOD_GF, s_P);
	WbaPkaPut(PKA_ECC_SCALAR_MUL_IN_K, K);
	WbaPkaPut(PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_X, Px);
	WbaPkaPut(PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_Y, Py);
	WbaPkaPut(PKA_ECC_SCALAR_MUL_IN_N_PRIME_ORDER, s_N);

	if (!WbaPkaRun(WBA_PKA_MODE_ECC_MUL) ||
		PKA->RAM[PKA_ECC_SCALAR_MUL_OUT_ERROR] != WBA_PKA_NO_ERROR)
	{
		return false;
	}
	WbaPkaGet(Rx, PKA_ECC_SCALAR_MUL_OUT_RESULT_X);
	WbaPkaGet(Ry, PKA_ECC_SCALAR_MUL_OUT_RESULT_Y);
	return true;
}

// Montgomery parameter R^2 mod p of the field prime, computed by the
// accelerator as the reference HAL does for the point check.
static bool WbaPkaMontgomery(uint32_t R2[8])
{
	PKA->RAM[PKA_MONTGOMERY_PARAM_IN_MOD_NB_BITS] = WBA_PKA_BITS;
	WbaPkaPut(PKA_MONTGOMERY_PARAM_IN_MODULUS, s_P);

	if (!WbaPkaRun(WBA_PKA_MODE_MONTGOMERY))
	{
		return false;
	}
	for (uint32_t i = 0U; i < 8U; i++)
	{
		R2[i] = PKA->RAM[PKA_MONTGOMERY_PARAM_OUT_PARAMETER + i];
	}
	return true;
}

// 1 when (X, Y) is a point of P-256, 0 when it is not, -1 on a failure of the
// block. Both coordinates must be below the field prime. R2 is the
// Montgomery parameter of the prime. pRawError, when given, receives the
// error word of the block.
static int WbaPkaPointCheckR2(const uint8_t X[32], const uint8_t Y[32],
							  const uint32_t R2[8], uint32_t *pRawError)
{
	if (!P256LessBe(X, s_P, 32U) || !P256LessBe(Y, s_P, 32U))
	{
		return 0;
	}
	PKA->RAM[PKA_POINT_CHECK_IN_MOD_NB_BITS] = WBA_PKA_BITS;
	PKA->RAM[PKA_POINT_CHECK_IN_A_COEFF_SIGN] = 1U;
	WbaPkaPut(PKA_POINT_CHECK_IN_A_COEFF, s_AbsA);
	WbaPkaPut(PKA_POINT_CHECK_IN_B_COEFF, s_B);
	WbaPkaPut(PKA_POINT_CHECK_IN_MOD_GF, s_P);
	WbaPkaPut(PKA_POINT_CHECK_IN_INITIAL_POINT_X, X);
	WbaPkaPut(PKA_POINT_CHECK_IN_INITIAL_POINT_Y, Y);
	WbaPkaPutWords(PKA_POINT_CHECK_IN_MONTGOMERY_PARAM, R2);

	if (!WbaPkaRun(WBA_PKA_MODE_PT_CHECK))
	{
		return -1;
	}
	uint32_t err = PKA->RAM[PKA_POINT_CHECK_OUT_ERROR];

	if (pRawError != nullptr)
	{
		*pRawError = err;
	}
	return err == WBA_PKA_NO_ERROR ? 1 : 0;
}

static int WbaPkaPointCheck(const uint8_t X[32], const uint8_t Y[32])
{
	uint32_t r2[8];

	if (!WbaPkaMontgomery(r2))
	{
		return -1;
	}
	WbaPkaWipeRam();

	return WbaPkaPointCheckR2(X, Y, r2, nullptr);
}

// Hash to the integer e of ECDSA for P-256: the leftmost 32 bytes of a longer
// hash, a shorter hash as a number, then reduced below the order.
static void WbaHashToScalar(const uint8_t *pHash, size_t Len, uint8_t E[32])
{
	memset(E, 0, 32U);
	if (Len >= 32U)
	{
		memcpy(E, pHash, 32U);
	}
	else
	{
		memcpy(E + 32U - Len, pHash, Len);
	}
	if (!P256LessBe(E, s_N, 32U))
	{
		int borrow = 0;

		for (int i = 31; i >= 0; i--)
		{
			int d = (int)E[i] - (int)s_N[i] - borrow;

			borrow = d < 0 ? 1 : 0;
			E[i] = (uint8_t)d;
		}
	}
}

// ECDSA signature with the nonce K. 0 done, 1 the block rejected the nonce
// (try another), -1 failure of the block.
static int WbaPkaSignWithK(const uint8_t D[32], const uint8_t E[32],
						   const uint8_t K[32], uint8_t Sig[64])
{
	PKA->RAM[PKA_ECDSA_SIGN_IN_ORDER_NB_BITS] = WBA_PKA_BITS;
	PKA->RAM[PKA_ECDSA_SIGN_IN_MOD_NB_BITS] = WBA_PKA_BITS;
	PKA->RAM[PKA_ECDSA_SIGN_IN_A_COEFF_SIGN] = 1U;
	WbaPkaPut(PKA_ECDSA_SIGN_IN_A_COEFF, s_AbsA);
	WbaPkaPut(PKA_ECDSA_SIGN_IN_B_COEFF, s_B);
	WbaPkaPut(PKA_ECDSA_SIGN_IN_MOD_GF, s_P);
	WbaPkaPut(PKA_ECDSA_SIGN_IN_K, K);
	WbaPkaPut(PKA_ECDSA_SIGN_IN_INITIAL_POINT_X, s_Gx);
	WbaPkaPut(PKA_ECDSA_SIGN_IN_INITIAL_POINT_Y, s_Gy);
	WbaPkaPut(PKA_ECDSA_SIGN_IN_HASH_E, E);
	WbaPkaPut(PKA_ECDSA_SIGN_IN_PRIVATE_KEY_D, D);
	WbaPkaPut(PKA_ECDSA_SIGN_IN_ORDER_N, s_N);

	if (!WbaPkaRun(WBA_PKA_MODE_ECDSA_SIGN))
	{
		return -1;
	}
	if (PKA->RAM[PKA_ECDSA_SIGN_OUT_ERROR] != WBA_PKA_NO_ERROR)
	{
		return 1;
	}
	WbaPkaGet(Sig, PKA_ECDSA_SIGN_OUT_SIGNATURE_R);
	WbaPkaGet(Sig + 32, PKA_ECDSA_SIGN_OUT_SIGNATURE_S);
	if (P256IsZero(Sig, 32U) || P256IsZero(Sig + 32, 32U))
	{
		return 1;
	}
	return 0;
}

// ECDSA verification. 1 valid, 0 not valid, -1 failure of the block.
static int WbaPkaVerify(const uint8_t Pub[64], const uint8_t E[32],
						const uint8_t Sig[64])
{
	PKA->RAM[PKA_ECDSA_VERIF_IN_ORDER_NB_BITS] = WBA_PKA_BITS;
	PKA->RAM[PKA_ECDSA_VERIF_IN_MOD_NB_BITS] = WBA_PKA_BITS;
	PKA->RAM[PKA_ECDSA_VERIF_IN_A_COEFF_SIGN] = 1U;
	WbaPkaPut(PKA_ECDSA_VERIF_IN_A_COEFF, s_AbsA);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_MOD_GF, s_P);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_INITIAL_POINT_X, s_Gx);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_INITIAL_POINT_Y, s_Gy);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_X, Pub);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_Y, Pub + 32);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_SIGNATURE_R, Sig);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_SIGNATURE_S, Sig + 32);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_HASH_E, E);
	WbaPkaPut(PKA_ECDSA_VERIF_IN_ORDER_N, s_N);

	if (!WbaPkaRun(WBA_PKA_MODE_ECDSA_VER))
	{
		return -1;
	}
	return PKA->RAM[PKA_ECDSA_VERIF_OUT_RESULT] == WBA_PKA_NO_ERROR ? 1 : 0;
}

bool CryptoWbaPka::Enable()
{
	if (vbValid)
	{
		return true;
	}
	if (!OpAcquire())
	{
		return false;
	}
	vbValid = WbaPkaStart();
	OpRelease();
	return vbValid;
}

void CryptoWbaPka::Disable()
{
	if (!OpAcquire())
	{
		return;
	}
	vbValid = false;
	WbaPkaWipeRam();
	PKA->CR = 0U;
	CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_PKAEN);
	OpRelease();
}

void CryptoWbaPka::Reset()
{
	if (!OpAcquire())
	{
		return;
	}
	vbValid = WbaPkaStart();
	OpRelease();
}

void CryptoWbaPka::KeyReset(void *pKeyCtx)
{
	if (pKeyCtx != nullptr)
	{
		CryptoSecureWipe(pKeyCtx, sizeof(KeyCtx));
	}
}

CRYPTO_STATUS CryptoWbaPka::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
								   uint8_t *pPubKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr || pPubKey == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	KeyCtx *pk = (KeyCtx *)pKeyCtx;

	KeyReset(pk);
	memset(pPubKey, 0, 64U);

	if (vpRng == nullptr || !vpRng->IsSecure())
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!OpAcquire())
	{
		return CRYPTO_STATUS_BUSY;
	}

	uint8_t d[32];
	CRYPTO_STATUS st = P256RandomScalarStatus(vpRng, d);

	if (st != CRYPTO_STATUS_OK)
	{
		OpRelease();
		return st == CRYPTO_STATUS_BUSY || st == CRYPTO_STATUS_UNSUPPORTED ?
			st : CRYPTO_STATUS_FAIL;
	}

	bool ok = WbaPkaScalarMul(d, s_Gx, s_Gy, pPubKey, pPubKey + 32);

	WbaPkaWipeRam();
	OpRelease();
	if (!ok)
	{
		CryptoSecureWipe(d, sizeof(d));
		memset(pPubKey, 0, 64U);
		return CRYPTO_STATUS_FAIL;
	}
	memcpy(pk->PrivKey, d, 32U);
	pk->bKeyValid = true;
	CryptoSecureWipe(d, sizeof(d));
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoWbaPka::KeyImport(CRYPTO_CURVE Curve, void *pKeyCtx,
									  const uint8_t *pPrivKey,
									  uint8_t *pPubKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
		pPrivKey == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	KeyCtx *pk = (KeyCtx *)pKeyCtx;

	KeyReset(pk);
	if (pPubKey != nullptr)
	{
		memset(pPubKey, 0, 64U);
	}
	if (!P256ScalarInRange(pPrivKey))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!OpAcquire())
	{
		return CRYPTO_STATUS_BUSY;
	}

	uint8_t pub[64];
	bool ok = WbaPkaScalarMul(pPrivKey, s_Gx, s_Gy, pub, pub + 32);

	WbaPkaWipeRam();
	OpRelease();
	if (!ok)
	{
		CryptoSecureWipe(pub, sizeof(pub));
		return CRYPTO_STATUS_FAIL;
	}
	memcpy(pk->PrivKey, pPrivKey, 32U);
	pk->bKeyValid = true;
	if (pPubKey != nullptr)
	{
		memcpy(pPubKey, pub, sizeof(pub));
	}
	CryptoSecureWipe(pub, sizeof(pub));
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoWbaPka::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
								  const uint8_t *pPeerPubKey,
								  uint8_t *pSharedX, bool bKeepKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
		pPeerPubKey == nullptr || pSharedX == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	KeyCtx *pk = (KeyCtx *)pKeyCtx;

	memset(pSharedX, 0, 32U);
	if (!pk->bKeyValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!vbValid)
	{
		KeyReset(pk);
		return CRYPTO_STATUS_FAIL;
	}
	if (!OpAcquire())
	{
		return CRYPTO_STATUS_BUSY;
	}

	uint8_t rx[32];
	uint8_t ry[32];
	bool ok = WbaPkaPointCheck(pPeerPubKey, pPeerPubKey + 32) == 1 &&
			  WbaPkaScalarMul(pk->PrivKey, pPeerPubKey, pPeerPubKey + 32, rx,
							  ry);

	WbaPkaWipeRam();
	OpRelease();
	if (ok)
	{
		memcpy(pSharedX, rx, 32U);
	}
	CryptoSecureWipe(rx, sizeof(rx));
	CryptoSecureWipe(ry, sizeof(ry));
	if (!ok)
	{
		KeyReset(pk);
		return CRYPTO_STATUS_FAIL;
	}
	if (!bKeepKey)
	{
		KeyReset(pk);
	}
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoWbaPka::Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
								 const uint8_t *pHash, size_t HashLen,
								 uint8_t *pSig)
{
	if (Curve != CRYPTO_CURVE_P256 || Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		Key.Type != CRYPTO_KEY_ECC_P256 ||
		(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U ||
		pHash == nullptr || pSig == nullptr || Key.Plain.pData == nullptr ||
		Key.Plain.Len != 32U)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (vpRng == nullptr || !vpRng->IsSecure())
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	memset(pSig, 0, 64U);
	if (!vbValid || !P256ScalarInRange(Key.Plain.pData))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!OpAcquire())
	{
		return CRYPTO_STATUS_BUSY;
	}

	uint8_t e[32];
	uint8_t k[32];
	int rc = 1;
	CRYPTO_STATUS result = CRYPTO_STATUS_FAIL;

	WbaHashToScalar(pHash, HashLen, e);
	for (int attempt = 0; attempt < CRYPTO_WBA_SIGN_RETRY && rc == 1; attempt++)
	{
		CRYPTO_STATUS st = P256RandomScalarStatus(vpRng, k);

		if (st != CRYPTO_STATUS_OK)
		{
			rc = -1;
			result = st == CRYPTO_STATUS_BUSY ? CRYPTO_STATUS_BUSY :
				CRYPTO_STATUS_FAIL;
			break;
		}
		rc = WbaPkaSignWithK(Key.Plain.pData, e, k, pSig);
		WbaPkaWipeRam();
	}
	CryptoSecureWipe(k, sizeof(k));
	CryptoSecureWipe(e, sizeof(e));
	OpRelease();

	if (rc == 0)
	{
		return CRYPTO_STATUS_OK;
	}
	memset(pSig, 0, 64U);
	return result;
}

CRYPTO_STATUS CryptoWbaPka::Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
								   const uint8_t *pHash, size_t HashLen,
								   const uint8_t *pSig)
{
	if (Curve != CRYPTO_CURVE_P256 || pPubKey == nullptr ||
		pHash == nullptr || pSig == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!P256ScalarInRange(pSig) || !P256ScalarInRange(pSig + 32))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!OpAcquire())
	{
		return CRYPTO_STATUS_BUSY;
	}

	uint8_t e[32];
	WbaHashToScalar(pHash, HashLen, e);

	int valid = WbaPkaPointCheck(pPubKey, pPubKey + 32);

	if (valid == 1)
	{
		valid = WbaPkaVerify(pPubKey, e, pSig);
	}
	WbaPkaWipeRam();
	OpRelease();
	return valid == 1 ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

// Vectors of the accelerator checks: the ECDH sample shared with
// CryptoUecc::SelfTest, and RFC 6979 A.2.5 (P-256, SHA-256, "sample").
static const uint8_t s_TvPrivA[32] = {
	0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
	0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd };
static const uint8_t s_TvPubB[64] = {
	0x1e,0xa1,0xf0,0xf0,0x1f,0xaf,0x1d,0x96,0x09,0x59,0x22,0x84,0xf1,0x9e,0x4c,0x00,
	0x47,0xb5,0x8a,0xfd,0x86,0x15,0xa6,0x9f,0x55,0x90,0x77,0xb2,0x2f,0xaa,0xa1,0x90,
	0x4c,0x55,0xf3,0x3e,0x42,0x9d,0xad,0x37,0x73,0x56,0x70,0x3a,0x9a,0xb8,0x51,0x60,
	0x47,0x2d,0x11,0x30,0xe2,0x8e,0x36,0x76,0x5f,0x89,0xaf,0xf9,0x15,0xb1,0x21,0x4a };
static const uint8_t s_TvDh[32] = {
	0xec,0x02,0x34,0xa3,0x57,0xc8,0xad,0x05,0x34,0x10,0x10,0xa6,0x0a,0x39,0x7d,0x9b,
	0x99,0x79,0x6b,0x13,0xb4,0xf8,0x66,0xf1,0x86,0x8d,0x34,0xf3,0x73,0xbf,0xa6,0x98 };

// RFC 6979 A.2.5, P-256 with SHA-256 of "sample".
static const uint8_t s_TvSigD[32] = {
	0xc9,0xaf,0xa9,0xd8,0x45,0xba,0x75,0x16,0x6b,0x5c,0x21,0x57,0x67,0xb1,0xd6,0x93,
	0x4e,0x50,0xc3,0xdb,0x36,0xe8,0x9b,0x12,0x7b,0x8a,0x62,0x2b,0x12,0x0f,0x67,0x21 };
static const uint8_t s_TvSigPub[64] = {
	0x60,0xfe,0xd4,0xba,0x25,0x5a,0x9d,0x31,0xc9,0x61,0xeb,0x74,0xc6,0x35,0x6d,0x68,
	0xc0,0x49,0xb8,0x92,0x3b,0x61,0xfa,0x6c,0xe6,0x69,0x62,0x2e,0x60,0xf2,0x9f,0xb6,
	0x79,0x03,0xfe,0x10,0x08,0xb8,0xbc,0x99,0xa4,0x1a,0xe9,0xe9,0x56,0x28,0xbc,0x64,
	0xf2,0xf1,0xb2,0x0c,0x2d,0x7e,0x9f,0x51,0x77,0xa3,0xc2,0x94,0xd4,0x46,0x22,0x99 };
static const uint8_t s_TvSigHash[32] = {
	0xaf,0x2b,0xdb,0xe1,0xaa,0x9b,0x6e,0xc1,0xe2,0xad,0xe1,0xd6,0x94,0xf4,0x1f,0xc7,
	0x1a,0x83,0x1d,0x02,0x68,0xe9,0x89,0x15,0x62,0x11,0x3d,0x8a,0x62,0xad,0xd1,0xbf };
static const uint8_t s_TvSigK[32] = {
	0xa6,0xe3,0xc5,0x7d,0xd0,0x1a,0xbe,0x90,0x08,0x65,0x38,0x39,0x83,0x55,0xdd,0x4c,
	0x3b,0x17,0xaa,0x87,0x33,0x82,0xb0,0xf2,0x4d,0x61,0x29,0x49,0x3d,0x8a,0xad,0x60 };
static const uint8_t s_TvSigExpect[64] = {
	0xef,0xd4,0x8b,0x2a,0xac,0xb6,0xa8,0xfd,0x11,0x40,0xdd,0x9c,0xd4,0x5e,0x81,0xd6,
	0x9d,0x2c,0x87,0x7b,0x56,0xaa,0xf9,0x91,0xc3,0x4d,0x0e,0xa8,0x4e,0xaf,0x37,0x16,
	0xf7,0xcb,0x1c,0x94,0x2d,0x65,0x7c,0x41,0xd4,0x36,0xc7,0xa1,0xb6,0xe2,0x9f,0x65,
	0xf3,0xe9,0x00,0xdb,0xb9,0xaf,0xf4,0x06,0x4d,0xc4,0xab,0x2f,0x84,0x3a,0xcd,0xa8 };

int CryptoWbaPka::SelfTest()
{
	if (!vbValid)
	{
		return -1;
	}
	if (!OpAcquire())
	{
		return -1;
	}

	int rc = 0;
	uint8_t rx[32];
	uint8_t ry[32];
	uint8_t sig[64];

	if (WbaPkaPointCheck(s_TvPubB, s_TvPubB + 32) != 1)
	{
		rc = -2;
	}
	else if (!WbaPkaScalarMul(s_TvPrivA, s_TvPubB, s_TvPubB + 32, rx, ry) ||
			 memcmp(rx, s_TvDh, sizeof(s_TvDh)) != 0)
	{
		rc = -3;
	}
	else
	{
		uint8_t bad[64];

		memcpy(bad, s_TvPubB, sizeof(bad));
		bad[63] ^= 1U;
		if (WbaPkaPointCheck(bad, bad + 32) != 0)
		{
			rc = -4;
		}
	}
	if (rc == 0)
	{
		if (WbaPkaSignWithK(s_TvSigD, s_TvSigHash, s_TvSigK, sig) != 0 ||
			memcmp(sig, s_TvSigExpect, sizeof(sig)) != 0)
		{
			rc = -5;
		}
		else if (WbaPkaVerify(s_TvSigPub, s_TvSigHash, sig) != 1)
		{
			rc = -6;
		}
		else
		{
			uint8_t h2[32];

			memcpy(h2, s_TvSigHash, sizeof(h2));
			h2[0] ^= 1U;
			if (WbaPkaVerify(s_TvSigPub, h2, sig) != 0)
			{
				rc = -7;
			}
		}
	}
	WbaPkaWipeRam();
	OpRelease();
	CryptoSecureWipe(rx, sizeof(rx));
	CryptoSecureWipe(ry, sizeof(ry));
	return rc;
}

CryptoWbaPka *CryptoWbaPkaCreate(void *pMem, size_t MemSize, RngEngine *pRng)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoWbaPka) ||
		((uintptr_t)pMem & (alignof(CryptoWbaPka) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoWbaPka *p = new (pMem) CryptoWbaPka();
	p->SetRng(pRng);
	if (!p->Enable())
	{
		p->~CryptoWbaPka();
		return nullptr;
	}
	return p;
}
