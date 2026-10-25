/**-------------------------------------------------------------------------
@file	crypto_sam4l.cpp
@brief	Default crypto engines selected by the target library.

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
#include <string.h>
#include "sam4lxxx.h"
#include "coredev/interrupt.h"
#include "crypto_sam.h"

// Set SAM4L_AES_DIAG to 1 for a temporary hardware/software block comparison.
// Leave disabled for normal operation and performance measurements.
#ifndef SAM4L_AES_DIAG
#define SAM4L_AES_DIAG 0
#endif

#if SAM4L_AES_DIAG
extern "C" {
volatile uint32_t g_Sam4lAesDiagChecks = 0U;
volatile uint32_t g_Sam4lAesDiagEncryptMismatch = 0U;
volatile uint32_t g_Sam4lAesDiagDecryptMismatch = 0U;
volatile uint32_t g_Sam4lAesDiagReferenceFailures = 0U;
}
#endif

static bool s_AesBusy = false;
static bool s_RngBusy = false;

// Debugger-readable counters. No key, seed or plaintext is retained.
extern "C" {
volatile uint32_t g_Sam4lAesBlocks = 0U;
volatile uint32_t g_Sam4lAesCompleted = 0U;
volatile uint32_t g_Sam4lAesRngFailures = 0U;
volatile uint32_t g_Sam4lAesInputTimeouts = 0U;
volatile uint32_t g_Sam4lAesOutputTimeouts = 0U;
volatile uint32_t g_Sam4lAesLastStatus = 0U;
}

static void PmEnable(volatile uint32_t *pReg, uint32_t Mask)
{
	uintptr_t state = DisableInterrupt();
	uint32_t offset = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pReg) -
		reinterpret_cast<uintptr_t>(SAM4L_PM));

	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAU) | PM_UNLOCK_ADDR(offset);
	*pReg |= Mask;
	EnableInterrupt(state);
}

static bool RngWord(uint32_t *pWord)
{
	uint32_t polls = 1000000U;
	uint32_t status;

	while (polls > 0U)
	{
		status = SAM4L_TRNG->TRNG_ISR;
		if ((status & TRNG_ISR_DATRDY) != 0U)
		{
			*pWord = SAM4L_TRNG->TRNG_ODATA;
			return true;
		}
		polls--;
	}
	return false;
}

bool CryptoSam::Enable()
{
	uint32_t polls = 1000000U;
	uint32_t word = 0U;
	uint32_t clock;
	uint32_t hsbMask = PM_HSBMASK_AESA | PM_HSBMASK_HTOP0;
	uint32_t source = SAM4L_PM->PM_MCCTRL & PM_MCCTRL_MCSEL_Msk;
	uint32_t aesClock;

	// Table 13-8: 7 is reserved, not CLK_CPU. Select a running oscillator.
	switch (source)
	{
	case PM_MCCTRL_MCSEL(1U):
	case PM_MCCTRL_MCSEL_PLL0:
		// OSC0 is already running when used directly or as the PLL input.
		aesClock = SCIF_GCCTRL_OSCSEL(3U) | SCIF_GCCTRL_CEN;
		break;
	case PM_MCCTRL_MCSEL_DFLL0:
		// The default DFLL runs at 80 MHz; divide by two for GCLK4.
		aesClock = SCIF_GCCTRL_OSCSEL(2U) | SCIF_GCCTRL_DIVEN |
			SCIF_GCCTRL_DIV(0U) | SCIF_GCCTRL_CEN;
		break;
	case PM_MCCTRL_MCSEL_RC80M:
		aesClock = SCIF_GCCTRL_OSCSEL(4U) | SCIF_GCCTRL_DIVEN |
			SCIF_GCCTRL_DIV(0U) | SCIF_GCCTRL_CEN;
		break;
	case PM_MCCTRL_MCSEL(5U):
		aesClock = SCIF_GCCTRL_OSCSEL(5U) | SCIF_GCCTRL_CEN;
		break;
	case PM_MCCTRL_MCSEL(6U):
		aesClock = SCIF_GCCTRL_OSCSEL(6U) | SCIF_GCCTRL_CEN;
		break;
	default:
		aesClock = SCIF_GCCTRL_OSCSEL(0U) | SCIF_GCCTRL_CEN;
		break;
	}

	if (vbValid)
	{
		return true;
	}
	// TRNG is on PBA; AESA is on HSB and uses generic clock 4.
	PmEnable(&SAM4L_PM->PM_HSBMASK, hsbMask);
	PmEnable(&SAM4L_PM->PM_PBAMASK, PM_PBAMASK_TRNG);
	if ((SAM4L_PM->PM_HSBMASK & hsbMask) != hsbMask ||
		(SAM4L_PM->PM_PBAMASK & PM_PBAMASK_TRNG) == 0U)
	{
		return false;
	}
	clock = SAM4L_SCIF->SCIF_GCCTRL[4].SCIF_GCCTRL;
	SAM4L_SCIF->SCIF_GCCTRL[4].SCIF_GCCTRL = clock & ~SCIF_GCCTRL_CEN;
	while ((SAM4L_SCIF->SCIF_GCCTRL[4].SCIF_GCCTRL & SCIF_GCCTRL_CEN) != 0U && polls > 0U)
	{
		polls--;
	}
	if (polls == 0U)
	{
		return false;
	}
	// Apply the active clock source selected above.
	SAM4L_SCIF->SCIF_GCCTRL[4].SCIF_GCCTRL = aesClock;
	clock = SAM4L_SCIF->SCIF_GCCTRL[4].SCIF_GCCTRL;
	if ((clock & (SCIF_GCCTRL_OSCSEL_Msk | SCIF_GCCTRL_DIVEN |
		SCIF_GCCTRL_DIV_Msk | SCIF_GCCTRL_CEN)) != aesClock)
	{
		return false;
	}
	// ASF enables AESA once, after configuring the generic clock.
	SAM4L_AESA->AESA_CTRL = AESA_CTRL_ENABLE;
	SAM4L_TRNG->TRNG_IDR = TRNG_IDR_DATRDY;
	SAM4L_TRNG->TRNG_CR = TRNG_CR_KEY(0x524E47U) | TRNG_CR_ENABLE;
	if (!RngWord(&word))
	{
		SAM4L_TRNG->TRNG_CR = TRNG_CR_KEY(0x524E47U);
		CryptoSecureWipe(&word, sizeof(word));
		return false;
	}
	CryptoSecureWipe(&word, sizeof(word));
	vbValid = true;
	return true;
}

bool CryptoSam::WideKey() const
{
	// SAM4L implements 128-bit keys, despite the superset register fields.
	return false;
}

bool CryptoSam::AesOpBegin()
{
	uintptr_t state = DisableInterrupt();
	bool ok = !s_AesBusy;

	if (ok)
	{
		s_AesBusy = true;
	}
	EnableInterrupt(state);
	return ok;
}

void CryptoSam::AesOpEnd()
{
	uintptr_t state;
	unsigned i;

	for (i = 0U; i < 4U; i++)
	{
		SAM4L_AESA->AESA_KEY[i].AESA_KEY = 0U;
	}
	state = DisableInterrupt();
	s_AesBusy = false;
	EnableInterrupt(state);
}

bool CryptoSam::Block(const uint8_t *pKey, size_t KeyLen, bool Encrypt,
	const uint8_t In[16], uint8_t Out[16])
{
	uint32_t word = 0U;
	uint32_t polls = 1000000U;
	unsigned i;

	g_Sam4lAesBlocks++;
	if (KeyLen != 16U || Random(reinterpret_cast<uint8_t *>(&word), sizeof(word)) != CRYPTO_STATUS_OK)
	{
		g_Sam4lAesRngFailures++;
		CryptoSecureWipe(Out, 16U);
		return false;
	}
	// Keep AESA enabled between blocks, as in the ASF SAM4L driver.
	SAM4L_AESA->AESA_MODE = AESA_MODE_CTYPE(15U) | (Encrypt ? AESA_MODE_ENCRYPT : 0U);
	SAM4L_AESA->AESA_DRNGSEED = word;
	SAM4L_AESA->AESA_CTRL = AESA_CTRL_ENABLE | AESA_CTRL_NEWMSG;
	for (i = 0U; i < 4U; i++)
	{
		memcpy(&word, pKey + i * 4U, sizeof(word));
		SAM4L_AESA->AESA_KEY[i].AESA_KEY = word;
	}
	while ((SAM4L_AESA->AESA_SR & AESA_SR_IBUFRDY) == 0U && polls > 0U)
	{
		polls--;
	}
	if (polls == 0U)
	{
		g_Sam4lAesInputTimeouts++;
		g_Sam4lAesLastStatus = SAM4L_AESA->AESA_SR;
		CryptoSecureWipe(Out, 16U);
		CryptoSecureWipe(&word, sizeof(word));
		return false;
	}
	for (i = 0U; i < 4U; i++)
	{
		memcpy(&word, In + i * 4U, sizeof(word));
		SAM4L_AESA->AESA_IDATA = word;
	}
	polls = 1000000U;
	while ((SAM4L_AESA->AESA_SR & AESA_SR_ODATARDY) == 0U && polls > 0U)
	{
		polls--;
	}
	if (polls == 0U)
	{
		g_Sam4lAesOutputTimeouts++;
		g_Sam4lAesLastStatus = SAM4L_AESA->AESA_SR;
		CryptoSecureWipe(Out, 16U);
		CryptoSecureWipe(&word, sizeof(word));
		return false;
	}
	for (i = 0U; i < 4U; i++)
	{
		word = SAM4L_AESA->AESA_ODATA;
		memcpy(Out + i * 4U, &word, sizeof(word));
	}
	g_Sam4lAesLastStatus = SAM4L_AESA->AESA_SR;
	g_Sam4lAesCompleted++;
#if SAM4L_AES_DIAG
	{
		CryptoSoftAes reference;
		CryptoKey key = {};
		uint8_t expected[16];
		CRYPTO_STATUS status;

		key.Type = CRYPTO_KEY_AES_128;
		key.Loc = CRYPTO_KEY_LOC_PLAIN;
		key.Usage = Encrypt ? CRYPTO_KEY_USE_ENCRYPT : CRYPTO_KEY_USE_DECRYPT;
		key.Plain.pData = pKey;
		key.Plain.Len = 16U;
		reference.Enable();
		status = reference.Cipher(CRYPTO_CIPHER_ECB, Encrypt ? 1 : 0,
			key, nullptr, 0U, In, 16U, expected);
		g_Sam4lAesDiagChecks++;
		if (status != CRYPTO_STATUS_OK)
		{
			g_Sam4lAesDiagReferenceFailures++;
		}
		else if (memcmp(Out, expected, sizeof(expected)) != 0)
		{
			if (Encrypt)
			{
				g_Sam4lAesDiagEncryptMismatch++;
			}
			else
			{
				g_Sam4lAesDiagDecryptMismatch++;
			}
		}
		CryptoSecureWipe(expected, sizeof(expected));
	}
#endif
	CryptoSecureWipe(&word, sizeof(word));
	return true;
}

CRYPTO_STATUS CryptoSam::Random(uint8_t *pOut, size_t Len)
{
	uintptr_t state;
	uint32_t word = 0U;
	size_t off;
	size_t count;
	bool ok = true;

	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (pOut == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!vbValid)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	state = DisableInterrupt();
	if (s_RngBusy)
	{
		EnableInterrupt(state);
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}
	s_RngBusy = true;
	EnableInterrupt(state);
	for (off = 0U; off < Len; off += count)
	{
		if (!RngWord(&word))
		{
			ok = false;
			break;
		}
		count = Len - off < sizeof(word) ? Len - off : sizeof(word);
		memcpy(pOut + off, &word, count);
	}
	CryptoSecureWipe(&word, sizeof(word));
	state = DisableInterrupt();
	s_RngBusy = false;
	EnableInterrupt(state);
	if (!ok)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

bool CryptoSam::IsSecure() const
{
	return vbValid;
}

CRYPTO_STATUS CryptoSam::NarrowCipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
	const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	CryptoSoftAes software;

	software.Enable();
	return software.Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
}

extern CryptoSam &CryptoSamInstance();

CRYPTO_STATUS Crypto::Random(uint8_t *pOut, size_t Len)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return CryptoSamInstance().Random(pOut, Len);
}

bool Crypto::IsSecure() const
{
	if (!vbValid)
	{
		return false;
	}
	return CryptoSamInstance().IsSecure();
}

