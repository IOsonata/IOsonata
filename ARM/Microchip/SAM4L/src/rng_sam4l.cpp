/**-------------------------------------------------------------------------
@file	rng_sam4l.cpp

@brief	SAM4L TRNG random number engine.

The TRNG is on the PBA bridge. Enable clocks the bridge (PM_HSBMASK HTOP0)
and the TRNG (PM_PBAMASK), starts it with the TRNG_CR key and waits for the
first word. Random reads one 32-bit word per DATRDY. A word that does not
arrive in time fails the whole request and wipes the caller's buffer; no
software random generator is used instead. The register sequence is the one
the SAM4LC AESA provider used before the engine was shared.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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

#include "sam4lxxx.h"
#include "coredev/interrupt.h"
#include "crypto_rng_sam4l.h"

// Status polls before a TRNG word is reported as missing
#ifndef RNG_SAM4L_POLLS
#define RNG_SAM4L_POLLS		1000000U
#endif

// TRNG_CR writes are taken only with this key ("RNG" in ASCII)
#define RNG_SAM4L_CR_KEY	TRNG_CR_KEY(0x524E47U)

static bool s_RngBusy = false;
static CryptoRngSam4l s_Rng;

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
	uint32_t polls = RNG_SAM4L_POLLS;
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

CryptoRngSam4l &CryptoRngSam4lInstance(void)
{
	return s_Rng;
}

bool CryptoRngSam4l::Enable()
{
	uint32_t word = 0U;

	if (vbValid)
	{
		return true;
	}
	PmEnable(&SAM4L_PM->PM_HSBMASK, PM_HSBMASK_HTOP0);
	PmEnable(&SAM4L_PM->PM_PBAMASK, PM_PBAMASK_TRNG);
	if ((SAM4L_PM->PM_HSBMASK & PM_HSBMASK_HTOP0) == 0U ||
		(SAM4L_PM->PM_PBAMASK & PM_PBAMASK_TRNG) == 0U)
	{
		return false;
	}
	SAM4L_TRNG->TRNG_IDR = TRNG_IDR_DATRDY;
	SAM4L_TRNG->TRNG_CR = RNG_SAM4L_CR_KEY | TRNG_CR_ENABLE;
	if (!RngWord(&word))
	{
		SAM4L_TRNG->TRNG_CR = RNG_SAM4L_CR_KEY;
		CryptoSecureWipe(&word, sizeof(word));
		return false;
	}
	CryptoSecureWipe(&word, sizeof(word));
	vbValid = true;
	return true;
}

void CryptoRngSam4l::Disable()
{
	vbValid = false;
	SAM4L_TRNG->TRNG_CR = RNG_SAM4L_CR_KEY;
}

void CryptoRngSam4l::Reset()
{
	Disable();
	(void)Enable();
}

CRYPTO_STATUS CryptoRngSam4l::Random(uint8_t *pOut, size_t Len)
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
