/**-------------------------------------------------------------------------
@file	rng_stm32.cpp

@brief	STM32 hardware RNG provider: CryptoRngStm32 on the RngEngine facet.

		One implementation for STM32 families that have the RNG peripheral.
		This file does not call the STM32 HAL or LL drivers. It includes the
		common IOsonata STM32 CMSIS selector and accesses RCC/RNG registers
		directly.

		STM32F0/F030 does not have hardware RNG and must not link this file.

		H753 selects HSI48 for the RNG kernel without changing PLL or USB clock
		selection. Other families require a valid RNG kernel clock configured by
		their system clock code. Clock and data-ready waits are bounded.

@author	Hoang Nguyen Hoan
@date	Jul 2026

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
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "stm32.h"
#include "crypto_rng_stm32.h"

#ifndef RNG_STM32_TIMEOUT
#define RNG_STM32_TIMEOUT			1000000U
#endif

#if (IOSONATA_STM32_HAS_RNG != 1)
#error "rng_stm32.cpp: selected STM32 device does not have a hardware RNG"
#endif

#ifndef RNG_SR_CECS
#define RNG_SR_CECS					0U
#endif

#ifndef RNG_SR_SECS
#define RNG_SR_SECS					0U
#endif

#ifndef RNG_SR_CEIS
#define RNG_SR_CEIS					0U
#endif

#ifndef RNG_SR_SEIS
#define RNG_SR_SEIS					0U
#endif

static volatile bool s_RngInited = false;
static atomic_flag s_RngBusy = ATOMIC_FLAG_INIT;

static bool RngClockEnable(void)
{
#if defined(STM32H753xx)
	RCC->CR |= RCC_CR_HSI48ON;
	uint32_t polls = RNG_STM32_TIMEOUT;

	while ((RCC->CR & RCC_CR_HSI48RDY) == 0U && polls > 0U)
	{
		polls--;
	}
	if ((RCC->CR & RCC_CR_HSI48RDY) == 0U)
	{
		return false;
	}
	// RNGSEL=0 selects HSI48. USBSEL and all PLL settings are preserved.
	RCC->D2CCIP2R &= ~RCC_D2CCIP2R_RNGSEL;
#endif
#if defined(RCC_AHB2ENR_RNGEN)
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
	(void)RCC->AHB2ENR;
#elif defined(RCC_AHB2ENR1_RNGEN)
	RCC->AHB2ENR1 |= RCC_AHB2ENR1_RNGEN;
	(void)RCC->AHB2ENR1;
#elif defined(RCC_AHB2ENR2_RNGEN)
	RCC->AHB2ENR2 |= RCC_AHB2ENR2_RNGEN;
	(void)RCC->AHB2ENR2;
#elif defined(RCC_AHB3ENR_RNGEN)
	RCC->AHB3ENR |= RCC_AHB3ENR_RNGEN;
	(void)RCC->AHB3ENR;
#else
#error "rng_stm32.cpp: selected STM32 device header does not define RNG clock enable bit"
#endif
	return true;
}

static void RngReset(void)
{
#if defined(RCC_AHB2RSTR_RNGRST)
	RCC->AHB2RSTR |= RCC_AHB2RSTR_RNGRST;
	RCC->AHB2RSTR &= ~RCC_AHB2RSTR_RNGRST;
#elif defined(RCC_AHB2RSTR1_RNGRST)
	RCC->AHB2RSTR1 |= RCC_AHB2RSTR1_RNGRST;
	RCC->AHB2RSTR1 &= ~RCC_AHB2RSTR1_RNGRST;
#elif defined(RCC_AHB2RSTR2_RNGRST)
	RCC->AHB2RSTR2 |= RCC_AHB2RSTR2_RNGRST;
	RCC->AHB2RSTR2 &= ~RCC_AHB2RSTR2_RNGRST;
#elif defined(RCC_AHB3RSTR_RNGRST)
	RCC->AHB3RSTR |= RCC_AHB3RSTR_RNGRST;
	RCC->AHB3RSTR &= ~RCC_AHB3RSTR_RNGRST;
#else
	// Some STM32 device headers do not expose a separate RNG reset bit.
#endif
}

static void RngDisable(void)
{
	RNG->CR &= ~RNG_CR_RNGEN;
	(void)RNG->CR;
}

static void RngEnable(void)
{
	RNG->CR |= RNG_CR_RNGEN;
	(void)RNG->CR;
}

static void RngClearLatchedErrors(void)
{
#if (RNG_SR_CEIS != 0U) || (RNG_SR_SEIS != 0U)
	RNG->SR &= ~(RNG_SR_CEIS | RNG_SR_SEIS);
#else
	(void)RNG->SR;
#endif
}

static bool RngInit(void)
{
	if (s_RngInited && ((RNG->CR & RNG_CR_RNGEN) != 0U))
	{
		return true;
	}

	if (RngClockEnable() == false)
	{
		return false;
	}
	RngReset();
	RngClearLatchedErrors();
	RngEnable();

	s_RngInited = true;
	return true;
}

static bool RngReadWord(uint32_t *pWord)
{
	if (pWord == NULL)
	{
		return false;
	}

	for (uint32_t i = 0; i < RNG_STM32_TIMEOUT; i++)
	{
		uint32_t sr = RNG->SR;

		// Current and latched health errors invalidate this entire request.
		// Never clear an error and return subsequent data as if it did not occur.
		if (sr & (RNG_SR_SECS | RNG_SR_SEIS | RNG_SR_CECS | RNG_SR_CEIS))
		{
			return false;
		}

		if ((sr & RNG_SR_DRDY) != 0U)
		{
			*pWord = RNG->DR;
			if (RNG->SR & (RNG_SR_SECS | RNG_SR_SEIS | RNG_SR_CECS | RNG_SR_CEIS))
			{
				*pWord = 0;

				return false;
			}
			return true;
		}
	}

	return false;
}

static bool RngFill(uint8_t *pBuff, size_t Len)
{
	while (Len >= sizeof(uint32_t))
	{
		uint32_t w;

		if (RngReadWord(&w) == false)
		{
			return false;
		}

		memcpy(pBuff, &w, sizeof(w));
		pBuff += sizeof(w);
		Len -= sizeof(w);
	}

	if (Len > 0U)
	{
		uint32_t w;

		if (RngReadWord(&w) == false)
		{
			return false;
		}

		memcpy(pBuff, &w, Len);
	}

	return true;
}

bool CryptoRngStm32::Enable()
{
	if (vbEnabled && vbValid)
	{
		return true;
	}
	if (atomic_flag_test_and_set(&s_RngBusy))
	{
		return false;
	}
	bool ok = RngInit();
	vbEnabled = ok;
	vbValid = ok;
	atomic_flag_clear(&s_RngBusy);
	return ok;
}

void CryptoRngStm32::Disable()
{
	if (atomic_flag_test_and_set(&s_RngBusy))
	{
		return;
	}
	vbValid = false;
	if (vbEnabled)
	{
		RngDisable();
		vbEnabled = false;
	}
	atomic_flag_clear(&s_RngBusy);
}

void CryptoRngStm32::Reset()
{
	if (atomic_flag_test_and_set(&s_RngBusy))
	{
		return;
	}
	vbValid = false;
	RngDisable();
	s_RngInited = false;
	bool ok = RngInit();
	vbEnabled = ok;
	vbValid = ok;
	atomic_flag_clear(&s_RngBusy);
}

CRYPTO_STATUS CryptoRngStm32::Random(uint8_t *pOut, size_t Len)
{
	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (!vbValid || !vbEnabled || pOut == nullptr)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	if (atomic_flag_test_and_set(&s_RngBusy))
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}

	bool ok = RngFill(pOut, Len);
	if (ok == false)
	{
		RngDisable();
		vbValid = false;
		vbEnabled = false;
	}
	atomic_flag_clear(&s_RngBusy);
	if (!ok)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

CryptoRngStm32 *CryptoRngStm32Instance(void)
{
	static CryptoRngStm32 s_Instance;
	static bool s_Initialized = false;
	if (!s_Initialized)
	{
		s_Initialized = s_Instance.Enable();
	}
	return s_Initialized ? &s_Instance : nullptr;
}
