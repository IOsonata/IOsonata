/**-------------------------------------------------------------------------
@file	rng_stm32.cpp

@brief	STM32 hardware RNG provider: CryptoRngStm32 on the RngEngine facet.

		One implementation for STM32 families that have the RNG peripheral.
		This file does not call the STM32 HAL or LL drivers. It includes the
		common IOsonata STM32 CMSIS selector and accesses RCC/RNG registers
		directly.

		STM32F0/F030 does not have hardware RNG and must not link this file.

		This code enables the RNG peripheral clock and the RNG block. It does not
		rebuild the system clock tree. If the RNG kernel clock is not valid, the
		status register reports a clock error and Random reports failure. Configure
		the RNG clock source in the board/system clock code.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include <new>

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

static void RngClockEnable(void)
{
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
	// STM32 RNG error interrupt flags are cleared by writing 0 to the flag bits.
	RNG->SR &= ~(RNG_SR_CEIS | RNG_SR_SEIS);
#else
	(void)RNG->SR;
#endif
}

static void RngRecoverSeedError(void)
{
	RngDisable();
	RngClearLatchedErrors();

#if defined(RNG_CR_CONDRST)
	RNG->CR |= RNG_CR_CONDRST;
	RNG->CR &= ~RNG_CR_CONDRST;
#endif

	RngEnable();
}

static bool RngInit(void)
{
	if (s_RngInited && ((RNG->CR & RNG_CR_RNGEN) != 0U))
	{
		return true;
	}

	RngClockEnable();
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

		if ((sr & RNG_SR_SECS) != 0U)
		{
			// A seed error invalidates the current output stream. Restart RNG and
			// wait for a new valid word.
			RngRecoverSeedError();
			continue;
		}

		if ((sr & RNG_SR_CECS) != 0U)
		{
			// Clock error means the RNG source clock is not valid for output.
			return false;
		}

		if ((sr & RNG_SR_DRDY) != 0U)
		{
			*pWord = RNG->DR;
			return true;
		}
	}

	return false;
}

// Core fill: assumes RngInit has succeeded. Fills pBuff with Len random bytes,
// returns false on a hardware error (clock error or persistent seed error).
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

//-----------------------------------------------------------------------------
// OO engine.
//-----------------------------------------------------------------------------
bool CryptoRngStm32::Enable()
{
	bool ok = RngInit();
	vbValid = ok;
	return ok;
}

CRYPTO_STATUS CryptoRngStm32::Random(uint8_t *pOut, size_t Len)
{
	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (pOut == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (RngInit() == false)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	if (RngFill(pOut, Len) == false)
	{
		// RngFill writes words in place and can stop part way on a hardware
		// error, leaving some words of the output filled. Wipe the whole
		// requested range so a failed draw never exposes a partial result to a
		// caller that forgets to clear it.
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

CryptoRngStm32 *CryptoRngStm32Instance(void)
{
	// Singleton in internal static storage (no allocation). The peripheral holds
	// the hardware state; the engine object is stateless.
	static CryptoRngStm32 s_Instance;
	return &s_Instance;
}

