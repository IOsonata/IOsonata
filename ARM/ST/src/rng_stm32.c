/**-------------------------------------------------------------------------
@file	rng_stm32.c

@brief	STM32 hardware RNG provider using direct CMSIS register access.

		One implementation for STM32 families that have the RNG peripheral.
		This file does not call the STM32 HAL or LL drivers. It includes the
		common IOsonata STM32 CMSIS selector and accesses RCC/RNG registers
		directly.

		STM32F0/F030 does not have hardware RNG and must not link this file.

		This code enables the RNG peripheral clock and the RNG block. It does not
		rebuild the system clock tree. If the RNG kernel clock is not valid, the
		status register reports a clock error and RngGet returns false. Configure
		the RNG clock source in the board/system clock code.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "stm32.h"
#include "crypto/crypto.h"

#ifndef RNG_STM32_TIMEOUT
#define RNG_STM32_TIMEOUT			1000000U
#endif

#if (IOSONATA_STM32_HAS_RNG != 1)
#error "rng_stm32.c: selected STM32 device does not have a hardware RNG"
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
#error "rng_stm32.c: selected STM32 device header does not define RNG clock enable bit"
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

bool RngInit(void)
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

bool RngGet(uint8_t *pBuff, size_t Len)
{
	if (Len == 0U)
	{
		return true;
	}

	if (pBuff == NULL)
	{
		return false;
	}

	if (RngInit() == false)
	{
		return false;
	}

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
