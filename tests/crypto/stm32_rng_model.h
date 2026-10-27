/**-------------------------------------------------------------------------
@file	stm32_rng_model.h

@brief	Host test support: stm32 rng model.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#pragma once
#include <stdint.h>
#define IOSONATA_STM32_HAS_RNG 1
#define RCC_AHB2ENR_RNGEN 1U
#define RCC_AHB2RSTR_RNGRST 1U
#define RNG_CR_RNGEN 4U
#define RNG_SR_DRDY 1U
#define RNG_SR_CECS 2U
#define RNG_SR_SECS 4U
#define RNG_SR_CEIS 32U
#define RNG_SR_SEIS 64U
struct RngReg
{
	unsigned id;
	uint32_t value;
	operator uint32_t() const;
	RngReg &operator=(uint32_t v)
	{
		value = v;
		return *this;
	}
	RngReg &operator|=(uint32_t v)
	{
		return *this = uint32_t(*this) | v;
	}
	RngReg &operator&=(uint32_t v)
	{
		return *this = uint32_t(*this) & v;
	}
};
struct RngRegisters
{
	RngReg CR{0,0}, SR{1,0}, DR{2,0};
};
struct RccRegisters
{
	uint32_t AHB2ENR, AHB2RSTR;
};
extern RngRegisters g_Rng;
extern RccRegisters g_Rcc;
#define RNG (&g_Rng)
#define RCC (&g_Rcc)
