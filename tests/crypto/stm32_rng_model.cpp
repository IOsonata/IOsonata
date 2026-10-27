/**-------------------------------------------------------------------------
@file	stm32_rng_model.cpp

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

#include "stm32_rng_model.h"
#include "crypto_rng_stm32.h"
#include <assert.h>
#include <initializer_list>
#include <string.h>
#include <stdio.h>
RngRegisters g_Rng;
RccRegisters g_Rcc;
static uint32_t s_Error;
static bool s_AfterRead, s_Timeout;
static unsigned s_Reads;
static CryptoRngStm32 *s_Contender;
RngReg::operator uint32_t() const
{
	if (id == 1)
		return (s_Timeout ? 0U : RNG_SR_DRDY) | ((!s_AfterRead || s_Reads) ? s_Error : 0U);
	if (id == 2) {
		if (s_Contender) {
			uint8_t output[2] = {0xA5, 0xA5};
			assert(s_Contender->Random(output, 1) == CRYPTO_STATUS_BUSY);
			assert(output[0] == 0 && output[1] == 0xA5);
		}
		s_Reads++;
		return 0x12345678;
	}
	return value;
}
int main()
{
	CryptoRngStm32 rng, other;
	uint8_t output[7];
	assert(rng.Enable());
	assert(rng.Random(nullptr, 0) == CRYPTO_STATUS_OK);
	assert(other.Enable());
	s_Contender = &other;
	assert(rng.Random(output, 1) == CRYPTO_STATUS_OK);
	s_Contender = nullptr;
	for (uint32_t error : {RNG_SR_SECS, RNG_SR_SEIS, RNG_SR_CECS, RNG_SR_CEIS}) {
		for (bool after : {false, true}) {
			s_Error = 0;
			s_Reads = 0;
			rng.Reset();
			s_Error = error;
			s_AfterRead = after;
			memset(output, 0xA5, sizeof(output));
			assert(rng.Random(output+1, 5) == CRYPTO_STATUS_FAIL);
			assert(output[0] == 0xA5 && output[6] == 0xA5);
			for (unsigned i = 1; i < 6; i++)
				assert(output[i] == 0);
			assert((g_Rng.CR.value & RNG_CR_RNGEN) == 0);
			assert(rng.Random(output+1, 5) == CRYPTO_STATUS_FAIL);
		}
	}
	s_Error = 0;
	s_Timeout = true;
	rng.Reset();
	assert(rng.Random(output+1, 5) == CRYPTO_STATUS_FAIL);
	s_Timeout = false;
	rng.Reset();
	assert(rng.Random(output+1, 5) == CRYPTO_STATUS_OK);
	assert(output[0] == 0xA5 && output[6] == 0xA5);
	puts("STM32 RNG: current/latched errors before and after data, timeout, bounds and reset passed");
}
