/**-------------------------------------------------------------------------
@file	entropy_model.cpp

@brief	Host test support: entropy model.

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

#include "entropy_model.h"
#include "crypto/crypto.h"
#include <assert.h>
#include <openssl/rand.h>
#include <stdio.h>
#include <string.h>
unsigned g_Reads = 0, g_DiscardReads = 0;
bool g_Timeout = false, g_ModuleRunning = true;
EntropyRegs g_Trng;
OtpApi g_Otp{ModelRomRead};
RomApi *g_RomPointer = &g_Rom;
RomApi g_Rom{&g_Otp};
static uint8_t s_Seed[4];
static unsigned s_Read;
EntropyReg &EntropyReg::operator=(uint8_t v)
{
	value = v;
	if (id == 2 && (v & R9A02_TRNG_ST)) {
		assert(s_Read == 0 || s_Read == 4);
		assert(RAND_bytes(s_Seed, 4) == 1);
		s_Read = 0;
		if (!g_Timeout)
			value |= R9A02_TRNG_RDY;
	}
	return *this;
}
EntropyReg::operator uint8_t() const
{
	if (id == 1) {
		assert(g_Trng.TRNGSCR0.value & R9A02_TRNG_RDY);
		assert(s_Read < 4);
		g_Reads++;
		uint8_t v = s_Seed[s_Read++];
		if (s_Read == 4)
			g_Trng.TRNGSCR0.value &= ~R9A02_TRNG_RDY;
		return v;
	}
	return value;
}
uint32_t ModelRomRead()
{
	uint32_t v;
	assert(RAND_bytes(reinterpret_cast<unsigned char *>(&v), sizeof(v)) == 1);
	g_Reads++;
	return v;
}
extern int CryptoEngineTest();
int main()
{
	Crypto crypto;
	uint8_t out[9];
	unsigned before;
	assert(CryptoEngineTest() == 0);
	assert(crypto.Init() && crypto.IsSecure());
	for (unsigned len = 0; len <= 7; len++) {
		memset(out, 0xA5, sizeof(out));
		before = g_Reads;
		assert(crypto.Random(out+1, len) == CRYPTO_STATUS_OK);
		assert(out[0] == 0xA5 && out[len+1] == 0xA5);
#ifdef TEST_LPC
		assert(g_Reads - before == ((len+3)/4)*33);
#else
		assert(g_Reads - before == ((len+3)/4)*4);
#endif
	}
	assert(crypto.Random(nullptr, 1) == CRYPTO_STATUS_FAIL);
	assert(crypto.Random(nullptr, 0) == CRYPTO_STATUS_OK);
#ifndef TEST_LPC
	g_Timeout = true;
	memset(out, 0xA5, sizeof(out));
	assert(crypto.Random(out+1, 7) == CRYPTO_STATUS_FAIL);
	for (unsigned i = 1; i < 8; i++)
		assert(out[i] == 0);
	assert(out[0] == 0xA5 && out[8] == 0xA5);
	assert(g_Trng.TRNGSCR0.value == 0);
	g_Timeout = false;
	assert(crypto.Random(out+1, 7) == CRYPTO_STATUS_OK);
	g_ModuleRunning = false;
	Crypto failed;
	assert(!failed.Init());
#endif
	printf("Entropy model: byte bounds, complete seed reads and failures passed\n");
}
