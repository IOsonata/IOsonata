/**-------------------------------------------------------------------------
@example	crypto_softrng_test.cpp

@brief	Host validation for the OO CryptoSoftRng engine.

@author	Hoang Nguyen Hoan
@date	Jul. 15, 2026

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
#include <stdio.h>
#include <string.h>

#include "crypto/crypto_softrng.h"

static int s_pass, s_fail;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

int main(void)
{
	printf("CryptoSoftRng OO engine validation\n");
	alignas(CryptoSoftRng) static uint8_t mem[CRYPTO_SOFTRNG_MEMSIZE];
	CryptoSoftRng *engine = CryptoSoftRngCreate(mem, sizeof(mem));
	check("factory constructs aligned engine", engine != nullptr);
	if (engine == nullptr) return 1;

	RngEngine *rng = engine;
	check("usable through RngEngine facet", rng != nullptr);
	check("software PRNG is not security grade", !rng->IsSecure());

	uint8_t first[64], second[64];
	engine->Seed(0x1111222233334444ULL, 0x5555666677778888ULL);
	rng->Random(first, sizeof(first));
	engine->Seed(0x1111222233334444ULL, 0x5555666677778888ULL);
	rng->Random(second, sizeof(second));
	check("same seed reproduces the sequence",
		memcmp(first, second, sizeof(first)) == 0);

	engine->Seed(0x1111222233334444ULL, 0x5555666677778888ULL);
	rng->Random(first, sizeof(first));
	engine->Seed(0xAAAABBBBCCCCDDDDULL, 0x1234567890ABCDEFULL);
	rng->Random(second, sizeof(second));
	check("different seed changes the sequence",
		memcmp(first, second, sizeof(first)) != 0);

	uint8_t exact[19];
	memset(exact, 0xCC, sizeof(exact));
	check("non-aligned request succeeds",
		rng->Random(exact, 17) == CRYPTO_STATUS_OK);
	check("request does not overrun destination length",
		exact[17] == 0xCC && exact[18] == 0xCC);

	engine->Seed(0xC0FFEE1234567890ULL, 0x0BADF00DDEADBEEFULL);
	uint32_t seen[256]; memset(seen, 0, sizeof(seen));
	uint64_t sum = 0U;
	uint8_t chunk[256];
	for (int produced = 0; produced < 65536; produced += (int)sizeof(chunk))
	{
		rng->Random(chunk, sizeof(chunk));
		for (size_t i = 0; i < sizeof(chunk); i++)
		{
			seen[chunk[i]]++;
			sum += chunk[i];
		}
	}
	uint32_t meanT10 = (uint32_t)((sum * 10U) / 65536U);
	int missing = 0;
	for (int value = 0; value < 256; value++)
	{
		if (seen[value] == 0U) missing++;
	}
	check("basic distribution sanity",
		meanT10 > 1200U && meanT10 < 1350U && missing == 0);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
