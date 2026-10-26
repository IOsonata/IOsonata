/**-------------------------------------------------------------------------
@file	ccrng_test.cpp

@brief	Host test support: ccrng test.

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

#include "crypto_rng_nrf.h"
#include "cc3xx_intrf.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
int g_HashFailAfter = -1;
uintptr_t Cc3xxBase()
{
	return 0;
}
bool Cc3xxEnable()
{
	return true;
}
void Cc3xxDisable()
{
}
class Entropy : public Cc3xxIntrf {
public:
	unsigned draws = 0, stops = 0, serial = 0;
	bool busy = false, fail = false, active = false;
	bool StartRx(uint32_t addr) override {
		assert(addr == CC3XX_ADDR_RNG);
		if (busy)
			return false;
		assert(!active);
		active = true;
		return true;
	}
	int RxData(uint8_t *out, int len) override {
		assert(active && len == 48);
		++draws;
		for (int i = 0; i < len; ++i)
			out[i] = (uint8_t)(serial * 73U + i);
		++serial;
		return fail ? len - 1 : len;
	}
	void StopRx() override
	{
		assert(active);
		active = false;
		++stops;
	}
};
static void CheckZero(const uint8_t *p, size_t n) {
	for (size_t i = 0; i < n; ++i)
		assert(p[i] == 0);
}
int main(int argc, char **argv)
{
	Entropy entropy;
	CryptoRngNrf rng;
	std::vector<uint8_t> out(70002, 0xA5);
	assert(entropy.Init());
	assert(rng.Init(&entropy));
	if (argc == 2) {
		FILE *file = fopen(argv[1], "wb");
		assert(file);
		// Includes byte-limit reseeding inside a single request, then the call limit.
		for (size_t len : {1U, 16U, 33U, 1024U, 65536U, 70000U}) {
			assert(rng.Random(out.data()+1, len) == CRYPTO_STATUS_OK);
			assert(fwrite(out.data()+1, 1, len, file) == len);
		}
		for (unsigned i = 0; i < 1030; ++i) {
			assert(rng.Random(out.data()+1, 1) == CRYPTO_STATUS_OK);
			assert(fwrite(out.data()+1, 1, 1, file) == 1);
		}
		assert(fclose(file) == 0);
		assert(entropy.draws == 4 && entropy.draws == entropy.stops);
	}
	rng.Reset();
	assert(rng.Random(nullptr, 0) == CRYPTO_STATUS_OK);
	unsigned draws = entropy.draws;
	for (unsigned i = 0; i < 100; ++i) {
		assert(rng.Random(out.data()+1, 1) == CRYPTO_STATUS_OK);
	}
	assert(entropy.draws == draws+1);
	{
		int owner;
		draws = entropy.draws;
		assert(Cc3xxIntrfInstance()->OpHold(&owner));
		assert(rng.Random(out.data()+1, 33) == CRYPTO_STATUS_BUSY);
		CheckZero(out.data()+1, 33);
		assert(Cc3xxIntrfInstance()->OpRelease(&owner));
		assert(rng.Random(out.data()+1, 33) == CRYPTO_STATUS_OK);
		assert(entropy.draws == draws);
	}
	rng.Reset();
	entropy.busy = true;
	memset(out.data(), 0xA5, out.size());
	assert(rng.Random(out.data()+1, 33) == CRYPTO_STATUS_BUSY);
	CheckZero(out.data()+1, 33);
	assert(out[0] == 0xA5 && out[34] == 0xA5);
	entropy.busy = false;
	entropy.fail = true;
	assert(rng.Random(out.data()+1, 33) == CRYPTO_STATUS_FAIL);
	CheckZero(out.data()+1, 33);
	entropy.fail = false;
	// Fault every HMAC hash start through seed, output and final state update.
	for (int fault = 0; fault < 16; ++fault) {
		rng.Reset();
		g_HashFailAfter = fault;
		memset(out.data(), 0xA5, out.size());
		assert(rng.Random(out.data()+1, 64) == CRYPTO_STATUS_FAIL);
		CheckZero(out.data()+1, 64);
		assert(out[0] == 0xA5 && out[65] == 0xA5);
		g_HashFailAfter = -1;
		draws = entropy.draws;
		assert(rng.Random(out.data()+1, 64) == CRYPTO_STATUS_OK);
		assert(entropy.draws == draws+1);
	}
	// Reseed failure must erase even the prefix already generated in this call.
	rng.Reset();
	assert(rng.Random(out.data()+1, 65530) == CRYPTO_STATUS_OK);
	entropy.fail = true;
	memset(out.data(), 0xA5, out.size());
	assert(rng.Random(out.data()+1, 33) == CRYPTO_STATUS_FAIL);
	CheckZero(out.data()+1, 33);
	entropy.fail = false;
	rng.Disable();
	assert(rng.Random(out.data()+1, 1) == CRYPTO_STATUS_FAIL);
	assert(rng.Enable());
	draws = entropy.draws;
	assert(rng.Random(out.data()+1, 1) == CRYPTO_STATUS_OK);
	assert(entropy.draws == draws+1);
	assert(!entropy.active && entropy.draws == entropy.stops);
	rng.Disable();
	puts("CryptoCell DRBG: reseed, lifecycle, bounds, busy and failures passed");
}
