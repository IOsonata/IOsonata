/* H753 RNG register tests and existing crypto engine known answers.

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
*/
#include <new>
#include <string.h>
#include "stm32.h"
#include "crypto_rng_stm32.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

extern "C" {
volatile uint32_t g_RngFault;
}

#define CHECK(X) do { if (!(X)) { return __LINE__; } } while (0)

extern "C" int RunRngTest(unsigned Mode)
{
	alignas(CryptoRngStm32) uint8_t mem[sizeof(CryptoRngStm32)];
	CryptoRngStm32 *rng = new (mem) CryptoRngStm32();
	uint8_t data[19];

	g_RngFault = Mode;
	if (Mode == 1)
	{
		CHECK(rng->Enable() == false);
		CHECK(rng->Valid() == false);
		CHECK((RCC->AHB2ENR & RCC_AHB2ENR_RNGEN) == 0);

		return 0;
	}
	uint32_t clocks = RCC->D2CCIP2R;
	CHECK(rng->Enable());
	CHECK((RCC->D2CCIP2R & RCC_D2CCIP2R_RNGSEL) == 0);
	CHECK((RCC->D2CCIP2R & ~RCC_D2CCIP2R_RNGSEL) == (clocks & ~RCC_D2CCIP2R_RNGSEL));
	CHECK((RCC->CR & RCC_CR_HSI48RDY) != 0);
	CHECK((RCC->AHB2ENR & RCC_AHB2ENR_RNGEN) != 0);
	CHECK((RNG->CR & RNG_CR_RNGEN) != 0);
	CHECK((RNG->CR & RNG_CR_CED) == 0);
	CHECK(rng->Random(nullptr, 0) == CRYPTO_STATUS_OK);
	CHECK(rng->Random(nullptr, 1) == CRYPTO_STATUS_FAIL);
	memset(data, 0xA5, sizeof(data));
	if (Mode == 0)
	{
		CHECK(rng->Random(data, 17) == CRYPTO_STATUS_OK);
		CHECK(data[17] == 0xA5 && data[18] == 0xA5);
		CHECK(memcmp(data, data + 4, 4) != 0);
		rng->Disable();
		CHECK(rng->Random(data, 17) == CRYPTO_STATUS_FAIL);
	}
	else
	{
		CHECK(rng->Random(data, 17) == CRYPTO_STATUS_FAIL);
		CHECK(rng->Valid() == false);
		CHECK((RNG->CR & RNG_CR_RNGEN) == 0);
	}
	for (int i = 0; i < 17; i++)
	{
		CHECK(data[i] == 0);
	}
	CHECK(data[17] == 0xA5 && data[18] == 0xA5);
	g_RngFault = 0;
	rng->Reset();
	CHECK(rng->Valid());
	CHECK(rng->Random(data, 17) == CRYPTO_STATUS_OK);
	rng->Disable();
	CHECK((RCC->CR & RCC_CR_HSI48ON) != 0);

	return 0;
}

extern "C" int RunCryptoVectors(void)
{
	alignas(CryptoSoftAes) uint8_t aesMem[sizeof(CryptoSoftAes)];
	alignas(CryptoSoftSha256) uint8_t hashMem[sizeof(CryptoSoftSha256)];
	alignas(CryptoUecc) uint8_t eccMem[sizeof(CryptoUecc)];
	CryptoSoftAes *aes = CryptoSoftAesCreate(aesMem, sizeof(aesMem));
	CryptoSoftSha256 *hash = CryptoSoftSha256Create(hashMem, sizeof(hashMem));
	CryptoUecc *ecc = CryptoUeccCreate(eccMem, sizeof(eccMem), nullptr);

	CHECK(aes != nullptr && aes->SelfTest() == 0);
	CHECK(hash != nullptr && hash->SelfTest() == 0);
	CHECK(ecc != nullptr && ecc->SelfTest() == 0);

	return 0;
}
