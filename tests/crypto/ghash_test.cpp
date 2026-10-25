// Exercise the private multiply without adding a public crypto API.
#include <assert.h>
#include <stdio.h>
#include <chrono>
#include "../../src/crypto/crypto_aes_modes.cpp"

// Original byte-serial implementation retained as an independent reference.
static void ReferenceMul(uint8_t Z[16], const uint8_t H[16])
{
	uint8_t v[16], z[16];
	memcpy(v, H, 16);
	memset(z, 0, 16);
	for (int i = 0; i < 128; i++)
	{
		uint8_t bit = (uint8_t)((Z[i >> 3] >> (7 - (i & 7))) & 1U);
		uint8_t mask = (uint8_t)(0U - bit);
		for (int j = 0; j < 16; j++)
		{
			z[j] ^= (uint8_t)(v[j] & mask);
		}
		uint8_t lsb = (uint8_t)(v[15] & 1U);
		for (int j = 15; j > 0; j--)
		{
			v[j] = (uint8_t)((v[j] >> 1) | (v[j - 1] << 7));
		}
		v[0] >>= 1;
		v[0] ^= (uint8_t)((0U - lsb) & 0xE1U);
	}
	memcpy(Z, z, 16);
	AesWipe(v, sizeof(v));
	AesWipe(z, sizeof(z));
}


static void Check(const uint8_t X[16], const uint8_t H[16])
{
	uint8_t actual[18], key[18], expected[16];

	memset(actual, 0xA5, sizeof(actual));
	memset(key, 0x5A, sizeof(key));
	memcpy(actual + 1, X, 16);
	memcpy(key + 1, H, 16);
	memcpy(expected, X, 16);
	ReferenceMul(expected, H);
	GhashMul(actual + 1, key + 1);
	assert(memcmp(actual + 1, expected, 16) == 0);
	assert(memcmp(key + 1, H, 16) == 0);
	assert(actual[0] == 0xA5 && actual[17] == 0xA5);
	assert(key[0] == 0x5A && key[17] == 0x5A);
}

#ifdef GHASH_BENCHMARK
static long long Measure(void (*Multiply)(uint8_t *, const uint8_t *))
{
	uint8_t x[16] = {1}, h[16];
	unsigned i;
	std::chrono::steady_clock::time_point start;
	long long elapsed;

	memset(h, 0xA5, sizeof(h));
	start = std::chrono::steady_clock::now();
	for (i = 0U; i < 200000U; i++)
	{
		Multiply(x, h);
	}
	elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now() - start).count();
	printf("checksum %02x, ", x[0]);
	return elapsed / 200000;
}
#endif

int main()
{
	uint8_t x[16] = {}, h[16] = {};
	uint32_t random = 0x73891ABCU;
	unsigned i, j, round;

	Check(x, h);
	memset(x, 0xFF, sizeof(x));
	Check(x, h);
	memset(h, 0xFF, sizeof(h));
	Check(x, h);
	memset(x, 0, sizeof(x));
	Check(x, h);
	for (i = 0U; i < 128U; i++)
	for (j = 0U; j < 128U; j++)
	{
		memset(x, 0, sizeof(x));
		memset(h, 0, sizeof(h));
		x[i / 8U] = (uint8_t)(0x80U >> (i % 8U));
		h[j / 8U] = (uint8_t)(0x80U >> (j % 8U));
		Check(x, h);
	}
	for (round = 0U; round < 4096U; round++)
	{
		for (i = 0U; i < 16U; i++)
		{
			random = random * 1664525U + 1013904223U;
			x[i] = (uint8_t)(random >> 24);
			random = random * 1664525U + 1013904223U;
			h[i] = (uint8_t)(random >> 24);
		}
		Check(x, h);
	}
	puts("GHASH: 16384 basis pairs, 4096 mixed inputs, zero/ones and unaligned bounds passed");
#ifdef GHASH_BENCHMARK
	printf("byte reference: %lld ns/multiply\n", Measure(ReferenceMul));
	printf("word implementation: %lld ns/multiply\n", Measure(GhashMul));
#endif
}
