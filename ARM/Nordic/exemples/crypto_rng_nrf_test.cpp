/**-------------------------------------------------------------------------
@file	crypto_rng_nrf_test.cpp

@brief	On-target validation harness for the Nordic hardware RNG engine.

		Runs on an nRF target with a hardware RNG. A true random source has no
		known answer, so this does not check fixed vectors; it checks the facet,
		the IsSecure() claim, and statistical health of the byte stream, plus a
		non-repetition check that would catch a dead or stuck generator.

		Same shape as the other crypto examples: int main and printf, retargeted
		to the board console by the target build.

		Tests:
		  1. Singleton constructs and enables (hardware RNG brought up).
		  2. Usable as an RngEngine pointer and reports IsSecure() true.
		  3. Two large draws differ (generator is live, not stuck or dead).
		  4. Byte distribution health: mean near 127.5 and every one of the 256
		     byte values appears over a large sample.
		  5. Bit balance health: the count of set bits is near half of all bits.
		  6. Fills exactly Len for a non-block-aligned request (no overrun).

		These are sanity and liveness checks, not a certification of entropy
		quality; the underlying CTR-DRBG or peripheral is the entropy authority.

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crypto_rng_nrf.h"

static int s_pass = 0, s_fail = 0;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

int main(void)
{
	printf("CryptoRngNrf hardware RNG validation\n");

	CryptoRngNrf *e = CryptoRngNrfInstance();
	bool up = (e != nullptr) && e->Enable();
	check("singleton constructs and enables hardware RNG", up);
	if (!up) { printf("abort: hardware RNG not available\n"); return 1; }

	RngEngine *r = e;
	check("usable as RngEngine, reports IsSecure() true",
		  r != nullptr && r->IsSecure() == true);

	// Test 3: liveness. Two independent draws must differ; an all-equal result
	// means a stuck or dead generator.
	uint8_t a[64], b[64];
	bool da = (r->Random(a, sizeof(a)) == CRYPTO_STATUS_OK);
	bool db = (r->Random(b, sizeof(b)) == CRYPTO_STATUS_OK);
	check("two large draws succeed and differ (generator live)",
		  da && db && memcmp(a, b, sizeof(a)) != 0);

	// Test 4 + 5: statistical health over a large sample. Mean near 127.5, all
	// 256 byte values seen, and set-bit count near half of all bits.
	{
		uint32_t seen[256];
		memset(seen, 0, sizeof(seen));
		uint64_t byteSum = 0;
		uint64_t bitSet = 0;
		const int N = 65536;
		uint8_t chunk[256];
		bool drawOk = true;
		int produced = 0;
		while (produced < N && drawOk)
		{
			drawOk = (r->Random(chunk, sizeof(chunk)) == CRYPTO_STATUS_OK);
			for (size_t i = 0; i < sizeof(chunk) && produced < N; i++, produced++)
			{
				uint8_t v = chunk[i];
				seen[v]++;
				byteSum += v;
				uint8_t x = v;
				while (x) { bitSet += (x & 1U); x >>= 1; }
			}
		}
		// Mean as fixed-point tenths using integer math (no float printf).
		uint32_t meanT10 = (uint32_t)((byteSum * 10U) / N);
		int missing = 0;
		for (int v = 0; v < 256; v++) { if (seen[v] == 0) missing++; }

		check("byte distribution health (mean ~127.5, all 256 values seen)",
			  drawOk && meanT10 > 1200U && meanT10 < 1350U && missing == 0);
		printf("       (mean %u.%u, byte values unseen: %d)\n",
			   (unsigned)(meanT10 / 10U), (unsigned)(meanT10 % 10U), missing);

		// Set bits should be near 50 percent of all bits (N * 8). Report as
		// per-mille so it prints without float. Accept 480..520 per-mille.
		uint32_t setPerMille = (uint32_t)((bitSet * 1000U) / ((uint64_t)N * 8U));
		check("bit balance health (set bits near 50 percent)",
			  setPerMille > 480U && setPerMille < 520U);
		printf("       (set bits: %u per-mille of total)\n", (unsigned)setPerMille);
	}

	// Test 6: exact fill for a non-8-aligned length.
	uint8_t buf[19];
	memset(buf, 0x5A, sizeof(buf));
	uint8_t guard = 0x5A;
	bool okFill = (r->Random(buf, 17) == CRYPTO_STATUS_OK);
	check("fills exactly Len bytes (no overrun)",
		  okFill && buf[17] == guard && buf[18] == guard);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
