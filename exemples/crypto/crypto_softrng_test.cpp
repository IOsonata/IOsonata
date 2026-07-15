/**-------------------------------------------------------------------------
@file	crypto_softrng_test.cpp

@brief	Host validation harness for the OO CryptoSoftRng engine.

		Exercises CryptoSoftRng through the RngEngine facet. Includes only
		crypto_softrng.h. A PRNG is deterministic and not security grade, so the
		tests check the facet, the IsSecure() rule, reproducibility, and
		basic statistical sanity, not cryptographic quality.

		Tests:
		  1. Factory constructs; usable as an RngEngine pointer.
		  2. IsSecure() is false (the property that makes key generation decline
		     this engine).
		  3. Same seed gives the same sequence (deterministic).
		  4. Different seed gives a different sequence.
		  5. Fills exactly Len bytes for a non-block-aligned length.
		  6. Basic distribution sanity: over many bytes the mean is near 127 and
		     all 256 byte values appear.

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softrng.h"

static int s_pass = 0, s_fail = 0;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

int main(void)
{
	printf("CryptoSoftRng OO engine validation\n");

	static uint8_t mem[128];
	CryptoSoftRng *e = CryptoSoftRngCreate(mem, sizeof(mem));
	check("factory constructs engine", e != nullptr);
	if (e == nullptr) { printf("abort\n"); return 1; }

	RngEngine *r = e;
	check("usable as RngEngine pointer", r != nullptr);

	// Test 2: not security grade. This is the guard key generation relies on.
	check("IsSecure() is false (PRNG, not for key generation)", r->IsSecure() == false);

	// Test 3: same seed, same sequence.
	uint8_t a[64], b[64];
	e->Seed(0x1111222233334444ULL, 0x5555666677778888ULL);
	r->Random(a, sizeof(a));
	e->Seed(0x1111222233334444ULL, 0x5555666677778888ULL);
	r->Random(b, sizeof(b));
	check("same seed reproduces the same sequence", memcmp(a, b, sizeof(a)) == 0);

	// Test 4: different seed, different sequence.
	e->Seed(0x1111222233334444ULL, 0x5555666677778888ULL);
	r->Random(a, sizeof(a));
	e->Seed(0xAAAABBBBCCCCDDDDULL, 0x1234567890ABCDEFULL);
	r->Random(b, sizeof(b));
	check("different seed gives a different sequence", memcmp(a, b, sizeof(a)) != 0);

	// Test 5: exact fill for a non-8-aligned length.
	uint8_t buf[19];
	memset(buf, 0xCC, sizeof(buf));
	uint8_t guard = 0xCC;
	r->Random(buf, 17);
	// Byte 17 and 18 must be untouched (guard pattern), 0..16 likely changed.
	check("fills exactly Len bytes (no overrun)",
		  buf[17] == guard && buf[18] == guard);

	// Test 6: basic distribution sanity over a large sample.
	{
		e->Seed(0xC0FFEE1234567890ULL, 0x0BADF00DDEADBEEFULL);
		uint32_t seen[256];
		memset(seen, 0, sizeof(seen));
		uint64_t sum = 0;
		const int N = 65536;
		uint8_t chunk[256];
		int produced = 0;
		while (produced < N)
		{
			r->Random(chunk, sizeof(chunk));
			for (size_t i = 0; i < sizeof(chunk) && produced < N; i++, produced++)
			{
				seen[chunk[i]]++;
				sum += chunk[i];
			}
		}
		// Mean as fixed-point tenths (mean * 10) using integer math only, so it
		// prints on an embedded printf built without floating-point support.
		uint32_t meanT10 = (uint32_t)((sum * 10U) / N);	// e.g. 1278 == 127.8
		int missing = 0;
		for (int v = 0; v < 256; v++) { if (seen[v] == 0) missing++; }
		check("distribution sanity (mean near 127, all 256 values seen)",
			  meanT10 > 1200U && meanT10 < 1350U && missing == 0);
		printf("       (mean %u.%u, byte values unseen: %d)\n",
			   (unsigned)(meanT10 / 10U), (unsigned)(meanT10 % 10U), missing);
	}

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
