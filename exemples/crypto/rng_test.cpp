/**-------------------------------------------------------------------------
@example	rng_test.cpp

@brief	Nordic RNG engine hardware acceptance test.

		Validates the CryptoRngNrf engine and the entropy interface it is
		constructed on: CracenIntrf on nRF54 parts (CTR-DRBG), RngPeriphIntrf
		on nRF52/nRF53 (RNG peripheral). Run the test twice to cover both
		run-time paths of the interface:

		1. Without the SoftDevice enabled: draws come from the entropy
		   hardware directly.
		2. From a BLE application after the stack is up (call RngTest from
		   the app after BtAppInit): draws come from the SoftDevice entropy
		   pool. On nRF54L with S145 this is the path where the stack owns
		   the entropy hardware.

		The checks distinguish a working generator from the common failure
		shapes: draw failure, all-zero output, a stuck generator (constant
		or repeating bytes), grossly non-uniform output, and identical
		consecutive draws. They are quick plausibility checks, not a NIST
		SP800-22 run; passing them on hardware plus a successful LESC
		pairing (which consumes this engine through SMP) is the acceptance
		bar.

		The test prints each step, the entropy interface in use and the
		run-time SoftDevice state (which together identify the actual entropy
		source) to stdout: semihosting console or wherever the project
		retargets printf. Result is also left in g_RngTestResult (0 pass,
		negative fail code) and g_RngTestPassMask (one bit per passed step)
		for debugger inspection. The last draw is kept in g_RngTestSample.

@author	Hoang Nguyen Hoan
@date	Jul. 16, 2026

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

#include "nrf.h"
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif

#include "crypto/icrypto.h"
#include "crypto_rng_nrf.h"

enum RNG_TEST_ERR {
	RNG_TEST_ERR_NONE            = 0,
	RNG_TEST_ERR_INSTANCE        = -1,	// engine singleton failed to construct
	RNG_TEST_ERR_NOT_SECURE      = -2,	// engine does not report security grade
	RNG_TEST_ERR_DRAW            = -3,	// Random() returned failure
	RNG_TEST_ERR_ALL_ZERO        = -4,	// draw produced all zero bytes
	RNG_TEST_ERR_STUCK           = -5,	// one byte value dominates the sample
	RNG_TEST_ERR_REPEAT_DRAW     = -6,	// two consecutive draws identical
	RNG_TEST_ERR_MONOBIT         = -7,	// set-bit ratio far from one half
	RNG_TEST_ERR_HISTOGRAM       = -8,	// byte histogram grossly non uniform
	RNG_TEST_ERR_ADJACENT        = -9,	// adjacent equal bytes far above chance
	RNG_TEST_ERR_SMALL_DRAW      = -10,	// 1 byte draw failed
	RNG_TEST_ERR_LARGE_DRAW      = -11,	// large draw failed
};

volatile int g_RngTestResult;
volatile uint32_t g_RngTestPassMask;
volatile uint8_t g_RngTestSample[32];

#define RNG_TEST_MARK(Bit) \
	do { g_RngTestPassMask |= (1UL << (Bit)); } while (0)

// Sample size for the statistical checks. 16 KB draws in well under a second
// on every supported part and gives the histogram enough mass for the loose
// bounds below. Overridable for constrained targets; the bounds scale with it.
#ifndef RNG_TEST_SAMPLE_LEN
#define RNG_TEST_SAMPLE_LEN		16384U
#endif

// Derived statistical bounds (integer, loose by design):
// - monobit: expected set bits Len*4, allowed deviation ~6 sigma where
//   sigma = sqrt(Len*2). Approximated as Len/16 + 128 which stays above
//   6 sigma for Len >= 1024 while remaining a trivial integer expression.
// - histogram: expected Len/256 per bucket; stuck above 4x, empty below 1/8.
// - adjacent equal bytes: chance Len/256, allowed 3x.
#define RNG_TEST_MONOBIT_DEV	(RNG_TEST_SAMPLE_LEN / 16U + 128U)
#define RNG_TEST_HIST_EXPECT	(RNG_TEST_SAMPLE_LEN / 256U)
#define RNG_TEST_HIST_MAX		(RNG_TEST_HIST_EXPECT * 4U)
#define RNG_TEST_HIST_MIN		(RNG_TEST_HIST_EXPECT / 8U)
#define RNG_TEST_ADJ_MAX		(RNG_TEST_HIST_EXPECT * 3U)

static uint8_t s_Sample[RNG_TEST_SAMPLE_LEN];

static bool RngTestFail(int Result)
{
	g_RngTestResult = Result;
	printf("FAIL (code %d)\r\n", Result);
	return false;
}

// Report which entropy source the draws will use. The compile target picks
// the interface; the run-time SoftDevice state picks the branch inside it.
// This is the same condition the interface tests on every draw.
static void RngTestReportSource(void)
{
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
	printf("Entropy interface : CracenIntrf\r\n");
	const char *hw = "CRACEN CTR-DRBG (hardware)";
#else
	printf("Entropy interface : RngPeriphIntrf\r\n");
	const char *hw = "RNG peripheral registers (hardware)";
#endif
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	uint8_t sden = 0;
	(void)sd_softdevice_is_enabled(&sden);
	if (sden)
	{
		printf("SoftDevice        : present and ENABLED\r\n");
		printf("Entropy source    : SoftDevice entropy pool (sd_rand)\r\n");
	}
	else
	{
		printf("SoftDevice        : present, not enabled\r\n");
		printf("Entropy source    : %s\r\n", hw);
	}
#else
	printf("SoftDevice        : not in build\r\n");
	printf("Entropy source    : %s\r\n", hw);
#endif
}

static bool AllZero(const uint8_t *pData, size_t Len)
{
	uint8_t acc = 0;
	for (size_t i = 0; i < Len; i++)
	{
		acc |= pData[i];
	}
	return acc == 0;
}

bool RngTest(void)
{
	g_RngTestResult = RNG_TEST_ERR_NONE;
	g_RngTestPassMask = 0;

	printf("\r\n--- Nordic RNG engine acceptance test ---\r\n");
	RngTestReportSource();
	printf("Sample size       : %u bytes\r\n\r\n", (unsigned)RNG_TEST_SAMPLE_LEN);

	// Step 0 : engine constructs and reports security grade.
	printf("[0] engine construct/Enable      : ");
	RngEngine *rng = CryptoRngNrfInstance();
	if (rng == nullptr)
	{
		return RngTestFail(RNG_TEST_ERR_INSTANCE);
	}
	printf("PASS\r\n");
	RNG_TEST_MARK(0);

	printf("[1] security grade (IsSecure)    : ");
	if (!rng->IsSecure())
	{
		return RngTestFail(RNG_TEST_ERR_NOT_SECURE);
	}
	printf("PASS\r\n");
	RNG_TEST_MARK(1);

	// Step 2 : boundary sizes. A single byte and a full block must both work;
	// they exercise the interface chunking edges.
	printf("[2] 1 byte draw                  : ");
	uint8_t one = 0;
	if (rng->Random(&one, 1) != CRYPTO_STATUS_OK)
	{
		return RngTestFail(RNG_TEST_ERR_SMALL_DRAW);
	}
	printf("PASS (0x%02X)\r\n", one);
	RNG_TEST_MARK(2);

	printf("[3] two 32 byte draws            : ");
	uint8_t blk1[32];
	uint8_t blk2[32];
	if (rng->Random(blk1, sizeof(blk1)) != CRYPTO_STATUS_OK ||
		rng->Random(blk2, sizeof(blk2)) != CRYPTO_STATUS_OK)
	{
		return RngTestFail(RNG_TEST_ERR_DRAW);
	}
	memcpy((void *)g_RngTestSample, blk2, sizeof(blk2));
	printf("PASS\r\n    draw : ");
	for (int i = 0; i < 16; i++)
	{
		printf("%02X", blk2[i]);
	}
	printf("...\r\n");
	RNG_TEST_MARK(3);

	// Step 4 : draws are not degenerate.
	printf("[4] non-degenerate (not zero, draws differ) : ");
	if (AllZero(blk1, sizeof(blk1)) || AllZero(blk2, sizeof(blk2)))
	{
		return RngTestFail(RNG_TEST_ERR_ALL_ZERO);
	}
	if (memcmp(blk1, blk2, sizeof(blk1)) == 0)
	{
		return RngTestFail(RNG_TEST_ERR_REPEAT_DRAW);
	}
	printf("PASS\r\n");
	RNG_TEST_MARK(4);

	// Step 5 : large draw for the statistical checks.
	printf("[5] %u byte draw               : ", (unsigned)RNG_TEST_SAMPLE_LEN);
	if (rng->Random(s_Sample, RNG_TEST_SAMPLE_LEN) != CRYPTO_STATUS_OK)
	{
		return RngTestFail(RNG_TEST_ERR_LARGE_DRAW);
	}
	printf("PASS\r\n");
	RNG_TEST_MARK(5);

	// Step 6 : monobit. Expected set bits = Len * 4; allow the derived
	// deviation bound so a healthy generator never trips while a biased
	// one does.
	uint32_t bits = 0;
	for (size_t i = 0; i < RNG_TEST_SAMPLE_LEN; i++)
	{
		bits += (uint32_t)__builtin_popcount(s_Sample[i]);
	}
	{
		const int32_t expect = (int32_t)(RNG_TEST_SAMPLE_LEN * 4U);
		int32_t dev = (int32_t)bits - expect;
		if (dev < 0)
		{
			dev = -dev;
		}
		printf("[6] monobit                      : set bits %u, expected %d, deviation %d (limit %u) : ",
			   (unsigned)bits, (int)expect, (int)dev, (unsigned)RNG_TEST_MONOBIT_DEV);
		if (dev > (int32_t)RNG_TEST_MONOBIT_DEV)
		{
			return RngTestFail(RNG_TEST_ERR_MONOBIT);
		}
		printf("PASS\r\n");
	}
	RNG_TEST_MARK(6);

	// Step 7 : byte histogram. The bounds are deliberately loose: they pass
	// any plausible generator and fail a stuck or strongly patterned one.
	{
		uint16_t hist[256];
		memset(hist, 0, sizeof(hist));
		for (size_t i = 0; i < RNG_TEST_SAMPLE_LEN; i++)
		{
			hist[s_Sample[i]]++;
		}
		uint16_t hmin = 0xFFFF;
		uint16_t hmax = 0;
		int low = 0;
		for (int i = 0; i < 256; i++)
		{
			if (hist[i] < hmin) hmin = hist[i];
			if (hist[i] > hmax) hmax = hist[i];
			if (hist[i] < RNG_TEST_HIST_MIN) low++;
		}
		printf("[7] byte histogram               : min %u, max %u, expected %u each (bounds %u..%u) : ",
			   (unsigned)hmin, (unsigned)hmax, (unsigned)RNG_TEST_HIST_EXPECT,
			   (unsigned)RNG_TEST_HIST_MIN, (unsigned)RNG_TEST_HIST_MAX);
		if (hmax > RNG_TEST_HIST_MAX)
		{
			return RngTestFail(RNG_TEST_ERR_STUCK);
		}
		// A uniform source leaves no bucket under the minimum in practice;
		// allow a few as margin.
		if (low > 4)
		{
			return RngTestFail(RNG_TEST_ERR_HISTOGRAM);
		}
		printf("PASS\r\n");
	}
	RNG_TEST_MARK(7);

	// Step 8 : adjacent equal bytes. Chance rate is Len / 256; allow 3x. A
	// repeating or stuttering generator lands far above this.
	{
		uint32_t eq = 0;
		for (size_t i = 1; i < RNG_TEST_SAMPLE_LEN; i++)
		{
			if (s_Sample[i] == s_Sample[i - 1])
			{
				eq++;
			}
		}
		printf("[8] adjacent equal bytes         : %u, chance rate %u (limit %u) : ",
			   (unsigned)eq, (unsigned)RNG_TEST_HIST_EXPECT, (unsigned)RNG_TEST_ADJ_MAX);
		if (eq > RNG_TEST_ADJ_MAX)
		{
			return RngTestFail(RNG_TEST_ERR_ADJACENT);
		}
		printf("PASS\r\n");
	}
	RNG_TEST_MARK(8);

	printf("\r\nAll steps passed. Result 0, pass mask 0x%03X\r\n",
		   (unsigned)g_RngTestPassMask);
	return true;
}

#ifndef RNG_TEST_NO_MAIN
int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;
	(void)RngTest();
	while (true)
	{
		__WFE();
	}
}
#endif
