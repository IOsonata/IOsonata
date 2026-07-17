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

		Result is left in g_RngTestResult (0 pass, negative fail code) and
		g_RngTestPassMask (one bit per passed step) for debugger inspection.
		The last draw is kept in g_RngTestSample.

@author	Hoang Nguyen Hoan
@date	Jul. 17, 2026

@license MIT, (c) 2026 I-SYST. See crypto.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nrf.h"

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
	return false;
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

	// Step 0 : engine constructs and reports security grade.
	RngEngine *rng = CryptoRngNrfInstance();
	if (rng == nullptr)
	{
		return RngTestFail(RNG_TEST_ERR_INSTANCE);
	}
	RNG_TEST_MARK(0);

	if (!rng->IsSecure())
	{
		return RngTestFail(RNG_TEST_ERR_NOT_SECURE);
	}
	RNG_TEST_MARK(1);

	// Step 2 : boundary sizes. A single byte and a full block must both work;
	// they exercise the interface chunking edges.
	uint8_t one = 0;
	if (rng->Random(&one, 1) != CRYPTO_STATUS_OK)
	{
		return RngTestFail(RNG_TEST_ERR_SMALL_DRAW);
	}
	RNG_TEST_MARK(2);

	uint8_t blk1[32];
	uint8_t blk2[32];
	if (rng->Random(blk1, sizeof(blk1)) != CRYPTO_STATUS_OK ||
		rng->Random(blk2, sizeof(blk2)) != CRYPTO_STATUS_OK)
	{
		return RngTestFail(RNG_TEST_ERR_DRAW);
	}
	memcpy((void *)g_RngTestSample, blk2, sizeof(blk2));
	RNG_TEST_MARK(3);

	// Step 4 : draws are not degenerate.
	if (AllZero(blk1, sizeof(blk1)) || AllZero(blk2, sizeof(blk2)))
	{
		return RngTestFail(RNG_TEST_ERR_ALL_ZERO);
	}
	if (memcmp(blk1, blk2, sizeof(blk1)) == 0)
	{
		return RngTestFail(RNG_TEST_ERR_REPEAT_DRAW);
	}
	RNG_TEST_MARK(4);

	// Step 5 : large draw for the statistical checks.
	if (rng->Random(s_Sample, RNG_TEST_SAMPLE_LEN) != CRYPTO_STATUS_OK)
	{
		return RngTestFail(RNG_TEST_ERR_LARGE_DRAW);
	}
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
		if (dev > (int32_t)RNG_TEST_MONOBIT_DEV)
		{
			return RngTestFail(RNG_TEST_ERR_MONOBIT);
		}
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
		for (int i = 0; i < 256; i++)
		{
			if (hist[i] > RNG_TEST_HIST_MAX)
			{
				return RngTestFail(RNG_TEST_ERR_STUCK);
			}
		}
		int empty = 0;
		for (int i = 0; i < 256; i++)
		{
			if (hist[i] < RNG_TEST_HIST_MIN)
			{
				empty++;
			}
		}
		// A uniform source leaves no bucket under the minimum in practice;
		// allow a few as margin.
		if (empty > 4)
		{
			return RngTestFail(RNG_TEST_ERR_HISTOGRAM);
		}
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
		if (eq > RNG_TEST_ADJ_MAX)
		{
			return RngTestFail(RNG_TEST_ERR_ADJACENT);
		}
	}
	RNG_TEST_MARK(8);

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
