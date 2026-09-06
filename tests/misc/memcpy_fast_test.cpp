/**-------------------------------------------------------------------------
@file	memcpy_fast_test.cpp

@brief	Host regression tests for memcpy_fast.

Every case compares against memcpy over a buffer with guard bytes on both
sides, so a copy that writes outside its range is caught as well as one that
writes the wrong bytes. Both entry points are exercised: the inline
dispatcher in the header and the out of line bulk routine.

@author	Hoang Nguyen Hoan
@date	Sep. 5, 2026

@license

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

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "memcpy_fast.h"

#define GUARD			16U
#define MAXLEN			512U
#define ALIGNS			8U
#define ARENA			(MAXLEN + ALIGNS + 2U * GUARD)

alignas(16) static uint8_t s_Src[ARENA];
alignas(16) static uint8_t s_Dst[ARENA];
alignas(16) static uint8_t s_Ref[ARENA];

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

/// One copy through memcpy_fast and the same copy through memcpy, compared
/// over the whole arena so guard bytes are included.
static bool CopyMatches(size_t Len, unsigned DstAlign, unsigned SrcAlign,
						bool bInline)
{
	for (size_t i = 0; i < ARENA; i++)
	{
		s_Src[i] = (uint8_t)(rand() & 0xFF);
	}
	memset(s_Dst, 0xA5, ARENA);
	memcpy(s_Ref, s_Dst, ARENA);

	uint8_t *pD = s_Dst + GUARD + DstAlign;
	uint8_t *pR = s_Ref + GUARD + DstAlign;
	const uint8_t *pS = s_Src + GUARD + SrcAlign;

	void *pRet = bInline ? memcpy_fast(pD, pS, Len) :
						   memcpy_fast_bulk(pD, pS, Len);
	memcpy(pR, pS, Len);

	return pRet == pD && memcmp(s_Dst, s_Ref, ARENA) == 0;
}

// The five sizes the header dispatches without a call, at the alignment they
// are meant for. These are the path ported from TaktOS.
static void TestFixedSizes(void)
{
	static const size_t sizes[] = { 4U, 8U, 12U, 16U, 32U };

	for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++)
	{
		CHECK(CopyMatches(sizes[i], 0U, 0U, true));
		CHECK(CopyMatches(sizes[i], 0U, 0U, false));
	}
}

// The same sizes with one pointer off a word boundary must not take the word
// wide path, and must still produce the right bytes.
static void TestFixedSizesUnaligned(void)
{
	static const size_t sizes[] = { 4U, 8U, 12U, 16U, 32U };

	for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++)
	{
		for (unsigned a = 1U; a < 4U; a++)
		{
			CHECK(CopyMatches(sizes[i], a, 0U, true));
			CHECK(CopyMatches(sizes[i], 0U, a, true));
			CHECK(CopyMatches(sizes[i], a, a, true));
		}
	}
}

// Every length from zero to MAXLEN at every combination of source and
// destination offset. This is the case that finds head and tail errors.
static void TestAllLengthsAllAlignments(void)
{
	for (size_t len = 0U; len <= MAXLEN; len++)
	{
		for (unsigned da = 0U; da < ALIGNS; da++)
		{
			for (unsigned sa = 0U; sa < ALIGNS; sa++)
			{
				if (!CopyMatches(len, da, sa, true))
				{
					printf("FAIL inline len=%zu da=%u sa=%u\n", len, da, sa);
					s_Fail++;
					return;
				}
				if (!CopyMatches(len, da, sa, false))
				{
					printf("FAIL bulk len=%zu da=%u sa=%u\n", len, da, sa);
					s_Fail++;
					return;
				}
			}
		}
	}
}

// Lengths that straddle the four word body, where the loop hands over to the
// single word loop and then to the tail.
static void TestBodyBoundaries(void)
{
	static const size_t sizes[] = {
		13U, 15U, 16U, 17U, 31U, 32U, 33U, 47U, 48U, 49U, 63U, 64U, 65U
	};

	for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++)
	{
		for (unsigned a = 0U; a < 4U; a++)
		{
			CHECK(CopyMatches(sizes[i], a, a, false));
		}
	}
}

static void TestZeroAndNull(void)
{
	uint8_t buf[4] = { 1U, 2U, 3U, 4U };

	// Zero size returns the destination and touches nothing.
	CHECK(memcpy_fast(buf, buf, 0U) == buf);
	CHECK(memcpy_fast_bulk(buf, buf, 0U) == buf);
	CHECK(buf[0] == 1U && buf[3] == 4U);

	// A null on either side returns without writing.
	CHECK(memcpy_fast_bulk(nullptr, buf, 4U) == nullptr);
	CHECK(memcpy_fast_bulk(buf, nullptr, 4U) == buf);
	CHECK(buf[0] == 1U && buf[3] == 4U);
}

// A copy where destination and source overlap is undefined for memcpy and for
// this routine too. What is checked here is only that a copy onto itself is
// harmless, since callers do hit that case.
static void TestSelfCopy(void)
{
	alignas(4) uint8_t buf[64];

	for (unsigned i = 0; i < sizeof(buf); i++)
	{
		buf[i] = (uint8_t)i;
	}

	CHECK(memcpy_fast(buf, buf, sizeof(buf)) == buf);

	for (unsigned i = 0; i < sizeof(buf); i++)
	{
		CHECK(buf[i] == (uint8_t)i);
	}
}

struct Case { const char *Name; void (*Fn)(void); };

int main(void)
{
	srand(20260905U);

	static const Case cases[] = {
		{ "fixed sizes", TestFixedSizes },
		{ "fixed sizes unaligned", TestFixedSizesUnaligned },
		{ "body boundaries", TestBodyBoundaries },
		{ "zero and null", TestZeroAndNull },
		{ "self copy", TestSelfCopy },
		{ "all lengths all alignments", TestAllLengthsAllAlignments },
	};

	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
	{
		const int before = s_Fail;
		cases[i].Fn();
		printf("%-28s %s\n", cases[i].Name,
			   s_Fail == before ? "pass" : "FAIL");
	}

	printf("%s\n", s_Fail == 0 ? "all pass" : "FAILURES");
	return s_Fail == 0 ? 0 : 1;
}
