/**-------------------------------------------------------------------------
@file	bench.cpp

@brief	Timing harness for memcpy_fast and CFifo.

Not a regression test. It prints a table so two builds can be compared.

Every measurement is repeated and the median is reported, with the spread
between the fastest and slowest repeat alongside it. A single timing run on
a host with other work on it is not repeatable, so a difference smaller than
the reported spread means nothing. Compare medians, and only believe a change
that is larger than the spread of both runs.

Usage:

	./bench [all|memcpy|cfifo] [repeats]

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
#include <algorithm>
#include <chrono>
#include <vector>

#include "cfifo.h"
#include "memcpy_fast.h"

#define ARENA			4096U
#define BLK				32U
#define SLOTS			64U
#define DEFAULT_REPEATS	7

alignas(16) static uint8_t s_Src[ARENA];
alignas(16) static uint8_t s_Dst[ARENA];
alignas(4) static uint8_t s_FifoMem[CFIFO_TOTAL_MEMSIZE(SLOTS, BLK)];
alignas(4) static uint8_t s_ByteFifoMem[CFIFO_MEMSIZE(4096)];

// Kept live so the optimizer cannot delete the work being timed.
static volatile uint64_t s_Sink;

static int s_Repeats = DEFAULT_REPEATS;

#define BARRIER()		__asm__ __volatile__("" ::: "memory")

struct Result { double Median; double Spread; };

/// Run Fn Repeats times, each doing Iters operations, and report the median
/// nanoseconds per operation plus the fastest to slowest span as a percent.
template <typename F>
static Result Measure(unsigned Iters, F Fn)
{
	std::vector<double> runs;

	// One untimed pass so caches and branch state are warm.
	Fn();

	for (int r = 0; r < s_Repeats; r++)
	{
		BARRIER();
		const auto t0 = std::chrono::steady_clock::now();
		Fn();
		const auto t1 = std::chrono::steady_clock::now();
		BARRIER();

		const double ns =
			std::chrono::duration<double, std::nano>(t1 - t0).count();
		runs.push_back(ns / (double)Iters);
	}

	std::sort(runs.begin(), runs.end());

	Result res;
	res.Median = runs[runs.size() / 2];
	res.Spread = runs.front() > 0.0 ?
		(runs.back() - runs.front()) * 100.0 / runs.front() : 0.0;

	return res;
}

static void BenchCopy(void)
{
	static const size_t sizes[] = { 4U, 8U, 12U, 16U, 32U, 64U, 128U, 512U };
	static const unsigned aligns[][2] = { { 0U, 0U }, { 1U, 1U }, { 0U, 1U } };
	static const char *alignName[] = { "word", "same odd", "mixed" };

	printf("\nmemcpy_fast against the standard library\n");
	printf("median ns per call over %d repeats, and what that is in MB per "
		   "second\n", s_Repeats);
	printf("%6s %-9s %9s %9s %9s %9s %9s %8s\n",
		   "size", "align", "memcpy", "fast", "bulk",
		   "memcpy", "fast", "fast/std");
	printf("%6s %-9s %9s %9s %9s %9s %9s %8s\n",
		   "", "", "ns", "ns", "ns", "MB/s", "MB/s", "");

	for (unsigned a = 0; a < 3U; a++)
	{
		for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++)
		{
			const size_t len = sizes[i];
			uint8_t *pD = s_Dst + aligns[a][0];
			const uint8_t *pS = s_Src + aligns[a][1];
			const unsigned iters = 200000U;

			Result std = Measure(iters, [&]() {
				for (unsigned n = 0; n < iters; n++)
				{
					memcpy(pD, pS, len);
					BARRIER();
				}
				s_Sink += pD[0];
			});

			Result fast = Measure(iters, [&]() {
				for (unsigned n = 0; n < iters; n++)
				{
					memcpy_fast(pD, pS, len);
					BARRIER();
				}
				s_Sink += pD[0];
			});

			Result bulk = Measure(iters, [&]() {
				for (unsigned n = 0; n < iters; n++)
				{
					memcpy_fast_bulk(pD, pS, len);
					BARRIER();
				}
				s_Sink += pD[0];
			});

			// Nanoseconds per call turned into bytes per second, so the
			// cost can be read against a transfer rate.
			const double stdRate = std.Median > 0.0 ?
				(double)len * 1000.0 / std.Median : 0.0;
			const double fastRate = fast.Median > 0.0 ?
				(double)len * 1000.0 / fast.Median : 0.0;
			const double ratio =
				std.Median > 0.0 ? fast.Median / std.Median : 0.0;

			printf("%6zu %-9s %9.2f %9.2f %9.2f %9.0f %9.0f %7.2fx%s\n",
				   len, alignName[a],
				   std.Median, fast.Median, bulk.Median,
				   stdRate, fastRate, ratio,
				   ratio < 1.0 ? " +" : "");
		}
	}
}

static void BenchCFifo(void)
{
	printf("\nCFifo, median ns per operation over %d repeats\n", s_Repeats);
	printf("%-28s %10s %9s\n", "operation", "ns/op", "spread");

	hCFifo_t h = CFifoInit(s_FifoMem, sizeof(s_FifoMem), BLK, true);
	if (h == nullptr)
	{
		printf("CFifoInit failed\n");
		return;
	}

	const unsigned iters = 200000U;

	Result pg = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			uint8_t *p = CFifoPut(h);
			if (p != nullptr) { p[0] = (uint8_t)n; }
			const uint8_t *g = CFifoGet(h);
			if (g != nullptr) { s_Sink += g[0]; }
		}
	});
	printf("%-28s %7.2f %8.0f%%\n", "put + get, one block",
		   pg.Median, pg.Spread);

	Result pgm = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			int cnt = 8;
			uint8_t *p = CFifoPutMultiple(h, &cnt);
			if (p != nullptr) { p[0] = (uint8_t)n; }
			cnt = 8;
			const uint8_t *g = CFifoGetMultiple(h, &cnt);
			if (g != nullptr) { s_Sink += g[0]; }
		}
	});
	printf("%-28s %7.2f %8.0f%%\n", "put + get multiple, 8",
		   pgm.Median, pgm.Spread);

	Result pk = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			uint8_t *p = CFifoPut(h);
			if (p != nullptr) { p[0] = (uint8_t)n; }
			const uint8_t *q = CFifoPeek(h);
			if (q != nullptr) { s_Sink += q[0]; }
			(void)CFifoGet(h);
		}
	});
	printf("%-28s %7.2f %8.0f%%\n", "put + peek + get", pk.Median, pk.Spread);

	Result ua = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			s_Sink += (uint64_t)CFifoUsed(h);
			s_Sink += (uint64_t)CFifoAvail(h);
		}
	});
	printf("%-28s %7.2f %8.0f%%\n", "used + avail", ua.Median, ua.Spread);

	hCFifo_t b = CFifoInit(s_ByteFifoMem, sizeof(s_ByteFifoMem), 1U, true);
	if (b == nullptr)
	{
		printf("byte CFifoInit failed\n");
		return;
	}

	uint8_t buf[64];
	memset(buf, 0x5AU, sizeof(buf));

	const unsigned rwIters = 50000U;
	Result rw = Measure(rwIters, [&]() {
		for (unsigned n = 0; n < rwIters; n++)
		{
			(void)CFifoWrite(b, buf, (int)sizeof(buf));
			(void)CFifoRead(b, buf, (int)sizeof(buf));
			s_Sink += buf[0];
		}
	});
	printf("%-28s %7.2f %8.0f%%\n", "write + read, 64 bytes",
		   rw.Median, rw.Spread);
}

int main(int argc, char **argv)
{
	bool bCopy = true;
	bool bFifo = true;
	int argRepeats = 1;

	if (argc > 1 && atoi(argv[1]) == 0)
	{
		if (strcmp(argv[1], "memcpy") == 0) { bFifo = false; }
		else if (strcmp(argv[1], "cfifo") == 0) { bCopy = false; }
		argRepeats = 2;
	}

	if (argc > argRepeats)
	{
		s_Repeats = atoi(argv[argRepeats]);
		if (s_Repeats < 1) { s_Repeats = DEFAULT_REPEATS; }
	}

	for (unsigned i = 0; i < ARENA; i++)
	{
		s_Src[i] = (uint8_t)i;
	}

	printf("IOsonata misc benchmark\n");
	printf("Spread is fastest to slowest across repeats. A difference "
		   "smaller than\nthe spread is not a result.\n");

	if (bCopy)
	{
		BenchCopy();
		printf("\nA plus marks where memcpy_fast beats the standard library.\n"
			   "Above the fixed sizes the host library uses vector moves that\n"
			   "a Cortex-M library does not have, so this table says what the\n"
			   "host does, not what the target will do.\n");
	}

	if (bFifo)
	{
		BenchCFifo();
	}

	printf("\nchecksum %llu\n", (unsigned long long)s_Sink);

	return 0;
}
