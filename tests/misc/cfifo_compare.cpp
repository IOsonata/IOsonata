/**-------------------------------------------------------------------------
@file	cfifo_compare.cpp

@brief	Same CFifo harness built against two versions of the implementation.

Compiled twice. Without CFIFO_LEGACY it links the current src/cfifo.c. With
CFIFO_LEGACY it links the 2021 implementation, which the Makefile extracts
from git rather than keeping a copy in the tree.

The 2021 version has no CFifoPeek and no CFifoIsBlocking, and its
CFifoRead/CFifoWrite were never finished, so those cases report skip there
instead of failing. Everything else runs on both.

Correctness comes first, then timing. Timing repeats each measurement and
reports the median with the spread between the fastest and slowest repeat.
A difference smaller than the spread is not a result.

Usage:

	./cfifo_compare [repeats]

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
#include <sys/wait.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <deque>
#include <vector>

#include "cfifo.h"

#ifdef CFIFO_LEGACY
#define VERSION_NAME			"2021 (4c69ab6c)"
typedef HCFIFO					hCFifo_t;
#define HAS_PEEK				0
#define HAS_ISBLOCKING			0
#define HAS_READWRITE			0
#else
#define VERSION_NAME			"current"
#define HAS_PEEK				1
#define HAS_ISBLOCKING			1
#define HAS_READWRITE			1
#endif

#define BLK						8U
#define POW2_SLOTS				8U
#define ODD_SLOTS				5U
#define ONE_SLOT				1U
#define DEFAULT_REPEATS			7

alignas(4) static uint8_t s_Pow2Mem[CFIFO_TOTAL_MEMSIZE(POW2_SLOTS, BLK)];
alignas(4) static uint8_t s_OddMem[CFIFO_TOTAL_MEMSIZE(ODD_SLOTS, BLK)];
alignas(4) static uint8_t s_OneMem[CFIFO_TOTAL_MEMSIZE(ONE_SLOT, BLK)];
#if HAS_READWRITE
alignas(4) static uint8_t s_ByteMem[CFIFO_MEMSIZE(256)];
#endif

static int s_Fail;
static int s_Skip;
static volatile uint64_t s_Sink;
static int s_Repeats = DEFAULT_REPEATS;

#define CHECK(c) do { if (!(c)) { \
	printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

#define BARRIER()				__asm__ __volatile__("" ::: "memory")

static void FillBlock(uint8_t *pBlk, uint8_t Seed)
{
	for (unsigned i = 0; i < BLK; i++)
	{
		pBlk[i] = (uint8_t)(Seed + i);
	}
}

static bool BlockIs(const uint8_t *pBlk, uint8_t Seed)
{
	for (unsigned i = 0; i < BLK; i++)
	{
		if (pBlk[i] != (uint8_t)(Seed + i))
		{
			return false;
		}
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////////
// Correctness
//////////////////////////////////////////////////////////////////////////////

static void TestInitValidation(void)
{
	alignas(4) static uint8_t tiny[sizeof(CFifo_t) + 4U];

	CHECK(CFifoInit(nullptr, sizeof(s_Pow2Mem), BLK, true) == nullptr);
	CHECK(CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), 0U, true) == nullptr);

	// Header fits but not one whole block. A handle here leaves a FIFO with
	// zero slots, which every later call has to cope with.
	CHECK(CFifoInit(tiny, sizeof(CFifo_t), BLK, true) == nullptr);
	CHECK(CFifoInit(tiny, sizeof(CFifo_t) + BLK - 1U, BLK, true) == nullptr);

	hCFifo_t h = CFifoInit(tiny, sizeof(CFifo_t) + 4U, 4U, true);
	CHECK(h != nullptr);
	CHECK(h != nullptr && CFifoAvail(h) == 1);
	CHECK(h != nullptr && CFifoUsed(h) == 0);
	CHECK(h != nullptr && CFifoBlockSize(h) == 4U);
}

static void TestEmptyState(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	CHECK(CFifoUsed(h) == 0);
	CHECK(CFifoAvail(h) == (int)POW2_SLOTS);
	CHECK(CFifoGet(h) == nullptr);

	int cnt = 4;
	CHECK(CFifoGetMultiple(h, &cnt) == nullptr);
	CHECK(cnt == 0);
#if HAS_PEEK
	CHECK(CFifoPeek(h) == nullptr);
#endif
#if HAS_ISBLOCKING
	CHECK(CFifoIsBlocking(h) == true);
#endif
}

static void TestSingleRoundTrip(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	uint8_t *p = CFifoPut(h);
	CHECK(p != nullptr);
	if (p == nullptr) { return; }
	FillBlock(p, 0x10U);

	CHECK(CFifoUsed(h) == 1);
	CHECK(CFifoAvail(h) == (int)POW2_SLOTS - 1);

#if HAS_PEEK
	const uint8_t *q = CFifoPeek(h);
	CHECK(q != nullptr && BlockIs(q, 0x10U));
	CHECK(CFifoUsed(h) == 1);
#endif

	const uint8_t *r = CFifoGet(h);
	CHECK(r != nullptr && BlockIs(r, 0x10U));
	CHECK(CFifoUsed(h) == 0);
	CHECK(CFifoGet(h) == nullptr);
}

// A one slot FIFO is the smallest ring there is, and full and empty are only
// one operation apart.
static void TestSingleSlot(void)
{
	hCFifo_t h = CFifoInit(s_OneMem, sizeof(s_OneMem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	CHECK(CFifoAvail(h) == 1);

	uint8_t *p = CFifoPut(h);
	CHECK(p != nullptr);
	if (p != nullptr) { FillBlock(p, 0x30U); }

	CHECK(CFifoUsed(h) == 1);
	CHECK(CFifoAvail(h) == 0);
	CHECK(CFifoPut(h) == nullptr);

	const uint8_t *g = CFifoGet(h);
	CHECK(g != nullptr && BlockIs(g, 0x30U));
	CHECK(CFifoUsed(h) == 0);

	// Cycle it enough times that the index wraps the slot many times over.
	for (unsigned i = 0; i < 1000U; i++)
	{
		uint8_t *w = CFifoPut(h);
		CHECK(w != nullptr);
		if (w == nullptr) { return; }
		FillBlock(w, (uint8_t)i);

		const uint8_t *r = CFifoGet(h);
		CHECK(r != nullptr);
		if (r == nullptr || !BlockIs(r, (uint8_t)i))
		{
			printf("  FAIL single slot cycle %u\n", i);
			s_Fail++;
			return;
		}
	}
}

static void TestFullBlocking(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	for (unsigned i = 0; i < POW2_SLOTS; i++)
	{
		uint8_t *p = CFifoPut(h);
		CHECK(p != nullptr);
		if (p != nullptr) { FillBlock(p, (uint8_t)i); }
	}

	CHECK(CFifoAvail(h) == 0);
	CHECK(CFifoUsed(h) == (int)POW2_SLOTS);
	CHECK(CFifoPut(h) == nullptr);
	CHECK(CFifoUsed(h) == (int)POW2_SLOTS);

	// Oldest comes out first and nothing was disturbed by the refused put.
	for (unsigned i = 0; i < POW2_SLOTS; i++)
	{
		const uint8_t *g = CFifoGet(h);
		CHECK(g != nullptr && BlockIs(g, (uint8_t)i));
	}
}

static void TestFullDropping(void)
{
	hCFifo_t h = CFifoInit(s_OddMem, sizeof(s_OddMem), BLK, false);
	if (h == nullptr) { CHECK(false); return; }

	for (unsigned i = 0; i < ODD_SLOTS; i++)
	{
		uint8_t *p = CFifoPut(h);
		CHECK(p != nullptr);
		if (p != nullptr) { FillBlock(p, (uint8_t)i); }
	}

	CHECK(CFifoAvail(h) == 0);

	// A put into a full dropping FIFO must succeed and discard the oldest.
	uint8_t *p = CFifoPut(h);
	CHECK(p != nullptr);
	if (p != nullptr) { FillBlock(p, 0x40U); }
	CHECK(CFifoUsed(h) == (int)ODD_SLOTS);

	const uint8_t *g = CFifoGet(h);
	CHECK(g != nullptr && BlockIs(g, 1U));
}

static void TestGetMultiple(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	for (unsigned i = 0; i < POW2_SLOTS; i++)
	{
		uint8_t *p = CFifoPut(h);
		if (p != nullptr) { FillBlock(p, (uint8_t)(i * 0x10U)); }
	}

	int cnt = (int)POW2_SLOTS;
	const uint8_t *p = CFifoGetMultiple(h, &cnt);
	CHECK(p != nullptr);
	CHECK(cnt == (int)POW2_SLOTS);
	for (int i = 0; i < cnt && p != nullptr; i++)
	{
		CHECK(BlockIs(p + i * BLK, (uint8_t)(i * 0x10U)));
	}
	CHECK(CFifoUsed(h) == 0);

	// More than is present returns what is present.
	uint8_t *w = CFifoPut(h);
	if (w != nullptr) { FillBlock(w, 0x77U); }
	cnt = 100;
	p = CFifoGetMultiple(h, &cnt);
	CHECK(p != nullptr && cnt == 1);
	CHECK(p != nullptr && BlockIs(p, 0x77U));

	// A null count falls back to the single block form.
	w = CFifoPut(h);
	if (w != nullptr) { FillBlock(w, 0x21U); }
	const uint8_t *s = CFifoGetMultiple(h, nullptr);
	CHECK(s != nullptr && BlockIs(s, 0x21U));
}

// The run handed back must stop at the physical end of the buffer.
static void TestGetMultipleStopsAtWrap(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	for (unsigned i = 0; i < POW2_SLOTS - 2U; i++)
	{
		uint8_t *p = CFifoPut(h);
		if (p != nullptr) { FillBlock(p, 0xEEU); }
		(void)CFifoGet(h);
	}

	for (unsigned i = 0; i < 4U; i++)
	{
		uint8_t *p = CFifoPut(h);
		CHECK(p != nullptr);
		if (p != nullptr) { FillBlock(p, (uint8_t)(0x50U + i)); }
	}

	int cnt = 4;
	const uint8_t *p = CFifoGetMultiple(h, &cnt);
	CHECK(p != nullptr);
	CHECK(cnt == 2);
	CHECK(p != nullptr && BlockIs(p, 0x50U));
	CHECK(p != nullptr && BlockIs(p + BLK, 0x51U));

	cnt = 4;
	p = CFifoGetMultiple(h, &cnt);
	CHECK(p != nullptr);
	CHECK(cnt == 2);
	CHECK(p != nullptr && BlockIs(p, 0x52U));
	CHECK(p != nullptr && BlockIs(p + BLK, 0x53U));
	CHECK(CFifoUsed(h) == 0);
}

static void TestPutMultiple(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	int cnt = (int)POW2_SLOTS;
	uint8_t *p = CFifoPutMultiple(h, &cnt);
	CHECK(p != nullptr);
	CHECK(cnt == (int)POW2_SLOTS);
	for (int i = 0; i < cnt && p != nullptr; i++)
	{
		FillBlock(p + i * BLK, (uint8_t)(i + 1));
	}
	CHECK(CFifoUsed(h) == (int)POW2_SLOTS);

	// Full and blocking must hand back nothing and report zero, not a slot
	// the caller would then write into.
	cnt = 2;
	CHECK(CFifoPutMultiple(h, &cnt) == nullptr);
	CHECK(cnt == 0);

	for (unsigned i = 0; i < POW2_SLOTS; i++)
	{
		const uint8_t *g = CFifoGet(h);
		CHECK(g != nullptr && BlockIs(g, (uint8_t)(i + 1)));
	}

	// Asking for more than fits gives what fits, never more.
	cnt = 1000;
	p = CFifoPutMultiple(h, &cnt);
	CHECK(p != nullptr);
	CHECK(cnt > 0 && cnt <= (int)POW2_SLOTS);

	CFifoFlush(h);

	uint8_t *s = CFifoPutMultiple(h, nullptr);
	CHECK(s != nullptr);
	CHECK(CFifoUsed(h) == 1);
}

// Put multiple into a full dropping FIFO. Whatever it returns, the count and
// the pointer have to agree: a non null pointer with a zero count is a slot
// the caller does not own.
static void TestPutMultipleDropping(void)
{
	hCFifo_t h = CFifoInit(s_OddMem, sizeof(s_OddMem), BLK, false);
	if (h == nullptr) { CHECK(false); return; }

	int cnt = (int)ODD_SLOTS;
	uint8_t *p = CFifoPutMultiple(h, &cnt);
	CHECK(p != nullptr && cnt > 0);

	cnt = 2;
	p = CFifoPutMultiple(h, &cnt);
	CHECK((p == nullptr) == (cnt == 0));
	CHECK(cnt >= 0 && cnt <= (int)ODD_SLOTS);
	CHECK(CFifoUsed(h) <= (int)ODD_SLOTS);
	CHECK(CFifoAvail(h) >= 0);
}

static void TestFlush(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	for (unsigned i = 0; i < POW2_SLOTS / 2U; i++)
	{
		uint8_t *p = CFifoPut(h);
		if (p != nullptr) { FillBlock(p, (uint8_t)i); }
	}

	CHECK(CFifoUsed(h) > 0);
	CFifoFlush(h);
	CHECK(CFifoUsed(h) == 0);
	CHECK(CFifoAvail(h) == (int)POW2_SLOTS);
	CHECK(CFifoGet(h) == nullptr);

	uint8_t *p = CFifoPut(h);
	CHECK(p != nullptr);
	if (p != nullptr) { FillBlock(p, 0x99U); }
	const uint8_t *g = CFifoGet(h);
	CHECK(g != nullptr && BlockIs(g, 0x99U));

	// Flush on a full FIFO, and flush twice.
	for (unsigned i = 0; i < POW2_SLOTS; i++)
	{
		(void)CFifoPut(h);
	}
	CFifoFlush(h);
	CFifoFlush(h);
	CHECK(CFifoUsed(h) == 0);
	CHECK(CFifoAvail(h) == (int)POW2_SLOTS);
}

static void TestReadWrite(void)
{
#if HAS_READWRITE
	hCFifo_t h = CFifoInit(s_ByteMem, sizeof(s_ByteMem), 1U, true);
	if (h == nullptr) { CHECK(false); return; }

	uint8_t src[40];
	uint8_t dst[40];

	for (unsigned i = 0; i < sizeof(src); i++)
	{
		src[i] = (uint8_t)(i * 3U + 1U);
	}

	CHECK(CFifoWrite(h, src, (int)sizeof(src)) == (int)sizeof(src));
	memset(dst, 0, sizeof(dst));
	CHECK(CFifoRead(h, dst, (int)sizeof(dst)) == (int)sizeof(dst));
	CHECK(memcmp(src, dst, sizeof(src)) == 0);
	CHECK(CFifoUsed(h) == 0);

	// Block size above one with a length that is not a multiple of it. The
	// guard bytes catch a read that copies whole blocks past the request.
	hCFifo_t b = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (b == nullptr) { CHECK(false); return; }

	uint8_t guarded[64];
	memset(guarded, 0xC3U, sizeof(guarded));

	const int odd = (int)BLK * 3 + 5;
	CHECK(CFifoWrite(b, src, odd) == odd);
	CHECK(CFifoRead(b, guarded, odd) == odd);
	CHECK(memcmp(guarded, src, (size_t)odd) == 0);
	for (unsigned i = (unsigned)odd; i < sizeof(guarded); i++)
	{
		CHECK(guarded[i] == 0xC3U);
	}

	CHECK(CFifoRead(nullptr, dst, 4) == 0);
	CHECK(CFifoRead(b, nullptr, 4) == 0);
	CHECK(CFifoRead(b, dst, 0) == 0);
	CHECK(CFifoWrite(nullptr, src, 4) == 0);
	CHECK(CFifoWrite(b, nullptr, 4) == 0);
	CHECK(CFifoWrite(b, src, 0) == 0);
#else
	s_Skip++;
#endif
}

// Used and avail must always add up to the slot count, whatever sequence of
// operations got the FIFO into its current state.
static void TestUsedAvailInvariant(void)
{
	hCFifo_t h = CFifoInit(s_OddMem, sizeof(s_OddMem), BLK, false);
	if (h == nullptr) { CHECK(false); return; }

	for (unsigned round = 0; round < 500U; round++)
	{
		(void)CFifoPut(h);

		const int used = CFifoUsed(h);
		const int avail = CFifoAvail(h);

		if (used < 0 || used > (int)ODD_SLOTS ||
			avail < 0 || avail > (int)ODD_SLOTS ||
			used + avail != (int)ODD_SLOTS)
		{
			printf("  FAIL invariant round=%u used=%d avail=%d\n",
				   round, used, avail);
			s_Fail++;
			return;
		}

		if ((round % 3U) == 0U)
		{
			(void)CFifoGet(h);
		}
	}
}

static void TestNullHandle(void)
{
	CHECK(CFifoUsed(nullptr) == 0);
	CHECK(CFifoAvail(nullptr) == 0);
	CHECK(CFifoGet(nullptr) == nullptr);
	CHECK(CFifoPut(nullptr) == nullptr);
#if HAS_PEEK
	CHECK(CFifoPeek(nullptr) == nullptr);
#endif
	CFifoFlush(nullptr);

	int cnt = 4;
	CHECK(CFifoGetMultiple(nullptr, &cnt) == nullptr);
	cnt = 4;
	CHECK(CFifoPutMultiple(nullptr, &cnt) == nullptr);
}

// Randomized differential test. A deque holds what a correct FIFO would
// contain, and every operation is applied to both. This is the case that
// finds states no hand written sequence reaches.
static void TestAgainstModel(void)
{
	static const uint32_t slotCounts[2] = { POW2_SLOTS, ODD_SLOTS };
	uint8_t *mem[2] = { s_Pow2Mem, s_OddMem };
	const uint32_t memsize[2] = {
		(uint32_t)sizeof(s_Pow2Mem), (uint32_t)sizeof(s_OddMem)
	};

	for (unsigned k = 0; k < 2U; k++)
	{
		hCFifo_t h = CFifoInit(mem[k], memsize[k], BLK, true);
		if (h == nullptr) { CHECK(false); continue; }

		std::deque<uint8_t> model;
		const size_t cap = slotCounts[k];
		uint8_t seed = 0U;

		for (unsigned step = 0; step < 200000U; step++)
		{
			switch (rand() % 6)
			{
			case 0:
			case 1:
				{
					uint8_t *p = CFifoPut(h);
					if (model.size() >= cap)
					{
						if (p != nullptr)
						{
							printf("  FAIL put succeeded when full, slots=%u\n",
								   slotCounts[k]);
							s_Fail++;
							return;
						}
					}
					else
					{
						if (p == nullptr)
						{
							printf("  FAIL put refused with room, slots=%u\n",
								   slotCounts[k]);
							s_Fail++;
							return;
						}
						FillBlock(p, seed);
						model.push_back(seed);
						seed++;
					}
				}
				break;

			case 2:
			case 3:
				{
					const uint8_t *g = CFifoGet(h);
					if (model.empty())
					{
						if (g != nullptr)
						{
							printf("  FAIL get returned data when empty\n");
							s_Fail++;
							return;
						}
					}
					else
					{
						if (g == nullptr || !BlockIs(g, model.front()))
						{
							printf("  FAIL get wrong value, step %u\n", step);
							s_Fail++;
							return;
						}
						model.pop_front();
					}
				}
				break;

			case 4:
				{
					int want = (rand() % (int)(cap + 2U)) + 1;
					int cnt = want;
					const uint8_t *p = CFifoGetMultiple(h, &cnt);

					if (cnt < 0 || (size_t)cnt > model.size() || cnt > want)
					{
						printf("  FAIL get multiple count %d, model %zu\n",
							   cnt, model.size());
						s_Fail++;
						return;
					}
					if (cnt > 0 && p == nullptr)
					{
						printf("  FAIL get multiple null with count %d\n", cnt);
						s_Fail++;
						return;
					}
					for (int i = 0; i < cnt; i++)
					{
						if (!BlockIs(p + i * BLK, model.front()))
						{
							printf("  FAIL get multiple value at %d\n", i);
							s_Fail++;
							return;
						}
						model.pop_front();
					}
				}
				break;

			default:
				{
					int want = (rand() % (int)(cap + 2U)) + 1;
					int cnt = want;
					uint8_t *p = CFifoPutMultiple(h, &cnt);

					if (cnt < 0 || cnt > want ||
						(size_t)cnt > cap - model.size())
					{
						printf("  FAIL put multiple count %d, room %zu\n",
							   cnt, cap - model.size());
						s_Fail++;
						return;
					}
					if ((p == nullptr) != (cnt == 0))
					{
						printf("  FAIL put multiple pointer and count "
							   "disagree, cnt=%d\n", cnt);
						s_Fail++;
						return;
					}
					for (int i = 0; i < cnt; i++)
					{
						FillBlock(p + i * BLK, seed);
						model.push_back(seed);
						seed++;
					}
				}
				break;
			}

			if ((size_t)CFifoUsed(h) != model.size())
			{
				printf("  FAIL used %d, model %zu, step %u\n",
					   CFifoUsed(h), model.size(), step);
				s_Fail++;
				return;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////
// Timing
//////////////////////////////////////////////////////////////////////////////

struct Result { double Median; double Spread; };

template <typename F>
static Result Measure(unsigned Iters, F Fn)
{
	std::vector<double> runs;

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

static void Timing(void)
{
	printf("\ntiming, median ns per operation over %d repeats\n", s_Repeats);
	printf("%-28s %10s %9s\n", "operation", "ns/op", "spread");

	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { printf("init failed\n"); return; }

	const unsigned iters = 300000U;

	Result pg = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			uint8_t *p = CFifoPut(h);
			if (p != nullptr) { p[0] = (uint8_t)n; }
			const uint8_t *g = CFifoGet(h);
			if (g != nullptr) { s_Sink += g[0]; }
		}
	});
	printf("%-28s %8.2f %8.0f%%\n", "put + get", pg.Median, pg.Spread);

	Result pm = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			int cnt = 4;
			uint8_t *p = CFifoPutMultiple(h, &cnt);
			if (p != nullptr) { p[0] = (uint8_t)n; }
			cnt = 4;
			const uint8_t *g = CFifoGetMultiple(h, &cnt);
			if (g != nullptr) { s_Sink += g[0]; }
		}
	});
	printf("%-28s %8.2f %8.0f%%\n", "put + get multiple, 4",
		   pm.Median, pm.Spread);

	Result fill = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters / POW2_SLOTS; n++)
		{
			for (unsigned i = 0; i < POW2_SLOTS; i++)
			{
				uint8_t *p = CFifoPut(h);
				if (p != nullptr) { p[0] = (uint8_t)i; }
			}
			for (unsigned i = 0; i < POW2_SLOTS; i++)
			{
				const uint8_t *g = CFifoGet(h);
				if (g != nullptr) { s_Sink += g[0]; }
			}
		}
	});
	printf("%-28s %8.2f %8.0f%%\n", "fill and drain, 8 slots",
		   fill.Median, fill.Spread);

	Result ua = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			s_Sink += (uint64_t)CFifoUsed(h);
			s_Sink += (uint64_t)CFifoAvail(h);
		}
	});
	printf("%-28s %8.2f %8.0f%%\n", "used + avail", ua.Median, ua.Spread);

	Result fl = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			uint8_t *p = CFifoPut(h);
			if (p != nullptr) { p[0] = (uint8_t)n; }
			CFifoFlush(h);
			s_Sink += (uint64_t)CFifoUsed(h);
		}
	});
	printf("%-28s %8.2f %8.0f%%\n", "put + flush", fl.Median, fl.Spread);

	hCFifo_t nb = CFifoInit(s_OddMem, sizeof(s_OddMem), BLK, false);
	if (nb == nullptr) { return; }

	// Keep it full so every put takes the drop path.
	for (unsigned i = 0; i < ODD_SLOTS; i++)
	{
		(void)CFifoPut(nb);
	}

	Result drop = Measure(iters, [&]() {
		for (unsigned n = 0; n < iters; n++)
		{
			uint8_t *p = CFifoPut(nb);
			if (p != nullptr) { p[0] = (uint8_t)n; }
		}
	});
	printf("%-28s %8.2f %8.0f%%\n", "put when full, dropping",
		   drop.Median, drop.Spread);
}

struct Case { const char *Name; void (*Fn)(void); };

#define VERDICT_PASS			0
#define VERDICT_FAIL			1
#define VERDICT_SKIP			2
#define VERDICT_CRASH			3

/**
 * Run one case in a child process. The older implementation can divide by
 * zero on arguments the current one rejects, and a crash in one case should
 * be reported rather than ending the comparison.
 */
static int RunIsolated(void (*Fn)(void))
{
	fflush(stdout);

	const pid_t pid = fork();

	if (pid < 0)
	{
		// No child available, so run it here and accept the risk.
		const int failBefore = s_Fail;
		const int skipBefore = s_Skip;
		Fn();
		return s_Fail != failBefore ? VERDICT_FAIL :
			(s_Skip != skipBefore ? VERDICT_SKIP : VERDICT_PASS);
	}

	if (pid == 0)
	{
		s_Fail = 0;
		s_Skip = 0;
		Fn();
		fflush(stdout);
		_exit(s_Fail != 0 ? VERDICT_FAIL :
			  (s_Skip != 0 ? VERDICT_SKIP : VERDICT_PASS));
	}

	int status = 0;
	(void)waitpid(pid, &status, 0);

	if (!WIFEXITED(status))
	{
		return VERDICT_CRASH;
	}

	return WEXITSTATUS(status);
}

int main(int argc, char **argv)
{
	if (argc > 1)
	{
		s_Repeats = atoi(argv[1]);
		if (s_Repeats < 1) { s_Repeats = DEFAULT_REPEATS; }
	}

	srand(20260905U);

	static const Case cases[] = {
		{ "init validation", TestInitValidation },
		{ "empty state", TestEmptyState },
		{ "single round trip", TestSingleRoundTrip },
		{ "single slot ring", TestSingleSlot },
		{ "full, blocking", TestFullBlocking },
		{ "full, dropping", TestFullDropping },
		{ "get multiple", TestGetMultiple },
		{ "get multiple stops at wrap", TestGetMultipleStopsAtWrap },
		{ "put multiple", TestPutMultiple },
		{ "put multiple, dropping", TestPutMultipleDropping },
		{ "flush", TestFlush },
		{ "read and write", TestReadWrite },
		{ "used avail invariant", TestUsedAvailInvariant },
		{ "null handle", TestNullHandle },
		{ "random against model", TestAgainstModel },
	};

	printf("CFifo %s\n", VERSION_NAME);

	int failed = 0;
	int skipped = 0;
	int crashed = 0;

	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
	{
		const int verdict = RunIsolated(cases[i].Fn);

		if (verdict == VERDICT_CRASH)
		{
			printf("%-28s CRASH\n", cases[i].Name);
			crashed++;
		}
		else if (verdict == VERDICT_FAIL)
		{
			printf("%-28s FAIL\n", cases[i].Name);
			failed++;
		}
		else if (verdict == VERDICT_SKIP)
		{
			printf("%-28s skip\n", cases[i].Name);
			skipped++;
		}
		else
		{
			printf("%-28s pass\n", cases[i].Name);
		}
	}

	printf("%s, %d failed, %d crashed, %d skipped\n",
		   (failed == 0 && crashed == 0) ? "all pass" : "FAILURES",
		   failed, crashed, skipped);

	if (RunIsolated(Timing) == VERDICT_CRASH)
	{
		printf("timing CRASHED\n");
		crashed++;
	}

	return (failed == 0 && crashed == 0) ? 0 : 1;
}
