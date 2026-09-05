/**-------------------------------------------------------------------------
@file	cfifo_test.cpp

@brief	Host regression tests for CFifo.

Covers every entry point, both the power of two slot count that takes the
mask path and a count that takes the modulo path, both blocking and drop
oldest behaviour, and the ring wrap in every operation that can meet it.

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
#include <string.h>

#include "cfifo.h"

#define BLK				8U
#define POW2_SLOTS		8U
#define ODD_SLOTS		5U
#define BIG_SLOTS		64U

alignas(4) static uint8_t s_Pow2Mem[CFIFO_TOTAL_MEMSIZE(POW2_SLOTS, BLK)];
alignas(4) static uint8_t s_OddMem[CFIFO_TOTAL_MEMSIZE(ODD_SLOTS, BLK)];
alignas(4) static uint8_t s_BigMem[CFIFO_TOTAL_MEMSIZE(BIG_SLOTS, BLK)];
alignas(4) static uint8_t s_ByteMem[CFIFO_MEMSIZE(64)];

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

/// Write a recognisable pattern into one block.
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

static void TestInitValidation(void)
{
	alignas(4) static uint8_t tiny[sizeof(CFifo_t) + 4U];

	// Null memory, zero block size.
	CHECK(CFifoInit(nullptr, sizeof(s_Pow2Mem), BLK, true) == nullptr);
	CHECK(CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), 0U, true) == nullptr);

	// Room for the header but not one whole block.
	CHECK(CFifoInit(tiny, sizeof(CFifo_t), BLK, true) == nullptr);
	CHECK(CFifoInit(tiny, sizeof(CFifo_t) + BLK - 1U, BLK, true) == nullptr);

	// Exactly one block fits.
	hCFifo_t h = CFifoInit(tiny, sizeof(CFifo_t) + 4U, 4U, true);
	CHECK(h != nullptr);
	CHECK(h != nullptr && CFifoAvail(h) == 1);
	CHECK(h != nullptr && CFifoUsed(h) == 0);
}

static void TestGeometryAndAccessors(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	CHECK(h != nullptr);
	if (h == nullptr) { return; }

	CHECK(CFifoBlockSize(h) == BLK);
	CHECK(CFifoIsBlocking(h) == true);
	CHECK(CFifoAvail(h) == (int)POW2_SLOTS);
	CHECK(CFifoUsed(h) == 0);
	CHECK(CFifoPeek(h) == nullptr);
	CHECK(CFifoGet(h) == nullptr);

	hCFifo_t nb = CFifoInit(s_OddMem, sizeof(s_OddMem), BLK, false);
	CHECK(nb != nullptr);
	CHECK(nb != nullptr && CFifoIsBlocking(nb) == false);
	CHECK(nb != nullptr && CFifoAvail(nb) == (int)ODD_SLOTS);
}

// Single put and get, and the peek that must not consume.
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

	const uint8_t *q = CFifoPeek(h);
	CHECK(q != nullptr && BlockIs(q, 0x10U));
	CHECK(CFifoUsed(h) == 1);

	const uint8_t *r = CFifoGet(h);
	CHECK(r == q);
	CHECK(r != nullptr && BlockIs(r, 0x10U));
	CHECK(CFifoUsed(h) == 0);
	CHECK(CFifoGet(h) == nullptr);
}

// Fill to capacity, then confirm a blocking FIFO refuses and a dropping one
// discards the oldest and counts it.
static void TestFullBehaviour(void)
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

	// Oldest first.
	const uint8_t *g = CFifoGet(h);
	CHECK(g != nullptr && BlockIs(g, 0U));

	hCFifo_t nb = CFifoInit(s_OddMem, sizeof(s_OddMem), BLK, false);
	if (nb == nullptr) { CHECK(false); return; }

	for (unsigned i = 0; i < ODD_SLOTS; i++)
	{
		uint8_t *p = CFifoPut(nb);
		CHECK(p != nullptr);
		if (p != nullptr) { FillBlock(p, (uint8_t)i); }
	}

	CHECK(CFifoAvail(nb) == 0);

	// Dropping mode always hands back a slot, and the oldest entry goes.
	uint8_t *p = CFifoPut(nb);
	CHECK(p != nullptr);
	if (p != nullptr) { FillBlock(p, 0x40U); }
	CHECK(CFifoUsed(nb) == (int)ODD_SLOTS);

	const uint8_t *g2 = CFifoGet(nb);
	CHECK(g2 != nullptr && BlockIs(g2, 1U));
}

// A get of more than one block returns only the run before the wrap.
static void TestGetMultipleContiguous(void)
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
	for (int i = 0; i < cnt; i++)
	{
		CHECK(BlockIs(p + i * BLK, (uint8_t)(i * 0x10U)));
	}
	CHECK(CFifoUsed(h) == 0);

	// Asking for more than is present returns what is present.
	uint8_t *w = CFifoPut(h);
	if (w != nullptr) { FillBlock(w, 0x77U); }
	cnt = 100;
	p = CFifoGetMultiple(h, &cnt);
	CHECK(p != nullptr && cnt == 1);
	CHECK(p != nullptr && BlockIs(p, 0x77U));

	// Empty reports zero and no pointer.
	cnt = 4;
	CHECK(CFifoGetMultiple(h, &cnt) == nullptr);
	CHECK(cnt == 0);

	// A null count falls back to the single block form.
	w = CFifoPut(h);
	if (w != nullptr) { FillBlock(w, 0x21U); }
	const uint8_t *s = CFifoGetMultiple(h, nullptr);
	CHECK(s != nullptr && BlockIs(s, 0x21U));

	// A non positive count is rejected.
	cnt = 0;
	CHECK(CFifoGetMultiple(h, &cnt) == nullptr);
	CHECK(cnt == 0);
	cnt = -3;
	CHECK(CFifoGetMultiple(h, &cnt) == nullptr);
	CHECK(cnt == 0);
}

// The run returned must stop at the physical end of the buffer, so a reader
// that has wrapped gets two shorter runs rather than one that walks off.
static void TestGetMultipleStopsAtWrap(void)
{
	hCFifo_t h = CFifoInit(s_Pow2Mem, sizeof(s_Pow2Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	// Advance the read point so the next put wraps.
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
	for (int i = 0; i < cnt; i++)
	{
		FillBlock(p + i * BLK, (uint8_t)(i + 1));
	}
	CHECK(CFifoUsed(h) == (int)POW2_SLOTS);
	CHECK(CFifoAvail(h) == 0);

	// Full and blocking returns nothing and reports zero.
	cnt = 2;
	CHECK(CFifoPutMultiple(h, &cnt) == nullptr);
	CHECK(cnt == 0);

	for (unsigned i = 0; i < POW2_SLOTS; i++)
	{
		const uint8_t *g = CFifoGet(h);
		CHECK(g != nullptr && BlockIs(g, (uint8_t)(i + 1)));
	}

	// Asking for more than fits gives what fits.
	cnt = 1000;
	p = CFifoPutMultiple(h, &cnt);
	CHECK(p != nullptr);
	CHECK(cnt > 0 && cnt <= (int)POW2_SLOTS);

	CFifoFlush(h);

	// A null count falls back to the single block form.
	uint8_t *s = CFifoPutMultiple(h, nullptr);
	CHECK(s != nullptr);
	CHECK(CFifoUsed(h) == 1);

	// A non positive count is rejected.
	cnt = 0;
	CHECK(CFifoPutMultiple(h, &cnt) == nullptr);
	CHECK(cnt == 0);
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
	CHECK(CFifoPeek(h) == nullptr);

	// Still usable afterwards.
	uint8_t *p = CFifoPut(h);
	CHECK(p != nullptr);
	if (p != nullptr) { FillBlock(p, 0x99U); }
	const uint8_t *g = CFifoGet(h);
	CHECK(g != nullptr && BlockIs(g, 0x99U));
}

// Read and write walk whole blocks. A length that is not a block multiple
// must copy only what the caller asked for.
static void TestReadWrite(void)
{
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

	// Block size larger than one, with a length that is not a multiple of it.
	hCFifo_t b = CFifoInit(s_BigMem, sizeof(s_BigMem), BLK, true);
	if (b == nullptr) { CHECK(false); return; }

	// Guard bytes catch a read that copies whole blocks past the request.
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

	// Null and non positive arguments.
	CHECK(CFifoRead(nullptr, dst, 4) == 0);
	CHECK(CFifoRead(b, nullptr, 4) == 0);
	CHECK(CFifoRead(b, dst, 0) == 0);
	CHECK(CFifoWrite(nullptr, src, 4) == 0);
	CHECK(CFifoWrite(b, nullptr, 4) == 0);
	CHECK(CFifoWrite(b, src, 0) == 0);
}

// Many cycles through the ring, so every slot is used at every position and
// the wrap arithmetic is exercised in both the mask and modulo forms.
static void TestLongRunWrap(void)
{
	static const uint32_t slots[2] = { POW2_SLOTS, ODD_SLOTS };
	uint8_t *mem[2] = { s_Pow2Mem, s_OddMem };
	const uint32_t memsize[2] = {
		(uint32_t)sizeof(s_Pow2Mem), (uint32_t)sizeof(s_OddMem)
	};

	for (unsigned k = 0; k < 2U; k++)
	{
		hCFifo_t h = CFifoInit(mem[k], memsize[k], BLK, true);
		if (h == nullptr) { CHECK(false); continue; }

		uint8_t seed = 0U;
		uint8_t expect = 0U;

		for (unsigned round = 0; round < 500U; round++)
		{
			const unsigned batch = (round % slots[k]) + 1U;

			for (unsigned i = 0; i < batch; i++)
			{
				uint8_t *p = CFifoPut(h);
				if (p == nullptr) { break; }
				FillBlock(p, seed++);
			}

			while (CFifoUsed(h) > 0)
			{
				const uint8_t *g = CFifoGet(h);
				CHECK(g != nullptr);
				if (g == nullptr) { break; }
				if (!BlockIs(g, expect))
				{
					printf("FAIL wrap slots=%u round=%u\n", slots[k], round);
					s_Fail++;
					return;
				}
				expect++;
			}
		}

		CHECK(CFifoUsed(h) == 0);
		CHECK(CFifoAvail(h) == (int)slots[k]);
	}
}

// Used and avail must always add up to the slot count and never report a
// value outside that range.
static void TestUsedAvailInvariant(void)
{
	hCFifo_t h = CFifoInit(s_OddMem, sizeof(s_OddMem), BLK, false);
	if (h == nullptr) { CHECK(false); return; }

	for (unsigned round = 0; round < 200U; round++)
	{
		uint8_t *p = CFifoPut(h);
		CHECK(p != nullptr);

		const int used = CFifoUsed(h);
		const int avail = CFifoAvail(h);

		CHECK(used >= 0 && used <= (int)ODD_SLOTS);
		CHECK(avail >= 0 && avail <= (int)ODD_SLOTS);
		CHECK(used + avail == (int)ODD_SLOTS);

		if ((round % 3U) == 0U)
		{
			(void)CFifoGet(h);
		}
	}

	// Null handle is answered, not crashed.
	CHECK(CFifoUsed(nullptr) == 0);
	CHECK(CFifoAvail(nullptr) == 0);
	CHECK(CFifoGet(nullptr) == nullptr);
	CHECK(CFifoPut(nullptr) == nullptr);
	CHECK(CFifoPeek(nullptr) == nullptr);
	CFifoFlush(nullptr);
}

struct Case { const char *Name; void (*Fn)(void); };

int main(void)
{
	static const Case cases[] = {
		{ "init validation", TestInitValidation },
		{ "geometry and accessors", TestGeometryAndAccessors },
		{ "single round trip", TestSingleRoundTrip },
		{ "full behaviour", TestFullBehaviour },
		{ "get multiple contiguous", TestGetMultipleContiguous },
		{ "get multiple stops at wrap", TestGetMultipleStopsAtWrap },
		{ "put multiple", TestPutMultiple },
		{ "flush", TestFlush },
		{ "read and write", TestReadWrite },
		{ "used avail invariant", TestUsedAvailInvariant },
		{ "long run wrap", TestLongRunWrap },
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
