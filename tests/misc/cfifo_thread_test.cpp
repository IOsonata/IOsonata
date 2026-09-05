/**-------------------------------------------------------------------------
@file	cfifo_thread_test.cpp

@brief	Concurrent access tests for CFifo.

CFifo is used from an interrupt on every port, so the producer and the
consumer really do run at the same time. These cases run them in two threads
and check what survives.

One thing shapes every case here. CFifoPut advances PutIdx and then returns
the slot for the caller to fill:

	uint32_t slot = cfifo_slot(pFifo, putIdx);
	CFIFO_ATOMIC_STORE(&pFifo->PutIdx, putIdx + 1U, __ATOMIC_RELEASE);
	return cfifo_addr(pFifo, slot);

So the block is visible to a consumer before the producer has written it.
The index handover is atomic, the block contents are not. A producer that
can be preempted between the put and the fill has to keep the consumer out
for that window, which on a Cortex-M port means interrupts off. usb_intrf
does exactly that. These tests measure both sides of it: with the window
closed, and with it open.

Build with -fsanitize=thread for race detection. That cannot be combined
with the address sanitizer, so it has its own target.

Usage:

	./cfifo_thread_test [operations]

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
#include <atomic>
#include <mutex>
#include <thread>

#include "cfifo.h"

#ifdef CFIFO_LEGACY
#define VERSION_NAME			"2021 (4c69ab6c)"
typedef HCFIFO					hCFifo_t;
#else
#define VERSION_NAME			"current"
#endif

// Thread sanitizer stops on the first race, so the case that deliberately
// creates one is left out of that build.
#if defined(__SANITIZE_THREAD__)
#define UNDER_TSAN				1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define UNDER_TSAN				1
#else
#define UNDER_TSAN				0
#endif
#else
#define UNDER_TSAN				0
#endif

#define BLK						16U
#define WORDS_PER_BLK			(BLK / sizeof(uint32_t))
#define SLOTS					8U
#define DEFAULT_OPS				2000000U

alignas(4) static uint8_t s_Mem[CFIFO_TOTAL_MEMSIZE(SLOTS, BLK)];

static int s_Fail;
static unsigned s_Ops = DEFAULT_OPS;

#define CHECK(c) do { if (!(c)) { \
	printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

// Stands in for the interrupt disable a port uses around the put and the
// fill. Both sides take it, which is what interrupts off amounts to: the
// consumer cannot run inside the producer's window.
static std::mutex s_Critical;

/// Write the sequence number into every word of the block, so a block read
/// before the producer filled it, or read while it was being filled, shows
/// up as a word that does not match the rest.
static void FillSeq(uint8_t *pBlk, uint32_t Seq)
{
	uint32_t *pW = (uint32_t *)pBlk;

	for (unsigned i = 0; i < WORDS_PER_BLK; i++)
	{
		pW[i] = Seq;
	}
}

/// Return the sequence number if every word agrees, otherwise report a tear.
static bool ReadSeq(const uint8_t *pBlk, uint32_t *pSeq)
{
	const uint32_t *pW = (const uint32_t *)pBlk;
	const uint32_t first = pW[0];

	for (unsigned i = 1; i < WORDS_PER_BLK; i++)
	{
		if (pW[i] != first)
		{
			return false;
		}
	}

	*pSeq = first;
	return true;
}

//////////////////////////////////////////////////////////////////////////////
// Single producer, single consumer, with the producer window closed. This is
// the pattern every port uses and the one that has to be exactly right.
//////////////////////////////////////////////////////////////////////////////

static void TestSpscGuarded(void)
{
	hCFifo_t h = CFifoInit(s_Mem, sizeof(s_Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	std::atomic<uint32_t> produced(0U);
	std::atomic<bool> done(false);
	std::atomic<uint32_t> torn(0U);
	std::atomic<uint32_t> outOfOrder(0U);
	std::atomic<uint32_t> consumed(0U);

	std::thread producer([&]() {
		uint32_t seq = 0U;

		while (seq < s_Ops)
		{
			std::lock_guard<std::mutex> lock(s_Critical);
			uint8_t *p = CFifoPut(h);
			if (p != nullptr)
			{
				FillSeq(p, seq);
				seq++;
			}
		}

		produced.store(seq, std::memory_order_release);
		done.store(true, std::memory_order_release);
	});

	std::thread consumer([&]() {
		uint32_t expect = 0U;

		for (;;)
		{
			uint32_t seq = 0U;
			bool got = false;

			{
				std::lock_guard<std::mutex> lock(s_Critical);
				const uint8_t *g = CFifoGet(h);
				if (g != nullptr)
				{
					if (!ReadSeq(g, &seq))
					{
						torn.fetch_add(1U, std::memory_order_relaxed);
					}
					got = true;
				}
			}

			if (got)
			{
				if (seq != expect)
				{
					outOfOrder.fetch_add(1U, std::memory_order_relaxed);
				}
				expect = seq + 1U;
				consumed.fetch_add(1U, std::memory_order_relaxed);
			}
			else if (done.load(std::memory_order_acquire))
			{
				break;
			}
		}
	});

	producer.join();
	consumer.join();

	// Every item put must come out, exactly once, in order, whole.
	CHECK(torn.load() == 0U);
	CHECK(outOfOrder.load() == 0U);
	CHECK(consumed.load() == produced.load());
	CHECK(CFifoUsed(h) == 0);

	if (torn.load() != 0U || outOfOrder.load() != 0U)
	{
		printf("  torn %u, out of order %u, produced %u, consumed %u\n",
			   torn.load(), outOfOrder.load(),
			   produced.load(), consumed.load());
	}
}

//////////////////////////////////////////////////////////////////////////////
// Same, through the multiple block calls.
//////////////////////////////////////////////////////////////////////////////

static void TestSpscMultipleGuarded(void)
{
	hCFifo_t h = CFifoInit(s_Mem, sizeof(s_Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	std::atomic<uint32_t> produced(0U);
	std::atomic<uint32_t> consumed(0U);
	std::atomic<bool> done(false);
	std::atomic<uint32_t> torn(0U);
	std::atomic<uint32_t> outOfOrder(0U);
	std::atomic<uint32_t> nullWithCount(0U);

	std::thread producer([&]() {
		uint32_t seq = 0U;

		while (seq < s_Ops)
		{
			std::lock_guard<std::mutex> lock(s_Critical);
			int cnt = 1 + (int)(seq % 4U);
			uint8_t *p = CFifoPutMultiple(h, &cnt);

			// A null pointer with a non zero count is a slot the caller
			// does not own. Count it rather than write through it.
			if (p == nullptr)
			{
				if (cnt != 0)
				{
					nullWithCount.fetch_add(1U, std::memory_order_relaxed);
				}
				continue;
			}

			for (int i = 0; i < cnt; i++)
			{
				FillSeq(p + i * BLK, seq);
				seq++;
			}
		}

		produced.store(seq, std::memory_order_release);
		done.store(true, std::memory_order_release);
	});

	std::thread consumer([&]() {
		uint32_t expect = 0U;

		for (;;)
		{
			int cnt = 4;
			bool got = false;

			{
				std::lock_guard<std::mutex> lock(s_Critical);
				const uint8_t *g = CFifoGetMultiple(h, &cnt);

				for (int i = 0; i < cnt && g != nullptr; i++)
				{
					uint32_t seq = 0U;
					if (!ReadSeq(g + i * BLK, &seq))
					{
						torn.fetch_add(1U, std::memory_order_relaxed);
					}
					else if (seq != expect)
					{
						outOfOrder.fetch_add(1U, std::memory_order_relaxed);
					}
					expect = seq + 1U;
					consumed.fetch_add(1U, std::memory_order_relaxed);
					got = true;
				}
			}

			if (!got && done.load(std::memory_order_acquire) &&
				CFifoUsed(h) == 0)
			{
				break;
			}
		}
	});

	producer.join();
	consumer.join();

	CHECK(torn.load() == 0U);
	CHECK(outOfOrder.load() == 0U);
	CHECK(consumed.load() == produced.load());
	CHECK(CFifoUsed(h) == 0);
	CHECK(nullWithCount.load() == 0U);

	if (nullWithCount.load() != 0U)
	{
		printf("  put multiple returned null with a non zero count %u "
			   "times\n", nullWithCount.load());
	}
}

//////////////////////////////////////////////////////////////////////////////
// Drop oldest mode with a live consumer. This is the only path where the
// producer writes GetIdx, and the compare and swap in CFifoPut exists for
// exactly this race. Items may be dropped, so the check is that what does
// come out is whole and strictly increasing.
//////////////////////////////////////////////////////////////////////////////

static void TestDroppingConcurrent(void)
{
	hCFifo_t h = CFifoInit(s_Mem, sizeof(s_Mem), BLK, false);
	if (h == nullptr) { CHECK(false); return; }

	std::atomic<bool> done(false);
	std::atomic<uint32_t> torn(0U);
	std::atomic<uint32_t> backwards(0U);
	std::atomic<uint32_t> consumed(0U);

	std::thread producer([&]() {
		for (uint32_t seq = 0U; seq < s_Ops; seq++)
		{
			std::lock_guard<std::mutex> lock(s_Critical);
			uint8_t *p = CFifoPut(h);
			if (p != nullptr)
			{
				FillSeq(p, seq);
			}
		}

		done.store(true, std::memory_order_release);
	});

	std::thread consumer([&]() {
		uint32_t last = 0U;
		bool first = true;

		for (;;)
		{
			uint32_t seq = 0U;
			bool got = false;

			{
				std::lock_guard<std::mutex> lock(s_Critical);
				const uint8_t *g = CFifoGet(h);
				if (g != nullptr)
				{
					if (!ReadSeq(g, &seq))
					{
						torn.fetch_add(1U, std::memory_order_relaxed);
					}
					got = true;
				}
			}

			if (got)
			{
				if (!first && seq <= last)
				{
					backwards.fetch_add(1U, std::memory_order_relaxed);
				}
				last = seq;
				first = false;
				consumed.fetch_add(1U, std::memory_order_relaxed);
			}
			else if (done.load(std::memory_order_acquire))
			{
				break;
			}
		}
	});

	producer.join();
	consumer.join();

	// Dropping is allowed. Repeats, reordering and torn blocks are not.
	CHECK(torn.load() == 0U);
	CHECK(backwards.load() == 0U);
	CHECK(CFifoUsed(h) >= 0 && CFifoUsed(h) <= (int)SLOTS);

	if (backwards.load() != 0U)
	{
		printf("  out of order or repeated %u of %u\n",
			   backwards.load(), consumed.load());
	}
}

//////////////////////////////////////////////////////////////////////////////
// Index integrity while both sides run flat out. Nothing here reads block
// contents, so it isolates the index arithmetic from the data handover.
//////////////////////////////////////////////////////////////////////////////

static void TestIndexIntegrity(void)
{
	hCFifo_t h = CFifoInit(s_Mem, sizeof(s_Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	std::atomic<bool> done(false);
	std::atomic<uint32_t> bad(0U);
	std::atomic<uint32_t> disagreed(0U);
	std::atomic<uint32_t> reads(0U);
	std::atomic<uint32_t> puts(0U);
	std::atomic<uint32_t> gets(0U);

	std::thread producer([&]() {
		for (uint32_t n = 0U; n < s_Ops; n++)
		{
			if (CFifoPut(h) != nullptr)
			{
				puts.fetch_add(1U, std::memory_order_relaxed);
			}
		}

		done.store(true, std::memory_order_release);
	});

	std::thread consumer([&]() {
		for (;;)
		{
			if (CFifoGet(h) != nullptr)
			{
				gets.fetch_add(1U, std::memory_order_relaxed);
			}
			else if (done.load(std::memory_order_acquire))
			{
				break;
			}

			const int used = CFifoUsed(h);
			const int avail = CFifoAvail(h);

			// Each must be in range on its own. They are not required to
			// add up: each call takes its own pair of relaxed loads, so
			// the other side can move an index between them, and between
			// the two calls. How often that shows is counted below.
			if (used < 0 || used > (int)SLOTS ||
				avail < 0 || avail > (int)SLOTS)
			{
				bad.fetch_add(1U, std::memory_order_relaxed);
			}

			if (used + avail != (int)SLOTS)
			{
				disagreed.fetch_add(1U, std::memory_order_relaxed);
			}

			reads.fetch_add(1U, std::memory_order_relaxed);
		}
	});

	producer.join();
	consumer.join();

	// Both threads are joined here, so these are quiet reads.
	CHECK(bad.load() == 0U);
	CHECK(gets.load() <= puts.load());
	CHECK(puts.load() - gets.load() == (uint32_t)CFifoUsed(h));

	if (bad.load() != 0U)
	{
		printf("  out of range %u times\n", bad.load());
	}

	// Not a defect. Each side gets a conservative answer about its own
	// index: a producer reading avail can only understate the room, a
	// consumer reading used can only understate what is waiting. Worth
	// knowing that the two calls do not add up while both sides run.
	printf("  used and avail disagreed %u of %u concurrent reads\n",
		   disagreed.load(), reads.load());
}

//////////////////////////////////////////////////////////////////////////////
// The producer window left open. CFifoPut publishes the slot before the
// caller fills it, so a consumer running in that window reads a block that
// holds whatever was there before. This is not a defect to fix in CFifo, it
// is the reason the callers close the window, and this case measures what
// happens when they do not.
//////////////////////////////////////////////////////////////////////////////

static void TestSpscUnguarded(void)
{
#if UNDER_TSAN
	printf("  skipped under thread sanitizer, the race is deliberate\n");
#else
	hCFifo_t h = CFifoInit(s_Mem, sizeof(s_Mem), BLK, true);
	if (h == nullptr) { CHECK(false); return; }

	std::atomic<bool> done(false);
	std::atomic<uint32_t> torn(0U);
	std::atomic<uint32_t> wrong(0U);
	std::atomic<uint32_t> consumed(0U);

	std::thread producer([&]() {
		for (uint32_t seq = 0U; seq < s_Ops; seq++)
		{
			uint8_t *p = CFifoPut(h);
			while (p == nullptr)
			{
				p = CFifoPut(h);
			}
			FillSeq(p, seq);
		}

		done.store(true, std::memory_order_release);
	});

	std::thread consumer([&]() {
		uint32_t expect = 0U;

		for (;;)
		{
			const uint8_t *g = CFifoGet(h);

			if (g != nullptr)
			{
				uint32_t seq = 0U;
				if (!ReadSeq(g, &seq))
				{
					torn.fetch_add(1U, std::memory_order_relaxed);
				}
				else if (seq != expect)
				{
					wrong.fetch_add(1U, std::memory_order_relaxed);
					expect = seq;
				}
				expect++;
				consumed.fetch_add(1U, std::memory_order_relaxed);
			}
			else if (done.load(std::memory_order_acquire))
			{
				break;
			}
		}
	});

	producer.join();
	consumer.join();

	// No assertion. The numbers are the point: they say how much data a
	// caller loses by not closing the window around put and fill.
	printf("  torn %u, wrong value %u, of %u consumed\n",
		   torn.load(), wrong.load(), consumed.load());
#endif
}

struct Case { const char *Name; void (*Fn)(void); };

int main(int argc, char **argv)
{
	if (argc > 1)
	{
		const int n = atoi(argv[1]);
		if (n > 0) { s_Ops = (unsigned)n; }
	}

	static const Case cases[] = {
		{ "spsc, window closed", TestSpscGuarded },
		{ "spsc multiple, window closed", TestSpscMultipleGuarded },
		{ "dropping, concurrent", TestDroppingConcurrent },
		{ "index integrity", TestIndexIntegrity },
		{ "spsc, window open", TestSpscUnguarded },
	};

	printf("CFifo %s concurrency, %u operations per case%s\n",
		   VERSION_NAME, s_Ops,
		   UNDER_TSAN ? ", thread sanitizer on" : "");

	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
	{
		const int before = s_Fail;
		cases[i].Fn();
		printf("%-32s %s\n", cases[i].Name,
			   s_Fail == before ? "pass" : "FAIL");
	}

	printf("%s\n", s_Fail == 0 ? "all pass" : "FAILURES");
	return s_Fail == 0 ? 0 : 1;
}
