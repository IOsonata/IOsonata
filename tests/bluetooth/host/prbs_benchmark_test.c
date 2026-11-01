/* Exercise the shared notification path as both C (ST) and C++ (IOsonata). */
#include <assert.h>
#include <stdio.h>
#include "ble_prbs_benchmark.h"

static PrbsBenchmark_t s_Benchmark;
static uint32_t s_Now;
static unsigned s_Done;
static unsigned s_Scheduled;

static uint32_t TestClock(void)
{
	return s_Now;
}

static bool TestSchedule(void)
{
	s_Scheduled++;
	return true;
}

static bool TestWrite(const uint8_t *pData, uint16_t Len)
{
	return fwrite(pData, 1, Len, stdout) == Len;
}

static void TestDone(void)
{
	s_Done++;
}

static const PrbsBenchAdapters_t s_Adapters = {
	TestClock,
	TestSchedule,
	TestWrite,
	TestDone
};

static void TestTrial(uint8_t Phy, unsigned Packets)
{
	uint8_t data[PRBS_BENCH_PAYLOAD];
	uint8_t previous;
	uint32_t start;
	uint32_t elapsed;
	unsigned packet;
	unsigned i;

	PrbsBenchReset(&s_Benchmark, Phy);
	start = 0xfffffff0U;
	previous = 0xff;
	for (elapsed = 0; elapsed < PRBS_BENCH_DURATION; elapsed += 15)
	{
		s_Now = start + elapsed;
		for (packet = 0; packet < Packets; packet++)
		{
			for (i = 0; i < sizeof(data); i++)
			{
				previous = ((previous << 1) | (((previous >> 6) ^ (previous >> 5)) & 1)) & 0x7f;
				data[i] = previous;
			}
			PrbsBenchNotify(&s_Benchmark, data, sizeof(data));
		}
		// Reusing the sender buffer cannot change a copied notification.
		memset(data, 0, sizeof(data));
		while (CFifoUsed(s_Benchmark.Queue) != 0)
		{
			PrbsBenchReceive(&s_Benchmark);
		}
	}
	s_Now = start + PRBS_BENCH_DURATION;
	PrbsBenchPoll(&s_Benchmark, s_Now);
	assert(s_Benchmark.Stats.Bytes == Packets * 4000 * PRBS_BENCH_PAYLOAD);
	assert(s_Benchmark.Stats.Notifications == Packets * 4000);
	assert(s_Benchmark.Stats.Breaks == 0);
	assert(s_Benchmark.Stats.OutOfFrame == 0);
	assert(s_Benchmark.Dropped == 0);
	assert(s_Benchmark.FinalPrinted && s_Benchmark.Complete);
}

int main(void)
{
	assert(PrbsBenchInit(&s_Benchmark, &s_Adapters, 1));
	TestTrial(1, 4);
	TestTrial(2, 8);
	assert(s_Done == 2);
	assert(s_Scheduled != 0);
	return 0;
}
