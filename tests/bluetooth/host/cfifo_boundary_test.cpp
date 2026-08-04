// CFIFO initialization and partial-block copy boundaries.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "cfifo.h"

namespace {

bttest::Context s_Test("CFIFO boundary host tests");

void TestRejectsZeroSlots()
{
	alignas(CFifo_t) uint8_t tooSmall[sizeof(CFifo_t) + 7] = {};
	BT_CHECK(s_Test,
		CFifoInit(tooSmall, sizeof(tooSmall), 8, true) == nullptr);

	alignas(CFifo_t) uint8_t exact[CFIFO_TOTAL_MEMSIZE(1, 8)] = {};
	hCFifo_t fifo = CFifoInit(exact, sizeof(exact), 8, true);
	BT_CHECK(s_Test, fifo != nullptr);
	BT_CHECK(s_Test, CFifoAvail(fifo) == 1);
	BT_CHECK(s_Test, CFifoUsed(fifo) == 0);
}

void TestPartialBlockRoundTrip()
{
	alignas(CFifo_t) uint8_t memory[CFIFO_TOTAL_MEMSIZE(2, 8)] = {};
	hCFifo_t fifo = CFifoInit(memory, sizeof(memory), 8, true);
	BT_CHECK(s_Test, fifo != nullptr);

	uint8_t source[10];
	for (size_t i = 0; i < sizeof(source); i++)
	{
		source[i] = (uint8_t)(0x40U + i);
	}

	BT_CHECK(s_Test, CFifoWrite(fifo, source, sizeof(source)) == 10);
	BT_CHECK(s_Test, CFifoUsed(fifo) == 2);
	BT_CHECK(s_Test, CFifoAvail(fifo) == 0);

	uint8_t output[10] = {};
	BT_CHECK(s_Test, CFifoRead(fifo, output, sizeof(output)) == 10);
	BT_CHECK(s_Test, std::memcmp(output, source, sizeof(source)) == 0);
	BT_CHECK(s_Test, CFifoUsed(fifo) == 0);
	BT_CHECK(s_Test, CFifoAvail(fifo) == 2);
}

void TestPartialWriteZeroFillsTail()
{
	alignas(CFifo_t) uint8_t memory[CFIFO_TOTAL_MEMSIZE(2, 8)];
	std::memset(memory, 0xA5, sizeof(memory));
	hCFifo_t fifo = CFifoInit(memory, sizeof(memory), 8, true);
	BT_CHECK(s_Test, fifo != nullptr);

	uint8_t source[10];
	std::memset(source, 0x5A, sizeof(source));
	BT_CHECK(s_Test, CFifoWrite(fifo, source, sizeof(source)) == 10);

	int blocks = 2;
	uint8_t *p = CFifoGetMultiple(fifo, &blocks);
	BT_CHECK(s_Test, p != nullptr);
	BT_CHECK(s_Test, blocks == 2);
	BT_CHECK(s_Test, std::memcmp(p, source, sizeof(source)) == 0);
	for (size_t i = sizeof(source); i < 16; i++)
	{
		BT_CHECK(s_Test, p[i] == 0);
	}
}

void TestBlockingCapacity()
{
	alignas(CFifo_t) uint8_t memory[CFIFO_TOTAL_MEMSIZE(2, 4)] = {};
	hCFifo_t fifo = CFifoInit(memory, sizeof(memory), 4, true);
	BT_CHECK(s_Test, fifo != nullptr);

	uint8_t source[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	BT_CHECK(s_Test, CFifoWrite(fifo, source, sizeof(source)) == 8);
	BT_CHECK(s_Test, CFifoPut(fifo) == nullptr);
	BT_CHECK(s_Test, CFifoUsed(fifo) == 2);
	BT_CHECK(s_Test, CFifoAvail(fifo) == 0);
}

void TestPeekDoesNotConsume()
{
	alignas(CFifo_t) uint8_t memory[CFIFO_TOTAL_MEMSIZE(2, 8)] = {};
	hCFifo_t fifo = CFifoInit(memory, sizeof(memory), 8, true);
	BT_CHECK(s_Test, fifo != nullptr);
	BT_CHECK(s_Test, CFifoPeek(fifo) == nullptr);

	uint8_t expected[8] = { 0x10, 0x11, 0x12, 0x13,
		0x14, 0x15, 0x16, 0x17 };
	uint8_t *pPut = CFifoPut(fifo);
	BT_CHECK(s_Test, pPut != nullptr);
	if (pPut != nullptr)
	{
		std::memcpy(pPut, expected, sizeof(expected));
	}

	BT_CHECK(s_Test, CFifoUsed(fifo) == 1);
	uint8_t *pPeek1 = CFifoPeek(fifo);
	uint8_t *pPeek2 = CFifoPeek(fifo);
	BT_CHECK(s_Test, pPeek1 != nullptr);
	BT_CHECK(s_Test, pPeek2 == pPeek1);
	BT_CHECK(s_Test, CFifoUsed(fifo) == 1);
	if (pPeek1 != nullptr)
	{
		BT_CHECK(s_Test,
			std::memcmp(pPeek1, expected, sizeof(expected)) == 0);
	}

	uint8_t *pGet = CFifoGet(fifo);
	BT_CHECK(s_Test, pGet == pPeek1);
	BT_CHECK(s_Test, CFifoUsed(fifo) == 0);
	BT_CHECK(s_Test, CFifoPeek(fifo) == nullptr);
}

} // namespace

int main()
{
	s_Test.Run("reject zero slots", TestRejectsZeroSlots);
	s_Test.Run("partial block round trip", TestPartialBlockRoundTrip);
	s_Test.Run("partial write zero fill", TestPartialWriteZeroFillsTail);
	s_Test.Run("blocking capacity", TestBlockingCapacity);
	s_Test.Run("peek does not consume", TestPeekDoesNotConsume);
	return s_Test.Finish();
}
