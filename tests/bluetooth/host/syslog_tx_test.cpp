// Coverage for SysLog direct output and optional CFifo record buffering.
// DeviceIntrf owns transport transfer behavior. SysLog performs one
// DeviceIntrfTx call for direct output and one call per buffered record.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "cfifo.h"
#include "device_intrf.h"
#include "syslog.h"

namespace {

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

uint8_t s_Captured[512];
int s_CapturedLen = 0;
int s_ChunkLimit = 512;
int s_TxCalls = 0;

bool MockStartTx(DevIntrf_t * const, uint32_t)
{
	return true;
}

void MockStopTx(DevIntrf_t * const)
{
}

int MockTxData(DevIntrf_t * const, const uint8_t *pData, int DataLen)
{
	s_TxCalls++;
	int n = DataLen < s_ChunkLimit ? DataLen : s_ChunkLimit;
	if (n > 0 && s_CapturedLen + n <= (int)sizeof(s_Captured))
	{
		std::memcpy(s_Captured + s_CapturedLen, pData, (size_t)n);
		s_CapturedLen += n;
	}
	return n;
}

void MakeIntrf(DevIntrf_t *pIntrf)
{
	atomic_flag_clear(&pIntrf->bBusy);
	atomic_store(&pIntrf->EnCnt, 0);
	atomic_store(&pIntrf->bTxReady, false);
	atomic_store(&pIntrf->bNoStop, false);
	pIntrf->StartTx = MockStartTx;
	pIntrf->StopTx = MockStopTx;
	pIntrf->TxData = MockTxData;
	pIntrf->MaxRetry = 3;
}

void ResetCapture(int ChunkLimit = 512)
{
	std::memset(s_Captured, 0, sizeof(s_Captured));
	s_CapturedLen = 0;
	s_ChunkLimit = ChunkLimit;
	s_TxCalls = 0;
}

int FlushAll(SysLog_t *pLog, hCFifo_t hFifo)
{
	int total = 0;
	while (CFifoUsed(hFifo) > 0)
	{
		int n = SysLogFlush(pLog);
		if (n <= 0)
		{
			break;
		}
		total += n;
	}
	return total;
}

// SysLog does not implement a second partial-transfer loop above DeviceIntrf.
void TestDirectWriteUsesDeviceIntrfOnce()
{
	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, &intrf, 0, nullptr, 0);

	ResetCapture(8);
	int n = SysLogPrintf(&log, "0123456789ABCDEFGHIJ");

	CHECK(n == 8);
	CHECK(s_TxCalls == 1);
	CHECK(s_CapturedLen == 8);
	CHECK(std::memcmp(s_Captured, "01234567", 8) == 0);
}

// A configured CFifo stores complete records without requiring an output sink.
void TestBufferedBeforeSink()
{
	alignas(4) uint8_t fifoMem[CFIFO_TOTAL_MEMSIZE(4, SYSLOG_LINE_MAX)];
	hCFifo_t hFifo = CFifoInit(fifoMem, sizeof(fifoMem), SYSLOG_LINE_MAX, false);
	CHECK(hFifo != nullptr);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, nullptr, 0, nullptr, 0);
	SysLogSetBuffer(&log, hFifo);

	ResetCapture();
	CHECK(SysLogPrintf(&log, "first\n") == 6);
	CHECK(SysLogPrintf(&log, "second\n") == 7);
	CHECK(CFifoUsed(hFifo) == 2);
	CHECK(s_CapturedLen == 0);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLogSetSink(&log, &intrf, 0);

	CHECK(SysLogFlush(&log) == 6);
	CHECK(CFifoUsed(hFifo) == 1);
	CHECK(SysLogFlush(&log) == 7);
	CHECK(CFifoUsed(hFifo) == 0);
	CHECK(s_TxCalls == 2);
	CHECK(s_CapturedLen == 13);
	CHECK(std::memcmp(s_Captured, "first\nsecond\n", 13) == 0);
}

// Buffered printf formats directly into the FIFO block, so the configured
// record size can be larger than the direct-output formatting scratch.
void TestBufferedRecordUsesFifoBlockSize()
{
	constexpr uint32_t kRecordSize = 200U;
	alignas(4) uint8_t fifoMem[CFIFO_TOTAL_MEMSIZE(2, kRecordSize)];
	hCFifo_t hFifo = CFifoInit(fifoMem, sizeof(fifoMem), kRecordSize, false);
	CHECK(hFifo != nullptr);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, nullptr, 0, nullptr, 0);
	SysLogSetBuffer(&log, hFifo);

	char text[181];
	std::memset(text, 'x', sizeof(text) - 1U);
	text[sizeof(text) - 1U] = '\0';
	CHECK(SysLogPrintf(&log, "%s", text) == 180);
	CHECK(CFifoUsed(hFifo) == 1);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	ResetCapture();
	SysLogSetSink(&log, &intrf, 0);
	CHECK(SysLogFlush(&log) == 180);
	CHECK(s_CapturedLen == 180);
	CHECK(std::memcmp(s_Captured, text, 180) == 0);
}

// A zero result means the sink did not accept the record. The reader leaves
// it queued so a later flush can send the same complete record.
void TestBufferedFlushRetriesNotReadyRecord()
{
	alignas(4) uint8_t fifoMem[CFIFO_TOTAL_MEMSIZE(2, SYSLOG_LINE_MAX)];
	hCFifo_t hFifo = CFifoInit(fifoMem, sizeof(fifoMem), SYSLOG_LINE_MAX, false);
	CHECK(hFifo != nullptr);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, nullptr, 0, nullptr, 0);
	SysLogSetBuffer(&log, hFifo);
	CHECK(SysLogPrintf(&log, "record\n") == 7);
	CHECK(CFifoUsed(hFifo) == 1);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLogSetSink(&log, &intrf, 0);

	ResetCapture(0);
	CHECK(SysLogFlush(&log) == 0);
	CHECK(CFifoUsed(hFifo) == 1);
	CHECK(s_CapturedLen == 0);

	ResetCapture();
	CHECK(SysLogFlush(&log) == 7);
	CHECK(CFifoUsed(hFifo) == 0);
	CHECK(s_CapturedLen == 7);
	CHECK(std::memcmp(s_Captured, "record\n", 7) == 0);
}

// Partial acceptance is transport-specific. SysLog reports it and keeps the
// record queued, but does not retry the remainder or the whole record.
void TestBufferedFlushLeavesPartialRecordQueued()
{
	alignas(4) uint8_t fifoMem[CFIFO_TOTAL_MEMSIZE(2, SYSLOG_LINE_MAX)];
	hCFifo_t hFifo = CFifoInit(fifoMem, sizeof(fifoMem), SYSLOG_LINE_MAX, false);
	CHECK(hFifo != nullptr);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, nullptr, 0, nullptr, 0);
	SysLogSetBuffer(&log, hFifo);
	CHECK(SysLogPrintf(&log, "record\n") == 7);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLogSetSink(&log, &intrf, 0);
	ResetCapture(3);

	CHECK(SysLogFlush(&log) == 3);
	CHECK(CFifoUsed(hFifo) == 1);
	CHECK(s_CapturedLen == 3);
	CHECK(std::memcmp(s_Captured, "rec", 3) == 0);
}

// A negative result is the DeviceIntrf asynchronous path. It leaves the
// interface transaction open until DeviceIntrfTxComplete and is not a
// synchronous retry condition for SysLog.
void TestBufferedFlushLeavesAsyncRecordQueued()
{
	alignas(4) uint8_t fifoMem[CFIFO_TOTAL_MEMSIZE(2, SYSLOG_LINE_MAX)];
	hCFifo_t hFifo = CFifoInit(fifoMem, sizeof(fifoMem), SYSLOG_LINE_MAX, false);
	CHECK(hFifo != nullptr);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, nullptr, 0, nullptr, 0);
	SysLogSetBuffer(&log, hFifo);
	CHECK(SysLogPrintf(&log, "record\n") == 7);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLogSetSink(&log, &intrf, 0);
	ResetCapture(-1);

	CHECK(SysLogFlush(&log) == -1);
	CHECK(CFifoUsed(hFifo) == 1);
	CHECK(s_CapturedLen == 0);
	DeviceIntrfTxComplete(&intrf);
}

// Empty records are discarded so a failed format cannot block later records.
void TestBufferedFlushDropsEmptyRecord()
{
	alignas(4) uint8_t fifoMem[CFIFO_TOTAL_MEMSIZE(2, SYSLOG_LINE_MAX)];
	hCFifo_t hFifo = CFifoInit(fifoMem, sizeof(fifoMem), SYSLOG_LINE_MAX, false);
	CHECK(hFifo != nullptr);

	uint8_t *pBlock = CFifoPut(hFifo);
	CHECK(pBlock != nullptr);
	pBlock[0] = 0U;
	CHECK(CFifoUsed(hFifo) == 1);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, &intrf, 0, nullptr, 0);
	SysLogSetBuffer(&log, hFifo);

	ResetCapture();
	CHECK(SysLogFlush(&log) == 0);
	CHECK(CFifoUsed(hFifo) == 0);
	CHECK(s_TxCalls == 0);
}

// Non-blocking CFifo policy drops the oldest complete log record when full.
void TestBufferedDropIsRecordBased()
{
	alignas(4) uint8_t fifoMem[CFIFO_TOTAL_MEMSIZE(2, SYSLOG_LINE_MAX)];
	hCFifo_t hFifo = CFifoInit(fifoMem, sizeof(fifoMem), SYSLOG_LINE_MAX, false);
	CHECK(hFifo != nullptr);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, nullptr, 0, nullptr, 0);
	SysLogSetBuffer(&log, hFifo);

	CHECK(SysLogPrintf(&log, "one\n") == 4);
	CHECK(SysLogPrintf(&log, "two\n") == 4);
	CHECK(SysLogPrintf(&log, "three\n") == 6);
	CHECK(hFifo->DropCnt == 1U);
	CHECK(CFifoUsed(hFifo) == 2);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	ResetCapture();
	SysLogSetSink(&log, &intrf, 0);

	CHECK(FlushAll(&log, hFifo) == 10);
	CHECK(s_TxCalls == 2);
	CHECK(s_CapturedLen == 10);
	CHECK(std::memcmp(s_Captured, "two\nthree\n", 10) == 0);
}

// SysLogStatus formatting is unchanged for direct output.
void TestStatusFormatting()
{
	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, &intrf, 0, nullptr, 0);

	SysStatus_t st = SYSSTATUS_TYPE_ERR | (0x0ABu << 16) | 0x1234u;
	ResetCapture();
	SysLogStatus(&log, st, nullptr);

	const char *want = "E:0AB:1234\r\n";
	CHECK(s_CapturedLen == (int)std::strlen(want));
	CHECK(std::memcmp(s_Captured, want, std::strlen(want)) == 0);

	ResetCapture();
	SysLogStatus(&log, st, "oops");
	const char *want2 = "E:0AB:1234 oops\r\n";
	CHECK(s_CapturedLen == (int)std::strlen(want2));
	CHECK(std::memcmp(s_Captured, want2, std::strlen(want2)) == 0);
}

// Records below the configured MinType are dropped before reaching either
// the FIFO or the output interface.
void TestStatusFilter()
{
	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, &intrf, 0, nullptr, SYSSTATUS_TYPE_ERR);

	ResetCapture();
	SysLogStatus(&log, SYSSTATUS_TYPE_WRN | 0x0001u, nullptr);
	CHECK(s_CapturedLen == 0);

	SysLogStatus(&log, SYSSTATUS_TYPE_ERR | 0x0002u, nullptr);
	CHECK(s_CapturedLen > 0);
}

} // namespace

int main()
{
	TestDirectWriteUsesDeviceIntrfOnce();
	TestBufferedBeforeSink();
	TestBufferedRecordUsesFifoBlockSize();
	TestBufferedFlushRetriesNotReadyRecord();
	TestBufferedFlushLeavesPartialRecordQueued();
	TestBufferedFlushLeavesAsyncRecordQueued();
	TestBufferedFlushDropsEmptyRecord();
	TestBufferedDropIsRecordBased();
	TestStatusFormatting();
	TestStatusFilter();

	if (s_Failures != 0)
	{
		std::printf("SysLog host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SysLog host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}