// Coverage for the SysLog record store and its drain to a DeviceIntrf.
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

// Line length used by the tests that do not care about it.
constexpr uint32_t kLineLen = 128U;

SysLogCfg_t MakeCfg(uint8_t *pMem, uint32_t MemSize, uint32_t RecordLen,
					bool bBlocking)
{
	SysLogCfg_t cfg{};
	cfg.pMem = pMem;
	cfg.MemSize = MemSize;
	cfg.RecordLen = RecordLen;
	cfg.bBlocking = bBlocking;
	return cfg;
}

int FlushAll(SysLog_t *pLog)
{
	int total = 0;
	int n;
	while ((n = SysLogFlush(pLog)) > 0)
	{
		total += n;
	}
	return total;
}

// The store keeps complete records without requiring an output sink.
void TestBufferedBeforeSink()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(4, kLineLen)];

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, nullptr, 0, nullptr, 0));

	ResetCapture();
	CHECK(SysLogPrintf(&log, "first\n") == 6);
	CHECK(SysLogPrintf(&log, "second\n") == 7);
	CHECK(s_CapturedLen == 0);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLogSetSink(&log, &intrf, 0);

	CHECK(SysLogFlush(&log) == 6);
	CHECK(SysLogFlush(&log) == 7);
	CHECK(SysLogFlush(&log) == 0);
	CHECK(s_TxCalls == 2);
	CHECK(s_CapturedLen == 13);
	CHECK(std::memcmp(s_Captured, "first\nsecond\n", 13) == 0);
}

// One record is one RecordLen block. Text longer than that is truncated
// to the block, leaving room for the terminating NUL.
void TestRecordTruncatedToLineMax()
{
	constexpr int kKeep = (int)kLineLen - 1;
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, nullptr, 0, nullptr, 0));

	char text[kLineLen + 51];
	std::memset(text, 'x', sizeof(text) - 1U);
	text[sizeof(text) - 1U] = '\0';
	CHECK(SysLogPrintf(&log, "%s", text) == kKeep);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	ResetCapture();
	SysLogSetSink(&log, &intrf, 0);
	CHECK(SysLogFlush(&log) == kKeep);
	CHECK(s_CapturedLen == kKeep);
	CHECK(std::memcmp(s_Captured, text, (size_t)kKeep) == 0);
}

// A zero result means the sink did not accept the record. The reader leaves
// it queued so a later flush can send the same complete record.
void TestBufferedFlushRetriesNotReadyRecord()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, nullptr, 0, nullptr, 0));
	CHECK(SysLogPrintf(&log, "record\n") == 7);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLogSetSink(&log, &intrf, 0);

	ResetCapture(0);
	CHECK(SysLogFlush(&log) == 0);
	CHECK(s_CapturedLen == 0);

	ResetCapture();
	CHECK(SysLogFlush(&log) == 7);
	CHECK(SysLogFlush(&log) == 0);
	CHECK(s_CapturedLen == 7);
	CHECK(std::memcmp(s_Captured, "record\n", 7) == 0);
}

// Partial acceptance is transport-specific. SysLog reports it and keeps the
// record queued, but does not retry the remainder or the whole record.
void TestBufferedFlushLeavesPartialRecordQueued()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, nullptr, 0, nullptr, 0));
	CHECK(SysLogPrintf(&log, "record\n") == 7);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLogSetSink(&log, &intrf, 0);
	ResetCapture(3);

	CHECK(SysLogFlush(&log) == 3);
	CHECK(s_CapturedLen == 3);
	CHECK(std::memcmp(s_Captured, "rec", 3) == 0);

	// Still queued, so a sink that can take it whole gets the whole record.
	s_ChunkLimit = 512;
	CHECK(SysLogFlush(&log) == 7);
}

// A negative result is the DeviceIntrf asynchronous path. It leaves the
// interface transaction open until DeviceIntrfTxComplete and is not a
// synchronous retry condition for SysLog.
void TestBufferedFlushLeavesAsyncRecordQueued()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, nullptr, 0, nullptr, 0));
	CHECK(SysLogPrintf(&log, "record\n") == 7);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLogSetSink(&log, &intrf, 0);
	ResetCapture(-1);

	CHECK(SysLogFlush(&log) == -1);
	CHECK(s_CapturedLen == 0);
	DeviceIntrfTxComplete(&intrf);

	s_ChunkLimit = 512;
	CHECK(SysLogFlush(&log) == 7);
}

// Empty records are discarded so a failed format cannot block later records.
void TestBufferedFlushDropsEmptyRecord()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, &intrf, 0, nullptr, 0));

	// A format that produces no characters still occupies a block.
	CHECK(SysLogPrintf(&log, "%s", "") == 0);

	ResetCapture();
	CHECK(SysLogFlush(&log) == 0);
	CHECK(s_TxCalls == 0);

	// The empty record was discarded, so the next one is delivered.
	CHECK(SysLogPrintf(&log, "after\n") == 6);
	CHECK(SysLogFlush(&log) == 6);
	CHECK(s_CapturedLen == 6);
	CHECK(std::memcmp(s_Captured, "after\n", 6) == 0);
}

// The non-blocking store policy drops the oldest complete record when full.
void TestBufferedDropIsRecordBased()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, nullptr, 0, nullptr, 0));

	CHECK(SysLogPrintf(&log, "one\n") == 4);
	CHECK(SysLogPrintf(&log, "two\n") == 4);
	CHECK(SysLogPrintf(&log, "three\n") == 6);

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);
	ResetCapture();
	SysLogSetSink(&log, &intrf, 0);

	CHECK(FlushAll(&log) == 10);
	CHECK(s_TxCalls == 2);
	CHECK(s_CapturedLen == 10);
	CHECK(std::memcmp(s_Captured, "two\nthree\n", 10) == 0);
}

// SysLogStatus record format, read back through a flush.
void TestStatusFormatting()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, &intrf, 0, nullptr, 0));

	SysStatus_t st = SYSSTATUS_TYPE_ERR | (0x0ABu << 16) | 0x1234u;
	ResetCapture();
	SysLogStatus(&log, st, nullptr);
	FlushAll(&log);

	const char *want = "E:0AB:1234\r\n";
	CHECK(s_CapturedLen == (int)std::strlen(want));
	CHECK(std::memcmp(s_Captured, want, std::strlen(want)) == 0);

	ResetCapture();
	SysLogStatus(&log, st, "oops");
	FlushAll(&log);
	const char *want2 = "E:0AB:1234 oops\r\n";
	CHECK(s_CapturedLen == (int)std::strlen(want2));
	CHECK(std::memcmp(s_Captured, want2, std::strlen(want2)) == 0);
}

// Records below the configured MinType never reach the store.
void TestStatusFilter()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, &intrf, 0, nullptr, SYSSTATUS_TYPE_ERR));

	ResetCapture();
	CHECK(SysLogStatus(&log, SYSSTATUS_TYPE_WRN | 0x0001u, nullptr) == 0);
	CHECK(FlushAll(&log) == 0);
	CHECK(s_CapturedLen == 0);

	CHECK(SysLogStatus(&log, SYSSTATUS_TYPE_ERR | 0x0002u, nullptr) > 0);
	CHECK(FlushAll(&log) > 0);
	CHECK(s_CapturedLen > 0);
}

// Memory too small for one record leaves the instance dormant, and a dormant
// instance takes no records.
void TestInitRefusesShortMemory()
{
	alignas(4) uint8_t fifoMem[8];

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, true);
	CHECK(SysLogInit(&log, &cfg, &intrf, 0, nullptr, 0) == false);

	ResetCapture();
	CHECK(SysLogPrintf(&log, "dropped\n") == 0);
	CHECK(SysLogFlush(&log) == 0);
	CHECK(s_CapturedLen == 0);
}

// No memory at all is refused the same way.
void TestInitRefusesNullMemory()
{
	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(nullptr, 0, kLineLen, true);
	CHECK(SysLogInit(&log, &cfg, nullptr, 0, nullptr, 0) == false);
	CHECK(SysLogInit(&log, nullptr, nullptr, 0, nullptr, 0) == false);
	CHECK(SysLogPrintf(&log, "dropped\n") == 0);
	CHECK(SysLogFlush(&log) == 0);
}

// A failed re-init leaves no live store behind.
void TestFailedReinitLeavesDormant()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];
	alignas(4) uint8_t shortMem[8];

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLineLen, false);
	CHECK(SysLogInit(&log, &cfg, nullptr, 0, nullptr, 0));
	CHECK(SysLogPrintf(&log, "one\n") == 4);

	SysLogCfg_t bad = MakeCfg(shortMem, sizeof(shortMem), kLineLen, false);
	CHECK(SysLogInit(&log, &bad, nullptr, 0, nullptr, 0) == false);
	CHECK(SysLogPrintf(&log, "two\n") == 0);
	CHECK(SysLogFlush(&log) == 0);
}

// A shorter RecordLen gives more records in the same memory, and truncates
// each one to that length.
void TestShortRecordLen()
{
	constexpr uint32_t kLen = 16U;
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(3, kLen)];

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLen, true);
	CHECK(SysLogInit(&log, &cfg, &intrf, 0, nullptr, 0));

	// Three records fit where a 128 byte line would have given none.
	CHECK(SysLogPrintf(&log, "one\n") == 4);
	CHECK(SysLogPrintf(&log, "two\n") == 4);
	CHECK(SysLogPrintf(&log, "three\n") == 6);

	ResetCapture();
	CHECK(FlushAll(&log) == 14);
	CHECK(s_CapturedLen == 14);
	CHECK(std::memcmp(s_Captured, "one\ntwo\nthree\n", 14) == 0);

	// Anything past the record length is cut, NUL included.
	char text[64];
	std::memset(text, 'x', sizeof(text) - 1U);
	text[sizeof(text) - 1U] = '\0';
	CHECK(SysLogPrintf(&log, "%s", text) == (int)kLen - 1);

	ResetCapture();
	CHECK(SysLogFlush(&log) == (int)kLen - 1);
	CHECK(s_CapturedLen == (int)kLen - 1);
}

// A status record is truncated to the record length the same way.
void TestShortRecordLenTruncatesStatus()
{
	constexpr uint32_t kLen = 8U;
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLen)];

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLen, true);
	CHECK(SysLogInit(&log, &cfg, &intrf, 0, nullptr, 0));

	SysStatus_t st = SYSSTATUS_TYPE_ERR | (0x0ABu << 16) | 0x1234u;
	ResetCapture();
	CHECK(SysLogStatus(&log, st, "detail") == (int)kLen - 1);
	CHECK(SysLogFlush(&log) == (int)kLen - 1);
	CHECK(s_CapturedLen == (int)kLen - 1);
	CHECK(std::memcmp(s_Captured, "E:0AB:1", (size_t)kLen - 1U) == 0);
}

// There is no ceiling on RecordLen. A long record is formatted in place and
// both a printf record and a status record use the whole length.
void TestLongRecordLen()
{
	constexpr uint32_t kLen = 400U;
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLen)];

	DevIntrf_t intrf{};
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t cfg = MakeCfg(fifoMem, sizeof(fifoMem), kLen, true);
	CHECK(SysLogInit(&log, &cfg, &intrf, 0, nullptr, 0));

	char text[350];
	std::memset(text, 'x', sizeof(text) - 1U);
	text[sizeof(text) - 1U] = '\0';
	CHECK(SysLogPrintf(&log, "%s", text) == 349);

	ResetCapture();
	CHECK(SysLogFlush(&log) == 349);
	CHECK(s_CapturedLen == 349);

	// A status record reaches past a 128 byte line too.
	SysStatus_t st = SYSSTATUS_TYPE_ERR | (0x0ABu << 16) | 0x1234u;
	ResetCapture();
	constexpr int kStatusLen = 10 + 1 + 349 + 2;
	CHECK(SysLogStatus(&log, st, text) == kStatusLen);
	CHECK(SysLogFlush(&log) == kStatusLen);
	CHECK(s_CapturedLen == kStatusLen);
	CHECK(std::memcmp(s_Captured, "E:0AB:1234 x", 12) == 0);
}

// One byte leaves no room for the terminating NUL, and 0 is not a default.
void TestRecordLenTooShortRefused()
{
	alignas(4) uint8_t fifoMem[SYSLOG_MEMSIZE(2, kLineLen)];

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogCfg_t one = MakeCfg(fifoMem, sizeof(fifoMem), 1U, true);
	CHECK(SysLogInit(&log, &one, nullptr, 0, nullptr, 0) == false);
	CHECK(SysLogPrintf(&log, "dropped\n") == 0);

	SysLogCfg_t zero = MakeCfg(fifoMem, sizeof(fifoMem), 0U, true);
	CHECK(SysLogInit(&log, &zero, nullptr, 0, nullptr, 0) == false);
}

} // namespace

int main()
{
	TestBufferedBeforeSink();
	TestRecordTruncatedToLineMax();
	TestBufferedFlushRetriesNotReadyRecord();
	TestBufferedFlushLeavesPartialRecordQueued();
	TestBufferedFlushLeavesAsyncRecordQueued();
	TestBufferedFlushDropsEmptyRecord();
	TestBufferedDropIsRecordBased();
	TestStatusFormatting();
	TestStatusFilter();
	TestInitRefusesShortMemory();
	TestInitRefusesNullMemory();
	TestFailedReinitLeavesDormant();
	TestShortRecordLen();
	TestShortRecordLenTruncatesStatus();
	TestLongRecordLen();
	TestRecordLenTooShortRefused();

	if (s_Failures != 0)
	{
		std::printf("SysLog host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SysLog host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}