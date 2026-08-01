// Coverage for the SysLog transmit path. DeviceIntrfTx returns the number of
// bytes the sink accepted and only retries internally when it accepted none, so
// a partial write (sink FIFO full mid-line) leaves the tail unsent unless the
// caller resends the remainder. A mock interface that accepts a fixed chunk per
// call proves the whole line is delivered across several transmits.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

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
int s_ChunkLimit = 8;		// bytes the mock sink accepts per TxData call

bool MockStartTx(DevIntrf_t * const, uint32_t)
{
	return true;
}

void MockStopTx(DevIntrf_t * const)
{
}

int MockTxData(DevIntrf_t * const, const uint8_t *pData, int DataLen)
{
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
	std::memset(pIntrf, 0, sizeof(*pIntrf));
	pIntrf->StartTx  = MockStartTx;
	pIntrf->StopTx   = MockStopTx;
	pIntrf->TxData   = MockTxData;
	pIntrf->MaxRetry = 3;
}

// A line longer than one sink chunk must be delivered whole.
void TestPartialWriteResendsRemainder()
{
	DevIntrf_t intrf;
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, &intrf, 0, nullptr, 0);

	const char *msg = "0123456789ABCDEFGHIJ";	// 20 chars, > 8-byte chunk
	const int msgLen = 20;

	s_CapturedLen = 0;
	s_ChunkLimit = 8;
	SysLogPrintf(&log, "%s", msg);

	CHECK(s_CapturedLen == msgLen);
	CHECK(std::memcmp(s_Captured, msg, msgLen) == 0);
}

// A line that fits in a single chunk still goes out intact.
void TestSingleChunkLine()
{
	DevIntrf_t intrf;
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, &intrf, 0, nullptr, 0);

	s_CapturedLen = 0;
	s_ChunkLimit = 64;
	SysLogPrintf(&log, "hi");

	CHECK(s_CapturedLen == 2);
	CHECK(std::memcmp(s_Captured, "hi", 2) == 0);
}

// Tiny chunks (1 byte per call) still deliver the whole line.
void TestOneBytePerCall()
{
	DevIntrf_t intrf;
	MakeIntrf(&intrf);

	SysLog_t log;
	std::memset(&log, 0, sizeof(log));
	SysLogInit(&log, &intrf, 0, nullptr, 0);

	const char *msg = "abcdefghij";		// 10 chars
	s_CapturedLen = 0;
	s_ChunkLimit = 1;
	SysLogPrintf(&log, "%s", msg);

	CHECK(s_CapturedLen == 10);
	CHECK(std::memcmp(s_Captured, msg, 10) == 0);
}

} // namespace

int main()
{
	TestPartialWriteResendsRemainder();
	TestSingleChunkLine();
	TestOneBytePerCall();

	if (s_Failures != 0)
	{
		std::printf("SysLog Tx host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SysLog Tx host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
