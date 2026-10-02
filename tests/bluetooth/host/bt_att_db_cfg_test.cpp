#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_att.h"

// The attribute database in memory the application supplies through its own
// g_BtAttDBMemCfg. One test binary has one descriptor, so the library default
// is covered by bt_att_db_test and the application one here.

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

// Larger than the library default so that a database still held to the
// default size is told apart from one using this memory.
const size_t kAppMemSize = 8192;
const int kDataLen = 100;

alignas(BtAttDBEntry_t) uint8_t s_AppAttDBMem[kAppMemSize];

bool InAppMem(const void *p)
{
	const uint8_t *b = static_cast<const uint8_t *>(p);

	return b >= s_AppAttDBMem && b < s_AppAttDBMem + kAppMemSize;
}

// Entries are added until the database refuses one. Returns how many fit.
int Fill(void)
{
	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, { 0x2A00 } };
	int n = 0;

	for (;;)
	{
		BtAttDBEntry_t *e = BtAttDBAddEntry(&uuid, kDataLen);
		if (e == nullptr)
		{
			break;
		}
		CHECK(InAppMem(e));
		CHECK(InAppMem(e->Data + kDataLen - 1));
		std::memset(e->Data, 0xA5, kDataLen);
		n++;
		if (n > 1000)
		{
			break;
		}
	}

	return n;
}

void TestApplicationMemoryIsUsed(void)
{
	// Asking for more than the descriptor gives is held to the descriptor
	BtAttDBInit(1024U * 1024U);

	size_t entrySize = (sizeof(BtAttDBEntry_t) + kDataLen +
		alignof(BtAttDBEntry_t) - 1U) & ~(size_t)(alignof(BtAttDBEntry_t) - 1U);
	int expected = (int)((kAppMemSize - sizeof(BtAttDBEntry_t)) / entrySize);

	int n = Fill();
	CHECK(n == expected);
	// More than the library default memory could hold
	CHECK((size_t)n * entrySize > 2048U);

	// Every entry is still found after the database filled up
	for (int h = 1; h <= n; h++)
	{
		BtAttDBEntry_t *e = BtAttDBFindHandle((uint16_t)h);
		CHECK(e != nullptr && InAppMem(e));
	}
	CHECK(BtAttDBFindHandle((uint16_t)(n + 1)) == nullptr);
}

void TestSmallerSizeIsHonored(void)
{
	BtAttDBInit(1024);

	size_t entrySize = (sizeof(BtAttDBEntry_t) + kDataLen +
		alignof(BtAttDBEntry_t) - 1U) & ~(size_t)(alignof(BtAttDBEntry_t) - 1U);
	int expected = (int)((1024U - sizeof(BtAttDBEntry_t)) / entrySize);

	CHECK(Fill() == expected);
}

}	// namespace

// The application descriptor, in place of the library default
extern "C" const BtAttDBMemCfg_t g_BtAttDBMemCfg = { s_AppAttDBMem, kAppMemSize };

int main(void)
{
	TestApplicationMemoryIsUsed();
	TestSmallerSizeIsHonored();

	if (s_Failures != 0)
	{
		std::printf("ATT database application memory tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("ATT database application memory tests: PASS (%d checks)\n", s_Checks);

	return 0;
}
