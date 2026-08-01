#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_att.h"

namespace {

static_assert(sizeof(void *) > sizeof(uint32_t),
			  "ATT database host test must run with 64-bit pointers");

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

uintptr_t Address(const void *p)
{
	return reinterpret_cast<uintptr_t>(p);
}

void TestInvalidArguments()
{
	BtAttDBInit(128);

	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, 0xFFF1 };

	CHECK(BtAttDBAddEntry(nullptr, 4) == nullptr);
	CHECK(BtAttDBAddEntry(&uuid, -1) == nullptr);
	CHECK(BtAttDBAddEntry(&uuid, 0x10000) == nullptr);

	BtAttDBEntry_t *entry = BtAttDBAddEntry(&uuid, 4);
	CHECK(entry != nullptr);
	if (entry != nullptr)
	{
		CHECK(entry->Hdl == 1);
		CHECK(entry->DataLen == 4);
	}
}

void TestNativePointerLinks()
{
	BtAttDBInit(512);

	BtUuid16_t uuidA = { 0, BT_UUID_TYPE_16, 0xFFF1 };
	BtUuid16_t uuidB = { 0, BT_UUID_TYPE_16, 0xFFF2 };

	BtAttDBEntry_t *first = BtAttDBAddEntry(&uuidA, 1);
	BtAttDBEntry_t *second = BtAttDBAddEntry(&uuidB, 7);
	BtAttDBEntry_t *third = BtAttDBAddEntry(&uuidA, 19);

	CHECK(first != nullptr);
	CHECK(second != nullptr);
	CHECK(third != nullptr);
	if (first == nullptr || second == nullptr || third == nullptr)
	{
		return;
	}

	CHECK(first->Hdl == 1);
	CHECK(second->Hdl == 2);
	CHECK(third->Hdl == 3);

	CHECK(first->pPrev == nullptr);
	CHECK(first->pNext == second);
	CHECK(second->pPrev == first);
	CHECK(second->pNext == third);
	CHECK(third->pPrev == second);

	BtAttDBEntry_t *tail = third->pNext;
	CHECK(tail != nullptr);
	if (tail != nullptr)
	{
		CHECK(tail->pPrev == third);
		CHECK(tail->pNext == nullptr);
	}

	CHECK(Address(first) % alignof(BtAttDBEntry_t) == 0);
	CHECK(Address(second) % alignof(BtAttDBEntry_t) == 0);
	CHECK(Address(third) % alignof(BtAttDBEntry_t) == 0);
	CHECK(Address(first) < Address(second));
	CHECK(Address(second) < Address(third));
	CHECK(Address(third) < Address(tail));

	CHECK(Address(second) - Address(first) >= sizeof(BtAttDBEntry_t) + 1U);
	CHECK(Address(third) - Address(second) >= sizeof(BtAttDBEntry_t) + 7U);
	CHECK(Address(tail) - Address(third) >= sizeof(BtAttDBEntry_t) + 19U);

	std::memset(first->Data, 0x11, first->DataLen);
	std::memset(second->Data, 0x22, second->DataLen);
	std::memset(third->Data, 0x33, third->DataLen);

	CHECK(first->Data[0] == 0x11);
	CHECK(second->Data[0] == 0x22);
	CHECK(second->Data[6] == 0x22);
	CHECK(third->Data[0] == 0x33);
	CHECK(third->Data[18] == 0x33);

	CHECK(BtAttDBFindHandle(1) == first);
	CHECK(BtAttDBFindHandle(2) == second);
	CHECK(BtAttDBFindHandle(3) == third);
	CHECK(BtAttDBFindHandle(4) == nullptr);

	CHECK(BtAttDBFindUuid(nullptr, &uuidA) == first);
	CHECK(BtAttDBFindUuid(second, &uuidA) == third);
	CHECK(BtAttDBFindUuidRange(&uuidA, 2, 3) == third);
	CHECK(BtAttDBFindUuidRange(&uuidA, 2, 2) == nullptr);

	BtAttDBEntrySetPermission(second,
							BT_ATT_PERMISSION_READ |
							BT_ATT_PERMISSION_WRITE_ENCRYPT);
	CHECK(BtAttDBEntryGetPermission(second) ==
		  (BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE_ENCRYPT));
	CHECK(BtAttDBEntryGetPermission(nullptr) == 0);
}

void TestExhaustionDoesNotCorruptTail()
{
	BtAttDBInit(256);

	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, 0xFFF3 };
	BtAttDBEntry_t *entries[16] = {};
	int count = 0;

	while (count < static_cast<int>(sizeof(entries) / sizeof(entries[0])))
	{
		BtAttDBEntry_t *entry = BtAttDBAddEntry(&uuid, 12);
		if (entry == nullptr)
		{
			break;
		}
		entries[count++] = entry;
	}

	CHECK(count > 0);
	CHECK(count < static_cast<int>(sizeof(entries) / sizeof(entries[0])));
	CHECK(BtAttDBAddEntry(&uuid, 12) == nullptr);

	if (count > 0)
	{
		BtAttDBEntry_t *last = entries[count - 1];
		BtAttDBEntry_t *tail = last->pNext;

		CHECK(tail != nullptr);
		if (tail != nullptr)
		{
			CHECK(tail->pPrev == last);
			CHECK(tail->pNext == nullptr);
		}

		for (int i = 0; i < count; ++i)
		{
			CHECK(entries[i]->Hdl == static_cast<uint16_t>(i + 1));
			CHECK(BtAttDBFindHandle(static_cast<uint16_t>(i + 1)) == entries[i]);
		}
		CHECK(BtAttDBFindHandle(static_cast<uint16_t>(count + 1)) == nullptr);
	}
}

void TestReinitialization()
{
	BtAttDBInit(256);

	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, 0xFFF4 };
	BtAttDBEntry_t *first = BtAttDBAddEntry(&uuid, 8);
	BtAttDBEntry_t *second = BtAttDBAddEntry(&uuid, 8);

	CHECK(first != nullptr);
	CHECK(second != nullptr);
	CHECK(BtAttDBFindHandle(2) == second);

	BtAttDBInit(256);
	CHECK(BtAttDBFindHandle(1) == nullptr);
	CHECK(BtAttDBFindHandle(2) == nullptr);

	BtAttDBEntry_t *reset = BtAttDBAddEntry(&uuid, 8);
	CHECK(reset != nullptr);
	if (reset != nullptr)
	{
		CHECK(reset->Hdl == 1);
		CHECK(reset->pPrev == nullptr);
		CHECK(reset->pNext != nullptr);
	}
}

} // namespace

int main()
{
	TestInvalidArguments();
	TestNativePointerLinks();
	TestExhaustionDoesNotCorruptTail();
	TestReinitialization();

	if (s_Failures != 0)
	{
		std::printf("ATT database host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("ATT database host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
