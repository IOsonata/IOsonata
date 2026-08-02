// Adversarial application-layer multi-link tests.
//
// The public "All" operations must cover the peer pool rather than an
// unrelated local array limit. A zero-length value is also a real replacement
// and must update the readable characteristic state before it is sent.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_peer.h"

namespace {

bttest::Context s_Test("Application adversarial host tests");

constexpr size_t kLinkCount = 10;
uint16_t s_Links[kLinkCount] = {};
uint16_t s_NotifyHdl[kLinkCount] = {};
uint16_t s_IndicateHdl[kLinkCount] = {};
uint16_t s_DisconnectHdl[kLinkCount] = {};
size_t s_NotifyCount = 0;
size_t s_IndicateCount = 0;
size_t s_DisconnectCount = 0;
int s_SetValueCount = 0;
size_t s_SetValueLen = 0;

BtGattChar_t s_Char;
uint8_t s_Value[2] = { 0x12, 0x34 };

void Reset()
{
	s_NotifyCount = 0;
	s_IndicateCount = 0;
	s_DisconnectCount = 0;
	s_SetValueCount = 0;
	s_SetValueLen = 0;
	std::memset(s_NotifyHdl, 0, sizeof(s_NotifyHdl));
	std::memset(s_IndicateHdl, 0, sizeof(s_IndicateHdl));
	std::memset(s_DisconnectHdl, 0, sizeof(s_DisconnectHdl));
}

void TestAllOperationsCoverEntirePool()
{
	Reset();

	BT_CHECK(s_Test,
			BtAppNotifyAll(&s_Char, s_Value, sizeof(s_Value)) ==
			(int)kLinkCount);
	BT_CHECK(s_Test, s_NotifyCount == kLinkCount);
	for (size_t i = 0; i < kLinkCount; i++)
	{
		BT_CHECK(s_Test, s_NotifyHdl[i] == s_Links[i]);
	}

	Reset();
	BT_CHECK(s_Test,
			BtAppIndicateAll(&s_Char, s_Value, sizeof(s_Value)) ==
			(int)kLinkCount);
	BT_CHECK(s_Test, s_IndicateCount == kLinkCount);
	for (size_t i = 0; i < kLinkCount; i++)
	{
		BT_CHECK(s_Test, s_IndicateHdl[i] == s_Links[i]);
	}

	Reset();
	BT_CHECK(s_Test, BtAppDisconnectAll() == (int)kLinkCount);
	BT_CHECK(s_Test, s_DisconnectCount == kLinkCount);
	for (size_t i = 0; i < kLinkCount; i++)
	{
		BT_CHECK(s_Test, s_DisconnectHdl[i] == s_Links[i]);
	}
}

void TestZeroLengthValueReplacesStoredValue()
{
	Reset();

	BT_CHECK(s_Test,
			BtAppNotifyConn(s_Links[0], &s_Char, nullptr, 0));
	BT_CHECK(s_Test, s_SetValueCount == 1);
	BT_CHECK(s_Test, s_SetValueLen == 0);
	BT_CHECK(s_Test, s_NotifyCount == 1);
}

} // namespace

extern "C" {

// The All forms walk the peer pool rather than copying handles into a fixed
// local array, so the stub pool has to expose slots.
BtDevice_t *BtPeerSlot(uint16_t Idx)
{
	static BtDevice_t s_Slots[kLinkCount];
	static bool s_Init = false;

	if (s_Init == false)
	{
		for (size_t i = 0; i < kLinkCount; i++)
		{
			s_Slots[i].Conn.Hdl = s_Links[i];
		}
		s_Init = true;
	}

	return Idx < kLinkCount ? &s_Slots[Idx] : nullptr;
}

uint16_t BtPeerCount(void)
{
	return (uint16_t)kLinkCount;
}

size_t BtPeerGetConnectedHandles(uint16_t *pHdl, size_t MaxCount)
{
	size_t count = kLinkCount < MaxCount ? kLinkCount : MaxCount;
	if (pHdl != nullptr)
	{
		for (size_t i = 0; i < count; i++)
		{
			pHdl[i] = s_Links[i];
		}
	}
	return count;
}

BtDevice_t *BtPeerGetActive(void)
{
	return nullptr;
}

bool BtGattCharSetValue(BtGattChar_t *, void * const, size_t Len)
{
	s_SetValueCount++;
	s_SetValueLen = Len;
	return true;
}

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *,
					  void * const, size_t)
{
	if (s_NotifyCount < kLinkCount)
	{
		s_NotifyHdl[s_NotifyCount++] = ConnHdl;
	}
	return true;
}

bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *,
						void * const, size_t)
{
	if (s_IndicateCount < kLinkCount)
	{
		s_IndicateHdl[s_IndicateCount++] = ConnHdl;
	}
	return true;
}

void BtAppDisconnectConn(uint16_t ConnHdl)
{
	if (s_DisconnectCount < kLinkCount)
	{
		s_DisconnectHdl[s_DisconnectCount++] = ConnHdl;
	}
}

} // extern "C"

int main()
{
	std::memset(&s_Char, 0, sizeof(s_Char));
	for (size_t i = 0; i < kLinkCount; i++)
	{
		s_Links[i] = (uint16_t)(0x0100 + i);
	}

	s_Test.Run("all-link operations", TestAllOperationsCoverEntirePool);
	s_Test.Run("zero-length value replacement",
			   TestZeroLengthValueReplacesStoredValue);
	return s_Test.Finish();
}
