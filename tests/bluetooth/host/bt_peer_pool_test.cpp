#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_peer.h"

namespace {

int s_Failures = 0;
int s_Checks = 0;
int s_CccdClearCount = 0;
uint16_t s_LastClearedHdl = BT_CONN_HDL_INVALID;
int s_ServiceDisconnectCount = 0;
int s_StateChangeCount = 0;
bool s_LastConnectedState = false;
int s_RejectedCount = 0;
uint16_t s_LastRejectedHdl = BT_CONN_HDL_INVALID;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

} // namespace

BtGattSrvc_t *BtGattGetSrvcList(void)
{
	return nullptr;
}

void BtGattCccdClear(uint16_t ConnHdl)
{
	++s_CccdClearCount;
	s_LastClearedHdl = ConnHdl;
}

void BtGattSrvcDisconnected(BtGattSrvc_t *)
{
	++s_ServiceDisconnectCount;
}

void BtPeerConnectionStateChanged(bool Connected)
{
	++s_StateChangeCount;
	s_LastConnectedState = Connected;
}

void BtPeerConnectionRejected(uint16_t ConnHdl)
{
	++s_RejectedCount;
	s_LastRejectedHdl = ConnHdl;
}

namespace {

void TestMisalignedCallerPool()
{
	constexpr size_t kPeerCount = 3;
	alignas(BtDevice_t)
	std::array<uint8_t, BT_PEER_POOL_MEMSIZE(kPeerCount) + 1U> raw = {};
	uint8_t *pool = raw.data() + 1U;

	CHECK(BtPeerInit(pool, BT_PEER_POOL_MEMSIZE(kPeerCount)));
	CHECK(BtPeerCount() == kPeerCount);

	for (uint16_t i = 0; i < kPeerCount; ++i)
	{
		BtDevice_t *slot = BtPeerSlot(i);
		CHECK(slot != nullptr);
		if (slot != nullptr)
		{
			CHECK(reinterpret_cast<uintptr_t>(slot) % alignof(BtDevice_t) == 0);
			CHECK(slot->Conn.Hdl == BT_CONN_HDL_INVALID);
		}
	}

	CHECK(BtPeerSlot(kPeerCount) == nullptr);
}

void TestAllocationAndLongWriteSlices()
{
	// Establish this test's own pool. The previous test initialized the peer
	// manager against a buffer on its stack, which is out of scope now; the
	// long-write and alloc calls below must run against live memory.
	constexpr size_t kPeerCount = 3;
	alignas(BtDevice_t) std::array<uint8_t, BT_PEER_POOL_MEMSIZE(kPeerCount)> pool = {};
	CHECK(BtPeerInit(pool.data(), pool.size()));
	CHECK(BtPeerCount() == kPeerCount);

	std::array<uint8_t, 96> longWrite = {};
	BtPeerLongWrInit(longWrite.data(), longWrite.size());

	BtDevice_t *first = BtPeerAlloc(0x10);
	BtDevice_t *second = BtPeerAlloc(0x20);
	BtDevice_t *third = BtPeerAlloc(0x30);

	CHECK(first != nullptr);
	CHECK(second != nullptr);
	CHECK(third != nullptr);
	CHECK(BtPeerAlloc(0x40) == nullptr);
	CHECK(BtPeerAlloc(0x20) == second);

	if (first != nullptr && second != nullptr && third != nullptr)
	{
		CHECK(first->Conn.pLongWrBuff == longWrite.data());
		CHECK(second->Conn.pLongWrBuff == longWrite.data() + 32);
		CHECK(third->Conn.pLongWrBuff == longWrite.data() + 64);
		CHECK(first->Conn.LongWrBuffSize == 32);
		CHECK(second->Conn.LongWrBuffSize == 32);
		CHECK(third->Conn.LongWrBuffSize == 32);
	}

	uint16_t handles[3] = {};
	CHECK(BtPeerGetConnectedHandles(handles, 3) == 3);
	CHECK(handles[0] == 0x10);
	CHECK(handles[1] == 0x20);
	CHECK(handles[2] == 0x30);

	s_CccdClearCount = 0;
	s_LastClearedHdl = BT_CONN_HDL_INVALID;
	BtPeerFree(second);
	CHECK(s_CccdClearCount == 1);
	CHECK(s_LastClearedHdl == 0x20);
	CHECK(BtPeerFindByHdl(0x20) == nullptr);
	CHECK(BtPeerIsConnected());

	BtDevice_t *replacement = BtPeerAlloc(0x40);
	CHECK(replacement == second);
	if (replacement != nullptr)
	{
		CHECK(replacement->Conn.pLongWrBuff == longWrite.data() + 32);
		CHECK(replacement->Conn.LongWrBuffSize == 32);
	}

	BtPeerFree(first);
	BtPeerFree(third);
	BtPeerFree(replacement);
	CHECK(!BtPeerIsConnected());
}

void TestTruncatedHandleScan()
{
	constexpr uint16_t kPeerCount = 10;
	alignas(BtDevice_t)
	std::array<uint8_t, BT_PEER_POOL_MEMSIZE(kPeerCount)> pool = {};
	CHECK(BtPeerInit(pool.data(), pool.size()));

	for (uint16_t i = 0; i < kPeerCount; i++)
	{
		CHECK(BtPeerAlloc((uint16_t)(0x100U + i)) != nullptr);
	}

	// A truncated request returns the first MaxCount handles in pool order,
	// the same set on every call.
	uint16_t first[8] = {};
	uint16_t second[8] = {};
	CHECK(BtPeerGetConnectedHandles(first, 8) == 8);
	CHECK(BtPeerGetConnectedHandles(second, 8) == 8);
	for (uint16_t i = 0; i < 8; i++)
	{
		CHECK(first[i] == (uint16_t)(0x100U + i));
		CHECK(second[i] == first[i]);
	}

	// A request that can hold every active link returns them all in order.
	uint16_t all[kPeerCount] = {};
	CHECK(BtPeerGetConnectedHandles(all, kPeerCount) == kPeerCount);
	for (uint16_t i = 0; i < kPeerCount; i++)
	{
		CHECK(all[i] == (uint16_t)(0x100U + i));
	}
}

// A pool sized above the hard ceiling is capped at BT_DEV_CONN_MAX, and no
// more than that many links can be allocated.
void TestConnMaxCeiling()
{
	constexpr uint16_t kOversize = BT_DEV_CONN_MAX + 5;
	alignas(BtDevice_t)
	std::array<uint8_t, BT_PEER_POOL_MEMSIZE(kOversize)> pool = {};
	CHECK(BtPeerInit(pool.data(), pool.size()));

	// Only BT_DEV_CONN_MAX slots are usable even though the buffer holds more.
	uint16_t i = 0;
	for (; i < BT_DEV_CONN_MAX; i++)
	{
		CHECK(BtPeerAlloc((uint16_t)(0x200U + i)) != nullptr);
	}
	// The next allocation is refused: the pool is capped at the ceiling.
	CHECK(BtPeerAlloc((uint16_t)(0x200U + i)) == nullptr);
}

void TestConnectionLifecycleHooks()
{
	alignas(BtDevice_t) uint8_t pool[BT_PEER_POOL_MEMSIZE(1)] = {};
	CHECK(BtPeerInit(pool, sizeof(pool)));

	s_StateChangeCount = 0;
	s_LastConnectedState = false;
	s_RejectedCount = 0;
	s_LastRejectedHdl = BT_CONN_HDL_INVALID;

	const uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };
	BtDevice_t *peer = BtPeerConnected(0x41, BT_CONN_ROLE_PERIPHERAL,
		0, addr, 0, nullptr);
	CHECK(peer != nullptr);
	CHECK(s_StateChangeCount == 1);
	CHECK(s_LastConnectedState);
	CHECK(s_RejectedCount == 0);

	CHECK(BtPeerConnected(0x42, BT_CONN_ROLE_PERIPHERAL,
		0, addr, 0, nullptr) == nullptr);
	CHECK(s_RejectedCount == 1);
	CHECK(s_LastRejectedHdl == 0x42);
	CHECK(s_StateChangeCount == 1);

	BtPeerFree(peer);
	CHECK(s_StateChangeCount == 2);
	CHECK(!s_LastConnectedState);
}

void TestConnectedFieldsAndDefaultPool()
{
	CHECK(BtPeerInit(nullptr, 0));
	CHECK(BtPeerCount() == BT_DEV_CONN_MAX);

	const uint8_t peerAddr[6] = { 1, 2, 3, 4, 5, 6 };
	const uint8_t ownAddr[6] = { 6, 5, 4, 3, 2, 1 };
	BtDevice_t *peer = BtPeerConnected(0x55, 1, 2, peerAddr, 3, ownAddr);

	CHECK(peer != nullptr);
	if (peer != nullptr)
	{
		CHECK(peer->Conn.Hdl == 0x55);
		CHECK(peer->Conn.Role == 1);
		CHECK(peer->Conn.PeerAddrType == 2);
		CHECK(peer->Conn.OwnAddrType == 3);
		CHECK(std::memcmp(peer->Conn.PeerAddr, peerAddr, sizeof(peerAddr)) == 0);
		CHECK(std::memcmp(peer->Conn.OwnAddr, ownAddr, sizeof(ownAddr)) == 0);
	}

	BtPeerFree(peer);
}

// BtPeerRole reports this device's LL role on a link in the HCI encoding, and
// UNKNOWN for a missing record or an unconverted value (for example a config
// bitmask stored by a port that has not been fixed), so role gates never
// misread a foreign encoding as a role.
void TestPeerRole()
{
	CHECK(BtPeerInit(nullptr, 0));

	const uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };

	BtDevice_t *periph = BtPeerConnected(0x10, BT_CONN_ROLE_PERIPHERAL, 0, addr, 0, nullptr);
	CHECK(periph != nullptr);
	CHECK(BtPeerRole(0x10) == BT_CONN_ROLE_PERIPHERAL);

	BtDevice_t *central = BtPeerConnected(0x11, BT_CONN_ROLE_CENTRAL, 0, addr, 0, nullptr);
	CHECK(central != nullptr);
	CHECK(BtPeerRole(0x11) == BT_CONN_ROLE_CENTRAL);

	// No record for this handle.
	CHECK(BtPeerRole(0x77) == BT_CONN_ROLE_UNKNOWN);

	// A slot that has been allocated but whose role has not been filled in
	// reports UNKNOWN, not CENTRAL. BT_CONN_ROLE_CENTRAL is 0, so a zeroed
	// record used to read as a central link, and the SMP and L2CAP role gates
	// refuse a PDU on that reading: a peripheral would answer a Pairing
	// Request with Command Not Supported.
	BtDevice_t *raw = BtPeerAlloc(0x13);
	CHECK(raw != nullptr);
	CHECK(BtPeerRole(0x13) == BT_CONN_ROLE_UNKNOWN);
	BtPeerFree(raw);

	// A value outside the HCI encoding (a BT_GAP_ROLE_* bitmask, 0x0C for
	// PERIPHERAL|CENTRAL) reports UNKNOWN rather than a role.
	BtDevice_t *bogus = BtPeerConnected(0x12, 0x0C, 0, addr, 0, nullptr);
	CHECK(bogus != nullptr);
	CHECK(BtPeerRole(0x12) == BT_CONN_ROLE_UNKNOWN);

	BtPeerFree(periph);
	BtPeerFree(central);
	BtPeerFree(bogus);
}

void TestInvalidPool()
{
	uint8_t tiny[1] = {};
	CHECK(!BtPeerInit(tiny, sizeof(tiny)));
}

} // namespace

int main()
{
	TestMisalignedCallerPool();
	TestAllocationAndLongWriteSlices();
	TestTruncatedHandleScan();
	TestConnMaxCeiling();
	TestConnectionLifecycleHooks();
	TestConnectedFieldsAndDefaultPool();
	TestPeerRole();
	TestInvalidPool();

	if (s_Failures != 0)
	{
		std::printf("Peer pool host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("Peer pool host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
