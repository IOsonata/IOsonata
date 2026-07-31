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

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

} // namespace

BtGattSrvc_t * const BtGattGetSrvcList(void)
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

void TestConnectedFieldsAndDefaultPool()
{
	CHECK(BtPeerInit(nullptr, 0));
	CHECK(BtPeerCount() == 4);

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
	TestConnectedFieldsAndDefaultPool();
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
