// Signed-write receive-counter persistence tests.
//
// The receive counter is protected by a durably reserved window. Counters in a
// window are accepted only after its exclusive upper bound has been committed.
// After reset, restore starts at that bound, skipping every counter that might
// have been accepted before power loss.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"

namespace {

bttest::Context s_Test("SMP signed-write counter tests");

constexpr uint16_t kConnHdl = 0x0042;
constexpr uint32_t kWindow = 32;
constexpr uint32_t kRefill = 8;

BtDevice_t s_Peer;
BtSmpKeys_t s_Keys;
uint8_t s_LastRecord[256];
size_t s_LastRecordLen;
int s_LastSlot;
int s_SaveCount;
bool s_AutoComplete;
bool s_SaveSucceeds;

uint32_t Crc32(const void *pData, size_t Len)
{
	const uint8_t *p = static_cast<const uint8_t *>(pData);
	uint32_t crc = 0xFFFFFFFFU;
	for (size_t i = 0; i < Len; ++i)
	{
		crc ^= p[i];
		for (int bit = 0; bit < 8; ++bit)
		{
			uint32_t mask = 0U - (crc & 1U);
			crc = (crc >> 1) ^ (0xEDB88320U & mask);
		}
	}
	return ~crc;
}

void PutLe32(uint8_t *p, uint32_t Value)
{
	p[0] = static_cast<uint8_t>(Value);
	p[1] = static_cast<uint8_t>(Value >> 8);
	p[2] = static_cast<uint8_t>(Value >> 16);
	p[3] = static_cast<uint8_t>(Value >> 24);
}

bool AllZero(const uint8_t *p, size_t Len)
{
	uint8_t nz = 0;
	for (size_t i = 0; i < Len; ++i)
	{
		nz |= p[i];
	}
	return nz == 0;
}

void ResetHarness(bool AutoComplete)
{
	BtSmpBondClearAll();
	std::memset(&s_Peer, 0, sizeof(s_Peer));
	std::memset(&s_Keys, 0, sizeof(s_Keys));
	std::memset(s_LastRecord, 0, sizeof(s_LastRecord));

	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.Conn.PeerAddrType = 0;
	for (size_t i = 0; i < sizeof(s_Peer.Conn.PeerAddr); ++i)
	{
		s_Peer.Conn.PeerAddr[i] = static_cast<uint8_t>(0x20U + i);
	}

	for (size_t i = 0; i < sizeof(s_Keys.Ltk); ++i)
	{
		s_Keys.Ltk[i] = static_cast<uint8_t>(0x40U + i);
		s_Keys.Csrk[i] = static_cast<uint8_t>(0x80U + i);
	}
	s_Keys.EncKeySize = 16;
	s_Keys.IdAddrType = 0;
	std::memcpy(s_Keys.IdAddr, s_Peer.Conn.PeerAddr,
				sizeof(s_Keys.IdAddr));
	s_Keys.bAuthenticated = true;
	s_Keys.bSc = true;
	s_Keys.bValid = true;

	s_LastRecordLen = 0;
	s_LastSlot = -1;
	s_SaveCount = 0;
	s_AutoComplete = AutoComplete;
	s_SaveSucceeds = true;
}

void AddBond()
{
	BtSmpBondAdd(kConnHdl, &s_Keys);
}

bool VerifyCounter(uint32_t Counter, bool ValidMac = true)
{
	uint8_t msg[12] = {
		BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD,
		0x34, 0x12,
		0xA1, 0xB2, 0xC3, 0xD4,
		0, 0, 0, 0, 0
	};
	uint8_t sig[12] = {};

	PutLe32(&msg[7], Counter);
	PutLe32(sig, Counter);
	BtSmpSignMac(s_Keys.Csrk, msg, 11, &sig[4]);
	if (!ValidMac)
	{
		sig[4] ^= 0x80;
	}

	return BtSmpSignVerify(kConnHdl, msg, 11, sig);
}

void CompleteLast(bool Success)
{
	BT_CHECK(s_Test, s_LastSlot >= 0);
	BT_CHECK(s_Test, s_LastRecordLen > 0);
	BtSmpBondPersistComplete(s_LastSlot, s_LastRecord,
								 s_LastRecordLen, Success);
}

void TestUncommittedRangeFailsClosed()
{
	ResetHarness(false);
	AddBond();

	BT_CHECK(s_Test, s_SaveCount == 1);
	BT_CHECK(s_Test, !VerifyCounter(0));

	CompleteLast(true);
	BT_CHECK(s_Test, !VerifyCounter(0, false));
	BT_CHECK(s_Test, VerifyCounter(0));
	BT_CHECK(s_Test, !VerifyCounter(0));
	BT_CHECK(s_Test, VerifyCounter(1));
	BT_CHECK(s_Test, !VerifyCounter(UINT32_MAX));
}

void TestResetSkipsReservedWindow()
{
	ResetHarness(true);
	AddBond();
	BT_CHECK(s_Test, VerifyCounter(0));
	BT_CHECK(s_Test, VerifyCounter(7));

	uint8_t persisted[sizeof(s_LastRecord)];
	size_t persistedLen = s_LastRecordLen;
	std::memcpy(persisted, s_LastRecord, persistedLen);

	// Simulate loss of all RAM state while retaining the last committed record.
	BtSmpBondClearAll();
	s_AutoComplete = true;
	s_SaveCount = 0;
	BtSmpBondRestore(0, persisted, persistedLen);

	BT_CHECK(s_Test, s_SaveCount == 1); // reserves the next window
	BT_CHECK(s_Test, !VerifyCounter(0));
	BT_CHECK(s_Test, !VerifyCounter(kWindow - 1));
	BT_CHECK(s_Test, VerifyCounter(kWindow));
}

void TestRefillBlocksAtOldBoundary()
{
	ResetHarness(true);
	AddBond();

	// Keep the initial window committed, but delay its refill write.
	s_AutoComplete = false;
	BT_CHECK(s_Test, VerifyCounter(kWindow - kRefill - 1));
	BT_CHECK(s_Test, s_SaveCount == 2);

	for (uint32_t counter = kWindow - kRefill; counter < kWindow; ++counter)
	{
		BT_CHECK(s_Test, VerifyCounter(counter));
	}

	BT_CHECK(s_Test, !VerifyCounter(kWindow));
	CompleteLast(true);
	BT_CHECK(s_Test, VerifyCounter(kWindow));
}

void TestFailedReservationStaysClosed()
{
	ResetHarness(false);
	AddBond();

	CompleteLast(false);
	BT_CHECK(s_Test, !VerifyCounter(0));

	CompleteLast(true);
	BT_CHECK(s_Test, VerifyCounter(0));
}

void TestVersionOneRetiresUncertainCsrk()
{
	ResetHarness(true);
	AddBond();

	uint8_t legacy[sizeof(s_LastRecord)];
	size_t legacyLen = s_LastRecordLen;
	std::memcpy(legacy, s_LastRecord, legacyLen);

	// Header layout is fixed: magic[0..3], version[4..5], length[6..7],
	// CRC[8..11]. Version 1 used the same record size but persisted an exact
	// receive counter asynchronously, so its CSRK cannot remain replay-safe.
	legacy[4] = 1;
	legacy[5] = 0;
	std::memset(&legacy[8], 0, 4);
	PutLe32(&legacy[8], Crc32(legacy, legacyLen));

	BtSmpBondClearAll();
	s_AutoComplete = true;
	s_SaveCount = 0;
	BtSmpBondRestore(0, legacy, legacyLen);

	BtSmpKeys_t restored = {};
	BT_CHECK(s_Test, BtSmpBondKeysLookup(kConnHdl, 0, 0, &restored));
	BT_CHECK(s_Test, std::memcmp(restored.Ltk, s_Keys.Ltk,
								sizeof(restored.Ltk)) == 0);
	BT_CHECK(s_Test, AllZero(restored.Csrk, sizeof(restored.Csrk)));
	BT_CHECK(s_Test, !VerifyCounter(0));
	BT_CHECK(s_Test, s_SaveCount == 1);
	BT_CHECK(s_Test, s_LastRecord[4] == 2 && s_LastRecord[5] == 0);
}

} // namespace

extern "C" {

void BtSmpBondPersistComplete(int Slot, const void *pBond,
									 size_t Len, bool Success);

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	return Hdl == kConnHdl ? &s_Peer : nullptr;
}

bool BtSmpRpaResolve(const uint8_t[16], const uint8_t[6])
{
	return false;
}

void BtSmpSignMac(const uint8_t Csrk[16], const uint8_t *pMsg,
				  size_t Len, uint8_t Mac[8])
{
	uint32_t a = 0x811C9DC5U;
	uint32_t b = 0x9E3779B9U;

	for (size_t i = 0; i < 16; ++i)
	{
		a = (a ^ Csrk[i]) * 16777619U;
		b ^= (static_cast<uint32_t>(Csrk[i]) << ((i & 3U) * 8U));
		b = (b << 5) | (b >> 27);
	}
	for (size_t i = 0; i < Len; ++i)
	{
		a = (a ^ pMsg[i]) * 16777619U;
		b += static_cast<uint32_t>(pMsg[i]) + (b << 6) + (b >> 2);
	}

	PutLe32(Mac, a);
	PutLe32(Mac + 4, b);
}

void CryptoSecureWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = static_cast<volatile uint8_t *>(pData);
	while (Len-- > 0)
	{
		*p++ = 0;
	}
}

void BtSmpBondSave(int Slot, const void *pBond, size_t Len)
{
	s_SaveCount++;
	s_LastSlot = Slot;
	s_LastRecordLen = Len <= sizeof(s_LastRecord) ? Len : 0;
	if (s_LastRecordLen > 0)
	{
		std::memcpy(s_LastRecord, pBond, s_LastRecordLen);
	}

	if (s_AutoComplete && s_LastRecordLen > 0)
	{
		BtSmpBondPersistComplete(Slot, pBond, Len, s_SaveSucceeds);
	}
}

void BtSmpBondErase(void)
{
	// Simulated power cycles clear RAM through BtSmpBondClearAll while the test
	// keeps a copied committed record for the subsequent restore.
}

} // extern "C"

int main()
{
	s_Test.Run("uncommitted range fails closed",
			   TestUncommittedRangeFailsClosed);
	s_Test.Run("reset skips reserved window",
			   TestResetSkipsReservedWindow);
	s_Test.Run("refill blocks at old boundary",
			   TestRefillBlocksAtOldBoundary);
	s_Test.Run("failed reservation stays closed",
			   TestFailedReservationStaysClosed);
	s_Test.Run("version one retires uncertain CSRK",
			   TestVersionOneRetiresUncertainCsrk);
	return s_Test.Finish();
}
