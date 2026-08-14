#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "syslog.h"

namespace {

int s_Checks;
int s_Failures;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

std::array<std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE>, 8> s_Packets = {};
std::array<size_t, 8> s_PacketLen = {};
size_t s_PacketCount;
size_t s_FailPacket;

bool s_ReserveResult;
int s_ReserveCount;
uint16_t s_ReserveConnHdl;
uint16_t s_ReserveNbPkt;
int s_ReleaseCount;
uint16_t s_ReleaseConnHdl;
int s_ResetCount;
uint16_t s_ResetConnHdl;

int s_CommandCount;
uint16_t s_CommandOpCode;
uint8_t s_CommandParam[3];

void Reset()
{
	s_PacketCount = 0;
	s_FailPacket = 0;
	s_ReserveResult = true;
	s_ReserveCount = 0;
	s_ReserveConnHdl = 0;
	s_ReserveNbPkt = 0;
	s_ReleaseCount = 0;
	s_ReleaseConnHdl = 0;
	s_ResetCount = 0;
	s_ResetConnHdl = 0;
	s_CommandCount = 0;
	s_CommandOpCode = 0;
	std::memset(s_CommandParam, 0, sizeof(s_CommandParam));
}

uint32_t CaptureAcl(void *pData, uint32_t Len)
{
	if (s_FailPacket != 0 && s_PacketCount + 1 == s_FailPacket)
	{
		return 0;
	}
	if (s_PacketCount >= s_Packets.size() ||
		Len > s_Packets[s_PacketCount].size())
	{
		return 0;
	}
	s_PacketLen[s_PacketCount] = Len;
	std::memcpy(s_Packets[s_PacketCount].data(), pData, Len);
	s_PacketCount++;
	return Len;
}

uint8_t CaptureCommand(BtHciDevice_t * const, uint16_t OpCode,
					   const void *pParam, uint8_t ParamLen,
					   void *, uint8_t)
{
	s_CommandCount++;
	s_CommandOpCode = OpCode;
	if (pParam != nullptr && ParamLen == sizeof(s_CommandParam))
	{
		std::memcpy(s_CommandParam, pParam, sizeof(s_CommandParam));
	}
	return 0;
}

using RawPacket = std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE>;

BtHciACLDataPacket_t *BuildPdu(RawPacket &Raw,
							   uint16_t ConnHdl, uint16_t Cid,
							   uint16_t PayloadLen)
{
	Raw.fill(0);
	BtHciACLDataPacket_t *pAcl =
		reinterpret_cast<BtHciACLDataPacket_t *>(Raw.data());
	BtL2CapHdr_t *pL2 = reinterpret_cast<BtL2CapHdr_t *>(pAcl->Data);
	pAcl->Hdr.ConnHdl = ConnHdl;
	pAcl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	pAcl->Hdr.BCFlag = 0;
	pL2->Len = PayloadLen;
	pL2->Cid = Cid;
	for (uint16_t i = 0; i < PayloadLen; i++)
	{
		pAcl->Data[sizeof(BtL2CapHdr_t) + i] =
			static_cast<uint8_t>(0x30U + i);
	}
	pAcl->Hdr.Len = static_cast<uint16_t>(
		sizeof(BtL2CapHdr_t) + PayloadLen);
	return pAcl;
}

void CheckDisconnect(uint16_t ConnHdl)
{
	CHECK(s_CommandCount == 1);
	CHECK(s_CommandOpCode == BT_HCI_CMD_LINKCTRL_DISCONNECT);
	CHECK(s_CommandParam[0] == static_cast<uint8_t>(ConnHdl));
	CHECK(s_CommandParam[1] == static_cast<uint8_t>(ConnHdl >> 8));
	CHECK(s_CommandParam[2] == 0x13);
}

void TestSingleSmpReservation()
{
	Reset();
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureCommand;
	alignas(4) RawPacket raw = {};
	BtHciACLDataPacket_t *pAcl =
		BuildPdu(raw, 0x0123, BT_L2CAP_CID_SEC_MNGR, 2);

	CHECK(BtHciSendAcl(&dev, pAcl) ==
		  pAcl->Hdr.Len + sizeof(pAcl->Hdr));
	CHECK(s_ReserveCount == 1);
	CHECK(s_ReserveConnHdl == 0x0123);
	CHECK(s_ReserveNbPkt == 1);
	CHECK(s_ReleaseCount == 0);
	CHECK(s_PacketCount == 1);
	CHECK(s_CommandCount == 0);
}

void TestFragmentedSmpReservation()
{
	Reset();
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureCommand;
	BtHciSetLeAclBuffer(&dev, 8, 3);
	alignas(4) RawPacket raw = {};
	BtHciACLDataPacket_t *pAcl =
		BuildPdu(raw, 0x0234, BT_L2CAP_CID_SEC_MNGR, 17);

	CHECK(pAcl->Hdr.Len == 21);
	CHECK(BtHciSendAcl(&dev, pAcl) ==
		  pAcl->Hdr.Len + sizeof(pAcl->Hdr));
	CHECK(s_ReserveCount == 1);
	CHECK(s_ReserveConnHdl == 0x0234);
	CHECK(s_ReserveNbPkt == 3);
	CHECK(s_ReleaseCount == 0);
	CHECK(s_PacketCount == 3);
	CHECK(dev.AclCredit == 0);
	CHECK(s_CommandCount == 0);
}

void TestReservationFailureDropsSmp()
{
	Reset();
	s_ReserveResult = false;
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureCommand;
	alignas(4) RawPacket raw = {};
	BtHciACLDataPacket_t *pAcl =
		BuildPdu(raw, 0x0345, BT_L2CAP_CID_SEC_MNGR, 2);

	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_ReserveCount == 1);
	CHECK(s_ReserveNbPkt == 1);
	CHECK(s_ReleaseCount == 0);
	CHECK(s_PacketCount == 0);
	CheckDisconnect(0x0345);
}

void TestCreditFailureDropsSmp()
{
	Reset();
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureCommand;
	BtHciSetLeAclBuffer(&dev, 8, 2);
	alignas(4) RawPacket raw = {};
	BtHciACLDataPacket_t *pAcl =
		BuildPdu(raw, 0x0456, BT_L2CAP_CID_SEC_MNGR, 17);

	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_ReserveCount == 0);
	CHECK(s_ReleaseCount == 0);
	CHECK(s_PacketCount == 0);
	CHECK(dev.AclCredit == 2);
	CheckDisconnect(0x0456);
}

void TestFirstSendFailureReleasesAndDropsSmp()
{
	Reset();
	s_FailPacket = 1;
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureCommand;
	BtHciSetLeAclBuffer(&dev, 8, 3);
	alignas(4) RawPacket raw = {};
	BtHciACLDataPacket_t *pAcl =
		BuildPdu(raw, 0x0567, BT_L2CAP_CID_SEC_MNGR, 17);

	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_ReserveCount == 1);
	CHECK(s_ReserveNbPkt == 3);
	CHECK(s_ReleaseCount == 1);
	CHECK(s_ReleaseConnHdl == 0x0567);
	CHECK(s_PacketCount == 0);
	CHECK(dev.AclCredit == 3);
	CheckDisconnect(0x0567);
}

void TestPartialSendFailureKeepsReservationAndDropsSmp()
{
	Reset();
	s_FailPacket = 2;
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureCommand;
	BtHciSetLeAclBuffer(&dev, 8, 5);
	alignas(4) RawPacket raw = {};
	BtHciACLDataPacket_t *pAcl =
		BuildPdu(raw, 0x0678, BT_L2CAP_CID_SEC_MNGR, 17);

	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_ReserveCount == 1);
	CHECK(s_ReserveNbPkt == 3);
	CHECK(s_ReleaseCount == 0);
	CHECK(s_PacketCount == 1);
	CHECK(dev.AclCredit == 4);
	CheckDisconnect(0x0678);

	// One fragment is in the controller and the rest of the PDU never will
	// be, so this group can never reach the count it reserved. Its
	// completion would otherwise retire the oldest entry in the ring, which
	// belongs to something else. The send order cannot be reconstructed and
	// the link is going, so the ring goes with it.
	CHECK(s_ResetCount == 1);
	CHECK(s_ResetConnHdl == 0x0678);
}

void TestNonSmpFailureBehaviorUnchanged()
{
	Reset();
	s_FailPacket = 1;
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureCommand;
	BtHciSetLeAclBuffer(&dev, 8, 3);
	alignas(4) RawPacket raw = {};
	BtHciACLDataPacket_t *pAcl =
		BuildPdu(raw, 0x0789, BT_L2CAP_CID_ATT, 17);

	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_ReserveCount == 0);
	CHECK(s_ReleaseCount == 0);
	CHECK(s_PacketCount == 0);
	CHECK(dev.AclCredit == 3);
	CHECK(s_CommandCount == 0);

	// Nothing was accepted, so the caller's own release is the right answer
	// and the ring is left alone.
	CHECK(s_ResetCount == 0);
}

// The same partial send on a link that is not carrying SMP. The PDU has no
// reservation of its own here, its caller made one, and the ring still has to
// go because the fragments already accepted have no group left to complete.
void TestPartialSendResetsRingWithoutSmp()
{
	Reset();
	s_FailPacket = 2;
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureCommand;
	BtHciSetLeAclBuffer(&dev, 8, 5);
	alignas(4) RawPacket raw = {};
	BtHciACLDataPacket_t *pAcl =
		BuildPdu(raw, 0x0234, BT_L2CAP_CID_ATT, 17);

	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_ReserveCount == 0);
	CHECK(s_ReleaseCount == 0);
	CHECK(s_PacketCount == 1);
	CHECK(s_ResetCount == 1);
	CHECK(s_ResetConnHdl == 0x0234);
	CheckDisconnect(0x0234);
}

} // namespace

bool BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)
{
	s_ReserveCount++;
	s_ReserveConnHdl = ConnHdl;
	s_ReserveNbPkt = NbPkt;
	return s_ReserveResult;
}

void BtGattTxPendRelease(uint16_t ConnHdl)
{
	s_ReleaseCount++;
	s_ReleaseConnHdl = ConnHdl;
}

void BtGattTxPendReset(uint16_t ConnHdl)
{
	s_ResetCount++;
	s_ResetConnHdl = ConnHdl;
}

// bt_hci_host.cpp logs on paths this test does not exercise. Section garbage
// collection used to drop those references; a new one anywhere in the file
// would break the link instead of the test.
extern "C" {

SysLog_t *SysLogGet(void)
{
	static SysLog_t log = {};
	return &log;
}

int SysLogPrintf(SysLog_t * const, const char *, ...)
{
	return 0;
}

} // extern "C"

int main()
{
	TestSingleSmpReservation();
	TestFragmentedSmpReservation();
	TestReservationFailureDropsSmp();
	TestCreditFailureDropsSmp();
	TestFirstSendFailureReleasesAndDropsSmp();
	TestPartialSendFailureKeepsReservationAndDropsSmp();
	TestNonSmpFailureBehaviorUnchanged();
	TestPartialSendResetsRingWithoutSmp();

	if (s_Failures != 0)
	{
		std::printf("Bluetooth SMP TX accounting tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}
	std::printf("Bluetooth SMP TX accounting tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
