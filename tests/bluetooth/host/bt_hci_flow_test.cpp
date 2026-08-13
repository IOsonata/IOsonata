#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_smp.h"
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

struct CapturedPacket {
	size_t Len;
	std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> Data;
};

std::array<CapturedPacket, 12> s_AclPackets = {};
size_t s_AclPacketCount = 0;

std::array<uint8_t, BTHCICTLR_CMD_HDR_LEN + BTHCICTLR_CMD_PARAM_MAX> s_Command = {};
size_t s_CommandLen = 0;

struct SmpCapture {
	uint16_t ConnHdl;
	size_t Len;
	std::array<uint8_t, 64> Data;
};

std::array<SmpCapture, 8> s_SmpPackets = {};
size_t s_SmpPacketCount = 0;

uint16_t s_CompletedHdl = 0;
uint16_t s_CompletedCount = 0;
int s_CompletedCallbacks = 0;
int s_ScanReports = 0;
int s_SmpDisconnectCount = 0;
uint16_t s_SmpDisconnectHdl = 0;
int s_LtkRequestCount = 0;

// Number of the fragment the stub transport refuses, counted from 1. Zero
// accepts everything. Lets a test drive a transport that stops accepting part
// way through a fragmented PDU.
size_t s_AclFailOnPacket = 0;

// Device the ACL stub reads the credit count from, and what it saw as each
// fragment arrived. Shows whether the credits for the whole PDU were taken
// before the first fragment or one at a time as they went out.
BtHciDevice_t *s_AclObservedDev = nullptr;
int16_t s_AclCreditAtSend[8] = {};

// Commands the device issues, so a test can see the disconnect a dropped link
// produces.
uint16_t s_DevCmdOpCode = 0;
uint8_t  s_DevCmdParam[8] = {};
uint8_t  s_DevCmdParamLen = 0;
int      s_DevCmdCount = 0;

uint8_t CaptureDevCommand(BtHciDevice_t * const, uint16_t OpCode, const void *pParam,
						  uint8_t ParamLen, void *, uint8_t)
{
	s_DevCmdOpCode = OpCode;
	s_DevCmdParamLen = ParamLen;
	if (pParam != nullptr && ParamLen <= sizeof(s_DevCmdParam))
	{
		std::memcpy(s_DevCmdParam, pParam, ParamLen);
	}
	++s_DevCmdCount;
	return 0;
}

void ResetAclCapture()
{
	s_AclPacketCount = 0;
	s_AclFailOnPacket = 0;
	s_AclObservedDev = nullptr;
	std::memset(s_AclCreditAtSend, 0, sizeof(s_AclCreditAtSend));
	s_DevCmdCount = 0;
	s_DevCmdOpCode = 0;
	s_DevCmdParamLen = 0;
	for (auto &pkt : s_AclPackets)
	{
		pkt.Len = 0;
		pkt.Data.fill(0);
	}
}

void ResetSmpCapture()
{
	s_SmpPacketCount = 0;
	for (auto &pkt : s_SmpPackets)
	{
		pkt.ConnHdl = 0;
		pkt.Len = 0;
		pkt.Data.fill(0);
	}
}

uint32_t CaptureAcl(void *pData, uint32_t Len)
{
	if (s_AclPacketCount >= s_AclPackets.size() ||
		Len > s_AclPackets[s_AclPacketCount].Data.size())
	{
		return 0;
	}

	if (s_AclFailOnPacket != 0 && s_AclPacketCount + 1 == s_AclFailOnPacket)
	{
		return 0;					// transport refused this fragment
	}

	if (s_AclObservedDev != nullptr &&
		s_AclPacketCount < (sizeof(s_AclCreditAtSend) / sizeof(s_AclCreditAtSend[0])))
	{
		s_AclCreditAtSend[s_AclPacketCount] = s_AclObservedDev->AclCredit;
	}

	CapturedPacket &pkt = s_AclPackets[s_AclPacketCount++];
	pkt.Len = Len;
	std::memcpy(pkt.Data.data(), pData, Len);
	return Len;
}

size_t CaptureCommand(BtHciCtlrDev_t * const, void * const pData, size_t Len)
{
	if (Len > s_Command.size())
	{
		return 0;
	}

	s_CommandLen = Len;
	std::memcpy(s_Command.data(), pData, Len);
	return Len;
}

void CaptureCompleted(uint16_t ConnHdl, uint16_t Count)
{
	s_CompletedHdl = ConnHdl;
	s_CompletedCount = Count;
	++s_CompletedCallbacks;
}

size_t s_LastScanLen = 0;
uint8_t s_LastScanData[512] = {};

bool CaptureScanReport(int8_t, uint8_t, uint8_t[6], size_t Len, uint8_t *pData)
{
	++s_ScanReports;
	s_LastScanLen = Len;
	if (Len > 0 && Len <= sizeof(s_LastScanData) && pData != nullptr)
	{
		std::memcpy(s_LastScanData, pData, Len);
	}
	return true;
}

void FeedAclFragment(BtHciDevice_t *pDev, uint16_t ConnHdl, uint8_t PbFlag,
					 const uint8_t *pData, uint16_t Len)
{
	alignas(4) std::array<uint8_t, BT_HCI_BUFFER_MAX_SIZE> raw = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw.data());

	pAcl->Hdr.ConnHdl = ConnHdl;
	pAcl->Hdr.PBFlag = PbFlag;
	pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
	pAcl->Hdr.Len = Len;
	std::memcpy(pAcl->Data, pData, Len);

	BtHciProcessData(pDev, pAcl);
}

size_t BuildSmpPdu(uint8_t *pOut, uint8_t Code, const uint8_t *pData, size_t Len)
{
	// Write the L2CAP header (Len, Cid) and SMP code as on-air little-endian
	// bytes rather than casting pOut to the full BtL2CapPdu_t union: the caller
	// buffers are smaller than that union, so the cast would be an out-of-bounds
	// object access (flagged by clang UBSan). Byte writes are also alignment
	// safe.
	uint16_t l2Len = static_cast<uint16_t>(Len + 1U);
	uint16_t cid   = BT_L2CAP_CID_SEC_MNGR;
	pOut[0] = static_cast<uint8_t>(l2Len & 0xFF);
	pOut[1] = static_cast<uint8_t>(l2Len >> 8);
	pOut[2] = static_cast<uint8_t>(cid & 0xFF);
	pOut[3] = static_cast<uint8_t>(cid >> 8);
	pOut[4] = Code;
	if (Len > 0)
	{
		std::memcpy(pOut + 5, pData, Len);
	}
	return sizeof(BtL2CapHdr_t) + Len + 1U;
}

void TestCommandFraming()
{
	BtHciCtlrDev_t ctlr = {};
	ctlr.SendCommand = CaptureCommand;
	s_CommandLen = 0;
	s_Command.fill(0);

	const uint8_t param[] = { 0x11, 0x22, 0x33 };
	const uint16_t opcode = 0x2042;

	CHECK(BtHciCtlrSendCommand(&ctlr, opcode, param, sizeof(param)) ==
		  BTHCICTLR_CMD_HDR_LEN + sizeof(param));
	CHECK(s_CommandLen == BTHCICTLR_CMD_HDR_LEN + sizeof(param));
	CHECK(s_Command[0] == 0x42);
	CHECK(s_Command[1] == 0x20);
	CHECK(s_Command[2] == sizeof(param));
	CHECK(std::memcmp(&s_Command[3], param, sizeof(param)) == 0);

	CHECK(BtHciCtlrSendCommand(&ctlr, opcode, nullptr, 1) == 0);
}

void TestAclFragmentationAndCredits()
{
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	ResetAclCapture();

	alignas(4) std::array<uint8_t, 64> raw = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw.data());
	pAcl->Hdr.ConnHdl = 0x123;
	pAcl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
	pAcl->Hdr.Len = 25;
	for (uint16_t i = 0; i < pAcl->Hdr.Len; ++i)
	{
		pAcl->Data[i] = static_cast<uint8_t>(i);
	}

	BtHciSetLeAclBuffer(&dev, 10, 3);
	CHECK(dev.AclMaxLen == 10);
	CHECK(dev.AclCreditMax == 3);
	CHECK(dev.AclCredit == 3);
	CHECK(dev.CmdCredit == 1);

	CHECK(BtHciSendAcl(&dev, pAcl) == pAcl->Hdr.Len + sizeof(pAcl->Hdr));
	CHECK(s_AclPacketCount == 3);
	CHECK(dev.AclCredit == 0);

	const uint16_t expectedLen[] = { 10, 10, 5 };
	uint16_t offset = 0;
	for (size_t i = 0; i < s_AclPacketCount; ++i)
	{
		const BtHciACLDataPacket_t *p =
			reinterpret_cast<const BtHciACLDataPacket_t *>(s_AclPackets[i].Data.data());
		CHECK(p->Hdr.ConnHdl == 0x123);
		CHECK(p->Hdr.Len == expectedLen[i]);
		CHECK(p->Hdr.PBFlag == (i == 0 ? BT_HCI_PBFLAG_START_NONFLUSHABLE
									   : BT_HCI_PBFLAG_CONTINUING_FRAGMENT));
		CHECK(s_AclPackets[i].Len == expectedLen[i] + sizeof(p->Hdr));
		CHECK(std::memcmp(p->Data, &pAcl->Data[offset], expectedLen[i]) == 0);
		offset = static_cast<uint16_t>(offset + expectedLen[i]);
	}

	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_AclPacketCount == 3);
}

// F12: the credit gate and what happens when the transport stops accepting
// part way through a fragmented PDU.
//
// The credits for the whole PDU are taken before the first fragment goes out.
// Comparing without taking left them available to anything else sending on the
// device between fragments. If a fragment is refused, the credits for the ones
// never sent go back, and a PDU already started in the controller cannot be
// completed or withdrawn, so the link is dropped rather than left holding a
// START forever.
void TestAclFragmentSendFailure()
{
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureDevCommand;
	ResetAclCapture();

	alignas(4) std::array<uint8_t, 64> raw = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw.data());
	pAcl->Hdr.ConnHdl = 0x0123;
	pAcl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
	pAcl->Hdr.Len = 25;
	for (uint16_t i = 0; i < pAcl->Hdr.Len; ++i)
	{
		pAcl->Data[i] = static_cast<uint8_t>(i);
	}

	// 25 bytes over 10-byte fragments is three packets, and eight credits.
	BtHciSetLeAclBuffer(&dev, 10, 8);

	// The transport refuses the second fragment. One is already in the
	// controller, so that credit stays spent and the other two come back.
	s_AclFailOnPacket = 2;
	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_AclPacketCount == 1);
	CHECK(dev.AclCredit == 7);

	// The link cannot carry the rest of the PDU, so it is disconnected.
	CHECK(s_DevCmdCount == 1);
	CHECK(s_DevCmdOpCode == BT_HCI_CMD_LINKCTRL_DISCONNECT);
	CHECK(s_DevCmdParamLen == 3);
	CHECK(s_DevCmdParam[0] == 0x23);
	CHECK(s_DevCmdParam[1] == 0x01);
	CHECK(s_DevCmdParam[2] == 0x13);

	// A refusal on the very first fragment puts nothing on the link, so every
	// credit comes back and there is nothing to disconnect for.
	ResetAclCapture();
	BtHciSetLeAclBuffer(&dev, 10, 8);
	s_AclFailOnPacket = 1;
	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_AclPacketCount == 0);
	CHECK(dev.AclCredit == 8);
	CHECK(s_DevCmdCount == 0);

	// Too few credits for the whole PDU: refused before anything is sent.
	ResetAclCapture();
	BtHciSetLeAclBuffer(&dev, 10, 2);
	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_AclPacketCount == 0);
	CHECK(dev.AclCredit == 2);
	CHECK(s_DevCmdCount == 0);

	// With exactly enough, the PDU goes out and the credits are spent. The
	// count seen by the transport shows they were all taken before the first
	// fragment rather than one at a time: taking them one at a time would have
	// left 2 and 1 available while fragments 1 and 2 were being handed over,
	// which is what another sender could have spent.
	ResetAclCapture();
	s_AclObservedDev = &dev;
	BtHciSetLeAclBuffer(&dev, 10, 3);
	CHECK(BtHciSendAcl(&dev, pAcl) == pAcl->Hdr.Len + sizeof(pAcl->Hdr));
	CHECK(s_AclPacketCount == 3);
	CHECK(dev.AclCredit == 0);
	CHECK(s_DevCmdCount == 0);
	CHECK(s_AclCreditAtSend[0] == 0);
	CHECK(s_AclCreditAtSend[1] == 0);
	CHECK(s_AclCreditAtSend[2] == 0);
}

// Send one single-fragment ACL PDU on a link, carrying an ATT CID so the SMP
// branch of BtHciSendAcl is not involved.
uint32_t SendOneAcl(BtHciDevice_t *pDev, uint16_t ConnHdl)
{
	alignas(4) std::array<uint8_t, 32> raw = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw.data());

	pAcl->Hdr.ConnHdl = ConnHdl;
	pAcl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
	pAcl->Hdr.Len = 8;
	pAcl->Data[0] = 4;					// L2CAP length
	pAcl->Data[1] = 0;
	pAcl->Data[2] = BT_L2CAP_CID_ATT & 0xFF;
	pAcl->Data[3] = BT_L2CAP_CID_ATT >> 8;

	return BtHciSendAcl(pDev, pAcl);
}

void FeedCompleted(BtHciDevice_t *pDev, uint16_t ConnHdl, uint16_t NbPkt)
{
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 5] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvt->Hdr.Evt = BT_HCI_EVT_NB_COMPLETED_PACKET;
	pEvt->Hdr.Len = 5;
	BtHciEvtNbCompletedPkt_t *p =
		reinterpret_cast<BtHciEvtNbCompletedPkt_t *>(pEvt->Data);
	p->NbHdl = 1;
	p->Completed[0].Hdl = ConnHdl;
	p->Completed[0].NbPkt = NbPkt;

	BtHciProcessEvent(pDev, pEvt);
}

void FeedDisconnect(BtHciDevice_t *pDev, uint16_t ConnHdl)
{
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) +
						  sizeof(BtHciEvtDisconComplete_t)] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvt->Hdr.Evt = BT_HCI_EVT_DISCONN_COMPLETE;
	pEvt->Hdr.Len = sizeof(BtHciEvtDisconComplete_t);
	BtHciEvtDisconComplete_t *p =
		reinterpret_cast<BtHciEvtDisconComplete_t *>(pEvt->Data);
	p->Status = 0;
	p->ConnHdl = ConnHdl;
	p->Reason = 0x13;

	BtHciProcessEvent(pDev, pEvt);
}

// Vol 4 Part E 4.3: on Disconnection Complete the host shall assume every
// unacknowledged packet sent for that handle has been flushed and its buffer
// freed, and the controller does not have to report those packets in a Number
// Of Completed Packets event first. Credits held by packets in flight at
// teardown are therefore returned by nobody unless the host returns them, and
// the pool shrinks on every disconnect until sending stops working.
void TestAclCreditsReturnedOnDisconnect()
{
	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	dev.Command = CaptureDevCommand;
	ResetAclCapture();

	const uint16_t hdlA = 0x0011;
	const uint16_t hdlB = 0x0022;

	BtHciSetLeAclBuffer(&dev, 27, 4);
	CHECK(dev.AclCredit == 4);

	// Two packets outstanding on A, one on B.
	CHECK(SendOneAcl(&dev, hdlA) != 0);
	CHECK(SendOneAcl(&dev, hdlA) != 0);
	CHECK(SendOneAcl(&dev, hdlB) != 0);
	CHECK(dev.AclCredit == 1);

	// A goes down: its two credits come back, B keeps holding its one.
	FeedDisconnect(&dev, hdlA);
	CHECK(dev.AclCredit == 3);

	FeedDisconnect(&dev, hdlB);
	CHECK(dev.AclCredit == 4);

	// Packets already reported complete must not be returned a second time.
	// The credit count is kept below the maximum by B's outstanding packet so
	// a double return would show rather than being hidden by the clamp.
	ResetAclCapture();
	BtHciSetLeAclBuffer(&dev, 27, 4);
	CHECK(SendOneAcl(&dev, hdlA) != 0);
	CHECK(SendOneAcl(&dev, hdlA) != 0);
	FeedCompleted(&dev, hdlA, 2);
	CHECK(dev.AclCredit == 4);
	CHECK(SendOneAcl(&dev, hdlB) != 0);
	CHECK(dev.AclCredit == 3);
	FeedDisconnect(&dev, hdlA);
	CHECK(dev.AclCredit == 3);
	FeedDisconnect(&dev, hdlB);
	CHECK(dev.AclCredit == 4);

	// The reported failure: repeated connect, send, disconnect cycles with
	// traffic outstanding used to drive the pool to zero and leave every
	// further send refused.
	ResetAclCapture();
	BtHciSetLeAclBuffer(&dev, 27, 4);
	for (int i = 0; i < 20; ++i)
	{
		ResetAclCapture();
		uint16_t hdl = static_cast<uint16_t>(0x0100 + i);
		CHECK(SendOneAcl(&dev, hdl) != 0);
		CHECK(SendOneAcl(&dev, hdl) != 0);
		FeedDisconnect(&dev, hdl);
	}
	CHECK(dev.AclCredit == 4);
	CHECK(SendOneAcl(&dev, hdlA) != 0);

	// A transport that stops part way through a fragmented PDU leaves only the
	// fragments it accepted in the controller, so only those are returned.
	ResetAclCapture();
	BtHciSetLeAclBuffer(&dev, 10, 8);

	alignas(4) std::array<uint8_t, 64> raw = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw.data());
	pAcl->Hdr.ConnHdl = hdlA;
	pAcl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
	pAcl->Hdr.Len = 25;
	pAcl->Data[0] = 21;
	pAcl->Data[1] = 0;
	pAcl->Data[2] = BT_L2CAP_CID_ATT & 0xFF;
	pAcl->Data[3] = BT_L2CAP_CID_ATT >> 8;

	s_AclFailOnPacket = 2;
	CHECK(BtHciSendAcl(&dev, pAcl) == 0);
	CHECK(s_AclPacketCount == 1);
	CHECK(dev.AclCredit == 7);
	FeedDisconnect(&dev, hdlA);
	CHECK(dev.AclCredit == 8);
}

void TestAclCompletionCredits()
{
	BtHciDevice_t dev = {};
	dev.AclCreditMax = 3;
	dev.AclCredit = 0;
	dev.SendCompleted = CaptureCompleted;
	s_CompletedCallbacks = 0;

	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 5] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvt->Hdr.Evt = BT_HCI_EVT_NB_COMPLETED_PACKET;
	pEvt->Hdr.Len = 5;
	BtHciEvtNbCompletedPkt_t *p =
		reinterpret_cast<BtHciEvtNbCompletedPkt_t *>(pEvt->Data);
	p->NbHdl = 1;
	p->Completed[0].Hdl = 0x123;
	p->Completed[0].NbPkt = 2;

	BtHciProcessEvent(&dev, pEvt);
	CHECK(dev.AclCredit == 2);
	CHECK(s_CompletedCallbacks == 1);
	CHECK(s_CompletedHdl == 0x123);
	CHECK(s_CompletedCount == 2);

	p->Completed[0].NbPkt = 9;
	BtHciProcessEvent(&dev, pEvt);
	CHECK(dev.AclCredit == 3);
}

// A Number Of Completed Packets event whose NbHdl count exceeds what the event
// length can hold must be bounded to the payload; the buffer is sized to exactly
// one entry so an unbounded walk would run off the end (caught by ASan).
void TestCompletedPacketsBounded()
{
	BtHciDevice_t dev = {};
	dev.AclCreditMax = 20;
	dev.AclCredit = 0;
	dev.SendCompleted = CaptureCompleted;
	s_CompletedCallbacks = 0;

	// Payload holds NbHdl(1) + exactly one Completed entry (4) = 5 bytes.
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 5] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvt->Hdr.Evt = BT_HCI_EVT_NB_COMPLETED_PACKET;
	pEvt->Hdr.Len = 5;
	BtHciEvtNbCompletedPkt_t *p =
		reinterpret_cast<BtHciEvtNbCompletedPkt_t *>(pEvt->Data);
	p->NbHdl = 10;					// lies: only one entry actually present
	p->Completed[0].Hdl = 0x321;
	p->Completed[0].NbPkt = 4;

	BtHciProcessEvent(&dev, pEvt);

	// Only the single in-bounds entry is processed.
	CHECK(dev.AclCredit == 4);
	CHECK(s_CompletedCallbacks == 1);
	CHECK(s_CompletedHdl == 0x321);
}

void TestCommandCreditsAndCompletion()
{
	BtHciDevice_t dev = {};
	uint8_t ret[2] = {};
	const uint16_t opcode = 0x201C;

	dev.CmdCredit = 0;
	dev.CmdOpCode = opcode;
	dev.pCmdRet = ret;
	dev.CmdRetLen = sizeof(ret);
	dev.CmdDone = false;

	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 6] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvt->Hdr.Evt = BT_HCI_EVT_COMMAND_COMPLETE;
	pEvt->Hdr.Len = 6;
	BtHciEvtCmdComplete_t *p = reinterpret_cast<BtHciEvtCmdComplete_t *>(pEvt->Data);
	p->NbCmdPacket = 3;
	p->CmdCode = opcode;
	p->RetParam[0] = 0x0C;
	p->RetParam[1] = 0xA5;
	p->RetParam[2] = 0x5A;

	BtHciProcessEvent(&dev, pEvt);
	CHECK(dev.CmdCredit == 3);
	CHECK(dev.CmdDone);
	CHECK(dev.CmdStatus == 0x0C);
	CHECK(ret[0] == 0xA5 && ret[1] == 0x5A);

	alignas(4) uint8_t statusRaw[sizeof(BtHciEvtPacketHdr_t) + sizeof(BtHciEvtCmdStatus_t)] = {};
	BtHciEvtPacket_t *pStatusEvt = reinterpret_cast<BtHciEvtPacket_t *>(statusRaw);
	pStatusEvt->Hdr.Evt = BT_HCI_EVT_COMMAND_STATUS;
	pStatusEvt->Hdr.Len = sizeof(BtHciEvtCmdStatus_t);
	BtHciEvtCmdStatus_t *pStatus =
		reinterpret_cast<BtHciEvtCmdStatus_t *>(pStatusEvt->Data);
	pStatus->Status = 0x01;
	pStatus->NbCmdPacket = 1;
	pStatus->CmdCode = static_cast<uint16_t>(opcode + 1U);

	dev.CmdDone = false;
	BtHciProcessEvent(&dev, pStatusEvt);
	CHECK(dev.CmdCredit == 1);
	CHECK(!dev.CmdDone);
}

// A Command Complete shorter than NbCmdPacket(1) + CmdCode(2) is malformed and
// must be dropped without updating credits or reading the status byte (H2).
void TestCommandCompleteBounds()
{
	BtHciDevice_t dev = {};
	dev.CmdCredit = 7;			// sentinel that must survive a malformed event
	dev.CmdOpCode = 0x201C;
	dev.CmdDone = false;

	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 6] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvt->Hdr.Evt = BT_HCI_EVT_COMMAND_COMPLETE;
	pEvt->Hdr.Len = 2;			// too short for NbCmdPacket + CmdCode
	BtHciEvtCmdComplete_t *p = reinterpret_cast<BtHciEvtCmdComplete_t *>(pEvt->Data);
	p->NbCmdPacket = 0x7F;
	p->CmdCode = 0x201C;

	BtHciProcessEvent(&dev, pEvt);
	CHECK(dev.CmdCredit == 7);
	CHECK(!dev.CmdDone);
}

void TestAclReassembly()
{
	BtHciDevice_t dev = {};
	ResetSmpCapture();

	uint8_t pdu[32] = {};
	const uint8_t payload[] = { 1, 2, 3, 4, 5 };
	size_t pduLen = BuildSmpPdu(pdu, 0x0B, payload, sizeof(payload));

	FeedAclFragment(&dev, 0x201, BT_HCI_PBFLAG_START_NONFLUSHABLE, pdu, 6);
	CHECK(s_SmpPacketCount == 0);
	FeedAclFragment(&dev, 0x201, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
					&pdu[6], static_cast<uint16_t>(pduLen - 6));

	CHECK(s_SmpPacketCount == 1);
	CHECK(s_SmpPackets[0].ConnHdl == 0x201);
	CHECK(s_SmpPackets[0].Len == sizeof(payload) + 1U);
	CHECK(s_SmpPackets[0].Data[0] == 0x0B);
	CHECK(std::memcmp(&s_SmpPackets[0].Data[1], payload, sizeof(payload)) == 0);
}

void TestInterleavedReassembly()
{
	BtHciDevice_t dev = {};
	ResetSmpCapture();

	uint8_t pduA[32] = {};
	uint8_t pduB[32] = {};
	const uint8_t dataA[] = { 0xA1, 0xA2, 0xA3 };
	const uint8_t dataB[] = { 0xB1, 0xB2, 0xB3, 0xB4 };
	size_t lenA = BuildSmpPdu(pduA, 0x01, dataA, sizeof(dataA));
	size_t lenB = BuildSmpPdu(pduB, 0x02, dataB, sizeof(dataB));

	FeedAclFragment(&dev, 0x301, BT_HCI_PBFLAG_START_NONFLUSHABLE, pduA, 5);
	FeedAclFragment(&dev, 0x302, BT_HCI_PBFLAG_START_NONFLUSHABLE, pduB, 5);
	CHECK(s_SmpPacketCount == 0);

	FeedAclFragment(&dev, 0x302, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
					&pduB[5], static_cast<uint16_t>(lenB - 5));
	FeedAclFragment(&dev, 0x301, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
					&pduA[5], static_cast<uint16_t>(lenA - 5));

	CHECK(s_SmpPacketCount == 2);
	CHECK(s_SmpPackets[0].ConnHdl == 0x302);
	CHECK(s_SmpPackets[0].Data[0] == 0x02);
	CHECK(s_SmpPackets[1].ConnHdl == 0x301);
	CHECK(s_SmpPackets[1].Data[0] == 0x01);
}

// A Disconnection Complete event must free the SMP link record via
// BtSmpDisconnected, so cleanup does not depend on the application callback.
void TestDisconnectFreesSmp()
{
	BtHciDevice_t dev = {};
	s_SmpDisconnectCount = 0;
	s_SmpDisconnectHdl = 0;

	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) +
						  sizeof(BtHciEvtDisconComplete_t)] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvt->Hdr.Evt = BT_HCI_EVT_DISCONN_COMPLETE;
	pEvt->Hdr.Len = sizeof(BtHciEvtDisconComplete_t);
	BtHciEvtDisconComplete_t *p =
		reinterpret_cast<BtHciEvtDisconComplete_t *>(pEvt->Data);
	p->Status = 0;
	p->ConnHdl = 0x0042;
	p->Reason = 0x13;

	BtHciProcessEvent(&dev, pEvt);

	CHECK(s_SmpDisconnectCount == 1);
	CHECK(s_SmpDisconnectHdl == 0x0042);
}

void TestMalformedFragmentsAndEvents()
{
	BtHciDevice_t dev = {};
	ResetSmpCapture();

	const uint8_t orphan[] = { 1, 2, 3 };
	FeedAclFragment(&dev, 0x401, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
					orphan, sizeof(orphan));
	CHECK(s_SmpPacketCount == 0);

	uint8_t pdu[16] = {};
	const uint8_t payload[] = { 1, 2, 3 };
	size_t pduLen = BuildSmpPdu(pdu, 0x03, payload, sizeof(payload));
	FeedAclFragment(&dev, 0x401, BT_HCI_PBFLAG_START_NONFLUSHABLE, pdu, 5);

	const uint8_t tooLong[] = { 9, 9, 9, 9, 9 };
	FeedAclFragment(&dev, 0x401, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
					tooLong, sizeof(tooLong));
	FeedAclFragment(&dev, 0x401, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
					&pdu[5], static_cast<uint16_t>(pduLen - 5));
	CHECK(s_SmpPacketCount == 0);

	s_ScanReports = 0;
	dev.ScanReport = CaptureScanReport;
	// Physically size the buffer for the report structs written below; the
	// event stays malformed via Hdr.Len = 11 (shorter than the claimed report),
	// which is what the handler bounds against. A buffer only 11 bytes past the
	// event header would make the struct member writes an out-of-bounds access.
	alignas(4) uint8_t evtRaw[sizeof(BtHciEvtPacketHdr_t) + 32] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(evtRaw);
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = 11;
	BtHciLeEvtPacket_t *pLe = reinterpret_cast<BtHciLeEvtPacket_t *>(pEvt->Data);
	pLe->Evt = BT_HCI_EVT_LE_ADV_REPORT;
	BtHciLeEvtAdvReport_t *pReport =
		reinterpret_cast<BtHciLeEvtAdvReport_t *>(pLe->Data);
	pReport->NbReport = 1;
	pReport->Report[0].DataLen = 20;

	BtHciProcessEvent(&dev, pEvt);
	CHECK(s_ScanReports == 0);
}

// A fixed-payload LE meta event whose Hdr.Len is too short to contain the
// declared structure must be dropped, not parsed out of bounds. The LTK
// request feeds SMP, so an unchecked read there is the worst case (H1).
void TestTruncatedLeMetaEvents()
{
	BtHciDevice_t dev = {};

	// Full-length LE Long Term Key Request: subevent(1) + payload(12) = 13.
	{
		alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 13] = {};
		BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
		pEvt->Hdr.Evt = BT_HCI_EVT_LE;
		pEvt->Hdr.Len = 13;
		BtHciLeEvtPacket_t *pLe = reinterpret_cast<BtHciLeEvtPacket_t *>(pEvt->Data);
		pLe->Evt = BT_HCI_EVT_LE_LONGTERM_KEY_RQST;
		s_LtkRequestCount = 0;
		BtHciProcessEvent(&dev, pEvt);
		CHECK(s_LtkRequestCount == 1);
	}

	// Truncated: Hdr.Len 8 is shorter than the 12-byte payload. The bound must
	// stop it before it reaches SMP.
	{
		alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 13] = {};
		BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
		pEvt->Hdr.Evt = BT_HCI_EVT_LE;
		pEvt->Hdr.Len = 8;
		BtHciLeEvtPacket_t *pLe = reinterpret_cast<BtHciLeEvtPacket_t *>(pEvt->Data);
		pLe->Evt = BT_HCI_EVT_LE_LONGTERM_KEY_RQST;
		s_LtkRequestCount = 0;
		BtHciProcessEvent(&dev, pEvt);
		CHECK(s_LtkRequestCount == 0);
	}
}

// Multi-fragment Extended Advertising Reports (Data_Status = "more data") are
// reassembled into one report delivered on the terminating "complete" fragment
// (G3, Vol 4 Part E 7.7.65.13).
void FeedExtAdvReport(BtHciDevice_t *pDev, const uint8_t Addr[6], uint8_t Sid,
					  uint8_t DataStatus, const char *pData, uint8_t DataLen)
{
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 1 + 24 + 64] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = static_cast<uint8_t>(1 + 1 + 24 + DataLen);
	BtHciLeEvtPacket_t *pLe = reinterpret_cast<BtHciLeEvtPacket_t *>(pEvt->Data);
	pLe->Evt = BT_HCI_EVT_LE_EXT_ADV_REPORT;
	uint8_t *body = reinterpret_cast<uint8_t *>(pLe->Data);
	body[0] = 1;                            // NbReport
	BtExtAdvReport_t *r = reinterpret_cast<BtExtAdvReport_t *>(body + 1);
	r->Type = static_cast<uint16_t>(DataStatus << 5);
	r->AddrType = 0;
	std::memcpy(r->Addr, Addr, 6);
	r->AdvSid = Sid;
	r->Rssi = -40;
	r->DataLen = DataLen;
	std::memcpy(r->Data, pData, DataLen);
	BtHciProcessEvent(pDev, pEvt);
}

void TestExtAdvReassembly()
{
	BtHciDevice_t dev = {};
	dev.ScanReport = CaptureScanReport;
	const uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };

	s_ScanReports = 0;
	s_LastScanLen = 0;

	// Fragment 1 (more data to come): buffered, not delivered.
	FeedExtAdvReport(&dev, addr, 7, 1, "AB", 2);
	CHECK(s_ScanReports == 0);

	// Fragment 2 (complete): delivers the reassembled "ABCD".
	FeedExtAdvReport(&dev, addr, 7, 0, "CD", 2);
	CHECK(s_ScanReports == 1);
	CHECK(s_LastScanLen == 4);
	CHECK(std::memcmp(s_LastScanData, "ABCD", 4) == 0);

	// A standalone complete report is delivered directly.
	s_ScanReports = 0;
	FeedExtAdvReport(&dev, addr, 7, 0, "XY", 2);
	CHECK(s_ScanReports == 1);
	CHECK(s_LastScanLen == 2);
	CHECK(std::memcmp(s_LastScanData, "XY", 2) == 0);

	// A truncated fragment (Data_Status 2) is not delivered as complete.
	s_ScanReports = 0;
	FeedExtAdvReport(&dev, addr, 7, 2, "ZZ", 2);
	CHECK(s_ScanReports == 0);
}

// Events shorter than the fixed structure their handler reads must be dropped.
// They stay inside the receive buffer, so this is not an out of bounds read;
// the handler would act on bytes left over from the previous packet.
void TestShortFixedEventsDropped()
{
	BtHciDevice_t dev = {};
	dev.SendCompleted = CaptureCompleted;

	alignas(4) uint8_t raw[sizeof(BtHciEvtPacketHdr_t) + 8] = {};
	BtHciEvtPacket_t *pEvt = reinterpret_cast<BtHciEvtPacket_t *>(raw);

	// Command Status: Status(1) + NbCmdPacket(1) + CmdCode(2) = 4.
	dev.CmdCredit = 7;
	dev.CmdOpCode = 0x201C;
	dev.CmdDone = false;
	pEvt->Hdr.Evt = BT_HCI_EVT_COMMAND_STATUS;
	pEvt->Hdr.Len = 3;					// one short
	BtHciEvtCmdStatus_t *pCs = reinterpret_cast<BtHciEvtCmdStatus_t *>(pEvt->Data);
	pCs->NbCmdPacket = 0x7F;
	pCs->CmdCode = 0x201C;
	BtHciProcessEvent(&dev, pEvt);
	CHECK(dev.CmdCredit == 7);			// credits untouched
	CHECK(!dev.CmdDone);

	// A full length Command Status still works.
	pEvt->Hdr.Len = sizeof(BtHciEvtCmdStatus_t);
	pCs->NbCmdPacket = 2;
	BtHciProcessEvent(&dev, pEvt);
	CHECK(dev.CmdCredit == 2);

	// Disconnection Complete: Status(1) + ConnHdl(2) + Reason(1) = 4.
	s_SmpDisconnectCount = 0;
	std::memset(raw, 0, sizeof(raw));
	pEvt->Hdr.Evt = BT_HCI_EVT_DISCONN_COMPLETE;
	pEvt->Hdr.Len = 3;					// one short
	BtHciProcessEvent(&dev, pEvt);
	CHECK(s_SmpDisconnectCount == 0);

	pEvt->Hdr.Len = sizeof(BtHciEvtDisconComplete_t);
	BtHciEvtDisconComplete_t *pDc =
		reinterpret_cast<BtHciEvtDisconComplete_t *>(pEvt->Data);
	pDc->ConnHdl = 0x0042;
	BtHciProcessEvent(&dev, pEvt);
	CHECK(s_SmpDisconnectCount == 1);

	// LE meta with no subevent byte at all.
	s_LtkRequestCount = 0;
	std::memset(raw, 0, sizeof(raw));
	pEvt->Hdr.Evt = BT_HCI_EVT_LE;
	pEvt->Hdr.Len = 0;
	BtHciProcessEvent(&dev, pEvt);
	CHECK(s_LtkRequestCount == 0);
}

} // namespace

extern "C" {

uint32_t BtAttProcessReq(uint16_t, BtAttReqRsp_t * const, int,
						 BtAttReqRsp_t * const)
{
	return 0;
}

void BtAttProcessRsp(uint16_t, BtAttReqRsp_t * const, int)
{
}

uint32_t BtL2CapProcessSignal(BtHciDevice_t * const, uint16_t,
							  BtL2CapPdu_t const * const, uint16_t,
							  BtL2CapPdu_t * const)
{
	return 0;
}

void BtProcessSmpData(BtHciDevice_t * const, uint16_t ConnHdl,
					  BtL2CapSmp_t * const pSmp, size_t Len)
{
	if (s_SmpPacketCount >= s_SmpPackets.size() ||
		Len > s_SmpPackets[s_SmpPacketCount].Data.size())
	{
		return;
	}

	SmpCapture &pkt = s_SmpPackets[s_SmpPacketCount++];
	pkt.ConnHdl = ConnHdl;
	pkt.Len = Len;
	std::memcpy(pkt.Data.data(), pSmp, Len);
}

void BtSmpProcessLtkRequest(BtHciDevice_t * const, uint16_t, uint64_t, uint16_t)
{
	++s_LtkRequestCount;
}

void BtSmpLocalPubKeyReady(BtHciDevice_t * const, uint8_t,
						   const uint8_t *, const uint8_t *)
{
}

void BtSmpDhKeyReady(BtHciDevice_t * const, uint8_t, const uint8_t *)
{
}

void BtSmpEncryptionChanged(BtHciDevice_t * const, uint16_t, uint8_t, uint8_t)
{
}

void BtSmpDisconnected(uint16_t ConnHdl)
{
	++s_SmpDisconnectCount;
	s_SmpDisconnectHdl = ConnHdl;
}

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
	TestCommandFraming();
	TestAclFragmentationAndCredits();
	TestAclFragmentSendFailure();
	TestAclCreditsReturnedOnDisconnect();
	TestAclCompletionCredits();
	TestCompletedPacketsBounded();
	TestCommandCreditsAndCompletion();
	TestCommandCompleteBounds();
	TestAclReassembly();
	TestInterleavedReassembly();
	TestMalformedFragmentsAndEvents();
	TestTruncatedLeMetaEvents();
	TestShortFixedEventsDropped();
	TestExtAdvReassembly();
	TestDisconnectFreesSmp();

	if (s_Failures != 0)
	{
		std::printf("Bluetooth HCI flow tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("Bluetooth HCI flow tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
