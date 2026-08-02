#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_smp.h"
#include "syslog.h"

#include "bt_compliance.h"
#include "bt_packet.h"
#include "bt_virtual_clock.h"
#include "bt_virtual_controller.h"
#include "bt_virtual_peer.h"

namespace {

using btcompliance::Context;
using btcompliance::PacketBuilder;
using btcompliance::Requirement;
using btcompliance::VirtualClock;
using btcompliance::VirtualController;
using btcompliance::VirtualPeer;

int s_CompletedCalls = 0;
uint16_t s_CompletedHandle = 0;
uint16_t s_CompletedCount = 0;
int s_ScanReports = 0;
size_t s_LastScanLength = 0;
uint8_t s_LastScanData[64] = {};
int s_AttRequests = 0;
uint16_t s_LastAttHandle = 0;
int s_LastAttLength = 0;
uint8_t s_LastAttOpcode = 0;
int s_SmpDisconnects = 0;

void ResetCaptures()
{
	s_CompletedCalls = 0;
	s_CompletedHandle = 0;
	s_CompletedCount = 0;
	s_ScanReports = 0;
	s_LastScanLength = 0;
	std::memset(s_LastScanData, 0, sizeof(s_LastScanData));
	s_AttRequests = 0;
	s_LastAttHandle = 0;
	s_LastAttLength = 0;
	s_LastAttOpcode = 0;
	s_SmpDisconnects = 0;
}

void CaptureCompleted(uint16_t ConnHdl, uint16_t Count)
{
	s_CompletedCalls++;
	s_CompletedHandle = ConnHdl;
	s_CompletedCount = Count;
}

bool CaptureScanReport(int8_t, uint8_t, uint8_t[6], size_t Length, uint8_t *pData)
{
	s_ScanReports++;
	s_LastScanLength = Length;
	if (Length <= sizeof(s_LastScanData) && Length != 0 && pData != nullptr)
	{
		std::memcpy(s_LastScanData, pData, Length);
	}
	return true;
}

struct ClockCapture {
	int Count;
	int Values[4];
};

struct ClockEvent {
	ClockCapture *pCapture;
	int Value;
};

void CaptureClockEvent(void *pContext)
{
	ClockEvent *pEvent = static_cast<ClockEvent *>(pContext);
	if (pEvent != nullptr && pEvent->pCapture != nullptr && pEvent->pCapture->Count < 4)
	{
		pEvent->pCapture->Values[pEvent->pCapture->Count++] = pEvent->Value;
	}
}

void BuildAcl(BtHciACLDataPacket_t *pAcl, uint16_t ConnHdl,
	const uint8_t *pData, uint16_t Length)
{
	pAcl->Hdr.ConnHdl = ConnHdl;
	pAcl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
	pAcl->Hdr.Len = Length;
	if (Length != 0)
	{
		std::memcpy(pAcl->Data, pData, Length);
	}
}

void TestVirtualTime(Context &ctx)
{
	static const Requirement req = {
		"HARNESS-TIME-BV-01", "IOsonata Bluetooth tests", "Virtual time",
		"events run only when time advances and equal deadlines preserve insertion order"
	};
	ctx.Begin(req);

	VirtualClock clock;
	ClockCapture capture = {};
	ClockEvent events[3] = {
		{ &capture, 1 }, { &capture, 2 }, { &capture, 3 }
	};
	BT_CHECK(ctx, clock.ScheduleUs(20, CaptureClockEvent, &events[0]));
	BT_CHECK(ctx, clock.ScheduleUs(10, CaptureClockEvent, &events[1]));
	BT_CHECK(ctx, clock.ScheduleUs(10, CaptureClockEvent, &events[2]));
	BT_CHECK(ctx, clock.PendingCount() == 3);
	clock.AdvanceUs(9);
	BT_CHECK(ctx, capture.Count == 0);
	clock.AdvanceUs(1);
	BT_CHECK(ctx, capture.Count == 2);
	BT_CHECK(ctx, capture.Values[0] == 2);
	BT_CHECK(ctx, capture.Values[1] == 3);
	clock.AdvanceUs(10);
	BT_CHECK(ctx, capture.Count == 3);
	BT_CHECK(ctx, capture.Values[2] == 1);
	BT_CHECK(ctx, clock.PendingCount() == 0);
	ctx.End();
}

void TestControllerCommandBoundary(Context &ctx)
{
	static const Requirement req = {
		"HCI-CMD-BV-01", "Core Vol 4 Part E", "5.4.1",
		"controller command status, parameters and bounded return data cross the hardware boundary"
	};
	ctx.Begin(req);

	VirtualClock clock;
	VirtualController controller(clock);
	BtHciDevice_t host = {};
	BT_CHECK(ctx, controller.Attach(&host));

	const uint8_t response[] = { 0x11, 0x22, 0x33 };
	const uint8_t parameters[] = { 0x44, 0x55 };
	uint8_t returned[2] = {};
	controller.SetCommandResponse(0x201C, 0x0C, response, sizeof(response));
	BT_CHECK(ctx, BtHciCommand(&host, 0x201C, parameters, sizeof(parameters),
		returned, sizeof(returned)) == 0x0C);
	BT_CHECK(ctx, controller.CommandCount() == 1);
	const VirtualController::CommandRecord *pCommand = controller.CommandAt(0);
	BT_CHECK(ctx, pCommand != nullptr);
	if (pCommand != nullptr)
	{
		BT_CHECK(ctx, pCommand->Opcode == 0x201C);
		BT_CHECK(ctx, pCommand->ParamLen == sizeof(parameters));
		BT_CHECK(ctx, std::memcmp(pCommand->Param, parameters, sizeof(parameters)) == 0);
	}
	BT_CHECK(ctx, returned[0] == 0x11);
	BT_CHECK(ctx, returned[1] == 0x22);
	ctx.End();
}

void TestAclFragmentationAndCredits(Context &ctx)
{
	static const Requirement req = {
		"HCI-ACL-FLOW-BV-01", "Core Vol 4 Part E", "5.4.2 and 4.1.1",
		"ACL fragmentation consumes all required credits and completed packets restore them"
	};
	ctx.Begin(req);

	ResetCaptures();
	VirtualClock clock;
	VirtualController controller(clock);
	BtHciDevice_t host = {};
	host.SendCompleted = CaptureCompleted;
	BT_CHECK(ctx, controller.Attach(&host));
	BtHciSetLeAclBuffer(&host, 10, 3);

	alignas(4) uint8_t raw[64] = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw);
	uint8_t payload[25];
	for (size_t i = 0; i < sizeof(payload); i++)
	{
		payload[i] = static_cast<uint8_t>(i);
	}
	BuildAcl(pAcl, 0x0123, payload, sizeof(payload));

	BT_CHECK(ctx, BtHciSendAcl(&host, pAcl) == sizeof(payload) + sizeof(pAcl->Hdr));
	BT_CHECK(ctx, controller.AclCount() == 3);
	BT_CHECK(ctx, host.AclCredit == 0);

	const uint16_t expectedLength[3] = { 10, 10, 5 };
	uint16_t offset = 0;
	for (size_t i = 0; i < 3; i++)
	{
		const VirtualController::PacketRecord *pRecord = controller.AclAt(i);
		BT_CHECK(ctx, pRecord != nullptr);
		if (pRecord == nullptr)
		{
			continue;
		}
		const BtHciACLDataPacket_t *pFragment =
			reinterpret_cast<const BtHciACLDataPacket_t *>(pRecord->Data);
		BT_CHECK(ctx, pFragment->Hdr.ConnHdl == 0x0123);
		BT_CHECK(ctx, pFragment->Hdr.Len == expectedLength[i]);
		BT_CHECK(ctx, pFragment->Hdr.PBFlag ==
			(i == 0 ? BT_HCI_PBFLAG_START_NONFLUSHABLE : BT_HCI_PBFLAG_CONTINUING_FRAGMENT));
		BT_CHECK(ctx, std::memcmp(pFragment->Data, payload + offset, expectedLength[i]) == 0);
		offset = static_cast<uint16_t>(offset + expectedLength[i]);
	}

	BT_CHECK(ctx, controller.CompletePackets(0x0123, 2));
	BT_CHECK(ctx, host.AclCredit == 2);
	BT_CHECK(ctx, s_CompletedCalls == 1);
	BT_CHECK(ctx, s_CompletedHandle == 0x0123);
	BT_CHECK(ctx, s_CompletedCount == 2);
	BT_CHECK(ctx, controller.CompletePackets(0x0123, 5));
	BT_CHECK(ctx, host.AclCredit == 3);
	ctx.End();
}

void TestAclTransportFailure(Context &ctx)
{
	static const Requirement req = {
		"HCI-ACL-FAIL-BI-01", "Core Vol 4 Part E", "5.4.2",
		"transport refusal restores unsent credits and drops a link only after a start fragment escaped"
	};
	ctx.Begin(req);

	VirtualClock clock;
	VirtualController controller(clock);
	BtHciDevice_t host = {};
	BT_CHECK(ctx, controller.Attach(&host));

	alignas(4) uint8_t raw[64] = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw);
	uint8_t payload[25] = {};
	BuildAcl(pAcl, 0x0345, payload, sizeof(payload));

	BtHciSetLeAclBuffer(&host, 10, 8);
	controller.FailAclOnCall(1);
	BT_CHECK(ctx, BtHciSendAcl(&host, pAcl) == 0);
	BT_CHECK(ctx, controller.AclCount() == 0);
	BT_CHECK(ctx, controller.CommandCount() == 0);
	BT_CHECK(ctx, host.AclCredit == 8);

	controller.Reset();
	BtHciSetLeAclBuffer(&host, 10, 8);
	controller.FailAclOnCall(2);
	BT_CHECK(ctx, BtHciSendAcl(&host, pAcl) == 0);
	BT_CHECK(ctx, controller.AclCount() == 1);
	BT_CHECK(ctx, host.AclCredit == 7);
	BT_CHECK(ctx, controller.CommandCount() == 1);
	const VirtualController::CommandRecord *pCommand = controller.CommandAt(0);
	BT_CHECK(ctx, pCommand != nullptr);
	if (pCommand != nullptr)
	{
		BT_CHECK(ctx, pCommand->Opcode == BT_HCI_CMD_LINKCTRL_DISCONNECT);
		BT_CHECK(ctx, pCommand->ParamLen == 3);
		BT_CHECK(ctx, pCommand->Param[0] == 0x45);
		BT_CHECK(ctx, pCommand->Param[1] == 0x03);
		BT_CHECK(ctx, pCommand->Param[2] == 0x13);
	}
	ctx.End();
}

void TestCompletedPacketListBounds(Context &ctx)
{
	static const Requirement req = {
		"HCI-EVT-NCP-BI-01", "Core Vol 4 Part E", "7.7.19",
		"Number Of Completed Packets never walks beyond the entries carried by the event"
	};
	ctx.Begin(req);

	ResetCaptures();
	VirtualClock clock;
	VirtualController controller(clock);
	BtHciDevice_t host = {};
	host.SendCompleted = CaptureCompleted;
	BT_CHECK(ctx, controller.Attach(&host));
	BtHciSetLeAclBuffer(&host, 20, 4);
	host.AclCredit = 1;

	uint8_t malformed[5] = { 2, 0x21, 0x00, 0x02, 0x00 };
	BT_CHECK(ctx, controller.InjectEvent(BT_HCI_EVT_NB_COMPLETED_PACKET,
		malformed, sizeof(malformed)));
	BT_CHECK(ctx, s_CompletedCalls == 1);
	BT_CHECK(ctx, s_CompletedHandle == 0x0021);
	BT_CHECK(ctx, s_CompletedCount == 2);
	BT_CHECK(ctx, host.AclCredit == 3);

	uint8_t emptyList = 0;
	BT_CHECK(ctx, controller.InjectEvent(BT_HCI_EVT_NB_COMPLETED_PACKET,
		&emptyList, sizeof(emptyList)));
	BT_CHECK(ctx, s_CompletedCalls == 1);
	ctx.End();
}

void TestCommandEventBoundsAndTiming(Context &ctx)
{
	static const Requirement req = {
		"HCI-EVT-CMD-BI-01", "Core Vol 4 Part E", "7.7.14 and 7.7.15",
		"short command events are ignored and a delayed matching completion updates only at delivery time"
	};
	ctx.Begin(req);

	VirtualClock clock;
	VirtualController controller(clock);
	BtHciDevice_t host = {};
	BT_CHECK(ctx, controller.Attach(&host));

	uint8_t returned[2] = { 0, 0 };
	host.CmdCredit = 7;
	host.CmdOpCode = 0x201C;
	host.CmdStatus = 0xEE;
	host.CmdRetLen = sizeof(returned);
	host.pCmdRet = returned;
	host.CmdDone = false;

	uint8_t tooShort[2] = { 0x7F, 0x1C };
	BT_CHECK(ctx, controller.InjectEvent(BT_HCI_EVT_COMMAND_COMPLETE,
		tooShort, sizeof(tooShort)));
	BT_CHECK(ctx, host.CmdCredit == 7);
	BT_CHECK(ctx, !host.CmdDone);
	BT_CHECK(ctx, host.CmdStatus == 0xEE);

	uint8_t complete[6] = { 3, 0x1C, 0x20, 0x00, 0xA5, 0x5A };
	BT_CHECK(ctx, controller.ScheduleEventUs(100, BT_HCI_EVT_COMMAND_COMPLETE,
		complete, sizeof(complete)));
	clock.AdvanceUs(99);
	BT_CHECK(ctx, !host.CmdDone);
	BT_CHECK(ctx, host.CmdCredit == 7);
	clock.AdvanceUs(1);
	BT_CHECK(ctx, host.CmdDone);
	BT_CHECK(ctx, host.CmdCredit == 3);
	BT_CHECK(ctx, host.CmdStatus == 0);
	BT_CHECK(ctx, returned[0] == 0xA5);
	BT_CHECK(ctx, returned[1] == 0x5A);
	ctx.End();
}

void TestAdvertisingReportBounds(Context &ctx)
{
	static const Requirement req = {
		"HCI-LE-ADV-BI-01", "Core Vol 4 Part E", "7.7.65.2 and 7.7.65.13",
		"legacy and extended advertising report lists reject missing and truncated records"
	};
	ctx.Begin(req);

	ResetCaptures();
	VirtualClock clock;
	VirtualController controller(clock);
	BtHciDevice_t host = {};
	host.ScanReport = CaptureScanReport;
	BT_CHECK(ctx, controller.Attach(&host));

	uint8_t oneReport = 1;
	BT_CHECK(ctx, controller.InjectLeEvent(BT_HCI_EVT_LE_ADV_REPORT,
		&oneReport, sizeof(oneReport)));
	BT_CHECK(ctx, s_ScanReports == 0);
	BT_CHECK(ctx, controller.InjectLeEvent(BT_HCI_EVT_LE_EXT_ADV_REPORT,
		&oneReport, sizeof(oneReport)));
	BT_CHECK(ctx, s_ScanReports == 0);

	uint8_t malformedLegacy[12] = {};
	PacketBuilder malformed(malformedLegacy, sizeof(malformedLegacy));
	malformed.U8(1);
	malformed.U8(0);
	malformed.U8(0);
	uint8_t address[6] = { 1, 2, 3, 4, 5, 6 };
	malformed.Bytes(address, sizeof(address));
	malformed.U8(4);
	malformed.U8(0xAA);
	malformed.U8(0xBB);
	BT_CHECK(ctx, controller.InjectLeEvent(BT_HCI_EVT_LE_ADV_REPORT,
		malformed.Data(), static_cast<uint8_t>(malformed.Length())));
	BT_CHECK(ctx, s_ScanReports == 0);

	uint8_t validLegacy[16] = {};
	PacketBuilder valid(validLegacy, sizeof(validLegacy));
	valid.U8(1);
	valid.U8(0);
	valid.U8(0);
	valid.Bytes(address, sizeof(address));
	valid.U8(3);
	const uint8_t advData[3] = { 0x02, 0x01, 0x06 };
	valid.Bytes(advData, sizeof(advData));
	valid.U8(static_cast<uint8_t>(-40));
	BT_CHECK(ctx, valid.Valid());
	BT_CHECK(ctx, controller.InjectLeEvent(BT_HCI_EVT_LE_ADV_REPORT,
		valid.Data(), static_cast<uint8_t>(valid.Length())));
	BT_CHECK(ctx, s_ScanReports == 1);
	BT_CHECK(ctx, s_LastScanLength == sizeof(advData));
	BT_CHECK(ctx, std::memcmp(s_LastScanData, advData, sizeof(advData)) == 0);
	ctx.End();
}

void TestPeerReassemblyAndPoolExhaustion(Context &ctx)
{
	static const Requirement req = {
		"HCI-ACL-RX-BI-01", "Core Vol 4 Part E", "5.4.2",
		"orphan continuations are dropped and bounded per-link reassembly cannot cross-contaminate links"
	};
	ctx.Begin(req);

	ResetCaptures();
	VirtualClock clock;
	VirtualController controller(clock);
	BtHciDevice_t host = {};
	BT_CHECK(ctx, controller.Attach(&host));
	VirtualPeer peer(controller, 1);

	uint8_t opcode = BT_ATT_OPCODE_ATT_READ_REQ;
	BT_CHECK(ctx, peer.SendOrphanContinuation(&opcode, 1));
	BT_CHECK(ctx, s_AttRequests == 0);
	BT_CHECK(ctx, peer.SendAtt(&opcode, 1, sizeof(BtL2CapHdr_t)));
	BT_CHECK(ctx, s_AttRequests == 1);
	BT_CHECK(ctx, s_LastAttHandle == 1);
	BT_CHECK(ctx, s_LastAttLength == 1);
	BT_CHECK(ctx, s_LastAttOpcode == opcode);

	s_AttRequests = 0;
	uint8_t l2a[5] = { 1, 0, static_cast<uint8_t>(BT_L2CAP_CID_ATT & 0xFF),
		static_cast<uint8_t>(BT_L2CAP_CID_ATT >> 8), BT_ATT_OPCODE_ATT_READ_REQ };
	uint8_t l2b[5] = { 1, 0, static_cast<uint8_t>(BT_L2CAP_CID_ATT & 0xFF),
		static_cast<uint8_t>(BT_L2CAP_CID_ATT >> 8), BT_ATT_OPCODE_ATT_WRITE_REQ };
	BT_CHECK(ctx, controller.InjectAcl(10, BT_HCI_PBFLAG_START_NONFLUSHABLE, l2a, 4));
	BT_CHECK(ctx, controller.InjectAcl(11, BT_HCI_PBFLAG_START_NONFLUSHABLE, l2b, 4));
	BT_CHECK(ctx, controller.InjectAcl(12, BT_HCI_PBFLAG_START_NONFLUSHABLE, l2a, 4));
	BT_CHECK(ctx, controller.InjectAcl(12, BT_HCI_PBFLAG_CONTINUING_FRAGMENT, l2a + 4, 1));
	BT_CHECK(ctx, s_AttRequests == 0);
	BT_CHECK(ctx, controller.InjectAcl(10, BT_HCI_PBFLAG_CONTINUING_FRAGMENT, l2a + 4, 1));
	BT_CHECK(ctx, controller.InjectAcl(11, BT_HCI_PBFLAG_CONTINUING_FRAGMENT, l2b + 4, 1));
	BT_CHECK(ctx, s_AttRequests == 2);
	ctx.End();
}

void TestPeerReadsFragmentedHostOutput(Context &ctx)
{
	static const Requirement req = {
		"HARNESS-PEER-BV-01", "IOsonata Bluetooth tests", "Virtual peer",
		"virtual peer independently reassembles fragmented host ACL output into one L2CAP PDU"
	};
	ctx.Begin(req);

	VirtualClock clock;
	VirtualController controller(clock);
	BtHciDevice_t host = {};
	BT_CHECK(ctx, controller.Attach(&host));
	BtHciSetLeAclBuffer(&host, 5, 4);
	VirtualPeer peer(controller, 0x0044);

	alignas(4) uint8_t raw[32] = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw);
	uint8_t l2pdu[7] = { 3, 0, static_cast<uint8_t>(BT_L2CAP_CID_ATT & 0xFF),
		static_cast<uint8_t>(BT_L2CAP_CID_ATT >> 8), 0x0A, 0x01, 0x00 };
	BuildAcl(pAcl, 0x0044, l2pdu, sizeof(l2pdu));
	BT_CHECK(ctx, BtHciSendAcl(&host, pAcl) == sizeof(l2pdu) + sizeof(pAcl->Hdr));
	BT_CHECK(ctx, controller.AclCount() == 2);

	uint16_t cid = 0;
	uint8_t payload[8] = {};
	size_t payloadLen = 0;
	BT_CHECK(ctx, peer.ReadHostL2cap(&cid, payload, sizeof(payload), &payloadLen));
	BT_CHECK(ctx, cid == BT_L2CAP_CID_ATT);
	BT_CHECK(ctx, payloadLen == 3);
	BT_CHECK(ctx, std::memcmp(payload, l2pdu + sizeof(BtL2CapHdr_t), 3) == 0);
	ctx.End();
}

void RecordKnownGaps(Context &ctx)
{
	static const Requirement zeroCount = {
		"HCI-EVT-COUNT-BI-00", "Core Vol 4 Part E", "7.7.19 and 7.7.65",
		"a variable-list event proves its count byte exists before reading it"
	};
	ctx.Begin(zeroCount);
	ctx.Skip("known open finding: exact header-only event still reaches a count read before the length guard");
	ctx.End();

	static const Requirement extAdvDrop = {
		"HCI-EXTADV-REASM-BI-01", "Core Vol 4 Part E", "7.7.65.13",
		"a final extended advertising fragment is not delivered after its earlier fragment was dropped"
	};
	ctx.Begin(extAdvDrop);
	ctx.Skip("known open finding: reassembly-pool exhaustion does not retain a dropped-fragment marker");
	ctx.End();

	static const Requirement controllerInit = {
		"HCI-CTLR-INIT-BI-01", "IOsonata controller port", "initialization",
		"controller enable fails when target controller initialization fails"
	};
	ctx.Begin(controllerInit);
	ctx.Skip("requires the target-port hardware stub layer; generic virtual HCI does not compile vendor SDK ports yet");
	ctx.End();
}

} // namespace

extern "C" {

uint32_t BtAttProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pReq,
	int ReqLen, BtAttReqRsp_t * const)
{
	s_AttRequests++;
	s_LastAttHandle = ConnHdl;
	s_LastAttLength = ReqLen;
	s_LastAttOpcode = ReqLen > 0 && pReq != nullptr ? pReq->OpCode : 0;
	return 0;
}

void BtAttProcessRsp(uint16_t, BtAttReqRsp_t * const, int)
{
}

uint32_t BtL2CapProcessSignal(BtHciDevice_t * const, uint16_t,
	BtL2CapPdu_t const * const, uint16_t, BtL2CapPdu_t * const)
{
	return 0;
}

void BtProcessSmpData(BtHciDevice_t * const, uint16_t,
	BtL2CapSmp_t * const, size_t)
{
}

void BtSmpProcessLtkRequest(BtHciDevice_t * const, uint16_t, uint64_t, uint16_t)
{
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

void BtSmpDisconnected(uint16_t)
{
	s_SmpDisconnects++;
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
	Context ctx("Bluetooth HCI compliance foundation");
	TestVirtualTime(ctx);
	TestControllerCommandBoundary(ctx);
	TestAclFragmentationAndCredits(ctx);
	TestAclTransportFailure(ctx);
	TestCompletedPacketListBounds(ctx);
	TestCommandEventBoundsAndTiming(ctx);
	TestAdvertisingReportBounds(ctx);
	TestPeerReassemblyAndPoolExhaustion(ctx);
	TestPeerReadsFragmentedHostOutput(ctx);
	RecordKnownGaps(ctx);

	const bool strict = std::getenv("BT_COMPLIANCE_STRICT") != nullptr;
	return ctx.Finish(strict);
}
