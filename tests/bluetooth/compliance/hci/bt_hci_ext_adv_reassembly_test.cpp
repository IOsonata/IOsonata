#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_smp.h"
#include "syslog.h"

#include "bt_compliance.h"
#include "bt_virtual_clock.h"
#include "bt_virtual_controller.h"

namespace {

using btcompliance::Context;
using btcompliance::Requirement;
using btcompliance::VirtualClock;
using btcompliance::VirtualController;

static_assert(BT_EXT_ADV_REASSEMBLY_MAX >= 1650,
	"Core extended advertising reassembly must support 1650 octets");

int s_ScanReports;
size_t s_ScanLength;
uint8_t s_ScanData[1650];

bool CaptureScanReport(int8_t, uint8_t, uint8_t[6], size_t Length, uint8_t *pData)
{
	s_ScanReports++;
	s_ScanLength = Length;
	if (pData != nullptr && Length <= sizeof(s_ScanData))
	{
		std::memcpy(s_ScanData, pData, Length);
	}
	return true;
}

bool FeedExtAdv(VirtualController &controller, const uint8_t Address[6],
	uint8_t Sid, uint8_t DataStatus, const uint8_t *pData, uint8_t DataLen)
{
	// LE Meta subevent byte is added by InjectLeEvent. The payload here is
	// Num_Reports (1) + fixed report fields (24) + DataLen. Keep DataLen <=220
	// so the complete HCI event remains well inside the 255-byte parameter limit.
	uint8_t parameters[1 + 24 + 220] = {};
	if (DataLen > 220 || (DataLen != 0 && pData == nullptr))
	{
		return false;
	}

	parameters[0] = 1;
	BtExtAdvReport_t *pReport =
		reinterpret_cast<BtExtAdvReport_t *>(parameters + 1);
	pReport->Type = static_cast<uint16_t>(DataStatus << 5);
	pReport->AddrType = 0;
	std::memcpy(pReport->Addr, Address, 6);
	pReport->AdvSid = Sid;
	pReport->Rssi = -30;
	pReport->DataLen = DataLen;
	if (DataLen != 0)
	{
		std::memcpy(pReport->Data, pData, DataLen);
	}

	return controller.InjectLeEvent(BT_HCI_EVT_LE_EXT_ADV_REPORT,
		parameters, static_cast<uint8_t>(1 + 24 + DataLen));
}

void TestCoreMaximumExtendedAdvertisingReassembly(Context &ctx)
{
	static const Requirement req = {
		"HCI-EXTADV-REASM-BV-02", "Core Vol 4 Part E", "7.7.65.13",
		"the host reassembles the full 1650-octet extended advertising data limit across bounded HCI events"
	};
	ctx.Begin(req);

	s_ScanReports = 0;
	s_ScanLength = 0;
	std::memset(s_ScanData, 0, sizeof(s_ScanData));

	VirtualClock clock;
	BtHciDevice_t host = {};
	host.ScanReport = CaptureScanReport;
	VirtualController controller(clock);
	BT_CHECK(ctx, controller.Attach(&host));

	const uint8_t addr[6] = { 0xA5, 1, 2, 3, 4, 5 };
	uint8_t fragment[220];
	size_t offset = 0;

	for (int n = 0; n < 7; n++)
	{
		for (size_t i = 0; i < sizeof(fragment); i++)
		{
			fragment[i] = static_cast<uint8_t>((offset + i) & 0xFFU);
		}
		BT_CHECK(ctx, FeedExtAdv(controller, addr, 7, 1,
			fragment, sizeof(fragment)));
		offset += sizeof(fragment);
		BT_CHECK(ctx, s_ScanReports == 0);
	}

	uint8_t last[110];
	for (size_t i = 0; i < sizeof(last); i++)
	{
		last[i] = static_cast<uint8_t>((offset + i) & 0xFFU);
	}
	BT_CHECK(ctx, FeedExtAdv(controller, addr, 7, 0, last, sizeof(last)));

	BT_CHECK(ctx, offset + sizeof(last) == 1650);
	BT_CHECK(ctx, s_ScanReports == 1);
	BT_CHECK(ctx, s_ScanLength == 1650);
	for (size_t i = 0; i < sizeof(s_ScanData); i++)
	{
		BT_CHECK(ctx, s_ScanData[i] == static_cast<uint8_t>(i & 0xFFU));
	}

	controller.Detach();
	ctx.End();
}

// An advertiser that starts a chain and goes away sends no terminating
// fragment, so its slot is never released by the chain that claimed it. With
// two slots, two such advertisers used to pin the table for the rest of the
// session: every fragmented report from every other advertiser was abandoned,
// and the dropped table then swallowed one standalone report from each of them
// too. Short reports kept arriving, so scanning looked healthy.
void TestAbandonedChainsDoNotStarveTheTable(Context &ctx)
{
	static const Requirement req = {
		"HCI-EXTADV-REASM-BV-03", "Core Vol 4 Part E", "7.7.65.13",
		"an advertiser that abandons a fragmented report does not stop the host reassembling any other advertiser's"
	};
	ctx.Begin(req);

	s_ScanReports = 0;
	s_ScanLength = 0;

	VirtualClock clock;
	BtHciDevice_t host = {};
	host.ScanReport = CaptureScanReport;
	VirtualController controller(clock);
	BT_CHECK(ctx, controller.Attach(&host));
	BtHciExtAdvReasmReset();

	const uint8_t abandonedA[6] = { 0xA1, 1, 2, 3, 4, 5 };
	const uint8_t abandonedB[6] = { 0xB2, 1, 2, 3, 4, 5 };
	const uint8_t good[6]       = { 0xC3, 1, 2, 3, 4, 5 };
	const uint8_t head[4] = { 0x11, 0x22, 0x33, 0x44 };
	const uint8_t tail[3] = { 0x55, 0x66, 0x77 };

	// Two advertisers each start a chain and send nothing more.
	BT_CHECK(ctx, FeedExtAdv(controller, abandonedA, 1, 1, head, sizeof(head)));
	BT_CHECK(ctx, FeedExtAdv(controller, abandonedB, 2, 1, head, sizeof(head)));
	BT_CHECK(ctx, s_ScanReports == 0);

	// A third advertiser now has to be able to reassemble.
	BT_CHECK(ctx, FeedExtAdv(controller, good, 3, 1, head, sizeof(head)));
	BT_CHECK(ctx, s_ScanReports == 0);
	BT_CHECK(ctx, FeedExtAdv(controller, good, 3, 0, tail, sizeof(tail)));
	BT_CHECK(ctx, s_ScanReports == 1);
	BT_CHECK(ctx, s_ScanLength == sizeof(head) + sizeof(tail));
	BT_CHECK(ctx, s_ScanData[0] == 0x11);
	BT_CHECK(ctx, s_ScanData[4] == 0x55);
	BT_CHECK(ctx, s_ScanData[6] == 0x77);

	// And so does a fourth, and a fifth, for as long as the scan runs.
	for (int n = 0; n < 8; n++)
	{
		uint8_t addr[6] = { 0xD0, 1, 2, 3, 4, (uint8_t)n };
		BT_CHECK(ctx, FeedExtAdv(controller, addr, 4, 1, head, sizeof(head)));
		BT_CHECK(ctx, FeedExtAdv(controller, addr, 4, 0, tail, sizeof(tail)));
		BT_CHECK(ctx, s_ScanReports == 2 + n);
		BT_CHECK(ctx, s_ScanLength == sizeof(head) + sizeof(tail));
	}

	controller.Detach();
	ctx.End();
}

// Reclaiming a slot must not turn the abandoned advertiser's next fragment
// into a report: the terminating fragment of a chain whose head was discarded
// looks exactly like a standalone complete report on the wire.
void TestReclaimedChainTailIsNotDelivered(Context &ctx)
{
	static const Requirement req = {
		"HCI-EXTADV-REASM-BV-04", "Core Vol 4 Part E", "7.7.65.13",
		"the tail of a reassembly the host gave up on is not delivered as whole advertising data"
	};
	ctx.Begin(req);

	s_ScanReports = 0;

	VirtualClock clock;
	BtHciDevice_t host = {};
	host.ScanReport = CaptureScanReport;
	VirtualController controller(clock);
	BT_CHECK(ctx, controller.Attach(&host));
	BtHciExtAdvReasmReset();

	const uint8_t old[6] = { 0xE1, 1, 2, 3, 4, 5 };
	const uint8_t head[4] = { 0x11, 0x22, 0x33, 0x44 };
	const uint8_t tail[3] = { 0x55, 0x66, 0x77 };

	BT_CHECK(ctx, FeedExtAdv(controller, old, 1, 1, head, sizeof(head)));

	// Two newer chains take both slots, so the first advertiser loses its.
	for (int n = 0; n < 2; n++)
	{
		uint8_t addr[6] = { 0xF0, 1, 2, 3, 4, (uint8_t)n };
		BT_CHECK(ctx, FeedExtAdv(controller, addr, 2, 1, head, sizeof(head)));
	}
	BT_CHECK(ctx, s_ScanReports == 0);

	// The first advertiser's terminating fragment now arrives. Its head is
	// gone, so what is left is a suffix and must not reach the application.
	BT_CHECK(ctx, FeedExtAdv(controller, old, 1, 0, tail, sizeof(tail)));
	BT_CHECK(ctx, s_ScanReports == 0);

	// Once that record is spent the advertiser is ordinary again.
	BT_CHECK(ctx, FeedExtAdv(controller, old, 1, 0, tail, sizeof(tail)));
	BT_CHECK(ctx, s_ScanReports == 1);
	BT_CHECK(ctx, s_ScanLength == sizeof(tail));

	controller.Detach();
	ctx.End();
}

// Reclaim takes the chain that stopped, not the one still arriving.
void TestActiveChainSurvivesReclaim(Context &ctx)
{
	static const Requirement req = {
		"HCI-EXTADV-REASM-BV-05", "Core Vol 4 Part E", "7.7.65.13",
		"a fragmented report still receiving fragments is not the one the host gives up on"
	};
	ctx.Begin(req);

	s_ScanReports = 0;

	VirtualClock clock;
	BtHciDevice_t host = {};
	host.ScanReport = CaptureScanReport;
	VirtualController controller(clock);
	BT_CHECK(ctx, controller.Attach(&host));
	BtHciExtAdvReasmReset();

	const uint8_t stalled[6] = { 0x51, 1, 2, 3, 4, 5 };
	const uint8_t running[6] = { 0x52, 1, 2, 3, 4, 5 };
	const uint8_t head[4] = { 0x11, 0x22, 0x33, 0x44 };
	const uint8_t more[2] = { 0xAA, 0xBB };
	const uint8_t tail[3] = { 0x55, 0x66, 0x77 };

	BT_CHECK(ctx, FeedExtAdv(controller, stalled, 1, 1, head, sizeof(head)));
	BT_CHECK(ctx, FeedExtAdv(controller, running, 2, 1, head, sizeof(head)));

	// The second advertiser keeps sending, so it is the newer of the two.
	for (int n = 0; n < 3; n++)
	{
		BT_CHECK(ctx, FeedExtAdv(controller, running, 2, 1, more, sizeof(more)));
	}

	// A third chain needs a slot. The stalled one has to be what goes.
	const uint8_t third[6] = { 0x53, 1, 2, 3, 4, 5 };
	BT_CHECK(ctx, FeedExtAdv(controller, third, 3, 1, head, sizeof(head)));

	// The running chain finishes whole.
	BT_CHECK(ctx, FeedExtAdv(controller, running, 2, 0, tail, sizeof(tail)));
	BT_CHECK(ctx, s_ScanReports == 1);
	BT_CHECK(ctx, s_ScanLength ==
		sizeof(head) + 3 * sizeof(more) + sizeof(tail));
	BT_CHECK(ctx, s_ScanData[0] == 0x11);
	BT_CHECK(ctx, s_ScanData[4] == 0xAA);
	BT_CHECK(ctx, s_ScanData[10] == 0x55);

	controller.Detach();
	ctx.End();
}

// Scanning is the only thing that produces fragments, so a chain cannot
// continue across a gap in it.
void TestResetDiscardsPartialReports(Context &ctx)
{
	static const Requirement req = {
		"HCI-EXTADV-REASM-BV-06", "Core Vol 4 Part E", "7.7.65.13",
		"a fragmented report interrupted by scanning stopping is discarded rather than held"
	};
	ctx.Begin(req);

	s_ScanReports = 0;

	VirtualClock clock;
	BtHciDevice_t host = {};
	host.ScanReport = CaptureScanReport;
	VirtualController controller(clock);
	BT_CHECK(ctx, controller.Attach(&host));
	BtHciExtAdvReasmReset();

	const uint8_t addr[6] = { 0x61, 1, 2, 3, 4, 5 };
	const uint8_t head[4] = { 0x11, 0x22, 0x33, 0x44 };
	const uint8_t tail[3] = { 0x55, 0x66, 0x77 };

	BT_CHECK(ctx, FeedExtAdv(controller, addr, 1, 1, head, sizeof(head)));

	// What BtGapScanStop and BtGapScanStart both do.
	BtHciExtAdvReasmReset();

	// The old head is gone, so the fragment that would have terminated it is
	// a standalone report and nothing from before the gap is prepended.
	BT_CHECK(ctx, FeedExtAdv(controller, addr, 1, 0, tail, sizeof(tail)));
	BT_CHECK(ctx, s_ScanReports == 1);
	BT_CHECK(ctx, s_ScanLength == sizeof(tail));
	BT_CHECK(ctx, s_ScanData[0] == 0x55);

	// And the record of an abandoned advertiser does not survive either, so
	// it cannot swallow a report in the next scan.
	s_ScanReports = 0;
	BT_CHECK(ctx, FeedExtAdv(controller, addr, 1, 1, head, sizeof(head)));
	for (int n = 0; n < 2; n++)
	{
		uint8_t other[6] = { 0x70, 1, 2, 3, 4, (uint8_t)n };
		BT_CHECK(ctx, FeedExtAdv(controller, other, 2, 1, head, sizeof(head)));
	}
	BtHciExtAdvReasmReset();
	BT_CHECK(ctx, FeedExtAdv(controller, addr, 1, 0, tail, sizeof(tail)));
	BT_CHECK(ctx, s_ScanReports == 1);
	BT_CHECK(ctx, s_ScanLength == sizeof(tail));

	controller.Detach();
	ctx.End();
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
	Context ctx("Bluetooth extended advertising RX compliance");
	TestCoreMaximumExtendedAdvertisingReassembly(ctx);
	TestAbandonedChainsDoNotStarveTheTable(ctx);
	TestReclaimedChainTailIsNotDelivered(ctx);
	TestActiveChainSurvivesReclaim(ctx);
	TestResetDiscardsPartialReports(ctx);
	return ctx.Finish(false);
}
