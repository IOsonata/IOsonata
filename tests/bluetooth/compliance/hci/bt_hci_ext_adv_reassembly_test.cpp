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
	return ctx.Finish(false);
}
