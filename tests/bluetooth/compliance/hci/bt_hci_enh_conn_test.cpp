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

int s_EnhancedCount;
uint16_t s_ConnHdl;
uint8_t s_Role;
uint8_t s_IdentityType;
uint8_t s_IdentityAddr[6];
uint8_t s_LocalRpa[6];
uint8_t s_PeerRpa[6];
int s_LegacyCount;
uint8_t s_LegacyType;
uint8_t s_LegacyAddr[6];

void CaptureEnhanced(uint16_t ConnHdl, uint8_t Role,
	uint8_t PeerIdentityAddrType, const uint8_t PeerIdentityAddr[6],
	const uint8_t LocalRpa[6], const uint8_t PeerRpa[6])
{
	s_EnhancedCount++;
	s_ConnHdl = ConnHdl;
	s_Role = Role;
	s_IdentityType = PeerIdentityAddrType;
	std::memcpy(s_IdentityAddr, PeerIdentityAddr, sizeof(s_IdentityAddr));
	std::memcpy(s_LocalRpa, LocalRpa, sizeof(s_LocalRpa));
	std::memcpy(s_PeerRpa, PeerRpa, sizeof(s_PeerRpa));
}

void CaptureLegacy(uint16_t, uint8_t, uint8_t AddrType, uint8_t Addr[6])
{
	s_LegacyCount++;
	s_LegacyType = AddrType;
	std::memcpy(s_LegacyAddr, Addr, sizeof(s_LegacyAddr));
}

void ResetCapture()
{
	s_EnhancedCount = 0;
	s_ConnHdl = 0;
	s_Role = 0;
	s_IdentityType = 0;
	std::memset(s_IdentityAddr, 0, sizeof(s_IdentityAddr));
	std::memset(s_LocalRpa, 0, sizeof(s_LocalRpa));
	std::memset(s_PeerRpa, 0, sizeof(s_PeerRpa));
	s_LegacyCount = 0;
	s_LegacyType = 0;
	std::memset(s_LegacyAddr, 0, sizeof(s_LegacyAddr));
}

void FillEvent(BtHciLeEvtEnhConnComplete_t &evt)
{
	std::memset(&evt, 0, sizeof(evt));
	evt.Status = 0;
	evt.ConnHdl = 0x1234;
	evt.Role = 1;
	evt.PeerAddrType = 3;
	const uint8_t identity[6] = { 0x61, 0x62, 0x63, 0x64, 0x65, 0xC6 };
	const uint8_t localRpa[6] = { 0x10, 0x20, 0x30, 0x40, 0x50, 0x46 };
	const uint8_t peerRpa[6] = { 0x11, 0x21, 0x31, 0x41, 0x51, 0x47 };
	std::memcpy(evt.PeerAddr, identity, sizeof(identity));
	std::memcpy(evt.LocalResolPriAddr, localRpa, sizeof(localRpa));
	std::memcpy(evt.PeerResolPriAddr, peerRpa, sizeof(peerRpa));
	evt.ConnInterval = 24;
	evt.SupervTimeout = 400;
}

void TestEnhancedConnectionAddresses(Context &ctx)
{
	static const Requirement req = {
		"HCI-LE-ENH-CONN-BV-01", "Core Vol 4 Part E", "7.7.65.10 and 7.7.65.36",
		"Enhanced Connection Complete forwards the peer identity and both resolvable private addresses without losing the legacy callback fallback"
	};
	ctx.Begin(req);

	VirtualClock clock;
	BtHciDevice_t host = {};
	host.EnhancedConnected = CaptureEnhanced;
	host.Connected = CaptureLegacy;
	VirtualController controller(clock);
	BT_CHECK(ctx, controller.Attach(&host));

	BtHciLeEvtEnhConnComplete_t evt;
	FillEvent(evt);
	ResetCapture();
	BT_CHECK(ctx, controller.InjectLeEvent(BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V1,
		&evt, sizeof(evt)));
	BT_CHECK(ctx, s_EnhancedCount == 1);
	BT_CHECK(ctx, s_LegacyCount == 0);
	BT_CHECK(ctx, s_ConnHdl == evt.ConnHdl);
	BT_CHECK(ctx, s_Role == evt.Role);
	BT_CHECK(ctx, s_IdentityType == evt.PeerAddrType);
	BT_CHECK(ctx, std::memcmp(s_IdentityAddr, evt.PeerAddr, 6) == 0);
	BT_CHECK(ctx, std::memcmp(s_LocalRpa, evt.LocalResolPriAddr, 6) == 0);
	BT_CHECK(ctx, std::memcmp(s_PeerRpa, evt.PeerResolPriAddr, 6) == 0);

	// A truncated Enhanced Connection Complete must not invoke either callback.
	ResetCapture();
	BT_CHECK(ctx, controller.InjectLeEvent(BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V1,
		&evt, static_cast<uint8_t>(sizeof(evt) - 1U)));
	BT_CHECK(ctx, s_EnhancedCount == 0);
	BT_CHECK(ctx, s_LegacyCount == 0);

	// Existing HCI users that do not provide the enhanced callback retain the
	// old identity-address callback behavior.
	host.EnhancedConnected = nullptr;
	ResetCapture();
	BT_CHECK(ctx, controller.InjectLeEvent(BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE_V2,
		&evt, sizeof(evt)));
	BT_CHECK(ctx, s_EnhancedCount == 0);
	BT_CHECK(ctx, s_LegacyCount == 1);
	BT_CHECK(ctx, s_LegacyType == evt.PeerAddrType);
	BT_CHECK(ctx, std::memcmp(s_LegacyAddr, evt.PeerAddr, 6) == 0);

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
	Context ctx("Bluetooth enhanced connection address compliance");
	TestEnhancedConnectionAddresses(ctx);
	return ctx.Finish(false);
}
