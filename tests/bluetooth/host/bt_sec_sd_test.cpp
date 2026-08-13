/**-------------------------------------------------------------------------
@file	bt_sec_sd_test.cpp

@brief	Host tests for the nRF52 SoftDevice security module,
		ARM/Nordic/nRF52/src/bt_sec_sd.cpp.

		Two duties are covered. First, that a link whose security this device
		refuses is actually dropped: the SoftDevice accepts two HCI reasons in
		sd_ble_gap_disconnect and answers anything else with
		NRF_ERROR_INVALID_PARAM while leaving the link up, so the reason and
		the return both matter. Second, that a peer holding an existing bond
		record is refused before the pairing runs rather than after, which is
		what Core Vol 3 Part H 2.3 asks for: "If the responding device does
		not support pairing or pairing cannot be performed then the responding
		device shall respond using the Pairing Failed message with the error
		code Pairing Not Supported".

@author	Hoang Nguyen Hoan
@date	Aug. 13, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/
#include <cstring>

#include "bt_sec_sd_test_stubs.h"
#include "bt_test_harness.h"

#include "ble.h"
#include "ble_conn_state.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_error.h"
#include "peer_manager_types.h"
#include "security_manager.h"

// IOsonata extensions the module exports beyond the SDK sm surface. No header
// declares them, so they are named here with the linkage the definitions have.
void BtSecSdScOnlySet(bool bEnable);
void BtSecSdMinKeySizeSet(uint8_t Size);

namespace {

bttest::Context g_Ctx("bt_sec_sd_test");

#define CHECK(expr)	BT_CHECK(g_Ctx, expr)

const uint16_t kConnHdl = 0;
const pm_peer_id_t kBondedPeerId = 5;

ble_gap_sec_params_t g_LocalParams;

void ResetStubsAndParams(void)
{
	SecSdStubsReset();

	std::memset(&g_LocalParams, 0, sizeof(g_LocalParams));
	g_LocalParams.bond = 1;
	g_LocalParams.lesc = 1;
	g_LocalParams.min_key_size = 7;
	g_LocalParams.max_key_size = 16;
	g_LocalParams.kdist_own.enc = 1;
	g_LocalParams.kdist_own.id = 1;
	g_LocalParams.kdist_peer.enc = 1;
	g_LocalParams.kdist_peer.id = 1;
	sm_sec_params_set(&g_LocalParams);

	BtSecSdScOnlySet(false);
	BtSecSdMinKeySizeSet(7);
}

void FeedEvt(uint16_t EvtId, const ble_gap_evt_t *pGapEvt)
{
	ble_evt_t evt;

	std::memset(&evt, 0, sizeof(evt));
	evt.header.evt_id = EvtId;
	evt.evt.gap_evt = *pGapEvt;
	sm_ble_evt_handler(&evt);
}

void FeedSecParamsRequest(uint16_t ConnHdl, bool bPeerBond)
{
	ble_gap_evt_t gap;

	std::memset(&gap, 0, sizeof(gap));
	gap.conn_handle = ConnHdl;
	gap.params.sec_params_request.peer_params.bond = bPeerBond ? 1 : 0;
	gap.params.sec_params_request.peer_params.lesc = 1;
	gap.params.sec_params_request.peer_params.min_key_size = 7;
	gap.params.sec_params_request.peer_params.max_key_size = 16;
	FeedEvt(BLE_GAP_EVT_SEC_PARAMS_REQUEST, &gap);
}

void FeedAuthStatusSuccess(uint16_t ConnHdl, bool bBonded)
{
	ble_gap_evt_t gap;

	std::memset(&gap, 0, sizeof(gap));
	gap.conn_handle = ConnHdl;
	gap.params.auth_status.auth_status = BLE_GAP_SEC_STATUS_SUCCESS;
	gap.params.auth_status.bonded = bBonded ? 1 : 0;
	FeedEvt(BLE_GAP_EVT_AUTH_STATUS, &gap);
}

void FeedConnSecUpdate(uint16_t ConnHdl, uint8_t KeySize)
{
	ble_gap_evt_t gap;

	std::memset(&gap, 0, sizeof(gap));
	gap.conn_handle = ConnHdl;
	gap.params.conn_sec_update.conn_sec.encr_key_size = KeySize;
	FeedEvt(BLE_GAP_EVT_CONN_SEC_UPDATE, &gap);
}

// Answer PM_EVT_CONN_SEC_CONFIG_REQ the way an application that permits
// re-pairing would.
void AllowRepairingHook(pm_evt_t *pEvt)
{
	if (pEvt->evt_id == PM_EVT_CONN_SEC_CONFIG_REQ)
	{
		pm_conn_sec_config_t cfg;

		cfg.allow_repairing = true;
		sm_conn_sec_config_reply(pEvt->conn_handle, &cfg);
	}
}

bool DisconnectedWith(uint8_t Reason)
{
	return g_SecSdDisconnect.Count >= 1 && g_SecSdDisconnect.Reason[0] == Reason;
}

// Index of the first sd_ble_gap_sec_params_reply, or -1.
int FirstReply(void)
{
	return g_SecSdParamsReply.Count > 0 ? 0 : -1;
}

//-----------------------------------------------------------------------------
// Disconnect reason.
//
// ble_gap.h documents sd_ble_gap_disconnect as accepting
// BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION and
// BLE_HCI_CONN_INTERVAL_UNACCEPTABLE only. Authentication Failure, 0x05,
// returns NRF_ERROR_INVALID_PARAM and the link stays up, which is the whole
// failure being tested for here: a link the module has decided to drop, and
// which is still up and passing data afterwards.
//
// Core Vol 4 Part E 7.1.6 lists Remote User Terminated Connection among the
// reasons a Host may give in HCI_Disconnect, so the accepted value is also a
// valid one to put on air.
//-----------------------------------------------------------------------------

// The application refused to let an already bonded peer re-pair, and the
// refusal is only discovered after key distribution. The link is encrypted
// with a key the application rejected, so it has to go.
void TestRefusedRepairingDisconnects()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, true);
	SecSdPeerIdSet(kConnHdl, PM_PEER_ID_INVALID);
	SecSdDuplicateBondSet(kBondedPeerId);

	FeedSecParamsRequest(kConnHdl, true);
	FeedAuthStatusSuccess(kConnHdl, true);

	CHECK(g_SecSdDisconnect.Count == 1);
	CHECK(DisconnectedWith(BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));
	// The refused bond is not written.
	CHECK(g_SecSdWriteBufStoreCount == 0);
}

// An under-keyed link is failed and dropped at CONN_SEC_UPDATE, on a fresh
// pairing or a reconnect alike.
void TestUnderKeyedLinkDisconnects()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, true);
	BtSecSdMinKeySizeSet(16);

	FeedConnSecUpdate(kConnHdl, 7);

	CHECK(g_SecSdDisconnect.Count == 1);
	CHECK(DisconnectedWith(BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));
}

// NRF_ERROR_BUSY means another procedure holds the link, not that the link may
// stay up. The request has to be repeated.
void TestBusyDisconnectIsRetried()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, true);
	BtSecSdMinKeySizeSet(16);
	SecSdDisconnectResultSet(NRF_ERROR_BUSY);

	FeedConnSecUpdate(kConnHdl, 7);

	CHECK(g_SecSdDisconnect.Count == 2);
	CHECK(g_SecSdDisconnect.Reason[1] == BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

//-----------------------------------------------------------------------------
// Re-pairing gate.
//
// Core Vol 3 Part H 2.3: "If the responding device does not support pairing or
// pairing cannot be performed then the responding device shall respond using
// the Pairing Failed message with the error code Pairing Not Supported
// otherwise it responds with a Pairing Response message." Vol 3 Part H C.5.1
// draws the same exchange. An application that will not replace an existing
// bond is the "pairing cannot be performed" case, and the answer belongs at
// the Pairing Request, before the keys are exchanged, not after.
//-----------------------------------------------------------------------------

// A peer that is already identified, which is every bonded peer, and an
// application that does not permit re-pairing. The refusal is a Pairing
// Failed, and no keyset is handed to the SoftDevice.
void TestKnownPeerRefusedAtPairingRequest()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, false);
	SecSdPeerIdSet(kConnHdl, kBondedPeerId);

	FeedSecParamsRequest(kConnHdl, true);

	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_CONFIG_REQ) == 1);
	CHECK(g_SecSdParamsReply.Count == 1);
	int i = FirstReply();
	CHECK(i >= 0 && g_SecSdParamsReply.SecStatus[i] == BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP);
	CHECK(i >= 0 && !g_SecSdParamsReply.bHasKeyset[i]);
	// The application was never asked for pairing parameters, because the
	// pairing was already refused.
	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_PARAMS_REQ) == 0);
	// The existing bond record is untouched.
	CHECK(g_SecSdWriteBufStoreCount == 0);
}

// The peer id reaches the application with the event, so it can tell which
// bond it is being asked about.
void TestConfigReqNamesThePeer()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, false);
	SecSdPeerIdSet(kConnHdl, kBondedPeerId);

	FeedSecParamsRequest(kConnHdl, true);

	const pm_evt_t *pEvt = SecSdEvtFind(PM_EVT_CONN_SEC_CONFIG_REQ);
	CHECK(pEvt != nullptr);
	CHECK(pEvt != nullptr && pEvt->peer_id == kBondedPeerId);
	CHECK(pEvt != nullptr && pEvt->conn_handle == kConnHdl);
}

// The same peer, and an application that permits it. Pairing runs.
void TestKnownPeerAllowedProceeds()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, false);
	SecSdPeerIdSet(kConnHdl, kBondedPeerId);
	SecSdEvtHookSet(AllowRepairingHook);

	FeedSecParamsRequest(kConnHdl, true);

	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_CONFIG_REQ) == 1);
	CHECK(g_SecSdParamsReply.Count == 1);
	int i = FirstReply();
	CHECK(i >= 0 && g_SecSdParamsReply.SecStatus[i] == BLE_GAP_SEC_STATUS_SUCCESS);
	CHECK(i >= 0 && g_SecSdParamsReply.bHasKeyset[i]);
}

// Having answered once, the application is not asked again for the same
// procedure when the bond is committed.
void TestAllowedPeerIsNotAskedTwice()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, true);
	SecSdPeerIdSet(kConnHdl, kBondedPeerId);
	SecSdDuplicateBondSet(kBondedPeerId);
	SecSdEvtHookSet(AllowRepairingHook);

	FeedSecParamsRequest(kConnHdl, true);
	FeedAuthStatusSuccess(kConnHdl, true);

	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_CONFIG_REQ) == 1);
	CHECK(g_SecSdDisconnect.Count == 0);
	// Permitted, so the new bond replaces the old record.
	CHECK(g_SecSdWriteBufStoreCount == 1);
}

// Control. An unknown peer pairs without the application being asked at all.
void TestUnknownPeerIsNotGated()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, false);
	SecSdPeerIdSet(kConnHdl, PM_PEER_ID_INVALID);

	FeedSecParamsRequest(kConnHdl, true);

	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_CONFIG_REQ) == 0);
	CHECK(g_SecSdParamsReply.Count == 1);
	int i = FirstReply();
	CHECK(i >= 0 && g_SecSdParamsReply.SecStatus[i] == BLE_GAP_SEC_STATUS_SUCCESS);
	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_PARAMS_REQ) == 1);
}

// Control. The gate is a responding device duty. A central reaching
// SEC_PARAMS_REQUEST asked for the pairing itself, so there is nothing to
// refuse on its behalf.
void TestCentralIsNotGated()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_CENTRAL, false);
	SecSdPeerIdSet(kConnHdl, kBondedPeerId);

	FeedSecParamsRequest(kConnHdl, true);

	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_CONFIG_REQ) == 0);
	CHECK(g_SecSdParamsReply.Count == 1);
	int i = FirstReply();
	CHECK(i >= 0 && g_SecSdParamsReply.SecStatus[i] == BLE_GAP_SEC_STATUS_SUCCESS);
}

// A refused pairing is still a security procedure that started and failed, so
// the application hears about it through the ordinary failure path when the
// SoftDevice reports the Pairing Failed back.
void TestRefusalReportsProcedureFailure()
{
	ResetStubsAndParams();
	SecSdLinkSet(kConnHdl, true, BLE_GAP_ROLE_PERIPH, false);
	SecSdPeerIdSet(kConnHdl, kBondedPeerId);

	FeedSecParamsRequest(kConnHdl, true);
	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_START) == 1);

	ble_gap_evt_t gap;
	std::memset(&gap, 0, sizeof(gap));
	gap.conn_handle = kConnHdl;
	gap.params.auth_status.auth_status = BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP;
	gap.params.auth_status.error_src = BLE_GAP_SEC_STATUS_SOURCE_LOCAL;
	FeedEvt(BLE_GAP_EVT_AUTH_STATUS, &gap);

	CHECK(SecSdEvtCount(PM_EVT_CONN_SEC_FAILED) == 1);
}

} // namespace

int main(void)
{
	if (sm_init() != NRF_SUCCESS)
	{
		std::printf("bt_sec_sd_test: sm_init failed\n");
		return 1;
	}

	g_Ctx.Run("refused repairing disconnects", TestRefusedRepairingDisconnects);
	g_Ctx.Run("under keyed link disconnects", TestUnderKeyedLinkDisconnects);
	g_Ctx.Run("busy disconnect is retried", TestBusyDisconnectIsRetried);
	g_Ctx.Run("known peer refused at pairing request", TestKnownPeerRefusedAtPairingRequest);
	g_Ctx.Run("config req names the peer", TestConfigReqNamesThePeer);
	g_Ctx.Run("known peer allowed proceeds", TestKnownPeerAllowedProceeds);
	g_Ctx.Run("allowed peer is not asked twice", TestAllowedPeerIsNotAskedTwice);
	g_Ctx.Run("unknown peer is not gated", TestUnknownPeerIsNotGated);
	g_Ctx.Run("central is not gated", TestCentralIsNotGated);
	g_Ctx.Run("refusal reports procedure failure", TestRefusalReportsProcedureFailure);

	return g_Ctx.Finish();
}
