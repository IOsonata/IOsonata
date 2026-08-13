/**-------------------------------------------------------------------------
@file	bt_sec_sd_test_stubs.h

@brief	Host stand-ins for the SoftDevice and the nRF5 SDK peer data layer,
		so ARM/Nordic/nRF52/src/bt_sec_sd.cpp can be compiled and driven on a
		desktop.

		The types in nordic_stub are written to the field names bt_sec_sd.cpp
		uses, not to the SDK layout. Nothing here checks flash record layout;
		what these stubs exercise is the control flow of the module, which
		calls it made, in what order, and with what arguments.

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
#ifndef __BT_SEC_SD_TEST_STUBS_H__
#define __BT_SEC_SD_TEST_STUBS_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"
#include "ble_conn_state.h"
#include "ble_gap.h"
#include "peer_manager_types.h"

#define SEC_SD_CAPTURE_MAX			16

// Every sd_ble_gap_disconnect the module made.
typedef struct {
	int Count;
	uint16_t ConnHdl[SEC_SD_CAPTURE_MAX];
	uint8_t Reason[SEC_SD_CAPTURE_MAX];
} SecSdDisconnectCapture_t;

// Every sd_ble_gap_sec_params_reply the module made.
typedef struct {
	int Count;
	uint16_t ConnHdl[SEC_SD_CAPTURE_MAX];
	uint8_t SecStatus[SEC_SD_CAPTURE_MAX];
	bool bHasParams[SEC_SD_CAPTURE_MAX];
	bool bHasKeyset[SEC_SD_CAPTURE_MAX];
} SecSdParamsReplyCapture_t;

// Every pm_evt_t the module emitted.
typedef struct {
	int Count;
	pm_evt_t Evt[SEC_SD_CAPTURE_MAX];
} SecSdEvtCapture_t;

extern SecSdDisconnectCapture_t g_SecSdDisconnect;
extern SecSdParamsReplyCapture_t g_SecSdParamsReply;
extern SecSdEvtCapture_t g_SecSdEvt;

extern int g_SecSdAuthenticateCount;
extern int g_SecSdEncryptCount;
extern int g_SecSdWriteBufReleaseCount;
extern int g_SecSdWriteBufStoreCount;

// Result the next sd_ble_gap_disconnect returns. Consumed once, then the stub
// falls back to NRF_SUCCESS.
void SecSdDisconnectResultSet(uint32_t Result);

// Result every sd_ble_gap_sec_params_reply returns until changed.
void SecSdParamsReplyResultSet(uint32_t Result);

// Answer im_find_duplicate_bonding_data gives, that is, whether the peer that
// just distributed its keys already has a bond record.
void SecSdDuplicateBondSet(pm_peer_id_t PeerId);

// Peer id im_peer_id_get_by_conn_handle reports for a connection.
void SecSdPeerIdSet(uint16_t ConnHdl, pm_peer_id_t PeerId);

// Connection state the module reads back through ble_conn_state.
void SecSdLinkSet(uint16_t ConnHdl, bool bValid, uint8_t Role, bool bEncrypted);

// Called from inside pm_sm_evt_handler, so a test can answer an event the way
// an application would, through the sm_* reply calls.
typedef void (*SecSdEvtHook_t)(pm_evt_t *pEvt);
void SecSdEvtHookSet(SecSdEvtHook_t Hook);

// Clear every capture, every script and the whole connection table.
void SecSdStubsReset(void);

// Count of captured events with this id.
int SecSdEvtCount(pm_evt_id_t EvtId);

// First captured event with this id, or nullptr.
const pm_evt_t *SecSdEvtFind(pm_evt_id_t EvtId);

#endif
