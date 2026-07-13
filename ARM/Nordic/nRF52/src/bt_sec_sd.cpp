/**-------------------------------------------------------------------------
@file	bt_sec_sd.cpp

@brief	IOsonata security manager for the nRF5 SDK SoftDevice path.

		Replaces the nRF5 SDK peer_manager modules security_manager.c and
		security_dispatcher.c with one module. It implements the sm_* API
		surface that peer_manager.c calls (security_manager.h), emits the same
		pm_evt_t events through pm_sm_evt_handler, and drives the SoftDevice
		GAP security procedures directly. The peer data layer stays SDK:
		peer_database, peer_data_storage, id_manager, pm_buffer and FDS are
		used unchanged, which keeps bond records byte-identical for the SDK
		OTA DFU bootloader handoff (ble_dfu_bonded).

		LESC runs on the IOsonata bt_lesc module (bt_lesc_sd.cpp) through the
		BtLesc* names; nrf_ble_lesc and its shim are not used. This module is
		the single delivery point of BLE events into BtLescOnBleEvt.

		The keyset handed to sd_ble_gap_sec_params_reply points into the
		pm_peer_data_bonding_t inside the peer_database write buffer, so the
		SoftDevice writes the distributed keys straight into the FDS-destined
		structure; AUTH_STATUS commits it with pdb_write_buf_store. This is
		what guarantees the FDS record layout identity.

		Repeated pairing attempt protection: the SDK auth_status_tracker
		module (ast_*) stays linked; failures are reported to it and
		blacklisted peers are rejected with BLE_GAP_SEC_STATUS_REPEATED_ATTEMPTS,
		gated by PM_RA_PROTECTION_ENABLED as in the SDK.

		Both roles are implemented. Runs entirely in the pm BLE event dispatch
		context, the same as the SDK modules it replaces.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "sdk_common.h"

#if NRF_MODULE_ENABLED(PEER_MANAGER)

#include "ble.h"
#include "ble_gap.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_conn_state.h"
#include "nrf_sdh_ble.h"			// NRF_SDH_BLE_TOTAL_LINK_COUNT

#include "security_manager.h"		// the sm_* API this module implements
#include "security_dispatcher.h"	// smd_init, called by pm_init; the rest of
									// the smd layer is absorbed into this module
#include "peer_manager_types.h"
#include "peer_manager_internal.h"	// pm_sm_evt_handler
#include "peer_database.h"
#include "peer_data_storage.h"
#include "id_manager.h"

#if PM_RA_PROTECTION_ENABLED
#include "auth_status_tracker.h"
#endif

#include "bt_lesc.h"

// Event sink in peer_manager.c; the same extern hookup the SDK sm uses.
extern "C" void pm_sm_evt_handler(pm_evt_t * p_sm_evt);

// The context type used in PM_EVT_CONN_SEC_PARAMS_REQ events and in calls to
// sm_sec_params_reply(). Layout compatible with the SDK sm context: the app
// interacts with it only through sm_sec_params_reply.
typedef struct {
	ble_gap_sec_params_t *pSecParams;		// params to use in the reply
	ble_gap_sec_params_t  SecParamsMem;		// buffer holding the params
	bool                  bReplyCalled;		// sm_sec_params_reply was called
} SecParamsReplyCtx_t;

static bool                          s_bInit;
static ble_gap_sec_params_t          s_SecParams;			// default sec params buffer
static ble_gap_sec_params_t         *s_pSecParams;			// NULL until sm_sec_params_set

// SIG policy the frozen SDK never offered. SC only rejects legacy pairing
// with Authentication Requirements (Core Vol 3 Part H 2.3.5.1); the minimum
// encryption key size is checked on every CONN_SEC_UPDATE and an under-keyed
// link is failed and disconnected (KNOB class downgrade mitigation). The SMP
// floor is 7; security sensitive applications set 16.
static bool                          s_bScOnly;
static uint8_t                       s_MinKeySize = 7;

// Per-connection state, indexed by connection handle (SoftDevice handles are
// 0 .. NRF_SDH_BLE_TOTAL_LINK_COUNT-1). PeerPk is the LESC peer public key
// receive buffer for the keyset: per link, so concurrent pairings cannot
// overwrite each other (the SDK dispatcher shares one static buffer here).
// The pending reply record preserves the exact application answer, including
// a rejection, across an NRF_ERROR_BUSY retry; the SDK loses it and re-asks.
typedef struct {
	ble_gap_lesc_p256_pk_t PeerPk;			// SoftDevice writes the peer key here
	ble_gap_sec_params_t   ReplyParams;		// pending reply parameters
	bool                   bReplyValid;		// a reply is pending retry
	bool                   bReplyReject;	// the pending reply is a rejection
	bool                   bStorePending;	// bond store deferred on busy
	bool                   bStoreNewPeer;	// the pending store allocated the peer id
	pm_peer_id_t           StorePeerId;		// peer id of the pending store
	uint8_t                EncrKeySize;		// negotiated key size, from CONN_SEC_UPDATE
} BtSecSdLink_t;

static BtSecSdLink_t s_Links[NRF_SDH_BLE_TOTAL_LINK_COUNT];

static inline BtSecSdLink_t *LinkGet(uint16_t ConnHdl)
{
	return (ConnHdl < NRF_SDH_BLE_TOTAL_LINK_COUNT) ? &s_Links[ConnHdl] : nullptr;
}

static void LinkReplyClear(uint16_t ConnHdl)
{
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	if (pLink != nullptr)
	{
		pLink->bReplyValid  = false;
		pLink->bReplyReject = false;
	}
}

// Procedure bookkeeping and retry flags, one bit per connection.
static ble_conn_state_user_flag_id_t s_FlagSecProc          = BLE_CONN_STATE_USER_FLAG_INVALID;
static ble_conn_state_user_flag_id_t s_FlagSecProcPairing   = BLE_CONN_STATE_USER_FLAG_INVALID;
static ble_conn_state_user_flag_id_t s_FlagSecProcBonding   = BLE_CONN_STATE_USER_FLAG_INVALID;
static ble_conn_state_user_flag_id_t s_FlagAllowRepairing   = BLE_CONN_STATE_USER_FLAG_INVALID;
static ble_conn_state_user_flag_id_t s_FlagSecurePendBusy   = BLE_CONN_STATE_USER_FLAG_INVALID;
static ble_conn_state_user_flag_id_t s_FlagSecureForceRepair = BLE_CONN_STATE_USER_FLAG_INVALID;
static ble_conn_state_user_flag_id_t s_FlagSecureNullParams = BLE_CONN_STATE_USER_FLAG_INVALID;
static ble_conn_state_user_flag_id_t s_FlagReplyPendBusy    = BLE_CONN_STATE_USER_FLAG_INVALID;

// ---- Event emission ---------------------------------------------------------

static pm_evt_t NewEvt(pm_evt_id_t EvtId, uint16_t ConnHdl)
{
	pm_evt_t evt;

	memset(&evt, 0, sizeof(evt));
	evt.evt_id      = EvtId;
	evt.conn_handle = ConnHdl;
	evt.peer_id     = im_peer_id_get_by_conn_handle(ConnHdl);

	return evt;
}

static void EvtSend(pm_evt_t *pEvt)
{
	pm_sm_evt_handler(pEvt);
}

static void UnexpectedErrorSend(uint16_t ConnHdl, ret_code_t ErrCode)
{
	pm_evt_t evt = NewEvt(PM_EVT_ERROR_UNEXPECTED, ConnHdl);

	evt.params.error_unexpected.error = ErrCode;
	EvtSend(&evt);
}

// ---- Procedure bookkeeping --------------------------------------------------

static void SecProcStart(uint16_t ConnHdl, bool bSuccess, pm_conn_sec_procedure_t Procedure)
{
	ble_conn_state_user_flag_set(ConnHdl, s_FlagSecProc, bSuccess);
	if (bSuccess)
	{
		ble_conn_state_user_flag_set(ConnHdl, s_FlagSecProcPairing,
									 Procedure != PM_CONN_SEC_PROCEDURE_ENCRYPTION);
		ble_conn_state_user_flag_set(ConnHdl, s_FlagSecProcBonding,
									 Procedure == PM_CONN_SEC_PROCEDURE_BONDING);

		pm_evt_t evt = NewEvt(PM_EVT_CONN_SEC_START, ConnHdl);
		evt.params.conn_sec_start.procedure = Procedure;
		EvtSend(&evt);
	}
}

// True while a pairing (not encryption-only) procedure runs on the link.
static bool ProcIsPairing(uint16_t ConnHdl)
{
	return ble_conn_state_user_flag_get(ConnHdl, s_FlagSecProc) &&
		   ble_conn_state_user_flag_get(ConnHdl, s_FlagSecProcPairing);
}

static pm_conn_sec_procedure_t ProcGet(uint16_t ConnHdl)
{
	if (!ble_conn_state_user_flag_get(ConnHdl, s_FlagSecProcPairing))
	{
		return PM_CONN_SEC_PROCEDURE_ENCRYPTION;
	}
	return ble_conn_state_user_flag_get(ConnHdl, s_FlagSecProcBonding) ?
		   PM_CONN_SEC_PROCEDURE_BONDING : PM_CONN_SEC_PROCEDURE_PAIRING;
}

// Common failure emission: report CONN_SEC_FAILED with the in-flight procedure
// and clear the procedure flag.
static void SecFailureSend(uint16_t ConnHdl, pm_sec_error_code_t Error, uint8_t ErrorSrc)
{
	if (!ble_conn_state_user_flag_get(ConnHdl, s_FlagSecProc))
	{
		return;		// no procedure in flight; nothing to report
	}
	ble_conn_state_user_flag_set(ConnHdl, s_FlagSecProc, false);

	pm_evt_t evt = NewEvt(PM_EVT_CONN_SEC_FAILED, ConnHdl);
	evt.params.conn_sec_failed.procedure = ProcGet(ConnHdl);
	evt.params.conn_sec_failed.error     = Error;
	evt.params.conn_sec_failed.error_src = ErrorSrc;
	EvtSend(&evt);
}

// ---- Keyset construction (the FDS layout anchor) ----------------------------

// Point the SoftDevice keyset into the pm_peer_data_bonding_t inside the
// peer_database write buffer, so the distributed keys land directly in the
// FDS-destined structure. Returns NRF_SUCCESS, NRF_ERROR_BUSY (no buffer yet,
// retried by the pending pump) or an internal error.
static ret_code_t SecKeysetFill(uint16_t ConnHdl, uint8_t Role,
								ble_gap_sec_keyset_t *pKeyset)
{
	pm_peer_data_t peerData;
	ret_code_t r = pdb_write_buf_get(PDB_TEMP_PEER_ID(ConnHdl),
									 PM_PEER_DATA_ID_BONDING, 1, &peerData);
	if (r == NRF_ERROR_BUSY)
	{
		return r;
	}
	if (r != NRF_SUCCESS)
	{
		return NRF_ERROR_INTERNAL;
	}

	memset(peerData.p_bonding_data, 0, sizeof(pm_peer_data_bonding_t));
	peerData.p_bonding_data->own_role = Role;

	pKeyset->keys_own.p_enc_key  = &peerData.p_bonding_data->own_ltk;
	pKeyset->keys_own.p_pk       = BtLescPubKeyGet();
	pKeyset->keys_peer.p_enc_key = &peerData.p_bonding_data->peer_ltk;
	pKeyset->keys_peer.p_id_key  = &peerData.p_bonding_data->peer_ble_id;

	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	if (pLink == nullptr)
	{
		(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(ConnHdl),
									 PM_PEER_DATA_ID_BONDING);
		return NRF_ERROR_INVALID_STATE;
	}
	pKeyset->keys_peer.p_pk      = &pLink->PeerPk;

	// The address the peer used at connection establishment; overwritten by
	// the identity if the peer distributes one.
	r = im_ble_addr_get(ConnHdl, &peerData.p_bonding_data->peer_ble_id.id_addr_info);
	if (r != NRF_SUCCESS)
	{
		(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(ConnHdl),
									 PM_PEER_DATA_ID_BONDING);
		return NRF_ERROR_INVALID_STATE;
	}

	return NRF_SUCCESS;
}

// ---- sec params reply (SEC_PARAMS_REQUEST answer) ---------------------------

static ret_code_t ParamsReplyPerform(uint16_t ConnHdl, ble_gap_sec_params_t *pSecParams)
{
	uint8_t              role = ble_conn_state_role(ConnHdl);
	uint8_t              secStatus = BLE_GAP_SEC_STATUS_SUCCESS;
	ble_gap_sec_params_t *pReplyParams = pSecParams;
	ble_gap_sec_keyset_t keyset;
	ret_code_t           r = NRF_SUCCESS;
	bool                 bKeysetPrepared = false;

	memset(&keyset, 0, sizeof(keyset));

	if (role == BLE_GAP_ROLE_INVALID)
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

#if PM_RA_PROTECTION_ENABLED
	if (ast_peer_blacklisted(ConnHdl))
	{
		secStatus    = BLE_GAP_SEC_STATUS_REPEATED_ATTEMPTS;
		pReplyParams = NULL;
	}
	else
#endif
	if (pSecParams == NULL)
	{
		// Reject pairing.
		secStatus    = BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP;
		pReplyParams = NULL;
	}
	else
	{
		r = SecKeysetFill(ConnHdl, role, &keyset);
		if (r != NRF_SUCCESS)
		{
			ble_conn_state_user_flag_set(ConnHdl, s_FlagReplyPendBusy,
										 r == NRF_ERROR_BUSY);
			return r;
		}
		bKeysetPrepared = true;
	}

	if (role == BLE_GAP_ROLE_CENTRAL)
	{
		// Central: parameters were given at sd_ble_gap_authenticate.
		pReplyParams = NULL;
	}

	r = sd_ble_gap_sec_params_reply(ConnHdl, secStatus, pReplyParams,
									(secStatus == BLE_GAP_SEC_STATUS_SUCCESS) ?
									&keyset : NULL);
	ble_conn_state_user_flag_set(ConnHdl, s_FlagReplyPendBusy, r == NRF_ERROR_BUSY);

	if (bKeysetPrepared && r != NRF_SUCCESS && r != NRF_ERROR_BUSY)
	{
		(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(ConnHdl),
									 PM_PEER_DATA_ID_BONDING);
	}

	if (r == NRF_ERROR_INVALID_STATE && !ble_conn_state_valid(ConnHdl))
	{
		return NRF_SUCCESS;		// link dropped; benign
	}
	// A live link returning INVALID_STATE is a sequencing defect (duplicate
	// reply, wrong phase) and surfaces to the caller.
	// NRF_ERROR_BUSY propagates: ReplyAttempt must see it to keep the exact
	// reply for the retry. Conversion to a success result for the public API
	// happens at the callers.
	return r;
}

// Perform a reply and keep the exact answer, including a rejection, for the
// BUSY retry. On a terminal result the pending record is cleared.
static ret_code_t ReplyAttempt(uint16_t ConnHdl, ble_gap_sec_params_t *pSecParams)
{
	ret_code_t r = ParamsReplyPerform(ConnHdl, pSecParams);
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);

	if (pLink != nullptr)
	{
		if (r == NRF_ERROR_BUSY)
		{
			pLink->bReplyReject = (pSecParams == NULL);
			if (pSecParams != NULL)
			{
				pLink->ReplyParams = *pSecParams;
			}
			pLink->bReplyValid = true;
		}
		else
		{
			pLink->bReplyValid  = false;
			pLink->bReplyReject = false;
		}
	}
	return r;
}

// PM_EVT_CONN_SEC_PARAMS_REQ handling: emit the event with a reply context; if
// the application does not answer within its handler through
// sm_sec_params_reply, reply with the context (default) parameters.
static void ParamsRequestProcess(uint16_t ConnHdl, const ble_gap_sec_params_t *pPeerParams)
{
	SecParamsReplyCtx_t ctx;

	ctx.pSecParams   = s_pSecParams;
	ctx.bReplyCalled = false;

	pm_evt_t evt = NewEvt(PM_EVT_CONN_SEC_PARAMS_REQ, ConnHdl);
	evt.params.conn_sec_params_req.p_peer_params = pPeerParams;
	evt.params.conn_sec_params_req.p_context     = &ctx;
	EvtSend(&evt);

	if (!ctx.bReplyCalled)
	{
		ret_code_t r = ReplyAttempt(ConnHdl, ctx.pSecParams);
		if (r != NRF_SUCCESS && r != NRF_ERROR_BUSY)
		{
			UnexpectedErrorSend(ConnHdl, r);
		}
	}
}

// ---- link secure (pm_conn_secure and Security Request) ----------------------

static ret_code_t LinkSecureCentralEncrypt(uint16_t ConnHdl, pm_peer_id_t PeerId)
{
	pm_peer_data_flash_t peerData;
	const ble_gap_enc_key_t *pKey = NULL;

	ret_code_t r = pdb_peer_data_ptr_get(PeerId, PM_PEER_DATA_ID_BONDING, &peerData);
	if (r == NRF_SUCCESS)
	{
		// Peer distributed its LTK as peripheral; for LESC both sides hold
		// the same LTK under own_ltk.
		pKey = &peerData.p_bonding_data->peer_ltk;
		if (peerData.p_bonding_data->own_ltk.enc_info.lesc)
		{
			pKey = &peerData.p_bonding_data->own_ltk;
		}
	}
	else if (r != NRF_ERROR_NOT_FOUND)
	{
		return (r == NRF_ERROR_BUSY) ? NRF_ERROR_BUSY : NRF_ERROR_INTERNAL;
	}

	if (pKey == NULL || !pKey->enc_info.ltk_len)
	{
		return NRF_ERROR_NOT_FOUND;		// no usable key; caller falls back to pairing
	}

	r = sd_ble_gap_encrypt(ConnHdl, &pKey->master_id, &pKey->enc_info);
	if (r == NRF_SUCCESS)
	{
		SecProcStart(ConnHdl, true, PM_CONN_SEC_PROCEDURE_ENCRYPTION);
	}
	return r;
}

static ret_code_t LinkSecureAuthenticate(uint16_t ConnHdl, ble_gap_sec_params_t *pSecParams)
{
	ret_code_t r = sd_ble_gap_authenticate(ConnHdl, pSecParams);

	if (r == NRF_SUCCESS)
	{
		uint8_t role = ble_conn_state_role(ConnHdl);
		if (role == BLE_GAP_ROLE_CENTRAL && pSecParams != NULL)
		{
			SecProcStart(ConnHdl, true, pSecParams->bond ?
						 PM_CONN_SEC_PROCEDURE_BONDING : PM_CONN_SEC_PROCEDURE_PAIRING);
		}
		// Peripheral: this sends the Security Request; the procedure starts
		// at SEC_PARAMS_REQUEST when the central responds.
	}
	else if (r == NRF_ERROR_NO_MEM)
	{
		r = NRF_ERROR_BUSY;		// too many concurrent procedures; retried
	}
	return r;
}

static ret_code_t LinkSecure(uint16_t ConnHdl, bool bNullParams, bool bForceRepairing,
							 bool bSendEvents)
{
	ret_code_t            r;
	ble_gap_sec_params_t *pParams = bNullParams ? NULL : s_pSecParams;
	uint8_t               role = ble_conn_state_role(ConnHdl);

	if (role == BLE_GAP_ROLE_INVALID)
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

	if (role == BLE_GAP_ROLE_CENTRAL && !bForceRepairing)
	{
		pm_peer_id_t peerId = im_peer_id_get_by_conn_handle(ConnHdl);

		if (peerId != PM_PEER_ID_INVALID)
		{
			r = LinkSecureCentralEncrypt(ConnHdl, peerId);
			if (r != NRF_ERROR_NOT_FOUND)
			{
				goto done;
			}
			// No stored key: fall through to pairing.
		}
	}

	r = LinkSecureAuthenticate(ConnHdl, pParams);

done:
	// Track retry state for BUSY; remember the call shape for the retry.
	ble_conn_state_user_flag_set(ConnHdl, s_FlagSecurePendBusy, r == NRF_ERROR_BUSY);
	if (r == NRF_ERROR_BUSY)
	{
		ble_conn_state_user_flag_set(ConnHdl, s_FlagSecureForceRepair, bForceRepairing);
		ble_conn_state_user_flag_set(ConnHdl, s_FlagSecureNullParams, bNullParams);
	}

	if (bSendEvents && r != NRF_SUCCESS && r != NRF_ERROR_BUSY &&
		r != NRF_ERROR_INVALID_STATE)
	{
		if (r == NRF_ERROR_TIMEOUT)
		{
			pm_evt_t evt = NewEvt(PM_EVT_CONN_SEC_FAILED, ConnHdl);
			evt.params.conn_sec_failed.procedure =
				(pParams != NULL && pParams->bond) ? PM_CONN_SEC_PROCEDURE_BONDING
												   : PM_CONN_SEC_PROCEDURE_PAIRING;
			evt.params.conn_sec_failed.error     = PM_CONN_SEC_ERROR_SMP_TIMEOUT;
			evt.params.conn_sec_failed.error_src = BLE_GAP_SEC_STATUS_SOURCE_LOCAL;
			EvtSend(&evt);
		}
		else
		{
			UnexpectedErrorSend(ConnHdl, r);
		}
	}

	if (r == NRF_ERROR_BUSY ||
		(r == NRF_ERROR_INVALID_STATE && !ble_conn_state_valid(ConnHdl)))
	{
		r = NRF_SUCCESS;	// BUSY retried by the pump; INVALID_STATE: link gone
	}
	return r;
}

// ---- GAP event handlers -----------------------------------------------------

static void SecParamsRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	if (s_bScOnly && !pGapEvt->params.sec_params_request.peer_params.lesc)
	{
		// Secure Connections Only: reject legacy pairing without asking the
		// application. The failure is reported through AUTH_STATUS.
		(void)sd_ble_gap_sec_params_reply(pGapEvt->conn_handle,
										  BLE_GAP_SEC_STATUS_AUTH_REQ, NULL, NULL);
		return;
	}

	if (ble_conn_state_role(pGapEvt->conn_handle) == BLE_GAP_ROLE_PERIPH)
	{
		// New security procedure: reset the repairing decision for the link.
		ble_conn_state_user_flag_set(pGapEvt->conn_handle, s_FlagAllowRepairing, false);
		SecProcStart(pGapEvt->conn_handle, true,
					 pGapEvt->params.sec_params_request.peer_params.bond ?
					 PM_CONN_SEC_PROCEDURE_BONDING : PM_CONN_SEC_PROCEDURE_PAIRING);
	}
	ParamsRequestProcess(pGapEvt->conn_handle,
						 &pGapEvt->params.sec_params_request.peer_params);
}

static void SecInfoRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	const ble_gap_enc_info_t *pEncInfo = NULL;
	pm_peer_data_flash_t      peerData;

	pm_peer_id_t peerId = im_peer_id_get_by_master_id(
							(ble_gap_master_id_t *)&pGapEvt->params.sec_info_request.master_id);
	if (peerId == PM_PEER_ID_INVALID)
	{
		peerId = im_peer_id_get_by_conn_handle(pGapEvt->conn_handle);
	}
	else
	{
		// The peer may have been unrecognized until now (e.g. random
		// non-resolvable advertising address). Record the discovery.
		im_new_peer_id(pGapEvt->conn_handle, peerId);
	}

	SecProcStart(pGapEvt->conn_handle, true, PM_CONN_SEC_PROCEDURE_ENCRYPTION);

	if (peerId != PM_PEER_ID_INVALID)
	{
		ret_code_t r = pdb_peer_data_ptr_get(peerId, PM_PEER_DATA_ID_BONDING, &peerData);
		if (r == NRF_SUCCESS)
		{
			// Reply with own LTK when it is the one the request names. A LESC
			// LTK has an all-zero master id (nothing to compare, and the
			// compare treats a zero id as invalid), so the lesc flag alone
			// selects it, exactly as the SDK dispatcher does.
			const ble_gap_enc_key_t *pKey = &peerData.p_bonding_data->own_ltk;

			if (pGapEvt->params.sec_info_request.enc_info &&
				(pKey->enc_info.lesc ||
				 im_master_ids_compare(&pKey->master_id,
									   &pGapEvt->params.sec_info_request.master_id)))
			{
				pEncInfo = &pKey->enc_info;
			}
		}
		else if (r != NRF_ERROR_NOT_FOUND)
		{
			UnexpectedErrorSend(pGapEvt->conn_handle, r);
		}
	}

	ret_code_t r = sd_ble_gap_sec_info_reply(pGapEvt->conn_handle, pEncInfo, NULL, NULL);
	if (r == NRF_ERROR_INVALID_STATE)
	{
		// Another module already replied; nothing to do.
	}
	else if (r != NRF_SUCCESS)
	{
		UnexpectedErrorSend(pGapEvt->conn_handle, r);
	}
	else if (pGapEvt->params.sec_info_request.enc_info && pEncInfo == NULL)
	{
		SecFailureSend(pGapEvt->conn_handle, PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING,
					   BLE_GAP_SEC_STATUS_SOURCE_LOCAL);
	}
}

// Central receives a Security Request from the peripheral.
static void SecRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	bool bForceRepairing = false;
	bool bNullParams = (s_pSecParams == NULL);

	if (!bNullParams && ble_conn_state_encrypted(pGapEvt->conn_handle))
	{
		pm_conn_sec_status_t req;

		memset(&req, 0, sizeof(req));
		req.bonded         = pGapEvt->params.sec_request.bond;
		req.mitm_protected = pGapEvt->params.sec_request.mitm;
		req.lesc           = pGapEvt->params.sec_request.lesc;
		bForceRepairing    = !sm_sec_is_sufficient(pGapEvt->conn_handle, &req);
	}

	(void)LinkSecure(pGapEvt->conn_handle, bNullParams, bForceRepairing, true);

	// Forward the request to the application, as the SDK does.
	pm_evt_t evt = NewEvt(PM_EVT_SLAVE_SECURITY_REQ, pGapEvt->conn_handle);
	evt.params.slave_security_req.bond = pGapEvt->params.sec_request.bond;
	evt.params.slave_security_req.mitm = pGapEvt->params.sec_request.mitm;
	EvtSend(&evt);
}

static void PairingSuccessSend(const ble_gap_evt_t *pGapEvt, bool bDataStored)
{
	ble_conn_state_user_flag_set(pGapEvt->conn_handle, s_FlagSecProc, false);

	pm_evt_t evt = NewEvt(PM_EVT_CONN_SEC_SUCCEEDED, pGapEvt->conn_handle);
	evt.params.conn_sec_succeeded.procedure = pGapEvt->params.auth_status.bonded ?
			PM_CONN_SEC_PROCEDURE_BONDING : PM_CONN_SEC_PROCEDURE_PAIRING;
	evt.params.conn_sec_succeeded.data_stored = bDataStored;
	EvtSend(&evt);
}

// Commit the bond. NRF_ERROR_BUSY (peer data queue full) defers the store and
// the retry pumps repeat it; the terminal pairing event is emitted only when
// the store reaches a terminal result, so data_stored is truthful.
static void BondStoreAttempt(uint16_t ConnHdl, pm_peer_id_t PeerId, bool bNewPeer)
{
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	ret_code_t r = pdb_write_buf_store(PDB_TEMP_PEER_ID(ConnHdl),
									   PM_PEER_DATA_ID_BONDING, PeerId);

	if (r == NRF_ERROR_BUSY && pLink != nullptr)
	{
		pLink->bStorePending = true;
		pLink->bStoreNewPeer = bNewPeer;
		pLink->StorePeerId   = PeerId;
		return;			// retried by the pumps; no terminal event yet
	}

	if (pLink != nullptr)
	{
		pLink->bStorePending = false;
	}

	ble_conn_state_user_flag_set(ConnHdl, s_FlagSecProc, false);

	pm_evt_t evt = NewEvt(PM_EVT_CONN_SEC_SUCCEEDED, ConnHdl);
	evt.params.conn_sec_succeeded.procedure   = PM_CONN_SEC_PROCEDURE_BONDING;
	evt.params.conn_sec_succeeded.data_stored = (r == NRF_SUCCESS);
	if (r != NRF_SUCCESS)
	{
		UnexpectedErrorSend(ConnHdl, r);
		if (bNewPeer)
		{
			(void)im_peer_free(PeerId);
		}
	}
	EvtSend(&evt);
}

static void AuthStatusSuccessProcess(const ble_gap_evt_t *pGapEvt)
{
	uint16_t connHdl = pGapEvt->conn_handle;
	BtSecSdLink_t *pKsLink = LinkGet(connHdl);

	if (pKsLink != nullptr && pKsLink->EncrKeySize != 0 &&
		pKsLink->EncrKeySize < s_MinKeySize)
	{
		// The key size check already failed the procedure and requested the
		// disconnect; do not store a bond keyed below policy.
		(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(connHdl), PM_PEER_DATA_ID_BONDING);
		return;
	}

	if (!pGapEvt->params.auth_status.bonded)
	{
		// Pairing without bonding: nothing to store. Release the buffer that
		// was prepared at the params reply.
		(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(connHdl), PM_PEER_DATA_ID_BONDING);
		PairingSuccessSend(pGapEvt, false);
		return;
	}

	// Locate or allocate the peer id for the new bond.
	bool         bNewPeerId = false;
	pm_peer_data_t peerData;
	pm_peer_id_t peerId = im_peer_id_get_by_conn_handle(connHdl);

	ret_code_t r = pdb_write_buf_get(PDB_TEMP_PEER_ID(connHdl),
									 PM_PEER_DATA_ID_BONDING, 1, &peerData);
	if (r != NRF_SUCCESS)
	{
		UnexpectedErrorSend(connHdl, r);
		PairingSuccessSend(pGapEvt, false);
		return;
	}

	if (peerId == PM_PEER_ID_INVALID)
	{
		// Repairing detection: an existing bond for this identity.
		peerId = im_find_duplicate_bonding_data(peerData.p_bonding_data, PM_PEER_ID_INVALID);
		if (peerId != PM_PEER_ID_INVALID)
		{
			// Known identity re-pairing. Map the connection to the existing
			// peer first (the SDK does), so peer lookups work even when the
			// application denies. Then ask unless a decision was already made
			// for this procedure; default deny. On deny the old bond record is
			// kept, and the live link stays encrypted with the newly
			// negotiated key until the application disconnects: the SMP
			// exchange completed inside the SoftDevice before the peer
			// identity could be known, so denial after the fact is the only
			// option this stack offers, matching the SDK.
			im_new_peer_id(connHdl, peerId);

			if (!ble_conn_state_user_flag_get(connHdl, s_FlagAllowRepairing))
			{
				pm_evt_t evt = NewEvt(PM_EVT_CONN_SEC_CONFIG_REQ, connHdl);
				evt.peer_id = peerId;
				EvtSend(&evt);

				if (!ble_conn_state_user_flag_get(connHdl, s_FlagAllowRepairing))
				{
					// Denied: drop the new bond data, keep the old record, and
					// disconnect. The link is encrypted with a key the
					// application refused, so leaving it up and reporting
					// success (the SDK behavior) is wrong. The procedure flag
					// stays set; the disconnect path reports CONN_SEC_FAILED.
					(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(connHdl),
												PM_PEER_DATA_ID_BONDING);
					(void)sd_ble_gap_disconnect(connHdl,
												BLE_HCI_AUTHENTICATION_FAILURE);
					return;
				}
			}
		}
	}

	if (peerId == PM_PEER_ID_INVALID)
	{
		peerId = pds_peer_id_allocate();
		if (peerId == PM_PEER_ID_INVALID)
		{
			(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(connHdl),
										 PM_PEER_DATA_ID_BONDING);
			UnexpectedErrorSend(connHdl, NRF_ERROR_NO_MEM);
			PairingSuccessSend(pGapEvt, false);
			return;
		}
		bNewPeerId = true;
		im_new_peer_id(connHdl, peerId);
	}

	BondStoreAttempt(connHdl, peerId, bNewPeerId);
}

static void AuthStatusProcess(const ble_gap_evt_t *pGapEvt)
{
	LinkReplyClear(pGapEvt->conn_handle);

	if (pGapEvt->params.auth_status.auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
	{
		AuthStatusSuccessProcess(pGapEvt);
	}
	else
	{
		(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(pGapEvt->conn_handle),
									PM_PEER_DATA_ID_BONDING);
		SecFailureSend(pGapEvt->conn_handle,
					   pGapEvt->params.auth_status.auth_status,
					   pGapEvt->params.auth_status.error_src);
#if PM_RA_PROTECTION_ENABLED
		ast_auth_error_notify(pGapEvt->conn_handle);
#endif
	}
}

static void ConnSecUpdateProcess(const ble_gap_evt_t *pGapEvt)
{
	uint8_t keySize = pGapEvt->params.conn_sec_update.conn_sec.encr_key_size;
	BtSecSdLink_t *pLink = LinkGet(pGapEvt->conn_handle);

	if (pLink != nullptr)
	{
		pLink->EncrKeySize = keySize;
	}

	if (ble_conn_state_encrypted(pGapEvt->conn_handle) && keySize < s_MinKeySize)
	{
		// Under-keyed link: fail the procedure and disconnect regardless of
		// whether this is a fresh pairing or a reconnect.
		SecFailureSend(pGapEvt->conn_handle, BLE_GAP_SEC_STATUS_ENC_KEY_SIZE,
					   BLE_GAP_SEC_STATUS_SOURCE_LOCAL);
		(void)sd_ble_gap_disconnect(pGapEvt->conn_handle,
									BLE_HCI_AUTHENTICATION_FAILURE);
		return;
	}

	if (ProcIsPairing(pGapEvt->conn_handle))
	{
		return;		// pairing completion is reported at AUTH_STATUS
	}

	if (!ble_conn_state_encrypted(pGapEvt->conn_handle))
	{
		SecFailureSend(pGapEvt->conn_handle, PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING,
					   BLE_GAP_SEC_STATUS_SOURCE_REMOTE);
		return;
	}

	ble_conn_state_user_flag_set(pGapEvt->conn_handle, s_FlagSecProc, false);

	pm_evt_t evt = NewEvt(PM_EVT_CONN_SEC_SUCCEEDED, pGapEvt->conn_handle);
	evt.params.conn_sec_succeeded.procedure   = PM_CONN_SEC_PROCEDURE_ENCRYPTION;
	evt.params.conn_sec_succeeded.data_stored = false;
	EvtSend(&evt);
}

static void DisconnectProcess(const ble_gap_evt_t *pGapEvt)
{
	pm_sec_error_code_t error =
		(pGapEvt->params.disconnected.reason == BLE_HCI_CONN_TERMINATED_DUE_TO_MIC_FAILURE) ?
		PM_CONN_SEC_ERROR_MIC_FAILURE : PM_CONN_SEC_ERROR_DISCONNECT;

	SecFailureSend(pGapEvt->conn_handle, error, BLE_GAP_SEC_STATUS_SOURCE_LOCAL);
	LinkReplyClear(pGapEvt->conn_handle);

	BtSecSdLink_t *pLink = LinkGet(pGapEvt->conn_handle);
	if (pLink != nullptr && pLink->bStorePending)
	{
		pLink->bStorePending = false;
		if (pLink->bStoreNewPeer)
		{
			(void)im_peer_free(pLink->StorePeerId);
		}
	}
	(void)pdb_write_buf_release(PDB_TEMP_PEER_ID(pGapEvt->conn_handle),
								PM_PEER_DATA_ID_BONDING);
}

// ---- Retry pumps ------------------------------------------------------------

static void ReplyPendingHandle(uint16_t ConnHdl, void *pCtx)
{
	(void)pCtx;
	ble_gap_sec_params_t *pParams = s_pSecParams;
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);

	if (pLink != nullptr && pLink->bReplyValid)
	{
		// Replay the application reply exactly, including a rejection.
		pParams = pLink->bReplyReject ? NULL : &pLink->ReplyParams;
	}
	(void)ReplyAttempt(ConnHdl, pParams);
}

static void SecurePendingHandle(uint16_t ConnHdl, void *pCtx)
{
	(void)pCtx;
	bool bForce = ble_conn_state_user_flag_get(ConnHdl, s_FlagSecureForceRepair);
	bool bNull  = ble_conn_state_user_flag_get(ConnHdl, s_FlagSecureNullParams);

	(void)LinkSecure(ConnHdl, bNull, bForce, true);
}

static void PendingPumpsRun(void)
{
	(void)ble_conn_state_for_each_set_user_flag(s_FlagReplyPendBusy,
											ReplyPendingHandle, NULL);
	(void)ble_conn_state_for_each_set_user_flag(s_FlagSecurePendBusy,
											SecurePendingHandle, NULL);

	for (uint16_t h = 0; h < NRF_SDH_BLE_TOTAL_LINK_COUNT; h++)
	{
		if (s_Links[h].bStorePending && ble_conn_state_valid(h))
		{
			s_Links[h].bStorePending = false;
			BondStoreAttempt(h, s_Links[h].StorePeerId, s_Links[h].bStoreNewPeer);
		}
	}
}

// ---- sm_* API surface (called by peer_manager.c) ----------------------------

extern "C" {

// pm_init calls smd_init right after sm_init. Everything the dispatcher
// initialized lives in this module and is set up in sm_init, so this only
// checks the ordering held.
ret_code_t smd_init(void)
{
	return s_bInit ? NRF_SUCCESS : NRF_ERROR_INVALID_STATE;
}

ret_code_t sm_init(void)
{
	if (s_bInit)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	if (!BtLescInit())
	{
		return NRF_ERROR_INTERNAL;
	}

	s_FlagSecProc           = ble_conn_state_user_flag_acquire();
	s_FlagSecProcPairing    = ble_conn_state_user_flag_acquire();
	s_FlagSecProcBonding    = ble_conn_state_user_flag_acquire();
	s_FlagAllowRepairing    = ble_conn_state_user_flag_acquire();
	s_FlagSecurePendBusy    = ble_conn_state_user_flag_acquire();
	s_FlagSecureForceRepair = ble_conn_state_user_flag_acquire();
	s_FlagSecureNullParams  = ble_conn_state_user_flag_acquire();
	s_FlagReplyPendBusy     = ble_conn_state_user_flag_acquire();

	if (s_FlagReplyPendBusy == BLE_CONN_STATE_USER_FLAG_INVALID)
	{
		return NRF_ERROR_INTERNAL;	// raise BLE_CONN_STATE_USER_FLAG_COUNT
	}

#if PM_RA_PROTECTION_ENABLED
	ret_code_t r = ast_init();
	if (r != NRF_SUCCESS)
	{
		return r;
	}
#endif

	s_bInit = true;
	return NRF_SUCCESS;
}

void sm_ble_evt_handler(ble_evt_t const * p_ble_evt)
{
	const ble_gap_evt_t *pGapEvt = &p_ble_evt->evt.gap_evt;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
	{
		// Connection handles are recycled by the SoftDevice. Reset the whole
		// per-link record so no state from a previous connection on this
		// handle (pending reply, deferred store, key size, peer key) can leak
		// into the new one. The structural reset here is the invariant; the
		// per-event clears elsewhere are then belt and braces.
		BtSecSdLink_t *pLink = LinkGet(pGapEvt->conn_handle);
		if (pLink != nullptr)
		{
			memset(pLink, 0, sizeof(*pLink));
		}
		break;
	}

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		SecParamsRequestProcess(pGapEvt);
		break;

	case BLE_GAP_EVT_SEC_INFO_REQUEST:
		SecInfoRequestProcess(pGapEvt);
		break;

	case BLE_GAP_EVT_SEC_REQUEST:
		SecRequestProcess(pGapEvt);
		break;

	case BLE_GAP_EVT_AUTH_STATUS:
		AuthStatusProcess(pGapEvt);
		break;

	case BLE_GAP_EVT_CONN_SEC_UPDATE:
		ConnSecUpdateProcess(pGapEvt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		DisconnectProcess(pGapEvt);
		break;

	default:
		break;
	}

	// LESC key handling: single delivery point into the bt_lesc module.
	BtLescOnBleEvt(p_ble_evt);

	PendingPumpsRun();
}

void sm_pdb_evt_handler(pm_evt_t * p_event)
{
	switch (p_event->evt_id)
	{
	case PM_EVT_FLASH_GARBAGE_COLLECTED:
	case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
	case PM_EVT_PEER_DATA_UPDATE_FAILED:
	case PM_EVT_PEER_DELETE_SUCCEEDED:
	case PM_EVT_PEER_DELETE_FAILED:
		// Storage capacity may have changed; retry pending work.
		PendingPumpsRun();
		break;

	default:
		break;
	}
}

// IOsonata extensions beyond the SDK sm ABI.
void BtSecSdScOnlySet(bool bEnable)
{
	s_bScOnly = bEnable;
}

void BtSecSdMinKeySizeSet(uint8_t Size)
{
	if (Size >= 7 && Size <= 16)
	{
		s_MinKeySize = Size;
	}
}

ret_code_t sm_sec_params_set(ble_gap_sec_params_t * p_sec_params)
{
	if (p_sec_params == NULL)
	{
		s_pSecParams = NULL;
	}
	else
	{
		s_SecParams  = *p_sec_params;
		s_pSecParams = &s_SecParams;
	}
	return NRF_SUCCESS;
}

ret_code_t sm_sec_params_reply(uint16_t conn_handle, ble_gap_sec_params_t * p_sec_params,
							   void const * p_context)
{
	if (p_context == NULL)
	{
		return NRF_ERROR_NULL;
	}

	SecParamsReplyCtx_t *pCtx = (SecParamsReplyCtx_t *)p_context;

	if (p_sec_params == NULL)
	{
		pCtx->pSecParams = NULL;			// reject pairing
	}
	else
	{
		pCtx->SecParamsMem = *p_sec_params;
		pCtx->pSecParams   = &pCtx->SecParamsMem;
	}
	pCtx->bReplyCalled = true;

	ret_code_t r = ReplyAttempt(conn_handle, pCtx->pSecParams);
	if (r == NRF_ERROR_BUSY)
	{
		r = NRF_SUCCESS;	// retried by the pending pump
	}
	return r;
}

void sm_conn_sec_config_reply(uint16_t conn_handle, pm_conn_sec_config_t * p_conn_sec_config)
{
	if (p_conn_sec_config != NULL)
	{
		ble_conn_state_user_flag_set(conn_handle, s_FlagAllowRepairing,
									 p_conn_sec_config->allow_repairing);
	}
}

ret_code_t sm_lesc_public_key_set(ble_gap_lesc_p256_pk_t * p_public_key)
{
	// The LESC key pair is owned by the bt_lesc module; an externally
	// supplied key is not supported.
	(void)p_public_key;
	return NRF_ERROR_FORBIDDEN;
}

ret_code_t sm_conn_sec_status_get(uint16_t conn_handle, pm_conn_sec_status_t * p_conn_sec_status)
{
	if (p_conn_sec_status == NULL)
	{
		return NRF_ERROR_NULL;
	}

	uint8_t role = ble_conn_state_role(conn_handle);
	if (role == BLE_GAP_ROLE_INVALID)
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

	pm_peer_id_t peerId = im_peer_id_get_by_conn_handle(conn_handle);

	memset(p_conn_sec_status, 0, sizeof(*p_conn_sec_status));
	p_conn_sec_status->connected      = true;
	p_conn_sec_status->encrypted      = ble_conn_state_encrypted(conn_handle);
	p_conn_sec_status->mitm_protected = ble_conn_state_mitm_protected(conn_handle);
	p_conn_sec_status->bonded         = (peerId != PM_PEER_ID_INVALID);
	p_conn_sec_status->lesc           = ble_conn_state_lesc(conn_handle);

	if (!p_conn_sec_status->lesc && peerId != PM_PEER_ID_INVALID &&
		p_conn_sec_status->encrypted)
	{
		// Encrypted from a stored LESC bond: the link flag only reflects a
		// live pairing, so consult the bond record.
		pm_peer_data_flash_t peerData;
		if (pdb_peer_data_ptr_get(peerId, PM_PEER_DATA_ID_BONDING, &peerData) == NRF_SUCCESS)
		{
			p_conn_sec_status->lesc = peerData.p_bonding_data->own_ltk.enc_info.lesc;
		}
	}

	return NRF_SUCCESS;
}

bool sm_sec_is_sufficient(uint16_t conn_handle, pm_conn_sec_status_t * p_sec_status_req)
{
	pm_conn_sec_status_t status;

	if (sm_conn_sec_status_get(conn_handle, &status) != NRF_SUCCESS)
	{
		return false;
	}

	return (!p_sec_status_req->connected      || status.connected) &&
		   (!p_sec_status_req->encrypted      || status.encrypted) &&
		   (!p_sec_status_req->mitm_protected || status.mitm_protected) &&
		   (!p_sec_status_req->bonded         || status.bonded) &&
		   (!p_sec_status_req->lesc           || status.lesc);
}

ret_code_t sm_link_secure(uint16_t conn_handle, bool force_repairing)
{
	if (!s_bInit)
	{
		return NRF_ERROR_INVALID_STATE;
	}
	return LinkSecure(conn_handle, s_pSecParams == NULL, force_repairing, false);
}

}	// extern "C"

#endif // NRF_MODULE_ENABLED(PEER_MANAGER)
