/**-------------------------------------------------------------------------
@file	bt_sec_bm.cpp

@brief	IOsonata security manager for the sdk-nrf-bm SoftDevice path (nRF54L).

		Replaces the sdk-nrf-bm peer_manager modules security_manager.c and
		security_dispatcher.c with one module, the same shape as the nRF52
		implementation bt_sec_sd.cpp. It implements the sm_* API surface peer_manager.c
		calls (security_manager.h), provides smd_init (peer_manager.c calls it
		right after sm_init), emits the same struct pm_evt events through
		pm_sm_evt_handler, and drives the S145 SoftDevice GAP security
		procedures directly. The peer data layer stays in place: peer_database,
		peer_id and the IOsonata peer_data_storage replacement are used
		unchanged, so bond records keep their layout.

		LESC runs on the IOsonata BtLesc module (CryptoDev_t based)
		under CONFIG_PM_LESC, exactly where the stock module called it: init
		in sm_init, the public key at the params reply, event delivery at the
		end of sm_ble_evt_handler. DHKey computation stays deferred to
		BtLescRequestHandler in the application main loop.

		The keyset handed to sd_ble_gap_sec_params_reply points into the
		pm_peer_data_bonding inside the peer_database write buffer, so the
		SoftDevice writes the distributed keys straight into the structure the
		store commits at AUTH_STATUS with pdb_write_buf_store.

		Fixes carried over from the nRF52 implementation, all applying here too:
		- Per-connection state record. The stock dispatcher shares one static
		  LESC peer public key buffer across links, so concurrent pairings
		  overwrite each other. Here each link owns its receive buffer.
		- The pending params reply preserves the exact application answer,
		  including a rejection, across an NRF_ERROR_BUSY retry; the stock
		  module loses it.
		- Re-pairing denied at AUTH_STATUS disconnects the link instead of
		  reporting success while it stays encrypted with a key the
		  application refused. The early reject at params-reply time (a stock
		  sdk-nrf-bm improvement over the frozen nRF5 SDK) is kept as well.
		- SIG policy the stock module never offered: Secure Connections Only
		  rejects legacy pairing with Authentication Requirements (Core Vol 3
		  Part H 2.3.5.1); the minimum encryption key size is checked on
		  every CONN_SEC_UPDATE and an under-keyed link is failed and
		  disconnected (KNOB class downgrade mitigation).
		- Connection handle recycling: the whole per-link record is reset at
		  BLE_GAP_EVT_CONNECTED.
		- A bond store hitting NRF_ERROR_BUSY is deferred and retried by the
		  pending pumps, so the terminal pairing event reports a truthful
		  data_stored. NRF_ERROR_RESOURCES emits PM_EVT_STORAGE_FULL and
		  reports the store as accepted, matching the stock module.

		Divergence from stock kept on purpose: a central securing a link with
		a known peer id but no readable bond data (NRF_ERROR_NOT_FOUND) falls
		back to pairing instead of failing, matching the nRF52 implementation.

		Both roles are implemented, each gated by its SoftDevice variant
		option (CONFIG_SOFTDEVICE_PERIPHERAL / CONFIG_SOFTDEVICE_CENTRAL).
		Runs entirely in the pm BLE event dispatch context, the same as the
		stock modules it replaces.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include <nrf_error.h>
#include <ble.h>
#include <ble_gap.h>
#include <ble_err.h>
#include <ble_hci.h>

#include <zephyr/logging/log.h>

#include <bm/softdevice_handler/nrf_sdh_ble.h>
#include <bm/bluetooth/peer_manager/peer_manager_types.h>

#include <modules/peer_manager_internal.h>
#include <modules/security_manager.h>		// the sm_* API this module implements
#include <modules/security_dispatcher.h>	// smd_init, called by pm_init; the rest
											// of the smd layer is absorbed here
#include <modules/conn_state.h>
#include <modules/peer_database.h>
#include <modules/peer_data_storage.h>
#include <modules/id_manager.h>

#if defined(CONFIG_PM_RA_PROTECTION)
#include <modules/auth_status_tracker.h>
#endif

#if defined(CONFIG_PM_LESC)
#include "bt_lesc.h"
#endif

LOG_MODULE_DECLARE(peer_manager, CONFIG_PEER_MANAGER_LOG_LEVEL);

// Event sink in peer_manager.c; the same extern hookup the stock sm uses.
extern "C" void pm_sm_evt_handler(struct pm_evt *sm_evt);

// The context type used in PM_EVT_CONN_SEC_PARAMS_REQ events and in calls to
// sm_sec_params_reply(). The application interacts with it only through
// sm_sec_params_reply.
typedef struct {
	ble_gap_sec_params_t *pSecParams;		// params to use in the reply
	ble_gap_sec_params_t  SecParamsMem;		// buffer holding the params
	bool                  bReplyCalled;		// sm_sec_params_reply was called
} SecParamsReplyCtx_t;

static bool                  s_bInit;
static ble_gap_sec_params_t  s_SecParams;	// default sec params buffer
static ble_gap_sec_params_t *s_pSecParams;	// NULL until sm_sec_params_set

#if !defined(CONFIG_PM_LESC)
// Application supplied LESC public key, stock behavior when the lesc module
// is not part of the build.
static ble_gap_lesc_p256_pk_t *s_pUserLescPk;
#endif

// SIG policy the stock module never offered. See the file header.
static bool    s_bScOnly;
static uint8_t s_MinKeySize = 7;

// Per-connection state, indexed by nrf_sdh_ble_idx_get (S145 connection
// handles are not guaranteed dense). PeerPk is the LESC peer public key
// receive buffer for the keyset: per link, so concurrent pairings cannot
// overwrite each other (the stock dispatcher shares one static buffer).
// The pending reply record preserves the exact application answer, including
// a rejection, across an NRF_ERROR_BUSY retry.
typedef struct {
	ble_gap_lesc_p256_pk_t PeerPk;			// SoftDevice writes the peer key here
	ble_gap_sec_params_t   ReplyParams;		// pending reply parameters
	bool                   bReplyValid;		// a reply is pending retry
	bool                   bReplyReject;	// the pending reply is a rejection
	bool                   bStorePending;	// bond store deferred on busy
	bool                   bStoreNewPeer;	// the pending store allocated the peer id
	uint16_t               StorePeerId;		// peer id of the pending store
	uint16_t               StoreConnHdl;	// connection handle of the pending store
	uint8_t                EncrKeySize;		// negotiated key size, from CONN_SEC_UPDATE
} BtSecBmLink_t;

static BtSecBmLink_t s_Links[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];

static inline BtSecBmLink_t *LinkGet(uint16_t ConnHdl)
{
	const int idx = nrf_sdh_ble_idx_get(ConnHdl);

	return (idx >= 0 && idx < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT) ?
		   &s_Links[idx] : nullptr;
}

static void LinkReplyClear(uint16_t ConnHdl)
{
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
	if (pLink != nullptr)
	{
		pLink->bReplyValid  = false;
		pLink->bReplyReject = false;
	}
}

// Procedure bookkeeping and retry flags, one bit per connection.
static int s_FlagSecProc           = PM_CONN_STATE_USER_FLAG_INVALID;
static int s_FlagSecProcPairing    = PM_CONN_STATE_USER_FLAG_INVALID;
static int s_FlagSecProcBonding    = PM_CONN_STATE_USER_FLAG_INVALID;
static int s_FlagAllowRepairing    = PM_CONN_STATE_USER_FLAG_INVALID;
static int s_FlagSecurePendBusy    = PM_CONN_STATE_USER_FLAG_INVALID;
static int s_FlagSecureForceRepair = PM_CONN_STATE_USER_FLAG_INVALID;
static int s_FlagSecureNullParams  = PM_CONN_STATE_USER_FLAG_INVALID;
static int s_FlagReplyPendBusy     = PM_CONN_STATE_USER_FLAG_INVALID;

// ---- Event emission ---------------------------------------------------------

static struct pm_evt NewEvt(enum pm_evt_id EvtId, uint16_t ConnHdl)
{
	struct pm_evt evt;

	memset(&evt, 0, sizeof(evt));
	evt.evt_id      = EvtId;
	evt.conn_handle = ConnHdl;
	evt.peer_id     = im_peer_id_get_by_conn_handle(ConnHdl);

	return evt;
}

static void EvtSend(struct pm_evt *pEvt)
{
	pm_sm_evt_handler(pEvt);
}

static void UnexpectedErrorSend(uint16_t ConnHdl, uint32_t ErrCode)
{
	struct pm_evt evt = NewEvt(PM_EVT_ERROR_UNEXPECTED, ConnHdl);

	evt.error_unexpected.error = ErrCode;
	EvtSend(&evt);
}

static void StorageFullSend(uint16_t ConnHdl)
{
	struct pm_evt evt = NewEvt(PM_EVT_STORAGE_FULL, ConnHdl);

	EvtSend(&evt);
}

// ---- Procedure bookkeeping --------------------------------------------------

static void SecProcStart(uint16_t ConnHdl, bool bSuccess, enum pm_conn_sec_procedure Procedure)
{
	pm_conn_state_user_flag_set(ConnHdl, s_FlagSecProc, bSuccess);
	if (bSuccess)
	{
		pm_conn_state_user_flag_set(ConnHdl, s_FlagSecProcPairing,
									Procedure != PM_CONN_SEC_PROCEDURE_ENCRYPTION);
		pm_conn_state_user_flag_set(ConnHdl, s_FlagSecProcBonding,
									Procedure == PM_CONN_SEC_PROCEDURE_BONDING);

		struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_START, ConnHdl);
		evt.conn_sec_start.procedure = Procedure;
		EvtSend(&evt);
	}
}

// True while a pairing (not encryption-only) procedure runs on the link.
static bool ProcIsPairing(uint16_t ConnHdl)
{
	return pm_conn_state_user_flag_get(ConnHdl, s_FlagSecProc) &&
		   pm_conn_state_user_flag_get(ConnHdl, s_FlagSecProcPairing);
}

static enum pm_conn_sec_procedure ProcGet(uint16_t ConnHdl)
{
	if (!pm_conn_state_user_flag_get(ConnHdl, s_FlagSecProcPairing))
	{
		return PM_CONN_SEC_PROCEDURE_ENCRYPTION;
	}
	return pm_conn_state_user_flag_get(ConnHdl, s_FlagSecProcBonding) ?
		   PM_CONN_SEC_PROCEDURE_BONDING : PM_CONN_SEC_PROCEDURE_PAIRING;
}

// Common failure emission: report CONN_SEC_FAILED with the in-flight procedure
// and clear the procedure flag.
static void SecFailureSend(uint16_t ConnHdl, uint16_t Error, uint8_t ErrorSrc)
{
	if (!pm_conn_state_user_flag_get(ConnHdl, s_FlagSecProc))
	{
		return;		// no procedure in flight; nothing to report
	}
	pm_conn_state_user_flag_set(ConnHdl, s_FlagSecProc, false);

	struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_FAILED, ConnHdl);
	evt.conn_sec_failed.procedure = ProcGet(ConnHdl);
	evt.conn_sec_failed.error     = Error;
	evt.conn_sec_failed.error_src = ErrorSrc;
	EvtSend(&evt);
}

static bool AllowRepairing(uint16_t ConnHdl)
{
	return pm_conn_state_user_flag_get(ConnHdl, s_FlagAllowRepairing);
}

static ble_gap_lesc_p256_pk_t *LescPubKeyGet(void)
{
#if defined(CONFIG_PM_LESC)
	return BtLescPubKeyGet();
#else
	return s_pUserLescPk;
#endif
}

static void WriteBufRelease(uint16_t ConnHdl);

// ---- Keyset construction ----------------------------------------------------

// Point the SoftDevice keyset into the pm_peer_data_bonding inside the
// peer_database write buffer, so the distributed keys land directly in the
// structure the store commits. Returns NRF_SUCCESS, NRF_ERROR_BUSY (no buffer
// yet, retried by the pending pump) or an internal error.
static uint32_t SecKeysetFill(uint16_t ConnHdl, uint8_t Role,
							  ble_gap_sec_keyset_t *pKeyset)
{
	struct pm_peer_data peerData;
	uint16_t tempPeerId;

	uint32_t r = pdb_temp_peer_id_get(ConnHdl, &tempPeerId);
	if (r == NRF_SUCCESS)
	{
		r = pdb_write_buf_get(tempPeerId, PM_PEER_DATA_ID_BONDING, 1, &peerData);
	}
	if (r == NRF_ERROR_BUSY)
	{
		return r;
	}
	if (r != NRF_SUCCESS)
	{
		return NRF_ERROR_INTERNAL;
	}

	memset(peerData.bonding_data, 0, sizeof(struct pm_peer_data_bonding));
	peerData.bonding_data->own_role = Role;

	pKeyset->keys_own.p_enc_key  = &peerData.bonding_data->own_ltk;
	pKeyset->keys_own.p_pk       = LescPubKeyGet();
	pKeyset->keys_peer.p_enc_key = &peerData.bonding_data->peer_ltk;
	pKeyset->keys_peer.p_id_key  = &peerData.bonding_data->peer_ble_id;

	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
	if (pLink == nullptr)
	{
		(void)pdb_write_buf_release(tempPeerId, PM_PEER_DATA_ID_BONDING);
		return NRF_ERROR_INVALID_STATE;
	}
	pKeyset->keys_peer.p_pk      = &pLink->PeerPk;

	// The address the peer used at connection establishment; overwritten by
	// the identity if the peer distributes one.
	r = im_ble_addr_get(ConnHdl, &peerData.bonding_data->peer_ble_id.id_addr_info);
	if (r != NRF_SUCCESS)
	{
		(void)pdb_write_buf_release(tempPeerId, PM_PEER_DATA_ID_BONDING);
		return NRF_ERROR_INVALID_STATE;
	}

	return NRF_SUCCESS;
}

// ---- sec params reply (SEC_PARAMS_REQUEST answer) ---------------------------

static uint32_t ParamsReplyPerform(uint16_t ConnHdl, ble_gap_sec_params_t *pSecParams)
{
	uint8_t              role = pm_conn_state_role(ConnHdl);
	uint8_t              secStatus = BLE_GAP_SEC_STATUS_SUCCESS;
	ble_gap_sec_keyset_t keyset;
	uint32_t             r = NRF_SUCCESS;
	bool                 bWriteBufHeld = false;

	memset(&keyset, 0, sizeof(keyset));

	if (role == BLE_GAP_ROLE_INVALID)
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

#if defined(CONFIG_PM_RA_PROTECTION)
	if (ast_peer_deny_listed(ConnHdl))
	{
		secStatus  = BLE_GAP_SEC_STATUS_REPEATED_ATTEMPTS;
		pSecParams = NULL;
	}
	else
#endif
	if (pSecParams == NULL)
	{
		// Reject pairing.
		secStatus = BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP;
	}
	else
	{
#if defined(CONFIG_SOFTDEVICE_PERIPHERAL)
		if (role == BLE_GAP_ROLE_PERIPH && !AllowRepairing(ConnHdl) &&
			im_peer_id_get_by_conn_handle(ConnHdl) != PM_PEER_ID_INVALID)
		{
			// A bond already exists for an identified peer: ask before the
			// pairing runs (early reject, a stock sdk-nrf-bm improvement).
			// Peers only identified after key distribution are still caught
			// at AUTH_STATUS.
			struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_CONFIG_REQ, ConnHdl);
			EvtSend(&evt);
			if (!AllowRepairing(ConnHdl))
			{
				secStatus = BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP;
			}
		}
#endif

		if (secStatus == BLE_GAP_SEC_STATUS_SUCCESS)
		{
			if (!pSecParams->bond)
			{
				// Pairing without bonding: no store buffer, but the LESC
				// public key exchange still needs its buffers.
				BtSecBmLink_t *pLink = LinkGet(ConnHdl);
				if (pLink == nullptr)
				{
					return BLE_ERROR_INVALID_CONN_HANDLE;
				}
				keyset.keys_own.p_pk  = LescPubKeyGet();
				keyset.keys_peer.p_pk = &pLink->PeerPk;
			}
			else
			{
				r = SecKeysetFill(ConnHdl, role, &keyset);
				if (r != NRF_SUCCESS)
				{
					pm_conn_state_user_flag_set(ConnHdl, s_FlagReplyPendBusy,
												r == NRF_ERROR_BUSY);
					return r;
				}
				bWriteBufHeld = true;
			}
		}
	}

	// Peripheral replies with its parameters; a central gave them at
	// sd_ble_gap_authenticate. The keyset is always passed, zeroed on reject.
	ble_gap_sec_params_t *pReplyParams = NULL;
#if defined(CONFIG_SOFTDEVICE_PERIPHERAL)
	if (role == BLE_GAP_ROLE_PERIPH && secStatus == BLE_GAP_SEC_STATUS_SUCCESS)
	{
		pReplyParams = pSecParams;
	}
#endif

	r = sd_ble_gap_sec_params_reply(ConnHdl, secStatus, pReplyParams, &keyset);
	pm_conn_state_user_flag_set(ConnHdl, s_FlagReplyPendBusy, r == NRF_ERROR_BUSY);

	if (bWriteBufHeld && r != NRF_SUCCESS && r != NRF_ERROR_BUSY)
	{
		WriteBufRelease(ConnHdl);
	}

	if (r == NRF_ERROR_INVALID_STATE && !pm_conn_state_valid(ConnHdl))
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
static uint32_t ReplyAttempt(uint16_t ConnHdl, ble_gap_sec_params_t *pSecParams)
{
	uint32_t r = ParamsReplyPerform(ConnHdl, pSecParams);
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);

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

	struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_PARAMS_REQ, ConnHdl);
	evt.conn_sec_params_req.peer_params = pPeerParams;
	evt.conn_sec_params_req.context     = &ctx;
	EvtSend(&evt);

	if (!ctx.bReplyCalled)
	{
		uint32_t r = ReplyAttempt(ConnHdl, ctx.pSecParams);
		if (r != NRF_SUCCESS && r != NRF_ERROR_BUSY)
		{
			UnexpectedErrorSend(ConnHdl, r);
		}
	}
}

// ---- link secure (pm_conn_secure and Security Request) ----------------------

#if defined(CONFIG_SOFTDEVICE_CENTRAL)
static uint32_t LinkSecureCentralEncrypt(uint16_t ConnHdl, uint16_t PeerId)
{
	struct pm_peer_data_bonding bondData;
	struct pm_peer_data peerData;
	const uint32_t bufSize = sizeof(bondData);
	const ble_gap_enc_key_t *pKey = NULL;

	memset(&bondData, 0, sizeof(bondData));
	peerData.bonding_data = &bondData;

	uint32_t r = pds_peer_data_read(PeerId, PM_PEER_DATA_ID_BONDING,
									&peerData, &bufSize);
	if (r == NRF_SUCCESS)
	{
		// Peer distributed its LTK as peripheral; for LESC both sides hold
		// the same LTK under own_ltk.
		pKey = &bondData.peer_ltk;
		if (bondData.own_ltk.enc_info.lesc)
		{
			pKey = &bondData.own_ltk;
		}
		else if (!im_master_id_is_valid(&pKey->master_id))
		{
			pKey = NULL;		// no usable legacy key
		}
	}
	else if (r == NRF_ERROR_BUSY)
	{
		return NRF_ERROR_BUSY;	// storage busy; retried by the pump
	}
	else if (r != NRF_ERROR_NOT_FOUND)
	{
		return NRF_ERROR_INTERNAL;
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
#endif

static uint32_t LinkSecureAuthenticate(uint16_t ConnHdl, ble_gap_sec_params_t *pSecParams)
{
	uint32_t r = sd_ble_gap_authenticate(ConnHdl, pSecParams);

	if (r == NRF_SUCCESS)
	{
#if defined(CONFIG_SOFTDEVICE_CENTRAL)
		if (pm_conn_state_role(ConnHdl) == BLE_GAP_ROLE_CENTRAL && pSecParams != NULL)
		{
			SecProcStart(ConnHdl, true, pSecParams->bond ?
						 PM_CONN_SEC_PROCEDURE_BONDING : PM_CONN_SEC_PROCEDURE_PAIRING);
		}
#endif
		// Peripheral: this sends the Security Request; the procedure starts
		// at SEC_PARAMS_REQUEST when the central responds.
	}
	else if (r == NRF_ERROR_NO_MEM)
	{
		r = NRF_ERROR_BUSY;		// too many concurrent procedures; retried
	}
	return r;
}

static uint32_t LinkSecure(uint16_t ConnHdl, bool bNullParams, bool bForceRepairing,
						   bool bSendEvents)
{
	uint32_t              r;
	ble_gap_sec_params_t *pParams = bNullParams ? NULL : s_pSecParams;
	uint8_t               role = pm_conn_state_role(ConnHdl);

	if (role == BLE_GAP_ROLE_INVALID)
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

#if defined(CONFIG_SOFTDEVICE_CENTRAL)
	if (role == BLE_GAP_ROLE_CENTRAL)
	{
		// Record the repairing decision at procedure start, stock behavior.
		pm_conn_state_user_flag_set(ConnHdl, s_FlagAllowRepairing, bForceRepairing);

		if (!bForceRepairing)
		{
			uint16_t peerId = im_peer_id_get_by_conn_handle(ConnHdl);

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
	}
#else
	(void)bForceRepairing;
#endif

	r = LinkSecureAuthenticate(ConnHdl, pParams);

#if defined(CONFIG_SOFTDEVICE_CENTRAL)
done:
#endif
	// Track retry state for BUSY; remember the call shape for the retry.
	pm_conn_state_user_flag_set(ConnHdl, s_FlagSecurePendBusy, r == NRF_ERROR_BUSY);
	if (r == NRF_ERROR_BUSY)
	{
		pm_conn_state_user_flag_set(ConnHdl, s_FlagSecureForceRepair, bForceRepairing);
		pm_conn_state_user_flag_set(ConnHdl, s_FlagSecureNullParams, bNullParams);
	}

	if (bSendEvents && r != NRF_SUCCESS && r != NRF_ERROR_BUSY &&
		r != NRF_ERROR_INVALID_STATE)
	{
		if (r == NRF_ERROR_TIMEOUT)
		{
			struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_FAILED, ConnHdl);
			evt.conn_sec_failed.procedure =
				(pParams != NULL && pParams->bond) ? PM_CONN_SEC_PROCEDURE_BONDING
												   : PM_CONN_SEC_PROCEDURE_PAIRING;
			evt.conn_sec_failed.error     = PM_CONN_SEC_ERROR_SMP_TIMEOUT;
			evt.conn_sec_failed.error_src = BLE_GAP_SEC_STATUS_SOURCE_LOCAL;
			EvtSend(&evt);
		}
		else
		{
			UnexpectedErrorSend(ConnHdl, r);
		}
	}

	if (r == NRF_ERROR_BUSY ||
		(r == NRF_ERROR_INVALID_STATE && !pm_conn_state_valid(ConnHdl)))
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

#if defined(CONFIG_SOFTDEVICE_PERIPHERAL)
	if (pm_conn_state_role(pGapEvt->conn_handle) == BLE_GAP_ROLE_PERIPH)
	{
		// New security procedure: reset the repairing decision for the link.
		pm_conn_state_user_flag_set(pGapEvt->conn_handle, s_FlagAllowRepairing, false);
		SecProcStart(pGapEvt->conn_handle, true,
					 pGapEvt->params.sec_params_request.peer_params.bond ?
					 PM_CONN_SEC_PROCEDURE_BONDING : PM_CONN_SEC_PROCEDURE_PAIRING);
	}
#endif
	ParamsRequestProcess(pGapEvt->conn_handle,
						 &pGapEvt->params.sec_params_request.peer_params);
}

#if defined(CONFIG_SOFTDEVICE_PERIPHERAL)
static void SecInfoRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	const ble_gap_enc_info_t *pEncInfo = NULL;
	struct pm_peer_data_bonding bondData;
	struct pm_peer_data peerData;
	const uint32_t bufSize = sizeof(bondData);

	uint16_t peerId = im_peer_id_get_by_master_id(
						&pGapEvt->params.sec_info_request.master_id);
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
		memset(&bondData, 0, sizeof(bondData));
		peerData.bonding_data = &bondData;

		uint32_t r = pds_peer_data_read(peerId, PM_PEER_DATA_ID_BONDING,
										&peerData, &bufSize);
		if (r == NRF_SUCCESS)
		{
			// Reply with own LTK when it is the one the request names. A LESC
			// LTK has an all-zero master id (nothing to compare, and the
			// compare treats a zero id as invalid), so the lesc flag alone
			// selects it, exactly as the stock dispatcher does.
			const ble_gap_enc_key_t *pKey = &bondData.own_ltk;

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

	// S145 copies the enc info during the call; the stack buffer is fine.
	uint32_t r = sd_ble_gap_sec_info_reply(pGapEvt->conn_handle, pEncInfo);
	if (r == NRF_ERROR_INVALID_STATE)
	{
		// Another module already replied, or the link is going down; the
		// DISCONNECTED handling catches the latter.
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
#endif // CONFIG_SOFTDEVICE_PERIPHERAL

#if defined(CONFIG_SOFTDEVICE_CENTRAL)
// Central receives a Security Request from the peripheral.
static void SecRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	bool bForceRepairing = false;
	bool bNullParams = (s_pSecParams == NULL);

	if (!bNullParams && pm_conn_state_encrypted(pGapEvt->conn_handle))
	{
		struct pm_conn_sec_status req;

		memset(&req, 0, sizeof(req));
		req.bonded         = pGapEvt->params.sec_request.bond;
		req.mitm_protected = pGapEvt->params.sec_request.mitm;
		req.lesc           = pGapEvt->params.sec_request.lesc;
		bForceRepairing    = !sm_sec_is_sufficient(pGapEvt->conn_handle, &req);
	}

	(void)LinkSecure(pGapEvt->conn_handle, bNullParams, bForceRepairing, true);

	// Forward the request to the application, as the stock module does.
	struct pm_evt evt = NewEvt(PM_EVT_PERIPHERAL_SECURITY_REQ, pGapEvt->conn_handle);
	evt.peripheral_security_req = pGapEvt->params.sec_request;
	EvtSend(&evt);
}
#endif // CONFIG_SOFTDEVICE_CENTRAL

static void PairingSuccessSend(const ble_gap_evt_t *pGapEvt, bool bDataStored)
{
	pm_conn_state_user_flag_set(pGapEvt->conn_handle, s_FlagSecProc, false);

	struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_SUCCEEDED, pGapEvt->conn_handle);
	evt.conn_sec_succeeded.procedure = pGapEvt->params.auth_status.bonded ?
			PM_CONN_SEC_PROCEDURE_BONDING : PM_CONN_SEC_PROCEDURE_PAIRING;
	evt.conn_sec_succeeded.data_stored = bDataStored;
	EvtSend(&evt);
}

// Commit the bond. NRF_ERROR_BUSY defers the store and the retry pumps repeat
// it; the terminal pairing event is emitted only when the store reaches a
// terminal result, so data_stored is truthful. NRF_ERROR_RESOURCES means the
// store was accepted but the storage needs maintenance: PM_EVT_STORAGE_FULL is
// emitted and the store reported as done, matching the stock module.
static void BondStoreAttempt(uint16_t ConnHdl, uint16_t PeerId, bool bNewPeer)
{
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
	uint16_t tempPeerId;

	uint32_t r = pdb_temp_peer_id_get(ConnHdl, &tempPeerId);
	if (r == NRF_SUCCESS)
	{
		r = pdb_write_buf_store(tempPeerId, PM_PEER_DATA_ID_BONDING, PeerId);
	}

	if (r == NRF_ERROR_BUSY && pLink != nullptr)
	{
		pLink->bStorePending = true;
		pLink->bStoreNewPeer = bNewPeer;
		pLink->StorePeerId   = PeerId;
		pLink->StoreConnHdl  = ConnHdl;
		return;			// retried by the pumps; no terminal event yet
	}

	if (pLink != nullptr)
	{
		pLink->bStorePending = false;
	}

	if (r == NRF_ERROR_RESOURCES)
	{
		StorageFullSend(ConnHdl);
		r = NRF_SUCCESS;
	}

	pm_conn_state_user_flag_set(ConnHdl, s_FlagSecProc, false);

	struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_SUCCEEDED, ConnHdl);
	evt.conn_sec_succeeded.procedure   = PM_CONN_SEC_PROCEDURE_BONDING;
	evt.conn_sec_succeeded.data_stored = (r == NRF_SUCCESS);
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

static void WriteBufRelease(uint16_t ConnHdl)
{
	uint16_t tempPeerId;

	if (pdb_temp_peer_id_get(ConnHdl, &tempPeerId) == NRF_SUCCESS)
	{
		(void)pdb_write_buf_release(tempPeerId, PM_PEER_DATA_ID_BONDING);
	}
}

static void AuthStatusSuccessProcess(const ble_gap_evt_t *pGapEvt)
{
	uint16_t connHdl = pGapEvt->conn_handle;
	BtSecBmLink_t *pKsLink = LinkGet(connHdl);

	if (pKsLink != nullptr && pKsLink->EncrKeySize != 0 &&
		pKsLink->EncrKeySize < s_MinKeySize)
	{
		// The key size check already failed the procedure and requested the
		// disconnect; do not store a bond keyed below policy.
		WriteBufRelease(connHdl);
		return;
	}

	if (!pGapEvt->params.auth_status.bonded)
	{
		// Pairing without bonding: nothing was allocated, nothing to store.
		PairingSuccessSend(pGapEvt, false);
		return;
	}

	// Locate or allocate the peer id for the new bond.
	bool bNewPeerId = false;
	struct pm_peer_data peerData;
	uint16_t tempPeerId;
	uint16_t peerId = im_peer_id_get_by_conn_handle(connHdl);

	uint32_t r = pdb_temp_peer_id_get(connHdl, &tempPeerId);
	if (r == NRF_SUCCESS)
	{
		r = pdb_write_buf_get(tempPeerId, PM_PEER_DATA_ID_BONDING, 1, &peerData);
	}
	if (r != NRF_SUCCESS)
	{
		UnexpectedErrorSend(connHdl, r);
		PairingSuccessSend(pGapEvt, false);
		return;
	}

	if (peerId == PM_PEER_ID_INVALID)
	{
		// Repairing detection: an existing bond for this identity.
		peerId = im_find_duplicate_bonding_data(peerData.bonding_data, PM_PEER_ID_INVALID);
		if (peerId != PM_PEER_ID_INVALID)
		{
			// Known identity re-pairing. Map the connection to the existing
			// peer first (the stock module does), so peer lookups work even
			// when the application denies. Then ask unless a decision was
			// already made for this procedure; default deny. On deny the old
			// bond record is kept, and the link is disconnected: it is
			// encrypted with a key the application refused, so leaving it up
			// and reporting success (the stock behavior) is wrong. The
			// procedure flag stays set; the disconnect path reports
			// CONN_SEC_FAILED.
			im_new_peer_id(connHdl, peerId);

			if (!AllowRepairing(connHdl))
			{
				struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_CONFIG_REQ, connHdl);
				evt.peer_id = peerId;
				EvtSend(&evt);

				if (!AllowRepairing(connHdl))
				{
					WriteBufRelease(connHdl);
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
			WriteBufRelease(connHdl);
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
		WriteBufRelease(pGapEvt->conn_handle);
		SecFailureSend(pGapEvt->conn_handle,
					   pGapEvt->params.auth_status.auth_status,
					   pGapEvt->params.auth_status.error_src);
#if defined(CONFIG_PM_RA_PROTECTION)
		ast_auth_error_notify(pGapEvt->conn_handle);
#endif
	}
}

static void ConnSecUpdateProcess(const ble_gap_evt_t *pGapEvt)
{
	uint8_t keySize = pGapEvt->params.conn_sec_update.conn_sec.encr_key_size;
	BtSecBmLink_t *pLink = LinkGet(pGapEvt->conn_handle);

	if (pLink != nullptr)
	{
		pLink->EncrKeySize = keySize;
	}

	if (pm_conn_state_encrypted(pGapEvt->conn_handle) && keySize < s_MinKeySize)
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

	if (!pm_conn_state_encrypted(pGapEvt->conn_handle))
	{
		SecFailureSend(pGapEvt->conn_handle, PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING,
					   BLE_GAP_SEC_STATUS_SOURCE_REMOTE);
		return;
	}

	pm_conn_state_user_flag_set(pGapEvt->conn_handle, s_FlagSecProc, false);

	struct pm_evt evt = NewEvt(PM_EVT_CONN_SEC_SUCCEEDED, pGapEvt->conn_handle);
	evt.conn_sec_succeeded.procedure   = PM_CONN_SEC_PROCEDURE_ENCRYPTION;
	evt.conn_sec_succeeded.data_stored = false;
	EvtSend(&evt);
}

static void DisconnectProcess(const ble_gap_evt_t *pGapEvt)
{
	uint16_t error =
		(pGapEvt->params.disconnected.reason == BLE_HCI_CONN_TERMINATED_DUE_TO_MIC_FAILURE) ?
		PM_CONN_SEC_ERROR_MIC_FAILURE : PM_CONN_SEC_ERROR_DISCONNECT;

	SecFailureSend(pGapEvt->conn_handle, error, BLE_GAP_SEC_STATUS_SOURCE_LOCAL);
	LinkReplyClear(pGapEvt->conn_handle);

	BtSecBmLink_t *pLink = LinkGet(pGapEvt->conn_handle);
	if (pLink != nullptr && pLink->bStorePending)
	{
		pLink->bStorePending = false;
		if (pLink->bStoreNewPeer)
		{
			(void)im_peer_free(pLink->StorePeerId);
		}
	}
	WriteBufRelease(pGapEvt->conn_handle);
}

// ---- Retry pumps ------------------------------------------------------------

static void ReplyPendingHandle(uint16_t ConnHdl, void *pCtx)
{
	(void)pCtx;
	ble_gap_sec_params_t *pParams = s_pSecParams;
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);

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
	bool bForce = pm_conn_state_user_flag_get(ConnHdl, s_FlagSecureForceRepair);
	bool bNull  = pm_conn_state_user_flag_get(ConnHdl, s_FlagSecureNullParams);

	(void)LinkSecure(ConnHdl, bNull, bForce, true);
}

static void PendingPumpsRun(void)
{
	(void)pm_conn_state_for_each_set_user_flag(s_FlagReplyPendBusy,
											   ReplyPendingHandle, NULL);
	(void)pm_conn_state_for_each_set_user_flag(s_FlagSecurePendBusy,
											   SecurePendingHandle, NULL);

	for (int i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_Links[i].bStorePending && pm_conn_state_valid(s_Links[i].StoreConnHdl))
		{
			s_Links[i].bStorePending = false;
			BondStoreAttempt(s_Links[i].StoreConnHdl, s_Links[i].StorePeerId,
							 s_Links[i].bStoreNewPeer);
		}
	}
}

// ---- sm_* API surface (called by peer_manager.c) ----------------------------

extern "C" {

// pm_init calls smd_init right after sm_init. Everything the dispatcher
// initialized lives in this module and is set up in sm_init, so this only
// checks the ordering held.
uint32_t smd_init(void)
{
	return s_bInit ? NRF_SUCCESS : NRF_ERROR_INVALID_STATE;
}

uint32_t sm_init(void)
{
	if (s_bInit)
	{
		return NRF_ERROR_INVALID_STATE;
	}

#if defined(CONFIG_PM_LESC)
	if (!BtLescInit())
	{
		return NRF_ERROR_INTERNAL;
	}
#endif

	s_FlagSecProc           = pm_conn_state_user_flag_acquire();
	s_FlagSecProcPairing    = pm_conn_state_user_flag_acquire();
	s_FlagSecProcBonding    = pm_conn_state_user_flag_acquire();
	s_FlagAllowRepairing    = pm_conn_state_user_flag_acquire();
	s_FlagSecurePendBusy    = pm_conn_state_user_flag_acquire();
	s_FlagSecureForceRepair = pm_conn_state_user_flag_acquire();
	s_FlagSecureNullParams  = pm_conn_state_user_flag_acquire();
	s_FlagReplyPendBusy     = pm_conn_state_user_flag_acquire();

	if (s_FlagReplyPendBusy == PM_CONN_STATE_USER_FLAG_INVALID)
	{
		LOG_ERR("Could not acquire conn_state user flags. Increase "
				"PM_CONN_STATE_USER_FLAG_COUNT in the pm_conn_state module.");
		return NRF_ERROR_INTERNAL;
	}

#if defined(CONFIG_PM_RA_PROTECTION)
	uint32_t r = ast_init();
	if (r != NRF_SUCCESS)
	{
		return r;
	}
#endif

	s_bInit = true;
	return NRF_SUCCESS;
}

void sm_ble_evt_handler(const ble_evt_t *ble_evt)
{
	const ble_gap_evt_t *pGapEvt = &ble_evt->evt.gap_evt;

	switch (ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
	{
		// Connection handles are recycled. Reset the whole per-link record so
		// no state from a previous connection on this slot (pending reply,
		// deferred store, key size, peer key) can leak into the new one. The
		// structural reset here is the invariant; the per-event clears
		// elsewhere are then belt and braces.
		BtSecBmLink_t *pLink = LinkGet(pGapEvt->conn_handle);
		if (pLink != nullptr)
		{
			memset(pLink, 0, sizeof(*pLink));
		}
		break;
	}

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		SecParamsRequestProcess(pGapEvt);
		break;

#if defined(CONFIG_SOFTDEVICE_PERIPHERAL)
	case BLE_GAP_EVT_SEC_INFO_REQUEST:
		SecInfoRequestProcess(pGapEvt);
		break;
#endif

#if defined(CONFIG_SOFTDEVICE_CENTRAL)
	case BLE_GAP_EVT_SEC_REQUEST:
		SecRequestProcess(pGapEvt);
		break;
#endif

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

#if defined(CONFIG_PM_LESC)
	// LESC key handling: single delivery point into the lesc module. DHKey
	// computation stays deferred to BtLescRequestHandler in the
	// application main loop.
	BtLescOnBleEvt(ble_evt);
#endif

	PendingPumpsRun();
}

void sm_pdb_evt_handler(struct pm_evt *event)
{
	switch (event->evt_id)
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

// IOsonata extensions beyond the stock sm ABI.
void BtSecBmScOnlySet(bool bEnable)
{
	s_bScOnly = bEnable;
}

void BtSecBmMinKeySizeSet(uint8_t Size)
{
	if (Size >= 7 && Size <= 16)
	{
		s_MinKeySize = Size;
	}
}

uint32_t sm_sec_params_set(ble_gap_sec_params_t *sec_params)
{
	if (sec_params == NULL)
	{
		s_pSecParams = NULL;
	}
	else
	{
		s_SecParams  = *sec_params;
		s_pSecParams = &s_SecParams;
	}
	return NRF_SUCCESS;
}

uint32_t sm_sec_params_reply(uint16_t conn_handle, ble_gap_sec_params_t *sec_params,
							 const void *context)
{
	if (context == NULL)
	{
		return NRF_ERROR_NULL;
	}

	SecParamsReplyCtx_t *pCtx = (SecParamsReplyCtx_t *)context;

	if (sec_params == NULL)
	{
		pCtx->pSecParams = NULL;			// reject pairing
	}
	else
	{
		pCtx->SecParamsMem = *sec_params;
		pCtx->pSecParams   = &pCtx->SecParamsMem;
	}
	pCtx->bReplyCalled = true;

	uint32_t r = ReplyAttempt(conn_handle, pCtx->pSecParams);
	if (r == NRF_ERROR_BUSY)
	{
		r = NRF_SUCCESS;	// retried by the pending pump
	}
	return r;
}

void sm_conn_sec_config_reply(uint16_t conn_handle, struct pm_conn_sec_config *conn_sec_config)
{
	if (conn_sec_config != NULL)
	{
		pm_conn_state_user_flag_set(conn_handle, s_FlagAllowRepairing,
									conn_sec_config->allow_repairing);
	}
}

uint32_t sm_lesc_public_key_set(ble_gap_lesc_p256_pk_t *public_key)
{
#if defined(CONFIG_PM_LESC)
	// The LESC key pair is owned by the lesc module; an externally supplied
	// key is not supported.
	(void)public_key;
	return NRF_ERROR_FORBIDDEN;
#else
	s_pUserLescPk = public_key;
	return NRF_SUCCESS;
#endif
}

uint32_t sm_conn_sec_status_get(uint16_t conn_handle, struct pm_conn_sec_status *conn_sec_status)
{
	if (conn_sec_status == NULL)
	{
		return NRF_ERROR_NULL;
	}

	uint8_t role = pm_conn_state_role(conn_handle);
	if (role == BLE_GAP_ROLE_INVALID)
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

	uint16_t peerId = im_peer_id_get_by_conn_handle(conn_handle);

	memset(conn_sec_status, 0, sizeof(*conn_sec_status));
	conn_sec_status->connected      = true;
	conn_sec_status->encrypted      = pm_conn_state_encrypted(conn_handle);
	conn_sec_status->mitm_protected = pm_conn_state_mitm_protected(conn_handle);
	conn_sec_status->bonded         = (peerId != PM_PEER_ID_INVALID);
	conn_sec_status->lesc           = pm_conn_state_lesc(conn_handle);

	if (!conn_sec_status->lesc && peerId != PM_PEER_ID_INVALID &&
		conn_sec_status->encrypted)
	{
		// Encrypted from a stored LESC bond: the link flag only reflects a
		// live pairing, so consult the bond record.
		struct pm_peer_data_bonding bondData;
		struct pm_peer_data peerData;
		const uint32_t bufSize = sizeof(bondData);

		memset(&bondData, 0, sizeof(bondData));
		peerData.bonding_data = &bondData;

		if (pds_peer_data_read(peerId, PM_PEER_DATA_ID_BONDING,
							   &peerData, &bufSize) == NRF_SUCCESS)
		{
			conn_sec_status->lesc = bondData.own_ltk.enc_info.lesc;
		}
	}

	return NRF_SUCCESS;
}

bool sm_sec_is_sufficient(uint16_t conn_handle, struct pm_conn_sec_status *sec_status_req)
{
	struct pm_conn_sec_status status;

	if (sm_conn_sec_status_get(conn_handle, &status) != NRF_SUCCESS)
	{
		return false;
	}

	return (!sec_status_req->connected      || status.connected) &&
		   (!sec_status_req->encrypted      || status.encrypted) &&
		   (!sec_status_req->mitm_protected || status.mitm_protected) &&
		   (!sec_status_req->bonded         || status.bonded) &&
		   (!sec_status_req->lesc           || status.lesc);
}

uint32_t sm_link_secure(uint16_t conn_handle, bool force_repairing)
{
	if (!s_bInit)
	{
		return NRF_ERROR_INVALID_STATE;
	}
	return LinkSecure(conn_handle, s_pSecParams == NULL, force_repairing, false);
}

}	// extern "C"
