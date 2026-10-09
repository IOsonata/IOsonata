/**-------------------------------------------------------------------------
@file	bt_sec_sd.cpp

@brief	IOsonata security manager for the nRF5 SDK SoftDevice path.

The SoftDevice owns the on-air SMP state machine. IOsonata owns security
policy, peer state, bond records and persistence. No nRF5 SDK Peer Manager,
security_manager, id_manager, FDS or auth_status_tracker state is used here.

The SoftDevice keyset points into fixed per-link storage. On successful
bonding the resulting keys are converted to BtSmpKeys_t and committed through
the generic IOsonata bond table/PDS path. Repeated-attempt policy is the same
identity-scoped policy used by the generic SMP host.

@author	Hoang Nguyen Hoan
@date	Oct. 9, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nrf_error.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh_ble.h"

#include "bt_lesc.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_pds.h"
#include "bluetooth/bt_smp.h"
#include "crypto/icrypto.h"

/******** For DEBUG Trace ************/
#define DEBUG_ENABLE
#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

typedef struct __Bt_Sec_Sd_Link {
	ble_gap_enc_key_t       OwnEnc;
	ble_gap_enc_key_t       PeerEnc;
	ble_gap_id_key_t        PeerId;
	ble_gap_sign_info_t     OwnSign;
	ble_gap_sign_info_t     PeerSign;
	ble_gap_lesc_p256_pk_t  PeerPk;
	ble_gap_sec_params_t    ReplyParams;
	bool                    bReplyPending;
	bool                    bReplyReject;
	bool                    bSecurePending;
	bool                    bForceRepair;
	bool                    bPairing;
	bool                    bBonding;
	bool                    bSc;
	bool                    bAuthenticated;
	bool                    bSecuredNotified;
	uint8_t                 KeySize;
} BtSecSdLink_t;

static BtSecSdLink_t s_Links[NRF_SDH_BLE_TOTAL_LINK_COUNT];
static ble_gap_sec_params_t s_SecParams;
static bool s_bInit;
static bool s_bScOnly;
static bool s_bBondStore;
static uint8_t s_MinKeySize = BT_SMP_MIN_ENC_KEY_SIZE;

static inline BtSecSdLink_t *LinkGet(uint16_t ConnHdl)
{
	return ConnHdl < NRF_SDH_BLE_TOTAL_LINK_COUNT ? &s_Links[ConnHdl] : nullptr;
}

static uint64_t RandGet(const uint8_t Rand[BLE_GAP_SEC_RAND_LEN])
{
	uint64_t v = 0;
	for (int i = BLE_GAP_SEC_RAND_LEN - 1; i >= 0; i--)
	{
		v = (v << 8) | Rand[i];
	}
	return v;
}

static void RandSet(uint8_t Rand[BLE_GAP_SEC_RAND_LEN], uint64_t Value)
{
	for (int i = 0; i < BLE_GAP_SEC_RAND_LEN; i++)
	{
		Rand[i] = (uint8_t)Value;
		Value >>= 8;
	}
}

static bool KeyPresent(const uint8_t *pKey, size_t Len)
{
	uint8_t v = 0;
	for (size_t i = 0; i < Len; i++)
	{
		v |= pKey[i];
	}
	return v != 0;
}

static bool IsCentral(uint16_t ConnHdl)
{
	return BtPeerRole(ConnHdl) == BT_CONN_ROLE_CENTRAL;
}

static bool IsPeripheral(uint16_t ConnHdl)
{
	return BtPeerRole(ConnHdl) == BT_CONN_ROLE_PERIPHERAL;
}

static void LinkKeyBuffersReset(BtSecSdLink_t *pLink)
{
	if (pLink == nullptr)
	{
		return;
	}
	CryptoSecureWipe(&pLink->OwnEnc, sizeof(pLink->OwnEnc));
	CryptoSecureWipe(&pLink->PeerEnc, sizeof(pLink->PeerEnc));
	CryptoSecureWipe(&pLink->PeerId, sizeof(pLink->PeerId));
	CryptoSecureWipe(&pLink->OwnSign, sizeof(pLink->OwnSign));
	CryptoSecureWipe(&pLink->PeerSign, sizeof(pLink->PeerSign));
	CryptoSecureWipe(&pLink->PeerPk, sizeof(pLink->PeerPk));
	pLink->bSc = false;
	pLink->bAuthenticated = false;
	pLink->KeySize = 0;
}

static void LinkReset(uint16_t ConnHdl)
{
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	if (pLink != nullptr)
	{
		CryptoSecureWipe(pLink, sizeof(*pLink));
	}
}

static void ConnSecClear(uint16_t ConnHdl)
{
	BtConnSec_t sec = {};
	BtGapConnSecSet(ConnHdl, &sec);
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer != nullptr)
	{
		pPeer->bSecure = false;
	}
}

static void LinkSecFromBond(BtSecSdLink_t *pLink, const BtSmpKeys_t *pKeys)
{
	if (pLink == nullptr || pKeys == nullptr || !pKeys->bValid)
	{
		return;
	}
	pLink->bSc = pKeys->bSc;
	pLink->bAuthenticated = pKeys->bAuthenticated;
	pLink->KeySize = pKeys->EncKeySize;
}

static void ConnSecUpdate(uint16_t ConnHdl, const ble_gap_conn_sec_t *pSdSec)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	if (pPeer == nullptr || pSdSec == nullptr)
	{
		return;
	}

	BtConnSec_t sec = {};
	bool encrypted = pSdSec->sec_mode.sm == 1 && pSdSec->sec_mode.lv >= 2;
	if (encrypted)
	{
		sec.KeySize = pSdSec->encr_key_size;
		if (pSdSec->sec_mode.lv >= 4)
		{
			sec.Level = BT_GAP_SEC_LEVEL_LESC_AUTH;
			sec.Flags |= BT_GAP_SEC_FLAG_SC;
		}
		else if (pSdSec->sec_mode.lv >= 3)
		{
			sec.Level = BT_GAP_SEC_LEVEL_ENC_AUTH;
		}
		else
		{
			sec.Level = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
		}

		if (pLink != nullptr && pLink->bSc)
		{
			sec.Flags |= BT_GAP_SEC_FLAG_SC;
		}
		if (BtSmpBonded(ConnHdl))
		{
			sec.Flags |= BT_GAP_SEC_FLAG_BONDED;
		}
	}

	BtGapConnSecSet(ConnHdl, &sec);
	pPeer->bSecure = encrypted;

	if (pLink != nullptr)
	{
		pLink->KeySize = pSdSec->encr_key_size;
	}

	if (encrypted && pSdSec->encr_key_size < s_MinKeySize)
	{
		DEBUG_PRINTF("SEC: key size %u below policy %u, disconnect\r\n",
			(unsigned)pSdSec->encr_key_size, (unsigned)s_MinKeySize);
		(void)sd_ble_gap_disconnect(ConnHdl, BLE_HCI_AUTHENTICATION_FAILURE);
		return;
	}

	if (encrypted)
	{
		BtGattCccdRestoreBonded(ConnHdl);
		if (pLink != nullptr && !pLink->bSecuredNotified)
		{
			pLink->bSecuredNotified = true;
			BtAppEvtSecured(ConnHdl);
		}
	}
}

static void KeysetFill(uint16_t ConnHdl, ble_gap_sec_keyset_t *pKeyset)
{
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	memset(pKeyset, 0, sizeof(*pKeyset));
	if (pLink == nullptr)
	{
		return;
	}

	pKeyset->keys_own.p_pk = BtLescPubKeyGet();
	pKeyset->keys_peer.p_pk = &pLink->PeerPk;

	if (!s_SecParams.bond)
	{
		return;
	}

	pKeyset->keys_own.p_enc_key = &pLink->OwnEnc;
	// S140 requires the peer encryption-key pointer to be NULL for an
	// LE Secure Connections-only procedure. Legacy pairing still needs it
	// because the peer may distribute a distinct LTK for role reversal.
	pKeyset->keys_peer.p_enc_key = s_bScOnly ? nullptr : &pLink->PeerEnc;
	pKeyset->keys_peer.p_id_key = &pLink->PeerId;

	if (s_SecParams.kdist_own.sign)
	{
		pKeyset->keys_own.p_sign_key = &pLink->OwnSign;
	}
	if (s_SecParams.kdist_peer.sign)
	{
		pKeyset->keys_peer.p_sign_key = &pLink->PeerSign;
	}
}

static ret_code_t ParamsReplyAttempt(uint16_t ConnHdl, const ble_gap_sec_params_t *pParams)
{
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	if (pLink == nullptr || BtPeerFindByHdl(ConnHdl) == nullptr)
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

	if (!BtSmpPairingAttemptAllowed(ConnHdl, BtSmpMsTick()))
	{
		// Core Vol 3 Part H 2.3.6: during the waiting interval do not
		// respond to another pairing procedure from the same claimant.
		// Drop the link rather than pinning a SoftDevice procedure until its
		// SMP timer expires.
		(void)sd_ble_gap_disconnect(ConnHdl, BLE_HCI_AUTHENTICATION_FAILURE);
		pLink->bReplyPending = false;
		return NRF_SUCCESS;
	}

	uint8_t secStatus = BLE_GAP_SEC_STATUS_SUCCESS;
	ble_gap_sec_params_t *pReply = const_cast<ble_gap_sec_params_t *>(pParams);
	if (pReply == nullptr)
	{
		secStatus = BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP;
	}
	else if (s_bScOnly && !pReply->lesc)
	{
		secStatus = BLE_GAP_SEC_STATUS_AUTH_REQ;
		pReply = nullptr;
	}

	ble_gap_sec_keyset_t keyset;
	memset(&keyset, 0, sizeof(keyset));
	if (secStatus == BLE_GAP_SEC_STATUS_SUCCESS)
	{
		LinkKeyBuffersReset(pLink);
		KeysetFill(ConnHdl, &keyset);
		if (s_SecParams.lesc && keyset.keys_own.p_pk == nullptr)
		{
			pLink->bReplyPending = true;
			pLink->bReplyReject = false;
			pLink->ReplyParams = *pParams;
			return NRF_ERROR_BUSY;
		}
	}

	// Only a peripheral returns local security parameters here. A central
	// already supplied them to sd_ble_gap_authenticate().
	ble_gap_sec_params_t *pSdParams = IsPeripheral(ConnHdl) ? pReply : nullptr;
	ret_code_t r = sd_ble_gap_sec_params_reply(ConnHdl, secStatus, pSdParams,
		secStatus == BLE_GAP_SEC_STATUS_SUCCESS ? &keyset : nullptr);

	if (r == NRF_ERROR_BUSY)
	{
		pLink->bReplyPending = true;
		pLink->bReplyReject = pParams == nullptr;
		if (pParams != nullptr)
		{
			pLink->ReplyParams = *pParams;
		}
	}
	else
	{
		pLink->bReplyPending = false;
		pLink->bReplyReject = false;
	}
	return r;
}

static void ParamsRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	uint16_t h = pGapEvt->conn_handle;
	BtSecSdLink_t *pLink = LinkGet(h);
	if (pLink == nullptr)
	{
		return;
	}

	const ble_gap_sec_params_t *pPeer = &pGapEvt->params.sec_params_request.peer_params;
	if (!BtSmpPairingAttemptAllowed(h, BtSmpMsTick()))
	{
		(void)sd_ble_gap_disconnect(h, BLE_HCI_AUTHENTICATION_FAILURE);
		return;
	}

	if (pPeer->max_key_size < s_MinKeySize ||
		pPeer->max_key_size > BT_SMP_MAX_ENC_KEY_SIZE ||
		(s_bScOnly && !pPeer->lesc))
	{
		uint8_t st = pPeer->max_key_size < s_MinKeySize ||
			pPeer->max_key_size > BT_SMP_MAX_ENC_KEY_SIZE ?
			BLE_GAP_SEC_STATUS_ENC_KEY_SIZE : BLE_GAP_SEC_STATUS_AUTH_REQ;
		(void)sd_ble_gap_sec_params_reply(h, st, nullptr, nullptr);
		return;
	}

	pLink->bPairing = true;
	pLink->bBonding = pPeer->bond && s_SecParams.bond;
	ret_code_t r = ParamsReplyAttempt(h, &s_SecParams);
	if (r != NRF_SUCCESS && r != NRF_ERROR_BUSY)
	{
		DEBUG_PRINTF("SEC: params reply failed 0x%X\r\n", (unsigned)r);
	}
}

static void KeysFromLink(uint16_t ConnHdl, const ble_gap_evt_auth_status_t *pAuth,
	BtSmpKeys_t *pKeys)
{
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	memset(pKeys, 0, sizeof(*pKeys));
	if (pLink == nullptr)
	{
		return;
	}

	bool sc = pAuth->lesc || pLink->OwnEnc.enc_info.lesc;
	pKeys->bSc = sc;
	pKeys->bAuthenticated = pLink->OwnEnc.enc_info.auth ||
		pLink->PeerEnc.enc_info.auth;
	pKeys->EncKeySize = pLink->KeySize != 0 ? pLink->KeySize :
		(pLink->OwnEnc.enc_info.ltk_len != 0 ? pLink->OwnEnc.enc_info.ltk_len :
		 pLink->PeerEnc.enc_info.ltk_len);

	if (sc)
	{
		memcpy(pKeys->Ltk, pLink->OwnEnc.enc_info.ltk, sizeof(pKeys->Ltk));
		memcpy(pKeys->LocalLtk, pKeys->Ltk, sizeof(pKeys->LocalLtk));
	}
	else
	{
		memcpy(pKeys->Ltk, pLink->PeerEnc.enc_info.ltk, sizeof(pKeys->Ltk));
		pKeys->Rand = RandGet(pLink->PeerEnc.master_id.rand);
		pKeys->Ediv = pLink->PeerEnc.master_id.ediv;
		memcpy(pKeys->LocalLtk, pLink->OwnEnc.enc_info.ltk, sizeof(pKeys->LocalLtk));
		pKeys->LocalRand = RandGet(pLink->OwnEnc.master_id.rand);
		pKeys->LocalEdiv = pLink->OwnEnc.master_id.ediv;
	}

	memcpy(pKeys->Irk, pLink->PeerId.id_info.irk, sizeof(pKeys->Irk));
	pKeys->IdAddrType = pLink->PeerId.id_addr_info.addr_type;
	memcpy(pKeys->IdAddr, pLink->PeerId.id_addr_info.addr, sizeof(pKeys->IdAddr));
	memcpy(pKeys->Csrk, pLink->PeerSign.csrk, sizeof(pKeys->Csrk));

	pKeys->bValid = pKeys->EncKeySize >= BT_SMP_MIN_ENC_KEY_SIZE &&
		pKeys->EncKeySize <= BT_SMP_MAX_ENC_KEY_SIZE &&
		KeyPresent(pKeys->Ltk, sizeof(pKeys->Ltk));
}

static void AuthStatusProcess(const ble_gap_evt_t *pGapEvt)
{
	uint16_t h = pGapEvt->conn_handle;
	BtSecSdLink_t *pLink = LinkGet(h);
	if (pLink == nullptr)
	{
		return;
	}
	pLink->bReplyPending = false;

	const ble_gap_evt_auth_status_t *pAuth = &pGapEvt->params.auth_status;
	if (pAuth->auth_status != BLE_GAP_SEC_STATUS_SUCCESS)
	{
		DEBUG_PRINTF("SEC: pairing failed hdl=%u status=0x%02X src=%u\r\n",
			(unsigned)h, (unsigned)pAuth->auth_status, (unsigned)pAuth->error_src);
		BtSmpPairingAttemptFailed(h, BtSmpMsTick());
		pLink->bPairing = false;
		pLink->bBonding = false;
		return;
	}

	BtSmpPairingAttemptSucceeded(h);
	pLink->bSc = pAuth->lesc != 0;
	pLink->bAuthenticated = pLink->OwnEnc.enc_info.auth ||
		pLink->PeerEnc.enc_info.auth;

	if (pAuth->bonded)
	{
		BtSmpKeys_t keys;
		KeysFromLink(h, pAuth, &keys);
		if (!keys.bValid || !BtSmpBondAdd(h, &keys))
		{
			BtSmpBondStoreFailed(h);
		}
		else
		{
			LinkSecFromBond(pLink, &keys);
		}
		CryptoSecureWipe(&keys, sizeof(keys));
	}

	pLink->bPairing = false;
	pLink->bBonding = false;

	// AUTH_STATUS and CONN_SEC_UPDATE ordering is stack-defined. If the
	// security update already ran, surface completion now; otherwise that
	// event will do it.
	BtDevice_t *pPeer = BtPeerFindByHdl(h);
	if (pPeer != nullptr && pPeer->bSecure && !pLink->bSecuredNotified)
	{
		pLink->bSecuredNotified = true;
		BtAppEvtSecured(h);
	}
}

static void SecInfoRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	uint16_t h = pGapEvt->conn_handle;
	const ble_gap_evt_sec_info_request_t *pReq = &pGapEvt->params.sec_info_request;
	uint64_t rand = RandGet(pReq->master_id.rand);
	uint16_t ediv = pReq->master_id.ediv;
	BtSmpKeys_t keys;
	const ble_gap_enc_info_t *pEnc = nullptr;
	ble_gap_enc_info_t enc;
	memset(&enc, 0, sizeof(enc));
	memset(&keys, 0, sizeof(keys));

	if (pReq->enc_info && BtSmpBondKeysLookup(h, rand, ediv, &keys))
	{
		const uint8_t *pLtk = keys.bSc ? keys.Ltk : keys.LocalLtk;
		if (KeyPresent(pLtk, 16))
		{
			memcpy(enc.ltk, pLtk, 16);
			enc.lesc = keys.bSc;
			enc.auth = keys.bAuthenticated;
			enc.ltk_len = keys.EncKeySize;
			pEnc = &enc;
			LinkSecFromBond(LinkGet(h), &keys);
		}
	}

	ret_code_t r = sd_ble_gap_sec_info_reply(h, pEnc, nullptr, nullptr);
	if (r != NRF_SUCCESS && r != NRF_ERROR_INVALID_STATE)
	{
		DEBUG_PRINTF("SEC: info reply failed 0x%X\r\n", (unsigned)r);
	}
	CryptoSecureWipe(&enc, sizeof(enc));
	CryptoSecureWipe(&keys, sizeof(keys));
}

static bool SecRequestSatisfied(uint16_t ConnHdl, const ble_gap_evt_sec_request_t *pReq)
{
	BtConnSec_t sec;
	if (!BtGapConnSecGet(ConnHdl, &sec) || sec.Level == BT_GAP_SEC_LEVEL_NONE)
	{
		return false;
	}
	if (pReq->bond && !(sec.Flags & BT_GAP_SEC_FLAG_BONDED))
	{
		return false;
	}
	if (pReq->mitm && sec.Level < BT_GAP_SEC_LEVEL_ENC_AUTH)
	{
		return false;
	}
	if (pReq->lesc && !(sec.Flags & BT_GAP_SEC_FLAG_SC))
	{
		return false;
	}
	return true;
}

static ret_code_t EncryptFromBond(uint16_t ConnHdl, const BtSmpKeys_t *pKeys)
{
	ble_gap_enc_info_t enc;
	ble_gap_master_id_t id;
	memset(&enc, 0, sizeof(enc));
	memset(&id, 0, sizeof(id));

	memcpy(enc.ltk, pKeys->Ltk, 16);
	enc.lesc = pKeys->bSc;
	enc.auth = pKeys->bAuthenticated;
	enc.ltk_len = pKeys->EncKeySize;
	if (!pKeys->bSc)
	{
		id.ediv = pKeys->Ediv;
		RandSet(id.rand, pKeys->Rand);
	}

	ret_code_t r = sd_ble_gap_encrypt(ConnHdl, &id, &enc);
	CryptoSecureWipe(&enc, sizeof(enc));
	CryptoSecureWipe(&id, sizeof(id));
	return r;
}

ret_code_t BtSecSdSecure(uint16_t ConnHdl, bool ForceRepair)
{
	BtSecSdLink_t *pLink = LinkGet(ConnHdl);
	if (!s_bInit || pLink == nullptr || BtPeerFindByHdl(ConnHdl) == nullptr)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	if (!BtSmpPairingAttemptAllowed(ConnHdl, BtSmpMsTick()))
	{
		return NRF_ERROR_INVALID_STATE;
	}

	ret_code_t r = NRF_ERROR_NOT_FOUND;
	if (IsCentral(ConnHdl) && !ForceRepair)
	{
		BtSmpKeys_t keys;
		memset(&keys, 0, sizeof(keys));
		if (BtSmpBondKeysLookup(ConnHdl, 0, 0, &keys) && keys.bValid)
		{
			LinkSecFromBond(pLink, &keys);
			r = EncryptFromBond(ConnHdl, &keys);
			CryptoSecureWipe(&keys, sizeof(keys));
			if (r == NRF_SUCCESS)
			{
				pLink->bSecurePending = false;
				return r;
			}
		}
		else
		{
			CryptoSecureWipe(&keys, sizeof(keys));
		}
	}

	r = sd_ble_gap_authenticate(ConnHdl, &s_SecParams);
	if (r == NRF_ERROR_NO_MEM || r == NRF_ERROR_BUSY)
	{
		pLink->bSecurePending = true;
		pLink->bForceRepair = ForceRepair;
		return NRF_ERROR_BUSY;
	}
	pLink->bSecurePending = false;
	if (r == NRF_SUCCESS && IsCentral(ConnHdl))
	{
		pLink->bPairing = true;
		pLink->bBonding = s_SecParams.bond != 0;
	}
	return r;
}

static void SecRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	if (SecRequestSatisfied(pGapEvt->conn_handle, &pGapEvt->params.sec_request))
	{
		return;
	}
	(void)BtSecSdSecure(pGapEvt->conn_handle, true);
}

void BtSecSdBleEvt(const ble_evt_t *pEvt)
{
	if (!s_bInit || pEvt == nullptr)
	{
		return;
	}

	const ble_gap_evt_t *pGapEvt = &pEvt->evt.gap_evt;
	switch (pEvt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			LinkReset(pGapEvt->conn_handle);
			ConnSecClear(pGapEvt->conn_handle);
			break;

		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			ParamsRequestProcess(pGapEvt);
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
			ConnSecUpdate(pGapEvt->conn_handle,
				&pGapEvt->params.conn_sec_update.conn_sec);
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			ConnSecClear(pGapEvt->conn_handle);
			LinkReset(pGapEvt->conn_handle);
			break;

		default:
			break;
	}

	// Single delivery point for the SoftDevice LESC helper. It owns the P-256
	// private-key lifetime and regenerates after every completed or aborted
	// pairing procedure.
	BtLescOnBleEvt(pEvt);
}

void BtSecSdAuthConfig(uint8_t IoCaps, uint8_t AuthReq)
{
	if (IoCaps > BT_SMP_IOCAPS_KEYBOARD_DISPLAY)
	{
		return;
	}
	s_SecParams.io_caps = IoCaps;
	s_SecParams.bond = (AuthReq & BT_SMP_AUTHREQ_BONDING_FLAG_MASK) !=
		BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING;
	s_SecParams.mitm = (AuthReq & BT_SMP_AUTHREQ_MITM) != 0;
	s_SecParams.keypress = (AuthReq & BT_SMP_AUTHREQ_KEYPRESS) != 0;

	// BtSmpAuthConfig is the portable IOsonata SMP configuration entry.
	// The generic host is Secure-Connections-only; keep the vendor-host port
	// on the same policy when this entry point is used.
	s_SecParams.lesc = 1;
	s_bScOnly = true;
}

void BtSecSdOobSet(bool Enable)
{
	s_SecParams.oob = Enable ? 1 : 0;
}

void BtSecSdScOnlySet(bool Enable)
{
	s_bScOnly = Enable;
	if (Enable)
	{
		s_SecParams.lesc = 1;
	}
}

void BtSecSdMinKeySizeSet(uint8_t Size)
{
	if (Size >= BT_SMP_MIN_ENC_KEY_SIZE && Size <= BT_SMP_MAX_ENC_KEY_SIZE)
	{
		s_MinKeySize = Size;
		s_SecParams.min_key_size = Size;
	}
}

void BtSecSdCheckStatus(void)
{
	if (!s_bInit)
	{
		return;
	}

	for (uint16_t h = 0; h < NRF_SDH_BLE_TOTAL_LINK_COUNT; h++)
	{
		BtSecSdLink_t *pLink = &s_Links[h];
		if (BtPeerFindByHdl(h) == nullptr)
		{
			continue;
		}
		if (pLink->bReplyPending)
		{
			const ble_gap_sec_params_t *p = pLink->bReplyReject ?
				nullptr : &pLink->ReplyParams;
			(void)ParamsReplyAttempt(h, p);
		}
		if (pLink->bSecurePending)
		{
			(void)BtSecSdSecure(h, pLink->bForceRepair);
		}
	}

	if (s_bBondStore)
	{
		BtSmpBondNvmCheckStatus();
	}
}

void BtSecSdPoll(void)
{
	if (s_bBondStore)
	{
		BtSmpBondNvmPoll();
	}
}

bool BtSecSdInit(const ble_gap_sec_params_t *pParams)
{
	if (s_bInit)
	{
		return true;
	}
	if (pParams == nullptr)
	{
		return false;
	}

	memset(s_Links, 0, sizeof(s_Links));
	s_SecParams = *pParams;
	s_MinKeySize = pParams->min_key_size >= BT_SMP_MIN_ENC_KEY_SIZE ?
		pParams->min_key_size : BT_SMP_MIN_ENC_KEY_SIZE;
	s_bScOnly = pParams->lesc != 0;

	if (!BtLescInit())
	{
		DEBUG_PRINTF("SEC: BtLescInit failed\r\n");
		return false;
	}

	// One portable bond format/store for SoftDevice and SDC. The nRF52 Nvm
	// interface submits flash operations through sd_flash_* while a SoftDevice
	// owns the controller, so no FDS/Peer Manager arbitration is needed.
	int r = BtSmpBondNvmInit();
	if (r != 0)
	{
		DEBUG_PRINTF("SEC: bond store init failed %d\r\n", r);
		return false;
	}
	s_bBondStore = true;
	s_bInit = true;
	return true;
}
