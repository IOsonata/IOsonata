/**-------------------------------------------------------------------------
@file	bt_sec_bm.cpp

@brief	IOsonata security manager for the sdk-nrf-bm S145 SoftDevice path.

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
#include <bm/softdevice_handler/nrf_sdh_ble.h>

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
	bool                    bReplySc;
	bool                    bSecurePending;
	bool                    bForceRepair;
	bool                    bPairing;
	bool                    bBonding;
	bool                    bSc;
	bool                    bAuthenticated;
	bool                    bSecuredNotified;
	uint8_t                 KeySize;
} BtSecBmLink_t;

static BtSecBmLink_t s_Links[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static ble_gap_sec_params_t s_SecParams;
static bool s_bInit;
static bool s_bScOnly;
static bool s_bBondStore;
static bool s_bIdentitySyncPending;
static uint8_t s_MinKeySize = BT_SMP_MIN_ENC_KEY_SIZE;
static bool s_bAuthCfgSet;
static uint8_t s_AuthIoCaps;
static uint8_t s_AuthReq;
static bool s_bOobCfgSet;
static bool s_bOobCfg;
static bool s_bScOnlyCfgSet;
static bool s_bMinKeyCfgSet;
static uint32_t s_LastBondPoll;

static inline BtSecBmLink_t *LinkGet(uint16_t ConnHdl)
{
	const int idx = nrf_sdh_ble_idx_get(ConnHdl);
	return idx >= 0 && idx < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT ?
		&s_Links[idx] : nullptr;
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

static uint32_t SoftDeviceIdentityListSync(void)
{
	ble_gap_id_key_t keys[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
	const ble_gap_id_key_t *ptrs[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
	memset(keys, 0, sizeof(keys));
	memset(ptrs, 0, sizeof(ptrs));

	uint8_t count = 0;
	for (int slot = 0; slot < BtSmpBondSlotCount() &&
		count < BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT; slot++)
	{
		uint8_t addrType;
		uint8_t addr[6];
		uint8_t irk[16];
		if (!BtSmpBondIdentityGet(slot, &addrType, addr, irk))
		{
			continue;
		}

		keys[count].id_addr_info.addr_type =
			addrType == BTADDR_TYPE_PUBLIC ?
			BLE_GAP_ADDR_TYPE_PUBLIC : BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
		memcpy(keys[count].id_addr_info.addr, addr, sizeof(addr));
		memcpy(keys[count].id_info.irk, irk, sizeof(irk));
		ptrs[count] = &keys[count];
		count++;
		CryptoSecureWipe(irk, sizeof(irk));
	}

	uint32_t r = count == 0 ?
		sd_ble_gap_device_identities_set(nullptr, nullptr, 0) :
		sd_ble_gap_device_identities_set(ptrs, nullptr, count);
	CryptoSecureWipe(keys, sizeof(keys));
	return r;
}

static uint32_t SoftDeviceLocalIdentitySync(void)
{
	ble_gap_privacy_params_t privacy;
	ble_gap_irk_t deviceIrk;
	uint8_t savedIrk[16];
	memset(&privacy, 0, sizeof(privacy));
	memset(&deviceIrk, 0, sizeof(deviceIrk));
	memset(savedIrk, 0, sizeof(savedIrk));
	privacy.p_device_irk = &deviceIrk;

	uint32_t r = sd_ble_gap_privacy_get(&privacy);
	if (r != NRF_SUCCESS)
	{
		return r;
	}

	if (BtSmpLocalIrkGet(savedIrk))
	{
		if (memcmp(deviceIrk.irk, savedIrk, sizeof(savedIrk)) != 0)
		{
			memcpy(deviceIrk.irk, savedIrk, sizeof(savedIrk));
			privacy.p_device_irk = &deviceIrk;
			r = sd_ble_gap_privacy_set(&privacy);
		}
	}
	else if (KeyPresent(deviceIrk.irk, sizeof(deviceIrk.irk)))
	{
		(void)BtSmpLocalIrkSet(deviceIrk.irk);
	}
	else
	{
		r = NRF_ERROR_INVALID_DATA;
	}

	CryptoSecureWipe(savedIrk, sizeof(savedIrk));
	CryptoSecureWipe(&deviceIrk, sizeof(deviceIrk));
	return r;
}

static uint32_t SoftDeviceIdentitySync(void)
{
	uint32_t r = SoftDeviceLocalIdentitySync();
	if (r != NRF_SUCCESS)
	{
		return r;
	}
	return SoftDeviceIdentityListSync();
}

static bool IsCentral(uint16_t ConnHdl)
{
	return BtPeerRole(ConnHdl) == BT_CONN_ROLE_CENTRAL;
}

static bool IsPeripheral(uint16_t ConnHdl)
{
	return BtPeerRole(ConnHdl) == BT_CONN_ROLE_PERIPHERAL;
}

static void LinkKeyBuffersReset(BtSecBmLink_t *pLink)
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
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
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

static void LinkSecFromBond(BtSecBmLink_t *pLink, const BtSmpKeys_t *pKeys)
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
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
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

static void KeysetFill(uint16_t ConnHdl, bool bSc, ble_gap_sec_keyset_t *pKeyset)
{
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
	memset(pKeyset, 0, sizeof(*pKeyset));
	if (pLink == nullptr)
	{
		return;
	}

	if (bSc)
	{
		pKeyset->keys_own.p_pk = BtLescPubKeyGet();
		pKeyset->keys_peer.p_pk = &pLink->PeerPk;
	}

	if (!s_SecParams.bond)
	{
		return;
	}

	pKeyset->keys_own.p_enc_key = &pLink->OwnEnc;
	// In LE Secure Connections the peer encryption-key pointer must be NULL;
	// the LTK is derived, not distributed. Legacy pairing needs this buffer
	// because the peer may distribute a distinct LTK for later role reversal.
	pKeyset->keys_peer.p_enc_key = bSc ? nullptr : &pLink->PeerEnc;
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

static uint32_t ParamsReplyAttempt(uint16_t ConnHdl, const ble_gap_sec_params_t *pParams,
								 bool bSc)
{
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
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
		pLink->bReplySc = false;
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
		KeysetFill(ConnHdl, bSc, &keyset);
		if (bSc && keyset.keys_own.p_pk == nullptr)
		{
			pLink->bReplyPending = true;
			pLink->bReplyReject = false;
			pLink->bReplySc = bSc;
			pLink->ReplyParams = *pParams;
			return NRF_ERROR_BUSY;
		}
	}

	// Only a peripheral returns local security parameters here. A central
	// already supplied them to sd_ble_gap_authenticate().
	ble_gap_sec_params_t *pSdParams = IsPeripheral(ConnHdl) ? pReply : nullptr;
	uint32_t r = sd_ble_gap_sec_params_reply(ConnHdl, secStatus, pSdParams,
		secStatus == BLE_GAP_SEC_STATUS_SUCCESS ? &keyset : nullptr);

	if (r == NRF_ERROR_BUSY)
	{
		pLink->bReplyPending = true;
		pLink->bReplyReject = pParams == nullptr;
		pLink->bReplySc = bSc;
		if (pParams != nullptr)
		{
			pLink->ReplyParams = *pParams;
		}
	}
	else
	{
		pLink->bReplyPending = false;
		pLink->bReplyReject = false;
		pLink->bReplySc = false;
	}
	return r;
}

static void ParamsRequestProcess(const ble_gap_evt_t *pGapEvt)
{
	uint16_t h = pGapEvt->conn_handle;
	BtSecBmLink_t *pLink = LinkGet(h);
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
	bool bSc = s_SecParams.lesc && pPeer->lesc;
	uint32_t r = ParamsReplyAttempt(h, &s_SecParams, bSc);
	if (r != NRF_SUCCESS && r != NRF_ERROR_BUSY)
	{
		DEBUG_PRINTF("SEC: params reply failed 0x%X\r\n", (unsigned)r);
	}
}

static void KeysFromLink(uint16_t ConnHdl, const ble_gap_evt_auth_status_t *pAuth,
	BtSmpKeys_t *pKeys)
{
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
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

	bool peerLtk = KeyPresent(pKeys->Ltk, sizeof(pKeys->Ltk));
	bool localLtk = KeyPresent(pKeys->LocalLtk, sizeof(pKeys->LocalLtk));
	pKeys->bValid = pKeys->EncKeySize >= BT_SMP_MIN_ENC_KEY_SIZE &&
		pKeys->EncKeySize <= BT_SMP_MAX_ENC_KEY_SIZE &&
		(sc ? peerLtk : (peerLtk || localLtk));
}

static void AuthStatusProcess(const ble_gap_evt_t *pGapEvt)
{
	uint16_t h = pGapEvt->conn_handle;
	BtSecBmLink_t *pLink = LinkGet(h);
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
			s_bIdentitySyncPending = true;
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

	uint32_t r = sd_ble_gap_sec_info_reply(h, pEnc, nullptr, nullptr);
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

static uint32_t EncryptFromBond(uint16_t ConnHdl, const BtSmpKeys_t *pKeys)
{
	// Central encryption uses the LTK distributed by the peer (or the common
	// derived LTK for SC). A legacy bond may legitimately contain only the
	// locally distributed LTK; that record is still useful when this device is
	// the peripheral, but it cannot start encryption as the central.
	if (pKeys == nullptr || !KeyPresent(pKeys->Ltk, sizeof(pKeys->Ltk)))
	{
		return NRF_ERROR_NOT_FOUND;
	}

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

	uint32_t r = sd_ble_gap_encrypt(ConnHdl, &id, &enc);
	CryptoSecureWipe(&enc, sizeof(enc));
	CryptoSecureWipe(&id, sizeof(id));
	return r;
}

uint32_t BtSecBmSecure(uint16_t ConnHdl, bool ForceRepair)
{
	BtSecBmLink_t *pLink = LinkGet(ConnHdl);
	if (!s_bInit || pLink == nullptr || BtPeerFindByHdl(ConnHdl) == nullptr)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	if (!BtSmpPairingAttemptAllowed(ConnHdl, BtSmpMsTick()))
	{
		return NRF_ERROR_INVALID_STATE;
	}

	uint32_t r = NRF_ERROR_NOT_FOUND;
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
	(void)BtSecBmSecure(pGapEvt->conn_handle, true);
}

void BtSecBmBleEvt(const ble_evt_t *pEvt)
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
			if (s_bIdentitySyncPending)
			{
				uint32_t r = SoftDeviceIdentitySync();
				s_bIdentitySyncPending = r != NRF_SUCCESS;
			}
			break;

		default:
			break;
	}

	// Single delivery point for the SoftDevice LESC helper. It owns the P-256
	// private-key lifetime and regenerates after every completed or aborted
	// pairing procedure.
	BtLescOnBleEvt(pEvt);
}

static void BtSecBmAuthConfigApply(void)
{
	if (!s_bAuthCfgSet)
	{
		return;
	}
	s_SecParams.io_caps = s_AuthIoCaps;
	s_SecParams.bond = (s_AuthReq & BT_SMP_AUTHREQ_BONDING_FLAG_MASK) !=
		BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING;
	s_SecParams.mitm = (s_AuthReq & BT_SMP_AUTHREQ_MITM) != 0;
	s_SecParams.keypress = (s_AuthReq & BT_SMP_AUTHREQ_KEYPRESS) != 0;

	// The portable IOsonata SMP API selects Secure Connections. A caller that
	// needs legacy fallback stays on the port's initial SecType configuration.
	s_SecParams.lesc = 1;
	s_bScOnly = true;
	s_bScOnlyCfgSet = true;
}

void BtSecBmAuthConfig(uint8_t IoCaps, uint8_t AuthReq)
{
	if (IoCaps > BT_SMP_IOCAPS_KEYBOARD_DISPLAY)
	{
		return;
	}
	s_AuthIoCaps = IoCaps;
	s_AuthReq = AuthReq;
	s_bAuthCfgSet = true;
	BtSecBmAuthConfigApply();
}

void BtSecBmOobSet(bool Enable)
{
	s_bOobCfg = Enable;
	s_bOobCfgSet = true;
	s_SecParams.oob = Enable ? 1 : 0;
}

void BtSecBmScOnlySet(bool Enable)
{
	s_bScOnly = Enable;
	s_bScOnlyCfgSet = true;
	if (Enable)
	{
		s_SecParams.lesc = 1;
	}
}

void BtSecBmMinKeySizeSet(uint8_t Size)
{
	if (Size >= BT_SMP_MIN_ENC_KEY_SIZE && Size <= BT_SMP_MAX_ENC_KEY_SIZE)
	{
		s_MinKeySize = Size;
		s_bMinKeyCfgSet = true;
		s_SecParams.min_key_size = Size;
	}
}

void BtSecBmCheckStatus(void)
{
	if (!s_bInit)
	{
		return;
	}
	if (s_bIdentitySyncPending && !BtPeerIsConnected())
	{
		uint32_t r = SoftDeviceIdentitySync();
		s_bIdentitySyncPending = r != NRF_SUCCESS;
	}

	for (uint16_t i = 0; i < BtPeerCount(); i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer == nullptr || pPeer->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}
		uint16_t h = pPeer->Conn.Hdl;
		BtSecBmLink_t *pLink = LinkGet(h);
		if (pLink == nullptr)
		{
			continue;
		}
		if (pLink->bReplyPending)
		{
			const ble_gap_sec_params_t *p = pLink->bReplyReject ?
				nullptr : &pLink->ReplyParams;
			(void)ParamsReplyAttempt(h, p, pLink->bReplySc);
		}
		if (pLink->bSecurePending)
		{
			(void)BtSecBmSecure(h, pLink->bForceRepair);
		}
	}

	if (s_bBondStore)
	{
		BtSmpBondNvmCheckStatus();

		// nRF54's GRTC is free-running under the SoftDevice; this port does
		// not own a periodic compare interrupt. Advance storage retry backoff
		// from the status hook only when a real second elapsed.
		uint32_t now = BtSmpMsTick();
		if ((uint32_t)(now - s_LastBondPoll) >= 1000U)
		{
			s_LastBondPoll = now;
			BtSmpBondNvmPoll();
		}
	}
}

void BtSecBmPoll(void)
{
	if (s_bBondStore)
	{
		s_LastBondPoll = BtSmpMsTick();
		BtSmpBondNvmPoll();
	}
}

bool BtSecBmInit(const ble_gap_sec_params_t *pParams)
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
	if (!s_bMinKeyCfgSet)
	{
		s_MinKeySize = pParams->min_key_size >= BT_SMP_MIN_ENC_KEY_SIZE ?
			pParams->min_key_size : BT_SMP_MIN_ENC_KEY_SIZE;
	}
	s_SecParams.min_key_size = s_MinKeySize;
	if (!s_bScOnlyCfgSet)
	{
		s_bScOnly = pParams->lesc != 0;
	}
	BtSecBmAuthConfigApply();
	if (s_bOobCfgSet)
	{
		s_SecParams.oob = s_bOobCfg ? 1 : 0;
	}

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
	uint32_t identityStatus = SoftDeviceIdentitySync();
	if (identityStatus != NRF_SUCCESS)
	{
		DEBUG_PRINTF("SEC: identity sync failed 0x%X\r\n",
			(unsigned)identityStatus);
		s_bIdentitySyncPending = true;
	}
	else
	{
		s_bIdentitySyncPending = false;
	}

	s_bBondStore = true;
	s_LastBondPoll = BtSmpMsTick();
	s_bInit = true;
	return true;
}
