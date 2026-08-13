/**-------------------------------------------------------------------------
@file	bt_sec_sd_test_stubs.cpp

@brief	Implementation of the host stand-ins declared in
		bt_sec_sd_test_stubs.h.

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

#include "ble_hci.h"
#include "bt_lesc.h"
#include "id_manager.h"
#include "nrf_error.h"
#include "nrf_sdh_ble.h"
#include "peer_data_storage.h"
#include "peer_database.h"
#include "peer_manager_internal.h"

SecSdDisconnectCapture_t g_SecSdDisconnect;
SecSdParamsReplyCapture_t g_SecSdParamsReply;
SecSdEvtCapture_t g_SecSdEvt;

int g_SecSdAuthenticateCount;
int g_SecSdEncryptCount;
int g_SecSdWriteBufReleaseCount;
int g_SecSdWriteBufStoreCount;

namespace {

struct StubLink {
	bool bValid;
	uint8_t Role;
	bool bEncrypted;
	bool bMitm;
	bool bLesc;
	uint32_t Flags;
	pm_peer_id_t PeerId;
};

StubLink s_Links[NRF_SDH_BLE_TOTAL_LINK_COUNT];
uint16_t s_FlagNext;

uint32_t s_DisconnectResult = NRF_SUCCESS;
bool s_bDisconnectResultArmed;
uint32_t s_ParamsReplyResult = NRF_SUCCESS;
pm_peer_id_t s_DuplicateBond = PM_PEER_ID_INVALID;
pm_peer_id_t s_AllocPeerId = 1;
SecSdEvtHook_t s_EvtHook;

// One write buffer, which is all a two link harness needs: the tests never
// hold two open at once.
pm_peer_data_bonding_t s_WriteBuf;
bool s_bWriteBufTaken;
pm_peer_id_t s_WriteBufOwner = PM_PEER_ID_INVALID;

// One stored bond record, returned by pdb_peer_data_ptr_get.
pm_peer_data_bonding_t s_StoredBond;

StubLink *LinkGet(uint16_t ConnHdl)
{
	return (ConnHdl < NRF_SDH_BLE_TOTAL_LINK_COUNT) ? &s_Links[ConnHdl] : nullptr;
}

} // namespace

void SecSdDisconnectResultSet(uint32_t Result)
{
	s_DisconnectResult = Result;
	s_bDisconnectResultArmed = true;
}

void SecSdParamsReplyResultSet(uint32_t Result)
{
	s_ParamsReplyResult = Result;
}

void SecSdDuplicateBondSet(pm_peer_id_t PeerId)
{
	s_DuplicateBond = PeerId;
}

void SecSdPeerIdSet(uint16_t ConnHdl, pm_peer_id_t PeerId)
{
	StubLink *pLink = LinkGet(ConnHdl);
	if (pLink != nullptr)
	{
		pLink->PeerId = PeerId;
	}
}

void SecSdLinkSet(uint16_t ConnHdl, bool bValid, uint8_t Role, bool bEncrypted)
{
	StubLink *pLink = LinkGet(ConnHdl);
	if (pLink != nullptr)
	{
		pLink->bValid = bValid;
		pLink->Role = Role;
		pLink->bEncrypted = bEncrypted;
	}
}

void SecSdEvtHookSet(SecSdEvtHook_t Hook)
{
	s_EvtHook = Hook;
}

void SecSdStubsReset(void)
{
	std::memset(&g_SecSdDisconnect, 0, sizeof(g_SecSdDisconnect));
	std::memset(&g_SecSdParamsReply, 0, sizeof(g_SecSdParamsReply));
	std::memset(&g_SecSdEvt, 0, sizeof(g_SecSdEvt));
	std::memset(s_Links, 0, sizeof(s_Links));
	std::memset(&s_WriteBuf, 0, sizeof(s_WriteBuf));
	std::memset(&s_StoredBond, 0, sizeof(s_StoredBond));

	for (size_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		s_Links[i].PeerId = PM_PEER_ID_INVALID;
	}

	g_SecSdAuthenticateCount = 0;
	g_SecSdEncryptCount = 0;
	g_SecSdWriteBufReleaseCount = 0;
	g_SecSdWriteBufStoreCount = 0;

	s_DisconnectResult = NRF_SUCCESS;
	s_bDisconnectResultArmed = false;
	s_ParamsReplyResult = NRF_SUCCESS;
	s_DuplicateBond = PM_PEER_ID_INVALID;
	s_AllocPeerId = 1;
	s_EvtHook = nullptr;
	s_bWriteBufTaken = false;
	s_WriteBufOwner = PM_PEER_ID_INVALID;
}

int SecSdEvtCount(pm_evt_id_t EvtId)
{
	int n = 0;

	for (int i = 0; i < g_SecSdEvt.Count && i < SEC_SD_CAPTURE_MAX; i++)
	{
		if (g_SecSdEvt.Evt[i].evt_id == EvtId)
		{
			n++;
		}
	}
	return n;
}

const pm_evt_t *SecSdEvtFind(pm_evt_id_t EvtId)
{
	for (int i = 0; i < g_SecSdEvt.Count && i < SEC_SD_CAPTURE_MAX; i++)
	{
		if (g_SecSdEvt.Evt[i].evt_id == EvtId)
		{
			return &g_SecSdEvt.Evt[i];
		}
	}
	return nullptr;
}

// ---- ble_conn_state ---------------------------------------------------------

extern "C" ble_conn_state_user_flag_id_t ble_conn_state_user_flag_acquire(void)
{
	if (s_FlagNext >= BLE_CONN_STATE_USER_FLAG_COUNT)
	{
		return BLE_CONN_STATE_USER_FLAG_INVALID;
	}
	return (ble_conn_state_user_flag_id_t)s_FlagNext++;
}

extern "C" bool ble_conn_state_user_flag_get(uint16_t ConnHdl,
											 ble_conn_state_user_flag_id_t FlagId)
{
	StubLink *pLink = LinkGet(ConnHdl);

	if (pLink == nullptr || FlagId >= BLE_CONN_STATE_USER_FLAG_COUNT)
	{
		return false;
	}
	return (pLink->Flags & (1U << FlagId)) != 0;
}

extern "C" void ble_conn_state_user_flag_set(uint16_t ConnHdl,
											 ble_conn_state_user_flag_id_t FlagId,
											 bool Value)
{
	StubLink *pLink = LinkGet(ConnHdl);

	if (pLink == nullptr || FlagId >= BLE_CONN_STATE_USER_FLAG_COUNT)
	{
		return;
	}
	if (Value)
	{
		pLink->Flags |= (1U << FlagId);
	}
	else
	{
		pLink->Flags &= ~(1U << FlagId);
	}
}

extern "C" uint32_t ble_conn_state_for_each_set_user_flag(
		ble_conn_state_user_flag_id_t FlagId, ble_conn_state_user_function_t Func, void *pCtx)
{
	uint32_t n = 0;

	for (uint16_t h = 0; h < NRF_SDH_BLE_TOTAL_LINK_COUNT; h++)
	{
		if (ble_conn_state_user_flag_get(h, FlagId))
		{
			n++;
			if (Func != nullptr)
			{
				Func(h, pCtx);
			}
		}
	}
	return n;
}

extern "C" bool ble_conn_state_valid(uint16_t ConnHdl)
{
	StubLink *pLink = LinkGet(ConnHdl);
	return pLink != nullptr && pLink->bValid;
}

extern "C" uint8_t ble_conn_state_role(uint16_t ConnHdl)
{
	StubLink *pLink = LinkGet(ConnHdl);
	return (pLink != nullptr && pLink->bValid) ? pLink->Role : BLE_GAP_ROLE_INVALID;
}

extern "C" bool ble_conn_state_encrypted(uint16_t ConnHdl)
{
	StubLink *pLink = LinkGet(ConnHdl);
	return pLink != nullptr && pLink->bEncrypted;
}

extern "C" bool ble_conn_state_mitm_protected(uint16_t ConnHdl)
{
	StubLink *pLink = LinkGet(ConnHdl);
	return pLink != nullptr && pLink->bMitm;
}

extern "C" bool ble_conn_state_lesc(uint16_t ConnHdl)
{
	StubLink *pLink = LinkGet(ConnHdl);
	return pLink != nullptr && pLink->bLesc;
}

// ---- id_manager -------------------------------------------------------------

extern "C" pm_peer_id_t im_peer_id_get_by_conn_handle(uint16_t ConnHdl)
{
	StubLink *pLink = LinkGet(ConnHdl);
	return (pLink != nullptr) ? pLink->PeerId : PM_PEER_ID_INVALID;
}

extern "C" pm_peer_id_t im_peer_id_get_by_master_id(ble_gap_master_id_t *pMasterId)
{
	(void)pMasterId;
	return PM_PEER_ID_INVALID;
}

extern "C" void im_new_peer_id(uint16_t ConnHdl, pm_peer_id_t PeerId)
{
	SecSdPeerIdSet(ConnHdl, PeerId);
}

extern "C" ret_code_t im_peer_free(pm_peer_id_t PeerId)
{
	(void)PeerId;
	return NRF_SUCCESS;
}

extern "C" ret_code_t im_ble_addr_get(uint16_t ConnHdl, ble_gap_addr_t *pAddr)
{
	(void)ConnHdl;
	if (pAddr != nullptr)
	{
		std::memset(pAddr, 0, sizeof(*pAddr));
	}
	return NRF_SUCCESS;
}

extern "C" bool im_master_id_is_valid(ble_gap_master_id_t const *pMasterId)
{
	if (pMasterId == nullptr)
	{
		return false;
	}
	if (pMasterId->ediv != 0)
	{
		return true;
	}
	for (size_t i = 0; i < sizeof(pMasterId->rand); i++)
	{
		if (pMasterId->rand[i] != 0)
		{
			return true;
		}
	}
	return false;
}

extern "C" bool im_master_ids_compare(ble_gap_master_id_t const *pA,
									  ble_gap_master_id_t const *pB)
{
	if (pA == nullptr || pB == nullptr)
	{
		return false;
	}
	return std::memcmp(pA, pB, sizeof(*pA)) == 0;
}

extern "C" pm_peer_id_t im_find_duplicate_bonding_data(
		pm_peer_data_bonding_t const *pBondingData, pm_peer_id_t Skip)
{
	(void)pBondingData;
	(void)Skip;
	return s_DuplicateBond;
}

// ---- peer_database and peer_data_storage ------------------------------------

extern "C" ret_code_t pdb_write_buf_get(pm_peer_id_t PeerId, uint8_t DataId, uint32_t Words,
										pm_peer_data_t *pPeerData)
{
	(void)DataId;
	(void)Words;

	if (pPeerData == nullptr)
	{
		return NRF_ERROR_NULL;
	}
	if (s_bWriteBufTaken && s_WriteBufOwner != PeerId)
	{
		return NRF_ERROR_BUSY;
	}

	s_bWriteBufTaken = true;
	s_WriteBufOwner = PeerId;

	pPeerData->data_id = DataId;
	pPeerData->length_words = Words;
	pPeerData->p_bonding_data = &s_WriteBuf;
	return NRF_SUCCESS;
}

extern "C" ret_code_t pdb_write_buf_release(pm_peer_id_t PeerId, uint8_t DataId)
{
	(void)DataId;

	g_SecSdWriteBufReleaseCount++;
	if (!s_bWriteBufTaken || s_WriteBufOwner != PeerId)
	{
		return NRF_ERROR_NOT_FOUND;
	}
	s_bWriteBufTaken = false;
	s_WriteBufOwner = PM_PEER_ID_INVALID;
	return NRF_SUCCESS;
}

extern "C" ret_code_t pdb_write_buf_store(pm_peer_id_t PeerId, uint8_t DataId,
										  pm_peer_id_t NewPeerId)
{
	(void)DataId;
	(void)NewPeerId;

	g_SecSdWriteBufStoreCount++;
	if (!s_bWriteBufTaken || s_WriteBufOwner != PeerId)
	{
		return NRF_ERROR_NOT_FOUND;
	}
	s_StoredBond = s_WriteBuf;
	s_bWriteBufTaken = false;
	s_WriteBufOwner = PM_PEER_ID_INVALID;
	return NRF_SUCCESS;
}

extern "C" ret_code_t pdb_peer_data_ptr_get(pm_peer_id_t PeerId, uint8_t DataId,
											pm_peer_data_flash_t *pPeerData)
{
	(void)PeerId;

	if (pPeerData == nullptr)
	{
		return NRF_ERROR_NULL;
	}
	pPeerData->data_id = DataId;
	pPeerData->length_words = 0;
	pPeerData->p_bonding_data = &s_StoredBond;
	return NRF_SUCCESS;
}

extern "C" pm_peer_id_t pds_peer_id_allocate(void)
{
	return s_AllocPeerId++;
}

// ---- SoftDevice GAP ---------------------------------------------------------

extern "C" uint32_t sd_ble_gap_disconnect(uint16_t ConnHdl, uint8_t HciStatusCode)
{
	if (g_SecSdDisconnect.Count < SEC_SD_CAPTURE_MAX)
	{
		g_SecSdDisconnect.ConnHdl[g_SecSdDisconnect.Count] = ConnHdl;
		g_SecSdDisconnect.Reason[g_SecSdDisconnect.Count] = HciStatusCode;
	}
	g_SecSdDisconnect.Count++;

	// The SoftDevice accepts two reasons only, and answers anything else with
	// NRF_ERROR_INVALID_PARAM while leaving the link up. See ble_gap.h,
	// sd_ble_gap_disconnect.
	if (HciStatusCode != BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION &&
		HciStatusCode != BLE_HCI_CONN_INTERVAL_UNACCEPTABLE)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

	if (s_bDisconnectResultArmed)
	{
		s_bDisconnectResultArmed = false;
		return s_DisconnectResult;
	}
	return NRF_SUCCESS;
}

extern "C" uint32_t sd_ble_gap_authenticate(uint16_t ConnHdl,
											const ble_gap_sec_params_t *pSecParams)
{
	(void)ConnHdl;
	(void)pSecParams;
	g_SecSdAuthenticateCount++;
	return NRF_SUCCESS;
}

extern "C" uint32_t sd_ble_gap_encrypt(uint16_t ConnHdl,
									   const ble_gap_master_id_t *pMasterId,
									   const ble_gap_enc_info_t *pEncInfo)
{
	(void)ConnHdl;
	(void)pMasterId;
	(void)pEncInfo;
	g_SecSdEncryptCount++;
	return NRF_SUCCESS;
}

extern "C" uint32_t sd_ble_gap_sec_params_reply(uint16_t ConnHdl, uint8_t SecStatus,
												const ble_gap_sec_params_t *pSecParams,
												const ble_gap_sec_keyset_t *pKeyset)
{
	if (g_SecSdParamsReply.Count < SEC_SD_CAPTURE_MAX)
	{
		int i = g_SecSdParamsReply.Count;
		g_SecSdParamsReply.ConnHdl[i] = ConnHdl;
		g_SecSdParamsReply.SecStatus[i] = SecStatus;
		g_SecSdParamsReply.bHasParams[i] = pSecParams != nullptr;
		g_SecSdParamsReply.bHasKeyset[i] = pKeyset != nullptr;
	}
	g_SecSdParamsReply.Count++;
	return s_ParamsReplyResult;
}

extern "C" uint32_t sd_ble_gap_sec_info_reply(uint16_t ConnHdl,
											  const ble_gap_enc_info_t *pEncInfo,
											  const ble_gap_irk_t *pIdInfo,
											  const ble_gap_sign_info_t *pSignInfo)
{
	(void)ConnHdl;
	(void)pEncInfo;
	(void)pIdInfo;
	(void)pSignInfo;
	return NRF_SUCCESS;
}

extern "C" uint32_t sd_ble_gap_lesc_oob_data_get(uint16_t ConnHdl,
												 ble_gap_lesc_p256_pk_t *pPk,
												 ble_gap_lesc_oob_data_t *pData)
{
	(void)ConnHdl;
	(void)pPk;
	(void)pData;
	return NRF_SUCCESS;
}

extern "C" uint32_t sd_ble_gap_lesc_oob_data_set(uint16_t ConnHdl,
												 ble_gap_lesc_oob_data_t *pOwn,
												 ble_gap_lesc_oob_data_t *pPeer)
{
	(void)ConnHdl;
	(void)pOwn;
	(void)pPeer;
	return NRF_SUCCESS;
}

// ---- peer_manager event sink ------------------------------------------------

extern "C" void pm_sm_evt_handler(pm_evt_t *p_sm_evt)
{
	if (p_sm_evt == nullptr)
	{
		return;
	}
	if (g_SecSdEvt.Count < SEC_SD_CAPTURE_MAX)
	{
		g_SecSdEvt.Evt[g_SecSdEvt.Count] = *p_sm_evt;
	}
	g_SecSdEvt.Count++;

	if (s_EvtHook != nullptr)
	{
		s_EvtHook(p_sm_evt);
	}
}

// ---- bt_lesc ----------------------------------------------------------------
// The LESC module is exercised by bt_lesc_sd_test; here it only has to exist.

namespace {
ble_gap_lesc_p256_pk_t s_LocalPk;
}

bool BtLescInit(void)
{
	return true;
}

void BtLescOnBleEvt(const ble_evt_t *pBleEvt)
{
	(void)pBleEvt;
}

ble_gap_lesc_p256_pk_t *BtLescPubKeyGet(void)
{
	return &s_LocalPk;
}
