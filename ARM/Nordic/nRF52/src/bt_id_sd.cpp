/**-------------------------------------------------------------------------
@file	bt_id_sd.cpp

@brief	IOsonata identity manager for the nRF5 SDK SoftDevice path.

		Replaces the nRF5 SDK peer_manager module id_manager.c with the same
		API surface (id_manager.h): the connection to peer id mapping, peer
		address resolution against stored IRKs, bond identity duplicate
		detection, and the whitelist, device identity and privacy passthroughs
		to the SoftDevice. peer_manager.c, gcm/gscm and bt_sec_sd keep calling
		the im_* names unchanged; peer data reads go through the kept SDK
		layer (peer_database, peer_data_storage, FDS), so nothing about the
		stored bond format changes.

		Address resolution uses the ah() hash (Core spec Vol 3 Part H 2.2.2)
		over sd_ecb_block_encrypt, byte for byte the SDK construction: the
		SoftDevice ECB structure is byte reversed relative to the spec
		strings, so the IRK and prand are reversed in and the hash is the
		reversed tail of the ciphertext.

		One event is emitted, PM_EVT_BONDED_PEER_CONNECTED, through the two
		extern sinks the SDK module used (pm_im_evt_handler in peer_manager.c
		and gcm_im_evt_handler in gatt_cache_manager.c). Those references also
		extract this object from the archive.

		The connection map entries are validated lazily with
		ble_conn_state_valid at read time, the same lifecycle as the SDK
		module; entries are never proactively cleared on disconnect.

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
#include "ble_conn_state.h"
#include "nrf_soc.h"
#include "nrf_sdh_ble.h"			// NRF_SDH_BLE_TOTAL_LINK_COUNT

#include "id_manager.h"				// the im_* API this module implements
#include "peer_manager_types.h"
#include "peer_database.h"
#include "peer_data_storage.h"

// Resolvable private address layout: hash = addr[0..2], prand = addr[3..5].
#define BT_ID_ADDR_HASH_LEN			3
#define BT_ID_ADDR_PRAND_LEN		3

// Event sinks, the same extern hookup the SDK module used.
extern "C" void pm_im_evt_handler(pm_evt_t * p_event);
extern "C" void gcm_im_evt_handler(pm_evt_t * p_event);

// Connection to peer mapping, indexed by connection handle. Entries are
// validated with ble_conn_state_valid when read.
typedef struct {
	// PM_PEER_ID_INVALID is not zero, so the static array below relies on this
	// member initializer, not on zero initialization. A recycled or not yet
	// resolved handle must never map to peer id 0, which can be a valid peer.
	pm_peer_id_t   PeerId = PM_PEER_ID_INVALID;
	ble_gap_addr_t PeerAddr = {};
} BtIdConn_t;

static BtIdConn_t   s_Conns[NRF_SDH_BLE_TOTAL_LINK_COUNT];

// Peer ids of the current whitelist, remembered so im_whitelist_get can
// return the addresses and IRKs.
static uint8_t      s_WlistPeerCnt;
static pm_peer_id_t s_WlistPeers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];

static void EvtSend(pm_evt_t *pEvt)
{
	pm_im_evt_handler(pEvt);
	gcm_im_evt_handler(pEvt);
}

static bool IrkIsValid(const ble_gap_irk_t *pIrk)
{
	for (uint32_t i = 0; i < BLE_GAP_SEC_KEY_LEN; i++)
	{
		if (pIrk->irk[i] != 0)
		{
			return true;
		}
	}
	return false;
}

static bool AddrCompare(const ble_gap_addr_t *pAddr1, const ble_gap_addr_t *pAddr2)
{
	if (pAddr1 == nullptr || pAddr2 == nullptr)
	{
		return false;
	}
	if (pAddr1->addr_type != pAddr2->addr_type)
	{
		return false;
	}
	return memcmp(pAddr1->addr, pAddr2->addr, BLE_GAP_ADDR_LEN) == 0;
}

// ah() (Core spec Vol 3 Part H 2.2.2) over the SoftDevice ECB. The ECB
// structure is byte reversed relative to the spec strings: reverse the IRK
// into the key, the prand into the low cleartext bytes, and take the hash
// from the reversed tail of the ciphertext. Byte for byte the SDK
// construction; a mistake here fails silently as no resolution.
static void Ah(const uint8_t *pK, const uint8_t *pR, uint8_t *pHash)
{
	nrf_ecb_hal_data_t ecb;

	for (uint32_t i = 0; i < SOC_ECB_KEY_LENGTH; i++)
	{
		ecb.key[i] = pK[SOC_ECB_KEY_LENGTH - 1 - i];
	}

	memset(ecb.cleartext, 0, SOC_ECB_KEY_LENGTH - BT_ID_ADDR_PRAND_LEN);
	for (uint32_t i = 0; i < BT_ID_ADDR_PRAND_LEN; i++)
	{
		ecb.cleartext[SOC_ECB_KEY_LENGTH - 1 - i] = pR[i];
	}

	(void)sd_ecb_block_encrypt(&ecb);	// can only return NRF_SUCCESS

	for (uint32_t i = 0; i < BT_ID_ADDR_HASH_LEN; i++)
	{
		pHash[i] = ecb.ciphertext[SOC_ECB_KEY_LENGTH - 1 - i];
	}
}

// ---- im_* API surface -------------------------------------------------------

extern "C" {

bool im_address_resolve(ble_gap_addr_t const * p_addr, ble_gap_irk_t const * p_irk)
{
	uint8_t localHash[BT_ID_ADDR_HASH_LEN];

	if (p_addr->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE)
	{
		return false;
	}

	Ah(p_irk->irk, &p_addr->addr[BT_ID_ADDR_HASH_LEN], localHash);

	return memcmp(p_addr->addr, localHash, BT_ID_ADDR_HASH_LEN) == 0;
}

void im_ble_evt_handler(ble_evt_t const * ble_evt)
{
	if (ble_evt->header.evt_id != BLE_GAP_EVT_CONNECTED)
	{
		return;
	}

	const ble_gap_evt_t *pGapEvt = &ble_evt->evt.gap_evt;
	const ble_gap_addr_t *pAddr = &pGapEvt->params.connected.peer_addr;
	pm_peer_id_t matchId = PM_PEER_ID_INVALID;

	if (pAddr->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE)
	{
		pm_peer_id_t         peerId;
		pm_peer_data_flash_t peerData;

		pds_peer_data_iterate_prepare();

		switch (pAddr->addr_type)
		{
		case BLE_GAP_ADDR_TYPE_PUBLIC:
		case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:
			while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId, &peerData))
			{
				if (AddrCompare(pAddr,
								&peerData.p_bonding_data->peer_ble_id.id_addr_info))
				{
					matchId = peerId;
					break;
				}
			}
			break;

		case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
			while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId, &peerData))
			{
				// A bond whose peer distributed no IRK stores an all-zero IRK;
				// ah() over a zero key can false match, so it never resolves.
				if (IrkIsValid(&peerData.p_bonding_data->peer_ble_id.id_info) &&
					im_address_resolve(pAddr,
									   &peerData.p_bonding_data->peer_ble_id.id_info))
				{
					matchId = peerId;
					break;
				}
			}
			break;

		default:
			break;
		}
	}

	if (pGapEvt->conn_handle < NRF_SDH_BLE_TOTAL_LINK_COUNT)
	{
		s_Conns[pGapEvt->conn_handle].PeerId   = matchId;
		s_Conns[pGapEvt->conn_handle].PeerAddr = *pAddr;
	}

	if (matchId != PM_PEER_ID_INVALID)
	{
		pm_evt_t evt;

		memset(&evt, 0, sizeof(evt));
		evt.evt_id      = PM_EVT_BONDED_PEER_CONNECTED;
		evt.conn_handle = pGapEvt->conn_handle;
		evt.peer_id     = matchId;
		EvtSend(&evt);
	}
}

bool im_is_duplicate_bonding_data(pm_peer_data_bonding_t const * p_bonding_data1,
								  pm_peer_data_bonding_t const * p_bonding_data2)
{
	const ble_gap_addr_t *pAddr1 = &p_bonding_data1->peer_ble_id.id_addr_info;
	const ble_gap_addr_t *pAddr2 = &p_bonding_data2->peer_ble_id.id_addr_info;

	bool dupIrk = (memcmp(p_bonding_data1->peer_ble_id.id_info.irk,
						  p_bonding_data2->peer_ble_id.id_info.irk,
						  BLE_GAP_SEC_KEY_LEN) == 0) &&
				  IrkIsValid(&p_bonding_data1->peer_ble_id.id_info) &&
				  IrkIsValid(&p_bonding_data2->peer_ble_id.id_info);
	bool dupAddr = AddrCompare(pAddr1, pAddr2);
	bool idAddrs = (pAddr1->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE) &&
				   (pAddr1->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE) &&
				   (pAddr2->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE) &&
				   (pAddr2->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE);

	return (dupAddr && idAddrs) || (dupIrk && !idAddrs);
}

pm_peer_id_t im_find_duplicate_bonding_data(pm_peer_data_bonding_t const * p_bonding_data,
											pm_peer_id_t                   peer_id_skip)
{
	pm_peer_id_t         peerId;
	pm_peer_data_flash_t peerData;

	pds_peer_data_iterate_prepare();
	while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId, &peerData))
	{
		if ((peerId != peer_id_skip) &&
			im_is_duplicate_bonding_data(p_bonding_data, peerData.p_bonding_data))
		{
			return peerId;
		}
	}
	return PM_PEER_ID_INVALID;
}

pm_peer_id_t im_peer_id_get_by_conn_handle(uint16_t conn_handle)
{
	if (conn_handle >= NRF_SDH_BLE_TOTAL_LINK_COUNT || !ble_conn_state_valid(conn_handle))
	{
		return PM_PEER_ID_INVALID;
	}
	return s_Conns[conn_handle].PeerId;
}

pm_peer_id_t im_peer_id_get_by_master_id(ble_gap_master_id_t const * p_master_id)
{
	pm_peer_id_t         peerId;
	pm_peer_data_flash_t peerData;

	pds_peer_data_iterate_prepare();
	while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId, &peerData))
	{
		if (im_master_ids_compare(p_master_id,
								  &peerData.p_bonding_data->own_ltk.master_id) ||
			im_master_ids_compare(p_master_id,
								  &peerData.p_bonding_data->peer_ltk.master_id))
		{
			return peerId;
		}
	}
	return PM_PEER_ID_INVALID;
}

uint16_t im_conn_handle_get(pm_peer_id_t peer_id)
{
	if (peer_id == PM_PEER_ID_INVALID)
	{
		return BLE_CONN_HANDLE_INVALID;
	}
	for (uint16_t h = 0; h < NRF_SDH_BLE_TOTAL_LINK_COUNT; h++)
	{
		if (s_Conns[h].PeerId == peer_id && ble_conn_state_valid(h))
		{
			return h;
		}
	}
	return BLE_CONN_HANDLE_INVALID;
}

bool im_master_id_is_valid(ble_gap_master_id_t const * p_master_id)
{
	if (p_master_id->ediv != 0)
	{
		return true;
	}
	for (uint32_t i = 0; i < BLE_GAP_SEC_RAND_LEN; i++)
	{
		if (p_master_id->rand[i] != 0)
		{
			return true;
		}
	}
	return false;
}

bool im_master_ids_compare(ble_gap_master_id_t const * p_master_id1,
						   ble_gap_master_id_t const * p_master_id2)
{
	if (!im_master_id_is_valid(p_master_id1))
	{
		return false;
	}
	if (p_master_id1->ediv != p_master_id2->ediv)
	{
		return false;
	}
	return memcmp(p_master_id1->rand, p_master_id2->rand, BLE_GAP_SEC_RAND_LEN) == 0;
}

void im_new_peer_id(uint16_t conn_handle, pm_peer_id_t peer_id)
{
	if (conn_handle < NRF_SDH_BLE_TOTAL_LINK_COUNT)
	{
		s_Conns[conn_handle].PeerId = peer_id;
	}
}

ret_code_t im_peer_free(pm_peer_id_t peer_id)
{
	uint16_t   connHdl = im_conn_handle_get(peer_id);
	ret_code_t r = pdb_peer_free(peer_id);

	if (r == NRF_SUCCESS && connHdl < NRF_SDH_BLE_TOTAL_LINK_COUNT)
	{
		s_Conns[connHdl].PeerId = PM_PEER_ID_INVALID;
	}
	return r;
}

ret_code_t im_ble_addr_get(uint16_t conn_handle, ble_gap_addr_t * p_ble_addr)
{
	if (p_ble_addr == NULL)
	{
		return NRF_ERROR_NULL;
	}
	if (conn_handle >= NRF_SDH_BLE_TOTAL_LINK_COUNT || !ble_conn_state_valid(conn_handle))
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

	*p_ble_addr = s_Conns[conn_handle].PeerAddr;
	return NRF_SUCCESS;
}

}	// extern "C"

// Collect identity addresses and IRKs for a list of peers from their bond
// records. Only identity addresses (public or random static) are usable for
// the whitelist and the device identity list.
static ret_code_t PeersIdKeysGet(const pm_peer_id_t *pPeers, uint32_t PeerCnt,
								 ble_gap_addr_t *pAddrs, uint32_t *pAddrCnt,
								 ble_gap_irk_t *pIrks, uint32_t *pIrkCnt)
{
	pm_peer_data_bonding_t bond;
	pm_peer_data_t         peerData;
	const uint32_t         bufSize = sizeof(bond);
	bool bCopyAddrs = (pAddrs != NULL) && (pAddrCnt != NULL);
	bool bCopyIrks  = (pIrks != NULL) && (pIrkCnt != NULL);

	memset(&peerData, 0, sizeof(peerData));
	peerData.p_bonding_data = &bond;

	for (uint32_t i = 0; i < PeerCnt; i++)
	{
		memset(&bond, 0, sizeof(bond));
		ret_code_t r = pds_peer_data_read(pPeers[i], PM_PEER_DATA_ID_BONDING,
										  &peerData, &bufSize);
		if (r == NRF_ERROR_NOT_FOUND || r == NRF_ERROR_INVALID_PARAM)
		{
			return NRF_ERROR_NOT_FOUND;
		}

		uint8_t addrType = bond.peer_ble_id.id_addr_info.addr_type;
		if (addrType != BLE_GAP_ADDR_TYPE_PUBLIC &&
			addrType != BLE_GAP_ADDR_TYPE_RANDOM_STATIC)
		{
			return BLE_ERROR_GAP_INVALID_BLE_ADDR;
		}

		if (bCopyAddrs)
		{
			memcpy(&pAddrs[i], &bond.peer_ble_id.id_addr_info, sizeof(ble_gap_addr_t));
			(*pAddrCnt)++;
		}
		if (bCopyIrks)
		{
			memcpy(&pIrks[i], bond.peer_ble_id.id_info.irk, BLE_GAP_SEC_KEY_LEN);
			(*pIrkCnt)++;
		}
	}
	return NRF_SUCCESS;
}

extern "C" {

ret_code_t im_whitelist_set(pm_peer_id_t const * p_peers, uint32_t peer_cnt)
{
	// The remembered peer list commits only after the SoftDevice accepted the
	// new whitelist, so im_whitelist_get never reports a list that failed to
	// install (validation error or BLE_ERROR_GAP_WHITELIST_IN_USE).
	if (p_peers == NULL || peer_cnt == 0)
	{
		ret_code_t r = sd_ble_gap_whitelist_set(NULL, 0);
		if (r == NRF_SUCCESS)
		{
			memset(s_WlistPeers, 0, sizeof(s_WlistPeers));
			s_WlistPeerCnt = 0;
		}
		return r;
	}
	if (peer_cnt > BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

	uint32_t              addrCnt = 0;
	ble_gap_addr_t        addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
	const ble_gap_addr_t *addrPtrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];

	memset(addrs, 0, sizeof(addrs));
	ret_code_t r = PeersIdKeysGet(p_peers, peer_cnt, addrs, &addrCnt, NULL, NULL);
	if (r != NRF_SUCCESS)
	{
		return r;
	}

	for (uint32_t i = 0; i < peer_cnt; i++)
	{
		addrPtrs[i] = &addrs[i];
	}
	r = sd_ble_gap_whitelist_set(addrPtrs, peer_cnt);
	if (r == NRF_SUCCESS)
	{
		memset(s_WlistPeers, 0, sizeof(s_WlistPeers));
		s_WlistPeerCnt = (uint8_t)peer_cnt;
		memcpy(s_WlistPeers, p_peers, sizeof(pm_peer_id_t) * peer_cnt);
	}
	return r;
}

ret_code_t im_whitelist_get(ble_gap_addr_t * p_addrs, uint32_t * p_addr_cnt,
							ble_gap_irk_t * p_irks, uint32_t * p_irk_cnt)
{
	if (((p_addr_cnt != NULL) && (s_WlistPeerCnt > *p_addr_cnt)) ||
		((p_irk_cnt  != NULL) && (s_WlistPeerCnt > *p_irk_cnt)))
	{
		return NRF_ERROR_NO_MEM;
	}
	if (p_addr_cnt != NULL)
	{
		*p_addr_cnt = 0;
	}
	if (p_irk_cnt != NULL)
	{
		*p_irk_cnt = 0;
	}
	return PeersIdKeysGet(s_WlistPeers, s_WlistPeerCnt,
						  p_addrs, p_addr_cnt, p_irks, p_irk_cnt);
}

ret_code_t im_device_identities_list_set(pm_peer_id_t const * p_peers,
										 uint32_t             peer_cnt)
{
	pm_peer_data_bonding_t  bond;
	pm_peer_data_t          peerData;
	const uint32_t          bufSize = sizeof(bond);
	ble_gap_id_key_t        keys[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
	const ble_gap_id_key_t *keyPtrs[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];

	if (peer_cnt > BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT)
	{
		return NRF_ERROR_INVALID_PARAM;
	}
	if (p_peers == NULL || peer_cnt == 0)
	{
		return sd_ble_gap_device_identities_set(NULL, NULL, 0);
	}

	memset(&peerData, 0, sizeof(peerData));
	peerData.p_bonding_data = &bond;

	for (uint32_t i = 0; i < peer_cnt; i++)
	{
		memset(&bond, 0, sizeof(bond));
		ret_code_t r = pds_peer_data_read(p_peers[i], PM_PEER_DATA_ID_BONDING,
										  &peerData, &bufSize);
		if (r == NRF_ERROR_NOT_FOUND || r == NRF_ERROR_INVALID_PARAM)
		{
			return NRF_ERROR_NOT_FOUND;
		}

		uint8_t addrType = bond.peer_ble_id.id_addr_info.addr_type;
		if (addrType != BLE_GAP_ADDR_TYPE_PUBLIC &&
			addrType != BLE_GAP_ADDR_TYPE_RANDOM_STATIC)
		{
			return BLE_ERROR_GAP_INVALID_BLE_ADDR;
		}

		memcpy(&keys[i], &bond.peer_ble_id, sizeof(ble_gap_id_key_t));
		keyPtrs[i] = &keys[i];
	}
	return sd_ble_gap_device_identities_set(keyPtrs, NULL, peer_cnt);
}

ret_code_t im_id_addr_set(ble_gap_addr_t const * p_addr)
{
	return sd_ble_gap_addr_set(p_addr);
}

ret_code_t im_id_addr_get(ble_gap_addr_t * p_addr)
{
	if (p_addr == NULL)
	{
		return NRF_ERROR_NULL;
	}
	return sd_ble_gap_addr_get(p_addr);
}

ret_code_t im_privacy_set(pm_privacy_params_t const * p_privacy_params)
{
	return sd_ble_gap_privacy_set(p_privacy_params);
}

ret_code_t im_privacy_get(pm_privacy_params_t * p_privacy_params)
{
	return sd_ble_gap_privacy_get(p_privacy_params);
}

}	// extern "C"

#endif // NRF_MODULE_ENABLED(PEER_MANAGER)
