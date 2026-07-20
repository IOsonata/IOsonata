/**-------------------------------------------------------------------------
@file	bt_id_bm.cpp

@brief	IOsonata identity manager for the sdk-nrf-bm SoftDevice path.

		Replaces the sdk-nrf-bm Peer Manager id_manager.c while keeping the
		existing im_* API. Peer Manager, GATT cache and security callers remain
		unchanged. Bond records remain in peer_database and peer_data_storage
		during this migration stage.

		The module owns connection-to-peer mapping, address resolution, duplicate
		bond detection, master-ID lookup, allow-list state, device identities and
		privacy passthroughs. S145 connection handles are mapped through
		nrf_sdh_ble_idx_get and are not assumed to be dense.

		Resolvable private addresses are never tested with an all-zero IRK. The
		cached allow list changes only after the SoftDevice accepts the new list.

@author	Hoang Nguyen Hoan
@date	Jul. 14, 2026

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
#include <string.h>
#include <stdio.h>

#include <nrf_error.h>
#include <nrf_soc.h>
#include <ble.h>
#include <ble_gap.h>
#include <ble_err.h>

#include <bm/softdevice_handler/nrf_sdh_ble.h>
#include <bm/bluetooth/peer_manager/peer_manager_types.h>

#include <modules/conn_state.h>
#include <modules/id_manager.h>
#include <modules/peer_database.h>
#include <modules/peer_data_storage.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(peer_manager, CONFIG_PEER_MANAGER_LOG_LEVEL);

#define BT_ID_ADDR_HASH_LEN		3U
#define BT_ID_ADDR_PRAND_LEN	3U

extern "C" void pm_im_evt_handler(struct pm_evt *event);
extern "C" void gcm_im_evt_handler(struct pm_evt *event);

typedef struct {
	uint16_t       ConnHdl = BLE_CONN_HANDLE_INVALID;
	uint16_t       PeerId = PM_PEER_ID_INVALID;
	ble_gap_addr_t PeerAddr = {};
} BtIdConn_t;

static BtIdConn_t s_Conns[CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT];
static uint8_t    s_AllowPeerCnt;
static uint16_t   s_AllowPeers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];

static void EvtSend(struct pm_evt *pEvt)
{
	pm_im_evt_handler(pEvt);
	gcm_im_evt_handler(pEvt);
}

static bool IrkIsValid(const ble_gap_irk_t *pIrk)
{
	if (pIrk == nullptr)
	{
		return false;
	}

	for (uint32_t i = 0; i < BLE_GAP_SEC_KEY_LEN; i++)
	{
		if (pIrk->irk[i] != 0U)
		{
			return true;
		}
	}
	return false;
}

static bool AddrCompare(const ble_gap_addr_t *pAddr1, const ble_gap_addr_t *pAddr2)
{
	if (pAddr1 == nullptr || pAddr2 == nullptr ||
		pAddr1->addr_type != pAddr2->addr_type)
	{
		return false;
	}

	return memcmp(pAddr1->addr, pAddr2->addr, BLE_GAP_ADDR_LEN) == 0;
}

// Bluetooth ah() over the SoftDevice ECB interface. The ECB structure is byte
// reversed relative to the Core specification strings.
static bool Ah(const uint8_t *pKey, const uint8_t *pRand, uint8_t *pHash)
{
	nrf_ecb_hal_data_t ecb;

	for (uint32_t i = 0; i < SOC_ECB_KEY_LENGTH; i++)
	{
		ecb.key[i] = pKey[SOC_ECB_KEY_LENGTH - 1U - i];
	}

	memset(ecb.cleartext, 0, SOC_ECB_KEY_LENGTH - BT_ID_ADDR_PRAND_LEN);
	for (uint32_t i = 0; i < BT_ID_ADDR_PRAND_LEN; i++)
	{
		ecb.cleartext[SOC_ECB_KEY_LENGTH - 1U - i] = pRand[i];
	}

	if (sd_ecb_block_encrypt(&ecb) != NRF_SUCCESS)
	{
		return false;
	}

	for (uint32_t i = 0; i < BT_ID_ADDR_HASH_LEN; i++)
	{
		pHash[i] = ecb.ciphertext[SOC_ECB_KEY_LENGTH - 1U - i];
	}
	return true;
}

static void BondIterPrepare(struct pm_peer_data_const *pPeerData,
							uint8_t *pBuffer, uint16_t *pIter)
{
	memset(pPeerData, 0, sizeof(*pPeerData));
	memset(pBuffer, 0, PM_PEER_DATA_MAX_SIZE);
	pPeerData->all_data = pBuffer;
	pds_peer_data_iterate_prepare(pIter);
}

static uint32_t PeersIdKeysGet(const uint16_t *pPeers, uint32_t PeerCnt,
							   ble_gap_addr_t *pAddrs, uint32_t *pAddrCnt,
							   ble_gap_irk_t *pIrks, uint32_t *pIrkCnt)
{
	const bool copyAddrs = pAddrs != nullptr && pAddrCnt != nullptr;
	const bool copyIrks = pIrks != nullptr && pIrkCnt != nullptr;

	if (!copyAddrs && !copyIrks)
	{
		return NRF_ERROR_NULL;
	}
	if ((copyAddrs && *pAddrCnt < PeerCnt) ||
		(copyIrks && *pIrkCnt < PeerCnt))
	{
		return NRF_ERROR_NO_MEM;
	}

	if (copyAddrs)
	{
		*pAddrCnt = 0;
	}
	if (copyIrks)
	{
		*pIrkCnt = 0;
	}
	if (PeerCnt == 0U)
	{
		return NRF_SUCCESS;
	}
	if (pPeers == nullptr)
	{
		return NRF_ERROR_NULL;
	}

	struct pm_peer_data_bonding bond;
	struct pm_peer_data peerData;
	const uint32_t bufSize = sizeof(bond);

	memset(&peerData, 0, sizeof(peerData));
	peerData.bonding_data = &bond;

	for (uint32_t i = 0; i < PeerCnt; i++)
	{
		memset(&bond, 0, sizeof(bond));
		uint32_t status = pds_peer_data_read(pPeers[i], PM_PEER_DATA_ID_BONDING,
										 &peerData, &bufSize);
		if (status != NRF_SUCCESS)
		{
			return status == NRF_ERROR_INVALID_PARAM ? NRF_ERROR_NOT_FOUND : status;
		}

		const uint8_t addrType = bond.peer_ble_id.id_addr_info.addr_type;
		if (addrType != BLE_GAP_ADDR_TYPE_PUBLIC &&
			addrType != BLE_GAP_ADDR_TYPE_RANDOM_STATIC)
		{
			return BLE_ERROR_GAP_INVALID_BLE_ADDR;
		}

		if (copyAddrs)
		{
			pAddrs[i] = bond.peer_ble_id.id_addr_info;
			(*pAddrCnt)++;
		}
		if (copyIrks)
		{
			pIrks[i] = bond.peer_ble_id.id_info;
			(*pIrkCnt)++;
		}
	}

	return NRF_SUCCESS;
}

extern "C" {

bool im_address_resolve(const ble_gap_addr_t *addr, const ble_gap_irk_t *irk)
{
	uint8_t localHash[BT_ID_ADDR_HASH_LEN];

	if (addr == nullptr || !IrkIsValid(irk) ||
		addr->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE)
	{
		return false;
	}

	return Ah(irk->irk, &addr->addr[BT_ID_ADDR_HASH_LEN], localHash) &&
		   memcmp(addr->addr, localHash, BT_ID_ADDR_HASH_LEN) == 0;
}

void im_ble_evt_handler(const ble_evt_t *bleEvt)
{
	if (bleEvt == nullptr || bleEvt->header.evt_id != BLE_GAP_EVT_CONNECTED)
	{
		return;
	}

	const ble_gap_evt_t *pGapEvt = &bleEvt->evt.gap_evt;
	const ble_gap_addr_t *pAddr = &pGapEvt->params.connected.peer_addr;
	const int idx = nrf_sdh_ble_idx_get(pGapEvt->conn_handle);

	if (idx < 0 || idx >= CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT)
	{
		return;
	}

	uint16_t matchId = PM_PEER_ID_INVALID;

	if (pAddr->addr_id_peer)
	{
		// The SoftDevice resolved this peer against the device identity list
		// and delivered its stored identity address. Match that identity to
		// its peer id directly so a bonded reconnect finds the existing bond.
		uint16_t peerId;
		uint16_t iter;
		struct pm_peer_data_const peerData;
		uint8_t buffer[PM_PEER_DATA_MAX_SIZE];

		BondIterPrepare(&peerData, buffer, &iter);

		while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId,
									 &peerData, &iter))
		{
			if (AddrCompare(pAddr,
							&peerData.bonding_data->peer_ble_id.id_addr_info))
			{
				matchId = peerId;
				break;
			}
		}
	}
	else if (pAddr->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE)
	{
		uint16_t peerId;
		uint16_t iter;
		struct pm_peer_data_const peerData;
		uint8_t buffer[PM_PEER_DATA_MAX_SIZE];

		BondIterPrepare(&peerData, buffer, &iter);

		switch (pAddr->addr_type)
		{
		case BLE_GAP_ADDR_TYPE_PUBLIC:
		case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:
			while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId,
										 &peerData, &iter))
			{
				if (AddrCompare(pAddr,
								&peerData.bonding_data->peer_ble_id.id_addr_info))
				{
					matchId = peerId;
					break;
				}
			}
			break;

		case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
			while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId,
										 &peerData, &iter))
			{
				const ble_gap_irk_t *pIrk = &peerData.bonding_data->peer_ble_id.id_info;
				if (IrkIsValid(pIrk) && im_address_resolve(pAddr, pIrk))
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

#ifdef BT_PDS_TRACE
	{
		extern void BtPdsTraceOut(const char *pStr);
		char b[80];
		snprintf(b, sizeof(b),
				 "im connect: hdl=%u addr_type=%u resolved peerId=%u\r\n",
				 pGapEvt->conn_handle, pAddr->addr_type, matchId);
		BtPdsTraceOut(b);
	}
#endif
	s_Conns[idx].ConnHdl = pGapEvt->conn_handle;
	s_Conns[idx].PeerId = matchId;
	s_Conns[idx].PeerAddr = *pAddr;

	if (matchId != PM_PEER_ID_INVALID)
	{
		struct pm_evt event;
		memset(&event, 0, sizeof(event));
		event.evt_id = PM_EVT_BONDED_PEER_CONNECTED;
		event.conn_handle = pGapEvt->conn_handle;
		event.peer_id = matchId;
		EvtSend(&event);
	}
}

bool im_is_duplicate_bonding_data(const struct pm_peer_data_bonding *bondingData1,
								  const struct pm_peer_data_bonding *bondingData2)
{
	if (bondingData1 == nullptr || bondingData2 == nullptr)
	{
		return false;
	}

	const ble_gap_addr_t *pAddr1 = &bondingData1->peer_ble_id.id_addr_info;
	const ble_gap_addr_t *pAddr2 = &bondingData2->peer_ble_id.id_addr_info;
	const bool duplicateIrk =
		memcmp(bondingData1->peer_ble_id.id_info.irk,
			   bondingData2->peer_ble_id.id_info.irk,
			   BLE_GAP_SEC_KEY_LEN) == 0 &&
		IrkIsValid(&bondingData1->peer_ble_id.id_info) &&
		IrkIsValid(&bondingData2->peer_ble_id.id_info);
	const bool duplicateAddr = AddrCompare(pAddr1, pAddr2);
	const bool identityAddrs =
		pAddr1->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE &&
		pAddr1->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE &&
		pAddr2->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE &&
		pAddr2->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE;

	return (duplicateAddr && identityAddrs) ||
		   (duplicateIrk && !identityAddrs);
}

uint16_t im_find_duplicate_bonding_data(const struct pm_peer_data_bonding *bondingData,
										uint16_t peerIdSkip)
{
	if (bondingData == nullptr)
	{
		return PM_PEER_ID_INVALID;
	}

	uint16_t peerId;
	uint16_t iter;
	struct pm_peer_data_const peerData;
	uint8_t buffer[PM_PEER_DATA_MAX_SIZE];
	BondIterPrepare(&peerData, buffer, &iter);

	while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId,
								 &peerData, &iter))
	{
		if (peerId != peerIdSkip &&
			im_is_duplicate_bonding_data(bondingData, peerData.bonding_data))
		{
			return peerId;
		}
	}
	return PM_PEER_ID_INVALID;
}

uint16_t im_peer_id_get_by_conn_handle(uint16_t connHandle)
{
	const int idx = nrf_sdh_ble_idx_get(connHandle);

	if (idx < 0 || idx >= CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT ||
		!pm_conn_state_valid(connHandle) || s_Conns[idx].ConnHdl != connHandle)
	{
		return PM_PEER_ID_INVALID;
	}
	return s_Conns[idx].PeerId;
}

uint16_t im_peer_id_get_by_master_id(const ble_gap_master_id_t *masterId)
{
	if (masterId == nullptr)
	{
		return PM_PEER_ID_INVALID;
	}

	uint16_t peerId;
	uint16_t iter;
	struct pm_peer_data_const peerData;
	uint8_t buffer[PM_PEER_DATA_MAX_SIZE];
	BondIterPrepare(&peerData, buffer, &iter);

	while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peerId,
								 &peerData, &iter))
	{
		if (im_master_ids_compare(masterId,
								  &peerData.bonding_data->own_ltk.master_id) ||
			im_master_ids_compare(masterId,
								  &peerData.bonding_data->peer_ltk.master_id))
		{
			return peerId;
		}
	}
	return PM_PEER_ID_INVALID;
}

uint16_t im_conn_handle_get(uint16_t peerId)
{
	if (peerId == PM_PEER_ID_INVALID)
	{
		return BLE_CONN_HANDLE_INVALID;
	}

	for (uint32_t i = 0; i < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
	{
		if (s_Conns[i].PeerId == peerId &&
			pm_conn_state_valid(s_Conns[i].ConnHdl))
		{
			return s_Conns[i].ConnHdl;
		}
	}
	return BLE_CONN_HANDLE_INVALID;
}

bool im_master_id_is_valid(const ble_gap_master_id_t *masterId)
{
	if (masterId == nullptr)
	{
		return false;
	}
	if (masterId->ediv != 0U)
	{
		return true;
	}
	for (uint32_t i = 0; i < BLE_GAP_SEC_RAND_LEN; i++)
	{
		if (masterId->rand[i] != 0U)
		{
			return true;
		}
	}
	return false;
}

bool im_master_ids_compare(const ble_gap_master_id_t *masterId1,
						   const ble_gap_master_id_t *masterId2)
{
	if (!im_master_id_is_valid(masterId1) || masterId2 == nullptr ||
		masterId1->ediv != masterId2->ediv)
	{
		return false;
	}
	return memcmp(masterId1->rand, masterId2->rand,
				  BLE_GAP_SEC_RAND_LEN) == 0;
}

void im_new_peer_id(uint16_t connHandle, uint16_t peerId)
{
	const int idx = nrf_sdh_ble_idx_get(connHandle);

	if (idx >= 0 && idx < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT)
	{
		s_Conns[idx].ConnHdl = connHandle;
		s_Conns[idx].PeerId = peerId;
	}
}

uint32_t im_peer_free(uint16_t peerId)
{
	const uint16_t connHandle = im_conn_handle_get(peerId);
	const uint32_t status = pdb_peer_free(peerId);

	if (status == NRF_SUCCESS && connHandle != BLE_CONN_HANDLE_INVALID)
	{
		const int idx = nrf_sdh_ble_idx_get(connHandle);
		if (idx >= 0 && idx < CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT)
		{
			s_Conns[idx].PeerId = PM_PEER_ID_INVALID;
		}
	}
	return status;
}

uint32_t im_ble_addr_get(uint16_t connHandle, ble_gap_addr_t *bleAddr)
{
	if (bleAddr == nullptr)
	{
		return NRF_ERROR_NULL;
	}

	const int idx = nrf_sdh_ble_idx_get(connHandle);
	if (idx < 0 || idx >= CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT ||
		!pm_conn_state_valid(connHandle) || s_Conns[idx].ConnHdl != connHandle)
	{
		return BLE_ERROR_INVALID_CONN_HANDLE;
	}

	*bleAddr = s_Conns[idx].PeerAddr;
	return NRF_SUCCESS;
}

uint32_t im_allow_list_set(const uint16_t *peers, const uint32_t peerCnt)
{
	if (peers == nullptr || peerCnt == 0U)
	{
		const uint32_t status = sd_ble_gap_whitelist_set(nullptr, 0);
		if (status == NRF_SUCCESS)
		{
			memset(s_AllowPeers, 0, sizeof(s_AllowPeers));
			s_AllowPeerCnt = 0;
		}
		return status;
	}
	if (peerCnt > BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
	{
		return NRF_ERROR_DATA_SIZE;
	}

	uint32_t addrCnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
	ble_gap_addr_t addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
	const ble_gap_addr_t *addrPtrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
	memset(addrs, 0, sizeof(addrs));

	uint32_t status = PeersIdKeysGet(peers, peerCnt, addrs, &addrCnt,
										 nullptr, nullptr);
	if (status != NRF_SUCCESS)
	{
		return status;
	}

	for (uint32_t i = 0; i < peerCnt; i++)
	{
		addrPtrs[i] = &addrs[i];
	}

	status = sd_ble_gap_whitelist_set(addrPtrs, peerCnt);
	if (status == NRF_SUCCESS)
	{
		memset(s_AllowPeers, 0, sizeof(s_AllowPeers));
		s_AllowPeerCnt = (uint8_t)peerCnt;
		memcpy(s_AllowPeers, peers, peerCnt * sizeof(peers[0]));
	}
	return status;
}

uint32_t im_allow_list_get(ble_gap_addr_t *addrs, uint32_t *addrCnt,
						   ble_gap_irk_t *irks, uint32_t *irkCnt)
{
	if ((addrs == nullptr && irks == nullptr) ||
		(addrCnt == nullptr && irkCnt == nullptr))
	{
		return NRF_ERROR_NULL;
	}
	if ((addrCnt != nullptr && s_AllowPeerCnt > *addrCnt) ||
		(irkCnt != nullptr && s_AllowPeerCnt > *irkCnt))
	{
		return NRF_ERROR_NO_MEM;
	}

	return PeersIdKeysGet(s_AllowPeers, s_AllowPeerCnt,
						  addrs, addrCnt, irks, irkCnt);
}

uint32_t im_device_identities_list_set(const uint16_t *peers, uint32_t peerCnt)
{
	if (peerCnt > BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT)
	{
		return NRF_ERROR_INVALID_PARAM;
	}
	if (peers == nullptr || peerCnt == 0U)
	{
		return sd_ble_gap_device_identities_set(nullptr, nullptr, 0);
	}

	struct pm_peer_data_bonding bond;
	struct pm_peer_data peerData;
	const uint32_t bufSize = sizeof(bond);
	ble_gap_id_key_t keys[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
	const ble_gap_id_key_t *keyPtrs[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];

	memset(&peerData, 0, sizeof(peerData));
	peerData.bonding_data = &bond;
	memset(keys, 0, sizeof(keys));

	for (uint32_t i = 0; i < peerCnt; i++)
	{
		memset(&bond, 0, sizeof(bond));
		uint32_t status = pds_peer_data_read(peers[i], PM_PEER_DATA_ID_BONDING,
										 &peerData, &bufSize);
		if (status != NRF_SUCCESS)
		{
			return status == NRF_ERROR_INVALID_PARAM ? NRF_ERROR_NOT_FOUND : status;
		}

		const uint8_t addrType = bond.peer_ble_id.id_addr_info.addr_type;
		if (addrType != BLE_GAP_ADDR_TYPE_PUBLIC &&
			addrType != BLE_GAP_ADDR_TYPE_RANDOM_STATIC)
		{
			return BLE_ERROR_GAP_INVALID_BLE_ADDR;
		}

		keys[i] = bond.peer_ble_id;
		keyPtrs[i] = &keys[i];
	}

	return sd_ble_gap_device_identities_set(keyPtrs, nullptr, peerCnt);
}

uint32_t im_id_addr_set(const ble_gap_addr_t *addr)
{
	return sd_ble_gap_addr_set(addr);
}

uint32_t im_id_addr_get(ble_gap_addr_t *addr)
{
	if (addr == nullptr)
	{
		return NRF_ERROR_NULL;
	}
	return sd_ble_gap_addr_get(addr);
}

uint32_t im_privacy_set(const ble_gap_privacy_params_t *privacyParams)
{
	return sd_ble_gap_privacy_set(privacyParams);
}

uint32_t im_privacy_get(ble_gap_privacy_params_t *privacyParams)
{
	return sd_ble_gap_privacy_get(privacyParams);
}

} // extern "C"
