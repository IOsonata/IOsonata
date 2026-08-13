#ifndef __BT_TEST_ID_MANAGER_H__
#define __BT_TEST_ID_MANAGER_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_gap.h"
#include "peer_manager_types.h"
#include "sdk_common.h"

#ifdef __cplusplus
extern "C" {
#endif

pm_peer_id_t im_peer_id_get_by_conn_handle(uint16_t ConnHdl);
pm_peer_id_t im_peer_id_get_by_master_id(ble_gap_master_id_t *pMasterId);
void im_new_peer_id(uint16_t ConnHdl, pm_peer_id_t PeerId);
ret_code_t im_peer_free(pm_peer_id_t PeerId);
ret_code_t im_ble_addr_get(uint16_t ConnHdl, ble_gap_addr_t *pAddr);
bool im_master_id_is_valid(ble_gap_master_id_t const *pMasterId);
bool im_master_ids_compare(ble_gap_master_id_t const *pA, ble_gap_master_id_t const *pB);
pm_peer_id_t im_find_duplicate_bonding_data(pm_peer_data_bonding_t const *pBondingData,
											pm_peer_id_t Skip);

#ifdef __cplusplus
}
#endif

#endif
