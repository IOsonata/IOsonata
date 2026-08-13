#ifndef __BT_TEST_PEER_DATABASE_H__
#define __BT_TEST_PEER_DATABASE_H__

#include <stdint.h>

#include "peer_manager_types.h"
#include "sdk_common.h"

// Write buffer key for a connection that has no peer id yet. The SDK derives
// it from the connection handle at the top of the peer id range.
#define PDB_TEMP_PEER_ID(conn_handle)	\
	((pm_peer_id_t)(PM_PEER_ID_INVALID - 1U - (conn_handle)))

#ifdef __cplusplus
extern "C" {
#endif

ret_code_t pdb_write_buf_get(pm_peer_id_t PeerId, uint8_t DataId, uint32_t Words,
							 pm_peer_data_t *pPeerData);
ret_code_t pdb_write_buf_release(pm_peer_id_t PeerId, uint8_t DataId);
ret_code_t pdb_write_buf_store(pm_peer_id_t PeerId, uint8_t DataId, pm_peer_id_t NewPeerId);
ret_code_t pdb_peer_data_ptr_get(pm_peer_id_t PeerId, uint8_t DataId,
								 pm_peer_data_flash_t *pPeerData);

#ifdef __cplusplus
}
#endif

#endif
