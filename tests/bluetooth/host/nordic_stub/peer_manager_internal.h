#ifndef __BT_TEST_PEER_MANAGER_INTERNAL_H__
#define __BT_TEST_PEER_MANAGER_INTERNAL_H__

#include "peer_manager_types.h"
#include "sdk_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Event sink in peer_manager.c.
void pm_sm_evt_handler(pm_evt_t *p_sm_evt);

#ifdef __cplusplus
}
#endif

#endif
