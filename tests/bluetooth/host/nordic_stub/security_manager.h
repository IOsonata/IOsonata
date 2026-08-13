#ifndef __BT_TEST_SECURITY_MANAGER_H__
#define __BT_TEST_SECURITY_MANAGER_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"
#include "ble_gap.h"
#include "peer_manager_types.h"
#include "sdk_common.h"

// The sm_* surface peer_manager.c calls. C linkage, as in the SDK header.
#ifdef __cplusplus
extern "C" {
#endif

ret_code_t sm_init(void);
void sm_ble_evt_handler(ble_evt_t const *p_ble_evt);
ret_code_t sm_sec_params_set(ble_gap_sec_params_t *p_sec_params);
ret_code_t sm_sec_params_reply(uint16_t conn_handle, ble_gap_sec_params_t *p_sec_params,
							   void const *p_context);
void sm_conn_sec_config_reply(uint16_t conn_handle, pm_conn_sec_config_t *p_conn_sec_config);
ret_code_t sm_lesc_public_key_set(ble_gap_lesc_p256_pk_t *p_public_key);
ret_code_t sm_conn_sec_status_get(uint16_t conn_handle, pm_conn_sec_status_t *p_conn_sec_status);
bool sm_sec_is_sufficient(uint16_t conn_handle, pm_conn_sec_status_t *p_sec_status_req);
ret_code_t sm_link_secure(uint16_t conn_handle, bool force_repairing);

#ifdef __cplusplus
}
#endif

#endif
