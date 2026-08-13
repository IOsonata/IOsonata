#ifndef __BT_TEST_BLE_CONN_STATE_H__
#define __BT_TEST_BLE_CONN_STATE_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_gap.h"
#include "sdk_common.h"

// One bit per connection, allocated by ble_conn_state_user_flag_acquire. The
// SDK caps the number; the harness offers more than the module asks for so
// acquisition never fails unless a test makes it fail.
#define BLE_CONN_STATE_USER_FLAG_COUNT		24
#define BLE_CONN_STATE_USER_FLAG_INVALID	0xFFFFU

typedef uint16_t ble_conn_state_user_flag_id_t;

typedef void (*ble_conn_state_user_function_t)(uint16_t ConnHdl, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

ble_conn_state_user_flag_id_t ble_conn_state_user_flag_acquire(void);
bool ble_conn_state_user_flag_get(uint16_t ConnHdl, ble_conn_state_user_flag_id_t FlagId);
void ble_conn_state_user_flag_set(uint16_t ConnHdl, ble_conn_state_user_flag_id_t FlagId,
								  bool Value);
uint32_t ble_conn_state_for_each_set_user_flag(ble_conn_state_user_flag_id_t FlagId,
											   ble_conn_state_user_function_t Func,
											   void *pCtx);
bool ble_conn_state_valid(uint16_t ConnHdl);
uint8_t ble_conn_state_role(uint16_t ConnHdl);
bool ble_conn_state_encrypted(uint16_t ConnHdl);
bool ble_conn_state_mitm_protected(uint16_t ConnHdl);
bool ble_conn_state_lesc(uint16_t ConnHdl);

#ifdef __cplusplus
}
#endif

#endif
