#ifndef __BT_TEST_BLE_GAP_H__
#define __BT_TEST_BLE_GAP_H__

#include <stdbool.h>
#include <stdint.h>

#define BLE_CONN_HANDLE_INVALID			0xFFFFU
#define BLE_GAP_LESC_P256_PK_LEN		64U
#define BLE_GAP_LESC_DHKEY_LEN			32U

#define BLE_GAP_SEC_STATUS_SUCCESS		0x00U
#define BLE_GAP_SEC_STATUS_DHKEY_FAILURE	0x0BU

#define BLE_GAP_EVT_CONNECTED			0x10U
#define BLE_GAP_EVT_DISCONNECTED		0x11U
#define BLE_GAP_EVT_SEC_PARAMS_REQUEST		0x13U
#define BLE_GAP_EVT_AUTH_STATUS			0x19U
#define BLE_GAP_EVT_LESC_DHKEY_REQUEST		0x1BU

typedef struct {
	uint8_t pk[BLE_GAP_LESC_P256_PK_LEN];
} ble_gap_lesc_p256_pk_t;

typedef struct {
	uint8_t key[BLE_GAP_LESC_DHKEY_LEN];
} ble_gap_lesc_dhkey_t;

typedef struct {
	uint8_t addr_type;
	uint8_t addr[6];
} ble_gap_addr_t;

typedef struct {
	ble_gap_addr_t addr;
	uint8_t r[16];
	uint8_t c[16];
} ble_gap_lesc_oob_data_t;

typedef struct {
	bool oobd_req;
	ble_gap_lesc_p256_pk_t *p_pk_peer;
} ble_gap_evt_lesc_dhkey_request_t;

typedef struct {
	ble_gap_evt_lesc_dhkey_request_t lesc_dhkey_request;
} ble_gap_evt_params_t;

typedef struct {
	uint16_t conn_handle;
	ble_gap_evt_params_t params;
} ble_gap_evt_t;

#ifdef __cplusplus
extern "C" {
#endif

uint32_t sd_ble_gap_lesc_oob_data_get(uint16_t ConnHdl,
								 ble_gap_lesc_p256_pk_t *pPk,
								 ble_gap_lesc_oob_data_t *pData);
uint32_t sd_ble_gap_lesc_oob_data_set(uint16_t ConnHdl,
								 ble_gap_lesc_oob_data_t *pOwn,
								 ble_gap_lesc_oob_data_t *pPeer);

#ifdef __cplusplus
}
#endif

#endif
