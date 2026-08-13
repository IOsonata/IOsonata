#ifndef __BT_TEST_BLE_GAP_H__
#define __BT_TEST_BLE_GAP_H__

#include <stdbool.h>
#include <stdint.h>

#define BLE_CONN_HANDLE_INVALID			0xFFFFU
#define BLE_GAP_LESC_P256_PK_LEN		64U
#define BLE_GAP_LESC_DHKEY_LEN			32U
#define BLE_GAP_SEC_KEY_LEN			16U
#define BLE_GAP_SEC_RAND_LEN			8U

// Security status values. The SoftDevice offsets the Security Manager
// Pairing Failed reasons of Core Vol 3 Part H Table 3.7 by 0x80; the two
// values below 0x80 are locally generated.
#define BLE_GAP_SEC_STATUS_SUCCESS		0x00U
#define BLE_GAP_SEC_STATUS_TIMEOUT		0x01U
#define BLE_GAP_SEC_STATUS_AUTH_REQ		0x83U
#define BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP	0x85U
#define BLE_GAP_SEC_STATUS_ENC_KEY_SIZE		0x86U
#define BLE_GAP_SEC_STATUS_UNSPECIFIED		0x88U
#define BLE_GAP_SEC_STATUS_REPEATED_ATTEMPTS	0x89U
#define BLE_GAP_SEC_STATUS_DHKEY_FAILURE	0x8BU

#define BLE_GAP_SEC_STATUS_SOURCE_LOCAL		0x00U
#define BLE_GAP_SEC_STATUS_SOURCE_REMOTE	0x01U

#define BLE_GAP_ROLE_INVALID			0x0U
#define BLE_GAP_ROLE_PERIPH			0x1U
#define BLE_GAP_ROLE_CENTRAL			0x2U

#define BLE_GAP_EVT_CONNECTED			0x10U
#define BLE_GAP_EVT_DISCONNECTED		0x11U
#define BLE_GAP_EVT_SEC_PARAMS_REQUEST		0x13U
#define BLE_GAP_EVT_SEC_INFO_REQUEST		0x14U
#define BLE_GAP_EVT_AUTH_STATUS			0x19U
#define BLE_GAP_EVT_CONN_SEC_UPDATE		0x1AU
#define BLE_GAP_EVT_LESC_DHKEY_REQUEST		0x1BU
#define BLE_GAP_EVT_SEC_REQUEST			0x1CU

typedef struct {
	uint8_t pk[BLE_GAP_LESC_P256_PK_LEN];
} ble_gap_lesc_p256_pk_t;

typedef struct {
	uint8_t key[BLE_GAP_LESC_DHKEY_LEN];
} ble_gap_lesc_dhkey_t;

typedef struct {
	uint8_t addr_id_peer : 1;
	uint8_t addr_type : 7;
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
	uint8_t bond : 1;
	uint8_t mitm : 1;
	uint8_t lesc : 1;
	uint8_t keypress : 1;
	uint8_t io_caps : 3;
	uint8_t oob : 1;
	uint8_t min_key_size;
	uint8_t max_key_size;
	struct {
		uint8_t enc : 1;
		uint8_t id : 1;
		uint8_t sign : 1;
		uint8_t link : 1;
	} kdist_own;
	struct {
		uint8_t enc : 1;
		uint8_t id : 1;
		uint8_t sign : 1;
		uint8_t link : 1;
	} kdist_peer;
} ble_gap_sec_params_t;

typedef struct {
	uint8_t ltk[BLE_GAP_SEC_KEY_LEN];
	uint8_t lesc : 1;
	uint8_t auth : 1;
	uint8_t ltk_len : 6;
} ble_gap_enc_info_t;

typedef struct {
	uint16_t ediv;
	uint8_t rand[BLE_GAP_SEC_RAND_LEN];
} ble_gap_master_id_t;

typedef struct {
	ble_gap_enc_info_t enc_info;
	ble_gap_master_id_t master_id;
} ble_gap_enc_key_t;

typedef struct {
	uint8_t irk[BLE_GAP_SEC_KEY_LEN];
} ble_gap_irk_t;

typedef struct {
	ble_gap_irk_t id_info;
	ble_gap_addr_t id_addr_info;
} ble_gap_id_key_t;

typedef struct {
	uint8_t csrk[BLE_GAP_SEC_KEY_LEN];
} ble_gap_sign_info_t;

typedef struct {
	ble_gap_enc_key_t *p_enc_key;
	ble_gap_id_key_t *p_id_key;
	ble_gap_sign_info_t *p_sign_key;
	ble_gap_lesc_p256_pk_t *p_pk;
} ble_gap_sec_keys_t;

typedef struct {
	ble_gap_sec_keys_t keys_own;
	ble_gap_sec_keys_t keys_peer;
} ble_gap_sec_keyset_t;

typedef struct {
	uint8_t sec_mode : 4;
	uint8_t lv : 4;
} ble_gap_conn_sec_mode_t;

typedef struct {
	ble_gap_conn_sec_mode_t sec_mode;
	uint8_t encr_key_size;
} ble_gap_conn_sec_t;

typedef struct {
	ble_gap_sec_params_t peer_params;
} ble_gap_evt_sec_params_request_t;

typedef struct {
	ble_gap_addr_t peer_addr;
	ble_gap_master_id_t master_id;
	uint8_t enc_info : 1;
	uint8_t id_info : 1;
	uint8_t sign_info : 1;
} ble_gap_evt_sec_info_request_t;

typedef struct {
	uint8_t bond : 1;
	uint8_t mitm : 1;
	uint8_t lesc : 1;
	uint8_t keypress : 1;
} ble_gap_evt_sec_request_t;

typedef struct {
	uint8_t auth_status;
	uint8_t error_src : 2;
	uint8_t bonded : 1;
	uint8_t lesc : 1;
} ble_gap_evt_auth_status_t;

typedef struct {
	ble_gap_conn_sec_t conn_sec;
} ble_gap_evt_conn_sec_update_t;

typedef struct {
	uint8_t reason;
} ble_gap_evt_disconnected_t;

typedef struct {
	ble_gap_evt_lesc_dhkey_request_t lesc_dhkey_request;
	ble_gap_evt_sec_params_request_t sec_params_request;
	ble_gap_evt_sec_info_request_t sec_info_request;
	ble_gap_evt_sec_request_t sec_request;
	ble_gap_evt_auth_status_t auth_status;
	ble_gap_evt_conn_sec_update_t conn_sec_update;
	ble_gap_evt_disconnected_t disconnected;
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

uint32_t sd_ble_gap_disconnect(uint16_t ConnHdl, uint8_t HciStatusCode);
uint32_t sd_ble_gap_authenticate(uint16_t ConnHdl,
								 const ble_gap_sec_params_t *pSecParams);
uint32_t sd_ble_gap_encrypt(uint16_t ConnHdl, const ble_gap_master_id_t *pMasterId,
							const ble_gap_enc_info_t *pEncInfo);
uint32_t sd_ble_gap_sec_params_reply(uint16_t ConnHdl, uint8_t SecStatus,
									 const ble_gap_sec_params_t *pSecParams,
									 const ble_gap_sec_keyset_t *pKeyset);
uint32_t sd_ble_gap_sec_info_reply(uint16_t ConnHdl,
								   const ble_gap_enc_info_t *pEncInfo,
								   const ble_gap_irk_t *pIdInfo,
								   const ble_gap_sign_info_t *pSignInfo);

#ifdef __cplusplus
}
#endif

#endif
