#ifndef __BT_TEST_PEER_MANAGER_TYPES_H__
#define __BT_TEST_PEER_MANAGER_TYPES_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_gap.h"
#include "sdk_common.h"

// Peer identifier. The SDK reserves the top of the range for the write buffer
// keys handed out by PDB_TEMP_PEER_ID, so a temporary id never collides with a
// stored one.
typedef uint16_t pm_peer_id_t;

#define PM_PEER_ID_INVALID			0xFFFFU
#define PM_PEER_ID_N_AVAILABLE_IDS		8U

#define PM_PEER_DATA_ID_BONDING			0x9U

// Security failure reasons reported in PM_EVT_CONN_SEC_FAILED. The Security
// Manager Pairing Failed reasons arrive here as BLE_GAP_SEC_STATUS_ values;
// the ones below are generated locally and sit above that range.
typedef uint16_t pm_sec_error_code_t;

#define PM_CONN_SEC_ERROR_BASE			0x1000U
#define PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING	(PM_CONN_SEC_ERROR_BASE + 0x06U)
#define PM_CONN_SEC_ERROR_MIC_FAILURE		(PM_CONN_SEC_ERROR_BASE + 0x3DU)
#define PM_CONN_SEC_ERROR_DISCONNECT		(PM_CONN_SEC_ERROR_BASE + 0x100U)
#define PM_CONN_SEC_ERROR_SMP_TIMEOUT		(PM_CONN_SEC_ERROR_BASE + 0x101U)

typedef enum {
	PM_CONN_SEC_PROCEDURE_ENCRYPTION,
	PM_CONN_SEC_PROCEDURE_BONDING,
	PM_CONN_SEC_PROCEDURE_PAIRING,
} pm_conn_sec_procedure_t;

typedef struct {
	uint8_t own_role;
	ble_gap_enc_key_t own_ltk;
	ble_gap_enc_key_t peer_ltk;
	ble_gap_id_key_t peer_ble_id;
	ble_gap_sign_info_t peer_sign_key;
} pm_peer_data_bonding_t;

typedef struct {
	uint8_t data_id;
	uint32_t length_words;
	union {
		pm_peer_data_bonding_t *p_bonding_data;
		void *p_all_data;
	};
} pm_peer_data_t;

typedef struct {
	uint8_t data_id;
	uint32_t length_words;
	union {
		pm_peer_data_bonding_t const *p_bonding_data;
		void const *p_all_data;
	};
} pm_peer_data_flash_t;

typedef struct {
	bool connected;
	bool encrypted;
	bool mitm_protected;
	bool bonded;
	bool lesc;
} pm_conn_sec_status_t;

typedef struct {
	bool allow_repairing;
} pm_conn_sec_config_t;

typedef enum {
	PM_EVT_BONDED_PEER_CONNECTED,
	PM_EVT_CONN_SEC_START,
	PM_EVT_CONN_SEC_SUCCEEDED,
	PM_EVT_CONN_SEC_FAILED,
	PM_EVT_CONN_SEC_CONFIG_REQ,
	PM_EVT_CONN_SEC_PARAMS_REQ,
	PM_EVT_STORAGE_FULL,
	PM_EVT_ERROR_UNEXPECTED,
	PM_EVT_PEER_DATA_UPDATE_SUCCEEDED,
	PM_EVT_PEER_DATA_UPDATE_FAILED,
	PM_EVT_PEER_DELETE_SUCCEEDED,
	PM_EVT_PEER_DELETE_FAILED,
	PM_EVT_PEERS_DELETE_SUCCEEDED,
	PM_EVT_PEERS_DELETE_FAILED,
	PM_EVT_LOCAL_DB_CACHE_APPLIED,
	PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED,
	PM_EVT_SERVICE_CHANGED_IND_SENT,
	PM_EVT_SERVICE_CHANGED_IND_CONFIRMED,
	PM_EVT_SLAVE_SECURITY_REQ,
	PM_EVT_FLASH_GARBAGE_COLLECTED,
	PM_EVT_FLASH_GARBAGE_COLLECTION_FAILED,
} pm_evt_id_t;

typedef struct {
	pm_conn_sec_procedure_t procedure;
} pm_conn_secured_evt_start_t;

typedef struct {
	pm_conn_sec_procedure_t procedure;
	bool data_stored;
} pm_conn_secured_evt_t;

typedef struct {
	pm_conn_sec_procedure_t procedure;
	pm_sec_error_code_t error;
	uint8_t error_src;
} pm_conn_secure_failed_evt_t;

typedef struct {
	ble_gap_sec_params_t const *p_peer_params;
	void const *p_context;
} pm_conn_sec_params_req_evt_t;

typedef struct {
	bool bond;
	bool mitm;
} pm_slave_security_req_evt_t;

typedef struct {
	ret_code_t error;
} pm_evt_error_unexpected_t;

typedef struct {
	pm_evt_id_t evt_id;
	uint16_t conn_handle;
	pm_peer_id_t peer_id;
	union {
		pm_conn_secured_evt_start_t conn_sec_start;
		pm_conn_secured_evt_t conn_sec_succeeded;
		pm_conn_secure_failed_evt_t conn_sec_failed;
		pm_conn_sec_params_req_evt_t conn_sec_params_req;
		pm_slave_security_req_evt_t slave_security_req;
		pm_evt_error_unexpected_t error_unexpected;
	} params;
} pm_evt_t;

#endif
