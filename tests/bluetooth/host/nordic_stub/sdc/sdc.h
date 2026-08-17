// Stand-in for the nrfxlib SoftDevice Controller header, enough of it to build
// ARM/Nordic/src/bt_hci_ctlr_sdc.cpp on a host.
//
// The configuration tag values, the sdc_cfg_t member names and the
// sdc_support_* names are the ones the vendor header has, because the whole
// point of the test that uses this is to check which of them the controller
// setup reaches. Anything the setup does not touch is left out.
#ifndef __SDC_STUB_H__
#define __SDC_STUB_H__

#include <stdint.h>

#define SDC_CFG_TYPE_NONE								0
#define SDC_CFG_TYPE_BUFFER_CFG							1
#define SDC_CFG_TYPE_PERIPHERAL_COUNT					2
#define SDC_CFG_TYPE_CENTRAL_COUNT						3
#define SDC_CFG_TYPE_ADV_COUNT							4
#define SDC_CFG_TYPE_ADV_BUFFER_CFG						5
#define SDC_CFG_TYPE_SCAN_BUFFER_CFG					6
#define SDC_CFG_TYPE_FAL_SIZE							7
#define SDC_CFG_TYPE_EXTENDED_FEATURE_PAGE_COUNT		8
#define SDC_CFG_TYPE_PERIODIC_ADV_COUNT					9
#define SDC_CFG_TYPE_PERIODIC_SYNC_COUNT				10
#define SDC_CFG_TYPE_PERIODIC_SYNC_BUFFER_CFG			11
#define SDC_CFG_TYPE_PERIODIC_ADV_LIST_SIZE				12
#define SDC_CFG_TYPE_PERIODIC_ADV_RSP_COUNT				13
#define SDC_CFG_TYPE_PERIODIC_ADV_RSP_BUFFER_CFG		14
#define SDC_CFG_TYPE_PERIODIC_ADV_RSP_FAILURE_REPORTING_CFG	15
#define SDC_CFG_TYPE_PERIODIC_SYNC_RSP_TX_BUFFER_CFG	16
#define SDC_CFG_TYPE_EVENT_LENGTH						17

#define SDC_DEFAULT_RESOURCE_CFG_TAG					0

// What sdc_hci_get documents as the smallest buffer it will write into.
#define HCI_MSG_BUFFER_MAX_SIZE							258

typedef struct { uint8_t count; } sdc_cfg_role_count_t;

typedef struct {
	uint8_t rx_packet_size;
	uint8_t tx_packet_size;
	uint8_t rx_packet_count;
	uint8_t tx_packet_count;
} sdc_cfg_buffer_cfg_t;

typedef struct { uint16_t max_adv_data; } sdc_cfg_adv_buffer_cfg_t;
typedef struct { uint8_t count; } sdc_cfg_scan_buffer_cfg_t;
typedef struct { uint16_t event_length_us; } sdc_cfg_event_length_t;

typedef struct {
	uint8_t tx_buffer_count;
	uint8_t max_tx_data_size;
	uint8_t rx_buffer_count;
} sdc_cfg_periodic_adv_rsp_buffer_cfg_t;

typedef union {
	sdc_cfg_buffer_cfg_t buffer_cfg;
	sdc_cfg_role_count_t peripheral_count;
	sdc_cfg_role_count_t central_count;
	sdc_cfg_role_count_t adv_count;
	sdc_cfg_adv_buffer_cfg_t adv_buffer_cfg;
	sdc_cfg_scan_buffer_cfg_t scan_buffer_cfg;
	sdc_cfg_event_length_t event_length;
	uint16_t fal_size;
	uint8_t extended_feature_page_count;
	sdc_cfg_role_count_t periodic_adv_count;
	sdc_cfg_role_count_t periodic_sync_count;
	sdc_cfg_scan_buffer_cfg_t periodic_sync_buffer_cfg;
	uint8_t periodic_adv_list_size;
	sdc_cfg_role_count_t periodic_adv_rsp_count;
	sdc_cfg_periodic_adv_rsp_buffer_cfg_t periodic_adv_rsp_buffer_cfg;
	uint8_t periodic_adv_rsp_failure_reporting_cfg;
	sdc_cfg_scan_buffer_cfg_t periodic_sync_rsp_tx_buffer_cfg;
} sdc_cfg_t;

typedef void (*sdc_fault_handler_t)(const char *, uint32_t);
typedef void (*sdc_callback_t)(void);
typedef struct { void (*rand_poll)(uint8_t *, uint8_t); } sdc_rand_source_t;

#ifdef __cplusplus
extern "C" {
#endif

int32_t sdc_init(sdc_fault_handler_t Handler);
int32_t sdc_rand_source_register(const sdc_rand_source_t *pSource);
int32_t sdc_cfg_set(uint8_t Tag, uint8_t Type, const sdc_cfg_t *pCfg);
int32_t sdc_enable(sdc_callback_t Callback, uint8_t *pMemPool);
int32_t sdc_disable(void);

void sdc_support_adv(void);
void sdc_support_ext_adv(void);
void sdc_support_scan(void);
void sdc_support_ext_scan(void);
void sdc_support_peripheral(void);
void sdc_support_central(void);
void sdc_support_ext_central(void);
void sdc_support_dle_peripheral(void);
void sdc_support_dle_central(void);
void sdc_support_phy_update_peripheral(void);
void sdc_support_phy_update_central(void);
void sdc_support_le_2m_phy(void);
void sdc_support_le_coded_phy(void);
void sdc_support_le_power_control(void);
void sdc_support_le_power_control_peripheral(void);
void sdc_support_le_power_control_central(void);
void sdc_support_le_conn_cte_rsp_peripheral(void);
void sdc_support_le_conn_cte_rsp_central(void);
void sdc_support_le_periodic_adv(void);
void sdc_support_le_periodic_sync(void);
void sdc_support_le_periodic_adv_with_rsp(void);
void sdc_support_le_periodic_sync_with_rsp(void);

#ifdef __cplusplus
}
#endif

#endif // __SDC_STUB_H__
