/* STM32WBA65I-DK1 adaptation of the CubeWBA throughput client. */
#ifndef ST_THROUGHPUT_DK1_CONFIG_H
#define ST_THROUGHPUT_DK1_CONFIG_H

#include "app_conf.h"
#include "ble_conf.h"
#include "simple_nvm_arbiter_conf.h"

/* GAP events must reach the automatic client, not the interactive sample. */
#undef BLE_CFG_MAX_NBR_CB
#define BLE_CFG_MAX_NBR_CB 1

/* The DK1 shares USART1 PB12/PA8 with the Nucleo sample, but not its
 * buttons or LEDs. The comparison starts automatically after host init.
 * Force this header before SDK headers so quoted app_conf.h includes
 * cannot restore the Nucleo pin configuration. */
#undef CFG_BSP_ON_NUCLEO
#undef CFG_LED_SUPPORTED
#define CFG_LED_SUPPORTED 0
#undef CFG_BUTTON_SUPPORTED
#define CFG_BUTTON_SUPPORTED 0

/* Match IOsonata WBA host capacity; the benchmark still requests MTU 247.
 * The host FIFO size here is in bytes. */
#undef CFG_BLE_NUM_GATT_SERVICES
#define CFG_BLE_NUM_GATT_SERVICES 8
#undef CFG_BLE_NUM_GATT_ATTRIBUTES
#define CFG_BLE_NUM_GATT_ATTRIBUTES 68
#undef CFG_BLE_ATT_VALUE_ARRAY_SIZE
#define CFG_BLE_ATT_VALUE_ARRAY_SIZE 1344
#undef PREP_WRITE_LIST_SIZE
#define PREP_WRITE_LIST_SIZE BLE_DEFAULT_PREP_WRITE_LIST_SIZE
#undef CFG_BLE_MBLOCK_COUNT_MARGIN
#define CFG_BLE_MBLOCK_COUNT_MARGIN 80
#undef CFG_BLE_MBLOCK_COUNT
#define CFG_BLE_MBLOCK_COUNT (BLE_MBLOCKS_CALC(PREP_WRITE_LIST_SIZE, \
	CFG_BLE_ATT_MTU_MAX, CFG_BLE_NUM_LINK) + CFG_BLE_MBLOCK_COUNT_MARGIN)
#undef CFG_BLE_EATT_BEARER_PER_LINK
#define CFG_BLE_EATT_BEARER_PER_LINK 0
#undef CFG_BLE_EXTRA_DATA_BUF_SIZE
#define CFG_BLE_EXTRA_DATA_BUF_SIZE 0
#undef CFG_BLE_LONG_WRITE_DATA_BUF_SIZE
#define CFG_BLE_LONG_WRITE_DATA_BUF_SIZE 0
/* 384 host cache words plus the two words reserved by ST's NVM manager. */
#undef SNVMA_NVM_ID_1_BUFFER_1_SIZE
#define SNVMA_NVM_ID_1_BUFFER_1_SIZE ((384U + 2U) * 8U)
#undef CFG_RF_TX_POWER_TABLE_ID
#define CFG_RF_TX_POWER_TABLE_ID 0
#undef CFG_BLE_ATT_MTU_MAX
#define CFG_BLE_ATT_MTU_MAX 251
#undef CFG_BLE_NUM_LINK
#define CFG_BLE_NUM_LINK 2
#undef CFG_BLE_HOST_EVENT_BUF_SIZE
#define CFG_BLE_HOST_EVENT_BUF_SIZE 4096
#undef CFG_BLE_OPTIONS
#define CFG_BLE_OPTIONS BLE_OPTIONS_EXTENDED_ADV
#undef CFG_BLE_ATTR_PREPARE_WRITE_VALUE_SIZE
#define CFG_BLE_ATTR_PREPARE_WRITE_VALUE_SIZE BLE_DEFAULT_PREP_WRITE_LIST_SIZE
#undef CFG_BLE_COC_NBR_MAX
#define CFG_BLE_COC_NBR_MAX 0
#undef CFG_BLE_COC_MPS_MAX
#define CFG_BLE_COC_MPS_MAX 0
#undef CFG_BLE_COC_INITIATOR_NBR_MAX
#define CFG_BLE_COC_INITIATOR_NBR_MAX 0
#undef CFG_PHY_PREF_TX
#define CFG_PHY_PREF_TX HCI_TX_PHYS_LE_1M_PREF
#undef CFG_PHY_PREF_RX
#define CFG_PHY_PREF_RX HCI_RX_PHYS_LE_1M_PREF

#endif
