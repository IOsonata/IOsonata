/**-------------------------------------------------------------------------
@file	bm_config_defaults.h

@brief	Default CONFIG_ values extracted from sdk-nrf-bm Kconfig files

These defaults match the Kconfig defaults for sdk-nrf-bm v2.9.1.
Override any value by defining it before including bm_compat.h or
in your project's build configuration.

@author	IOsonata
@date	2025

@license MIT
----------------------------------------------------------------------------*/

#ifndef BM_CONFIG_DEFAULTS_H__
#define BM_CONFIG_DEFAULTS_H__

/* === SoftDevice / SDH BLE configuration === */

#ifndef CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT
#define CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT       2
#endif

#ifndef CONFIG_NRF_SDH_BLE_PERIPHERAL_LINK_COUNT
#define CONFIG_NRF_SDH_BLE_PERIPHERAL_LINK_COUNT  1
#endif

#ifndef CONFIG_NRF_SDH_BLE_CENTRAL_LINK_COUNT
#define CONFIG_NRF_SDH_BLE_CENTRAL_LINK_COUNT     0
#endif

#ifndef CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE
#define CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE      247
#endif

#ifndef CONFIG_NRF_SDH_BLE_GAP_EVENT_LENGTH
#define CONFIG_NRF_SDH_BLE_GAP_EVENT_LENGTH       3
#endif

#ifndef CONFIG_NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE
#define CONFIG_NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE    1408
#endif

#ifndef CONFIG_NRF_SDH_BLE_VS_UUID_COUNT
#define CONFIG_NRF_SDH_BLE_VS_UUID_COUNT          0
#endif

#ifndef CONFIG_NRF_SDH_BLE_CONN_TAG
#define CONFIG_NRF_SDH_BLE_CONN_TAG               99
#endif

#ifndef CONFIG_NRF_SDH_DISPATCH_MODEL
#define CONFIG_NRF_SDH_DISPATCH_MODEL             0
#endif

#ifndef CONFIG_NRF_SDH_CLOCK_LF_SRC
#define CONFIG_NRF_SDH_CLOCK_LF_SRC               1  /* 0=RC, 1=LFXO (fallback only, runtime uses g_McuOsc) */
#endif

/* RC calibration defaults — only used when source is RC;
 * nrf_sdh_enable() forces these to 0 when LFXO is detected at runtime.
 */
#ifndef CONFIG_NRF_SDH_CLOCK_LF_RC_CTIV
#define CONFIG_NRF_SDH_CLOCK_LF_RC_CTIV           16
#endif

#ifndef CONFIG_NRF_SDH_CLOCK_LF_RC_TEMP_CTIV
#define CONFIG_NRF_SDH_CLOCK_LF_RC_TEMP_CTIV      2
#endif

#ifndef CONFIG_NRF_SDH_CLOCK_LF_ACCURACY
#define CONFIG_NRF_SDH_CLOCK_LF_ACCURACY          0
#endif

#ifndef CONFIG_NRF_SDH_CLOCK_HFCLK_LATENCY
#define CONFIG_NRF_SDH_CLOCK_HFCLK_LATENCY        1500
#endif

#ifndef CONFIG_NRF_SDH_CLOCK_HFINT_CALIBRATION_INTERVAL
#define CONFIG_NRF_SDH_CLOCK_HFINT_CALIBRATION_INTERVAL  60
#endif

//#ifndef CONFIG_NRF_SDH_DISPATCH_MODEL_IRQ
//#define CONFIG_NRF_SDH_DISPATCH_MODEL_IRQ         0
//#endif

/* SDH dispatch model — define ONE:
 * CONFIG_NRF_SDH_DISPATCH_MODEL_IRQ is default (defined below).
 * To use scheduler: #define CONFIG_NRF_SDH_DISPATCH_MODEL_SCHED 1
 * To use polling:   #define CONFIG_NRF_SDH_DISPATCH_MODEL_POLL  1
 */

#ifndef CONFIG_NRF_SDH_DISPATCH_MODEL_SCHED
#define CONFIG_NRF_SDH_DISPATCH_MODEL_SCHED		1
#endif

#ifndef CONFIG_NRF_SDH_CLOCK_LF_SRC_XO
#define CONFIG_NRF_SDH_CLOCK_LF_SRC_XO           1
#endif

#ifndef CONFIG_NRF_SDH_BLE_SERVICE_CHANGED
#define CONFIG_NRF_SDH_BLE_SERVICE_CHANGED        0
#endif

/* BLE role enables — IOsonata supports both peripheral and central
 * at runtime via BtAppCfg_t.Role, so both must always be compiled in.
 * The SDK uses a mix of #if defined() and #if value guards, so both
 * macros must be defined AND set to 1. */
#ifndef CONFIG_SOFTDEVICE_PERIPHERAL
#define CONFIG_SOFTDEVICE_PERIPHERAL              1
#endif

#ifndef CONFIG_SOFTDEVICE_CENTRAL
#define CONFIG_SOFTDEVICE_CENTRAL                 1
#endif

#ifndef CONFIG_NRF_SDH_SOC_RAND_SEED
#define CONFIG_NRF_SDH_SOC_RAND_SEED              1
#endif

#ifndef CONFIG_NRF_SDH_STR_TABLES
#define CONFIG_NRF_SDH_STR_TABLES                 1
#endif


/* === Peer Manager === */

#ifndef CONFIG_PM_MAX_REGISTRANTS
#define CONFIG_PM_MAX_REGISTRANTS                 3
#endif

#ifndef CONFIG_PM_FLASH_BUFFERS
#define CONFIG_PM_FLASH_BUFFERS                   4
#endif

#ifndef CONFIG_PM_BM_ZMS_SECTOR_SIZE
#define CONFIG_PM_BM_ZMS_SECTOR_SIZE              1024
#endif

#ifndef CONFIG_PM_HANDLER_SEC_DELAY_MS
#define CONFIG_PM_HANDLER_SEC_DELAY_MS            0
#endif

#ifndef CONFIG_PM_RA_PROTECTION_TRACKED_PEERS_NUM
#define CONFIG_PM_RA_PROTECTION_TRACKED_PEERS_NUM 8
#endif

#ifndef CONFIG_PM_RA_PROTECTION_MIN_WAIT_INTERVAL
#define CONFIG_PM_RA_PROTECTION_MIN_WAIT_INTERVAL 4000
#endif

#ifndef CONFIG_PM_RA_PROTECTION_MAX_WAIT_INTERVAL
#define CONFIG_PM_RA_PROTECTION_MAX_WAIT_INTERVAL 64000
#endif

#ifndef CONFIG_PM_RA_PROTECTION_REWARD_PERIOD
#define CONFIG_PM_RA_PROTECTION_REWARD_PERIOD     10000
#endif

#ifndef CONFIG_PM_PEER_RANKS
#define CONFIG_PM_PEER_RANKS                      1
#endif

/* Define to enable: #define CONFIG_PM_SERVICE_CHANGED 1 */

/* Define to enable LESC (undefined = disabled):
 * #define CONFIG_PM_LESC                     1
 * #define CONFIG_PM_LESC_GENERATE_NEW_KEYS   1
 * #define CONFIG_PM_LESC_PRIVATE_KEY_EXPORT  1
 */


/* === BLE GATT Queue === */

#ifndef CONFIG_BLE_GQ_MAX_CONNECTIONS
#define CONFIG_BLE_GQ_MAX_CONNECTIONS             1
#endif

#ifndef CONFIG_BLE_GQ_QUEUE_SIZE
#define CONFIG_BLE_GQ_QUEUE_SIZE                  8
#endif

#ifndef CONFIG_BLE_GQ_HEAP_SIZE
#define CONFIG_BLE_GQ_HEAP_SIZE                   256
#endif


/* === BLE Advertising === */

#ifndef CONFIG_BLE_ADV_NAME
#define CONFIG_BLE_ADV_NAME                       "nRF_BM_device"
#endif

#ifndef CONFIG_BLE_ADV_RESTART_ON_DISCONNECT
#define CONFIG_BLE_ADV_RESTART_ON_DISCONNECT      1
#endif

#ifndef CONFIG_BLE_ADV_EXTENDED_ADVERTISING
#define CONFIG_BLE_ADV_EXTENDED_ADVERTISING        1
#endif

#ifndef CONFIG_BLE_ADV_DIRECTED_ADVERTISING
#define CONFIG_BLE_ADV_DIRECTED_ADVERTISING        1
#endif

#ifndef CONFIG_BLE_ADV_DIRECTED_ADVERTISING_HIGH_DUTY
#define CONFIG_BLE_ADV_DIRECTED_ADVERTISING_HIGH_DUTY  1
#endif

#ifndef CONFIG_BLE_ADV_USE_ALLOW_LIST
#define CONFIG_BLE_ADV_USE_ALLOW_LIST             0
#endif

#ifndef CONFIG_BLE_ADV_FAST_ADVERTISING_INTERVAL
#define CONFIG_BLE_ADV_FAST_ADVERTISING_INTERVAL  32
#endif

#ifndef CONFIG_BLE_ADV_FAST_ADVERTISING_TIMEOUT
#define CONFIG_BLE_ADV_FAST_ADVERTISING_TIMEOUT   1000
#endif

#ifndef CONFIG_BLE_ADV_SLOW_ADVERTISING_INTERVAL
#define CONFIG_BLE_ADV_SLOW_ADVERTISING_INTERVAL  64
#endif

#ifndef CONFIG_BLE_ADV_SLOW_ADVERTISING_TIMEOUT
#define CONFIG_BLE_ADV_SLOW_ADVERTISING_TIMEOUT   18000
#endif

#ifndef CONFIG_BLE_ADV_DIRECTED_ADVERTISING_INTERVAL
#define CONFIG_BLE_ADV_DIRECTED_ADVERTISING_INTERVAL  32
#endif

#ifndef CONFIG_BLE_ADV_DIRECTED_ADVERTISING_TIMEOUT
#define CONFIG_BLE_ADV_DIRECTED_ADVERTISING_TIMEOUT   100
#endif

#ifndef CONFIG_BLE_ADV_PRIMARY_PHY
#define CONFIG_BLE_ADV_PRIMARY_PHY                0x00
#endif

#ifndef CONFIG_BLE_ADV_SECONDARY_PHY
#define CONFIG_BLE_ADV_SECONDARY_PHY              0x00
#endif


/* === BLE Connection Parameters === */

#ifndef CONFIG_BLE_CONN_PARAMS_MIN_CONN_INTERVAL
#define CONFIG_BLE_CONN_PARAMS_MIN_CONN_INTERVAL  6
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_MAX_CONN_INTERVAL
#define CONFIG_BLE_CONN_PARAMS_MAX_CONN_INTERVAL  256
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_PERIPHERAL_LATENCY
#define CONFIG_BLE_CONN_PARAMS_PERIPHERAL_LATENCY 0
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_SUP_TIMEOUT
#define CONFIG_BLE_CONN_PARAMS_SUP_TIMEOUT        100
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_MAX_PERIPHERAL_LATENCY_DEVIATION
#define CONFIG_BLE_CONN_PARAMS_MAX_PERIPHERAL_LATENCY_DEVIATION  6
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_MAX_SUP_TIMEOUT_DEVIATION
#define CONFIG_BLE_CONN_PARAMS_MAX_SUP_TIMEOUT_DEVIATION  400
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_NEGOTIATION_RETRIES
#define CONFIG_BLE_CONN_PARAMS_NEGOTIATION_RETRIES  2
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_DATA_LENGTH_TX
#define CONFIG_BLE_CONN_PARAMS_DATA_LENGTH_TX     27
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_DATA_LENGTH_RX
#define CONFIG_BLE_CONN_PARAMS_DATA_LENGTH_RX     27
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_PHY
#define CONFIG_BLE_CONN_PARAMS_PHY                0x00
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_ATT_MTU
#define CONFIG_BLE_CONN_PARAMS_ATT_MTU            CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_INITIATE_ATT_MTU_EXCHANGE
#define CONFIG_BLE_CONN_PARAMS_INITIATE_ATT_MTU_EXCHANGE  1
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_AUTO_ATT_MTU
#define CONFIG_BLE_CONN_PARAMS_AUTO_ATT_MTU       1
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_AUTO_DATA_LENGTH
#define CONFIG_BLE_CONN_PARAMS_AUTO_DATA_LENGTH   1
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_DISCONNECT_ON_FAILURE
#define CONFIG_BLE_CONN_PARAMS_DISCONNECT_ON_FAILURE  0
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_INITIATE_DATA_LENGTH_UPDATE
#define CONFIG_BLE_CONN_PARAMS_INITIATE_DATA_LENGTH_UPDATE  0
#endif

#ifndef CONFIG_BLE_CONN_PARAMS_INITIATE_PHY_UPDATE
#define CONFIG_BLE_CONN_PARAMS_INITIATE_PHY_UPDATE  0
#endif


/* === BLE Conn State === */

#ifndef CONFIG_BLE_CONN_STATE_USER_FLAG_COUNT
#define CONFIG_BLE_CONN_STATE_USER_FLAG_COUNT      24
#endif


/* === BLE DB Discovery === */

#ifndef CONFIG_BLE_DB_DISCOVERY_MAX_SRV
#define CONFIG_BLE_DB_DISCOVERY_MAX_SRV           6
#endif

#ifndef CONFIG_BLE_DB_DISCOVERY_SRV_DISC_START_HANDLE
#define CONFIG_BLE_DB_DISCOVERY_SRV_DISC_START_HANDLE  0x0001
#endif


/* === BLE Scan === */

#ifndef CONFIG_BLE_SCAN_FILTER
#define CONFIG_BLE_SCAN_FILTER                    1
#endif

#ifndef CONFIG_BLE_SCAN_NAME_COUNT
#define CONFIG_BLE_SCAN_NAME_COUNT                1
#endif

#ifndef CONFIG_BLE_SCAN_NAME_MAX_LEN
#define CONFIG_BLE_SCAN_NAME_MAX_LEN              32
#endif

#ifndef CONFIG_BLE_SCAN_SHORT_NAME_COUNT
#define CONFIG_BLE_SCAN_SHORT_NAME_COUNT          0
#endif

#ifndef CONFIG_BLE_SCAN_SHORT_NAME_MAX_LEN
#define CONFIG_BLE_SCAN_SHORT_NAME_MAX_LEN        32
#endif

#ifndef CONFIG_BLE_SCAN_UUID_COUNT
#define CONFIG_BLE_SCAN_UUID_COUNT                0
#endif

#ifndef CONFIG_BLE_SCAN_ADDRESS_COUNT
#define CONFIG_BLE_SCAN_ADDRESS_COUNT             0
#endif

#ifndef CONFIG_BLE_SCAN_APPEARANCE_COUNT
#define CONFIG_BLE_SCAN_APPEARANCE_COUNT           0
#endif

#ifndef CONFIG_BLE_SCAN_BUFFER_SIZE
#define CONFIG_BLE_SCAN_BUFFER_SIZE               32
#endif

#ifndef CONFIG_BLE_SCAN_INTERVAL
#define CONFIG_BLE_SCAN_INTERVAL                  160
#endif

#ifndef CONFIG_BLE_SCAN_WINDOW
#define CONFIG_BLE_SCAN_WINDOW                    80
#endif

#ifndef CONFIG_BLE_SCAN_DURATION
#define CONFIG_BLE_SCAN_DURATION                  0
#endif

#ifndef CONFIG_BLE_SCAN_MIN_CONNECTION_INTERVAL
#define CONFIG_BLE_SCAN_MIN_CONNECTION_INTERVAL    6
#endif

#ifndef CONFIG_BLE_SCAN_MAX_CONNECTION_INTERVAL
#define CONFIG_BLE_SCAN_MAX_CONNECTION_INTERVAL    24
#endif

#ifndef CONFIG_BLE_SCAN_PERIPHERAL_LATENCY
#define CONFIG_BLE_SCAN_PERIPHERAL_LATENCY        0
#endif

#ifndef CONFIG_BLE_SCAN_SUPERVISION_TIMEOUT
#define CONFIG_BLE_SCAN_SUPERVISION_TIMEOUT        3200
#endif


/* === BLE QWR === */

#ifndef CONFIG_BLE_QWR_MAX_ATTR
#define CONFIG_BLE_QWR_MAX_ATTR                   1
#endif


/* === BLE HIDS === */

/* Define to enable:
 * #define CONFIG_BLE_HIDS_BOOT_MOUSE     1
 * #define CONFIG_BLE_HIDS_BOOT_KEYBOARD  1
 */

#ifndef CONFIG_BLE_HIDS_INPUT_REPORT_MAX_NUM
#define CONFIG_BLE_HIDS_INPUT_REPORT_MAX_NUM      10
#endif

#ifndef CONFIG_BLE_HIDS_INPUT_REPORT_MAX_LEN
#define CONFIG_BLE_HIDS_INPUT_REPORT_MAX_LEN      0
#endif

#ifndef CONFIG_BLE_HIDS_OUTPUT_REPORT_MAX_NUM
#define CONFIG_BLE_HIDS_OUTPUT_REPORT_MAX_NUM     10
#endif

#ifndef CONFIG_BLE_HIDS_OUTPUT_REPORT_MAX_LEN
#define CONFIG_BLE_HIDS_OUTPUT_REPORT_MAX_LEN     0
#endif

#ifndef CONFIG_BLE_HIDS_FEATURE_REPORT_MAX_NUM
#define CONFIG_BLE_HIDS_FEATURE_REPORT_MAX_NUM    10
#endif

#ifndef CONFIG_BLE_HIDS_FEATURE_REPORT_MAX_LEN
#define CONFIG_BLE_HIDS_FEATURE_REPORT_MAX_LEN    0
#endif

#ifndef CONFIG_BLE_HIDS_MAX_CLIENTS
#define CONFIG_BLE_HIDS_MAX_CLIENTS               1
#endif

#ifndef CONFIG_BLE_HIDS_DEFAULT_PROTOCOL_MODE
#define CONFIG_BLE_HIDS_DEFAULT_PROTOCOL_MODE     0
#endif


/* === BLE HRS === */

#ifndef CONFIG_BLE_HRS_MAX_BUFFERED_RR_INTERVALS
#define CONFIG_BLE_HRS_MAX_BUFFERED_RR_INTERVALS  20
#endif

#ifndef CONFIG_BLE_HRS_CLIENT_RR_INTERVALS_MAX_COUNT
#define CONFIG_BLE_HRS_CLIENT_RR_INTERVALS_MAX_COUNT  20
#endif


/* === BLE CGMS === */

#ifndef CONFIG_BLE_CGMS_DB_RECORDS_MAX
#define CONFIG_BLE_CGMS_DB_RECORDS_MAX            100
#endif


/* === BLE Radio Notification === */

/* Define ONE to select radio notification type:
 * #define CONFIG_BLE_RADIO_NOTIFICATION_ON_ACTIVE    1
 * #define CONFIG_BLE_RADIO_NOTIFICATION_ON_INACTIVE  1
 * #define CONFIG_BLE_RADIO_NOTIFICATION_ON_BOTH      1
 */

#ifndef CONFIG_BLE_RADIO_NOTIFICATION_IRQ_PRIO
#define CONFIG_BLE_RADIO_NOTIFICATION_IRQ_PRIO    3
#endif


/* === BLE BMS === */

#ifndef CONFIG_BLE_BMS_AUTHORIZATION_CODE
#define CONFIG_BLE_BMS_AUTHORIZATION_CODE          "ABCD"
#endif


/* === BLE DIS === */

#ifndef CONFIG_BLE_DIS_MANUFACTURER_NAME
#define CONFIG_BLE_DIS_MANUFACTURER_NAME          "Nordic Semiconductor"
#endif

#ifndef CONFIG_BLE_DIS_MODEL_NUMBER
#define CONFIG_BLE_DIS_MODEL_NUMBER               "SOC"
#endif

#ifndef CONFIG_BLE_DIS_PNP_VID_SRC
#define CONFIG_BLE_DIS_PNP_VID_SRC                1
#endif

#ifndef CONFIG_BLE_DIS_PNP_VID
#define CONFIG_BLE_DIS_PNP_VID                    0
#endif

#ifndef CONFIG_BLE_DIS_PNP_PID
#define CONFIG_BLE_DIS_PNP_PID                    0
#endif

#ifndef CONFIG_BLE_DIS_PNP_VER
#define CONFIG_BLE_DIS_PNP_VER                    1
#endif

#ifndef CONFIG_BLE_DIS_SERIAL_NUMBER
#define CONFIG_BLE_DIS_SERIAL_NUMBER              ""
#endif

#ifndef CONFIG_BLE_DIS_FW_REVISION
#define CONFIG_BLE_DIS_FW_REVISION                ""
#endif

#ifndef CONFIG_BLE_DIS_HW_REVISION
#define CONFIG_BLE_DIS_HW_REVISION                ""
#endif

#ifndef CONFIG_BLE_DIS_SW_REVISION
#define CONFIG_BLE_DIS_SW_REVISION                ""
#endif

#ifndef CONFIG_BLE_DIS_SYSTEM_ID
#define CONFIG_BLE_DIS_SYSTEM_ID                  0
#endif

#ifndef CONFIG_BLE_DIS_SYSTEM_ID_MID
#define CONFIG_BLE_DIS_SYSTEM_ID_MID              0
#endif

#ifndef CONFIG_BLE_DIS_SYSTEM_ID_OUI
#define CONFIG_BLE_DIS_SYSTEM_ID_OUI              0
#endif

#ifndef CONFIG_BLE_DIS_PNP_ID
#define CONFIG_BLE_DIS_PNP_ID                     0
#endif

#ifndef CONFIG_BLE_DIS_REGULATORY_CERT
#define CONFIG_BLE_DIS_REGULATORY_CERT            0
#endif

#ifndef CONFIG_BLE_DIS_REGULATORY_CERT_LIST
#define CONFIG_BLE_DIS_REGULATORY_CERT_LIST       0
#endif


/* === Drivers / Platform === */

#ifndef CONFIG_BM_TIMER_IRQ_PRIO
#define CONFIG_BM_TIMER_IRQ_PRIO                  5
#endif

#ifndef CONFIG_BM_GPIOTE_IRQ_PRIO
#define CONFIG_BM_GPIOTE_IRQ_PRIO                 3
#endif

#ifndef CONFIG_BM_SCHEDULER_BUF_SIZE
#define CONFIG_BM_SCHEDULER_BUF_SIZE              512
#endif

#ifndef CONFIG_BM_BUTTONS_NUM_PINS
#define CONFIG_BM_BUTTONS_NUM_PINS                24
#endif


/* === ZMS (Zephyr-less Memory Storage) === */

#ifndef CONFIG_BM_ZMS_CUSTOM_BLOCK_SIZE
#define CONFIG_BM_ZMS_CUSTOM_BLOCK_SIZE           32
#endif

#ifndef CONFIG_BM_ZMS_LOOKUP_CACHE_SIZE
#define CONFIG_BM_ZMS_LOOKUP_CACHE_SIZE           128
#endif

#ifndef CONFIG_BM_ZMS_OP_QUEUE_SIZE
#define CONFIG_BM_ZMS_OP_QUEUE_SIZE               16
#endif


/* === Flash / Storage === */

/*
 * Memory layout from DTS (nRF54L15 S145 SoftDevice):
 *
 * SRAM 0x20000000:
 *   softdevice_static_ram  @ 0x0000  size 0x1780
 *   softdevice_dynamic_ram @ 0x1780  size 0x3000 (12K)
 *   app_ram                @ 0x4780  size 0x3B480 (237K)
 *
 * RRAM partitions:
 *   slot0_partition         @ 0x000000  size 1371K
 *   storage_partition       @ 0x156c00  size 8K
 *     peer_manager_partition  @ 0x156c00  size 4K
 *     storage0_partition      @ 0x157c00  size 4K
 *   softdevice_partition    @ 0x158c00  size 144K
 */

#ifndef CONFIG_APP_RAM_START
#define CONFIG_APP_RAM_START              0x20004780UL
#endif

#ifndef CONFIG_PM_PARTITION_OFFSET
#define CONFIG_PM_PARTITION_OFFSET        0x156c00UL
#endif

#ifndef CONFIG_PM_PARTITION_SIZE
#define CONFIG_PM_PARTITION_SIZE          0x1000UL
#endif

#ifndef CONFIG_STORAGE0_PARTITION_OFFSET
#define CONFIG_STORAGE0_PARTITION_OFFSET  0x157c00UL
#endif

#ifndef CONFIG_STORAGE0_PARTITION_SIZE
#define CONFIG_STORAGE0_PARTITION_SIZE    0x1000UL
#endif

#ifndef CONFIG_NRF_RRAM_REGION_ADDRESS_RESOLUTION
#define CONFIG_NRF_RRAM_REGION_ADDRESS_RESOLUTION 0x1000
#endif

#ifndef CONFIG_NRF_RRAM_REGION_SIZE_UNIT
#define CONFIG_NRF_RRAM_REGION_SIZE_UNIT          0x1000
#endif


/* === MCUmgr Transport === */

#ifndef CONFIG_MCUMGR_TRANSPORT_NETBUF_COUNT
#define CONFIG_MCUMGR_TRANSPORT_NETBUF_COUNT      4
#endif

#ifndef CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE
#define CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE        2475
#endif

#ifndef CONFIG_MCUMGR_TRANSPORT_BM_UART_RX_BUF_COUNT
#define CONFIG_MCUMGR_TRANSPORT_BM_UART_RX_BUF_COUNT  8
#endif

#ifndef CONFIG_MCUMGR_TRANSPORT_BM_UART_RX_BUF_SIZE
#define CONFIG_MCUMGR_TRANSPORT_BM_UART_RX_BUF_SIZE   128
#endif

#ifndef CONFIG_MCUMGR_TRANSPORT_BM_UART_UARTE_IRQ_PRIO
#define CONFIG_MCUMGR_TRANSPORT_BM_UART_UARTE_IRQ_PRIO  3
#endif


/* === NFC === */

#ifndef CONFIG_BM_NFCT_IRQ_PRIORITY
#define CONFIG_BM_NFCT_IRQ_PRIORITY               3
#endif

#ifndef CONFIG_BM_NFC_NDEF_CH_MAJOR_VERSION
#define CONFIG_BM_NFC_NDEF_CH_MAJOR_VERSION       1
#endif

#ifndef CONFIG_BM_NFC_NDEF_CH_MINOR_VERSION
#define CONFIG_BM_NFC_NDEF_CH_MINOR_VERSION       5
#endif

/* === SOC series detection === */
/* Auto-detect from the chip define passed by the build system */
#ifndef CONFIG_SOC_SERIES_NRF54L
#if defined(NRF54L15_XXAA) || defined(NRF54L10_XXAA) || defined(NRF54L05_XXAA)
#define CONFIG_SOC_SERIES_NRF54L  1
#else
#define CONFIG_SOC_SERIES_NRF54L  0
#endif
#endif

/* === SoftDevice base address in RRAM ===
 *
 * Replaces Zephyr's FIXED_PARTITION_OFFSET(softdevice_partition).
 * Must match the reg property in the devicetree / linker script.
 *
 * S145 on nRF54L15: 0x158C00  (partition@158c00, 144 K)
 * S115 on nRF54L05: 0x62000   (partition@62000)
 */
#ifndef CONFIG_SOFTDEVICE_BASE_ADDRESS
#if defined(NRF54L15_XXAA)
#define CONFIG_SOFTDEVICE_BASE_ADDRESS  0x158C00
#elif defined(NRF54L05_XXAA)
#define CONFIG_SOFTDEVICE_BASE_ADDRESS  0x62000
#else
#define CONFIG_SOFTDEVICE_BASE_ADDRESS  0
#endif
#endif

/* Compatibility shim for Zephyr devicetree macro used in irq_connect.c */
#ifndef FIXED_PARTITION_OFFSET
#define FIXED_PARTITION_OFFSET(label)   CONFIG_SOFTDEVICE_BASE_ADDRESS
#endif


#endif /* BM_CONFIG_DEFAULTS_H__ */
