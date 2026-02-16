/*
 * IOsonata SoftDevice Handler configuration
 *
 * Override these in your project's board config header before
 * including nrf_sdh.h, or via compiler -D flags.
 */

#ifndef NRF_SDH_CONFIG_H__
#define NRF_SDH_CONFIG_H__

/*--- Clock configuration ---*/

/* LF clock source: 0 = RC, 1 = LFXO */
#ifndef CONFIG_NRF_SDH_CLOCK_LF_SRC
#define CONFIG_NRF_SDH_CLOCK_LF_SRC  1
#endif

/* RC calibration timer interval (1/4 sec units). 0 for LFXO. */
#ifndef CONFIG_NRF_SDH_CLOCK_LF_RC_CTIV
#define CONFIG_NRF_SDH_CLOCK_LF_RC_CTIV  0
#endif

/* RC temp calibration interval. 0 for LFXO. */
#ifndef CONFIG_NRF_SDH_CLOCK_LF_RC_TEMP_CTIV
#define CONFIG_NRF_SDH_CLOCK_LF_RC_TEMP_CTIV  0
#endif

/*
 * LF clock accuracy:
 *  0 = 250 ppm, 1 = 500 ppm, 2 = 150 ppm, 3 = 100 ppm,
 *  4 = 75 ppm,  5 = 50 ppm,  6 = 30 ppm,  7 = 20 ppm,
 *  8 = 10 ppm,  9 = 5 ppm,  10 = 2 ppm,  11 = 1 ppm
 */
#ifndef CONFIG_NRF_SDH_CLOCK_LF_ACCURACY
#define CONFIG_NRF_SDH_CLOCK_LF_ACCURACY  0
#endif

/* HFCLK ramp-up latency in microseconds */
#ifndef CONFIG_NRF_SDH_CLOCK_HFCLK_LATENCY
#define CONFIG_NRF_SDH_CLOCK_HFCLK_LATENCY  1500
#endif

/* HFINT calibration interval in seconds */
#ifndef CONFIG_NRF_SDH_CLOCK_HFINT_CALIBRATION_INTERVAL
#define CONFIG_NRF_SDH_CLOCK_HFINT_CALIBRATION_INTERVAL  60
#endif

/*--- Dispatch model ---*/
/*
 * How SoftDevice events reach the application:
 *  0 = IRQ    : dispatched directly in SD_EVT_IRQHandler
 *  1 = SCHED  : deferred via bm_scheduler to main loop
 *  2 = POLL   : application calls nrf_sdh_evts_poll() manually
 */
#ifndef CONFIG_NRF_SDH_DISPATCH_MODEL
#define CONFIG_NRF_SDH_DISPATCH_MODEL  1
#endif

#define NRF_SDH_DISPATCH_MODEL_IRQ    0
#define NRF_SDH_DISPATCH_MODEL_SCHED  1
#define NRF_SDH_DISPATCH_MODEL_POLL   2

/*--- BLE configuration ---*/

/* Connection configuration tag */
#ifndef CONFIG_NRF_SDH_BLE_CONN_TAG
#define CONFIG_NRF_SDH_BLE_CONN_TAG  99
#endif

/* Maximum number of peripheral links (0 if central-only) */
#ifndef CONFIG_NRF_SDH_BLE_PERIPHERAL_LINK_COUNT
#define CONFIG_NRF_SDH_BLE_PERIPHERAL_LINK_COUNT  1
#endif

/* Maximum number of central links (0 if peripheral-only) */
#ifndef CONFIG_NRF_SDH_BLE_CENTRAL_LINK_COUNT
#define CONFIG_NRF_SDH_BLE_CENTRAL_LINK_COUNT  0
#endif

/*
 * Maximum total concurrent connections.
 * S115: max 2, S145: max 5
 */
#ifndef CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT
#define CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT  1
#endif

/* GAP event length in 1.25 ms units. Must be >= 6 for coded PHY. */
#ifndef CONFIG_NRF_SDH_BLE_GAP_EVENT_LENGTH
#define CONFIG_NRF_SDH_BLE_GAP_EVENT_LENGTH  3
#endif

/* ATT maximum MTU size (23 = default, no DLE) */
#ifndef CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE
#define CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE  23
#endif

/* GATT attribute table size in bytes (must be multiple of 4) */
#ifndef CONFIG_NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE
#define CONFIG_NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE  1408
#endif

/* Number of vendor-specific (128-bit) UUIDs */
#ifndef CONFIG_NRF_SDH_BLE_VS_UUID_COUNT
#define CONFIG_NRF_SDH_BLE_VS_UUID_COUNT  0
#endif

/* Include Service Changed characteristic in attribute table: 0 or 1 */
#ifndef CONFIG_NRF_SDH_BLE_SERVICE_CHANGED
#define CONFIG_NRF_SDH_BLE_SERVICE_CHANGED  0
#endif

/*--- SoC configuration ---*/

/* Automatically seed SoftDevice RNG via CRACEN TRNG: 0 or 1 */
#ifndef CONFIG_NRF_SDH_SOC_RAND_SEED
#define CONFIG_NRF_SDH_SOC_RAND_SEED  1
#endif

/*--- Misc ---*/

/* Build string tables for event-to-string conversion: 0 or 1 */
#ifndef CONFIG_NRF_SDH_STR_TABLES
#define CONFIG_NRF_SDH_STR_TABLES  1
#endif

/* Scheduler buffer size in bytes (only used if dispatch model = SCHED) */
#ifndef CONFIG_BM_SCHEDULER_BUF_SIZE
#define CONFIG_BM_SCHEDULER_BUF_SIZE  1024
#endif

#endif /* NRF_SDH_CONFIG_H__ */
