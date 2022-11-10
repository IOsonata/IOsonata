/*--------------------------------------------------------------------------
@file	ble_app_nrf52.cpp

@brief	Nordic SDK based BLE peripheral application creation helper


@author	Hoang Nguyen Hoan
@date	Dec 26, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <atomic>

#include "sdk_config.h"
#include "nordic_common.h"
#include "ble_hci.h"
#include "nrf_error.h"
#include "ble_gatt.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "nrf_ble_gatt.h"
#include "peer_manager.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_dfu_settings.h"
#include "nrf_bootloader_info.h"

#ifndef __ARMCC_VERSION
#include "nrf_crypto.h"
#endif

#include "nrf_ble_lesc.h"
#include "nrf_ble_scan.h"
#include "nrf_drv_rng.h"

#include "istddef.h"
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/ble_app.h"
#include "bluetooth/ble_appearance.h"
#include "ble_app_nrf5.h"
#include "ble_dev.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/ble_srvc.h"
#include "app_evt_handler.h"

extern "C" void nrf_sdh_soc_evts_poll(void * p_context);
extern "C" ret_code_t nrf_sdh_enable(nrf_clock_lf_cfg_t *clock_lf_cfg);

#define BLEAPP_CONN_CFG_TAG            1     /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define BLEAPP_OBSERVER_PRIO           1                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
//#define BLEAPP_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_TIMER_OP_QUEUE_SIZE         10                                           /**< Size of timer operation queues. */

#define SCHED_MAX_EVENT_DATA_SIZE 		20 /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE          		40                        /**< Maximum number of events in the scheduler queue. */
#endif

#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */

#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define MAX_ADV_UUID					10

// ORable application role
//#define BLEAPP_ROLE_PERIPHERAL			1
//#define BLEAPP_ROLE_CENTRAL				2

// These are to be passed as parameters
#define SCAN_INTERVAL           MSEC_TO_UNITS(100, UNIT_0_625_MS)      /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             MSEC_TO_UNITS(100, UNIT_0_625_MS)		/**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0                                 		/**< Timout when scanning. 0x0000 disables timeout. */

#pragma pack(push, 4)

typedef struct _BleAppData {
	BLEAPP_STATE State;
	BLEAPP_ROLE Role;
	uint8_t AdvHdl;		// Advertisement handle
	uint16_t ConnHdl;	// BLE connection handle
	int ConnLedPort;
	int ConnLedPin;
	uint8_t ConnLedActLevel;
	uint16_t VendorId;
	ble_gap_adv_params_t AdvParam;
	//ble_gap_adv_data_t AdvData;
	int PeriphDevCnt;
//	BLEAPP_PERIPH *pPeriphDev;
	uint32_t (*SDEvtHandler)(void) ;
	//ble_advdata_t AdvData;
	//ble_advdata_t SrData;
    //ble_advdata_manuf_data_t ManufData;
    //ble_advdata_manuf_data_t SRManufData;
	int MaxMtu;
	bool bSecure;
	bool bAdvertising;
	bool bExtAdv;
	bool bScan;
   // bool BleInitialized;
} BleAppData_t;

#pragma pack(pop)

// S132 tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm
// S140 tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.

static const int8_t s_TxPowerdBm[] = {
	-40, -20, -16, -12, -8, -4, 0,
#ifdef S132
	3, 4
#else
	2, 3, 4, 5, 6, 7, 8
#endif
};

static const int s_NbTxPowerdBm = sizeof(s_TxPowerdBm) / sizeof(int8_t);

//BLE_ADVERTISING_DEF(g_AdvInstance);             /**< Advertising module instance. */
NRF_BLE_GATT_DEF(s_Gatt);

BleAppData_t g_BleAppData = {
		BLEAPP_STATE_UNKNOWN, BLEAPP_ROLE_PERIPHERAL, BLE_GAP_ADV_SET_HANDLE_NOT_SET, BLE_CONN_HANDLE_INVALID, -1, -1,
};

//static volatile bool s_BleStarted = false;

alignas(4) static uint8_t s_BleAppAdvBuff[256];
alignas(4) static BleAdvPacket_t s_BleAppAdvPkt = { 31, 0, s_BleAppAdvBuff};
alignas(4) static BleAdvPacket_t s_BleAppExtAdvPkt = { 255, 0, s_BleAppAdvBuff};

alignas(4) static uint8_t s_BleAppSrBuff[256];
alignas(4) static BleAdvPacket_t s_BleAppSrPkt = { 31, 0, s_BleAppSrBuff};
alignas(4) static BleAdvPacket_t s_BleAppExtSrPkt = { 255, 0, s_BleAppSrBuff};

static ble_gap_adv_data_t s_BleAppAdvData = {
	.adv_data = {s_BleAppAdvBuff, 0},
	.scan_rsp_data = {s_BleAppSrBuff, 0}
};

pm_peer_id_t g_PeerMngrIdToDelete = PM_PEER_ID_INVALID;
//static nrf_ble_gatt_t s_Gatt;                                     /**< GATT module instance. */
#if 0
ble_db_discovery_t s_DbDiscovery = {
	.services = {},
	.srv_count = 0,
	.curr_char_ind = 0,
	.curr_srv_ind = 0,
	.discovery_in_progress = 0,
	.discovery_pending = 0,
	.discoveries_count = 0,
	.conn_handle = BLE_CONN_HANDLE_INVALID
};

NRF_SDH_BLE_OBSERVER(s_DbDiscovery_obs,
                     BLE_DB_DISC_BLE_OBSERVER_PRIO,
                     ble_db_discovery_on_ble_evt, &s_DbDiscovery);
#endif

NRF_BLE_SCAN_DEF(g_Scan);

static ble_gap_scan_params_t s_BleScanParams =
{
#if (NRF_SD_BLE_API_VERSION >= 6)
	1,
	0,
#endif
    1,		// Active scan
#if (NRF_SD_BLE_API_VERSION <= 2)
	0,	// .selective
	NULL,	// .p_whitelist
#endif
#if (NRF_SD_BLE_API_VERSION >= 3)
	0,				// Use whitelist
	1, 				// Report directed advertisement
#endif
	SCAN_INTERVAL,	// Scan interval
	SCAN_WINDOW,	// Scan window
	SCAN_TIMEOUT,	// Scan timeout
};

static uint8_t g_BleScanBuff[BLE_GAP_SCAN_BUFFER_EXTENDED_MAX];

static ble_data_t g_BleScanReportData = {
	.p_data = g_BleScanBuff,
	.len = BLE_GAP_SCAN_BUFFER_EXTENDED_MAX
};

//#endif

/**@brief Bluetooth SIG debug mode Private Key */
__ALIGN(4) __WEAK extern const uint8_t g_lesc_private_key[32] = {
    0xbd,0x1a,0x3c,0xcd,0xa6,0xb8,0x99,0x58,0x99,0xb7,0x40,0xeb,0x7b,0x60,0xff,0x4a,
    0x50,0x3f,0x10,0xd2,0xe3,0xb3,0xc9,0x74,0x38,0x5f,0xc5,0xa3,0xd4,0xf6,0x49,0x3f,
};

__ALIGN(4) static ble_gap_lesc_p256_pk_t    s_lesc_public_key;      /**< LESC ECC Public Key */
__ALIGN(4) static ble_gap_lesc_dhkey_t      s_lesc_dh_key;          /**< LESC ECC DH Key*/
static ble_gap_conn_sec_mode_t s_gap_conn_mode;

bool isConnected()
{
	return g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID;
}

static void BleConnLedOff() {
	if (g_BleAppData.ConnLedPort < 0 || g_BleAppData.ConnLedPin < 0)
		return;

	if (g_BleAppData.ConnLedActLevel)
	{
	    IOPinClear(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
	}
	else
	{
	    IOPinSet(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
	}
}

static void BleConnLedOn() {
	if (g_BleAppData.ConnLedPort < 0 || g_BleAppData.ConnLedPin < 0)
		return;

    if (g_BleAppData.ConnLedActLevel)
    {
        IOPinSet(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
    }
    else
    {
        IOPinClear(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
    }
}

void BleAppEnterDfu()
{
    // SDK14 use this
    uint32_t err_code = sd_power_gpregret_clr(0, 0xffffffff);
    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    NVIC_SystemReset();
#if 0

    uint32_t err_code = nrf_dfu_flash_init(true);

    nrf_dfu_settings_init(true);

    s_dfu_settings.enter_buttonless_dfu = true;

    err_code = nrf_dfu_settings_write(BleAppDfuCallback);
    if (err_code != NRF_SUCCESS)
    {
    }
#endif
}

bool BleAppNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (g_BleAppData.ConnHdl == BT_GATT_HANDLE_INVALID)
	{
		return false;
	}

	if (pChar->bNotify == false)
		return false;

    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = pChar->ValHdl;//.value_handle;
    params.p_data = pData;
    params.p_len = &DataLen;

    uint32_t err_code = sd_ble_gatts_hvx(g_BleAppData.ConnHdl, &params);

    return true;
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void BleAppDisconnect()
{
	if (g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
    {
		uint32_t err_code = sd_ble_gap_disconnect(g_BleAppData.ConnHdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code == NRF_ERROR_INVALID_STATE)
        {
            g_BleAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

void BleAppGapDeviceNameSet(const char* pDeviceName)
{
    uint32_t                err_code;

    err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
                                          (const uint8_t *)pDeviceName,
                                          strlen( pDeviceName ));
    APP_ERROR_CHECK(err_code);
    //ble_advertising_restart_without_whitelist(&g_AdvInstance);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */

static void BleAppGapParamInit(const BleAppCfg_t *pBleAppCfg)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;

    switch (pBleAppCfg->SecType)
    {
    	case BLEAPP_SECTYPE_NONE:
    	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_STATICKEY_NO_MITM:
		    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_STATICKEY_MITM:
		    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_LESC_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_SIGNED_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_SIGNED_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(&s_gap_conn_mode);
    	    break;
    }
/*
    if (pBleAppCfg->pDevName != NULL)
    {
    	err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
                                          (const uint8_t *) pBleAppCfg->pDevName,
                                          strlen(pBleAppCfg->pDevName));
    	APP_ERROR_CHECK(err_code);
    }

    err_code = sd_ble_gap_appearance_set(pBleAppCfg->Appearance);
    APP_ERROR_CHECK(err_code);
*/
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

//    if (pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT)
//    if (pBleAppCfg->AdvType != BLEADV_TYPE_ADV_NONCONN_IND)
    if (pBleAppCfg->Role & BLEAPP_ROLE_PERIPHERAL)
    {
		gap_conn_params.min_conn_interval = MSEC_TO_UNITS(pBleAppCfg->ConnIntervalMin, UNIT_1_25_MS);// MIN_CONN_INTERVAL;
		gap_conn_params.max_conn_interval = MSEC_TO_UNITS(pBleAppCfg->ConnIntervalMax, UNIT_1_25_MS);//MAX_CONN_INTERVAL;
		gap_conn_params.slave_latency     = SLAVE_LATENCY;
		gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

		err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
		APP_ERROR_CHECK(err_code);
    }

   // BtGapInit(pBleAppCfg->Role);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(g_BleAppData.ConnHdl, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        //APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
        {
/*        	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
        	{
//        		uint32_t ret = ble_advdata_set(&(g_AdvInstance.advdata), &g_BleAppData.SRData);
        		uint32_t err_code = sd_ble_gap_adv_start(&s_AdvParams, BLEAPP_CONN_CFG_TAG);
        	}
        	else
        	   ret_code_t err_code = ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_SLOW);*/
          //  APP_ERROR_CHECK(err_code);
        }
           // sleep_mode_enter();
            break;
        default:
            break;
    }
}

static void on_adv_error(uint32_t)
{

}

/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t const * p_ble_evt)
{
    uint32_t err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    uint16_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	g_BleAppData.bAdvertising = false;

        	BtGapAddConnection(p_gap_evt->conn_handle, role,
        					   p_gap_evt->params.connected.peer_addr.addr_type,
        					   (uint8_t*)p_gap_evt->params.connected.peer_addr.addr);

        	BleConnLedOn();
        	g_BleAppData.ConnHdl = p_ble_evt->evt.gap_evt.conn_handle;
        	g_BleAppData.bScan = false;
        	g_BleAppData.State = BLEAPP_STATE_CONNECTED;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        	BleConnLedOff();
        	BtGapDeleteConnection(p_gap_evt->conn_handle);
        	g_BleAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;
        	g_BleAppData.State = BLEAPP_STATE_IDLE;
        	BleAppAdvStart();

        	break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            {
            }
            break;
        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            //char passkey[8 + 1];
            //memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, 8);
            //passkey[8] = 0;
            // Don't send delayed Security Request if security procedure is already in progress.
           // err_code = app_timer_stop(m_sec_req_timer_id);
           // APP_ERROR_CHECK(err_code);

        	//g_Uart.printf("Passkey: %s\r\n", passkey);
        } break; // BLE_GAP_EVT_PASSKEY_DISPLAY

        //*****
        // This section is required for compatibility with iPhone X series and other devices that has Bluetooth 5 support
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
//            printf("PHY update request.\r\n");
            ble_gap_phys_t const phys =
            {
                /*.tx_phys =*/ BLE_GAP_PHY_AUTO,
                /*.rx_phys =*/ BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE:
        	if (p_ble_evt->evt.gap_evt.params.phy_update.status == 0)
        	{
        		//printf("%x %x\n", p_ble_evt->evt.gap_evt.params.phy_update.rx_phy, p_ble_evt->evt.gap_evt.params.phy_update.tx_phy);
        	}
        	break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
       {
           ble_gap_data_length_params_t dl_params;

//           printf("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST\r\n");

           // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
           memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
           err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
           APP_ERROR_CHECK(err_code);
       } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
            uint16_t max_con_int = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
            uint16_t min_con_int = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;

           // m_ble_params_info.con_interval = max_con_int;
            //ble_its_ble_params_info_send(&m_its, &m_ble_params_info);
//            printf("Con params updated: CI %i, %i\r\n", (int)min_con_int, (int)max_con_int);
        } break;
        //*****

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(g_BleAppData.ConnHdl, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            if (  p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT
                ||p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_LIMIT_REACHED)
            {
            	// Latest SDK restart advertising automatically.  Therefore this flag should not change
            	g_BleAppData.bAdvertising = false;
                BleAppAdvTimeoutHandler();
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
#if 0
        	// The SDK no longer set this event
        	if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT)//BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
            	g_BleAppData.bAdvertising = false;
            	BleAppAdvTimeoutHandler();
            	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
				{
					//err_code = ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_FAST);
            			//APP_ERROR_CHECK(err_code);
				}
            }
#endif
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
            	g_BleAppData.bScan = false;
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

  //      case BLE_EVT_USER_MEM_REQUEST:
  //          err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
  //          APP_ERROR_CHECK(err_code);
  //          break; // BLE_EVT_USER_MEM_REQUEST

/*        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
//                    printf("BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST\r\n");
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST
        */
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
//            NRF_LOG_DEBUG("BLE_GAP_EVT_AUTH_KEY_REQUEST");

            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        //	 printf("BLE_GAP_EVT_LESC_DHKEY_REQUEST %x\r\n", role);
        //    NRF_LOG_INFO("%s: BLE_GAP_EVT_LESC_DHKEY_REQUEST\r\n", nrf_log_push(roles_str[role]));
#if NRF_SD_BLE_API_VERSION <= 3
            static nrf_crypto_key_t peer_pk;
            peer_pk.p_le_data = &p_ble_evt->evt.gap_evt.params.lesc_dhkey_request.p_pk_peer->pk[0];
            peer_pk.len = BLE_GAP_LESC_P256_PK_LEN;
            err_code = nrf_crypto_shared_secret_compute(NRF_CRYPTO_CURVE_SECP256R1, &m_crypto_key_sk, &peer_pk, &m_crypto_key_dhkey);
            APP_ERROR_CHECK(err_code);
#else

#if 0
            static nrf_value_length_t peer_public_key_raw = {0};

            peer_public_key_raw.p_value = &p_ble_evt->evt.gap_evt.params.lesc_dhkey_request.p_pk_peer->pk[0];
            peer_public_key_raw.length = BLE_GAP_LESC_P256_PK_LEN;

            err_code = nrf_crypto_ecc_public_key_from_raw(NRF_CRYPTO_BLE_ECDH_CURVE_INFO,
                                                          &peer_public_key_raw,
                                                          &s_peer_public_key);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_crypto_ecdh_shared_secret_compute(NRF_CRYPTO_BLE_ECDH_CURVE_INFO,
                                                            &s_private_key,
                                                            &s_peer_public_key,
                                                            &s_dh_key);
            APP_ERROR_CHECK(err_code);
#endif
#endif
            err_code = sd_ble_gap_lesc_dhkey_reply(g_BleAppData.ConnHdl, &s_lesc_dh_key);
            APP_ERROR_CHECK(err_code);
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             if (p_ble_evt->evt.gap_evt.params.auth_status.auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
             {
                 //printf("Authorization succeeded!");
             }
             else
             {
                 //printf("Authorization failed with code: %u!",
                   //           p_ble_evt->evt.gap_evt.params.auth_status.auth_status);
             }
/*             printf("%x : BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x\r\n",
                          role,
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));*/
            break;

#if (NRF_SD_BLE_API_VERSION >= 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
//           printf("%x:BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST %d\r\n", p_ble_evt->header.evt_id, g_BleAppData.MaxMtu);
        	if (role == BLE_GAP_ROLE_CENTRAL)
        		break;
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
            										   g_BleAppData.MaxMtu);
            										   //NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

#if (NRF_SD_BLE_API_VERSION > 3)
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            break;
#else
        case BLE_EVT_TX_COMPLETE:
            break;

#endif


        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;
    uint16_t role = ble_conn_state_role(p_evt->conn_handle);

    switch (p_evt->evt_id)
    {
		case PM_EVT_BONDED_PEER_CONNECTED:
			{
				// Start Security Request timer.
				//err_code = app_timer_start(g_SecReqTimerId, SECURITY_REQUEST_DELAY, NULL);
				//APP_ERROR_CHECK(err_code);
			}
			break;

			case PM_EVT_CONN_SEC_SUCCEEDED:
			{
				pm_conn_sec_status_t conn_sec_status;

				// Check if the link is authenticated (meaning at least MITM).
				err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
				APP_ERROR_CHECK(err_code);

				if (conn_sec_status.mitm_protected)
				{
				}
				else
				{
					// The peer did not use MITM, disconnect.
					err_code = pm_peer_id_get(g_BleAppData.ConnHdl, &g_PeerMngrIdToDelete);
					APP_ERROR_CHECK(err_code);
					err_code = sd_ble_gap_disconnect(g_BleAppData.ConnHdl,
													 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					APP_ERROR_CHECK(err_code);
				}
			}
			break;

        case PM_EVT_CONN_SEC_FAILED:
            if (g_BleAppData.bSecure && g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(g_BleAppData.ConnHdl,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
			{
				// Accept pairing request from an already bonded peer.
				pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
				pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
			}
			break;

        case PM_EVT_STORAGE_FULL:
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }

            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:

            //ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_FAST);
            err_code = sd_ble_gap_adv_start(g_BleAppData.AdvHdl, BLEAPP_CONN_CFG_TAG);

            break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();

            break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        	break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t const * p_ble_evt, void *p_context)
{
    uint16_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);

    on_ble_evt(p_ble_evt);
    if ((role == BLE_GAP_ROLE_CENTRAL) || g_BleAppData.Role & (BLEAPP_ROLE_CENTRAL | BLEAPP_ROLE_OBSERVER))
    {
#if 0
    	switch (p_ble_evt->header.evt_id)
        {
            case BLE_GAP_EVT_TIMEOUT:
            {
                const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

                ble_gap_evt_timeout_t const * p_timeout = &p_gap_evt->params.timeout;

                if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
                {
                    g_BleAppData.bScan = false;
                }
            }
            break;
        }
#endif
        BleCentralEvtUserHandler((ble_evt_t *)p_ble_evt);
    }
    if (g_BleAppData.Role & BLEAPP_ROLE_PERIPHERAL)
    {
        BlePeriphEvtUserHandler((ble_evt_t *)p_ble_evt);
    }
}

/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void BleAppPeerMngrInit(BLEAPP_SECTYPE SecType, uint8_t SecKeyExchg, bool bEraseBond)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (bEraseBond)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    sec_param.bond = 1;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    switch (SecType)
    {
    	case BLEAPP_SECTYPE_NONE:
			break;
		case BLEAPP_SECTYPE_STATICKEY_NO_MITM:
			break;
		case BLEAPP_SECTYPE_STATICKEY_MITM:
			sec_param.mitm = 1;
			break;
		case BLEAPP_SECTYPE_LESC_MITM:
		case BLEAPP_SECTYPE_SIGNED_MITM:
			sec_param.mitm = 1;
		    sec_param.lesc = 1;
			break;
		case BLEAPP_SECTYPE_SIGNED_NO_MITM:
		    sec_param.lesc = 1;
			break;
    }

/*    if (SecType == BLEAPP_SECTYPE_STATICKEY_MITM ||
    	SecType == BLEAPP_SECTYPE_LESC_MITM ||
		SecType == BLEAPP_SECTYPE_SIGNED_MITM)
    {
    	sec_param.mitm = 1;
    }
*/
    int type = SecKeyExchg & (BLEAPP_SECEXCHG_KEYBOARD | BLEAPP_SECEXCHG_DISPLAY);
    switch (type)
    {
		case BLEAPP_SECEXCHG_KEYBOARD:
			sec_param.keypress = 1;
			sec_param.io_caps  = BLE_GAP_IO_CAPS_KEYBOARD_ONLY;
			break;

		case BLEAPP_SECEXCHG_DISPLAY:
			sec_param.io_caps  = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
    			break;
		case (BLEAPP_SECEXCHG_KEYBOARD | BLEAPP_SECEXCHG_DISPLAY):
			sec_param.keypress = 1;
			sec_param.io_caps  = BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY;
			break;
    }

    if (SecKeyExchg & BLEAPP_SECEXCHG_OOB)
    {
    	sec_param.oob = 1;
//    	nfc_ble_pair_init(&g_AdvInstance, NFC_PAIRING_MODE_JUST_WORKS);
    }

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    // Generate the ECDH key pair and set public key in the peer-manager.
    //err_code = ble_lesc_ecc_keypair_generate_and_set();
    //APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Security Request timer timeout.
 *
 * @details This function will be called each time the Security Request timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    uint32_t err_code;

    if (g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
    {
        // Initiate bonding.
        //NRF_LOG_DEBUG("Start encryption\r\n");
        err_code = pm_conn_secure(g_BleAppData.ConnHdl, false);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

bool BleAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
#if 0
	uint32_t err;

	if (pAdvData && AdvLen > 0)
	{
		int l = min(AdvLen, g_BleAppData.bExtAdv ? BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED : BLE_GAP_ADV_SET_DATA_SIZE_MAX);

		memcpy(g_BleAppData.ManufData.data.p_data, pAdvData, l);

		g_BleAppData.ManufData.data.size = l;
        g_AdvInstance.adv_data.adv_data.len = g_BleAppData.bExtAdv ? BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED : BLE_GAP_ADV_SET_DATA_SIZE_MAX;

        err = ble_advdata_encode(&g_BleAppData.AdvData, g_AdvInstance.adv_data.adv_data.p_data, &g_AdvInstance.adv_data.adv_data.len);
		APP_ERROR_CHECK(err);

	}

	if (pSrData && SrLen > 0)
	{
		int l = min(SrLen, g_BleAppData.bExtAdv ? BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED : BLE_GAP_ADV_SET_DATA_SIZE_MAX);

		memcpy(g_BleAppData.SRManufData.data.p_data, pSrData, l);

		g_BleAppData.SRManufData.data.size = l;
		g_AdvInstance.adv_data.scan_rsp_data.len = g_BleAppData.bExtAdv ? BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED : BLE_GAP_ADV_SET_DATA_SIZE_MAX;

		uint32_t err = ble_advdata_encode(&g_BleAppData.SrData, g_AdvInstance.adv_data.scan_rsp_data.p_data,
										 &g_AdvInstance.adv_data.scan_rsp_data.len);
		APP_ERROR_CHECK(err);
	}
#else
	if (g_BleAppData.State != BLEAPP_STATE_ADVERTISING)
	{
		return false;
	}

	BleAdvPacket_t *advpkt;
	BleAdvPacket_t *srpkt;

	if (g_BleAppData.bExtAdv == true)
	{
		advpkt = &s_BleAppExtAdvPkt;
		srpkt = &s_BleAppExtSrPkt;
	}
	else
	{
		advpkt = &s_BleAppAdvPkt;
		srpkt = &s_BleAppSrPkt;
	}

	if (pAdvData)
	{
		int l = AdvLen + 2;
		BleAdvData_t *p = BleAdvDataAllocate(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = g_BleAppData.VendorId;
		memcpy(&p->Data[2], pAdvData, AdvLen);

		s_BleAppAdvData.adv_data.len = advpkt->Len;
	}

	if (pSrData)
	{
		int l = SrLen + 2;
		BleAdvData_t *p = BleAdvDataAllocate(srpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = g_BleAppData.VendorId;
		memcpy(&p->Data[2], pAdvData, AdvLen);

		s_BleAppAdvData.scan_rsp_data.len = srpkt->Len;
	}
#endif

	// SDK15 doesn't allow dynamically updating adv data.  Have to stop and re-start advertising
	if (g_BleAppData.bAdvertising == true)
	{
		sd_ble_gap_adv_stop(g_BleAppData.AdvHdl);
	}

	uint32_t err = sd_ble_gap_adv_set_configure(&g_BleAppData.AdvHdl, &s_BleAppAdvData, NULL);
//	APP_ERROR_CHECK(err);

	if (g_BleAppData.bAdvertising == true)
	{
		g_BleAppData.bAdvertising = false;
		BleAppAdvStart();//BLEAPP_ADVMODE_FAST);
	}

#if 0
    int l = min(Len, BLE_GAP_ADV_MAX_SIZE);

    memcpy(g_AdvInstance.manuf_data_array, pData, l);
    uint32_t ret = ble_advdata_set(&(g_AdvInstance.advdata), &g_BleAppData.SRData);
#endif

    return g_BleAppData.bAdvertising;
}

void BleAppAdvStart()//BLEAPP_ADVMODE AdvMode)
{
	if (g_BleAppData.bAdvertising == true || g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
		return;

	g_BleAppData.bAdvertising = true;
#if 1
	uint32_t err_code = sd_ble_gap_adv_start(g_BleAppData.AdvHdl, BLEAPP_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

#else
//	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
	//if (g_BleAppData.AdvType == BLEADV_TYPE_ADV_NONCONN_IND)
	if (g_BleAppData.Role & BLEAPP_ROLE_BROADCASTER)
	{
		uint32_t err_code = sd_ble_gap_adv_start(g_AdvInstance.adv_handle, g_AdvInstance.conn_cfg_tag);
        APP_ERROR_CHECK(err_code);
	}
	else
	{
		uint32_t err_code = ble_advertising_start(&g_AdvInstance, (ble_adv_mode_t)BLEAPP_ADVMODE_FAST);//AdvMode);
		if (err_code != NRF_SUCCESS)
		{
			 APP_ERROR_CHECK(err_code);
		}
	}
#endif

	g_BleAppData.State = BLEAPP_STATE_ADVERTISING;
}

void BleAppAdvStop()
{
	sd_ble_gap_adv_stop(g_BleAppData.AdvHdl);
	g_BleAppData.bAdvertising = false;
}

/**@brief Overloadable function for initializing the Advertising functionality.
 */
__WEAK bool BleAppAdvInit(const BleAppCfg_t *pCfg)
{
    uint32_t               err_code;
    ble_advertising_init_t	initdata;
    uint8_t flags = BT_GAP_DATA_TYPE_FLAGS_NO_BREDR;
	BleAdvPacket_t *advpkt;
	BleAdvPacket_t *srpkt;
	uint8_t proptype = 0;

	memset(&g_BleAppData.AdvParam, 0, sizeof(ble_gap_adv_params_t));

    //memset(&initdata, 0, sizeof(ble_advertising_init_t));
    //memset(&g_AdvInstance, 0, sizeof(ble_advertising_t));

#if 1

	if (g_BleAppData.bExtAdv == true)
	{
		advpkt = &s_BleAppExtAdvPkt;
		srpkt = &s_BleAppExtSrPkt;
	}
	else
	{
		advpkt = &s_BleAppAdvPkt;
		srpkt = &s_BleAppSrPkt;
	}

	if (pCfg->Role & BLEAPP_ROLE_PERIPHERAL)
	{
		if (pCfg->AdvTimeout != 0)
		{
			flags |= BT_GAP_DATA_TYPE_FLAGS_LIMITED_DISCOVERABLE;
		}
		else
		{
			flags |= BT_GAP_DATA_TYPE_FLAGS_GENERAL_DISCOVERABLE;
		}
		g_BleAppData.AdvParam.properties.type = pCfg->bExtAdv ?
												BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED :
												BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
	}
	else if (pCfg->Role & BLEAPP_ROLE_BROADCASTER)
	{
		g_BleAppData.AdvParam.properties.type = pCfg->bExtAdv ?
												BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED :
												BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
	}

	if (BleAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_FLAGS, &flags, 1) == false)
	{
		return false;
	}

    if (pCfg->Appearance != BLE_APPEAR_UNKNOWN_GENERIC)
    {
    	if (BleAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_APPEARANCE, (uint8_t*)&pCfg->Appearance, 2) == false)
    	{
    		// Don't care whether we are able to add appearance or not. Appearance is optional.
    		//return false;
    	}
    }

    BleAdvPacket_t *uidadvpkt;

    if (pCfg->pDevName != NULL)
    {
    	size_t l = strlen(pCfg->pDevName);
    	uint8_t type = BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME;
    	size_t mxl = advpkt->MaxLen - advpkt->Len - 2;

    	if (l > 30 || l > mxl)
    	{
    		// Short name
    		type = BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME;
    		l = min((size_t)30, mxl);
    	}

    	if (BleAdvDataAdd(advpkt, type, (uint8_t*)pCfg->pDevName, l) == false)
    	{
    		return false;
    	}

    	uidadvpkt = pCfg->bExtAdv ? advpkt : srpkt;
    }
    else
    {
    	uidadvpkt = advpkt;
    }

    if (pCfg->pAdvUuid != NULL && pCfg->Role & BLEAPP_ROLE_PERIPHERAL)
    {
    	/*
    	if (pCfg->pAdvUuid->BaseIdx > 0 && pCfg->pAdvUuid->Type == BT_UUID_TYPE_16)
    	{

    		// Convert custom uuid to 128 bits
			int l = (pCfg->pAdvUuid->Count - 1) * sizeof(BtUuid_t) + sizeof(BtUuidArr_t);
			uint8_t x[l];

			memcpy(x, pCfg->pAdvUuid, l);
			BtUuidArr_t *ua = (BtUuidArr_t*)x;

			ua->Type = BT_UUID_TYPE_128;


			for (int i = 0; i < pCfg->pAdvUuid->Count; i++)
			{
				ble_uuid_t uid = { pCfg->pAdvUuid->Uuid16[i], (uint8_t)(pCfg->pAdvUuid->BaseIdx == 0 ? BLE_UUID_TYPE_BLE : BLE_UUID_TYPE_VENDOR_BEGIN)};

				uint8_t len = 0;
				uint32_t res = sd_ble_uuid_encode(&uid, &len, ua->Uuid128[i]);
				if (res != 0)
				{
					//printf("err %x\n", res);
				}
			}
			if (BleAdvDataAddUuid(uidadvpkt, ua, pCfg->bCompleteUuidList) == false)
			{

			}
    	}
    	else*/
    	{
			if (BleAdvDataAddUuid(uidadvpkt, pCfg->pAdvUuid, pCfg->bCompleteUuidList) == false)
			{

			}
    	}
    }

	if (pCfg->pAdvManData != NULL)
	{
		int l = pCfg->AdvManDataLen + 2;
		BleAdvData_t *p = BleAdvDataAllocate(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = pCfg->VendorID;
		memcpy(&p->Data[2], pCfg->pAdvManData, pCfg->AdvManDataLen);
	}

	if (pCfg->pSrManData != NULL)
	{
		int l = pCfg->SrManDataLen + 2;
		BleAdvData_t *p = BleAdvDataAllocate(srpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = pCfg->VendorID;
		memcpy(&p->Data[2], pCfg->pSrManData, pCfg->SrManDataLen);
	}
    g_BleAppData.AdvParam.p_peer_addr 	= NULL;	// Undirected advertisement.
    g_BleAppData.AdvParam.interval    	= MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
    g_BleAppData.AdvParam.duration     	= MSEC_TO_UNITS(pCfg->AdvTimeout, UNIT_10_MS);
    //g_BleAppData.AdvParam.channel_mask[5]	= (7 << 5);	// All channels
    g_BleAppData.AdvParam.filter_policy	= BLE_GAP_ADV_FP_ANY;
	g_BleAppData.AdvParam.primary_phy 	= BLE_GAP_PHY_1MBPS;
	g_BleAppData.AdvParam.secondary_phy	= BLE_GAP_PHY_2MBPS;

	s_BleAppAdvData.adv_data.len = advpkt->Len;
	s_BleAppAdvData.scan_rsp_data.len = srpkt->Len;

    err_code = sd_ble_gap_adv_set_configure(&g_BleAppData.AdvHdl, &s_BleAppAdvData, &g_BleAppData.AdvParam);
    APP_ERROR_CHECK(err_code);

#else
//    memset(&g_AdvInstance.adv_params, 0, sizeof(g_AdvInstance.adv_params));
    g_AdvInstance.conn_cfg_tag = BLEAPP_CONN_CFG_TAG;//BLE_CONN_CFG_TAG_DEFAULT;//BLEAPP_CONN_CFG_TAG;
    g_AdvInstance.adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    g_AdvInstance.adv_mode_current = BLE_ADV_MODE_IDLE;
    g_AdvInstance.adv_modes_config.ble_adv_secondary_phy= BLE_GAP_PHY_2MBPS;
    g_AdvInstance.adv_modes_config.ble_adv_primary_phy= BLE_GAP_PHY_1MBPS;
    g_AdvInstance.current_slave_link_conn_handle = BLE_CONN_HANDLE_INVALID;

    g_AdvInstance.adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
    g_AdvInstance.adv_params.secondary_phy = BLE_GAP_PHY_2MBPS;
    g_AdvInstance.evt_handler = on_adv_evt;
    //g_AdvInstance.error_handler = on_adv_error;
    g_AdvInstance.adv_params.p_peer_addr 		= NULL;                             // Undirected advertisement.
    g_AdvInstance.adv_params.filter_policy		= BLE_GAP_ADV_FP_ANY;
    g_AdvInstance.adv_params.interval    		= MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
    g_AdvInstance.adv_params.duration     		= MSEC_TO_UNITS(pCfg->AdvTimeout, UNIT_10_MS);
    g_AdvInstance.p_adv_data = &g_AdvInstance.adv_data;
    g_AdvInstance.adv_data.adv_data.p_data = g_AdvInstance.enc_advdata[0];
    g_AdvInstance.adv_data.scan_rsp_data.p_data = g_AdvInstance.enc_scan_rsp_data[0];

	g_AdvInstance.adv_modes_config.ble_adv_fast_enabled  = true;
	g_AdvInstance.adv_modes_config.ble_adv_extended_enabled = pCfg->bExtAdv;
	g_AdvInstance.adv_modes_config.ble_adv_fast_interval = MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
	g_AdvInstance.adv_modes_config.ble_adv_fast_timeout  = MSEC_TO_UNITS(pCfg->AdvTimeout, UNIT_10_MS);

	if (pCfg->AdvSlowInterval > 0)
	{
		g_AdvInstance.adv_modes_config.ble_adv_slow_enabled  = true;
		g_AdvInstance.adv_modes_config.ble_adv_slow_interval = MSEC_TO_UNITS(pCfg->AdvSlowInterval, UNIT_0_625_MS);
		g_AdvInstance.adv_modes_config.ble_adv_slow_timeout  = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
	}

	g_AdvInstance.adv_modes_config.ble_adv_extended_enabled = pCfg->bExtAdv;

	if (pCfg->bExtAdv == true)
    {
    	if (pCfg->Role & BLEAPP_ROLE_BROADCASTER)
    	{
    		g_AdvInstance.adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    	}
    	if (pCfg->Role & BLEAPP_ROLE_PERIPHERAL)
    	{
    		g_AdvInstance.adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
    	}

    	g_AdvInstance.adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED;
    	g_AdvInstance.adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED;
    }
    else
    {
    	if (pCfg->Role & BLEAPP_ROLE_BROADCASTER)
    	{
    		g_AdvInstance.adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    	}
    	if (pCfg->Role & BLEAPP_ROLE_PERIPHERAL)
    	{
    		g_AdvInstance.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    	}
    	g_AdvInstance.adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    	g_AdvInstance.adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
   }

    if (pCfg->pAdvManData != NULL)
    {
		g_BleAppData.ManufData.company_identifier = pCfg->VendorID;
		g_BleAppData.ManufData.data.p_data = (uint8_t*)pCfg->pAdvManData;
		g_BleAppData.ManufData.data.size = pCfg->AdvManDataLen;
    }
    if (pCfg->pSrManData != NULL)
    {
		g_BleAppData.SRManufData.company_identifier = pCfg->VendorID;
		g_BleAppData.SRManufData.data.p_data = (uint8_t*)pCfg->pSrManData;
		g_BleAppData.SRManufData.data.size = pCfg->SrManDataLen;
    }
    // Build advertising data struct to pass into @ref ble_advertising_init.

    if (pCfg->Appearance != BLE_APPEARANCE_UNKNOWN)
    {
    	initdata.advdata.include_appearance = true;
    }

    if (pCfg->AdvTimeout != 0)
    {
        // ADV for a limited time, use this flag
        initdata.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    }
    else
    {
        // Always ADV use this flag
        initdata.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    }

    if (pCfg->pDevName != NULL)
    {
    	if (strlen(pCfg->pDevName) > 30)
    	{
    		initdata.advdata.name_type      = BLE_ADVDATA_SHORT_NAME;
    		initdata.advdata.short_name_len = 30;//strlen(pCfg->pDevName);
    	}
    	else
    	{
    		initdata.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    		initdata.advdata.short_name_len = strlen(pCfg->pDevName);
    	}
    }
    else
    {
    		initdata.advdata.name_type = BLE_ADVDATA_NO_NAME;
    }

	ble_uuid_t uid[MAX_ADV_UUID];

	if (pCfg->pAdvUuid != NULL)
    {
		if (pCfg->pAdvUuid->Type == BLE_UUID_TYPE_16)
		{
			uint8_t utype = pCfg->pAdvUuid->BaseIdx == 0 ?
							BLE_UUID_TYPE_BLE : BLE_UUID_TYPE_VENDOR_BEGIN;
			for (int i = 0; i < min(pCfg->pAdvUuid->Count, MAX_ADV_UUID); i++)
			{
				uid[i].uuid = pCfg->pAdvUuid->Val[i].Uuid16;
				uid[i].type = utype;
			}
		}
    }

	if (pCfg->bExtAdv)
	{
        if (pCfg->pAdvUuid != NULL)
        {
			if (pCfg->bCompleteUuidList)
			{
				initdata.advdata.uuids_complete.uuid_cnt = pCfg->pAdvUuid->Count;
				initdata.advdata.uuids_complete.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
			}
			else
			{
				initdata.advdata.uuids_more_available.uuid_cnt = pCfg->pAdvUuid->Count;
				initdata.advdata.uuids_more_available.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
			}
        }
		if (pCfg->pAdvManData != NULL)
		{
			initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
		}
	}
	else
	{
		if (initdata.advdata.name_type == BLE_ADVDATA_NO_NAME)
		{
			if (pCfg->pAdvUuid != NULL)
			{
				if (pCfg->pAdvManData != NULL)
				{
					initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;

					if (pCfg->bCompleteUuidList)
					{
						initdata.srdata.uuids_complete.uuid_cnt = pCfg->pAdvUuid->Count;
						initdata.srdata.uuids_complete.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
					}
					else
					{
						initdata.srdata.uuids_more_available.uuid_cnt = pCfg->pAdvUuid->Count;
						initdata.srdata.uuids_more_available.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
					}
				}
				else
				{
					if (pCfg->bCompleteUuidList)
					{
						initdata.advdata.uuids_complete.uuid_cnt = pCfg->pAdvUuid->Count;
						initdata.advdata.uuids_complete.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
					}
					else
					{
						initdata.advdata.uuids_more_available.uuid_cnt = pCfg->pAdvUuid->Count;
						initdata.advdata.uuids_more_available.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
					}
					if (pCfg->pSrManData != NULL)
					{
						initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
					}
				}
			}
			else
			{
				if (pCfg->pAdvManData != NULL)
				{
					initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
				}
				if (pCfg->pSrManData != NULL)
				{
					initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
				}
			}
		}
		else
		{
			if (pCfg->pAdvUuid != NULL)
			{
				if (pCfg->bCompleteUuidList)
				{
					initdata.srdata.uuids_complete.uuid_cnt = pCfg->pAdvUuid->Count;
					initdata.srdata.uuids_complete.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
				}
				else
				{
					initdata.srdata.uuids_more_available.uuid_cnt = pCfg->pAdvUuid->Count;
					initdata.srdata.uuids_more_available.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
				}

				if (pCfg->pAdvManData != NULL)
				{
					initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
				}

			}
			else
			{
				if (pCfg->pAdvManData != NULL)
				{
					initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
				}
				if (pCfg->pSrManData != NULL)
				{
					initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
				}
			}
		}
	}

    err_code = ble_advdata_encode(&initdata.advdata, g_AdvInstance.adv_data.adv_data.p_data, &g_AdvInstance.adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    //if (pCfg->pSrManData)
    {
    	err_code = ble_advdata_encode(&initdata.srdata, g_AdvInstance.adv_data.scan_rsp_data.p_data, &g_AdvInstance.adv_data.scan_rsp_data.len);
    	APP_ERROR_CHECK(err_code);
    }

    memcpy(&g_BleAppData.AdvData, &initdata.advdata, sizeof(ble_advdata_t));
    memcpy(&g_BleAppData.SrData, &initdata.srdata, sizeof(ble_advdata_t));

    err_code = sd_ble_gap_adv_set_configure(&g_AdvInstance.adv_handle, &g_AdvInstance.adv_data, &g_AdvInstance.adv_params);
    APP_ERROR_CHECK(err_code);

    g_AdvInstance.initialized = true;
#endif

    return true;
}

void BleAppDisInit(const BleAppCfg_t *pBleAppCfg)
{
    ble_dis_init_t   dis_init;
    ble_dis_pnp_id_t pnp_id;

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

	if (pBleAppCfg->pDevDesc)
	{
		ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char*)pBleAppCfg->pDevDesc->ManufName);
		ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char*)pBleAppCfg->pDevDesc->ModelName);
		if (pBleAppCfg->pDevDesc->pSerialNoStr)
			ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char*)pBleAppCfg->pDevDesc->pSerialNoStr);
		if (pBleAppCfg->pDevDesc->pFwVerStr)
			ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char*)pBleAppCfg->pDevDesc->pFwVerStr);
		if (pBleAppCfg->pDevDesc->pHwVerStr)
			ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char*)pBleAppCfg->pDevDesc->pHwVerStr);
	}

    pnp_id.vendor_id_source = BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG;
    pnp_id.vendor_id  = pBleAppCfg->VendorID;
    pnp_id.product_id = pBleAppCfg->ProductId;
    dis_init.p_pnp_id = &pnp_id;

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    switch (pBleAppCfg->SecType)
    {
    	case BLEAPP_SECTYPE_STATICKEY_NO_MITM:
    	    dis_init.dis_char_rd_sec = SEC_JUST_WORKS;
    		break;
    	case BLEAPP_SECTYPE_STATICKEY_MITM:
    	    dis_init.dis_char_rd_sec = SEC_MITM;
    		break;
    	case BLEAPP_SECTYPE_LESC_MITM:
    	    dis_init.dis_char_rd_sec = SEC_JUST_WORKS;
    		break;
    	case BLEAPP_SECTYPE_SIGNED_NO_MITM:
    	    dis_init.dis_char_rd_sec = SEC_SIGNED;
    		break;
    	case BLEAPP_SECTYPE_SIGNED_MITM:
    	    dis_init.dis_char_rd_sec = SEC_SIGNED_MITM;
    		break;
    	case BLEAPP_SECTYPE_NONE:
    	default:
    	    dis_init.dis_char_rd_sec = SEC_OPEN;
    	    break;
    }

    uint32_t err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

}

uint16_t BleAppGetConnHandle()
{
	return g_BleAppData.ConnHdl;
}

/**@brief Function for handling events from the GATT library. */
void BleGattEvtHandler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t * p_evt)
{
    if ((g_BleAppData.ConnHdl == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
    	//g_BleAppData.MaxMtu = p_evt->params.att_mtu_effective - 3;//OPCODE_LENGTH - HANDLE_LENGTH;
       // m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Data len is set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
 //   printf("ATT MTU exchange completed. central 0x%x peripheral 0x%x\r\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void BleAppGattInit(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&s_Gatt, BleGattEvtHandler);
    APP_ERROR_CHECK(err_code);

    if (g_BleAppData.Role & BLEAPP_ROLE_PERIPHERAL)
    {
    	err_code = nrf_ble_gatt_att_mtu_periph_set(&s_Gatt, g_BleAppData.MaxMtu);
    	APP_ERROR_CHECK(err_code);

    	if (g_BleAppData.MaxMtu >= 27)
    	{
    		// 251 bytes is max dat length as per Bluetooth core spec 5, vol 6, part b, section 4.5.10
    		// 27 - 251 bytes is hardcoded in nrf_ble_gat of the SDK.
    		uint8_t dlen = g_BleAppData.MaxMtu > 254 ? 251: g_BleAppData.MaxMtu - 3;
    		err_code = nrf_ble_gatt_data_length_set(&s_Gatt, BLE_CONN_HANDLE_INVALID, dlen);
    		APP_ERROR_CHECK(err_code);
    	}
      	ble_opt_t opt;

      	memset(&opt, 0x00, sizeof(opt));
      	opt.common_opt.conn_evt_ext.enable = 1;

      	err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
      	APP_ERROR_CHECK(err_code);
    }

    if (g_BleAppData.Role & BLEAPP_ROLE_CENTRAL)
    {
    	err_code = nrf_ble_gatt_att_mtu_central_set(&s_Gatt, g_BleAppData.MaxMtu);
    	APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
    if (p_fds_evt->id == FDS_EVT_GC)
    {
//        printf("GC completed");
    }
}

bool BleAppConnectable(const BleAppCfg_t *pBleAppCfg, bool bEraseBond)
{
	uint32_t err_code;

    //BleAppInitUserData();

	BleAppGapParamInit(pBleAppCfg);

	//gatt_init();

//	if (pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT)
   // if (pBleAppCfg->AdvType != BLEADV_TYPE_ADV_NONCONN_IND)
	if (pBleAppCfg->Role & BLEAPP_ROLE_PERIPHERAL)
		conn_params_init();

	BleAppInitUserServices();

	//if (pBleAppCfg->bEnDevInfoService)
	if (pBleAppCfg->pDevDesc != NULL)
		BleAppDisInit(pBleAppCfg);

	return true;
}


/**@brief Function for Initializing the Nordic SoftDevice firmware stack
 *
 * @param[in] CentLinkCount
 * @param[in] PeriLinkCount
 * @param[in] bConnectable
 */
bool BleAppStackInit(int CentLinkCount, int PeriLinkCount, bool bConnectable)
{
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;

    ret_code_t err_code = nrf_sdh_ble_default_cfg_set(BLEAPP_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);


    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the number of custom UUIDS.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    //if (CentLinkCount > 0)
        ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = BLESRVC_UUID_BASE_MAXCNT;
    //else
    //	ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 2;
    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum number of connections.
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                     = BLEAPP_CONN_CFG_TAG;
	ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = PeriLinkCount;
	ble_cfg.gap_cfg.role_count_cfg.central_role_count = CentLinkCount;
	ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = CentLinkCount ? BLE_GAP_ROLE_COUNT_CENTRAL_SEC_DEFAULT: 0;
	err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the maximum ATT MTU.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                 = BLEAPP_CONN_CFG_TAG;
	ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = g_BleAppData.MaxMtu;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the maximum event length.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                     = BLEAPP_CONN_CFG_TAG;
	ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
	ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = PeriLinkCount + CentLinkCount;//BLE_GAP_CONN_COUNT_DEFAULT;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gatts_cfg.attr_tab_size.attr_tab_size = 3000;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gatts_cfg.service_changed.service_changed = 1;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
#if 0
    memset(&ble_cfg, 0, sizeof ble_cfg);
    ble_cfg.conn_cfg.conn_cfg_tag 					= BLEAPP_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 10;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
#endif
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(g_BleObserver, BLEAPP_OBSERVER_PRIO, ble_evt_dispatch, NULL);

    return true;
}

static void ble_rtos_evt_dispatch(ble_evt_t const * p_ble_evt, void *p_context)
{
    g_BleAppData.SDEvtHandler();
}

int8_t GetValidTxPower(int TxPwr)
{
	int8_t retval = s_TxPowerdBm[0];

	for (int i = 1; i < s_NbTxPowerdBm; i++)
	{
		if (s_TxPowerdBm[i] > TxPwr)
			break;

		retval = s_TxPowerdBm[i];
	}

	return retval;
}

uint32_t GetLFAccuracy(uint32_t AccPpm)
{
	uint32_t retval = 0;

	if (AccPpm < 2)
	{
		retval = 11;
	}
	else if (AccPpm < 5)
	{
		retval = 10;
	}
	else if (AccPpm < 10)
	{
		retval = 9;
	}
	else if (AccPpm < 20)
	{
		retval = 8;
	}
	else if (AccPpm < 30)
	{
		retval = 7;
	}
	else if (AccPpm < 50)
	{
		retval = 6;
	}
	else if (AccPpm < 75)
	{
		retval = 5;
	}
	else if (AccPpm < 100)
	{
		retval = 4;
	}
	else if (AccPpm < 150)
	{
		retval = 3;
	}
	else if (AccPpm < 250)
	{
		retval = 2;
	}
	else if (AccPpm < 500)
	{
		retval = 0;
	}
	else
	{
		retval = 1;
	}

	return retval;
}

/**
 * @brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
bool BleAppInit(const BleAppCfg_t *pBleAppCfg)//, bool bEraseBond)
{
	ret_code_t err_code;

	g_BleAppData.State = BLEAPP_STATE_UNKNOWN;
	g_BleAppData.bExtAdv = pBleAppCfg->bExtAdv;
	g_BleAppData.bScan = false;
	g_BleAppData.bAdvertising = false;
	g_BleAppData.VendorId = pBleAppCfg->VendorID;
	g_BleAppData.ConnLedPort = pBleAppCfg->ConnLedPort;
	g_BleAppData.ConnLedPin = pBleAppCfg->ConnLedPin;
	g_BleAppData.ConnLedActLevel = pBleAppCfg->ConnLedActLevel;

	if (pBleAppCfg->ConnLedPort != -1 && pBleAppCfg->ConnLedPin != -1)
    {
		IOPinConfig(pBleAppCfg->ConnLedPort, pBleAppCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

		BleConnLedOff();
    }

	g_BleAppData.Role = pBleAppCfg->Role;
	g_BleAppData.AdvHdl = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    g_BleAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;

    if (pBleAppCfg->MaxMtu > NRF_BLE_MAX_MTU_SIZE)
		g_BleAppData.MaxMtu = pBleAppCfg->MaxMtu;
    else
    	g_BleAppData.MaxMtu = NRF_BLE_MAX_MTU_SIZE;

    app_timer_init();
#if 0
    switch (g_BleAppData.AppMode)
    {
		case BLEAPP_MODE_LOOP:
		case BLEAPP_MODE_NOCONNECT:
			// app_timer_init();
			break;
		case BLEAPP_MODE_APPSCHED:
			// app_timer_init();
			APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
			break;
		case BLEAPP_MODE_RTOS:
			if (pBleAppCfg->SDEvtHandler == NULL)
				return false;

			g_BleAppData.SDEvtHandler = pBleAppCfg->SDEvtHandler;

			break;
			default:
				;
    }
#endif
	g_BleAppData.SDEvtHandler = pBleAppCfg->SDEvtHandler;
    if (pBleAppCfg->SDEvtHandler == NULL)
    {
		APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    }

//    nrf_ble_lesc_init();

    nrf_clock_lf_cfg_t lfclk = {
    	0
    };

	OscDesc_t const *lfosc = GetLowFreqOscDesc();
	if (lfosc->Type == OSC_TYPE_RC)
	{
		lfclk.source = NRF_CLOCK_LF_SRC_RC;
		lfclk.rc_ctiv = 16;
		lfclk.rc_temp_ctiv = 2;
	}
	else
	{
		lfclk.accuracy = GetLFAccuracy(lfosc->Accuracy);
		lfclk.source = NRF_CLOCK_LF_SRC_XTAL;
	}

	err_code = nrf_sdh_enable(&lfclk);//(nrf_clock_lf_cfg_t *)&pBleAppCfg->ClkCfg);
    APP_ERROR_CHECK(err_code);

    // Initialize SoftDevice.
    BleAppStackInit(pBleAppCfg->CentLinkCount, pBleAppCfg->PeriLinkCount,
    				pBleAppCfg->Role & BLEAPP_ROLE_PERIPHERAL);
    				//pBleAppCfg->AdvType != BLEADV_TYPE_ADV_NONCONN_IND);
//    				pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT);

    //err_code = ble_lesc_init();
    //APP_ERROR_CHECK(err_code);

	if (pBleAppCfg->PeriLinkCount > 0 && pBleAppCfg->AdvInterval > 0)
	{
		//g_BleAppData.AppRole |= BLEAPP_ROLE_PERIPHERAL;

		if (pBleAppCfg->pDevName != NULL)
	    {
			int l = strlen(pBleAppCfg->pDevName);
	        err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
	                                          (const uint8_t *) pBleAppCfg->pDevName,
	                                          min(l, 30));
	        APP_ERROR_CHECK(err_code);
	    }
	    err_code = sd_ble_gap_appearance_set(pBleAppCfg->Appearance);
	    APP_ERROR_CHECK(err_code);
	}

    if (pBleAppCfg->Role & BLEAPP_ROLE_PERIPHERAL)
	//if (pBleAppCfg->AdvType != BLEADV_TYPE_ADV_NONCONN_IND)
    {
    	BleAppConnectable(pBleAppCfg, false);//bEraseBond);
    }

    if (pBleAppCfg->CentLinkCount > 0)
	{
//		g_BleAppData.AppRole |= BLEAPP_ROLE_CENTRAL;
//		ret_code_t err_code = ble_db_discovery_init(BleAppDBDiscoveryHandler);
//		APP_ERROR_CHECK(err_code);
    }

    BleAppGattInit();

    BleAppInitUserData();

    BleAppPeerMngrInit(pBleAppCfg->SecType, pBleAppCfg->SecExchg, false);//bEraseBond);

	if (pBleAppCfg->SecType != BLEAPP_SECTYPE_NONE)
	{
	    g_BleAppData.bSecure = true;
	}
	else
	{
	    g_BleAppData.bSecure = false;
	}

   // err_code = fds_register(fds_evt_handler);
   // APP_ERROR_CHECK(err_code);

    // Generate the ECDH key pair and set public key in the peer-manager.
    //err_code = ble_lesc_ecc_keypair_generate_and_set();
    //APP_ERROR_CHECK(err_code);

    if (g_BleAppData.Role & (BLEAPP_ROLE_PERIPHERAL | BLEAPP_ROLE_BROADCASTER))
    {
        if (BleAppAdvInit(pBleAppCfg) == false)
        {
        	return false;
        }

        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, g_BleAppData.AdvHdl, GetValidTxPower(pBleAppCfg->TxPower));
        APP_ERROR_CHECK(err_code);
        BtGapInit(g_BleAppData.Role);
    }
    else
    {
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, g_BleAppData.AdvHdl, GetValidTxPower(pBleAppCfg->TxPower));
        APP_ERROR_CHECK(err_code);
    }

#if (__FPU_USED == 1)
    // Patch for softdevice & FreeRTOS to sleep properly when FPU is in used
    NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif

    if (AppEvtHandlerInit(pBleAppCfg->pEvtHandlerQueMem, pBleAppCfg->EvtHandlerQueMemSize) == false)
    {
    	return false;
    }

    g_BleAppData.State = BLEAPP_STATE_INITIALIZED;

    return true;
}

void BleAppRun()
{
	if (g_BleAppData.State != BLEAPP_STATE_INITIALIZED)
	{
		return;
	}

	g_BleAppData.bAdvertising = false;
	g_BleAppData.State = BLEAPP_STATE_IDLE;

	if (g_BleAppData.Role & (BLEAPP_ROLE_PERIPHERAL | BLEAPP_ROLE_BROADCASTER))// != BLEAPP_ROLE_CENTRAL)
	{
		BleAppAdvStart();//BLEAPP_ADVMODE_FAST);
	}

	while (1)
    {
//		if (g_BleAppData.AppMode == BLEAPP_MODE_RTOS)
    	if (g_BleAppData.SDEvtHandler != NULL)
		{
			BleAppRtosWaitEvt();
		}
		else
		{
//			if (g_BleAppData.AppMode == BLEAPP_MODE_APPSCHED)
			{
				app_sched_execute();
				AppEvtHandlerExec();
			}
			nrf_ble_lesc_request_handler();
			sd_app_evt_wait();
		}
    }

	/*	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
		{
			uint32_t err_code = sd_ble_gap_adv_start(g_AdvInstance.adv_handle, BLEAPP_CONN_CFG_TAG);
			//uint32_t err_code = ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
		}
		else
		{
			if (g_BleAppData.AppRole & BLEAPP_ROLE_PERIPHERAL)
			{
				uint32_t err_code = ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_FAST);
				APP_ERROR_CHECK(err_code);
			}
		}
	*/
}


void BleTimerAppRun()
{
	if (g_BleAppData.State != BLEAPP_STATE_INITIALIZED)
	{
		return;
	}

	g_BleAppData.bAdvertising = false;
	g_BleAppData.State = BLEAPP_STATE_IDLE;

	if (g_BleAppData.Role & (BLEAPP_ROLE_PERIPHERAL | BLEAPP_ROLE_BROADCASTER))
	{
		BleAppAdvStart();//BLEAPP_ADVMODE_FAST);
	}

    while (1)
    {
//		if (g_BleAppData.AppMode == BLEAPP_MODE_RTOS)
    	if (g_BleAppData.SDEvtHandler != NULL)
		{
			BleAppRtosWaitEvt();
		}
		else
		{
			//if (g_BleAppData.AppMode == BLEAPP_MODE_APPSCHED)
			{
				app_sched_execute();
			}
			nrf_ble_lesc_request_handler();
			sd_app_evt_wait();
		}
    }

	/*	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
		{
			uint32_t err_code = sd_ble_gap_adv_start(g_AdvInstance.adv_handle, BLEAPP_CONN_CFG_TAG);
			//uint32_t err_code = ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
		}
		else
		{
			if (g_BleAppData.AppRole & BLEAPP_ROLE_PERIPHERAL)
			{
				uint32_t err_code = ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_FAST);
				APP_ERROR_CHECK(err_code);
			}
		}
	*/
}


void BleAppScan()
{
	ret_code_t err_code;

	if (g_BleAppData.bScan == true)
	{
		err_code = sd_ble_gap_scan_start(NULL, &g_BleScanReportData);
	}
	else
	{
	    g_BleAppData.bScan = true;

		err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	}
	APP_ERROR_CHECK(err_code);
}

void BleAppScanStop()
{
	if (g_BleAppData.bScan == true)
	{
		ret_code_t err_code = sd_ble_gap_scan_stop();
		APP_ERROR_CHECK(err_code);
		g_BleAppData.bScan = false;
	}
}

bool BleAppScanInit(BleAppScanCfg_t *pCfg)
{
	if (pCfg == NULL)
	{
		return false;
	}

	s_BleScanParams.timeout = pCfg->Timeout;
	s_BleScanParams.window = pCfg->Duration;
	s_BleScanParams.interval = pCfg->Interval;

    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ble_uuid128_t uid;

    memcpy(uid.uuid128, pCfg->BaseUid, 16);

    ret_code_t err_code = sd_ble_uuid_vs_add(&uid, &uidtype);
    APP_ERROR_CHECK(err_code);

    g_BleAppData.bScan = true;

	err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}

bool BleAppScanInit(ble_uuid128_t * const pBaseUid, ble_uuid_t * const pServUid)
{
    ble_uuid128_t base_uid = *pBaseUid;
    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ret_code_t err_code = sd_ble_uuid_vs_add(&base_uid, &uidtype);
    APP_ERROR_CHECK(err_code);

    //ble_db_discovery_evt_register(pServUid);
    g_BleAppData.bScan = true;

	err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}

//bool BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam)
uint32_t BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam)
{
	ret_code_t err_code = sd_ble_gap_connect(pDevAddr, &s_BleScanParams,
                                  	  	  	 pConnParam,
											 BLEAPP_CONN_CFG_TAG);
    //APP_ERROR_CHECK(err_code);

    g_BleAppData.bScan = false;

    //return err_code == NRF_SUCCESS;
    return err_code;
}

bool BleAppEnableNotify(uint16_t ConnHandle, uint16_t CharHandle)//ble_uuid_t * const pCharUid)
{
    uint32_t                 err_code;
    ble_gattc_write_params_t write_params;
    uint8_t                  buf[BLE_CCCD_VALUE_LEN];

    buf[0] = BLE_GATT_HVX_NOTIFICATION;
    buf[1] = 0;

    write_params.write_op = BLE_GATT_OP_WRITE_CMD;//BLE_GATT_OP_WRITE_REQ;
    write_params.flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
    write_params.handle   = CharHandle;
    write_params.offset   = 0;
    write_params.len      = sizeof(buf);
    write_params.p_value  = buf;

    err_code = sd_ble_gattc_write(ConnHandle, &write_params);

    return err_code == NRF_SUCCESS;
}

bool BleAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DataLen)
{
	if (ConnHandle == BLE_CONN_HANDLE_INVALID || CharHandle == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = CharHandle,
        .offset   = 0,
        .len      = DataLen,
        .p_value  = pData
    };

    return sd_ble_gattc_write(ConnHandle, &write_params) == NRF_SUCCESS;
}

/**@brief   Function for polling SoftDevice events.
 *
 * @note    This function is compatible with @ref app_sched_event_handler_t.
 *
 * @param[in]   p_event_data Pointer to the event data.
 * @param[in]   event_size   Size of the event data.
 */
static void appsh_events_poll(void * p_event_data, uint16_t event_size)
{
    nrf_sdh_evts_poll();

    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);
}

extern "C" void SD_EVT_IRQHandler(void)
{
#if 0
	switch (g_BleAppData.AppMode)
	{
		case BLEAPP_MODE_LOOP:
		case BLEAPP_MODE_NOCONNECT:
			nrf_sdh_evts_poll();
			break;
		case BLEAPP_MODE_APPSCHED:
			{
				ret_code_t ret_code = app_sched_event_put(NULL, 0, appsh_events_poll);

				APP_ERROR_CHECK(ret_code);
			}
			break;
		case BLEAPP_MODE_RTOS:
			if (g_BleAppData.SDEvtHandler)
			{
				g_BleAppData.SDEvtHandler();
			}
			break;
		default:
			;
	}
#endif
	if (g_BleAppData.SDEvtHandler != NULL)
	{
		g_BleAppData.SDEvtHandler();
	}
	else
	{
		ret_code_t ret_code = app_sched_event_put(NULL, 0, appsh_events_poll);

		APP_ERROR_CHECK(ret_code);
	}
}

void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc)
{

}

// We need this here in order for the Linker to keep the nrf_sdh_soc.c
// which is require for Softdevice to function properly
// Create section set "sdh_soc_observers".
// This is needed for FSTORAGE event to work.
NRF_SDH_STACK_OBSERVER(m_nrf_sdh_soc_evts_poll, NRF_SDH_SOC_STACK_OBSERVER_PRIO) = {
    .handler   = nrf_sdh_soc_evts_poll,
    .p_context = NULL,
};

#ifndef __ARMCC_VERSION

#if NRF_MODULE_ENABLED(NRF_CRYPTO) && NRF_MODULE_ENABLED(NRF_CRYPTO_BACKEND_CC310)
extern nrf_crypto_backend_info_t const cc310_backend;
// Just to make the linker to keep the nrf_hw_backend
__attribute__ ((used)) static uint32_t s_pcc310_backend_info = (uint32_t)&cc310_backend;
#endif

#if NRF_MODULE_ENABLED(NRF_CRYPTO) && NRF_MODULE_ENABLED(NRF_CRYPTO_BACKEND_NRF_HW_RNG)
extern nrf_crypto_backend_info_t const nrf_hw_backend;
// Just to make the linker to keep the nrf_hw_backend
__attribute__ ((used)) static uint32_t s_pnrf_hw_backend_info = (uint32_t)&nrf_hw_backend;
#endif

#endif

