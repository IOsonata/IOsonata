/**-------------------------------------------------------------------------
@file	bt_app_nrf52.cpp

@brief	Nordic SDK based Bluetooth application creation helper


@author	Hoang Nguyen Hoan
@date	Dec 26, 2016

@license

MIT License

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <atomic>

#include "sdk_config.h"
#include "nordic_common.h"
#include "ble_hci.h"
#include "nrf_error.h"
#include "ble_gatt.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "nrf_ble_gatt.h"
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_dfu_settings.h"
#include "nrf_bootloader_info.h"

#include "nrf_ble_scan.h"
#include "nrf_drv_rng.h"

#include "istddef.h"
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "timer_nrfx.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_smp.h"
//#include "ble_app_nrf5.h"
#include "bluetooth/bt_dev.h"
#include "app_evt_handler.h"		// AppWait, overridden here
#include "bt_lesc.h"
#include "sd_dispatch.h"

/******** For DEBUG ************/
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

extern "C" void nrf_sdh_soc_evts_poll(void * p_context);
extern "C" ret_code_t nrf_sdh_enable(nrf_clock_lf_cfg_t *clock_lf_cfg);

/**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#if (NRF_SD_BLE_API_VERSION <= 3)
   #define NRF_BLE_MAX_MTU_SIZE        GATT_MTU_SIZE_DEFAULT
#else

#if  defined(BLE_GATT_MTU_SIZE_DEFAULT) && !defined(GATT_MTU_SIZE_DEFAULT)
#define GATT_MTU_SIZE_DEFAULT BLE_GATT_MTU_SIZE_DEFAULT
#endif

#if  defined(BLE_GATT_ATT_MTU_DEFAULT) && !defined(GATT_MTU_SIZE_DEFAULT)
#define GATT_MTU_SIZE_DEFAULT  			BLE_GATT_ATT_MTU_DEFAULT
#endif

#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT

#endif

#define BTAPP_CONN_CFG_TAG            1     /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define BLEAPP_OBSERVER_PRIO           1                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
//#define BLEAPP_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define SCHED_MAX_EVENT_DATA_SIZE 		20 /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE          		40                        /**< Maximum number of events in the scheduler queue. */
#endif

//#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
//#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  5		// Seconds from connect to the first connection parameter update request
#define NEXT_CONN_PARAMS_UPDATE_DELAY   30		// Seconds between the following requests

#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define MAX_ADV_UUID					10

// ORable application role
//#define BLEAPP_ROLE_PERIPHERAL			1
//#define BLEAPP_ROLE_CENTRAL				2

// These are to be passed as parameters
//#define SCAN_INTERVAL           MSEC_TO_UNITS(100, UNIT_0_625_MS)      /**< Determines scan interval in units of 0.625 millisecond. */
//#define SCAN_WINDOW             MSEC_TO_UNITS(100, UNIT_0_625_MS)		/**< Determines scan window in units of 0.625 millisecond. */
//#define SCAN_TIMEOUT            0                                 		/**< Timout when scanning. 0x0000 disables timeout. */

#if 1
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
#endif

// GATT module instance. It is not registered as a SoftDevice observer of its
// own (NRF_BLE_GATT_DEF): an observer is always kept by the linker. The
// connection event handler passes it the events, so the module is linked
// only with connection support.
static nrf_ble_gatt_t s_Gatt;

// Connection support. What a link needs (peer table, GAP setup, connection
// parameters, GATT module, Device Information Service, connection events,
// GATT timeout) is reached only through this table. BtAppConnInit installs
// it. An application that only advertises or scans never reaches
// BtAppConnInit, so none of it is linked.
typedef struct {
	void (*Evt)(ble_evt_t const * p_ble_evt, void *p_context);
	void (*Tick)(void);
	void (*SrvcDone)(const BtAppCfg_t *pCfg);
} BtAppNrf52Conn_t;

static const BtAppNrf52Conn_t *s_pBtAppNrf52Conn = nullptr;

// Configuration given to BtAppInit, used by BtAppConnInit
static const BtAppCfg_t *s_pBtAppCfg = nullptr;

// Set when BtAppInit is past the service setup. Connection support started
// after that point finishes the service setup itself.
static bool s_bBtAppSrvcDone = false;

// g_BtAppData definition and helpers (isConnected, BtConnected, BtInitialized,
// BtAppConnLedOff/On) moved to src/bluetooth/bt_app.cpp.

//BtDevice_t g_BtDevnRF5;

//static volatile bool s_BleStarted = false;

// =====================================================================
// Central GATT client discovery (nRF52 SoftDevice path)
// ---------------------------------------------------------------------
// The SoftDevice owns the host, so discovery runs on sd_ble_gattc_*_discover
// and the matching BLE_GATTC_EVT_* responses (dispatched from the central
// event switch). Sequence: all primary services, then characteristics per
// service, then descriptors per characteristic to resolve the CCCD handle.
// On completion BtDeviceDiscovered(pDev) is called (weak default in
// bt_app.cpp, overridden by the app).
//
// The nRF52 SoftDevice ble_gattc API is identical to the S145 one used by the
// nRF54L bm port, so this is the same state machine.
//
// The cursor lives in the peer record (pDev->Discovery), one per link. It used
// to be three file-scope statics, so starting discovery on a second connection
// overwrote the first procedure's position and both walked the survivor's
// table. Each response resolves its peer from the event's connection handle,
// which is the only thing that says which procedure the response belongs to.
// =====================================================================

static void BtAppDiscStartChar(BtDevice_t *pDev);
static void BtAppDiscStartDesc(BtDevice_t *pDev);

bool BtAppDiscoverDevice(BtDevice_t * const pDev)
{
	if (pDev == NULL)
	{
		return false;
	}

	pDev->Discovery.SrvIdx  = 0;
	pDev->Discovery.CharIdx = 0;

	pDev->NbSrvc = 0;
	if (BtDeviceSrvcCacheAttach(pDev) == false)
	{
		// No free discovery cache, see g_BtDevSrvcCacheCfg
		return false;
	}
	memset(pDev->pServices, 0, sizeof(BtGattDBSrvc_t) * BT_DEV_SERVICE_MAXCNT);

	// NULL uuid filter discovers every primary service from handle 1.
	return sd_ble_gattc_primary_services_discover(pDev->Conn.Hdl, 0x0001, NULL) == NRF_SUCCESS;
}

static void BtAppDiscStartChar(BtDevice_t *pDev)
{
	while (pDev->Discovery.SrvIdx < pDev->NbSrvc)
	{
		BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->Discovery.SrvIdx];
		ble_gattc_handle_range_t range;

		range.start_handle = pSrvc->handle_range.StartHdl + 1;  // skip service declaration
		range.end_handle   = pSrvc->handle_range.EndHdl;

		if (range.start_handle <= range.end_handle &&
		    sd_ble_gattc_characteristics_discover(pDev->Conn.Hdl, &range) == NRF_SUCCESS)
		{
			return;     // wait for BLE_GATTC_EVT_CHAR_DISC_RSP
		}
		pDev->Discovery.SrvIdx++; // empty service or request rejected; advance
	}

	// All services covered for characteristics. Resolve descriptors (CCCD).
	pDev->Discovery.SrvIdx  = 0;
	pDev->Discovery.CharIdx = 0;
	BtAppDiscStartDesc(pDev);
}

static void BtAppDiscStartDesc(BtDevice_t *pDev)
{
	while (pDev->Discovery.SrvIdx < pDev->NbSrvc)
	{
		BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->Discovery.SrvIdx];

		while (pDev->Discovery.CharIdx < pSrvc->char_count)
		{
			BtGattDBChar_t *pCh    = &pSrvc->characteristics[pDev->Discovery.CharIdx];
			uint16_t        valHdl = pCh->characteristic.handle_value;
			uint16_t        start  = valHdl + 1;
			uint16_t        end;

			if ((pDev->Discovery.CharIdx + 1) < pSrvc->char_count)
			{
				end = pSrvc->characteristics[pDev->Discovery.CharIdx + 1].characteristic.handle_decl - 1;
			}
			else
			{
				end = pSrvc->handle_range.EndHdl;
			}

			if (valHdl != 0xFFFF && start <= end)
			{
				ble_gattc_handle_range_t range;
				range.start_handle = start;
				range.end_handle   = end;
				if (sd_ble_gattc_descriptors_discover(pDev->Conn.Hdl, &range) == NRF_SUCCESS)
				{
					return; // wait for BLE_GATTC_EVT_DESC_DISC_RSP
				}
			}
			pDev->Discovery.CharIdx++; // no descriptor range or request rejected
		}
		pDev->Discovery.SrvIdx++;
		pDev->Discovery.CharIdx = 0;
	}

	// Every service and characteristic processed. Discovery complete.
	BtDeviceDiscovered(pDev);
}

static void BtAppDiscPrimSrvcRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = BtPeerFindByHdl(pEvt->conn_handle);
	if (pDev == NULL || pDev->pServices == NULL)
	{
		// Unknown link, or discovery not started by BtAppDiscoverDevice
		return;
	}

	if (pEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
	{
		const ble_gattc_evt_prim_srvc_disc_rsp_t *p = &pEvt->params.prim_srvc_disc_rsp;
		uint16_t lastEnd = 0;

		for (uint16_t i = 0; i < p->count; i++)
		{
			if (pDev->NbSrvc >= BT_DEV_SERVICE_MAXCNT)
			{
				break;
			}

			BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->NbSrvc];
			pSrvc->srv_uuid.BaseIdx = 0;
			pSrvc->srv_uuid.Type    = BT_UUID_TYPE_16;
			pSrvc->srv_uuid.Uuid    = p->services[i].uuid.uuid;
			pSrvc->handle_range.StartHdl = p->services[i].handle_range.start_handle;
			pSrvc->handle_range.EndHdl   = p->services[i].handle_range.end_handle;
			pSrvc->char_count = 0;
			lastEnd = p->services[i].handle_range.end_handle;
			pDev->NbSrvc++;
		}

		// Continue past the last range unless DB end or local table full.
		if (lastEnd != 0xFFFF && pDev->NbSrvc < BT_DEV_SERVICE_MAXCNT &&
		    sd_ble_gattc_primary_services_discover(pDev->Conn.Hdl, lastEnd + 1, NULL) == NRF_SUCCESS)
		{
			return;
		}
	}

	// ATTRIBUTE_NOT_FOUND, DB end, or table full: service phase done.
	pDev->Discovery.SrvIdx = 0;
	BtAppDiscStartChar(pDev);
}

static void BtAppDiscCharRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = BtPeerFindByHdl(pEvt->conn_handle);
	if (pDev == NULL || pDev->Discovery.SrvIdx >= pDev->NbSrvc)
	{
		return;
	}

	BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->Discovery.SrvIdx];

	if (pEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
	{
		const ble_gattc_evt_char_disc_rsp_t *p = &pEvt->params.char_disc_rsp;
		uint16_t lastVal = 0;

		for (uint16_t i = 0; i < p->count; i++)
		{
			if (pSrvc->char_count >= BT_GATT_DB_MAX_CHARS)
			{
				break;
			}

			BtGattDBChar_t         *pCh  = &pSrvc->characteristics[pSrvc->char_count];
			const ble_gattc_char_t *pSrc = &p->chars[i];

			pCh->characteristic.uuid.BaseIdx = 0;
			pCh->characteristic.uuid.Type    = BT_UUID_TYPE_16;
			pCh->characteristic.uuid.Uuid    = pSrc->uuid.uuid;

			pCh->characteristic.char_props.broadcast      = pSrc->char_props.broadcast;
			pCh->characteristic.char_props.read           = pSrc->char_props.read;
			pCh->characteristic.char_props.write_wo_resp  = pSrc->char_props.write_wo_resp;
			pCh->characteristic.char_props.write          = pSrc->char_props.write;
			pCh->characteristic.char_props.notify         = pSrc->char_props.notify;
			pCh->characteristic.char_props.indicate       = pSrc->char_props.indicate;
			pCh->characteristic.char_props.auth_signed_wr = pSrc->char_props.auth_signed_wr;

			pCh->characteristic.char_ext_props = pSrc->char_ext_props;
			pCh->characteristic.handle_decl    = pSrc->handle_decl;
			pCh->characteristic.handle_value   = pSrc->handle_value;

			pCh->cccd_handle       = BLE_GATT_HANDLE_INVALID;
			pCh->ext_prop_handle   = BLE_GATT_HANDLE_INVALID;
			pCh->user_desc_handle  = BLE_GATT_HANDLE_INVALID;
			pCh->report_ref_handle = BLE_GATT_HANDLE_INVALID;

			lastVal = pSrc->handle_value;
			pSrvc->char_count++;
		}

		// Continue characteristics within the same service range.
		if (lastVal != 0 && lastVal < 0xFFFF && pSrvc->char_count < BT_GATT_DB_MAX_CHARS &&
		    (lastVal + 1) <= pSrvc->handle_range.EndHdl)
		{
			ble_gattc_handle_range_t range;
			range.start_handle = lastVal + 1;
			range.end_handle   = pSrvc->handle_range.EndHdl;
			if (sd_ble_gattc_characteristics_discover(pDev->Conn.Hdl, &range) == NRF_SUCCESS)
			{
				return;
			}
		}
	}

	// Service complete (ATTRIBUTE_NOT_FOUND, table full, or error). Advance.
	pDev->Discovery.SrvIdx++;
	BtAppDiscStartChar(pDev);
}

static void BtAppDiscDescRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = BtPeerFindByHdl(pEvt->conn_handle);
	if (pDev == NULL || pDev->Discovery.SrvIdx >= pDev->NbSrvc)
	{
		return;
	}

	if (pEvt->gatt_status == BLE_GATT_STATUS_SUCCESS &&
		pDev->Discovery.CharIdx < pDev->pServices[pDev->Discovery.SrvIdx].char_count)
	{
		const ble_gattc_evt_desc_disc_rsp_t *p = &pEvt->params.desc_disc_rsp;
		BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->Discovery.SrvIdx];
		BtGattDBChar_t *pCh   = &pSrvc->characteristics[pDev->Discovery.CharIdx];

		for (uint16_t i = 0; i < p->count; i++)
		{
			switch (p->descs[i].uuid.uuid)
			{
				case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
					pCh->cccd_handle = p->descs[i].handle;
					break;
				case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
					pCh->ext_prop_handle = p->descs[i].handle;
					break;
				case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
					pCh->user_desc_handle = p->descs[i].handle;
					break;
				case BT_UUID_DESCRIPTOR_REPORT_REFERENCE:
					pCh->report_ref_handle = p->descs[i].handle;
					break;
				default:
					break;
			}
		}
	}

	// Advance to the next characteristic regardless of result.
	pDev->Discovery.CharIdx++;
	BtAppDiscStartDesc(pDev);
}

void BtAppEnterDfu()
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

// BtAppNotify/BtAppIndicate, their Conn and All forms are the shared weak
// implementations in src/bluetooth/bt_app.cpp. This port used to carry a copy
// that read the active connection handle. What stays here is the disconnect
// command, which is the one part only this port can issue.

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
#if 1

void BtAppDisconnectConn(uint16_t ConnHdl)
{
	if (ConnHdl != BLE_CONN_HANDLE_INVALID)
    {
		uint32_t err_code = sd_ble_gap_disconnect(ConnHdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}
    }
}

void BtAppGapDeviceNameSet(const char* pDeviceName)
{
	BtGapSetDevName(pDeviceName);
/*
    uint32_t                err_code;

    err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
                                          (const uint8_t *)pDeviceName,
                                          strlen( pDeviceName ));
    APP_ERROR_CHECK(err_code);
    //ble_advertising_restart_without_whitelist(&g_AdvInstance);
*/
}
#endif

// Connection parameter negotiation of a peripheral link. When the parameters
// set by the central are outside the preferred ones (PPCP), an update is
// requested FIRST_CONN_PARAMS_UPDATE_DELAY after the connection, then every
// NEXT_CONN_PARAMS_UPDATE_DELAY, MAX_CONN_PARAMS_UPDATE_COUNT times at most.
// The link is dropped when the central still does not follow. The delays
// are counted by the 1 s trigger of the port timer.
#if NRF_SDH_BLE_PERIPHERAL_LINK_COUNT > 0
#define BTAPP_CONNPARAM_MAX		NRF_SDH_BLE_PERIPHERAL_LINK_COUNT
#else
#define BTAPP_CONNPARAM_MAX		1
#endif

typedef struct {
	uint16_t ConnHdl;		// BLE_CONN_HANDLE_INVALID when the slot is free
	uint8_t Delay;			// Seconds left before the next request, 0 when not counting
	uint8_t Count;			// Number of requests sent
} BtAppConnParam_t;

static BtAppConnParam_t s_BtAppConnParam[BTAPP_CONNPARAM_MAX];

static BtAppConnParam_t *BtAppConnParamFind(uint16_t ConnHdl)
{
	for (int i = 0; i < BTAPP_CONNPARAM_MAX; i++)
	{
		if (s_BtAppConnParam[i].ConnHdl == ConnHdl)
		{
			return &s_BtAppConnParam[i];
		}
	}

	return nullptr;
}

// Compare the parameters of the link to the preferred ones
static bool BtAppConnParamOk(const ble_gap_conn_params_t *pParam)
{
	ble_gap_conn_params_t pref;

	if (sd_ble_gap_ppcp_get(&pref) != NRF_SUCCESS)
	{
		// Nothing to compare to
		return true;
	}

	// max_conn_interval of the event is the interval set by the central
	if (pParam->max_conn_interval < pref.min_conn_interval ||
		pParam->max_conn_interval > pref.max_conn_interval)
	{
		return false;
	}

	uint32_t dev = NRF_BLE_CONN_PARAMS_MAX_SLAVE_LATENCY_DEVIATION;
	uint32_t hi = pref.slave_latency + dev;
	uint32_t lo = pref.slave_latency - min(dev, (uint32_t)pref.slave_latency);

	if (pParam->slave_latency < lo || pParam->slave_latency > hi)
	{
		return false;
	}

	dev = NRF_BLE_CONN_PARAMS_MAX_SUPERVISION_TIMEOUT_DEVIATION;
	hi = pref.conn_sup_timeout + dev;
	lo = pref.conn_sup_timeout - min(dev, (uint32_t)pref.conn_sup_timeout);

	if (pParam->conn_sup_timeout < lo || pParam->conn_sup_timeout > hi)
	{
		return false;
	}

	return true;
}

// Start or end the negotiation according to the parameters of the link
static void BtAppConnParamCheck(BtAppConnParam_t *pSlot, const ble_gap_conn_params_t *pParam)
{
	if (BtAppConnParamOk(pParam))
	{
		pSlot->Delay = 0;
		pSlot->Count = 0;

		return;
	}

	pSlot->Delay = pSlot->Count == 0 ? FIRST_CONN_PARAMS_UPDATE_DELAY :
									   NEXT_CONN_PARAMS_UPDATE_DELAY;
}

static void BtAppConnParamEvt(ble_evt_t const *pEvt)
{
	ble_gap_evt_t const *pGapEvt = &pEvt->evt.gap_evt;
	BtAppConnParam_t *pSlot;

	switch (pEvt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			if (pGapEvt->params.connected.role != BLE_GAP_ROLE_PERIPH)
			{
				break;
			}
			pSlot = BtAppConnParamFind(BLE_CONN_HANDLE_INVALID);
			if (pSlot != nullptr)
			{
				pSlot->ConnHdl = pGapEvt->conn_handle;
				pSlot->Count = 0;
				BtAppConnParamCheck(pSlot, &pGapEvt->params.connected.conn_params);
			}
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			pSlot = BtAppConnParamFind(pGapEvt->conn_handle);
			if (pSlot != nullptr)
			{
				pSlot->Delay = 0;
				pSlot->ConnHdl = BLE_CONN_HANDLE_INVALID;
			}
			break;

		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
			pSlot = BtAppConnParamFind(pGapEvt->conn_handle);
			if (pSlot != nullptr)
			{
				BtAppConnParamCheck(pSlot, &pGapEvt->params.conn_param_update.conn_params);
			}
			break;

		default:
			break;
	}
}

// Called every second by the port timer
static void BtAppConnParamTick(void)
{
	for (int i = 0; i < BTAPP_CONNPARAM_MAX; i++)
	{
		BtAppConnParam_t *pSlot = &s_BtAppConnParam[i];

		if (pSlot->ConnHdl == BLE_CONN_HANDLE_INVALID || pSlot->Delay == 0)
		{
			continue;
		}

		pSlot->Delay--;
		if (pSlot->Delay > 0)
		{
			continue;
		}

		if (pSlot->Count < MAX_CONN_PARAMS_UPDATE_COUNT)
		{
			ble_gap_conn_params_t pref;

			// The answer comes as BLE_GAP_EVT_CONN_PARAM_UPDATE, which
			// checks the new parameters and counts the next delay.
			if (sd_ble_gap_ppcp_get(&pref) == NRF_SUCCESS &&
				sd_ble_gap_conn_param_update(pSlot->ConnHdl, &pref) == NRF_SUCCESS)
			{
				pSlot->Count++;
			}
		}
		else
		{
			// The central did not follow, drop the link
			pSlot->Count = 0;
			(void)sd_ble_gap_disconnect(pSlot->ConnHdl, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		}
	}
}

static void BtAppConnParamInit(void)
{
	for (int i = 0; i < BTAPP_CONNPARAM_MAX; i++)
	{
		s_BtAppConnParam[i].ConnHdl = BLE_CONN_HANDLE_INVALID;
		s_BtAppConnParam[i].Delay = 0;
		s_BtAppConnParam[i].Count = 0;
	}
}

// Peer Manager event handling, the SMP user interaction bridge and the LESC
// OOB data handling are in bt_sec_nrf52.cpp. That object is linked only when
// the application calls BtAppSecInit.

/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void BtAppNrf52ConnEvt(ble_evt_t const * p_ble_evt, void *p_context)
{
    uint32_t err_code;

	// GATT module first, as when it was an observer of its own
	nrf_ble_gatt_on_ble_evt(p_ble_evt, &s_Gatt);

	BtAppConnParamEvt(p_ble_evt);

	ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
	uint16_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);

	// The LESC module receives BLE events through the peer_manager path:
	// sm_ble_evt_handler (bt_sec_sd.cpp) forwards every event to BtLescOnBleEvt
	// as its single delivery point. Calling it here as well would deliver each
	// event twice and run sd_ble_gap_lesc_oob_data_set twice on an OOB DHKey
	// request, which latches the module internal error.

	// Trace security-relevant GAP events to diagnose the pairing flow.
	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			DEBUG_PRINTF("SEC: EVT_SEC_PARAMS_REQUEST (pairing started by peer/PM)\r\n");
			if (g_BtAppData.bSecInit == false)
			{
				// The application did not start the security module
				// (BtAppSecInit). Nothing else answers this request, so
				// refuse the pairing now and not after the SMP timeout.
				(void)sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
						BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
			}
			break;
		case BLE_GAP_EVT_SEC_INFO_REQUEST:
			if (g_BtAppData.bSecInit == false)
			{
				// No bond is kept without the security module: no key.
				(void)sd_ble_gap_sec_info_reply(p_ble_evt->evt.gap_evt.conn_handle,
						NULL, NULL, NULL);
			}
			break;
		case BLE_GAP_EVT_SEC_REQUEST:
			if (g_BtAppData.bSecInit == false)
			{
				// Central role: reject the peripheral Security Request.
				(void)sd_ble_gap_authenticate(p_ble_evt->evt.gap_evt.conn_handle, NULL);
			}
			break;
		case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
			DEBUG_PRINTF("SEC: EVT_LESC_DHKEY_REQUEST (LESC in progress)\r\n");
			break;
		// BLE_GAP_EVT_PASSKEY_DISPLAY and BLE_GAP_EVT_AUTH_KEY_REQUEST are
		// handled by the security observer in bt_sec_nrf52.cpp.
		case BLE_GAP_EVT_AUTH_STATUS:
			DEBUG_PRINTF("SEC: EVT_AUTH_STATUS status=0x%X lesc=%d bonded=%d\r\n",
					p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
					p_ble_evt->evt.gap_evt.params.auth_status.lesc,
					p_ble_evt->evt.gap_evt.params.auth_status.bonded);
			break;
		case BLE_GAP_EVT_CONN_SEC_UPDATE:
		{
			const ble_gap_conn_sec_t *pcs = &p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec;
			DEBUG_PRINTF("SEC: EVT_CONN_SEC_UPDATE sm=%d lv=%d ks=%d\r\n",
					pcs->sec_mode.sm, pcs->sec_mode.lv, pcs->encr_key_size);
			// Capture the negotiated encryption key size for BtGapConnSecGet.
			// pm_conn_sec_status_get does not report key size; this event does.
			BtDevice_t *pdev = BtPeerFindByHdl(p_ble_evt->evt.gap_evt.conn_handle);
			if (pdev != nullptr)
			{
				pdev->Conn.Sec.KeySize = pcs->encr_key_size;
			}
		}
			break;
		default:
			break;
	}

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
		{
			uint8_t connRole = p_gap_evt->params.connected.role;
			uint8_t peerRole = connRole == BLE_GAP_ROLE_CENTRAL ? BT_CONN_ROLE_CENTRAL
							 : connRole == BLE_GAP_ROLE_PERIPH ? BT_CONN_ROLE_PERIPHERAL
							 : BT_CONN_ROLE_UNKNOWN;

        	// Own address not stamped: SMP runs in the SoftDevice, not the
        	// IOsonata toolbox, so the record is left for the
        	// BtSmpLocalAddrGet fallback.
			BtDevice_t *pPeer = BtPeerConnected(p_gap_evt->conn_handle,
										 peerRole,
        								 p_gap_evt->params.connected.peer_addr.addr_type,
        								 (uint8_t*)p_gap_evt->params.connected.peer_addr.addr,
        								 0, NULL);
			if (pPeer == nullptr)
			{
				// BtPeerConnected already routes rejection through
				// BtPeerConnectionRejected. Do not expose the untracked link to
				// security, GATT, or application callbacks.
				return;
			}

        	BtAppConnLedOn();
        	g_BtAppData.State = BTAPP_STATE_CONNECTED;

        	// If a secure SecType was configured, security is requested on
        	// the link by the security observer in bt_sec_nrf52.cpp
        	// (pm_conn_secure -> sd_ble_gap_authenticate). It runs after this
        	// dispatcher and after the Peer Manager have seen the event.

        	BtAppEvtConnected(p_ble_evt->evt.gap_evt.conn_handle);
		}
        	break;
        case BLE_GAP_EVT_DISCONNECTED:
			{
				uint16_t connHdl = p_ble_evt->evt.gap_evt.conn_handle;
				BtDevice_t *pPeer = BtPeerFindByHdl(connHdl);

				if (pPeer != nullptr)
				{
					BtPeerFree(pPeer);
				}

				bool connected = BtPeerIsConnected();
				if (connected)
				{
					g_BtAppData.State = BTAPP_STATE_CONNECTED;
					BtAppConnLedOn();
				}
				else
				{
					g_BtAppData.State = BTAPP_STATE_IDLE;
					BtAppConnLedOff();
				}

				if (pPeer != nullptr)
				{
					BtAppEvtDisconnected(connHdl);
				}

				// A disconnect can free peripheral capacity or the last peer slot.
				// BtAdvStart performs both checks and is a no-op for central-only apps.
				BtAdvStart();
			}
        	break;
        case BLE_GAP_EVT_ADV_SET_TERMINATED:
        	BtAppAdvTimeoutHandler();
        	break;
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
        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
        	ble_gap_data_length_params_t dl_params;

//           printf("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST\r\n");

			// Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
			memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
			err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
			APP_ERROR_CHECK(err_code);
        } break;
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored. The event names the link
            // that is missing them; the active handle could be a different one.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING
        case BLE_EVT_USER_MEM_REQUEST:
            {
                // Long write: hand the SoftDevice this link's reassembly buffer
                // so it can queue the prepared writes. Replied once per
                // connection here (not per service) to avoid multiple
                // sd_ble_user_mem_reply calls on the same request.
                BtDevice_t *pConn = BtPeerFindByHdl(p_ble_evt->evt.common_evt.conn_handle);
                if (pConn != nullptr && pConn->Conn.pLongWrBuff != nullptr)
                {
                    ble_user_mem_block_t mblk;
                    memset(&mblk, 0, sizeof(mblk));
                    mblk.p_mem = pConn->Conn.pLongWrBuff;
                    mblk.len   = pConn->Conn.LongWrBuffSize;
                    memset(pConn->Conn.pLongWrBuff, 0, pConn->Conn.LongWrBuffSize);
                    err_code = sd_ble_user_mem_reply(p_ble_evt->evt.common_evt.conn_handle, &mblk);
                    APP_ERROR_CHECK(err_code);
                }
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
   }
   // on_ble_evt(p_ble_evt);
#if 1
    if ((role == BLE_GAP_ROLE_CENTRAL) || g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER))
    {
    	switch (p_ble_evt->header.evt_id)
        {
    		// BLE_GAP_EVT_ADV_REPORT and the scan timeout are handled by the
    		// scan observer in bt_scan_nrf52.cpp. Keeping them out of this
    		// dispatcher lets a build that never scans leave the scan module
    		// and its report buffer out of the link.
            case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
                BtAppDiscPrimSrvcRsp(&p_ble_evt->evt.gattc_evt);
                break;

            case BLE_GATTC_EVT_CHAR_DISC_RSP:
                BtAppDiscCharRsp(&p_ble_evt->evt.gattc_evt);
                break;

            case BLE_GATTC_EVT_DESC_DISC_RSP:
                BtAppDiscDescRsp(&p_ble_evt->evt.gattc_evt);
                break;

            case BLE_GATTC_EVT_HVX:
            	{
            		const ble_gattc_evt_hvx_t *p_hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
            		BtGattClientNotified(p_ble_evt->evt.gattc_evt.conn_handle,
            		                     p_hvx->handle, (uint8_t*)p_hvx->data, p_hvx->len);
            		// An indication must be confirmed by the client.
            		if (p_hvx->type == BLE_GATT_HVX_INDICATION)
            		{
            			sd_ble_gattc_hv_confirm(p_ble_evt->evt.gattc_evt.conn_handle,
            			                        p_hvx->handle);
            		}
            	}
            	break;
#if 0
            case BLE_GAP_EVT_SCAN_REQ_REPORT:
            	{
            	    ble_gap_evt_scan_req_report_t const * p_req_report = &p_gap_evt->params.scan_req_report;
            	}
            	break;
            case BLE_GATTC_EVT_HVX:
            	if (p_ble_evt->evt.gattc_evt.params.hvx.handle == g_BleRxCharHdl)
            	{
            		g_Uart.Tx(p_ble_evt->evt.gattc_evt.params.hvx.data, p_ble_evt->evt.gattc_evt.params.hvx.len);
            	}
            	break;
#endif
        }
    	BtAppCentralEvtHandler(p_ble_evt->header.evt_id, (void*)p_ble_evt);
    }
    if (g_BtAppData.AppDevice.Conn.Role & BTAPP_ROLE_PERIPHERAL)
    {
    	// HVN TX complete gives a count, not a handle. Drain the per-peer
    	// send-order ring once per event.
    	if (p_ble_evt->header.evt_id == BLE_GATTS_EVT_HVN_TX_COMPLETE)
    	{
    		BtGattSendCompleted(p_ble_evt->evt.gatts_evt.conn_handle,
    			p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count);
    	}
    	else if (p_ble_evt->header.evt_id == BLE_GATTS_EVT_HVC)
    	{
    		// Client confirmed the outstanding indication.
    		BtGattHandleValueConfirm(p_ble_evt->evt.gatts_evt.conn_handle);
    	}
    	BtGattEvtHandler(p_ble_evt->header.evt_id, (void*)p_ble_evt);
    	BtAppPeriphEvtHandler(p_ble_evt->header.evt_id, (void*)p_ble_evt);
    }
#endif
}



// BLE event observer of the port. With connection support the events go to
// the connection event handler above. Without it there is no link, and only
// the events of advertising and scanning are of interest.
static void ble_evt_dispatch(ble_evt_t const * p_ble_evt, void *p_context)
{
	if (s_pBtAppNrf52Conn != nullptr)
	{
		s_pBtAppNrf52Conn->Evt(p_ble_evt, p_context);

		return;
	}

	if (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED)
	{
		BtAppAdvTimeoutHandler();
	}

	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER))
	{
		BtAppCentralEvtHandler(p_ble_evt->header.evt_id, (void*)p_ble_evt);
	}
}

/**@brief Function for handling events from the GATT library. */
void BtGattEvtHandler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t * p_evt)
{
    // The MTU update belongs to the link the event names, so there is nothing
    // to compare it against.
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
    	// Record the negotiated ATT_MTU on the peer slot. The body here was
    	// commented out and nothing else wrote the slot, so
    	// BtGattNrf52ValueLenValid read zero, fell back to the default and
    	// refused every notification and indication over 20 octets for the
    	// life of a link that had negotiated 247. The GATT module has already
    	// applied the Core Vol 3 Part F 3.4.2.2 minimum of the two Rx MTUs, so
    	// the effective value is taken as it stands and only refused if it is
    	// below what every ATT PDU may assume.
    	BtDevice_t *pConn = BtPeerFindByHdl(p_evt->conn_handle);
    	if (pConn != nullptr &&
    		p_evt->params.att_mtu_effective >= BT_ATT_MTU_MIN)
    	{
    		pConn->Conn.MaxMtu = p_evt->params.att_mtu_effective;
    	}
    }
 //   printf("ATT MTU exchange completed. central 0x%x peripheral 0x%x\r\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void BtGattInit(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&s_Gatt, BtGattEvtHandler);
    APP_ERROR_CHECK(err_code);

    if (g_BtAppData.AppDevice.Conn.Role & BT_GAP_ROLE_PERIPHERAL)
    {
    	err_code = nrf_ble_gatt_att_mtu_periph_set(&s_Gatt, g_BtAppData.AppDevice.Conn.MaxMtu);
    	APP_ERROR_CHECK(err_code);

    	if (g_BtAppData.AppDevice.Conn.MaxMtu >= 27)
    	{
    		// 251 bytes is max dat length as per Bluetooth core spec 5, vol 6, part b, section 4.5.10
    		// 27 - 251 bytes is hardcoded in nrf_ble_gat of the SDK.
    		// It is very confusing in the SDK. According to the Specs, it should be ATT MTU + 4
    		// where Max length cannot be more than 251. Which means ATT MTU max is no more than 247.
    		uint8_t dlen = g_BtAppData.AppDevice.Conn.MaxMtu + 4;//> 254 ? 251: g_BtAppData.AppDevice.Conn.MaxMtu - 3;
    		err_code = nrf_ble_gatt_data_length_set(&s_Gatt, BLE_CONN_HANDLE_INVALID, dlen);
    		APP_ERROR_CHECK(err_code);
    	}
      	ble_opt_t opt;

      	memset(&opt, 0x00, sizeof(opt));
      	opt.common_opt.conn_evt_ext.enable = 1;

      	err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
      	APP_ERROR_CHECK(err_code);
    }

    if (g_BtAppData.AppDevice.Conn.Role & BT_GAP_ROLE_CENTRAL)
    {
    	err_code = nrf_ble_gatt_att_mtu_central_set(&s_Gatt, g_BtAppData.AppDevice.Conn.MaxMtu);
    	APP_ERROR_CHECK(err_code);

    	ble_opt_t opt;
    	memset(&opt, 0x00, sizeof(opt));
    	opt.gattc_opt.uuid_disc.auto_add_vs_enable = 1;
    	err_code = sd_ble_opt_set(BLE_GATTC_OPT_UUID_DISC, &opt);
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


void BtDisInit(const BtAppCfg_t *pCfg)
{
    ble_dis_init_t   dis_init;
    ble_dis_pnp_id_t pnp_id;

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

	if (pCfg->pDevInfo)
	{
		ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char*)pCfg->pDevInfo->ManufName);
		ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char*)pCfg->pDevInfo->ModelName);
		if (pCfg->pDevInfo->pSerialNoStr)
			ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char*)pCfg->pDevInfo->pSerialNoStr);
		if (pCfg->pDevInfo->pFwVerStr)
			ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char*)pCfg->pDevInfo->pFwVerStr);
		if (pCfg->pDevInfo->pHwVerStr)
			ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char*)pCfg->pDevInfo->pHwVerStr);
	}

    pnp_id.vendor_id_source = BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG;
    pnp_id.vendor_id  = pCfg->VendorId;
    pnp_id.product_id = pCfg->ProductId;
    dis_init.p_pnp_id = &pnp_id;

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    switch (pCfg->SecType)
    {
    	case BTGAP_SECTYPE_STATICKEY_NO_MITM:
    	    dis_init.dis_char_rd_sec = SEC_JUST_WORKS;
    		break;
    	case BTGAP_SECTYPE_STATICKEY_MITM:
    	    dis_init.dis_char_rd_sec = SEC_MITM;
    		break;
    	case BTGAP_SECTYPE_LESC_MITM:
    	    dis_init.dis_char_rd_sec = SEC_JUST_WORKS;
    		break;
    	case BTGAP_SECTYPE_SIGNED_NO_MITM:
    	    dis_init.dis_char_rd_sec = SEC_SIGNED;
    		break;
    	case BTGAP_SECTYPE_SIGNED_MITM:
    	    dis_init.dis_char_rd_sec = SEC_SIGNED_MITM;
    		break;
    	case BTGAP_SECTYPE_NONE:
    	default:
    	    dis_init.dis_char_rd_sec = SEC_OPEN;
    	    break;
    }

    uint32_t err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for Initializing the Nordic SoftDevice firmware stack
 *
 * @param[in] PeriphDevMax
 * @param[in] CentralDevMax
 * @param[in] bConnectable
 */
bool BtAppStackInit(int MaxMtu, int PeriphDevMax, int CentralDevMax, bool bConnectable)
{
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;

    ret_code_t err_code = nrf_sdh_ble_default_cfg_set(BTAPP_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);


    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the number of custom UUIDS.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    //if (PeriphDevMax > 0)
        ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 4;//BLESRVC_UUID_BASE_MAXCNT;
    //else
    //	ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 2;
    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum number of connections.
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                     = BTAPP_CONN_CFG_TAG;
	ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = CentralDevMax;
	ble_cfg.gap_cfg.role_count_cfg.central_role_count = PeriphDevMax;
	ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = PeriphDevMax ? BLE_GAP_ROLE_COUNT_CENTRAL_SEC_DEFAULT: 0;
	err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the GAP device name attribute. The default storage is only
	// BLE_GAP_DEVNAME_DEFAULT_LEN (31) octets; without this, a device name
	// longer than 31 octets makes sd_ble_gap_device_name_set fail. Stack-
	// located, open write permission, sized to the protocol maximum (248).
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ble_cfg.gap_cfg.device_name_cfg.write_perm);
	ble_cfg.gap_cfg.device_name_cfg.vloc        = BLE_GATTS_VLOC_STACK;
	ble_cfg.gap_cfg.device_name_cfg.p_value     = NULL;
	ble_cfg.gap_cfg.device_name_cfg.current_len = 0;
	ble_cfg.gap_cfg.device_name_cfg.max_len     = BLE_GAP_DEVNAME_MAX_LEN;
	err_code = sd_ble_cfg_set(BLE_GAP_CFG_DEVICE_NAME, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the maximum ATT MTU.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                 = BTAPP_CONN_CFG_TAG;
	ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = MaxMtu;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the maximum event length.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                     = BTAPP_CONN_CFG_TAG;
	ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
	ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = CentralDevMax + PeriphDevMax;//BLE_GAP_CONN_COUNT_DEFAULT;
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

#if 1
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
#endif

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
static void BtAppSDDispatch(void);

static void BtAppNrf52TimerHandler(TimerDev_t * const pTimer, uint32_t Evt);

const static TimerCfg_t s_BtAppNrf52TimerCfg = {
	.DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,
	.EvtHandler = BtAppNrf52TimerHandler
};

// Port timer: millisecond clock of the GATT transaction timeout, 1 s count of
// the connection parameter negotiation and 1 s wakeup of the main loop. Only
// a link uses it. It is the timer device and not the Timer class: the class
// initializes through TimerInit, which links the high frequency timer driver
// along with the low frequency one.
static TimerDev_t s_BtAppNrf52Timer;

// Set while the timeout check is in the queue, so that it is queued once
static volatile bool s_bBtAppNrf52TickQueued = false;

// Timeout check of the connection support, queued once per second
static void BtAppNrf52TickEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_bBtAppNrf52TickQueued = false;

	// Generic indication transaction timeout (Core Vol 3 Part F 3.3.3). Cheap
	// no-op when nothing is pending.
	// The pairing timeout (Core Vol 3 Part H 3.4) is not driven here: the
	// SoftDevice runs SMP and its timer on this port.
	if (s_pBtAppNrf52Conn != nullptr)
	{
		s_pBtAppNrf52Conn->Tick();
	}
}

static void BtAppNrf52TimerHandler(TimerDev_t * const pTimer, uint32_t Evt)
{
	(void)pTimer;

	if (Evt & TIMER_EVT_TRIGGER(0))
	{
		BtAppConnParamTick();

		// The timeout check runs outside the interrupt, so that a fully silent
		// link still reaches it.
		if (s_bBtAppNrf52TickQueued == false)
		{
			s_bBtAppNrf52TickQueued = true;
			if (BtEvtQue(0, nullptr, BtAppNrf52TickEvt) == false)
			{
				s_bBtAppNrf52TickQueued = false;
			}
		}
	}
}

static void BtAppNrf52TimerStart(void)
{
	if (nRFxLFTimerInit(&s_BtAppNrf52Timer, &s_BtAppNrf52TimerCfg))
	{
		s_BtAppNrf52Timer.EnableTrigger(&s_BtAppNrf52Timer, 0, 1000000000ULL,
										TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	}
}

// Millisecond clock for the generic SMP/GATT transaction timeouts, overriding
// the weak BtSmpMsTick/BtGattMsTick defaults. Declared in bt_smp.h /
// bt_gatt.h, so no linkage specifier is needed here.
uint32_t BtSmpMsTick(void)
{
	if (s_BtAppNrf52Timer.GetTickCount == nullptr)
	{
		// Timer not started
		return 0;
	}

	return TimerGetMilisecond(&s_BtAppNrf52Timer);
}

uint32_t BtGattMsTick(void)
{
	return BtSmpMsTick();
}

// Spec-strict indication transaction timeout: Core Vol 3 Part F 3.3.3 requires
// closing the bearer, so disconnect the link. Overrides the generic weak
// default that only clears the outstanding-indication flag.
void BtGattIndicationTimeout(uint16_t ConnHdl)
{
	sd_ble_gap_disconnect(ConnHdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

// Last step of the service setup, after the application has added its
// services: Device Information Service and the GATT module.
static void BtAppNrf52SrvcDone(const BtAppCfg_t *pCfg)
{
	if (pCfg->pDevInfo != NULL)
	{
		BtDisInit(pCfg);
	}

	BtGattInit();
}

static const BtAppNrf52Conn_t s_BtAppNrf52Conn = {
	.Evt = BtAppNrf52ConnEvt,
	.Tick = BtGattIndicationTimeoutCheck,
	.SrvcDone = BtAppNrf52SrvcDone,
};

static bool BtAppNrf52ConnStart(const BtAppCfg_t *pCfg)
{
	if (!BtPeerInit(pCfg->pPeerPoolMem, pCfg->PeerPoolMemSize))
	{
		DEBUG_PRINTF("BtAppConnInit FAIL: BtPeerInit (pool mem=%p size=%d)\r\n",
			(void*)pCfg->pPeerPoolMem, (int)pCfg->PeerPoolMemSize);
		return false;
	}

	if (pCfg->PeriphDevMax + pCfg->CentralDevMax > (int)BtPeerCount())
	{
		// Peer pool holds fewer slots than the number of links requested.
		// Provide a larger pool, see g_BtPeerPoolCfg in bt_peer.h
		DEBUG_PRINTF("BtAppConnInit FAIL: peer pool %d slots < %d links\r\n",
			(int)BtPeerCount(), pCfg->PeriphDevMax + pCfg->CentralDevMax);
		return false;
	}

	// Split the long-write reassembly pool across the peer slots so each
	// link gets its own buffer (per Conn.pLongWrBuff).
	BtPeerLongWrInit(pCfg->pLongWrPoolMem, pCfg->LongWrPoolMemSize);

	BtGapCfg_t gapcfg = {
		.Role = pCfg->Role,
		.SecType = pCfg->SecType,
		.AdvInterval = pCfg->AdvInterval,
		.AdvTimeout = pCfg->AdvTimeout,
		.ConnIntervalMin = pCfg->ConnIntervalMin,
		.ConnIntervalMax = pCfg->ConnIntervalMax,
		.SlaveLatency = BT_GAP_CONN_SLAVE_LATENCY,
		.SupTimeout = BT_GAP_CONN_SUP_TIMEOUT
	};

	// The GAP and GATT services are the table a client reads first.
	if (BtGapInit(&gapcfg) == false)
	{
		DEBUG_PRINTF("BtAppConnInit FAIL: BtGapInit (GAP/GATT base services)\r\n");
		return false;
	}

	if (pCfg->pDevName != NULL)
	{
		BtGapSetDevName(pCfg->pDevName);
	}

	BtAppConnParamInit();

	// The timer is started here, with the SoftDevice enabled, so that the
	// low frequency clock is the one the SoftDevice started.
	BtAppNrf52TimerStart();

	return true;
}

/**
 * @brief	Start connection support.
 *
 * The call is what links the connection part of the port. The stack calls
 * it when the first GATT service is added, when a connection is initiated
 * and when security is started, so an application only calls it itself when
 * it is connectable without any service of its own. It needs the SoftDevice
 * enabled, which is the case from BtAppInitUserServices on.
 *
 * @return	true - connection support started
 */
bool BtAppConnInit(void)
{
	if (s_pBtAppNrf52Conn != nullptr)
	{
		// Already started
		return true;
	}

	if (s_pBtAppCfg == nullptr)
	{
		// BtAppInit has not been called yet
		return false;
	}

	// Set first: BtGapInit adds the GAP and GATT services, which comes back
	// to this function.
	s_pBtAppNrf52Conn = &s_BtAppNrf52Conn;

	if (BtAppNrf52ConnStart(s_pBtAppCfg) == false)
	{
		s_pBtAppNrf52Conn = nullptr;

		return false;
	}

	if (s_bBtAppSrvcDone)
	{
		// Started after BtAppInit, as a central does at its first connect
		BtAppNrf52SrvcDone(s_pBtAppCfg);
	}

	return true;
}

// Default queue of the SDK scheduler. An application defines its own
// g_BtAppSchedCfg to size it, or to leave the scheduler out, see bt_app.h.
static uint32_t s_BtAppSchedMem[CEIL_DIV(APP_SCHED_BUF_SIZE(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE),
										 sizeof(uint32_t))];

extern "C" __attribute__((weak)) const BtAppSchedCfg_t g_BtAppSchedCfg = {
	s_BtAppSchedMem, sizeof(s_BtAppSchedMem), SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE
};

bool BtAppInit(const BtAppCfg_t *pCfg)//, bool bEraseBond)
{
	ret_code_t err_code;

	//g_BleAppData.State = BLEAPP_STATE_UNKNOWN;
	//g_BleAppData.bExtAdv = pBleAppCfg->bExtAdv;
	//g_BleAppData.bScan = false;
	//g_BleAppData.bAdvertising = false;
	//g_BleAppData.VendorId = pBleAppCfg->VendorID;
	g_BtAppData.AppDevice.Conn.Role = pCfg->Role;
	// Kept for BtAppSecInit, which the application calls from
	// BtAppInitUserData when it uses security.
	g_BtAppData.SecType = pCfg->SecType;
	g_BtAppData.SecExchg = pCfg->SecExchg;
	g_BtAppData.bSecInit = false;
	g_BtAppData.AdvHdl = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

	// The peer table, the GAP setup, the connection parameters module and
	// the GATT module are set up by BtAppConnInit, see the service setup
	// below.
	s_pBtAppCfg = pCfg;
	s_bBtAppSrvcDone = false;
	g_BtAppData.ConnLedPort = pCfg->ConnLedPort;
	g_BtAppData.ConnLedPin = pCfg->ConnLedPin;
	g_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;
	g_BtAppData.bScan = false;
	//s_BtDevnRF5.bAdvertising = false;
	g_BtAppData.AppDevice.VendorId = pCfg->VendorId;
	g_BtAppData.AppDevice.ProductId = pCfg->ProductId;
	g_BtAppData.AppDevice.ProductVer = pCfg->ProductVer;
	g_BtAppData.AppDevice.Appearance = pCfg->Appearance;

	if (pCfg->ConnLedPort != -1 && pCfg->ConnLedPin != -1)
    {
		IOPinConfig(pCfg->ConnLedPort, pCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

		BtAppConnLedOff();
    }

	//g_BleAppData.Role = pBleAppCfg->Role;
	//g_BleAppData.AdvHdl = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    //g_BleAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;
	g_BtAppData.AppDevice.Conn.MaxMtu = NRF_BLE_MAX_MTU_SIZE;

    if (pCfg->MaxMtu > NRF_BLE_MAX_MTU_SIZE)
    	g_BtAppData.AppDevice.Conn.MaxMtu = pCfg->MaxMtu;
    else
    	g_BtAppData.AppDevice.Conn.MaxMtu = NRF_BLE_MAX_MTU_SIZE;

	// SDK scheduler, for applications that post events to it. The queue is
	// described by g_BtAppSchedCfg. An application that does not use the
	// scheduler defines the descriptor without memory and the default queue
	// is then not linked.
	if (g_BtAppSchedCfg.pMem != nullptr)
	{
		if (((uintptr_t)g_BtAppSchedCfg.pMem & 3U) != 0 ||
			g_BtAppSchedCfg.MemSize < APP_SCHED_BUF_SIZE(g_BtAppSchedCfg.EvtSize, g_BtAppSchedCfg.QueSize))
		{
			DEBUG_PRINTF("BtAppInit FAIL: scheduler queue memory (mem=%p size=%d)\r\n",
				g_BtAppSchedCfg.pMem, (int)g_BtAppSchedCfg.MemSize);
			return false;
		}

		err_code = app_sched_init(g_BtAppSchedCfg.EvtSize, g_BtAppSchedCfg.QueSize,
								  g_BtAppSchedCfg.pMem);
		APP_ERROR_CHECK(err_code);
	}

    nrf_clock_lf_cfg_t lfclk = {
    	0
    };

    SetSoftdeviceDispatch(BtAppSDDispatch);

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
    BtAppStackInit(g_BtAppData.AppDevice.Conn.MaxMtu, pCfg->PeriphDevMax, pCfg->CentralDevMax,
    				pCfg->Role & BTAPP_ROLE_PERIPHERAL);
    				//pBleAppCfg->AdvType != BLEADV_TYPE_ADV_NONCONN_IND);
//    				pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT);

/*
	if (pCfg->pDevName != NULL)
    {
		int l = strlen(pCfg->pDevName);
        err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
                                          (const uint8_t *) pCfg->pDevName,
                                          min(l, 30));
        APP_ERROR_CHECK(err_code);
    }
    */
    err_code = sd_ble_gap_appearance_set(pCfg->Appearance);
    APP_ERROR_CHECK(err_code);

    //BtDevInit(pCfg);

	// Service setup. Adding the first service starts connection support
	// (BtAppConnInit), which sets up GAP and the connection parameters module
	// ahead of the application services.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtAppInitUserServices();

		if (s_pBtAppNrf52Conn == nullptr)
		{
			// Peripheral role without any service. The application has to
			// call BtAppConnInit in BtAppInitUserServices to be connectable,
			// or use BTAPP_ROLE_BROADCASTER.
			DEBUG_PRINTF("BtAppInit FAIL: peripheral role but connection support was not started\r\n");
			return false;
		}
	}

	if (s_pBtAppNrf52Conn != nullptr)
	{
		s_pBtAppNrf52Conn->SrvcDone(pCfg);
	}
	s_bBtAppSrvcDone = true;

    BtAppInitUserData();

    // The security module (Peer Manager, LESC, ECDH engine) is linked and
    // started only when the application calls BtAppSecInit, normally from
    // BtAppInitUserData above. A configuration that asks for security without
    // starting it must not run unprotected.
    if (pCfg->SecType != BTGAP_SECTYPE_NONE && g_BtAppData.bSecInit == false)
    {
    	DEBUG_PRINTF("BtAppInit FAIL: SecType=%d but BtAppSecInit was not called\r\n",
    				 (int)pCfg->SecType);
    	return false;
    }

    g_BtAppData.AppDevice.bSecure = pCfg->SecType != BTGAP_SECTYPE_NONE;

    DEBUG_PRINTF("BtAppInit: reached adv gate, role=0x%02x\r\n",
        (unsigned)g_BtAppData.AppDevice.Conn.Role);
    if (g_BtAppData.AppDevice.Conn.Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_BROADCASTER))
    {
        if (BtAppAdvInit(pCfg) == false)
        {
        	DEBUG_PRINTF("BtAppInit FAIL: BtAppAdvInit\r\n");
        	return false;
        }
        DEBUG_PRINTF("BtAppInit: BtAppAdvInit ok, AdvHdl=%d\r\n",
            (int)g_BtAppData.AdvHdl);

        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, g_BtAppData.AdvHdl, GetValidTxPower(pCfg->TxPower));
        APP_ERROR_CHECK(err_code);
       // BtGapInit(g_BtAppData.AppDevice.Conn.Role);
    }
    else
    {
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, g_BtAppData.AdvHdl, GetValidTxPower(pCfg->TxPower));
        APP_ERROR_CHECK(err_code);
    }
    //err_code = ble_lesc_init();
    //APP_ERROR_CHECK(err_code);

#if (__FPU_USED == 1)
    // Patch for softdevice & FreeRTOS to sleep properly when FPU is in used
    NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif

    g_BtAppData.State = BTAPP_STATE_INITIALIZED;

    // Advertising starts here and not from the event queue: an application
    // interrupt can fill the queue before the application runs it.
    if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
    {
    	BtAdvStart();
    }

    return true;
}





//uint32_t BtAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam)
bool BtAppConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)
{
    g_BtAppData.bScan = false;

    return BtGapConnect(pPeerAddr, pConnParam);

#if 0
	ret_code_t err_code = sd_ble_gap_connect(pDevAddr, &s_BleScanParams,
                                  	  	  	 pConnParam,
											 BTAPP_CONN_CFG_TAG);
    //APP_ERROR_CHECK(err_code);

    g_BtAppData.bScan = false;
#endif

    //return err_code == NRF_SUCCESS;
    return true;
}

bool BtAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DataLen)
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

bool BtAppEnableNotify(uint16_t ConnHandle, uint16_t CharHandle)//ble_uuid_t * const pCharUid)
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


void BtAppCheckStatus(void)
{
	BtLescCheckStatus();
}

// Wait of AppRun while the SoftDevice is enabled: the SoftDevice must be the
// one putting the core to sleep. Overrides the weak WFE default, and is linked
// only by an application that uses Bluetooth. C linkage, as declared in
// app_evt_handler.h, or the weak default stays selected.
void AppWait(void)
{
	if (nrf_sdh_is_enabled() == false)
	{
		__WFE();
		return;
	}

	sd_app_evt_wait();
}

// Trampoline called from sd_dispatch.cpp's SD_EVT_IRQHandler. SoftDevice
// events are handled in the interrupt; what must run outside of it is queued
// with BtEvtQue by the handlers.
static void BtAppSDDispatch(void)
{
	nrf_sdh_evts_poll();
}

#if 1
// We need this here in order for the Linker to keep the nrf_sdh_soc.c
// which is require for Softdevice to function properly
// Create section set "sdh_soc_observers".
// This is needed for FSTORAGE event to work.
extern "C" void nrf_sdh_soc_evts_poll(void * p_context);

NRF_SDH_STACK_OBSERVER(m_nrf_sdh_soc_evts_poll, NRF_SDH_SOC_STACK_OBSERVER_PRIO) = {
    .handler   = nrf_sdh_soc_evts_poll,
    .p_context = NULL,
};

#endif
