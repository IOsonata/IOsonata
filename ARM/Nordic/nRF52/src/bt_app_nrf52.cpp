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
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
//#include "ble_app_nrf5.h"
#include "bluetooth/bt_dev.h"
#include "app_evt_handler.h"
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

#define APP_TIMER_OP_QUEUE_SIZE         10                                           /**< Size of timer operation queues. */

#define SCHED_MAX_EVENT_DATA_SIZE 		20 /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE          		40                        /**< Maximum number of events in the scheduler queue. */
#endif

//#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
//#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

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

NRF_BLE_GATT_DEF(s_Gatt);

// g_BtAppData definition and helpers (isConnected, BtConnected, BtInitialized,
// BtAppConnLedOff/On) moved to src/bluetooth/bt_app.cpp.

//BtDev_t g_BtDevnRF5;

//static volatile bool s_BleStarted = false;

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




//#endif

// =====================================================================
// LEGACY DISCOVERY STATE MACHINE - disabled.
// Step 7 lifts the discovery state machine to src/bluetooth/bt_dev.cpp
// (generic) and consumes the peer pool via BtPeer* helpers. The Nordic-specific
// state machine below was tightly coupled to ble_uuid_t / ble_gattc_*
// types and can't coexist with the Voci-unified BtDevice_t struct that
// uses BtUuid16_t / BtGattcHdlRange_t. Keeping it here as a reference
// until step 7's replacement lands; remove this block at that point.
#if 0
static BtDev_t *s_pBlePeriphData = NULL;

void BlePeriphDiscEvtHandler(ble_evt_t const *p_ble_evt, void *p_context);

__attribute__ ((section("." "sdh_ble_observers1"))) __attribute__((used))
static nrf_sdh_ble_evt_observer_t s_BlePeriphDiscObs = {
	.handler   = BlePeriphDiscEvtHandler,
	.p_context = (void*)&s_pBlePeriphData
};

static int s_CurSrvcHdl = 1;
static ble_uuid_t *s_CurSrvcUuid = NULL;
static ble_gatt_db_srv_t s_CurSrvcDisc;
static int s_CurSrvcIdx = 0;
static int s_CurCharIdx = 0;
static ble_gattc_handle_range_t s_CurRange;

void BlePeriphDiscEvtHandler(ble_evt_t const *p_ble_evt, void *p_context)
{
	BtDev_t *periph = *(BtDev_t**)p_context;
	ble_gattc_evt_t    const * p_ble_gattc_evt = &(p_ble_evt->evt.gattc_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
        	{
        		if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
                {
        			// Service discovered

        			ble_gattc_evt_prim_srvc_disc_rsp_t const * p_prim_srvc_disc_rsp_evt =
                		&(p_ble_gattc_evt->params.prim_srvc_disc_rsp);

            		for (int i = 0; i < p_prim_srvc_disc_rsp_evt->count; i++, periph->NbSrvc++)
					{
            			periph->Services[periph->NbSrvc].char_count = 0;
	        			periph->Services[periph->NbSrvc].srv_uuid = p_prim_srvc_disc_rsp_evt->services[i].uuid;
	        			periph->Services[periph->NbSrvc].handle_range = p_prim_srvc_disc_rsp_evt->services[i].handle_range;

						s_CurSrvcHdl = p_prim_srvc_disc_rsp_evt->services[i].handle_range.end_handle;
					}

					s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;

					// Find more services
					s_CurRange = periph->Services[periph->NbSrvc - 1].handle_range;

					uint32_t err_code = sd_ble_gattc_primary_services_discover(periph->Conn.Hdl, s_CurRange.end_handle + 1, NULL);
                }
        		else
        		{
        			// All service discovered
        			s_CurSrvcIdx = 0;
					s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;
					// Find characteristics

					uint32_t err = sd_ble_gattc_characteristics_discover(periph->Conn.Hdl, &s_CurRange);
        		}
        	}
        	break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:
            if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
            {
            	// Characteristic discovered

                ble_gattc_evt_char_disc_rsp_t const * p_char_disc_rsp_evt;

                p_char_disc_rsp_evt = &(p_ble_gattc_evt->params.char_disc_rsp);

                for (int i = 0; i < p_char_disc_rsp_evt->count; i++, periph->Services[s_CurSrvcIdx].char_count++)
                {
        			memcpy(&periph->Services[s_CurSrvcIdx].characteristics[periph->Services[s_CurSrvcIdx].char_count].characteristic,
                    	   &p_char_disc_rsp_evt->chars[i], sizeof(ble_gattc_char_t));
                    periph->Services[s_CurSrvcIdx].characteristics[periph->Services[s_CurSrvcIdx].char_count].cccd_handle = BLE_GATT_HANDLE_INVALID;
                    periph->Services[s_CurSrvcIdx].characteristics[periph->Services[s_CurSrvcIdx].char_count].ext_prop_handle = BLE_GATT_HANDLE_INVALID;
                    periph->Services[s_CurSrvcIdx].characteristics[periph->Services[s_CurSrvcIdx].char_count].user_desc_handle = BLE_GATT_HANDLE_INVALID;
                    periph->Services[s_CurSrvcIdx].characteristics[periph->Services[s_CurSrvcIdx].char_count].report_ref_handle = BLE_GATT_HANDLE_INVALID;

                    s_CurRange.start_handle = p_char_disc_rsp_evt->chars[i].handle_value + 1;
                }

                // Find more characteristics
				uint32_t err = sd_ble_gattc_characteristics_discover(periph->Conn.Hdl, &s_CurRange);
            }
            else
            {
            	// Find char for next service

				s_CurSrvcIdx++;

				if (s_CurSrvcIdx < periph->NbSrvc)
				{
					s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;
					s_CurRange.start_handle++;	// the service uses 1 handle already
					uint32_t err = sd_ble_gattc_characteristics_discover(periph->Conn.Hdl, &s_CurRange);
				}
				else
				{
					// All char found, Find descriptor
					s_CurSrvcIdx = 0;
					s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;

					uint32_t err_code = sd_ble_gattc_descriptors_discover(periph->Conn.Hdl, &s_CurRange);
				}
            }
            break;

        case BLE_GATTC_EVT_DESC_DISC_RSP:
            if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
            {
            	const ble_gattc_evt_desc_disc_rsp_t * p_desc_disc_rsp_evt = &(p_ble_gattc_evt->params.desc_disc_rsp);

            	for (int i = 0; i < p_desc_disc_rsp_evt->count; i++)
            	{
            		//g_Uart.printf("DESC %d uuid %x\r\n", i, p_desc_disc_rsp_evt->descs[i].uuid.uuid);
                    switch (p_desc_disc_rsp_evt->descs[i].uuid.uuid)
                    {
                        case BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG:
                            periph->Services[s_CurSrvcIdx].characteristics[s_CurCharIdx].cccd_handle = p_desc_disc_rsp_evt->descs[i].handle;
                            s_CurRange.start_handle++;
                            break;

                        case BLE_UUID_DESCRIPTOR_CHAR_EXT_PROP:
                            periph->Services[s_CurSrvcIdx].characteristics[s_CurCharIdx].ext_prop_handle = p_desc_disc_rsp_evt->descs[i].handle;
                            s_CurRange.start_handle++;
                            break;

                        case BLE_UUID_DESCRIPTOR_CHAR_USER_DESC:
                            periph->Services[s_CurSrvcIdx].characteristics[s_CurCharIdx].user_desc_handle = p_desc_disc_rsp_evt->descs[i].handle;
                            s_CurRange.start_handle++;
                            break;

                        case BLE_UUID_REPORT_REF_DESCR:
                            periph->Services[s_CurSrvcIdx].characteristics[s_CurCharIdx].report_ref_handle = p_desc_disc_rsp_evt->descs[i].handle;
                            s_CurRange.start_handle++;
                            break;

                        case BLE_UUID_CHARACTERISTIC:
                        	{
                        		s_CurRange.start_handle++;
                        	}
                        	break;
                        default:
                        	;
                    }
            	}
            }

            s_CurCharIdx++;

			while (s_CurCharIdx >= periph->Services[s_CurSrvcIdx].char_count)
			{
				s_CurSrvcIdx++;
				s_CurCharIdx = 0;
				if (s_CurSrvcIdx >= periph->NbSrvc)
				{
					BleDevDiscovered(periph);
					return;
				}
			}
			{
				s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;
				s_CurRange.start_handle = periph->Services[s_CurSrvcIdx].characteristics[s_CurCharIdx].characteristic.handle_value + 1;
				uint32_t err = sd_ble_gattc_descriptors_discover(periph->Conn.Hdl, &s_CurRange);
			}
            break;

        default:
            break;
    }

}

bool BtAppDiscoverDevice(BtDev_t * const pDev)
{
	s_pBlePeriphData = pDev;
	s_CurSrvcIdx = 0;
	s_CurCharIdx = 0;

    uint32_t err_code = sd_ble_gattc_primary_services_discover(pDev->Conn.Hdl, 1, NULL);

    return err_code == NRF_SUCCESS;
    //return err_code;
}
#endif // legacy discovery state machine

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
// nRF54L bm port, so this is the same state machine. Single active discovery
// at a time; the cursor lives in file-scope statics, not per peer.
// =====================================================================

static BtDevice_t *s_pDiscDev    = NULL;    // peer under discovery
static uint8_t     s_DiscSrvIdx  = 0;       // service cursor (char and desc phases)
static uint8_t     s_DiscCharIdx = 0;       // characteristic cursor (desc phase)

static void BtAppDiscStartChar(BtDevice_t *pDev);
static void BtAppDiscStartDesc(BtDevice_t *pDev);

bool BtAppDiscoverDevice(BtDev_t * const pDev)
{
	if (pDev == NULL)
	{
		return false;
	}

	s_pDiscDev    = pDev;
	s_DiscSrvIdx  = 0;
	s_DiscCharIdx = 0;

	pDev->NbSrvc = 0;
	memset(pDev->Services, 0, sizeof(BtGattDBSrvc_t) * BT_DEV_SERVICE_MAXCNT);

	// NULL uuid filter discovers every primary service from handle 1.
	return sd_ble_gattc_primary_services_discover(pDev->Conn.Hdl, 0x0001, NULL) == NRF_SUCCESS;
}

static void BtAppDiscStartChar(BtDevice_t *pDev)
{
	while (s_DiscSrvIdx < pDev->NbSrvc)
	{
		BtGattDBSrvc_t *pSrvc = &pDev->Services[s_DiscSrvIdx];
		ble_gattc_handle_range_t range;

		range.start_handle = pSrvc->handle_range.StartHdl + 1;  // skip service declaration
		range.end_handle   = pSrvc->handle_range.EndHdl;

		if (range.start_handle <= range.end_handle &&
		    sd_ble_gattc_characteristics_discover(pDev->Conn.Hdl, &range) == NRF_SUCCESS)
		{
			return;     // wait for BLE_GATTC_EVT_CHAR_DISC_RSP
		}
		s_DiscSrvIdx++; // empty service or request rejected; advance
	}

	// All services covered for characteristics. Resolve descriptors (CCCD).
	s_DiscSrvIdx  = 0;
	s_DiscCharIdx = 0;
	BtAppDiscStartDesc(pDev);
}

static void BtAppDiscStartDesc(BtDevice_t *pDev)
{
	while (s_DiscSrvIdx < pDev->NbSrvc)
	{
		BtGattDBSrvc_t *pSrvc = &pDev->Services[s_DiscSrvIdx];

		while (s_DiscCharIdx < pSrvc->char_count)
		{
			BtGattDBChar_t *pCh    = &pSrvc->characteristics[s_DiscCharIdx];
			uint16_t        valHdl = pCh->characteristic.handle_value;
			uint16_t        start  = valHdl + 1;
			uint16_t        end;

			if ((s_DiscCharIdx + 1) < pSrvc->char_count)
			{
				end = pSrvc->characteristics[s_DiscCharIdx + 1].characteristic.handle_decl - 1;
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
			s_DiscCharIdx++; // no descriptor range or request rejected
		}
		s_DiscSrvIdx++;
		s_DiscCharIdx = 0;
	}

	// Every service and characteristic processed. Discovery complete.
	BtDeviceDiscovered(pDev);
}

static void BtAppDiscPrimSrvcRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = s_pDiscDev;
	if (pDev == NULL)
	{
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

			BtGattDBSrvc_t *pSrvc = &pDev->Services[pDev->NbSrvc];
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
	s_DiscSrvIdx = 0;
	BtAppDiscStartChar(pDev);
}

static void BtAppDiscCharRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = s_pDiscDev;
	if (pDev == NULL)
	{
		return;
	}

	BtGattDBSrvc_t *pSrvc = &pDev->Services[s_DiscSrvIdx];

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
	s_DiscSrvIdx++;
	BtAppDiscStartChar(pDev);
}

static void BtAppDiscDescRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = s_pDiscDev;
	if (pDev == NULL)
	{
		return;
	}

	if (pEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
	{
		const ble_gattc_evt_desc_disc_rsp_t *p = &pEvt->params.desc_disc_rsp;
		BtGattDBSrvc_t *pSrvc = &pDev->Services[s_DiscSrvIdx];
		BtGattDBChar_t *pCh   = &pSrvc->characteristics[s_DiscCharIdx];

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
	s_DiscCharIdx++;
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

bool BtAppNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	uint16_t connHdl = BtAppGetConnHandle();
	uint8_t emptyData = 0;

	if (pChar == nullptr)
	{
		return false;
	}

	if (DataLen > 0 && pData == nullptr)
	{
		return false;
	}

	if (connHdl == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

	if (DataLen > 0 && BtGattCharSetValue(pChar, pData, DataLen) == false)
	{
		return false;
	}

	if (pChar->bNotify == false)
	{
		return false;
	}

	if (pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	if (pData == nullptr)
	{
		pData = &emptyData;
	}

    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = pChar->ValHdl;//.value_handle;
    params.p_data = pData;
    params.p_len = &DataLen;

    uint32_t err_code = sd_ble_gatts_hvx(connHdl, &params);

    return err_code == NRF_SUCCESS;
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
#if 1

void BtAppDisconnect()
{
	if (BtAppGetConnHandle() != BLE_CONN_HANDLE_INVALID)
    {
		uint32_t err_code = sd_ble_gap_disconnect(BtAppGetConnHandle(), BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
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
        err_code = sd_ble_gap_disconnect(BtAppGetConnHandle(), BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
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

				DEBUG_PRINTF("SEC: CONN_SEC_SUCCEEDED hdl=%d encrypted=%d mitm=%d bonded=%d lesc=%d proc=%d\r\n",
						p_evt->conn_handle,
						conn_sec_status.encrypted, conn_sec_status.mitm_protected,
						conn_sec_status.bonded, conn_sec_status.lesc,
						p_evt->params.conn_sec_succeeded.procedure);

				if (conn_sec_status.mitm_protected)
				{
				}
				else
				{
#if 0
					// The peer did not use MITM, disconnect.
					err_code = pm_peer_id_get(BtAppGetConnHandle(), &g_PeerMngrIdToDelete);
					APP_ERROR_CHECK(err_code);
					err_code = sd_ble_gap_disconnect(BtAppGetConnHandle(),
													 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					APP_ERROR_CHECK(err_code);
#endif
				}

				// Link is encrypted. Notify the app so it can run work that needs
				// an encrypted link (e.g. a central reading protected chars).
				BtAppEvtSecured(p_evt->conn_handle);
			}
			break;

        case PM_EVT_CONN_SEC_FAILED:
            DEBUG_PRINTF("SEC: CONN_SEC_FAILED hdl=%d procedure=%d error=0x%X\r\n",
            		p_evt->conn_handle,
            		p_evt->params.conn_sec_failed.procedure,
            		p_evt->params.conn_sec_failed.error);
            if (g_BtAppData.AppDevice.bSecure && BtAppGetConnHandle() != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(BtAppGetConnHandle(),
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
			{
				DEBUG_PRINTF("SEC: CONN_SEC_CONFIG_REQ hdl=%d (allow repairing)\r\n", p_evt->conn_handle);
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
            err_code = sd_ble_gap_adv_start(g_BtAppData.AdvHdl, BTAPP_CONN_CFG_TAG);

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
    uint32_t err_code;
	ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
	uint16_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);

	// Let the LESC module observe events (it handles the DHKey request and
	// replies to the SoftDevice; the heavy ECDH runs in nrf_ble_lesc_request_handler).
	nrf_ble_lesc_on_ble_evt(p_ble_evt);

	// Trace security-relevant GAP events to diagnose the pairing flow.
	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			DEBUG_PRINTF("SEC: EVT_SEC_PARAMS_REQUEST (pairing started by peer/PM)\r\n");
			break;
		case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
			DEBUG_PRINTF("SEC: EVT_LESC_DHKEY_REQUEST (LESC in progress)\r\n");
			break;
		case BLE_GAP_EVT_AUTH_KEY_REQUEST:
			DEBUG_PRINTF("SEC: EVT_AUTH_KEY_REQUEST type=%d (passkey/oob needed)\r\n",
					p_ble_evt->evt.gap_evt.params.auth_key_request.key_type);
			break;
		case BLE_GAP_EVT_AUTH_STATUS:
			DEBUG_PRINTF("SEC: EVT_AUTH_STATUS status=0x%X lesc=%d bonded=%d\r\n",
					p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
					p_ble_evt->evt.gap_evt.params.auth_status.lesc,
					p_ble_evt->evt.gap_evt.params.auth_status.bonded);
			break;
		case BLE_GAP_EVT_CONN_SEC_UPDATE:
			DEBUG_PRINTF("SEC: EVT_CONN_SEC_UPDATE sm=%d lv=%d\r\n",
					p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.sm,
					p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.lv);
			break;
		default:
			break;
	}

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	BtAppConnLedOn();
        	BtPeerConnected(p_gap_evt->conn_handle, role,
        					   p_gap_evt->params.connected.peer_addr.addr_type,
        					   (uint8_t*)p_gap_evt->params.connected.peer_addr.addr);
        	g_BtAppData.State = BTAPP_STATE_CONNECTED;

        	// If a secure SecType was configured, the SoftDevice backend requests
        	// security on the link itself (pm_conn_secure -> sd_ble_gap_authenticate).
        	// This is backend-internal so the application stays SDK-neutral - it
        	// does not call any backend-specific security-request function.
        	if (g_BtAppData.AppDevice.bSecure)
        	{
        		err_code = pm_conn_secure(p_gap_evt->conn_handle, false);
        		DEBUG_PRINTF("SEC: connect hdl=%d bSecure=1 pm_conn_secure=0x%X\r\n",
        				p_gap_evt->conn_handle, err_code);
        		if (err_code != NRF_ERROR_INVALID_STATE && err_code != NRF_ERROR_BUSY)
        		{
        			APP_ERROR_CHECK(err_code);
        		}
        	}
        	else
        	{
        		DEBUG_PRINTF("SEC: connect hdl=%d bSecure=0 (no security requested)\r\n",
        				p_gap_evt->conn_handle);
        	}

        	BtAppEvtConnected(p_ble_evt->evt.gap_evt.conn_handle);

        	break;
        case BLE_GAP_EVT_DISCONNECTED:
			{
				uint16_t connHdl = p_ble_evt->evt.gap_evt.conn_handle;
				BtDevice_t *pPeer = BtPeerFindByHdl(connHdl);

				BtAppConnLedOff();
				BtPeerFree(pPeer);

				if (BtPeerIsConnected() == false)
				{
					g_BtAppData.State = BTAPP_STATE_IDLE;
				}

				BtAppEvtDisconnected(connHdl);

				if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
				{
					BtAdvStart();
				}
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
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(BtAppGetConnHandle(), NULL, 0, 0);
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
    		case BLE_GAP_EVT_ADV_REPORT:
				{
					// Scan data report
					ble_gap_evt_adv_report_t * p_adv_report = (ble_gap_evt_adv_report_t*)&p_gap_evt->params.adv_report;

					bool res = BtAppScanReport(p_adv_report->rssi, p_adv_report->peer_addr.addr_type,
							p_adv_report->peer_addr.addr, p_adv_report->data.len, p_adv_report->data.p_data);
					// Continue scan
					if (res == true)
					{
						BtAppScan();
					}
					else
					{
						BtAppScanStop();
					}
				}
    			break;
            case BLE_GAP_EVT_TIMEOUT:
				{
					const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

					ble_gap_evt_timeout_t const * p_timeout = &p_gap_evt->params.timeout;

					if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
					{
						g_BtAppData.bScan = false;
						BtAppScanTimeoutHandler();
					}
				}
            break;

            case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
                BtAppDiscPrimSrvcRsp(&p_ble_evt->evt.gattc_evt);
                break;

            case BLE_GATTC_EVT_CHAR_DISC_RSP:
                BtAppDiscCharRsp(&p_ble_evt->evt.gattc_evt);
                break;

            case BLE_GATTC_EVT_DESC_DISC_RSP:
                BtAppDiscDescRsp(&p_ble_evt->evt.gattc_evt);
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
    	BtGattEvtHandler(p_ble_evt->header.evt_id, (void*)p_ble_evt);
    	BtAppPeriphEvtHandler(p_ble_evt->header.evt_id, (void*)p_ble_evt);
    }
#endif
}



/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void BtAppPeerMngrInit(BTGAP_SECTYPE SecType, uint8_t SecKeyExchg, bool bEraseBond)
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
    	case BTGAP_SECTYPE_NONE:
			break;
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
			break;
		case BTGAP_SECTYPE_STATICKEY_MITM:
			sec_param.mitm = 1;
			break;
		case BTGAP_SECTYPE_LESC_MITM:
		case BTGAP_SECTYPE_SIGNED_MITM:
			sec_param.mitm = 1;
		    sec_param.lesc = 1;
			break;
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
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
    int type = SecKeyExchg & (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY);
    switch (type)
    {
		case BTAPP_SECEXCHG_KEYBOARD:
			sec_param.keypress = 1;
			sec_param.io_caps  = BLE_GAP_IO_CAPS_KEYBOARD_ONLY;
			break;

		case BTAPP_SECEXCHG_DISPLAY:
			sec_param.io_caps  = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
    			break;
		case (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY):
			sec_param.keypress = 1;
			sec_param.io_caps  = BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY;
			break;
    }

    if (SecKeyExchg & BTAPP_SECEXCHG_OOB)
    {
    	sec_param.oob = 1;
//    	nfc_ble_pair_init(&g_AdvInstance, NFC_PAIRING_MODE_JUST_WORKS);
    }

    err_code = pm_sec_params_set(&sec_param);
    DEBUG_PRINTF("SEC: pm_sec_params_set=0x%X lesc=%d mitm=%d bond=%d io=%d\r\n",
    		err_code, sec_param.lesc, sec_param.mitm, sec_param.bond, sec_param.io_caps);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    DEBUG_PRINTF("SEC: pm_register=0x%X\r\n", err_code);
    APP_ERROR_CHECK(err_code);

    // Initialise the LESC module. This sets up nrf_crypto (mbedTLS backend on
    // this target provides the secp256r1 ECDH) and generates the local ECDH key
    // pair. The module owns the key pair, handles the LESC DHKey request, and
    // replies to the SoftDevice; the app only routes BLE events to
    // nrf_ble_lesc_on_ble_evt and pumps nrf_ble_lesc_request_handler in the loop.
    err_code = nrf_ble_lesc_init();
    DEBUG_PRINTF("SEC: nrf_ble_lesc_init=0x%X\r\n", err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void BtGattEvtHandler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t * p_evt)
{
    if ((BtAppGetConnHandle() == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
    	//g_BleAppData.MaxMtu = p_evt->params.att_mtu_effective - 3;//OPCODE_LENGTH - HANDLE_LENGTH;
       // m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Data len is set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
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
 * @param[in] CentLinkCount
 * @param[in] PeriLinkCount
 * @param[in] bConnectable
 */
bool BtAppStackInit(int MaxMtu, int CentLinkCount, int PeriLinkCount, bool bConnectable)
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
    //if (CentLinkCount > 0)
        ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 4;//BLESRVC_UUID_BASE_MAXCNT;
    //else
    //	ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 2;
    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum number of connections.
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                     = BTAPP_CONN_CFG_TAG;
	ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = PeriLinkCount;
	ble_cfg.gap_cfg.role_count_cfg.central_role_count = CentLinkCount;
	ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = CentLinkCount ? BLE_GAP_ROLE_COUNT_CENTRAL_SEC_DEFAULT: 0;
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

bool BtAppInit(const BtAppCfg_t *pCfg)//, bool bEraseBond)
{
	ret_code_t err_code;

	//g_BleAppData.State = BLEAPP_STATE_UNKNOWN;
	//g_BleAppData.bExtAdv = pBleAppCfg->bExtAdv;
	//g_BleAppData.bScan = false;
	//g_BleAppData.bAdvertising = false;
	//g_BleAppData.VendorId = pBleAppCfg->VendorID;
	g_BtAppData.AppDevice.Conn.Role = pCfg->Role;
	g_BtAppData.AdvHdl = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
	if (!BtPeerInit(pCfg->pPeerPoolMem, pCfg->PeerPoolMemSize))
	{
		return false;
	}

	// Split the long-write reassembly pool across the peer slots so each
	// link gets its own buffer (per Conn.pLongWrBuff).
	BtPeerLongWrInit(pCfg->pLongWrPoolMem, pCfg->LongWrPoolMemSize);

	// Connection pool removed: the peer manager (BtPeerInit above) owns
	// the single connection table now.
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

    app_timer_init();
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    if (AppEvtHandlerInit(pCfg->pEvtHandlerQueMem, pCfg->EvtHandlerQueMemSize) == false)
    {
    	return false;
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
    BtAppStackInit(g_BtAppData.AppDevice.Conn.MaxMtu, pCfg->CentLinkCount, pCfg->PeriLinkCount,
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

	BtGapInit(&gapcfg);

	if (pCfg->pDevName != NULL)
	{
		BtGapSetDevName(pCfg->pDevName);
	}

	conn_params_init();

	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtAppInitUserServices();
	}

	if (pCfg->pDevInfo != NULL)
	{
		BtDisInit(pCfg);
	}

    BtGattInit();

    BtAppInitUserData();

    BtAppPeerMngrInit(pCfg->SecType, pCfg->SecExchg, false);//bEraseBond);


    g_BtAppData.AppDevice.bSecure = pCfg->SecType != BTGAP_SECTYPE_NONE;

    if (g_BtAppData.AppDevice.Conn.Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_BROADCASTER))
    {
        if (BtAppAdvInit(pCfg) == false)
        {
        	return false;
        }

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

    return true;
}

void BtAppRun()
{
	if (g_BtAppData.State != BTAPP_STATE_INITIALIZED)
	{
		return;
	}

	//g_BleAppData.bAdvertising = false;
	//g_BleAppData.State = BLEAPP_STATE_IDLE;

	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))// != BLEAPP_ROLE_CENTRAL)
	{
		BtAdvStart();//BLEAPP_ADVMODE_FAST);
	}

	while (1)
    {
		app_sched_execute();
		AppEvtHandlerExec();
		nrf_ble_lesc_request_handler();
		BtAppEvtWait();
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


void BtAppEvtDispatch()
{
    nrf_sdh_evts_poll();                    /* let the handlers run first, incase the EVENT occured before creating this task */
}

// Port-level weak default for BtAppEvtWait. Bare-metal polling apps use this.
// RTOS apps provide a strong override (e.g. ulTaskNotifyTake / TaktOSSemTake)
// in their bridge code, which beats this weak.
__attribute__((weak)) void BtAppEvtWait(void)
{
	sd_app_evt_wait();
}

// Trampoline called from sd_dispatch.cpp's SD_EVT_IRQHandler.
// Notifies any RTOS waiter then drains SoftDevice events so NRF_SDH observers run.
static void BtAppSDDispatch(void)
{
	BtAppEvtNotify();
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
#endif

