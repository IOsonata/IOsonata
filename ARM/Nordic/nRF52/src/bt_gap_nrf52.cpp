/**-------------------------------------------------------------------------
@file	bt_gatt_nrf52.cpp

@brief	Implement Bluetooth Generic Access Profile (GAP) using Nordic nRF5_SDK

Core Bluetooth Vol.1, Part A, 6.2

The Bluetooth system defines a base profile which all Bluetooth devices implement.
This profile is the Generic Access Profile (GAP), which defines the basic
requirements of a Bluetooth device. For instance, for BR/EDR, it defines a
Bluetooth device to include the Radio, Baseband, Link Manager, L2CAP, and the
Service Discovery protocol functionality; for LE, it defines the Physical Layer,
Link Layer, L2CAP, Security Manager, Attribute Protocol and Generic Attribute Profile.
This ties all the various layers together to form the basic requirements for a
Bluetooth device. It also describes the behaviors and methods for device discovery,
connection establishment, security, authentication, association models and
service discovery.

This implementation is to be used with Nordic nRF5_SDK

@author	Hoang Nguyen Hoan
@date	Mar. 25, 2014

@license

MIT License

Copyright (c) 2014-2020 I-SYST inc. All rights reserved.

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
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "app_error.h"
#include "ble_srv_common.h"
#include "ble_gap.h"

#include "bluetooth/bt_gap.h"

#define BT_GAP_CONN_CFG_TAG            1     /**< A tag identifying the SoftDevice BLE configuration. */

static ble_gap_scan_params_t s_ScanParams =
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
	MSEC_TO_UNITS(BT_GAP_SCAN_INTERVAL, UNIT_0_625_MS),	// Scan interval
	MSEC_TO_UNITS(BT_GAP_SCAN_WINDOW, UNIT_0_625_MS),	// Scan window
	BT_GAP_SCAN_TIMEOUT,	// Scan timeout
};

static ble_gap_conn_sec_mode_t s_gap_conn_mode;

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */

void BtGapParamInit(const BtGapCfg_t *pCfg)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;

    switch (pCfg->SecType)
    {
    	case BTGAP_SECTYPE_NONE:
    	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&s_gap_conn_mode);
    	    break;
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
		    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&s_gap_conn_mode);
    	    break;
		case BTGAP_SECTYPE_STATICKEY_MITM:
		    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&s_gap_conn_mode);
    	    break;
		case BTGAP_SECTYPE_LESC_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&s_gap_conn_mode);
    	    break;
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(&s_gap_conn_mode);
    	    break;
		case BTGAP_SECTYPE_SIGNED_MITM:
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
    if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
    {
		gap_conn_params.min_conn_interval = MSEC_TO_UNITS(pCfg->ConnIntervalMin, UNIT_1_25_MS);// MIN_CONN_INTERVAL;
		gap_conn_params.max_conn_interval = MSEC_TO_UNITS(pCfg->ConnIntervalMax, UNIT_1_25_MS);//MAX_CONN_INTERVAL;
		gap_conn_params.slave_latency     = BT_GAP_CONN_SLAVE_LATENCY;
		gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(BT_GAP_CONN_SUP_TIMEOUT, UNIT_10_MS);

		err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
		APP_ERROR_CHECK(err_code);
    }

   // BtGapInit(pBleAppCfg->Role);
}

void BtGapSetDevName(const char* pDeviceName)
{
    uint32_t                err_code;

    err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
                                          (const uint8_t *)pDeviceName,
                                          strlen( pDeviceName ));
    APP_ERROR_CHECK(err_code);
    //ble_advertising_restart_without_whitelist(&g_AdvInstance);
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)//, BtGapScanParam_t * const pScanParam)
{
	ble_gap_scan_params_t scparam = {};
	ble_gap_conn_params_t cparam;
	ble_gap_addr_t addr = { .addr_id_peer = 0, .addr_type = pPeerAddr->Type, };

	memcpy(addr.addr,  pPeerAddr->Addr, 6);

	cparam.min_conn_interval = MSEC_TO_UNITS(pConnParam->IntervalMin, UNIT_1_25_MS);
	cparam.max_conn_interval = MSEC_TO_UNITS(pConnParam->IntervalMax, UNIT_1_25_MS);
	cparam.slave_latency = pConnParam->Latency;
	cparam.conn_sup_timeout = MSEC_TO_UNITS(pConnParam->Timeout, UNIT_10_MS);


	ret_code_t err_code = sd_ble_gap_connect(&addr, &s_ScanParams, &cparam,
											 BT_GAP_CONN_CFG_TAG);
    //APP_ERROR_CHECK(err_code);

//    s_BtAppData.bScan = false;

    return err_code == NRF_SUCCESS;
}

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
	if (pCfg == NULL)
	{
		return false;
	}

	s_ScanParams.timeout = pCfg->Param.Timeout;
	s_ScanParams.window = pCfg->Param.Duration;
	s_ScanParams.interval = pCfg->Param.Interval;

    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ble_uuid128_t uid;

    memcpy(uid.uuid128, pCfg->BaseUid, 16);

    ret_code_t err_code = sd_ble_uuid_vs_add(&uid, &uidtype);
    APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	static ble_data_t repdata;

	repdata.len = Len;
	repdata.p_data = pBuff;

	uint32_t err_code = sd_ble_gap_scan_start(&s_ScanParams, &repdata);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
	static ble_data_t repdata;

	repdata.len = Len;
	repdata.p_data = pBuff;

	uint32_t err_code = sd_ble_gap_scan_start(nullptr, &repdata);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}

void BtGapScanStop()
{
	sd_ble_gap_scan_stop();
}
