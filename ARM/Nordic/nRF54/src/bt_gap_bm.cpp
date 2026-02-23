/**-------------------------------------------------------------------------
@file	bt_gap_bm.cpp

@brief	Implement Bluetooth Generic Access Profile (GAP) using Nordic baremetal SDK (S145)

Core Bluetooth Vol.1, Part A, 6.2

This implementation targets nRF54L15 with SoftDevice S145 via sdk-nrf-bm.
It mirrors the legacy nRF5_SDK GAP implementation while using the same
sd_ble_gap_* SVC API.

@author	Hoang Nguyen Hoan
@date	Feb. 17, 2026

@license

MIT License

Copyright (c) 2014-2026 I-SYST inc. All rights reserved.

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

#include "nrf_error.h"
#include "ble.h"
#include "ble_gap.h"

#include "bm/softdevice_handler/nrf_sdh_ble.h"

#include "bluetooth/bt_gap.h"

/// Unit conversion macros (same as old nRF5_SDK definitions)
#define UNIT_0_625_MS					625			/**< Number of microseconds in 0.625 milliseconds. */
#define UNIT_1_25_MS					1250		/**< Number of microseconds in 1.25 milliseconds. */
#define UNIT_10_MS						10000		/**< Number of microseconds in 10 milliseconds. */
#define MSEC_TO_UNITS(MS, UNIT)			(((MS) * 1000) / (UNIT))

#define BT_GAP_CONN_CFG_TAG			CONFIG_NRF_SDH_BLE_CONN_TAG

static ble_gap_scan_params_t s_ScanParams;
static ble_gap_conn_sec_mode_t s_gap_conn_mode;

static inline void BtGapScanParamsDefault()
{
	memset(&s_ScanParams, 0, sizeof(s_ScanParams));
	s_ScanParams.active = 1;
#if defined(BLE_GAP_SCAN_FP_ACCEPT_ALL)
	s_ScanParams.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
#endif
#if defined(BLE_GAP_PHY_1MBPS)
	s_ScanParams.scan_phys = BLE_GAP_PHY_1MBPS;
#endif
	s_ScanParams.interval = MSEC_TO_UNITS(BT_GAP_SCAN_INTERVAL, UNIT_0_625_MS);
	s_ScanParams.window   = MSEC_TO_UNITS(BT_GAP_SCAN_WINDOW, UNIT_0_625_MS);
	s_ScanParams.timeout  = BT_GAP_SCAN_TIMEOUT;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
void BtGapParamInit(const BtGapCfg_t *pCfg)
{
	uint32_t err_code;
	ble_gap_conn_params_t gap_conn_params;

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

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
	{
		gap_conn_params.min_conn_interval = MSEC_TO_UNITS(pCfg->ConnIntervalMin, UNIT_1_25_MS);
		gap_conn_params.max_conn_interval = MSEC_TO_UNITS(pCfg->ConnIntervalMax, UNIT_1_25_MS);
		gap_conn_params.slave_latency     = pCfg->SlaveLatency;
		gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(pCfg->SupTimeout, UNIT_10_MS);

		err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
		(void)err_code;
	}

	// Default scan params
	BtGapScanParamsDefault();
}

void BtGapSetDevName(const char* pDeviceName)
{
	uint32_t err_code;

	err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
										  (const uint8_t *)pDeviceName,
										  strlen(pDeviceName));
	(void)err_code;
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)
{
	ble_gap_conn_params_t cparam;
	ble_gap_addr_t addr = { .addr_id_peer = 0, .addr_type = pPeerAddr->Type };

	memcpy(addr.addr, pPeerAddr->Addr, 6);

	cparam.min_conn_interval = MSEC_TO_UNITS(pConnParam->IntervalMin, UNIT_1_25_MS);
	cparam.max_conn_interval = MSEC_TO_UNITS(pConnParam->IntervalMax, UNIT_1_25_MS);
	cparam.slave_latency     = pConnParam->Latency;
	cparam.conn_sup_timeout  = MSEC_TO_UNITS(pConnParam->Timeout, UNIT_10_MS);

	ret_code_t err_code = sd_ble_gap_connect(&addr, &s_ScanParams, &cparam,
									 BT_GAP_CONN_CFG_TAG);

	return err_code == NRF_SUCCESS;
}

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
	if (pCfg == NULL)
	{
		return false;
	}

	// Convert parameters are passed in the same units as the legacy implementation
	BtGapScanParamsDefault();
	s_ScanParams.timeout  = pCfg->Param.Timeout;
	s_ScanParams.window   = pCfg->Param.Duration;
	s_ScanParams.interval = pCfg->Param.Interval;

	uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;
	ble_uuid128_t uid;

	memcpy(uid.uuid128, pCfg->BaseUid, 16);

	ret_code_t err_code = sd_ble_uuid_vs_add(&uid, &uidtype);

	return err_code == NRF_SUCCESS;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	static ble_data_t repdata;

	repdata.len = Len;
	repdata.p_data = pBuff;

	uint32_t err_code = sd_ble_gap_scan_start(&s_ScanParams, &repdata);

	return err_code == NRF_SUCCESS;
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
	static ble_data_t repdata;

	repdata.len = Len;
	repdata.p_data = pBuff;

	uint32_t err_code = sd_ble_gap_scan_start(nullptr, &repdata);

	return err_code == NRF_SUCCESS;
}

void BtGapScanStop()
{
	sd_ble_gap_scan_stop();
}
