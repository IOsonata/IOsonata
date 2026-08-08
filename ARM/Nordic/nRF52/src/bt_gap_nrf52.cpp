/**-------------------------------------------------------------------------
 * @file	bt_gatt_nrf52.cpp
 *
 * @brief	Implement Bluetooth Generic Access Profile (GAP) using Nordic nRF5_SDK
 *
 * Core Bluetooth Vol.1, Part A, 6.2
 *
 * The Bluetooth system defines a base profile which all Bluetooth devices implement.
 * This profile is the Generic Access Profile (GAP), which defines the basic
 * requirements of a Bluetooth device. For instance, for BR/EDR, it defines a
 * Bluetooth device to include the Radio, Baseband, Link Manager, L2CAP, and the
 * Service Discovery protocol functionality; for LE, it defines the Physical Layer,
 * Link Layer, L2CAP, Security Manager, Attribute Protocol and Generic Attribute Profile.
 * This ties all the various layers together to form the basic requirements for a
 * Bluetooth device. It also describes the behaviors and methods for device discovery,
 * connection establishment, security, authentication, association models and
 * service discovery.
 *
 * This implementation is to be used with Nordic nRF5_SDK
 *
 * @author	Hoang Nguyen Hoan
 * @date	Mar. 25, 2014
 *
 * @license
 *
 * MIT License
 *
 * Copyright (c) 2014-2020 I-SYST inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "app_error.h"
#include "ble_srv_common.h"
#include "ble_gap.h"
#include "peer_manager.h"

#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_peer.h"

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

// Device-name write permission. Default to OPEN (sm=1, lv=1) so it is valid
// for roles that do not run BtGapParamInit (e.g. broadcaster). BtGapParamInit
// overrides it for secured peripheral/central. A zeroed value is NO_ACCESS,
// which makes sd_ble_gap_device_name_set fail.
static ble_gap_conn_sec_mode_t s_gap_conn_mode = { 1, 1 };

void BtGapParamInit(const BtGapCfg_t *pCfg)
{
	uint32_t              err_code;
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
		default:
			BLE_GAP_CONN_SEC_MODE_SET_OPEN(&s_gap_conn_mode);
			break;
	}

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
	{
		// Connection interval is float msec; divide by the 1.25ms BLE unit
		// directly. Avoids C++23 warning about MSEC_TO_UNITS macro mixing
		// float with the Nordic SDK's unnamed enum for UNIT_1_25_MS.
		gap_conn_params.min_conn_interval = (uint16_t)(pCfg->ConnIntervalMin / 1.25f);
		gap_conn_params.max_conn_interval = (uint16_t)(pCfg->ConnIntervalMax / 1.25f);
		gap_conn_params.slave_latency     = BT_GAP_CONN_SLAVE_LATENCY;
		gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(BT_GAP_CONN_SUP_TIMEOUT, UNIT_10_MS);

		err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
		APP_ERROR_CHECK(err_code);
	}
}

void BtGapSetDevName(const char* pDeviceName)
{
	uint32_t err_code;

	err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
	                                      (const uint8_t *)pDeviceName,
	                                      strlen(pDeviceName));
	APP_ERROR_CHECK(err_code);
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)
{
	ble_gap_conn_params_t cparam;
	ble_gap_addr_t addr = { .addr_id_peer = 0, .addr_type = pPeerAddr->Type, };

	memcpy(addr.addr, pPeerAddr->Addr, 6);

	cparam.min_conn_interval = (uint16_t)(pConnParam->IntervalMin / 1.25f);
	cparam.max_conn_interval = (uint16_t)(pConnParam->IntervalMax / 1.25f);
	cparam.slave_latency     = pConnParam->Latency;
	cparam.conn_sup_timeout  = MSEC_TO_UNITS(pConnParam->Timeout, UNIT_10_MS);

	ret_code_t err_code = sd_ble_gap_connect(&addr, &s_ScanParams, &cparam, BT_GAP_CONN_CFG_TAG);

	return err_code == NRF_SUCCESS;
}

static bool BtGapNrf52ScanParamsConvert(uint32_t IntervalMs,
	uint32_t WindowMs, uint16_t *pInterval, uint16_t *pWindow)
{
	uint64_t interval = ((uint64_t)IntervalMs * 1600U) / 1000U;
	uint64_t window = ((uint64_t)WindowMs * 1600U) / 1000U;
	if (pInterval == nullptr || pWindow == nullptr || interval < 0x0004U ||
		interval > 0x4000U || window < 0x0004U || window > 0x4000U ||
		window > interval)
	{
		return false;
	}

	*pInterval = (uint16_t)interval;
	*pWindow = (uint16_t)window;
	return true;
}

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
	if (pCfg == NULL ||
		(pCfg->Param.Phy != BT_GAP_PHY_1MBITS &&
		 pCfg->Param.Phy != BT_GAP_PHY_CODED) ||
		pCfg->Param.Timeout > 655U)
	{
		return false;
	}

	uint16_t interval;
	uint16_t window;
	if (!BtGapNrf52ScanParamsConvert(pCfg->Param.Interval,
		pCfg->Param.Duration, &interval, &window))
	{
		return false;
	}

	s_ScanParams.active = (pCfg->Type == BTSCAN_TYPE_ACTIVE) ? 1 : 0;
	s_ScanParams.timeout = (uint16_t)(pCfg->Param.Timeout * 100U);
	s_ScanParams.window = window;
	s_ScanParams.interval = interval;

#if (NRF_SD_BLE_API_VERSION >= 6)
	if (pCfg->Param.Phy == BT_GAP_PHY_CODED)
	{
#ifdef BLE_GAP_PHY_CODED
		s_ScanParams.scan_phys = BLE_GAP_PHY_CODED;
#else
		return false;
#endif
	}
	else
	{
		s_ScanParams.scan_phys = BLE_GAP_PHY_1MBPS;
	}
#else
	if (pCfg->Param.Phy != BT_GAP_PHY_1MBITS)
	{
		return false;
	}
#endif

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

	repdata.len    = Len;
	repdata.p_data = pBuff;

	uint32_t err_code = sd_ble_gap_scan_start(&s_ScanParams, &repdata);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
	static ble_data_t repdata;

	repdata.len    = Len;
	repdata.p_data = pBuff;

	uint32_t err_code = sd_ble_gap_scan_start(nullptr, &repdata);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}

void BtGapScanStop()
{
	sd_ble_gap_scan_stop();
}

// Strong override of the weak generic BtGapConnSecGet. On nRF52 the SoftDevice
// and Peer Manager own link security, so map their live status into BtConnSec_t.
// Level and flags come from pm_conn_sec_status_get; KeySize is the negotiated
// encryption key size captured at BLE_GAP_EVT_CONN_SEC_UPDATE.
bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	if (pSec == nullptr)
	{
		return false;
	}

	BtDevice_t *p = BtPeerFindByHdl(ConnHdl);
	if (p == nullptr)
	{
		return false;
	}

	pm_conn_sec_status_t st;
	if (pm_conn_sec_status_get(ConnHdl, &st) != NRF_SUCCESS)
	{
		return false;
	}

	pSec->Level = BT_GAP_SEC_LEVEL_NONE;
	pSec->KeySize = 0;
	pSec->Flags = 0;

	if (st.encrypted)
	{
		if (st.mitm_protected)
		{
			pSec->Level = st.lesc ? BT_GAP_SEC_LEVEL_LESC_AUTH : BT_GAP_SEC_LEVEL_ENC_AUTH;
		}
		else
		{
			pSec->Level = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
		}

		// Real negotiated size. 0 here only if the update event has not run,
		// which keeps the key-size gate on the safe (reject) side.
		pSec->KeySize = p->Conn.Sec.KeySize;
	}

	if (st.bonded)
	{
		pSec->Flags |= BT_GAP_SEC_FLAG_BONDED;
	}

	if (st.lesc)
	{
		pSec->Flags |= BT_GAP_SEC_FLAG_SC;
	}

	return true;
}
