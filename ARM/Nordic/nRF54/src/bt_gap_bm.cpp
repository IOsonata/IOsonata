/**-------------------------------------------------------------------------
@file	bt_gap_bm.cpp

@brief	Implement Bluetooth Generic Access Profile (GAP)

Implementation for Nordic baremetal sdk (sdk-nrf-bm) with SoftDevice S145
on nRF54L15.

Core Bluetooth Vol.1, Part A, 6.2

@author	Hoang Nguyen Hoan
@date	Feb. 25, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#include "convutil.h"
#include "bluetooth/bt_gap.h"
#include "bm_compat.h"

#define BT_GAP_CONN_CFG_TAG			CONFIG_NRF_SDH_BLE_CONN_TAG

static ble_gap_scan_params_t s_ScanParams = {
	.active = 1,
	.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
	.scan_phys = BLE_GAP_PHY_1MBPS,
	.interval = MSEC_TO_0_625(BT_GAP_SCAN_INTERVAL),
	.window = MSEC_TO_0_625(BT_GAP_SCAN_WINDOW),
	.timeout = BT_GAP_SCAN_TIMEOUT,
};

static ble_gap_conn_sec_mode_t s_GapConnMode;

static inline uint16_t mSecTo10Ms(uint32_t t)
{
	return (uint16_t)((t + 5U) / 10U);
}

static void BtGapSetSecMode(ble_gap_conn_sec_mode_t *pSecMode, BTGAP_SECTYPE SecType)
{
	switch (SecType)
	{
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(pSecMode);
			break;
		case BTGAP_SECTYPE_STATICKEY_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(pSecMode);
			break;
		case BTGAP_SECTYPE_LESC_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(pSecMode);
			break;
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
		case BTGAP_SECTYPE_SIGNED_MITM:
			// Signed-only security mode is not used for GAP device name in this port.
			BLE_GAP_CONN_SEC_MODE_SET_OPEN(pSecMode);
			break;
		case BTGAP_SECTYPE_NONE:
		default:
			BLE_GAP_CONN_SEC_MODE_SET_OPEN(pSecMode);
			break;
	}
}

void BtGapParamInit(const BtGapCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return;
	}

	BtGapSetSecMode(&s_GapConnMode, (BTGAP_SECTYPE)pCfg->SecType);

	if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
	{
		ble_gap_conn_params_t gap_conn_params;
		memset(&gap_conn_params, 0, sizeof(gap_conn_params));

		gap_conn_params.min_conn_interval = mSecTo1_25(pCfg->ConnIntervalMin);
		gap_conn_params.max_conn_interval = mSecTo1_25(pCfg->ConnIntervalMax);
		gap_conn_params.slave_latency = BT_GAP_CONN_SLAVE_LATENCY;
		gap_conn_params.conn_sup_timeout = mSecTo10Ms(BT_GAP_CONN_SUP_TIMEOUT);

		(void)sd_ble_gap_ppcp_set(&gap_conn_params);
	}
}

void BtGapSetDevName(const char *pDeviceName)
{
	if (pDeviceName == nullptr)
	{
		return;
	}

	(void)sd_ble_gap_device_name_set(&s_GapConnMode,
									(const uint8_t *)pDeviceName,
									(uint16_t)strlen(pDeviceName));
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)
{
	if (pPeerAddr == nullptr || pConnParam == nullptr)
	{
		return false;
	}

	ble_gap_addr_t addr;
	memset(&addr, 0, sizeof(addr));
	addr.addr_id_peer = 0;
	addr.addr_type = pPeerAddr->Type;
	memcpy(addr.addr, pPeerAddr->Addr, sizeof(addr.addr));

	ble_gap_conn_params_t cparam;
	memset(&cparam, 0, sizeof(cparam));
	cparam.min_conn_interval = mSecTo1_25(pConnParam->IntervalMin);
	cparam.max_conn_interval = mSecTo1_25(pConnParam->IntervalMax);
	cparam.slave_latency = pConnParam->Latency;
	cparam.conn_sup_timeout = mSecTo10Ms(pConnParam->Timeout);

	uint32_t err_code = sd_ble_gap_connect(&addr, &s_ScanParams, &cparam,
										   BT_GAP_CONN_CFG_TAG);

	return err_code == NRF_SUCCESS;
}

static bool IsAllZero(const uint8_t *pData, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		if (pData[i] != 0)
		{
			return false;
		}
	}

	return true;
}

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	s_ScanParams.active = (pCfg->Type == BTSCAN_TYPE_ACTIVE) ? 1 : 0;
	s_ScanParams.interval = mSecTo0_625((float)pCfg->Param.Interval);
	s_ScanParams.window = mSecTo0_625((float)pCfg->Param.Duration);
	s_ScanParams.timeout = (uint16_t)pCfg->Param.Timeout;
	s_ScanParams.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
	s_ScanParams.scan_phys = BLE_GAP_PHY_1MBPS;

	if (pCfg->Param.Phy & BT_GAP_PHY_CODED)
	{
#ifdef BLE_GAP_PHY_CODED
		s_ScanParams.scan_phys = BLE_GAP_PHY_CODED;
#endif
	}
	else if (pCfg->Param.Phy & BT_GAP_PHY_2MBITS)
	{
#ifdef BLE_GAP_PHY_2MBPS
		s_ScanParams.scan_phys = BLE_GAP_PHY_2MBPS;
#else
		s_ScanParams.scan_phys = BLE_GAP_PHY_1MBPS;
#endif
	}

	if (!IsAllZero(pCfg->BaseUid, sizeof(pCfg->BaseUid)))
	{
		uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;
		ble_uuid128_t uid;
		memcpy(uid.uuid128, pCfg->BaseUid, sizeof(uid.uuid128));
		(void)sd_ble_uuid_vs_add(&uid, &uidtype);
	}

	return true;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	if (pBuff == nullptr || Len == 0)
	{
		return false;
	}

	ble_data_t repdata;
	repdata.p_data = pBuff;
	repdata.len = Len;

	uint32_t err_code = sd_ble_gap_scan_start(&s_ScanParams, &repdata);

	return err_code == NRF_SUCCESS;
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
	if (pBuff == nullptr || Len == 0)
	{
		return false;
	}

	ble_data_t repdata;
	repdata.p_data = pBuff;
	repdata.len = Len;

	uint32_t err_code = sd_ble_gap_scan_start(nullptr, &repdata);

	return err_code == NRF_SUCCESS;
}

void BtGapScanStop()
{
	(void)sd_ble_gap_scan_stop();
}
