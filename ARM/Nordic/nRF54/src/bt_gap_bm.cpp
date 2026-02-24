/**-------------------------------------------------------------------------
@file	bt_gap_bm.cpp

@brief	Implement Bluetooth Generic Access Profile (GAP) using Nordic baremetal SDK (S145)

Core Bluetooth Vol.1, Part A, 6.2

@author	Hoang Nguyen Hoan
@date	Feb. 24, 2026

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
#include "ble_srv_common.h"

#include "bluetooth/bt_gap.h"

#ifndef UNIT_0_625_MS
#define UNIT_0_625_MS                          625
#endif

#ifndef UNIT_1_25_MS
#define UNIT_1_25_MS                           1250
#endif

#ifndef UNIT_10_MS
#define UNIT_10_MS                             10000
#endif

#ifndef MSEC_TO_UNITS
#define MSEC_TO_UNITS(MS, UNIT)                (((MS) * 1000) / (UNIT))
#endif

#if defined(CONFIG_NRF_SDH_BLE_CONN_TAG)
#define BT_GAP_CONN_CFG_TAG                    CONFIG_NRF_SDH_BLE_CONN_TAG
#else
#define BT_GAP_CONN_CFG_TAG                    1
#endif

static ble_gap_scan_params_t s_ScanParams =
{
#if (NRF_SD_BLE_API_VERSION >= 6)
    .extended = 1,
    .report_incomplete_evts = 0,
    .active = 1,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .scan_phys = BLE_GAP_PHY_1MBPS,
#elif (NRF_SD_BLE_API_VERSION >= 3)
    .active = 1,
    .use_whitelist = 0,
    .interval = MSEC_TO_UNITS(BT_GAP_SCAN_INTERVAL, UNIT_0_625_MS),
    .window = MSEC_TO_UNITS(BT_GAP_SCAN_WINDOW, UNIT_0_625_MS),
    .timeout = BT_GAP_SCAN_TIMEOUT,
    .adv_dir_report = 1,
#else
    .active = 1,
    .selective = 0,
    .p_whitelist = NULL,
    .interval = MSEC_TO_UNITS(BT_GAP_SCAN_INTERVAL, UNIT_0_625_MS),
    .window = MSEC_TO_UNITS(BT_GAP_SCAN_WINDOW, UNIT_0_625_MS),
    .timeout = BT_GAP_SCAN_TIMEOUT,
#endif
};

static ble_gap_conn_sec_mode_t s_GapConnMode;

void BtGapParamInit(const BtGapCfg_t *pCfg)
{
    if (pCfg == nullptr)
    {
        return;
    }

    switch (pCfg->SecType)
    {
        case BTGAP_SECTYPE_NONE:
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&s_GapConnMode);
            break;
        case BTGAP_SECTYPE_STATICKEY_NO_MITM:
            BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&s_GapConnMode);
            break;
        case BTGAP_SECTYPE_STATICKEY_MITM:
            BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&s_GapConnMode);
            break;
        case BTGAP_SECTYPE_LESC_MITM:
            BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&s_GapConnMode);
            break;
        case BTGAP_SECTYPE_SIGNED_NO_MITM:
            BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(&s_GapConnMode);
            break;
        case BTGAP_SECTYPE_SIGNED_MITM:
            BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(&s_GapConnMode);
            break;
        default:
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&s_GapConnMode);
            break;
    }

    if (pCfg->Role & BT_GAP_ROLE_PERIPHERAL)
    {
        ble_gap_conn_params_t gap_conn_params = {};

        gap_conn_params.min_conn_interval = MSEC_TO_UNITS(pCfg->ConnIntervalMin, UNIT_1_25_MS);
        gap_conn_params.max_conn_interval = MSEC_TO_UNITS(pCfg->ConnIntervalMax, UNIT_1_25_MS);
        gap_conn_params.slave_latency = BT_GAP_CONN_SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout = MSEC_TO_UNITS(BT_GAP_CONN_SUP_TIMEOUT, UNIT_10_MS);

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
                                     strlen(pDeviceName));
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)
{
    if (pPeerAddr == nullptr || pConnParam == nullptr)
    {
        return false;
    }

    ble_gap_conn_params_t cparam = {};
    ble_gap_addr_t addr = {
        .addr_id_peer = 0,
        .addr_type = pPeerAddr->Type,
    };

    memcpy(addr.addr, pPeerAddr->Addr, sizeof(addr.addr));

    cparam.min_conn_interval = MSEC_TO_UNITS(pConnParam->IntervalMin, UNIT_1_25_MS);
    cparam.max_conn_interval = MSEC_TO_UNITS(pConnParam->IntervalMax, UNIT_1_25_MS);
    cparam.slave_latency = pConnParam->Latency;
    cparam.conn_sup_timeout = MSEC_TO_UNITS(pConnParam->Timeout, UNIT_10_MS);

    ret_code_t err_code = sd_ble_gap_connect(&addr, &s_ScanParams, &cparam, BT_GAP_CONN_CFG_TAG);

    return err_code == NRF_SUCCESS;
}

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
    if (pCfg == nullptr)
    {
        return false;
    }

#if (NRF_SD_BLE_API_VERSION >= 6)
    s_ScanParams.active = (pCfg->Type == BTSCAN_TYPE_ACTIVE) ? 1 : 0;
    s_ScanParams.scan_phys = pCfg->Param.Phy;
#else
    s_ScanParams.active = (pCfg->Type == BTSCAN_TYPE_ACTIVE) ? 1 : 0;
#endif

    s_ScanParams.timeout = pCfg->Param.Timeout;
    s_ScanParams.window = MSEC_TO_UNITS(pCfg->Param.Duration, UNIT_0_625_MS);
    s_ScanParams.interval = MSEC_TO_UNITS(pCfg->Param.Interval, UNIT_0_625_MS);

    uint8_t uuid_type = BLE_UUID_TYPE_VENDOR_BEGIN;
    ble_uuid128_t uuid;

    memcpy(uuid.uuid128, pCfg->BaseUid, sizeof(uuid.uuid128));

    ret_code_t err_code = sd_ble_uuid_vs_add(&uuid, &uuid_type);

    return err_code == NRF_SUCCESS;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
    static ble_data_t rep_data;

    rep_data.len = Len;
    rep_data.p_data = pBuff;

    uint32_t err_code = sd_ble_gap_scan_start(&s_ScanParams, &rep_data);

    return err_code == NRF_SUCCESS;
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
    static ble_data_t rep_data;

    rep_data.len = Len;
    rep_data.p_data = pBuff;

    uint32_t err_code = sd_ble_gap_scan_start(nullptr, &rep_data);

    return err_code == NRF_SUCCESS;
}

void BtGapScanStop(void)
{
    (void)sd_ble_gap_scan_stop();
}
