/**-------------------------------------------------------------------------
@file	bt_scan_nrf52.cpp

@brief	nRF5_SDK SoftDevice scanning functions.

        Extracted from bt_app_nrf52.cpp in step 4b of the Voci refactor.
        Owns the scan buffer, the SoftDevice scan-report data struct, and
        the BtAppScan* function family.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2016, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "ble.h"
#include "ble_gap.h"
#include "nrf_ble_scan.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_error.h"

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"

// --- Scan buffer and SoftDevice scan-report data ---

NRF_BLE_SCAN_DEF(g_Scan);

static uint8_t g_BleScanBuff[BLE_GAP_SCAN_BUFFER_EXTENDED_MAX];

static ble_data_t g_BleScanReportData = {
	.p_data = g_BleScanBuff,
	.len = BLE_GAP_SCAN_BUFFER_EXTENDED_MAX
};

void BtAppScan(void)
{
	if (g_BtAppData.bScan == true)
	{
		//err_code = sd_ble_gap_scan_start(NULL, &g_BleScanReportData);
		BtGapScanNext(g_BleScanReportData.p_data, g_BleScanReportData.len);
	}
	else
	{
		g_BtAppData.bScan = true;

//		err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
		BtGapScanStart(g_BleScanReportData.p_data, g_BleScanReportData.len);
	}
}

void BtAppScanStop(void)
{
	BtGapScanStop();

	g_BtAppData.bScan = false;
}

bool BtAppScanInit(BtGapScanCfg_t *pCfg)
{
	if (BtGapScanInit(pCfg) == false)
	{
		return false;
	}
/*
	s_BleScanParams.timeout = pCfg->Timeout;
	s_BleScanParams.window = pCfg->Duration;
	s_BleScanParams.interval = pCfg->Interval;

    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ble_uuid128_t uid;

    memcpy(uid.uuid128, pCfg->BaseUid, 16);

    ret_code_t err_code = sd_ble_uuid_vs_add(&uid, &uidtype);
    APP_ERROR_CHECK(err_code);
*/
    g_BtAppData.bScan = true;

//	err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
//	APP_ERROR_CHECK(err_code);

//	return err_code == NRF_SUCCESS;
    return BtGapScanStart(g_BleScanReportData.p_data, g_BleScanReportData.len);
}
