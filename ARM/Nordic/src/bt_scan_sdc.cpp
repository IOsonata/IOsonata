/**-------------------------------------------------------------------------
@file	bt_scan_sdc.cpp

@brief	SoftDevice Controller (nrfxlib SDC) scanning functions.

        Extracted from bt_app_sdc.cpp in step 4b of the Voci refactor.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "sdc.h"
#include "sdc_soc.h"
#include "sdc_hci_cmd_le.h"

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"

void BtAppScan()
{
#if 0
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
#endif

	BtGapScanStart(NULL, 0);

}

void BtAppScanStop()
{
	if (g_BtAppData.bScan == true)
	{
		//ret_code_t err_code = sd_ble_gap_scan_stop();
		//APP_ERROR_CHECK(err_code);
		g_BtAppData.bScan = false;
	}
}
#if 1
bool BtAppScanInit(BtGapScanCfg_t *pCfg)
{
	sdc_hci_cmd_le_set_scan_params_t paraam;

	BtGapScanInit(pCfg);

	return false;
#if 0
	if (pCfg == NULL)
	{
		return false;
	}

	s_BleScanParams.timeout = pCfg->Timeout;
	s_BleScanParams.window = pCfg->Duration;
	s_BleScanParams.interval = pCfg->Interval;

    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ret_code_t err_code = sd_ble_uuid_vs_add(&pCfg->BaseUid, &uidtype);
    APP_ERROR_CHECK(err_code);

    g_BleAppData.bScan = true;

	err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
#endif
}
#endif
