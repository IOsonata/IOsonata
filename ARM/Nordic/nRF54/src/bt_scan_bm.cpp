/**-------------------------------------------------------------------------
@file	bt_scan_bm.cpp

@brief	sdk-nrf-bm SoftDevice (S145) scanning functions.

        Extracted from bt_app_bm.cpp in step 4b of the Voci refactor.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "nrf_error.h"
#include "ble.h"
#include "ble_gap.h"
#include "bm/softdevice_handler/nrf_sdh_ble.h"

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"

// --- Scan buffer for central / observer mode ---
// S145 on nRF54L15 caps extended-adv data at BLE_GAP_SCAN_BUFFER_EXTENDED_MAX_SUPPORTED (255).
// Same buffer covers legacy 31-byte scans too.
alignas(4) static uint8_t s_BleScanBuff[BLE_GAP_SCAN_BUFFER_EXTENDED_MAX_SUPPORTED];

// Scan timeout is a GAP event from the SoftDevice. Keep the scan state and
// application callback with the scan module rather than making the application
// dispatcher own scan-specific state.
static void BtScanBmEvtHandler(const ble_evt_t *pEvt, void *pCtx)
{
	(void)pCtx;
	if (pEvt->header.evt_id == BLE_GAP_EVT_TIMEOUT &&
		pEvt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
	{
		g_BtAppData.bScan = false;
		BtAppScanTimeoutHandler();
	}
}

NRF_SDH_BLE_OBSERVER(s_BtScanBmObserver, BtScanBmEvtHandler, NULL, USER);

bool BtAppScanInit(BtGapScanCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	BtGapScanCfg_t cfg = *pCfg;
	if (cfg.Param.Phy == 0)
	{
		cfg.Param.Phy = BT_GAP_PHY_1MBITS;
	}

	if (BtGapScanInit(&cfg) == false)
	{
		return false;
	}

	g_BtAppData.bScan = false;
	if (BtGapScanStart(s_BleScanBuff, sizeof(s_BleScanBuff)) == false)
	{
		return false;
	}

	g_BtAppData.bScan = true;
	return true;
}

void BtAppScan(void)
{
	if (g_BtAppData.bScan)
	{
		// Re-arm the scan with the same parameters (SD requirement after
		// every adv report when reporting is one-shot).
		if (BtGapScanNext(s_BleScanBuff, sizeof(s_BleScanBuff)) == false)
		{
			g_BtAppData.bScan = false;
		}
	}
	else
	{
		if (BtGapScanStart(s_BleScanBuff, sizeof(s_BleScanBuff)))
		{
			g_BtAppData.bScan = true;
		}
	}
}

void BtAppScanStop(void)
{
	BtGapScanStop();
	g_BtAppData.bScan = false;
}
