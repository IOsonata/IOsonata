/**-------------------------------------------------------------------------
@file	bt_scan_stm32wba.cpp

@brief	STM32WBAxx BLE port - app-level scan wrappers.

        Thin layer above bt_gap_stm32wba.cpp - the actual ACI calls live
        in BtGapScan*. This file only gates state and delegates, same as
        bt_scan_bm.cpp.

        ST's BLE stack reports scan results via HCI LE Advertising Report
        events handled in bt_app_stm32wba.cpp's HCI event router, so this
        TU does not need a scan buffer like the nRF SoftDevice ports do.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"

bool BtAppScanInit(BtGapScanCfg_t *pCfg)
{
	if (pCfg == NULL)
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
	// ST's GAP procedure reports advertising results as HCI events. The args
	// here stay for API parity with the SoftDevice ports.
	if (BtGapScanStart(NULL, 0) == false)
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
		// ST stack keeps scanning until terminated. Just confirm.
		if (BtGapScanNext(NULL, 0) == false)
		{
			g_BtAppData.bScan = false;
		}
	}
	else if (BtGapScanStart(NULL, 0))
	{
		g_BtAppData.bScan = true;
	}
}

void BtAppScanStop(void)
{
	BtGapScanStop();
	g_BtAppData.bScan = false;
}
