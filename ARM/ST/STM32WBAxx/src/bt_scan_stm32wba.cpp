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

static bool s_bScanning;

bool BtScanIsActive(void)
{
	return s_bScanning;
}

void BtScanEnded(void)
{
	s_bScanning = false;
}


bool BtAppScanInit(BtGapScanCfg_t *pCfg)
{
	if (pCfg == NULL)
	{
		return false;
	}

	if (BtGapScanInit(pCfg) == false)
	{
		return false;
	}

	s_bScanning = BtGapScanStart(NULL, 0);
	return s_bScanning;
}

void BtAppScan(void)
{
	if (s_bScanning)
	{
		// ST stack keeps scanning until terminated. Just confirm.
		BtGapScanNext(NULL, 0);
	}
	else
	{
		s_bScanning = BtGapScanStart(NULL, 0);
	}
}

void BtAppScanStop(void)
{
	if (s_bScanning)
	{
		// The GAP port clears the state only when ST accepts the stop.
		BtGapScanStop();
	}
}
