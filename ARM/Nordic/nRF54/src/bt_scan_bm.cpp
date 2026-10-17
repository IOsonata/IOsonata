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


// --- Scan buffer for central / observer mode ---
// S145 on nRF54L15 caps extended-adv data at BLE_GAP_SCAN_BUFFER_EXTENDED_MAX_SUPPORTED (255).
// Same buffer covers legacy 31-byte scans too.
alignas(4) static uint8_t s_BleScanBuff[BLE_GAP_SCAN_BUFFER_EXTENDED_MAX_SUPPORTED];

bool BtAppScanInit(BtGapScanCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	if (BtGapScanInit(pCfg) == false)
	{
		return false;
	}

	s_bScanning = BtGapScanStart(s_BleScanBuff, sizeof(s_BleScanBuff));
	return s_bScanning;
}

void BtAppScan(void)
{
	if (s_bScanning)
	{
		// Re-arm the scan with the same parameters (SD requirement after
		// every adv report when reporting is one-shot).
		BtGapScanNext(s_BleScanBuff, sizeof(s_BleScanBuff));
	}
	else
	{
		s_bScanning = BtGapScanStart(s_BleScanBuff, sizeof(s_BleScanBuff));
	}
}

void BtAppScanStop(void)
{
	BtGapScanStop();
	s_bScanning = false;
}
