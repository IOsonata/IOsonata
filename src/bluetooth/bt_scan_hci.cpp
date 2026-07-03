/**-------------------------------------------------------------------------
@file	bt_scan_hci.cpp

@brief	Application scanning wrapper over the HCI GAP layer.

        Thin app-level entry points that delegate to the generic BtGapScan*
        functions. No vendor headers.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"

void BtAppScan()
{
	if (BtGapScanStart(NULL, 0))
	{
		g_BtAppData.bScan = true;
	}
}

void BtAppScanStop()
{
	if (g_BtAppData.bScan == true)
	{
		BtGapScanStop();
		g_BtAppData.bScan = false;
	}
}

bool BtAppScanInit(BtGapScanCfg_t *pCfg)
{
	return BtGapScanInit(pCfg);
}
