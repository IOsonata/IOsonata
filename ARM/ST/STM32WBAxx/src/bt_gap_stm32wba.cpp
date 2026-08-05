/**-------------------------------------------------------------------------
@file	bt_gap_stm32wba.cpp

@brief	STM32WBAxx BLE port - Generic Access Profile (GAP) procedures.

        Overrides the weak generic implementations of BtGapParamInit,
        BtGapSetDevName, BtGapConnect, BtGapScanInit/Start/Stop/Next.
        The application port owns the one-time ST GAP initialization and its
        native service handles; this file owns scan and connection procedures.

        Core Bluetooth Vol.1, Part A, 6.2

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal.h"

#include "ble_types.h"
#include "ble_std.h"
#include "ble.h"
#include "ble_gap_aci.h"
#include "ble_hci_le.h"

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"

#ifndef BLE_STATUS_SUCCESS
#define BLE_STATUS_SUCCESS				0x00
#endif

// 1ms -> stack-unit conversions. HCI uses 0.625ms for adv/scan intervals
// and 1.25ms for connection intervals. Conn supervision timeout is 10ms.
#define BT_GAP_MSEC_TO_0_625(ms)		(((uint32_t)(ms) * 1600U) / 1000U)
#define BT_GAP_MSEC_TO_1_25(ms)			(((uint32_t)(ms) * 800U) / 1000U)
#define BT_GAP_MSEC_TO_10MS(ms)			(((uint32_t)(ms) + 9U) / 10U)

// HCI scan-type values.
#define BT_GAP_HCI_SCAN_PASSIVE			0x00
#define BT_GAP_HCI_SCAN_ACTIVE			0x01

// --- Port-private GAP state ---

typedef struct __Bt_Gap_WbaState {
	uint16_t	ScanInterval;		//!< Scan interval in 0.625ms units
	uint16_t	ScanWindow;			//!< Scan window in 0.625ms units
	uint8_t		ScanType;			//!< HCI scan type (active/passive)
	uint8_t		ScanFilterDup;		//!< Filter duplicate adv reports
	bool		bSettingDevName;	//!< Break BtGap/BtApp name-set forwarding recursion
} BtGapWbaState_t;

static BtGapWbaState_t s_GapWba = {
	.ScanInterval  = (uint16_t)BT_GAP_MSEC_TO_0_625(BT_GAP_SCAN_INTERVAL),
	.ScanWindow    = (uint16_t)BT_GAP_MSEC_TO_0_625(BT_GAP_SCAN_WINDOW),
	.ScanType      = BT_GAP_HCI_SCAN_ACTIVE,
	.ScanFilterDup = 1,
	.bSettingDevName = false,
};

// --- Public overrides (weak in bt_gap.cpp) ---

void BtGapParamInit(const BtGapCfg_t *pCfg)
{
	// BtAppInit owns the one permitted aci_gap_init call, native GAP handles,
	// appearance, IO capability and authentication configuration. Repeating
	// those operations here returns BLE_STATUS_NOT_ALLOWED on the ST stack and
	// leaves this port with invalid duplicate handles. Connection-parameter
	// procedures are connection-scoped and therefore cannot be issued here with
	// a fabricated handle before a link exists.
	(void)pCfg;
}

void BtGapSetDevName(const char *pName)
{
	if (pName == NULL || s_GapWba.bSettingDevName)
	{
		return;
	}

	// BtAppGapDeviceNameSet owns the native service handles returned by the
	// single aci_gap_init call. It calls BtGapSetDevName after updating the ST
	// database, so guard the nested call rather than maintaining a second handle
	// copy in this translation unit.
	s_GapWba.bSettingDevName = true;
	BtAppGapDeviceNameSet(pName);
	s_GapWba.bSettingDevName = false;
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr,
                  BtGapConnParams_t * const pConnParam)
{
	if (pPeerAddr == NULL || pConnParam == NULL)
	{
		return false;
	}

	uint8_t ret = aci_gap_create_connection(
		s_GapWba.ScanInterval,
		s_GapWba.ScanWindow,
		pPeerAddr->Type,
		pPeerAddr->Addr,
		0,							// own address type = public
		(uint16_t)BT_GAP_MSEC_TO_1_25(pConnParam->IntervalMin),
		(uint16_t)BT_GAP_MSEC_TO_1_25(pConnParam->IntervalMax),
		pConnParam->Latency,
		(uint16_t)BT_GAP_MSEC_TO_10MS(pConnParam->Timeout),
		0,							// min CE length
		0);							// max CE length

	return ret == BLE_STATUS_SUCCESS;
}

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
	if (pCfg == NULL)
	{
		return false;
	}

	s_GapWba.ScanType      = (pCfg->Type == BTSCAN_TYPE_ACTIVE)
	                         ? BT_GAP_HCI_SCAN_ACTIVE : BT_GAP_HCI_SCAN_PASSIVE;
	s_GapWba.ScanInterval  = (uint16_t)BT_GAP_MSEC_TO_0_625(pCfg->Param.Interval);
	s_GapWba.ScanWindow    = (uint16_t)BT_GAP_MSEC_TO_0_625(pCfg->Param.Duration);
	s_GapWba.ScanFilterDup = 1;

	return true;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	(void)pBuff;	// ST stack reports results via HCI events, not a buffer
	(void)Len;

	uint8_t ret = aci_gap_start_observation_procedure(
		s_GapWba.ScanInterval,
		s_GapWba.ScanWindow,
		s_GapWba.ScanType,
		0,							// own address type = public
		s_GapWba.ScanFilterDup,
		0);							// scanning filter policy = accept all

	return ret == BLE_STATUS_SUCCESS;
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
	// ST's stack auto-rearms after each report; nRF SoftDevice doesn't,
	// which is why the generic API has a Next call. Here Next is a no-op
	// confirmation that scanning is still active.
	(void)pBuff;
	(void)Len;
	return true;
}

void BtGapScanStop(void)
{
	(void)aci_gap_terminate_gap_procedure(GAP_OBSERVATION_PROC);
}

// Strong override of the weak generic BtGapConnSecGet. The ST host owns link
// security on this port, so query it through aci_gap_get_security_level and map
// the reported security level (1 to 4) into BtConnSec_t.
bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	if (pSec == nullptr)
	{
		return false;
	}

	uint8_t mode = 0;
	uint8_t level = 0;
	if (aci_gap_get_security_level(ConnHdl, &mode, &level) != BLE_STATUS_SUCCESS)
	{
		return false;
	}

	pSec->Level = BT_GAP_SEC_LEVEL_NONE;
	pSec->KeySize = 0;
	pSec->Flags = 0;

	switch (level)
	{
		case 0x02:
			pSec->Level = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
			break;
		case 0x03:
			pSec->Level = BT_GAP_SEC_LEVEL_ENC_AUTH;
			break;
		case 0x04:
			pSec->Level = BT_GAP_SEC_LEVEL_LESC_AUTH;
			pSec->Flags |= BT_GAP_SEC_FLAG_SC;
			break;
		default:
			// 0x01 is security mode 1 level 1 (no security).
			break;
	}

	if (level >= 0x02)
	{
		// aci_gap_get_security_level does not report the per-link key size. SC
		// and WBA pairing use a 16-octet key, so report 16 on an encrypted link.
		pSec->KeySize = 16;
	}

	// The ST host owns the bond table as an opaque blob and a reliable per-link
	// bonded query is not available here (resolvable peer address plus opaque
	// storage), so BT_GAP_SEC_FLAG_BONDED is left unset on this port.

	return true;
}
