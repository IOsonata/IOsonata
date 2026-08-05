/**-------------------------------------------------------------------------
@file	bt_gap_stm32wba.cpp

@brief	STM32WBAxx BLE port - Generic Access Profile (GAP) procedures.

        Overrides the weak generic implementations of BtGapParamInit,
        BtGapSetDevName, BtGapConnect, BtGapScanInit/Start/Stop/Next.
        The application port owns the one-time ST GAP initialization and its
        native service handles; this file owns scan, connection and per-link
        connection-parameter procedures.

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
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include "bluetooth/bt_peer.h"
#include "app_evt_handler.h"

#ifndef BLE_STATUS_SUCCESS
#define BLE_STATUS_SUCCESS				0x00
#endif

#ifndef CFG_BLE_NUM_LINK
#define CFG_BLE_NUM_LINK				2
#endif

#ifndef BT_GAP_WBA_CONN_PARAMS_FIRST_DELAY_MS
#define BT_GAP_WBA_CONN_PARAMS_FIRST_DELAY_MS	5000U
#endif
#ifndef BT_GAP_WBA_CONN_PARAMS_NEXT_DELAY_MS
#define BT_GAP_WBA_CONN_PARAMS_NEXT_DELAY_MS	30000U
#endif
#ifndef BT_GAP_WBA_CONN_PARAMS_MAX_ATTEMPTS
#define BT_GAP_WBA_CONN_PARAMS_MAX_ATTEMPTS		3U
#endif

// 1ms -> stack-unit conversions. HCI uses 0.625ms for adv/scan intervals
// and 1.25ms for connection intervals. Conn supervision timeout is 10ms.
#define BT_GAP_MSEC_TO_0_625(ms)		(((uint32_t)(ms) * 1600U) / 1000U)
#define BT_GAP_MSEC_TO_10MS(ms)			(((uint32_t)(ms) + 9U) / 10U)

// HCI scan-type values.
#define BT_GAP_HCI_SCAN_PASSIVE			0x00
#define BT_GAP_HCI_SCAN_ACTIVE			0x01

// --- Port-private GAP state ---

typedef struct __Bt_Gap_WbaConnParamLink {
	uint16_t	ConnHdl;			//!< Real controller connection handle
	uint32_t	DueTime;			//!< Next request or completion-watchdog time
	uint8_t		Role;				//!< Local BT_CONN_ROLE_* on this link
	uint8_t		Attempts;			//!< Requests issued for this link
	bool		bAwaitingComplete;	//!< Command accepted; wait for response/completion
} BtGapWbaConnParamLink_t;

typedef struct __Bt_Gap_WbaState {
	uint16_t	ScanInterval;		//!< Scan interval in 0.625ms units
	uint16_t	ScanWindow;			//!< Scan window in 0.625ms units
	uint8_t		ScanType;			//!< HCI scan type (active/passive)
	uint8_t		ScanFilterDup;		//!< Filter duplicate adv reports
	bool		bSettingDevName;	//!< Break BtGap/BtApp name-set forwarding recursion
	bool		bConnParamsValid;	//!< Preferred parameters passed spec-range checks
	bool		bConnParamPumpRegistered;
	uint16_t	ConnIntervalMin;	//!< Preferred minimum, 1.25ms units
	uint16_t	ConnIntervalMax;	//!< Preferred maximum, 1.25ms units
	uint16_t	ConnLatency;		//!< Preferred peripheral latency
	uint16_t	ConnSupTimeout;	//!< Preferred supervision timeout, 10ms units
} BtGapWbaState_t;

static BtGapWbaState_t s_GapWba = {
	.ScanInterval  = (uint16_t)BT_GAP_MSEC_TO_0_625(BT_GAP_SCAN_INTERVAL),
	.ScanWindow    = (uint16_t)BT_GAP_MSEC_TO_0_625(BT_GAP_SCAN_WINDOW),
	.ScanType      = BT_GAP_HCI_SCAN_ACTIVE,
	.ScanFilterDup = 1,
	.bSettingDevName = false,
	.bConnParamsValid = false,
	.bConnParamPumpRegistered = false,
};

static BtGapWbaConnParamLink_t s_ConnParamLink[CFG_BLE_NUM_LINK];

static uint16_t BtGapWbaMsecTo1_25(float Ms)
{
	if (Ms <= 0.0f)
	{
		return 0;
	}

	float units = Ms / 1.25f;
	if (units >= 65535.0f)
	{
		return 0xFFFF;
	}

	return (uint16_t)(units + 0.5f);
}

static bool BtGapWbaConnParamsValid(uint16_t IntervalMin,
		uint16_t IntervalMax, uint16_t Latency, uint16_t Timeout)
{
	if (IntervalMin < 0x0006 || IntervalMax > 0x0C80 ||
		IntervalMin > IntervalMax || Latency > 0x01F3 ||
		Timeout < 0x000A || Timeout > 0x0C80)
	{
		return false;
	}

	// Core Vol 6 Part B: supervision timeout must be greater than
	// 2 * max_interval * (latency + 1). Convert the mixed HCI units without
	// floating point: timeout(10ms) * 4 > interval(1.25ms) * (latency + 1).
	return (uint32_t)Timeout * 4U >
		(uint32_t)IntervalMax * ((uint32_t)Latency + 1U);
}

static void BtGapWbaConnParamClear(BtGapWbaConnParamLink_t *pLink)
{
	if (pLink == nullptr)
	{
		return;
	}

	memset(pLink, 0, sizeof(*pLink));
	pLink->ConnHdl = BT_CONN_HDL_INVALID;
}

static void BtGapWbaConnParamResetAll(void)
{
	for (size_t i = 0; i < CFG_BLE_NUM_LINK; i++)
	{
		BtGapWbaConnParamClear(&s_ConnParamLink[i]);
	}
}

static BtGapWbaConnParamLink_t *BtGapWbaConnParamFind(uint16_t ConnHdl)
{
	for (size_t i = 0; i < CFG_BLE_NUM_LINK; i++)
	{
		if (s_ConnParamLink[i].ConnHdl == ConnHdl)
		{
			return &s_ConnParamLink[i];
		}
	}
	return nullptr;
}

static BtGapWbaConnParamLink_t *BtGapWbaConnParamAlloc(uint16_t ConnHdl)
{
	BtGapWbaConnParamLink_t *pLink = BtGapWbaConnParamFind(ConnHdl);
	if (pLink != nullptr)
	{
		return pLink;
	}

	for (size_t i = 0; i < CFG_BLE_NUM_LINK; i++)
	{
		if (s_ConnParamLink[i].ConnHdl == BT_CONN_HDL_INVALID)
		{
			return &s_ConnParamLink[i];
		}
	}
	return nullptr;
}

static bool BtGapWbaConnParamsMatch(uint16_t Interval, uint16_t Latency,
		uint16_t Timeout)
{
	return Interval >= s_GapWba.ConnIntervalMin &&
		Interval <= s_GapWba.ConnIntervalMax &&
		Latency == s_GapWba.ConnLatency &&
		Timeout == s_GapWba.ConnSupTimeout;
}

static void BtGapWbaConnParamRetry(BtGapWbaConnParamLink_t *pLink)
{
	if (pLink == nullptr)
	{
		return;
	}

	pLink->bAwaitingComplete = false;
	if (pLink->Attempts >= BT_GAP_WBA_CONN_PARAMS_MAX_ATTEMPTS)
	{
		BtGapWbaConnParamClear(pLink);
		return;
	}

	pLink->DueTime = HAL_GetTick() + BT_GAP_WBA_CONN_PARAMS_NEXT_DELAY_MS;
}

static void BtGapWbaConnParamIssue(BtGapWbaConnParamLink_t *pLink)
{
	if (pLink == nullptr ||
		pLink->Attempts >= BT_GAP_WBA_CONN_PARAMS_MAX_ATTEMPTS)
	{
		BtGapWbaConnParamClear(pLink);
		return;
	}

	pLink->Attempts++;
	tBleStatus status;
	if (pLink->Role == BT_CONN_ROLE_PERIPHERAL)
	{
		status = aci_l2cap_connection_parameter_update_req(
			pLink->ConnHdl,
			s_GapWba.ConnIntervalMin,
			s_GapWba.ConnIntervalMax,
			s_GapWba.ConnLatency,
			s_GapWba.ConnSupTimeout);
	}
	else if (pLink->Role == BT_CONN_ROLE_CENTRAL)
	{
		status = aci_gap_start_connection_update(
			pLink->ConnHdl,
			s_GapWba.ConnIntervalMin,
			s_GapWba.ConnIntervalMax,
			s_GapWba.ConnLatency,
			s_GapWba.ConnSupTimeout,
			0,	// minimum connection-event length
			0);	// maximum connection-event length
	}
	else
	{
		BtGapWbaConnParamClear(pLink);
		return;
	}

	if (status == BLE_STATUS_SUCCESS)
	{
		pLink->bAwaitingComplete = true;
		// Also bounds a lost L2CAP response or HCI completion event.
		pLink->DueTime = HAL_GetTick() + BT_GAP_WBA_CONN_PARAMS_NEXT_DELAY_MS;
	}
	else
	{
		BtGapWbaConnParamRetry(pLink);
	}
}

static void BtGapWbaConnParamProcess(void)
{
	if (!s_GapWba.bConnParamsValid)
	{
		return;
	}

	uint32_t now = HAL_GetTick();
	for (size_t i = 0; i < CFG_BLE_NUM_LINK; i++)
	{
		BtGapWbaConnParamLink_t *pLink = &s_ConnParamLink[i];
		if (pLink->ConnHdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}

		if (BtPeerFindByHdl(pLink->ConnHdl) == nullptr)
		{
			BtGapWbaConnParamClear(pLink);
			continue;
		}

		if ((int32_t)(now - pLink->DueTime) < 0)
		{
			continue;
		}

		// Due while awaiting means the response/completion watchdog expired.
		pLink->bAwaitingComplete = false;
		BtGapWbaConnParamIssue(pLink);
	}
}

void BtGapWbaConnParamsConnected(uint16_t ConnHdl, uint8_t Role,
		uint16_t Interval, uint16_t Latency, uint16_t Timeout)
{
	if (!s_GapWba.bConnParamsValid ||
		(Role != BT_CONN_ROLE_PERIPHERAL && Role != BT_CONN_ROLE_CENTRAL) ||
		BtGapWbaConnParamsMatch(Interval, Latency, Timeout))
	{
		return;
	}

	BtGapWbaConnParamLink_t *pLink = BtGapWbaConnParamAlloc(ConnHdl);
	if (pLink == nullptr)
	{
		return;
	}

	BtGapWbaConnParamClear(pLink);
	pLink->ConnHdl = ConnHdl;
	pLink->Role = Role;
	pLink->DueTime = HAL_GetTick() + BT_GAP_WBA_CONN_PARAMS_FIRST_DELAY_MS;
}

void BtGapWbaConnParamsDisconnected(uint16_t ConnHdl)
{
	BtGapWbaConnParamClear(BtGapWbaConnParamFind(ConnHdl));
}

void BtGapWbaConnParamsUpdateComplete(uint16_t ConnHdl, uint8_t Status)
{
	BtGapWbaConnParamLink_t *pLink = BtGapWbaConnParamFind(ConnHdl);
	if (pLink == nullptr)
	{
		return;
	}

	if (Status == BLE_STATUS_SUCCESS)
	{
		BtGapWbaConnParamClear(pLink);
	}
	else
	{
		BtGapWbaConnParamRetry(pLink);
	}
}

void BtGapWbaConnParamsL2capResponse(uint16_t ConnHdl, uint16_t Result)
{
	BtGapWbaConnParamLink_t *pLink = BtGapWbaConnParamFind(ConnHdl);
	if (pLink == nullptr || pLink->Role != BT_CONN_ROLE_PERIPHERAL)
	{
		return;
	}

	// Result 0 means accepted; the HCI update-complete event finishes the
	// procedure. A rejection is retried after the bounded backoff.
	if (Result != 0)
	{
		BtGapWbaConnParamRetry(pLink);
	}
}

// --- Public overrides (weak in bt_gap.cpp) ---

void BtGapParamInit(const BtGapCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return;
	}

	BtGapWbaConnParamResetAll();
	s_GapWba.ConnIntervalMin = BtGapWbaMsecTo1_25(pCfg->ConnIntervalMin);
	s_GapWba.ConnIntervalMax = BtGapWbaMsecTo1_25(pCfg->ConnIntervalMax);
	s_GapWba.ConnLatency = pCfg->SlaveLatency;
	s_GapWba.ConnSupTimeout = (uint16_t)BT_GAP_MSEC_TO_10MS(pCfg->SupTimeout);

	if (!s_GapWba.bConnParamPumpRegistered)
	{
		s_GapWba.bConnParamPumpRegistered =
			AppEvtHandlerIdleRegister(BtGapWbaConnParamProcess);
	}

	s_GapWba.bConnParamsValid = s_GapWba.bConnParamPumpRegistered &&
		(pCfg->Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL)) != 0 &&
		BtGapWbaConnParamsValid(
			s_GapWba.ConnIntervalMin, s_GapWba.ConnIntervalMax,
			s_GapWba.ConnLatency, s_GapWba.ConnSupTimeout);

	if (s_GapWba.bConnParamsValid && (pCfg->Role & BT_GAP_ROLE_PERIPHERAL))
	{
		// Keep the generic GAP mirror coherent. The native ST PPCP handle is
		// owned by the one aci_gap_init call in bt_app_stm32wba.cpp.
		BtGattPreferedConnParams_t ppcp = {
			.IntervalMin = s_GapWba.ConnIntervalMin,
			.IntervalMax = s_GapWba.ConnIntervalMax,
			.Latency = s_GapWba.ConnLatency,
			.Timeout = s_GapWba.ConnSupTimeout,
		};
		BtGapSetPreferedConnParam(&ppcp);
	}
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

	uint16_t intervalMin = BtGapWbaMsecTo1_25(pConnParam->IntervalMin);
	uint16_t intervalMax = BtGapWbaMsecTo1_25(pConnParam->IntervalMax);
	uint16_t timeout = (uint16_t)BT_GAP_MSEC_TO_10MS(pConnParam->Timeout);
	if (!BtGapWbaConnParamsValid(
			intervalMin, intervalMax, pConnParam->Latency, timeout))
	{
		return false;
	}

	uint8_t ret = aci_gap_create_connection(
		s_GapWba.ScanInterval,
		s_GapWba.ScanWindow,
		pPeerAddr->Type,
		pPeerAddr->Addr,
		0,							// own address type = public
		intervalMin,
		intervalMax,
		pConnParam->Latency,
		timeout,
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
