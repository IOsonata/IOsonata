/**-------------------------------------------------------------------------
@file	bt_gap_stm32wba.cpp

@brief	STM32WBAxx BLE port - Generic Access Profile (GAP) procedures.

        Overrides the weak generic implementations of BtGapParamInit,
        BtGapSetDevName, BtGapSetAppearance, BtGapSetPreferedConnParam,
        BtGapConnect and BtGapScanInit/Start/Stop/Next. The application port
        owns the one permitted aci_gap_init call; the WBA source hook captures
        its exact native service and characteristic handles here. This file
        owns native GAP value updates, scan, connection and per-link
        connection-parameter procedures.

        Core Bluetooth Vol.1, Part A, 6.2

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc. All rights reserved.

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

extern "C" bool AppEvtHandlerIdleRegister(void (*Handler)(void));

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

// HCI scan-type values and ST scan PHY flags.
#define BT_GAP_HCI_SCAN_PASSIVE			0x00
#define BT_GAP_HCI_SCAN_ACTIVE			0x01
#define BT_GAP_HCI_SCAN_INTERVAL_MIN	0x0004U
#define BT_GAP_HCI_SCAN_INTERVAL_MAX	0x4000U
#define BT_GAP_WBA_SCAN_PHY_1M			0x01U
#define BT_GAP_WBA_SCAN_PHY_CODED		0x04U

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
	uint8_t		ScanPhy;			//!< ST LE_1M_PHY_BIT or LE_CODED_PHY_BIT
	bool		bConnParamsValid;	//!< Preferred parameters passed spec-range checks
	bool		bConnParamPumpRegistered;
	uint16_t	ConnIntervalMin;	//!< Preferred minimum, 1.25ms units
	uint16_t	ConnIntervalMax;	//!< Preferred maximum, 1.25ms units
	uint16_t	ConnLatency;		//!< Preferred peripheral latency
	uint16_t	ConnSupTimeout;	//!< Preferred supervision timeout, 10ms units
	uint16_t	GapSrvcHdl;		//!< Native GAP service handle returned by aci_gap_init
	uint16_t	DevNameCharHdl;	//!< Native device-name characteristic handle
	uint16_t	AppearanceCharHdl;	//!< Native appearance characteristic handle
	uint16_t	PpcpCharHdl;		//!< Native peripheral preferred connection params handle
	uint8_t		DevNameMaxLen;		//!< Native device-name characteristic capacity
	bool		bLegacyNameUpdatePending;	//!< Old app helper already performed native update
} BtGapWbaState_t;

static BtGapWbaState_t s_GapWba = {
	.ScanInterval  = (uint16_t)BT_GAP_MSEC_TO_0_625(BT_GAP_SCAN_INTERVAL),
	.ScanWindow    = (uint16_t)BT_GAP_MSEC_TO_0_625(BT_GAP_SCAN_WINDOW),
	.ScanType      = BT_GAP_HCI_SCAN_ACTIVE,
	.ScanFilterDup = 1,
	.ScanPhy       = BT_GAP_WBA_SCAN_PHY_1M,
	.bConnParamsValid = false,
	.bConnParamPumpRegistered = false,
	.GapSrvcHdl = BT_ATT_HANDLE_INVALID,
	.DevNameCharHdl = BT_ATT_HANDLE_INVALID,
	.AppearanceCharHdl = BT_ATT_HANDLE_INVALID,
	.PpcpCharHdl = BT_ATT_HANDLE_INVALID,
	.DevNameMaxLen = 0,
	.bLegacyNameUpdatePending = false,
};

static BtGapWbaConnParamLink_t s_ConnParamLink[CFG_BLE_NUM_LINK];

// aci_gap_init returns the service, Device Name characteristic and Appearance
// characteristic handles. ST creates Peripheral Preferred Connection Parameters
// immediately after Appearance when the peripheral role is enabled, so its
// characteristic handle is Appearance + 2 (declaration/value pair spacing).
void BtGapWbaNativeHandlesSet(uint8_t Role, uint8_t DeviceNameMaxLen,
		uint16_t GapSrvcHdl, uint16_t DevNameCharHdl,
		uint16_t AppearanceCharHdl)
{
	s_GapWba.GapSrvcHdl = GapSrvcHdl;
	s_GapWba.DevNameCharHdl = DevNameCharHdl;
	s_GapWba.AppearanceCharHdl = AppearanceCharHdl;
	s_GapWba.DevNameMaxLen = DeviceNameMaxLen;
	s_GapWba.bLegacyNameUpdatePending = false;
	s_GapWba.PpcpCharHdl = BT_ATT_HANDLE_INVALID;

	if ((Role & GAP_PERIPHERAL_ROLE) != 0 &&
		AppearanceCharHdl != BT_ATT_HANDLE_INVALID &&
		AppearanceCharHdl <= (uint16_t)(UINT16_MAX - 2U))
	{
		s_GapWba.PpcpCharHdl = (uint16_t)(AppearanceCharHdl + 2U);
	}
}

// BtAppGapDeviceNameSet historically used GapService + 2, which is the value
// handle in the native service layout. aci_gatt_update_char_value expects the
// characteristic handle returned by aci_gap_init. Correct only that exact GAP
// path; all user-service updates pass through unchanged.
uint16_t BtGapWbaNativeCharHandleResolve(uint16_t SrvcHdl, uint16_t CharHdl)
{
	if (SrvcHdl == s_GapWba.GapSrvcHdl &&
		CharHdl == (uint16_t)(s_GapWba.GapSrvcHdl + 2U) &&
		s_GapWba.DevNameCharHdl != BT_ATT_HANDLE_INVALID)
	{
		return s_GapWba.DevNameCharHdl;
	}
	return CharHdl;
}

void BtGapWbaNativeCharUpdateComplete(uint16_t SrvcHdl,
		uint16_t RequestedCharHdl, uint16_t ActualCharHdl, uint8_t Status)
{
	if (Status == BLE_STATUS_SUCCESS &&
		SrvcHdl == s_GapWba.GapSrvcHdl &&
		RequestedCharHdl != ActualCharHdl &&
		ActualCharHdl == s_GapWba.DevNameCharHdl)
	{
		// BtAppGapDeviceNameSet calls BtGapSetDevName immediately after this
		// native update. Consume only that one forwarding call.
		s_GapWba.bLegacyNameUpdatePending = true;
	}
}

static bool BtGapWbaNativeCharUpdate(uint16_t CharHdl,
		const uint8_t *pData, uint8_t Len)
{
	if (s_GapWba.GapSrvcHdl == BT_ATT_HANDLE_INVALID ||
		CharHdl == BT_ATT_HANDLE_INVALID || (Len > 0 && pData == nullptr))
	{
		return false;
	}

	return aci_gatt_update_char_value(s_GapWba.GapSrvcHdl, CharHdl,
		0, Len, (uint8_t *)pData) == BLE_STATUS_SUCCESS;
}

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

static bool BtGapWbaScanParamsConvert(uint32_t IntervalMs,
		uint32_t WindowMs, uint16_t *pInterval, uint16_t *pWindow)
{
	if (pInterval == nullptr || pWindow == nullptr)
	{
		return false;
	}

	uint64_t interval = ((uint64_t)IntervalMs * 1600U) / 1000U;
	uint64_t window = ((uint64_t)WindowMs * 1600U) / 1000U;
	if (interval < BT_GAP_HCI_SCAN_INTERVAL_MIN ||
		interval > BT_GAP_HCI_SCAN_INTERVAL_MAX ||
		window < BT_GAP_HCI_SCAN_INTERVAL_MIN ||
		window > BT_GAP_HCI_SCAN_INTERVAL_MAX ||
		window > interval)
	{
		return false;
	}

	*pInterval = (uint16_t)interval;
	*pWindow = (uint16_t)window;
	return true;
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
	// A controller handle may be reused after a lost disconnect event. Remove
	// any stale state before deciding whether this new link needs an update.
	BtGapWbaConnParamClear(BtGapWbaConnParamFind(ConnHdl));

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

void BtGapWbaConnParamsUpdateComplete(uint16_t ConnHdl, uint8_t Status,
		uint16_t Interval, uint16_t Latency, uint16_t Timeout)
{
	BtGapWbaConnParamLink_t *pLink = BtGapWbaConnParamFind(ConnHdl);
	if (pLink == nullptr)
	{
		return;
	}

	if (Status == BLE_STATUS_SUCCESS)
	{
		if (BtGapWbaConnParamsMatch(Interval, Latency, Timeout))
		{
			BtGapWbaConnParamClear(pLink);
		}
		else if (pLink->bAwaitingComplete)
		{
			BtGapWbaConnParamRetry(pLink);
		}
	}
	else if (pLink->bAwaitingComplete)
	{
		BtGapWbaConnParamRetry(pLink);
	}
}

void BtGapWbaConnParamsL2capResponse(uint16_t ConnHdl, uint16_t Result)
{
	BtGapWbaConnParamLink_t *pLink = BtGapWbaConnParamFind(ConnHdl);
	if (pLink == nullptr || pLink->Role != BT_CONN_ROLE_PERIPHERAL ||
		!pLink->bAwaitingComplete)
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

	bool paramsValid = BtGapWbaConnParamsValid(
		s_GapWba.ConnIntervalMin, s_GapWba.ConnIntervalMax,
		s_GapWba.ConnLatency, s_GapWba.ConnSupTimeout);

	if (!s_GapWba.bConnParamPumpRegistered)
	{
		s_GapWba.bConnParamPumpRegistered =
			AppEvtHandlerIdleRegister(BtGapWbaConnParamProcess);
	}

	s_GapWba.bConnParamsValid = paramsValid &&
		s_GapWba.bConnParamPumpRegistered &&
		(pCfg->Role & (BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL)) != 0;

	if (paramsValid && (pCfg->Role & BT_GAP_ROLE_PERIPHERAL) != 0)
	{
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
	if (pName == nullptr)
	{
		return;
	}

	// The old BtAppGapDeviceNameSet path already performed the corrected native
	// write before forwarding here. Consume only that one forwarding call.
	if (s_GapWba.bLegacyNameUpdatePending)
	{
		s_GapWba.bLegacyNameUpdatePending = false;
		return;
	}

	size_t len = strlen(pName);
	if (len > s_GapWba.DevNameMaxLen)
	{
		len = s_GapWba.DevNameMaxLen;
	}
	if (len > UINT8_MAX)
	{
		len = UINT8_MAX;
	}

	(void)BtGapWbaNativeCharUpdate(
		s_GapWba.DevNameCharHdl, (const uint8_t *)pName, (uint8_t)len);
}

void BtGapSetAppearance(uint16_t Val)
{
	uint8_t value[2] = {
		(uint8_t)(Val & 0xFFU),
		(uint8_t)(Val >> 8),
	};
	if (BtGapWbaNativeCharUpdate(s_GapWba.AppearanceCharHdl,
			value, sizeof(value)))
	{
		g_BtAppData.AppDevice.Appearance = Val;
	}
}

void BtGapSetPreferedConnParam(BtGattPreferedConnParams_t *pVal)
{
	if (pVal == nullptr)
	{
		return;
	}

	uint8_t value[8] = {
		(uint8_t)(pVal->IntervalMin & 0xFFU),
		(uint8_t)(pVal->IntervalMin >> 8),
		(uint8_t)(pVal->IntervalMax & 0xFFU),
		(uint8_t)(pVal->IntervalMax >> 8),
		(uint8_t)(pVal->Latency & 0xFFU),
		(uint8_t)(pVal->Latency >> 8),
		(uint8_t)(pVal->Timeout & 0xFFU),
		(uint8_t)(pVal->Timeout >> 8),
	};
	(void)BtGapWbaNativeCharUpdate(
		s_GapWba.PpcpCharHdl, value, sizeof(value));
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
	if (pCfg == NULL ||
		(pCfg->Param.Phy != BT_GAP_PHY_1MBITS &&
		 pCfg->Param.Phy != BT_GAP_PHY_CODED))
	{
		return false;
	}

	uint16_t interval;
	uint16_t window;
	if (!BtGapWbaScanParamsConvert(
			pCfg->Param.Interval, pCfg->Param.Duration, &interval, &window))
	{
		return false;
	}

	s_GapWba.ScanType = (pCfg->Type == BTSCAN_TYPE_ACTIVE) ?
		BT_GAP_HCI_SCAN_ACTIVE : BT_GAP_HCI_SCAN_PASSIVE;
	s_GapWba.ScanInterval = interval;
	s_GapWba.ScanWindow = window;
	s_GapWba.ScanFilterDup = 1;
	s_GapWba.ScanPhy = pCfg->Param.Phy == BT_GAP_PHY_CODED ?
		BT_GAP_WBA_SCAN_PHY_CODED : BT_GAP_WBA_SCAN_PHY_1M;

	return aci_gap_set_scan_configuration(
		s_GapWba.ScanFilterDup,
		0,							// scanning filter policy = accept all
		s_GapWba.ScanPhy,
		s_GapWba.ScanType,
		s_GapWba.ScanInterval,
		s_GapWba.ScanWindow) == BLE_STATUS_SUCCESS;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	(void)pBuff;	// ST stack reports results via HCI events, not a buffer
	(void)Len;

	return aci_gap_start_procedure(
		GAP_OBSERVATION_PROC,
		s_GapWba.ScanPhy,
		0,
		0) == BLE_STATUS_SUCCESS;
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

	// The ST host API reports the security mode and level but not the
	// negotiated encryption-key size. Leave KeySize at zero, which the common
	// ATT security gate defines as "not reported", rather than asserting 16.

	// The ST host owns the bond table as an opaque blob and a reliable per-link
	// bonded query is not available here (resolvable peer address plus opaque
	// storage), so BT_GAP_SEC_FLAG_BONDED is left unset on this port.

	return true;
}
