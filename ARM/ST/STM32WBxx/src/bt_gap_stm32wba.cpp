/**-------------------------------------------------------------------------
@file	bt_gap_stm32wba.cpp

@brief	STM32WBAxx BLE port - Generic Access Profile (GAP) procedures.

        Overrides the weak generic implementations of BtGapParamInit,
        BtGapSetDevName, BtGapConnect, BtGapScanInit/Start/Stop/Next.
        Owns the GAP service handles returned by aci_gap_init and the
        port-private scan parameter state.

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
#include "ble_gatt_aci.h"
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

// SMP key-size limits per BT spec.
#define SEC_PARAM_MIN_KEY_SIZE			7
#define SEC_PARAM_MAX_KEY_SIZE			16

// Default IO capability for SMP. Apps that need a different one can call
// aci_gap_set_io_capability themselves after BtAppInit.
#ifndef BT_GAP_DEFAULT_IO_CAPABILITY
#define BT_GAP_DEFAULT_IO_CAPABILITY	0x03	// NoInputNoOutput
#endif

// ST GAP role bits, matching aci_gap_init's role parameter.
#define BT_GAP_ROLE_PERIPHERAL_BIT		0x01
#define BT_GAP_ROLE_BROADCASTER_BIT		0x02
#define BT_GAP_ROLE_CENTRAL_BIT			0x04
#define BT_GAP_ROLE_OBSERVER_BIT		0x08

// HCI scan-type values.
#define BT_GAP_HCI_SCAN_PASSIVE			0x00
#define BT_GAP_HCI_SCAN_ACTIVE			0x01

// --- Port-private GAP state ---

typedef struct __Bt_Gap_WbaState {
	uint16_t	GapSrvcHdl;			//!< GAP service start handle (0x1800)
	uint16_t	DevNameCharHdl;		//!< Device-name characteristic value handle
	uint16_t	AppearanceCharHdl;	//!< Appearance characteristic value handle

	uint16_t	ScanInterval;		//!< Scan interval in 0.625ms units
	uint16_t	ScanWindow;			//!< Scan window in 0.625ms units
	uint8_t		ScanType;			//!< HCI scan type (active/passive)
	uint8_t		ScanFilterDup;		//!< Filter duplicate adv reports
} BtGapWbaState_t;

static BtGapWbaState_t s_GapWba = {
	.GapSrvcHdl        = 0,
	.DevNameCharHdl    = 0,
	.AppearanceCharHdl = 0,
	.ScanInterval      = (uint16_t)BT_GAP_MSEC_TO_0_625(BT_GAP_SCAN_INTERVAL),
	.ScanWindow        = (uint16_t)BT_GAP_MSEC_TO_0_625(BT_GAP_SCAN_WINDOW),
	.ScanType          = BT_GAP_HCI_SCAN_ACTIVE,
	.ScanFilterDup     = 1,
};

// Map BtGap SecType to (bonding, mitm, sc) triple for ST's auth-req call.
static void MapSecType(BTGAP_SECTYPE SecType,
                       uint8_t *pBonding, uint8_t *pMitm, uint8_t *pSc)
{
	*pBonding = 0;
	*pMitm    = 0;
	*pSc      = 0;

	switch (SecType)
	{
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
			*pBonding = 1;
			break;
		case BTGAP_SECTYPE_STATICKEY_MITM:
		case BTGAP_SECTYPE_SIGNED_MITM:
			*pBonding = 1;
			*pMitm    = 1;
			break;
		case BTGAP_SECTYPE_LESC_MITM:
			*pBonding = 1;
			*pMitm    = 1;
			*pSc      = 1;
			break;
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
			*pBonding = 1;
			break;
		case BTGAP_SECTYPE_NONE:
		default:
			break;
	}
}

// Map BtAppCfg_t::Role bits to ST's aci_gap_init role byte.
static uint8_t MapRoleBits(uint8_t Role)
{
	uint8_t out = 0;
	if (Role & BTAPP_ROLE_PERIPHERAL)  out |= BT_GAP_ROLE_PERIPHERAL_BIT;
	if (Role & BTAPP_ROLE_BROADCASTER) out |= BT_GAP_ROLE_BROADCASTER_BIT;
	if (Role & BTAPP_ROLE_CENTRAL)     out |= BT_GAP_ROLE_CENTRAL_BIT;
	if (Role & BTAPP_ROLE_OBSERVER)    out |= BT_GAP_ROLE_OBSERVER_BIT;
	return out;
}

// --- Public overrides (weak in bt_gap.cpp) ---

void BtGapParamInit(const BtGapCfg_t *pCfg)
{
	if (pCfg == NULL)
	{
		return;
	}

	// Create the GAP service. aci_gap_init returns the start handle plus
	// the value handles for the mandatory device-name and appearance
	// characteristics. Privacy left disabled by default.
	uint8_t ret = aci_gap_init(MapRoleBits(pCfg->Role),
	                           0x00,	// privacy disabled
	                           0,		// dev_name_char_len (0 = use default)
	                           &s_GapWba.GapSrvcHdl,
	                           &s_GapWba.DevNameCharHdl,
	                           &s_GapWba.AppearanceCharHdl);
	if (ret != BLE_STATUS_SUCCESS)
	{
		return;
	}

	// SMP / pairing config.
	uint8_t bonding, mitm, sc;
	MapSecType((BTGAP_SECTYPE)pCfg->SecType, &bonding, &mitm, &sc);

	aci_gap_set_authentication_requirement(
		bonding,
		mitm,
		sc,
		0,							// keypress not supported
		SEC_PARAM_MIN_KEY_SIZE,
		SEC_PARAM_MAX_KEY_SIZE,
		0,							// no fixed pin
		0,							// fixed pin value (unused)
		0);							// identity address type = public

	aci_gap_set_io_capability(BT_GAP_DEFAULT_IO_CAPABILITY);

	// Push appearance into the GAP service char.
	uint16_t appearance = g_BtAppData.Appearance;
	aci_gatt_update_char_value(s_GapWba.GapSrvcHdl,
	                           s_GapWba.AppearanceCharHdl,
	                           0, sizeof(appearance),
	                           (uint8_t *)&appearance);

	// Preferred peripheral connection parameters - peripheral role only.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		aci_l2cap_connection_parameter_update_req(
			0,		// connection handle not used in this peripheral-pref call
			(uint16_t)BT_GAP_MSEC_TO_1_25(pCfg->ConnIntervalMin),
			(uint16_t)BT_GAP_MSEC_TO_1_25(pCfg->ConnIntervalMax),
			pCfg->SlaveLatency,
			(uint16_t)BT_GAP_MSEC_TO_10MS(pCfg->SupTimeout));
	}
}

void BtGapSetDevName(const char *pName)
{
	if (pName == NULL || s_GapWba.GapSrvcHdl == 0)
	{
		return;
	}

	size_t len = strlen(pName);
	if (len > 248)	// ST GAP device-name char max length
	{
		len = 248;
	}

	aci_gatt_update_char_value(s_GapWba.GapSrvcHdl,
	                           s_GapWba.DevNameCharHdl,
	                           0, (uint8_t)len, (uint8_t *)pName);
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
