/**-------------------------------------------------------------------------
@file	bt_adv_stm32wba.cpp

@brief	STM32WBAxx BLE port - advertising path.

        Uses the generic BtAdvEncode helper (from src/bluetooth/
        bt_adv.cpp) to build the AD payload, then pushes it to the ST BLE
        stack via either legacy HCI commands (legacy adv) or aci_gap_adv_set_*
        commands (extended adv). Mirrors the structure of bt_adv_bm.cpp.

        Adv-data buffers are owned by this TU; nothing about them needs to
        be exposed to peer files.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal.h"

#include "ble_types.h"
#include "ble_std.h"
#include "ble.h"
#include "ble_gap_aci.h"
#include "ble_hci_le.h"

#include "istddef.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_appearance.h"

/******** For DEBUG ************/
//#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

#ifndef BLE_STATUS_SUCCESS
#define BLE_STATUS_SUCCESS				0x00
#endif

// Legacy adv payload is capped at 31 octets per BT spec. Extended adv on
// STM32WBA tops out at 251 octets in a single fragment with the default
// stack config; apps that need longer can adjust ATT_MTU_MAX in app_conf.h
// (the stack will then accept up to 1650 over multiple fragments).
#define BT_WBA_ADV_LEGACY_MAX			31
#define BT_WBA_ADV_EXTENDED_MAX			251

// Single advertising set. ST allows up to CFG_BLE_NUM_ADV_SETS (usually 1
// or 2) - we use set 0 across this port.
#define BT_WBA_ADV_HANDLE				0

// 1ms -> 0.625ms unit conversion (rounded down). HCI adv interval is in
// 0.625 ms units across both legacy and extended adv params.
#define BT_WBA_MSEC_TO_0_625(ms)		(((uint32_t)(ms) * 1600U) / 1000U)
// 10ms unit for adv duration / timeout.
#define BT_WBA_MSEC_TO_10MS(ms)			(((uint32_t)(ms) + 9U) / 10U)

// Extended adv event_properties bitfield (HCI v5.0+).
#define BT_WBA_EVT_PROP_CONNECTABLE		0x0001
#define BT_WBA_EVT_PROP_SCANNABLE		0x0002
#define BT_WBA_EVT_PROP_DIRECTED		0x0004
#define BT_WBA_EVT_PROP_HIGH_DUTY		0x0008
#define BT_WBA_EVT_PROP_LEGACY			0x0010
#define BT_WBA_EVT_PROP_ANON			0x0020
#define BT_WBA_EVT_PROP_INC_TX_POWER	0x0040

// Adv-data operation = complete data (single fragment).
#define BT_WBA_ADV_DATA_OP_COMPLETE		0x03
#define BT_WBA_ADV_DATA_FRAG_PREF_NONE	0x01

// All three channels (37, 38, 39).
#define BT_WBA_ADV_CHAN_MAP_ALL			0x07

// PHY constants for extended adv.
#define BT_WBA_PHY_1M					0x01
#define BT_WBA_PHY_2M					0x02
#define BT_WBA_PHY_CODED				0x03

// --- Adv packet buffers ---
//
// Each pair (legacy + extended) shares the same underlying byte buffer;
// the BtAdvPacket_t.MaxLen field caps each into the right window. This
// mirrors bt_adv_bm.cpp - lets app code reuse the buffer between modes.
alignas(4) static uint8_t s_BtAppAdvBuff[BT_WBA_ADV_EXTENDED_MAX + 5];
alignas(4) static BtAdvPacket_t s_BtAppAdvPkt = {
	BT_WBA_ADV_LEGACY_MAX, 0, s_BtAppAdvBuff
};
alignas(4) static BtAdvPacket_t s_BtAppExtAdvPkt = {
	BT_WBA_ADV_EXTENDED_MAX, 0, s_BtAppAdvBuff
};

alignas(4) static uint8_t s_BtAppSrBuff[BT_WBA_ADV_EXTENDED_MAX + 5];
alignas(4) static BtAdvPacket_t s_BtAppSrPkt = {
	BT_WBA_ADV_LEGACY_MAX, 0, s_BtAppSrBuff
};
alignas(4) static BtAdvPacket_t s_BtAppExtSrPkt = {
	BT_WBA_ADV_EXTENDED_MAX, 0, s_BtAppSrBuff
};

// Map BtAppCfg_t::Role to the HCI legacy advertising type. ADV_IND for
// connectable, ADV_NONCONN_IND for broadcaster-only. ADV_SCAN_IND would
// be scannable-only - not exposed here.
static uint8_t MapLegacyAdvType(uint8_t Role)
{
	if (Role & BTAPP_ROLE_PERIPHERAL)
	{
		return 0x00;	// ADV_IND - connectable + scannable
	}
	return 0x03;	// ADV_NONCONN_IND - non-connectable + non-scannable
}

// Map BtAppCfg_t::Role to the HCI extended-adv event_properties bitfield.
static uint16_t MapExtAdvEvtProps(uint8_t Role)
{
	uint16_t prop = 0;

	if (Role & BTAPP_ROLE_PERIPHERAL)
	{
		prop |= BT_WBA_EVT_PROP_CONNECTABLE;
	}
	// Scannable-not-connectable would set EVT_PROP_SCANNABLE without
	// EVT_PROP_CONNECTABLE; not exposed via BtAppCfg_t.

	return prop;
}

void BtAdvStart(void)
{
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING ||
	    g_BtAppData.ConnHdl != 0xFFFFU)
	{
		return;
	}

	uint8_t ret;
	if (g_BtAppData.bExtAdv == false)
	{
		ret = hci_le_set_advertising_enable(1);
	}
	else
	{
		// aci_gap_adv_set_enable takes an Adv_Set_t array. Single set
		// here; duration 0 = until explicitly disabled.
		Adv_Set_t advSet = {
			.Advertising_Handle              = BT_WBA_ADV_HANDLE,
			.Duration                        = 0,
			.Max_Extended_Advertising_Events = 0,
		};
		ret = aci_gap_adv_set_enable(1, 1, &advSet);
	}

	if (ret == BLE_STATUS_SUCCESS)
	{
		g_BtAppData.State = BTAPP_STATE_ADVERTISING;
	}
	else
	{
		DEBUG_PRINTF("BtAdvStart failed: 0x%02x\r\n", ret);
	}
}

void BtAdvStop(void)
{
	if (g_BtAppData.bExtAdv == false)
	{
		hci_le_set_advertising_enable(0);
	}
	else
	{
		Adv_Set_t advSet = {
			.Advertising_Handle              = BT_WBA_ADV_HANDLE,
			.Duration                        = 0,
			.Max_Extended_Advertising_Events = 0,
		};
		aci_gap_adv_set_enable(0, 1, &advSet);
	}

	g_BtAppData.State = BTAPP_STATE_IDLE;
}

// Push the encoded packet pair to the ST stack. Splits between legacy
// and extended adv command sets - same buffer contents on either side.
static bool PushAdvDataToStack(const BtAppCfg_t *pCfg,
                               BtAdvPacket_t *pAdv, BtAdvPacket_t *pSr)
{
	uint8_t ret;
	uint16_t intervalUnits = (uint16_t)BT_WBA_MSEC_TO_0_625(pCfg->AdvInterval);

	if (g_BtAppData.bExtAdv == false)
	{
		// Legacy path. Adv params first, then data, then scan response.
		uint8_t  zeros[6]  = { 0, };
		uint8_t  advType   = MapLegacyAdvType(pCfg->Role);

		ret = hci_le_set_advertising_parameters(
			intervalUnits,			// min interval (0.625 ms units)
			intervalUnits,			// max interval - same for fixed rate
			advType,
			0,						// own address type = public
			0,						// peer address type
			zeros,					// peer address
			BT_WBA_ADV_CHAN_MAP_ALL,
			0);						// filter policy = any
		if (ret != BLE_STATUS_SUCCESS)
		{
			DEBUG_PRINTF("hci_le_set_advertising_parameters: 0x%02x\r\n", ret);
			return false;
		}

		ret = hci_le_set_advertising_data((uint8_t)pAdv->Len, pAdv->pData);
		if (ret != BLE_STATUS_SUCCESS)
		{
			DEBUG_PRINTF("hci_le_set_advertising_data: 0x%02x\r\n", ret);
			return false;
		}

		// Scan response is optional - stack accepts a zero-length buffer.
		ret = hci_le_set_scan_response_data((uint8_t)pSr->Len, pSr->pData);
		if (ret != BLE_STATUS_SUCCESS)
		{
			DEBUG_PRINTF("hci_le_set_scan_response_data: 0x%02x\r\n", ret);
			return false;
		}
	}
	else
	{
		// Extended adv path. aci_gap_adv_set_configuration takes the
		// full parameter set in one call.
		uint8_t zeros[6] = { 0, };
		uint16_t evtProp = MapExtAdvEvtProps(pCfg->Role);

		ret = aci_gap_adv_set_configuration(
			0x7F,						// TX power = host-pref (no preference)
			BT_WBA_ADV_HANDLE,
			evtProp,
			intervalUnits,				// primary interval min
			intervalUnits,				// primary interval max
			BT_WBA_ADV_CHAN_MAP_ALL,
			0,							// own address type = public
			0, zeros,					// peer
			0,							// filter policy = any
			BT_WBA_PHY_1M,				// primary phy
			0,							// secondary max skip
			BT_WBA_PHY_2M,				// secondary phy
			0,							// sid
			0);							// scan req notif disabled
		if (ret != BLE_STATUS_SUCCESS)
		{
			DEBUG_PRINTF("aci_gap_adv_set_configuration: 0x%02x\r\n", ret);
			return false;
		}

		ret = aci_gap_adv_set_data(BT_WBA_ADV_HANDLE,
		                           BT_WBA_ADV_DATA_OP_COMPLETE,
		                           (uint8_t)pAdv->Len, pAdv->pData);
		if (ret != BLE_STATUS_SUCCESS)
		{
			DEBUG_PRINTF("aci_gap_adv_set_data: 0x%02x\r\n", ret);
			return false;
		}

		if (pSr->Len > 0)
		{
			ret = aci_gap_adv_set_scan_resp_data(BT_WBA_ADV_HANDLE,
			                                     BT_WBA_ADV_DATA_OP_COMPLETE,
			                                     (uint8_t)pSr->Len,
			                                     pSr->pData);
			if (ret != BLE_STATUS_SUCCESS)
			{
				DEBUG_PRINTF("aci_gap_adv_set_scan_resp_data: 0x%02x\r\n", ret);
				return false;
			}
		}
	}

	return true;
}

__attribute__((weak)) bool BtAppAdvInit(const BtAppCfg_t *pCfg)
{
	if (pCfg == NULL)
	{
		return false;
	}

	BtAdvPacket_t *advpkt = g_BtAppData.bExtAdv
		? &s_BtAppExtAdvPkt : &s_BtAppAdvPkt;
	BtAdvPacket_t *srpkt  = g_BtAppData.bExtAdv
		? &s_BtAppExtSrPkt  : &s_BtAppSrPkt;

	// Reset previously-encoded content so re-init works.
	advpkt->Len = 0;
	srpkt->Len  = 0;

	// Generic AD payload encode.
	if (BtAdvEncode(pCfg, advpkt, srpkt) == false)
	{
		DEBUG_PRINTF("BtAdvEncode failed\r\n");
		return false;
	}

	// Push to the ST stack.
	return PushAdvDataToStack(pCfg, advpkt, srpkt);
}

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen,
                        uint8_t *pSrData, int SrLen)
{
	BtAdvPacket_t *advpkt = g_BtAppData.bExtAdv
		? &s_BtAppExtAdvPkt : &s_BtAppAdvPkt;
	BtAdvPacket_t *srpkt  = g_BtAppData.bExtAdv
		? &s_BtAppExtSrPkt  : &s_BtAppSrPkt;

	// Update the manuf-specific data record on the adv packet.
	if (pAdvData != NULL && AdvLen > 0)
	{
		BtAdvDataRemove(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA);
		int l = AdvLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(advpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = g_BtAppData.VendorId;
		memcpy(&p->Data[2], pAdvData, AdvLen);
	}

	// Update the manuf-specific data record on the scan response.
	if (pSrData != NULL && SrLen > 0)
	{
		BtAdvDataRemove(srpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA);
		int l = SrLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(srpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = g_BtAppData.VendorId;
		memcpy(&p->Data[2], pSrData, SrLen);
	}

	// Re-push only the data buffers (params unchanged).
	uint8_t ret;
	if (g_BtAppData.bExtAdv == false)
	{
		ret = hci_le_set_advertising_data((uint8_t)advpkt->Len, advpkt->pData);
		if (ret != BLE_STATUS_SUCCESS)
		{
			return false;
		}

		if (srpkt->Len > 0)
		{
			ret = hci_le_set_scan_response_data((uint8_t)srpkt->Len,
			                                    srpkt->pData);
			if (ret != BLE_STATUS_SUCCESS)
			{
				return false;
			}
		}
	}
	else
	{
		ret = aci_gap_adv_set_data(BT_WBA_ADV_HANDLE,
		                           BT_WBA_ADV_DATA_OP_COMPLETE,
		                           (uint8_t)advpkt->Len, advpkt->pData);
		if (ret != BLE_STATUS_SUCCESS)
		{
			return false;
		}

		if (srpkt->Len > 0)
		{
			ret = aci_gap_adv_set_scan_resp_data(BT_WBA_ADV_HANDLE,
			                                     BT_WBA_ADV_DATA_OP_COMPLETE,
			                                     (uint8_t)srpkt->Len,
			                                     srpkt->pData);
			if (ret != BLE_STATUS_SUCCESS)
			{
				return false;
			}
		}
	}

	return true;
}
