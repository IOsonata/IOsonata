/**-------------------------------------------------------------------------
@file	bt_adv_stm32wba.cpp

@brief	STM32WBAxx BLE port - advertising path.

        Uses the generic BtAdvEncode helper and ST advertising-set commands.
        The legacy-PDU property preserves legacy packets while the stack owns
        advertising duration and reports advertising-set termination.
        Adv-data buffers are owned by this TU.

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

#include "bt_wba_stack.h"

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
#define BT_WBA_MSEC_TO_10MS(ms)			((ms) / 10U + ((ms) % 10U != 0U))

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

// Secondary PHY for extended advertising.
#define BT_WBA_PHY_2M					0x02

// --- Adv packet buffers ---
//
// Each pair (legacy + extended) shares the same underlying byte buffer;
// the BtAdvPacket_t.MaxLen field caps each into the right window. This
// mirrors bt_adv_bm.cpp - lets app code reuse the buffer between modes.
static bool s_bAdvertising;
static uint16_t s_AdvDuration;
static uint16_t s_PeripheralLinkCount;

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

// The command family is always advertising-set; bExtAdv selects the PDU.
static uint16_t MapAdvEvtProps(uint8_t Role)
{
	uint16_t prop = g_BtAppData.bExtAdv ? 0 : BT_WBA_EVT_PROP_LEGACY;

	if (Role & BTAPP_ROLE_PERIPHERAL)
	{
		prop |= BT_WBA_EVT_PROP_CONNECTABLE;
		if (!g_BtAppData.bExtAdv)
		{
			prop |= BT_WBA_EVT_PROP_SCANNABLE;
		}
	}
	return prop;
}

static uint16_t BtAdvPeripheralConnCount(void)
{
	uint16_t count = 0;
	uint16_t peerCount = BtPeerCount();

	for (uint16_t i = 0; i < peerCount; i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer != nullptr && pPeer->Conn.Hdl != BT_CONN_HDL_INVALID &&
			pPeer->Conn.Role == BT_CONN_ROLE_PERIPHERAL)
		{
			count++;
		}
	}

	return count;
}

static bool BtAdvPeerSlotAvailable(void)
{
	uint16_t peerCount = BtPeerCount();

	for (uint16_t i = 0; i < peerCount; i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer != nullptr && pPeer->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			return true;
		}
	}

	return false;
}

static bool BtAdvCanStart(void)
{
	uint8_t role = g_BtAppData.AppDevice.Conn.Role;

	if ((role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER)) == 0)
	{
		return false;
	}

	if (role & BTAPP_ROLE_PERIPHERAL)
	{
		return s_PeripheralLinkCount != 0 &&
			BtAdvPeerSlotAvailable() &&
			BtAdvPeripheralConnCount() < s_PeripheralLinkCount;
	}

	return true;
}

static void BtAdvStateUpdate(void)
{
	if (BtPeerIsConnected())
	{
		g_BtAppData.State = BTAPP_STATE_CONNECTED;
	}
	else
	{
		g_BtAppData.State = s_bAdvertising ?
			BTAPP_STATE_ADVERTISING : BTAPP_STATE_IDLE;
	}
}

// A connectable advertising set stops when it creates a peripheral link.
// Record that controller-side transition without sending a redundant disable.
void BtAdvWbaConnected(void)
{
	s_bAdvertising = false;
	BtAdvStateUpdate();
}

void BtAdvStart(void)
{
	if (s_bAdvertising || !BtAdvCanStart())
	{
		BtAdvStateUpdate();
		return;
	}

	Adv_Set_t advSet = {
		.Advertising_Handle = BT_WBA_ADV_HANDLE,
		.Duration = s_AdvDuration,
		.Max_Extended_Advertising_Events = 0,
	};
	uint8_t ret = aci_gap_adv_set_enable(1, 1, &advSet);

	if (ret == BLE_STATUS_SUCCESS)
	{
		s_bAdvertising = true;
		BtAdvStateUpdate();
	}
	else
	{
		DEBUG_PRINTF("BtAdvStart failed: 0x%02x\r\n", ret);
	}
}

void BtAdvStop(void)
{
	if (!s_bAdvertising)
	{
		BtAdvStateUpdate();
		return;
	}

	Adv_Set_t advSet = {
		.Advertising_Handle = BT_WBA_ADV_HANDLE,
		.Duration = 0,
		.Max_Extended_Advertising_Events = 0,
	};
	uint8_t ret = aci_gap_adv_set_enable(0, 1, &advSet);
	if (ret != BLE_STATUS_SUCCESS)
	{
		DEBUG_PRINTF("BtAdvStop failed: 0x%02x\r\n", ret);
		return;
	}

	s_bAdvertising = false;
	BtAdvStateUpdate();
}

// Push the encoded packet pair using the same command family for both PDUs.
static bool PushAdvDataToStack(const BtAppCfg_t *pCfg,
							   BtAdvPacket_t *pAdv, BtAdvPacket_t *pSr)
{
	uint8_t zeros[6] = {0,};
	uint32_t interval = BT_WBA_MSEC_TO_0_625(pCfg->AdvInterval);
	uint8_t ret = aci_gap_adv_set_configuration(
		0, BT_WBA_ADV_HANDLE, MapAdvEvtProps(pCfg->Role),
		interval, interval, BT_WBA_ADV_CHAN_MAP_ALL,
		0, 0, zeros, 0, 0x7F, 0,
		g_BtAppData.bExtAdv ? BT_WBA_PHY_2M : 0x01,
		0, 0);
	if (ret != BLE_STATUS_SUCCESS)
	{
		DEBUG_PRINTF("aci_gap_adv_set_configuration: 0x%02x\r\n", ret);
		return false;
	}

	ret = aci_gap_adv_set_adv_data(BT_WBA_ADV_HANDLE,
		BT_WBA_ADV_DATA_OP_COMPLETE, BT_WBA_ADV_DATA_FRAG_PREF_NONE,
		(uint8_t)pAdv->Len, pAdv->pData);
	if (ret != BLE_STATUS_SUCCESS)
	{
		return false;
	}

	if (pSr->Len > 0)
	{
		ret = aci_gap_adv_set_scan_resp_data(BT_WBA_ADV_HANDLE,
			BT_WBA_ADV_DATA_OP_COMPLETE, BT_WBA_ADV_DATA_FRAG_PREF_NONE,
			(uint8_t)pSr->Len, pSr->pData);
		if (ret != BLE_STATUS_SUCCESS)
		{
			return false;
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

	// Give the encoder the full packet capacities. It performs its own legacy
	// attempt with 31-octet limits and reports the selected PDU mode.
	BtAdvPacket_t *advpkt = &s_BtAppExtAdvPkt;
	BtAdvPacket_t *srpkt = &s_BtAppExtSrPkt;
	bool bScannable = false;

	s_bAdvertising = false;
	uint32_t duration = BT_WBA_MSEC_TO_10MS(pCfg->AdvTimeout);
	s_AdvDuration = duration > UINT16_MAX ? UINT16_MAX : (uint16_t)duration;
	s_PeripheralLinkCount = pCfg->CentralDevMax > 0 ?
		(uint16_t)pCfg->CentralDevMax : 0;
	advpkt->Len = 0;
	srpkt->Len = 0;

	if (BtAdvEncode(pCfg, advpkt, srpkt, &g_BtAppData.bExtAdv,
					&bScannable) == false)
	{
		DEBUG_PRINTF("BtAdvEncode failed\r\n");
		return false;
	}

	// Legacy and extended packet views share storage. Preserve the encoded
	// lengths in the legacy views so later manufacturer-data updates select
	// the same packet state.
	if (g_BtAppData.bExtAdv == false)
	{
		s_BtAppAdvPkt.Len = advpkt->Len;
		s_BtAppSrPkt.Len = srpkt->Len;
		advpkt = &s_BtAppAdvPkt;
		srpkt = &s_BtAppSrPkt;
	}

	(void)bScannable;
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
		*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
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
		*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
		memcpy(&p->Data[2], pSrData, SrLen);
	}

	// Update data through the same advertising-set commands used at init.
	uint8_t ret;

	ret = aci_gap_adv_set_adv_data(BT_WBA_ADV_HANDLE,
								BT_WBA_ADV_DATA_OP_COMPLETE,
								BT_WBA_ADV_DATA_FRAG_PREF_NONE,
								(uint8_t)advpkt->Len, advpkt->pData);
	if (ret != BLE_STATUS_SUCCESS)
	{
		return false;
	}

	if (srpkt->Len > 0)
	{
		ret = aci_gap_adv_set_scan_resp_data(BT_WBA_ADV_HANDLE,
									  BT_WBA_ADV_DATA_OP_COMPLETE,
									  BT_WBA_ADV_DATA_FRAG_PREF_NONE,
									  (uint8_t)srpkt->Len,
									  srpkt->pData);
		if (ret != BLE_STATUS_SUCCESS)
		{
			return false;
		}
	}

	return true;
}
