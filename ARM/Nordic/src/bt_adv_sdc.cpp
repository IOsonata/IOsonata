/**-------------------------------------------------------------------------
@file	bt_adv_sdc.cpp

@brief	SoftDevice Controller (nrfxlib SDC) advertising functions.

        Extracted from bt_app_sdc.cpp in step 4 of the Voci refactor.
        Owns the advertising packet buffers (overlaid with HCI command
        structures) and the BtAppAdv* function family.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "sdc.h"
#include "sdc_soc.h"
#include "sdc_hci_cmd_le.h"
#include "sdc_hci.h"
#include "sdc_hci_vs.h"
#include "sdc_hci_cmd_controller_baseband.h"

#include "istddef.h"
#include "idelay.h"
#include "convutil.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_appearance.h"

// Debug printf via SysLog. Enable UART_DEBUG_ENABLE and call
// SysLogGetInstance()->Init(g_Uart) in the application to see output.
//#define UART_DEBUG_ENABLE
#ifdef UART_DEBUG_ENABLE
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

// --- Adv packet buffers (overlaid with HCI command structs) ---

#if 1
alignas(4) static uint8_t s_BtDevAdvBuff[260];
alignas(4) static sdc_hci_cmd_le_set_adv_data_t &s_BtDevAdvData = *(sdc_hci_cmd_le_set_adv_data_t*)s_BtDevAdvBuff;
alignas(4) static BtAdvPacket_t s_BtDevAdvPkt = { sizeof(s_BtDevAdvData.adv_data), 0, s_BtDevAdvData.adv_data};

alignas(4) static sdc_hci_cmd_le_set_ext_adv_data_t &s_BtDevExtAdvData = *(sdc_hci_cmd_le_set_ext_adv_data_t*)s_BtDevAdvBuff;
alignas(4) static BtAdvPacket_t s_BtDevExtAdvPkt = { 255, 0, s_BtDevExtAdvData.adv_data};

alignas(4) static uint8_t s_BtDevSrBuff[260];

alignas(4) static sdc_hci_cmd_le_set_scan_response_data_t &s_BtDevSrData = *(sdc_hci_cmd_le_set_scan_response_data_t*)s_BtDevSrBuff;
alignas(4) static BtAdvPacket_t s_BtDevSrPkt = { sizeof(s_BtDevSrData.scan_response_data), 0, s_BtDevSrData.scan_response_data};

alignas(4) static sdc_hci_cmd_le_set_ext_scan_response_data_t &s_BtDevExtSrData = *(sdc_hci_cmd_le_set_ext_scan_response_data_t*)s_BtDevSrBuff;
alignas(4) static BtAdvPacket_t s_BtDevExtSrPkt = { 255, 0, s_BtDevExtSrData.scan_response_data};
#endif

// Advertising duration in 10 ms units, cached at init for the enable command.
// 0 means no timeout.
static uint16_t s_BtDevAdvDuration = 0;

static int BtAppAdvEnable(void);
static int BtAppAdvDisable(void);

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
	if (g_BtAppData.State != BTAPP_STATE_ADVERTISING && g_BtAppData.State != BTAPP_STATE_IDLE)
	{
		return false;
	}

	BtAdvPacket_t *advpkt = &s_BtDevExtAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BtDevExtSrPkt;

	if (pAdvData)
	{
		int l = AdvLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
		memcpy(&p->Data[2], pAdvData, AdvLen);

		s_BtDevExtAdvData.adv_handle          = 0;
		s_BtDevExtAdvData.operation           = 3;
		s_BtDevExtAdvData.fragment_preference = 1;
		s_BtDevExtAdvData.adv_data_length     = advpkt->Len;

		if (sdc_hci_cmd_le_set_ext_adv_data(&s_BtDevExtAdvData) != 0)
		{
			return false;
		}
	}

	if (pSrData)
	{
		int l = SrLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(srpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
		memcpy(&p->Data[2], pSrData, SrLen);

		s_BtDevExtSrData.adv_handle               = 0;
		s_BtDevExtSrData.operation                = 3;
		s_BtDevExtSrData.fragment_preference      = 1;
		s_BtDevExtSrData.scan_response_data_length = srpkt->Len;

		if (sdc_hci_cmd_le_set_ext_scan_response_data(&s_BtDevExtSrData) != 0)
		{
			return false;
		}
	}

	// Re-enable advertising with the updated data. Disable first so the enable
	// is not rejected with Command Disallowed (0x0C) when the set is still
	// considered enabled. Mirrors the nRF52 stop-then-start on data update.
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)
	{
		BtAppAdvDisable();
		BtAppAdvEnable();
	}

	return true;
}

// Issue the extended advertising enable command for adv set 0 with the cached
// duration. Safe to call whether the set is currently enabled (the controller
// resets the duration timer) or stopped (it restarts). Returns 0 on success.
static int BtAppAdvEnable(void)
{
	uint8_t buff[sizeof(sdc_hci_cmd_le_set_ext_adv_enable_t) + sizeof(sdc_hci_le_set_ext_adv_enable_array_params_t)];

	sdc_hci_cmd_le_set_ext_adv_enable_t *x = (sdc_hci_cmd_le_set_ext_adv_enable_t*)buff;

	x->enable = 1;
	x->num_sets = 1;
	x->array_params[0].adv_handle = 0;
	x->array_params[0].duration = s_BtDevAdvDuration;
	x->array_params[0].max_ext_adv_events = 0;

	return sdc_hci_cmd_le_set_ext_adv_enable(x);
}

// Disable advertising for adv set 0 (controller command only, does not change
// app State). Disabling an already-disabled set has no effect.
static int BtAppAdvDisable(void)
{
	uint8_t buff[sizeof(sdc_hci_cmd_le_set_ext_adv_enable_t) + sizeof(sdc_hci_le_set_ext_adv_enable_array_params_t)];

	sdc_hci_cmd_le_set_ext_adv_enable_t *x = (sdc_hci_cmd_le_set_ext_adv_enable_t*)buff;

	x->enable = 0;
	x->num_sets = 1;
	x->array_params[0].adv_handle = 0;
	x->array_params[0].duration = 0;
	x->array_params[0].max_ext_adv_events = 0;

	return sdc_hci_cmd_le_set_ext_adv_enable(x);
}

void BtAppAdvStart()
{
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)
		return;

	if (BtAppAdvEnable() == 0)
	{
		g_BtAppData.State = BTAPP_STATE_ADVERTISING;
	}
}

void BtAppAdvStop()
{
	uint8_t buff[sizeof(sdc_hci_cmd_le_set_ext_adv_enable_t) + sizeof(sdc_hci_le_set_ext_adv_enable_array_params_t)];

	sdc_hci_cmd_le_set_ext_adv_enable_t *x = (sdc_hci_cmd_le_set_ext_adv_enable_t*)buff;

	x->enable = 0;
	x->num_sets = 1;
	x->array_params[0].adv_handle = 0;
	x->array_params[0].duration = 0;
	x->array_params[0].max_ext_adv_events = 0;

	sdc_hci_cmd_le_set_ext_adv_enable(x);

	g_BtAppData.State = BTAPP_STATE_IDLE;
}

bool BtAppAdvInit(const BtAppCfg_t * const pCfg)
{
	BtAdvPacket_t *advpkt = &s_BtDevExtAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BtDevExtSrPkt;

	// Encode the AD payload. BtAdvEncode decides legacy vs extended from how the
	// records pack, and reports it via bExtAdv/scannable.
	bool scannable = false;

	if (BtAdvEncode(pCfg, advpkt, srpkt, &g_BtAppData.bExtAdv, &scannable) == false)
	{
		return false;
	}

	// The SDC reaches both legacy and extended advertising through the extended
	// HCI commands. Legacy advertising PDUs (so older centrals can see the set)
	// are selected with the legacy event-property bit; this also makes the
	// advertising duration / timeout available, which the old legacy enable
	// command does not provide.
	uint16_t extprop = 0;

	if (g_BtAppData.bExtAdv == false)
	{
		extprop |= BTADV_EXTADV_EVT_PROP_LEGACY;
	}

	// Role dictates connectable / scannable per the validated mapping:
	//   peripheral legacy   -> connectable + scannable (ADV_IND)
	//   peripheral extended -> connectable only (extended cannot be scannable)
	//   broadcaster         -> neither (non-connectable non-scannable)
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		extprop |= BTADV_EXTADV_EVT_PROP_CONNECTABLE;
	}

	if (scannable)
	{
		extprop |= BTADV_EXTADV_EVT_PROP_SCANNABLE;
	}

	// Populate the SDC extended advertising parameters struct directly. Casting
	// BtExtAdvParam_t onto it does not work: the bitfield layout differs from
	// the packed HCI struct, which misaligns the PHY and trailing fields.
	sdc_hci_cmd_le_set_ext_adv_params_t exadvparm;
	memset(&exadvparm, 0, sizeof(exadvparm));
	exadvparm.adv_handle = 0;
	exadvparm.adv_event_properties.raw[0] = (uint8_t)(extprop & 0xFF);
	exadvparm.adv_event_properties.raw[1] = (uint8_t)((extprop >> 8) & 0xFF);
	exadvparm.primary_adv_interval_min = mSecTo0_625(pCfg->AdvInterval);
	exadvparm.primary_adv_interval_max = mSecTo0_625(pCfg->AdvInterval + 50);
	exadvparm.primary_adv_channel_map  = 7;
	exadvparm.own_address_type         = BTADDR_TYPE_RAND;
	exadvparm.peer_address_type        = 0;
	exadvparm.adv_filter_policy        = 0;
	exadvparm.adv_tx_power             = 0;
	exadvparm.primary_adv_phy          = BTADV_EXTADV_PHY_1M;
	exadvparm.secondary_adv_max_skip   = 0;
	exadvparm.secondary_adv_phy        = BTADV_EXTADV_PHY_2M;
	exadvparm.adv_sid                  = 0;
	exadvparm.scan_request_notification_enable = 0;

	sdc_hci_cmd_le_set_ext_adv_params_return_t rexadvparm;
	int res = sdc_hci_cmd_le_set_ext_adv_params(&exadvparm, &rexadvparm);
	DEBUG_PRINTF("set_ext_adv_params: res=%d extprop=%x\r\n", res, extprop);
	if (res != 0)
	{
		return false;
	}

	// Extended advertising with a random own-address type requires the random
	// address to be set per advertising set; the legacy LE Set Random Address
	// command does not configure it. Use the device's configured random address.
	{
		uint8_t atype = 0;
		sdc_hci_cmd_le_set_adv_set_random_address_t ranaddr;
		ranaddr.adv_handle = 0;
		BtSmpLocalAddrGet(&atype, ranaddr.random_address);
		res = sdc_hci_cmd_le_set_adv_set_random_address(&ranaddr);
		DEBUG_PRINTF("set_adv_set_rand_addr: res=%d\r\n", res);
		if (res != 0)
		{
			return false;
		}
	}

	s_BtDevExtAdvData.adv_handle          = 0;
	s_BtDevExtAdvData.operation           = 3;
	s_BtDevExtAdvData.fragment_preference = 1;
	s_BtDevExtAdvData.adv_data_length     = advpkt->Len;
	res = sdc_hci_cmd_le_set_ext_adv_data(&s_BtDevExtAdvData);
	DEBUG_PRINTF("set_ext_adv_data: res=%d len=%d\r\n", res, advpkt->Len);
	if (res != 0)
	{
		return false;
	}

	// Scan response data only exists for a scannable set; setting it on a
	// non-scannable set is rejected by the controller.
	if (scannable && srpkt->Len > 0)
	{
		s_BtDevExtSrData.adv_handle               = 0;
		s_BtDevExtSrData.operation                = 3;
		s_BtDevExtSrData.fragment_preference      = 1;
		s_BtDevExtSrData.scan_response_data_length = srpkt->Len;
		res = sdc_hci_cmd_le_set_ext_scan_response_data(&s_BtDevExtSrData);
		DEBUG_PRINTF("set_ext_scan_resp: res=%d len=%d\r\n", res, srpkt->Len);
		if (res != 0)
		{
			return false;
		}
	}

	// Cache the advertising duration (10 ms units) for the enable command.
	s_BtDevAdvDuration = mSecTo10Ms(pCfg->AdvTimeout);

	DEBUG_PRINTF("BtAppAdvInit returns true\r\n");
	return true;
}
