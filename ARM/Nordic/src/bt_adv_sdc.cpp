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
#include "convutil.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_appearance.h"

// Debug printf: mirrors the guard in bt_app_sdc.cpp.
//#define UART_DEBUG_ENABLE
#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
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

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
	if (g_BtAppData.State != BTAPP_STATE_ADVERTISING && g_BtAppData.State != BTAPP_STATE_IDLE)
	{
		return false;
	}

	BtAdvPacket_t *advpkt;
	BtAdvPacket_t *srpkt;

	if (g_BtAppData.bExtAdv == true)
	{
		advpkt = &s_BtDevExtAdvPkt;
		srpkt = &s_BtDevExtSrPkt;
	}
	else
	{
		advpkt = &s_BtDevAdvPkt;
		srpkt = &s_BtDevSrPkt;
	}

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

    	if (g_BtAppData.bExtAdv == true)
    	{
    		s_BtDevExtAdvData.adv_handle = 0;
    		s_BtDevExtAdvData.operation = 3;
    		s_BtDevExtAdvData.fragment_preference = 1;
    		s_BtDevExtAdvData.adv_data_length = advpkt->Len;

    		int res = sdc_hci_cmd_le_set_ext_adv_data(&s_BtDevExtAdvData);
    		if (res != 0)
    		{
    			return false;
    		}
    	}
    	else
    	{
			s_BtDevAdvData.adv_data_length = s_BtDevAdvPkt.Len;

			int res = sdc_hci_cmd_le_set_adv_data(&s_BtDevAdvData);
    		if (res != 0)
    		{
    			return false;
    		}
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
		memcpy(&p->Data[2], pAdvData, AdvLen);

    	if (g_BtAppData.bExtAdv == false)
    	{
			s_BtDevSrData.scan_response_data_length = s_BtDevSrPkt.Len;

			int res = sdc_hci_cmd_le_set_scan_response_data(&s_BtDevSrData);
    		if (res != 0)
    		{
    			return false;
    		}
    	}
	}

	return true;
}

void BtAppAdvStart()
{
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)// || g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
		return;

	int res = 0;

	if (g_BtAppData.bExtAdv == true)
	{
		uint8_t buff[100];

		sdc_hci_cmd_le_set_ext_adv_enable_t *x = (sdc_hci_cmd_le_set_ext_adv_enable_t*)buff;

		x->enable = 1;
		x->num_sets = 1;
		x->array_params[0].adv_handle = 0;
		x->array_params[0].duration = 0;
		x->array_params[0].max_ext_adv_events = 0;

		res = sdc_hci_cmd_le_set_ext_adv_enable(x);
	}
	else
	{
		sdc_hci_cmd_le_set_adv_enable_t x = { 1 };

		res = sdc_hci_cmd_le_set_adv_enable(&x);
	}

	if (res == 0)
	{
		//g_BleAppData.bAdvertising = true;
		g_BtAppData.State = BTAPP_STATE_ADVERTISING;
	}
}

void BtAppAdvStop()
{
	int res = 0;

	if (g_BtAppData.bExtAdv == true)
	{
		uint8_t buff[100];

		sdc_hci_cmd_le_set_ext_adv_enable_t *x = (sdc_hci_cmd_le_set_ext_adv_enable_t*)buff;

		x->enable = 0;
		x->num_sets = 1;
		x->array_params[0].adv_handle = 0;
		x->array_params[0].duration = 0;
		x->array_params[0].max_ext_adv_events = 0;

		res = sdc_hci_cmd_le_set_ext_adv_enable(x);
	}
	else
	{
		sdc_hci_cmd_le_set_adv_enable_t x = { 0 };

		res = sdc_hci_cmd_le_set_adv_enable(&x);
	}

	g_BtAppData.State = BTAPP_STATE_IDLE;
}

bool BtAppAdvInit(const BtAppCfg_t * const pCfg)
{
	uint16_t extprop = 0;
	BtAdvPacket_t *advpkt;
	BtAdvPacket_t *srpkt;

	if (g_BtAppData.bExtAdv)
	{
		advpkt = &s_BtDevExtAdvPkt;
		srpkt  = &s_BtDevExtSrPkt;
	}
	else
	{
		advpkt = &s_BtDevAdvPkt;
		srpkt  = &s_BtDevSrPkt;
	}

	// SDC-specific extended-adv event-prop flags based on role.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		extprop |= BTADV_EXTADV_EVT_PROP_CONNECTABLE;
	}

	// Generic AD payload encode.
	if (BtAdvEncode(pCfg, advpkt, srpkt) == false)
	{
		return false;
	}

	// SDC-specific: push to controller via HCI commands.
	if (g_BtAppData.bExtAdv == false)
	{
		sdc_hci_cmd_le_set_adv_params_t advparam = {
			.adv_interval_min  = mSecTo0_625(pCfg->AdvInterval),
			.adv_interval_max  = mSecTo0_625(pCfg->AdvInterval + 50),
			.adv_type          = BTADV_TYPE_ADV_NONCONN_IND,
			.own_address_type  = BTADDR_TYPE_PUBLIC,
			.peer_address_type = 0,
			.peer_address      = {0,},
			.adv_channel_map   = 7,
			.adv_filter_policy = 0
		};

		if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
		{
			advparam.adv_type = BTADV_TYPE_ADV_IND;
		}

		if (sdc_hci_cmd_le_set_adv_params(&advparam) != 0)
		{
			return false;
		}

		s_BtDevAdvData.adv_data_length = advpkt->Len;
		if (sdc_hci_cmd_le_set_adv_data(&s_BtDevAdvData) != 0)
		{
			return false;
		}

		if (srpkt->Len > 0)
		{
			s_BtDevSrData.scan_response_data_length = srpkt->Len;
			if (sdc_hci_cmd_le_set_scan_response_data(&s_BtDevSrData) != 0)
			{
				return false;
			}
		}
	}
	else
	{
		// Extended advertising path.
		BtExtAdvParam_t extparam = {
			.AdvHdl          = 0,
			.EvtProp         = extprop,
			.PrimIntervalMin = mSecTo0_625(pCfg->AdvInterval),
			.PrimIntervalMax = mSecTo0_625(pCfg->AdvInterval + 50),
			.PrimChanMap     = 7,
			.OwnAddrType     = BTADDR_TYPE_PUBLIC,
			.PrimPhy         = BTADV_EXTADV_PHY_1M,
			.SecondPhy       = BTADV_EXTADV_PHY_2M,
			.ScanNotifEnable = 0,
		};

		sdc_hci_cmd_le_set_ext_adv_params_t &exadvparm =
			*(sdc_hci_cmd_le_set_ext_adv_params_t*)&extparam;
		sdc_hci_cmd_le_set_ext_adv_params_return_t rexadvparm;
		if (sdc_hci_cmd_le_set_ext_adv_params(&exadvparm, &rexadvparm) != 0)
		{
			return false;
		}

		s_BtDevExtAdvData.adv_handle         = 0;
		s_BtDevExtAdvData.operation          = 3;
		s_BtDevExtAdvData.fragment_preference = 1;
		s_BtDevExtAdvData.adv_data_length    = advpkt->Len;
		if (sdc_hci_cmd_le_set_ext_adv_data(&s_BtDevExtAdvData) != 0)
		{
			return false;
		}

		if (srpkt->Len > 0)
		{
			s_BtDevExtSrData.adv_handle              = 0;
			s_BtDevExtSrData.operation               = 3;
			s_BtDevExtSrData.fragment_preference     = 1;
			s_BtDevExtSrData.scan_response_data_length = advpkt->Len;
			if (sdc_hci_cmd_le_set_ext_scan_response_data(&s_BtDevExtSrData) != 0)
			{
				return false;
			}
		}
	}

	DEBUG_PRINTF("BtAppAdvInit returns true\r\n");
	return true;
}
