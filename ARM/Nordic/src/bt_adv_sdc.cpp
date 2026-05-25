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
		*(uint16_t *)p->Data = g_BtAppData.VendorId;
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
		*(uint16_t *)p->Data = g_BtAppData.VendorId;
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
	uint8_t flags = BT_GAP_DATA_TYPE_FLAGS_NO_BREDR;
	uint16_t extprop = 0;//BLE_EXT_ADV_EVT_PROP_LEGACY;
	BtAdvPacket_t *advpkt;
	BtAdvPacket_t *srpkt;

	//g_BleAppTimer.Init(s_TimerCfg);

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

	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		if (pCfg->AdvTimeout != 0)
		{
			flags |= BT_GAP_DATA_TYPE_FLAGS_LIMITED_DISCOVERABLE;
		}
		else
		{
			flags |= BT_GAP_DATA_TYPE_FLAGS_GENERAL_DISCOVERABLE;
		}
		extprop |= BTADV_EXTADV_EVT_PROP_CONNECTABLE;// | BLE_EXT_ADV_EVT_PROP_SCANNABLE;
	}
	else if (pCfg->Role & BTAPP_ROLE_BROADCASTER)
	{
		//extprop |= BLE_EXT_ADV_EVT_PROP_OMIT_ADDR;
		//extprop |= BLE_EXT_ADV_EVT_PROP_SCANNABLE;
		//extprop = 0;
		//flags |= GAP_DATA_TYPE_FLAGS_LIMITED_DISCOVERABLE;
	}

	if (BtAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_FLAGS, &flags, 1) == false)
	{
		return false;
	}

    if (pCfg->Appearance != BT_APPEAR_UNKNOWN_GENERIC)
    {
    	if (BtAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_APPEARANCE, (uint8_t*)&pCfg->Appearance, 2) == false)
    	{
    		return false;
    	}
    }

    BtAdvPacket_t *uidadvpkt;

    if (pCfg->pDevName != NULL)
    {
    	size_t l = strlen(pCfg->pDevName);
    	uint8_t type = BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME;
    	size_t mxl = advpkt->MaxLen - advpkt->Len - 2;

    	if (l > 30 || l > mxl)
    	{
    		// Short name
    		type = BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME;
    		l = min((size_t)30, mxl);
    	}

    	if (BtAdvDataAdd(advpkt, type, (uint8_t*)pCfg->pDevName, l) == false)
    	{
    		return false;
    	}

    	uidadvpkt = pCfg->bExtAdv ? advpkt : srpkt;
    }
    else
    {
    	uidadvpkt = advpkt;
    }

    if (pCfg->pAdvUuid != NULL && pCfg->Role & BTAPP_ROLE_PERIPHERAL)
    {
    	if (BtAdvDataAddUuid(uidadvpkt, pCfg->pAdvUuid, pCfg->bCompleteUuidList) == false)
    	{
    		return false;
    	}

    }

	if (pCfg->pAdvManData != NULL)
	{
		int l = pCfg->AdvManDataLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = pCfg->VendorId;
		memcpy(&p->Data[2], pCfg->pAdvManData, pCfg->AdvManDataLen);
	}

	if (pCfg->pSrManData != NULL)
	{
		int l = pCfg->SrManDataLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(srpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = pCfg->VendorId;
		memcpy(&p->Data[2], pCfg->pSrManData, pCfg->SrManDataLen);
	}

	if (g_BtAppData.bExtAdv == false)
	{
		sdc_hci_cmd_le_set_adv_params_t advparam = {
			.adv_interval_min = mSecTo0_625(pCfg->AdvInterval),
			.adv_interval_max = mSecTo0_625(pCfg->AdvInterval + 50),
			.adv_type = BTADV_TYPE_ADV_NONCONN_IND,//ADV_DIRECT_IND,
			.own_address_type = BTADDR_TYPE_PUBLIC,
			.peer_address_type = 0,
			.peer_address = {0,},
			.adv_channel_map = 7,
			.adv_filter_policy = 0
		};

		if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
		{
			advparam.adv_type = BTADV_TYPE_ADV_IND;
		}

		int sdc_res = sdc_hci_cmd_le_set_adv_params(&advparam);

		if (sdc_res != 0)
		{
			return false;
		}

		s_BtDevAdvData.adv_data_length = advpkt->Len;

		sdc_res = sdc_hci_cmd_le_set_adv_data(&s_BtDevAdvData);
		if (sdc_res != 0)
		{
			return false;
		}

		if (srpkt->Len > 0)
		{
			s_BtDevSrData.scan_response_data_length = srpkt->Len;

			sdc_res = sdc_hci_cmd_le_set_scan_response_data(&s_BtDevSrData);
			if (sdc_res != 0)
			{
				return false;
			}
		}
	}
	else
	{
		// Use extended advertisement

		BtExtAdvParam_t extparam = {
			.AdvHdl = 0,
			.EvtProp = extprop,//BLE_EXT_ADV_EVT_PROP_CONNECTABLE,// | BLE_EXT_ADV_EVT_PROP_SCANNABLE,
			.PrimIntervalMin = mSecTo0_625(pCfg->AdvInterval),
			.PrimIntervalMax = mSecTo0_625(pCfg->AdvInterval + 50),
			.PrimChanMap = 7,
			.OwnAddrType = BTADDR_TYPE_PUBLIC,
			.PrimPhy = BTADV_EXTADV_PHY_1M,
			.SecondPhy = BTADV_EXTADV_PHY_2M,
			.ScanNotifEnable = 0,
		};

		sdc_hci_cmd_le_set_ext_adv_params_t &exadvparm = *(sdc_hci_cmd_le_set_ext_adv_params_t*)&extparam;
		sdc_hci_cmd_le_set_ext_adv_params_return_t rexadvparm;
		int res = sdc_hci_cmd_le_set_ext_adv_params(&exadvparm, &rexadvparm);

		if (res != 0)
		{
			return false;
//			printf("sdc_hci_cmd_le_set_ext_adv_params : %x\n", res);
		}

		s_BtDevExtAdvData.adv_handle = 0;
		s_BtDevExtAdvData.operation = 3;
		s_BtDevExtAdvData.fragment_preference = 1;
		s_BtDevExtAdvData.adv_data_length = advpkt->Len;

		res = sdc_hci_cmd_le_set_ext_adv_data(&s_BtDevExtAdvData);
		if (res != 0)
		{
			return false;
//			printf("sdc_hci_cmd_le_set_ext_adv_data : %x\n", res);
		}

		if (srpkt->Len > 0)
		{
			s_BtDevExtSrData.adv_handle = 0;
			s_BtDevExtSrData.operation = 3;
			s_BtDevExtSrData.fragment_preference = 1;
			s_BtDevExtSrData.scan_response_data_length = advpkt->Len;

			res = sdc_hci_cmd_le_set_ext_scan_response_data(&s_BtDevExtSrData);
			if (res != 0)
			{
				return false;
//				printf("sdc_hci_cmd_le_set_ext_adv_data : %x\n", res);
			}
		}
	}

	DEBUG_PRINTF("BtAppAdvInit returns true\r\n");
	return true;
}
