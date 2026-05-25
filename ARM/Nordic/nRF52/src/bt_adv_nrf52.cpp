/**-------------------------------------------------------------------------
@file	bt_adv_nrf52.cpp

@brief	nRF5_SDK SoftDevice advertising functions.

        Extracted from bt_app_nrf52.cpp in step 4 of the Voci refactor.
        Owns the advertising packet buffers, the SoftDevice adv data struct,
        and the BtAdv* / BtAppAdv* function family.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2016, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "sdk_config.h"
#include "nordic_common.h"
#include "ble_hci.h"
#include "nrf_error.h"
#include "ble_gatt.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "nrf_ble_gatt.h"
#include "app_util_platform.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_error.h"
#include "compiler_abstraction.h"

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_appearance.h"

// Port-local: SoftDevice BLE configuration tag. Must match the value used
// by BtAppStackInit when calling sd_ble_cfg_set / sd_ble_gap_adv_start.
#define BTAPP_CONN_CFG_TAG            1

// --- Adv packet buffers and SoftDevice adv data ---
// File-scope: nothing outside bt_adv_nrf52.cpp needs these.

#pragma pack(push, 4)
typedef struct __Bt_App_Nrf52_Data {
	ble_gap_adv_params_t AdvParam;		//!< SoftDevice adv params
} BtAppNrf52Data_t;
#pragma pack(pop)

static BtAppNrf52Data_t s_Nrf52Data = { {0} };

alignas(4) static uint8_t s_BleAppAdvBuff[256];
alignas(4) static BtAdvPacket_t s_BleAppAdvPkt    = { 31, 0, s_BleAppAdvBuff };
alignas(4) static BtAdvPacket_t s_BleAppExtAdvPkt = { 255, 0, s_BleAppAdvBuff };

alignas(4) static uint8_t s_BleAppSrBuff[256];
alignas(4) static BtAdvPacket_t s_BleAppSrPkt    = { 31, 0, s_BleAppSrBuff };
alignas(4) static BtAdvPacket_t s_BleAppExtSrPkt = { 255, 0, s_BleAppSrBuff };

static ble_gap_adv_data_t s_BtAppAdvData = {
	.adv_data      = { s_BleAppAdvBuff, 0 },
	.scan_rsp_data = { s_BleAppSrBuff, 0 }
};

void BtAdvStart()//BLEAPP_ADVMODE AdvMode)
{
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING || g_BtAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
		return;

//	g_BleAppData.bAdvertising = true;
#if 1
	uint32_t err_code = sd_ble_gap_adv_start(g_BtAppData.AdvHdl, BTAPP_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

#else
//	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
	//if (g_BleAppData.AdvType == BLEADV_TYPE_ADV_NONCONN_IND)
	if (g_BleAppData.Role & BLEAPP_ROLE_BROADCASTER)
	{
		uint32_t err_code = sd_ble_gap_adv_start(g_AdvInstance.adv_handle, g_AdvInstance.conn_cfg_tag);
        APP_ERROR_CHECK(err_code);
	}
	else
	{
		uint32_t err_code = ble_advertising_start(&g_AdvInstance, (ble_adv_mode_t)BLEAPP_ADVMODE_FAST);//AdvMode);
		if (err_code != NRF_SUCCESS)
		{
			 APP_ERROR_CHECK(err_code);
		}
	}
#endif

	g_BtAppData.State = BTAPP_STATE_ADVERTISING;
}

void BtAdvStop()
{
	sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
	g_BtAppData.State = BTAPP_STATE_IDLE;
}

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
//	return BtDevAdvManDataSet(pAdvData, AdvLen, pSrData, SrLen);
	if (g_BtAppData.State != BTAPP_STATE_ADVERTISING && g_BtAppData.State != BTAPP_STATE_IDLE)
	{
		return false;
	}

	BtAdvPacket_t *advpkt;
	BtAdvPacket_t *srpkt;

	if (g_BtAppData.bExtAdv == true)
	{
		advpkt = &s_BleAppExtAdvPkt;
		srpkt = &s_BleAppExtSrPkt;
	}
	else
	{
		advpkt = &s_BleAppAdvPkt;
		srpkt = &s_BleAppSrPkt;
	}

	if (g_BtAppData.bExtAdv == false)
	{
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

			s_BtAppAdvData.adv_data.len = advpkt->Len;
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

			s_BtAppAdvData.scan_rsp_data.len = srpkt->Len;
		}
	}
	else
	{
		// Ext Adv
		int l = 2;

		if (pAdvData)
		{
			l += AdvLen;
		}

		if (pSrData)
		{
			l += SrLen;
		}

		if (l > 2)
		{
			BtAdvData_t *p = BtAdvDataAllocate(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

			if (p == NULL)
			{
				return false;
			}
			*(uint16_t *)p->Data = g_BtAppData.VendorId;
			memcpy(&p->Data[2], pAdvData, AdvLen);
			memcpy(&p->Data[2 + AdvLen], pSrData, SrLen);
			s_BtAppAdvData.adv_data.len = advpkt->Len;
		}
	}
	// SDK15 doesn't allow dynamically updating adv data.  Have to stop and re-start advertising
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)
	{
		sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
	}

	uint32_t err = sd_ble_gap_adv_set_configure(&g_BtAppData.AdvHdl, &s_BtAppAdvData, NULL);
//	APP_ERROR_CHECK(err);

	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)
	{
		g_BtAppData.State = BTAPP_STATE_IDLE;
		BtAdvStart();//BLEAPP_ADVMODE_FAST);
	}

#if 0
    int l = min(Len, BLE_GAP_ADV_MAX_SIZE);

    memcpy(g_AdvInstance.manuf_data_array, pData, l);
    uint32_t ret = ble_advdata_set(&(g_AdvInstance.advdata), &g_BleAppData.SRData);
#endif

    return g_BtAppData.State == BTAPP_STATE_ADVERTISING;
}

/**@brief Overloadable function for initializing the Advertising functionality.
 */
__WEAK bool BtAppAdvInit(const BtAppCfg_t *pCfg)
{
    uint32_t               err_code;
    ble_advertising_init_t	initdata;
    uint8_t flags = BT_GAP_DATA_TYPE_FLAGS_NO_BREDR;
	BtAdvPacket_t *advpkt;
	BtAdvPacket_t *srpkt;
	uint8_t proptype = 0;

	memset(&s_Nrf52Data.AdvParam, 0, sizeof(ble_gap_adv_params_t));

    //memset(&initdata, 0, sizeof(ble_advertising_init_t));
    //memset(&g_AdvInstance, 0, sizeof(ble_advertising_t));

#if 1

	if (g_BtAppData.bExtAdv == true)
	{
		advpkt = &s_BleAppExtAdvPkt;
		srpkt = &s_BleAppExtSrPkt;
	}
	else
	{
		advpkt = &s_BleAppAdvPkt;
		srpkt = &s_BleAppSrPkt;
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
		s_Nrf52Data.AdvParam.properties.type = pCfg->bExtAdv ?
												BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED :
												BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
	}
	else if (pCfg->Role & BTAPP_ROLE_BROADCASTER)
	{
		s_Nrf52Data.AdvParam.properties.type = pCfg->bExtAdv ?
												BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED :
												BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
	}

	if (BtAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_FLAGS, &flags, 1) == false)
	{
		return false;
	}

    if (pCfg->Appearance != BT_APPEAR_UNKNOWN_GENERIC)
    {
    	if (BtAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_APPEARANCE, (uint8_t*)&pCfg->Appearance, 2) == false)
    	{
    		// Don't care whether we are able to add appearance or not. Appearance is optional.
    		//return false;
    	}
    }

    BtAdvPacket_t *uidadvpkt;

    if (pCfg->bExtAdv == false)
    {
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
    }
    else
    {
    	// Ext Adv
    	int l = 2;

		if (pCfg->pAdvManData != NULL)
		{
			l += pCfg->AdvManDataLen;
		}

		if (pCfg->pSrManData != NULL)
		{
			l += pCfg->SrManDataLen;
		}

		if (l > 2)
		{
			BtAdvData_t *p = BtAdvDataAllocate(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

			if (p == NULL)
			{
				return false;
			}
			*(uint16_t *)p->Data = pCfg->VendorId;
			memcpy(&p->Data[2], pCfg->pAdvManData, pCfg->AdvManDataLen);
			memcpy(&p->Data[2 + pCfg->AdvManDataLen], pCfg->pSrManData, pCfg->SrManDataLen);
		}
    }

	if (pCfg->pDevName != NULL)
    {
    	if (BtAdvDataSetDevName(advpkt, pCfg->pDevName) == false)
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

		}
    }

	s_Nrf52Data.AdvParam.p_peer_addr 	= NULL;	// Undirected advertisement.
	s_Nrf52Data.AdvParam.interval    	= MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
	s_Nrf52Data.AdvParam.duration     	= MSEC_TO_UNITS(pCfg->AdvTimeout, UNIT_10_MS);
    //g_BleAppData.AdvParam.channel_mask[5]	= (7 << 5);	// All channels
	s_Nrf52Data.AdvParam.filter_policy	= BLE_GAP_ADV_FP_ANY;
	s_Nrf52Data.AdvParam.primary_phy 	= BLE_GAP_PHY_1MBPS;
	s_Nrf52Data.AdvParam.secondary_phy	= BLE_GAP_PHY_2MBPS;

	s_BtAppAdvData.adv_data.len = advpkt->Len;
	s_BtAppAdvData.scan_rsp_data.len = srpkt->Len;

    err_code = sd_ble_gap_adv_set_configure(&g_BtAppData.AdvHdl, &s_BtAppAdvData, &s_Nrf52Data.AdvParam);
    APP_ERROR_CHECK(err_code);

#else
//    memset(&g_AdvInstance.adv_params, 0, sizeof(g_AdvInstance.adv_params));
    g_AdvInstance.conn_cfg_tag = BLEAPP_CONN_CFG_TAG;//BLE_CONN_CFG_TAG_DEFAULT;//BLEAPP_CONN_CFG_TAG;
    g_AdvInstance.adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    g_AdvInstance.adv_mode_current = BLE_ADV_MODE_IDLE;
    g_AdvInstance.adv_modes_config.ble_adv_secondary_phy= BLE_GAP_PHY_2MBPS;
    g_AdvInstance.adv_modes_config.ble_adv_primary_phy= BLE_GAP_PHY_1MBPS;
    g_AdvInstance.current_slave_link_conn_handle = BLE_CONN_HANDLE_INVALID;

    g_AdvInstance.adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
    g_AdvInstance.adv_params.secondary_phy = BLE_GAP_PHY_2MBPS;
    g_AdvInstance.evt_handler = on_adv_evt;
    //g_AdvInstance.error_handler = on_adv_error;
    g_AdvInstance.adv_params.p_peer_addr 		= NULL;                             // Undirected advertisement.
    g_AdvInstance.adv_params.filter_policy		= BLE_GAP_ADV_FP_ANY;
    g_AdvInstance.adv_params.interval    		= MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
    g_AdvInstance.adv_params.duration     		= MSEC_TO_UNITS(pCfg->AdvTimeout, UNIT_10_MS);
    g_AdvInstance.p_adv_data = &g_AdvInstance.adv_data;
    g_AdvInstance.adv_data.adv_data.p_data = g_AdvInstance.enc_advdata[0];
    g_AdvInstance.adv_data.scan_rsp_data.p_data = g_AdvInstance.enc_scan_rsp_data[0];

	g_AdvInstance.adv_modes_config.ble_adv_fast_enabled  = true;
	g_AdvInstance.adv_modes_config.ble_adv_extended_enabled = pCfg->bExtAdv;
	g_AdvInstance.adv_modes_config.ble_adv_fast_interval = MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
	g_AdvInstance.adv_modes_config.ble_adv_fast_timeout  = MSEC_TO_UNITS(pCfg->AdvTimeout, UNIT_10_MS);

	if (pCfg->AdvSlowInterval > 0)
	{
		g_AdvInstance.adv_modes_config.ble_adv_slow_enabled  = true;
		g_AdvInstance.adv_modes_config.ble_adv_slow_interval = MSEC_TO_UNITS(pCfg->AdvSlowInterval, UNIT_0_625_MS);
		g_AdvInstance.adv_modes_config.ble_adv_slow_timeout  = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
	}

	g_AdvInstance.adv_modes_config.ble_adv_extended_enabled = pCfg->bExtAdv;

	if (pCfg->bExtAdv == true)
    {
    	if (pCfg->Role & BLEAPP_ROLE_BROADCASTER)
    	{
    		g_AdvInstance.adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    	}
    	if (pCfg->Role & BLEAPP_ROLE_PERIPHERAL)
    	{
    		g_AdvInstance.adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
    	}

    	g_AdvInstance.adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED;
    	g_AdvInstance.adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED;
    }
    else
    {
    	if (pCfg->Role & BLEAPP_ROLE_BROADCASTER)
    	{
    		g_AdvInstance.adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    	}
    	if (pCfg->Role & BLEAPP_ROLE_PERIPHERAL)
    	{
    		g_AdvInstance.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    	}
    	g_AdvInstance.adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    	g_AdvInstance.adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
   }

    if (pCfg->pAdvManData != NULL)
    {
		g_BleAppData.ManufData.company_identifier = pCfg->VendorID;
		g_BleAppData.ManufData.data.p_data = (uint8_t*)pCfg->pAdvManData;
		g_BleAppData.ManufData.data.size = pCfg->AdvManDataLen;
    }
    if (pCfg->pSrManData != NULL)
    {
		g_BleAppData.SRManufData.company_identifier = pCfg->VendorID;
		g_BleAppData.SRManufData.data.p_data = (uint8_t*)pCfg->pSrManData;
		g_BleAppData.SRManufData.data.size = pCfg->SrManDataLen;
    }
    // Build advertising data struct to pass into @ref ble_advertising_init.

    if (pCfg->Appearance != BLE_APPEARANCE_UNKNOWN)
    {
    	initdata.advdata.include_appearance = true;
    }

    if (pCfg->AdvTimeout != 0)
    {
        // ADV for a limited time, use this flag
        initdata.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    }
    else
    {
        // Always ADV use this flag
        initdata.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    }

    if (pCfg->pDevName != NULL)
    {
    	if (strlen(pCfg->pDevName) > 30)
    	{
    		initdata.advdata.name_type      = BLE_ADVDATA_SHORT_NAME;
    		initdata.advdata.short_name_len = 30;//strlen(pCfg->pDevName);
    	}
    	else
    	{
    		initdata.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    		initdata.advdata.short_name_len = strlen(pCfg->pDevName);
    	}
    }
    else
    {
    		initdata.advdata.name_type = BLE_ADVDATA_NO_NAME;
    }

	ble_uuid_t uid[MAX_ADV_UUID];

	if (pCfg->pAdvUuid != NULL)
    {
		if (pCfg->pAdvUuid->Type == BLE_UUID_TYPE_16)
		{
			uint8_t utype = pCfg->pAdvUuid->BaseIdx == 0 ?
							BLE_UUID_TYPE_BLE : BLE_UUID_TYPE_VENDOR_BEGIN;
			for (int i = 0; i < min(pCfg->pAdvUuid->Count, MAX_ADV_UUID); i++)
			{
				uid[i].uuid = pCfg->pAdvUuid->Val[i].Uuid16;
				uid[i].type = utype;
			}
		}
    }

	if (pCfg->bExtAdv)
	{
        if (pCfg->pAdvUuid != NULL)
        {
			if (pCfg->bCompleteUuidList)
			{
				initdata.advdata.uuids_complete.uuid_cnt = pCfg->pAdvUuid->Count;
				initdata.advdata.uuids_complete.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
			}
			else
			{
				initdata.advdata.uuids_more_available.uuid_cnt = pCfg->pAdvUuid->Count;
				initdata.advdata.uuids_more_available.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
			}
        }
		if (pCfg->pAdvManData != NULL)
		{
			initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
		}
	}
	else
	{
		if (initdata.advdata.name_type == BLE_ADVDATA_NO_NAME)
		{
			if (pCfg->pAdvUuid != NULL)
			{
				if (pCfg->pAdvManData != NULL)
				{
					initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;

					if (pCfg->bCompleteUuidList)
					{
						initdata.srdata.uuids_complete.uuid_cnt = pCfg->pAdvUuid->Count;
						initdata.srdata.uuids_complete.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
					}
					else
					{
						initdata.srdata.uuids_more_available.uuid_cnt = pCfg->pAdvUuid->Count;
						initdata.srdata.uuids_more_available.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
					}
				}
				else
				{
					if (pCfg->bCompleteUuidList)
					{
						initdata.advdata.uuids_complete.uuid_cnt = pCfg->pAdvUuid->Count;
						initdata.advdata.uuids_complete.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
					}
					else
					{
						initdata.advdata.uuids_more_available.uuid_cnt = pCfg->pAdvUuid->Count;
						initdata.advdata.uuids_more_available.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
					}
					if (pCfg->pSrManData != NULL)
					{
						initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
					}
				}
			}
			else
			{
				if (pCfg->pAdvManData != NULL)
				{
					initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
				}
				if (pCfg->pSrManData != NULL)
				{
					initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
				}
			}
		}
		else
		{
			if (pCfg->pAdvUuid != NULL)
			{
				if (pCfg->bCompleteUuidList)
				{
					initdata.srdata.uuids_complete.uuid_cnt = pCfg->pAdvUuid->Count;
					initdata.srdata.uuids_complete.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
				}
				else
				{
					initdata.srdata.uuids_more_available.uuid_cnt = pCfg->pAdvUuid->Count;
					initdata.srdata.uuids_more_available.p_uuids  = uid;//(ble_uuid_t*)pCfg->pAdvUuids;
				}

				if (pCfg->pAdvManData != NULL)
				{
					initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
				}

			}
			else
			{
				if (pCfg->pAdvManData != NULL)
				{
					initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
				}
				if (pCfg->pSrManData != NULL)
				{
					initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
				}
			}
		}
	}

    err_code = ble_advdata_encode(&initdata.advdata, g_AdvInstance.adv_data.adv_data.p_data, &g_AdvInstance.adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    //if (pCfg->pSrManData)
    {
    	err_code = ble_advdata_encode(&initdata.srdata, g_AdvInstance.adv_data.scan_rsp_data.p_data, &g_AdvInstance.adv_data.scan_rsp_data.len);
    	APP_ERROR_CHECK(err_code);
    }

    memcpy(&g_BleAppData.AdvData, &initdata.advdata, sizeof(ble_advdata_t));
    memcpy(&g_BleAppData.SrData, &initdata.srdata, sizeof(ble_advdata_t));

    err_code = sd_ble_gap_adv_set_configure(&g_AdvInstance.adv_handle, &g_AdvInstance.adv_data, &g_AdvInstance.adv_params);
    APP_ERROR_CHECK(err_code);

    g_AdvInstance.initialized = true;
#endif

    return true;
}
