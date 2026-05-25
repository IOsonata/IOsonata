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
	BtAdvPacket_t *advpkt;
	BtAdvPacket_t *srpkt;

	memset(&s_Nrf52Data.AdvParam, 0, sizeof(ble_gap_adv_params_t));

	if (g_BtAppData.bExtAdv)
	{
		advpkt = &s_BleAppExtAdvPkt;
		srpkt  = &s_BleAppExtSrPkt;
	}
	else
	{
		advpkt = &s_BleAppAdvPkt;
		srpkt  = &s_BleAppSrPkt;
	}

	// SoftDevice-specific adv-type enum based on role.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
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

	// Generic AD payload assembly.
	if (BtAdvAssembleFromCfg(pCfg, advpkt, srpkt) == false)
	{
		return false;
	}

	// SoftDevice adv params + push to controller.
	s_Nrf52Data.AdvParam.p_peer_addr   = NULL;
	s_Nrf52Data.AdvParam.interval      = MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
	s_Nrf52Data.AdvParam.duration      = MSEC_TO_UNITS(pCfg->AdvTimeout,  UNIT_10_MS);
	s_Nrf52Data.AdvParam.filter_policy = BLE_GAP_ADV_FP_ANY;
	s_Nrf52Data.AdvParam.primary_phy   = BLE_GAP_PHY_1MBPS;
	s_Nrf52Data.AdvParam.secondary_phy = BLE_GAP_PHY_2MBPS;

	s_BtAppAdvData.adv_data.len      = advpkt->Len;
	s_BtAppAdvData.scan_rsp_data.len = srpkt->Len;

	uint32_t err_code = sd_ble_gap_adv_set_configure(
		&g_BtAppData.AdvHdl, &s_BtAppAdvData, &s_Nrf52Data.AdvParam);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}

