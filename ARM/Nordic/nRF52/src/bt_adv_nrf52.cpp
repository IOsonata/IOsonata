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
#include "compiler_abstraction.h"

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt_init.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_peer.h"

//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

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
alignas(4) static BtAdvPacket_t s_BleAppAdvPkt = { 255, 0, s_BleAppAdvBuff };

alignas(4) static uint8_t s_BleAppSrBuff[256];
alignas(4) static BtAdvPacket_t s_BleAppSrPkt = { BT_ADV_LEGACY_DATA_MAX, 0, s_BleAppSrBuff };

static ble_gap_adv_data_t s_BtAppAdvData = {
	.adv_data      = { s_BleAppAdvBuff, 0 },
	.scan_rsp_data = { s_BleAppSrBuff, 0 }
};

void BtAdvStart()
{
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING || BtPeerIsConnected())
	{
		return;
	}

	uint32_t err_code = sd_ble_gap_adv_start(g_BtAppData.AdvHdl,
		BTAPP_CONN_CFG_TAG);
	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("BtAdvStart failed: 0x%08" PRIx32 "\r\n", err_code);
		return;
	}

	g_BtAppData.State = BTAPP_STATE_ADVERTISING;
}

void BtAdvStop()
{
	sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
	g_BtAppData.State = BTAPP_STATE_IDLE;
}

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData,
		int SrLen)
{
	if (g_BtAppData.State != BTAPP_STATE_ADVERTISING &&
		g_BtAppData.State != BTAPP_STATE_IDLE)
	{
		return false;
	}
	if (AdvLen < 0 || SrLen < 0 || (AdvLen > 0 && pAdvData == nullptr) ||
		(SrLen > 0 && pSrData == nullptr))
	{
		return false;
	}

	BtAdvPacket_t *advpkt = &s_BleAppAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BleAppSrPkt;

	if (g_BtAppData.bExtAdv == false)
	{
		if (pAdvData != nullptr && AdvLen > 0)
		{
			int l = AdvLen + 2;
			BtAdvData_t *p = BtAdvDataAllocate(advpkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

			if (p == NULL)
			{
				return false;
			}
			*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
			memcpy(&p->Data[2], pAdvData, (size_t)AdvLen);

			s_BtAppAdvData.adv_data.len = advpkt->Len;
		}

		if (pSrData != nullptr && SrLen > 0)
		{
			int l = SrLen + 2;
			BtAdvData_t *p = BtAdvDataAllocate(srpkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

			if (p == NULL)
			{
				return false;
			}
			*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
			memcpy(&p->Data[2], pSrData, (size_t)SrLen);

			s_BtAppAdvData.scan_rsp_data.len = srpkt->Len;
		}
	}
	else
	{
		int l = 2 + AdvLen + SrLen;
		if (l > 2)
		{
			BtAdvData_t *p = BtAdvDataAllocate(advpkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

			if (p == NULL)
			{
				return false;
			}
			*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
			if (AdvLen > 0)
			{
				memcpy(&p->Data[2], pAdvData, (size_t)AdvLen);
			}
			if (SrLen > 0)
			{
				memcpy(&p->Data[2 + AdvLen], pSrData, (size_t)SrLen);
			}
			s_BtAppAdvData.adv_data.len = advpkt->Len;
		}
	}

	bool restart = g_BtAppData.State == BTAPP_STATE_ADVERTISING;
	if (restart)
	{
		uint32_t stopErr = sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
		if (stopErr != NRF_SUCCESS)
		{
			return false;
		}
	}

	uint32_t err = sd_ble_gap_adv_set_configure(&g_BtAppData.AdvHdl,
		&s_BtAppAdvData, NULL);
	if (err != NRF_SUCCESS)
	{
		return false;
	}

	if (restart)
	{
		g_BtAppData.State = BTAPP_STATE_IDLE;
		BtAdvStart();
		return g_BtAppData.State == BTAPP_STATE_ADVERTISING;
	}

	return true;
}

/**@brief Overloadable function for initializing the Advertising functionality.
 */
__WEAK bool BtAppAdvInit(const BtAppCfg_t *pCfg)
{
	if (pCfg == nullptr || !BtGattInitStatusComplete())
	{
		DEBUG_PRINTF("BtAppAdvInit refused: GATT init error %u\r\n",
			(unsigned)BtGattInitStatusErrorGet());
		return false;
	}

	BtAdvPacket_t *advpkt = &s_BleAppAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BleAppSrPkt;

	memset(&s_Nrf52Data.AdvParam, 0, sizeof(ble_gap_adv_params_t));

	bool scannable = false;
	if (BtAdvEncode(pCfg, advpkt, srpkt, &g_BtAppData.bExtAdv,
		&scannable) == false)
	{
		return false;
	}

	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		s_Nrf52Data.AdvParam.properties.type = g_BtAppData.bExtAdv ?
			BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED :
			BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
	}
	else if (pCfg->Role & BTAPP_ROLE_BROADCASTER)
	{
		if (g_BtAppData.bExtAdv)
		{
			s_Nrf52Data.AdvParam.properties.type =
				BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
		}
		else
		{
			s_Nrf52Data.AdvParam.properties.type = scannable ?
				BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED :
				BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
		}
	}

	s_Nrf52Data.AdvParam.p_peer_addr   = NULL;
	s_Nrf52Data.AdvParam.interval      = MSEC_TO_UNITS(pCfg->AdvInterval,
		UNIT_0_625_MS);
	s_Nrf52Data.AdvParam.duration      = MSEC_TO_UNITS(pCfg->AdvTimeout,
		UNIT_10_MS);
	s_Nrf52Data.AdvParam.filter_policy = BLE_GAP_ADV_FP_ANY;
	s_Nrf52Data.AdvParam.primary_phy   = BLE_GAP_PHY_1MBPS;
	s_Nrf52Data.AdvParam.secondary_phy = BLE_GAP_PHY_2MBPS;

	s_BtAppAdvData.adv_data.len = advpkt->Len;
	if (scannable)
	{
		s_BtAppAdvData.scan_rsp_data.p_data = s_BleAppSrBuff;
		s_BtAppAdvData.scan_rsp_data.len    = srpkt->Len;
	}
	else
	{
		s_BtAppAdvData.scan_rsp_data.p_data = NULL;
		s_BtAppAdvData.scan_rsp_data.len    = 0;
	}

	uint32_t err_code = sd_ble_gap_adv_set_configure(
		&g_BtAppData.AdvHdl, &s_BtAppAdvData, &s_Nrf52Data.AdvParam);
	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("sd_ble_gap_adv_set_configure failed: 0x%08" PRIx32
			"\r\n", err_code);
		return false;
	}

	return true;
}
