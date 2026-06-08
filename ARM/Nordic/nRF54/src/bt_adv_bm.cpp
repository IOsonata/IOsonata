/**-------------------------------------------------------------------------
@file	bt_adv_bm.cpp

@brief	sdk-nrf-bm SoftDevice (S145) advertising functions.

        Extracted from bt_app_bm.cpp in step 4 of the Voci refactor.
        Owns the advertising packet buffers, the SoftDevice adv data struct,
        and the BtAdv* / BtAppAdv* function family.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "nrf_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_gatt.h"
#include "ble_gatts.h"
#include "bm/softdevice_handler/nrf_sdh.h"
#include "bm/softdevice_handler/nrf_sdh_ble.h"

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_appearance.h"

// Debug printf: mirrors the guard in bt_app_bm.cpp.
#ifdef BM_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

// Port-local. Mirrors the values in bt_app_bm.cpp.
#define BTAPP_CONN_CFG_TAG              CONFIG_NRF_SDH_BLE_CONN_TAG

#ifndef UNIT_0_625_MS
#define UNIT_0_625_MS                   625
#endif
#ifndef UNIT_10_MS
#define UNIT_10_MS                      10000
#endif
#ifndef MSEC_TO_UNITS
#define MSEC_TO_UNITS(MS, UNIT)         (((MS) * 1000) / (UNIT))
#endif

// --- Adv packet buffers and SoftDevice adv data ---

#pragma pack(push, 4)
typedef struct __Bt_App_Bm_Data {
	ble_gap_adv_params_t AdvParam;		//!< sdk-nrf-bm adv params
} BtAppBmData_t;
#pragma pack(pop)

static BtAppBmData_t s_BmData = { {0} };

alignas(4) static uint8_t s_BtAppAdvBuff[256];
alignas(4) static BtAdvPacket_t s_BtAppAdvPkt = { 255, 0, s_BtAppAdvBuff };

alignas(4) static uint8_t s_BtAppSrBuff[256];
alignas(4) static BtAdvPacket_t s_BtAppSrPkt = { BT_ADV_LEGACY_DATA_MAX, 0, s_BtAppSrBuff };

static ble_gap_adv_data_t s_BtAppAdvData = {
	.adv_data      = { s_BtAppAdvBuff, 0 },
	.scan_rsp_data = { s_BtAppSrBuff, 0 }
};

void BtAdvStart()
{
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING ||
		BtAppGetConnHandle() != BLE_CONN_HANDLE_INVALID)
	{
		return;
	}

	uint32_t err_code = sd_ble_gap_adv_start(g_BtAppData.AdvHdl, BTAPP_CONN_CFG_TAG);
	if (err_code == NRF_SUCCESS)
	{
		g_BtAppData.State = BTAPP_STATE_ADVERTISING;
	}
	else
	{
		DEBUG_PRINTF("BtAdvStart failed: 0x%x\r\n", err_code);
	}
}

void BtAdvStop()
{
	sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
	g_BtAppData.State = BTAPP_STATE_IDLE;
}

__attribute__((weak)) bool BtAppAdvInit(const BtAppCfg_t *pCfg)
{
	BtAdvPacket_t *advpkt = &s_BtAppAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BtAppSrPkt;

	memset(&s_BmData.AdvParam, 0, sizeof(ble_gap_adv_params_t));

	// Encode the AD payload. BtAdvEncode decides legacy vs extended from how the
	// records pack, and reports it via bExtAdv/scannable.
	bool scannable = false;

	if (BtAdvEncode(pCfg, advpkt, srpkt, &g_BtAppData.bExtAdv, &scannable) == false)
	{
		return false;
	}

	// SoftDevice adv-type enum from role + decided mode + scannable.
	// The SoftDevice expresses legacy vs extended PDUs through the type enum;
	// the legacy enums emit classic ADV_* PDUs that older centrals can see.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		// Connectable. Legacy ADV_IND is scannable; extended connectable is
		// non-scannable (spec forbids connectable + scannable in extended).
		s_BmData.AdvParam.properties.type = g_BtAppData.bExtAdv ?
			BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED :
			BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
	}
	else if (pCfg->Role & BTAPP_ROLE_BROADCASTER)
	{
		// Non-connectable. Scannable only if the encode placed data on the scan
		// response; otherwise non-scannable (no scan response).
		if (g_BtAppData.bExtAdv)
		{
			s_BmData.AdvParam.properties.type =
				BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
		}
		else
		{
			s_BmData.AdvParam.properties.type = scannable ?
				BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED :
				BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
		}
	}

	// SoftDevice adv params + push to controller.
	s_BmData.AdvParam.p_peer_addr   = NULL;
	s_BmData.AdvParam.interval      = MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
	s_BmData.AdvParam.duration      = MSEC_TO_UNITS(pCfg->AdvTimeout,  UNIT_10_MS);
	s_BmData.AdvParam.filter_policy = BLE_GAP_ADV_FP_ANY;
	s_BmData.AdvParam.primary_phy   = BLE_GAP_PHY_1MBPS;
	s_BmData.AdvParam.secondary_phy = BLE_GAP_PHY_2MBPS;

	s_BtAppAdvData.adv_data.len = advpkt->Len;

	// Scan response only exists for a scannable set; leave it null otherwise.
	if (scannable)
	{
		s_BtAppAdvData.scan_rsp_data.p_data = s_BtAppSrBuff;
		s_BtAppAdvData.scan_rsp_data.len    = srpkt->Len;
	}
	else
	{
		s_BtAppAdvData.scan_rsp_data.p_data = NULL;
		s_BtAppAdvData.scan_rsp_data.len    = 0;
	}

	uint32_t err_code = sd_ble_gap_adv_set_configure(
		&g_BtAppData.AdvHdl, &s_BtAppAdvData, &s_BmData.AdvParam);

	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("sd_ble_gap_adv_set_configure failed: 0x%x\r\n", err_code);
		return false;
	}

	return true;
}

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
	if (g_BtAppData.State != BTAPP_STATE_ADVERTISING &&
		g_BtAppData.State != BTAPP_STATE_IDLE)
	{
		return false;
	}

	BtAdvPacket_t *advpkt = &s_BtAppAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BtAppSrPkt;

	if (pAdvData)
	{
		int l = AdvLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(advpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
		memcpy(&p->Data[2], pAdvData, AdvLen);

		s_BtAppAdvData.adv_data.len = advpkt->Len;
	}

	if (pSrData)
	{
		int l = SrLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(srpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = g_BtAppData.AppDevice.VendorId;
		memcpy(&p->Data[2], pSrData, SrLen);

		s_BtAppAdvData.scan_rsp_data.len = srpkt->Len;
	}

	// The SoftDevice does not allow a data update while the set is running:
	// stop, reconfigure, restart. Mirrors the SDC disable/enable on update.
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)
	{
		sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
	}

	sd_ble_gap_adv_set_configure(&g_BtAppData.AdvHdl, &s_BtAppAdvData, NULL);

	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)
	{
		g_BtAppData.State = BTAPP_STATE_IDLE;
		BtAdvStart();
	}

	return g_BtAppData.State == BTAPP_STATE_ADVERTISING;
}
