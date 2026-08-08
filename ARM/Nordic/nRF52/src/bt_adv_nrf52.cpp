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

// Run before the application observer (priority 1). Advertising termination
// state must be current before its timeout or connection callback runs.
#define BT_ADV_NRF52_OBSERVER_PRIO    0

// --- Adv packet buffers and SoftDevice adv data ---
// File-scope: nothing outside bt_adv_nrf52.cpp needs these.

#pragma pack(push, 4)
typedef struct __Bt_App_Nrf52_Data {
	ble_gap_adv_params_t AdvParam;		//!< SoftDevice adv params
} BtAppNrf52Data_t;
#pragma pack(pop)

static BtAppNrf52Data_t s_Nrf52Data = { {0} };
static bool s_bAdvertising;
static int s_PeripheralLinkCount;
static uint16_t s_PendingPeripheralHdl = BT_CONN_HDL_INVALID;

alignas(4) static uint8_t s_BleAppAdvBuff[256];
alignas(4) static BtAdvPacket_t s_BleAppAdvPkt = { 255, 0, s_BleAppAdvBuff };

alignas(4) static uint8_t s_BleAppSrBuff[256];
alignas(4) static BtAdvPacket_t s_BleAppSrPkt = { BT_ADV_LEGACY_DATA_MAX, 0, s_BleAppSrBuff };

static ble_gap_adv_data_t s_BtAppAdvData = {
	.adv_data      = { s_BleAppAdvBuff, 0 },
	.scan_rsp_data = { s_BleAppSrBuff, 0 }
};

static bool BtAdvCanStart()
{
	uint8_t role = g_BtAppData.AppDevice.Conn.Role;

	if ((role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER)) == 0)
	{
		return false;
	}

	if ((role & BTAPP_ROLE_PERIPHERAL) == 0)
	{
		return true;
	}

	uint16_t peripheralCount = 0;
	uint16_t freeCount = 0;
	uint16_t peerCount = BtPeerCount();

	for (uint16_t i = 0; i < peerCount; i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer == nullptr)
		{
			continue;
		}
		if (pPeer->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			freeCount++;
		}
		else if (pPeer->Conn.Role == BT_CONN_ROLE_PERIPHERAL)
		{
			peripheralCount++;
		}
	}

	// The priority-0 connection observer runs before BtPeerConnected. Reserve
	// the slot the accepted peripheral link is about to consume and include it
	// in the configured peripheral-link limit. If observer ordering changes and
	// the record already exists, do not count it twice.
	if (s_PendingPeripheralHdl != BT_CONN_HDL_INVALID &&
		BtPeerFindByHdl(s_PendingPeripheralHdl) == nullptr)
	{
		peripheralCount++;
		if (freeCount > 0)
		{
			freeCount--;
		}
	}

	return s_PeripheralLinkCount > 0 && freeCount > 0 &&
		peripheralCount < s_PeripheralLinkCount;
}

static void BtAdvStateUpdate()
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

static void BtAdvNrf52EvtHandler(ble_evt_t const *pEvt, void *pContext)
{
	(void)pContext;

	switch (pEvt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			if (pEvt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH)
			{
				// Connectable advertising stops when it creates a peripheral
				// link. Restart before the application callback, while reserving
				// the peer slot that callback is about to populate.
				s_bAdvertising = false;
				s_PendingPeripheralHdl = pEvt->evt.gap_evt.conn_handle;
				BtAdvStateUpdate();
				BtAdvStart();
				s_PendingPeripheralHdl = BT_CONN_HDL_INVALID;
			}
			break;

		case BLE_GAP_EVT_ADV_SET_TERMINATED:
			s_bAdvertising = false;
			BtAdvStateUpdate();
			break;

		default:
			break;
	}
}

NRF_SDH_BLE_OBSERVER(s_BtAdvObserver, BT_ADV_NRF52_OBSERVER_PRIO,
	BtAdvNrf52EvtHandler, NULL);

void BtAdvStart()
{
	if (s_bAdvertising)
	{
		BtAdvStateUpdate();
		return;
	}

	if (!BtAdvCanStart())
	{
		BtAdvStateUpdate();
		return;
	}

	uint32_t err_code = sd_ble_gap_adv_start(g_BtAppData.AdvHdl,
		BTAPP_CONN_CFG_TAG);
	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("BtAdvStart failed: 0x%08" PRIx32 "\r\n", err_code);
		return;
	}

	s_bAdvertising = true;
	BtAdvStateUpdate();
}

void BtAdvStop()
{
	if (!s_bAdvertising)
	{
		BtAdvStateUpdate();
		return;
	}

	uint32_t err_code = sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
	if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
	{
		DEBUG_PRINTF("BtAdvStop failed: 0x%08" PRIx32 "\r\n", err_code);
		return;
	}

	s_bAdvertising = false;
	BtAdvStateUpdate();
}

static void BtAdvNrf52VendorIdWrite(uint8_t *pData)
{
	pData[0] = (uint8_t)(g_BtAppData.AppDevice.VendorId & 0xFFU);
	pData[1] = (uint8_t)(g_BtAppData.AppDevice.VendorId >> 8);
}

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData,
		int SrLen)
{
	if (g_BtAppData.State != BTAPP_STATE_ADVERTISING &&
		g_BtAppData.State != BTAPP_STATE_IDLE &&
		g_BtAppData.State != BTAPP_STATE_CONNECTED)
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
	uint8_t oldAdvData[255];
	uint8_t oldSrData[BT_ADV_LEGACY_DATA_MAX];
	int oldAdvLen = advpkt->Len;
	int oldSrLen = srpkt->Len;
	memcpy(oldAdvData, advpkt->pData, (size_t)oldAdvLen);
	memcpy(oldSrData, srpkt->pData, (size_t)oldSrLen);

	bool restart = s_bAdvertising;
	if (restart)
	{
		uint32_t stopErr = sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
		if (stopErr != NRF_SUCCESS && stopErr != NRF_ERROR_INVALID_STATE)
		{
			return false;
		}
		s_bAdvertising = false;
		BtAdvStateUpdate();
	}

	if (g_BtAppData.bExtAdv == false)
	{
		if (pAdvData != nullptr && AdvLen > 0)
		{
			int l = AdvLen + 2;
			BtAdvData_t *p = BtAdvDataAllocate(advpkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

			if (p == NULL)
			{
				goto rollback;
			}
			BtAdvNrf52VendorIdWrite(p->Data);
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
				goto rollback;
			}
			BtAdvNrf52VendorIdWrite(p->Data);
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
				goto rollback;
			}
			BtAdvNrf52VendorIdWrite(p->Data);
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

	if (sd_ble_gap_adv_set_configure(&g_BtAppData.AdvHdl,
		&s_BtAppAdvData, NULL) != NRF_SUCCESS)
	{
		goto rollback;
	}

	if (restart)
	{
		BtAdvStart();
		return s_bAdvertising;
	}

	return true;

rollback:
	memcpy(advpkt->pData, oldAdvData, (size_t)oldAdvLen);
	advpkt->Len = oldAdvLen;
	memcpy(srpkt->pData, oldSrData, (size_t)oldSrLen);
	srpkt->Len = oldSrLen;
	s_BtAppAdvData.adv_data.len = oldAdvLen;
	s_BtAppAdvData.scan_rsp_data.len = oldSrLen;

	if (restart)
	{
		BtAdvStart();
	}
	return false;
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

	s_bAdvertising = false;
	s_PeripheralLinkCount = pCfg->PeriLinkCount;
	s_PendingPeripheralHdl = BT_CONN_HDL_INVALID;
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
