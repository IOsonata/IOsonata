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

Copyright (c) 2026, I-SYST inc. All rights reserved.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <limits.h>
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
#include "bluetooth/bt_peer.h"

//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
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
static bool s_bAdvertising;
static uint8_t s_PeripheralLinkCount;

alignas(4) static uint8_t s_BtAppAdvBuff[256];
alignas(4) static BtAdvPacket_t s_BtAppAdvPkt = { 255, 0, s_BtAppAdvBuff };

alignas(4) static uint8_t s_BtAppSrBuff[256];
alignas(4) static BtAdvPacket_t s_BtAppSrPkt = { BT_ADV_LEGACY_DATA_MAX, 0, s_BtAppSrBuff };

static ble_gap_adv_data_t s_BtAppAdvData = {
	.adv_data      = { s_BtAppAdvBuff, 0 },
	.scan_rsp_data = { s_BtAppSrBuff, 0 }
};

static uint16_t BtAdvPeripheralConnCount()
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

static bool BtAdvPeerSlotAvailable()
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

static bool BtAdvCanStart()
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

static bool BtAdvDataCanAllocate(const BtAdvPacket_t *pPacket, uint8_t Type,
								 int DataLen)
{
	if (pPacket == nullptr || pPacket->pData == nullptr || DataLen < 0 ||
		DataLen > pPacket->MaxLen || DataLen > 254 ||
		pPacket->Len < 0 || pPacket->Len > pPacket->MaxLen)
	{
		return false;
	}

	int used = pPacket->Len;
	int idx = 0;
	int remaining = pPacket->Len;

	while (remaining > 0)
	{
		if (remaining < (int)sizeof(BtAdvDataHdr_t))
		{
			return false;
		}

		const BtAdvDataHdr_t *pHdr =
			(const BtAdvDataHdr_t *)&pPacket->pData[idx];
		int recordLen = pHdr->Len + 1;
		if (pHdr->Len == 0 || recordLen > remaining)
		{
			return false;
		}

		if (pHdr->Type == Type)
		{
			used -= recordLen;
			break;
		}

		idx += recordLen;
		remaining -= recordLen;
	}

	return DataLen + 2 <= pPacket->MaxLen - used;
}

// A connectable advertising set stops when it creates a peripheral link. S145
// does not report that as BLE_GAP_EVT_ADV_SET_TERMINATED, so the connection
// observer calls this before deciding whether capacity remains for another link.
void BtAdvBmConnected()
{
	s_bAdvertising = false;
	BtAdvStateUpdate();
}

// Timeout and event-limit termination are reported explicitly by S145.
void BtAdvBmTerminated()
{
	s_bAdvertising = false;
	BtAdvStateUpdate();
}

void BtAdvStart()
{
	if (s_bAdvertising)
	{
		// A central-role connection can consume the last peer slot while the
		// connectable advertising set remains active. Stop the set as soon as
		// the common capacity check says another peripheral link cannot be
		// represented.
		if (!BtAdvCanStart())
		{
			BtAdvStop();
			return;
		}

		BtAdvStateUpdate();
		return;
	}

	if (!BtAdvCanStart())
	{
		return;
	}

	uint32_t err_code = sd_ble_gap_adv_start(g_BtAppData.AdvHdl, BTAPP_CONN_CFG_TAG);
	if (err_code == NRF_SUCCESS)
	{
		s_bAdvertising = true;
		BtAdvStateUpdate();
	}
	else
	{
		DEBUG_PRINTF("BtAdvStart failed: 0x%x\r\n", err_code);
	}
}

void BtAdvStop()
{
	if (!s_bAdvertising)
	{
		return;
	}

	uint32_t err_code = sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
	if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
	{
		DEBUG_PRINTF("BtAdvStop failed: 0x%x\r\n", err_code);
		return;
	}

	s_bAdvertising = false;
	BtAdvStateUpdate();
}

__attribute__((weak)) bool BtAppAdvInit(const BtAppCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	BtAdvPacket_t *advpkt = &s_BtAppAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BtAppSrPkt;

	s_bAdvertising = false;
	s_PeripheralLinkCount = pCfg->CentralDevMax;
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
	if (AdvLen < 0 || SrLen < 0 || AdvLen > INT_MAX - 2 || SrLen > INT_MAX - 2 ||
		(AdvLen != 0 && pAdvData == nullptr) ||
		(SrLen != 0 && pSrData == nullptr) ||
		g_BtAppData.AdvHdl == BLE_GAP_ADV_SET_HANDLE_NOT_SET)
	{
		return false;
	}

	BtAdvPacket_t *advpkt = &s_BtAppAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BtAppSrPkt;
	int advDataLen = AdvLen + 2;
	int srDataLen = SrLen + 2;

	// Validate both replacements without touching the buffers. SoftDevice owns
	// the configured buffers while advertising is active.
	if ((pAdvData != nullptr && !BtAdvDataCanAllocate(advpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, advDataLen)) ||
		(pSrData != nullptr && !BtAdvDataCanAllocate(srpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, srDataLen)))
	{
		return false;
	}

	bool wasAdvertising = s_bAdvertising;
	if (wasAdvertising)
	{
		uint32_t err_code = sd_ble_gap_adv_stop(g_BtAppData.AdvHdl);
		if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
		{
			DEBUG_PRINTF("BtAppAdvManDataSet stop failed: 0x%x\r\n", err_code);
			return false;
		}
		s_bAdvertising = false;
		BtAdvStateUpdate();
	}

	if (pAdvData != nullptr)
	{
		BtAdvData_t *p = BtAdvDataAllocate(advpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, advDataLen);

		if (p == NULL)
		{
			DEBUG_PRINTF("BtAppAdvManDataSet adv allocation invariant failed\r\n");
			if (wasAdvertising)
			{
				BtAdvStart();
			}
			return false;
		}
		uint16_t vendorId = g_BtAppData.AppDevice.VendorId;
		memcpy(p->Data, &vendorId, sizeof(vendorId));
		if (AdvLen != 0)
		{
			memcpy(&p->Data[2], pAdvData, AdvLen);
		}

		s_BtAppAdvData.adv_data.len = advpkt->Len;
	}

	if (pSrData != nullptr)
	{
		BtAdvData_t *p = BtAdvDataAllocate(srpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, srDataLen);

		if (p == NULL)
		{
			DEBUG_PRINTF("BtAppAdvManDataSet scan allocation invariant failed\r\n");
			return false;
		}
		uint16_t vendorId = g_BtAppData.AppDevice.VendorId;
		memcpy(p->Data, &vendorId, sizeof(vendorId));
		if (SrLen != 0)
		{
			memcpy(&p->Data[2], pSrData, SrLen);
		}

		s_BtAppAdvData.scan_rsp_data.p_data = s_BtAppSrBuff;
		s_BtAppAdvData.scan_rsp_data.len = srpkt->Len;
	}

	uint32_t err_code = sd_ble_gap_adv_set_configure(
		&g_BtAppData.AdvHdl, &s_BtAppAdvData, NULL);
	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("BtAppAdvManDataSet configure failed: 0x%x\r\n", err_code);
		// The live buffers now contain the requested data. Do not restart an
		// advertising set whose reconfiguration was rejected.
		return false;
	}

	if (wasAdvertising)
	{
		BtAdvStart();
		return s_bAdvertising;
	}

	return true;
}
