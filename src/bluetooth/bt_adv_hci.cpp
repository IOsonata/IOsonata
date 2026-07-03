/**-------------------------------------------------------------------------
@file	bt_adv_hci.cpp

@brief	Advertising over standard HCI.

        Drives advertising through standard HCI commands sent with
        BtHciCommand, so it works with any HCI controller. The SDC is one
        such controller; a radio only link layer that presents HCI is
        another. No vendor headers here: only opcodes and packed HCI
        parameter layouts.

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

#include "istddef.h"
#include "convutil.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_appearance.h"

// --- Packed standard HCI command parameter layouts (Core Vol 4 Part E) ---

#pragma pack(push, 1)

typedef struct {
	uint8_t  AdvHandle;
	uint8_t  EvtProp[2];			//!< Advertising_Event_Properties
	uint8_t  PrimIntervalMin[3];	//!< Primary_Advertising_Interval_Min, 0.625 ms units
	uint8_t  PrimIntervalMax[3];	//!< Primary_Advertising_Interval_Max, 0.625 ms units
	uint8_t  PrimChanMap;
	uint8_t  OwnAddrType;
	uint8_t  PeerAddrType;
	uint8_t  PeerAddr[6];
	uint8_t  FilterPolicy;
	int8_t   TxPower;
	uint8_t  PrimPhy;
	uint8_t  SecMaxSkip;
	uint8_t  SecPhy;
	uint8_t  Sid;
	uint8_t  ScanReqNotif;
} BtHciLeExtAdvParams_t;			//!< 25 octets

typedef struct {
	uint8_t  AdvHandle;
	uint8_t  Operation;
	uint8_t  FragPref;
	uint8_t  DataLen;
	uint8_t  Data[251];				//!< Variable, only DataLen bytes are sent
} BtHciLeExtAdvData_t;

typedef struct {
	uint8_t  Enable;
	uint8_t  NumSets;
	uint8_t  AdvHandle;
	uint8_t  Duration[2];			//!< 10 ms units
	uint8_t  MaxExtAdvEvts;
} BtHciLeExtAdvEnable_t;			//!< One advertising set

typedef struct {
	uint8_t  AdvHandle;
	uint8_t  RandAddr[6];
} BtHciLeAdvSetRandAddr_t;

#pragma pack(pop)

// Header length (bytes before the variable data) of the ext adv/scan data cmd.
#define BTADV_HCI_DATA_HDR_LEN		(sizeof(BtHciLeExtAdvData_t) - 251)

// --- Adv packet buffers (data region overlaid by BtAdvPacket_t) ---

alignas(4) static uint8_t s_BtDevAdvBuff[sizeof(BtHciLeExtAdvData_t)];
alignas(4) static BtHciLeExtAdvData_t &s_BtDevExtAdvData = *(BtHciLeExtAdvData_t*)s_BtDevAdvBuff;
alignas(4) static BtAdvPacket_t s_BtDevExtAdvPkt = { 251, 0, s_BtDevExtAdvData.Data };

alignas(4) static uint8_t s_BtDevSrBuff[sizeof(BtHciLeExtAdvData_t)];
alignas(4) static BtHciLeExtAdvData_t &s_BtDevExtSrData = *(BtHciLeExtAdvData_t*)s_BtDevSrBuff;
alignas(4) static BtAdvPacket_t s_BtDevExtSrPkt = { 251, 0, s_BtDevExtSrData.Data };

// Advertising duration in 10 ms units, cached at init for the enable command.
// 0 means no timeout.
static uint16_t s_BtDevAdvDuration = 0;

static int BtAppAdvEnable(void);
static int BtAppAdvDisable(void);

static inline void BtAdvWr16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void BtAdvWr24(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
	p[2] = (uint8_t)((v >> 16) & 0xFF);
}

static inline BtHciDevice_t *BtAdvHciDev(void)
{
	return g_BtAppData.AppDevice.pHciDev;
}

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
	if (g_BtAppData.State != BTAPP_STATE_ADVERTISING && g_BtAppData.State != BTAPP_STATE_IDLE)
	{
		return false;
	}

	BtHciDevice_t *pDev = BtAdvHciDev();
	if (pDev == nullptr)
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

		s_BtDevExtAdvData.AdvHandle = 0;
		s_BtDevExtAdvData.Operation = 3;
		s_BtDevExtAdvData.FragPref  = 1;
		s_BtDevExtAdvData.DataLen   = advpkt->Len;

		if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA, &s_BtDevExtAdvData, BTADV_HCI_DATA_HDR_LEN + advpkt->Len, NULL, 0) != 0)
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

		s_BtDevExtSrData.AdvHandle = 0;
		s_BtDevExtSrData.Operation = 3;
		s_BtDevExtSrData.FragPref  = 1;
		s_BtDevExtSrData.DataLen   = srpkt->Len;

		if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_RESP_DATA, &s_BtDevExtSrData, BTADV_HCI_DATA_HDR_LEN + srpkt->Len, NULL, 0) != 0)
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
	BtHciDevice_t *pDev = BtAdvHciDev();
	if (pDev == nullptr)
	{
		return -1;
	}

	BtHciLeExtAdvEnable_t x;
	x.Enable = 1;
	x.NumSets = 1;
	x.AdvHandle = 0;
	BtAdvWr16(x.Duration, s_BtDevAdvDuration);
	x.MaxExtAdvEvts = 0;

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE, &x, sizeof(x), NULL, 0);
}

// Disable advertising for adv set 0 (controller command only, does not change
// app State). Disabling an already-disabled set has no effect.
static int BtAppAdvDisable(void)
{
	BtHciDevice_t *pDev = BtAdvHciDev();
	if (pDev == nullptr)
	{
		return -1;
	}

	BtHciLeExtAdvEnable_t x;
	x.Enable = 0;
	x.NumSets = 1;
	x.AdvHandle = 0;
	BtAdvWr16(x.Duration, 0);
	x.MaxExtAdvEvts = 0;

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE, &x, sizeof(x), NULL, 0);
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
	BtAppAdvDisable();

	g_BtAppData.State = BTAPP_STATE_IDLE;
}

bool BtAppAdvInit(const BtAppCfg_t * const pCfg)
{
	BtHciDevice_t *pDev = BtAdvHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	BtAdvPacket_t *advpkt = &s_BtDevExtAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BtDevExtSrPkt;

	// Encode the AD payload. BtAdvEncode decides legacy vs extended from how the
	// records pack, and reports it via bExtAdv/scannable.
	bool scannable = false;

	if (BtAdvEncode(pCfg, advpkt, srpkt, &g_BtAppData.bExtAdv, &scannable) == false)
	{
		return false;
	}

	// Both legacy and extended advertising are reached through the extended HCI
	// commands. Legacy advertising PDUs (so older centrals can see the set) are
	// selected with the legacy event-property bit; this also makes the
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

	// Build the LE Set Extended Advertising Parameters command directly in the
	// packed HCI layout. Casting BtExtAdvParam_t onto it does not work: its
	// bitfield layout differs from the wire format.
	BtHciLeExtAdvParams_t p;
	memset(&p, 0, sizeof(p));
	p.AdvHandle = 0;
	BtAdvWr16(p.EvtProp, extprop);
	BtAdvWr24(p.PrimIntervalMin, mSecTo0_625(pCfg->AdvInterval));
	BtAdvWr24(p.PrimIntervalMax, mSecTo0_625(pCfg->AdvInterval + 50));
	p.PrimChanMap  = 7;
	p.OwnAddrType  = BTADDR_TYPE_RAND;
	p.PeerAddrType = 0;
	p.FilterPolicy = 0;
	p.TxPower      = 0;
	p.PrimPhy      = BTADV_EXTADV_PHY_1M;
	p.SecMaxSkip   = 0;
	p.SecPhy       = BTADV_EXTADV_PHY_2M;
	p.Sid          = 0;
	p.ScanReqNotif = 0;

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM, &p, sizeof(p), NULL, 0) != 0)
	{
		return false;
	}

	// Extended advertising with a random own-address type requires the random
	// address to be set per advertising set; the legacy LE Set Random Address
	// command does not configure it. Use the device's configured random address.
	{
		uint8_t atype = 0;
		BtHciLeAdvSetRandAddr_t ra;
		ra.AdvHandle = 0;
		BtSmpLocalAddrGet(&atype, ra.RandAddr);

		if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR, &ra, sizeof(ra), NULL, 0) != 0)
		{
			return false;
		}
	}

	s_BtDevExtAdvData.AdvHandle = 0;
	s_BtDevExtAdvData.Operation = 3;
	s_BtDevExtAdvData.FragPref  = 1;
	s_BtDevExtAdvData.DataLen   = advpkt->Len;

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA, &s_BtDevExtAdvData, BTADV_HCI_DATA_HDR_LEN + advpkt->Len, NULL, 0) != 0)
	{
		return false;
	}

	// Scan response data only exists for a scannable set; setting it on a
	// non-scannable set is rejected by the controller.
	if (scannable && srpkt->Len > 0)
	{
		s_BtDevExtSrData.AdvHandle = 0;
		s_BtDevExtSrData.Operation = 3;
		s_BtDevExtSrData.FragPref  = 1;
		s_BtDevExtSrData.DataLen   = srpkt->Len;

		if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_RESP_DATA, &s_BtDevExtSrData, BTADV_HCI_DATA_HDR_LEN + srpkt->Len, NULL, 0) != 0)
		{
			return false;
		}
	}

	// Cache the advertising duration (10 ms units) for the enable command.
	s_BtDevAdvDuration = mSecTo10Ms(pCfg->AdvTimeout);

	return true;
}
