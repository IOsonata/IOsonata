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
// The alignment lives on the backing buffer above; a reference has no storage
// of its own to align (clang rejects alignas on a reference, g++ ignores it).
static BtHciLeExtAdvData_t &s_BtDevExtAdvData = *(BtHciLeExtAdvData_t*)s_BtDevAdvBuff;
alignas(4) static BtAdvPacket_t s_BtDevExtAdvPkt = { 251, 0, s_BtDevExtAdvData.Data };

alignas(4) static uint8_t s_BtDevSrBuff[sizeof(BtHciLeExtAdvData_t)];
static BtHciLeExtAdvData_t &s_BtDevExtSrData = *(BtHciLeExtAdvData_t*)s_BtDevSrBuff;
alignas(4) static BtAdvPacket_t s_BtDevExtSrPkt = { 251, 0, s_BtDevExtSrData.Data };

// Advertising duration in 10 ms units, cached at init for the enable command.
// 0 means no timeout.
static uint16_t s_BtDevAdvDuration = 0;

// The own address the advertising set was last programmed with. What a new
// peripheral-role connection is stamped with, so the SMP toolbox computes
// f5/f6/c1 with the address the peer actually saw.
static uint8_t s_BtAdvOwnAddrType;
static uint8_t s_BtAdvOwnAddr[6];

// Own address currently in use on air for advertising. Falls back to the
// device's configured address when the set was never programmed.
void BtAdvOwnAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	for (int i = 0; i < 6; i++)
	{
		if (s_BtAdvOwnAddr[i] != 0)
		{
			*pType = s_BtAdvOwnAddrType;
			memcpy(pAddr, s_BtAdvOwnAddr, 6);
			return;
		}
	}
	BtSmpLocalAddrGet(pType, pAddr);
}

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
		// Company identifier is little-endian on air. Write the two octets
		// directly rather than through a uint16_t cast, which assumes host byte
		// order and an aligned destination (Core Spec Supplement Part A 1.4).
		p->Data[0] = (uint8_t)(g_BtAppData.AppDevice.VendorId & 0xFF);
		p->Data[1] = (uint8_t)(g_BtAppData.AppDevice.VendorId >> 8);
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
		// Company identifier is little-endian on air; see the note above.
		p->Data[0] = (uint8_t)(g_BtAppData.AppDevice.VendorId & 0xFF);
		p->Data[1] = (uint8_t)(g_BtAppData.AppDevice.VendorId >> 8);
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
		// The disable result is not decisive: the set may already have been
		// stopped by the controller, for example on a duration expiry not yet
		// processed. The enable result is. If it fails the device is off air
		// with the new data loaded, so report the failure and record IDLE
		// rather than leave the state claiming the device is advertising.
		BtAppAdvDisable();

		if (BtAppAdvEnable() != 0)
		{
			g_BtAppData.State = BTAPP_STATE_IDLE;
			return false;
		}
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
	// Only report idle once the controller has accepted the disable. If the
	// command failed the set is still on air, and moving to IDLE would make a
	// later BtAppAdvStart look like a no-op while the device keeps
	// advertising with whatever data it had.
	if (BtAppAdvDisable() != 0)
	{
		return;
	}

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

	// A legacy connectable set is ADV_IND, which is always scannable. Core Vol 4
	// Part E 7.8.53 defines only a fixed set of legacy event-property values;
	// LEGACY|CONNECTABLE on its own (0x11) is not one of them and the controller
	// rejects it with Invalid HCI Command Parameters. Force the scannable
	// property (an empty scan response is valid) so a name-only peripheral, the
	// most common configuration, is accepted and advertises.
	if (g_BtAppData.bExtAdv == false && (pCfg->Role & BTAPP_ROLE_PERIPHERAL))
	{
		scannable = true;
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
	// Primary_Advertising_Interval_Min has a floor of 0x000020 (20 ms) in the
	// LE Set Extended Advertising Parameters command; a smaller configured
	// interval is otherwise rejected by the controller with Invalid HCI Command
	// Parameters. Clamp up to the floor and keep max >= min.
	uint32_t primMin = mSecTo0_625(pCfg->AdvInterval);
	uint32_t primMax = mSecTo0_625(pCfg->AdvInterval + 50);
	if (primMin < 0x20)
	{
		primMin = 0x20;
	}
	if (primMax < primMin)
	{
		primMax = primMin;
	}
	BtAdvWr24(p.PrimIntervalMin, primMin);
	BtAdvWr24(p.PrimIntervalMax, primMax);
	p.PrimChanMap  = 7;
	// Own address type follows the configured local identity instead of being
	// forced to Random. A public identity advertises as public and needs no
	// random address; a random identity advertises as random with a validated
	// static random address loaded per set.
	uint8_t  localType = 0;
	uint8_t  localAddr[6];
	BtSmpLocalAddrGet(&localType, localAddr);
	bool useRandom = (localType == BTADDR_TYPE_RAND ||
					  localType == BTADDR_TYPE_RANDOM_STATIC);
	p.OwnAddrType  = useRandom ? BTADDR_TYPE_RAND : BTADDR_TYPE_PUBLIC;
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
	// command does not configure it. This is the single point where the on-air
	// own address is chosen; a privacy layer that rotates resolvable private
	// addresses replaces the address here, and the recorded copy is what a new
	// connection is stamped with.
	if (useRandom)
	{
		// Reject an address that is not a valid static random address (for
		// example the all-zero default) rather than have the controller fail
		// the command. See BtAddrIsStaticRandom, Vol 6 Part B 1.3.2.1.
		if (BtAddrIsStaticRandom(localAddr) == false)
		{
			return false;
		}

		BtHciLeAdvSetRandAddr_t ra;
		ra.AdvHandle = 0;
		memcpy(ra.RandAddr, localAddr, 6);

		if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR, &ra, sizeof(ra), NULL, 0) != 0)
		{
			return false;
		}

		s_BtAdvOwnAddrType = BTADDR_TYPE_RAND;
		memcpy(s_BtAdvOwnAddr, localAddr, 6);
	}
	else
	{
		s_BtAdvOwnAddrType = BTADDR_TYPE_PUBLIC;
		memcpy(s_BtAdvOwnAddr, localAddr, 6);
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
