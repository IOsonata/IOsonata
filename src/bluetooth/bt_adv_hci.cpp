/**-------------------------------------------------------------------------
@file	bt_adv_hci.cpp

@brief	Advertising over standard HCI, legacy, extended, periodic and PAwR.

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
#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gatt_init.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_appearance.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to trace advertising state changes for this file. Output
// goes to the SysLog transport the app configured. A release build defines
// NDEBUG, which strips the trace regardless.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

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
	uint8_t  PrimPhyOptions;		//!< Primary_Advertising_PHY_Options
	uint8_t  SecPhyOptions;			//!< Secondary_Advertising_PHY_Options
} BtHciLeExtAdvParamsV2_t;			//!< 27 octets, the v1 layout plus the two coding fields

// The v1 command is sent from a v2 object by shortening the length, which only
// holds while v1 is byte for byte the leading part of v2. Check the join and
// both sizes rather than trusting the field lists to stay in step.
static_assert(sizeof(BtHciLeExtAdvParams_t) == 25, "v1 extended advertising parameters must be 25 octets");
static_assert(sizeof(BtHciLeExtAdvParamsV2_t) == 27, "v2 extended advertising parameters must be 27 octets");
static_assert(offsetof(BtHciLeExtAdvParamsV2_t, ScanReqNotif) ==
	offsetof(BtHciLeExtAdvParams_t, ScanReqNotif),
	"v2 must extend v1, not reorder it");
static_assert(offsetof(BtHciLeExtAdvParamsV2_t, PrimPhyOptions) ==
	sizeof(BtHciLeExtAdvParams_t),
	"the v2 coding fields must follow the whole v1 layout");

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

typedef struct {
	uint8_t  IntervalMin[2];
	uint8_t  IntervalMax[2];
	uint8_t  AdvType;
	uint8_t  OwnAddrType;
	uint8_t  PeerAddrType;
	uint8_t  PeerAddr[6];
	uint8_t  ChanMap;
	uint8_t  FilterPolicy;
} BtHciLeAdvParams_t;				//!< 15 octets

typedef struct {
	uint8_t  DataLen;
	uint8_t  Data[BT_ADV_LEGACY_DATA_MAX];
} BtHciLeAdvData_t;				//!< 32 octets

#pragma pack(pop)

#define BTADV_HCI_DATA_FRAGMENT_MAX	251
#define BTADV_HCI_DATA_HDR_LEN		(sizeof(BtHciLeExtAdvData_t) - BTADV_HCI_DATA_FRAGMENT_MAX)
#define BTADV_EXTADV_DATA_MAX		1650
#define BTADV_EXTADV_OP_INTERMEDIATE	0x00
#define BTADV_EXTADV_OP_FIRST		0x01
#define BTADV_EXTADV_OP_LAST		0x02
#define BTADV_EXTADV_OP_COMPLETE	0x03
#define BTADV_LEGACY_ADV_IND		0x00
#define BTADV_LEGACY_ADV_SCAN_IND	0x02
#define BTADV_LEGACY_ADV_NONCONN_IND	0x03

// --- Adv packet buffers ---
// Extended advertising Host data can be up to 1650 octets. HCI transports it
// in commands carrying at most 251 data octets each, so the retained packet
// storage is independent of the command fragment buffer.
alignas(4) static uint8_t s_BtDevAdvBuff[BTADV_EXTADV_DATA_MAX];
alignas(4) static BtAdvPacket_t s_BtDevExtAdvPkt = {
	BTADV_EXTADV_DATA_MAX, 0, s_BtDevAdvBuff
};

alignas(4) static uint8_t s_BtDevSrBuff[BTADV_EXTADV_DATA_MAX];
alignas(4) static BtAdvPacket_t s_BtDevExtSrPkt = {
	BTADV_EXTADV_DATA_MAX, 0, s_BtDevSrBuff
};

// One reusable full-size scratch packet keeps live manufacturer-data updates
// off the task stack. Advertising state is global and the update path is
// serialized with the rest of BtApp advertising control.
alignas(4) static uint8_t s_BtDevScratchBuff[BTADV_EXTADV_DATA_MAX];

// Advertising duration in 10 ms units, cached at init for the extended enable
// command. 0 means no timeout.
static uint16_t s_BtDevAdvDuration = 0;
static bool s_BtDevUseExtAdvCmd = true;
static bool s_BtDevAdvScannable = false;
static BtHciCapabilities_t s_BtAdvReadCapabilities;
static const BtHciCapabilities_t *s_pBtAdvCapabilities = nullptr;

// Requested LE Coded PHY coding for the advertising set, Core Vol 4 Part E
// 7.8.53. Zero on both means no coding was asked for, which is the state every
// build starts in and the only value a controller without Advertising Coding
// Selection accepts. A non zero value also selects the LE Coded PHY for that
// advertising physical channel, because the options field is ignored on any
// other PHY and asking for a coding without the PHY would do nothing.
static uint8_t s_BtAdvPrimPhyOptions = BT_HCI_ADV_PHY_OPT_NONE;
static uint8_t s_BtAdvSecPhyOptions = BT_HCI_ADV_PHY_OPT_NONE;

bool BtAdvCodingSelectionSet(uint8_t PrimOption, uint8_t SecOption)
{
	if (PrimOption > BT_HCI_ADV_PHY_OPT_MAX ||
		SecOption > BT_HCI_ADV_PHY_OPT_MAX)
	{
		return false;
	}

	s_BtAdvPrimPhyOptions = PrimOption;
	s_BtAdvSecPhyOptions = SecOption;

	return true;
}

void BtAdvCodingSelectionGet(uint8_t *pPrimOption, uint8_t *pSecOption)
{
	if (pPrimOption != nullptr)
	{
		*pPrimOption = s_BtAdvPrimPhyOptions;
	}
	if (pSecOption != nullptr)
	{
		*pSecOption = s_BtAdvSecPhyOptions;
	}
}

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

static bool BtAdvExtLimitsValid(uint16_t AdvLen, uint16_t ScanResponseLen)
{
	if (s_pBtAdvCapabilities == nullptr)
	{
		return false;
	}

	if ((s_pBtAdvCapabilities->Valid & BT_HCI_CAP_VALID_ADV_SET_COUNT) != 0 &&
		s_pBtAdvCapabilities->AdvSetCount == 0)
	{
		return false;
	}

	if ((s_pBtAdvCapabilities->Valid & BT_HCI_CAP_VALID_MAX_ADV_DATA_LEN) != 0 &&
		(AdvLen > s_pBtAdvCapabilities->MaxAdvDataLen ||
		 ScanResponseLen > s_pBtAdvCapabilities->MaxAdvDataLen))
	{
		return false;
	}

	return true;
}

static bool BtAdvCommandSetSelect(bool ExtPayload, bool UseRandomAddress,
	bool UseScanResponse, uint16_t AdvLen, uint16_t ScanResponseLen,
	bool *pUseExtCommands)
{
	if (pUseExtCommands == nullptr || s_pBtAdvCapabilities == nullptr)
	{
		return false;
	}

	if (BtHciCapabilitiesExtendedAdvertisingSupported(s_pBtAdvCapabilities,
		UseRandomAddress, UseScanResponse) &&
		BtAdvExtLimitsValid(AdvLen, ScanResponseLen))
	{
		*pUseExtCommands = true;
		return true;
	}

	if (ExtPayload || AdvLen > BT_ADV_LEGACY_DATA_MAX ||
		ScanResponseLen > BT_ADV_LEGACY_DATA_MAX)
	{
		return false;
	}

	if (BtHciCapabilitiesLegacyAdvertisingSupported(s_pBtAdvCapabilities,
		UseRandomAddress, UseScanResponse) == false)
	{
		return false;
	}

	*pUseExtCommands = false;
	return true;
}

static int BtAdvDataWrite(BtHciDevice_t *pDev, BtAdvPacket_t *pPacket,
	bool ScanResponse, bool *pPartialWrite = nullptr)
{
	if (pPartialWrite != nullptr)
	{
		*pPartialWrite = false;
	}

	if (pDev == nullptr || pPacket == nullptr || s_pBtAdvCapabilities == nullptr ||
		pPacket->Len < 0 || pPacket->Len > pPacket->MaxLen ||
		(ScanResponse && s_BtDevAdvScannable == false))
	{
		return -1;
	}

	if (s_BtDevUseExtAdvCmd)
	{
		uint16_t commandBit = ScanResponse ?
			BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_RESPONSE_DATA :
			BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA;
		if (BtHciCapabilitiesCommandSupported(s_pBtAdvCapabilities,
			commandBit) == false ||
			BtAdvExtLimitsValid(ScanResponse ? 0 : (uint16_t)pPacket->Len,
				ScanResponse ? (uint16_t)pPacket->Len : 0) == false)
		{
			return -1;
		}

		uint16_t opcode = ScanResponse ?
			BT_HCI_CMD_CTLR_SET_EXT_SCAN_RESP_DATA :
			BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA;
		BtHciLeExtAdvData_t data = {};
		data.AdvHandle = 0;
		data.FragPref = 1;

		if (pPacket->Len <= BTADV_HCI_DATA_FRAGMENT_MAX)
		{
			data.Operation = BTADV_EXTADV_OP_COMPLETE;
			data.DataLen = (uint8_t)pPacket->Len;
			if (pPacket->Len > 0)
			{
				memcpy(data.Data, pPacket->pData, (size_t)pPacket->Len);
			}
			return BtHciCommand(pDev, opcode, &data,
				(uint8_t)(BTADV_HCI_DATA_HDR_LEN + pPacket->Len), NULL, 0);
		}

		int offset = 0;
		int remaining = pPacket->Len;
		bool wroteFragment = false;

		while (remaining > 0)
		{
			int fragmentLen = remaining > BTADV_HCI_DATA_FRAGMENT_MAX ?
				BTADV_HCI_DATA_FRAGMENT_MAX : remaining;

			if (offset == 0)
			{
				data.Operation = BTADV_EXTADV_OP_FIRST;
			}
			else if (remaining > BTADV_HCI_DATA_FRAGMENT_MAX)
			{
				data.Operation = BTADV_EXTADV_OP_INTERMEDIATE;
			}
			else
			{
				data.Operation = BTADV_EXTADV_OP_LAST;
			}

			data.DataLen = (uint8_t)fragmentLen;
			memcpy(data.Data, pPacket->pData + offset, (size_t)fragmentLen);

			int res = BtHciCommand(pDev, opcode, &data,
				(uint8_t)(BTADV_HCI_DATA_HDR_LEN + fragmentLen), NULL, 0);
			if (res != 0)
			{
				if (pPartialWrite != nullptr)
				{
					*pPartialWrite = wroteFragment;
				}
				return res;
			}

			wroteFragment = true;
			offset += fragmentLen;
			remaining -= fragmentLen;
		}

		return 0;
	}

	if (pPacket->Len > BT_ADV_LEGACY_DATA_MAX)
	{
		return -1;
	}

	uint16_t commandBit = ScanResponse ?
		BT_HCI_CAP_CMD_LE_SET_SCAN_RESPONSE_DATA :
		BT_HCI_CAP_CMD_LE_SET_ADV_DATA;
	if (BtHciCapabilitiesCommandSupported(s_pBtAdvCapabilities,
		commandBit) == false)
	{
		return -1;
	}

	BtHciLeAdvData_t data = {};
	data.DataLen = (uint8_t)pPacket->Len;
	memcpy(data.Data, pPacket->pData, (size_t)pPacket->Len);

	uint16_t opcode = ScanResponse ?
		BT_HCI_CMD_CTLR_SET_SCAN_RESP_DATA :
		BT_HCI_CMD_CTLR_SET_ADV_DATA;
	return BtHciCommand(pDev, opcode, &data, sizeof(data), NULL, 0);
}

static bool BtAdvManDataSetPacket(BtAdvPacket_t *pPacket,
	const uint8_t *pData, int Len)
{
	BtAdvData_t *p = BtAdvDataAllocate(pPacket,
		BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, Len + 2);
	if (p == nullptr)
	{
		return false;
	}

	p->Data[0] = (uint8_t)(g_BtAppData.AppDevice.VendorId & 0xFF);
	p->Data[1] = (uint8_t)(g_BtAppData.AppDevice.VendorId >> 8);
	if (Len > 0)
	{
		memcpy(&p->Data[2], pData, (size_t)Len);
	}
	return true;
}

static void BtAdvPacketCopy(BtAdvPacket_t *pDst, const BtAdvPacket_t *pSrc)
{
	memcpy(pDst->pData, pSrc->pData, (size_t)pSrc->Len);
	pDst->Len = pSrc->Len;
}

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
	if (g_BtAppData.State != BTAPP_STATE_ADVERTISING && g_BtAppData.State != BTAPP_STATE_IDLE)
	{
		return false;
	}

	if (AdvLen < 0 || SrLen < 0 ||
		(AdvLen > 0 && pAdvData == nullptr) ||
		(SrLen > 0 && pSrData == nullptr) ||
		AdvLen > s_BtDevExtAdvPkt.MaxLen - 2 ||
		SrLen > s_BtDevExtSrPkt.MaxLen - 2 ||
		(pSrData != nullptr && s_BtDevAdvScannable == false))
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
	bool changeAdv = pAdvData != nullptr;
	bool changeSr = pSrData != nullptr;

	// One full-size reusable packet is enough for both updates. Validate both
	// replacements before stopping advertising so an allocation failure cannot
	// interrupt an otherwise working advertiser.
	BtAdvPacket_t scratch = {
		BTADV_EXTADV_DATA_MAX, 0, s_BtDevScratchBuff
	};

	if (changeAdv)
	{
		BtAdvPacketCopy(&scratch, advpkt);
		if (BtAdvManDataSetPacket(&scratch, pAdvData, AdvLen) == false)
		{
			return false;
		}
	}
	if (changeSr)
	{
		BtAdvPacketCopy(&scratch, srpkt);
		if (BtAdvManDataSetPacket(&scratch, pSrData, SrLen) == false)
		{
			return false;
		}
	}

	bool restart = g_BtAppData.State == BTAPP_STATE_ADVERTISING;
	if (restart)
	{
		if (BtAppAdvDisable() != 0)
		{
			return false;
		}
		g_BtAppData.State = BTAPP_STATE_IDLE;
	}

	bool advWritten = false;
	bool advPartial = false;
	bool srPartial = false;
	if (changeAdv)
	{
		BtAdvPacketCopy(&scratch, advpkt);
		(void)BtAdvManDataSetPacket(&scratch, pAdvData, AdvLen);
		if (BtAdvDataWrite(pDev, &scratch, false, &advPartial) != 0)
		{
			goto rollback;
		}
		advWritten = true;
	}

	if (changeSr)
	{
		BtAdvPacketCopy(&scratch, srpkt);
		(void)BtAdvManDataSetPacket(&scratch, pSrData, SrLen);
		if (BtAdvDataWrite(pDev, &scratch, true, &srPartial) != 0)
		{
			goto rollback;
		}
	}

	// Controller accepted all requested writes. Apply the same validated edits
	// to the retained packets only now, so they remain the rollback copy until
	// the controller state is complete.
	if (changeAdv)
	{
		BtAdvPacketCopy(&scratch, advpkt);
		(void)BtAdvManDataSetPacket(&scratch, pAdvData, AdvLen);
		BtAdvPacketCopy(advpkt, &scratch);
	}
	if (changeSr)
	{
		BtAdvPacketCopy(&scratch, srpkt);
		(void)BtAdvManDataSetPacket(&scratch, pSrData, SrLen);
		BtAdvPacketCopy(srpkt, &scratch);
	}

	if (restart)
	{
		if (BtAppAdvEnable() != 0)
		{
			return false;
		}
		g_BtAppData.State = BTAPP_STATE_ADVERTISING;
	}

	return true;

rollback:
	// A failed fragmented write may have left a partial Host data sequence in
	// the controller. Restore the retained packet before advertising is allowed
	// to resume. A single complete command that fails has not replaced the old
	// data and does not need this extra write.
	if (srPartial && BtAdvDataWrite(pDev, srpkt, true) != 0)
	{
		return false;
	}
	if ((advWritten || advPartial) && BtAdvDataWrite(pDev, advpkt, false) != 0)
	{
		return false;
	}

	if (restart && BtAppAdvEnable() == 0)
	{
		g_BtAppData.State = BTAPP_STATE_ADVERTISING;
	}

	return false;
}

static int BtAppAdvEnable(void)
{
	BtHciDevice_t *pDev = BtAdvHciDev();
	if (pDev == nullptr)
	{
		return -1;
	}

	if (s_BtDevUseExtAdvCmd)
	{
		BtHciLeExtAdvEnable_t x;
		x.Enable = 1;
		x.NumSets = 1;
		x.AdvHandle = 0;
		BtAdvWr16(x.Duration, s_BtDevAdvDuration);
		x.MaxExtAdvEvts = 0;

		return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE,
			&x, sizeof(x), NULL, 0);
	}

	uint8_t enable = 1;
	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADV_ENABLE,
		&enable, sizeof(enable), NULL, 0);
}

static int BtAppAdvDisable(void)
{
	BtHciDevice_t *pDev = BtAdvHciDev();
	if (pDev == nullptr)
	{
		return -1;
	}

	if (s_BtDevUseExtAdvCmd)
	{
		BtHciLeExtAdvEnable_t x;
		x.Enable = 0;
		x.NumSets = 1;
		x.AdvHandle = 0;
		BtAdvWr16(x.Duration, 0);
		x.MaxExtAdvEvts = 0;

		return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE,
			&x, sizeof(x), NULL, 0);
	}

	uint8_t enable = 0;
	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADV_ENABLE,
		&enable, sizeof(enable), NULL, 0);
}

void BtAppAdvStart()
{
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)
		return;

	int res = BtAppAdvEnable();
	if (res == 0)
	{
		g_BtAppData.State = BTAPP_STATE_ADVERTISING;

		return;
	}

	DEBUG_PRINTF("ADV enable failed, status=0x%02x, not advertising\r\n",
				 (unsigned)res);
}

void BtAppAdvStop()
{
	if (BtAppAdvDisable() != 0)
	{
		return;
	}

	g_BtAppData.State = BTAPP_STATE_IDLE;
}

// Portable advertising control. Target ports such as nRF54 BM provide strong
// BtAdvStart/BtAdvStop definitions; the HCI implementation is the generic
// fallback used by SDC and other standard-HCI controllers.
__attribute__((weak)) void BtAdvStart()
{
	BtAppAdvStart();
}

__attribute__((weak)) void BtAdvStop()
{
	BtAppAdvStop();
}

// Strong override of the weak seam in bt_hci_host, the advertising twin of
// BtHciScanTimeout. Core Vol 4 Part E 7.7.65.18 fires this event for two
// unrelated reasons and the host has to tell them apart.
//
// Status 0 means a connection was created. That is not a timeout, and the
// shipped handlers restart advertising or rewrite the advertising data, so
// every successful connection re-armed the set. The connection path owns the
// state from there: BtAppDisconnected restarts advertising when the last link
// goes. The event can arrive either side of LE Connection Complete, so nothing
// is written to the state here either.
//
// Any other status ends advertising in the controller while the host still
// believes the set is enabled. The event defines two: Advertising Timeout
// (0x3C) when the duration elapsed, Limit Reached (0x43) when the extended
// advertising event count did. The state has to leave ADVERTISING or
// BtAppAdvStart returns early at its first line and the device never
// advertises again, which is what a handler like TPHSensorTag's, calling
// BtAdvStart and nothing else, ran into.
//
// The handle names the set that ended, and only set 0 is the application's.
// The periodic train rides BT_ADV_PERIODIC_ADV_HANDLE, and while
// 7.8.61 forces that set non connectable and non scannable and it is enabled
// with neither a duration nor an event limit, so it has no way to reach here
// today, the filter is what keeps that true if it gains one.
void BtAdvSetTerminatedEvt(uint8_t Status, uint8_t AdvHdl, uint16_t ConnHdl)
{
	(void)ConnHdl;

	if (AdvHdl != 0 || Status == BT_HCI_SUCCESS)
	{
		return;
	}

	// Only clear the advertising state. A device with another link up is
	// CONNECTED here, and that is the peer table's to report.
	if (g_BtAppData.State == BTAPP_STATE_ADVERTISING)
	{
		g_BtAppData.State = BTAPP_STATE_IDLE;
	}

	BtHciDevice_t *pDev = BtAdvHciDev();
	if (pDev != nullptr && pDev->AdvTimeout)
	{
		pDev->AdvTimeout();
	}
}

bool BtAppAdvInit(const BtAppCfg_t * const pCfg)
{
	if (pCfg == nullptr || !BtGattInitStatusComplete())
	{
		DEBUG_PRINTF("BtAppAdvInit refused: GATT init error %u\r\n",
			(unsigned)BtGattInitStatusErrorGet());
		return false;
	}

	BtHciDevice_t *pDev = BtAdvHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	s_pBtAdvCapabilities = BtHciCapabilitiesForDeviceGet(pDev);
	if (s_pBtAdvCapabilities == nullptr || s_pBtAdvCapabilities->Valid == 0)
	{
		if (BtHciCapabilitiesRead(pDev, &s_BtAdvReadCapabilities) == false)
		{
			return false;
		}

		// Nothing claimed the host controlled FeatureSet bits on this path:
		// the controller layer that normally does it at start is not the one
		// holding this device. Advertising init runs before anything can
		// connect, which is what Core Vol 4 Part E 7.8.115 requires.
		(void)BtHciHostFeaturesClaim(pDev, &s_BtAdvReadCapabilities);

		s_pBtAdvCapabilities = &s_BtAdvReadCapabilities;
	}

	BtAdvPacket_t *advpkt = &s_BtDevExtAdvPkt;
	BtAdvPacket_t *srpkt  = &s_BtDevExtSrPkt;
	bool scannable = false;

	if (BtAdvEncode(pCfg, advpkt, srpkt, &g_BtAppData.bExtAdv, &scannable) == false)
	{
		return false;
	}
	if (advpkt->Len < 0 || srpkt->Len < 0 ||
		advpkt->Len > advpkt->MaxLen || srpkt->Len > srpkt->MaxLen)
	{
		return false;
	}

	uint16_t extprop = 0;
	if (g_BtAppData.bExtAdv == false)
	{
		extprop |= BTADV_EXTADV_EVT_PROP_LEGACY;
	}
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		extprop |= BTADV_EXTADV_EVT_PROP_CONNECTABLE;
	}
	if (g_BtAppData.bExtAdv == false && (pCfg->Role & BTAPP_ROLE_PERIPHERAL))
	{
		scannable = true;
	}
	if (scannable)
	{
		extprop |= BTADV_EXTADV_EVT_PROP_SCANNABLE;
	}

	uint8_t localType = 0;
	uint8_t localAddr[6];
	BtSmpLocalAddrGet(&localType, localAddr);
	bool useRandom = (localType == BTADDR_TYPE_RAND ||
		localType == BTADDR_TYPE_RANDOM_STATIC);
	if (useRandom && BtAddrIsStaticRandom(localAddr) == false)
	{
		return false;
	}

	bool useScanResponse = scannable && srpkt->Len > 0;
	bool useExtCommands;
	if (BtAdvCommandSetSelect(g_BtAppData.bExtAdv, useRandom,
		useScanResponse, (uint16_t)advpkt->Len,
		useScanResponse ? (uint16_t)srpkt->Len : 0, &useExtCommands) == false)
	{
		return false;
	}

	// LE Set Advertising Enable has no duration parameter. Reject a nonzero
	// timeout when the legacy command set is selected.
	if (useExtCommands == false && pCfg->AdvTimeout != 0)
	{
		return false;
	}

	// Core Vol 4 Part E 7.8.5 allows 0x0020 to 0x4000 for a legacy set, and
	// 7.8.53 allows 0x000020 to 0xFFFFFF for an extended one. Both bounds are
	// applied against a value converted in 32 bits, so the upper test can
	// actually fire; while the conversion truncated to 16 bits it never could,
	// and the whole extended range above 10.24 s was unreachable.
	uint32_t primMin = mSecTo0_625(pCfg->AdvInterval);
	uint32_t primMax = mSecTo0_625((uint32_t)(pCfg->AdvInterval + 50));
	if (primMin < 0x20)
	{
		primMin = 0x20;
	}
	if (primMax < primMin)
	{
		primMax = primMin;
	}
	if ((useExtCommands && primMax > 0xFFFFFF) ||
		(useExtCommands == false && primMax > 0x4000))
	{
		return false;
	}

	// 7.8.56: a Duration of 0x0000 means advertise until the Host disables the
	// set. So a request that is nonzero but rounds to zero has to become the
	// smallest duration the command can express rather than an endless one,
	// and a request past the field is refused rather than wrapped.
	uint32_t advDuration = mSecTo10Ms(pCfg->AdvTimeout);
	if (advDuration > 0xFFFF)
	{
		return false;
	}
	if (advDuration == 0 && pCfg->AdvTimeout != 0)
	{
		advDuration = 1;
	}

	s_BtDevUseExtAdvCmd = useExtCommands;
	s_BtDevAdvScannable = scannable;

	if (useExtCommands)
	{
		// A coding was asked for, the controller has Advertising Coding
		// Selection and it has the v2 command that holds the fields. The
		// coded PHY itself is a separate feature: without it there is no
		// channel the options could apply to, so the request is dropped
		// rather than sent to be refused.
		//
		// A set using legacy PDUs is the same case. 7.8.53: "If legacy
		// advertising PDUs are being used, the Primary_Advertising_PHY shall
		// indicate the LE 1M PHY", and legacy PDUs never reach the secondary
		// channel at all, so neither option has anywhere to apply. Sending
		// the coded PHY anyway is what the controller refuses, and since the
		// payload decides legacy against extended, a small payload turned an
		// accepted coding request into a device that did not advertise.
		bool legacy = (extprop & BTADV_EXTADV_EVT_PROP_LEGACY) != 0;

		bool coding = legacy == false &&
			(s_BtAdvPrimPhyOptions != BT_HCI_ADV_PHY_OPT_NONE ||
			s_BtAdvSecPhyOptions != BT_HCI_ADV_PHY_OPT_NONE) &&
			BtHciCapabilitiesLeFeatureSupported(s_pBtAdvCapabilities,
				BT_HCI_CAP_LE_FEATURE_CODED_PHY) &&
			BtHciCapabilitiesAdvertisingCodingSelectionSupported(
				s_pBtAdvCapabilities);

		BtHciLeExtAdvParamsV2_t p;
		memset(&p, 0, sizeof(p));
		p.AdvHandle = 0;
		BtAdvWr16(p.EvtProp, extprop);
		BtAdvWr24(p.PrimIntervalMin, primMin);
		BtAdvWr24(p.PrimIntervalMax, primMax);
		p.PrimChanMap = 7;
		p.OwnAddrType = useRandom ? BTADDR_TYPE_RAND : BTADDR_TYPE_PUBLIC;
		p.PeerAddrType = 0;
		p.FilterPolicy = 0;
		p.TxPower = 0;
		p.PrimPhy = BTADV_EXTADV_PHY_1M;
		p.SecMaxSkip = 0;
		p.SecPhy = BtHciCapabilitiesLeFeatureSupported(s_pBtAdvCapabilities,
			BT_HCI_CAP_LE_FEATURE_PHY_2M) ?
			BTADV_EXTADV_PHY_2M : BTADV_EXTADV_PHY_1M;
		p.Sid = 0;
		p.ScanReqNotif = 0;

		if (coding)
		{
			// The options field is ignored unless the matching PHY is the LE
			// Coded PHY, so select it on the channel the caller named.
			if (s_BtAdvPrimPhyOptions != BT_HCI_ADV_PHY_OPT_NONE)
			{
				p.PrimPhy = BTADV_EXTADV_PHY_CODED;
				p.PrimPhyOptions = s_BtAdvPrimPhyOptions;
			}
			if (s_BtAdvSecPhyOptions != BT_HCI_ADV_PHY_OPT_NONE)
			{
				p.SecPhy = BTADV_EXTADV_PHY_CODED;
				p.SecPhyOptions = s_BtAdvSecPhyOptions;
			}

			if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM_V2,
				&p, sizeof(p), NULL, 0) != 0)
			{
				return false;
			}
		}
		else if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM,
			&p, sizeof(BtHciLeExtAdvParams_t), NULL, 0) != 0)
		{
			// The v1 layout is the leading 25 octets of the v2 one, and both
			// trailing option fields are zero on this path.
			return false;
		}

		if (useRandom)
		{
			BtHciLeAdvSetRandAddr_t ra;
			ra.AdvHandle = 0;
			memcpy(ra.RandAddr, localAddr, 6);

			if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR,
				&ra, sizeof(ra), NULL, 0) != 0)
			{
				return false;
			}
		}
	}
	else
	{
		BtHciLeAdvParams_t p = {};
		BtAdvWr16(p.IntervalMin, (uint16_t)primMin);
		BtAdvWr16(p.IntervalMax, (uint16_t)primMax);
		p.AdvType = (pCfg->Role & BTAPP_ROLE_PERIPHERAL) ?
			BTADV_LEGACY_ADV_IND :
			(scannable ? BTADV_LEGACY_ADV_SCAN_IND :
			 BTADV_LEGACY_ADV_NONCONN_IND);
		p.OwnAddrType = useRandom ? BTADDR_TYPE_RAND : BTADDR_TYPE_PUBLIC;
		p.PeerAddrType = 0;
		p.ChanMap = 7;
		p.FilterPolicy = 0;

		if (useRandom && BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_RANDOM_ADDR,
			localAddr, sizeof(localAddr), NULL, 0) != 0)
		{
			return false;
		}

		if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADV_PARAM,
			&p, sizeof(p), NULL, 0) != 0)
		{
			return false;
		}
	}

	if (BtAdvDataWrite(pDev, advpkt, false) != 0)
	{
		return false;
	}
	if (useScanResponse && BtAdvDataWrite(pDev, srpkt, true) != 0)
	{
		return false;
	}

	s_BtAdvOwnAddrType = useRandom ? BTADDR_TYPE_RAND : BTADDR_TYPE_PUBLIC;
	memcpy(s_BtAdvOwnAddr, localAddr, 6);
	s_BtDevAdvDuration = (uint16_t)advDuration;

	return true;
}

// --- Periodic advertising, Core Vol 4 Part E 7.8.61 to 7.8.63 ---
//
// A periodic advertising train rides on an extended advertising set. 7.8.61
// refuses the parameters with Invalid HCI Command Parameters when that set is
// scannable, connectable, legacy or anonymous, so the train cannot share the
// set the connectable peripheral above uses. It owns a second set instead,
// which lets a device stay connectable while a train runs.

#pragma pack(push, 1)

typedef struct {
	uint8_t  AdvHandle;
	uint8_t  IntervalMin[2];		//!< 1.25 ms units
	uint8_t  IntervalMax[2];		//!< 1.25 ms units
	uint8_t  Properties[2];
} BtAdvPeriodicParams_t;

typedef struct {
	uint8_t  AdvHandle;
	uint8_t  Operation;
	uint8_t  DataLen;
	uint8_t  Data[BT_ADV_PERIODIC_FRAGMENT_MAX];
} BtAdvPeriodicData_t;			//!< No fragment preference field, unlike ext adv data

typedef struct {
	uint8_t  Enable;
	uint8_t  AdvHandle;
} BtAdvPeriodicEnable_t;

typedef struct {
	uint8_t  Options;
	uint8_t  AdvSid;
	uint8_t  AdvAddrType;
	uint8_t  AdvAddr[6];
	uint8_t  Skip[2];
	uint8_t  SyncTimeout[2];		//!< 10 ms units
	uint8_t  SyncCteType;
} BtAdvPeriodicCreateSync_t;

typedef struct {
	uint8_t  SyncHdl[2];
} BtAdvPeriodicTerminateSync_t;

typedef struct {
	uint8_t  SyncHdl[2];
	uint8_t  Properties[2];
	uint8_t  NumSubevents;
	uint8_t  Subevents[BT_ADV_PAWR_SYNC_SUBEVENT_MAX];
} BtAdvPawrSyncSubevent_t;

typedef struct {
	uint8_t  SyncHdl[2];
	uint8_t  RequestEvent[2];
	uint8_t  RequestSubevent;
	uint8_t  ResponseSubevent;
	uint8_t  ResponseSlot;
	uint8_t  ResponseDataLen;
	uint8_t  ResponseData[BT_ADV_PAWR_RESPONSE_DATA_MAX];
} BtAdvPawrResponseData_t;

typedef struct {
	uint8_t  AdvHandle;
	uint8_t  IntervalMin[2];		//!< 1.25 ms units
	uint8_t  IntervalMax[2];		//!< 1.25 ms units
	uint8_t  Properties[2];
	uint8_t  NumSubevents;
	uint8_t  SubeventInterval;
	uint8_t  ResponseSlotDelay;
	uint8_t  ResponseSlotSpacing;
	uint8_t  NumResponseSlots;
} BtAdvPeriodicParamsV2_t;

typedef struct {
	uint8_t  AdvHandle;
	uint8_t  NumSubeventsWithData;
	uint8_t  Subevent;
	uint8_t  ResponseSlotStart;
	uint8_t  ResponseSlotCount;
	uint8_t  SubeventDataLen;
	uint8_t  SubeventData[BT_ADV_PAWR_SUBEVENT_DATA_MAX];
} BtAdvPeriodicSubeventData_t;

#pragma pack(pop)

static_assert(sizeof(BtAdvPeriodicParams_t) == 7, "periodic advertising parameters must be 7 octets");
static_assert(sizeof(BtAdvPeriodicEnable_t) == 2, "periodic advertising enable must be 2 octets");
static_assert(sizeof(BtAdvPeriodicCreateSync_t) == 14, "create sync must be 14 octets");
static_assert(sizeof(BtAdvPeriodicTerminateSync_t) == 2, "terminate sync must be 2 octets");
static_assert(sizeof(BtAdvPeriodicParamsV2_t) == 12, "periodic advertising parameters v2 must be 12 octets");

// Data operation values, Core Vol 4 Part E 7.8.62. The same encoding the
// extended advertising data command uses, so the BTADV_EXTADV_OP_* values
// above apply here too.

// Advertising event properties for the set the train rides on. Every bit clear
// is non connectable, non scannable, non directed, non legacy and non
// anonymous, which is the only combination 7.8.61 accepts.
#define BT_ADV_PERIODIC_EXT_EVT_PROP		0x0000

static BtHciDevice_t *s_pBtAdvPeriodicDev = nullptr;
static uint8_t s_BtAdvPeriodicData[BT_ADV_PERIODIC_DATA_MAX];
static uint16_t s_BtAdvPeriodicDataLen = 0;
static bool s_BtAdvPeriodicReady = false;
static bool s_BtAdvPeriodicRunning = false;

// Subevents the train was configured for. Zero when the train is not PAwR,
// which is what makes a subevent data command refusable before it is sent.
// Cleared by both init functions, so a plain train after a PAwR one does not
// inherit a count that no longer describes what is on air.
static uint8_t s_BtAdvPawrSubevents = 0;

// One synchronised train at a time. A second would need a handle table and a
// way for the application to say which train a report belongs to; the
// reassembly is written against a single handle and refuses anything else.
// Shared by the plain periodic reports and the PAwR subevent ones, since a
// device is synchronised to one train either way.
static uint16_t s_BtAdvPeriodicSyncHdl;
static bool s_BtAdvPeriodicSyncActive;
static uint8_t s_BtAdvPeriodicRxData[BT_ADV_PERIODIC_DATA_MAX];
static uint16_t s_BtAdvPeriodicRxLen;
static bool s_BtAdvPeriodicRxDrop;

// Periodic advertising interval, in the 1.25 ms units of 7.8.61, to the
// primary advertising interval of 7.8.53, which is in 0.625 ms units. The two
// commands express the same quantity in different units and the value used to
// be passed across unchanged, so the primary train ran at twice the density
// asked for, and the legal minimum periodic interval of 0x0006 landed at 6
// where 7.8.53 requires 0x20, which the controller refuses. Every periodic
// interval under 20 ms failed to start for that reason.
//
// Clamping up rather than refusing is what the 7.8.53 floor means here: the
// primary train only advertises where the periodic one is, so pacing it slower
// than the periodic interval costs nothing but the time to find the train.
static uint32_t BtAdvPeriodicPrimInterval(uint16_t Periodic1_25Ms)
{
	uint32_t units = (uint32_t)Periodic1_25Ms * 2U;

	if (units < 0x20)
	{
		units = 0x20;
	}

	// 0xFFFF in 1.25 ms units doubles to 0x1FFFE, well inside the 0xFFFFFF
	// the field holds, so only the floor can bind.
	return units;
}

// Own_Address_Type for the train's set, or false when the request has no
// meaning here. 7.8.53 gives 0x02 and 0x03 to a controller generated
// resolvable private address taken from the resolving list, which is not what
// a caller naming an address type is asking for, and nothing in this stack
// programs a local IRK. The value used to be passed through unchecked, so a
// caller asking for the identity address got an RPA request on the wire.
//
// BTADDR_TYPE_RANDOM_STATIC resolves to the random device address, the same
// way BtAppAdvInit resolves the type it gets from BtSmpLocalAddrGet.
static bool BtAdvPeriodicOwnAddrType(uint8_t Requested, uint8_t *pOwnAddrType)
{
	switch (Requested)
	{
		case BTADDR_TYPE_PUBLIC:
			*pOwnAddrType = BTADDR_TYPE_PUBLIC;
			return true;
		case BTADDR_TYPE_RAND:
		case BTADDR_TYPE_RANDOM_STATIC:
			*pOwnAddrType = BTADDR_TYPE_RAND;
			return true;
		default:
			return false;
	}
}

// Create the extended advertising set a train rides on. Non connectable and
// non scannable is forced by 7.8.61. Shared by the plain train and the PAwR
// one, which differ only in the parameters command that follows.
static bool BtAdvPeriodicExtSetCreate(BtHciDevice_t * const pDev,
	const BtHciCapabilities_t *pCap, uint16_t IntervalMin,
	uint16_t IntervalMax, uint8_t RequestedAddrType, uint8_t Sid)
{
	uint8_t OwnAddrType = BTADDR_TYPE_PUBLIC;
	if (BtAdvPeriodicOwnAddrType(RequestedAddrType, &OwnAddrType) == false)
	{
		return false;
	}

	BtHciLeExtAdvParams_t ext;
	memset(&ext, 0, sizeof(ext));
	ext.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	BtAdvWr16(ext.EvtProp, BT_ADV_PERIODIC_EXT_EVT_PROP);
	BtAdvWr24(ext.PrimIntervalMin, BtAdvPeriodicPrimInterval(IntervalMin));
	BtAdvWr24(ext.PrimIntervalMax, BtAdvPeriodicPrimInterval(IntervalMax));
	ext.PrimChanMap = 7;
	ext.OwnAddrType = OwnAddrType;
	ext.FilterPolicy = 0;
	ext.TxPower = 0;
	ext.PrimPhy = BTADV_EXTADV_PHY_1M;
	ext.SecMaxSkip = 0;
	ext.SecPhy = BtHciCapabilitiesLeFeatureSupported(pCap,
		BT_HCI_CAP_LE_FEATURE_PHY_2M) ?
		BTADV_EXTADV_PHY_2M : BTADV_EXTADV_PHY_1M;
	ext.Sid = Sid;
	ext.ScanReqNotif = 0;

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM,
		&ext, sizeof(ext), nullptr, 0) != 0)
	{
		return false;
	}

	if (OwnAddrType != BTADDR_TYPE_RAND)
	{
		return true;
	}

	if (BtHciCapabilitiesCommandSupported(pCap,
		BT_HCI_CAP_CMD_LE_SET_ADV_SET_RANDOM_ADDRESS) == false)
	{
		return false;
	}

	// A set advertising from a random address needs one, and it is per set:
	// 7.8.52 takes an Advertising_Handle, and the only other caller in this
	// file names set 0, the connectable one. So the train's set had none and
	// the enable was refused. The set has to exist first, which is why this
	// follows the parameters rather than preceding them.
	uint8_t localType = 0;
	uint8_t localAddr[6];
	BtSmpLocalAddrGet(&localType, localAddr);
	if (BtAddrIsStaticRandom(localAddr) == false)
	{
		return false;
	}

	BtHciLeAdvSetRandAddr_t ra;
	ra.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	memcpy(ra.RandAddr, localAddr, 6);

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR,
		&ra, sizeof(ra), nullptr, 0) == 0;
}

// Shared preflight: interval range and the capabilities every train needs.
static bool BtAdvPeriodicCommonValid(BtHciDevice_t * const pDev,
	uint16_t IntervalMin, uint16_t IntervalMax,
	const BtHciCapabilities_t **ppCap)
{
	if (IntervalMin < BT_ADV_PERIODIC_INTERVAL_MIN ||
		IntervalMax < BT_ADV_PERIODIC_INTERVAL_MIN ||
		IntervalMin > IntervalMax)
	{
		return false;
	}

	const BtHciCapabilities_t *pCap = BtHciCapabilitiesForDeviceGet(pDev);

	if (BtHciCapabilitiesPeriodicAdvertisingSupported(pCap) == false ||
		BtHciCapabilitiesLeFeatureSupported(pCap,
			BT_HCI_CAP_LE_FEATURE_EXT_ADV) == false ||
		BtHciCapabilitiesCommandSupported(pCap,
			BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS) == false ||
		BtHciCapabilitiesCommandSupported(pCap,
			BT_HCI_CAP_CMD_LE_SET_EXT_ADV_ENABLE) == false)
	{
		return false;
	}

	// The train needs its own extended advertising set, so a controller that
	// reports only one cannot run this alongside connectable advertising.
	if ((pCap->Valid & BT_HCI_CAP_VALID_ADV_SET_COUNT) != 0 &&
		pCap->AdvSetCount <= BT_ADV_PERIODIC_ADV_HANDLE)
	{
		return false;
	}

	*ppCap = pCap;
	return true;
}

bool BtAdvPeriodicInit(const BtAdvPeriodicCfg_t *pCfg)
{
	BtHciDevice_t *pDev = BtAdvHciDev();

	s_pBtAdvPeriodicDev = nullptr;
	s_BtAdvPeriodicReady = false;
	s_BtAdvPeriodicRunning = false;
	s_BtAdvPeriodicDataLen = 0;
	s_BtAdvPawrSubevents = 0;

	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	// 7.8.61 puts both interval parameters in 1.25 ms units with a range of
	// 0x0006 to 0xFFFF, and requires min to be no greater than max.
	const BtHciCapabilities_t *pCap = nullptr;
	if (BtAdvPeriodicCommonValid(pDev, pCfg->IntervalMin, pCfg->IntervalMax,
		&pCap) == false)
	{
		return false;
	}

	if (BtAdvPeriodicExtSetCreate(pDev, pCap, pCfg->IntervalMin,
		pCfg->IntervalMax, pCfg->OwnAddrType, pCfg->Sid) == false)
	{
		return false;
	}

	BtAdvPeriodicParams_t p;	memset(&p, 0, sizeof(p));
	p.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	BtAdvWr16(p.IntervalMin, pCfg->IntervalMin);
	BtAdvWr16(p.IntervalMax, pCfg->IntervalMax);
	BtAdvWr16(p.Properties, pCfg->IncludeTxPower ?
		BT_ADV_PERIODIC_PROP_INCLUDE_TXPOWER : 0);

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM,
		&p, sizeof(p), nullptr, 0) != 0)
	{
		return false;
	}

	s_pBtAdvPeriodicDev = pDev;
	s_BtAdvPeriodicReady = true;

	return true;
}

bool BtAdvPeriodicDataSet(const uint8_t *pData, size_t Len)
{
	if (s_BtAdvPeriodicReady == false || s_pBtAdvPeriodicDev == nullptr ||
		Len > BT_ADV_PERIODIC_DATA_MAX || (Len > 0 && pData == nullptr))
	{
		return false;
	}

	BtHciDevice_t *pDev = s_pBtAdvPeriodicDev;
	size_t off = 0;
	bool first = true;

	// The controller refuses an enable while the set holds a partial update,
	// so a run that fails part way leaves the train unusable until a whole
	// set of fragments lands. Record zero so a later enable is refused here
	// rather than by the controller.
	s_BtAdvPeriodicDataLen = 0;

	do
	{
		size_t chunk = Len - off;
		if (chunk > BT_ADV_PERIODIC_FRAGMENT_MAX)
		{
			chunk = BT_ADV_PERIODIC_FRAGMENT_MAX;
		}
		bool last = (off + chunk) >= Len;

		BtAdvPeriodicData_t d;
		memset(&d, 0, sizeof(d));
		d.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
		if (first && last)
		{
			d.Operation = BTADV_EXTADV_OP_COMPLETE;
		}
		else if (first)
		{
			d.Operation = BTADV_EXTADV_OP_FIRST;
		}
		else if (last)
		{
			d.Operation = BTADV_EXTADV_OP_LAST;
		}
		else
		{
			d.Operation = BTADV_EXTADV_OP_INTERMEDIATE;
		}
		d.DataLen = (uint8_t)chunk;
		if (chunk > 0)
		{
			memcpy(d.Data, &pData[off], chunk);
		}

		// Only the octets in use are sent, not the whole fragment buffer.
		if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA,
			&d, (uint8_t)(3 + chunk), nullptr, 0) != 0)
		{
			return false;
		}

		off += chunk;
		first = false;
	} while (off < Len);

	if (Len > 0)
	{
		memcpy(s_BtAdvPeriodicData, pData, Len);
	}
	s_BtAdvPeriodicDataLen = (uint16_t)Len;

	return true;
}

size_t BtAdvPeriodicDataGet(uint8_t *pBuff, size_t BuffLen)
{
	if (pBuff != nullptr && BuffLen >= s_BtAdvPeriodicDataLen)
	{
		memcpy(pBuff, s_BtAdvPeriodicData, s_BtAdvPeriodicDataLen);
	}

	return s_BtAdvPeriodicDataLen;
}

bool BtAdvPeriodicStart(void)
{
	if (s_BtAdvPeriodicReady == false || s_pBtAdvPeriodicDev == nullptr)
	{
		return false;
	}

	BtHciDevice_t *pDev = s_pBtAdvPeriodicDev;

	// 7.8.63 turns the train on for the set. It does not start until the set
	// itself is advertising, so the extended enable follows. Doing it in this
	// order means the first extended advertising event already points at a
	// live train.
	BtAdvPeriodicEnable_t pe;
	pe.Enable = 1;
	pe.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE,
		&pe, sizeof(pe), nullptr, 0) != 0)
	{
		return false;
	}

	BtHciLeExtAdvEnable_t ee;
	memset(&ee, 0, sizeof(ee));
	ee.Enable = 1;
	ee.NumSets = 1;
	ee.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE,
		&ee, sizeof(ee), nullptr, 0) != 0)
	{
		// Leave nothing half started: the train is on but nothing advertises
		// it, which is a state no caller asked for.
		pe.Enable = 0;
		(void)BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE,
			&pe, sizeof(pe), nullptr, 0);
		return false;
	}

	s_BtAdvPeriodicRunning = true;

	return true;
}

bool BtAdvPeriodicStop(void)
{
	if (s_pBtAdvPeriodicDev == nullptr)
	{
		return false;
	}

	BtHciDevice_t *pDev = s_pBtAdvPeriodicDev;
	bool ok = true;

	BtAdvPeriodicEnable_t pe;
	pe.Enable = 0;
	pe.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE,
		&pe, sizeof(pe), nullptr, 0) != 0)
	{
		ok = false;
	}

	// Disabling the set has no effect on a train that is already running, per
	// 7.8.63, so both have to be turned off and the second is attempted even
	// when the first was refused.
	BtHciLeExtAdvEnable_t ee;
	memset(&ee, 0, sizeof(ee));
	ee.Enable = 0;
	ee.NumSets = 1;
	ee.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE,
		&ee, sizeof(ee), nullptr, 0) != 0)
	{
		ok = false;
	}

	if (ok)
	{
		s_BtAdvPeriodicRunning = false;
	}

	return ok;
}

bool BtAdvPeriodicIsRunning(void)
{
	return s_BtAdvPeriodicRunning;
}

// --- Periodic Advertising with Responses, advertiser ---

bool BtAdvPawrInit(const BtAdvPawrCfg_t *pCfg)
{
	BtHciDevice_t *pDev = BtAdvHciDev();

	s_pBtAdvPeriodicDev = nullptr;
	s_BtAdvPeriodicReady = false;
	s_BtAdvPeriodicRunning = false;
	s_BtAdvPeriodicDataLen = 0;
	s_BtAdvPawrSubevents = 0;

	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	// A PAwR train has at least one subevent; zero would be the plain train,
	// which BtAdvPeriodicInit already covers.
	if (pCfg->NumSubevents == 0 ||
		pCfg->NumSubevents > BT_ADV_PAWR_SUBEVENT_MAX ||
		pCfg->SubeventInterval < BT_ADV_PAWR_SUBEVENT_INTERVAL_MIN)
	{
		return false;
	}

	// The response slot parameters are all or nothing. 7.8.61 gives each of
	// them zero as the no response slots value, and a train with slots but no
	// spacing, or spacing but no slots, is not a shape the controller accepts.
	bool slots = pCfg->NumResponseSlots != 0;
	if (slots)
	{
		if (pCfg->ResponseSlotDelay == 0 ||
			pCfg->ResponseSlotDelay > BT_ADV_PAWR_RSP_SLOT_DELAY_MAX ||
			pCfg->ResponseSlotSpacing < BT_ADV_PAWR_RSP_SLOT_SPACING_MIN)
		{
			return false;
		}
	}
	else if (pCfg->ResponseSlotDelay != 0 || pCfg->ResponseSlotSpacing != 0)
	{
		return false;
	}

	const BtHciCapabilities_t *pCap = nullptr;
	if (BtAdvPeriodicCommonValid(pDev, pCfg->IntervalMin, pCfg->IntervalMax,
		&pCap) == false)
	{
		return false;
	}

	if (BtHciCapabilitiesPawrAdvertiserSupported(pCap) == false)
	{
		return false;
	}

	if (BtAdvPeriodicExtSetCreate(pDev, pCap, pCfg->IntervalMin,
		pCfg->IntervalMax, pCfg->OwnAddrType, pCfg->Sid) == false)
	{
		return false;
	}

	BtAdvPeriodicParamsV2_t p;
	memset(&p, 0, sizeof(p));
	p.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	BtAdvWr16(p.IntervalMin, pCfg->IntervalMin);
	BtAdvWr16(p.IntervalMax, pCfg->IntervalMax);
	BtAdvWr16(p.Properties, pCfg->IncludeTxPower ?
		BT_ADV_PERIODIC_PROP_INCLUDE_TXPOWER : 0);
	p.NumSubevents = pCfg->NumSubevents;
	p.SubeventInterval = pCfg->SubeventInterval;
	p.ResponseSlotDelay = pCfg->ResponseSlotDelay;
	p.ResponseSlotSpacing = pCfg->ResponseSlotSpacing;
	p.NumResponseSlots = pCfg->NumResponseSlots;

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM_V2,
		&p, sizeof(p), nullptr, 0) != 0)
	{
		return false;
	}

	s_pBtAdvPeriodicDev = pDev;
	s_BtAdvPeriodicReady = true;
	s_BtAdvPawrSubevents = pCfg->NumSubevents;

	return true;
}

bool BtAdvPawrSubeventDataSet(uint8_t Subevent, uint8_t RspSlotStart,
	uint8_t RspSlotCount, const uint8_t *pData, size_t Len)
{
	if (s_BtAdvPeriodicReady == false || s_pBtAdvPeriodicDev == nullptr ||
		s_BtAdvPawrSubevents == 0 || Subevent >= s_BtAdvPawrSubevents ||
		Len > BT_ADV_PAWR_SUBEVENT_DATA_MAX || (Len > 0 && pData == nullptr))
	{
		return false;
	}

	// One subevent per command. The command can hold several, but the request
	// event names a range the caller walks one at a time, and a single
	// subevent per command keeps a refusal attributable to one subevent.
	BtAdvPeriodicSubeventData_t s;
	memset(&s, 0, sizeof(s));
	s.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	s.NumSubeventsWithData = 1;
	s.Subevent = Subevent;
	s.ResponseSlotStart = RspSlotStart;
	s.ResponseSlotCount = RspSlotCount;
	s.SubeventDataLen = (uint8_t)Len;
	if (Len > 0)
	{
		memcpy(s.SubeventData, pData, Len);
	}

	// Only the octets in use are sent: handle, count, then the one subevent
	// block of four header octets and its data.
	return BtHciCommand(s_pBtAdvPeriodicDev,
		BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_SUBEVENT_DATA,
		&s, (uint8_t)(6 + Len), nullptr, 0) == 0;
}

void BtAdvPawrSubeventDataRequestEvt(uint8_t AdvHandle, uint8_t SubeventStart,
	uint8_t SubeventDataCount)
{
	if (AdvHandle != BT_ADV_PERIODIC_ADV_HANDLE)
	{
		return;
	}

	BtAdvPawrSubeventDataRequest(AdvHandle, SubeventStart, SubeventDataCount);
}

void BtAdvPawrResponseReportEvt(uint8_t AdvHandle, uint8_t Subevent,
	uint8_t TxStatus, uint8_t NumResponses, const uint8_t *pResponses,
	size_t ResponsesLen)
{
	(void)TxStatus;

	if (AdvHandle != BT_ADV_PERIODIC_ADV_HANDLE || pResponses == nullptr)
	{
		return;
	}

	// Each response is a 6 octet header, TxPower, RSSI, CTE type, response
	// slot, data status and length, then its data. Walk by stride and bound
	// every step against what the event delivered, because NumResponses and
	// each length come off the wire.
	const uint8_t *cur = pResponses;
	const uint8_t *end = pResponses + ResponsesLen;

	for (uint8_t i = 0; i < NumResponses; i++)
	{
		if (cur + 6 > end)
		{
			break;
		}

		int8_t rssi = (int8_t)cur[1];
		uint8_t slot = cur[3];
		uint8_t dataStatus = cur[4];
		uint8_t len = cur[5];

		if (cur + 6 + len > end)
		{
			break;
		}

		// Only a complete response is handed up. An incomplete one is a piece
		// of an answer, and there is no second report to finish it with.
		if (dataStatus == BT_ADV_PERIODIC_DATA_COMPLETE)
		{
			BtAdvPawrResponse(AdvHandle, Subevent, slot, rssi, len, &cur[6]);
		}

		cur += 6 + len;
	}
}

// --- PAwR scanner ---

// Reassembly for a synchronised PAwR train. A subevent report can arrive in
// several parts the same way a plain periodic report can, and the event and
// subevent a reply quotes belong to the first part.
static uint16_t s_BtAdvPawrRxEvent;
static uint8_t s_BtAdvPawrRxSubevent;
static bool s_BtAdvPawrRxDrop;

bool BtAdvPawrSyncSubeventSet(uint16_t SyncHdl,
	const uint8_t *pSubevents, uint8_t Count)
{
	BtHciDevice_t *pDev = BtAdvHciDev();

	if (pDev == nullptr || pSubevents == nullptr || Count == 0 ||
		Count > BT_ADV_PAWR_SYNC_SUBEVENT_MAX)
	{
		return false;
	}

	for (uint8_t i = 0; i < Count; i++)
	{
		if (pSubevents[i] > BT_ADV_PAWR_SUBEVENT_NUM_MAX)
		{
			return false;
		}
	}

	if (BtHciCapabilitiesCommandSupported(BtHciCapabilitiesForDeviceGet(pDev),
		BT_HCI_CAP_CMD_LE_SET_PERIODIC_SYNC_SUBEVENT) == false)
	{
		return false;
	}

	BtAdvPawrSyncSubevent_t s;
	memset(&s, 0, sizeof(s));
	BtAdvWr16(s.SyncHdl, SyncHdl);
	BtAdvWr16(s.Properties, 0);
	s.NumSubevents = Count;
	memcpy(s.Subevents, pSubevents, Count);

	// Only the subevents named are sent, not the whole array.
	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_SYNC_SUBEVENT,
		&s, (uint8_t)(5 + Count), nullptr, 0) == 0;
}

bool BtAdvPawrResponseDataSet(uint16_t SyncHdl,
	uint16_t RequestEvent, uint8_t RequestSubevent, uint8_t ResponseSubevent,
	uint8_t ResponseSlot, const uint8_t *pData, size_t Len)
{
	BtHciDevice_t *pDev = BtAdvHciDev();

	if (pDev == nullptr || Len > BT_ADV_PAWR_RESPONSE_DATA_MAX ||
		(Len > 0 && pData == nullptr) ||
		RequestSubevent > BT_ADV_PAWR_SUBEVENT_NUM_MAX ||
		ResponseSubevent > BT_ADV_PAWR_SUBEVENT_NUM_MAX)
	{
		return false;
	}

	if (BtHciCapabilitiesCommandSupported(BtHciCapabilitiesForDeviceGet(pDev),
		BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_RESPONSE_DATA) == false)
	{
		return false;
	}

	BtAdvPawrResponseData_t r;
	memset(&r, 0, sizeof(r));
	BtAdvWr16(r.SyncHdl, SyncHdl);
	BtAdvWr16(r.RequestEvent, RequestEvent);
	r.RequestSubevent = RequestSubevent;
	r.ResponseSubevent = ResponseSubevent;
	r.ResponseSlot = ResponseSlot;
	r.ResponseDataLen = (uint8_t)Len;
	if (Len > 0)
	{
		memcpy(r.ResponseData, pData, Len);
	}

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_RESPONSE_DATA,
		&r, (uint8_t)(8 + Len), nullptr, 0) == 0;
}

void BtAdvPawrSubeventReportEvt(uint16_t SyncHdl, uint16_t EventCounter,
	uint8_t Subevent, int8_t TxPower, int8_t Rssi, uint8_t DataStatus,
	size_t Len, const uint8_t *pData)
{
	(void)TxPower;

	if (s_BtAdvPeriodicSyncActive == false || SyncHdl != s_BtAdvPeriodicSyncHdl)
	{
		return;
	}

	if (DataStatus == BT_ADV_PERIODIC_DATA_RX_FAILED)
	{
		s_BtAdvPeriodicRxLen = 0;
		s_BtAdvPawrRxDrop = false;
		return;
	}

	// The event and subevent a reply quotes belong to the packet the payload
	// started in, so they are taken from the first part and kept.
	if (s_BtAdvPeriodicRxLen == 0)
	{
		s_BtAdvPawrRxEvent = EventCounter;
		s_BtAdvPawrRxSubevent = Subevent;
	}

	if (Len > 0 && pData != nullptr)
	{
		if (s_BtAdvPeriodicRxLen + Len > BT_ADV_PERIODIC_DATA_MAX)
		{
			s_BtAdvPawrRxDrop = true;
		}
		else
		{
			memcpy(&s_BtAdvPeriodicRxData[s_BtAdvPeriodicRxLen], pData, Len);
			s_BtAdvPeriodicRxLen = (uint16_t)(s_BtAdvPeriodicRxLen + Len);
		}
	}

	if (DataStatus == BT_ADV_PERIODIC_DATA_MORE)
	{
		return;
	}

	if (DataStatus == BT_ADV_PERIODIC_DATA_COMPLETE &&
		s_BtAdvPawrRxDrop == false)
	{
		BtAdvPawrSubeventReport(SyncHdl, s_BtAdvPawrRxEvent,
			s_BtAdvPawrRxSubevent, Rssi, s_BtAdvPeriodicRxLen,
			s_BtAdvPeriodicRxData);
	}

	s_BtAdvPeriodicRxLen = 0;
	s_BtAdvPawrRxDrop = false;
}

// --- Synchronising to someone else's train ---

// Create Sync Options bit 0 selects the periodic advertiser list instead of
// the address and set id in the command. Nothing populates that list yet, so
// the option is always clear and the train is always named explicitly.
#define BT_ADV_PERIODIC_SYNC_OPT_DISABLE_REPORTING	(1U << 1)

bool BtAdvPeriodicSyncCreate(const BtAdvPeriodicSyncCfg_t *pCfg)
{
	BtHciDevice_t *pDev = BtAdvHciDev();

	if (pDev == nullptr || pCfg == nullptr ||
		pCfg->AdvSid > BT_ADV_PERIODIC_SID_MAX ||
		pCfg->AdvAddrType > 1 ||
		pCfg->Skip > BT_ADV_PERIODIC_SKIP_MAX ||
		pCfg->SyncTimeout < BT_ADV_PERIODIC_SYNC_TIMEOUT_MIN ||
		pCfg->SyncTimeout > BT_ADV_PERIODIC_SYNC_TIMEOUT_MAX)
	{
		return false;
	}

	if (BtHciCapabilitiesPeriodicSyncSupported(
		BtHciCapabilitiesForDeviceGet(pDev)) == false)
	{
		return false;
	}

	BtAdvPeriodicCreateSync_t c;
	memset(&c, 0, sizeof(c));
	c.Options = pCfg->DisableReporting ?
		BT_ADV_PERIODIC_SYNC_OPT_DISABLE_REPORTING : 0;
	c.AdvSid = pCfg->AdvSid;
	c.AdvAddrType = pCfg->AdvAddrType;
	memcpy(c.AdvAddr, pCfg->AdvAddr, sizeof(c.AdvAddr));
	BtAdvWr16(c.Skip, pCfg->Skip);
	BtAdvWr16(c.SyncTimeout, pCfg->SyncTimeout);
	c.SyncCteType = 0;

	// Start reassembly clean: a request that succeeds must not deliver the
	// tail of whatever the previous train was sending.
	s_BtAdvPeriodicRxLen = 0;
	s_BtAdvPeriodicRxDrop = false;

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC,
		&c, sizeof(c), nullptr, 0) == 0;
}

bool BtAdvPeriodicSyncCancel(void)
{
	BtHciDevice_t *pDev = BtAdvHciDev();

	if (pDev == nullptr)
	{
		return false;
	}

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL,
		nullptr, 0, nullptr, 0) == 0;
}

bool BtAdvPeriodicSyncTerminate(uint16_t SyncHdl)
{
	BtHciDevice_t *pDev = BtAdvHciDev();

	if (pDev == nullptr)
	{
		return false;
	}

	BtAdvPeriodicTerminateSync_t t;
	BtAdvWr16(t.SyncHdl, SyncHdl);

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC,
		&t, sizeof(t), nullptr, 0) != 0)
	{
		return false;
	}

	// No Sync Lost event follows a terminate, so the state is dropped here.
	if (s_BtAdvPeriodicSyncActive && s_BtAdvPeriodicSyncHdl == SyncHdl)
	{
		s_BtAdvPeriodicSyncActive = false;
		s_BtAdvPeriodicRxLen = 0;
		s_BtAdvPeriodicRxDrop = false;
	}

	return true;
}

// Strong overrides of the seams bt_hci_host declares weak. Track the handle
// first, then hand the event to the application, so a callback that turns
// straight round and terminates the sync sees consistent state.
void BtAdvPeriodicSyncEstablishedEvt(uint8_t Status, uint16_t SyncHdl,
	uint8_t AdvSid, uint8_t AdvAddrType, const uint8_t AdvAddr[6],
	uint8_t AdvPhy, uint16_t Interval)
{
	if (Status == 0)
	{
		s_BtAdvPeriodicSyncHdl = SyncHdl;
		s_BtAdvPeriodicSyncActive = true;
	}
	s_BtAdvPeriodicRxLen = 0;
	s_BtAdvPeriodicRxDrop = false;

	BtAdvPeriodicSyncEstablished(Status, SyncHdl, AdvSid, AdvAddrType,
		AdvAddr, AdvPhy, Interval);
}

void BtAdvPeriodicSyncLostEvt(uint16_t SyncHdl)
{
	if (s_BtAdvPeriodicSyncActive && s_BtAdvPeriodicSyncHdl == SyncHdl)
	{
		s_BtAdvPeriodicSyncActive = false;
	}
	s_BtAdvPeriodicRxLen = 0;
	s_BtAdvPeriodicRxDrop = false;

	BtAdvPeriodicSyncLost(SyncHdl);
}

void BtAdvPeriodicReportFragment(uint16_t SyncHdl, int8_t TxPower, int8_t Rssi,
	uint8_t DataStatus, size_t Len, const uint8_t *pData)
{
	if (s_BtAdvPeriodicSyncActive == false || SyncHdl != s_BtAdvPeriodicSyncHdl)
	{
		return;
	}

	// 0xFF says the controller failed to receive a subevent PDU. There is no
	// payload and whatever was accumulating cannot be completed.
	if (DataStatus == BT_ADV_PERIODIC_DATA_RX_FAILED)
	{
		s_BtAdvPeriodicRxLen = 0;
		s_BtAdvPeriodicRxDrop = false;
		return;
	}

	if (Len > 0 && pData != nullptr)
	{
		if (s_BtAdvPeriodicRxLen + Len > sizeof(s_BtAdvPeriodicRxData))
		{
			// More than a whole payload can hold. Drop the rest of this one
			// rather than deliver a prefix that looks complete.
			s_BtAdvPeriodicRxDrop = true;
		}
		else
		{
			memcpy(&s_BtAdvPeriodicRxData[s_BtAdvPeriodicRxLen], pData, Len);
			s_BtAdvPeriodicRxLen = (uint16_t)(s_BtAdvPeriodicRxLen + Len);
		}
	}

	if (DataStatus == BT_ADV_PERIODIC_DATA_MORE)
	{
		return;
	}

	// Truncated means the rest is never coming, so what is held is a fragment
	// of a payload the sender meant to be longer. Reporting it would look like
	// a whole one.
	if (DataStatus == BT_ADV_PERIODIC_DATA_COMPLETE &&
		s_BtAdvPeriodicRxDrop == false)
	{
		BtAdvPeriodicSyncReport(SyncHdl, TxPower, Rssi,
			s_BtAdvPeriodicRxLen, s_BtAdvPeriodicRxData);
	}

	s_BtAdvPeriodicRxLen = 0;
	s_BtAdvPeriodicRxDrop = false;
}
