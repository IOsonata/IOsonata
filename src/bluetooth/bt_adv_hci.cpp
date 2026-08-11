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
	if ((useExtCommands && primMax > 0xFFFFFF) ||
		(useExtCommands == false && primMax > 0x4000))
	{
		return false;
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
		bool coding = (s_BtAdvPrimPhyOptions != BT_HCI_ADV_PHY_OPT_NONE ||
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
	s_BtDevAdvDuration = mSecTo10Ms(pCfg->AdvTimeout);

	return true;
}
