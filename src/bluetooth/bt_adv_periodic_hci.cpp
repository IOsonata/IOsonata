/**-------------------------------------------------------------------------
@file	bt_adv_periodic_hci.cpp

@brief	Periodic advertising over standard HCI.

        A periodic advertising train rides on an extended advertising set.
        Core Vol 4 Part E 7.8.61 refuses the parameters with Invalid HCI
        Command Parameters when that set is scannable, connectable, legacy or
        anonymous, so the train cannot share the set bt_adv_hci uses for a
        connectable peripheral. This module owns a second set instead, which
        lets a device stay connectable while a train runs.

        Drives everything through BtHciCommand, so it works with any HCI
        controller, not just one vendor's.

@author	Hoang Nguyen Hoan
@date	Aug. 11, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_adv.h"

/******** For DEBUG Trace ************/
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

#pragma pack(push, 1)

typedef struct {
	uint8_t  AdvHandle;
	uint8_t  EvtProp[2];
	uint8_t  PrimIntervalMin[3];
	uint8_t  PrimIntervalMax[3];
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
} BtAdvPeriodicExtParams_t;

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
} BtAdvPeriodicData_t;

typedef struct {
	uint8_t  Enable;
	uint8_t  AdvHandle;
} BtAdvPeriodicEnable_t;

typedef struct {
	uint8_t  Enable;
	uint8_t  NumSets;
	uint8_t  AdvHandle;
	uint8_t  Duration[2];
	uint8_t  MaxExtAdvEvts;
} BtAdvPeriodicExtEnable_t;

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

static_assert(sizeof(BtAdvPeriodicExtParams_t) == 25, "extended advertising parameters must be 25 octets");
static_assert(sizeof(BtAdvPeriodicParams_t) == 7, "periodic advertising parameters must be 7 octets");
static_assert(sizeof(BtAdvPeriodicEnable_t) == 2, "periodic advertising enable must be 2 octets");
static_assert(sizeof(BtAdvPeriodicExtEnable_t) == 6, "extended advertising enable must be 6 octets");
static_assert(sizeof(BtAdvPeriodicCreateSync_t) == 14, "create sync must be 14 octets");
static_assert(sizeof(BtAdvPeriodicTerminateSync_t) == 2, "terminate sync must be 2 octets");
static_assert(sizeof(BtAdvPeriodicParamsV2_t) == 12, "periodic advertising parameters v2 must be 12 octets");

// Data operation values, Core Vol 4 Part E 7.8.62. The same encoding the
// extended advertising data command uses.
#define BT_ADV_PERIODIC_OP_INTERMEDIATE		0x00
#define BT_ADV_PERIODIC_OP_FIRST			0x01
#define BT_ADV_PERIODIC_OP_LAST				0x02
#define BT_ADV_PERIODIC_OP_COMPLETE			0x03

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

static void BtAdvPeriodicWr16(uint8_t *pDest, uint16_t Value)
{
	pDest[0] = (uint8_t)(Value & 0xFF);
	pDest[1] = (uint8_t)(Value >> 8);
}

static void BtAdvPeriodicWr24(uint8_t *pDest, uint32_t Value)
{
	pDest[0] = (uint8_t)(Value & 0xFF);
	pDest[1] = (uint8_t)((Value >> 8) & 0xFF);
	pDest[2] = (uint8_t)((Value >> 16) & 0xFF);
}

// Create the extended advertising set a train rides on. Non connectable and
// non scannable is forced by 7.8.61; the primary interval only paces the
// extended advertising events that point at the train, so it follows the
// periodic interval rather than being configured separately. Shared by the
// plain train and the PAwR one, which differ only in the parameters command
// that follows.
static bool BtAdvPeriodicExtSetCreate(BtHciDevice_t * const pDev,
	const BtHciCapabilities_t *pCap, uint16_t IntervalMin,
	uint16_t IntervalMax, uint8_t OwnAddrType, uint8_t Sid)
{
	BtAdvPeriodicExtParams_t ext;
	memset(&ext, 0, sizeof(ext));
	ext.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	BtAdvPeriodicWr16(ext.EvtProp, BT_ADV_PERIODIC_EXT_EVT_PROP);
	BtAdvPeriodicWr24(ext.PrimIntervalMin, IntervalMin);
	BtAdvPeriodicWr24(ext.PrimIntervalMax, IntervalMax);
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

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM,
		&ext, sizeof(ext), nullptr, 0) == 0;
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

bool BtAdvPeriodicInit(BtHciDevice_t * const pDev, const BtAdvPeriodicCfg_t *pCfg)
{
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
	BtAdvPeriodicWr16(p.IntervalMin, pCfg->IntervalMin);
	BtAdvPeriodicWr16(p.IntervalMax, pCfg->IntervalMax);
	BtAdvPeriodicWr16(p.Properties, pCfg->IncludeTxPower ?
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
			d.Operation = BT_ADV_PERIODIC_OP_COMPLETE;
		}
		else if (first)
		{
			d.Operation = BT_ADV_PERIODIC_OP_FIRST;
		}
		else if (last)
		{
			d.Operation = BT_ADV_PERIODIC_OP_LAST;
		}
		else
		{
			d.Operation = BT_ADV_PERIODIC_OP_INTERMEDIATE;
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

	BtAdvPeriodicExtEnable_t ee;
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
	BtAdvPeriodicExtEnable_t ee;
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

bool BtAdvPawrInit(BtHciDevice_t * const pDev, const BtAdvPawrCfg_t *pCfg)
{
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
	BtAdvPeriodicWr16(p.IntervalMin, pCfg->IntervalMin);
	BtAdvPeriodicWr16(p.IntervalMax, pCfg->IntervalMax);
	BtAdvPeriodicWr16(p.Properties, pCfg->IncludeTxPower ?
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

__attribute__((weak)) void BtAdvPawrSubeventDataRequest(uint8_t AdvHandle,
	uint8_t SubeventStart, uint8_t SubeventDataCount)
{
	(void)AdvHandle; (void)SubeventStart; (void)SubeventDataCount;
}

__attribute__((weak)) void BtAdvPawrResponse(uint8_t AdvHandle,
	uint8_t Subevent, uint8_t ResponseSlot, int8_t Rssi, size_t Len,
	const uint8_t *pData)
{
	(void)AdvHandle; (void)Subevent; (void)ResponseSlot; (void)Rssi;
	(void)Len; (void)pData;
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
static uint16_t s_BtAdvPawrRxLen;
static bool s_BtAdvPawrRxDrop;

bool BtAdvPawrSyncSubeventSet(BtHciDevice_t * const pDev, uint16_t SyncHdl,
	const uint8_t *pSubevents, uint8_t Count)
{
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
	BtAdvPeriodicWr16(s.SyncHdl, SyncHdl);
	BtAdvPeriodicWr16(s.Properties, 0);
	s.NumSubevents = Count;
	memcpy(s.Subevents, pSubevents, Count);

	// Only the subevents named are sent, not the whole array.
	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_SYNC_SUBEVENT,
		&s, (uint8_t)(5 + Count), nullptr, 0) == 0;
}

bool BtAdvPawrResponseDataSet(BtHciDevice_t * const pDev, uint16_t SyncHdl,
	uint16_t RequestEvent, uint8_t RequestSubevent, uint8_t ResponseSubevent,
	uint8_t ResponseSlot, const uint8_t *pData, size_t Len)
{
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
	BtAdvPeriodicWr16(r.SyncHdl, SyncHdl);
	BtAdvPeriodicWr16(r.RequestEvent, RequestEvent);
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

__attribute__((weak)) void BtAdvPawrSubeventReport(uint16_t SyncHdl,
	uint16_t EventCounter, uint8_t Subevent, int8_t Rssi, size_t Len,
	const uint8_t *pData)
{
	(void)SyncHdl; (void)EventCounter; (void)Subevent; (void)Rssi;
	(void)Len; (void)pData;
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

bool BtAdvPeriodicSyncCreate(BtHciDevice_t * const pDev,
	const BtAdvPeriodicSyncCfg_t *pCfg)
{
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
	BtAdvPeriodicWr16(c.Skip, pCfg->Skip);
	BtAdvPeriodicWr16(c.SyncTimeout, pCfg->SyncTimeout);
	c.SyncCteType = 0;

	// Start reassembly clean: a request that succeeds must not deliver the
	// tail of whatever the previous train was sending.
	s_BtAdvPeriodicRxLen = 0;
	s_BtAdvPeriodicRxDrop = false;

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC,
		&c, sizeof(c), nullptr, 0) == 0;
}

bool BtAdvPeriodicSyncCancel(BtHciDevice_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL,
		nullptr, 0, nullptr, 0) == 0;
}

bool BtAdvPeriodicSyncTerminate(BtHciDevice_t * const pDev, uint16_t SyncHdl)
{
	if (pDev == nullptr)
	{
		return false;
	}

	BtAdvPeriodicTerminateSync_t t;
	BtAdvPeriodicWr16(t.SyncHdl, SyncHdl);

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

__attribute__((weak)) void BtAdvPeriodicSyncEstablished(uint8_t Status,
	uint16_t SyncHdl, uint8_t AdvSid, uint8_t AdvAddrType,
	const uint8_t AdvAddr[6], uint8_t AdvPhy, uint16_t Interval)
{
	(void)Status; (void)SyncHdl; (void)AdvSid; (void)AdvAddrType;
	(void)AdvAddr; (void)AdvPhy; (void)Interval;
}

__attribute__((weak)) void BtAdvPeriodicSyncReport(uint16_t SyncHdl,
	int8_t TxPower, int8_t Rssi, size_t Len, const uint8_t *pData)
{
	(void)SyncHdl; (void)TxPower; (void)Rssi; (void)Len; (void)pData;
}

__attribute__((weak)) void BtAdvPeriodicSyncLost(uint16_t SyncHdl)
{
	(void)SyncHdl;
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
