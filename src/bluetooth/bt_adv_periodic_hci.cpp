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

#pragma pack(pop)

static_assert(sizeof(BtAdvPeriodicExtParams_t) == 25, "extended advertising parameters must be 25 octets");
static_assert(sizeof(BtAdvPeriodicParams_t) == 7, "periodic advertising parameters must be 7 octets");
static_assert(sizeof(BtAdvPeriodicEnable_t) == 2, "periodic advertising enable must be 2 octets");
static_assert(sizeof(BtAdvPeriodicExtEnable_t) == 6, "extended advertising enable must be 6 octets");

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

bool BtAdvPeriodicInit(BtHciDevice_t * const pDev, const BtAdvPeriodicCfg_t *pCfg)
{
	s_pBtAdvPeriodicDev = nullptr;
	s_BtAdvPeriodicReady = false;
	s_BtAdvPeriodicRunning = false;
	s_BtAdvPeriodicDataLen = 0;

	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	// 7.8.61 puts both interval parameters in 1.25 ms units with a range of
	// 0x0006 to 0xFFFF, and requires min to be no greater than max.
	if (pCfg->IntervalMin < BT_ADV_PERIODIC_INTERVAL_MIN ||
		pCfg->IntervalMax < BT_ADV_PERIODIC_INTERVAL_MIN ||
		pCfg->IntervalMin > pCfg->IntervalMax)
	{
		return false;
	}

	const BtHciCapabilities_t *pCap = BtHciCapabilitiesForDeviceGet(pDev);

	// The train needs its own extended advertising set, so a controller that
	// reports only one cannot run this alongside connectable advertising.
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

	if ((pCap->Valid & BT_HCI_CAP_VALID_ADV_SET_COUNT) != 0 &&
		pCap->AdvSetCount <= BT_ADV_PERIODIC_ADV_HANDLE)
	{
		return false;
	}

	// Create the set the train rides on. Non connectable and non scannable is
	// forced by 7.8.61; the primary interval only paces the extended
	// advertising events that point at the train, so it follows the periodic
	// interval rather than being configured separately.
	BtAdvPeriodicExtParams_t ext;
	memset(&ext, 0, sizeof(ext));
	ext.AdvHandle = BT_ADV_PERIODIC_ADV_HANDLE;
	BtAdvPeriodicWr16(ext.EvtProp, BT_ADV_PERIODIC_EXT_EVT_PROP);
	BtAdvPeriodicWr24(ext.PrimIntervalMin, pCfg->IntervalMin);
	BtAdvPeriodicWr24(ext.PrimIntervalMax, pCfg->IntervalMax);
	ext.PrimChanMap = 7;
	ext.OwnAddrType = pCfg->OwnAddrType;
	ext.FilterPolicy = 0;
	ext.TxPower = 0;
	ext.PrimPhy = BTADV_EXTADV_PHY_1M;
	ext.SecMaxSkip = 0;
	ext.SecPhy = BtHciCapabilitiesLeFeatureSupported(pCap,
		BT_HCI_CAP_LE_FEATURE_PHY_2M) ?
		BTADV_EXTADV_PHY_2M : BTADV_EXTADV_PHY_1M;
	ext.Sid = pCfg->Sid;
	ext.ScanReqNotif = 0;

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM,
		&ext, sizeof(ext), nullptr, 0) != 0)
	{
		return false;
	}

	BtAdvPeriodicParams_t p;
	memset(&p, 0, sizeof(p));
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
