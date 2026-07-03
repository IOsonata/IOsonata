/**-------------------------------------------------------------------------
@file	bt_gap_hci.cpp

@brief	Bluetooth Generic Access Profile (GAP) over standard HCI.

        Scan and connection setup driven through standard HCI commands sent
        with BtHciCommand, so it works with any HCI controller. No vendor
        headers: only opcodes and packed HCI parameter layouts.

Core Bluetooth Vol.1, Part A, 6.2

@author	Hoang Nguyen Hoan
@date	Jan. 20, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

----------------------------------------------------------------------------*/
#include <memory.h>

#include "istddef.h"
#include "convutil.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_gap.h"

// Debug printf via SysLog. Enable to trace command status.
#if 0
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

// --- Packed standard HCI command parameter layouts (Core Vol 4 Part E) ---

#pragma pack(push, 1)

typedef struct {
	uint8_t  OwnAddrType;
	uint8_t  FilterPolicy;
	uint8_t  ScanPhys;			//!< Scanning_PHYs bitmask, one PHY here
	uint8_t  ScanType;
	uint8_t  ScanInterval[2];
	uint8_t  ScanWindow[2];
} BtHciLeExtScanParams1_t;		//!< 8 octets, single PHY

typedef struct {
	uint8_t  Enable;
	uint8_t  FilterDup;
	uint8_t  Duration[2];
	uint8_t  Period[2];
} BtHciLeExtScanEnable_t;		//!< 6 octets

typedef struct {
	uint8_t  ScanInterval[2];
	uint8_t  ScanWindow[2];
	uint8_t  FilterPolicy;
	uint8_t  PeerAddrType;
	uint8_t  PeerAddr[6];
	uint8_t  OwnAddrType;
	uint8_t  ConnIntervalMin[2];
	uint8_t  ConnIntervalMax[2];
	uint8_t  MaxLatency[2];
	uint8_t  SupervisionTimeout[2];
	uint8_t  MinCeLength[2];
	uint8_t  MaxCeLength[2];
} BtHciLeCreateConn_t;			//!< 25 octets

#pragma pack(pop)

BtGapScanParam_t s_ScanParams;

static inline void BtGapWr16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline BtHciDevice_t *BtGapHciDev(void)
{
	return g_BtAppData.AppDevice.pHciDev;
}

// Validate the HCI-encoded connection parameters before LE Create Connection
// (Core Vol 4 Part E 7.8.12, Vol 6 Part B 4.5.2). Arguments are in HCI units:
// interval in 1.25 ms steps, timeout in 10 ms steps, latency a raw count.
// Rejecting out-of-range or self-inconsistent values here avoids a controller
// error and, worse, a link that supervision-times-out immediately.
static bool BtGapCreateConnParamValid(uint16_t IntervalMin, uint16_t IntervalMax,
									  uint16_t Latency, uint16_t Timeout)
{
	if (IntervalMin < 0x0006 || IntervalMax > 0x0C80 || IntervalMin > IntervalMax)
	{
		return false;
	}
	if (Latency > 0x01F3)
	{
		return false;
	}
	if (Timeout < 0x000A || Timeout > 0x0C80)
	{
		return false;
	}
	// Supervision_Timeout (ms) must exceed 2 * (1 + Latency) * ConnIntervalMax
	// (ms). In HCI units (Timeout x 10 ms, IntervalMax x 1.25 ms) this reduces
	// to the integer test Timeout * 4 > (1 + Latency) * IntervalMax.
	if ((uint32_t)Timeout * 4 <= (uint32_t)(1 + Latency) * IntervalMax)
	{
		return false;
	}
	return true;
}

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	memcpy(&s_ScanParams, &pCfg->Param, sizeof(BtGapScanParam_t));

	BtHciLeExtScanParams1_t p;
	p.OwnAddrType  = 1;
	p.FilterPolicy = 0;
	p.ScanPhys     = pCfg->Param.Phy;
	// Map the API scan type to the HCI Scan_Type field (Core Vol 4 Part E
	// 7.8.64): 0x00 = passive, 0x01 = active. Only BTSCAN_TYPE_ACTIVE requests
	// active scanning; the passive variants map to passive. Assigning the enum
	// directly would send ACTIVE (2) as an invalid value and PASSIVE_EXT (1) as
	// active.
	p.ScanType     = (pCfg->Type == BTSCAN_TYPE_ACTIVE) ? 1 : 0;
	BtGapWr16(p.ScanInterval, mSecTo0_625(pCfg->Param.Interval));
	BtGapWr16(p.ScanWindow, mSecTo0_625(pCfg->Param.Duration));

	uint8_t res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM, &p, sizeof(p), NULL, 0);
	DEBUG_PRINTF("SET_EXT_SCAN_PARAM: res=%d\r\n", res);

	return res == 0;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	BtHciLeExtScanEnable_t e;
	e.Enable = 1;
	e.FilterDup = 1;
	BtGapWr16(e.Duration, 0);
	BtGapWr16(e.Period, 0);

	uint8_t res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE, &e, sizeof(e), NULL, 0);
	DEBUG_PRINTF("SET_EXT_SCAN_ENABLE: res=%d\r\n", res);

	return res == 0;
}

void BtGapScanStop()
{
	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return;
	}

	BtHciLeExtScanEnable_t e;
	memset(&e, 0, sizeof(e));

	BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE, &e, sizeof(e), NULL, 0);
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
	return true;
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)
{
	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	uint16_t connIntervalMin = mSecTo1_25(pConnParam->IntervalMin);
	uint16_t connIntervalMax = mSecTo1_25(pConnParam->IntervalMax);
	uint16_t supTimeout      = (uint16_t)(pConnParam->Timeout / 10);

	if (BtGapCreateConnParamValid(connIntervalMin, connIntervalMax,
								  pConnParam->Latency, supTimeout) == false)
	{
		return false;
	}

	BtHciLeCreateConn_t p;
	BtGapWr16(p.ScanInterval, mSecTo0_625(s_ScanParams.Interval));
	BtGapWr16(p.ScanWindow, mSecTo0_625(s_ScanParams.Duration));
	p.FilterPolicy  = 0;
	p.PeerAddrType  = pPeerAddr->Type;
	memcpy(p.PeerAddr, pPeerAddr->Addr, 6);
	p.OwnAddrType   = s_ScanParams.OwnAddrType;
	BtGapWr16(p.ConnIntervalMin, connIntervalMin);
	BtGapWr16(p.ConnIntervalMax, connIntervalMax);
	BtGapWr16(p.MaxLatency, pConnParam->Latency);
	BtGapWr16(p.SupervisionTimeout, supTimeout);
	BtGapWr16(p.MinCeLength, 0);
	BtGapWr16(p.MaxCeLength, 0);

	uint8_t res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_CREATE_CONN, &p, sizeof(p), NULL, 0);

	return res == 0;
}
