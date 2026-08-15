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
#include "bluetooth/bt_smp.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the app configured (UART, USB, RTT, BLE, or any other
// DeviceIntrf); the trace does not assume a transport. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
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

typedef struct {
	uint8_t  ConnHdl[2];
} BtHciLeReadPhy_t;				//!< 2 octets, 7.8.47

typedef struct {
	uint8_t  ConnHdl[2];
	uint8_t  TxPhy;
	uint8_t  RxPhy;
} BtHciLeReadPhyRet_t;			//!< 4 octets of return parameters, 7.8.47

typedef struct {
	uint8_t  ConnHdl[2];
	uint8_t  AllPhys;
	uint8_t  TxPhys;
	uint8_t  RxPhys;
	uint8_t  PhyOptions[2];
} BtHciLeSetPhy_t;				//!< 7 octets, 7.8.49

typedef struct {
	uint8_t  ConnHdl[2];
	uint8_t  TxOctets[2];
	uint8_t  TxTime[2];
} BtHciLeSetDataLen_t;			//!< 6 octets, 7.8.33

#pragma pack(pop)

// ALL_PHYS of LE Set PHY, Core Vol 4 Part E 7.8.49. A set bit says the host
// has no preference in that direction, and the matching PHY mask is then
// ignored.
#define BT_GAP_ALL_PHYS_TX_ANY		(1<<0)
#define BT_GAP_ALL_PHYS_RX_ANY		(1<<1)

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

// Clamp the HCI-encoded LE scan interval and window to the valid range
// 0x0004..0x4000 (Core Vol 4 Part E 7.8.64 scan, 7.8.66 create connection) and
// keep the window no larger than the interval. A small or misconfigured value
// would otherwise reach the controller as Invalid HCI Command Parameters.
static inline void BtGapClampScanParams(uint16_t *pInterval, uint16_t *pWindow)
{
	if (*pInterval < 0x0004)
	{
		*pInterval = 0x0004;
	}
	else if (*pInterval > 0x4000)
	{
		*pInterval = 0x4000;
	}

	if (*pWindow < 0x0004)
	{
		*pWindow = 0x0004;
	}
	else if (*pWindow > 0x4000)
	{
		*pWindow = 0x4000;
	}

	if (*pWindow > *pInterval)
	{
		*pWindow = *pInterval;
	}
}

// Resolve the Own_Address_Type for scanning or initiating from the configured
// local identity. A public identity uses Public and needs no controller random
// address. A random identity uses Random and requires the global controller
// random address to be programmed first (LE Set Random Address, Core Vol 4
// Part E 7.8.4); scanning or initiating as Random with an unset (all-zero)
// controller random address yields an undefined on-air address or Command
// Disallowed. Returns the HCI Own_Address_Type, or 0xFF if the configured
// random address is not a valid static random address (Vol 6 Part B 1.3.2.1).
static uint8_t BtGapResolveOwnAddr(BtHciDevice_t *pDev)
{
	uint8_t localType = 0;
	uint8_t localAddr[6];
	BtSmpLocalAddrGet(&localType, localAddr);

	if (localType != BTADDR_TYPE_RAND && localType != BTADDR_TYPE_RANDOM_STATIC)
	{
		return BTADDR_TYPE_PUBLIC;
	}

	if (BtAddrIsStaticRandom(localAddr) == false)
	{
		return 0xFF;
	}

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_RANDOM_ADDR, localAddr, 6, NULL, 0) != 0)
	{
		return 0xFF;
	}

	return BTADDR_TYPE_RAND;
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
	if (pCfg == nullptr)
	{
		return false;
	}

	// This command layout contains one Scan_Type/Interval/Window block. HCI
	// requires one such block for every selected scanning PHY. Support either
	// 1M or Coded here, and reject zero, 2M (not a scanning PHY), or a combined
	// mask instead of sending a short parameter list for the advertised mask.
	if (pCfg->Param.Phy != BT_GAP_PHY_1MBITS &&
		pCfg->Param.Phy != BT_GAP_PHY_CODED)
	{
		return false;
	}

	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	memcpy(&s_ScanParams, &pCfg->Param, sizeof(BtGapScanParam_t));

	BtHciLeExtScanParams1_t p;
	uint8_t ownAddrType = BtGapResolveOwnAddr(pDev);
	if (ownAddrType == 0xFF)
	{
		return false;
	}
	p.OwnAddrType  = ownAddrType;
	p.FilterPolicy = 0;
	p.ScanPhys     = pCfg->Param.Phy;
	// Map the API scan type to the HCI Scan_Type field (Core Vol 4 Part E
	// 7.8.64): 0x00 = passive, 0x01 = active. Only BTSCAN_TYPE_ACTIVE requests
	// active scanning; the passive variants map to passive. Assigning the enum
	// directly would send ACTIVE (2) as an invalid value and PASSIVE_EXT (1) as
	// active.
	p.ScanType     = (pCfg->Type == BTSCAN_TYPE_ACTIVE) ? 1 : 0;
	uint16_t scanInterval = mSecTo0_625(pCfg->Param.Interval);
	uint16_t scanWindow   = mSecTo0_625(pCfg->Param.Duration);
	BtGapClampScanParams(&scanInterval, &scanWindow);
	BtGapWr16(p.ScanInterval, scanInterval);
	BtGapWr16(p.ScanWindow, scanWindow);

	uint8_t res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM, &p, sizeof(p), NULL, 0);
	DEBUG_PRINTF("SET_EXT_SCAN_PARAM: res=%d\r\n", res);

	return res == 0;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	(void)pBuff;
	(void)Len;

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
	(void)pBuff;
	(void)Len;

	return true;
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)
{
	if (pPeerAddr == nullptr || pConnParam == nullptr)
	{
		return false;
	}

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
	uint16_t scanInterval = mSecTo0_625(s_ScanParams.Interval);
	uint16_t scanWindow   = mSecTo0_625(s_ScanParams.Duration);
	BtGapClampScanParams(&scanInterval, &scanWindow);
	BtGapWr16(p.ScanInterval, scanInterval);
	BtGapWr16(p.ScanWindow, scanWindow);
	p.FilterPolicy  = 0;
	p.PeerAddrType  = pPeerAddr->Type;
	memcpy(p.PeerAddr, pPeerAddr->Addr, 6);
	uint8_t ownAddrType = BtGapResolveOwnAddr(pDev);
	if (ownAddrType == 0xFF)
	{
		return false;
	}
	p.OwnAddrType   = ownAddrType;
	BtGapWr16(p.ConnIntervalMin, connIntervalMin);
	BtGapWr16(p.ConnIntervalMax, connIntervalMax);
	BtGapWr16(p.MaxLatency, pConnParam->Latency);
	BtGapWr16(p.SupervisionTimeout, supTimeout);
	BtGapWr16(p.MinCeLength, 0);
	BtGapWr16(p.MaxCeLength, 0);

	uint8_t res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_CREATE_CONN, &p, sizeof(p), NULL, 0);

	return res == 0;
}

// --- Link procedures on an established connection ---

bool BtGapReadPhy(uint16_t ConnHdl, uint8_t *pTxPhy, uint8_t *pRxPhy)
{
	if (pTxPhy == nullptr || pRxPhy == nullptr)
	{
		return false;
	}

	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	BtHciLeReadPhy_t p;
	BtGapWr16(p.ConnHdl, ConnHdl);

	BtHciLeReadPhyRet_t r;
	memset(&r, 0, sizeof(r));

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_READ_PHY, &p, sizeof(p),
					 &r, sizeof(r)) != 0)
	{
		return false;
	}

	// TX_PHY and RX_PHY are enumerated 1, 2, 3 for 1M, 2M and coded (7.8.47),
	// where the API uses the bit mask LE Set PHY takes. Converting here keeps
	// one PHY encoding in the API rather than one per command.
	if (r.TxPhy < 1 || r.TxPhy > 3 || r.RxPhy < 1 || r.RxPhy > 3)
	{
		return false;
	}

	*pTxPhy = (uint8_t)(1 << (r.TxPhy - 1));
	*pRxPhy = (uint8_t)(1 << (r.RxPhy - 1));

	return true;
}

bool BtGapSetPhy(uint16_t ConnHdl, uint8_t TxPhys, uint8_t RxPhys,
				 uint16_t PhyOptions)
{
	if ((TxPhys & ~BT_GAP_PHY_ALL) != 0 || (RxPhys & ~BT_GAP_PHY_ALL) != 0 ||
		PhyOptions > BT_GAP_PHY_OPT_MAX)
	{
		return false;
	}

	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	BtHciLeSetPhy_t p;
	BtGapWr16(p.ConnHdl, ConnHdl);

	// An empty mask is no preference, which 7.8.49 spells in ALL_PHYS. Sending
	// it in the mask instead would be a request for no PHY at all, which the
	// controller answers with Invalid HCI Command Parameters.
	p.AllPhys = 0;
	if (TxPhys == 0)
	{
		p.AllPhys |= BT_GAP_ALL_PHYS_TX_ANY;
	}
	if (RxPhys == 0)
	{
		p.AllPhys |= BT_GAP_ALL_PHYS_RX_ANY;
	}

	p.TxPhys = TxPhys;
	p.RxPhys = RxPhys;
	BtGapWr16(p.PhyOptions, PhyOptions);

	uint8_t res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PHY, &p, sizeof(p),
							   NULL, 0);
	DEBUG_PRINTF("SET_PHY: res=%d\r\n", res);

	return res == 0;
}

bool BtGapSetDataLength(uint16_t ConnHdl, uint16_t TxOctets, uint16_t TxTime)
{
	if (TxOctets < BT_GAP_DATA_LEN_OCTETS_MIN ||
		TxOctets > BT_GAP_DATA_LEN_OCTETS_MAX ||
		TxTime < BT_GAP_DATA_LEN_TIME_MIN ||
		TxTime > BT_GAP_DATA_LEN_TIME_MAX)
	{
		return false;
	}

	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	BtHciLeSetDataLen_t p;
	BtGapWr16(p.ConnHdl, ConnHdl);
	BtGapWr16(p.TxOctets, TxOctets);
	BtGapWr16(p.TxTime, TxTime);

	uint8_t res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_DATA_LEN, &p,
							   sizeof(p), NULL, 0);
	DEBUG_PRINTF("SET_DATA_LEN: res=%d\r\n", res);

	return res == 0;
}
