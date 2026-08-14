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
#include "bluetooth/bt_hci_cap.h"
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
	uint8_t  ScanType;
	uint8_t  ScanInterval[2];
	uint8_t  ScanWindow[2];
	uint8_t  OwnAddrType;
	uint8_t  FilterPolicy;
} BtHciLeScanParams_t;			//!< 7 octets

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
} BtHciLeScanEnable_t;			//!< 2 octets

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
	uint8_t  FilterPolicy;
	uint8_t  OwnAddrType;
	uint8_t  PeerAddrType;
	uint8_t  PeerAddr[6];
	uint8_t  InitiatingPhys;		//!< Initiating_PHYs bitmask, one PHY here
	uint8_t  ScanInterval[2];
	uint8_t  ScanWindow[2];
	uint8_t  ConnIntervalMin[2];
	uint8_t  ConnIntervalMax[2];
	uint8_t  MaxLatency[2];
	uint8_t  SupervisionTimeout[2];
	uint8_t  MinCeLength[2];
	uint8_t  MaxCeLength[2];
} BtHciLeExtCreateConn1_t;		//!< 26 octets, single PHY

#pragma pack(pop)

BtGapScanParam_t s_ScanParams;

static bool s_BtGapUseExtScanCmd = true;
static uint8_t s_BtGapScanPhy = BT_GAP_PHY_1MBITS;
static BtHciCapabilities_t s_BtGapReadCapabilities;

static inline void BtGapWr16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline BtHciDevice_t *BtGapHciDev(void)
{
	return g_BtAppData.AppDevice.pHciDev;
}

static const BtHciCapabilities_t *BtGapCapabilitiesGet(BtHciDevice_t *pDev)
{
	const BtHciCapabilities_t *pCapabilities =
		BtHciCapabilitiesForDeviceGet(pDev);
	if (pCapabilities != nullptr && pCapabilities->Valid != 0)
	{
		return pCapabilities;
	}

	if (BtHciCapabilitiesRead(pDev, &s_BtGapReadCapabilities) == false)
	{
		return nullptr;
	}

	return &s_BtGapReadCapabilities;
}

// Clamp the HCI-encoded LE scan interval and window to the valid range
// 0x0004..0x4000 (Core Vol 4 Part E 7.8.64 scan, 7.8.66 create connection) and
// keep the window no larger than the interval. A small or misconfigured value
// would otherwise reach the controller as Invalid HCI Command Parameters.
// Core Vol 4 Part E 7.8.10 allows 0x0004 to 0x4000 for both the scan interval
// and the scan window, 2.5 ms to 10.24 s, and requires the window to be no
// larger than the interval.
//
// Takes the converted values in 32 bits so an over-range request clamps down
// to 0x4000. While the conversion truncated first, 40960 ms wrapped to 0 and
// then clamped up to 0x0004, so a request for a 41 second duty cycle became
// continuous scanning at full power, the opposite of what was asked.
static inline void BtGapClampScanParams(uint32_t *pInterval, uint32_t *pWindow)
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
// Part E 7.8.4). Returns 0xFF when the configured random address is invalid or
// the controller cannot execute LE Set Random Address.
static uint8_t BtGapResolveOwnAddr(BtHciDevice_t *pDev,
	const BtHciCapabilities_t *pCapabilities)
{
	uint8_t localType = 0;
	uint8_t localAddr[6];
	BtSmpLocalAddrGet(&localType, localAddr);

	if (localType != BTADDR_TYPE_RAND && localType != BTADDR_TYPE_RANDOM_STATIC)
	{
		return BTADDR_TYPE_PUBLIC;
	}

	if (BtAddrIsStaticRandom(localAddr) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_SET_RANDOM_ADDRESS) == false)
	{
		return 0xFF;
	}

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_RANDOM_ADDR,
		localAddr, sizeof(localAddr), NULL, 0) != 0)
	{
		return 0xFF;
	}

	return BTADDR_TYPE_RAND;
}

// Validate the HCI-encoded connection parameters before LE Create Connection
// (Core Vol 4 Part E 7.8.12, Vol 6 Part B 4.5.2). Arguments are in HCI units:
// interval in 1.25 ms steps, timeout in 10 ms steps, latency a raw count.
// Rejecting out-of-range or self-inconsistent values here avoids a controller
// error and a link that supervision-times-out immediately.
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

	// LE Set Extended Scan Enable encodes Duration as uint16_t in 10 ms
	// units. BtGapScanParam_t.Timeout is seconds, so 655 s is the largest
	// value that can be represented without changing the requested timeout.
	if (pCfg->Param.Timeout > 655U)
	{
		return false;
	}

	// This implementation carries one Scan_Type/Interval/Window block. HCI
	// requires one block for every selected scanning PHY, so support one primary
	// advertising PHY at a time and reject 2M, zero, and combined masks.
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

	const BtHciCapabilities_t *pCapabilities = BtGapCapabilitiesGet(pDev);
	if (pCapabilities == nullptr)
	{
		return false;
	}

	bool useExtScan;
	if (pCfg->Param.Phy == BT_GAP_PHY_CODED)
	{
		if (BtHciCapabilitiesLeFeatureSupported(pCapabilities,
			BT_HCI_CAP_LE_FEATURE_CODED_PHY) == false ||
			BtHciCapabilitiesExtendedScanningSupported(pCapabilities) == false)
		{
			return false;
		}
		useExtScan = true;
	}
	else if (pCfg->Param.Timeout != 0)
	{
		// Legacy LE Set Scan Enable has no duration field. A finite timeout
		// therefore requires the extended scan command family.
		if (BtHciCapabilitiesExtendedScanningSupported(pCapabilities) == false)
		{
			return false;
		}
		useExtScan = true;
	}
	else if (BtHciCapabilitiesExtendedScanningSupported(pCapabilities))
	{
		useExtScan = true;
	}
	else if (BtHciCapabilitiesLegacyScanningSupported(pCapabilities))
	{
		useExtScan = false;
	}
	else
	{
		return false;
	}

	uint8_t ownAddrType = BtGapResolveOwnAddr(pDev, pCapabilities);
	if (ownAddrType == 0xFF)
	{
		return false;
	}

	uint32_t scanInterval = mSecTo0_625(pCfg->Param.Interval);
	uint32_t scanWindow   = mSecTo0_625(pCfg->Param.Duration);
	BtGapClampScanParams(&scanInterval, &scanWindow);
	uint8_t scanType = (pCfg->Type == BTSCAN_TYPE_ACTIVE) ? 1 : 0;
	uint8_t res;

	if (useExtScan)
	{
		BtHciLeExtScanParams1_t p;
		p.OwnAddrType  = ownAddrType;
		p.FilterPolicy = 0;
		p.ScanPhys     = pCfg->Param.Phy;
		p.ScanType     = scanType;
		BtGapWr16(p.ScanInterval, scanInterval);
		BtGapWr16(p.ScanWindow, scanWindow);

		res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM,
			&p, sizeof(p), NULL, 0);
		DEBUG_PRINTF("SET_EXT_SCAN_PARAM: res=%d\r\n", res);
	}
	else
	{
		BtHciLeScanParams_t p;
		p.ScanType = scanType;
		BtGapWr16(p.ScanInterval, scanInterval);
		BtGapWr16(p.ScanWindow, scanWindow);
		p.OwnAddrType = ownAddrType;
		p.FilterPolicy = 0;

		res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_SCAN_PARAM,
			&p, sizeof(p), NULL, 0);
		DEBUG_PRINTF("SET_SCAN_PARAM: res=%d\r\n", res);
	}

	if (res != 0)
	{
		return false;
	}

	memcpy(&s_ScanParams, &pCfg->Param, sizeof(BtGapScanParam_t));
	s_BtGapUseExtScanCmd = useExtScan;
	s_BtGapScanPhy = pCfg->Param.Phy;

	return true;
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

	uint8_t res;
	if (s_BtGapUseExtScanCmd)
	{
		BtHciLeExtScanEnable_t e;
		e.Enable = 1;
		e.FilterDup = 1;
		BtGapWr16(e.Duration, (uint16_t)(s_ScanParams.Timeout * 100U));
		BtGapWr16(e.Period, 0);

		res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE,
			&e, sizeof(e), NULL, 0);
		DEBUG_PRINTF("SET_EXT_SCAN_ENABLE: res=%d\r\n", res);
	}
	else
	{
		BtHciLeScanEnable_t e = { 1, 1 };
		res = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_SCAN_ENABLE,
			&e, sizeof(e), NULL, 0);
		DEBUG_PRINTF("SET_SCAN_ENABLE: res=%d\r\n", res);
	}

	return res == 0;
}

void BtGapScanStop()
{
	BtHciDevice_t *pDev = BtGapHciDev();
	if (pDev == nullptr)
	{
		return;
	}

	if (s_BtGapUseExtScanCmd)
	{
		BtHciLeExtScanEnable_t e = {};
		BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE,
			&e, sizeof(e), NULL, 0);
	}
	else
	{
		BtHciLeScanEnable_t e = {};
		BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_SCAN_ENABLE,
			&e, sizeof(e), NULL, 0);
	}
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

	const BtHciCapabilities_t *pCapabilities = BtGapCapabilitiesGet(pDev);
	if (pCapabilities == nullptr)
	{
		return false;
	}

	uint8_t initiatingPhy = s_BtGapScanPhy;
	bool useExtCreate;
	if (initiatingPhy == BT_GAP_PHY_CODED)
	{
		if (BtHciCapabilitiesLeFeatureSupported(pCapabilities,
			BT_HCI_CAP_LE_FEATURE_CODED_PHY) == false ||
			BtHciCapabilitiesExtendedInitiatingSupported(pCapabilities) == false)
		{
			return false;
		}
		useExtCreate = true;
	}
	else if (initiatingPhy == BT_GAP_PHY_1MBITS)
	{
		if (BtHciCapabilitiesExtendedInitiatingSupported(pCapabilities))
		{
			useExtCreate = true;
		}
		else if (BtHciCapabilitiesLegacyInitiatingSupported(pCapabilities))
		{
			useExtCreate = false;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	uint32_t scanInterval = mSecTo0_625(s_ScanParams.Interval);
	uint32_t scanWindow   = mSecTo0_625(s_ScanParams.Duration);
	BtGapClampScanParams(&scanInterval, &scanWindow);

	uint8_t ownAddrType = BtGapResolveOwnAddr(pDev, pCapabilities);
	if (ownAddrType == 0xFF)
	{
		return false;
	}

	if (useExtCreate)
	{
		BtHciLeExtCreateConn1_t p = {};
		p.FilterPolicy = 0;
		p.OwnAddrType = ownAddrType;
		p.PeerAddrType = pPeerAddr->Type;
		memcpy(p.PeerAddr, pPeerAddr->Addr, 6);
		p.InitiatingPhys = initiatingPhy;
		BtGapWr16(p.ScanInterval, scanInterval);
		BtGapWr16(p.ScanWindow, scanWindow);
		BtGapWr16(p.ConnIntervalMin, connIntervalMin);
		BtGapWr16(p.ConnIntervalMax, connIntervalMax);
		BtGapWr16(p.MaxLatency, pConnParam->Latency);
		BtGapWr16(p.SupervisionTimeout, supTimeout);
		BtGapWr16(p.MinCeLength, 0);
		BtGapWr16(p.MaxCeLength, 0);

		return BtHciCommand(pDev, BT_HCI_CMD_CTLR_EXT_CREATE_CONN,
			&p, sizeof(p), NULL, 0) == 0;
	}

	BtHciLeCreateConn_t p = {};
	BtGapWr16(p.ScanInterval, scanInterval);
	BtGapWr16(p.ScanWindow, scanWindow);
	p.FilterPolicy  = 0;
	p.PeerAddrType  = pPeerAddr->Type;
	memcpy(p.PeerAddr, pPeerAddr->Addr, 6);
	p.OwnAddrType   = ownAddrType;
	BtGapWr16(p.ConnIntervalMin, connIntervalMin);
	BtGapWr16(p.ConnIntervalMax, connIntervalMax);
	BtGapWr16(p.MaxLatency, pConnParam->Latency);
	BtGapWr16(p.SupervisionTimeout, supTimeout);
	BtGapWr16(p.MinCeLength, 0);
	BtGapWr16(p.MaxCeLength, 0);

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_CREATE_CONN,
		&p, sizeof(p), NULL, 0) == 0;
}
