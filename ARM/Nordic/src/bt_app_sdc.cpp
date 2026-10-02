/**-------------------------------------------------------------------------
@file	bt_app_sdc.cpp

@brief	Bluetooth application creation helper using softdevice controller


@author	Hoang Nguyen Hoan
@date	Mar. 8, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <atomic>
#include <stdlib.h>

//#include "mpsl.h"
//#include "mpsl_fem_init.h"
#include "sdc.h"
#include "sdc_soc.h"
#include "sdc_hci.h"
#include "sdc_hci_vs.h"

#include "istddef.h"
#include "convutil.h"
#include "nrf_mac.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "timer_nrfx.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_smp.h"		// BtSmpLocalAddrGet override
#include "bluetooth/bt_adv.h"		// BtAdvOwnAddrGet for the connection stamp
#include "bluetooth/bt_ead.h"		// Encrypted Advertising Data engines

#include "crypto_rng_nrf.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/services/bt_dis.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "nrf_mpsl.h"
#include "iopinctrl.h"
#include "app_evt_handler.h"


#define BT_SDC_RX_MAX_PACKET_COUNT			2
#define BT_SDC_TX_MAX_PACKET_COUNT			3

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the a configured (UART, USB, RTT, BLE, or any other
// DeviceIntrf); the trace does not assume a transport. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

// Which persistence the build uses. Deliberately independent of DEBUG_ENABLE
// and of NDEBUG: the release build is where a bond has to survive a reset, and
// this one line says whether the store is even in the picture.
#define STORE_TRACE

#ifdef STORE_TRACE
#include "syslog.h"
#define STORE_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define STORE_PRINTF(...)
#endif

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

void BtAppEvtHandler(BtHciDevice_t * const pDev, uint32_t Evt);
void BtAppConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6]);
void BtAppDisconnected(uint16_t ConnHdl, uint8_t Reason);
void BtAppSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent);
bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *DavData);

static void BtAppSdcTimerHandler(TimerDev_t * const pTimer, uint32_t Evt);
static inline uint32_t BtAppSendData(void *pData, uint32_t Len);

static BtHciDevice_t s_BtHciDev = {
	.pCtx = (void*)&g_BtAppData,
	.SendData = BtAppSendData,
	.Command = BtHciCmdSdc,
	.EvtHandler = BtAppEvtHandler,
	// Connected, Disconnected and SendCompleted are set by BtAppConnInit
	.ScanReport = BtAppScanReport,
	.AdvTimeout = BtAppAdvTimeoutHandler,
	//.DiscoverDevice = BtAppDiscoverDevice,
};

// SDC controller instance. The HCI pump and transport live in
// bt_hci_ctlr_sdc; this app wires the receive handler to the host.
static BtHciCtlrDev_t s_BtHciCtlr;

// Connection support. What a link needs (peer table, attribute database, GAP
// and GATT services, ACL data path, GATT timeout) is reached only through
// this table. BtAppConnInit installs it. An application that only advertises
// or scans never reaches BtAppConnInit, so none of it is linked.
typedef struct {
	void (*AclData)(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pPkt);
	void (*Tick)(void);
	bool (*SrvcDone)(const BtAppCfg_t *pCfg);
	void (*TimerStart)(void);
} BtAppSdcConn_t;

static const BtAppSdcConn_t *s_pBtAppSdcConn = nullptr;

// Configuration given to BtAppInit, used by BtAppConnInit
static const BtAppCfg_t *s_pBtAppCfg = nullptr;

// Set at the end of BtAppInit. Connection support started after that point
// starts the timer itself.
static bool s_bBtAppInitDone = false;


// BtAppData_t now declared in bluetooth/bt_app.h, accessed via g_BtAppData.
// SDC port has no SDK-specific state in BtAppData_t scope; everything that
// remains here is already in the cross-arch struct.

// g_BtAppData definition and helpers (isConnected, BtAppConnLedOff/On) moved to
// src/bluetooth/bt_app.cpp.

// On-air local address SMP needs for c1/f5/f6 (set when we configure the
// random static address below). Defaults to public/zero until then.
static uint8_t s_BtSmpLocalAddr[6] = {0};
static uint8_t s_BtSmpLocalAddrType = 0;



/**@brief Bluetooth SIG debug mode Private Key */
__ALIGN(4) __WEAK extern const uint8_t g_lesc_private_key[32] = {
    0xbd,0x1a,0x3c,0xcd,0xa6,0xb8,0x99,0x58,0x99,0xb7,0x40,0xeb,0x7b,0x60,0xff,0x4a,
    0x50,0x3f,0x10,0xd2,0xe3,0xb3,0xc9,0x74,0x38,0x5f,0xc5,0xa3,0xd4,0xf6,0x49,0x3f,
};



const static TimerCfg_t s_BtAppSdcTimerCfg = {
    .DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,
	.EvtHandler = BtAppSdcTimerHandler
};

// Application timer: millisecond clock of the SMP and GATT transaction
// timeouts and their 1 s check. Only a link uses it. It is the timer device
// and not the Timer class: the class initializes through TimerInit, which
// links the high frequency timer driver along with the low frequency one.
static TimerDev_t s_BtAppSdcTimer;

static inline uint32_t BtAppSendData(void *pData, uint32_t Len) {
	return (uint32_t)BtHciCtlrSdcSend(pData, Len);
}

// Override the weak SMP accessor so the toolbox uses the device's configured
// address. Declared in bt_smp.h, so no linkage specifier is needed here.
void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	*pType = s_BtSmpLocalAddrType;
	memcpy(pAddr, s_BtSmpLocalAddr, 6);
}

// Route each HCI packet the controller drains to the host process entry.
static void BtAppSdcCtlrRx(BtHciCtlrDev_t * const pDev, bool bIsEvent, uint8_t *pPacket)
{
	if (bIsEvent)
	{
		BtHciProcessEvent(&s_BtHciDev, (BtHciEvtPacket_t*)pPacket);
	}
	else if (s_pBtAppSdcConn != nullptr)
	{
		s_pBtAppSdcConn->AclData(&s_BtHciDev, (BtHciACLDataPacket_t*)pPacket);
	}
}

// Millisecond clock for the generic SMP/GATT transaction timeouts. These
// override the weak BtSmpMsTick/BtGattMsTick defaults (which return 0, leaving
// the timeouts inert). Both are declared in bt_smp.h / bt_gatt.h, so no
// linkage specifier is needed here. The timer is started with connection support.
uint32_t BtSmpMsTick(void)
{
	if (s_BtAppSdcTimer.GetTickCount == nullptr)
	{
		// Timer not started
		return 0;
	}

	return (uint32_t)(s_BtAppSdcTimer.GetTickCount(&s_BtAppSdcTimer) * s_BtAppSdcTimer.nsPeriod / 1000000ULL);
}

uint32_t BtGattMsTick(void)
{
	return BtSmpMsTick();
}

// Spec-strict indication transaction timeout: Core Vol 3 Part F 3.3.3 requires
// closing the bearer, so disconnect the link. The generic weak default only
// clears the outstanding-indication flag.
void BtGattIndicationTimeout(uint16_t ConnHdl)
{
	uint8_t param[3];
	param[0] = (uint8_t)(ConnHdl & 0xFF);
	param[1] = (uint8_t)(ConnHdl >> 8);
	param[2] = 0x13;	// Remote User Terminated Connection

	BtHciCommand(&s_BtHciDev, BT_HCI_CMD_LINKCTRL_DISCONNECT, param, sizeof(param), NULL, 0);
}

#if 0
static void BtStackMpslAssert(const char * const file, const uint32_t line)
{
	DEBUG_PRINTF("MPSL Fault: %s, %d\n", file, line);
	while(1);
}
#endif




static void BtAppSdcTimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
        // Drive the generic indication transaction timeout (Core Vol 3
        // Part F 3.3.3). Cheap no-op when nothing is pending.
		if (s_pBtAppSdcConn != nullptr)
		{
			s_pBtAppSdcConn->Tick();
		}

        // Wake the main loop once per period. The pairing timeout (Core
        // Vol 3 Part H 3.4) is checked there by the security module, as an
        // idle handler, when the application uses security.
        BtAppEvtNotify();
    }
}

void BtAppSetDevName(const char *pName)
{
	BtGapSetDevName(pName);
}
/*
char *BleAppGetDevName()
{
	//return s_BtGapCharDevName;
}

*/

void BtAppEvtHandler(BtHciDevice_t * const pDev, uint32_t Evt)
{

}

void BtAppConnected(uint16_t ConnHdl, uint8_t Role, uint8_t PeerAddrType, uint8_t PeerAddr[6])
{
	// The own address the peer saw when it made this link. Peripheral role
	// means the peer connected to our advertising, so the address is whatever
	// the advertising set was programmed with; as the central the initiator
	// address is the device's configured one. The SMP toolbox computes
	// f5/f6/c1 with the stamped value.
	uint8_t ownType = 0;
	uint8_t ownAddr[6];
	if (Role == BT_CONN_ROLE_PERIPHERAL)
	{
		BtAdvOwnAddrGet(&ownType, ownAddr);
	}
	else
	{
		BtSmpLocalAddrGet(&ownType, ownAddr);
	}

	// Allocate and populate the peer record in one step. BtPeerConnected
	// allocs (or reuses) the slot for ConnHdl and fills Role/PeerAddr.
	// The state machine in bt_attrsp.cpp looks the peer up by ConnHdl.
	BtDevice_t *pPeer = BtPeerConnected(ConnHdl, Role, PeerAddrType, PeerAddr,
										ownType, ownAddr);
	if (pPeer != NULL)
	{
		pPeer->pHciDev = (BtHciDevice_t*) &s_BtHciDev;
		s_BtHciDev.pBtDev = (void*) pPeer;
	}

	// Defer MTU exchange until after encryption/service discovery.
	// BtAttExchangeMtuRequest(&s_BtHciDev, ConnHdl, BtAttGetMtu());

	// Discovery is a per-link decision, so it is gated on this link's role
	// rather than on the device's configured role bitmask.
	if (Role == BT_CONN_ROLE_CENTRAL)
	{
		// TODO: obtain the connected peripheral device's name and store to pPeer->Name;
		//BtAppDiscoverDevice(&s_BtHciDev, ConnHdl);
	}

	// If a secure SecType was configured, the link is secured by the security
	// module (bt_sec_sdc.cpp), which hooks the connection callback when the
	// application starts it with BtAppSecInit.

	BtAppEvtConnected(ConnHdl);
}

bool BtAppDiscoverDevice(BtDevice_t * const pDev)
{
	DEBUG_PRINTF("Start discovering device\r\n");

	// Reset counter and Service list
	pDev->NbSrvc = 0;
	if (BtDeviceSrvcCacheAttach(pDev) == false)
	{
		// No free discovery cache, see g_BtDevSrvcCacheCfg
		return false;
	}
	memset(pDev->pServices, 0, sizeof(BtGattDBSrvc_t) * BT_DEV_SERVICE_MAXCNT);

	// Start the discover process by discovering the Primary services
	BtUuid_t Uuid = {
			.BaseIdx = 0, // Standard bluetooth
			.Type = BT_UUID_TYPE_16,
			.Uuid16 = BT_UUID_DECLARATIONS_PRIMARY_SERVICE,
	};

	return BtAttStartReadByGroupTypeRequest(pDev->pHciDev, pDev->Conn.Hdl, 1, 0xFFFF, &Uuid);
}

void BtAppDisconnected(uint16_t ConnHdl, uint8_t Reason)
{
//	s_BtGapSrvc.ConnHdl = BT_GATT_HANDLE_INVALID;
//	s_BtGattSrvc.ConnHdl = BT_GATT_HANDLE_INVALID;

	DEBUG_PRINTF("BtAppDisconnected: ConnHdl= %d (0x%x); Reason = %d (0x%x)\r\n",
			ConnHdl, ConnHdl, Reason, Reason);

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	BtPeerFree(pPeer);

	bool bConnected = BtPeerIsConnected();

	if (bConnected == false)
	{
//		BtGattSrvcDisconnected(&s_BtGapSrvc);
//		BtGattSrvcDisconnected(&s_BtGattSrvc);

		g_BtAppData.State = BTAPP_STATE_IDLE;
	}

	BtAppEvtDisconnected(ConnHdl);

	if (bConnected == false &&
		(g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER)))
	{
		BtAdvStart();
	}
}

void BtAppSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent)
{
	BtGattSendCompleted(ConnHdl, NbPktSent);
}

void BtAppEnterDfu()
{
	/* TODO: implement */
}

void BtAppDisconnectConn(uint16_t ConnHdl)
{
	if (ConnHdl == BT_CONN_HDL_INVALID)
	{
		DEBUG_PRINTF("BtAppDisconnect: invalid handle\r\n");
		return;
	}

	// HCI Disconnect, Link Control OGF=0x01/OCF=0x0006.
	// Reason 0x13 = Remote User Terminated Connection.
	uint8_t param[3];
	param[0] = (uint8_t)(ConnHdl & 0xFF);
	param[1] = (uint8_t)(ConnHdl >> 8);
	param[2] = 0x13;

	uint8_t rc = BtHciCommand(&s_BtHciDev, BT_HCI_CMD_LINKCTRL_DISCONNECT, param, sizeof(param), NULL, 0);
	DEBUG_PRINTF("BtAppDisconnect: hdl=%u rc=%u\r\n", ConnHdl, rc);
}
/*
void BleAppGapDeviceNameSet(const char* pDeviceName)
{
	BleAdvPacket_t *advpkt;

	if (g_BleAppData.bExtAdv == true)
	{
		advpkt = &s_BleAppExtAdvPkt;
	}
	else
	{
		advpkt = &s_BleAppAdvPkt;
	}

	size_t l = strlen(pDeviceName);
	uint8_t type = BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME;

	if (l < 14)
	{
		// Short name
		type = BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME;
	}

	BleAdvDataAdd(advpkt, type, (uint8_t*)pDeviceName, l);

	BtGattCharSetValue(&s_BtGapChar[0], (void*)pDeviceName, l);
}
*/










bool BtAppStackInit(const BtAppCfg_t *pCfg)
{
	BtHciCtlrCfg_t ctlrcfg = { };
	ctlrcfg.RxHandler = BtAppSdcCtlrRx;
	ctlrcfg.OnWake = BtAppEvtNotify;
	ctlrcfg.Role = pCfg->Role;
	ctlrcfg.CentralDevMax = pCfg->CentralDevMax;
	ctlrcfg.PeriphDevMax = pCfg->PeriphDevMax;
	ctlrcfg.RxPktCount = BT_SDC_RX_MAX_PACKET_COUNT;
	ctlrcfg.TxPktCount = BT_SDC_TX_MAX_PACKET_COUNT;
	ctlrcfg.MaxDataLen = BTAPP_DEFAULT_MAX_DATA_LEN;
	// What the application asked for reaches the controller here. Without this
	// the counts stay zero, the controller reserves nothing, and the periodic
	// advertising commands are refused however well formed they are.
	ctlrcfg.PeriodicAdvCount = pCfg->PeriodicAdvCount;
	ctlrcfg.PeriodicSyncCount = pCfg->PeriodicSyncCount;
	ctlrcfg.PawrAdvCount = pCfg->PawrAdvCount;
	ctlrcfg.PawrSyncCount = pCfg->PawrSyncCount;

	if (BtHciCtlrEnable(&s_BtHciCtlr, &ctlrcfg) == false)
	{
		return false;
	}

	// The controller was configured with these ACL buffer parameters above.
	// Use the generic HCI host credit gate instead of the old SDC-local
	// s_SdcAclTxPktAvail counter.
	BtHciSetLeAclBuffer(&s_BtHciDev, ctlrcfg.MaxDataLen, ctlrcfg.TxPktCount);

	return true;
}

// Last step of the service setup of a peripheral, after the application has
// added its services: Device Information Service, appearance and preferred
// connection parameters.
static bool BtAppSdcSrvcDone(const BtAppCfg_t *pCfg)
{
	// Register Device Information Service when the app supplies device
	// info. Generic bt_dis adds it to the same ATT DB as user services.
	if (pCfg->pDevInfo != NULL && !BtDisInit(pCfg))
	{
		return false;
	}

	BtGapSetAppearance(pCfg->Appearance);

	BtGattPreferedConnParams_t connparm = {
		MSEC_TO_1_25(pCfg->ConnIntervalMin), MSEC_TO_1_25(pCfg->ConnIntervalMax),
		0, 400};
	BtGapSetPreferedConnParam(&connparm);

	return true;
}

// Start the app timer with a 1 s continuous trigger. It sources the SMP/GATT
// millisecond clock (BtSmp/GattMsTick above) and its handler drives the
// 30 s transaction-timeout checks. 1 s cadence is ample for a 30 s deadline.
static void BtAppSdcTimerStart(void)
{
	if (nRFxLFTimerInit(&s_BtAppSdcTimer, &s_BtAppSdcTimerCfg))
	{
		s_BtAppSdcTimer.EnableTrigger(&s_BtAppSdcTimer, 0, 1000000000ULL,
									  TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	}
}

static const BtAppSdcConn_t s_BtAppSdcConn = {
	.AclData = BtHciProcessData,
	.Tick = BtGattIndicationTimeoutCheck,
	.SrvcDone = BtAppSdcSrvcDone,
	.TimerStart = BtAppSdcTimerStart,
};

// Engines of Encrypted Advertising Data: the AES engine of the controller
// and the hardware RNG. Called by the EAD module the first time it needs an
// engine, so they are linked only when the application uses EAD
// (BtAdvEadKeySet, BtAdvDecrypt). Declared in bt_ead.h.
bool BtEadEngineInit(void)
{
	return BtEadInit(BtCryptoCtlrSdcInit(), CryptoRngNrfInstance());
}

static bool BtAppSdcConnStart(const BtAppCfg_t *pCfg)
{
	// Initialize the peer/connection table (and its long-write pool) before
	// the stack can produce any connection or data event.
	if (!BtPeerInit(pCfg->pPeerPoolMem, pCfg->PeerPoolMemSize))
	{
		return false;
	}

	if (pCfg->PeriphDevMax + pCfg->CentralDevMax > (int)BtPeerCount())
	{
		// Peer pool holds fewer slots than the number of links requested.
		// Provide a larger pool, see g_BtPeerPoolCfg in bt_peer.h
		return false;
	}
	BtPeerLongWrInit(pCfg->pLongWrPoolMem, pCfg->LongWrPoolMemSize);

	BtAttSetMtu(pCfg->MaxMtu);

	if (pCfg->AttDBMemSize > 0)
	{
		BtAttDBInit(pCfg->AttDBMemSize);
	}

	s_BtHciDev.Connected = BtAppConnected;
	s_BtHciDev.Disconnected = BtAppDisconnected;
	s_BtHciDev.SendCompleted = BtAppSendCompleted;

	// Commands that act on a link, in either role
	BtHciCtlrLinkSupport();

	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		// Has to be asked before the controller is enabled
		BtHciCtlrPeripheralSupport();
	}

	BtGapCfg_t gapcfg = {
		.Role = pCfg->Role,
		.SecType = pCfg->SecType,
		.AdvInterval = pCfg->AdvInterval,
		.AdvTimeout = pCfg->AdvTimeout,
		.ConnIntervalMin = pCfg->ConnIntervalMin,
		.ConnIntervalMax = pCfg->ConnIntervalMax,
		.SlaveLatency = 0,
		.SupTimeout = 400
	};

	DEBUG_PRINTF("BtGapInit\r\n");

	// The GAP and GATT services are the table a client reads first.
	if (BtGapInit(&gapcfg) == false)
	{
		return false;
	}

	BtGapSetDevName(pCfg->pDevName);

	return true;
}

/**
 * @brief	Start connection support.
 *
 * The call is what links the connection part of the stack. The stack calls
 * it when the first GATT service is added and when a connection is
 * initiated, so an application only calls it itself when it is connectable
 * without any service of its own.
 *
 * @return	true - connection support started
 */
bool BtAppConnInit(void)
{
	if (s_pBtAppSdcConn != nullptr)
	{
		// Already started
		return true;
	}

	if (s_pBtAppCfg == nullptr)
	{
		// BtAppInit has not been called yet
		return false;
	}

	// Set first: BtGapInit adds the GAP and GATT services, which comes back
	// to this function.
	s_pBtAppSdcConn = &s_BtAppSdcConn;

	if (BtAppSdcConnStart(s_pBtAppCfg) == false)
	{
		s_pBtAppSdcConn = nullptr;

		return false;
	}

	if (s_bBtAppInitDone)
	{
		// Started after BtAppInit, as a central does at its first connect
		BtAppSdcTimerStart();
	}

	return true;
}

/**
 * @brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
bool BtAppInit(const BtAppCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	int32_t res = 0;

	// The peer table, the attribute database and the GAP services are set
	// up by BtAppConnInit, see the service setup below.
	s_pBtAppCfg = pCfg;
	s_bBtAppInitDone = false;
#if 0
	mpsl_clock_lfclk_cfg_t lfclk = {MPSL_CLOCK_LF_SRC_RC, 0,};
	OscDesc_t const *lfosc = GetLowFreqOscDesc();

	// Set default clock based on system oscillator settings
	if (lfosc->Type == OSC_TYPE_RC)
	{
		lfclk.source = MPSL_CLOCK_LF_SRC_RC;
		lfclk.rc_ctiv = MPSL_RECOMMENDED_RC_CTIV;
		lfclk.rc_temp_ctiv = MPSL_RECOMMENDED_RC_TEMP_CTIV;
	}
	else
	{
		lfclk.accuracy_ppm = lfosc->Accuracy;
		lfclk.source = MPSL_CLOCK_LF_SRC_XTAL;
	}

	lfclk.skip_wait_lfclk_started = MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED;

	mpsl_fem_init();

	DEBUG_PRINTF("mpsl_init\r\n");


	// Initialize Nordic multi-protocol support library (MPSL)
#ifdef NRF54L15_XXAA
	//NVIC_SetPriority(SWI00_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	//NVIC_EnableIRQ(SWI00_IRQn);

	res = mpsl_init(&lfclk, SWI00_IRQn, BtStackMpslAssert);
	res = mpsl_clock_hfclk_latency_set(MPSL_CLOCK_HF_LATENCY_TYPICAL);
	mpsl_pan_rfu();
#else
	// The low priority line must be a real NVIC interrupt; PendSV cannot be
	// pended through the NVIC, so MPSL's low priority processing never ran
	// with it. The handler lives in nrf_mpsl.cpp.
	res = mpsl_init(&lfclk, SWI5_EGU5_IRQn, BtStackMpslAssert);
#endif

	if (res < 0)
	{
		return false;
	}

#ifdef NRF54L15_XXAA
	NVIC_SetPriority(SWI00_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	NVIC_EnableIRQ(SWI00_IRQn);
//	NVIC_SetPriority(RADIO_0_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
//	NVIC_EnableIRQ(RADIO_0_IRQn);
#else
	NVIC_SetPriority(SWI5_EGU5_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	NVIC_EnableIRQ(SWI5_EGU5_IRQn);
#endif
#endif

//	if (MpslInit() == false)
//	{
//		return false;
//	}

	g_BtAppData.CoexMode = pCfg->CoexMode;

	if (pCfg->CoexMode == BTAPP_COEXMODE_1W)
	{
		//mpsl_coex_support_1wire_gpiote_if();
	}
	else if (pCfg->CoexMode == BTAPP_COEXMODE_3W)
	{
		//mpsl_coex_support_802152_3wire_gpiote_if();
	}

	g_BtAppData.AppDevice.Conn.Role = pCfg->Role;
	g_BtAppData.AppDevice.pHciDev = &s_BtHciDev;		// host used by the HCI operation layer (bt_adv_hci etc.)
	// Kept for BtAppSecInit, which the application calls from
	// BtAppInitUserData when it uses security.
	g_BtAppData.SecType = pCfg->SecType;
	g_BtAppData.SecExchg = pCfg->SecExchg;
	g_BtAppData.bSecInit = false;
	DEBUG_PRINTF("g_BtAppData.AppDevice.Conn.Role = %d\r\n", g_BtAppData.AppDevice.Conn.Role);

	g_BtAppData.bScan = false;
//	g_BtAppData.bAdvertising = false;
	g_BtAppData.AppDevice.VendorId = pCfg->VendorId;
	g_BtAppData.AppDevice.ProductId = pCfg->ProductId;
	g_BtAppData.AppDevice.ProductVer = pCfg->ProductVer;
	g_BtAppData.AppDevice.Appearance = pCfg->Appearance;
	g_BtAppData.ConnLedPort = pCfg->ConnLedPort;
	g_BtAppData.ConnLedPin = pCfg->ConnLedPin;
	g_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;

	if (pCfg->ConnLedPort != -1 && pCfg->ConnLedPin != -1)
    {
		IOPinConfig(pCfg->ConnLedPort, pCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

		BtAppConnLedOff();
    }

   // g_BleAppData.Role = pBleAppCfg->Role;

	// Service setup. It runs before the controller is enabled because the
	// controller has to know by then whether links are used. Adding the first
	// service starts connection support (BtAppConnInit), which also installs
	// the GAP and GATT services ahead of the application ones.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtAppInitUserServices();

		if (s_pBtAppSdcConn == nullptr)
		{
			// Peripheral role without any service. The application has to
			// call BtAppConnInit in BtAppInitUserServices to be connectable,
			// or use BTAPP_ROLE_BROADCASTER.
			STORE_PRINTF("BtAppInit FAIL: peripheral role but connection support was not started\r\n");
			return false;
		}

		if (s_pBtAppSdcConn->SrvcDone(pCfg) == false)
		{
			return false;
		}
	}

    if (BtAppStackInit(pCfg) == false)
    {
    	DEBUG_PRINTF("BtAppStackInit failed\r\n");
    	return false;
    }

	// Device address: read the factory-unique value from FICR (NRF_FICR->
	// DEVICEADDR) and use it as a RANDOM STATIC address. This is the proper
	// nRF mechanism and avoids the Zephyr-specific vendor HCI commands
	// (sdc_hci_cmd_vs_zephyr_*). The top two bits of the MSO must be 1 for a
	// static random address; nrf_get_mac_address() already sets them.
	uint64_t mac = nrf_get_mac_address();

	uint8_t ranaddr[6];
	for (int i = 0; i < 6; i++)
	{
		ranaddr[i] = (uint8_t)(mac >> (8 * i));	// LSB first
	}
	// Ensure the static-random marker even if the FICR value changes.
	ranaddr[5] = (ranaddr[5] & 0x3f) | 0xc0;

	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_SET_RANDOM_ADDR, ranaddr, sizeof(ranaddr), NULL, 0) != 0)
	{
		return false;
	}

	// SMP f5/f6 must use the same local address/type the peer sees.
	memcpy(s_BtSmpLocalAddr, ranaddr, 6);
	s_BtSmpLocalAddrType = 1;	// random

	DEBUG_PRINTF("local addr %02x:%02x:%02x:%02x:%02x:%02x type=1\r\n",
				 ranaddr[5], ranaddr[4], ranaddr[3], ranaddr[2], ranaddr[1], ranaddr[0]);

	// LE Read Maximum Data Length return: supported max TX octets, TX time, RX
	// octets, RX time, each 2 bytes little endian.
	uint8_t maxlen[8] = {0};
	BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN, NULL, 0, maxlen, sizeof(maxlen));
	uint16_t maxTxOctets = (uint16_t)(maxlen[0] | (maxlen[1] << 8));
	uint16_t maxTxTime   = (uint16_t)(maxlen[2] | (maxlen[3] << 8));

	uint16_t txOctets = (uint16_t)min(maxTxOctets, pCfg->MaxMtu);
	uint8_t datalen[4];
	datalen[0] = (uint8_t)(txOctets & 0xff);
	datalen[1] = (uint8_t)(txOctets >> 8);
	datalen[2] = (uint8_t)(maxTxTime & 0xff);
	datalen[3] = (uint8_t)(maxTxTime >> 8);
	BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_WRITE_SUGG_DEFAULT_DATA_LEN, datalen, sizeof(datalen), NULL, 0);

	sdc_default_tx_power_set(pCfg->TxPower);

	// Enable all LE meta events EXCEPT the LE Remote Connection Parameter Request
	// event (LE event mask octet 0, bit 5). When that event is unmasked the
	// controller defers every peer connection-parameter-update to the host and
	// waits for an explicit reply. The app layer does not issue that reply, so
	// leaving it enabled stalls the link-layer parameter update a central runs
	// right after connecting or pairing, and the link drops on supervision timeout.
	uint8_t evmask[8];
	memset(evmask, 0xff, sizeof(evmask));
	evmask[0] &= ~(1 << 5);		// LE Remote Connection Parameter Request event
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_SET_EVENT_MASK, evmask, sizeof(evmask), NULL, 0))
	{
		return false;
	}

	uint8_t cbevmask[8];
	memset(cbevmask, 0xff, sizeof(cbevmask));
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_BASEBAND_SET_EVENT_MASK, cbevmask, sizeof(cbevmask), NULL, 0))
	{
		return false;
	}

	uint8_t cbevmask2[8];
	memset(cbevmask2, 0xff, sizeof(cbevmask2));
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_BASEBAND_SET_EVENT_MASK_PAGE2, cbevmask2, sizeof(cbevmask2), NULL, 0))
	{
		return false;
	}

	// The engines of Encrypted Advertising Data are bound by BtEadEngineInit
	// above, when the EAD module first needs them.

	BtAppInitUserData();

	// The security module (SMP, its ECDH engine and the bond store) is linked
	// and started only when the application calls BtAppSecInit, normally from
	// BtAppInitUserData above. A configuration that asks for security without
	// starting it must not run unprotected.
	if (pCfg->SecType != BTGAP_SECTYPE_NONE && g_BtAppData.bSecInit == false)
	{
		STORE_PRINTF("BtAppInit FAIL: SecType=%d but BtAppSecInit was not called\r\n",
					 (int)pCfg->SecType);
		return false;
	}

	// Record whether security was requested. The security module secures
	// each new link when this is set.
	g_BtAppData.AppDevice.bSecure = (pCfg->SecType != BTGAP_SECTYPE_NONE);

    if (pCfg->Role & (BTAPP_ROLE_BROADCASTER | BTAPP_ROLE_PERIPHERAL))
    {
		if (BtAppAdvInit(pCfg) == false)
		{
			return false;
		}
    }
/*
    BtGapInit(pCfg->Role);

    if (pCfg->Role & (BTDEV_ROLE_BROADCASTER | BTDEV_ROLE_PERIPHERAL))
    {
    	if (pCfg->Role & BTDEV_ROLE_PERIPHERAL)
    	{
//    		BtGattSrvcAdd(&s_BtGattSrvc, &s_BtGattSrvcCfg);
//    		BtGattSrvcAdd(&s_BtGapSrvc, &s_BtGapSrvcCfg);

    		BleAppInitUserServices();
    	}

    	if (BleAppAdvInit(pBleAppCfg) == false)
    	{
    		return false;
    	}

    	size_t count = 0;
    	BtGattListEntry_t *tbl = GetEntryTable(&count);
#if 0
    	for (int i = 0; i < count; i++)
    	{
    		DEBUG_PRINTF("tbl[%d]: Hdl: %d (0x%04x), Uuid: %04x, Data: ", i, tbl[i].Hdl, tbl[i].Hdl, tbl[i].TypeUuid.Uuid);
    		uint8_t *p = (uint8_t*)&tbl[i].Val32;
    		for (int j = 0; j < 20; j++)
    		{
    			DEBUG_PRINTF("0x%02x ", p[j]);
    		}
    		DEBUG_PRINTF("\r\n");
    	}
#endif
    }
*/
    //BleAppGapDeviceNameSet(pBleAppCfg->pDevName);
#if !defined(NRF54L15_XXAA) && !defined(NRF54LM20A_XXAA) && !defined(NRF54LM20B_XXAA)
#if (__FPU_USED == 1)
    // Patch for softdevice & FreeRTOS to sleep properly when FPU is in used
    NVIC_SetPriority(FPU_IRQn, 6);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif
#endif

    if (AppEvtHandlerInit(pCfg->pEvtHandlerQueMem, pCfg->EvtHandlerQueMemSize) == false)
    {
    	return false;
    }

	// Connection pool removed: the peer manager (BtPeerInit above) owns
	// the single connection table now.

	// The app timer serves the SMP and GATT timeouts of a link, so it is
	// started only with connection support.
	if (s_pBtAppSdcConn != nullptr)
	{
		s_pBtAppSdcConn->TimerStart();
	}
	s_bBtAppInitDone = true;

    g_BtAppData.State = BTAPP_STATE_INITIALIZED;


	return true;
}

void BtAppRun()
{
	if (g_BtAppData.State != BTAPP_STATE_INITIALIZED)
	{
		return;
	}

	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		BtAdvStart();
	}

DEBUG_PRINTF("Loop\r\n");

	while (1)
	{
		BtAppEvtWait();
		AppEvtHandlerExec();

		BtHciCtlrProcess(&s_BtHciCtlr);
	}
}

// Port-level weak default for BtAppEvtWait. Bare-metal apps fall through to
// __WFE; RTOS apps provide a strong override that does a semaphore take.
__attribute__((weak)) void BtAppEvtWait(void)
{
	__WFE();
}

#if 0
bool BleAppScanInit(ble_uuid128_t * const pBaseUid, ble_uuid_t * const pServUid)
{
    ble_uuid128_t base_uid = *pBaseUid;
    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ret_code_t err_code = sd_ble_uuid_vs_add(&base_uid, &uidtype);
    APP_ERROR_CHECK(err_code);

    //ble_db_discovery_evt_register(pServUid);
    g_BleAppData.bScan = true;

	err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}
#endif

#if 0
bool BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam)
{
	ret_code_t err_code = sd_ble_gap_connect(pDevAddr, &s_BleScanParams,
                                  	  	  	 pConnParam,
											 BLEAPP_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    g_BleAppData.bScan = false;

    return err_code == NRF_SUCCESS;
}
#endif

bool BtAppEnableNotify(uint16_t ConnHandle, uint16_t CccdHandle)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHandle);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return false;
	}

	// Enable notifications by writing 0x0001 to the characteristic's CCCD.
	// A Write Request is used so the server acknowledges the configuration.
	uint8_t cccd[2] = { 0x01, 0x00 };
	return BtAttWriteRequest(pPeer->pHciDev, ConnHandle, CccdHandle, cccd, sizeof(cccd));
}

bool BtAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DataLen)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHandle);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return false;
	}

	// Write without response, matching the write-command path on the SoftDevice
	// ports and the BlueIO UART TX characteristic (WRITE | WRITEWORESP).
	return BtAttWriteCommand(pPeer->pHciDev, ConnHandle, CharHandle, pData, DataLen);
}

// BtAppNotify/BtAppIndicate, their Conn and All forms are the shared weak
// implementations in src/bluetooth/bt_app.cpp. This port used to carry a copy
// that read the active connection handle. What stays here is the disconnect
// command, which is the one part only this port can issue.

bool BleAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DataLen)
{
	return false;
#if 0
	if (ConnHandle == BLE_CONN_HANDLE_INVALID || CharHandle == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = CharHandle,
        .offset   = 0,
        .len      = DataLen,
        .p_value  = pData
    };

    return sd_ble_gattc_write(ConnHandle, &write_params) == NRF_SUCCESS;
#endif
}

#if 0
extern "C" {
#ifdef NRF54L15_XXAA
void SWI00_IRQHandler(void)
#else
void PendSV_Handler(void)
#endif
{
	DEBUG_PRINTF("mpsl_low_priority_process\r\n");
	mpsl_low_priority_process();
}


#ifdef NRF54L15_XXAA
void RADIO_0_IRQHandler(void)
#else
void RADIO_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_RADIO_Handler\r\n");
	MPSL_IRQ_RADIO_Handler();
}

#ifdef NRF54L15_XXAA
void CLOCK_POWER_IRQHandler()
#else
void POWER_CLOCK_IRQHandler()
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_CLOCK_Handler\r\n");
	MPSL_IRQ_CLOCK_Handler();
}

#ifdef NRF54L15_XXAA
void GRTC_3_IRQHandler(void)
#else
void RTC0_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_RTC0_Handler\r\n");
	MPSL_IRQ_RTC0_Handler();
}

#ifdef NRF54L15_XXAA
void TIMER10_IRQHandler(void)
#else
void TIMER0_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_TIMER0_Handler\r\n");
	MPSL_IRQ_TIMER0_Handler();
}

/** @brief MPSL requesting CONSTLAT to be on.
 *
 * The application needs to implement this function.
 * MPSL will call the function when it needs CONSTLAT to be on.
 * It only calls the function on nRF54L Series devices.
 */
void mpsl_constlat_request_callback(void)
{
	DEBUG_PRINTF("mpsl_constlat_request_callback\r\n");
}

/** @brief De-request CONSTLAT to be on.
 *
 * The application needs to implement this function.
 * MPSL will call the function when it no longer needs CONSTLAT to be on.
 * It only only calls the function on nRF54L Series devices.
 */
void mpsl_lowpower_request_callback(void)
{
	DEBUG_PRINTF("mpsl_lowpower_request_callback\r\n");
}

}

#endif
