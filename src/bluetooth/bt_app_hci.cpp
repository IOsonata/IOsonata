/**-------------------------------------------------------------------------
@file	bt_app_hci.cpp

@brief	Bluetooth application host over HCI

The IOsonata Bluetooth host for any controller reached through HCI. The
controller is a BtHciCtlr (bt_hci_ctlr.h), a DeviceIntrf, and this file never
sees what is behind it: the SoftDevice Controller linked into the image on
nRF52/nRF53/nRF54, an nRF5x running the HciController firmware behind a UART
(H4), the HciController on the network core of an nRF5340 behind the IPC
rings, or a USB dongle. The MCU library links this file with one controller
port, bt_hci_ctlr_sdc.cpp, bt_hci_ctlr_uart.cpp or bt_hci_ctlr_ipc.cpp, the
way a sensor driver is given an I2C or an SPI.

What the host asks of its controller is the generic BtHciCtlr set: Enable to
bring it up, Send for ACL data, BtHciCtlrCommand for every command, RxHandler
for what comes back and BtHciCtlrProcess to pump it. The address, the ACL
buffer sizes, the AES and random numbers of SMP and Encrypted Advertising
Data all come from the controller over HCI, so nothing here names an MCU.

The host has no timer of its own. The SMP and GATT transaction timeouts read
BtSmpMsTick and BtGattMsTick, whose library defaults return 0 and leave the
timeouts inert; a controller port (the SoftDevice Controller port does) or
the application defines them from a timer it has. The once a second work of
a link (indication timeout, pairing timeout, bond save retry) runs from
BtAppCheckStatus.

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
#include <stdint.h>
#include <string.h>

#include "istddef.h"
#include "convutil.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_smp.h"		// BtSmpLocalAddrGet override, BtSmpMsTick
#include "bluetooth/bt_adv.h"		// BtAdvOwnAddrGet for the connection stamp
#include "bluetooth/bt_ead.h"		// Encrypted Advertising Data engines
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/services/bt_dis.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_hci_ctlr.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the application configured (UART, USB, RTT, BLE, or any
// other DeviceIntrf); the trace does not assume a transport. A release build
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

// Controller ACL buffers asked for. The controller answers what it has
// through LE Read Buffer Size, which is what the host sizes itself from.
// Keep two buffers beyond the eight-notification 2M target so completion
// processing can refill the controller before the queued data runs out.
#define BT_APP_HCI_RX_MAX_PACKET_COUNT		2
#define BT_APP_HCI_TX_MAX_PACKET_COUNT		10

// Period of the link checks run from BtAppCheckStatus when a millisecond
// clock is available. The checks they pace are the 30 s transaction
// timeouts, for which one second is ample.
#define BT_APP_HCI_TICK_MSEC				1000

// Host implementation state. Neither its layout nor storage leaves this file.
static struct {
	BtHciDevice_t *pHciDev;
	uint8_t Role;
	BTGAP_SECTYPE SecType;
	uint8_t SecExchg;
	bool bSecInit;
	bool bInitialized;
	int8_t ConnLedPort;
	int8_t ConnLedPin;
	uint8_t ConnLedActLevel;
} s_BtAppData = {
	.pHciDev = nullptr,
	.Role = 0,
	.SecType = BTGAP_SECTYPE_NONE,
	.SecExchg = 0,
	.bSecInit = false,
	.bInitialized = false,
	.ConnLedPort = -1,
	.ConnLedPin = -1,
	.ConnLedActLevel = 0,
};

bool BtInitialized(void)
{
	return s_BtAppData.bInitialized;
}

// Called by the peer module and this host's event handlers.
void BtAppConnLedOff(void)
{
	if (s_BtAppData.ConnLedPort < 0 || s_BtAppData.ConnLedPin < 0)
	{
		return;
	}
	if (s_BtAppData.ConnLedActLevel)
	{
		IOPinClear(s_BtAppData.ConnLedPort, s_BtAppData.ConnLedPin);
	}
	else
	{
		IOPinSet(s_BtAppData.ConnLedPort, s_BtAppData.ConnLedPin);
	}
}

void BtAppConnLedOn(void)
{
	if (s_BtAppData.ConnLedPort < 0 || s_BtAppData.ConnLedPin < 0)
	{
		return;
	}
	if (s_BtAppData.ConnLedActLevel)
	{
		IOPinSet(s_BtAppData.ConnLedPort, s_BtAppData.ConnLedPin);
	}
	else
	{
		IOPinClear(s_BtAppData.ConnLedPort, s_BtAppData.ConnLedPin);
	}
}

// A connection/init event must not force the optional scan object into the image.
extern void BtScanEnded(void) __attribute__((weak));

static void BtAppScanEnded(void)
{
	if (BtScanEnded != nullptr)
	{
		BtScanEnded();
	}
}

// Internal security operations; they do not link the optional security module.
BTGAP_SECTYPE BtAppSecConfigGet(uint8_t *pExchg)
{
	*pExchg = s_BtAppData.SecExchg;
	return s_BtAppData.SecType;
}

bool BtAppSecIsStarted(void)
{
	return s_BtAppData.bSecInit;
}

void BtAppSecStarted(void)
{
	s_BtAppData.bSecInit = true;
}

void BtAppEvtHandler(BtHciDevice_t * const pDev, uint32_t Evt);
void BtAppConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6]);
void BtAppDisconnected(uint16_t ConnHdl, uint8_t Reason);
void BtAppSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent);
bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *DavData);

static uint32_t BtAppSendData(void *pData, uint32_t Len);

static BtHciDevice_t s_BtHciDev = {
	.pCtx = nullptr,
	.SendData = BtAppSendData,
	.Command = BtHciCtlrCommand,
	.EvtHandler = BtAppEvtHandler,
	// Connected, Disconnected and SendCompleted are set by BtAppConnInit
	.ScanReport = BtAppScanReport,
	.AdvTimeout = BtAppAdvTimeoutHandler,
};

// The controller. Its port decides what Enable, Send and BtHciCtlrCommand
// do; this host wires the receive handler.
static BtHciCtlrDev_t s_BtHciCtlr;

// Connection support. What a link needs (peer table, attribute database, GAP
// and GATT services, ACL data path, GATT timeout) is reached only through
// this table. BtAppConnInit installs it. An application that only advertises
// or scans never reaches BtAppConnInit, so none of it is linked.
typedef struct {
	void (*AclData)(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pPkt);
	void (*Tick)(void);
	bool (*SrvcDone)(const BtAppCfg_t *pCfg);
} BtAppHciConn_t;

static const BtAppHciConn_t *s_pBtAppHciConn = nullptr;

// Periodic security work: pairing timeout and bond save retry, run once per
// second from BtAppCheckStatus. Set by BtAppSecInit through BtAppSecPollSet.
// An application that does not use security never sets it, so none of it is
// linked.
static void (*s_pBtAppSecPoll)(void) = nullptr;

// Millisecond stamp of the last link check, see BtAppCheckStatus
static uint32_t s_BtAppTickMs = 0;

// Configuration given to BtAppInit, used by BtAppConnInit
static const BtAppCfg_t *s_pBtAppCfg = nullptr;

// On-air local address SMP needs for c1/f5/f6 (set when the random static
// address is configured below). Defaults to public/zero until then.
static uint8_t s_BtSmpLocalAddr[6] = {0};
static uint8_t s_BtSmpLocalAddrType = 0;

static uint32_t BtAppSendData(void *pData, uint32_t Len)
{
	if (s_BtHciCtlr.Send == nullptr)
	{
		return 0;
	}

	return (uint32_t)s_BtHciCtlr.Send(&s_BtHciCtlr, pData, Len);
}

// Override the weak SMP accessor so the toolbox uses the device's configured
// address. Declared in bt_smp.h, so no linkage specifier is needed here.
void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	*pType = s_BtSmpLocalAddrType;
	memcpy(pAddr, s_BtSmpLocalAddr, 6);
}

// Route each HCI packet the controller hands up to the host process entry.
static void BtAppCtlrRx(BtHciCtlrDev_t * const pDev, bool bIsEvent, uint8_t *pPacket)
{
	(void)pDev;

	if (bIsEvent)
	{
		BtHciProcessEvent(&s_BtHciDev, (BtHciEvtPacket_t*)pPacket);
	}
	else if (s_pBtAppHciConn != nullptr)
	{
		s_pBtAppHciConn->AclData(&s_BtHciDev, (BtHciACLDataPacket_t*)pPacket);
	}
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

// Security and DFU modules stay optional until the application uses them.
extern "C" void BtSmpCheckStatus(void) __attribute__((weak));
extern "C" void BtSmpBondNvmCheckStatus(void) __attribute__((weak));
extern void BtDfuSmpCheckStatus(void) __attribute__((weak));
extern void BtHciTxCheckStatus(BtHciDevice_t *pDev) __attribute__((weak));

// Called from the application status check (AppRun when its queue is empty,
// the Bluetooth worker of an RTOS before it blocks). Retries the work the
// Bluetooth queue refused and runs the link checks.
void BtAppCheckStatus(void)
{
	BtHciCtlrCheckStatus();
	if (BtHciTxCheckStatus) BtHciTxCheckStatus(&s_BtHciDev);

	// Once a second when a millisecond clock is available, each pass when it
	// is not: without a clock the timeouts these checks serve are inert and
	// what remains of them is cheap.
	uint32_t ms = BtSmpMsTick();
	if (ms == 0 || (uint32_t)(ms - s_BtAppTickMs) >= BT_APP_HCI_TICK_MSEC)
	{
		s_BtAppTickMs = ms;

		// Generic indication transaction timeout (Core Vol 3 Part F 3.3.3).
		// Cheap no-op when nothing is pending.
		if (s_pBtAppHciConn != nullptr)
		{
			s_pBtAppHciConn->Tick();
		}

		// Pairing timeout (Core Vol 3 Part H 3.4) and bond save retry, when
		// the application uses security.
		if (s_pBtAppSecPoll != nullptr)
		{
			s_pBtAppSecPoll();
		}
	}

	if (BtSmpCheckStatus != nullptr)
	{
		BtSmpCheckStatus();
	}
	if (BtSmpBondNvmCheckStatus != nullptr)
	{
		BtSmpBondNvmCheckStatus();
	}
	if (BtDfuSmpCheckStatus != nullptr)
	{
		BtDfuSmpCheckStatus();
	}
}

void BtAppSetDevName(const char *pName)
{
	BtGapSetDevName(pName);
}

void BtAppEvtHandler(BtHciDevice_t * const pDev, uint32_t Evt)
{
	(void)pDev;
	(void)Evt;
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

	// If a secure SecType was configured, the link is secured by the security
	// module (bt_sec_hci.cpp), which hooks the connection callback when the
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
	(void)Reason;

	DEBUG_PRINTF("BtAppDisconnected: ConnHdl= %d (0x%x); Reason = %d (0x%x)\r\n",
			ConnHdl, ConnHdl, Reason, Reason);

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	BtPeerFree(pPeer);

	bool bConnected = BtConnected();

	BtAppEvtDisconnected(ConnHdl);

	if (bConnected == false &&
		(s_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER)))
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
	(void)rc;
	DEBUG_PRINTF("BtAppDisconnect: hdl=%u rc=%u\r\n", ConnHdl, rc);
}

// Read the LE ACL buffers of the controller: the length of an ACL packet it
// takes and how many it holds. A controller that shares its buffers with
// BR/EDR reports 0 for LE and gives them in Read Buffer Size instead.
static bool BtAppAclBufRead(uint16_t *pLen, uint8_t *pCnt)
{
	// LE Read Buffer Size return: packet length (2), packet count (1)
	uint8_t le[3] = {0};
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_READ_BUFF_SIZE, NULL, 0, le, sizeof(le)) != 0)
	{
		return false;
	}

	*pLen = (uint16_t)(le[0] | (le[1] << 8));
	*pCnt = le[2];

	if (*pLen != 0 && *pCnt != 0)
	{
		return true;
	}

	// Read Buffer Size return: ACL length (2), SCO length (1), ACL count (2),
	// SCO count (2)
	uint8_t bb[7] = {0};
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_INFO_READ_BUFFER_SIZE, NULL, 0, bb, sizeof(bb)) != 0)
	{
		return false;
	}

	uint16_t cnt = (uint16_t)(bb[3] | (bb[4] << 8));

	*pLen = (uint16_t)(bb[0] | (bb[1] << 8));
	*pCnt = cnt > 255 ? 255 : (uint8_t)cnt;

	return *pLen != 0 && *pCnt != 0;
}

bool BtAppStackInit(const BtAppCfg_t *pCfg)
{
	BtHciCtlrCfg_t ctlrcfg = { };
	ctlrcfg.RxHandler = BtAppCtlrRx;
	// No wake callback: the controller port delivers the received HCI
	// packets in its own context, and the host queues what must run outside it.
	ctlrcfg.Role = pCfg->Role;
	ctlrcfg.CentralDevMax = pCfg->CentralDevMax;
	ctlrcfg.PeriphDevMax = pCfg->PeriphDevMax;
	ctlrcfg.RxPktCount = BT_APP_HCI_RX_MAX_PACKET_COUNT;
	ctlrcfg.TxPktCount = BT_APP_HCI_TX_MAX_PACKET_COUNT;
	ctlrcfg.MaxDataLen = BTAPP_DEFAULT_MAX_DATA_LEN;
	// What the application asked for reaches the controller here. Without this
	// the counts stay zero, the controller reserves nothing, and the periodic
	// advertising commands are refused however well formed they are.
	ctlrcfg.PeriodicAdvCount = pCfg->PeriodicAdvCount;
	ctlrcfg.PeriodicSyncCount = pCfg->PeriodicSyncCount;
	ctlrcfg.PawrAdvCount = pCfg->PawrAdvCount;
	ctlrcfg.PawrSyncCount = pCfg->PawrSyncCount;
	ctlrcfg.TxPower = (int8_t)pCfg->TxPower;

	if (BtHciCtlrEnable(&s_BtHciCtlr, &ctlrcfg) == false)
	{
		return false;
	}

	// A fresh controller: ask it again what crypto it has when first needed.
	BtCryptoCtlrReset();

	// The ACL buffers of the controller, whatever it is, set the TX
	// fragmentation and the credit flow control of the host.
	uint16_t acllen = 0;
	uint8_t aclcnt = 0;
	if (BtAppAclBufRead(&acllen, &aclcnt) == false)
	{
		DEBUG_PRINTF("No LE ACL buffer\r\n");
		return false;
	}

	BtHciSetLeAclBuffer(&s_BtHciDev, acllen, aclcnt);

	return true;
}

// Last step of the service setup of a peripheral, after the application has
// added its services: Device Information Service, appearance and preferred
// connection parameters.
static bool BtAppHciSrvcDone(const BtAppCfg_t *pCfg)
{
	// Register Device Information Service when the app supplies device
	// info. Generic bt_dis adds it to the same ATT DB as user services.
	if (pCfg->pDevInfo != NULL && !BtDisInit(pCfg))
	{
		STORE_PRINTF("BtAppInit FAIL: Device Information service initialization\r\n");
		return false;
	}

	BtGapSetAppearance(pCfg->Appearance);

	BtGattPreferedConnParams_t connparm = {
		MSEC_TO_1_25(pCfg->ConnIntervalMin), MSEC_TO_1_25(pCfg->ConnIntervalMax),
		0, 400};
	BtGapSetPreferedConnParam(&connparm);

	return true;
}

static const BtAppHciConn_t s_BtAppHciConn = {
	.AclData = BtHciProcessData,
	.Tick = BtGattIndicationTimeoutCheck,
	.SrvcDone = BtAppHciSrvcDone,
};

// Engines of Encrypted Advertising Data: the AES and the random numbers of
// the controller, over HCI. Called by the EAD module the first time it needs
// an engine, so they are linked only when the application uses EAD
// (BtAdvEadKeySet, BtAdvDecrypt). Declared in bt_ead.h.
bool BtEadEngineInit(void)
{
	bool res = BtEadInit(BtCryptoCtlrInit(), BtCryptoCtlrRng());

	// Whole-packet CCM from the controller when it has the I-SYST vendor
	// command, otherwise block by block over LE Encrypt.
	BtEadSetAeadEngine(BtCryptoCtlrAead());

	return res;
}

static bool BtAppHciConnStart(const BtAppCfg_t *pCfg)
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
	s_BtHciDev.ConnectFailed = BtAppEvtConnectFailed;
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
	if (s_pBtAppHciConn != nullptr)
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
	s_pBtAppHciConn = &s_BtAppHciConn;

	if (BtAppHciConnStart(s_pBtAppCfg) == false)
	{
		s_pBtAppHciConn = nullptr;

		return false;
	}

	return true;
}

// The address of this device, in this order:
//  1. the application's own, from BtAppStaticAddrGet (a provisioned
//     identity), a static random address;
//  2. the static address of the controller, from the I-SYST Read Static
//     Address vendor command (HciController and the IOsonata controller
//     ports answer it from the factory address of the part), static random;
//  3. the public address of the controller, from Read BD_ADDR, used as
//     public when it is not all zeros, as a dongle with a programmed
//     address or another vendor's controller has it;
//  4. one generated from the controller's random numbers, kept for this
//     session, static random.
// A static random address gets the top two bits of its most significant
// octet set, that is what makes it one, and is loaded with LE Set Random
// Address; a public one needs nothing loaded. SMP and advertising take the
// address and its type from here so the peer sees the same identity in
// both.
__attribute__((weak)) bool BtAppStaticAddrGet(uint8_t Addr[6])
{
	(void)Addr;

	return false;
}

static bool BtAppAddrIsZero(const uint8_t Addr[6])
{
	for (int i = 0; i < 6; i++)
	{
		if (Addr[i] != 0)
		{
			return false;
		}
	}

	return true;
}

static bool BtAppAddrSet(void)
{
	uint8_t addr[6] = {0};
	bool bRandom = true;
	bool bHave = BtAppStaticAddrGet(addr);

	if (bHave == false)
	{
		bHave = BtHciCommand(&s_BtHciDev, BT_HCI_CMD_VS_ISYST_READ_STATIC_ADDR, NULL, 0,
							 addr, sizeof(addr)) == 0 && BtAppAddrIsZero(addr) == false;
	}

	if (bHave == false)
	{
		bHave = BtHciCommand(&s_BtHciDev, BT_HCI_CMD_INFO_READ_BD_ADDR, NULL, 0,
							 addr, sizeof(addr)) == 0 && BtAppAddrIsZero(addr) == false;
		bRandom = !bHave;
	}

	if (bHave == false)
	{
		if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_RAND, NULL, 0, addr, sizeof(addr)) != 0)
		{
			return false;
		}
	}

	if (bRandom)
	{
		// Ensure the static-random marker whatever the source.
		addr[5] = (addr[5] & 0x3f) | 0xc0;

		if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_SET_RANDOM_ADDR, addr, sizeof(addr), NULL, 0) != 0)
		{
			return false;
		}
	}

	// SMP f5/f6 must use the same local address/type the peer sees.
	memcpy(s_BtSmpLocalAddr, addr, 6);
	s_BtSmpLocalAddrType = bRandom ? 1 : 0;

	DEBUG_PRINTF("local addr %02x:%02x:%02x:%02x:%02x:%02x type=%d\r\n",
				 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], s_BtSmpLocalAddrType);

	return true;
}

/**
 * @brief	Bluetooth application initialization.
 *
 * Brings up the controller, configures the host and starts advertising for
 * a peripheral or broadcaster.
 */
bool BtAppInit(const BtAppCfg_t *pCfg)
{
	s_BtAppData.bInitialized = false;
	if (pCfg == nullptr)
	{
		return false;
	}

	// The peer table, the attribute database and the GAP services are set
	// up by BtAppConnInit, see the service setup below.
	s_pBtAppCfg = pCfg;

	s_BtAppData.Role = pCfg->Role;
	s_BtAppData.pHciDev = &s_BtHciDev;
	// Kept for BtAppSecInit, which the application calls from
	// BtAppInitUserData when it uses security.
	s_BtAppData.SecType = pCfg->SecType;
	s_BtAppData.SecExchg = pCfg->SecExchg;
	s_BtAppData.bSecInit = false;
	DEBUG_PRINTF("BLE role = %d\r\n", s_BtAppData.Role);

	BtAppScanEnded();
	s_BtAppData.ConnLedPort = pCfg->ConnLedPort;
	s_BtAppData.ConnLedPin = pCfg->ConnLedPin;
	s_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;

	if (pCfg->ConnLedPort != -1 && pCfg->ConnLedPin != -1)
    {
		IOPinConfig(pCfg->ConnLedPort, pCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

		BtAppConnLedOff();
    }

	// Service setup. It runs before the controller is enabled because the
	// controller has to know by then whether links are used. Adding the first
	// service starts connection support (BtAppConnInit), which also installs
	// the GAP and GATT services ahead of the application ones.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtAppInitUserServices();

		if (s_pBtAppHciConn == nullptr)
		{
			// Peripheral role without any service. The application has to
			// call BtAppConnInit in BtAppInitUserServices to be connectable,
			// or use BTAPP_ROLE_BROADCASTER.
			STORE_PRINTF("BtAppInit FAIL: peripheral role but connection support was not started\r\n");
			return false;
		}

		if (s_pBtAppHciConn->SrvcDone(pCfg) == false)
		{
			return false;
		}
	}

    if (BtAppStackInit(pCfg) == false)
    {
    	DEBUG_PRINTF("BtAppStackInit failed\r\n");
    	return false;
    }

	if (BtAppAddrSet() == false)
	{
		DEBUG_PRINTF("No device address\r\n");
		return false;
	}

	// LE Read Maximum Data Length return: supported max TX octets, TX time, RX
	// octets, RX time, each 2 bytes little endian.
	uint8_t maxlen[8] = {0};
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN, NULL, 0, maxlen, sizeof(maxlen)) == 0)
	{
		uint16_t maxTxOctets = (uint16_t)(maxlen[0] | (maxlen[1] << 8));
		uint16_t maxTxTime   = (uint16_t)(maxlen[2] | (maxlen[3] << 8));

		// The link-layer payload includes the four-byte L2CAP header.
		uint32_t mtu = max((uint32_t)pCfg->MaxMtu, (uint32_t)23U);
		uint16_t txOctets = (uint16_t)min((uint32_t)maxTxOctets, min(mtu + 4U, (uint32_t)251U));
		uint8_t datalen[4];
		datalen[0] = (uint8_t)(txOctets & 0xff);
		datalen[1] = (uint8_t)(txOctets >> 8);
		datalen[2] = (uint8_t)(maxTxTime & 0xff);
		datalen[3] = (uint8_t)(maxTxTime >> 8);
		BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_WRITE_SUGG_DEFAULT_DATA_LEN, datalen, sizeof(datalen), NULL, 0);
	}

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
	if (pCfg->SecType != BTGAP_SECTYPE_NONE && s_BtAppData.bSecInit == false)
	{
		STORE_PRINTF("BtAppInit FAIL: SecType=%d but security initialization did not complete\r\n",
					 (int)pCfg->SecType);
		return false;
	}

    if (pCfg->Role & (BTAPP_ROLE_BROADCASTER | BTAPP_ROLE_PERIPHERAL))
    {
		if (BtAppAdvInit(pCfg) == false)
		{
			return false;
		}
    }

	s_BtAppTickMs = BtSmpMsTick();
	s_BtAppData.bInitialized = true;

	// Advertising starts here and not from the event queue: an application
	// interrupt can fill the queue before the application runs it.
	if (s_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
    	BtAdvStart();
	}

	return true;
}

// Called by BtAppSecInit of the security module (bt_sec_hci.cpp) to have its
// periodic work called once per second from BtAppCheckStatus.
void BtAppSecPollSet(void (*Poll)(void))
{
	s_pBtAppSecPoll = Poll;
}

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
// implementations in src/bluetooth/bt_app.cpp.

// Internal controller access for the HCI command and security modules.
BtHciDevice_t *BtAppHciDevice(void)
{
	return s_BtAppData.pHciDev;
}
