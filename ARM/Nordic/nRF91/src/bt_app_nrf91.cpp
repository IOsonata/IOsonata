/**-------------------------------------------------------------------------
@file	bt_app_nrf91.cpp

@brief	Bluetooth application port of the nRF91 series

The nRF91 has no Bluetooth radio. The IOsonata Bluetooth host runs here and
talks HCI over a UART (H4) to an nRF5x running the HciController firmware,
which runs the controller. See bt_hci_uart.h for the transport.

The application gives the UART to the controller in g_BtHciUartCfg. When it
leaves pRxMem or pTxMem NULL, this port supplies the FIFO memory.

Received HCI packets are framed and handled from the Bluetooth event queue
(BtEvtQue), never in the UART interrupt. Commands wait for their response in
the calling context, which is the event queue or BtAppInit.

Not supported yet on this port: security (BtAppSecInit), Encrypted
Advertising Data, and the default TX power of BtAppCfg_t, for which standard
HCI has no command.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <string.h>

#include "nrf.h"

#include "istddef.h"
#include "convutil.h"
#include "coredev/iopincfg.h"
#include "coredev/timer.h"
#include "timer_nrfx.h"
#include "iopinctrl.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_smp.h"		// BtSmpLocalAddrGet override
#include "bluetooth/bt_adv.h"		// BtAdvOwnAddrGet for the connection stamp
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/services/bt_dis.h"
#include "bluetooth/bt_hci_uart.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. A release build defines
// NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

// Received packet queue depth. Packets the controller sends while a command
// waits for its response are kept here.
#ifndef BT_APP_NRF91_PKT_COUNT
#define BT_APP_NRF91_PKT_COUNT			8
#endif

// UART FIFOs used when the application gives none. The RX FIFO holds what
// arrives while the event queue is busy, 2 KB is 20 ms at 1 Mbaud.
#ifndef BT_APP_NRF91_UART_RXFIFO_SIZE
#define BT_APP_NRF91_UART_RXFIFO_SIZE	2048
#endif

#ifndef BT_APP_NRF91_UART_TXFIFO_SIZE
#define BT_APP_NRF91_UART_TXFIFO_SIZE	1024
#endif

// Packets handled per event before the handler queues itself again, so the
// other events of the queue run in between.
#define BT_APP_NRF91_RX_BURST			4

// HCI Reset attempts. The controller may still be starting when the nRF91 is
// up, each attempt waits up to the transport timeout.
#define BT_APP_NRF91_RESET_TRY			5

#if defined(NRF_TRUSTZONE_NONSECURE) && defined(NRF_FICR_NS)
#define BT_APP_NRF91_FICR				NRF_FICR_NS
#else
#define BT_APP_NRF91_FICR				NRF_FICR_S
#endif

static_assert(BT_APP_NRF91_UART_TXFIFO_SIZE >= BT_HCI_UART_TXFIFO_MIN,
			  "TX FIFO must hold one whole HCI packet");

void BtAppEvtHandler(BtHciDevice_t * const pDev, uint32_t Evt);
void BtAppConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6]);
void BtAppDisconnected(uint16_t ConnHdl, uint8_t Reason);
void BtAppSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent);

static void BtAppNrf91TimerHandler(TimerDev_t * const pTimer, uint32_t Evt);
static uint32_t BtAppNrf91SendData(void *pData, uint32_t Len);
static uint8_t BtAppNrf91Command(BtHciDevice_t * const pDev, uint16_t OpCode, const void *pParam,
								 uint8_t ParamLen, void *pRet, uint8_t RetLen);

static BtHciDevice_t s_BtHciDev = {
	.pCtx = (void*)&g_BtAppData,
	.SendData = BtAppNrf91SendData,
	.Command = BtAppNrf91Command,
	.EvtHandler = BtAppEvtHandler,
	// Connected, Disconnected and SendCompleted are set by BtAppConnInit
	.ScanReport = BtAppScanReport,
	.AdvTimeout = BtAppAdvTimeoutHandler,
};

// HCI transport to the controller
static BtHciUartDev_t s_BtHciUart;

alignas(4) static uint8_t s_BtAppNrf91PktMem[BT_HCI_UART_PKTMEM_SIZE(BT_APP_NRF91_PKT_COUNT)];
alignas(4) static uint8_t s_BtAppNrf91UartRxMem[CFIFO_MEMSIZE(BT_APP_NRF91_UART_RXFIFO_SIZE)];
alignas(4) static uint8_t s_BtAppNrf91UartTxMem[CFIFO_MEMSIZE(BT_APP_NRF91_UART_TXFIFO_SIZE)];

// Connection support. What a link needs (peer table, attribute database, GAP
// and GATT services, ACL data path, GATT timeout) is reached only through
// this table. BtAppConnInit installs it. An application that only advertises
// or scans never reaches BtAppConnInit, so none of it is linked.
typedef struct {
	void (*AclData)(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pPkt);
	void (*Tick)(void);
	bool (*SrvcDone)(const BtAppCfg_t *pCfg);
	void (*TimerStart)(void);
} BtAppNrf91Conn_t;

static const BtAppNrf91Conn_t *s_pBtAppNrf91Conn = nullptr;

// Configuration given to BtAppInit, used by BtAppConnInit
static const BtAppCfg_t *s_pBtAppCfg = nullptr;

// Set at the end of BtAppInit. Connection support started after that point
// starts the timer itself.
static bool s_bBtAppInitDone = false;

// On-air local address SMP needs for c1/f5/f6, set with the random static
// address below.
static uint8_t s_BtSmpLocalAddr[6] = {0};
static uint8_t s_BtSmpLocalAddrType = 0;

const static TimerCfg_t s_BtAppNrf91TimerCfg = {
	.DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,
	.EvtHandler = BtAppNrf91TimerHandler
};

// Application timer: millisecond clock of the SMP and GATT transaction
// timeouts and their 1 s check. Only a link uses it.
static TimerDev_t s_BtAppNrf91Timer;

// Deferred work state, shared by the received packet event and the timer
// event: a refused BtEvtQue leaves the work pending, BtAppCheckStatus queues
// it again.
enum {
	BT_APP_NRF91_EVT_IDLE,
	BT_APP_NRF91_EVT_PENDING,
	BT_APP_NRF91_EVT_QUEUED,
};

static volatile uint8_t s_BtAppNrf91RxState = BT_APP_NRF91_EVT_IDLE;
static volatile uint8_t s_BtAppNrf91TickState = BT_APP_NRF91_EVT_IDLE;

static void BtAppNrf91EvtQue(volatile uint8_t *pState, BtEvtQueHandler_t Handler)
{
	if (*pState == BT_APP_NRF91_EVT_PENDING)
	{
		*pState = BT_APP_NRF91_EVT_QUEUED;
		if (BtEvtQue(0, nullptr, Handler) == false)
		{
			*pState = BT_APP_NRF91_EVT_PENDING;
		}
	}
}

static uint32_t BtAppNrf91SendData(void *pData, uint32_t Len)
{
	return BtHciUartSend(&s_BtHciUart, BT_HCI_UART_PKT_ACL, pData, Len) ? Len : 0;
}

static uint8_t BtAppNrf91Command(BtHciDevice_t * const pDev, uint16_t OpCode, const void *pParam,
								 uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	return BtHciUartCommand(&s_BtHciUart, pDev, OpCode, pParam, ParamLen, pRet, RetLen);
}

// Command Complete and Command Status, as soon as they are framed. The host
// only sets the command response and credit fields of the device for them.
static void BtAppNrf91CmdRsp(BtHciUartDev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	(void)pDev;
	(void)Type;
	(void)Len;

	BtHciProcessEvent(&s_BtHciDev, (BtHciEvtPacket_t*)pPkt);
}

// Every other packet, in arrival order, from the event queue
static void BtAppNrf91Rx(BtHciUartDev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	(void)pDev;
	(void)Len;

	switch (Type)
	{
		case BT_HCI_UART_PKT_EVT:
			BtHciProcessEvent(&s_BtHciDev, (BtHciEvtPacket_t*)pPkt);
			break;

		case BT_HCI_UART_PKT_ACL:
			if (s_pBtAppNrf91Conn != nullptr)
			{
				s_pBtAppNrf91Conn->AclData(&s_BtHciDev, (BtHciACLDataPacket_t*)pPkt);
			}
			break;

		default:
			// No synchronous or ISO data on this port
			break;
	}
}

static void BtAppNrf91RxEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	// Bytes arriving from here on queue the next event
	s_BtAppNrf91RxState = BT_APP_NRF91_EVT_IDLE;

	if (BtHciUartProcess(&s_BtHciUart, BT_APP_NRF91_RX_BURST))
	{
		// Packets left, handled by the next event
		s_BtAppNrf91RxState = BT_APP_NRF91_EVT_PENDING;
		BtAppNrf91EvtQue(&s_BtAppNrf91RxState, BtAppNrf91RxEvt);
	}
}

// Bytes arrived, in the UART interrupt
static void BtAppNrf91RxReady(BtHciUartDev_t * const pDev)
{
	(void)pDev;

	if (s_BtAppNrf91RxState == BT_APP_NRF91_EVT_IDLE)
	{
		s_BtAppNrf91RxState = BT_APP_NRF91_EVT_PENDING;
		BtAppNrf91EvtQue(&s_BtAppNrf91RxState, BtAppNrf91RxEvt);
	}
}

// Override the weak SMP accessor so the toolbox uses the device's configured
// address. Declared in bt_smp.h, so no linkage specifier is needed here.
void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	*pType = s_BtSmpLocalAddrType;
	memcpy(pAddr, s_BtSmpLocalAddr, 6);
}

// Millisecond clock for the generic SMP/GATT transaction timeouts. These
// override the weak BtSmpMsTick/BtGattMsTick defaults (which return 0, leaving
// the timeouts inert). The timer is started with connection support.
uint32_t BtSmpMsTick(void)
{
	if (s_BtAppNrf91Timer.GetTickCount == nullptr)
	{
		// Timer not started
		return 0;
	}

	return TimerGetMilisecond(&s_BtAppNrf91Timer);
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

// Timeout checks, queued once per second by the timer
static void BtAppNrf91TickEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_BtAppNrf91TickState = BT_APP_NRF91_EVT_IDLE;

	// Generic indication transaction timeout (Core Vol 3 Part F 3.3.3). Cheap
	// no-op when nothing is pending.
	if (s_pBtAppNrf91Conn != nullptr)
	{
		s_pBtAppNrf91Conn->Tick();
	}
}

static void BtAppNrf91TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
	(void)pTimer;

	if (Evt & TIMER_EVT_TRIGGER(0))
	{
		if (s_BtAppNrf91TickState == BT_APP_NRF91_EVT_IDLE)
		{
			s_BtAppNrf91TickState = BT_APP_NRF91_EVT_PENDING;
		}
		BtAppNrf91EvtQue(&s_BtAppNrf91TickState, BtAppNrf91TickEvt);
	}
}

// Security and DFU modules stay optional until the application uses them.
extern "C" void BtSmpCheckStatus(void) __attribute__((weak));
extern "C" void BtSmpBondNvmCheckStatus(void) __attribute__((weak));
extern void BtDfuSmpCheckStatus(void) __attribute__((weak));

void BtAppCheckStatus(void)
{
	BtAppNrf91EvtQue(&s_BtAppNrf91RxState, BtAppNrf91RxEvt);
	BtAppNrf91EvtQue(&s_BtAppNrf91TickState, BtAppNrf91TickEvt);
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
	// address is the device's configured one.
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

	BtDevice_t *pPeer = BtPeerConnected(ConnHdl, Role, PeerAddrType, PeerAddr,
										ownType, ownAddr);
	if (pPeer != NULL)
	{
		pPeer->pHciDev = (BtHciDevice_t*) &s_BtHciDev;
		s_BtHciDev.pBtDev = (void*) pPeer;
	}

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

	bool bConnected = BtPeerIsConnected();

	if (bConnected == false)
	{
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
}

void BtAppDisconnectConn(uint16_t ConnHdl)
{
	if (ConnHdl == BT_CONN_HDL_INVALID)
	{
		return;
	}

	// HCI Disconnect, reason 0x13 Remote User Terminated Connection
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
static bool BtAppNrf91AclBufRead(uint16_t *pLen, uint8_t *pCnt)
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
	(void)pCfg;

	UARTCfg_t ucfg = g_BtHciUartCfg;

	if (ucfg.pRxMem == nullptr)
	{
		ucfg.pRxMem = s_BtAppNrf91UartRxMem;
		ucfg.RxMemSize = sizeof(s_BtAppNrf91UartRxMem);
	}
	if (ucfg.pTxMem == nullptr)
	{
		ucfg.pTxMem = s_BtAppNrf91UartTxMem;
		ucfg.TxMemSize = sizeof(s_BtAppNrf91UartTxMem);
	}

	BtHciUartCfg_t hcicfg = {};
	hcicfg.pUartCfg = &ucfg;
	hcicfg.pPktMem = s_BtAppNrf91PktMem;
	hcicfg.PktMemSize = sizeof(s_BtAppNrf91PktMem);
	hcicfg.RxHandler = BtAppNrf91Rx;
	hcicfg.CmdRspHandler = BtAppNrf91CmdRsp;
	hcicfg.RxReady = BtAppNrf91RxReady;

	if (BtHciUartInit(&s_BtHciUart, &hcicfg) == false)
	{
		DEBUG_PRINTF("BtHciUartInit failed\r\n");
		return false;
	}

	// One command may be sent before the first Command Complete
	s_BtHciDev.CmdCredit = 1;

	uint8_t res = BT_HCI_UART_ERR_TIMEOUT;
	for (int i = 0; i < BT_APP_NRF91_RESET_TRY && res != 0; i++)
	{
		// Bytes from the controller start can leave the framing out of step
		BtHciUartFlush(&s_BtHciUart);
		res = BtHciCommand(&s_BtHciDev, BT_HCI_CMD_BASEBAND_RESET, NULL, 0, NULL, 0);
	}

	if (res != 0)
	{
		DEBUG_PRINTF("HCI Reset failed %d\r\n", res);
		return false;
	}

	uint16_t acllen = 0;
	uint8_t aclcnt = 0;
	if (BtAppNrf91AclBufRead(&acllen, &aclcnt) == false)
	{
		DEBUG_PRINTF("No LE ACL buffer\r\n");
		return false;
	}

	// TX fragmentation and credit flow control from the controller buffers
	BtHciSetLeAclBuffer(&s_BtHciDev, acllen, aclcnt);

	return true;
}

// Last step of the service setup of a peripheral, after the application has
// added its services: Device Information Service, appearance and preferred
// connection parameters.
static bool BtAppNrf91SrvcDone(const BtAppCfg_t *pCfg)
{
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
// 30 s transaction-timeout checks.
static void BtAppNrf91TimerStart(void)
{
	if (nRFxLFTimerInit(&s_BtAppNrf91Timer, &s_BtAppNrf91TimerCfg))
	{
		s_BtAppNrf91Timer.EnableTrigger(&s_BtAppNrf91Timer, 0, 1000000000ULL,
										TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	}
}

static const BtAppNrf91Conn_t s_BtAppNrf91Conn = {
	.AclData = BtHciProcessData,
	.Tick = BtGattIndicationTimeoutCheck,
	.SrvcDone = BtAppNrf91SrvcDone,
	.TimerStart = BtAppNrf91TimerStart,
};

static bool BtAppNrf91ConnStart(const BtAppCfg_t *pCfg)
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
	if (s_pBtAppNrf91Conn != nullptr)
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
	s_pBtAppNrf91Conn = &s_BtAppNrf91Conn;

	if (BtAppNrf91ConnStart(s_pBtAppCfg) == false)
	{
		s_pBtAppNrf91Conn = nullptr;

		return false;
	}

	if (s_bBtAppInitDone)
	{
		// Started after BtAppInit, as a central does at its first connect
		BtAppNrf91TimerStart();
	}

	return true;
}

bool BtAppInit(const BtAppCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	s_pBtAppCfg = pCfg;
	s_bBtAppInitDone = false;

	g_BtAppData.CoexMode = pCfg->CoexMode;
	g_BtAppData.AppDevice.Conn.Role = pCfg->Role;
	g_BtAppData.AppDevice.pHciDev = &s_BtHciDev;		// host used by the HCI operation layer (bt_adv_hci etc.)
	g_BtAppData.SecType = pCfg->SecType;
	g_BtAppData.SecExchg = pCfg->SecExchg;
	g_BtAppData.bSecInit = false;
	g_BtAppData.bScan = false;
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

	// Service setup. Adding the first service starts connection support
	// (BtAppConnInit), which also installs the GAP and GATT services ahead of
	// the application ones.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtAppInitUserServices();

		if (s_pBtAppNrf91Conn == nullptr)
		{
			// Peripheral role without any service. The application has to
			// call BtAppConnInit in BtAppInitUserServices to be connectable,
			// or use BTAPP_ROLE_BROADCASTER.
			return false;
		}

		if (s_pBtAppNrf91Conn->SrvcDone(pCfg) == false)
		{
			return false;
		}
	}

	if (BtAppStackInit(pCfg) == false)
	{
		return false;
	}

	// Device address: a random static address made from the nRF91 device
	// identifier, so it stays the same across resets and does not depend on
	// the controller board. The top two bits of the most significant byte are
	// 1 for a static random address.
	uint32_t id0 = BT_APP_NRF91_FICR->INFO.DEVICEID[0];
	uint32_t id1 = BT_APP_NRF91_FICR->INFO.DEVICEID[1];
	uint8_t ranaddr[6];

	ranaddr[0] = (uint8_t)id0;
	ranaddr[1] = (uint8_t)(id0 >> 8);
	ranaddr[2] = (uint8_t)(id0 >> 16);
	ranaddr[3] = (uint8_t)(id0 >> 24);
	ranaddr[4] = (uint8_t)id1;
	ranaddr[5] = (uint8_t)(((id1 >> 8) & 0x3f) | 0xc0);

	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_SET_RANDOM_ADDR, ranaddr, sizeof(ranaddr), NULL, 0) != 0)
	{
		return false;
	}

	// SMP f5/f6 must use the same local address/type the peer sees.
	memcpy(s_BtSmpLocalAddr, ranaddr, 6);
	s_BtSmpLocalAddrType = 1;	// random

	// LE Read Maximum Data Length return: supported max TX octets, TX time, RX
	// octets, RX time, each 2 bytes little endian.
	uint8_t maxlen[8] = {0};
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN, NULL, 0, maxlen, sizeof(maxlen)) == 0)
	{
		uint16_t maxTxOctets = (uint16_t)(maxlen[0] | (maxlen[1] << 8));
		uint16_t maxTxTime   = (uint16_t)(maxlen[2] | (maxlen[3] << 8));

		uint16_t txOctets = (uint16_t)min(maxTxOctets, pCfg->MaxMtu);
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
	// waits for an explicit reply, which the app layer does not issue.
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

	BtAppInitUserData();

	// Security is not supported on this port yet. A configuration that asks
	// for it must not run unprotected.
	if (pCfg->SecType != BTGAP_SECTYPE_NONE && g_BtAppData.bSecInit == false)
	{
		return false;
	}

	g_BtAppData.AppDevice.bSecure = (pCfg->SecType != BTGAP_SECTYPE_NONE);

	if (pCfg->Role & (BTAPP_ROLE_BROADCASTER | BTAPP_ROLE_PERIPHERAL))
	{
		if (BtAppAdvInit(pCfg) == false)
		{
			return false;
		}
	}

	// The app timer serves the SMP and GATT timeouts of a link, so it is
	// started only with connection support.
	if (s_pBtAppNrf91Conn != nullptr)
	{
		s_pBtAppNrf91Conn->TimerStart();
	}
	s_bBtAppInitDone = true;

	g_BtAppData.State = BTAPP_STATE_INITIALIZED;

	// Advertising starts here and not from the event queue: an application
	// interrupt can fill the queue before the application runs it.
	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		BtAdvStart();
	}

	return true;
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

	// Write without response, matching the other ports
	return BtAttWriteCommand(pPeer->pHciDev, ConnHandle, CharHandle, pData, DataLen);
}
