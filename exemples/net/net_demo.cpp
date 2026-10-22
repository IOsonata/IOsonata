/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Network setup for the generic MQTT and CoAP examples.
 */
#include "app_evt_handler.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "syslog.h"
#include "net_demo.h"
#include "board.h"
#include NET_DEMO_RNG_HEADER

#ifndef NET_DEMO_MODEM_TLS
#define NET_DEMO_MODEM_TLS	0
#endif

static_assert(NET_DEMO_MODEM_TLS == 0 || NET_DEMO_MODEM_TLS == 1,
	"NET_DEMO_MODEM_TLS must be 0 or 1");

#if NET_DEMO_MODEM_TLS
#if NET_DEMO_MEDIUM != NET_DEMO_MEDIUM_LTE
#error "NET_DEMO_MODEM_TLS requires the LTE medium"
#endif
#ifndef NET_DEMO_TLS_REPLACE_CA
#define NET_DEMO_TLS_REPLACE_CA	0
#endif
static_assert(NET_DEMO_TLS_REPLACE_CA == 0 || NET_DEMO_TLS_REPLACE_CA == 1,
	"NET_DEMO_TLS_REPLACE_CA must be 0 or 1");
static_assert(NET_DEMO_TLS_SEC_TAG >= 0 && NET_DEMO_TLS_SEC_TAG <= 2147483647,
	"NET_DEMO_TLS_SEC_TAG must be a nonnegative int");
#endif

#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
#include "lte/lte.h"
#include "net/sock_intrf.h"
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
#include "net/ethernet.h"
#include "net/ip.h"
#include "net/sock_intrf.h"
#elif NET_DEMO_MEDIUM != NET_DEMO_MEDIUM_CUSTOM
#error "Select LTE, ETHERNET or CUSTOM with NET_DEMO_MEDIUM"
#endif

#define NET_DEMO_TRIG_NETWORK	0
#define NET_DEMO_TRIG_PUMP	1
#define NET_DEMO_RESTART_MS	30000U

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
static const IOPinCfg_t s_UartPins[] = NET_DEMO_UART_PINS;
alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(1024)];
static const UARTCfg_t s_UartCfg = {
	.DevNo = NET_DEMO_UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = NET_DEMO_UART_RATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = NET_DEMO_INT_PRIO,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = NET_DEMO_UART_DMA,
};
static const TimerCfg_t s_TimerCfg = {
	.DevNo = NET_DEMO_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = NET_DEMO_TIMER_FREQ,
	.IntPrio = NET_DEMO_INT_PRIO,
};
alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 192)];
static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = 192,
	.bBlocking = false,
};
static UART s_Uart;
static TimerDev_t s_Timer;
static RngEngine *s_pRng;
static DeviceIntrf *s_pEndpoint;
static void (*s_pHandler)(uint32_t Now);
static uint32_t s_MediumRetry;
static bool s_Initialized;
static bool s_PumpStarted;
static bool s_MediumStarted;
static bool s_WasReady;
static volatile bool s_PumpQueued;

#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
static void LteEvent(LTE_EVT Evt, const LteStatus_t * const pStatus);

static const LteCfg_t s_LteCfg = {
	.Rat = NET_DEMO_RAT,
	.RatPref = NET_DEMO_RAT_PREF,
	.bGnss = false,
	.pBand = nullptr,
	.NbBand = 0,
	.pApn = NET_DEMO_APN,
	.PdnType = LTE_PDN_IPV4V6,
	.bPsm = false,
	.PsmTau = 0,
	.PsmActive = 0,
	.EdrxCycle = 0,
	.bRai = false,
	.IntPrio = NET_DEMO_INT_PRIO,
	.pTimer = &s_Timer,
	.TimerTrigNo = NET_DEMO_TRIG_NETWORK,
	.UrcMemSize = 0,
	.EvtHandler = LteEvent,
	.UrcHandler = nullptr,
};
static SockIntrf s_Socket;
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
static const IOPinCfg_t s_EthernetPins[] = ETHERNET_PINS;
static const EthernetCfg_t s_EthernetCfg = {
	.DevNo = 0,
	.pIOPinMap = s_EthernetPins,
	.NbIOPins = sizeof(s_EthernetPins) / sizeof(s_EthernetPins[0]),
	.MacAddr = ETHERNET_MAC_ADDR,
	.PhyAddr = ETHERNET_PHY_ADDR,
	.IntPrio = NET_DEMO_INT_PRIO,
	.bIntEn = true,
	.EvtCB = nullptr,
};
static const IpCfg_t s_IpCfg = {
	.Addr = NET_DEMO_IP_ADDR,
	.Netmask = NET_DEMO_NETMASK,
	.Gateway = NET_DEMO_GATEWAY,
	.Dns = NET_DEMO_DNS,
	.bDhcp = NET_DEMO_DHCP,
};
static Ethernet s_Ethernet;
static SockIntrf s_Socket;
#endif

uint32_t NetDemoNow()
{
	return (uint32_t)TimerGetMilisecond(&s_Timer);
}

RngEngine *NetDemoRng()
{
	return s_pRng;
}

const char *NetDemoMediumName()
{
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
	return "LTE";
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
	return "Ethernet";
#elif defined(NET_DEMO_MEDIUM_NAME)
	return NET_DEMO_MEDIUM_NAME;
#else
	return "custom network";
#endif
}

const char *NetDemoSecurityName()
{
#if NET_DEMO_MODEM_TLS
	return "TLS 1.2, CA/hostname verified";
#else
	return "plaintext";
#endif
}

bool NetDemoReady()
{
	if (!s_MediumStarted)
	{
		return false;
	}
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
	return LteRegistered();
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
	return IpReady();
#else
	return NetDemoCustomReady();
#endif
}

bool NetDemoConnected()
{
	if (!s_pEndpoint || !NetDemoReady())
	{
		return false;
	}
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_CUSTOM
	return NetDemoCustomConnected();
#else
	return s_Socket.Connected();
#endif
}

void NetDemoClose()
{
	if (!s_pEndpoint)
	{
		return;
	}
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_CUSTOM
	NetDemoCustomClose();
#else
	s_Socket.Close();
#endif
	s_pEndpoint = nullptr;
}

DeviceIntrf *NetDemoOpen(bool Datagram, const char *pHost, uint16_t Port)
{
	DevIntrf_t *dev;

	if (s_pEndpoint || !NetDemoReady() || !pHost || !pHost[0] || !Port)
	{
		return nullptr;
	}
#if NET_DEMO_MODEM_TLS
	if (Datagram)
	{
		SysLogPrintf(SysLogGet(), "Modem TLS demo requires a stream interface\r\n");
		return nullptr;
	}
#endif
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_CUSTOM
	s_pEndpoint = NetDemoCustomOpen(Datagram, pHost, Port);
	if (!s_pEndpoint || !NetDemoCustomConnected())
	{
		NetDemoCustomClose();
		s_pEndpoint = nullptr;
		return nullptr;
	}
#else
	SockIntrfCfg_t cfg = {
#if NET_DEMO_MODEM_TLS
		.Proto = SOCKINTRF_PROTO_TLS,
#else
		.Proto = Datagram ? SOCKINTRF_PROTO_UDP : SOCKINTRF_PROTO_TCP,
#endif
		.pHost = pHost,
		.Port = Port,
		.LocalPort = 0,
#if NET_DEMO_MODEM_TLS
		.SecTag = NET_DEMO_TLS_SEC_TAG,
		.bPeerVerify = true,
#else
		.SecTag = -1,
		.bPeerVerify = false,
#endif
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
		// The nRF91 socket port gives DNS/connect their own wait.
		.Timeout = 100,
#else
		// lwIP uses this wait for DNS and connect as well as stream TX.
		.Timeout = 10000,
#endif
		.EvtCB = nullptr,
	};

	if (!s_Socket.Init(cfg) || !s_Socket.Connected())
	{
		s_Socket.Close();
		return nullptr;
	}
	s_pEndpoint = &s_Socket;
#endif
	dev = *s_pEndpoint;
	if (!dev)
	{
		NetDemoClose();
		return nullptr;
	}
	// Protocol pumps control retry timing and preserve datagram boundaries.
	dev->MaxRetry = 0;
	return s_pEndpoint;
}

#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
static void LteEvent(LTE_EVT Evt, const LteStatus_t * const pStatus)
{
	// The LTE core dispatches this callback from the foreground event queue.
	switch (Evt)
	{
		case LTE_EVT_REGISTERED:
			SysLogPrintf(SysLogGet(), "Registered %s, %s\r\n",
				pStatus->RegStat == LTE_REG_ROAMING ? "roaming" : "home",
				pStatus->Rat == LTE_RAT_NBIOT ? "NB-IoT" : "LTE-M");
			break;
		case LTE_EVT_UNREGISTERED:
			// Close now even if registration returns before the next pump.
			NetDemoClose();
			SysLogPrintf(SysLogGet(), "Unregistered, state %u\r\n", (unsigned)pStatus->RegStat);
			break;
		case LTE_EVT_REG_STATE:
			SysLogPrintf(SysLogGet(), "Registration state %u\r\n", (unsigned)pStatus->RegStat);
			break;
		case LTE_EVT_RRC_CONNECTED:
			SysLogPrintf(SysLogGet(), "RRC connected\r\n");
			break;
		case LTE_EVT_RRC_IDLE:
			SysLogPrintf(SysLogGet(), "RRC idle\r\n");
			break;
		case LTE_EVT_MODEM_FAULT:
			NetDemoClose();
			s_MediumStarted = false;
			s_MediumRetry = NetDemoNow();
			SysLogPrintf(SysLogGet(), "Modem fault, restart in 30 s\r\n");
			break;
		default:
			break;
	}
}
#endif

#if NET_DEMO_MODEM_TLS
static bool PrepareModemTls()
{
	const char *ca = NET_DEMO_TLS_CA_CERT;
	bool present;

	// LteInit starts attachment. Stop it before touching persistent credentials,
	// retaining the LTE port's modem initialization and fault ownership.
	if (!LteDisconnect())
	{
		SysLogPrintf(SysLogGet(), "TLS credential setup: LTE radio-off failed\r\n");
		return false;
	}
	present = SockIntrfCredExists(NET_DEMO_TLS_SEC_TAG, SOCKINTRF_CRED_CA_CERT);
	if (!present || NET_DEMO_TLS_REPLACE_CA)
	{
		if (!ca || !ca[0])
		{
			SysLogPrintf(SysLogGet(), "TLS credential setup: CA required for tag %d\r\n",
				(int)NET_DEMO_TLS_SEC_TAG);
			return false;
		}
		if (!SockIntrfCredWrite(NET_DEMO_TLS_SEC_TAG, SOCKINTRF_CRED_CA_CERT, ca) ||
			!SockIntrfCredExists(NET_DEMO_TLS_SEC_TAG, SOCKINTRF_CRED_CA_CERT))
		{
			SysLogPrintf(SysLogGet(), "TLS credential setup: CA write/check failed for tag %d\r\n",
				(int)NET_DEMO_TLS_SEC_TAG);
			return false;
		}
		SysLogPrintf(SysLogGet(), "TLS CA stored under tag %d\r\n", (int)NET_DEMO_TLS_SEC_TAG);
	}
	else
	{
		SysLogPrintf(SysLogGet(), "TLS using stored CA under tag %d\r\n", (int)NET_DEMO_TLS_SEC_TAG);
	}
	if (!LteConnect())
	{
		SysLogPrintf(SysLogGet(), "TLS credential setup: LTE reattach failed\r\n");
		return false;
	}
	return true;
}
#endif

static void StartMedium()
{
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
	char info[48];
	bool started;

	// Do not expose a stale registered state during credential provisioning.
	s_MediumStarted = false;
	started = LteInit(&s_LteCfg);
	if (started)
	{
		if (LteGetInfo(LTE_INFO_FWVER, info, sizeof(info)))
		{
			SysLogPrintf(SysLogGet(), "Modem firmware %s\r\n", info);
		}
	}
#if NET_DEMO_MODEM_TLS
	if (started)
	{
		started = PrepareModemTls();
	}
#endif
	s_MediumStarted = started;
	if (s_MediumStarted)
	{
		SysLogPrintf(SysLogGet(), "Attaching; PSM, eDRX and RAI disabled for this test\r\n");
	}
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
	s_MediumStarted = s_Ethernet.Init(s_EthernetCfg);
	if (s_MediumStarted)
	{
		s_MediumStarted = IpInit(&s_IpCfg, (EthernetDev_t *)s_Ethernet,
			&s_Timer, NET_DEMO_TRIG_NETWORK);
		if (!s_MediumStarted)
		{
			EthernetClose((EthernetDev_t *)s_Ethernet);
		}
	}
	if (s_MediumStarted)
	{
		SysLogPrintf(SysLogGet(), "Ethernet started, waiting for IPv4\r\n");
	}
#else
	s_MediumStarted = NetDemoCustomInit(&s_Timer);
#endif
	// Setup can block. Each retry waits from its completion, including wrap.
	s_MediumRetry = NetDemoNow();
	if (!s_MediumStarted)
	{
		SysLogPrintf(SysLogGet(), "Network initialization failed, retry in 30 s\r\n");
	}
}

static void Pump(uint32_t Evt, void *pCtx)
{
	uint32_t now = NetDemoNow();
	bool ready;

	(void)Evt;
	(void)pCtx;
	s_PumpQueued = false;
	if (!s_MediumStarted && (uint32_t)(now - s_MediumRetry) >= NET_DEMO_RESTART_MS)
	{
		StartMedium();
	}
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
	if (s_MediumStarted)
	{
		IpProcess();
	}
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_CUSTOM
	if (s_MediumStarted)
	{
		NetDemoCustomProcess(NetDemoNow());
	}
#endif
	ready = NetDemoReady();
	if (!ready)
	{
		NetDemoClose();
	}
	if (ready != s_WasReady)
	{
		s_WasReady = ready;
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
		uint8_t addr[4];

		if (ready && IpAddress(addr))
		{
			SysLogPrintf(SysLogGet(), "Ethernet IPv4 %u.%u.%u.%u\r\n",
				addr[0], addr[1], addr[2], addr[3]);
		}
		else
#endif
		{
			SysLogPrintf(SysLogGet(), "Network %s\r\n", ready ? "ready" : "down");
		}
	}
	// Always notify the application, including a lost network or failed init.
	// It owns protocol state, request deadlines and reconnection pacing.
	s_pHandler(NetDemoNow());
}

static void PumpTimer(TimerDev_t *pTimer, int TrigNo, void *pCtx)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pCtx;
	if (!s_PumpQueued)
	{
		// Coalesce timer ticks. A full queue is retried on the next tick.
		s_PumpQueued = AppEvtHandlerQue(0, nullptr, Pump);
	}
}

bool NetDemoInit()
{
	if (s_Initialized)
	{
		return true;
	}
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!s_Uart.Init(s_UartCfg))
	{
		return false;
	}
	if (!SysLogInit(SysLogGet(), &s_LogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0))
	{
		return false;
	}
	if (!TimerInit(&s_Timer, &s_TimerCfg))
	{
		SysLogPrintf(SysLogGet(), "Timer initialization failed\r\n");
		return false;
	}
	s_pRng = NET_DEMO_RNG();
	if (!s_pRng || !s_pRng->Enable())
	{
		SysLogPrintf(SysLogGet(), "Random generator initialization failed\r\n");
		return false;
	}
	s_Initialized = true;
	return true;
}

bool NetDemoStart(void (*pHandler)(uint32_t Now))
{
	if (!s_Initialized || !pHandler || s_PumpStarted)
	{
		return false;
	}
	s_pHandler = pHandler;
	if (!msTimerEnableTrigger(&s_Timer, NET_DEMO_TRIG_PUMP, 20,
		TIMER_TRIG_TYPE_CONTINUOUS, PumpTimer, nullptr))
	{
		s_pHandler = nullptr;
		SysLogPrintf(SysLogGet(), "Application timer initialization failed\r\n");
		return false;
	}
	s_PumpStarted = true;
	StartMedium();
	return true;
}
