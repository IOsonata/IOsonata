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

#ifndef NET_DEMO_NATIVE_TLS
#define NET_DEMO_NATIVE_TLS	0
#endif

static_assert(NET_DEMO_NATIVE_TLS == 0 || NET_DEMO_NATIVE_TLS == 1,
	"NET_DEMO_NATIVE_TLS must be 0 or 1");

#if NET_DEMO_NATIVE_TLS
#include <string.h>
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"
#include "net/tls.h"
#include "net_demo_tls_trust.h"
#endif

#ifndef NET_DEMO_MODEM_TLS
#define NET_DEMO_MODEM_TLS	0
#endif

static_assert(NET_DEMO_MODEM_TLS == 0 || NET_DEMO_MODEM_TLS == 1,
	"NET_DEMO_MODEM_TLS must be 0 or 1");

static_assert(!NET_DEMO_NATIVE_TLS || !NET_DEMO_MODEM_TLS,
	"Select either native TLS or modem TLS");

#ifndef NET_DEMO_LTE_RECOVERY_TEST
#define NET_DEMO_LTE_RECOVERY_TEST	0
#endif

static_assert(NET_DEMO_LTE_RECOVERY_TEST == 0 || NET_DEMO_LTE_RECOVERY_TEST == 1,
	"NET_DEMO_LTE_RECOVERY_TEST must be 0 or 1");

#if NET_DEMO_LTE_RECOVERY_TEST && NET_DEMO_MEDIUM != NET_DEMO_MEDIUM_LTE
#error "NET_DEMO_LTE_RECOVERY_TEST requires the LTE medium"
#endif

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
#if NET_DEMO_NATIVE_TLS
alignas(4) static uint8_t s_UartRxMem[CFIFO_MEMSIZE(128)];
#endif
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
#if NET_DEMO_NATIVE_TLS
	.RxMemSize = sizeof(s_UartRxMem),
	.pRxMem = s_UartRxMem,
#endif
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

#if NET_DEMO_NATIVE_TLS
static CryptoSoftSha256 s_TlsHash;
static CryptoSoftAes s_TlsAes;
static CryptoUecc s_TlsEcc;
static Tls s_Tls;
static uint8_t s_TlsHandshake[8192];
static uint8_t s_TlsTranscript[12288];
static uint64_t s_UtcSeconds;
static uint32_t s_UtcTick;
static uint32_t s_UtcRemainder;
static char s_TimeLine[32];
static size_t s_TimeLength;
static bool s_TimeDiscard;
static bool s_UtcValid;
static bool s_TlsOpening;
static bool s_TlsFailed;

static void UpdateUtc(uint32_t Now)
{
	uint64_t elapsed = (uint32_t)(Now - s_UtcTick);

	s_UtcTick = Now;
	if (s_UtcValid)
	{
		elapsed += s_UtcRemainder;
		s_UtcSeconds += elapsed / 1000U;
		s_UtcRemainder = elapsed % 1000U;
	}
}

static void SetConsoleTime()
{
	uint64_t seconds = 0;
	size_t i;

	if (s_TimeDiscard || s_TimeLength <= 5U ||
		memcmp(s_TimeLine, "time ", 5) != 0)
	{
		SysLogPrintf(SysLogGet(), "UTC command rejected; use time <Unix seconds>\r\n");
		return;
	}
	for (i = 5; i < s_TimeLength; i++)
	{
		if (s_TimeLine[i] < '0' || s_TimeLine[i] > '9' ||
			seconds > (253402300799ULL - (uint8_t)(s_TimeLine[i] - '0')) / 10U)
		{
			SysLogPrintf(SysLogGet(), "UTC command rejected; invalid seconds\r\n");
			return;
		}
		seconds = seconds * 10U + (uint8_t)(s_TimeLine[i] - '0');
	}
	if (!seconds || s_pEndpoint)
	{
		SysLogPrintf(SysLogGet(), "UTC command rejected; zero time or active connection\r\n");
		return;
	}
	// The physical console and commissioning PC are the time source for this
	// demo. Reset requires provisioning again; no build date or SNTP fallback.
	s_UtcSeconds = seconds;
	s_UtcTick = NetDemoNow();
	s_UtcRemainder = 0;
	s_UtcValid = true;
	SysLogPrintf(SysLogGet(), "TLS UTC set from console\r\n");
}

static void ProcessConsoleTime(uint32_t Now)
{
	uint8_t value;
	unsigned count;

	UpdateUtc(Now);
	for (count = 0; count < 32U && s_Uart.Rx(&value, 1) == 1; count++)
	{
		if (value == '\r' || value == '\n')
		{
			if (s_TimeLength || s_TimeDiscard)
			{
				SetConsoleTime();
			}
			s_TimeLength = 0;
			s_TimeDiscard = false;
		}
		else if (value < ' ' || value > '~' || s_TimeLength == sizeof(s_TimeLine))
		{
			s_TimeDiscard = true;
		}
		else if (!s_TimeDiscard)
		{
			s_TimeLine[s_TimeLength++] = (char)value;
		}
	}
}

static bool PrepareNativeTls()
{
	const char *name = NET_DEMO_TLS_SERVER_NAME;

	if (!g_NetDemoTlsTrustAnchorCount || !name || !name[0])
	{
		SysLogPrintf(SysLogGet(), "Native TLS configuration missing: generate CA header and set server name\r\n");
		return false;
	}
	s_TlsEcc.SetRng(s_pRng);
	if (!s_TlsHash.Enable() || !s_TlsAes.Enable() || !s_TlsEcc.Enable())
	{
		SysLogPrintf(SysLogGet(), "Native TLS crypto initialization failed\r\n");
		return false;
	}
	s_UtcTick = NetDemoNow();
	SysLogPrintf(SysLogGet(), "Native TLS waiting for UTC: time <Unix seconds>\r\n");
	return true;
}

static bool OpenNativeTls(DevIntrf_t *pTransport)
{
	TlsCertCfg_t cert = {
		.pSign = &s_TlsEcc,
		.pTrustAnchors = g_NetDemoTlsTrustAnchors,
		.TrustAnchorCount = g_NetDemoTlsTrustAnchorCount,
		.UnixTime = 0,
		.pHandshakeBuffer = s_TlsHandshake,
		.HandshakeBufferSize = sizeof(s_TlsHandshake),
		.pTranscriptBuffer = s_TlsTranscript,
		.TranscriptBufferSize = sizeof(s_TlsTranscript),
	};
	TlsCfg_t cfg = {
		.pHash = &s_TlsHash,
		.pMac = &s_TlsHash,
		.pAead = &s_TlsAes,
		.pRng = s_pRng,
		.pPsk = nullptr,
		.PskLen = 0,
		.pIdentity = nullptr,
		.IdentityLen = 0,
		.pServerName = NET_DEMO_TLS_SERVER_NAME,
		.Timeout = 30000U,
		.pKeyAgree = &s_TlsEcc,
		.pCert = &cert,
	};
	uint32_t now = NetDemoNow();

	UpdateUtc(now);
	cert.UnixTime = s_UtcSeconds;
	s_TlsFailed = false;
	s_TlsOpening = s_Tls.Init(cfg, pTransport, now);
	if (!s_TlsOpening)
	{
		SysLogPrintf(SysLogGet(), "Native TLS initialization failed, error=%u certificate_error=%d\r\n",
			(unsigned)s_Tls.Error(), (int)s_Tls.CertificateError());
	}
	return s_TlsOpening;
}
#endif

#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
static void LteEvent(LTE_EVT Evt, const LteStatus_t * const pStatus);

#if NET_DEMO_LTE_RECOVERY_TEST
enum LTE_RECOVERY_PHASE {
	LTE_RECOVERY_WAIT_ENDPOINT,
	LTE_RECOVERY_WAIT_DISCONNECT,
	LTE_RECOVERY_RADIO_OFF,
	LTE_RECOVERY_RETRY_CONNECT,
	LTE_RECOVERY_WAIT_REGISTRATION,
	LTE_RECOVERY_DONE,
};

static LTE_RECOVERY_PHASE s_LteRecoveryPhase;
static uint32_t s_LteRecoveryTime;
static bool s_LteRecoverySawDown;

static bool LteRecoveryGated()
{
	return s_LteRecoveryPhase == LTE_RECOVERY_RADIO_OFF ||
		s_LteRecoveryPhase == LTE_RECOVERY_RETRY_CONNECT ||
		s_LteRecoveryPhase == LTE_RECOVERY_WAIT_REGISTRATION;
}
#endif

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
#elif NET_DEMO_NATIVE_TLS
	return "TLS 1.3, CA/hostname/time verified";
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
#if NET_DEMO_NATIVE_TLS
	if (!s_UtcValid)
	{
		return false;
	}
#endif
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
#if NET_DEMO_LTE_RECOVERY_TEST
	if (LteRecoveryGated())
	{
		return false;
	}
#endif
	return LteRegistered();
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
	return IpReady();
#else
	return NetDemoCustomReady();
#endif
}

static bool TransportConnected()
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

bool NetDemoOpening()
{
#if NET_DEMO_NATIVE_TLS
	return s_pEndpoint && NetDemoReady() && s_TlsOpening;
#else
	return false;
#endif
}

bool NetDemoConnected()
{
#if NET_DEMO_NATIVE_TLS
	return s_pEndpoint && NetDemoReady() && s_Tls.Connected();
#else
	return TransportConnected();
#endif
}

void NetDemoClose()
{
	if (!s_pEndpoint)
	{
		return;
	}
#if NET_DEMO_NATIVE_TLS
	s_Tls.Reset();
	s_TlsOpening = false;
#endif
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_CUSTOM
	NetDemoCustomClose();
#else
	s_Socket.Close();
#endif
	s_pEndpoint = nullptr;
#if NET_DEMO_LTE_RECOVERY_TEST
	if (s_LteRecoveryPhase == LTE_RECOVERY_WAIT_DISCONNECT)
	{
		s_LteRecoveryPhase = LTE_RECOVERY_WAIT_ENDPOINT;
	}
#endif
}

DeviceIntrf *NetDemoOpen(bool Datagram, const char *pHost, uint16_t Port)
{
	DevIntrf_t *dev;

	if (s_pEndpoint || !NetDemoReady() || !pHost || !pHost[0] || !Port)
	{
		return nullptr;
	}
#if NET_DEMO_MODEM_TLS || NET_DEMO_NATIVE_TLS
	if (Datagram)
	{
		SysLogPrintf(SysLogGet(), "TLS demo requires a stream interface\r\n");
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
#if NET_DEMO_NATIVE_TLS
	if (!OpenNativeTls(dev))
	{
		NetDemoClose();
		return nullptr;
	}
	return &s_Tls;
#else
	return s_pEndpoint;
#endif
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
#if NET_DEMO_LTE_RECOVERY_TEST
			if (s_LteRecoveryPhase == LTE_RECOVERY_WAIT_REGISTRATION)
			{
				// Only a new event after radio-on releases the readiness gate.
				s_LteRecoveryPhase = LTE_RECOVERY_DONE;
				SysLogPrintf(SysLogGet(), "LTE recovery test: registration restored; waiting for protocol recovery\r\n");
			}
#endif
			break;
		case LTE_EVT_UNREGISTERED:
#if NET_DEMO_LTE_RECOVERY_TEST
			if (LteRecoveryGated())
			{
				s_LteRecoverySawDown = true;
			}
#endif
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
#if NET_DEMO_LTE_RECOVERY_TEST
			if (LteRecoveryGated())
			{
				s_LteRecoveryPhase = LTE_RECOVERY_DONE;
				SysLogPrintf(SysLogGet(), "LTE recovery test: aborted by modem fault\r\n");
			}
#endif
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

#if NET_DEMO_LTE_RECOVERY_TEST
static void LteRecoveryProcess(uint32_t Now)
{
	switch (s_LteRecoveryPhase)
	{
		case LTE_RECOVERY_WAIT_ENDPOINT:
		case LTE_RECOVERY_WAIT_DISCONNECT:
			if (!NetDemoConnected())
			{
				s_LteRecoveryPhase = LTE_RECOVERY_WAIT_ENDPOINT;
				return;
			}
			if (s_LteRecoveryPhase == LTE_RECOVERY_WAIT_ENDPOINT)
			{
				s_LteRecoveryPhase = LTE_RECOVERY_WAIT_DISCONNECT;
				s_LteRecoveryTime = Now;
				SysLogPrintf(SysLogGet(), "LTE recovery test: radio off in 120 s of connected transport\r\n");
				return;
			}
			if ((uint32_t)(Now - s_LteRecoveryTime) < 120000U)
			{
				return;
			}
			// Consume the one-shot before issuing the command. The ordinary
			// network-loss path closes the endpoint after radio-off succeeds.
			s_LteRecoveryPhase = LTE_RECOVERY_DONE;
			s_LteRecoverySawDown = false;
			if (!LteDisconnect())
			{
				SysLogPrintf(SysLogGet(), "LTE recovery test: radio-off failed; test aborted\r\n");
				return;
			}
			s_LteRecoveryTime = NetDemoNow();
			s_LteRecoveryPhase = LTE_RECOVERY_RADIO_OFF;
			SysLogPrintf(SysLogGet(), "LTE recovery test: radio off for 60 s\r\n");
			return;
		case LTE_RECOVERY_RADIO_OFF:
			if ((uint32_t)(Now - s_LteRecoveryTime) < 60000U)
			{
				return;
			}
			if (!s_LteRecoverySawDown)
			{
				SysLogPrintf(SysLogGet(), "LTE recovery test: offline notification missing; loss unverified\r\n");
			}
			break;
		case LTE_RECOVERY_RETRY_CONNECT:
			if ((uint32_t)(Now - s_LteRecoveryTime) < NET_DEMO_RESTART_MS)
			{
				return;
			}
			break;
		case LTE_RECOVERY_WAIT_REGISTRATION:
			if ((uint32_t)(Now - s_LteRecoveryTime) >= 180000U)
			{
				// Do not reopen from a stale cached registration or leave the
				// test's gate stuck forever if a notification never arrives.
				s_LteRecoveryPhase = LTE_RECOVERY_DONE;
				s_MediumStarted = false;
				s_MediumRetry = NetDemoNow();
				SysLogPrintf(SysLogGet(), "LTE recovery test: registration unverified; LTE reinitialization in 30 s\r\n");
			}
			return;
		default:
			return;
	}
	// Both the radio-off hold and failed attach retries keep the foreground
	// queue running. Command acceptance is not network registration.
	bool started = LteConnect();

	s_LteRecoveryTime = NetDemoNow();
	if (started)
	{
		s_LteRecoveryPhase = LTE_RECOVERY_WAIT_REGISTRATION;
		SysLogPrintf(SysLogGet(), "LTE recovery test: radio on; waiting for registration\r\n");
	}
	else
	{
		s_LteRecoveryPhase = LTE_RECOVERY_RETRY_CONNECT;
		SysLogPrintf(SysLogGet(), "LTE recovery test: radio-on failed; retry in 30 s\r\n");
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
#if NET_DEMO_NATIVE_TLS
	TLS_STATE state;
#endif

	(void)Evt;
	(void)pCtx;
	s_PumpQueued = false;
	if (!s_MediumStarted && (uint32_t)(now - s_MediumRetry) >= NET_DEMO_RESTART_MS)
	{
		StartMedium();
	}
#if NET_DEMO_LTE_RECOVERY_TEST
	LteRecoveryProcess(NetDemoNow());
#endif
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
#if NET_DEMO_NATIVE_TLS
	ProcessConsoleTime(NetDemoNow());
	if (s_pEndpoint)
	{
		state = s_Tls.Process(NetDemoNow(), TransportConnected());

		if (state == TLS_READY)
		{
			s_TlsOpening = false;
		}
		else if (state == TLS_FAILED || state == TLS_CLOSED)
		{
			s_TlsOpening = false;
			if (!s_TlsFailed)
			{
				s_TlsFailed = true;
				SysLogPrintf(SysLogGet(), "Native TLS ended, error=%u certificate_error=%d\r\n",
					(unsigned)s_Tls.Error(), (int)s_Tls.CertificateError());
			}
		}
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
#if NET_DEMO_NATIVE_TLS
	// Poll at most 32 console bytes per foreground pump without waiting.
	((DevIntrf_t *)s_Uart)->MaxRetry = 0;
#endif
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
#if NET_DEMO_NATIVE_TLS
	if (!PrepareNativeTls())
	{
		return false;
	}
#endif
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
