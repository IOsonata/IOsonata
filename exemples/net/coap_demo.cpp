/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * CoAP client interoperability and recovery over Ethernet UDP.
 */
#include <string.h>
#include "app_evt_handler.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "net/coap.h"
#include "net/ethernet.h"
#include "net/ip.h"
#include "net/sock_intrf.h"
#include "syslog.h"
#include "board.h"
#include COAP_DEMO_RNG_HEADER

typedef enum
{
	DEMO_EMPTY,
	DEMO_HELLO,
	DEMO_BINARY,
	DEMO_SLOW
} DEMO_RESPONSE;

typedef struct
{
	COAP_TYPE Type;
	uint8_t Method;
	const char *pPath;
	uint8_t Code;
	DEMO_RESPONSE Response;
	bool bPayload;
} CoapDemoStep_t;

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
static const IOPinCfg_t s_EthernetPins[] = ETHERNET_PINS;
static const IOPinCfg_t s_UartPins[] = UART_PINS;
static const EthernetCfg_t s_EthernetCfg = {
	.DevNo = 0,
	.pIOPinMap = s_EthernetPins,
	.NbIOPins = sizeof(s_EthernetPins) / sizeof(s_EthernetPins[0]),
	.MacAddr = ETHERNET_MAC_ADDR,
	.PhyAddr = ETHERNET_PHY_ADDR,
	.IntPrio = 6,
	.bIntEn = true,
	.EvtCB = nullptr,
};
alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(1024)];
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};
static const TimerCfg_t s_TimerCfg = {
	.DevNo = ETHERNET_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 100000,
	.IntPrio = 6,
};
static const IpCfg_t s_IpCfg = {
	.Addr = {192, 168, 1, 50},
	.Netmask = {255, 255, 255, 0},
	.Gateway = {192, 168, 1, 1},
	.Dns = {192, 168, 1, 1},
	.bDhcp = true,
};
static const SockIntrfCfg_t s_SockCfg = {
	.Proto = SOCKINTRF_PROTO_UDP,
	.pHost = COAP_DEMO_HOST,
	.Port = COAP_DEMO_PORT,
	.LocalPort = 0,
	.SecTag = 0,
	.bPeerVerify = false,
	.Timeout = 1000,
	.EvtCB = nullptr,
};
static const CoapDemoStep_t s_Steps[] = {
	{COAP_CON, COAP_GET, "hello", COAP_CONTENT, DEMO_HELLO, false},
	{COAP_CON, COAP_POST, "echo", COAP_CONTENT, DEMO_BINARY, true},
	{COAP_NON, COAP_POST, "echo", COAP_CONTENT, DEMO_BINARY, true},
	{COAP_CON, COAP_PUT, "value", COAP_CHANGED, DEMO_EMPTY, true},
	{COAP_CON, COAP_GET, "value", COAP_CONTENT, DEMO_BINARY, false},
	{COAP_CON, COAP_DELETE, "value", COAP_DELETED, DEMO_EMPTY, false},
	{COAP_CON, COAP_GET, "value", COAP_NOT_FOUND, DEMO_EMPTY, false},
	{COAP_CON, COAP_GET, "slow", COAP_CONTENT, DEMO_SLOW, false},
};
static const uint8_t s_Hello[] = "IOsonata CoAP\n";
static const uint8_t s_Slow[] = "separate response\n";
static const uint8_t s_BinaryFormat[] = {42};
static const size_t s_Lengths[] = {1, 16, 257, 900};
static Ethernet s_Ethernet;
static UART s_Uart;
static TimerDev_t s_Timer;
static SockIntrf s_Socket;
static Coap s_Coap;
static RngEngine *s_pRng;
static uint8_t s_Payload[900];
static size_t s_PayloadLen;
static size_t s_Step;
static uint32_t s_Cycle;
static uint32_t s_Checked;
static uint32_t s_Failures;
static uint32_t s_Retry;
static uint32_t s_ProbeStarted;
static uint32_t s_ProbeDelay;
static uint32_t s_LastRequest;
static bool s_Started;
static bool s_Waiting;
static bool s_RecoveryWaiting;
static bool s_WasReady;
static volatile bool s_ReportQueued;
alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 128)];
static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = 128,
	.bBlocking = false,
};

static void CoapDown(uint32_t Now, const char *pReason)
{
	bool keep = s_Started && IpReady() && s_Socket.Connected() &&
		s_Coap.Error() != COAP_ERROR_TRANSPORT;

	s_Failures++;
	SysLogPrintf(SysLogGet(), "CoAP FAIL: %s, error=%u checked=%lu failures=%lu\r\n",
		pReason, (unsigned)s_Coap.Error(), (unsigned long)s_Checked,
		(unsigned long)s_Failures);
	s_ProbeDelay = s_Coap.RetryAfter(Now);
	s_ProbeStarted = Now;
	if (keep)
	{
		// Preserve peer IDs, duplicates and unanswered-traffic rate state.
		s_Coap.Release();
	}
	else
	{
		// Keep the remaining peer pacing delay across endpoint replacement.
		s_Coap.Reset();
		s_Socket.Close();
		s_Started = false;
	}
	s_Waiting = false;
	s_Step = 0;
	s_Retry = Now;
	s_RecoveryWaiting = true;
}

static bool CheckResponse(const CoapMessage_t *pResponse)
{
	const CoapDemoStep_t *step = &s_Steps[s_Step];
	const uint8_t *expected = nullptr;
	size_t len = 0;

	if (step->Response == DEMO_HELLO)
	{
		expected = s_Hello;
		len = sizeof(s_Hello) - 1U;
	}
	else if (step->Response == DEMO_BINARY)
	{
		expected = s_Payload;
		len = s_PayloadLen;
	}
	else if (step->Response == DEMO_SLOW)
	{
		expected = s_Slow;
		len = sizeof(s_Slow) - 1U;
		// The supplied aiocoap server sends an empty ACK, then a separate CON.
		if (pResponse->Type != COAP_CON)
		{
			return false;
		}
	}
	return pResponse->Code == step->Code && pResponse->PayloadLen == len &&
		(len == 0 || memcmp(pResponse->pPayload, expected, len) == 0);
}

static void StartRequest(uint32_t Now)
{
	const CoapDemoStep_t *step = &s_Steps[s_Step];
	CoapOption_t options[2] = {};
	CoapRequest_t request = {};
	uint32_t pattern;
	size_t i;

	if (s_Step == 1)
	{
		s_PayloadLen = s_Lengths[s_Cycle % (sizeof(s_Lengths) / sizeof(s_Lengths[0]))];
		pattern = s_Cycle + 1U;
		for (i = 0; i < s_PayloadLen; i++)
		{
			pattern = pattern * 1664525U + 1013904223U;
			s_Payload[i] = (uint8_t)(pattern >> 24);
		}
	}
	options[0].Number = COAP_OPTION_URI_PATH;
	options[0].pValue = (const uint8_t *)step->pPath;
	options[0].Len = strlen(step->pPath);
	request.Type = step->Type;
	request.Code = step->Method;
	request.pOptions = options;
	request.OptionCount = 1;
	if (step->bPayload)
	{
		options[1].Number = COAP_OPTION_CONTENT_FORMAT;
		options[1].pValue = s_BinaryFormat;
		options[1].Len = sizeof(s_BinaryFormat);
		request.OptionCount = 2;
		request.pPayload = s_Payload;
		request.PayloadLen = s_PayloadLen;
	}
	if (!s_Coap.Request(request, Now))
	{
		if (s_Coap.Error() == COAP_ERROR_RATE)
		{
			s_ProbeDelay = s_Coap.RetryAfter(Now);
			s_ProbeStarted = Now;
			return;
		}
		CoapDown(Now, "request rejected");
		return;
	}
	s_Waiting = true;
	s_LastRequest = Now;
}

static void CoapTest(uint32_t Now)
{
	const CoapCfg_t cfg = {s_pRng, 95000, COAP_DEMO_PROBING_RATE};
	const CoapMessage_t *response;
	DevIntrf_t *transport;
	COAP_STATE state;

	if (s_ProbeDelay && (uint32_t)(Now - s_ProbeStarted) >= s_ProbeDelay)
	{
		s_ProbeDelay = 0;
	}
	if (s_RecoveryWaiting && (uint32_t)(Now - s_Retry) >= 3000U)
	{
		s_RecoveryWaiting = false;
	}
	if (!s_Started)
	{
		if (s_RecoveryWaiting || s_ProbeDelay)
		{
			return;
		}
		if (!s_Socket.Init(s_SockCfg))
		{
			CoapDown(TimerGetMilisecond(&s_Timer), "UDP open");
			return;
		}
		transport = s_Socket;
		transport->MaxRetry = 0;
		Now = TimerGetMilisecond(&s_Timer);
		if (!s_Coap.Init(cfg, &s_Socket, Now))
		{
			CoapDown(Now, "initialization");
			return;
		}
		s_Started = true;
		SysLogPrintf(SysLogGet(), "CoAP UDP peer %s:%u, plaintext, one request/second\r\n",
			s_SockCfg.pHost, (unsigned)s_SockCfg.Port);
	}
	state = s_Coap.Process(Now, s_Socket.Connected());
	if (state == COAP_FAILED)
	{
		CoapDown(Now, "exchange");
		return;
	}
	response = s_Coap.Response();
	if (response)
	{
		if (!s_Waiting || !CheckResponse(response))
		{
			CoapDown(Now, "response code, type or payload mismatch");
			return;
		}
		s_Checked++;
		if (s_Checked == 1 || s_Checked % 100U == 0)
		{
			SysLogPrintf(SysLogGet(), "CoAP checked %lu failures=%lu\r\n",
				(unsigned long)s_Checked, (unsigned long)s_Failures);
		}
		s_Coap.Release();
		s_Waiting = false;
		s_Step++;
		if (s_Step == sizeof(s_Steps) / sizeof(s_Steps[0]))
		{
			s_Step = 0;
			s_Cycle++;
			if (s_Cycle == 1)
			{
				SysLogPrintf(SysLogGet(), "CoAP methods, CON/NON and separate response verified\r\n");
			}
		}
	}
	if (!s_Waiting && !s_Coap.Busy() && !s_RecoveryWaiting && !s_ProbeDelay &&
		s_Coap.RetryAfter(Now) == 0 && (uint32_t)(Now - s_LastRequest) >= 1000U)
	{
		StartRequest(Now);
	}
}

static void Report(uint32_t Evt, void *pCtx)
{
	uint8_t addr[4];
	bool ready = IpReady();
	uint32_t now = TimerGetMilisecond(&s_Timer);

	(void)Evt;
	(void)pCtx;
	s_ReportQueued = false;
	if (ready != s_WasReady)
	{
		s_WasReady = ready;
		if (ready && IpAddress(addr))
		{
			SysLogPrintf(SysLogGet(), "IPv4 %u.%u.%u.%u, link %lu Mbit/s\r\n",
				addr[0], addr[1], addr[2], addr[3],
				(unsigned long)(s_Ethernet.Rate() / 1000000U));
		}
		else
		{
			SysLogPrintf(SysLogGet(), "Network down\r\n");
		}
	}
	if (!ready)
	{
		if (s_Started)
		{
			CoapDown(now, "network down");
		}
		return;
	}
	CoapTest(now);
}

static void ReportTimer(TimerDev_t *pTimer, int TrigNo, void *pCtx)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pCtx;
	if (!s_ReportQueued)
	{
		s_ReportQueued = AppEvtHandlerQue(0, nullptr, Report);
	}
}

int main()
{
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_LogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
	SysLogPrintf(SysLogGet(), "CoAP over UDP: LAN8742A RMII, DHCP, console 1 Mbaud\r\n");
	s_pRng = COAP_DEMO_RNG();
	if (!s_pRng || !s_pRng->Enable())
	{
		SysLogPrintf(SysLogGet(), "RNG initialization failed\r\n");
	}
	else if (!TimerInit(&s_Timer, &s_TimerCfg))
	{
		SysLogPrintf(SysLogGet(), "Timer initialization failed\r\n");
	}
	else if (!s_Ethernet.Init(s_EthernetCfg))
	{
		SysLogPrintf(SysLogGet(), "Ethernet initialization failed\r\n");
	}
	else if (!IpInit(&s_IpCfg, (EthernetDev_t *)s_Ethernet, &s_Timer, 0))
	{
		SysLogPrintf(SysLogGet(), "IP initialization failed\r\n");
	}
	else if (!msTimerEnableTrigger(&s_Timer, 1, 10, TIMER_TRIG_TYPE_CONTINUOUS,
		ReportTimer, nullptr))
	{
		SysLogPrintf(SysLogGet(), "CoAP timer initialization failed\r\n");
	}
	else
	{
		SysLogPrintf(SysLogGet(), "Waiting for Ethernet link and DHCP\r\n");
	}
	AppRun();
	return 0;
}
