/**-------------------------------------------------------------------------
@file	ethernet_demo.cpp

@brief	Ethernet DHCP/ping and optional connected UDP/TCP echo client.

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

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
#include <stdio.h>
#include <string.h>
#include "app_evt_handler.h"
#include "net/ethernet.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "net/ip.h"
#include "net/sock_intrf.h"
#include "syslog.h"
#include "board.h"

#ifndef NET_DEMO_HOST
#define NET_DEMO_HOST nullptr
#endif
#ifndef NET_DEMO_PROTO
#define NET_DEMO_PROTO SOCKINTRF_PROTO_UDP
#endif
#ifndef NET_DEMO_TEST_COUNT
#define NET_DEMO_TEST_COUNT 0
#endif
#ifndef NET_DEMO_HTTP_TEST
#define NET_DEMO_HTTP_TEST 0
#endif
#ifndef NET_DEMO_TLS_TEST
#define NET_DEMO_TLS_TEST 0
#endif
#ifndef NET_DEMO_MQTT_TEST
#define NET_DEMO_MQTT_TEST 0
#endif
#if NET_DEMO_MQTT_TEST
#include "net/mqtt.h"
static_assert(NET_DEMO_HTTP_TEST == 0 && NET_DEMO_TEST_COUNT == 0,
	"Select one Ethernet application test");
#endif
#if NET_DEMO_TLS_TEST
#include "net/tls.h"
#include NET_DEMO_TLS_ENGINE_HEADER
#include NET_DEMO_TLS_RNG_HEADER
#if NET_DEMO_TLS_DHE
#include NET_DEMO_TLS_AGREE_HEADER
#endif
static_assert(NET_DEMO_HTTP_TEST == 0 && NET_DEMO_TEST_COUNT == 0,
	"Select one Ethernet test mode");
#endif

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
// Hold complete startup and diagnostic bursts while the TX interrupt drains them.
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
	.Addr = {192,168,1,50},
	.Netmask = {255,255,255,0},
	.Gateway = {192,168,1,1},
	.Dns = {192,168,1,1},
	.bDhcp = true,
};
static const SockIntrfCfg_t s_SockCfg = {
#if NET_DEMO_HTTP_TEST
	.Proto = SOCKINTRF_PROTO_TCP,
	.pHost = "example.com",
	.Port = 80,
#else
	.Proto = NET_DEMO_PROTO,
	.pHost = NET_DEMO_HOST,
	.Port =
#if NET_DEMO_MQTT_TEST
		NET_DEMO_MQTT_PORT,
#else
		2469,
#endif
#endif
	.LocalPort = 0,
	.SecTag = 0,
	.bPeerVerify = false,
	.Timeout = NET_DEMO_HTTP_TEST ? 10000 : 3000,
	.EvtCB = nullptr,
};
static Ethernet s_Ethernet;
static UART s_Uart;
static TimerDev_t s_Timer;
static SockIntrf s_Socket;
static uint32_t s_Message;
static char s_MessageData[64];
static int s_MessageLength;
static int s_MessageOffset;
static bool s_WasReady;
static volatile bool s_ReportQueued;
alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 128)];
static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = 128,
	.bBlocking = false,
};

#if NET_DEMO_TEST_COUNT > 0
static_assert(NET_DEMO_TEST_COUNT <= 1000000, "Echo byte counters require at most 1000000 messages");
// One outstanding echo; fixed storage also covers a multi-segment TCP message.
static uint8_t s_TestTx[4096];
static uint8_t s_TestRx[4097];
static uint32_t s_TestPassed;
static uint32_t s_TestBytes;
static uint32_t s_TestStart;
static uint32_t s_TestSent;
static int s_TestLength;
static int s_TestTxOffset;
static int s_TestRxOffset;
static bool s_TestStarted;
static bool s_TestDone;

static void EchoTestFail(const char *pReason)
{
	s_TestDone = true;
	SysLogPrintf(SysLogGet(), "Echo FAIL: %s, passed=%lu len=%d tx=%d rx=%d\r\n",
		pReason, (unsigned long)s_TestPassed, s_TestLength, s_TestTxOffset, s_TestRxOffset);
	s_Socket.Close();
}

static void EchoTest(uint32_t Now)
{
	static const int lengths[] = {1, 15, 16, 17, 255, 256, 257, 1024, 1472, 4096};
	bool udp = s_SockCfg.Proto == SOCKINTRF_PROTO_UDP;
	int count;

	if (s_TestDone)
	{
		return;
	}
	if (s_TestStarted == false)
	{
		s_TestStarted = true;
		s_TestStart = Now;
		SysLogPrintf(SysLogGet(), "Echo test: %s, %lu messages, peer %s:%u\r\n",
			udp ? "UDP" : "TCP", (unsigned long)NET_DEMO_TEST_COUNT, s_SockCfg.pHost, s_SockCfg.Port);
	}
	if (s_TestLength == 0)
	{
		uint32_t pattern = s_TestPassed + 1U;

		s_TestLength = lengths[s_TestPassed % (udp ? 9U : 10U)];
		s_TestTxOffset = 0;
		s_TestRxOffset = 0;
		s_TestSent = Now;
		for (int i = 0; i < s_TestLength; i++)
		{
			pattern = pattern * 1664525U + 1013904223U;
			s_TestTx[i] = (uint8_t)(pattern >> 24);
		}
	}
	if (s_TestTxOffset < s_TestLength)
	{
		count = s_Socket.Tx(0, s_TestTx + s_TestTxOffset, s_TestLength - s_TestTxOffset);
		if (count < 0 || count > s_TestLength - s_TestTxOffset ||
			(udp && count != 0 && count != s_TestLength))
		{
			EchoTestFail("invalid send length");

			return;
		}
		s_TestTxOffset += count;
	}
	// Bounded foreground work; TCP may split the echoed stream arbitrarily.
	for (int i = 0; i < 8; i++)
	{
		count = s_Socket.Rx(0, s_TestRx, sizeof(s_TestRx));
		if (count == 0)
		{
			break;
		}
		if (count < 0 || count > s_TestTxOffset - s_TestRxOffset ||
			(udp && count != s_TestLength))
		{
			EchoTestFail("unexpected receive length");

			return;
		}
		if (memcmp(s_TestTx + s_TestRxOffset, s_TestRx, count) != 0)
		{
			EchoTestFail("data mismatch");

			return;
		}
		s_TestRxOffset += count;
	}
	if (s_TestRxOffset == s_TestLength)
	{
		s_TestPassed++;
		s_TestBytes += (uint32_t)s_TestLength;
		if (s_TestPassed == NET_DEMO_TEST_COUNT)
		{
			s_TestDone = true;
			SysLogPrintf(SysLogGet(), "Echo PASS: %lu messages, %lu bytes each way, %lu ms\r\n",
				(unsigned long)s_TestPassed, (unsigned long)s_TestBytes, (unsigned long)(Now - s_TestStart));
			s_Socket.Close();
		}
		else if (s_TestPassed % 100U == 0)
		{
			SysLogPrintf(SysLogGet(), "Echo checked %lu/%lu\r\n",
				(unsigned long)s_TestPassed, (unsigned long)NET_DEMO_TEST_COUNT);
		}
		s_TestLength = 0;
	}
	else if ((uint32_t)(Now - s_TestSent) >= 5000U)
	{
		EchoTestFail("5 second timeout");
	}
}
#endif

#if NET_DEMO_HTTP_TEST
static bool s_HttpStarted;
static bool s_HttpDone;
static uint32_t s_HttpStart;
static int s_HttpTxOffset;
static int s_HttpLineLength;
static char s_HttpLine[96];

static void HttpFinish(const char *pResult)
{
	SysLogPrintf(SysLogGet(), "%s\r\n", pResult);
	s_HttpDone = true;
	s_Socket.Close();
}

static void HttpTest(uint32_t Now)
{
	static const char request[] = "GET / HTTP/1.0\r\nHost: example.com\r\nConnection: close\r\n\r\n";
	uint8_t rx[128];
	int count;

	if (s_HttpDone)
	{
		return;
	}
	if (s_HttpStarted == false)
	{
		s_HttpStarted = true;
		s_HttpStart = Now;
		SysLogPrintf(SysLogGet(), "DNS/HTTP test: example.com:80\r\n");
		if (s_Socket.Init(s_SockCfg) == false)
		{
			HttpFinish("DNS/HTTP FAIL: name lookup or TCP connect failed");

			return;
		}
		SysLogPrintf(SysLogGet(), "DNS/TCP connected: example.com:80\r\n");
	}
	if (s_HttpTxOffset < (int)sizeof(request) - 1)
	{
		count = s_Socket.Tx(0, (const uint8_t *)request + s_HttpTxOffset,
			sizeof(request) - 1 - s_HttpTxOffset);
		if (count < 0 || count > (int)sizeof(request) - 1 - s_HttpTxOffset)
		{
			HttpFinish("DNS/HTTP FAIL: invalid send length");

			return;
		}
		s_HttpTxOffset += count;
	}
	// Read buffered bytes even if the peer has already closed its TCP side.
	count = s_Socket.Rx(0, rx, sizeof(rx));
	if (count < 0 || count > (int)sizeof(rx))
	{
		HttpFinish("DNS/HTTP FAIL: invalid receive length");

		return;
	}
	for (int i = 0; i < count; i++)
	{
		if (rx[i] == '\n')
		{
			if (s_HttpLineLength < 14 || s_HttpLine[s_HttpLineLength - 1] != '\r' ||
				(memcmp(s_HttpLine, "HTTP/1.0 ", 9) != 0 && memcmp(s_HttpLine, "HTTP/1.1 ", 9) != 0) ||
				s_HttpLine[9] < '2' || s_HttpLine[9] > '5' ||
				s_HttpLine[10] < '0' || s_HttpLine[10] > '9' ||
				s_HttpLine[11] < '0' || s_HttpLine[11] > '9' || s_HttpLine[12] != ' ')
			{
				HttpFinish("DNS/HTTP FAIL: invalid HTTP status line");

				return;
			}
			s_HttpLine[s_HttpLineLength - 1] = 0;
			SysLogPrintf(SysLogGet(), "HTTP response: %s\r\n", s_HttpLine);
			HttpFinish("DNS/HTTP PASS: received HTTP status from public hostname");

			return;
		}
		if (s_HttpLineLength == (int)sizeof(s_HttpLine) - 1 ||
			(s_HttpLineLength > 0 && s_HttpLine[s_HttpLineLength - 1] == '\r') ||
			(rx[i] != '\r' && (rx[i] < 32 || rx[i] > 126)))
		{
			HttpFinish("DNS/HTTP FAIL: invalid or oversized status line");

			return;
		}
		s_HttpLine[s_HttpLineLength++] = (char)rx[i];
	}
	if (s_Socket.Connected() == false && count <= 0)
	{
		HttpFinish("DNS/HTTP FAIL: socket closed before HTTP status");
	}
	else if ((uint32_t)(Now - s_HttpStart) >= 15000U)
	{
		HttpFinish("DNS/HTTP FAIL: 15 second timeout");
	}
}
#endif


#if NET_DEMO_MQTT_TEST
static Mqtt s_Mqtt;
static bool s_MqttStarted;
static bool s_MqttSubscribed;
static bool s_MqttSubscribeSent;
static bool s_MqttWaiting;
static uint32_t s_MqttChecked;
static uint32_t s_MqttSentAt;
#if !NET_DEMO_TLS_TEST
static uint32_t s_MqttRetry;
#endif
static char s_MqttPayload[48];
static int s_MqttLength;

static void MqttTestReset()
{
	s_Mqtt.Reset();
	s_MqttStarted = false;
	s_MqttSubscribed = false;
	s_MqttSubscribeSent = false;
	s_MqttWaiting = false;
}

static bool MqttTest(DeviceIntrf *pTransport, uint32_t Now, bool Open)
{
	const MqttCfg_t cfg = {NET_DEMO_MQTT_CLIENT_ID, nullptr, nullptr, 0, 15, 5000};
	const MqttMessage_t *message;
	MQTT_STATE state;

	if (!s_MqttStarted)
	{
		if (!s_Mqtt.Init(cfg, pTransport, Now))
		{
			SysLogPrintf(SysLogGet(), "MQTT init failed: %d\r\n", (int)s_Mqtt.Error());
			return false;
		}
		s_MqttStarted = true;
		SysLogPrintf(SysLogGet(), "MQTT connecting: %s\r\n", NET_DEMO_MQTT_CLIENT_ID);
	}
	state = s_Mqtt.Process(Now, Open);
	if (state == MQTT_FAILED || state == MQTT_CLOSED)
	{
		SysLogPrintf(SysLogGet(), "MQTT ended: error=%d result=%u\r\n",
			(int)s_Mqtt.Error(), (unsigned)s_Mqtt.Result());
		return false;
	}
	if (!s_Mqtt.Connected())
	{
		return true;
	}
	if (!s_MqttSubscribeSent)
	{
		s_MqttSubscribeSent = s_Mqtt.Subscribe(NET_DEMO_MQTT_TOPIC, 1, Now);
	}
	else if (!s_MqttSubscribed && !s_Mqtt.Busy())
	{
		if (s_Mqtt.Result() == 0x80)
		{
			SysLogPrintf(SysLogGet(), "MQTT subscription denied\r\n");
			return false;
		}
		s_MqttSubscribed = true;
		SysLogPrintf(SysLogGet(), "MQTT subscribed: %s QoS=%u\r\n",
			NET_DEMO_MQTT_TOPIC, (unsigned)s_Mqtt.Result());
	}
	message = s_Mqtt.Message();
	if (message)
	{
		if (!s_MqttWaiting || message->TopicLen != strlen(NET_DEMO_MQTT_TOPIC) ||
			memcmp(message->pTopic, NET_DEMO_MQTT_TOPIC, message->TopicLen) ||
			message->PayloadLen != (size_t)s_MqttLength ||
			memcmp(message->pPayload, s_MqttPayload, message->PayloadLen))
		{
			SysLogPrintf(SysLogGet(), "MQTT echo mismatch (use a unique, non-retained test topic)\r\n");
			s_Mqtt.Release();
			return false;
		}
		s_Mqtt.Release();
		s_MqttWaiting = false;
		s_MqttChecked++;
		if (s_MqttChecked == 1 || s_MqttChecked % 100 == 0)
		{
			SysLogPrintf(SysLogGet(), "MQTT echo checked %lu\r\n", (unsigned long)s_MqttChecked);
		}
	}
	if (s_MqttWaiting && (uint32_t)(Now - s_MqttSentAt) >= 5000U)
	{
		SysLogPrintf(SysLogGet(), "MQTT echo timeout\r\n");
		return false;
	}
	if (s_MqttSubscribed && !s_MqttWaiting && !s_Mqtt.Busy() &&
		(uint32_t)(Now - s_MqttSentAt) >= 1000U)
	{
		s_MqttLength = snprintf(s_MqttPayload, sizeof(s_MqttPayload),
			"IOsonata MQTT %lu", (unsigned long)s_MqttChecked);
		if (s_Mqtt.Publish(NET_DEMO_MQTT_TOPIC, (const uint8_t *)s_MqttPayload,
			s_MqttLength, 1, false, Now))
		{
			s_MqttWaiting = true;
			s_MqttSentAt = Now;
		}
	}
	return true;
}

#if !NET_DEMO_TLS_TEST
static void MqttTcpTest(uint32_t Now)
{
	if (!s_MqttStarted)
	{
		if ((uint32_t)(Now - s_MqttRetry) < 3000U)
		{
			return;
		}
		if (!s_Socket.Init(s_SockCfg))
		{
			SysLogPrintf(SysLogGet(), "MQTT TCP connect failed, retry in 3 seconds\r\n");
			s_MqttRetry = TimerGetMilisecond(&s_Timer);
			s_Socket.Close();
			return;
		}
	}
	if (!MqttTest(&s_Socket, TimerGetMilisecond(&s_Timer), s_Socket.Connected()))
	{
		MqttTestReset();
		s_Socket.Close();
		s_MqttRetry = TimerGetMilisecond(&s_Timer);
	}
}
#endif
#endif

#if NET_DEMO_TLS_TEST
static NET_DEMO_TLS_AES_TYPE s_TlsAes;
static NET_DEMO_TLS_HASH_TYPE s_TlsHash;
#if NET_DEMO_TLS_DHE
alignas(uint64_t) static uint8_t s_TlsAgreeMem[NET_DEMO_TLS_AGREE_MEMSIZE];
static KeyAgreeEngine *s_pTlsAgree;
#endif
static Tls s_Tls;
static bool s_TlsStarted;
static bool s_TlsReady;
static uint32_t s_TlsRetry;
static uint32_t s_TlsDeadline;
static uint32_t s_TlsChecked;
static uint32_t s_TlsLastPump;
static int s_TlsSent;
static int s_TlsReceived;
static char s_TlsMessage[64];
static uint8_t s_TlsReply[64];
static int s_TlsLength;

static void TlsTestDown(uint32_t Now, const char *pReason)
{
#if NET_DEMO_MQTT_TEST
	MqttTestReset();
#endif
	SysLogPrintf(SysLogGet(), "TLS: %s (error=%d), %s\r\n",
		pReason, (int)s_Tls.Error(),
		IpReady() ? "retry in 3 seconds" : "waiting for network recovery");
	s_Tls.Reset();
	s_Socket.Close();
	s_TlsStarted = false;
	s_TlsReady = false;
	s_TlsLength = 0;
	s_TlsRetry = Now;
}

static void TlsTest(uint32_t Now)
{
	// Public test credential, matching the README's OpenSSL command.
	// Replace with a provisioned secret for any deployment.
	static const uint8_t psk[32] = {
		0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
		16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
	};
	static const uint8_t identity[] = "IOsonata";
	TlsDev_t *pTls = s_Tls;
	TLS_STATE before;
	TLS_STATE state;
	uint32_t start;
	uint32_t elapsed;
	uint32_t gap;
	uint32_t age;
	uint32_t txAge;
	size_t txPending;
	size_t rxCount;
	size_t rxNeed;
	bool open;

	if (s_TlsStarted == false)
	{
		if ((uint32_t)(Now - s_TlsRetry) < 3000U)
		{
			return;
		}
#if NET_DEMO_TLS_DHE
		if (s_pTlsAgree == nullptr)
		{
			s_pTlsAgree = NET_DEMO_TLS_AGREE_CREATE(s_TlsAgreeMem,
				sizeof(s_TlsAgreeMem), NET_DEMO_TLS_RNG());
			if (s_pTlsAgree == nullptr)
			{
				TlsTestDown(Now, "P-256 engine init failed");
				return;
			}
		}
#endif
		TlsCfg_t cfg = {
			.pHash = &s_TlsHash,
			.pMac = &s_TlsHash,
			.pAead = &s_TlsAes,
			.pRng = NET_DEMO_TLS_RNG(),
			.pPsk = psk,
			.PskLen = sizeof(psk),
			.pIdentity = identity,
			.IdentityLen = sizeof(identity) - 1,
			.pServerName = nullptr,
			.Timeout = 10000,
#if NET_DEMO_TLS_DHE
			.pKeyAgree = s_pTlsAgree,
#else
			.pKeyAgree = nullptr,
#endif
		};
		if (!s_TlsAes.Enable())
		{
			TlsTestDown(TimerGetMilisecond(&s_Timer), "AES enable failed");
			return;
		}
		if (!s_TlsHash.Enable())
		{
			TlsTestDown(TimerGetMilisecond(&s_Timer), "HASH enable failed");
			return;
		}
		start = TimerGetMilisecond(&s_Timer);
		open = s_Socket.Init(s_SockCfg);
		Now = TimerGetMilisecond(&s_Timer);
		SysLogPrintf(SysLogGet(), "TLS TCP connect: ok=%u ms=%lu\r\n",
			(unsigned)open, (unsigned long)(Now - start));
		if (!open)
		{
			TlsTestDown(Now, "TCP connect failed");
			return;
		}
		start = TimerGetMilisecond(&s_Timer);
		open = s_Tls.Init(cfg, (DevIntrf_t *)s_Socket, start);
		Now = TimerGetMilisecond(&s_Timer);
		SysLogPrintf(SysLogGet(), "TLS init: ok=%u ms=%lu\r\n",
			(unsigned)open, (unsigned long)(Now - start));
		if (!open)
		{
			TlsTestDown(Now, "TLS init failed");
			return;
		}
		s_TlsLastPump = Now;
		s_TlsStarted = true;
		SysLogPrintf(SysLogGet(), "TLS 1.3 PSK handshake: %s:%u\r\n", s_SockCfg.pHost, s_SockCfg.Port);
	}
	// Preserve only counters and state: failure wipes the TLS record buffers.
	start = TimerGetMilisecond(&s_Timer);
	before = pTls->State;
	gap = start - s_TlsLastPump;
	age = start - pTls->Started;
	txPending = pTls->TxLen - pTls->TxOffset;
	txAge = txPending != 0 ? start - pTls->TxStarted : 0;
	rxCount = pTls->RxCount;
	rxNeed = pTls->RxNeed;
	open = s_Socket.Connected();
	state = s_Tls.Process(start, open);
	Now = TimerGetMilisecond(&s_Timer);
	elapsed = Now - start;
	s_TlsLastPump = Now;
	if (state != before || elapsed >= 100U || gap >= 1000U)
	{
		SysLogPrintf(SysLogGet(), "TLS pump: state=%u->%u tcp=%u ms=%lu gap=%lu\r\n",
			(unsigned)before, (unsigned)state, (unsigned)open,
			(unsigned long)elapsed, (unsigned long)gap);
	}
	if (state == TLS_FAILED)
	{
		SysLogPrintf(SysLogGet(), "TLS before pump: age=%lu tx=%lu txage=%lu rx=%lu/%lu\r\n",
			(unsigned long)age, (unsigned long)txPending, (unsigned long)txAge,
			(unsigned long)rxCount, (unsigned long)rxNeed);
	}
	if (state == TLS_FAILED || state == TLS_CLOSED)
	{
		TlsTestDown(Now, "connection ended");
		return;
	}
	if (!s_Tls.Connected())
	{
		return;
	}
	if (!s_TlsReady)
	{
		s_TlsReady = true;
		SysLogPrintf(SysLogGet(), NET_DEMO_TLS_DHE ?
			"TLS authenticated: AES-128-GCM/SHA-256, PSK-DHE P-256\r\n" :
			"TLS authenticated: AES-128-GCM/SHA-256, PSK-only\r\n");
	}
#if NET_DEMO_MQTT_TEST
	if (!MqttTest(&s_Tls, Now, s_Tls.Connected()))
	{
		TlsTestDown(TimerGetMilisecond(&s_Timer), "MQTT session failed");
	}
	return;
#endif
	if (s_TlsLength == 0)
	{
		s_TlsLength = snprintf(s_TlsMessage, sizeof(s_TlsMessage),
			"IOsonata TLS %lu\n", (unsigned long)s_TlsChecked);
		s_TlsSent = 0;
		s_TlsReceived = 0;
		s_TlsDeadline = Now;
	}
	if (s_TlsSent < s_TlsLength)
	{
		s_TlsSent += s_Tls.Tx(0, (const uint8_t *)s_TlsMessage + s_TlsSent, s_TlsLength - s_TlsSent);
	}
	s_TlsReceived += s_Tls.Rx(0, s_TlsReply + s_TlsReceived, sizeof(s_TlsReply) - s_TlsReceived);
	if (s_TlsReceived > s_TlsLength)
	{
		TlsTestDown(Now, "unexpected reply length");
		return;
	}
	if (s_TlsReceived == s_TlsLength)
	{
		// openssl s_server -rev reverses each line, preserving its newline.
		for (int i = 0; i < s_TlsLength; i++)
		{
			uint8_t expected = i == s_TlsLength - 1 ? '\n' : s_TlsMessage[s_TlsLength - 2 - i];
			if (s_TlsReply[i] != expected)
			{
				TlsTestDown(Now, "reply mismatch");
				return;
			}
		}
		s_TlsChecked++;
		if (s_TlsChecked == 1 || s_TlsChecked % 100 == 0)
		{
			SysLogPrintf(SysLogGet(), "TLS echo checked %lu\r\n", (unsigned long)s_TlsChecked);
		}
		s_TlsLength = 0;
	}
	else if ((uint32_t)(Now - s_TlsDeadline) >= 5000U)
	{
		TlsTestDown(Now, "echo timeout");
	}
}
#endif

static void Report(uint32_t Evt, void *pCtx)
{
	uint8_t addr[4];
	uint8_t rx[256];
	bool ready = IpReady();
	int count;

	(void)Evt;
	(void)pCtx;
	s_ReportQueued = false;
	if (ready != s_WasReady)
	{
		s_WasReady = ready;
		if (ready && IpAddress(addr))
		{
			SysLogPrintf(SysLogGet(), "IPv4 %u.%u.%u.%u, link %lu Mbit/s\r\n",
				addr[0], addr[1], addr[2], addr[3], (unsigned long)(s_Ethernet.Rate() / 1000000));
		}
		else
		{
			SysLogPrintf(SysLogGet(), "Network down\r\n");
		}
	}
	if (ready == false)
	{
#if NET_DEMO_MQTT_TEST
		MqttTestReset();
#endif
#if NET_DEMO_TLS_TEST
		if (s_TlsStarted)
		{
			TlsTestDown(TimerGetMilisecond(&s_Timer), "network down");
		}
#endif
#if NET_DEMO_HTTP_TEST
		if (s_HttpStarted && s_HttpDone == false)
		{
			HttpFinish("DNS/HTTP FAIL: network down");
		}
#endif
#if NET_DEMO_TEST_COUNT > 0
		if (s_TestStarted && s_TestDone == false)
		{
			EchoTestFail("network down");
		}
#endif
		s_Socket.Close();
		s_MessageLength = 0;
		s_MessageOffset = 0;

		return;
	}
#if NET_DEMO_TLS_TEST
	TlsTest(TimerGetMilisecond(&s_Timer));
	return;
#endif
#if NET_DEMO_MQTT_TEST && !NET_DEMO_TLS_TEST
	MqttTcpTest(TimerGetMilisecond(&s_Timer));
	return;
#endif
#if NET_DEMO_HTTP_TEST
	HttpTest(TimerGetMilisecond(&s_Timer));

	return;
#endif
	if (s_SockCfg.pHost == nullptr)
	{
		return;
	}
#if NET_DEMO_TEST_COUNT > 0
	if (s_TestDone)
	{
		return;
	}
#endif
	if (s_Socket.Connected() == false)
	{
#if NET_DEMO_TEST_COUNT > 0
		if (s_TestStarted)
		{
			EchoTestFail("socket disconnected");

			return;
		}
#endif
		s_MessageLength = 0;
		s_MessageOffset = 0;
		if (s_Socket.Init(s_SockCfg) == false)
		{
			SysLogPrintf(SysLogGet(), "Socket connection failed: %s:%u\r\n", s_SockCfg.pHost, s_SockCfg.Port);
#if NET_DEMO_TEST_COUNT > 0
			EchoTestFail("name lookup or connect failed");
#endif

			return;
		}
	}
#if NET_DEMO_TEST_COUNT > 0
	EchoTest(TimerGetMilisecond(&s_Timer));

	return;
#endif
	while ((count = s_Socket.Rx(0, rx, sizeof(rx) - 1)) > 0)
	{
		rx[count] = 0;
		SysLogPrintf(SysLogGet(), "RX %d: %s\r\n", count, (char *)rx);
	}
	if (s_MessageLength == 0)
	{
		s_MessageLength = snprintf(s_MessageData, sizeof(s_MessageData), "IOsonata Ethernet %lu", (unsigned long)s_Message++);
		s_MessageOffset = 0;
	}
	count = s_Socket.Tx(0, (const uint8_t *)s_MessageData + s_MessageOffset, s_MessageLength - s_MessageOffset);
	if (count > 0)
	{
		s_MessageOffset += count;
	}
	if (s_MessageOffset == s_MessageLength)
	{
		SysLogPrintf(SysLogGet(), "TX %s\r\n", s_MessageData);
		s_MessageLength = 0;
		s_MessageOffset = 0;
	}
	else
	{
		SysLogPrintf(SysLogGet(), "TX retry on next interval\r\n");
	}
}

static void ReportTimer(TimerDev_t *pTimer, int TrigNo, void *pCtx)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pCtx;
	if (s_ReportQueued == false)
	{
		s_ReportQueued = AppEvtHandlerQue(0, nullptr, Report);
	}
}

int main()
{
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_LogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
	SysLogPrintf(SysLogGet(), "Ethernet: LAN8742A RMII, DHCP, console 1 Mbaud\r\n");
	if (TimerInit(&s_Timer, &s_TimerCfg) == false)
	{
		SysLogPrintf(SysLogGet(), "Timer initialization failed\r\n");
	}
	else if (s_Ethernet.Init(s_EthernetCfg) == false)
	{
		SysLogPrintf(SysLogGet(), "Ethernet initialization failed\r\n");
	}
	else if (IpInit(&s_IpCfg, (EthernetDev_t *)s_Ethernet, &s_Timer, 0) == false)
	{
		SysLogPrintf(SysLogGet(), "IP initialization failed\r\n");
	}
	else if (msTimerEnableTrigger(&s_Timer, 1, NET_DEMO_MQTT_TEST || NET_DEMO_TLS_TEST || NET_DEMO_HTTP_TEST || NET_DEMO_TEST_COUNT > 0 ? 10 : 1000,
		TIMER_TRIG_TYPE_CONTINUOUS, ReportTimer, nullptr) == 0)
	{
		SysLogPrintf(SysLogGet(), "Report timer initialization failed\r\n");
	}
	else
	{
		SysLogPrintf(SysLogGet(), "Waiting for Ethernet link and DHCP\r\n");
	}
	AppRun();

	return 0;
}

