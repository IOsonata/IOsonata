/**-------------------------------------------------------------------------
@example	http_server.cpp

@brief	Small web server on Ethernet.

Gets an address by DHCP and shows it on the console. Open that address in a
browser. The page comes from flash, its script reads /status once a second and
its button toggles the first LED of the board. From a computer:

	curl http://<address>/status
	curl "http://<address>/hello?name=Hoan"
	curl -X POST http://<address>/led

The server knows nothing of Ethernet: it works on SockIntrf. Another network
needs only its own start up in NetworkStart and a SockIntrf that can listen.
Anyone reaching the port can use these routes. It is plain HTTP, for a network
that is trusted.

@author	Hoang Nguyen Hoan
@date	Nov. 4, 2026

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
#include <stdint.h>
#include <string.h>

#include "app_evt_handler.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "iopinctrl.h"
#include "net/ethernet.h"
#include "net/http_server.h"
#include "net/ip.h"
#include "syslog.h"
#include "board.h"

#ifndef HTTP_DEMO_DHCP
#define HTTP_DEMO_DHCP			true
#endif
#ifndef HTTP_DEMO_IP_ADDR
// Used when HTTP_DEMO_DHCP is false
#define HTTP_DEMO_IP_ADDR		{192, 168, 1, 50}
#define HTTP_DEMO_NETMASK		{255, 255, 255, 0}
#define HTTP_DEMO_GATEWAY		{192, 168, 1, 1}
#endif

// Timer triggers
#define HTTP_DEMO_TRIG_NETWORK	0
#define HTTP_DEMO_TRIG_PUMP		1
// The server sends at most one TCP window per pass, about 5.8 KB
#define HTTP_DEMO_PUMP_MS		10

#define HTTP_DEMO_NBCONN		2
// The longest request. What it leaves free is for a handler to build its response.
#define HTTP_DEMO_BUFF_SIZE		1536

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static const IOPinCfg_t s_UartPins[] = UART_PINS;
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
alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 128)];
static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = 128,
	.bBlocking = false,
};
static const TimerCfg_t s_TimerCfg = {
	.DevNo = ETHERNET_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 100000,
	.IntPrio = 6,
};
static const IOPinCfg_t s_EthernetPins[] = ETHERNET_PINS;
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
static const IpCfg_t s_IpCfg = {
	.Addr = HTTP_DEMO_IP_ADDR,
	.Netmask = HTTP_DEMO_NETMASK,
	.Gateway = HTTP_DEMO_GATEWAY,
	.Dns = HTTP_DEMO_GATEWAY,
	.bDhcp = HTTP_DEMO_DHCP,
};
#ifdef LED_PINS_MAP
static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
#endif

static UART s_Uart;
static TimerDev_t s_Timer;
static Ethernet s_Ethernet;
static HttpServer s_Http;
static uint32_t s_Requests;
static bool s_WasReady;
static volatile bool s_PumpQueued;

static const char s_IndexPage[] = R"page(<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>IOsonata</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{font-family:sans-serif;margin:2em}pre{background:#eee;padding:1em}</style>
</head><body>
<h1>IOsonata web server</h1>
<pre id="status">Reading...</pre>
<button onclick="fetch('/led',{method:'POST'})">Toggle LED</button>
<script>
function update(){
	fetch('/status').then(r=>r.text()).then(t=>{
		document.getElementById('status').textContent=t;
	}).catch(e=>{
		document.getElementById('status').textContent='No answer';
	});
}
setInterval(update,1000);
update();
</script>
</body></html>
)page";

static void StatusHandler(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *pCtx)
{
	uint8_t addr[4] = {};

	(void)pReq;
	(void)pCtx;
	s_Requests++;
	IpAddress(addr);
	pRsp->pType = "application/json";
	pRsp->pHeaders = "Cache-Control: no-store\r\n";
	HttpPrintf(pRsp, "{\"uptime_ms\":%lu,\"requests\":%lu,\"address\":\"%u.%u.%u.%u\",\"link_mbps\":%lu}\n",
			   (unsigned long)TimerGetMilisecond(&s_Timer), (unsigned long)s_Requests,
			   addr[0], addr[1], addr[2], addr[3], (unsigned long)(s_Ethernet.Rate() / 1000000UL));
}

static void HelloHandler(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *pCtx)
{
	char name[32];

	(void)pCtx;
	s_Requests++;
	if (HttpParamGet(pReq->pQuery, "name", name, sizeof(name)) < 0)
	{
		strcpy(name, "world");
	}
	pRsp->pType = "text/plain";
	HttpPrintf(pRsp, "Hello %s\n", name);
}

static void LedHandler(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *pCtx)
{
	(void)pReq;
	(void)pCtx;
	s_Requests++;
#ifdef LED_PINS_MAP
	IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
#endif
	pRsp->Status = 204;
}

static const HttpRoute_t s_Routes[] = {
	{ "/", 0, "text/html", s_IndexPage, sizeof(s_IndexPage) - 1, nullptr, nullptr, nullptr },
	{ "/status", 0, nullptr, nullptr, 0, nullptr, StatusHandler, nullptr },
	{ "/hello", 0, nullptr, nullptr, 0, nullptr, HelloHandler, nullptr },
	{ "/led", HTTP_METHOD_POST, nullptr, nullptr, 0, nullptr, LedHandler, nullptr },
};

alignas(4) static uint8_t s_HttpMem[HTTP_SERVER_MEMSIZE(HTTP_DEMO_NBCONN, HTTP_DEMO_BUFF_SIZE)];
static const HttpServerCfg_t s_HttpCfg = {
	.Port = 80,
	.pRoutes = s_Routes,
	.NbRoute = sizeof(s_Routes) / sizeof(s_Routes[0]),
	.pMem = s_HttpMem,
	.MemSize = sizeof(s_HttpMem),
	.NbConn = HTTP_DEMO_NBCONN,
	.Timeout = 0,
};

static void Pump(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_PumpQueued = false;
	s_Http.Process(TimerGetMilisecond(&s_Timer));

	const bool ready = IpReady();

	if (ready != s_WasReady)
	{
		uint8_t addr[4];

		s_WasReady = ready;
		if (ready && IpAddress(addr))
		{
			SysLogPrintf(SysLogGet(), "Open http://%u.%u.%u.%u/\r\n", addr[0], addr[1], addr[2], addr[3]);
		}
		else
		{
			SysLogPrintf(SysLogGet(), "Network down\r\n");
		}
	}
}

static void PumpTimer(TimerDev_t *pTimer, int TrigNo, void *pCtx)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pCtx;
	// Coalesce ticks. A full queue is retried on the next one.
	if (s_PumpQueued == false)
	{
		s_PumpQueued = AppEvtHandlerQue(0, nullptr, Pump);
	}
}

static bool NetworkStart(void)
{
	if (s_Ethernet.Init(s_EthernetCfg) == false)
	{
		SysLogPrintf(SysLogGet(), "Ethernet initialization failed\r\n");

		return false;
	}
	if (IpInit(&s_IpCfg, (EthernetDev_t *)s_Ethernet, &s_Timer, HTTP_DEMO_TRIG_NETWORK) == false)
	{
		SysLogPrintf(SysLogGet(), "IP initialization failed\r\n");

		return false;
	}

	return true;
}

int main()
{
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_LogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
#ifdef LED_PINS_MAP
	IOPinCfg(s_Leds, sizeof(s_Leds) / sizeof(s_Leds[0]));
#endif
	if (TimerInit(&s_Timer, &s_TimerCfg) == false)
	{
		SysLogPrintf(SysLogGet(), "Timer initialization failed\r\n");
	}
	else if (NetworkStart() == false || s_Http.Init(s_HttpCfg) == false)
	{
		SysLogPrintf(SysLogGet(), "Web server not started\r\n");
	}
	else if (msTimerEnableTrigger(&s_Timer, HTTP_DEMO_TRIG_PUMP, HTTP_DEMO_PUMP_MS,
								  TIMER_TRIG_TYPE_CONTINUOUS, PumpTimer, nullptr) == 0)
	{
		SysLogPrintf(SysLogGet(), "Timer trigger failed\r\n");
	}
	else
	{
		SysLogPrintf(SysLogGet(), "Waiting for the network\r\n");
	}
	AppRun();

	return 0;
}
