/**-------------------------------------------------------------------------
@file	http_tap_host.cpp

@brief	The example exemples/net/http_server.cpp on a computer, on a TAP interface.

The example, the server, the IP service and lwIP are the real code. Only the Ethernet
driver, the timer and the UART are replaced, so that real HTTP clients of the computer
connect to it over the kernel TCP stack. A second server on port 8081 has routes for
the test: a large page, a body echo and a short time limit. tests/net/run_http_tap.py
drives it.

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
#define main HttpExampleMain
#include "../../exemples/net/http_server.cpp"
#undef main

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_tun.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define TAP_NAME	"iotap0"
#define MAX_TRIG	2

uint64_t g_NetTestMs;
int g_LedToggles;

static int s_TapFd = -1;
static EthernetDev_t *s_pEth;
static volatile sig_atomic_t s_Stop;

static struct {
	uint64_t Period;
	uint64_t Next;
	TimerTrigEvtHandler_t Handler;
	void *pCtx;
} s_Trig[MAX_TRIG];

static uint64_t ClockMs(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return (uint64_t)ts.tv_sec * 1000U + (uint64_t)ts.tv_nsec / 1000000U;
}

static uint64_t TimerTick(TimerDev_t *)
{
	return ClockMs();
}

static int TimerTrigs(TimerDev_t *)
{
	return MAX_TRIG;
}

static uint64_t TimerTrigOn(TimerDev_t *, int TrigNo, uint64_t Period, TIMER_TRIG_TYPE,
							TimerTrigEvtHandler_t Handler, void *pCtx)
{
	s_Trig[TrigNo].Period = Period / 1000000U;
	s_Trig[TrigNo].Next = ClockMs() + s_Trig[TrigNo].Period;
	s_Trig[TrigNo].Handler = Handler;
	s_Trig[TrigNo].pCtx = pCtx;

	return Period;
}

static void TimerTrigOff(TimerDev_t *, int TrigNo)
{
	s_Trig[TrigNo].Handler = nullptr;
}

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const)
{
	pTimer->Freq = 1000;
	pTimer->GetTickCount = TimerTick;
	pTimer->GetMaxTrigger = TimerTrigs;
	pTimer->EnableTrigger = TimerTrigOn;
	pTimer->DisableTrigger = TimerTrigOff;

	return true;
}

static int UartTx(DevIntrf_t *, const uint8_t *pData, int Len)
{
	fwrite(pData, 1, (size_t)Len, stdout);
	fflush(stdout);

	return Len;
}

static bool DevStart(DevIntrf_t *, uint32_t)
{
	return true;
}

static void DevStop(DevIntrf_t *)
{
}

bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *)
{
	pDev->DevIntrf.StartTx = DevStart;
	pDev->DevIntrf.TxData = UartTx;
	pDev->DevIntrf.StopTx = DevStop;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

void UARTSetCtrlLineState(UARTDev_t * const, uint32_t)
{
}

void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}

static int EthTx(DevIntrf_t *, const uint8_t *pData, int Len)
{
	return write(s_TapFd, pData, (size_t)Len) == Len ? Len : 0;
}

static int EthRx(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	ssize_t n = read(s_TapFd, pBuff, (size_t)Len);

	return n > 0 ? (int)n : 0;
}

static void *EthHandle(DevIntrf_t *pDev)
{
	return pDev->pDevData;
}

static uint32_t EthRate(DevIntrf_t *)
{
	return 100000000;
}

bool EthernetInit(EthernetDev_t *pDev, const EthernetCfg_t *pCfg)
{
	struct ifreq ifr = {};
	struct sockaddr_in addr = {};
	int sock;

	s_TapFd = open("/dev/net/tun", O_RDWR | O_NONBLOCK);
	if (s_TapFd < 0)
	{
		return false;
	}
	ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
	strncpy(ifr.ifr_name, TAP_NAME, IFNAMSIZ - 1);
	if (ioctl(s_TapFd, TUNSETIFF, &ifr) < 0)
	{
		return false;
	}
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	addr.sin_family = AF_INET;
	inet_pton(AF_INET, "10.99.0.1", &addr.sin_addr);
	memcpy(&ifr.ifr_addr, &addr, sizeof(addr));
	if (ioctl(sock, SIOCSIFADDR, &ifr) < 0)
	{
		return false;
	}
	inet_pton(AF_INET, "255.255.255.0", &addr.sin_addr);
	memcpy(&ifr.ifr_netmask, &addr, sizeof(addr));
	if (ioctl(sock, SIOCSIFNETMASK, &ifr) < 0 || ioctl(sock, SIOCGIFFLAGS, &ifr) < 0)
	{
		return false;
	}
	ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
	if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0)
	{
		return false;
	}
	close(sock);

	memcpy(pDev->MacAddr, pCfg->MacAddr, ETHERNET_ADDR_SIZE);
	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_ETH;
	pDev->DevIntrf.GetHandle = EthHandle;
	pDev->DevIntrf.GetRate = EthRate;
	pDev->DevIntrf.StartRx = DevStart;
	pDev->DevIntrf.RxData = EthRx;
	pDev->DevIntrf.StopRx = DevStop;
	pDev->DevIntrf.StartTx = DevStart;
	pDev->DevIntrf.TxData = EthTx;
	pDev->DevIntrf.StopTx = DevStop;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	s_pEth = pDev;

	return true;
}

void EthernetClose(EthernetDev_t *)
{
}

void EthernetPollLink(EthernetDev_t *)
{
}

// Routes of the test server
#define BIG_SIZE	300000

static uint8_t s_Big[BIG_SIZE];

static void EchoHandler(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *)
{
	pRsp->pType = "application/octet-stream";
	pRsp->pData = pReq->pBody;
	pRsp->Len = pReq->BodyLen;
}

static const HttpRoute_t s_TestRoutes[] = {
	{ "/big", 0, "application/octet-stream", s_Big, BIG_SIZE, nullptr, nullptr, nullptr },
	{ "/echo", HTTP_METHOD_POST, nullptr, nullptr, 0, nullptr, EchoHandler, nullptr },
};

alignas(4) static uint8_t s_TestMem[HTTP_SERVER_MEMSIZE(2, 1024)];
static HttpServer s_TestHttp;

static void TestServerInit(void)
{
	HttpServerCfg_t cfg = {};

	for (uint32_t i = 0; i < BIG_SIZE; i++)
	{
		s_Big[i] = (uint8_t)(i * 2654435761U >> 24);
	}
	cfg.Port = 8081;
	cfg.pRoutes = s_TestRoutes;
	cfg.NbRoute = sizeof(s_TestRoutes) / sizeof(s_TestRoutes[0]);
	cfg.pMem = s_TestMem;
	cfg.MemSize = sizeof(s_TestMem);
	cfg.NbConn = 2;
	cfg.Timeout = 1000;
	if (!s_TestHttp.Init(cfg))
	{
		printf("Test server init failed\n");
		exit(1);
	}
}

// Stands for the sleep of the MCU until its next interrupt
void AppWait(void)
{
	struct pollfd pfd = { s_TapFd, POLLIN, 0 };
	uint64_t now;

	if (s_Stop)
	{
		printf("LED toggles %d\n", g_LedToggles);
		fflush(stdout);
		exit(0);
	}
	poll(&pfd, 1, 1);
	now = ClockMs();
	for (int i = 0; i < MAX_TRIG; i++)
	{
		if (s_Trig[i].Handler != nullptr && now >= s_Trig[i].Next)
		{
			s_Trig[i].Next = now + s_Trig[i].Period;
			s_Trig[i].Handler(&s_Timer, i, s_Trig[i].pCtx);
		}
	}
	if ((pfd.revents & POLLIN) && s_pEth != nullptr && s_pEth->DevIntrf.EvtCB != nullptr)
	{
		s_pEth->DevIntrf.EvtCB(&s_pEth->DevIntrf, DEVINTRF_EVT_RX_DATA, nullptr, 0);
	}
	s_TestHttp.Process((uint32_t)now);
}

static void OnSignal(int)
{
	s_Stop = 1;
}

int main()
{
	signal(SIGTERM, OnSignal);
	signal(SIGINT, OnSignal);
	setvbuf(stdout, nullptr, _IOLBF, 0);
	TestServerInit();

	return HttpExampleMain();
}
