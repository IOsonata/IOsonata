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
	.Proto = NET_DEMO_PROTO,
	.pHost = NET_DEMO_HOST,
	.Port = 2469,
	.LocalPort = 0,
	.SecTag = 0,
	.bPeerVerify = false,
	.Timeout = 3000,
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
		s_Socket.Close();
		s_MessageLength = 0;
		s_MessageOffset = 0;

		return;
	}
	if (s_SockCfg.pHost == nullptr)
	{
		return;
	}
	if (s_Socket.Connected() == false)
	{
		s_MessageLength = 0;
		s_MessageOffset = 0;
		if (s_Socket.Init(s_SockCfg) == false)
		{
			SysLogPrintf(SysLogGet(), "Socket connection failed: %s:%u\r\n", s_SockCfg.pHost, s_SockCfg.Port);

			return;
		}
	}
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
	else if (msTimerEnableTrigger(&s_Timer, 1, 1000, TIMER_TRIG_TYPE_CONTINUOUS, ReportTimer, nullptr) == 0)
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
