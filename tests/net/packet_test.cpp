/**-------------------------------------------------------------------------
@file	packet_test.cpp

@brief	Real lwIP raw API with an independent ARP/UDP/TCP packet peer.

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
#include <stdint.h>
#include "net/ip.h"
#include "net/sock_intrf.h"
#include "app_evt_handler.h"

uint64_t g_NetTestMs;
static uint8_t s_Frames[32][1514];
static int s_Len[32];
static int s_Head;
static int s_Count;
static uint32_t s_Seq = 10000;
static bool s_Link = true;
static bool s_DropSyn;
static bool s_DropDns;
static uint8_t s_DnsQuery[1514];
static int s_DnsLen;
static int s_DhcpOffers;
static int s_DhcpAcks;
static bool s_FailTrigger;
static bool s_FullQueue;
static int s_Events;
static int s_TcpBytes;
static int s_Resets;
static AppEvtHandler_t s_Queued;
static TimerTrigEvtHandler_t s_TimerHandler;
static TimerDev_t s_Timer;
static EthernetDev_t s_Ethernet;
static int s_Checks;
static bool s_IcmpReply;
static const uint8_t s_Mac[6] = {2, 0, 0, 0, 0, 2};
static const uint8_t s_Ip[4] = {192,168,1,10};
static const IpCfg_t s_Cfg = {{192,168,1,50},{255,255,255,0},{192,168,1,1},{192,168,1,10},false};

#define CHECK(X) \
	do \
	{ \
		s_Checks++; \
		if (!(X)) \
		{ \
			printf("FAIL line %d: %s\n", __LINE__, #X); \
			return 1; \
		} \
	} while (0)

static uint16_t R16(const uint8_t *p)
{
	return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t R32(const uint8_t *p)
{
	return ((uint32_t)R16(p) << 16) | R16(p + 2);
}

static void W16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)v;
}

static void W32(uint8_t *p, uint32_t v)
{
	W16(p, (uint16_t)(v >> 16));
	W16(p + 2, (uint16_t)v);
}

static uint16_t Sum(const uint8_t *p, int Len, uint32_t Seed = 0)
{
	uint32_t sum = Seed;

	while (Len > 1)
	{
		sum += R16(p);
		p += 2;
		Len -= 2;
	}
	if (Len)
	{
		sum += (uint16_t)*p << 8;
	}
	while (sum >> 16)
	{
		sum = (sum & 65535) + (sum >> 16);
	}

	return (uint16_t)~sum;
}

static void Enqueue(const uint8_t *p, int Len)
{
	int tail = (s_Head + s_Count) % 32;

	if (s_Count < 32)
	{
		memcpy(s_Frames[tail], p, Len);
		s_Len[tail] = Len;
		s_Count++;
	}
}

static void IpHeader(uint8_t *p, int Len, int Proto)
{
	memset(p, 0, 34);
	memcpy(p, s_Ethernet.MacAddr, 6);
	memcpy(p + 6, s_Mac, 6);
	W16(p + 12, 0x0800);
	p[14] = 0x45;
	W16(p + 16, (uint16_t)(Len - 14));
	p[22] = 64;
	p[23] = (uint8_t)Proto;
	memcpy(p + 26, s_Ip, 4);
	memcpy(p + 30, s_Cfg.Addr, 4);
	W16(p + 24, Sum(p + 14, 20));
}

static void UdpReply(const uint8_t *p, int Len)
{
	uint8_t reply[1514];

	IpHeader(reply, Len, 17);
	W16(reply + 34, R16(p + 36));
	W16(reply + 36, R16(p + 34));
	W16(reply + 38, (uint16_t)(Len - 34));
	W16(reply + 40, 0);
	memcpy(reply + 42, p + 42, Len - 42);
	Enqueue(reply, Len);
}

static void TcpReply(const uint8_t *p, int Len, uint8_t Flags)
{
	uint8_t reply[1514] = {};
	int hdr = (p[46] >> 4) * 4;
	int data = Len - 34 - hdr;
	int copy = Flags == 0x18 ? data : 0;
	int tcpHdr = Flags == 0x12 ? 24 : 20;
	uint32_t ack = R32(p + 38) + (uint32_t)data + ((p[47] & 3) ? 1 : 0);
	uint32_t pseudo;

	IpHeader(reply, 34 + tcpHdr + copy, 6);
	W16(reply + 34, R16(p + 36));
	W16(reply + 36, R16(p + 34));
	W32(reply + 38, s_Seq);
	W32(reply + 42, ack);
	reply[46] = (uint8_t)((tcpHdr / 4) << 4);
	if (tcpHdr == 24)
	{
		reply[54] = 2;
		reply[55] = 4;
		W16(reply + 56, 1460);
	}
	reply[47] = Flags;
	W16(reply + 48, 8192);
	if (copy)
	{
		memcpy(reply + 34 + tcpHdr, p + 34 + hdr, copy);
	}
	pseudo = R16(reply + 26) + R16(reply + 28) + R16(reply + 30) + R16(reply + 32) + 6 + tcpHdr + copy;
	W16(reply + 50, Sum(reply + 34, tcpHdr + copy, pseudo));
	Enqueue(reply, 34 + tcpHdr + copy);
	s_Seq += (uint32_t)copy + ((Flags & 3) ? 1 : 0);
}

static void DnsReply(const uint8_t *p, int Len)
{
	uint8_t reply[1514] = {};
	int size = Len + 16;

	IpHeader(reply, size, 17);
	W16(reply + 34, 53);
	W16(reply + 36, R16(p + 34));
	W16(reply + 38, (uint16_t)(size - 34));
	memcpy(reply + 42, p + 42, Len - 42);
	W16(reply + 44, 0x8180);
	W16(reply + 48, 1);
	W16(reply + 50, 0);
	W16(reply + 52, 0);
	W16(reply + Len, 0xC00C);
	W16(reply + Len + 2, 1);
	W16(reply + Len + 4, 1);
	W32(reply + Len + 6, 60);
	W16(reply + Len + 10, 4);
	memcpy(reply + Len + 12, s_Ip, 4);
	Enqueue(reply, size);
}

static void DhcpReply(const uint8_t *p, int Len)
{
	uint8_t reply[400] = {};
	uint8_t options[] = {53,1,2,54,4,192,168,1,10,51,4,0,0,14,16,
		1,4,255,255,255,0,3,4,192,168,1,1,6,4,192,168,1,10,255};
	int type = 0;
	int size = 42 + 240 + sizeof(options);

	for (int i = 282; i < Len && p[i] != 255;)
	{
		if (p[i] == 0)
		{
			i++;
			continue;
		}
		if (i + 2 >= Len || i + 2 + p[i + 1] > Len)
		{
			break;
		}
		if (p[i] == 53 && p[i + 1] == 1)
		{
			type = p[i + 2];
		}
		i += p[i + 1] + 2;
	}
	if (type != 1 && type != 3)
	{
		return;
	}
	if (type == 1)
	{
		s_DhcpOffers++;
	}
	else
	{
		s_DhcpAcks++;
		options[2] = 5;
	}
	IpHeader(reply, size, 17);
	memset(reply, 255, 6);
	memset(reply + 30, 255, 4);
	W16(reply + 24, 0);
	W16(reply + 24, Sum(reply + 14, 20));
	W16(reply + 34, 67);
	W16(reply + 36, 68);
	W16(reply + 38, (uint16_t)(size - 34));
	reply[42] = 2;
	reply[43] = 1;
	reply[44] = 6;
	memcpy(reply + 46, p + 46, 4);
	memcpy(reply + 58, s_Cfg.Addr, 4);
	memcpy(reply + 62, s_Ip, 4);
	memcpy(reply + 70, p + 70, 16);
	W32(reply + 278, 0x63825363);
	memcpy(reply + 282, options, sizeof(options));
	Enqueue(reply, size);
}

static int FrameTx(DevIntrf_t *, const uint8_t *p, int Len)
{
	if (R16(p + 12) == 0x0806 && R16(p + 20) == 1 && memcmp(p + 38, s_Ip, 4) == 0)
	{
		uint8_t reply[42];

		memcpy(reply, p, sizeof(reply));
		memcpy(reply, p + 6, 6);
		memcpy(reply + 6, s_Mac, 6);
		W16(reply + 20, 2);
		memcpy(reply + 22, s_Mac, 6);
		memcpy(reply + 28, s_Ip, 4);
		memcpy(reply + 32, p + 22, 6);
		memcpy(reply + 38, p + 28, 4);
		Enqueue(reply, sizeof(reply));
	}
	if (R16(p + 12) == 0x0800 && p[23] == 1 && Len == 46)
	{
		s_IcmpReply = p[34] == 0 && Sum(p + 34, 12) == 0 &&
			R16(p + 38) == 0x1234 && R16(p + 40) == 1 && memcmp(p + 42, "ping", 4) == 0;
	}
	if (R16(p + 12) == 0x0800 && p[23] == 17 && R16(p + 36) == 2469)
	{
		UdpReply(p, Len);
	}
	if (R16(p + 12) == 0x0800 && p[23] == 17 && R16(p + 36) == 53)
	{
		memcpy(s_DnsQuery, p, Len);
		s_DnsLen = Len;
		if (s_DropDns == false)
		{
			DnsReply(p, Len);
		}
	}
	if (R16(p + 12) == 0x0800 && p[23] == 17 && R16(p + 36) == 67)
	{
		DhcpReply(p, Len);
	}
	if (R16(p + 12) == 0x0800 && p[23] == 6)
	{
		if (p[47] & 4)
		{
			s_Resets++;
		}
		else if ((p[47] & 2) && s_DropSyn == false)
		{
			s_Seq = 10000;
			TcpReply(p, Len, 0x12);
		}
		else if (p[47] & 1)
		{
			TcpReply(p, Len, 0x11);
		}
		else if (Len > 34 + (p[46] >> 4) * 4)
		{
			s_TcpBytes += Len - 34 - (p[46] >> 4) * 4;
			TcpReply(p, Len, 0x18);
		}
	}

	return Len;
}

static int FrameRx(DevIntrf_t *, uint8_t *p, int Len)
{
	int n;

	if (s_Count == 0)
	{
		return 0;
	}
	n = s_Len[s_Head];
	if (n > Len)
	{
		return 0;
	}
	memcpy(p, s_Frames[s_Head], n);
	s_Head = (s_Head + 1) % 32;
	s_Count--;

	return n;
}

static bool Start(DevIntrf_t *, uint32_t)
{
	return true;
}

static void Stop(DevIntrf_t *)
{
}

static void *Handle(DevIntrf_t *)
{
	return &s_Ethernet;
}

static uint32_t Rate(DevIntrf_t *)
{
	return s_Link ? 100000000 : 0;
}

void EthernetPollLink(EthernetDev_t *)
{
}

void EthernetClose(EthernetDev_t *)
{
}

static uint64_t Tick(TimerDev_t *)
{
	return g_NetTestMs;
}

static int Triggers(TimerDev_t *)
{
	return 2;
}

static uint64_t Trigger(TimerDev_t *, int, uint64_t Period, TIMER_TRIG_TYPE, TimerTrigEvtHandler_t Handler, void *)
{
	s_TimerHandler = Handler;

	return s_FailTrigger ? 0 : Period;
}

static void TriggerOff(TimerDev_t *, int)
{
	s_TimerHandler = nullptr;
}

bool AppEvtHandlerQue(uint32_t, void *, AppEvtHandler_t Handler)
{
	if (s_FullQueue)
	{
		return false;
	}
	s_Queued = Handler;

	return true;
}

static int Event(DevIntrf_t *, DEVINTRF_EVT, uint8_t *, int)
{
	s_Events++;

	return 0;
}

static void Pump(int Count = 10)
{
	for (int i = 0; i < Count; i++)
	{
		g_NetTestMs += 10;
		IpProcess();
	}
}

int main()
{
	SockIntrfDev_t udp = {};
	SockIntrfDev_t tcp = {};
	SockIntrfCfg_t cfg = {SOCKINTRF_PROTO_UDP, "192.168.1.10", 2469, 0, 0, false, 20, Event};
	uint8_t data[3000];
	uint8_t rx[3000];

	memcpy(s_Ethernet.MacAddr, "\x02\x00\x00\x00\x00\x01", 6);
	s_Ethernet.DevIntrf.Type = DEVINTRF_TYPE_ETH;
	s_Ethernet.DevIntrf.GetHandle = Handle;
	s_Ethernet.DevIntrf.GetRate = Rate;
	s_Ethernet.DevIntrf.StartTx = Start;
	s_Ethernet.DevIntrf.StartRx = Start;
	s_Ethernet.DevIntrf.TxData = FrameTx;
	s_Ethernet.DevIntrf.RxData = FrameRx;
	s_Ethernet.DevIntrf.StopTx = Stop;
	s_Ethernet.DevIntrf.StopRx = Stop;
	s_Timer.Freq = 1000;
	s_Timer.GetTickCount = Tick;
	s_Timer.GetMaxTrigger = Triggers;
	s_Timer.EnableTrigger = Trigger;
	s_Timer.DisableTrigger = TriggerOff;
	CHECK(!IpInit(nullptr, &s_Ethernet, &s_Timer, 0));
	s_FailTrigger = true;
	CHECK(!IpInit(&s_Cfg, &s_Ethernet, &s_Timer, 0));
	s_FailTrigger = false;
	s_FullQueue = true;
	CHECK(IpInit(&s_Cfg, &s_Ethernet, &s_Timer, 0));
	CHECK(s_Queued == nullptr);
	s_FullQueue = false;
	IpCheckStatus();
	CHECK(s_Queued != nullptr);
	s_Queued(0, nullptr);
	s_Queued = nullptr;
	CHECK(IpReady());
	CHECK(IpAddress(rx) && memcmp(rx, s_Cfg.Addr, 4) == 0);
	uint8_t ping[46] = {};

	IpHeader(ping, sizeof(ping), 1);
	ping[34] = 8;
	W16(ping + 38, 0x1234);
	W16(ping + 40, 1);
	memcpy(ping + 42, "ping", 4);
	W16(ping + 36, Sum(ping + 34, 12));
	Enqueue(ping, sizeof(ping));
	Pump();
	CHECK(s_IcmpReply);
	CHECK(SockIntrfInit(&udp, &cfg));
	CHECK(SockIntrfTx(&udp, (const uint8_t *)"first", 5) == 5);
	Pump();
	CHECK(SockIntrfTx(&udp, (const uint8_t *)"second", 6) == 6);
	Pump();
	CHECK(SockIntrfRx(&udp, rx, 2) == 2 && memcmp(rx, "fi", 2) == 0);
	CHECK(SockIntrfRx(&udp, rx, sizeof(rx)) == 6 && memcmp(rx, "second", 6) == 0);
	CHECK(SockIntrfRx(&udp, rx, sizeof(rx)) == 0);
	CHECK(SockIntrfTx(&udp, data, 1473) == 0);
	CHECK(SockIntrfTx(&udp, (const uint8_t *)"again", 5) == 5);
	Pump();
	CHECK(SockIntrfRx(&udp, rx, sizeof(rx)) == 5);
	cfg.Proto = SOCKINTRF_PROTO_TLS;
	CHECK(!SockIntrfInit(&tcp, &cfg));
	cfg.Proto = SOCKINTRF_PROTO_TCP;
	CHECK(SockIntrfInit(&tcp, &cfg));
	for (int i = 0; i < (int)sizeof(data); i++)
	{
		data[i] = (uint8_t)(i * 37);
	}
	CHECK(SockIntrfTx(&tcp, data, sizeof(data)) == (int)sizeof(data));
	Pump();
	CHECK(s_TcpBytes == (int)sizeof(data));
	CHECK(SockIntrfRx(&tcp, rx, 17) == 17);
	CHECK(SockIntrfRx(&tcp, rx + 17, sizeof(rx) - 17) == (int)sizeof(rx) - 17);
	CHECK(memcmp(rx, data, sizeof(data)) == 0);
	SockIntrfClose(&tcp);
	s_DropSyn = true;
	CHECK(!SockIntrfInit(&tcp, &cfg));
	s_DropSyn = false;
	CHECK(SockIntrfInit(&tcp, &cfg));
	CHECK(SockIntrfTx(&udp, (const uint8_t *)"one", 3) == 3);
	Pump();
	CHECK(SockIntrfTx(&udp, (const uint8_t *)"two", 3) == 3);
	Pump();
	s_Link = false;
	Pump(30);
	CHECK(!IpReady() && !SockIntrfConnected(&tcp) && !SockIntrfConnected(&udp));
	CHECK(SockIntrfRx(&udp, rx, sizeof(rx)) == 3 && memcmp(rx, "one", 3) == 0);
	CHECK(SockIntrfRx(&udp, rx, sizeof(rx)) == 3 && memcmp(rx, "two", 3) == 0);
	s_Link = true;
	Pump(30);
	CHECK(IpReady());
	SockIntrfClose(&tcp);
	SockIntrfClose(&udp);
	IpClose();
	CHECK(!IpReady());
	CHECK(s_TimerHandler == nullptr);
	CHECK(IpInit(&s_Cfg, &s_Ethernet, &s_Timer, 0));
	Pump();
	CHECK(SockIntrfInit(&tcp, &cfg));
	SockIntrfClose(&tcp);
	IpClose();
	// DHCP negotiates an address through the independent packet peer.
	IpCfg_t dynamic = s_Cfg;

	dynamic.bDhcp = true;
	CHECK(IpInit(&dynamic, &s_Ethernet, &s_Timer, 0));
	CHECK(!IpReady());
	Pump(2000);
	CHECK(s_DhcpOffers > 0 && s_DhcpAcks > 0);
	CHECK(IpReady() && IpAddress(rx) && memcmp(rx, s_Cfg.Addr, 4) == 0);
	cfg.Proto = SOCKINTRF_PROTO_UDP;
	cfg.pHost = "echo.test";
	CHECK(SockIntrfInit(&udp, &cfg));
	CHECK(SockIntrfTx(&udp, (const uint8_t *)"dns", 3) == 3);
	Pump();
	CHECK(SockIntrfRx(&udp, rx, sizeof(rx)) == 3 && memcmp(rx, "dns", 3) == 0);
	SockIntrfClose(&udp);

	// A late DNS answer must not modify a replacement socket in the slot.
	s_DropDns = true;
	cfg.pHost = "late.test";
	CHECK(!SockIntrfInit(&udp, &cfg));
	CHECK(s_DnsLen > 0);
	cfg.pHost = "192.168.1.10";
	CHECK(SockIntrfInit(&udp, &cfg));
	DnsReply(s_DnsQuery, s_DnsLen);
	Pump();
	CHECK(SockIntrfConnected(&udp));
	CHECK(SockIntrfTx(&udp, (const uint8_t *)"late", 4) == 4);
	Pump();
	CHECK(SockIntrfRx(&udp, rx, sizeof(rx)) == 4 && memcmp(rx, "late", 4) == 0);
	SockIntrfClose(&udp);
	IpClose();
	CHECK(s_Events > 0);
	printf("Real lwIP packet tests: %d checks PASS\n", s_Checks);

	return 0;
}
