/**-------------------------------------------------------------------------
@file	ip_lwip.cpp

@brief	IPv4 service and connected sockets on an injected Ethernet interface.

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
#include <string.h>
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "net/ip.h"
#include "net/sock_intrf.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip/timeouts.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "idelay.h"

#define IP_SOCKET_COUNT		4
#define IP_RX_QUEUE_COUNT	4
#define IP_PROCESS_FRAMES	8
#define IP_SERVICE_MS		10

typedef struct {
	SockIntrfDev_t *pDev;
	struct tcp_pcb *pTcp;
	struct udp_pcb *pUdp;
	struct pbuf *Rx[IP_RX_QUEUE_COUNT];
	uint16_t RxOffset;
	uint8_t RxHead;
	uint8_t RxCount;
	uint32_t Timeout;
	uint32_t Generation;
	ip_addr_t DnsAddr;
	bool bDnsDone;
	bool bConnecting;
	bool bRxEvent;
	bool bStateEvent;
} IpSocket_t;

typedef struct {
	IpSocket_t *pSock;
	uint32_t Generation;
	bool bUsed;
} IpDns_t;

static IpDns_t s_IpDns[4];
static struct netif s_IpNetif;
static IpSocket_t s_IpSockets[IP_SOCKET_COUNT];
static EthernetDev_t *s_pIpEthernet;
static TimerDev_t *s_pIpTimer;
static int s_IpTrig;
static DevIntrfEvtHandler_t s_IpOldEthCallback;
static uint32_t s_IpLinkTime;
static uint32_t s_IpRandom = 0x6D2B79F5UL;
static bool s_IpInitialized;
static bool s_IpStackReady;
static bool s_IpProcessing;
static bool s_IpDhcp;
static volatile bool s_IpPending;
static volatile bool s_IpQueued;
static uint8_t s_IpFrame[ETHERNET_FRAME_SIZE];

extern "C" u32_t sys_now(void)
{
	return s_pIpTimer != nullptr ? TimerGetMilisecond(s_pIpTimer) : 0;
}

extern "C" unsigned int IpRandom(void)
{
	// DNS transaction IDs and ephemeral ports; not a cryptographic RNG.
	s_IpRandom ^= s_IpRandom << 13;
	s_IpRandom ^= s_IpRandom >> 17;
	s_IpRandom ^= s_IpRandom << 5;

	return s_IpRandom;
}

static void IpRun(uint32_t Evt, void *pCtx)
{
	uint32_t state = DisableInterrupt();

	(void)Evt;
	(void)pCtx;
	s_IpQueued = false;
	s_IpPending = false;
	EnableInterrupt(state);
	IpProcess();
}

void IpCheckStatus(void)
{
	uint32_t state = DisableInterrupt();

	if (s_IpInitialized && s_IpPending && s_IpQueued == false)
	{
		s_IpQueued = AppEvtHandlerQue(0, nullptr, IpRun);
	}
	EnableInterrupt(state);
}

static void IpWake(void)
{
	s_IpPending = true;
	IpCheckStatus();
}

static void IpTimer(TimerDev_t *pTimer, int TrigNo, void *pCtx)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pCtx;
	IpWake();
}

static int IpEthernetEvent(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Len)
{
	(void)pDev;
	(void)Evt;
	(void)pData;
	(void)Len;
	IpWake();

	return 0;
}

static void IpNotify(void)
{
	for (int i = 0; i < IP_SOCKET_COUNT; i++)
	{
		IpSocket_t *sock = &s_IpSockets[i];
		SockIntrfDev_t *dev = sock->pDev;
		uint32_t generation = sock->Generation;
		bool rx = sock->bRxEvent;
		bool changed = sock->bStateEvent;

		sock->bRxEvent = false;
		sock->bStateEvent = false;
		if (dev == nullptr || dev->DevIntrf.EvtCB == nullptr)
		{
			continue;
		}
		if (rx)
		{
			dev->DevIntrf.EvtCB(&dev->DevIntrf, DEVINTRF_EVT_RX_DATA, nullptr, 0);
		}
		if (changed && sock->pDev == dev && sock->Generation == generation && dev->DevIntrf.EvtCB != nullptr)
		{
			dev->DevIntrf.EvtCB(&dev->DevIntrf, DEVINTRF_EVT_STATECHG, nullptr, 0);
		}
	}
}

static void IpSocketDown(IpSocket_t *pSock)
{
	if (pSock->pDev != nullptr)
	{
		pSock->pDev->bConnected = false;
		pSock->bConnecting = false;
		pSock->bStateEvent = true;
	}
}

static void IpTcpError(void *pArg, err_t Err)
{
	IpSocket_t *sock = (IpSocket_t *)pArg;

	(void)Err;
	// lwIP already freed this PCB. Queued received bytes remain readable.
	sock->pTcp = nullptr;
	IpSocketDown(sock);
}

static err_t IpTcpConnected(void *pArg, struct tcp_pcb *pPcb, err_t Err)
{
	IpSocket_t *sock = (IpSocket_t *)pArg;

	(void)pPcb;
	sock->bConnecting = false;
	if (sock->pDev != nullptr)
	{
		sock->pDev->bConnected = Err == ERR_OK;
	}

	return ERR_OK;
}

static err_t IpTcpReceived(void *pArg, struct tcp_pcb *pPcb, struct pbuf *pData, err_t Err)
{
	IpSocket_t *sock = (IpSocket_t *)pArg;
	int tail;

	(void)pPcb;
	if (pData == nullptr)
	{
		IpSocketDown(sock);

		return ERR_OK;
	}
	if (Err != ERR_OK || sock->RxCount == IP_RX_QUEUE_COUNT)
	{
		// Refuse without consuming; lwIP retains and retries the packet.
		return ERR_MEM;
	}
	tail = (sock->RxHead + sock->RxCount) % IP_RX_QUEUE_COUNT;
	sock->Rx[tail] = pData;
	sock->RxCount++;
	sock->bRxEvent = true;

	return ERR_OK;
}

static void IpUdpReceived(void *pArg, struct udp_pcb *pPcb, struct pbuf *pData,
		const ip_addr_t *pAddr, u16_t Port)
{
	IpSocket_t *sock = (IpSocket_t *)pArg;
	int tail;

	(void)pPcb;
	(void)pAddr;
	(void)Port;
	if (sock->RxCount == IP_RX_QUEUE_COUNT)
	{
		pbuf_free(pData);
		return;
	}
	tail = (sock->RxHead + sock->RxCount) % IP_RX_QUEUE_COUNT;
	sock->Rx[tail] = pData;
	sock->RxCount++;
	sock->bRxEvent = true;
}

static void IpDnsFound(const char *pName, const ip_addr_t *pAddr, void *pArg)
{
	IpDns_t *query = (IpDns_t *)pArg;
	IpSocket_t *sock = query->pSock;

	(void)pName;
	if (sock != nullptr && sock->Generation == query->Generation && sock->pDev != nullptr)
	{
		if (pAddr != nullptr)
		{
			sock->DnsAddr = *pAddr;
		}
		sock->bDnsDone = true;
	}
	query->pSock = nullptr;
	query->bUsed = false;
}

static err_t IpOutput(struct netif *pNetif, struct pbuf *pFrame)
{
	DevIntrf_t *eth = &s_pIpEthernet->DevIntrf;
	uint8_t frame[ETHERNET_FRAME_SIZE];

	(void)pNetif;
	if (pFrame->tot_len > sizeof(frame))
	{
		return ERR_BUF;
	}
	pbuf_copy_partial(pFrame, frame, pFrame->tot_len, 0);

	return DeviceIntrfTx(eth, 0, frame, pFrame->tot_len) == pFrame->tot_len ? ERR_OK : ERR_MEM;
}

static err_t IpNetifInit(struct netif *pNetif)
{
	pNetif->name[0] = 'e';
	pNetif->name[1] = 'n';
	pNetif->hwaddr_len = ETHERNET_ADDR_SIZE;
	memcpy(pNetif->hwaddr, s_pIpEthernet->MacAddr, ETHERNET_ADDR_SIZE);
	pNetif->mtu = 1500;
	pNetif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
	pNetif->output = etharp_output;
	pNetif->linkoutput = IpOutput;

	return ERR_OK;
}

static void IpAbort(IpSocket_t *pSock)
{
	if (pSock->pTcp != nullptr)
	{
		tcp_arg(pSock->pTcp, nullptr);
		tcp_err(pSock->pTcp, nullptr);
		tcp_recv(pSock->pTcp, nullptr);
		tcp_abort(pSock->pTcp);
		pSock->pTcp = nullptr;
	}
	if (pSock->pUdp != nullptr)
	{
		udp_remove(pSock->pUdp);
		pSock->pUdp = nullptr;
	}
	IpSocketDown(pSock);
}

void IpProcess(void)
{
	uint32_t now;

	if (s_IpInitialized == false || s_IpProcessing)
	{
		return;
	}
	s_IpProcessing = true;
	now = sys_now();
	if ((uint32_t)(now - s_IpLinkTime) >= 250U)
	{
		bool link;

		s_IpLinkTime = now;
		EthernetPollLink(s_pIpEthernet);
		link = DeviceIntrfGetRate(&s_pIpEthernet->DevIntrf) != 0;
		if (link != (netif_is_link_up(&s_IpNetif) != 0))
		{
			if (link)
			{
				netif_set_link_up(&s_IpNetif);
			}
			else
			{
				netif_set_link_down(&s_IpNetif);
				for (int i = 0; i < IP_SOCKET_COUNT; i++)
				{
					IpAbort(&s_IpSockets[i]);
				}
			}
		}
	}

	for (int i = 0; i < IP_PROCESS_FRAMES; i++)
	{
		int len = DeviceIntrfRx(&s_pIpEthernet->DevIntrf, 0, s_IpFrame, sizeof(s_IpFrame));
		struct pbuf *packet;

		if (len <= 0)
		{
			break;
		}
		packet = pbuf_alloc(PBUF_RAW, (u16_t)len, PBUF_POOL);
		if (packet != nullptr)
		{
			if (pbuf_take(packet, s_IpFrame, (u16_t)len) != ERR_OK ||
				ethernet_input(packet, &s_IpNetif) != ERR_OK)
			{
				pbuf_free(packet);
			}
		}
		if (i == IP_PROCESS_FRAMES - 1)
		{
			IpWake();
		}
	}
	sys_check_timeouts();
	IpNotify();
	s_IpProcessing = false;
}

bool IpReady(void)
{
	return s_IpInitialized && netif_is_link_up(&s_IpNetif) &&
		(s_IpDhcp == false || dhcp_supplied_address(&s_IpNetif)) &&
		ip4_addr_isany_val(*netif_ip4_addr(&s_IpNetif)) == false;
}

bool IpAddress(uint8_t Addr[4])
{
	const ip4_addr_t *addr;

	if (Addr == nullptr || IpReady() == false)
	{
		return false;
	}
	addr = netif_ip4_addr(&s_IpNetif);
	Addr[0] = ip4_addr1(addr);
	Addr[1] = ip4_addr2(addr);
	Addr[2] = ip4_addr3(addr);
	Addr[3] = ip4_addr4(addr);

	return true;
}

void IpClose(void)
{
	if (s_IpInitialized == false)
	{
		return;
	}
	TimerDisableTrigger(s_pIpTimer, s_IpTrig);
	for (int i = 0; i < IP_SOCKET_COUNT; i++)
	{
		if (s_IpSockets[i].pDev != nullptr)
		{
			SockIntrfClose(s_IpSockets[i].pDev);
		}
	}
	if (s_IpDhcp)
	{
		dhcp_stop(&s_IpNetif);
		dhcp_cleanup(&s_IpNetif);
	}
	netif_set_down(&s_IpNetif);
	netif_remove(&s_IpNetif);
	s_pIpEthernet->DevIntrf.EvtCB = s_IpOldEthCallback;
	s_IpInitialized = false;
	s_IpPending = false;
	s_pIpTimer = nullptr;
	// Keep bQueued until its already-queued handler runs. A close/reopen
	// must not publish a second event while the old event is still queued.
}

bool IpInit(const IpCfg_t *pCfg, EthernetDev_t *pEthernet, TimerDev_t *pTimer, int TrigNo)
{
	ip4_addr_t addr;
	ip4_addr_t mask;
	ip4_addr_t gateway;
	ip_addr_t dns;

	if (s_IpInitialized || pCfg == nullptr || pEthernet == nullptr || pTimer == nullptr ||
		pTimer->GetTickCount == nullptr || pTimer->Freq == 0 || pTimer->EnableTrigger == nullptr ||
		pTimer->GetMaxTrigger == nullptr || pTimer->DisableTrigger == nullptr || TrigNo < 0 || TrigNo >= TimerGetMaxTrigger(pTimer) ||
		pEthernet->DevIntrf.Type != DEVINTRF_TYPE_ETH || pEthernet->DevIntrf.GetHandle == nullptr ||
		pEthernet->DevIntrf.GetHandle(&pEthernet->DevIntrf) != pEthernet)
	{
		return false;
	}
	s_pIpEthernet = pEthernet;
	s_pIpTimer = pTimer;
	s_IpTrig = TrigNo;
	s_IpDhcp = pCfg->bDhcp;
	s_IpLinkTime = sys_now() - 250U;
	for (int i = 0; i < ETHERNET_ADDR_SIZE; i++)
	{
		s_IpRandom = (s_IpRandom << 5) ^ (s_IpRandom >> 2) ^ pEthernet->MacAddr[i];
	}
	s_IpRandom ^= sys_now();
	if (s_IpRandom == 0)
	{
		s_IpRandom = 1;
	}
	// lwIP is initialized once. Closing a netif must not reset live timeout
	// or TIME_WAIT lists retained by the stack.
	if (s_IpStackReady == false)
	{
		lwip_init();
		s_IpStackReady = true;
	}
	else
	{
		sys_restart_timeouts();
	}
	IP4_ADDR(&addr, pCfg->Addr[0], pCfg->Addr[1], pCfg->Addr[2], pCfg->Addr[3]);
	IP4_ADDR(&mask, pCfg->Netmask[0], pCfg->Netmask[1], pCfg->Netmask[2], pCfg->Netmask[3]);
	IP4_ADDR(&gateway, pCfg->Gateway[0], pCfg->Gateway[1], pCfg->Gateway[2], pCfg->Gateway[3]);
	if (pCfg->bDhcp)
	{
		ip4_addr_set_zero(&addr);
		ip4_addr_set_zero(&mask);
		ip4_addr_set_zero(&gateway);
	}
	if (netif_add(&s_IpNetif, &addr, &mask, &gateway, nullptr, IpNetifInit, ethernet_input) == nullptr)
	{
		return false;
	}
	s_IpOldEthCallback = pEthernet->DevIntrf.EvtCB;
	pEthernet->DevIntrf.EvtCB = IpEthernetEvent;
	s_IpInitialized = true;
	netif_set_default(&s_IpNetif);
	netif_set_up(&s_IpNetif);
	IP_ADDR4(&dns, pCfg->Dns[0], pCfg->Dns[1], pCfg->Dns[2], pCfg->Dns[3]);
	dns_setserver(0, &dns);
	if ((pCfg->bDhcp && dhcp_start(&s_IpNetif) != ERR_OK) ||
		msTimerEnableTrigger(pTimer, TrigNo, IP_SERVICE_MS, TIMER_TRIG_TYPE_CONTINUOUS, IpTimer, nullptr) == 0)
	{
		IpClose();

		return false;
	}
	IpWake();

	return true;
}

static IpSocket_t *IpSocket(SockIntrfDev_t *pDev)
{
	if (pDev != nullptr)
	{
		for (int i = 0; i < IP_SOCKET_COUNT; i++)
		{
			if (s_IpSockets[i].pDev == pDev)
			{
				return &s_IpSockets[i];
			}
		}
	}

	return nullptr;
}

static void IpSockNoOp(DevIntrf_t *pDev)
{
	(void)pDev;
}

static void *IpSockHandle(DevIntrf_t *pDev)
{
	return pDev->pDevData;
}

static uint32_t IpSockRate(DevIntrf_t *pDev)
{
	(void)pDev;

	return 0;
}

static uint32_t IpSockSetRate(DevIntrf_t *pDev, uint32_t Rate)
{
	(void)Rate;

	return IpSockRate(pDev);
}

static bool IpSockStartRx(DevIntrf_t *pDev, uint32_t Addr)
{
	(void)Addr;

	return IpSocket((SockIntrfDev_t *)pDev->pDevData) != nullptr;
}

static bool IpSockStartTx(DevIntrf_t *pDev, uint32_t Addr)
{
	SockIntrfDev_t *dev = (SockIntrfDev_t *)pDev->pDevData;

	(void)Addr;

	return IpSocket(dev) != nullptr && dev->bConnected;
}

static int IpSockRx(DevIntrf_t *pDev, uint8_t *pData, int Len)
{
	IpSocket_t *sock = IpSocket((SockIntrfDev_t *)pDev->pDevData);
	int count = 0;

	if (sock == nullptr)
	{
		return 0;
	}
	while (sock->RxCount > 0 && count < Len)
	{
		struct pbuf *packet = sock->Rx[sock->RxHead];
		int n = packet->tot_len - sock->RxOffset;

		if (n > Len - count)
		{
			n = Len - count;
		}
		pbuf_copy_partial(packet, pData + count, (u16_t)n, sock->RxOffset);
		count += n;
		sock->RxOffset += (uint16_t)n;
		if (sock->pTcp != nullptr)
		{
			tcp_recved(sock->pTcp, (u16_t)n);
		}
		if (sock->pDev->Proto == SOCKINTRF_PROTO_UDP || sock->RxOffset == packet->tot_len)
		{
			pbuf_free(packet);
			sock->RxHead = (sock->RxHead + 1) % IP_RX_QUEUE_COUNT;
			sock->RxCount--;
			sock->RxOffset = 0;
		}
		if (sock->pDev->Proto == SOCKINTRF_PROTO_UDP)
		{
			break;
		}
	}

	return count;
}

static int IpSockTx(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	IpSocket_t *sock = IpSocket((SockIntrfDev_t *)pDev->pDevData);
	uint32_t start = sys_now();
	uint32_t generation;
	uint32_t timeout;
	SockIntrfDev_t *dev = (SockIntrfDev_t *)pDev->pDevData;

	if (sock == nullptr || sock->pDev->bConnected == false)
	{
		return 0;
	}
	generation = sock->Generation;
	timeout = sock->Timeout;
	if (sock->pUdp != nullptr)
	{
		struct pbuf *packet;
		err_t res;

		if (Len > 1472)
		{
			return 0;
		}
		packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t)Len, PBUF_POOL);
		if (packet == nullptr)
		{
			return 0;
		}
		res = pbuf_take(packet, pData, (u16_t)Len);
		if (res == ERR_OK)
		{
			res = udp_send(sock->pUdp, packet);
		}
		pbuf_free(packet);

		return res == ERR_OK ? Len : 0;
	}

	// TCP can accept a prefix. The caller retries the remainder, just as
	// with the modem implementation. Never return a negative async count.
	do
	{
		if (sock->pDev != dev || sock->Generation != generation || sock->pTcp == nullptr || dev->bConnected == false)
		{
			return 0;
		}
		int n = tcp_sndbuf(sock->pTcp);

		if (n > Len)
		{
			n = Len;
		}
		if (n > 0 && tcp_write(sock->pTcp, pData, (u16_t)n, TCP_WRITE_FLAG_COPY) == ERR_OK)
		{
			tcp_output(sock->pTcp);

			return n;
		}
		IpProcess();
		usDelay(1000);
	} while ((uint32_t)(sys_now() - start) < timeout);

	return 0;
}

static void IpSockOff(DevIntrf_t *pDev)
{
	SockIntrfClose((SockIntrfDev_t *)pDev->pDevData);
}

void SockIntrfClose(SockIntrfDev_t *pDev)
{
	IpSocket_t *sock = IpSocket(pDev);
	uint32_t generation;

	if (pDev == nullptr)
	{
		return;
	}
	if (sock != nullptr)
	{
		if (sock->pTcp != nullptr)
		{
			struct tcp_pcb *pcb = sock->pTcp;

			tcp_arg(pcb, nullptr);
			tcp_err(pcb, nullptr);
			tcp_recv(pcb, nullptr);
			if (tcp_close(pcb) != ERR_OK)
			{
				tcp_abort(pcb);
			}
			sock->pTcp = nullptr;
		}
		if (sock->pUdp != nullptr)
		{
			udp_remove(sock->pUdp);
		}
		while (sock->RxCount > 0)
		{
			pbuf_free(sock->Rx[sock->RxHead]);
			sock->RxHead = (sock->RxHead + 1) % IP_RX_QUEUE_COUNT;
			sock->RxCount--;
		}
		// lwIP has no query-cancel API. Keep the small query context alive,
		// detached from this socket, until the eventual result/timeout.
		for (int i = 0; i < 4; i++)
		{
			if (s_IpDns[i].pSock == sock)
			{
				s_IpDns[i].pSock = nullptr;
			}
		}
		generation = sock->Generation + 1;
		memset(sock, 0, sizeof(*sock));
		sock->Generation = generation;
	}
	pDev->Hdl = -1;
	pDev->bConnected = false;
}

bool SockIntrfInit(SockIntrfDev_t *pDev, const SockIntrfCfg_t *pCfg)
{
	IpSocket_t *sock = nullptr;
	DevIntrf_t *dev;
	err_t res;
	uint32_t start;
	IpDns_t *query = nullptr;

	if (pDev == nullptr || pCfg == nullptr || pCfg->pHost == nullptr || pCfg->pHost[0] == 0 ||
		pCfg->Port == 0 || (pCfg->Proto != SOCKINTRF_PROTO_UDP && pCfg->Proto != SOCKINTRF_PROTO_TCP) ||
		pCfg->Timeout > 0x7FFFFFFFUL || s_IpProcessing)
	{
		return false;
	}
	SockIntrfClose(pDev);
	if (IpReady() == false)
	{
		return false;
	}
	for (int i = 0; i < IP_SOCKET_COUNT; i++)
	{
		if (s_IpSockets[i].pDev == nullptr)
		{
			sock = &s_IpSockets[i];
			pDev->Hdl = i;
			break;
		}
	}
	if (sock == nullptr)
	{
		return false;
	}
	sock->pDev = pDev;
	sock->Timeout = pCfg->Timeout != 0 ? pCfg->Timeout : SOCKINTRF_TIMEOUT_DEFAULT;
	pDev->Proto = pCfg->Proto;
	dev = &pDev->DevIntrf;
	dev->pDevData = pDev;
	dev->Type = DEVINTRF_TYPE_ETH;
	dev->IntPrio = 0;
	dev->EvtCB = pCfg->EvtCB;
	dev->Disable = IpSockNoOp;
	dev->Enable = IpSockNoOp;
	dev->GetRate = IpSockRate;
	dev->SetRate = IpSockSetRate;
	dev->StartRx = IpSockStartRx;
	dev->RxData = IpSockRx;
	dev->StopRx = IpSockNoOp;
	dev->StartTx = IpSockStartTx;
	dev->TxData = IpSockTx;
	dev->TxSrData = IpSockTx;
	dev->StopTx = IpSockNoOp;
	dev->Reset = IpSockNoOp;
	dev->PowerOff = IpSockOff;
	dev->GetHandle = IpSockHandle;
	dev->MaxRetry = 0;
	dev->bDma = false;
	dev->bIntEn = pCfg->EvtCB != nullptr;
	dev->EnCnt = 1;
	dev->bTxReady = true;
	dev->bNoStop = false;
	atomic_flag_clear(&dev->bBusy);

	start = sys_now();
	for (int i = 0; i < 4; i++)
	{
		if (s_IpDns[i].bUsed == false)
		{
			query = &s_IpDns[i];
			query->pSock = sock;
			query->Generation = sock->Generation;
			query->bUsed = true;
			break;
		}
	}
	if (ipaddr_aton(pCfg->pHost, &sock->DnsAddr))
	{
		res = ERR_OK;
	}
	else if (query != nullptr)
	{
		res = dns_gethostbyname(pCfg->pHost, &sock->DnsAddr, IpDnsFound, query);
	}
	else
	{
		res = ERR_MEM;
	}
	if (res != ERR_INPROGRESS && query != nullptr)
	{
		query->pSock = nullptr;
		query->bUsed = false;
	}
	if (res == ERR_INPROGRESS)
	{
		while (sock->bDnsDone == false && (uint32_t)(sys_now() - start) < sock->Timeout)
		{
			IpProcess();
			usDelay(1000);
		}
		res = sock->bDnsDone && ip_addr_isany(&sock->DnsAddr) == false ? ERR_OK : ERR_TIMEOUT;
	}
	if (res != ERR_OK || IpReady() == false)
	{
		SockIntrfClose(pDev);

		return false;
	}

	if (pCfg->Proto == SOCKINTRF_PROTO_UDP)
	{
		sock->pUdp = udp_new();
		if (sock->pUdp != nullptr && udp_bind(sock->pUdp, IP_ADDR_ANY, pCfg->LocalPort) == ERR_OK &&
			udp_connect(sock->pUdp, &sock->DnsAddr, pCfg->Port) == ERR_OK)
		{
			udp_recv(sock->pUdp, IpUdpReceived, sock);
			pDev->bConnected = true;
		}
	}
	else
	{
		sock->pTcp = tcp_new();
		if (sock->pTcp != nullptr)
		{
			tcp_arg(sock->pTcp, sock);
			tcp_err(sock->pTcp, IpTcpError);
			tcp_recv(sock->pTcp, IpTcpReceived);
			if (tcp_bind(sock->pTcp, IP_ADDR_ANY, pCfg->LocalPort) == ERR_OK)
			{
				sock->bConnecting = true;
				res = tcp_connect(sock->pTcp, &sock->DnsAddr, pCfg->Port, IpTcpConnected);
				if (res == ERR_OK)
				{
					while (sock->bConnecting && (uint32_t)(sys_now() - start) < sock->Timeout)
					{
						IpProcess();
						usDelay(1000);
					}
				}
			}
		}
	}
	if (pDev->bConnected == false)
	{
		SockIntrfClose(pDev);

		return false;
	}

	return true;
}

bool SockIntrfRai(SockIntrfDev_t *pDev, SOCKINTRF_RAI Rai)
{
	(void)pDev;
	(void)Rai;

	return false;
}

bool SockIntrfCredWrite(int SecTag, SOCKINTRF_CRED Type, const char *pData)
{
	(void)SecTag;
	(void)Type;
	(void)pData;

	return false;
}

bool SockIntrfCredDelete(int SecTag, SOCKINTRF_CRED Type)
{
	(void)SecTag;
	(void)Type;

	return false;
}

bool SockIntrfCredExists(int SecTag, SOCKINTRF_CRED Type)
{
	(void)SecTag;
	(void)Type;

	return false;
}
