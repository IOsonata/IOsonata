/**-------------------------------------------------------------------------
@file	sock_intrf_nrf91.cpp

@brief	Socket interface, nRF91 port on the modem IP stack (nrf_socket).

Implements net/sock_intrf.h. The modem does the name lookup, the IP stack
and TLS/DTLS. The poll callback of the socket (NRF_SO_POLLCB) runs in the
Modem library interrupt and gives the configuration EvtCB the data arrived
and closed events.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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
#include <errno.h>

#include "nrf.h"

#include "nrf_socket.h"
#include "nrf_modem_at.h"
#include "nrf_errno.h"
#include "coredev/interrupt.h"
#include "net/sock_intrf.h"

/** @addtogroup device_intrf
  * @{
  */

// %CMNG operations
#define SOCKINTRF_NRF91_CMNG_WRITE		0
#define SOCKINTRF_NRF91_CMNG_LIST		1
#define SOCKINTRF_NRF91_CMNG_DELETE		3

// Response of a %CMNG list for one tag and type
#define SOCKINTRF_NRF91_CMNG_RESP_LEN	160

// Open sockets, for the poll callback, which only has the handle
static SockIntrfDev_t *s_pSockIntrfNrf91[NRF_MODEM_MAX_SOCKET_COUNT];

// RAI socket option values, indexed by SOCKINTRF_RAI
static const int s_SockIntrfNrf91Rai[] = {
	NRF_RAI_NO_DATA, NRF_RAI_LAST, NRF_RAI_ONE_RESP, NRF_RAI_ONGOING, NRF_RAI_WAIT_MORE
};

// %CMNG credential types, indexed by SOCKINTRF_CRED: root CA certificate,
// client certificate, client private key, PSK, PSK identity
static const int s_SockIntrfNrf91CredType[] = { 0, 1, 2, 3, 4 };

// Errors after which the socket cannot be used again. Others (no memory,
// rate control, message too large, timeout) leave it open.
static const int s_SockIntrfNrf91Fatal[] = {
	NRF_EBADF, NRF_ENOTCONN, NRF_ECONNRESET, NRF_ECONNABORTED, NRF_EPIPE,
	NRF_ESHUTDOWN, NRF_ENETDOWN, NRF_ENETUNREACH, NRF_EHOSTDOWN
};

// Modem library interrupt
static void SockIntrfNrf91Poll(struct nrf_pollfd *pPoll)
{
	SockIntrfDev_t *dev = nullptr;

	for (int i = 0; i < NRF_MODEM_MAX_SOCKET_COUNT; i++)
	{
		if (s_pSockIntrfNrf91[i] != nullptr && s_pSockIntrfNrf91[i]->Hdl == pPoll->fd)
		{
			dev = s_pSockIntrfNrf91[i];
			break;
		}
	}

	if (dev == nullptr)
	{
		return;
	}

	DevIntrfEvtHandler_t cb = dev->DevIntrf.EvtCB;

	// Data still readable after a close is reported first
	if ((pPoll->revents & NRF_POLLIN) && cb != nullptr)
	{
		cb(&dev->DevIntrf, DEVINTRF_EVT_RX_DATA, nullptr, 0);
	}

	if (pPoll->revents & (NRF_POLLHUP | NRF_POLLERR | NRF_POLLNVAL))
	{
		dev->bConnected = false;
		if (cb != nullptr)
		{
			cb(&dev->DevIntrf, DEVINTRF_EVT_STATECHG, nullptr, 0);
		}
	}
}

static bool SockIntrfNrf91Stream(SockIntrfDev_t * const pDev)
{
	return pDev->Proto == SOCKINTRF_PROTO_TCP || pDev->Proto == SOCKINTRF_PROTO_TLS;
}

// The handle belongs to this interface: set by Init, which also sets
// pDevData. A zeroed interface has handle 0, which is not its own.
static bool SockIntrfNrf91Owned(SockIntrfDev_t * const pDev)
{
	return pDev->DevIntrf.pDevData == pDev && pDev->Hdl >= 0;
}

static bool SockIntrfNrf91Fatal(int Err)
{
	for (size_t i = 0; i < sizeof(s_SockIntrfNrf91Fatal) / sizeof(s_SockIntrfNrf91Fatal[0]); i++)
	{
		if (Err == s_SockIntrfNrf91Fatal[i])
		{
			return true;
		}
	}

	return false;
}

static void SockIntrfNrf91Disable(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static void SockIntrfNrf91Enable(DevIntrf_t * const pDev)
{
	(void)pDev;
}

// The network sets the rate
static uint32_t SockIntrfNrf91GetRate(DevIntrf_t * const pDev)
{
	(void)pDev;

	return 0;
}

static uint32_t SockIntrfNrf91SetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	(void)pDev;
	(void)Rate;

	return 0;
}

// Data left after a close can still be read
static bool SockIntrfNrf91StartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)DevAddr;

	return ((SockIntrfDev_t *)pDev->pDevData)->Hdl >= 0;
}

static int SockIntrfNrf91RxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	SockIntrfDev_t *dev = (SockIntrfDev_t *)pDev->pDevData;
	ssize_t n = nrf_recv(dev->Hdl, pBuff, (size_t)BuffLen, NRF_MSG_DONTWAIT);

	if (n > 0)
	{
		return (int)n;
	}

	// 0 on a stream is the peer closing it, an empty datagram otherwise
	if ((n == 0 && SockIntrfNrf91Stream(dev)) || (n < 0 && SockIntrfNrf91Fatal(errno)))
	{
		dev->bConnected = false;
	}

	return 0;
}

static void SockIntrfNrf91StopRx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static bool SockIntrfNrf91StartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)DevAddr;

	SockIntrfDev_t *dev = (SockIntrfDev_t *)pDev->pDevData;

	return dev->Hdl >= 0 && dev->bConnected;
}

// Waits at most the configuration timeout. A failure returns 0, never a
// negative value, which would leave the transfer open.
static int SockIntrfNrf91TxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	SockIntrfDev_t *dev = (SockIntrfDev_t *)pDev->pDevData;
	ssize_t n = nrf_send(dev->Hdl, pData, (size_t)DataLen, 0);

	if (n > 0)
	{
		return (int)n;
	}

	if (n < 0 && SockIntrfNrf91Fatal(errno))
	{
		dev->bConnected = false;
	}

	return 0;
}

static void SockIntrfNrf91StopTx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static void SockIntrfNrf91Reset(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static void SockIntrfNrf91PowerOff(DevIntrf_t * const pDev)
{
	SockIntrfClose((SockIntrfDev_t *)pDev->pDevData);
}

static void *SockIntrfNrf91GetHandle(DevIntrf_t * const pDev)
{
	return pDev->pDevData;
}

// Add or remove an open socket for the poll callback
static bool SockIntrfNrf91Slot(SockIntrfDev_t * const pDev, bool bAdd)
{
	uint32_t state = DisableInterrupt();
	bool done = false;

	for (int i = 0; i < NRF_MODEM_MAX_SOCKET_COUNT && done == false; i++)
	{
		if (bAdd ? s_pSockIntrfNrf91[i] == nullptr : s_pSockIntrfNrf91[i] == pDev)
		{
			s_pSockIntrfNrf91[i] = bAdd ? pDev : nullptr;
			done = true;
		}
	}
	EnableInterrupt(state);

	return done;
}

// Options, local port and connection of a new socket to one address
static bool SockIntrfNrf91Open(const SockIntrfCfg_t * const pCfg, struct nrf_addrinfo *pAddr, int Hdl)
{
	uint32_t ms = pCfg->Timeout != 0 ? pCfg->Timeout : SOCKINTRF_TIMEOUT_DEFAULT;
	struct nrf_timeval tv = { ms / 1000U, (ms % 1000U) * 1000U };

	if (nrf_setsockopt(Hdl, NRF_SOL_SOCKET, NRF_SO_SNDTIMEO, &tv, sizeof(tv)) != 0)
	{
		return false;
	}

	if (pCfg->Proto == SOCKINTRF_PROTO_DTLS || pCfg->Proto == SOCKINTRF_PROTO_TLS)
	{
		nrf_sec_tag_t tag = (nrf_sec_tag_t)pCfg->SecTag;
		int verify = pCfg->bPeerVerify ? NRF_SO_SEC_PEER_VERIFY_REQUIRED : NRF_SO_SEC_PEER_VERIFY_NONE;

		if (nrf_setsockopt(Hdl, NRF_SOL_SECURE, NRF_SO_SEC_TAG_LIST, &tag, sizeof(tag)) != 0 ||
			nrf_setsockopt(Hdl, NRF_SOL_SECURE, NRF_SO_SEC_PEER_VERIFY, &verify, sizeof(verify)) != 0 ||
			nrf_setsockopt(Hdl, NRF_SOL_SECURE, NRF_SO_SEC_HOSTNAME, pCfg->pHost, strlen(pCfg->pHost)) != 0)
		{
			return false;
		}
	}

	if (pCfg->LocalPort != 0)
	{
		if (pAddr->ai_family == NRF_AF_INET6)
		{
			struct nrf_sockaddr_in6 local = {};

			local.sin6_family = NRF_AF_INET6;
			local.sin6_port = nrf_htons(pCfg->LocalPort);
			if (nrf_bind(Hdl, (struct nrf_sockaddr *)&local, sizeof(local)) != 0)
			{
				return false;
			}
		}
		else
		{
			struct nrf_sockaddr_in local = {};

			local.sin_family = NRF_AF_INET;
			local.sin_port = nrf_htons(pCfg->LocalPort);
			if (nrf_bind(Hdl, (struct nrf_sockaddr *)&local, sizeof(local)) != 0)
			{
				return false;
			}
		}
	}

	if (pAddr->ai_family == NRF_AF_INET6)
	{
		((struct nrf_sockaddr_in6 *)pAddr->ai_addr)->sin6_port = nrf_htons(pCfg->Port);
	}
	else
	{
		((struct nrf_sockaddr_in *)pAddr->ai_addr)->sin_port = nrf_htons(pCfg->Port);
	}

	return nrf_connect(Hdl, pAddr->ai_addr, pAddr->ai_addrlen) == 0;
}

bool SockIntrfInit(SockIntrfDev_t * const pDev, const SockIntrfCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pHost == nullptr || pCfg->Port == 0 ||
		(unsigned)pCfg->Proto > SOCKINTRF_PROTO_TLS)
	{
		return false;
	}

	// A socket still open from an earlier Init is closed first
	SockIntrfClose(pDev);

	bool stream = pCfg->Proto == SOCKINTRF_PROTO_TCP || pCfg->Proto == SOCKINTRF_PROTO_TLS;
	int type = stream ? NRF_SOCK_STREAM : NRF_SOCK_DGRAM;
	int proto = pCfg->Proto == SOCKINTRF_PROTO_UDP ? NRF_IPPROTO_UDP :
				pCfg->Proto == SOCKINTRF_PROTO_TCP ? NRF_IPPROTO_TCP :
				pCfg->Proto == SOCKINTRF_PROTO_DTLS ? NRF_SPROTO_DTLS1v2 : NRF_SPROTO_TLS1v2;

	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.IntPrio = 0;
	pDev->DevIntrf.EvtCB = pCfg->EvtCB;
	pDev->DevIntrf.MaxRetry = 0;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_CEL;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = pCfg->EvtCB != nullptr;
	pDev->DevIntrf.Disable = SockIntrfNrf91Disable;
	pDev->DevIntrf.Enable = SockIntrfNrf91Enable;
	pDev->DevIntrf.GetRate = SockIntrfNrf91GetRate;
	pDev->DevIntrf.SetRate = SockIntrfNrf91SetRate;
	pDev->DevIntrf.StartRx = SockIntrfNrf91StartRx;
	pDev->DevIntrf.RxData = SockIntrfNrf91RxData;
	pDev->DevIntrf.StopRx = SockIntrfNrf91StopRx;
	pDev->DevIntrf.StartTx = SockIntrfNrf91StartTx;
	pDev->DevIntrf.TxData = SockIntrfNrf91TxData;
	pDev->DevIntrf.TxSrData = SockIntrfNrf91TxData;
	pDev->DevIntrf.StopTx = SockIntrfNrf91StopTx;
	pDev->DevIntrf.Reset = SockIntrfNrf91Reset;
	pDev->DevIntrf.PowerOff = SockIntrfNrf91PowerOff;
	pDev->DevIntrf.GetHandle = SockIntrfNrf91GetHandle;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	atomic_store(&pDev->DevIntrf.EnCnt, 1);
	atomic_store(&pDev->DevIntrf.bTxReady, true);
	atomic_store(&pDev->DevIntrf.bNoStop, false);
	pDev->Proto = pCfg->Proto;
	pDev->bConnected = false;
	pDev->Hdl = -1;

	struct nrf_addrinfo hints = {};
	struct nrf_addrinfo *res = nullptr;

	hints.ai_family = NRF_AF_UNSPEC;
	hints.ai_socktype = type;

	if (nrf_getaddrinfo(pCfg->pHost, nullptr, &hints, &res) != 0 || res == nullptr)
	{
		return false;
	}

	// Each address in turn: an IPv6 one fails on an IPv4 only bearer
	int hdl = -1;

	for (struct nrf_addrinfo *ai = res; ai != nullptr && hdl < 0; ai = ai->ai_next)
	{
		hdl = nrf_socket(ai->ai_family, type, proto);
		if (hdl >= 0 && SockIntrfNrf91Open(pCfg, ai, hdl) == false)
		{
			nrf_close(hdl);
			hdl = -1;
		}
	}

	nrf_freeaddrinfo(res);

	if (hdl < 0)
	{
		return false;
	}

	pDev->Hdl = hdl;
	pDev->bConnected = true;

	if (SockIntrfNrf91Slot(pDev, true) == false)
	{
		SockIntrfClose(pDev);

		return false;
	}

	// Events from here on: the socket is ready for them
	struct nrf_modem_pollcb cb = { SockIntrfNrf91Poll, NRF_POLLIN, false };

	if (nrf_setsockopt(hdl, NRF_SOL_SOCKET, NRF_SO_POLLCB, &cb, sizeof(cb)) != 0)
	{
		SockIntrfClose(pDev);

		return false;
	}

	// Data that arrived before the callback was set is reported now
	struct nrf_pollfd pfd = { hdl, NRF_POLLIN, 0 };

	if (nrf_poll(&pfd, 1, 0) > 0 && (pfd.revents & NRF_POLLIN) && pDev->DevIntrf.EvtCB != nullptr)
	{
		pDev->DevIntrf.EvtCB(&pDev->DevIntrf, DEVINTRF_EVT_RX_DATA, nullptr, 0);
	}

	return true;
}

void SockIntrfClose(SockIntrfDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return;
	}

	// Out of the table first: no poll callback for it from here on
	bool open = SockIntrfNrf91Slot(pDev, false);

	if (open || SockIntrfNrf91Owned(pDev))
	{
		nrf_close(pDev->Hdl);
	}
	pDev->bConnected = false;
	pDev->Hdl = -1;
}

bool SockIntrfRai(SockIntrfDev_t * const pDev, SOCKINTRF_RAI Rai)
{
	if (pDev == nullptr || SockIntrfNrf91Owned(pDev) == false ||
		(unsigned)Rai >= sizeof(s_SockIntrfNrf91Rai) / sizeof(s_SockIntrfNrf91Rai[0]))
	{
		return false;
	}

	int val = s_SockIntrfNrf91Rai[Rai];

	return nrf_setsockopt(pDev->Hdl, NRF_SOL_SOCKET, NRF_SO_RAI, &val, sizeof(val)) == 0;
}

static bool SockIntrfNrf91CredArgs(int SecTag, SOCKINTRF_CRED Type)
{
	return SecTag >= 0 &&
		   (unsigned)Type < sizeof(s_SockIntrfNrf91CredType) / sizeof(s_SockIntrfNrf91CredType[0]);
}

// The modem stores credentials with %CMNG, with the radio off (CFUN 0 or 4)
bool SockIntrfCredWrite(int SecTag, SOCKINTRF_CRED Type, const char *pData)
{
	if (SockIntrfNrf91CredArgs(SecTag, Type) == false || pData == nullptr || pData[0] == 0 ||
		strchr(pData, '"') != nullptr)
	{
		return false;
	}

	return nrf_modem_at_printf("AT%%CMNG=%d,%d,%d,\"%s\"", SOCKINTRF_NRF91_CMNG_WRITE, SecTag,
							   s_SockIntrfNrf91CredType[Type], pData) == 0;
}

bool SockIntrfCredDelete(int SecTag, SOCKINTRF_CRED Type)
{
	if (SockIntrfNrf91CredArgs(SecTag, Type) == false)
	{
		return false;
	}

	return nrf_modem_at_printf("AT%%CMNG=%d,%d,%d", SOCKINTRF_NRF91_CMNG_DELETE, SecTag,
							   s_SockIntrfNrf91CredType[Type]) == 0;
}

// A stored credential is listed as %CMNG: <tag>,<type>,<hash>
bool SockIntrfCredExists(int SecTag, SOCKINTRF_CRED Type)
{
	char resp[SOCKINTRF_NRF91_CMNG_RESP_LEN];

	if (SockIntrfNrf91CredArgs(SecTag, Type) == false ||
		nrf_modem_at_cmd(resp, sizeof(resp), "AT%%CMNG=%d,%d,%d", SOCKINTRF_NRF91_CMNG_LIST, SecTag,
						 s_SockIntrfNrf91CredType[Type]) != 0)
	{
		return false;
	}

	return strstr(resp, "%CMNG:") != nullptr;
}

/** @} End of group device_intrf */
