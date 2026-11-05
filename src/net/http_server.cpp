/**-------------------------------------------------------------------------
@file	http_server.cpp

@brief	Small HTTP/1.1 server on TCP sockets that wait for a peer.

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
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "net/http_server.h"

#define HTTP_SERVER_PORT_DEFAULT	80U
#define HTTP_SERVER_TIMEOUT_DEFAULT	5000U
// Longest wait in the socket when its send buffer is full. The server tries again
// on its next pass rather than hold up the other connections.
#define HTTP_SERVER_SOCK_WAIT		1U
// Largest write given to a socket at a time
#define HTTP_SERVER_TX_MAX			0x7FFFU

typedef enum __Http_Conn_State {
	HTTP_CONN_CLOSED,
	HTTP_CONN_LISTEN,
	HTTP_CONN_RECEIVE,
	HTTP_CONN_SEND,
	HTTP_CONN_DRAIN,
} HTTP_CONN_STATE;

static const struct {
	const char *pName;
	HTTP_METHOD Method;
} s_HttpMethods[] = {
	{ "GET", HTTP_METHOD_GET },
	{ "HEAD", HTTP_METHOD_HEAD },
	{ "POST", HTTP_METHOD_POST },
	{ "PUT", HTTP_METHOD_PUT },
	{ "DELETE", HTTP_METHOD_DELETE },
};

static const char *HttpReason(uint16_t Status)
{
	switch (Status)
	{
		case 200: return "OK";
		case 201: return "Created";
		case 202: return "Accepted";
		case 204: return "No Content";
		case 301: return "Moved Permanently";
		case 302: return "Found";
		case 303: return "See Other";
		case 304: return "Not Modified";
		case 400: return "Bad Request";
		case 401: return "Unauthorized";
		case 403: return "Forbidden";
		case 404: return "Not Found";
		case 405: return "Method Not Allowed";
		case 408: return "Request Timeout";
		case 413: return "Content Too Large";
		case 431: return "Request Header Fields Too Large";
		case 500: return "Internal Server Error";
		case 501: return "Not Implemented";
		case 503: return "Service Unavailable";
		case 505: return "HTTP Version Not Supported";
		default: return "";
	}
}

static HttpServerConn_t *HttpConn(HttpServerDev_t * const pDev, int Idx)
{
	return (HttpServerConn_t *)((uint8_t *)pDev->pConn + (uint32_t)Idx * pDev->ConnSize);
}

static const char *HttpHeaderFind(const char *pHead, uint32_t HeadLen, const char *pName,
								  uint32_t * const pLen, const char **ppNext)
{
	const char *line = pHead;
	const char *end = pHead + HeadLen;
	const size_t namelen = strlen(pName);

	while (line < end)
	{
		const char *eol = (const char *)memchr(line, '\r', (size_t)(end - line));

		if (eol == nullptr)
		{
			eol = end;
		}
		if ((size_t)(eol - line) > namelen && line[namelen] == ':' &&
			strncasecmp(line, pName, namelen) == 0)
		{
			const char *val = line + namelen + 1;
			const char *last = eol;

			while (val < last && (*val == ' ' || *val == '\t'))
			{
				val++;
			}
			while (last > val && (last[-1] == ' ' || last[-1] == '\t'))
			{
				last--;
			}
			*pLen = (uint32_t)(last - val);
			if (ppNext != nullptr)
			{
				*ppNext = eol;
			}

			return val;
		}
		line = eol + 2;
	}

	return nullptr;
}

const char *HttpHeaderGet(const HttpRequest_t * const pReq, const char *pName, uint32_t * const pLen)
{
	if (pReq == nullptr || pName == nullptr || pLen == nullptr)
	{
		return nullptr;
	}

	return HttpHeaderFind(pReq->pHead, pReq->HeadLen, pName, pLen, nullptr);
}

static int HttpHexVal(char C)
{
	if (C >= '0' && C <= '9')
	{
		return C - '0';
	}
	if (C >= 'a' && C <= 'f')
	{
		return C - 'a' + 10;
	}
	if (C >= 'A' && C <= 'F')
	{
		return C - 'A' + 10;
	}

	return -1;
}

int HttpParamGet(const char *pParams, const char *pName, char *pVal, int ValSize)
{
	if (pParams == nullptr || pName == nullptr || pVal == nullptr || ValSize <= 0)
	{
		return -1;
	}

	const size_t namelen = strlen(pName);
	const char *p = pParams;

	while (*p != 0)
	{
		const char *end = strchr(p, '&');

		if (end == nullptr)
		{
			end = p + strlen(p);
		}
		if ((size_t)(end - p) > namelen && p[namelen] == '=' && strncmp(p, pName, namelen) == 0)
		{
			int len = 0;

			for (p += namelen + 1; p < end; p++)
			{
				int c = (uint8_t)*p;

				if (c == '+')
				{
					c = ' ';
				}
				else if (c == '%')
				{
					if (end - p < 3 || HttpHexVal(p[1]) < 0 || HttpHexVal(p[2]) < 0)
					{
						return -1;
					}
					c = HttpHexVal(p[1]) * 16 + HttpHexVal(p[2]);
					if (c == 0)
					{
						return -1;
					}
					p += 2;
				}
				if (len + 1 >= ValSize)
				{
					return -1;
				}
				pVal[len++] = (char)c;
			}
			pVal[len] = 0;

			return len;
		}
		p = *end != 0 ? end + 1 : end;
	}

	return -1;
}

bool HttpPrintf(HttpResponse_t * const pRsp, const char *pFormat, ...)
{
	if (pRsp == nullptr || pFormat == nullptr || pRsp->pBuff == nullptr)
	{
		return false;
	}
	if (pRsp->pData != pRsp->pBuff)
	{
		pRsp->pData = pRsp->pBuff;
		pRsp->Len = 0;
	}

	const uint32_t room = pRsp->Len < pRsp->BuffSize ? pRsp->BuffSize - pRsp->Len : 0;
	va_list args;

	va_start(args, pFormat);
	int n = vsnprintf(room != 0 ? pRsp->pBuff + pRsp->Len : nullptr, room, pFormat, args);
	va_end(args);
	if (n < 0 || (uint32_t)n >= room)
	{
		// A length past the buffer gets the response status 500
		pRsp->Len = pRsp->BuffSize + 1U;

		return false;
	}
	pRsp->Len += (uint32_t)n;

	return true;
}

static void HttpConnClose(HttpServerConn_t * const pConn)
{
	SockIntrfClose(&pConn->Sock);
	pConn->State = HTTP_CONN_CLOSED;
}

// Status line and headers of the response into the connection. Sets what to send.
static void HttpConnRespond(HttpServerConn_t * const pConn, HttpResponse_t * const pRsp, const char *pAllow, uint32_t Now)
{
	const bool nobody = pRsp->Status == 204 || pRsp->Status == 304;
	const uint32_t len = nobody || pRsp->pData == nullptr ? 0 : pRsp->Len;
	const char *type = nobody == false && (pRsp->pType != nullptr || len != 0) ?
		(pRsp->pType != nullptr ? pRsp->pType : "application/octet-stream") : nullptr;
	char length[32] = "";
	int n;

	if (nobody == false)
	{
		snprintf(length, sizeof(length), "Content-Length: %lu\r\n", (unsigned long)len);
	}
	n = snprintf(pConn->Head, sizeof(pConn->Head),
				 "HTTP/1.1 %u %s\r\n%s%s%s%s%s%s%sConnection: close\r\n\r\n",
				 (unsigned)pRsp->Status, HttpReason(pRsp->Status),
				 type != nullptr ? "Content-Type: " : "", type != nullptr ? type : "", type != nullptr ? "\r\n" : "",
				 length,
				 pAllow != nullptr ? "Allow: " : "", pAllow != nullptr ? pAllow : "",
				 pRsp->pHeaders != nullptr ? pRsp->pHeaders : "");

	pConn->pTx = (const uint8_t *)pRsp->pData;
	pConn->TxLen = pConn->Req.Method == HTTP_METHOD_HEAD ? 0 : len;
	if (n < 0 || n >= (int)sizeof(pConn->Head))
	{
		// The headers of the application do not fit
		static const char s_Fail[] = "HTTP/1.1 500 Internal Server Error\r\n"
			"Content-Length: 0\r\nConnection: close\r\n\r\n";

		memcpy(pConn->Head, s_Fail, sizeof(s_Fail) - 1);
		n = sizeof(s_Fail) - 1;
		pConn->TxLen = 0;
	}
	pConn->HeadSize = (uint16_t)n;
	pConn->HeadOff = 0;
	pConn->TxOff = 0;
	pConn->Started = Now;
	pConn->State = HTTP_CONN_SEND;
}

// A response the server makes up itself, its reason as the body
static void HttpConnError(HttpServerConn_t * const pConn, uint16_t Status, const char *pAllow, uint32_t Now)
{
	HttpResponse_t rsp = {};

	rsp.Status = Status;
	rsp.pType = "text/plain";
	rsp.pData = HttpReason(Status);
	rsp.Len = (uint32_t)strlen((const char *)rsp.pData);
	HttpConnRespond(pConn, &rsp, pAllow, Now);
}

// The request line and the headers are all here. Returns 0 when good, or the status
// to answer with. The buffer is changed, 0 ends the path and the query.
static uint16_t HttpParseHead(HttpServerDev_t * const pDev, HttpServerConn_t * const pConn, uint32_t HeadEnd)
{
	char *buff = pConn->pBuff;
	char *line = (char *)memchr(buff, '\r', HeadEnd);
	const char *headend = buff + HeadEnd;
	uint32_t vallen;
	const char *val;
	const char *next;
	uint32_t bodylen = 0;

	if (memchr(buff, 0, HeadEnd) != nullptr || line == nullptr)
	{
		return 400;
	}

	char *target = (char *)memchr(buff, ' ', (size_t)(line - buff));
	char *version = target != nullptr ? (char *)memchr(target + 1, ' ', (size_t)(line - target - 1)) : nullptr;

	if (version == nullptr || target == buff)
	{
		return 400;
	}

	bool known = false;

	for (size_t i = 0; i < sizeof(s_HttpMethods) / sizeof(s_HttpMethods[0]); i++)
	{
		if (strlen(s_HttpMethods[i].pName) == (size_t)(target - buff) &&
			memcmp(buff, s_HttpMethods[i].pName, (size_t)(target - buff)) == 0)
		{
			pConn->Req.Method = s_HttpMethods[i].Method;
			known = true;
			break;
		}
	}

	const size_t verlen = (size_t)(line - version - 1);

	if (verlen != 8 || memcmp(version + 1, "HTTP/1.", 7) != 0)
	{
		return verlen >= 5 && memcmp(version + 1, "HTTP/", 5) == 0 ? 505 : 400;
	}
	if (version[8] != '0' && version[8] != '1')
	{
		return 505;
	}
	target++;
	if (*target != '/')
	{
		return 400;
	}
	for (const char *p = target; p < version; p++)
	{
		if ((uint8_t)*p <= ' ' || (uint8_t)*p >= 0x7F)
		{
			return 400;
		}
	}
	if (known == false)
	{
		return 501;
	}

	pConn->Req.pHead = line + 2;
	pConn->Req.HeadLen = (uint32_t)(headend - pConn->Req.pHead - 2);

	if (version[8] == '1' &&
		HttpHeaderFind(pConn->Req.pHead, pConn->Req.HeadLen, "Host", &vallen, nullptr) == nullptr)
	{
		return 400;
	}
	if (HttpHeaderFind(pConn->Req.pHead, pConn->Req.HeadLen, "Transfer-Encoding", &vallen, nullptr) != nullptr)
	{
		return 501;
	}
	val = HttpHeaderFind(pConn->Req.pHead, pConn->Req.HeadLen, "Content-Length", &vallen, &next);
	if (val != nullptr)
	{
		uint32_t nextlen;

		if (vallen == 0 || HttpHeaderFind(next, (uint32_t)(headend - next), "Content-Length", &nextlen, nullptr) != nullptr)
		{
			return 400;
		}
		for (uint32_t i = 0; i < vallen; i++)
		{
			if (val[i] < '0' || val[i] > '9')
			{
				return 400;
			}
			// Stops growing where it cannot overflow, any such length is refused
			if (bodylen < 0x10000000U)
			{
				bodylen = bodylen * 10U + (uint32_t)(val[i] - '0');
			}
		}
		// One byte stays free to end the body with a 0
		if (bodylen > pDev->BuffSize - 1U - HeadEnd)
		{
			const uint32_t got = pConn->RxLen - HeadEnd;

			pConn->DrainLen = bodylen > got ? bodylen - got : 0;

			return 413;
		}
	}
	pConn->ReqLen = HeadEnd + bodylen;
	pConn->Req.BodyLen = bodylen;
	pConn->Req.pBody = buff + HeadEnd;

	*version = 0;
	pConn->Req.pPath = target;

	char *query = strchr(target, '?');

	if (query != nullptr)
	{
		*query = 0;
		pConn->Req.pQuery = query + 1;
	}
	else
	{
		pConn->Req.pQuery = version;
	}

	return 0;
}

// The whole request is here
static void HttpConnDispatch(HttpServerDev_t * const pDev, HttpServerConn_t * const pConn, uint32_t Now)
{
	const uint8_t method = (uint8_t)pConn->Req.Method;
	uint8_t allowed = 0;

	pConn->pBuff[pConn->ReqLen] = 0;

	for (int i = 0; i < pDev->NbRoute; i++)
	{
		const HttpRoute_t *route = &pDev->pRoutes[i];

		if (route->pPath != nullptr && strcmp(route->pPath, pConn->Req.pPath) != 0)
		{
			continue;
		}

		uint8_t methods = route->Methods != 0 ? route->Methods : (uint8_t)HTTP_METHOD_GET;

		if (methods & HTTP_METHOD_GET)
		{
			methods |= HTTP_METHOD_HEAD;
		}
		if ((methods & method) == 0)
		{
			allowed |= methods;
			continue;
		}

		HttpResponse_t rsp = {};

		rsp.Status = 200;
		rsp.pType = route->pType;
		rsp.pHeaders = route->pHeaders;
		rsp.pData = route->pData;
		rsp.Len = route->DataLen;
		rsp.pBuff = pConn->pBuff + pConn->ReqLen + 1U;
		rsp.BuffSize = pDev->BuffSize - pConn->ReqLen - 1U;
		if (route->Handler != nullptr)
		{
			route->Handler(&pConn->Req, &rsp, route->pCtx);
		}
		if (rsp.Status < 200 || rsp.Status > 599 || (rsp.pData == rsp.pBuff && rsp.Len > rsp.BuffSize))
		{
			HttpConnError(pConn, 500, nullptr, Now);
		}
		else
		{
			HttpConnRespond(pConn, &rsp, nullptr, Now);
		}

		return;
	}

	if (allowed != 0)
	{
		char allow[48] = "";
		size_t n = 0;

		for (size_t i = 0; i < sizeof(s_HttpMethods) / sizeof(s_HttpMethods[0]); i++)
		{
			if (allowed & s_HttpMethods[i].Method)
			{
				n += (size_t)snprintf(allow + n, sizeof(allow) - n, "%s%s", n != 0 ? ", " : "", s_HttpMethods[i].pName);
			}
		}
		snprintf(allow + n, sizeof(allow) - n, "\r\n");
		HttpConnError(pConn, 405, allow, Now);
	}
	else
	{
		HttpConnError(pConn, 404, nullptr, Now);
	}
}

static void HttpConnReceive(HttpServerDev_t * const pDev, HttpServerConn_t * const pConn, uint32_t Now)
{
	const uint32_t max = pDev->BuffSize - 1U;
	bool progress = false;

	while (pConn->RxLen < max)
	{
		int n = SockIntrfRx(&pConn->Sock, (uint8_t *)pConn->pBuff + pConn->RxLen, (int)(max - pConn->RxLen));

		if (n <= 0)
		{
			break;
		}
		pConn->RxLen += (uint32_t)n;
		progress = true;
	}

	if (pConn->ReqLen == 0 && progress)
	{
		// The end of the head: the first blank line
		for (uint32_t i = 3; i < pConn->RxLen; i++)
		{
			if (pConn->pBuff[i] == '\n' && pConn->pBuff[i - 1] == '\r' &&
				pConn->pBuff[i - 2] == '\n' && pConn->pBuff[i - 3] == '\r')
			{
				uint16_t status = HttpParseHead(pDev, pConn, i + 1U);

				if (status != 0)
				{
					HttpConnError(pConn, status, nullptr, Now);

					return;
				}
				break;
			}
		}
		if (pConn->ReqLen == 0 && pConn->RxLen >= max)
		{
			HttpConnError(pConn, 431, nullptr, Now);

			return;
		}
	}
	if (pConn->ReqLen != 0 && pConn->RxLen >= pConn->ReqLen)
	{
		HttpConnDispatch(pDev, pConn, Now);

		return;
	}
	if (SockIntrfConnected(&pConn->Sock) == false)
	{
		HttpConnClose(pConn);
	}
	else if ((uint32_t)(Now - pConn->Started) >= pDev->Timeout)
	{
		HttpConnError(pConn, 408, nullptr, Now);
	}
}

static void HttpConnSend(HttpServerDev_t * const pDev, HttpServerConn_t * const pConn, uint32_t Now)
{
	while (pConn->HeadOff < pConn->HeadSize || pConn->TxOff < pConn->TxLen)
	{
		const bool head = pConn->HeadOff < pConn->HeadSize;
		const uint8_t *data = head ? (const uint8_t *)pConn->Head + pConn->HeadOff : pConn->pTx + pConn->TxOff;
		uint32_t len = head ? pConn->HeadSize - pConn->HeadOff : pConn->TxLen - pConn->TxOff;

		if (len > HTTP_SERVER_TX_MAX)
		{
			len = HTTP_SERVER_TX_MAX;
		}

		int n = SockIntrfTx(&pConn->Sock, data, (int)len);

		if (n <= 0)
		{
			if (SockIntrfConnected(&pConn->Sock) == false ||
				(uint32_t)(Now - pConn->Started) >= pDev->Timeout)
			{
				HttpConnClose(pConn);
			}

			return;
		}
		if (head)
		{
			pConn->HeadOff += (uint16_t)n;
		}
		else
		{
			pConn->TxOff += (uint32_t)n;
		}
		pConn->Started = Now;
	}
	pConn->State = HTTP_CONN_DRAIN;
}

// Closing with unread data, or data arriving after closing, sends a reset, and a reset
// can lose the response on the way to the peer. Read what is left, until the peer has
// sent the body it announced and has stopped sending. The time limit bounds this.
static void HttpConnDrain(HttpServerDev_t * const pDev, HttpServerConn_t * const pConn, uint32_t Now)
{
	bool read = false;

	for (int i = 0; i < 8; i++)
	{
		int n = SockIntrfRx(&pConn->Sock, (uint8_t *)pConn->pBuff, (int)pDev->BuffSize);

		if (n <= 0)
		{
			break;
		}
		read = true;
		pConn->DrainLen = (uint32_t)n < pConn->DrainLen ? pConn->DrainLen - (uint32_t)n : 0;
	}
	if ((read == false && pConn->DrainLen == 0) || SockIntrfConnected(&pConn->Sock) == false ||
		(uint32_t)(Now - pConn->Started) >= pDev->Timeout)
	{
		HttpConnClose(pConn);
	}
}

static void HttpConnProcess(HttpServerDev_t * const pDev, HttpServerConn_t * const pConn, uint32_t Now)
{
	if (pConn->State == HTTP_CONN_LISTEN && SockIntrfListening(&pConn->Sock) == false)
	{
		pConn->Started = Now;
		pConn->RxLen = 0;
		pConn->ReqLen = 0;
		pConn->DrainLen = 0;
		memset(&pConn->Req, 0, sizeof(pConn->Req));
		pConn->Req.Method = HTTP_METHOD_GET;
		pConn->State = HTTP_CONN_RECEIVE;
	}
	if (pConn->State == HTTP_CONN_RECEIVE)
	{
		HttpConnReceive(pDev, pConn, Now);
	}
	if (pConn->State == HTTP_CONN_SEND)
	{
		HttpConnSend(pDev, pConn, Now);
	}
	if (pConn->State == HTTP_CONN_DRAIN)
	{
		HttpConnDrain(pDev, pConn, Now);
	}
	// Last, so that a connection just closed is ready for the next peer before the
	// network stack runs again.
	if (pConn->State == HTTP_CONN_CLOSED)
	{
		SockIntrfCfg_t cfg = {};

		cfg.Proto = SOCKINTRF_PROTO_TCP;
		cfg.LocalPort = pDev->Port;
		cfg.Timeout = HTTP_SERVER_SOCK_WAIT;
		if (SockIntrfListen(&pConn->Sock, &cfg))
		{
			pConn->State = HTTP_CONN_LISTEN;
		}
	}
}

void HttpServerProcess(HttpServerDev_t * const pDev, uint32_t Now)
{
	if (pDev == nullptr || pDev->pConn == nullptr)
	{
		return;
	}
	for (int i = 0; i < pDev->NbConn; i++)
	{
		HttpConnProcess(pDev, HttpConn(pDev, i), Now);
	}
}

void HttpServerClose(HttpServerDev_t * const pDev)
{
	if (pDev == nullptr || pDev->pConn == nullptr)
	{
		return;
	}
	for (int i = 0; i < pDev->NbConn; i++)
	{
		HttpConnClose(HttpConn(pDev, i));
	}
}

bool HttpServerInit(HttpServerDev_t * const pDev, const HttpServerCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pRoutes == nullptr || pCfg->NbRoute <= 0 ||
		pCfg->pMem == nullptr || ((uintptr_t)pCfg->pMem & 3U) != 0 || pCfg->NbConn <= 0)
	{
		return false;
	}

	uint32_t connsize = (pCfg->MemSize / (uint32_t)pCfg->NbConn) & ~3U;

	if (connsize < (uint32_t)sizeof(HttpServerConn_t) + HTTP_SERVER_BUFF_MIN)
	{
		return false;
	}
	HttpServerClose(pDev);
	memset(pDev, 0, sizeof(HttpServerDev_t));
	memset(pCfg->pMem, 0, pCfg->MemSize);
	pDev->pRoutes = pCfg->pRoutes;
	pDev->NbRoute = pCfg->NbRoute;
	pDev->pConn = (HttpServerConn_t *)pCfg->pMem;
	pDev->NbConn = pCfg->NbConn;
	pDev->ConnSize = connsize;
	pDev->BuffSize = connsize - (uint32_t)sizeof(HttpServerConn_t);
	pDev->Timeout = pCfg->Timeout != 0 ? pCfg->Timeout : HTTP_SERVER_TIMEOUT_DEFAULT;
	pDev->Port = pCfg->Port != 0 ? pCfg->Port : (uint16_t)HTTP_SERVER_PORT_DEFAULT;
	for (int i = 0; i < pDev->NbConn; i++)
	{
		HttpServerConn_t *conn = HttpConn(pDev, i);

		conn->Sock.Hdl = -1;
		conn->pBuff = (char *)(conn + 1);
		conn->State = HTTP_CONN_CLOSED;
	}

	return true;
}
