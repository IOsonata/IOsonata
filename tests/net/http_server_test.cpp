/**-------------------------------------------------------------------------
@file	http_server_test.cpp

@brief	HttpServer against a socket model: partial transfers, bad requests,
		timeouts and random input. No network stack.

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include "net/http_server.h"

#define MAX_SOCK	4
#define BUFF_SIZE	768

typedef struct {
	SockIntrfDev_t *pDev;
	std::string Rx;			// To the server
	std::string Tx;			// From the server
	int RxChunk;			// Most a read returns, 0 for any
	int TxChunk;			// Most a write takes, 0 for any
	int TxRefuse;			// Writes to refuse
	int TxBudget;			// Bytes it still takes, -1 for no limit
	bool bListening;
	uint16_t Port;
} SockModel_t;

static SockModel_t s_Sock[MAX_SOCK];
static bool s_ListenFail;
static int s_Closes;
static int s_Checks;
static int s_HandlerCalls;
static HttpRequest_t s_LastReq;
static std::string s_LastPath;
static std::string s_LastQuery;
static std::string s_LastBody;
static int s_LastBodyLen;

#define CHECK(X) \
	do \
	{ \
		s_Checks++; \
		if (!(X)) \
		{ \
			printf("FAIL line %d: %s\n", __LINE__, #X); \
			exit(1); \
		} \
	} while (0)

static SockModel_t *Model(DevIntrf_t *pDev)
{
	return (SockModel_t *)pDev->pDevData;
}

static bool ModelStart(DevIntrf_t *pDev, uint32_t)
{
	return Model(pDev)->pDev != nullptr;
}

static bool ModelStartTx(DevIntrf_t *pDev, uint32_t)
{
	return Model(pDev)->pDev != nullptr && Model(pDev)->pDev->bConnected;
}

static void ModelStop(DevIntrf_t *)
{
}

static int ModelRx(DevIntrf_t *pDev, uint8_t *pBuff, int Len)
{
	SockModel_t *sock = Model(pDev);
	int n = (int)sock->Rx.size();

	if (n > Len)
	{
		n = Len;
	}
	if (sock->RxChunk != 0 && n > sock->RxChunk)
	{
		n = sock->RxChunk;
	}
	memcpy(pBuff, sock->Rx.data(), (size_t)n);
	sock->Rx.erase(0, (size_t)n);

	return n;
}

static int ModelTx(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	SockModel_t *sock = Model(pDev);
	int n = Len;

	if (sock->TxRefuse > 0)
	{
		sock->TxRefuse--;

		return 0;
	}
	if (sock->TxChunk != 0 && n > sock->TxChunk)
	{
		n = sock->TxChunk;
	}
	if (sock->TxBudget >= 0)
	{
		if (n > sock->TxBudget)
		{
			n = sock->TxBudget;
		}
		sock->TxBudget -= n;
	}
	sock->Tx.append((const char *)pData, (size_t)n);

	return n;
}

bool SockIntrfListen(SockIntrfDev_t * const pDev, const SockIntrfCfg_t * const pCfg)
{
	if (s_ListenFail || pCfg->Proto != SOCKINTRF_PROTO_TCP || pCfg->LocalPort == 0)
	{
		return false;
	}
	SockIntrfClose(pDev);
	for (int i = 0; i < MAX_SOCK; i++)
	{
		if (s_Sock[i].pDev == nullptr)
		{
			DevIntrf_t *dev = &pDev->DevIntrf;

			s_Sock[i].pDev = pDev;
			s_Sock[i].bListening = true;
			s_Sock[i].Port = pCfg->LocalPort;
			memset((void *)dev, 0, sizeof(*dev));
			dev->pDevData = &s_Sock[i];
			dev->StartRx = ModelStart;
			dev->RxData = ModelRx;
			dev->StopRx = ModelStop;
			dev->StartTx = ModelStartTx;
			dev->TxData = ModelTx;
			dev->StopTx = ModelStop;
			atomic_flag_clear(&dev->bBusy);
			pDev->Hdl = i;
			pDev->bListening = true;
			pDev->bConnected = false;

			return true;
		}
	}

	return false;
}

void SockIntrfClose(SockIntrfDev_t * const pDev)
{
	for (int i = 0; i < MAX_SOCK; i++)
	{
		if (s_Sock[i].pDev == pDev)
		{
			s_Sock[i].pDev = nullptr;
			s_Closes++;
		}
	}
	pDev->Hdl = -1;
	pDev->bConnected = false;
	pDev->bListening = false;
}

// A peer connecting. Returns the model of the socket taking it.
static SockModel_t *Accept(void)
{
	for (int i = 0; i < MAX_SOCK; i++)
	{
		if (s_Sock[i].pDev != nullptr && s_Sock[i].pDev->bListening)
		{
			s_Sock[i].pDev->bListening = false;
			s_Sock[i].pDev->bConnected = true;
			s_Sock[i].Rx.clear();
			s_Sock[i].Tx.clear();
			s_Sock[i].RxChunk = 0;
			s_Sock[i].TxChunk = 0;
			s_Sock[i].TxRefuse = 0;
			s_Sock[i].TxBudget = -1;

			return &s_Sock[i];
		}
	}

	return nullptr;
}

static const char s_Page[] = "<html>page</html>";
static char s_Large[20000];

static void HandlerHello(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *pCtx)
{
	char name[32];

	s_HandlerCalls++;
	s_LastReq = *pReq;
	s_LastPath = pReq->pPath;
	s_LastQuery = pReq->pQuery;
	s_LastBody = pReq->pBody;
	s_LastBodyLen = (int)pReq->BodyLen;
	CHECK(pCtx == s_Page);
	pRsp->pType = "text/plain";
	if (HttpParamGet(pReq->pQuery, "name", name, sizeof(name)) < 0)
	{
		strcpy(name, "nobody");
	}
	HttpPrintf(pRsp, "hello %s", name);
	HttpPrintf(pRsp, " %d", 42);
}

static void HandlerEcho(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *)
{
	s_HandlerCalls++;
	pRsp->pType = "text/plain";
	pRsp->Status = 201;
	pRsp->pData = pReq->pBody;
	pRsp->Len = pReq->BodyLen;
}

static void HandlerOverflow(const HttpRequest_t *, HttpResponse_t *pRsp, void *)
{
	static char s_Text[4000];

	memset(s_Text, 'x', sizeof(s_Text) - 1);
	CHECK(HttpPrintf(pRsp, "%s", s_Text) == false);
	CHECK(HttpPrintf(pRsp, "more") == false);
}

static void HandlerExact(const HttpRequest_t *, HttpResponse_t *pRsp, void *)
{
	memset(pRsp->pBuff, 'y', pRsp->BuffSize);
	pRsp->pData = pRsp->pBuff;
	pRsp->Len = pRsp->BuffSize;
	pRsp->pType = "text/plain";
}

static void HandlerNoContent(const HttpRequest_t *, HttpResponse_t *pRsp, void *)
{
	pRsp->Status = 204;
}

static void HandlerBadStatus(const HttpRequest_t *, HttpResponse_t *pRsp, void *)
{
	pRsp->Status = 100;
}

static void HandlerRedirect(const HttpRequest_t *, HttpResponse_t *pRsp, void *)
{
	pRsp->Status = 302;
	pRsp->pHeaders = "Location: /\r\n";
}

static void HandlerLongHead(const HttpRequest_t *, HttpResponse_t *pRsp, void *)
{
	static char s_Headers[400];

	memset(s_Headers, 'h', sizeof(s_Headers) - 3);
	strcpy(s_Headers + sizeof(s_Headers) - 3, "\r\n");
	pRsp->pHeaders = s_Headers;
	pRsp->pType = "text/plain";
	pRsp->pData = "body";
	pRsp->Len = 4;
}

static void HandlerHeader(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *)
{
	uint32_t len = 0;
	const char *val = HttpHeaderGet(pReq, "x-TEST", &len);

	pRsp->pType = "text/plain";
	if (val == nullptr)
	{
		HttpPrintf(pRsp, "none");
	}
	else
	{
		HttpPrintf(pRsp, "[%.*s]", (int)len, val);
	}
}

static void HandlerNoData(const HttpRequest_t *, HttpResponse_t *pRsp, void *)
{
	pRsp->Len = 100;
}

static void HandlerAny(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *)
{
	pRsp->Status = 404;
	pRsp->pType = "text/html";
	HttpPrintf(pRsp, "custom %s", pReq->pPath);
}

static const HttpRoute_t s_Routes[] = {
	{ "/", 0, "text/html", s_Page, sizeof(s_Page) - 1, nullptr, nullptr, nullptr },
	{ "/hello", 0, nullptr, nullptr, 0, nullptr, HandlerHello, (void *)s_Page },
	{ "/echo", HTTP_METHOD_POST | HTTP_METHOD_PUT, nullptr, nullptr, 0, nullptr, HandlerEcho, nullptr },
	{ "/large", 0, "application/octet-stream", s_Large, sizeof(s_Large), nullptr, nullptr, nullptr },
	{ "/gz", 0, "text/html", s_Page, 4, "Content-Encoding: gzip\r\n", nullptr, nullptr },
	{ "/overflow", 0, nullptr, nullptr, 0, nullptr, HandlerOverflow, nullptr },
	{ "/exact", 0, nullptr, nullptr, 0, nullptr, HandlerExact, nullptr },
	{ "/nocontent", 0, nullptr, nullptr, 0, nullptr, HandlerNoContent, nullptr },
	{ "/badstatus", 0, nullptr, nullptr, 0, nullptr, HandlerBadStatus, nullptr },
	{ "/redirect", 0, nullptr, nullptr, 0, nullptr, HandlerRedirect, nullptr },
	{ "/longhead", 0, nullptr, nullptr, 0, nullptr, HandlerLongHead, nullptr },
	{ "/header", 0, nullptr, nullptr, 0, nullptr, HandlerHeader, nullptr },
	{ "/nodata", 0, nullptr, nullptr, 0, nullptr, HandlerNoData, nullptr },
};

static const HttpRoute_t s_RoutesAny[] = {
	{ "/", 0, "text/html", s_Page, sizeof(s_Page) - 1, nullptr, nullptr, nullptr },
	{ nullptr, 0, nullptr, nullptr, 0, nullptr, HandlerAny, nullptr },
};

alignas(4) static uint8_t s_Mem[HTTP_SERVER_MEMSIZE(2, BUFF_SIZE)];
static HttpServerDev_t s_Dev;
static uint32_t s_Now = 1000;

static void Pump(int Count = 1, uint32_t Step = 10)
{
	for (int i = 0; i < Count; i++)
	{
		s_Now += Step;
		HttpServerProcess(&s_Dev, s_Now);
	}
}

static void Start(const HttpRoute_t *pRoutes, int NbRoute, uint32_t Timeout = 0)
{
	HttpServerCfg_t cfg = {};

	HttpServerClose(&s_Dev);
	cfg.Port = 8080;
	cfg.pRoutes = pRoutes;
	cfg.NbRoute = NbRoute;
	cfg.pMem = s_Mem;
	cfg.MemSize = sizeof(s_Mem);
	cfg.NbConn = 2;
	cfg.Timeout = Timeout;
	CHECK(HttpServerInit(&s_Dev, &cfg));
	Pump();
}

// Sends a whole request on a new connection, returns the response
static std::string Request(const std::string &Text)
{
	SockModel_t *sock = Accept();

	CHECK(sock != nullptr);
	sock->Rx = Text;
	// Leftover bytes take a pass of their own to read before closing
	Pump(4);

	std::string res = sock->Tx;

	// A peer that announced a body it does not send leaves after the response
	if (sock->pDev->bListening == false)
	{
		sock->pDev->bConnected = false;
		Pump();
	}
	CHECK(sock->pDev->bListening);

	return res;
}

static std::string Body(const std::string &Rsp)
{
	size_t pos = Rsp.find("\r\n\r\n");

	return pos == std::string::npos ? "<no head>" : Rsp.substr(pos + 4);
}

static int Status(const std::string &Rsp)
{
	return Rsp.compare(0, 9, "HTTP/1.1 ") == 0 ? atoi(Rsp.c_str() + 9) : -1;
}

#define GET(Path) Request("GET " Path " HTTP/1.1\r\nHost: t\r\n\r\n")

static void TestConfig(void)
{
	HttpServerDev_t dev = {};
	HttpServerCfg_t cfg = {};
	static uint8_t small[HTTP_SERVER_MEMSIZE(1, HTTP_SERVER_BUFF_MIN - 8)];

	CHECK(!HttpServerInit(nullptr, &cfg));
	CHECK(!HttpServerInit(&dev, nullptr));
	CHECK(!HttpServerInit(&dev, &cfg));
	cfg.pRoutes = s_Routes;
	cfg.NbRoute = 1;
	cfg.pMem = s_Mem;
	cfg.MemSize = sizeof(s_Mem);
	CHECK(!HttpServerInit(&dev, &cfg));
	cfg.NbConn = 2;
	cfg.pMem = s_Mem + 1;
	CHECK(!HttpServerInit(&dev, &cfg));
	cfg.pMem = small;
	cfg.MemSize = sizeof(small);
	CHECK(!HttpServerInit(&dev, &cfg));
	cfg.NbConn = 1;
	CHECK(!HttpServerInit(&dev, &cfg));
	cfg.pMem = s_Mem;
	cfg.MemSize = sizeof(s_Mem);
	cfg.NbConn = 2;
	CHECK(HttpServerInit(&dev, &cfg));
	CHECK(dev.Port == 80 && dev.Timeout == 5000 && dev.BuffSize == BUFF_SIZE);
	HttpServerProcess(nullptr, 0);
	HttpServerClose(nullptr);
	CHECK(HttpHeaderGet(nullptr, "a", nullptr) == nullptr);
}

static void TestBasic(void)
{
	std::string r;

	Start(s_Routes, sizeof(s_Routes) / sizeof(s_Routes[0]));
	CHECK(s_Sock[0].bListening && s_Sock[0].Port == 8080);

	r = GET("/");
	CHECK(Status(r) == 200);
	CHECK(Body(r) == s_Page);
	CHECK(r.find("Content-Type: text/html\r\n") != std::string::npos);
	CHECK(r.find("Content-Length: 17\r\n") != std::string::npos);
	CHECK(r.find("Connection: close\r\n") != std::string::npos);

	// HEAD gets the same headers and no body, the handler runs
	r = Request("HEAD / HTTP/1.1\r\nHost: t\r\n\r\n");
	CHECK(Status(r) == 200 && Body(r).empty());
	CHECK(r.find("Content-Length: 17\r\n") != std::string::npos);
	s_HandlerCalls = 0;
	r = Request("HEAD /hello HTTP/1.1\r\nHost: t\r\n\r\n");
	CHECK(Status(r) == 200 && Body(r).empty() && s_HandlerCalls == 1);

	// HTTP/1.0 needs no Host
	r = Request("GET / HTTP/1.0\r\n\r\n");
	CHECK(Status(r) == 200 && Body(r) == s_Page);

	// Handler, query and extra header lines
	r = GET("/hello?name=a%20b+c&x=1");
	CHECK(Status(r) == 200 && Body(r) == "hello a b c 42");
	CHECK(s_LastPath == "/hello" && s_LastQuery == "name=a%20b+c&x=1" && s_LastBody.empty());
	r = GET("/hello");
	CHECK(Body(r) == "hello nobody 42" && s_LastQuery.empty());
	r = GET("/gz");
	CHECK(r.find("Content-Encoding: gzip\r\n") != std::string::npos && Body(r) == "<htm");
	r = GET("/redirect");
	CHECK(Status(r) == 302 && r.find("Location: /\r\n") != std::string::npos);
	CHECK(r.find("Content-Type") == std::string::npos && r.find("Content-Length: 0\r\n") != std::string::npos);
	r = GET("/nocontent");
	CHECK(Status(r) == 204 && r.find("Content-Length") == std::string::npos && Body(r).empty());

	// Header lookup: any case, spaces around the value removed
	r = Request("GET /header HTTP/1.1\r\nHost: t\r\nX-Test:  \t  a value\t \r\nOther: b\r\n\r\n");
	CHECK(Body(r) == "[a value]");
	r = Request("GET /header HTTP/1.1\r\nHost: t\r\nX-Test2: a\r\nX-Tes: b\r\n\r\n");
	CHECK(Body(r) == "none");

	// POST body, including one in a later segment
	r = Request("POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 5\r\n\r\nhello");
	CHECK(Status(r) == 201 && Body(r) == "hello");
	r = Request("PUT /echo HTTP/1.1\r\nhost: t\r\ncontent-length:   3  \r\n\r\nabcEXTRA");
	CHECK(Status(r) == 201 && Body(r) == "abc");
	r = Request("POST /echo HTTP/1.1\r\nHost: t\r\n\r\n");
	CHECK(Status(r) == 201 && Body(r).empty());
}

static void TestErrors(void)
{
	std::string r;

	Start(s_Routes, sizeof(s_Routes) / sizeof(s_Routes[0]));
	CHECK(Status(GET("/missing")) == 404);
	CHECK(Status(GET("/hello/")) == 404);
	CHECK(Status(GET("/%2F")) == 404);
	r = Request("POST / HTTP/1.1\r\nHost: t\r\nContent-Length: 0\r\n\r\n");
	CHECK(Status(r) == 405 && r.find("Allow: GET, HEAD\r\n") != std::string::npos);
	r = GET("/echo");
	CHECK(Status(r) == 405 && r.find("Allow: POST, PUT\r\n") != std::string::npos);
	r = Request("DELETE /echo HTTP/1.1\r\nHost: t\r\n\r\n");
	CHECK(Status(r) == 405);
	CHECK(Status(Request("OPTIONS / HTTP/1.1\r\nHost: t\r\n\r\n")) == 501);
	CHECK(Status(Request("get / HTTP/1.1\r\nHost: t\r\n\r\n")) == 501);
	CHECK(Status(Request("GET / HTTP/1.1\r\n\r\n")) == 400);
	CHECK(Status(Request("GET / HTTP/2.0\r\nHost: t\r\n\r\n")) == 505);
	CHECK(Status(Request("GET / HTTP/1.2\r\nHost: t\r\n\r\n")) == 505);
	CHECK(Status(Request("GET / JUNK\r\nHost: t\r\n\r\n")) == 400);
	CHECK(Status(Request("GET /\r\nHost: t\r\n\r\n")) == 400);
	CHECK(Status(Request("GET\r\nHost: t\r\n\r\n")) == 400);
	CHECK(Status(Request("\r\n\r\n")) == 400);
	CHECK(Status(Request(" / HTTP/1.1\r\n\r\n")) == 400);
	CHECK(Status(Request("GET  / HTTP/1.1\r\nHost: t\r\n\r\n")) == 400);
	CHECK(Status(Request("GET /a b HTTP/1.1\r\nHost: t\r\n\r\n")) == 400);
	CHECK(Status(Request("GET http://t/ HTTP/1.1\r\nHost: t\r\n\r\n")) == 400);
	CHECK(Status(Request("GET * HTTP/1.1\r\nHost: t\r\n\r\n")) == 400);
	CHECK(Status(Request(std::string("GET /\0 HTTP/1.1\r\nHost: t\r\n\r\n", 28))) == 400);
	CHECK(Status(Request(std::string("GET / HTTP/1.1\r\nHost: t\r\nX: a\0b\r\n\r\n", 35))) == 400);
	CHECK(Status(Request(std::string("GET /\x01 HTTP/1.1\r\nHost: t\r\n\r\n"))) == 400);
	CHECK(Status(Request(std::string("GET /\xFF HTTP/1.1\r\nHost: t\r\n\r\n"))) == 400);
	CHECK(Status(Request("POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: x\r\n\r\n")) == 400);
	CHECK(Status(Request("POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: -1\r\n\r\n")) == 400);
	CHECK(Status(Request("POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length:\r\n\r\n")) == 400);
	CHECK(Status(Request("POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 1\r\nContent-Length: 1\r\n\r\na")) == 400);
	CHECK(Status(Request("POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 99999999999999999999\r\n\r\n")) == 413);
	CHECK(Status(Request("POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 720\r\n\r\n")) == 413);
	CHECK(Status(Request("POST /echo HTTP/1.1\r\nHost: t\r\nTransfer-Encoding: chunked\r\n\r\n")) == 501);

	// Responses the application got wrong
	CHECK(Status(GET("/overflow")) == 500);
	CHECK(Status(GET("/badstatus")) == 500);
	r = GET("/nodata");
	CHECK(Status(r) == 200 && r.find("Content-Length: 0\r\n") != std::string::npos);
	r = GET("/longhead");
	CHECK(Status(r) == 500 && Body(r).empty() && r.find("hhhh") == std::string::npos);
	r = GET("/exact");
	CHECK(Status(r) == 200 && Body(r).size() > 600 && Body(r).find_first_not_of('y') == std::string::npos);

	// The largest request head fitting the buffer, and one past it
	std::string big = "GET / HTTP/1.1\r\nHost: t\r\nX: ";

	big.append(BUFF_SIZE - 1U - big.size() - 4U, 'a');
	big += "\r\n\r\n";
	CHECK(big.size() == BUFF_SIZE - 1U);
	CHECK(Status(Request(big)) == 200);
	big = "GET / HTTP/1.1\r\nHost: t\r\nX: ";
	big.append(BUFF_SIZE, 'a');
	big += "\r\n\r\n";
	CHECK(Status(Request(big)) == 431);
	big = "GET /";
	big.append(BUFF_SIZE, 'a');
	CHECK(Status(Request(big)) == 431);

	// The largest body fitting, one byte for the 0
	std::string head = "POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: ";
	int room = BUFF_SIZE - 1 - (int)head.size() - 3 - 4;

	big = head + std::to_string(room) + "\r\n\r\n";
	CHECK((int)big.size() == (int)head.size() + 3 + 4);
	CHECK(Status(Request(big + std::string(room, 'z'))) == 201);
	big = head + std::to_string(room + 1) + "\r\n\r\n";
	CHECK(Status(Request(big)) == 413);
}

static void TestPartial(void)
{
	std::string r;

	Start(s_Routes, sizeof(s_Routes) / sizeof(s_Routes[0]));

	// A request arriving one byte at a time
	SockModel_t *sock = Accept();
	const std::string text = "POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 4\r\n\r\nbody";

	for (size_t i = 0; i < text.size(); i++)
	{
		sock->Rx = text.substr(i, 1);
		Pump();
		CHECK(i + 1 == text.size() || sock->Tx.empty());
	}
	Pump();
	CHECK(Status(sock->Tx) == 201 && Body(sock->Tx) == "body");

	// A large page through a socket taking little at a time and refusing some
	for (size_t i = 0; i < sizeof(s_Large); i++)
	{
		s_Large[i] = (char)(i * 31 + (i >> 8));
	}
	sock = Accept();
	sock->TxChunk = 7;
	sock->TxRefuse = 3;
	sock->RxChunk = 3;
	sock->Rx = "GET /large HTTP/1.1\r\nHost: t\r\n\r\n";
	Pump(4);
	for (int i = 0; i < 100 && sock->pDev != nullptr && sock->pDev->bConnected; i++)
	{
		sock->TxRefuse = i % 3;
		Pump();
	}
	CHECK(Status(sock->Tx) == 200);
	CHECK(Body(sock->Tx) == std::string(s_Large, sizeof(s_Large)));

	// The head alone taking several writes
	sock = Accept();
	sock->TxChunk = 1;
	sock->Rx = "GET / HTTP/1.1\r\nHost: t\r\n\r\n";
	Pump(2);
	CHECK(Status(sock->Tx) == 200 && Body(sock->Tx) == s_Page);
}

static void TestLifetime(void)
{
	std::string r;
	SockModel_t *a;
	SockModel_t *b;

	// Two connections at once, each keeps its own request
	Start(s_Routes, sizeof(s_Routes) / sizeof(s_Routes[0]));
	a = Accept();
	b = Accept();
	CHECK(a != nullptr && b != nullptr && a != b);
	CHECK(Accept() == nullptr);
	a->Rx = "GET /hel";
	b->Rx = "GET /hello?name=b HTTP/1.1\r\nHost: t\r\n\r\n";
	Pump();
	CHECK(Body(b->Tx) == "hello b 42");
	a->Rx = "lo?name=a HTTP/1.1\r\nHost: t\r\n\r\n";
	Pump();
	CHECK(Body(a->Tx) == "hello a 42");
	// Both listen again
	CHECK(s_Dev.pConn != nullptr);
	CHECK(Accept() != nullptr);
	Pump(1);
	HttpServerClose(&s_Dev);
	CHECK(s_Sock[0].pDev == nullptr && s_Sock[1].pDev == nullptr);

	// A peer that sends nothing gets 408 after the time limit, also over a wrap
	Start(s_Routes, sizeof(s_Routes) / sizeof(s_Routes[0]), 1000);
	s_Now = 0xFFFFFF00UL;
	a = Accept();
	Pump(1);
	a->Rx = "GET / HT";
	Pump(99);
	CHECK(a->Tx.empty() && s_Now < 0x10000000UL);
	Pump(2);
	CHECK(Status(a->Tx) == 408);

	// A peer leaving in the middle of a request: no response, the socket listens again
	s_Closes = 0;
	a = Accept();
	a->Rx = "GET / HT";
	Pump();
	a->pDev->bConnected = false;
	Pump();
	CHECK(a->Tx.empty() && s_Closes == 1 && a->pDev != nullptr && a->pDev->bListening);

	// Connected and gone before the server noticed
	s_Closes = 0;
	a = Accept();
	a->pDev->bConnected = false;
	Pump();
	CHECK(s_Closes == 1 && a->pDev->bListening);

	// A whole request followed by the peer leaving: nothing to send to
	s_Closes = 0;
	a = Accept();
	a->Rx = "GET / HTTP/1.1\r\nHost: t\r\n\r\n";
	a->pDev->bConnected = false;
	Pump();
	CHECK(a->Tx.empty() && s_Closes == 1);

	// A peer not taking the response is dropped after the time limit
	a = Accept();
	a->TxRefuse = 1000000;
	a->Rx = "GET /large HTTP/1.1\r\nHost: t\r\n\r\n";
	s_Closes = 0;
	Pump(99);
	CHECK(s_Closes == 0);
	Pump(3);
	CHECK(s_Closes == 1 && a->Tx.empty());

	// A peer taking the response slowly stays, however long the whole of it takes
	a = Accept();
	a->TxBudget = 0;
	a->Rx = "GET /large HTTP/1.1\r\nHost: t\r\n\r\n";
	s_Closes = 0;
	for (int i = 0; i < 200 && s_Closes == 0; i++)
	{
		a->TxBudget = 300;
		Pump(1, 500);
	}
	CHECK(s_Closes == 1 && Status(a->Tx) == 200);
	CHECK(Body(a->Tx).size() == sizeof(s_Large));

	// Listen failing is tried again, on each pass
	HttpServerClose(&s_Dev);
	s_ListenFail = true;
	Pump(3);
	CHECK(s_Sock[0].pDev == nullptr);
	s_ListenFail = false;
	Pump();
	CHECK(s_Sock[0].pDev != nullptr && s_Sock[1].pDev != nullptr);

	// The body of a refused POST is read before closing
	Start(s_Routes, sizeof(s_Routes) / sizeof(s_Routes[0]));
	s_Closes = 0;
	a = Accept();
	a->Rx = "POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 2000\r\n\r\n";
	a->Rx += std::string(300, 'p');
	Pump();
	CHECK(Status(a->Tx) == 413 && s_Closes == 0);
	Pump();
	CHECK(s_Closes == 0);
	a->Rx = std::string(500, 'q');
	Pump();
	CHECK(a->Rx.empty() && s_Closes == 0);
	a->Rx = std::string(1200, 'r');
	Pump();
	CHECK(a->Rx.empty() && s_Closes == 0);
	Pump();
	CHECK(s_Closes == 1);

	// More data than it announced, read for as long as it comes
	s_Closes = 0;
	a = Accept();
	a->Rx = "POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 900\r\n\r\n";
	a->Rx += std::string(700, 'p');
	Pump();
	CHECK(Status(a->Tx) == 413 && s_Closes == 0);
	a->Rx = std::string(5000, 'q');
	Pump();
	CHECK(s_Closes == 0);
	Pump();
	CHECK(s_Closes == 1);

	// A body announced and never sent holds the connection only up to the time limit
	Start(s_Routes, sizeof(s_Routes) / sizeof(s_Routes[0]), 1000);
	s_Closes = 0;
	a = Accept();
	a->Rx = "POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 99999999999999\r\n\r\n";
	Pump(98);
	CHECK(Status(a->Tx) == 413 && s_Closes == 0);
	Pump(3);
	CHECK(s_Closes == 1);

	// Any path route
	Start(s_RoutesAny, 2);
	r = GET("/");
	CHECK(Status(r) == 200);
	r = GET("/any/thing");
	CHECK(Status(r) == 404 && Body(r) == "custom /any/thing");
	r = Request("POST /x HTTP/1.1\r\nHost: t\r\n\r\n");
	CHECK(Status(r) == 405);
}

static void TestParam(void)
{
	char v[16];

	CHECK(HttpParamGet("a=1&b=2", "a", v, sizeof(v)) == 1 && strcmp(v, "1") == 0);
	CHECK(HttpParamGet("a=1&b=2", "b", v, sizeof(v)) == 1 && strcmp(v, "2") == 0);
	CHECK(HttpParamGet("a=1&b=2", "c", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("&&a=x&", "a", v, sizeof(v)) == 1 && v[0] == 'x');
	CHECK(HttpParamGet("a=", "a", v, sizeof(v)) == 0 && v[0] == 0);
	CHECK(HttpParamGet("ab=1", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=1", "ab", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("A=1", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=x%20y+z", "a", v, sizeof(v)) == 5 && strcmp(v, "x y z") == 0);
	CHECK(HttpParamGet("a=%41%6a%6A", "a", v, sizeof(v)) == 3 && strcmp(v, "Ajj") == 0);
	CHECK(HttpParamGet("a=%4", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=%", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=%zz", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=%4&b=11", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=%00", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=123456789012345", "a", v, sizeof(v)) == 15);
	CHECK(HttpParamGet("a=1234567890123456", "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=1", "a", v, 1) == -1);
	CHECK(HttpParamGet("a=", "a", v, 1) == 0);
	CHECK(HttpParamGet("a=1", "a", v, 0) == -1);
	CHECK(HttpParamGet(nullptr, "a", v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=1", nullptr, v, sizeof(v)) == -1);
	CHECK(HttpParamGet("a=1", "a", nullptr, sizeof(v)) == -1);
}

// Random bytes and damaged requests, in random pieces. ASan and UBSan watch the memory.
// The server must never answer anything but a well formed status line.
static void TestRandom(void)
{
	static const char *s_Seeds[] = {
		"GET / HTTP/1.1\r\nHost: t\r\n\r\n",
		"POST /echo HTTP/1.1\r\nHost: t\r\nContent-Length: 5\r\n\r\nhello",
		"HEAD /hello?name=%41 HTTP/1.0\r\nX-Test: a\r\n\r\n",
		"GET /header HTTP/1.1\r\nHost: t\r\nX-Test:\r\n\r\n",
	};
	unsigned seed = 12345;

	Start(s_Routes, sizeof(s_Routes) / sizeof(s_Routes[0]), 100);
	for (int run = 0; run < 200000; run++)
	{
		std::string text;
		int kind = rand_r(&seed) % 3;

		if (kind == 0)
		{
			int n = rand_r(&seed) % 900;

			for (int i = 0; i < n; i++)
			{
				text += (char)rand_r(&seed);
			}
		}
		else
		{
			text = s_Seeds[rand_r(&seed) % 4];
			int damage = rand_r(&seed) % 6;

			for (int i = 0; i < damage && !text.empty(); i++)
			{
				size_t pos = (size_t)rand_r(&seed) % text.size();

				switch (rand_r(&seed) % 4)
				{
					case 0:
						text[pos] = (char)rand_r(&seed);
						break;
					case 1:
						text.erase(pos, (size_t)rand_r(&seed) % 8);
						break;
					case 2:
						text.insert(pos, (size_t)rand_r(&seed) % 4, (char)rand_r(&seed));
						break;
					default:
						text.insert(pos, "\r\n");
						break;
				}
			}
		}

		SockModel_t *sock = Accept();

		CHECK(sock != nullptr);
		sock->RxChunk = 1 + rand_r(&seed) % 200;
		sock->TxChunk = 1 + rand_r(&seed) % 200;
		sock->Rx = text;
		for (int i = 0; i < 40 && sock->pDev != nullptr && sock->pDev->bListening == false; i++)
		{
			Pump(1, 5);
		}
		if (!sock->Tx.empty())
		{
			int status = Status(sock->Tx);

			CHECK(status >= 200 && status <= 599);
			CHECK(sock->Tx.find("\r\n\r\n") != std::string::npos);
		}
		sock->pDev->bConnected = false;
		Pump(2, 5);
	}
}

int main(void)
{
	TestConfig();
	TestBasic();
	TestErrors();
	TestPartial();
	TestLifetime();
	TestParam();
	TestRandom();
	HttpServerClose(&s_Dev);
	printf("HttpServer tests: %d checks PASS\n", s_Checks);

	return 0;
}
