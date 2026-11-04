/**-------------------------------------------------------------------------
@file	http_server.h

@brief	Small HTTP/1.1 server on TCP sockets that wait for a peer.

The application gives a table of routes. A route serves a fixed page from flash
or calls a handler, which answers from the buffer of the connection or from its
own memory. There is no file system and no heap: all the memory comes from the
application, HTTP_SERVER_MEMSIZE(NbConn, BuffSize) bytes.

Each connection answers one request and closes, "Connection: close". NbConn
requests are in progress at once. A browser opening more connections than that
waits for a free one, TCP retries for it.

BuffSize limits one request, head and body together. What it does not use is left
for the handler to build its answer. A browser sends 500 to 1000 bytes of head,
so give it 1536 bytes or more.

HttpServerProcess does the work. Call it often from one foreground owner, the
same one as the IP service. How long it takes depends on the handlers: it waits
about 1 ms per connection when a socket cannot take more. A connection that gets
no complete request or takes no data for the Timeout of the configuration is
closed.

The server works on SockIntrf and SockIntrfListen, so it needs a network
implementation that can listen. It is plain HTTP without authentication: anyone
reaching the port can call the routes. A handler can read the Authorization
header with HttpHeaderGet, which is of little use without TLS. Do not expose it
to a network that is not trusted.

Not supported: keep-alive, chunked requests and responses, TLS. A client that
closes its sending side after the request gets no response.

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
#ifndef __HTTP_SERVER_H__
#define __HTTP_SERVER_H__

#include <stdint.h>
#include <stdbool.h>

#include "net/sock_intrf.h"

/// Status line and header lines of a response
#define HTTP_SERVER_HEAD_MAX		256
/// Smallest buffer per connection the server accepts
#define HTTP_SERVER_BUFF_MIN		256

/// Request methods. As bits they form the Methods of a route.
typedef enum __Http_Method {
	HTTP_METHOD_GET = 1,
	HTTP_METHOD_HEAD = 2,					//!< Follows GET, a route does not need to list it
	HTTP_METHOD_POST = 4,
	HTTP_METHOD_PUT = 8,
	HTTP_METHOD_DELETE = 16,
} HTTP_METHOD;

#pragma pack(push, 4)

/// A request as a handler sees it. Valid only during the call.
typedef struct __Http_Request {
	HTTP_METHOD Method;						//!< On HEAD, the body the handler gives is not sent
	const char *pPath;						//!< Path as sent, not decoded
	const char *pQuery;						//!< After the '?', not decoded, "" when none
	const char *pHead;						//!< Header lines, each ending in CRLF, see HttpHeaderGet
	uint32_t HeadLen;						//!< Length of the header lines
	const char *pBody;						//!< Body, followed by a 0, "" when none
	uint32_t BodyLen;						//!< Length of the body
} HttpRequest_t;

/// The response a handler fills. Holds the values of the route on entry.
typedef struct __Http_Response {
	uint16_t Status;						//!< 200 on entry
	const char *pType;						//!< Content type
	const char *pHeaders;					//!< More header lines, each ending in CRLF, may be NULL
	const void *pData;						//!< Body: pBuff or memory valid until the connection closes
	uint32_t Len;							//!< Length of the body
	char *pBuff;							//!< Free part of the connection buffer
	uint32_t BuffSize;						//!< Its size
} HttpResponse_t;

/**
 * @brief	Answer a request.
 *
 * Runs in HttpServerProcess and holds up everything in the foreground until it
 * returns. HttpPrintf builds a body. A body too large for the buffer is answered
 * with status 500.
 *
 * @param	pReq	: The request
 * @param	pRsp	: The response to fill
 * @param	pCtx	: pCtx of the route
 */
typedef void (*HttpHandler_t)(const HttpRequest_t *pReq, HttpResponse_t *pRsp, void *pCtx);

typedef struct __Http_Route {
	const char *pPath;						//!< Exact path of the request, NULL for any, put last
	uint8_t Methods;						//!< HTTP_METHOD_ bits, 0 for GET
	const char *pType;						//!< Content type, may be NULL for a handler to set
	const void *pData;						//!< Fixed body, in flash for example, may be NULL
	uint32_t DataLen;						//!< Length of the fixed body
	const char *pHeaders;					//!< More header lines, each ending in CRLF, may be NULL
	HttpHandler_t Handler;					//!< Called for the request, NULL to send the fixed body
	void *pCtx;								//!< Passed to the handler
} HttpRoute_t;

typedef struct __Http_Server_Cfg {
	uint16_t Port;							//!< TCP port, 0 for 80
	const HttpRoute_t *pRoutes;				//!< Routes, first match applies. Kept, not copied.
	int NbRoute;							//!< Number of routes
	uint8_t *pMem;							//!< Memory, 4 bytes aligned, valid until Close
	uint32_t MemSize;						//!< Its size, HTTP_SERVER_MEMSIZE(NbConn, BuffSize)
	int NbConn;								//!< Number of connections. A socket for each must be free.
	uint32_t Timeout;						//!< Limit in ms on a stalled peer, 0 for 5000
} HttpServerCfg_t;

/// One connection. Public only for HTTP_SERVER_MEMSIZE.
typedef struct __Http_Server_Conn {
	SockIntrfDev_t Sock;
	HttpRequest_t Req;
	char *pBuff;
	const uint8_t *pTx;
	uint32_t TxLen;
	uint32_t TxOff;
	uint32_t RxLen;
	uint32_t ReqLen;						//!< Length of head and body, known once the head is complete
	uint32_t DrainLen;						//!< Body of a refused request still to come from the peer
	uint32_t Started;
	uint16_t HeadSize;
	uint16_t HeadOff;
	uint8_t State;
	char Head[HTTP_SERVER_HEAD_MAX];
} HttpServerConn_t;

/// Zero-initialize. One foreground owner, no concurrent calls.
typedef struct __Http_Server_Dev {
	const HttpRoute_t *pRoutes;
	int NbRoute;
	HttpServerConn_t *pConn;
	int NbConn;
	uint32_t ConnSize;
	uint32_t BuffSize;
	uint32_t Timeout;
	uint16_t Port;
} HttpServerDev_t;

#pragma pack(pop)

/// Memory for HttpServerCfg_t pMem
#define HTTP_SERVER_MEMSIZE(NbConn, BuffSize) \
	((uint32_t)(NbConn) * (uint32_t)(sizeof(HttpServerConn_t) + (((BuffSize) + 3U) & ~3U)))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Prepare the server. The first HttpServerProcess starts listening.
 *
 * @param	pDev	: Server
 * @param	pCfg	: Configuration
 *
 * @return	true - configuration accepted
 */
bool HttpServerInit(HttpServerDev_t * const pDev, const HttpServerCfg_t * const pCfg);

/**
 * @brief	Listen, read requests and send responses.
 *
 * The network going down ends the connections and the server takes new ones when
 * it returns. The application does not need to tell it.
 *
 * @param	pDev	: Server
 * @param	Now		: Time in ms, may wrap around
 */
void HttpServerProcess(HttpServerDev_t * const pDev, uint32_t Now);

/// Close all connections and stop listening. HttpServerProcess starts again.
void HttpServerClose(HttpServerDev_t * const pDev);

/**
 * @brief	Add to the body of a response, in the buffer of the connection.
 *
 * @param	pRsp	: Response the handler received
 * @param	pFormat	: As printf
 *
 * @return	true - added. false - no room, the request gets status 500 unless
 * 			the handler sets another body.
 */
bool HttpPrintf(HttpResponse_t * const pRsp, const char *pFormat, ...) __attribute__((format(printf, 2, 3)));

/**
 * @brief	Find a header of the request.
 *
 * @param	pReq	: The request
 * @param	pName	: Name of the header, any case, without the ':'
 * @param	pLen	: Length of the value
 *
 * @return	The value, not followed by a 0, without the spaces around it. NULL
 * 			when the request does not have the header.
 */
const char *HttpHeaderGet(const HttpRequest_t * const pReq, const char *pName, uint32_t * const pLen);

/**
 * @brief	Get a value from a query or a form body, as "a=1&b=2%203".
 *
 * Decodes %xx and '+'. The name is compared exactly.
 *
 * @param	pParams	: pQuery or pBody of the request
 * @param	pName	: Name of the value
 * @param	pVal	: Decoded value, 0 terminated
 * @param	ValSize	: Size of pVal
 *
 * @return	Length of the value. -1 when it is missing, does not fit, or decodes
 * 			to something other than text (a bad %xx, or %00).
 */
int HttpParamGet(const char *pParams, const char *pName, char *pVal, int ValSize);

#ifdef __cplusplus
}

#include "device.h"

/// C++ wrapper of the same server
class HttpServer : public Device {
public:
	HttpServer() = default;
	~HttpServer() { HttpServerClose(&vDev); }
	HttpServer(const HttpServer &) = delete;
	HttpServer &operator=(const HttpServer &) = delete;

	bool Init(const HttpServerCfg_t &Cfg) {
		vbEnabled = HttpServerInit(&vDev, &Cfg);
		Valid(vbEnabled);
		return vbEnabled;
	}
	bool Enable() override {
		vbEnabled = vDev.pConn != nullptr;
		return vbEnabled;
	}
	void Disable() override {
		HttpServerClose(&vDev);
		vbEnabled = false;
	}
	void Reset() override { HttpServerClose(&vDev); }
	void PowerOff() override { Disable(); }

	void Process(uint32_t Now) {
		if (vbEnabled)
		{
			HttpServerProcess(&vDev, Now);
		}
	}

private:
	HttpServerDev_t vDev = {};
	bool vbEnabled = false;
};

#endif

#endif // __HTTP_SERVER_H__
