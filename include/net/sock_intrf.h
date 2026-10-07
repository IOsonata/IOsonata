/**-------------------------------------------------------------------------
@file	sock_intrf.h

@brief	Network socket as a DeviceIntrf.

A connected UDP, TCP, DTLS or TLS socket used like any other DeviceIntrf:

	SockIntrf sock;

	sock.Init(cfg);				// name lookup, socket, connect
	sock.Tx(0, pData, Len);		// one datagram, or bytes of the stream
	sock.Rx(0, pBuff, Size);	// what has arrived, 0 when nothing

The socket is connected to one peer at Init, so the DevAddr selector of the
transfers is not used, as on a UART. With UDP and DTLS one TxData sends one
datagram and one RxData returns one datagram, cut to the buffer size.

TxData waits until the data is handed to the network stack, at most the
configuration Timeout. RxData does not wait: it returns 0 when nothing has
arrived. The configuration EvtCB is called in interrupt context with
DEVINTRF_EVT_RX_DATA when data arrives and DEVINTRF_EVT_STATECHG when the
peer or the network closes the socket (SockIntrfConnected then returns
false). Both have no buffer. A transfer on a closed socket returns 0. A
temporary error (no buffer, rate control, timeout) also returns 0 and leaves
the socket open; data left after a close can still be read.

The IP stack and the name lookup belong to the network implementation of
the target, so a port implements this API: on nRF91 the modem
(sock_intrf_nrf91.cpp). TLS and DTLS credentials are stored in it
beforehand with SockIntrfCredWrite, under the security tag the socket
configuration gives. Where the network stack is in a modem, it writes and
deletes them only with the radio off. On LTE, LteInit starts the attach,
so:

	LteInit(&ltecfg);
	if (SockIntrfCredExists(Tag, SOCKINTRF_CRED_CA_CERT) == false)
	{
		LteDisconnect();
		SockIntrfCredWrite(Tag, SOCKINTRF_CRED_CA_CERT, s_CaPem);
		LteConnect();
	}

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
#ifndef __SOCK_INTRF_H__
#define __SOCK_INTRF_H__

#include <stdint.h>
#include <stdbool.h>

#include "device_intrf.h"

/** @addtogroup device_intrf
  * @{
  */

/// Default TxData wait, in ms
#define SOCKINTRF_TIMEOUT_DEFAULT		30000

/// Transport protocol
typedef enum __Sock_Intrf_Proto {
	SOCKINTRF_PROTO_UDP,
	SOCKINTRF_PROTO_TCP,
	SOCKINTRF_PROTO_DTLS,				//!< DTLS 1.2 over UDP
	SOCKINTRF_PROTO_TLS,				//!< TLS 1.2 over TCP
} SOCKINTRF_PROTO;

/// Release assistance hint to the network, LTE (3GPP RAI)
typedef enum __Sock_Intrf_Rai {
	SOCKINTRF_RAI_NO_DATA,				//!< Nothing more to send or receive, release now
	SOCKINTRF_RAI_LAST,					//!< The next send is the last
	SOCKINTRF_RAI_ONE_RESP,				//!< The next send is the last, one response expected
	SOCKINTRF_RAI_ONGOING,				//!< More traffic follows the next send
	SOCKINTRF_RAI_WAIT_MORE,			//!< Keep the connection, more data coming
} SOCKINTRF_RAI;

/// DTLS and TLS credential kinds, stored under a security tag
typedef enum __Sock_Intrf_Cred {
	SOCKINTRF_CRED_CA_CERT,				//!< Certificate of the authority the peer is checked with, PEM
	SOCKINTRF_CRED_CERT,				//!< Own certificate, PEM
	SOCKINTRF_CRED_KEY,					//!< Own private key, PEM
	SOCKINTRF_CRED_PSK,					//!< Pre-shared key, hexadecimal string
	SOCKINTRF_CRED_PSK_ID,				//!< Pre-shared key identity
} SOCKINTRF_CRED;

#pragma pack(push, 4)

typedef struct __Sock_Intrf_Cfg {
	SOCKINTRF_PROTO Proto;				//!< Transport protocol
	const char *pHost;					//!< Peer name or numeric address
	uint16_t Port;						//!< Peer port
	uint16_t LocalPort;					//!< Local port, 0 for any
	int SecTag;							//!< DTLS and TLS: security tag of the credentials
	bool bPeerVerify;					//!< DTLS and TLS: verify the peer certificate
	uint32_t Timeout;					//!< TxData wait in ms, 0 for SOCKINTRF_TIMEOUT_DEFAULT
	DevIntrfEvtHandler_t EvtCB;			//!< Data arrived and socket closed events, may be NULL
} SockIntrfCfg_t;

typedef struct __Sock_Intrf_Dev {
	DevIntrf_t DevIntrf;				//!< This interface
	int Hdl;							//!< Socket handle of the port, -1 when closed
	SOCKINTRF_PROTO Proto;				//!< Transport protocol
	volatile bool bConnected;			//!< Open and not closed by the peer or the network
} SockIntrfDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Look the peer up, open the socket and connect it.
 *
 * Waits for the name lookup and, with TCP and TLS, for the connection. Not
 * from an interrupt. The network must be up (LTE_EVT_REGISTERED on LTE).
 *
 * @param	pDev	: Socket interface to initialize
 * @param	pCfg	: Configuration
 *
 * @return	true - connected
 */
bool SockIntrfInit(SockIntrfDev_t * const pDev, const SockIntrfCfg_t * const pCfg);

/// Close the socket. Init opens it again.
void SockIntrfClose(SockIntrfDev_t * const pDev);

/// true while the socket is open and connected
static inline bool SockIntrfConnected(SockIntrfDev_t * const pDev) { return pDev->bConnected; }

/**
 * @brief	Release assistance hint for the next send.
 *
 * Lets the network release the radio connection early after the last
 * exchange, which saves power with PSM and eDRX. On LTE the modem also needs
 * it enabled (LteCfg_t bRai).
 *
 * @return	true - set
 */
bool SockIntrfRai(SockIntrfDev_t * const pDev, SOCKINTRF_RAI Rai);

/**
 * @brief	Store a credential in the network implementation.
 *
 * Replaces the one of the same kind under the tag. Not from an interrupt.
 * See the file description for when the network takes it.
 *
 * @param	SecTag	: Security tag, the one of the socket configuration
 * @param	Type	: Kind of credential
 * @param	pData	: The credential, 0 terminated, without double quote
 *
 * @return	true - stored
 */
bool SockIntrfCredWrite(int SecTag, SOCKINTRF_CRED Type, const char *pData);

/// Remove a credential, with the radio off as for SockIntrfCredWrite. false
/// when nothing was stored or the network refused.
bool SockIntrfCredDelete(int SecTag, SOCKINTRF_CRED Type);

/// true when a credential of that kind is stored under the tag
bool SockIntrfCredExists(int SecTag, SOCKINTRF_CRED Type);

static inline int SockIntrfRx(SockIntrfDev_t * const pDev, uint8_t *pBuff, int BuffLen) {
	return DeviceIntrfRx(&pDev->DevIntrf, 0, pBuff, BuffLen);
}

static inline int SockIntrfTx(SockIntrfDev_t * const pDev, const uint8_t *pData, int DataLen) {
	return DeviceIntrfTx(&pDev->DevIntrf, 0, pData, DataLen);
}

#ifdef __cplusplus
}

/// C++ wrapper of the same socket interface
class SockIntrf : public DeviceIntrf {
public:
	SockIntrf() { vDevData.Hdl = -1; }
	// The modem interrupt must not reach a destroyed object
	virtual ~SockIntrf() { Close(); }

	bool Init(const SockIntrfCfg_t &Cfg) { return SockIntrfInit(&vDevData, &Cfg); }
	void Close(void) { SockIntrfClose(&vDevData); }
	bool Connected(void) { return SockIntrfConnected(&vDevData); }
	bool Rai(SOCKINTRF_RAI Hint) { return SockIntrfRai(&vDevData, Hint); }

	operator DevIntrf_t * () { return &vDevData.DevIntrf; }
	operator SockIntrfDev_t * () { return &vDevData; }

	// A socket has no transfer rate to set
	uint32_t Rate(uint32_t DataRate) { return DeviceIntrfSetRate(*this, DataRate); }
	uint32_t Rate(void) { return DeviceIntrfGetRate(*this); }

private:
	SockIntrfDev_t vDevData = {};
};

#endif

/** @} End of group device_intrf */

#endif
