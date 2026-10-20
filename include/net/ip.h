/**-------------------------------------------------------------------------
@file	ip.h

@brief	Bare-metal IP service on an injected Ethernet interface.

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

#ifndef __IP_H__
#define __IP_H__

#include "coredev/ethernet.h"
#include "coredev/timer.h"

#pragma pack(push, 4)
typedef struct {
	uint8_t Addr[4];
	uint8_t Netmask[4];
	uint8_t Gateway[4];
	uint8_t Dns[4];
	bool bDhcp;
} IpCfg_t;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// One foreground owner, one Ethernet interface. Timer is already initialized
/// and remains running; TrigNo reserves a 10 ms service trigger until Close.
/// The Ethernet interface is already initialized. This service owns its event
/// callback until Close. IPv4, ARP, ICMP, DHCP, DNS, UDP and TCP use lwIP's raw
/// API and fixed memory pools. No thread or BSD socket API is required.
bool IpInit(const IpCfg_t *pCfg, EthernetDev_t *pEthernet, TimerDev_t *pTimer, int TrigNo);
void IpClose(void);
bool IpReady(void);
bool IpAddress(uint8_t Addr[4]);

/// Foreground, serialized with all SockIntrf calls. Normally called through
/// the queued network event. Delivers socket callbacks deferred by blocking
/// connect/send waits, after those calls return and release transfer locks.
void IpProcess(void);

/// Retry a network event refused by a full application queue. Does not drain
/// the application queue or run the IP stack. AppCheckStatus calls this weakly.
void IpCheckStatus(void);

#ifdef __cplusplus
}
#endif
#endif
