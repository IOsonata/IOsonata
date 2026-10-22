/**-------------------------------------------------------------------------
@file	lwipopts.h

@brief	Fixed-pool, bare-metal lwIP profile for IOsonata MCU libraries.

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

#ifndef __IOSONATA_LWIPOPTS_H__
#define __IOSONATA_LWIPOPTS_H__

#define NO_SYS					1
#define SYS_LIGHTWEIGHT_PROT		0
#define LWIP_SOCKET				0
#define LWIP_NETCONN			0
#define LWIP_IPV4				1
#define LWIP_IPV6				0
#define LWIP_ARP				1
#define LWIP_ICMP				1
#define LWIP_DHCP				1
#define LWIP_DNS				1
#define LWIP_UDP				1
#define LWIP_TCP				1
#define LWIP_RAW				0
#define LWIP_NETIF_HOSTNAME		0
#define LWIP_NETIF_STATUS_CALLBACK 1
#define LWIP_NETIF_LINK_CALLBACK	0
#define LWIP_STATS				0
#define LWIP_NETIF_LOOPBACK		0
#define IP_REASSEMBLY			0
#define IP_FRAG					0
#define LWIP_IGMP				0
#define MEM_ALIGNMENT			8
#define MEM_LIBC_MALLOC			0
#define MEM_USE_POOLS			1
#define MEMP_USE_CUSTOM_POOLS	1
#define MEM_USE_POOLS_TRY_BIGGER_POOL 1
#define MEMP_MEM_MALLOC			0
#define MEMP_NUM_UDP_PCB			8
#define MEMP_NUM_TCP_PCB			4
#define MEMP_NUM_TCP_PCB_LISTEN	0
#define MEMP_NUM_TCP_SEG			32
#define MEMP_NUM_PBUF			32
#define PBUF_POOL_SIZE			24
#define PBUF_POOL_BUFSIZE		1536
#define TCP_MSS					1460
#define TCP_WND					(4 * TCP_MSS)
#define TCP_SND_BUF				(4 * TCP_MSS)
#define TCP_SND_QUEUELEN			16
#define LWIP_RAND()				IpRandom()

#ifdef __cplusplus
extern "C" {
#endif
unsigned int IpRandom(void);
#ifdef __cplusplus
}
#endif

#endif
