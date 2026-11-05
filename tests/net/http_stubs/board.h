/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Board of the host run of exemples/net/http_server.cpp on a TAP interface.
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#define UART_DEVNO				0
#define UART_PINS				{ {0, 0, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} }
#define ETHERNET_TIMER_DEVNO	0
#define ETHERNET_PHY_ADDR		0
#define ETHERNET_MAC_ADDR		{0x02, 0x00, 0x00, 0x99, 0x00, 0x02}
#define ETHERNET_PINS			{ {0, 0, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} }
#define LED_PINS_MAP			{ {0, 1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} }
#define HTTP_DEMO_DHCP			false
#define HTTP_DEMO_IP_ADDR		{10, 99, 0, 2}
#define HTTP_DEMO_NETMASK		{255, 255, 255, 0}
#define HTTP_DEMO_GATEWAY		{10, 99, 0, 1}

#endif
