/* SPDX-License-Identifier: MIT
 * Host configuration for net_demo_setup_test.cpp only.
 */
#ifndef __NET_DEMO_TEST_BOARD_H__
#define __NET_DEMO_TEST_BOARD_H__

#define NET_DEMO_MEDIUM		NET_DEMO_TEST_MEDIUM
#define NET_DEMO_UART_DEVNO	0
#define NET_DEMO_UART_PINS	{ \
	{0, 1, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 2, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}
#define NET_DEMO_UART_RATE	115200
#define NET_DEMO_UART_DMA		true
#define NET_DEMO_INT_PRIO		5
#define NET_DEMO_TIMER_DEVNO	0
#define NET_DEMO_TIMER_FREQ	1000
#define NET_DEMO_RNG_HEADER	"net_demo_test_rng.h"
#define NET_DEMO_RNG()		NetDemoTestRng()
#define NET_DEMO_APN			nullptr
#define NET_DEMO_RAT			LTE_RAT_LTEM
#define NET_DEMO_RAT_PREF		LTE_RAT_NONE
#define ETHERNET_PINS		NET_DEMO_UART_PINS
#define ETHERNET_MAC_ADDR		{2, 0, 0, 0, 0, 1}
#define ETHERNET_PHY_ADDR		0
#define NET_DEMO_IP_ADDR		{192, 0, 2, 20}
#define NET_DEMO_NETMASK		{255, 255, 255, 0}
#define NET_DEMO_GATEWAY		{192, 0, 2, 1}
#define NET_DEMO_DNS			{192, 0, 2, 1}
#define NET_DEMO_DHCP			true

#endif
