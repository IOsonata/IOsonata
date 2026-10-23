/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Native TLS certificate MQTT example configuration: NUCLEO-H753ZI.
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

#define BOARD NUCLEO_H753ZI
#include "stm32_boards.h"

#define NET_DEMO_MEDIUM			NET_DEMO_MEDIUM_ETHERNET
#define NET_DEMO_UART_DEVNO		BOARD_VCP_DEVNO
#define NET_DEMO_UART_PINS		BOARD_VCP_PINS
#define NET_DEMO_UART_RATE		1000000U
#define NET_DEMO_UART_DMA			false
#define NET_DEMO_INT_PRIO			6
#define NET_DEMO_TIMER_DEVNO		8
#define NET_DEMO_TIMER_FREQ		100000U
#define NET_DEMO_RNG_HEADER		"crypto_rng_stm32.h"
#define NET_DEMO_RNG()			CryptoRngStm32Instance()

#define NET_DEMO_DHCP			true
// Used when DHCP is disabled; set all four for the attached network.
#define NET_DEMO_IP_ADDR			{192, 168, 1, 50}
#define NET_DEMO_NETMASK			{255, 255, 255, 0}
#define NET_DEMO_GATEWAY			{192, 168, 1, 1}
#define NET_DEMO_DNS				{192, 168, 1, 1}

#define ETHERNET_PHY_ADDR		0
// Locally administered demonstration address; assign a distinct MAC per board.
#define ETHERNET_MAC_ADDR		{0x02, 0x00, 0x00, 0x75, 0x30, 0x01}
// MB1364 H753ZI: RMII AF11. TXD1 uses PB13, not PG14.
#define ETHERNET_PINS { \
	{IOPORTA, 1, IOPINOP_FUNC11, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 2, IOPINOP_FUNC11, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 1, IOPINOP_FUNC11, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 7, IOPINOP_FUNC11, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 4, IOPINOP_FUNC11, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 5, IOPINOP_FUNC11, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTG, 11, IOPINOP_FUNC11, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTG, 13, IOPINOP_FUNC11, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTB, 13, IOPINOP_FUNC11, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Local broker address and its certificate DNS name are configured separately.
// Generate the CA header with Python/mqtt_cert_demo.py before building.
#define NET_DEMO_NATIVE_TLS		1
#define NET_DEMO_TLS_SERVER_NAME	"iosonata.test"
#define MQTT_DEMO_HOST			"10.5.5.51"
#define MQTT_DEMO_PORT			8883
#define MQTT_DEMO_INTERVAL		10000U

#endif
