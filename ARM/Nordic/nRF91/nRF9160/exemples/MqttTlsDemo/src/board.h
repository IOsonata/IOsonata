/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Mqtt modem TLS example configuration: nRF9160 Thingy:91 or DK.
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "net_demo_tls_ca.h"

// Select one board. Thingy:91 needs its working nRF52840 UART-to-USB bridge.
//#define NRF9160_DK
#define NORDIC_THINGY91

#if defined(NORDIC_THINGY91)
#define NET_DEMO_UART_RX_PIN		19
#define NET_DEMO_UART_TX_PIN		18
#elif defined(NRF9160_DK)
#define NET_DEMO_UART_RX_PIN		28
#define NET_DEMO_UART_TX_PIN		29
#else
#error "Select NRF9160_DK or NORDIC_THINGY91"
#endif

// UART0, 115200 baud, 8N1, no flow control.
#define NET_DEMO_UART_PINS { \
	{0, NET_DEMO_UART_RX_PIN, 1, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, NET_DEMO_UART_TX_PIN, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define NET_DEMO_MEDIUM			NET_DEMO_MEDIUM_LTE
#define NET_DEMO_UART_DEVNO		0
#define NET_DEMO_UART_RATE		115200U
#define NET_DEMO_UART_DMA			true
#define NET_DEMO_INT_PRIO			5
#define NET_DEMO_TIMER_DEVNO		0
#define NET_DEMO_TIMER_FREQ		0
#define NET_DEMO_RNG_HEADER		"crypto_rng_nrf.h"
#define NET_DEMO_RNG()			CryptoRngNrfInstance()

// nullptr uses the network default APN; supply a string if the SIM requires it.
#define NET_DEMO_APN				nullptr
#define NET_DEMO_RAT				LTE_RAT_LTEM
#define NET_DEMO_RAT_PREF			LTE_RAT_LTEM

// Reserve this modem credential tag for the demo CA.
#define NET_DEMO_MODEM_TLS		1
#define NET_DEMO_TLS_SEC_TAG		42420
#define NET_DEMO_TLS_CA_CERT		g_NetDemoMosquittoCa
#define NET_DEMO_TLS_REPLACE_CA	0

// Public test broker over modem TLS. Synthetic data uses a random topic.
#define MQTT_DEMO_HOST			"test.mosquitto.org"
#define MQTT_DEMO_PORT			8883
#define MQTT_DEMO_INTERVAL		10000U

#endif
