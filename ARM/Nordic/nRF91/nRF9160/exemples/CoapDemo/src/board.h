/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Coap generic example configuration: nRF9160 Thingy:91 or DK.
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

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

// Public read-only GET. Select 0 and a reachable helper for the full exercise.
#define COAP_DEMO_PUBLIC_TEST	1
#define COAP_DEMO_HOST			"coap.me"
#define COAP_DEMO_RESOURCE		"test"
#define COAP_DEMO_PORT			5683
#define COAP_DEMO_INTERVAL		10000U
// Zero selects the library's default 1-byte/second unanswered-peer rate.
#define COAP_DEMO_PROBING_RATE	0U

#endif
