/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Mqtt generic example configuration: nRF9151 DK or nRF9161 DK.
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

// nRF9151 DK / nRF9161 DK: UART0 through VCOM0, 115200 baud, 8N1.
#define NET_DEMO_UART_PINS { \
	{0, 26, 1, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 27, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
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

// Public test broker. The application sends synthetic data on a random topic.
#define MQTT_DEMO_HOST			"test.mosquitto.org"
#define MQTT_DEMO_PORT			1883
#define MQTT_DEMO_INTERVAL		10000U

#endif
