/**-------------------------------------------------------------------------
@file	board.h

@brief	MQTT over LTE-M on the Nordic nRF9161 DK and nRF9151 DK.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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

#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

// Public test broker: plaintext TCP, no username/password. Synthetic test
// payloads only; the application generates a random client ID and topic.
#define MQTT_DEMO_HOST			"test.mosquitto.org"
#define MQTT_DEMO_PORT			1883
#define MQTT_DEMO_INTERVAL		10000U
#define MQTT_DEMO_RNG_HEADER		"crypto_rng_nrf.h"
#define MQTT_DEMO_RNG()			CryptoRngNrfInstance()

// NULL uses the network's default APN; set an APN string if the SIM needs it.
#define LTE_MQTT_APN				nullptr
#define LTE_MQTT_RAT				LTE_RAT_LTEM
#define LTE_MQTT_RAT_PREF		LTE_RAT_LTEM

// Both DKs use UART0 on the interface MCU's VCOM0.
// Diagnostic console: 1000000 baud, 8N1, no flow control.
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			26
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			27
#define UART_TX_PINOP		1

// RTC0: modem wait trigger 0 and application pump trigger 1.
#define LTE_TIMER_DEVNO		0

#endif
