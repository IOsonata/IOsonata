/**-------------------------------------------------------------------------
@file	board.h

@brief	CoAP over LTE-M on Nordic Thingy:91 or the nRF9160 DK.

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

// Read-only public client test. Set PUBLIC_TEST to 0 for the full exercise
// against Python/coap_server.py and select your reachable helper host.
#define COAP_DEMO_PUBLIC_TEST	1
#define COAP_DEMO_HOST			"coap.me"
#define COAP_DEMO_RESOURCE		"test"
#define COAP_DEMO_PORT			5683
#define COAP_DEMO_INTERVAL		10000U
// Zero selects the library's default 1-byte/second unanswered-peer rate.
#define COAP_DEMO_PROBING_RATE	0U
#define COAP_DEMO_RNG_HEADER		"crypto_rng_nrf.h"
#define COAP_DEMO_RNG()			CryptoRngNrfInstance()

// NULL uses the network's default APN; set an APN string if the SIM needs it.
// LTE-M matches the existing Thingy:91 validation configuration.
#define LTE_COAP_APN			nullptr
#define LTE_COAP_RAT			LTE_RAT_LTEM
#define LTE_COAP_RAT_PREF	LTE_RAT_LTEM

// Board selection: define one. The Thingy:91 console needs its nRF52840
// Connectivity Bridge or HciController firmware to forward UART0 to USB.
//#define NRF9160_DK
#define NORDIC_THINGY91

// Diagnostic console: UART0, 1000000 baud, 8N1, no flow control.
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PINOP		1

#if defined(NORDIC_THINGY91)
#define UART_RX_PIN			19
#define UART_TX_PIN			18
#elif defined(NRF9160_DK)
#define UART_RX_PIN			28
#define UART_TX_PIN			29
#else
#error "Select the board: NRF9160_DK or NORDIC_THINGY91"
#endif

// RTC0: modem wait trigger 0 and application pump trigger 1.
#define LTE_TIMER_DEVNO		0

#endif
