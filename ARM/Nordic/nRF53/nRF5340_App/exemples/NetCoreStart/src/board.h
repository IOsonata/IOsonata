/**-------------------------------------------------------------------------
@file	board.h

@brief	Board definitions of the network core starter, nRF5340 Audio DK or DK application core

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

@license	MIT, Copyright (c) 2026, I-SYST inc.

----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

// Define NRF5340_DK_PCA10095 in the project symbols for the standard nRF5340 DK.
// Default is the nRF5340 Audio DK.
#if defined(NRF5340_DK_PCA10095)
// Network-core console on the DK VCOM.
#define NET_UART_PORT	1
#define NET_UART_RX_PIN	0
#define NET_UART_TX_PIN	1

// DK LED1, active low
#define LED1_PORT		0
#define LED1_PIN		28
#define LED1_PINOP		0
#define LED1_ACTIVE_LOW	1
#else
// Network-core console on the Audio DK VCOM.
#define NET_UART_PORT	1
#define NET_UART_RX_PIN	8
#define NET_UART_TX_PIN	9

// Audio DK RGB1 green, active high
#define LED1_PORT		0
#define LED1_PIN		25
#define LED1_PINOP		0
#define LED1_ACTIVE_LOW	0
#endif

#endif // __BOARD_H__
