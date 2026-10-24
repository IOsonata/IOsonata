#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

// nRF5340 Audio DK by default; define NRF5340_DK_PCA10095 for DK.
#define UART_RETARGET
#define UART_DEVNO 0
#define UART_RX_PINOP 0
#define UART_TX_PINOP 0
#if defined(NRF5340_DK_PCA10095)
#define UART_RX_PORT 0
#define UART_RX_PIN 22
#define UART_TX_PORT 0
#define UART_TX_PIN 20
#else
#define UART_RX_PORT 1
#define UART_RX_PIN 4
#define UART_TX_PORT 1
#define UART_TX_PIN 5
#endif

#endif
