#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "nrf.h"

#define CRYPTO_TEST_TICKS_INIT() do { CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; } while (0)
#define CRYPTO_TEST_TICKS() (DWT->CYCCNT)
#define CRYPTO_TEST_TICKS_PER_US 64U
#define UART_DEVNO 0
#define UART_RX_PINOP 1
#define UART_TX_PINOP 1
#if defined(NRF5340_DK_PCA10095)
#define UART_RX_PORT 1
#define UART_RX_PIN 0
#define UART_TX_PORT 1
#define UART_TX_PIN 1
#else
#define UART_RX_PORT 1
#define UART_RX_PIN 8
#define UART_TX_PORT 1
#define UART_TX_PIN 9
#endif

#endif
