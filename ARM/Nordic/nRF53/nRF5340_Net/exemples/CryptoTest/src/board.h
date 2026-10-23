#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "crypto_ecb_nrf53.h"

// Hardware AES; software P-256 comparison and deterministic test RNG.
#define CRYPTO_TEST_AES_TYPE CryptoEcbNrf53
#define CRYPTO_TEST_AES_MEMSIZE CRYPTO_ECB_NRF53_MEMSIZE
#define CRYPTO_TEST_AES_CREATE(m, s) CryptoEcbNrf53Create((m), (s))

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
