#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "nrf.h"
#include "crypto_ecb_nrf53.h"
#include "crypto_rng_nrf.h"

#define CRYPTO_TEST_AES_TYPE		CryptoEcbNrf53
#define CRYPTO_TEST_AES_MEMSIZE		CRYPTO_ECB_NRF53_MEMSIZE
#define CRYPTO_TEST_AES_CREATE(m, s)	CryptoEcbNrf53Create((m), (s))
#define CRYPTO_TEST_RNG_TYPE		CryptoRngNrf
#define CRYPTO_TEST_RNG_MEMSIZE		sizeof(CryptoRngNrf)
#define CRYPTO_TEST_RNG_CREATE(m, s)	CryptoRngNrfInstance()
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
