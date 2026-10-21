/**-------------------------------------------------------------------------
@file	board.h

@brief	NUCLEO-H753ZI crypto test board configuration.

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

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
#define BOARD					NUCLEO_H753ZI
#include "stm32_boards.h"

#define UART_RETARGET
#define UART_DEVNO				BOARD_VCP_DEVNO
#define UART_RX_PORT			IOPORTD
#define UART_RX_PIN				9
#define UART_RX_PINOP			IOPINOP_FUNC7
#define UART_TX_PORT			IOPORTD
#define UART_TX_PIN				8
#define UART_TX_PINOP			IOPINOP_FUNC7

// Hardware RNG, CRYP AES and one-shot HASH SHA-256; software P-256.
#include "crypto_stm32h7.h"
#define CRYPTO_TEST_ENGINE_HEADER	"crypto_rng_stm32.h"
#define CRYPTO_TEST_RNG_TYPE			CryptoRngStm32
#define CRYPTO_TEST_RNG_MEMSIZE		sizeof(CryptoRngStm32)
#define CRYPTO_TEST_RNG_CREATE(m, s)	CryptoRngStm32Instance()

#define CRYPTO_TEST_AES_TYPE CryptoStm32H7Aes
#define CRYPTO_TEST_AES_MEMSIZE CRYPTO_STM32H7_AES_MEMSIZE
#define CRYPTO_TEST_AES_CREATE(m, s) CryptoStm32H7AesCreate((m), (s))
#define CRYPTO_TEST_HASH_TYPE CryptoStm32H7Hash
#define CRYPTO_TEST_HASH_MEMSIZE CRYPTO_STM32H7_HASH_MEMSIZE
#define CRYPTO_TEST_HASH_CREATE(m, s) CryptoStm32H7HashCreate((m), (s))

#endif
