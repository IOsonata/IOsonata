/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

// Board, one of the names in stm32_boards.h. This STM32WBA65 project builds for
// STM32WBA65I_DK1.
#define BOARD				STM32WBA65I_DK1

#include "stm32_boards.h"
#include "coredev/system_core_clock.h"

// Output on USART2, the Arduino D0/D1 pins, also the second ST-LINK virtual
// COM port. The test prints one line per check and the totals.
#define UART_RETARGET
#define UART_DEVNO			BOARD_VCP_DEVNO
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			8
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTB
#define UART_TX_PIN			12
#define UART_TX_PINOP		IOPINOP_FUNC7

// Engines under test: the STM32WBA AES, PKA and RNG blocks. The software
// engines are the reference inside the test.
#define CRYPTO_TEST_ENGINE_HEADER	"crypto/crypto_stm32wba.h"

#define CRYPTO_TEST_AES_TYPE		CryptoWbaAes
#define CRYPTO_TEST_AES_MEMSIZE		CRYPTO_WBA_AES_MEMSIZE
#define CRYPTO_TEST_AES_CREATE(m, s)	CryptoWbaAesCreate((m), (s))

#define CRYPTO_TEST_PKA_TYPE		CryptoWbaPka
#define CRYPTO_TEST_PKA_MEMSIZE		CRYPTO_WBA_PKA_MEMSIZE
#define CRYPTO_TEST_PKA_CREATE(m, s, r)	CryptoWbaPkaCreate((m), (s), (r))

#define CRYPTO_TEST_RNG_TYPE		CryptoWbaRng
#define CRYPTO_TEST_RNG_MEMSIZE		CRYPTO_WBA_RNG_MEMSIZE
#define CRYPTO_TEST_RNG_CREATE(m, s)	CryptoWbaRngCreate((m), (s))

// Timing from the Cortex-M33 cycle counter (DWT), needs no library
#define CRYPTO_TEST_TICKS_INIT()	do { \
		*(volatile uint32_t *)0xE000EDFCU |= (1U << 24); \
		*(volatile uint32_t *)0xE0001000U |= 1U; } while (0)
#define CRYPTO_TEST_TICKS()			(*(volatile uint32_t *)0xE0001004U)
#define CRYPTO_TEST_TICKS_PER_US	(SystemCoreClockGet() / 1000000U)

#endif
