/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

ESP32C6 I2cMasterSlave console: UART0 TX on GPIO16, RX on GPIO17.
Use a 3.3 V UART adapter with common ground.

PinOp on ESP32 follows the canonical IOsonata convention: a pin's
alternate function is selected by IOPINOP_FUNCn macros (or equivalent
chip-specific aliases).  For ESP32 the IOPinConfig() in
iopincfg_esp32.c interprets the value as a GPIO matrix signal index:

    IOPINOP_GPIO        : plain GPIO, controlled via GPIO_OUT/IN_REG
    IOPINOP_FUNC0 + n   : route matrix signal n to/from this pin

Direction (IOPinDir_INPUT vs IOPINDIR_OUTPUT) selects which side of the
matrix is programmed.  esp32xx_uart.h exposes ready-made aliases
(ESP32_PINOP_U0TXD / U0RXD / U1TXD / etc.) so board files don't have
to know the raw signal index.

@author	Hoang Nguyen Hoan
@date	May 2026

@license

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#define UART_RETARGET

#include "coredev/iopincfg.h"
#include "esp32xx_uart.h"

// *** ESP32C6 UART console
#define UART_DEVNO          0

#define UART_RX_PORT        0
#define UART_RX_PIN         17
#define UART_RX_PINOP       ESP32_PINOP_U0RXD

#define UART_TX_PORT        0
#define UART_TX_PIN         16
#define UART_TX_PINOP       ESP32_PINOP_U0TXD

#define UART_CTS_PORT       -1
#define UART_CTS_PIN        -1
#define UART_CTS_PINOP      IOPINOP_GPIO

#define UART_RTS_PORT       -1
#define UART_RTS_PIN        -1
#define UART_RTS_PINOP      IOPINOP_GPIO

#define UART_PINS           { \
	{ UART_RX_PORT,  UART_RX_PIN,  UART_RX_PINOP,  IOPINDIR_INPUT,  IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_TX_PORT,  UART_TX_PIN,  UART_TX_PINOP,  IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT,  IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
}

#define UART_FLOWCTRL       UART_FLWCTRL_NONE


#define UART_BAUDRATE 1000000
#define UART_INT_MODE true
#define UART_DMA_MODE false

#include "esp32xx_i2c.h"
// Define I2C_TEST_BOARD_TRACE in the example compiler settings to capture
// register snapshots and use polling UART for diagnostic output.
#ifdef I2C_TEST_BOARD_TRACE
#define I2C_TEST_TRACE_SIZE 24
#define I2C_TEST_TRACE_DELAY_US 2000
#define I2C_TEST_UART_INT_ENABLE false
#endif
#define I2C_SLAVE_DEVNO 0
#define I2C_SLAVE_DMA_ENABLE false
#define I2C_SLAVE_SCL_DIR IOPINDIR_BI
#define I2C_SLAVE_SDA_PORT 0
#define I2C_SLAVE_SDA_PIN 4
#define I2C_SLAVE_SDA_PINOP ESP32_PINOP_I2C0_SDA
#define I2C_SLAVE_SCL_PORT 0
#define I2C_SLAVE_SCL_PIN 5
#define I2C_SLAVE_SCL_PINOP ESP32_PINOP_I2C0_SCL

#define I2C_MASTER_DEVNO 1
#define I2C_MASTER_DMA_ENABLE false
#define I2C_MASTER_INT_ENABLE false
#define I2C_MASTER_SCL_DIR IOPINDIR_BI
#define I2C_MASTER_SDA_PORT 0
#define I2C_MASTER_SDA_PIN 6
#define I2C_MASTER_SDA_PINOP ESP32_PINOP_LP_I2C_SDA
#define I2C_MASTER_SCL_PORT 0
#define I2C_MASTER_SCL_PIN 7
#define I2C_MASTER_SCL_PINOP ESP32_PINOP_LP_I2C_SCL
#define I2C_LOOPBACK_WIRING "Jumpers: GPIO6 (LP SDA) to GPIO4 (HP SDA), GPIO7 (LP SCL) to GPIO5 (HP SCL)."

#endif // __BOARD_H__
