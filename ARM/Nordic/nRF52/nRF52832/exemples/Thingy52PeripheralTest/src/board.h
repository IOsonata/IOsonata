/** Thingy:52 production PCA20020 pin map. SPDX-License-Identifier: MIT */
#ifndef __BOARD_H__
#define __BOARD_H__
#include "thingy52_board.h"

// External UART adapter: P4.3 / ANA_DIG0 -> adapter RX; P4.4 -> adapter TX.
#define UART_DEVNO 0
#define UART_TX_PORT 0
#define UART_TX_PIN 2
#define UART_TX_PINOP 1
#define UART_RX_PORT 0
#define UART_RX_PIN 3
#define UART_RX_PINOP 1

#define I2C_DEVNO THINGY52_I2C_DEVNO
#define I2C_SDA_PORT THINGY52_I2C_PORT
#define I2C_SDA_PIN THINGY52_I2C_SDA_PIN
#define I2C_SDA_PINOP 1
#define I2C_SCL_PORT THINGY52_I2C_PORT
#define I2C_SCL_PIN THINGY52_I2C_SCL_PIN
#define I2C_SCL_PINOP 1
#endif
