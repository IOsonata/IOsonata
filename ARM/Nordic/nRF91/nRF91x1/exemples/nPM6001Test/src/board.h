/**-------------------------------------------------------------------------
@file board.h

@brief Thingy:91 X board definitions for nPM6001 test.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#define UART_DEVNO					0
#define UART_RX_PORT				0
#define UART_RX_PIN					0
#define UART_RX_PINOP				1
#define UART_TX_PORT				0
#define UART_TX_PIN					1
#define UART_TX_PINOP				1

#define I2C_DEVNO					2
#define I2C_SDA_PORT				0
#define I2C_SDA_PIN					9
#define I2C_SDA_PINOP				1
#define I2C_SCL_PORT				0
#define I2C_SCL_PIN					8
#define I2C_SCL_PINOP				1

#define NPM6001_READY_NPM1300_GPIO	4
#define NPM6001_ENABLE_NPM1300_LDSW	0

/* Leave voltage programming disabled for the board-safe default test. */
#define NPM6001_TEST_SET_VOLTAGE		0
#define NPM6001_TEST_VOLTAGE_MV		3300

#endif
