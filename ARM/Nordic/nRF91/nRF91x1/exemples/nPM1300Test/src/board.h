/**-------------------------------------------------------------------------
@file board.h

@brief Thingy:91 X board definitions for nPM1300 test.

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


/* Destructive programming is disabled for the board-safe default test. */
#define NPM1300_TEST_PROGRAM_CHARGER	0
#define NPM1300_TEST_PROGRAM_RAILS	0

#endif
