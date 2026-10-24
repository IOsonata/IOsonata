/**-------------------------------------------------------------------------
@file board.h

@brief Thingy:91 board definitions for ADP5360 PMIC test.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#define UART_DEVNO				0
#define UART_RX_PORT			0
#define UART_RX_PIN				19
#define UART_RX_PINOP			1
#define UART_TX_PORT			0
#define UART_TX_PIN				18
#define UART_TX_PINOP			1

#define I2C_DEVNO				2
#define I2C_SDA_PORT			0
#define I2C_SDA_PIN				11
#define I2C_SDA_PINOP			1
#define I2C_SCL_PORT			0
#define I2C_SCL_PIN				12
#define I2C_SCL_PINOP			1

/*
 * The default test is read-only. Enable only when regulator programming
 * is intentionally being tested on hardware.
 */
#define ADP5360_TEST_PROGRAM_RAILS	0

#endif
