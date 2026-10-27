/**-------------------------------------------------------------------------
@file board.h

@brief Thingy:91 board definitions for ADP5360 PMIC test.

@license
MIT License

Copyright (c) 2026 I-SYST inc.

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
