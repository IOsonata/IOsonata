/**-------------------------------------------------------------------------
@example	board.h

@brief	LPCXpresso54628 board definitions for I2CMasterSlave

The master is the on board I2C bus, Flexcomm 2, and the slave is Flexcomm 8
on the J9 expansion header. Wire the two buses together, see README.md.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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

// Output on the LPC-Link2 VCOM, Flexcomm 0, P0_29 RXD and P0_30 TXD
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			29
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			30
#define UART_TX_PINOP		IOPINOP_FUNC1

// Both Flexcomms have DMA channels, all eight mode combinations run
#define I2C_MASTER_HAS_DMA	1
#define I2C_SLAVE_HAS_DMA	1
#define I2C_MASTER_DEVNO	2
#define I2C_SLAVE_DEVNO		8
#define I2C_MASTER_INT_PRIO	7
#define I2C_SLAVE_INT_PRIO	6

// Master, Flexcomm 2 on board bus with board pull-ups, also on the PMod
// header J11, P3_23 SDA (J11 pin 10) and P3_24 SCL (J11 pin 9), function 1.
// The audio codec and the accelerometer are on this bus.
#define I2C_MASTER_SDA_PORT		3
#define I2C_MASTER_SDA_PIN		23
#define I2C_MASTER_SDA_PINOP	IOPINOP_FUNC1

#define I2C_MASTER_SCL_PORT		3
#define I2C_MASTER_SCL_PIN		24
#define I2C_MASTER_SCL_PINOP	IOPINOP_FUNC1

// Slave, Flexcomm 8, P1_17 SDA (J9 pin 4) and P1_18 SCL (J9 pin 2),
// function 2, the pins of the NXP SDK I2C board to board examples
#define I2C_SLAVE_SDA_PORT		1
#define I2C_SLAVE_SDA_PIN		17
#define I2C_SLAVE_SDA_PINOP		IOPINOP_FUNC2

#define I2C_SLAVE_SCL_PORT		1
#define I2C_SLAVE_SCL_PIN		18
#define I2C_SLAVE_SCL_PINOP		IOPINOP_FUNC2

#endif
