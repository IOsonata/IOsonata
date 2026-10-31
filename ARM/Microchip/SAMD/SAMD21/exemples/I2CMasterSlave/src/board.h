/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro I2C master and slave loopback

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
One SERCOM is the I2C master, another one the slave, wired together on
the SAM D21 Xplained Pro. The results go to the EDBG COM port.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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

#include "coredev/iopincfg.h"

// EDBG virtual COM port on SERCOM3, function C: RX PA23 pad 1, TX PA22 pad 0.
// Configure RX before TX in the pin map.
#define UART_DEVNO			3
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			23
#define UART_RX_PINOP		IOPINOP_PERIPHC
#define UART_TX_PORT		IOPORTA
#define UART_TX_PIN			22
#define UART_TX_PINOP		IOPINOP_PERIPHC

// Wiring, pins with the I2C pad drive:
//	master SDA PA08 (EXT1 11) <-> slave SDA PB12 (EXT2 7)
//	master SCL PA09 (EXT1 12) <-> slave SCL PB13 (EXT2 8)
// The pull-ups of the pins are enabled, external 4.7 kohm pull-ups to VCC
// make the edges faster.

// Master on SERCOM2, function D: SDA PA08 pad 0, SCL PA09 pad 1
#define I2C_MASTER_DEVNO		2
#define I2C_MASTER_SDA_PORT		IOPORTA
#define I2C_MASTER_SDA_PIN		8
#define I2C_MASTER_SDA_PINOP	IOPINOP_PERIPHD
#define I2C_MASTER_SCL_PORT		IOPORTA
#define I2C_MASTER_SCL_PIN		9
#define I2C_MASTER_SCL_PINOP	IOPINOP_PERIPHD

// Slave on SERCOM4, function C: SDA PB12 pad 0, SCL PB13 pad 1
#define I2C_SLAVE_DEVNO			4
#define I2C_SLAVE_SDA_PORT		IOPORTB
#define I2C_SLAVE_SDA_PIN		12
#define I2C_SLAVE_SDA_PINOP		IOPINOP_PERIPHC
#define I2C_SLAVE_SCL_PORT		IOPORTB
#define I2C_SLAVE_SCL_PIN		13
#define I2C_SLAVE_SCL_PINOP		IOPINOP_PERIPHC

// The SAM D port runs the I2C master in polling, interrupt, DMA and DMA with
// interrupt mode, the slave on interrupts without DMA.
#define I2C_TEST_MASTER_MASK	0x0F
#define I2C_TEST_SLAVE_MASK		0x01

// Cortex-M0+ priorities 0 to 3, the slave above the master
#define I2C_MASTER_INT_PRIO		2
#define I2C_SLAVE_INT_PRIO		1

#endif // __BOARD_H__
