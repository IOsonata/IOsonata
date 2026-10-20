/**-------------------------------------------------------------------------
@file	board.h

@brief	I2C master and slave on the same FPB-R9A02G021

IICA1 is the master, IICA0 the slave at address 0x22. Wiring:
	P010 (SDAA1)	to P103 (SDAA0_B)	SDA
	P011 (SCLA1)	to P102 (SCLA0_B)	SCL
and a pull-up from each line to 3.3 V, 4.7 kOhm for example. The IICA pins
are in output mode once the IICA is enabled (24.2.9), their internal pull-ups
are then off (16.2.5 PCR).
Connectors (FPB-R9A02G021 quick start guide): P010 and P011 on the Arduino
header, P102 and P103 on Arduino D12 and D11, Pmod 1 J13-3 and J13-2.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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

// The IICA needs PCLKB at 32 MHz or lower (notes of table 8.2), the HOCO is
// set to 32 MHz. SystemInit runs PCLKB at the core clock.
#define MCUOSC	{ {OSC_TYPE_RC, 32000000, 0, 0}, {OSC_TYPE_RC, 32768, 0, 0}, false }

// SAU UART0 on the on-board J-Link OB virtual COM port, R31 and R33 fitted
// and E56 closed (board defaults):
// TxD0_B P302, RxD0_B P303. PSEL 0x11 (SAU) is IOPINOP_FUNC16.
#define UART_DEVNO				0
#define UART_RX_PORT			3
#define UART_RX_PIN				3
#define UART_RX_PINOP			IOPINOP_FUNC16
#define UART_TX_PORT			3
#define UART_TX_PIN				2
#define UART_TX_PINOP			IOPINOP_FUNC16

// Master IICA1: SDAA1 P010, SCLA1 P011. PSEL 0x07 (IICA) is IOPINOP_FUNC6.
#define I2C_MASTER_DEVNO		1
#define I2C_MASTER_SDA_PORT		0
#define I2C_MASTER_SDA_PIN		10
#define I2C_MASTER_SDA_PINOP	IOPINOP_FUNC6
#define I2C_MASTER_SCL_PORT		0
#define I2C_MASTER_SCL_PIN		11
#define I2C_MASTER_SCL_PINOP	IOPINOP_FUNC6

// Polling. true for the data in the interrupt. No DMA, I2CInit fails with it.
#define I2C_MASTER_DMA_ENABLE	false
#define I2C_MASTER_INT_ENABLE	false

// Slave IICA0: SDAA0_B P103, SCLA0_B P102 (SCLA0_A and SDAA0_A, P302 and
// P303, are the console)
#define I2C_SLAVE_DEVNO			0
#define I2C_SLAVE_SDA_PORT		1
#define I2C_SLAVE_SDA_PIN		3
#define I2C_SLAVE_SDA_PINOP		IOPINOP_FUNC6
#define I2C_SLAVE_SCL_PORT		1
#define I2C_SLAVE_SCL_PIN		2
#define I2C_SLAVE_SCL_PINOP		IOPINOP_FUNC6

// The slave works in its interrupt, without DMA. It stretches SCL after each
// byte until its handler runs, its priority is above the master's: the
// master interrupt waits for the stop condition at the end of a transfer.
#define I2C_SLAVE_DMA_ENABLE	false
#define I2C_SLAVE_INT_ENABLE	true
#define I2C_SLAVE_INT_PRIO		7
#define I2C_MASTER_INT_PRIO		6

#endif
