/**-------------------------------------------------------------------------
@file	board.h

@brief	RA4M1 shared I2C master/slave example board configuration.

		SCI0 on P410/P411 is the console. Renesas EK-RA4M1 reference pin
		layout: IIC1 master on P206/P100 and IIC0 slave on P401/P408.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#define UART_DEVNO				0
#define UART_RX_PORT			4
#define UART_RX_PIN				10
#define UART_RX_PINOP			IOPINOP_FUNC3
#define UART_TX_PORT			4
#define UART_TX_PIN				11
#define UART_TX_PINOP			IOPINOP_FUNC3

#define UART_PINS				{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define I2C_MASTER_DEVNO		1
#define I2C_MASTER_SDA_PORT		2
#define I2C_MASTER_SDA_PIN		6
#define I2C_MASTER_SDA_PINOP	IOPINOP_FUNC6
#define I2C_MASTER_SCL_PORT		1
#define I2C_MASTER_SCL_PIN		0
#define I2C_MASTER_SCL_PINOP	IOPINOP_FUNC6

#define I2C_MASTER_DMA_ENABLE	false
#define I2C_MASTER_INT_ENABLE	false

// Shared master/slave example: IIC0 uses interrupt callbacks.
#define I2C_SLAVE_DEVNO		0
#define I2C_SLAVE_SDA_PORT		4
#define I2C_SLAVE_SDA_PIN		1
#define I2C_SLAVE_SDA_PINOP	IOPINOP_FUNC6
#define I2C_SLAVE_SCL_PORT		4
#define I2C_SLAVE_SCL_PIN		0
#define I2C_SLAVE_SCL_PINOP	IOPINOP_FUNC6

#define I2C_SLAVE_DMA_ENABLE	false
#define I2C_SLAVE_INT_ENABLE	true

// Print the IIC register state after each step (RA4M1 diagnostic helper)
#include "i2c_ra4m1.h"
#include "ra4m1xxx.h"
#define I2C_DEBUG_DUMP(name, i2c) { \
	char dbg[640]; \
	Ra4m1I2cDump(i2c, dbg, sizeof(dbg)); \
	printf("%s %s\r\n", name, dbg); }

// The existing I2C_DEBUG_DUMP outputs the peripheral-mode ICCR1 SCLI/SDAI
// and PFS values immediately after Init, enabling a direct comparison.

// Read-only SCL sampling after both IIC units are initialized.
// Capture changes that a single ICCR1 dump could miss.
#define I2C_DEBUG_SCL_SAMPLES() \
{ \
	int masterHigh = 0; \
	int slaveHigh = 0; \
	int bothHigh = 0; \
	for (int i = 0; i < 256; i++) \
	{ \
		bool mh = (R_IIC1->ICCR1 & 0x02U) != 0U; \
		bool sh = (R_IIC0->ICCR1 & 0x02U) != 0U; \
		masterHigh += mh ? 1 : 0; \
		slaveHigh += sh ? 1 : 0; \
		bothHigh += (mh && sh) ? 1 : 0; \
		for (int delay = 0; delay < 256; delay++) \
		{ \
			__NOP(); \
		} \
	} \
	printf("SCL idle samples master_high=%d/256 slave_high=%d/256 both_high=%d/256\r\n", \
		masterHigh, slaveHigh, bothHigh); \
}

// Isolation test: disable only IIC0's peripheral drive, sample the line,
// then restore the saved controller state before the example transfers.
// There is no active transaction at this point.
#define I2C_DEBUG_SLAVE_ICE_TEST() \
{ \
	const uint8_t cr1 = R_IIC0->ICCR1; \
	const uint8_t ier = R_IIC0->ICIER; \
	R_IIC0->ICIER = 0U; \
	R_IIC0->ICCR1 = (uint8_t)(cr1 & (uint8_t)~0x80U); \
	__DSB(); \
	for (int i = 0; i < 256; i++) \
	{ \
		__NOP(); \
	} \
	const uint8_t masterOff = R_IIC1->ICCR1; \
	const uint8_t slaveOff = R_IIC0->ICCR1; \
	R_IIC0->ICCR1 = cr1; \
	R_IIC0->ICIER = ier; \
	__DSB(); \
	for (int i = 0; i < 256; i++) \
	{ \
		__NOP(); \
	} \
	printf("IIC0 ICE isolation M_off=%02x S_off=%02x M_on=%02x S_on=%02x\r\n", \
		masterOff, slaveOff, R_IIC1->ICCR1, R_IIC0->ICCR1); \
}

#endif // __BOARD_H__
