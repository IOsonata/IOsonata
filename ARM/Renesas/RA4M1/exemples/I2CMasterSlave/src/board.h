/**-------------------------------------------------------------------------
@file	board.h

@brief	RA4M1 shared I2C master/slave example board configuration.

		SCI0 on P410/P411 is the console. Renesas EK-RA4M1 reference pin
		layout: IIC1 master on P206/P100 and IIC0 slave on P401/P400.

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
#define I2C_DEBUG_DUMP(name, i2c) { \
	char dbg[640]; \
	Ra4m1I2cDump(i2c, dbg, sizeof(dbg)); \
	printf("%s %s\r\n", name, dbg); }

// Read each I2C pin as a plain GPIO input before the I2C drivers take the
// pins. The pin is first driven low to discharge it, then read as an input
// without pull-up (nopull=1 means something outside the MCU pulls it up),
// then driven low again and read with the internal pull-up. A pin that reads
// 0 with the pull-up is held low from outside the MCU.
#include "iopinctrl.h"
#define I2C_DEBUG_PIN_TEST(name, port, pin) { \
	IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL); \
	IOPinClear(port, pin); \
	for (volatile int d = 0; d < 200000; d++) {} \
	IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL); \
	for (volatile int d = 0; d < 200000; d++) {} \
	int nu = IOPinRead(port, pin); \
	IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL); \
	IOPinClear(port, pin); \
	for (volatile int d = 0; d < 200000; d++) {} \
	IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL); \
	for (volatile int d = 0; d < 200000; d++) {} \
	int up = IOPinRead(port, pin); \
	printf("pin %s P%d%02d pullup=%d nopull=%d\r\n", name, port, pin, up, nu); }

// Test only: repeat the master write with a changed setting and print the
// dumps. A trace that reaches state 07 means the master was not blocked.
#define I2C_DEBUG_TRY(name, stmt) { \
	stmt; \
	uint8_t tryOff = 4; \
	uint8_t tryData[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }; \
	int tryCnt = g_I2CMaster.Write(I2C_SLAVE_ADDR, &tryOff, 1, tryData, 8); \
	printf("try %s write=%d\r\n", name, tryCnt); \
	I2C_DEBUG_DUMP("M try", g_I2CMaster); \
	I2C_DEBUG_DUMP("S try", g_I2CSlave); }
// Give one master pin to the port with its pull-up on and read what the slave
// IIC block (held in reset, pins in IIC mode) sees on that line through its own
// ICCR1 input bit. A 1 means a port pull-up raises the line and the slave pin
// in IIC mode does not hold it low. A 0 means the slave pin holds it low.
#define I2C_DEBUG_LINE_TEST(name, port, pin, bit) { \
	IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL); \
	for (volatile int d = 0; d < 2000; d++) {} \
	int lineShort = (R_IIC1->ICCR1 >> bit) & 1; \
	for (volatile int d = 0; d < 2000000; d++) {} \
	int lineLong = (R_IIC1->ICCR1 >> bit) & 1; \
	printf("line %s seen by slave block: short=%d long=%d\r\n", name, lineShort, lineLong); }
// Same line, but the master pin is a port output set to open drain and left
// released (PODR=1, PDR=1, NCODR=1), with the pull-up bit on. This is the
// electrical state of an IIC pin that is not pulling the line low.
#define I2C_DEBUG_LINE_TEST_OD(name, port, pin, bit) { \
	IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN); \
	IOPinSet(port, pin); \
	for (volatile int d = 0; d < 2000; d++) {} \
	int odShort = (R_IIC1->ICCR1 >> bit) & 1; \
	for (volatile int d = 0; d < 2000000; d++) {} \
	int odLong = (R_IIC1->ICCR1 >> bit) & 1; \
	printf("line %s seen by slave block: short=%d long=%d\r\n", name, odShort, odLong); }
#define I2C_DEBUG_SLAVE_POKES() { \
	R_IIC1->ICCR1 = 0xC0U; \
	I2C_DEBUG_LINE_TEST("SCL via P408 port pull-up", I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN, 1) \
	I2C_DEBUG_LINE_TEST("SDA via P401 port pull-up", I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN, 0) \
	I2C_DEBUG_LINE_TEST_OD("SCL via P408 open-drain output, pull-up bit", I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN, 1) \
	I2C_DEBUG_LINE_TEST_OD("SDA via P401 open-drain output, pull-up bit", I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN, 0) }

// The IIC block does not use the internal pull-up of its own pins, and the
// board has no pull-up on them. Use two spare pins in GPIO input mode with
// their internal pull-up as the pull-up source, each wired with a jumper to
// the I2C line:
//   P204 (J1-5) to the SCL wire (P408 / P100)
//   P206 (J1-3) to the SDA wire (P401 / P101)
// Each is about a 50k pull-up, so the rise time is slow. Test only.
// Test only: drive each spare pin low, then high, and read what the master
// IIC block sees on its SCL (ICCR1 bit 1) or SDA (bit 0) input. The input
// follows the pin only if the jumper is on that wire. Then the pin is
// released with its pull-up on and read again after a short and a long wait.
#define I2C_DEBUG_JUMPER_LINE(name, pin, bit) { \
	IOPinConfig(2, pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL); \
	IOPinClear(2, pin); \
	for (volatile int d = 0; d < 20000; d++) {} \
	int jLo = (R_IIC0->ICCR1 >> bit) & 1; \
	IOPinSet(2, pin); \
	for (volatile int d = 0; d < 20000; d++) {} \
	int jHi = (R_IIC0->ICCR1 >> bit) & 1; \
	IOPinConfig(2, pin, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL); \
	for (volatile int d = 0; d < 2000; d++) {} \
	int jP1 = (R_IIC0->ICCR1 >> bit) & 1; \
	for (volatile int d = 0; d < 2000000; d++) {} \
	int jP2 = (R_IIC0->ICCR1 >> bit) & 1; \
	printf("jumper %s low=%d high=%d pullup_short=%d pullup_long=%d\r\n", name, jLo, jHi, jP1, jP2); }
#define I2C_DEBUG_JUMPER_TEST() { \
	I2C_DEBUG_JUMPER_LINE("P204 on SCL?", 4, 1) \
	I2C_DEBUG_JUMPER_LINE("P206 on SDA?", 6, 0) }

#define I2C_DEBUG_PULLUP_PINS() { \
	IOPinConfig(2, 4, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL); \
	IOPinConfig(2, 6, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL); }

// Show what the two spare pull-up pins look like after the I2C init: the PFS
// word (PCR is bit 4, PDR bit 2, PIDR bit 1) and the level read on the pin.
#define I2C_DEBUG_PULLUP_STATE() { \
	printf("pullup P204 pfs=%08x pin=%d  P206 pfs=%08x pin=%d\r\n", \
		(unsigned int)R_PFS->PORT[2].PIN[4].PmnPFS, IOPinRead(2, 4), \
		(unsigned int)R_PFS->PORT[2].PIN[6].PmnPFS, IOPinRead(2, 6)); }

#define I2C_DEBUG_PIN_TESTS() { \
	I2C_DEBUG_PIN_TEST("M SDA", I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN) \
	I2C_DEBUG_PIN_TEST("M SCL", I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN) \
	I2C_DEBUG_PIN_TEST("S SDA", I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN) \
	I2C_DEBUG_PIN_TEST("S SCL", I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN) \
	I2C_DEBUG_PIN_TEST("P205 ", 2, 5) \
	I2C_DEBUG_PIN_TEST("P206 ", 2, 6) \
	I2C_DEBUG_PIN_TEST("P204 ", 2, 4) \
	I2C_DEBUG_PIN_TEST("P400 ", 4, 0) \
	I2C_DEBUG_PIN_TEST("P407 ", 4, 7) }

#endif // __BOARD_H__
