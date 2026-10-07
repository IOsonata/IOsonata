/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM4L8 Xplained Pro I2C master/slave loopback wiring.

Connect the two independent SAM4L TWI instances on EXT4:
  PB14 (TWIMS3 TWD, master SDA)  <-> PB00 (TWIMS1 TWD, slave SDA)
  PB15 (TWIMS3 TWCK, master SCL) <-> PB01 (TWIMS1 TWCK, slave SCL)

TWIMS0 on PA23/PA24 is deliberately not used here because those pins are also
wired to the on-board EDBG DGI TWI interface.

The master side enables pull-ups through the existing shared example pin
configuration. External I2C pull-ups can also be used.

----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"

#define MCUOSC { \
	{ OSC_TYPE_XTAL, 12000000, 20, 180 }, \
	{ OSC_TYPE_XTAL, 32768, 20, 125 }, true }

#define UART_DEVNO			1
#define UART_RX_PORT		IOPORTC
#define UART_RX_PIN			26
#define UART_RX_PINOP		IOPINOP_PERIPHA
#define UART_TX_PORT		IOPORTC
#define UART_TX_PIN			27
#define UART_TX_PINOP		IOPINOP_PERIPHA

// Master: TWIM3 / TWIMS3 on PB14, PB15 peripheral C.
#define I2C_MASTER_DEVNO		3
#define I2C_MASTER_SDA_PORT	IOPORTB
#define I2C_MASTER_SDA_PIN	14
#define I2C_MASTER_SDA_PINOP	IOPINOP_PERIPHC
#define I2C_MASTER_SCL_PORT	IOPORTB
#define I2C_MASTER_SCL_PIN	15
#define I2C_MASTER_SCL_PINOP	IOPINOP_PERIPHC
#define I2C_MASTER_DMA_ENABLE	false
#define I2C_MASTER_INT_ENABLE	true

// Slave: TWIS1 / TWIMS1 on PB00, PB01 peripheral A.
// Slave mode is interrupt driven by the SAM4L port.
#define I2C_SLAVE_DEVNO		1
#define I2C_SLAVE_SDA_PORT	IOPORTB
#define I2C_SLAVE_SDA_PIN	0
#define I2C_SLAVE_SDA_PINOP	IOPINOP_PERIPHA
#define I2C_SLAVE_SCL_PORT	IOPORTB
#define I2C_SLAVE_SCL_PIN	1
#define I2C_SLAVE_SCL_PINOP	IOPINOP_PERIPHA
#define I2C_SLAVE_DMA_ENABLE	false
#define I2C_SLAVE_INT_ENABLE	true

#endif // __BOARD_H__
