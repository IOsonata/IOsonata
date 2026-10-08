#ifndef __BOARD_H__
#define __BOARD_H__

// Example wiring only. Check these pins and clocks against your SAM4LS board.
// SAM4LS hardware validation has not been performed.

#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"

// Use the library default internal clocks; define MCUOSC for your board.

#define UART_DEVNO			1
#define UART_RX_PORT		IOPORTC
#define UART_RX_PIN			26
#define UART_RX_PINOP		IOPINOP_PERIPHA
#define UART_TX_PORT		IOPORTC
#define UART_TX_PIN			27
#define UART_TX_PINOP		IOPINOP_PERIPHA


// SAM4LS C-package example TWI: TWIS0 on PA23/PA24 peripheral B.
#define I2C_SLAVE_DEVNO		0
#define I2C_SLAVE_SDA_PORT	IOPORTA
#define I2C_SLAVE_SDA_PIN	23
#define I2C_SLAVE_SDA_PINOP	IOPINOP_PERIPHB
#define I2C_SLAVE_SCL_PORT	IOPORTA
#define I2C_SLAVE_SCL_PIN	24
#define I2C_SLAVE_SCL_PINOP	IOPINOP_PERIPHB
#define I2C_SLAVE_DMA_ENABLE	false

#endif
