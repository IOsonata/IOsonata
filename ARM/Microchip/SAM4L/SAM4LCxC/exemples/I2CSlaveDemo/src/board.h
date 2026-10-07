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


// SAM4L8 Xplained Pro EXT1 TWI: TWIS0 on PA23/PA24 peripheral B.
#define I2C_SLAVE_DEVNO		0
#define I2C_SLAVE_SDA_PORT	IOPORTA
#define I2C_SLAVE_SDA_PIN	23
#define I2C_SLAVE_SDA_PINOP	IOPINOP_PERIPHB
#define I2C_SLAVE_SCL_PORT	IOPORTA
#define I2C_SLAVE_SCL_PIN	24
#define I2C_SLAVE_SCL_PINOP	IOPINOP_PERIPHB
#define I2C_SLAVE_DMA_ENABLE	false

#endif
