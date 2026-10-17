/**-------------------------------------------------------------------------
@file	stm32_boards.h

@brief	Pin maps of the ST evaluation boards used by the STM32 examples

		The board.h of an example selects the board, then includes this file:

			#define BOARD		B_L475E_IOT01A
			#include "stm32_boards.h"

		For the selected board it defines:

		BOARD_NAME			board name string
		BOARD_UART_DEVNO	UART for an external serial adapter, on the
		BOARD_UART_PINS		Arduino D0 (RX) and D1 (TX) pins where the board
							has them
		BOARD_VCP_DEVNO		UART connected to the ST-LINK virtual COM port,
		BOARD_VCP_PINS		when the board has one
		BOARD_LED_PINS		user LEDs, BOARD_LED_ACTIVE 1 when they light on a
							high level, 0 on a low level (LED_LOGIC_HIGH and
							LED_LOGIC_LOW)
		BOARD_BUT_PINS		user button and its BOARD_BUT1_SENSE, when the
							board has a button read as a GPIO
		BOARD_I2C_DEVNO		I2C instance and its SDA and SCL pins on the
		BOARD_I2C_PINS		Arduino header, where defined
		BOARD_SPI_DEVNO		SPI instance with BOARD_SPI_SCK, _MISO, _MOSI and
							_CS _PORT, _PIN and _PINOP, where defined

		DevNo is the position of the instance in the table of the STM32 UART
		driver, ARM/ST/src/uart_stm32.cpp, counting only the instances of the
		MCU: LPUART1, USART1, USART2, USART3, USART4 or UART4, USART5 or UART5,
		USART6.

		Each board accepts only the MCU of its project, given by the device
		define of the project (STM32L476xx...). The pin maps are for the
		examples, the library does not use them.

		Pins from the board user manuals and board description files, pin
		alternate functions from the device pin data.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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

#ifndef __STM32_BOARDS_H__
#define __STM32_BOARDS_H__

#include "coredev/iopincfg.h"

// Board names for BOARD
#define STM32F0308_DISCO		1	//!< STM32F0308-DISCO, STM32F030R8
#define STM32L476G_DISCO		2	//!< 32L476GDISCOVERY, STM32L476VG
#define B_L475E_IOT01A			3	//!< B-L475E-IOT01A and B-L475E-IOT01A2, STM32L475VG
#define NUCLEO_L432KC			4	//!< NUCLEO-L432KC, STM32L432KC
#define NUCLEO_G474RE			5	//!< NUCLEO-G474RE, STM32G474RE
#define NUCLEO_H753ZI			6	//!< NUCLEO-H753ZI, STM32H753ZI
#define STM32WBA65I_DK1			7	//!< STM32WBA65I-DK1, STM32WBA65RI

#ifndef BOARD
#error "stm32_boards.h: define BOARD before including this file"
#endif

// UART pins: RX then TX
#define STM32_BOARD_UART(Port, RxPin, RxOp, TxPin, TxOp)	{ \
	{Port, RxPin, RxOp, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{Port, TxPin, TxOp, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define STM32_BOARD_UART2(RxPort, RxPin, RxOp, TxPort, TxPin, TxOp)	{ \
	{RxPort, RxPin, RxOp, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{TxPort, TxPin, TxOp, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// I2C pins: SDA then SCL, open drain with the pull-up of the pin
#define STM32_BOARD_I2C(SdaPort, SdaPin, SdaOp, SclPort, SclPin, SclOp)	{ \
	{SdaPort, SdaPin, SdaOp, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
	{SclPort, SclPin, SclOp, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
}

#define STM32_BOARD_LED(Port, Pin)	{Port, Pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}

#if BOARD == STM32F0308_DISCO
// No ST-LINK virtual COM port on this board.
#if !defined(STM32F030x8)
#error "STM32F0308-DISCO: build the STM32F030x8 project"
#endif
#define BOARD_NAME			"STM32F0308-DISCO"
// USART1 (DevNo 0) on PA10 RX and PA9 TX, AF1
#define BOARD_UART_DEVNO	0
#define BOARD_UART_PINS		STM32_BOARD_UART(IOPORTA, 10, IOPINOP_FUNC1, 9, IOPINOP_FUNC1)
// LD3 green PC9, LD4 blue PC8
#define BOARD_LED_PINS		{ STM32_BOARD_LED(IOPORTC, 9), STM32_BOARD_LED(IOPORTC, 8), }
#define BOARD_LED_ACTIVE	1
// B1 user on PA0, active high
#define BOARD_BUT_PINS		{ {IOPORTA, 0, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, }
#define BOARD_BUT1_SENSE	IOPINSENSE_HIGH_TRANSITION

#elif BOARD == STM32L476G_DISCO
// No Arduino connector: the serial adapter UART is the virtual COM port UART.
#if !defined(STM32L476xx)
#error "32L476GDISCOVERY: build the STM32L476 project"
#endif
#define BOARD_NAME			"32L476GDISCOVERY"
// USART2 (DevNo 2) on PD6 RX and PD5 TX, AF7, ST-LINK virtual COM port
#define BOARD_VCP_DEVNO		2
#define BOARD_VCP_PINS		STM32_BOARD_UART(IOPORTD, 6, IOPINOP_FUNC7, 5, IOPINOP_FUNC7)
#define BOARD_UART_DEVNO	BOARD_VCP_DEVNO
#define BOARD_UART_PINS		BOARD_VCP_PINS
// LD4 red PB2, LD5 green PE8
#define BOARD_LED_PINS		{ STM32_BOARD_LED(IOPORTB, 2), STM32_BOARD_LED(IOPORTE, 8), }
#define BOARD_LED_ACTIVE	1
// Joystick center on PA0, active high
#define BOARD_BUT_PINS		{ {IOPORTA, 0, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, }
#define BOARD_BUT1_SENSE	IOPINSENSE_HIGH_TRANSITION

#elif BOARD == B_L475E_IOT01A
// The STM32L476 project runs on the STM32L475VG: same device, no LCD
// controller.
#if !defined(STM32L475xx) && !defined(STM32L476xx)
#error "B-L475E-IOT01A: build the STM32L476 project"
#endif
#define BOARD_NAME			"B-L475E-IOT01A"
// UART4 (DevNo 4) on Arduino D0 PA1 RX and D1 PA0 TX, AF8
#define BOARD_UART_DEVNO	4
#define BOARD_UART_PINS		STM32_BOARD_UART(IOPORTA, 1, IOPINOP_FUNC8, 0, IOPINOP_FUNC8)
// USART1 (DevNo 1) on PB7 RX and PB6 TX, AF7, ST-LINK virtual COM port
#define BOARD_VCP_DEVNO		1
#define BOARD_VCP_PINS		STM32_BOARD_UART(IOPORTB, 7, IOPINOP_FUNC7, 6, IOPINOP_FUNC7)
// LD1 green PA5 (also Arduino D13, SPI1 SCK), LD2 green PB14
#define BOARD_LED_PINS		{ STM32_BOARD_LED(IOPORTA, 5), STM32_BOARD_LED(IOPORTB, 14), }
#define BOARD_LED_ACTIVE	1
// B1 user on PC13, active low
#define BOARD_BUT_PINS		{ {IOPORTC, 13, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, }
#define BOARD_BUT1_SENSE	IOPINSENSE_LOW_TRANSITION

#elif BOARD == NUCLEO_L432KC
// Arduino Nano connector. No user button.
#if !defined(STM32L432xx)
#error "NUCLEO-L432KC: build an STM32L432 project"
#endif
#define BOARD_NAME			"NUCLEO-L432KC"
// USART1 (DevNo 1) on D0 PA10 RX and D1 PA9 TX, AF7
#define BOARD_UART_DEVNO	1
#define BOARD_UART_PINS		STM32_BOARD_UART(IOPORTA, 10, IOPINOP_FUNC7, 9, IOPINOP_FUNC7)
// USART2 (DevNo 2) on PA15 RX AF3 and PA2 TX AF7, ST-LINK virtual COM port
#define BOARD_VCP_DEVNO		2
#define BOARD_VCP_PINS		STM32_BOARD_UART(IOPORTA, 15, IOPINOP_FUNC3, 2, IOPINOP_FUNC7)
// LD3 green PB3
#define BOARD_LED_PINS		{ STM32_BOARD_LED(IOPORTB, 3), }
#define BOARD_LED_ACTIVE	1

#elif BOARD == NUCLEO_G474RE
// STM32G474 library and examples in ARM/ST/STM32G4xx. UART DevNo: LPUART1 0,
// USART1 1, USART2 2, USART3 3, UART4 4, UART5 5.
#if !defined(STM32G474xx)
#error "NUCLEO-G474RE: build an STM32G474 project"
#endif
#define BOARD_NAME			"NUCLEO-G474RE"
// USART1 (DevNo 1) on Arduino D0 PC5 RX and D1 PC4 TX, AF7
#define BOARD_UART_DEVNO	1
#define BOARD_UART_PINS		STM32_BOARD_UART(IOPORTC, 5, IOPINOP_FUNC7, 4, IOPINOP_FUNC7)
// LPUART1 (DevNo 0) on PA3 RX and PA2 TX, AF12, ST-LINK virtual COM port
#define BOARD_VCP_DEVNO		0
#define BOARD_VCP_PINS		STM32_BOARD_UART(IOPORTA, 3, IOPINOP_FUNC12, 2, IOPINOP_FUNC12)
// LD2 green PA5 (also Arduino D13)
#define BOARD_LED_PINS		{ STM32_BOARD_LED(IOPORTA, 5), }
#define BOARD_LED_ACTIVE	1
// B1 user on PC13, active low
#define BOARD_BUT_PINS		{ {IOPORTC, 13, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, }
#define BOARD_BUT1_SENSE	IOPINSENSE_LOW_TRANSITION

#elif BOARD == NUCLEO_H753ZI
// No IOsonata STM32H7 port yet. DevNo as the UART driver would number the
// H753 instances: LPUART1 0, USART1 1, USART2 2, USART3 3, UART4 4, UART5 5,
// USART6 6.
#if !defined(STM32H753xx)
#error "NUCLEO-H753ZI: build an STM32H753 project"
#endif
#define BOARD_NAME			"NUCLEO-H753ZI"
// USART1 (DevNo 1) on Arduino D0 PB7 RX and D1 PB6 TX, AF7
#define BOARD_UART_DEVNO	1
#define BOARD_UART_PINS		STM32_BOARD_UART(IOPORTB, 7, IOPINOP_FUNC7, 6, IOPINOP_FUNC7)
// USART3 (DevNo 3) on PD9 RX and PD8 TX, AF7, ST-LINK virtual COM port
#define BOARD_VCP_DEVNO		3
#define BOARD_VCP_PINS		STM32_BOARD_UART(IOPORTD, 9, IOPINOP_FUNC7, 8, IOPINOP_FUNC7)
// LD1 green PB0, LD2 yellow PE1, LD3 red PB14
#define BOARD_LED_PINS		{ STM32_BOARD_LED(IOPORTB, 0), STM32_BOARD_LED(IOPORTE, 1), STM32_BOARD_LED(IOPORTB, 14), }
#define BOARD_LED_ACTIVE	1
// B1 user on PC13, active high
#define BOARD_BUT_PINS		{ {IOPORTC, 13, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, }
#define BOARD_BUT1_SENSE	IOPINSENSE_HIGH_TRANSITION

#elif BOARD == STM32WBA65I_DK1
// The joystick is read on the ADC (PA3), no GPIO button. LD3 blue is left
// out: the user manual and the board files disagree on its pin.
#if !defined(STM32WBA65xx)
#error "STM32WBA65I-DK1: build the STM32WBA65 project"
#endif
#define BOARD_NAME			"STM32WBA65I-DK1"
// USART2 (DevNo 2) on Arduino D0 PA11 RX and D1 PA12 TX, AF3. The user manual
// also names these pins as a second ST-LINK virtual COM port.
#define BOARD_UART_DEVNO	2
#define BOARD_UART_PINS		STM32_BOARD_UART(IOPORTA, 11, IOPINOP_FUNC3, 12, IOPINOP_FUNC3)
// USART1 (DevNo 1) on PA8 RX and PB12 TX, AF7, ST-LINK virtual COM port
#define BOARD_VCP_DEVNO		1
#define BOARD_VCP_PINS		STM32_BOARD_UART2(IOPORTA, 8, IOPINOP_FUNC7, IOPORTB, 12, IOPINOP_FUNC7)
// I2C1 (DevNo 0) on the Arduino D14 SDA PB1 and D15 SCL PB2, AF4
#define BOARD_I2C_DEVNO		0
#define BOARD_I2C_PINS		STM32_BOARD_I2C(IOPORTB, 1, IOPINOP_FUNC4, IOPORTB, 2, IOPINOP_FUNC4)
// SPI2 (DevNo 1), AF5 from the ST pin data, on the Arduino pins of UM3462
// table 8: SCK PB10 (D13), MISO PA9 (D12), MOSI PC3 (D11). The chip select is a
// GPIO driven by the driver, PB9 (D10).
#define BOARD_SPI_DEVNO		1
#define BOARD_SPI_SCK_PORT	IOPORTB
#define BOARD_SPI_SCK_PIN	10
#define BOARD_SPI_SCK_PINOP	IOPINOP_FUNC5
#define BOARD_SPI_MISO_PORT	IOPORTA
#define BOARD_SPI_MISO_PIN	9
#define BOARD_SPI_MISO_PINOP	IOPINOP_FUNC5
#define BOARD_SPI_MOSI_PORT	IOPORTC
#define BOARD_SPI_MOSI_PIN	3
#define BOARD_SPI_MOSI_PINOP	IOPINOP_FUNC5
#define BOARD_SPI_CS_PORT	IOPORTB
#define BOARD_SPI_CS_PIN	9
#define BOARD_SPI_CS_PINOP	IOPINOP_GPIO
// LD6 green PD8, LD5 red PD9
#define BOARD_LED_PINS		{ STM32_BOARD_LED(IOPORTD, 8), STM32_BOARD_LED(IOPORTD, 9), }
#define BOARD_LED_ACTIVE	0

#else
#error "stm32_boards.h: BOARD is not one of the boards listed"
#endif

#endif
