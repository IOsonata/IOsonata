/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

// Board, one of the names in stm32_boards.h. This STM32WBA65 project builds for
// STM32WBA65I_DK1.
#define BOARD				STM32WBA65I_DK1

#include "stm32_boards.h"

// LD6 green PD8 and LD5 red PD9
#define LED_PINS_MAP		BOARD_LED_PINS

// Timer DevNo 0 (LPTIM1) and 1 (LPTIM2) are used by the Bluetooth port. This
// is the free one the example would use for its own timing, TIM16.
#define TIMER_DEVNO			2

// Log output on the first ST-LINK virtual COM port, USART1, RX PA8, TX PB12.
#define UART_DEVNO			BOARD_VCP_DEVNO
#define UART_PINS			BOARD_VCP_PINS

#define LED_PINS			BOARD_LED_PINS
#define CONNECT_LED_PORT	IOPORTD
#define CONNECT_LED_PIN		8
#define CONNECT_LED_LOGIC	BOARD_LED_ACTIVE
#define BLUEIO_CONNECT_LED_PORT	CONNECT_LED_PORT
#define BLUEIO_CONNECT_LED_PIN	CONNECT_LED_PIN

// USART1 on the first ST-LINK VCP. No hardware flow control.
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			8
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTB
#define UART_TX_PIN			12
#define UART_TX_PINOP		IOPINOP_FUNC7
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		IOPINOP_GPIO

// Optional external button: Arduino D4 / PC13 to GND, as in Blinky.
// The onboard joystick is analog; it is not a GPIO button.
#define BUT1_INT			13
#define BUTTON_PINS		{ \
	{IOPORTC, 13, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define BLE_STATUS_DISPLAY 1
#define DEVICE_NAME "BLEStatus"
#define OLED_SPI_DEVNO 2
#define OLED_SCK_PORT IOPORTA
#define OLED_SCK_PIN 0
#define OLED_SCK_PINOP IOPINOP_FUNC6
#define OLED_MOSI_PORT IOPORTB
#define OLED_MOSI_PIN 8
#define OLED_MOSI_PINOP IOPINOP_FUNC6
#define OLED_CS_PORT IOPORTE
#define OLED_CS_PIN 1
#define OLED_DC_PORT IOPORTE
#define OLED_DC_PIN 0
#define OLED_RST_PORT IOPORTE
#define OLED_RST_PIN 3

#define DISPLAY_DRIVER LcdSSD1315
#define DISPLAY_ORIENTATION DISPL_ORIENT_LANDSCAPE
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_STRIDE 16
#define DISPLAY_PIXEL_SIZE 1
// IOsonata names the active polarity: HIGH means CPOL=0, idle low.
#define DISPLAY_SPI_CLKPOL SPICLKPOL_HIGH
#define DISPLAY_SPI_DMA_ENABLE false

#define SPI_DEVNO OLED_SPI_DEVNO
#define SPI_PHY SPIPHY_NORMAL
#define SPI_RATE 4000000
#define SPI_PINS { \
	{OLED_SCK_PORT, OLED_SCK_PIN, OLED_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{OLED_MOSI_PORT, OLED_MOSI_PIN, OLED_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{OLED_CS_PORT, OLED_CS_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}
#define TFT_PINS { \
	{OLED_DC_PORT, OLED_DC_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{OLED_RST_PORT, OLED_RST_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}

#endif

