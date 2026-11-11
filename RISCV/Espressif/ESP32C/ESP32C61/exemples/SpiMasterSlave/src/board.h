/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

ESP32-C61-DevKitC-1 SpiMasterSlave.
UART0 TX: GPIO11; RX: GPIO10. Console: 1,000,000 baud, 8N1.
The test uses the generic application; board connections are defined below.

@author	Hoang Nguyen Hoan
@date	May 2026

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
#define UART_RETARGET

#include "coredev/iopincfg.h"
#include "esp32xx_uart.h"
#include "esp32xx_spi.h"

// UART console
#define UART_DEVNO          0

#define UART_RX_PORT        0
#define UART_RX_PIN         10
#define UART_RX_PINOP       ESP32_PINOP_U0RXD

#define UART_TX_PORT        0
#define UART_TX_PIN         11
#define UART_TX_PINOP       ESP32_PINOP_U0TXD

#define UART_CTS_PORT       -1
#define UART_CTS_PIN        -1
#define UART_CTS_PINOP      IOPINOP_GPIO

#define UART_RTS_PORT       -1
#define UART_RTS_PIN        -1
#define UART_RTS_PINOP      IOPINOP_GPIO

#define UART_PINS           { \
	{ UART_RX_PORT,  UART_RX_PIN,  UART_RX_PINOP,  IOPINDIR_INPUT,  IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_TX_PORT,  UART_TX_PIN,  UART_TX_PINOP,  IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT,  IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
}

#define UART_FLOWCTRL       UART_FLWCTRL_NONE


#define SPI_MASTER_SOFTWARE 1
#define SPI_MASTER_RATE 10000
#define SPI_TEST_SLAVE_MASK 1
#define SPI_TEST_FRAME_SIZE 64
#define SPI_MASTER_DEVNO 0
#define SPI_SLAVE_DEVNO 0
#define SPI_MASTER_CS_PORT 0
#define SPI_MASTER_CS_PIN 0
#define SPI_MASTER_CS_PINOP IOPINOP_GPIO
#define SPI_SLAVE_CS_PORT 0
#define SPI_SLAVE_CS_PIN 1
#define SPI_SLAVE_CS_PINOP ESP32_PINOP_SPI2_CS
#define SPI_MASTER_SCK_PORT 0
#define SPI_MASTER_SCK_PIN 2
#define SPI_MASTER_SCK_PINOP IOPINOP_GPIO
#define SPI_SLAVE_SCK_PORT 0
#define SPI_SLAVE_SCK_PIN 3
#define SPI_SLAVE_SCK_PINOP ESP32_PINOP_SPI2_SCK
#define SPI_MASTER_MOSI_PORT 0
#define SPI_MASTER_MOSI_PIN 4
#define SPI_MASTER_MOSI_PINOP IOPINOP_GPIO
#define SPI_SLAVE_MOSI_PORT 0
#define SPI_SLAVE_MOSI_PIN 5
#define SPI_SLAVE_MOSI_PINOP ESP32_PINOP_SPI2_MOSI
#define SPI_MASTER_MISO_PORT 0
#define SPI_MASTER_MISO_PIN 6
#define SPI_MASTER_MISO_PINOP IOPINOP_GPIO
#define SPI_SLAVE_MISO_PORT 0
#define SPI_SLAVE_MISO_PIN 7
#define SPI_SLAVE_MISO_PINOP ESP32_PINOP_SPI2_MISO
#define SPI_LOOPBACK_WIRING "Jumpers: GPIO0-CS-GPIO1, GPIO2-SCK-GPIO3, GPIO4-MOSI-GPIO5, GPIO6-MISO-GPIO7."

#endif // __BOARD_H__
