/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

ESP32-C3-DevKitM-1 hardware:
  - On-board WS2812B RGB LED on GPIO 8
  - USB <-> UART bridge (CP2102N) wired to UART0:
      TXD0 = GPIO 21 (C3 -> host)
      RXD0 = GPIO 20 (host -> C3)
    The same UART0 path the ROM bootloader uses for the boot banner.

PinOp on ESP32 follows the canonical IOsonata convention: a pin's
alternate function is selected by IOPINOP_FUNCn macros (or equivalent
chip-specific aliases).  For ESP32 the IOPinConfig() in
iopincfg_esp32.c interprets the value as a GPIO matrix signal index:

    IOPINOP_GPIO        : plain GPIO, controlled via GPIO_OUT/IN_REG
    IOPINOP_FUNC0 + n   : route matrix signal n to/from this pin

Direction (IOPinDir_INPUT vs IOPINDIR_OUTPUT) selects which side of the
matrix is programmed.  esp32xx_uart.h exposes ready-made aliases
(ESP32_PINOP_U0TXD / U0RXD / U1TXD / etc.) so board files don't have
to know the raw signal index.

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

#include "coredev/iopincfg.h"
#include "esp32xx_uart.h"

// *** ESP32-C3-DevKitM-1
#define UART_DEVNO          0

#define UART_RX_PORT        0
#define UART_RX_PIN         20
#define UART_RX_PINOP       ESP32_PINOP_U0RXD

#define UART_TX_PORT        0
#define UART_TX_PIN         21
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

#endif // __BOARD_H__
