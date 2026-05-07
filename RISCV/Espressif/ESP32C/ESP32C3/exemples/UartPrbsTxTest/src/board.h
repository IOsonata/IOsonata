/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

ESP32-C3-DevKitM-1 hardware:
  - On-board WS2812B RGB LED on GPIO 8
  - USB ↔ UART bridge (CP2102N) wired to UART0:
      TXD0 = GPIO 21 (C3 → host)
      RXD0 = GPIO 20 (host → C3)
    The same UART0 path the ROM bootloader uses for the boot banner.

PINOP on the C3 is the IOMUX function selector.  Setting it to 1 puts
the pad in MCU_SEL = GPIO mode and lets the GPIO matrix route the
UART signal to/from the pad.  This is what `iopincfg_esp32.c` programs
when IOPinConfig() is called with PinOp = 1.

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

// *** ESP32-C3-DevKitM-1
#define UART_DEVNO          0

#define UART_RX_PORT        0
#define UART_RX_PIN         20
#define UART_RX_PINOP       1

#define UART_TX_PORT        0
#define UART_TX_PIN         21
#define UART_TX_PINOP       1

#define UART_CTS_PORT       -1
#define UART_CTS_PIN        -1
#define UART_CTS_PINOP      0

#define UART_RTS_PORT       -1
#define UART_RTS_PIN        -1
#define UART_RTS_PINOP      0

#define UART_PINS           { \
    { UART_RX_PORT,  UART_RX_PIN,  UART_RX_PINOP,  IOPINDIR_INPUT,  IOPINRES_NONE, IOPINTYPE_NORMAL }, \
    { UART_TX_PORT,  UART_TX_PIN,  UART_TX_PINOP,  IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
    { UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT,  IOPINRES_NONE, IOPINTYPE_NORMAL }, \
    { UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
}

#define UART_FLOWCTRL       UART_FLWCTRL_NONE

#endif // __BOARD_H__
