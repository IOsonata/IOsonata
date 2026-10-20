/**-------------------------------------------------------------------------
@file	board.h

@brief	STM32WBA65I-DK1 onboard M95P32 SPI EEPROM configuration (UM3462).

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#ifndef __BOARD_H__
#define __BOARD_H__

#define BOARD STM32WBA65I_DK1
#include "stm32_boards.h"

#define NVM_MODE 1
#define EEPROM_SPI 1
//#ifndef EEPROM_DEMO_UART
//#define EEPROM_DEMO_UART 1
//#endif
#define EEPROM_UART_DEVNO BOARD_VCP_DEVNO
#define EEPROM_UART_PINS BOARD_VCP_PINS

#define EEPROM_SPI_DEVNO 2
// SCK PA0, MISO PA1 and MOSI PB8 are SPI3 AF6. Memory CS is PH3.
// A second, unused CS keeps the OLED PE1 deselected on this shared bus.
#define EEPROM_SPI_PINS { \
	{IOPORTA, 0, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 1, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTB, 8, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTH, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTE, 1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}

#define EEPROM_DEV_ADDR 0
#define EEPROM_ADDR_SIZE 3
#define EEPROM_PAGE_SIZE 512
#define EEPROM_SIZE (4UL * 1024UL * 1024UL)
#define EEPROM_WRITE_DELAY_MS 0 // SPI Nvm polls WIP instead of using a delay
// ReadId stores 20 00 16 in the low bytes of a little-endian uint32_t.
#define EEPROM_DEVICE_ID 0x00160020UL
#define EEPROM_DEVICE_ID_SIZE 3

#endif
