/**-------------------------------------------------------------------------
@file board.h
@brief STM32WBA65I-DK1 onboard SSD1315 OLED pin mapping (UM3462).
@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#define BOARD STM32WBA65I_DK1
#include "stm32_boards.h"

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

#endif
