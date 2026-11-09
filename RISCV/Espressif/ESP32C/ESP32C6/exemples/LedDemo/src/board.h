/**-------------------------------------------------------------------------
@file	board.h
@brief	Onboard SPI RGB LED connections for the ESP32 devkit.
@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "esp32xx_spi.h"

// The onboard addressable RGB LED data input is GPIO8.
#define LED_DEMO_SPI_ENABLE 1
#define LED_SPI_DEVNO 0
#define LED_SPI_RATE 3333333
#define LED_SPI_PINS { \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 8, ESP32_PINOP_SPI2_MOSI, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif // __BOARD_H__
