/**-------------------------------------------------------------------------
@file	board.h
@brief	External GPIO test connections for the ESP32 devkit.
@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

#define PULSE_TRAIN_PINS_MAP { \
	{0, 4, IOPINOP_GPIO}, \
	{0, 5, IOPINOP_GPIO}, \
	{0, 6, IOPINOP_GPIO}, \
	{0, 7, IOPINOP_GPIO}, \
}

#endif // __BOARD_H__
