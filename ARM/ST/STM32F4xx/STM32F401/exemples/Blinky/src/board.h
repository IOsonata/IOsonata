#ifndef __BOARD_H__
#define __BOARD_H__

#define BOARD NUCLEO_F401RE
#include "stm32_boards.h"

#define LED_PINS_MAP BOARD_LED_PINS
#define BUTTON_PINS_MAP BOARD_BUT_PINS
#define BUT1_INT 13
#define BUT1_INT_PRIO 6
#define BUT1_SENSE BOARD_BUT1_SENSE

#endif
