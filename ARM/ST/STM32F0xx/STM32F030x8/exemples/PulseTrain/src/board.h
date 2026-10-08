// STM32F0308-DISCO application wiring. Change here for another board.
#ifndef __BOARD_H__
#define __BOARD_H__
#include "stm32f0xx.h"

// Full STM32F030R8 LQFP64 GPIO test: PA0-15, PB0-15, PC0-15, PD2,
// PF0/1/4/5/6/7 (55 pins). NRST, BOOT0 and supply pins are not GPIO.
// This also drives SWD PA13/PA14 and oscillator pins: run standalone with
// conflicting ST-LINK/oscillator connections isolated and an internal clock.
#define PULSE_TRAIN_PINS_MAP { \
	{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, \
	{0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 7, 0}, \
	{0, 8, 0}, {0, 9, 0}, {0, 10, 0}, {0, 11, 0}, \
	{0, 12, 0}, {0, 13, 0}, {0, 14, 0}, {0, 15, 0}, \
	{1, 0, 0}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, \
	{1, 4, 0}, {1, 5, 0}, {1, 6, 0}, {1, 7, 0}, \
	{1, 8, 0}, {1, 9, 0}, {1, 10, 0}, {1, 11, 0}, \
	{1, 12, 0}, {1, 13, 0}, {1, 14, 0}, {1, 15, 0}, \
	{2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, \
	{2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0}, \
	{2, 8, 0}, {2, 9, 0}, {2, 10, 0}, {2, 11, 0}, \
	{2, 12, 0}, {2, 13, 0}, {2, 14, 0}, {2, 15, 0}, \
	{3, 2, 0}, {5, 0, 0}, {5, 1, 0}, {5, 4, 0}, \
	{5, 5, 0}, {5, 6, 0}, {5, 7, 0}, \
}

#endif
