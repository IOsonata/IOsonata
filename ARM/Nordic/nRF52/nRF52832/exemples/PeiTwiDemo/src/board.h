#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// SDA P0.14 and SCL P0.15 use IOPINRES_PULLUP (external 4.7K
// pull-ups to VDD are recommended for a normal 100-kHz I2C bus).
// Address 0x7E intentionally has no attached slave. This tests
// TWIM ERROR -> STOP hardware routing; it is not a data-path test.
#define PEI_TWI_EMPTY_ADDRESS	0x7EU
#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG

#endif // __BOARD_H__
