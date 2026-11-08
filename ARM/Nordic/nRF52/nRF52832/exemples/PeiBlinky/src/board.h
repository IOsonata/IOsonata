#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// nRF52832 Timer DevNo 1 is RTC1 (32768 Hz). Bare-metal only:
// S132/SoftDevice owns RTC1 when the Bluetooth stack is active.
#define PEI_BLINKY_TIMER_DEVNO	1
#define PEI_BLINKY_TIMER_FREQ	0

#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG

// Two active-low buttons on BLYST-NANO-DK: BUT1 starts, BUT2 stops.
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG
#define PEI_BLINKY_BUTTON_SENSE	IOPINSENSE_LOW_TRANSITION

#endif // __BOARD_H__
