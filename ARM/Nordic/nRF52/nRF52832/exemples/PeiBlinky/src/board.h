#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// nRF52832 DevNo 6 is high-frequency TIMER3. Freq = 0 selects 16 MHz.
#define PEI_BLINKY_TIMER_DEVNO	6
#define PEI_BLINKY_TIMER_FREQ	0

#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG

// Two active-low buttons on BLYST-NANO-DK: BUT1 starts, BUT2 stops.
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG
#define PEI_BLINKY_BUTTON_SENSE	IOPINSENSE_LOW_TRANSITION

#endif // __BOARD_H__
