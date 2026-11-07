#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// First LED on BLYST-NANO-DK. All PEI routing is in the shared example.
// TIMER2 (HF timer number 2). Platform projects may choose another timer.
#define PEI_BLINKY_TIMER_DEVNO	(TimerGetHighFreqDevNo() + 2)

#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG

// Two active-low buttons on BLYST-NANO-DK: BUT1 starts, BUT2 stops.
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG
#define PEI_BLINKY_BUTTON_SENSE	IOPINSENSE_LOW_TRANSITION

#endif // __BOARD_H__
