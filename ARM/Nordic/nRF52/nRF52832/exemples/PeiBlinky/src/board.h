#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// First LED on BLYST-NANO-DK. All PEI routing is in the shared example.
// TIMER2 (HF timer number 2). Platform projects may choose another timer.
#define PEI_BLINKY_TIMER_DEVNO	(TimerGetHighFreqDevNo() + 2)

#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG

#endif // __BOARD_H__
