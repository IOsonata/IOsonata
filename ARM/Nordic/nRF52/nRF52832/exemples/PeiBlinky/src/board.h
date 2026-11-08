#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// TIMER2 is the default. Define PEI_BLINKY_USE_RTC in the Eclipse
// project to run the exact same hardware routes from RTC2 instead.
// RTC2 is application-owned in this bare-metal BLYST-NANO-DK example.
#if defined(PEI_BLINKY_USE_RTC)
#define PEI_BLINKY_TIMER_DEVNO	2
#define PEI_BLINKY_TIMER_FREQ	32768U
#else
#define PEI_BLINKY_TIMER_DEVNO	(TimerGetHighFreqDevNo() + 2)
#define PEI_BLINKY_TIMER_FREQ	1000000U
#endif

#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG

// Two active-low buttons on BLYST-NANO-DK: BUT1 starts, BUT2 stops.
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG
#define PEI_BLINKY_BUTTON_SENSE	IOPINSENSE_LOW_TRANSITION

#endif // __BOARD_H__
