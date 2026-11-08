#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG

// Active-low buttons on BLYST-NANO-DK: BUT1 fires the software event.
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG
#define PEI_SW_EVENT_BUTTON_SENSE	IOPINSENSE_LOW_TRANSITION

#endif // __BOARD_H__
