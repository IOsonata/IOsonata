#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// PWM0 channel 0: RGB red (P0.28), 1 kHz, 30 percent duty.
// BUT1 starts sequence 0, BUT2 stops PWM through PPI.
// Physical LED1 toggles when PWM reports SEQSTARTED[0].
// RGB blue toggles when PWM reports STOPPED.
#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG

#endif // __BOARD_H__
