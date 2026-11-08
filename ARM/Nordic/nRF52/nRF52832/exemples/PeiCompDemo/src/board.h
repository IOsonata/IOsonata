#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// nRF52832 LPCOMP AIN0 is on P0.02, BLYST-NANO-DK BUT1.
// BUT2 (P0.13) triggers the comparator START task over hardware PPI.
#define PEI_COMP_AIN	0
#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG

#endif // __BOARD_H__
