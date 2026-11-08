#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// SPIM2 SCK P0.14, MOSI P0.15, MISO P0.16 (unconnected).
// No external SPI slave or CS wiring needed for this TX-only test.
// BUT1 starts SPIM2; END toggles LED1 and STOPs SPIM;
// STOPPED toggles RGB blue, all through PPI.
#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG

#endif // __BOARD_H__
