#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// UART0 UARTE EasyDMA: RX P0.08, TX P0.07 at 1 Mbaud.
// BUT1 P0.02 starts an eight-byte TX through PPI.
// ENDTX toggles discrete LED1 P0.30 and STOPTX.
// TXSTOPPED toggles RGB blue P0.27.
#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG
#define BUTTON_PINS_MAP	BLYST_NANO_DK_BUT_PINS_CFG

#endif // __BOARD_H__
