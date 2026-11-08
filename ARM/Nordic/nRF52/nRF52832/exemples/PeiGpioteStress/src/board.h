#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// This standalone test temporarily uses the LEDs and UART pins as
// passive inputs. No UART peripheral is enabled.
#define STRESS_EXTRA_PIN	BLYST_NANO_DK_UART_CTS_PIN
#define LED_PINS_MAP		BLYST_NANO_DK_LED_PINS_CFG
#define BUTTON_PINS_MAP		BLYST_NANO_DK_BUT_PINS_CFG

#endif // __BOARD_H__
