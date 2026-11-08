#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// nRF52832 TIMER3 at 16 MHz; RTC2 is DevNo 2 if desired.
#define PEI_SAADC_TIMER_DEVNO	6
#define PEI_SAADC_TIMER_FREQ	0
#define PEI_SAADC_PERIOD_MS	500U

// The ADC driver maps logical pin number 8 to the internal VDD input.
#define PEI_SAADC_INPUT	8
#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG

#endif // __BOARD_H__
