#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"

// nRF52832 TIMER3 (DevNo 6): HF 16 MHz.
// Change to 2 for RTC2 (LF 32.768 kHz).
#ifndef PEI_CC_TEST_TIMER_DEVNO
#define PEI_CC_TEST_TIMER_DEVNO	6
#endif

#ifndef PEI_CC_TEST_TIMER_FREQ
#define PEI_CC_TEST_TIMER_FREQ	0
#endif

// 1: ISR on CC0, PEI on CC1. 0: PEI on CC0, ISR on CC1.
#ifndef PEI_CC_TEST_ISR_FIRST
#define PEI_CC_TEST_ISR_FIRST	0
#endif

#define PEI_CC_TEST_PEI_PERIOD_MS	500U
#define PEI_CC_TEST_ISR_PERIOD_MS	250U
#define LED_PINS_MAP	BLYST_NANO_DK_LED_PINS_CFG

#endif // __BOARD_H__
