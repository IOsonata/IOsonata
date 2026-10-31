/* STM32WBA65I-DK1 adaptation of the CubeWBA throughput client. */
#ifndef ST_THROUGHPUT_DK1_CONFIG_H
#define ST_THROUGHPUT_DK1_CONFIG_H

#include "app_conf.h"

/* The DK1 shares USART1 PB12/PA8 with the Nucleo sample, but not its
 * buttons or LEDs. Keep ST's serial interpreter and use B1/B2/B3 there.
 * Force this header before SDK headers so quoted app_conf.h includes
 * cannot restore the Nucleo pin configuration. */
#undef CFG_BSP_ON_NUCLEO
#undef CFG_LED_SUPPORTED
#define CFG_LED_SUPPORTED 0
#undef CFG_BUTTON_SUPPORTED
#define CFG_BUTTON_SUPPORTED 0

#endif
