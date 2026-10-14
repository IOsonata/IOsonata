/* Headers that the STM32CubeWBA Target and Common sources expect from the
 * application. The Cube application main.h also declares the peripheral
 * init functions of the generated project. None of them is used here. */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbaxx_hal.h"
#include "app_conf.h"
#include "stm32wbaxx_ll_pka.h"
#include "stm32wbaxx_ll_bus.h"
#include "stm32wbaxx_ll_cortex.h"
#include "stm32wbaxx_ll_rcc.h"
#include "stm32wbaxx_ll_system.h"
#include "stm32wbaxx_ll_utils.h"
#include "stm32wbaxx_ll_pwr.h"

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
