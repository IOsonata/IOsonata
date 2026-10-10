/* STM32WBA common Cortex-M33 system initialization.
 * ResetEntry owns .data/.bss and the C runtime.
 * Peripheral clocks and oscillator changes belong to the platform clock
 * driver; this implementation preserves the 16 MHz HSI reset clock.
 */
#include <stdint.h>
#include "stm32wbaxx.h"
#include "system_stm32wbaxx.h"

extern void (* const __Vectors[])(void);

uint32_t SystemCoreClock = 16000000U;
const uint8_t AHBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
const uint8_t APBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
const uint8_t AHB5PrescTable[8] = {1U, 1U, 1U, 1U, 2U, 3U, 4U, 6U};

void SystemInit(void)
{
    /* WBA65 reset runs from HSI16. Do not change clock mux, flash latency,
       TrustZone or peripheral ownership until the MCU clock port is present. */
    SCB->VTOR = (uint32_t)__Vectors;
    __DSB();
    __ISB();
    SystemCoreClock = 16000000U;
}

/* Valid for initial HSI16 bring-up only. Replace with live RCC decoding
 * before enabling PLL/HSE or changing AHB prescalers.
 */
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 16000000U;
}
