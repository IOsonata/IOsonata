/**-------------------------------------------------------------------------
@file	main.c  (Blinky — MPCCR diagnostic)

@brief	Bypass-msDelay diagnostic.  Reads MPCCR twice with a fixed NOP
	gap between, lights the WS2812 according to what it sees:

	  GREEN   MPCCR is incrementing (perf counter is enabled — the
	          Esp32EnablePerfCounter() call is running and the rdcycle
	          path in idelay.h should work).
	  RED     MPCCR returned 0 both times (counter not enabled — the
	          lib hasn't been rebuilt with the new system_esp32_system.c,
	          so old Esp32SystemInit without the enable is being linked).
	  BLUE    MPCCR returned the same non-zero value twice (very
	          unlikely — would mean the counter is paused or saturated).

	The bit-bang uses only NOPs, no msDelay, so it works regardless of
	whether the cycle counter is alive.

@author	Hoang Nguyen Hoan
@date	May 2026
@license MIT
----------------------------------------------------------------------------*/

#include <stdint.h>

#include "coredev/iopincfg.h"

#define GPIO_OUT_W1TS_REG       (*(volatile uint32_t *)(0x60004008UL))
#define GPIO_OUT_W1TC_REG       (*(volatile uint32_t *)(0x6000400CUL))
#define WS2812_PIN              8U
#define WS2812_MASK             (1UL << WS2812_PIN)

#define WS_NOP_T0H()  __asm volatile (".rept 28 \n nop \n .endr")
#define WS_NOP_T0L()  __asm volatile (".rept 56 \n nop \n .endr")
#define WS_NOP_T1H()  __asm volatile (".rept 56 \n nop \n .endr")
#define WS_NOP_T1L()  __asm volatile (".rept 24 \n nop \n .endr")

#define WS_SEND_BIT(cond)                       \
    do {                                        \
        if (cond) {                             \
            GPIO_OUT_W1TS_REG = WS2812_MASK;    \
            WS_NOP_T1H();                       \
            GPIO_OUT_W1TC_REG = WS2812_MASK;    \
            WS_NOP_T1L();                       \
        } else {                                \
            GPIO_OUT_W1TS_REG = WS2812_MASK;    \
            WS_NOP_T0H();                       \
            GPIO_OUT_W1TC_REG = WS2812_MASK;    \
            WS_NOP_T0L();                       \
        }                                       \
    } while (0)

static void ws2812_send_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t bits = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    for (int i = 23; i >= 0; i--)
    {
        WS_SEND_BIT(bits & (1UL << i));
    }
    GPIO_OUT_W1TC_REG = WS2812_MASK;
}

/* Hard NOP-only delay so we don't pull idelay.h into the diagnostic. */
static void nop_delay(uint32_t loops)
{
    volatile uint32_t i;
    for (i = 0; i < loops; i++) { /* spin */ }
}

int main(void)
{
    IOPinConfig(0, WS2812_PIN, 0,
                IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    GPIO_OUT_W1TC_REG = WS2812_MASK;
    nop_delay(8000);                /* > 50 µs latch reset */

    /*--- Probe MPCCR --------------------------------------------------*/
    uint32_t c1, c2;
    __asm volatile ("csrr %0, 0x7E2" : "=r"(c1));
    nop_delay(1000);                /* let the counter advance noticeably */
    __asm volatile ("csrr %0, 0x7E2" : "=r"(c2));

    while (1)
    {
        if (c1 == 0U && c2 == 0U)
        {
            ws2812_send_pixel(40, 0, 0);   /* RED   — counter dead       */
        }
        else if (c2 > c1)
        {
            ws2812_send_pixel(0, 40, 0);   /* GREEN — counter running    */
        }
        else
        {
            ws2812_send_pixel(0, 0, 40);   /* BLUE  — paused / saturated */
        }
        nop_delay(1000000);
    }
}
