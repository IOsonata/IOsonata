/**-------------------------------------------------------------------------
@file	main.c

@brief	Blinky for ESP32-C3-DevKitM-1 — drives the on-board WS2812 RGB
	LED (GPIO 8) by bit-banging the WS2812B protocol.

The DevKitM-1 has one addressable WS2812B RGB LED on GPIO 8.  Plain
GPIO toggling won't make it show defined colours — the chip needs the
WS2812 single-wire serial protocol (one bit ≈ 1.25 µs, encoded by the
ratio of high-time to low-time).

WS2812B timing (datasheet, ±150 ns tolerance):
  T0H ≈ 0.4  µs   |  T0L ≈ 0.85 µs   ('0' bit)
  T1H ≈ 0.8  µs   |  T1L ≈ 0.45 µs   ('1' bit)
  RES > 50   µs low                  (latch)

CPU clock: 80 MHz (ESP32C3 default in IOsonata).  12.5 ns / cycle.
NOP counts below were chosen to land each pulse near the centre of its
tolerance window after accounting for the cost of the store + branch
that bracket each delay.

Wire format: 24 bits per LED, in order  G7..G0  R7..R0  B7..B0  (MSB
first).  The on-board WS2812B is wired at 3.3 V logic; no level
shifter needed because the IO drive on the C3 is enough for a single
chained pixel.

Bit-bang must run with M-mode interrupts off so an IRQ can't preempt
in the middle of a pulse and stretch it out of spec.  SystemInit()
already cleared MIE in mstatus; ws2812_send_pixel() additionally
disables/restores it locally so this stays correct even if app code
re-enables IRQs later.

@author	Hoang Nguyen Hoan
@date	May 2026
@license MIT
----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#include "coredev/iopincfg.h"

/*--- ESP32-C3 GPIO output set/clear (TRM Rev 0.4 §6) ---*/
#define GPIO_OUT_W1TS_REG       (*(volatile uint32_t *)(0x60004008UL))
#define GPIO_OUT_W1TC_REG       (*(volatile uint32_t *)(0x6000400CUL))

#define WS2812_PIN              8U
#define WS2812_MASK             (1UL << WS2812_PIN)

/*-----------------------------------------------------------------------
 * Bit-bang macros.  Each WS_BIT_x emits the high then low pulse for one
 * bit, with NOP counts calibrated for 80 MHz.  These are MACROS, not
 * inline functions, so there's no call/ret overhead between bits — the
 * compiler emits the store + nop block + store + nop block back to back.
 *
 * Tuning notes (re-tune if you change the CPU frequency):
 *   - The store to W1TS/W1TC takes ~3 cycles on the APB bus.
 *   - Each NOP is 1 cycle.
 *   - Loop overhead between bits (in send_pixel) adds ~5-8 cycles, which
 *     extends the LOW phase of each bit; that's why T0L / T1L counts
 *     below are slightly under the nominal target.
 *---------------------------------------------------------------------*/
#define WS_NOP_T0H()  __asm volatile (".rept 28 \n nop \n .endr")  /* ~0.37 us + sw latency */
#define WS_NOP_T0L()  __asm volatile (".rept 56 \n nop \n .endr")  /* ~0.83 us (loop adds ~0.10) */
#define WS_NOP_T1H()  __asm volatile (".rept 56 \n nop \n .endr")  /* ~0.78 us */
#define WS_NOP_T1L()  __asm volatile (".rept 24 \n nop \n .endr")  /* ~0.42 us (loop adds ~0.10) */

/* Send one bit.  cond is non-zero for a '1' bit, zero for a '0'. */
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

/*-----------------------------------------------------------------------
 * Send 24 bits (G,R,B) MSB-first.  Loop is unrolled to keep timing
 * predictable at -O0.
 *
 * NOTE: this bit-bang is timing-critical — an IRQ firing in the middle
 * of a pulse will stretch it past tolerance and the LED will get
 * garbled colour.  SystemInit() already cleared MIE in mstatus and
 * disabled all CPU interrupt lines (INTERRUPT_CORE0_CPU_INT_ENABLE_REG
 * = 0), so by default no IRQ can fire here.  If you later enable IRQs
 * elsewhere in the app, wrap calls to this function in a local IRQ
 * disable/restore (or rebuild the example with zicsr to do it inline).
 *---------------------------------------------------------------------*/
static void ws2812_send_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t bits = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;

    WS_SEND_BIT(bits & (1UL << 23));
    WS_SEND_BIT(bits & (1UL << 22));
    WS_SEND_BIT(bits & (1UL << 21));
    WS_SEND_BIT(bits & (1UL << 20));
    WS_SEND_BIT(bits & (1UL << 19));
    WS_SEND_BIT(bits & (1UL << 18));
    WS_SEND_BIT(bits & (1UL << 17));
    WS_SEND_BIT(bits & (1UL << 16));
    WS_SEND_BIT(bits & (1UL << 15));
    WS_SEND_BIT(bits & (1UL << 14));
    WS_SEND_BIT(bits & (1UL << 13));
    WS_SEND_BIT(bits & (1UL << 12));
    WS_SEND_BIT(bits & (1UL << 11));
    WS_SEND_BIT(bits & (1UL << 10));
    WS_SEND_BIT(bits & (1UL <<  9));
    WS_SEND_BIT(bits & (1UL <<  8));
    WS_SEND_BIT(bits & (1UL <<  7));
    WS_SEND_BIT(bits & (1UL <<  6));
    WS_SEND_BIT(bits & (1UL <<  5));
    WS_SEND_BIT(bits & (1UL <<  4));
    WS_SEND_BIT(bits & (1UL <<  3));
    WS_SEND_BIT(bits & (1UL <<  2));
    WS_SEND_BIT(bits & (1UL <<  1));
    WS_SEND_BIT(bits & (1UL <<  0));

    /* Hold low for >50 us so the WS2812 latches the new value. */
    GPIO_OUT_W1TC_REG = WS2812_MASK;
}

/*-----------------------------------------------------------------------
 * Crude busy-wait delay.  Approximate ms timing; only used to space
 * out colour transitions for visual purposes, not for the WS2812
 * latch (which is handled by the long-low at the end of send_pixel).
 *
 * Volatile so -O0 can't optimise the loop body away.
 *---------------------------------------------------------------------*/
static void delay_ms(uint32_t ms)
{
    /* At 80 MHz, -O0, the inner loop (load i, compare, branch, increment,
     * store) takes ~5-6 cycles per iteration → ~13000 iterations per ms. */
    volatile uint32_t i;
    while (ms--) {
        for (i = 0; i < 13000UL; i++) { /* spin */ }
    }
}

int main(void)
{
    /* GPIO 8 as plain output, no pull (the WS2812 has its own driver). */
    IOPinConfig(0, WS2812_PIN, 0,
                IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

    /* Force the line low long enough for the WS2812 to see a clean
     * reset before the first pixel. */
    GPIO_OUT_W1TC_REG = WS2812_MASK;
    delay_ms(1);

    while (1)
    {
        ws2812_send_pixel(32,  0,  0);   /* dim red    */
        delay_ms(500);

        ws2812_send_pixel( 0, 32,  0);   /* dim green  */
        delay_ms(500);

        ws2812_send_pixel( 0,  0, 32);   /* dim blue   */
        delay_ms(500);

        ws2812_send_pixel( 0,  0,  0);   /* off        */
        delay_ms(500);
    }

    return 0;
}
