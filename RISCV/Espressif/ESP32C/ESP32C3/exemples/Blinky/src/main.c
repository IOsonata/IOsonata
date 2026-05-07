/**-------------------------------------------------------------------------
@file	main.c

@brief	Blinky — IOsonata smoke test for the ESP32-C3

Toggles a GPIO at ~5 Hz using the IOsonata pin API
(IOPinConfig / IOPinSet / IOPinClear), independent of idelay.h.

Why not msDelay() from idelay.h:
  The current generic RISC-V idelay.h selects the rdcycle path when
  __riscv_zicsr is defined (which is true for -march=rv32imczicsr).
  The ESP32-C3 CPU exposes rdcycle / mcycle, but at reset the counter
  may be gated by mcountinhibit.CY, in which case rdcycle returns a
  stuck value and the busy-wait loop never satisfies its exit
  condition.  This Blinky therefore uses an in-source volatile spin
  loop so first-boot validation does not depend on the cycle counter
  being live.  The proper fix (SYSTIMER-based delay or explicit
  mcountinhibit clear) belongs in the framework, not here.

Pin choice:
  Drives GPIO 8 directly.  On the ESP32-C3-DevKitM-1 this is the
  on-board WS2812 data line — at a ~5 Hz toggle the LED flickers
  (random colour during the long high, dark during the long low),
  giving a clear visual confirmation that boot is healthy.  GPIO 8
  is also a strapping pin (must be high at reset for normal boot);
  driving it after boot is fine.

@author	Hoang Nguyen Hoan
@date	Mar. 2026

@license	MIT
----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#include "coredev/iopincfg.h"
#include "iopinctrl.h"

#define BLINKY_PORT     0
#define BLINKY_PIN      9

/* Approx ~100 ms at -O0 on an 80 MHz CPU.  Volatile so the compiler can't
 * optimise the loop away; exact period is unimportant — only that it's
 * long enough to be visible. */
static void BlinkyDelay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 800000UL; i++) { /* spin */ }
}

int main(void)
{
    IOPinConfig(BLINKY_PORT, BLINKY_PIN, 0,
                IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

    while (1)
    {
        IOPinSet(BLINKY_PORT, BLINKY_PIN);
        BlinkyDelay();
        IOPinClear(BLINKY_PORT, BLINKY_PIN);
        BlinkyDelay();
    }

    return 0;
}
