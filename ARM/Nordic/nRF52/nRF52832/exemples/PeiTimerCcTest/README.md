# PeiTimerCcTest (nRF52832)

This is a standalone Eclipse Embedded CDT managed project. Import the ioc
directory in IOcomposer/Eclipse. It leaves the existing PeiBlinky unchanged.

Two independent compare channels share one free-running counter:
- PEI (500 ms): event -> PPI -> GPIOTE toggle task on LED0.
- Ordinary Timer trigger (250 ms): interrupt callback toggles LED1.

The Timer ISR advances each CC, but does not forward the PEI event.
There is no printf or semihosting output. As in PeiBlinky, the Eclipse
project links the standalone pei_nrf52.cpp PPI backend directly; do not
use SoftDevice-owned PPI resources in this bare-metal test.

## Four configurations

Set macros in src/board.h, build, flash, and observe both LEDs.

| Timer | PEI first | ISR first |
| --- | --- | --- |
| TIMER3 (DevNo 6, default) | Pending | Pending |
| RTC2 (DevNo 2) | Pending | Pending |

With PEI_CC_TEST_ISR_FIRST=0, PEI claims CC0 and ISR claims CC1.
With PEI_CC_TEST_ISR_FIRST=1, ISR claims CC0 and PEI claims CC1.

## Verification

Allow the application to run uninterrupted for at least 20 IRQ
callbacks (about five seconds). Inspect the global g_PeiCcTest.

- Status 2 means 250 ms ISR intervals and monotonic elapsed ticks passed.
- Errors 0 and BadIntervals 0 are expected.
- ExpectedIrqTicks is approximately 4,000,000 on TIMER3 and 8,192 on RTC2.
- MinIrqTicks and MaxIrqTicks should be near ExpectedIrqTicks.
- ElapsedTicks must keep increasing; no compare stops or clears the timer.
- Both LEDs should toggle independently; LED0 verifies PPI/GPIOTE routing.

Status 2 is NOT sufficient to prove PPI routing without observing LED0.
The diagnostic interval tolerance is 20%; it is not a precision timing
benchmark. Do not halt the CPU, set breakpoints or use semihosting while
collecting timing samples: continuous CC scheduling depends on Timer IRQs.

After observing both LEDs and Status 2/Errors 0, record hardware PASS for
the corresponding matrix configuration. No hardware PASS is claimed
by creating this test project alone.
