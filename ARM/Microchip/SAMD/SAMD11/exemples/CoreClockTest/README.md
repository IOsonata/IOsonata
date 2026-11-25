# SAMD11D14AM CoreClockTest

Build the SAMD11 library and CoreClockTest with matching Eclipse configurations.
The default image runs the shared HF/LF clock checks, CPU power modes,
peripheral clock rate requests and readback, then tests RTC timing, periodic
and one-shot callbacks, and TimerPowerOff. Output uses the EDBG UART.

To test another timer, set `CORE_CLOCK_TEST_TIMER_DEVNO` in the application's
C++ preprocessor symbols and rebuild. Each image runs the same clock checks.

| DevNo | Timer | Flash image (bytes) |
| ---: | --- | ---: |
| 0 (default) | RTC | 15,064 |
| 1 | TC1 | 15,940 |
| 2 | TC2 | 15,988 |
| 3 | TCC0 | 16,200 |
| 4 | SysTick | 11,944 |

Sizes include flash alignment with xPack Arm GCC 15.2.1 and the 16 KiB SAMD11
linker script. The full generic test, including concurrent timers, the
independent HF timing reference and sleep-delay checks, exceeds flash by
6,772 bytes with delay calibration disabled and 8,520 bytes with it enabled.

## Delay calibration during porting

Set `CORE_CLOCK_TEST_DELAY_CALIBRATION=1` and rebuild the application.
This 12,104-byte image uses SysTick to measure the delay loops at 48 and
24 MHz and then restores performance mode. The two `Delay divisors` lines
give the microsecond and nanosecond divisors scaled by 1,000,000.

## Earlier single-timer hardware measurements

| DevNo | Peripheral | Elapsed for 110 ms | Periodic callbacks | One-shot and PowerOff |
| --- | --- | ---: | ---: | --- |
| 0 | RTC | 110,779 us | 5 | PASS |
| 2 | TC2 | 110,462 us | 5 | PASS |
| 3 | TCC0 | 110,272 us | 5 | PASS |
| 4 | SysTick | 110,048 us | 5 | PASS |

TC1 and TC2 share the TC driver.
