# nRF5340_App ClockDemo

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
PowerOffDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

Bare-metal HFXO requests/releases are balanced. No BLE stack is started.

This project runs three clock-selection cycles. UART is drained and powered
off before each change, then initialized again to update its baud divider.
Clock selection completes before the timer is initialized because some ports
retain timer module ownership after TimerDisable. It then checks the first
high-frequency timer at 1 MHz requested rate, with a 10 ms trigger over 250 ms.
The returned timer rate may differ from 1 MHz. Delay and interrupt checks use
the actual rate/period. They do not independently measure oscillator accuracy.
No watchdog is enabled. No sleep is requested by this configuration; use
PowerOffDemo for target wake validation. Reset restores startup clocks.

An omitted selector prints SKIP. A configured selector that fails prints FAIL;
unsupported selections are not silently counted as successful tests. Runtime
clock ownership, board wiring and hardware operation still require board
validation. See the cross-target power/clock demo matrix for build results.

## PCA10121 application-core timing (November 19, 2026)

Hardware UART output passed three cycles of HFXO request/release, LF RC
selection, APB readback and timer IRQ checks. The 1 MHz timer measured
`msDelay(250)` = 438251 us with 43 IRQs (FAIL) and
`nsDelay(250000000)` = 250447 us with 25 IRQs (PASS).

The application-core `SystemMicroSecLoopCnt` divisor was adjusted from
16 MHz to 28 MHz, based on that single 64 MHz board measurement. The
nanosecond factor was left unchanged. Rebuild both the nRF5340 application
library and `ClockDemo` and repeat the hardware run; the new calibration has
**not yet passed hardware testing**. Source-level check:
`python3 tests/nordic/check_nrf5340_application_delay.py`.
