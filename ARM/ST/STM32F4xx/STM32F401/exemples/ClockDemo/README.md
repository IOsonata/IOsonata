# STM32F401 ClockDemo

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
TimerDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

The port lacks generic runtime selectors. This project checks startup readback and timer operation only.

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
