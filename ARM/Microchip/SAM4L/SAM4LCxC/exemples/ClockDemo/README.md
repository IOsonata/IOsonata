# SAM4LCxC ClockDemo

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
TimerDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

Internal sources only; external crystal frequency is not assumed.

This project runs three clock-selection cycles. UART is drained and powered
off before each change, then initialized again to update its baud divider.
Clock selection completes before the timer is initialized because some ports
retain timer module ownership after TimerDisable. It then checks the first
high-frequency timer at 1 MHz requested rate, with a 10 ms trigger over 250 ms.
The returned timer rate may differ from 1 MHz. Delay and interrupt checks use
the actual rate/period. They do not independently measure oscillator accuracy.
No watchdog is enabled. Connect OLED1 to EXT2. After the initial checks, send
uppercase U at each UART prompt (1,000,000 baud). Three SLEEP0 cycles follow;
press and release OLED1 BUTTON1 (PC06) at each sleep prompt. The GPIO interrupt
must be observed, then clocks, both delays, timer interrupts and UART RX are
checked again. Expect zero failures. Hardware confirmation is pending.

SLEEP0 stops the CPU while peripheral clocks continue. This checks resumable
sleep; it does not exercise WAIT, RETENTION or reset-on-wake Backup mode.
BUTTON1 uses GPIO interrupt group 8 here. PowerOffDemo uses the separate EIC8
Backup wake route on the same pin. Reset restores startup clocks.

An omitted selector prints SKIP. A configured selector that fails prints FAIL;
unsupported selections are not silently counted as successful tests. Runtime
clock ownership, board wiring and hardware operation still require board
validation. See the cross-target power/clock demo matrix for build results.
