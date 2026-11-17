# SAMD11 ClockDemo

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
PowerOffDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

LF selection omitted because a running DFLL/DPLL may own its reference.

This project runs three clock-selection cycles. UART is drained and powered
off before each change, then initialized again to update its baud divider.
Clock selection completes before the timer is initialized because some ports
retain timer module ownership after TimerDisable. It then checks SysTick (timer 4) at 1 MHz requested rate, with a 10 ms trigger over 250 ms.
The returned timer rate may differ from 1 MHz. Delay and interrupt checks use
the actual rate/period. They do not independently measure oscillator accuracy.
Debug and Release use size optimization and LTO. The linker retains _sbrk
for newlib formatting after LTO. Flash use is 15,294 bytes (15,086 text plus
208 data); static RAM is 2,144 bytes, before stack use.
No watchdog is enabled. No sleep is requested by this configuration; use
PowerOffDemo for target wake validation. Reset restores startup clocks.

An omitted selector prints SKIP. A configured selector that fails prints FAIL;
unsupported selections are not silently counted as successful tests. Runtime
clock ownership, board wiring and hardware operation still require board
validation. See the cross-target power/clock demo matrix for build results.

The first board run stopped in GCLK synchronization during core selection.
The main-clock transition now preserves the existing divider while selecting
OSC8M, synchronizes that source change, and only then clears the divider.
Previously the divider was cleared while FDPLL96M still supplied 96 MHz,
exceeding the 48 MHz CPU limit. The revised write sequence passes a register
model check and both SAMD11/SAMD21 demo build configurations. Board retest is
pending.

ClockDemo drains both the syslog record store and polling UART FIFO after
startup and result output, before changing clocks, and before its final idle
loop. A partial UART write leaves the remainder in syslog; draining only the
UART FIFO can leave the last visible output in the middle of a line.

The following board run completed all three core/APB selections. Its timer
reported 203202 us and 20 interrupts for a requested 250 ms delay. The old
usDelay scaling selected three 13-cycle iterations per microsecond at 48 MHz,
which predicts 203125 us. SAM D11/D21 now calculate the iteration count from
the full requested interval and current CPU frequency before dividing by
13 million. ClockDemo and PowerOffDemo rebuild for both targets in Debug and
Release. The corrected delay still requires a board run.

OSC8M identifies the PLL reference selected by SystemCoreClockSelect; the
clock tree is rebuilt to 48 MHz. It does not request an 8 MHz CPU clock.
