# SAMD11 ClockDemo

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
PowerOffDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

Select the run in `src/board.h` with `CLOCK_DEMO_RESUME_TEST`:

- `1` (default): check the startup clock, measure both delays, then perform
  three Standby wake cycles. Press and release SW0 after each
  `Entering sleep; press SW0 to resume` message. At each UART RX prompt,
  send uppercase `U` from the terminal within 30 seconds (press Enter if the
  terminal buffers a line). This runs once before sleep and after every wake.
  Each wake checks CPU/APB
  clocks, oscillator descriptors, UART output, timer interrupts and both delays.
- `0`: run the three OSC8M clock-selection cycles, then measure both delays.
  UART is drained and powered off before each change, then initialized again.

The two modes share the generic ClockDemo source. Combining clock selection
and wake support in one D11 build exceeded flash by 600 bytes, so the board
selects the required checks at compile time. The Standby run keeps the startup
clock; it does not perform runtime source selection. The APB clock stays at
48 MHz because the SAMD setter does not support a different rate.

LF selection is omitted because a running DFLL/DPLL may own its reference.
Timer initialization is deferred until clock checks complete because some ports
retain timer module ownership after TimerDisable. SysTick (timer 4) requests
1 MHz and a 10 ms trigger. Both msDelay(250) and nsDelay(250000000) are measured
against that timer. The returned timer rate may differ from 1 MHz; checks use
the actual rate/period. They do not independently measure oscillator accuracy.
The timer is disabled and UART drained before each sleep. No watchdog is enabled.

Debug and Release use size optimization and LTO. The linker retains _sbrk for
newlib formatting after LTO. Both configurations linked with these sizes:

| Mode | Flash (text + data) | Static RAM (data + bss) |
| --- | ---: | ---: |
| Clock selection | 15,466 bytes | 2,152 bytes |
| Standby/resume | 16,120 bytes | 2,132 bytes |

RAM totals exclude stack use. PowerOffDemo remains available for the
static/stack SRAM continuity check. Reset restores startup clocks.

An omitted selector prints SKIP. A configured selector that fails prints FAIL;
unsupported selections are not silently counted as successful tests. Runtime
clock ownership, board wiring and hardware operation still require board
validation. See the cross-target power/clock demo matrix for build results.

The first board run stopped in GCLK synchronization during core selection.
The main-clock transition now preserves the existing divider while selecting
OSC8M, synchronizes that source change, and only then clears the divider.
Previously the divider was cleared while FDPLL96M still supplied 96 MHz,
exceeding the 48 MHz CPU limit. The revised write sequence passes a register
model check and both SAMD11/SAMD21 demo build configurations. The later
board run completed all three selections.

ClockDemo drains both the syslog record store and polling UART FIFO after
startup and result output, before changing clocks, and before its final idle
loop. A partial UART write leaves the remainder in syslog; draining only the
UART FIFO can leave the last visible output in the middle of a line.

The following board run completed all three core/APB selections. Its timer
reported 203202 us and 20 interrupts for a requested 250 ms delay. The old
usDelay scaling selected three 13-cycle iterations per microsecond at 48 MHz,
which predicts 203125 us. The corrected board run measured 250106 us and
25 interrupts, completing with zero failures.

The shared ARM delay now uses SystemMicroSecLoopCnt with 16 fractional bits.
SystemCoreClockUpdate refreshes it from the CPU frequency and the port's loop
calibration. The SAMD port uses 13 cycles per microsecond-delay loop and three
cycles per nanosecond-delay loop. idelay.h has no SAMD-specific calculation.
The maintainer's run of 2de35f281 passed all three clock selections and both
delay checks: msDelay measured 250094 us, nsDelay measured 250107 us, both
with 25 interrupts. ClockDemo completed with zero failures at 48 MHz.
The maintainer's Standby run at e017243bc passed all three SW0 wakes, CPU/APB
clock and LF descriptor checks, and timer interrupts. Post-wake msDelay
measured 250097-250103 us; nsDelay measured 250108-250112 us, with 25 interrupts
in every sample and zero failures. PowerOffDemo separately passed five
consecutive wakes with both 128-byte SRAM samples preserved.

The added polling UART RX check remains to be run on hardware. Missing the
response deadline increments the failure count; the demo continues. Input
cleanup is bounded, and unrelated characters do not count as a successful
response. UART interrupt/DMA receive modes are outside this check.

OSC8M identifies the PLL reference selected by SystemCoreClockSelect; the
clock tree is rebuilt to 48 MHz. It does not request an 8 MHz CPU clock.
