# CoreClockTest — generic MCU test

> **Release 0.14 migration:** This Eclipse managed project now compiles the shared
> `exemples/misc/core_clock_test.cpp` test. The legacy demo instructions below
> are historical reference only; they do **not** describe the current test flow.
> Check `src/board.h` for this board's UART, oscillator, timer and wake-pin setup.
> Use the generic test output and capability reporting to determine supported
> functionality; build and hardware validation remain target-specific.

## Historical demo notes (pre-migration)

# SAMD21 CoreClockTest

## Current clock-only configuration

CoreClockTest runs the configured OSC8M source and APB requests, timer
interrupts, calibrated delays and normal UART RX on SERCOM3/EDBG
at 115200 baud. It no longer enters Standby. The optional OLED1
wake input and SW0 Standby test belong to PowerOffDemo, whose
WAKE_BUT_* board definitions select the requested wake pin.
The new demo split is not yet compiled or hardware revalidated.

## Historical combined CoreClockTest documentation

The following instructions and hardware observations describe the
earlier combined clock/sleep demo. Ignore its old CoreClockTest sleep
prompts and CLOCK_DEMO_OLED1_WAKE guidance for the current code.

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
PowerOffDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

LF selection omitted because a running DFLL/DPLL may own its reference.

This project runs three clock-selection cycles. UART is drained and powered
off before each change, then initialized again to update its baud divider.
Clock selection completes before the timer is initialized because some ports
retain timer module ownership after TimerDisable. It then checks the first
high-frequency timer at 1 MHz requested rate, with a 10 ms trigger over 250 ms.
The returned timer rate may differ from 1 MHz. Delay and interrupt checks use
the actual rate/period. They do not independently measure oscillator accuracy.
No watchdog is enabled. After the initial timer checks, send uppercase U at
115200 baud when prompted. Three Standby cycles then check clocks, delays,
timer interrupts and UART RX after wake. Press and release SW0 at each sleep
prompt, and send U at each UART prompt. Reset restores startup clocks.

For OLED1 on EXT1, set `CLOCK_DEMO_OLED1_WAKE` to 1 in `src/board.h`.
Use OLED1 BUTTON1 (PB04) at the wake prompts. This selects the wake input;
CoreClockTest still prints to UART and does not initialize the display.

SAMD21 hardware validation: three Standby cycles with SW0 passed, including
CPU/APB and LF descriptor checks, both delays, timer IRQs and UART RX.
The 250 ms delays measured approximately 250.99 ms at a 750 kHz timer rate.
OLED1 BUTTON1 (PB04) also passed three Standby cycles with zero failures.
Both delays measured approximately 251.01 ms, with 25 interrupts per sample;
UART RX and the CPU/APB/LF descriptor checks passed after every wake.

An omitted selector prints SKIP. A configured selector that fails prints FAIL;
unsupported selections are not silently counted as successful tests. Runtime
clock ownership, board wiring and hardware operation still require board
validation. See the cross-target power/clock demo matrix for build results.
