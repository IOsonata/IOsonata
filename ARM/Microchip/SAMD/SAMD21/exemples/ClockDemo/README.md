# SAMD21 ClockDemo

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
ClockDemo still prints to UART and does not initialize the display.

SAMD21 hardware validation: three Standby cycles with SW0 passed, including
CPU/APB and LF descriptor checks, both delays, timer IRQs and UART RX.
The 250 ms delays measured approximately 250.99 ms at a 750 kHz timer rate.
OLED1 BUTTON1 wake remains to be checked on hardware.

An omitted selector prints SKIP. A configured selector that fails prints FAIL;
unsupported selections are not silently counted as successful tests. Runtime
clock ownership, board wiring and hardware operation still require board
validation. See the cross-target power/clock demo matrix for build results.
