# CorePowerTest — generic MCU test

> **Release 0.14 migration:** This Eclipse managed project now compiles the shared
> `exemples/misc/core_power_test.cpp` test. The legacy demo instructions below
> are historical reference only; they do **not** describe the current test flow.
> Check `src/board.h` for this board's UART, oscillator, timer and wake-pin setup.
> Use the generic test output and capability reporting to determine supported
> functionality; build and hardware validation remain target-specific.

## Historical demo notes (pre-migration)

# STM32L4S9 CorePowerTest

Build the matching library and this Eclipse managed project in the same
configuration. UART pins come from TimerDemo; default baud is 1,000,000.
This configuration does not assume a board wake pin. After reset-reason and
retention-register output, it powers off the UART and requests MCU power off.
Use RESET or a power cycle to start again. It is not a GPIO wake test.

For a supported wake input, define WAKE_BUT_PORT, WAKE_BUT_PIN and
WAKE_BUT_PINOP in board.h, with WAKE_BUT_SENSE for its active transition.
Verify that the pin interrupt of the target wakes the MCU from that power mode.
CorePowerTest on a target with existing verified wiring is the reference for
that board. No watchdog is enabled. Build and hardware status are separate;
see the cross-target demo matrix.
