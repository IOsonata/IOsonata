# STM32L432 PowerOffDemo

Build the matching library and this Eclipse managed project in the same
configuration. UART pins come from TimerDemo; default baud is 1,000,000.
This configuration does not assume a board wake pin. After reset-reason and
retention-register output, it powers off the UART and requests MCU power off.
Use RESET or a power cycle to start again. It is not a GPIO wake test.

For a supported wake input, define WAKE_BUT_PORT, WAKE_BUT_PIN and
WAKE_BUT_PINOP in board.h, with WAKE_BUT_SENSE for its active transition.
Verify that the target SystemWakePin supports that pin and power mode.
PowerOffDemo on a target with existing verified wiring is the reference for
that board. No watchdog is enabled. Build and hardware status are separate;
see the cross-target demo matrix.
