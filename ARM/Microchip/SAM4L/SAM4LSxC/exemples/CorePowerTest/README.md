# CorePowerTest — generic MCU test

> **Release 0.14 migration:** This Eclipse managed project now compiles the shared
> `exemples/misc/core_power_test.cpp` test. The legacy demo instructions below
> are historical reference only; they do **not** describe the current test flow.
> Check `src/board.h` for this board's UART, oscillator, timer and wake-pin setup.
> Use the generic test output and capability reporting to determine supported
> functionality; build and hardware validation remain target-specific.

## Historical demo notes (pre-migration)

# SAM4LSxC CorePowerTest

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

## Automatic AST Backup wake

Set `POWER_DEMO_TIMER_WAKE_MS` to `3000` in `src/board.h`, or define it in the
application compiler settings. Leave it at zero for the existing button run.
Rebuild the library and demo. The timer run disables the configured button wake,
sets a single AST alarm, drains the UART, then enters Backup. After three
seconds the application should restart with reset reason `0x1000` (timer wake)
and an incremented retained start count. Repeat several cycles. AST registers
survive Backup; TimerInit clears the previous alarm and wake state before
arming the next one. SRAM is lost, so timer handles are initialized again.

This configuration compiles in Debug and Release. AST Backup wake hardware
validation is pending; the earlier pin-wake results remain separate.

## Resumable sleep profiles

An application with a supported wired wake input may set
`POWER_DEMO_RESUME_WAKE=1` for standby. With
`POWER_DEMO_TIMER_WAKE_MS=1000`, an AST alarm provides automatic
standby wake. Unlike Backup mode, execution resumes in place.

`POWER_DEMO_UART_WAKE=1` selects USART0 PB00/PB01 and resumable
PICOUART wake instead of USART1. Use a 3.3 V adapter, transmit
one U at 9600 baud to wake, then return to the normal console baud.
See `exemples/uart/sam4l_wake.md`. UART wake and these standby modes
remain without hardware validation on this board.

## System-owned sleep coordination (review 0.14)

For resumable sleep, CorePowerTest now leaves its UART enabled and calls
`SystemSleep()` directly after draining pending output. The target system
coordinates temporary UART suspension and restoration with the driver's
clock and interrupt state, without changing its enable reference count.
The UART-wake profile retains PICOUART's own receive preparation.
Previous runs used a different suspend sequence; this refactor requires
a new clean library/demo build and board test before validation.
