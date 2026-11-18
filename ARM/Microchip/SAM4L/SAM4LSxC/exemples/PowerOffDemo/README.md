# SAM4LSxC PowerOffDemo

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
