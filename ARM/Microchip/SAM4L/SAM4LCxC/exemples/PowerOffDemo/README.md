# SAM4L8 Xplained Pro PowerOffDemo

Connect OLED1 Xplained Pro to **EXT2**. The demo uses OLED1 **BUTTON1**,
connected through EXT2 pin 9 to **PC06 / EIC8**, as an active-low Backup wake
input. The on-board SW0 is not the wake button for this configuration.

Build the SAM4LCxC library and PowerOffDemo in the same configuration.
Flash the application and run without an active debugger session. UART output
uses USART1 at 1000000 baud on the EDBG virtual COM port.

After `Power off, press the button to wake up`, press and release OLED1
BUTTON1. The demo drains UART output before shutting it down. A successful wake restarts
the application and reports `Reset reason 0x100: wake up by a pin`. Repeat
several times. The start count should increase if backup register retention
works; record the output to verify both wake and retention.

This demo does not initialize the OLED display. Its output remains on UART.
The first hardware wake reported pin wake and advanced the retained count
from 1 to 2. Re-arming then failed because the EIC line survived Backup while
its SRAM ownership flag was cleared. The driver now uses the retained Backup
pin routing to recognize that line.

Pin assignments: Microchip SAM4L8 Xplained Pro User Guide, table 4-2, and
OLED1 Xplained Pro User Guide, extension header table.

Repeated wakes remained intermittent with EIC IMR cleared, including a
one-second button hold more than two seconds after the prompt. Wake setup now
enables the peripheral interrupt, as required by datasheet section 21.6.5,
without enabling its NVIC interrupt. Disabling a wake pin masks the peripheral
interrupt again.

After a clean rebuild on November 17, 2026, the maintainer reported eight
consecutive Backup wakes. Every wake reported reset reason 0x100, and the
retained start count advanced from 1 to 9. No intermittent failure occurred
in this run. The initial startup reported software reset (0x8). This confirms
pin wake and retention of the start-count register; it is not a test of every
backup register or a sleep-current measurement.

## Automatic AST Backup wake

Set `POWER_DEMO_TIMER_WAKE_MS` to `3000` in `src/board.h`, or define it in the
application compiler settings. Leave it at zero for the existing button run.
Rebuild the library and demo. The timer run disables the configured button wake,
sets a single AST alarm, drains the UART, then enters Backup. After three
seconds the application should restart with reset reason `0x1000` (timer wake)
and an incremented retained start count. Repeat several cycles. AST registers
survive Backup; TimerInit clears the previous alarm and wake state before
arming the next one. SRAM is lost, so timer handles are initialized again.

This configuration compiles in Debug and Release. The maintainer confirmed four
consecutive AST Backup wakes with reset reason 0x1000 and retained start counts
1 through 5. The earlier pin-wake results remain separate.

## Resumable standby profiles (not hardware revalidated)

The default configuration above remains Backup mode using USART1/EDBG.
For a resumable EIC wake test set `POWER_DEMO_RESUME_WAKE=1` in the project
compiler symbols and keep `POWER_DEMO_UART_WAKE=0`. OLED1 BUTTON1
on PC06 remains the wake source. Set
`POWER_DEMO_STANDBY_RAM_TEST=1` to check retained stack and static bytes.

For automatic AST wake in resumable sleep, set
`POWER_DEMO_RESUME_WAKE=1`, `POWER_DEMO_TIMER_WAKE_MS=1000`,
and `POWER_DEMO_UART_WAKE=0`. The demo configures the timer through
the normal Timer interface. SystemSleep selects and restores its sleep
clock configuration. Leave `POWER_DEMO_RESUME_WAKE` undefined for the
previously validated reset-on-wake Backup timer test.

For UART wake, set `POWER_DEMO_UART_WAKE=1` in board.h or the project
symbols. This selects resumable sleep and USART0 RX PB00 / TX PB01 for
the whole demo. The EDBG USART1 connection is no longer the console for
this profile. Connect a 3.3 V USB-UART adapter and common ground. Run
`python3 Python/sam4l_uart_wake.py --port PORT --baud 1000000`.
PICOUART sleep entry and USART restoration belong to the SAM4L driver
and SystemSleep. UART wake has not been hardware tested.

## System-owned sleep coordination (review 0.14)

For resumable sleep, PowerOffDemo now leaves its UART enabled and calls
`SystemSleep()` directly after draining pending output. The target system
coordinates temporary UART suspension and restoration with the driver's
clock and interrupt state, without changing its enable reference count.
The UART-wake profile retains PICOUART's own receive preparation.
Previous runs used a different suspend sequence; this refactor requires
a new clean library/demo build and board test before validation.

## Driver ownership after the power/clock refactor

During resumable sleep, the SAM4L system now coordinates idle UART,
I2C master and SPI master clock suspension without application-specific
sleep code. TWIS keeps its enabled address-match SleepWalking request.
Active masters or unsupported SPI slave sleep prevent entry. The
new coordination has not been compiled or tested on hardware.
