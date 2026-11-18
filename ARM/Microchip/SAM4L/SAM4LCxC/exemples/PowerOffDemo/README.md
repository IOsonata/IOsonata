# SAM4L8 Xplained Pro PowerOffDemo

Connect OLED1 Xplained Pro to **EXT2**. The demo uses OLED1 **BUTTON1**,
connected through EXT2 pin 9 to **PC06 / EIC8**, as an active-low
Standby or Backup wake input. The on-board SW0 is not the wake button.

Build the SAM4LCxC library and PowerOffDemo in the same configuration.
Flash the application and run without an active debugger session. UART output
uses USART1 at 1000000 baud on the EDBG virtual COM port.

The default board.h now selects resumable Standby:
`POWER_DEMO_RESUME_WAKE=1`, `POWER_DEMO_STANDBY_RAM_TEST=1`,
`POWER_DEMO_UART_WAKE=0` and `POWER_DEMO_TIMER_WAKE_MS=0`.
After `Entering Standby`, press OLED1 BUTTON1. Each wake should print
`UART rate after resume: PASS`, `CPU/APB clock after resume: PASS`,
`SRAM sample PASS: 128 static + 128 stack bytes`, and an increasing
`Resumed from Standby` count. Repeat at least ten times.

## Previously validated Backup-mode button wake

Set `POWER_DEMO_RESUME_WAKE=0` (in board.h or the project compiler
symbols), then clean rebuild and flash. The historical results below
describe this reset-on-wake mode, not the new default Standby mode.

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

Set `POWER_DEMO_RESUME_WAKE=0` and `POWER_DEMO_TIMER_WAKE_MS=3000`
in `src/board.h` or the application compiler settings. Leave the timer
wake value at zero for the button test.
Rebuild the library and demo. The timer run disables the configured button wake,
sets a single AST alarm, drains the UART, then enters Backup. After three
seconds the application should restart with reset reason `0x1000` (timer wake)
and an incremented retained start count. Repeat several cycles. AST registers
survive Backup; TimerInit clears the previous alarm and wake state before
arming the next one. SRAM is lost, so timer handles are initialized again.

This configuration compiles in Debug and Release. The maintainer confirmed four
consecutive AST Backup wakes with reset reason 0x1000 and retained start counts
1 through 5. The earlier pin-wake results remain separate.

## Resumable Standby profiles

The default board.h selects resumable EIC wake with USART1/EDBG and
retained static/stack RAM checks. OLED1 BUTTON1 on PC06 is the wake
source. The previous Backup validation does not verify this profile.

For automatic AST wake in resumable sleep, set
`POWER_DEMO_RESUME_WAKE=1`, `POWER_DEMO_TIMER_WAKE_MS=1000`,
and `POWER_DEMO_UART_WAKE=0`. The demo configures the timer through
the normal Timer interface. SystemSleep selects and restores its sleep
clock configuration. Set `POWER_DEMO_RESUME_WAKE=0` for the
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
The USART1 suspend/resume path and EIC wake were exercised in the
maintainer's November 18 hardware run below. I2C/SPI operation and
PICOUART receive wake remain separate validation tasks.

## Driver ownership after the power/clock refactor

During resumable sleep, the SAM4L system now coordinates idle UART,
I2C master and SPI master clock suspension without application-specific
sleep code. TWIS keeps its enabled address-match SleepWalking request.
Active masters or unsupported SPI slave sleep prevent entry. The
November 18 EIC wake run exercised USART1 restoration, not concurrent
I2C/SPI traffic or PDCA transfer refusal on hardware.

## Repeated RETENTION wake fix

Following the first BUTTON1 wake, the EIC interrupt status can remain
latched. An uncleared EIC wake event may cause the next WFI to return
immediately, producing rapid `Resumed from Standby` messages without a
reset. SystemWakePin uses level-triggered EIC detection. Acknowledging
the flag while BUTTON1 remains pressed does not remove the wake condition,
because the active level can set it again. SystemSleep now acknowledges
only its selected EIC wake pins immediately before the next WFI, when
the wake input can be inactive. PM WAKE is then cleared before entry.
Other EIC interrupt lines are not acknowledged by this operation.

Rebuild the library and PowerOffDemo after this change. The hardware
validation is ten distinct BUTTON1 press/release cycles, with the CPU
remaining asleep between presses. Repeated log output without a new
button press is a failure even when UART, CPU/APB clock and SRAM checks
all report PASS. A host register model checks the EIC acknowledgement;
the button-wake hardware test was repeated on November 18 as recorded
below; other wake sources and current consumption remain unverified.

## November 18, 2026: EIC RETENTION wake retest

With the default resumable Standby profile and OLED1 BUTTON1, the
maintainer initially observed rapid repeated resume messages after the
first button press, even after the button was released. The counter
advanced into the 0x000005xx range without a new startup banner;
UART rate, CPU/APB clocks and SRAM checks all passed during that loop.
This was not a reset-on-wake failure.

After changing SystemSleep to acknowledge the EIC wake interrupt
immediately before the next WFI (while the input may have been
released), the maintainer reported that the continuous wake loop no
longer occurred. The serial log shows six consecutive resume counts
0x00000001 through 0x00000006, each followed by:

- UART rate after resume: PASS
- CPU/APB clock after resume: PASS
- SRAM sample PASS: 128 static + 128 stack bytes

No second startup banner appears in the provided log. This is hardware
evidence of six resumable wake/resume cycles with intact UART, clocks
and sampled RAM. The log itself does not time-stamp individual button
presses or measure sleep current. Ten distinct button presses with a
quiet interval between them, AST resumable wake and PICOUART receive
wake remain additional validation goals.
