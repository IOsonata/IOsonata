# SAM4L8 Xplained Pro PowerOffDemo

Build the SAM4LCxC library and PowerOffDemo in matching configurations.
Flash with EDBG and run without an active debugger session. The current
review-branch default tests USART0/PICOUART wake using an external USB-UART
adapter on PB00/PB01 at 1000000 baud during normal operation.

For the validated EIC wake profile, set `POWER_DEMO_UART_WAKE=0`,
connect OLED1 Xplained Pro to **EXT2**, and use its **BUTTON1**,
connected through EXT2 pin 9 to **PC06 / EIC8**. That profile uses
USART1 at 1000000 baud on EDBG's virtual COM port. The on-board SW0
is not the wake button.

The default board.h now selects USART0/PICOUART receive wake for the
next hardware test: `POWER_DEMO_RESUME_WAKE=1`,
`POWER_DEMO_STANDBY_RAM_TEST=1`, `POWER_DEMO_UART_WAKE=1`, and
`POWER_DEMO_TIMER_WAKE_MS=0`. This is resumable WAIT sleep.
For the previously validated EIC button-wake test, override
`POWER_DEMO_UART_WAKE=0`, clean-build, and monitor USART1 on EDBG.
After `Entering Standby`, press OLED1 BUTTON1. Each wake should print
`UART rate after resume: PASS`, `CPU/APB clock after resume: PASS`,
`SRAM sample PASS: 128 static + 128 stack bytes`, and an increasing
`Resumed from Standby` count. Repeat at least ten times.

## Next hardware test: PICOUART receive wake

With the default UART-wake profile in board.h, USART0 RX PB00 is shared
with PICOUART and USART0 TX is PB01. Connect a 3.3 V USB-UART adapter:
adapter TX to EXT1 (or EXT4) pin 13 / PB00, adapter RX to
EXT1 (or EXT4) pin 14 / PB01, and adapter GND to pin 19.
These are the board's USART0 pins (Table 4-1 or 4-4, SAM4L8
Xplained Pro user guide). EXT2 pin 13/14 instead carries USART1
PC26/PC27 and will not work for this test.
Do not connect a 5 V UART signal or power the board from the adapter.
This is not the EDBG virtual COM port.

Build the SAM4LCxC library and PowerOffDemo in matching Release
configurations. Flash the application with EDBG and stop the debugger.
Open the adapter on macOS with:

```sh
ls /dev/cu.usbserial*
python3 Python/sam4l_uart_wake.py --port /dev/cu.usbserial-ADAPTER --baud 1000000 --cycles 5
```

Start the helper before resetting the board. The helper checks normal
USART0 RX at 1000000 baud, acknowledges each sleep request by sending
`R` at 1000000 baud, and waits for the MCU's 9600-baud setup prompt.
It then switches the FTDI adapter to 9600 baud **before** PICOUART is
activated. The MCU holds its normal USART0 awake for 750 ms to allow
the adapter to settle, while the host waits 1.25 s before sending `U`
and switching back to 1000000 baud. Each wake must report `UART wake byte: PASS`,
`UART rate after resume: PASS`, and
`CPU/APB clock after resume: PASS`. The SRAM sample should also
pass. This PICOUART profile completed ten consecutive physical-board wake cycles
on November 18, 2026; see the latest results below. Sleep current is not yet measured.

If the host script prints `Timeout: waiting for PowerOffDemo output`:
check that the adapter is wired to EXT1 or EXT4, not EXT2; verify
adapter RX is connected to pin 14 (MCU TX), adapter TX to pin 13
(MCU RX), and both share ground. Ensure the latest UART-wake
PowerOffDemo is flashed and no other program owns the adapter port.
Use a scope or logic analyzer on PB01 to distinguish missing UART
transmission from a host baud/wiring issue before modifying the driver.

## Previously validated Backup-mode button wake

Set `POWER_DEMO_RESUME_WAKE=0` and `POWER_DEMO_UART_WAKE=0`
(in board.h or the project compiler symbols), then clean rebuild
and flash. The historical results below
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

Set `POWER_DEMO_RESUME_WAKE=0`, `POWER_DEMO_UART_WAKE=0`, and
`POWER_DEMO_TIMER_WAKE_MS=3000` in `src/board.h` or the application
compiler settings. Leave the timer
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

Set `POWER_DEMO_UART_WAKE=0` to select resumable EIC wake with
USART1/EDBG and retained static/stack RAM checks. OLED1 BUTTON1 on
PC06 is the wake source. This profile passed hardware validation as
recorded below. The previous Backup validation remains separate.

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
quiet interval between them and PICOUART receive wake remain
additional validation goals. The separate AST resumable wake result is
recorded below.

## Diagnosing an AST Standby initialization failure

With `POWER_DEMO_RESUME_WAKE=1` and `POWER_DEMO_TIMER_WAKE_MS=1000`,
the SAM4L demo prints `SAM4L TimerInit stage 0x...` when the
initial call to TimerInit returns false. It also reports BPM_PMCON,
BSCIF_PCLKSR, PM_PBDMASK and NVIC_ISER1. If AST register access has
been enabled (stage 5 or later), it reports AST_SR, AST_CLOCK and AST_CR.
The diagnostic only reads hardware registers, does not alter the
clock or timer driver, and is compiled only for SAM4L timer Standby
tests. The board.h configuration is not changed by this diagnostic.

Stage values in hex: 1 invalid configuration, 2 previous timer owner,
3 low-frequency clock source, 4 frequency selection, 5 AST sync,
6 NVIC ownership conflict, 7 AST setup, 8 counter reset, 9 start.
Report the complete UART failure output when an AST failure occurs.

## November 18: AST initialization synchronization failure

The maintainer's first resumable AST test reported
`SAM4L TimerInit stage 0x00000005`, `BPM_PMCON 0x00010002`,
`BSCIF_PCLKSR 0x00001582`, `PM_PBDMASK 0x0000003F`,
`NVIC_ISER1 0x00000000`, `AST_SR 0x01000000`,
and `AST_CR 0x00000000`. The selected RC32K clock was reported
ready; AST_SR.BUSY was set while AST_SR.CLKBUSY was clear.
The observed values do not reveal AST_CLOCK.CEN, which was not yet
printed.

Previously, the timer initializer waited for both AST_SR.BUSY and
AST_SR.CLKBUSY before enabling the AST prescaler clock. A pending
register synchronization may remain busy until that clock runs.
The code now waits for CLKBUSY while changing AST_CLOCK and waits
for BUSY only after enabling AST_CLOCK.CEN. A register-model regression
simulates BUSY set with CEN clear and verifies AST initialization.
The subsequent physical-board run completed 12 automatic AST wakes,
recorded below.

## November 18, 2026: automatic AST RETENTION wake

After enabling the AST prescaler clock before waiting for AST_SR.BUSY,
the maintainer reported a repeating PowerOffDemo log with
`POWER_DEMO_RESUME_WAKE=1` and `POWER_DEMO_TIMER_WAKE_MS=1000`.
The program intentionally re-arms its one-shot AST alarm and goes back
to Standby after each wake. Unlike the earlier button-wake regression,
repeated wakeups are expected in this configuration.

The captured log shows 12 consecutive automatic AST wake/resume cycles
(counts 0x00000001 through 0x0000000C), all reporting:

- UART rate after resume: PASS
- CPU/APB clock after resume: PASS
- Standby timer wake: PASS
- SRAM sample PASS: 128 static + 128 stack bytes

The timer check includes an accepted period and a wake callback, and
checks elapsed timer time against a 98-102 percent window of the
configured 1000 ms. No new startup banner appears, supporting
resumable RETENTION rather than reset-on-wake. This result verifies the
AST initialization and automatic alarm wake sequence on the connected
SAM4LC8C board; power consumption and independent wall-clock timing
have not been measured. PICOUART wake remains unverified on hardware.

## November 18, 2026: stale AST wake after switching demos

After 12 successful automatic AST Standby cycles, the maintainer
rebuilt PowerOffDemo with `POWER_DEMO_TIMER_WAKE_MS=0` for EIC-only
button wake. It again resumed continuously without new button presses.
The timer profile is no longer active in the new firmware, but SAM4L
AST hardware survives non-power-on resets. Its previously configured
AST_WER/AST_CR wake state can therefore persist when the button-only
application does not call TimerInit.

SystemInit now disarms inherited AST interrupts, wake enables and
pending events once during startup, after configuring the selected
32 kHz oscillator and before drivers initialize. It does not alter
the low-frequency oscillator or reset AST during a normal SystemSleep
call. If a clock is disabled while AST BUSY remains set, startup
enables the AST prescaler clock before waiting for synchronization.
The system restores the original PBD AST clock gate afterward.

A host register-model test covers inherited AST wake enables after a
firmware reset, an idle AST, and failure on stuck clock synchronization.
After the startup cleanup was applied, the maintainer reported that
BUTTON1 wake worked again with `POWER_DEMO_TIMER_WAKE_MS=0` without the
continuous loop. The provided serial log shows six consecutive resumable
wakes (counts 0x00000001 through 0x00000006), with every cycle reporting
UART rate restoration, CPU/APB clock restoration, and 128-byte static
plus 128-byte stack SRAM sample PASS. This validates the EIC-only wake
path after switching away from the AST test. The improvement is consistent
with stale AST wake state being cleared at startup; the log does not by
itself establish the exact underlying register cause.

Independent power measurements and PICOUART receive wake remain pending.

## November 18, 2026: PICOUART first hardware observation

With the 3.3 V adapter correctly connected to USART0, the maintainer
received the startup banner and `Entering UART sleep`. The host helper
sent `U` at 9600 baud and subsequently displayed `UART wake byte: FAIL`.
This does not establish whether SystemSleep entered WAIT or returned
before the host sent the byte, because both the host and the MCU
previously waited about 100 ms before proceeding.

The host script now waits 350 ms while checking for an unsolicited
MCU reply before sending `U`. If an early reply arrives, it reports
that sleep was refused or another event caused an early wake.
The SAM4L UART driver also records one internal 32-bit diagnostic
during wake preparation and restoration. On a failed UART wake,
PowerOffDemo prints `SAM4L UART sleep diagnostic 0x........`.

Diagnostic bits:

| Bit | Meaning |
| --- | --- |
| 0 | Wake requested |
| 1 | Sleep preparation rejected |
| 2 | PICOUART preparation completed |
| 3 | PICOUART restore executed |
| 4 | PM_ISR.WAKE was set at restore |
| 5 | PM_WCAUSE reported PICOUART |
| 6 | PICOUART_SR.DRDY was set |
| 7 | Captured byte queued in UART RX FIFO |
| 8 | Invalid or already active wake owner |
| 9 | Required LF clock not ready |
| 10 | PICOUART already enabled |
| 11 | Conflicting PM wake registration or pending PM IRQ |
| 12 | USART0 clock gate disabled |
| 13 | Pending UART TX |
| 14 | Pending UART RX |
| 15 | PICOUART RX pin held low |
| 16 | PICOUART DRDY already set before rearming |
| 17 | PICOUART DRDY set immediately after enabling |
| 18 | PM wake status already set immediately after enabling |
| 19 | PM wake cause previously reported PICOUART |
| 24..31 | Captured byte, if a resume occurred |

This trace is restricted to the SAM4L UART implementation and its
existing private sleep state. No generic UART, SystemSleep, EIC or
AST API was modified. It is intended for removal after hardware
validation and is not itself evidence of PICOUART wake support.

## November 18: unsolicited PICOUART frame at wake entry

The next trace was `SAM4L UART sleep diagnostic 0xFF0000FD`.
It shows PICOUART was armed, PM reported a PICOUART wake, the
peripheral reported a received frame, and the received byte 0xFF was
delivered into the UART FIFO. The host had not yet transmitted its
requested 0x55 ('U') byte, so this is an unsolicited receive event.
The USART0 console and UART and clock restoration were functioning.

The SAM4L datasheet, section 25.6.1, specifies that PICOUART DRDY
is cleared on detection of a new start bit, not by reading RHR.
The driver now samples DRDY before and immediately after arming
PICOUART, plus the prior PM wake cause and PM wake status immediately
after arming. Bits 16-19 distinguish previously retained receiver
status from a new frame or wake event arising after handover.
No valid receive byte is discarded, and the full-frame wake mode
is unchanged. The source of the unsolicited 0xFF remains unverified.

## November 18: verify normal USART0 RX before PICOUART

The next trace was `SAM4L UART sleep diagnostic 0xFF0200FD`, meaning
the PICOUART DRDY flag became set immediately upon enabling the
receiver, with received byte 0xFF, before the host transmitted U.
The startup banner establishes that USART0 TX and the adapter RX
work, but does not establish the opposite direction (adapter TX
to PB00). It is not yet possible to attribute the unwanted frame to
the USART handover rather than an unconnected or unstable RX input.

PowerOffDemo now checks normal USART0 reception before testing
PICOUART. The host helper responds to `UART RX: send U at 1000000 baud`
with one U at the normal console rate and verifies the
`UART RX normal: PASS` result. A failure is reported before
enabling the wake receiver. This test does not change driver
clock management or PICOUART wake behavior.

If normal RX passes and the PICOUART diagnostic still reports
premature DRDY, check PB00 with a scope or logic analyzer across
USART0 disable and PICOUART enable. The idle state should stay
high, and no start bit should occur before the host wake byte.

## November 18: PICOUART wake handover sequencing

The normal USART0 RX preflight passed at 1000000 baud with the external
adapter on PB00/PB01. PICOUART still produced `0xFF0200FD` before the
host transmitted the 9600-baud wake byte. The diagnostic confirms
DRDY became set immediately after enabling PICOUART even though
it was clear before enabling, and the PM subsequently reported a
PICOUART wake. The received 0xFF was not the host's U (0x55).

The driver previously suspended USART0 and enabled the PM
asynchronous PICOUART wake before enabling PICOUART itself.
This can expose the PM to a receiver event during handover.
It now configures and enables PICOUART while the idle USART0 RX
input remains connected and its peripheral clock is running,
then disables the USART and finally enables PM_AWEN.PICOUART.
The full-frame wake source and generic wake API are unchanged.
The source ordering is checked by `tests/sam4l/run_power.py`.

This sequencing is a targeted hardware fix candidate; it has not
yet been proven to suppress the immediate 0xFF on the board.
If DRDY still asserts as soon as PICOUART is enabled, the next
diagnostic is the PB00 voltage and waveform during the handover,
not a change to the application wake token.

## November 18: PICOUART retained wake cleanup at startup

With USART0 normal RX validated, PICOUART still produced a premature
0xFF frame. The repeated diagnostic was `0xFF0200FD`: its DRDY bit
was clear before enabling the peripheral but set immediately after.
Changing the USART0-to-PICOUART handover sequence did not change
this result.

The Microchip ASF driver confirms SOURCE=2 selects full-frame wake.
The SAM4L datasheet identifies PICOUART as a low-frequency receiver
in the backup clock domain. A non-power-on reset can retain peripheral
configuration while the owning software state is reinitialized.
SystemInit now disables inherited PICOUART reception and its old
wake selection after initializing the 32 kHz clock, before any UART
driver initializes. It waits for PICOUART_SR.EN to clear before
writing PICOUART_CFG, as required by the hardware. An unsuccessful
disable stops startup with SAM4L_STARTUP_PICOUART instead of modifying
configuration while the receiver is active. Only the PICOUART bits in PM_AWEN and
BPM_BKUPWEN are cleared; other wake selections are preserved.
The original PBD peripheral clock mask is restored.

The host register test checks the owned wake bits, config, disable
command and peripheral clock restoration. This startup cleanup is
not proven to clear a retained DRDY flag, which the datasheet says
is cleared by detecting a new start bit. It is also not yet known
whether the premature 0xFF is retained hardware state or a fresh
false frame. A complete power cycle is an additional distinguishing
test. Do not classify this as a successful PICOUART wake until the
external 9600-baud U is received after sleep entry.

## November 18: live PB00 input check for PICOUART

After the PICOUART startup reset, the physical board still received
an unsolicited 0xFF when PICOUART was enabled, with diagnostic
`0xFF0200FD`. The normal USART0 RX preflight continued to pass.

The SAM4L datasheet (23.6.1.3) notes GPIO_PVR is not updated for
a peripheral-owned pin unless GPIO input monitoring is enabled.
The earlier idle-level check used IOPinRead(PB00) without enabling
GPIO ownership or an interrupt, so it was not a reliable check of
the live RX signal. In the revised SAM4L UART sleep path,
Sam4lUartPrepareSleep temporarily assigns PB00 to GPIO input
with the existing RX pull-up, checks its physical idle level, and
then disables USART0 before enabling PICOUART. The original GPIO
clock gate and peripheral pin selection are restored on wake
before USART0 RX is enabled again. The normal USART0 baud rate,
PICOUART 9600-baud full-frame mode and generic UART API do not
change. On resume the code waits for PICOUART_SR.EN to clear
before writing PICOUART_CFG.

The temporary diagnostic adds bit 20 when PB00 input monitoring was
enabled and bit 21 when the live input was high before PICOUART
enable. If bit 15 is set, the live input was low and the driver
refused sleep. The expected new successful handover diagnostics
retain bits 20 and 21; an unsolicited 0xFF with these bits set
would show the line was high at the handover check, not prove
that a correct wake byte was received.

The new GPIO handover source-order check was added to
`tests/sam4l/run_power.py`. This is a hardware test candidate
and has not been compiled or validated on the SAM4LC8C board.

## November 18: GPIO input Schmitt-trigger initialization

On the first hardware run after the PB00 GPIO handover change, the
diagnostic was `0x00108003`. It reports GPIO ownership enabled
(bit 20), RX sampled low (bit 15), and sleep preparation rejected
(bit 1). PICOUART was never armed. This replaces the previous
false-frame wake behavior and proves neither PICOUART wake nor a
physical low level on the adapter line.

The IOsonata SAM4L `IOPinConfig()` implementation sets
`GPIO_STERS` while configuring a GPIO input, but the temporary
PB00 input setup in `Sam4lUartPrepareSleep()` omitted that write.
The driver now sets `GPIO_STERS` before enabling GPIO ownership,
then samples PB00. It remembers whether the Schmitt trigger had
previously been enabled and restores that state if the input
check rejects sleep or when the UART is resumed.
The original GPIO ownership and clock state remain preserved.

This is a candidate fix for the low GPIO sample; source ordering
checks were updated. The board has not yet validated this revision.

## November 18: PICOUART direct RX and early-frame refusal

The latest one-cycle hardware run again received 0xFF before the host
transmitted its 9600-baud U. Diagnostic `0xFF3200FD` records the
temporary PB00 GPIO input probe as idle/high, followed by immediate
PICOUART DRDY and an unsolicited received byte 0xFF. This proves the
normal USART0 RX link and the sampled idle level were working, but
does not establish the presence or origin of a short transition during
the handover.

The SAM4L datasheet, section 25.5.1 and table 25-5, specifies that
PICOUART is connected directly to PB00/USART0 RXD and requires no
GPIO mux assignment. The driver now leaves PB00 owned by USART0,
enables PICOUART while USART0 is still active, waits for PICOUART_SR.EN,
and checks that no frame is already pending before disabling USART0
and arming PM_AWEN. A stale or early DRDY or an enable timeout refuses
sleep, rolls back the peripheral configuration, and keeps UART active.
A frame captured during an aborted handover is delivered to the RX
FIFO rather than silently discarded. The full-frame wake mode and
public UART API remain unchanged.

The temporary GPIO probe and its state restoration were removed.
Diagnostic bit 22 now indicates a PICOUART enable timeout; bits 20
and 21 are no longer generated. The host helper explicitly exits on
the first FAIL line, even when the application itself continues
running. Tests check direct-input source order and rollback cases.

**Hardware validation pending.** This is a targeted handover fix
candidate, not a verified PICOUART wake result. Retest with the
external 3.3 V adapter and check for a quiet 350 ms interval followed
by a 9600-baud U. If early DRDY persists without any GPIO mux
change, capture the PB00 waveform during activation.

## November 18: PICOUART latched-DRDY handling

The next hardware run on the direct-RXD0 handover branch reported
`0xFF0200CB` followed by repeated `0x00010003`. The first
attempt observed DRDY and byte 0xFF during PICOUART activation,
then rejected sleep before PM wake was armed. The subsequent attempts
were rejected because DRDY remained set.

Datasheet 42023H, sections 25.6.1 and 25.6.2, states that DRDY is
latched until a new start bit is detected and that full-frame wake
occurs when a valid frame is received. A previously latched DRDY
cannot be cleared by reading RHR or disabling the receiver. The
driver now records this retained status but does not reject sleep
solely because DRDY is set, either before or after enabling PICOUART.
The receiver still requires SR.EN, uses full-frame wake (SOURCE=2),
and enables PM_AWEN after the handover.

On resume, only a PICOUART wake reported by PM together with DRDY
permits moving the received byte to the normal UART FIFO.
Unsolicited 0xFF recorded before arming is not automatically
delivered to the UART application.

PowerOffDemo now stops its UART-wake profile on the first failed
reception instead of incrementing the resumed-standby counter and
continuously retrying. The host helper already terminates on an
unsolicited early response. This does not change the generic
SystemSleep/UARTEnableWake interfaces.

This is an unverified hardware hypothesis: the PM wake signal must
be a new frame event, rather than a level continuously asserted by
latched DRDY. Retest with a single U after a quiet interval to
distinguish these cases. No successful PICOUART hardware wake has
yet been demonstrated.

## November 18: correct host baud-switch ordering

The next physical-board run still reported `0xFF0200FD` and an
unsolicited PICOUART wake *before* the Python helper changed the FTDI
baud rate. The helper waited 350 ms for an early MCU response while
USART0 had already been handed over to PICOUART. This was the wrong
ordering for isolating the host adapter's baud-rate transition.

PowerOffDemo now requests a normal-speed `R` handshake before each
attempt and emits a final 1 Mbaud setup prompt. It drains UART TX
and delays 750 ms with USART0 still enabled; only then does it call
`SystemSleep()` and allow PICOUART activation. The host switches
to 9600 immediately after the prompt, waits 1.25 s while checking
for unexpected data, sends one `U`, and returns to the normal
1 Mbaud console rate. No new generic UART or power API is involved.

If the MCU still wakes with `0xFF` before `U`, the baud-change
timing has been isolated and the next investigation is the PICOUART
input and activation behavior. This is a test-sequence correction;
physical wake validation remains pending.

## November 18: first successful PICOUART hardware wake

The maintainer's first run with host baud changed before PICOUART
activation produced `UART RX normal: PASS`, followed by a 9600-baud
`U`, `UART wake byte: PASS`, `UART rate after resume: PASS`, and
`CPU/APB clock after resume: PASS`. The host reported
`UART wake cycle 1/1 PASS`: one successful hardware PICOUART
receive wake with USART0 and the clocks restored.

The prior helper accepted that result before reading the MCU resume
counter and SRAM-retention message. SRAM retention therefore was
not independently confirmed by the host in that run.

The host now requires the next MCU resume count and
`SRAM sample PASS: 128 static + 128 stack bytes` before accepting
any wake cycle. Simulated serial tests cover two successful cycles,
RAM failure, absent RAM output, and a wrong resume count.

Next hardware acceptance target: 10 consecutive cycles passing all
checks, followed by separate sleep-current measurement. The host
handshake and baud switching belong only to the test harness,
never the production SystemSleep interface.

## November 18, 2026: ten consecutive PICOUART wakes PASS

The maintainer reran the synchronized USART0/PICOUART wake profile
using `python3 Python/sam4l_uart_wake.py --baud 1000000 --cycles 10`
with the external 3.3 V adapter connected to PB00/PB01.

All ten cycles passed the host's complete acceptance criteria:

| Check | Physical-board result |
| --- | --- |
| Normal USART0 RX at 1000000 baud | PASS at startup |
| FTDI switch to 9600 baud before PICOUART activation | Performed each cycle |
| PICOUART wake byte `U` (0x55) at 9600 baud | 10/10 PASS |
| USART0 configured baud after resume | 10/10 PASS |
| CPU/APB clocks after resume | 10/10 PASS |
| Resume counter | 0x00000001 through 0x0000000A, sequential |
| SRAM sample: 128 static + 128 stack bytes | 10/10 PASS |

The final host message was
`[host] UART wake cycle 10/10 PASS, including SRAM.`
No UART wake, clock-restore or sampled-RAM failure occurred in the log.

This verifies repeated wake-byte delivery and resumption under the
PowerOffDemo test handshake. It does **not** independently measure
low-power current or prove all SRAM bytes were retained. Other wake
sources, simultaneous peripheral activity and long-duration sleep
reliability remain outside this test.

The host handshake coordinates external test traffic only.
Production `SystemSleep()` remains autonomous: the UART driver
arms PICOUART and restores USART0 without host participation.
No UART driver or system power code was changed for this result.
