# Power and clock integration review for 0.14

Working branch: review_system_power_014

This review follows AGENTS.md and the existing system initialization and
DeviceIntrf API. The system supplies power and clock policy; peripherals own
their resources. Applications configure peripherals, choose wake sources
through public interfaces and request sleep. Normal application code must
not select a target-specific sleep clock or restore peripheral registers.

## Existing interfaces and ownership

- SystemInit establishes the clock tree and default power state.
- SystemCoreClockSelect, SystemPeriphClockSet/Get and
  SystemLowFreqClockSelect manage explicit clock changes. Active users can
  block reconfiguration. This is separate from entering SystemSleep.
- SystemCoreClockUpdate maintains delay scaling after clock changes.
  SAM4L and SAMD update the ARM SystemMicroSecLoopCnt factor. ESP32-C3/C6
  use their RISC-V cycle-counter timing and SystemCoreClockPeriodus/ns.
- DeviceIntrf Enable/Disable use reference counting. PowerOff releases
  a device and requires initialization before reuse. Drivers own their
  module clock gates and their transfer-rate programming.
- UARTEnableWake is an optional per-device request. The SAM4L USART0
  implementation manages PICOUART, PM.AWEN, RX byte recovery and baud
  restoration internally; the generic fallback reports unsupported.
- SystemSleep must select compatible power and wake conditions and return
  with the application's supported resources usable. SystemPowerOff is
  reset-on-wake and is not equivalent to Standby.

## Refactor on this branch

ClockDemo no longer calls SystemSleep or configures application-side
sleep baud, wake pins or sleep APB dividers. It retains explicit clock
selection, bus divider, LF clock, delay, timer IRQ and optional UART RX
checks. ClockDemo may power off and reinitialize a UART during an
intentional explicit clock-rate change; this is not a sleep operation.

PowerOffDemo owns the existing GPIO, Backup and retention checks and
now has selectable SAM4L UART and resumable AST wake exercises.
It checks the UART configured rate and CPU/APB readback after every
resumable wake, and its UART output exercises TX. These checks do not
establish measured sleep current or true PMU entry.

SAM4L UART wake selection now checks DeviceIntrf enable ownership
and peripheral clock enable while interrupts are masked. USART0
wake transport remains at the normal configured baud while PICOUART
receives at 9600 baud. EDBG USART1 remains the default board console;
PB00/PB01 requires an external 3.3 V adapter. The host helper runs
repeated cycles and reports timeout and failure.

SAMD11 ClockDemo uses its prior clock-only configuration by default.
The old D11 combined Standby option is removed, avoiding a redundant
profile. All five target ClockDemo board files have no wake settings.
Historical board evidence is retained in READMEs with a clear separation
from current, untested changes.

## Target review and current limits

| Target | Existing reviewed mechanism | Important remaining work |
| --- | --- | --- |
| SAM4L | PM/SCIF/BSCIF clock setters, AST/EIC, Backup/RETENTION/WAIT, driver-owned USART pause/PICOUART and TWIS PM.AWEN | Evaluate other active peripheral clocks and shared asynchronous wake sources; PICOUART wake 10/10 hardware PASS; TWIS/ADC SleepWalking, mixed peripherals and sleep current unverified |
| SAMD11 | Shared GCLK and delay update, EIC/NMI Standby, system-owned UART suspend/resume | Flash-limited PowerOffDemo needs clean link and hardware recheck; other peripherals still need review |
| SAMD21 | Shared GCLK and SERCOM ownership, EIC Standby, system-owned UART suspend/resume | Other peripherals still need sleep review; validate rebuilt PowerOffDemo |
| ESP32-C3 | SYSTEM clock tree, calibrated RC_FAST, RTC Light/Deep sleep, system-owned UART clock gates | WFI fallback cannot prove PMU entry; other peripheral lifecycles and startup dots remain unverified |
| ESP32-C6 | PCR clock tree, calibrated RC_FAST/LF, PMU PLL retention, system-owned UART gates and RTC wake | Review PMU rejection and other peripheral lifecycles; measure sleep current |

The system and UART implementations now coordinate the known UART sleep
path, without exposing a new public object or changing DeviceIntrf enable
references. PowerOffDemo has no explicit UART Disable/Enable in its
resumable sleep path.

On SAM4L, SystemSleep selects WAIT/RETENTION/SLEEP0 according to active
wake resources. The UART driver retains the PICOUART wake selection and
temporarily gates other idle USARTs only for deep resumable sleep. The
system alone arms, acknowledges and restores the shared PM wake interrupt;
UART samples the PICOUART wake byte before the system clears its flag.

On SAMD11/SAMD21, SystemSleep calls the UART's private pause/resume
functions around the existing EIC/NMI Standby sequence. The UART owns
SERCOM clock gating, pin mux, TX completion and interrupt masks.
Driver reference counts and FIFOs are preserved.

On ESP32-C3/C6, the shared SystemSleep function owns GPIO RTC wake
preparation, temporary UART clock gating and restoration. Chip-specific
Esp32PowerSleep functions control only their RTC/PMU clock, regulator and
cache transitions. Ordinary CPU WFI does not suspend the UART.

Outstanding peripheral ownership work remains for non-UART active
transports, DMA and other clock consumers. Current functions reject a
UART sleep preparation request while a transaction is still active;
SystemSleep is void, so a rejected request simply returns without entering
the intended deep mode. Applications are responsible for completing
transfers, but not for hardware-specific clock or UART restoration.

## Previous hardware evidence

- SAM4LC: RC and APB switching, delays, timers, UART RX, repeated
  EIC/AST Standby and GPIO/AST Backup wakes passed in reported runs.
  Display driver tests also passed the reported interrupt/DMA modes.
- SAMD11/SAMD21: reported Standby and sampled static/stack RAM retention
  passed. Earlier combined ClockDemo runs checked post-wake clocks,
  delays, timer IRQs and UART RX.
- ESP32-C3: the recorded final ClockDemo run passed nine Light-sleep
  wakes and ten polling UART RX prompts. PowerOffDemo recorded six
  Light-sleep wakes and four GPIO3 Deep-sleep restarts.
- ESP32-C6: recorded combined ClockDemo runs passed nine Light-sleep
  wakes, clock restoration, timer IRQs and calibrated delays. GPIO3
  Deep-sleep restart tests also passed in reported runs.

These are historical results for prior firmware revisions. The later
SAM4L PICOUART validation on November 18, 2026 completed 10 consecutive
hardware wake cycles with received byte, UART baud, CPU/APB clocks,
sequential wake counters and sampled SRAM all PASS. This validates that
specific wake path, not I2C/TWIS or ADC SleepWalking, all sleep modes or
power consumption.

## Required validation before completion

1. Clean-build the SAM4LC/LS libraries and both demos in Debug/Release
   using the Eclipse managed source/project settings. Verify optional
   UART wake on/off, EIC/AST and Backup configurations. Earlier UART
   links used cached libraries and are not clean-build evidence.
2. Clean-build SAMD11 and SAMD21 libraries/demos in both configurations,
   check D11 flash usage and the previous standalone clock profile.
3. Clean-build ESP32-C3/C6 library and demo configurations in
   Debug/Release, using their existing RISC-V managed project settings.
4. Re-run existing board demonstrations. Verify ordinary UART TX/RX,
   timers, I2C and other enabled peripherals after wake, including
   repeated wake and mixed wake ownership. Do not infer untested modes.
5. Review and implement coordinated sleep for active non-UART transports,
   DMA and any peripheral requiring a clock through WAIT/Standby, preserving
   their normal DeviceIntrf and Device lifecycle.
6. Measure sleep current and distinguish accepted PMU entry from
   a CPU WFI fallback with independent hardware evidence.

## Environment limits for this change set

The earlier /workspace/scratch working tree is absent. The connected
GitHub repository provided branch source and writable ref access, but
the local environment has no checked-out IOsonata tree or ARM/RISC-V
cross compiler. ASCII, conditional nesting and branch/diff checks
were performed on the edited source. Existing SAM4L and SAMD host test
stubs were updated for internal sleep calls. No actual compiler, library
rebuild, unit test or hardware run was performed after these edits.
GitHub reports no commit status checks for the current head.

## SAM4L master transport sleep coordination

The SAM4L system implementation now asks the I2C and SPI drivers to
prepare for deep resumable sleep alongside the existing UART driver.
Each enabled idle I2C or SPI master stores its interrupt mask, stops its
controller and gates its own bus clock without changing DeviceIntrf
reference counts, clock dividers, pins or transfer settings. The
system restores those drivers after WFI, or rolls back any already
suspended driver when another driver refuses entry.

I2C slave address matching retains its PM.AWEN and power-clock request
for TWIS SleepWalking. An active slave transaction prevents sleep.
SAM4L SPI has no supported asynchronous slave wake mechanism; an
enabled SPI slave prevents deep resumable sleep. Active master,
DMA or incomplete operations also prevent that sleep attempt.
No application-side clock switching is required for these paths.

This changes untested hardware behavior. No SAM4L SPI or I2C
post-wake hardware result, clean library rebuild or current
measurement exists yet. Other peripheral clock consumers (including
USB and ADC) have not been covered by this driver coordination.
Do not mark SAM4L power management complete until they are reviewed
and the corresponding hardware paths are validated.

## SystemSleep driver ownership update

The subsequent refactor extends driver-owned sleep coordination on all five
targets, without modifying DeviceIntrf or introducing a new public interface.

SAM4L:
- Idle USARTs, I2C masters and SPI masters release their peripheral clocks.
- DeviceIntrf enable counts and transfer settings remain unchanged.
- UART PICOUART and TWIS retain their independent asynchronous wake resources.
- Busy masters and unsupported SPI slave wake prevent deep sleep.
- SystemSleep restores the suspended drivers in reverse order after wake,
  or restores them if another driver refuses sleep preparation.

SAMD11/SAMD21:
- The system coordinates the existing SERCOM UART, I2C and SPI owners.
- Idle masters use their own Disable/Enable implementation without touching
  DeviceIntrf reference counts. Active transfers and enabled I2C/SPI slaves
  prevent Standby because no slave wake behavior is implemented here.
- EIC/NMI Standby wake resources remain managed in the pin driver.

ESP32-C3/C6:
- SystemSleep prepares RTC wake routing before requesting the transport
  drivers to pause. Idle UART, I2C and SPI controllers stop their own clocks.
- After the RTC sleep attempt, the drivers restore before RTC wake routing
  is released. Rejected sleep also restores the pre-entry state.
- Without an RTC wake source, SystemSleep leaves transport configuration
  unchanged and uses ordinary interrupt-resumable CPU WFI.
- Other Espressif ports are unchanged by the new I2C/SPI sleep functions.

The demo is not responsible for disabling, reconfiguring or restoring
these peripherals around SystemSleep. Explicit runtime clock-source
experiments in ClockDemo remain separate from low-power sleep entry.

### Validation still required

The additional transport-sleep review performed source inspection,
branch comparison and static checks, without clean builds or hardware
runs for all modified transport paths. Separately, the SAM4L PICOUART
wake path completed ten functional physical-board wake cycles on
November 18, 2026. Do not extend that result to untested peripherals.

Before treating any target as complete, perform a clean managed-project
library and application build for Debug and Release, exercise multiple
simultaneously initialized transports, repeated sleep/resume, a busy-master
refusal and its rollback, and the existing wake/retention cases. SAM4L
PICOUART wake now has 10/10 physical-board cycles PASS; TWIS
SleepWalking still needs hardware testing. SPI/I2C slave behavior
and peripheral wake modes
outside the implemented set remain unsupported or deferred.

System-wide clock and power ownership for USB, ADC and other optional
peripherals remains a separate architectural review. Sleep current
measurements and independent confirmation of actual low-power mode
entry are still pending. Do not merge this work based on source checks.

## USB clock ownership during resumable sleep

The SAM4L and SAMD USB device controllers retain ownership of their
USB clock/reference while the host session is started. Their driver
state now reports that deep resumable sleep is not supported while
the controller is started (including when the USB bus is suspended).
SystemSleep checks this before suspending other peripherals. A rejected
sleep preserves the USB session; it does not detach or change USB clocks.

Stopping the USB controller normally releases this restriction. This
does not implement USB-powered wake or a USB clock RUNSTDBY configuration.
Both require separate target-specific work and hardware validation,
including enumeration, SOF, suspend/resume and endpoint transfers.
The fallback is intentional rather than inferring that bus suspend makes
a low-frequency CPU sleep mode safe.

## Analog peripheral ownership on SAMD11/SAMD21

The SAMD DAC driver now reports whether its output is enabled.
SystemSleep rejects Standby while the DAC is actively driving an
output: this driver does not configure RUNSTDBY or an alternate
output-retention supply, so assuming uninterrupted analog output
would be incorrect. An initialized but disabled DAC does not
prevent Standby. Applications do not manage DAC clocks on behalf
of sleep; the DAC keeps its existing Enable/Disable lifecycle.

The SAMD ADC driver supports only software-triggered, blocking
single conversions on these ports. It does not currently advertise
ADC sleep-walking or asynchronous conversion wake. Its clock
requests are handled by its own Init/PowerOff implementation.
No ADC sleep-state transition has been added without target
hardware evidence. Continuous or event-triggered ADC operation
would require separate preparation and restore support.
SystemSleep on SAMD also rejects entry from an interrupt handler.

## SAMD ADC standby ownership

The existing SAMD ADC supports only software-triggered blocking reads;
there is no ADC interrupt, DMA or SleepWalking conversion path.
The ADC driver now pauses its enabled converter before Standby and
re-enables it on resume, without losing unread software sample data,
pin ownership, reference configuration or its generic clock assignment.
It preserves the previously disabled state and marks the first
conversion after re-enabling for discard via the existing Enable method.

If ADC synchronization fails during preparation, SystemSleep refuses
Standby and invokes its normal resume path; the driver does not
claim an accepted sleep mode. The SAMD Standby host entry fixture
includes the USB, DAC, ADC, I2C, SPI and UART checks, a refused-entry
matrix, rollback and an ISR call. The tests still require execution
and the full embedded libraries still require clean builds.

## SAMD DMAC active-transfer sleep check

SAMD11 and SAMD21 now ask the existing DMAC driver whether all hardware
channels and outstanding requests are idle before entering Standby.
The DMAC driver reads channel ENABLE/SWRST, the hardware busy and pending
sets, interrupt status, and active-bus flags without stopping a transfer,
freeing an allocated channel or changing its selected channel register.
If the DMAC register interface is gated while channels are allocated,
the sleep attempt is refused rather than guessing whether DMA is idle.

This check happens before ADC and transport suspension. It protects
transfers owned by drivers outside the UART/I2C/SPI set as well.
The existing SAMD host Standby fixture now covers a DMAC refusal and
requires WFI not to execute on refusal; the compile script also
includes dmac_samd.c. These checks are not target build or hardware
validation.

## SAM4L DMA and high-frequency timer sleep check

The SAM4L system now checks the shared PDCA bus gates and all 16
PDCA channel transfer-enable bits before entering deep resumable
sleep. If only one PDCA bus gate is on, it refuses sleep because the
channel registers cannot be read safely. This does not cancel a DMA
transfer or change any driver's channel ownership. An enabled bus
with no transferring channels is permitted.

The existing SAM4L Timer implementation exposes Sam4lTimerClockBusy,
which SystemSleep now reuses for TC0/TC1 when their module clock gates
are enabled. A running or unhealthy high-frequency timer prevents
deep sleep. The 32 kHz AST timer is deliberately excluded because
it is an intended wake source and its enabled state is preserved.

The SAM4L host register test now includes mismatched PDCA clock
gates, an active PDCA channel, and a running TC0 refusal, followed
by successful sleep when these resources are idle. This is source
and register-model coverage only. Target compilation, timer
continuity, DMA behavior and power consumption remain unverified.

## High-frequency timer ownership during low-power sleep

SAM4L checks the timer driver's existing active-clock state for
TC0/TC1 before deep resumable sleep and preserves AST as a wake
source. SAMD11/SAMD21 now also refuse Standby when any bound,
running TC/TCC timer exists. A bound SysTick timer is refused
conservatively because the system does not preserve its core-clock
timebase across Standby. A stopped high-frequency timer does not
prevent sleep, and the existing low-frequency RTC counter remains
outside this high-frequency check.

ESP32-C3/C6 now check running Timer Group counters before entering
RTC/PMU Light-sleep. A running counter prevents that transition;
a stopped one does not. Ordinary CPU WFI without RTC wake remains
available. Timer Enable/Disable and trigger state are unchanged by
these checks, so existing timers remain owned by their drivers.

The SAMD and ESP32 timer host tests now exercise these decisions.
The SAMD Standby entry host fixture covers timer refusal before
any peripheral is suspended. Existing RTC clock continuity and
wakeup on SAMD, and power-domain transitions on ESP32, still
require hardware verification.

## SAMD11/SAMD21 low-frequency RTC and EIC wake clock

The SAMD clock implementation now provides
SamdLowFreqClockStandbyEnable to keep the *selected* 32.768 kHz
oscillator running in Standby. It reads GCLK generator 1's source,
checks the oscillator is enabled and ready, and sets only that
oscillator's RUNSTDBY bit. A disabled, unready or unrecognized
source is refused without changing the clock tree.

SamdRtcSetup requests this before enabling the RTC generic clock.
The SAMD11 NMI EIC wake path now makes the same clock request rather
than writing OSC32K/XOSC32K directly. Both keep their own channel
ownership. The generator's RUNSTDBY bit is for driving the optional
GCLK_IO pin; with GCLK1 output disabled it runs when a peripheral
requests it (SAM D21 GCLK GENCTRL documentation). The selected
oscillator's RUNSTDBY bit, not a GCLK output setting, is required
for RTC/EIC activity in Standby.

No clock change is requested by PowerOffDemo. The source tests now
check the two wake drivers and both possible oscillators; the target
source compilation script includes system_samd.c and
timer_samd_rtc.cpp. No physical current or RTC wake continuity
measurement has been performed for this change.

## Low-frequency clock register test

Run `python3 tests/samd/run_lfclock_standby.py` to compile the
production SamdLowFreqClockStandbyEnable function in a host register
model at -O0 and -O2. The test checks OSC32K and XOSC32K selection,
readiness and output-enable requirements, unchanged oscillator
configuration, interrupts restored to their previous state, and
refusal of an unready or unknown source.

This host model does not test the synchronization timing of GCLK
or the physical RTC wake circuit. Hardware validation remains required.

## SAM4L PICOUART functional hardware validation (November 18, 2026)

The maintainer's SAM4L8 Xplained Pro test established 10/10
consecutive PICOUART UART wake cycles at 9600 baud with the
restored 1 Mbaud USART0, CPU/APB clocks, sequential wake counters
and 128-byte static plus 128-byte stack SRAM samples all PASS.
The Python host regression suite passed 5/5 tests.

A subsequent `--wake-delay 20 --cycles 1` run also passed
PICOUART byte reception, UART/clock restoration and sampled SRAM.
The host waits 20 seconds after changing baud before sending
the wake byte; the MCU's 750 ms pre-sleep test guard is included
in that interval.

This validates the functional UART-wake path with both repeated
and delayed wake requests. Actual low-power current could not be
measured because no current-measurement equipment was available.
Mixed-peripheral sleep handling, clean managed-project build coverage,
and independent low-power mode/current verification remain separate
validation items. The host's baud switch/handshake are test-only;
normal applications do not coordinate `SystemSleep()` with a host.
