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
| SAM4L | PM/SCIF/BSCIF clock setters, AST/EIC, Backup/RETENTION/WAIT, UART PICOUART hooks and TWIS PM.AWEN | Ordinary USART/peripheral suspend not coordinated automatically; assess shared PM wake ownership and multiple wake sources; hardware UART, TWIS and ADC SleepWalking are unverified |
| SAMD11 | Shared SAMD GCLK clock setters and delay update, EIC/NMI Standby arm/restore | UART/peripheral suspend still performed by the demo; flash-limited ClockDemo requires a clean link; no full sleep-resource review |
| SAMD21 | Shared SAMD GCLK and SERCOM ownership, EIC Standby arm/restore | Driver sleep participation beyond EIC is not integrated; validate new PowerOffDemo clock check |
| ESP32-C3 | SYSTEM clock tree, calibrated RC_FAST, RTC Light/Deep sleep, driver UART clock gates | PowerOffDemo still disables UART in application code; WFI fallback cannot prove PMU entry; investigate previously reported startup dots |
| ESP32-C6 | PCR clock tree, calibrated RC_FAST and LF, PMU PLL retention, driver UART clocks and RTC wake | PowerOffDemo still disables UART in application code; review PMU request rejection and WFI fallback semantics; verify external sleep current |

The same generic PowerOffDemo source still calls Disable/Enable around normal
resumable sleep when UART wake is not selected. This is an unresolved
ownership bypass, not an implementation of the desired architecture.
It was preserved for the previously hardware-tested SAMD and ESP32 paths
rather than changing their power sequencing without the board/toolchain.
The next architectural change must move safe peripheral suspend/resume
to the existing target system and driver implementations; do not
introduce another public manager merely to avoid using existing code.

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

These are historical results for prior firmware revisions. They do not
verify the new demo split. No hardware validation for SAM4L UART wake,
I2C SleepWalking, ADC SleepWalking, all sleep modes or consumption exists
in this review.

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
5. Only then refactor the remaining application-side Disable/Enable
   sleep preparation into the target system/driver lifecycle. Validate
   it before removing that behavior from the demos.
6. Measure sleep current and distinguish accepted PMU entry from
   a CPU WFI fallback with independent hardware evidence.

## Environment limits for this change set

The earlier /workspace/scratch working tree is absent. The connected
GitHub repository provided branch source and writable ref access, but
the local environment has no checked-out IOsonata tree or ARM/RISC-V
cross compiler. ASCII, conditional nesting and branch/diff checks
were performed on the edited source. No actual compiler, library
rebuild, unit test or hardware run was performed after these edits.
GitHub reports no commit status checks for the current head.
