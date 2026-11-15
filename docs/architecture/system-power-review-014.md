# System power review (0.14)

Current hardware result, November 15, 2026: SAMD11 Xplained Pro SW0/PA14
woke PowerOffDemo five consecutive times, counts 0x00000001 through
0x00000005, followed by another Standby entry. Repeated resume and counter
continuity passed. SAMD21 wake, full SRAM testing and sleep-current
measurements remain pending. The sections below retain the earlier review
sequence; their pending SAMD11 wake statements are superseded by this result.

Review basis: `main` immediately after STM32WBA65 PR #139 squash merge,
plus unmerged SAM D11/D21 PR #140. This document is an implementation
audit, **not** a new power API design or an assertion that untested code works.

## Design issue: power management versus reset-on-wake

`include/coredev/system_core_clock.h` currently specifies that
`SystemPowerOff()` enters a mode exited **only by reset** and never
returns. `docs/architecture/core-power-clock-ports.md` repeats that
restriction. The shared `exemples/misc/power_off_demo.cpp` assumes a new
startup after wake, uses `SystemResetReason()` to detect wake, and tests
RAM retention only after a reset.

Those assumptions describe Nordic System OFF, STM32 Standby/Shutdown and
Microchip SAM4 Backup correctly for the current implementations. They
are not a general description of the available lowest-power states of
every MCU. SAM D11/D21 Standby uses SLEEPDEEP + WFI, keeps RAM in low
power and wakes via asynchronous interrupt, normally resuming execution.
The current contract prevents exposing that capability.

**Do not silently change the header or make existing `SystemPowerOff()`
implementations return.** Decide and document the power operation
semantics, wake-resume behavior, ISR ownership and compatibility first.
Reset reason is an optional diagnostic, not the objective of saving power.

## Implementation review

| Family | Existing power state | Source assessment | Validation status |
| --- | --- | --- | --- |
| nRF52 | System OFF, SoftDevice ownership where enabled | Correct architectural mode; source handles SENSE, reset reasons and RAM retention; check failed SoftDevice requests and partial retention updates | Existing hardware validation; not retested in this audit |
| nRF53/nRF54L/nRF91 | System OFF via REGULATORS where supported | Correct mode; network core deliberately cannot independently power off; modem/radio/clock ownership needs per-core verification | nRF54L/LM20 and nRF5340 application hardware results recorded in architecture notes; combinations not universally tested |
| STM32 F0/F4/L1/L4/G4/H7 | Standby or Shutdown | Correct reset-exit mode for these implementations; watch power-domain/RTC flag ownership, package-specific WKUP pins and active peripheral clock/transfer shutdown | Existing host/target checks; hardware coverage varies by part |
| STM32WBA65I | Standby with optional full-bank SRAM retention | PC13 WKUP2, SRAM2 retention and SRAM1 non-retention passed; disabling USART1/DMA before entry was necessary | Final cleaned-up firmware: two successive hardware wake cycles PASS |
| SAM4E | Backup (supply off to SRAM) | Correct reset-exit mode; RTC/RTT wake paths and application peripheral shutdown still need hardware evidence | Host tests, hardware not confirmed |
| SAM4L | Backup (SRAM off) | Correct reset-exit mode; alternate Retention sleep is not yet exposed by generic API | Host tests, hardware coverage incomplete |
| SAM D11/D21 (PR #141 draft) | SystemSleep entry/resume, asynchronous EIC wake and PM RCAUSE reset diagnostics implemented on review branch | Standby source compiled on both targets; full link and wake validation pending; reset-only demo intentionally omitted | Reset-model 128/128 each, Arm source compilation PASS; power wake not tested |
| LPC/RA/RE/ESP32 families | No generic port in main for most targets | Not reviewed as implemented generic power backends; should be scoped to available hardware | Pending |

## Specific implementation and test risks

1. **API compatibility**: An interrupt-resuming `SystemPowerOff()` would
   violate the header's current non-return guarantee and the generic demo's
   post-reset assertions. Updating this needs an explicit design decision,
   not a per-MCU workaround.
2. **Active peripherals**: WBA65 did not wake when USART1/DMA remained
   active. `SystemPowerOff()` does not centrally coordinate driver shutdown.
   Define clearly whether callers must stop UART/DMA/radio/USB, or whether
   supported drivers provide an orderly system-power transition.
3. **SAMD functionality**: PR #140 currently implements only
   `SystemResetReason()` and adds separate reset demonstration projects;
   they do not validate Standby current, wakeup or SRAM retention.
4. **Generic demo**: The current startup counter, RAM comparisons and
   wake-reason logic assume a reset after wake. The same example needs a
   resume path for asynchronous-interrupt wake without discarding the
   already validated reset path.
5. **Claimed support**: A compiled power source does not imply that every
   MCU of that series or its radio/USB configurations were hardware tested.
   Keep validation tied to exact targets and modes.
6. **Source style**: File-scope types, globals, constants and prototypes
   belong before function implementations. Local variables may remain in
   their relevant context. Apply the file-scope rule to earlier work too.

## Compatibility-preserving API direction

Introduce `SystemSleep(void)` as an optional **interrupt-resumable**
low-power operation. The MCU implementation should enter its deepest
supported sleep mode that resumes execution, after callers have configured
the wake sources and stopped incompatible peripherals. It returns normally
after wake and preserves the stack and SRAM. The state does **not** imply
a reset or a new startup. `SystemPowerOff(void)` retains its existing
reset-exit semantics for all current validated backends. No existing
backend will be modified simply to satisfy the new declaration.

The `SystemSleep()` entry has now been implemented for SAM D11/D21 in
`ARM/Microchip/SAMD/src/system_sleep_samd.c` and linked into both
Eclipse-managed MCU libraries. It sets SLEEPDEEP, executes WFI and restores
the original SCR after wake. **This is entry/resume logic only**: a working
asynchronous wake source, peripheral shutdown and target compilation/hardware
validation still need to be demonstrated. Do not call the port complete yet.

The SAMD EIC code currently configures edge detection with a filter and
uses generic clock generator 0 (main), while Standby wake requires a
suitable asynchronous EIC wake source and `EIC_WAKEUP` enable. Before
using `SystemSleep` in the shared demo, review EIC ownership, wake bits,
filter availability and low-frequency generator selection. Changing every
EIC interrupt to a standby configuration would be a regression risk.

## SAM D11/D21 first Standby integration (unvalidated)

The review branch now has a concrete interrupt-resumable implementation:

- `SystemSleep()` enters Standby with SLEEPDEEP/WFI, restores the SCR state
  on return, and rearms explicitly configured EIC wake interrupt lines.
- SAMD-specific `SystemWakePin()` uses an EIC EXTINT line owned by the
  power wake path. It selects asynchronous **low/high level detection with
  filter disabled**, sets `EIC_WAKEUP`, and arms the interrupt. The wake
  callback disables further level-triggered IRQ delivery until rearmed on
  another sleep cycle.
- Existing GPIO interrupts using edge detection/filter/GCLK0 are unchanged
  and cannot be silently taken over by a Standby wake pin. An EIC event
  source already owned by another driver is refused.
- The shared `exemples/misc/power_off_demo.cpp` now has a
  `POWER_DEMO_RESUME_WAKE=1` path: stop the UART before Standby, return
  after an EIC wake, restart UART and log a wake counter. Existing
  reset-on-wake demo logic is unchanged.
- Eclipse-managed PowerOffDemo projects are provided for SAM D11 and D21.
  SAMD21 uses SW0/PA15 (EXTINT15). SAMD11 SW0/PA14 is routed to the dedicated NMI wake path, so no
  external switch is needed.
- Build checks are in `tests/samd/standby_compile.sh` (invoke using `sh`
  if the script is not executable in the checkout).

This code has **not been compiled or exercised on target hardware**.
Validate clock availability, interrupt delivery, actual Standby entry,
wake latency, RAM continuity and repeated cycles before claiming PASS.
Microchip SAM D21 datasheet DS40001882G §21.6.3 specifies asynchronous
detection for unfiltered level sensing without `GCLK_EIC`.

## Branch consolidation

The useful work from `port_samd_power_014` (former PR #140) has been
incorporated into this review branch: production `SystemResetReason()`,
the default `McuOsc_t.DcDc` initializer fix, reset model test runner,
the Arm GNU compile harness and the Eclipse source links for D11/D21.
Both libraries retain the Standby source and the reset-reason source.

The separate `ResetReasonDemo` projects and
`exemples/misc/reset_reason_demo.cpp` were deliberately not copied.
The single shared `PowerOffDemo` serves as the low-power validation
application. This branch has not been hardware-validated or merged.

## Pre-hardware review after consolidation

Three source-level issues were addressed on the review branch:

1. `IOPinEnableInterrupt()` and `IOPinSetSense()` now refuse to take or
   silently reconfigure an EIC line owned by Standby wake. Disabling that
   GPIO interrupt releases its `EIC_WAKEUP` setting.
2. Clearing the Standby wake bit is performed while the EIC is disabled
   and synchronized, rather than writing the configuration while enabled.
3. `SystemSleep()` masks interrupts while rearming the wake source and
   executing WFI, restoring the previous PRIMASK afterward. This keeps
   an early wake interrupt from being handled before WFI and lost.

Source checks are available in `python3 tests/samd/run_standby_source.py`.
These checks and the new code have **not** yet been run on either target
after the changes; physical wake and measured sleep current remain pending.

## Suggested correction order

1. Define and approve the generic API's sleep/deep-sleep/off semantics and
   wake-return behavior. Preserve existing applications where possible.
2. Change the shared PowerOffDemo to exercise reset-on-wake and
   resume-on-interrupt without two unrelated applications.
3. On `port_samd_power_014`, remove the duplicate ResetReasonDemo
   application projects, implement SAM D11/D21 low-power Standby including
   EIC/RTC wake as appropriate, and test SRAM continuity after wake.
4. Compile the two Cortex-M0+ libraries/examples and run the SAM D11/D21
   hardware wake tests. Keep PR #140 draft until the power path is validated.
5. Audit and regress the existing Nordic, STM32 and SAM4 power paths for
   driver/clock ownership, before merging any cross-family API change.

References: `include/coredev/system_core_clock.h`,
`exemples/misc/power_off_demo.cpp`,
`ARM/Nordic/nRF52/src/system_power_nrf52.cpp`,
`ARM/Nordic/src/system_power_nrfx.cpp`,
`ARM/ST/src/system_power_stm32.cpp`,
`ARM/ST/STM32WBAxx/src/system_power_stm32wba.cpp`,
`ARM/Microchip/SAM4E/src/system_power_sam4e.cpp`,
`ARM/Microchip/SAM4L/src/system_power_sam4l.cpp`,
and unmerged `ARM/Microchip/SAMD/src/system_power_samd.cpp`.

Microchip SAM D21/DA1 datasheet DS40001882G §16.6.2.8.2 identifies
Standby entry through SLEEPDEEP+WFI, SRAM preservation and asynchronous
interrupt wake. SAM D11 datasheet 42363 describes equivalent wake behavior.

## SAMD11 onboard SW0 wake (pending hardware validation)

SAMD11's onboard SW0 is PA14, the dedicated EIC NMI input. The Standby
wake port now handles this pin separately from EXTINT; it configures the
NMICTRL falling-edge detector, clears NMIFLAG in the NMI handler, and
selects the low-frequency EIC generic clock. The generic PowerOffDemo
uses PA14 without external jumpers. SAMD21 retains PA15/EXTINT15 wake.

The NMI vector is shared with other application-level critical NMI uses;
using this pin as a wake source reserves that handler while the power
wake feature is enabled. The new NMI path is not hardware-validated yet.

