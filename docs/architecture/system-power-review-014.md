# System power review (0.14)

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
| SAM D11/D21 (PR #140) | **Not implemented**; only reset-cause getter added | Missing actual power-saving feature. PM RCAUSE mapping is valid but is not power management; duplicate ResetReasonDemo should not be the primary example | Reset-model 128/128 for each part, Cortex-M0+ source compilation PASS; power wake not tested |
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
6. **Source style**: Several existing drivers declare variables midway
   through functions or inside loops, against the project convention that
   declarations are placed at function entry. Clean these incrementally,
   without mixing cosmetic changes into functional power-mode commits.

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
`ARM/Microchip/SAMD/src/system_sleep_samd.cpp` and linked into both
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
