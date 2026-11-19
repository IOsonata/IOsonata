# Nordic power and clock review, IOsonata 0.14

Scope: nRF52, nRF5340 application/network, nRF54L15, nRF54LM20 and nRF91.
The initial nRF7120 work is preserved on `wip_nrf71_power_clock_014` and
excluded from this review because its clock backend requires a separate
register-compatible implementation. nRF54H is excluded.

## Architecture

The same `exemples/misc/power_off_demo.cpp` runs on every MCU.
Board headers select the console and wake source; system power/clock code
and peripheral drivers implement MCU-specific behavior.

`SystemSleep()` provides interrupt-resumable System ON idle. It does not
enter System OFF or take ownership of running peripherals. Across nRF52,
nRF53, nRF54L and nRF91, `SystemWakePin()` configures GPIO SENSE for
System OFF; `SystemSleep()` temporarily arms a GPIOTE interrupt for
System ON GPIO wake. An enabled nRF52 SoftDevice uses
`sd_app_evt_wait()` only when the caller permits normal IRQ delivery;
if PRIMASK was set by the caller, the SoftDevice path refuses sleep rather
than enabling interrupts behind the caller. Otherwise the CPU uses WFI.
SCR and interrupt-mask state are restored. GPIO wake on nRF53/nRF54L/nRF91 still requires a target
build and hardware test.

System ON sleep differs from Microchip's deeper WAIT/Standby modes:
Nordic peripherals and radio that remain enabled keep their clocks and
may continue transfers or raise interrupts. Automatically disabling UART,
I2C, SPI or USB here would break receive, slave and radio wake semantics;
their device drivers continue to own their enabled state. The HAL must
not silently call peripheral Disable/Enable (which may flush FIFOs).
Deep peripheral suspension is not claimed by this System ON API.

`SystemPowerOff()` enters reset-exit System OFF. The application or
peripheral owner must finish incompatible UART, DMA, USB, radio and modem
work before shutdown. The generic PowerOffDemo drains the UART when the
board selects `POWER_DEMO_UART_OFF`.

`SystemCoreClockSelect()` requests/releases the HFXO. The first owner
(direct hardware, SoftDevice or MPSL) remains responsible until final
release; requests are reference-counted. Blocking first-start and
last-release transitions are refused in interrupt context.
`SystemLowFreqClockSelect()` refuses an interrupt-context source switch
or a switch while SoftDevice/MPSL owns LFCLK.

## Implementation

| Family | Resumable sleep | System OFF |
| --- | --- | --- |
| nRF52 | `system_power_nrf52.cpp`, GPIOTE / SoftDevice-aware | GPIO SENSE, reset on wake |
| nRF5340 application | Shared Nordic WFI backend | Application-core regulators |
| nRF5340 network | Shared Nordic WFI backend | Controlled by application core |
| nRF54L15 / LM20 | Shared Nordic WFI backend | Regulator and clock ownership checks |
| nRF91 | Shared Nordic WFI backend | Modem must be shut down first |

nRF53, nRF54L and nRF91 target hardware sleep, wake and retention
behavior still requires configuration-specific validation.

## Verified nRF52 results (November 18, 2026)

- HFXO bare-metal request/release: 5/5 checks PASS.
- HfxoTestBle: 5/5 hardware checks PASS for the reported configuration.
  The report did not identify the SoftDevice versus SDC build.
- ClockDemo: three clock selection cycles PASS; delay requests of
  250 ms measured 250153 us (msDelay) and 250154 us (nsDelay), with
  25 timer interrupts each; zero failures.
- Generic PowerOffDemo System OFF: four GPIO wake/reset cycles PASS,
  wake reason 0x100, retained start count 1 through 5, complete UART
  output after enabling drain/power-off in both nRF52 board profiles.
- Generic PowerOffDemo System ON with button: four resumable GPIO
  wake cycles PASS, UART/CPU/APB continuity and static/stack SRAM sample.
- Generic PowerOffDemo System ON with RTC timer: six automatic wake
  cycles PASS, with UART/CPU/APB continuity, timer timing check and
  128-byte static plus 128-byte stack SRAM samples.
- `tests/nordic/check_power_source.py`: 9/9 source checks PASS
  before the nRF71 test separation.
- `tests/nordic/run_system_sleep.py`: nRF52, nRF52 SoftDevice, shared
  Nordic and (before separation) nRF71 host models PASS.
- Focused source compilation of nRF52832/nRF52840 power/clock code
  in -O0 and -Os: 8/8 PASS. This is not a complete managed-project link.

The generic PowerOffDemo timer fix starts its one-shot timer with
`TimerEnable()` after arming the trigger and before `SystemSleep()`.
It is common to every MCU and was validated by the nRF52 RTC wake run.

## Peripheral ownership review

- UART/UARTE: the existing `Enable()` path resets counters and flushes the TX
  FIFO, so using `Disable()/Enable()` as a transparent sleep suspend would
  lose application state. In System ON, UART remains owned and can continue
  receiving if an appropriate interrupt is enabled. UART RX as a *new*
  low-power wake source is not claimed; `UARTEnableWake` must report false
  where no corresponding wake backend exists.
- TWI/TWIM/TWIS: `Disable()` now selects the actual configured transport
  (legacy TWI, TWIM master or TWIS slave), matching `Enable()`. A slave
  cannot be disabled transparently while expected to detect an address.
  System ON leaves I2C active; deeper sleep requires a separate driver
  prepare/resume design and hardware verification.
- SPI/SPIM/SPIS: an active master or a slave awaiting selection retains
  driver ownership in System ON. Existing Enable/Disable cannot be used
  blindly for low-power suspend without transfer-state and PEI checks.
- USB and radio: active sessions retain their peripheral/clock owners in
  System ON. A radio stack controls its own timing; the nRF52 SoftDevice
  uses `sd_app_evt_wait()` for application wake filtering.

This behavior is compatible with the generic application API but is **not**
equivalent to SAM4L WAIT/RETENTION or SAMD Standby power-state selection.
Neither low-power current nor seamless suspension of these active Nordic
transports has been validated. Do not report those modes as completed.

## Managed Eclipse build validation

Use the checked-in CDT projects and their existing Debug/Release
configurations; do not create a separate Make/CMake project. The dedicated
runner uses an isolated workspace per target, imports that target's library,
ClockDemo and PowerOffDemo projects, and performs sequential clean builds.
Projects not provided for a target are marked SKIPPED, not PASS.

```sh
python3 tests/nordic/build_managed_power_clock.py --check-only \
    --target nrf52832 --target nrf52840 --target nrf5340_app \
    --target nrf5340_net --target nrf54l15 --target nrf54lm20 \
    --target nrf9160 --target nrf91x1

python3 tests/nordic/build_managed_power_clock.py \
    --target nrf52832 --target nrf52840

# Optionally use the exact IOcomposer/Eclipse launcher:
# --ide /Applications/IOcomposer.app/Contents/MacOS/iocomposerc
```

The runner now checks the Eclipse output for compiler/linker, Make and
headless-build error diagnostics in addition to the process exit status.
It does not count a successful launcher exit as a successful build if
the log contains an error. Verify the classifier independently with:

```sh
python3 tests/nordic/test_build_managed_power_clock.py
```

The build uses the installed Eclipse CDT managed-build extension and
the project's existing external toolchain and libraries. Logs are written
to `tests/nordic/build_logs`. A linked TaktOS archive or SDK not
installed at the paths expected by the project may cause a real link
failure; the runner must not classify missing external dependencies as
a PASS. The builder does not program devices or measure power current.

This managed-project runner has not been executed on a macOS Eclipse
installation as part of the repository review.

## Outstanding verification

- Rerun the source/model regression tests after the nRF71 separation.
- Recheck full Eclipse managed-project Debug/Release links for the
  changed configurations; focused source compile is not a full link.
- Verify SoftDevice and SDC/MPSL enable/handover behavior with HFXO
  requests held across stack initialization.
- Validate TrustZone, modem ownership, MEMCONF retention and
  application wake on nRF53, nRF54L and nRF91 hardware.
- UART RX as a wake source and physical sleep-current measurements
  are not covered by the reported nRF52 tests.

## Regression commands

```sh
python3 tests/nordic/validate_power_clock.py
python3 tests/nordic/validate_power_clock.py \
    --compiler /opt/xPacks/xpack-arm-none-eabi-15.2.1/bin/arm-none-eabi-g++ \
    --target nrf52832 --target nrf52840
# Add --target nrf5340_app / nrf5340_net / nrf54l15 / nrf54lm20a /
# nrf54lm20b / nrf9160 / nrf9120 for other installed targets.
```

Host/source checks and production-source syntax compilation are not
substitutes for target linking and hardware validation.
