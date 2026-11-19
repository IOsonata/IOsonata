# Nordic power and clock review, IOsonata 0.14

Scope: nRF52, nRF5340 application/network cores, nRF54L15, nRF54LM20,
nRF91, and the initial nRF7120 port. nRF54H is excluded from this work.

## Generic architecture

`SystemSleep()` is an interrupt-resumable System ON CPU idle operation.
The MCU remains configured, and clocks and peripherals remain under their
owners. The nRF52 SoftDevice-enabled path calls sd_app_evt_wait(), so only
application-relevant events cause a normal application wake. In bare-metal,
SDC/MPSL and other Nordic configurations it masks interrupts around WFI to
avoid consuming a pending wake before entry, then restores PRIMASK and the
original SCB SCR. It does not
request System OFF or shut down the radio or USB.

`SystemPowerOff()` means reset-exit System OFF. The application stops
incompatible UART, DMA, USB, radio, and modem activity before requesting it.
The target power code does not take ownership of unrelated drivers.

`SystemCoreClockSelect()` on established Nordic targets requests and
releases HFXO. Requests are reference-counted and the first acquisition
retains its original direct, SoftDevice or MPSL owner until final release.
A long hardware transition is refused from interrupt context.

`SystemLowFreqClockSelect()` is refused while SoftDevice/MPSL owns LFCLK,
and while executing in an interrupt handler. Switching sources requires
that low-frequency peripheral users are stopped.

## Implementation matrix

| Port | Resumable System ON sleep | Reset-exit System OFF | Clock integration |
| --- | --- | --- | --- |
| nRF52 | `system_power_nrf52.cpp` | Existing SoftDevice-aware implementation | Shared clock backend |
| nRF5340 application | Shared Nordic power backend | Secure regulator, subject to radio ownership | Shared clock backend |
| nRF5340 network | Shared Nordic power backend | Controlled by application core, not this core | Shared clock backend |
| nRF54L15 / LM20 | Shared Nordic power backend | Existing implementation checks HFXO/radio ownership first | Shared clock backend and startup PLL |
| nRF91 | Shared Nordic power backend | Requires modem shutdown first | Shared clock backend |
| nRF7120 | Dedicated `system_sleep_nrf71.c` | Not implemented in the initial port | Startup PLL/OSC and shared clock backend; full clock-tree audit pending |

nRF71 is not the same clock or secure-power configuration as nRF54L.
Its initial port is not declared fully power-ready merely because it compiles.
Non-secure System OFF may require the secure service, and the retainable RAM
regions and RF clock ownership must be verified with nRF71-specific metadata
before adding a System OFF implementation.

## Validation

- Existing Nordic HFXO source checks: `python3 tests/nordic/check_clock_hfxo_source.py`.
- New host WFI/interrupt-state model: `python3 tests/nordic/run_system_sleep.py`.
- Verify Debug/Release library and example linking for each target separately.
- Run PowerOffDemo only on boards available for hardware testing, and record
  the exact wake source, reset reason, SRAM retention and clocks afterward.
- Never confuse the host WFI model with hardware sleep-current measurement.

The new SystemSleep and LFCLK ISR changes have not yet been tested with a
complete Arm target build. Do not merge until target compatibility is checked.

## Production code review findings (November 18, 2026)

The following review covers actual existing target backends, not only newly
introduced API functions. Source inspection and managed-project links were
checked for nRF52832/nRF52840, nRF5340 application/network, nRF54L15,
nRF54LM20, nRF9160/nRF91x1 and nRF7120.

### Corrected on the review branch

1. The nRF52 SoftDevice-enabled SystemSleep path now uses sd_app_evt_wait().
   A raw WFI bypassed application-event filtering and could return to
   application code on radio-only activity. Non-SoftDevice configurations
   retain WFI, preserving clocks, memory, PRIMASK and SCR.
2. nRFGpioGetReg previously rejected an encoded domain bit (0x80) before
   checking it, and selected the register using the unmasked value. It now
   selects the physical port from the low seven bits while retaining the
   domain selector. SystemWakePin validation uses the physical port number.
   This does not configure SPU security; a non-secure caller can only
   configure pins already assigned to its security domain.
3. First HFXO start/final stop and LFCLK source transitions refuse to wait
   for oscillator hardware from interrupt context. Nested HFXO requests
   remain reference-counted; the owner of the first request performs the
   final release.
4. The initial nRF71 library now links a dedicated interrupt-resumable
   SystemSleep source; this is not an nRF71 System OFF implementation.
5. The earlier branch changes enforce per-port wake-pin limits and initialize
   the nRF71 CPU delay factors when the configured PLL rate changes.

### Existing implementation inspected

- nRF52 System OFF uses sd_power_system_off when SoftDevice is enabled,
  otherwise direct POWER register access; reset reason and 8-bit GPREGRET
  accesses also use the SoftDevice path where applicable.
- The nRF53/nRF91 backend uses secure regulators/VMC where available.
  The nRF5340 network core cannot independently request the application
  core's System OFF. nRF91 modem activity must be stopped by its owner.
- nRF54L System OFF refuses entry if outstanding HFXO requests, clock
  transitions or active MPSL ownership remain. MEMCONF section retention
  is applied at System OFF and separate VPR context is not retained.
- GPIO wake is a SENSE/DETECT level mechanism rather than an edge-only IRQ;
  wake pins must be initialized and released to inactive levels before OFF.
- All established Nordic managed library projects inspected link their
  appropriate existing power and shared clock source. nRF71 has no
  secure power backend linked in its initial project.

### Still requiring explicit verification

- Debug/Release compile and link of all target variants, including
  nRF52 bare-metal/SoftDevice/SDC and nRF53 secure/non-secure configurations.
- Full SoftDevice and SDC/MPSL enable/handover tests with an HFXO request
  already held before stack initialization, and release after initialization.
  The shared clock source intentionally refuses unsafe direct-owner release
  after radio takeover. The stack initializer still needs to coordinate
  that transition; the generic API cannot silently migrate ownership.
- Verify nRF53/nRF91 TrustZone access and VMC memory retention on the actual
  selected build/security configuration; a secure register alias is not a
  drop-in replacement for a non-secure secure-service request.
- Validate nRF54L MEMCONF retention bank behavior and no outstanding
  radio/USB request before OFF, on each relevant silicon variant.
- Validate actual nRF71 CLK/HFXO/LFCLK register compatibility and its
  secure power-service architecture before adding System OFF or RAM
  retention. Do not assume nRF54L's clock tree is identical.
- Measure real sleep currents and wake timing; the host WFI model tests
  instruction/control-state transitions, not physical power consumption.

### Regression commands

```sh
python3 tests/nordic/check_clock_hfxo_source.py
python3 tests/nordic/check_power_source.py
python3 tests/nordic/run_system_sleep.py
```

These tests are source/model checks only. A clean managed Debug/Release
target build and physical tests of the changed paths are separate gates.
