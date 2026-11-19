# Nordic power and clock review, IOsonata 0.14

Scope: nRF52, nRF5340 application/network cores, nRF54L15, nRF54LM20,
nRF91, and the initial nRF7120 port. nRF54H is excluded from this work.

## Generic architecture

`SystemSleep()` is an interrupt-resumable System ON CPU idle operation.
The MCU remains configured, and clocks and peripherals remain under their
owners. It masks interrupts around WFI to avoid consuming a pending wake
before entry, then restores PRIMASK and the original SCB SCR. It does not
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
