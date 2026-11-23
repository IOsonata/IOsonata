# Nordic clock ownership and driver-access review (0.14)

Working branch: `refactor_nordic_clock_ownership_014`
Reference baseline: release `0.13` at `e8a6de5f07a9`.
Merged baseline: `main` at `0c5f6a031a78`.

This review distinguishes defects already in 0.13 from issues introduced
later. Source inspection does not establish that a firmware was built or
executed. Keep this work separate from unmerged PR #144.

## Ownership rules

- `SystemInit()` establishes the initial oscillator, regulator and clock tree.
- System Clock controls shared oscillator selection and request lifetimes.
- Timer, USB and other peripherals own their registers and request shared
  clocks through the system implementation.
- System Power decides the sleep state and coordinates existing peripheral
  drivers. Applications do not switch or restore MCU clocks for sleep.
- The application does not inspect another driver's FIFO or call UART RX to
  advance UART TX.
- GPIO and PEI retain ownership of pin interrupts and event routing.

## Corrected on the working branch

| Source | Correction |
| --- | --- |
| `ARM/Nordic/src/timer_lf_nrfx.cpp` | RTC no longer selects, starts or stops shared LFCLK directly |
| `ARM/Nordic/src/system_clock_nrfx.cpp` | Counts RTC LFCLK requests and refuses incompatible active source choices |
| `ARM/Nordic/src/timer_hf_nrfx.cpp` | No timer-side PLL start; retains HFXO request if release is refused |
| `ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp` | Retains HFXO request state when the clock manager refuses release |
| `exemples/misc/core_clock_test.cpp` | No UART FIFO reads or RX-for-TX servicing |
| `exemples/misc/power_off_demo.cpp` | No UART FIFO reads, RX-for-TX servicing or SAM4L register diagnostics |
| `exemples/i2c/i2c_slave_demo.cpp` | Application no longer disables UART around SystemSleep or reads its private TX status |
| `exemples/i2c/sleepwalking.md` | Documents system-managed UART preparation |
| `AGENTS.md` | Corrects function-entry declaration and statement-format guidance |

The nRF52 delay correction was derived from measured board timing.
The nRF52832 ClockDemo result with `13/6` measured 250123 us for
msDelay(250) and 125065 us for nsDelay(250 ms). A subsequent `13/3`
run measured 250123 us and 260123 us, respectively. The nanosecond delay
remains outside the 2-percent check, and that result predates the polling
UART PowerOff fix. Do not interpret it as a passing calibration.

## Further inspection required

| Source | Observation | Baseline |
| --- | --- | --- |
| `exemples/bluetooth/uart_ble_central.cpp` | Statistics read `UARTDev_t::hTxFifo` and `TxDropCnt` directly; preserve the diagnostic functionality when correcting it | Already in 0.13 |
| `ARM/Nordic/nRF52/nRF52832/exemples/SlimeVRTracker/src/main.cpp` | Application starts HFCLK through CLOCK registers rather than System Clock; review ESB ownership first | Already in 0.13 |
| `ARM/Nordic/nRF54/nRF54LM20A/exemples/TinyUsbComboStress/src/tinyusb_combo_port.cpp` (same file in nRF54LM20B) | Bring-up code starts XO24M directly; decide whether the USB port or system clock implementation should own this resource | Already in 0.13 |
| `ARM/Nordic/src/system_power_nrfx.cpp` | SystemPowerOff can return on failed HFXO precondition, despite the generic nonreturning description | Introduced after 0.13 |
| `ARM/ST/STM32WBAxx/src/system_power_stm32wba.cpp` | SystemPowerOff can return when a power register check fails | Introduced after 0.13 |
| `ARM/Nordic/src/system_clock_nrfx.cpp` | BLE ownership handover and final release in interrupt context require separate target validation | Introduced after 0.13 |

`ClockDemoFlush()` and `PowerDemoDrain()` allow transmission time
without reading private UART state. Their fixed delay is not an exact
hardware-TX-complete operation and must not be represented as one.
The nRF52 legacy polling UART drains its private software queue and
waits for the last TX completion inside `nRFUARTPowerOff()`. The driver
owns the operation; `ClockDemo` does not poll RX to advance TX.
The generic ClockDemo no longer powers the UART off between clock
operations. Shared oscillator ownership has separate HFXO/SDC tests,
while ClockDemo tests core and peripheral clock APIs and timer behavior.
UART completion handling still requires target-specific validation.
The existing generic UART interface does not supply a portable on-wire
drain operation. Do not change the generic interface without a separate
cross-MCU review.

The inspection covered 18 changed shared example sources and 15 PEI
example sources for these direct-access patterns, plus the named system
and peripheral implementations. This is not a line-by-line audit of the
entire repository.

## Checks and hardware work

Source checks:
- `python3 tests/nordic/check_example_driver_ownership.py`
- `python3 tests/nordic/check_clock_peripheral_ownership.py`
- `python3 tests/nordic/check_delay_calibration.py`

The scripts are source checks; successful results do not replace a build
or board measurement. Their presence is not evidence they were executed.

Required target validation:
1. Clean-build the nRF52832 library and ClockDemo Release with matching
   managed Eclipse configurations. Recheck HFXO, LF RC and the two 250 ms
   delays: both must measure 245000 through 255000 us.
2. Run TimerDemo on RTC0 and RTC1; verify interrupts, timer continuity,
   and repeated Disable/Enable cycles.
3. Exercise two RTC consumers together: disabling the first must not stop
   the second; an incompatible LFCLK source request must be rejected.
4. Rebuild nRF52840 and the other affected Nordic MCU configurations,
   including SoftDevice and MPSL variants. Validate their ownership
   handover rather than inferring it from the nRF52832 bare-metal test.
5. Rebuild the generic PowerOffDemo and SAM4L I2C sleep-test profile
   because their output and sleep preparation paths changed.

No merge is authorized by these checks alone.
