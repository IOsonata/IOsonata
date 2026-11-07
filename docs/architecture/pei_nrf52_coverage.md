# nRF52 PEI coverage and acceptance tracker (0.14)

PEI development remains on nRF52 until the peripheral event/task coverage in
this table is complete and tested on representative hardware. Do not move
implementation work to nRF53/nRF54 first. The nRF52 PEI backend already owns
PPI routing; *peripheral drivers* must own endpoint configuration, lifetime,
resource checks and task readiness. No ISR is permitted to forward a PEI event.

## Source and product scope

Nordic product specifications:
- [nRF52805](https://docs.nordicsemi.com/r/bundle/ps_nrf52805)
- [nRF52810](https://docs.nordicsemi.com/r/bundle/ps_nrf52810)
- [nRF52811](https://docs.nordicsemi.com/r/bundle/ps_nrf52811)
- [nRF52820](https://docs.nordicsemi.com/r/bundle/ps_nrf52820)
- [nRF52832](https://docs.nordicsemi.com/r/bundle/ps_nrf52832)
- [nRF52833](https://docs.nordicsemi.com/r/bundle/ps_nrf52833)
- [nRF52840](https://docs.nordicsemi.com/r/bundle/ps_nrf52840)
- [Nordic nRF52 series comparison](https://docs.nordicsemi.com/r/bundle/struct_nrf52/page/struct/nrf52.html)

Target features must be selected from device-header registers at compile
time. The peripheral sets vary: nRF52805 has no PWM/PDM/I2S or USB;
nRF52820 has USB but lacks SAADC/PWM/PDM; nRF52833 has USB; QSPI is
specific to nRF52840. Verify the exact availability of each instance,
event and task from the selected Nordic MDK header. Never reference a
peripheral that is absent on the selected SKU.

## Required HAL endpoint integration

Status keys:
- **Working hardware before C PEI refactor**: hardware behavior was observed,
  but must be rerun after the `PeiDev_t` change.
- **Initial endpoint added**: only the listed subset is implemented;
  not a complete peripheral integration.
- **Pending**: no validated endpoint API yet.
- **Restricted**: must enforce peripheral/stack ownership, and fail safely
  when the resource is unavailable. This is not permission to configure
  stack-owned hardware.

| Peripheral / driver | Hardware event source coverage | Hardware task destination coverage | Current status |
| --- | --- | --- | --- |
| TIMER / `timer_hf_nrfx.cpp` | Compare[n], plus other events implemented by the selected TIMER | START, STOP, CLEAR, COUNT, CAPTURE[n] and device-supported tasks | Compare -> toggle and START/STOP **working hardware before C PEI refactor**; additional endpoints pending |
| RTC / `timer_lf_nrfx.cpp` | TICK, OVRFLW, COMPARE[n]; RTC requires EVTEN independent of INTEN | START, STOP, CLEAR, TRIGOVRFLW where present | START/STOP **initial endpoint added**; event endpoints and remaining tasks pending |
| GPIOTE / `iopincfg_nrfx.c` | IN[n] edge, PORT event where supported | OUT[n] SET/CLEAR/TOGGLE through configured channels | IN[n] + output toggle **working hardware before C PEI refactor**; PORT and lifecycle stress pending |
| SAADC / `adc_nrf52_saadc.cpp` | STARTED, END, DONE, RESULTDONE, STOPPED, CALIBRATEDONE and available limits | START, SAMPLE, STOP, CALIBRATEOFFSET | **Pending**; cannot expose SAMPLE until conversion channels and EasyDMA buffers are configured and reserved |
| COMP / LPCOMP / `analog_comp_nrf5x.cpp` | READY, UP, DOWN, CROSS and supported variants | START, STOP, SAMPLE where provided | **Pending**; only expose events/tasks from the configured comparator |
| PWM / `pwm_nrfx.cpp` | SEQSTARTED[n], SEQEND[n], LOOPSDONE, PWMPERIODEND, STOPPED | SEQSTART[n], STOP and NEXTSTEP if implemented | **Pending**; requires valid sequence buffers and owner-safe start/stop |
| PDM / `pdm_nrfx.cpp` | STARTED, END, STOPPED | START, STOP | **Pending**; must not bypass sample-buffer configuration |
| I2S / `i2s_nrfx.cpp` | RXPTRUPD, TXPTRUPD, STOPPED | START, STOP | **Pending**; respect DMA pointer and double-buffer ownership |
| SPI / SPIM / SPIS / `spi_nrfx.cpp` | Ready, started, ended, stopped, error/transfer events actually present in each mode | START, STOP, SUSPEND, RESUME and mode-specific tasks | **Pending**; legacy SPI and EasyDMA modes share instances and are not interchangeable |
| TWI / TWIM / TWIS / `i2c_nrfx.cpp` | Transfer ready, stopped, error and other supported events per mode | STARTRX, STARTTX, STOP, SUSPEND, RESUME and mode-specific tasks | **Pending**; bus state and DMA buffer ownership stay with I2C driver |
| UART / UARTE / `uart_nrfx.cpp` | RX/TX ready, transfer end, stopped, error and mode-specific events | STARTRX, STOPRX, STARTTX, STOPTX, FLUSHRX where available | **Pending**; do not trigger unprepared EasyDMA |
| EGU (new nRF52 peripheral driver) | TRIGGERED[n] | TRIGGER[n] | **Pending**; no existing standalone EGU HAL driver in repository |
| QDEC (new nRF52 peripheral driver) | SAMPLERDY, REPORTRDY, ACCOF and device events | START, STOP, READCLR and device-supported tasks | **Pending**; no existing QDEC HAL driver |
| RNG / `rng_nrfx.cpp` | VALRDY | START, STOP | **Pending**; respect random-driver ownership and SoftDevice RNG service |
| TEMP / `temp_nrf5x.cpp` | DATARDY | START, STOP | **Pending**; respect temperature measurement lifecycle |
| WDT / `wdt_nrfx.cpp` | TIMEOUT | START | **Restricted**; START cannot be undone without reset on nRF52 |
| POWER / CLOCK / `power_clock_irq_nrf52.cpp` | Hardware power and clock events | Supported clock start/stop tasks | **Restricted**; shared clock ownership and power-state transitions |
| NFCT / `nfct_nrfx.cpp` | Field, frame, TX/RX and other device-supported events | Supported NFCT tasks | **Restricted**; nrfx NFCT stack owns its configured transfer state |
| USBD / `usb_ctrlr_nrf52.cpp` | SOF, endpoint, bus and USB state events | Endpoint/task triggers where actually supported | **Restricted**; only nRF52820/33/40, with endpoint-buffer ownership |
| QSPI / `qspi_nrfx.cpp` | READY and supported device events | ACTIVATE, DEACTIVATE, transfer/erase tasks | **Restricted**; nRF52840 only; never infer DMA/command descriptor from a PEI link |
| RADIO / AAR / CCM / ECB | Radio, cryptography and packet processing events actually exposed in the selected MCU | Supported hardware tasks | **Restricted**; leave SoftDevice/SDC resources untouched. Only allow explicitly standalone-owned operations |
| MWU and other supported event-generating blocks | Hardware events present on the selected MCU | Real hardware task registers, if any | **Pending**; do not invent tasks for event-only blocks |
| PPI groups and fixed channels / `pei_nrf52.cpp` | Hardware group/channel events where present | CHG[n].EN/DIS and fixed-channel functionality | **Pending**; this belongs inside the PEI routing backend, not a peripheral-specific data-transfer driver |

There is no need to give NVMC, FICR, UICR, plain GPIO registers or
software-only interrupt sources invented PEI endpoints. Include a peripheral
only when it implements a real PPI-accessible event/task register on the SKU.

## Design and implementation order

1. **Finish TIMER + RTC + GPIOTE.** Expose their remaining hardware
   endpoints without changing the existing ISR-based Timer trigger modes.
   RTC EVTEN must be managed separately from INTEN. Continuous RTC
   compare events cannot be promised ISR-free until an actual hardware
   compare-rearm method is implemented and validated.
2. **SAADC and comparators.** Validate RTC/TIMER event -> ADC sampling and
   comparator threshold -> GPIO or TIMER task, with peripheral-owned
   channel/DMA configuration.
3. **PWM/PDM/I2S.** Validate sequence/buffer event chains and correct
   START/STOP lifecycle. No DMA transfer configuration by PEI.
4. **SPIM/SPIS/TWIM/TWIS/UARTE and legacy serial modes.** Cover the
   underlying mode-specific register maps without stealing an instance
   already in use.
5. **EGU/QDEC/RNG/TEMP, PPI groups.** Add missing peripheral drivers where
   IOsonata currently has none; test event fan-out/fan-in and exhaustion.
6. **Restricted and variant-specific peripherals.** CLOCK, POWER, WDT,
   NFCT, RADIO/CCM/AAR/ECB, QSPI, USBD, MWU. Return unsupported for
   stack-owned or unprepared hardware states rather than provide unsafe
   automation.

## Completion criteria before nRF53

- All *available and safely application-owned* hardware event/task
  endpoints in the product specifications have driver API coverage.
- Each event/task endpoint is backed by a configured peripheral, and
  disable/deinit releases routing before reuse. No raw register addresses,
  PPI channel numbers or interconnect-specific adapter files in examples.
- C (`PeiDev_t`) and C++ (`Pei`) applications can use the same endpoints.
  Existing C HAL APIs remain valid; no new C++ dependency in C drivers.
- No ISR forwarding fallback. Test peripheral routing with the ISR disabled
  where autonomous behavior is promised.
- PPI fan-out, fan-in, source rebinding, lifecycle, channel exhaustion and
  SoftDevice/SDC reservation behavior are tested.
- MCU-variant compile matrix includes nRF52805, 52810, 52811, 52820,
  52832, 52833 and 52840, with clear unsupported-feature results.
- Hardware acceptance covers at least nRF52832 and nRF52840 including
  variant-specific features, and Bluetooth coexistence where relevant.
- Explicitly documented hardware-only / firmware-managed limitations
  are not falsely reported as supported PEI tasks.

## Validation status

The BLYST-NANO-DK Timer + two LEDs + two buttons demonstrated working
PPI hardware prior to the conversion of `Pei` to the C `PeiDev_t`
matrix implementation. That result must be rechecked after the C API
refactor. RTC START/STOP task endpoints have been prepared for inclusion;
they have **not** been MCU-compiled or hardware-tested. All other rows
remain unimplemented for PEI unless independently marked above.
