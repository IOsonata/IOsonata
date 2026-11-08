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
| TIMER / `timer_hf_nrfx.cpp` | COMPARE[n] single-shot/continuous; other events pending | START, STOP, CLEAR; COUNT and CAPTURE[n] pending | **Mixed-CC TIMER3 hardware working** in both CC orders (user report, Nov 8); COUNT/CAPTURE endpoints pending |
| RTC / `timer_lf_nrfx.cpp` | TICK, OVRFLW; COMPARE[n] single-shot/continuous | START, STOP, CLEAR, TRIGOVRFLW | **Mixed-CC RTC0 hardware working** in both CC orders; **RTC2 working** with ISR-first order (user report, Nov 8); other sources and lifecycle pending |
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
   Preserve the original free-running counter and ISR-rescheduled CC
   behavior: PPI forwards compare events only to hardware tasks, not
   through an interrupt. Validate mixed PEI/ISR channels, one-shot
   disarm, teardown and elapsed-time continuity on hardware. Then finish
   TIMER capture/counter, GPIOTE PORT and other remaining endpoints.
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

## RTC compare and PEI behavior

Each nRF52 RTC compare channel is independent. `TimerEnablePeiTrigger()`
binds one `EVENTS_COMPARE[n]` register as a source in the caller's
`PeiDev_t` / `Pei` matrix. The RTC driver never connects that event to
`TASKS_STOP` or `TASKS_CLEAR`, and never stops or resets the counter
in response to a compare event. The elapsed-time API remains usable.

A continuous trigger reprograms only `CC[n]` in the normal RTC ISR:
`CC[n] = (COUNTER + interval_ticks) & 0xFFFFFF`. The event-to-task
path remains entirely in hardware through PPI. A single-shot trigger
disarms only the matching compare event and interrupt after firing.
Neither mode reserves other CC channels. Ordinary interrupt-driven
triggers, PEI CC events, and TICK/OVRFLW PEI sources may coexist.

`TimerEnablePeiEvent()` binds RTC TICK/COUNTER_OVR event sources by
setting `EVTEN` independently from `INTEN`. The driver's ownership
records ensure the corresponding event can be unbound without stealing
other sources. Explicit counter START, STOP and CLEAR commands remain
available through the Timer API; compare triggers never issue them.

The price of preserving the free-running counter is that continuous
compare scheduling requires a Timer ISR to advance the next CC value.
That ISR never forwards the event to the PEI destination. Tests must
confirm the separate compare schedules, correct elapsed time, and
hardware PPI routing while the Timer ISR is active.

Hardware test: the nRF52832 `PeiBlinky` board now selects `DevNo = 6`
(TIMER3 at 16 MHz with `Freq = 0`) by default. Build and flash the
standalone bare-metal example with the same two LEDs and BUT1/BUT2.
Confirm start/stop and 500 ms alternating LEDs, then test release/reuse
and inspect ISR entries. This tests the HF TIMER backend, **not RTC**;
a separate RTC2 (`DevNo = 2`) configuration is required to validate the
low-frequency RTC PEI path.

## Completion criteria before nRF53

- All *available and safely application-owned* hardware event/task
  endpoints in the product specifications have driver API coverage.
- Each event/task endpoint is backed by a configured peripheral, and
  disable/deinit releases routing before reuse. No raw register addresses,
  PPI channel numbers or interconnect-specific adapter files in examples.
- C (`PeiDev_t`) and C++ (`Pei`) applications can use the same endpoints.
  Existing C HAL APIs remain valid; no new C++ dependency in C drivers.
- No ISR event-to-task forwarding fallback. Test hardware PPI routing
  independently from the Timer ISR used solely for continuous CC updates.
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
matrix implementation. After the free-running counter correction,
the user confirmed PeiTimerCcTest working on RTC0 and HF TIMER3 with
both CC allocation orders, and on RTC2 with the ISR-first allocation
(November 8, 2026). An RTC0 debugger capture showed Status = 2,
Errors = 0, TimerFrequency = 32768, and PeiTrigger = 0.
Exact numeric diagnostics for TIMER3/RTC2 were not supplied.
The RTC2 PEI-first order has not yet been separately confirmed.
Compare interrupts advance only their own CC in continuous mode;
PEI event-to-task delivery remains hardware PPI. No compare stops
or clears the elapsed counter. Single-shot behavior, RTC raw TICK/
OVRFLW sources, teardown/reuse, and remaining endpoints still
require dedicated validation. Other peripheral rows remain pending
unless independently marked above.
