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
| TIMER / `timer_hf_nrfx.cpp` | COMPARE[n] single-shot/continuous; other events pending | START, STOP, CLEAR; CAPTURE[n] pending | **Mixed-CC TIMER3 hardware working** in both CC orders (user report, Nov 8); CAPTURE endpoint pending |
| TIMER in counter mode / `counter_nrfx.cpp` (`counter.h`) | COMPARE0, the limit event of the counter | COUNT, START, STOP | **Initial endpoint added**: Counter on a TIMER of the timer driver table, low power counter mode, limit in CC[0] with the COMPARE0_CLEAR short; PeiCounterDemo (button GPIOTE IN to COUNT, limit to LED toggle) hardware check pending |
| RTC / `timer_lf_nrfx.cpp` | TICK, OVRFLW; COMPARE[n] single-shot/continuous | START, STOP, CLEAR, TRIGOVRFLW | **Mixed-CC RTC0 hardware working** in both CC orders; **RTC2 working** with ISR-first order (user report, Nov 8); other sources and lifecycle pending |
| GPIOTE / `iopincfg_nrfx.c` | IN[n] edge; shared PORT SENSE/DETECT source implemented on nRF52 | OUT[n] SET/CLEAR/TOGGLE through configured channels | **PeiPortDemo functional hardware PASS**; **PeiGpioteStress automatic hardware PASS** (Nov 8 screenshot: Status 2, Error 0, 64 exhaustion/reuse, 128 PPI-route, 128 IRQ rounds, IrqCount 13); BUT2 physical route confirmation and cross-variant tests pending |
| SAADC / `adc_nrf52_saadc.cpp` | STARTED, END, DONE, RESULTDONE, STOPPED implemented; CALIBRATEDONE and limit events pending (limits reach the application through the driver callback) | START, SAMPLE, STOP implemented; CALIBRATEOFFSET pending | **Initial endpoint added**: single-conversion/no-IRQ driver-owned EasyDMA preparation, nRF52832 erratum 150 guard, C/C++ PEI routing; continuous mode with double-buffered blocks accepts SAMPLE only; PeiSaadcDemo TIMER->SAMPLE / END->START and PeiAdcDemo TIMER->SAMPLE hardware checks pending |
| COMP / LPCOMP / `analog_comp_nrf5x.cpp` | READY, DOWN, UP, CROSS implemented for nRF52 | START, STOP, SAMPLE (single mode only) implemented | **Initial endpoint added:** common C/C++ HAL, owner/enable checks and no-IRQ PEI routing; PeiCompDemo hardware validation pending |
| PWM / `pwm_nrfx.cpp` | SEQSTARTED[0/1], SEQEND[0/1], LOOPSDONE, PWMPERIODEND, STOPPED implemented | SEQSTART[0/1], STOP implemented; NEXTSTEP conditional on decoder mode | **PeiPwmDemo functional hardware PASS on nRF52832** (maintainer report, Nov 8, 2026): BUT1/BUT2 PWM start/stop and event indication; driver-owned static EasyDMA sequences, channel/ENABLE checks and C/C++ APIs. Additional PWM events, NEXTSTEP and lifecycle stress pending |
| PDM / `pdm_nrfx.cpp` | STARTED, END, STOPPED implemented on nRF52 | START, STOP after FIFO/DMA preparation | **Initial PEI integration:** C/C++ APIs; PdmPreparePeiStart reserves the first driver-owned FIFO block and configures SAMPLE.PTR/MAXCNT; mandatory STARTED IRQ buffer refill; source/API regressions added. No device hardware test. |
| I2S / `i2s_nrfx.cpp` | RXPTRUPD (RX configured), TXPTRUPD (TX configured), STOPPED implemented on nRF52 | START, STOP with initialized and armed driver-owned DMA | **Initial PEI integration:** C/C++ APIs; I2SPreparePeiStart re-arms existing RX/TX buffers without START after I2SDisable; IRQ pointer service required. Source/API regressions added. No device hardware test. |
| SPI / SPIM / SPIS / `spi_nrfx.cpp` | SPI READY; SPIM STARTED/ENDRX/ENDTX/END/STOPPED; SPIS ENDRX/END/ACQUIRED | SPIM START/STOP/SUSPEND/RESUME only after driver-prepared EasyDMA; legacy SPI no tasks; SPIS ACQUIRE/RELEASE still IRQ-owned | **Initial endpoint added**: mode-specific C/C++ PEI APIs, single-block SPIM DMA preparation and release, PeiSpiDemo and static tests; MCU/hardware validation pending |
| TWI / TWIM / TWIS / `i2c_nrfx.cpp` | Transfer ready, stopped, error and other supported events per mode | STARTRX, STARTTX, STOP, SUSPEND, RESUME and mode-specific tasks | **Pending**; bus state and DMA buffer ownership stay with I2C driver |
| UART / UARTE / `uart_nrfx.cpp` | RX/TX ready, transfer end, stopped, error and mode-specific events | STARTRX, STOPRX, STARTTX, STOPTX, FLUSHRX where available | **Pending**; do not trigger unprepared EasyDMA |
| EGU / `pei_nrf52.cpp` (owned by PEI, no standalone driver) | TRIGGERED[n] as a software event source (`PeiBindSwEvent`, `PeiTriggerSwEvent`) | TRIGGER[n] (`PeiGetSwEventTask`) | **Initial endpoint added**: host-model tests for all build profiles; PeiSwEvent example on nRF52832 not yet run on hardware. No EGU interrupt or callback |
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
   TIMER capture/counter and GPIOTE PORT lifecycle on hardware, then
   other remaining endpoints.
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

## SAADC task/event implementation (initial nRF52 subset)

The ADC driver retains ownership of its configured SAADC channels and
its internal, fixed EasyDMA result buffer. The new
AdcnRF52::PreparePeiSampling() method configures RESULT.PTR and
RESULT.MAXCNT and executes an initial START without taking a sample.
The C API in ARM/Nordic/include/adc_nrf52_pei.h and AdcnRF52 C++
wrappers then expose START/SAMPLE/STOP hardware tasks and
STARTED/END/DONE/RESULTDONE/STOPPED events to PEI. The existing
AdcDevice abstraction is still C++ only; the PEI routing functions
are callable from C but do not provide a separate C ADC configuration
API.

For this first subset the conversion configuration must be
single-conversion and noninterrupt, with at least one enabled channel.
No PEI callback/ISR forwards an event. To make periodic conversions,
the new PeiSaadcDemo on nRF52832 connects TIMER3.COMPARE -> SAMPLE
and SAADC.END -> START (rearm), with independent hardware LED tasks
on each event. Both paths operate through PPI.

Continuous mode with interrupt is the second accepted state. After
StartConversion, when the SAADC internal timer is not used (several
channels or Rate 0), the driver owns two EasyDMA blocks of
BlockRounds() rounds; the END interrupt restarts the next block and
moves the full one to the channel FIFOs. PEI may then connect any
event to SAMPLE, each event converts one round. START and STOP are
refused in this mode, they would work against the END interrupt.
PeiAdcDemo (nRF52832 BLYST-NANO-DK and nRF52840 DK) connects
TIMER3.COMPARE -> SAMPLE this way, with limits on one channel; it ran
in an instruction set emulator with TIMER3, PPI and SAADC models, not
yet on hardware.

On nRF52832, SAADC acquisition time must be greater than 5 us
before enabling PPI-driven START (Nordic erratum 150). The driver
rejects unsafe configurations during preparation. CALIBRATEOFFSET,
calibration completion, channel limit events, reconfiguration during
active PEI routes, and stack-owned SAADC arbitration remain pending.
Disconnect SAADC task destinations and event producers before
StopConversion(), CloseChannel(), Disable(), or PowerOff().

The PeiSaadcDemo Eclipse project is added, but no MCU build or
hardware result has been confirmed. Do not mark SAADC as fully
covered based on static source tests.

## TIMER counter mode (Counter)

`counter.h` is the generic event counter. On the nRF it is a TIMER of the
high frequency timer driver put in low power counter mode: the Counter and
the Timer are two objects on the same TIMER instances. `CounterInit` and
`TimerInit` each set the mode of the TIMER they are given, the last one has
it. `counter_nrfx.cpp` uses the TIMER table of `timer_hf_nrfx.cpp`; the timer
interrupt handler calls `nRFxCounterIrq` for a TIMER in counter mode. The
timer driver has an empty weak `nRFxCounterIrq` that `counter_nrfx.cpp`
replaces, so an application without `CounterInit` links no counter code, the
same way the USB ISO support is kept out of `usb_ctrlr_nrf52.cpp`.

The counted events reach the COUNT task with PEI. An input pin is routed by
the application with the existing GPIO and PEI calls:

```c
IOPinEnableEvent(&pei, &evt, PortNo, PinNo, IOPINSENSE_LOW_TRANSITION);
CounterConnectTask(&counter, &pei, evt, COUNTER_TASK_COUNT);
PeiEnableEvent(&pei, evt);
```

The limit is CC[0] with the COMPARE0_CLEAR short, COMPARE0 is the limit
event; without limit CC[0] is 0 and COMPARE0 is the 32 bit wrap. The
interrupt folds each period into the 64 bit count.

## Comparator COMP/LPCOMP task and event integration

The generic AnalogComp C/C++ HAL now exposes READY, DOWN, UP and CROSS
hardware event endpoints and START, STOP and SAMPLE task destinations.
Its Nordic driver selects COMP or LPCOMP from the MCU register map; the
nRF52 implementation checks the initialized comparator owner and the
hardware ENABLE state before binding or connecting routes. PEI only
routes the already-configured comparator: the driver retains threshold,
reference, input pin and power management.

In CONTINUOUS mode, the comparator driver uses READY->SAMPLE, and rejects
external PEI SAMPLE destinations to avoid conflicting triggers. In
SINGLE mode the SAMPLE task is available after the comparator has been
started. When no conventional callback is configured, the comparator
IRQ is not enabled; hardware PEI routing does not need an ISR.
Applications must disconnect comparator task destinations and unbind
its event sources before disabling/reconfiguring the comparator.

PeiCompDemo on nRF52832 BLYST-NANO-DK demonstrates a hardware GPIOTE
input event driving LPCOMP START and four independent comparator
event sources routed to LEDs. BUT1 is AIN0/P0.02 and crosses the VDD/2
reference when pressed and released. Initial hardware validation,
SINGLE-mode SAMPLE, teardown/reuse and other nRF52 variants remain
pending. No hardware pass is inferred from static checks.

## PWM event/task integration (initial nRF52 subset)

The existing PWM driver owns all configuration and both internal
static four-channel EasyDMA sequences. No PEI-specific PWM buffers or
heap state are allocated. PWMInit(), PWMOpenChannel(), DutyCycle() and
PWMEnable() prepare a valid PWM output before the PWM-specific PEI
C/C++ APIs bind events or connect tasks. The driver verifies the
owning PwmDev_t, channel mask, enabled register, frequency, counter
top and both sequence DMA pointers/counts.

The initial event sources are SEQSTARTED[0/1], SEQEND[0/1],
LOOPSDONE, PWMPERIODEND and STOPPED. Hardware task destinations are
SEQSTART[0/1] and STOP. NEXTSTEP is only connectable when the
driver-owned DECODER mode is NextStep. The existing HAL configures
RefreshCount, so NEXTSTEP is not exercised by the default PWM class.
Disconnecting a task is allowed even if its decoder mode subsequently
changes, to avoid retaining a stale hardware PPI connection.

PWMInit() now resets any previous LOOP/SHORTS/INTEN state and rejects
a zero configuration frequency. The PWM driver remains the only owner
of output pins and DMA buffers; PEI does not invoke a CPU ISR to
forward or acknowledge PWM events. The new wrapper functions are
optional weak PEI references and should not extract the PEI matrix
when a normal PWM-only firmware is linked.

The standalone nRF52832 PeiPwmDemo maps BUT1 to PWM0.SEQSTART0 and
BUT2 to PWM0.STOP, both through hardware PPI. PWM channel 0 drives
RGB red, while PWM SEQSTARTED0 and STOPPED toggle the physically
separate LED1 and RGB blue, respectively. C/C++ compile checks and
source contract tests are added. The maintainer confirmed that
PeiPwmDemo works on nRF52832 BLYST-NANO-DK (Nov 8, 2026), validating
the demonstrated start/stop routing and indication behavior. This does
not individually verify every supported event or task. Further lifecycle
stress, second-sequence tests, NEXTSTEP decoder support and cross-variant
testing remain outstanding.

## PDM and I2S PEI integration (nRF52, hardware unvalidated)

PDM exposes STARTED, END and STOPPED source events and START/STOP task
destinations through the existing PdmDev_t C HAL. No second FIFO or
DMA buffer is allocated. PdmInit must successfully create a caller-owned
CFIFO and enable normal PDM interrupts (bIntEn = true). A caller who
wants a GPIO or TIMER event to START capture first invokes
PdmPreparePeiStart, which reserves one FIFO block, sets SAMPLE.PTR and
SAMPLE.MAXCNT, and does not issue TASKS_START. Then connect and enable
its PEI route. The normal PDM STARTED ISR replenishes the double-buffered
DMA pointer for continuous sampling. Do not route START with no
FIFO or no interrupt refill. Releasing and reconnecting the PEI route
does not transfer FIFO ownership; the caller must disconnect routes
before stopping, changing mode or disabling PDM. Do not restart a
stopping PDM before STOPPED is observed.

I2S exposes RXPTRUPD, TXPTRUPD and STOPPED source events and START/STOP
task destinations through the existing I2SDev_t C/C++ HAL. It retains
the two 128-word static RX and TX DMA buffers, the normal interrupt
pointer-update service and FIFOs. RX/TX event availability is restricted
to actually enabled transfer directions. The normal I2SInit starts
streaming as before. An application that needs PPI-controlled START
initializes I2S with bIntEn = true, disables it once, then calls
I2SPreparePeiStart. This reconfigures DMA pointers, IRQ service and
hardware ENABLE, but does **not** trigger TASKS_START. DeviceIntrf's
enable reference count is restored so a normal I2SDisable later
releases the prepared/started interface. PEI routes only the task
register; I2S remains responsible for pointer updates.

Neither driver introduces software interrupt event-forwarding. Routing
functions use weak references to the optional PEI matrix, so a
PDM/I2S-only firmware should not retain the PPI backend. Only the
nRF52 legacy PPI endpoints are exposed; nRF53/nRF54 use other
interconnect backends and are not covered by these APIs yet.

The hardware-free tests check the C and C++ public declarations,
register endpoint mapping, essential buffer/IRQ preparation contracts
and non-PEI linker-section isolation. They are not a substitute for
compiling against all supported installed Nordic CMSIS headers or a
live microphone/I2S controller. These features remain **implemented
but hardware-unvalidated**. Future device tests should exercise PDM
FIFO rollover and STOPPED/restart sequencing, I2S RX-only, TX-only
and duplex streaming, pointer-update event delivery, STOPPED, and
repeated source/task release.

## SPI/SPIM/SPIS PEI integration (initial nRF52 subset)

Only event sources belonging to the currently configured SPI mode
are exposed. Legacy SPI provides READY; SPIM provides STARTED,
ENDRX, ENDTX, END and STOPPED; SPIS provides ENDRX, END and
ACQUIRED. The shared register base does not make these interchangeable.
SPIS ACQUIRE/RELEASE tasks are not connected through PEI: the
existing slave ISR owns the semaphore and DMA pointer updates.

SPIM START, STOP, SUSPEND and RESUME may be connected after
SPIPreparePeiTransfer() arms one block with the existing private TX
buffer (max 256 bytes) and optionally caller-owned RX RAM. The
configured SPI must be MASTER, 8-bit, noninterrupt EasyDMA and
manual-CS; the caller owns chip select, RX lifetime and PPI teardown.
nRF52832 one-byte RX anomaly 58 is rejected where the driver's CPU
fallback would normally be needed. Ordinary SPI transfers and SPIInit
are blocked while PEI has reserved the SPIM transfer. Disconnect task
routes, then call SPIReleasePeiTransfer() to stop and free DMA pointers.
No PEI ISR event forwarding or separate DMA buffer is introduced.

PeiSpiDemo on nRF52832 BLYST-NANO-DK needs no external SPI slave:
BUT1 starts an eight-byte SPIM2 transmission; END routes to STOP
and LED1, and STOPPED toggles RGB blue, all via PPI. Host API/source
checks were added but target MCU compilation, hardware behavior,
SPIS semaphore handoff, shared SPI/TWI lifecycle and all variants
remain unvalidated.

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
require dedicated validation. The user also confirmed PeiPortDemo
working on nRF52832 (November 8, 2026). This exercises the shared
GPIOTE PORT event and independent IN[n] PPI paths after the demo's
setup-time unbind/rebind and task/event resource reuse sequence.
An additional PeiGpioteStress managed project now exercises the
8-channel exhaustion boundary, duplicate channel/pin rejection,
64 allocation/reuse rounds and 128 PPI-route plus conventional IRQ
lifecycle rounds, then arms the two buttons for hardware coexistence.
User debugger capture on November 8, 2026 confirmed the automatic
resource/lifecycle checks: Status 2, Error 0, ExhaustionRounds 64,
ReusedChannels 64, RouteRounds 128, IrqRounds 128, LastIrqNo 0,
and IrqCount 13. This verifies ordinary IRQ execution, not by
itself the physical BUT2-to-LED1 PPI event route. Cross-variant
and SoftDevice/SDC GPIOTE ownership tests remain unvalidated. Other peripheral rows
remain pending unless independently marked above.
