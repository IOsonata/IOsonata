# Peripheral Event Interconnect (PEI)

PEI is a programmable hardware interconnect matrix. It connects peripheral
**event sources** to peripheral **task destinations**. It does not transfer data
and is not derived from `DeviceIntrf` or `Device`.

```text
Timer compare -------+
GPIO edge -----------+--> PEI interconnect matrix --> GPIO task
Peripheral event ----+                           +--> Timer task
                                                 +--> ADC sample
```

The arrows represent possible connections, not universal MCU capabilities.
A connection is created by programming the MCU's event-routing hardware.
After configuration, event delivery does not require a CPU interrupt or
software forwarding callback.

## Generic connection model

`PeiDev_t` owns the complete **logical event-to-task matrix** in C.
`Pei` is a thin C++ wrapper around the same C device and operations.
Neither application language has privileged access to PEI functionality.
One event can activate multiple tasks; fan-in is allowed when the
hardware backend supports it without introducing unintended connections.

A C application uses caller-owned device storage:

```c
PeiDev_t pei;
PeiEvent_t evt = 0;

PeiInit(&pei, 0);
PeiBindEvent(&pei, &evt, hardwareEvent);
PeiConnectTask(&pei, evt, hardwareTaskA);
PeiConnectTask(&pei, evt, hardwareTaskB);
PeiEnableEvent(&pei, evt);
```

The equivalent C++ application uses `Pei pei;`, `pei.BindEvent()`,
`pei.Connect()` and `pei.Enable()`. Both paths execute the same
`src/coredev/pei.c` matrix implementation. PEI stores fixed-size event
and route tables in `PeiDev_t`; it allocates no heap memory.

A peripheral driver supplies an already-configured hardware endpoint.
PEI owns only its relationship to other peripheral endpoints:

```cpp
Pei pei;
PeiEvent_t evt = 0;

pei.BindEvent(evt, hardwareEvent);
pei.Connect(evt, hardwareTaskA);
pei.Connect(evt, hardwareTaskB);
pei.Enable(evt);
```

These operations configure only the MCU event interconnect. They do
**not** configure a timer period, ADC conversion, GPIO pin, DMA
descriptor, peripheral clock, interrupt handler or peripheral state.

- `BindEvent(evt, endpoint)` assigns an event source, allocating a
  persistent `PeiEvent_t` identity when `evt == 0`.
- `Connect(evt, task)` adds one logical relationship. A repeated pair is
  idempotent; a backend may reject unsupported fan-in or other topology.
- `Enable(evt)`/`Disable(evt)` change the event's routing state, not
  the peripheral's event generation or interrupt settings.
- `Disconnect(evt, task)` removes one relationship without affecting
  any other task or event.
- `UnbindEvent(evt, endpoint)` disables and releases the physical routes
  **before** the producing driver reuses the endpoint. It retains the
  event identity, task relationships and logical enable state.
- `ReleaseEvent(evt)` removes the identity and all its relationships.

The matrix uses caller-owned, fixed-size storage: 16 events and 32
connections by default, overridable with `PEI_MAX_EVENTS` and
`PEI_MAX_CONNECTIONS` at build time. Callers must serialize changes
to one matrix. Live `PeiDev_t` instances must not be copied or moved.

`PeiBindEvent`, `PeiConnectTask`, `PeiEnableEvent`,
`PeiDisableEvent`, `PeiUnbindEvent`, `PeiDisconnectTask` and
`PeiReleaseEvent` are the public C matrix operations. The older
`PeiConnect`, `PeiEnable`, `PeiDisable` and `PeiDisconnect`
still operate on one low-level `PeiConn_t` routing resource and are
used internally by the C matrix and the MCU backend. Physical channel
numbers are never part of either application interface.

`PeiEndpoint_t` is the MCU-specific hardware endpoint identifier:
a register address on Nordic, a generator/user index on SAMD, or an
event/destination selector on R9A02. `PeiEvent_t` is **not** a hardware
channel. An unsupported route returns failure; it is never emulated
with software interrupt forwarding.

## Software events

A software event is an event source that the CPU fires. The MCU port
allocates the hardware for it, an EGU channel on nRF52. Its task
connections then run in hardware, all from the same event: one CPU write
starts every connected task at the same time.

```c
PeiEvent_t sw = 0;

PeiBindSwEvent(&pei, &sw);
PeiConnectTask(&pei, sw, hardwareTaskA);
PeiConnectTask(&pei, sw, hardwareTaskB);
PeiEnableEvent(&pei, sw);
PeiTriggerSwEvent(&pei, sw);
```

C++: `pei.BindSwEvent(sw)`, `pei.Trigger(sw)`. Apart from its source a
software event is an ordinary matrix event: enable, disable, connect and
disconnect work the same way, and the tasks run only while it is enabled.
`PeiUnbindEvent` and `PeiReleaseEvent` give the hardware back to the port.
An unbound identity keeps its task connections, a new `PeiBindSwEvent`
on it allocates new hardware and reconnects them.

`PeiGetSwEventTask` (C++ `SwEventTask`) returns the hardware task that
fires the software event. Connecting another event to it fires the same
task set from hardware too, for example a button and the CPU starting the
same group of tasks.

The CPU writes the trigger itself, it does not forward an event from an
interrupt. No interrupt is enabled on the software event hardware, so there
is no callback. A port without software event hardware leaves the MCU
functions undefined; the weak references in `src/coredev/pei.c` then make
`PeiBindSwEvent` return false.

## MCU implementation rules

- Select only hardware-supported event/task combinations. Return failure when
  a connection cannot be represented; do not use an ISR as a substitute.
- Keep channel numbers and allocation tables in the MCU port. A logical
  connection must not expose a hardware channel or group number.
- Reserve channels already used by Bluetooth stacks, vendor libraries and
  other drivers. In particular, the nRF52 PPI mask depends on the SoftDevice
  variant and whether it is enabled; never assume all channels are free.
- Allocate and release atomically with respect to competing callers and
  interrupts, using the target's established critical-section method.
- Do not overwrite existing routes; on failed setup, release everything
  allocated during that attempt.
- Do not disable or reconfigure an event source or task peripheral when
  disconnecting the matrix connection. The peripheral's driver owns it.
- Reject cross-domain routes unless the MCU has the required hardware
  bridge between the domains.
- One-to-many and many-to-one behavior is available only when supported by
  the particular matrix and selected endpoint pair.

## Initial implementation sequence (0.14)

1. nRF52 PPI: an event-to-task connection, allocation/release, enable/disable,
   and hardware event-to-GPIOTE validation on nRF52832/BLYST-NANO-DK.
2. Nordic DPPI on supported nRF53 and nRF54 devices, including publisher,
   subscriber and security-domain ownership.
3. Microchip SAMD EVSYS: adapt the existing `SamdEvsysOpen` and
   `SamdEvsysClose` implementation without breaking timer or EIC users.
4. ESP32 ETM and other interconnect hardware where present, subject to each
   port's actual capabilities and maturity.

## Validation requirements

- C and C++ compilation of the generic header.
- Allocate, enable, disable, release and reuse connections; check resource
  exhaustion and repeated lifecycle calls.
- Connect one source to two destinations and two sources to one destination
  where the target supports it.
- Verify that failed connections preserve existing hardware routes.
- On hardware, route timer compare to a GPIOTE output task with no event
  forwarding interrupt. Verify that toggling stops on disable and resumes on
  enable without reallocation.
- Repeat with Bluetooth active on the target, verifying that existing
  stack-owned channels remain untouched.

## Status

The matrix-owning C++ interface and nRF52 PPI/nRF5340 DPPI low-level backends are committed.
Host-model tests exercise lifecycle and routing for both backends.
On November 7, 2026, the nRF52832 HAL and the refactored
`PeiBlinky` C++ example were reported compiled successfully on
BLYST-NANO-DK. The LED was observed flashing at approximately 1 Hz,
consistent with a 500 ms Timer compare -> PPI -> GPIOTE
task path. This is a **functional hardware PASS** for the HAL-integrated
example, not a measured frequency or debugger proof of zero ISR entries.
The nRF52832 compile is not a complete cross-target build validation.
Debugger IRQ tracing, PEI disable/re-enable and release tests,
SoftDevice/SDC coexistence, cross-compilation against installed nRF53
MDK and nRF5340 on-board tests remain pending.


## Optional PEI linking

The standard nRF52 GPIO and Timer HAL must not pull `pei.o` or
`pei_nrf52.o` into a firmware that does not use PEI. The previous
TimerDemo map showed `iopincfg_nrfx.o` extracting the PEI matrix via
`PeiGetEventEndpoint`, then that matrix extracting the PPI backend.

GPIO PEI entry points remain implemented directly in
`iopincfg_nrfx.c`, but their matrix references are weak. They check
for the optional matrix functions before reserving hardware channels.
Ordinary GPIO calls cannot extract the PEI matrix from a static HAL
archive.

The nRF52 TIMER and RTC drivers assign PEI callbacks during
`TimerInit`. They use the existing compare registers and `TimerTrig_t`
state. Continuous compares are rescheduled by the TIMER/RTC ISR;
PPI routes each compare event directly to its task. No compare
connection stops or clears the counter. `DisableTimerTrigger`
unbinds only the selected compare event.

Callback pointers may retain PEI-specific TIMER code in applications
that do not use PEI. Check the final linked sections and sizes before
accepting this initialization change.

This depends on GCC weak references and link garbage collection, as
used in the Eclipse managed projects. The source changes are not a
substitute for testing the actual final link: build TimerDemo and run
`tests/pei/check_no_pei_link.py` on its **newly generated** linker
map. Verify separately that PeiBlinky still functions with PEI.

## nRF52 PEI completion scope

Development stays on nRF52 until the product-variant peripheral event/task
coverage and safety restrictions are recorded as complete. The detailed
inventory, implementation order and acceptance checklist are in
[PEI nRF52 coverage and acceptance tracker](pei_nrf52_coverage.md).
Do not infer an available endpoint from its register address range alone.
Peripheral ownership and the device-header register map are decisive.

The nRF52 RTC Timer driver exposes TICK/OVRFLW event sources with
independent EVTEN control, and START/STOP/CLEAR/TRIGOVRFLW hardware
task endpoints. RTC and HF Timer CC channels may be used independently
by interrupt-driven triggers and PEI event sources. A PEI compare event
never stops or resets the elapsed-time counter; `TimerGetTickCount`
and its time-conversion APIs remain available.

In continuous mode, the existing Timer ISR advances only the matched
CC register relative to the current counter. PEI dispatches that
hardware event to its destination without CPU forwarding. Single-shot
mode disarms the selected event after its first match, without affecting
the shared counter or other CC channels. Disable/re-enable of the PEI
route does not reset the counter or interrupt-driven CC schedule.
These changes require a fresh MCU build and hardware revalidation. The
BLYST-NANO-DK PeiBlinky now selects `DevNo = 6` (TIMER3 at 16 MHz,
with `Freq = 0`) for the standalone PEI test. The RTC backend remains
unvalidated. The TIMER3 selection also requires a fresh MCU build and
hardware test; Bluetooth coexistence needs separate resource checks.

### nRF52 shared GPIOTE PORT event source

`IOPinEnablePortEvent(PeiDev_t *, PeiEvent_t *)` binds
`GPIOTE.EVENTS_PORT` to a PEI logical event. C++ uses the same C
implementation with `Pei &` and `PeiEvent_t &`.

PORT is a single event generated on the rising edge of the GPIO DETECT
signal combined across SENSE-enabled input pins. It is not a dedicated
per-pin edge source, does not identify the triggering pin and does not
consume an IN[n] GPIOTE channel. Configure low/high sensing separately
with the existing `IOPinSetSense()` API. Do not silently substitute
PORT for `IOPinEnableEvent()`.

The new API only binds the event endpoint. It neither changes
`PIN_CNF.SENSE`, `DETECTMODE`, `EVENTS_PORT`, `LATCH`, nor enables
PORT interrupts. `IOPinDisablePortEvent()` unbinds that logical event
without disturbing GPIO sensing, unrelated PORT interrupt users or
other GPIOTE channel allocations. The caller restores SENSE and
releases the logical PEI identity separately. Existing PEI destinations
can be retained across unbind and rebind.

A continuously asserted SENSE level, or the latch-based DETECT mode,
may prevent the DETECT signal from producing another rising edge.
The application remains responsible for the GPIO detect lifecycle.
The new standalone nRF52832 `PeiPortDemo` tests PORT routing alongside
an independent GPIOTE IN[n] source; hardware validation is pending.

### GPIOTE channel ownership and coexistence

The nRF52 legacy GPIOTE driver tracks task and event channel
ownership separately from conventional GPIO interrupt callbacks.
`IOPinEnableInterrupt()` now checks the actual channel CONFIG
and rejects a second assignment of the same pin across channels
while holding the local interrupt lock. Automatic IRQ channel
allocation skips any nonzero CONFIG entry, including those owned
externally. It clears an old EVENTS_IN flag on reuse. The GPIOTE
IRQ handler acknowledges only events owned by conventional IRQ
users, not PEI-owned IN[n] or PORT sources.

The PeiGpioteStress hardware example verifies exhaustion/recovery
and repeated lifecycle calls through the public GPIO and PEI
APIs, then a live ordinary IRQ alongside a PPI/GPIOTE task. Its
nRF52832 board result has not yet been reported. This does not
authorize stealing stack-owned GPIOTE channels or imply that
every nRF52 MCU exposes eight channels.

### SAADC hardware sample chain

For the initial nRF52 SAADC PEI integration, the existing
AdcnRF52 C++ driver still owns channel configuration and the
fixed internal EasyDMA result array. A separate C-compatible
header, ARM/Nordic/include/adc_nrf52_pei.h, provides PEI
event/task routing functions. AdcnRF52 also offers C++ methods
for the same operations.

After Init() and OpenChannel(), call PreparePeiSampling() in
single-conversion mode with bInterrupt=false. This configures the
driver's internal RESULT.PTR/MAXCNT buffer and issues one START,
without a SAMPLE task. PEI may then connect hardware TIMER compare
to SAADC SAMPLE and SAADC END to SAADC START. The END event can
fan out to GPIO tasks, so actual conversion completion is visible
without invoking an interrupt to forward the event.

Supported initial events: STARTED, END, DONE, RESULTDONE, STOPPED.
Supported tasks: START, SAMPLE, STOP. Calibration and channel limit
endpoints require separate lifecycle validation and remain pending.
This does not add any DMA buffer or PEI-only driver state to an
ordinary ADC application.

The nRF52832 erratum 150 requires TACQ > 5 us if SAADC START is
triggered by PPI. The driver rejects shorter acquisition times
for this prepared mode. All destination tasks must be disconnected
and source events unbound before stopping or reconfiguring ADC
channels, since the PEI matrix cannot safely operate a disabled
DMA destination.

The standalone nRF52832 PeiSaadcDemo Eclipse project validates this
chain on BLYST-NANO-DK with a 500 ms TIMER3 compare indicator and a
separate SAADC END indicator. Hardware testing remains pending.

### Analog comparator events and tasks on nRF52

The generic AnalogComp C/C++ HAL declares PEI binding of comparator
READY, UP, DOWN and CROSS hardware events, and connections to START,
STOP and SAMPLE tasks. Nordic's analog_comp_nrf5x.cpp implements those
endpoints for nRF52 using the COMP/LPCOMP register map. The driver
continues to own analog input selection, threshold/reference configuration
and ENABLE state, with no PEI-only allocation or event-forwarding ISR.

An initialized, enabled comparator is required for new routes. In
CONTINUOUS mode READY_SAMPLE already triggers SAMPLE, so a second
PPI SAMPLE destination is rejected. SINGLE mode may connect SAMPLE
after an explicit comparator START and READY. An application that uses
a pure PEI path leaves the conventional comparator EvtHandler null;
that avoids enabling the comparator interrupt.

The nRF52832 PeiCompDemo example on BLYST-NANO-DK uses a GPIO event
on BUT2 to start LPCOMP in hardware, and BLYST BUT1 (AIN0) to cross
an approximate VDD/2 reference. READY/CROSS/UP/DOWN feed four GPIO
toggle tasks via PPI. It does not claim ADC-like sample buffering or
software event forwarding. Hardware testing is pending. Disconnect
all routes before powering off or reinitializing a comparator.

### nRF52 PWM sequence event/task endpoints

The standard PwmDev_t / Pwm HAL is extended for C and C++ PEI
routing. PWM0..PWMn events SEQSTARTED[0/1], SEQEND[0/1],
LOOPSDONE, PWMPERIODEND and STOPPED may serve as event sources;
prepared PWM output tasks SEQSTART[0/1] and STOP may be PEI
destinations. NEXTSTEP is refused in the current RefreshCount
decoder mode and is only valid if the driver owner enables NextStep
mode.

PWMInit owns the two static RAM sequences. A configured, enabled
PWM device with at least one output channel is required before a new
PEI connection; both sequence pointers and counts are validated.
No extra PEI-specific buffer or IRQ is introduced. The default
PWM sequence with LOOP.CNT=0 is played once, then the last duty is
maintained until STOP. The driver does not automatically add a
LOOPSDONE replay shortcut. Disconnect all PEI relationships before
stopping, closing channels or reinitializing a PWM instance.

PeiPwmDemo on the nRF52832 BLYST-NANO-DK uses BUT1 and BUT2
to drive PWM0 SEQSTART0 and STOP over PPI without the CPU.
RGB red provides a PWM output (1 kHz, 30% duty);
SEQSTARTED0 toggles the separate LED1, and STOPPED toggles RGB blue.
It uses the ordinary PWM class and driver-owned sequence buffers.
The maintainer reported PeiPwmDemo working on nRF52832 on
November 8, 2026 (functional hardware PASS). This validates the
example's demonstrated start/stop route; remaining endpoints and
long-term resource lifecycle are not thereby independently verified.
No semihosting or debugger variable C-linkage wrappers are added.

### nRF52 PDM and I2S audio PEI endpoints

PDM (coredev/pdm.h) maps STARTED, END, STOPPED and START, STOP
to the hardware endpoints of the initialized nRF52 PDM peripheral.
PdmPreparePeiStart() reserves the initial sample block in the
application's existing CFIFO and programs SAMPLE.PTR/MAXCNT without
starting capture. It requires an enabled PDM interrupt handler because
the STARTED interrupt supplies successive buffers. PEI does not
prepare or exchange microphone sample buffers. The caller owns PDM
configuration, STOPPED acknowledgement and teardown.

I2S (coredev/i2s.h) maps RXPTRUPD, TXPTRUPD, STOPPED and START,
STOP to the configured nRF52 I2S. The driver's ordinary I2SInit()
continues to start streaming immediately. For PEI-controlled START,
initialize with bIntEn=true, call I2SDisable(), then
I2SPreparePeiStart(). The latter reuses the driver's two static RX/TX
EasyDMA buffers, enables the normal pointer-update interrupt service,
and arms without issuing the START task. A device interface enable
count is retained so I2SDisable() continues to clean up. Only
configured RX/TX directions can bind their pointer-update events.

Both C and C++ applications use the same PEI matrix, and normal
PDM/I2S firmware should not link unused PEI code. No microphone or
I2S external device is currently available for this project's hardware
validation, so the implementation is not marked as device-tested.
The nRF53 and nRF54 DPPI/TDM ports are not implied by nRF52 support.

### nRF52 SPI/SPIM/SPIS hardware PEI endpoints

The existing SPI HAL exposes mode-specific event registers: SPI
READY; SPIM STARTED, ENDRX, ENDTX, END, STOPPED; SPIS ENDRX,
END, ACQUIRED. Only SPIM START, STOP, SUSPEND and RESUME tasks
are connected after SPIPreparePeiTransfer() validates and stages a
driver-owned DMA transaction. SPIS ACQUIRE/RELEASE remain with the
slave interrupt handler, which owns its semaphore and buffers.

SPIM preparation requires an 8-bit master with manual CS,
noninterrupt EasyDMA, a stable caller-owned RX buffer if used,
and a single TX block copied into the existing private TX buffer.
Disconnect PPI routes before SPIReleasePeiTransfer(). The nRF52832
PeiSpiDemo exercises BUT1 -> SPIM2 START, END -> STOP and LED1,
STOPPED -> RGB blue, without any slave peripheral. Target compilation
and hardware validation remain pending.

### nRF52 TWI/TWIM/TWIS event sources and prepared TWIM task destinations

The existing I2C HAL now exposes the actual legacy TWI byte events,
TWIM EasyDMA transfer events, and TWIS slave READ/WRITE/transfer-state
events through C and C++ PeiDev_t calls. The configured I2C mode
determines which hardware event register may be bound; shared
serial peripheral base addresses are not interchangeable modes.

TWIM STARTRX/STARTTX/STOP/SUSPEND/RESUME are hardware PEI
destinations only after I2CPreparePeiTransfer() reserves the
ordinary I2C busy flag and stages one TX or RX buffer. TX is copied
into the driver's existing private 64-byte buffer, and RX is
caller-owned RAM. The driver configures one seven-bit address and
LASTTX_STOP or LASTRX_STOP hardware shortcut, but no START task.
ERROR can connect directly to STOP over PPI on NACK; no CPU
event-forwarding ISR is introduced. All routes must be disabled
and removed before I2CReleasePeiTransfer(), which stops the bus
and releases DMA pointers and the busy flag.

The driver retains the byte-at-a-time legacy TWI ISR and the
TWIS READ/WRITE interrupt flow that prepares its RX/TX buffers.
Those task destinations are not yet exposed through PEI.
PeiTwiDemo (nRF52832 BLYST-NANO-DK) is an optional no-target
NACK recovery test: BUT1 starts TWIM1 via PPI; ERROR triggers
STOP and LED1, and STOPPED triggers RGB blue. SDA/SCL need
reliable pull-up resistors. MCU build and hardware validation
are pending.

### nRF52 UART/UARTE hardware events and prepared transmit tasks

UART0 legacy bytewise mode exposes CTS, NCTS, RXDRDY, TXDRDY,
ERROR and RXTO as PEI source events. UARTE EasyDMA adds ENDRX,
ENDTX, RXSTARTED, TXSTARTED and TXSTOPPED sources. These are
peripheral event addresses, not CPU callbacks or invented IRQ
routing. The active UART configuration and hardware owner
select the register map. CTS/NCTS require enabled hardware
flow control before binding.

For a UARTE task destination, UARTPreparePeiTx() stages
one short TX block in the original private driver TxDmaMem
at the currently configured baud rate. The normal TX FIFO
must be empty, and bIntMode=false; hardware flow control is
disabled for this initial TX mode. The application then
connects a source event to STARTTX and may fan out ENDTX
to STOPTX and an independent GPIO task. UARTReleasePeiTx()
waits for a started transfer to stop and only then clears
TXD.PTR/MAXCNT for reuse. The normal RX DMA continues to
belong to the driver: STARTRX, STOPRX and FLUSHRX are not
external PEI tasks in this initial integration.

PeiUartDemo on nRF52832 BLYST-NANO-DK exercises BUT1 ->
UARTE0 STARTTX, UARTE0 ENDTX -> STOPTX + discrete LED1,
and TXSTOPPED -> RGB blue at 1 Mbaud, without semihosting,
a receiving UART device or event-forwarding ISR. Hardware
and MCU build acceptance remain pending.

### nRF52 QDEC continuous sampling and PPI groups

QDEC can sample continuously with hardware START/STOP and expose
SAMPLERDY, REPORTRDY, ACCOF, DBLRDY and STOPPED. Its IRQ keeps
control of READCLRACC and accumulated position/error data.
QdecPreparePeiStart() allows a hardware source to start the
already-configured sampling engine.

The PEI PPI-group mechanism can enable and disable existing
owned routes through TASKS_CHG[n].EN/DIS, allowing repeated
arming and conditional routing without CPU intervention for
every event. The PeiGroupDemo illustrates the hardware
arm/fire/disarm mechanism. Group lifecycle checks protect
channel ownership, external task targets and SoftDevice/SDC
resource reservations.

### Deliberately excluded one-shot peripheral wrappers

WDT/TEMP/RNG PEI wrappers and passive POWER/CLOCK PEI
event bindings were removed after a use-case audit. Watchdog
TIMEOUT already causes reset; TEMP one-shot START/DATARDY
still needs software data processing; RNG START/VALRDY/STOP
does not export entropy; and passive POWER/CLOCK notifications
do not demonstrate useful recurring automation. Normal
watchdog, temperature, entropy and platform power/clock
drivers remain. The TEMP INTENCLR/DATARDY correctness fix
is retained independently of PEI.

PDM, I2S, SPI/SPIM, TWI/TWIM and UART/UARTE PEI code is
provisional until a repeatable hardware-driven sequence,
including safe DMA buffer reuse and recovery, is verified.
A single button-driven transfer or STOP event is insufficient
for acceptance in release 0.14.

## nRF52 PPI driver (0.14)

`ARM/Nordic/nRF52/src/pei_nrf52.cpp` implements the initial generic PEI
functions through programmable PPI channels. The nRF52805 has 10
configurable channels; nRF52810/nRF52832/nRF52840 have 20. Channels 20-31
have fixed endpoints and are never allocated by this implementation.

Channel and group counts are derived from the MCU device-header PPI register
arrays, rather than legacy PPI_CH_NUM/PPI_GROUP_NUM macros. S132/S140
application availability is computed by excluding
NRF_SOC_SD_PPI_CHANNELS_SD_ENABLED_MSK, which the current SoftDevice headers
provide.

Connection endpoints are addresses of peripheral event and task registers,
for example `&NRF_TIMER2->EVENTS_COMPARE[0]` and
`&NRF_GPIOTE->TASKS_OUT[0]`. The connection owns only PPI configuration:
peripheral initialization, operating frequency, input/output pin assignment,
and the event/task functions remain with those peripheral drivers.

Allocation uses a static table and an opaque per-connection token.
Existing enabled channels, programmed EEP/TEP/FORK endpoints, and channels
present in any PPI group are left untouched. Channels 0 and 1 are excluded
because the repository's Nordic Direct Test Mode code uses them directly.
S132/S140 and SDC build profiles also exclude their reserved channels,
including channels 17-19. The library does not wait for Bluetooth to start
before reserving those channels. Application code cannot increase the free
pool by defining additional macros during its own compilation.

The SoftDevice build uses `sd_ppi_channel_assign()` and
`sd_ppi_channel_enable_set()`/`clr()`; those calls are made with interrupts
restored. SDC and bare-metal builds configure application-owned PPI registers
directly. No ISR forwards an event. The nRF52 driver supports fan-out and
fan-in by allocating separate channels for each connection. A connection
object must retain its address throughout its lifetime, and callers must
serialize operations on that same object.

The nRF52 `Options` field must be zero. Only peripheral event registers
(offset `0x100..0x1FC`) and task registers (offset `0x000..0x0FC`) in the
nRF52 peripheral memory region are accepted. A valid register-range check
cannot prove a particular register is implemented: callers must supply
real peripheral event/task endpoints for their selected MCU.

Run `python3 tests/pei/run.py` for host model validation and
`python3 tests/pei/check_nrf52_cmsis.py` to syntax-check the driver with
the currently installed Nordic CMSIS, S132/S140 and SDC headers.
The latter test does not link firmware or use nrfx HAL drivers. The
`PeiBlinky` nRF52832 IOcomposer example is intended for
BLYST-NANO-DK hardware validation. Its hardware results are not recorded yet.


### nRF52 software events on EGU

`pei_nrf52.cpp` allocates software events on EGU channels: 16 per
instance, EGU0-EGU1 on nRF52805/810/811, EGU0-EGU5 on the other parts.
The event endpoint is `EVENTS_TRIGGERED[n]`, `PeiTriggerSwEvent` writes
`TASKS_TRIGGER[n]`, which is also the endpoint `PeiGetSwEventTask` returns.
PEI never sets the EGU `INTEN`, so the shared SWIn_EGUn interrupt is not
raised; software interrupt users that only pend the SWI in the NVIC are not
affected. A channel whose `INTEN` bit is already set by another user is
skipped.

Reserved instances: with a SoftDevice, EGU1 (radio notification), EGU2
(SoftDevice events), EGU4 and EGU5 (SoftDevice); in SDC builds EGU5, the
MPSL low priority interrupt (`nrf_mpsl.h`). That leaves EGU0 and EGU3 in
SoftDevice builds and EGU0 alone on a SoftDevice nRF52805/810.
`NRF_EGUS_USED` reserves more instances at library build time, bit n for
EGUn. The interrupt-delivering direction (a hardware event into an EGU
channel with its interrupt enabled, to call application code) is not
provided.

## nRF5340 DPPI driver (0.14)

`ARM/Nordic/nRF53/src/pei_nrf53.c` implements the same `PeiConnect`,
`PeiEnable`, `PeiDisable`, and `PeiDisconnect` API on the nRF5340
application and network cores. `DevNo` and `Options` must be zero.

On nRF53, a peripheral event's `PUBLISH_*` register and a peripheral task's
`SUBSCRIBE_*` register are each at offset +0x80 from the event/task
register. This backend accepts event offsets 0x100..0x17C and task offsets
0x000..0x07C within the local peripheral address space. The caller must
supply actual implemented event/task registers; checking the address range
alone cannot establish whether an endpoint exists.

DPPI supports a single publisher to multiple subscribers on one channel.
Connections sharing one event share the physical channel, while retaining
independent enable/disable state. Disabling one subscriber does not interrupt
other subscribers. A task has only one subscription register: this backend
rejects attempting to connect two distinct events to the same task.

Each core has 32 channels; the driver supports up to 32 simultaneous logical
connections. The allocator excludes enabled channels, groups containing
channels, and channels specified by `NRF_DPPI_CHANNELS_USED` or
`NRFX_DPPI_CHANNELS_USED`. Disabled channels allocated by other drivers
**must** be reserved via these build-time masks. The driver never
reconfigures peripheral event/task producers and consumers.

The application core uses its secure or nonsecure DPPIC according to
`NRF_TRUSTZONE_NONSECURE`; the network core uses its DPPIC. Routes crossing
security domains are rejected. No software forwarding or ISR is used.

Run `python3 tests/pei/run_nrf53.py` for host-model validation of the
application core in both security modes and the network core. These tests
do not replace an nRF5340 MCU build or on-board timer-to-GPIOTE validation.

## Standalone nRF52832 example configuration

`PeiBlinky` is a bare-metal application, but the nRF52832 HAL's
standard Debug and Release library configurations are built for S132.
To avoid a SoftDevice SVC fault, its Eclipse managed project compiles the
production `ARM/Nordic/nRF52/src/pei_nrf52.cpp` as an application source,
without `SOFTDEVICE_PRESENT`. The linked HAL archive continues to provide
other drivers. This does not change the SoftDevice-aware PEI backend used
by Bluetooth applications and is not a general runtime detection solution.

### Hardware start/stop through button events

The BLYST-NANO-DK `PeiBlinky` example also exercises peripheral-to-
peripheral control in both directions. The GPIO driver prepares falling-
edge GPIOTE events on BUT1 (P0.02) and BUT2 (P0.13) using the C
`IOPinEnableEvent` API. The Timer driver
exposes its START/STOP hardware tasks. With `DevNo = 6`, PEI routes
BUT1 to TIMER3.START and BUT2 to TIMER3.STOP. TIMER3.COMPARE fans out
through PEI to two GPIOTE toggle tasks with opposite initial LED states.
The TIMER3 configuration requires a fresh hardware validation.

The CPU configures these relationships once, executes the timer STOP
task for initial state, and then waits. Subsequent button events operate
the timer directly in hardware. Unlike `Timer::Disable()`, the initial
STOP task leaves the timer clock enabled, allowing a hardware START event
to resume counting. The two buttons require no GPIO interrupts, polling
or DMA. The GPIO driver reserves input-event and output-task channels in
its shared GPIOTE ownership pool. `IOPinEnableEvent`,
`IOPinDisableEvent`, `IOPinEnableTask` and `IOPinDisableTask`
are implemented in C and use `PeiDev_t *`. The existing C Timer
functions are extended with `TimerEnablePeiTrigger`,
`TimerConnectTask` and `TimerExecuteTask`. Their C++ counterparts
delegate to the same C functions and matrix. There is no public
`IOPinOpenEvent` layer or GPIOTE channel number.

Hardware observation (November 7, 2026): the device tester reports
the two-button, two-LED `PeiBlinky` works on BLYST-NANO-DK (nRF52832).
BUT1 starts the alternating LEDs, BUT2 stops/freezes them, and BUT1
resumes the Timer. This is a **functional hardware PASS** for the
button-to-Timer START/STOP routes and one-to-two compare fan-out.
No debugger IRQ trace or independent timing measurement has been
reported for this configuration.

The shared `exemples/pei/pei_blinky.cpp` contains no MCU register
access or platform adapter. It uses the existing Timer object and the
new `Timer::EnableTimerTrigger(Pei&, PeiEvent_t&, ...)` overload to
configure a compare source and bind it in PEI. The GPIO
`IOPinEnableTask` wrapper configures a task using its GPIO backend
and registers the endpoint relationship in PEI.

On nRF52, every CC compares against the free-running Timer counter.
The Timer ISR advances only the matched CC for continuous scheduling
and disarms a single-shot event without stopping the counter. PEI
routes the hardware compare event directly to GPIOTE, not through the
ISR. Ordinary CC interrupts and PEI compare channels can coexist.
The revised single-shot and continuous paths need a clean MCU build
and hardware validation. `Timer::DisableTimerTrigger(TrigNo)` unbinds
only that CC source. GPIO task slots share allocation with GPIOTE
interrupts.

PEI itself still configures only the interconnect. The updated HAL and example have been compiled and the LED has been
observed flashing at approximately 1 Hz on BLYST-NANO-DK before the
free-running CC scheduling correction. Hardware revalidation, expected
Timer compare IRQ activity and routing lifecycle checks remain pending.
