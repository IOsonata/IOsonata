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
   and no-ISR timer-to-GPIOTE validation on nRF52832/BLYST-NANO-DK.
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

`TimerInit` does not install PEI trigger callbacks. The first explicit
PEI Timer call invokes `PeiTimerPrepare`, which installs the applicable
MCU callback through `TimerPeiPrepareMcu`. Non-PEI Timer applications
do not reference this optional path. HF Timer and RTC PEI compare trigger ownership reuse existing
`TimerTrig_t` and CC bookkeeping fields. The optional RTC private
PPI handles and TICK/OVRFLW binding records are referenced exclusively
from PEI callbacks, allowing the linker to discard all their RAM in
non-PEI applications. Normal RTC methods check PEI ownership indirectly
through the optional Timer callback, retaining no PEI-specific tables.
Hardware-only triggers still release PEI routes through
`DisableTimerTrigger`.

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

The nRF52 RTC Timer driver now exposes TICK/OVRFLW event sources
with independent EVTEN control, and START/STOP/CLEAR/TRIGOVRFLW task
endpoints through the existing C Timer interface. RTC compare
supports both hardware-only single-shot and continuous operation:
the driver requests one private PPI route from COMPARE to STOP or
CLEAR respectively. The PEI backend owns physical channel assignment;
the Timer driver owns the RTC compare and the lifetime of its route.

RTC compare mode requires exclusive RTC counter ownership and rejects
conflicting trigger or raw-event users. Single-shot stops the counter;
continuous mode periodically clears it without an ISR. These additions
have **not** yet been MCU compiled or tested on hardware. The
BLYST-NANO-DK PeiBlinky now selects `DevNo = 6` (TIMER3 at 16 MHz,
with `Freq = 0`) for the standalone PEI test. The RTC backend remains
unvalidated. The TIMER3 selection also requires a fresh MCU build and
hardware test; Bluetooth coexistence needs separate resource checks.

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
configure a hardware-only compare source and bind it in PEI. The GPIO
`IOPinEnableTask` wrapper configures a task using its GPIO backend
and registers the endpoint relationship in PEI.

On nRF52, the Timer driver uses hardware COMPARE->CLEAR for continuous
triggers and COMPARE->STOP for single-shot triggers, without a Timer ISR.
Both shortcuts affect the entire counter and require exclusive compare
ownership; conflicting ISR compare and shortcut configurations are refused.
The single-shot path still needs MCU compilation and hardware validation.
`Timer::DisableTimerTrigger(TrigNo)` unbinds PEI before freeing its
compare slot. GPIO task slots share allocation with GPIOTE interrupts.

PEI itself still configures only the interconnect. The updated HAL and example have been compiled and the LED has been
observed flashing at approximately 1 Hz on BLYST-NANO-DK. IRQ absence
and routing lifecycle checks remain to be verified.
