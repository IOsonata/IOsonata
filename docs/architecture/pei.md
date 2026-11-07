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

`include/coredev/pei.h` defines the C interface and a C++ wrapper. Each
`PeiConn_t` or `Pei` instance owns **one logical event-to-task connection**.
Multiple connection instances can be used to form a one-to-many or many-to-one
routing graph if the MCU supports those connections. Application code never
chooses a PPI, DPPI or EVSYS channel directly.

- `PeiConnect` reserves resources and connects one event to one task; the
  connection is initially disabled.
- `PeiEnable` and `PeiDisable` switch the connection without reallocating it.
- `PeiDisconnect` disables the connection and releases its resources.
- A connection state starts with `Handle == 0`; successful allocation assigns
  an opaque handle. The target implementation clears it on disconnect.
- Repeated enable/disable and disconnect operations are safe. Connecting an
  already-connected object fails without affecting its current connection.

`PeiCfg_t` identifies the interconnect instance (`DevNo`), one event source,
one task destination, and optional target-specific settings (`Options == 0`
selects the default). `PeiEndpoint_t` is an opaque endpoint identifier. On one
MCU it may represent a peripheral register address, on another an event
number or hardware user ID. The MCU implementation validates the identifiers.

The C++ `Pei` object owns the connection and disconnects it when destroyed;
it uses the same C functions as applications written in C. Connection storage
is caller-owned. Hardware resource tables must be statically allocated.

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

The generic interface, nRF52 PPI, and nRF5340 DPPI drivers are committed.
Host-model tests exercise lifecycle and routing for both backends. Full
MCU builds, cross-compilation against the installed nRF53 MDK, and on-board
validation remain pending.


## nRF52 PPI driver (0.14)

`ARM/Nordic/nRF52/src/pei_nrf52.c` implements the initial generic PEI
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
`PeiTimerGpiote` nRF52832 IOcomposer example is intended for
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

`PeiTimerGpiote` is a bare-metal application, but the nRF52832 HAL's
standard Debug and Release library configurations are built for S132.
To avoid a SoftDevice SVC fault, its Eclipse managed project compiles the
production `ARM/Nordic/nRF52/src/pei_nrf52.c` as an application source,
without `SOFTDEVICE_PRESENT`. The linked HAL archive continues to provide
other drivers. This does not change the SoftDevice-aware PEI backend used
by Bluetooth applications and is not a general runtime detection solution.

GPIO initialization now uses `IOPinConfig` and `IOPinSet`. TIMER2 compare
and GPIOTE task-mode configuration are still native register operations,
which should move into the timer and pin drivers. The generic PEI routing
API should not acquire responsibility for peripheral initialization.
