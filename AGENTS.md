# IOsonata Repository Work Guide

This file is required reading before reviewing or changing IOsonata.

IOsonata is not a collection of independent drivers. Its design is visible only
when the base classes, generic implementation, target port, and application
example are read together. Do not infer architecture from one file.

## Required preflight

Before changing a subsystem:

1. Read its public generic header.
2. Read its complete generic implementation.
3. Read every target implementation affected by the change.
4. Read at least one working example and the relevant tests.
5. Check recent merged work touching the same subsystem.
6. State which existing pattern the change follows.

For a base API change, also inspect every implementation and caller. Do not
change a shared base API without explicit authorization.

Hardware build and test output from the maintainer is authoritative.

## Core object model

### DeviceIntrf: the transport root

`DeviceIntrf` represents a means of transferring data. It is not limited to a
physical bus.

Physical implementations include:

- UART
- I2C
- SPI, QSPI, and OSPI
- USB
- network transports

Software and protocol implementations can also be `DeviceIntrf` objects:

- `Slip`, which layers framing over another `DeviceIntrf`
- `BtIntrf`, which presents a GATT service as a serial-style interface
- internal controller adapters such as `NvmIntrf`

The common transfer sequence is:

```text
StartRx -> RxData -> StopRx
StartTx -> TxData -> StopTx
Read    -> command/address phase -> receive phase
Write   -> command/address phase -> transmit phase
```

`DevIntrf_t::bBusy` protects one transfer. The framework acquires it in
`DeviceIntrfStartRx()` or `DeviceIntrfStartTx()` and releases it in the matching
stop helper. An implementation hook must not independently take or clear that
flag.

`DevIntrf_t::EnCnt` is the shared-interface enable reference count. The physical
interface is enabled on the 0 to 1 transition and disabled on the last release.

### Synchronous and asynchronous transfer

IOsonata supports both modes in the same `DeviceIntrf` API.

In synchronous mode, `RxData()` and `TxData()` return the number of bytes moved.
The generic helper then executes the matching stop phase.

In asynchronous mode, `RxData()` or `TxData()` may return `-1` to indicate that
the transfer has started and completion will arrive later. The target ISR or
stack callback completes the transfer and emits the appropriate
`DEVINTRF_EVT_*` event through `DevIntrf_t::EvtCB`.

Do not mistake interface-level asynchronous completion for a missing feature.
Study `src/device_intrf.cpp`, `ARM/Nordic/src/spi_nrfx.cpp`, and
`ARM/Nordic/src/i2c_nrfx.cpp` before changing asynchronous behavior.

A device operation may span multiple interface transfers. Such a device needs
its own operation state in addition to the interface's per-transfer busy flag.
That state belongs in the device implementation; it does not replace or bypass
`DeviceIntrf`.

### Device: the device tree

`Device` represents a device, hardware block, or software engine using a
`DeviceIntrf`.

It provides the common device shape:

- `Enable()`
- `Disable()`
- `Reset()`
- `PowerOff()` where applicable
- `Valid()`
- device address and device ID
- injected `DeviceIntrf *`
- device event callback
- optional timer integration

A device driver accepts `DeviceIntrf *`, not a concrete UART, I2C, or SPI type.
The device's identity must not be tied to its transport.

The environmental sensor example demonstrates this directly: the same sensor
object receives either an `I2C` or `SPI` instance through a `DeviceIntrf *` and
the sensor implementation remains unchanged.

Capability families may use virtual inheritance from `Device`. Read the complete
family and concrete implementation before changing inheritance or moving state.

## C and C++ are both first-class

Core interfaces normally have:

- a C data structure containing the implementation state and function table;
- C helper functions operating on that structure;
- a C++ class wrapping the same implementation.

The C++ layer must not create a second unrelated implementation. Examples such
as `exemples/uart/uart_prbs_tx.cpp` show both forms using the same driver.

## Generic and target separation

IOsonata is compiled as a separate library for each MCU or MCU configuration.
The generic layer defines the portable object and behavior. The target layer
implements controller access, interrupts, DMA, arbitration, and MCU-specific
limits.

Do not put a target transfer limit into `DeviceIntrf` merely because one target
has that limit. Chunking belongs in the target interface implementation unless
the peripheral device itself defines the boundary.

Do not review a generic layer without reading the target ports. Do not review a
target port without reading the generic caller.

A public symbol must have one definer in each library configuration. Weak
fallbacks must fail closed where unsupported behavior would be unsafe.

## Configuration and memory rules

Configuration is C/C++ data placed beside the application code. IOsonata does
not use Device Tree, Kconfig, or CMake as its project model.

Board differences normally belong in `board.h` pin maps. MCU support belongs in
the MCU port, not in a board-specific fork.

Driver and real-time paths use static or caller-owned memory:

- no heap allocation;
- no exceptions;
- no RTTI;
- no template-heavy design;
- no hidden allocation;
- no unbounded work in interrupt handlers.

Use fixed storage, caller-provided buffers, CFIFO, and explicit operation state.
Record the minimum fact in interrupt context and defer substantial processing.

FreeRTOS is optional and orthogonal to the IOsonata object hierarchy. Do not make
RTOS classes derive from `DeviceIntrf`. Bare-metal and RTOS examples should use
the same IOsonata interfaces.

## Layering examples to study

### UART

Read together:

- `include/coredev/uart.h`
- `ARM/Nordic/src/uart_nrfx.cpp`
- `exemples/uart/uart_prbs_tx.cpp`
- `exemples/uart/uart_prbs_tx_freertos.cpp`

UART shows the C/C++ dual API, static CFIFO storage, interrupt-driven producers
and consumers, DMA buffering, and an RTOS application using the same interface.

### I2C and SPI

Read together:

- `include/coredev/i2c.h`
- `include/coredev/spi.h`
- the relevant target `i2c_*.cpp` and `spi_*.cpp`
- a device example that can use either interface

These ports show polling and interrupt operation, DMA chunking, repeated-start
handling, device selection, and `DEVINTRF_EVT_COMPLETED` delivery.

### Layered interfaces

Read together:

- `include/slip_intrf.h`
- `src/slip_intrf.cpp`
- the UART SLIP PRBS examples

`Slip` is itself a `DeviceIntrf` and delegates to an injected physical
`DeviceIntrf`. This is protocol layering, not inheritance from UART.

### Bluetooth

Read together:

- `include/bluetooth/bt_intrf.h`
- `src/bluetooth/bt_intrf.cpp`
- the complete generic Bluetooth stack layer
- every relevant target port
- `exemples/bluetooth/bleintrf_prbs_tx.cpp`

`BtIntrf` exposes a GATT service through the normal `DeviceIntrf` operations.
Its FIFOs, GATT callbacks, application event queue, and target stack integration
must be considered together.

### Sensors

Read together:

- `include/sensors/sensor.h`
- the capability bases used by the concrete sensor
- the complete concrete driver
- `exemples/sensor/env_tph_demo.cpp`

Sensors demonstrate `Device` inheritance, capability interfaces, injected
transport, optional timer operation, lifecycle, and one object implementing
multiple sensor views.

### Crypto

Crypto engines are devices, not transports. Read the complete generic crypto
object tree, provider selection, each target provider, Bluetooth security
callers, and hardware tests before changing it.

Software capability bases are working implementations. Hardware providers
replace only supported operations. Operation storage is static or
caller-provided, secrets are wiped, and security failures must fail closed.

### Storage

Read together:

- `include/storage/nvm.h`
- `src/storage/nvm.cpp`
- `include/storage/nvm_intrf.h`
- every target `nvm_*.cpp`
- `include/storage/diskio_nvm.h`
- storage tests and hardware demos

`Nvm` is a `Device` using an injected `DeviceIntrf`. External SPI/I2C memories
and internal memory adapters share the generic NVM object, while controller
arbitration remains in the target interface.

For asynchronous NVM work, preserve the normal `DeviceIntrf` completion path and
add NVM operation state for multi-transfer sequencing. Do not remove the async
configuration merely because the current NVM implementation is polling.

## Repository work rules

- Work only on the requested branch.
- Verify the branch head before editing.
- Do not create GitHub workflows unless explicitly requested.
- Keep commits focused.
- Do not rewrite unrelated code while fixing a subsystem.
- Preserve public APIs unless a change is explicitly authorized.
- Read complete implementations, not selected snippets.
- Validate against examples and hardware behavior.
- Never claim a build or hardware result that was not actually run.

## Writing and code style

Use plain engineering language. Avoid promotional or artificial wording.

- ASCII text in source and project documentation.
- Tabs for source indentation, matching the existing file.
- No template-based replacement for the runtime-polymorphic architecture.
- Avoid large macro frameworks and broad conditional-compilation designs.
- Do not introduce `printf` into library paths for diagnostics; use the existing
  logging mechanism where applicable.
- Follow the naming and C/C++ wrapper pattern already used by the subsystem.

## Required first reading for architecture work

1. `README.md`
2. `docs/beyond_blinky_free_edition.md`, especially the object-model chapters
3. `include/device_intrf.h`
4. `src/device_intrf.cpp`
5. `docs/architecture/devintrf-implementer-notes.md`
6. `include/device.h`
7. `src/device.cpp`
8. At least three subsystem sets from the sections above
9. The full subsystem being changed, including target ports and examples

Do not propose a design change until this reading is complete.