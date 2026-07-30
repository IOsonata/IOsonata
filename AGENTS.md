# IOsonata Repository Work Guide

This file is required reading before reviewing or changing IOsonata.

IOsonata is not a collection of independent drivers. Its design is visible only
when the base classes, generic implementation, MCU ports, and application
examples are read together. Do not infer architecture from one file, one target,
or one MCU vendor.

## Required preflight

Before changing a subsystem:

1. Read its public generic header.
2. Read its complete generic implementation.
3. Search for every target implementation and every public symbol definition.
4. Read every target implementation affected by the change.
5. For shared behavior, read representative implementations from every MCU
   architecture or vendor family where the subsystem exists.
6. Read at least one working example and the relevant tests.
7. Check recent merged work touching the same subsystem.
8. State which existing pattern the change follows.

For changes to `Device`, `DeviceIntrf`, common configuration structures, return
semantics, lifecycle, interrupt behavior, DMA behavior, or asynchronous behavior,
a Nordic-only review is never sufficient. At minimum, search and inspect the
available implementations across:

- Nordic Arm ports;
- ST STM32 ports;
- Renesas Arm and RISC-V ports;
- Microchip SAM ports;
- NXP LPC ports;
- RISC-V Espressif ports;
- host ports when the changed API is implemented there.

Do not change a shared base API without explicit authorization.

Hardware build and test output from the maintainer is authoritative.

## Port status is part of the review

Not every port has the same maturity or supported operating modes. During the
preflight, classify each relevant port as one of:

- current and complete for the behavior being studied;
- intentionally limited, such as polling-only or master-only;
- incomplete, with stubs or TODO paths;
- stale, using older aliases, fields, or copied implementation code.

A limited or stale port is not the architectural reference. It still matters for
source compatibility and for understanding what a shared change may break.
Never generalize framework behavior from an unfinished port, and never ignore an
unfinished port when changing a common header or function signature.

## Core object model

### DeviceIntrf: the transport root

`DeviceIntrf` represents a means of transferring data. It is not limited to a
physical bus.

Physical implementations include:

- UART;
- I2C;
- SPI, QSPI, and OSPI;
- USB;
- network transports.

Software and internal implementations can also be `DeviceIntrf` objects:

- `Slip`, which layers framing over another `DeviceIntrf`;
- `BtIntrf`, which presents a GATT service as a serial-style interface;
- internal controller access paths such as `CracenIntrf` and `NvmIntrf`.

The generic transfer shape is:

```text
StartRx(selector) -> RxData -> StopRx
StartTx(selector) -> TxData -> StopTx
Read              -> command/address phase -> receive phase
Write             -> command/address phase -> transmit phase
```

`StartRx` and `StartTx` open a transfer and pass a target-defined selector. They
are not universally physical bus START conditions. `StopRx` and `StopTx` close
the corresponding transfer and release its interface-level serialization.

The selector meaning depends on the interface:

- SPI uses it as a zero-based chip-select index;
- I2C uses it as the 7-bit device address;
- CRACEN uses it to select the PKE register block, PKE operand memory,
  CryptoMaster, or RNG engine;
- UART normally ignores it because the stream endpoint is already selected by
  the UART instance;
- another interface may define another selector meaning.

The start and stop hooks also perform interface-specific work. SPI may assert and
deassert chip select. I2C may generate bus START, repeated START, and STOP
conditions. CRACEN selects an engine or memory window and clears the transfer
selection when the operation closes. UART implementations commonly use no-op
start and stop hooks.

`DevIntrf_t::bBusy` protects one transfer. The framework acquires it in
`DeviceIntrfStartRx()` or `DeviceIntrfStartTx()` and releases it in the matching
stop helper. An implementation hook must not independently take or clear that
flag.

`DevIntrf_t::EnCnt` is the shared-interface enable reference count. The physical
interface is enabled on the 0 to 1 transition and disabled on the last release.
Older ports may initialize or use this field differently; inspect them before
changing lifecycle behavior.

### Polling, interrupt, and DMA use

Execution mode is selected case by case. It is not a fixed property of UART,
I2C, SPI, master mode, or slave mode.

UART is normally interrupt driven because its main use is continuous streaming.
RX data can arrive at any time, and blocking until an entire stream is complete
is not useful. UART ports therefore commonly use interrupt-driven CFIFO
producers and consumers, with optional DMA for larger TX bursts.

I2C and SPI master operations are commonly short, bounded transactions. Polling
is often the simplest and fastest implementation because interrupt setup,
context switching, and completion handling can cost more than the transfer.
Interrupt or DMA operation is still appropriate for long transfers, strict CPU
latency requirements, concurrent work, or controller-specific reasons.

I2C and SPI slave operation usually needs interrupts because the external master
controls when the transaction starts and advances. The slave cannot choose a
convenient polling window without risking lost protocol events.

The actual device, transfer length, direction, controller, power requirement,
latency requirement, and application usage determine the mode. Do not infer it
from the interface type alone, and do not add a generic rule that all master
transfers are polling or all enabled interrupt fields imply asynchronous
completion.

Return behavior also depends on the interface implementation:

- a synchronous direct transfer normally returns the number of bytes moved;
- a direct interrupt or DMA transfer may return `-1` to indicate that completion
  will arrive later through `DevIntrf_t::EvtCB`;
- a FIFO-backed UART `TxData()` commonly returns the number of bytes accepted
  into the software FIFO while the ISR or DMA engine drains the FIFO later;
- a positive UART return therefore does not mean every byte is already on the
  wire.

Some target ports are intentionally polling-only. Some expose interrupt fields
but have incomplete interrupt or DMA paths. `bIntEn` alone does not prove that a
specific asynchronous completion pattern is implemented.

Before changing polling, interrupt, DMA, or completion behavior, read:

- `src/device_intrf.cpp`;
- the complete target implementation for the interface being changed;
- representative UART, I2C, and SPI ports from other MCU families;
- the actual device driver and usage pattern that depend on the transfer.

A device operation may span multiple interface transfers. Such a device needs
its own operation state in addition to the interface's per-transfer busy flag.
That state belongs in the device implementation; it does not replace or bypass
`DeviceIntrf`.

### Device: the device tree

`Device` represents a device, hardware block, or software engine using a
`DeviceIntrf`.

It provides the common device shape:

- `Enable()`;
- `Disable()`;
- `Reset()`;
- `PowerOff()` where applicable;
- `Valid()`;
- device address and device ID;
- injected `DeviceIntrf *`;
- device event callback;
- optional timer integration.

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

Older C ports are also part of the compatibility surface. Do not assume that all
ports use the newest typedef names or initialize every recently added field.

## Generic and target separation

IOsonata is compiled as a separate library for each MCU or MCU configuration.
The generic layer defines the portable object and behavior. The target layer
implements controller access, interrupts, DMA, arbitration, clocking, pin
routing, and MCU-specific limits.

Do not put a target transfer limit into `DeviceIntrf` merely because one target
has that limit. Chunking belongs in the target interface implementation unless
the peripheral device itself defines the boundary.

Do not review a generic layer without reading the target ports. Do not review a
target port without reading the generic caller. For a common API change, search
all implementations before proposing the change.

A public symbol must have one definer in each library configuration. Weak
fallbacks must fail closed where unsupported behavior would be unsafe.

Different MCU ports can divide responsibilities differently. For example, an
Espressif UART port may delegate clock reset, GPIO matrix routing, and interrupt
matrix setup to separate target modules, while another MCU keeps those operations
in the UART source file. Read all cooperating target files before judging the
implementation incomplete.

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

FreeRTOS and TaktOS are orthogonal to the IOsonata object hierarchy. Do not make
RTOS classes derive from `DeviceIntrf`. Bare-metal and RTOS examples should use
the same IOsonata interfaces.

## MCU-port reading matrix

For shared interface or architecture work, use a matrix rather than a single
reference port.

### Nordic

Read the applicable nRF52, nRF54, or nRF91 implementation, including shared
peripheral arbitration and SoftDevice or stack integration where relevant.
Nordic ports provide examples of polling, interrupt, and DMA operation, but the
selected mode must be evaluated against the actual device and transfer usage.

### ST STM32

Read both an ordinary peripheral port and specialized roots where present:

- UART;
- I2C;
- SPI;
- QSPI;
- OSPI.

Do not assume that the STM32 interrupt or DMA path is complete merely because
configuration fields exist. Check the transfer functions and IRQ handler.

### Renesas

Read the available Arm and RISC-V implementations. Renesas UART ports show MCU
families where FIFO and non-FIFO instances share one generic UART object but use
different register and interrupt handling.

### Microchip

Read SAM4E and SAM4L implementations as applicable. They show controller-specific
DMA engines, separate RX and TX completion paths, repeated-start command support,
and ports with partially implemented interrupt modes.

### NXP

Read both the MCU-specific initializer and any shared LPC implementation file.
Several LPC ports are older C implementations and are useful for compatibility,
but should not be treated as the only reference for current framework behavior.

### RISC-V Espressif

Read the peripheral source together with its clock, reset, pin-matrix, and
interrupt-routing modules. Verify that the file is an actual Espressif
implementation rather than an unfinished copied port before using it as a design
reference.

## Layering examples to study

### UART

Read together:

- `include/coredev/uart.h`;
- the complete target UART implementation;
- its target support modules where clock, pin routing, DMA, or IRQ setup is
  delegated;
- `exemples/uart/uart_prbs_tx.cpp`;
- `exemples/uart/uart_prbs_tx_freertos.cpp`;
- `exemples/uart/uart_prbs_tx_taktos.cpp` when scheduler interaction matters.

UART shows the C/C++ dual API, static CFIFO storage, interrupt-driven producers
and consumers, DMA buffering, and applications using the same interface under
different schedulers.

### I2C and SPI

Read together:

- `include/coredev/i2c.h`;
- `include/coredev/spi.h`;
- all target implementations affected by the change;
- at least one implementation from each MCU family where common behavior is
  being changed;
- a device example that can use either interface.

These ports show polling master transfers, interrupt-driven slave operation,
DMA chunking, repeated-start handling, device selection, specialized QSPI/OSPI
command phases, and completion events. They also show that the selected execution
mode is determined by the controller and usage, not just by the bus name.

### Layered interfaces

Read together:

- `include/slip_intrf.h`;
- `src/slip_intrf.cpp`;
- the UART SLIP PRBS examples.

`Slip` is itself a `DeviceIntrf` and delegates to an injected physical
`DeviceIntrf`. This is protocol layering, not inheritance from UART.

### Bluetooth

Read together:

- `include/bluetooth/bt_intrf.h`;
- `src/bluetooth/bt_intrf.cpp`;
- the complete generic Bluetooth stack layer;
- every relevant target port;
- `exemples/bluetooth/bleintrf_prbs_tx.cpp`.

`BtIntrf` exposes a GATT service through the normal `DeviceIntrf` operations.
Its FIFOs, GATT callbacks, application event queue, and target stack integration
must be considered together.

### Sensors

Read together:

- `include/sensors/sensor.h`;
- the capability bases used by the concrete sensor;
- the complete concrete driver;
- `exemples/sensor/env_tph_demo.cpp`.

Sensors demonstrate `Device` inheritance, capability interfaces, injected
transport, optional timer operation, lifecycle, and one object implementing
multiple sensor views.

### Crypto

Crypto engines are devices, not transports. Read the complete generic crypto
object tree, provider selection, each target provider, Bluetooth security
callers, and hardware tests before changing it.

The BA414EP and CryptoMaster drivers demonstrate that `DeviceIntrf` selectors are
not limited to bus addresses. `CracenIntrf::StartTx()` and `StartRx()` use
`DevAddr` to select the PKE register block, PKE operand memory, CryptoMaster, or
RNG path. BA414EP and CryptoMaster recovery open a transfer on their selector,
call the address-selected interface reset, and close the transfer. The start and
stop pair both serializes the shared CRACEN access and makes the selected engine
explicit.

Software capability bases are working implementations. Hardware providers
replace only supported operations. Operation storage is static or
caller-provided, secrets are wiped, and security failures must fail closed.

### Storage

Read together:

- `include/storage/nvm.h`;
- `src/storage/nvm.cpp`;
- `include/storage/nvm_intrf.h`;
- every target `nvm_*.cpp`;
- the I2C, SPI, QSPI, and OSPI ports that may be injected into `Nvm`;
- `include/storage/diskio_nvm.h`;
- storage tests and hardware demos.

`Nvm` is a `Device` using an injected `DeviceIntrf`. External SPI/I2C memories
and internal memory adapters share the generic NVM object, while controller
arbitration remains in the target interface.

For NVM interrupt or DMA work, preserve the normal `DeviceIntrf` behavior of the
injected interface and add NVM operation state only when a device operation spans
multiple interface transfers. Decide polling, interrupt, or DMA operation from
the actual memory, transfer length, controller, and application requirements.
Do not remove an existing configuration capability merely because the current
NVM implementation uses polling.

## Repository work rules

- Work only on the requested branch.
- Verify the branch head before editing.
- Do not create GitHub workflows unless explicitly requested.
- Keep commits focused.
- Do not rewrite unrelated code while fixing a subsystem.
- Preserve public APIs unless a change is explicitly authorized.
- Read complete implementations, not selected snippets.
- Search all target definitions before changing shared behavior.
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

## Documentation precedence

Current repository documentation and source define the present architecture,
IOcomposer workflow, project layout and implementation behavior.

`docs/beyond_blinky_free_edition.md` is a machine-readable snapshot of the
December 2025 publication. It remains useful for the object-design model and
historical background, but current repository documentation takes precedence
where tools, procedures, project organization or implementation details differ.
Human readers should use https://leanpub.com/beyondblinky for the free or
complete edition.

## Required first reading for architecture work

1. `README.md`.
2. `docs/architecture/README.md`.
3. `docs/architecture/iocomposer-workflow.md`.
4. `docs/architecture/devintrf-implementer-notes.md`.
5. `include/device_intrf.h`.
6. `src/device_intrf.cpp`.
7. `include/device.h`.
8. `src/device.cpp`.
9. `docs/beyond_blinky_free_edition.md` for the December 2025 object-design
   background, not as current workflow authority.
10. At least three complete subsystem sets from the sections above.
11. For shared interface work, the cross-MCU port matrix for that interface.
12. The full subsystem being changed, including generic code, target ports,
    examples, tests, and recent merged work.

Do not propose a design change until this reading is complete.
