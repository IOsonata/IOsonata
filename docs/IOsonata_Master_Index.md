# IOsonata Documentation Index

This index points to the current IOsonata architecture, workflow and implementation references.

## Start here

1. [README](../README.md) — project scope, precompiled-library model, benchmarks and supported execution models.
2. [Getting started](getting-started.md) — install IOcomposer, build an MCU library, open an example, flash and debug.
3. [FAQ](FAQ.md) — concise answers about architecture, project layout and the build model.
4. [Quick reference](quick-reference.md) — commands and common project tasks.
5. [Supported targets](supported-targets.md) — current MCU and validation status.

## Architecture

### [Architecture overview](architecture/README.md)

Canonical definition of:

- the MCU-centric precompiled-library model;
- application-owned board configuration;
- `Device` and `DeviceIntrf`;
- generic and target separation;
- linker-selected code;
- static-memory and real-time rules;
- bare-metal and RTOS independence.

### [IOcomposer workflow](architecture/iocomposer-workflow.md)

Official build and project workflow:

- install IOcomposer;
- build one MCU target or all targets;
- produce Debug and Release libraries;
- create or open applications;
- link the precompiled library;
- update IOsonata without creating a product build matrix.

### [DeviceIntrf implementer notes](architecture/devintrf-implementer-notes.md)

Rules for implementing a new transferable data path:

- `DevIntrf_t` state and function table;
- wrapper ownership of `bBusy`;
- command/read and repeated-start sequencing;
- polling, FIFO, interrupt and DMA return behaviour;
- event context;
- enable reference counting;
- target-owned chunking;
- device-operation state above one transfer.

### [Device inheritance and composition](architecture/device-composition.md)

Rules for:

- behavioural inheritance;
- runtime polymorphism;
- shared virtual `Device` state;
- multi-function physical devices;
- independent devices in one package;
- configuration-driven variation;
- object composition and adapters.

## Development workflow

### [Dependencies](dependencies.md)

Installed tools, target SDK boundaries, optional application libraries, update rules and offline use.

### [Quick reference](quick-reference.md)

Common IOcomposer actions, board configuration, interface setup, examples, updates and troubleshooting.

### [Getting started](getting-started.md)

Recommended first run using nRF52832, Blinky and a supported debug probe.

## Core source references

### Object foundation

- `include/device_intrf.h`
- `src/device_intrf.cpp`
- `include/device.h`
- `src/device.cpp`

### Core interfaces

- `include/coredev/uart.h`
- `include/coredev/i2c.h`
- `include/coredev/spi.h`
- `include/coredev/timer.h`
- `include/slip_intrf.h`
- `include/bluetooth/bt_intrf.h`

### Device families

- `include/sensors/`
- `include/display/`
- `include/storage/`
- `include/crypto/`
- `include/bluetooth/`

### Generic implementations

- `src/coredev/`
- `src/storage/`
- `src/crypto/`
- `src/bluetooth/`
- `src/sensors/`

### Target ports

- `ARM/`
- `RISCV/`
- `Win/`
- `Linux/`

A subsystem review must include its generic header, generic implementation, all affected target ports, at least one working example and the recent merged changes touching it.

## Examples

Reusable source lives under `exemples/`. Buildable applications live under each MCU target.

Useful entry points include:

- `exemples/misc/blinky.c`;
- `exemples/uart/uart_retarget_demo.cpp`;
- `exemples/uart/uart_prbs_tx.cpp`;
- `exemples/uart/uart_prbs_tx_freertos.cpp`;
- `exemples/uart/uart_prbs_tx_taktos.cpp`;
- `exemples/i2c/i2c_master_demo.cpp`;
- `exemples/spi/spi_master_demo.cpp`;
- `exemples/storage/`;
- `exemples/bluetooth/`;
- `exemples/sensor/`.

## Learning the object model

**Beyond Blinky: Object-Oriented C++ Programming — Fun, Fast, and Fearless Embedded Development** explains the orchard model, `DeviceIntrf`, `Device`, device families, runtime polymorphism, MCU porting and measured performance.

- [Download the free edition or buy the complete book on Leanpub](https://leanpub.com/beyondblinky)
- [Amazon](https://www.amazon.com/Beyond-Blinky-Object-Oriented-Programming-Development/dp/1069933511)
- [Machine-readable December 2025 publication snapshot](beyond_blinky_free_edition.md) — retained for repository search and RAG indexing

Current repository architecture and workflow documents take precedence over the December 2025 publication snapshot when procedures or project organization differ.

## Documentation rules

New documentation must preserve these architectural statements:

- IOcomposer is the official IDE.
- IOsonata is built as a reusable static library per MCU target and build profile.
- Board and product configuration do not rebuild the MCU library.
- Bare metal, TaktOS, FreeRTOS and ThreadX can link the same selected library binary.
- `DeviceIntrf` models data transfer; `Device` models device behaviour.
- Target limits stay in target ports; device protocol limits stay in device drivers.
- Static or caller-owned memory is preferred in driver and real-time paths.
- Examples must use real repository paths and current public APIs.
- Published book snapshots must be dated and must not be presented as current workflow authority.

Avoid documenting generated build files, internal project-format names or historical workflows as user-facing architecture.
