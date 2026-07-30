# IOsonata Architecture

IOsonata is a precompiled, MCU-centric hardware-abstraction and device-driver library.

The application owns its board data, memory layout, startup policy and execution model. IOsonata supplies one reusable static library for each supported MCU target and build profile.

```text
Application sources + board.h + optional kernel
                         |
                         | final link
                         v
              libIOsonata_<MCU>.a
                         |
          generic objects + MCU target port
                         |
                       MCU
```

For a selected Debug or Release profile, bare-metal applications, TaktOS, FreeRTOS and ThreadX link the same IOsonata library binary. IOsonata is not rebuilt for the board, application, peripheral set or operating system.

## Core design rules

### MCU target, not board target

An MCU port contains the supported startup, interrupt, clock, GPIO, communication, timer, storage, Bluetooth, crypto and device implementations for that MCU or MCU configuration.

Board differences are application data. `board.h` normally supplies:

- pin maps;
- oscillator configuration;
- external-device configuration;
- board-level definitions.

Changing that data does not recompile the MCU implementation.

### Compile once, select at link time

Supported implementations are compiled into the static library. The application references the objects it uses. The static linker extracts the required object files, and section garbage collection removes unreferenced code and data.

The application image changes. The IOsonata library artifact does not.

### No feature-build matrix

IOsonata does not generate another HAL binary for each combination of board, RTOS, I2C, SPI, sensor, storage, Bluetooth or security option.

A library rebuild is required when IOsonata source for that MCU changes. It is not required when an application changes its board data, object composition or scheduler.

## Object architecture

IOsonata is organized around two main object families.

### `DeviceIntrf`: how data moves

`DeviceIntrf` represents a transferable data path. It is not limited to a physical bus.

Examples include:

- UART;
- I2C;
- SPI, QSPI and OSPI;
- USB;
- Bluetooth GATT presented through `BtIntrf`;
- SLIP layered over another interface;
- internal crypto and memory-controller access paths.

A device driver depends on `DeviceIntrf`, not on one concrete bus type.

### `Device`: what the object is

`Device` represents a hardware device, software engine or functional device branch. It provides common lifecycle, identity, interface injection, event and timer state.

Derived families add device behaviour such as:

- sensor sampling;
- display control;
- storage access;
- crypto operations;
- filesystem block access.

Inheritance is used for real behavioural families and runtime polymorphism. It is not used merely to encode a different register map, transport or command set.

See [Device inheritance and polymorphic composition](device-composition.md).

## C and C++

C and C++ use the same underlying implementations.

Core interfaces normally provide:

- a C structure containing implementation state and function pointers;
- C helper functions operating on that structure;
- a C++ class wrapping the same implementation.

The C++ layer does not create a second independent driver.

## Generic and target separation

The generic layer defines portable behaviour and object structure. The target layer implements controller access, interrupts, DMA, clocking, pin routing, arbitration and MCU-specific limits.

A target transfer limit belongs in the target interface implementation. A device-defined page, frame or protocol boundary belongs in the device driver. Neither belongs in the generic `DeviceIntrf` API merely because one controller requires it.

## Memory and real-time rules

Driver and real-time paths use static or caller-owned storage.

- No heap is required in core data paths.
- No exceptions or RTTI are required.
- No template-generated device hierarchy is used.
- Interrupt handlers perform bounded work.
- Long operations use explicit state, callbacks or polling.
- Buffers and ownership remain visible to the caller.

## Execution-model independence

IOsonata does not own the scheduler.

```text
bare metal
TaktOS
FreeRTOS
ThreadX
    |
    +---- same selected libIOsonata_<MCU>.a
```

Kernel-specific synchronization remains above the IOsonata hardware and device layer unless a concrete device operation requires its own internal operation state.

## Official development workflow

[IOcomposer](https://iocomposer.io) is the official IDE for IOsonata.

The installed library builder discovers supported MCU library projects, lets the user select one MCU or build all targets, then produces Debug and Release libraries. Applications created or opened in IOcomposer link the selected precompiled library.

See [IOcomposer workflow](iocomposer-workflow.md).

## Architecture documents

- [IOcomposer workflow](iocomposer-workflow.md)
- [DeviceIntrf implementer notes](devintrf-implementer-notes.md)
- [Device inheritance and polymorphic composition](device-composition.md)
- [Getting started](../getting-started.md)
- [Quick reference](../quick-reference.md)
- [Supported targets](../supported-targets.md)
