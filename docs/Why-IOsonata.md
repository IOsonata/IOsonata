# Why IOsonata

IOsonata is built around a different reuse boundary from most embedded frameworks.

The reusable deliverable is not a source configuration that every application rebuilds. It is a precompiled static library for one MCU target and build profile:

```text
libIOsonata_<MCU>.a
```

Debug and Release are separate profiles. Within either selected profile, boards, products, applications and supported execution models using that MCU link the same IOsonata library binary.

## One MCU port, reused unchanged

A supported MCU library contains the target implementation for:

- startup and interrupt vectors;
- clock and system initialization;
- GPIO, UART, I2C, SPI, timers and PWM;
- Bluetooth, storage, NVM and crypto where supported;
- device drivers and the common `Device` and `DeviceIntrf` architecture.

Changing the board, application or scheduler does not create another HAL build.

The application supplies board and product data through `board.h`, including pin maps, oscillator configuration, external-device configuration and other board definitions. These values may change without recompiling IOsonata.

## Compile once, link only what is used

Supported implementations are compiled into the MCU static library once. The application references the objects it needs.

The static linker extracts the required object files, and section garbage collection removes unreferenced functions and data.

```text
same library + application uses I2C and Sensor A
    -> I2C and Sensor A are linked

same library + application uses SPI and Sensor B
    -> SPI and Sensor B are linked

same library + application uses neither
    -> neither appears in the firmware
```

The final firmware image changes. The IOsonata library artifact does not.

## Object design instead of configuration-generated types

IOsonata uses ordinary C++ object-oriented design:

- encapsulation;
- inheritance;
- runtime polymorphism;
- object composition.

It does not generate a different driver type for every application configuration, and it does not require template-heavy peripheral abstractions.

The architecture is organized around two primary object families.

### `DeviceIntrf`: how data moves

`DeviceIntrf` represents a transferable data path such as UART, I2C, SPI, Bluetooth, SLIP, USB or an internal controller interface.

A device driver depends on the interface role instead of one concrete controller type.

### `Device`: what the object does

`Device` provides common lifecycle, identity, callback, timer and interface-association state for sensors, displays, storage devices, crypto engines and other hardware or software devices.

Inheritance represents device families. Runtime polymorphism allows application code to use a common behaviour while selecting a concrete implementation at runtime.

## One device driver, any compatible interface

A device supporting both I2C and SPI does not need separate generated driver types.

```cpp
I2C i2c;
SPI spi;
Timer timer;
AccelLsm303agr accel;

DeviceIntrf *interface = useSpi
    ? static_cast<DeviceIntrf *>(&spi)
    : static_cast<DeviceIntrf *>(&i2c);

accel.Init(accelCfg, interface, &timer);
```

The interface object changes. The sensor-driver implementation does not.

The same principle supports protocol composition. For example, `Slip` can wrap a UART or Bluetooth interface while the application continues to use `DeviceIntrf`.

## C and C++ share the same implementation

Core IOsonata interfaces normally provide:

- a C structure containing state and function pointers;
- C helper functions operating on that structure;
- a C++ class wrapping the same implementation.

The C and C++ APIs do not require two independent drivers.

Low-level operations that do not benefit from object state remain lightweight C/C++ functions. GPIO pin control, for example, is not forced into a class hierarchy.

## The application owns the system

IOsonata does not own:

- `main()`;
- the scheduler;
- the board configuration;
- the memory layout;
- the application execution model.

The same selected MCU library can be linked with:

- bare metal;
- event-driven applications;
- TaktOS;
- FreeRTOS;
- ThreadX.

Only the application and optional kernel change at the final link. IOsonata is not rebuilt with RTOS-specific macros.

## No mandatory configuration framework

The official IOsonata model does not require:

- CMake;
- Kconfig;
- Devicetree;
- a BSP-generated source tree;
- a per-product HAL build matrix.

Configuration remains ordinary C/C++ data owned by the application.

[IOcomposer](https://iocomposer.io) is the official IDE. It provides the supported toolchains, project integration, debug environment and MCU-library builders.

Each target normally has an IOcomposer application project containing the MCU build settings, `board.h`, linker and debug configuration, references to shared application source, and the link to the precompiled IOsonata library. The application project compiles the application; it does not rebuild IOsonata.

## Static memory and explicit execution

Core driver and real-time paths are designed around:

- static or caller-owned storage;
- fixed FIFOs;
- explicit operation state;
- bounded interrupt work;
- no required heap allocation;
- no required exceptions or RTTI.

Polling, interrupt and DMA operation are selected by the concrete controller, transfer and application requirements rather than imposed globally by the interface type.

## Measured on hardware

IOsonata publishes on-target measurements rather than treating object-oriented overhead as an assumption.

PRBS-verified UART results reported in the repository include:

| Target | IOsonata C++ OOD | Zephyr C | nrfx C |
|---|---:|---:|---:|
| nRF54L15 DK | **102.2 KB/s** | 82.9 KB/s | 87.0 KB/s |
| nRF52832 DK, 2 MBaud | **203 KB/s** | Not supported in the tested configuration | 183.7 KB/s |

The TaktOS, FreeRTOS and ThreadX benchmark applications also link the same precompiled IOsonata MCU library, keeping the HAL constant while the kernel changes.

See the benchmark details in the [README](../README.md) and the published *Beyond Blinky* edition on [Leanpub](https://leanpub.com/beyondblinky).

## What portability means here

IOsonata separates portability into three layers:

1. The MCU port implements controller, interrupt, DMA, clock and hardware details.
2. `board.h` supplies board and product data.
3. Generic device and application code uses `Device` and `DeviceIntrf` rather than target-specific controller APIs.

A new board using an already supported MCU normally requires board data and a target application project, not another IOsonata port.

A new MCU requires one MCU port and one reusable library build. Applications for that MCU then share the resulting library artifact.

Host implementations support selected generic logic and interfaces on macOS, Linux and Windows. Hardware-dependent behaviour still requires validation on the target MCU and board.

## When IOsonata is a good fit

IOsonata is intended for projects that value:

- direct control of startup, memory and execution;
- reusable device drivers across MCU families;
- one validated MCU library instead of per-product HAL generation;
- bare-metal and RTOS choice without changing the HAL binary;
- static memory and explicit interrupt behaviour;
- C and C++ interoperability;
- runtime object composition without template-generated driver trees.

Projects that require a large integrated operating-system distribution with its own application lifecycle, package model and configuration language may prefer a different platform.

## Read next

- [Architecture overview](architecture/README.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [FAQ](FAQ.md)
- [Getting Started](getting-started.md)
- [Supported Targets](supported-targets.md)
- [Device inheritance and composition](architecture/device-composition.md)
