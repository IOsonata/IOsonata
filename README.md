# IOsonata

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

## High-efficiency, fully object-oriented embedded C++

### Port once. Compile once. Link only what you use.

IOsonata is an open-source, multi-architecture hardware-abstraction and device-driver library for microcontrollers.

It uses real object-oriented design—encapsulation, inheritance, runtime polymorphism and object composition—while matching or exceeding the performance of tested C-only HALs and frameworks in published on-target benchmarks.

Each MCU target is ported once and compiled into reusable Debug and Release static libraries. Within a selected build profile, every application links the exact same library binary:

```text
libIOsonata_<MCU>.a
```

Every board, product and execution model using that MCU target links the same validated library artifact.

- No board-specific HAL fork
- No RTOS-specific HAL build
- No I2C build, SPI build or sensor-specific build
- No Kconfig or Devicetree feature matrix
- No CMake in the official build workflow
- No heap required in the core data paths

The application owns `main()`, its board configuration, its memory layout and its execution model.

### Measured on hardware

- **IOsonata UART on nRF54L15:** 102.2 KB/s, compared with nrfx at 87.0 KB/s and Zephyr at 82.9 KB/s.
- **TaktOS on nRF54L15 KVB:** up to 3.12× Zephyr and 2.88× ThreadX across the reported scheduler, synchronization and IPC operations.
- **Controlled kernel comparison:** TaktOS, FreeRTOS and ThreadX all link the exact same precompiled IOsonata library binary.

Detailed benchmark tables, configurations and methodology appear below.

---

## One MCU port. One validated binary.

Debug and Release are separate build profiles. For either selected profile, the same IOsonata library binary is reused unchanged across boards, products, applications and supported execution models.

The MCU library contains the complete supported implementation for that target:

```text
libIOsonata_<MCU>.a
├── startup and interrupt vectors
├── clock and system initialization
├── GPIO
├── UART
├── I2C
├── SPI
├── timers and PWM
├── Bluetooth support
├── storage and NVM
├── crypto
├── sensors
├── displays
├── USB
└── common Device and DeviceIntrf architecture
```

Board variants do not rebuild that library.

The application supplies board data through `board.h`, including:

- pin maps
- `McuOsc_t` oscillator configuration
- external-device configuration
- board-level definitions

Changing pins or oscillator selection changes the data supplied by the application. It does not change or recompile the MCU implementation.

The standard linker script also remains the same for boards using the same MCU. It changes only when an application deliberately reserves memory for something such as:

- DFU or a bootloader
- NVM/PDS partitions
- secure and non-secure images
- a user-defined flash or RAM partition

Those application memory choices still do not rebuild the IOsonata library.

```text
Application A + board A + TaktOS    ─┐
Application B + board B + FreeRTOS  ─┼── same libIOsonata_<MCU>.a
Application C + board C + ThreadX   ─┤
Application D + board D + bare metal─┘
```

The library is not reconfigured or recompiled for the board, application or operating system.

---

## Compile everything once. Let the linker select it.

Traditional configurable frameworks rebuild the hardware layer for combinations of:

```text
MCU
× board
× RTOS
× peripheral selection
× protocol stack
× storage option
× security option
× sensor set
× boot configuration
```

That rapidly creates thousands of possible framework binaries.

IOsonata does not use one compilation macro for I2C, another for SPI, another for BLE, and another for every sensor combination. Supported implementations are compiled into the static library once.

The application references the objects it uses. The static linker extracts the required object files, and section garbage collection removes unreferenced functions and data.

```text
Same library + application references I2C and Sensor A
    → I2C and Sensor A are linked

Same library + application references SPI and Sensor B
    → SPI and Sensor B are linked

Same library + application references neither
    → neither appears in the final firmware
```

The final application image changes. The IOsonata library artifact does not.

### No HAL CI/CD build farm

The precompiled library is the deliverable, not a source recipe that every product must regenerate.

A consuming project compiles its application and optional kernel, then links the existing validated IOsonata library. No CI/CD pipeline is required to rebuild IOsonata for every board, RTOS, peripheral or product combination because there is no such rebuild matrix.

---

## Full object-oriented design without giving up the machine

IOsonata is not C code hidden behind class syntax, and it is not a template generator producing a different type for every application configuration.

Its design is built around two primary object families.

### `DeviceIntrf`: the data path

`DeviceIntrf` represents a transferable data path such as:

- I2C
- SPI
- UART
- Bluetooth
- SLIP
- another hardware or software interface

A device driver depends on the interface role, not a specific bus implementation.

### `Device`: the device family

`Device` provides the common lifecycle and interface association used by sensors, displays, storage devices and other hardware or software devices.

Inheritance represents real device families. Runtime polymorphism lets application code work through a common interface while concrete implementations remain replaceable.

### One sensor driver. Any compatible interface.

The LSM303AGR driver accepts a `DeviceIntrf` rather than generating separate I2C and SPI driver types:

```cpp
I2C i2c;
SPI spi;
Timer timer;
AccelLsm303agr accel;

// Initialize i2c and spi for the target application.
DeviceIntrf *interface = useSpi
    ? static_cast<DeviceIntrf *>(&spi)
    : static_cast<DeviceIntrf *>(&i2c);

accel.Init(accelCfg, interface, &timer);
```

This is one sensor driver. The selected interface changes; the driver implementation does not.

The same model also allows protocol composition. A `Slip` object can wrap a UART or Bluetooth interface while the application continues to use `DeviceIntrf`.

Low-level operations that do not benefit from object state remain lightweight C/C++ functions. GPIO pin control, for example, is not forced into a class hierarchy.

---

## One HAL binary. Any execution model.

IOsonata does not own the scheduler and does not require an RTOS.

The same precompiled MCU library is used with:

- bare metal
- event-driven applications
- [TaktOS](https://github.com/IOsonata/TaktOS)
- FreeRTOS
- ThreadX

TaktOS, FreeRTOS and ThreadX benchmark projects all link the exact same precompiled IOsonata library. IOsonata is not rebuilt with OS-specific macros for any of them. Only the kernel changes at the final application link.

That is binary-level RTOS independence, not merely source compatibility.

---

## Measured against C implementations

### IOsonata UART versus C HALs/frameworks

PRBS-verified UART throughput on Nordic hardware:

| Target | IOsonata C++ OOD | Zephyr C | nrfx C |
|---|---:|---:|---:|
| nRF54L15 DK | **102.2 KB/s** | 82.9 KB/s | 87.0 KB/s |
| nRF52832 DK, 2 MBaud | **203 KB/s** | Not supported in the tested configuration | 183.7 KB/s |

The benchmark uses the same IOsonata `DeviceIntrf` path used by normal applications and verifies the received PRBS stream for errors. See [Beyond Blinky — free edition](https://leanpub.com/beyondblinky) for the benchmark description and methodology.

Object-oriented design is not the performance problem. Poor implementation is.

### IOsonata + TaktOS versus complete C RTOS/framework stacks

For the native KVB comparison, TaktOS, FreeRTOS and ThreadX use:

- the same MCU and board
- the same precompiled IOsonata HAL library
- the same startup and vector implementation
- the same clock configuration
- the same UART driver
- the same benchmark application
- the same compiler family and optimization level

Only the kernel changes.

KVB results on nRF54L15, Cortex-M33 at 128 MHz, GCC 12.2.x, `-Os`, 1 kHz tick:

| KVB operation | TaktOS | FreeRTOS 11.3 | ThreadX 6.4.2 | Zephyr 4.3.99 |
|---|---:|---:|---:|---:|
| Cooperative yields / 10 s | **10,658,299** | 6,730,284 | 4,034,399 | 4,608,098 |
| Semaphore pairs / s | **1,468,992** | 390,280 | 1,085,516 | 526,438 |
| Mutex pairs / s | **1,323,191** | 269,394 | 460,008 | 424,115 |
| Queue pairs / s | **342,282** | 171,693 | 303,617 | 153,958 |

KVB publishes throughput only when the corresponding behaviour test also passes.

Zephyr is evaluated as its complete integrated stack. Its benchmark build is deliberately stripped and optimized for equivalent functionality:

- size optimization enabled
- logging subsystem disabled
- heap disabled
- Bluetooth disabled
- networking disabled
- USB disabled
- 1 kHz periodic tick
- comparable stack checking and runtime validation retained

This is not a comparison against an unoptimized default Zephyr build.

See the [TaktOS on-target benchmark results](https://github.com/IOsonata/TaktOS#on-target-benchmark-results) for KVB, Thread-Metric, binary-size notes, configurations and methodology.

---

## Design model at a glance

| Design property | Template-generated C++ HAL | Zephyr / NCS | IOsonata |
|---|---|---|---|
| Primary reuse mechanism | Compile-time specialization | Kconfig + Devicetree + source rebuild | Precompiled MCU static library |
| HAL rebuilt per application configuration | Usually | Yes | **No** |
| Peripheral and driver selection | Templates/macros | Kconfig/Devicetree | Object references + linker |
| Runtime inheritance and polymorphism | Usually absent | C device model | **Core architecture** |
| Same HAL binary across board variants | Usually no | No | **Yes** |
| Same HAL binary across RTOSes | Usually no | Zephyr-integrated | **Bare metal, TaktOS, FreeRTOS, ThreadX** |
| Device Tree required | No | Yes | **No** |
| Official build requires CMake | Varies | Yes | **No** |
| Startup and application entry | Application-defined | Framework initializes the kernel and devices before application entry | **Application-defined** |

---

## Repository coverage

IOsonata includes generic and target-specific implementations across:

- ARM Cortex-M and RISC-V
- GPIO, UART, I2C, SPI, ADC, PWM and timers
- Bluetooth interfaces, security and standard services
- storage, flash, EEPROM, NVM and filesystem integration
- hardware and software crypto providers
- sensors, IMUs, displays and touch controllers
- USB and host-side interfaces
- bare-metal, TaktOS, FreeRTOS and ThreadX applications

Target support has different validation levels. See [Supported Targets](docs/supported-targets.md) for the current matrix.

---

## Hardware reference platforms

IOsonata is developed and validated on I-SYST reference hardware as well as vendor development kits.

| Platform | MCU / role | IOsonata use |
|---|---|---|
| [BLYST Nano](https://www.i-syst.com/products/blyst-nano) / IDK-BLYST-NANO | nRF52832 Cortex-M4F | Primary BLE, UART, sensor and low-power baseline |
| BLYSTL15 | nRF54L15 Cortex-M33 | nRF54L and `sdk-nrf-bm` bare-metal baseline |
| BLUEIO-TAG-EVIM | nRF52832 with environmental and motion sensors | Bluetooth and multi-sensor reference platform |
| [IDAP-Link](https://www.i-syst.com/products/idap-link) | CMSIS-DAP SWD/JTAG probe with USB-UART bridge | Flashing, debugging and serial output |
| CS-BLYST-06 / IBK-NRF52840 | nRF52832 / nRF52840 breakout boards | MCU bring-up and peripheral development |

### Current hardware-validation baseline

| MCU target | Reference hardware | Status |
|---|---|---|
| nRF52832 | IDK-BLYST-NANO, BLUEIO-TAG-EVIM, Nordic nRF52 DK | Hardware validated |
| nRF54L15 | BLYSTL15, Nordic nRF54L15 DK | Hardware validated |

STM32 source ports and target projects remain in the repository, but no STM32 target is listed as a current hardware-validation baseline until an exact board and test record are documented. See [Supported Targets](docs/supported-targets.md).

---

## Built with IOsonata

- [TaktOS](https://github.com/IOsonata/TaktOS), including the KVB and Thread-Metric cross-kernel benchmarks
- [SlimeVRFirmware](https://github.com/IOsonata/SlimeVRFirmware)
- Bluetooth UART bridges running bare metal, with FreeRTOS and with TaktOS
- I-SYST BLYST, BlueIO and sensor reference applications
- Storage, NVM, crypto, IMU and standard Bluetooth-service implementations in this repository

---

## Quick start

### Recommended first run

```text
Install IOcomposer
    → run the IOsonata library builder
    → select nRF52832
    → open ARM/Nordic/nRF52/nRF52832/exemples/Blinky/ioc/
    → use IDK-BLYST-NANO or an nRF52832 DK
    → build, flash and debug with IDAP-Link, J-Link or another supported probe
```

### Install IOcomposer

[IOcomposer](https://iocomposer.io) provides an embedded development environment with ARM and RISC-V toolchains, OpenOCD, SDK integration and IOsonata project support.

**macOS**

```bash
curl -fsSL https://iocomposer.io/install_ioc_macos.sh -o /tmp/install_ioc_macos.sh && bash /tmp/install_ioc_macos.sh
```

**Linux**

```bash
curl -fsSL https://iocomposer.io/install_ioc_linux.sh -o /tmp/install_ioc_linux.sh && bash /tmp/install_ioc_linux.sh
```

**Windows — PowerShell as Administrator**

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://iocomposer.io/install_ioc_windows.ps1 | iex"
```

The repository also contains standalone installation scripts under [`Installer/`](Installer/) for developers who prefer to assemble the environment directly.

### Build the MCU libraries

After installation, run the platform-specific builder from `IOsonata/Installer`.

**macOS**

```bash
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_macos.sh
```

**Linux**

```bash
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_linux.sh
```

**Windows — PowerShell**

```powershell
& "$env:USERPROFILE\IOcomposer\IOsonata\Installer\build_iosonata_lib_win.ps1"
```

The builder:

1. Locates the IOcomposer installation.
2. Discovers the supported IOsonata MCU library projects under `*/lib/ioc/`.
3. Presents an interactive menu to select one MCU target or build all targets.
4. Runs the IOcomposer managed builder headlessly.
5. Builds both Debug and Release configurations and places the libraries in the selected project's `Debug/` and `Release/` directories.

When TaktOS is installed, its ARM and RISC-V library projects are built automatically after the selected IOsonata MCU library. Use `--no-taktos` on macOS/Linux or `-NoTaktos` on Windows to build IOsonata only.

### Open a working example

In IOcomposer:

1. Select **File → Open Projects from File System...**
2. Open a target project, for example:
   `ARM/Nordic/nRF52/nRF52832/exemples/Blinky/ioc/`
3. Build, flash and debug.

Generic reusable examples are under [`exemples/`](exemples/). Buildable MCU projects are under each target's `exemples/` directory.

See [Getting Started](docs/getting-started.md) and the [Quick Reference](docs/quick-reference.md) for additional workflows.

---

## Project layout

```text
include/                         Core and device-family headers
src/                             Generic implementations
exemples/                        Reusable application examples
ARM/                             ARM architecture and MCU targets
RISCV/                           RISC-V architecture and MCU targets
docs/                            Architecture, porting and usage documentation
Installer/                       Cross-platform development-environment installers
```

---

## Learn the architecture

**Beyond Blinky: Object-Oriented C++ Programming — Fun, Fast, and Fearless Embedded Development**

- [Download the free edition or buy the complete book on Leanpub](https://leanpub.com/beyondblinky)
- [Amazon](https://www.amazon.com/Beyond-Blinky-Object-Oriented-Programming-Development/dp/1069933511)

Publication status and retrieval links are kept in `docs/beyond_blinky_free_edition.md`. The dated machine-readable extraction is `docs/beyond_blinky_free_edition_2025_raw.md` for repository search and RAG indexing. Current repository documentation takes precedence for setup, build workflow and project organization.

The book explains the object model, `DeviceIntrf`, `Device`, driver portability, MCU porting and the measured-performance approach used by IOsonata.

---

## Related projects

- [TaktOS](https://github.com/IOsonata/TaktOS) — deterministic kernel using the same IOsonata HAL library
- [IOcomposer](https://iocomposer.io) — embedded development environment and AI-assisted engineering tools
- [SlimeVRFirmware](https://github.com/IOsonata/SlimeVRFirmware) — motion-tracking firmware built on IOsonata

---

## Contributing

Useful contributions include:

- new MCU ports that preserve the precompiled-library model
- validated target examples
- device drivers using `Device` and `DeviceIntrf`
- benchmark results with reproducible configurations
- documentation and porting improvements

Use [GitHub Issues](https://github.com/IOsonata/IOsonata/issues) for defects and feature requests, and [GitHub Discussions](https://github.com/IOsonata/IOsonata/discussions) for design discussions.

---

## License

IOsonata is released under the MIT License. See [LICENSE](LICENSE).

---

## Maintainer

IOsonata is maintained by [I-SYST inc.](https://i-syst.com), Brossard, Québec, Canada.

**Make your IO sing!**
