# IOsonata

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

## High-efficiency, fully object-oriented embedded C++

### Port once. Compile once. Link only what you use.

IOsonata is an open-source, bare-metal C++ hardware-abstraction layer (HAL), device-driver library and event-driven firmware framework for microcontrollers. It includes a Bluetooth LE host, a composable USB device stack, shared networking (sockets, lwIP, TLS 1.3, MQTT and CoAP), LTE and GNSS on nRF91, NVM storage, crypto, and sensor and display drivers.

MCU support varies by target and peripheral. The [MCU support matrix](docs/supported-targets.md) covers Nordic, STM32, Renesas, Microchip SAM, NXP LPC, Espressif and RP2040 projects, separating implemented core drivers, incomplete integration and recorded hardware results. Hardware results are recorded for Nordic nRF52, nRF53, nRF54L and nRF91, STM32F0, L1, L4, G4, H7 and WBA, Renesas RA4M1 and R9A02G021, Microchip SAM4L and SAM D, and NXP LPC54628 parts, with different tested functions on each.

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
- **Bluetooth on STM32WBA65:** 79,304 / 109,657 B/s notification receive at 1M / 2M PHY, compared with 79,157 / 109,198 B/s for ST's reference client under the same settings.
- **USB on nRF52840 and nRF54LM20:** 2,000-second composite runs, bare metal and TaktOS, with no data errors.

Detailed benchmark tables, configurations and methodology appear below.

---

## Release 0.13

[IOsonata 0.13](https://github.com/IOsonata/IOsonata/releases/tag/0.13) completes the USB device stack started in 0.12, adds new MCU ports, moves USB and Bluetooth to one application event queue and adds shared networking.

- **USB device stack:** CDC, custom bulk, HID, MSC and Bluetooth HCI functions, with interrupt and isochronous transports. Device classes register with the common USB core, which allocates interfaces and endpoints.
- **USB device ports:** nRF52840, nRF54LM20 (full and high speed), SAM4L, RA4M1, STM32L4, STM32H753, STM32WBA65, LPC11U35, LPC1769, LPC54628 (USB1 high speed), SAM D11 and SAM D21.
- **New MCU ports:** RA4M1, LPC11U35, LPC1769, LPC54628, SAM D11, SAM D21, SAM4LS, STM32F030x8, STM32L432, STM32G474, STM32H753 and an initial STM32L152RE port.
- **Bluetooth on more targets:** STM32WBA65 on the ST host and Basic Plus link layer, without the ST HAL. The IOsonata HCI host runs over UART on nRF91 and over IPC on the nRF5340 application core, both to HciController.
- **Nordic Bluetooth security:** no nRF5 SDK or sdk-nrf-bm Peer Manager, with shared bond storage across SoftDevice and SDC builds.
- **One application event queue:** `AppRun()` replaces `BtAppRun()` and runs the USB, Bluetooth and LTE work.
- **Networking:** `SockIntrf`, lwIP over Ethernet on STM32H753 and LPC54628, TLS 1.3, MQTT 3.1.1 and CoAP. nRF91 adds modem and LTE integration.
- **STM32 timers:** one driver for F0, L4, G4 and WBA.
- **Installer:** clones and builds everything the projects need.

A clean build in the installer layout with xPack Arm GNU 15.2.1 and RISC-V GNU 15.2.0 builds all 122 library configurations and 1,333 of 1,428 example configurations.

Applications moving from 0.12 need source changes and a rebuilt MCU library. The [release notes](https://github.com/IOsonata/IOsonata/releases/tag/0.13) list the upgrade steps, the examples that do not build and the functions not yet run on hardware.

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
- USB
- Bluetooth
- TCP and UDP sockets
- SLIP and TLS
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

The same model also allows protocol composition. A `Slip` object can wrap a UART or Bluetooth interface while the application continues to use `DeviceIntrf`. A `Tls` object wraps a TCP `SockIntrf` the same way, and `Mqtt` accepts either one as its `DeviceIntrf`.

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

## Bluetooth and USB measurements

### Bluetooth notification receive

[`BlePrbsRx`](exemples/bluetooth/ble_prbs_rx.md) checks a PRBS notification stream in firmware over 60-second trials, with a 15 ms connection interval and ATT MTU 247. These runs report no PRBS errors.

| Target and stack | 1M PHY | 2M PHY |
|---|---:|---:|
| nRF52, SoftDevice Controller | 75,194 B/s | 132,894 B/s |
| nRF52, SoftDevice | 60,833 B/s | 132,931 B/s |
| STM32WBA65, IOsonata | 79,304 B/s | 109,657 B/s |
| STM32WBA65, ST reference client | 79,157 B/s | 109,198 B/s |

Both STM32WBA65 clients use the same settings and stay below the 2M target of eight notifications per connection interval.

### Composite USB endurance

`UsbComboStress` and `UsbComboStressTaktOS` run CDC loopback, CDC PRBS TX, HID, interrupt and isochronous traffic at the same time. Each run lasted 2,000 seconds with no loopback, PRBS, HID, interrupt or isochronous data errors.

| Target and mode | CDC loopback, each direction | All traffic |
|---|---:|---:|
| nRF52840 bare metal | 349,239 B/s | 824,175 B/s |
| nRF52840 TaktOS | 362,100 B/s | 851,937 B/s |
| nRF54LM20 bare metal | 2,426,717 B/s | 5,012,791 B/s |
| nRF54LM20 TaktOS | 2,649,511 B/s | 5,474,970 B/s |

These results show sustained composite traffic on the tested builds. They are not a speed comparison: the firmware revision and compiler were not recorded with the runs, and the nRF54LM20 bare-metal run had two host pauses totalling 33.2 seconds. See the [USB User Guide](docs/usb-user-guide.md) for the test procedure.

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
| Startup and application entry | Application-defined | Framework initializes the kernel and devices before application entry | MCU port provides startup; standard `main()` application entry |

---

## Repository coverage

IOsonata includes generic and target-specific implementations across:

- ARM Cortex-M and RISC-V
- GPIO, UART, I2C, SPI, ADC, DAC, PWM, DMA, RTC, watchdog and timers
- Bluetooth LE host, HCI transports, security and standard services
- USB device stack: CDC, HID, MSC, custom bulk, interrupt, isochronous and Bluetooth HCI functions
- networking: sockets, lwIP over Ethernet, TLS 1.3, MQTT and CoAP
- LTE modem and GNSS on nRF91
- storage, flash, EEPROM, NVM, SD card and filesystem integration
- hardware and software crypto providers
- sensors, IMUs, displays, touch controllers, audio devices and cameras
- firmware update (DFU)
- macOS, Linux and Windows host interfaces
- bare-metal, TaktOS, FreeRTOS and ThreadX applications

Nordic ports are IOsonata's most developed and most complete MCU family, with
extensive Bluetooth, peripheral, sensor, storage and RTOS examples. STM32F0,
STM32L4 and SAM4LC also have established hardware use.
[Supported MCUs](docs/supported-targets.md) describes each target's implemented
peripherals, application examples, validation and remaining work.

---

## Hardware reference platforms

IOsonata is developed and validated on I-SYST reference hardware as well as vendor development kits.

| Platform | MCU / role | IOsonata use |
|---|---|---|
| [BLYST Nano](https://www.i-syst.com/products/blyst-nano) / IDK-BLYST-NANO | nRF52832 Cortex-M4F | BLE, UART, sensor and low-power application development |
| BLYSTL15 | nRF54L15 Cortex-M33 | nRF54L Bluetooth, `sdk-nrf-bm` and RTOS development |
| BLUEIO-TAG-EVIM | nRF52832 with environmental and motion sensors | Bluetooth and multi-sensor reference platform |
| [IDAP-Link](https://www.i-syst.com/products/idap-link) | CMSIS-DAP SWD/JTAG probe with USB-UART bridge | Flashing, debugging and serial output |
| CS-BLYST-06 / IBK-NRF52840 | nRF52832 / nRF52840 breakout boards | MCU bring-up and peripheral development |

### Recorded hardware validation

| MCU target | Recorded coverage |
|---|---|
| nRF52832 | BLE, UART, sensor and low-power reference applications |
| nRF54L15 | UART, Bluetooth and TaktOS benchmarks |
| nRF52840 | Bare-metal and TaktOS composite USB endurance |
| nRF54LM20 | Bare-metal and TaktOS high-speed composite USB endurance |
| nRF5340 | Audio DK Release playback, microphone capture and I2S restart recovery |
| nRF9160 | Modem AT communication, LTE-M registration and UDP echo with TaktOS |
| nRF9161 | LTE-M registration and UDP echo with TaktOS |
| SAM4LC8C | Hardware validated on SAM4L8 Xplained Pro: startup, GPIO, UART, AST/TC timers, I2C, SPI, CDC, ISO, manual suspend/wake and CryptoTest with AESA |
| SAM D21 / SAM D11 | SAM D21 and SAM D11 Xplained Pro: startup at 48 MHz and UART PRBS TX at the 1 Mbaud line rate |
| RA4M1 | EK-RA4M1: startup/GPIO, timer callbacks, SCI UART, USB (CDC, CDC plus ISO, interrupt loopback, MSC) and CryptoTest with SCE5. IIC hardware validation is deferred beyond 0.13 |
| R9A02G021 | FPB-R9A02G021: UART0, IICA and SAU SPI master/slave, timer callbacks, ADC12, RTC, Sleep and Software Standby wake-up |
| STM32F030x8 | Hardware validated on STM32F0308-DISCO; startup, GPIO, USART1, TIM6/TIM16 and UART DMA PRBS |
| STM32L476 / STM32L496 / STM32L4S9 | Hardware validated and used in existing projects, confirmed by the maintainer |
| STM32L476 (B-L475E-IOT01A2) | Shared drivers: GPIO, UART in all modes, I2C, QUADSPI flash through Nvm in single line and quad mode, USB CDC device |
| STM32L432 | NUCLEO-L432KC: GPIO, timers, UART TX, I2C and SPI master/slave |
| STM32L152RE | NUCLEO-L152RE: Blinky and UART TX |
| STM32G474 | NUCLEO-G474RE: GPIO/EXTI, UART in all modes at 1 Mbaud, LPTIM1 and TIM |
| STM32H753 | NUCLEO-H753ZI: startup, GPIO, USART3 interrupt and DMA TX/RX, TIM2, USB CDC device, Ethernet (DHCP, UDP, TCP, DNS, HTTP) and CryptoTest |
| STM32WBA65 | STM32WBA65I-DK1: GPIO/EXTI, UART, timers, I2C/SPI master/slave, BLE peripheral, central and security, notification throughput, crypto and USB CDC |
| LPC54628 | LPCXpresso54628: UART TX at 1 Mbaud in polling mode, CTIMER triggers and CryptoTest |

These are function-specific maintainer results, not a claim that every
peripheral or the latest commit has passed hardware validation.
[Supported MCUs](docs/supported-targets.md) records peripheral coverage, examples,
project integration, build results and hardware scope.


---

## Built with IOsonata

- [TaktOS](https://github.com/IOsonata/TaktOS), including the KVB and Thread-Metric cross-kernel benchmarks
- [SlimeVRFirmware](https://github.com/IOsonata/SlimeVRFirmware)
- [HciController](https://github.com/IOsonata/HciController), Bluetooth HCI controller firmware for nRF5x parts
- Bluetooth UART bridges running bare metal, with FreeRTOS and with TaktOS
- I-SYST BLYST, BlueIO and sensor reference applications
- Storage, NVM, crypto, IMU and standard Bluetooth-service implementations in this repository

---

## Common questions

### Is IOsonata a C++ alternative to the nRF5 SDK on nRF54L15?

Nordic has placed the nRF5 SDK in maintenance mode, and it does not support the nRF54 series. IOsonata supports nRF52 and nRF54L15 with the same `Device`/`DeviceIntrf` object model, and shared application source under `exemples/` builds for both. On nRF54L15 it runs over the Nordic SoftDevice Controller or the bare-metal SDK S145 SoftDevice, without Zephyr, CMake, Kconfig or Devicetree. Bluetooth security does not use the nRF5 SDK or sdk-nrf-bm Peer Manager, and SoftDevice and SDC builds share one bond storage.

### Can I write Bluetooth LE firmware for nRF52 or nRF54 without Zephyr?

Yes. A bare-metal application calls `BtAppInit()` and then `AppRun()`; an application using pairing or bonding also calls `BtAppSecInit()` from `BtAppInitUserData()`. The Bluetooth stack supports broadcaster, observer, peripheral and central roles, GATT services, pairing with bond persistence, periodic advertising and synchronization. The IOsonata HCI host adds periodic advertising with responses (PAwR) and Encrypted Advertising Data where the controller supports them. See the [Bluetooth User Guide](docs/bluetooth-user-guide.md).

### Can Bluetooth run on an STM32 or on an MCU without a radio?

Yes. STM32WBA65 uses ST's host and Basic Plus link layer through the IOsonata port, without the ST HAL. On nRF91 and other MCUs without a radio, the IOsonata HCI host runs over UART to an nRF5x running [HciController](https://github.com/IOsonata/HciController). The nRF5340 application core uses the same host over IPC to HciController on the network core.

### Is IOsonata event-driven? Does it need an RTOS?

It does not need an RTOS. Interrupts do the time-critical work and queue deferred work to one application event queue (`AppEvtHandlerQue()`). `AppRun()` runs that queue first in first out and sleeps until the next interrupt when it is empty. Bluetooth, USB and LTE share the same queue and loop. The same precompiled library also runs under TaktOS, FreeRTOS or ThreadX; an RTOS application overrides `UsbEvtQue()` and `BtEvtQue()` to pass that work to its subsystem thread.

### Does IOsonata include a USB device stack?

Yes. An application combines CDC ACM, HID, mass storage, custom bulk, interrupt, isochronous and Bluetooth HCI functions, and the stack assigns the interface and endpoint numbers. Native ports cover nRF52840, nRF54LM20, SAM4L, SAM D11/D21, RA4M1, STM32L4, STM32H753, STM32WBA65, LPC11U35, LPC1769 and LPC54628. USB host mode is not available in 0.13. See the [USB User Guide](docs/usb-user-guide.md).

### Does IOsonata support networking?

Yes. `SockIntrf` presents connected TCP and UDP through `DeviceIntrf`. STM32H753 and LPC54628 use lwIP over Ethernet, and nRF91 uses the IP stack of its modem. TLS 1.3, MQTT 3.1.1 and CoAP are shared implementations that run over a connected socket. See [Networking](docs/networking.md).

### Is object-oriented C++ too slow or too large for a microcontroller?

Not when the implementation is careful. IOsonata uses inheritance and runtime polymorphism, is built without RTTI or exceptions, needs no heap in its core data paths, and links only the objects the application references. Its UART path measured 102.2 KB/s on nRF54L15 against 87.0 KB/s for nrfx and 82.9 KB/s for Zephyr (see [Measured against C implementations](#measured-against-c-implementations)).

### How does IOsonata compare with Zephyr, modm or Mbed?

Zephyr and the nRF Connect SDK rebuild the hardware layer for each configuration through CMake, Kconfig and Devicetree. Template-based C++ HALs such as modm specialize the code at compile time. Arm has ended Mbed OS. IOsonata ships one precompiled library per MCU and uses runtime polymorphism, so board, sensor and RTOS choices do not rebuild it. See [Design model at a glance](#design-model-at-a-glance).

### Which IDE and toolchain does it use?

[IOcomposer](https://iocomposer.io) is the official IDE. Its installer sets up the Arm and RISC-V toolchains, OpenOCD and SDK dependencies, and builds the MCU libraries. No CMake is needed. See [Getting Started](docs/getting-started.md).

More answers are in the [FAQ](docs/FAQ.md).

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

See [Getting Started](docs/getting-started.md), the
[Quick Reference](docs/quick-reference.md), the
[Bluetooth User Guide](docs/bluetooth-user-guide.md), the
[USB User Guide](docs/usb-user-guide.md), [Networking](docs/networking.md) and the
[Buzzer User Guide](docs/buzzer.md) for additional workflows. The
[documentation index](docs/README.md) lists the other guides.

---

## Project layout

```text
include/                         Core and device-family headers
src/                             Generic implementations
exemples/                        Reusable application examples
ARM/                             ARM architecture and MCU targets
RISCV/                           RISC-V architecture and MCU targets
Linux/, OSX/, Win/               Host interfaces and libraries
fatfs/, littlefs/, micro-ecc/    Third-party source used by the libraries
Python/                          Host test runners and tools
tests/                           Host, model and target test suites
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

- [IOsonata website](https://iosonata.github.io) - overview, comparisons, benchmarks and FAQ
- [TaktOS](https://github.com/IOsonata/TaktOS) — deterministic kernel using the same IOsonata HAL library
- [IOcomposer](https://iocomposer.io) — embedded development environment and AI-assisted engineering tools
- [SlimeVRFirmware](https://github.com/IOsonata/SlimeVRFirmware) — motion-tracking firmware built on IOsonata
- [HciController](https://github.com/IOsonata/HciController) - Bluetooth HCI controller firmware used with the IOsonata HCI host

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

IOsonata is maintained by [I-SYST inc.](https://i-syst.com), Boucherville, Quebec, Canada.

**Make your IO sing!**
