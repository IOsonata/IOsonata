# IOsonata FAQ

Concise answers about the IOsonata architecture, build model and development workflow.

## What is IOsonata?

IOsonata is a multi-architecture hardware-abstraction and device-driver library for microcontrollers. It provides generic device and interface objects together with MCU-specific implementations for startup, interrupts, clocks, communication peripherals, storage, Bluetooth, crypto, sensors and other supported hardware.

## Is IOsonata a source framework rebuilt by every application?

No. IOsonata is compiled as a static library for each supported MCU target and build profile:

```text
libIOsonata_<MCU>.a
```

Debug and Release are separate profiles. Within a selected profile, applications using that MCU link the same library binary.

## What changes between boards using the same MCU?

Board and product differences normally remain application data, primarily in `board.h`:

- pin maps;
- oscillator configuration;
- external-device configuration;
- board-level definitions.

Changing those values does not rebuild the IOsonata MCU library.

## What changes when the MCU changes?

Select and build the library for the new MCU target, then use the corresponding target application project. The generic application and device-driver source can remain unchanged when the required interfaces and devices are supported by both targets.

## When must the IOsonata library be rebuilt?

Rebuild the affected MCU library when IOsonata source for that MCU changes.

Do not rebuild it merely because you changed:

- board pins;
- an application feature;
- the selected sensor or interface object;
- bare metal versus an RTOS;
- application source.

## Does the library contain every supported driver for the MCU?

Yes. Supported implementations are compiled into the static library. The application references the objects it uses, the linker extracts the required object files, and section garbage collection removes unreferenced code and data.

The final firmware changes. The library artifact does not.

## Does this create a large firmware image?

No. A static archive is not copied into the firmware as one indivisible block. Only referenced object files and retained sections are linked into the application image.

## Does IOsonata require CMake, Kconfig or Devicetree?

No. They are not part of the official IOsonata build and configuration model.

Configuration is ordinary C/C++ data owned by the application. IOcomposer is the official IDE and supplies the supported managed-build workflow.

## What is IOcomposer?

[IOcomposer](https://iocomposer.io) is the official IDE for IOsonata. Its installer provides the development environment, toolchains, OpenOCD, SDK integration, project support and the MCU-library build scripts.

See the [IOcomposer workflow](architecture/iocomposer-workflow.md).

## How do I build an IOsonata MCU library?

Run the installed platform builder:

**macOS**

```bash
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_macos.sh
```

**Linux**

```bash
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_linux.sh
```

**Windows**

```powershell
& "$env:USERPROFILE\IOcomposer\IOsonata\Installer\build_iosonata_lib_win.ps1"
```

The builder discovers the supported MCU library projects and lets you select one target or build all targets. It builds both Debug and Release libraries.

## What is the `ioc/` project directory?

Current migrated IOsonata managed-build projects use `ioc/` as the repository directory name for IOcomposer projects.

For example:

```text
ARM/Nordic/nRF52/nRF52832/exemples/Blinky/ioc/
```

is the IOcomposer target project used to compile the nRF52832 Blinky application. MCU library projects use the same convention under `lib/ioc/`.

The directory contains the managed-build metadata and target configuration used by IOcomposer. Older documentation may still show `Eclipse/`; for migrated targets, use the current `ioc/` path.

## What is inside a per-target application project?

A target project normally contains:

- managed-build metadata;
- MCU and compiler settings;
- include and library paths;
- linker and debug configuration;
- `board.h`;
- references to shared application source;
- the link to `libIOsonata_<MCU>.a`.

It normally does not duplicate the shared example source. A target-specific example may contain its own source when the application genuinely depends on that target.

## Where does example source live?

Reusable example source normally lives under the top-level `exemples/` directory. Each supported MCU target has a buildable IOcomposer project that references the shared source and supplies its own board and target configuration.

This lets the same example source be compiled for several MCU targets without copying it into every target project.

## Does an application project compile IOsonata source?

No. It compiles the application and optional kernel, then links the previously built IOsonata MCU library.

A project may reference shared example source, but that does not mean it rebuilds the IOsonata implementation.

## What does portable mean in IOsonata?

Portable means the generic device or application logic is not tied to one MCU, board or concrete bus implementation.

Portability is divided into three parts:

- the MCU port implements the controller and hardware details;
- `board.h` supplies board and product data;
- device drivers depend on `DeviceIntrf`, not directly on one target controller.

Not every target supports every feature at the same maturity level. See [Supported Targets](supported-targets.md).

## What is `DeviceIntrf`?

`DeviceIntrf` represents a transferable data path. It can represent a physical controller or a software/internal interface, including:

- UART;
- I2C;
- SPI, QSPI or OSPI;
- USB;
- Bluetooth GATT through `BtIntrf`;
- SLIP layered over another interface;
- internal crypto or memory-controller access paths.

A device driver accepts a `DeviceIntrf *` instead of depending directly on one concrete controller type.

## What is `Device`?

`Device` is the common base for hardware devices, software engines and device capability families. It supplies common lifecycle, identity, interface injection, callback and timer state.

Derived families add behaviour such as sensor sampling, display control, storage access or crypto operations.

## Can one driver use I2C or SPI?

Yes, when the physical device supports both interfaces. The driver receives a `DeviceIntrf *` and uses the selected concrete interface at runtime.

The interface object changes. The device-driver implementation does not.

## Does IOsonata use templates to generate drivers?

No. IOsonata uses ordinary C++ classes, inheritance, runtime polymorphism and object composition. It does not generate a different driver type for every application configuration.

Low-level operations that do not benefit from object state remain lightweight C/C++ functions.

## Does object-oriented C++ make IOsonata slower than C HALs?

Not inherently. IOsonata publishes on-target UART and kernel benchmark results showing that its object-oriented paths can match or exceed the tested C implementations.

See the benchmark section in the [README](../README.md) and [Beyond Blinky on Leanpub](https://leanpub.com/beyondblinky).

## Does IOsonata require dynamic memory?

No heap is required in the core driver and real-time data paths. IOsonata uses static storage, caller-owned buffers, fixed FIFOs and explicit operation state.

## Does IOsonata require exceptions or RTTI?

No. The library is designed to operate without exceptions or RTTI.

## Can IOsonata be used from C?

Yes. Core interfaces normally provide:

- a C structure containing state and function pointers;
- C helper functions operating on that structure;
- a C++ class wrapping the same implementation.

The C and C++ APIs do not require two independent drivers.

## Does IOsonata require an RTOS?

No. IOsonata can be used with:

- bare metal;
- event-driven applications;
- TaktOS;
- FreeRTOS;
- ThreadX.

IOsonata does not own the scheduler.

## Is the library rebuilt for each RTOS?

No. For a selected MCU and build profile, bare-metal, TaktOS, FreeRTOS and ThreadX applications link the same IOsonata library binary. The application and optional kernel change at the final link.

## Does IOsonata own `main()`?

No. The application owns `main()`, board configuration, memory layout and execution model.

## Does a different linker script require rebuilding IOsonata?

No. An application may use a different memory layout for a bootloader, DFU region, NVM/PDS area, secure/non-secure partition or another deliberate flash/RAM reservation. That changes the final application link, not the IOsonata library.

## Does IOsonata need a CI/CD build matrix for every product?

No. The HAL is not regenerated for every board, RTOS, peripheral and product combination.

CI may still be used to test IOsonata source changes or produce library releases. It is not required to manufacture another HAL binary for every application configuration.

## Can IOsonata be tested on a host computer?

Some generic logic and host-supported interfaces can be built or tested on macOS, Linux or Windows. Host support does not mean every MCU peripheral driver executes unchanged on a desktop operating system.

Hardware-dependent behaviour must still be validated on the target MCU and board.

## Which hardware is used for validation?

IOsonata is developed and validated on I-SYST hardware and vendor development kits. Current reference platforms include BLYST Nano/IDK-BLYST-NANO, BLYSTL15, BLUEIO-TAG-EVIM, IDAP-Link and supported vendor boards.

See the hardware-reference section in the [README](../README.md) and the current [Supported Targets](supported-targets.md) matrix.

## Why is the book also stored as Markdown?

`beyond_blinky_free_edition.md` records the publication status and links to `beyond_blinky_free_edition_2025_raw.md`, the machine-readable extraction of the December 2025 free edition. It is retained for repository search and RAG indexing, not as the current setup or workflow guide.

Current architecture and workflow documentation takes precedence where the published book differs. Human readers should [download the free edition or buy the complete book on Leanpub](https://leanpub.com/beyondblinky).

## How do I start?

1. Install IOcomposer.
2. Run the MCU-library builder.
3. Select a supported MCU.
4. Open the corresponding target example project in IOcomposer.
5. Configure `board.h` for the board.
6. Build, flash and debug.

The recommended first path uses the nRF52832 Blinky target project. See [Getting Started](getting-started.md).

## Where should I read next?

- [Architecture overview](architecture/README.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [Getting Started](getting-started.md)
- [Quick Reference](quick-reference.md)
- [Supported Targets](supported-targets.md)
- [DeviceIntrf implementer notes](architecture/devintrf-implementer-notes.md)
- [Device inheritance and composition](architecture/device-composition.md)
- [Beyond Blinky free and complete editions](https://leanpub.com/beyondblinky)
