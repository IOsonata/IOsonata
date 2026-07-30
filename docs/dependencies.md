# IOsonata Dependencies

IOcomposer and the IOsonata installers manage the common development tools and repository layout. Additional SDKs and libraries are target- or feature-specific.

## Installation root

The default layout is:

```text
~/IOcomposer/
├── IOsonata/
├── external/
└── workspaces/
```

On Windows, use `%USERPROFILE%\IOcomposer`.

## Installed development tools

The installer provides the tools required by the selected installation profile, including:

- IOcomposer;
- Arm and RISC-V cross-compilers;
- OpenOCD;
- the IOsonata repository;
- project integration and the project wizard;
- the MCU-library build scripts;
- supported SDK checkouts used by the installed targets.

The installer output is the source of truth for exact versions and locations.

## Target SDKs

Target projects may use vendor headers, controller libraries, radio stacks or binary components under `~/IOcomposer/external/`.

Common families include:

- Nordic nRF5 SDK, nrfx and bare-metal SDK components;
- ST STM32 device and wireless-stack packages;
- Renesas device support;
- Microchip and NXP device headers;
- Espressif target support;
- CMSIS components for Arm targets.

Do not assume that every MCU family uses the same vendor package or that one package covers every family generation. Check the selected target project and [`supported-targets.md`](supported-targets.md).

## Optional libraries

Applications may add optional libraries according to their features.

### Filesystems

- FatFs;
- littlefs.

IOsonata supplies storage and `DiskIO` integration where supported. The filesystem remains an application dependency.

### Networking

- lwIP or another application-selected network stack.

### Sensor algorithms

Some sensors require vendor algorithm libraries, calibration data or binary components. Examples can include Bosch environmental processing and TDK/InvenSense motion algorithms.

Check the vendor licence before redistribution.

### RTOS kernels

IOsonata itself is scheduler-independent.

- TaktOS can be installed and built by the library builder.
- FreeRTOS and ThreadX are application/kernel dependencies.
- All supported execution models link the same selected IOsonata MCU library binary.

## Dependency ownership

The dependency boundary is:

```text
application and optional kernel
    + optional filesystem/network/algorithm libraries
    + vendor components required by the MCU target
    + precompiled libIOsonata_<MCU>.a
```

A product does not rebuild IOsonata merely because it selects another optional application library.

## Verify an installation

After installation:

1. launch IOcomposer;
2. run the IOsonata MCU-library builder;
3. confirm the selected target builds Debug and Release libraries;
4. open a working target example;
5. build and link the application.

The library builder is the most direct dependency check because it exercises the toolchain, target project and required includes together.

## Missing SDK or header

When a target build reports a missing vendor header or unresolved vendor symbol:

1. identify which target project produced the error;
2. inspect its include and library paths in IOcomposer;
3. verify the expected repository or SDK under `~/IOcomposer/external/`;
4. rerun the installer if the dependency is installer-managed;
5. rebuild the selected MCU library;
6. clean and relink the application.

Do not fix a missing target dependency by copying vendor source into an application project.

## Updating IOsonata

```bash
cd ~/IOcomposer/IOsonata
git pull
```

After an IOsonata source update:

1. run the MCU-library builder;
2. rebuild each affected MCU library once;
3. clean and relink applications using those libraries.

Application, board and RTOS changes do not require another IOsonata library build.

## Updating external dependencies

Update an external dependency only when the selected target or application requires it. After updating:

- rebuild the affected MCU library if the dependency is consumed by IOsonata target code;
- otherwise rebuild only the application or optional kernel that consumes it;
- rerun hardware validation for the affected subsystem.

Do not track an external package's moving default branch in a release product without recording the tested revision.

## Offline use

After installation and dependency checkout, IOcomposer and IOsonata builds are local. Preserve:

- the IOcomposer installation;
- `~/IOcomposer/IOsonata`;
- `~/IOcomposer/external`;
- the toolchain installation;
- application workspaces.

## Licence review

IOsonata is MIT licensed. External SDKs, radio stacks, filesystems and sensor algorithm libraries retain their own licences. Review those licences before commercial redistribution.

## Related documentation

- [Getting started](getting-started.md)
- [Quick reference](quick-reference.md)
- [Architecture overview](architecture/README.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [Supported targets](supported-targets.md)
