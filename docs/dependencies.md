# IOsonata Dependencies

This document defines which tools, SDKs and optional libraries belong to IOcomposer, the MCU target, IOsonata or the application.

## Installed layout

The default layout is:

```text
~/IOcomposer/
├── IOsonata/
├── external/
└── workspaces/
```

On Windows, use `%USERPROFILE%\IOcomposer`.

## Development tools

The installed environment provides the tools required by the selected installation profile, including:

- IOcomposer;
- Arm and RISC-V cross-compilers;
- OpenOCD;
- the IOsonata repository;
- project and debug integration;
- the MCU-library builders;
- supported SDK checkouts used by installed targets.

The installer output records the exact versions and locations used by that installation.

## MCU-target dependencies

A target port may require vendor headers, controller libraries, radio stacks or binary components under `~/IOcomposer/external/`.

Examples include:

- Nordic nRF5 SDK, nrfx and bare-metal SDK components;
- ST STM32 device and wireless-stack packages;
- Renesas device support;
- Microchip and NXP device headers;
- Espressif target support;
- CMSIS components for Arm targets.

Different MCU generations from the same vendor may use different packages. Check the selected target project and [Supported Targets](supported-targets.md).

Do not fix a missing target dependency by copying vendor source into an application project.

## Application dependencies

Applications may add libraries that are not part of the IOsonata MCU library.

### Filesystems

- FatFs;
- littlefs.

IOsonata supplies storage and `DiskIO` integration where supported. The filesystem remains an application dependency.

### Networking

The application may select lwIP or another network stack.

### Sensor and algorithm libraries

Some devices require vendor algorithms, calibration data or binary components. Check the vendor licence and redistribution terms before including them in a product.

### RTOS kernels

IOsonata does not own the scheduler.

- TaktOS can be installed and built by the library builder.
- FreeRTOS and ThreadX are application/kernel dependencies.
- Bare metal, TaktOS, FreeRTOS and ThreadX can link the same selected IOsonata MCU library binary.

## Ownership boundary

```text
application source and board.h
    + optional kernel
    + optional filesystem, network or algorithm libraries
    + vendor components required by the MCU target
    + precompiled libIOsonata_<MCU>.a
```

A product does not rebuild IOsonata merely because it selects another board, kernel or optional application library.

Rebuild the affected MCU library when:

- IOsonata source for that MCU changes;
- a vendor dependency consumed by the target implementation changes;
- compiler or target-build settings used to produce the library change.

Rebuild only the application when:

- application source changes;
- `board.h` changes;
- an application-only library changes;
- the optional kernel changes;
- the application memory layout changes.

## Verify an installation

1. launch IOcomposer;
2. run the MCU-library builder;
3. build the selected target in Debug and Release;
4. open a working target example;
5. build, link, flash and run the application.

The MCU-library build checks the compiler, target project, include paths and required target dependencies together.

## Missing SDK or header

When a target build reports a missing vendor header or unresolved vendor symbol:

1. identify the target project that failed;
2. inspect its include and library paths in IOcomposer;
3. verify the expected package under `~/IOcomposer/external/`;
4. rerun the installer when the package is installer-managed;
5. rebuild the selected MCU library;
6. clean and relink the application.

## Updating IOsonata

```bash
cd ~/IOcomposer/IOsonata
git pull
```

After an IOsonata source update:

1. run the MCU-library builder;
2. rebuild each affected MCU library once;
3. clean and relink applications using those libraries.

## Updating external dependencies

Update an external dependency only when the selected target or application requires it.

After updating:

- rebuild the MCU library when IOsonata target code consumes the dependency;
- otherwise rebuild only the application or optional kernel that consumes it;
- repeat hardware validation for the affected subsystem;
- record the tested dependency revision.

Do not rely on a moving default branch for a release product without recording the tested commit or release.

## Offline use

After installation and dependency checkout, normal builds are local. Preserve:

- the IOcomposer installation;
- `~/IOcomposer/IOsonata`;
- `~/IOcomposer/external`;
- the installed toolchains;
- application workspaces.

## Licences

IOsonata is MIT licensed. Vendor SDKs, radio stacks, filesystems and algorithm libraries retain their own licences. Review those licences before redistribution.

## Related documentation

- [Documentation index](README.md)
- [Getting Started](getting-started.md)
- [Quick Reference](quick-reference.md)
- [Architecture overview](architecture/README.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [Supported Targets](supported-targets.md)
