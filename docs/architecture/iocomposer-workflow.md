# IOcomposer Precompiled-Library Workflow

[IOcomposer](https://iocomposer.io) is the official IDE for IOsonata.

The workflow is built around one rule:

> Build the MCU library once, then link that same binary into every application using that MCU and build profile.

## Installed layout

The default installation root is:

```text
~/IOcomposer
├── IOsonata/
├── external/
└── workspaces/
```

On Windows, the equivalent root is under `%USERPROFILE%\IOcomposer`.

The installer supplies:

- IOcomposer;
- Arm and RISC-V toolchains;
- OpenOCD;
- IOsonata;
- supported vendor SDKs and project integration;
- the IOsonata project wizard;
- the MCU-library build scripts.

## Step 1: Build the MCU library

Run the platform builder after installation or after changing IOsonata source.

### macOS

```bash
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_macos.sh
```

### Linux

```bash
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_linux.sh
```

### Windows

```powershell
& "$env:USERPROFILE\IOcomposer\IOsonata\Installer\build_iosonata_lib_win.ps1"
```

The builder:

1. locates the IOcomposer installation;
2. discovers supported IOsonata MCU library projects;
3. presents a target-selection menu;
4. lets the user select one MCU or build all targets;
5. runs the IOcomposer managed builder headlessly;
6. produces Debug and Release static libraries.

```text
<MCU library project>/Debug/libIOsonata_<MCU>.a
<MCU library project>/Release/libIOsonata_<MCU>.a
```

When TaktOS is installed, the builder also builds the detected TaktOS architecture libraries. Use `--no-taktos` on macOS/Linux or `-NoTaktos` on Windows to build IOsonata only.

## Step 2: Create or open an application

A new application is created from:

```text
File → New → Project → IOsonata Project
```

The project wizard selects the MCU, toolchain and application features. Those selections configure the application project. They do not regenerate or recompile IOsonata.

A generated application contains application-owned files such as:

```text
my_project/
├── include/
│   ├── board.h
│   └── my_project.h
└── src/
    └── main.cpp
```

The project links the previously built MCU library.

```text
application sources
+ board.h
+ optional kernel
+ libIOsonata_<MCU>.a
= final firmware
```

## Step 3: Supply board data

`board.h` contains data belonging to the product or board:

- pin assignments;
- `McuOsc_t` oscillator selection;
- external-device configuration;
- board-level constants.

A board variant using the same MCU does not require another IOsonata build.

The linker script changes only when the application deliberately changes its memory map, for example:

- bootloader or DFU reservation;
- NVM/PDS region;
- secure/non-secure image partition;
- custom flash or RAM region.

Those application memory choices still do not rebuild IOsonata.

## Step 4: Build, flash and debug

In IOcomposer:

1. select the Debug or Release application configuration;
2. build the project;
3. select the appropriate OpenOCD or probe configuration;
4. start debugging;
5. resume from `main()`.

IDAP-Link, J-Link, ST-Link and other supported probes can be used according to the target configuration.

## Shared example source

IOsonata example source is organized under `exemples/` and reused by target application projects.

A target project may reference shared example or application source without copying it. That source sharing does not mean the MCU implementation is recompiled. The target project still links the precompiled IOsonata library.

## Updating IOsonata

After changing or updating IOsonata source:

1. run the MCU-library builder;
2. rebuild the affected MCU target once;
3. clean and relink applications that use that library.

There is no separate HAL rebuild for every board, product, RTOS or peripheral combination.

## RTOS use

Bare-metal, TaktOS, FreeRTOS and ThreadX applications use the same selected IOsonata library binary.

```text
TaktOS application   ─┐
FreeRTOS application ─┼── libIOsonata_<MCU>.a
ThreadX application  ─┤
bare-metal app       ─┘
```

Only the application and optional kernel change at the final link.

## CI/CD scope

A CI system may be used to test IOsonata source changes or produce release artifacts. It is not required to regenerate a HAL for every product configuration because IOsonata has no board/RTOS/peripheral build matrix.

The reusable library is the deliverable. The consuming project compiles its own application and optional kernel, then links the existing library.
