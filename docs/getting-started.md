# Getting Started with IOsonata

This guide uses [IOcomposer](https://iocomposer.io), the official IDE for IOsonata.

You need:

- a supported MCU board;
- a compatible SWD/JTAG debug probe;
- a USB cable and target power;
- internet access for the initial installer and SDK checkout.

The recommended first run uses an nRF52832 target project and the shared Blinky example.

## First-run path

```text
install IOcomposer
    -> build the nRF52832 IOsonata library
    -> open the nRF52832 Blinky target project
    -> configure board.h
    -> build, flash and debug
```

The same structure applies to other supported MCU targets.

## 1. Install IOcomposer

### macOS

```bash
curl -fsSL https://iocomposer.io/install_ioc_macos.sh -o /tmp/install_ioc_macos.sh && bash /tmp/install_ioc_macos.sh
```

### Linux

```bash
curl -fsSL https://iocomposer.io/install_ioc_linux.sh -o /tmp/install_ioc_linux.sh && bash /tmp/install_ioc_linux.sh
```

### Windows - PowerShell as Administrator

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://iocomposer.io/install_ioc_windows.ps1 | iex"
```

The default installation root is `~/IOcomposer` on macOS/Linux and `%USERPROFILE%\IOcomposer` on Windows.

The installation provides:

- IOcomposer;
- Arm and RISC-V toolchains;
- OpenOCD;
- the IOsonata repository;
- project and debug integration;
- supported SDK checkouts;
- the MCU-library builders.

Standalone environment scripts are also kept under [`Installer/`](../Installer/).

## 2. Build the MCU library

Run the platform builder after installation.

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

The builder discovers the MCU library projects in the current checkout. Select the MCU used by the board, or select **Build All**.

For this first run, select **nRF52832**.

The builder produces matching Debug and Release archives:

```text
<MCU library project>/Debug/libIOsonata_<MCU>.a
<MCU library project>/Release/libIOsonata_<MCU>.a
```

For either selected profile, every application using that MCU links the same IOsonata library binary. Board, product, peripheral and RTOS changes do not create another HAL build.

When TaktOS is installed, the builder also builds detected TaktOS architecture libraries. Use `--no-taktos` on macOS/Linux or `-NoTaktos` on Windows to build IOsonata only.

## 3. Open the Blinky target project

1. Launch IOcomposer.
2. Select **File -> Open Projects from File System...**
3. Browse to:
   `~/IOcomposer/IOsonata/ARM/Nordic/nRF52/nRF52832/exemples/Blinky/`
4. Enable **Search for nested projects**.
5. Select the discovered target project and finish the import.

Use an IDK-BLYST-NANO/BLYST Nano or Nordic nRF52 DK for the documented nRF52832 first-run path.

The target project normally contains build metadata, MCU settings, linker/debug configuration and `board.h`. It references the shared application source and links the precompiled MCU library. It does not compile the IOsonata implementation again.

## 4. Configure the board

Edit the project `board.h` for the actual hardware.

Board data normally includes:

- pin assignments;
- `McuOsc_t` oscillator configuration;
- external-device configuration;
- board-level definitions.

Changing board data changes the application. It does not rebuild the IOsonata MCU library.

## 5. Build the application

1. Select the Debug or Release application configuration.
2. Right-click the project in Project Explorer.
3. Select **Build Project**.
4. Check the Console for compiler, linker and size output.

The application build is:

```text
application source
+ board.h
+ optional kernel
+ libIOsonata_<MCU>.a
= firmware image
```

The application profile must link the matching Debug or Release IOsonata library.

## 6. Flash and debug

1. Connect the probe to the target SWD/JTAG pins.
2. Select the target's configured debug launch.
3. Start debugging.
4. IOcomposer flashes the image and normally halts at `main()`.
5. Press **F8** to resume.

IDAP-Link, J-Link, ST-Link and other supported probes can be used when the target project provides the appropriate configuration.

## 7. Create an application

After the working example builds and runs:

1. select **File -> New -> Project -> IOsonata Project**;
2. enter the project name;
3. select C++ or C;
4. select the Arm or RISC-V toolchain;
5. select the MCU family and target;
6. select the application features offered by the wizard;
7. finish the project creation;
8. configure `board.h`;
9. build and debug.

Feature selection configures the application project. It does not generate another IOsonata library.

## Shared examples and target projects

Reusable source lives under [`exemples/`](../exemples/):

```text
exemples/
├── misc/
├── uart/
├── i2c/
├── spi/
├── storage/
├── bluetooth/
└── sensor/
```

Each supported MCU normally has target projects that reference the shared source and provide target-specific build and board configuration. Source belongs inside a target project only when the example is genuinely target-specific.

Useful starting points include:

- `Blinky` - `exemples/misc/blinky.c`;
- `UartRetargetDemo` - `exemples/uart/uart_retarget_demo.cpp`;
- `UartPrbsTxTest` - `exemples/uart/uart_prbs_tx.cpp`;
- `I2CMasterDemo` - `exemples/i2c/i2c_master_demo.cpp`;
- `SPIMasterDemo` - `exemples/spi/spi_master_demo.cpp`;
- storage examples under `exemples/storage/`;
- Bluetooth examples under `exemples/bluetooth/`;
- sensor examples under `exemples/sensor/`.

Use the [Quick Reference](quick-reference.md) for interface configuration examples and common project actions.

## Using an RTOS

Bare-metal, TaktOS, FreeRTOS and ThreadX applications can link the same selected `libIOsonata_<MCU>.a`.

The application and optional kernel change at the final link. IOsonata is not rebuilt with scheduler-specific macros.

## Updating IOsonata

After updating or changing IOsonata source:

1. run the platform library builder;
2. rebuild each affected MCU library once;
3. clean and relink applications using that library;
4. update application code when a public API changed.

Application-only, board-only and RTOS-only changes do not require another IOsonata library build.

## Troubleshooting

### Project wizard is missing

Rerun the IOcomposer installer. The installer owns the IDE and project integration.

### Compiler is not found

Verify the installed Arm or RISC-V toolchain, or rerun the installer.

### Vendor headers or symbols are missing

Verify the required SDK under `~/IOcomposer/external/`, rebuild the selected MCU library, then rebuild the application.

### Target connection fails

Check target power, USB cable, SWD/JTAG wiring, selected probe configuration and probe firmware.

### The application links an older library

Run the MCU-library builder again, clean the application and relink it.

## Read next

- [Documentation index](README.md)
- [Quick Reference](quick-reference.md)
- [Architecture overview](architecture/README.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [Dependencies](dependencies.md)
- [Supported Targets](supported-targets.md)
