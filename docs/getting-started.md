# Getting Started with IOsonata

This guide uses [IOcomposer](https://iocomposer.io), the official IDE for IOsonata.

**Typical first run:** 15–30 minutes  
**Prerequisites:** none; the installer supplies the development tools.

## First-run path

```text
Install IOcomposer
    -> build the nRF52832 IOsonata library
    -> open the nRF52832 Blinky project
    -> configure board.h
    -> build, flash and debug
```

The same workflow applies to other supported MCU targets.

## 1. Install IOcomposer

### macOS

```bash
curl -fsSL https://iocomposer.io/install_ioc_macos.sh -o /tmp/install_ioc_macos.sh && bash /tmp/install_ioc_macos.sh
```

### Linux

```bash
curl -fsSL https://iocomposer.io/install_ioc_linux.sh -o /tmp/install_ioc_linux.sh && bash /tmp/install_ioc_linux.sh
```

### Windows — PowerShell as Administrator

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://iocomposer.io/install_ioc_windows.ps1 | iex"
```

The default installation root is `~/IOcomposer` on macOS/Linux and `%USERPROFILE%\IOcomposer` on Windows.

The installation includes:

- IOcomposer;
- Arm and RISC-V toolchains;
- OpenOCD;
- IOsonata;
- the project wizard;
- supported SDK integration;
- the MCU-library build scripts.

The repository also provides the foundation installer scripts under [`Installer/`](../Installer/) for installations without the AI assistance layer.

## 2. Build the precompiled MCU library

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

The builder discovers the supported MCU library projects and presents an interactive menu. Select the MCU used by your board, or select **Build All**.

For the primary first-run path, select **nRF52832**.

The builder produces:

```text
<MCU library project>/Debug/libIOsonata_<MCU>.a
<MCU library project>/Release/libIOsonata_<MCU>.a
```

Every application using the selected MCU and build profile links the same library binary. IOsonata is not rebuilt for the board, application, peripheral selection or RTOS.

When TaktOS is installed, the builder also builds detected TaktOS architecture libraries. Use `--no-taktos` on macOS/Linux or `-NoTaktos` on Windows to build IOsonata only.

## 3. Open the Blinky example

1. Launch IOcomposer.
2. Select **File → Open Projects from File System...**
3. Browse to:
   `~/IOcomposer/IOsonata/ARM/Nordic/nRF52/nRF52832/exemples/Blinky/`
4. Enable **Search for nested projects**.
5. Select the discovered project and finish the import.

Use an IDK-BLYST-NANO/BLYST Nano or Nordic nRF52 DK for this first run.

## 4. Configure the board

Edit the project `board.h` for the actual hardware.

Board data normally includes:

- pin assignments;
- `McuOsc_t` oscillator selection;
- external-device configuration;
- board-level constants.

Example pin definitions:

```c
#define LED1_PORT       0
#define LED1_PIN        17
#define LED1_PINOP      0

#define UART_TX_PORT    0
#define UART_TX_PIN     6
#define UART_TX_PINOP   0
```

Changing board data does not rebuild the IOsonata MCU library.

## 5. Build the application

1. Select the Debug or Release application configuration.
2. Right-click the project in Project Explorer.
3. Select **Build Project**.
4. Check the Console for compiler, linker and size output.

The application build compiles the application and optional kernel, then links the existing IOsonata library.

```text
application sources
+ board.h
+ optional kernel
+ libIOsonata_<MCU>.a
= firmware image
```

## 6. Flash and debug

### IDAP-Link

1. Connect IDAP-Link to the target SWD/JTAG pins.
2. Select the matching OpenOCD debug configuration.
3. Start debugging.
4. IOcomposer flashes the firmware and halts at `main()`.
5. Press **F8** to resume.

IDAP-Link is I-SYST's CMSIS-DAP debug probe with SWD/JTAG and USB-UART support.

J-Link, ST-Link and other supported probes can be used with the appropriate target configuration.

## Create a new application

After confirming the working example:

1. Select **File → New → Project → IOsonata Project**.
2. Enter the project name and select C++ or C.
3. Select the Arm or RISC-V toolchain.
4. Enter author and licence information.
5. Select the MCU family and target.
6. Select application features such as UART, I2C, SPI, USB or Bluetooth where offered.
7. Finish the wizard.

The feature selections configure the application project. They do not generate another IOsonata HAL binary.

A generated project contains application-owned files such as:

```text
my_project/
├── include/
│   ├── board.h
│   └── my_project.h
└── src/
    └── main.cpp
```

The project links the precompiled library for the selected MCU and build profile.

## Understanding shared example source

Reusable example source lives under [`exemples/`](../exemples/) and is organized by category:

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

Target application projects reference the shared example source. The source is not copied for every MCU target.

This source sharing is separate from the MCU implementation. The target application still links the precompiled IOsonata library.

## Recommended examples

Project names below are available on the nRF52832 target, with many also available on other targets.

### Start here

- `Blinky` — `exemples/misc/blinky.c`
- `UartRetargetDemo` — `exemples/uart/uart_retarget_demo.cpp`

### Communication

- `I2CMasterDemo` — `exemples/i2c/i2c_master_demo.cpp`
- `SPIMasterDemo` — `exemples/spi/spi_master_demo.cpp`
- `UartPrbsTxTest` — `exemples/uart/uart_prbs_tx.cpp`

### Storage

- `FlashMemoryDemo`
- `EepromDemo`
- NVM and DiskIO tests under `exemples/storage/`

### Bluetooth

- `BleAdvertiser`
- `UartBleDemo`
- `UartBleFreeRTOS`
- `UartBleTaktOS`

### Sensors

- `MotionSensorDemo`
- `TPHDemo`

## Add an I2C device

Add the board pins:

```c
#define I2C0_SDA_PORT    0
#define I2C0_SDA_PIN     26
#define I2C0_SCL_PORT    0
#define I2C0_SCL_PIN     27
```

Configure the interface in `main.cpp`:

```cpp
#include "coredev/i2c.h"

static const IOPinCfg_t s_I2cPins[] = {
    {I2C0_SDA_PORT, I2C0_SDA_PIN, 0, IOPINDIR_BI,
     IOPINRES_NONE, IOPINTYPE_NORMAL},
    {I2C0_SCL_PORT, I2C0_SCL_PIN, 0, IOPINDIR_OUTPUT,
     IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const I2CCfg_t s_I2cCfg = {
    .DevNo = 0,
    .Type = I2CTYPE_STANDARD,
    .Mode = I2CMODE_MASTER,
    .pIOPinMap = s_I2cPins,
    .NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
    .Rate = 100000,
};

I2C g_I2c;

// In HardwareInit():
g_I2c.Init(s_I2cCfg);
```

A sensor driver then receives `&g_I2c` through `DeviceIntrf *`. The same driver can receive a compatible SPI interface without becoming a separate sensor type.

## UART output

Projects using UART retargeting can use standard output:

```cpp
#include <stdio.h>

printf("Sensor reading: %d\n", value);
```

Use CoolTerm, `screen`, PuTTY, TeraTerm or another serial terminal at the configured rate.

## Using an RTOS

IOsonata works with bare metal, TaktOS, FreeRTOS and ThreadX.

The same selected `libIOsonata_<MCU>.a` is linked by every execution model. The kernel is compiled and linked above the IOsonata hardware and device layer.

Examples include:

- `UartPrbsTxTestFreeRTOS`;
- `UartPrbsTxTestTaktOS`;
- `UartBleFreeRTOS`;
- `UartBleTaktOS`.

## Updating IOsonata

After updating or changing IOsonata source:

1. run the platform library builder;
2. rebuild the affected MCU library once;
3. clean and relink the applications using that library;
4. update application code if a public API changed.

Do not create another HAL build for every board or product.

## Troubleshooting

### Project wizard is missing

Rerun the IOcomposer installer. The installer owns the IDE and wizard integration.

### Arm compiler is not found

Rerun the installer and verify that the installed toolchain directory is on `PATH`.

### Nordic symbols are unresolved

Verify that the installer completed the required Nordic SDK checkout under `~/IOcomposer/external/`, then rebuild the MCU library before rebuilding the application.

### Target connection fails

Check:

- target power;
- USB cable;
- SWD/JTAG wiring;
- selected probe and target configuration;
- probe firmware;
- another USB port.

### Application uses an older IOsonata binary

Run the MCU-library builder again, clean the application and relink it.

## Next documentation

- [Architecture overview](architecture/README.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [DeviceIntrf implementer notes](architecture/devintrf-implementer-notes.md)
- [Device composition](architecture/device-composition.md)
- [Quick reference](quick-reference.md)
- [Dependencies](dependencies.md)
- [Supported targets](supported-targets.md)

## Support

- Documentation: `~/IOcomposer/IOsonata/docs/`
- Examples: `~/IOcomposer/IOsonata/exemples/`
- GitHub Issues: https://github.com/IOsonata/IOsonata/issues
- GitHub Discussions: https://github.com/IOsonata/IOsonata/discussions
