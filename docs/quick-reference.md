# IOsonata Quick Reference

Fast lookup for common IOcomposer and IOsonata tasks.

## Install IOcomposer

### macOS

```bash
curl -fsSL https://iocomposer.io/install_ioc_macos.sh -o /tmp/install_ioc_macos.sh && bash /tmp/install_ioc_macos.sh
```

### Linux

```bash
curl -fsSL https://iocomposer.io/install_ioc_linux.sh -o /tmp/install_ioc_linux.sh && bash /tmp/install_ioc_linux.sh
```

### Windows

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://iocomposer.io/install_ioc_windows.ps1 | iex"
```

## Build an MCU library

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

Select one MCU or **Build All**. The builder discovers current library projects under `lib/ioc/` and produces Debug and Release libraries.

```text
<MCU project>/lib/ioc/Debug/libIOsonata_<MCU>.a
<MCU project>/lib/ioc/Release/libIOsonata_<MCU>.a
```

The selected binary is reused across boards, products, bare metal, TaktOS, FreeRTOS and ThreadX.

## Open an example

1. Select **File → Open Projects from File System...**
2. Browse directly to the target project, for example:
   `~/IOcomposer/IOsonata/ARM/Nordic/nRF52/nRF52832/exemples/Blinky/ioc/`
3. Select the project.
4. Build, flash and debug.

## Create a project

1. Select **File → New → Project → IOsonata Project**.
2. Enter the project name.
3. Select C++ or C.
4. Select the Arm or RISC-V toolchain.
5. Enter author and licence information.
6. Select the MCU family and target.
7. Select application features.
8. Finish the wizard.

Feature selection configures the application. It does not rebuild IOsonata.

## Build configurations

- **Debug:** development and debugging.
- **Release:** optimized application image.

The application configuration must link the matching IOsonata library profile.

## Board configuration

Application-owned board data normally lives in `board.h`.

```c
#define LED1_PORT       0
#define LED1_PIN        17
#define LED1_PINOP      0

#define UART_TX_PORT    0
#define UART_TX_PIN     6
#define UART_TX_PINOP   0
```

Board data may include:

- pin maps;
- oscillator selection;
- external-device configuration;
- board constants.

Changing board data does not rebuild the MCU library.

## I2C configuration

```cpp
#include "coredev/i2c.h"

static const IOPinCfg_t s_I2cPins[] = {
    {0, 26, 0, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {0, 27, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
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
g_I2c.Init(s_I2cCfg);
```

## SPI configuration

```cpp
#include "coredev/spi.h"

static const IOPinCfg_t s_SpiPins[] = {
    {0, 25, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {0, 24, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {0, 23, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const SPICfg_t s_SpiCfg = {
    .DevNo = 0,
    .Phy = SPIPHY_NORMAL,
    .Mode = SPIMODE_MASTER,
    .pIOPinMap = s_SpiPins,
    .NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
    .Rate = 4000000,
    .DataSize = 8,
    .MaxRetry = 5,
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK,
    .ClkPol = SPICLKPOL_LOW,
    .ChipSel = SPICSEL_AUTO,
};

SPI g_Spi;
g_Spi.Init(s_SpiCfg);
```

## Inject an interface into a device

```cpp
I2C i2c;
SPI spi;
Timer timer;
AccelLsm303agr accel;

DeviceIntrf *intrf = useSpi
    ? static_cast<DeviceIntrf *>(&spi)
    : static_cast<DeviceIntrf *>(&i2c);

accel.Init(accelCfg, intrf, &timer);
```

The sensor implementation remains the same. Only the injected interface changes.

## UART output

```cpp
#include <stdio.h>
printf("Value: %d\n", value);
```

Use a serial terminal at the configured rate.

## Build and debug

- Build: right-click project → **Build Project**.
- Clean: right-click project → **Clean Project**.
- Start debug: select the configured OpenOCD or probe launch.
- Resume: **F8**.
- Step into: **F5**.
- Step over: **F6**.
- Step return: **F7**.

Supported probes depend on the target configuration and can include IDAP-Link, J-Link, ST-Link and other CMSIS-DAP probes.

## Common examples

| Task | Target project / shared source |
|---|---|
| Blink an LED | `Blinky` / `exemples/misc/blinky.c` |
| UART printf | `UartRetargetDemo` / `exemples/uart/uart_retarget_demo.cpp` |
| UART throughput | `UartPrbsTxTest` / `exemples/uart/uart_prbs_tx.cpp` |
| I2C master | `I2CMasterDemo` / `exemples/i2c/i2c_master_demo.cpp` |
| SPI master | `SPIMasterDemo` / `exemples/spi/spi_master_demo.cpp` |
| Bluetooth advertising | `BleAdvertiser` |
| UART over Bluetooth | `UartBleDemo` |
| FreeRTOS integration | `UartBleFreeRTOS`, `UartPrbsTxTestFreeRTOS` |
| TaktOS integration | `UartBleTaktOS`, `UartPrbsTxTestTaktOS` |
| NVM/storage | projects using `exemples/storage/` |
| Motion/environment sensors | `MotionSensorDemo`, `TPHDemo` |

## Change board, MCU or RTOS

### Different board, same MCU

Change `board.h`. Reuse the same IOsonata library.

### Different MCU

Run the library builder and select the new MCU. Create or open the corresponding target application project under its current `ioc/` directory.

### Different execution model

Compile the application with bare metal, TaktOS, FreeRTOS or ThreadX and link the same selected IOsonata library.

## Update IOsonata

```bash
cd ~/IOcomposer/IOsonata
git pull
```

Then:

1. run the MCU-library builder;
2. rebuild the affected MCU library once;
3. clean and relink the application.

## Troubleshooting

### Project wizard missing

Rerun the IOcomposer installer.

### Toolchain missing

Verify the installed Arm or RISC-V compiler is on `PATH`, or rerun the installer.

### Unresolved target symbols

Verify the required SDK under `~/IOcomposer/external/`, rebuild the MCU library, then rebuild the application.

### Old API or binary mismatch

Rebuild the selected MCU library, clean the application and update application code using current examples.

### Target connection failure

Check power, cable, SWD/JTAG wiring, probe firmware and selected target configuration.

## Architecture links

- [Architecture overview](architecture/README.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [DeviceIntrf implementer notes](architecture/devintrf-implementer-notes.md)
- [Device composition](architecture/device-composition.md)
- [Getting started](getting-started.md)
- [Dependencies](dependencies.md)
- [Supported targets](supported-targets.md)
