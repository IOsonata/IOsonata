# Getting Started with IOsonata

This guide walks you through creating your first IOsonata project using the official IOcomposer workflow.

**Time required**: 15-30 minutes  
**Prerequisites**: None (installer handles everything)

---

## Quick Start

### Step 1: Install the Development Environment

You have two installation paths. Both produce the same IOcomposer managed project workflow — the same build system, the same wizard, the same project model. The only difference is AI assistance.

#### Option A — IOcomposer (Recommended)

IOcomposer runs the standard IOsonata installer and then adds two things on top:

- **IOcomposer AI plugin** — installed into the IDE's `dropins/` directory for design-time AI assistance
- **External SDK index** — builds a searchable index over your installed vendor SDKs for context-aware AI queries

👉 https://iocomposer.io

**macOS:**
```bash
/bin/bash -c "$(curl -fsSL https://iocomposer.io/install_ioc_macos.sh)"
```

**Linux:**
```bash
/bin/bash -c "$(curl -fsSL https://iocomposer.io/install_ioc_linux.sh)"
```

**Windows (PowerShell as Administrator):**
```powershell
$u   = "https://iocomposer.io/install_ioc_windows.ps1"
$dst = "$env:TEMP\install_ioc_windows.ps1"
irm $u -OutFile $dst
powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -File $dst
```

The default installation root is `~/IOcomposer`. Override with `--home <path>`.

#### Option B — IOsonata Installer (Foundation Only)

If you prefer the development environment without the AI layer:

**macOS:**
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_macos.sh)"
```

**Linux:**
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_linux.sh)"
```

**Windows (PowerShell as Administrator):**
```powershell
$u   = "https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_win.ps1"
$dst = "$env:TEMP\install_iocdevtools_win.ps1"
irm $u -OutFile $dst
powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -File $dst
```

#### What both options install

- The IOcomposer IDE platform (Embedded CDT)
- ARM/RISC-V toolchains
- OpenOCD debugger
- IOsonata framework
- Project wizard plugin

**Installation size:** ~1GB

---

### Step 2: Build the Precompiled MCU Library

After installation, run the platform-specific library builder.

**macOS:**
```bash
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_macos.sh
```

**Linux:**
```bash
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_linux.sh
```

**Windows (PowerShell):**
```powershell
& "$env:USERPROFILE\IOcomposer\IOsonata\Installer\build_iosonata_lib_win.ps1"
```

The builder discovers the supported MCU library projects and presents an interactive menu. Select the MCU used by your board, or choose **Build All**.

For the primary first-run path, select **nRF52832**, then use the `Blinky` project with an IDK-BLYST-NANO/BLYST Nano or Nordic nRF52 DK.

It then runs the IOcomposer managed builder headlessly and produces both configurations:

```text
<MCU library project>/Debug/libIOsonata_<MCU>.a
<MCU library project>/Release/libIOsonata_<MCU>.a
```

Every application using that MCU and build configuration links the same library binary. The library is not rebuilt for the board, application, peripheral selection or RTOS.

When TaktOS is installed, the builder also builds all detected TaktOS ARM and RISC-V libraries. Use `--no-taktos` on macOS/Linux or `-NoTaktos` on Windows to build IOsonata only.

---

### Step 3: Create Your First Project

1. **Launch IOcomposer** (installed by the script)

2. **File → New → Project → IOsonata Project → Next**

3. **Page 1 - Project Configuration:**
   - **Project name**: e.g., "my_first_project"
   - **Project type**: C++ Project (recommended) or C Project
   - **Toolchain**: GNU ARM Embedded (for ARM) or GNU RISC-V Embedded
   - Click **Next**

4. **Page 2 - Copyright & License:**
   - **Author Name**: Your name
   - **Copyright Holder**: Your company/name
   - **License**: MIT, Apache 2.0, BSD 3-Clause, GPL v3, LGPL v3, Proprietary, or Custom
   - Click **Next**

5. **Page 3 - MCU & Peripherals:**
   
   **IOsonata Location:**
   - Path to IOsonata installation (default: auto-detected)
   
   **MCU Selection:**
   - **Family**: Nordic nRF52, Nordic nRF54, Nordic nRF91, ST STM32, or Renesas RE01
   - **Target**: Select specific chip (e.g., NRF52832_XXAA, STM32L476)
   
   **BLE Configuration** (shown only for nRF52):
   - **Mode**: Disable, Advertiser/Broadcast, Peripheral, or Central
   
   **nRF91 Features** (shown only for nRF91):
   - ☑ Enable modem
   - ☑ Enable GNSS
   
   **Peripherals** (checkboxes):
   - ☑ UART (checked by default)
   - ☐ I2C
   - ☐ SPI
   - ☐ USB (enabled only for nRF52840 and some STM32)

6. **Click Finish**

The wizard generates a complete project:
```
my_first_project/
├── include/
│   ├── board.h         # Pin assignments (customize this!)
│   └── my_first_project.h
└── src/
    └── main.c          # Application code (or main.cpp for C++)
```

The wizard configures the application project to link the previously built IOsonata MCU library. MCU and peripheral selections configure the application; they do not generate or rebuild a different HAL.

---

### Step 4: Customize Hardware Pins

**Edit `include/board.h`** to match your actual board:

```c
// TODO: Define LED pins based on your board
#define LED1_PORT               0
#define LED1_PIN                17     // ← Change to your board's LED pin
#define LED1_PINOP              0

// TODO: Configure UART pins for your board
#define UART_TX_PORT            0
#define UART_TX_PIN             6      // ← Change to match your board
#define UART_TX_PINOP           0
```

Look for `TODO:` comments—these show what needs customization.

---

### Step 5: Build

1. **Right-click project** in Project Explorer
2. **Select "Build Project"**
3. **Watch Console** tab for build progress

**Expected output:**
```
Building target: my_first_project.elf
arm-none-eabi-size my_first_project.elf
   text    data     bss     dec     hex filename
  12340     256     512   13108    3334 my_first_project.elf
Build Finished (took 3.2s)
```

**Success!** Your firmware is built.

---

### Step 6: Flash to Hardware

#### Using IDAP-Link (Recommended)

1. **Connect IDAP-Link** to your board's SWD/JTAG pins
2. **Right-click project** → Debug As → GDB OpenOCD Debugging
3. **IOcomposer flashes firmware** and halts at `main()`
4. **Press F8** (Resume) to run

**IDAP-Link** is I-SYST's CMSIS-DAP debug probe. It works with OpenOCD and PyOCD, supporting both SWD and JTAG protocols.

#### Using Other Debug Probes

**J-Link or other CMSIS-DAP probes**: Similar workflow, select appropriate debug configuration in IOcomposer.

**Your firmware is now running on hardware!**

---

## Next Steps

### Understanding the Project Structure

**Generated files:**
- `board.h` - Board pin, oscillator and external-device configuration
- `<project>.h` - Application configuration
- `main.c` or `main.cpp` - Application code
- `.cproject` - IOcomposer managed-build configuration
- `.project` - Project metadata

The application includes IOsonata headers and links the precompiled library for the selected MCU and build profile:

```text
Application sources
    + board.h
    + libIOsonata_<MCU>.a
    → application firmware
```

Some projects also show linked resources for shared example or application source. Those links avoid copying reusable source into every target project. They do not cause the IOsonata MCU implementation to be rebuilt.

**Why this matters:**
- The same IOsonata binary is reused by bare-metal, TaktOS, FreeRTOS and ThreadX applications
- Board and application changes do not rebuild the HAL
- Updating IOsonata itself requires rebuilding the MCU library once, not rebuilding a separate HAL variant for every product

---

## Learning from Examples

IOsonata example code is written once and shared by every MCU target.
The source lives in `exemples/`, organized by category (`uart/`, `i2c/`,
`spi/`, `storage/`, `bluetooth/`, `sensor/`, `misc/`, ...). What you
open and build is the per-MCU project under each target's `exemples/`
directory; it links that shared source. The same example code runs on
every supported MCU.

### Opening an Example Project

1. **File** → **Open Projects from File System...**
2. **Browse** to a target project folder, for example:
   `~/IOcomposer/IOsonata/ARM/Nordic/nRF52/nRF52832/exemples/Blinky/Eclipse/`
   (pattern: `ARM/[Vendor]/[Series]/[MCU]/exemples/[Example]/Eclipse/`)
3. **Enable** "Search for nested projects"
4. **Click Finish**
5. **Build and flash** (same as above)

### Recommended Examples to Try

Project names below are from the nRF52832 target; most exist for the
other targets too. The shared source file is named beside each.

**Start here:**
- `Blinky` - LED toggling (`exemples/misc/blinky.c`)
- `UartRetargetDemo` - printf over UART (`exemples/uart/uart_retarget_demo.cpp`)

**Add peripherals:**
- `I2CMasterDemo` - I2C master transfers (`exemples/i2c/i2c_master_demo.cpp`)
- `SPIMasterDemo` - SPI master transfers (`exemples/spi/spi_master_demo.cpp`)
- `FlashMemoryDemo` - external flash storage (`exemples/storage/flash_memory_demo.cpp`)

**BLE (Nordic only):**
- `BleAdvertiser` - Bluetooth advertising (`exemples/bluetooth/ble_advertiser.cpp`)
- `UartBleDemo` - BLE UART bridge (`exemples/bluetooth/uart_ble.cpp`)

**Sensors:**
- `MotionSensorDemo`, `TPHDemo` - motion and environmental sensors (`exemples/sensor/`)

---

## Common Customizations

### Adding I2C Sensor

1. **Add pins to `board.h`:**
```c
#define I2C0_SDA_PORT    0
#define I2C0_SDA_PIN     26
#define I2C0_SCL_PORT    0
#define I2C0_SCL_PIN     27
```

2. **Configure in `main.cpp`:**
```cpp
#include "coredev/i2c.h"

static const IOPinCfg_t s_I2cPins[] = {
    {I2C0_SDA_PORT, I2C0_SDA_PIN, 0, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {I2C0_SCL_PORT, I2C0_SCL_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const I2CCfg_t s_I2cCfg = {
    .DevNo = 0,
    .Type = I2CTYPE_STANDARD,
    .Mode = I2CMODE_MASTER,
    .pIOPinMap = s_I2cPins,
    .NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
    .Rate = 100000,  // 100kHz
};

I2C g_I2c;

// In HardwareInit():
g_I2c.Init(s_I2cCfg);
```

3. **Build and flash**

### Adding UART Debug Output

UART is typically initialized by the wizard. To use it:

```c
#include <stdio.h>

printf("Debug value: %d\n", myValue);
```

View output via serial terminal:

**CoolTerm (recommended - works on macOS, Linux, Windows):**
- Download: https://freeware.the-meiers.org/
- Simple, cross-platform serial terminal
- Connect to your board's COM port at 115200 baud

**Platform-specific alternatives:**
```bash
# Linux/macOS
screen /dev/ttyUSB0 115200

# Windows: PuTTY or TeraTerm
```

---

## Debugging

### Printf Debugging

The wizard configures printf to output via UART automatically.

```c
printf("Sensor reading: %d\n", value);
printf("Error code: 0x%04X\n", error);
```

**View output with CoolTerm** (recommended - cross-platform):
- Download: https://freeware.the-meiers.org/
- Connect to board's COM port at 115200 baud
- Works on macOS, Linux, and Windows

### Debugging with IDAP-Link

**IDAP-Link** is I-SYST's CMSIS-DAP debug probe supporting OpenOCD and PyOCD.

**Setting breakpoints:**
1. Double-click in left margin of code editor
2. Blue dot appears = breakpoint set

**Running debugger:**
1. Connect IDAP-Link to board
2. Right-click project → Debug As → GDB OpenOCD Debugging
3. Debugger halts at `main()`
4. Use toolbar buttons:
   - F5: Step Into
   - F6: Step Over
   - F7: Step Return
   - F8: Resume

**Viewing variables:**
- Hover over variable in code
- Add to "Variables" view
- Add to "Expressions" view

**Alternative debug probes:** J-Link, ST-Link, and other CMSIS-DAP debuggers also work with IOcomposer.

---

## Troubleshooting

### "Project Wizard not found"

**Cause:** Wizard plugin not installed

**Fix:** Run installer again, or manually:
```
Help → Install New Software
Add → Archive → Browse to ~/IOcomposer/IOsonata/Installer/eclipse_plugin/*.jar
```

### "arm-none-eabi-gcc: command not found"

**Cause:** Toolchain not in PATH

**Fix:** Rerun installer, or add manually:
```bash
export PATH="$HOME/.local/xPacks/@xpack-dev-tools/arm-none-eabi-gcc/.content/bin:$PATH"
```

### "Undefined reference to nrf_xxx"

**Cause:** Nordic SDK not installed

**Fix:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK.git
git clone https://github.com/NordicSemiconductor/nrfx.git
```

### "Could not connect to target"

**Cause:** Debugger connection issue

**Fix:**
1. Check USB cable (IDAP-Link or other probe)
2. Verify board power
3. Try different USB port
4. Check debugger firmware version (IDAP-Link, J-Link, etc.)
5. Verify SWD/JTAG connections

### Build Errors After Updating IOsonata

**Cause:** The application is linking an older MCU library, or the application uses an API that changed.

**Fix:**
1. Run the platform library builder again and rebuild the selected MCU library.
2. Clean and rebuild the application.
3. If the API changed, update the application using the current examples under `exemples/`.

---

## Advanced Topics

### Multiple Boards with Same MCU

IOsonata's MCU-centric design makes board variants simple:

1. **Create project** for MCU (e.g., nRF52832)
2. **Customize `board.h`** for Board A
3. **Copy `board.h`** to `board_boardA.h`
4. **Create `board_boardB.h`** with different pins
5. **Switch between boards** by including different header

**Application code stays the same.** Board data may change pins, oscillator selection and external-device configuration without rebuilding IOsonata.

### Using with an RTOS

IOsonata works with bare-metal, event-driven, or RTOS architectures,
and the same precompiled library serves all of them.

Real integration examples per target: `UartBleFreeRTOS` and
`UartPrbsTxTestFreeRTOS` (FreeRTOS), `UartBleTaktOS` and
`UartPrbsTxTestTaktOS` (TaktOS), under
`ARM/Nordic/nRF52/[MCU]/exemples/`. The shared sources are
`exemples/uart/uart_prbs_tx_freertos.cpp` and
`uart_prbs_tx_taktos.cpp`.

### Cross-Platform Development

**macOS/Linux/Windows:**
- Same IOcomposer workspace
- Same projects
- Same builds
- Portable development

The installer handles platform differences automatically.

---

## Next Documentation

Once you're comfortable with basic workflows:

- **`quick-reference.md`** - Fast lookup for common tasks
- **`dependencies.md`** - External SDK requirements
- **`supported-targets.md`** - What MCUs are validated
- **`architecture/eclipse-workflow.md`** - Why linked resources matter

---

## Support

- **Documentation**: `~/IOcomposer/IOsonata/docs/`
- **Examples**: `~/IOcomposer/IOsonata/exemples/`
- **GitHub Issues**: https://github.com/IOsonata/IOsonata/issues
- **GitHub Discussions**: https://github.com/IOsonata/IOsonata/discussions

---

**You're ready to build professional embedded firmware with IOsonata!** 🚀
