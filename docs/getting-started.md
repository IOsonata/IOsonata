# Getting Started with IOsonata

This guide walks you through creating your first IOsonata project using the official Eclipse-based workflow.

**Time required**: 15-30 minutes  
**Prerequisites**: None (installer handles everything)

---

## Quick Start

### Step 1: Install the Development Environment

You have two installation paths. Both produce the same Eclipse CDT managed project workflow — the same build system, the same wizard, the same project model. The only difference is AI assistance.

#### Option A — IOcomposer (Recommended)

IOcomposer runs the standard IOsonata installer and then adds two things on top:

- **IOcomposer AI plugin** — installed into Eclipse's `dropins/` directory for design-time AI assistance
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
$u   = "https://iocomposer.io/install_ioc_win.ps1"
$dst = "$env:TEMP\install_ioc_win.ps1"
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

- Eclipse Embedded CDT
- ARM/RISC-V toolchains
- OpenOCD debugger
- IOsonata framework
- Project wizard plugin

**Installation size:** ~1GB

---

### Step 2: Create Your First Project

1. **Launch Eclipse** (installed by the script)

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

---

### Step 3: Customize Hardware Pins

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

### Step 4: Build

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

### Step 5: Flash to Hardware

#### Using IDAP-Link (Recommended)

1. **Connect IDAP-Link** to your board's SWD/JTAG pins
2. **Right-click project** → Debug As → GDB OpenOCD Debugging
3. **Eclipse flashes firmware** and halts at `main()`
4. **Press F8** (Resume) to run

**IDAP-Link** is I-SYST's CMSIS-DAP debug probe. It works with OpenOCD and PyOCD, supporting both SWD and JTAG protocols.

#### Using Other Debug Probes

**J-Link or other CMSIS-DAP probes**: Similar workflow, select appropriate debug configuration in Eclipse.

**Your firmware is now running on hardware!**

---

## Next Steps

### Understanding the Project Structure

**Generated files:**
- `board.h` - Hardware pin definitions (customize this)
- `<project>.h` - Application configuration
- `main.c` - Your application code
- `.cproject` - Eclipse build configuration
- `.project` - Eclipse project metadata

**Linked resources** (in Project Explorer):
```
my_project/
├── Linked Resources/
│   ├── IOsonata/       # Framework code
│   ├── ARM/            # Architecture code
│   └── Nordic/         # Vendor code (if applicable)
```

These link to `~/IOcomposer/IOsonata/` - **not copied**, shared across all projects.

**Why this matters:**
- Framework updates propagate to all projects automatically
- Zero code duplication
- Fixes apply everywhere

---

## Learning from Examples

IOsonata includes reference examples showing how to use specific features.

### Opening an Example Project

1. **File** → **Open Projects from File System...**
2. **Browse** to example folder:
   - Generic: `~/IOcomposer/IOsonata/exemples/Blink/`
   - Target-specific: `~/IOcomposer/IOsonata/ARM/Nordic/nRF52/nRF52832/exemples/Blink/Eclipse/`
3. **Enable** "Search for nested projects"
4. **Click Finish**
5. **Build and flash** (same as above)

### Recommended Examples to Try

**Start here:**
- `exemples/Blink/` - LED toggling
- `exemples/UartPrintf/` - Serial debugging with printf

**Add peripherals:**
- `exemples/I2CScanner/` - Scan I2C bus for devices
- `exemples/SpiFlash/` - SPI flash memory operations

**BLE (Nordic only):**
- `exemples/BleAdvertiser/` - Bluetooth advertising
- `exemples/BleUart/` - BLE UART service

**Sensors:**
- `exemples/sensor/` - Motion sensors, environmental sensors

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

2. **Configure in `main.c`:**
```c
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

### Eclipse Debugger with IDAP-Link

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

**Alternative debug probes:** J-Link, ST-Link, and other CMSIS-DAP debuggers also work with Eclipse.

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

**Cause:** Project uses old API

**Fix:**
1. Check GitHub issues for breaking changes
2. Update project code to match new API
3. See `exemples/` for updated usage patterns

---

## Advanced Topics

### Multiple Boards with Same MCU

IOsonata's MCU-centric design makes board variants simple:

1. **Create project** for MCU (e.g., nRF52832)
2. **Customize `board.h`** for Board A
3. **Copy `board.h`** to `board_boardA.h`
4. **Create `board_boardB.h`** with different pins
5. **Switch between boards** by including different header

**Application code stays the same.** Only pins change.

### Using with FreeRTOS

IOsonata works with bare-metal, event-driven, or RTOS architectures.

See `exemples/FreeRTOS/` for integration examples.

### Cross-Platform Development

**macOS/Linux/Windows:**
- Same Eclipse workspace
- Same projects
- Same builds
- Portable development

The installer handles platform differences automatically.

---

## Alternative Build Systems (Advanced)

**Official support:** Eclipse Embedded CDT (as described above)

**Community alternatives:** Some advanced users have created custom Makefiles or CMake configurations.

**These are not officially supported.** If you need non-Eclipse builds, you'll need to:
- Understand ARM toolchains deeply
- Write your own build files
- Manage dependencies manually
- Handle linker scripts yourself

**See:** Community build files (if available) or write your own.

**For 99% of users:** Use Eclipse as described in this guide.

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
