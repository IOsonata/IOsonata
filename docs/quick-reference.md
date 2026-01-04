# IOsonata Quick Reference

Fast lookup for common tasks using Eclipse Embedded CDT.

---

## Project Creation

### Create New Project

1. File → New → Project → IOsonata Project → Next
2. **Page 1:** Project name, C/C++ type, toolchain → Next
3. **Page 2:** Author, copyright, license → Next
4. **Page 3:** MCU family, target, BLE mode, peripherals → Finish

**IMPORTANT:** Edit `include/board.h` to set pins for your board

### Open Example Project

1. File → Open Projects from File System...
2. Browse to: `exemples/Blink/Eclipse/` (or target-specific example)
3. Enable "Search for nested projects"
4. Click Finish

---

## Build & Flash

### Build Project

Right-click project → Build Project  
Or: **Ctrl+B** (Cmd+B on macOS)

### Clean Build

Right-click project → Clean Project  
Then build again

### Flash Firmware

1. Connect IDAP-Link to board
2. Right-click project → Debug As → GDB OpenOCD Debugging
3. Firmware flashes automatically
4. Press **F8** to run

**Alternative debug probes:** J-Link, ST-Link work similarly

### Release Build

Right-click project → Build Configurations → Set Active → Release  
Build Project

---

## Hardware Configuration

### Customize Pins (Required)
**File:** `include/board.h`

```c
// LEDs
#define LED1_PORT     0
#define LED1_PIN      17
#define LED1_PINOP    0

// UART
#define UART_TX_PORT  0
#define UART_TX_PIN   6
#define UART_TX_PINOP 0
```

### Add I2C
**Add to `board.h`:**
```c
#define I2C0_SDA_PORT    0
#define I2C0_SDA_PIN     26
#define I2C0_SCL_PORT    0
#define I2C0_SCL_PIN     27
```

**Add to `main.c`:**
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
    .NbIOPins = 2,
    .Rate = 100000,  // 100kHz
};

I2C g_I2c;
// In HardwareInit(): g_I2c.Init(s_I2cCfg);
```

### Add SPI
**Add to `board.h`:**
```c
#define SPI_SCK_PORT     0
#define SPI_SCK_PIN      25
#define SPI_MISO_PORT    0
#define SPI_MISO_PIN     24
#define SPI_MOSI_PORT    0
#define SPI_MOSI_PIN     23
```

**Add to `main.c`:**
```c
#include "coredev/spi.h"

static const IOPinCfg_t s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const SPICfg_t s_SpiCfg = {
    .DevNo = 0,
    .Phy = SPIPHY_NORMAL,
    .Mode = SPIMODE_MASTER,
    .pIOPinMap = s_SpiPins,
    .NbIOPins = 3,
    .Rate = 4000000,  // 4MHz
    .DataSize = 8,
    .MaxRetry = 5,
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK,
    .ClkPol = SPICLKPOL_LOW,
    .ChipSel = SPICSEL_AUTO,
};

SPI g_Spi;
// In HardwareInit(): g_Spi.Init(s_SpiCfg);
```

---

## Debugging

### Printf Debugging

```c
#include <stdio.h>

printf("Value: %d\n", value);
printf("Error: 0x%04X\n", errorCode);
```

**View output:**

CoolTerm (recommended - cross-platform):
- Download: https://freeware.the-meiers.org/
- Connect to COM port at 115200 baud

Linux/macOS alternatives:
```bash
screen /dev/ttyUSB0 115200
minicom -D /dev/ttyUSB0 -b 115200
```

Windows alternatives: PuTTY, TeraTerm

### Set Breakpoints

Double-click in left margin of code editor  
Blue dot appears = breakpoint set

### Debug Controls

- **F5** - Step Into
- **F6** - Step Over
- **F7** - Step Return
- **F8** - Resume
- **F9** - Toggle Breakpoint

### View Variables

- Hover over variable in code
- Add to "Variables" view
- Add to "Expressions" view

---

## File Locations

### Generic Examples
```
~/IOcomposer/IOsonata/exemples/
├── Blink/
├── UartPrintf/
├── I2CScanner/
├── SpiFlash/
└── sensor/
```

### Target-Specific Examples
```
~/IOcomposer/IOsonata/ARM/[Vendor]/[Series]/[MCU]/exemples/
```

**Nordic nRF52832:**
```
~/IOcomposer/IOsonata/ARM/Nordic/nRF52/nRF52832/exemples/
├── Blink/Eclipse/
├── UartPrintf/Eclipse/
└── BleAdvertiser/Eclipse/
```

### Your Projects
```
~/IOcomposer/workspace/
├── project1/
├── project2/
└── project3/
```

### Dependencies
```
~/IOcomposer/external/
├── nRF5_SDK/        # Nordic SDK
├── nrfx/            # Nordic HAL
├── BSEC/            # Bosch sensor library
└── lwip/            # Network stack
```

---

## Common Tasks

### Change MCU Target

Right-click project → Properties → C/C++ Build → Settings  
Adjust MCU-specific defines and linker script

**Better approach:** Create new project with wizard for different MCU

### Switch Between Board Variants
Create multiple `board_*.h` files:
```c
// board_devkit.h - Development board pins
// board_production.h - Production board pins

// In main.c:
#include "board_devkit.h"  // Or board_production.h
```

### Add Sensor Driver
```c
#include "sensors/bme680_sensor.h"

Bme680 g_Bme680;
g_Bme680.Init(config);
```

See `exemples/sensor/` for usage examples.

### Change UART Baud Rate
Edit in `main.c`:
```c
static const UARTCfg_t s_UartCfg = {
    .Rate = 115200,  // Change this
    // ...
};
```

---

## MCU-Specific

### Nordic nRF52/nRF54

**Linker scripts:**
- With BLE (Softdevice): `s132_nrf52832_xxaa.ld`
- Without BLE: `gcc_nrf52832_xxaa.ld`
- With MBR (DFU support): `mbr_nrf52832_xxaa.ld`

**Flash Softdevice (if using BLE):**

Using IDAP-Link with IDAPnRFProg (recommended):
- IDAPnRFProg automatically merges application + softdevice
- No separate mergehex step needed

Using other tools:
- Find softdevice hex: `~/IOcomposer/external/nRF5_SDK/components/softdevice/s132/hex/`
- Merge with Nordic mergehex utility, then flash

**Nordic dependencies:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK.git
git clone https://github.com/NordicSemiconductor/nrfx.git
```

### STM32

**Linker scripts:**  
`~/IOcomposer/IOsonata/ARM/ST/STM32L4xx/STM32L476/ldscript/`

**STM32 dependencies:**
```bash
cd ~/IOcomposer/external
# STM32 HAL/LL drivers (download from ST website)
```

---

## Recommended Examples by Use Case

### "I want to blink an LED"
`exemples/Blink/`

### "I want UART debug output"
`exemples/UartPrintf/`

### "I want to scan I2C bus"
`exemples/I2CScanner/`

### "I want BLE advertising"
`ARM/Nordic/nRF52/nRF52832/exemples/BleAdvertiser/`

### "I want environmental sensor"
`exemples/sensor/` - Look for BME680 or similar

### "I want motion tracking"
`exemples/sensor/` - Look for ICM-20948 or similar

### "I want to use FreeRTOS"
`exemples/FreeRTOS/`

---

## Troubleshooting

### Build fails: "undefined reference to nrf_xxx"
**Fix:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK.git
git clone https://github.com/NordicSemiconductor/nrfx.git
```

### Flash fails: "Could not connect to target"
**Check:**
1. USB cable connected (IDAP-Link or debug probe)
2. Board powered
3. Correct debugger selected (OpenOCD, PyOCD, J-Link)
4. Try different USB port
5. Verify SWD/JTAG connections

### Printf not working
**Check:**
1. UART pins configured in `board.h`
2. UART initialized in `HardwareInit()`
3. CoolTerm (or serial terminal) connected at correct baud rate (115200)
4. Correct COM port selected

### Eclipse indexer errors

Project → C/C++ Index → Rebuild

### Can't find wizard

Help → Install New Software → Add → Archive  
Select: `~/IOcomposer/IOsonata/Installer/eclipse_plugin/*.jar`

---

## Keyboard Shortcuts

### Eclipse
```
Ctrl+B (Cmd+B)     - Build
Ctrl+Space         - Auto-complete
Ctrl+Shift+F       - Format code
Ctrl+F             - Find in file
Ctrl+H             - Search in project
F3                 - Go to definition
Ctrl+/             - Toggle comment
```

### Debugger
```
F5   - Step Into
F6   - Step Over
F7   - Step Return
F8   - Resume
F9   - Toggle Breakpoint
```

---

## Performance Tips

### Optimize Build Time
1. Use Release build for final testing
2. Disable indexer if slow: Project → Properties → C/C++ General → Indexer
3. Increase Eclipse memory: eclipse.ini → -Xmx2048m

### Optimize Flash Time
1. Use Debug build for development (smaller)
2. Use Release build for deployment (optimized)

### Reduce Code Size

Right-click project → Properties → C/C++ Build → Settings → Optimization  
Select: **-Os** (Optimize for size)

---

## Configuration Files

### Project Structure
```
my_project/
├── .cproject          # Build configuration (Eclipse managed)
├── .project           # Project metadata
├── .settings/         # Eclipse settings
├── include/
│   ├── board.h        # CUSTOMIZE THIS
│   └── my_project.h   # Application config
└── src/
    └── main.c         # Application code
```

### Don't Edit Manually
- `.cproject` - Use Eclipse GUI to configure
- `.project` - Use Eclipse GUI to configure

### Edit in Text Editor
- `board.h` - Pin definitions
- `main.c` - Application code
- `my_project.h` - Application config

---

## Advanced

### Using Linked Resources

**Linked resources** in Project Explorer point to shared code:
```
Linked Resources/IOsonata/ → ~/IOcomposer/IOsonata/
```

**Why:**
- Zero code duplication
- Framework updates propagate automatically
- Single source of truth

**See:** `docs/architecture/eclipse-workflow.md`

### Multiple Workspaces

You can have multiple Eclipse workspaces:
```
~/IOcomposer/workspace_project_a/
~/IOcomposer/workspace_project_b/
~/IOcomposer/workspace_experiments/
```

All reference same IOsonata installation.

### Custom Board Definitions

Create board-specific headers:
```c
// boards/myboard_v1.h
#ifndef __MYBOARD_V1_H__
#define __MYBOARD_V1_H__

#define LED1_PORT    0
#define LED1_PIN     13
// ... all pins

#endif
```

Include in project:
```c
#include "boards/myboard_v1.h"
```

---

## Quick Command Reference

### Installation

Run installer for your platform:
- macOS/Linux: See `getting-started.md`
- Windows: Run PowerShell script

### Building (Eclipse)

Right-click project → Build Project

### Flashing (Eclipse)

Connect IDAP-Link → Debug As → GDB OpenOCD Debugging

### Serial Monitor

**CoolTerm (recommended):**
- https://freeware.the-meiers.org/
- Cross-platform (macOS, Linux, Windows)

**Linux/macOS alternatives:**
```bash
screen /dev/ttyUSB0 115200
minicom -D /dev/ttyUSB0 -b 115200
```

---

## Where to Get Help

- **Documentation**: `~/IOcomposer/IOsonata/docs/`
- **Examples**: `~/IOcomposer/IOsonata/exemples/`
- **GitHub Issues**: https://github.com/IOsonata/IOsonata/issues
- **Discussions**: https://github.com/IOsonata/IOsonata/discussions

---

**This reference covers 95% of common tasks. For deeper topics, see full documentation.**
