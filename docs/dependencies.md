# IOsonata Dependencies

External SDKs and libraries required for specific MCU families and features.

---

## Core Dependencies (Installed Automatically)

The IOsonata installer handles these automatically:

### ARM Toolchain
- **gcc-arm-none-eabi** (version 13+)
- **Location**: `~/.local/xPacks/@xpack-dev-tools/arm-none-eabi-gcc/`
- **Used for**: ARM Cortex-M compilation

### RISC-V Toolchain (Optional)
- **gcc-riscv-none-embed**
- **Location**: `~/.local/xPacks/@xpack-dev-tools/riscv-none-embed-gcc/`
- **Used for**: RISC-V compilation

### Debugger
- **OpenOCD** or **PyOCD** (recommended with IDAP-Link)
- **SEGGER J-Link** (alternative)
- **Used for**: Flash and debug

**IDAP-Link** is I-SYST's CMSIS-DAP debug probe supporting both OpenOCD and PyOCD.

### Eclipse
- **Eclipse Embedded CDT**
- **Used for**: Development environment

---

## Vendor SDK Dependencies

These are **NOT installed automatically**. Install as needed for your target MCU.

**Installation location**: `~/IOcomposer/external/`

---

### Nordic Semiconductor (nRF52/nRF53/nRF91/nRF54)

#### nRF5 SDK (Required for Nordic targets)

**Purpose**: BLE stack, peripheral drivers, board support

**Install:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK.git
```

**Why IOsonata fork:**
- C++ compatibility fixes
- Updated nrfx integration
- Maintained actively

**Version**: Based on Nordic SDK 17.1.0

**Used for**: 
- nRF52 series (primary)
- nRF53 series (experimental)
- nRF91 series (experimental)

#### nrfx HAL (Required for Nordic targets)

**Purpose**: Low-level hardware abstraction layer

**Install:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/NordicSemiconductor/nrfx.git
```

**Version**: Latest from Nordic

**Used for**:
- All Nordic nRF52/nRF53/nRF91/nRF54

#### Nordic Mesh SDK (Optional)

**Purpose**: Bluetooth Mesh networking

**Install:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK_Mesh.git
```

**Used for**:
- Bluetooth Mesh applications
- Primarily nRF52

---

### STMicroelectronics (STM32)

#### STM32 HAL/LL Drivers (Required for STM32)

**Purpose**: Hardware abstraction, peripheral drivers

**Install:**
1. Download from ST website: https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html
2. Extract to `~/IOcomposer/external/STM32Cube_FW_[series]/`

**Alternative:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/STMicroelectronics/STM32CubeL4.git  # For L4 series
```

**Used for**:
- STM32L4 series (baseline support)
- Other STM32 families (as added)

---

### Renesas

#### Renesas FSP (Required for Renesas targets)

**Purpose**: Flexible Software Package

**Install:**
Download from Renesas website

**Used for**:
- Renesas RE family (baseline support)

---

## Sensor Libraries

These provide high-level interfaces for specific sensor chips.

### Bosch BSEC (BME680 Air Quality Sensor)

**Purpose**: Advanced air quality calculations for BME680

**Install:**
```bash
cd ~/IOcomposer/external
# Download from Bosch Sensortec:
# https://www.bosch-sensortec.com/software-tools/software/bsec/
```

**License**: Bosch proprietary (free for most uses)

**Used for**:
- BME680 environmental sensor
- Air quality index calculations

**Example**: `exemples/sensor/` (BME680 examples)

### InvenSense Motion Drivers (IMU Sensors)

**Purpose**: Motion processing for InvenSense IMUs

**Install:**
```bash
cd ~/IOcomposer/external
# Download from InvenSense/TDK:
# https://www.invensense.com/developers/
```

**Used for**:
- ICM-20948 (9-axis IMU)
- ICM-42688 (6-axis IMU)
- Other InvenSense/TDK sensors

**Example**: `exemples/sensor/` (IMU examples)

---

## Networking Libraries

### lwIP (Lightweight IP Stack)

**Purpose**: TCP/IP networking for embedded systems

**Install:**
```bash
cd ~/IOcomposer/external
git clone https://git.savannah.nongnu.org/git/lwip.git
```

**Used for**:
- Ethernet connectivity
- Wi-Fi modules (ESP-AT, etc.)
- Network applications

**Example**: Check IOsonata examples for lwIP integration

---

## Filesystem Libraries

### FatFs (FAT Filesystem)

**Purpose**: FAT12/16/32 filesystem support

**Install:**
```bash
cd ~/IOcomposer/external
# Download from: http://elm-chan.org/fsw/ff/00index_e.html
```

**Used for**:
- SD card access
- SPI/SDIO flash with FAT filesystem

### LittleFS (Embedded Filesystem)

**Purpose**: Fail-safe filesystem for embedded flash

**Install:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/littlefs-project/littlefs.git
```

**Used for**:
- Internal flash storage
- Wear-leveling required applications

---

## Quick Reference by Use Case

### "I'm using nRF52832"
**Required:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK.git
git clone https://github.com/NordicSemiconductor/nrfx.git
```

### "I'm using STM32L476"
**Required:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/STMicroelectronics/STM32CubeL4.git
```

### "I'm using BME680 sensor"
**Required:**
```bash
# Download BSEC from Bosch Sensortec website
# Extract to ~/IOcomposer/external/BSEC/
```

### "I need Bluetooth Mesh"
**Required:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK_Mesh.git
```

### "I need TCP/IP networking"
**Required:**
```bash
cd ~/IOcomposer/external
git clone https://git.savannah.nongnu.org/git/lwip.git
```

---

## Dependency Installation Check

### Verify Nordic Dependencies

**Check nRF5_SDK:**
```bash
ls ~/IOcomposer/external/nRF5_SDK/components/softdevice/
# Should show: common/ mbr/ s112/ s113/ s132/ s140/ s212/ s332/ s340/
```

**Check nrfx:**
```bash
ls ~/IOcomposer/external/nrfx/drivers/
# Should show: include/ src/
```

### Verify STM32 Dependencies

```bash
ls ~/IOcomposer/external/STM32CubeL4/Drivers/
# Should show: STM32L4xx_HAL_Driver/ CMSIS/
```

---

## Troubleshooting

### "undefined reference to nrf_xxx"

**Problem**: Nordic SDK not found

**Fix:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK.git
git clone https://github.com/NordicSemiconductor/nrfx.git
```

Then rebuild project in Eclipse.

### "Cannot find stm32l4xx.h"

**Problem**: STM32 HAL not found

**Fix:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/STMicroelectronics/STM32CubeL4.git
```

Then rebuild project in Eclipse.

### "BSEC library not found"

**Problem**: BSEC not installed

**Fix:**
1. Download from Bosch Sensortec website
2. Extract to `~/IOcomposer/external/BSEC/`
3. Follow BSEC integration guide

### Build works on one machine, fails on another

**Problem**: Dependencies not installed on second machine

**Fix:**
Check `~/IOcomposer/external/` on both machines. Install missing dependencies.

---

## Dependency Licenses

**Check licenses before commercial use:**

| Dependency | License | Commercial OK? |
|------------|---------|----------------|
| IOsonata | MIT | ✅ Yes |
| nRF5_SDK | Nordic 5-clause | ✅ Yes |
| nrfx | BSD 3-clause | ✅ Yes |
| STM32Cube | BSD 3-clause | ✅ Yes |
| lwIP | BSD | ✅ Yes |
| FatFs | BSD-style | ✅ Yes |
| LittleFS | BSD 3-clause | ✅ Yes |
| BSEC | Bosch proprietary | ⚠️ Check terms |

**BSEC note**: Free for most uses, but check Bosch license terms for your specific application.

---

## Updating Dependencies

### Update IOsonata
```bash
cd ~/IOcomposer/IOsonata
git pull origin master
```

Eclipse projects automatically see updates (linked resources).

### Update nRF5_SDK
```bash
cd ~/IOcomposer/external/nRF5_SDK
git pull origin master
```

Rebuild affected projects.

### Update STM32Cube
```bash
cd ~/IOcomposer/external/STM32CubeL4
git pull origin master
```

Rebuild affected projects.

---

## Dependency Size Reference

**Approximate sizes:**

| Dependency | Size | Install Time |
|------------|------|--------------|
| IOsonata | ~50MB | 30s |
| nRF5_SDK | ~300MB | 2min |
| nrfx | ~20MB | 30s |
| STM32CubeL4 | ~500MB | 5min |
| lwIP | ~5MB | 10s |
| BSEC | ~10MB | 10s |

**Total for Nordic development**: ~370MB  
**Total for STM32 development**: ~570MB

---

## Offline Development

**All dependencies are local** - no internet required after initial install.

**Portable setup:**
1. Copy `~/IOcomposer/` to USB drive
2. Copy Eclipse installation
3. Work on any machine (same OS)

---

## Advanced: Custom Dependency Locations

**Default**: `~/IOcomposer/external/`

**Custom location**:
1. Right-click project → Properties
2. C/C++ Build → Settings → Include Paths
3. Add custom paths

**Not recommended** - breaks portability between team members.

---

## Summary

**For Nordic development:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/IOsonata/nRF5_SDK.git
git clone https://github.com/NordicSemiconductor/nrfx.git
```

**For STM32 development:**
```bash
cd ~/IOcomposer/external
git clone https://github.com/STMicroelectronics/STM32CubeL4.git
```

**For sensors/networking**: Install as needed per table above.

**Everything else**: Installer handles it.

---

## Support

- **GitHub Issues**: https://github.com/IOsonata/IOsonata/issues
- **Discussions**: https://github.com/IOsonata/IOsonata/discussions
- **Documentation**: `~/IOcomposer/IOsonata/docs/`
