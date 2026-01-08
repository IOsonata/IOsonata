# IOsonata Development Tools - User Guide

Welcome to IOsonata! This guide explains the installation and build scripts available for setting up your IOsonata MCU development environment.

## Table of Contents

- [Overview](#overview)
- [Available Scripts](#available-scripts)
- [Quick Start](#quick-start)
- [Online Installation](#online-installation-no-clone-required)
- [Installation Scripts](#installation-scripts)
- [Clone Scripts](#clone-scripts)
- [Build Scripts](#build-scripts)
- [Common Workflows](#common-workflows)
- [Troubleshooting](#troubleshooting)
- [Platform-Specific Notes](#platform-specific-notes)

---

## Overview

IOsonata is a cross-platform C++ framework for embedded MCU development. These scripts automate the setup of your development environment including:

- **Eclipse Embedded CDT IDE** - For development and debugging
- **ARM and RISC-V toolchains** - Cross-compilers and build tools
- **IOsonata SDK** - The framework source code
- **Dependencies** - External libraries (Nordic SDKs, TinyUSB, LVGL, etc.)
- **Library builds** - Pre-compiled IOsonata libraries for your target MCU

## Available Scripts

#
---

## Online Installation (No Clone Required)

You can run the installation scripts directly from GitHub without cloning the repository first!

### One-Line Installation

**macOS:**
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/refs/heads/master/Installer/install_iocdevtools_macos.sh | bash
```

**Linux:**
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/refs/heads/master/Installer/install_iocdevtools_linux.sh | bash
```

**Windows (PowerShell as Administrator):**
```powershell
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/refs/heads/master/Installer/install_iocdevtools_win.ps1 | iex
```

### Recommended: Download Then Run

For better security, download and review the script first:

**macOS/Linux:**
```bash
# Download
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/refs/heads/master/Installer/install_iocdevtools_macos.sh

# Review (IMPORTANT!)
cat install_iocdevtools_macos.sh | less

# Run
chmod +x install_iocdevtools_macos.sh
./install_iocdevtools_macos.sh
```
**Linux:**
```bash
# Download
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/refs/heads/master/Installer/install_iocdevtools_linux.sh

# Review (IMPORTANT!)
cat install_iocdevtools_linux.sh | less

# Run
chmod +x install_iocdevtools_linux.sh
./install_iocdevtools_linux.sh

**Windows:**
```powershell
# Download
Invoke-WebRequest -Uri https://raw.githubusercontent.com/IOsonata/IOsonata/refs/heads/master/Installer/install_iocdevtools_win.ps1 -OutFile install_iocdevtools_win.ps1

# Review (IMPORTANT!)
notepad install_iocdevtools_win.ps1

# Run
.\install_iocdevtools_win.ps1
```

> **⚠️ Security Note**: Always review scripts before running them, especially when piping from the internet. See [ONLINE_INSTALLATION.md](ONLINE_INSTALLATION.md) for detailed security best practices.

## Installation Scripts (Complete Setup)

Install everything needed for IOsonata development:

| Platform | Script | Description |
|----------|--------|-------------|
| macOS | `install_iocdevtools_macos.sh` | Full macOS development environment |
| Linux | `install_iocdevtools_linux.sh` | Full Linux development environment |
| Windows | `install_iocdevtools_win.ps1` | Full Windows development environment |

### Clone Scripts (Repository Setup)

Clone IOsonata and dependencies without installing tools:

| Platform | Script | Description |
|----------|--------|-------------|
| macOS | `clone_iosonata_sdk_macos.sh` | Clone repositories on macOS |
| Linux | `clone_iosonata_sdk_linux.sh` | Clone repositories on Linux |
| Windows | `clone_iosonata_sdk_win.ps1` | Clone repositories on Windows |

### Build Scripts (Library Building)

Build IOsonata libraries for specific MCU targets:

| Platform | Script | Description |
|----------|--------|-------------|
| macOS | `build_iosonata_lib_macos.sh` | Build libraries on macOS |
| Linux | `build_iosonata_lib_linux.sh` | Build libraries on Linux |
| Windows | `build_iosonata_lib_win.ps1` | Build libraries on Windows |

---

## Quick Start

### For New Users (Complete Setup)

**macOS:**
```bash
./install_iocdevtools_macos.sh
```

**Linux:**
```bash
./install_iocdevtools_linux.sh
```

**Windows (PowerShell as Administrator):**
```powershell
.\install_iocdevtools_win.ps1
```

This will:
1. Install Eclipse Embedded CDT IDE
2. Install ARM and RISC-V toolchains
3. Clone IOsonata SDK and dependencies
4. Prompt you to build libraries for your MCU

### For Existing Eclipse Users

If you already have Eclipse installed:

**Clone repositories:**
```bash
./clone_iosonata_sdk_macos.sh  # or linux/win
```

**Build libraries:**
```bash
./build_iosonata_lib_macos.sh  # or linux/win
```

---

## Installation Scripts

### Purpose

Installation scripts set up a **complete development environment** from scratch:

- Download and install Eclipse Embedded CDT
- Install xPack ARM GCC and RISC-V GCC toolchains
- Install OpenOCD debugger
- Clone IOsonata SDK
- Clone all dependencies (Nordic SDKs, TinyUSB, FreeRTOS, etc.)
- Configure Eclipse with proper paths
- Optionally build IOsonata libraries

### Usage

#### macOS

```bash
# Basic installation
./install_iocdevtools_macos.sh

# Custom installation directory
./install_iocdevtools_macos.sh --home /path/to/custom/location

# Force reinstall everything
./install_iocdevtools_macos.sh --force-update

# Uninstall (keeps repositories)
./install_iocdevtools_macos.sh --uninstall

# Show help
./install_iocdevtools_macos.sh --help
```

#### Linux

```bash
# Basic installation
./install_iocdevtools_linux.sh

# Custom installation directory
./install_iocdevtools_linux.sh --home /path/to/custom/location

# Force reinstall
./install_iocdevtools_linux.sh --force-update

# Show help
./install_iocdevtools_linux.sh --help
```

#### Windows

```powershell
# Run PowerShell as Administrator, then:

# Basic installation
.\install_iocdevtools_win.ps1

# Custom installation directory
.\install_iocdevtools_win.ps1 -SdkHome C:\Dev\IOsonata

# Force reinstall
.\install_iocdevtools_win.ps1 -ForceUpdate

# Show help
Get-Help .\install_iocdevtools_win.ps1
```

### What Gets Installed

**Default Installation Paths:**

| Platform | Eclipse | Toolchains | IOsonata SDK |
|----------|---------|------------|--------------|
| macOS | `/Applications/Eclipse.app` | `/opt/xPacks` | `~/IOcomposer` |
| Linux | `/opt/eclipse` | `/opt/xPacks` | `~/IOcomposer` |
| Windows | `C:\Program Files\Eclipse Embedded CDT` | `C:\Program Files\xPacks` | `%USERPROFILE%\IOcomposer` |

**Repository Structure:**
```
~/IOcomposer/                     (or custom --home path)
├── IOsonata/                     # Main framework
│   ├── ARM/                      # ARM MCU implementations
│   ├── RISC-V/                   # RISC-V MCU implementations
│   ├── include/                  # Public headers
│   ├── src/                      # Core source code
│   └── Installer/                    # Build scripts
│       ├── build_iosonata_lib_macos.sh
│       ├── build_iosonata_lib_linux.sh
│       └── build_iosonata_lib_win.ps1
└── external/                     # Dependencies
    ├── nrfx/                     # Nordic HAL
    ├── sdk-nrf-bm/              # Nordic bare-metal SDK
    ├── sdk-nrfxlib/             # Nordic libraries
    ├── nRF5_SDK/                # Nordic SDK v17
    ├── nRF5_SDK_Mesh/           # Nordic Mesh SDK
    ├── BSEC/                    # Bosch sensor library
    ├── Fusion/                  # Sensor fusion library
    ├── vqf/                     # Quaternion filter
    ├── lvgl/                    # Graphics library
    ├── lwip/                    # TCP/IP stack
    ├── FreeRTOS-Kernel/         # RTOS kernel
    └── tinyusb/                 # USB stack
```

### Installation Time

Approximate time to complete installation:

- **Download + Install**: 10-20 minutes (depends on internet speed)
- **First-time library build**: 5-10 minutes per MCU target

---

## Clone Scripts

### Purpose

Clone scripts are for users who **already have Eclipse** and toolchains installed. They only:

- Clone IOsonata SDK
- Clone all dependencies
- Configure Eclipse workspace (optional)
- Optionally build libraries (if Eclipse detected)

### Usage

#### macOS

```bash
# Clone to default location (~/IOcomposer)
./clone_iosonata_sdk_macos.sh

# Clone to custom location
./clone_iosonata_sdk_macos.sh --home /path/to/custom

# Force re-clone all repositories
./clone_iosonata_sdk_macos.sh --mode force

# Also configure Eclipse settings
./clone_iosonata_sdk_macos.sh --eclipse

# Show help
./clone_iosonata_sdk_macos.sh --help
```

#### Linux

```bash
# Clone to default location
./clone_iosonata_sdk_linux.sh

# Clone to custom location
./clone_iosonata_sdk_linux.sh --home /path/to/custom

# Force re-clone
./clone_iosonata_sdk_linux.sh --mode force

# Configure Eclipse
./clone_iosonata_sdk_linux.sh --eclipse
```

#### Windows

```powershell
# Clone to default location
.\clone_iosonata_sdk_win.ps1

# Clone to custom location
.\clone_iosonata_sdk_win.ps1 -Home C:\Dev\IOsonata

# Force re-clone
.\clone_iosonata_sdk_win.ps1 -Mode force

# Configure Eclipse
.\clone_iosonata_sdk_win.ps1 -Eclipse
```

### When to Use Clone Scripts

Use clone scripts when:

- ✅ You already have Eclipse installed
- ✅ You already have ARM/RISC-V toolchains
- ✅ You only need the IOsonata source code
- ✅ You're setting up a new workspace
- ✅ You want to update repositories

### What Gets Cloned

Clone scripts download the same repositories as installation scripts:
- IOsonata framework
- All external dependencies (Nordic SDKs, TinyUSB, etc.)

---

## Build Scripts

### Purpose

Build scripts compile IOsonata libraries for specific MCU targets. These standalone scripts can be run:

- **Multiple times** - Build for different MCUs without re-running installer
- **After source changes** - Rebuild libraries when you modify IOsonata code
- **From anywhere** - As long as Eclipse and IOsonata are installed

### Usage

#### macOS

```bash
# Build with default IOsonata location (~/IOcomposer)
./build_iosonata_lib_macos.sh

# Build with custom IOsonata location
./build_iosonata_lib_macos.sh --home /path/to/iosonata

# Show help
./build_iosonata_lib_macos.sh --help
```

#### Linux

```bash
# Build with default location
./build_iosonata_lib_linux.sh

# Build with custom location
./build_iosonata_lib_linux.sh --home /path/to/iosonata

# Show help
./build_iosonata_lib_linux.sh --help
```

#### Windows

```powershell
# Build with default location
.\build_iosonata_lib_win.ps1

# Build with custom location
.\build_iosonata_lib_win.ps1 -SdkHome C:\Dev\IOsonata

# Show help
Get-Help .\build_iosonata_lib_win.ps1
```

### Interactive Menu

When you run a build script, it presents an interactive menu:

```
=========================================================
  IOsonata Library Builder (macOS)
  Version: v1.0.0
=========================================================

✓ Eclipse found at: /Applications/Eclipse.app
✓ IOsonata SDK found at: /Users/john/IOcomposer/IOsonata

Available IOsonata library projects:

   1) ARM/Nordic/nRF52/nRF52832/lib/Eclipse
   2) ARM/Nordic/nRF52/nRF52840/lib/Eclipse
   3) ARM/Nordic/nRF53/nRF5340/lib/Eclipse
   4) ARM/Nordic/nRF54/nRF54H20/lib/Eclipse
   5) ARM/Nordic/nRF54/nRF54L15/lib/Eclipse
   6) ARM/Nordic/nRF91/lib/Eclipse
   7) ARM/NXP/LPC11xx/lib/Eclipse
   8) ARM/NXP/LPC17xx/lib/Eclipse
   9) ARM/ST/STM32F0xx/lib/Eclipse
  10) ARM/ST/STM32F4xx/lib/Eclipse
  11) ARM/ST/STM32L4xx/lib/Eclipse
  12) ARM/ST/STM32U5xx/lib/Eclipse
  13) RISC-V/GigaDevice/GD32VF103/lib/Eclipse
   0) Exit

Select project to build (0-13): 
```

### What Gets Built

For each MCU target, the build script creates:

```
IOsonata/ARM/Nordic/nRF52/nRF52840/lib/Eclipse/
├── Debug/
│   └── libIOsonata_nRF52840.a      # Debug build (with symbols)
└── Release/
    └── libIOsonata_nRF52840.a      # Release build (optimized)
```

These libraries are then linked into your firmware projects.

### Build Time

- **First build**: 3-5 minutes per MCU
- **Incremental rebuild**: 1-2 minutes

### Build Output

After successful build:

```
✅ IOsonata library build completed for ARM/Nordic/nRF52/nRF52840/lib/Eclipse

Libraries created:
-rw-r--r--  1 john  staff   2.1M Jan  3 10:30 Debug/libIOsonata_nRF52840.a
-rw-r--r--  1 john  staff   1.8M Jan  3 10:30 Release/libIOsonata_nRF52840.a

=========================================================
Build complete! You can now use these libraries in your
firmware projects.
=========================================================
```

---

## Common Workflows

### Workflow 1: Fresh Start (New User)

```bash
# 1. Install everything
./install_iocdevtools_macos.sh

# 2. During installation, select MCU to build
#    (e.g., nRF52840)

# 3. Start developing!
# Open Eclipse and import your firmware project
```

### Workflow 2: Supporting Multiple MCUs

You're developing a product that uses three different MCUs:

```bash
# Build for nRF52840
./build_iosonata_lib_macos.sh
# Select: 2) ARM/Nordic/nRF52/nRF52840

# Build for nRF52832
./build_iosonata_lib_macos.sh
# Select: 1) ARM/Nordic/nRF52/nRF52832

# Build for STM32F4
./build_iosonata_lib_macos.sh
# Select: 10) ARM/ST/STM32F4xx

# All three libraries ready!
```

### Workflow 3: Updating IOsonata

When new IOsonata version is released:

```bash
# 1. Update IOsonata repository
cd ~/IOcomposer/IOsonata
git pull

# 2. Rebuild libraries for your MCU
cd ~/IOcomposer
./build_iosonata_lib_macos.sh
```

### Workflow 4: Contributing to IOsonata

You're making changes to IOsonata framework:

```bash
# 1. Make your changes
cd ~/IOcomposer/IOsonata
vim src/iopincfg.cpp

# 2. Rebuild library to test
cd ~/IOcomposer
./build_iosonata_lib_macos.sh

# 3. Test in your firmware project

# 4. Repeat until satisfied
```

### Workflow 5: Clean Reinstall

Something went wrong, start fresh:

```bash
# 1. Uninstall (keeps repositories and workspaces)
./install_iocdevtools_macos.sh --uninstall

# 2. Reinstall
./install_iocdevtools_macos.sh --force-update
```

### Workflow 6: Setting Up Team Environment

Setting up multiple developers on your team:

```bash
# Each developer runs:
./install_iocdevtools_macos.sh

# Or if team already has standardized tools:
./clone_iosonata_sdk_macos.sh

# Build for your team's MCU target(s)
./build_iosonata_lib_macos.sh
```

---

## Troubleshooting

### Installation Issues

#### "Permission denied" errors

**macOS/Linux:**
```bash
chmod +x install_iocdevtools_macos.sh
./install_iocdevtools_macos.sh
```

**Windows:**
```powershell
# Run PowerShell as Administrator
# Right-click PowerShell → "Run as Administrator"
```

#### Eclipse installation fails

**macOS:**
```bash
# Check if old Eclipse exists
ls -la /Applications/Eclipse.app

# Remove it and try again
sudo rm -rf /Applications/Eclipse.app
./install_iocdevtools_macos.sh --force-update
```

**Linux:**
```bash
# Check permissions
ls -la /opt/eclipse

# Remove and reinstall
sudo rm -rf /opt/eclipse
./install_iocdevtools_linux.sh --force-update
```

#### Out of disk space

Installation requires approximately:
- **Eclipse**: 500 MB
- **Toolchains**: 1.5 GB
- **Repositories**: 2 GB
- **Total**: ~4 GB free space needed

### Build Issues

#### "Eclipse not found"

**macOS:**
```bash
# Check if Eclipse is installed
ls -la /Applications/Eclipse.app

# If not, run installer
./install_iocdevtools_macos.sh
```

**Linux:**
```bash
# Check Eclipse
ls -la /opt/eclipse

# If not installed
./install_iocdevtools_linux.sh
```

#### "IOsonata directory not found"

```bash
# Check IOsonata location
ls -la ~/IOcomposer/IOsonata

# If not found, clone it
./clone_iosonata_sdk_macos.sh
```

#### Build fails with compiler errors

```bash
# Check build log
cat /tmp/build_iosonata_lib.log

# Common issues:
# 1. Missing toolchain - reinstall with --force-update
# 2. Corrupted .cproject - delete and re-clone
# 3. Wrong Eclipse version - reinstall Eclipse
```

#### Build hangs indefinitely

This should not happen with the updated scripts (using `-no-indexer`), but if it does:

```bash
# Kill the build
killall eclipse

# Try again - script uses -no-indexer to prevent hangs
./build_iosonata_lib_macos.sh
```

### Repository Issues

#### Git clone fails

```bash
# Check internet connection
ping github.com

# Check if git is installed
git --version

# If git missing, install it first
# macOS: xcode-select --install
# Linux: sudo apt-get install git
# Windows: Download from git-scm.com
```

#### Repository already exists

```bash
# Normal mode skips existing repositories
./clone_iosonata_sdk_macos.sh

# To force re-clone
./clone_iosonata_sdk_macos.sh --mode force
```

---

## Platform-Specific Notes

### macOS

**Requirements:**
- macOS 10.15+ (Catalina or later)
- Xcode Command Line Tools: `xcode-select --install`
- 4 GB free disk space

**Default Paths:**
- Eclipse: `/Applications/Eclipse.app`
- Toolchains: `/opt/xPacks`
- IOsonata: `~/IOcomposer`

**Notes:**
- Installation requires `sudo` for Eclipse and toolchains
- First Eclipse launch may show security warning - allow in System Preferences
- Build scripts use `eclipsec` (console launcher) to avoid GUI dialogs

### Linux

**Requirements:**
- Ubuntu 20.04+ or equivalent distribution
- `sudo` access for system installations
- 4 GB free disk space

**Default Paths:**
- Eclipse: `/opt/eclipse`
- Toolchains: `/opt/xPacks`
- IOsonata: `~/IOcomposer`

**Package Dependencies:**
The installer will check and prompt to install:
- `curl`, `tar`, `wget`
- `git`
- Java Runtime (for Eclipse)

**Notes:**
- Installation requires `sudo` for system paths
- Build scripts use `eclipsec` (console launcher)
- Eclipse may require GTK3 libraries on some distributions

### Windows

**Requirements:**
- Windows 10/11
- PowerShell 5.1+ (built into Windows)
- Administrator privileges
- 4 GB free disk space

**Default Paths:**
- Eclipse: `C:\Program Files\Eclipse Embedded CDT`
- Toolchains: `C:\Program Files\xPacks`
- IOsonata: `%USERPROFILE%\IOcomposer`

**Notes:**
- **Must run PowerShell as Administrator** for installation
- Installation requires 7-Zip (auto-installed if missing)
- Windows Defender may scan downloaded files (slows installation)
- Consider adding exclusions for development directories

**PowerShell Execution Policy:**
```powershell
# If you get execution policy errors
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process

# Then run the script
.\install_iocdevtools_win.ps1
```

---

## Advanced Usage

### Custom Installation Paths

All scripts support custom paths:

```bash
# Install to custom location
./install_iocdevtools_macos.sh --home /Volumes/External/Development

# Clone to custom location
./clone_iosonata_sdk_macos.sh --home /Volumes/External/Development

# Build from custom location
./build_iosonata_lib_macos.sh --home /Volumes/External/Development
```

### Automated/Scripted Installation

For CI/CD or automated setups:

```bash
# Installation scripts can run non-interactively
# They will skip library build prompt if no TTY

# Clone scripts are fully automated
./clone_iosonata_sdk_macos.sh --home /path/to/ci/workspace

# Build scripts require interactive selection
# For automation, consider calling Eclipse directly
```

### Using Different Eclipse

If you have your own Eclipse installation:

```bash
# Clone repositories only
./clone_iosonata_sdk_macos.sh

# Point to your Eclipse when building
# Modify BUILD_SCRIPT path in the script, or
# Use the standalone build script manually
```

---

## Getting Help

### Script Help

All scripts have built-in help:

```bash
# macOS/Linux
./install_iocdevtools_macos.sh --help
./clone_iosonata_sdk_macos.sh --help
./build_iosonata_lib_macos.sh --help

# Windows
Get-Help .\install_iocdevtools_win.ps1
Get-Help .\clone_iosonata_sdk_win.ps1
Get-Help .\build_iosonata_lib_win.ps1
```

### Log Files

Build scripts create detailed logs:

```bash
# Check build log
cat /tmp/build_iosonata_lib.log

# View recent errors
tail -50 /tmp/build_iosonata_lib.log
```

### Community Support

- **GitHub Issues**: Report bugs or request features
- **IOsonata Website**: [https://iosonata.com](https://iosonata.com)
- **Documentation**: [https://github.com/IOsonata/IOsonata](https://github.com/IOsonata/IOsonata)

---

## FAQ

### Q: Which script should I use?

**A:** For first-time setup, use the **installation script** for your platform. It does everything.

### Q: Can I use my existing Eclipse?

**A:** Yes! Use the **clone script** to get the code, then use the **build script** to compile libraries.

### Q: How do I build for multiple MCUs?

**A:** Run the **build script** multiple times, selecting a different MCU each time.

### Q: Do I need to rebuild libraries when I update IOsonata?

**A:** Yes, after pulling new IOsonata code, run the build script to recompile libraries.

### Q: Can I build libraries without Eclipse?

**A:** No, the build scripts use Eclipse CDT's headless build system. Eclipse is required.

### Q: What if I don't want to build libraries?

**A:** During installation, select **0) Skip library build** when prompted.

### Q: How do I uninstall?

**A:** Run the installer with `--uninstall` flag. This removes tools but keeps your repositories and workspaces.

### Q: Can I have multiple IOsonata installations?

**A:** Yes! Use the `--home` parameter to specify different directories.

---

## License

IOsonata is released under the Apache License 2.0. See LICENSE file in the repository for details.

---

## Version History

- **v1.0.0** - Initial release with refactored standalone build scripts
- Installer scripts: macOS v1.0.87, Linux v1.0.87, Windows v1.0.87-win
- Clone scripts: All platforms v1.3.0
- Build scripts: All platforms v1.0.0

---

**Happy Building!** 🚀

For the latest updates and documentation, visit: [https://github.com/IOsonata/IOsonata](https://github.com/IOsonata/IOsonata)
