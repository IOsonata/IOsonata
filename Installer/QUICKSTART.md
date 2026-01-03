# IOsonata Quick Start Guide

Get up and running with IOsonata in minutes!


## 🌐 Fastest Way: Run Directly from GitHub

Don't want to clone the repository? Run the installer directly from GitHub!

### macOS
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash
```

### Linux
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash
```

### Windows (PowerShell as Administrator)
```powershell
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 | iex
```

> **💡 Pro Tip**: For better security, download the script first, review it, then run it. See [ONLINE_INSTALLATION.md](ONLINE_INSTALLATION.md) for details.

---

## 📦 Alternative: Download Scripts First

If you prefer to review before running (recommended):

## 📦 Downloaded Scripts Setup

Choose your platform and run:

### macOS
```bash
./install_iocdevtools_macos.sh
```

### Linux
```bash
./install_iocdevtools_linux.sh
```

### Windows (as Administrator)
```powershell
.\install_iocdevtools_win.ps1
```

**That's it!** This installs Eclipse, toolchains, clones repositories, and prompts you to build libraries.

---

## 🎯 What Do These Scripts Do?

### Installation Scripts → Complete Setup
- ✅ Install Eclipse IDE
- ✅ Install ARM & RISC-V toolchains
- ✅ Clone IOsonata & dependencies
- ✅ Build libraries for your MCU

**Use when**: Setting up for the first time

### Clone Scripts → Code Only
- ✅ Clone IOsonata & dependencies
- ✅ Configure Eclipse (optional)

**Use when**: You already have Eclipse and toolchains

### Build Scripts → Compile Libraries
- ✅ Build IOsonata library for any MCU
- ✅ Can run multiple times for different MCUs

**Use when**: You need to build or rebuild libraries

---

## 🚀 Common Tasks

### Build for Multiple MCUs

Supporting nRF52840, nRF52832, and STM32F4?

```bash
./build_iosonata_lib_macos.sh  # Select nRF52840
./build_iosonata_lib_macos.sh  # Select nRF52832
./build_iosonata_lib_macos.sh  # Select STM32F4
```

Done! Three libraries ready to use.

### Update IOsonata

```bash
cd ~/IOcomposer/IOsonata
git pull
cd ..
./build_iosonata_lib_macos.sh  # Rebuild your libraries
```

### Rebuild After Changes

Made changes to IOsonata source?

```bash
./build_iosonata_lib_macos.sh  # Rebuild
```

---

## 📁 Where Everything Lives

### macOS
- Eclipse: `/Applications/Eclipse.app`
- IOsonata: `~/IOcomposer/IOsonata`
- Libraries: `~/IOcomposer/IOsonata/ARM/.../lib/Eclipse/Debug|Release/`

### Linux
- Eclipse: `/opt/eclipse`
- IOsonata: `~/IOcomposer/IOsonata`
- Libraries: `~/IOcomposer/IOsonata/ARM/.../lib/Eclipse/Debug|Release/`

### Windows
- Eclipse: `C:\Program Files\Eclipse Embedded CDT`
- IOsonata: `%USERPROFILE%\IOcomposer\IOsonata`
- Libraries: `%USERPROFILE%\IOcomposer\IOsonata\ARM\...\lib\Eclipse\Debug|Release\`

---

## ⚠️ Troubleshooting

### Permission Denied (macOS/Linux)
```bash
chmod +x *.sh
```

### Windows Execution Policy Error
```powershell
# Run PowerShell as Administrator
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
```

### Eclipse Not Found
```bash
# Just run the installer - it will install Eclipse
./install_iocdevtools_macos.sh
```

### Build Fails
```bash
# Check the log
cat /tmp/build_iosonata_lib.log
```

---

## 📖 Need More Help?

See the full [README.md](README.md) for:
- Detailed usage instructions
- All command-line options
- Platform-specific notes
- Advanced workflows
- Complete troubleshooting guide

---

## ⏱️ Time Estimates

- **Installation**: 10-20 minutes
- **Clone only**: 2-5 minutes  
- **Build library**: 3-5 minutes per MCU

---

**Ready to develop!** 🎉

For documentation and examples, see: [https://github.com/IOsonata/IOsonata](https://github.com/IOsonata/IOsonata)
