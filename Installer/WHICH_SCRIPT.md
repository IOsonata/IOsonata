# Which Script Should I Use?

Quick decision guide to help you choose the right IOsonata script for your situation.

## Decision Tree

```
Do you have Eclipse installed?
│
├─ NO  → Use INSTALLATION SCRIPT
│         (installs everything + clones + builds)
│
└─ YES → Do you have IOsonata cloned?
          │
          ├─ NO  → Use CLONE SCRIPT
          │         (clones repos + optionally builds)
          │
          └─ YES → Do you need to build/rebuild libraries?
                    │
                    ├─ YES → Use BUILD SCRIPT
                    │         (builds for any MCU)
                    │
                    └─ NO  → You're all set! 🎉
```


---

## 🌐 Online Installation Option

**Don't have the scripts yet?** You can run them directly from GitHub!

```bash
# macOS
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash

# Linux  
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash

# Windows (PowerShell as Admin)
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 | iex
```

See [ONLINE_INSTALLATION.md](ONLINE_INSTALLATION.md) for security best practices and detailed instructions.

---

## Script Comparison

| Feature | Installation | Clone | Build |
|---------|-------------|-------|-------|
| **Installs Eclipse** | ✅ Yes | ❌ No | ❌ No |
| **Installs Toolchains** | ✅ Yes | ❌ No | ❌ No |
| **Clones IOsonata** | ✅ Yes | ✅ Yes | ❌ No |
| **Clones Dependencies** | ✅ Yes | ✅ Yes | ❌ No |
| **Builds Libraries** | ✅ Optional | ✅ Optional | ✅ Yes |
| **Can Run Multiple Times** | ⚠️ Yes* | ⚠️ Yes* | ✅ Yes |
| **Time Required** | 15-20 min | 3-5 min | 3-5 min |

*With `--force-update` flag

---

## Use Case Guide

### Scenario 1: Brand New Setup

**Situation**: You have nothing installed, want complete setup.

**Use**: Installation Script

**Command**:
```bash
./install_iocdevtools_macos.sh
```

**What Happens**:
1. ✅ Installs Eclipse
2. ✅ Installs ARM & RISC-V toolchains
3. ✅ Clones IOsonata & dependencies
4. ✅ Prompts to build library

**Time**: ~15-20 minutes

---

### Scenario 2: Have Eclipse, Need Code

**Situation**: You already have Eclipse and toolchains installed, just need IOsonata source.

**Use**: Clone Script

**Command**:
```bash
./clone_iosonata_sdk_macos.sh
```

**What Happens**:
1. ✅ Clones IOsonata & dependencies
2. ✅ Detects Eclipse
3. ✅ Offers to build library

**Time**: ~3-5 minutes

---

### Scenario 3: Building for Multiple MCUs

**Situation**: You need to build libraries for nRF52840, nRF52832, and STM32F4.

**Use**: Build Script (3 times)

**Commands**:
```bash
./build_iosonata_lib_macos.sh  # Select nRF52840
./build_iosonata_lib_macos.sh  # Select nRF52832
./build_iosonata_lib_macos.sh  # Select STM32F4
```

**What Happens**:
1. ✅ First run: Builds nRF52840 library
2. ✅ Second run: Builds nRF52832 library
3. ✅ Third run: Builds STM32F4 library

**Time**: ~3-5 minutes per MCU

---

### Scenario 4: Modified IOsonata Source

**Situation**: You made changes to IOsonata and need to rebuild.

**Use**: Build Script

**Command**:
```bash
vim ~/IOcomposer/IOsonata/src/iopincfg.cpp  # Make changes
./build_iosonata_lib_macos.sh               # Rebuild
```

**What Happens**:
1. ✅ Rebuilds library with your changes
2. ✅ Creates new Debug and Release libraries

**Time**: ~3-5 minutes

---

### Scenario 5: Updating IOsonata

**Situation**: New IOsonata version released, you want to update.

**Use**: Git Pull + Build Script

**Commands**:
```bash
cd ~/IOcomposer/IOsonata
git pull
cd ..
./build_iosonata_lib_macos.sh
```

**What Happens**:
1. ✅ Updates IOsonata source
2. ✅ Rebuilds library with new code

**Time**: ~3-5 minutes

---

### Scenario 6: Something Broke, Need Clean Slate

**Situation**: Environment is messed up, want to start fresh.

**Use**: Installation Script with Force Update

**Commands**:
```bash
./install_iocdevtools_macos.sh --uninstall  # Clean up
./install_iocdevtools_macos.sh --force-update  # Fresh install
```

**What Happens**:
1. ✅ Removes old Eclipse and toolchains
2. ✅ Keeps your repositories and workspaces
3. ✅ Reinstalls everything fresh

**Time**: ~15-20 minutes

---

### Scenario 7: Team Standardization

**Situation**: Setting up 5 developers with same environment.

**Use**: Installation Script (each developer)

**Commands**:
```bash
# Each developer runs:
./install_iocdevtools_macos.sh

# Everyone builds same MCU target
./build_iosonata_lib_macos.sh  # Select team's MCU
```

**What Happens**:
1. ✅ Consistent environment across team
2. ✅ Everyone has same tools, versions
3. ✅ Same libraries built

**Time**: ~15-20 minutes per developer (one-time)

---

### Scenario 8: CI/CD Pipeline

**Situation**: Automating builds in continuous integration.

**Use**: Clone Script + Build Script

**Commands**:
```bash
# In CI pipeline:
./clone_iosonata_sdk_linux.sh --mode force
# (Build script would need non-interactive mode - future feature)
```

**What Happens**:
1. ✅ Fresh clone every time
2. ✅ Deterministic builds
3. ⚠️  Build requires manual MCU selection (for now)

**Note**: Non-interactive build mode planned for future.

---

## Quick Reference Table

| I want to... | Use this script | Platform | Time |
|--------------|----------------|----------|------|
| **Set up everything from scratch** | `install_iocdevtools_*` | All | 15-20 min |
| **Just get the code** | `clone_iosonata_sdk_*` | All | 3-5 min |
| **Build one MCU library** | `build_iosonata_lib_*` | All | 3-5 min |
| **Build multiple MCUs** | `build_iosonata_lib_*` (×N) | All | 3-5 min each |
| **Rebuild after changes** | `build_iosonata_lib_*` | All | 3-5 min |
| **Update IOsonata** | `git pull` + `build_*` | All | 3-5 min |
| **Clean reinstall** | `install_*` --force-update | All | 15-20 min |
| **Update repos only** | `clone_*` --mode force | All | 3-5 min |

---

## Platform-Specific Scripts

### macOS
- `install_iocdevtools_macos.sh`
- `clone_iosonata_sdk_macos.sh`
- `build_iosonata_lib_macos.sh`

### Linux
- `install_iocdevtools_linux.sh`
- `clone_iosonata_sdk_linux.sh`
- `build_iosonata_lib_linux.sh`

### Windows
- `install_iocdevtools_win.ps1`
- `clone_iosonata_sdk_win.ps1`
- `build_iosonata_lib_win.ps1`

---

## Common Flags

### Installation Scripts

```bash
--help              # Show help
--version           # Show version
--home <path>       # Custom installation path
--force-update      # Reinstall everything
--uninstall         # Remove tools (keep repos)
```

### Clone Scripts

```bash
--help              # Show help
--home <path>       # Custom clone path
--mode force        # Re-clone all repos
--eclipse           # Configure Eclipse settings
```

### Build Scripts

```bash
--help              # Show help
--home <path>       # Custom IOsonata path
```

---

## When NOT to Use Scripts

### Don't Use Installation Script If:
- ❌ You already have Eclipse and toolchains
- ❌ You want custom Eclipse version
- ❌ You use different toolchains (e.g., Keil, IAR)

**Use Clone Script instead**

### Don't Use Clone Script If:
- ❌ You need Eclipse and toolchains installed
- ❌ You only want to rebuild libraries

**Use Installation Script or Build Script instead**

### Don't Use Build Script If:
- ❌ You don't have Eclipse installed
- ❌ You don't have IOsonata cloned
- ❌ You just need to get the code

**Use Installation Script or Clone Script first**

---

## Recommended Workflows

### For Beginners
```bash
# Step 1: Run installer (does everything)
./install_iocdevtools_macos.sh

# Step 2: Start developing!
```

### For Experienced Users
```bash
# Step 1: Clone if needed
./clone_iosonata_sdk_macos.sh

# Step 2: Build for your MCU(s)
./build_iosonata_lib_macos.sh
```

### For Power Users
```bash
# Install once
./install_iocdevtools_macos.sh

# Update periodically
cd ~/IOcomposer/IOsonata && git pull && cd ..

# Rebuild as needed
./build_iosonata_lib_macos.sh  # Run N times for N MCUs
```

---

## Still Confused?

### Simple Rule

**First time?** → Use Installation Script

**Have Eclipse?** → Use Clone Script

**Just building?** → Use Build Script

---

## Need More Help?

- Read [README.md](README.md) for detailed usage
- Read [QUICKSTART.md](QUICKSTART.md) for quick start
- Check [ARCHITECTURE.md](ARCHITECTURE.md) for technical details
- See [CHANGELOG.md](CHANGELOG.md) for version history

---

**Pro Tip**: You can run build script as many times as you want for different MCUs! 🚀
