# Run IOsonata Scripts Directly from GitHub

You can run IOsonata installation and clone scripts directly from GitHub without cloning the repository first. This is the fastest way to get started!

## ⚠️ Security Warning

**Important**: Only run scripts from trusted sources! Review the script content before executing.

```bash
# ALWAYS review the script first:
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | less

# Then run it if you trust it
```

## Quick Installation (One-Liner)

### macOS

**Method 1: Pipe to bash** (fastest)
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash
```

**Method 2: Download then run** (recommended - allows review)
```bash
# Download
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh

# Review (optional but recommended)
less install_iocdevtools_macos.sh

# Make executable and run
chmod +x install_iocdevtools_macos.sh
./install_iocdevtools_macos.sh
```

**With custom installation path:**
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash -s -- --home /custom/path
```

### Linux

**Method 1: Pipe to bash**
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash
```

**Method 2: Download then run** (recommended)
```bash
# Download
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh

# Review (optional but recommended)
less install_iocdevtools_linux.sh

# Make executable and run
chmod +x install_iocdevtools_linux.sh
./install_iocdevtools_linux.sh
```

**Using wget instead of curl:**
```bash
wget https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh
chmod +x install_iocdevtools_linux.sh
./install_iocdevtools_linux.sh
```

**With custom installation path:**
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash -s -- --home /custom/path
```

### Windows

**Method 1: Direct execution** (PowerShell as Administrator)
```powershell
# Download and execute
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 | iex
```

**Method 2: Download then run** (recommended)
```powershell
# Download
Invoke-WebRequest -Uri https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 -OutFile install_iocdevtools_win.ps1

# Review (optional but recommended)
notepad install_iocdevtools_win.ps1

# Run
.\install_iocdevtools_win.ps1
```

**With custom installation path:**
```powershell
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 | iex -SdkHome C:\Custom\Path
```

## Clone Scripts (Just Get the Code)

If you already have Eclipse and toolchains installed, you can run clone scripts directly:

### macOS
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/clone_iosonata_sdk_macos.sh | bash
```

### Linux
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/clone_iosonata_sdk_linux.sh | bash
```

### Windows (PowerShell as Administrator)
```powershell
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/clone_iosonata_sdk_win.ps1 | iex
```

## Build Scripts (Compile Libraries)

You can also download build scripts directly (requires Eclipse and IOsonata already installed):

### macOS
```bash
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/build_iosonata_lib_macos.sh
chmod +x build_iosonata_lib_macos.sh
./build_iosonata_lib_macos.sh
```

### Linux
```bash
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/build_iosonata_lib_linux.sh
chmod +x build_iosonata_lib_linux.sh
./build_iosonata_lib_linux.sh
```

### Windows
```powershell
Invoke-WebRequest -Uri https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/build_iosonata_lib_win.ps1 -OutFile build_iosonata_lib_win.ps1
.\build_iosonata_lib_win.ps1
```

## Advanced: Passing Arguments

You can pass arguments to scripts when piping:

### macOS/Linux
```bash
# With custom path
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash -s -- --home /custom/path

# Force update
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash -s -- --force-update

# Multiple arguments
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash -s -- --home /custom/path --force-update
```

### Windows
```powershell
# With custom path
$script = iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1
Invoke-Expression "& { $script } -SdkHome C:\Custom\Path"

# Force update
$script = iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1
Invoke-Expression "& { $script } -ForceUpdate"
```

## URL Structure

All scripts are available at:
```
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/<script-name>
```

Available scripts:
- `install_iocdevtools_macos.sh`
- `install_iocdevtools_linux.sh`
- `install_iocdevtools_win.ps1`
- `clone_iosonata_sdk_macos.sh`
- `clone_iosonata_sdk_linux.sh`
- `clone_iosonata_sdk_win.ps1`
- `build_iosonata_lib_macos.sh`
- `build_iosonata_lib_linux.sh`
- `build_iosonata_lib_win.ps1`

## Benefits of Direct Execution

✅ **Fastest setup** - No need to clone repo first  
✅ **Always latest** - Gets current version from main branch  
✅ **Convenient** - One command to start  
✅ **Perfect for CI/CD** - Easy automation  

## Best Practices

### 1. Review Before Running

**Always review scripts from the internet:**
```bash
# View script in browser first
open https://github.com/IOsonata/IOsonata/blob/main/Installer/install_iocdevtools_macos.sh

# Or download and review locally
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh
cat install_iocdevtools_macos.sh | less
```

### 2. Use HTTPS

Always use HTTPS URLs (not HTTP) to prevent man-in-the-middle attacks:
```bash
# ✅ Good - HTTPS
https://raw.githubusercontent.com/...

# ❌ Bad - HTTP (insecure)
http://raw.githubusercontent.com/...
```

### 3. Download Then Run (Recommended)

For production or important systems, download first:
```bash
# 1. Download
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh

# 2. Review
cat install_iocdevtools_macos.sh

# 3. Verify integrity (if checksums available)
# sha256sum install_iocdevtools_macos.sh

# 4. Run
chmod +x install_iocdevtools_macos.sh
./install_iocdevtools_macos.sh
```

### 4. Pin to Specific Version (Optional)

For reproducibility, use a specific commit or tag:
```bash
# Use specific release tag
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/v1.0.87/Installer/install_iocdevtools_macos.sh | bash

# Use specific commit
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/abc123def/Installer/install_iocdevtools_macos.sh | bash
```

## Troubleshooting

### curl: command not found (macOS)

Install Xcode Command Line Tools:
```bash
xcode-select --install
```

### curl: command not found (Linux)

Install curl:
```bash
# Ubuntu/Debian
sudo apt-get install curl

# Fedora/RHEL
sudo dnf install curl

# Or use wget instead
wget -O - https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash
```

### PowerShell Execution Policy Error (Windows)

Run PowerShell as Administrator and set execution policy:
```powershell
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
```

### SSL/TLS Certificate Errors

If you get certificate errors, your system certificates might be outdated:
```bash
# macOS - update certificates
sudo softwareupdate --install --all

# Linux - update ca-certificates
sudo apt-get update && sudo apt-get install ca-certificates

# Or bypass (not recommended for production)
curl -k -fsSL https://... | bash
```

### Script Downloaded But Won't Run

Make sure it's executable:
```bash
chmod +x install_iocdevtools_macos.sh
./install_iocdevtools_macos.sh
```

## Examples for Different Use Cases

### Quick Demo Setup

Test IOsonata quickly:
```bash
# macOS
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash -s -- --home /tmp/iosonata-demo
```

### CI/CD Pipeline

In GitHub Actions or similar:
```yaml
- name: Install IOsonata
  run: |
    curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash -s -- --home $GITHUB_WORKSPACE/iosonata
```

### Team Onboarding

Send new team members one command:
```bash
# macOS team
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash

# Linux team
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash

# Windows team (PowerShell as Admin)
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 | iex
```

### Production Setup

For production systems, use the download-then-run method:
```bash
# Download specific version
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/v1.0.87/Installer/install_iocdevtools_linux.sh

# Review thoroughly
cat install_iocdevtools_linux.sh

# Run after verification
chmod +x install_iocdevtools_linux.sh
./install_iocdevtools_linux.sh
```

## Comparison: Direct vs Local

| Method | Speed | Security | Offline | Review |
|--------|-------|----------|---------|--------|
| **Direct** | ⚡ Fastest | ⚠️ Trust needed | ❌ No | ⚠️ Optional |
| **Download + Run** | 🐢 Slower | ✅ Can review | ✅ Yes | ✅ Easy |
| **Clone Repo** | 🐌 Slowest | ✅ Full control | ✅ Yes | ✅ Complete |

## When to Use Direct Execution

**Good for:**
- ✅ Quick testing
- ✅ Personal development machines
- ✅ Trusted environments
- ✅ CI/CD pipelines
- ✅ Documentation examples

**Not recommended for:**
- ❌ Production servers
- ❌ Air-gapped systems
- ❌ High-security environments
- ❌ When you need to review changes

## Alternative: Using Git Sparse Checkout

If you only want the scripts (not full IOsonata):
```bash
# Initialize sparse checkout
git clone --filter=blob:none --sparse https://github.com/IOsonata/IOsonata.git
cd IOsonata
git sparse-checkout add tools

# Now you have just the Installer/ directory
cd Installer
./install_iocdevtools_macos.sh
```

## Summary

**Fastest setup:**
```bash
# macOS
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash

# Linux
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash

# Windows (PowerShell as Admin)
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 | iex
```

**Recommended (safer):**
```bash
# Download, review, run
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh
cat install_iocdevtools_macos.sh  # Review!
chmod +x install_iocdevtools_macos.sh
./install_iocdevtools_macos.sh
```

---

**Ready to install? Pick your platform and run the command!** 🚀
