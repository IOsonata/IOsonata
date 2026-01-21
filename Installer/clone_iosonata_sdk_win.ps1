# clone_iosonata_sdk_win.ps1
# =========================================================
#  clone_iosonata_sdk_win
# ---------------------------------------------------------
#  Purpose: Clone IOsonata SDK and dependencies
#  Platform: Windows (PowerShell)
#  Version: v1.1.3
# =========================================================

param(
    [string]$SdkHome = "$env:USERPROFILE\IOcomposer",
    [string]$Mode = "normal",
    [switch]$Eclipse,
    [switch]$Help
)

$ErrorActionPreference = 'Stop'
$SCRIPT_VERSION = "v1.1.3"

# --- Banner ---
function Show-Banner {
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host "     IOsonata SDK Cloning Utility for Windows" -ForegroundColor White
    Write-Host "     Version: $SCRIPT_VERSION" -ForegroundColor White
    Write-Host "=========================================================" -ForegroundColor Blue
}

# --- Help Menu ---
function Show-Help {
    Show-Banner
    Write-Host "Usage: .\clone_iosonata_sdk_win.ps1 [options]" -ForegroundColor White
    Write-Host ""
    Write-Host "Options:" -ForegroundColor White
    Write-Host "  -SdkHome <path>     Set custom root directory (default: ~\IOcomposer)"
    Write-Host "  -Mode <mode>        Clone mode: 'normal' (default) or 'force'"
    Write-Host "  -Eclipse            Configure Eclipse system properties"
    Write-Host "  -Help               Show this help message and exit"
    Write-Host ""
    Write-Host "Examples:" -ForegroundColor White
    Write-Host "  .\clone_iosonata_sdk_win.ps1                     # Clone into ~\IOcomposer"
    Write-Host "  .\clone_iosonata_sdk_win.ps1 -SdkHome C:\Dev    # Custom directory"
    Write-Host "  .\clone_iosonata_sdk_win.ps1 -Mode force         # Force re-clone all repos"
    Write-Host "  .\clone_iosonata_sdk_win.ps1 -Eclipse            # Also configure Eclipse"
    Write-Host ""
    exit 0
}

if ($Help) { Show-Help }

# --- Prepare directories ---
$ROOT = $SdkHome
if (-not (Test-Path $ROOT)) {
    New-Item -Path $ROOT -ItemType Directory -Force | Out-Null
}
$ROOT = (Resolve-Path $ROOT).Path
$EXT = Join-Path $ROOT "external"
if (-not (Test-Path $EXT)) {
    New-Item -Path $EXT -ItemType Directory -Force | Out-Null
}

Show-Banner
Write-Host "Root directory:       $ROOT" -ForegroundColor White
Write-Host "Mode:                 $Mode" -ForegroundColor White
Write-Host "External path:        $EXT" -ForegroundColor White

# --- Detect Eclipse Installation ---
$ECLIPSE_DIR = "$env:ProgramFiles\Eclipse Embedded CDT"
$ECLIPSE_INSTALLED = $false
if (Test-Path "$ECLIPSE_DIR\eclipse.exe") {
    $ECLIPSE_INSTALLED = $true
    Write-Host "   `[OK`] Eclipse detected at $ECLIPSE_DIR" -ForegroundColor Green
    Write-Host "=========================================================`n" -ForegroundColor Blue
}
Write-Host "Configure Eclipse:    $Eclipse" -ForegroundColor White
Write-Host "---------------------------------------------------------" -ForegroundColor Blue
Write-Host ""

# --- Function: Clone or update repo ---
function Sync-Repository {
    param(
        [string]$Url,
        [string]$Target,
        [bool]$RecurseSubmodules = $false
    )
    
    if (Test-Path $Target) {
        if ($Mode -eq "force") {
            Write-Host "   `[WARN`] Re-cloning $Target ..." -ForegroundColor Yellow
            Remove-Item -Path $Target -Recurse -Force
            if ($RecurseSubmodules) {
                git clone --depth=1 --recurse-submodules $Url $Target
            } else {
                git clone --depth=1 $Url $Target
            }
        } else {
            Write-Host "   `[INFO`] Updating $Target ..." -ForegroundColor Yellow
            Push-Location $Target
            try {
                git pull --rebase
                if ($RecurseSubmodules) {
                    Write-Host "      -> Updating submodules..." -ForegroundColor Blue
                    git submodule update --init --recursive
                }
            } finally {
                Pop-Location
            }
        }
    } else {
        Write-Host "   `[INFO`] Cloning $Target ..." -ForegroundColor Blue
        if ($RecurseSubmodules) {
            git clone --depth=1 --recurse-submodules $Url $Target
            Write-Host "      `[OK`] Submodules initialized" -ForegroundColor Green
        } else {
            git clone --depth=1 $Url $Target
        }
    }
}

# --- IOsonata Library Build Function ---

# ---------------------------------------------------------
# Clone IOsonata
# ---------------------------------------------------------
Write-Host ">>> Cloning IOsonata..." -ForegroundColor Green
Sync-Repository -Url "https://github.com/IOsonata/IOsonata.git" -Target "$ROOT\IOsonata"

# ---------------------------------------------------------
# Clone External Repos
# ---------------------------------------------------------
Write-Host ""
Write-Host ">>> Cloning dependencies into: $EXT" -ForegroundColor Blue
Write-Host ""

$repos = @{
    "https://github.com/NordicSemiconductor/nrfx.git" = "nrfx"
    "https://github.com/nrfconnect/sdk-nrf-bm.git" = "sdk-nrf-bm"
    "https://github.com/nrfconnect/sdk-nrfxlib.git" = "sdk-nrfxlib"
    "https://github.com/IOsonata/nRF5_SDK.git" = "nRF5_SDK"
    "https://github.com/IOsonata/nRF5_SDK_Mesh.git" = "nRF5_SDK_Mesh"
    "https://github.com/boschsensortec/Bosch-BSEC2-Library.git" = "BSEC"
    "https://github.com/xioTechnologies/Fusion.git" = "Fusion"
    "https://github.com/lvgl/lvgl.git" = "lvgl"
    "https://github.com/lwip-tcpip/lwip.git" = "lwip"
    "https://github.com/hathach/tinyusb.git" = "tinyusb"
}

foreach ($repo in $repos.GetEnumerator()) {
    $target = Join-Path $EXT $repo.Value
    Sync-Repository -Url $repo.Key -Target $target -RecurseSubmodules $false
}

# ---------------------------------------------------------
# Clone FreeRTOS-Kernel
# ---------------------------------------------------------
Write-Host ""
Write-Host ">>> Cloning FreeRTOS-Kernel..." -ForegroundColor Blue
Sync-Repository -Url "https://github.com/FreeRTOS/FreeRTOS-Kernel.git" -Target "$EXT\FreeRTOS-Kernel" -RecurseSubmodules $false

Write-Host ""
Write-Host "   `[OK`] All repositories cloned successfully!" -ForegroundColor Green

# ---------------------------------------------------------
# Generate makefile_path.mk
# ---------------------------------------------------------
Write-Host ""
Write-Host ">>> Generating makefile_path.mk for Makefile-based builds..." -ForegroundColor Cyan

$MAKEFILE_PATH_MK = Join-Path $ROOT "IOsonata\makefile_path.mk"

# Convert Windows paths to Unix-style for Makefiles
$ROOT_UNIX = $ROOT -replace '\\', '/'
$EXT_UNIX = $EXT -replace '\\', '/'

$makefileContent = @"
# makefile_path.mk
# Auto-generated by clone_iosonata_sdk_win.ps1
# This file contains path macros for IOsonata projects
# Include this file in your project Makefile: include `$(IOSONATA_ROOT)/makefile_path.mk

# ============================================
# IOsonata Paths
# ============================================
IOSONATA_ROOT = `$(IOCOMPOSER_HOME)/IOsonata
IOSONATA_INCLUDE = `$(IOSONATA_ROOT)/include
IOSONATA_SRC = `$(IOSONATA_ROOT)/src

# ============================================
# ARM-specific Paths
# ============================================
ARM_ROOT = `$(IOSONATA_ROOT)/ARM
ARM_CMSIS = `$(ARM_ROOT)/CMSIS
ARM_CMSIS_INCLUDE = `$(ARM_CMSIS)/Include
ARM_INCLUDE = `$(ARM_ROOT)/include
ARM_SRC = `$(ARM_ROOT)/src
ARM_LDSCRIPT = `$(ARM_ROOT)/ldscript

# Vendor-specific paths
ARM_NORDIC = `$(ARM_ROOT)/Nordic
ARM_NXP = `$(ARM_ROOT)/NXP
ARM_ST = `$(ARM_ROOT)/ST
ARM_MICROCHIP = `$(ARM_ROOT)/Microchip
ARM_RENESAS = `$(ARM_ROOT)/Renesas

# ============================================
# RISC-V-specific Paths
# ============================================
RISCV_ROOT = `$(IOSONATA_ROOT)/RISCV
RISCV_INCLUDE = `$(RISCV_ROOT)/include
RISCV_SRC = `$(RISCV_ROOT)/src
RISCV_LDSCRIPT = `$(RISCV_ROOT)/ldscript

# Vendor-specific paths
RISCV_ESPRESSIF = `$(RISCV_ROOT)/Espressif
RISCV_NORDIC = `$(RISCV_ROOT)/Nordic
RISCV_RENESAS = `$(RISCV_ROOT)/Renesas

# ============================================
# External Libraries
# ============================================
EXTERNAL_ROOT = `$(IOCOMPOSER_HOME)/external
NRFX_ROOT = `$(EXTERNAL_ROOT)/nrfx
SDK_NRF_BM_ROOT = `$(EXTERNAL_ROOT)/sdk-nrf-bm
SDK_NRFXLIB_ROOT = `$(EXTERNAL_ROOT)/sdk-nrfxlib
NRF5_SDK_ROOT = `$(EXTERNAL_ROOT)/nRF5_SDK
NRF5_SDK_MESH_ROOT = `$(EXTERNAL_ROOT)/nRF5_SDK_Mesh
BSEC_ROOT = `$(EXTERNAL_ROOT)/BSEC
FUSION_ROOT = `$(EXTERNAL_ROOT)/Fusion
LVGL_ROOT = `$(EXTERNAL_ROOT)/lvgl
LWIP_ROOT = `$(EXTERNAL_ROOT)/lwip
FREERTOS_KERNEL_ROOT = `$(EXTERNAL_ROOT)/FreeRTOS-Kernel
TINYUSB_ROOT = `$(EXTERNAL_ROOT)/tinyusb

# ============================================
# Additional IOsonata Modules
# ============================================
FATFS_ROOT = `$(IOSONATA_ROOT)/fatfs
LITTLEFS_ROOT = `$(IOSONATA_ROOT)/littlefs
MICRO_ECC_ROOT = `$(IOSONATA_ROOT)/micro-ecc

# ============================================
# Common Include Paths (for -I flags)
# ============================================
IOSONATA_INCLUDES = -I`$(IOSONATA_INCLUDE) \
                    -I`$(IOSONATA_INCLUDE)/bluetooth \
                    -I`$(IOSONATA_INCLUDE)/audio \
                    -I`$(IOSONATA_INCLUDE)/converters \
                    -I`$(IOSONATA_INCLUDE)/coredev \
                    -I`$(IOSONATA_INCLUDE)/display \
                    -I`$(IOSONATA_INCLUDE)/imu \
                    -I`$(IOSONATA_INCLUDE)/miscdev \
                    -I`$(IOSONATA_INCLUDE)/pwrmgnt \
                    -I`$(IOSONATA_INCLUDE)/sensors \
                    -I`$(IOSONATA_INCLUDE)/storage \
                    -I`$(IOSONATA_INCLUDE)/sys \
                    -I`$(IOSONATA_INCLUDE)/usb

ARM_INCLUDES = -I`$(ARM_INCLUDE) \
               -I`$(ARM_CMSIS_INCLUDE)

RISCV_INCLUDES = -I`$(RISCV_INCLUDE)

# ============================================
# Environment Variables (optional)
# ============================================
ifndef IOCOMPOSER_HOME
`$(error IOCOMPOSER_HOME is not set. Please set it to your IOcomposer root directory)
endif

export NRFX_HOME := `$(NRFX_ROOT)
export NRFXLIB_HOME := `$(SDK_NRFXLIB_ROOT)
export NRF5_SDK_HOME := `$(NRF5_SDK_ROOT)
export NRF5_SDK_MESH_HOME := `$(NRF5_SDK_MESH_ROOT)
export BSEC_HOME := `$(BSEC_ROOT)
"@

Set-Content -Path $MAKEFILE_PATH_MK -Value $makefileContent

Write-Host "   `[OK`] makefile_path.mk created at: $MAKEFILE_PATH_MK" -ForegroundColor Green
Write-Host "   Projects can include it with: include `$(IOSONATA_ROOT)/makefile_path.mk"
Write-Host "   Make sure to set IOCOMPOSER_HOME=$ROOT_UNIX in your environment or Makefile"
Write-Host ""

# ---------------------------------------------------------
# Install IOsonata Eclipse Plugin (helper function)
# ---------------------------------------------------------
function Install-IosonataPlugin {
    param([string]$EclipseDir)
    
    Write-Host ""
    Write-Host ">>> Installing IOsonata Eclipse Plugin..." -ForegroundColor Cyan
    
    $pluginDir = "$ROOT\IOsonata\Installer\eclipse_plugin"
    $dropinsDir = "$EclipseDir\dropins"
    
    # Check if plugin directory exists
    if (-not (Test-Path $pluginDir)) {
        Write-Host "   `[WARN`] Plugin directory not found at $pluginDir" -ForegroundColor Yellow
        Write-Host "    Skipping plugin installation."
        return
    }
    
    # Find the latest plugin jar file
    $pluginFiles = Get-ChildItem -Path $pluginDir -Filter "org.iosonata.embedcdt.templates.wizard_*.jar" -ErrorAction SilentlyContinue
    
    if (-not $pluginFiles -or $pluginFiles.Count -eq 0) {
        Write-Host "   `[WARN`] No IOsonata plugin jar file found in $pluginDir" -ForegroundColor Yellow
        Write-Host "    Skipping plugin installation."
        return
    }
    
    # Sort by version and get the latest
    $latestPlugin = $pluginFiles | Sort-Object Name -Descending | Select-Object -First 1
    
    Write-Host "   -> Found plugin: $($latestPlugin.Name)" -ForegroundColor Green
    
    # Create dropins directory if it doesn't exist
    if (-not (Test-Path $dropinsDir)) {
        New-Item -Path $dropinsDir -ItemType Directory -Force | Out-Null
    }
    
    # Remove old versions of the plugin
    $oldPlugins = Get-ChildItem -Path $dropinsDir -Filter "org.iosonata.embedcdt.templates.wizard_*.jar" -ErrorAction SilentlyContinue
    
    if ($oldPlugins) {
        Write-Host "   -> Removing old plugin versions..." -ForegroundColor Yellow
        foreach ($oldPlugin in $oldPlugins) {
            Write-Host "      - Removing: $($oldPlugin.Name)"
            Remove-Item -Path $oldPlugin.FullName -Force
        }
    }
    
    # Copy the latest plugin to dropins
    Write-Host "   -> Installing plugin to $dropinsDir" -ForegroundColor Cyan
    Copy-Item -Path $latestPlugin.FullName -Destination $dropinsDir -Force
    
    Write-Host "   `[OK`] IOsonata Eclipse Plugin installed: $($latestPlugin.Name)" -ForegroundColor Green
}

# ---------------------------------------------------------
# Configure Eclipse (if requested)
# ---------------------------------------------------------
if ($Eclipse) {
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host "Configuring Eclipse System Properties" -ForegroundColor White
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host ""
    
    $ECLIPSE_DIR = "$env:ProgramFiles\Eclipse Embedded CDT"
    $ECLIPSE_INI = Join-Path $ECLIPSE_DIR "eclipse.ini"
    
    if (-not (Test-Path $ECLIPSE_INI)) {
        Write-Host "   `[ERR`] Eclipse not found at: $ECLIPSE_DIR" -ForegroundColor Red
        Write-Host "   Please install Eclipse first or skip Eclipse configuration." -ForegroundColor Yellow
    } else {
        Write-Host "   `[OK`] Eclipse found" -ForegroundColor Green
        Write-Host ""
        Write-Host ">>> Configuring IOsonata and IOcomposer system properties..."
        
        # Convert Windows paths to forward slashes for Java
        $ROOT_JAVA = $ROOT -replace '\\', '/'
        $IOSONATA_PATH_JAVA = $ROOT_JAVA

        # Read eclipse.ini content
        $iniContent = Get-Content $ECLIPSE_INI
        
        # Remove old properties if they exist
        $iniContent = $iniContent | Where-Object { 
            $_ -notmatch '^-Diosonata\.home=' -and 
            $_ -notmatch '^-Diosonata_loc=' -and 
            $_ -notmatch '^-Diocomposer_home=' 
        }
        
        # Find -vmargs line index
        $vmArgsIndex = -1
        for ($i = 0; $i -lt $iniContent.Count; $i++) {
            if ($iniContent[$i] -eq '-vmargs') {
                $vmArgsIndex = $i
                break
            }
        }
        
        if ($vmArgsIndex -ge 0) {
            # Insert after -vmargs
            $newContent = @()
            $newContent += $iniContent[0..$vmArgsIndex]
            $newContent += "-Diosonata_loc=$IOSONATA_PATH_JAVA"
            $newContent += "-Diocomposer_home=$ROOT_JAVA"
            if ($vmArgsIndex + 1 -lt $iniContent.Count) {
                $newContent += $iniContent[($vmArgsIndex + 1)..($iniContent.Count - 1)]
            }
            $iniContent = $newContent
        } else {
            # No -vmargs section, add it at the end
            $iniContent += "-vmargs"
            $iniContent += "-Diosonata_loc=$IOSONATA_PATH_JAVA"
            $iniContent += "-Diocomposer_home=$ROOT_JAVA"
        }
        
        # Write back to eclipse.ini (requires admin if in Program Files)
        try {
            Set-Content -Path $ECLIPSE_INI -Value $iniContent
            
            Write-Host "   `[OK`] Eclipse system properties configured:" -ForegroundColor Green
            Write-Host "   iosonata_loc = $IOSONATA_PATH_JAVA" -ForegroundColor White
            Write-Host "   iocomposer_home = $ROOT_JAVA" -ForegroundColor White
            Write-Host ""
            Write-Host "   `[INFO`] Usage in CDT projects:" -ForegroundColor Yellow
            Write-Host "   `${system_property:iosonata_loc}/include"
            Write-Host "   `${system_property:iocomposer_home}/external/nrfx"
            Write-Host ""
            
            # Install Eclipse plugin
            Install-IosonataPlugin -EclipseDir $ECLIPSE_DIR
            Write-Host ""
            
            Write-Host "   `[WARN`] Restart Eclipse for changes to take effect." -ForegroundColor Yellow
        } catch {
            Write-Host "   `[ERR`] Failed to write eclipse.ini. Try running as Administrator." -ForegroundColor Red
            Write-Host "   Error: $($_.Exception.Message)" -ForegroundColor Red
        }
    }
}

# ---------------------------------------------------------
# Summary
# ---------------------------------------------------------
Write-Host ""
Write-Host "=========================================================" -ForegroundColor Blue
Write-Host "Summary" -ForegroundColor White
Write-Host "=========================================================" -ForegroundColor Blue
Write-Host "IOsonata Root:        $ROOT" -ForegroundColor White
Write-Host "IOsonata Framework:   $ROOT\IOsonata" -ForegroundColor White
Write-Host "External Repos:       $EXT" -ForegroundColor White
Write-Host ""
Write-Host "Cloned repositories:" -ForegroundColor White
Write-Host "  - IOsonata"
Write-Host "  - nrfx"
Write-Host "  - sdk-nrf-bm"
Write-Host "  - sdk-nrfxlib"
Write-Host "  - nRF5_SDK"
Write-Host "  - nRF5_SDK_Mesh"
Write-Host "  - BSEC"
Write-Host "  - Fusion"
Write-Host "  - lvgl"
Write-Host "  - lwip"
Write-Host "  - FreeRTOS-Kernel"
Write-Host "  - tinyusb"
Write-Host ""

if ($Eclipse -and (Test-Path "$env:ProgramFiles\Eclipse Embedded CDT\eclipse.ini")) {
    Write-Host "Eclipse Configuration:" -ForegroundColor White
    Write-Host "  - System properties configured"
    Write-Host "  - iosonata_loc = $ROOT\IOsonata"
    Write-Host "  - iocomposer_home = $ROOT"
    Write-Host ""
}

Write-Host "---------------------------------------------------------" -ForegroundColor Blue

# --- Auto-Build IOsonata Libraries (if Eclipse detected) ---
if ($ECLIPSE_INSTALLED) {
    $BUILD_SCRIPT = "$ROOT\IOsonata\Installer\build_iosonata_lib_win.ps1"
    
    if (Test-Path $BUILD_SCRIPT) {
        Write-Host ""
        Write-Host "=========================================================" -ForegroundColor Blue
        Write-Host "   IOsonata Library Auto-Build" -ForegroundColor White
        Write-Host "=========================================================" -ForegroundColor Blue
        Write-Host ""
        & $BUILD_SCRIPT -SdkHome $ROOT
    } else {
        Write-Host ""
        Write-Host "Note: Build script not found at:" -ForegroundColor Yellow
        Write-Host "      $BUILD_SCRIPT"
        Write-Host ""
        Write-Host "To build libraries, download build script from:"
        Write-Host "  https://github.com/IOsonata/IOsonata"
        Write-Host ""
    }
} else {
    Write-Host ""
    Write-Host "Note: Eclipse not detected." -ForegroundColor Yellow
    Write-Host "      To build IOsonata libraries, install Eclipse and run:"
    Write-Host "      .\install_iocdevtools_win.ps1"
    Write-Host ""
}

Write-Host "Done. Happy building!" -ForegroundColor Green
Write-Host ""