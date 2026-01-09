#Requires -RunAsAdministrator

<#
.SYNOPSIS
    Installs IOcomposer MCU Development Tools on Windows.
.DESCRIPTION
    Installs Python, 7-Zip, build tools, and Eclipse to Program Files.
#>
param (
    [string]$SdkHome = "$env:USERPROFILE\IOcomposer",
    [switch]$ForceUpdate,
    [switch]$Uninstall,
    [switch]$Version,
    [switch]$Help
)

$ErrorActionPreference = 'Stop'
$SCRIPT_NAME = "install_iocdevtools_windows.ps1"
$SCRIPT_VERSION = "v1.0.92-win"

function Show-Help {
@"
Usage: .\$SCRIPT_NAME [OPTION]

Options:
  -Help            Show help and exit
  -Version         Show version and exit
  -SdkHome <path>  Set custom SDK installation root
  -ForceUpdate     Force reinstall
  -Uninstall       Remove toolchains + Eclipse
"@
}

if ($Help) { Show-Help; exit 0 }
if ($Version) { Write-Host "$($SCRIPT_NAME.Split('.')[0]) $SCRIPT_VERSION"; exit 0 }

$ROOT = $SdkHome
$TOOLS = "$env:ProgramFiles\xPacks"
$ECLIPSE_DIR = "$env:ProgramFiles\Eclipse Embedded CDT"
$SEVENZIP_PATH = "C:\Program Files\7-Zip\7z.exe"
$PythonExecutablePath = $null

# --- Header ---
Write-Host "==============================================" -ForegroundColor Green
Write-Host "   IOcomposer MCU Dev Tools Installer (Windows) " -ForegroundColor Green
Write-Host "   Version: $SCRIPT_VERSION"
Write-Host "=============================================="
Write-Host

# --- Mode Detection ---
$MODE = "install"
if ($ForceUpdate) { $MODE = "force"; Write-Host ">>> Force update mode enabled" -ForegroundColor Yellow }
if ($Uninstall) { $MODE = "uninstall"; Write-Host ">>> Uninstall mode enabled" -ForegroundColor Yellow }

if ($MODE -ne "uninstall") {
    Write-Host ">>> Using SDK home folder: $ROOT"
    Write-Host ">>> Tools will be installed in: $TOOLS"
}

# --- Helpers ---
function Find-PythonExecutable {
    $pythonInPath = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonInPath -and $pythonInPath.Source -notlike "*\Microsoft\WindowsApps\*") {
        Write-Host "   [OK] Found Python in PATH: $($pythonInPath.Source)" -ForegroundColor Green
        return $pythonInPath.Source
    }
    $searchPaths = @( "$env:LOCALAPPDATA\Programs\Python", "$env:ProgramFiles\Python" )
    foreach ($path in $searchPaths) {
        if (Test-Path $path) {
            $found = Get-ChildItem -Path $path -Directory -Filter "Python3*" | 
                     Where-Object { Test-Path (Join-Path $_.FullName "python.exe") } | 
                     Sort-Object Name -Descending | Select-Object -First 1
            if ($found) {
                $py = Join-Path $found.FullName "python.exe"
                Write-Host "   [OK] Found Python at: $py" -ForegroundColor Green
                return $py
            }
        }
    }
    return $null
}

function Java-Hash {
    param([string]$InputString)
    $script = 'import sys; s=sys.argv[1]; h=0; [h := (31*h + ord(c)) & 0xFFFFFFFF for c in s]; print(h - 2**32 if h >= 2**31 else h)'
    return (& $PythonExecutablePath -c $script $InputString).Trim()
}

function Get-ToolVersion { 
    param([string]$ToolPath, [string]$ToolName)
    if ([string]::IsNullOrEmpty($ToolPath)) { return "Not found" }
    $exePath = Join-Path $ToolPath "bin\$ToolName.exe"
    if (-not (Test-Path $exePath)) { return "Not found" }
    $previousErrorAction = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
    $rawOutput = & $exePath --version *>&1
    $ErrorActionPreference = $previousErrorAction
    $ver = $rawOutput | Select-Object -First 1
    if ($ver -match '(\d+\.\d+\.\d+)') { return $matches[1] } else { return "Unknown" }
}

# --- Pre-flight Checks (Python + 7-Zip) ---
if ($MODE -ne "uninstall") {
    # 1. Check Python
    $PythonExecutablePath = Find-PythonExecutable
    if (-not $PythonExecutablePath) {
        Write-Host "[INFO] Python not found. Installing..." -ForegroundColor Yellow
        try {
            $url = "https://www.python.org/ftp/python/3.12.4/python-3.12.4-amd64.exe"
            $inst = Join-Path $env:TEMP "python-installer.exe"
            curl.exe -L -o $inst $url
            Start-Process -FilePath $inst -ArgumentList "/quiet InstallAllUsers=1 PrependPath=1" -Wait
            $PythonExecutablePath = Find-PythonExecutable
            if (-not $PythonExecutablePath) { Throw "Python install failed." }
        } catch { Write-Host "[ERROR] $_" -ForegroundColor Red; exit 1 }
    }

    # 2. Check 7-Zip (Auto-Install)
    if (-not (Test-Path $SEVENZIP_PATH)) {
        Write-Host "[INFO] 7-Zip not found. Downloading & Installing..." -ForegroundColor Yellow
        try {
            # Download 7-Zip 24.07 (latest stable x64)
            $7zUrl = "https://www.7-zip.org/a/7z2407-x64.exe"
            $7zInst = Join-Path $env:TEMP "7z-installer.exe"
            
            Write-Host "   -> Downloading 7-Zip..."
            curl.exe -L -o $7zInst $7zUrl
            
            Write-Host "   -> Installing 7-Zip..."
            # /S runs silent install to default C:\Program Files\7-Zip
            Start-Process -FilePath $7zInst -ArgumentList "/S" -Wait
            
            if (Test-Path $SEVENZIP_PATH) {
                Write-Host "   [OK] 7-Zip installed successfully." -ForegroundColor Green
            } else {
                Throw "7-Zip installation finished but executable not found at $SEVENZIP_PATH"
            }
        } catch {
            Write-Host "[ERROR] Failed to install 7-Zip: $_" -ForegroundColor Red
            exit 1
        }
    } else {
        Write-Host "   [OK] Found 7-Zip." -ForegroundColor Green
    }

    if (-not (Get-Command git -ErrorAction SilentlyContinue)) { Write-Host "[ERROR] Git not found. Please install Git for Windows." -ForegroundColor Red; exit 1 }
}

# --- Directory Setup ---
if (-not (Test-Path $ROOT)) { New-Item -Path $ROOT -ItemType Directory -Force | Out-Null }
$EXT = "$ROOT\external"; if (-not (Test-Path $EXT)) { New-Item -Path $EXT -ItemType Directory -Force | Out-Null }
if (-not (Test-Path $TOOLS)) { New-Item -Path $TOOLS -ItemType Directory -Force | Out-Null }

function Add-DefenderExclusion {
    param([string]$Path)
    try {
        if ((Get-MpPreference).ExclusionPath -notcontains $Path) {
            Set-MpPreference -ExclusionPath $Path
            Write-Host "   [OK] Added Defender exclusion: $Path" -ForegroundColor Green
        }
    } catch { Write-Host "[WARN] Failed to set Defender exclusion." -ForegroundColor Yellow }
}
Add-DefenderExclusion -Path $ROOT

# --- Uninstall ---
if ($MODE -eq "uninstall") {
    $ans = Read-Host "Remove toolchains + Eclipse? (y/N)"
    if ($ans -ne 'y') { exit 0 }
    if (Test-Path $TOOLS) { Remove-Item $TOOLS -Recurse -Force }
    if (Test-Path $ECLIPSE_DIR) { Remove-Item $ECLIPSE_DIR -Recurse -Force }
    Remove-Item "$env:USERPROFILE\.p2", "$env:USERPROFILE\.eclipse" -Recurse -Force -ErrorAction SilentlyContinue
    Write-Host ">>> Uninstall complete!" -ForegroundColor Green
    exit 0
}

# --- Arch ---
$ARCH = $env:PROCESSOR_ARCHITECTURE
$PLATFORM = if ($ARCH -eq "ARM64") { "win32-arm64" } else { "win32-x64" }

# --- Install xPack ---
function Add-ToSystemPath {
    param([string]$Dir)
    $curr = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    if ($curr -split ';' -notcontains $Dir) {
        [Environment]::SetEnvironmentVariable('Path', "$curr;$Dir", 'Machine')
        Write-Host "   [OK] Added to PATH: $Dir"
    }
}

function Install-xPack {
    param([string]$Repo, [string]$Tool, [string]$Name)
    Write-Host ">>> Checking $Name..."
    $json = Invoke-RestMethod -Uri "https://api.github.com/repos/xpack-dev-tools/$Repo/releases/latest"
    $tag = $json.tag_name; $ver = $tag.TrimStart('v'); $base = ($ver -split '-')[0]
    
    $toolExe = Get-Command "$Tool.exe" -ErrorAction SilentlyContinue
    if ($toolExe) {
        $currVer = (& $toolExe.Source --version 2>&1 | Select-Object -First 1)
        if ($currVer -match $base -and $MODE -ne "force") {
            Write-Host "   [OK] $Name up-to-date." -ForegroundColor Green
            return (Split-Path (Split-Path $toolExe.Source -Parent) -Parent)
        }
    }

    $url = ($json.assets | Where-Object { $_.name -like "*$PLATFORM.zip" }).browser_download_url
    if (-not $url) { Write-Host "[ERROR] No URL for $Name" -ForegroundColor Red; return $null }

    Write-Host ">>> Installing $Name $ver..."
    $zip = Join-Path $env:TEMP "$Name.zip"; curl.exe -L -o $zip $url
    $tmp = Join-Path $env:TEMP "xp-extract"; if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
    & $SEVENZIP_PATH x $zip -o"$tmp" -y | Out-Null
    
    $root = Get-ChildItem $tmp -Directory | Select-Object -First 1
    $target = Join-Path $TOOLS $root.Name
    if (Test-Path $target) { Remove-Item $target -Recurse -Force }
    Move-Item $root.FullName $target
    Remove-Item $zip, $tmp -Recurse -Force
    Add-ToSystemPath "$target\bin"
    return $target
}

$BUILDTOOLS_DIR = Install-xPack "windows-build-tools-xpack" "make" "Windows Build Tools"
$ARM_DIR = Install-xPack "arm-none-eabi-gcc-xpack" "arm-none-eabi-gcc" "ARM GCC"
$RISCV_DIR = Install-xPack "riscv-none-elf-gcc-xpack" "riscv-none-elf-gcc" "RISC-V GCC"
$OPENOCD_DIR = Install-xPack "openocd-xpack" "openocd" "OpenOCD"

# --- Install Eclipse ---
Write-Host; Write-Host ">>> Checking Eclipse..." -ForegroundColor Cyan
$MIRROR = "https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"
$RELEASES = (Invoke-WebRequest "$MIRROR/" -UseBasicParsing).Content -split '\r?\n' | 
            Where-Object { $_ -match '20[0-9]{2}-[0-9]{2}' } | 
            ForEach-Object { ([regex]::Match($_, '20[0-9]{2}-[0-9]{2}')).Value } | 
            Sort-Object -Descending | Get-Unique | Select-Object -First 10

$ECLIPSE_URL = ""; $LATEST = ""
foreach ($REL in $RELEASES) {
    $U1 = "$MIRROR/$REL/R/eclipse-embedcdt-$REL-R-win32-x86_64.zip"
    $U2 = "$MIRROR/$REL/R/eclipse-embedcpp-$REL-R-win32-x86_64.zip"
    try { Invoke-WebRequest $U1 -Method Head -EA Stop -UseBasicParsing | Out-Null; $ECLIPSE_URL=$U1; $LATEST=$REL; break } catch {}
    try { Invoke-WebRequest $U2 -Method Head -EA Stop -UseBasicParsing | Out-Null; $ECLIPSE_URL=$U2; $LATEST=$REL; break } catch {}
}

if (-not $ECLIPSE_URL) { Write-Host "[ERROR] Eclipse not found." -ForegroundColor Red; exit 1 }

if ($MODE -eq "force" -or -not (Test-Path $ECLIPSE_DIR)) {
    Write-Host ">>> Installing Eclipse $LATEST..."
    $zip = "$env:TEMP\eclipse.zip"; curl.exe -L -o $zip $ECLIPSE_URL
    if (Test-Path $ECLIPSE_DIR) { Remove-Item $ECLIPSE_DIR -Recurse -Force }
    $tmp = "$env:TEMP\ec-extract"; & $SEVENZIP_PATH x $zip -o"$tmp" -y | Out-Null
    Move-Item "$tmp\eclipse" $ECLIPSE_DIR
    Remove-Item $zip, $tmp -Recurse -Force
    Write-Host "   [OK] Installed at $ECLIPSE_DIR" -ForegroundColor Green
} else {
    Write-Host "   [OK] Eclipse already installed." -ForegroundColor Green
}

# --- Eclipse Shortcuts ---
$WshShell = New-Object -ComObject WScript.Shell
$sc = $WshShell.CreateShortcut("$([Environment]::GetFolderPath('Desktop'))\Eclipse Embedded.lnk")
$sc.TargetPath = "$ECLIPSE_DIR\eclipse.exe"; $sc.Save()
$sc = $WshShell.CreateShortcut("$env:ProgramData\Microsoft\Windows\Start Menu\Programs\Eclipse Embedded.lnk")
$sc.TargetPath = "$ECLIPSE_DIR\eclipse.exe"; $sc.Save()

# --- Seed Prefs ---
$SET = "$ECLIPSE_DIR\configuration\.settings"; New-Item $SET -ItemType Directory -Force | Out-Null
$AH = [long](Java-Hash "xPack GNU Arm Embedded GCC"); if ($AH -lt 0) { $AH += 4294967296 }
$RH = [long](Java-Hash "xPack GNU RISC-V Embedded GCC"); if ($RH -lt 0) { $RH += 4294967296 }; $RH += 1
$AD = $ARM_DIR -replace '\\','/'; $RD = $RISCV_DIR -replace '\\','/'; $OD = $OPENOCD_DIR -replace '\\','/'
$RT = $ROOT -replace '\\','/'; $EX = $EXT -replace '\\','/'

Set-Content "$SET\org.eclipse.embedcdt.core.prefs" "eclipse.preferences.version=1`nxpack.arm.toolchain.path=$AD/bin`nxpack.riscv.toolchain.path=$RD/bin`nxpack.openocd.path=$OD/bin`nxpack.strict=true"
Set-Content "$SET\org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" "toolchain.path.$AH=$AD/bin`ntoolchain.path.1287942917=$ARM_DIR/bin`ntoolchain.path.strict=true"
Set-Content "$SET\org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" "toolchain.path.$RH=$RD/bin`ntoolchain.path.strict=true"
Set-Content "$SET\org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" "install.folder=$OD/bin`ninstall.folder.strict=true"
$BT = (Join-Path $BUILDTOOLS_DIR "bin").Replace('\','\\')
Set-Content "$SET\org.eclipse.embedcdt.managedbuild.cross.core.prefs" "buildTools.path=$BT`neclipse.preferences.version=1"
Set-Content "$SET\org.eclipse.core.runtime.prefs" "eclipse.preferences.version=1`nenvironment/project/IOCOMPOSER_HOME/value=$RT`nenvironment/project/ARM_GCC_HOME/value=$AD/bin`nenvironment/project/RISCV_GCC_HOME/value=$RD/bin`nenvironment/project/OPENOCD_HOME/value=$OD/bin`nenvironment/project/NRFX_HOME/value=$EX/nrfx`nenvironment/project/NRFXLIB_HOME/value=$EX/sdk-nrfxlib`nenvironment/project/NRF5_SDK_HOME/value=$EX/nRF5_SDK`nenvironment/project/NRF5_SDK_MESH_HOME/value=$EX/nRF5_SDK_Mesh`nenvironment/project/BSEC_HOME/value=$EX/BSEC"

# --- eclipse.ini ---
$INI = "$ECLIPSE_DIR\eclipse.ini"; $TXT = Get-Content $INI
$TXT = $TXT | Where-Object { $_ -notmatch '^-Diosonata' -and $_ -notmatch '^-Diocomposer' }
$IDX = $TXT.IndexOf('-vmargs')
if ($IDX -ge 0) {
    $NEW = $TXT[0..$IDX] + "-Diosonata_loc=$RT" + "-Diocomposer_home=$RT" + $TXT[($IDX+1)..($TXT.Count-1)]
} else {
    $NEW = $TXT + "-vmargs" + "-Diosonata_loc=$RT" + "-Diocomposer_home=$RT"
}
Set-Content $INI $NEW
Write-Host "   [OK] System properties configured." -ForegroundColor Green

# --- Plugin ---
function Install-Plugin {
    $PDIR = "$ROOT\IOsonata\Installer\eclipse_plugin"
    if (-not (Test-Path $PDIR)) { return }
    $JAR = Get-ChildItem $PDIR "org.iosonata.embedcdt.templates.firmware_*.jar" | Sort Name -Desc | Select -First 1
    if ($JAR) {
        $DEST = "$ECLIPSE_DIR\dropins"
        New-Item $DEST -ItemType Directory -Force | Out-Null
        Remove-Item "$DEST\org.iosonata.embedcdt.templates.firmware_*.jar" -Force -ErrorAction SilentlyContinue
        Copy-Item $JAR.FullName $DEST -Force
        Write-Host "   [OK] Plugin installed: $($JAR.Name)" -ForegroundColor Green
    }
}

# --- Repos ---
function Sync-Repo { 
    param([string]$U, [string]$D, [bool]$R=$false)
    if (Test-Path $D) {
        if ($MODE -eq 'force') { Remove-Item $D -Recurse -Force; git clone --depth=1 $U $D } 
        else { Push-Location $D; git pull; Pop-Location }
    } else { git clone --depth=1 $U $D }
    if ($R -and (Test-Path $D)) { Push-Location $D; git submodule update --init --recursive; Pop-Location }
}

Write-Host; Write-Host ">>> Syncing Repos..." -ForegroundColor Cyan
Sync-Repo "https://github.com/IOsonata/IOsonata.git" "$ROOT\IOsonata"
$REPOS = @{ 
    "https://github.com/NordicSemiconductor/nrfx.git"="$EXT\nrfx"; 
    "https://github.com/nrfconnect/sdk-nrf-bm.git"="$EXT\sdk-nrf-bm";
    "https://github.com/nrfconnect/sdk-nrfxlib.git"="$EXT\sdk-nrfxlib"; 
    "https://github.com/IOsonata/nRF5_SDK.git"="$EXT\nRF5_SDK"; 
    "https://github.com/IOsonata/nRF5_SDK_Mesh.git"="$EXT\nRF5_SDK_Mesh"; 
    "https://github.com/boschsensortec/Bosch-BSEC2-Library.git"="$EXT\BSEC"; 
    "https://github.com/xioTechnologies/Fusion.git"="$EXT\Fusion";
    "https://github.com/dlaidig/vqf.git"="$EXT\vqf";
    "https://github.com/lvgl/lvgl.git"="$EXT\lvgl";
    "https://github.com/lwip-tcpip/lwip.git"="$EXT\lwip";
    "https://github.com/hathach/tinyusb.git"="$EXT\tinyusb"
}
foreach ($k in $REPOS.Keys) { Sync-Repo $k $REPOS[$k] }
Sync-Repo "https://github.com/FreeRTOS/FreeRTOS-Kernel.git" "$EXT\FreeRTOS-Kernel"

# --- Plugin Install Call ---
Install-Plugin

# --- Makefile ---
$MK = "$ROOT\IOsonata\makefile_path.mk"
$ADU = $ARM_DIR -replace '\\','/'; $RDU = $RISCV_DIR -replace '\\','/'; $ODU = $OPENOCD_DIR -replace '\\','/'
$MK_CONTENT = @"
# makefile_path.mk
# Generated by install_iocdevtools_win.ps1
ifndef IOCOMPOSER_HOME
`$(error IOCOMPOSER_HOME is not set)
endif
IOSONATA_ROOT = `$(IOCOMPOSER_HOME)/IOsonata
IOSONATA_INCLUDE = `$(IOSONATA_ROOT)/include
IOSONATA_SRC = `$(IOSONATA_ROOT)/src
ARM_ROOT = `$(IOSONATA_ROOT)/ARM
ARM_CMSIS = `$(ARM_ROOT)/CMSIS
ARM_CMSIS_INCLUDE = `$(ARM_CMSIS)/Include
ARM_INCLUDE = `$(ARM_ROOT)/include
ARM_SRC = `$(ARM_ROOT)/src
ARM_LDSCRIPT = `$(ARM_ROOT)/ldscript
RISCV_ROOT = `$(IOSONATA_ROOT)/RISCV
RISCV_INCLUDE = `$(RISCV_ROOT)/include
RISCV_SRC = `$(RISCV_ROOT)/src
RISCV_LDSCRIPT = `$(RISCV_ROOT)/ldscript
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
FATFS_ROOT = `$(IOSONATA_ROOT)/fatfs
LITTLEFS_ROOT = `$(IOSONATA_ROOT)/littlefs
MICRO_ECC_ROOT = `$(IOSONATA_ROOT)/micro-ecc
IOSONATA_INCLUDES = -I`$(IOSONATA_INCLUDE) -I`$(IOSONATA_INCLUDE)/bluetooth -I`$(IOSONATA_INCLUDE)/audio -I`$(IOSONATA_INCLUDE)/converters -I`$(IOSONATA_INCLUDE)/coredev -I`$(IOSONATA_INCLUDE)/display -I`$(IOSONATA_INCLUDE)/imu -I`$(IOSONATA_INCLUDE)/miscdev -I`$(IOSONATA_INCLUDE)/pwrmgnt -I`$(IOSONATA_INCLUDE)/sensors -I`$(IOSONATA_INCLUDE)/storage -I`$(IOSONATA_INCLUDE)/sys -I`$(IOSONATA_INCLUDE)/usb
ARM_INCLUDES = -I`$(ARM_INCLUDE) -I`$(ARM_CMSIS_INCLUDE)
RISCV_INCLUDES = -I`$(RISCV_INCLUDE)
export NRFX_HOME := `$(NRFX_ROOT)
export NRFXLIB_HOME := `$(SDK_NRFXLIB_ROOT)
export NRF5_SDK_HOME := `$(NRF5_SDK_ROOT)
export NRF5_SDK_MESH_HOME := `$(NRF5_SDK_MESH_ROOT)
export BSEC_HOME := `$(BSEC_ROOT)
ARM_GCC_ROOT = $ADU
ARM_GCC_BIN = $ADU/bin
ARM_GCC = `$(ARM_GCC_BIN)/arm-none-eabi-gcc
ARM_GPP = `$(ARM_GCC_BIN)/arm-none-eabi-g++
ARM_AS = `$(ARM_GCC_BIN)/arm-none-eabi-as
ARM_LD = `$(ARM_GCC_BIN)/arm-none-eabi-ld
ARM_AR = `$(ARM_GCC_BIN)/arm-none-eabi-ar
ARM_OBJCOPY = `$(ARM_GCC_BIN)/arm-none-eabi-objcopy
ARM_OBJDUMP = `$(ARM_GCC_BIN)/arm-none-eabi-objdump
ARM_SIZE = `$(ARM_GCC_BIN)/arm-none-eabi-size
ARM_GDB = `$(ARM_GCC_BIN)/arm-none-eabi-gdb
RISCV_GCC_ROOT = $RDU
RISCV_GCC_BIN = $RDU/bin
RISCV_GCC = `$(RISCV_GCC_BIN)/riscv-none-elf-gcc
RISCV_GPP = `$(RISCV_GCC_BIN)/riscv-none-elf-g++
RISCV_AS = `$(RISCV_GCC_BIN)/riscv-none-elf-as
RISCV_LD = `$(RISCV_GCC_BIN)/riscv-none-elf-ld
RISCV_AR = `$(RISCV_GCC_BIN)/riscv-none-elf-ar
RISCV_OBJCOPY = `$(RISCV_GCC_BIN)/riscv-none-elf-objcopy
RISCV_OBJDUMP = `$(RISCV_GCC_BIN)/riscv-none-elf-objdump
RISCV_SIZE = `$(RISCV_GCC_BIN)/riscv-none-elf-size
RISCV_GDB = `$(RISCV_GCC_BIN)/riscv-none-elf-gdb
OPENOCD_ROOT = $ODU
OPENOCD = $ODU/bin/openocd
"@
Set-Content $MK $MK_CONTENT
Write-Host "   [OK] makefile_path.mk generated." -ForegroundColor Green

# --- Build ---
$BS = "$ROOT\IOsonata\Installer\build_iosonata_lib_win.ps1"
if (Test-Path $BS) {
    Write-Host; Write-Host ">>> Building IOsonata Libs..." -ForegroundColor Cyan
    & $BS -SdkHome $ROOT
} else {
    Write-Host "   [INFO] Build script not found (will be available after full sync)."
}

Write-Host; Write-Host "==============================================" -ForegroundColor Green
Write-Host " Installation Complete" -ForegroundColor Green
Write-Host "==============================================" -ForegroundColor Green