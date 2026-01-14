#Requires -RunAsAdministrator

<#
.SYNOPSIS
    Installs IOcomposer MCU Development Tools on Windows.
.DESCRIPTION
    Installs Python, 7-Zip, build tools, and Eclipse to Program Files.
    Refined for parity with macOS preference seeding (path hashing).
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
$SCRIPT_VERSION = "v1.0.94-win"

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

$SEVENZIP_EXE = $null
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

function Refresh-EnvVars {
    $machine = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $user    = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$machine;$user"
}

function Find-PythonExecutable {
    Write-Host "   [DEBUG] Searching for Python..." -ForegroundColor DarkGray
    
    $inPath = Get-Command python -ErrorAction SilentlyContinue
    if ($inPath -and $inPath.Source -notlike "*\Microsoft\WindowsApps\*") {
        try { & $inPath.Source --version | Out-Null; return $inPath.Source } catch {}
    }

    $pyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($pyLauncher) {
        try {
            $path = (& $pyLauncher.Source -c "import sys; print(sys.executable)") 2>$null
            if ($path -and (Test-Path $path)) { return $path }
        } catch {}
    }

    $regPaths = @("HKLM:\SOFTWARE\Python\PythonCore", "HKCU:\SOFTWARE\Python\PythonCore")
    foreach ($hkey in $regPaths) {
        if (Test-Path $hkey) {
            $versions = Get-ChildItem $hkey -ErrorAction SilentlyContinue | Sort-Object Name -Descending
            foreach ($v in $versions) {
                $installPathKey = Join-Path $v.PSPath "InstallPath"
                if (Test-Path $installPathKey) {
                    $p = (Get-ItemProperty $installPathKey).'(default)'
                    $exe = Join-Path $p "python.exe"
                    if (Test-Path $exe) { return $exe }
                }
            }
        }
    }
    return $null
}

function Find-7Zip {
    Write-Host "   [DEBUG] Searching for 7-Zip..." -ForegroundColor DarkGray
    $regKeys = @("HKLM:\SOFTWARE\7-Zip", "HKLM:\SOFTWARE\WOW6432Node\7-Zip")
    foreach ($key in $regKeys) {
        if (Test-Path $key) {
            $path = (Get-ItemProperty $key -Name "Path" -ErrorAction SilentlyContinue).Path
            if ($path) { $exe = Join-Path $path "7z.exe"; if (Test-Path $exe) { return $exe } }
        }
    }
    $candidates = @("$env:ProgramFiles\7-Zip\7z.exe", "${env:ProgramFiles(x86)}\7-Zip\7z.exe", "C:\Program Files\7-Zip\7z.exe")
    foreach ($c in $candidates) { if (Test-Path $c) { return $c } }
    return $null
}

function Java-Hash {
    param([string]$InputString)
    # Python script must mirror the exact Java String.hashCode() logic used by macOS script
    $script = 'import sys; s=sys.argv[1]; h=0; [h := (31*h + ord(c)) & 0xFFFFFFFF for c in s]; print(h - 2**32 if h >= 2**31 else h)'
    return (& $PythonExecutablePath -c $script $InputString).Trim()
}

# --- Pre-flight Checks ---
if ($MODE -ne "uninstall") {
    $PythonExecutablePath = Find-PythonExecutable
    if (-not $PythonExecutablePath) {
        Write-Host "[INFO] Python not found. Installing..." -ForegroundColor Yellow
        try {
            $url = "https://www.python.org/ftp/python/3.12.4/python-3.12.4-amd64.exe"
            $inst = Join-Path $env:TEMP "python-installer.exe"
            curl.exe -L -o $inst $url
            Start-Process -FilePath $inst -ArgumentList "/quiet InstallAllUsers=1 PrependPath=1" -Wait
            Refresh-EnvVars; $PythonExecutablePath = Find-PythonExecutable
            if (-not $PythonExecutablePath) { Throw "Python install failed." }
        } catch { Write-Host "[ERROR] $_" -ForegroundColor Red; exit 1 }
    }

    $SEVENZIP_EXE = Find-7Zip
    if (-not $SEVENZIP_EXE) {
        Write-Host "[INFO] 7-Zip not found. Installing..." -ForegroundColor Yellow
        try {
            $7zUrl = "https://www.7-zip.org/a/7z2407-x64.exe"
            $7zInst = Join-Path $env:TEMP "7z-installer.exe"
            curl.exe -L -o $7zInst $7zUrl
            Start-Process -FilePath $7zInst -ArgumentList "/S" -Wait
            $SEVENZIP_EXE = Find-7Zip
            if (-not $SEVENZIP_EXE) { 
                $def = "C:\Program Files\7-Zip\7z.exe"
                if (Test-Path $def) { $SEVENZIP_EXE = $def } else { Throw "7-Zip failed." }
            }
        } catch { Write-Host "[ERROR] $_" -ForegroundColor Red; exit 1 }
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
    } catch { Write-Host "[WARN] Failed to set Defender exclusion (requires Admin)." -ForegroundColor Yellow }
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
    try { $json = Invoke-RestMethod -Uri "https://api.github.com/repos/xpack-dev-tools/$Repo/releases/latest" } catch { exit 1 }
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
    & $SEVENZIP_EXE x $zip -o"$tmp" -y | Out-Null
    
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
$years = 0..3 | ForEach-Object { (Get-Date).Year - $_ }
$months = "12", "09", "06", "03"
$ECLIPSE_URL = ""; $LATEST = ""

Write-Host "   Probing for latest Eclipse release..." -NoNewline
foreach ($y in $years) {
    if ($ECLIPSE_URL) { break }
    foreach ($m in $months) {
        $REL = "$y-$m"
        $U1 = "$MIRROR/$REL/R/eclipse-embedcdt-$REL-R-win32-x86_64.zip"
        $U2 = "$MIRROR/$REL/R/eclipse-embedcpp-$REL-R-win32-x86_64.zip"
        try { if ((Invoke-WebRequest $U1 -Method Head -EA SilentlyContinue).StatusCode -eq 200) { $ECLIPSE_URL=$U1; $LATEST=$REL; break } } catch {}
        try { if ((Invoke-WebRequest $U2 -Method Head -EA SilentlyContinue).StatusCode -eq 200) { $ECLIPSE_URL=$U2; $LATEST=$REL; break } } catch {}
    }
}
Write-Host " Done."

if (-not $ECLIPSE_URL) { Write-Host "`n[ERROR] No Eclipse download found." -ForegroundColor Red; exit 1 }

if ($MODE -eq "force" -or -not (Test-Path $ECLIPSE_DIR)) {
    Write-Host ">>> Installing Eclipse $LATEST..."
    $zip = "$env:TEMP\eclipse.zip"; curl.exe -L -o $zip $ECLIPSE_URL
    if (Test-Path $ECLIPSE_DIR) { Remove-Item $ECLIPSE_DIR -Recurse -Force }
    $tmp = "$env:TEMP\ec-extract"; & $SEVENZIP_EXE x $zip -o"$tmp" -y | Out-Null
    Move-Item "$tmp\eclipse" $ECLIPSE_DIR
    Remove-Item $zip, $tmp -Recurse -Force
    Write-Host "   [OK] Installed at $ECLIPSE_DIR" -ForegroundColor Green
} else { Write-Host "   [OK] Eclipse already installed." -ForegroundColor Green }

# --- Eclipse Shortcuts ---
$WshShell = New-Object -ComObject WScript.Shell
$sc = $WshShell.CreateShortcut("$([Environment]::GetFolderPath('Desktop'))\Eclipse Embedded.lnk")
$sc.TargetPath = "$ECLIPSE_DIR\eclipse.exe"; $sc.Save()
$sc = $WshShell.CreateShortcut("$env:ProgramData\Microsoft\Windows\Start Menu\Programs\Eclipse Embedded.lnk")
$sc.TargetPath = "$ECLIPSE_DIR\eclipse.exe"; $sc.Save()

# --- Seed Prefs (Parity with macOS) ---
Write-Host; Write-Host ">>> Seeding Eclipse preferences..." -ForegroundColor Cyan
$SET = "$ECLIPSE_DIR\configuration\.settings"; New-Item $SET -ItemType Directory -Force | Out-Null

# IMPORTANT: Windows paths must be forward-slashed for Eclipse prefs
$AD = $ARM_DIR -replace '\\','/'; $RD = $RISCV_DIR -replace '\\','/'; $OD = $OPENOCD_DIR -replace '\\','/'
$RT = $ROOT -replace '\\','/'; $EX = $EXT -replace '\\','/'
$BT = (Join-Path $BUILDTOOLS_DIR "bin") -replace '\\','/'

# Hashing must be done on the PATH string, matching macOS logic
$AH = [long](Java-Hash "$AD/bin"); if ($AH -lt 0) { $AH += 4294967296 }
$RH = [long](Java-Hash "$RD/bin"); if ($RH -lt 0) { $RH += 4294967296 }

# 1. org.eclipse.cdt.core.prefs
$cdt_prefs = @"
eclipse.preferences.version=1
environment/buildEnvironmentInclude=true
org.eclipse.cdt.core.parser.taskTags=TODO,FIXME,XXX
"@
Set-Content "$SET\org.eclipse.cdt.core.prefs" $cdt_prefs -Encoding UTF8

# 2. org.eclipse.embedcdt.core.prefs
$embed_prefs = @"
eclipse.preferences.version=1
buildtools.path.$AH=$AD/bin
buildtools.path.$RH=$RD/bin
buildtools.path.strict=true
"@
Set-Content "$SET\org.eclipse.embedcdt.core.prefs" $embed_prefs -Encoding UTF8

# 3. org.eclipse.embedcdt.managedbuild.core.prefs
$build_prefs = @"
eclipse.preferences.version=1
toolchain.path.$AH=$AD/bin
toolchain.path.$RH=$RD/bin
toolchain.path.strict=true
"@
Set-Content "$SET\org.eclipse.embedcdt.managedbuild.core.prefs" $build_prefs -Encoding UTF8

# 4. org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs
$ocd_prefs = @"
install.folder=$OD/bin
install.folder.strict=true
"@
Set-Content "$SET\org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" $ocd_prefs -Encoding UTF8

# 5. org.eclipse.embedcdt.managedbuild.cross.core.prefs (Windows specific: needs Build Tools)
# We add this because Windows doesn't have native 'make'. MacOS/Linux don't strictly need it in prefs if in path.
Set-Content "$SET\org.eclipse.embedcdt.managedbuild.cross.core.prefs" "buildTools.path=$BT`neclipse.preferences.version=1"

# --- eclipse.ini ---
$INI = "$ECLIPSE_DIR\eclipse.ini"; $TXT = Get-Content $INI
$TXT = $TXT | Where-Object { $_ -notmatch '^-Diosonata' -and $_ -notmatch '^-Diocomposer' }
$IDX = $TXT.IndexOf('-vmargs')
if ($IDX -ge 0) {
    $NEW = $TXT[0..$IDX] + "-Diosonata_loc=$RT" + $TXT[($IDX+1)..($TXT.Count-1)]
} else {
    $NEW = $TXT + "-vmargs" + "-Diosonata_loc=$RT"
}
Set-Content $INI $NEW
Write-Host "   [OK] eclipse.ini configured (iosonata_loc)." -ForegroundColor Green

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

Install-Plugin

# --- Makefile ---
$MK = "$ROOT\IOsonata\makefile_path.mk"
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
ARM_GCC_ROOT = $AD
ARM_GCC_BIN = $AD/bin
ARM_GCC = `$(ARM_GCC_BIN)/arm-none-eabi-gcc
ARM_GPP = `$(ARM_GCC_BIN)/arm-none-eabi-g++
ARM_AS = `$(ARM_GCC_BIN)/arm-none-eabi-as
ARM_LD = `$(ARM_GCC_BIN)/arm-none-eabi-ld
ARM_AR = `$(ARM_GCC_BIN)/arm-none-eabi-ar
ARM_OBJCOPY = `$(ARM_GCC_BIN)/arm-none-eabi-objcopy
ARM_OBJDUMP = `$(ARM_GCC_BIN)/arm-none-eabi-objdump
ARM_SIZE = `$(ARM_GCC_BIN)/arm-none-eabi-size
ARM_GDB = `$(ARM_GCC_BIN)/arm-none-eabi-gdb
RISCV_GCC_ROOT = $RD
RISCV_GCC_BIN = $RD/bin
RISCV_GCC = `$(RISCV_GCC_BIN)/riscv-none-elf-gcc
RISCV_GPP = `$(RISCV_GCC_BIN)/riscv-none-elf-g++
RISCV_AS = `$(RISCV_GCC_BIN)/riscv-none-elf-as
RISCV_LD = `$(RISCV_GCC_BIN)/riscv-none-elf-ld
RISCV_AR = `$(RISCV_GCC_BIN)/riscv-none-elf-ar
RISCV_OBJCOPY = `$(RISCV_GCC_BIN)/riscv-none-elf-objcopy
RISCV_OBJDUMP = `$(RISCV_GCC_BIN)/riscv-none-elf-objdump
RISCV_SIZE = `$(RISCV_GCC_BIN)/riscv-none-elf-size
RISCV_GDB = `$(RISCV_GCC_BIN)/riscv-none-elf-gdb
OPENOCD_ROOT = $OD
OPENOCD = $OD/bin/openocd
"@
Set-Content $MK $MK_CONTENT
Write-Host "   [OK] makefile_path.mk generated." -ForegroundColor Green

# --- Build ---
$BS = "$ROOT\IOsonata\Installer\build_iosonata_lib_win.ps1"
if (Test-Path $BS) {
    Write-Host; Write-Host ">>> Building IOsonata Libs..." -ForegroundColor Cyan
    & $BS -SdkHome $ROOT
} else { Write-Host "   [INFO] Build script not found (will be available after full sync)." }

Write-Host; Write-Host "==============================================" -ForegroundColor Green
Write-Host " Installation Complete" -ForegroundColor Green
Write-Host "==============================================" -ForegroundColor Green