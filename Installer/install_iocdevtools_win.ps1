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
$SCRIPT_NAME = "install_iocdevtools_win.ps1"
$SCRIPT_VERSION = "v1.0.100-win"

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
    # Python script mirrors the exact Java String.hashCode() logic used by macOS script
    # Returns signed 32-bit integer (can be negative) - DO NOT adjust afterward
    $script = @'
import sys
s = sys.argv[1]
h = 0
for c in s:
    h = (31 * h + ord(c)) & 0xFFFFFFFF
if h >= 2**31:
    h -= 2**32
print(h)
'@
    return (& $PythonExecutablePath -c $script $InputString).Trim()
}

function Set-ContentNoBom {
    param([string]$Path, [string]$Value)
    # Write UTF-8 without Byte Order Mark (BOM) compatible with Eclipse
    $enc = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Path, $Value, $enc)
}

# --- Pre-flight Checks ---
if ($MODE -ne "uninstall") {
    $PythonExecutablePath = Find-PythonExecutable
    if (-not $PythonExecutablePath) {
        Write-Host "[INFO] Python not found. Installing..." -ForegroundColor Yellow
        try {
            # Detect architecture for Python installer
            $pyArch = if ($env:PROCESSOR_ARCHITECTURE -eq "ARM64") { "arm64" } else { "amd64" }
            $url = "https://www.python.org/ftp/python/3.12.4/python-3.12.4-$pyArch.exe"
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
            # Detect architecture for 7-Zip installer
            $7zArch = if ($env:PROCESSOR_ARCHITECTURE -eq "ARM64") { "arm64" } else { "x64" }
            $7zUrl = "https://www.7-zip.org/a/7z2407-$7zArch.exe"
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
    
    # Remove xPacks toolchains
    if (Test-Path $TOOLS) {
        Write-Host ">>> Removing toolchains from $TOOLS..."
        try {
            # Try to stop any processes that might be locking files
            Get-Process | Where-Object { $_.Path -like "$TOOLS*" } | Stop-Process -Force -ErrorAction SilentlyContinue
            Start-Sleep -Milliseconds 500
            Remove-Item $TOOLS -Recurse -Force -ErrorAction Stop
            Write-Host "   [OK] Toolchains removed." -ForegroundColor Green
        } catch {
            Write-Host "   [WARN] Could not fully remove $TOOLS - some files may be in use." -ForegroundColor Yellow
            Write-Host "   Try closing any applications using these tools and run uninstall again." -ForegroundColor Yellow
        }
    }
    
    # Remove Eclipse
    if (Test-Path $ECLIPSE_DIR) {
        Write-Host ">>> Removing Eclipse from $ECLIPSE_DIR..."
        try {
            Get-Process eclipse -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
            Start-Sleep -Milliseconds 500
            Remove-Item $ECLIPSE_DIR -Recurse -Force -ErrorAction Stop
            Write-Host "   [OK] Eclipse removed." -ForegroundColor Green
        } catch {
            Write-Host "   [WARN] Could not fully remove Eclipse - it may be running." -ForegroundColor Yellow
        }
    }
    
    # Remove Eclipse user data
    Write-Host ">>> Removing Eclipse user settings..."
    Remove-Item "$env:USERPROFILE\.p2", "$env:USERPROFILE\.eclipse" -Recurse -Force -ErrorAction SilentlyContinue
    Write-Host "   [OK] Eclipse user settings removed." -ForegroundColor Green
    
    # Remove IDAP tools
    Write-Host ">>> Removing IDAP tools..."
    $IDAP_DIR = "$ROOT\IDAP"
    if (Test-Path $IDAP_DIR) {
        Remove-Item $IDAP_DIR -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "   [OK] IDAP tools removed." -ForegroundColor Green
    }
    
    # Prompt for IOsonata and external SDK removal
    $ans2 = Read-Host "Also remove IOsonata and external SDK folders? (y/N)"
    if ($ans2 -eq 'y') {
        if (Test-Path "$ROOT\IOsonata") { 
            Remove-Item "$ROOT\IOsonata" -Recurse -Force -ErrorAction SilentlyContinue
            Write-Host "   [OK] IOsonata removed." -ForegroundColor Green
        }
        if (Test-Path "$ROOT\external") { 
            Remove-Item "$ROOT\external" -Recurse -Force -ErrorAction SilentlyContinue 
            Write-Host "   [OK] External SDK removed." -ForegroundColor Green
        }
        if (Test-Path "$ROOT\.iocomposer") { 
            Remove-Item "$ROOT\.iocomposer" -Recurse -Force -ErrorAction SilentlyContinue 
            Write-Host "   [OK] .iocomposer removed." -ForegroundColor Green
        }
    } else {
        Write-Host ">>> Repositories under $ROOT and workspace dirs were kept."
    }
    
    # If the ROOT folder is now empty, remove it (otherwise keep it).
    if (Test-Path $ROOT) {
        $items = Get-ChildItem $ROOT -Force -ErrorAction SilentlyContinue
        if ($items.Count -eq 0) {
            Remove-Item $ROOT -Force -ErrorAction SilentlyContinue
            Write-Host "   [OK] Removed empty root folder: $ROOT" -ForegroundColor Green
        } else {
            Write-Host "   [INFO] Keeping root folder (not empty): $ROOT" -ForegroundColor Yellow
        }
    }
    
    Write-Host ">>> Uninstall complete!" -ForegroundColor Green
    exit 0
}

# --- Arch Detection ---
$ARCH = $env:PROCESSOR_ARCHITECTURE
$XPACK_PLATFORM = if ($ARCH -eq "ARM64") { "win32-arm64" } else { "win32-x64" }
$ECLIPSE_ARCH = if ($ARCH -eq "ARM64") { "aarch64" } else { "x86_64" }
Write-Host ">>> Detected architecture: $ARCH -> Eclipse arch=$ECLIPSE_ARCH"

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
    try { $json = Invoke-RestMethod -Uri "https://api.github.com/repos/xpack-dev-tools/$Repo/releases/latest" } 
    catch { 
        Write-Host "[ERROR] Failed to query GitHub API for $Name" -ForegroundColor Red
        exit 1 
    }
    $tag = $json.tag_name
    $ver = $tag.TrimStart('v')
    $base = ($ver -split '-')[0]
    
    $toolExe = Get-Command "$Tool.exe" -ErrorAction SilentlyContinue
    if ($toolExe) {
        # Fix: Added try-catch to avoid $ErrorActionPreference = 'Stop' from being triggered
        $currVerOutput = try { (& $toolExe.Source --version 2>&1) | Out-String -Stream | Select-Object -First 1 } catch { "" }        # Extract version number for exact comparison
        if ($currVerOutput -match '(\d+\.\d+\.\d+)') {
            $currVer = $Matches[1]
            if ($currVer -eq $base -and $MODE -ne "force") {
                Write-Host "   [OK] $Name up-to-date ($currVer)." -ForegroundColor Green
                return (Split-Path (Split-Path $toolExe.Source -Parent) -Parent)
            }
        }
    }

    # Find asset URL matching our platform
    $url = ($json.assets | Where-Object { $_.name -like "*$XPACK_PLATFORM.zip" }).browser_download_url
    if (-not $url) { 
        Write-Host "[ERROR] No download URL found for $Name ($XPACK_PLATFORM)" -ForegroundColor Red
        return $null 
    }

    Write-Host ">>> Installing $Name $ver..."
    $zip = Join-Path $env:TEMP "$Name.zip"
    curl.exe -L -o $zip $url
    $tmp = Join-Path $env:TEMP "xp-extract"
    if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
    & $SEVENZIP_EXE x $zip -o"$tmp" -y | Out-Null
    
    $root = Get-ChildItem $tmp -Directory | Select-Object -First 1
    if (-not $root) {
        Write-Host "[ERROR] Failed to extract $Name" -ForegroundColor Red
        return $null
    }
    $target = Join-Path $TOOLS $root.Name
    if (Test-Path $target) { Remove-Item $target -Recurse -Force }
    Move-Item $root.FullName $target
    Remove-Item $zip, $tmp -Recurse -Force
    Add-ToSystemPath "$target\bin"
    Write-Host "   [OK] Installed at $target" -ForegroundColor Green
    return $target
}

$BUILDTOOLS_DIR = Install-xPack "windows-build-tools-xpack" "make" "Windows Build Tools"
$ARM_DIR = Install-xPack "arm-none-eabi-gcc-xpack" "arm-none-eabi-gcc" "ARM GCC"
$RISCV_DIR = Install-xPack "riscv-none-elf-gcc-xpack" "riscv-none-elf-gcc" "RISC-V GCC"
$OPENOCD_DIR = Install-xPack "openocd-xpack" "openocd" "OpenOCD"

# Validate critical tools were installed
if (-not $ARM_DIR) { Write-Host "[ERROR] ARM GCC installation failed" -ForegroundColor Red; exit 1 }
if (-not $RISCV_DIR) { Write-Host "[ERROR] RISC-V GCC installation failed" -ForegroundColor Red; exit 1 }
if (-not $OPENOCD_DIR) { Write-Host "[ERROR] OpenOCD installation failed" -ForegroundColor Red; exit 1 }

# --- Install Eclipse ---
Write-Host; Write-Host ">>> Checking Eclipse..." -ForegroundColor Cyan
$MIRROR = "https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"

# Enumerate available releases from mirror
Write-Host "   Probing for latest Eclipse release..." -NoNewline
$ECLIPSE_URL = ""
$LATEST = ""

try {
    $releasePage = Invoke-WebRequest -Uri "$MIRROR/" -UseBasicParsing -TimeoutSec 30
    $releases = [regex]::Matches($releasePage.Content, 'href="(20\d{2}-\d{2})/"') | 
                ForEach-Object { $_.Groups[1].Value } | 
                Sort-Object -Descending -Unique
    
    foreach ($rel in $releases) {
        if ($ECLIPSE_URL) { break }
        # Use architecture-appropriate URL
        $U1 = "$MIRROR/$rel/R/eclipse-embedcdt-$rel-R-win32-$ECLIPSE_ARCH.zip"
        $U2 = "$MIRROR/$rel/R/eclipse-embedcpp-$rel-R-win32-$ECLIPSE_ARCH.zip"
        
        try {
            $resp = Invoke-WebRequest $U1 -Method Head -UseBasicParsing -TimeoutSec 10
            if ($resp.StatusCode -eq 200) { $ECLIPSE_URL = $U1; $LATEST = $rel; break }
        } catch {}
        try {
            $resp = Invoke-WebRequest $U2 -Method Head -UseBasicParsing -TimeoutSec 10
            if ($resp.StatusCode -eq 200) { $ECLIPSE_URL = $U2; $LATEST = $rel; break }
        } catch {}
    }
} catch {
    Write-Host " Failed to enumerate releases." -ForegroundColor Yellow
}

# Fallback to year/month probing if enumeration failed
if (-not $ECLIPSE_URL) {
    $years = 0..3 | ForEach-Object { (Get-Date).Year - $_ }
    $months = "12", "09", "06", "03"
    foreach ($y in $years) {
        if ($ECLIPSE_URL) { break }
        foreach ($m in $months) {
            $REL = "$y-$m"
            $U1 = "$MIRROR/$REL/R/eclipse-embedcdt-$REL-R-win32-$ECLIPSE_ARCH.zip"
            $U2 = "$MIRROR/$REL/R/eclipse-embedcpp-$REL-R-win32-$ECLIPSE_ARCH.zip"
            try { if ((Invoke-WebRequest $U1 -Method Head -UseBasicParsing -EA SilentlyContinue).StatusCode -eq 200) { $ECLIPSE_URL=$U1; $LATEST=$REL; break } } catch {}
            try { if ((Invoke-WebRequest $U2 -Method Head -UseBasicParsing -EA SilentlyContinue).StatusCode -eq 200) { $ECLIPSE_URL=$U2; $LATEST=$REL; break } } catch {}
        }
    }
}
Write-Host " Done."

if (-not $ECLIPSE_URL) { Write-Host "`n[ERROR] No Eclipse download found for architecture $ECLIPSE_ARCH." -ForegroundColor Red; exit 1 }

if ($MODE -eq "force" -or -not (Test-Path $ECLIPSE_DIR)) {
    Write-Host ">>> Installing Eclipse $LATEST..."
    $zip = "$env:TEMP\eclipse.zip"
    curl.exe -L -o $zip $ECLIPSE_URL
    if (Test-Path $ECLIPSE_DIR) { Remove-Item $ECLIPSE_DIR -Recurse -Force }
    $tmp = "$env:TEMP\ec-extract"
    if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
    & $SEVENZIP_EXE x $zip -o"$tmp" -y | Out-Null
    Move-Item "$tmp\eclipse" $ECLIPSE_DIR
    Remove-Item $zip, $tmp -Recurse -Force
    Write-Host "   [OK] Installed at $ECLIPSE_DIR" -ForegroundColor Green
} else { 
    Write-Host "   [OK] Eclipse already installed (use -ForceUpdate to reinstall)." -ForegroundColor Green 
}

# --- Eclipse Shortcuts ---
$WshShell = New-Object -ComObject WScript.Shell
$sc = $WshShell.CreateShortcut("$([Environment]::GetFolderPath('Desktop'))\Eclipse Embedded.lnk")
$sc.TargetPath = "$ECLIPSE_DIR\eclipse.exe"; $sc.Save()
$sc = $WshShell.CreateShortcut("$env:ProgramData\Microsoft\Windows\Start Menu\Programs\Eclipse Embedded.lnk")
$sc.TargetPath = "$ECLIPSE_DIR\eclipse.exe"; $sc.Save()

# --- Seed Prefs to BOTH locations (required for Eclipse version compatibility) ---
# Location 1: Eclipse installation directory (works immediately)
# Location 2: User directory ~/.eclipse (required by some Eclipse versions)

# IMPORTANT: Windows paths must be forward-slashed for Eclipse prefs
$AD = $ARM_DIR -replace '\\','/'
$RD = $RISCV_DIR -replace '\\','/'
$OD = $OPENOCD_DIR -replace '\\','/'
$RT = $ROOT -replace '\\','/'

# Build tools path (may be null if not installed)
$BT = ""
if ($BUILDTOOLS_DIR) {
    $BT = (Join-Path $BUILDTOOLS_DIR "bin") -replace '\\','/'
}

# Hash calculation - use toolchain NAME strings like macOS does (not paths)
# The Java-Hash function returns signed 32-bit integer
# Convert negative to unsigned for Eclipse prefs
$ARM_HASH_RAW = [long](Java-Hash "xPack GNU Arm Embedded GCC")
if ($ARM_HASH_RAW -lt 0) { $ARM_HASH_RAW = $ARM_HASH_RAW + 4294967296 }
$AH = $ARM_HASH_RAW

$RISCV_HASH_RAW = [long](Java-Hash "xPack GNU RISC-V Embedded GCC")
if ($RISCV_HASH_RAW -lt 0) { $RISCV_HASH_RAW = $RISCV_HASH_RAW + 4294967296 }
$RH = $RISCV_HASH_RAW + 1  # +1 matches macOS behavior

# Function to write ALL preference files to installation directory
function Write-EclipseInstallPrefs {
    param([string]$SettingsDir)
    
    New-Item $SettingsDir -ItemType Directory -Force | Out-Null

    # 1. org.eclipse.core.runtime.prefs (with environment variables like macOS)
    $runtime_prefs = @"
eclipse.preferences.version=1
environment/project/IOCOMPOSER_HOME/value=$RT
environment/project/ARM_GCC_HOME/value=$AD/bin
environment/project/RISCV_GCC_HOME/value=$RD/bin
environment/project/OPENOCD_HOME/value=$OD/bin
"@
    Set-ContentNoBom -Path "$SettingsDir\org.eclipse.core.runtime.prefs" -Value $runtime_prefs

    # 2. org.eclipse.cdt.core.prefs
    $cdt_prefs = @"
eclipse.preferences.version=1
environment/buildEnvironmentInclude=true
org.eclipse.cdt.core.parser.taskTags=TODO,FIXME,XXX
"@
    Set-Content "$SettingsDir\org.eclipse.cdt.core.prefs" $cdt_prefs -Encoding UTF8

    # 3. org.eclipse.embedcdt.core.prefs (xPack paths)
    $embed_prefs = @"
eclipse.preferences.version=1
xpack.arm.toolchain.path=$AD
xpack.riscv.toolchain.path=$RD
xpack.openocd.path=$OD
xpack.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.core.prefs" $embed_prefs -Encoding UTF8

    # 4a. org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs (ARM-specific)
    $arm_prefs = @"
eclipse.preferences.version=1
toolchain.path.$AH=$AD
toolchain.path.1287942917=$AD
toolchain.path.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" $arm_prefs -Encoding UTF8

    # 4b. org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs (RISC-V-specific)
    $riscv_prefs = @"
eclipse.preferences.version=1
toolchain.path.$RH=$RD
toolchain.path.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" $riscv_prefs -Encoding UTF8

    # 5. org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs
    $ocd_prefs = @"
eclipse.preferences.version=1
executable=bin\\openocd.exe
install.folder=$OD
install.folder.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" $ocd_prefs -Encoding UTF8

    # 7. org.eclipse.embedcdt.managedbuild.cross.core.prefs (Windows specific: needs Build Tools for make)
    if ($BT) {
        $cross_prefs = @"
eclipse.preferences.version=1
buildTools.path=$BT
"@
        Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.core.prefs" $cross_prefs -Encoding UTF8
    }
}

# Function to write user prefs to user directory (matching macOS behavior)
function Write-EclipseUserPrefs {
    param([string]$SettingsDir)
    
    New-Item $SettingsDir -ItemType Directory -Force | Out-Null

    # User prefs files (matching macOS behavior)

    # 1. org.eclipse.embedcdt.core.prefs
    $embed_prefs = @"
eclipse.preferences.version=1
xpack.arm.toolchain.path=$AD
xpack.riscv.toolchain.path=$RD
xpack.openocd.path=$OD
xpack.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.core.prefs" $embed_prefs -Encoding UTF8

    # 2. org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs
    $arm_prefs = @"
eclipse.preferences.version=1
toolchain.path.$AH=$AD
toolchain.path.1287942917=$AD
toolchain.path.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" $arm_prefs -Encoding UTF8

    # 3. org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs
    $riscv_prefs = @"
eclipse.preferences.version=1
toolchain.path.$RH=$RD
toolchain.path.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" $riscv_prefs -Encoding UTF8

    # 4. org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs
    $ocd_prefs = @"
eclipse.preferences.version=1
executable=bin\\openocd.exe
install.folder=$OD
install.folder.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" $ocd_prefs -Encoding UTF8

    # 5. org.eclipse.core.runtime.prefs
    $runtime_prefs = @"
eclipse.preferences.version=1
environment/project/IOCOMPOSER_HOME/value=$RT
environment/project/ARM_GCC_HOME/value=$AD/bin
environment/project/RISCV_GCC_HOME/value=$RD/bin
environment/project/OPENOCD_HOME/value=$OD/bin
environment/project/NRFX_HOME/value=$RT/external/nrfx
environment/project/NRFXLIB_HOME/value=$RT/external/sdk-nrfxlib
environment/project/NRF5_SDK_HOME/value=$RT/external/nRF5_SDK
environment/project/NRF5_SDK_MESH_HOME/value=$RT/external/nRF5_SDK_Mesh
environment/project/BSEC_HOME/value=$RT/external/BSEC
"@
    Set-ContentNoBom -Path "$SettingsDir\org.eclipse.core.runtime.prefs" -Value $runtime_prefs
}

# --- Location 1: Eclipse Installation Directory ---
Write-Host; Write-Host ">>> Seeding Eclipse MCU preferences (installation directory)..." -ForegroundColor Cyan
$INSTALL_SET = "$ECLIPSE_DIR\configuration\.settings"
Write-EclipseInstallPrefs $INSTALL_SET
Write-Host "   [OK] Preferences seeded in $INSTALL_SET" -ForegroundColor Green

# --- Location 2: User Directory (~/.eclipse) ---
# First, initialize Eclipse to create the user configuration directory (like macOS)
Write-Host; Write-Host ">>> Initializing Eclipse to create instance configuration..." -ForegroundColor Cyan
$eclipseExe = "$ECLIPSE_DIR\eclipse.exe"
if (Test-Path $eclipseExe) {
    try {
        $initProcess = Start-Process -FilePath $eclipseExe -ArgumentList "-nosplash", "-initialize" -Wait -PassThru -WindowStyle Hidden
        if ($initProcess.ExitCode -eq 0) {
            Write-Host "   [OK] Eclipse initialized." -ForegroundColor Green
        } else {
            Write-Host "   [WARN] Eclipse initialization returned exit code $($initProcess.ExitCode)" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "   [WARN] Eclipse initialization failed: $_" -ForegroundColor Yellow
    }
}

Write-Host; Write-Host ">>> Seeding Eclipse MCU preferences (user directory)..." -ForegroundColor Cyan
$USER_ECLIPSE = "$env:USERPROFILE\.eclipse"

# Find existing Eclipse platform configuration directories
$userConfigs = @()
if (Test-Path $USER_ECLIPSE) {
    $userConfigs = Get-ChildItem $USER_ECLIPSE -Directory -Filter "org.eclipse.platform_*" -ErrorAction SilentlyContinue |
                   Sort-Object Name -Descending
}

if ($userConfigs.Count -gt 0) {
    foreach ($cfg in $userConfigs) {
        $userSet = Join-Path $cfg.FullName "configuration\.settings"
        Write-EclipseUserPrefs $userSet
        Write-Host "   [OK] Preferences seeded in $userSet" -ForegroundColor Green
    }
} else {
    Write-Host "   [INFO] User Eclipse config not found (Eclipse not yet run)." -ForegroundColor Yellow
    Write-Host "   [INFO] Installation-level prefs are set; user prefs will be created on next run." -ForegroundColor Yellow
}

# --- eclipse.ini ---
Write-Host; Write-Host ">>> Configuring eclipse.ini..." -ForegroundColor Cyan
$INI = "$ECLIPSE_DIR\eclipse.ini"

# Backup before modifying
Copy-Item $INI "$INI.bak" -Force

$TXT = Get-Content $INI
# Remove old properties if they exist (match macOS behavior)
$TXT = $TXT | Where-Object { $_ -notmatch '^-Diosonata' -and $_ -notmatch '^-Diocomposer' }
$IDX = [array]::IndexOf($TXT, '-vmargs')
if ($IDX -ge 0) {
    $before = $TXT[0..$IDX]
    $after = if ($IDX -lt $TXT.Count - 1) { $TXT[($IDX+1)..($TXT.Count-1)] } else { @() }
    # Add both properties like macOS does
    $NEW = $before + "-Diosonata_loc=$RT" + "-Diocomposer_home=$RT" + $after
} else {
    # No -vmargs section, add it at the end with our properties
    $NEW = $TXT + "-vmargs" + "-Diosonata_loc=$RT" + "-Diocomposer_home=$RT"
}
Set-Content $INI $NEW
Write-Host "   [OK] eclipse.ini configured:" -ForegroundColor Green
Write-Host "        iosonata_loc=$RT"
Write-Host "        iocomposer_home=$RT"

# --- Plugin ---
function Install-Plugin {
    Write-Host; Write-Host ">>> Installing IOsonata Eclipse Plugin..." -ForegroundColor Cyan
    $PDIR = "$ROOT\IOsonata\Installer\eclipse_plugin"
    if (-not (Test-Path $PDIR)) { 
        Write-Host "   [WARN] Plugin directory not found at $PDIR" -ForegroundColor Yellow
        return 
    }
    $JAR = Get-ChildItem $PDIR "org.iosonata.embedcdt.templates.wizard_*.jar" -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1
    if ($JAR) {
        $DEST = "$ECLIPSE_DIR\dropins"
        New-Item $DEST -ItemType Directory -Force | Out-Null
        # Remove old versions
        Get-ChildItem "$DEST\org.iosonata.embedcdt.templates.wizard_*.jar" -ErrorAction SilentlyContinue | Remove-Item -Force
        Copy-Item $JAR.FullName $DEST -Force
        Write-Host "   [OK] Plugin installed: $($JAR.Name)" -ForegroundColor Green
    } else {
        Write-Host "   [WARN] No plugin jar found in $PDIR" -ForegroundColor Yellow
    }
}

# ---------------------------------------------------------
# Sync IOsonata + external SDKs (single source of truth)
# ---------------------------------------------------------
Write-Host; Write-Host ">>> Syncing IOsonata + external SDKs..." -ForegroundColor Cyan

$CLONE_URL = "https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/clone_iosonata_sdk_win.ps1"
$TMP_CLONE = Join-Path $env:TEMP "clone_iosonata_sdk_win.ps1"
$CLONE_LOG = Join-Path $env:TEMP "clone_iosonata_sdk_win.log"

try {
    curl.exe -fsSL $CLONE_URL -o $TMP_CLONE
    if ($LASTEXITCODE -ne 0) { throw "Download failed" }
} catch {
    Write-Host "[ERROR] Failed to download clone script:" -ForegroundColor Red
    Write-Host "   $CLONE_URL" -ForegroundColor Red
    exit 1
}

# Prefer -NoBuild if supported by the clone script (newer versions). If not supported,
# retry without it. We capture output so we can detect whether the builder ran.
$CLONE_INVOKED_BUILD = $false

# First, check if the clone script supports -NoBuild by examining its content
$cloneScriptContent = Get-Content $TMP_CLONE -Raw -ErrorAction SilentlyContinue
$supportsNoBuild = $cloneScriptContent -match '\$NoBuild|\-NoBuild'

$cloneArgs = @("-SdkHome", $ROOT)
if ($supportsNoBuild) {
    $cloneArgs += "-NoBuild"
}
if ($MODE -eq "force") {
    $cloneArgs += "-ForceUpdate"
}

# Run clone script and capture output
$ErrorActionPreference = 'Continue'
$cloneOutput = & powershell.exe -ExecutionPolicy Bypass -File $TMP_CLONE @cloneArgs 2>&1 | Tee-Object -FilePath $CLONE_LOG
$cloneExitCode = $LASTEXITCODE
$cloneOutput | Write-Host
$ErrorActionPreference = 'Stop'

# If clone failed and we tried -NoBuild, retry without it
if ($cloneExitCode -ne 0 -and $supportsNoBuild) {
    Write-Host "   [WARN] Clone script may not support -NoBuild yet. Retrying without it..." -ForegroundColor Yellow
    $cloneArgs = @("-SdkHome", $ROOT)
    if ($MODE -eq "force") {
        $cloneArgs += "-ForceUpdate"
    }
    $ErrorActionPreference = 'Continue'
    $cloneOutput = & powershell.exe -ExecutionPolicy Bypass -File $TMP_CLONE @cloneArgs 2>&1 | Tee-Object -FilePath $CLONE_LOG
    $cloneOutput | Write-Host
    $ErrorActionPreference = 'Stop'
}

Remove-Item $TMP_CLONE -Force -ErrorAction SilentlyContinue

# Detect whether the clone script already invoked the IOsonata library builder.
if (Test-Path $CLONE_LOG) {
    $logContent = Get-Content $CLONE_LOG -Raw -ErrorAction SilentlyContinue
    if ($logContent -match "IOsonata Library (Auto-)?Build|IOsonata Library Builder|Available IOsonata library projects|Select project to build") {
        $CLONE_INVOKED_BUILD = $true
    }
    Remove-Item $CLONE_LOG -Force -ErrorAction SilentlyContinue
}

# --- Install IDAP Tools ---
$IDAP_DIR = "$ROOT\IDAP"
$IDAP_PROG = "$IDAP_DIR\IDAPnRFProg.exe"
New-Item -Path $IDAP_DIR -ItemType Directory -Force | Out-Null

if (-not (Test-Path $IDAP_PROG) -or $MODE -eq "force") {
    Write-Host
    Write-Host ">>> Downloading IDAPnRFProg tool..."
    $IDAP_URL = "https://sourceforge.net/projects/idaplinkfirmware/files/Windows/IDAPnRFProg_Win_2_1_240807.zip/download"
    $IDAP_TMP = Join-Path $env:TEMP "IDAPnRFProg.zip"
    
    $curlExitCode = 0
    try {
        $ErrorActionPreference = 'Continue'
        curl.exe -fL $IDAP_URL -o $IDAP_TMP --silent --show-error
        $curlExitCode = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
    } catch {
        $curlExitCode = 1
        $ErrorActionPreference = 'Stop'
    }
    
    if ($curlExitCode -eq 0 -and (Test-Path $IDAP_TMP)) {
        & $SEVENZIP_EXE e $IDAP_TMP -o"$IDAP_DIR" -y | Out-Null
        Remove-Item $IDAP_TMP -Force -ErrorAction SilentlyContinue
        Write-Host "   [OK] IDAPnRFProg installed at: $IDAP_PROG" -ForegroundColor Green
    } else {
        Write-Host "   [WARN] Failed to download IDAPnRFProg. You can download manually from:" -ForegroundColor Yellow
        Write-Host "         https://sourceforge.net/projects/idaplinkfirmware/files/Windows/" -ForegroundColor Yellow
        Remove-Item $IDAP_TMP -Force -ErrorAction SilentlyContinue
    }
} else {
    Write-Host "   [OK] IDAPnRFProg already installed (skipping)" -ForegroundColor Green
}

Install-Plugin

# --- Makefile (aligned with macOS/Linux) ---
Write-Host; Write-Host ">>> Generating makefile_path.mk..." -ForegroundColor Cyan
$MK = "$ROOT\IOsonata\makefile_path.mk"
$MK_CONTENT = @"
# makefile_path.mk
# Auto-generated by install_iocdevtools_win.ps1 $SCRIPT_VERSION
# This file contains all path macros required to compile IOsonata projects using Makefiles
# Include this file in your project Makefile: include `$(IOSONATA_ROOT)/makefile_path.mk

# ============================================
# Toolchain Paths
# ============================================
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

# ============================================
# IOsonata Paths
# ============================================
# IOCOMPOSER_HOME must be set to your IOcomposer root directory
ifndef IOCOMPOSER_HOME
`$(error IOCOMPOSER_HOME is not set. Please set it to your IOcomposer root directory)
endif

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
VQF_ROOT = `$(EXTERNAL_ROOT)/vqf
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
export ARM_GCC_HOME := $AD/bin
export RISCV_GCC_HOME := $RD/bin
export OPENOCD_HOME := $OD/bin
export NRFX_HOME := `$(NRFX_ROOT)
export NRFXLIB_HOME := `$(SDK_NRFXLIB_ROOT)
export NRF5_SDK_HOME := `$(NRF5_SDK_ROOT)
export NRF5_SDK_MESH_HOME := `$(NRF5_SDK_MESH_ROOT)
export BSEC_HOME := `$(BSEC_ROOT)
"@
Set-Content $MK $MK_CONTENT -Encoding UTF8
Write-Host "   [OK] makefile_path.mk generated." -ForegroundColor Green
Write-Host "   Projects can include it with: include `$(IOSONATA_ROOT)/makefile_path.mk" -ForegroundColor Green

# ---------------------------------------------------------
# IOsonata Library Build (run exactly once)
# ---------------------------------------------------------
# If the clone script already invoked the builder, do NOT run it again.
# If it did NOT invoke the builder (e.g., clone supports -NoBuild), run it here.
if (-not $CLONE_INVOKED_BUILD) {
    $BS = "$ROOT\IOsonata\Installer\build_iosonata_lib_win.ps1"
    if (Test-Path $BS) {
        Write-Host
        Write-Host "=========================================================" -ForegroundColor Cyan
        Write-Host "  IOsonata Library Build" -ForegroundColor Cyan
        Write-Host "=========================================================" -ForegroundColor Cyan
        Write-Host
        & $BS -SdkHome $ROOT
    } else { 
        Write-Host
        Write-Host "   [INFO] IOsonata library builder not found:" -ForegroundColor Yellow
        Write-Host "   $BS" -ForegroundColor Yellow
        Write-Host "   (Skipping library build)" -ForegroundColor Yellow
        Write-Host
    }
} else {
    Write-Host
    Write-Host "   [INFO] IOsonata libraries were already handled by the clone step (skipping rebuild)." -ForegroundColor Yellow
    Write-Host
}

# --- Summary ---
Write-Host
Write-Host "==============================================" -ForegroundColor Green
Write-Host " IOcomposer MCU Dev Tools Installation Summary" -ForegroundColor Green
Write-Host "==============================================" -ForegroundColor Green

$ECLIPSE_VER = "Not installed"
$eclipseProduct = "$ECLIPSE_DIR\.eclipseproduct"
if (Test-Path $eclipseProduct) {
    $prodContent = Get-Content $eclipseProduct -Raw
    if ($prodContent -match 'version=(.+)') { $ECLIPSE_VER = $Matches[1].Trim() }
}

$ARM_VER = "Not found"
if ($ARM_DIR -and (Test-Path "$ARM_DIR\bin\arm-none-eabi-gcc.exe")) {
    try {
        $verOut = (& "$ARM_DIR\bin\arm-none-eabi-gcc.exe" --version 2>&1) | Out-String -Stream | Select-Object -First 1
        if ($verOut -match '(\d+\.\d+\.\d+)') { $ARM_VER = $Matches[1] }
    } catch {
        $ARM_VER = "Installed"
    }
}

$RISCV_VER = "Not found"
if ($RISCV_DIR -and (Test-Path "$RISCV_DIR\bin\riscv-none-elf-gcc.exe")) {
    try {
        $verOut = (& "$RISCV_DIR\bin\riscv-none-elf-gcc.exe" --version 2>&1) | Out-String -Stream | Select-Object -First 1
        if ($verOut -match '(\d+\.\d+\.\d+)') { $RISCV_VER = $Matches[1] }
    } catch {
        $RISCV_VER = "Installed"
    }
}

$OPENOCD_VER = "Not found"
if ($OPENOCD_DIR -and (Test-Path "$OPENOCD_DIR\bin\openocd.exe")) {
    try {
        $verOut = (& "$OPENOCD_DIR\bin\openocd.exe" --version 2>&1) | Out-String -Stream | Select-Object -First 1
        if ($verOut -match '(\d+\.\d+\.\d+)') { $OPENOCD_VER = $Matches[1] }
    } catch {
        $OPENOCD_VER = "Installed"
    }
}

$OPENOCD_VER = "Not found"
if ($OPENOCD_DIR -and (Test-Path "$OPENOCD_DIR\bin\openocd.exe")) {
    try {
        $verOut = (& "$OPENOCD_DIR\bin\openocd.exe" --version 2>&1) | Out-String -Stream | Select-Object -First 1
        if ($verOut -match '(\d+\.\d+\.\d+)') { $OPENOCD_VER = $Matches[1] }
    } catch {
        $OPENOCD_VER = "Installed"
    }
}

Write-Host ("{0,-25} {1}" -f "Eclipse Embedded CDT:", $ECLIPSE_VER)
Write-Host ("{0,-25} {1}" -f "ARM GCC:", $ARM_VER)
Write-Host ("{0,-25} {1}" -f "RISC-V GCC:", $RISCV_VER)
Write-Host ("{0,-25} {1}" -f "OpenOCD:", $OPENOCD_VER)
Write-Host
Write-Host ("{0,-25} {1}" -f "iocomposer_home:", $RT)
Write-Host ("{0,-25} {1}" -f "iosonata_loc:", $RT)

Write-Host "==============================================" -ForegroundColor Green
Write-Host " Installation complete!" -ForegroundColor Green
Write-Host "==============================================" -ForegroundColor Green
Write-Host
Write-Host "Usage in .cproject files:"
Write-Host "  `${system_property:iosonata_loc}/IOsonata/include"
Write-Host "  `${system_property:iosonata_loc}/IOsonata/ARM/include"
Write-Host "  `${system_property:iosonata_loc}/IOsonata/ARM/CMSIS/Include"
Write-Host